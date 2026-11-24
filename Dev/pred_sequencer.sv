// Execute the two streaming prediction passes using the UKF arithmetic ports.
module UKF_pred_sequencer #(
    parameter bit SRAM_STORAGE = 1'b0,
    parameter bit USE_SKY130 = 1'b0,
    parameter logic [160:0] INITIAL_STATE = '0,
    parameter logic [479:0] INITIAL_S = '0,
    parameter logic [31:0] ORIGIN_LONGITUDE = '0, ORIGIN_LATITUDE = '0,
    parameter logic [31:0] DT = 32'h3b23d70a, // 1/400 s
    parameter logic [31:0] SIGMA_SCALE = 32'h3f8f1bbd, // sqrt(1.25)
    parameter logic [31:0] W_NONCENTRAL = 32'h3ecccccd, // 0.4
    parameter logic [31:0] SQRT_W_NONCENTRAL = 32'h3f21e89b,
    parameter logic [31:0] W_MEAN_CENTRAL = 32'hc0400000, // -3
    parameter logic [31:0] SQRT_ABS_W_CENTRAL = 32'h3f000000,
    parameter bit CENTRAL_DOWNDATE = 1'b1,
    parameter logic [479:0] PROCESS_S = {
        32'h3c23d70a, 32'b0, 32'h3c23d70a, 64'b0, 32'h3c23d70a,
        96'b0, 32'h3c23d70a, 128'b0, 32'h3c23d70a
    }
) (
    input logic clock, reset, start, wheel_valid,
    // The UKF holds the prior and wheel inputs through the done edge.
    input UKF_OUT_PACKET prior,
    input pred_s_factor_t prior_s,
    input logic [31:0] wheel_velocity,
    input logic load_correction_noise,
    input meas_s_factor_t staged_correction_noise,
    output meas_s_factor_t correction_noise,
    input logic load_correction_measurement,
    input corr_measurement_t correction_measurement,
    input ukf_spad_request_t correction_spad_request,
    output ukf_spad_response_t correction_spad_response,
    input logic converter_owner, client_read,
    output logic client_read_valid, initializing,
    input logic mac_grant, div_grant,
    input ukf_mac_response_t mac_response,
    input ukf_divsqrt_response_t div_response,
    output ukf_mac_request_t mac_request,
    output ukf_divsqrt_request_t div_request,
    output logic done,
    output PRED_PACKET prediction,
    output pred_s_factor_t prediction_s
`ifdef USE_POWER_PINS
    , inout wire vccd1, vssd1
`endif
);
    localparam bit DT_VALID = !DT[31] && DT[30:23] > 8'd1 && DT[30:23] < 8'hff;
    typedef enum logic [3:0] {
        IDLE, ISSUE, WAIT_MAC, ISSUE_ADD, WAIT_ADD, WAIT_DIV, FINISH,
        INITIALIZE, LOAD_PROCESS, LOAD_MEASUREMENT, LOAD_NOISE,
        FETCH, LATCH, COLLECT, COMMIT
    } state_t;
    state_t state;
    logic [8:0] pc, return_pc [0:2];
    logic [1:0] depth;
    logic success;
    pred_ukf_uop uop;
    localparam logic [6:0] SPAD_FIRST = 7'd21, SPAD_LAST = 7'd36;
    localparam int ANGLE_INDEX = int'(PRED_ANGLE_ARG) - int'(SPAD_FIRST);
    logic [31:0] spad [0:15];
    logic [31:0] prior_data [0:20];
    logic [3:0] spad_addr_a, spad_addr_b;
    logic [31:0] working_sq [0:14], collected_sq [0:14];
    // Use explicit word slices across packed-array module ports.
    wire [479:0] prior_words = prior_s;
    wire [319:0] staged_noise_words = staged_correction_noise;
    wire [479:0] prediction_words;
    wire [319:0] correction_noise_words;
    assign prediction_s = prediction_words;
    assign correction_noise = SRAM_STORAGE ? '0 : correction_noise_words;

    logic [31:0] a, b, signed_b, result_word;
    logic complete, write_result, div_finished, bad_result, bad_control;
    logic [4:0] transfer_index;
    corr_measurement_t held_measurement;
    wire [127:0] measurement_words = held_measurement;
    logic heading_negative;
    logic [31:0] held_a, held_b, memory_a, memory_b;
    logic memory_valid;

    assign prior_data[0] = prior.ukf_x;
    assign prior_data[1] = prior.ukf_y;
    assign prior_data[2] = prior.ukf_theta;
    assign prior_data[3] = prior.ukf_omega;
    assign prior_data[4] = prior.ukf_wheelB;
    assign prior_data[5] = wheel_velocity;
    for (genvar i = 0; i < 15; i++)
        assign prior_data[6+i] = prior_words[32*i +: 32];
    pred_microcode_prog rom (.pc(pc), .central_downdate(CENTRAL_DOWNDATE),
                            .wheel_valid(wheel_valid), .uop(uop));

    function automatic logic [31:0] read_word(input logic [6:0] addr);
        case (addr)
            7'd64: read_word = SIGMA_SCALE;
            7'd65: read_word = W_NONCENTRAL;
            7'd66: read_word = W_MEAN_CENTRAL;
            7'd67: read_word = SQRT_ABS_W_CENTRAL;
            7'd83: read_word = SQRT_W_NONCENTRAL;
            7'd95: read_word = DT;
            // Normal positive sample intervals are required. Scaling by
            // one half is exact for normal inputs above the minimum exponent.
            7'd96: read_word = {1'b0, DT[30:23] - 8'd1, DT[22:0]};
            default: read_word = prom_math_read(addr);
        endcase
    endfunction

    function automatic logic [31:0] operand(
        input logic [6:0] addr, input logic [31:0] prior_word, scratch_word, sq_word, wrap_word
    );
        // Correction uses the same read ports with its own noise address
        // range. Prediction bank ranges and ROM addresses are disjoint.
        if (correction_spad_request.active)
            operand = addr < SPAD_FIRST ? prior_word :
                      addr <= SPAD_LAST ? scratch_word :
                      addr >= PROM_SR_BASE && addr < PROM_SR_BASE + 7'd10 ? sq_word : prom_math_read(addr);
        else
            operand = (prior_word & {32{addr < SPAD_FIRST}}) |
                  (scratch_word & {32{addr >= SPAD_FIRST && addr <= SPAD_LAST}}) |
                  (sq_word & {32{addr >= 68 && addr < 83}}) |
                  (angle_wrap_step(wrap_word) & {32{addr == PROM_WRAP_STEP}}) |
                  read_word(addr);
    endfunction

    assign spad_addr_a = correction_spad_request.active ? correction_spad_request.read_a :
                         uop.src_a[3:0] - SPAD_FIRST[3:0];
    assign spad_addr_b = correction_spad_request.active ? correction_spad_request.read_b :
                         uop.src_b[3:0] - SPAD_FIRST[3:0];
    wire corr_active = correction_spad_request.active;
    wire [6:0] corr_addr_a = correction_spad_request.operand_a;
    wire [6:0] corr_addr_b = correction_spad_request.operand_b;
    wire [6:0] addr_a = corr_active ? corr_addr_a : uop.src_a;
    wire [6:0] addr_b = corr_active ? corr_addr_b : uop.src_b;
    wire [3:0] noise_addr_a = corr_active ? corr_addr_a[3:0] - PROM_SR_BASE[3:0] : uop.src_a[3:0] - 4'd4;
    wire [3:0] noise_addr_b = corr_active ? corr_addr_b[3:0] - PROM_SR_BASE[3:0] : uop.src_b[3:0] - 4'd4;
    if (SRAM_STORAGE) begin : sram
        logic read_enable, write_enable;
        logic [7:0] read_addr_a, read_addr_b, write_addr;
        logic [31:0] write_data;
        wire [6:0] pred_src_a = uop.src_a == PROM_WRAP_STEP ? PRED_ANGLE_ARG : uop.src_a;
        wire [6:0] pred_src_b = uop.src_b == PROM_WRAP_STEP ? PRED_ANGLE_ARG : uop.src_b;

        // Translate each client's logical noise and origin addresses.
        function automatic logic [7:0] physical_addr(input logic [6:0] addr);
            if (converter_owner && (addr == 70 || addr == 71))
                physical_addr = 8'd52 + {7'b0, addr[0]};
            else if (!converter_owner && corr_active &&
                     addr >= PROM_SR_BASE && addr < PROM_SR_BASE + 7'd10)
                physical_addr = 8'd37 + {1'b0, addr} - {1'b0, PROM_SR_BASE};
            else if (!corr_active && addr >= 68 && addr < 83)
                physical_addr = 8'd37 + {1'b0, addr} - 8'd68;
            else
                physical_addr = {1'b0, addr};
        endfunction

        function automatic logic [7:0] main_addr(input logic [4:0] index);
            main_addr = {3'b0, index} + (index >= 5 ? 8'd1 : 8'd0);
        endfunction

        assign initializing = state == INITIALIZE;
        assign client_read_valid = state == IDLE && memory_valid;
        assign correction_spad_response = {
            (converter_owner || corr_addr_a <= SPAD_LAST ||
             (corr_addr_a >= PROM_SR_BASE && corr_addr_a < PROM_SR_BASE + 7'd10))
                ? memory_a : prom_math_read(corr_addr_a),
            (corr_addr_b <= SPAD_LAST ||
             (corr_addr_b >= PROM_SR_BASE && corr_addr_b < PROM_SR_BASE + 7'd10))
                ? memory_b : prom_math_read(corr_addr_b),
            heading_negative
        };
        assign a = held_a;
        assign b = held_b;

        always_comb begin
            read_enable = 1'b0;
            write_enable = 1'b0;
            read_addr_a = physical_addr(pred_src_a);
            read_addr_b = physical_addr(pred_src_b);
            write_addr = physical_addr(uop.dest);
            write_data = result_word;
            case (state)
                INITIALIZE: begin
                    write_enable = 1'b1;
                    write_addr = main_addr(transfer_index);
                    if (transfer_index < 5)
                        write_data = INITIAL_STATE[160-32*transfer_index -: 32];
                    else if (transfer_index < 20)
                        write_data = INITIAL_S[32*(transfer_index-5) +: 32];
                    else begin
                        write_addr = transfer_index == 20 ? 8'd52 : 8'd53;
                        write_data = transfer_index == 20 ? ORIGIN_LONGITUDE : ORIGIN_LATITUDE;
                    end
                end
                LOAD_PROCESS: begin
                    write_enable = 1'b1;
                    write_addr = 8'd37 + {3'b0, transfer_index};
                    write_data = PROCESS_S[32*transfer_index +: 32];
                    if (transfer_index == 15) begin
                        write_addr = 8'd5;
                        write_data = wheel_velocity;
                    end
                end
                LOAD_NOISE: begin
                    write_enable = 1'b1;
                    write_addr = 8'd37 + {3'b0, transfer_index};
                    write_data = staged_noise_words[32*transfer_index +: 32];
                end
                LOAD_MEASUREMENT: begin
                    write_enable = 1'b1;
                    write_addr = 8'd21 + {3'b0, transfer_index};
                    write_data = measurement_words[127-32*transfer_index -: 32];
                end
                FETCH: read_enable = 1'b1;
                COLLECT: begin
                    read_enable = 1'b1;
                    read_addr_a = transfer_index < 5 ? 8'd28 + {3'b0, transfer_index} :
                                                        8'd32 + {3'b0, transfer_index};
                end
                COMMIT: begin
                    write_enable = memory_valid;
                    write_addr = main_addr(transfer_index);
                    write_data = memory_a;
                end
                IDLE: if (corr_active && !start && !load_correction_noise) begin
                    read_enable = client_read;
                    read_addr_a = physical_addr(corr_addr_a);
                    read_addr_b = physical_addr(corr_addr_b);
                    write_enable = correction_spad_request.write_enable;
                    write_addr = physical_addr(corr_addr_a);
                    write_data = correction_spad_request.write_data;
                end
                default: write_enable = write_result && !bad_control && !bad_result &&
                    ((uop.dest >= SPAD_FIRST && uop.dest <= SPAD_LAST) ||
                     (uop.dest >= 68 && uop.dest < 83));
            endcase
        end

        ukf_regfile #(.USE_SKY130(USE_SKY130)) bank (
            .clock(clock), .reset(reset), .read_enable(read_enable),
            .read_addr_a(read_addr_a), .read_addr_b(read_addr_b),
            .write_enable(write_enable), .write_addr(write_addr), .write_data(write_data),
            .read_valid(memory_valid), .read_data_a(memory_a), .read_data_b(memory_b)
`ifdef USE_POWER_PINS
            , .vccd1(vccd1), .vssd1(vssd1)
`endif
        );
    end else begin : parallel_storage
        assign initializing = 1'b0;
        assign client_read_valid = 1'b1;
        assign correction_spad_response = {a, b, spad[2][31]};
        assign a = operand(addr_a, prior_data[addr_a[4:0]], spad[spad_addr_a],
                           working_sq[noise_addr_a], spad[ANGLE_INDEX]);
        assign b = operand(addr_b, prior_data[addr_b[4:0]], spad[spad_addr_b],
                           working_sq[noise_addr_b], spad[ANGLE_INDEX]);
    end
    assign signed_b = {b[31] ^ uop.neg_b, b[30:0]};

    always_comb begin
        mac_request = '0;
        div_request = '0;
        if (state == ISSUE && mac_grant && !mac_response.busy) begin
            if (uop.dp == UOP_MAC)
                mac_request = {1'b1, uop.clr_acc, uop.wb_en, a, signed_b};
            else if (uop.dp == UOP_ADD)
                mac_request = {1'b1, 1'b1, 1'b0, a, 32'h3f800000};
        end
        if (state == ISSUE_ADD && mac_grant && !mac_response.busy)
            mac_request = {1'b1, 1'b0, 1'b1, signed_b, 32'h3f800000};
        if (state == ISSUE && div_grant && !div_response.busy &&
            (uop.dp == UOP_DIV || uop.dp == UOP_SQRT))
            div_request = {1'b1, uop.dp == UOP_SQRT, a, signed_b};
    end

    assign div_finished = state == WAIT_DIV && div_response.done;
    assign complete = div_finished || (state == WAIT_ADD && mac_response.valid) ||
                      (state == WAIT_MAC && uop.dp == UOP_MAC &&
                       (uop.wb_en ? mac_response.valid : !mac_response.busy)) ||
                      (state == ISSUE && uop.dp == UOP_NOP);
    assign write_result = complete && uop.wb_en;
    assign result_word = div_finished ? div_response.result : mac_response.result;
    assign bad_result = (write_result && result_word[30:23] == 8'hff) ||
                        (div_finished && (div_response.invalid ||
                         div_response.divide_by_zero || div_response.overflow ||
                         (uop.dp == UOP_SQRT && !(|result_word[30:0]))));
    assign bad_control = (uop.seq == SEQ_CALL && depth == 3) ||
                         (uop.seq == SEQ_RET && depth == 0) ||
                         (uop.seq == SEQ_ANGLE_CHECK && !angle_supported(a));

    assign done = state == FINISH;
    assign prediction = {spad[7], spad[8], spad[9], spad[10], spad[11], done && success};
    for (genvar i = 0; i < 15; i++)
        assign prediction_words[32*i +: 32] = SRAM_STORAGE ? collected_sq[i] : working_sq[i];
    for (genvar i = 0; i < 10; i++)
        assign correction_noise_words[32*i +: 32] = working_sq[i];

    // The legacy path uses parallel scratch access. SRAM mode fills only
    // the prediction output snapshot from the serial commit.
    wire spad_write_pred = complete && !bad_control && !bad_result && uop.wb_en &&
                           uop.dest >= SPAD_FIRST && uop.dest <= SPAD_LAST;
    wire spad_write_corr = state == IDLE && !start && !load_correction_noise &&
                           correction_spad_request.write_enable;
    wire [3:0] spad_write_addr = spad_write_corr ? correction_spad_request.write_addr :
                                 uop.dest[3:0] - SPAD_FIRST[3:0];
    wire [31:0] spad_write_data = spad_write_corr ? correction_spad_request.write_data : result_word;
    if (SRAM_STORAGE) begin : collect_prediction
        always_ff @(posedge clock)
            if (state == COMMIT && memory_valid) begin
                if (transfer_index < 5)
                    spad[7+transfer_index] <= memory_a;
                else
                    collected_sq[transfer_index-5] <= memory_a;
            end
    end else begin : parallel_scratch
        always_ff @(posedge clock) begin
            if (!reset && state == IDLE && !start && !load_correction_noise && load_correction_measurement) begin
                spad[0] <= correction_measurement.x;
                spad[1] <= correction_measurement.y;
                spad[2] <= correction_measurement.theta;
                spad[3] <= correction_measurement.omega;
            end else if (!reset && (spad_write_pred || spad_write_corr))
                spad[spad_write_addr] <= spad_write_data;
        end

    end

    function automatic logic [31:0] sram_operand(input logic [6:0] addr, input logic [31:0] word);
        if (addr <= SPAD_LAST || (addr >= 68 && addr < 83))
            sram_operand = word;
        else if (addr == PROM_WRAP_STEP)
            sram_operand = angle_wrap_step(word);
        else
            sram_operand = read_word(addr);
    endfunction

    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            pc <= '0;
            depth <= '0;
            return_pc[0] <= '0;
            return_pc[1] <= '0;
            return_pc[2] <= '0;
            success <= 1'b0;
            transfer_index <= '0;
            heading_negative <= 1'b0;
            if (SRAM_STORAGE)
                state <= INITIALIZE;
            else
                state <= IDLE;
        end else if (SRAM_STORAGE && state == INITIALIZE) begin
            transfer_index <= transfer_index + 1'b1;
            if (transfer_index == 21)
                state <= IDLE;
        end else if (SRAM_STORAGE && state == LOAD_PROCESS) begin
            transfer_index <= transfer_index + 1'b1;
            if (transfer_index == 15)
                state <= FETCH;
        end else if (SRAM_STORAGE && (state == LOAD_MEASUREMENT || state == LOAD_NOISE)) begin
            transfer_index <= transfer_index + 1'b1;
            if (state == LOAD_MEASUREMENT && transfer_index == 3) begin
                transfer_index <= '0;
                state <= LOAD_NOISE;
            end else if (state == LOAD_NOISE && transfer_index == 9)
                state <= IDLE;
        end else if (SRAM_STORAGE && state == FETCH) begin
            state <= LATCH;
        end else if (SRAM_STORAGE && state == LATCH) begin
            if (memory_valid) begin
                held_a <= sram_operand(uop.src_a, memory_a);
                held_b <= sram_operand(uop.src_b, memory_b);
                state <= ISSUE;
            end
        end else if (SRAM_STORAGE && state == COLLECT) begin
            state <= COMMIT;
        end else if (SRAM_STORAGE && state == COMMIT) begin
            if (memory_valid) begin
                transfer_index <= transfer_index + 1'b1;
                if (transfer_index == 19)
                    state <= FINISH;
                else
                    state <= COLLECT;
            end
        end else if (state == IDLE) begin
            if (start) begin
                if (!SRAM_STORAGE)
                    for (int i = 0; i < 15; i++)
                        working_sq[i] <= PROCESS_S[32*i +: 32];
                pc <= '0;
                depth <= '0;
                success <= 1'b0;
                if (!DT_VALID)
                    state <= FINISH;
                else if (SRAM_STORAGE) begin
                    transfer_index <= '0;
                    state <= LOAD_PROCESS;
                end else
                    state <= ISSUE;
            end else if (SRAM_STORAGE && load_correction_measurement) begin
                transfer_index <= '0;
                held_measurement <= correction_measurement;
                heading_negative <= correction_measurement.theta[31];
                state <= LOAD_MEASUREMENT;
            end else if (SRAM_STORAGE && load_correction_noise) begin
                transfer_index <= '0;
                state <= LOAD_NOISE;
            end else if (!SRAM_STORAGE && load_correction_noise) begin
                // Prediction has already committed SQ into retained S.
                for (int i = 0; i < 10; i++)
                    working_sq[i] <= staged_noise_words[32*i +: 32];
            end else if (SRAM_STORAGE && !converter_owner &&
                         correction_spad_request.write_enable && corr_addr_a == 7'd23)
                heading_negative <= correction_spad_request.write_data[31];
        end else if (state == FINISH) begin
            state <= IDLE;
        end else if (state == ISSUE && uop.seq == SEQ_HALT) begin
            success <= uop.dp == UOP_DONE;
            if (SRAM_STORAGE && uop.dp == UOP_DONE) begin
                transfer_index <= '0;
                state <= COLLECT;
            end else
                state <= FINISH;
        end else if (complete) begin
            if (bad_control || bad_result) begin
                state <= FINISH;
            end else begin
                if (!SRAM_STORAGE && write_result) begin
                    if (uop.dest >= 68 && uop.dest < 83)
                        working_sq[uop.dest[3:0] - 4'd4] <= result_word;
                end
                if (SRAM_STORAGE)
                    state <= FETCH;
                else
                    state <= ISSUE;
                pc <= pc + 9'd1;
                case (uop.seq)
                    SEQ_CALL: begin
                        return_pc[depth] <= pc + 9'd1;
                        depth <= depth + 2'd1;
                        pc <= uop.targ_pc;
                    end
                    SEQ_RET: begin
                        pc <= return_pc[depth - 2'd1];
                        depth <= depth - 2'd1;
                    end
                    SEQ_JUMP: pc <= uop.targ_pc;
                    SEQ_FP_LT: if (fp32_lt(a, b)) pc <= uop.targ_pc;
                    default: begin end
                endcase
            end
        end else if (mac_request.start) begin
            state <= state == ISSUE_ADD ? WAIT_ADD : WAIT_MAC;
        end else if (div_request.start) begin
            state <= WAIT_DIV;
        end else if (state == WAIT_MAC && uop.dp == UOP_ADD && !mac_response.busy) begin
            state <= ISSUE_ADD;
        end
    end
endmodule
