module UKF_pred_sequencer #(
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
    input ukf_defs::UKF_OUT_PACKET prior,
    input ukf_defs::pred_s_factor_t prior_s,
    input logic [31:0] wheel_velocity,
    input logic load_correction_noise,
    input ukf_defs::meas_s_factor_t staged_correction_noise,
    output ukf_defs::meas_s_factor_t correction_noise,
    input logic load_correction_measurement,
    input ukf_defs::corr_measurement_t correction_measurement,
    input ukf_defs::ukf_spad_request_t correction_spad_request,
    output ukf_defs::ukf_spad_response_t correction_spad_response,
    input logic mac_grant, div_grant,
    input ukf_defs::ukf_mac_response_t mac_response,
    input ukf_defs::ukf_divsqrt_response_t div_response,
    output ukf_defs::ukf_mac_request_t mac_request,
    output ukf_defs::ukf_divsqrt_request_t div_request,
    output logic done,
    output ukf_defs::PRED_PACKET prediction,
    output ukf_defs::pred_s_factor_t prediction_s
);
    localparam bit DT_VALID = !DT[31] && DT[30:23] > 8'd1 && DT[30:23] < 8'hff;
    typedef enum logic [2:0] {IDLE, ISSUE, WAIT_MAC, ISSUE_ADD, WAIT_ADD, WAIT_DIV, FINISH} state_t;
    state_t state;
    logic [8:0] pc, return_pc [0:2];
    logic [1:0] depth;
    logic success;
    ukf_defs::pred_ukf_uop uop;
    localparam logic [6:0] SPAD_FIRST = 7'd21, SPAD_LAST = 7'd36;
    localparam int ANGLE_INDEX = ukf_defs::PRED_ANGLE_ARG - SPAD_FIRST;
    logic [31:0] spad [0:15];
    logic [31:0] prior_data [0:20];
    logic [3:0] spad_addr_a, spad_addr_b;
    logic [6:0] sq_addr_a, sq_addr_b, sq_dest;
    logic [31:0] working_sq [0:14];

    logic [31:0] a, b, signed_b, result_word;
    logic complete, write_result, div_finished, bad_result, bad_control;

    assign prior_data[0] = prior.ukf_x;
    assign prior_data[1] = prior.ukf_y;
    assign prior_data[2] = prior.ukf_theta;
    assign prior_data[3] = prior.ukf_omega;
    assign prior_data[4] = prior.ukf_wheelB;
    assign prior_data[5] = wheel_velocity;
    for (genvar i = 0; i < 15; i++)
        assign prior_data[6+i] = prior_s[i];
    pred_microcode_prog rom (
        .pc(pc), .central_downdate(CENTRAL_DOWNDATE), .wheel_valid(wheel_valid), .uop(uop)
    );

    function automatic logic [31:0] read_word(input logic [6:0] addr);
        case (addr)
            7'd37: read_word = SQRT_W_NONCENTRAL;
            7'd38: read_word = SIGMA_SCALE;
            7'd39: read_word = W_NONCENTRAL;
            7'd40: read_word = W_MEAN_CENTRAL;
            7'd41: read_word = SQRT_ABS_W_CENTRAL;
            ukf_defs::PROM_MATH_BASE + 7'd2: read_word = DT;
            // Normal positive sample intervals are required. Scaling by
            // one half is exact for normal inputs above the minimum exponent.
            ukf_defs::PROM_MATH_BASE + 7'd3: read_word = {1'b0, DT[30:23] - 8'd1, DT[22:0]};
            default: read_word = ukf_defs::prom_math_read(addr);
        endcase
    endfunction

    function automatic logic [31:0] operand(
        input logic [6:0] addr, input logic [31:0] prior_word, scratch_word, sq_word, wrap_word
    );
        if (addr < SPAD_FIRST) operand = prior_word;
        else if (addr >= SPAD_FIRST && addr <= SPAD_LAST) operand = scratch_word;
        else if (addr >= 7'd42 && addr < 7'd57) operand = sq_word;
        else if (addr == ukf_defs::PROM_WRAP_STEP) operand = ukf_defs::angle_wrap_step(wrap_word);
        else operand = read_word(addr);
    endfunction

    assign spad_addr_a = correction_spad_request.active ? correction_spad_request.read_a :
                         uop.src_a[3:0] - SPAD_FIRST[3:0];
    assign spad_addr_b = correction_spad_request.active ? correction_spad_request.read_b :
                         uop.src_b[3:0] - SPAD_FIRST[3:0];
    assign correction_spad_response = {spad[spad_addr_a], spad[spad_addr_b], spad[2][31]};
    assign sq_addr_a = uop.src_a - 7'd42;
    assign sq_addr_b = uop.src_b - 7'd42;
    assign sq_dest = uop.dest - 7'd42;
    assign a = operand(uop.src_a, prior_data[uop.src_a[4:0]],
                       spad[spad_addr_a],
                       working_sq[sq_addr_a[3:0]], spad[ANGLE_INDEX]);
    assign b = operand(uop.src_b, prior_data[uop.src_b[4:0]],
                       spad[spad_addr_b],
                       working_sq[sq_addr_b[3:0]], spad[ANGLE_INDEX]);
    assign signed_b = {b[31] ^ uop.neg_b, b[30:0]};

    always_comb begin
        mac_request = '0;
        div_request = '0;
        if (state == ISSUE && mac_grant && !mac_response.busy) begin
            if (uop.dp == ukf_defs::UOP_MAC)
                mac_request = {1'b1, uop.clr_acc, uop.wb_en, a, signed_b};
            else if (uop.dp == ukf_defs::UOP_ADD)
                mac_request = {1'b1, 1'b1, 1'b0, a, 32'h3f800000};
        end
        if (state == ISSUE_ADD && mac_grant && !mac_response.busy)
            mac_request = {1'b1, 1'b0, 1'b1, signed_b, 32'h3f800000};
        if (state == ISSUE && div_grant && !div_response.busy &&
            (uop.dp == ukf_defs::UOP_DIV || uop.dp == ukf_defs::UOP_SQRT))
            div_request = {1'b1, uop.dp == ukf_defs::UOP_SQRT, a, signed_b};
    end

    assign div_finished = state == WAIT_DIV && div_response.done;
    assign complete = div_finished || (state == WAIT_ADD && mac_response.valid) ||
                      (state == WAIT_MAC && uop.dp == ukf_defs::UOP_MAC &&
                       (uop.wb_en ? mac_response.valid : !mac_response.busy)) ||
                      (state == ISSUE && uop.dp == ukf_defs::UOP_NOP);
    assign write_result = complete && uop.wb_en;
    assign result_word = div_finished ? div_response.result : mac_response.result;
    assign bad_result = (write_result && result_word[30:23] == 8'hff) ||
                        (div_finished && (div_response.invalid ||
                         div_response.divide_by_zero || div_response.overflow ||
                         (uop.dp == ukf_defs::UOP_SQRT && !(|result_word[30:0]))));
    assign bad_control = (uop.seq == ukf_defs::SEQ_CALL && depth == 3) ||
                         (uop.seq == ukf_defs::SEQ_RET && depth == 0) ||
                         (uop.seq == ukf_defs::SEQ_ANGLE_CHECK && !ukf_defs::angle_supported(a));

    assign done = state == FINISH;
    assign prediction = {spad[7], spad[8], spad[9], spad[10], spad[11], done && success};
    assign prediction_s = {
        working_sq[14], working_sq[13], working_sq[12], working_sq[11], working_sq[10],
        working_sq[9],  working_sq[8],  working_sq[7],  working_sq[6],  working_sq[5],
        working_sq[4],  working_sq[3],  working_sq[2],  working_sq[1],  working_sq[0]
    };
    assign correction_noise = {
        working_sq[9], working_sq[8], working_sq[7], working_sq[6], working_sq[5],
        working_sq[4], working_sq[3], working_sq[2], working_sq[1], working_sq[0]
    };

    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            pc <= '0;
            depth <= '0;
            return_pc[0] <= '0;
            return_pc[1] <= '0;
            return_pc[2] <= '0;
            success <= 1'b0;
            state <= IDLE;
        end else if (state == IDLE) begin
            if (start) begin
                for (int i = 0; i < 15; i++)
                    working_sq[i] <= PROCESS_S[32*i +: 32];
                pc <= '0;
                depth <= '0;
                success <= 1'b0;
                state <= DT_VALID ? ISSUE : FINISH;
            end else if (load_correction_noise) begin
                // Prediction has already committed SQ into retained S.
                for (int i = 0; i < 10; i++)
                    working_sq[i] <= staged_correction_noise[i];
            end
            if (!start && load_correction_measurement) begin
                spad[0] <= correction_measurement.x;
                spad[1] <= correction_measurement.y;
                spad[2] <= correction_measurement.theta;
                spad[3] <= correction_measurement.omega;
            end else if (!start && correction_spad_request.write_enable) begin
                spad[correction_spad_request.write_addr] <= correction_spad_request.write_data;
            end
        end else if (state == FINISH) begin
            state <= IDLE;
        end else if (state == ISSUE && uop.seq == ukf_defs::SEQ_HALT) begin
            success <= uop.dp == ukf_defs::UOP_DONE;
            state <= FINISH;
        end else if (complete) begin
            if (bad_control || bad_result) begin
                state <= FINISH;
            end else begin
                if (write_result) begin
                    if (uop.dest >= SPAD_FIRST && uop.dest <= SPAD_LAST)
                        spad[uop.dest[3:0] - SPAD_FIRST[3:0]] <= result_word;
                    else if (uop.dest >= 7'd42 && uop.dest < 7'd57)
                        working_sq[sq_dest[3:0]] <= result_word;
                end
                state <= ISSUE;
                pc <= pc + 9'd1;
                case (uop.seq)
                    ukf_defs::SEQ_CALL: begin
                        return_pc[depth] <= pc + 9'd1;
                        depth <= depth + 2'd1;
                        pc <= uop.targ_pc;
                    end
                    ukf_defs::SEQ_RET: begin
                        pc <= return_pc[depth - 2'd1];
                        depth <= depth - 2'd1;
                    end
                    ukf_defs::SEQ_JUMP: pc <= uop.targ_pc;
                    ukf_defs::SEQ_FP_LT: if (ukf_defs::fp32_lt(a, b)) pc <= uop.targ_pc;
                    default: begin end
                endcase
            end
        end else if (mac_request.start) begin
            if (state == ISSUE_ADD)
                state <= WAIT_ADD;
            else
                state <= WAIT_MAC;
        end else if (div_request.start) begin
            state <= WAIT_DIV;
        end else if (state == WAIT_MAC && uop.dp == ukf_defs::UOP_ADD && !mac_response.busy) begin
            state <= ISSUE_ADD;
        end
    end
endmodule

