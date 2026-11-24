// Execute the two streaming prediction passes using the UKF arithmetic ports.
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
    input logic mac_grant, div_grant,
    input ukf_mac_response_t mac_response,
    input ukf_divsqrt_response_t div_response,
    output ukf_mac_request_t mac_request,
    output ukf_divsqrt_request_t div_request,
    output logic done,
    output PRED_PACKET prediction,
    output pred_s_factor_t prediction_s
);
    localparam bit DT_VALID = !DT[31] && DT[30:23] > 8'd1 && DT[30:23] < 8'hff;
    typedef enum logic [2:0] {IDLE, ISSUE, WAIT_MAC, ISSUE_ADD, WAIT_ADD, WAIT_DIV, FINISH} state_t;
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
    pred_microcode_prog rom (.pc(pc), .central_downdate(CENTRAL_DOWNDATE),
                            .wheel_valid(wheel_valid), .uop(uop));

    function automatic logic [31:0] read_word(input logic [6:0] addr);
        case (addr)
            7'd64: return SIGMA_SCALE;
            7'd65: return W_NONCENTRAL;
            7'd66: return W_MEAN_CENTRAL;
            7'd67: return SQRT_ABS_W_CENTRAL;
            7'd83: return SQRT_W_NONCENTRAL;
            7'd95: return DT;
            // Normal positive sample intervals are required. Scaling by
            // one half is exact for normal inputs above the minimum exponent.
            7'd96: return {1'b0, DT[30:23] - 8'd1, DT[22:0]};
            default: return prom_math_read(addr);
        endcase
    endfunction

    function automatic logic [31:0] operand(
        input logic [6:0] addr, input logic [31:0] prior_word, scratch_word, sq_word, wrap_word
    );
        if (addr < SPAD_FIRST) return prior_word;
        if (addr >= SPAD_FIRST && addr <= SPAD_LAST) return scratch_word;
        if (addr >= 68 && addr < 83) return sq_word;
        if (addr == PROM_WRAP_STEP) return angle_wrap_step(wrap_word);
        return read_word(addr);
    endfunction

    assign spad_addr_a = correction_spad_request.active ? correction_spad_request.read_a :
                         uop.src_a[3:0] - SPAD_FIRST[3:0];
    assign spad_addr_b = correction_spad_request.active ? correction_spad_request.read_b :
                         uop.src_b[3:0] - SPAD_FIRST[3:0];
    assign correction_spad_response = {spad[spad_addr_a], spad[spad_addr_b], spad[2][31]};
    assign a = operand(uop.src_a, prior_data[uop.src_a[4:0]],
                       spad[spad_addr_a],
                       working_sq[uop.src_a[3:0] - 4'd4], spad[ANGLE_INDEX]);
    assign b = operand(uop.src_b, prior_data[uop.src_b[4:0]],
                       spad[spad_addr_b],
                       working_sq[uop.src_b[3:0] - 4'd4], spad[ANGLE_INDEX]);
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
        assign prediction_s[i] = working_sq[i];
    for (genvar i = 0; i < 10; i++)
        assign correction_noise[i] = working_sq[i];

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
            end else if (load_correction_measurement) begin
                spad[0] <= correction_measurement.x;
                spad[1] <= correction_measurement.y;
                spad[2] <= correction_measurement.theta;
                spad[3] <= correction_measurement.omega;
            end else if (correction_spad_request.write_enable) begin
                spad[correction_spad_request.write_addr] <= correction_spad_request.write_data;
            end
        end else if (state == FINISH) begin
            state <= IDLE;
        end else if (state == ISSUE && uop.seq == SEQ_HALT) begin
            success <= uop.dp == UOP_DONE;
            state <= FINISH;
        end else if (complete) begin
            if (bad_control || bad_result) begin
                state <= FINISH;
            end else begin
                if (write_result) begin
                    if (uop.dest >= SPAD_FIRST && uop.dest <= SPAD_LAST)
                        spad[uop.dest[3:0] - SPAD_FIRST[3:0]] <= result_word;
                    else if (uop.dest >= 68 && uop.dest < 83)
                        working_sq[uop.dest[3:0] - 4'd4] <= result_word;
                end
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
