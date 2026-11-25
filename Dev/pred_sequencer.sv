// Execute the two streaming prediction passes using the UKF arithmetic ports.
module UKF_pred_sequencer #(
    parameter logic [31:0] DT = 32'h3b23d70a, // 1/400 s
    parameter logic [31:0] SIGMA_SCALE = 32'h3f8f1bbd, // sqrt(1.25)
    parameter logic [31:0] W_NONCENTRAL = 32'h3ecccccd, // 0.4
    parameter logic [31:0] SQRT_W_NONCENTRAL = 32'h3f21e89b,
    parameter logic [31:0] W_MEAN_CENTRAL = 32'hc0400000, // -3
    parameter logic [31:0] SQRT_ABS_W_CENTRAL = 32'h3f000000,
    parameter bit CENTRAL_DOWNDATE = 1'b1
) (
    input logic clock, reset, start, wheel_valid,
    output logic [6:0] read_addr_a, read_addr_b, write_addr,
    output logic write_enable,
    output logic [31:0] write_data,
    input logic [31:0] read_data_a, read_data_b, wrap_word,
    input logic mac_grant, div_grant,
    input ukf_mac_response_t mac_response,
    input ukf_divsqrt_response_t div_response,
    output ukf_mac_request_t mac_request,
    output ukf_divsqrt_request_t div_request,
    output logic done, success
);
    localparam bit DT_VALID = !DT[31] && DT[30:23] > 8'd1 && DT[30:23] < 8'hff;
    typedef enum logic [2:0] {IDLE, ISSUE, WAIT_MAC, ISSUE_ADD, WAIT_ADD, WAIT_DIV, FINISH} state_t;
    state_t state;
    logic [8:0] pc, return_pc [0:2];
    logic [1:0] depth;
    pred_ukf_uop uop;
    logic [31:0] a, b, signed_b, result_word;
    logic complete, write_result, div_finished, bad_result, bad_control;

    pred_microcode_prog rom (.pc(pc), .central_downdate(CENTRAL_DOWNDATE),
                            .wheel_valid(wheel_valid), .uop(uop));

    function automatic logic [31:0] prediction_constant(input logic [6:0] addr);
        case (addr)
            7'd64: prediction_constant = SIGMA_SCALE;
            7'd65: prediction_constant = W_NONCENTRAL;
            7'd66: prediction_constant = W_MEAN_CENTRAL;
            7'd67: prediction_constant = SQRT_ABS_W_CENTRAL;
            7'd83: prediction_constant = SQRT_W_NONCENTRAL;
            7'd95: prediction_constant = DT;
            // Normal positive sample intervals are required. Scaling by
            // one half is exact for normal inputs above the minimum exponent.
            7'd96: prediction_constant = {1'b0, DT[30:23] - 8'd1, DT[22:0]};
            default: prediction_constant = 32'b0;
        endcase
    endfunction

    assign read_addr_a = uop.src_a;
    assign read_addr_b = uop.src_b;
    assign a = read_data_a | prediction_constant(read_addr_a) |
               (angle_wrap_step(wrap_word) & {32{read_addr_a == PROM_WRAP_STEP}});
    assign b = read_data_b | prediction_constant(read_addr_b) |
               (angle_wrap_step(wrap_word) & {32{read_addr_b == PROM_WRAP_STEP}});
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
    assign write_enable = write_result && !bad_control && !bad_result;
    assign write_addr = uop.dest;
    assign write_data = result_word;

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
                pc <= '0;
                depth <= '0;
                success <= 1'b0;
                state <= DT_VALID ? ISSUE : FINISH;
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
