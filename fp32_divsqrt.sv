`timescale 1ns/1ps
module fp32_divsqrt (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic        mode,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic        busy,
    output logic        done,
    output logic [31:0] result,
    output logic        invalid,
    output logic        divide_by_zero,
    output logic        overflow,
    output logic        underflow,
    output logic        inexact
);
    localparam [31:0] QUIET_NAN = 32'h7fc00000;
    typedef enum logic [2:0] {
        IDLE, NORMALIZE, SEED, DIGIT, FINISH_DIGITS, DENORMALIZE, ROUND, COMPLETE
    } state_t;
    state_t state;

    logic root_mode, result_sign;
    logic signed [9:0] exp_a, exp_b, result_exp;
    logic [23:0] sig_a, sig_b;
    logic [25:0] root_pairs;
    logic [27:0] remainder;
    logic [26:0] digits;
    logic [4:0] digit_index;
    logic [31:0] next_result;
    logic [4:0] next_flags;

    logic [27:0] alu_left, alu_right;
    logic [27:0] alu_value, next_remainder;
    logic subtract, carry, round_up, discarded;
    logic [7:0] biased_exp, carried_exp;
    wire [29:0] alu_extended = {1'b0, alu_left, 1'b1}
        + {1'b0, alu_right ^ {28{subtract}}, subtract};

    function automatic logic is_nan(input logic [31:0] word);
        is_nan = (&word[30:23]) && (|word[22:0]);
    endfunction

    function automatic logic is_inf(input logic [31:0] word);
        is_inf = (&word[30:23]) && !(|word[22:0]);
    endfunction

    function automatic logic is_zero(input logic [31:0] word);
        is_zero = !(|word[30:0]);
    endfunction

    function automatic logic signed [9:0] exponent(input logic [31:0] word);
        exponent = (word[30:23] == 0 ? 10'sd1 : $signed({2'b00, word[30:23]}))
                   - 10'sd127;
    endfunction

    assign busy = state != IDLE;
    assign {carry, alu_value} = alu_extended[29:1];
    assign next_remainder = carry ? alu_value : alu_left;
    assign round_up = digits[2] && (digits[3] || (|digits[1:0]));
    assign discarded = |digits[2:0];
    assign biased_exp = result_exp[7:0] + 8'd127;
    assign carried_exp = result_exp[7:0] + 8'd128;

    // One carry chain serves the magnitude comparison, both trial subtractions,
    // and the final rounding increment. The carry bit means no borrow.
    always @* begin
        alu_left = '0;
        alu_right = '0;
        subtract = 1'b0;
        case (state)
            SEED: begin
                alu_left = {4'b0, sig_a};
                alu_right = {4'b0, sig_b};
                subtract = 1'b1;
            end
            DIGIT: begin
                alu_left = root_mode ? {remainder[25:0], root_pairs[25:24]} : remainder;
                alu_right = root_mode ? {1'b0, digits[24:0], 2'b01} : {4'b0, sig_b};
                subtract = 1'b1;
            end
            ROUND: begin
                alu_left = {4'b0, digits[26:3]};
                alu_right = {27'b0, round_up};
            end
            default: begin end
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            done <= 1'b0;
            result <= '0;
            {invalid, divide_by_zero, overflow, underflow, inexact} <= '0;
            root_mode <= 1'b0;
            result_sign <= 1'b0;
            exp_a <= '0;
            exp_b <= '0;
            result_exp <= '0;
            sig_a <= '0;
            sig_b <= '0;
            root_pairs <= '0;
            remainder <= '0;
            digits <= '0;
            digit_index <= '0;
            next_result <= '0;
            next_flags <= '0;
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: if (start) begin
                    root_mode <= mode;
                    result_sign <= !mode && (a[31] ^ b[31]);
                    next_flags <= '0;
                    state <= COMPLETE;
                    if (is_nan(a) || (!mode && is_nan(b))) begin
                        next_result <= QUIET_NAN;
                        next_flags[4] <= (is_nan(a) && !a[22]) ||
                                         (!mode && is_nan(b) && !b[22]);
                    end else if ((mode && a[31] && !is_zero(a)) ||
                                 (!mode && ((is_zero(a) && is_zero(b)) ||
                                            (is_inf(a) && is_inf(b))))) begin
                        next_result <= QUIET_NAN;
                        next_flags[4] <= 1'b1;
                    end else if (mode && (is_zero(a) || is_inf(a))) begin
                        next_result <= a;
                    end else if (!mode && (is_inf(a) || is_zero(b))) begin
                        next_result <= {a[31] ^ b[31], 8'hff, 23'b0};
                        next_flags[3] <= !is_inf(a);
                    end else if (!mode && (is_zero(a) || is_inf(b))) begin
                        next_result <= {a[31] ^ b[31], 31'b0};
                    end else begin
                        exp_a <= exponent(a);
                        exp_b <= exponent(b);
                        sig_a <= {a[30:23] != 0, a[22:0]};
                        sig_b <= {b[30:23] != 0, b[22:0]};
                        state <= NORMALIZE;
                    end
                end

                NORMALIZE: begin
                    if (!sig_a[23]) begin
                        sig_a <= sig_a << 1;
                        exp_a <= exp_a - 10'sd1;
                    end
                    if (!root_mode && !sig_b[23]) begin
                        sig_b <= sig_b << 1;
                        exp_b <= exp_b - 10'sd1;
                    end
                    if (sig_a[23] && (root_mode || sig_b[23]))
                        state <= SEED;
                end

                SEED: begin
                    digits <= '0;
                    digit_index <= '0;
                    if (root_mode) begin
                        // Odd exponents double the significand before rooting.
                        result_exp <= exp_a >>> 1;
                        root_pairs <= exp_a[0] ? {sig_a, 2'b0} : {1'b0, sig_a, 1'b0};
                        remainder <= '0;
                    end else begin
                        result_exp <= exp_a - exp_b - (carry ? 10'sd0 : 10'sd1);
                        remainder <= carry ? {4'b0, sig_a} : {3'b0, sig_a, 1'b0};
                    end
                    state <= DIGIT;
                end

                DIGIT: begin
                    digits <= {digits[25:0], carry};
                    root_pairs <= {root_pairs[23:0], 2'b0};
                    remainder <= root_mode ? next_remainder : {next_remainder[26:0], 1'b0};
                    digit_index <= digit_index + 1'b1;
                    if (digit_index == 5'd25)
                        state <= FINISH_DIGITS;
                end

                FINISH_DIGITS: begin
                    // 24 retained bits, guard, round, and a sticky remainder.
                    digits <= {digits[25:0], |remainder};
                    state <= DENORMALIZE;
                end

                DENORMALIZE: begin
                    if (result_exp < -10'sd126) begin
                        digits <= {1'b0, digits[26:2], digits[1] | digits[0]};
                        result_exp <= digits[26:1] == 0 ? -10'sd126 : result_exp + 10'sd1;
                    end else begin
                        state <= ROUND;
                    end
                end

                ROUND: begin
                    next_flags[0] <= discarded;
                    if (result_exp > 10'sd127 || (result_exp == 10'sd127 && alu_value[24])) begin
                        next_result <= {result_sign, 8'hff, 23'b0};
                        next_flags[2:0] <= 3'b101;
                    end else if (alu_value[24]) begin
                        next_result <= {result_sign, carried_exp, 23'b0};
                    end else if (alu_value[23]) begin
                        next_result <= {result_sign, biased_exp, alu_value[22:0]};
                    end else begin
                        next_result <= {result_sign, 8'b0, alu_value[22:0]};
                        next_flags[1] <= discarded;
                    end
                    state <= COMPLETE;
                end

                COMPLETE: begin
                    result <= next_result;
                    {invalid, divide_by_zero, overflow, underflow, inexact} <= next_flags;
                    done <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
