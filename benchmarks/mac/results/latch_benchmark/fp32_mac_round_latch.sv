// Generated from production fp32_mac.sv by latch_benchmark.py.
`timescale 1ns/1ps
// Production Architecture A, specialized from the verified benchmark core
// (ARCH=0, TOPOLOGY=4, FAST_SHIFT=1). The experimental variants remain separate.
module fp32_mac_round_latch (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic        clear,
    input  logic        last,
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic        busy,
    output logic        done,
    output logic        result_valid,
    output logic [31:0] result,
    output logic [4:0]  exception_flags
);
    localparam [31:0] QUIET_NAN = 32'h7fc00000;
    typedef enum logic [3:0] {
        IDLE, MULTIPLY, NORMALIZE_PRODUCT, NORMALIZE_ACC, ALIGN,
        ADD, NEGATE_SUM, NORMALIZE_SUM, ROUND, PACK, COMPLETE
    } state_t;
    state_t state;

    logic [31:0] accumulator, next_value, addend;
    logic [4:0] flags;
    logic publish, product_sign, sum_sign;
    logic [23:0] multiplicand;
    logic [4:0] multiply_step;
    localparam integer MULTIPLY_CYCLES = 13;
    logic [26:0] digits;
    logic [27:0] booth_upper;
    logic [23:0] booth_lower;
    wire [27:0] booth_sum;

    // Radix-4 Booth recoding selects 0, ±A, or ±2A. One narrow adder
    // commits two low bits per step, with a final unsigned correction digit.
    wire [2:0] digit = digits[2:0];
    wire negative = digit[2];
    wire [2:0] recode = negative ? ~digit : digit;
    wire [2:0] magnitude = {2'b0,recode[0]} + {2'b0,recode[1]};
    wire [27:0] term = magnitude == 1 ? {4'b0,multiplicand} :
                       magnitude == 2 ? {3'b0,multiplicand,1'b0} : 28'b0;
    wire [28:0] extended = {booth_upper,1'b1} + {term ^ {28{negative}},negative};
    assign booth_sum = extended[28:1];

    // Bit 50 is the normalized leading bit; bit 51 holds addition carry.
    logic [51:0] product_mag, acc_mag;
    logic signed [9:0] product_exp, acc_exp;
    logic [24:0] rounded;
    logic discarded;
    logic [7:0] biased_exp;
    logic [51:0] alu_left, alu_right, alu_result;
    logic subtract, round_up;
    logic product_infinite, invalid_product, signaling_nan;

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

    function automatic logic [23:0] significand(input logic [31:0] word);
        significand = {word[30:23] != 0, word[22:0]};
    endfunction

    function automatic logic [51:0] shift_right_jam(input logic [51:0] mag);
        shift_right_jam = {1'b0, mag[51:2], mag[1] | mag[0]};
    endfunction

    assign busy = state != IDLE;
    assign addend = clear ? 32'b0 : accumulator;
    assign round_up = product_mag[26] && (product_mag[27] || (|product_mag[25:0]));

    assign product_infinite = (is_inf(a) || is_inf(b)) &&
                              !(is_zero(a) || is_zero(b) || is_nan(a) || is_nan(b));
    assign invalid_product = (is_inf(a) && is_zero(b)) || (is_zero(a) && is_inf(b));
    assign signaling_nan = (is_nan(a) && !a[22]) || (is_nan(b) && !b[22]);

    // Share the wide significand adder across add/subtract and round.
    always @* begin
        alu_left = '0;
        alu_right = '0;
        subtract = 1'b0;
        case (state)
            ADD: begin
                subtract = product_sign != sum_sign;
                alu_left = product_mag;
                alu_right = acc_mag;
            end
            NEGATE_SUM: begin
                subtract = 1'b1;
                alu_right = product_mag;
            end
            ROUND: begin
                alu_left = {28'b0, product_mag[50:27]};
                alu_right = {51'b0, round_up};
            end
            default: begin end
        endcase
    end
    // Four-bit local ripple with a parallel prefix between the 13 groups.
    wire [51:0] adder_right = alu_right ^ {52{subtract}};
    wire [51:0] p = alu_left ^ adder_right, g = alu_left & adder_right;
    wire [51:0] carry;
    wire [12:0] block_p, block_g;
    wire [12:0] prefix_p [0:4], prefix_g [0:4];
    assign alu_result = p ^ carry;
    assign carry[0] = subtract;
    assign prefix_p[0] = block_p;
    assign prefix_g[0] = block_g;

    for (genvar k = 0; k < 13; k++) begin : group
        localparam integer I = 4*k;
        wire [3:0] lp = p[I +: 4], lg = g[I +: 4];
        assign block_p[k] = &lp;
        assign block_g[k] = lg[3] | (lp[3] & lg[2]) |
            (lp[3] & lp[2] & lg[1]) | (lp[3] & lp[2] & lp[1] & lg[0]);
        if (k > 0)
            assign carry[I] = prefix_g[4][k-1] | (prefix_p[4][k-1] & subtract);
        for (genvar j = 1; j < 4; j++)
            assign carry[I+j] = lg[j-1] | (lp[j-1] & carry[I+j-1]);
    end

    for (genvar level = 0; level < 4; level++) begin : prefix_level
        localparam integer SPAN = 1 << level;
        for (genvar k = 0; k < 13; k++) begin : prefix_bit
            if (k >= SPAN) begin : combine
                assign prefix_p[level+1][k] = prefix_p[level][k] & prefix_p[level][k-SPAN];
                assign prefix_g[level+1][k] = prefix_g[level][k] |
                    (prefix_p[level][k] & prefix_g[level][k-SPAN]);
            end else begin : pass
                assign prefix_p[level+1][k] = prefix_p[level][k];
                assign prefix_g[level+1][k] = prefix_g[level][k];
            end
        end
    end

    // Binary group selection forms a balanced leading-zero detector.
    function automatic [5:0] left_count(input logic [51:0] mag);
        logic [63:0] x;
        logic [5:0] zeros;
        begin
            x = {mag[50:0], 13'b0};
            zeros = 0;
            if (!(|x[63:32])) begin x = x << 32; zeros[5] = 1; end
            if (!(|x[63:48])) begin x = x << 16; zeros[4] = 1; end
            if (!(|x[63:56])) begin x = x << 8; zeros[3] = 1; end
            if (!(|x[63:60])) begin x = x << 4; zeros[2] = 1; end
            if (!(|x[63:62])) begin x = x << 2; zeros[1] = 1; end
            if (!x[63]) zeros[0] = 1;
            left_count = zeros;
        end
    endfunction

    function automatic [51:0] align_jam(input logic [51:0] mag,
                                          input logic [10:0] distance);
        logic [51:0] x;
        integer k;
        begin
            x = mag;
            for (k = 0; k < 6; k = k + 1)
                if (distance[k])
                    x = (x >> (1 << k)) |
                        {51'b0, |(x & (52'hfffffffffffff >> (52 - (1 << k))))};
            align_jam = distance >= 52 ? {51'b0, |mag} : x;
        end
    endfunction

    wire [5:0] product_left = left_count(product_mag);
    wire [5:0] acc_left = left_count(acc_mag);
    wire signed [10:0] product_wide = {product_exp[9],product_exp};
    wire signed [10:0] acc_wide = {acc_exp[9],acc_exp};
    wire signed [10:0] normal_room = product_wide + 11'sd126;
    wire [5:0] sum_left = normal_room >= $signed({5'b0,product_left}) ?
                          product_left : normal_room[5:0];

    // Capture the enable on the opposite phase so the data gate stays
    // stable while open. PACK is reached only after a fresh ROUND capture.
    logic round_enable;
    always_latch begin
        if (clk)
            round_enable <= state == ROUND;
    end

    always_latch begin
        if (!clk && round_enable) begin
            rounded <= alu_result[24:0];
            discarded <= |product_mag[26:0];
            biased_exp <= product_exp[7:0] + 8'd127;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            accumulator <= '0;
            result <= '0;
            exception_flags <= '0;
            done <= 1'b0;
            result_valid <= 1'b0;
            publish <= 1'b0;
            product_sign <= 1'b0;
            sum_sign <= 1'b0;
            flags <= '0;
            next_value <= '0;
            multiplicand <= '0;
            multiply_step <= '0;
            digits <= '0;
            booth_upper <= '0;
            booth_lower <= '0;
            product_mag <= '0;
            acc_mag <= '0;
            product_exp <= '0;
            acc_exp <= '0;
        end else begin
            done <= 1'b0;
            result_valid <= 1'b0;
            case (state)
                IDLE: begin
                    if (clear)
                        accumulator <= '0;
                    if (start) begin
                        publish <= last;
                        flags <= '0;
                        state <= COMPLETE;
                        if (invalid_product || signaling_nan ||
                            (product_infinite && is_inf(addend) &&
                             ((a[31] ^ b[31]) != addend[31]))) begin
                            next_value <= QUIET_NAN;
                            flags[4] <= 1'b1;
                        end else if (is_nan(a) || is_nan(b) || is_nan(addend)) begin
                            next_value <= QUIET_NAN;
                        end else if (product_infinite) begin
                            next_value <= {a[31] ^ b[31], 8'hff, 23'b0};
                        end else if (is_inf(addend)) begin
                            next_value <= addend;
                        end else if (is_zero(a) || is_zero(b)) begin
                            next_value <= is_zero(addend)
                                ? {(a[31] ^ b[31]) & addend[31], 31'b0} : addend;
                        end else begin
                            product_sign <= a[31] ^ b[31];
                            sum_sign <= addend[31];
                            product_exp <= exponent(a) + exponent(b) + 10'sd1;
                            acc_exp <= exponent(addend);
                            acc_mag <= {1'b0, significand(addend), 27'b0};
                            multiplicand <= significand(a);
                            digits <= {2'b0,significand(b),1'b0};
                            booth_upper <= '0;
                            booth_lower <= '0;
                            multiply_step <= '0;
                            state <= MULTIPLY;
                        end
                    end
                end

                MULTIPLY: begin
                    multiply_step <= multiply_step + 1'b1;
                    digits <= digits >> 2;
                    booth_upper <= {{2{booth_sum[27]}},booth_sum[27:2]};
                    booth_lower <= {booth_sum[1:0],booth_lower[23:2]};
                    if (multiply_step == MULTIPLY_CYCLES-1) begin
                        // Once the low 24 bits are committed, the unsigned
                        // correction digit contributes only to the upper half.
                        product_mag <= {1'b0,booth_sum[23:0],booth_lower,3'b0};
                        state <= NORMALIZE_PRODUCT;
                    end
                end

                NORMALIZE_PRODUCT: begin
                    product_mag <= product_mag << product_left;
                    product_exp <= product_exp - $signed({4'b0,product_left});
                    state <= NORMALIZE_ACC;
                end

                NORMALIZE_ACC: begin
                    if (acc_mag == 0) begin
                        sum_sign <= product_sign;
                        state <= NORMALIZE_SUM;
                    end else begin
                        acc_mag <= acc_mag << acc_left;
                        acc_exp <= acc_exp - $signed({4'b0,acc_left});
                        state <= ALIGN;
                    end
                end

                ALIGN: begin
                    if (product_exp < acc_exp) begin
                        product_mag <= align_jam(product_mag, acc_wide - product_wide);
                        product_exp <= acc_exp;
                    end else if (acc_exp < product_exp) begin
                        acc_mag <= align_jam(acc_mag, product_wide - acc_wide);
                    end
                    state <= ADD;
                end

                ADD: begin
                    product_mag <= alu_result;
                    // Aligned inputs have bit 51 clear, so a negative difference
                    // is distinguishable from a same-sign addition carry.
                    if (subtract && alu_result[51]) begin
                        state <= NEGATE_SUM;
                    end else begin
                        sum_sign <= product_sign;
                        state <= NORMALIZE_SUM;
                    end
                end

                NEGATE_SUM: begin
                    product_mag <= alu_result;
                    state <= NORMALIZE_SUM;
                end

                NORMALIZE_SUM: begin
                    if (product_mag == 0) begin
                        next_value <= '0;
                        state <= COMPLETE;
                    end else begin
                        if (product_exp < -10'sd126) begin
                            product_mag <= align_jam(product_mag, -normal_room);
                            product_exp <= -10'sd126;
                        end else if (product_mag[51]) begin
                            product_mag <= shift_right_jam(product_mag);
                            product_exp <= product_exp + 10'sd1;
                        end else if (!product_mag[50]) begin
                            product_mag <= product_mag << sum_left;
                            product_exp <= product_exp - $signed({4'b0,sum_left});
                        end
                        state <= ROUND;
                    end
                end

                ROUND: begin
                    state <= PACK;
                end

                PACK: begin
                    if (product_exp > 10'sd127 || (product_exp == 10'sd127 && rounded[24])) begin
                        next_value <= {sum_sign, 8'hff, 23'b0};
                        flags[2:0] <= 3'b101;
                    end else if (rounded[24]) begin
                        next_value <= {sum_sign, biased_exp + 8'd1, 23'b0};
                        flags[0] <= discarded;
                    end else if (rounded[23]) begin
                        next_value <= {sum_sign, biased_exp, rounded[22:0]};
                        flags[0] <= discarded;
                    end else begin
                        next_value <= {sum_sign, 8'b0, rounded[22:0]};
                        flags[1:0] <= {discarded, discarded};
                    end
                    state <= COMPLETE;
                end

                COMPLETE: begin
                    accumulator <= next_value;
                    exception_flags <= flags;
                    done <= 1'b1;
                    if (publish) begin
                        result <= next_value;
                        result_valid <= 1'b1;
                    end
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
