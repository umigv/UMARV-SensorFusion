
module fp32_mac (
    input  logic        clock,
    input  logic        reset,
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
    // During multiply, product_mag[50:3] is the combined partial
    // product and multiplier. After normalization, bit 50 is the leading
    // bit and bit 51 is reserved for addition carry.
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

    // Share one significand adder across serial multiply, add/subtract and round.
    always @* begin
        alu_left = '0;
        alu_right = '0;
        subtract = 1'b0;
        case (state)
            MULTIPLY: begin
                alu_left = {28'b0, product_mag[50:27]};
                alu_right = product_mag[3] ? {28'b0, multiplicand} : 52'b0;
            end
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
    // Sparse carry lookahead: 13 four-bit blocks, with a parallel prefix
    // over block boundaries. The same tree handles add and subtract.
    localparam integer CARRY_BLOCKS = 13;
    localparam integer PREFIX_LEVELS = $clog2(CARRY_BLOCKS);
    wire [51:0] alu_rhs = alu_right ^ {52{subtract}};
    wire [51:0] bit_p = alu_left ^ alu_rhs;
    wire [51:0] bit_g = alu_left & alu_rhs;
    wire [CARRY_BLOCKS-1:0] prefix_p [0:PREFIX_LEVELS];
    wire [CARRY_BLOCKS-1:0] prefix_g [0:PREFIX_LEVELS];

    for (genvar group_index = 0; group_index < CARRY_BLOCKS; group_index++) begin : carry_block
        localparam integer LOW_BIT = 4 * group_index;
        wire [3:0] p = bit_p[LOW_BIT +: 4];
        wire [3:0] g = bit_g[LOW_BIT +: 4];
        wire [3:0] carry;

        assign prefix_p[0][group_index] = &p;
        assign prefix_g[0][group_index] = g[3] | (p[3] & g[2]) |
            (p[3] & p[2] & g[1]) | (p[3] & p[2] & p[1] & g[0]);

        if (group_index == 0) begin : first
            assign carry[0] = subtract;
        end else begin : later
            assign carry[0] = prefix_g[PREFIX_LEVELS][group_index - 1] |
                (prefix_p[PREFIX_LEVELS][group_index - 1] & subtract);
        end
        assign carry[1] = g[0] | (p[0] & carry[0]);
        assign carry[2] = g[1] | (p[1] & g[0]) | (p[1] & p[0] & carry[0]);
        assign carry[3] = g[2] | (p[2] & g[1]) |
            (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & carry[0]);
        assign alu_result[LOW_BIT +: 4] = p ^ carry;
    end

    for (genvar level = 0; level < PREFIX_LEVELS; level++) begin : prefix_level
        localparam integer SPAN = 1 << level;
        for (genvar group_index = 0; group_index < CARRY_BLOCKS; group_index++) begin : group_prefix
            if (group_index >= SPAN) begin : combine
                assign prefix_p[level + 1][group_index] =
                    prefix_p[level][group_index] & prefix_p[level][group_index - SPAN];
                assign prefix_g[level + 1][group_index] = prefix_g[level][group_index] |
                    (prefix_p[level][group_index] & prefix_g[level][group_index - SPAN]);
            end else begin : forward
                assign prefix_p[level + 1][group_index] = prefix_p[level][group_index];
                assign prefix_g[level + 1][group_index] = prefix_g[level][group_index];
            end
        end
    end

    always_ff @(posedge clock or negedge reset) begin
        if (!reset) begin
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
            product_mag <= '0;
            acc_mag <= '0;
            product_exp <= '0;
            acc_exp <= '0;
            rounded <= '0;
            discarded <= 1'b0;
            biased_exp <= '0;
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
                            product_mag <= {25'b0, significand(b), 3'b0};
                            multiply_step <= '0;
                            state <= MULTIPLY;
                        end
                    end
                end

                MULTIPLY: begin
                    product_mag <= {1'b0, alu_result[24:0], product_mag[26:4], 3'b0};
                    multiply_step <= multiply_step + 1'b1;
                    if (multiply_step == 5'd23)
                        state <= NORMALIZE_PRODUCT;
                end

                NORMALIZE_PRODUCT: begin
                    if (product_mag[50]) begin
                        state <= NORMALIZE_ACC;
                    end else begin
                        product_mag <= product_mag << 1;
                        product_exp <= product_exp - 10'sd1;
                    end
                end

                NORMALIZE_ACC: begin
                    if (acc_mag == 0) begin
                        sum_sign <= product_sign;
                        state <= NORMALIZE_SUM;
                    end else if (acc_mag[50]) begin
                        state <= ALIGN;
                    end else begin
                        acc_mag <= acc_mag << 1;
                        acc_exp <= acc_exp - 10'sd1;
                    end
                end

                ALIGN: begin
                    if (product_exp < acc_exp) begin
                        product_mag <= shift_right_jam(product_mag);
                        product_exp <= product_mag[51:1] == 0 ? acc_exp : product_exp + 10'sd1;
                    end else if (acc_exp < product_exp) begin
                        acc_mag <= shift_right_jam(acc_mag);
                        acc_exp <= acc_mag[51:1] == 0 ? product_exp : acc_exp + 10'sd1;
                    end else begin
                        state <= ADD;
                    end
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
                    end else if (product_mag[51] || product_exp < -10'sd126) begin
                        product_mag <= shift_right_jam(product_mag);
                        product_exp <= product_mag[51:1] == 0 ? -10'sd126 : product_exp + 10'sd1;
                    end else if (!product_mag[50] && product_exp > -10'sd126) begin
                        product_mag <= product_mag << 1;
                        product_exp <= product_exp - 10'sd1;
                    end else begin
                        state <= ROUND;
                    end
                end

                ROUND: begin
                    rounded <= alu_result[24:0];
                    discarded <= |product_mag[26:0];
                    biased_exp <= product_exp[7:0] + 8'd127;
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

