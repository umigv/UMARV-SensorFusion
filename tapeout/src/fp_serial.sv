// Small IEEE single-precision add / subtract / multiply / divide / square root, one bit per clock,
// plus "integer difference to float" for turning sensor counts into numbers.
//
// Built for area, not speed: one 29-bit adder is shared by every operation, and every shift is
// by one place per clock. An operation takes about 30 to 80 clocks. A 400 Hz filter beat is
// 125,000 clocks, so a few hundred operations per beat fit easily.
//
// Rounding is to nearest, ties to even. Numbers below the smallest normal value are treated as
// zero, and a result too large becomes infinity. There is no not-a-number handling.
//   square root   takes operand a only and ignores its sign
//   integer difference   takes the low 26 bits of a and of b as signed integers and returns
//                        a - b as a float (exact up to 24 bits, rounded beyond)
//
// Use: hold op steady, put operand a on din with load_a, then operand b with load_b, then pulse
// start. done is high for one clock when result is ready; result holds until the next start.
module fp_serial (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        load_a,
    input  logic        load_b,
    input  logic        start,
    input  logic [2:0]  op,         // 0 add, 1 subtract, 2 multiply, 3 divide, 4 square root, 5 integer difference
    input  logic [31:0] din,
    output logic        busy,
    output logic        done,
    output logic [31:0] result
);
    localparam [3:0] S_IDLE = 4'd0, S_SWAP = 4'd1, S_ALIGN = 4'd2, S_ADD = 4'd3, S_NORM = 4'd4,
                     S_MUL = 4'd5, S_MULFIX = 4'd6, S_MULGS = 4'd7, S_DIV = 4'd8, S_DIVFIX = 4'd9,
                     S_ROUND = 4'd10, S_PACK = 4'd11, S_ISUB = 4'd12, S_INEG = 4'd13, S_SQRT = 4'd14,
                     S_SQRTFIX = 4'd15;
    localparam [2:0] OP_SUB = 3'd1, OP_MUL = 3'd2, OP_DIV = 3'd3, OP_SQRT = 3'd4, OP_ISUB = 3'd5;

    logic [3:0]  st;
    logic [27:0] x;          // [27] carry, [26:3] mantissa, [2:0] guard bits
    logic [26:0] y;          // [26:3] mantissa, [2:0] guard bits
    logic [25:0] q;          // multiplier, then low half of the product; or the quotient
    logic        sx, sy;
    logic [7:0]  ex, ey;
    logic signed [10:0] e;   // result exponent, with room to go out of range
    logic [4:0]  cnt;
    logic        is_sub, is_zero, is_inf;

    wire [7:0]  din_e = din[30:23];
    wire [23:0] din_m = {|din_e, (din_e == 8'd0) ? 23'd0 : din[22:0]};

    // the one adder
    logic [28:0] ad_p, ad_r;
    logic        ad_sub;
    wire  [28:0] ad_t = ad_sub ? (ad_p - ad_r) : (ad_p + ad_r);
    always_comb begin
        ad_p = {1'b0, x};
        ad_r = {2'b00, y};
        ad_sub = 1'b0;
        case (st)
            S_ADD:   ad_sub = is_sub;
            S_MUL:   ad_r = q[0] ? {2'b00, y} : 29'd0;
            S_DIV:   ad_sub = 1'b1;
            S_ROUND: ad_r = 29'd8;
            S_ISUB:  begin ad_p = {x[27], x}; ad_r = {y[26], y[26], y}; ad_sub = 1'b1; end
            S_INEG:  begin ad_p = 29'd0; ad_r = {x[27], x}; ad_sub = 1'b1; end
            S_SQRT:  begin ad_p = {1'b0, x[25:0], y[26:25]}; ad_r = {2'b00, q[24:0], 2'b01}; ad_sub = 1'b1; end
            default: ;
        endcase
    end

    wire round_up = x[2] && (x[1] || x[0] || x[3]);
    wire [8:0] sqrt_e = {1'b0, ex} + 9'd127;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; done <= 1'b0; x <= 28'd0; y <= 27'd0; q <= 26'd0;
            sx <= 1'b0; sy <= 1'b0; ex <= 8'd0; ey <= 8'd0; e <= 11'sd0; cnt <= 5'd0;
            is_sub <= 1'b0; is_zero <= 1'b0; is_inf <= 1'b0; result <= 32'd0;
        end else begin
            done <= 1'b0;
            case (st)
                S_IDLE: begin
                    if (load_a) begin
                        sx <= din[31]; ex <= din_e; q <= {2'b00, din_m};
                        x <= (op == OP_ISUB) ? {{2{din[25]}}, din[25:0]} : {1'b0, din_m, 3'b000};
                    end
                    if (load_b) begin
                        sy <= din[31]; ey <= din_e;
                        y <= (op == OP_ISUB) ? {din[25], din[25:0]} : {din_m, 3'b000};
                    end
                    if (start) begin
                        is_zero <= 1'b0; is_inf <= 1'b0;
                        case (op)
                            OP_MUL: begin
                                e <= $signed({3'b000, ex}) + $signed({3'b000, ey}) - 11'sd127;
                                sx <= sx ^ sy; x <= 28'd0; cnt <= 5'd23; st <= S_MUL;
                            end
                            OP_DIV: begin
                                e <= $signed({3'b000, ex}) - $signed({3'b000, ey}) + 11'sd127;
                                sx <= sx ^ sy; q <= 26'd0; cnt <= 5'd25; st <= S_DIV;
                                is_inf <= (ey == 8'd0);
                            end
                            OP_SQRT: begin
                                // the result exponent is half the operand's; an odd one gives up a bit to the mantissa
                                e <= $signed({2'b00, sqrt_e[8:1]});
                                y <= ex[0] ? {1'b0, x[26:3], 2'b00} : {x[26:3], 3'b000};
                                sx <= 1'b0; x <= 28'd0; q <= 26'd0; cnt <= 5'd24;
                                if (ex == 8'd0) begin is_zero <= 1'b1; st <= S_PACK; end
                                else st <= S_SQRT;
                            end
                            OP_ISUB: st <= S_ISUB;
                            default: begin
                                if (op == OP_SUB) sy <= ~sy;
                                st <= (ey > ex) ? S_SWAP : S_ALIGN;
                            end
                        endcase
                    end
                end

                // ---- add and subtract ----
                S_SWAP: begin
                    x <= {1'b0, y}; y <= x[26:0];
                    sx <= sy; sy <= sx; ex <= ey; ey <= ex;
                    st <= S_ALIGN;
                end
                S_ALIGN: begin
                    is_sub <= sx ^ sy;
                    e <= $signed({3'b000, ex});
                    if (ex != ey && y[26:1] != 26'd0) begin
                        y <= {1'b0, y[26:2], y[1] | y[0]};     // the lowest bit remembers what fell off
                        ey <= ey + 8'd1;
                    end else begin
                        st <= S_ADD;
                    end
                end
                S_ADD: begin
                    if (is_sub && ad_t[28]) begin
                        st <= S_SWAP;                           // the other one was larger: go round again
                        ey <= ex;
                    end else begin
                        x <= ad_t[27:0];
                        st <= S_NORM;
                    end
                end
                S_NORM: begin
                    if (x == 28'd0) begin
                        is_zero <= 1'b1; sx <= 1'b0; st <= S_PACK;
                    end else if (x[27]) begin
                        x <= {1'b0, x[27:2], x[1] | x[0]}; e <= e + 11'sd1;
                    end else if (!x[26]) begin
                        x <= {x[26:0], 1'b0}; e <= e - 11'sd1;
                    end else begin
                        st <= S_ROUND;
                    end
                end

                // ---- multiply: 24 add-and-shift steps ----
                S_MUL: begin
                    x <= {1'b0, ad_t[27:4], 3'b000};
                    q <= {2'b00, ad_t[3], q[23:1]};
                    cnt <= cnt - 5'd1;
                    if (cnt == 5'd0) st <= S_MULFIX;
                end
                S_MULFIX: begin
                    if (x[26]) begin
                        e <= e + 11'sd1;
                    end else begin
                        x <= {1'b0, x[25:3], q[23], 3'b000};
                        q <= {2'b00, q[22:0], 1'b0};
                    end
                    st <= S_MULGS;
                end
                S_MULGS: begin
                    x[2:0] <= {q[23], |q[22:0], 1'b0};
                    if (!x[26]) begin is_zero <= 1'b1; st <= S_PACK; end
                    else st <= S_ROUND;
                end

                // ---- divide: 26 subtract-and-shift steps ----
                S_DIV: begin
                    if (!ad_t[28]) x <= {ad_t[26:0], 1'b0};
                    else           x <= {x[26:0], 1'b0};
                    q <= {q[24:0], ~ad_t[28]};
                    cnt <= cnt - 5'd1;
                    if (cnt == 5'd0) st <= S_DIVFIX;
                end
                S_DIVFIX: begin
                    if (q[25]) begin
                        x <= {1'b0, q[25:2], q[1], q[0] | (x != 28'd0), 1'b0};
                    end else begin
                        x <= {1'b0, q[24:1], q[0], (x != 28'd0), 1'b0};
                        e <= e - 11'sd1;
                    end
                    if (q[25:24] == 2'b00) begin is_zero <= 1'b1; st <= S_PACK; end
                    else st <= S_ROUND;
                end

                // ---- integer difference: subtract, take the size, then normalise like a sum ----
                S_ISUB: begin
                    x <= ad_t[27:0]; sx <= ad_t[28]; e <= 11'sd153;
                    st <= ad_t[28] ? S_INEG : S_NORM;
                end
                S_INEG: begin
                    x <= ad_t[27:0];
                    st <= S_NORM;
                end

                // ---- square root: 25 steps, one result bit each ----
                S_SQRT: begin
                    if (!ad_t[28]) begin x <= ad_t[27:0];              q <= {q[24:0], 1'b1}; end
                    else           begin x <= {x[25:0], y[26:25]};     q <= {q[24:0], 1'b0}; end
                    y <= {y[24:0], 2'b00};
                    cnt <= cnt - 5'd1;
                    if (cnt == 5'd0) st <= S_SQRTFIX;
                end
                S_SQRTFIX: begin
                    x <= {1'b0, q[24:1], q[0], (x != 28'd0), 1'b0};
                    st <= S_ROUND;
                end

                // ---- shared ending ----
                S_ROUND: begin
                    if (round_up) begin
                        if (ad_t[27]) begin
                            x <= {2'b01, 26'd0}; e <= e + 11'sd1;
                        end else begin
                            x <= ad_t[27:0];
                        end
                    end
                    st <= S_PACK;
                end
                default: begin   // S_PACK
                    if (is_inf || (!is_zero && e >= 11'sd255)) result <= {sx, 8'hFF, 23'd0};
                    else if (is_zero || e <= 11'sd0)           result <= {sx, 31'd0};
                    else                                       result <= {sx, e[7:0], x[25:3]};
                    done <= 1'b1;
                    st <= S_IDLE;
                end
            endcase
        end
    end

    assign busy = (st != S_IDLE);
endmodule
