`timescale 1ns/1ps
// Signed fixed-point restoring divider. a, b, c share FRAC fractional bits.
// c = trunc_toward_zero((a * 2**FRAC) / b), saturated to WIDTH bits.
// Accept start only when !busy. Normal latency: WIDTH+FRAC clocks after
// acceptance; divide-by-zero completes on acceptance. done pulses one cycle.
// resetn is synchronous, active low. Results/flags hold until next completion.
// One shared WIDTH+1-bit subtractor; no multiplication or division operators.
module division #(
    parameter integer WIDTH = 32,
    parameter integer FRAC = 16
)(
    input  logic clk,
    input  logic resetn,
    input  logic start,
    input  logic signed [WIDTH-1:0] a,
    input  logic signed [WIDTH-1:0] b,
    output logic signed [WIDTH-1:0] c,
    output logic busy,
    output logic done,
    output logic div_by_zero,
    output logic overflow
);
    localparam integer STEPS = WIDTH + FRAC;
    localparam integer CW = $clog2(STEPS + 1);
    localparam logic [WIDTH-1:0] MIN_MAG = {1'b1, {(WIDTH-1){1'b0}}};
    localparam logic [WIDTH-1:0] MAX_POS = {1'b0, {(WIDTH-1){1'b1}}};
    logic [STEPS-1:0] quot_shift;
    logic [WIDTH-1:0] remainder, divisor;
    logic [CW-1:0] count;
    logic negative;
    wire [WIDTH-1:0] a_mag = a[WIDTH-1] ? (~a + 1'b1) : a;
    wire [WIDTH-1:0] b_mag = b[WIDTH-1] ? (~b + 1'b1) : b;
    wire [WIDTH:0] trial = {remainder, quot_shift[STEPS-1]};
    wire [WIDTH:0] difference = trial - {1'b0, divisor};
    wire take_bit = !difference[WIDTH];
    wire [STEPS-1:0] next_q = (quot_shift << 1) | {{(STEPS-1){1'b0}}, take_bit};
    wire [WIDTH-1:0] next_r = take_bit ? difference[WIDTH-1:0] : trial[WIDTH-1:0];
    wire [STEPS-1:0] limit = negative ? MIN_MAG : MAX_POS;

    assign quotient = c;

    initial begin
        if (WIDTH < 2 || FRAC < 0 || FRAC >= WIDTH)
            $fatal(1, "Require WIDTH >= 2 and 0 <= FRAC < WIDTH");
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            c <= '0; busy <= 0; done <= 0;
            div_by_zero <= 0; overflow <= 0;
            quot_shift <= '0; remainder <= '0; divisor <= '0;
            count <= '0; negative <= 0;
        end else begin
            done <= 0;
            if (!busy) begin
                if (start) begin
                    if (b == 0) begin
                        c <= a[WIDTH-1] ? MIN_MAG : MAX_POS;
                        div_by_zero <= 1; overflow <= 0; done <= 1;
                    end else begin
                        quot_shift <= {a_mag, {FRAC{1'b0}}};
                        remainder <= '0; divisor <= b_mag;
                        negative <= a[WIDTH-1] ^ b[WIDTH-1];
                        count <= STEPS; busy <= 1;
                    end
                end
            end else begin
                quot_shift <= next_q; 
                remainder <= next_r;
                count <= count - 1'b1;
                if (count == 1) begin
                    busy <= 0; done <= 1; div_by_zero <= 0;
                    overflow <= next_q > limit;
                    if (next_q > limit)
                        c <= negative ? MIN_MAG : MAX_POS;
                    else
                        c <= negative ? (~next_q[WIDTH-1:0] + 1'b1) : next_q[WIDTH-1:0];
                end
            end
        end
    end
endmodule
