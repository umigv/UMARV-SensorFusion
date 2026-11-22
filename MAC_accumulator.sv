`timescale 1ns/1ps
// Dot-product MAC. Inputs have FRAC fractional bits; the internal accumulator
// retains 2*FRAC fractional bits. No per-term rounding or output saturation.
// Pulse start while !busy. clear starts a new sum with a*b; otherwise add a*b.
// last includes this product and publishes a rounded, saturated result.
// clear/last are sampled with start; clear+last computes a one-term sum.
// Each accepted term completes one clock later with done=1. result_valid pulses
// only for last; result/overflow otherwise hold. Inputs/starts while busy ignored.
// Final rounding is nearest, ties away from zero. FRAC=0 needs no rounding.
// ACC_WIDTH defaults to room for at least 256 full-range products. Choose it
// from the maximum term count and operand bounds, not the number of MAC lanes.
// Internal overflow wraps and sets sticky acc_overflow until clear/reset.
// overflow on last flags either accumulator overflow or output saturation;
// a sum with acc_overflow is invalid even if later terms cancel the wrapped sum.
// Synchronous active-low reset clears everything and aborts pending work.
module MAC_accumulator #(
    parameter integer WIDTH = 32,
    parameter integer FRAC = 16,
    parameter integer ACC_WIDTH = 2*WIDTH + 8
)(
    input logic clk, resetn, start, clear, last,
    input logic signed [WIDTH-1:0] a, b,
    output logic signed [WIDTH-1:0] result,
    output logic busy, done, result_valid, overflow, acc_overflow,
    output wire signed [ACC_WIDTH-1:0] full_result
);
    localparam integer EXT = ACC_WIDTH + 1;
    localparam logic signed [WIDTH-1:0] MAX_POS = {1'b0, {(WIDTH-1){1'b1}}};
    localparam logic signed [WIDTH-1:0] MIN_NEG = {1'b1, {(WIDTH-1){1'b0}}};
    localparam logic signed [EXT-1:0] MAX_EXT = {{(EXT-WIDTH){1'b0}}, MAX_POS};
    localparam logic signed [EXT-1:0] MIN_EXT = {{(EXT-WIDTH){1'b1}}, MIN_NEG};
    logic signed [ACC_WIDTH-1:0] accumulator;
    // Unrounded sum, updated on each done and finalized with result_valid.
    assign full_result = accumulator;
    logic signed [2*WIDTH-1:0] product;
    logic clear_pending, last_pending;
    wire signed [EXT-1:0] product_ext = {{(EXT-2*WIDTH){product[2*WIDTH-1]}}, product};
    wire signed [EXT-1:0] base = clear_pending ? '0 :
        {accumulator[ACC_WIDTH-1], accumulator};
    wire signed [EXT-1:0] total = base + product_ext;
    wire sum_overflow = total[ACC_WIDTH] != total[ACC_WIDTH-1];
    wire fault = (clear_pending ? 1'b0 : acc_overflow) | sum_overflow;
    // The extra bit permits absolute value of the most negative accumulator.
    wire [EXT-1:0] magnitude = total < 0 ? -total : total;
    wire [EXT-1:0] rounded_magnitude;
    generate
        if (FRAC == 0) begin : gen_integer
            assign rounded_magnitude = magnitude;
        end else begin : gen_fraction
            assign rounded_magnitude = (magnitude >> FRAC) + magnitude[FRAC-1];
        end
    endgenerate
    wire signed [EXT-1:0] scaled = total < 0 ? -$signed(rounded_magnitude) :
                                                              $signed(rounded_magnitude);
    initial begin
        if (WIDTH < 2 || FRAC < 0 || FRAC >= WIDTH || ACC_WIDTH < 2*WIDTH)
            $fatal(1, "Require WIDTH >= 2, 0 <= FRAC < WIDTH, ACC_WIDTH >= 2*WIDTH");
    end
    always_ff @(posedge clk) begin
        if (!resetn) begin
            accumulator <= '0; product <= '0;
            clear_pending <= 0; last_pending <= 0;
            result <= '0; busy <= 0; done <= 0;
            result_valid <= 0; overflow <= 0; acc_overflow <= 0;
        end else begin
            done <= 0; result_valid <= 0;
            if (busy) begin
                accumulator <= total[ACC_WIDTH-1:0];
                acc_overflow <= fault;
                busy <= 0; done <= 1;
                if (last_pending) begin
                    result_valid <= 1;
                    overflow <= fault || scaled > MAX_EXT || scaled < MIN_EXT;
                    if (scaled > MAX_EXT) result <= MAX_POS;
                    else if (scaled < MIN_EXT) result <= MIN_NEG;
                    else result <= scaled[WIDTH-1:0];
                end
            end else if (start) begin
                product <= a*b;
                clear_pending <= clear; last_pending <= last;
                busy <= 1;
            end
        end
    end
endmodule
