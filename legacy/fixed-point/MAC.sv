`timescale 1ns/1ps
// Signed fixed-point fused multiply-add: result = a*b + c.
// All ports use WIDTH bits with FRAC fractional bits. Preserve full product
// precision through addition, then truncate toward zero and saturate.
// Accept start when !busy; complete one clock after acceptance. Starts while
// busy are ignored. done pulses for one cycle; result/overflow hold until the
// next completion. Synchronous active-low reset aborts pending work.
// Feed result back to c for a sequence of accumulated products.
module MAC #(
    parameter integer WIDTH = 32,
    parameter integer FRAC = 16
)(
    input  logic clk,
    input  logic resetn,
    input  logic start,
    input  logic signed [WIDTH-1:0] a,
    input  logic signed [WIDTH-1:0] b,
    input  logic signed [WIDTH-1:0] c,
    output logic signed [WIDTH-1:0] result,
    output logic busy,
    output logic done,
    output logic overflow
);
    localparam integer EXT = 2*WIDTH + 1;
    localparam logic signed [WIDTH-1:0] MAX_POS = {1'b0, {(WIDTH-1){1'b1}}};
    localparam logic signed [WIDTH-1:0] MIN_NEG = {1'b1, {(WIDTH-1){1'b0}}};
    localparam logic signed [EXT-1:0] MAX_EXT = {{(EXT-WIDTH){1'b0}}, MAX_POS};
    localparam logic signed [EXT-1:0] MIN_EXT = {{(EXT-WIDTH){1'b1}}, MIN_NEG};
    logic signed [2*WIDTH-1:0] product;
    logic signed [WIDTH-1:0] addend;
    wire signed [EXT-1:0] extended_product = {product[2*WIDTH-1], product};
    wire signed [EXT-1:0] extended_addend = {{(EXT-WIDTH){addend[WIDTH-1]}}, addend};
    wire signed [EXT-1:0] total = extended_product + (extended_addend <<< FRAC);
    // Shift the magnitude so negative fractions truncate toward zero too.
    wire signed [EXT-1:0] scaled = total < 0
        ? -((-total) >>> FRAC) : (total >>> FRAC);

    initial begin
        if (WIDTH < 2 || FRAC < 0 || FRAC >= WIDTH)
            $fatal(1, "Require WIDTH >= 2 and 0 <= FRAC < WIDTH");
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            product <= '0;
            addend <= '0;
            result <= '0;
            busy <= 1'b0;
            done <= 1'b0;
            overflow <= 1'b0;
        end else begin
            done <= 1'b0;
            if (busy) begin
                busy <= 1'b0;
                done <= 1'b1;
                overflow <= scaled > MAX_EXT || scaled < MIN_EXT;
                if (scaled > MAX_EXT) result <= MAX_POS;
                else if (scaled < MIN_EXT) result <= MIN_NEG;
                else result <= scaled[WIDTH-1:0];
            end else if (start) begin
                product <= a * b;
                addend <= c;
                busy <= 1'b1;
            end
        end
    end
endmodule
