`timescale 1ns/1ps
// Independent wide dot-product accumulators sharing only clock and reset.
// clear[i] starts a sum with a[i]*b[i]; subsequent terms add full products.
// last[i] includes this term and publishes a rounded, saturated result[i].
// Each lane follows MAC_accumulator timing: done on every completed term,
// result_valid only on last. start[i] is accepted only when !busy[i].
// Operand/result arrays are indexed [lane]; status vectors use the same index.
module MAC_array #(
    parameter integer LANES = 11,
    parameter integer WIDTH = 32,
    parameter integer FRAC = 16,
    parameter integer ACC_WIDTH = 2*WIDTH + 8
)(
    input  logic clk,
    input  logic resetn,
    input  logic [LANES-1:0] start,
    input  logic [LANES-1:0] clear,
    input  logic [LANES-1:0] last,
    input  logic signed [WIDTH-1:0] a [0:LANES-1],
    input  logic signed [WIDTH-1:0] b [0:LANES-1],
    output wire signed [WIDTH-1:0] result [0:LANES-1],
    output wire [LANES-1:0] busy,
    output wire [LANES-1:0] done,
    output wire [LANES-1:0] result_valid,
    output wire [LANES-1:0] overflow,
    output wire [LANES-1:0] acc_overflow
);
    initial begin
        if (LANES < 1)
            $fatal(1, "Require LANES >= 1");
    end

    genvar lane;
    generate
        for (lane = 0; lane < LANES; lane = lane + 1) begin : gen_mac
            MAC_accumulator #(.WIDTH(WIDTH), .FRAC(FRAC),
                              .ACC_WIDTH(ACC_WIDTH)) mac_inst (
                .clk(clk),
                .resetn(resetn),
                .start(start[lane]),
                .clear(clear[lane]),
                .last(last[lane]),
                .a(a[lane]),
                .b(b[lane]),
                .result(result[lane]),
                .busy(busy[lane]),
                .done(done[lane]),
                .result_valid(result_valid[lane]),
                .overflow(overflow[lane]),
                .acc_overflow(acc_overflow[lane])
            );
        end
    endgenerate
endmodule
