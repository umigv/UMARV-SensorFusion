`timescale 1ns/1ps
module mac_physical_top (
    input wire clk, rst_n, start, clear, last,
    input wire [31:0] a, b,
    output wire busy, done, result_valid,
    output wire [31:0] result,
    output wire [4:0] exception_flags
);
    fp32_mac_matched_booth #(.ARCH(0), .FAST_SHIFT(0)) impl (.*);
endmodule
