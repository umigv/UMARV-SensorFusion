`timescale 1ns/1ps
module fp32_mac_a #(parameter integer TOPOLOGY=4, FAST_SHIFT=1) (
    input wire clk, rst_n, start, clear, last,
    input wire [31:0] a, b,
    output wire busy, done, result_valid,
    output wire [31:0] result,
    output wire [4:0] exception_flags
);
    fp32_mac_experiment #(.ARCH(0), .TOPOLOGY(TOPOLOGY), .FAST_SHIFT(FAST_SHIFT))
        core (.*);
endmodule

module fp32_mac_b #(parameter integer TOPOLOGY=4, FAST_SHIFT=1) (
    input wire clk, rst_n, start, clear, last,
    input wire [31:0] a, b,
    output wire busy, done, result_valid,
    output wire [31:0] result,
    output wire [4:0] exception_flags
);
    fp32_mac_experiment #(.ARCH(1), .TOPOLOGY(TOPOLOGY), .FAST_SHIFT(FAST_SHIFT))
        core (.*);
endmodule

module fp32_mac_c #(parameter integer TOPOLOGY=4, FAST_SHIFT=1) (
    input wire clk, rst_n, start, clear, last,
    input wire [31:0] a, b,
    output wire busy, done, result_valid,
    output wire [31:0] result,
    output wire [4:0] exception_flags
);
    fp32_mac_experiment #(.ARCH(2), .TOPOLOGY(TOPOLOGY), .FAST_SHIFT(FAST_SHIFT))
        core (.*);
endmodule

module fp32_mac_d #(parameter integer TOPOLOGY=4, FAST_SHIFT=1) (
    input wire clk, rst_n, start, clear, last,
    input wire [31:0] a, b,
    output wire busy, done, result_valid,
    output wire [31:0] result,
    output wire [4:0] exception_flags
);
    fp32_mac_experiment #(.ARCH(3), .TOPOLOGY(TOPOLOGY), .FAST_SHIFT(FAST_SHIFT))
        core (.*);
endmodule

module fp32_mac_e #(parameter integer TOPOLOGY=4, FAST_SHIFT=1) (
    input wire clk, rst_n, start, clear, last,
    input wire [31:0] a, b,
    output wire busy, done, result_valid,
    output wire [31:0] result,
    output wire [4:0] exception_flags
);
    fp32_mac_experiment #(.ARCH(4), .TOPOLOGY(TOPOLOGY), .FAST_SHIFT(FAST_SHIFT))
        core (.*);
endmodule

module fp32_mac_f #(parameter integer TOPOLOGY=4, FAST_SHIFT=1) (
    input wire clk, rst_n, start, clear, last,
    input wire [31:0] a, b,
    output wire busy, done, result_valid,
    output wire [31:0] result,
    output wire [4:0] exception_flags
);
    fp32_mac_experiment #(.ARCH(5), .TOPOLOGY(TOPOLOGY), .FAST_SHIFT(FAST_SHIFT))
        core (.*);
endmodule

module fp32_mac_g #(parameter integer TOPOLOGY=4, FAST_SHIFT=1) (
    input wire clk, rst_n, start, clear, last,
    input wire [31:0] a, b,
    output wire busy, done, result_valid,
    output wire [31:0] result,
    output wire [4:0] exception_flags
);
    fp32_mac_experiment #(.ARCH(6), .TOPOLOGY(TOPOLOGY), .FAST_SHIFT(FAST_SHIFT))
        core (.*);
endmodule
