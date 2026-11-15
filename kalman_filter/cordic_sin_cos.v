`timescale 1ns/1ps
module Cordic_Sin_Cos (
    input wire clk,
    input wire reset,

    input wire [15:0] theta_in, // this is the target angle we want to calculate

    output wire [15:0] sin_output,
    output wire [15:0] cos_output
);

    wire [15:0] x;
    wire [15:0] y;

    // As part of the CORDIC algorithm, hardcode x=0.60725, y=0, so that the final vector (which
    // increases slightly every step of the algorithm) ends with x apprx. = 1. It grows by a predictable factor of roughly 1.64676
    assign x = 16'b00_10011011011101;  // using fixed point (w/ 1 sign bit, 1 integer bit, 14 frac bits), we can represent -2.0 to +1.9999
    assign y = 1'd0;                   // so 0.60725 * 2^14 = apprx. 9949, -> 10011011011101

    // Put actual cordic algorithm here - python pseudocode for algo:
    // ITERS = 16
    // theta_table = [atan2(1, 2**i) for i in range(ITERS)]

    // def compute_K(n):
    //     """
    //     Compute K(n) for n = ITERS. This could also be
    //     stored as an explicit constant if ITERS above is fixed.
    //     """
    //     k = 1.0
    //     for i in range(n):
    //         k *= 1 / sqrt(1 + 2 ** (-2 * i))
    //     return k

    // def cordic(alpha: float, n: int) -> tuple[float, float]:
    //     assert n <= ITERS
    //     K_n = compute_K(n)
    //     theta = 0.0
    //     x = 1.0
    //     y = 0.0
    //     P2i = 1  # This will be 2**(-i) in the loop below
    //     for arc_tangent in theta_table[:n]:
    //         sigma = +1 if theta < alpha else -1
    //         theta += sigma * arc_tangent
    //         x, y = x - sigma * y * P2i, sigma * P2i * x + y
    //         P2i /= 2
    //     return x * K_n, y * K_n

endmodule