`timescale 1ns/1ps
// Signed fixed-point input; nonnegative result in the same Q format.
// result_raw = floor(sqrt(a_raw * 2**FRAC)).
// start is accepted only while idle. Negative inputs complete immediately
// with result=0 and negative_input=1; valid inputs take STEPS clocks.
// done pulses for one cycle; result and flag hold until next completion.
// Synchronous active-low reset aborts any operation.
module SQRT #(
    parameter integer WIDTH = 32,
    parameter integer FRAC = 16
)(
    input  logic clk,
    input  logic resetn,
    input  logic start,
    input  logic signed [WIDTH-1:0] a,
    output logic signed [WIDTH-1:0] result,
    output logic busy,
    output logic done,
    output logic negative_input
);
    localparam integer STEPS = (WIDTH + FRAC + 1) / 2;
    localparam integer RAD_WIDTH = 2 * STEPS;
    localparam integer CW = $clog2(STEPS + 1);
    logic [RAD_WIDTH-1:0] radicand;
    logic [STEPS-1:0] root;
    logic [STEPS+1:0] remainder;
    logic [CW-1:0] count;

    // Consume two radicand bits and produce one root bit per clock.
    wire [STEPS+1:0] trial = (remainder << 2) |
        {{STEPS{1'b0}}, radicand[RAD_WIDTH-1 -: 2]};
    wire [STEPS+1:0] divisor = {root, 2'b01};
    wire [STEPS+2:0] difference = {1'b0, trial} - {1'b0, divisor};
    wire take_bit = !difference[STEPS+2];
    wire [STEPS-1:0] next_root = (root << 1) | take_bit;

    initial begin
        if (WIDTH < 2 || FRAC < 0 || FRAC >= WIDTH)
            $fatal(1, "Require WIDTH >= 2 and 0 <= FRAC < WIDTH");
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            result <= '0;
            busy <= 1'b0;
            done <= 1'b0;
            negative_input <= 1'b0;
            radicand <= '0;
            root <= '0;
            remainder <= '0;
            count <= '0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                if (start) begin
                    if (a[WIDTH-1]) begin
                        result <= '0;
                        negative_input <= 1'b1;
                        done <= 1'b1;
                    end else begin
                        radicand <= {{(RAD_WIDTH-WIDTH-FRAC){1'b0}}, a,
                                     {FRAC{1'b0}}};
                        root <= '0;
                        remainder <= '0;
                        count <= STEPS;
                        busy <= 1'b1;
                    end
                end
            end else begin
                radicand <= radicand << 2;
                root <= next_root;
                remainder <= take_bit ? difference[STEPS+1:0] : trial;
                count <= count - 1'b1;
                if (count == 1) begin
                    result <= {{(WIDTH-STEPS){1'b0}}, next_root};
                    negative_input <= 1'b0;
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            end
        end
    end
endmodule
