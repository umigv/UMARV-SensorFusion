`timescale 1ns/1ps
module mac_booth4_row(input wire [23:0] a, input wire [2:0] digit,
                      output wire [25:0] row);
    wire negative = digit[2];
    wire [25:0] magnitude = (digit == 3'b001 || digit == 3'b010 ||
        digit == 3'b101 || digit == 3'b110) ? {2'b0, a} :
        (digit == 3'b011 || digit == 3'b100) ? {1'b0, a, 1'b0} : 26'b0;
    assign row = negative ? -magnitude : magnitude;
endmodule

// Serial Booth recurrence over the signed upper remainder. Each iteration
// commits K low bits before shifting the upper remainder arithmetically.
module mac_booth_step #(parameter integer K = 2) (
    input wire [23:0] a,
    input wire [25:0] triple,
    input wire [3:0] encoded_digit,
    input wire [27:0] upper,
    output wire [27:0] sum
);
    wire [3:0] digit = K == 2 ? {encoded_digit[2], encoded_digit[2:0]} : encoded_digit;
    wire negative = digit[3];
    wire [3:0] recode = negative ? ~digit : digit;
    wire [2:0] magnitude = K == 2 ?
        ({2'b0,recode[0]} + {2'b0,recode[1]}) :
        ({2'b0,recode[0]} + {2'b0,recode[1]} + {1'b0,recode[2],1'b0});
    wire [27:0] term = magnitude == 1 ? {4'b0,a} :
                       magnitude == 2 ? {3'b0,a,1'b0} :
                       magnitude == 3 ? {2'b0,triple} :
                       magnitude == 4 ? {2'b0,a,2'b0} : 28'b0;
    wire [28:0] extended = {upper,1'b1} + {term ^ {28{negative}},negative};
    assign sum = extended[28:1];
endmodule

module mac_product_terms #(parameter integer ARCH = 2) (
    input wire [23:0] a, b,
    input wire [23:0] digits,
    input wire [51:0] partial_sum,
    output wire [51:0] left, right
);
    if (ARCH < 5) begin : chunk
        localparam integer K = ARCH == 2 ? 4 : ARCH == 3 ? 8 : 12;
        // Horner recurrence walks most-significant chunk first.
        wire [23+K:0] chunk_product = a * digits[23 -: K];
        assign left = partial_sum << K;
        assign right = {{(28-K){1'b0}}, chunk_product};
    end else begin : parallel_product
        wire [47:0] x, y;
        if (ARCH == 5) begin : raw
            mac_dadda_unsigned reducer(a, b, x, y);
        end else begin : recoded
            mac_dadda_booth reducer(a, b, x, y);
        end
        assign left = {4'b0,x};
        assign right = {4'b0,y};
    end
endmodule
