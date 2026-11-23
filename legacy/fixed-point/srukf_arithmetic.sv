`timescale 1ns/1ps

// Single shared signed product/accumulation path. Each command contributes
// one unrounded product, optionally shifted left to align binary points.
module srukf_mac (
    input wire clk, resetn, start, clear, subtract,
    input wire signed [31:0] a, b,
    input wire [2:0] align,
    output reg signed [71:0] result,
    output reg done, overflow
);
    reg pending, first, minus;
    reg [2:0] shift;
    reg signed [63:0] product;
    wire signed [72:0] aligned = 73'(product) <<< shift;
    wire signed [72:0] base = first ? 73'sd0 : 73'(result);
    wire signed [72:0] sum = base + (minus ? -aligned : aligned);
    always @(posedge clk) begin
        if (!resetn) begin
            pending <= 0; done <= 0; overflow <= 0; result <= 0;
            first <= 0; minus <= 0; shift <= 0; product <= 0;
        end else begin
            done <= pending;
            pending <= start;
            if (start) begin
                product <= a*b;
                first <= clear;
                minus <= subtract;
                shift <= align;
            end
            if (pending) begin
                result <= sum[71:0];
                overflow <= sum[72] != sum[71];
            end
        end
    end
endmodule

// F50 radicand to F25 factor. The raw integer square root consumes the
// entire MAC result. Explicit sign and range checks precede publication.
module srukf_full_root (
    input wire clk, resetn, start,
    input wire signed [71:0] radicand,
    output reg signed [31:0] result,
    output reg done, negative, overflow
);
    reg [71:0] bits;
    reg [35:0] root;
    reg [37:0] remainder;
    reg [5:0] count;
    wire [37:0] trial = (remainder << 2) | {36'd0,bits[71:70]};
    wire [38:0] difference = {1'b0,trial} - {1'b0,root,2'b01};
    wire take_bit = !difference[38];
    wire [35:0] next_root = {root[34:0],take_bit};
    always @(posedge clk) begin
        if (!resetn) begin
            count <= 0; bits <= 0; root <= 0; remainder <= 0;
            result <= 0; done <= 0; negative <= 0; overflow <= 0;
        end else begin
            done <= 0;
            if (count != 0) begin
                bits <= bits << 2;
                root <= next_root;
                remainder <= take_bit ? difference[37:0] : trial;
                count <= count-1'b1;
                if (count == 1) begin
                    overflow <= |next_root[35:31];
                    result <= |next_root[35:31] ? 32'sh7fffffff : next_root[31:0];
                    done <= 1;
                end
            end else if (start) begin
                negative <= radicand[71];
                overflow <= 0;
                if (radicand[71]) begin
                    result <= 0;
                    done <= 1;
                end else begin
                    bits <= radicand;
                    count <= 36;
                    root <= 0;
                    remainder <= 0;
                end
            end
        end
    end
endmodule

// Division of a prescaled 72-bit numerator by a signed 32-bit denominator.
// Quotient is truncated toward zero. Only the dividend/quotient shift
// register is wide. The restoring subtractor is 33 bits.
module srukf_full_div (
    input wire clk, resetn, start,
    input wire signed [71:0] numerator,
    input wire signed [31:0] denominator,
    output reg signed [31:0] result,
    output reg done, zero, overflow
);
    reg [71:0] quotient;
    reg [31:0] remainder, divisor;
    reg [6:0] count;
    reg negative;
    wire [32:0] trial = {remainder,quotient[71]};
    wire [32:0] difference = trial - {1'b0,divisor};
    wire take_bit = !difference[32];
    wire [71:0] next_q = {quotient[70:0],take_bit};
    wire [71:0] limit = negative ? 72'd2147483648 : 72'd2147483647;
    always @(posedge clk) begin
        if (!resetn) begin
            count <= 0; quotient <= 0; remainder <= 0; divisor <= 0;
            negative <= 0; result <= 0; done <= 0; zero <= 0; overflow <= 0;
        end else begin
            done <= 0;
            if (count != 0) begin
                quotient <= next_q;
                remainder <= take_bit ? difference[31:0] : trial[31:0];
                count <= count-1'b1;
                if (count == 1) begin
                    overflow <= next_q > limit;
                    if (next_q > limit) result <= negative ? 32'sh80000000 : 32'sh7fffffff;
                    else result <= negative ? -next_q[31:0] : next_q[31:0];
                    done <= 1;
                end
            end else if (start) begin
                zero <= denominator == 0;
                overflow <= 0;
                if (denominator == 0) begin
                    result <= numerator[71] ? 32'sh80000000 : 32'sh7fffffff;
                    done <= 1;
                end else begin
                    count <= 72;
                    remainder <= 0;
                    quotient <= numerator[71] ? -numerator : numerator;
                    divisor <= denominator[31] ? -denominator : denominator;
                    negative <= numerator[71] ^ denominator[31];
                end
            end
        end
    end
endmodule
