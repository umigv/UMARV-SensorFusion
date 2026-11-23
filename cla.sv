`timescale 1ns/1ps
// ============================================================================
// Carry Lookahead Adder (CLA) in SystemVerilog
//
// Computes sum = a + b + cin with carry-lookahead logic.
// Carries are evaluated through lookahead tree structures using generate (g)
// and propagate (p) signals to eliminate ripple-carry latency.
//
// Included Modules:
//   1. cla        - Top-level parameterized CLA (defaults to 32 bits, supports
//                   hierarchical fast paths for 32/64 bits and generic N-bit).
//   2. cla_32bit  - Dedicated 32-bit hierarchical CLA (2x cla_16bit + LCU).
//   3. cla_64bit  - Dedicated 64-bit hierarchical CLA (4x cla_16bit + top LCU).
//   4. cla_16bit  - 16-bit CLA block (4x cla_4bit + cla_lcu_4bit).
//   5. cla_4bit   - 4-bit CLA slice with group P & G outputs.
//   6. cla_lcu_4bit - 4-bit Lookahead Carry Unit.
//
// Ports for top module (cla):
//   a        - [WIDTH-1:0] First operand
//   b        - [WIDTH-1:0] Second operand
//   cin      - Carry input
//   sum      - [WIDTH-1:0] Addition result
//   cout     - Carry output (unsigned overflow)
//   overflow - Signed 2's complement overflow flag
// ============================================================================

module cla #(
    parameter integer WIDTH = 32
)(
    input  logic [WIDTH-1:0] a,
    input  logic [WIDTH-1:0] b,
    input  logic             cin,
    output logic [WIDTH-1:0] sum,
    output logic             cout,
    output logic             overflow
);
    initial begin
        if (WIDTH < 1)
            $fatal(1, "Require WIDTH >= 1");
    end

    generate
        if (WIDTH == 32) begin : gen_cla_32
            cla_32bit u_cla32 (
                .a        (a),
                .b        (b),
                .cin      (cin),
                .sum      (sum),
                .cout     (cout),
                .overflow (overflow),
                .pg       (),
                .gg       ()
            );
        end else if (WIDTH == 64) begin : gen_cla_64
            cla_64bit u_cla64 (
                .a        (a),
                .b        (b),
                .cin      (cin),
                .sum      (sum),
                .cout     (cout),
                .overflow (overflow),
                .pg       (),
                .gg       ()
            );
        end else if (WIDTH == 16) begin : gen_cla_16
            cla_16bit u_cla16 (
                .a        (a),
                .b        (b),
                .cin      (cin),
                .sum      (sum),
                .cout     (cout),
                .overflow (overflow),
                .pg       (),
                .gg       ()
            );
        end else begin : gen_cla_generic
            // Parameterized N-bit carry lookahead logic
            logic [WIDTH-1:0] g;
            logic [WIDTH-1:0] p;
            logic [WIDTH:0]   c;

            assign g = a & b;
            assign p = a ^ b;
            assign c[0] = cin;

            genvar i, j;
            for (i = 0; i < WIDTH; i++) begin : gen_carries
                logic [i:0] terms;

                assign terms[0] = cin & (&p[i:0]);

                for (j = 0; j < i; j++) begin : gen_terms
                    assign terms[j+1] = g[j] & (&p[i:j+1]);
                end

                assign c[i+1] = g[i] | (|terms);
            end

            assign sum      = p ^ c[WIDTH-1:0];
            assign cout     = c[WIDTH];
            assign overflow = c[WIDTH] ^ c[WIDTH-1];
        end
    endgenerate

endmodule


// ============================================================================
// 64-bit Hierarchical Carry Lookahead Adder
// Built from four 16-bit CLA blocks and a top-level Lookahead Carry Unit.
// ============================================================================
module cla_64bit (
    input  logic [63:0] a,
    input  logic [63:0] b,
    input  logic        cin,
    output logic [63:0] sum,
    output logic        cout,
    output logic        overflow,
    output logic        pg,
    output logic        gg
);
    logic [3:0] block_p;
    logic [3:0] block_g;
    logic [4:0] block_c;

    assign block_c[0] = cin;

    // Top-level Lookahead Carry Unit for the four 16-bit blocks
    cla_lcu_4bit top_lcu (
        .p   (block_p),
        .g   (block_g),
        .cin (cin),
        .c   (block_c[4:1]),
        .pg  (pg),
        .gg  (gg)
    );

    genvar k;
    generate
        for (k = 0; k < 4; k++) begin : gen_16bit_slices
            cla_16bit slice (
                .a        (a[16*k +: 16]),
                .b        (b[16*k +: 16]),
                .cin      (block_c[k]),
                .sum      (sum[16*k +: 16]),
                .cout     (),
                .overflow (),
                .pg       (block_p[k]),
                .gg       (block_g[k])
            );
        end
    endgenerate

    assign cout     = block_c[4];
    assign overflow = (a[63] == b[63]) && (sum[63] != a[63]);

endmodule


// ============================================================================
// 32-bit Hierarchical Carry Lookahead Adder
// Built from two 16-bit CLA blocks with lookahead carry propagation.
// ============================================================================
module cla_32bit (
    input  logic [31:0] a,
    input  logic [31:0] b,
    input  logic        cin,
    output logic [31:0] sum,
    output logic        cout,
    output logic        overflow,
    output logic        pg,
    output logic        gg
);
    logic [1:0] block_p;
    logic [1:0] block_g;
    logic [2:0] block_c;

    assign block_c[0] = cin;

    // Fast lookahead carries across the two 16-bit blocks
    assign block_c[1] = block_g[0] | (block_p[0] & cin);
    assign block_c[2] = block_g[1] | (block_p[1] & block_g[0]) | (block_p[1] & block_p[0] & cin);

    cla_16bit lower_half (
        .a        (a[15:0]),
        .b        (b[15:0]),
        .cin      (block_c[0]),
        .sum      (sum[15:0]),
        .cout     (),
        .overflow (),
        .pg       (block_p[0]),
        .gg       (block_g[0])
    );

    cla_16bit upper_half (
        .a        (a[31:16]),
        .b        (b[31:16]),
        .cin      (block_c[1]),
        .sum      (sum[31:16]),
        .cout     (),
        .overflow (),
        .pg       (block_p[1]),
        .gg       (block_g[1])
    );

    assign cout     = block_c[2];
    assign pg       = block_p[1] & block_p[0];
    assign gg       = block_g[1] | (block_p[1] & block_g[0]);
    assign overflow = (a[31] == b[31]) && (sum[31] != a[31]);

endmodule


// ============================================================================
// 16-bit Carry Lookahead Adder Block
// Built with four 4-bit CLA blocks and one 4-bit Lookahead Carry Unit (LCU).
// ============================================================================
module cla_16bit (
    input  logic [15:0] a,
    input  logic [15:0] b,
    input  logic        cin,
    output logic [15:0] sum,
    output logic        cout,
    output logic        overflow,
    output logic        pg,
    output logic        gg
);
    logic [3:0] block_p;
    logic [3:0] block_g;
    logic [4:0] block_c;

    assign block_c[0] = cin;

    // Lookahead Carry Unit computes carries between 4-bit blocks
    cla_lcu_4bit lcu (
        .p   (block_p),
        .g   (block_g),
        .cin (cin),
        .c   (block_c[4:1]),
        .pg  (pg),
        .gg  (gg)
    );

    // Four 4-bit CLA slices
    genvar k;
    generate
        for (k = 0; k < 4; k++) begin : gen_cla_slices
            cla_4bit slice (
                .a    (a[4*k +: 4]),
                .b    (b[4*k +: 4]),
                .cin  (block_c[k]),
                .sum  (sum[4*k +: 4]),
                .cout (),
                .pg   (block_p[k]),
                .gg   (block_g[k])
            );
        end
    endgenerate

    assign cout     = block_c[4];
    assign overflow = (a[15] == b[15]) && (sum[15] != a[15]);

endmodule


// ============================================================================
// 4-bit Carry Lookahead Adder Block
// Produces 4-bit sum and block-level group propagate (pg) and generate (gg).
// ============================================================================
module cla_4bit (
    input  logic [3:0] a,
    input  logic [3:0] b,
    input  logic       cin,
    output logic [3:0] sum,
    output logic       cout,
    output logic       pg,   // Block propagate
    output logic       gg    // Block generate
);
    logic [3:0] g;
    logic [3:0] p;
    logic [4:0] c;

    assign g = a & b;
    assign p = a ^ b;

    assign c[0] = cin;
    assign c[1] = g[0] | (p[0] & c[0]);
    assign c[2] = g[1] | (p[1] & g[0]) | (p[1] & p[0] & c[0]);
    assign c[3] = g[2] | (p[2] & g[1]) | (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & c[0]);
    assign c[4] = g[3] | (p[3] & g[2]) | (p[3] & p[2] & g[1]) | (p[3] & p[2] & p[1] & g[0]) | (p[3] & p[2] & p[1] & p[0] & c[0]);

    assign sum  = p ^ c[3:0];
    assign cout = c[4];

    // Block group propagate & generate for higher-level LCU
    assign pg   = &p;
    assign gg   = g[3] | (p[3] & g[2]) | (p[3] & p[2] & g[1]) | (p[3] & p[2] & p[1] & g[0]);

endmodule


// ============================================================================
// 4-bit Lookahead Carry Unit (LCU)
// Takes group propagate (p) and generate (g) from 4 blocks and computes
// lookahead carries (c[4:1]) for each block in parallel.
// ============================================================================
module cla_lcu_4bit (
    input  logic [3:0] p,    // Block propagate signals from 4 lower blocks
    input  logic [3:0] g,    // Block generate signals from 4 lower blocks
    input  logic       cin,  // Global carry-in
    output logic [4:1] c,    // Lookahead carries into blocks 1, 2, 3, and carry-out
    output logic       pg,   // Super-group propagate
    output logic       gg    // Super-group generate
);
    assign c[1] = g[0] | (p[0] & cin);
    assign c[2] = g[1] | (p[1] & g[0]) | (p[1] & p[0] & cin);
    assign c[3] = g[2] | (p[2] & g[1]) | (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & cin);
    assign c[4] = g[3] | (p[3] & g[2]) | (p[3] & p[2] & g[1]) | (p[3] & p[2] & p[1] & g[0]) | (p[3] & p[2] & p[1] & p[0] & cin);

    assign pg   = &p;
    assign gg   = g[3] | (p[3] & g[2]) | (p[3] & p[2] & g[1]) | (p[3] & p[2] & p[1] & g[0]);

endmodule
