`timescale 1ns/1ps
// TOPOLOGY: 0 ripple, 1 Brent-Kung, 2 Han-Carlson, 3 Kogge-Stone,
// 4 four-bit local ripple with a parallel prefix between blocks.
module experiment_adder #(parameter integer TOPOLOGY = 4) (
    input wire [51:0] a, b, input wire cin, output wire [51:0] sum
);
    wire [51:0] p = a ^ b, g = a & b;
    wire [51:0] carry;
    assign sum = p ^ carry;
    assign carry[0] = cin;
    if (TOPOLOGY == 0) begin : ripple
        for (genvar i = 1; i < 52; i++)
            assign carry[i] = g[i-1] | (p[i-1] & carry[i-1]);
    end else if (TOPOLOGY == 4) begin : hybrid
        wire [12:0] block_p, block_g, prefix_p, prefix_g;
        for (genvar k = 0; k < 13; k++) begin : group
            localparam integer I = 4*k;
            wire [3:0] lp = p[I +: 4], lg = g[I +: 4];
            assign block_p[k] = &lp;
            assign block_g[k] = lg[3] | (lp[3] & lg[2]) |
                (lp[3] & lp[2] & lg[1]) | (lp[3] & lp[2] & lp[1] & lg[0]);
            if (k > 0)
                assign carry[I] = prefix_g[k-1] | (prefix_p[k-1] & cin);
            for (genvar j = 1; j < 4; j++)
                assign carry[I+j] = lg[j-1] | (lp[j-1] & carry[I+j-1]);
        end
        experiment_prefix #(.WIDTH(13), .STYLE(3)) tree
            (block_p, block_g, prefix_p, prefix_g);
    end else begin : prefix
        wire [51:0] pp, pg;
        experiment_prefix #(.WIDTH(52), .STYLE(TOPOLOGY)) tree (p, g, pp, pg);
        for (genvar i = 1; i < 52; i++)
            assign carry[i] = pg[i-1] | (pp[i-1] & cin);
    end
endmodule

module experiment_prefix #(parameter integer WIDTH = 52, STYLE = 3) (
    input wire [WIDTH-1:0] p, g, output wire [WIDTH-1:0] pp, pg
);
    localparam integer LEVELS = $clog2(WIDTH);
    localparam integer STAGES = STYLE == 1 ? 2*LEVELS-1 : STYLE == 2 ? LEVELS+1 : LEVELS;
    wire [WIDTH-1:0] ps [0:STAGES], gs [0:STAGES];
    assign ps[0] = p;
    assign gs[0] = g;
    assign pp = ps[STAGES];
    assign pg = gs[STAGES];

    for (genvar s = 0; s < STAGES; s++) begin : level
        localparam integer SPAN = STYLE == 1 && s >= LEVELS ?
            (1 << (2*LEVELS-2-s)) : STYLE == 2 && s == LEVELS ? 1 : 1 << s;
        for (genvar i = 0; i < WIDTH; i++) begin : lane
            localparam integer ENABLE = STYLE == 1 ?
                (s < LEVELS ? (i+1) % (2*SPAN) == 0 :
                 (i+1) % (2*SPAN) == SPAN && i >= 2*SPAN) :
                STYLE == 2 ? (s == LEVELS ? i % 2 == 0 : i % 2 == 1) : 1;
            if (i >= SPAN && ENABLE) begin : combine
                assign ps[s+1][i] = ps[s][i] & ps[s][i-SPAN];
                assign gs[s+1][i] = gs[s][i] | (ps[s][i] & gs[s][i-SPAN]);
            end else begin : pass
                assign ps[s+1][i] = ps[s][i];
                assign gs[s+1][i] = gs[s][i];
            end
        end
    end
endmodule
