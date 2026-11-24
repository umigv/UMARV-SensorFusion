// 256 x 16 memory made of latches, for the lidar point buffer.
//
// A latch is about half the size of a flip-flop with a write enable, and the buffer is the
// largest thing in the lidar block.
//
// How a write stays safe:
//   clock 0   we, waddr, wdata are registered.
//   clock 1   the word select is decoded from the registered address. It may glitch while it
//             settles; nothing is listening yet.
//   clock 2   a clock gate per word (a latch that is open while clk is low, then an AND with
//             clk) passes one clean clk-high pulse to the selected word's 16 latches. The data,
//             registered a second time, is steady for the whole of this clock.
// So a word holds its new value from the middle of the second clock after the write.
// Writes on consecutive clocks are fine.
//
// Reading is combinational: rdata follows raddr.
//
// For synthesis the clock gates and latches are Sky130 cells placed by name. Simulation without
// the cell library uses the equivalent behaviour below.
module lidar_latch_mem (
    input  logic        clk,
    input  logic        we,
    input  logic [7:0]  waddr,
    input  logic [15:0] wdata,
    input  logic [7:0]  raddr,
    output logic [15:0] rdata
);

    logic        we_q;
    logic [7:0]  wa_q;
    logic [15:0] wd_q, wd_q2;
    always_ff @(posedge clk) begin
        we_q  <= we;
        wa_q  <= waddr;
        wd_q  <= wdata;
        wd_q2 <= wd_q;
    end

    wire [15:0] q [0:255];
    assign rdata = q[raddr];

    genvar i, b;
    generate
        for (i = 0; i < 256; i = i + 1) begin : word
            wire sel = we_q && (wa_q == i);
            wire gclk;
`ifdef SYNTHESIS
            sky130_fd_sc_hd__dlclkp_1 u_gate (.CLK(clk), .GATE(sel), .GCLK(gclk));
            for (b = 0; b < 16; b = b + 1) begin : bit_
                sky130_fd_sc_hd__dlxtp_1 u_latch (.D(wd_q2[b]), .GATE(gclk), .Q(q[i][b]));
            end
`else
            reg        sel_l;
            reg [15:0] d;
            always_latch if (!clk) sel_l = sel;
            assign gclk = clk & sel_l;
            always_latch if (gclk) d = wd_q2;
            assign q[i] = d;
`endif
        end
    endgenerate

endmodule
