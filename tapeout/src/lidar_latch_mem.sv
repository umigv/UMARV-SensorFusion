// 256 x 16 memory made of latches, for the lidar point buffer.
//
// A latch is about half the size of a flip-flop with a write enable, and the buffer is the
// largest thing in the lidar block.
//
// RULE FOR THE USER OF THIS BLOCK: writes must be at least 3 clocks apart. The point buffer
// writes once per lidar return, which is 12 clocks apart at the fastest.
//
// How a write stays safe, with no gated clocks:
//   rising edge 0     we, waddr and wdata are registered. The data register then holds its
//                     value until the next write, at least 3 clocks later.
//   falling edge 0.5  one flip-flop per word, clocked on the FALLING edge, takes the decoded
//                     word select. Its output is that word's latch enable: a clean, glitch-free
//                     level from a flip-flop, high for exactly one clock (falling edge 0.5 to
//                     falling edge 1.5).
//   The data has been steady for half a clock before the enable rises and stays steady for at
//   least a clock and a half after it falls. Both margins are there by construction; they do
//   not rely on the tools checking a latch's timing.
// A word holds its new value from falling edge 0.5, and must not be read before rising edge 2.
//
// Reading is combinational: rdata follows raddr.
//
// An earlier version used one clock gate per word. The place-and-route tools treated the 256
// gated clocks as clock nets, gave each of them wide wiring, and could not route the result.
//
// For synthesis the latches are Sky130 cells placed by name. Simulation without the cell
// library uses the equivalent behaviour below.
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
    logic [15:0] wd_q;
    always_ff @(posedge clk) begin
        we_q <= we;
        if (we) begin
            wa_q <= waddr;
            wd_q <= wdata;
        end
    end

    // one latch enable per word, from a flip-flop on the falling edge
    logic [255:0] gate;
    integer n;
    always_ff @(negedge clk) begin
        for (n = 0; n < 256; n = n + 1)
            gate[n] <= we_q && (wa_q == n[7:0]);
    end

    wire [15:0] q [0:255];
    assign rdata = q[raddr];

    genvar i;
    generate
        for (i = 0; i < 256; i = i + 1) begin : word
`ifdef SYNTHESIS
            genvar b;
            for (b = 0; b < 16; b = b + 1) begin : bit_
                sky130_fd_sc_hd__dlxtp_1 u_latch (.D(wd_q[b]), .GATE(gate[i]), .Q(q[i][b]));
            end
`else
            reg [15:0] d;
            always_latch if (gate[i]) d = wd_q;
            assign q[i] = d;
`endif
        end
    endgenerate

endmodule
