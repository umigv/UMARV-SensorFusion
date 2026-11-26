// FPGA replacement for tapeout/src/lidar_latch_mem.sv.
//
// Keeps the exact same port list, combinational read, and 2-clock write latency
// as the Sky130 latch-based ASIC memory, using Kintex-7 distributed RAM (LUTRAM)
// so tapeout/src/ stays untouched.
module lidar_latch_mem (
    input  logic        clk,
    input  logic        we,
    input  logic [7:0]  waddr,
    input  logic [15:0] wdata,
    input  logic [7:0]  raddr,
    output logic [15:0] rdata
);

    // Match the 2-clock write latency of the ASIC latch array:
    //   posedge 0: register we, waddr, wdata
    //   posedge 1: second pipeline stage
    //   posedge 2: commit to distributed RAM
    logic        we_q1, we_q2;
    logic [7:0]  wa_q1, wa_q2;
    logic [15:0] wd_q1, wd_q2;

    (* ram_style = "distributed" *) logic [15:0] mem [0:255];

    always_ff @(posedge clk) begin
        we_q1 <= we;
        wa_q1 <= waddr;
        wd_q1 <= wdata;

        we_q2 <= we_q1;
        wa_q2 <= wa_q1;
        wd_q2 <= wd_q1;

        if (we_q2) begin
            mem[wa_q2] <= wd_q2;
        end
    end

    assign rdata = mem[raddr];

endmodule
