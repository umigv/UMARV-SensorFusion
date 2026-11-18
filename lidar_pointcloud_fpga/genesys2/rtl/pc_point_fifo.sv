// Plain synchronous FIFO (block RAM, registered read: rd_data is valid the
// cycle after rd_en).
module pc_point_fifo #(
    parameter int WIDTH = 112,
    parameter int DEPTH = 512          // power of two
) (
    input  logic                   clk,
    input  logic                   rst,
    input  logic                   wr_en,
    input  logic [WIDTH-1:0]       wr_data,
    input  logic                   rd_en,
    output logic [WIDTH-1:0]       rd_data,
    output logic [$clog2(DEPTH):0] level,
    output logic                   overflow     // pulse: write dropped
);
    localparam int AW = $clog2(DEPTH);

    (* ram_style = "block" *)
    logic [WIDTH-1:0] mem [0:DEPTH-1];
    logic [AW-1:0]    wr_ptr, rd_ptr;

    wire full  = level[AW];
    wire empty = (level == '0);
    wire do_wr = wr_en && !full;
    wire do_rd = rd_en && !empty;

    always_ff @(posedge clk) begin
        if (do_wr) mem[wr_ptr] <= wr_data;
        if (do_rd) rd_data <= mem[rd_ptr];
    end

    always_ff @(posedge clk) begin
        overflow <= wr_en && full;
        if (do_wr) wr_ptr <= wr_ptr + 1'b1;
        if (do_rd) rd_ptr <= rd_ptr + 1'b1;
        level <= level + (AW+1)'(do_wr) - (AW+1)'(do_rd);
        if (rst) begin
            wr_ptr   <= '0;
            rd_ptr   <= '0;
            level    <= '0;
            overflow <= 1'b0;
        end
    end
endmodule
