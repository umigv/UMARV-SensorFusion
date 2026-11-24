// Asynchronous FIFO: independent write and read clocks, Gray-coded pointers.
//
// Brought over from the EECS 427 final project (FINAL/verilog/accelerator/async_fifo.sv),
// where it passed SpyGlass CDC checks at depth 2. The logic is unchanged. Edits made for the
// Tiny Tapeout tools, all mechanical:
//   - the unused `include of acc_defs.svh is gone
//   - "parameter int" is written as plain "parameter"
//   - size casts such as PTR_WIDTH'(1) are written out as concatenations
//   - the initial block that checked the parameters with $fatal is removed
//     (DEPTH must still be a power of two, 2 or more; WIDTH at least 1)

// Reset assertion is asynchronous. Supply reset release synchronized to clk.
module twoflopsync #(
    parameter WIDTH = 4
) (
    input  logic clk, rst,
    input  logic [WIDTH-1:0] d_in,
    output logic [WIDTH-1:0] d_out
);
    (* ASYNC_REG = "TRUE" *) logic [WIDTH-1:0] stage1, stage2;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            stage1 <= '0;
            stage2 <= '0;
        end else begin
            stage1 <= d_in;
            stage2 <= stage1;
        end
    end
    assign d_out = stage2;
endmodule

module bin2gray #(
    parameter WIDTH = 4
) (
    input  logic [WIDTH-1:0] bin_in,
    output logic [WIDTH-1:0] gray_out
);
    assign gray_out = (bin_in >> 1) ^ bin_in;
endmodule

// Independent-clock storage with registered read data. Contents are not reset.
module fifo #(
    parameter WIDTH = 16,
    parameter DEPTH = 8,
    parameter ADDR_WIDTH = $clog2(DEPTH)
) (
    input  logic wr_en, rd_en, wr_clk, rd_clk, wr_full,
    input  logic [ADDR_WIDTH-1:0] wr_ptr, rd_ptr,
    input  logic [WIDTH-1:0] data_in,
    output logic [WIDTH-1:0] data_out
);
    logic [WIDTH-1:0] mem [0:DEPTH-1];
    always_ff @(posedge wr_clk) begin
        if (wr_en && !wr_full) mem[wr_ptr] <= data_in;
    end
    always_ff @(posedge rd_clk) begin
        if (rd_en) data_out <= mem[rd_ptr];
    end
endmodule

// Power-of-two DEPTH >= 2. Both resets must be asserted for a FIFO flush.
// Hold traffic until both domains have initialized. Single-sided live reset
// is unsupported. rd_valid identifies registered data from an accepted read;
// this is a request/read-response interface, not first-word-fall-through.
module async_fifo #(
    parameter WIDTH = 16,
    parameter DEPTH = 8
) (
    input  logic wr_clk, wr_rst, wr_en,
    input  logic [WIDTH-1:0] wr_data_in,
    output logic wr_full,
    input  logic rd_clk, rd_rst, rd_en,
    output logic [WIDTH-1:0] rd_data_out,
    output logic rd_valid,
    output logic rd_empty
);
    localparam ADDR_WIDTH = $clog2(DEPTH);
    localparam PTR_WIDTH = ADDR_WIDTH + 1;
    // XOR the top two Gray bits; works for DEPTH=2 without negative slices.
    localparam [PTR_WIDTH-1:0] PTR_ONE = {{(PTR_WIDTH-1){1'b0}}, 1'b1};
    localparam [PTR_WIDTH-1:0] FULL_MASK = (PTR_ONE << ADDR_WIDTH) | (PTR_ONE << (ADDR_WIDTH-1));

    logic [PTR_WIDTH-1:0] wr_bin, rd_bin, wr_bin_next, rd_bin_next;
    logic [PTR_WIDTH-1:0] wr_gray_local, rd_gray_local;
    logic [PTR_WIDTH-1:0] wr_gray_next, rd_gray_next;
    logic [PTR_WIDTH-1:0] wr_gray_sync, rd_gray_sync;
    logic wr_accept, rd_accept, wr_full_next, rd_empty_next;

    assign wr_accept = wr_en && !wr_full && !wr_rst;
    assign rd_accept = rd_en && !rd_empty && !rd_rst;
    assign wr_bin_next = wr_bin + {{(PTR_WIDTH-1){1'b0}}, wr_accept};
    assign rd_bin_next = rd_bin + {{(PTR_WIDTH-1){1'b0}}, rd_accept};

    bin2gray #(.WIDTH(PTR_WIDTH)) wgray (.bin_in(wr_bin_next), .gray_out(wr_gray_next));
    bin2gray #(.WIDTH(PTR_WIDTH)) rgray (.bin_in(rd_bin_next), .gray_out(rd_gray_next));

    twoflopsync #(.WIDTH(PTR_WIDTH)) wsync (
        .clk(rd_clk), .rst(rd_rst), .d_in(wr_gray_local), .d_out(wr_gray_sync)
    );
    twoflopsync #(.WIDTH(PTR_WIDTH)) rsync (
        .clk(wr_clk), .rst(wr_rst), .d_in(rd_gray_local), .d_out(rd_gray_sync)
    );

    assign rd_empty_next = (rd_gray_next == wr_gray_sync);
    assign wr_full_next = (wr_gray_next == (rd_gray_sync ^ FULL_MASK));

    always_ff @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            rd_bin <= '0;
            rd_gray_local <= '0;
            rd_empty <= 1'b1;
            rd_valid <= 1'b0;
        end else begin
            rd_bin <= rd_bin_next;
            rd_gray_local <= rd_gray_next;
            rd_empty <= rd_empty_next;
            rd_valid <= rd_accept;
        end
    end

    always_ff @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wr_bin <= '0;
            wr_gray_local <= '0;
            wr_full <= 1'b0;
        end else begin
            wr_bin <= wr_bin_next;
            wr_gray_local <= wr_gray_next;
            wr_full <= wr_full_next;
        end
    end

    fifo #(.WIDTH(WIDTH), .DEPTH(DEPTH)) f (
        .wr_en(wr_accept), .wr_clk(wr_clk), .wr_full(wr_full),
        .rd_en(rd_accept), .rd_clk(rd_clk),
        .data_in(wr_data_in), .data_out(rd_data_out),
        .wr_ptr(wr_bin[ADDR_WIDTH-1:0]), .rd_ptr(rd_bin[ADDR_WIDTH-1:0])
    );
endmodule
