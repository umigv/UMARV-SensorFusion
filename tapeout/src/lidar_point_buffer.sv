// Lidar stage 6: buffer of raw distances.
//
// Returns arrive from the parser in bursts, twice as fast as they can be sent on. This block
// holds the raw 16-bit distances until the transform takes them, and keeps them grouped the way
// the lidar sends them: in data blocks of 32 returns that share one azimuth.
//
// Write side
//   A data block is only made visible to the read side once all 32 of its returns have been
//   written ("committed"). If the packet ends part-way through a block, or there is no room for
//   a whole block, the returns already written for it are discarded. So the read side can always
//   count on exactly 32 returns per block.
//   A new sweep is flagged on the first block whose azimuth is smaller than the block before it.
//
// Read side
//   blk0_* describes the oldest committed block and blk1_* the one after it. rd_data is the
//   oldest unread distance; rd_pop steps to the next. blk_pop discards blk0 once its 32
//   distances have been read.
//
// Storage: 256 distances in latches (lidar_latch_mem) and 8 block records in flip-flops.
// A distance is readable two clocks after it is written; a block is only made visible after
// its last distance, and the transform takes far longer than that to ask for it.
module lidar_point_buffer (
    input  logic        clk,
    input  logic        rst,                // synchronous, active high

    // from the parser
    input  logic        pt_valid,           // one return
    input  logic [15:0] pt_distance,
    input  logic [15:0] pt_azimuth,         // azimuth of the block this return belongs to
    input  logic        pkt_end,            // one clock, after the last return of a packet

    output logic        wr_sweep_start,     // one clock: the block now arriving starts a new sweep
    output logic        overflow,           // one clock: a block was discarded for lack of room

    // to the transform
    output logic        blk0_valid,
    output logic [15:0] blk0_azimuth,
    output logic        blk0_sweep,         // this block is the first of a sweep
    output logic        blk1_valid,
    output logic [15:0] blk1_azimuth,
    output logic        blk1_sweep,
    input  logic        blk_pop,
    output logic [15:0] rd_data,
    input  logic        rd_pop
);

    // ---- distances ------------------------------------------------------------------------------
    logic [8:0]  wr_ptr;            // next write, including returns not yet committed
    logic [8:0]  wr_commit;         // everything before this is visible to the read side
    logic [8:0]  rd_ptr;
    wire  [8:0]  used = wr_ptr - rd_ptr;

    // ---- block records: {first of a sweep, azimuth} ---------------------------------------------
    logic [16:0] bmem [0:7];
    logic [3:0]  bwr, brd;
    wire  [3:0]  bcount = bwr - brd;
    wire  [2:0]  brd1   = brd[2:0] + 3'd1;

    assign blk0_valid   = (bcount != 4'd0);
    assign blk1_valid   = (bcount >= 4'd2);
    assign blk0_azimuth = bmem[brd[2:0]][15:0];
    assign blk0_sweep   = bmem[brd[2:0]][16];
    assign blk1_azimuth = bmem[brd1][15:0];
    assign blk1_sweep   = bmem[brd1][16];

    // ---- write-side block tracking -------------------------------------------------------------
    logic [4:0]  pt_cnt;            // returns of the current block seen so far
    logic        dropping;          // the current block is being discarded
    logic [15:0] prev_az, blk_az;
    logic        sweep_pending;     // a sweep boundary has been seen and not yet delivered

    wire first_of_block = pt_valid && (pt_cnt == 5'd0);
    wire last_of_block  = pt_valid && (pt_cnt == 5'd31);
    wire wrapped        = first_of_block && (pt_azimuth < prev_az);
    wire no_room        = (used > 9'd224) || (bcount == 4'd8);     // need 32 free distances and one block record
    wire drop_now       = first_of_block ? no_room : dropping;

    lidar_latch_mem u_mem (
        .clk   (clk),
        .we    (pt_valid && !drop_now && !rst),
        .waddr (wr_ptr[7:0]),
        .wdata (pt_distance),
        .raddr (rd_ptr[7:0]),
        .rdata (rd_data)
    );

    always_ff @(posedge clk) begin
        if (rst) begin
            wr_ptr <= 9'd0; wr_commit <= 9'd0; rd_ptr <= 9'd0;
            bwr <= 4'd0; brd <= 4'd0;
            pt_cnt <= 5'd0; dropping <= 1'b0;
            prev_az <= 16'd0; blk_az <= 16'd0; sweep_pending <= 1'b0;
            wr_sweep_start <= 1'b0; overflow <= 1'b0;
        end else begin
            wr_sweep_start <= 1'b0;
            overflow       <= 1'b0;

            if (rd_pop)  rd_ptr <= rd_ptr + 9'd1;
            if (blk_pop) brd    <= brd + 4'd1;

            if (pt_valid) begin
                if (first_of_block) begin
                    blk_az   <= pt_azimuth;
                    prev_az  <= pt_azimuth;
                    dropping <= no_room;
                    if (no_room) overflow <= 1'b1;
                    if (wrapped) begin sweep_pending <= 1'b1; wr_sweep_start <= 1'b1; end
                end
                if (!drop_now) begin
                    wr_ptr <= wr_ptr + 9'd1;
                end
                pt_cnt <= pt_cnt + 5'd1;                            // wraps from 31 to 0
                if (last_of_block && !drop_now) begin               // the whole block is in: commit it
                    wr_commit <= wr_ptr + 9'd1;
                    bmem[bwr[2:0]] <= {sweep_pending, blk_az};
                    bwr <= bwr + 4'd1;
                    sweep_pending <= 1'b0;
                end
            end else if (pkt_end && (pt_cnt != 5'd0)) begin
                // the packet stopped part-way through a block: forget the part that arrived
                wr_ptr <= wr_commit;
                pt_cnt <= 5'd0;
            end
        end
    end

endmodule
