// Lidar block top level: VLP-16 packets in over Ethernet, motion-corrected x, y, z points out.
//
// What "motion-corrected" means here (agreed with the navigation team):
//   Each sweep of the lidar is put into ONE coordinate frame: the robot's own frame at the moment
//   the sweep started. Every point is turned back and moved back by however much the robot has
//   turned and travelled since that moment. Points are NOT rotated or moved into the world frame.
//   The filter's state is sent alongside, unchanged, so the receiver can place the sweep on the
//   map itself.
//
//   The sweep's START is the reference, not its end, because points are sent as they arrive and
//   the robot's motion up to a point is known when that point is sent; its motion after is not.
//
//   PHY -> [1] MII receive, frame check, clock crossing  (mii_recv, async_fifo)
//       -> [2] header check + packet parser            (lidar_vlp16_rx, v16PacketParser)
//       -> [3] buffer of raw distances, sweep detection (lidar_point_buffer)
//       -> [4] transform to x, y, z                     (lidar_xyz, lidar_laser_table)
//               <- motion since the start of the sweep  (lidar_motion)
//               <- sine and cosine                      (lidar_cordic)
//       -> [5] frame builder, clock crossing, frame check, MII transmit -> PHY
//                                                       (lidar_frame_tx, async_fifo, mii_send)
//          bypass: received frames go to the transmitter as they are
//
// The filter is not written yet. Until it is, tie state_valid to 0: no correction is applied,
// the state fields of the outgoing header are whatever the state_* pins hold, and bit 0 of the
// header's flags tells the receiver that they are not valid.
//
// Clocks: everything runs on clk except the MII pins. mii_rx_* are in step with mii_rx_clk and
// mii_tx_* with mii_tx_clk, both 25 MHz from the PHYs; stages 1 and 5 cross into and out of clk.
// Only the MII receiver, the MII transmitter and one side of each FIFO run on the PHY clocks.
//
// Written in the plain SystemVerilog subset the Tiny Tapeout tools accept: no structs, enums,
// packages, size casts or 'return'.
module lidar_top (
    input  logic        clk,            // chip clock
    input  logic        rst_n,          // active low, released in step with clk

    // ---- Settings, from the register block --------------------------------------------
    input  logic        enable,         // 0: block held in reset, nothing transmitted
    input  logic        bypass,         // 1: forward received frames unchanged (stage 9)
    input  logic [15:0] lidar_port,     // UDP port the lidar sends to. 0 = any
    input  logic        scale_4mm,      // output step: 0 = 2 mm per count (+/-65 m), 1 = 4 mm (+/-131 m)
    input  logic [1:0]  lidar_model,    // 0 VLP-16 / Puck LITE, 1 Puck Hi-Res, 2 VLP-32C, 3 HDL-32E
    input  logic signed [15:0] yaw_offset,  // lidar mounting angle, 0.01 degree, added to every azimuth
    input  logic        corr_rot_en,    // 1: correct for the robot turning during the sweep
    input  logic        corr_shift_en,  // 1: correct for the robot moving during the sweep
    input  logic [47:0] src_mac,        // addresses for the outgoing frames
    input  logic [47:0] dst_mac,
    input  logic [31:0] src_ip,
    input  logic [31:0] dst_ip,
    input  logic [15:0] src_port,
    input  logic [15:0] dst_port,

    // ---- MII receive, from the PHY on the lidar side -----------------------------------
    input  logic        mii_rx_clk,     // 25 MHz, from the PHY
    input  logic        mii_rx_dv,
    input  logic [3:0]  mii_rxd,

    // ---- MII transmit, to the PHY on the host side -------------------------------------
    input  logic        mii_tx_clk,     // 25 MHz, from the PHY
    output logic        mii_tx_en,
    output logic [3:0]  mii_txd,

    // ---- State, from the Kalman filter --------------------------------------------------
    // IEEE single-precision bit patterns (proposed; to be agreed with the filter's authors).
    // All change together on state_valid and hold until the next one.
    //   state_v and state_omega drive the correction (stage 4 converts them to fixed point).
    //   All of them are copied unchanged into the header of every outgoing frame.
    input  logic        state_valid,    // one clock: the values below are a new state
    input  logic [31:0] state_ts,       // time_us of the beat this state belongs to
    input  logic [31:0] state_x,        // fp32, metres
    input  logic [31:0] state_y,        // fp32, metres
    input  logic [31:0] state_theta,    // fp32, radians
    input  logic [31:0] state_omega,    // fp32, radians per second
    input  logic [31:0] state_v,        // fp32, metres per second, forward

    // ---- Time, from the timestamper ----------------------------------------------------
    input  logic [31:0] time_us,

    // ---- Status, one-clock pulses for the register block's counters --------------------
    output logic        stat_lidar_pkt,     // a complete lidar data packet was parsed
    output logic        stat_other_pkt,     // a frame that was not a lidar data packet
    output logic        stat_rx_crc_bad,    // a received frame failed its frame check
    output logic        stat_frame_sent,    // an outgoing frame was completed
    output logic        stat_buf_overflow,  // a data block was discarded because the buffer was full
    output logic        stat_motion_overrun,// a firing was asked for before the previous one was worked out
    output logic        stat_tx_underrun    // an outgoing frame was cut short (or, in bypass, lost bytes)
);

    // Reset. The block is held in reset by the chip reset or by enable = 0. That request is
    // registered once on clk, then each clock domain gets its own copy that asserts at once
    // and releases two of its own clock edges later.
    logic lidar_off;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) lidar_off <= 1'b1;
        else        lidar_off <= !enable;
    end

    logic rst;                      // clk domain, active high
    reset_release u_rst_core (.clk(clk), .arst(lidar_off), .rst(rst));

    logic rx_rst;                   // mii_rx_clk domain, active high
    reset_release u_rst_rx (.clk(mii_rx_clk), .arst(lidar_off), .rst(rx_rst));

    logic tx_rst;                   // mii_tx_clk domain, active high
    reset_release u_rst_tx (.clk(mii_tx_clk), .arst(lidar_off), .rst(tx_rst));

    // =====================================================================================
    // [1] MII receive, frame check (CRC-32) and clock crossing
    //
    //   mii_rx_clk domain: mii_recv turns nibbles into bytes, removes the preamble, checks the
    //                      frame's CRC and marks the last byte.
    //   crossing:          an 8-entry asynchronous FIFO carries {bad frame, last, byte}. Bytes
    //                      arrive every 80 ns and are read every 20 ns, so it never holds more
    //                      than a couple of entries.
    //   clk domain:        the FIFO is read whenever it is not empty.
    //
    // The frame's 4 check bytes are passed on as its last 4 bytes; stage 2 ignores them.
    // =====================================================================================
    logic [7:0] mii_data;
    logic       mii_valid, mii_sop, mii_eop, mii_error;

    mii_recv u_mii_recv (
        .RX_CLK   (mii_rx_clk),
        .RX_RST   (rx_rst),
        .RXD      (mii_rxd),
        .RX_DV    (mii_rx_dv),
        .RX_ER    (1'b0),               // no pin for it
        .rx_data  (mii_data),
        .rx_valid (mii_valid),
        .rx_sop   (mii_sop),
        .rx_eop   (mii_eop),
        .rx_error (mii_error)
    );

    logic [9:0] rx_fifo_out;
    logic       rx_fifo_valid, rx_fifo_empty, rx_fifo_full;

    async_fifo #(
        .WIDTH (10),
        .DEPTH (8)
    ) u_rx_fifo (
        .wr_clk      (mii_rx_clk),
        .wr_rst      (rx_rst),
        .wr_en       (mii_valid),
        .wr_data_in  ({mii_error, mii_eop, mii_data}),
        .wr_full     (rx_fifo_full),    // cannot happen while clk is faster than the byte rate
        .rd_clk      (clk),
        .rd_rst      (rst),
        .rd_en       (!rx_fifo_empty),
        .rd_data_out (rx_fifo_out),
        .rd_valid    (rx_fifo_valid),
        .rd_empty    (rx_fifo_empty)
    );

    logic [7:0] rx_tdata;
    logic       rx_tvalid;
    logic       rx_tlast;
    logic       rx_crc_bad;

    assign rx_tdata   = rx_fifo_out[7:0];
    assign rx_tvalid  = rx_fifo_valid;
    assign rx_tlast   = rx_fifo_valid && rx_fifo_out[8];
    assign rx_crc_bad = rx_fifo_valid && rx_fifo_out[8] && rx_fifo_out[9];

    assign stat_rx_crc_bad = rx_crc_bad;

    // =====================================================================================
    // [2] Header check and packet parser
    // =====================================================================================
    logic        pt_valid;
    logic [15:0] pt_distance;       // raw, 2 or 4 mm per count depending on the lidar
    logic [15:0] pt_azimuth;        // raw, 0.01 degree per count, of the block's first firing
    logic [3:0]  pt_laser_id;       // not used: later stages count positions within a block
    logic [7:0]  pt_reflectivity;
    logic        rx_tready_unused;

    lidar_vlp16_rx u_rx (
        .clk             (clk),
        .rst             (rst),
        .lidar_port      (lidar_port),
        // Zero-distance returns are kept: later stages work out each point's laser and
        // firing from its position in the packet, so no position may go missing.
        .drop_zero       (1'b0),
        .s_tdata         (rx_tdata),
        .s_tvalid        (rx_tvalid),
        .s_tready        (rx_tready_unused),
        .s_tlast         (rx_tlast),
        .pause           (1'b0),            // the incoming link cannot be paused
        .pt_valid        (pt_valid),
        .pt_distance     (pt_distance),
        .pt_azimuth      (pt_azimuth),
        .pt_laser_id     (pt_laser_id),
        .pt_reflectivity (pt_reflectivity),
        .pkt_accepted    (stat_lidar_pkt),
        .pkt_ignored     (stat_other_pkt)
    );

    // =====================================================================================
    // [3] Buffer of raw distances, and sweep detection
    //
    // Every model in the family sends 12 data blocks of 32 returns per packet. The buffer keeps
    // whole blocks only, with each block's azimuth, and marks the first block of each sweep
    // (the one whose azimuth is smaller than the block before it).
    //
    // In bypass nothing is put into the buffer.
    // =====================================================================================
    // The parser's last returns can trail the end-of-packet pulse by a few clocks.
    logic [3:0] pkt_end_d;
    always_ff @(posedge clk) begin
        if (rst) pkt_end_d <= 4'd0;
        else     pkt_end_d <= {pkt_end_d[2:0], stat_lidar_pkt || stat_other_pkt};
    end

    logic        buf_sweep_start;
    logic        blk0_valid, blk0_sweep, blk1_valid, blk1_sweep, blk_pop, buf_rd_pop;
    logic [15:0] blk0_azimuth, blk1_azimuth, buf_rd_data;

    lidar_point_buffer u_buffer (
        .clk            (clk),
        .rst            (rst),
        .pt_valid       (pt_valid && !bypass),
        .pt_distance    (pt_distance),
        .pt_azimuth     (pt_azimuth),
        .pkt_end        (pkt_end_d[3]),
        .wr_sweep_start (buf_sweep_start),
        .overflow       (stat_buf_overflow),
        .blk0_valid     (blk0_valid),
        .blk0_azimuth   (blk0_azimuth),
        .blk0_sweep     (blk0_sweep),
        .blk1_valid     (blk1_valid),
        .blk1_azimuth   (blk1_azimuth),
        .blk1_sweep     (blk1_sweep),
        .blk_pop        (blk_pop),
        .rd_data        (buf_rd_data),
        .rd_pop         (buf_rd_pop)
    );

    // Chip time at which the newest sweep's first block arrived. This is the sweep's
    // reference moment, sent in the header as sweep_time_us.
    logic [31:0] sweep_time_us;
    always_ff @(posedge clk) begin
        if (rst)                  sweep_time_us <= 32'd0;
        else if (buf_sweep_start) sweep_time_us <= time_us;
    end

    // =====================================================================================
    // Laser geometry and per-model constants (lidar_laser_table)
    // =====================================================================================
    logic        [4:0]  tbl_laser;
    logic signed [15:0] tbl_sin_el, tbl_cos_el;
    logic signed [9:0]  tbl_az_offset;
    logic               model_lasers_32, model_dist_4mm;
    logic        [15:0] model_firing_ns;
    logic        [1:0]  id_model_unused;
    logic               id_known_unused;

    lidar_laser_table u_laser_table (
        .model            (lidar_model),
        .laser            (tbl_laser),
        .sin_el           (tbl_sin_el),
        .cos_el           (tbl_cos_el),
        .az_offset        (tbl_az_offset),
        .lasers_32        (model_lasers_32),
        .dist_4mm         (model_dist_4mm),
        .firing_period_ns (model_firing_ns),
        .product_id       (8'h00),          // the model is a setting; it is not read from the packet
        .model_from_id    (id_model_unused),
        .model_id_known   (id_known_unused)
    );

    // =====================================================================================
    // [4] Transform to x, y, z
    //
    // lidar_xyz takes blocks from the buffer. For each firing it asks lidar_motion how far the
    // robot has turned and moved since the sweep started; lidar_motion runs the CORDIC on the
    // corrected angle. Firings are asked for as their points leave the buffer, not as they
    // arrive, so points still waiting from one sweep never get the next sweep's totals.
    // =====================================================================================
    logic               firing_start, firing_second, sweep_start;
    logic        [15:0] firing_azimuth;

    logic               cordic_start;
    logic signed [17:0] cordic_theta;       // radians, 15 fractional bits
    logic signed [17:0] cordic_sin, cordic_cos;
    logic               cordic_busy, cordic_done;
    logic               cordic_overflow_unused, cordic_error_unused;

    logic               fire_done, fire_sweep_start, motion_state_seen;
    logic signed [15:0] fire_sin0, fire_cos0, fire_sin1, fire_cos1, fire_sin2, fire_cos2, fire_sin3, fire_cos3;
    logic signed [15:0] shift_x, shift_y;   // output counts, in the sweep-start frame

    lidar_motion u_motion (
        .clk              (clk),
        .rst              (rst),
        .corr_rot_en      (corr_rot_en),
        .corr_shift_en    (corr_shift_en),
        .scale_4mm        (scale_4mm),
        .four_offsets     (lidar_model == 2'd2),
        .firing_period_ns (model_firing_ns),
        .yaw_offset       (yaw_offset),
        .state_valid      (state_valid),
        .state_omega      (state_omega),
        .state_v          (state_v),
        .firing_start     (firing_start),
        .firing_second    (firing_second),
        .sweep_start      (sweep_start),
        .azimuth          (firing_azimuth),
        .cordic_start     (cordic_start),
        .cordic_theta     (cordic_theta),
        .cordic_done      (cordic_done),
        .cordic_sin       (cordic_sin),
        .cordic_cos       (cordic_cos),
        .fire_done        (fire_done),
        .fire_sweep_start (fire_sweep_start),
        .sin0 (fire_sin0), .cos0 (fire_cos0), .sin1 (fire_sin1), .cos1 (fire_cos1),
        .sin2 (fire_sin2), .cos2 (fire_cos2), .sin3 (fire_sin3), .cos3 (fire_cos3),
        .shift_x          (shift_x),
        .shift_y          (shift_y),
        .state_seen       (motion_state_seen),
        .overrun          (stat_motion_overrun)
    );

    lidar_cordic u_cordic (
        .clk       (clk),
        .resetn    (!rst),
        .start     (cordic_start),
        .mode      (2'd0),                  // rotate: sine and cosine
        .theta     (cordic_theta),
        .sin_theta (cordic_sin),
        .cos_theta (cordic_cos),
        .busy      (cordic_busy),
        .done      (cordic_done),
        .overflow  (cordic_overflow_unused),
        .error     (cordic_error_unused)
    );

    logic               xyz_blk_valid, xyz_blk_sweep, xyz_blk_take, xyz_pt_req, xyz_pt_done;
    logic        [15:0] xyz_blk_azimuth;
    logic signed [15:0] xyz_x, xyz_y, xyz_z;

    lidar_xyz u_xyz (
        .clk             (clk),
        .rst             (rst),
        .lasers_32       (model_lasers_32),
        .dist_4mm        (model_dist_4mm),
        .scale_4mm       (scale_4mm),
        .blk0_valid      (blk0_valid),
        .blk0_azimuth    (blk0_azimuth),
        .blk0_sweep      (blk0_sweep),
        .blk1_valid      (blk1_valid),
        .blk1_azimuth    (blk1_azimuth),
        .blk1_sweep      (blk1_sweep),
        .blk_pop         (blk_pop),
        .rd_data         (buf_rd_data),
        .rd_pop          (buf_rd_pop),
        .firing_start    (firing_start),
        .firing_second   (firing_second),
        .sweep_start     (sweep_start),
        .azimuth         (firing_azimuth),
        .fire_done       (fire_done),
        .m_sin0 (fire_sin0), .m_cos0 (fire_cos0), .m_sin1 (fire_sin1), .m_cos1 (fire_cos1),
        .m_sin2 (fire_sin2), .m_cos2 (fire_cos2), .m_sin3 (fire_sin3), .m_cos3 (fire_cos3),
        .m_shift_x       (shift_x),
        .m_shift_y       (shift_y),
        .tbl_laser       (tbl_laser),
        .tbl_sin_el      (tbl_sin_el),
        .tbl_cos_el      (tbl_cos_el),
        .tbl_az_offset   (tbl_az_offset),
        .out_blk_valid   (xyz_blk_valid),
        .out_blk_azimuth (xyz_blk_azimuth),
        .out_blk_sweep   (xyz_blk_sweep),
        .out_blk_take    (xyz_blk_take),
        .pt_req          (xyz_pt_req),
        .pt_done         (xyz_pt_done),
        .x               (xyz_x),
        .y               (xyz_y),
        .z               (xyz_z)
    );

    // =====================================================================================
    // [5] Frame builder, clock crossing, frame check (CRC-32) and MII transmit
    //
    //   clk domain:        lidar_frame_tx writes the frame's bytes, or in bypass the received
    //                      frame's bytes, as {bad, last, byte}.
    //   crossing:          a 16-entry asynchronous FIFO. The builder fills it faster than the
    //                      line empties it, so it stays full during a frame and covers short
    //                      waits in the builder.
    //   mii_tx_clk domain: mii_send adds the preamble and the checksum and drives the pins.
    //
    // The layout of the outgoing frame is in docs/lidar_output_format.md.
    // =====================================================================================
    logic       tx_wr_en, tx_wr_full, tx_dropped, frame_started_unused;
    logic [9:0] tx_wr_data;

    lidar_frame_tx u_frame_tx (
        .clk            (clk),
        .rst            (rst),
        .bypass         (bypass),
        .src_mac        (src_mac),
        .dst_mac        (dst_mac),
        .src_ip         (src_ip),
        .dst_ip         (dst_ip),
        .src_port       (src_port),
        .dst_port       (dst_port),
        .lidar_model    (lidar_model),
        .scale_4mm      (scale_4mm),
        .state_seen     (motion_state_seen),
        .state_ts       (state_ts),
        .state_x        (state_x),
        .state_y        (state_y),
        .state_theta    (state_theta),
        .state_omega    (state_omega),
        .state_v        (state_v),
        .arr_sweep_time (sweep_time_us),
        .flag_rx_bad    (rx_crc_bad),
        .flag_overflow  (stat_buf_overflow),
        .blk_valid      (xyz_blk_valid),
        .blk_azimuth    (xyz_blk_azimuth),
        .blk_sweep      (xyz_blk_sweep),
        .blk_take       (xyz_blk_take),
        .pt_req         (xyz_pt_req),
        .pt_done        (xyz_pt_done),
        .x              (xyz_x),
        .y              (xyz_y),
        .z              (xyz_z),
        .rx_tdata       (rx_tdata),
        .rx_tvalid      (rx_tvalid),
        .rx_tlast       (rx_tlast),
        .rx_bad         (rx_crc_bad),
        .wr_en          (tx_wr_en),
        .wr_data        (tx_wr_data),
        .wr_full        (tx_wr_full),
        .frame_started  (frame_started_unused),
        .tx_dropped     (tx_dropped)
    );

    logic       tx_rd_en, tx_rd_valid, tx_rd_empty;
    logic [9:0] tx_rd_data;

    async_fifo #(
        .WIDTH (10),
        .DEPTH (16)
    ) u_tx_fifo (
        .wr_clk      (clk),
        .wr_rst      (rst),
        .wr_en       (tx_wr_en),
        .wr_data_in  (tx_wr_data),
        .wr_full     (tx_wr_full),
        .rd_clk      (mii_tx_clk),
        .rd_rst      (tx_rst),
        .rd_en       (tx_rd_en),
        .rd_data_out (tx_rd_data),
        .rd_valid    (tx_rd_valid),
        .rd_empty    (tx_rd_empty)
    );

    logic tx_frame_sent, tx_underrun;       // mii_tx_clk domain

    mii_send u_mii_send (
        .TX_CLK     (mii_tx_clk),
        .TX_RST     (tx_rst),
        .rd_en      (tx_rd_en),
        .rd_data    (tx_rd_data),
        .rd_valid   (tx_rd_valid),
        .rd_empty   (tx_rd_empty),
        .TX_EN      (mii_tx_en),
        .TXD        (mii_txd),
        .frame_sent (tx_frame_sent),
        .underrun   (tx_underrun)
    );

    // The two status pulses come back to clk as level changes: a flop toggles on each pulse,
    // is synchronised with two flops, and a change is turned back into a pulse.
    logic sent_tgl, und_tgl;
    always_ff @(posedge mii_tx_clk or posedge tx_rst) begin
        if (tx_rst) begin
            sent_tgl <= 1'b0; und_tgl <= 1'b0;
        end else begin
            if (tx_frame_sent) sent_tgl <= !sent_tgl;
            if (tx_underrun)   und_tgl  <= !und_tgl;
        end
    end

    logic [2:0] sent_s, und_s;
    always_ff @(posedge clk) begin
        if (rst) begin
            sent_s <= 3'd0; und_s <= 3'd0;
        end else begin
            sent_s <= {sent_s[1:0], sent_tgl};
            und_s  <= {und_s[1:0],  und_tgl};
        end
    end
    assign stat_frame_sent  = sent_s[2] ^ sent_s[1];
    assign stat_tx_underrun = (und_s[2] ^ und_s[1]) || tx_dropped;

    wire _unused = &{mii_sop, rx_fifo_full, rx_tready_unused, pt_laser_id, pt_reflectivity,
                     id_model_unused, id_known_unused,
                     cordic_busy, cordic_overflow_unused, cordic_error_unused,
                     fire_sweep_start, frame_started_unused, 1'b0};

endmodule
