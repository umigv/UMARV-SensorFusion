// VLP-16 Ethernet frames in  ->  XYZ float32 point cloud UDP frames out.
//
//   MAC RX -> vlp16_udp_rx -> stage_t (existing polar->cartesian transform)
//          -> pc_fix2float x3 -> point FIFO -> pc_udp_tx -> MAC TX
//
// Everything runs in one clock domain (the MAC's logic clock).
`include "sys_defs.svh"

module lidar_pc_core #(
    parameter logic [15:0] LIDAR_PORT = 16'd0,       // 0 = any UDP port
    // 1: REP-103 / velodyne_driver frame (x forward, y left, z up)
    // 0: raw stage_t frame              (y forward, x right, z up)
    parameter logic        ROS_FRAME  = 1'b1,
    parameter logic [47:0] LOCAL_MAC  = 48'h02_00_00_00_00_01,
    parameter logic [31:0] LOCAL_IP   = {8'd192, 8'd168, 8'd1, 8'd50},
    parameter logic [47:0] DEST_MAC   = 48'hFF_FF_FF_FF_FF_FF,
    parameter logic [31:0] DEST_IP    = {8'd255, 8'd255, 8'd255, 8'd255},
    parameter logic [15:0] SRC_PORT   = 16'd2370,
    parameter logic [15:0] DEST_PORT  = 16'd2370
) (
    input  logic       clk,
    input  logic       rst,

    // Ethernet frames from MAC
    input  logic [7:0] rx_tdata,
    input  logic       rx_tvalid,
    output logic       rx_tready,
    input  logic       rx_tlast,

    // Ethernet frames to MAC
    output logic [7:0] tx_tdata,
    output logic       tx_tvalid,
    input  logic       tx_tready,
    output logic       tx_tlast,

    // Status pulses
    output logic       stat_lidar_pkt,
    output logic       stat_other_pkt,
    output logic       stat_pc_pkt,
    output logic       stat_fifo_overflow
);

    localparam int FIFO_DEPTH = 512;
    localparam int LEVEL_W    = $clog2(FIFO_DEPTH) + 1;
    localparam int T_LAT      = 2;                   // stage_t latency
    localparam int F_LAT      = 3;                   // pc_fix2float latency
    localparam int LAT        = T_LAT + F_LAT;

    // ---- front end -----------------------------------------------------------
    logic        pt_valid;
    logic [15:0] pt_distance, pt_azimuth;
    logic [3:0]  pt_laser_id;
    logic        pause;

    vlp16_udp_rx #(
        .LIDAR_PORT(LIDAR_PORT),
        .DROP_ZERO(1'b1)
    ) rx_inst (
        .clk, .rst,
        .s_tdata(rx_tdata), .s_tvalid(rx_tvalid), .s_tready(rx_tready), .s_tlast(rx_tlast),
        .pause,
        .pt_valid, .pt_distance, .pt_azimuth, .pt_laser_id,
        .pt_reflectivity(),
        .pkt_accepted(stat_lidar_pkt),
        .pkt_ignored(stat_other_pkt)
    );

    // ---- coordinate transform (unchanged stage_t, lane 0 only) --------------
    // stage_t works on signed 16-bit distances.  Raw VLP-16 distances above
    // 32767 (> 65.5 m) are halved here and doubled again in the float
    // conversion so they do not wrap negative.
    INGRESS_PACKET   ingress;
    TRANSFORM_PACKET xyz;
    wire             far = pt_distance[15];

    always_comb begin
        ingress             = '0;
        ingress.valid       = {1'b0, pt_valid};
        ingress.distance[0] = far ? DATA'({1'b0, pt_distance[15:1]}) : DATA'(pt_distance);
        ingress.azimuth[0]  = DATA'(pt_azimuth);
        ingress.laser_id[0] = pt_laser_id;
    end

    stage_t transform_inst (
        .clk,
        .rst_n(!rst),
        .stall(1'b0),
        .data_in(ingress),
        .t2f_pipe(xyz)
    );

    // side-band delay lines
    logic [T_LAT-1:0]       far_dly;
    logic [F_LAT-1:0]       valid_dly;
    logic [LAT-1:0][15:0]   az_dly;

    always_ff @(posedge clk) begin
        far_dly   <= {far_dly[T_LAT-2:0], far};
        valid_dly <= {valid_dly[F_LAT-2:0], xyz.valid[0]};
        az_dly    <= {az_dly[LAT-2:0], pt_azimuth};
        if (rst) valid_dly <= '0;
    end
    wire far_t = far_dly[T_LAT-1];

    // ---- to float32 metres, in the output frame -----------------------------
    DATA  src_x, src_y;
    logic neg_y;
    always_comb begin
        if (ROS_FRAME) begin
            src_x = xyz.y[0];      // forward
            src_y = xyz.x[0];      // left = -right
            neg_y = 1'b1;
        end else begin
            src_x = xyz.x[0];
            src_y = xyz.y[0];
            neg_y = 1'b0;
        end
    end

    logic [31:0] fx, fy, fz;
    pc_fix2float f2f_x (.clk, .din(src_x),    .x2(far_t), .negate(1'b0),  .fout(fx));
    pc_fix2float f2f_y (.clk, .din(src_y),    .x2(far_t), .negate(neg_y), .fout(fy));
    pc_fix2float f2f_z (.clk, .din(xyz.z[0]), .x2(far_t), .negate(1'b0),  .fout(fz));

    // ---- point FIFO ----------------------------------------------------------
    logic [LEVEL_W-1:0] fifo_level;
    logic               fifo_rd;
    logic [111:0]       fifo_rd_data;
    wire                fifo_wr = valid_dly[F_LAT-1];

    pc_point_fifo #(
        .WIDTH(112),
        .DEPTH(FIFO_DEPTH)
    ) fifo_inst (
        .clk, .rst,
        .wr_en(fifo_wr),
        .wr_data({az_dly[LAT-1], fz, fy, fx}),
        .rd_en(fifo_rd),
        .rd_data(fifo_rd_data),
        .level(fifo_level),
        .overflow(stat_fifo_overflow)
    );

    // Stop pulling bytes from the MAC while the FIFO is nearly full.  Margin
    // covers points already in the parser / transform / float pipeline.
    always_ff @(posedge clk)
        pause <= (fifo_level >= LEVEL_W'(FIFO_DEPTH - 16));

    // ---- back end ------------------------------------------------------------
    pc_udp_tx #(
        .LEVEL_W(LEVEL_W),
        .SRC_MAC(LOCAL_MAC),
        .DST_MAC(DEST_MAC),
        .SRC_IP(LOCAL_IP),
        .DST_IP(DEST_IP),
        .SRC_PORT(SRC_PORT),
        .DST_PORT(DEST_PORT)
    ) tx_inst (
        .clk, .rst,
        .fifo_level,
        .fifo_wr,
        .fifo_rd,
        .fifo_data(fifo_rd_data),
        .m_tdata(tx_tdata), .m_tvalid(tx_tvalid), .m_tready(tx_tready), .m_tlast(tx_tlast),
        .pkt_sent(stat_pc_pkt)
    );

endmodule
