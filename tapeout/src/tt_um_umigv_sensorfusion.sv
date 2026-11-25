/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tiny Tapeout top level for the sensor fusion chip.
 *
 *   timestamper   400 Hz beat, VN-300 sync and parser, dead-wheel encoder reader
 *   lidar_top     lidar packets in over MII, motion-corrected points out over MII
 *   spi_regs      host SPI port: every setting, the event counters and the latest measurements
 *
 * The Kalman filter (ukf_top) is only instantiated when WITH_UKF is defined, because it is not
 * written yet. Without it, or with bit 7 of the control register clear, the state the lidar
 * block uses (turn rate, speed, and the position and heading it copies into its packets) is
 * whatever the host writes into the register file.
 *
 * Pins
 *   ui[0]  VN-300 serial data            uo[0]  VN-300 SyncIn pulse
 *   ui[1]  encoder SPI MISO              uo[1]  encoder SPI CS_N
 *   ui[2]  host SPI SCK                  uo[2]  encoder SPI SCK
 *   ui[3]  host SPI CS_N                 uo[3]  encoder SPI MOSI
 *   ui[4]  host SPI MOSI                 uo[4]  measurements-ready pulse
 *   ui[5]  lidar MII RX_DV               uo[5]  host SPI MISO
 *   ui[6]  host-side MII TX_CLK          uo[6]  spare (0)
 *   ui[7]  lidar MII RX_CLK              uo[7]  host-side MII TX_EN
 *   uio[3:0]  lidar MII RXD (inputs)     uio[7:4]  host-side MII TXD (outputs)
 */

module tt_um_umigv_sensorfusion (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // 50 MHz
    input  wire       rst_n     // reset_n - low to reset
);

    // Reset: asserts immediately, releases on a clock edge
    logic [1:0] rst_sync;
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) rst_sync <= 2'b00;
        else       rst_sync <= {rst_sync[0], 1'b1};
    end
    logic rst_n_s;
    assign rst_n_s = rst_sync[1];

    logic        state_valid, spi_rd;
    logic [6:0]  spi_raddr;
    logic [31:0] spi_rword;
    logic [31:0] state_ts, state_x, state_y, state_theta, state_omega, state_v;

    // ---- settings and status, through the host SPI port ---------------------------------
    logic        ts_enable, lidar_enable, lidar_bypass, scale_4mm, corr_rot_en, corr_shift_en, dw_check_turns;
    logic        state_from_filter, bus_we;
    logic [6:0]  bus_addr;
    logic [7:0]  bus_wdata;
    logic [1:0]  lidar_model;
    logic [15:0] period_us, deadline_us, lidar_port, src_port, dst_port;
    logic signed [15:0] yaw_offset;
    logic [47:0] src_mac, dst_mac;
    logic [31:0] src_ip, dst_ip;
    logic        host_state_valid;
    logic [31:0] host_state_ts, host_state_x, host_state_y, host_state_theta, host_state_omega, host_state_v;
    logic [6:0]  lidar_stat;
    logic        host_miso;

    // ---- measurements from the timestamper ----------------------------------------------
    logic vn_sync, dw_cs_n, dw_sck, dw_mosi, ukf_pulse;
    logic        [31:0] beat_ts;
    logic               vn_valid, dw_valid;
    logic        [31:0] vn_yaw_f32, vn_rate_f32;
    logic        [63:0] vn_lat_f64, vn_lon_f64;
    logic signed [27:0] dw_delta;
    logic        [39:0] time_us;

    spi_regs u_regs (
        .clk              (clk),
        .rst_n            (rst_n_s),
        .spi_sck          (ui_in[2]),
        .spi_cs_n         (ui_in[3]),
        .spi_mosi         (ui_in[4]),
        .spi_miso         (host_miso),
        .ts_enable        (ts_enable),
        .lidar_enable     (lidar_enable),
        .lidar_bypass     (lidar_bypass),
        .scale_4mm        (scale_4mm),
        .corr_rot_en      (corr_rot_en),
        .corr_shift_en    (corr_shift_en),
        .dw_check_turns   (dw_check_turns),
        .state_from_filter(state_from_filter),
        .bus_we           (bus_we),
        .bus_addr         (bus_addr),
        .bus_wdata        (bus_wdata),
        .lidar_model      (lidar_model),
        .period_us        (period_us),
        .deadline_us      (deadline_us),
        .lidar_port       (lidar_port),
        .yaw_offset       (yaw_offset),
        .src_mac          (src_mac),
        .dst_mac          (dst_mac),
        .src_ip           (src_ip),
        .dst_ip           (dst_ip),
        .src_port         (src_port),
        .dst_port         (dst_port),
        .spi_rd           (spi_rd),
        .spi_raddr        (spi_raddr),
        .spi_rword        (spi_rword),
        .host_state_valid (host_state_valid),
        .host_state_ts    (host_state_ts),
        .host_state_x     (host_state_x),
        .host_state_y     (host_state_y),
        .host_state_theta (host_state_theta),
        .host_state_omega (host_state_omega),
        .host_state_v     (host_state_v),
        .stat_pulse       (lidar_stat),
        .beat_ts          (beat_ts),
        .vn_valid         (vn_valid),
        .dw_valid         (dw_valid),
        .vn_yaw_f32       (vn_yaw_f32),
        .vn_rate_f32      (vn_rate_f32),
        .vn_lat_f64       (vn_lat_f64),
        .vn_lon_f64       (vn_lon_f64),
        .dw_delta         (dw_delta)
    );

    timestamper u_timestamper (
        .clk            (clk),
        .rst_n          (rst_n_s),
        .enable         (ts_enable),
        .incr           (32'd0),        // unused
        .period_us      (period_us),
        .spi_half_div   (8'd0),         // unused
        .deadline_us    (deadline_us),
        .dw_check_turns (dw_check_turns),
        .vn_sync        (vn_sync),
        .vn_rx          (ui_in[0]),
        .cs_n           (dw_cs_n),
        .sck            (dw_sck),
        .mosi           (dw_mosi),
        .miso           (ui_in[1]),
        .ukf_pulse      (ukf_pulse),
        .ukf_beat_ts    (beat_ts),
        .ukf_vn_valid   (vn_valid),
        .ukf_vn_yaw_f32 (vn_yaw_f32),
        .ukf_vn_rate_f32(vn_rate_f32),
        .ukf_vn_lat_f64 (vn_lat_f64),
        .ukf_vn_lon_f64 (vn_lon_f64),
        .ukf_dw_valid   (dw_valid),
        .ukf_dw_delta   (dw_delta),
        .time_us        (time_us)
    );

    // ---- Kalman filter, and the choice of where the lidar block's state comes from --------

`ifdef WITH_UKF
    // The filter's six output words are also where the host-written state lives while the
    // filter is off, so there is one source and no selector for the values.
    logic        kf_valid;

    ukf_top u_ukf (
        .clk         (clk),
        .rst_n       (rst_n_s),
        .enable      (state_from_filter),
        .meas_pulse  (ukf_pulse),
        .beat_ts     (beat_ts),
        .vn_valid    (vn_valid),
        .vn_yaw_f32  (vn_yaw_f32),
        .vn_rate_f32 (vn_rate_f32),
        .vn_lat_f64  (vn_lat_f64),
        .vn_lon_f64  (vn_lon_f64),
        .dw_valid    (dw_valid),
        .dw_delta    (dw_delta),
        .cfg_we      (bus_we),
        .cfg_addr    (bus_addr),
        .cfg_wdata   (bus_wdata),
        .spi_rd      (spi_rd),
        .spi_raddr   (spi_raddr),
        .spi_rword   (spi_rword),
        .state_valid (kf_valid),
        .state_ts    (state_ts),
        .state_x     (state_x),
        .state_y     (state_y),
        .state_theta (state_theta),
        .state_omega (state_omega),
        .state_v     (state_v)
    );

    assign state_valid = state_from_filter ? kf_valid : host_state_valid;
    wire _unused_hs = &{host_state_ts, host_state_x, host_state_y, host_state_theta, host_state_omega, host_state_v, 1'b0};
`else
    assign spi_rword = 32'd0;
    wire _unused_spi = &{spi_rd, spi_raddr, 1'b0};
    assign state_valid = host_state_valid;
    assign state_ts    = host_state_ts;
    assign state_x     = host_state_x;
    assign state_y     = host_state_y;
    assign state_theta = host_state_theta;
    assign state_omega = host_state_omega;
    assign state_v     = host_state_v;
    wire _unused_kf = &{state_from_filter, bus_we, bus_addr, bus_wdata, 1'b0};
`endif

    // ---- lidar block ------------------------------------------------------------------------
    logic       mii_tx_en;
    logic [3:0] mii_txd;

    lidar_top u_lidar (
        .clk           (clk),
        .rst_n         (rst_n_s),
        .enable        (lidar_enable),
        .bypass        (lidar_bypass),
        .lidar_port    (lidar_port),
        .scale_4mm     (scale_4mm),
        .lidar_model   (lidar_model),
        .yaw_offset    (yaw_offset),
        .corr_rot_en   (corr_rot_en),
        .corr_shift_en (corr_shift_en),
        .src_mac       (src_mac),
        .dst_mac       (dst_mac),
        .src_ip        (src_ip),
        .dst_ip        (dst_ip),
        .src_port      (src_port),
        .dst_port      (dst_port),
        .mii_rx_clk    (ui_in[7]),
        .mii_rx_dv     (ui_in[5]),
        .mii_rxd       (uio_in[3:0]),
        .mii_tx_clk    (ui_in[6]),
        .mii_tx_en     (mii_tx_en),
        .mii_txd       (mii_txd),
        .state_valid   (state_valid),
        .state_ts      (state_ts),
        .state_x       (state_x),
        .state_y       (state_y),
        .state_theta   (state_theta),
        .state_omega   (state_omega),
        .state_v       (state_v),
        .time_us       (time_us[31:0]),
        .stat_lidar_pkt      (lidar_stat[0]),
        .stat_other_pkt      (lidar_stat[1]),
        .stat_rx_crc_bad     (lidar_stat[2]),
        .stat_frame_sent     (lidar_stat[3]),
        .stat_buf_overflow   (lidar_stat[4]),
        .stat_motion_overrun (lidar_stat[5]),
        .stat_tx_underrun    (lidar_stat[6])
    );

    // Pins out
    assign uo_out[0] = vn_sync;
    assign uo_out[1] = dw_cs_n;
    assign uo_out[2] = dw_sck;
    assign uo_out[3] = dw_mosi;
    assign uo_out[4] = ukf_pulse;
    assign uo_out[5] = host_miso;
    assign uo_out[6] = 1'b0;
    assign uo_out[7] = mii_tx_en;

    assign uio_out = {mii_txd, 4'h0};
    assign uio_oe  = 8'hF0;

    // List all unused inputs to prevent warnings
    wire _unused = &{ena, uio_in[7:4], time_us[39:32], 1'b0};

endmodule
