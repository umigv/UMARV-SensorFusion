// Board-independent core: taxi 1G RGMII MAC + lidar point cloud pipeline.
//
// The Ethernet MAC is taxi_eth_mac_1g_rgmii_fifo from
// https://github.com/fpganinja/taxi (lib/taxi, CERN-OHL-S-2.0).
module fpga_core #(
    parameter logic        SIM        = 1'b0,
    parameter string       VENDOR     = "XILINX",
    parameter string       FAMILY     = "kintex7",
    // PHY reset is held for 2^PHY_RST_BITS clocks (2^21 @ 125 MHz = 16.8 ms)
    parameter int          PHY_RST_BITS = 21,

    parameter logic [15:0] LIDAR_PORT = 16'd0,
    parameter logic        ROS_FRAME  = 1'b1,
    parameter logic [47:0] LOCAL_MAC  = 48'h02_00_00_00_00_01,
    parameter logic [31:0] LOCAL_IP   = {8'd192, 8'd168, 8'd1, 8'd50},
    parameter logic [47:0] DEST_MAC   = 48'hFF_FF_FF_FF_FF_FF,
    parameter logic [31:0] DEST_IP    = {8'd255, 8'd255, 8'd255, 8'd255},
    parameter logic [15:0] SRC_PORT   = 16'd2370,
    parameter logic [15:0] DEST_PORT  = 16'd2370
) (
    // 125 MHz, 125 MHz +90 deg, synchronous reset
    input  wire logic       clk,
    input  wire logic       clk90,
    input  wire logic       rst,

    output wire logic [7:0] led,

    // Ethernet: 1000BASE-T RGMII
    input  wire logic       phy_rx_clk,
    input  wire logic [3:0] phy_rxd,
    input  wire logic       phy_rx_ctl,
    output wire logic       phy_tx_clk,
    output wire logic [3:0] phy_txd,
    output wire logic       phy_tx_ctl,
    output wire logic       phy_reset_n
);

    // ---- PHY reset ------------------------------------------------------------
    logic [PHY_RST_BITS:0] phy_rst_cnt;
    always_ff @(posedge clk) begin
        if (rst)                                phy_rst_cnt <= '0;
        else if (!phy_rst_cnt[PHY_RST_BITS])    phy_rst_cnt <= phy_rst_cnt + 1'b1;
    end
    assign phy_reset_n = phy_rst_cnt[PHY_RST_BITS];

    // ---- MAC ------------------------------------------------------------------
    taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) axis_tx();
    taxi_axis_if #(.DATA_W(8), .ID_W(8), .USER_EN(1), .USER_W(1)) axis_rx();
    taxi_axis_if #(.DATA_W(96), .KEEP_W(1), .ID_W(8)) axis_tx_cpl();
    taxi_axis_if #(.DATA_W(16), .KEEP_W(1), .KEEP_EN(0), .LAST_EN(0),
                   .USER_EN(1), .USER_W(1), .ID_EN(1), .ID_W(10)) axis_stat();

    assign axis_tx_cpl.tready = 1'b1;
    assign axis_stat.tready   = 1'b1;

    logic       rx_error_bad_fcs;
    logic       rx_fifo_overflow;
    logic       rx_fifo_good_frame;
    logic [1:0] link_speed;

    taxi_eth_mac_1g_rgmii_fifo #(
        .SIM(SIM),
        .VENDOR(VENDOR),
        .FAMILY(FAMILY),
        .USE_CLK90(1'b1),
        .STAT_EN(1'b0),
        .TX_FIFO_DEPTH(4096),
        .TX_FRAME_FIFO(1'b1),
        .RX_FIFO_DEPTH(4096),
        .RX_FRAME_FIFO(1'b1)
    ) eth_mac_inst (
        .gtx_clk(clk),
        .gtx_clk90(clk90),
        .gtx_rst(rst),
        .logic_clk(clk),
        .logic_rst(rst),

        .s_axis_tx(axis_tx),
        .m_axis_tx_cpl(axis_tx_cpl),
        .m_axis_rx(axis_rx),

        .rgmii_rx_clk(phy_rx_clk),
        .rgmii_rxd(phy_rxd),
        .rgmii_rx_ctl(phy_rx_ctl),
        .rgmii_tx_clk(phy_tx_clk),
        .rgmii_txd(phy_txd),
        .rgmii_tx_ctl(phy_tx_ctl),

        .stat_clk(clk),
        .stat_rst(rst),
        .m_axis_stat(axis_stat),

        .tx_error_underflow(),
        .tx_fifo_overflow(),
        .tx_fifo_bad_frame(),
        .tx_fifo_good_frame(),
        .rx_error_bad_frame(),
        .rx_error_bad_fcs(rx_error_bad_fcs),
        .rx_fifo_overflow(rx_fifo_overflow),
        .rx_fifo_bad_frame(),
        .rx_fifo_good_frame(rx_fifo_good_frame),
        .link_speed(link_speed),

        .cfg_tx_pad_en(1'b1),
        .cfg_tx_min_pkt_len(8'd60-1),
        .cfg_tx_max_pkt_len(16'd1518-1),
        .cfg_tx_ifg(8'd12),
        .cfg_tx_enable(1'b1),
        .cfg_rx_max_pkt_len(16'd1518-1),
        .cfg_rx_enable(1'b1)
    );

    // ---- lidar pipeline -------------------------------------------------------
    logic stat_lidar_pkt, stat_other_pkt, stat_pc_pkt, stat_fifo_overflow;

    assign axis_tx.tkeep = '1;
    assign axis_tx.tstrb = '1;
    assign axis_tx.tid   = '0;
    assign axis_tx.tdest = '0;
    assign axis_tx.tuser = '0;

    lidar_pc_core #(
        .LIDAR_PORT(LIDAR_PORT),
        .ROS_FRAME(ROS_FRAME),
        .LOCAL_MAC(LOCAL_MAC),
        .LOCAL_IP(LOCAL_IP),
        .DEST_MAC(DEST_MAC),
        .DEST_IP(DEST_IP),
        .SRC_PORT(SRC_PORT),
        .DEST_PORT(DEST_PORT)
    ) core_inst (
        .clk, .rst,
        .rx_tdata(axis_rx.tdata),
        .rx_tvalid(axis_rx.tvalid),
        .rx_tready(axis_rx.tready),
        .rx_tlast(axis_rx.tlast),
        .tx_tdata(axis_tx.tdata),
        .tx_tvalid(axis_tx.tvalid),
        .tx_tready(axis_tx.tready),
        .tx_tlast(axis_tx.tlast),
        .stat_lidar_pkt,
        .stat_other_pkt,
        .stat_pc_pkt,
        .stat_fifo_overflow
    );

    // ---- LEDs -----------------------------------------------------------------
    // Activity LEDs toggle per event (a lidar at 754 pkt/s shows as a dim glow);
    // error LEDs are sticky until reset.
    logic led_lidar, led_other, led_pc;
    logic err_fcs, err_rx_ovf, err_pt_ovf;
    logic [8:0]  lidar_div;
    logic [10:0] pc_div;

    always_ff @(posedge clk) begin
        if (stat_lidar_pkt) begin
            lidar_div <= lidar_div + 1'b1;
            if (&lidar_div) led_lidar <= !led_lidar;      // ~0.7 Hz blink at 754 pkt/s
        end
        if (stat_pc_pkt) begin
            pc_div <= pc_div + 1'b1;
            if (&pc_div) led_pc <= !led_pc;
        end
        if (stat_other_pkt)     led_other  <= !led_other;
        if (rx_error_bad_fcs)   err_fcs    <= 1'b1;
        if (rx_fifo_overflow)   err_rx_ovf <= 1'b1;
        if (stat_fifo_overflow) err_pt_ovf <= 1'b1;
        if (rst) begin
            led_lidar  <= 1'b0;
            led_other  <= 1'b0;
            led_pc     <= 1'b0;
            lidar_div  <= '0;
            pc_div     <= '0;
            err_fcs    <= 1'b0;
            err_rx_ovf <= 1'b0;
            err_pt_ovf <= 1'b0;
        end
    end

    assign led[0] = led_lidar;          // blinking: lidar packets being parsed
    assign led[1] = led_pc;             // blinking: point cloud datagrams going out
    assign led[2] = led_other;          // toggles on every non-lidar frame
    assign led[3] = link_speed[1];      // on: RGMII link at 1000 Mb/s
    assign led[4] = link_speed[0];      // on: 100 Mb/s
    assign led[5] = err_fcs;            // sticky: frame with bad FCS received
    assign led[6] = err_rx_ovf;         // sticky: MAC RX FIFO overflow
    assign led[7] = err_pt_ovf;         // sticky: point FIFO overflow

endmodule
