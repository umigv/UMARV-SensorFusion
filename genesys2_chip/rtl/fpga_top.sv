// Digilent Genesys 2 (XC7K325T-2FFG900C) top level for the Tiny Tapeout
// sensor-fusion chip (tt_um_umigv_sensorfusion).
//
// Keeps tapeout/src/ unmodified (using genesys2_chip/rtl/lidar_latch_mem.sv in
// place of the Sky130 latch array) and adapts the single onboard RTL8211E RGMII
// PHY to the chip's 100 Mbit MII receive (lidar) and transmit (host) ports.
//
// Pmod wiring (all 3.3 V):
//   Pmod JC (Host ESP32 SPI):
//     jc[0] (pin 1) = host SPI SCK  (in)
//     jc[1] (pin 2) = host SPI CS_N (in)
//     jc[2] (pin 3) = host SPI MOSI (in)
//     jc[3] (pin 4) = host SPI MISO (out)
//     jc[4] (pin 7) = meas_ready    (out, uo[4])
//   Pmod JD (VN-300 + AMT22):
//     jd[0] (pin 1) = VN-300 vn_rx  (in, 3.3V TTL UART2 TX)
//     jd[1] (pin 2) = VN-300 SyncIn (out)
//     jd[2] (pin 3) = AMT22 CS_N    (out)
//     jd[3] (pin 4) = AMT22 SCK     (out)
//     jd[4] (pin 7) = AMT22 MOSI    (out)
//     jd[5] (pin 8) = AMT22 MISO    (in, <=3.3V!)
`timescale 1ns / 1ps

module fpga_top #(
    parameter bit SIM = 1'b0
) (
    // 200 MHz LVDS system clock, active-low reset button
    input  wire       sysclk_p,
    input  wire       sysclk_n,
    input  wire       cpu_resetn,

    // Status LEDs
    output wire [7:0] led,

    // Onboard Ethernet: RTL8211E-VL RGMII + MDIO (Bank 33 1.5V, reset Bank 13 3.3V)
    input  wire       eth_rxck,
    input  wire [3:0] eth_rxd,
    input  wire       eth_rxctl,
    output wire       eth_txck,
    output wire [3:0] eth_txd,
    output wire       eth_txctl,
    output wire       eth_phyrst_n,
    output wire       eth_mdc,
    inout  wire       eth_mdio,

    // Pmod JC: Host ESP32 SPI + measurements-ready pulse (3.3 V)
    input  wire       jc_sck,
    input  wire       jc_cs_n,
    input  wire       jc_mosi,
    output wire       jc_miso,
    output wire       jc_meas_ready,

    // Pmod JD: VN-300 TTL serial + SyncIn, AMT22 SPI (3.3 V)
    input  wire       jd_vn_rx,
    output wire       jd_vn_sync,
    output wire       jd_dw_cs_n,
    output wire       jd_dw_sck,
    output wire       jd_dw_mosi,
    input  wire       jd_dw_miso
);

    // =========================================================================
    // 1. Clocking: 200 MHz LVDS -> MMCM -> 50 MHz, 25 MHz, 25 MHz +90°, 200 MHz
    // =========================================================================
    wire clk_200_ibuf;
    wire clk50_mmcm, clk25_mmcm, clk25_90_mmcm, clk200_mmcm;
    wire clk50, clk25, clk25_90, clk200;
    wire mmcm_clkfb, mmcm_locked;
    wire idelay_rdy;

    generate
        if (SIM) begin : gen_sim_clk
            assign clk50       = sysclk_p; // driven at 50 MHz in sim or divided below
            assign clk200      = sysclk_p;
            assign mmcm_locked = cpu_resetn;
            assign idelay_rdy  = cpu_resetn;
            // Generate 25 MHz and 25 MHz +90 deg in simulation from 50 MHz
            logic c25_r = 1'b0, c25_90_r = 1'b0;
            always_ff @(posedge sysclk_p) c25_r    <= ~c25_r;
            always_ff @(negedge sysclk_p) c25_90_r <= c25_r;
            assign clk25    = c25_r;
            assign clk25_90 = c25_90_r;
        end else begin : gen_xilinx_clk
            IBUFGDS u_sysclk_ibufgds (
                .I  (sysclk_p),
                .IB (sysclk_n),
                .O  (clk_200_ibuf)
            );

            // VCO = 200 * 5 = 1000 MHz
            //   CLKOUT0 = 1000 / 20 = 50 MHz (chip core)
            //   CLKOUT1 = 1000 / 40 = 25 MHz (MII TX + RGMII TXD/TXCTL)
            //   CLKOUT2 = 1000 / 40 = 25 MHz +90 deg (RGMII TXCK)
            //   CLKOUT3 = 1000 / 5  = 200 MHz (IDELAYCTRL refclk)
            MMCME2_BASE #(
                .BANDWIDTH          ("OPTIMIZED"),
                .CLKIN1_PERIOD      (5.0),
                .REF_JITTER1        (0.010),
                .DIVCLK_DIVIDE      (1),
                .CLKFBOUT_MULT_F    (5.0),
                .CLKFBOUT_PHASE     (0.0),
                .CLKOUT0_DIVIDE_F   (20.0),
                .CLKOUT0_DUTY_CYCLE (0.5),
                .CLKOUT0_PHASE      (0.0),
                .CLKOUT1_DIVIDE     (40),
                .CLKOUT1_DUTY_CYCLE (0.5),
                .CLKOUT1_PHASE      (0.0),
                .CLKOUT2_DIVIDE     (40),
                .CLKOUT2_DUTY_CYCLE (0.5),
                .CLKOUT2_PHASE      (90.0),
                .CLKOUT3_DIVIDE     (5),
                .CLKOUT3_DUTY_CYCLE (0.5),
                .CLKOUT3_PHASE      (0.0),
                .CLKOUT4_DIVIDE     (1),
                .CLKOUT4_CASCADE    ("FALSE"),
                .CLKOUT5_DIVIDE     (1),
                .CLKOUT6_DIVIDE     (1),
                .STARTUP_WAIT       ("FALSE")
            ) u_mmcm (
                .CLKIN1   (clk_200_ibuf),
                .CLKFBIN  (mmcm_clkfb),
                .CLKFBOUT (mmcm_clkfb),
                .CLKFBOUTB(),
                .CLKOUT0  (clk50_mmcm),
                .CLKOUT0B (),
                .CLKOUT1  (clk25_mmcm),
                .CLKOUT1B (),
                .CLKOUT2  (clk25_90_mmcm),
                .CLKOUT2B (),
                .CLKOUT3  (clk200_mmcm),
                .CLKOUT3B (),
                .CLKOUT4  (),
                .CLKOUT5  (),
                .CLKOUT6  (),
                .RST      (!cpu_resetn),
                .PWRDWN   (1'b0),
                .LOCKED   (mmcm_locked)
            );

            BUFG u_bufg_50   (.I(clk50_mmcm),    .O(clk50));
            BUFG u_bufg_25   (.I(clk25_mmcm),    .O(clk25));
            BUFG u_bufg_25_90(.I(clk25_90_mmcm), .O(clk25_90));
            BUFG u_bufg_200  (.I(clk200_mmcm),   .O(clk200));
        end
    endgenerate

    // Synchronized active-high resets for 50 MHz and 25 MHz domains
    logic rst50, rst25, chip_rst_n;
    logic vio_soft_rst;
    wire  sys_arst = !mmcm_locked || !cpu_resetn || vio_soft_rst;

    reset_release u_rst50 (.clk(clk50), .arst(sys_arst), .rst(rst50));
    reset_release u_rst25 (.clk(clk25), .arst(sys_arst), .rst(rst25));
    assign chip_rst_n = !rst50;

    // PHY reset: hold low for 2^20 clocks at 50 MHz (~21 ms, RTL8211E requires >= 10 ms)
    localparam int PHY_RST_BITS = SIM ? 4 : 20;
    logic [PHY_RST_BITS:0] phy_rst_cnt;
    always_ff @(posedge clk50 or posedge rst50) begin
        if (rst50)                          phy_rst_cnt <= '0;
        else if (!phy_rst_cnt[PHY_RST_BITS]) phy_rst_cnt <= phy_rst_cnt + 1'b1;
    end
    assign eth_phyrst_n = phy_rst_cnt[PHY_RST_BITS];

    // =========================================================================
    // 2. RGMII IDELAY + 100 Mbit RGMII <-> MII Shim + MDIO Initializer
    // =========================================================================
    wire [3:0] eth_rxd_dly;
    wire       eth_rxctl_dly;

    generate
        if (SIM) begin : gen_sim_idelay
            assign eth_rxd_dly   = eth_rxd;
            assign eth_rxctl_dly = eth_rxctl;
        end else begin : gen_xilinx_idelay
            IDELAYCTRL u_idelayctrl (
                .REFCLK (clk200),
                .RST    (rst50),
                .RDY    (idelay_rdy)
            );

            for (genvar n = 0; n < 4; n = n + 1) begin : phy_rxd_idelay_bit
                IDELAYE2 #(
                    .IDELAY_TYPE ("FIXED")
                ) idelay_inst (
                    .IDATAIN     (eth_rxd[n]),
                    .DATAOUT     (eth_rxd_dly[n]),
                    .DATAIN      (1'b0),
                    .C           (1'b0),
                    .CE          (1'b0),
                    .INC         (1'b0),
                    .CINVCTRL    (1'b0),
                    .CNTVALUEIN  (5'd0),
                    .CNTVALUEOUT (),
                    .LD          (1'b0),
                    .LDPIPEEN    (1'b0),
                    .REGRST      (1'b0)
                );
            end

            IDELAYE2 #(
                .IDELAY_TYPE ("FIXED")
            ) phy_rx_ctl_idelay (
                .IDATAIN     (eth_rxctl),
                .DATAOUT     (eth_rxctl_dly),
                .DATAIN      (1'b0),
                .C           (1'b0),
                .CE          (1'b0),
                .INC         (1'b0),
                .CINVCTRL    (1'b0),
                .CNTVALUEIN  (5'd0),
                .CNTVALUEOUT (),
                .LD          (1'b0),
                .LDPIPEEN    (1'b0),
                .REGRST      (1'b0)
            );
        end
    endgenerate

    // MDIO tristate IOBUF
    logic mdio_o, mdio_t, mdio_i;
    generate
        if (SIM) begin : gen_sim_mdio_io
            assign eth_mdio = mdio_t ? 1'bz : mdio_o;
            assign mdio_i   = eth_mdio;
        end else begin : gen_xilinx_mdio_io
            IOBUF u_mdio_iobuf (
                .I  (mdio_o),
                .T  (mdio_t),
                .O  (mdio_i),
                .IO (eth_mdio)
            );
        end
    endgenerate

    // VIO control signals (all default to 0 = normal external hardware mode!)
    logic        vio_phy_loopback;
    logic        vio_spi_sel, vio_spi_go_tgl, vio_spi_is_wr;
    logic [6:0]  vio_spi_addr;
    logic [7:0]  vio_spi_wdata;
    logic        vio_sim_sensors;
    logic        vio_pktgen_sel, vio_pktgen_go_tgl, vio_mon_clr_tgl;

    logic        mdio_init_done, mdio_link_up, mdio_duplex;
    logic [1:0]  mdio_speed;
    logic [15:0] phy_id1, phy_id2, reg_bmcr, reg_bmsr, reg_gbcr, reg_physr;

    mdio_init_100m #(
        .PHY_ADDR      (5'd1),
        .CLK_DIV       (SIM ? 6 : 20),
        .BOOT_WAIT_CYC (SIM ? 20 : 2_500_000),
        .POLL_WAIT_CYC (SIM ? 50 : 500_000)
    ) u_mdio (
        .clk          (clk50),
        .rst          (rst50),
        .phy_rst_done (eth_phyrst_n),
        .loopback_en  (vio_phy_loopback),
        .mdc          (eth_mdc),
        .mdio_o       (mdio_o),
        .mdio_t       (mdio_t),
        .mdio_i       (mdio_i),
        .init_done    (mdio_init_done),
        .phy_id1      (phy_id1),
        .phy_id2      (phy_id2),
        .reg_bmcr     (reg_bmcr),
        .reg_bmsr     (reg_bmsr),
        .reg_gbcr     (reg_gbcr),
        .reg_physr    (reg_physr),
        .link_up      (mdio_link_up),
        .link_speed   (mdio_speed),
        .link_duplex  (mdio_duplex)
    );

    logic       mii_rx_clk, mii_rx_dv, mii_rx_er;
    logic [3:0] mii_rxd;
    logic       ibs_link, ibs_duplex;
    logic [1:0] ibs_speed;

    logic       chip_tx_en, pktgen_tx_en, shim_tx_en;
    logic [3:0] chip_txd, pktgen_txd, shim_txd;

    // Synchronize vio_pktgen_sel into clk25 domain
    (* ASYNC_REG = "TRUE" *) logic [1:0] pktgen_sel_25;
    always_ff @(posedge clk25) begin
        if (rst25) pktgen_sel_25 <= 2'b00;
        else       pktgen_sel_25 <= {pktgen_sel_25[0], vio_pktgen_sel};
    end

    assign shim_tx_en = pktgen_sel_25[1] ? pktgen_tx_en : chip_tx_en;
    assign shim_txd   = pktgen_sel_25[1] ? pktgen_txd   : chip_txd;

    rgmii_mii_100m #(
        .SIM (SIM)
    ) u_rgmii (
        .clk25        (clk25),
        .clk25_90     (clk25_90),
        .rst25        (rst25),
        .rgmii_rx_clk (eth_rxck),
        .rgmii_rx_ctl (eth_rxctl_dly),
        .rgmii_rxd    (eth_rxd_dly),
        .rgmii_tx_clk (eth_txck),
        .rgmii_tx_ctl (eth_txctl),
        .rgmii_txd    (eth_txd),
        .mii_rx_clk   (mii_rx_clk),
        .mii_rx_dv    (mii_rx_dv),
        .mii_rx_er    (mii_rx_er),
        .mii_rxd      (mii_rxd),
        .ibs_link     (ibs_link),
        .ibs_speed    (ibs_speed),
        .ibs_duplex   (ibs_duplex),
        .mii_tx_en    (shim_tx_en),
        .mii_txd      (shim_txd)
    );

    // =========================================================================
    // 3. Tiny Tapeout Chip Instance (unmodified tapeout/src/ RTL)
    // =========================================================================
    logic       test_spi_sck, test_spi_cs_n, test_spi_mosi;
    logic       sim_vn_rx, sim_dw_miso;

    wire  [7:0] ui_in = {
        mii_rx_clk,                                     // ui[7]: lidar MII RX_CLK
        clk25,                                          // ui[6]: host MII TX_CLK
        mii_rx_dv,                                      // ui[5]: lidar MII RX_DV
        vio_spi_sel     ? test_spi_mosi : jc_mosi,      // ui[4]: host SPI MOSI
        vio_spi_sel     ? test_spi_cs_n : jc_cs_n,      // ui[3]: host SPI CS_N
        vio_spi_sel     ? test_spi_sck  : jc_sck,       // ui[2]: host SPI SCK
        vio_sim_sensors ? sim_dw_miso   : jd_dw_miso,   // ui[1]: encoder MISO
        vio_sim_sensors ? sim_vn_rx     : jd_vn_rx      // ui[0]: VN-300 serial in
    };
    wire  [7:0] uio_in = {4'h0, mii_rxd};
    wire  [7:0] uo_out, uio_out, uio_oe;

    tt_um_umigv_sensorfusion u_chip (
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ena     (1'b1),
        .clk     (clk50),
        .rst_n   (chip_rst_n)
    );

    assign jd_vn_sync    = uo_out[0];
    assign jd_dw_cs_n    = uo_out[1];
    assign jd_dw_sck     = uo_out[2];
    assign jd_dw_mosi    = uo_out[3];
    assign jc_meas_ready = uo_out[4];
    assign jc_miso       = uo_out[5];
    assign chip_tx_en    = uo_out[7];
    assign chip_txd      = uio_out[7:4];

    // =========================================================================
    // 4. JTAG/VIO Self-Test & Telemetry Harness
    // =========================================================================
    logic [31:0] meas_beat_period, meas_deadline_clks;
    logic [15:0] meas_sync_width, meas_beat_cnt, meas_dw_reads;
    logic [7:0]  spi_rdata;
    logic        spi_done_tgl;
    logic [7:0]  pktgen_sent_cnt;
    logic [15:0] mon_tx_frames, mon_tx_good_fcs, mon_tx_bad_fcs, mon_tx_pc2x_ok;
    logic [7:0]  mon_last_flags;
    logic [15:0] mon_last_len, mon_last_p0_x, mon_last_p0_y, mon_last_p0_z;

    fpga_selftest u_selftest (
        .clk50              (clk50),
        .rst50              (rst50),
        .clk25              (clk25),
        .rst25              (rst25),
        .mon_vn_sync        (uo_out[0]),
        .mon_dw_cs_n        (uo_out[1]),
        .mon_dw_sck         (uo_out[2]),
        .mon_dw_mosi        (uo_out[3]),
        .mon_ukf_pulse      (uo_out[4]),
        .meas_beat_period   (meas_beat_period),
        .meas_sync_width    (meas_sync_width),
        .meas_deadline_clks (meas_deadline_clks),
        .meas_beat_cnt      (meas_beat_cnt),
        .meas_dw_reads      (meas_dw_reads),
        .spi_go_tgl         (vio_spi_go_tgl),
        .spi_is_wr          (vio_spi_is_wr),
        .spi_addr           (vio_spi_addr),
        .spi_wdata          (vio_spi_wdata),
        .spi_rdata          (spi_rdata),
        .spi_done_tgl       (spi_done_tgl),
        .test_spi_sck       (test_spi_sck),
        .test_spi_cs_n      (test_spi_cs_n),
        .test_spi_mosi      (test_spi_mosi),
        .chip_spi_miso      (uo_out[5]),
        .sim_sensors_en     (vio_sim_sensors),
        .sim_vn_rx          (sim_vn_rx),
        .sim_dw_miso        (sim_dw_miso),
        .pktgen_go_tgl      (vio_pktgen_go_tgl),
        .pktgen_tx_en       (pktgen_tx_en),
        .pktgen_txd         (pktgen_txd),
        .pktgen_sent_cnt    (pktgen_sent_cnt),
        .chip_tx_en         (chip_tx_en),
        .chip_txd           (chip_txd),
        .mon_clr_tgl        (vio_mon_clr_tgl),
        .mon_tx_frames      (mon_tx_frames),
        .mon_tx_good_fcs    (mon_tx_good_fcs),
        .mon_tx_bad_fcs     (mon_tx_bad_fcs),
        .mon_tx_pc2x_ok     (mon_tx_pc2x_ok),
        .mon_last_flags     (mon_last_flags),
        .mon_last_len       (mon_last_len),
        .mon_last_p0_x      (mon_last_p0_x),
        .mon_last_p0_y      (mon_last_p0_y),
        .mon_last_p0_z      (mon_last_p0_z)
    );

    // =========================================================================
    // 4b. UKF Live Telemetry Capture (when WITH_UKF is defined)
    // =========================================================================
    logic [15:0] ukf_state_cnt;
    logic [7:0]  ukf_overruns;
    logic        ukf_started, ukf_running, ukf_state_valid;
    logic [9:0]  ukf_pc;
    logic [31:0] ukf_x, ukf_y, ukf_theta, ukf_omega, ukf_v, ukf_r;

`ifdef WITH_UKF
    assign ukf_state_valid = u_chip.u_ukf.state_valid;
    assign ukf_overruns    = u_chip.u_ukf.overruns;
    assign ukf_started     = u_chip.u_ukf.started;
    assign ukf_running     = u_chip.u_ukf.running;
    assign ukf_pc          = u_chip.u_ukf.pc;

    always_ff @(posedge clk50) begin
        if (rst50) begin
            ukf_state_cnt <= 16'd0;
            ukf_x         <= 32'd0;
            ukf_y         <= 32'd0;
            ukf_theta     <= 32'd0;
            ukf_omega     <= 32'd0;
            ukf_v         <= 32'd0;
            ukf_r         <= 32'd0;
        end else if (u_chip.u_ukf.state_valid) begin
            ukf_state_cnt <= ukf_state_cnt + 16'd1;
            ukf_x         <= u_chip.u_ukf.state_x;
            ukf_y         <= u_chip.u_ukf.state_y;
            ukf_theta     <= u_chip.u_ukf.state_theta;
            ukf_omega     <= u_chip.u_ukf.state_omega;
            ukf_v         <= u_chip.u_ukf.state_v;
            ukf_r         <= u_chip.u_ukf.u_store.q[10]; // dead-wheel radius (m)
        end
    end
`else
    assign ukf_state_valid = 1'b0;
    assign ukf_overruns    = 8'd0;
    assign ukf_started     = 1'b0;
    assign ukf_running     = 1'b0;
    assign ukf_pc          = 10'd0;
    assign ukf_state_cnt   = 16'd0;
    assign ukf_x           = 32'd0;
    assign ukf_y           = 32'd0;
    assign ukf_theta       = 32'd0;
    assign ukf_omega       = 32'd0;
    assign ukf_v           = 32'd0;
    assign ukf_r           = 32'd0;
`endif

    // =========================================================================
    // 5. Status LEDs (all 50 MHz domain)
    // =========================================================================
    // Tap internal status pulses from u_chip for LEDs & ILA visibility
    wire [6:0] stat_p = u_chip.lidar_stat;
    wire       vn_ok  = u_chip.vn_valid;
    wire       dw_ok  = u_chip.dw_valid;

    logic [7:0]  beat_div;
    logic [8:0]  lidar_div;
    logic [9:0]  tx_div;
    logic        led_beat, led_lidar, led_tx;
    logic        err_rx_sticky, err_pipe_sticky;

    always_ff @(posedge clk50) begin
        if (rst50) begin
            beat_div <= 8'd0; lidar_div <= 9'd0; tx_div <= 10'd0;
            led_beat <= 1'b0; led_lidar <= 1'b0; led_tx <= 1'b0;
            err_rx_sticky <= 1'b0; err_pipe_sticky <= 1'b0;
        end else begin
            if (uo_out[4]) begin
                beat_div <= beat_div + 8'd1;
                if (beat_div == 8'd199) begin beat_div <= 8'd0; led_beat <= !led_beat; end // 1 Hz at 400 Hz beat
            end
            if (stat_p[0]) begin
                lidar_div <= lidar_div + 9'd1;
                if (&lidar_div) led_lidar <= !led_lidar;
            end
            if (stat_p[3]) begin
                tx_div <= tx_div + 10'd1;
                if (&tx_div) led_tx <= !led_tx;
            end
            if (stat_p[2]) err_rx_sticky <= 1'b1;                             // bad RX CRC
            if (stat_p[4] || stat_p[5] || stat_p[6] || (ukf_overruns != 8'd0))
                err_pipe_sticky <= 1'b1; // buf ovf / motion ovr / tx und / ukf overrun
        end
    end

    assign led[0] = mdio_link_up && (mdio_speed == 2'b01); // 100M Ethernet link up
    assign led[1] = led_beat;                              // 1 Hz heartbeat from 400 Hz beat
    assign led[2] = led_lidar;                             // lidar packets parsed
    assign led[3] = led_tx;                                // PC2X frames sent
    assign led[4] = vn_ok;                                 // VN-300 measurement valid
    assign led[5] = dw_ok;                                 // AMT22 encoder measurement valid
    assign led[6] = err_rx_sticky;                         // sticky RX CRC error
    assign led[7] = err_pipe_sticky;                       // sticky pipeline error

    // =========================================================================
    // 6. Vivado VIO + ILA Cores (excluded in simulation)
    // =========================================================================
    generate
        if (SIM) begin : gen_sim_vio
            assign vio_soft_rst      = 1'b0;
            assign vio_phy_loopback  = 1'b0;
            assign vio_spi_sel       = 1'b0;
            assign vio_spi_go_tgl    = 1'b0;
            assign vio_spi_is_wr     = 1'b0;
            assign vio_spi_addr      = 7'd0;
            assign vio_spi_wdata     = 8'd0;
            assign vio_sim_sensors   = 1'b0;
            assign vio_pktgen_sel    = 1'b0;
            assign vio_pktgen_go_tgl = 1'b0;
            assign vio_mon_clr_tgl   = 1'b0;
        end else begin : gen_xilinx_debug
            // Synchronize clk25 monitor outputs to clk50 for clean VIO readback
            (* ASYNC_REG = "TRUE" *) logic [15:0] mon_tx_frames_s1,   mon_tx_frames_50;
            (* ASYNC_REG = "TRUE" *) logic [15:0] mon_tx_good_fcs_s1, mon_tx_good_fcs_50;
            (* ASYNC_REG = "TRUE" *) logic [15:0] mon_tx_bad_fcs_s1,  mon_tx_bad_fcs_50;
            (* ASYNC_REG = "TRUE" *) logic [15:0] mon_tx_pc2x_ok_s1,  mon_tx_pc2x_ok_50;
            (* ASYNC_REG = "TRUE" *) logic [7:0]  pktgen_sent_s1,     pktgen_sent_50;
            (* ASYNC_REG = "TRUE" *) logic [7:0]  mon_flags_s1,       mon_flags_50;
            (* ASYNC_REG = "TRUE" *) logic [15:0] mon_len_s1,         mon_len_50;
            (* ASYNC_REG = "TRUE" *) logic [15:0] mon_p0x_s1,         mon_p0x_50;
            (* ASYNC_REG = "TRUE" *) logic [15:0] mon_p0y_s1,         mon_p0y_50;
            (* ASYNC_REG = "TRUE" *) logic [15:0] mon_p0z_s1,         mon_p0z_50;
            (* ASYNC_REG = "TRUE" *) logic [3:0]  ibs_s1,             ibs_50;

            always_ff @(posedge clk50) begin
                mon_tx_frames_s1   <= mon_tx_frames;   mon_tx_frames_50   <= mon_tx_frames_s1;
                mon_tx_good_fcs_s1 <= mon_tx_good_fcs; mon_tx_good_fcs_50 <= mon_tx_good_fcs_s1;
                mon_tx_bad_fcs_s1  <= mon_tx_bad_fcs;  mon_tx_bad_fcs_50  <= mon_tx_bad_fcs_s1;
                mon_tx_pc2x_ok_s1  <= mon_tx_pc2x_ok;  mon_tx_pc2x_ok_50  <= mon_tx_pc2x_ok_s1;
                pktgen_sent_s1     <= pktgen_sent_cnt; pktgen_sent_50     <= pktgen_sent_s1;
                mon_flags_s1       <= mon_last_flags;  mon_flags_50       <= mon_flags_s1;
                mon_len_s1         <= mon_last_len;    mon_len_50         <= mon_len_s1;
                mon_p0x_s1         <= mon_last_p0_x;   mon_p0x_50         <= mon_p0x_s1;
                mon_p0y_s1         <= mon_last_p0_y;   mon_p0y_50         <= mon_p0y_s1;
                mon_p0z_s1         <= mon_last_p0_z;   mon_p0z_50         <= mon_p0z_s1;
                ibs_s1             <= {ibs_duplex, ibs_speed, ibs_link};
                ibs_50             <= ibs_s1;
            end

            vio_ctrl u_vio (
                .clk        (clk50),
                // Status inputs
                .probe_in0  ({mmcm_locked, idelay_rdy, eth_phyrst_n, mdio_init_done,
                              mdio_link_up, mdio_speed, mdio_duplex, ibs_50, vn_ok, dw_ok,
                              err_rx_sticky, err_pipe_sticky}), // 16 bits
                .probe_in1  ({phy_id1, phy_id2}),               // 32 bits
                .probe_in2  ({reg_bmcr, reg_bmsr}),             // 32 bits
                .probe_in3  ({reg_gbcr, reg_physr}),            // 32 bits
                .probe_in4  (meas_beat_period),                 // 32 bits
                .probe_in5  ({meas_sync_width, meas_beat_cnt}), // 32 bits
                .probe_in6  (meas_deadline_clks),               // 32 bits
                .probe_in7  ({meas_dw_reads, spi_done_tgl, 7'd0, spi_rdata}), // 32 bits
                .probe_in8  ({mon_tx_frames_50, mon_tx_good_fcs_50}),         // 32 bits
                .probe_in9  ({mon_tx_bad_fcs_50, mon_tx_pc2x_ok_50}),         // 32 bits
                .probe_in10 ({pktgen_sent_50, mon_flags_50, mon_len_50}),     // 32 bits
                .probe_in11 ({mon_p0x_50, mon_p0y_50}),                       // 32 bits
                .probe_in12 ({mon_p0z_50, u_chip.u_timestamper.status_raw}),  // 32 bits
                .probe_in13 ({ukf_state_cnt, ukf_overruns, 6'd0, ukf_started, ukf_running}), // 32 bits
                .probe_in14 (ukf_x),                                          // 32 bits fp32
                .probe_in15 (ukf_y),                                          // 32 bits fp32
                .probe_in16 (ukf_theta),                                      // 32 bits fp32
                .probe_in17 (ukf_omega),                                      // 32 bits fp32
                .probe_in18 (ukf_v),                                          // 32 bits fp32
                .probe_in19 (ukf_r),                                          // 32 bits fp32
                // Control outputs (all default 0)
                .probe_out0 (vio_soft_rst),
                .probe_out1 (vio_phy_loopback),
                .probe_out2 (vio_spi_sel),
                .probe_out3 (vio_spi_go_tgl),
                .probe_out4 (vio_spi_is_wr),
                .probe_out5 (vio_spi_addr),
                .probe_out6 (vio_spi_wdata),
                .probe_out7 (vio_sim_sensors),
                .probe_out8 (vio_pktgen_sel),
                .probe_out9 (vio_pktgen_go_tgl),
                .probe_out10(vio_mon_clr_tgl)
            );

            // 50 MHz ILA: VN-300 parser, AMT22 encoder SPI, host SPI, lidar stats, UKF
            ila_core50 u_ila50 (
                .clk    (clk50),
                .probe0 ({uo_out[0], ui_in[0], u_chip.u_timestamper.rx_byte_ready,
                          u_chip.u_timestamper.vn_done, u_chip.u_timestamper.vn_good,
                          u_chip.u_timestamper.valid_header, u_chip.u_timestamper.crc_ok,
                          u_chip.u_timestamper.status_ok}),                   // 8b VN flags
                .probe1 ({u_chip.u_timestamper.uart_byte_ctr,
                          u_chip.u_timestamper.rx_byte,
                          u_chip.u_timestamper.crc}),                         // 30b VN parser
                .probe2 ({uo_out[1], uo_out[2], uo_out[3], ui_in[1],
                          u_chip.u_timestamper.dw_done, u_chip.u_timestamper.dw_good,
                          u_chip.u_timestamper.rd_ok, uo_out[4]}),            // 8b AMT22 + pulse
                .probe3 (u_chip.u_timestamper.rx_shift),                      // 32b AMT22 raw
                .probe4 ({ui_in[2], ui_in[3], ui_in[4], uo_out[5],
                          u_chip.bus_we, u_chip.bus_addr, u_chip.bus_wdata}), // 20b Host SPI
                .probe5 ({stat_p, vn_ok, dw_ok, u_chip.u_lidar.rx_tvalid,
                          u_chip.u_lidar.rx_tlast, u_chip.u_lidar.rx_crc_bad,
                          u_chip.u_lidar.pt_valid, 3'd0}),                    // 16b Lidar core
                .probe6 ({ukf_state_valid, ukf_started, ukf_overruns, ukf_pc})// 20b UKF
            );

            // 25 MHz RX ILA: RGMII/MII receive stream directly off the PHY
            ila_mii_rx u_ila_rx (
                .clk    (mii_rx_clk),
                .probe0 ({mii_rx_dv, mii_rx_er, ibs_duplex, ibs_speed, ibs_link, 2'd0}), // 8b
                .probe1 (mii_rxd)                                                        // 4b
            );
        end
    endgenerate

    wire _unused = &{uio_oe, uio_out[3:0], uo_out[6], 1'b0};

endmodule
