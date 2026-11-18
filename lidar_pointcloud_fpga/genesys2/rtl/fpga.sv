// Digilent Genesys 2 (XC7K325T-2FFG900C) top level.
//
// VLP-16 UDP in on the on-board RTL8211E RGMII PHY, XYZ float32 point cloud
// UDP out on the same port.  Clocking / IDELAY structure follows the taxi
// KC705 RGMII example design.
`resetall
`timescale 1ns / 1ps
`default_nettype none

module fpga #(
    parameter logic [15:0] LIDAR_PORT = 16'd0,       // 0 = any UDP port
    parameter logic        ROS_FRAME  = 1'b1,
    parameter logic [47:0] LOCAL_MAC  = 48'h02_00_00_00_00_01,
    parameter logic [31:0] LOCAL_IP   = {8'd192, 8'd168, 8'd1, 8'd50},
    parameter logic [47:0] DEST_MAC   = 48'hFF_FF_FF_FF_FF_FF,
    parameter logic [31:0] DEST_IP    = {8'd255, 8'd255, 8'd255, 8'd255},
    parameter logic [15:0] SRC_PORT   = 16'd2370,
    parameter logic [15:0] DEST_PORT  = 16'd2370
) (
    // 200 MHz LVDS system clock, active-low reset button
    input  wire logic       sysclk_p,
    input  wire logic       sysclk_n,
    input  wire logic       cpu_resetn,

    output wire logic [7:0] led,

    // Ethernet: 1000BASE-T RGMII (RTL8211E-VL)
    input  wire logic       eth_rxck,
    input  wire logic [3:0] eth_rxd,
    input  wire logic       eth_rxctl,
    output wire logic       eth_txck,
    output wire logic [3:0] eth_txd,
    output wire logic       eth_txctl,
    output wire logic       eth_phyrst_n
);

// Clock and reset
wire clk_200mhz_ibufg;

wire clk_mmcm_out;
wire clk_int;
wire clk90_mmcm_out;
wire clk90_int;
wire rst_int;

wire clk_200mhz_mmcm_out;
wire clk_200mhz_int;

wire mmcm_rst = !cpu_resetn;
wire mmcm_locked;
wire mmcm_clkfb;

IBUFGDS
clk_200mhz_ibufgds_inst (
    .I(sysclk_p),
    .IB(sysclk_n),
    .O(clk_200mhz_ibufg)
);

// MMCM: 200 MHz in, VCO = 200 * 5 = 1000 MHz
//   CLKOUT0 125 MHz, CLKOUT1 125 MHz +90 deg (RGMII TX clock), CLKOUT2 200 MHz (IDELAYCTRL)
MMCME2_BASE #(
    .BANDWIDTH("OPTIMIZED"),
    .CLKIN1_PERIOD(5.0),
    .REF_JITTER1(0.010),
    .DIVCLK_DIVIDE(1),
    .CLKFBOUT_MULT_F(5),
    .CLKFBOUT_PHASE(0),
    .CLKOUT0_DIVIDE_F(8),
    .CLKOUT0_DUTY_CYCLE(0.5),
    .CLKOUT0_PHASE(0),
    .CLKOUT1_DIVIDE(8),
    .CLKOUT1_DUTY_CYCLE(0.5),
    .CLKOUT1_PHASE(90),
    .CLKOUT2_DIVIDE(5),
    .CLKOUT2_DUTY_CYCLE(0.5),
    .CLKOUT2_PHASE(0),
    .CLKOUT3_DIVIDE(1),
    .CLKOUT4_DIVIDE(1),
    .CLKOUT4_CASCADE("FALSE"),
    .CLKOUT5_DIVIDE(1),
    .CLKOUT6_DIVIDE(1),
    .STARTUP_WAIT("FALSE")
)
clk_mmcm_inst (
    .CLKIN1(clk_200mhz_ibufg),
    .CLKFBIN(mmcm_clkfb),
    .CLKFBOUT(mmcm_clkfb),
    .CLKFBOUTB(),
    .CLKOUT0(clk_mmcm_out),
    .CLKOUT0B(),
    .CLKOUT1(clk90_mmcm_out),
    .CLKOUT1B(),
    .CLKOUT2(clk_200mhz_mmcm_out),
    .CLKOUT2B(),
    .CLKOUT3(),
    .CLKOUT3B(),
    .CLKOUT4(),
    .CLKOUT5(),
    .CLKOUT6(),
    .RST(mmcm_rst),
    .PWRDWN(1'b0),
    .LOCKED(mmcm_locked)
);

BUFG clk_bufg_inst        (.I(clk_mmcm_out),        .O(clk_int));
BUFG clk90_bufg_inst      (.I(clk90_mmcm_out),      .O(clk90_int));
BUFG clk_200mhz_bufg_inst (.I(clk_200mhz_mmcm_out), .O(clk_200mhz_int));

taxi_sync_reset #(
    .N(4)
)
sync_reset_inst (
    .clk(clk_int),
    .rst(!mmcm_locked),
    .out(rst_int)
);

// IDELAY on the RGMII receive data (tap value set in genesys2.xdc)
wire [3:0] eth_rxd_int;
wire       eth_rxctl_int;

IDELAYCTRL
idelayctrl_inst (
    .REFCLK(clk_200mhz_int),
    .RST(rst_int),
    .RDY()
);

for (genvar n = 0; n < 4; n = n + 1) begin : phy_rxd_idelay_bit

    IDELAYE2 #(
        .IDELAY_TYPE("FIXED")
    )
    idelay_inst (
        .IDATAIN(eth_rxd[n]),
        .DATAOUT(eth_rxd_int[n]),
        .DATAIN(1'b0),
        .C(1'b0),
        .CE(1'b0),
        .INC(1'b0),
        .CINVCTRL(1'b0),
        .CNTVALUEIN(5'd0),
        .CNTVALUEOUT(),
        .LD(1'b0),
        .LDPIPEEN(1'b0),
        .REGRST(1'b0)
    );

end

IDELAYE2 #(
    .IDELAY_TYPE("FIXED")
)
phy_rx_ctl_idelay (
    .IDATAIN(eth_rxctl),
    .DATAOUT(eth_rxctl_int),
    .DATAIN(1'b0),
    .C(1'b0),
    .CE(1'b0),
    .INC(1'b0),
    .CINVCTRL(1'b0),
    .CNTVALUEIN(5'd0),
    .CNTVALUEOUT(),
    .LD(1'b0),
    .LDPIPEEN(1'b0),
    .REGRST(1'b0)
);

fpga_core #(
    .SIM(1'b0),
    .VENDOR("XILINX"),
    .FAMILY("kintex7"),
    .LIDAR_PORT(LIDAR_PORT),
    .ROS_FRAME(ROS_FRAME),
    .LOCAL_MAC(LOCAL_MAC),
    .LOCAL_IP(LOCAL_IP),
    .DEST_MAC(DEST_MAC),
    .DEST_IP(DEST_IP),
    .SRC_PORT(SRC_PORT),
    .DEST_PORT(DEST_PORT)
)
core_inst (
    .clk(clk_int),
    .clk90(clk90_int),
    .rst(rst_int),
    .led(led),
    .phy_rx_clk(eth_rxck),
    .phy_rxd(eth_rxd_int),
    .phy_rx_ctl(eth_rxctl_int),
    .phy_tx_clk(eth_txck),
    .phy_txd(eth_txd),
    .phy_tx_ctl(eth_txctl),
    .phy_reset_n(eth_phyrst_n)
);

endmodule

`resetall
