// 100BASE-TX RGMII <-> MII shim for Kintex-7 / RTL8211E-VL on Digilent Genesys 2.
//
// Per RGMII v2.0 and RTL8211E datasheet at 100 Mbps (25 MHz clock):
//   - RX: PHY drives eth_rxck at 25 MHz with +2 ns delay (Genesys 2 RXDLY=1).
//         On rising edge:  eth_rxctl = RX_DV, eth_rxd[3:0] = MII nibble.
//         On falling edge: eth_rxctl = RX_DV ^ RX_ER, eth_rxd[3:0] = duplicate nibble.
//         When !RX_DV && !RX_ER, eth_rxd[3:0] carries in-band status:
//           [0] = link up, [2:1] = speed (00=10M, 01=100M, 10=1000M), [3] = duplex.
//   - TX: MAC drives eth_txck at 25 MHz shifted +90 deg (Genesys 2 TXDLY=0).
//         Both edges of eth_txd[3:0] carry the same MII nibble; both edges of
//         eth_txctl carry mii_tx_en (since TX_ER = 0).
module rgmii_mii_100m #(
    parameter bit SIM = 1'b0
) (
    // 25 MHz TX clocks from MMCM (0 deg and +90 deg)
    input  logic       clk25,
    input  logic       clk25_90,
    input  logic       rst25,

    // RGMII pins (already passed through IDELAY on RX in top level)
    input  logic       rgmii_rx_clk,
    input  logic       rgmii_rx_ctl,
    input  logic [3:0] rgmii_rxd,
    output logic       rgmii_tx_clk,
    output logic       rgmii_tx_ctl,
    output logic [3:0] rgmii_txd,

    // MII receive interface (25 MHz mii_rx_clk domain)
    output logic       mii_rx_clk,
    output logic       mii_rx_dv,
    output logic       mii_rx_er,
    output logic [3:0] mii_rxd,

    // In-band link status decoded on mii_rx_clk
    output logic       ibs_link,
    output logic [1:0] ibs_speed,
    output logic       ibs_duplex,

    // MII transmit interface (25 MHz clk25 domain)
    input  logic       mii_tx_en,
    input  logic [3:0] mii_txd
);

    // ---- RX clock buffer and DDR capture -------------------------------------
    logic       rx_ctl_r, rx_ctl_f;
    logic [3:0] rxd_r, rxd_f;

    generate
        if (SIM) begin : gen_sim
            assign mii_rx_clk = rgmii_rx_clk;

            logic       ctl_pos, ctl_neg;
            logic [3:0] d_pos, d_neg;
            always_ff @(posedge mii_rx_clk) begin
                ctl_pos <= rgmii_rx_ctl;
                d_pos   <= rgmii_rxd;
            end
            always_ff @(negedge mii_rx_clk) begin
                ctl_neg <= rgmii_rx_ctl;
                d_neg   <= rgmii_rxd;
            end
            always_ff @(posedge mii_rx_clk) begin
                rx_ctl_r <= ctl_pos;
                rx_ctl_f <= ctl_neg;
                rxd_r    <= d_pos;
                rxd_f    <= d_neg;
            end

            // TX: drive DDR pins with +90 deg clock
            assign rgmii_tx_clk = clk25_90;
            always_ff @(posedge clk25) begin
                if (rst25) begin
                    rgmii_tx_ctl <= 1'b0;
                    rgmii_txd    <= 4'h0;
                end else begin
                    rgmii_tx_ctl <= mii_tx_en;
                    rgmii_txd    <= mii_txd;
                end
            end
        end else begin : gen_xilinx
            BUFG u_rx_clk_bufg (
                .I (rgmii_rx_clk),
                .O (mii_rx_clk)
            );

            IDDR #(
                .DDR_CLK_EDGE ("SAME_EDGE_PIPELINED"),
                .INIT_Q1      (1'b0),
                .INIT_Q2      (1'b0),
                .SRTYPE       ("SYNC")
            ) u_iddr_rxctl (
                .Q1 (rx_ctl_r),
                .Q2 (rx_ctl_f),
                .C  (mii_rx_clk),
                .CE (1'b1),
                .D  (rgmii_rx_ctl),
                .R  (1'b0),
                .S  (1'b0)
            );

            for (genvar i = 0; i < 4; i = i + 1) begin : gen_iddr_rxd
                IDDR #(
                    .DDR_CLK_EDGE ("SAME_EDGE_PIPELINED"),
                    .INIT_Q1      (1'b0),
                    .INIT_Q2      (1'b0),
                    .SRTYPE       ("SYNC")
                ) u_iddr_d (
                    .Q1 (rxd_r[i]),
                    .Q2 (rxd_f[i]),
                    .C  (mii_rx_clk),
                    .CE (1'b1),
                    .D  (rgmii_rxd[i]),
                    .R  (1'b0),
                    .S  (1'b0)
                );
            end

            // TX clock: 25 MHz +90 deg via ODDR
            ODDR #(
                .DDR_CLK_EDGE ("SAME_EDGE"),
                .INIT         (1'b0),
                .SRTYPE       ("SYNC")
            ) u_oddr_txck (
                .Q  (rgmii_tx_clk),
                .C  (clk25_90),
                .CE (1'b1),
                .D1 (1'b1),
                .D2 (1'b0),
                .R  (1'b0),
                .S  (1'b0)
            );

            // TX_CTL: duplicate TX_EN on both edges (TX_ER = 0)
            ODDR #(
                .DDR_CLK_EDGE ("SAME_EDGE"),
                .INIT         (1'b0),
                .SRTYPE       ("SYNC")
            ) u_oddr_txctl (
                .Q  (rgmii_tx_ctl),
                .C  (clk25),
                .CE (1'b1),
                .D1 (mii_tx_en),
                .D2 (mii_tx_en),
                .R  (rst25),
                .S  (1'b0)
            );

            // TXD[3:0]: duplicate nibble on both edges for 100BASE-TX RGMII
            for (genvar i = 0; i < 4; i = i + 1) begin : gen_oddr_txd
                ODDR #(
                    .DDR_CLK_EDGE ("SAME_EDGE"),
                    .INIT         (1'b0),
                    .SRTYPE       ("SYNC")
                ) u_oddr_d (
                    .Q  (rgmii_txd[i]),
                    .C  (clk25),
                    .CE (1'b1),
                    .D1 (mii_txd[i]),
                    .D2 (mii_txd[i]),
                    .R  (rst25),
                    .S  (1'b0)
                );
            end
        end
    endgenerate

    assign mii_rx_dv = rx_ctl_r;
    assign mii_rx_er = rx_ctl_r ^ rx_ctl_f;
    assign mii_rxd   = rxd_r;

    // Capture in-band status during inter-frame idle (!RX_DV && !RX_ER)
    always_ff @(posedge mii_rx_clk) begin
        if (!rx_ctl_r && !rx_ctl_f) begin
            ibs_link   <= rxd_r[0];
            ibs_speed  <= rxd_r[2:1];
            ibs_duplex <= rxd_r[3];
        end
    end

    wire _unused = &{rxd_f, 1'b0};

endmodule
