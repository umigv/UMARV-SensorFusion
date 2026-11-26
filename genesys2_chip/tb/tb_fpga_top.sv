// End-to-end simulation of fpga_top (SIM=1) including:
//   - MDIO initializer talking to a behavioral RTL8211E PHY slave model
//   - 100BASE-TX RGMII loopback through rgmii_mii_100m
//   - VIO SPI master reading/writing tt_um_umigv_sensorfusion's spi_regs
//   - Synthetic VN-300 + AMT22 responders through timestamper
//   - Synthetic VLP-16 Ethernet generator -> RGMII loopback -> lidar_top -> PC2X v3 verifier
`timescale 1ns / 1ps

module tb_fpga_top;
    logic clk50 = 1'b0;
    always #10 clk50 = ~clk50; // 50 MHz

    logic cpu_resetn = 1'b0;
    wire [7:0] led;

    // RGMII loopback wires (simulating RTL8211E 100M PCS loopback)
    wire       eth_txck, eth_txctl, eth_phyrst_n, eth_mdc;
    wire [3:0] eth_txd;
    tri1       eth_mdio;

    wire       eth_rxck  = eth_txck;
    wire       eth_rxctl = eth_txctl;
    wire [3:0] eth_rxd   = eth_txd;

    // Pmod idle lines
    logic jc_sck = 1'b0, jc_cs_n = 1'b1, jc_mosi = 1'b0;
    wire  jc_miso, jc_meas_ready;
    logic jd_vn_rx = 1'b1, jd_dw_miso = 1'b0;
    wire  jd_vn_sync, jd_dw_cs_n, jd_dw_sck, jd_dw_mosi;

    fpga_top #(.SIM(1'b1)) dut (
        .sysclk_p      (clk50),
        .sysclk_n      (~clk50),
        .cpu_resetn    (cpu_resetn),
        .led           (led),
        .eth_rxck      (eth_rxck),
        .eth_rxd       (eth_rxd),
        .eth_rxctl     (eth_rxctl),
        .eth_txck      (eth_txck),
        .eth_txd       (eth_txd),
        .eth_txctl     (eth_txctl),
        .eth_phyrst_n  (eth_phyrst_n),
        .eth_mdc       (eth_mdc),
        .eth_mdio      (eth_mdio),
        .jc_sck        (jc_sck),
        .jc_cs_n       (jc_cs_n),
        .jc_mosi       (jc_mosi),
        .jc_miso       (jc_miso),
        .jc_meas_ready (jc_meas_ready),
        .jd_vn_rx      (jd_vn_rx),
        .jd_vn_sync    (jd_vn_sync),
        .jd_dw_cs_n    (jd_dw_cs_n),
        .jd_dw_sck     (jd_dw_sck),
        .jd_dw_mosi    (jd_dw_mosi),
        .jd_dw_miso    (jd_dw_miso)
    );

    // ---- Behavioral RTL8211E MDIO slave (PHY addr 1) -------------------------
    logic [15:0] phy_regs [0:31];
    initial begin
        for (int i = 0; i < 32; i++) phy_regs[i] = 16'h0000;
        phy_regs[0]  = 16'h1140; // default 1G AN
        phy_regs[1]  = 16'h796D; // link up, AN complete
        phy_regs[2]  = 16'h001C; // RTL8211E PHYID1
        phy_regs[3]  = 16'hC915; // RTL8211E PHYID2
        phy_regs[4]  = 16'h01E1;
        phy_regs[9]  = 16'h0300; // default 1G adv
        phy_regs[17] = 16'h6400; // PHYSR: 100M (01), FD (1), link up (bit 10)
    end

    logic       mdio_drv = 1'b0, mdio_val = 1'b1;
    assign eth_mdio = mdio_drv ? mdio_val : 1'bz;

    int         m_bit = 0;
    logic [1:0] m_op;
    logic [4:0] m_phy, m_reg;
    logic [15:0] m_wdata, m_rdata;
    logic [1:0] m_st = 2'd0; // 0: wait start, 1: hdr, 2: rd, 3: wr

    always @(posedge eth_mdc) begin
        case (m_st)
            2'd0: begin
                mdio_drv <= 1'b0;
                if (eth_mdio === 1'b0) begin
                    m_st  <= 2'd1;
                    m_bit <= 0;
                end
            end
            2'd1: begin
                if (m_bit == 0) begin
                    if (eth_mdio !== 1'b1) m_st <= 2'd0;
                end else if (m_bit <= 2)  m_op  <= {m_op[0], eth_mdio};
                else if (m_bit <= 7)      m_phy <= {m_phy[3:0], eth_mdio};
                else if (m_bit <= 12)     m_reg <= {m_reg[3:0], eth_mdio};
                if (m_bit == 12) begin
                    if (m_op == 2'b10 && m_phy == 5'd1) begin
                        m_rdata <= phy_regs[{m_reg[3:0], eth_mdio}];
                        m_st    <= 2'd2;
                        m_bit   <= 0;
                    end else if (m_op == 2'b01 && m_phy == 5'd1) begin
                        m_st  <= 2'd3;
                        m_bit <= 0;
                    end else begin
                        m_st  <= 2'd0;
                    end
                end else begin
                    m_bit <= m_bit + 1;
                end
            end
            2'd2: begin
                // Read TA + 16 bits
                if (m_bit == 17) begin
                    mdio_drv <= 1'b0;
                    m_st     <= 2'd0;
                end else begin
                    m_bit <= m_bit + 1;
                end
            end
            2'd3: begin
                // Write TA (2 bits) + 16 bits
                if (m_bit >= 2) m_wdata <= {m_wdata[14:0], eth_mdio};
                if (m_bit == 17) begin
                    phy_regs[m_reg] <= {m_wdata[14:0], eth_mdio};
                    m_st <= 2'd0;
                end else begin
                    m_bit <= m_bit + 1;
                end
            end
        endcase
    end

    always @(negedge eth_mdc) begin
        if (m_st == 2'd2) begin
            if (m_bit == 0) begin
                mdio_drv <= 1'b0;
            end else if (m_bit == 1) begin
                mdio_drv <= 1'b1;
                mdio_val <= 1'b0;
            end else if (m_bit <= 17) begin
                mdio_drv <= 1'b1;
                mdio_val <= m_rdata[17 - m_bit];
            end
        end else begin
            mdio_drv <= 1'b0;
        end
    end

    // ---- Test helpers forcing VIO outputs in SIM mode ------------------------
    int errors = 0, checks = 0;
    task automatic check(input bit ok, input string what);
        checks++;
        if (!ok) begin errors++; $display("FAIL: %s", what); end
        else     $display("PASS: %s", what);
    endtask

    logic spi_tgl = 1'b0, pg_tgl = 1'b0;
    logic tb_we = 1'b0; logic [6:0] tb_addr = 7'd0; logic [7:0] tb_wdata = 8'd0;
    task automatic vio_spi_xfer(input bit we, input logic [6:0] addr, input logic [7:0] wdata, output logic [7:0] rdata);
        logic prev_done;
        prev_done = dut.spi_done_tgl;
        tb_we    = we;
        tb_addr  = addr;
        tb_wdata = wdata;
        force dut.vio_spi_sel   = 1'b1;
        force dut.vio_spi_is_wr = tb_we;
        force dut.vio_spi_addr  = tb_addr;
        force dut.vio_spi_wdata = tb_wdata;
        spi_tgl = ~spi_tgl;
        force dut.vio_spi_go_tgl = spi_tgl;
        wait (dut.spi_done_tgl !== prev_done);
        @(posedge clk50);
        rdata = dut.spi_rdata;
    endtask

    logic [7:0] rd, r0, r1, r2, r3;
    initial begin
        force dut.vio_soft_rst      = 1'b0;
        force dut.vio_phy_loopback  = 1'b0;
        force dut.vio_spi_sel       = 1'b0;
        force dut.vio_spi_go_tgl    = 1'b0;
        force dut.vio_spi_is_wr     = 1'b0;
        force dut.vio_spi_addr      = 7'd0;
        force dut.vio_spi_wdata     = 8'd0;
        force dut.vio_sim_sensors   = 1'b0;
        force dut.vio_pktgen_sel    = 1'b0;
        force dut.vio_pktgen_go_tgl = 1'b0;
        force dut.vio_mon_clr_tgl   = 1'b0;

        repeat (20) @(posedge clk50);
        cpu_resetn = 1'b1;

        // 1. Wait for MDIO init
        wait (dut.mdio_init_done == 1'b1);
        repeat (500) @(posedge clk50);
        $display("MDIO readback: phy_id1=0x%04X phy_id2=0x%04X gbcr=0x%04X bmcr=0x%04X",
                 dut.phy_id1, dut.phy_id2, phy_regs[9], phy_regs[0]);
        check(dut.phy_id1 == 16'h001C && dut.phy_id2 == 16'hC915, "MDIO read RTL8211E PHY ID 001C:C915");
        check(phy_regs[9] == 16'h0000 && phy_regs[0] == 16'h1200, "MDIO disabled 1G adv & restarted AN");

        // Switch MDIO to loopback mode
        force dut.vio_phy_loopback = 1'b1;
        wait (phy_regs[0] == 16'h6100);
        check(phy_regs[0] == 16'h6100, "MDIO wrote 0x6100 (100M FD loopback)");

        // 2. Test Host SPI via VIO master
        vio_spi_xfer(1'b0, 7'h00, 8'h00, rd); check(rd == 8'hA7, "SPI ID reg 0x00 == 0xA7");
        vio_spi_xfer(1'b0, 7'h01, 8'h00, rd); check(rd == 8'h01, "SPI Ver reg 0x01 == 0x01");
        vio_spi_xfer(1'b0, 7'h02, 8'h00, rd); check(rd == 8'h43, "SPI Ctrl reg 0x02 == 0x43");

        // 3. Test synthetic VN-300 + AMT22 sensors through timestamper (shorten period to 1000 us for fast sim)
        vio_spi_xfer(1'b1, 7'h04, 8'hE8, rd); // period = 1000 us (0x03E8)
        vio_spi_xfer(1'b1, 7'h05, 8'h03, rd);
        vio_spi_xfer(1'b1, 7'h06, 8'h20, rd); // deadline = 800 us (0x0320)
        vio_spi_xfer(1'b1, 7'h07, 8'h03, rd);
        force dut.vio_sim_sensors = 1'b1;

        // Wait for 3 beats (first beat establishes dw_prev, second+ beats produce valid delta)
        wait (dut.meas_beat_cnt >= 16'd3);
        repeat (40000) @(posedge clk50); // wait for 3rd beat's VN message (0.65 ms)
        check(dut.meas_sync_width == 16'd500, "vn_sync pulse width == 500 clks (10 us)");
        vio_spi_xfer(1'b0, 7'h4C, 8'h00, rd);
        check(rd == 8'h03, "timestamper vn_valid=1 & dw_valid=1 over SPI");
        vio_spi_xfer(1'b0, 7'h50, 8'h00, r0);
        vio_spi_xfer(1'b0, 7'h51, 8'h00, r1);
        vio_spi_xfer(1'b0, 7'h52, 8'h00, r2);
        vio_spi_xfer(1'b0, 7'h53, 8'h00, r3);
        check({r3, r2, r1, r0} == 32'h42340000, "VN-300 yaw == 45.0f (0x42340000)");
        vio_spi_xfer(1'b0, 7'h68, 8'h00, r0);
        vio_spi_xfer(1'b0, 7'h69, 8'h00, r1);
        check({r1, r0} == 16'd450, "AMT22 dw_delta == 450 counts");

        // 4. Test real UKF + VLP-16 packet generator -> RGMII TX -> RGMII RX loopback -> lidar_top -> PC2X v3 verifier
`ifdef WITH_UKF
        // Restore default 400 Hz (2500 us) beat for UKF dt constant, enable UKF + motion correction (0xF3)
        vio_spi_xfer(1'b1, 7'h04, 8'hC4, rd); // period = 2500 us (0x09C4)
        vio_spi_xfer(1'b1, 7'h05, 8'h09, rd);
        vio_spi_xfer(1'b1, 7'h06, 8'hD0, rd); // deadline = 2000 us (0x07D0)
        vio_spi_xfer(1'b1, 7'h07, 8'h07, rd);
        vio_spi_xfer(1'b1, 7'h02, 8'hF3, rd); // enable UKF state source + rot/shift correction
        wait (dut.ukf_state_cnt >= 16'd3);
        @(posedge clk50);
        $display("UKF live state #%0d: overruns=%0d x=%f y=%f theta=%f omega=%f v=%f r=%f",
                 dut.ukf_state_cnt, dut.ukf_overruns,
                 $bitstoshortreal(dut.ukf_x), $bitstoshortreal(dut.ukf_y),
                 $bitstoshortreal(dut.ukf_theta), $bitstoshortreal(dut.ukf_omega),
                 $bitstoshortreal(dut.ukf_v), $bitstoshortreal(dut.ukf_r));
        check(dut.ukf_state_cnt >= 16'd3 && dut.ukf_overruns == 8'd0, "UKF produced >=3 states with 0 overruns");
        check($bitstoshortreal(dut.ukf_v) > 3.0 && $bitstoshortreal(dut.ukf_v) < 3.9, "UKF speed converged near 3.45 m/s");
`endif
        force dut.vio_pktgen_sel = 1'b1;
        repeat (20) @(posedge clk50);
        pg_tgl = ~pg_tgl;
        force dut.vio_pktgen_go_tgl = pg_tgl;
        wait (dut.mon_tx_frames >= 16'd8);
        repeat (5000) @(posedge clk50);

        $display("TX monitor: frames=%0d good_fcs=%0d bad_fcs=%0d pc2x_ok=%0d len=%0d flags=0x%02x p0=(%0d, %0d, %0d)",
                 dut.mon_tx_frames, dut.mon_tx_good_fcs, dut.mon_tx_bad_fcs, dut.mon_tx_pc2x_ok,
                 dut.mon_last_len, dut.mon_last_flags,
                 $signed(dut.mon_last_p0_x), $signed(dut.mon_last_p0_y), $signed(dut.mon_last_p0_z));
        check(dut.mon_tx_frames == 16'd8 && dut.mon_tx_good_fcs == 16'd8 && dut.mon_tx_bad_fcs == 16'd0,
              "8 PC2X frames with valid Ethernet FCS");
        check(dut.mon_tx_pc2x_ok == 16'd8 && dut.mon_last_len == 16'd1270,
              "All 8 frames valid PC2X v3 (1270 bytes)");
`ifdef WITH_UKF
        check(dut.mon_last_flags[0] == 1'b1, "PC2X v3 header carries live UKF state_valid=1");
`endif

        vio_spi_xfer(1'b0, 7'h40, 8'h00, r0); // stat_lidar_pkt
        vio_spi_xfer(1'b0, 7'h42, 8'h00, r1); // stat_rx_crc_bad
        vio_spi_xfer(1'b0, 7'h43, 8'h00, r2); // stat_frame_sent
        $display("SPI counters: lidar_pkt=%0d rx_crc_bad=%0d frame_sent=%0d", r0, r1, r2);
        check(r0 == 8'd4 && r1 == 8'd0 && r2 == 8'd8, "SPI counters: 4 lidar pkts in, 0 CRC bad, 8 frames out");

        $display("=== %0d checks, %0d errors ===", checks, errors);
        if (errors == 0) $display("FPGA_TOP SIM PASSED");
        else             $display("FPGA_TOP SIM FAILED");
        $finish;
    end
endmodule
