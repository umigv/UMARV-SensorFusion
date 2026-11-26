// Clause-22 MDIO controller for the Genesys 2 Realtek RTL8211E-VL PHY.
//
// Genesys 2 straps PHY address = 5'd1, RXDLY = 1 (+2 ns), TXDLY = 0 (0 ns).
// After PHY reset release and a boot delay:
//   1. Reads PHYID1 (0x02, expect 0x001C) and PHYID2 (0x03, expect 0xC915).
//   2. Configures the PHY:
//      - loopback_en == 0 (normal): disables 1000BASE-T advertisement (Reg 0x09 = 0x0000),
//        sets 10/100 advertisement (Reg 0x04 = 0x01E1), restarts auto-negotiation
//        (Reg 0x00 = 0x1200) so the PHY links at 100BASE-TX on any switch/router.
//      - loopback_en == 1 (bench self-test): disables 1000BASE-T (Reg 0x09 = 0x0000) and
//        forces 100M full-duplex PCS loopback (Reg 0x00 = 0x6100).
//   3. Continuously polls BMCR (0x00), BMSR (0x01), GBCR (0x09), and PHYSR (0x11).
//      Re-runs configuration immediately whenever loopback_en changes.
module mdio_init_100m #(
    parameter logic [4:0] PHY_ADDR      = 5'd1,
    parameter int         CLK_DIV       = 20,        // MDC half-period in 50 MHz clocks (20 -> 1.25 MHz)
    parameter int         BOOT_WAIT_CYC = 2_500_000, // 50 ms after PHY reset release
    parameter int         POLL_WAIT_CYC = 500_000    // 10 ms between status polls
) (
    input  logic        clk,
    input  logic        rst,            // active high, synchronous to clk
    input  logic        phy_rst_done,   // high once eth_phyrst_n is released
    input  logic        loopback_en,    // 0: normal 100M auto-neg, 1: 100M FD PHY loopback

    output logic        mdc,
    output logic        mdio_o,
    output logic        mdio_t,         // 1 = Hi-Z (input), 0 = drive mdio_o
    input  logic        mdio_i,

    output logic        init_done,
    output logic [15:0] phy_id1,
    output logic [15:0] phy_id2,
    output logic [15:0] reg_bmcr,
    output logic [15:0] reg_bmsr,
    output logic [15:0] reg_gbcr,
    output logic [15:0] reg_physr,
    output logic        link_up,
    output logic [1:0]  link_speed,     // 00: 10M, 01: 100M, 10: 1000M
    output logic        link_duplex     // 1: full duplex
);

    // ---- Low-level MDIO bit shifter (64 bits: 32 preamble + 32 frame) ---------
    logic        cmd_start;
    logic        cmd_is_wr;
    logic [4:0]  cmd_reg;
    logic [15:0] cmd_wdata;
    logic        cmd_busy;
    logic        cmd_done;
    logic [15:0] cmd_rdata;

    logic [$clog2(CLK_DIV+1)-1:0] div_cnt;
    logic [6:0]  bit_idx;       // 0..63
    logic        mdc_r;
    logic [63:0] tx_sr;
    logic [15:0] rx_sr;
    logic        op_wr;

    // Input synchronizer for mdio_i
    logic [1:0] mdio_i_s;
    always_ff @(posedge clk) begin
        if (rst) mdio_i_s <= 2'b11;
        else     mdio_i_s <= {mdio_i_s[0], mdio_i};
    end

    assign mdc = mdc_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            cmd_busy  <= 1'b0;
            cmd_done  <= 1'b0;
            cmd_rdata <= 16'd0;
            div_cnt   <= '0;
            bit_idx   <= 7'd0;
            mdc_r     <= 1'b0;
            tx_sr     <= 64'hFFFF_FFFF_FFFF_FFFF;
            rx_sr     <= 16'd0;
            op_wr     <= 1'b0;
            mdio_o    <= 1'b1;
            mdio_t    <= 1'b1;
        end else begin
            cmd_done <= 1'b0;

            if (!cmd_busy) begin
                mdc_r  <= 1'b0;
                mdio_t <= 1'b1;
                mdio_o <= 1'b1;
                if (cmd_start) begin
                    cmd_busy <= 1'b1;
                    op_wr    <= cmd_is_wr;
                    bit_idx  <= 7'd0;
                    div_cnt  <= '0;
                    // 32-bit preamble + ST(01) + OP(01=WR,10=RD) + PHYAD + REGAD + TA(10/11) + DATA
                    tx_sr    <= {32'hFFFF_FFFF, 2'b01, (cmd_is_wr ? 2'b01 : 2'b10),
                                 PHY_ADDR, cmd_reg, (cmd_is_wr ? 2'b10 : 2'b11),
                                 (cmd_is_wr ? cmd_wdata : 16'hFFFF)};
                    rx_sr    <= 16'd0;
                    mdio_o   <= 1'b1;
                    mdio_t   <= 1'b0;
                end
            end else begin
                if (div_cnt == CLK_DIV - 1) begin
                    div_cnt <= '0;
                    if (!mdc_r) begin
                        // Rising edge of MDC: PHY samples write data; we sample read data
                        mdc_r <= 1'b1;
                        if (!op_wr && (bit_idx >= 7'd48)) begin
                            rx_sr <= {rx_sr[14:0], mdio_i_s[1]};
                        end
                    end else begin
                        // Falling edge of MDC: advance to next bit
                        mdc_r <= 1'b0;
                        if (bit_idx == 7'd63) begin
                            cmd_busy  <= 1'b0;
                            cmd_done  <= 1'b1;
                            cmd_rdata <= rx_sr;
                            mdio_t    <= 1'b1;
                            mdio_o    <= 1'b1;
                        end else begin
                            bit_idx <= bit_idx + 7'd1;
                            tx_sr   <= {tx_sr[62:0], 1'b1};
                            mdio_o  <= tx_sr[62];
                            // For reads, release bus starting at TA bit 0 (bit_idx 46 -> next is 46)
                            if (!op_wr && (bit_idx + 7'd1 >= 7'd46))
                                mdio_t <= 1'b1;
                            else
                                mdio_t <= 1'b0;
                        end
                    end
                end else begin
                    div_cnt <= div_cnt + 1'b1;
                end
            end
        end
    end

    // ---- High-level sequencer ------------------------------------------------
    typedef enum logic [3:0] {
        ST_BOOT_WAIT,
        ST_RD_ID1,
        ST_RD_ID2,
        ST_WR_GBCR,
        ST_WR_ANAR,
        ST_WR_BMCR,
        ST_POLL_WAIT,
        ST_POLL_BMCR,
        ST_POLL_BMSR,
        ST_POLL_GBCR,
        ST_POLL_PHYSR
    } seq_t;

    seq_t        st;
    logic [23:0] wait_cnt;
    logic        issued;
    logic        lb_applied;

    always_ff @(posedge clk) begin
        if (rst || !phy_rst_done) begin
            st          <= ST_BOOT_WAIT;
            wait_cnt    <= 24'd0;
            issued      <= 1'b0;
            cmd_start   <= 1'b0;
            cmd_is_wr   <= 1'b0;
            cmd_reg     <= 5'd0;
            cmd_wdata   <= 16'd0;
            init_done   <= 1'b0;
            lb_applied  <= 1'b0;
            phy_id1     <= 16'd0;
            phy_id2     <= 16'd0;
            reg_bmcr    <= 16'd0;
            reg_bmsr    <= 16'd0;
            reg_gbcr    <= 16'd0;
            reg_physr   <= 16'd0;
        end else begin
            cmd_start <= 1'b0;

            case (st)
                ST_BOOT_WAIT: begin
                    if (wait_cnt >= BOOT_WAIT_CYC) begin
                        wait_cnt <= 24'd0;
                        issued   <= 1'b0;
                        st       <= ST_RD_ID1;
                    end else begin
                        wait_cnt <= wait_cnt + 24'd1;
                    end
                end

                ST_RD_ID1: begin
                    if (!issued) begin
                        cmd_start <= 1'b1; cmd_is_wr <= 1'b0; cmd_reg <= 5'h02; issued <= 1'b1;
                    end else if (cmd_done) begin
                        phy_id1 <= cmd_rdata; issued <= 1'b0; st <= ST_RD_ID2;
                    end
                end

                ST_RD_ID2: begin
                    if (!issued) begin
                        cmd_start <= 1'b1; cmd_is_wr <= 1'b0; cmd_reg <= 5'h03; issued <= 1'b1;
                    end else if (cmd_done) begin
                        phy_id2 <= cmd_rdata; issued <= 1'b0; st <= ST_WR_GBCR;
                    end
                end

                // Disable 1000BASE-T advertisement (Reg 0x09 = 0x0000)
                ST_WR_GBCR: begin
                    if (!issued) begin
                        cmd_start <= 1'b1; cmd_is_wr <= 1'b1; cmd_reg <= 5'h09; cmd_wdata <= 16'h0000;
                        issued <= 1'b1;
                    end else if (cmd_done) begin
                        issued <= 1'b0;
                        st     <= loopback_en ? ST_WR_BMCR : ST_WR_ANAR;
                    end
                end

                // Advertise 100BASE-TX FD/HD + 10BASE-T FD/HD (Reg 0x04 = 0x01E1)
                ST_WR_ANAR: begin
                    if (!issued) begin
                        cmd_start <= 1'b1; cmd_is_wr <= 1'b1; cmd_reg <= 5'h04; cmd_wdata <= 16'h01E1;
                        issued <= 1'b1;
                    end else if (cmd_done) begin
                        issued <= 1'b0; st <= ST_WR_BMCR;
                    end
                end

                // Normal: Auto-Neg Enable + Restart AN (0x1200)
                // Loopback: Loopback=1, 100M, AN=0, Full Duplex (0x6100)
                ST_WR_BMCR: begin
                    if (!issued) begin
                        cmd_start  <= 1'b1;
                        cmd_is_wr  <= 1'b1;
                        cmd_reg    <= 5'h00;
                        cmd_wdata  <= loopback_en ? 16'h6100 : 16'h1200;
                        lb_applied <= loopback_en;
                        issued     <= 1'b1;
                    end else if (cmd_done) begin
                        init_done <= 1'b1;
                        issued    <= 1'b0;
                        wait_cnt  <= 24'd0;
                        st        <= ST_POLL_BMCR;
                    end
                end

                ST_POLL_WAIT: begin
                    if (loopback_en != lb_applied) begin
                        issued <= 1'b0;
                        st     <= ST_WR_GBCR;
                    end else if (wait_cnt >= POLL_WAIT_CYC) begin
                        wait_cnt <= 24'd0;
                        issued   <= 1'b0;
                        st       <= ST_POLL_BMCR;
                    end else begin
                        wait_cnt <= wait_cnt + 24'd1;
                    end
                end

                ST_POLL_BMCR: begin
                    if (!issued) begin
                        cmd_start <= 1'b1; cmd_is_wr <= 1'b0; cmd_reg <= 5'h00; issued <= 1'b1;
                    end else if (cmd_done) begin
                        reg_bmcr <= cmd_rdata; issued <= 1'b0; st <= ST_POLL_BMSR;
                    end
                end

                ST_POLL_BMSR: begin
                    if (!issued) begin
                        cmd_start <= 1'b1; cmd_is_wr <= 1'b0; cmd_reg <= 5'h01; issued <= 1'b1;
                    end else if (cmd_done) begin
                        reg_bmsr <= cmd_rdata; issued <= 1'b0; st <= ST_POLL_GBCR;
                    end
                end

                ST_POLL_GBCR: begin
                    if (!issued) begin
                        cmd_start <= 1'b1; cmd_is_wr <= 1'b0; cmd_reg <= 5'h09; issued <= 1'b1;
                    end else if (cmd_done) begin
                        reg_gbcr <= cmd_rdata; issued <= 1'b0; st <= ST_POLL_PHYSR;
                    end
                end

                ST_POLL_PHYSR: begin
                    if (!issued) begin
                        cmd_start <= 1'b1; cmd_is_wr <= 1'b0; cmd_reg <= 5'h11; issued <= 1'b1;
                    end else if (cmd_done) begin
                        reg_physr <= cmd_rdata; issued <= 1'b0; wait_cnt <= 24'd0; st <= ST_POLL_WAIT;
                    end
                end

                default: st <= ST_BOOT_WAIT;
            endcase
        end
    end

    assign link_up     = reg_physr[10];
    assign link_speed  = reg_physr[15:14];
    assign link_duplex = reg_physr[13];

endmodule
