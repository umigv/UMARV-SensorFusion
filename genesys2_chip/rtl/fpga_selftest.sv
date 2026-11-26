// Non-intrusive JTAG/VIO bench self-test harness for Genesys 2 bring-up.
//
// All active stimulus generators are disabled by default (VIO outputs = 0),
// so out of reset the board talks directly to external Pmods and Ethernet.
// When enabled via Vivado VIO over JTAG, this block can verify on real silicon:
//   1. Timestamper beat period (expect 125,000 clks = 400 Hz), vn_sync width
//      (expect 500 clks = 10 us), and deadline delay (expect 100,000 clks).
//   2. Host SPI register file (real SPI Mode-0 waveforms on ui_in[2..4]/uo[5]).
//   3. Synthetic VN-300 (921,600 baud 56-byte binary msg + CRC16) and AMT22
//      (SPI Mode-0 with odd check bits) responders for on-chip sensor parser test.
//   4. Synthetic VLP-16 Ethernet frame generator (1252B with real Ethernet FCS)
//      driven out to the physical RTL8211E PHY in 100M loopback, plus a live
//      hardware verifier of the chip's outgoing PC2X v3 frames (FCS + header + XYZ).
module fpga_selftest (
    input  logic        clk50,
    input  logic        rst50,
    input  logic        clk25,
    input  logic        rst25,

    // ---- Passive monitors of chip outputs (50 MHz) ----------------------------
    input  logic        mon_vn_sync,
    input  logic        mon_dw_cs_n,
    input  logic        mon_dw_sck,
    input  logic        mon_dw_mosi,
    input  logic        mon_ukf_pulse,

    output logic [31:0] meas_beat_period,
    output logic [15:0] meas_sync_width,
    output logic [31:0] meas_deadline_clks,
    output logic [15:0] meas_beat_cnt,
    output logic [15:0] meas_dw_reads,

    // ---- VIO Host SPI Master (50 MHz) -----------------------------------------
    input  logic        spi_go_tgl,     // toggle to start a 2-byte SPI transfer
    input  logic        spi_is_wr,
    input  logic [6:0]  spi_addr,
    input  logic [7:0]  spi_wdata,
    output logic [7:0]  spi_rdata,
    output logic        spi_done_tgl,

    output logic        test_spi_sck,
    output logic        test_spi_cs_n,
    output logic        test_spi_mosi,
    input  logic        chip_spi_miso,

    // ---- Synthetic VN-300 & AMT22 responders (50 MHz) -------------------------
    input  logic        sim_sensors_en,
    output logic        sim_vn_rx,
    output logic        sim_dw_miso,

    // ---- Synthetic VLP-16 Ethernet generator (25 MHz clk25) -------------------
    input  logic        pktgen_go_tgl,  // toggle (50 MHz) to send 4 VLP-16 frames
    output logic        pktgen_tx_en,
    output logic [3:0]  pktgen_txd,
    output logic [7:0]  pktgen_sent_cnt,

    // ---- Chip MII TX verifier (25 MHz clk25) ----------------------------------
    input  logic        chip_tx_en,
    input  logic [3:0]  chip_txd,
    input  logic        mon_clr_tgl,    // toggle (50 MHz) to clear TX verifier counters
    output logic [15:0] mon_tx_frames,
    output logic [15:0] mon_tx_good_fcs,
    output logic [15:0] mon_tx_bad_fcs,
    output logic [15:0] mon_tx_pc2x_ok,
    output logic [7:0]  mon_last_flags,
    output logic [15:0] mon_last_len,
    output logic [15:0] mon_last_p0_x,
    output logic [15:0] mon_last_p0_y,
    output logic [15:0] mon_last_p0_z
);

    // =========================================================================
    // 1. Timestamper passive period / pulse-width / deadline monitor (50 MHz)
    // =========================================================================
    logic sync_d, cs_d;
    logic [31:0] beat_timer, dead_timer;
    logic [15:0] sync_timer;
    logic        have_sync, in_dead;

    always_ff @(posedge clk50) begin
        if (rst50) begin
            sync_d <= 1'b0; cs_d <= 1'b1;
            beat_timer <= 32'd0; dead_timer <= 32'd0; sync_timer <= 16'd0;
            have_sync <= 1'b0; in_dead <= 1'b0;
            meas_beat_period <= 32'd0; meas_sync_width <= 16'd0;
            meas_deadline_clks <= 32'd0; meas_beat_cnt <= 16'd0; meas_dw_reads <= 16'd0;
        end else begin
            sync_d <= mon_vn_sync;
            cs_d   <= mon_dw_cs_n;

            if (have_sync) beat_timer <= beat_timer + 32'd1;
            if (mon_vn_sync && !sync_d) begin
                if (have_sync) meas_beat_period <= beat_timer + 32'd1;
                beat_timer    <= 32'd0;
                have_sync     <= 1'b1;
                sync_timer    <= 16'd1;
                dead_timer    <= 32'd0;
                in_dead       <= 1'b1;
                meas_beat_cnt <= meas_beat_cnt + 16'd1;
            end else if (mon_vn_sync) begin
                sync_timer <= sync_timer + 16'd1;
            end else if (sync_d && !mon_vn_sync) begin
                meas_sync_width <= sync_timer;
            end

            if (in_dead) begin
                if (mon_ukf_pulse) begin
                    meas_deadline_clks <= dead_timer + 32'd1;
                    in_dead <= 1'b0;
                end else begin
                    dead_timer <= dead_timer + 32'd1;
                end
            end

            if (cs_d && !mon_dw_cs_n) begin
                meas_dw_reads <= meas_dw_reads + 16'd1;
            end
        end
    end

    // =========================================================================
    // 2. VIO Host SPI Mode-0 Master (2 bytes: {we, addr}, data) @ 1.56 MHz
    // =========================================================================
    logic        spi_go_d;
    logic [1:0]  spi_st;
    logic [4:0]  spi_div;
    logic [4:0]  spi_bit;       // 0..15
    logic [15:0] spi_tx_sh;
    logic [7:0]  spi_rx_sh;

    always_ff @(posedge clk50) begin
        if (rst50) begin
            spi_go_d      <= 1'b0;
            spi_st        <= 2'd0;
            spi_div       <= 5'd0;
            spi_bit       <= 5'd0;
            spi_tx_sh     <= 16'd0;
            spi_rx_sh     <= 8'd0;
            test_spi_cs_n <= 1'b1;
            test_spi_sck  <= 1'b0;
            test_spi_mosi <= 1'b0;
            spi_rdata     <= 8'd0;
            spi_done_tgl  <= 1'b0;
        end else begin
            spi_go_d <= spi_go_tgl;
            case (spi_st)
                2'd0: begin
                    test_spi_cs_n <= 1'b1;
                    test_spi_sck  <= 1'b0;
                    if (spi_go_tgl != spi_go_d) begin
                        spi_tx_sh     <= {spi_is_wr, spi_addr, spi_wdata};
                        test_spi_cs_n <= 1'b0;
                        test_spi_mosi <= spi_is_wr;
                        spi_bit       <= 5'd0;
                        spi_div       <= 5'd0;
                        spi_st        <= 2'd1;
                    end
                end
                // Low half of SCK
                2'd1: begin
                    test_spi_sck <= 1'b0;
                    if (spi_div == 5'd15) begin
                        spi_div      <= 5'd0;
                        test_spi_sck <= 1'b1;
                        spi_rx_sh    <= {spi_rx_sh[6:0], chip_spi_miso};
                        spi_st       <= 2'd2;
                    end else begin
                        spi_div <= spi_div + 5'd1;
                    end
                end
                // High half of SCK
                2'd2: begin
                    test_spi_sck <= 1'b1;
                    if (spi_div == 5'd15) begin
                        spi_div      <= 5'd0;
                        test_spi_sck <= 1'b0;
                        if (spi_bit == 5'd15) begin
                            spi_st <= 2'd3;
                        end else begin
                            spi_bit       <= spi_bit + 5'd1;
                            spi_tx_sh     <= {spi_tx_sh[14:0], 1'b0};
                            test_spi_mosi <= spi_tx_sh[14];
                            spi_st        <= 2'd1;
                        end
                    end else begin
                        spi_div <= spi_div + 5'd1;
                    end
                end
                // CS hold before release
                2'd3: begin
                    test_spi_sck <= 1'b0;
                    if (spi_div == 5'd15) begin
                        test_spi_cs_n <= 1'b1;
                        spi_rdata     <= spi_rx_sh;
                        spi_done_tgl  <= !spi_done_tgl;
                        spi_st        <= 2'd0;
                    end else begin
                        spi_div <= spi_div + 5'd1;
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // 3a. Synthetic AMT22 responder (steps +450 counts per read, odd check bits)
    // =========================================================================
    function automatic [15:0] amt_pack(input [13:0] d);
        reg k1, k0;
        begin
            k1 = !(d[13]^d[11]^d[9]^d[7]^d[5]^d[3]^d[1]);
            k0 = !(d[12]^d[10]^d[8]^d[6]^d[4]^d[2]^d[0]);
            amt_pack = {k1, k0, d};
        end
    endfunction

    logic [27:0] sim_wheel;
    logic [31:0] amt_sh;
    logic        amt_cs_d, amt_sck_d;
    wire  [27:0] wheel_next = sim_wheel + 28'd450;
    wire  [15:0] amt_w_pos  = amt_pack(wheel_next[13:0]);
    wire  [15:0] amt_w_trn  = amt_pack(wheel_next[27:14]);

    always_ff @(posedge clk50) begin
        if (rst50 || !sim_sensors_en) begin
            sim_wheel   <= 28'h0003F00;
            amt_sh      <= 32'd0;
            amt_cs_d    <= 1'b1;
            amt_sck_d   <= 1'b0;
            sim_dw_miso <= 1'b0;
        end else begin
            amt_cs_d  <= mon_dw_cs_n;
            amt_sck_d <= mon_dw_sck;
            if (amt_cs_d && !mon_dw_cs_n) begin
                // On CS_N falling edge, increment wheel by +450 and load 32-bit response
                sim_wheel   <= wheel_next;
                amt_sh      <= {amt_w_pos, amt_w_trn};
                sim_dw_miso <= amt_w_pos[15];
            end else if (!mon_dw_cs_n && amt_sck_d && !mon_dw_sck) begin
                // Shift out next bit on SCK falling edge (SPI Mode 0)
                amt_sh      <= {amt_sh[30:0], 1'b0};
                sim_dw_miso <= amt_sh[30];
            end
        end
    end

    // =========================================================================
    // 3b. Synthetic VN-300 921,600-baud responder (56-byte binary msg + CRC16)
    // =========================================================================
    function automatic [15:0] vn_crc_next(input [15:0] c, input [7:0] d);
        reg [15:0] x;
        begin
            x = {c[7:0], c[15:8]};
            x = x ^ {8'h00, d};
            x = x ^ {12'h000, x[7:4]};
            x = x ^ {x[3:0], 12'h000};
            x = x ^ {3'b000, x[7:0], 5'b00000};
            vn_crc_next = x;
        end
    endfunction

    function automatic [7:0] vn_payload_byte(input [5:0] idx);
        begin
            case (idx)
                6'd0:  vn_payload_byte = 8'hFA;
                6'd1:  vn_payload_byte = 8'h01;
                6'd2:  vn_payload_byte = 8'h68;
                6'd3:  vn_payload_byte = 8'h10;
                // yaw = 45.0f (0x42340000), LE
                6'd4:  vn_payload_byte = 8'h00; 6'd5:  vn_payload_byte = 8'h00;
                6'd6:  vn_payload_byte = 8'h34; 6'd7:  vn_payload_byte = 8'h42;
                // rate = 1.0f (0x3F800000), LE
                6'd24: vn_payload_byte = 8'h00; 6'd25: vn_payload_byte = 8'h00;
                6'd26: vn_payload_byte = 8'h80; 6'd27: vn_payload_byte = 8'h3F;
                // lat = 42.29365d (0x4045259652BD3C36), LE
                6'd28: vn_payload_byte = 8'h36; 6'd29: vn_payload_byte = 8'h3C;
                6'd30: vn_payload_byte = 8'hBD; 6'd31: vn_payload_byte = 8'h52;
                6'd32: vn_payload_byte = 8'h96; 6'd33: vn_payload_byte = 8'h25;
                6'd34: vn_payload_byte = 8'h45; 6'd35: vn_payload_byte = 8'h40;
                // lon = -83.712d (0xC054ED916872B021), LE
                6'd36: vn_payload_byte = 8'h21; 6'd37: vn_payload_byte = 8'hB0;
                6'd38: vn_payload_byte = 8'h72; 6'd39: vn_payload_byte = 8'h68;
                6'd40: vn_payload_byte = 8'h91; 6'd41: vn_payload_byte = 8'hED;
                6'd42: vn_payload_byte = 8'h54; 6'd43: vn_payload_byte = 8'hC0;
                // INS status = 0x0306 (mode=2 tracking, GNSS fix=1, error=0)
                6'd52: vn_payload_byte = 8'h06; 6'd53: vn_payload_byte = 8'h03;
                default: vn_payload_byte = 8'h00;
            endcase
        end
    endfunction

    logic [1:0]  vn_st;
    logic [11:0] vn_delay;
    logic [5:0]  vn_bidx;
    logic [3:0]  vn_bit;
    logic [25:0] vn_baud_acc;   // exact 921,600 baud fractional accumulator
    logic [9:0]  vn_frame;
    logic [15:0] vn_crc;
    wire  [15:0] vn_crc_53 = vn_crc_next(vn_crc, vn_payload_byte(6'd53));

    always_ff @(posedge clk50) begin
        if (rst50 || !sim_sensors_en) begin
            vn_st       <= 2'd0;
            vn_delay    <= 12'd0;
            vn_bidx     <= 6'd0;
            vn_bit      <= 4'd0;
            vn_baud_acc <= 26'd0;
            vn_frame    <= 10'h3FF;
            vn_crc      <= 16'd0;
            sim_vn_rx   <= 1'b1;
        end else begin
            case (vn_st)
                2'd0: begin
                    sim_vn_rx <= 1'b1;
                    if (mon_vn_sync && !sync_d) begin
                        vn_delay <= 12'd0;
                        vn_st    <= 2'd1;
                    end
                end
                // 50 us response delay after sync pulse
                2'd1: begin
                    sim_vn_rx <= 1'b1;
                    if (vn_delay == 12'd2500) begin
                        vn_bidx     <= 6'd0;
                        vn_crc      <= 16'd0;
                        vn_frame    <= {1'b1, 8'hFA, 1'b0};
                        vn_bit      <= 4'd0;
                        vn_baud_acc <= 26'd0;
                        sim_vn_rx   <= 1'b0;    // start bit
                        vn_st       <= 2'd2;
                    end else begin
                        vn_delay <= vn_delay + 12'd1;
                    end
                end
                // Shift 56 bytes at exact 921,600 baud (10 bits/byte)
                2'd2: begin
                    if (vn_baud_acc + 26'd921_600 >= 26'd50_000_000) begin
                        vn_baud_acc <= vn_baud_acc + 26'd921_600 - 26'd50_000_000;
                        if (vn_bit == 4'd9) begin
                            if (vn_bidx >= 6'd1 && vn_bidx <= 6'd53)
                                vn_crc <= vn_crc_next(vn_crc, vn_payload_byte(vn_bidx));
                            if (vn_bidx == 6'd55) begin
                                sim_vn_rx <= 1'b1;
                                vn_st     <= 2'd0;
                            end else begin
                                vn_bidx <= vn_bidx + 6'd1;
                                vn_bit  <= 4'd0;
                                // Load next byte; compute final CRC for byte 54 boundary
                                if (vn_bidx == 6'd53) begin
                                    vn_frame <= {1'b1, vn_crc_53[15:8], 1'b0};
                                end else if (vn_bidx == 6'd54) begin
                                    vn_frame <= {1'b1, vn_crc[7:0], 1'b0};
                                end else begin
                                    vn_frame <= {1'b1, vn_payload_byte(vn_bidx + 6'd1), 1'b0};
                                end
                                sim_vn_rx <= 1'b0;
                            end
                        end else begin
                            vn_bit    <= vn_bit + 4'd1;
                            vn_frame  <= {1'b1, vn_frame[9:1]};
                            sim_vn_rx <= vn_frame[1];
                        end
                    end else begin
                        vn_baud_acc <= vn_baud_acc + 26'd921_600;
                    end
                end
                default: vn_st <= 2'd0;
            endcase
        end
    end

    // =========================================================================
    // 4. Synthetic VLP-16 Ethernet frame generator (25 MHz clk25)
    // =========================================================================
    function automatic [31:0] eth_crc_bit(input [31:0] c, input d);
        eth_crc_bit = (c[0] ^ d) ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
    endfunction
    function automatic [31:0] eth_crc_nib(input [31:0] c, input [3:0] d);
        eth_crc_nib = eth_crc_bit(eth_crc_bit(eth_crc_bit(eth_crc_bit(c, d[0]), d[1]), d[2]), d[3]);
    endfunction

    // Synchronize toggle from clk50 -> clk25
    (* ASYNC_REG = "TRUE" *) logic [2:0] pg_go_s;
    always_ff @(posedge clk25) begin
        if (rst25) pg_go_s <= 3'd0;
        else       pg_go_s <= {pg_go_s[1:0], pktgen_go_tgl};
    end
    wire pg_trigger = pg_go_s[2] ^ pg_go_s[1];

    logic [2:0]  pg_st;
    logic [2:0]  pg_pkt_rem;
    logic [10:0] pg_bidx;       // 0..1247
    logic [3:0]  pg_blk_i;      // 0..11
    logic [6:0]  pg_blk_off;    // 0..99
    logic [15:0] pg_az;
    logic [4:0]  pg_cnt;
    logic        pg_phase;      // 0: low nibble, 1: high nibble
    logic [15:0] pg_gap;
    logic [31:0] pg_crc, pg_fcs_sr;
    logic [7:0]  pg_byte;

    // Construct VLP-16 Ethernet/IPv4/UDP byte on the fly
    always_comb begin
        pg_byte = 8'h00;
        if (pg_bidx < 11'd42) begin
            case (pg_bidx)
                11'd0,11'd1,11'd2,11'd3,11'd4,11'd5: pg_byte = 8'hFF; // dst MAC broadcast
                11'd6:  pg_byte = 8'h60; 11'd7:  pg_byte = 8'h76; 11'd8: pg_byte = 8'h88;
                11'd9:  pg_byte = 8'h00; 11'd10: pg_byte = 8'h00; 11'd11: pg_byte = 8'h01;
                11'd12: pg_byte = 8'h08; 11'd13: pg_byte = 8'h00;     // IPv4
                11'd14: pg_byte = 8'h45; 11'd15: pg_byte = 8'h00;
                11'd16: pg_byte = 8'h04; 11'd17: pg_byte = 8'hD2;     // IP len = 1234
                11'd20: pg_byte = 8'h40; 11'd21: pg_byte = 8'h00;     // DF
                11'd22: pg_byte = 8'h40; 11'd23: pg_byte = 8'h11;     // TTL 64, UDP
                11'd26: pg_byte = 8'd192; 11'd27: pg_byte = 8'd168; 11'd28: pg_byte = 8'd1; 11'd29: pg_byte = 8'd201;
                11'd30: pg_byte = 8'd255; 11'd31: pg_byte = 8'd255; 11'd32: pg_byte = 8'd255; 11'd33: pg_byte = 8'd255;
                11'd34: pg_byte = 8'h09; 11'd35: pg_byte = 8'h40;     // src port 2368
                11'd36: pg_byte = 8'h09; 11'd37: pg_byte = 8'h40;     // dst port 2368
                11'd38: pg_byte = 8'h04; 11'd39: pg_byte = 8'hBE;     // UDP len 1214
                default: pg_byte = 8'h00;
            endcase
        end else if (pg_bidx < 11'd1242) begin
            case (pg_blk_off)
                7'd0: pg_byte = 8'hFF;
                7'd1: pg_byte = 8'hEE;
                7'd2: pg_byte = pg_az[7:0];
                7'd3: pg_byte = pg_az[15:8];
                // Point 0 (bytes 4,5,6): distance = 5000 (0x1388 -> 10.000 m), refl = 0x80
                7'd4: pg_byte = 8'h88;
                7'd5: pg_byte = 8'h13;
                7'd6: pg_byte = 8'h80;
                default: begin
                    // Remaining points: distance = 2500 (0x09C4 -> 5.000 m)
                    if ((pg_blk_off - 7'd4) % 3 == 0)      pg_byte = 8'hC4;
                    else if ((pg_blk_off - 7'd4) % 3 == 1) pg_byte = 8'h09;
                    else                                   pg_byte = 8'h40;
                end
            endcase
        end else begin
            case (pg_bidx)
                11'd1246: pg_byte = 8'h37; // strongest return
                11'd1247: pg_byte = 8'h22; // VLP-16 product ID
                default:  pg_byte = 8'h00;
            endcase
        end
    end

    wire [3:0] pg_nib = pg_phase ? pg_byte[7:4] : pg_byte[3:0];

    always_ff @(posedge clk25) begin
        if (rst25) begin
            pg_st <= 3'd0; pg_pkt_rem <= 3'd0; pg_bidx <= 11'd0; pg_blk_i <= 4'd0;
            pg_blk_off <= 7'd0; pg_az <= 16'd0; pg_cnt <= 5'd0; pg_phase <= 1'b0;
            pg_gap <= 16'd0; pg_crc <= 32'hFFFF_FFFF; pg_fcs_sr <= 32'd0;
            pktgen_tx_en <= 1'b0; pktgen_txd <= 4'h0; pktgen_sent_cnt <= 8'd0;
        end else begin
            case (pg_st)
                3'd0: begin
                    pktgen_tx_en <= 1'b0; pktgen_txd <= 4'h0;
                    if (pg_trigger) begin
                        pg_pkt_rem <= 3'd4;         // send 4 VLP-16 packets (produces 8 PC2X frames)
                        pg_az      <= 16'd0;
                        pg_cnt     <= 5'd0;
                        pg_st      <= 3'd1;
                    end
                end
                // Preamble (15x 5 + 1x D)
                3'd1: begin
                    pktgen_tx_en <= 1'b1;
                    pktgen_txd   <= (pg_cnt == 5'd15) ? 4'hD : 4'h5;
                    pg_cnt       <= pg_cnt + 5'd1;
                    if (pg_cnt == 5'd15) begin
                        pg_bidx    <= 11'd0;
                        pg_blk_i   <= 4'd0;
                        pg_blk_off <= 7'd0;
                        pg_phase   <= 1'b0;
                        pg_crc     <= 32'hFFFF_FFFF;
                        pg_st      <= 3'd2;
                    end
                end
                // 1248 payload bytes (low nibble then high nibble)
                3'd2: begin
                    pktgen_tx_en <= 1'b1;
                    pktgen_txd   <= pg_nib;
                    pg_crc       <= eth_crc_nib(pg_crc, pg_nib);
                    pg_phase     <= !pg_phase;
                    if (pg_phase) begin
                        if (pg_bidx == 11'd1247) begin
                            pg_fcs_sr <= ~eth_crc_nib(pg_crc, pg_nib);
                            pg_cnt    <= 5'd0;
                            pg_st     <= 3'd3;
                        end else begin
                            pg_bidx <= pg_bidx + 11'd1;
                            if (pg_bidx >= 11'd42 && pg_bidx < 11'd1242) begin
                                if (pg_blk_off == 7'd99) begin
                                    pg_blk_off <= 7'd0;
                                    pg_blk_i   <= pg_blk_i + 4'd1;
                                    pg_az      <= (pg_az >= 16'd35960) ? (pg_az - 16'd35960) : (pg_az + 16'd40);
                                end else begin
                                    pg_blk_off <= pg_blk_off + 7'd1;
                                end
                            end
                        end
                    end
                end
                // 8 FCS nibbles
                3'd3: begin
                    pktgen_tx_en <= 1'b1;
                    pktgen_txd   <= pg_fcs_sr[3:0];
                    pg_fcs_sr    <= {4'h0, pg_fcs_sr[31:4]};
                    pg_cnt       <= pg_cnt + 5'd1;
                    if (pg_cnt == 5'd7) begin
                        pktgen_sent_cnt <= pktgen_sent_cnt + 8'd1;
                        pg_gap          <= 16'd0;
                        pg_st           <= 3'd4;
                    end
                end
                // Inter-packet gap (~1.33 ms = 33,250 clocks @ 25 MHz)
                3'd4: begin
                    pktgen_tx_en <= 1'b0; pktgen_txd <= 4'h0;
                    if (pg_gap == 16'd33250) begin
                        if (pg_pkt_rem <= 3'd1) begin
                            pg_st <= 3'd0;
                        end else begin
                            pg_pkt_rem <= pg_pkt_rem - 3'd1;
                            pg_cnt     <= 5'd0;
                            pg_st      <= 3'd1;
                        end
                    end else begin
                        pg_gap <= pg_gap + 16'd1;
                    end
                end
                default: pg_st <= 3'd0;
            endcase
        end
    end

    // =========================================================================
    // 5. Chip MII TX verifier (25 MHz clk25): checks FCS, length, PC2X v3, XYZ
    // =========================================================================
    (* ASYNC_REG = "TRUE" *) logic [2:0] mon_clr_s;
    always_ff @(posedge clk25) begin
        if (rst25) mon_clr_s <= 3'd0;
        else       mon_clr_s <= {mon_clr_s[1:0], mon_clr_tgl};
    end
    wire mon_clr = mon_clr_s[2] ^ mon_clr_s[1];

    logic [1:0]  m_st;
    logic        m_phase;
    logic [3:0]  m_lo;
    logic [10:0] m_bcnt;
    logic [31:0] m_crc;
    logic [31:0] m_magic;
    logic [7:0]  m_ver, m_flags;
    logic [15:0] m_p0_x, m_p0_y, m_p0_z;
    wire  [7:0]  m_byte = {chip_txd, m_lo};

    always_ff @(posedge clk25) begin
        if (rst25 || mon_clr) begin
            m_st <= 2'd0; m_phase <= 1'b0; m_lo <= 4'h0; m_bcnt <= 11'd0;
            m_crc <= 32'hFFFF_FFFF; m_magic <= 32'd0; m_ver <= 8'd0; m_flags <= 8'd0;
            m_p0_x <= 16'd0; m_p0_y <= 16'd0; m_p0_z <= 16'd0;
            mon_tx_frames <= 16'd0; mon_tx_good_fcs <= 16'd0; mon_tx_bad_fcs <= 16'd0;
            mon_tx_pc2x_ok <= 16'd0; mon_last_flags <= 8'd0; mon_last_len <= 16'd0;
            mon_last_p0_x <= 16'd0; mon_last_p0_y <= 16'd0; mon_last_p0_z <= 16'd0;
        end else begin
            case (m_st)
                2'd0: begin
                    if (chip_tx_en && chip_txd == 4'hD) begin
                        m_st    <= 2'd1;
                        m_phase <= 1'b0;
                        m_bcnt  <= 11'd0;
                        m_crc   <= 32'hFFFF_FFFF;
                    end
                end
                2'd1: begin
                    if (!chip_tx_en) begin
                        mon_tx_frames <= mon_tx_frames + 16'd1;
                        mon_last_len  <= {5'd0, m_bcnt};
                        if ((m_crc == 32'hDEBB20E3) && !m_phase) begin
                            mon_tx_good_fcs <= mon_tx_good_fcs + 16'd1;
                            if ((m_bcnt == 11'd1270) && (m_magic == 32'h50433258) && (m_ver == 8'd3)) begin
                                mon_tx_pc2x_ok <= mon_tx_pc2x_ok + 16'd1;
                                mon_last_flags <= m_flags;
                                mon_last_p0_x  <= m_p0_x;
                                mon_last_p0_y  <= m_p0_y;
                                mon_last_p0_z  <= m_p0_z;
                            end
                        end else begin
                            mon_tx_bad_fcs <= mon_tx_bad_fcs + 16'd1;
                        end
                        m_st <= 2'd0;
                    end else begin
                        m_crc   <= eth_crc_nib(m_crc, chip_txd);
                        m_phase <= !m_phase;
                        if (!m_phase) begin
                            m_lo <= chip_txd;
                        end else begin
                            m_bcnt <= m_bcnt + 11'd1;
                            case (m_bcnt)
                                11'd42: m_magic[31:24] <= m_byte;
                                11'd43: m_magic[23:16] <= m_byte;
                                11'd44: m_magic[15:8]  <= m_byte;
                                11'd45: m_magic[7:0]   <= m_byte;
                                11'd46: m_ver          <= m_byte;
                                11'd47: m_flags        <= m_byte;
                                11'd94: m_p0_x[7:0]    <= m_byte;
                                11'd95: m_p0_x[15:8]   <= m_byte;
                                11'd96: m_p0_y[7:0]    <= m_byte;
                                11'd97: m_p0_y[15:8]   <= m_byte;
                                11'd98: m_p0_z[7:0]    <= m_byte;
                                11'd99: m_p0_z[15:8]   <= m_byte;
                                default: ;
                            endcase
                        end
                    end
                end
                default: m_st <= 2'd0;
            endcase
        end
    end

endmodule
