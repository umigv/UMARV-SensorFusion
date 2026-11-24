// Test of the whole lidar block, from the MII receive pins to the MII transmit pins.
//
// A model of the lidar-side PHY sends VLP-16 packets and other frames on a 25 MHz clock; a model
// of the host-side PHY collects what the block transmits on another 25 MHz clock. Neither clock
// is related to the chip's 50 MHz clock; both periods are changed between runs
// (plusargs +rxper= and +txper=, picoseconds per half period).
//
// Checked:
//   - every point out of the parser against what was sent
//   - every firing's sine, cosine and shift against the same motion worked out in real numbers
//   - every transmitted frame: preamble, checksum, Ethernet/IP/UDP headers, payload header, and
//     every x, y, z against real-number arithmetic on the distances that were sent
//   - bypass: frames come out byte for byte, a damaged frame comes out damaged
module tb_lidar_top;
    logic clk = 0; always #10000 clk = ~clk;                 // 50 MHz
    int rx_half = 20003, tx_half = 19996;
    logic rx_clk = 0; initial begin void'($value$plusargs("rxper=%d", rx_half)); #3777; forever #(rx_half) rx_clk = ~rx_clk; end
    logic tx_clk = 0; initial begin void'($value$plusargs("txper=%d", tx_half)); #9123; forever #(tx_half) tx_clk = ~tx_clk; end
    logic rst_n = 0, enable = 1, bypass = 0, rx_dv = 0; logic [3:0] rxd = 0;
    logic mii_tx_en; logic [3:0] mii_txd; logic s_lidar, s_other, s_crc, s_sent, s_ovf, s_mot, s_und;
    logic state_valid = 0; logic [31:0] state_omega = 0, state_v = 0;
    logic [31:0] time_us = 0; always @(posedge clk) time_us <= time_us + 1;
    localparam [47:0] SMAC = 48'h02_00_00_AA_BB_01, DMAC = 48'hFF_FF_FF_FF_FF_FF;
    localparam [31:0] SIP = 32'hC0A80164, DIP = 32'hC0A801FF, S_TS = 32'h12345678;
    localparam real SX = 1.5, SY = -2.25, STH = 0.5;
    lidar_top dut (.clk, .rst_n, .enable, .bypass, .lidar_port(16'd2368), .scale_4mm(1'b0),
        .lidar_model(2'd0), .yaw_offset(16'sd0), .corr_rot_en(1'b1), .corr_shift_en(1'b1),
        .src_mac(SMAC), .dst_mac(DMAC), .src_ip(SIP), .dst_ip(DIP), .src_port(16'd4000), .dst_port(16'd4001),
        .mii_rx_clk(rx_clk), .mii_rx_dv(rx_dv), .mii_rxd(rxd), .mii_tx_clk(tx_clk), .mii_tx_en, .mii_txd,
        .state_valid, .state_ts(S_TS), .state_x($shortrealtobits(shortreal'(SX))), .state_y($shortrealtobits(shortreal'(SY))),
        .state_theta($shortrealtobits(shortreal'(STH))), .state_omega, .state_v,
        .time_us, .stat_lidar_pkt(s_lidar), .stat_other_pkt(s_other), .stat_rx_crc_bad(s_crc), .stat_frame_sent(s_sent),
        .stat_buf_overflow(s_ovf), .stat_motion_overrun(s_mot), .stat_tx_underrun(s_und));

    function automatic logic [31:0] crc32(input byte unsigned q[$], input int n);
        logic [31:0] c = '1;
        for (int i = 0; i < n; i++) begin c ^= q[i]; repeat (8) c = c[0] ? (c >> 1) ^ 32'hEDB88320 : (c >> 1); end
        return ~c;
    endfunction
    task automatic nib(input logic [3:0] x); @(negedge rx_clk); rx_dv = 1; rxd = x; endtask
    task automatic gap(input int n); @(negedge rx_clk); rx_dv = 0; rxd = $urandom; repeat (n) @(negedge rx_clk); endtask
    task automatic send(input byte unsigned pay[$], input bit corrupt);
        byte unsigned q[$]; logic [31:0] c;
        q = pay; c = crc32(pay, pay.size()); for (int i = 0; i < 4; i++) q.push_back(c[8*i +: 8]);
        if (corrupt) q[q.size() - 2] ^= 8'h01;               // damage the checksum, not the data
        repeat (15) nib(4'h5); nib(4'hD);
        foreach (q[i]) begin nib(q[i][3:0]); nib(q[i][7:4]); end
        gap(24);
    endtask

    // ---- what is sent ---------------------------------------------------------------------------
    logic [43:0] exp[$];            // expected parser output, in order: {azimuth, distance, reflectivity, laser}
    int ref_az[$], ref_d[$];        // expected blocks, in order: azimuth, then 32 distances each
    int az = 35000;
    function automatic void lidar_packet(output byte unsigned f[$]);
        int a, d, r;
        f = {};
        for (int i = 0; i < 42 + 1206; i++) f.push_back($urandom);
        f[12] = 8'h08; f[13] = 8'h00; f[14] = 8'h45; f[20] = 8'h40; f[21] = 8'h00; f[23] = 8'h11; f[36] = 8'h09; f[37] = 8'h40; f[38] = 8'h04; f[39] = 8'hBE;
        for (int b = 0; b < 12; b++) begin
            a = az % 36000; az += 40;
            f[42 + 100*b] = 8'hFF; f[43 + 100*b] = 8'hEE; f[44 + 100*b] = a[7:0]; f[45 + 100*b] = a[15:8];
            ref_az.push_back(a);
            for (int p = 0; p < 32; p++) begin
                r = $urandom_range(0, 31);
                d = (r == 0) ? 0 : (r == 1) ? $urandom_range(50000, 65535) : $urandom_range(1, 15000);    // none, far, ordinary
                f[42 + 100*b + 4 + 3*p] = d[7:0]; f[42 + 100*b + 5 + 3*p] = d[15:8];
                ref_d.push_back(d);
                exp.push_back({a[15:0], d[15:0], f[42 + 100*b + 6 + 3*p], 4'(p % 16)});
            end
        end
    endfunction

    // ---- parser output and motion results (inside the block) -----------------------------------
    int pts = 0, wrong = 0, pk = 0, other = 0, crc_bad = 0, sweeps = 0, overflow = 0, sent = 0, buf_ovf = 0, underrun = 0;
    always @(posedge clk) if (rst_n) begin
        if (dut.pt_valid) begin
            pts++;
            if (exp.size() == 0 || {dut.pt_azimuth, dut.pt_distance, dut.pt_reflectivity, dut.pt_laser_id} !== exp.pop_front()) wrong++;
        end
        pk += s_lidar; other += s_other; crc_bad += s_crc; sent += s_sent; buf_ovf += s_ovf; underrun += s_und;
    end
    always @(posedge rx_clk) if (dut.mii_valid && dut.rx_fifo_full) overflow++;

    localparam real W = 1.0, V = 2.2, TF = 55.296e-6, PI = 3.14159265358979;
    real T = 0, X = 0, Y = 0, worst_ang = 0, worst_shift = 0, th, d; int fired = 0, mot_overrun = 0;
    always @(posedge clk) if (rst_n) begin
        mot_overrun += s_mot;
        if (dut.fire_done) begin
            if (dut.fire_sweep_start) begin T = 0; X = 0; Y = 0; sweeps++; end
            else begin T += W * TF; X += V * TF * $cos(T); Y += V * TF * $sin(T); end
            th = (dut.u_motion.w_az + (dut.u_motion.w_second ? dut.u_motion.az_step / 2 : 0)) / 100.0 * PI / 180.0 - T;
            d = $sqrt(($sin(th) - dut.fire_sin0 / 32768.0) ** 2 + ($cos(th) - dut.fire_cos0 / 32768.0) ** 2); if (d > worst_ang) worst_ang = d;
            d = $abs(dut.shift_x - X / 0.002); if (d > worst_shift) worst_shift = d;
            d = $abs(dut.shift_y - Y / 0.002); if (d > worst_shift) worst_shift = d;
            fired++;
        end
    end

    // ---- host-side PHY model: collect transmitted frames -----------------------------------------
    byte unsigned rxq[$]; int nibs = 0; logic [3:0] lo; bit pre_ok = 1, in_frame = 0;
    int frames = 0, bad_pre = 0, bad_fcs = 0, bad_hdr = 0, bad_pt = 0, pads = 0, pts_out = 0, skipped = 0, flag_bad_frames = 0, last_gap = 100, short_gap = 0;
    real worst_pt = 0;
    // reference motion for the transmitted points, kept apart from the one above
    real rT = 0, rX = 0, rY = 0; int r_prev_az = 0, r_seq = 0, r_sweep = 0; bit r_first = 1;
    byte unsigned by_exp[$][$]; bit by_bad[$]; int by_ok = 0, by_wrong = 0;

    function automatic int u16(input int i); return {rxq[i+1], rxq[i]}; endfunction
    function automatic int s16(input int i); logic signed [15:0] v = {rxq[i+1], rxq[i]}; return v; endfunction
    function automatic logic [31:0] u32(input int i); return {rxq[i+3], rxq[i+2], rxq[i+1], rxq[i]}; endfunction
    function automatic real clampr(input real v); return (v > 32767) ? 32767 : (v < -32767) ? -32767 : v; endfunction

    function automatic void check_point_frame();
        int o, a, step, dd, sum; bit sw, hdr_good; real el, g, ang, ex, ey, ez, e, tol;
        logic [7:0] fl;
        hdr_good = (rxq.size() == 1270);
        if (hdr_good) begin
            for (int i = 0; i < 6; i++) hdr_good &= (rxq[i] == DMAC[8*(5-i) +: 8]) && (rxq[6+i] == SMAC[8*(5-i) +: 8]);
            hdr_good &= ({rxq[12], rxq[13]} == 16'h0800) && (rxq[14] == 8'h45) && ({rxq[16], rxq[17]} == 1252) && (rxq[23] == 8'h11);
            sum = 0; for (int i = 14; i < 34; i += 2) sum += {rxq[i], rxq[i+1]};
            sum = (sum & 16'hFFFF) + (sum >> 16); sum = (sum & 16'hFFFF) + (sum >> 16);
            hdr_good &= (sum == 16'hFFFF);
            hdr_good &= ({rxq[26], rxq[27], rxq[28], rxq[29]} == SIP) && ({rxq[30], rxq[31], rxq[32], rxq[33]} == DIP);
            hdr_good &= ({rxq[34], rxq[35]} == 4000) && ({rxq[36], rxq[37]} == 4001) && ({rxq[38], rxq[39]} == 1232);
            hdr_good &= (u32(42) == 32'h58324350) && (rxq[46] == 3) && (u16(48) == 192) && (u32(50) == r_seq);
            fl = rxq[47]; hdr_good &= (fl[0] == 1) && (fl[1] == 0) && (fl[3] == 0);
            if (fl[2]) flag_bad_frames++;
            hdr_good &= (rxq[58] == 0) && (rxq[59] == 6) && (rxq[60] == 6) && (rxq[61] == 32);
            hdr_good &= (u32(66) == S_TS) && (u32(70) == $shortrealtobits(shortreal'(SX))) && (u32(74) == $shortrealtobits(shortreal'(SY)))
                     && (u32(78) == $shortrealtobits(shortreal'(STH))) && (u32(82) == $shortrealtobits(shortreal'(W))) && (u32(86) == $shortrealtobits(shortreal'(V)));
            if (ref_az.size() > 0) begin
                sw = !r_first && (ref_az[0] < r_prev_az);
                hdr_good &= (u16(56) == ref_az[0]) && (u16(54) == r_sweep + sw);
                if ((r_sweep + sw) > 0) hdr_good &= (u32(62) != 0);
            end
        end
        r_seq++;
        if (!hdr_good) begin bad_hdr++; return; end
        for (int b = 0; b < 6; b++) begin
            o = 90 + 196 * b;
            if (rxq[o+2][1]) begin pads++; continue; end
            if (ref_az.size() == 0) begin bad_pt++; continue; end
            a = ref_az.pop_front(); step = a - r_prev_az; if (step < 0) step += 36000;
            sw = !r_first && (a < r_prev_az);
            if (u16(o) != a || rxq[o+2][0] != sw || rxq[o+3] != 0) bad_hdr++;
            if (sw) r_sweep++;
            for (int p = 0; p < 32; p++) begin
                if (p == 0 || p == 16) begin
                    if (sw && p == 0) begin rT = 0; rX = 0; rY = 0; end
                    else begin rT += W * TF; rX += V * TF * $cos(rT); rY += V * TF * $sin(rT); end
                end
                dd = ref_d.pop_front(); pts_out++;
                if (r_first && p >= 16) begin skipped++; continue; end       // no earlier block to take the azimuth step from
                el = (((p % 16) % 2) ? (p % 16) : (p % 16) - 15) * PI / 180.0;
                ang = (a + ((p >= 16) ? step / 2 : 0)) / 100.0 * PI / 180.0 - rT;
                g = dd * $cos(el);
                if (dd == 0) begin ex = -32768; ey = -32768; ez = -32768; end
                else begin ex = clampr(g * $cos(ang) + rX / 0.002); ey = clampr(-g * $sin(ang) + rY / 0.002); ez = clampr(dd * $sin(el)); end
                tol = 2.5 + g * 4.0e-4;
                e = $abs(s16(o + 4 + 6*p) - ex); if ($abs(s16(o + 6 + 6*p) - ey) > e) e = $abs(s16(o + 6 + 6*p) - ey); if ($abs(s16(o + 8 + 6*p) - ez) > e) e = $abs(s16(o + 8 + 6*p) - ez);
                if (e > tol) begin bad_pt++; if (bad_pt < 6) $display("  point wrong: block az %0d p %0d d %0d got %0d %0d %0d want %.1f %.1f %.1f", a, p, dd, s16(o+4+6*p), s16(o+6+6*p), s16(o+8+6*p), ex, ey, ez); end
                if (dd != 0 && e / (1.0 + g * 1.6e-4) > worst_pt) worst_pt = e / (1.0 + g * 1.6e-4);
            end
            r_prev_az = a; r_first = 0;
        end
    endfunction

    function automatic void check_bypass_frame(input bit fcs_good);
        byte unsigned want[$]; bit wb, same;
        if (by_exp.size() == 0) begin by_wrong++; return; end
        want = by_exp.pop_front(); wb = by_bad.pop_front();
        same = (rxq.size() == want.size() + 4);
        if (same) foreach (want[i]) same &= (want[i] == rxq[i]);
        if (same && (fcs_good == !wb)) by_ok++; else by_wrong++;
    endfunction

    always @(posedge tx_clk) begin
        if (mii_tx_en) begin
            if (!in_frame) begin in_frame = 1; nibs = 0; rxq = {}; pre_ok = 1; if (last_gap < 24) short_gap++; end
            if (nibs < 15) pre_ok &= (mii_txd == 4'h5);
            else if (nibs == 15) pre_ok &= (mii_txd == 4'hD);
            else if (nibs[0] == 0) lo = mii_txd;
            else rxq.push_back({mii_txd, lo});
            nibs++;
        end else begin
            if (in_frame) begin
                bit fcs_good;
                in_frame = 0; last_gap = 0; frames++;
                if (!pre_ok || nibs[0]) bad_pre++;
                fcs_good = (rxq.size() > 4) && (crc32(rxq, rxq.size() - 4) == {rxq[rxq.size()-1], rxq[rxq.size()-2], rxq[rxq.size()-3], rxq[rxq.size()-4]});
                if (bypass) check_bypass_frame(fcs_good);
                else begin if (!fcs_good) bad_fcs++; check_point_frame(); end
            end
            last_gap++;
        end
    end

    task automatic set_state(); @(posedge clk); state_valid <= 1; @(posedge clk); state_valid <= 0; endtask
    task automatic restart(); @(posedge clk); enable = 0; repeat (50) @(posedge clk); enable = 1; gap(60); endtask

    // Packets are spaced like the fastest lidar in the family (VLP-32C in dual-return mode: one
    // packet every 332 us; a VLP-16 sends one every 1327 us), in receive clocks.
    localparam int SPACING = 5780;
    byte unsigned f[$], junk[$];
    initial begin
        state_omega = $shortrealtobits(shortreal'(W)); state_v = $shortrealtobits(shortreal'(V));
        repeat (6) @(posedge clk); rst_n = 1; gap(40);
        set_state();
        // 40 lidar packets with other traffic and one damaged frame mixed in
        for (int k = 0; k < 40; k++) begin
            lidar_packet(f); send(f, 0);
            if (k % 3 != 2) gap(SPACING);                     // spacing varies: every third gap is twice as long
            if (k % 9 == 4) begin junk = {}; for (int i = 0; i < 90; i++) junk.push_back($urandom); junk[12] = 8'h08; junk[13] = 8'h06; send(junk, 0); end
            if (k == 20) begin gap(SPACING); lidar_packet(f); send(f, 1); gap(2 * SPACING); end      // good data, bad checksum: points still come out, flagged
            if (k % 3 == 2) gap(2 * SPACING);
        end
        gap(4000);                                            // let the transmitter finish
        // switch the block off and on again, then one more packet
        restart();
        T = 0; X = 0; Y = 0; rT = 0; rX = 0; rY = 0; r_first = 1; r_prev_az = 0; r_seq = 0; r_sweep = 0;   // the block's totals were cleared
        set_state(); gap(10);
        lidar_packet(f); send(f, 0);
        gap(8000);
        $display("PHY clock half periods: receive %0d ps, transmit %0d ps", rx_half, tx_half);
        $display("lidar packets accepted %0d, other frames ignored %0d, bad-checksum flags %0d, sweeps started %0d", pk, other, crc_bad, sweeps);
        $display("points out of the parser %0d, wrong %0d, expected but missing %0d, receive FIFO overflows %0d", pts, wrong, exp.size(), overflow);
        $display("firings worked out %0d, worst angle error %.1e rad, worst shift error %.2f counts, motion overruns %0d", fired, worst_ang, worst_shift, mot_overrun);
        $display("frames transmitted %0d (status pulses %0d), bad preamble %0d, bad checksum %0d, bad header %0d, short gaps %0d", frames, sent, bad_pre, bad_fcs, bad_hdr, short_gap);
        $display("points transmitted %0d, wrong %0d, not checked %0d, padding blocks %0d, blocks never sent %0d, worst point error %.2f counts", pts_out, bad_pt, skipped, pads, ref_az.size(), worst_pt);
        $display("frames flagged for a bad received checksum %0d, buffer overflows %0d, transmit underruns %0d", flag_bad_frames, buf_ovf, underrun);
        // bypass: three frames straight through
        bypass = 1; restart();
        junk = {}; for (int i = 0; i < 90; i++) junk.push_back($urandom); by_exp.push_back(junk); by_bad.push_back(0); send(junk, 0);
        lidar_packet(f); by_exp.push_back(f); by_bad.push_back(0); send(f, 0);
        junk = {}; for (int i = 0; i < 300; i++) junk.push_back($urandom); by_exp.push_back(junk); by_bad.push_back(1); send(junk, 1);
        gap(400);
        $display("bypass: frames through correctly %0d, wrong %0d, missing %0d, underruns %0d", by_ok, by_wrong, by_exp.size(), underrun);
        if (pk == 43 && other == 6 && crc_bad == 2 && wrong == 0 && overflow == 0 && sweeps == 1
            && fired == 42 * 24 && worst_ang < 3.0e-4 && worst_shift < 1.0 && mot_overrun == 0
            && frames == 84 + 3 && sent == 84 + 3 && bad_pre == 0 && bad_fcs == 0 && bad_hdr == 0 && short_gap == 0
            && pts_out == 42 * 384 && bad_pt == 0 && skipped == 32 && pads == 0 && flag_bad_frames == 1 && buf_ovf == 0 && underrun == 0
            && by_ok == 3 && by_wrong == 0 && by_exp.size() == 0)
             $display("TEST PASSED"); else $display("TEST FAILED");
        $finish;
    end
endmodule
