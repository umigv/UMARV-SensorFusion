`timescale 1ns/1ps
module tb;
    logic clk = 0; always #20 clk = ~clk;          // 25 MHz
    logic rst = 1, dv = 0, er = 0; logic [3:0] rxd = 0;
    logic [7:0] d; logic v, sop, eop, err;
    mii_recv dut (.RX_CLK(clk), .RX_RST(rst), .RXD(rxd), .RX_DV(dv), .RX_ER(er), .rx_data(d), .rx_valid(v), .rx_sop(sop), .rx_eop(eop), .rx_error(err));
    // downstream: straight into the lidar front end (same clock here, since this test has no clock crossing)
    logic pt_valid, acc, ign, rdy; logic [15:0] pd, pa; logic [3:0] pl; logic [7:0] pr;
    lidar_vlp16_rx fe (.clk, .rst, .lidar_port(16'd2368), .drop_zero(1'b0), .s_tdata(d), .s_tvalid(v), .s_tready(rdy), .s_tlast(eop), .pause(1'b0),
                       .pt_valid, .pt_distance(pd), .pt_azimuth(pa), .pt_laser_id(pl), .pt_reflectivity(pr), .pkt_accepted(acc), .pkt_ignored(ign));

    function automatic logic [31:0] crc32(input byte unsigned q[$]);
        logic [31:0] c = '1;
        foreach (q[i]) begin c ^= q[i]; repeat (8) c = c[0] ? (c >> 1) ^ 32'hEDB88320 : (c >> 1); end
        return ~c;
    endfunction

    byte unsigned got[$]; int n_sop, n_eop, n_err, sop_at, fails = 0, pts = 0, accs = 0, min_gap = 999, last_v = -999, cyc = 0;
    always @(posedge clk) begin
        cyc++;
        if (v) begin
            if (sop) begin n_sop++; sop_at = got.size(); end
            got.push_back(d); if (cyc - last_v < min_gap) min_gap = cyc - last_v; last_v = cyc;
            if (eop) begin n_eop++; n_err += err; end
            if (err && !eop) begin fails++; $display("FAIL rx_error without rx_eop"); end
        end else if (sop || eop || err) begin fails++; $display("FAIL flag without rx_valid"); end
        pts += pt_valid; accs += acc;
    end
    task automatic nib(input logic [3:0] x, input bit e = 0); @(negedge clk); dv = 1; rxd = x; er = e; endtask
    task automatic idle(input int n); @(negedge clk); dv = 0; er = 0; rxd = $urandom; repeat (n) @(negedge clk); endtask
    // kind: 0 good, 1 corrupt a byte, 2 drop the last nibble, 3 RX_ER in the middle, 4 no start marker (garbage)
    task automatic frame(input byte unsigned pay[$], input int pre_nibbles, input int kind);
        byte unsigned q[$]; logic [31:0] c;
        q = pay; c = crc32(pay); for (int i = 0; i < 4; i++) q.push_back(c[8*i +: 8]);
        if (kind == 1) q[q.size() / 2] ^= 8'h08;
        if (kind == 4) begin repeat (40) nib($urandom_range(0, 4)); return; end
        repeat (pre_nibbles) nib(4'h5);
        nib(4'hD);
        foreach (q[i]) begin
            nib(q[i][3:0], (kind == 3 && i == 20));
            if (!(kind == 2 && i == q.size() - 1)) nib(q[i][7:4]);
        end
    endtask
    task automatic check(input string name, input byte unsigned pay[$], input int pre, input int kind, input int exp_frames, input int exp_err);
        byte unsigned exp[$]; logic [31:0] c; bit same;
        got = {}; n_sop = 0; n_eop = 0; n_err = 0;
        frame(pay, pre, kind); idle(24);
        exp = pay; c = crc32(pay); for (int i = 0; i < 4; i++) exp.push_back(c[8*i +: 8]);
        if (kind == 1) exp[exp.size() / 2] ^= 8'h08;
        if (kind == 2) void'(exp.pop_back());
        same = (exp_frames == 0) ? (got.size() == 0) : (got == exp);
        if (!same || n_sop != exp_frames || n_eop != exp_frames || n_err != exp_err || (exp_frames && sop_at != 0)) begin
            fails++; $display("FAIL %s: %0d bytes out (expected %0d), sop %0d eop %0d err %0d", name, got.size(), exp.size(), n_sop, n_eop, n_err);
        end else $display("ok   %-46s %4d bytes, sop %0d, eop %0d, error %0d", name, got.size(), n_sop, n_eop, n_err);
    endtask
    byte unsigned p[$], check9[$] = '{8'h31, 8'h32, 8'h33, 8'h34, 8'h35, 8'h36, 8'h37, 8'h38, 8'h39};
    function automatic void fill(input int n); p = {}; for (int i = 0; i < n; i++) p.push_back($urandom); endfunction
    initial begin
        if (crc32(check9) !== 32'hCBF43926) begin fails++; $display("FAIL testbench CRC"); end
        repeat (4) @(negedge clk); rst = 0; idle(5);
        fill(60);   check("minimum-size frame", p, 15, 0, 1, 0);
        fill(1500); check("maximum-size frame", p, 15, 0, 1, 0);
        fill(200);  check("short preamble (3 nibbles)", p, 3, 0, 1, 0);
        fill(200);  check("start marker with no preamble", p, 0, 0, 1, 0);
        fill(200);  check("one corrupted byte", p, 15, 1, 1, 1);
        fill(200);  check("ends half-way through a byte", p, 15, 2, 1, 1);
        fill(200);  check("RX_ER during the frame", p, 15, 3, 1, 1);
        fill(10);   check("garbage with no start marker", p, 15, 4, 0, 0);
        fill(300);  check("good frame right after the garbage", p, 15, 0, 1, 0);
        // back-to-back frames with the minimum gap (12 byte times)
        got = {}; n_sop = 0; n_eop = 0; n_err = 0;
        for (int k = 0; k < 5; k++) begin fill(64 + 100 * k); frame(p, 15, 0); idle(23); end
        if (n_sop != 5 || n_eop != 5 || n_err != 0) begin fails++; $display("FAIL back-to-back: sop %0d eop %0d err %0d", n_sop, n_eop, n_err); end
        else $display("ok   %-46s %4d bytes, sop %0d, eop %0d, error %0d", "five frames with the minimum gap", got.size(), n_sop, n_eop, n_err);
        // RX_DV stuck high: cut at the length limit and flagged
        got = {}; n_sop = 0; n_eop = 0; n_err = 0;
        repeat (15) nib(4'h5); nib(4'hD); repeat (5000) nib($urandom); idle(24);
        if (got.size() != 1536 || n_eop != 1 || n_err != 1) begin fails++; $display("FAIL stuck RX_DV: %0d bytes eop %0d err %0d", got.size(), n_eop, n_err); end
        else $display("ok   %-46s %4d bytes, sop %0d, eop %0d, error %0d", "RX_DV stuck high for 2,500 bytes", got.size(), n_sop, n_eop, n_err);
        fill(100);  check("good frame after the stuck one", p, 15, 0, 1, 0);
        // a real-size VLP-16 packet through to the lidar front end
        p = {}; for (int i = 0; i < 42 + 1206; i++) p.push_back($urandom);
        p[12] = 8'h08; p[13] = 8'h00; p[14] = 8'h45; p[20] = 8'h40; p[21] = 8'h00; p[23] = 8'h11; p[36] = 8'h09; p[37] = 8'h40; p[38] = 8'h04; p[39] = 8'hBE;
        for (int b = 0; b < 12; b++) begin p[42 + 100*b] = 8'hFF; p[43 + 100*b] = 8'hEE; end
        pts = 0; accs = 0;
        check("VLP-16 packet (1,248 bytes + checksum)", p, 15, 0, 1, 0);
        if (pts != 384 || accs != 1) begin fails++; $display("FAIL lidar front end: %0d points, %0d packets", pts, accs); end
        else $display("ok   lidar front end fed from this block: %0d points, %0d packet accepted", pts, accs);
        $display("closest two output bytes: %0d clocks apart", min_gap);
        $display("%s", fails == 0 ? "ALL CHECKS PASSED" : "CHECKS FAILED");
        $finish;
    end
endmodule
