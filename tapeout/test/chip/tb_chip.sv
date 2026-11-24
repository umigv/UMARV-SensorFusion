// Test of the chip top level (tt_um_umigv_sensorfusion) through its pins only: the host SPI port,
// the two MII ports and the sensor pins. No sensors are attached (the VN-300 line idles high and
// the encoder answers zeros). Checks that the register file really controls the timestamper and
// the lidar block, and that their status really comes back through it.
module tb_chip;
    logic clk = 0; always #10000 clk = ~clk;                 // 50 MHz
    logic rx_clk = 0; initial begin #3777; forever #20003 rx_clk = ~rx_clk; end
    logic tx_clk = 0; initial begin #9123; forever #19996 tx_clk = ~tx_clk; end
    logic rst_n = 0, sck = 0, cs_n = 1, mosi = 0, rx_dv = 0; logic [3:0] rxd = 0;
    wire [7:0] uo, uio_out, uio_oe;
    tt_um_umigv_sensorfusion dut (.ui_in({rx_clk, tx_clk, rx_dv, mosi, cs_n, sck, 1'b0, 1'b1}), .uo_out(uo),
        .uio_in({4'h0, rxd}), .uio_out(uio_out), .uio_oe(uio_oe), .ena(1'b1), .clk, .rst_n);
    wire vn_sync = uo[0], miso = uo[5], tx_en = uo[7]; wire [3:0] txd = uio_out[7:4];

    int errors = 0, checks = 0;
    task automatic check(input bit ok, input string what); checks++; if (!ok) begin errors++; $display("WRONG %s", what); end endtask

    // ---- host SPI master, 2.5 MHz ---------------------------------------------------------------
    localparam int H = 200000;
    task automatic xfer(input byte unsigned tx, output byte unsigned rx);
        for (int i = 7; i >= 0; i--) begin mosi = tx[i]; #(H); sck = 1; rx[i] = miso; #(H); sck = 0; end
    endtask
    task automatic wr(input int a, input byte unsigned d[$]);
        byte unsigned x; cs_n = 0; #(H); xfer(8'h80 | a[6:0], x); foreach (d[i]) xfer(d[i], x); #(H); cs_n = 1; #(3 * H);
    endtask
    task automatic rd(input int a, input int n, output byte unsigned d[$]);
        byte unsigned x; d = {}; cs_n = 0; #(H); xfer(a[6:0], x); repeat (n) begin xfer(8'h00, x); d.push_back(x); end #(H); cs_n = 1; #(3 * H);
    endtask
    task automatic expect_rd(input int a, input byte unsigned want[$], input string what);
        byte unsigned got[$]; rd(a, want.size(), got); checks++;
        if (got != want) begin errors++; $display("WRONG %s at %02h: got %p want %p", what, a, got, want); end
    endtask

    // ---- lidar-side PHY: send frames -------------------------------------------------------------
    function automatic logic [31:0] crc32(input byte unsigned q[$], input int n);
        logic [31:0] c = '1;
        for (int i = 0; i < n; i++) begin c ^= q[i]; repeat (8) c = c[0] ? (c >> 1) ^ 32'hEDB88320 : (c >> 1); end
        return ~c;
    endfunction
    task automatic nib(input logic [3:0] x); @(negedge rx_clk); rx_dv = 1; rxd = x; endtask
    task automatic gap(input int n); @(negedge rx_clk); rx_dv = 0; rxd = 0; repeat (n) @(negedge rx_clk); endtask
    task automatic send(input byte unsigned pay[$]);
        byte unsigned q[$]; logic [31:0] c;
        q = pay; c = crc32(pay, pay.size()); for (int i = 0; i < 4; i++) q.push_back(c[8*i +: 8]);
        repeat (15) nib(4'h5); nib(4'hD);
        foreach (q[i]) begin nib(q[i][3:0]); nib(q[i][7:4]); end
        gap(24);
    endtask
    int az = 1000;
    function automatic void lidar_packet(output byte unsigned f[$]);
        int a; f = {};
        for (int i = 0; i < 42 + 1206; i++) f.push_back($urandom);
        f[12] = 8'h08; f[13] = 8'h00; f[14] = 8'h45; f[20] = 8'h40; f[21] = 8'h00; f[23] = 8'h11; f[36] = 8'h09; f[37] = 8'h40; f[38] = 8'h04; f[39] = 8'hBE;
        for (int b = 0; b < 12; b++) begin
            a = az % 36000; az += 40;
            f[42 + 100*b] = 8'hFF; f[43 + 100*b] = 8'hEE; f[44 + 100*b] = a[7:0]; f[45 + 100*b] = a[15:8];
        end
    endfunction

    // ---- host-side PHY: collect frames -----------------------------------------------------------
    byte unsigned rxq[$], last[$]; int nibs = 0, frames = 0, bad_fcs = 0; logic [3:0] lo; bit in_frame = 0;
    always @(posedge tx_clk) begin
        if (tx_en) begin
            if (!in_frame) begin in_frame = 1; nibs = 0; rxq = {}; end
            if (nibs >= 16) begin if (nibs[0] == 0) lo = txd; else rxq.push_back({txd, lo}); end
            nibs++;
        end else if (in_frame) begin
            in_frame = 0; frames++; last = rxq;
            if (rxq.size() < 5 || crc32(rxq, rxq.size() - 4) != {rxq[rxq.size()-1], rxq[rxq.size()-2], rxq[rxq.size()-3], rxq[rxq.size()-4]}) bad_fcs++;
        end
    end
    function automatic logic [31:0] u32(input int i); return {last[i+3], last[i+2], last[i+1], last[i]}; endfunction
    function automatic bit ip_sum_ok(); int s = 0; for (int i = 14; i < 34; i += 2) s += {last[i], last[i+1]}; s = (s & 16'hFFFF) + (s >> 16); s = (s & 16'hFFFF) + (s >> 16); return s == 16'hFFFF; endfunction

    // ---- beat spacing, from the sync pin ---------------------------------------------------------
    int cyc = 0, last_sync = 0, spacing = 0, beats = 0; logic sync_d = 0;
    always @(posedge clk) begin cyc++; sync_d <= vn_sync; if (vn_sync && !sync_d) begin spacing = cyc - last_sync; last_sync = cyc; beats++; end end

    byte unsigned f[$], q[$], junk[$]; int n;
    initial begin
        repeat (6) @(posedge clk); rst_n = 1; gap(40);
        check(uio_oe == 8'hF0, "pin directions");
        expect_rd(0, '{8'hA7, 8'h01, 8'h43}, "ID, version, control at reset");

        // 1. With no host settings at all: lidar packets go through with the reset addresses.
        repeat (2) begin lidar_packet(f); send(f); gap(6000); end
        check(frames == 4 && bad_fcs == 0, "four frames with reset settings");
        check(last.size() == 1270 && last[0] == 8'hFF && last[5] == 8'hFF && {last[6], last[7], last[8], last[9], last[10], last[11]} == 48'h020000000001, "reset MAC addresses");
        check({last[26], last[27], last[28], last[29]} == {8'd192, 8'd168, 8'd1, 8'd50} && {last[36], last[37]} == 2370 && ip_sum_ok(), "reset IP and port");
        check(last[47][0] == 0, "no state yet: flag clear");

        // 2. Settings written over SPI reach the lidar block.
        q = '{8'h02, 8'h11, 8'h22, 8'h33, 8'h44, 8'h55,  8'hAA, 8'hBB, 8'hCC, 8'hDD, 8'hEE, 8'h0F,  8'd10, 8'd0, 8'd0, 8'd7,  8'd10, 8'd0, 8'd0, 8'd9,  8'h34, 8'h12,  8'h78, 8'h56};
        wr(8'h0C, q);
        q = '{8'h78, 8'h56, 8'h34, 8'h12,  8'h00, 8'h00, 8'hC0, 8'h3F,  8'h00, 8'h00, 8'h10, 8'hC0,  8'h00, 8'h00, 8'h00, 8'h3F,  8'h00, 8'h00, 8'h00, 8'h00,  8'h00, 8'h00, 8'h00, 8'h00};
        wr(8'h24, q);                                         // time 12345678, x 1.5, y -2.25, theta 0.5, omega 0, v 0
        q = '{8'h73}; wr(8'h02, q);                           // both corrections on
        q = '{8'h01}; wr(8'h3C, q);                           // hand the state over
        repeat (2) begin lidar_packet(f); send(f); gap(6000); end
        check(frames == 8 && bad_fcs == 0, "four more frames");
        check({last[0], last[1], last[2], last[3], last[4], last[5]} == 48'hAABBCCDDEE0F && {last[6], last[7], last[8], last[9], last[10], last[11]} == 48'h021122334455, "MAC addresses from registers");
        check({last[26], last[27], last[28], last[29]} == {8'd10, 8'd0, 8'd0, 8'd7} && {last[30], last[31], last[32], last[33]} == {8'd10, 8'd0, 8'd0, 8'd9} && ip_sum_ok(), "IP addresses from registers");
        check({last[34], last[35]} == 16'h1234 && {last[36], last[37]} == 16'h5678, "UDP ports from registers");
        check(last[47][0] == 1 && u32(66) == 32'h12345678 && u32(70) == 32'h3FC00000 && u32(74) == 32'hC0100000 && u32(78) == 32'h3F000000, "host state in the packet header");

        // 3. Status comes back: 4 lidar packets in, 8 frames out, nothing else.
        expect_rd(8'h40, '{8'd4, 8'd0, 8'd0, 8'd8, 8'd0, 8'd0, 8'd0}, "event counters");
        q = '{8'h00}; wr(8'h3D, q); expect_rd(8'h40, '{8'd0, 8'd0, 8'd0, 8'd0}, "counters cleared");

        // 4. Timestamper: default beat is 2,500 us; then change it over SPI.
        wait (beats >= 3); check(spacing == 125000, "default beat spacing 2,500 us");
        q = '{8'hE8, 8'h03, 8'h20, 8'h03}; wr(8'h04, q);      // period 1,000 us, deadline 800 us
        n = beats; wait (beats >= n + 3); check(spacing == 50000, "beat spacing 1,000 us after the write");
        repeat (45000) @(posedge clk);                        // past the deadline of this beat
        rd(8'h48, 5, q);
        check({q[3], q[2], q[1], q[0]} == dut.beat_ts && dut.beat_ts != 0, "beat time reads back");
        check(q[4] == 8'h00, "no sensors attached: both valid bits clear");
        q = '{8'h72}; wr(8'h02, q); n = beats; repeat (120000) @(posedge clk); check(beats == n, "timestamper off: no beats");
        q = '{8'h73}; wr(8'h02, q);

        // 5. Bypass and lidar off, both from the control register.
        q = '{8'h77}; wr(8'h02, q); gap(100);
        junk = {}; for (int i = 0; i < 120; i++) junk.push_back($urandom); n = frames; send(junk); gap(400);
        check(frames == n + 1 && bad_fcs == 0 && last.size() == 124, "bypass: one frame out");
        begin bit same = 1; foreach (junk[i]) same &= (junk[i] == last[i]); check(same, "bypass: frame unchanged"); end
        q = '{8'h71}; wr(8'h02, q); gap(100);
        n = frames; lidar_packet(f); send(f); gap(8000); check(frames == n, "lidar off: nothing transmitted");

        $display("%0d checks, %0d wrong; %0d frames transmitted, %0d beats", checks, errors, frames, beats);
        if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED");
        $finish;
    end
endmodule
