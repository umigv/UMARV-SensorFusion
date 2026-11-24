// Test of the host SPI port and register file. A model of the microcontroller writes and reads
// registers at several SCK rates and the results are compared with what was written and with
// the status inputs.
module tb_spi_regs;
    logic clk = 0; always #10 clk = ~clk;                    // 50 MHz
    logic rst_n = 0, sck = 0, cs_n = 1, mosi = 0, miso;
    logic [6:0] stat = 0; logic [31:0] beat_ts = 32'hA1B2C3D4, yaw = 32'h40490FDB, rate = 32'hBF000000;
    logic [63:0] lat = 64'h4045_2A3D_70A3_D70A, lon = 64'hC054_F0A3_D70A_3D71; logic signed [27:0] dw = -28'sd1234; logic vn_v = 1, dw_v = 0;
    logic ts_en, li_en, byp, s4, cr, cs, chk; logic [1:0] model; logic [15:0] per, dl, lport, sport, dport; logic signed [15:0] yawo;
    logic [47:0] smac, dmac; logic [31:0] sip, dip, h_ts, h_x, h_y, h_th, h_om, h_v; logic h_valid;
    spi_regs dut (.clk, .rst_n, .spi_sck(sck), .spi_cs_n(cs_n), .spi_mosi(mosi), .spi_miso(miso),
        .ts_enable(ts_en), .lidar_enable(li_en), .lidar_bypass(byp), .scale_4mm(s4), .corr_rot_en(cr), .corr_shift_en(cs), .dw_check_turns(chk),
        .lidar_model(model), .period_us(per), .deadline_us(dl), .lidar_port(lport), .yaw_offset(yawo), .src_mac(smac), .dst_mac(dmac),
        .src_ip(sip), .dst_ip(dip), .src_port(sport), .dst_port(dport), .host_state_valid(h_valid), .host_state_ts(h_ts), .host_state_x(h_x),
        .host_state_y(h_y), .host_state_theta(h_th), .host_state_omega(h_om), .host_state_v(h_v), .stat_pulse(stat), .beat_ts,
        .vn_valid(vn_v), .dw_valid(dw_v), .vn_yaw_f32(yaw), .vn_rate_f32(rate), .vn_lat_f64(lat), .vn_lon_f64(lon), .dw_delta(dw));

    int half = 100, errors = 0, checks = 0, strobes = 0;     // half an SCK period in ns: 100 = 5 MHz
    always @(posedge clk) strobes += h_valid;
    task automatic xfer(input byte unsigned tx, output byte unsigned rx);
        for (int i = 7; i >= 0; i--) begin mosi = tx[i]; #(half); sck = 1; rx[i] = miso; #(half); sck = 0; end
    endtask
    task automatic wr(input int a, input byte unsigned d[$]);
        byte unsigned x; cs_n = 0; #(half); xfer(8'h80 | a[6:0], x); foreach (d[i]) xfer(d[i], x); #(half); cs_n = 1; #(3 * half);
    endtask
    task automatic rd(input int a, input int n, output byte unsigned d[$]);
        byte unsigned x; d = {}; cs_n = 0; #(half); xfer(a[6:0], x); repeat (n) begin xfer($urandom, x); d.push_back(x); end #(half); cs_n = 1; #(3 * half);
    endtask
    task automatic expect_rd(input int a, input byte unsigned want[$], input string what);
        byte unsigned got[$]; rd(a, want.size(), got); checks++;
        if (got != want) begin errors++; $display("WRONG %s at %02h: got %p want %p", what, a, got, want); end
    endtask
    task automatic check(input bit ok, input string what); checks++; if (!ok) begin errors++; $display("WRONG %s", what); end endtask

    byte unsigned q[$], img[$];
    initial begin
        repeat (5) @(posedge clk); rst_n = 1; #200;
        // reset values, on the outputs and read back
        check(ts_en && li_en && !byp && !s4 && !cr && !cs && chk && model == 0, "control reset");
        check(per == 2500 && dl == 2000 && lport == 2368 && yawo == 0 && sport == 2370 && dport == 2370, "number resets");
        check(smac == 48'h020000000001 && dmac == '1 && sip == {8'd192, 8'd168, 8'd1, 8'd50} && dip == '1, "address resets");
        expect_rd(0, '{8'hA7, 8'h01, 8'h43, 8'h00, 8'hC4, 8'h09, 8'hD0, 8'h07}, "ID and first registers");
        // at three SCK rates: write every writable register in one transfer, read it all back
        for (int r = 0; r < 3; r++) begin
            half = (r == 0) ? 100 : (r == 1) ? 137 : 1000;       // 5 MHz, 3.6 MHz, 0.5 MHz
            img = {}; for (int i = 2; i < 60; i++) img.push_back($urandom);
            wr(2, img); expect_rd(2, img, "burst write and read back");
            check({byp, li_en, ts_en} == img[0][2:0] && model == img[1][1:0] && per == {img[3], img[2]} && dl == {img[5], img[4]}, "settings follow registers");
            check(smac == {img[10], img[11], img[12], img[13], img[14], img[15]} && dip == {img[26], img[27], img[28], img[29]} && dport == {img[33], img[32]}, "addresses follow registers");
            check(h_ts == {img[37], img[36], img[35], img[34]} && h_v == {img[57], img[56], img[55], img[54]}, "host state follows registers");
            // single-register writes, and the read-only registers refuse writes
            q = '{8'h5A}; wr(8'h0A, q); expect_rd(8'h0A, q, "single write"); img[8] = 8'h5A;
            q = '{8'h00, 8'h00}; wr(0, q); expect_rd(0, '{8'hA7, 8'h01}, "ID is read only");
            expect_rd(2, img, "neighbours untouched");
        end
        // the state strobe, the counters and their clear
        q = '{8'h01}; wr(8'h3C, q); #100; check(strobes == 1, "one state strobe");
        for (int i = 0; i < 7; i++) repeat (i + 1) begin @(posedge clk); stat[i] <= 1; @(posedge clk); stat[i] <= 0; end
        repeat (300) begin @(posedge clk); stat[0] <= 1; end @(posedge clk); stat <= 0; #100;
        expect_rd(8'h40, '{8'hFF, 8'd2, 8'd3, 8'd4, 8'd5, 8'd6, 8'd7}, "counters, first one stopped at 255");
        q = '{8'h00}; wr(8'h3D, q); expect_rd(8'h40, '{8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0, 8'd0}, "counters cleared");
        check(strobes == 1, "clearing counters is not a state strobe");
        // measurements
        expect_rd(8'h48, '{8'hD4, 8'hC3, 8'hB2, 8'hA1, 8'h01}, "beat time and valid bits");
        expect_rd(8'h50, '{8'hDB, 8'h0F, 8'h49, 8'h40, 8'h00, 8'h00, 8'h00, 8'hBF}, "yaw and rate");
        expect_rd(8'h58, '{8'h0A, 8'hD7, 8'hA3, 8'h70, 8'h3D, 8'h2A, 8'h45, 8'h40, 8'h71, 8'h3D, 8'h0A, 8'hD7, 8'hA3, 8'hF0, 8'h54, 8'hC0}, "latitude and longitude");
        expect_rd(8'h68, '{8'h2E, 8'hFB, 8'hFF, 8'hFF}, "encoder travel, sign extended");
        // a transfer cut off part-way through a byte writes nothing and the next transfer is clean
        cs_n = 0; #(half); mosi = 1; repeat (11) begin #(half); sck = 1; #(half); sck = 0; end cs_n = 1; #(3 * half);
        expect_rd(2, img, "after a cut-off transfer");
        $display("%0d checks, %0d wrong", checks, errors);
        if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED");
        $finish;
    end
endmodule
