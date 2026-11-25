// Whole-chip test with the real Kalman filter (ukf_top.sv) in its socket, driven through the pins only.
//
//   VN-300 model  -> serial pin  \
//   encoder model <-> SPI pins    > timestamper -> filter -> lidar block -> MII transmit
//   lidar frames  -> MII receive /
//   host SPI: settings in, measurements and counters out
//
// The wheel turns 450 counts a beat (3.45 m/s with a 50 mm radius) and the VN-300 reports a turn
// rate of 1 rad/s clockwise. Every beat's measurements and the state the filter then produced are
// written to sys_log.txt; check_sys.py runs the same measurements through the exact model of the
// filter and compares bit for bit. Checked here:
//   - the measurements the host reads over SPI are the ones the sensor models sent
//   - the state in every transmitted lidar frame is one the filter produced, value for value
//   - the lidar block's motion correction uses that state (its shift grows at the robot's speed)
//   - switching the state source back to the host registers takes effect
module tb_system_ukf;
    logic clk = 0; always #10000 clk = ~clk;                 // 50 MHz
    logic rx_clk = 0; initial begin #3777; forever #20003 rx_clk = ~rx_clk; end
    logic tx_clk = 0; initial begin #9123; forever #19996 tx_clk = ~tx_clk; end
    logic rst_n = 0, sck = 0, cs_n = 1, mosi = 0, rx_dv = 0, vn_rx = 1, enc_miso = 0; logic [3:0] rxd = 0;
    wire [7:0] uo, uio_out, uio_oe;
    tt_um_umigv_sensorfusion dut (.ui_in({rx_clk, tx_clk, rx_dv, mosi, cs_n, sck, enc_miso, vn_rx}), .uo_out(uo),
        .uio_in({4'h0, rxd}), .uio_out(uio_out), .uio_oe(uio_oe), .ena(1'b1), .clk, .rst_n);
    wire vn_sync = uo[0], enc_cs_n = uo[1], enc_sck = uo[2], enc_mosi = uo[3], ready = uo[4], miso = uo[5], tx_en = uo[7];
    wire [3:0] txd = uio_out[7:4];

    int errors = 0, checks = 0;
    logic [31:0] kept;
    task automatic check(input bit ok, input string what); checks++; if (!ok) begin errors++; $display("WRONG %s", what); end endtask
    localparam real PI = 3.14159265358979, M_PER_COUNT = 0.2 / 16384.0, TF = 55.296e-6;
    localparam int  WHEEL_STEP = 450;                        // counts per beat: 2.197 m/s
    localparam real RATE = 1.0;                              // rad/s

    // ---- VN-300 model: answers each sync pulse with a 56-byte message ---------------------------
    localparam int BIT_PS = 1085069;                         // 921,600 baud
    function automatic logic [15:0] crcf(input logic [15:0] c, input logic [7:0] d);
        int unsigned x; x = c;
        x = ((x >> 8) & 16'hFF) | ((x << 8) & 16'hFFFF); x ^= d; x ^= (x & 16'hFF) >> 4;
        x = (x ^ (x << 12)) & 16'hFFFF; x = (x ^ ((x & 16'hFF) << 5)) & 16'hFFFF; return x[15:0];
    endfunction
    task automatic uart_send(input byte unsigned d);
        vn_rx = 0; #(BIT_PS); for (int i = 0; i < 8; i++) begin vn_rx = d[i]; #(BIT_PS); end vn_rx = 1; #(BIT_PS);
    endtask
    int vn_beat = 0; shortreal sent_yaw, sent_rate; real sent_lat, sent_lon;
    always @(posedge vn_sync) begin
        automatic byte unsigned msg[56]; automatic logic [31:0] f; automatic logic [63:0] d; automatic logic [15:0] c;
        sent_yaw = 10.0 + RATE * 0.0025 * 180.0 / PI * vn_beat; sent_rate = RATE;
        sent_lat = 42.2936411 + 1.0e-7 * vn_beat; sent_lon = -83.7119955; vn_beat++;
        for (int i = 0; i < 56; i++) msg[i] = $urandom;
        msg[0] = 8'hFA; msg[1] = 8'h01; msg[2] = 8'h68; msg[3] = 8'h10;
        f = $shortrealtobits(sent_yaw);  for (int i = 0; i < 4; i++) msg[4 + i]  = f[8*i +: 8];
        f = $shortrealtobits(sent_rate); for (int i = 0; i < 4; i++) msg[24 + i] = f[8*i +: 8];
        d = $realtobits(sent_lat);       for (int i = 0; i < 8; i++) msg[28 + i] = d[8*i +: 8];
        d = $realtobits(sent_lon);       for (int i = 0; i < 8; i++) msg[36 + i] = d[8*i +: 8];
        msg[52] = 8'h06; msg[53] = 8'h03;
        c = 0; for (int i = 1; i <= 53; i++) c = crcf(c, msg[i]);
        msg[54] = c[15:8]; msg[55] = c[7:0];
        #50_000_000;
        for (int i = 0; i < 56; i++) uart_send(msg[i]);
    end

    // ---- AMT22 encoder model (SPI mode 0) ---------------------------------------------------------
    function automatic logic [15:0] with_check(input logic [13:0] d);
        return {!(d[13]^d[11]^d[9]^d[7]^d[5]^d[3]^d[1]), !(d[12]^d[10]^d[8]^d[6]^d[4]^d[2]^d[0]), d};
    endfunction
    logic signed [27:0] wheel = 28'sh0003F00; logic [15:0] w_pos, w_turns, out_word; logic [31:0] got; int nbits;
    always @(negedge enc_cs_n) begin
        wheel = wheel + WHEEL_STEP; w_pos = with_check(wheel[13:0]); w_turns = with_check(wheel[27:14]);
        out_word = w_pos; nbits = 0; got = '0; enc_miso <= #400000 out_word[15];
    end
    always @(posedge enc_sck) if (!enc_cs_n) begin got = {got[30:0], enc_mosi}; nbits++; end
    always @(negedge enc_sck) if (!enc_cs_n) begin
        if (nbits == 16) out_word = (got[7:0] == 8'hA0) ? w_turns : 16'hDEAD; else out_word = {out_word[14:0], 1'b0};
        enc_miso <= #60000 out_word[15];
    end

    // ---- host SPI master, 2.5 MHz -----------------------------------------------------------------
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

    // ---- lidar-side PHY -----------------------------------------------------------------------------
    function automatic logic [31:0] crc32(input byte unsigned q[$], input int n);
        logic [31:0] c = '1;
        for (int i = 0; i < n; i++) begin c ^= q[i]; repeat (8) c = c[0] ? (c >> 1) ^ 32'hEDB88320 : (c >> 1); end
        return ~c;
    endfunction
    task automatic nib(input logic [3:0] x); @(negedge rx_clk); rx_dv = 1; rxd = x; endtask
    task automatic gap(input int n); @(negedge rx_clk); rx_dv = 0; rxd = 0; repeat (n) @(negedge rx_clk); endtask
    int az = 35000;
    task automatic send_lidar_packet();
        byte unsigned f[$], q[$]; logic [31:0] c; int a;
        for (int i = 0; i < 42 + 1206; i++) f.push_back($urandom);
        f[12] = 8'h08; f[13] = 8'h00; f[14] = 8'h45; f[20] = 8'h40; f[21] = 8'h00; f[23] = 8'h11; f[36] = 8'h09; f[37] = 8'h40; f[38] = 8'h04; f[39] = 8'hBE;
        for (int b = 0; b < 12; b++) begin
            a = az % 36000; az += 40;
            f[42 + 100*b] = 8'hFF; f[43 + 100*b] = 8'hEE; f[44 + 100*b] = a[7:0]; f[45 + 100*b] = a[15:8];
        end
        q = f; c = crc32(f, f.size()); for (int i = 0; i < 4; i++) q.push_back(c[8*i +: 8]);
        repeat (15) nib(4'h5); nib(4'hD);
        foreach (q[i]) begin nib(q[i][3:0]); nib(q[i][7:4]); end
        gap(24);
    endtask

    // ---- what the filter produced, by beat time, as it left the filter ------------------------------
    typedef struct { logic [31:0] x, y, theta, omega, v; } state_t;
    state_t produced[logic [31:0]];
    real mv = 0, mom = 0; int beats = 0, states = 0, fd;
    logic [31:0] last_ts; shortreal yaw_at_pulse;
    initial fd = $fopen("sys_log.txt", "w");
    always @(posedge clk) if (rst_n && ready) begin
        #1; beats++; last_ts = dut.beat_ts; yaw_at_pulse = sent_yaw;
        $fdisplay(fd, "M %0d %0d %h %h %h %h %h %h", dut.vn_valid, dut.dw_valid, dut.vn_yaw_f32, dut.vn_rate_f32, dut.vn_lat_f64, dut.vn_lon_f64,
                  {4'd0, dut.dw_delta}, dut.beat_ts);
    end
    always @(posedge clk) if (rst_n && dut.u_ukf.state_valid) begin
        #1; states++;
        produced[dut.u_ukf.state_ts] = '{dut.u_ukf.state_x, dut.u_ukf.state_y, dut.u_ukf.state_theta, dut.u_ukf.state_omega, dut.u_ukf.state_v};
        mv = $bitstoshortreal(dut.u_ukf.state_v); mom = $bitstoshortreal(dut.u_ukf.state_omega);
        $fdisplay(fd, "S %h %h %h %h %h %h", dut.u_ukf.state_ts, dut.u_ukf.state_x, dut.u_ukf.state_y, dut.u_ukf.state_theta, dut.u_ukf.state_omega, dut.u_ukf.state_v);
    end

    // ---- host-side PHY: collect frames and check the state in each header --------------------------
    byte unsigned rxq[$]; int nibs = 0, frames = 0, bad_fcs = 0, state_frames = 0, bad_state = 0, host_frames = 0; logic [3:0] lo; bit in_frame = 0;
    logic [31:0] seen_ts[$];
    function automatic logic [31:0] u32(input int i); return {rxq[i+3], rxq[i+2], rxq[i+1], rxq[i]}; endfunction
    function automatic bit close(input logic [31:0] bits, input real want); return $abs($bitstoshortreal(bits) - want) <= 1.0e-5 + 1.0e-5 * $abs(want); endfunction
    always @(posedge tx_clk) begin
        if (tx_en) begin
            if (!in_frame) begin in_frame = 1; nibs = 0; rxq = {}; end
            if (nibs >= 16) begin if (nibs[0] == 0) lo = txd; else rxq.push_back({txd, lo}); end
            nibs++;
        end else if (in_frame) begin
            in_frame = 0; frames++;
            if (rxq.size() != 1270 || crc32(rxq, 1266) != u32(1266)) bad_fcs++;
            else if (rxq[47][0]) begin
                automatic logic [31:0] ts = u32(66);
                if (ts == 32'h12345678) host_frames++;
                else begin
                    state_frames++; seen_ts.push_back(ts);
                    if (!produced.exists(ts)) begin bad_state++; $display("  frame carries a state time %0d that no beat had", ts); end
                    else if (!(u32(70) == produced[ts].x && u32(74) == produced[ts].y && u32(78) == produced[ts].theta
                               && u32(82) == produced[ts].omega && u32(86) == produced[ts].v)) begin
                        bad_state++;
                        $display("  state wrong at %0d: got %h %h %h %h %h want %h %h %h %h %h", ts, u32(70), u32(74), u32(78), u32(82), u32(86),
                                 produced[ts].x, produced[ts].y, produced[ts].theta, produced[ts].omega, produced[ts].v);
                    end
                end
            end
        end
    end

    // ---- the motion stage's shift, against the speed the filter reported ---------------------------
    int since_sweep = -1; real worst_shift = 0, e;
    always @(posedge clk) if (rst_n && dut.u_lidar.fire_done) begin
        if (dut.u_lidar.fire_sweep_start) since_sweep = 0; else if (since_sweep >= 0) since_sweep++;
        if (since_sweep > 0) begin
            // distance along the path so far, in 2 mm counts; the path bends by under 0.02 rad here
            e = $abs($sqrt(1.0 * dut.u_lidar.shift_x ** 2 + 1.0 * dut.u_lidar.shift_y ** 2) - mv * since_sweep * TF / 0.002);
            if (e > worst_shift) worst_shift = e;
        end
    end

    byte unsigned q[$]; int n, distinct = 0, shift_n = 0; real shift_err = 0;
    initial begin
        repeat (6) @(posedge clk); rst_n = 1; gap(40);
        q = '{8'hF3}; wr(8'h02, q);                           // everything on, corrections on, state from the filter
        wait (beats >= 4);
        // lidar packets across several beats
        repeat (14) begin send_lidar_packet(); gap(13000); end
        gap(4000);

        // measurements over SPI, read just after a beat's ready pulse
        @(posedge clk iff ready); #100_000;
        rd(8'h48, 5, q);  check({q[3], q[2], q[1], q[0]} == last_ts && q[4] == 8'h03, "beat time and both valid bits over SPI");
        rd(8'h50, 8, q);  check({q[3], q[2], q[1], q[0]} == $shortrealtobits(yaw_at_pulse) && {q[7], q[6], q[5], q[4]} == $shortrealtobits(shortreal'(RATE)), "yaw and turn rate over SPI");
        rd(8'h58, 8, q);  check({q[7], q[6], q[5], q[4], q[3], q[2], q[1], q[0]} == dut.vn_lat_f64 && $bitstoreal(dut.vn_lat_f64) > 42.29 && $bitstoreal(dut.vn_lat_f64) < 42.30, "latitude over SPI");
        rd(8'h68, 4, q);  check({q[3], q[2], q[1], q[0]} == WHEEL_STEP, "wheel travel over SPI");
        rd(8'h40, 7, q);  check(q[0] == 14 && q[3] == 28 && q[1] == 0 && q[2] == 0 && q[4] == 0 && q[5] == 0 && q[6] == 0, "counters: 14 packets in, 28 frames out, no errors");

        check(frames == 28 && bad_fcs == 0, "28 good frames");
        check(state_frames == 28 && bad_state == 0, "every frame carries a state the filter produced");
        begin logic [31:0] u[$]; u = seen_ts.unique(); distinct = u.size(); check(distinct >= 4, "the state in the frames moves on from beat to beat"); end
        shift_err = worst_shift; shift_n = since_sweep;
        check(since_sweep > 100 && worst_shift < 1.0, "motion correction follows the filter's speed");
        check($abs(mv - 3.4515) < 0.04 && $abs(mom + 1.0) < 0.02, "the filter's speed and turn rate: about 3.45 m/s and -1 rad/s");
        check(states >= beats - 2 && dut.u_ukf.overruns == 0, "a state for every beat, none missed");

        // switching the source back to the host registers works
        // the host's state goes into the filter's own words, so the filter must be off and idle first
        q = '{8'h73}; wr(8'h02, q); #2_500_000_000;
        q = '{8'h78, 8'h56, 8'h34, 8'h12}; wr(8'h24, q);
        rd(8'h24, 4, q); check({q[3], q[2], q[1], q[0]} == 32'h12345678, "host-written state reads back");
        // the filter's tunable constants, through the address register 6C and the word at 70-73
        q = '{8'd96}; wr(8'h6C, q);                           // the first tunable constant
        rd(8'h70, 4, q); kept = {q[3], q[2], q[1], q[0]};
        check(kept == 32'h45dd8e86, "a tunable constant reads its built-in value before it is written");
        q = '{8'hDD, 8'hCC, 8'hBB, 8'h3A}; wr(8'h70, q);
        rd(8'h70, 4, q); check({q[3], q[2], q[1], q[0]} == 32'h3ABBCCDD, "a tunable constant reads back what the host wrote");
        q = '{8'd65}; wr(8'h6C, q);                           // a fixed constant (1.0): cannot be written
        q = '{8'h00, 8'h00, 8'h00, 8'h00}; wr(8'h70, q);
        rd(8'h70, 4, q); check({q[3], q[2], q[1], q[0]} == 32'h3F800000, "a fixed constant is read-only");
        q = '{8'd1}; wr(8'h6C, q);                            // any filter word can be read: word 1 is the state's x
        rd(8'h70, 4, q); check({q[3], q[2], q[1], q[0]} == dut.u_ukf.state_x, "filter storage reads through 70-73");
        q = '{8'd96}; wr(8'h6C, q);
        q = '{kept[7:0], kept[15:8], kept[23:16], kept[31:24]}; wr(8'h70, q);
        q = '{8'h01}; wr(8'h3C, q);
        n = frames; repeat (2) begin send_lidar_packet(); gap(13000); end
        check(frames == n + 4 && host_frames == 4, "state source switched to the host registers");

        $display("%0d checks, %0d wrong; %0d beats, %0d frames, %0d different filter states seen in frames, worst shift error %.2f counts over %0d firings",
                 checks, errors, beats, frames, distinct, shift_err, shift_n);
        $display("filter states produced: %0d", states);
        $fclose(fd);
        if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED");
        $finish;
    end
endmodule
