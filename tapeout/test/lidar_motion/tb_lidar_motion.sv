// Test of lidar_motion with lidar_cordic. Firings are generated with a known turn rate and speed
// that change every 2.5 ms, and every result is compared with the same motion worked out in
// real numbers (true sine and cosine, no small-angle shortcuts).
module tb_lidar_motion;
    logic clk = 0; always #10 clk = ~clk;
    logic rst = 1;
    logic corr_rot_en = 1, corr_shift_en = 1, scale_4mm = 0, four_offsets = 0;
    logic [15:0] firing_period_ns = 16'd55296; logic signed [15:0] yaw_offset = 0;
    logic state_valid = 0; logic [31:0] state_omega = 0, state_v = 0;
    logic firing_start = 0, firing_second = 0, sweep_start = 0; logic [15:0] azimuth = 0;
    logic cordic_start, cordic_busy, cordic_done, cov, cerr; logic signed [17:0] cordic_theta, cordic_sin, cordic_cos;
    logic fire_done, fire_sweep_start, state_seen, overrun;
    logic signed [15:0] sin0, cos0, sin1, cos1, sin2, cos2, sin3, cos3, shift_x, shift_y;
    lidar_motion dut (.*);
    lidar_cordic cordic (.clk, .resetn(!rst), .start(cordic_start), .mode(2'd0), .theta(cordic_theta), .sin_theta(cordic_sin), .cos_theta(cordic_cos),
                         .busy(cordic_busy), .done(cordic_done), .overflow(cov), .error(cerr));

    localparam real PI = 3.14159265358979;
    real w_now = 0, v_now = 0, T = 0, X = 0, Y = 0;         // truth: rad/s, m/s, radians, metres
    real worst_ang, worst_shift, worst_ang_all = 0, worst_shift_all = 0; int fails = 0, firings = 0, overruns = 0, clocks_max = 0;
    always @(posedge clk) overruns += overrun;

    task automatic set_state(input real w, input real v);
        @(posedge clk); state_omega <= $shortrealtobits(shortreal'(w)); state_v <= $shortrealtobits(shortreal'(v)); state_valid <= 1;
        @(posedge clk); state_valid <= 0;
        w_now = (w > 7.99976) ? 7.99976 : (w < -7.99976) ? -7.99976 : w;
        v_now = (v > 7.99976) ? 7.99976 : (v < -7.99976) ? -7.99976 : v;
        repeat (12) @(posedge clk);
    endtask

    bit hdl = 0; int az_inc = 40;
    int az_prev = 0, az_step = 0;
    task automatic fire(input int az, input bit second, input bit sweep, input int gap);
        real tf, step_m, a_eff, th, es, ec, ex, ey, d; int n, c0; real offs[4];
        tf = firing_period_ns * 1.0e-9; step_m = scale_4mm ? 0.004 : 0.002;
        if (sweep) begin T = 0; X = 0; Y = 0; end
        else begin T += w_now * tf; X += v_now * tf * $cos(T); Y += v_now * tf * $sin(T); end
        if (!second) begin az_step = (az - az_prev + 36000) % 36000; az_prev = az; end
        a_eff = az + (second ? az_step / 2 : 0) + yaw_offset;
        @(posedge clk); azimuth <= az; firing_second <= second; sweep_start <= sweep; firing_start <= 1;
        @(posedge clk); firing_start <= 0; sweep_start <= 0; c0 = 0;
        while (!fire_done) begin @(posedge clk); c0++; end
        if (c0 > clocks_max) clocks_max = c0;
        #1;
        offs[0] = four_offsets ? 4.2 : 0; offs[1] = four_offsets ? 1.4 : 0; offs[2] = four_offsets ? -1.4 : 0; offs[3] = four_offsets ? -4.2 : 0;
        for (int k = 0; k < 4; k++) begin
            th = (a_eff / 100.0 + offs[k]) * PI / 180.0 - (corr_rot_en ? T : 0);
            es = (k == 0 ? sin0 : k == 1 ? sin1 : k == 2 ? sin2 : sin3) / 32768.0 - $sin(th);
            ec = (k == 0 ? cos0 : k == 1 ? cos1 : k == 2 ? cos2 : cos3) / 32768.0 - $cos(th);
            d = $sqrt(es * es + ec * ec); if (d > worst_ang) worst_ang = d;
        end
        ex = shift_x - (corr_shift_en ? X / step_m : 0); ey = shift_y - (corr_shift_en ? Y / step_m : 0);
        d = ($abs(ex) > $abs(ey)) ? $abs(ex) : $abs(ey); if (d > worst_shift) worst_shift = d;
        if (fire_sweep_start !== sweep) begin fails++; $display("FAIL sweep flag at firing %0d", firings); end
        firings++;
        repeat (gap) @(posedge clk);
    endtask

    // one sweep: 'blocks' data blocks, state changing every 2.5 ms as the filter would
    task automatic sweep(input string name, input int blocks, input real w0, input real w1, input real v0, input real v1, input int az0, input int gap);
        int az, per_state; real f, tol_ang, tol_shift;
        worst_ang = 0; worst_shift = 0; az = az0;
        per_state = 2500000 / firing_period_ns;
        set_state(w0, v0);
        for (int b = 0; b < blocks; b++) begin
            f = b * 1.0 / blocks;
            if (((four_offsets ? b : 2 * b) % per_state) < (four_offsets ? 1 : 2) && b > 0) set_state(w0 + (w1 - w0) * f, v0 + (v1 - v0) * f);
            fire(az % 36000, 0, b == 0, gap);
            if (!four_offsets && !hdl) fire(az % 36000, 1, 0, gap);
            az += az_inc;
        end
        tol_ang = 3.0e-4; tol_shift = 1.0;
        if (worst_ang > tol_ang || worst_shift > tol_shift) begin fails++; $display("FAIL %s", name); end
        $display("%-44s turned %7.3f deg, moved %6.1f, %6.1f counts | worst angle error %.1e rad, worst shift error %.2f counts",
                 name, T * 180 / PI, X / (scale_4mm ? 0.004 : 0.002), Y / (scale_4mm ? 0.004 : 0.002), worst_ang, worst_shift);
        if (worst_ang > worst_ang_all) worst_ang_all = worst_ang; if (worst_shift > worst_shift_all) worst_shift_all = worst_shift;
    endtask

    initial begin
        repeat (5) @(posedge clk); rst <= 0; repeat (3) @(posedge clk);
        // before any state has arrived: angles from the lidar alone, no shift, and state_seen low
        worst_ang = 0; worst_shift = 0;
        for (int b = 0; b < 20; b++) begin fire(100 + 40 * b, 0, b == 0, 200); fire(100 + 40 * b, 1, 0, 200); end
        $display("%-44s worst angle error %.1e rad, worst shift error %.2f counts, state_seen %b", "no filter state yet", worst_ang, worst_shift, state_seen);
        if (state_seen !== 1'b0 || worst_ang > 3.0e-4 || worst_shift > 0.0) begin fails++; $display("FAIL no filter state yet"); end
        // one full rotation is 900 blocks (0.4 degree per block at 600 RPM)
        sweep("standing still", 900, 0, 0, 0, 0, 0, 200);
        sweep("straight ahead, 2.2 m/s", 900, 0, 0, 2.2, 2.2, 0, 200);
        sweep("turning left 1 rad/s at 2.2 m/s", 900, 1.0, 1.0, 2.2, 2.2, 35000, 200);
        sweep("turning right 2 rad/s at 2.2 m/s", 900, -2.0, -2.0, 2.2, 2.2, 12345, 200);
        sweep("turn rate and speed changing", 900, -1.5, 2.0, 0.3, 2.2, 20000, 200);
        sweep("reversing while turning", 900, 0.7, 0.7, -1.0, -1.0, 9000, 200);
        sweep("spinning on the spot, 3 rad/s", 900, 3.0, 3.0, 0, 0, 0, 200);
        yaw_offset = -9000;
        sweep("lidar mounted 90 degrees round", 900, 1.0, 1.0, 2.2, 2.2, 0, 200);
        yaw_offset = 0; scale_4mm = 1;
        sweep("4 mm output step", 900, 1.0, 1.0, 2.2, 2.2, 0, 200);
        scale_4mm = 0; corr_rot_en = 0;
        sweep("rotation correction switched off", 300, 1.0, 1.0, 2.2, 2.2, 0, 200);
        corr_rot_en = 1; corr_shift_en = 0;
        sweep("shift correction switched off", 300, 1.0, 1.0, 2.2, 2.2, 0, 200);
        corr_shift_en = 1;
        // slow lidar, 300 RPM: twice as many firings per sweep
        az_inc = 20; sweep("300 RPM sweep, turning 1 rad/s", 1800, 1.0, 1.0, 2.2, 2.2, 0, 200); az_inc = 40;
        // VLP-32C: 32 lasers, one firing per block, four laser offsets, 4 mm
        four_offsets = 1; scale_4mm = 1; az_inc = 20;
        sweep("VLP-32C mode", 1800, 1.0, -1.0, 2.2, 1.0, 500, 400);
        // HDL-32E: 32 lasers, one firing per block, 46.08 us
        four_offsets = 0; scale_4mm = 0; hdl = 1; firing_period_ns = 16'd46080; az_inc = 17;
        sweep("HDL-32E mode", 2170, 1.0, 1.0, 2.2, 2.2, 0, 400);
        hdl = 0; firing_period_ns = 16'd55296; az_inc = 40;
        // values a filter should never send: clamped, not wild
        sweep("turn rate 1e30, speed -1e30 (clamped to 8)", 50, 1.0e30, 1.0e30, -1.0e30, -1.0e30, 0, 200);
        @(posedge clk); state_omega <= 32'h7FC00000; state_v <= 32'h7FC00000; state_valid <= 1; @(posedge clk); state_valid <= 0; w_now = 0; v_now = 0; repeat (12) @(posedge clk);
        worst_ang = 0; worst_shift = 0;
        for (int b = 0; b < 30; b++) begin fire(40 * b, 0, b == 0, 200); fire(40 * b, 1, 0, 200); end
        $display("%-44s worst angle error %.1e rad, worst shift error %.2f counts", "not-a-number turn rate and speed (treated as 0)", worst_ang, worst_shift);
        if (worst_ang > 3.0e-4 || worst_shift > 1.0) begin fails++; $display("FAIL not-a-number case"); end
        // firings arriving faster than they can be served must be flagged
        @(posedge clk); firing_start <= 1; @(posedge clk); @(posedge clk); @(posedge clk); firing_start <= 0; repeat (400) @(posedge clk);
        $display("firings checked %0d, slowest firing %0d clocks, overrun flags %0d (2 expected from the last check)", firings, clocks_max, overruns);
        $display("worst angle error %.1e rad (%.4f deg), worst shift error %.2f counts", worst_ang_all, worst_ang_all * 180 / PI, worst_shift_all);
        $display("%s", (fails == 0 && overruns == 2) ? "TEST PASSED" : "TEST FAILED");
        $finish;
    end
endmodule
