`timescale 1ns/1ps
// Runs ukf_top for a few beats and compares every storage word with the exact model's (gen_tb.py),
// bit for bit, after each beat. Also reports how many clocks a beat takes.
module tb_ukf;
    logic clk = 0, rst_n = 0, enable = 1, meas_pulse = 0, vn_valid = 0, dw_valid = 0, state_valid;
    logic [31:0] beat_ts = 0, vn_yaw_f32 = 0, vn_rate_f32 = 0, state_ts, state_x, state_y, state_theta, state_omega, state_v;
    logic [63:0] vn_lat_f64 = 0, vn_lon_f64 = 0;
    logic signed [27:0] dw_delta = 0;
    ukf_top dut (.clk, .rst_n, .enable, .meas_pulse, .beat_ts, .vn_valid, .vn_yaw_f32, .vn_rate_f32, .vn_lat_f64, .vn_lon_f64,
                 .dw_valid, .dw_delta, .cfg_we(1'b0), .cfg_addr(7'd0), .cfg_wdata(8'd0),
                 .state_valid, .state_ts, .state_x, .state_y, .state_theta, .state_omega, .state_v);
    always #10 clk = ~clk;

    logic [263:0] stim [0:255];
    logic [2115:0] expect_ [0:255];
    integer n, i, w, bad, clocks, worst, got_valid, words_checked;
    logic [63:0] mask;
    logic [31:0] want;
    always @(posedge clk) if (state_valid) got_valid = 1;

    initial begin
        $readmemh("stim.hex", stim); $readmemh("expect.hex", expect_);
        n = 0; while (n < 256 && stim[n] !== 264'bx) n = n + 1;
        bad = 0; worst = 0; words_checked = 0;
        repeat (4) @(posedge clk); rst_n = 1; repeat (40) @(posedge clk);
        for (i = 0; i < n; i = i + 1) begin
            @(negedge clk);
            {vn_valid, dw_valid} = stim[i][257:256];
            vn_yaw_f32 = stim[i][255:224]; vn_rate_f32 = stim[i][223:192]; vn_lat_f64 = stim[i][191:128]; vn_lon_f64 = stim[i][127:64];
            dw_delta = stim[i][59:32]; beat_ts = stim[i][31:0];
            got_valid = 0; meas_pulse = 1; @(negedge clk); meas_pulse = 0; clocks = 1;
            @(negedge clk);
            while (dut.q != 3'd1) begin @(negedge clk); clocks = clocks + 1; if (clocks > 400000) begin $display("HANG in beat %0d at step %0d", i, dut.pc); $finish; end end
            repeat (4) @(negedge clk);
            if (clocks > worst) worst = clocks;
            if (got_valid !== expect_[i][2112]) begin bad = bad + 1; $display("beat %0d: new state %0d, expected %0d", i, got_valid, expect_[i][2112]); end
            mask = expect_[i][2111:2048];
            for (w = 0; w < 64; w = w + 1) if (mask[w]) begin
                want = expect_[i][32*w +: 32]; words_checked = words_checked + 1;
                if (dut.u_store.q[w] !== want) begin
                    bad = bad + 1;
                    if (bad <= 12) $display("beat %0d word %0d: got %h want %h", i, w, dut.u_store.q[w], want);
                end
            end
            $display("beat %0d: %0d clocks (%0d%% of a 2.5 ms beat), x %f y %f theta %f omega %f v %f", i, clocks, clocks / 1250,
                     $bitstoshortreal(state_x), $bitstoshortreal(state_y), $bitstoshortreal(state_theta), $bitstoshortreal(state_omega), $bitstoshortreal(state_v));
        end
        $display("%0d beats, %0d words compared, %0d wrong; longest beat %0d clocks; overruns %0d", n, words_checked, bad, worst, dut.overruns);
        if (bad == 0) $display("TEST PASSED"); else $display("TEST FAILED");
        $finish;
    end
endmodule
