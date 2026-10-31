`timescale 1ns/1ps
// Testbench for srukf_scheduler.
// The math units don't exist yet, so each done_* is faked as start_* delayed
// by a few cycles. Six scenarios run and the results are checked automatically.

module tb_srukf_scheduler;
    reg clk = 0;
    always #5 clk = ~clk;   // 100 MHz

    reg resetn, tick_1khz, gps_valid, odom_valid, init_done;
    wire start_sigma, start_predict, start_pred_cov, start_meas_map,
         start_meas_cov, start_gain, start_update;
    wire meas_select, estimate_valid, tick_overrun, busy;

    // fake math units: done = start delayed DLY cycles
    localparam DLY = 4;
    reg [DLY-1:0] d_sigma, d_predict, d_pred_cov, d_meas_map, d_meas_cov, d_gain, d_update;
    always @(posedge clk) begin
        d_sigma    <= {d_sigma[DLY-2:0],    start_sigma};
        d_predict  <= {d_predict[DLY-2:0],  start_predict};
        d_pred_cov <= {d_pred_cov[DLY-2:0], start_pred_cov};
        d_meas_map <= {d_meas_map[DLY-2:0], start_meas_map};
        d_meas_cov <= {d_meas_cov[DLY-2:0], start_meas_cov};
        d_gain     <= {d_gain[DLY-2:0],     start_gain};
        d_update   <= {d_update[DLY-2:0],   start_update};
    end

    srukf_scheduler dut (
        .clk(clk), .resetn(resetn),
        .tick_1khz(tick_1khz), .gps_valid(gps_valid), .odom_valid(odom_valid),
        .init_done(init_done),
        .done_sigma(d_sigma[DLY-1]),       .done_predict(d_predict[DLY-1]),
        .done_pred_cov(d_pred_cov[DLY-1]), .done_meas_map(d_meas_map[DLY-1]),
        .done_meas_cov(d_meas_cov[DLY-1]), .done_gain(d_gain[DLY-1]),
        .done_update(d_update[DLY-1]),
        .start_sigma(start_sigma), .start_predict(start_predict),
        .start_pred_cov(start_pred_cov), .start_meas_map(start_meas_map),
        .start_meas_cov(start_meas_cov), .start_gain(start_gain),
        .start_update(start_update),
        .meas_select(meas_select), .estimate_valid(estimate_valid),
        .tick_overrun(tick_overrun), .busy(busy)
    );

    // count what the DUT did so each scenario can check it
    integer n_est, n_update, n_gps_upd, n_odom_upd, errors;
    always @(posedge clk) begin
        if (estimate_valid) n_est = n_est + 1;
        if (start_update) begin
            n_update = n_update + 1;
            if (meas_select == 0) n_gps_upd  = n_gps_upd  + 1;
            else                  n_odom_upd = n_odom_upd + 1;
        end
    end

    // one-cycle pulses on the inputs (task args are copied, so one task per signal)
    task pulse_tick; begin @(negedge clk); tick_1khz  = 1; @(negedge clk); tick_1khz  = 0; end endtask
    task pulse_gps;  begin @(negedge clk); gps_valid  = 1; @(negedge clk); gps_valid  = 0; end endtask
    task pulse_odom; begin @(negedge clk); odom_valid = 1; @(negedge clk); odom_valid = 0; end endtask

    task clear_counts;
        begin n_est = 0; n_update = 0; n_gps_upd = 0; n_odom_upd = 0; end
    endtask

    task check(input cond, input [511:0] msg);
        begin
            if (!cond) begin errors = errors + 1; $display("FAIL: %0s", msg); end
            else $display("ok:   %0s", msg);
        end
    endtask

    initial begin
        errors = 0; clear_counts;
        resetn = 0; tick_1khz = 0; gps_valid = 0; odom_valid = 0; init_done = 0;
        repeat (3) @(negedge clk);
        resetn = 1;

        // 1. cold start: a tick before init_done must be ignored
        pulse_tick;
        repeat (20) @(negedge clk);
        check(n_est == 0 && !busy, "no activity before init_done");
        init_done = 1;
        repeat (2) @(negedge clk);

        // 2. tick with no sensor data: predict only
        clear_counts;
        pulse_tick;
        repeat (60) @(negedge clk);
        check(n_est == 1,    "dead-reckoning tick produced one estimate");
        check(n_update == 0, "dead-reckoning tick ran no correction");

        // 3. odometry arrives mid-tick, then the tick
        clear_counts;
        pulse_odom;
        repeat (7) @(negedge clk);
        pulse_tick;
        repeat (80) @(negedge clk);
        check(n_est == 1 && n_update == 1,       "odom tick: one estimate, one correction");
        check(n_odom_upd == 1 && n_gps_upd == 0, "odom tick used meas_select=ODOM");

        // 4. GPS and odom both pending on the same tick
        clear_counts;
        pulse_gps;
        pulse_odom;
        pulse_tick;
        repeat (120) @(negedge clk);
        check(n_est == 1 && n_update == 2,       "gps+odom tick: one estimate, two corrections");
        check(n_gps_upd == 1 && n_odom_upd == 1, "gps+odom tick: one of each");

        // 5. a tick arriving while still busy
        clear_counts;
        check(tick_overrun == 0, "no overrun so far");
        pulse_tick;
        repeat (3) @(negedge clk);
        pulse_tick;
        repeat (80) @(negedge clk);
        check(tick_overrun == 1, "overrun flagged when tick arrives mid-work");

        // 6. the next clean tick must not re-apply old samples
        clear_counts;
        pulse_tick;
        repeat (60) @(negedge clk);
        check(n_update == 0, "stale measurements were not re-applied");

        if (errors == 0) $display("\nALL CHECKS PASSED");
        else             $display("\n%0d CHECK(S) FAILED", errors);
        $finish;
    end
endmodule
