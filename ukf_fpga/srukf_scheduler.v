`timescale 1ns/1ps
// Control FSM for the SR-UKF (square-root unscented Kalman filter).
//
// Once per 1 kHz tick it runs the predict step, then one correction for each
// sensor that delivered data since the last tick (GPS first, then odometry,
// matching the C golden model), then publishes the estimate. It does no math:
// each math unit is driven by a one-cycle start_* pulse and reports back with
// a one-cycle done_* pulse, so unit latency can change without touching this.

module srukf_scheduler (
    input  wire clk,             // fabric clock (~100 MHz FPGA, 500 MHz ASIC target)
    input  wire resetn,          // synchronous active-low reset

    input  wire tick_1khz,       // 1-cycle pulse every 1 ms; starts a filter period
    input  wire gps_valid,       // 1-cycle pulse: new GNSS x,y written to input regs
    input  wire odom_valid,      // 1-cycle pulse: new v, theta, omega written to input regs
    input  wire init_done,       // level: initial state and S loaded; filter may run

    // done pulses from the math units
    input  wire done_sigma,      // sigma-point generator finished
    input  wire done_predict,    // motion model applied to all sigma points
    input  wire done_pred_cov,   // QR of predicted deviations -> new S
    input  wire done_meas_map,   // h(x) applied to all sigma points
    input  wire done_meas_cov,   // QR of measurement deviations -> S_z, P_xz
    input  wire done_gain,       // Kalman gain K solved
    input  wire done_update,     // state corrected, S downdated

    // start pulses to the math units (1 cycle each)
    output reg  start_sigma,
    output reg  start_predict,
    output reg  start_pred_cov,
    output reg  start_meas_map,
    output reg  start_meas_cov,
    output reg  start_gain,
    output reg  start_update,

    output reg  meas_select,     // which model the shared correction path uses: 0 = GPS (x,y), 1 = ODOM (v,theta,omega)
    output reg  estimate_valid,  // 1-cycle pulse: output state registers hold this tick's result
    output reg  tick_overrun,    // sticky until reset: a tick arrived while still working
    output reg  busy             // level: high from tick until OUTPUT, low while idle
);

    // State encoding. Numbers follow the S0..S10 sketch in the design doc.
    localparam [3:0]
        WAIT_FIX      = 4'd0,  // cold start, wait for init_done
        IDLE          = 4'd1,  // wait for next tick
        SIGMA_GEN     = 4'd2,  // 11 sigma points from x and S
        PREDICT_PASS1 = 4'd3,  // f(x) on each sigma point, accumulate mean
        PREDICT_PASS2 = 4'd4,  // deviations + QR -> predicted S
        CHECK_READY   = 4'd5,  // any sensor data pending?
        MEAS_SIGMA    = 4'd6,  // sigma points from predicted x and S
        MEAS_PASS1    = 4'd7,  // h(x) on each sigma point, accumulate z mean
        MEAS_PASS2    = 4'd8,  // deviations + QR -> S_z, cross-covariance P_xz
        GAIN_SOLVE    = 4'd9,  // K = P_xz * inv(S_z S_z^T)
        UPDATE        = 4'd10, // x += K*(z - z_mean), S = cholupdate(S, K*S_z, -1)
        OUTPUT        = 4'd11; // publish x, y, theta, omega, v

    reg [3:0] state, next_state;

    // Sensor pulses are not phase-locked to the tick, so they can arrive at any
    // cycle. Latch each into a pending bit; the FSM clears it when it starts
    // that sensor's correction so a sample is never applied twice.
    reg gps_pending, odom_pending;
    reg clr_gps, clr_odom;

    always @(posedge clk) begin
        if (!resetn) begin
            gps_pending  <= 1'b0;
            odom_pending <= 1'b0;
        end else begin
            if (clr_gps)         gps_pending  <= 1'b0;
            else if (gps_valid)  gps_pending  <= 1'b1;
            if (clr_odom)        odom_pending <= 1'b0;
            else if (odom_valid) odom_pending <= 1'b1;
        end
    end

    // state register
    always @(posedge clk) begin
        if (!resetn) state <= WAIT_FIX;
        else         state <= next_state;
    end

    // next-state logic. Default is hold, so each math state waits for its done.
    always @(*) begin
        next_state = state;
        case (state)
            WAIT_FIX:       if (init_done)     next_state = IDLE;
            IDLE:           if (tick_1khz)     next_state = SIGMA_GEN;

            SIGMA_GEN:      if (done_sigma)    next_state = PREDICT_PASS1;
            PREDICT_PASS1:  if (done_predict)  next_state = PREDICT_PASS2;
            PREDICT_PASS2:  if (done_pred_cov) next_state = CHECK_READY;

            // no pending data -> skip correction (dead reckoning, the common case)
            CHECK_READY:    if (gps_pending || odom_pending) next_state = MEAS_SIGMA;
                            else                             next_state = OUTPUT;

            MEAS_SIGMA:     if (done_sigma)    next_state = MEAS_PASS1;
            MEAS_PASS1:     if (done_meas_map) next_state = MEAS_PASS2;
            MEAS_PASS2:     if (done_meas_cov) next_state = GAIN_SOLVE;
            GAIN_SOLVE:     if (done_gain)     next_state = UPDATE;

            // back to CHECK_READY so a second sensor pending gets its own pass
            UPDATE:         if (done_update)   next_state = CHECK_READY;

            OUTPUT:                            next_state = IDLE;
            default:                           next_state = WAIT_FIX;
        endcase
    end

    // Output logic. `entering` is high only on the cycle a transition happens,
    // so each start_* fires exactly once when its state is entered and is not
    // re-fired while waiting for done.
    wire entering = (next_state != state);

    always @(posedge clk) begin
        if (!resetn) begin
            start_sigma    <= 1'b0;
            start_predict  <= 1'b0;
            start_pred_cov <= 1'b0;
            start_meas_map <= 1'b0;
            start_meas_cov <= 1'b0;
            start_gain     <= 1'b0;
            start_update   <= 1'b0;
            meas_select    <= 1'b0;
            estimate_valid <= 1'b0;
            tick_overrun   <= 1'b0;
            busy           <= 1'b0;
            clr_gps        <= 1'b0;
            clr_odom       <= 1'b0;
        end else begin
            // all pulses default low each cycle
            start_sigma    <= 1'b0;
            start_predict  <= 1'b0;
            start_pred_cov <= 1'b0;
            start_meas_map <= 1'b0;
            start_meas_cov <= 1'b0;
            start_gain     <= 1'b0;
            start_update   <= 1'b0;
            estimate_valid <= 1'b0;
            clr_gps        <= 1'b0;
            clr_odom       <= 1'b0;

            // busy tracks tick -> OUTPUT; a tick while busy means we overran
            if (state == IDLE && next_state == SIGMA_GEN) busy <= 1'b1;
            if (state == OUTPUT)                           busy <= 1'b0;
            if (tick_1khz && busy)                         tick_overrun <= 1'b1;

            if (entering) begin
                case (next_state)
                    SIGMA_GEN:     start_sigma    <= 1'b1;
                    PREDICT_PASS1: start_predict  <= 1'b1;
                    PREDICT_PASS2: start_pred_cov <= 1'b1;
                    MEAS_SIGMA: begin
                        // pick the sensor for this pass; GPS has priority so
                        // the order is always GPS then ODOM. Reuses the same
                        // sigma generator as the predict step.
                        if (gps_pending) begin
                            meas_select <= 1'b0;
                            clr_gps     <= 1'b1;
                        end else begin
                            meas_select <= 1'b1;
                            clr_odom    <= 1'b1;
                        end
                        start_sigma <= 1'b1;
                    end
                    MEAS_PASS1:    start_meas_map <= 1'b1;
                    MEAS_PASS2:    start_meas_cov <= 1'b1;
                    GAIN_SOLVE:    start_gain     <= 1'b1;
                    UPDATE:        start_update   <= 1'b1;
                    OUTPUT:        estimate_valid <= 1'b1;  // no math, just flag the result
                    default: ;
                endcase
            end
        end
    end

endmodule
