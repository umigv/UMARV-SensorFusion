// SIMULATION ONLY. A stand-in for the Kalman filter, with the port list proposed for the real one
// (ukf_top), so the rest of the chip can be wired up and tested before the filter exists.
// It is not a filter and cannot be synthesised: it uses real-number arithmetic.
//
// What it does on every measurement pulse:
//     omega = the VN-300's turn rate                       (kept from before if not valid)
//     theta = the VN-300's yaw, degrees to radians         (else theta + omega * dt)
//     v     = wheel counts * M_PER_COUNT / dt              (kept from before if not valid)
//     x    += v * cos(theta) * dt        y += v * sin(theta) * dt
// and LATENCY clocks later it presents the result with one state_valid pulse, the way a real
// filter would after its arithmetic. Latitude and longitude are ignored.
//
// Assumptions the real filter's authors must confirm or replace:
//   - measurements arrive raw: yaw in degrees and turn rate in rad/s as IEEE single precision,
//     latitude and longitude in degrees as double precision, wheel travel as a signed count
//   - outputs are IEEE single precision: metres, radians, rad/s, m/s (forward speed)
//   - state_ts is the beat_ts of the measurements the state was computed from
//   - the state is ready before the next beat
//   - tuning values arrive as byte writes (cfg_we, cfg_addr, cfg_wdata) at addresses 6C-7F
module ukf_top #(
    parameter real M_PER_COUNT = 0.2 / 16384.0,     // one wheel turn = 0.2 m, 16384 counts per turn
    parameter int  LATENCY     = 5000               // clocks from the measurements to the state (100 us)
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,

    input  logic        meas_pulse,
    input  logic [31:0] beat_ts,
    input  logic        vn_valid,
    input  logic [31:0] vn_yaw_f32,
    input  logic [31:0] vn_rate_f32,
    input  logic [63:0] vn_lat_f64,
    input  logic [63:0] vn_lon_f64,
    input  logic        dw_valid,
    input  logic signed [27:0] dw_delta,

    input  logic        cfg_we,
    input  logic [6:0]  cfg_addr,
    input  logic [7:0]  cfg_wdata,

    output logic        state_valid,
    output logic [31:0] state_ts,
    output logic [31:0] state_x, state_y, state_theta, state_omega, state_v
);
    localparam real PI = 3.14159265358979;
    real x = 0, y = 0, theta = 0, omega = 0, v = 0, dt;
    logic [31:0] prev_ts, ts_held; bit have_prev = 0;
    int countdown = 0, cfg_writes = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x = 0; y = 0; theta = 0; omega = 0; v = 0; have_prev = 0; countdown = 0;
            state_valid <= 1'b0; state_ts <= '0;
            state_x <= '0; state_y <= '0; state_theta <= '0; state_omega <= '0; state_v <= '0;
        end else begin
            state_valid <= 1'b0;
            if (cfg_we && cfg_addr >= 7'h6C) cfg_writes++;
            if (meas_pulse && enable) begin
                dt = have_prev ? (beat_ts - prev_ts) * 1.0e-6 : 0.0;
                if (vn_valid) omega = $bitstoshortreal(vn_rate_f32);
                if (vn_valid) theta = $bitstoshortreal(vn_yaw_f32) * PI / 180.0; else theta = theta + omega * dt;
                if (dw_valid && dt > 0) v = dw_delta * M_PER_COUNT / dt;
                x = x + v * $cos(theta) * dt;
                y = y + v * $sin(theta) * dt;
                prev_ts = beat_ts; have_prev = 1;
                ts_held = beat_ts;
                countdown = LATENCY;
            end else if (countdown > 0) begin
                countdown--;
                if (countdown == 0) begin
                    state_ts    <= ts_held;
                    state_x     <= $shortrealtobits(shortreal'(x));
                    state_y     <= $shortrealtobits(shortreal'(y));
                    state_theta <= $shortrealtobits(shortreal'(theta));
                    state_omega <= $shortrealtobits(shortreal'(omega));
                    state_v     <= $shortrealtobits(shortreal'(v));
                    state_valid <= 1'b1;
                end
            end
        end
    end
    wire _unused = &{vn_lat_f64, vn_lon_f64, cfg_wdata, 1'b0};
endmodule
