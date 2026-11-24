# UKF top level

```text
Timestamper snapshot (ukf_pulse, ukf_busy low)
  ├─ Wheel delta → int_to_fp32 → deadwheel_velocity
  │                              ↓ FP32 wheel velocity, if available
  │   Retained state and S → UKF_pred_engine → prediction backend interface
  │                                               ↓ state + S on prediction_done
  └─ Captured VN packet → FP64-to-FP32 → lat_to_xy → east/north metres
                          VN yaw/rate → ENU radians and rad/s
                                                   ↓ x/y/theta/omega on convert_done
                                  UKF_corr_engine waits for both completions
                                                   ↓
                             Four whitened scalar updates in corr_microcode
                                                   ↓ state + S on ukf_valid
                                 Outputs and prior for the next prediction
```

The timestamper supplies a one-cycle `ukf_pulse`, the snapshot timestamp in
microseconds, the sensor valid flags, and the raw fields. Pulses while
`ukf_busy` is high are ignored. A beat with either sensor valid starts
prediction. A beat with VN data also starts correction for that same snapshot.
A wheel-only beat advances the retained prediction without a correction pulse.
A VN-only beat requests prediction with `prediction_wheel_valid` low.

The wheel converter, coordinate converter, heading conversion, and correction
microcode use the same `fp32_mac`. Coordinate conversion starts after wheel
preprocessing releases the MAC. At that point the prediction backend and
converter may run concurrently. Correction captures each result independently
and starts once it has both. The MAC's `reset` pin is active low. The existing
correction divider/square-root unit remains dedicated to correction.

## Prediction backend handoff

Neither merge parent supplies an executable SR-UKF prediction backend.
`pred_microcode_prog` is an incomplete instruction table with repeated PC
entries. It has no execution sequencer, completed sigma-point propagation,
mean or covariance reduction, or factor output. Its `UOP_TRIG` instructions
and the incoming wheel preprocessing are preserved.

Sample `prediction_prior`, `prediction_prior_s`, `prediction_ts`,
`prediction_wheel_valid`, and `prediction_wheel_velocity` on
`prediction_start`. A backend must accept this pulse and respond once with
`prediction_done`, `prediction_result`, and `prediction_result_s`. On that
edge, `prediction_result.pred_valid` distinguishes success from failure.
The top level and testbench support a completion as early as the start edge.
There is one outstanding transaction and no timeout. Reset must cancel the
outstanding work in the backend as well as in the UKF. A stale reply could
otherwise be mistaken for the next transaction's reply.

Configure the FP32 `WHEEL_VELOCITY_SCALE` for the deployed wheel and sample
interval, and supply `INITIAL_STATE` and `INITIAL_S` for the starting prior.
Keep the full `measurement_s` stable through the accepted VN transaction.

## Coordinates and heading

The existing `lat_to_xy` consumes FP32 latitude and longitude in **degrees**.
It uses a spherical local tangent-plane approximation that produces east
(`x`) and north (`y`) in metres, using the cosine of the reference latitude.
It accepts a one-cycle `start` while idle and publishes both outputs on a
one-cycle `xy_valid`. Its arithmetic and memory map are preserved.

The converter reads origin longitude at address 70 and origin latitude at 71.
The incoming RTL and documentation supply no geographic origin values. Set the
top-level FP32 parameters `ORIGIN_LONGITUDE` and `ORIGIN_LATITUDE` to the
deployed origin. The default radius is 6,371,000 m and is configurable through
`EARTH_RADIUS`. The adapter supplies 1.0 at 96, radians per degree at 107,
the radius at 108, and the six existing cosine coefficient slots at 109–114.

`vn_to_correction` rounds the raw FP64 coordinates to nearest FP32 before
passing them to `lat_to_xy`. That preserves the converter's original FP32
interface and subtraction order. Coordinate quantization can be significant
for nearby points, especially at large absolute longitudes. Subtracting the
origin in FP64 would require a separate precision change.

VN yaw is clockwise from north in degrees. The adapter supplies
`theta = pi/2 - yaw * pi/180` in radians, counterclockwise from east. VN body Z
points down, so the adapter supplies `omega = -vn_rate` in radians per second.
Angle wrapping remains deferred. Use the same heading and rate convention in
the prediction backend and when configuring the measurement noise factor.

## Output and feedback

`ukf_valid` now qualifies the completed correction pair, independent of the
retained `ukf_beat` input. On that pulse, the five state words and
`corrected_s` come from the same successful fourth update. Between valid
pulses the ports show the retained prior. The state order is
`[x, y, theta, omega, wheel_bias]`. The factor is the row-major upper triangle
starting at packed word 0, with `P = SᵀS`. Measurement noise uses the full
upper-triangular factor `R = S_RᵀS_R`.

A rejected prediction or downdate pulses `ukf_failed` and suppresses
`ukf_valid`. A failed correction leaves the matched successful prediction as
the next prior. A successful correction replaces both retained values.
`UKF_TEMP.sv` holds the historical wrapper and is outside the current UKF
source list.

Run `python3 tests/run_ukf_integration.py` for a simulation using the real
sensor and correction RTL and a test driver for the missing prediction backend.
The existing correction regressions are `run_corr_x.py`, `run_corr_xy.py`,
`run_corr_xyz.py`, and `run_corr_full.py` in `tests/`.
