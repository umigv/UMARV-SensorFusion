# UKF top level

```text
Timestamper snapshot (ukf_pulse, ukf_busy low)
  ├─ Wheel delta → int_to_fp32 → deadwheel_velocity
  │                              ↓ FP32 wheel velocity, if available
  │   Retained state and S → UKF_pred_engine → UKF_pred_sequencer
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
microseconds, the sensor valid flags, and the raw fields. A wheel pulse starts
prediction when `ukf_busy` is low. A VN pulse may arrive during prediction.
The correction engine captures it for that in-flight result and waits
for both prediction and conversion before issuing correction arithmetic.
In the built-in path VN conversion waits until prediction releases the
scratchpad and shared MAC. It then writes the measurement to the residual
slots before correction starts.

While correction already owns a measurement, one later VN sample and its
noise factor can wait in the pending slot. An input arriving on the same
cycle that the slot is consumed can refill it. An additional pulse with
the slot still full raises `ukf_vn_overflow` and preserves the older sample.
A queued VN sample without an accompanying wheel pulse starts a VN-only
transaction. Wheel pulses during busy are still ignored. Both engines
share one `fp32_mac` and one `fp32_divsqrt`.

## Prediction

The default backend runs `pred_microcode_prog` in `UKF_pred_sequencer`.
It generates the central point and the positive and negative offsets from
the five rows of the original upper-triangular factor. The first pass
accumulates the weighted mean. The second pass regenerates the ten
noncentral points, updates the working factor, and then processes the
central point. Original state and factor stay live through that last
point. On completion the UKF copies the working factor directly into
retained S. No private prior or wheel-velocity copy is needed while the
built-in predictor owns the transaction.

The midpoint heading is `theta + omega * dt/2`. Position advances by
`(wheel_velocity - wheel_bias) * dt` along that heading. Heading advances
by `omega * dt`. Angular rate and bias carry through. VN-only beats skip
position propagation because wheel velocity is unavailable. The prediction
mean accumulates heading on the prior's locally continuous branch, wraps
the deviations, and wraps the published heading to `[-pi, pi)`.
The range-reduction and polynomial routines issue operations on the shared
MAC. The wrap check accepts finite angles within `[-128*pi, 128*pi]`.
Range reduction subtracts binary multiples of FP32 2*pi and requires at
most seven subtractions. Both trig calls reuse the same wrapped midpoint.

_Configuration_

- `PRED_DT` defaults to 2.5 ms and is independent of the snapshot timestamp.
  Set `WHEEL_VELOCITY_SCALE` for the same deployed interval.
- The default weights use dimension 5, alpha 0.5, beta 2, and kappa 0.
  The configuration supplies the sigma scale, noncentral mean weight,
  its square root, central mean weight, square root of the absolute
  central covariance weight, and the central update/downdate sign.
  Keep these values consistent when overriding them.
- Supply `INITIAL_STATE` and `INITIAL_S` for the starting prior.
  `PROCESS_S` is the packed upper triangle of the process noise factor.
  Its parameter values reload the existing 15-word working bank on each
  prediction. Supply a positive diagonal for the Cholesky operations.
- Present `measurement_s` with each VN pulse. The UKF captures the factor
  with the sample, including when it waits in the pending slot.

The historical backend ports remain available with `EXTERNAL_PREDICTION=1`.
An external backend samples the inputs on `prediction_start` and returns
state and factor together on `prediction_done`. Issue shared arithmetic
requests only with grant high and busy low. MAC rows with writeback wait
for valid. Accumulator-only MAC rows wait for busy to clear after issue.
DIVSQRT rows wait for done. An external backend must cancel outstanding
work on reset.

## Shared storage and handoff

The built-in UKF owns one global 52-word FP32 flip-flop bank in
`UKF.sv`. Prediction, VN conversion, and correction access it through
shared combinational read ports and addressed writes.

| Bank words | Contents |
| --- | --- |
| 0–4 | State |
| 5 | Held wheel velocity |
| 6–20 | Retained covariance factor |
| 21–36 | Shared scratch |
| 37–51 | Working process or measurement-noise factor |

During prediction, scratch offsets 0–4 hold the current point and then
its covariance-update vector. Offsets 7–11 hold the mean through both
passes. The remaining words hold phase-local temporaries. Angle argument,
square, and result use offsets 12, 13, and 6. The sign uses offset 15.
Prediction commits the mean and factor to the retained bank on its done
edge. VN conversion then uses the first four scratch words. Correction
uses those four words for residuals and the remaining twelve for its
temporaries. Correction updates the retained state and factor in place.

A VN sample may arrive while S_Q is still being updated, so the UKF
captures its ten-word runtime S_R in a separate configuration snapshot.
At correction load, S_R replaces the first ten S_Q words, after the
predicted S has been installed in the main bank. The next prediction
reloads all fifteen S_Q words from immutable `PROCESS_S`. A new S_R
comes from the accepted sample. The pending raw packet and its noise
snapshot remain independent. No additional state, factor, or converted
measurement bank is required.

See [the storage liveness inventory](../tests/srukf_storage_liveness.tsv)
for the complete register inventory, lifetimes and reuse decisions.

## Coordinates and heading

The existing `lat_to_xy` consumes FP32 latitude and longitude in **degrees**.
It uses a spherical local tangent-plane approximation that produces east
(`x`) and north (`y`) in metres, using the cosine of the reference latitude.
It accepts a one-cycle `start` while idle and publishes both outputs on a
one-cycle `xy_valid`. Its arithmetic is unchanged. Four local scratch words now cover the
sequential conversion phases.

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
The built-in engine writes conversion results directly into shared scratch
slots 0–3 on the conversion-done edge. The initial subtraction overwrites those
slots with the innovations, and the four whitened stages update them in
place. Raw pending VN input has separate storage. The standalone and
external-prediction builds capture conversion directly into their local
residual slots.

Correction wraps the initial heading innovation and each scalar update's
heading and heading residual into `[-pi, pi)` before using them in the next
stage. Angles outside the supported `[-128*pi, 128*pi]` range at a wrap step
fail correction. Use the same heading and rate convention in the prediction
backend and when configuring the measurement noise factor.

## Output and feedback

`ukf_valid` qualifies the registered correction commit. On that pulse,
the five outputs and `corrected_s` already contain the same correction
result. The registered handoff adds one cycle. When no update is active the ports
show the last estimate, subject to the failure status below. The state order is `[x, y, theta, omega, wheel_bias]`.
The factor is the row-major upper triangle
starting at packed word 0, with `P = SᵀS`. Measurement noise uses the full
upper-triangular factor `R = S_RᵀS_R`.

A rejected prediction or correction pulses `ukf_failed`. The built-in
predictor leaves the main prior untouched on prediction failure. A built-in
correction failure latches `ukf_reinit_required`, discards the pending VN
transaction, and blocks new wheel and VN input. It publishes no correction
result. The live output wires must be treated as invalid while correction
is updating them or the reset-required flag is set. Hold `rst_n` low to
reload `INITIAL_STATE` and `INITIAL_S` and clear the flag. Only submit fresh
inputs after reset. The external-backend option keeps the previous
correction rollback behavior and leaves `ukf_reinit_required` low.
`UKF_TEMP.sv` holds the historical wrapper and is outside the current UKF
source list.

Run `python3 tests/run_srukf_end_to_end.py` for the real prediction and
correction engines, numerical comparison against a dense software reference,
and measured cycle counts. `python3 tests/run_ukf_integration.py` exercises
the optional external backend handoff.
Run `python3 tests/run_pred_angle_math.py` for wrap and trig simulation.
The existing correction regressions are `run_corr_x.py`, `run_corr_xy.py`,
`run_corr_xyz.py`, and `run_corr_full.py` in `tests/`.

Run `python3 tests/run_srukf_schedule.py --check-deadlines` for staggered
releases and the pending-slot overflow/refill regression. Add
`--heading 398.5` for a bounded range-reduction stress case.

The 20 ns pre-placement STA flow uses `tests/synth_srukf.py --timing-map`
with the SKY130 HD Liberty, `tests/buffer_srukf_netlist.py` on the mapped
JSON, and `tests/srukf_timing.tcl` in OpenSTA after exporting the buffered
JSON as Verilog with Yosys `write_verilog -noattr -noexpr -simple-lhs`.

### Lowest-area integrated UKF map

The lowest-area tested SKY130 HD map uses flattened synthesis, default ABC
Liberty mapping, and post-map buffer trees with a fanout limit of 32. Run
these commands from the repository root with `liberty` set to the absolute
path of `sky130_fd_sc_hd__tt_025C_1v80.lib` and OpenSTA available as `sta`.

```sh
out=$(mktemp -d)
python3 -B tests/synth_srukf.py "$liberty" "$out" --flatten
python3 -B tests/buffer_srukf_netlist.py "$liberty" "$out/mapped.json" "$out/buffered.json" --fanout 32
yosys -Q -T -p "read_json $out/buffered.json; write_verilog -noattr -noexpr -simple-lhs $out/buffered.v"
python3 -B tests/report_srukf_area.py "$liberty" "$out/buffered.json" --flattened
LIBERTY="$liberty" MAPPED_VERILOG="$out/buffered.v" sta -exit tests/srukf_timing.tcl
```

Before the global-bank move, the buffered `UKF` netlist measured 190,441.40 µm² and 3,461 DFFs, with
+5.1449 ns pre-placement setup slack at 20 ns. Gate-level simulation matched
the RTL corrected state, covariance factor, and status timing for one
staggered wheel/VN transaction. These measurements cover the integrated
`UKF` top before physical placement and routing.
