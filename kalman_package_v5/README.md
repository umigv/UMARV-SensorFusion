# Five-state VN-300 / encoder square-root UKF (C11)

State order is `[x, y, heading, angular_velocity, velocity]`. There are 11 sigma
points. VN-300 provides x/y/heading/angular velocity; the external two-wheel
dead-wheel encoder system provides signed forward velocity. No encoder yaw-rate
measurement or raw IMU/GNSS data is used.

This version starts from the attached VN/encoder package. Both packet types now
run the same motion prediction before correcting the observed quantities.
Velocity is a persistent uncertain state, not a per-packet control input.

## Build and run

Inside `kalman_package_v3`:

```sh
make
make test
./kalman_filter < example_packets.txt > filtered_packets.txt
```

Requires a C11 compiler and libm. Tests require Python 3, standard library only.
Use WSL or a GCC/Make environment such as MSYS2 on Windows. Without Make:

```sh
gcc -std=c11 -O2 -Wall -Wextra -Werror kalman_filter.c kalman_interface.c kalman_internal.c kalman_math.c -lm -o kalman_filter
```

## Input and output

The VN/E input format remains:

```text
VN timestamp x y theta omega
E timestamp v
```

Units: seconds, meters, radians, rad/s, m/s. Position is local ENU (x east, y
north); heading is zero facing east and positive counterclockwise. Velocity is
signed forward speed. All fields must refer to the same vehicle reference point
and coordinate conventions. Body gyro z approximates heading rate for planar,
properly aligned vehicle motion. Apply mounting/lever-arm transforms upstream.

These text packets adapt the simulator's ROS outputs: VN contains the fused
INS pose and angular.z, and E contains encoder linear.x. Do not also fuse
copies of the same VN quantities from its other topics. If using NavSatFix,
convert latitude/longitude to ENU before supplying x/y.

Terminal output is deliberately unchanged, so the existing four-value MATLAB
analyzer continues to parse it:

```text
F timestamp x y theta omega status
```

Velocity is stored in `filter.state[SRUKF_V]` and returned by
`srukf_get_estimate()` as `SrukfEstimate.velocity`. Its variance and correlations
are in the returned 5x5 covariance. It is not added to F packets. The MATLAB
analyzer's speed panel therefore continues to show raw encoder input only.

Statuses are `initialized`, `vn_updated`, `encoder_updated`,
`measurement_rejected`, `invalid_packet`, `stale_packet`, or `update_failed`.
Before initialization, output is `WAIT reason`. Blank lines and full-line `#`
comments are accepted; packet tags are case-insensitive. Malformed/nonfinite
numbers, extra fields, inline comments and old G/O packets are rejected.
Diagnostics go to stderr. EOF stops the program.

## Identical prediction on every packet

`dt = packet.timestamp - filter.timestamp`. For every sigma point:

```text
mid_heading = heading + omega * dt / 2
x_new       = x + v * cos(mid_heading) * dt
y_new       = y + v * sin(mid_heading) * dt
heading_new = wrap(heading + omega * dt)
omega_new   = omega
v_new       = v
```

Both E and VN use the previously estimated v/omega. An arriving encoder speed
is used in the subsequent correction, never substituted into the prediction
that precedes its own measurement update. The unscented weighted mean can
slightly differ from evaluating the equations only at the mean state, because
heading/rate uncertainty affects mean displacement.

Process uncertainty grows on either kind of nonzero-time prediction. The
linear acceleration spectral density gives the local forward-displacement /
velocity covariance block:

```text
q_linear * [dt^3/3, dt^2/2;
            dt^2/2, dt]
```

Displacement noise is projected using the mean midpoint heading. Angular
acceleration gives the same form for heading/omega. Encoder measurement variance
is used only in its correction; it is not added again as process noise.
Within-step coupling of newly introduced angular noise into position and lateral
slip are approximated/omitted. The model has one midpoint step per event; it is
intended for frequent sensor samples, not long unsupported constant-motion gaps.

## Sensor-specific correction

Observations are projections of the state, not padded with predictions as fake
measurements:

- VN: `h(state) = [x, y, heading, omega]`, with full 4x4 R.
- E: `h(state) = [v]`, with scalar positive variance.

The default **literally implements the requested retain-unmeasured-prediction
behavior**:

```c
config.correct_unmeasured_states = false;
```

After VN, v retains its predicted mean. After E, x/y/heading/omega retain their
predicted means. Their corresponding marginal covariance blocks also stay at
the predicted values, while cross-covariances are updated consistently. VN's
four measured quantities may correct each other through their correlations.

This is a constrained Kalman correction. The usual joint Kalman update also
permits an unmeasured state to change through covariance with measured states.
For that conventional behavior, set `correct_unmeasured_states = true` before
initialization. Both policies use the same prediction and observation models.
The default does not attempt to estimate encoder scale bias using VN residuals.

For the constrained policy, unmeasured rows of the gain are zeroed. A simple
`P_new = P - K*Pzz*K'` downdate is not valid for that modified gain. This package
therefore uses the general Joseph covariance:

```text
P_new = (I-KH) P (I-KH)' + K R K'
```

The implementation QR-factors `[(I-KH)S, K*Lr]` directly to produce the new square
root. It does not form P or invert a covariance in the correction. This works
for both gain policies. Heading residuals and state headings are angle-aware.

Already-filtered VN outputs still have uncertainty; the correction blends them
with the motion estimate. This is external sensor fusion, not a recreation of
the VN-300's internal Kalman filter.

## Initialization, timing, and failure handling

The CLI buffers the latest packet of each type until a VN/E pair is at most
0.05 seconds apart. It initializes once using VN's four quantities plus encoder
v and their covariance blocks. It treats the pair as simultaneous at the later
time, an explicit bootstrap approximation. Prefer identical starting timestamps;
reduce `INITIAL_SYNC_TOLERANCE` for stricter synchronization. Neither startup
sample is corrected a second time. Library callers may instead supply a complete
synchronized state and 5x5 positive definite covariance to `srukf_init`.

Subsequent inputs must be chronological. **VN arrivals now propagate motion**,
so a VN sample between E samples no longer loses an integration interval. The
first packet at a timestamp advances the state; any second packet at the same
time has dt=0 and still performs its sensor correction. Either E/VN ordering is
supported at equal timestamps. Keep a deterministic ordering; gated nonlinear
updates can in general depend on update order. Replayed duplicates of a sensor
measurement are not detected and would count as extra information.

Both sensors have innovation gates. A rejected observation retains the full
successful prediction and its timestamp. Malformed data, invalid covariance,
stale timestamps and numerical failures leave the filter unchanged. Public
processing is transactional; no partial correction is committed. After errors,
the CLI prints the last valid estimate with its actual timestamp.

## Simulator defaults and tuning

Selected noise parameters match the inspected
[UMIGV sensor simulator](https://github.com/umigv/maverick/tree/4294faa4d08158548515377130c5ffb0ab6c252a/src/simulation/sensor_simulator)
commit `4294faa4d08158548515377130c5ffb0ab6c252a`:

| Measurement | CLI variance |
| --- | --- |
| VN x, y, each | `0.05^2 + 0.5^2 = 0.2525 m^2` |
| VN heading | `(0.5*pi/180)^2 + (0.3*pi/180)^2` rad^2 |
| VN angular velocity | `0.005^2 + 0.002^2 = 0.000029 (rad/s)^2` |
| Encoder velocity | `0.01^2 + (0.02 * measured_v)^2 (m/s)^2` |

VN uses additive white noise plus OU drift; encoder uses white noise plus an
OU multiplicative scale error. The CLI uses diagonal VN R; the API accepts full
correlated R. Sensor covariances must be positive definite, including strictly
positive encoder variance. Process noise may be rank deficient.

Configuration defaults:

- alpha=0.5, beta=2, kappa=0.
- Linear acceleration noise density=0.25 m^2/s^3.
- Angular acceleration noise density=0.01 rad^2/s^3.
- VN gate=13.28 (4 DOF), encoder gate=6.63 (1 DOF), approximately 99% thresholds.
  A gate <=0 disables that sensor's rejection gate.
- `correct_unmeasured_states=false` to preserve unmeasured predicted means.

The acceleration-noise values are retained starting settings, not hardware
calibrations. Marginal drift variance is not a full model of correlated errors
across time. Neither five-state policy explicitly estimates scale/bias drift or
tracks correlation with the VN-300's internal estimator. Accuracy improvements
must be measured, not assumed from the model change.

## Changed files and integration

- `kalman_interface.h/.c`: state dimension/order, persistent speed in estimates,
  encoder gate, correction policy, common prediction and sensor processing.
- `kalman_internal.h/.c`: stored-speed midpoint model, acceleration process
  noise, 4D/1D observations, gain masking and square-root Joseph correction.
- `kalman_filter.c`: VN/E startup pairing, scalar encoder correction and status.
- README, example and tests updated. Matrix utilities, workspace header and
  Makefile are unchanged; array sizes follow the dimension macros.

API changes from the four-state version require recompiling all callers.
`srukf_predict_to(filter,timestamp,workspace)` and `srukf_predict_step(filter,dt,
workspace)` no longer take a speed input. `srukf_motion_model` and
`srukf_process_noise_factor` likewise use stored state/configuration.
`SrukfVnMeasurement.covariance` remains 4x4 even though the state covariance is
now 5x5. `srukf_apply_correction` takes the sensor and measurement-noise factor,
not an innovation factor. For embedding, compile the three library C files and
exclude the CLI file. Use one workspace per concurrent operation.
Do not use `-ffast-math`; finite-value checks are required.

## Validation

`make test` checks midpoint motion, signed/zero speed, process covariance,
sigma-point reconstruction, both correction policies against independent dense
Kalman/Joseph calculations with correlated priors and correlated VN R, preservation
of unmeasured means/marginals, angle wrapping, prediction on both sensors,
interleaved timestamps, either equal-time order, startup pairing, gating and
rollback, strict CLI parsing, and four 60-second synthetic streams at VN rates
of 400, 100, and 5 Hz with a 400 Hz encoder stream.

AddressSanitizer/UndefinedBehaviorSanitizer command:

```sh
ASAN_OPTIONS=detect_leaks=0 make -B test CFLAGS='-O1 -g -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all'
```

Leak detection is disabled because this environment's process tracing prevents
it from running; the filter uses no dynamic allocation. Synthetic tests verify
implementation behavior and numerical stability, not real-vehicle accuracy.
