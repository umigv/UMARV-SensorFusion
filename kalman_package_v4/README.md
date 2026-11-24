# VN-300 / encoder square-root UKF (C11)

State order: `[x, y, heading, angular_velocity]`. There is no velocity state,
no cached velocity, and no published velocity. VN-300 supplies all four state
observations. External two-wheel dead-wheel encoders supply signed forward
speed as an uncertain motion input. Encoder angular velocity is not used.

The VN measurements represent the VN-300's already-filtered navigation outputs.
This filter combines them with encoder-based motion; it does not reconstruct
VectorNav's internal INS filter or consume raw accelerometer/gyro/GNSS data.
A VN correction still statistically blends the current estimate and the VN
observation; already-filtered does not mean error-free.

## Build and run

Inside the extracted `kalman_package_v3` directory:

```sh
make
./kalman_filter < example_packets.txt > filtered_packets.txt
make test
```

Requires a C11 compiler and libm; tests additionally require Python 3, with no
third-party Python dependencies. On Windows, use WSL or a GCC/Make environment
such as MSYS2. Without Make:

```sh
gcc -std=c11 -O2 -Wall -Wextra -Werror kalman_filter.c kalman_interface.c kalman_internal.c kalman_math.c -lm -o kalman_filter
```

## Packets and coordinate conventions

Exactly one packet per line:

```text
VN timestamp x y theta omega
E timestamp v
```

- Timestamps: seconds, preferably relative to session start.
- Position: meters in a common local ENU frame, x east and y north.
- Heading: radians, zero facing east, positive counterclockwise.
- Angular velocity: rad/s, positive counterclockwise for the planar model.
- Encoder velocity: signed forward m/s, not distance, wheel ticks, or speed magnitude.

These text packets are this application's adapter format, not the repository's
native ROS message wire format. Form one VN packet using co-timestamped fused
position, yaw, and angular.z (for example the simulated INS `odom` message).
Use only encoder `enc_vel.twist.twist.linear.x` for E. Do not also fuse duplicate
VN position/yaw/rate from its other topics. Transform mounting offsets and
reference-point differences into the same vehicle frame before supplying data.
If using the simulated NavSatFix topic, convert latitude/longitude to local ENU;
do not feed degrees into x/y. Body gyro z approximates heading rate only for
planar, suitably aligned vehicle motion.

Blank lines and full-line `#` comments are allowed. Packet tags are case
insensitive. Nonfinite values, range errors, incomplete numeric tokens, extra
fields, trailing junk, inline comments, unknown tags and overlong lines are
rejected. Old G/O packets are not accepted.

Output after each packet:

```text
F timestamp x y theta omega status
```

`F` distinguishes filtered output from `E` encoder input. Status is
`initialized`, `encoder_predicted`, `vn_updated`, `measurement_rejected`,
`invalid_packet`, `stale_packet`, or `update_failed`. Before initialization,
the CLI prints `WAIT reason`. Diagnostics go to stderr. Failed operations
report the last valid estimate with its own timestamp. EOF stops processing.
The earlier MATLAB G/O generator and old output parser need
adapting to these new packet layouts before use with this version.

## Encoder prediction

For `dt = encoder.timestamp - filter.timestamp`, every state sigma point uses
exactly this midpoint model:

```text
mid_heading = heading + omega * dt / 2
x_new       = x + v * cos(mid_heading) * dt
y_new       = y + v * sin(mid_heading) * dt
heading_new = wrap(heading + omega * dt)
omega_new   = omega
```

The four-state unscented transform has nine sigma points. Its weighted mean can
differ slightly from applying the equations only to the mean state, because
heading/rate uncertainty affects the expected displacement. Heading is wrapped
to [-pi, pi).

Speed is used only in the current call. An E packet performs prediction only:
there is no independent observation of x/y/heading/omega to correct against.
Treating the encoder-derived position as a new independent position measurement
would reuse the prior state and count its information twice. The next VN packet
provides the observation against which this propagated estimate is weighted.

Encoder uncertainty is included as process uncertainty using
`G = [dt*cos(mid_heading), dt*sin(mid_heading), 0, 0]^T` and
`Q_encoder = G * velocity_variance * G^T`. This is a first-order input-noise
approximation at the mean midpoint heading, independent of state uncertainty.
The state itself is propagated through the full nonlinear sigma-point model.

The retained acceleration-noise settings add:

- Forward displacement variance `q_linear * dt^3 / 3`, along midpoint heading.
- Heading/rate covariance
  `q_angular * [[dt^3/3, dt^2/2], [dt^2/2, dt]]`.

Within-step effects of new angular process noise on x/y and lateral slip are
not explicitly modeled. The full factor may be rank deficient; QR combines
it with propagated state uncertainty. Zero encoder variance is permitted.
No velocity uncertainty state persists between packets.

There is one midpoint step per E packet. The old automatic subdivision and
`max_prediction_dt` setting were removed: subdividing and injecting independent
speed error at every substep would incorrectly reduce the uncertainty of the
same encoder sample. Supply actual encoder samples over long intervals rather
than treating one late speed sample as the entire missing motion history.

## VN correction, initialization, and timing

The CLI initializes directly from the first valid VN packet and its covariance.
It needs no encoder startup pair and does not fuse that first VN twice. Earlier
encoder samples produce `WAIT need_initial_vn` and are not retained or replayed.
Library callers initialize explicitly with `srukf_init`.

For VN, the prediction is exactly the current state and covariance, with no
motion or added process noise. The observation model is identity. The joint
four-dimensional correction uses full R, angle-aware residuals, innovation
gating, triangular solves, and square-root covariance downdates. All four
measurements are accepted or rejected together.

Packets must be chronological. **After initialization, send E before VN at a
shared timestamp.** E integrates up to that time, then VN corrects it. At startup,
a VN packet can initialize directly; an E at that same timestamp has zero dt.

`filter.timestamp` is the last committed event time, including successful or
gated-out VN packets. Each E integrates only since that time, not since the
last E. Equal-time E packets are no-ops. Do not replay duplicate VN packets:
there is no sequence-ID deduplication, and they would count as extra observations.
A malformed packet, invalid covariance, stale timestamp or numerical failure
leaves the filter unchanged. A gated VN advances its timestamp but leaves state
and covariance unchanged.

**Identity prediction at VN is the requested approximation, not general
asynchronous propagation.** If VN arrives between E samples, the motion from
the last E to VN is not explicitly integrated, and the following E starts at
VN time. To avoid losing that interval, arrange an encoder prediction at each
VN timestamp (resampling encoder inputs upstream if needed). An alternative
would be holding/interpolating speed and predicting at VN arrival, which this
version deliberately does not implement. A VN-only stream also has no process
uncertainty growth, so repeated VN updates can make covariance too small while
a moving state lags. This design expects the faster encoder stream.

## Simulator-based default covariance

Reference: [UMIGV Maverick sensor simulator](https://github.com/umigv/maverick/tree/4294faa4d08158548515377130c5ffb0ab6c252a/src/simulation/sensor_simulator),
inspected commit `4294faa4d08158548515377130c5ffb0ab6c252a`.
The repository approximates fused VN outputs using additive white noise plus
Ornstein-Uhlenbeck (OU) drift; encoder errors use white noise plus OU scale
error. We use its VN angular-rate model, correcting the previous package's
encoder angular-rate assumption for this vehicle.

| Quantity | CLI variance |
| --- | --- |
| VN x and y, each | `0.05^2 + 0.5^2 = 0.2525 m^2` |
| VN heading | `(0.5*pi/180)^2 + (0.3*pi/180)^2` rad^2 |
| VN angular velocity | `0.005^2 + 0.002^2 = 0.000029 (rad/s)^2` |
| Encoder speed | `0.01^2 + (0.02 * measured_v)^2 (m/s)^2` |

CLI VN R is diagonal; library callers can supply the full symmetric positive
definite 4x4 covariance. Encoder variance is recalculated per packet.
Initialization uses VN R as its initial state covariance.

Config defaults retain alpha=0.5, beta=2, kappa=0,
linear acceleration noise density=0.25 m^2/s^3, and angular acceleration noise
density=0.01 rad^2/s^3. These process settings are starting values from the prior
package, not parameters measured from the simulator or hardware.
The new VN gate is 13.28 (approximately the 99% chi-square threshold for four
components); set it <=0 to disable it. There is no encoder innovation gate,
since encoder speed has no measurement prediction in the four-state model.

The reported drift terms describe marginal variance, not independent new noise
at every sample. This four-state filter does not estimate encoder scale error,
VN biases, or their time correlations. Repeated INS estimates can also share
errors with the current filter state; full per-packet R alone does not remove
that dependence. Thus matching these default variances does not guarantee
consistent covariance or better accuracy than VN alone. Compare at identical
timestamps and tune using the intended sample rates and sensor logs.

## API changes and source layout

This is a breaking API change; rebuild all callers.

- `SrukfStateIndex`: x=0, y=1, heading=2, omega=3. N=4, sigma count=9,
  max measurement dimension=4. Velocity removed from `SrukfEstimate`.
- `SrukfVnMeasurement` and `srukf_process_vn` replace GPS/odometry updates.
- `SrukfEncoderMeasurement` and `srukf_process_encoder` supply speed/variance
  for prediction, not a velocity observation update.
- `srukf_predict_to` now takes timestamp, velocity and velocity variance;
  internal `srukf_predict_step`, motion and process-noise signatures changed.
- `vn_gate_threshold` replaces the old two sensor gates. `max_prediction_dt`
  and `SRUKF_MAX_PREDICTION_STEPS` removed.
- `kalman_filter.c`: new parser, covariance policy, startup and four-state output.
- `kalman_interface.h/.c`, `kalman_internal.h/.c`: state/model/update changes.
- `kalman_math.h/.c`, `kalman_workspace.h` and Makefile are unchanged; their
  arrays already follow the dimension macros.
- Added `tests/test_core.c`, `tests/test_cli.py` and `example_packets.txt`.

For embedding, compile the three library C files, excluding `kalman_filter.c`.
Include `kalman_interface.h` and `kalman_workspace.h`. Use one workspace per
concurrent operation. Matrix strides count doubles, not bytes. Numerical
failures are reported rather than hidden with covariance jitter. Do not compile
with `-ffast-math`; finite-value checks are required.

## Validation

`make test` covers midpoint motion, reverse/zero speed, no cached velocity,
sigma-point covariance, encoder/process-noise propagation, a correlated 4x4 VN
update against an independent dense Kalman calculation, angle wrapping,
identity prediction, timing, gating, error rollback, initialization, strict
parsing, and three 60-second synthetic streams using the reference error laws.

The C and CLI suites also pass AddressSanitizer and UndefinedBehaviorSanitizer:

```sh
ASAN_OPTIONS=detect_leaks=0 make -B test CFLAGS='-O1 -g -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all'
```

Leak detection was disabled because it cannot run under this execution
environment's process tracing; this library does not allocate dynamic memory.

For the included synthetic tests (400 Hz encoder), position RMSE at VN epochs:

| Seed | VN rate | Filter | VN alone |
| --- | --- | --- | --- |
| 42 | 100 Hz | 0.2789 m | 0.3618 m |
| 7 | 100 Hz | 0.2158 m | 0.4406 m |
| 42 | 5 Hz | 0.3791 m | 0.3619 m |

All 84,303 packet events completed without numerical/update failures. One VN
packet was gated across those runs. These are implementation checks and a few
noise realizations, not a hardware accuracy claim. The 5 Hz case illustrates
that changing the state representation alone does not ensure improvement.
