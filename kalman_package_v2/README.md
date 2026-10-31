# GPS / odometry square-root UKF (C11)

State order: `[x, y, signed forward velocity, heading, angular velocity]`.
Units: meters, seconds, radians. Heading is counterclockwise from the positive
x axis; GPS and odometry must already be aligned to the same local frame.

## Build and run

Extract the ZIP, open a terminal inside its `kalman_package` folder, then:

```sh
make
./kalman_filter
```

Requires a C11 compiler and the C math library. On Windows use WSL or a
GCC/Make environment such as MSYS2. This is C, not C++.

Without Make:

```sh
gcc -std=c11 -O2 -Wall -Wextra -Werror kalman_filter.c kalman_interface.c kalman_internal.c kalman_math.c -lm -o kalman_filter
```

Run a recorded stream:

```sh
./kalman_filter < example_packets.txt
```

Strict packet format (one packet per line):

```text
G timestamp x y
O timestamp velocity heading angular_velocity
```

GPS requires exactly three numbers after `G`; the unused fifth GPS field is
not accepted, matching the uploaded package's four-field requirement.
Blank lines and full-line `#` comments are allowed. Letters are case insensitive.
NaN, infinity, overflow/underflow, incomplete numbers, trailing junk, extra
fields and overlong lines are rejected. Input numeric parsing uses the C locale.
Do not append inline comments.

Outputs:

```text
WAIT reason
E timestamp x y velocity heading angular_velocity status
```

Diagnostics go to stderr. Each estimate includes its actual timestamp; on a
failed update it is the last valid estimate. EOF stops the application.

## Initialization and timing

The CLI waits for a GPS/odometry pair at most 0.05 seconds apart, then treats
them as simultaneous at the later time. This is an explicit startup
approximation, not interpolation. Prefer identical startup timestamps; reduce
the tolerance for fast motion. Packets must arrive in chronological order;
equal timestamps are allowed. There is no delayed-measurement replay.

A failed update does not advance the CLI timestamp watermark. A gated-out
measurement retains the successful prediction. Public processing functions
leave the filter unchanged on other failures.

`srukf_predict_to` splits intervals using `max_prediction_dt`. One call permits
at most 100,000 nominal steps (plus two for rounding), rejecting larger gaps
with `SRUKF_INVALID_TIME` instead of effectively hanging. At the default 0.02 s
step, the nominal gap limit is 2,000 seconds. Applications needing longer gaps
must explicitly advance in chunks or reinitialize. Prefer time relative to
session start for floating-point timestamp resolution.

## Files and integration

- `kalman_interface.h/.c`: public state, configuration, initialization, sensor
  processing, prediction and estimate functions.
- `kalman_internal.h/.c`: sigma points, motion/noise models and update internals.
- `kalman_math.h/.c`: QR, Cholesky, triangular solves and covariance utilities.
- `kalman_workspace.h`: complete workspace definition. Only actively used
  fields remain; rebuild callers against this header.
- `kalman_filter.c`: standalone terminal application and `main`.

For another application, compile the three library `.c` files, exclude
`kalman_filter.c`, and include `kalman_interface.h` plus `kalman_workspace.h`
when allocating scratch storage. Headers are independently includable.
Use one workspace per concurrent operation. Matrix strides count doubles,
not bytes; callers must supply adequately sized buffers and obey documented
non-overlap requirements. Void internal helpers assume valid arguments.
`srukf_reset` clears all state/configuration; call `srukf_init` again afterward.

## Model and tuning limits

The deterministic model integrates constant-speed, constant-turn arcs using a
stable sinc formula. Process noise uses a frozen-heading approximation per
step; it does not explicitly model lateral slip or within-step coupling of
new angular noise into position. Angular means assume a local concentrated
distribution and reject ambiguous branches. This is not a multimodal heading
estimator.

Defaults and fixed CLI sensor standard deviations are illustrative, not
calibrated: GPS 1.5 m, speed 0.05 m/s, heading 0.035 rad, turn rate 0.02 rad/s.
The API accepts full per-packet covariances, including off-diagonal entries.
Persistent odometry bias and temporal correlation are not explicitly modeled.
No extra IMU input is fused independently of odometry.

Covariances used for initialization and sensor noise must be positive definite.
The process noise factor may be rank deficient. Numerical downdate failures
are returned; the library does not silently add covariance jitter. QR rejects
active-column residual norms below 64*DBL_EPSILON times their original column
norm as numerically rank deficient. Very ill-conditioned inputs may therefore
be rejected. Do not compile with `-ffast-math`: finite-value checks matter.

## Tests

```sh
make test
```

Requires Python 3 for CLI tests; no third-party Python packages are required.
Tests cover QR reconstruction, rank-deficient QR, Cholesky update/downdate,
triangular solves, a correlated GPS update against an independently computed
dense Kalman reference, process-noise covariance, turning motion, angular
wrap, rejection/rollback/reset, input parsing and a 1,100-update trajectory.

An undefined-behavior sanitizer run was also completed successfully:

```sh
make clean
make test CFLAGS='-O1 -g -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all'
```

Synthetic tests establish implementation consistency, not real-vehicle
localization accuracy. Validate tuning with actual sensor logs.
