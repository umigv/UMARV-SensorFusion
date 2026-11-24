# UKF top level

[`UKF.sv`](UKF.sv) connects the timestamper to the prediction and correction engines. All external ports are scalars or one-dimensional packed vectors. The engines share the 50 MHz clock. The timestamper supplies the fixed-rate beat and the measurement-ready pulse.

## Inputs

| Port | Width | Purpose |
|---|---:|---|
| `clk`, `rst_n` | 1 each | 50 MHz clock and active-low reset. The top level inverts reset for the engines. |
| `ukf_beat` | 1 | One clock at the timestamper beat boundary. |
| `ukf_pulse` | 1 | One clock when the measurement snapshot is ready. |
| `ukf_beat_ts` | 32 | Beat timestamp in microseconds. |
| `ukf_dw_valid` | 1 | Enables prediction on the measurement-ready pulse. |
| `ukf_dw_delta` | 28 signed | Wheel encoder delta in counts. |
| `ukf_vn_valid` | 1 | Enables correction on the measurement-ready pulse. |
| `ukf_vn_yaw_f32`, `ukf_vn_rate_f32` | 32 each | Raw IEEE-754 yaw in degrees and turn rate in rad/s. |
| `ukf_vn_lat_f64`, `ukf_vn_lon_f64` | 64 each | Raw IEEE-754 latitude and longitude in degrees. |

The UKF reads the timestamper’s held payload directly and uses `ukf_pulse` to qualify each sensor’s valid flag.

## Deferred XY integration

The standalone coordinate converter accepts FP32 latitude and longitude in degrees and returns east and north offsets in metres. It reads the externally initialized origin and constants through a single memory port with combinational reads and rising-edge writes. Reserve addresses 21–29 for the converter while `busy` is high.

| Address | FP32 value |
|---:|---|
| 70 | Origin longitude in degrees |
| 71 | Origin latitude in degrees |
| 96 | 1.0 |
| 107 | Radians per degree |
| 108 | Earth radius in metres |
| 109–114 by default | Cosine Taylor coefficients from −1/2! through +1/12!, in increasing degree order |

Set `COS_FIRST` to the first cosine coefficient address if the memory image places sine coefficients first. Submit a one-clock `start` while `busy` is low. Both outputs update together on `xy_valid`. Use nearby coordinates on the same continuous longitude branch.

## Internal wiring and outputs

The packed types in [`sys_defs.sv`](sys_defs.sv) carry measurements and engine results. The latest valid prediction or correction is retained and feeds back into prediction.

Connect `ukf_pred_done` to the timestamper side. It pulses for one clock when the prediction FSM reaches its completion state.

On each `ukf_beat`, `ukf_valid` pulses for one clock and presents the latest available estimate on `ukf_x`, `ukf_y`, `ukf_theta`, `ukf_omega`, and `ukf_wheelB` (32 bits each). The values hold between beats. A beat with no newer estimate repeats the previous estimate, initially zero after reset. The prediction and correction datapaths are still incomplete, so the current engines do not produce valid estimates.
