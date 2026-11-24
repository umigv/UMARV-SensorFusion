## How it works

This square-root unscented Kalman filter fuses wheel-encoder increments and VN300 position, yaw, and angular-rate measurements. It estimates east and north position, heading, angular rate, and wheel-velocity bias. Prediction, coordinate conversion, and correction time-share one FP32 MAC and one DIVSQRT. The interface accepts FP64 latitude and longitude and a runtime correlated measurement-noise factor.

The example configuration uses an origin of zero degrees latitude and longitude, a 2.5 ms sample period, and a wheel scale of 0.125 m/s per count. Initial state, initial covariance factor, process-noise factor, wheel scale, and origin are build-time parameters in the wrapper.

## How to test

Drive the interface synchronously to the project clock while `ena` is high. Hold each command and data byte stable across a rising edge. Each rising edge with STROBE high executes a command. The bidirectional pins are inputs.

| Command on `uio_in[2:0]` | Action |
|---|---|
| 1 | Append the byte on `ui_in` to the frame. Send most-significant byte first. |
| 2 | Submit a four-byte signed wheel count. The low 28 bits are used. Submit while idle. |
| 3 | Submit a 64-byte VN300 frame. It can be submitted while prediction is busy. |
| 4 | Discard a partial frame and clear the sticky failure, overflow and protocol flags. |

An accepted or rejected submit clears the staged frame count. To run at 400 Hz, submit a wheel frame every 125,000 clocks and the matching VN frame 10,000 clocks later at 50 MHz. Stage all required bytes before each submission edge.

The VN frame contains, in transmission order, eight bytes of FP64 latitude in degrees, eight bytes of FP64 longitude in degrees, four bytes of FP32 yaw in degrees clockwise from north, four bytes of FP32 angular rate in rad/s clockwise, then ten FP32 words. The ten words are the upper-triangular measurement factor, sent in *reverse* upper-triangular row order. Thus the last word in the frame is the first diagonal entry.

With `uio_in[4]` low, `uo_out` presents status.

| Bit | Meaning |
|---|---|
| 0 | Busy |
| 1 | Corrected result ready |
| 2 | Sticky failure |
| 3 | Reinitialization required |
| 4 | Sticky VN queue overflow |
| 5 | Sticky malformed or rejected command |

With `uio_in[4]` high, place a byte index 0–79 on `ui_in` and read `uo_out` while result-ready is set and busy is clear. Each FP32 word is read least-significant byte first. Words 0–4 are east, north, heading, angular rate, and velocity bias. Words 5–19 are the upper-triangular covariance factor in row order. Read the complete result before starting another update. A reinitialization-required fault requires asserting `rst_n` low to reset the filter.

The testbench submits two updates through the pins, checks the 400 Hz cycle budget, and compares the state and factor to independent reference values. It also checks malformed-frame reporting. Gate mode uses a slower test clock so the template's functional unit-delay cell models can settle. The deadline assertions count clock cycles.
