# UKF top level

[`UKF.sv`](UKF.sv) connects the timestamper to the prediction and correction engines. All external ports are scalars or one-dimensional packed vectors. The engines share the 50 MHz clock. The timestamper owns the 400 Hz beat and spaces the starts so prediction and correction run serially.

## Inputs and capture

| Port | Width | Purpose |
|---|---:|---|
| `clk`, `rst_n` | 1 each | 50 MHz clock and active-low reset. The top level inverts reset for the engines. |
| `ukf_beat_ts` | 32 | Beat timestamp in microseconds. Captured on either valid strobe. |
| `ukf_dw_valid` | 1 | One-clock strobe that opens the wheel-data latch and requests prediction. |
| `ukf_dw_delta` | 28 signed | Wheel encoder delta in counts. |
| `ukf_vn_valid` | 1 | One-clock strobe that opens the VN-data latches and requests correction. |
| `ukf_vn_yaw_f32`, `ukf_vn_rate_f32` | 32 each | Raw IEEE-754 yaw in degrees and turn rate in rad/s. |
| `ukf_vn_lat_f64`, `ukf_vn_lon_f64` | 64 each | Raw IEEE-754 latitude and longitude in degrees. |

The top level passes each strobe directly to its engine. Each engine checks its strobe combinationally while idle and enters its first processing state on the rising clock edge. Its state register carries the operation forward after the strobe falls. The resetless payload latches hold the measurements after their strobe falls.

The timestamper must present each strobe while its engine is idle, with the payload settled through the latch before the sampling edge. It must wait until prediction completes before asserting the VN strobe. The current [`timestamper.sv`](timestamper.sv) still uses the older shared-pulse scheme and needs to emit staggered strobes for this interface.

## Internal wiring and outputs

The top level currently uses the packed types in [`sys_defs.sv`](sys_defs.sv). `meas_regfile_t` carries the inputs to both engines, `PRED_PACKET` carries the prediction result to correction, and `UKF_OUT_PACKET` carries the correction result back to prediction and out through the flat ports.

The outputs are `ukf_valid` (1 bit) and `ukf_x`, `ukf_y`, `ukf_theta`, `ukf_omega`, and `ukf_wheelB` (32 bits each). They currently come directly from the correction engine. Publishing prediction on a beat without correction would require additional result routing.

The planned interface cleanup replaces the internal packet types with explicit signals while keeping the external ports flat.
