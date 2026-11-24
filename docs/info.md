## How it works

The 400 Hz timestamper triggers the VN-300, validates its UART packet, reads and validates the AMT22 encoder, and differences successive good encoder positions. It delivers one timestamped snapshot when both sensors answer or the 2 ms collection deadline expires. The SR-UKF alone scales encoder counts, narrows FP64 coordinates, converts coordinates and heading, and performs prediction and correlated correction with one FP32 MAC and one DIVSQRT.

The generated core uses example origin, wheel scale, initial state, and noise tuning. Set production calibration in the core export before using it with a different origin or encoder scale. After reset, the first good encoder position establishes the count baseline. A valid VN sample can still correct the state during that beat.

## Pin operation

Drive a 50 MHz clock and hold `ui[2]` high to collect. `uo[0:3]` retain the VN sync and encoder SPI signals. `uo[4]` pulses when the timestamper releases a snapshot. A wheel-bearing snapshot that arrives while the filter is busy is rejected as a unit and raises the sticky fault flag. A VN-only snapshot can queue behind an active correction. It is rejected during an older wheel-only prediction to prevent cross-beat pairing. The collection window and filter runtime are separate; a fixed sensor-response latency maintains a 125,000-clock release spacing. The integration test uses fixed response latency.

While collection is disabled and both acquisition and the UKF are idle, drive one measurement-noise byte on `uio_in` for each rising clock edge with `ui[4]` high. Write all 40 bytes MSB first. The ten FP32 words are in reverse upper-triangular row order, ending with the first diagonal. An incomplete load inhibits new acquisition. The default factor is restored by reset. The core keeps separate active and pending noise snapshots.

Read the corrected-state stream on `uo[5]` while `uo[6]` is high. The frame sends 168 bits MSB first, with each bit held eight clocks. The first byte is status. Bits 0 through 4 indicate corrected result, failure, reinitialization required, VN overflow, and rejected snapshot respectively. Five FP32 words follow in order of east, north, heading, angular rate and velocity bias. Fault-only frames carry zeros for the state. `uo[7]` latches faults until reset, including a fault arriving during an existing frame. A correction failure requiring reinitialization must be cleared by resetting the filter.

## Simulation and hardening

The pin-level fixture in `test/tb.v` drives valid VN UART and AMT22 SPI responses, programs measurement noise and checks two corrected frames against independent reference values. Use `make -C test` for the Tiny Tapeout Cocotb entry point or use Icarus directly:

```sh
iverilog -g2012 -s tb -o /tmp/srukf-tapeout.vvp src/srukf_core.v src/timestamper.sv src/tt_um_umigv_sensorfusion.sv test/tb.v
vvp /tmp/srukf-tapeout.vvp
```

Regenerate the checked-in core using `python3 -B tools/export_srukf.py /path/to/UMARV-SensorFusion` with the optimized core checkout and Yosys 0.52. The 8×2 footprint is a trial configuration pending placement, routing, timing and precheck.
