# Tapeout RTL snapshot

A copy of the chip's RTL, tests and docs as of 24 November 2026, taken from the working folder of
[umigv/umarv-sensorfusion-tapeout](https://github.com/umigv/umarv-sensorfusion-tapeout) (the
Tiny Tapeout project). It is here so the design can be cloned without that repository's build
workflows. The tapeout repository remains the place the chip is built from; this copy will go
stale.

| Folder | Contents |
|---|---|
| `src/` | the chip: `tt_um_umigv_sensorfusion.sv` (top level), `timestamper.sv`, `spi_regs.sv`, `lidar_top.sv` and its sub-blocks, `mii_recv.sv`, `mii_send.sv`, `async_fifo.sv`, and the Kalman filter: `ukf_top.sv`, `ukf_tables.sv`, `fp_serial.sv` |
| `test/` | one folder per test, each with a `run_vcs.sh` (Synopsys VCS); `test.py` and `tb.v` are the Tiny Tapeout cocotb smoke test |
| `docs/` | `lidar_output_format.md` (outgoing packet format, version 3), `timestamper.md` |
| `info.yaml` | Tiny Tapeout project description and pin list |

Start with `../docs/system_architecture.md` for the system picture, then the header comments
of `src/tt_um_umigv_sensorfusion.sv` (pinout) and `src/spi_regs.sv` (register map).

## The Kalman filter

A five-state square-root unscented filter (x, y, theta, omega, wheel radius) in single precision,
built as a small stored-program machine. **It is compiled in only when `WITH_UKF` is defined**;
without it the chip is the timestamper, SPI registers and lidar block, with the state supplied
by the host.

| File | What |
|---|---|
| `src/ukf_top.sv` | the machine: 64 words of latch storage, a sequencer, and the ports the top level expects. Its header lists what it assumes about the sensors |
| `src/ukf_tables.sv` | the 544 steps and 37 constants. **Generated** by `test/ukf/ukf_prog.py`; do not edit by hand |
| `src/fp_serial.sv` | the one arithmetic unit: add, subtract, multiply, divide, square root, integer-to-float, one bit per clock |
| `test/ukf/ukf_prog.py` | the filter's equations as a step program, an exact model of the machine, and the table writer |
| `test/ukf/run_vcs.sh` | arithmetic unit against numpy (45,723 vectors), then the filter against the exact model, every storage word after every beat |
| `test/ukf/mc.py` | accuracy against the simulation's filter over the whole simulated drive (needs `sim/rate_study`, which is not in this repository) |
| `test/system_ukf/run_vcs.sh` | whole chip with the filter in place, through the pins, then a bit-for-bit replay through the exact model |

## State

- Everything here passes RTL simulation, including the filter inside the whole chip.
- The chip **without** the filter has passed the full Tiny Tapeout build, precheck and gate-level
  test. It needs `"SYNTH_STRATEGY": "AREA 3"` in `src/config.json`, which is set.
- The chip **with** the filter synthesises to 480,000 um^2 of cells (69 % of the 8x4 block) and
  has **not** routed: two builds were still at 10,000 to 18,000 routing violations after three
  hours. Making it fit is open work.
- `test/system/ukf_standin.sv` is the older simulation-only stand-in, kept for its test.
  `src/fp32_mac.sv` is the filter team's unit, copied for reference and not used by anything here.

`HANDOFF_FPGA.md` is the brief for porting this to a Genesys 2 board.
