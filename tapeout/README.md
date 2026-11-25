# Tapeout RTL snapshot

A copy of the chip's RTL, tests and docs as of 25 November 2026, taken from the working folder of
[umigv/umarv-sensorfusion-tapeout](https://github.com/umigv/umarv-sensorfusion-tapeout) (the
Tiny Tapeout project). It is here so the design can be cloned without that repository's build
workflows. The tapeout repository remains the place the chip is built from; this copy will go
stale.

| Folder | Contents |
|---|---|
| `src/` | the chip: `tt_um_umigv_sensorfusion.sv` (top level), `timestamper.sv`, `spi_regs.sv`, `lidar_top.sv` and its sub-blocks, `mii_recv.sv`, `mii_send.sv`, `async_fifo.sv`, the Kalman filter: `ukf_top.sv`, `ukf_tables.sv`, `fp_serial.sv`, and `project.sdc` (timing constraints for the chip build) |
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
| `src/ukf_top.sv` | the machine: 62 words of latch storage plus 8 for constants the host can replace, a sequencer, and the ports the top level expects. Its header lists what it assumes about the sensors |
| `src/ukf_tables.sv` | the 544 steps and 40 constants. **Generated** by `test/ukf/ukf_prog.py`; do not edit by hand |
| `src/fp_serial.sv` | the one arithmetic unit: add, subtract, multiply, divide, square root, integer-to-float, one bit per clock |
| `test/ukf/ukf_prog.py` | the filter's equations as a step program, an exact model of the machine, and the table writer |
| `test/ukf/run_vcs.sh` | arithmetic unit against numpy (45,723 vectors), then the filter against the exact model, every storage word after every beat, including a run where the host replaces constants part-way |
| `test/ukf/mc.py` | accuracy against the simulation's filter over the whole simulated drive (needs `sim/rate_study`, which is not in this repository) |
| `test/system_ukf/run_vcs.sh` | whole chip with the filter in place, through the pins, then a bit-for-bit replay through the exact model |

## What changed since the 24 November snapshot

The filter computes the same thing: for the same inputs and the built-in constants its results
are bit for bit those of the earlier snapshot, except that a zero turn rate can come out as -0.

- **Constants the host can replace.** New SPI registers: `6C` holds the address of a filter word
  and `70`-`73` are that word. Any filter word can be read this way (storage 0-61, constants
  64-111, sensor words 112-126). Words 96 to 103 can also be written, while the filter is
  switched off: measurement noise for position, heading and turn rate, process noise for
  position, turn rate and heading, radians per encoder count, and the starting wheel radius.
  `test/ukf/ukf_prog.py` lists all sixteen candidates in order (`TUNABLE`); `NCFG` in
  `ukf_top.sv` sets how many are writable (8). Each keeps its built-in value until written and
  returns to it at reset.
- **Host-written state.** Registers `24`-`3B` now live in the filter's first six storage words
  instead of in the SPI block, so they are taken only while the filter is switched off
  (control bit 7 = 0) and idle.
- **Latch memories.** One enable pulse and an AND gate per word (two per word in the filter's
  storage) instead of a flip-flop per word. Writes must still be 3 clocks apart.
- **Two-clock paths.** Three places wait a clock or more before using a value, and
  `src/project.sdc` tells the chip's timing analysis so: the filter after its program counter
  changes (`word_ok` in `ukf_top.sv`), the SPI block fetching a byte to send (`fetch` in
  `spi_regs.sv`), and the lidar point buffer's read. In simulation each value is forced unknown
  during the wait, so logic that did not wait would fail the tests. A filter beat now takes up
  to 89,403 clocks (72 % of 2.5 ms).

## State

- Everything here passes RTL simulation: all seven test folders, including the filter inside
  the whole chip and the replay against the exact model.
- The chip **with** the filter and 8 replaceable constants has passed the full Tiny Tapeout
  build, precheck and gate-level test on trial branches of the tapeout repository (cells about
  470,000 um^2 of the 8x4 block). `src/config.json` holds the settings that needed: `AREA 3`
  synthesis, routability-driven placement off, a smaller hold margin before routing with a
  second hold repair after global routing, design repair after global routing, and long wires
  buffered.
- Timing is still being closed. With the two-clock paths the slow corner (100 C, 1.60 V) is
  within about +1 to -4 ns depending on placement, and one build failed the hold check by
  0.02 ns, so the hold margin is being raised. Treat `src/config.json` as work in progress.
- The gate-level smoke test (`test/test.py`) does not exercise the filter.
- `test/system` (the older simulation-only stand-in for the filter) was removed: the top level's
  filter ports have changed and `test/system_ukf` covers the same ground with the real filter.

`HANDOFF_FPGA.md` is the brief for porting this to a Genesys 2 board.
