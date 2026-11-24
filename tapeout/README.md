# Tapeout RTL snapshot

A copy of the chip's RTL, tests and docs as of 23 November 2026, taken from the working folder of
[umigv/umarv-sensorfusion-tapeout](https://github.com/umigv/umarv-sensorfusion-tapeout) (the
Tiny Tapeout project). It is here so the design can be cloned without that repository's build
workflows. The tapeout repository remains the place the chip is built from; this copy will go
stale.

| Folder | Contents |
|---|---|
| `src/` | the chip: `tt_um_umigv_sensorfusion.sv` (top level), `timestamper.sv`, `spi_regs.sv`, `lidar_top.sv` and its sub-blocks, `mii_recv.sv`, `mii_send.sv`, `async_fifo.sv` |
| `test/` | one folder per test, each with a `run_vcs.sh` (Synopsys VCS); `test.py` and `tb.v` are the Tiny Tapeout cocotb smoke test |
| `docs/` | `lidar_output_format.md` (outgoing packet format, version 3), `timestamper.md` |
| `info.yaml` | Tiny Tapeout project description and pin list |

Start with `../docs/system_architecture.md` for the system picture, then the header comments
of `src/tt_um_umigv_sensorfusion.sv` (pinout) and `src/spi_regs.sv` (register map).

State: everything here passes RTL simulation. The timestamper alone has passed a full Tiny
Tapeout build; the rest has not yet. The Kalman filter is not written: `test/system/ukf_standin.sv`
is a simulation-only stand-in with the proposed port list. `src/fp32_mac.sv` is the filter
team's unit, copied for reference and not used by anything here.

`HANDOFF_FPGA.md` is the brief for porting this to a Genesys 2 board.
