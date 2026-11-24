# Brief: port the chip to a Genesys 2 for testing with real sensors

Read this fully before acting.

## Project

A Tiny Tapeout Sky130 sensor-fusion chip for a ground robot (shuttle TTSKY26d, closes
30 November 2026, 8x4 tiles, 24 I/O, 50 MHz). The job: port the design to a Digilent Genesys 2
(Kintex-7 xc7k325tffg900-2) so it can be tested with real sensors before tapeout, with good
visibility into what it is doing.

## Where things are (this repository, branch `cheerfulconnor`)

| Path | What |
|---|---|
| `docs/system_architecture.md` | system diagram and block descriptions |
| `tapeout/src/tt_um_umigv_sensorfusion.sv` | chip top level and pinout |
| `tapeout/src/spi_regs.sv` | host SPI protocol and the full register map (header comment) |
| `tapeout/src/lidar_top.sv` | lidar block structure |
| `tapeout/src/timestamper.sv`, `tapeout/docs/timestamper.md` | timestamper |
| `tapeout/docs/lidar_output_format.md` | outgoing packet format (version 3) and a Python receiver |
| `tapeout/test/system/` | whole-chip test and the simulation-only filter stand-in |
| `lidar_pointcloud_fpga/genesys2/` | an earlier working Genesys 2 prototype. Use its `syn/genesys2.xdc`, `syn/Makefile` and `rtl/fpga.sv` (clocking, IDELAY, PHY reset) as the starting point |

The chip itself is built from a separate repository,
`git@github.com:umigv/umarv-sensorfusion-tapeout.git`. `tapeout/` here is a snapshot of it.

## What exists

All of it passes RTL simulation in Synopsys VCS. None of it is proven on hardware.

- **timestamper:** 400 Hz beat, VN-300 SyncIn pulse, 921,600-baud binary message parser with
  CRC, AMT22 encoder SPI master. Passed the full Tiny Tapeout build.
- **lidar_top:** MII receive -> async FIFO -> VLP-16 parser -> point buffer -> x, y, z transform
  with motion correction (`lidar_motion` + `lidar_cordic`) -> frame builder -> async FIFO -> MII
  transmit. Bypass mode forwards frames unchanged.
- **spi_regs:** SPI slave (mode 0, MSB first, byte 0 = `{write, addr[6:0]}`, address
  auto-increments). Holds all settings, event counters and the latest measurements.
- **Top level:** the three blocks wired together. The Kalman filter is **not** written; its
  socket (`ukf_top`) is compiled only with `WITH_UKF` defined. Without it the lidar block's
  state comes from host-written registers `24`-`3B`, strobed by a write to `3C`.
- **Tests:** `tapeout/test/{mii_recv,lidar_motion,lidar_top,spi_regs,chip,system}/run_vcs.sh`.
  If VCS is not on the machine, use whatever simulator is and say which.

## Chip pinout (the FPGA top must present these same signals)

| Pin | Function | Pin | Function |
|---|---|---|---|
| `ui[0]` | VN-300 serial in | `uo[0]` | VN-300 sync |
| `ui[1]` | encoder MISO | `uo[1..3]` | encoder CS_N, SCK, MOSI |
| `ui[2..4]` | host SPI SCK, CS_N, MOSI | `uo[4]` | measurements-ready pulse |
| `ui[5]` | lidar MII RX_DV | `uo[5]` | host SPI MISO |
| `ui[6]`, `ui[7]` | MII TX_CLK, RX_CLK | `uo[7]` | MII TX_EN |
| `uio[3:0]` | lidar MII RXD (in) | `uio[7:4]` | host MII TXD (out) |

## What has to change for the FPGA

1. **Ethernet.** The chip expects two 100 Mbit MII PHYs. The Genesys 2 has one gigabit RGMII
   PHY (RTL8211E). Plan: lidar, board and laptop on a switch; use the PHY's receive half as the
   lidar side and its transmit half as the host side. At 100 Mbit, RGMII carries MII nibbles on
   a 25 MHz clock, so a small RGMII-to-MII shim should let `mii_recv` and `mii_send` stay
   unchanged. The link must be 100 Mbit: use a 100 Mbit switch, or add an MDIO write that limits
   what the PHY advertises. **Verify RGMII 10/100 behaviour against the RTL8211E datasheet; do
   not assume.**
2. **Point buffer.** `lidar_latch_mem.sv` uses Sky130 latch cells under `` `ifdef SYNTHESIS ``
   and behavioural latches otherwise. Neither suits an FPGA. Add an FPGA variant (flip-flops or
   block RAM) with the same ports and the same two-clock write latency, selected by a define.
   Do not change the ASIC path.
3. **Clock.** Make 50 MHz from the board's 200 MHz oscillator.
4. **Keep the ASIC sources untouched** wherever possible. Put FPGA-only files in a new
   directory with its own top, constraints and build script.
5. **Visibility.** LEDs for link, lidar packets, frames sent, beat, sensor-valid bits and sticky
   error flags; ILA probes on the VN-300 parser, the encoder SPI, the MII receive stream and the
   host SPI. The event counters and measurements are already readable over the host SPI.

## Sensor wiring to plan for (confirm voltages from datasheets)

- **Host microcontroller:** an ESP32 (3.3 V) on a Pmod header, 4-wire SPI. Write a small sketch
  that reads ID register `00` (expects `A7`), dumps the counters and measurements, and can write
  settings.
- **VN-300:** TTL serial at 921,600 baud plus SyncIn. Message expected: binary, sync `FA`,
  group `01`, field word `0x1068`, 56 bytes.
- **AMT22:** a 5 V part, SPI mode 0; needs level shifting to 3.3 V.

## What this port exists to check

These are taken from datasheets and have never met hardware:

- the VN-300 message layout and CRC byte order (high byte first is assumed)
- that the VN-300's output is triggered by SyncIn
- the AMT22's check bits and its turns read
- real VLP-16 packets through `mii_recv`
- a real computer accepting the transmitted frames

## Open items on the ASIC side (context only)

- No Tiny Tapeout build containing the lidar block has passed yet. The first attempt ran out
  of room during hold fixing; the point buffer was then moved to latches.
- The two MII clocks have no timing constraints in the ASIC flow.
- Area is the main risk once the filter is added.

## How to work

- Commit or push only when the user asks.
- In the tapeout repository, commits are authored as
  `cheerfulconnor78 <181265863+cheerfulconnor78@users.noreply.github.com>` with no
  `Co-Authored-By` line.
- Files that go to Tiny Tapeout use plain SystemVerilog: no structs, enums, packages, size
  casts, `return` or typed function arguments.
- If data on a part is missing, fetch its datasheet; do not assume.
- Report results plainly, including what was not tested.

## First steps

1. Confirm the Vivado version and that the board is attached.
2. Run whichever simulations the machine supports, to confirm the checkout is sound.
3. Propose the FPGA directory layout and the RGMII shim approach in a few lines, then build
   it. Ask which sensors are physically connected before bring-up.
