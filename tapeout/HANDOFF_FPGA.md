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
| `tapeout/src/ukf_top.sv`, `ukf_tables.sv`, `fp_serial.sv` | the Kalman filter (new on 24 November). `ukf_top.sv`'s header lists what it assumes about the sensors |
| `tapeout/test/ukf/`, `tapeout/test/system_ukf/` | the filter's tests, its exact Python model and the generator of `ukf_tables.sv` |
| `tapeout/test/system/` | older whole-chip test with a simulation-only filter stand-in |
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
- **ukf_top (new):** a five-state square-root unscented Kalman filter (x east, y north, theta,
  omega, wheel radius) in single precision. It is a small stored-program machine: 64 words of
  storage, one bit-serial arithmetic unit (`fp_serial`), and a table of 544 steps
  (`ukf_tables.sv`, generated; never edit it by hand). One beat takes up to 85,612 clocks, 68 %
  of a 2.5 ms beat. In simulation it matches its exact Python model bit for bit, and over the
  whole simulated drive it matches the reference filter (0.876 m against 0.873 m position error).
- **Top level:** the blocks wired together. **The filter is compiled in only when `WITH_UKF` is
  defined: define it for the FPGA build.** Control register `02` bit 7 then chooses the state
  the lidar block uses: 1 = the filter's, 0 = host-written registers `24`-`3B`, strobed by a
  write to `3C` (the reset value is 0, so the host must set bit 7 to use the filter).
- **Tests:** `tapeout/test/{mii_recv,lidar_motion,lidar_top,spi_regs,chip,system,ukf,system_ukf}/run_vcs.sh`.
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
3. **Filter storage.** `ukf_store` (at the bottom of `ukf_top.sv`) is built the same way as the
   point buffer: Sky130 latch cells under `` `ifdef SYNTHESIS ``, behavioural latches otherwise.
   It needs the same treatment: an FPGA variant with the same ports and timing (a write is
   registered, lands half a clock later, and the word may be read two clocks after the write;
   reads are combinational; words 0 to 5 are also wired straight out as the state). Flip-flops
   are simplest, since six words must be visible at once. Vivado defines `SYNTHESIS` itself, so
   the selection needs its own define. Do not change the ASIC path.
4. **Clock.** Make 50 MHz from the board's 200 MHz oscillator. The filter's timing budget
   assumes exactly 50 MHz and a 400 Hz beat.
5. **Keep the ASIC sources untouched** wherever possible. Put FPGA-only files in a new
   directory with its own top, constraints and build script.
6. **Visibility.** LEDs for link, lidar packets, frames sent, beat, sensor-valid bits and sticky
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

And for the filter, all of them assumptions written into `ukf_tables.sv` through
`test/ukf/ukf_prog.py` (change them there and regenerate):

- yaw is degrees clockwise from north, and the turn rate is positive clockwise
- the encoder gives 16,384 counts per turn, and the nominal wheel radius is 50 mm
- latitude is between 32 and 64 degrees north and longitude between 64 and 128 degrees west
- the first beat with a good VN-300 message becomes the origin; x is east and y is north from it
- the tuning (process and measurement noise) is the simulation's; it is fixed in the table, and
  the SPI tuning writes at `6C`-`7F` are ignored for now

Useful things to look at on the board: the filter's six output words (they are in every lidar
frame header, and `ukf_top` raises `state_valid` once per beat), its `overruns` counter (a beat
that arrived before the last one finished), and whether the wheel radius it learns is sensible.
One known flaw: the six output words change over about 30 clocks at the end of a beat, so about
one lidar frame in 4,000 can carry a header that mixes two beats.

## Open items on the ASIC side (context only)

- The chip without the filter passes the full Tiny Tapeout build, precheck and gate-level test.
- The chip with the filter does not route yet (480,000 um^2 of cells, 69 % of the block). That
  is being worked on separately and does not affect the FPGA port.
- The two MII clocks have no timing constraints in the ASIC flow.

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
