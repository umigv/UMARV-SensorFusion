# Genesys 2 FPGA Port: ASIC Discrepancies, Parity Checklist & Stereo Odometry Compatibility

**Baseline ASIC snapshot:** commit `04025c5` (*"Update the tapeout snapshot: host-replaceable filter constants, shared state storage, two-clock paths"*)  
**Target FPGA:** Digilent Genesys 2 (`xc7k325tffg900-2`, Vivado 2026.1, `WNS = +8.265 ns`, `WHS = +0.020 ns`, `0` latches)  
**Verification status:** `tb_system_ukf` 17/17 PASS, `tb_fpga_top` 16/16 PASS, live Genesys 2 hardware selftest 38/38 PASS, live 400 Hz UKF + ESP32 + VN-300 + AMT22 + VLP-16 PC2X v3 verified.

---

## 1. Discrepancies Between Pulled ASIC RTL (`tapeout/src/`) and Current FPGA Implementation

### 1.1 RTL Modifications Inside `tapeout/src/`

Only **one** file in `tapeout/src/` is modified, and the ASIC (`else`) synthesis path is **100% untouched**:

| File | ASIC Path (default) | FPGA Path (`-d FPGA`) | Functional Equivalence |
|---|---|---|---|
| [`tapeout/src/ukf_top.sv`](../tapeout/src/ukf_top.sv) (`ukf_store`) | `WORDS=62`, `NCFG=8` implemented with `negedge`-pulsed `sky130_fd_sc_hd__and2_4` + `dlxtp_1` latches | Rising-edge flip-flops `q_main[0:61]` + `q_cfg[0:NCFG-1]` mapped onto `q[0:127]` | **Bit-for-bit identical** (exact 2-clock write latency, combinational read, 0 latches on Kintex-7) |

### 1.2 Replaced Module (Outside `tapeout/src/`, Selected in `syn/build.tcl`)

| ASIC File (excluded from FPGA build) | FPGA Replacement | Reason & Equivalence |
|---|---|---|
| [`tapeout/src/lidar_latch_mem.sv`](../tapeout/src/lidar_latch_mem.sv) | [`genesys2_chip/rtl/lidar_latch_mem.sv`](rtl/lidar_latch_mem.sv) | Replaces Sky130 `dlxtp_1` latches with Kintex-7 distributed LUTRAM (`256 × 16b`), preserving the exact 2-clock write / combinational read timing contract required by `lidar_point_buffer.sv` |

### 1.3 Board-Level Shims & Debug Harness (FPGA-Only, `genesys2_chip/rtl/`)

- **`rgmii_mii_100m.sv` + `mdio_init_100m.sv`**: Converts the Genesys 2 onboard RTL8211E-VL RGMII PHY (configured to 100BASE-TX full duplex via MDIO) to the chip's 4-bit MII interface. The ASIC uses MII directly.
- **`fpga_top.sv` + `fpga_selftest.sv`**: Generates `50 MHz` core / `25 MHz` MII TX clocks via MMCM from the 200 MHz LVDS board oscillator, routes Pmod JC (ESP32 host SPI) and Pmod JD (VN-300 + AMT22), and includes Vivado VIO/ILA debug hooks (`WITH_UKF`, `FPGA`).
- **Timing constraints (`xdc/genesys2_chip.xdc` vs `tapeout/src/project.sdc`)**: The Kintex-7 FPGA meets single-cycle 20 ns (`50 MHz`) timing across the entire chip (`WNS = +8.265 ns`) without needing the ASIC's 2-clock multicycle path relaxations from `project.sdc`.

### 1.4 Bench Sensor Setup & Runtime Register Differences

| Parameter / Interface | ASIC Reset Default / Target | Current Bench FPGA Setup | Reason |
|---|---|---|---|
| **VN-300 UART (`vn_rx`)** | Direct 921,600 baud TTL UART from VN-300 triggered by `vn_sync` at 400 Hz | Bridged from USB VN-300T-CR (`20 Hz` binary `0x1068`) → PC SDK → ESP32 Core-0 task (`921,600` baud on `GPIO 16 → JD1`) repeating latest cached frame on every `jd_vn_sync` | Bench VN-300 is USB-connected; Core-0 task ensures `< 15 µs` TX latency after `jd_vn_sync` |
| **`CTRL[6]` (`dw_check_turns`)** | `1` (expects 32-bit multi-turn AMT22) | **`0`** (`CTRL = 0x03` / `0x83` / `0xB3`) | Bench encoder is a **single-turn** 14-bit AMT22 (bytes 2–3 fail multi-turn parity check when `CTRL[6] = 1`) |
| **`Reg 0x08–0x09` (`lidar_port`)** | `2368` | **`0`** (accept any UDP port) | Bench VLP-16 transmits on a non-2368 destination UDP port |
| **`Reg 0x06–0x07` (`deadline_us`)** | `2000 µs` | **`1000 µs`** (auto-enforced by GUI on connect at 400 Hz) | Prevents a critical RTL race in `timestamper.sv` (see Section 2.1) |

### 1.5 Host-Side SDK / GUI Features (Not Part of ASIC)

- **ESP32 400 Hz dead-wheel accumulator** (`esp32_spi_host.ino`, `S`/`Z` commands): Samples `0x4C` + `0x68–0x6B` at `t = 2100 µs` after `jd_vn_sync`, unwraps 14-bit single-turn rollovers in software, and exposes cumulative counts & distance (`SensorFusionChip.deadwheel_distance_m()`).
- **Live VN-300 GNSS1/GNSS2 satellite & fix readout** (`VN300GnssStatus` in `sensorfusion_sdk.py`): Interleaves 1 Hz `$VNRRG,58` + `$VNRRG,103` queries alongside the binary stream and decodes `InsStatus` from binary bytes `52:54`.
- **Dashboard (`sensorfusion_gui.py`)**: Global-frame 2D lidar map with 1 s intensity fade, RANSAC + 3×3 OLS tilted ground-plane filtering, real-time robot arrow with indoor `VN+Wheel` dead-reckoning fallback when `vn_valid = 0`, and full SPI/UKF register write console.

---

## 2. Changes Required for Identical Functional Accuracy on ASIC

### 2.1 Critical RTL Race to Fix Before Tapeout (`timestamper.sv` / `spi_regs.sv`)

During live hardware testing, we caught and reproduced a fatal UKF blow-up (`OUT_X = +Inf`, `OUT_Y = -Inf`, `OUT_TH = 1e34`, `XH_R = -2.5e24`) caused by an interface race between `timestamper.sv`, `spi_regs.sv`, and `ukf_top.sv`:

- **Mechanism:** [`timestamper.sv`](../tapeout/src/timestamper.sv#L243-L249) shifts incoming VN-300 UART bytes **in-place** directly into `yaw_raw`, `rate_raw`, `lat_raw`, and `lon_raw` (`lat_raw <= {rx_byte, lat_raw[63:8]}`) without latching them into hold registers on `collect_end`.
- **Timing violation of the beat budget:** One UKF beat takes **`~1,300 µs`** (reading `LON`, `LAT`, `YAW`, `RATE` near the end of the beat), while `spi_regs.sv` defaults `deadline_us` (`0x06–0x07`) to **`2000 µs`** inside a **`2500 µs`** beat (`2000 + 1300 = 3300 µs > 2500 µs`).
- **Failure mode:** Any VN-300 packet that completes between `~1,200 µs` and `2,000 µs` after `vn_sync` still asserts `vn_valid = 1` and launches `ukf_top`. Beat `N+1` then begins at `2,500 µs` and shifts new UART bytes directly over `lat_raw`/`yaw_raw`/`rate_raw` **while beat `N`'s UKF is reading them**, injecting partial-shifted float/double bit patterns (`+1.6–2.8 m` single-beat `y` jumps, `+159 rad/s` `ω` spikes, and `sinpoly()` overflow to `±Inf`).
- **Recommended fix for identical ASIC accuracy:**
  1. **Best (RTL):** Double-buffer `yaw_raw`, `rate_raw`, `lat_raw`, `lon_raw` into hold registers on `collect_end` (or gate UART byte shifts while `ukf_top` is busy), **and**
  2. **Also recommended (`spi_regs.sv` default + host config):** Lower default `deadline_us` (`0x06–0x07`) from `2000 µs` to **`1000 µs`** at 400 Hz (`1000 + 1300 = 2300 µs < 2500 µs`). With this active on FPGA + Core-0 ESP32 TX, continuous 400 Hz stress tests achieved **0 jumps** (`max|xy| = 1.91e-5 m`, `r = 0.05000 m`).

### 2.2 Single-Turn vs. Multi-Turn AMT22 Encoder Rollover

- [`timestamper.sv`](../tapeout/src/timestamper.sv#L373-L396) constructs a 28-bit position `dw_now = {rx_shift[13:0], rx_shift[29:16]}` (`{turns[13:0], pos[13:0]}`) and computes `dw_delta_r = dw_now - dw_prev`.
- With a **single-turn** AMT22 (`CTRL[6] = 0`), the upper 14 turn bits do not increment, so every full shaft revolution (`16383 → 0`) produces a spurious **`-16,383` count jump** inside the FPGA UKF (even though the ESP32 debug accumulator unwraps 14-bit wrap in software).
- **Required for rover accuracy:** Use either a **multi-turn AMT22** (`CTRL[6] = 1`) **or** an MCU SPI interceptor that outputs a properly wrapped 28-bit multi-turn `{turns[13:0], pos[13:0]}` packet (see Section 3).

### 2.3 Direct VN-300 TTL Wiring & Rate Alignment

- On the rover, wire the VN-300T-CR TTL UART (`921,600` baud, Binary Output 1 `0x1068` triggered on `SyncIn` rising edge) directly to `vn_rx` and `vn_sync` rather than repeating 20 Hz USB frames through the ESP32 bridge.
- Note that `timestamper.sv` uses a single `vn_valid` flag requiring `InsStatus` mode `2` (`Tracking`) + `GpsFix = 1` + no sensor error bits (`L258`). Indoors or during prolonged GPS outage, `vn_valid = 0` skips **all** VN-300 updates (including yaw & turn rate) and keeps the UKF halted if `started == 0`.

### 2.4 Host-Writable Filter Constants & Mounting Parameters

Before deploying on the physical rover, write the vehicle-specific values via SPI (`0x02–0x23` and `0x6C`/`0x70–0x73`, all already supported in `sensorfusion_sdk.py` and `sensorfusion_gui.py`):
- **`RNOM` (word `103`):** Dead-wheel **radius** in metres (`diameter / 2`, reset `0.05 m`).
- **`CNT2RAD` (word `102`):** Radians per count (reset `2π / 16384` for 14-bit AMT22).
- **`RR01`, `RR2`, `RR3`, `SQ01`, `SQ3`, `SQ2` (words `96–101`):** Tune if GPS/IMU/odometry noise differs from the 400 Hz scaled defaults (note `RR01 = 7089.8 m²` gives a ~35 s GPS-only position time constant by design).
- **`yaw_offset` (`0x0A–0x0B`), `lidar_model` (`0x03`), `lidar_port` (`0x08–0x09`), and `CTRL` (`0x02`):** Set `CTRL[7] = 1` (UKF state), `CTRL[4:5]` (lidar rotation/translation motion correction), and `CTRL[6] = 1` (when using multi-turn AMT22 or MCU emulator).

---

## 3. Forward Compatibility with Stereo Visual Odometry (MCU SPI Interception)

### 3.1 Short Answer: Are Any ASIC/FPGA Hardware Changes Needed?

**No — zero ASIC or FPGA hardware changes are required** if an external MCU intercepts the AMT22 dead-wheel encoder, fuses it with stereo visual odometry (VO), and presents an identical 32-bit AMT22 multi-turn SPI slave response to the chip's encoder pins (`dw_cs_n`, `dw_sck`, `dw_mosi`, `dw_miso`).

### 3.2 Exact AMT22 SPI Slave Contract Expected by the Chip

Verified directly against [`tapeout/src/timestamper.sv`](../tapeout/src/timestamper.sv#L265-L402):

- **Trigger & Rate:** Initiated automatically on every 400 Hz beat (`beat` pulse).
- **SPI Mode & Clock:** Mode 0 (`CPOL = 0`, `CPHA = 0`), MSB-first, `SCK ≈ 1.923 MHz` (`13` clocks half-period at `50 MHz`). `CS_N` drops `3.0 µs` (`150` clocks) before the first `SCK` rising edge, pauses `3.0 µs` between each of the 4 bytes, and rises `3.0 µs` after byte 3 (~32 µs total).
- **Chip → MCU (`MOSI`):** 4 bytes `{0x00, 0xA0, 0x00, 0x00}` (standard AMT22 multi-turn read command).
- **MCU → Chip (`MISO`, 32 bits):**
  - **Bytes 0–1 (`rx_shift[31:16]`):** Position word `W_pos = {K1_p, K0_p, pos[13:0]}`
  - **Bytes 2–3 (`rx_shift[15:0]`):** Turn-count word `W_trn = {K1_t, K0_t, turns[13:0]}`
  - **Odd checkbits on each 16-bit word `w`:**
    ```c
    uint16_t amt22_encode14(uint16_t val14) {
      val14 &= 0x3FFF;
      uint16_t k1 = !(((val14>>13)^(val14>>11)^(val14>>9)^(val14>>7)^(val14>>5)^(val14>>3)^(val14>>1)) & 1);
      uint16_t k0 = !(((val14>>12)^(val14>>10)^(val14>>8)^(val14>>6)^(val14>>4)^(val14>>2)^(val14>>0)) & 1);
      return (k1 << 15) | (k0 << 14) | val14;
    }
    ```
  - **Reconstructed 28-bit cumulative count:** `dw_now = {turns[13:0], pos[13:0]}` (wrapping signed 28-bit arithmetic; `dw_delta = dw_now - dw_prev`, valid up to `±2^25` counts/beat).

### 3.3 Integration Notes & Nuances to Keep in Mind

1. **Use a Hardware SPI Slave + DMA/Double-Buffer:** At `1.92 MHz` SCK with `3 µs` CS-to-SCK lead time, software bit-banging is too slow. Use an MCU hardware SPI slave peripheral (e.g., STM32 or ESP32 SPI slave DMA) with a pre-loaded 4-byte TX buffer updated atomically right after `CS_N` rises so no transfer ever tears.
2. **Leave `CTRL[6] = 1` (`dw_check_turns = 1`) Enabled:** Because the MCU generates valid `K1/K0` checkbits on both the 14-bit position and 14-bit turn words, you get full **28-bit wrap-free virtual displacement** (also eliminating the single-turn AMT22 rollover issue even if the physical encoder wired to the MCU is single-turn!).
3. **Scale Calibration & Online Radius State (`XH[4] = r`):**
   - Convert the fused longitudinal forward displacement $s_{\text{fused}}$ (metres) to cumulative virtual counts:
     $$C_{\text{virt}} = \operatorname{round}\!\left(\frac{s_{\text{fused}}}{\text{CNT2RAD} \cdot \text{RNOM}}\right)$$
     (At default `CNT2RAD = 2π/16384`, `RNOM = 0.05 m`, `1 count = 19.175 µm`; you can also set `CNT2RAD`/`RNOM` via `0x6C`/`0x70–73` to any convenient metric scale such as `0.1 mm/count`.)
   - Because `ukf_top` continuously adapts `XH[4]` (`r`) against GPS, it will automatically absorb and track any residual stereo VO baseline/scale drift without breaking filter convergence.
4. **1-DOF Longitudinal Scope (and Optional 2D/Yaw Extension):**
   - Spoofing the AMT22 channel injects **1-DOF forward displacement** ($ds$ along robot heading $\theta$) — ideal for rejecting wheel slip and fusing longitudinal stereo VO distance with zero hardware changes.
   - It does **not** directly inject lateral slip ($dy_{\text{body}}$) or stereo VO yaw/turn-rate. If you ever want full 2D VO fusion or GPS-denied indoor operation in the future, the **same MCU** can optionally intercept and fuse VO into the **VN-300 UART** (`921,600` baud 56-byte binary `0xFA 01 68 10` frame) — likewise with **zero ASIC hardware changes**.
