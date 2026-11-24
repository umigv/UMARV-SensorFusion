# SPDX-License-Identifier: Apache-2.0
#
# Smoke test of the chip top level with no sensors, lidar or host attached (the chip runs on
# its reset settings): the VN-300 line idles high
# and the encoder returns all zeros. Checks that the beat runs at 400 Hz, that each beat
# produces a sync pulse and one 32-clock encoder read, and that the "measurements ready"
# pulse still comes, at the deadline, when nothing answers.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

CLK_NS = 20              # 50 MHz
PERIOD_CLKS = 125_000    # 2,500 us beat
DEADLINE_CLKS = 100_000  # 2,000 us


@cocotb.test()
async def test_beats_without_sensors(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())

    dut.ena.value = 1
    dut.uio_in.value = 0
    dut.ui_in.value = 0b0000_1001   # vn_rx idle high, encoder miso low, host SPI idle (cs_n high), MII idle
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    sync_rises, cs_falls, sck_rises, pulses = [], [], 0, []
    prev = int(dut.uo_out.value)
    assert prev & 0b0000_0010, "encoder chip select should idle high"
    assert not prev & 0b0000_0101, "sync and sck should idle low"

    total = 2 * PERIOD_CLKS + DEADLINE_CLKS + 5_000
    for cycle in range(total):
        await RisingEdge(dut.clk)
        now = int(dut.uo_out.value)
        rose, fell = now & ~prev, prev & ~now
        if rose & 0x01:
            sync_rises.append(cycle)
        if fell & 0x02:
            cs_falls.append(cycle)
        if rose & 0x04:
            sck_rises += 1
            assert not now & 0x02, "sck toggled while chip select was high"
        if rose & 0x10:
            pulses.append(cycle)
        prev = now

    dut._log.info(f"sync pulses at {sync_rises}, reads at {cs_falls}, ready pulses at {pulses}")
    assert len(sync_rises) == 2, "expected two beats"
    assert sync_rises[1] - sync_rises[0] == PERIOD_CLKS, "beats should be 2,500 us apart"
    assert len(cs_falls) == 2, "expected one encoder read per beat"
    assert sck_rises == 64, "expected 32 clock pulses per encoder read"
    assert len(pulses) == 2, "expected one ready pulse per beat"
    for s, p in zip(sync_rises, pulses):
        assert abs((p - s) - DEADLINE_CLKS) < 200, "with no sensors the ready pulse comes at the deadline"
    assert not prev & 0x80, "MII transmit should stay idle with no lidar"


# ---------------------------------------------------------------------------------------------
# Lidar path: one VLP-16 packet in on the MII receive pins, two frames of x, y, z out on the MII
# transmit pins. The chip runs on its reset settings (no host), so no motion correction is
# applied and every point can be checked against plain trigonometry. This is the test that
# exercises the latch point buffer in the gate-level run.
# ---------------------------------------------------------------------------------------------
import math
import zlib

from cocotb.triggers import Timer

MII_HALF_NS = 20                      # 25 MHz PHY clocks
DIST = [((7 * i + 3) % 29) * 300 + 400 for i in range(32)]      # distances in 2 mm counts, 0.8 to 17.6 m


def lidar_frame():
    f = bytearray((i * 37 + 11) & 0xFF for i in range(42 + 1206))
    f[12:14] = b"\x08\x00"; f[14] = 0x45; f[20:22] = b"\x40\x00"; f[23] = 0x11
    f[36:38] = (2368).to_bytes(2, "big"); f[38:40] = (1214).to_bytes(2, "big")
    for b in range(12):
        o = 42 + 100 * b
        f[o:o + 2] = b"\xff\xee"
        f[o + 2:o + 4] = (40 * b).to_bytes(2, "little")             # azimuth 0.00, 0.40, ... degrees
        for p in range(32):
            d = 0 if (b == 5 and p == 9) else DIST[p]               # one "no return"
            f[o + 4 + 3 * p:o + 6 + 3 * p] = d.to_bytes(2, "little")
    return bytes(f) + zlib.crc32(bytes(f)).to_bytes(4, "little")


def expected_point(block, p):
    d = 0 if (block == 5 and p == 9) else DIST[p]
    if d == 0:
        return (-32768, -32768, -32768)
    laser = p % 16
    el = math.radians(laser if laser % 2 else laser - 15)
    az = math.radians((40 * block + (20 if (p >= 16 and block > 0) else 0)) / 100.0)
    g = d * math.cos(el)
    return (g * math.cos(az), -g * math.sin(az), d * math.sin(el))


@cocotb.test()
async def test_lidar_packet(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    pins = {"ui": 0b0000_1001, "rxd": 0}

    def drive():
        dut.ui_in.value = pins["ui"]
        dut.uio_in.value = pins["rxd"]

    dut.ena.value = 1
    drive()
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    frame = lidar_frame()
    nibbles = [0x5] * 15 + [0xD]
    for byte in frame:
        nibbles += [byte & 0xF, byte >> 4]
    frames, cur, n_nib, low = [], None, 0, 0
    total_edges = 400 + len(nibbles) + 9000        # idle, the packet, then time for both frames to leave
    for edge in range(total_edges):
        # falling edge of both PHY clocks: change the receive pins
        k = edge - 400
        pins["ui"] &= ~0b1110_0000
        if 0 <= k < len(nibbles):
            pins["ui"] |= 0b0010_0000
            pins["rxd"] = nibbles[k]
        else:
            pins["rxd"] = 0
        drive()
        await Timer(MII_HALF_NS, unit="ns")
        # just before the rising edge: sample the transmit pins
        uo, uio = dut.uo_out.value, dut.uio_out.value
        tx_en = int(uo) >> 7 & 1 if uo.is_resolvable else 0
        if tx_en:
            if cur is None:
                cur, n_nib = [], 0
            nib = int(uio) >> 4
            if n_nib >= 16:
                if n_nib % 2 == 0:
                    low = nib
                else:
                    cur.append(nib << 4 | low)
            n_nib += 1
        elif cur is not None:
            frames.append(bytes(cur)); cur = None
        pins["ui"] |= 0b1100_0000
        drive()
        await Timer(MII_HALF_NS, unit="ns")

    dut._log.info(f"frames out: {[len(f) for f in frames]}")
    assert len(frames) == 2, "one lidar packet should produce two frames"
    worst = 0.0
    for n, f in enumerate(frames):
        assert len(f) == 1270, "frame length"
        assert zlib.crc32(f[:-4]).to_bytes(4, "little") == f[-4:], "frame checksum"
        assert f[42:46] == b"PC2X" and f[46] == 3, "payload magic and version"
        assert f[47] & 1 == 0, "no state has been supplied, so the state flag must be clear"
        assert int.from_bytes(f[50:54], "little") == n, "sequence number"
        for blk in range(6):
            o = 90 + 196 * blk
            block = 6 * n + blk
            assert int.from_bytes(f[o:o + 2], "little") == 40 * block, "block azimuth"
            assert f[o + 2] == 0, "no sweep start and no padding expected"
            for p in range(32):
                got = [int.from_bytes(f[o + 4 + 6 * p + 2 * a:o + 6 + 6 * p + 2 * a], "little", signed=True) for a in range(3)]
                want = expected_point(block, p)
                err = max(abs(g - w) for g, w in zip(got, want))
                worst = max(worst, err)
                assert err <= 3.0, f"block {block} point {p}: got {got}, expected {want}"
    dut._log.info(f"384 points checked, worst error {worst:.2f} counts")
