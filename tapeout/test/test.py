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
