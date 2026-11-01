"""Cocotb verification of v16PacketParser (run with python run_sim.py)."""
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer


async def initialize(dut):
    dut.resetn.value = 0
    dut.s_AXIstream_tvalid.value = 0
    dut.s_AXIstream_tlast.value = 0
    dut.s_AXIstream_tdata.value = 0
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    for _ in range(3):
        await FallingEdge(dut.clk)
    dut.resetn.value = 1


async def push(dut, byte, *, last=False, expected=None):
    """Drive at falling edge, sample registered output 1ns after rising edge."""
    await FallingEdge(dut.clk)
    dut.s_AXIstream_tdata.value = byte
    dut.s_AXIstream_tvalid.value = 1
    dut.s_AXIstream_tlast.value = int(last)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")
    assert int(dut.s_AXIstream_tready.value) == 1, "Unexpected backpressure"
    actual_valid = int(dut.out_data_valid.value)
    assert actual_valid == int(expected is not None), (
        f"Output pulse mismatch after byte {byte:02X}: got {actual_valid}, "
        f"expected {int(expected is not None)}"
    )
    if expected is not None:
        actual = (int(dut.out_azimuth.value),
                  int(dut.out_distance.value),
                  int(dut.out_reflectivity.value))
        assert actual == expected, f"Point mismatch: got {actual}, expected {expected}"
        return 1
    return 0


async def pause(dut, cycles=3):
    await FallingEdge(dut.clk)
    dut.s_AXIstream_tvalid.value = 0
    dut.s_AXIstream_tlast.value = 0
    for _ in range(cycles):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        assert int(dut.out_data_valid.value) == 0, "Output while stream paused"


async def send_packet(dut, *, corrupt_blocks=(), random_pauses=False, seed=123):
    rng = random.Random(seed)
    count = 0
    for _ in range(42):
        await push(dut, 0)
    for b in range(12):
        await push(dut, 0 if b in corrupt_blocks else 0xFF)
        await push(dut, 0xEE)
        await push(dut, b)
        await push(dut, 0x12)
        az = 0x1200 | b
        for p in range(32):
            await push(dut, p)
            if b == 0 and p == 0:
                await pause(dut)
            if random_pauses and rng.random() < 0.08:
                await pause(dut, rng.randint(1, 5))
            await push(dut, b)
            expected = None if b in corrupt_blocks else (az, (b << 8) | p, p + 10)
            count += await push(dut, p + 10, expected=expected)
    for i in range(6):
        await push(dut, 0, last=(i == 5))
    return count


@cocotb.test()
async def original_testbench_equivalent(dut):
    await initialize(dut)
    count = await send_packet(dut, corrupt_blocks={5})
    assert count == 352, f"Expected 352 outputs, got {count}"
    await push(dut, 0, last=True)
    dut._log.info("PASS original test: 352 points, corrupted block suppressed")


@cocotb.test()
async def two_packets_with_random_pauses(dut):
    await initialize(dut)
    first = await send_packet(dut, random_pauses=True, seed=41)
    second = await send_packet(dut, corrupt_blocks={1, 8}, random_pauses=True, seed=42)
    assert first == 384, f"First packet expected 384 points, got {first}"
    assert second == 320, f"Second packet expected 320 points, got {second}"
    dut._log.info("PASS consecutive packets and randomized valid gaps")


@cocotb.test()
async def early_tlast_restarts_frame(dut):
    await initialize(dut)
    # Incomplete header is terminated, next byte must restart at byte index zero.
    for i in range(17):
        await push(dut, i, last=(i == 16))
    count = await send_packet(dut)
    assert count == 384, f"Expected 384 points after early tlast, got {count}"
    dut._log.info("PASS early tlast resets packet alignment")
