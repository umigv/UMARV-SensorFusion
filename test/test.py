# SPDX-License-Identifier: Apache-2.0
import cocotb
from cocotb.triggers import RisingEdge, with_timeout


@cocotb.test()
async def test_sensor_fusion(dut):
    await with_timeout(RisingEdge(dut.done), 910, "ms")
