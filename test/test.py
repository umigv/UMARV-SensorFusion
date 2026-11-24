import cocotb
from cocotb.triggers import RisingEdge, with_timeout


@cocotb.test()
async def test_srukf(dut):
    await with_timeout(RisingEdge(dut.done), 510, "ms")
