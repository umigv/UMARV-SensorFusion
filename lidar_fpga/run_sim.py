from pathlib import Path
from cocotb_tools.runner import get_runner

ROOT = Path(__file__).resolve().parent
runner = get_runner("icarus")
runner.build(
    sources=[ROOT / "v16PacketParser.v"],
    hdl_toplevel="v16PacketParser",
    parameters={"HEADER_BYTES": 42},
    build_dir=ROOT / "sim_build",
    always=True,
)
runner.test(
    hdl_toplevel="v16PacketParser",
    test_module="test_v16",
    test_dir=ROOT,
)
