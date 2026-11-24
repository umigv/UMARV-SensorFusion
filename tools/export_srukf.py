"""Export the optimized core from the UMARV-SensorFusion checkout."""
from argparse import ArgumentParser
from pathlib import Path
from subprocess import run
import sys
from tempfile import TemporaryDirectory


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("core_repo", type=Path)
    core = parser.parse_args().core_repo.resolve(strict=True)
    sys.path.insert(0, str(core / "tests"))
    from synth_srukf import yosys_source
    from run_srukf_end_to_end import INITIAL_FACTOR, PROCESS_FACTOR, packed
    from run_corr_x import word

    initial = (int("".join(map(word, [.1, -.2, .7, -.8, .27])), 16) << 1) | 1
    output = Path(__file__).resolve().parents[1] / "src/srukf_core.v"
    with TemporaryDirectory() as temp:
        unit = Path(temp) / "design.sv"
        emitted = Path(temp) / "core.v"
        unit.write_text(yosys_source(core))
        commands = (
            f'read_verilog -sv "{unit}"',
            "chparam -set WHEEL_VELOCITY_SCALE 32'h3e000000 "
            f"-set INITIAL_STATE 161'h{initial:x} -set INITIAL_S 480'h{packed(INITIAL_FACTOR)} "
            f"-set PROCESS_S 480'h{packed(PROCESS_FACTOR)} UKF",
            "hierarchy -check -top UKF",
            "proc",
            "opt_clean",
            "check -assert",
            f'write_verilog -noattr "{emitted}"',
        )
        run(["yosys", "-Q", "-T", "-p", "; ".join(commands)], check=True)
        output.write_text(emitted.read_text().replace(str(unit), "design.sv"))


if __name__ == "__main__":
    main()
