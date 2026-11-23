"""Check the streamed five-state RTL against the integer model at each commit."""
import argparse
from pathlib import Path
import subprocess
import tempfile

RTL = Path(__file__).resolve().parents[1]
STUDY = RTL.parent / "tests" / "srukf_bias" / "study.cpp"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario")
    parser.add_argument("steps", type=int)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--study", type=Path)
    parser.add_argument("--skip-idle", action="store_true")
    parser.add_argument("--stalls", action="store_true")
    parser.add_argument("--reject-zero-bias", action="store_true")
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix="srukf-bias-rtl-") as directory:
        tmp = Path(directory)
        study = args.study.resolve() if args.study else tmp / "study"
        if not args.study:
            subprocess.run(["g++", "-std=c++17", "-O3", "-Wall", "-Wextra", "-Werror",
                            "-Wno-unused-function", STUDY, "-o", study], check=True)
        vectors = tmp / "vectors.hex"
        with vectors.open("w") as output:
            subprocess.run([study, RTL / "cordic.sv", args.scenario, str(args.steps),
                            str(args.start)], stdout=output, check=True)
        sources = ["srukf_bias_filter.sv", "srukf_400hz_scheduler.sv",
                   "srukf_arithmetic.sv", "cordic.sv", "tests/tb_srukf_bias.sv"]
        subprocess.run(["iverilog", "-g2012", "-s", "tb_srukf_bias",
                        f"-Ptb_srukf_bias.STEPS={args.steps}", "-o", tmp / "sim", *sources],
                       cwd=RTL, check=True)
        flags = [flag for flag, enable in (("+skip_idle", args.skip_idle), ("+stalls", args.stalls),
                                          ("+reject_zero_bias", args.reject_zero_bias))
                 if enable]
        subprocess.run(["vvp", tmp / "sim", f"+vectors={vectors}", *flags], check=True)


if __name__ == "__main__":
    main()
