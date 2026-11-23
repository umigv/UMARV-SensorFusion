"""Word-for-word check of the mixed RTL against the integer precision study."""
import argparse
from pathlib import Path
import subprocess
import tempfile

RTL = Path(__file__).resolve().parents[1]
STUDY = RTL.parent / "tests" / "srukf_precision" / "study.cpp"

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario")
    parser.add_argument("steps", type=int)
    parser.add_argument("--start", type=int, default=0, help="seed RTL from a reference checkpoint")
    parser.add_argument("--study", type=Path, help="reuse the compiled C++ study")
    parser.add_argument("--skip-idle", action="store_true")
    parser.add_argument("--stalls", action="store_true")
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix="srukf-mixed-") as tmp:
        tmp = Path(tmp)
        study = args.study.resolve() if args.study else tmp / "study"
        if not args.study:
            subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                            STUDY, "-o", study], check=True)
        with (tmp / "vectors.hex").open("w") as output:
            subprocess.run([study, RTL / "cordic.sv", f"rtl-fixture-{args.scenario}", str(args.steps),
                            str(args.start)],
                           stdout=output, check=True)
        sources = ("srukf_filter.sv", "srukf_400hz_scheduler.sv", "srukf_arithmetic.sv",
                   "cordic.sv", "tests/tb_srukf_mixed.sv")
        subprocess.run(["iverilog", "-g2012", "-s", "tb_srukf_mixed",
                        f"-Ptb_srukf_mixed.STEPS={args.steps}", "-o", tmp / "sim", *sources],
                       cwd=RTL, check=True)
        extra = [opt for opt, use in (("+skip_idle", args.skip_idle), ("+stalls", args.stalls)) if use]
        subprocess.run(["vvp", tmp / "sim", f"+vectors={tmp / 'vectors.hex'}", *extra], check=True)


if __name__ == "__main__":
    main()
