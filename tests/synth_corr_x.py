"""Map the correction engine to the supplied SKY130 Liberty library."""

from pathlib import Path
from re import findall, compile
from subprocess import run
from sys import argv
from tempfile import TemporaryDirectory


def main():
    if len(argv) not in (2, 3):
        raise SystemExit("Usage: python3 tests/synth_corr_x.py <sky130.lib> [netlist.json]")
    liberty = Path(argv[1]).resolve(strict=True)
    root = Path(__file__).resolve().parents[1]
    source = root / "Dev"
    defs = (source / "sys_defs.sv").read_text()

    # Yosys 0.52 resolves packed struct widths when their typedefs are in a
    # package. Preserve the production declaration layout and qualify the
    # same types and enum members in a temporary synthesis compilation unit.
    names = findall(r"}\s*(\w+)\s*;", defs)
    names += ["pred_s_factor_t", "meas_s_factor_t"]
    symbol = compile(r"\b(" + "|".join(names) + r"|UOP_[A-Z_]+|SEQ_[A-Z_]+|PROM_[A-Z_]+|prom_math_read|angle_supported|angle_needs_wrap|angle_wrap_step)\b")
    with TemporaryDirectory() as temp:
        unit = Path(temp) / "corr_package.sv"
        sources = [(source / f"{name}.sv").read_text()
                   for name in ("FP_MAC", "FP_SQRTDIV", "corr_microcode", "corr_engine")]
        unit.write_text("\n".join([f"package ukf_defs;\n{defs}\nendpackage",
                                    *[symbol.sub(r"ukf_defs::\1", rtl) for rtl in sources]]))
        commands = (f'read_verilog -sv "{unit}"; synth -noabc -top UKF_corr_engine; '
                    f'dfflibmap -liberty "{liberty}"; abc -liberty "{liberty}"; '
                    f'stat -liberty "{liberty}"')
        if len(argv) == 3:
            commands += f'; write_json "{Path(argv[2]).resolve()}"'
        run(["yosys", "-Q", "-T", "-p", commands], check=True)


if __name__ == "__main__":
    main()
