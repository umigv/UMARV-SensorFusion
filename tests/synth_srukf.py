"""Map the integrated UKF and a same-flow correction baseline to SKY130 HD."""

from argparse import ArgumentParser
from pathlib import Path
from re import findall, sub
from subprocess import run

from run_srukf_end_to_end import INITIAL_FACTOR, PROCESS_FACTOR, SOURCES, packed
from run_corr_x import word


def yosys_source(root):
    defs = (root / "Dev/sys_defs.sv").read_text()
    names = set(findall(r"}\s*(\w+)\s*;", defs))
    names.update(findall(r"\b((?:PROM_|PRED_ANGLE_|UOP_|SEQ_)\w+)\b", defs))
    names.update(("pred_s_factor_t", "meas_s_factor_t", "prom_math_read",
                  "angle_supported", "angle_needs_wrap", "angle_wrap_step", "fp32_lt"))
    sources = [f"package ukf_defs;\n{defs}\nendpackage"]
    for source in SOURCES:
        if source == "sys_defs":
            continue
        rtl = (root / f"Dev/{source}.sv").read_text()
        local = set(findall(r"localparam[^;=]*\b(\w+)\s*=", rtl))
        qualified = r"\b(" + "|".join(sorted(names - local)) + r")\b"
        rtl = sub(qualified, r"ukf_defs::\1", rtl)
        # Yosys' Verilog frontend requires assignments to function names.
        if source == "pred_sequencer":
            head, tail = rtl.split("    function automatic logic [31:0] operand", 1)
            rtl = head.replace("return ", "read_word = ") + (
                "    function automatic logic [31:0] operand" + tail
                .replace("        if (addr >=", "        else if (addr >=")
                .replace("        if (addr == PROM_", "        else if (addr == PROM_")
                .replace("        if (addr == ukf_defs::PROM_", "        else if (addr == ukf_defs::PROM_")
                .replace("return read_word(addr);", "else return read_word(addr);")
                .replace("return ", "operand = "))
        if source == "int_to_fp32":
            rtl = rtl.replace("integer i;\n", "").replace("for(i = 0;", "for(int i = 0;")
        if source == "vn_to_correction":
            rtl = rtl.replace("        if (unbiased", "        else if (unbiased", 2)
            rtl = rtl.replace("        shift = unbiased", "        else begin\n        shift = unbiased")
            rtl = rtl.replace("        exponent32 = 8'", "        else begin\n        exponent32 = 8'")
            rtl = rtl.replace("    endfunction", "        end\n        end\n    endfunction", 1)
            rtl = rtl.replace("return ", "narrow = ")
        sources.append(rtl)
    return "\n".join(sources)


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("liberty", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--correction-only", action="store_true")
    parser.add_argument("--timing-map", action="store_true",
                        help="enable constrained ABC buffering and sizing")
    parser.add_argument("--abc-delay-ps", type=int, default=20000,
                        help="ABC mapping target in ps with --timing-map (default 20000)")
    parser.add_argument("--flatten", action="store_true",
                        help="map across hierarchy boundaries")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    unit = output / "sources.sv"
    unit.write_text(yosys_source(root))
    liberty = args.liberty.resolve(strict=True)
    if args.correction_only:
        top = "UKF_corr_engine"
        setup = "chparam -set SHARE_MAC 1 -set REPORT_PRED_FAILURE 1 UKF_corr_engine"
    else:
        top = "UKF"
        initial = (int("".join(map(word, [.1, -.2, .7, -.8, .27])), 16) << 1) | 1
        setup = ("chparam -set WHEEL_VELOCITY_SCALE 32'h3e000000 "
                 "-set ORIGIN_LATITUDE 32'h42280000 -set ORIGIN_LONGITUDE 32'hc2a60000 "
                 f"-set INITIAL_STATE 161'h{initial:x} "
                 f"-set INITIAL_S 480'h{packed(INITIAL_FACTOR)} "
                 f"-set PROCESS_S 480'h{packed(PROCESS_FACTOR)} UKF")
    abc = f'abc -liberty "{liberty}"'
    if args.timing_map:
        constraint = output / "abc.constr"
        constraint.write_text("set_driving_cell sky130_fd_sc_hd__buf_4\nset_load 5.0\n")
        abc += f' -constr "{constraint}" -D {args.abc_delay_ps}'
    hierarchy_mode = " -flatten" if args.flatten else ""
    commands = "; ".join((
        f'read_verilog -sv "{unit}"', setup, f"hierarchy -check -top {top}",
        f"synth -noabc{hierarchy_mode} -top {top}", "check -assert", "delete t:$scopeinfo",
        f'dfflibmap -liberty "{liberty}"', abc,
        f'stat -liberty "{liberty}"', f'write_json "{output / "mapped.json"}"',
        f'write_verilog -noattr -noexpr -simple-lhs "{output / "mapped.v"}"',
    ))
    log_path = output / "yosys.log"
    with log_path.open("w") as log:
        run(["yosys", "-Q", "-T", "-p", commands], stdout=log, stderr=log, check=True)
    # An early undriven signal can disappear during constant propagation,
    # before the final check has a chance to report it.
    if findall(r"(?m)^Warning: Wire .* is used but has no driver\.$", log_path.read_text()):
        raise SystemExit(f"Synthesis found undriven wires. See {log_path}")


if __name__ == "__main__":
    main()
