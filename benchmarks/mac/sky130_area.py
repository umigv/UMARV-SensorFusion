"""Matched four-way MAC mapping plus a multiplier/shift ablation.

Pass an uncompressed SKY130 HD Liberty file. Keep paths free of quotes.
"""
import argparse
from collections import Counter
import hashlib
import json
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

sys.dont_write_bytecode = True
from benchmark import HERE, RESULTS, ROOT, SOURCES, TESTBENCH, fixtures
from synthesize import liberty_areas

CASES = {
    "original": ("fp32_mac_serial_legacy", None),
    "radix4_a": ("fp32_mac_a", None),
    "partial_d": ("fp32_mac_d", None),
    "dadda_f": ("fp32_mac_f", None),
    "original_fast": ("fp32_mac_original_fast", None),
    "booth_original_shifts": ("fp32_mac_matched_booth", 0),
    "booth_fast_shifts": ("fp32_mac_matched_booth", 1),
}
COMMON_TOP = "mac_physical_top"
PORTS = """
    input wire clk, rst_n, start, clear, last,
    input wire [31:0] a, b,
    output wire busy, done, result_valid,
    output wire [31:0] result,
    output wire [4:0] exception_flags
"""


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def select(source, begin, end):
    left = source.index(begin)
    return source[left:source.index(end, left)]


def isolation(out):
    """Copy production's adder and transplant only the four shift states."""
    production = (ROOT/"fp32_mac_serial_legacy.sv").read_text()
    experiment = (HERE/"fp32_mac_experiment.sv").read_text()
    adder = select(production, "    // Sparse carry lookahead", "    always_ff")
    shifts = select(experiment, "    // Binary group selection", "    always_ff")
    original_fast = production.replace("module fp32_mac_serial_legacy (",
                                       "module fp32_mac_original_fast (", 1)
    original_fast = original_fast.replace("    always_ff", "    localparam FAST_SHIFT = 1;\n\n"
                                           + shifts + "    always_ff", 1)
    for first, last in (("NORMALIZE_PRODUCT", "ADD"), ("NORMALIZE_SUM", "ROUND")):
        begin, end = f"                {first}:", f"                {last}:"
        original_fast = original_fast.replace(select(production, begin, end),
                                              select(experiment, begin, end), 1)
    booth = experiment.replace("module fp32_mac_experiment", "module fp32_mac_matched_booth", 1)
    booth = booth.replace(select(booth, "    experiment_adder", "    // Binary group selection"), adder, 1)
    variant = out/"isolation.sv"
    variant.write_text("// Generated from current sources by sky130_area.py.\n" + original_fast +
                       "\n" + booth)
    return variant


def wrapper(destination, name, parameter, simulation=False):
    params = f" #(.ARCH(0), .FAST_SHIFT({parameter}))" if parameter is not None else ""
    top = "fp32_mac" if simulation else COMMON_TOP
    internal = "core" if parameter is None and name == "fp32_mac_a" else ""
    alias = f"    wire [31:0] accumulator = impl.{internal+'.' if internal else ''}accumulator;\n"
    destination.write_text(f"`timescale 1ns/1ps\nmodule {top} ({PORTS});\n"
                           f"    {name}{params} impl (.*);\n"
                           + (alias if simulation else "") + "endmodule\n")


def design_sources(name, experimental):
    if name in ("fp32_mac", "fp32_mac_serial_legacy"):
        return [ROOT/f"{name}.sv"]
    return [*SOURCES, experimental]


def logged(log, args):
    with log.open("w") as output:
        subprocess.run(args, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT, check=True)


def verify(out, experimental):
    # The production and public Booth configurations already have full regressions.
    arms = ("original_fast", "booth_original_shifts", "booth_fast_shifts")
    with tempfile.TemporaryDirectory() as directory, ThreadPoolExecutor(len(arms)) as pool:
        temp = Path(directory)
        vectors = temp/"vectors"
        expected = len(fixtures(vectors, 0))

        def run(arm):
            name, parameter = CASES[arm]
            adapter = temp/f"{arm}.sv"
            wrapper(adapter, name, parameter, simulation=True)
            sim = temp/f"{arm}.vvp"
            subprocess.run(["iverilog", "-g2012", "-Wall", "-s", "tb_fp32_mac", "-o", sim,
                            *design_sources(name, experimental), adapter, TESTBENCH],
                           cwd=ROOT, check=True)
            log = out/f"{arm}_functional.log"
            logged(log, ["vvp", sim, f"+vectors={vectors}", "+latency"])
            data = log.read_text()
            wanted = {"original_fast": 33, "booth_original_shifts": 166, "booth_fast_shifts": 22}[arm]
            if f"PASS: {expected} reference MAC operations, max sampled latency {wanted} cycles" not in data:
                raise RuntimeError(f"{arm}: check result in {log}")
            # Keep the original worst-case witness and pass output alongside the exact fixture generator.
            log.write_text("\n".join(line for line in data.splitlines()
                                     if line.startswith(("LAT MAC 35 ", "PASS", str(TESTBENCH)))) + "\n")
            print(f"{arm}: {expected} bit-exact requests, maximum {wanted} cycles", flush=True)

        for future in [pool.submit(run, arm) for arm in arms]:
            future.result()


def plans(out, experimental, liberty, cases):
    for label, (name, param) in cases.items():
        arm = out / label
        arm.mkdir(exist_ok=True)
        adapter = arm/"top.sv"
        wrapper(adapter, name, param)
        sources = [*design_sources(name, experimental), adapter]
        lib = str(liberty) if liberty else "REQUIRED_SKY130_HD_LIBERTY.lib"
        (arm/"map.ys").write_text("\n".join([
            f'read_liberty -lib -ignore_miss_func "{lib}"',
            "read_verilog -sv " + " ".join(f'"{p}"' for p in sources),
            f"hierarchy -check -top {COMMON_TOP}",
            f"synth -flatten -noabc -top {COMMON_TOP}",
            f'dfflibmap -liberty "{lib}"',
            f'abc -D 20000 -liberty "{lib}"',
            "clean -purge", "check -assert",
            f'stat -liberty "{lib}"',
            f'write_json "{arm/"mapped.json"}"',
            f'write_verilog -noattr -noexpr "{arm/"mapped.v"}"',
        ]) + "\n")
        (arm/"sta.tcl").write_text(f"""read_liberty {{{lib}}}
read_verilog {{{arm/"mapped.v"}}}
link_design {COMMON_TOP}
set_cmd_units -time ns
read_sdc {{{out/"mac.sdc"}}}
report_checks -path_delay max -group_path_count 5 -format full_clock_expanded -digits 4
report_worst_slack -max
report_wns
report_clock_min_period -include_port_paths
exit
""")
    (out/"mac.sdc").write_text("""create_clock -name mac_clk -period 20.0 [get_ports clk]
set_input_delay -clock mac_clk 2.0 [get_ports {start clear last a* b*}]
set_output_delay -clock mac_clk 2.0 [all_outputs]
set_false_path -from [get_ports rst_n]
""")


def liberty_dffs(liberty):
    source = liberty.read_text()
    matches = list(re.finditer(r'\bcell\s*\(\s*"?([^"\s)]+)"?\s*\)\s*\{', source))
    return {cell[1] for i, cell in enumerate(matches)
            if re.search(r"\b(?:ff|ff_bank)\s*\(",
                         source[cell.end():matches[i+1].start() if i+1 < len(matches) else len(source)])}


def mapped_area(arm, lib_cells, dffs):
    data = json.loads((arm/"mapped.json").read_text())["modules"][COMMON_TOP]["cells"]
    counts = Counter(cell["type"] for cell in data.values() if cell["type"] != "$scopeinfo")
    if counts.keys() - lib_cells.keys():
        raise RuntimeError(f"{arm.name}: area unknown for {counts.keys() - lib_cells.keys()}")
    ff_count = sum(n for c, n in counts.items() if c in dffs)
    if any(lib_cells[c][1] and c not in dffs for c in counts):
        raise RuntimeError(f"{arm.name}: mapped latch or other non-DFF storage")
    seq = sum(n*lib_cells[c][0] for c, n in counts.items() if lib_cells[c][1])
    comb = sum(n*lib_cells[c][0] for c, n in counts.items() if not lib_cells[c][1])
    return {"cells": sum(counts.values()), "dffs": ff_count, "sequential_um2": seq,
            "combinational_um2": comb, "total_um2": seq+comb, "by_cell": dict(counts),
            "latch_instances": 0, "unmapped_logic_instances": 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--liberty", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--production", action="store_true", help="Map only the production MAC")
    parser.add_argument("--verify-isolation", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()
    if args.production and args.verify_isolation:
        parser.error("--verify-isolation applies only to the comparison variants")
    out = (args.out or RESULTS/("radix4_integration" if args.production else "sky130_area")).resolve()
    out.mkdir(parents=True, exist_ok=True)
    liberty = args.liberty.resolve(strict=True) if args.liberty else None
    tools = {tool: shutil.which(tool) for tool in ("yosys", "sta", "openroad", "openlane", "volare", "tt_tool")}
    cases = {"production": ("fp32_mac", None)} if args.production else CASES
    exp = None if args.production else isolation(out)
    plans(out, exp, liberty, cases)
    (out/"run.json").write_text(json.dumps({"tools": tools, "liberty": str(liberty) if liberty else None,
                                          "liberty_sha256": digest(liberty) if liberty else None,
                                          "production_sha256": digest(ROOT/"fp32_mac.sv"),
                                          "legacy_sha256": digest(ROOT/"fp32_mac_serial_legacy.sv"),
                                          "clock_ns": 20, "input_delay_ns": 2, "output_delay_ns": 2},
                                         indent=2)+"\n")
    if args.verify_isolation:
        verify(out, exp)
    if args.plan_only:
        return
    if not tools["yosys"] or liberty is None:
        parser.exit(1, "SKY130 mapping blocked: Yosys and an explicit SKY130 HD Liberty corner are required.\n")
    # Reject wrong libraries and make missing cell area a hard error.
    lib_cells = liberty_areas(liberty)
    if not lib_cells or any(not cell.startswith("sky130_fd_sc_hd__") for cell in lib_cells):
        parser.error("expected a SKY130 HD standard-cell Liberty file")
    summary = {}
    dffs = liberty_dffs(liberty)
    for label in cases:
        arm = out/label
        logged(arm/"yosys.log", [tools["yosys"], "-s", arm/"map.ys"])
        summary[label] = mapped_area(arm, lib_cells, dffs)
        if tools["sta"]:
            logged(arm/"sta.log", [tools["sta"], "-no_init", arm/"sta.tcl"])
    original = None if args.production else summary["original"]["total_um2"]
    if args.production:
        reference = RESULTS/"sky130_area_mapped/radix4_a/area.json"
        reference_run = reference.parents[1]/"run.json"
        if reference.exists() and reference_run.exists():
            previous_run = json.loads(reference_run.read_text())
            if previous_run["liberty_sha256"] == digest(liberty):
                previous = json.loads(reference.read_text())["total_um2"]
                area = summary["production"]
                area["delta_um2"] = area["total_um2"] - previous
                area["delta_pct"] = area["delta_um2"] * 100 / previous
    for label, values in summary.items():
        if original is not None:
            values["increase_pct"] = (values["total_um2"]-original)*100/original
        (out/label/"area.json").write_text(json.dumps(values, indent=2)+"\n")
    (out/"area_summary.json").write_text(json.dumps(summary, indent=2)+"\n")


if __name__ == "__main__":
    main()
