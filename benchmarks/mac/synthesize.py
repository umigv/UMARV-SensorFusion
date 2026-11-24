"""Run identical generic and optional SKY130 mapping and OpenSTA for all variants."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import shutil
import sys
import subprocess

sys.dont_write_bytecode = True
from benchmark import ARCHES, HERE, ROOT

RTL = [path for path in sorted(HERE.glob("*.sv")) if not path.name.startswith("test_")]


def settings():
    yield "baseline", "fp32_mac_serial_legacy", None, None
    for arch in ARCHES:
        yield arch, f"fp32_mac_{arch}", 4, 1
    yield "a_iterative_shift", "fp32_mac_a", 4, 0
    for topology in range(4):
        yield f"a_adder_{topology}", "fp32_mac_a", topology, 1


def liberty_areas(path):
    data = path.read_text()
    cells = list(re.finditer(r'\bcell\s*\(\s*"?([^"\s)]+)"?\s*\)\s*\{', data))
    areas = {}
    for i, cell in enumerate(cells):
        body = data[cell.end():cells[i+1].start() if i + 1 < len(cells) else len(data)]
        area = re.search(r'\barea\s*:\s*([\d.eE+-]+)', body)
        if area:
            areas[cell[1]] = (float(area[1]), bool(re.search(r'\b(?:ff|latch|ff_bank|latch_bank)\s*\(', body)))
    return areas


def run(log, argv):
    with log.open("w") as handle:
        subprocess.run(argv, stdout=handle, stderr=subprocess.STDOUT, check=True, cwd=ROOT)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--liberty", type=Path, help="Uncompressed SKY130 Liberty file")
    parser.add_argument("--out", type=Path, default=HERE / "results/synthesis")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    out = args.out.resolve()
    yosys = shutil.which("yosys")
    sta = shutil.which("sta")
    liberty = args.liberty.resolve(strict=True) if args.liberty else None
    status = {"yosys": yosys, "opensta": sta, "liberty": str(liberty) if liberty else None,
              "openroad": shutil.which("openroad"), "tiny_tapeout": shutil.which("tt_tool")}
    (out / "availability.json").write_text(json.dumps(status, indent=2) + "\n")
    if not yosys:
        parser.exit(1, "Yosys unavailable. No technology synthesis performed.\n")
    summary = {}
    for name, top, topology, fast in settings():
        target = out / name
        target.mkdir(exist_ok=True)
        sources = RTL if topology is not None else [ROOT / f"{top}.sv"]
        lines = ["read_verilog -sv " + " ".join(f'"{p}"' for p in sources)]
        if topology is not None:
            lines.append(f"chparam -set TOPOLOGY {topology} -set FAST_SHIFT {fast} {top}")
        lines += [f"hierarchy -check -top {top}", f"synth -flatten -noabc -top {top}",
                  "check -assert", f'write_json "{target / "generic.json"}"', "stat"]
        if liberty:
            lines += [f'dfflibmap -liberty "{liberty}"',
                      f'abc -D 20000 -liberty "{liberty}"', "clean",
                      f'stat -liberty "{liberty}"', f'write_json "{target / "mapped.json"}"',
                      f'write_verilog -noattr "{target / "mapped.v"}"']
        script = target / "run.ys"
        script.write_text("\n".join(lines) + "\n")
        run(target / "yosys.log", [yosys, "-s", script])
        generic_cells = json.loads((target / "generic.json").read_text())["modules"][top]["cells"]
        row = summary[name] = {"generic_cells": len(generic_cells)}
        if liberty:
            types = Counter(
                cell["type"] for cell in
                json.loads((target / "mapped.json").read_text())["modules"][top]["cells"].values()
                if cell["type"] != "$scopeinfo"
            )
            areas = liberty_areas(liberty)
            if types.keys() - areas.keys():
                raise RuntimeError(f"{name}: unmapped cells {types.keys() - areas.keys()}")
            seq = sum(count * areas[cell][0] for cell, count in types.items() if areas[cell][1])
            comb = sum(count * areas[cell][0] for cell, count in types.items() if not areas[cell][1])
            row.update(cells=sum(types.values()), combinational_um2=comb,
                       sequential_um2=seq, total_um2=comb+seq)
            if sta:
                tcl = target / "timing.tcl"
                tcl.write_text(f'''read_liberty {{{liberty}}}
read_verilog {{{target / "mapped.v"}}}
link_design {top}
create_clock -name clk -period 20 [get_ports clk]
set_input_delay -clock clk 2 [get_ports {{start clear last a* b*}}]
set_output_delay -clock clk 2 [all_outputs]
set_false_path -from [get_ports rst_n]
report_checks -path_delay max -group_path_count 5
report_worst_slack -max
report_clock_min_period
exit
''')
                run(target / "sta.log", [sta, "-no_init", tcl])
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")


if __name__ == "__main__":
    main()
