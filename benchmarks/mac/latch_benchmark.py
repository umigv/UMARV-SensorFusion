"""Measure a ROUND-to-PACK latch bank without editing the production MAC."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

from benchmark import RESULTS, ROOT, TESTBENCH, distribution, fixtures
from sky130_area import COMMON_TOP, digest, liberty_dffs, wrapper
from synthesize import liberty_areas

PRODUCTION = ROOT / "fp32_mac.sv"
HYBRID = "fp32_mac_round_latch"
LATCH_CELL = "sky130_fd_sc_hd__dlxtp_1"


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise ValueError(f"Expected one production anchor: {old!r}")
    return source.replace(old, new, 1)


def generate(out):
    source = replace_once(PRODUCTION.read_text(), "module fp32_mac (", f"module {HYBRID} (")
    for line in ("rounded <= '0;", "discarded <= 1'b0;", "biased_exp <= '0;",
                 "rounded <= alu_result[24:0];", "discarded <= |product_mag[26:0];",
                 "biased_exp <= product_exp[7:0] + 8'd127;"):
        anchor = next(row for row in source.splitlines() if row.strip() == line)
        source = replace_once(source, anchor + "\n", "")
    source = replace_once(source, "    always_ff @(posedge clk or negedge rst_n) begin", """    // Capture the enable on the opposite phase so the data gate stays
    // stable while open. PACK is reached only after a fresh ROUND capture.
    logic round_enable;
    always_latch begin
        if (clk)
            round_enable <= state == ROUND;
    end

    always_latch begin
        if (!clk && round_enable) begin
            rounded <= alu_result[24:0];
            discarded <= |product_mag[26:0];
            biased_exp <= product_exp[7:0] + 8'd127;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin""")
    path = out / f"{HYBRID}.sv"
    path.write_text("// Generated from production fp32_mac.sv by latch_benchmark.py.\n" + source)
    return path


def run(log, command, timeout=300):
    with log.open("w") as output:
        subprocess.run(command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT,
                       check=True, timeout=timeout)


def simulate(arm, source, top, temp, bench, expanded, directed, expected):
    sources = [source]
    if top != "fp32_mac":
        adapter = temp / "adapter.sv"
        wrapper(adapter, top, None, simulation=True)
        sources.append(adapter)
    executable = temp / f"{top}.vvp"
    run(arm/"compile.log", ["iverilog", "-g2012", "-Wall", "-s", "tb_fp32_mac",
                           "-o", executable, *sources, bench])
    run(arm/"protocol.log", ["vvp", executable, f"+vectors={directed}"])
    log = arm/"expanded.log"
    run(log, ["vvp", executable, f"+vectors={expanded}", "+latency"])
    protocol = (arm/"protocol.log").read_text()
    if "PASS: 10112 reference MAC operations" not in protocol:
        raise RuntimeError(f"{arm.name}: incomplete directed suite")
    cycles = []
    passed = False
    with log.open() as trace:
        for line in trace:
            if line.startswith("LAT MAC "):
                fields = line.split()
                if int(fields[2]) != len(cycles):
                    raise RuntimeError(f"{arm.name}: out-of-order trace")
                cycles.append(int(fields[-1]))
            elif line.startswith(f"PASS: {expected} reference MAC operations"):
                passed = True
    if not passed or len(cycles) != expected or max(cycles) != 22:
        raise RuntimeError(f"{arm.name}: incorrect completion count or latency")
    return cycles


def map_arm(arm, source, top, liberty, cells, dffs):
    adapter = arm/"top.sv"
    wrapper(adapter, top, None)
    techmap = arm.parent/"latch_map.v"
    script = arm/"map.ys"
    script.write_text(f"""read_liberty -lib -ignore_miss_func "{liberty}"
read_verilog -sv "{source}" "{adapter}"
hierarchy -check -top {COMMON_TOP}
synth -flatten -noabc -top {COMMON_TOP}
techmap -map "{techmap}"
dfflibmap -liberty "{liberty}"
abc -D 20000 -liberty "{liberty}"
clean -purge
check -assert
stat -liberty "{liberty}"
write_json "{arm/'mapped.json'}"
write_verilog -noattr -noexpr "{arm/'mapped.v'}"
""")
    run(arm/"yosys.log", ["yosys", "-s", script])
    netlist = json.loads((arm/"mapped.json").read_text())
    count = Counter(cell["type"] for cell in netlist["modules"][COMMON_TOP]["cells"].values()
                    if cell["type"] != "$scopeinfo")
    if missing := count.keys() - cells.keys():
        raise RuntimeError(f"{arm.name}: unmapped cells {missing}")
    if unexpected := {cell for cell in count if cells[cell][1] and cell not in dffs | {LATCH_CELL}}:
        raise RuntimeError(f"{arm.name}: unexpected sequential cells {unexpected}")
    sequential = sum(n*cells[c][0] for c, n in count.items() if cells[c][1])
    comb = sum(n*cells[c][0] for c, n in count.items() if not cells[c][1])
    return {"cells": sum(count.values()), "dffs": sum(count[c] for c in dffs),
            "latches": count[LATCH_CELL], "total_um2": comb+sequential,
            "combinational_um2": comb, "sequential_um2": sequential,
            "latch_um2": count[LATCH_CELL]*cells[LATCH_CELL][0], "by_cell": dict(count)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--liberty", required=True, type=Path)
    parser.add_argument("--out", type=Path, default=RESULTS/"latch_benchmark")
    args = parser.parse_args()
    out, lib = args.out.resolve(), args.liberty.resolve(strict=True)
    out.mkdir(parents=True, exist_ok=True)
    production_hash = digest(PRODUCTION)
    hybrid = generate(out)
    cells, dffs = liberty_areas(lib), liberty_dffs(lib)
    if LATCH_CELL not in cells or not cells[LATCH_CELL][1] or LATCH_CELL in dffs:
        raise ValueError("Expected SKY130 HD positive-enable latch")
    (out/"latch_map.v").write_text(f"""module \\$_DLATCH_P_ (input E, D, output Q);
    {LATCH_CELL} _TECHMAP_REPLACE_ (.GATE(E), .D(D), .Q(Q));
endmodule
""")
    designs = {"production": (PRODUCTION, "fp32_mac"), "hybrid": (hybrid, HYBRID)}
    report = {"production_sha256": production_hash, "liberty": str(lib), "liberty_sha256": digest(lib),
              "yosys_version": subprocess.check_output(["yosys", "-V"], text=True).strip(),
              "physical_timing": "unverified",
              "timing_tools": {tool: shutil.which(tool) for tool in ("sta", "openroad")}, "designs": {}}
    with tempfile.TemporaryDirectory() as folder, ThreadPoolExecutor(4) as pool:
        temp = Path(folder)
        expanded, directed = temp/"expanded.txt", temp/"directed.txt"
        expected = len(fixtures(expanded, 100_000))
        from test_fp32_mac import operations, write_vectors
        write_vectors(directed, operations())
        bench = temp/TESTBENCH.name
        source, edits = re.subn(r'(?m)^([ \t]*)#\d+;(?=\n[ \t]*\$fatal\(1, "Simulation timeout"\);)',
                                r'\g<1>#1000000000;', TESTBENCH.read_text())
        if edits != 1:
            raise ValueError("Shared testbench watchdog not found")
        bench.write_text(source)
        mappings, replays = {}, {}
        for name, (rtl, top) in designs.items():
            arm = out/name
            arm.mkdir(exist_ok=True)
            mappings[name] = pool.submit(map_arm, arm, rtl, top, lib, cells, dffs)
            replays[name] = pool.submit(simulate, arm, rtl, top, temp, bench, expanded, directed, expected)
        for name in designs:
            report["designs"][name] = area = mappings[name].result()
            (out/name/"area.json").write_text(json.dumps(area, indent=2)+"\n")
        baseline, hybrid = (report["designs"][name] for name in designs)
        assert baseline["latches"] == 0 and hybrid["latches"] == 35
        assert baseline["dffs"] - hybrid["dffs"] == 34
        print(f"Mapped area: {baseline['total_um2']:.4f} → {hybrid['total_um2']:.4f} µm²", flush=True)
        traces = {name: replays[name].result() for name in designs}
        assert traces["production"] == traces["hybrid"]
        assert digest(PRODUCTION) == production_hash
        for name, trace in traces.items():
            report["designs"][name]["latency_cycles"] = distribution(trace)
        report["identical_latency_traces"] = True
        report["delta_um2"] = hybrid["total_um2"] - baseline["total_um2"]
        report["delta_pct"] = report["delta_um2"]*100 / baseline["total_um2"]
    (out/"summary.json").write_text(json.dumps(report, indent=2)+"\n")
    print(f"Both designs passed {expected:,} expanded and 10,112 protocol requests; cycle traces identical.")


if __name__ == "__main__":
    main()
