"""Run the unchanged FP32 MAC testbench against baseline and experimental tops."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import gzip
import json
import math
from pathlib import Path
import random
import re
from statistics import mean, median
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT / "tests"))
from test_fp32_mac import fma, magnitude, operations

ARCHES = "abcdefg"
MULTIPLY = dict(zip(ARCHES, (13, 9, 6, 3, 2, 1, 1), strict=True))
SOURCES = sorted(path for path in HERE.glob("*.sv") if not path.name.startswith("test_"))
TESTBENCH = ROOT / "tests/tb_fp32_mac.sv"
RESULTS = HERE / "results"
ONE = 0x3F800000


def normalized(word):
    return 0x00800000 <= magnitude(word) < 0x7F800000


def requests(amount):
    # Retain the existing directed/reference workload, including reset checks
    # and the final known sum expected by the original bench.
    existing = list(operations(timing=True))
    yield from existing[:-1024]
    yield 1, 1, 0x7FC00000, ONE
    rng = random.Random(0xDA44A)
    for i in range(amount):
        kind = i % 5
        clear = int(i % 7 == 0)
        a, b = [rng.getrandbits(32) for _ in range(2)]
        if kind == 1:
            a = (a & 0x807FFFFF) | (rng.randrange(112, 144) << 23)
            b = (b & 0x807FFFFF) | (rng.randrange(112, 144) << 23)
        elif kind == 2:
            a = (a & 0x807FFFFF) | 1
        elif kind == 3:
            a = (a & 0x807FFFFF) | 0x00800000
            b = (b & 0x807FFFFF) | 0x7F000000
        elif kind == 4:
            a = (a & 0x807FFFFF) | 0x3F800000
            b = (b & 0x807FFFFF) | 0x3F800000
            product, _ = fma(a, b, 0)
            yield 1, 0, product ^ 0x80000000, ONE
            clear = 0
        yield clear, 1, a, b
    yield from existing[-1024:]


def fixtures(path, amount):
    c = result = 0
    classes = []
    with path.open("w") as stream:
        for clear, last, a, b in requests(amount):
            c = 0 if clear else c
            classes.append(1 if normalized(a) and normalized(b) and magnitude(c) == 0 else
                           2 if normalized(a) and normalized(b) and normalized(c) else 0)
            c, flags = fma(a, b, c)
            if last:
                result = c
            stream.write(f"{clear} {last} {a:08x} {b:08x} {result:08x} {c:08x} {flags:02x}\n")
    return classes


def distribution(values):
    values = sorted(values)
    return {"n": len(values), "min": values[0], "mean": mean(values),
            "median": median(values), "p95": values[math.ceil(len(values)*.95)-1],
            "p99": values[math.ceil(len(values)*.99)-1], "max": values[-1]}


def measure(job, directory, vectors, classes, save):
    name, arch, topology, fast = job
    adapter = directory / f"{name}_adapter.sv"
    files = [ROOT / "fp32_mac.sv"]
    if name != "production":
        top = f"fp32_mac_{arch}" if arch else "fp32_mac_serial_legacy"
        params = f" #(.TOPOLOGY({topology}), .FAST_SHIFT({fast}))" if arch else ""
        instance_path = "variant.core" if arch else "variant"
        ports = '''
    input wire clk, rst_n, start, clear, last,
    input wire [31:0] a, b,
    output wire busy, done, result_valid,
    output wire [31:0] result,
    output wire [4:0] exception_flags
'''
        adapter.write_text(f"""`timescale 1ns/1ps
module fp32_mac ({ports});
    {top}{params} variant (.*);
    wire [31:0] accumulator = {instance_path}.accumulator;
endmodule
""")
        files = [*(SOURCES if arch else [ROOT / f"{top}.sv"]), adapter]
    sim = directory / f"{name}.vvp"
    # Same bench and assertions. Elapse a larger global simulation window for
    # the expanded vector set while retaining its per-request timeout.
    tb = directory / "tb_fp32_mac.sv"
    subprocess.run(["iverilog", "-g2012", "-Wall", "-s", "tb_fp32_mac", "-o", sim,
                    *files, tb], check=True, cwd=ROOT)
    output = directory / f"{name}.log"
    with output.open("w") as stream:
        status = subprocess.run(["vvp", sim, f"+vectors={vectors}", "+latency"],
                                stdout=stream, text=True, timeout=900)
    data, subgroups, witness, worst = [], [[], [], []], "", -1
    with output.open() as source:
        for line in source:
            if line.startswith("LAT "):
                _, _, index, *_, cycles = line.split()
                i, cycle = int(index), int(cycles)
                if i != len(data):
                    raise RuntimeError(f"Misordered {name} at {i}")
                data.append(cycle)
                subgroups[classes[i]].append(cycle)
                if cycle > worst:
                    worst, witness = cycle, line.strip()
            elif "FATAL" in line or line.startswith("PASS"):
                print(f"{name}: {line.strip()}", flush=True)
    status.check_returncode()
    if len(data) != len(classes):
        raise RuntimeError(f"Missing {name} results")
    record = summarize_run(job, data, subgroups, witness)
    if save:
        with gzip.open(RESULTS / f"{name}.cycles.gz", "wt", compresslevel=6) as stream:
            stream.write("\n".join(map(str, data)) + "\n")
    return name, record


def summarize_run(job, data, subgroups, witness):
    name, arch, topology, fast = job
    bound = 22 if name == "production" else 177 if not arch else MULTIPLY[arch] + (9 if fast else 153)
    worst = max(data)
    if worst != bound:
        raise AssertionError((name, worst, bound))
    record = distribution(data)
    record.update({"bound": bound, "witness": witness, "first": distribution(subgroups[1]),
                   "continued": distribution(subgroups[2]), "histogram": dict(sorted(Counter(data).items())),
                   "architecture": "a" if name == "production" else arch or "baseline",
                   "topology": topology, "fast_shift": bool(fast)})
    return record


def collect(job, classes, vectors):
    """Rebuild summaries from successful raw streams, including resumed batches."""
    name = job[0]
    with gzip.open(RESULTS / f"{name}.cycles.gz", "rt") as source:
        data = [int(line) for line in source]
    if len(data) != len(classes):
        raise ValueError(f"{name}: expected {len(classes)} results, found {len(data)}")
    worst = max(data)
    index = data.index(worst)
    previous = "00000000"
    with vectors.open() as source:
        for i, line in enumerate(source):
            clear, _, a, b, _, accumulator, flags = line.split()
            if i == index:
                c = "00000000" if clear == "1" else previous
                witness = f"LAT MAC {index} {a} {b} {c} {accumulator} {flags} {worst}"
                break
            previous = accumulator
    groups = [[value for value, kind in zip(data, classes) if kind == k] for k in range(3)]
    return name, summarize_run(job, data, groups, witness)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--random", type=int, default=100_000)
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--only", nargs="+")
    parser.add_argument("--quick", action="store_true", help="Do not overwrite published results")
    parser.add_argument("--collect", action="store_true", help="Summarize successful saved cycle streams")
    args = parser.parse_args()
    jobs = [("baseline", "", 4, 0), *((a, a, 4, 1) for a in ARCHES),
            ("a_iterative_shift", "a", 4, 0)]
    if args.only:
        jobs.append(("production", "", 4, 1))
        jobs = [job for job in jobs if job[0] in args.only]
    if not jobs:
        parser.error("no matching architecture")
    with tempfile.TemporaryDirectory() as temp, ThreadPoolExecutor(max_workers=args.jobs) as executor:
        path = Path(temp)
        bench, changes = re.subn(
            r'(?m)^([ \t]*)#\d+;(?=\n[ \t]*\$fatal\(1, "Simulation timeout"\);)',
            r'\g<1>#1000000000;', TESTBENCH.read_text(),
        )
        if changes != 1:
            raise RuntimeError("Could not identify the shared bench's global simulation watchdog")
        (path / TESTBENCH.name).write_text(bench)
        vectors = path / "vectors.txt"
        classes = fixtures(vectors, args.random)
        print(f"Benchmark: {len(classes)} checked commands per architecture, {args.random} randomized targets", flush=True)
        if args.collect:
            result = dict(collect(job, classes, vectors) for job in jobs)
        else:
            tasks = [executor.submit(measure, job, path, vectors, classes, not args.quick) for job in jobs]
            result = dict(task.result() for task in tasks)
    if not args.quick:
        destination = RESULTS / ("summary.json" if not args.only else "summary_" + "_".join(args.only) + ".json")
        destination.write_text(json.dumps(result, indent=2) + "\n")
    for name, row in result.items():
        print(name, {key: row[key] for key in ("min", "mean", "median", "p95", "p99", "max", "bound")})


if __name__ == "__main__":
    main()
