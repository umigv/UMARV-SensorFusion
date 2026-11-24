"""Replay covariance microcode on A/D/F and model the rest of each SR-UKF phase."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
from pathlib import Path
from statistics import mean
import subprocess

import tempfile

from benchmark import ROOT, SOURCES, RESULTS

from cholesky_benchmark import (
    Case, Lane, N, ROW_END, cases, cholesky, fp32, matrix_covariance, simulate,
)

ARCHS = "adf"
SAMPLES = 8
OUT = RESULTS / "arch_a_feasibility"
DIV_MEAN, ROOT_MEAN = 33.792, 31.045


@dataclass(frozen=True)
class Work:
    clear: int
    continued: int
    div: int
    root: int
    overhead: int

    def __add__(self, other):
        return Work(*(a+b for a, b in zip(vars(self).values(), vars(other).values(), strict=True)))

    def __mul__(self, n):
        return Work(*(x*n for x in vars(self).values()))

    def cycles(self, clear, continued, div, root):
        return self.clear*clear + self.continued*continued + self.div*div + self.root*root + self.overhead


ORIGINAL = Work(25, 25, 20, 5, 345)
ROTATION = Work(35, 25, 5, 5, 365)
PRED_OTHER = Work(330, 275, 0, 0, 2545)
CORR_OTHER = Work(79, 174, 60, 10, 2125)


def posterior_case(r, sample):
    """Supply illustrative posterior downdate vectors from a binary64 H=[I4 0] solve."""
    p = matrix_covariance(r)
    # Positive measurement noise at the repository's state scales, varied by sample.
    noise_std = (0.12, 0.13, 0.025, 0.012)
    innovation = [[p[i][j] + (noise_std[i]**2 * (1 + sample/8) if i == j else 0)
                   for j in range(4)] + [0.0] for i in range(4)]
    innovation.append([0.0, 0.0, 0.0, 0.0, 1.0])
    upper = cholesky(innovation)
    assert upper is not None
    u = [[0.0]*4 for _ in range(N)]
    for i in range(N):
        for j in range(4):
            u[i][j] = (p[i][j] - sum(u[i][k]*upper[k][j] for k in range(j))) / upper[j][j]
    return Case("posterior", r, [(-1, tuple(fp32(u[i][j]) for i in range(N))) for j in range(4)])


def make_vectors(path):
    targets = {"central update": [], "central downdate": [], "posterior": []}
    for case in cases():
        if case.group.startswith("11-row "):
            key = case.group.removeprefix("11-row ")
            if len(targets[key]) < SAMPLES:
                targets[key].append(case)
        if case.group == "random update" and len(targets["posterior"]) < SAMPLES:
            targets["posterior"].append(posterior_case(case.r, len(targets["posterior"])))
        if all(len(group) == SAMPLES for group in targets.values()):
            break

    labels = []
    with path.open("w") as output:
        for family, group in targets.items():
            for sample, case in enumerate(group, start=1):
                choices = ("original",) if family == "posterior" else (
                    ("original", "rotation", "hybrid") if family == "central downdate"
                    else ("original", "rotation"))
                for mode in choices:
                    if mode != "hybrid":
                        lane, rows, failures = simulate(case, mode == "rotation", True)
                        if failures or len(rows) != len(case.steps):
                            raise RuntimeError((family, sample, mode, failures))
                    else:
                        lane = Lane(case.r, trace=True)
                        for sign, vector in case.steps:
                            lane.update(vector, sign, rotation=sign > 0)
                            lane.trace.append((ROW_END, 0, 0, 0, 0, 0, 0))
                    output.write(f"6 0 0 0 {len(labels):08x} 00 0\n")
                    labels.append((family, sample, mode, len(case.steps)))
                    for op, a, b, dst, value, flags, reads in lane.trace:
                        output.write(f"{op} {a} {b} {dst} {value:08x} {flags:02x} {reads}\n")
        output.write("6 0 0 0 ffffffff 00 0\n")
    return labels


def replay(arch, temp, vectors, labels):
    adapter = temp / f"adapter_{arch}.sv"
    adapter.write_text(f"""`timescale 1ns/1ps
module fp32_mac (
    input wire clk, rst_n, start, clear, last,
    input wire [31:0] a, b,
    output wire busy, done, result_valid,
    output wire [31:0] result,
    output wire [4:0] exception_flags
);
    fp32_mac_{arch} variant (.*);
    wire [31:0] accumulator = variant.core.accumulator;
endmodule
""")
    sim = temp / f"sim_{arch}"
    subprocess.run(["iverilog", "-g2012", "-Wall", "-s", "tb_cholesky_experiment",
                    "-o", sim, *SOURCES, adapter, ROOT/"fp32_divsqrt.sv",
                    ROOT/"tests/tb_cholesky_experiment.sv"], cwd=ROOT, check=True)
    with (OUT / f"{arch}_replay.log").open("w") as log:
        subprocess.run(["vvp", sim, f"+vectors={vectors}"], stdout=log, check=True, timeout=240)
    data, row_cycles = {}, {}
    for line in (OUT / f"{arch}_replay.log").read_text().splitlines():
        if line.startswith("RUN "):
            _, run, latency, requests = line.split()
            data[int(run)] = (int(latency), int(requests))
        elif line.startswith("ROW "):
            _, run, row, latency = line.split()
            run_rows = row_cycles.setdefault(int(run), [])
            if int(row) != len(run_rows) + 1:
                raise RuntimeError(f"{arch}: unexpected row marker: {line}")
            run_rows.append(int(latency))
    if data.keys() != set(range(len(labels))):
        raise RuntimeError(f"{arch}: incomplete RTL replay")
    for i, (_, _, mode, rows) in enumerate(labels):
        expected = 75*rows if mode == "original" else 70*rows + (5 if mode == "hybrid" else 0)
        if len(row_cycles[i]) != rows or (sum(row_cycles[i]), expected) != data[i]:
            raise RuntimeError(f"{arch}: row or request counts disagree for sequence {i}")
    print(f"{arch}: {sum(n for _, n in data.values())} bit-exact requests, {len(data)} sequences", flush=True)
    return [{"family": family, "sample": sample, "mode": mode, "rows": rows, "cycles": data[i][0],
             "requests": data[i][1]} for i, (family, sample, mode, rows) in enumerate(labels)]


def estimate(records):
    source = json.loads((RESULTS/"schedule.json").read_text())
    totals = {
        "central update/original": PRED_OTHER + ORIGINAL*11,
        "central update/rotation": PRED_OTHER + ROTATION*11,
        "central downdate/original": PRED_OTHER + ORIGINAL*11,
        "central downdate/rotation": PRED_OTHER + ROTATION*11,
        "central downdate/hybrid": PRED_OTHER + ROTATION*10 + ORIGINAL,
        "posterior/original": CORR_OTHER + ORIGINAL*4,
    }
    report = {"work": {key: vars(work) for key, work in totals.items()}, "architectures": {}}
    for arch, data in records.items():
        means = source[arch]["clear_mean"], source[arch]["continue_mean"], DIV_MEAN, ROOT_MEAN
        clear_bound = source[arch]["clear_bound"]
        row = report["architectures"][arch] = {}
        for key, work in totals.items():
            family, mode = key.split("/")
            subtotal = CORR_OTHER if family == "posterior" else PRED_OTHER
            samples = [r["cycles"] for r in data if r["family"] == family and r["mode"] == mode]
            assert len(samples) == SAMPLES
            expected = subtotal.cycles(*means) + mean(samples)
            bound = work.cycles(clear_bound, clear_bound+3, 81, 54)
            row[key] = {
                "all_shortcut_floor": work.cycles(1, 1, 1, 1),
                "recurrence_floor": work.cycles(clear_bound, clear_bound, 31, 31),
                "all_primitive_proxy": work.cycles(*means),
                "measured_kernel_min_mean_max": [min(samples), mean(samples), max(samples)],
                "hybrid_expected": expected,
                "hybrid_expected_margin": 62500-expected,
                "known_bound": bound,
                "known_bound_margin": 62500-bound,
            }
            print(f"{arch} {key}: kernel mean={mean(samples):.3f}, "
                  f"hybrid proxy={expected:.3f}, bound={bound}, margin={62500-bound}")
    return report


def main():
    OUT.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as directory, ThreadPoolExecutor(max_workers=3) as pool:
        temp = Path(directory)
        vectors = temp / "vectors"
        labels = make_vectors(vectors)
        jobs = {arch: pool.submit(replay, arch, temp, vectors, labels) for arch in ARCHS}
        records = {arch: job.result() for arch, job in jobs.items()}
    (OUT/"replay.json").write_text(json.dumps(records, indent=2)+"\n")
    (OUT/"estimate.json").write_text(json.dumps(estimate(records), indent=2)+"\n")


if __name__ == "__main__":
    main()
