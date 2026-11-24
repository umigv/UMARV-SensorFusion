"""Replay the planned FP32 phase arithmetic on the serial-fast and Booth MACs."""
import argparse
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import random
from statistics import mean
import subprocess
import tempfile

from sky130_area import CASES, RESULTS, ROOT, design_sources, isolation, wrapper
from cholesky_benchmark import (
    DIV, MAC_CLEAR, MAC_CONTINUE, MOVE, N, RUN, SQRT, U,
    Lane, cholesky, factor_error, matrix_covariance, r_slot, value, word,
)

OUT = RESULTS / "fast_shift_feasibility"
RF_WORDS = 2048
BUDGET = 62_500
SAMPLES = 8
MODES = ("original", "hybrid", "rotation")
OPS = ("clear", "continue", "div", "root", "move")


class Phase(Lane):
    def __init__(self, factor):
        super().__init__(factor, trace=True)
        self.rf.extend([0] * (RF_WORDS - len(self.rf)))
        self.next_slot = 29
        self.constants = {}
        self.host = Counter()
        self.stages = {}

    def slot(self):
        self.next_slot += 1
        return self.next_slot - 1

    def constant(self, number):
        bits = word(number)
        if bits == 0:
            return 0
        if bits == word(1):
            return 1
        if bits not in self.constants:
            slot = self.constants[bits] = self.slot()
            self.load(slot, bits)
        return self.constants[bits]

    def input_value(self, number):
        slot = self.slot()
        self.load(slot, word(number))
        return slot

    def number(self, slot):
        return value(self.operand(slot))

    def command(self, op, a, b=0, dst=None):
        dst = self.slot() if dst is None else dst
        self.issue(op, a, b, dst)
        return dst

    def product(self, a, b, dst=None):
        return self.command(MAC_CLEAR, a, b, dst)

    def fused(self, seed, a, b):
        self.issue(MAC_CLEAR, seed, 1)
        return self.command(MAC_CONTINUE, a, b)

    def dot(self, pairs, seed=None):
        if seed is not None:
            pairs = [(seed, 1), *pairs]
        for i, (a, b) in enumerate(pairs):
            dst = self.slot() if i == len(pairs)-1 else 0
            self.issue(MAC_CLEAR if i == 0 else MAC_CONTINUE, a, b, dst)
        return dst

    def host_command(self, kind, *numbers):
        self.host[kind] += 1
        slots = tuple(self.slot() for _ in numbers)
        for dst, number in zip(slots, numbers, strict=True):
            self.load(dst, word(number))
        return slots

    def wrap(self, slot):
        return self.host_command("wrap", math.remainder(self.number(slot), 2*math.pi))[0]

    def stage(self, name):
        prior = sum(self.stages.values(), Counter())
        self.stages[name] = self.counts - prior


def prediction(state, initial, process, delta, mode, central_sign):
    lane = Phase(process)
    center = [lane.input_value(x) for x in state]
    prior = [[lane.input_value(initial[i][j]) if j >= i else 0 for j in range(N)]
             for i in range(N)]
    gamma, dt, half = map(lane.constant, (2, 0.0025, 0.5))
    assert -(1 << 27) <= delta < (1 << 27)
    converted, = lane.host_command("convert", delta)
    # Fixture calibration of 50 micrometres/count over the 2.5 ms epoch.
    wheel = lane.product(converted, lane.constant(0.00005 / 0.0025))
    lane.stage("wheel")
    cov_target = matrix_covariance(lane.factor())
    # Outer sigma points precede the center so the central rank-one row is last.
    points = [(i, s) for i in range(N) for s in (1, -1)] + [(N, 1)]
    def propagate():
        sigma, propagated = [], []
        for row, sign in points:
            point = center[:]
            for j in range(row, N):
                point[j] = lane.fused(center[j], sign * gamma, prior[row][j])
            sigma.append(point)
        for x, y, heading, omega, bias in sigma:
            velocity = lane.fused(wheel, -bias, 1)
            distance, turn = lane.product(velocity, dt), lane.product(omega, dt)
            midpoint = lane.wrap(lane.fused(heading, turn, half))
            sine, cosine = lane.host_command("sincos", math.sin(lane.number(midpoint)),
                                            math.cos(lane.number(midpoint)))
            lift = lane.fused(heading, turn, 1)
            lane.wrap(lift)
            propagated.append([lane.fused(x, distance, cosine), lane.fused(y, distance, sine),
                               lift, omega, bias])
        return propagated

    first = propagate()
    lane.stage("first_pass")
    weight = [lane.constant(0.125)]*10 + [lane.constant(-0.25)]
    average = [lane.dot([(weights, row[j]) for weights, row in zip(weight, first, strict=True)])
               for j in range(N)]
    wrapped_heading = lane.wrap(average[2])
    lane.stage("mean")
    second = propagate()
    lane.stage("second_pass")
    for i, row in enumerate(second):
        sign = 1 if i < 10 else central_sign
        scale = lane.constant(math.sqrt(0.125 if i < 10 else 1.75))
        for j in range(N):
            deviation = lane.fused(row[j], -average[j], 1)
            lane.product(deviation, scale, dst=U+j)
        vector = [lane.number(U+j) for j in range(N)]
        for j in range(N):
            for k in range(N):
                cov_target[j][k] += sign * vector[j] * vector[k]
        lane.update_loaded(sign, mode == "rotation" or (mode == "hybrid" and sign > 0))
    lane.stage("deviations_and_factor")
    average[2] = wrapped_heading
    return lane, [lane.number(s) for s in average], lane.factor(), covariance_error(lane, cov_target)


def covariance_error(lane, target):
    if cholesky(target) is None:
        raise RuntimeError("Reference covariance is not positive definite")
    return factor_error(lane.factor(), target)[0]


def correction(state, predicted, observation, noise_std):
    lane = Phase(predicted)
    states = [lane.input_value(x) for x in state]
    measurements = [lane.input_value(x) for x in observation]
    innovations = [lane.fused(y, -x, 1) for x, y in zip(states[:4], measurements, strict=True)]
    innovations[2] = lane.wrap(innovations[2])
    lane.stage("innovation")
    # Save the predicted factor before overwriting its upper 4×4 for innovation QR.
    prior = [[lane.command(MOVE, r_slot(i, j)) if i <= j else 0
              for j in range(N)] for i in range(N)]
    for i, noise in enumerate(noise_std):
        for j in range(i, 4):
            lane.command(MOVE, lane.constant(noise) if j == i else 0, dst=U+j)
        lane.update_loaded(1, False, start=i, dimension=4)
    lane.stage("innovation_factor")
    forward = [[0]*4 for _ in range(N)]
    for i in range(N):
        for j in range(4):
            terms = [(prior[k][i], prior[k][j]) for k in range(4)]
            terms += [(-forward[i][k], r_slot(k, j)) for k in range(j)]
            forward[i][j] = lane.command(DIV, lane.dot(terms), r_slot(j, j))
    lane.stage("forward")
    gain = [[0]*4 for _ in range(N)]
    for i in range(N):
        for j in reversed(range(4)):
            terms = [(-gain[i][k], r_slot(j, k)) for k in range(j+1, 4)]
            gain[i][j] = lane.command(DIV, lane.dot(terms, seed=forward[i][j]), r_slot(j, j))
    lane.stage("backward")
    updated = [lane.dot(list(zip(row, innovations, strict=True)), seed=x)
               for row, x in zip(gain, states, strict=True)]
    lane.wrap(updated[2])
    lane.stage("state")
    for i in range(N):
        for j in range(i, N):
            lane.command(MOVE, prior[i][j], dst=r_slot(i, j))
    p = matrix_covariance(predicted)
    innovation = [[p[i][j] + (noise_std[i]**2 if i == j else 0) for j in range(4)]
                  + [0.0] for i in range(4)] + [[0.0]*4 + [1.0]]
    upper = cholesky(innovation)
    assert upper is not None
    reference_u = [[0.0]*4 for _ in range(N)]
    for i in range(N):
        for j in range(4):
            reference_u[i][j] = (p[i][j] - sum(reference_u[i][k]*upper[k][j]
                                              for k in range(j))) / upper[j][j]
    target = [[p[i][j] - sum(x*y for x, y in zip(reference_u[i], reference_u[j], strict=True))
               for j in range(N)] for i in range(N)]
    for col in range(4):
        for i in range(N):
            lane.command(MOVE, forward[i][col], dst=U+i)
        lane.update_loaded(-1, False)
    lane.stage("posterior")
    return lane, covariance_error(lane, target)


def scenarios(original_only=False):
    rng = random.Random(0x5F32)
    modes = ("original",) if original_only else MODES
    for sample in range(SAMPLES):
        state = [rng.uniform(-8, 8), rng.uniform(-8, 8), (-1)**sample * (math.pi-0.002),
                 rng.uniform(-1.8, 1.8), rng.uniform(-0.2, 0.2)]
        scales = (0.3, 0.32, 0.06, 0.028, 0.025)
        initial = [[0.0 if j < i else scales[j]*(1 if j == i else rng.uniform(-0.2, 0.2))
                    for j in range(N)] for i in range(N)]
        process = [[scales[j]*0.02 if i == j else 0.0 for j in range(N)] for i in range(N)]
        delta = (0, -17, 95, 34, 110, 51, -85, 62)[sample]
        for sign in (1, -1):
            for mode in modes:
                lane, predicted_state, factor, error = prediction(state, initial, process, delta, mode, sign)
                yield sample, f"{mode}/{sign:+}", lane, error
                if mode == "original" and sign == 1:
                    noise = (0.08, 0.08, 0.006, 0.012)
                    observation = [x + rng.gauss(0, sigma) for x, sigma in zip(predicted_state, noise)]
                    observation[2] = math.remainder(observation[2], 2*math.pi)
                    lane, error = correction(predicted_state, factor, observation, noise)
                    yield sample, "correction", lane, error


def count_commands(counter):
    return {name: counter[op] for op, name in enumerate(OPS)} | {
        name: counter[name] for name in ("reads", "writes")}


def make_vectors(path, original_only=False):
    metadata = []
    with path.open("w") as output:
        def emit(command):
            op, a, b, dst, bits, flags, reads = command
            output.write(f"{op} {a} {b} {dst} {bits:08x} {flags:02x} {reads}\n")

        for sample, label, lane, error in scenarios(original_only):
            emit((RUN, 0, 0, 0, len(metadata), 0, 0))
            for command in lane.trace:
                emit(command)
            counts = count_commands(lane.counts)
            expected = ((179, 274, 150, 30) if label == "correction" else
                        (605, 550, 220, 55) if label.startswith("original") else
                        (705, 550, 70, 55) if label == "hybrid/-1" else (715, 550, 55, 55))
            assert tuple(counts[k] for k in OPS[:4]) == expected, (label, counts)
            assert lane.host == ({"wrap": 2} if label == "correction" else
                                 {"wrap": 45, "sincos": 22, "convert": 1})
            assert error < 1e-4, (label, sample, error)
            metadata.append({"sample": sample, "scenario": label, "counts": counts,
                             "stages": {name: count_commands(c) for name, c in lane.stages.items()},
                             "host_commands": lane.host, "factor_relative_error": error})
        emit((RUN, 0, 0, 0, 0xffffffff, 0, 0))
    return metadata


def replay(arch, work, variant, vectors, metadata, out):
    adapter = work / f"{arch}_top.sv"
    if arch == "production":
        sources = [ROOT / "fp32_mac.sv"]
    else:
        name, param = CASES[arch]
        wrapper(adapter, name, param, simulation=True)
        sources = [*design_sources(name, variant), adapter]
    sim = work / arch
    path = out / f"{arch}.log"
    with path.open("w") as log:
        subprocess.run(["iverilog", "-g2012", "-s", "tb_cholesky_experiment",
                        f"-Ptb_cholesky_experiment.RF_WORDS={RF_WORDS}", "-o", sim,
                        *sources, ROOT/"fp32_divsqrt.sv",
                        ROOT/"tests/tb_cholesky_experiment.sv"],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
        subprocess.run(["vvp", sim, f"+vectors={vectors}", "+latency"],
                       stdout=log, stderr=subprocess.STDOUT, check=True, timeout=300)
    timings, runs = defaultdict(Counter), {}
    with path.open() as log:
        for line in log:
            if line.startswith("LAT "):
                run, op, latency, flags = map(int, line.split()[1:])
                timings[run][op, latency, flags] += 1
            elif line.startswith("RUN "):
                run, elapsed, requests = map(int, line.split()[1:])
                runs[run] = (elapsed, requests)
    assert runs.keys() == set(range(len(metadata)))
    records = []
    for i, entry in enumerate(metadata):
        counts = entry["counts"]
        elapsed, requests = runs[i]
        per_op = timings[i]
        assert requests == sum(per_op.values()) == sum(counts[k] for k in OPS[:4])
        execution = sum(latency*n for (_, latency, _), n in per_op.items())
        overhead = counts["reads"] + counts["writes"] + 2*(requests + counts["move"])
        assert execution + overhead == elapsed
        records.append({"replay_cycles": elapsed, "execution_cycles": execution,
                        "replay_overhead": overhead, "requests": requests,
                        "latency_and_flags": [[OPS[op], latency, flags, n]
                                              for (op, latency, flags), n in sorted(per_op.items())]})
    return records


def model(metadata, replayed):
    groups = defaultdict(list)
    for i, entry in enumerate(metadata):
        c = entry["counts"]
        host = entry["host_commands"]
        # Host wrap and conversion each use 1R/1W, sin/cos uses 1R/2W.
        host_control = 4*host.get("wrap", 0) + 4*host.get("convert", 0) + 5*host.get("sincos", 0)
        # Charge phase boundaries and a prediction copy of all 15 S_Q entries.
        extra = host_control + 2 + (0 if entry["scenario"] == "correction" else 60)
        for arch, records in replayed.items():
            run = records[i]
            clear = 30 if arch == "original_fast" else 19
            bound = sum(c[op]*limit for op, limit in zip(OPS[:4], (clear, clear+3, 81, 54)))
            groups[arch, entry["scenario"]].append((run["replay_cycles"], run["replay_cycles"] + extra,
                                                    bound + run["replay_overhead"] + extra))
    output = {arch: {} for arch in replayed}
    for (arch, scenario), samples in groups.items():
        replay, expected, bounds = zip(*samples, strict=True)
        estimate, bound = mean(expected), max(bounds)
        output[arch][scenario] = {
            "samples": len(samples), "replay_min_mean_max": [min(replay), mean(replay), max(replay)],
            "expected": estimate, "bound": bound,
            "expected_margin": BUDGET-estimate, "conservative_margin": BUDGET-bound,
            "expected_pct": estimate/BUDGET*100, "conservative_pct": bound/BUDGET*100,
        }
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--production", action="store_true",
                        help="Replay only original Cholesky with the production MAC")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    out = args.out or (RESULTS / "radix4_integration" / "phase_replay" if args.production else OUT)
    out = out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    architectures = ("production",) if args.production else ("original_fast", "radix4_a")
    with tempfile.TemporaryDirectory() as directory, ThreadPoolExecutor(2) as pool:
        work = Path(directory)
        vectors = work / "vectors"
        variant = None if args.production else isolation(work)
        metadata = make_vectors(vectors, args.production)
        jobs = {arch: pool.submit(replay, arch, work, variant, vectors, metadata, out)
                for arch in architectures}
        replayed = {arch: job.result() for arch, job in jobs.items()}
    (out/"replay.json").write_text(json.dumps({"scenarios": metadata, "architectures": replayed}, indent=2)+"\n")
    estimates = model(metadata, replayed)
    (out/"estimate.json").write_text(json.dumps(estimates, indent=2)+"\n")
    for arch, results in estimates.items():
        print(f"{arch}: {sum(r['requests'] for r in replayed[arch]):,} bit-exact requests in {len(metadata)} phases")
        for label, r in results.items():
            print(f"  {label}: replay mean {r['replay_min_mean_max'][1]:,.3f}, expected {r['expected']:,.3f}, "
                  f"bound {r['bound']:,}, conservative margin {r['conservative_margin']:,}")


if __name__ == "__main__":
    main()
