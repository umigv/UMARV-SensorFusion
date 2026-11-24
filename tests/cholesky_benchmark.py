"""Bit-exact FP32 schedule model, binary64 matrix reference, and RTL replay."""
import argparse
from collections import Counter
from dataclasses import dataclass
import math
from pathlib import Path
import random
from statistics import mean, median
from struct import pack, unpack
import subprocess
import sys
import tempfile

from test_fp32_mac import fma, finite_integer

N = 5
ONE = 0x3F800000
ROOT = Path(__file__).resolve().parents[1]


def word(x):
    return unpack(">I", pack(">f", x))[0]


def value(bits):
    return unpack(">f", pack(">I", bits))[0]


def quotient(a, b):
    """Divide finite operands, rounding an exact integer ratio to binary32."""
    numerator, denominator = finite_integer(a), finite_integer(b)
    sign = (a ^ b) & 0x80000000
    numerator, denominator = abs(numerator), abs(denominator)
    if not denominator:
        return (0x7FC00000, 16) if not numerator else (sign | 0x7F800000, 8)
    if not numerator:
        return sign, 0
    exponent = numerator.bit_length() - denominator.bit_length()
    if ((numerator << max(0, -exponent)) <
            (denominator << max(0, exponent))):
        exponent -= 1
    shift = 23 - max(exponent, -126)
    numerator <<= max(0, shift)
    denominator <<= max(0, -shift)
    significand, remainder = divmod(numerator, denominator)
    twice = 2 * remainder
    significand += twice > denominator or (twice == denominator and significand & 1)
    if significand == 1 << 24:
        significand >>= 1
        exponent += 1
    if exponent > 127:
        return sign | 0x7F800000, 5
    inexact = int(remainder != 0)
    if significand < 1 << 23:
        return sign | significand, 3 * inexact
    return sign | ((max(exponent, -126) + 127) << 23) | (significand & 0x7FFFFF), inexact


def square_root(a):
    """Round the exact square-root interval using integer squared midpoints."""
    integer = finite_integer(a)
    if integer <= 0:
        return (a, 0) if integer == 0 else (0x7FC00000, 16)
    exponent = (integer.bit_length() - 1 - 149) // 2
    shift = 2 * (23 - exponent) - 149
    numerator, denominator = integer << max(0, shift), 1 << max(0, -shift)
    significand = math.isqrt(numerator // denominator)
    inexact = int(significand * significand * denominator != numerator)
    midpoint = (2 * significand + 1) ** 2 * denominator
    significand += 4 * numerator > midpoint or (4 * numerator == midpoint and significand & 1)
    if significand == 1 << 24:
        significand >>= 1
        exponent += 1
    return ((exponent + 127) << 23) | (significand & 0x7FFFFF), inexact


def matrix_covariance(r):
    return [[math.fsum(r[k][i] * r[k][j] for k in range(N))
             for j in range(N)] for i in range(N)]


def cholesky(p):
    r = [[0.0] * N for _ in range(N)]
    for i in range(N):
        pivot = math.fsum([p[i][i], *(-r[k][i] ** 2 for k in range(i))])
        if not pivot > 0 or not math.isfinite(pivot):
            return None
        r[i][i] = math.sqrt(pivot)
        for j in range(i + 1, N):
            r[i][j] = math.fsum([p[i][j], *(-r[k][i] * r[k][j] for k in range(i))]) / r[i][i]
    return r


def factor_error(actual, expected):
    covariance = matrix_covariance(actual)
    errors = [covariance[i][j] - expected[i][j] for i in range(N) for j in range(N)]
    relative = math.hypot(*errors) / math.hypot(*(x for row in expected for x in row))
    return relative, max(map(abs, errors))


def r_slot(i, j):
    return 2 + i * N - i * (i - 1) // 2 + j - i


U = 17
Z, RADIUS, C, S, Q, TEMP, NEW_R = range(22, 29)
MAC_CLEAR, MAC_CONTINUE, DIV, SQRT, MOVE, LOAD, RUN, ROW_END = range(8)


class PivotError(Exception):
    pass


class Lane:
    def __init__(self, r, trace=False):
        self.rf = [0] * 29
        self.rf[1] = ONE
        self.acc = 0
        self.trace = [] if trace else None
        self.counts = Counter()
        for i in range(N):
            for j in range(i, N):
                self.load(r_slot(i, j), word(r[i][j]))

    def load(self, slot, bits):
        self.rf[slot] = bits
        if self.trace is not None:
            self.trace.append((LOAD, 0, 0, slot, bits, 0, 0))

    def operand(self, slot):
        return self.rf[abs(slot)] ^ (0x80000000 if slot < 0 else 0)

    def issue(self, op, a, b=0, dst=0):
        left, right = self.operand(a), self.operand(b)
        reads = int(abs(a) > 1) + int(abs(b) > 1 and abs(a) != abs(b))
        writes = int(dst != 0)
        if op <= MAC_CONTINUE:
            self.acc, flags = fma(left, right, self.acc if op == MAC_CONTINUE else 0)
            result = self.acc
        elif op == DIV:
            result, flags = quotient(left, right)
        elif op == SQRT:
            result, flags = square_root(left)
        else:
            result, flags = left, 0
        if self.trace is not None:
            self.trace.append((op, a, b, dst, result, flags, reads))
        self.counts.update({op: 1, "reads": reads, "writes": writes})
        if dst:
            self.rf[dst] = result
        if flags & 0x1C:
            raise PivotError(f"arithmetic flags 0x{flags:02x}")
        return result

    def factor(self):
        return [[value(self.rf[r_slot(i, j)]) if j >= i else 0.0
                 for j in range(N)] for i in range(N)]

    def update(self, vector, sign, rotation):
        for i, x in enumerate(vector):
            self.load(U + i, word(x))
        self.update_loaded(sign, rotation)

    def update_loaded(self, sign, rotation, start=0, dimension=N):
        """Update an upper factor from a row already in the register file."""
        issue = self.issue
        for k in range(start, dimension):
            diag, uk = r_slot(k, k), U + k
            if not value(self.rf[diag]) > 0:
                raise PivotError("nonpositive input diagonal")
            issue(MAC_CLEAR, diag, diag)
            radicand = issue(MAC_CONTINUE, sign * uk, uk, Z)
            if not value(radicand) > 0:
                raise PivotError("nonpositive computed radicand")
            radius = issue(SQRT, Z, dst=RADIUS)
            if not value(radius) > 0:
                raise PivotError("nonpositive radius")
            if rotation:
                issue(DIV, 1, RADIUS, Q)
                issue(MAC_CLEAR, diag, Q, C)
                issue(MAC_CLEAR, uk, Q, S)
            else:
                issue(DIV, RADIUS, diag, C)
                issue(DIV, uk, diag, S)
            issue(MOVE, RADIUS, dst=diag)
            for j in range(k + 1, dimension):
                old_r, old_u = r_slot(k, j), U + j
                if rotation:
                    issue(MAC_CLEAR, C, old_r)
                    issue(MAC_CONTINUE, sign * S, old_u, NEW_R)
                    issue(MAC_CLEAR, C, old_u)
                    issue(MAC_CONTINUE, -S, old_r, old_u)
                    issue(MOVE, NEW_R, dst=old_r)
                else:
                    issue(MAC_CLEAR, old_r, 1)
                    issue(MAC_CONTINUE, sign * S, old_u, TEMP)
                    issue(DIV, TEMP, C, old_r)
                    issue(MAC_CLEAR, C, old_u)
                    issue(MAC_CONTINUE, -S, old_r, old_u)


@dataclass
class Case:
    group: str
    r: list
    steps: list


def fp32(x):
    return value(word(x))


def represented(r):
    return [[fp32(x) for x in row] for row in r]


def cases():
    rng = random.Random(0xC401)
    scales = (2.0, 2.0, 0.3, 0.1, 0.025)

    def sample_factor(diagonal=False, singular=False, correlated=False):
        if diagonal:
            return [[scales[k] * 0.15 if j == k else 0.0 for j in range(N)] for k in range(N)]
        raw = [[rng.gauss(0, 1) for _ in range(N)] for _ in range(N)]
        if correlated:
            raw = [[0.94 * row[0] + 0.2 * x for x in row] for row in raw]
        ridge = 1e-8 if singular else (0.05 if not correlated else 0.002)
        p = [[scales[j] * scales[k] * (math.fsum(row[j] * row[k] for row in raw)
                                       + (ridge if j == k else 0))
              for k in range(N)] for j in range(N)]
        if singular:
            p = [[x * (1e-3 if j == 4 else 1) * (1e-3 if k == 4 else 1)
                  for k, x in enumerate(row)] for j, row in enumerate(p)]
        return cholesky(p)

    def update(gain):
        return tuple(fp32(rng.gauss(0, gain * s)) for s in scales)

    def removal(r, norm):
        direction = [rng.gauss(0, 1) for _ in scales]
        length = math.hypot(*direction)
        return tuple(fp32(norm * math.fsum(r[k][j] * direction[k] for k in range(N)) / length)
                     for j in range(N))

    for i in range(80):
        r = represented(sample_factor())
        yield Case("random update", r, [(1, update(10 ** rng.uniform(-1, 0.6)))])
        yield Case("valid downdate", r, [(-1, removal(r, rng.uniform(0.05, 0.85)))])
        yield Case("diagonal process noise", represented(sample_factor(diagonal=True)),
                   [(1, update(0.3))])
        yield Case("very small vector", r, [(1, update(1e-5))])
        yield Case("very large vector", r, [(1, update(30))])
        near = represented(sample_factor(singular=True, correlated=True))
        yield Case("ill-conditioned update", near, [(1, update(1e-3))])
        corr = represented(sample_factor(correlated=True))
        yield Case("correlated update", corr, [(1, update(0.4))])
        yield Case("near-boundary downdate", corr, [(-1, removal(corr, 1 - 2.0 ** -(16 + i % 13)))])
        yield Case("invalid downdate", r, [(-1, removal(r, 1.05))])

        initial = represented(sample_factor(diagonal=True))
        for sign in (1, -1):
            steps = [(1, update(0.25)) for _ in range(10)]
            p = matrix_covariance(initial)
            for _, u in steps:
                p = [[math.fsum((p[j][k], u[j] * u[k])) for k in range(N)] for j in range(N)]
            final = update(0.2) if sign == 1 else removal(cholesky(p), 0.55)
            yield Case(f"11-row central {'update' if sign == 1 else 'downdate'}",
                       initial, [*steps, (sign, final)])

    for _ in range(24):
        initial = represented(sample_factor(diagonal=True))
        yield Case("32 sequential updates", initial, [(1, update(0.25)) for _ in range(32)])

    identity = [[float(i == j) for j in range(N)] for i in range(N)]
    yield Case("exact zero pivot", identity, [(-1, (1.0, 0.0, 0.0, 0.0, 0.0))])
    yield Case("tiny positive pivot lost to rounding", identity,
               [(-1, (fp32(1 - 2**-24), fp32(2**-12), value(word(2**-12) - 1), 0.0, 0.0))])


def reference(case):
    initial = matrix_covariance(case.r)
    history = []
    for end in range(1, len(case.steps) + 1):
        p = [[math.fsum([initial[i][j], *(sign * u[i] * u[j]
                                          for sign, u in case.steps[:end])])
              for j in range(N)] for i in range(N)]
        history.append((p, cholesky(p)))
    return history


def simulate(case, rotation, trace):
    lane = Lane(case.r, trace)
    rows = []
    pivot_errors = []
    for sign, u in case.steps:
        try:
            lane.update(u, sign, rotation)
        except PivotError as error:
            pivot_errors.append(str(error))
            break
        rows.append(lane.factor())
        if lane.trace is not None:
            lane.trace.append((ROW_END, 0, 0, 0, 0, 0, 0))
    return lane, rows, pivot_errors


def benchmark():
    stats = {}
    replay = []
    for case in cases():
        history = reference(case)
        expected_valid = all(r is not None for _, r in history)
        group = stats.setdefault(case.group, {
            "cases": 0, "invalid": 0, "reject_valid": [0, 0], "accept_invalid": [0, 0],
            "bad_factor": [0, 0], "errors": [[], [], []], "paired": Counter(),
            "examples": [None, None]})
        group["cases"] += 1
        group["invalid"] += not expected_valid
        if expected_valid:
            group["errors"][2].append(factor_error(history[-1][1], history[-1][0]))
        collect_trace = group["cases"] <= 3 or (
            len(case.steps) == 11 and group["cases"] <= 8)
        results = []
        for rotation in (False, True):
            lane, rows, failures = simulate(case, rotation, collect_trace)
            if failures:
                group.setdefault("causes", [Counter(), Counter()])[rotation].update(failures)
            elif len(case.steps) == 1:
                assert lane.counts == (Counter({0: 35, 1: 25, 2: 5, 3: 5, 4: 15,
                                                "reads": 135, "writes": 60}) if rotation else
                                       Counter({0: 25, 1: 25, 2: 20, 3: 5, 4: 5,
                                                "reads": 130, "writes": 55})), lane.counts
            completed = len(rows) == len(case.steps)
            group["reject_valid"][rotation] += expected_valid and not completed
            group["accept_invalid"][rotation] += not expected_valid and completed
            group["bad_factor"][rotation] += completed and not all(
                math.isfinite(rows[-1][i][i]) and rows[-1][i][i] > 0 for i in range(N))
            if expected_valid and completed:
                errors = factor_error(rows[-1], history[-1][0])
                group["errors"][rotation].append(errors)
                results.append(errors[0])
            else:
                results.append(None)
            if collect_trace:
                replay.append((case.group, group["cases"], int(rotation), len(rows), lane.trace))
        if None not in results:
            a, b = results
            winner = 0 if a < b else 1 if b < a else 2
            group["paired"][winner] += 1
            if winner < 2:
                loser = results[1 - winner]
                ratio = loser / max(results[winner], 1e-30)
                prior = group["examples"][winner]
                if prior is None or ratio > prior[0]:
                    group["examples"][winner] = (ratio, group["cases"], a, b)

    print("Group | N | Reference invalid | A/B valid rejected | A/B invalid accepted | A/B bad factor")
    for name, g in stats.items():
        print(f"{name} | {g['cases']} | {g['invalid']} | {g['reject_valid']} | "
              f"{g['accept_invalid']} | {g['bad_factor']}")
        if "causes" in g:
            print(f"  rejection causes A/B: {[dict(c) for c in g['causes']]}")
    print("Group | N paired A/B/tie | A median/max relative; max abs | B same | C max relative/max abs")
    for name, g in stats.items():
        def error_string(rows):
            if not rows:
                return "—"
            return f"{median(r[0] for r in rows):.3e}/{max(r[0] for r in rows):.3e}; {max(r[1] for r in rows):.3e}"
        print(f"{name} | {list(g['paired'][i] for i in range(3))} | "
              + " | ".join(error_string(rows) for rows in g["errors"]))
        for algorithm, example in zip("AB", g["examples"]):
            if example and example[0] >= 2 and name in ("random update", "valid downdate", "near-boundary downdate"):
                ratio, i, a, b = example
                print(f"  {algorithm} example {i}: errors A={a:.3e}, B={b:.3e}, ratio={ratio:.2f}")
    return replay


def replay_rtl(groups):
    with tempfile.TemporaryDirectory() as directory:
        tmp = Path(directory)
        vectors, sim = tmp / "experiment.txt", tmp / "sim"
        with vectors.open("w") as stream:
            for i, (_, _, _, _, commands) in enumerate(groups):
                stream.write(f"6 0 0 0 {i:08x} 00 0\n")
                for op, a, b, dst, result, flags, reads in commands:
                    stream.write(f"{op} {a} {b} {dst} {result:08x} {flags:02x} {reads}\n")
            stream.write("6 0 0 0 ffffffff 00 0\n")
        subprocess.run(["iverilog", "-g2012", "-Wall", "-s", "tb_cholesky_experiment", "-o", sim,
                        "fp32_mac.sv", "fp32_divsqrt.sv", "tests/tb_cholesky_experiment.sv"],
                       cwd=ROOT, check=True)
        try:
            output = subprocess.check_output(["vvp", sim, f"+vectors={vectors}"],
                                             text=True, timeout=180)
        except subprocess.CalledProcessError as error:
            print("\n".join(error.output.splitlines()[-12:]), file=sys.stderr)
            raise
    records = {}
    row_cycles = {}
    request_count = 0
    for line in output.splitlines():
        if line.startswith("RUN "):
            _, index, elapsed, count = line.split()
            records[int(index)] = int(elapsed)
            request_count += int(count)
        elif line.startswith("ROW "):
            _, index, row, elapsed = line.split()
            row_list = row_cycles.setdefault(int(index), [])
            if int(row) != len(row_list) + 1:
                raise AssertionError("Misordered row marker")
            row_list.append(int(elapsed))
        else:
            print(line)
    if set(records) != set(range(len(groups))):
        raise AssertionError("RTL replay is missing completed groups")

    pairs = {}
    for i, (group, sample, rotation, complete, _) in enumerate(groups):
        pairs.setdefault((group, sample), {})[rotation] = (complete, records[i], row_cycles.get(i, []))
    print(f"RTL checked {request_count} arithmetic requests across {len(groups)} schedules")
    latency = {}
    for (group, sample), runs in pairs.items():
        a, b = runs[0], runs[1]
        if a[0] == b[0] and a[0] != 0:
            latency.setdefault(group, []).append((a[1], b[1]))
            if a[0] == 11:
                if sum(a[2]) != a[1] or sum(b[2]) != b[1]:
                    raise AssertionError("Row timings do not add to prediction covariance timing")
                latency.setdefault(group + " first ten", []).append((sum(a[2][:10]), sum(b[2][:10])))
                latency.setdefault(group + " central only", []).append((a[2][-1], b[2][-1]))
        print(f"RTL {group} #{sample}: A={a[1]} cycles/{a[0]} rows B={b[1]} cycles/{b[0]} rows")
    print("RTL group | Paired samples | Mean A | Mean B | Mean savings | Min/max savings")
    for group, values in latency.items():
        savings = [a - b for a, b in values]
        print(f"{group} | {len(values)} | {mean(a for a, _ in values):.3f} | "
              f"{mean(b for _, b in values):.3f} | {mean(savings):+.3f} | "
              f"{min(savings):+d}/{max(savings):+d}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rtl", action="store_true", help="Replay a stratified subset on the arithmetic RTL")
    arguments = parser.parse_args()
    experiments = benchmark()
    if arguments.rtl:
        replay_rtl(experiments)
