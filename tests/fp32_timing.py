"""Run the correctness oracles with RTL edge tracing and summarize latency."""
from dataclasses import dataclass
from math import ceil
import os
from pathlib import Path
from statistics import mean, median
import subprocess
import sys

from test_fp32_mac import finite_integer

ROOT = Path(__file__).resolve().parents[1]
INF = 0x7F800000
MIN_NORMAL = 0x00800000
ONE = 0x3F800000
SIGN = 0x80000000


@dataclass(frozen=True)
class Transaction:
    kind: str
    index: int
    a: int
    b: int
    c: int
    result: int
    flags: int
    cycles: int


def magnitude(word):
    return word & (SIGN - 1)


def normal(word):
    return MIN_NORMAL <= magnitude(word) < INF


def subnormal(word):
    return 0 < magnitude(word) < MIN_NORMAL


def nearly_cancels(tx):
    if tx.kind != "MAC" or not all(normal(word) for word in (tx.a, tx.b, tx.c)):
        return False
    product = finite_integer(tx.a) * finite_integer(tx.b)
    addend = finite_integer(tx.c) << 149
    return 0 < abs(product + addend) * 1024 <= abs(product)


def exponent_gap(tx):
    product = finite_integer(tx.a) * finite_integer(tx.b)
    addend = finite_integer(tx.c) << 149
    return abs(abs(product).bit_length() - abs(addend).bit_length())


def show(label, rows):
    cycles = sorted(tx.cycles for tx in rows)
    if not cycles:
        raise ValueError(f"No vectors in {label}")
    p95, p99 = (cycles[ceil(p * len(cycles)) - 1] for p in (0.95, 0.99))
    print(f"{label} | {len(cycles)} | {cycles[0]} | {mean(cycles):.3f} | "
          f"{median(cycles):g} | {p95} | {p99} | {cycles[-1]}")


def pattern(tx):
    return (f"{tx.kind} a={tx.a:08x} b={tx.b:08x} c={tx.c:08x} "
            f"result={tx.result:08x} flags={tx.flags:02x} cycles={tx.cycles}")


def run(module):
    try:
        output = subprocess.check_output(
            [sys.executable, f"tests/test_fp32_{module}.py"], cwd=ROOT, text=True,
            env={**os.environ, "FP32_TIMING": "1"}, timeout=240,
        )
    except subprocess.CalledProcessError as error:
        print("\n".join(error.output.splitlines()[-15:]), file=sys.stderr)
        raise
    rows = []
    for line in output.splitlines():
        if line.startswith("LAT "):
            _, kind, index, a, b, c, result, flags, cycles = line.split()
            rows.append(Transaction(kind, int(index), *map(lambda word: int(word, 16),
                                    (a, b, c, result, flags)), int(cycles)))
        else:
            print(line)
    if [row.index for row in rows] != list(range(len(rows))):
        raise ValueError(f"Missing or misordered {module} trace")
    return rows


def main():
    mac = run("mac")
    shared = run("divsqrt")
    division = [tx for tx in shared if tx.kind == "DIV"]
    root = [tx for tx in shared if tx.kind == "SQRT"]
    mul_normal = [t for t in mac if normal(t.a) and normal(t.b) and magnitude(t.c) == 0]
    fma_normal = [t for t in mac if all(normal(w) for w in (t.a, t.b, t.c))]
    groups = {
        "MAC all / consecutive": mac,
        "MAC normal operands, zero accumulator": mul_normal,
        "MAC normal operands, normal nonzero accumulator": fma_normal,
        "MAC normal operands, both positive": [t for t in mul_normal + fma_normal if not (t.a | t.b) & SIGN],
        "MAC normal operands, at least one negative": [t for t in mul_normal + fma_normal if (t.a | t.b) & SIGN],
        "MAC exponent gap >= 50": [t for t in mac if t.cycles > 1 and magnitude(t.c)
                                   and exponent_gap(t) >= 50],
        "MAC near cancellation": [t for t in mac if nearly_cancels(t)],
        "MAC exact cancellation": [t for t in mac if t.cycles > 1 and t.result == 0 and not t.flags],
        "MAC subnormal operand or accumulator": [t for t in mac if any(map(subnormal, (t.a, t.b, t.c)))],
        "MAC subnormal result": [t for t in mac if subnormal(t.result)],
        "MAC overflow": [t for t in mac if t.flags & 4],
        "MAC underflow": [t for t in mac if t.flags & 2],
        "MAC NaN / infinity input": [t for t in mac if max(map(magnitude, (t.a, t.b, t.c))) >= INF],
        "MAC zero multiplier input": [t for t in mac if min(map(magnitude, (t.a, t.b))) == 0],
        "Division all": division,
        "Division recurrence": [t for t in division if t.cycles > 1],
        "Division normal operands": [t for t in division if normal(t.a) and normal(t.b)],
        "Square root all": root,
        "Square root recurrence": [t for t in root if t.cycles > 1],
        "Square root positive normal": [t for t in root if normal(t.a) and not t.a & SIGN],
    }
    print("Group | N | Min | Mean | Median | P95 | P99 | Max")
    for label, transactions in groups.items():
        show(label, transactions)

    mac_bound = int(os.environ.get("FP32_MAC_LATENCY_BOUND", "22"))
    for rows, bound in ((mac, mac_bound), (division, 81), (root, 54)):
        witness = max(rows, key=lambda row: row.cycles)
        print("Maximum witness:", pattern(witness))
        if witness.cycles != bound:
            raise AssertionError(f"Expected to observe the derived bound of {bound}")

    targets = {
        "DIV": [(ONE, ONE), (ONE, 0x40400000), (INF - 1, 1), (1, INF - 1),
                (SIGN | ONE, 0x40400000), (1, 0x40000000), (ONE, 1), (1, 1),
                (MIN_NORMAL, 0x40000000), (0, ONE), (INF, ONE), (ONE, INF),
                (ONE, SIGN), (0, 0), (INF, INF), (0x7FC12345, ONE)],
        "SQRT": [(value, 0) for value in (ONE, 0x40000000, 0x40800000, 0, SIGN,
                 INF - 1, MIN_NORMAL, 1, 2, 3, 4, MIN_NORMAL - 1,
                 SIGN | ONE, INF, SIGN | INF, 0x7FC12345, 0xFF812345)],
    }
    for kind, operands in targets.items():
        index = {(t.a, t.b): t for t in shared if t.kind == kind}
        for pair in operands:
            print("Directed:", pattern(index[pair]))


if __name__ == "__main__":
    main()
