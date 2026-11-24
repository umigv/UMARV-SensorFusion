"""Compare div/sqrt RTL against the host's IEEE binary32 operations and fenv."""
from itertools import chain
import os
from pathlib import Path
import random
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
ONE = 0x3F800000
SIGN = 1 << 31
INF = 0x7F800000
MIN_NORMAL = 0x00800000
MAX_FINITE = INF - 1


def directed():
    special = [0, SIGN, 1, SIGN | 1, MIN_NORMAL, MAX_FINITE, ONE, SIGN | ONE,
               INF, SIGN | INF, 0x7FC12345, 0xFF812345]
    for a in special:
        for b in special:
            yield 0, a, b
            yield 1, a, b  # Includes NaNs in the ignored square-root operand.
    for a, b in [
        (ONE, 0x40000000), (ONE, 0x40400000), (SIGN | ONE, 0x40400000),
        (MAX_FINITE, 1), (1, MAX_FINITE), (1, 0x40000000),
        (3, 0x40000000), (5, 0x40000000), (SIGN | 1, 0x40000000),
        (MIN_NORMAL, 0x40000000),
        (0x00FFFFFE, 0x40000000), (0x00FFFFFF, 0x40000000), (0x01000000, 0x40000000),
        (MIN_NORMAL - 1, ONE - 1), (MIN_NORMAL, ONE + 1),
        (1, 1), (ONE, 1), (MAX_FINITE, ONE - 1),
    ]:
        yield 0, a, b
    for a in [1, 2, 3, 4, MIN_NORMAL - 1, MIN_NORMAL, 0x40000000, 0x40800000,
              MAX_FINITE, 0x3EFFFFFF, 0x3F000000, 0x3F000001,
              ONE - 1, ONE, ONE + 1, 0x407FFFFF, 0x40800001, SIGN | 0x40000000]:
        yield 1, a, 0


def randomized():
    rng = random.Random(0xD175A47)

    def finite_nonzero(positive=False):
        bits = 31 if positive else 32
        while True:
            word = rng.getrandbits(bits)
            if 0 < (word & (SIGN - 1)) < INF:
                return word

    for i in range(12_000):
        if i % 8 == 0:
            a = rng.randrange(1, MIN_NORMAL) | (rng.getrandbits(1) << 31)
            b = rng.choice([ONE, 0x40000000, 0x40400000, ONE + 1, ONE - 1])
            root = rng.randrange(1, MIN_NORMAL)
        else:
            a, b = finite_nonzero(), finite_nonzero()
            root = finite_nonzero(positive=True)
        yield 0, a, b
        yield 1, root, rng.getrandbits(32)


def main():
    timing = os.environ.get("FP32_TIMING") == "1"
    with tempfile.TemporaryDirectory() as directory:
        temporary = Path(directory)
        reference, sim, vectors = (temporary / name for name in ("reference", "sim", "vectors"))
        subprocess.run([*shlex.split(os.environ.get("CC", "cc")), "-std=c11", "-O2",
                        "-Wall", "-Wextra", "-Werror", "-frounding-math", "-fno-fast-math",
                        "-fsignaling-nans", "tests/fp32_divsqrt_reference.c",
                        "-o", reference, "-lm"], cwd=ROOT, check=True)
        inputs = [f"{mode} {a:08x} {b:08x}" for mode, a, b in chain(directed(), randomized())]
        expected = subprocess.check_output([reference], input="\n".join(inputs) + "\n",
                                           text=True).splitlines()
        vectors.write_text("".join(f"{request} {answer}\n"
                                   for request, answer in zip(inputs, expected, strict=True)))
        subprocess.run([*shlex.split(os.environ.get("IVERILOG", "iverilog")),
                        "-g2012", "-Wall", "-s", "tb_fp32_divsqrt", "-o", sim,
                        "fp32_divsqrt.sv", "tests/tb_fp32_divsqrt.sv"], cwd=ROOT, check=True)
        subprocess.run(["vvp", sim, f"+vectors={vectors}", *(["+latency"] if timing else [])],
                       check=True, timeout=180)


if __name__ == "__main__":
    main()
