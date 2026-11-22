"""Run software-reference, timing, contention, overflow and reset regressions."""
import random
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def vectors(width, frac):
    rng = random.Random(741)
    lo, hi = -(1 << (width - 1)), (1 << (width - 1)) - 1

    def clip(x):
        return max(lo, min(hi, x))

    cases = [([0, 1, -2, 3],
              [r * 4 + c + 1 if c >= r else hi for r in range(4) for c in range(4)],
              1 << frac),
             ([0]*4, [0]*16, 0), ([hi]*4, [hi]*16, hi),
             ([lo]*4, [lo]*16, hi), ([hi]*4, [1]*16, 1 << frac),
             ([lo]*4, [1]*16, 1 << frac)]
    if frac:
        cases += [([0]*4, [sign]*(16), 1 << (frac-1)) for sign in (-1, 1)]
    for n in range(120):
        bound = min(hi, 4 << frac) if n % 2 else hi
        cases.append(([rng.randint(-bound, bound) for _ in range(4)],
                      [rng.randint(-bound, bound) for _ in range(16)],
                      rng.randint(-bound, bound)))
    words = []
    for state, matrix, gamma in cases:
        expected = [state.copy() for _ in range(9)]
        overflow = False
        for r in range(4):
            for c in range(r, 4):
                product = gamma * matrix[r*4+c]
                magnitude = abs(product)
                rounded = ((magnitude + (1 << (frac-1))) >> frac) if frac else magnitude
                scaled = -rounded if product < 0 else rounded
                offset = clip(scaled)
                overflow |= offset != scaled
                for row, value in ((r+1, state[c]+offset), (r+5, state[c]-offset)):
                    expected[row][c] = clip(value)
                    overflow |= clip(value) != value
        words.extend(state + matrix + [gamma] + sum(expected, []) + [int(overflow)])
    mask = (1 << width)-1
    return '\n'.join(f'{v & mask:x}' for v in words), len(cases)


def run(args):
    subprocess.run(args, cwd=ROOT, check=True)


with tempfile.TemporaryDirectory(prefix='sigma-regression-') as directory:
    tmp = Path(directory)
    for width, frac in ((8, 0), (16, 4), (32, 16)):
        data, count = vectors(width, frac)
        path = tmp / 'vectors.hex'
        path.write_text(data + '\n')
        binary = str(tmp / 'reference')
        run(['iverilog', '-g2012', '-s', 'tb_sigma_reference',
             f'-Ptb_sigma_reference.WIDTH={width}', f'-Ptb_sigma_reference.FRAC={frac}',
             f'-Ptb_sigma_reference.CASES={count}', '-o', binary,
             'tests/tb_sigma_reference.sv', 'sigma_generator.sv', 'MAC_accumulator.sv'])
        run(['vvp', binary, f'+vectors={path}'])
    for timeout in (2, 3, 16):
        binary = str(tmp / 'watchdog')
        run(['iverilog', '-g2012', '-s', 'tb_sigma_watchdog',
             f'-Ptb_sigma_watchdog.TEST_TIMEOUT={timeout}', '-o', binary,
             'tests/tb_sigma_watchdog.sv', 'sigma_generator.sv', 'MAC_accumulator.sv'])
        run(['vvp', binary])
