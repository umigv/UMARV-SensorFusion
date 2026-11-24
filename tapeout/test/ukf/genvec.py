"""Test vectors for fp_serial: op, a, b, expected (numpy float32, round to nearest even).
Results that numpy makes subnormal are expected as signed zero, to match the unit."""
import numpy as np, struct, sys
rng = np.random.default_rng(7)
def bits(f): return struct.unpack("<I", struct.pack("<f", float(f)))[0]
def fl(u): return np.float32(struct.unpack("<f", struct.pack("<I", int(u)))[0])
rows = []
def add(op, a, b):
    a, b = np.float32(a), np.float32(b)
    with np.errstate(all="ignore"):
        r = [a + b, a - b, a * b, a / b][op]
    r = np.float32(r)
    if not np.isfinite(r) and not np.isinf(r): return
    rb = bits(r)
    if (rb >> 23) & 0xFF == 0: rb &= 0x80000000          # subnormal or zero -> zero
    if np.isnan(r): return
    # zero results: the unit gives +0 for exact cancellation; sign of zero is not checked
    rows.append((op, bits(a), bits(b), rb))
N = int(sys.argv[1]) if len(sys.argv) > 1 else 30000
# square root (op 4): a is used, its sign ignored; b is anything
for i in range(8000):
    k = rng.integers(0, 4)
    if k == 0: a = fl(rng.integers(0, 2**31))
    elif k == 1: a = np.float32(rng.uniform(0, 4) * 10.0 ** rng.integers(-12, 12))
    elif k == 2: a = np.float32(float(rng.integers(1, 4096)) ** 2 * 2.0 ** int(rng.integers(-20, 20)))     # exact squares
    else: a = fl((int(rng.integers(90, 160)) << 23) | (0x7FFFFF if rng.integers(0, 2) else int(rng.integers(0, 8))))
    ab = bits(a)
    if (ab >> 23) & 0xFF in (255,): continue
    if (ab >> 23) & 0xFF == 0: a = np.float32(0.0); ab = 0
    r = bits(np.sqrt(np.float32(abs(float(a))), dtype=np.float32))
    if rng.integers(0, 4) == 0: ab |= 0x80000000
    rows.append((4, ab, bits(fl(rng.integers(0, 2**32))), r))
# integer difference (op 5): low 26 bits of each word, signed
def s26(v): v &= (1 << 26) - 1; return v - (1 << 26) if v >> 25 else v
for i in range(8000):
    k = rng.integers(0, 4)
    if k == 0: a, b = int(rng.integers(0, 2**32)), int(rng.integers(0, 2**32))
    elif k == 1: a, b = int(rng.integers(-300, 300)) & 0xFFFFFFFF, 0
    elif k == 2: a = int(rng.integers(0, 2**26)); b = (a + int(rng.integers(-5000, 5000))) & 0xFFFFFFFF
    else: a = int(rng.integers(0, 2**26)); b = a
    r = np.float32(s26(a) - s26(b))
    rows.append((5, a, b, bits(r)))
for i in range(N):
    op = int(rng.integers(0, 4))
    kind = rng.integers(0, 6)
    if kind == 0:   # wide range of exponents
        a = fl(rng.integers(0, 2**32)); b = fl(rng.integers(0, 2**32))
        ea, eb = (bits(a) >> 23) & 0xFF, (bits(b) >> 23) & 0xFF
        if ea in (0, 255) or eb in (0, 255): continue
    elif kind == 1: # close magnitudes: cancellation in add and subtract
        a = np.float32(rng.normal() * 10 ** rng.uniform(-6, 6)); b = np.float32(a * (1 + rng.normal() * 10 ** rng.uniform(-8, -1)))
        if rng.integers(0, 2): b = -b
    elif kind == 2: # filter-like values
        a = np.float32(rng.normal() * 10 ** rng.uniform(-9, 4)); b = np.float32(rng.normal() * 10 ** rng.uniform(-9, 4))
    elif kind == 3: # exponents within the shift range, random mantissas (rounding cases)
        e = int(rng.integers(100, 150)); d = int(rng.integers(-27, 28))
        a = fl((int(rng.integers(0, 2)) << 31) | (e << 23) | int(rng.integers(0, 2**23)))
        b = fl((int(rng.integers(0, 2)) << 31) | ((e + d) << 23) | int(rng.integers(0, 2**23)))
    elif kind == 4: # mantissas with long runs of ones or zeros (ties, carries)
        def m():
            k = int(rng.integers(0, 24)); return ((1 << k) - 1) << int(rng.integers(0, 24 - k)) if k else 0
        e = int(rng.integers(110, 140)); d = int(rng.integers(-25, 26))
        a = fl((e << 23) | (m() & 0x7FFFFF)); b = fl((int(rng.integers(0, 2)) << 31) | ((e + d) << 23) | (m() & 0x7FFFFF))
    else:           # one operand zero or equal operands
        a = np.float32(rng.normal() * 100)
        b = [np.float32(0), a, -a][int(rng.integers(0, 3))]
        if rng.integers(0, 2): a, b = b, a
        if op == 3 and b == 0: continue
    if op == 3 and float(b) == 0.0: continue
    add(op, a, b)
with open("vectors.hex", "w") as f:
    for op, a, b, r in rows:
        f.write("%x%08x%08x%08x\n" % (op, a, b, r))
print(len(rows), "vectors")
