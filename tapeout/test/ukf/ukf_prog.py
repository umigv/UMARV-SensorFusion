"""Step program for a five-state square-root unscented Kalman filter on a small stored-program machine.

The machine (ukf_top.sv): 64 words of 32-bit storage, a table of constants, a few sensor words, one
bit-serial single-precision arithmetic unit (fp_serial.sv), a one-bit flag, and a table of steps.
A step is either  d = a op b  (optionally: record the result's sign in the flag; write only if the
flag is set / clear), or a jump, call, return or stop.

States: x east, y north, theta (radians, anticlockwise from east), omega, r (dead wheel radius).
Model and tuning are those of sim/rate_study/sim1k_ukf_vs_kf.py:
    mid = theta + omega dt/2;  x += r dphi cos(mid);  y += r dphi sin(mid);  theta += omega dt
    measurements x, y, theta, omega each read one state.

How it differs from the simulation's filter, and why the results are the same to rounding:
  - sigma points are made one at a time and never stored; prediction makes them twice (once for
    the mean, once for the spread)
  - the new covariance square root is built by 10 rank-one updates and one downdate, where the
    simulation uses a QR factorisation followed by the downdate
  - each measurement reads one state, so the four are applied one after another, each as a
    rank-one downdate, where the simulation builds a 4 x 4 system from sigma points

This file: the program, an exact model of the machine (bit for bit what the hardware computes),
a fast model for long runs, and writers for the Verilog tables.
"""
import math
import struct

import numpy as np

# ------------------------------------------------------------------------------------------------
# storage map
OUT_TS, OUT_X, OUT_Y, OUT_TH, OUT_OM, OUT_V = 0, 1, 2, 3, 4, 5      # wired straight to the lidar block
XH = [6, 7, 8, 9, 10]                                               # working state
S_BASE, SN_BASE = 11, 26                                            # covariance square root, and the one being built
V = [41, 42, 43, 44, 45]                                            # a sigma point, then a deviation / update vector
M = [46, 47, 48]                                                    # predicted mean of x, y, theta; later the measurements
ACC = [49, 50, 51]
DPHI, LAT0, LON0, KLON, GS = 52, 53, 54, 55, 56
T = [57, 58, 59, 60, 61]
WORDS = 62
CONST_BASE, EXT_BASE = 64, 112
# Constants the host can replace, in the order of their addresses (CFG_BASE upward). What each is:
#   RR01, RR2, RR3   measurement noise variance: position (m^2), heading (rad^2), turn rate ((rad/s)^2)
#   SQ01, SQ3, SQ2   process noise per beat (standard deviation): position, turn rate, heading
#   CNT2RAD          radians of wheel rotation per encoder count
#   RNOM             the wheel radius the filter starts from (m)
#   YAWK, YAW0       heading = YAW0 + YAWK * (the sensor's yaw word)
#   RATEK            turn rate = RATEK * (the sensor's rate word)
#   KLONU            metres east per count of the longitude word, at the equator (scaled by cos(latitude) inside)
#   KLAT             metres north per count of the latitude word
#   DT, HALFDT, HZ   the beat period in seconds, half of it, and its reciprocal (keep the three consistent)
TUNABLE = ["RR01", "RR2", "RR3", "SQ01", "SQ3", "SQ2", "CNT2RAD", "RNOM", "YAWK", "YAW0", "RATEK", "KLONU", "KLAT",
           "DT", "HALFDT", "HZ"]
CFG_BASE = 96
# sensor words. 112-115 are the raw halves of latitude and longitude, there only for the host to read
EXT = dict(YAW=120, RATE=121, LAT=122, LON=123, DW=124, TS=125, LATF=126)


def tri(i, j):
    assert i >= j
    return i * (i + 1) // 2 + j


def S(i, j): return S_BASE + tri(i, j)
def SN(i, j): return SN_BASE + tri(i, j)


# ------------------------------------------------------------------------------------------------
# tuning and constants (same numbers as sim1k; see sim/rate_study/params.py for their sources)
def constants(noise="scaled", rate=400.0, site_lat_deg=42.29):
    dt = 1.0 / rate
    n, alpha, beta, kappa = 5, 0.5, 2.0, 0.0
    lam = alpha * alpha * (n + kappa) - n
    gamma = math.sqrt(n + lam)
    wm0 = lam / (n + lam)
    wc0 = wm0 + 1 - alpha * alpha + beta
    wi = 1.0 / (2 * (n + lam))
    assert wc0 < 0, "the program applies the centre point as a downdate"
    DEG = math.pi / 180.0
    counts, r_nom = 2 ** 14, 0.05
    pos_sigma, head_sigma = 1.0 / math.sqrt(2.0), 1.0 * DEG
    gyro_sigma = 0.0035 * DEG * math.sqrt(256.0)
    pos_corr, head_corr = pos_sigma / 0.05, (0.2 * DEG) / (10.0 * DEG / 3600.0)
    scaled = noise == "scaled"
    r_pos = pos_sigma * (math.sqrt(rate * math.sqrt(2 * math.pi) * pos_corr) if scaled else 1.0)
    r_head = head_sigma * (math.sqrt(rate * math.sqrt(2 * math.pi) * head_corr) if scaled else 1.0)
    q_pos = math.sqrt(2.0 / 12.0) * (2 * math.pi / counts) * r_nom + 1e-3 * math.sqrt(dt)
    dQ = [q_pos, q_pos, 1e-4 * math.sqrt(dt), math.sqrt(0.01 * dt), 1e-7]
    dR = [r_pos, r_pos, r_head, gyro_sigma]
    d0 = [pos_sigma, pos_sigma, head_sigma, 0.5, 1e-3]
    # WGS-84 radii of curvature at the site, for turning degrees into metres
    a, e2 = 6378137.0, 6.69437999014e-3
    sl = math.sin(site_lat_deg * DEG)
    r_north = a * (1 - e2) / (1 - e2 * sl * sl) ** 1.5
    r_east = a / math.sqrt(1 - e2 * sl * sl)
    fixed = dict(ZERO=0.0, ONE=1.0, MONE=-1.0, GAMMA=gamma, NGAMMA=-gamma, WI=wi, SQRTWI=math.sqrt(wi),
             W0D=math.sqrt(abs(wc0)) * wi,
             PI=math.pi, NEGPI=-math.pi, TWOPI=2 * math.pi, HALFPI=math.pi / 2,
             DEG2RAD=DEG, EPS=1e-30,
             C3=-1 / 6.0, C5=1 / 120.0, C7=-1 / 5040.0, C9=1 / 362880.0, C11=-1 / 39916800.0,
             SQ4=dQ[4], D01=d0[0], D2=d0[2], D3=d0[3], D4=d0[4])
    # The values below can be replaced by the host (see TUNABLE). These are their reset values.
    # One count of the latitude word is 2^-27 degree; of the longitude word, 2^-26 degree, and
    # longitude counts grow westward (see ukf_top.sv for the assumptions).
    tunable = dict(RR01=dR[0] ** 2, RR2=dR[2] ** 2, RR3=dR[3] ** 2,
                   SQ01=dQ[0], SQ3=dQ[3], SQ2=dQ[2],
                   CNT2RAD=2 * math.pi / counts, RNOM=r_nom,
                   YAWK=-DEG, YAW0=math.pi / 2, RATEK=-1.0,
                   KLONU=-r_east * DEG * 2.0 ** -26, KLAT=r_north * DEG * 2.0 ** -27,
                   DT=dt, HALFDT=dt / 2, HZ=rate)
    assert list(tunable) == TUNABLE and len(fixed) <= CFG_BASE - CONST_BASE
    c = dict(fixed)
    for i in range(CFG_BASE - CONST_BASE - len(fixed)): c["_PAD%d" % i] = 0.0
    c.update(tunable)
    return c, dict(r_north=r_north, r_east=r_east, wi=wi, gamma=gamma, wc0=wc0, dQ=dQ, dR=dR, d0=d0, r_nom=r_nom, dt=dt)


# ------------------------------------------------------------------------------------------------
# the assembler
ADD, SUB, MUL, DIV, SQRT, ISUB, RAW, CTL = range(8)
JMP, CALL, RET, HALT, DONE, SETI = range(6)
ALWAYS, IF_F, IF_NF, IF_VN, IF_NVN, IF_DW, IF_NDW, IF_NINIT = range(8)
W_ALWAYS, W_NEVER, W_IF_F, W_IF_NF = 0, 1, 2, 3
FN_NAME = ["add", "sub", "mul", "div", "sqrt", "isub", "raw", "ctl"]


class Asm:
    def __init__(self, consts):
        self.code, self.labels = [], {}
        self.cnames = list(consts)
        self.cval = consts
        assert len(self.cnames) <= EXT_BASE - CONST_BASE

    def k(self, name): return CONST_BASE + self.cnames.index(name)
    def label(self, name): assert name not in self.labels; self.labels[name] = len(self.code)

    def op(self, fn, d, a, b, setf=False, cond=W_ALWAYS):
        a = self.k(a) if isinstance(a, str) and a in self.cval else (EXT[a] if isinstance(a, str) else a)
        b = self.k(b) if isinstance(b, str) and b in self.cval else (EXT[b] if isinstance(b, str) else b)
        assert 0 <= d < WORDS and 0 <= a < 128 and 0 <= b < 128
        self.code.append(("A", fn, d, a, b, int(setf), cond))

    def add(self, d, a, b, **k): self.op(ADD, d, a, b, **k)
    def sub(self, d, a, b, **k): self.op(SUB, d, a, b, **k)
    def mul(self, d, a, b, **k): self.op(MUL, d, a, b, **k)
    def div(self, d, a, b, **k): self.op(DIV, d, a, b, **k)
    def sqrt(self, d, a, **k): self.op(SQRT, d, a, "ZERO", **k)
    def isub(self, d, a, b, **k): self.op(ISUB, d, a, b, **k)
    def raw(self, d, a, **k): self.op(RAW, d, a, "ZERO", **k)
    def mov(self, d, a, **k): self.op(RAW, d, a, "ZERO", **k)
    def test(self, fn, a, b): self.op(fn, 0, a, b, setf=True, cond=W_NEVER)     # only the sign of the result is kept
    def ctl(self, cop, target=None, cond=ALWAYS): self.code.append(("C", cop, cond, target))
    def jmp(self, t, cond=ALWAYS): self.ctl(JMP, t, cond)
    def call(self, t): self.ctl(CALL, t)
    def ret(self): self.ctl(RET)

    def resolve(self):
        out = []
        for ins in self.code:
            if ins[0] == "C":
                out.append(("C", ins[1], ins[2], self.labels[ins[3]] if ins[3] is not None else 0))
            else:
                out.append(ins)
        return out


def build(consts):
    p = Asm(consts)
    T0, T1, T2, T3, T4 = T

    def wrap(w):                        # bring an angle within a turn of zero back into (-pi, pi]
        p.test(SUB, "PI", w);  p.sub(w, w, "TWOPI", cond=W_IF_F)               # w > pi
        p.test(ADD, w, "PI");  p.add(w, w, "TWOPI", cond=W_IF_F)               # w < -pi

    def outputs():                      # copy the finished state to the words the lidar block sees
        p.mul(T0, XH[4], DPHI); p.mul(T0, T0, "HZ")                           # forward speed
        for d, a in ((OUT_X, XH[0]), (OUT_Y, XH[1]), (OUT_TH, XH[2]), (OUT_OM, XH[3]), (OUT_V, T0)):
            p.raw(d, a)
        p.raw(OUT_TS, "TS")
        p.ctl(DONE)

    def theta_meas(d):                  # heading from the yaw word (reset values: degrees clockwise from north)
        p.mul(d, "YAW", "YAWK"); p.add(d, "YAW0", d)

    # ---------------- entry ----------------
    p.jmp("init", IF_NINIT)
    p.isub(T0, "DW", "ZERO"); p.mul(DPHI, T0, "CNT2RAD")
    p.jmp("have_dphi", IF_DW)
    p.mov(DPHI, "ZERO")
    p.label("have_dphi")

    # ---------------- prediction, first pass: the mean ----------------
    for i in range(5): p.mov(V[i], XH[i])
    p.call("model")
    for i in range(3): p.mov(M[i], V[i]); p.mov(ACC[i], "ZERO")
    for col in range(5):
        for g in ("GAMMA", "NGAMMA"):
            p.mov(GS, g); p.call("sigma%d" % col); p.call("model"); p.call("accum")
    for i in range(3):
        p.mul(T0, ACC[i], "WI"); p.add(M[i], M[i], T0)
    wrap(M[2])

    # ---------------- prediction, second pass: the spread ----------------
    for i in range(5):
        for j in range(i + 1):
            p.mov(SN(i, j), ("SQ01", "SQ01", "SQ2", "SQ3", "SQ4")[i] if i == j else "ZERO")
    for col in range(5):
        for g in ("GAMMA", "NGAMMA"):
            p.mov(GS, g); p.call("sigma%d" % col); p.call("model"); p.call("deviation")
            p.mov(GS, "ONE"); p.call("chol")
    for i in range(3): p.mul(V[i], ACC[i], "W0D")                               # the centre point, as a downdate
    p.mov(V[3], "ZERO"); p.mov(V[4], "ZERO")
    p.mov(GS, "MONE"); p.call("chol")
    for i in range(3): p.mov(XH[i], M[i])

    # ---------------- the four measurements, one at a time ----------------
    p.jmp("finish", IF_NVN)
    for m in range(4):
        for i in range(5):                                                      # V = P[:, m] = S S'[:, m]
            for j in range(min(i, m) + 1):
                if j == 0: p.mul(V[i], SN(i, 0), SN(m, 0))
                else:      p.mul(T0, SN(i, j), SN(m, j)); p.add(V[i], V[i], T0)
        if m == 0:   p.isub(T2, "LON", LON0); p.mul(T2, T2, KLON)
        elif m == 1: p.isub(T2, "LAT", LAT0); p.mul(T2, T2, "KLAT")
        elif m == 2: theta_meas(T2)
        else:        p.mul(T2, "RATE", "RATEK")
        p.sub(T2, T2, XH[m])
        if m == 2: wrap(T2)
        p.add(T0, V[m], ("RR01", "RR01", "RR2", "RR3")[m])
        p.call("update")
    wrap(XH[2])

    p.label("finish")
    for i in range(15): p.mov(S_BASE + i, SN_BASE + i)
    outputs()

    # ---------------- first beat with a good VN-300 message ----------------
    p.label("init")
    p.jmp("no_init", IF_NVN)
    p.raw(LAT0, "LAT"); p.raw(LON0, "LON")
    p.mul(T0, "LATF", "DEG2RAD"); p.call("cos"); p.mul(KLON, T1, "KLONU")
    p.mov(XH[0], "ZERO"); p.mov(XH[1], "ZERO")
    theta_meas(XH[2]); wrap(XH[2])
    p.mul(XH[3], "RATE", "RATEK"); p.mov(XH[4], "RNOM")
    for i in range(5):
        for j in range(i + 1):
            p.mov(S(i, j), ("D01", "D01", "D2", "D3", "D4")[i] if i == j else "ZERO")
    p.mov(DPHI, "ZERO")
    p.ctl(SETI)
    outputs()
    p.label("no_init")
    p.ctl(HALT)

    # ---------------- subroutines ----------------
    for col in range(5):                # one sigma point: the state plus GS times a column of S
        p.label("sigma%d" % col)
        for i in range(5):
            if i < col: p.mov(V[i], XH[i])
            else:       p.mul(T0, S(i, col), GS); p.add(V[i], XH[i], T0)
        p.ret()

    p.label("model")                    # the point in V through the motion model, in place
    p.mul(T3, V[3], "HALFDT"); p.add(T3, V[2], T3)
    p.mul(T4, V[4], DPHI)
    p.mov(T0, T3); p.call("cos"); p.mul(T1, T4, T1); p.add(V[0], V[0], T1)
    p.mov(T0, T3); p.call("sin"); p.mul(T1, T4, T1); p.add(V[1], V[1], T1)
    p.mul(T1, V[3], "DT"); p.add(V[2], V[2], T1)
    p.ret()

    p.label("cos")                      # T1 = cos(T0), |T0| up to a little over pi
    p.sub(T1, "ZERO", T0, setf=True); p.mov(T0, T1, cond=W_IF_NF)               # T0 = |T0|
    p.sub(T0, "HALFPI", T0)
    p.jmp("sinpoly")
    p.label("sin")                      # T1 = sin(T0), |T0| up to a little over pi
    p.test(SUB, "HALFPI", T0); p.sub(T0, "PI", T0, cond=W_IF_F)
    p.test(ADD, T0, "HALFPI"); p.sub(T0, "NEGPI", T0, cond=W_IF_F)
    p.label("sinpoly")                  # T1 = sin(T0) for |T0| up to about pi/2
    p.mul(T2, T0, T0)
    p.mul(T1, T2, "C11")
    for c in ("C9", "C7", "C5", "C3", "ONE"):
        p.add(T1, T1, c)
        if c != "ONE": p.mul(T1, T1, T2)
    p.mul(T1, T1, T0)
    p.ret()

    p.label("accum")                    # add this point's distance from the centre point
    for i in range(3):
        p.sub(T0, V[i], M[i])
        if i == 2: wrap(T0)
        p.add(ACC[i], ACC[i], T0)
    p.ret()

    p.label("deviation")                # V = sqrt(weight) * (point - mean)
    for i in range(3): p.sub(V[i], V[i], M[i])
    wrap(V[2])
    p.sub(V[3], V[3], XH[3]); p.sub(V[4], V[4], XH[4])
    for i in range(5): p.mul(V[i], V[i], "SQRTWI")
    p.ret()

    p.label("update")                   # T0 = innovation variance, T2 = innovation, V = P[:, m]
    p.sqrt(T1, T0)
    p.div(T2, T2, T1)
    for i in range(5): p.div(V[i], V[i], T1)
    for i in range(5): p.mul(T0, V[i], T2); p.add(XH[i], XH[i], T0)
    p.mov(GS, "MONE")
    # falls through into the downdate
    p.label("chol")                     # SN SN' <- SN SN' + GS * V V'   (GS is +1 or -1); V is used up
    for k in range(5):
        p.mul(T0, SN(k, k), SN(k, k)); p.mul(T1, V[k], V[k]); p.mul(T1, T1, GS); p.add(T0, T0, T1)
        p.test(SUB, T0, "EPS"); p.mov(T0, "EPS", cond=W_IF_F)                   # keep a failed downdate finite
        p.sqrt(T2, T0)
        if k < 4:
            p.div(T3, T2, SN(k, k)); p.div(T4, V[k], SN(k, k))
        p.mov(SN(k, k), T2)
        if k < 4: p.mul(T2, T4, GS)
        for j in range(k + 1, 5):
            p.mul(T0, T2, V[j]); p.add(T0, SN(j, k), T0); p.div(SN(j, k), T0, T3)
            p.mul(T1, T3, V[j]); p.mul(T0, T4, SN(j, k)); p.sub(V[j], T1, T0)
    p.ret()
    return p


# ------------------------------------------------------------------------------------------------
# exact model of the machine: every word is a 32-bit pattern, every operation as fp_serial does it
def f2b(f): return struct.unpack("<I", struct.pack("<f", float(f)))[0]
def b2f(u): return np.float32(struct.unpack("<f", struct.pack("<I", u & 0xFFFFFFFF))[0])
def s26(v): v &= (1 << 26) - 1; return v - (1 << 26) if v >> 25 else v
def _exp(u): return (u >> 23) & 0xFF
def _flush(u): return u if _exp(u) else (u & 0x80000000)


def unit(fn, a, b):
    """One operation on bit patterns, as the hardware does it. Inputs must be finite."""
    if fn == RAW: return a
    if fn == ISUB:
        return f2b(np.float32(s26(a) - s26(b)))
    a, b = _flush(a), _flush(b)
    assert _exp(a) != 255 and (fn == SQRT or _exp(b) != 255), "infinity reached an operation"
    with np.errstate(all="ignore"):
        if fn == SQRT:
            r = np.sqrt(np.float32(abs(float(b2f(a)))), dtype=np.float32)
        elif fn == ADD: r = b2f(a) + b2f(b)
        elif fn == SUB: r = b2f(a) - b2f(b)
        elif fn == MUL: r = b2f(a) * b2f(b)
        else:
            if (b & 0x7FFFFFFF) == 0: return ((a ^ b) & 0x80000000) | 0x7F800000
            r = b2f(a) / b2f(b)
    u = f2b(np.float32(r))
    if fn in (ADD, SUB) and (u & 0x7FFFFFFF) == 0: return 0          # exact cancellation gives +0
    if fn == SQRT and (u & 0x7FFFFFFF) == 0: return 0
    return _flush(u)


class Machine:
    def __init__(self, prog, consts, cnames):
        self.prog, self.mem, self.cnames = prog, [0] * WORDS, list(cnames)
        self.cbits = [f2b(np.float32(consts[n])) for n in cnames]
        self.inited, self.flag, self.steps, self.ops = False, 0, 0, [0] * 8
        self.written, self.max_depth = set(), 0

    def set_tunable(self, name, value):
        """What a host write of a tunable word does: the constant is replaced until reset."""
        self.cbits[self.cnames.index(name)] = f2b(np.float32(value))

    def read(self, addr, ext):
        if addr < WORDS: return self.mem[addr]
        if addr < EXT_BASE: return self.cbits[addr - CONST_BASE]
        return ext[addr - EXT_BASE]

    def beat(self, ext, vn, dw):
        """ext: the 16 sensor words (see ext_words). Returns True if a new state was produced."""
        pc, stack = 0, []
        while True:
            ins = self.prog[pc]; self.steps += 1
            if ins[0] == "C":
                _, cop, cond, tgt = ins
                take = [True, self.flag, not self.flag, vn, not vn, dw, not dw, not self.inited][cond]
                if cop == JMP: pc = tgt if take else pc + 1
                elif cop == CALL: stack.append(pc + 1); pc = tgt; self.max_depth = max(self.max_depth, len(stack))
                elif cop == RET: pc = stack.pop()
                elif cop == SETI: self.inited = True; pc += 1
                elif cop == HALT: return False
                else: return True
                continue
            _, fn, d, a, b, setf, cond = ins
            r = unit(fn, self.read(a, ext), self.read(b, ext))
            self.ops[fn] += 1
            write = cond == W_ALWAYS or (cond == W_IF_F and self.flag) or (cond == W_IF_NF and not self.flag)     # W_NEVER: no
            if setf: self.flag = r >> 31
            if write: self.mem[d] = r; self.written.add(d)
            pc += 1


# ------------------------------------------------------------------------------------------------
# sensor words from physical values
def double_bits(v): return struct.unpack("<Q", struct.pack("<d", float(v)))[0]


def ext_words(yaw_deg, rate, lat_deg, lon_deg, dw_counts, ts):
    lat, lon = double_bits(lat_deg), double_bits(lon_deg)
    dw = int(dw_counts) & 0xFFFFFFF
    dw |= 0xF0000000 if dw >> 27 else 0                     # the hardware presents it sign-extended
    return [lat & 0xFFFFFFFF, lat >> 32, lon & 0xFFFFFFFF, lon >> 32, 0, 0, 0, 0,
            f2b(np.float32(yaw_deg)), f2b(np.float32(rate)), (lat >> 20) & 0x3FFFFFF, (lon >> 20) & 0x3FFFFFF,
            dw, int(ts) & 0xFFFFFFFF, double_to_single_bits(lat), 0]


def double_to_single_bits(d):
    """What the hardware wiring does: sign, exponent re-biased, top 23 fraction bits (no rounding)."""
    sign, e, frac = d >> 63, (d >> 52) & 0x7FF, d & ((1 << 52) - 1)
    return (sign << 31) | (((e - 1023 + 127) & 0xFF) << 23) | (frac >> 29)


# ------------------------------------------------------------------------------------------------
# writers
def encode(ins):
    if ins[0] == "C":
        _, cop, cond, tgt = ins
        return (CTL << 23) | (cop << 20) | (cond << 17) | tgt
    _, fn, d, a, b, setf, cond = ins
    return (fn << 23) | (setf << 22) | (cond << 20) | (a << 13) | (b << 6) | d


def write_verilog(prog, consts, cnames, path):
    rom = "\n".join("            10'd%d: word = 26'h%07x;" % (i, encode(x)) for i, x in enumerate(prog))
    cr = "\n".join("            6'd%d: value = 32'h%08x;    // %s = %.9g" % (i, f2b(np.float32(consts[n])), n, consts[n])
                   for i, n in enumerate(cnames) if not n.startswith("_PAD"))
    open(path, "w").write('''// Generated by ukf_prog.py. Do not edit: change the program there and regenerate.
// ukf_steps: the filter's %d steps. ukf_consts: its %d constants.
// Step word: [25:23] operation (0 add, 1 subtract, 2 multiply, 3 divide, 4 square root, 5 integer
// difference, 6 copy, 7 control). For 0-6: [22] record the result's sign in the flag, [21:20] write
// always / if the flag is set (2) / if it is clear (3), [19:13] first operand, [12:6] second operand,
// [5:0] destination. For 7: [22:20] jump, call, return, stop, stop with a new state, mark as started;
// [19:17] condition; [9:0] target.
// Kept as its own block in synthesis: merged into its surroundings it maps to more cells and
// the chip does not route.
(* keep_hierarchy *)
module ukf_steps (
    input  logic [9:0]  pc,
    output logic [25:0] word
);
    always_comb begin
        case (pc)
%s
            default: word = 26'h%07x;
        endcase
    end
endmodule

module ukf_consts (
    input  logic [5:0]  index,
    output logic [31:0] value
);
    always_comb begin
        case (index)
%s
            default: value = 32'h00000000;
        endcase
    end
endmodule
''' % (len(prog), sum(not n.startswith('_PAD') for n in cnames), rom, (CTL << 23) | (HALT << 20), cr))


if __name__ == "__main__":
    import sys
    consts, info = constants()
    p = build(consts)
    prog = p.resolve()
    print("steps in the table: %d, constants: %d, storage words: %d" % (len(prog), len(p.cnames), WORDS))
    if len(sys.argv) > 1:
        write_verilog(prog, consts, p.cnames, sys.argv[1])
        print("wrote", sys.argv[1])
