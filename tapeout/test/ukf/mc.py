"""Accuracy of the step program against the simulation's square-root unscented filter (sim1k).

Same drive, sensors, noise and seeds as sim1k. The program is run on a fast model of the machine
(single precision, straight-line code generated from the step table; no special handling of
infinities or tiny numbers, which the exact model in ukf_prog.py has and which test_exact.py
compares against this one). Sensor values go in the way the chip receives them: yaw in degrees,
turn rate, latitude and longitude as doubles, wheel counts.

Run:  python3 -s mc.py <path to sim/rate_study>
"""
import json, math, os, sys
from multiprocessing import Pool
import numpy as np
sys.path.insert(0, sys.argv[1] if len(sys.argv) > 1 else ".")
import ukf_prog as U

F = np.float32


def gen_fast(prog, cnames, inited, vn, dw):
    """Python source for one beat along the path chosen by (inited, vn, dw). m: list of arrays."""
    lines, pc, stack, n = [], 0, [], 0
    def rd(addr, as_int):
        if addr < U.WORDS: return "m[%d]" % addr
        if addr < U.EXT_BASE: return ("ci[%d]" if as_int else "c[%d]") % (addr - U.CONST_BASE)
        return "x[%d]" % (addr - U.EXT_BASE)
    while True:
        ins = prog[pc]
        if ins[0] == "C":
            _, cop, cond, tgt = ins
            assert cond not in (U.IF_F, U.IF_NF)
            take = [True, None, None, vn, not vn, dw, not dw, not inited][cond]
            if cop == U.JMP: pc = tgt if take else pc + 1
            elif cop == U.CALL: stack.append(pc + 1); pc = tgt
            elif cop == U.RET: pc = stack.pop()
            elif cop == U.SETI: pc += 1
            else: break
            continue
        _, fn, d, a, b, setf, cond = ins
        if fn == U.RAW: e = rd(a, False)
        elif fn == U.ISUB: e = "isub(%s, %s)" % (rd(a, True), rd(b, True))
        elif fn == U.SQRT: e = "np.sqrt(np.abs(%s))" % rd(a, False)
        else: e = "(%s %s %s)" % (rd(a, False), "+-*/"[fn], rd(b, False))
        lines.append("r = %s" % e)
        if cond == U.W_ALWAYS: lines.append("m[%d] = r" % d)
        elif cond == U.W_IF_F: lines.append("m[%d] = np.where(f, r, m[%d])" % (d, d))
        else: lines.append("m[%d] = np.where(f, m[%d], r)" % (d, d))
        if setf: lines.append("f = np.signbit(r)")
        n += 1
        pc += 1
    return "def beat(m, x, c, ci, isub, np):\n    f = False\n" + "\n".join("    " + l for l in lines) + "\n", n


def isub(a, b):
    def s26(v):
        v = np.asarray(v, dtype=np.int64) & ((1 << 26) - 1)
        return np.where(v >> 25, v - (1 << 26), v)
    return (s26(a) - s26(b)).astype(F)


def run(noise):
    import params as P, sim1_filter_rate as base, sim1e_mixed_format as e, sim1k_ukf_vs_kf as k1
    consts, info = U.constants(noise)
    p = U.build(consts); prog = p.resolve()
    ns = {}
    exec(gen_fast(prog, p.cnames, False, True, True)[0].replace("def beat", "def beat_init"), ns)
    src, nops = gen_fast(prog, p.cnames, True, True, True)
    exec(src, ns)
    c = [F(consts[n]) for n in p.cnames]
    ci = [U.f2b(v) for v in c]
    seeds = [8000 + i for i in range(k1.MC_RUNS)]
    RATE, DT, COUNTS, R_TRUE, R_NOM = k1.RATE, k1.DT, k1.COUNTS, k1.R_TRUE, k1.R_NOM
    truth = base.TRUTH
    idx = np.arange(0, truth["t"].size, base.TRUTH_HZ // RATE)
    t_s = truth["t"][idx]
    base.wrap = e.wrap
    data = []
    for seed in seeds:                                   # exactly sim1k's sensor generation
        rng = np.random.default_rng(seed)
        pos_err, head_err = base.vn_errors(rng, "drift", t_s)
        zx, zy = truth["x"][idx] + pos_err[0], truth["y"][idx] + pos_err[1]
        zth = e.wrap(truth["th"][idx] + head_err)
        zw = truth["w"][idx] + rng.uniform(-P.VN_GYRO_BIAS, P.VN_GYRO_BIAS) + P.VN_GYRO_SIGMA * rng.standard_normal(idx.size)
        phi = truth["s"][idx] / R_TRUE
        code = np.floor((phi + P.AMT_ACCURACY * np.sin(phi + rng.uniform(0, 2 * np.pi))) / (2 * np.pi) * COUNTS).astype(np.int64)
        data.append(dict(zx=zx, zy=zy, zth=zth, zw=zw, code=code))
    n_beats = idx.size if len(sys.argv) < 3 else min(idx.size, int(sys.argv[2]))
    R = len(seeds)
    # ---- reference filter (the simulation's), per run
    ref = np.zeros((R, n_beats, 5))
    dQ, dR, d0 = np.array(info["dQ"]), np.array(info["dR"]), np.array(info["d0"])
    for r, d in enumerate(data):
        ukf = e.MixedUKF(5, base.ident, base.ident)
        xu = np.array([d["zx"][0], d["zy"][0], d["zth"][0], d["zw"][0], R_NOM]); Sm = np.diag(d0)
        ref[r, 0] = xu
        ang = d["code"] * (2 * np.pi / COUNTS)
        for k in range(1, n_beats):
            dphi = ang[k] - ang[k - 1]
            def f(X):
                Y = X.copy(); mid = X[:, 2] + 0.5 * X[:, 3] * DT
                Y[:, 0] += X[:, 4] * dphi * np.cos(mid); Y[:, 1] += X[:, 4] * dphi * np.sin(mid); Y[:, 2] += X[:, 3] * DT
                return Y
            xu, Sm = ukf.predict(xu, Sm, f, np.diag(dQ))
            xu, Sm = ukf.update(xu, Sm, np.array([d["zx"][k], d["zy"][k], d["zth"][k], d["zw"][k]]), [0, 1, 2, 3], np.diag(dR))
            ref[r, k] = xu
        assert ukf.fail == 0
    # ---- the program, all runs at once
    lat0, lon0, DEG = 42.2936, -83.7166, math.pi / 180
    def ext(k):
        yaw = (90.0 - np.degrees(np.array([d["zth"][k] for d in data]))).astype(F)
        rate = (-np.array([d["zw"][k] for d in data])).astype(F)
        lat = lat0 + np.array([d["zy"][k] - d["zy"][0] for d in data]) / (info["r_north"] * DEG)
        lon = lon0 + np.array([d["zx"][k] - d["zx"][0] for d in data]) / (info["r_east"] * DEG * math.cos(lat0 * DEG))
        lat_b, lon_b = lat.view(np.int64), lon.view(np.int64)
        latf = np.array([U.b2f(U.double_to_single_bits(int(v))) for v in lat_b], dtype=F)
        dw = np.array([d["code"][k] - d["code"][k - 1] if k else 0 for d in data], dtype=np.int64)
        return [yaw, rate, (lat_b >> 20) & 0x3FFFFFF, (lon_b >> 20) & 0x3FFFFFF, dw, np.full(R, 2500 * k), latf, None]
    m = [np.zeros(R, dtype=F) for _ in range(U.WORDS)]
    hw = np.zeros((R, n_beats, 5))
    with np.errstate(all="ignore"):
        ns["beat_init"](m, ext(0), c, ci, isub, np)
        hw[:, 0] = np.stack([np.broadcast_to(m[i], (R,)) for i in U.XH], axis=1)
        for k in range(1, n_beats):
            ns["beat"](m, ext(k), c, ci, isub, np)
            hw[:, k] = np.stack([np.broadcast_to(m[i], (R,)) for i in U.XH], axis=1)
    assert np.all(np.isfinite(hw)), "the program produced a non-finite state"
    for r, d in enumerate(data):                         # the chip's origin is the first fix
        hw[r, :, 0] += d["zx"][0]; hw[r, :, 1] += d["zy"][0]
    # ---- errors against the truth, after the simulation's burn-in
    k0 = int(base.BURN_IN_S * RATE)
    out = dict(noise=noise, beats=n_beats, ops_per_beat=nops)
    sl = slice(k0, n_beats)
    ti = idx[sl]
    for name, est in (("reference", ref), ("program", hw)):
        ex = truth["x"][ti][None, :] - est[:, sl, 0]; ey = truth["y"][ti][None, :] - est[:, sl, 1]
        eh = e.wrap(truth["th"][ti][None, :] - est[:, sl, 2]); ew = truth["w"][ti][None, :] - est[:, sl, 3]
        out[name] = dict(pos=float(np.sqrt(np.mean(ex ** 2 + ey ** 2))), head=float(np.degrees(np.sqrt(np.mean(eh ** 2)))),
                         rate=float(np.degrees(np.sqrt(np.mean(ew ** 2)))),
                         r_err_pct=[float(100 * (est[r, -1, 4] / R_TRUE - 1)) for r in range(R)])
    g = hw[:, 1:] - ref[:, 1:]
    out["gap"] = dict(pos_mm=float(1000 * np.max(np.hypot(g[:, :, 0], g[:, :, 1]))), head_deg=float(np.degrees(np.max(np.abs(e.wrap(g[:, :, 2]))))),
                      rate_dps=float(np.degrees(np.max(np.abs(g[:, :, 3])))), r_um=float(1e6 * np.max(np.abs(g[:, :, 4]))))
    return out


if __name__ == "__main__":
    with Pool(2) as pool:
        res = pool.map(run, ["scaled", "as specified"])
    json.dump(res, open("mc_result.json", "w"), indent=1)
    for o in res:
        print("%s noise, %d beats, %d operations per beat" % (o["noise"], o["beats"], o["ops_per_beat"]))
        for name in ("reference", "program"):
            a = o[name]
            print("  %-10s position %.4f m   heading %.4f deg   rate %.4f deg/s   radius error at the end %s %%" % (name, a["pos"], a["head"], a["rate"], " ".join("%+.2f" % v for v in a["r_err_pct"])))
        print("  largest gap between the two at any beat: position %.3f mm, heading %.5f deg, rate %.5f deg/s, radius %.3f um" % (o["gap"]["pos_mm"], o["gap"]["head_deg"], o["gap"]["rate_dps"], o["gap"]["r_um"]))
