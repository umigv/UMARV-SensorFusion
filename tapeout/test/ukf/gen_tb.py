"""Stimulus and expected storage contents for tb_ukf.sv, from the exact model of the machine."""
import math, struct, sys
import numpy as np
import ukf_prog as U
consts, info = U.constants()
p = U.build(consts); prog = p.resolve()
U.write_verilog(prog, consts, p.cnames, "ukf_tables.sv")
mc = U.Machine(prog, consts, p.cnames)
rng = np.random.default_rng(3)
lat0, lon0, DEG = 42.2936, -83.7166, math.pi / 180
th, x, y, v, w = math.radians(140.0), 0.0, 0.0, 1.8, 0.6
N = int(sys.argv[1]) if len(sys.argv) > 1 else 14
stim, exp = [], []
# Before beat RECONF the testbench switches the filter off, writes these tunable constants the way
# the host would, and switches it on again (so the filter starts afresh from the next good fix).
RECONF = 7
changes = [("RR01", consts["RR01"] * 4), ("RR2", consts["RR2"] * 0.5), ("SQ3", consts["SQ3"] * 2), ("CNT2RAD", 2 * math.pi / 16000),
           ("RNOM", 0.051), ("YAWK", consts["YAWK"]), ("HZ", consts["HZ"])]
cfg = []
for k in range(N):
    if k == RECONF:
        mc.inited = False
        for name, value in changes:
            mc.set_tunable(name, value)
            cfg.append("%02x%08x" % (U.CFG_BASE + U.TUNABLE.index(name), U.f2b(np.float32(value))))
    vn = k not in (0, 6, 7); dw = k not in (8,)
    yaw = 90 - math.degrees(th) + rng.normal() * 0.5
    yaw = (yaw + 180) % 360 - 180
    lat = lat0 + (y + rng.normal() * 0.7) / (info["r_north"] * DEG)
    lon = lon0 + (x + rng.normal() * 0.7) / (info["r_east"] * DEG * math.cos(lat0 * DEG))
    counts = int(round(v / 400 / 0.05075 / (2 * math.pi) * 16384)) * (-1 if k == 10 else 1)
    rate = -(w + rng.normal() * 0.001)
    ts = 2500 * (k + 1)
    ext = U.ext_words(yaw, rate, lat, lon, counts, ts)
    produced = mc.beat(ext, vn, dw)
    f32 = lambda f: struct.unpack("<I", struct.pack("<f", float(np.float32(f))))[0]
    stim.append("%02x%08x%08x%016x%016x%08x%08x" % ((int(vn) << 1) | int(dw), f32(yaw), f32(rate), U.double_bits(lat), U.double_bits(lon), counts & 0xFFFFFFF, ts))
    mask = sum(1 << i for i in mc.written)
    exp.append("%01x%016x%s" % (int(produced), mask, "".join("%08x" % (mc.mem[i] if i < U.WORDS else 0) for i in reversed(range(64)))))
    x += v / 400 * math.cos(th); y += v / 400 * math.sin(th); th += w / 400
open("cfg.hex", "w").write("\n".join(cfg) + "\n")
# for the read-back check: every constant's reset value, by address offset from 64
open("consts.hex", "w").write("\n".join("%08x" % U.f2b(np.float32(consts[n])) for n in p.cnames) + "\n")
open("stim.hex", "w").write("\n".join(stim) + "\n"); open("expect.hex", "w").write("\n".join(exp) + "\n")
print("beats %d, steps in table %d, deepest call %d, operations by kind %s" % (N, len(prog), mc.max_depth, dict(zip(U.FN_NAME, mc.ops))))
print("last state: x %.4f y %.4f theta %.4f omega %.4f v %.4f r %.5f" % tuple(float(U.b2f(mc.mem[i])) for i in (1, 2, 3, 4, 5, U.XH[4])))
