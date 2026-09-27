#!/usr/bin/env python3
"""When a u-blox receiver gives measurements through a flight: GPS satellites with a valid
pseudorange in each UBX-RXM-RAWX epoch, its own UBX-NAV-PVT fix (speed, altitude) against
the injection, and the epochs where NAV-PVT keeps coming but flags no fix -- the way a
u-blox withholds. The u-blox counterpart of px_limits.py / lc86_limits.py, on
run_radiated.py captures ('<t> U <class id payload hex>').

    ./ubx_limits.py CAPTURE OUT.png SHIFT SCENARIO        # CAPTURE plain or .gz
    ./ubx_limits.py captures/zed_f9p_raw20_gentle_alt_pad600_smooth.log out.png 420 gentle_alt

SHIFT is the pad minus the scenario's 180 s prologue (420 for *_pad600). Times are file time
= GPS TOW - 203400, read from RAWX rcvTow and NAV-PVT iTOW. The scenario JSON comes from
scenarios/ (gitignored: ./make_flights.py --lat 0 --lon -119 --only gentle_alt).
"""
import bisect
import gzip
import json
import math
import re
import statistics as stt
import struct
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                   # noqa: E402

SDR = Path(__file__).resolve().parent
TOW0 = 203400.0
cap, out = Path(sys.argv[1]), sys.argv[2]
SHIFT = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
SCEN = sys.argv[4] if len(sys.argv) > 4 else "spaceshot"

_sc = json.loads((SDR / "scenarios" / f"{SCEN}.json").read_text())
tr = _sc["truth"]
IGN = _sc.get("prologue_s", 180.0) + SHIFT
tt = [s["t"] for s in tr]
T_END = SHIFT + tt[-1]


def at(x, k):
    x -= SHIFT
    i = min(max(bisect.bisect_left(tt, x), 1), len(tt) - 1)
    a, b = tr[i - 1], tr[i]
    f = (x - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0.0
    return a[k] + f * (b[k] - a[k])


def crossings(key, thr):
    res = []
    for a, b in zip(tr, tr[1:]):
        if (a[key] - thr) * (b[key] - thr) < 0:
            res.append((SHIFT + a["t"] + (thr - a[key]) / (b[key] - a[key]) * (b["t"] - a["t"]),
                        "up" if b[key] > a[key] else "down"))
    return res


def lines(path):
    op = gzip.open if path.suffix == ".gz" else open
    with op(path, "rt", errors="replace") as fh:
        yield from fh


hdr = next(lines(cap))
m = re.search(r"tx (\S+)\.C8 gain (\d+)", hdr)
IQNAME, GAIN = (m.group(1), m.group(2)) if m else (SCEN, "?")
rawx = []          # (file t, GPS sats with valid pseudorange, GPS sats with valid carrier, median C/N0)
pvt = []           # (file t, valid, fixType, numSV, speed, height)
offs = []          # host - file time, from NAV-PVT, to place MON-COMMS (which has no GPS time)
comms = []         # (host t, txErrors, USB txPending, txUsage %, txPeakUsage %, overrunErrs, skipped)
for line in lines(cap):
    p = line.rstrip("\n").split(" ", 2)
    if len(p) < 3 or p[1] != "U":
        continue
    d = bytes.fromhex(p[2])
    if len(d) < 2:
        continue
    h = float(p[0])
    cls, mid, pl = d[0], d[1], d[2:]
    if cls == 0x0A and mid == 0x36 and len(pl) >= 8:                  # MON-COMMS
        for k in range(pl[1]):
            o = 8 + 40 * k
            if len(pl) < o + 40:
                break
            if struct.unpack_from("<H", pl, o)[0] == 0x0300:           # the USB port
                pend, = struct.unpack_from("<H", pl, o + 2)
                ovr, = struct.unpack_from("<H", pl, o + 18)
                skip, = struct.unpack_from("<I", pl, o + 36)
                comms.append((h, pl[2], pend, pl[o + 8], pl[o + 9], ovr, skip))
        continue
    if cls == 0x02 and mid == 0x15 and len(pl) >= 16:                 # RXM-RAWX
        rcv_tow = struct.unpack_from("<d", pl, 0)[0]
        n = pl[11]
        pr_ok, cp_ok, cn = set(), set(), []
        for i in range(n):
            o = 16 + 32 * i
            if len(pl) < o + 32:
                break
            gnss, sv, cno, trk = pl[o + 20], pl[o + 21], pl[o + 26], pl[o + 30]
            if gnss != 0:                                               # GPS only is injected
                continue
            if trk & 0x01:
                pr_ok.add(sv)
                cn.append(cno)
            if trk & 0x02:
                cp_ok.add(sv)
        rawx.append((rcv_tow - TOW0, len(pr_ok), len(cp_ok), stt.median(cn) if cn else 0.0))
    elif cls == 0x01 and mid == 0x07 and len(pl) >= 60:               # NAV-PVT
        itow = struct.unpack_from("<I", pl, 0)[0] / 1000.0
        fix_type, flags, num_sv = pl[20], pl[21], pl[23]
        height = struct.unpack_from("<i", pl, 32)[0] / 1000.0          # above the ellipsoid
        vn, ve, vd = (struct.unpack_from("<i", pl, k)[0] / 1000.0 for k in (48, 52, 56))
        valid = bool(flags & 0x01) and fix_type >= 2
        pvt.append((itow - TOW0, valid, fix_type, num_sv, math.sqrt(vn * vn + ve * ve + vd * vd), height))
        if itow > 1.0:
            offs.append(h - (itow - TOW0))

rt = [t for t, *_ in rawx]
pt = [t for t, *_ in pvt]
dt_r = stt.median([b - a for a, b in zip(rt, rt[1:])]) if len(rt) > 1 else 0
dt_p = stt.median([b - a for a, b in zip(pt, pt[1:])]) if len(pt) > 1 else 0
print(f"{cap.name}: {len(rawx)} RAWX epochs ({1 / dt_r if dt_r else 0:.1f} Hz), "
      f"{len(pvt)} NAV-PVT ({1 / dt_p if dt_p else 0:.1f} Hz)")

print("\nRAWX epoch gaps > 0.5 s (file time):")
for a, b in zip(rt, rt[1:]):
    if b - a > 0.5:
        print(f"  {a:7.2f} -> {b:7.2f} s ({b - a:5.1f} s)")
print("RAWX epochs with no valid GPS pseudorange, in spans:")
span = None
for t, n, _c, _cn in rawx + [(1e9, 99, 0, 0)]:
    if n == 0 and span is None:
        span = t
    elif n > 0 and span is not None:
        print(f"  {span:7.2f} -> {t:7.2f} s")
        span = None

print("\nfix changes (file time, NAV-PVT gnssFixOK):")
prev = None
for t, valid, ft, nsv, sp, h in pvt:
    if valid != prev:
        print(f"  {t:7.2f} s  {'FIX' if valid else 'no fix'} (type {ft}, {nsv} SV)  own speed {sp:6.1f} "
              f"(true {at(t, 'speed_mps'):6.1f}) m/s  own alt {h / 1000:6.2f} (true {at(t, 'alt_m') / 1000:6.2f}) km")
        prev = valid

if comms:
    off = stt.median(offs) if offs else 0.0
    alloc = sum(1 for c in comms if c[1] & 0x02)
    print(f"\nMON-COMMS, USB port ({len(comms)} reports): TX-buffer-full flag in {alloc}; "
          f"peak usage {max(c[4] for c in comms)} %; max pending {max(c[2] for c in comms)} B; "
          f"overrun slots {comms[-1][5]}; skipped {comms[-1][6]} B")
    print("  per minute (file time): max usage %, max pending B, TX-full reports, new overrun slots")
    by_min = {}
    for h, err, pend, use, _peak, ovr, _skip in comms:
        by_min.setdefault(int((h - off) // 60), []).append((err, pend, use, ovr))
    for mnt in sorted(by_min):
        r = by_min[mnt]
        print(f"  {60 * mnt:5d}-{60 * mnt + 60:<5d} s  {max(x[2] for x in r):3d}  {max(x[1] for x in r):6d}  "
              f"{sum(1 for x in r if x[0] & 0x02):3d}  {r[-1][3] - r[0][3]:4d}")

print("\ninjected crossings:")
for key, thr, name in (("speed_mps", 515.0, "515 m/s"), ("speed_mps", 500.0, "500 m/s"),
                       ("alt_m", 80000.0, "80 km"), ("alt_m", 18000.0, "18 km")):
    print(f"  {name:>8}: " + ", ".join(f"{t:.1f} s {d}" for t, d in crossings(key, thr)))

fig, axs = plt.subplots(3, 1, figsize=(11, 7.6), dpi=140, sharex=True, gridspec_kw=dict(hspace=0.12))
INK, INK3, RULE, NOFIX = "#1d2129", "#6b7280", "#e3e6ea", "#c9ced6"
ax = axs[0]
ax.plot(rt, [n for _t, n, _c, _cn in rawx], ".", ms=1.2, color="#2a78d6", label="valid pseudorange")
ax.plot(rt, [c for _t, _n, c, _cn in rawx], ".", ms=0.8, color="#7fb2ea", label="valid carrier phase")
ax.set_ylabel("GPS satellites\nper RAWX epoch", fontsize=8, color=INK3)
ax.legend(fontsize=7, frameon=False, loc="lower right")
ok = [(t, sp, h) for t, valid, _f, _n, sp, h in pvt if valid]
ax = axs[1]
ax.plot([t for t, *_ in ok], [s for _t, s, _h in ok], ".", ms=1.0, color="#eb6834", label="own speed while fixed (NAV-PVT)")
tx = [t / 10 for t in range(int(10 * (IGN - 80)), int(10 * T_END))]
ax.plot(tx, [at(t, "speed_mps") for t in tx], color=INK3, lw=1.0, label="injected")
ax.axhline(515, color=INK3, lw=0.6, ls=(0, (2, 2)))
ax.set_ylabel("speed, m/s", fontsize=8, color=INK3)
ax.legend(fontsize=7, frameon=False, loc="upper right")
ax = axs[2]
ax.plot([t for t, *_ in ok], [h / 1000 for _t, _s, h in ok], ".", ms=1.0, color="#1baf7a", label="own altitude while fixed (NAV-PVT)")
ax.plot(tx, [at(t, "alt_m") / 1000 for t in tx], color=INK3, lw=1.0, label="injected")
ax.axhline(80, color=INK3, lw=0.6, ls=(0, (2, 2)))
ax.axhline(18, color=INK3, lw=0.6, ls=(0, (1, 3)))
ax.set_ylabel("altitude, km", fontsize=8, color=INK3)
ax.set_xlabel("file time (s)", fontsize=8, color=INK3)
ax.legend(fontsize=7, frameon=False, loc="upper left")
nofix, s0 = [], None
for t, valid, *_ in pvt + [(1e9, True)]:
    if not valid and s0 is None:
        s0 = t
    elif valid and s0 is not None:
        nofix.append((s0, min(t, T_END)))
        s0 = None
for a in axs:
    for key, thr, col, alpha in (("speed_mps", 515.0, "#eda100", 0.10), ("alt_m", 80000.0, "#7b61ff", 0.08)):
        cs = crossings(key, thr)
        ups = [t for t, d in cs if d == "up"]
        downs = [t for t, d in cs if d == "down"]
        for t0 in ups:
            a.axvspan(t0, min(next((d for d in downs if d > t0), 1e9), T_END), color=col, alpha=alpha, lw=0)
    for s0, s1 in nofix:
        if s1 - s0 > 0.5:
            a.axvspan(s0, s1, ymin=0.0, ymax=0.05, color=NOFIX, lw=0)
    for t, _d in crossings("alt_m", 18000.0):
        a.axvline(t, color=INK3, lw=0.6, ls=(0, (1, 3)))
    a.axvline(IGN, color=INK3, lw=0.6)
    a.tick_params(labelsize=7, colors=INK3)
    a.grid(color=RULE, lw=0.5)
    for sp in a.spines.values():
        sp.set_color(RULE)
axs[0].set_xlim(IGN - 80, T_END)
name = cap.name.split("_raw")[0].upper().replace("_", "-")
axs[0].set_title(f"{name} (UBX RAWX + NAV-PVT, {1 / dt_p if dt_p else 0:.0f} Hz), {IQNAME}, gain {GAIN}; "
                 "amber = above 515 m/s, violet = above 80 km, grey foot = NAV-PVT with no fix",
                 fontsize=9, color=INK, loc="left")
fig.savefig(out, bbox_inches="tight")
print("wrote", out)
