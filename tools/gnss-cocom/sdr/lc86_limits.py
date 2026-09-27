#!/usr/bin/env python3
"""When the LC86G gives measurements through a flight: GPS satellites in each RTCM MSM7
epoch, its own $PQTMPVT fix (speed, altitude) against the injection, and the spans where
it sends nothing at all -- the LC86G enforces its limits by muting ALL output. The
LC86G counterpart of px_limits.py, on lc86_bench_run.py captures.

    ./lc86_limits.py CAPTURE OUT.png SHIFT SCENARIO      # CAPTURE plain or .gz
    ./lc86_limits.py results/lc86g_20260927_gentle_alt_pad600_smooth_balloon_msm7.log.gz out.png 420 gentle_alt

The scenario JSON is read from scenarios/ (gitignored): regenerate it with
./make_flights.py --lat 0 --lon -119 --only gentle_alt first.

SHIFT is the pad minus the scenario's 180 s prologue (420 for *_pad600). Times are file
time = GPS TOW - 203400, read from the MSM7 epoch and the $PQTMPVT TOW themselves; the
silent spans come from host time, mapped with the median host - TOW offset.
"""
import bisect
import gzip
import json
import math
import re
import statistics as stt
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                   # noqa: E402

SDR = Path(__file__).resolve().parent
sys.path.insert(0, str(SDR.parent))
import rtcm3                                                      # noqa: E402

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
    """The capture as text, plain or gzip'd (results/ keeps them gzip'd)."""
    op = gzip.open if path.suffix == ".gz" else open
    with op(path, "rt", errors="replace") as fh:
        yield from fh


hdr = next(lines(cap))
m = re.search(r"tx (\S+)\.C8 gain (\d+)", hdr)
IQNAME, GAIN = (m.group(1), m.group(2)) if m else (SCEN, "?")
msm = []           # (file t, satellites measured, median C/N0)
pvt = []           # (host t, file t, valid, fixmode, used, speed, alt)
host_all = []      # host t of every line the receiver sent, bar its aiding requests
aiding = []        # host t of $PAIR010 aiding requests: once a minute, even while muted
for line in lines(cap):
    p = line.rstrip("\n").split(" ", 2)
    if len(p) < 2 or p[1] == "#":
        continue
    try:
        h = float(p[0])
    except ValueError:
        continue
    if p[1].startswith("$PAIR010"):          # week + TOW only: not position, not measurements
        aiding.append(h)
        continue
    host_all.append(h)
    if p[1] == "R" and len(p) == 3:
        body = bytes.fromhex(p[2])
        if rtcm3.message_type(body) == 1077:
            r = rtcm3.parse_msm7(body)
            if r:
                _const, epoch_ms, cells = r
                sats = {c["prn"] for c in cells if c.get("cn0")}
                cn = [c["cn0"] for c in cells if c.get("cn0")]
                msm.append((epoch_ms / 1000.0 - TOW0, len(sats), stt.median(cn) if cn else 0.0))
    elif p[1].startswith("$PQTMPVT"):
        f = (p[1] + (" " + p[2] if len(p) == 3 else "")).split("*")[0].split(",")
        try:
            ft = int(f[2]) / 1000.0 - TOW0
            ok, fm, used = int(f[5] or 0), int(f[6] or 0), int(f[7] or 0)
        except (ValueError, IndexError):
            continue
        valid = ok > 0 and fm >= 2
        try:
            alt = float(f[11]) + float(f[12] or 0.0)                    # ellipsoidal, like the injection
            sp = math.sqrt(sum(float(f[k] or 0.0) ** 2 for k in (13, 14, 15)))
        except (ValueError, IndexError):
            alt = sp = float("nan")
        pvt.append((h, ft, valid, fm, used, sp, alt))

offs = [h - ft for h, ft, valid, *_ in pvt if ft > 0]
off = stt.median(offs) if offs else 0.0
print(f"{cap.name}: {len(msm)} MSM7 epochs, {len(pvt)} PQTMPVT; host - file = {off:.3f} s")

silent = [(a - off, b - off) for a, b in zip(host_all, host_all[1:]) if b - a > 0.5]
print("\nno output at all (file time; $PAIR010 aiding requests aside):")
for a, b in silent:
    n = sum(1 for h in aiding if a < h - off < b)
    print(f"  {a:7.2f} -> {b:7.2f} s ({b - a:5.1f} s)   injected {at(a, 'speed_mps'):6.1f} m/s, "
          f"{at(a, 'alt_m') / 1000:5.1f} km at the start -> {at(b, 'speed_mps'):6.1f} m/s, "
          f"{at(b, 'alt_m') / 1000:5.1f} km at the end" + (f"; {n} aiding request(s) inside" if n else ""))

mt = [t for t, *_ in msm]
print("\nMSM7 epoch gaps > 1.5 s:")
for a, b in zip(mt, mt[1:]):
    if b - a > 1.5:
        print(f"  {a:7.2f} -> {b:7.2f} s ({b - a:5.1f} s)")
first = next((t for t, n, _c in msm if n >= 4), None)
print(f"first MSM7 epoch with 4+ satellites: {first}")

print("\nfix changes (file time, from $PQTMPVT):")
prev = None
for h, ft, valid, fm, used, sp, alt in pvt:
    if valid != prev:
        print(f"  {ft:7.2f} s  {'FIX' if valid else 'no fix'} (mode {fm}, {used} used)  own speed {sp:6.1f} "
              f"(true {at(ft, 'speed_mps'):6.1f}) m/s  own alt {alt / 1000:6.2f} (true {at(ft, 'alt_m') / 1000:6.2f}) km")
        prev = valid

print("\ninjected crossings:")
for key, thr, name in (("speed_mps", 500.0, "500 m/s"), ("alt_m", 80000.0, "80 km"), ("alt_m", 18000.0, "18 km")):
    print(f"  {name:>8}: " + ", ".join(f"{t:.1f} s {d}" for t, d in crossings(key, thr)))

fig, axs = plt.subplots(3, 1, figsize=(11, 7.6), dpi=140, sharex=True, gridspec_kw=dict(hspace=0.12))
INK, INK3, RULE, SIL = "#1d2129", "#6b7280", "#e3e6ea", "#c9ced6"
ax = axs[0]
ax.plot(mt, [n for _t, n, _c in msm], ".", ms=2.2, color="#2a78d6")
ax.set_ylabel("GPS satellites\nper MSM7 epoch", fontsize=8, color=INK3)
ok = [(ft, sp, a) for _h, ft, valid, _fm, _u, sp, a in pvt if valid]
ax = axs[1]
ax.plot([t for t, *_ in ok], [s for _t, s, _a in ok], ".", ms=1.2, color="#eb6834", label="own speed while fixed ($PQTMPVT)")
tx = [t / 10 for t in range(int(10 * (IGN - 80)), int(10 * T_END))]
ax.plot(tx, [at(t, "speed_mps") for t in tx], color=INK3, lw=1.0, label="injected")
ax.axhline(500, color=INK3, lw=0.6, ls=(0, (2, 2)))
ax.set_ylabel("speed, m/s", fontsize=8, color=INK3)
ax.legend(fontsize=7, frameon=False, loc="upper right")
ax = axs[2]
ax.plot([t for t, *_ in ok], [a / 1000 for _t, _s, a in ok], ".", ms=1.2, color="#1baf7a", label="own altitude while fixed ($PQTMPVT)")
ax.plot(tx, [at(t, "alt_m") / 1000 for t in tx], color=INK3, lw=1.0, label="injected")
ax.axhline(80, color=INK3, lw=0.6, ls=(0, (2, 2)))
ax.axhline(18, color=INK3, lw=0.6, ls=(0, (1, 3)))
ax.set_ylabel("altitude, km", fontsize=8, color=INK3)
ax.set_xlabel("file time (s)", fontsize=8, color=INK3)
ax.legend(fontsize=7, frameon=False, loc="upper left")
for a in axs:
    for key, thr, col, alpha in (("speed_mps", 500.0, "#eda100", 0.10), ("alt_m", 80000.0, "#7b61ff", 0.08)):
        cs = crossings(key, thr)
        ups = [t for t, d in cs if d == "up"]
        downs = [t for t, d in cs if d == "down"]
        for t0 in ups:
            a.axvspan(t0, min(next((d for d in downs if d > t0), 1e9), T_END), color=col, alpha=alpha, lw=0)
    for s0, s1 in silent:
        if s1 - s0 > 1.0:
            a.axvspan(s0, s1, ymin=0.0, ymax=0.05, color=SIL, lw=0)
    for t, _d in crossings("alt_m", 18000.0):
        a.axvline(t, color=INK3, lw=0.6, ls=(0, (1, 3)))
    a.axvline(IGN, color=INK3, lw=0.6)
    a.tick_params(labelsize=7, colors=INK3)
    a.grid(color=RULE, lw=0.5)
    for sp in a.spines.values():
        sp.set_color(RULE)
axs[0].set_xlim(IGN - 80, T_END)
axs[0].set_title(f"LC86G (Balloon, 10 Hz, RTCM MSM7), {IQNAME}, gain {GAIN}; amber = above 500 m/s, "
                 "violet = above 80 km, dotted = 18 km, grey foot = no output at all",
                 fontsize=9, color=INK, loc="left")
fig.savefig(out, bbox_inches="tight")
print("wrote", out)
