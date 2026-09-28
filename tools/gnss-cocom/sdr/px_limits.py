#!/usr/bin/env python3
"""Where the PX1105R's output stops against the injected spaceshot: raw 0xE5 epochs
(measurement count, C/N0) and its own 0xDF fix (state, speed, altitude) per second,
the gaps in each, and a timeline figure."""
import bisect
import gzip
import json
import math
import struct
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                   # noqa: E402

SDR = Path(__file__).resolve().parent
TOW0 = 203400.0
cap = Path(sys.argv[1]) if len(sys.argv) > 1 else SDR / "captures" / "px1105r_smooth460_spaceshot.log"
out = sys.argv[2] if len(sys.argv) > 2 else "px_limits.png"
SHIFT = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0     # extra pad in front of the flight, s
SCEN = sys.argv[4] if len(sys.argv) > 4 else "spaceshot"

_sc = json.loads((SDR / "scenarios" / f"{SCEN}.json").read_text())
tr = _sc["truth"]
IGN = _sc.get("prologue_s", 180.0) + SHIFT
tt = [s["t"] for s in tr]


def at(x, k):
    x -= SHIFT                         # the flight is the original one, SHIFT s later
    i = min(max(bisect.bisect_left(tt, x), 1), len(tt) - 1)
    a, b = tr[i - 1], tr[i]
    f = (x - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0.0
    return a[k] + f * (b[k] - a[k])


def crossings(key, thr):
    out = []
    for a, b in zip(tr, tr[1:]):
        if (a[key] - thr) * (b[key] - thr) < 0:
            out.append((SHIFT + a["t"] + (thr - a[key]) / (b[key] - a[key]) * (b["t"] - a["t"]),
                        "up" if b[key] > a[key] else "down"))
    return out


def lla(x, y, z):
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - e2))
    for _ in range(6):
        n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - e2 * n / (n + h)))
    return math.degrees(lat), math.degrees(lon), h


import re as _re
_open = (lambda: gzip.open(cap, "rt", errors="replace")) if cap.suffix == ".gz" else \
    (lambda: open(cap, errors="replace"))     # results/ keeps the captures gzip'd
_hdr = _open().readline()
_m = _re.search(r"tx (\S+)\.C8", _hdr)
IQNAME = _m.group(1) if _m else f"smooth {SCEN}"      # the IQ file this capture played
e5, df = [], []           # (host t, file t, ...)
events = []               # (host t, text): what the runner did mid-run, e.g. a hot start
for line in _open():
    p = line.split(" ", 2)
    if len(p) >= 3 and p[1] == "#" and p[2].startswith("host: ") and not p[2].startswith("host: tx "):
        events.append((float(p[0]), p[2][len("host: "):].strip()))
        continue
    if len(p) < 3 or p[1] != "B":
        continue
    x = bytes.fromhex(p[2].strip())
    ht = float(p[0])
    if x[0] == 0xE5 and len(x) >= 14:
        tow = struct.unpack(">I", x[5:9])[0] / 1000.0
        n = x[13]
        cn = [x[14 + 31 * i + 3] for i in range(n) if len(x) >= 14 + 31 * (i + 1)]
        e5.append((ht, tow - TOW0, n, cn))
    elif x[0] == 0xDF and len(x) >= 61:
        st = x[2]
        tow = struct.unpack(">d", x[5:13])[0]
        X, Y, Z = struct.unpack(">ddd", x[13:37])
        vx, vy, vz = struct.unpack(">fff", x[37:49])
        alt = lla(X, Y, Z)[2] if (X or Y or Z) else float("nan")
        df.append((ht, tow - TOW0 if tow else None, st, math.sqrt(vx * vx + vy * vy + vz * vz), alt))

# host -> file time offset from frames that carry both
offs = sorted(h - f for h, f, *_ in e5 if f is not None) or sorted(h - f for h, f, *_ in df if f)
off = offs[len(offs) // 2]
print(f"capture {cap.name}: {len(e5)} raw epochs, {len(df)} fix frames; host - file = {off:.3f} s")
events = [(h - off, txt.split(" sent")[0]) for h, txt in events]
for ft, name in events:
    print(f"  host event at file {ft:7.2f} s: {name}")


def gaps(times, min_gap=0.5):
    out = []
    for a, b in zip(times, times[1:]):
        if b - a > min_gap:
            out.append((a, b))
    return out


e5t = [f for _h, f, *_ in e5]
print("\nraw 0xE5 epochs: first %.2f s, last %.2f s" % (e5t[0], e5t[-1]) if e5t else "no raw epochs")
for a, b in gaps(e5t):
    print(f"  raw output GAP {a:7.2f} -> {b:7.2f} s ({b - a:5.1f} s)")
# frames flow by host time even when the fix time is zero: use host time for DF
dft = [h - off for h, *_ in df]
for a, b in gaps(dft):
    print(f"  fix-frame GAP  {a:7.2f} -> {b:7.2f} s ({b - a:5.1f} s)")
fixed = [(h - off, st, sp, alt) for h, f, st, sp, alt in df if st >= 2]
prev = None
print("\nfix state changes (file time, from host clock):")
for h, f, st, sp, alt in df:
    if st != prev:
        ft = h - off
        print(f"  {ft:7.2f} s  state {st}  own speed {sp:7.1f} m/s (true {at(ft, 'speed_mps'):7.1f})  "
              f"own alt {alt / 1000 if alt == alt else float('nan'):6.2f} km (true {at(ft, 'alt_m') / 1000:6.2f})")
        prev = st

print("\ninjected crossings:")
for key, thr, name in (("speed_mps", 500.0, "500 m/s"), ("speed_mps", 515.0, "515 m/s"),
                       ("alt_m", 18000.0, "18 km"), ("alt_m", 80000.0, "80 km")):
    print(f"  {name:>8}: " + ", ".join(f"{t:.1f} s {d}" for t, d in crossings(key, thr) if t < 1000))

# figure
fig, axs = plt.subplots(3, 1, figsize=(11, 7.6), dpi=140, sharex=True,
                        gridspec_kw=dict(height_ratios=[1.0, 1.0, 1.0], hspace=0.12))
INK, INK3, RULE = "#1d2129", "#6b7280", "#e3e6ea"
ax = axs[0]
ax.plot([f for _h, f, n, _c in e5], [n for _h, _f, n, _c in e5], ".", ms=1.5, color="#2a78d6",
        rasterized=True)          # dense point clouds rasterize, so an .svg stays small
ax.set_ylabel("raw measurements\nper 0xE5 epoch", fontsize=8, color=INK3)
ax = axs[1]
ok = [(h - off, sp, a) for h, _f, st, sp, a in df if st >= 2]     # only while it has a fix
ax.plot([t for t, _s, _a in ok], [sp for _t, sp, _a in ok], ".", ms=1.2, color="#eb6834", label="own speed while fixed (0xDF)",
        rasterized=True)
tx = [t / 10 for t in range(int(10 * (IGN - 80)), int(10 * (IGN + 480)))]
ax.plot(tx, [at(t, "speed_mps") for t in tx], color=INK3, lw=1.0, label="injected")
ax.axhline(500, color=INK3, lw=0.6, ls=(0, (2, 2)))
ax.set_ylabel("speed, m/s", fontsize=8, color=INK3)
ax.legend(fontsize=7, frameon=False, loc="upper right")
ax = axs[2]
ax.plot([t for t, _s, _a in ok], [a / 1000 for _t, _s, a in ok], ".", ms=1.2, color="#1baf7a", label="own altitude while fixed (0xDF)",
        rasterized=True)
ax.plot(tx, [at(t, "alt_m") / 1000 for t in tx], color=INK3, lw=1.0, label="injected")
ax.axhline(80, color=INK3, lw=0.6, ls=(0, (2, 2)))
ax.axhline(18, color=INK3, lw=0.6, ls=(0, (1, 3)))
ax.set_ylabel("altitude, km", fontsize=8, color=INK3)
ax.set_xlabel("file time (s)", fontsize=8, color=INK3)
ax.legend(fontsize=7, frameon=False, loc="upper left")
for a in axs:
    for key, thr, col, alpha in (("speed_mps", 500.0, "#eda100", 0.10), ("alt_m", 80000.0, "#7b61ff", 0.08)):
        cs = crossings(key, thr)
        ups = [t for t, d in cs if d == "up"]; downs = [t for t, d in cs if d == "down"]
        for t0 in ups:
            t1 = next((d for d in downs if d > t0), 1e9)
            a.axvspan(t0, min(t1, 2000), color=col, alpha=alpha, lw=0)
    for t, _d in crossings("alt_m", 18000.0):
        a.axvline(t, color=INK3, lw=0.6, ls=(0, (1, 3)))
    a.axvline(IGN, color=INK3, lw=0.6)
    for ft, _name in events:
        a.axvline(ft, color=INK, lw=1.0, ls=(0, (4, 3)))
    a.tick_params(labelsize=7, colors=INK3)
    a.grid(color=RULE, lw=0.5)
    for sp in a.spines.values():
        sp.set_color(RULE)
for ft, name in events:
    axs[0].text(ft + 2, 9.2, f"{name} (file {ft:.1f} s)", fontsize=7.5, color=INK, va="top")
axs[0].set_xlim(IGN - 80, max(h - off for h, *_ in df) if df else IGN + 300)
axs[0].set_title(f"{cap.name.split('_')[0].upper()} (RTK kinematic base, raw 0xE5 at 20 Hz), {IQNAME}; "
                 "amber = above 500 m/s, violet = above 80 km, dotted = 18 km"
                 + ("; dashed = " + ", ".join(n for _t, n in events) if events else ""),
                 fontsize=9, color=INK, loc="left")
fig.savefig(out, bbox_inches="tight")
print("wrote", out)
