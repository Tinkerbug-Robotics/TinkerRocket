#!/usr/bin/env python3
"""A NEO-M8T flight of a wide SignalSim file (run_radiated capture 't U <cls><id><payload hex>', ignition at GPS TOW
204000): per system (GPS / Galileo / BeiDou) the pad C/N0 (RXM-RAWX median, T-60..T-5) and satellites with a valid
pseudorange on the pad, at ignition and through the burn until the M8T withholds raw (it stops RAWX above 515 m/s);
the last RAWX epoch before that, when raw resumes, and the fix: last NAV-PVT fix after ignition (own vs true speed)
and when the fix comes back.            m8t_summary.py CAPTURE [SCENARIO=hotshot]
"""
import bisect
import json
import statistics as st
import struct
import sys
from pathlib import Path

SDR = Path(__file__).resolve().parents[1]
IGN_TOW = 204000.0
SYS = {0: "GPS", 2: "GAL", 3: "BDS"}
cap = sys.argv[1]
scen = sys.argv[2] if len(sys.argv) > 2 else "hotshot"
sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
tr, PRO = sc["truth"], sc.get("prologue_s", 180.0)
tt = [s["t"] for s in tr]
BURN = next(s["t"] for s in tr if s["phase"] == "coast") - PRO
UNDER = sc["velocity_windows"][-1][1] - PRO
OVER = sc["velocity_windows"][0][0] - PRO


def vtrue(t):
    x = t + PRO
    i = min(max(bisect.bisect_left(tt, x), 1), len(tt) - 1)
    a, b = tr[i - 1], tr[i]
    f = (x - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0.0
    return a["speed_mps"] + f * (b["speed_mps"] - a["speed_mps"])


raw, pvt = [], []                      # raw: (T, {sys: {sv: cno}}); pvt: (T, fixType, ok, speed)
for line in open(cap, errors="replace"):
    p = line.split(" ", 2)
    if len(p) < 3 or p[1] != "U":
        continue
    h = p[2].strip()
    try:
        b = bytes.fromhex(h[4:])
    except ValueError:
        continue
    if h.startswith("0215") and len(b) >= 16:                     # RXM-RAWX
        tow, n = struct.unpack("<d", b[0:8])[0], b[11]
        sats = {}
        for j in range(n):
            m = b[16 + 32 * j: 48 + 32 * j]
            if len(m) < 32:
                break
            g, sv, cno, trk = m[20], m[21], m[26], m[30]
            if g in SYS and trk & 1:                                # prValid
                sats.setdefault(SYS[g], {})[sv] = cno
        raw.append((round(tow - IGN_TOW, 2), sats))
    elif h.startswith("0107") and len(b) >= 64:                   # NAV-PVT
        itow = struct.unpack("<I", b[0:4])[0] / 1000.0
        vn, ve, vd = struct.unpack("<iii", b[48:60])
        pvt.append((round(itow - IGN_TOW, 2), b[20], b[21] & 1, (vn * vn + ve * ve + vd * vd) ** 0.5 / 1000.0))
rt = [r[0] for r in raw]


def at(t):
    i = bisect.bisect_right(rt, t + 1e-6) - 1
    return raw[i] if i >= 0 else (None, {})


def fmt(s):
    return " ".join(f"{k} {len(s.get(k, {}))}" for k in ("GPS", "GAL", "BDS"))


print(Path(cap).name)
pad = [r for r in raw if -60 <= r[0] <= -5]
for k in ("GPS", "GAL", "BDS"):
    c = [v for r in pad for v in r[1].get(k, {}).values()]
    n = [len(r[1].get(k, {})) for r in pad]
    print(f"  pad {k}: C/N0 median {st.median(c) if c else float('nan'):.0f} dB-Hz, satellites median "
          f"{st.median(n) if n else 0:.0f}, max {max(n) if n else 0}")
marks = [0.0, 1.0, 2.0, 2.5, OVER] if BURN < 6 else [0.0, 2.8, 6.0, OVER]
for t in marks:
    r = at(t)
    print(f"  raw at T{t:+.1f}{' (515 m/s)' if t == OVER else ''}: {fmt(r[1])}  (epoch T{r[0]:+.2f})"
          if r[0] is not None else f"  raw at T{t:+.1f}: none")
after = [r for r in raw if 0 <= r[0] <= OVER + 1.0 and any(r[1].values())]       # up to the first 515 crossing
last = max(after, key=lambda r: r[0], default=None)
if last:
    print(f"  last raw epoch before the blocked window: T{last[0]:+.2f}: {fmt(last[1])}")
back = next((r for r in raw if r[0] > (last[0] + 0.5 if last else 0) and any(r[1].values())), None)
if back:
    print(f"  raw back at T{back[0]:+.2f}: {fmt(back[1])}")
fx = [q for q in pvt if 0 <= q[0] <= OVER + 1.0 and q[1] >= 2 and q[2]]
if fx:
    q = fx[-1]
    print(f"  last fix after ignition T{q[0]:+.2f}: own {q[3]:.0f} m/s, true {vtrue(q[0]):.0f} (behind "
          f"{vtrue(q[0]) - q[3]:.0f})")
fb = next((q for q in pvt if q[0] > (fx[-1][0] + 1.0 if fx else 0) and q[1] >= 2 and q[2]), None)
print(f"  fix back at T{fb[0]:+.2f} (true speed {vtrue(fb[0]):.0f} m/s)" if fb else "  no fix back in the capture")
