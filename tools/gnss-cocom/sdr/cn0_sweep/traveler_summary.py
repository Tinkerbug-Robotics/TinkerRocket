#!/usr/bin/env python3
"""A PX1105R flight of the traveler (pad600 timing: ignition at file time 600 s, GPS TOW 203400 at file start),
from gps-sdr-sim or SignalSim: satellites the receiver lists in its raw output (0xE5) per constellation at
ignition, T+2.8, T+6 and burnout (T+13); its last 2D/3D fix after ignition (own vs true speed there, the
COCOM stop); when a fix comes back after the blocked window and with how many satellites; and a 20 s timeline
(listed GPS/GAL/BDS, fix state, own speed vs truth).   traveler_summary.py CAPTURE [SCENARIO]"""
import bisect
import json
import struct
import sys
from pathlib import Path

SDR = Path(__file__).resolve().parents[1]
TOW0, IGN = 203400.0, 600.0
cap = sys.argv[1]
scen = sys.argv[2] if len(sys.argv) > 2 else "traveler_soft25"
sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
tr, PRO = sc["truth"], sc.get("prologue_s", 180.0)
tt = [s["t"] for s in tr]
G = {0: "GPS", 3: "GAL", 5: "BDS"}
BURN = next(s["t"] for s in tr if s["phase"] == "coast") - PRO           # burnout, from ignition (traveler 13, hotshot 4)
UNDER = sc["velocity_windows"][-1][1] - PRO                              # back under 515 m/s for good
MARKS = ((0.0, "ignition"), (2.8, "T+2.8"), (6.0, "T+6")) if BURN > 6.0 else ((0.0, "ignition"), (1.0, "T+1"),
                                                                             (2.0, "T+2"))


def truth(t):                                    # t from ignition -> (speed, altitude)
    x = t + PRO
    i = min(max(bisect.bisect_left(tt, x), 1), len(tt) - 1)
    a, b = tr[i - 1], tr[i]
    f = (x - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0.0
    alt = a.get("alt_m", a.get("altitude_m", 0.0))
    alt += f * (b.get("alt_m", b.get("altitude_m", 0.0)) - alt)
    return a["speed_mps"] + f * (b["speed_mps"] - a["speed_mps"]), alt


e5, df = {}, []
for line in open(cap, errors="replace"):
    p = line.split(" ", 2)
    if len(p) < 3 or p[1] != "B":
        continue
    try:
        x = bytes.fromhex(p[2].strip())
    except ValueError:
        continue
    if x[0] == 0xE5 and len(x) >= 14:
        t = round(struct.unpack(">I", x[5:9])[0] / 1000.0 - TOW0 - IGN, 2)
        s = {}
        for j in range(x[13]):
            r = x[14 + 31 * j: 14 + 31 * (j + 1)]
            if len(r) == 31 and (r[0] & 0x0F) in G and (r[0] >> 4) in (0, 1):
                s.setdefault(G[r[0] & 0x0F], set()).add(r[1])
        e5[t] = s
    elif x[0] == 0xDF and len(x) >= 61:
        t = round(struct.unpack(">d", x[5:13])[0] - TOW0 - IGN, 2)
        vx, vy, vz = struct.unpack(">fff", x[37:49])
        df.append((t, x[2], (vx * vx + vy * vy + vz * vz) ** 0.5))
e5t = sorted(e5)


def listed(t):                                   # nearest raw epoch at or before t
    i = bisect.bisect_right(e5t, t + 1e-6) - 1
    return e5[e5t[i]] if i >= 0 else {}


def fmt(s):
    return " ".join(f"{k} {len(s.get(k, ()))}" for k in ("GPS", "GAL", "BDS") if k in s or k == "GPS")


print(Path(cap).name)
for t, label in MARKS + ((BURN, f"burnout T+{BURN:.0f}"),):
    print(f"  listed at {label:13s}: {fmt(listed(t))}")
fixes = [d for d in df if 0 <= d[0] < UNDER - 1.0 and d[1] >= 2]      # before the blocked window ends
last = fixes[-1] if fixes else None                                      # (a flicker earlier is not the stop)
if last:
    v, h = truth(last[0])
    print(f"  last fix after ignition T+{last[0]:.2f}: own speed {last[2]:.0f} m/s, true {v:.0f} m/s "
          f"(behind {v - last[2]:.0f})")
back = next((d for d in df if d[0] > max(last[0] if last else 0.0, UNDER - 1.0) and d[1] >= 2), None)
if back:
    v, h = truth(back[0])
    print(f"  fix back at T+{back[0]:.1f} (true speed {v:.0f} m/s, altitude {h / 1000:.1f} km); listed then: "
          f"{fmt(listed(back[0]))}")
else:
    print(f"  no fix after T+{UNDER - 1.0:.0f} in the capture")
print("  timeline (T+, listed, best fix state in the 20 s bin, own speed / truth):")
dft = [d[0] for d in df]
for t0 in range(-20, 661, 20):
    i0, i1 = bisect.bisect_left(dft, t0), bisect.bisect_left(dft, t0 + 20)
    st = max((df[i][1] for i in range(i0, i1)), default=None)
    own = next((df[i][2] for i in range(i0, i1) if df[i][1] >= 2), None)
    v, h = truth(t0)
    print(f"    {t0:4d}: {fmt(listed(t0)):22s} fix {st}  own {own if own is None else round(own)} / true {v:.0f}"
          f"  alt {h / 1000:.1f} km")
