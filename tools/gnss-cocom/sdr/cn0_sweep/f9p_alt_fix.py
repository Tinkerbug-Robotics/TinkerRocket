#!/usr/bin/env python3
"""Does the ZED-F9P publish a fix above 80 km? NAV-PVT runs (fixType, gnssFixOK, numSV) with their height (hMSL and
ellipsoid) and speed ranges, against the truth's altitude and speed at the same times, from T-5 to the file end.
    f9p_alt_fix.py CAPTURE [SCENARIO]"""
import json
import struct
import sys
from pathlib import Path

SDR = Path(__file__).resolve().parents[1]
cap = sys.argv[1]
scen = sys.argv[2] if len(sys.argv) > 2 else "traveler_soft25"
sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
PRO = sc["prologue_s"]
TR = [(s["t"] - PRO, s["alt_m"], s["speed_mps"]) for s in sc["truth"]]


def truth(t):
    best = min(TR, key=lambda x: abs(x[0] - t))
    return best[1], best[2]


rows = []
for line in open(cap, errors="replace"):
    p = line.split(" ", 2)
    if len(p) < 3 or p[1] != "U" or not p[2].startswith("0107"):
        continue
    try:
        b = bytes.fromhex(p[2].strip()[4:])
    except ValueError:
        continue
    if len(b) < 92:
        continue
    t = struct.unpack_from("<I", b, 0)[0] / 1000.0 - 204000.0
    if t < -5:
        continue
    ok = bool(b[21] & 1)
    h_ell, h_msl = struct.unpack_from("<ii", b, 32)
    vn, ve, vd = struct.unpack_from("<iii", b, 48)
    rows.append((t, b[20], ok, b[23], h_ell / 1e6, h_msl / 1e6, (vn * vn + ve * ve + vd * vd) ** 0.5 / 1000.0))
runs, cur = [], None
for r in rows:
    key = (r[1], r[2])
    if cur is None or key != cur["key"] or r[0] - cur["t1"] > 0.5:
        cur = {"key": key, "t0": r[0], "t1": r[0], "rows": []}
        runs.append(cur)
    cur["t1"] = r[0]
    cur["rows"].append(r)
for c in runs:
    rs = c["rows"]
    a0, s0 = truth(c["t0"])
    a1, s1 = truth(c["t1"])
    hs = [r[5] for r in rs]
    vs = [r[6] for r in rs]
    print(f"T{c['t0']:+7.1f} .. T{c['t1']:+7.1f}  fixType {c['key'][0]} gnssFixOK {int(c['key'][1])}  numSV "
          f"{min(r[3] for r in rs)}-{max(r[3] for r in rs)}  hMSL {min(hs):7.2f}..{max(hs):7.2f} km  speed "
          f"{min(vs):6.0f}..{max(vs):6.0f} m/s   truth alt {a0 / 1000:6.2f}->{a1 / 1000:6.2f} km, speed "
          f"{s0:5.0f}->{s1:5.0f}")
