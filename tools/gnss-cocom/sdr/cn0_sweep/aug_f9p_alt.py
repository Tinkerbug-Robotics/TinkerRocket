#!/usr/bin/env python3
"""August's ZED-F9P gate captures (results/zed_f9p_{spaceshot,gentle_alt}.log.gz + .scenario.json): the scenario's
shape, the capture's line format, and the NAV-PVT fix runs with height and speed against the truth -- does the F9P ever
hold a fix above 80 km there, and how did it get above 80 km (with or without a fix while crossing)? Read-only.
    aug_f9p_alt.py NAME     (NAME = spaceshot | gentle_alt)"""
import gzip
import json
import struct
import sys
from pathlib import Path

R = Path(__file__).resolve().parents[1] / "results"
name = sys.argv[1]
sc = json.loads((R / f"zed_f9p_{name}.scenario.json").read_text())
print("scenario keys:", sorted(sc.keys()))
for k in ("prologue_s", "velocity_windows", "blocked_windows", "start_tow", "tow0", "gps_tow0"):
    if k in sc:
        print(f"  {k}: {sc[k]}")
tr = sc.get("truth", [])
if tr:
    print("  truth rows:", len(tr), "first:", tr[0])
    ap = max(tr, key=lambda s: s["alt_m"])
    print("  apogee:", {k: ap[k] for k in ap if k in ("t", "alt_m", "speed_mps", "phase")})
lines = gzip.open(R / f"zed_f9p_{name}.log.gz", "rt", errors="replace").read().splitlines()
print("capture lines:", len(lines))
for ln in lines[:3]:
    print("  ", ln[:110])
kinds = {}
for ln in lines[:2000]:
    p = ln.split(" ", 2)
    if len(p) >= 2:
        kinds[p[1]] = kinds.get(p[1], 0) + 1
print("  line kinds in the first 2000:", kinds)
# NAV-PVT from 'U' lines (t U 0107<hex>) or raw hex frames
rows = []
for ln in lines:
    p = ln.split(" ", 2)
    if len(p) < 3 or p[1] != "U" or not p[2].startswith("0107"):
        continue
    try:
        b = bytes.fromhex(p[2].strip()[4:])
    except ValueError:
        continue
    if len(b) < 92:
        continue
    itow = struct.unpack_from("<I", b, 0)[0] / 1000.0
    h_msl = struct.unpack_from("<i", b, 36)[0] / 1000.0
    vn, ve, vd = struct.unpack_from("<iii", b, 48)
    rows.append((itow, b[20], bool(b[21] & 1), b[23], h_msl, (vn * vn + ve * ve + vd * vd) ** 0.5 / 1000.0))
print("NAV-PVT rows:", len(rows))
if not rows:
    sys.exit()
runs, cur = [], None
for r in rows:
    key = (r[1] >= 2 and r[2])
    if cur is None or key != cur[2]:
        cur = [r, r, key]
        runs.append(cur)
    cur[1] = r
for a, b, fixed in runs:
    print(f"  iTOW {a[0]:9.1f} .. {b[0]:9.1f} ({b[0] - a[0]:6.1f} s)  {'FIX' if fixed else 'no '}  hMSL "
          f"{a[4] / 1000:7.2f} -> {b[4] / 1000:7.2f} km  speed {a[5]:5.0f} -> {b[5]:5.0f} m/s")
