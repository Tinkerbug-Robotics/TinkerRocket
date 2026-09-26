#!/usr/bin/env python3
"""What sky does the spaceshot scenario see? GPS elevations from the BRDC file
gps-sdr-sim used (c8/BRDC_2026230.rx2.n): at the scenario's origin and ignition, across
the day (run start every 20 min), and across a lat/lon grid at the scenario start.

Low = 5-30 deg matters on a vertical flight: a satellite's line-of-sight acceleration
and Doppler scale with sin(elevation), so low satellites ride through a boost best.
Found (2026-09-26): origin (0, -119) at 08:30 GPS is already the best sky that day,
14 GPS >= 5 deg with 10-11 of them low -- but the PX1105R's 15 deg default elevation
mask kept the 5 lowest out of its channel table (px1105r_run.py --elev-mask 3).

    ./sky_scan.py
"""
import math, struct, statistics as stt, sys
from pathlib import Path
SDR = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent
sys.path.insert(0, str(SDR))
from msm7_clock import rinex2_nav, sat_pos, rx_ecef
eph = rinex2_nav(SDR / "c8" / "BRDC_2026230.rx2.n")
TOW_DAY0, TOW_START = 172800.0, 203400.0          # Tue 2026-08-18 00:00 and 08:30 GPS
def elevs(lat, lon, h, tow):
    rx = rx_ecef(lat, lon, h); la, lo = math.radians(lat), math.radians(lon); out = {}
    for prn, lst in eph.items():
        e = min(lst, key=lambda q: abs(tow - q["toe"]))
        if abs(tow - e["toe"]) > 7200: continue
        p = sat_pos(e, tow); dx, dy, dz = p[0]-rx[0], p[1]-rx[1], p[2]-rx[2]
        E = -math.sin(lo)*dx + math.cos(lo)*dy
        N = -math.sin(la)*math.cos(lo)*dx - math.sin(la)*math.sin(lo)*dy + math.cos(la)*dz
        U = math.cos(la)*math.cos(lo)*dx + math.cos(la)*math.sin(lo)*dy + math.sin(la)*dz
        out[prn] = math.degrees(math.atan2(U, math.hypot(E, N)))
    return out
def window(lat, lon, t0, dur=660.0, step=60.0):
    """min over the run window (6 min pad + flight) of: count >= 5 deg, count 5-30 deg"""
    tot, low = [], []
    t = t0
    while t <= t0 + dur:
        el = elevs(lat, lon, 1200.0, t); tot.append(sum(1 for v in el.values() if v >= 5)); low.append(sum(1 for v in el.values() if 5 <= v < 30)); t += step
    return min(tot), min(low)
# 1. the current scenario at ignition (file 360 s = 08:36)
el = elevs(0.0, -119.0, 1200.0, TOW_START + 360)
vis = sorted(((v, p) for p, v in el.items() if v > 0), reverse=True)
print("current origin (0, -119) at ignition 08:36 GPS; GPS above the horizon (deg):")
print("  " + "  ".join(f"G{p}:{v:.0f}" for v, p in vis))
print(f"  -> {sum(1 for v, p in vis if v >= 5)} at >= 5 deg, {sum(1 for v, p in vis if 5 <= v < 30)} of them at 5-30 deg")
# 2. same place, across the day (start times every 20 min)
print("\nsame origin, run started at other times on Aug 18 (min over the 11-min run):")
best = []
for k in range(0, 72):
    t0 = TOW_DAY0 + k * 1200.0
    n, lo = window(0.0, -119.0, t0); best.append((n, lo, t0))
best.sort(key=lambda x: (x[0], x[1]), reverse=True)
fmt = lambda t: f"{int((t-TOW_DAY0)//3600):02d}:{int((t-TOW_DAY0)%3600//60):02d}"
cur = window(0.0, -119.0, TOW_START)
print(f"  current start 08:30: {cur[0]} sats >= 5 deg, {cur[1]} low")
print("  most sats:  " + ", ".join(f"{fmt(t)} {n} ({lo} low)" for n, lo, t in best[:5]))
best.sort(key=lambda x: (x[1], x[0]), reverse=True)
print("  most low:   " + ", ".join(f"{fmt(t)} {lo} low ({n} total)" for n, lo, t in best[:5]))
# 3. other places at the current start time
print("\nother origins, run started 08:30 GPS (min over the run): total >= 5 deg / low 5-30 deg")
lats = [60, 45, 30, 15, 0, -15, -30, -45]; lons = [-150, -119, -90, -60, -30, 0, 30, 60, 90, 120, 150, 180]
print("  lat\\lon " + "".join(f"{lo:>7}" for lo in lons))
for la in lats:
    print(f"  {la:>6} " + "".join("{:>7}".format("%d/%d" % window(la, lo, TOW_START, step=165.0)) for lo in lons))
