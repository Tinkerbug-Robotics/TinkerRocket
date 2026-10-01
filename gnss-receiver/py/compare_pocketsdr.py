#!/usr/bin/env python3
"""Compare a gnssrx run with Pocket SDR's pocket_trk log on the same input stream.

    compare_pocketsdr.py RUN_DIR POCKET_TRK_LOG [--truth LAT,LON,H] [--from S]

Positions are compared with the truth. Observables are compared at common
epochs. Each receiver labels epochs with its own clock, so the two "same"
epochs are slightly different instants: per epoch, the pseudorange
differences are fitted as a + (range rate) * dt (a: the clocks, dt: the
label offset) and the residuals are compared. Also Doppler and C/N0.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import statistics as st
from collections import defaultdict

A, F = 6378137.0, 1 / 298.257223563
E2 = F * (2 - F)


def ecef(lat, lon, h):
    n = A / math.sqrt(1 - E2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon), (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1 - E2) + h) * math.sin(lat))


def enu(lat_deg, lon_deg, h, ref):
    rlat, rlon, rh = ref
    x = ecef(math.radians(lat_deg), math.radians(lon_deg), h)
    r = ecef(math.radians(rlat), math.radians(rlon), rh)
    d = [x[i] - r[i] for i in range(3)]
    la, lo = math.radians(rlat), math.radians(rlon)
    e = -math.sin(lo) * d[0] + math.cos(lo) * d[1]
    n = -math.sin(la) * math.cos(lo) * d[0] - math.sin(la) * math.sin(lo) * d[1] + math.cos(la) * d[2]
    u = math.cos(la) * math.cos(lo) * d[0] + math.cos(la) * math.sin(lo) * d[1] + math.sin(la) * d[2]
    return e, n, u


def sow(y, mo, d, h, mi, s):
    t = dt.datetime(y, mo, d, h, mi) - dt.datetime(1980, 1, 6)
    return (t.days * 86400 + t.seconds) % 604800 + s


def stats(v):
    return (st.mean(v), st.pstdev(v)) if v else (float("nan"), float("nan"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("log")
    ap.add_argument("--truth", default="0.0,-119.0,1200.0")
    ap.add_argument("--from", dest="t_from", type=float, default=0.0, help="ignore epochs before this GPS second of week")
    a = ap.parse_args()
    ref = tuple(float(v) for v in a.truth.split(","))

    # Pocket SDR
    p_pos, p_obs = [], defaultdict(dict)
    for line in open(a.log):
        f = line.strip().split(",")
        if f[0] == "$POS":
            t = sow(int(f[2]), int(f[3]), int(f[4]), int(f[5]), int(f[6]), float(f[7]))
            p_pos.append((t, float(f[8]), float(f[9]), float(f[10])))
        elif f[0] == "$OBS" and f[8].startswith("G"):
            t = sow(int(f[2]), int(f[3]), int(f[4]), int(f[5]), int(f[6]), float(f[7]))
            p_obs[round(t, 3)][int(f[8][1:])] = (float(f[11]), float(f[13]), float(f[10]))  # pr, dop, cn0
    # gnssrx
    g_pos = [(float(r["rx_tow"]), float(r["lat_deg"]), float(r["lon_deg"]), float(r["h_m"]))
             for r in csv.DictReader(open(f"{a.run}/pvt.csv"))]
    g_obs = defaultdict(dict)
    for r in csv.DictReader(open(f"{a.run}/obs.csv")):
        g_obs[round(float(r["rx_tow"]), 3)][int(r["prn"])] = (float(r["pr_m"]), float(r["dop_hz"]), float(r["cn0"]))

    print("Position error vs truth (m): mean (sd)")
    for name, pos in (("gnssrx", g_pos), ("Pocket SDR", p_pos)):
        pts = [enu(la, lo, h, ref) for t, la, lo, h in pos if t >= a.t_from]
        if not pts:
            print(f"  {name:10s} no fixes")
            continue
        cols = list(zip(*pts))
        s = [stats(list(c)) for c in cols]
        print(f"  {name:10s} {len(pts):5d} fixes  E {s[0][0]:+.3f} ({s[0][1]:.3f})  N {s[1][0]:+.3f} ({s[1][1]:.3f})  "
              f"U {s[2][0]:+.3f} ({s[2][1]:.3f})")

    # Observables at common epochs (our epochs are on our clock, steered to GPS time; match within 1 ms).
    common = []
    p_keys = sorted(p_obs)
    import bisect
    for t in sorted(g_obs):
        if t < a.t_from:
            continue
        i = bisect.bisect_left(p_keys, t - 0.001)
        if i < len(p_keys) and abs(p_keys[i] - t) <= 0.001:
            common.append((t, p_keys[i]))
    import numpy as np
    lam = 299792458.0 / 1575.42e6
    dpr, ddop, dcn0 = defaultdict(list), defaultdict(list), defaultdict(list)
    offsets = []
    for tg, tp in common:
        sats = sorted(set(g_obs[tg]) & set(p_obs[tp]))
        if len(sats) < 4:
            continue
        d = np.array([g_obs[tg][s][0] - p_obs[tp][s][0] for s in sats])
        rate = np.array([-lam * g_obs[tg][s][1] for s in sats])  # range rate, m/s
        h = np.column_stack([np.ones_like(rate), rate])
        coef, *_ = np.linalg.lstsq(h, d, rcond=None)
        offsets.append(coef[1])
        res = d - h @ coef
        for k, s in enumerate(sats):
            dpr[s].append(float(res[k]))
            ddop[s].append(g_obs[tg][s][1] - p_obs[tp][s][1])
            dcn0[s].append(g_obs[tg][s][2] - p_obs[tp][s][2])
    print(f"\nObservables, gnssrx - Pocket SDR, {len(common)} common epochs; epoch-label offset "
          f"{1e3 * st.mean(offsets):+.3f} ms (sd {1e3 * st.pstdev(offsets):.3f})")
    print(" PRN   pseudorange m mean (sd)   Doppler Hz mean (sd)   C/N0 dB mean")
    allpr = []
    for s in sorted(dpr):
        mp, sp = stats(dpr[s])
        md, sd = stats(ddop[s])
        mc, _ = stats(dcn0[s])
        allpr += dpr[s]
        print(f" {s:3d}   {mp:+8.3f} ({sp:.3f})          {md:+7.3f} ({sd:.3f})        {mc:+5.2f}")
    if allpr:
        print(f" all   sd {st.pstdev(allpr):.3f} m")


if __name__ == "__main__":
    main()
