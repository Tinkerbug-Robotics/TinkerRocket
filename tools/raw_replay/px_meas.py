#!/usr/bin/env python3
"""Residuals of the PX1105R's measurements as the image's GNSS task built them (px_meas_check's CSV), against a known
receiver.

    px_meas.py static MEAS.csv                    # a still receiver at the --fixed point: velocity 0
    px_meas.py truth MEAS.csv SENSORS.csv.gz GPS_IGNITION_S PAD_LAT PAD_LON PAD_H [--from -60 --to 70]
    px_meas.py truthfile SENSORS.csv.gz GPS_IGNITION_S PAD_LAT PAD_LON PAD_H OUT   # the truth for px_meas_check

Pseudorange: pr - rho less each epoch's receiver clock (the GPS satellites' median) and each system's offset from it
(the median over the run): by system, C/N0 and elevation. Range rate: rr - u.(vs - v_rx) less each epoch's drift
(every satellite's median): by system and C/N0; against a trajectory, v_rx at the epoch's GPS instant plus a lag
scanned to find where the Dopplers sit in time (boost and coast, where the acceleration shows it).
"""
import gzip
import math
import sys

import numpy as np

A, E2 = 6378137.0, 6.69437999014e-3
SYS = {0: "GPS", 1: "GAL", 2: "BDS"}


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def up_of(r):
    lon = np.arctan2(r[:, 1], r[:, 0])
    p = np.hypot(r[:, 0], r[:, 1])
    lat = np.arctan2(r[:, 2], p * (1 - E2))
    return np.stack([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)], axis=1)


def per_epoch_median(t, x, mask):
    """Each epoch's median of x over mask, broadcast back to its rows."""
    out = np.full(len(t), np.nan)
    ut, inv = np.unique(t, return_inverse=True)
    for k in range(len(ut)):
        m = (inv == k) & mask
        if m.sum() >= 3:
            out[inv == k] = np.median(x[m])
    return out


def stats(x):
    x = x[np.isfinite(x)]
    if x.size == 0:
        return "n 0"
    return (f"n {x.size:6d} mean {np.mean(x):+.3f} rms {np.sqrt(np.mean(x * x)):.3f} p50|.| {np.median(np.abs(x)):.3f} "
            f"p95|.| {np.percentile(np.abs(x), 95):.3f}")


def report(d, pr_res, rr_res, el):
    print("  pseudorange residual (m), clock and system offsets out:")
    for s in (0, 1, 2):
        m = (d["sys"] == s) & (d["has_pr"] > 0)
        if m.any():
            print(f"    {SYS[s]}  {stats(pr_res[m])}")
    for lo, hi in ((0, 30), (30, 35), (35, 40), (40, 45), (45, 99)):
        m = (d["has_pr"] > 0) & (d["cn0"] >= lo) & (d["cn0"] < hi)
        if m.any():
            print(f"    C/N0 {lo:2d}-{hi:2d}  {stats(pr_res[m])}")
    for lo, hi in ((0, 15), (15, 30), (30, 60), (60, 91)):
        m = (d["has_pr"] > 0) & (el >= lo) & (el < hi)
        if m.any():
            print(f"    elev {lo:2d}-{hi:2d}  {stats(pr_res[m])}")
    print("  range-rate residual (m/s), each epoch's drift out:")
    for s in (0, 1, 2):
        m = (d["sys"] == s) & (d["has_rr"] > 0)
        if m.any():
            print(f"    {SYS[s]}  {stats(rr_res[m])}")
    for lo, hi in ((0, 30), (30, 35), (35, 40), (40, 45), (45, 99)):
        m = (d["has_rr"] > 0) & (d["cn0"] >= lo) & (d["cn0"] < hi)
        if m.any():
            print(f"    C/N0 {lo:2d}-{hi:2d}  {stats(rr_res[m])}")
    m = d["has_rr"] > 0
    z = rr_res[m] / d["sigma_rr"][m]
    z = z[np.isfinite(z)]
    print(f"    normalized by the builder's sigma_rr: rms {np.sqrt(np.mean(z * z)):.3f}, p95|.| "
          f"{np.percentile(np.abs(z), 95):.2f}")
    m = d["has_pr"] > 0
    z = pr_res[m] / d["sigma_pr"][m]
    z = z[np.isfinite(z)]
    print(f"    (pseudorange by its white sigma_pr: rms {np.sqrt(np.mean(z * z)):.2f} -- the correlated part, the "
          f"profile's pr_corr, is not in it)")


def residuals(d, v_rx):
    t = d["gps_s"]
    r = np.stack([d["rx"], d["ry"], d["rz"]], axis=1)
    u = np.stack([d["ux"], d["uy"], d["uz"]], axis=1)
    vs = np.stack([d["vsx"], d["vsy"], d["vsz"]], axis=1)
    el = np.degrees(np.arcsin(np.clip(np.sum(u * up_of(r), axis=1), -1, 1)))
    pr0 = d["pr"] - d["rho"]
    clk = per_epoch_median(t, pr0, (d["sys"] == 0) & (d["has_pr"] > 0))
    pr1 = pr0 - clk
    isb = {}
    for s in (0, 1, 2):
        m = (d["sys"] == s) & (d["has_pr"] > 0) & np.isfinite(pr1)
        isb[s] = float(np.median(pr1[m])) if m.any() else 0.0
    pr_res = pr1 - np.array([isb[int(s)] for s in d["sys"]])
    rr0 = d["rr"] - np.sum(u * (vs - v_rx), axis=1)
    drift = per_epoch_median(t, rr0, d["has_rr"] > 0)
    rr_res = rr0 - drift
    return pr_res, rr_res, el, isb, clk, drift


def ned_to_ecef_rot(lat0, lon0):
    sl, cl, so, co = math.sin(lat0), math.cos(lat0), math.sin(lon0), math.cos(lon0)
    # columns: north, east, down in ECEF
    return np.array([[-sl * co, -so, -cl * co], [-sl * so, co, -cl * so], [cl, 0.0, -sl]])


def lla_to_ecef(lat, lon, h):
    s, c = np.sin(lat), np.cos(lat)
    n = A / np.sqrt(1 - E2 * s * s)
    return np.stack([(n + h) * c * np.cos(lon), (n + h) * c * np.sin(lon), (n * (1 - E2) + h) * s], axis=-1)


def ecef_to_lla(x):
    lon = np.arctan2(x[:, 1], x[:, 0])
    p = np.hypot(x[:, 0], x[:, 1])
    lat = np.arctan2(x[:, 2], p * (1 - E2))
    for _ in range(6):
        n = A / np.sqrt(1 - E2 * np.sin(lat) ** 2)
        h = p / np.cos(lat) - n
        lat = np.arctan2(x[:, 2], p * (1 - E2 * n / (n + h)))
    return lat, lon, h


PHASES = {"pad": 0, "rail": 1, "boost": 2, "coast": 3, "drogue": 4, "descent": 4, "main": 5, "landed": 6, "ground": 6}


def read_truth(sensors):
    """t_s (from ignition), pn, pe, pd, vn, ve, vd, phase: a sim's sensors CSV, or a scenario JSON's 10 Hz truth (a
    real flight's sky: its file time less the prologue)."""
    if sensors.endswith(".json"):
        import json
        sc = json.load(open(sensors))
        tr = sc["truth"]
        pro = float(sc.get("prologue_s", 180.0))
        a0 = tr[0]["alt_m"]
        return np.array([[r["t"] - pro, r["north_m"], r["east_m"], -(r["alt_m"] - a0), r["v_north_mps"],
                          r["v_east_mps"], -r["v_up_mps"], PHASES.get(r.get("phase", "pad"), 4)] for r in tr])
    rows = [ln for ln in gzip.open(sensors, "rt") if not ln.startswith("#")]
    names = rows[0].strip().split(",")
    cols = [names.index(c) for c in ("t_s", "pn_m", "pe_m", "pd_m", "vn_mps", "ve_mps", "vd_mps", "phase")]
    d = np.loadtxt(rows[1:], delimiter=",", usecols=cols)
    return d


def main():
    mode = sys.argv[1]
    if mode == "static":
        d = load(sys.argv[2])
        pr_res, rr_res, el, isb, clk, drift = residuals(d, np.zeros((len(d), 3)))
        print(f"{sys.argv[2]}: {len(d)} measurements, {len(np.unique(d['gps_s']))} epochs; system offsets from GPS "
              f"(m): " + ", ".join(f"{SYS[s]} {isb[s]:+.2f}" for s in isb) +
              f"; ionosphere used on {100.0 * np.mean(d['use_iono'] > 0):.0f} % of rows")
        report(d, pr_res, rr_res, el)
        # paired against not
        for p in (0, 1):
            m = (d["paired"] == p) & (d["has_rr"] > 0)
            if m.any():
                print(f"    rr paired={p}: {stats(rr_res[m])}")
        return
    if mode == "truthfile":
        sensors, gps_ign, la, lo, h, out = sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]), \
            float(sys.argv[6]), sys.argv[7]
        d = read_truth(sensors)
        lat0, lon0 = math.radians(la), math.radians(lo)
        R = ned_to_ecef_rot(lat0, lon0)
        x0 = lla_to_ecef(lat0, lon0, h)
        dt = float(np.median(np.diff(d[:, 0])))
        k = np.arange(0, len(d), max(1, int(round(0.01 / dt))))   # 100 Hz rows (a 10 Hz truth: every row)
        x = x0 + d[k, 1:4] @ R.T
        lat, lon, hh = ecef_to_lla(x)
        with open(out, "w") as f:
            for i, j in enumerate(k):
                f.write(f"{gps_ign + d[j, 0]:.4f} {math.degrees(lat[i]):.10f} {math.degrees(lon[i]):.10f} {hh[i]:.4f}\n")
        return
    if mode == "truth":
        meas, sensors, gps_ign = sys.argv[2], sys.argv[3], float(sys.argv[4])
        la, lo, h = float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7])
        d = load(meas)
        tr = read_truth(sensors)
        R = ned_to_ecef_rot(math.radians(la), math.radians(lo))
        tf = d["gps_s"] - gps_ign
        burnout = float(tr[np.argmax(tr[:, 7] >= 3), 0])
        apogee = float(tr[int(np.argmin(tr[:, 3])), 0])
        print(f"truth: burnout T+{burnout:.2f}, apogee T+{apogee:.2f}")

        def v_at(ts):
            v = np.stack([np.interp(ts, tr[:, 0], tr[:, 4 + i]) for i in range(3)], axis=1)
            return v @ R.T

        for name, lo_t, hi_t in (("pad", -60, 0), ("boost", 0, burnout), ("coast", burnout, apogee - 2),
                                 ("apogee", apogee - 2, apogee + 2), ("descent", apogee + 2, 66)):
            m = (tf >= lo_t) & (tf < hi_t)
            if not m.any():
                continue
            sub = d[m]
            pr_res, rr_res, el, isb, clk, drift = residuals(sub, v_at(tf[m]))
            print(f"\n== {name} T{lo_t:+.1f}..{hi_t:+.1f}: {m.sum()} measurements, "
                  f"{len(np.unique(sub['gps_s']))} epochs, satellites/epoch {m.sum() / max(1, len(np.unique(sub['gps_s']))):.1f}; "
                  f"system offsets " + ", ".join(f"{SYS[s]} {isb[s]:+.2f}" for s in isb if isb[s] != 0.0))
            report(sub, pr_res, rr_res, el)
            if name in ("boost", "coast", "apogee"):
                best = None
                line = []
                for lag_ms in range(-100, 41, 5):
                    _, rr_l, _, _, _, _ = residuals(sub, v_at(tf[m] + lag_ms * 1e-3))
                    q = rr_l[np.isfinite(rr_l) & (sub["has_rr"] > 0)]
                    rms = float(np.sqrt(np.mean(q * q)))
                    line.append(f"{lag_ms:+d}:{rms:.3f}")
                    if best is None or rms < best[1]:
                        best = (lag_ms, rms)
                print(f"  Doppler time offset scan (v_rx at tag + lag ms: rms m/s): " + " ".join(line))
                print(f"  best: the Dopplers sit at the tag {best[0]:+d} ms (rms {best[1]:.3f} m/s)")
        return


if __name__ == "__main__":
    main()
