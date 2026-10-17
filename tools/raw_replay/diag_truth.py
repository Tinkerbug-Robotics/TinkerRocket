#!/usr/bin/env python3
"""The diagnosed epochs' measurements against the truth: each satellite's pseudorange and Doppler residual with the
receiver where the flight really was (the truth at the epoch's own GPS time, the Doppler at its 55.5 ms lag), the
common clock terms (the median) taken out; beside the filter's own innovations, likewise centred.

    diag_truth.py HARNESS_STDERR [--lag 0.0555] [--t0-tow 573786.0]

Reads the harness's DIAGDATA lines (--diag-from-s/--diag-to-s).
"""
import argparse
import math
import sys

import numpy as np

sys.path.insert(0, ".")
import scorelib as S  # noqa: E402

A, E2 = 6378137.0, 0.0066943799901
KC, OMEGA = 299792458.0, 7.2921151467e-5


def lla2ecef(lat, lon, h):
    s, c = math.sin(lat), math.cos(lat)
    n = A / math.sqrt(1 - E2 * s * s)
    return np.array([(n + h) * c * math.cos(lon), (n + h) * c * math.sin(lon), (n * (1 - E2) + h) * s])


def ned2ecef_mat(lat, lon):
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    # rows: N, E, D in ECEF
    Tm = np.array([[-sl * co, -sl * so, cl], [-so, co, 0.0], [-cl * co, -cl * so, -sl]])
    return Tm.T     # ECEF = Tm.T @ NED


def los(rs, vs, r, v):
    d = rs - r
    rho = np.linalg.norm(d)
    c, s = 1.0, 0.0
    rr = rs.copy()
    for _ in range(2):
        th = OMEGA * rho / KC
        c, s = 1 - 0.5 * th * th, th * (1 - th * th / 6)
        rr = np.array([c * rs[0] + s * rs[1], -s * rs[0] + c * rs[1], rs[2]])
        d = rr - r
        rho = np.linalg.norm(d)
    vsr = np.array([c * vs[0] + s * vs[1], -s * vs[0] + c * vs[1], vs[2]])
    u = d / rho
    return rho, float(u @ (vsr - v)), u


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stderr_file")
    ap.add_argument("--lag", type=float, default=0.0555)
    ap.add_argument("--t0-tow", type=float, default=573786.0)
    a = ap.parse_args()
    tr = S.Truth()
    lat0, lon0, h0 = (math.radians(S.PAD[0]), math.radians(S.PAD[1]), S.PAD[2])
    r0 = lla2ecef(lat0, lon0, h0)
    M = ned2ecef_mat(lat0, lon0)
    rows = {}
    for ln in open(a.stderr_file):
        if ln.startswith("DIAGDATA "):
            v = ln.split()[1:]
            tow = float(v[0])
            rows.setdefault(tow, []).append([float(x) for x in v])
    for tow, rs in rows.items():
        T = tow - a.t0_tow
        p, _, _ = tr.at(np.array([T]))
        _, vl, _ = tr.at(np.array([T - a.lag]))
        r_true = r0 + M @ p[0]
        v_true = M @ vl[0]
        out = []
        for x in rs:
            sys_, prn = int(x[1]), int(x[2])
            pr, rr = x[3], x[4]
            sp, sv = np.array(x[5:8]), np.array(x[8:11])
            r_f, v_f = np.array(x[11:14]), np.array(x[14:17])
            y_pr, y_rr, s_rr = x[17], x[18], x[19]
            rho, rate, u = los(sp, sv, r_true, v_true)
            out.append((sys_, prn, pr - rho, rr - rate, y_pr, y_rr, s_rr, u, np.linalg.norm(r_f - r_true),
                        np.linalg.norm(v_f - v_true)))
        pr_true = np.array([o[2] for o in out])
        rr_true = np.array([o[3] for o in out])
        ypr = np.array([o[4] for o in out])
        yrr = np.array([o[5] for o in out])
        mp, mr = np.median(pr_true), np.median(rr_true)
        mfp, mfr = np.median(ypr), np.median(yrr)
        print(f"epoch tow {tow:.3f} (GPS T{T:+.3f}): the filter's receiver {out[0][8]:.1f} m and {out[0][9]:.2f} m/s "
              f"from the truth; common terms: truth pr {mp:+.1f} m, rr {mr:+.2f} m/s | filter pr {mfp:+.1f} m, rr "
              f"{mfr:+.2f} m/s")
        print("   sat     truth residual (centred): pr m    Doppler m/s  | filter innovation (centred): pr m   "
              "Doppler m/s  sigma_rr")
        for o, a1, a2, b1, b2 in zip(out, pr_true - mp, rr_true - mr, ypr - mfp, yrr - mfr):
            name = ("G", "E", "C")[o[0]] + f"{o[1]:02d}"
            print(f"   {name}   {a1:+10.2f}   {a2:+10.2f}               | {b1:+10.2f}   {b2:+10.2f}   {o[6]:6.2f}")
        # the velocity error the Dopplers point to, least squares on the centred truth residuals (and a clock rate)
        U = np.array([o[7] for o in out])
        H = np.hstack([-U, np.ones((len(out), 1))])
        sol, *_ = np.linalg.lstsq(H, rr_true, rcond=None)
        dv_ned = M.T @ sol[:3]
        solf, *_ = np.linalg.lstsq(H, yrr, rcond=None)
        dvf_ned = M.T @ solf[:3]
        print(f"   Dopplers vs truth point to a velocity error N {dv_ned[0]:+.2f} E {dv_ned[1]:+.2f} D {dv_ned[2]:+.2f} m/s "
              f"(the measurements' own); vs the filter: N {dvf_ned[0]:+.2f} E {dvf_ned[1]:+.2f} D {dvf_ned[2]:+.2f} m/s")


if __name__ == "__main__":
    main()
