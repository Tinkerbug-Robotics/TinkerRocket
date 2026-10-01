#!/usr/bin/env python3
"""Each satellite's line-of-sight truth along a scenario, for trksim: one CSV per PRN with
t_s, dop_hz, rate_hz_s, el_deg at the trajectory's own sample times.

    los_truth.py --traj SCEN.csv --nav BRDC.rnx --t0-gps 203400 --from 570 --to 650 --out DIR

Times are file seconds; --t0-gps is the GPS time (s of week) of file second 0. Satellites
below --min-el (default 5 deg) at --from are left out.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gnssrx import rinex, truth  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--traj", type=Path, required=True)
    ap.add_argument("--nav", type=Path, required=True)
    ap.add_argument("--t0-gps", type=float, required=True)
    ap.add_argument("--from", dest="t_from", type=float, required=True)
    ap.add_argument("--to", dest="t_to", type=float, required=True)
    ap.add_argument("--min-el", type=float, default=5.0)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()

    nav = rinex.read_gps(a.nav)
    traj = truth.Trajectory(a.traj)
    t = traj.t[(traj.t >= a.t_from) & (traj.t <= a.t_to)]
    a.out.mkdir(parents=True, exist_ok=True)
    for prn in sorted({e.prn for e in nav}):
        el0 = truth.los(nav, prn, traj, t[:1], a.t0_gps)[0, 3]
        if el0 < a.min_el:
            continue
        L = truth.los(nav, prn, traj, t, a.t0_gps)
        path = a.out / f"prn{prn:02d}.csv"
        with open(path, "w") as f:
            f.write("t_s,dop_hz,rate_hz_s,el_deg\n")
            for k in range(t.size):
                f.write(f"{t[k]:.3f},{L[k, 1]:.4f},{L[k, 2]:.3f},{L[k, 3]:.3f}\n")
        print(f"PRN {prn:2d}: el {L[0, 3]:5.1f} deg, |rate| max {np.abs(L[:, 2]).max():7.1f} Hz/s -> {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
