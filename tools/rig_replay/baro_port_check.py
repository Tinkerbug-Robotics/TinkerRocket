#!/usr/bin/env python3
"""The barometer against the truth through a rig replay: its height error
(pad-relative pressure altitude) beside the raw-GNSS filter's modelled
static-port sigma (baro_port_k * q, from the truth's speed and the vertical
speed alone), to see when the filter should hold it out (#1600; #1579's term).

    baro_port_check.py OC_LOG [--flight sim_rp5_0829 | --csv SENSORS_CSV]
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "Data_Analysis"))
import plot_flight_data_mini as P  # noqa: E402
from score_log import EDGE_T, unwrap_us  # noqa: E402
from truth import Truth, flight_files  # noqa: E402

PORT_K = 0.0075            # m per Pa: RawGnssShadow::Config::baro_port_k
HOLD_M = 2.0               #   baro_hold_port_m


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("log")
    ap.add_argument("--flight", default="sim_rp5_0829")
    ap.add_argument("--csv")
    args = ap.parse_args()
    tr = Truth(args.csv or flight_files(args.flight)[0])
    recs, _s, _c = P.parse_binary_file(args.log)

    imu = recs["ISM6HG256"]
    it = unwrap_us([r["time_us"] for r in imu]) * 1e-6
    lo = np.array([[r["low_acc_x"], r["low_acc_y"], r["low_acc_z"]] for r in imu])
    amag = np.linalg.norm(lo, axis=1)
    first = int(np.argmax(amag > 20.0))
    k = first - 1
    base = EDGE_T - (it[k] + (20.0 - amag[k]) / (amag[first] - amag[k]) * (it[first] - it[k]))

    b = recs["BMP585"]
    bt = unwrap_us([r["time_us"] for r in b]) * 1e-6 + base
    p = np.array([r["pressure_pa"] for r in b], dtype=float)
    p0 = np.median(p[(bt > -20) & (bt < -1)])
    h_baro = 44330.0 * (1.0 - (p / p0) ** (1.0 / 5.255))
    x = tr.at(bt)
    h_true = -x[:, 2]
    rho = 1.225 * np.exp(-h_true / 8500.0)
    q = 0.5 * rho * np.sum(x[:, 3:6] ** 2, axis=1)
    q_v = 0.5 * rho * x[:, 5] ** 2
    err = h_baro - h_true
    print("      t   true h  speed     q Pa  port sigma  at vd    baro err")
    for t0 in (-1, 0.5, 1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 20, 30, 40, 50, 60):
        i = int(np.argmin(np.abs(bt - t0)))
        print(f"  {bt[i]:+6.2f} {h_true[i]:7.1f} {np.sqrt(2 * q[i] / rho[i]):6.1f} {q[i]:8.0f} "
              f"{PORT_K * q[i]:9.2f} {PORT_K * q_v[i]:7.2f} {err[i]:+9.2f}")
    flown = (bt > 0) & (bt < tr.edges["landed"])
    held = flown & (PORT_K * q_v > HOLD_M)
    if held.any():
        print(f"held out (port sigma at the vertical speed > {HOLD_M} m): t {bt[held][0]:+.2f} .. {bt[held][-1]:+.2f} s; "
              f"worst |error| while held {np.abs(err[held]).max():.1f} m, fused in flight "
              f"{np.abs(err[flown & ~held & (bt > 1)]).max():.1f} m")


if __name__ == "__main__":
    main()
