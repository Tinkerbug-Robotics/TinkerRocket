#!/usr/bin/env python3
"""Score a rig replay's flight log (the OC's file) against the flight's truth:
the raw-GNSS filter's records (0x96), the navigation state the flight computer
logs (NonSensor), the receiver's own fixes (GNSS) and the physics checks (0x9A)
-- is each flagged fix really off, and does any badly-off fix pass?

    score_log.py OC_LOG [--flight sim_rp5_0829 | --csv SENSORS_CSV --scenario JSON] [--blob REPLAY.bin] [--npz OUT]

The log's MCU clock is put on the flight's time by the ignition edge in the
replayed accelerometer: where the replay file's 500 Hz pairs cross 20 m/s^2
(rp5_0829's at t = +3.56 ms). Give --blob the file the replay image was built
with; without it the edge is worked out from the flight's CSV as
sim_replay_blob.py writes it with its default --from. Heights: the raw filter's are ellipsoidal, scored as they are;
the receiver's are above the geoid, so its median offset on the pad is
removed (and printed); NonSensor's ENU is from the flight computer's own pad
reference.
"""
import argparse
import datetime as dt
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "Data_Analysis"))
import plot_flight_data_mini as P  # noqa: E402
from truth import EDGE_MPS2, Truth, edge_t, flight_files  # noqa: E402


def unwrap_us(arr):
    a = np.asarray(arr, dtype=np.int64)
    wraps = np.cumsum(np.diff(a, prepend=a[0]) < -2 ** 31)
    return a + wraps * 2 ** 32


def phase_stats(tr, name, t, ep, ev, sel=None):
    print(f"\n{name}")
    print("  phase    (from, to s)      n   horiz p95/max (m)   vert p95/max (m)   vel p95/max (m/s)")
    for label, lo, hi in tr.windows():
        m = (t >= lo) & (t < hi)
        if sel is not None:
            m &= sel
        if not m.any():
            print(f"  {label:7s} {lo:+7.2f} {hi:+7.2f} {0:5d}")
            continue
        h = np.hypot(ep[m, 0], ep[m, 1])
        v = np.abs(ep[m, 2])
        vv = np.linalg.norm(ev[m], axis=1)
        print(f"  {label:7s} {lo:+7.2f} {hi:+7.2f} {m.sum():5d}   {np.percentile(h, 95):7.1f} {h.max():7.1f}     "
              f"{np.percentile(v, 95):7.1f} {v.max():7.1f}     {np.percentile(vv, 95):6.2f} {vv.max():6.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("log")
    ap.add_argument("--flight", default="sim_rp5_0829", help="its files under $COCOM_SDR/scenarios")
    ap.add_argument("--csv", help="the sensors CSV, if not the flight's")
    ap.add_argument("--scenario", help="the scenario JSON, if not the flight's")
    ap.add_argument("--blob", help="the replay file the image carried (its ignition edge)")
    ap.add_argument("--npz", help="write the error series here")
    args = ap.parse_args()
    csv, scen = flight_files(args.flight)
    csv, scen = args.csv or csv, args.scenario or scen
    tr = Truth(csv, scen)
    edge = edge_t(csv, scen, args.blob)

    recs, _stats, _cfg = P.parse_binary_file(args.log)
    print({k: len(v) for k, v in recs.items() if v})

    # ---- the MCU clock on the flight's time: the ignition edge
    imu = recs["ISM6HG256"]
    it = unwrap_us([r["time_us"] for r in imu]) * 1e-6
    lo = np.array([[r["low_acc_x"], r["low_acc_y"], r["low_acc_z"]] for r in imu])
    hi = np.array([[r["high_acc_x"], r["high_acc_y"], r["high_acc_z"]] for r in imu])
    near_rail = np.array([r["low_g_near_rail"] for r in imu])
    amag = np.where(near_rail, np.linalg.norm(hi, axis=1), np.linalg.norm(lo, axis=1))
    first = int(np.argmax(amag > EDGE_MPS2))
    k = first - 1
    t_cross = it[k] + (EDGE_MPS2 - amag[k]) / (amag[first] - amag[k]) * (it[first] - it[k])
    base = edge - t_cross
    print(f"time map: t = MCU s {base:+.4f}, from the ignition edge (the replay's at t = {edge * 1e3:+.3f} ms)")

    # ---- the raw-GNSS filter's records
    sh = recs["RawGnssEkf"]
    st = unwrap_us([r["time_us"] for r in sh]) * 1e-6 + base
    run = np.array([r["running"] for r in sh]).astype(bool)
    sp = tr.ned_from_lla([r["lat_deg"] for r in sh], [r["lon_deg"] for r in sh], [r["alt_m"] for r in sh])
    sv = np.array([[r["vel_n"], r["vel_e"], r["vel_d"]] for r in sh])
    x = tr.at(st)
    s_ep, s_ev = sp - x[:, 0:3], sv - x[:, 3:6]

    # ---- the navigation state the flight computer logs (ENU from its pad reference)
    ns = recs["NonSensor"]
    nt = unwrap_us([r["time_us"] for r in ns]) * 1e-6 + base
    npos = np.array([[r["n_pos"], r["e_pos"], -r["u_pos"]] for r in ns])
    nvel = np.array([[r["n_vel"], r["e_vel"], -r["u_vel"]] for r in ns])
    x = tr.at(nt)
    f_ep, f_ev = npos - x[:, 0:3], nvel - x[:, 3:6]

    # ---- the receiver's fixes, on their own UTC against the ignition's
    ign = tr.ignition_utc()
    gn = [r for r in recs["GNSS"] if r["fix_mode"] >= 2 and r["year"] >= 2020]
    gt = np.array([(dt.datetime(r["year"], r["month"], r["day"], r["hour"], r["minute"], r["second"])
                    + dt.timedelta(milliseconds=r["milli_sec"]) - ign).total_seconds() for r in gn])
    gp = tr.ned_from_lla([r["lat"] for r in gn], [r["lon"] for r in gn], [r["alt_m"] for r in gn])
    gv = np.array([[r["vel_n"], r["vel_e"], -r["vel_u"]] for r in gn])
    x = tr.at(gt)
    g_ep, g_ev = gp - x[:, 0:3], gv - x[:, 3:6]
    pad = (gt > -120) & (gt < 0)
    sep = float(np.median(g_ep[pad, 2])) if pad.any() else 0.0
    g_ep[:, 2] -= sep
    print(f"receiver: {len(gn)} fixes, its height {sep:+.1f} m from ellipsoidal on the pad (the geoid), removed")

    phase_stats(tr, "RAW-GNSS FILTER (its records, while running)", st, s_ep, s_ev, run)
    phase_stats(tr, "NAVIGATION STATE (NonSensor)", nt, f_ep, f_ev)
    phase_stats(tr, "RECEIVER'S OWN FIXES", gt, g_ep, g_ev)

    chk = recs["GnssCheck"]
    if chk:
        verd = np.array([r["verdict"] for r in chk])
        tc = unwrap_us([r["time_us"] for r in chk]) * 1e-6 + base
        idx = np.clip(np.searchsorted(gt, tc), 1, len(gt) - 1)
        pick = np.where(np.abs(gt[idx - 1] - tc) < np.abs(gt[idx] - tc), idx - 1, idx)
        ok = np.abs(gt[pick] - tc) < 0.03
        herr = np.linalg.norm(g_ep[pick, :], axis=1)
        verr = np.linalg.norm(g_ev[pick, :], axis=1)
        print(f"\nCHECKS vs TRUTH ({ok.sum()} of {len(chk)} check records matched to a fix within 30 ms)")
        for v in ("pass", "flagged", "unjudged"):
            m = ok & (verd == v)
            if m.any():
                print(f"  {v:8s} {m.sum():5d}: true pos err median {np.median(herr[m]):6.1f} "
                      f"p95 {np.percentile(herr[m], 95):7.1f} max {herr[m].max():7.1f} m; vel err median "
                      f"{np.median(verr[m]):5.2f} p95 {np.percentile(verr[m], 95):6.2f} max {verr[m].max():6.2f} m/s")
        bad = ok & ((herr > 15) | (verr > 3))
        good = ok & (herr < 5) & (verr < 1)
        print(f"  fixes truly off (>15 m or >3 m/s): {bad.sum()}; flagged {np.count_nonzero(bad & (verd == 'flagged'))}, "
              f"passed {np.count_nonzero(bad & (verd == 'pass'))}, unjudged {np.count_nonzero(bad & (verd == 'unjudged'))}")
        print(f"  fixes truly good (<5 m and <1 m/s): {good.sum()}; flagged {np.count_nonzero(good & (verd == 'flagged'))}")
    else:
        print("\nno check records (0x9A) in this log")

    if args.npz:
        np.savez(args.npz, st=st, s_ep=s_ep, s_ev=s_ev, run=run, nt=nt, f_ep=f_ep, f_ev=f_ev, gt=gt, g_ep=g_ep,
                 g_ev=g_ev)


if __name__ == "__main__":
    main()
