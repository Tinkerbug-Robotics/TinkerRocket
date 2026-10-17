#!/usr/bin/env python3
"""Score a rig replay's heading from thrust (HEADING_FIT_MSG 0x9F, TR_RawGnssShadow/HeadingFit.h) from the OC's
flight log: each fit on the flight's time, when its 1-sigma first came under 10 and 5 deg, the hand-off in act
mode, and what the fit cost the flight computer.

    score_heading.py OC_LOG [--ref OC_LOG] [--error DEG] [--blob REPLAY.bin]
                     [--flight sim_rp5_0829 | --csv SENSORS_CSV --scenario JSON] [--truth-quat q0,q1,q2,q3]

--ref: the same replay flown without -DTR_BENCH_HEADING_ERROR_DEG. The fit's turn rights the pad's heading, so the
two runs' turns differ by the error the image was built with (--error: what it should be), whatever the
magnetometer's own error; their navigation filters' headings differ by it until a hand-off.

--truth-quat: the sensors CSV's columns of the truth attitude (body FRD to NED, scalar first), if it has one: the
filter's heading and each fit are then scored against it too. Without it the columns that look like one are
listed.

The log's clock is put on the flight's time by the ignition edge, as score_log.py puts it (--blob).
"""
import argparse
import gzip
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "Data_Analysis"))
import plot_flight_data_mini as P  # noqa: E402
from truth import edge_t, flight_files, log_time_base, unwrap_us  # noqa: E402


def wrap_deg(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def twist_deg(a, b):
    """The rotation about NED down (deg) that takes attitude b to a: a's heading less b's, whatever their tilts
    (HeadingFit.cpp headingBetween). Attitudes body to NED, scalar first; n x 4 each."""
    a, b = np.atleast_2d(a), np.atleast_2d(b)
    r0 = a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1] + a[:, 2] * b[:, 2] + a[:, 3] * b[:, 3]
    r3 = -a[:, 0] * b[:, 3] - a[:, 1] * b[:, 2] + a[:, 2] * b[:, 1] + a[:, 3] * b[:, 0]
    return wrap_deg(np.degrees(2.0 * np.arctan2(r3, r0)))


def fits(recs, base):
    """The heading records as columns, on the flight's time."""
    h = recs.get("HeadingFit") or []
    if not h:
        return None
    col = lambda k: np.array([r[k] for r in h], float)
    return dict(t=unwrap_us([r["time_us"] for r in h]) * 1e-6 + base, psi=col("psi_deg"),
                sigma=np.array([np.nan if r["sigma_deg"] is None else r["sigma_deg"] for r in h]),
                filt=col("filt_deg"), turn=col("turn_deg"), n=col("n"), n_cut=col("n_cut"), sats=col("sats"),
                vel_us=col("vel_us"), solve_us=col("solve_us"), seed_ms=col("seed_ms"),
                speed=col("speed_mps"), act=np.array([r["act"] for r in h]),
                handed_now=np.array([r["handed_now"] for r in h]), rival=np.array([r["rival"] for r in h]))


def filter_quats(recs, base):
    """The raw filter's attitude (0x96, 10 Hz) on the flight's time: t, n x 4."""
    sh = [r for r in recs.get("RawGnssEkf") or [] if r.get("running", 1)]
    if not sh:
        return None, None
    t = unwrap_us([r["time_us"] for r in sh]) * 1e-6 + base
    return t, np.array([[r["q0"], r["q1"], r["q2"], r["q3"]] for r in sh], float)


def at(t, tq, q):
    """Attitudes at times t: the nearest logged (10 Hz is close enough for a heading)."""
    i = np.clip(np.searchsorted(tq, t), 1, len(tq) - 1)
    i = np.where(np.abs(tq[i - 1] - t) < np.abs(tq[i] - t), i - 1, i)
    return q[i]


def summary(f):
    """When the 1-sigma first came under 10 and 5 deg (t, its turn), the hand-off, the costs."""
    out = {}
    for lim in (10.0, 5.0):
        k = np.flatnonzero(f["sigma"] <= lim)
        out[f"sig{lim:g}"] = (float(f["t"][k[0]]), float(f["psi"][k[0]])) if k.size else None
    k = np.flatnonzero(f["handed_now"])
    out["handed"] = (float(f["t"][k[0]]), float(f["turn"][k[0]]), float(f["sigma"][k[0]])) if k.size else None
    out["last"] = (float(f["t"][-1]), float(f["psi"][-1]), float(f["sigma"][-1]))
    out["vel_us_max"], out["solve_us_max"] = float(f["vel_us"].max()), float(f["solve_us"].max())
    out["rivals"] = int(f["rival"].sum())
    return out


def compare(f, g):
    """Fit against fit, epoch by epoch (the same flight time within 0.2 s): the turns' difference (deg)."""
    out = []
    for i, t in enumerate(f["t"]):
        j = int(np.argmin(np.abs(g["t"] - t)))
        if abs(g["t"][j] - t) <= 0.2:
            out.append((float(t), float(wrap_deg(f["psi"][i] - g["psi"][j])), f["sigma"][i], g["sigma"][j]))
    return out


def truth_quats(csv_path, cols):
    """The sensors CSV's truth attitude: t, n x 4 (normalized)."""
    lines = [ln for ln in gzip.open(csv_path, "rt") if not ln.startswith("#")]
    names = lines[0].strip().split(",")
    d = np.loadtxt(lines[1:], delimiter=",", usecols=[names.index(c) for c in ["t_s"] + cols])
    q = d[:, 1:5]
    return d[:, 0], q / np.linalg.norm(q, axis=1)[:, None]


def attitude_columns(csv_path):
    names = next(ln for ln in gzip.open(csv_path, "rt") if not ln.startswith("#")).strip().split(",")
    return [c for c in names if any(k in c.lower() for k in ("q0", "qw", "quat", "yaw", "head", "roll", "att"))]


def show(name, f):
    s = summary(f)
    act = bool(f["act"].any())
    print(f"\n{name}: {len(f['t'])} fits ({'act' if act else 'observe'}), the seed {f['seed_ms'][0]:.0f} ms before launch")
    print("     t (s)   turn   1-sigma  filt-fit   turned  epochs cut sats  speed   vel_us solve_us  flags")
    for i in range(len(f["t"])):
        flags = ("H" if f["handed_now"][i] else "") + ("R" if f["rival"][i] else "")
        sig = "   --  " if np.isnan(f["sigma"][i]) else f"{f['sigma'][i]:7.2f}"
        print(f"  {f['t'][i]:+8.2f} {f['psi'][i]:+7.2f} {sig}  {f['filt'][i]:+8.2f} {f['turn'][i]:+8.2f}"
              f"  {f['n'][i]:5.0f} {f['n_cut'][i]:3.0f} {f['sats'][i]:4.0f} {f['speed'][i]:6.1f}"
              f"  {f['vel_us'][i]:7.0f} {f['solve_us'][i]:8.0f}  {flags}")
    fmt = lambda v: "never" if v is None else f"at {v[0]:+.2f} s (turn {v[1]:+.2f} deg)"
    print(f"  1-sigma under 10 deg {fmt(s['sig10'])}, under 5 {fmt(s['sig5'])}")
    if s["handed"]:
        print(f"  handed over at {s['handed'][0]:+.2f} s: turned {s['handed'][1]:+.2f} deg (1-sigma {s['handed'][2]:.2f})")
    print(f"  cost: velocity {s['vel_us_max']:.0f} us max, fit {s['solve_us_max']:.0f} us max; "
          f"{s['rivals']} fits with a rival valley")
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("log")
    ap.add_argument("--ref", help="the same replay without the bench heading error")
    ap.add_argument("--error", type=float, help="the bench heading error the log's image was built with, deg")
    ap.add_argument("--flight", default="sim_rp5_0829", help="its files under $COCOM_SDR/scenarios")
    ap.add_argument("--csv", help="the sensors CSV, if not the flight's")
    ap.add_argument("--scenario", help="the scenario JSON, if not the flight's")
    ap.add_argument("--blob", help="the replay file the image carried (its ignition edge)")
    ap.add_argument("--truth-quat", dest="truth_quat", help="the CSV's truth attitude columns, q0,q1,q2,q3")
    args = ap.parse_args()
    csv, scen = flight_files(args.flight)
    csv, scen = args.csv or csv, args.scenario or scen
    edge = edge_t(csv, scen, args.blob)

    runs = []
    for name, path in (("this run", args.log), ("the reference", args.ref)):
        if not path:
            continue
        recs, _stats, _cfg = P.parse_binary_file(path)
        base = log_time_base(recs, edge)
        f = fits(recs, base)
        if f is None:
            print(f"{name}: no heading records (0x9F) in {path}")
            return 1
        tq, q = filter_quats(recs, base)
        runs.append((name, f, tq, q))
        show(f"{name} ({os.path.basename(path)})", f)

    if args.truth_quat:
        tt, qt = truth_quats(csv, args.truth_quat.split(","))
        for name, f, tq, q in runs:
            if tq is None:
                continue
            e_filt = twist_deg(q, at(tq, tt, qt))                      # the filter's heading less the truth's
            e_at = np.interp(f["t"], tq, np.unwrap(np.radians(e_filt)))
            e_fit = wrap_deg(np.degrees(e_at) - f["filt"])               # the fit's: the filter's less (filter - fit)
            print(f"\n{name} against the truth: the fit's heading error by epoch (deg): "
                  + " ".join(f"{t:+.1f}s:{e:+.1f}" for t, e in zip(f["t"], e_fit)))
            for t in (0.0, 2.0, 5.0, 10.0, 20.0):
                k = int(np.argmin(np.abs(tq - t)))
                print(f"  the filter's heading error at {tq[k]:+.1f} s: {e_filt[k]:+.2f} deg")
    else:
        cols = attitude_columns(csv) if os.path.exists(csv) else []
        print(f"\n(no --truth-quat: the CSV's columns that may hold an attitude: {cols or 'none'})")

    if len(runs) == 2:
        (_, f, tq, q), (_, g, tr, qr) = runs
        c = compare(f, g)
        d = np.array([x[1] for x in c])
        print(f"\nthis run's turn less the reference's, {len(c)} epochs matched: median {np.median(d):+.2f} deg, "
              f"range {d.min():+.2f} to {d.max():+.2f}" + (f" (built with {args.error:+g}: the turn should be "
                                                          f"{-args.error:+g})" if args.error is not None else ""))
        for t, dd, s1, s2 in c:
            print(f"  {t:+7.2f} s  {dd:+7.2f} deg  (1-sigma {s1:.2f} / {s2:.2f})")
        if tq is not None and tr is not None:
            print("the navigation filters' headings, this run less the reference:")
            for t in (-5.0, 0.0, 2.0, 4.0, 6.0, 10.0, 20.0, 40.0):
                if tq[0] <= t <= tq[-1] and tr[0] <= t <= tr[-1]:
                    print(f"  {t:+6.1f} s  {float(twist_deg(at(t, tq, q), at(t, tr, qr))[0]):+7.2f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
