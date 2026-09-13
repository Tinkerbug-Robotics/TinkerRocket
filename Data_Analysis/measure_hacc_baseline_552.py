#!/usr/bin/env python3
"""#552 — does `hacc` actually improve the landing-prediction uncertainty?

The app has no GNSS velocity on the wire, so it makes one by differencing the
GNSS positions the telemetry frame already carries, and compares that against
the filter's velocity.  The gap is the uncertainty the ascent prediction
inherits (see `ascentVelocitySpreadMeters` in both apps).

Differencing two positions each uncertain by sigma over an interval dt gives a
velocity uncertain by sqrt(2)*sigma/dt.  On a flight with poor fixes that noise
swamps the measurement: Rolly Polly V flew four satellites at 29 m, which over
the 0.5 s frame interval is 82 m/s of pure noise.

This script A/Bs the two ways of spending `hacc` against landing truth:

  PINNED      ignore hacc, difference at the frame interval (what shipped
              before hacc reached the app)
  ADAPTIVE    let hacc choose the interval — long enough that the differencing
              noise sits under TARGET_NOISE_MPS, capped at MAX_BASELINE_S

and reports, per flight, what fraction of ascent predictions the resulting
radius actually covers, and how big that radius is relative to the real error.
A radius that covers is honest; one hugely larger than the error is useless,
so both columns have to be read together.

Result on the 2026-08-29 BARC set (four flights, one launch day): adaptive is
worth a 44% tighter radius on the bad-GNSS flight at slightly better coverage,
and changes nothing on the three healthy flights, whose hacc is 0-1 m.

Two findings that did NOT survive measurement, recorded so they are not
rebuilt: subtracting the noise floor in quadrature is worth almost nothing
(the over-read is driven by outlier fixes, not by a Gaussian at the sigma
level), and hacc is useless as a general confidence signal — the WORST flight
in this set, RIM-66 at 196 m median error, has a perfect hacc of 0 m, because
its error is a descent-propagation bias that nothing in the frame observes.

Usage:
    python3 measure_hacc_baseline_552.py <sweep.csv> <flight.bin>:<label> ...

where <sweep.csv> is the output of `sweep_landing_predictor.py` (columns
flight, phase, t_loss_s, err_m, pred_e, pred_n).
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from plot_flight_data_mini import parse_binary_file           # noqa: E402
from _ekf_logged import logged_binary                          # noqa: E402
from _ekf_replay import lla_rad_to_enu_m                       # noqa: E402

# These MUST match GnssVelocityCheck.kt / GnssVelocityCheck.swift.
TARGET_NOISE_MPS = 3.0
MAX_BASELINE_S = 4.0
QUANTIZATION_SIGMA_M = 1.11 / math.sqrt(12.0)   # lat/lon ship at 5 dp

FRAME_RATE_HZ = 2.0        # base-station relay rate
FLOOR_M = 25.0             # baseline bias seen on the healthy flights
SCATTER_K = 3.0
SCATTER_WINDOW_S = 2.0


def _flight_series(bin_path: Path):
    """Everything the app would have had, on the app's own 2 Hz grid."""
    rec, _, _ = parse_binary_file(str(bin_path))
    ref = logged_binary(rec, verbose=False).launch_ref
    ns = [r for r in rec["NonSensor"] if r.get("time_us") is not None]
    if not ns:
        raise RuntimeError(f"{bin_path}: no NonSensor rows")
    t0 = next((r["time_us"] for r in ns if r.get("launch")), ns[0]["time_us"])
    t_ns = (np.array([r["time_us"] for r in ns], float) - t0) / 1e6
    ve = np.array([float(r.get("e_vel") or 0) for r in ns])
    vn = np.array([float(r.get("n_vel") or 0) for r in ns])
    u = np.array([float(r.get("u_pos") or 0) for r in ns])
    apogee_s = t_ns[int(np.argmax(u))]

    g = [r for r in rec["GNSS"] if (r.get("num_sats") or 0) >= 4]
    t_g = (np.array([r["time_us"] for r in g], float) - t0) / 1e6
    lat = np.array([r["lat"] for r in g])
    lon = np.array([r["lon"] for r in g])
    h_acc = np.array([float(r["h_acc_m"]) if r.get("h_acc_m") is not None else 255.0
                      for r in g])

    dt = 1.0 / FRAME_RATE_HZ
    t_f = np.arange(0.0, apogee_s, dt)
    # Round to the wire's 5 dp before using it: the app never sees more.
    fe, fn = [], []
    for a, b in zip(np.round(np.interp(t_f, t_g, lat), 5),
                    np.round(np.interp(t_f, t_g, lon), 5)):
        e_, n_, _ = lla_rad_to_enu_m(math.radians(a), math.radians(b), 0.0, ref)
        fe.append(e_)
        fn.append(n_)
    return dict(t_f=t_f, fe=np.array(fe), fn=np.array(fn), dt=dt,
                t_ns=t_ns, ve=ve, vn=vn, apogee_s=apogee_s,
                h_acc=np.interp(t_f, t_g, np.clip(h_acc, 0, 254)))


def disagreement(s, *, use_hacc: bool):
    """|filter velocity - GNSS-derived velocity| on the frame grid."""
    t_f, fe, fn, dt = s["t_f"], s["fe"], s["fn"], s["dt"]
    sigma = np.hypot(np.maximum(s["h_acc"], 0.0), QUANTIZATION_SIGMA_M)
    out = np.zeros(len(t_f))
    for i in range(len(t_f)):
        want = math.sqrt(2.0) * sigma[i] / TARGET_NOISE_MPS if use_hacc else dt
        base = min(MAX_BASELINE_S, max(dt, math.ceil(want / dt) * dt))
        j = max(0, i - int(round(base / dt)))
        span = t_f[i] - t_f[j]
        if span <= 0:
            continue
        # Chord velocity is an AVERAGE, so compare it with the filter's average
        # over the same window, never with one endpoint.
        m = (s["t_ns"] >= t_f[j]) & (s["t_ns"] <= t_f[i])
        if m.sum() < 2:
            m = np.abs(s["t_ns"] - t_f[i]) < dt
        out[i] = math.hypot(s["ve"][m].mean() - (fe[i] - fe[j]) / span,
                            s["vn"][m].mean() - (fn[i] - fn[j]) / span)
    return out


def _scatter(t, pe, pn):
    """Spread of the last SCATTER_WINDOW_S of predictions — self-consistency."""
    sc = np.zeros(len(t))
    for i in range(len(t)):
        m = (t >= t[i] - SCATTER_WINDOW_S) & (t <= t[i])
        if m.sum() >= 3:
            sc[i] = np.sqrt(np.mean((pe[m] - pe[m].mean()) ** 2
                                    + (pn[m] - pn[m].mean()) ** 2))
    return sc


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sweep_csv", type=Path)
    ap.add_argument("flights", nargs="+",
                    help="<flight.bin>:<label as it appears in the sweep csv>")
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.sweep_csv)))
    results = {}
    for spec in args.flights:
        path, _, label = spec.rpartition(":")
        if not path:
            ap.error(f"expected <flight.bin>:<label>, got {spec!r}")
        s = _flight_series(Path(path))
        pts = sorted([r for r in rows if r["flight"] == label and r["phase"] == "ASCENT"],
                     key=lambda r: float(r["t_loss_s"]))
        if not pts:
            print(f"  (no ASCENT rows for {label!r} in {args.sweep_csv})")
            continue
        t = np.array([float(r["t_loss_s"]) for r in pts])
        err = np.array([float(r["err_m"]) for r in pts])
        sc = _scatter(t, np.array([float(r["pred_e"]) for r in pts]),
                      np.array([float(r["pred_n"]) for r in pts]))
        keep = t <= s["apogee_s"]
        t, err, sc = t[keep], err[keep], sc[keep]
        if not len(t):
            continue
        lever = np.maximum(s["apogee_s"] - t, 0.0)
        per = {}
        for name, use in (("PINNED", False), ("ADAPTIVE", True)):
            v = np.interp(t, s["t_f"], disagreement(s, use_hacc=use))
            per[name] = np.maximum.reduce([np.full(len(t), FLOOR_M),
                                           v * lever, SCATTER_K * sc])
        results[label] = (per, err, np.median(s["h_acc"]))

    for name in ("PINNED", "ADAPTIVE"):
        cov = n = 0
        print(f"\n===== {name} "
              f"{'(hacc ignored, frame-rate differencing)' if name == 'PINNED' else '(hacc picks the interval)'} =====")
        print(f"{'flight':<24} {'n':>4} {'med hacc':>9} {'covered':>8} "
              f"{'med radius':>11} {'med error':>10} {'med ratio':>10}")
        for label, (per, err, med_h) in results.items():
            r = per[name]
            c = err <= r
            cov += int(c.sum())
            n += len(err)
            print(f"{label:<24} {len(err):>4} {med_h:>9.1f} {100 * c.mean():>7.0f}% "
                  f"{np.median(r):>11.0f} {np.median(err):>10.0f} "
                  f"{np.median(r / np.maximum(err, 1e-6)):>10.2f}")
        if n:
            print(f"overall coverage {100 * cov / n:.0f}% of {n} ascent predictions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
