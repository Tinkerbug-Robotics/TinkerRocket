#!/usr/bin/env python3
"""Sweep T_loss across all flights in a directory and chart accuracy.

For each flight in `TestFlights/<date>/`, replay through the firmware EKF,
then for every T_loss in [boost_end, ekf_end] compute the predicted
landing error vs the LoRa-truth landing.  Produces:

  - per-flight error-vs-T_loss curve, colored by phase (ASCENT / drogue / main)
  - 4-up grid combining all flights
  - aggregate CSV at test_data/landing_pred_sweep.csv

Usage:
    python sweep_landing_predictor.py [<flights_dir>] [--step 0.5]
"""

from __future__ import annotations

import sys
import math
import csv
import argparse
from pathlib import Path
from dataclasses import dataclass

import numpy as np
import matplotlib
matplotlib.use("MacOSX")
import matplotlib.pyplot as plt

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "tinkerrocket-sim" / "src"))

from plot_flight_data_mini import parse_binary_file
from _ekf_replay import replay_binary
from _wind_profile import fetch_wind
from landing_predictor import (snapshot_at, predict_landing, actual_landing_enu,
                                RocketProfile, _infer_flight_datetime_utc,
                                _find_lora_csv, _find_binary)

DEFAULT_FLIGHTS_DIR = ("/Users/christianpedersen/Documents/Hobbies/ModelRockets/"
                       "TestFlights/2026_05_17")


@dataclass
class SweepPoint:
    t_loss_s: float
    phase: str
    err_m: float
    pred_e: float
    pred_n: float
    snap_e: float
    snap_n: float
    snap_u: float
    snap_vu: float
    used_gnss: bool


def sweep_flight(bin_path: Path, step_s: float,
                 utc_offset_h: float) -> dict:
    records, _, _ = parse_binary_file(str(bin_path))
    res = replay_binary(records, verbose=False)
    t0_us = res.phases.t0_us
    boost_end_s = ((res.phases.boost_end_us - t0_us)/1e6
                   if res.phases.boost_end_us else 1.0)
    apogee_s = ((res.phases.apogee_us - t0_us)/1e6
                if res.phases.apogee_us else None)
    ekf_end_s = float((res.ekf.t_us[-1] - t0_us) / 1e6) if len(res.ekf.t_us) else 0

    # Wind for the flight
    dt_utc = _infer_flight_datetime_utc(bin_path, utc_offset_h)
    wind = None
    if dt_utc is not None:
        try:
            wind = fetch_wind(res.launch_ref.lat_deg, res.launch_ref.lon_deg,
                              dt_utc, cache_dir=_HERE / "test_data" / "wind_cache")
        except Exception as e:
            print(f"    (wind fetch failed: {e})")

    act_e, act_n, _, act_t = actual_landing_enu(res, _find_lora_csv(bin_path))
    profile = RocketProfile()

    points: list[SweepPoint] = []
    t = max(boost_end_s + 0.2, 0.5)
    while t < ekf_end_s - 0.2:
        snap_ekf = snapshot_at(res, t, prefer_gnss_for_descent=False)
        snap = snapshot_at(res, t, prefer_gnss_for_descent=True)
        used_gnss = (snap.e_m != snap_ekf.e_m or snap.n_m != snap_ekf.n_m
                     or snap.u_m != snap_ekf.u_m)
        pred = predict_landing(snap, profile, wind=wind)
        err = math.hypot(pred.landing_e_m - act_e, pred.landing_n_m - act_n)
        points.append(SweepPoint(
            t_loss_s=t, phase=pred.phase, err_m=err,
            pred_e=pred.landing_e_m, pred_n=pred.landing_n_m,
            snap_e=snap.e_m, snap_n=snap.n_m, snap_u=snap.u_m,
            snap_vu=snap.vu_mps, used_gnss=used_gnss,
        ))
        t += step_s

    return dict(
        name=bin_path.parent.name.strip(),
        bin_path=bin_path,
        points=points,
        boost_end_s=boost_end_s,
        apogee_s=apogee_s,
        ekf_end_s=ekf_end_s,
        actual=(act_e, act_n, act_t),
        wind=wind,
    )


_PHASE_COLOR = {
    "ASCENT":         "#E67E22",
    "DESCENT_DROGUE": "#2980B9",
    "DESCENT_MAIN":   "#27AE60",
}


def plot_grid(sweeps: list[dict], out: Path):
    n = len(sweeps)
    cols = 2
    rows = (n + 1) // 2
    fig, axes = plt.subplots(rows, cols, figsize=(15, 4.0 * rows),
                              sharey=False)
    if rows == 1:
        axes = np.atleast_2d(axes)

    for ax, sw in zip(axes.flat, sweeps):
        ts = np.array([p.t_loss_s for p in sw["points"]])
        errs = np.array([p.err_m for p in sw["points"]])
        phases = [p.phase for p in sw["points"]]
        gnss = np.array([p.used_gnss for p in sw["points"]])

        # Scatter colored by phase; circle markers for EKF-only, X for GNSS-used
        for phase, color in _PHASE_COLOR.items():
            m = np.array([(ph == phase) and not g for ph, g in zip(phases, gnss)])
            if m.any():
                ax.plot(ts[m], errs[m], "o", color=color, ms=4, alpha=0.7,
                        label=f"{phase} (EKF)")
            m_g = np.array([(ph == phase) and g for ph, g in zip(phases, gnss)])
            if m_g.any():
                ax.plot(ts[m_g], errs[m_g], "s", color=color, ms=5, alpha=0.9,
                        markeredgecolor="black", markeredgewidth=0.5,
                        label=f"{phase} (GNSS)")

        if sw["apogee_s"]:
            ax.axvline(sw["apogee_s"], color="blue", ls=":", lw=0.8, alpha=0.6,
                       label="apogee")
        if sw["boost_end_s"]:
            ax.axvspan(0, sw["boost_end_s"], color="red", alpha=0.08)

        ax.set_yscale("log")
        ax.set_ylim(1, 2000)
        ax.set_xlabel("T_loss (s since first IMU sample)")
        ax.set_ylabel("Landing-prediction error (m)")
        ax.set_title(f"{sw['name']}  (apogee={sw['apogee_s']:.1f}s, "
                     f"end={sw['ekf_end_s']:.1f}s)")
        ax.grid(True, which="both", alpha=0.3)
        # Compact legend
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(handles, labels, fontsize=7, loc="upper right")

        # Reference lines
        ax.axhline(100, color="gray", lw=0.5, ls="--", alpha=0.4)
        ax.axhline(50, color="gray", lw=0.5, ls="--", alpha=0.4)

    # Hide any unused subplot
    for ax in axes.flat[n:]:
        ax.set_visible(False)

    fig.suptitle("Landing-prediction error vs T_loss — 2026-05-17 flights\n"
                 "(red = boost shaded; circles = EKF position; squares = GNSS-for-descent)",
                 fontsize=12, fontweight="bold")
    plt.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160, bbox_inches="tight")
    print(f"Saved {out}")
    return fig


def write_csv(sweeps: list[dict], out: Path):
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["flight", "t_loss_s", "phase", "used_gnss",
                    "snap_e", "snap_n", "snap_u", "snap_vu",
                    "pred_e", "pred_n", "actual_e", "actual_n", "err_m"])
        for sw in sweeps:
            act_e, act_n, _ = sw["actual"]
            for p in sw["points"]:
                w.writerow([sw["name"], f"{p.t_loss_s:.2f}", p.phase,
                            int(p.used_gnss),
                            f"{p.snap_e:.1f}", f"{p.snap_n:.1f}", f"{p.snap_u:.1f}",
                            f"{p.snap_vu:.2f}",
                            f"{p.pred_e:.1f}", f"{p.pred_n:.1f}",
                            f"{act_e:.1f}", f"{act_n:.1f}", f"{p.err_m:.1f}"])
    print(f"Saved {out}")


def print_aggregate(sweeps: list[dict]):
    print("\nAggregate summary (median / p90 error per phase, across all flights)")
    print("=" * 70)
    by_phase: dict[str, list[float]] = {}
    by_phase_gnss: dict[str, list[float]] = {}
    for sw in sweeps:
        for p in sw["points"]:
            key = p.phase
            (by_phase_gnss if p.used_gnss else by_phase).setdefault(key, []).append(p.err_m)

    def stat(arr):
        a = np.array(arr)
        return f"n={len(a):4d}  med={np.median(a):6.1f}m  p90={np.percentile(a,90):7.1f}m  max={a.max():7.1f}m"

    print(f"  {'phase':<16}  {'source':<10}  {'stats'}")
    for phase in ["ASCENT", "DESCENT_DROGUE", "DESCENT_MAIN"]:
        if phase in by_phase:
            print(f"  {phase:<16}  {'EKF-only':<10}  {stat(by_phase[phase])}")
        if phase in by_phase_gnss:
            print(f"  {phase:<16}  {'GNSS-pos':<10}  {stat(by_phase_gnss[phase])}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("flights_dir", type=Path, nargs="?",
                   default=Path(DEFAULT_FLIGHTS_DIR))
    p.add_argument("--step", type=float, default=0.5,
                   help="T_loss step (s)")
    p.add_argument("--utc-offset-h", type=float, default=-4.0)
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    flight_dirs = sorted([d for d in args.flights_dir.iterdir() if d.is_dir()])
    sweeps = []
    for fd in flight_dirs:
        bins = sorted(fd.glob("flight_*.bin"))
        if not bins:
            print(f"  (skip {fd.name}: no flight binary)")
            continue
        print(f"Sweeping {fd.name}...")
        sw = sweep_flight(bins[0], args.step, args.utc_offset_h)
        sweeps.append(sw)
        print(f"  {len(sw['points'])} sample points, "
              f"min err={min(p.err_m for p in sw['points']):.1f}m")

    if not sweeps:
        print("No flights to plot")
        return 1

    plot_grid(sweeps,
              _HERE.parent / "plots" / "landing_pred_sweep_2026_05_17.png")
    write_csv(sweeps, _HERE / "test_data" / "landing_pred_sweep.csv")
    print_aggregate(sweeps)

    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
