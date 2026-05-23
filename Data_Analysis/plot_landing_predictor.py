#!/usr/bin/env python3
"""Visualize landing-predictor output for a flight.

Plots actual trajectory (EKF + GNSS) and overlays predicted descent
tracks from several T_loss snapshots, so the user can see by eye how
each prediction compares to truth.

Usage:
    python plot_landing_predictor.py <flight_dir> --t-loss 5 15 36.8 46
"""

import sys
import math
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "tinkerrocket-sim" / "src"))

from plot_flight_data_mini import parse_binary_file
from _ekf_replay import replay_binary, lla_rad_to_enu_m, DEG2RAD
from _wind_profile import fetch_wind
from landing_predictor import (snapshot_at, predict_landing, actual_landing_enu,
                                RocketProfile, _infer_flight_datetime_utc,
                                _find_lora_csv, _find_binary)


def collect_trajectories(bin_path: Path, t_loss_list: list[float],
                          utc_offset_h: float):
    records, _, _ = parse_binary_file(str(bin_path))
    res = replay_binary(records, verbose=False)
    t0 = res.phases.t0_us

    # EKF trajectory in ENU
    ekf_e, ekf_n, ekf_u = [], [], []
    for i in range(len(res.ekf.t_us)):
        e, n, u = lla_rad_to_enu_m(res.ekf.lat_rad[i], res.ekf.lon_rad[i],
                                    res.ekf.alt_m[i], res.launch_ref)
        ekf_e.append(e); ekf_n.append(n); ekf_u.append(u)
    ekf_e = np.array(ekf_e); ekf_n = np.array(ekf_n); ekf_u = np.array(ekf_u)
    ekf_t_s = (res.ekf.t_us - t0) / 1e6

    # GNSS truth in ENU
    g_e, g_n, g_u = [], [], []
    for i in range(len(res.gnss.t_us)):
        e, n, u = lla_rad_to_enu_m(res.gnss.lat_deg[i] * DEG2RAD,
                                    res.gnss.lon_deg[i] * DEG2RAD,
                                    res.gnss.alt_m[i], res.launch_ref)
        g_e.append(e); g_n.append(n); g_u.append(u)
    g_e = np.array(g_e); g_n = np.array(g_n); g_u = np.array(g_u)
    g_t_s = (res.gnss.t_us - t0) / 1e6

    # LoRa CSV trajectory (extends past binary end)
    lora_csv = _find_lora_csv(bin_path)
    lora_e = lora_n = lora_u = lora_t = None
    if lora_csv:
        import csv
        es, ns, us, ts = [], [], [], []
        with open(lora_csv) as f:
            for r in csv.DictReader(f):
                try:
                    lat = float(r["lat"]); lon = float(r["lon"])
                    alt = float(r["alt_m"]); nsat = float(r["num_sats"])
                except (ValueError, KeyError):
                    continue
                if nsat < 4 or not math.isfinite(lat):
                    continue
                e, n, u = lla_rad_to_enu_m(lat * DEG2RAD, lon * DEG2RAD,
                                            alt, res.launch_ref)
                es.append(e); ns.append(n); us.append(u)
                ts.append(float(r["time_ms"]) / 1000.0)
        lora_e = np.array(es); lora_n = np.array(ns); lora_u = np.array(us)
        lora_t = np.array(ts)

    # Wind for predictions
    dt_utc = _infer_flight_datetime_utc(bin_path, utc_offset_h)
    wind = None
    if dt_utc is not None:
        try:
            wind = fetch_wind(res.launch_ref.lat_deg, res.launch_ref.lon_deg,
                              dt_utc, cache_dir=_HERE / "test_data" / "wind_cache")
        except Exception as e:
            print(f"  (wind fetch failed: {e})")

    # Predictions at each requested T_loss
    profile = RocketProfile()
    preds = []
    for t_loss in t_loss_list:
        snap = snapshot_at(res, t_loss)
        pred = predict_landing(snap, profile, wind=wind)
        preds.append((t_loss, snap, pred))

    actual = actual_landing_enu(res, lora_csv)

    return dict(
        ekf=(ekf_t_s, ekf_e, ekf_n, ekf_u),
        gnss=(g_t_s, g_e, g_n, g_u),
        lora=(lora_t, lora_e, lora_n, lora_u) if lora_t is not None else None,
        wind=wind,
        preds=preds,
        actual=actual,
        phases=res.phases,
        t0_us=t0,
        launch_ref=res.launch_ref,
    )


def plot_flight(data, title: str, out_path: Path):
    fig = plt.figure(figsize=(15, 10))
    gs = fig.add_gridspec(2, 2, height_ratios=[2, 1], width_ratios=[2, 1])
    ax_xy = fig.add_subplot(gs[0, 0])
    ax_z = fig.add_subplot(gs[0, 1])
    ax_err = fig.add_subplot(gs[1, :])

    ekf_t, ekf_e, ekf_n, ekf_u = data["ekf"]
    g_t, g_e, g_n, g_u = data["gnss"]
    lora = data["lora"]
    preds = data["preds"]
    act_e, act_n, act_u, act_t = data["actual"]

    # ---- Top-down (E vs N) ----
    # GNSS truth (most trustworthy when fix valid)
    ax_xy.scatter(g_e, g_n, c=g_t, cmap="viridis", s=8, alpha=0.6,
                  label="GNSS fixes (color = t)")
    # EKF trajectory in light gray for context
    ax_xy.plot(ekf_e, ekf_n, "-", color="gray", lw=0.8, alpha=0.5,
               label="EKF trajectory")
    if lora is not None:
        _, le, ln, _ = lora
        ax_xy.plot(le, ln, ".", ms=2, color="C2", alpha=0.4,
                   label=f"LoRa downlink ({len(le)} pts)")

    # Launch + actual landing
    ax_xy.scatter([0], [0], marker="^", s=200, c="black",
                  edgecolors="white", linewidths=1.5, zorder=10, label="Launch")
    ax_xy.scatter([act_e], [act_n], marker="*", s=320, c="red",
                  edgecolors="white", linewidths=1.5, zorder=10,
                  label=f"Actual landing (t={act_t:.0f}s)")

    # Each prediction: snapshot point + descent track + predicted landing
    colors = plt.cm.cool(np.linspace(0.1, 0.9, len(preds)))
    for (t_loss, snap, pred), color in zip(preds, colors):
        # Snapshot point (where the EKF was at T_loss)
        ax_xy.scatter([snap.e_m], [snap.n_m], marker="o", s=80, color=color,
                      edgecolors="black", linewidths=0.8, zorder=8,
                      label=f"T_loss={t_loss:.1f}s  {pred.phase}")
        # Predicted descent track
        if pred.descent_track:
            te = [p[0] for p in pred.descent_track]
            tn = [p[1] for p in pred.descent_track]
            ax_xy.plot(te, tn, "--", color=color, lw=1.4, alpha=0.8)
        # Predicted apogee (if ascent)
        if pred.apogee_e_m is not None:
            ax_xy.scatter([pred.apogee_e_m], [pred.apogee_n_m],
                          marker="P", s=80, color=color,
                          edgecolors="black", linewidths=0.6, zorder=7)
        # Predicted landing
        ax_xy.scatter([pred.landing_e_m], [pred.landing_n_m], marker="X",
                      s=160, color=color, edgecolors="black", linewidths=0.8,
                      zorder=9)

    ax_xy.set_xlabel("East (m)")
    ax_xy.set_ylabel("North (m)")
    ax_xy.set_title("Top-down: dashed = predicted descent, X = predicted landing")
    ax_xy.legend(loc="best", fontsize=8, framealpha=0.85)
    ax_xy.grid(True, alpha=0.3)
    ax_xy.set_aspect("equal")

    # ---- Altitude vs time ----
    ax_z.plot(ekf_t, ekf_u, "-", color="C3", lw=1, alpha=0.7, label="EKF U")
    ax_z.scatter(g_t, g_u, c=g_t, cmap="viridis", s=8, alpha=0.6)
    if lora is not None:
        lt, _, _, lu = lora
        ax_z.plot(lt, lu, ".", ms=2, color="C2", alpha=0.4)

    for (t_loss, snap, pred), color in zip(preds, colors):
        if pred.descent_track:
            tt = [p[3] for p in pred.descent_track]
            tu = [p[2] for p in pred.descent_track]
            ax_z.plot(tt, tu, "--", color=color, lw=1.4, alpha=0.8)
        ax_z.scatter([snap.t_s], [snap.u_m], marker="o", s=60, color=color,
                     edgecolors="black", linewidths=0.6, zorder=7)
    ax_z.scatter([act_t], [act_u], marker="*", s=200, c="red",
                 edgecolors="white", linewidths=1, zorder=10)
    ax_z.set_xlabel("Time (s)")
    ax_z.set_ylabel("Altitude U (m)")
    ax_z.set_title("Altitude vs time")
    ax_z.grid(True, alpha=0.3)

    # ---- Error vs T_loss (text summary) ----
    ax_err.axis("off")
    lines = [f"{title}", ""]
    lines.append(f"Actual landing: E={act_e:+.1f}  N={act_n:+.1f}  U={act_u:+.1f}  (t={act_t:.0f}s)")
    if data["wind"]:
        w = data["wind"]
        sfc = w.layers[0] if w.layers else None
        if sfc:
            lines.append(f"Wind (surface): {sfc.speed_kts:.1f} kts from {sfc.dir_from_deg:.0f}°  "
                          f"(source: {w.source})")
    lines.append("")
    lines.append(f"{'T_loss':>8} {'phase':<16} {'EKF pos at loss':<28} "
                 f"{'predicted landing':<22} {'error':>8}")
    for (t_loss, snap, pred) in preds:
        err = math.hypot(pred.landing_e_m - act_e, pred.landing_n_m - act_n)
        snap_str = f"E={snap.e_m:+6.0f} N={snap.n_m:+6.0f} U={snap.u_m:+5.0f}"
        pred_str = f"E={pred.landing_e_m:+6.0f} N={pred.landing_n_m:+6.0f}"
        lines.append(f"{t_loss:>7.1f}s {pred.phase:<16} {snap_str:<28} "
                     f"{pred_str:<22} {err:>7.1f}m")
    ax_err.text(0.01, 0.95, "\n".join(lines), family="monospace",
                fontsize=9, va="top", ha="left",
                transform=ax_err.transAxes)

    fig.suptitle(title, fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(out_path, dpi=160, bbox_inches="tight")
    print(f"  Saved: {out_path}")
    return fig


def main():
    p = argparse.ArgumentParser()
    p.add_argument("flight_path", type=Path)
    p.add_argument("--t-loss", type=float, nargs="+", required=True,
                   help="One or more T_loss times (s) to overlay")
    p.add_argument("--utc-offset-h", type=float, default=-4.0)
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--title", type=str, default=None)
    p.add_argument("--no-show", action="store_true",
                   help="Save only, don't open interactive window")
    args = p.parse_args()

    bin_path = _find_binary(args.flight_path)
    title = args.title or bin_path.parent.name
    out = args.out or (_HERE.parent / "plots" /
                       f"landing_pred_{bin_path.parent.name.strip().replace(' ', '_')}.png")
    out.parent.mkdir(parents=True, exist_ok=True)

    print(f"Plotting {bin_path}")
    data = collect_trajectories(bin_path, args.t_loss, args.utc_offset_h)
    plot_flight(data, title, out)
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
