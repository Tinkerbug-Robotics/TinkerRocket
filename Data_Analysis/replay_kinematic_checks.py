#!/usr/bin/env python3
"""Replay a binary flight log through the Python port of TR_KinematicChecks.

Drives ``sim_kinematic_checks.KinematicChecks`` with sensor + EKF data
read from a .bin file and reports when each detection flag fires vs when
the firmware (logged in NonSensor) fired it.

Usage:
    python replay_kinematic_checks.py <binary> [<binary>...] [--plot DIR]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from plot_flight_data_mini import parse_binary_file
from sim_kinematic_checks import KinematicChecks, G_MS2


NSF_BURNOUT = 1 << 4

BARO_MACH_LOCKOUT_ON  = 260.0
BARO_MACH_LOCKOUT_OFF = 240.0

LOW_G_FS_G = 16.0
LOW_G_SAT_THRESH_MS2 = (LOW_G_FS_G - 0.5) * G_MS2


def pressure_to_altitude_firmware(p_pa: float, p_ground: float) -> float:
    return 44330.0 * (1.0 - (p_pa / p_ground) ** (1.0 / 5.255))


def accel_norm_firmware(low_xyz, high_xyz) -> float:
    ax_l, ay_l, az_l = low_xyz
    near_sat = (abs(ax_l) > LOW_G_SAT_THRESH_MS2
                or abs(ay_l) > LOW_G_SAT_THRESH_MS2
                or abs(az_l) > LOW_G_SAT_THRESH_MS2)
    ax, ay, az = high_xyz if near_sat else low_xyz
    return math.sqrt(ax*ax + ay*ay + az*az)


def estimate_ground_pressure(baro_recs, nonsensor_recs) -> float:
    if not baro_recs or not nonsensor_recs:
        return 101325.0

    launch_us = None
    for r in nonsensor_recs:
        if r["launch"]:
            launch_us = r["time_us"]
            break

    cutoff_us = launch_us if launch_us is not None else baro_recs[-1]["time_us"]
    pre = [r["pressure_pa"] for r in baro_recs if r["time_us"] < cutoff_us]
    if not pre:
        pre = [r["pressure_pa"] for r in baro_recs[:50]]
    return sum(pre) / len(pre)


def build_events(records):
    events = []
    for r in records["ISM6HG256"]: events.append((r["time_us"], "imu",  r))
    for r in records["BMP585"]:    events.append((r["time_us"], "baro", r))
    for r in records["GNSS"]:      events.append((r["time_us"], "gnss", r))
    for r in records["NonSensor"]: events.append((r["time_us"], "ns",   r))
    events.sort(key=lambda e: (e[0], 0 if e[1] != "imu" else 1))
    return events


def logged_flag_times(nonsensor_recs, t0_us):
    out = {}
    for r in nonsensor_recs:
        flags_byte = r["flags"]
        burnout = bool(flags_byte & NSF_BURNOUT)
        for name, val in (
            ("launch",       r["launch"]),
            ("burnout",      burnout),
            ("vel_apogee",   r["vel_apogee"]),
            ("alt_apogee",   r["alt_apogee"]),
            ("gps_apogee",   r["gps_apogee"]),
            ("pitch_apogee", r["pitch_apogee"]),
            ("apogee_flag",  r["apogee_flag"]),
            ("alt_landed",   r["alt_landed"]),
        ):
            if val and name not in out:
                out[name] = (r["time_us"] - t0_us) / 1e6
    return out


def replay(binary_path: str) -> dict:
    records, stats, _config = parse_binary_file(binary_path)

    if not records["ISM6HG256"]:
        print(f"  {binary_path}: no IMU samples — skipping")
        return {}

    t0_us = records["ISM6HG256"][0]["time_us"]
    ground_pa = estimate_ground_pressure(records["BMP585"], records["NonSensor"])

    print(f"\n=== {Path(binary_path).name} ===")
    print(f"  IMU={len(records['ISM6HG256']):,}  Baro={len(records['BMP585']):,}  "
          f"GNSS={len(records['GNSS']):,}  NS={len(records['NonSensor']):,}")
    print(f"  ground pressure (pre-launch avg): {ground_pa:.1f} Pa")

    kc = KinematicChecks()

    latest_palt: float = 0.0
    latest_acc_mag: float = 0.0
    latest_pos = (0.0, 0.0, 0.0)
    latest_vel = (0.0, 0.0, 0.0)
    latest_roll_rate: float = 0.0
    latest_gps_alt: float = 0.0
    latest_gps_vel_u: float = 0.0
    latest_pitch_rad: float = math.pi / 2
    burnout: bool = False
    mach_locked_out: bool = False

    new_baro = False
    new_gps = False

    sim_fires: dict[str, float] = {}
    # Time-series capture for plotting.
    raw_palt_t, raw_palt_v = [], []
    sim_alt_t, sim_alt_v = [], []
    baro_reject_t: list[float] = []
    # Per-tick pass states (used to draw "condition true" bands).
    pass_t: list[float] = []
    pass_vel: list[bool] = []
    pass_baro: list[bool] = []
    pass_pitch: list[bool] = []
    gps_pass_t: list[float] = []
    gps_pass_v: list[bool] = []

    def _snap_edges(now_s: float, prev_rejects: int):
        for name, val in (
            ("launch_flag",       kc.launch_flag),
            ("vel_u_apogee_flag", kc.vel_u_apogee_flag),
            ("alt_apogee_flag",   kc.alt_apogee_flag),
            ("gps_apogee_flag",   kc.gps_apogee_flag),
            ("pitch_apogee_flag", kc.pitch_apogee_flag),
            ("apogee_flag",       kc.apogee_flag),
            ("alt_landed_flag",   kc.alt_landed_flag),
        ):
            if val and name not in sim_fires:
                sim_fires[name] = now_s
        if kc._consec_baro_rejects > prev_rejects:
            baro_reject_t.append(now_s)

    for t_us, kind, r in build_events(records):
        now_ms = (t_us - t0_us) // 1000

        if kind == "baro":
            latest_palt = pressure_to_altitude_firmware(r["pressure_pa"], ground_pa)
            raw_palt_t.append((t_us - t0_us) / 1e6)
            raw_palt_v.append(latest_palt)
            new_baro = True
            continue

        if kind == "gnss":
            latest_gps_alt = float(r["alt_m"])
            latest_gps_vel_u = float(r["vel_u"])
            new_gps = True
            # Record GPS pass-state at GPS sample time (separate axis since
            # GPS is much slower than IMU and shouldn't be smeared along it).
            gps_pass_t.append((t_us - t0_us) / 1e6)
            continue

        if kind == "ns":
            latest_pos = (r["e_pos"], r["n_pos"], r["u_pos"])
            latest_vel = (r["e_vel"], r["n_vel"], r["u_vel"])
            latest_pitch_rad = math.radians(r["pitch"])
            burnout = bool(r["flags"] & NSF_BURNOUT)
            speed = math.sqrt(r["e_vel"]**2 + r["n_vel"]**2 + r["u_vel"]**2)
            if not mach_locked_out and speed > BARO_MACH_LOCKOUT_ON:
                mach_locked_out = True
            elif mach_locked_out and speed < BARO_MACH_LOCKOUT_OFF:
                mach_locked_out = False
            continue

        # imu — main-loop tick
        latest_low_xyz  = (r["low_acc_x"],  r["low_acc_y"],  r["low_acc_z"])
        latest_high_xyz = (r["high_acc_x"], r["high_acc_y"], r["high_acc_z"])
        latest_acc_mag = accel_norm_firmware(latest_low_xyz, latest_high_xyz)
        latest_roll_rate = r["gyro_x"]

        prev_rejects = kc._consec_baro_rejects

        kc.kinematic_checks(
            pressure_altitude=latest_palt,
            acc_mag=latest_acc_mag,
            position=latest_pos,
            velocity=latest_vel,
            roll_rate=latest_roll_rate,
            new_baro=new_baro,
            gps_altitude=latest_gps_alt,
            gps_vel_u=latest_gps_vel_u,
            new_gps=new_gps,
            pitch_rad=latest_pitch_rad,
            burnout_detected=burnout,
            baro_locked_out=mach_locked_out,
            now_ms=now_ms,
        )
        new_baro = False
        new_gps = False

        now_s = now_ms / 1000.0
        sim_alt_t.append(now_s)
        sim_alt_v.append(kc.alt_est)
        pass_t.append(now_s)
        pass_vel.append(kc.last_vel_pass)
        pass_baro.append(kc.last_baro_pass)
        pass_pitch.append(kc.last_pitch_pass)
        # GPS pass is sticky between samples; record current value at the
        # GPS time we noted above. Aligning lengths:
        if len(gps_pass_t) > len(gps_pass_v):
            gps_pass_v.append(kc.last_gps_pass)
        _snap_edges(now_s, prev_rejects)

    logged = logged_flag_times(records["NonSensor"], t0_us)

    rows = [
        ("launch",       "launch_flag"),
        ("burnout",      None),
        ("vel_apogee",   "vel_u_apogee_flag"),
        ("alt_apogee",   "alt_apogee_flag"),
        ("gps_apogee",   "gps_apogee_flag"),
        ("pitch_apogee", "pitch_apogee_flag"),
        ("apogee_flag",  "apogee_flag"),
        ("alt_landed",   "alt_landed_flag"),
    ]
    print(f"  {'flag':<14} {'logged':>10} {'sim':>10}   Δ (s)")
    print(f"  {'─'*14} {'─'*10} {'─'*10}   {'─'*7}")
    for logged_name, sim_name in rows:
        l = logged.get(logged_name)
        s = sim_fires.get(sim_name) if sim_name else None
        l_str = f"{l:10.3f}" if l is not None else f"{'—':>10}"
        s_str = f"{s:10.3f}" if s is not None else f"{'—':>10}"
        if l is not None and s is not None:
            d_str = f"{s - l:+7.3f}"
        else:
            d_str = "      —"
        print(f"  {logged_name:<14} {l_str} {s_str}   {d_str}")
    print(f"  max_alt (sim): {kc.max_altitude:.1f} m   max_speed (sim): {kc.max_speed:.1f} m/s   "
          f"baro rejects: {len(baro_reject_t)}")

    # Compress per-tick pass arrays into contiguous (t_start, t_end) periods.
    def periods(times, states):
        out = []
        in_run = False
        run_start = 0.0
        prev_t = 0.0
        for t, s in zip(times, states):
            if s and not in_run:
                in_run = True
                run_start = t
            elif not s and in_run:
                in_run = False
                out.append((run_start, prev_t))
            prev_t = t
        if in_run:
            out.append((run_start, prev_t))
        return out

    # Rocket name = parent directory (works for 5/17 layout).
    rocket_label = Path(binary_path).parent.name.strip() or Path(binary_path).stem

    return {
        "name":      rocket_label,
        "filename":  Path(binary_path).name,
        "sim_fires": sim_fires,
        "logged":    logged,
        "raw_palt":  (raw_palt_t, raw_palt_v),
        "sim_alt":   (sim_alt_t, sim_alt_v),
        "baro_reject_t": baro_reject_t,
        "vel_periods":   periods(pass_t, pass_vel),
        "baro_periods":  periods(pass_t, pass_baro),
        "pitch_periods": periods(pass_t, pass_pitch),
        "gps_periods":   periods(gps_pass_t, gps_pass_v),
        "kc":        kc,
    }


def plot_flights(results, out_path: str | None = None, show: bool = False) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    n = len(results)
    cols = 2
    rows = (n + 1) // 2

    # Each "cell" is two stacked panes: altitude on top, condition-true
    # strip below. Use GridSpec with height ratios so the strip is short.
    fig = plt.figure(figsize=(15, 5.5 * rows))
    outer = GridSpec(rows, cols, figure=fig, hspace=0.4, wspace=0.22)

    det_style = {
        "vel":   ("Vel",   "tab:blue"),
        "baro":  ("Baro",  "tab:green"),
        "gps":   ("GPS",   "tab:orange"),
        "pitch": ("Pitch", "tab:purple"),
    }
    fire_marker = {"vel": "s", "baro": "^", "gps": "o", "pitch": "*"}
    flag_for_det = {
        "vel":   "vel_u_apogee_flag",
        "baro":  "alt_apogee_flag",
        "gps":   "gps_apogee_flag",
        "pitch": "pitch_apogee_flag",
    }

    for idx, res in enumerate(results):
        if not res:
            continue
        row, col = idx // cols, idx % cols
        sub = outer[row, col].subgridspec(2, 1, height_ratios=[3, 1], hspace=0.05)
        ax_alt = fig.add_subplot(sub[0])
        ax_pass = fig.add_subplot(sub[1], sharex=ax_alt)

        # ── Altitude pane ───────────────────────────────────────────
        t_raw, v_raw = res["raw_palt"]
        t_sim, v_sim = res["sim_alt"]
        ax_alt.plot(t_raw, v_raw, color="lightgray", lw=0.6, label="raw palt")
        ax_alt.plot(t_sim, v_sim, color="black",     lw=0.9, label="KF alt_est")

        # True apogee = peak of raw palt.
        if v_raw:
            peak_idx = max(range(len(v_raw)), key=lambda i: v_raw[i])
            ax_alt.axvline(t_raw[peak_idx], color="dimgray", ls=":", lw=0.8,
                           label=f"true apogee {t_raw[peak_idx]:.2f}s")

        # Sub-detector first-fire markers (when count crossed HI threshold).
        for det_key, flag_name in flag_for_det.items():
            t = res["sim_fires"].get(flag_name)
            if t is None:
                continue
            i = min(range(len(t_sim)), key=lambda j: abs(t_sim[j] - t))
            y = v_sim[i]
            label, color = det_style[det_key]
            ax_alt.scatter([t], [y], color=color, marker=fire_marker[det_key],
                           s=80, zorder=5, edgecolor="black", linewidth=0.6,
                           label=f"{label} fire {t:.2f}s")

        # Master apogee fire.
        t_master = res["sim_fires"].get("apogee_flag")
        if t_master is not None:
            ax_alt.axvline(t_master, color="red", lw=1.2, alpha=0.7,
                           label=f"master {t_master:.2f}s")

        # Baro rejects at bottom of altitude pane (just inside ylim floor).
        ALT_FLOOR = -50.0
        if res["baro_reject_t"]:
            ax_alt.scatter(res["baro_reject_t"],
                           [ALT_FLOOR + 5] * len(res["baro_reject_t"]),
                           color="red", marker="x", s=14, alpha=0.4,
                           label=f"baro reject ({len(res['baro_reject_t'])})")

        ax_alt.set_title(f"{res['name']}   ({res['filename']})", fontsize=10)
        ax_alt.set_ylabel("altitude (m)")
        ax_alt.set_ylim(bottom=ALT_FLOOR)
        ax_alt.grid(True, alpha=0.3)
        ax_alt.legend(fontsize=7, loc="best", ncol=2)
        plt.setp(ax_alt.get_xticklabels(), visible=False)

        # ── Pass-state pane ─────────────────────────────────────────
        # One row per detector at fixed Y. Draw each contiguous period
        # where the test condition was true as a horizontal bar.
        det_rows = [
            ("vel",   res["vel_periods"]),
            ("baro",  res["baro_periods"]),
            ("gps",   res["gps_periods"]),
            ("pitch", res["pitch_periods"]),
        ]
        for y_idx, (det_key, periods) in enumerate(det_rows):
            label, color = det_style[det_key]
            y = len(det_rows) - 1 - y_idx
            for (t0, t1) in periods:
                ax_pass.hlines(y, t0, max(t1, t0 + 0.001), color=color,
                               lw=8, alpha=0.7)
            ax_pass.text(-0.01, y, label, transform=ax_pass.get_yaxis_transform(),
                         ha="right", va="center", fontsize=8, color=color)

        ax_pass.set_yticks([])
        ax_pass.set_ylim(-0.7, len(det_rows) - 0.3)
        ax_pass.set_xlabel("t (s)")
        ax_pass.set_ylabel("condition\ntrue", fontsize=8)
        ax_pass.grid(True, axis="x", alpha=0.3)

        # Shared X range trimmed to action window.
        if t_raw:
            t_end = (res["sim_fires"].get("apogee_flag") or t_raw[-1]) + 3.0
            ax_alt.set_xlim(-0.5, min(t_end, t_raw[-1]))

    fig.suptitle("Apogee sub-detector firings & pass-conditions — sim_kinematic_checks",
                 fontsize=12, y=0.995)
    fig.tight_layout()
    if out_path:
        fig.savefig(out_path, dpi=140)
        print(f"\nPlot saved to: {out_path}")
    if show:
        plt.show()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("binary", nargs="+", help="One or more flight .bin files")
    ap.add_argument("--plot", help="Save per-flight plot to this path")
    ap.add_argument("--show", action="store_true",
                    help="Open interactive matplotlib window (zoom/pan)")
    args = ap.parse_args()

    results = [replay(p) for p in args.binary]

    if args.plot or args.show:
        plot_flights(results, args.plot, args.show)


if __name__ == "__main__":
    main()
