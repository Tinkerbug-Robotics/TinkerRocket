#!/usr/bin/env python3
"""Guidance-flight report from a TinkerRocket .bin flight log.

Decodes GUIDANCE_TELEM_MSG (0xCA) + NonSensor frames and produces a
guidance-focused report: control output (pitch/yaw fin commands, roll command)
and PN guidance parameters (LOS angle, closing velocity, lateral offset, accel
commands), aligned to flight time (T=0 at launch). Loads the matching .json
sidecar for apogee/burnout/config context when present.

Usage:
    python3 report_guidance_flight.py <flight.bin> [--out report.png]
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_flight_data_mini as P  # noqa: E402


def _arr(records, key):
    return np.array([r[key] for r in records], dtype=float)


def load_flight(bin_path):
    recs, stats, _ = P.parse_binary_file(bin_path)
    ns = recs["NonSensor"]
    g = recs["Guidance"]
    if not ns:
        raise SystemExit("No NonSensor frames — cannot establish launch time.")

    launched = [r for r in ns if r["launch"]]
    t_launch = (launched[0]["time_us"] if launched else ns[0]["time_us"]) / 1e6

    # Optional .json sidecar (apogee/burnout/config)
    side = os.path.splitext(bin_path)[0] + ".json"
    meta = {}
    if os.path.exists(side):
        with open(side) as f:
            meta = json.load(f)
    return recs, stats, t_launch, meta


def summarize(recs, stats, t_launch, meta):
    ns, g = recs["NonSensor"], recs["Guidance"]
    print("=" * 64)
    print("GUIDANCE FLIGHT REPORT")
    print("=" * 64)
    print(f"Frames: {stats['good_crc']} good CRC, {stats['bad_crc']} bad")
    if meta:
        settings = meta.get("settings", {})
        rc = settings.get("roll_control", {})
        print(f"FW: {settings.get('fw_git_sha','?')}"
              f"{' (dirty)' if settings.get('fw_dirty') else ''} | "
              f"guidance_enabled={rc.get('guidance_enabled')} | "
              f"activation delay_ms={rc.get('delay_ms')}")
        print(f"Apogee: {meta.get('max_altitude_m',0):.1f} m @ T+"
              f"{meta.get('apogee_time_s',0):.2f} s | "
              f"max speed {meta.get('max_speed_mps',0):.1f} m/s | "
              f"burnout T+{meta.get('burnout_time_s',0):.2f} s")

    if not g:
        print("\nNO guidance frames in this log.")
        return
    tg = _arr(g, "time_us") / 1e6 - t_launch
    print(f"\nGuidance active: T+{tg.min():.2f} .. T+{tg.max():.2f} s "
          f"({tg.max()-tg.min():.2f} s, {len(g)} frames @ ~10 Hz)")
    if meta:
        d = meta.get("settings", {}).get("roll_control", {}).get("delay_ms")
        if d is not None:
            print(f"  -> engaged {tg.min()*1000:.0f} ms after launch "
                  f"(activation delay = {d} ms; burnout at T+"
                  f"{meta.get('burnout_time_s',0):.2f} s) "
                  f"=> {'DURING boost' if tg.min() < meta.get('burnout_time_s',1e9) else 'post-burnout'}")
    pf, yf = _arr(g, "pitch_fin_cmd"), _arr(g, "yaw_fin_cmd")
    print(f"  pitch fin cmd: {pf.min():+.1f} .. {pf.max():+.1f} deg")
    print(f"  yaw   fin cmd: {yf.min():+.1f} .. {yf.max():+.1f} deg")
    print(f"  LOS angle:     {_arr(g,'los_angle').min():.2f} .. "
          f"{_arr(g,'los_angle').max():.2f} deg")
    print(f"  closing vel:   {_arr(g,'closing_vel').min():.1f} .. "
          f"{_arr(g,'closing_vel').max():.1f} m/s")
    print(f"  lateral offset:{_arr(g,'lateral_offset').min():.2f} .. "
          f"{_arr(g,'lateral_offset').max():.2f} m")
    amag = np.hypot(_arr(g, "accel_cmd_n"), _arr(g, "accel_cmd_e"))
    print(f"  PN |accel cmd|:{amag.min():.1f} .. {amag.max():.1f} m/s^2")


def make_report(recs, t_launch, meta, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ns, g = recs["NonSensor"], recs["Guidance"]
    t_ns = _arr(ns, "time_us") / 1e6 - t_launch
    alt = _arr(ns, "u_pos")
    spd = np.sqrt(_arr(ns, "e_vel")**2 + _arr(ns, "n_vel")**2 + _arr(ns, "u_vel")**2)
    roll_cmd = _arr(ns, "roll_cmd")
    roll = _arr(ns, "roll"); pitch = _arr(ns, "pitch"); yaw = _arr(ns, "yaw")

    has_g = bool(g)
    if has_g:
        tg = _arr(g, "time_us") / 1e6 - t_launch
        g0, g1 = tg.min(), tg.max()
    t_apo = meta.get("apogee_time_s")
    t_burn = meta.get("burnout_time_s")

    def shade(ax):
        if has_g:
            ax.axvspan(g0, g1, color="tab:green", alpha=0.10,
                       label="guidance active")
        if t_burn:
            ax.axvline(t_burn, color="gray", ls=":", lw=1)
        if t_apo:
            ax.axvline(t_apo, color="k", ls="--", lw=0.8)

    # Limit the x-window to the interesting part (launch .. shortly past apogee)
    x_hi = (t_apo + 2) if t_apo else t_ns.max()
    m = (t_ns >= -0.5) & (t_ns <= x_hi)

    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    title = "Guidance Flight Report"
    if meta:
        title += (f"  —  apogee {meta.get('max_altitude_m',0):.0f} m, "
                  f"act.delay {meta.get('settings',{}).get('roll_control',{}).get('delay_ms','?')} ms")
    fig.suptitle(title, fontsize=14, fontweight="bold")

    # (0,0) Flight context: altitude + speed
    ax = axes[0, 0]; shade(ax)
    ax.plot(t_ns[m], alt[m], color="tab:blue", label="Altitude (m)")
    ax.set_ylabel("Altitude (m)", color="tab:blue"); ax.set_xlabel("T+ (s)")
    axb = ax.twinx()
    axb.plot(t_ns[m], spd[m], color="tab:red", alpha=0.6, label="Speed (m/s)")
    axb.set_ylabel("Speed (m/s)", color="tab:red")
    ax.set_title("Flight context (burnout ·, apogee --)")
    ax.grid(True, alpha=0.3)

    # (0,1) CONTROL OUTPUT: commanded fin deflections
    ax = axes[0, 1]; shade(ax)
    ax.plot(t_ns[m], roll_cmd[m], color="tab:orange", lw=1, alpha=0.8,
            label="roll cmd (NonSensor)")
    if has_g:
        ax.plot(tg, _arr(g, "pitch_fin_cmd"), "o-", ms=3, color="tab:red",
                label="pitch fin cmd")
        ax.plot(tg, _arr(g, "yaw_fin_cmd"), "o-", ms=3, color="tab:green",
                label="yaw fin cmd")
    ax.set_ylabel("Commanded deflection (deg)"); ax.set_xlabel("T+ (s)")
    ax.set_title("Control output — fin commands (pre-mix)")
    ax.legend(loc="upper left", fontsize=8); ax.grid(True, alpha=0.3)

    # (1,0) Guidance geometry: LOS angle + lateral offset
    ax = axes[1, 0]; shade(ax)
    if has_g:
        ax.plot(tg, _arr(g, "los_angle"), "o-", ms=3, color="tab:blue",
                label="LOS angle (deg)")
        axb = ax.twinx()
        axb.plot(tg, _arr(g, "lateral_offset"), "s-", ms=3, color="tab:purple",
                 label="lateral offset (m)")
        axb.set_ylabel("Lateral offset (m)", color="tab:purple")
    ax.set_ylabel("LOS angle (deg)", color="tab:blue"); ax.set_xlabel("T+ (s)")
    ax.set_title("Guidance geometry"); ax.grid(True, alpha=0.3)

    # (1,1) Closing velocity + PN accel command magnitude (with saturation ref)
    ax = axes[1, 1]; shade(ax)
    if has_g:
        amag = np.hypot(_arr(g, "accel_cmd_n"), _arr(g, "accel_cmd_e"))
        ax.plot(tg, _arr(g, "closing_vel"), "o-", ms=3, color="tab:gray",
                label="closing vel (m/s)")
        axb = ax.twinx()
        axb.plot(tg, amag, "o-", ms=3, color="tab:red",
                 label="|PN accel cmd| (m/s²)")
        axb.set_ylabel("|PN accel cmd| (m/s²)", color="tab:red")
    ax.set_ylabel("Closing velocity (m/s)", color="tab:gray"); ax.set_xlabel("T+ (s)")
    ax.set_title("Closing velocity & PN accel command"); ax.grid(True, alpha=0.3)

    # (2,0) Attitude
    ax = axes[2, 0]; shade(ax)
    ax.plot(t_ns[m], roll[m], label="roll", alpha=0.8)
    ax.plot(t_ns[m], pitch[m], label="pitch", alpha=0.8)
    ax.plot(t_ns[m], yaw[m], label="yaw", alpha=0.8)
    ax.set_ylabel("Euler angle (deg)"); ax.set_xlabel("T+ (s)")
    ax.set_title("Attitude (NonSensor quaternion → Euler)")
    ax.legend(loc="upper left", fontsize=8); ax.grid(True, alpha=0.3)

    # (2,1) Fin commands zoomed to the guidance window
    ax = axes[2, 1]
    if has_g:
        if t_burn:
            ax.axvline(t_burn, color="gray", ls=":", lw=1, label="burnout")
        ax.plot(tg, _arr(g, "pitch_fin_cmd"), "o-", ms=4, color="tab:red",
                label="pitch fin cmd")
        ax.plot(tg, _arr(g, "yaw_fin_cmd"), "o-", ms=4, color="tab:green",
                label="yaw fin cmd")
        ax.set_xlim(g0 - 0.1, g1 + 0.1)
    ax.set_ylabel("Commanded deflection (deg)"); ax.set_xlabel("T+ (s)")
    ax.set_title("Fin commands — guidance window (zoom)")
    ax.legend(loc="upper left", fontsize=8); ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    print(f"\nReport saved to {out_path}")


def main():
    ap = argparse.ArgumentParser(description="Guidance flight report from a .bin log")
    ap.add_argument("bin", help="path to flight .bin")
    ap.add_argument("--out", default=None, help="output PNG path")
    args = ap.parse_args()
    out = args.out or (os.path.splitext(args.bin)[0] + "_guidance_report.png")

    recs, stats, t_launch, meta = load_flight(args.bin)
    summarize(recs, stats, t_launch, meta)
    make_report(recs, t_launch, meta, out)


if __name__ == "__main__":
    main()
