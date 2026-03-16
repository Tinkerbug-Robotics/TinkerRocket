#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_gaps.py

Analyzes binary flight data files for temporal gaps and missing data,
with a focus on the start-of-flight region.

Uses the existing parse_binary_file() from plot_flight_data_mini.py.
"""

import sys
import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from collections import defaultdict

# Add the script directory to path so we can import the parser
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from plot_flight_data_mini import parse_binary_file, ROCKET_STATES, NSF_LAUNCH

# ── Configuration ──────────────────────────────────────────────────────────────

FLIGHTS = [
    {
        "name": "Flight 1 - E24-4",
        "bin_path": "/Users/christianpedersen/Documents/Hobbies/Model Rockets/TestFlights/2026_03_14/Raw Downloads/Flight 1 - E24-4/Raw Data/flight_20260314_195900.bin",
        "output_dir": "/Users/christianpedersen/Documents/Hobbies/Model Rockets/TestFlights/2026_03_14/Raw Downloads/Flight 1 - E24-4/Analysis/",
    },
    {
        "name": "Flight 2 - F67-6",
        "bin_path": "/Users/christianpedersen/Documents/Hobbies/Model Rockets/TestFlights/2026_03_14/Raw Downloads/Flight 2 - F67-6/Raw Data/flight_20260314_224121.bin",
        "output_dir": "/Users/christianpedersen/Documents/Hobbies/Model Rockets/TestFlights/2026_03_14/Raw Downloads/Flight 2 - F67-6/Analysis/",
    },
]

# A gap is flagged when the delta between consecutive records exceeds
# GAP_MULTIPLIER times the median delta for that flight.
GAP_MULTIPLIER = 5.0

# Minimum absolute gap duration to report (microseconds). Filters out
# tiny noise even if it exceeds the multiplier.
MIN_GAP_US = 5000  # 5 ms

FIGURE_DPI = 150


# ── Helpers ────────────────────────────────────────────────────────────────────

def extract_all_timestamps(records):
    """
    From the records dict returned by parse_binary_file(), extract every
    timestamp along with its record type.

    Returns:
        List of (time_us, record_type_name) tuples, sorted by time_us.
    """
    all_ts = []
    for sensor_name, rec_list in records.items():
        for rec in rec_list:
            if "time_us" in rec:
                all_ts.append((rec["time_us"], sensor_name))
    all_ts.sort(key=lambda x: x[0])
    return all_ts


def extract_per_sensor_timestamps(records):
    """
    Returns dict: sensor_name -> sorted numpy array of time_us values.
    """
    per_sensor = {}
    for sensor_name, rec_list in records.items():
        times = [rec["time_us"] for rec in rec_list if "time_us" in rec]
        if times:
            per_sensor[sensor_name] = np.array(sorted(times), dtype=np.float64)
    return per_sensor


def find_launch_time(records):
    """
    Find the first NonSensor record where the 'launch' flag is True.
    Returns time_us or None.
    """
    for rec in records.get("NonSensor", []):
        if rec.get("launch", False):
            return rec["time_us"]
    return None


def find_gaps(timestamps_with_types, median_delta_us, gap_multiplier, min_gap_us):
    """
    Identify gaps in a sorted list of (time_us, type) tuples.

    Returns list of dicts with gap info.
    """
    gaps = []
    threshold = max(median_delta_us * gap_multiplier, min_gap_us)

    for i in range(1, len(timestamps_with_types)):
        t_prev, type_prev = timestamps_with_types[i - 1]
        t_curr, type_curr = timestamps_with_types[i]
        delta = t_curr - t_prev

        if delta > threshold:
            gaps.append({
                "index": i,
                "t_before_us": t_prev,
                "t_after_us": t_curr,
                "delta_us": delta,
                "type_before": type_prev,
                "type_after": type_curr,
            })
    return gaps


def format_time_us(us):
    """Format microseconds as human-readable time string."""
    s = us / 1e6
    if s < 60:
        return f"{s:.3f}s"
    m = int(s // 60)
    s_rem = s - m * 60
    return f"{m}m {s_rem:.3f}s"


def analyze_one_flight(flight_info):
    """
    Full analysis pipeline for one flight.
    Returns a dict with analysis results for further processing/plotting.
    """
    name = flight_info["name"]
    bin_path = flight_info["bin_path"]
    output_dir = flight_info["output_dir"]

    os.makedirs(output_dir, exist_ok=True)

    print("=" * 80)
    print(f"  ANALYSIS: {name}")
    print(f"  File: {bin_path}")
    print("=" * 80)

    # ── Parse ──────────────────────────────────────────────────────────────
    records, stats, config = parse_binary_file(bin_path)

    print(f"\nParsing statistics:")
    print(f"  File size:    {stats['file_size']:,} bytes")
    print(f"  Total frames: {stats['total_frames']}")
    print(f"  Good CRC:     {stats['good_crc']}")
    print(f"  Bad CRC:      {stats['bad_crc']}")
    print(f"  Type counts:")
    for tname, cnt in sorted(stats["type_counts"].items()):
        print(f"    {tname:20s}: {cnt}")

    # ── Timestamps ─────────────────────────────────────────────────────────
    all_ts = extract_all_timestamps(records)
    per_sensor = extract_per_sensor_timestamps(records)

    if len(all_ts) < 2:
        print("  ERROR: Not enough records to analyze.")
        return None

    t0 = all_ts[0][0]
    t_end = all_ts[-1][0]
    duration_s = (t_end - t0) / 1e6

    print(f"\nTimeline:")
    print(f"  First record at: {t0} us")
    print(f"  Last record at:  {t_end} us")
    print(f"  Duration:        {format_time_us(t_end - t0)}")

    # ── Launch time ────────────────────────────────────────────────────────
    launch_us = find_launch_time(records)
    if launch_us is not None:
        print(f"  Launch detected at: {launch_us} us  (T+{format_time_us(launch_us - t0)} from first record)")
    else:
        print("  Launch: NOT DETECTED in NonSensor data")

    # ── Per-sensor rate analysis ───────────────────────────────────────────
    print(f"\nPer-sensor sample rates:")
    sensor_median_deltas = {}
    for sname, times in sorted(per_sensor.items()):
        if len(times) < 2:
            print(f"  {sname:15s}: {len(times)} samples (not enough for rate calc)")
            continue
        deltas = np.diff(times)
        med = np.median(deltas)
        rate = 1e6 / med if med > 0 else 0
        sensor_median_deltas[sname] = med
        print(f"  {sname:15s}: {len(times):6d} samples, "
              f"median dt = {med:.0f} us ({rate:.1f} Hz), "
              f"min dt = {np.min(deltas):.0f} us, "
              f"max dt = {np.max(deltas):.0f} us")

    # ── Global gap detection ───────────────────────────────────────────────
    all_times_only = np.array([t for t, _ in all_ts], dtype=np.float64)
    all_deltas = np.diff(all_times_only)
    global_median_delta = np.median(all_deltas)
    global_mean_delta = np.mean(all_deltas)

    print(f"\nGlobal inter-record timing (all sensor types combined):")
    print(f"  Median delta: {global_median_delta:.1f} us ({1e6/global_median_delta:.1f} Hz)")
    print(f"  Mean delta:   {global_mean_delta:.1f} us ({1e6/global_mean_delta:.1f} Hz)")
    print(f"  Min delta:    {np.min(all_deltas):.1f} us")
    print(f"  Max delta:    {np.max(all_deltas):.1f} us")
    print(f"  Std delta:    {np.std(all_deltas):.1f} us")

    gaps = find_gaps(all_ts, global_median_delta, GAP_MULTIPLIER, MIN_GAP_US)

    print(f"\n{'─' * 80}")
    print(f"  GAPS DETECTED: {len(gaps)}  (threshold: {GAP_MULTIPLIER}x median = "
          f"{max(global_median_delta * GAP_MULTIPLIER, MIN_GAP_US):.0f} us = "
          f"{max(global_median_delta * GAP_MULTIPLIER, MIN_GAP_US) / 1000:.1f} ms)")
    print(f"{'─' * 80}")

    if gaps:
        # Header
        print(f"  {'#':>3s}  {'Gap Start':>14s}  {'Gap End':>14s}  "
              f"{'Duration':>12s}  {'Rel Launch':>12s}  {'Before':>12s}  {'After':>12s}")
        print(f"  {'':>3s}  {'(from t0)':>14s}  {'(from t0)':>14s}  "
              f"{'':>12s}  {'':>12s}  {'':>12s}  {'':>12s}")
        print(f"  {'-'*3}  {'-'*14}  {'-'*14}  "
              f"{'-'*12}  {'-'*12}  {'-'*12}  {'-'*12}")

        for i, g in enumerate(gaps):
            t_before_rel = (g["t_before_us"] - t0) / 1e6
            t_after_rel = (g["t_after_us"] - t0) / 1e6
            dur_ms = g["delta_us"] / 1000.0

            if launch_us is not None:
                rel_launch = (g["t_before_us"] - launch_us) / 1e6
                rel_str = f"{rel_launch:+.3f}s"
            else:
                rel_str = "N/A"

            print(f"  {i+1:3d}  {t_before_rel:14.6f}s  {t_after_rel:14.6f}s  "
                  f"{dur_ms:10.3f} ms  {rel_str:>12s}  "
                  f"{g['type_before']:>12s}  {g['type_after']:>12s}")

        # Total gap time
        total_gap_us = sum(g["delta_us"] for g in gaps)
        print(f"\n  Total gap time: {total_gap_us/1000:.1f} ms "
              f"({total_gap_us/1e6:.3f} s, "
              f"{100*total_gap_us/(t_end - t0):.2f}% of flight duration)")

    # ── Per-sensor gap analysis ────────────────────────────────────────────
    print(f"\n{'─' * 80}")
    print(f"  PER-SENSOR GAP ANALYSIS")
    print(f"{'─' * 80}")

    per_sensor_gaps = {}
    for sname, times in sorted(per_sensor.items()):
        if len(times) < 2:
            continue
        deltas = np.diff(times)
        med = np.median(deltas)
        threshold = max(med * GAP_MULTIPLIER, MIN_GAP_US)

        sensor_ts = [(t, sname) for t in times]
        sgaps = find_gaps(sensor_ts, med, GAP_MULTIPLIER, MIN_GAP_US)
        per_sensor_gaps[sname] = sgaps

        if sgaps:
            print(f"\n  {sname}: {len(sgaps)} gaps (threshold: {threshold:.0f} us = {threshold/1000:.1f} ms)")
            for j, g in enumerate(sgaps[:10]):  # Limit to first 10
                t_rel = (g["t_before_us"] - t0) / 1e6
                dur_ms = g["delta_us"] / 1000.0
                if launch_us is not None:
                    rel_l = (g["t_before_us"] - launch_us) / 1e6
                    print(f"    Gap {j+1}: at T+{t_rel:.3f}s (launch{rel_l:+.3f}s), duration {dur_ms:.1f} ms")
                else:
                    print(f"    Gap {j+1}: at T+{t_rel:.3f}s, duration {dur_ms:.1f} ms")
            if len(sgaps) > 10:
                print(f"    ... and {len(sgaps) - 10} more gaps")
        else:
            print(f"\n  {sname}: No gaps detected")

    # ── Rocket state timeline ──────────────────────────────────────────────
    print(f"\n{'─' * 80}")
    print(f"  ROCKET STATE TIMELINE")
    print(f"{'─' * 80}")

    ns_recs = records.get("NonSensor", [])
    if ns_recs:
        prev_state = None
        for rec in ns_recs:
            st = rec.get("rocket_state", -1)
            if st != prev_state:
                t_rel = (rec["time_us"] - t0) / 1e6
                state_name = ROCKET_STATES.get(st, f"UNKNOWN({st})")
                if launch_us is not None:
                    rel_l = (rec["time_us"] - launch_us) / 1e6
                    print(f"  State -> {state_name:12s} at T+{t_rel:.3f}s (launch{rel_l:+.3f}s)")
                else:
                    print(f"  State -> {state_name:12s} at T+{t_rel:.3f}s")
                prev_state = st

    # ── Start-of-flight focus ──────────────────────────────────────────────
    print(f"\n{'─' * 80}")
    print(f"  START-OF-FLIGHT DETAIL (first 5 seconds)")
    print(f"{'─' * 80}")

    window_us = 5.0 * 1e6  # 5 seconds
    early_records = [(t, typ) for t, typ in all_ts if (t - t0) < window_us]
    print(f"  Records in first 5s: {len(early_records)} out of {len(all_ts)} total")

    # Count per sensor in first 5s
    type_counts_early = defaultdict(int)
    for _, typ in early_records:
        type_counts_early[typ] += 1
    print(f"  Breakdown:")
    for sname, cnt in sorted(type_counts_early.items()):
        total_for_sensor = len(records.get(sname, []))
        pct = 100 * cnt / total_for_sensor if total_for_sensor > 0 else 0
        print(f"    {sname:15s}: {cnt:5d} records ({pct:.1f}% of total for this sensor)")

    # Show the first N records to see what's happening at startup
    print(f"\n  First 30 records:")
    print(f"  {'#':>4s}  {'Time (us)':>12s}  {'dt (us)':>10s}  {'dt (ms)':>10s}  {'Type':>12s}  {'Rel t0':>10s}")
    print(f"  {'-'*4}  {'-'*12}  {'-'*10}  {'-'*10}  {'-'*12}  {'-'*10}")
    for i, (t, typ) in enumerate(all_ts[:30]):
        if i == 0:
            dt_str = "---"
            dt_ms_str = "---"
        else:
            dt = t - all_ts[i-1][0]
            dt_str = f"{dt:.0f}"
            dt_ms_str = f"{dt/1000:.2f}"
        rel_s = (t - t0) / 1e6
        print(f"  {i:4d}  {t:12.0f}  {dt_str:>10s}  {dt_ms_str:>10s}  {typ:>12s}  {rel_s:.6f}s")

    # ── Return results for plotting ────────────────────────────────────────
    return {
        "name": name,
        "output_dir": output_dir,
        "records": records,
        "stats": stats,
        "all_ts": all_ts,
        "all_times": all_times_only,
        "all_deltas": all_deltas,
        "per_sensor": per_sensor,
        "gaps": gaps,
        "per_sensor_gaps": per_sensor_gaps,
        "launch_us": launch_us,
        "t0": t0,
        "global_median_delta": global_median_delta,
    }


def plot_gap_analysis(result):
    """
    Generate gap analysis plots for one flight.
    """
    if result is None:
        return []

    name = result["name"]
    output_dir = result["output_dir"]
    all_times = result["all_times"]
    all_deltas = result["all_deltas"]
    gaps = result["gaps"]
    launch_us = result["launch_us"]
    t0 = result["t0"]
    per_sensor = result["per_sensor"]
    per_sensor_gaps = result["per_sensor_gaps"]
    global_median_delta = result["global_median_delta"]

    plot_paths = []

    # Time axis: seconds from first record
    t_rel_s = (all_times[:-1] - t0) / 1e6  # delta is between consecutive, use earlier time
    delta_ms = all_deltas / 1000.0

    threshold_ms = max(global_median_delta * GAP_MULTIPLIER, MIN_GAP_US) / 1000.0

    # ── Plot 1: Full flight overview ───────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(16, 14), gridspec_kw={"height_ratios": [2, 2, 1]})
    fig.suptitle(f"Gap Analysis: {name}", fontsize=14, fontweight="bold")

    # Panel 1: All deltas (log scale)
    ax = axes[0]
    ax.scatter(t_rel_s, delta_ms, s=2, alpha=0.3, c="steelblue", label="Inter-record delta")
    ax.axhline(threshold_ms, color="red", linestyle="--", linewidth=1,
               label=f"Gap threshold ({threshold_ms:.1f} ms)")
    ax.axhline(global_median_delta / 1000.0, color="green", linestyle=":", linewidth=1,
               label=f"Median delta ({global_median_delta/1000:.2f} ms)")
    if launch_us is not None:
        launch_rel = (launch_us - t0) / 1e6
        ax.axvline(launch_rel, color="orange", linewidth=2, label=f"Launch (T+{launch_rel:.2f}s)")

    # Mark gaps
    for g in gaps:
        gt = (g["t_before_us"] - t0) / 1e6
        gd = g["delta_us"] / 1000.0
        ax.scatter([gt], [gd], s=80, marker="x", color="red", zorder=10)

    ax.set_yscale("log")
    ax.set_ylabel("Inter-record delta (ms, log scale)")
    ax.set_xlabel("Time from first record (s)")
    ax.set_title("Full Flight: Inter-record Time Delta")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 2: Start-of-flight zoom (first 10 seconds)
    ax2 = axes[1]
    zoom_end_s = 10.0
    mask = t_rel_s < zoom_end_s
    if np.any(mask):
        ax2.scatter(t_rel_s[mask], delta_ms[mask], s=6, alpha=0.5, c="steelblue",
                    label="Inter-record delta")
        ax2.axhline(threshold_ms, color="red", linestyle="--", linewidth=1,
                     label=f"Gap threshold ({threshold_ms:.1f} ms)")
        ax2.axhline(global_median_delta / 1000.0, color="green", linestyle=":", linewidth=1,
                     label=f"Median delta ({global_median_delta/1000:.2f} ms)")
        if launch_us is not None:
            launch_rel = (launch_us - t0) / 1e6
            if launch_rel < zoom_end_s:
                ax2.axvline(launch_rel, color="orange", linewidth=2,
                            label=f"Launch (T+{launch_rel:.2f}s)")

        # Mark gaps in zoom window
        for g in gaps:
            gt = (g["t_before_us"] - t0) / 1e6
            if gt < zoom_end_s:
                gd = g["delta_us"] / 1000.0
                ax2.scatter([gt], [gd], s=120, marker="x", color="red", zorder=10)
                ax2.annotate(f"{gd:.1f} ms", (gt, gd),
                             textcoords="offset points", xytext=(8, 8),
                             fontsize=8, color="red", fontweight="bold")

    ax2.set_yscale("log")
    ax2.set_ylabel("Inter-record delta (ms, log scale)")
    ax2.set_xlabel("Time from first record (s)")
    ax2.set_title("Start of Flight (first 10s): Inter-record Time Delta")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    # Panel 3: Record density (histogram)
    ax3 = axes[2]
    t_all_rel = (all_times - t0) / 1e6
    bin_width_s = 0.5
    bins = np.arange(0, t_all_rel[-1] + bin_width_s, bin_width_s)
    ax3.hist(t_all_rel, bins=bins, color="steelblue", alpha=0.7, edgecolor="none")
    if launch_us is not None:
        launch_rel = (launch_us - t0) / 1e6
        ax3.axvline(launch_rel, color="orange", linewidth=2, label=f"Launch")

    # Mark gap regions
    for g in gaps:
        g_start = (g["t_before_us"] - t0) / 1e6
        g_end = (g["t_after_us"] - t0) / 1e6
        ax3.axvspan(g_start, g_end, color="red", alpha=0.2)

    ax3.set_ylabel(f"Records per {bin_width_s}s bin")
    ax3.set_xlabel("Time from first record (s)")
    ax3.set_title("Record Density Over Time")
    ax3.legend(loc="upper right", fontsize=8)
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    path1 = os.path.join(output_dir, "gap_analysis_overview.png")
    fig.savefig(path1, dpi=FIGURE_DPI, bbox_inches="tight")
    plot_paths.append(path1)
    print(f"\n  Saved: {path1}")

    # ── Plot 2: Per-sensor delta view (start of flight) ────────────────────
    sensor_names = sorted(per_sensor.keys())
    n_sensors = len(sensor_names)
    if n_sensors > 0:
        fig2, axes2 = plt.subplots(n_sensors, 1, figsize=(16, 3 * n_sensors + 2),
                                    sharex=True)
        if n_sensors == 1:
            axes2 = [axes2]

        fig2.suptitle(f"Per-Sensor Gap Analysis (first 15s): {name}",
                       fontsize=14, fontweight="bold")

        zoom_end_s2 = 15.0
        for idx, sname in enumerate(sensor_names):
            ax = axes2[idx]
            times = per_sensor[sname]
            if len(times) < 2:
                ax.set_title(f"{sname}: insufficient data")
                continue

            deltas = np.diff(times)
            t_rel = (times[:-1] - t0) / 1e6
            d_ms = deltas / 1000.0

            mask = t_rel < zoom_end_s2
            if not np.any(mask):
                ax.set_title(f"{sname}: no data in first {zoom_end_s2}s")
                continue

            med_d = np.median(deltas)
            rate_hz = 1e6 / med_d if med_d > 0 else 0

            ax.scatter(t_rel[mask], d_ms[mask], s=8, alpha=0.6, c="steelblue")
            ax.axhline(med_d / 1000.0, color="green", linestyle=":", linewidth=1,
                        label=f"Median: {med_d/1000:.2f} ms ({rate_hz:.0f} Hz)")

            thresh_sensor = max(med_d * GAP_MULTIPLIER, MIN_GAP_US) / 1000.0
            ax.axhline(thresh_sensor, color="red", linestyle="--", linewidth=1,
                        label=f"Threshold: {thresh_sensor:.1f} ms")

            if launch_us is not None:
                launch_rel = (launch_us - t0) / 1e6
                if launch_rel < zoom_end_s2:
                    ax.axvline(launch_rel, color="orange", linewidth=2, alpha=0.7)

            # Mark per-sensor gaps
            sgaps = per_sensor_gaps.get(sname, [])
            for g in sgaps:
                gt = (g["t_before_us"] - t0) / 1e6
                if gt < zoom_end_s2:
                    gd = g["delta_us"] / 1000.0
                    ax.scatter([gt], [gd], s=100, marker="x", color="red", zorder=10)
                    ax.annotate(f"{gd:.0f}ms", (gt, gd),
                                textcoords="offset points", xytext=(5, 5),
                                fontsize=7, color="red")

            ax.set_yscale("log")
            ax.set_ylabel("Delta (ms)")
            ax.set_title(f"{sname}", fontsize=10)
            ax.legend(loc="upper right", fontsize=7)
            ax.grid(True, alpha=0.3)

        axes2[-1].set_xlabel("Time from first record (s)")
        plt.tight_layout()
        path2 = os.path.join(output_dir, "gap_analysis_per_sensor.png")
        fig2.savefig(path2, dpi=FIGURE_DPI, bbox_inches="tight")
        plot_paths.append(path2)
        print(f"  Saved: {path2}")

    # ── Plot 3: Start-of-flight record timeline ────────────────────────────
    fig3, ax3 = plt.subplots(figsize=(16, 6))
    fig3.suptitle(f"Record Timeline (first 5s): {name}", fontsize=14, fontweight="bold")

    # Assign each sensor a y-level
    sensor_y = {sname: i for i, sname in enumerate(sorted(per_sensor.keys()))}
    colors = plt.cm.Set2(np.linspace(0, 1, len(sensor_y)))
    color_map = {sname: colors[i] for i, sname in enumerate(sorted(per_sensor.keys()))}

    zoom_s = 5.0
    for sname, times in sorted(per_sensor.items()):
        y = sensor_y[sname]
        t_rel = (times - t0) / 1e6
        mask = t_rel < zoom_s
        if np.any(mask):
            ax3.scatter(t_rel[mask], np.full(np.sum(mask), y), s=4, alpha=0.6,
                        color=color_map[sname], label=sname)

    if launch_us is not None:
        launch_rel = (launch_us - t0) / 1e6
        if launch_rel < zoom_s:
            ax3.axvline(launch_rel, color="orange", linewidth=2, label="Launch")

    # Mark gap regions
    for g in gaps:
        g_start = (g["t_before_us"] - t0) / 1e6
        g_end = (g["t_after_us"] - t0) / 1e6
        if g_start < zoom_s:
            ax3.axvspan(g_start, min(g_end, zoom_s), color="red", alpha=0.15)

    ax3.set_yticks(list(sensor_y.values()))
    ax3.set_yticklabels(list(sensor_y.keys()))
    ax3.set_xlabel("Time from first record (s)")
    ax3.set_ylabel("Sensor Type")
    ax3.set_title("Individual Record Timestamps (first 5s)")
    ax3.legend(loc="upper right", fontsize=8, ncol=2)
    ax3.grid(True, alpha=0.3, axis="x")
    plt.tight_layout()
    path3 = os.path.join(output_dir, "gap_analysis_timeline.png")
    fig3.savefig(path3, dpi=FIGURE_DPI, bbox_inches="tight")
    plot_paths.append(path3)
    print(f"  Saved: {path3}")

    return plot_paths


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    all_plot_paths = []

    for flight in FLIGHTS:
        result = analyze_one_flight(flight)
        if result is not None:
            paths = plot_gap_analysis(result)
            all_plot_paths.extend(paths)
        print()

    # ── Summary comparison ─────────────────────────────────────────────────
    print("=" * 80)
    print("  CROSS-FLIGHT SUMMARY")
    print("=" * 80)
    print("  (See individual flight sections above for detailed gap tables)")
    print()

    # Show all plots
    plt.show()

    return all_plot_paths


if __name__ == "__main__":
    main()
