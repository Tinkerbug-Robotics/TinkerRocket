"""Gaps module — temporal gaps and per-sensor sample-rate health.

Wraps the core analysis from analyze_gaps.py.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from analyze_gaps import (  # noqa: E402
    extract_all_timestamps,
    extract_per_sensor_timestamps,
    find_gaps,
    find_launch_time,
    GAP_MULTIPLIER,
    MIN_GAP_US,
)

from ..flight import Flight
from ..registry import AnalysisResult


def _plot_gap_timeline(all_ts, gaps, t0, launch_us):
    if not all_ts:
        return None
    fig, ax = plt.subplots(figsize=(12, 3))
    times = np.array([t for t, _ in all_ts])
    ax.scatter((times - t0) / 1e6, np.ones(len(times)), s=2, alpha=0.4,
               color="tab:blue", label=f"{len(times):,} records")
    for g in gaps:
        ax.axvspan((g["t_before_us"] - t0) / 1e6, (g["t_after_us"] - t0) / 1e6,
                   color="red", alpha=0.4)
    if launch_us is not None:
        ax.axvline((launch_us - t0) / 1e6, color="green", linestyle="--",
                   label="launch", alpha=0.7)
    ax.set_xlabel("Time (s)")
    ax.set_yticks([])
    ax.set_title(f"Frame Timeline — {len(gaps)} gap(s) flagged")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def _plot_per_sensor_rates(per_sensor):
    if not per_sensor:
        return None
    fig, ax = plt.subplots(figsize=(10, 4))
    names, rates = [], []
    for sname, times in sorted(per_sensor.items()):
        if len(times) < 2:
            continue
        med = float(np.median(np.diff(times)))
        if med > 0:
            names.append(sname)
            rates.append(1e6 / med)
    if not names:
        plt.close(fig)
        return None
    ax.barh(names, rates, color="tab:blue", alpha=0.7)
    for i, r in enumerate(rates):
        ax.text(r, i, f" {r:.0f} Hz", va="center", fontsize=9)
    ax.set_xlabel("Median rate (Hz)")
    ax.set_title("Per-sensor sample rate")
    ax.grid(True, axis="x", alpha=0.3)
    fig.tight_layout()
    return fig


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="gaps", title="Frame Gaps & Sample Rates")
    records = flight.records

    all_ts = extract_all_timestamps(records)
    per_sensor = extract_per_sensor_timestamps(records)
    if len(all_ts) < 2:
        result.warnings.append("Not enough records to analyze.")
        return result

    t0 = all_ts[0][0]
    t_end = all_ts[-1][0]
    launch_us = find_launch_time(records)

    all_times = np.array([t for t, _ in all_ts], dtype=np.float64)
    deltas = np.diff(all_times)
    median_delta = float(np.median(deltas))

    gaps = find_gaps(all_ts, median_delta, GAP_MULTIPLIER, MIN_GAP_US)
    total_gap_us = sum(g["delta_us"] for g in gaps)

    result.metrics = {
        "total_frames": len(all_ts),
        "duration_s": round((t_end - t0) / 1e6, 2),
        "median_dt_us": round(median_delta, 1),
        "median_rate_hz": round(1e6 / median_delta, 1) if median_delta > 0 else 0,
        "gap_count": len(gaps),
        "total_gap_ms": round(total_gap_us / 1000.0, 2),
        "gap_pct_of_flight": round(100 * total_gap_us / (t_end - t0), 3) if t_end > t0 else 0,
    }

    if gaps:
        if len(gaps) > 10:
            result.warnings.append(f"{len(gaps)} gaps exceed threshold "
                                   f"(>{GAP_MULTIPLIER}× median dt = "
                                   f"{max(median_delta*GAP_MULTIPLIER, MIN_GAP_US)/1000:.1f} ms)")
        # Build a small table of the worst gaps
        worst = sorted(gaps, key=lambda g: -g["delta_us"])[:10]
        lines = ["  # | start (s) | dur (ms) | before -> after"]
        for i, g in enumerate(worst):
            t_rel = (g["t_before_us"] - t0) / 1e6
            lines.append(f"  {i+1:>2} | {t_rel:>9.3f} | {g['delta_us']/1000:>8.1f} | "
                         f"{g['type_before']:>10} -> {g['type_after']}")
        result.text = "\n".join(lines)

    fig = _plot_gap_timeline(all_ts, gaps, t0, launch_us)
    if fig is not None:
        result.figures.append(fig)
    fig = _plot_per_sensor_rates(per_sensor)
    if fig is not None:
        result.figures.append(fig)

    return result
