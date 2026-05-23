"""Gaps module — temporal gaps and per-sensor sample-rate health.

Adapted from analyze_gaps.py + analyze_gaps_detailed.py. Surfaces:

  * Per-sensor gap counts & median dt (each sensor judged against its own
    median, not the global one).
  * Gap-duration histogram.
  * Per-sensor inter-sample dt distributions (catches jitter and bimodal
    timing that a single "median" metric hides).
  * Phase classification (which rocket state did each gap fall in?).
"""

from __future__ import annotations

import sys
from collections import Counter
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
from plot_flight_data_mini import ROCKET_STATES  # noqa: E402

from ..flight import Flight
from ..registry import AnalysisResult


# ── Phase classification ──────────────────────────────────────────────────────

def _state_timeline(records) -> list[tuple[int, int]]:
    """Return list of (time_us, rocket_state) state transitions."""
    out = []
    prev = None
    for r in records.get("NonSensor", []):
        st = r.get("rocket_state", -1)
        if st != prev:
            out.append((r["time_us"], st))
            prev = st
    return out


def _state_at(time_us: int, timeline: list[tuple[int, int]]) -> int:
    """Look up rocket_state at a given timestamp via the timeline."""
    state = -1
    for ts, st in timeline:
        if ts <= time_us:
            state = st
        else:
            break
    return state


def _phase_breakdown(gaps, timeline) -> dict[str, int]:
    counts: Counter[str] = Counter()
    for g in gaps:
        st = _state_at(g["t_before_us"], timeline)
        counts[ROCKET_STATES.get(st, f"UNKNOWN({st})")] += 1
    return dict(counts)


# ── Per-sensor analysis ───────────────────────────────────────────────────────

def _per_sensor_summary(per_sensor) -> tuple[dict[str, dict[str, float]], dict[str, list[dict]]]:
    """Compute per-sensor rate stats and gap detection (sensor judged against itself)."""
    summary: dict[str, dict[str, float]] = {}
    per_sensor_gaps: dict[str, list[dict]] = {}
    for sname, times in per_sensor.items():
        if times.size < 2:
            continue
        dt = np.diff(times)
        med = float(np.median(dt))
        if med <= 0:
            continue
        gaps = find_gaps([(t, sname) for t in times], med, GAP_MULTIPLIER, MIN_GAP_US)
        summary[sname] = {
            "n": int(times.size),
            "median_dt_us": round(med, 1),
            "rate_hz": round(1e6 / med, 1),
            "min_dt_us": float(np.min(dt)),
            "max_dt_us": float(np.max(dt)),
            "jitter_std_us": round(float(np.std(dt)), 1),
            "gap_count": len(gaps),
        }
        per_sensor_gaps[sname] = gaps
    return summary, per_sensor_gaps


# ── Figures ───────────────────────────────────────────────────────────────────

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


def _plot_per_sensor_rates(summary):
    if not summary:
        return None
    fig, ax = plt.subplots(figsize=(10, 4))
    names = sorted(summary.keys())
    rates = [summary[n]["rate_hz"] for n in names]
    ax.barh(names, rates, color="tab:blue", alpha=0.7)
    for i, r in enumerate(rates):
        ax.text(r, i, f" {r:.0f} Hz", va="center", fontsize=9)
    ax.set_xlabel("Median rate (Hz)")
    ax.set_title("Per-sensor sample rate")
    ax.grid(True, axis="x", alpha=0.3)
    fig.tight_layout()
    return fig


def _plot_gap_duration_histogram(gaps):
    if not gaps:
        return None
    durs_ms = np.array([g["delta_us"] / 1000.0 for g in gaps])
    fig, ax = plt.subplots(figsize=(10, 4))
    upper = float(np.percentile(durs_ms, 99) if durs_ms.size > 1 else durs_ms[0])
    upper = max(upper, 5.0)
    ax.hist(durs_ms, bins=np.linspace(0, upper, 40), color="tab:red", alpha=0.75)
    ax.set_xlabel("Gap duration (ms)")
    ax.set_ylabel("Count")
    ax.set_title(f"Gap-duration distribution (n={len(gaps)}, longest={durs_ms.max():.1f} ms)")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    return fig


def _plot_per_sensor_dt(per_sensor):
    """Multi-panel histograms of inter-sample dt per sensor (jitter view)."""
    panels = [(n, t) for n, t in sorted(per_sensor.items()) if t.size >= 20]
    if not panels:
        return None
    n = len(panels)
    cols = 2
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(12, 2.5 * rows), squeeze=False)
    for i, (sname, times) in enumerate(panels):
        ax = axes[i // cols][i % cols]
        dt_ms = np.diff(times) / 1000.0
        med = float(np.median(dt_ms))
        # Clip extreme outliers for the histogram (gaps are visible elsewhere)
        clip = float(np.percentile(dt_ms, 99.5))
        clip = max(clip, med * 3)
        bins = np.linspace(0, clip, 50)
        ax.hist(dt_ms, bins=bins, color="tab:blue", alpha=0.7)
        ax.axvline(med, color="red", linestyle="--", lw=1.0, label=f"median {med:.2f} ms")
        ax.set_title(f"{sname} — inter-sample dt (n={times.size})", fontsize=10)
        ax.set_xlabel("dt (ms)")
        ax.set_ylabel("Count")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, axis="y", alpha=0.3)
    # Hide unused panels
    for j in range(n, rows * cols):
        axes[j // cols][j % cols].axis("off")
    fig.suptitle("Per-sensor inter-sample timing (jitter)", fontsize=11)
    fig.tight_layout()
    return fig


# ── Entry point ───────────────────────────────────────────────────────────────

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
    timeline = _state_timeline(records)

    all_times = np.array([t for t, _ in all_ts], dtype=np.float64)
    deltas = np.diff(all_times)
    median_delta = float(np.median(deltas))

    # Global merged-stream gaps
    gaps = find_gaps(all_ts, median_delta, GAP_MULTIPLIER, MIN_GAP_US)
    total_gap_us = sum(g["delta_us"] for g in gaps)

    # Per-sensor (each judged against its own median)
    sensor_summary, sensor_gaps = _per_sensor_summary(per_sensor)
    sensor_gap_totals = {s: len(g) for s, g in sensor_gaps.items() if g}

    # Phase breakdown
    phase = _phase_breakdown(gaps, timeline) if gaps else {}

    # Headline metrics (kept terse — per-sensor detail lives in text block)
    result.metrics = {
        "total_frames":       len(all_ts),
        "duration_s":         round((t_end - t0) / 1e6, 2),
        "median_dt_us":       round(median_delta, 1),
        "median_rate_hz":     round(1e6 / median_delta, 1) if median_delta > 0 else 0,
        "gap_count":          len(gaps),
        "longest_gap_ms":     round(max((g["delta_us"] for g in gaps), default=0) / 1000.0, 2),
        "total_gap_ms":       round(total_gap_us / 1000.0, 2),
        "gap_pct_of_flight":  round(100 * total_gap_us / (t_end - t0), 3) if t_end > t0 else 0,
        "per_sensor_gap_counts": (
            ", ".join(f"{s}={n}" for s, n in sorted(sensor_gap_totals.items()))
            if sensor_gap_totals else "none"
        ),
        "gaps_by_phase": (
            ", ".join(f"{p}={n}" for p, n in sorted(phase.items())) if phase else "—"
        ),
    }

    # Warnings
    if len(gaps) > 10:
        result.warnings.append(
            f"{len(gaps)} gaps exceed threshold "
            f"(>{GAP_MULTIPLIER}× median dt = "
            f"{max(median_delta*GAP_MULTIPLIER, MIN_GAP_US)/1000:.1f} ms)"
        )
    # Sensors that drop frames vs their own cadence
    for sname, cnt in sensor_gap_totals.items():
        if cnt >= 5:
            result.warnings.append(
                f"{sname}: {cnt} self-gaps "
                f"(jitter_std={sensor_summary[sname]['jitter_std_us']:.0f} µs)."
            )

    # Text block: per-sensor table + worst-gaps detail
    lines: list[str] = []
    lines.append("Per-sensor cadence:")
    lines.append("  sensor          n      rate(Hz)    median(µs)  jitter_std(µs)   gaps")
    lines.append("  " + "-" * 78)
    for s in sorted(sensor_summary.keys()):
        row = sensor_summary[s]
        lines.append(
            f"  {s:<14s}  {row['n']:>6d}  {row['rate_hz']:>10.1f}  "
            f"{row['median_dt_us']:>10.0f}  {row['jitter_std_us']:>14.0f}   "
            f"{row['gap_count']:>4d}"
        )

    if gaps:
        lines.append("")
        lines.append("Worst global gaps (top 10 by duration):")
        lines.append("   # | start (s) | dur (ms) | phase       | before -> after")
        lines.append("  " + "-" * 64)
        worst = sorted(gaps, key=lambda g: -g["delta_us"])[:10]
        for i, g in enumerate(worst):
            t_rel = (g["t_before_us"] - t0) / 1e6
            ph = ROCKET_STATES.get(_state_at(g["t_before_us"], timeline), "—")
            lines.append(
                f"  {i+1:>2} | {t_rel:>9.3f} | {g['delta_us']/1000:>8.1f} | "
                f"{ph:<11s} | {g['type_before']:>10} -> {g['type_after']}"
            )
    result.text = "\n".join(lines)

    # Figures
    for fn, args in (
        (_plot_gap_timeline,       (all_ts, gaps, t0, launch_us)),
        (_plot_per_sensor_rates,   (sensor_summary,)),
        (_plot_gap_duration_histogram, (gaps,)),
        (_plot_per_sensor_dt,      (per_sensor,)),
    ):
        try:
            fig = fn(*args)
        except Exception as e:  # noqa: BLE001
            result.warnings.append(f"{fn.__name__}: {type(e).__name__}: {e}")
            continue
        if fig is not None:
            result.figures.append(fig)

    return result
