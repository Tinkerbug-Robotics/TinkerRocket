"""GNSS staleness / health module.

Checks for: position freeze (lat/lon repeats), fix-mode drops, and frame-rate
gaps in the GNSS stream.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

from ..flight import Flight
from ..registry import AnalysisResult


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="gnss_staleness", title="GNSS Health")
    recs = flight.records
    t0 = flight.t0_us or 0

    gnss = recs.get("GNSS") or []
    if not gnss:
        result.warnings.append("No GNSS records.")
        return result

    g_t = get_array(gnss, "time_us")
    duration_s = (g_t[-1] - g_t[0]) / 1e6 if g_t.size > 1 else 0
    rate_hz = (g_t.size / duration_s) if duration_s > 0 else 0

    # Frame-rate gaps
    dt = np.diff(g_t) / 1e6
    med_dt = float(np.median(dt)) if dt.size else 0
    big_gaps = int(np.sum(dt > max(0.5, 5 * med_dt))) if dt.size else 0

    # Position freeze. Identical lat/lon alone is NOT staleness: a stationary
    # receiver (pad / landed) legitimately re-quantizes to the same 1e-7 deg
    # (~1 cm) coordinates between epochs — every pair flagged across the 7/05
    # flights was exactly that (solution time-of-day advancing, fix 3D, 16-30
    # sats). A fix only counts as FROZEN when the solution timestamp did not
    # advance either — i.e. the receiver/driver repeated a stale solution,
    # which the FC's gnss_fix_is_new EKF gate would also reject. Stationary
    # repeats stay visible as a metric.
    frozen = 0
    stationary_repeats = 0
    if "lat" in gnss[0]:
        lat = get_array(gnss, "lat")
        lon = get_array(gnss, "lon")
        if lat.size > 1:
            same_pos = (np.diff(lat) == 0) & (np.diff(lon) == 0)
            if all(k in gnss[0] for k in ("hour", "minute", "second", "milli_sec")):
                tod_ms = (get_array(gnss, "hour") * 3600000.0
                          + get_array(gnss, "minute") * 60000.0
                          + get_array(gnss, "second") * 1000.0
                          + get_array(gnss, "milli_sec"))
                same_tod = (np.diff(tod_ms) == 0)
                frozen = int(np.sum(same_pos & same_tod))
                stationary_repeats = int(np.sum(same_pos & ~same_tod))
            else:
                # Old logs without solution time-of-day: position-only count,
                # reported as the softer metric.
                stationary_repeats = int(np.sum(same_pos))

    # Fix-mode 0 (no fix) count
    fix_lost = 0
    if "fix_mode" in gnss[0]:
        fm = np.array([r.get("fix_mode", 0) for r in gnss])
        fix_lost = int(np.sum(fm == 0))

    # Sat-count statistics
    sat_min = sat_med = sat_max = None
    if "num_sats" in gnss[0]:
        sats = np.array([r["num_sats"] for r in gnss])
        sat_min, sat_med, sat_max = int(sats.min()), int(np.median(sats)), int(sats.max())

    result.metrics = {
        "gnss_frames": int(g_t.size),
        "gnss_rate_hz": round(rate_hz, 2),
        "median_dt_s": round(med_dt, 3),
        "big_gaps": big_gaps,
        "frozen_fixes_stale_tod": frozen,
        "stationary_position_repeats": stationary_repeats,
        "fix_mode_zero_count": fix_lost,
        "sats_min/med/max": (
            f"{sat_min} / {sat_med} / {sat_max}" if sat_min is not None else "—"
        ),
    }

    if frozen > 0:
        result.warnings.append(
            f"{frozen} FROZEN fix(es): identical lat/lon with a non-advancing "
            "solution timestamp — receiver/driver repeated a stale solution."
        )
    if fix_lost > 0:
        result.warnings.append(f"{fix_lost} frames with fix_mode=0 (no fix).")
    if big_gaps > 0:
        result.warnings.append(f"{big_gaps} frame gap(s) > 5× median dt.")

    # Figure: GNSS dt histogram + sat count + fix mode
    if g_t.size > 5:
        fig, axes = plt.subplots(1, 3, figsize=(14, 3.5))
        axes[0].hist(dt, bins=50, color="tab:blue", alpha=0.7)
        axes[0].set_xlabel("Inter-frame dt (s)")
        axes[0].set_ylabel("Count")
        axes[0].set_title(f"GNSS frame dt (median={med_dt:.3f}s)")
        axes[0].grid(True, alpha=0.3)

        if "num_sats" in gnss[0]:
            axes[1].plot((g_t - t0) / 1e6, sats, lw=0.7, color="tab:orange")
            axes[1].set_xlabel("Time (s)")
            axes[1].set_ylabel("Sat count")
            axes[1].set_title("Satellites in solution")
            axes[1].grid(True, alpha=0.3)

        if "fix_mode" in gnss[0]:
            axes[2].step((g_t - t0) / 1e6, fm, where="post", color="tab:green")
            axes[2].set_xlabel("Time (s)")
            axes[2].set_ylabel("fix_mode")
            axes[2].set_title("Fix mode timeline")
            axes[2].grid(True, alpha=0.3)
        fig.tight_layout()
        result.figures.append(fig)

    return result
