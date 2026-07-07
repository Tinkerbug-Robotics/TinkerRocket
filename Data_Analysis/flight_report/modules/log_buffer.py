"""Log-buffer module — MRAM ring high-water + smaller-chip viability.

Reads the LogBufferStats frames (msg 0xE2) that the OutComputer emits into
the flight log at ~1 Hz while a session is open.  Each frame is a snapshot
of the runtime ring capacity, current fill, peak fill since boot, overrun
count, and drop-oldest bytes.

The point of this module is to answer "is the 128 KB MR25H10 over-
provisioned?".  We report:

  * Peak ring fill in bytes and as a percent of the configured ring size.
  * Whether the ring ever overran or dropped tail bytes (i.e. data loss).
  * A viability table for common smaller / cheaper alternatives, marking
    each "would have fit" or "would have dropped data".

For pre-instrumentation flight logs (no LogBufferStats frames present),
the module emits a single warning and otherwise renders nothing — so the
report still renders for old captures.
"""

from __future__ import annotations

from typing import Optional

import matplotlib.pyplot as plt

from ..flight import Flight
from ..registry import AnalysisResult


# Alternative chip sizes (bytes) we want to evaluate against the peak ring
# fill.  Names are deliberately short — the table cell width is the limit.
_ALTERNATIVES = [
    ("MR25H10 (current, 1 Mbit MRAM)",        131072),
    ("CY15B102Q-class (1 Mbit FRAM)",         131072),
    ("MR25H40 / 256-Kbit FRAM",                32768),
    ("512-Kbit FRAM (e.g. FM25V05)",           65536),
    ("128-Kbit FRAM (e.g. FM25V01)",           16384),
]


def _peak_fill_pct(peak: int, capacity: int) -> Optional[float]:
    if capacity <= 0:
        return None
    return 100.0 * peak / capacity


def _viability_text(peak: int, capacity: int) -> str:
    """One-line-per-row 'would this chip have fit?' table."""
    lines = [
        f"Ring capacity (configured):  {capacity:>8,} bytes ({capacity / 1024.0:.1f} KB)",
        f"Ring peak fill (this flight):{peak:>8,} bytes ({peak / 1024.0:.1f} KB)",
        "",
        "Alternative chip viability (would it have held the observed peak?):",
        "",
        f"  {'Part':<38} {'Capacity':>10}  Verdict",
        f"  {'-' * 38} {'-' * 10}  {'-' * 28}",
    ]
    for name, size_bytes in _ALTERNATIVES:
        if peak <= size_bytes:
            verdict = f"OK ({peak / size_bytes * 100.0:5.1f}% used)"
        else:
            verdict = f"DROP — peak exceeds capacity by {peak - size_bytes:,} B"
        lines.append(f"  {name:<38} {size_bytes:>8,} B  {verdict}")
    return "\n".join(lines)


def _plot_ring_fill(samples, capacity: int):
    """Time-series of ring_fill vs time with ring_highwater + capacity ceiling."""
    if not samples:
        return None
    t0_us = samples[0]["time_us"]
    t_s = [(s["time_us"] - t0_us) / 1e6 for s in samples]
    fill = [s["ring_fill"] for s in samples]
    hi = [s["ring_highwater"] for s in samples]

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.fill_between(t_s, 0, fill, color="tab:blue", alpha=0.25,
                    label="ring fill (instantaneous)")
    ax.plot(t_s, fill, color="tab:blue", linewidth=1.0)
    ax.plot(t_s, hi, color="tab:red", linewidth=1.5,
            label="ring high-water (cumulative peak)")
    if capacity > 0:
        ax.axhline(capacity, color="black", linestyle="--", linewidth=1.0,
                   label=f"ring capacity ({capacity / 1024.0:.0f} KB)")
    ax.set_xlabel("Time since first LogBufferStats sample (s)")
    ax.set_ylabel("Bytes")
    ax.set_title("Log-buffer ring fill")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    return fig


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="log_buffer", title="Log Buffer (MRAM)")
    samples = flight.records.get("LogBufferStats", [])

    if not samples:
        # Old / pre-instrumentation flight — just say so, don't error.
        result.warnings.append(
            "No LogBufferStats frames in this log — flight predates the "
            "LOG_BUFFER_STATS_MSG (0xE2) instrumentation. Re-fly with current "
            "firmware to capture ring high-water data."
        )
        return result

    # All samples should report the same ring_size (set once at begin()).  If
    # they differ, surface a warning — likely cause is a parser misalignment.
    capacities = {s["ring_size"] for s in samples}
    if len(capacities) != 1:
        result.warnings.append(
            f"Inconsistent ring_size across {len(samples)} samples: {sorted(capacities)}. "
            "Suspect parser corruption or mid-flight config change."
        )
    capacity = max(capacities) if capacities else 0

    # ring_highwater is monotonic since boot, so the *max* over samples is the
    # session peak.  ring_overruns / ring_drop_oldest_bytes likewise.
    peak_fill = max(s["ring_highwater"] for s in samples)
    final_bad_sof = samples[-1]["ring_bad_sof_clears"]

    # Counter deltas, split at the FIRST inter-sample interval.  The stats
    # emitter starts with the log session, so samples[0]→samples[1] straddles
    # logging activation: its delta is dominated by normal PRE-activation pad
    # turnover (the rolling prelaunch window drop-oldests on every push while
    # the ring sits at the pad cap — by design, not data loss).  Only deltas
    # AFTER samples[1] are unambiguous in-flight events: the push path rejects
    # the incoming frame when logging is active, so a post-activation overrun
    # means live flight frames were actually discarded.
    first = samples[0]
    pivot = samples[1] if len(samples) > 1 else samples[-1]
    last = samples[-1]
    overruns_activation = max(0, pivot["ring_overruns"] - first["ring_overruns"])
    drop_activation = max(0, pivot["ring_drop_oldest_bytes"] - first["ring_drop_oldest_bytes"])
    overruns_inflight = max(0, last["ring_overruns"] - pivot["ring_overruns"])
    drop_inflight = max(0, last["ring_drop_oldest_bytes"] - pivot["ring_drop_oldest_bytes"])

    pct = _peak_fill_pct(peak_fill, capacity) or 0.0

    result.metrics = {
        "samples":                       len(samples),
        "ring_capacity_bytes":           capacity,
        "ring_capacity_kb":              round(capacity / 1024.0, 1),
        "peak_fill_bytes":               peak_fill,
        "peak_fill_kb":                  round(peak_fill / 1024.0, 1),
        "peak_fill_pct":                 round(pct, 1),
        # Pad cap is ring_size/2; activateLogging() raises the cap to the full
        # ring so the reserved half absorbs the launch drain.
        "overruns_activation_interval":  overruns_activation,
        "drop_oldest_B_activation_interval": drop_activation,
        "overruns_in_flight":            overruns_inflight,
        "drop_oldest_B_in_flight":       drop_inflight,
        "ring_bad_sof_clears_final":     final_bad_sof,
    }

    if overruns_inflight > 0:
        result.warnings.append(
            f"{overruns_inflight} ring overrun(s) AFTER activation — the push "
            "path rejects incoming frames while logging, so live flight frames "
            "were discarded (flush stalled or sustained inflow burst)."
        )
    if drop_inflight > 0:
        result.warnings.append(
            f"{drop_inflight:,} bytes drop-oldested AFTER activation — "
            "unexpected: drop-oldest should only run pre-activation. Suspect "
            "the log session (re)started mid-flight."
        )
    if pct >= 90.0:
        result.warnings.append(
            f"Ring peak fill {pct:.1f}% — within 10% of capacity. "
            "Smaller chip would be risky."
        )
    elif pct >= 70.0:
        result.warnings.append(
            f"Ring peak fill {pct:.1f}% — moderate headroom. "
            "Halving the chip would leave little margin."
        )

    result.text = _viability_text(peak_fill, capacity)
    fig = _plot_ring_fill(samples, capacity)
    if fig is not None:
        result.figures.append(fig)

    return result
