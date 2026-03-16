#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_gaps_detailed.py

Detailed analysis of small periodic gaps in binary flight data.
Focuses on:
  - Distribution of gap sizes (histogram)
  - Periodicity analysis (FFT of gap occurrence times)
  - Correlation with known firmware periods (LoRa TX, stats, BLE, etc.)
  - Gap "families" (clustering by size)
  - Gap character changes across flight phases

Known firmware periodic operations (OutComputer):
  - LoRa TX:   2 Hz  (500 ms period) -- config::LORA_TX_RATE_HZ = 2
  - Stats/BLE: 1 Hz  (1000 ms period) -- config::STATS_PERIOD_MS = 1000
  - I2C poll:  4 Hz  (250 ms period) -- CMD_REPEAT_LIMIT = 5 at 250ms poll

Known firmware rates (FlightComputer):
  - ISM6 IMU:     1200 Hz  (833 us period)
  - BMP585 baro:   500 Hz  (2000 us period)
  - MMC5983 mag:  1000 Hz  (1000 us period)
  - GNSS:           25 Hz  (40000 us period)
  - NonSensor:     250 Hz  (4000 us period)
  - Flight loop:  1200 Hz  (833 us period)
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from collections import defaultdict
from scipy import signal

# Add the script directory to path so we can import the parser
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from plot_flight_data_mini import parse_binary_file, ROCKET_STATES, NSF_LAUNCH

# -- Configuration --
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

# Gap detection: a "small gap" is between MIN_GAP_US and MAX_SMALL_GAP_US.
# This filters out both noise and the large launch/landing gaps.
MIN_GAP_US = 3000     # 3 ms  -- minimum to be considered a gap
MAX_SMALL_GAP_US = 50000  # 50 ms -- above this is a "big" gap (launch etc.)

# Known firmware periods to check correlation against
KNOWN_PERIODS_MS = {
    "LoRa TX (2 Hz)":          500.0,
    "Stats/BLE (1 Hz)":       1000.0,
    "I2C poll (~4 Hz)":        250.0,
    "NonSensor TX (250 Hz)":     4.0,
    "BMP585 (500 Hz)":           2.0,
    "IMU (1200 Hz)":        1000.0 / 1200.0,
}

FIGURE_DPI = 150


def extract_all_timestamps(records):
    """Extract all (time_us, sensor_name) sorted by time."""
    all_ts = []
    for sensor_name, rec_list in records.items():
        for rec in rec_list:
            if "time_us" in rec:
                all_ts.append((rec["time_us"], sensor_name))
    all_ts.sort(key=lambda x: x[0])
    return all_ts


def extract_per_sensor_timestamps(records):
    """Returns dict: sensor_name -> sorted numpy array of time_us."""
    per_sensor = {}
    for sensor_name, rec_list in records.items():
        times = [rec["time_us"] for rec in rec_list if "time_us" in rec]
        if times:
            per_sensor[sensor_name] = np.array(sorted(times), dtype=np.float64)
    return per_sensor


def find_launch_time(records):
    """Find launch time from NonSensor records."""
    for rec in records.get("NonSensor", []):
        if rec.get("launch", False):
            return rec["time_us"]
    return None


def get_rocket_state_timeline(records, t0):
    """Return list of (time_us_relative, state_name) for state transitions."""
    timeline = []
    ns_recs = records.get("NonSensor", [])
    if not ns_recs:
        return timeline
    prev_state = None
    for rec in ns_recs:
        st = rec.get("rocket_state", -1)
        if st != prev_state:
            state_name = ROCKET_STATES.get(st, f"UNKNOWN({st})")
            timeline.append((rec["time_us"] - t0, state_name))
            prev_state = st
    return timeline


def classify_flight_phase(t_us_rel, state_timeline):
    """Classify a relative timestamp into a flight phase string."""
    current_state = "UNKNOWN"
    for trans_t, state_name in state_timeline:
        if t_us_rel >= trans_t:
            current_state = state_name
        else:
            break
    return current_state


def find_small_gaps(all_ts, t0, min_gap_us, max_gap_us):
    """
    Find gaps in the 'small' range: min_gap_us < delta < max_gap_us.
    Returns list of dicts with gap info.
    """
    gaps = []
    for i in range(1, len(all_ts)):
        t_prev, type_prev = all_ts[i - 1]
        t_curr, type_curr = all_ts[i]
        delta = t_curr - t_prev
        if min_gap_us < delta < max_gap_us:
            gaps.append({
                "index": i,
                "t_before_us": t_prev,
                "t_after_us": t_curr,
                "delta_us": delta,
                "t_rel_us": t_prev - t0,
                "type_before": type_prev,
                "type_after": type_curr,
            })
    return gaps


def analyze_one_flight(flight_info):
    """Full detailed gap analysis for one flight."""
    name = flight_info["name"]
    bin_path = flight_info["bin_path"]
    output_dir = flight_info["output_dir"]
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 80)
    print(f"  DETAILED GAP ANALYSIS: {name}")
    print(f"  File: {bin_path}")
    print("=" * 80)

    # Parse
    records, stats, config_data = parse_binary_file(bin_path)
    all_ts = extract_all_timestamps(records)
    per_sensor = extract_per_sensor_timestamps(records)

    if len(all_ts) < 2:
        print("  ERROR: Not enough records.")
        return None

    t0 = all_ts[0][0]
    t_end = all_ts[-1][0]
    launch_us = find_launch_time(records)
    state_timeline = get_rocket_state_timeline(records, t0)

    all_times = np.array([t for t, _ in all_ts], dtype=np.float64)
    all_deltas = np.diff(all_times)
    median_delta = np.median(all_deltas)

    print(f"\n  Total records: {len(all_ts)}")
    print(f"  Duration: {(t_end - t0) / 1e6:.3f} s")
    print(f"  Median inter-record delta: {median_delta:.1f} us ({1e6/median_delta:.1f} Hz)")
    if launch_us:
        print(f"  Launch at: T+{(launch_us - t0)/1e6:.3f}s")
    print(f"  State timeline:")
    for t_rel, sname in state_timeline:
        print(f"    T+{t_rel/1e6:.3f}s -> {sname}")

    # -- Find small gaps --
    gaps = find_small_gaps(all_ts, t0, MIN_GAP_US, MAX_SMALL_GAP_US)
    gap_deltas_ms = np.array([g["delta_us"] / 1000.0 for g in gaps])
    gap_times_rel_s = np.array([g["t_rel_us"] / 1e6 for g in gaps])

    print(f"\n  Small gaps found ({MIN_GAP_US/1000:.0f}-{MAX_SMALL_GAP_US/1000:.0f} ms): {len(gaps)}")
    if len(gaps) == 0:
        print("  No small gaps to analyze.")
        return None

    print(f"  Gap duration stats:")
    print(f"    Mean:   {np.mean(gap_deltas_ms):.3f} ms")
    print(f"    Median: {np.median(gap_deltas_ms):.3f} ms")
    print(f"    Std:    {np.std(gap_deltas_ms):.3f} ms")
    print(f"    Min:    {np.min(gap_deltas_ms):.3f} ms")
    print(f"    Max:    {np.max(gap_deltas_ms):.3f} ms")
    print(f"    P5:     {np.percentile(gap_deltas_ms, 5):.3f} ms")
    print(f"    P25:    {np.percentile(gap_deltas_ms, 25):.3f} ms")
    print(f"    P75:    {np.percentile(gap_deltas_ms, 75):.3f} ms")
    print(f"    P95:    {np.percentile(gap_deltas_ms, 95):.3f} ms")

    # -- Gap families: find clusters in gap size distribution --
    print(f"\n  --- Gap Size Distribution ---")
    # Histogram bins for gap sizes
    bin_edges_ms = np.arange(MIN_GAP_US / 1000, MAX_SMALL_GAP_US / 1000 + 0.5, 0.5)
    counts, edges = np.histogram(gap_deltas_ms, bins=bin_edges_ms)
    print(f"  Histogram (0.5 ms bins):")
    for i, c in enumerate(counts):
        if c > 0:
            print(f"    {edges[i]:.1f}-{edges[i+1]:.1f} ms: {c:4d} gaps  {'#' * min(c, 60)}")

    # Identify clusters by finding peaks in the histogram (using finer bins)
    fine_bins = np.arange(MIN_GAP_US / 1000, MAX_SMALL_GAP_US / 1000 + 0.1, 0.1)
    fine_counts, fine_edges = np.histogram(gap_deltas_ms, bins=fine_bins)
    fine_centers = (fine_edges[:-1] + fine_edges[1:]) / 2

    # Smooth the histogram slightly for peak finding
    if len(fine_counts) > 5:
        kernel = np.ones(5) / 5
        smoothed = np.convolve(fine_counts, kernel, mode='same')
    else:
        smoothed = fine_counts.astype(float)

    peaks, peak_props = signal.find_peaks(smoothed, height=max(2, np.max(smoothed) * 0.05),
                                           distance=5, prominence=2)

    print(f"\n  Gap size peaks (families):")
    families = []
    for pk in peaks:
        center = fine_centers[pk]
        height = fine_counts[pk]
        # Count gaps within +/- 1ms of this peak
        mask = np.abs(gap_deltas_ms - center) < 1.0
        family_count = np.sum(mask)
        families.append((center, family_count))
        print(f"    Peak at {center:.1f} ms: {family_count} gaps in +/-1ms window")

    # -- Periodicity analysis: inter-gap intervals --
    print(f"\n  --- Periodicity Analysis ---")
    if len(gap_times_rel_s) > 2:
        inter_gap_ms = np.diff(gap_times_rel_s) * 1000  # Convert to ms

        print(f"  Inter-gap interval stats:")
        print(f"    Mean:   {np.mean(inter_gap_ms):.1f} ms")
        print(f"    Median: {np.median(inter_gap_ms):.1f} ms")
        print(f"    Std:    {np.std(inter_gap_ms):.1f} ms")
        print(f"    Min:    {np.min(inter_gap_ms):.1f} ms")
        print(f"    Max:    {np.max(inter_gap_ms):.1f} ms")

        # Histogram of inter-gap intervals
        ig_bins = np.arange(0, min(np.max(inter_gap_ms) + 10, 2000), 5)
        ig_counts, ig_edges = np.histogram(inter_gap_ms, bins=ig_bins)
        ig_centers = (ig_edges[:-1] + ig_edges[1:]) / 2

        print(f"\n  Inter-gap interval histogram (5 ms bins, showing top entries):")
        sorted_idx = np.argsort(ig_counts)[::-1]
        for rank, idx in enumerate(sorted_idx[:15]):
            if ig_counts[idx] > 0:
                print(f"    {ig_centers[idx]:.0f} ms: {ig_counts[idx]:4d} occurrences")

        # FFT of gap occurrence signal
        # Create a binary signal: 1 at gap times, 0 elsewhere, sampled at ~1kHz
        duration_s = gap_times_rel_s[-1] - gap_times_rel_s[0]
        if duration_s > 0.1:
            fs_fft = 1000.0  # 1 kHz sampling for gap-occurrence signal
            n_samples = int(duration_s * fs_fft)
            gap_signal = np.zeros(n_samples)
            for t in gap_times_rel_s:
                idx = int((t - gap_times_rel_s[0]) * fs_fft)
                if 0 <= idx < n_samples:
                    gap_signal[idx] = 1.0

            # Compute power spectrum
            if n_samples > 64:
                freqs = np.fft.rfftfreq(n_samples, d=1.0/fs_fft)
                fft_mag = np.abs(np.fft.rfft(gap_signal - np.mean(gap_signal)))
                power = fft_mag ** 2

                # Find dominant frequencies
                freq_mask = freqs > 0.1  # Ignore DC and very low freq
                if np.any(freq_mask):
                    masked_power = power[freq_mask]
                    masked_freqs = freqs[freq_mask]
                    top_n = min(10, len(masked_power))
                    top_idx = np.argsort(masked_power)[::-1][:top_n]

                    print(f"\n  FFT dominant frequencies in gap occurrence:")
                    for rank, idx in enumerate(top_idx):
                        f = masked_freqs[idx]
                        p = masked_power[idx]
                        period_ms = 1000.0 / f if f > 0 else float('inf')
                        print(f"    #{rank+1}: {f:.2f} Hz (period = {period_ms:.1f} ms), power = {p:.1f}")

                    # Check correlation with known periods
                    print(f"\n  Correlation with known firmware periods:")
                    for pname, period_ms in KNOWN_PERIODS_MS.items():
                        expected_freq = 1000.0 / period_ms
                        if expected_freq < freqs[-1]:
                            # Find nearest FFT bin
                            bin_idx = np.argmin(np.abs(freqs - expected_freq))
                            local_power = power[bin_idx]
                            # Check power in +/-2 bins
                            lo = max(0, bin_idx - 2)
                            hi = min(len(power), bin_idx + 3)
                            peak_local = np.max(power[lo:hi])
                            # Relative to median power
                            med_power = np.median(masked_power)
                            ratio = peak_local / med_power if med_power > 0 else 0
                            match_str = "** STRONG **" if ratio > 10 else ("* moderate *" if ratio > 3 else "  weak")
                            print(f"    {pname:30s}: f={expected_freq:.2f} Hz, "
                                  f"power={peak_local:.1f}, ratio={ratio:.1f}x median  {match_str}")
    else:
        inter_gap_ms = np.array([])
        print("  Not enough gaps for periodicity analysis.")

    # -- Phase analysis: gap timing modulo known periods --
    print(f"\n  --- Phase Analysis (gap times modulo known periods) ---")
    gap_times_rel_ms = gap_times_rel_s * 1000  # ms from t0

    for pname, period_ms in [("LoRa TX 500ms", 500.0),
                              ("Stats/BLE 1000ms", 1000.0),
                              ("I2C poll 250ms", 250.0)]:
        phases = np.mod(gap_times_rel_ms, period_ms)
        phase_norm = phases / period_ms  # 0 to 1

        # Are the phases clustered or uniform?
        # Use circular statistics: Rayleigh test
        theta = 2 * np.pi * phase_norm
        R = np.sqrt(np.mean(np.cos(theta))**2 + np.mean(np.sin(theta))**2)
        mean_phase_deg = np.degrees(np.arctan2(np.mean(np.sin(theta)), np.mean(np.cos(theta)))) % 360
        mean_phase_ms = (mean_phase_deg / 360.0) * period_ms

        print(f"  {pname}:")
        print(f"    Rayleigh R = {R:.4f} (1.0 = perfectly clustered, 0.0 = uniform)")
        print(f"    Mean phase = {mean_phase_ms:.1f} ms ({mean_phase_deg:.1f} deg)")
        if R > 0.3:
            print(f"    ** SIGNIFICANT phase clustering -- gaps are locked to this period **")
        elif R > 0.1:
            print(f"    * Moderate phase clustering")
        else:
            print(f"    No significant clustering (gaps not related to this period)")

    # -- Per-flight-phase analysis --
    print(f"\n  --- Gaps by Flight Phase ---")
    phase_gaps = defaultdict(list)
    for g in gaps:
        phase = classify_flight_phase(g["t_rel_us"], state_timeline)
        phase_gaps[phase].append(g["delta_us"] / 1000.0)

    for phase_name in ["INITIALIZATION", "READY", "PRELAUNCH", "INFLIGHT", "LANDED"]:
        if phase_name in phase_gaps:
            durs = np.array(phase_gaps[phase_name])
            print(f"\n  {phase_name}: {len(durs)} gaps")
            print(f"    Mean: {np.mean(durs):.3f} ms, Median: {np.median(durs):.3f} ms, "
                  f"Std: {np.std(durs):.3f} ms")
            # Duration in this phase
            # Find phase time range
            phase_start = None
            phase_end = None
            for i, (trans_t, sn) in enumerate(state_timeline):
                if sn == phase_name:
                    phase_start = trans_t / 1e6
                    if i + 1 < len(state_timeline):
                        phase_end = state_timeline[i+1][0] / 1e6
                    else:
                        phase_end = (t_end - t0) / 1e6
                    break
            if phase_start is not None and phase_end is not None:
                phase_dur = phase_end - phase_start
                if phase_dur > 0:
                    rate = len(durs) / phase_dur
                    print(f"    Phase duration: {phase_dur:.2f}s, Gap rate: {rate:.1f} gaps/s")

    # -- Per-sensor gap analysis (which sensor pairs bracket the gaps) --
    print(f"\n  --- Sensor Pair Analysis (which sensor types bracket gaps) ---")
    pair_counts = defaultdict(int)
    pair_durations = defaultdict(list)
    for g in gaps:
        pair = f"{g['type_before']} -> {g['type_after']}"
        pair_counts[pair] += 1
        pair_durations[pair].append(g["delta_us"] / 1000.0)

    for pair, count in sorted(pair_counts.items(), key=lambda x: -x[1]):
        durs = pair_durations[pair]
        print(f"    {pair:35s}: {count:4d} gaps, "
              f"mean={np.mean(durs):.2f} ms, med={np.median(durs):.2f} ms")

    # ========================================================================
    # PLOTS
    # ========================================================================
    plot_paths = []

    # -- Plot 1: Gap size distribution --
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(f"Detailed Gap Analysis: {name}", fontsize=14, fontweight="bold")

    # Panel 1a: Histogram of gap sizes
    ax = axes[0, 0]
    ax.hist(gap_deltas_ms, bins=np.arange(MIN_GAP_US/1000, MAX_SMALL_GAP_US/1000 + 0.25, 0.25),
            color="steelblue", edgecolor="none", alpha=0.8)
    for center, count in families:
        ax.axvline(center, color="red", linestyle="--", linewidth=1, alpha=0.7)
        ax.annotate(f"{center:.1f}ms\n({count})", xy=(center, ax.get_ylim()[1] * 0.9),
                    fontsize=7, color="red", ha="center",
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.8))
    ax.set_xlabel("Gap duration (ms)")
    ax.set_ylabel("Count")
    ax.set_title("Gap Size Distribution (small gaps only)")
    ax.grid(True, alpha=0.3)

    # Panel 1b: Gap times scatter with size
    ax = axes[0, 1]
    sc = ax.scatter(gap_times_rel_s, gap_deltas_ms, s=8, alpha=0.5, c=gap_deltas_ms,
                    cmap="viridis", edgecolors="none")
    plt.colorbar(sc, ax=ax, label="Gap duration (ms)")
    if launch_us:
        ax.axvline((launch_us - t0) / 1e6, color="orange", linewidth=2, label="Launch")
    # Mark state transitions
    for trans_t, sname in state_timeline:
        ax.axvline(trans_t / 1e6, color="gray", linestyle=":", linewidth=0.5, alpha=0.5)
    ax.set_xlabel("Time from first record (s)")
    ax.set_ylabel("Gap duration (ms)")
    ax.set_title("Gap Occurrences Over Time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 1c: Inter-gap interval histogram
    ax = axes[1, 0]
    if len(inter_gap_ms) > 0:
        plot_max = min(np.percentile(inter_gap_ms, 99) + 50, 2000)
        ig_plot_bins = np.arange(0, plot_max, 5)
        ax.hist(inter_gap_ms, bins=ig_plot_bins, color="coral", edgecolor="none", alpha=0.8)
        # Mark known periods
        for pname, period in KNOWN_PERIODS_MS.items():
            if period < plot_max:
                ax.axvline(period, color="green", linestyle="--", linewidth=1.5, alpha=0.8)
                ax.annotate(pname, xy=(period, ax.get_ylim()[1] * 0.85),
                            fontsize=6, color="green", rotation=90, va="top",
                            bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.8))
    ax.set_xlabel("Inter-gap interval (ms)")
    ax.set_ylabel("Count")
    ax.set_title("Time Between Consecutive Gaps")
    ax.grid(True, alpha=0.3)

    # Panel 1d: Gap rate over time (sliding window)
    ax = axes[1, 1]
    if len(gap_times_rel_s) > 5:
        window_s = 1.0
        t_bins = np.arange(gap_times_rel_s[0], gap_times_rel_s[-1], window_s / 2)
        gap_rate = np.zeros_like(t_bins)
        for i, tb in enumerate(t_bins):
            mask = (gap_times_rel_s >= tb) & (gap_times_rel_s < tb + window_s)
            gap_rate[i] = np.sum(mask) / window_s

        ax.plot(t_bins, gap_rate, color="steelblue", linewidth=1)
        ax.fill_between(t_bins, 0, gap_rate, alpha=0.3, color="steelblue")
        if launch_us:
            ax.axvline((launch_us - t0) / 1e6, color="orange", linewidth=2, label="Launch")
        for trans_t, sname in state_timeline:
            t_s = trans_t / 1e6
            if t_s > 0:
                ax.axvline(t_s, color="gray", linestyle=":", linewidth=0.5, alpha=0.5)
                ax.annotate(sname, xy=(t_s, ax.get_ylim()[1] * 0.95), fontsize=6,
                            rotation=90, va="top", color="gray")
    ax.set_xlabel("Time from first record (s)")
    ax.set_ylabel("Gap rate (gaps/s)")
    ax.set_title("Gap Occurrence Rate Over Time (1s sliding window)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path1 = os.path.join(output_dir, "detailed_gap_distribution.png")
    fig.savefig(path1, dpi=FIGURE_DPI, bbox_inches="tight")
    plot_paths.append(path1)
    print(f"\n  Saved: {path1}")
    plt.close(fig)

    # -- Plot 2: Periodicity and phase analysis --
    fig2, axes2 = plt.subplots(2, 3, figsize=(18, 10))
    fig2.suptitle(f"Gap Periodicity & Phase Analysis: {name}", fontsize=14, fontweight="bold")

    # Row 1: Phase plots (modulo known periods)
    for col, (pname, period_ms) in enumerate([("LoRa TX (500ms)", 500.0),
                                                ("Stats/BLE (1000ms)", 1000.0),
                                                ("I2C Poll (250ms)", 250.0)]):
        ax = axes2[0, col]
        phases = np.mod(gap_times_rel_ms, period_ms)

        ax.hist(phases, bins=50, color="steelblue", edgecolor="none", alpha=0.8)
        ax.set_xlabel(f"Phase within {period_ms:.0f}ms period (ms)")
        ax.set_ylabel("Count")
        ax.set_title(f"Gap phase mod {pname}")
        ax.set_xlim(0, period_ms)

        # Compute Rayleigh R
        theta = 2 * np.pi * (phases / period_ms)
        R = np.sqrt(np.mean(np.cos(theta))**2 + np.mean(np.sin(theta))**2)
        mean_phase_ms = (np.degrees(np.arctan2(np.mean(np.sin(theta)),
                                                np.mean(np.cos(theta)))) % 360) / 360.0 * period_ms
        ax.axvline(mean_phase_ms, color="red", linestyle="--", linewidth=1.5,
                   label=f"Mean={mean_phase_ms:.0f}ms, R={R:.3f}")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # Row 2, left: FFT power spectrum
    ax = axes2[1, 0]
    if len(gap_times_rel_s) > 10:
        duration_s_fft = gap_times_rel_s[-1] - gap_times_rel_s[0]
        if duration_s_fft > 0.1:
            fs_fft = 1000.0
            n_samples = int(duration_s_fft * fs_fft)
            gap_signal_plot = np.zeros(n_samples)
            for t in gap_times_rel_s:
                idx_p = int((t - gap_times_rel_s[0]) * fs_fft)
                if 0 <= idx_p < n_samples:
                    gap_signal_plot[idx_p] = 1.0

            freqs_plot = np.fft.rfftfreq(n_samples, d=1.0/fs_fft)
            fft_mag_plot = np.abs(np.fft.rfft(gap_signal_plot - np.mean(gap_signal_plot)))
            power_plot = fft_mag_plot ** 2

            # Plot up to 20 Hz (periods down to 50ms)
            freq_mask_plot = (freqs_plot > 0.1) & (freqs_plot < 20)
            if np.any(freq_mask_plot):
                ax.plot(freqs_plot[freq_mask_plot], power_plot[freq_mask_plot],
                        color="steelblue", linewidth=0.5)

                # Mark known frequencies
                for pk_name, pk_period in KNOWN_PERIODS_MS.items():
                    pk_freq = 1000.0 / pk_period
                    if 0.1 < pk_freq < 20:
                        ax.axvline(pk_freq, color="red", linestyle="--", linewidth=1, alpha=0.7)
                        ax.annotate(pk_name, xy=(pk_freq, ax.get_ylim()[1] * 0.8),
                                    fontsize=5, color="red", rotation=90, va="top")

    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Power")
    ax.set_title("FFT of Gap Occurrence Signal (0-20 Hz)")
    ax.grid(True, alpha=0.3)

    # Row 2, middle: Autocorrelation of inter-gap intervals
    ax = axes2[1, 1]
    if len(inter_gap_ms) > 20:
        # Compute autocorrelation of the gap occurrence signal
        max_lag = min(500, len(inter_gap_ms) // 2)
        ig_centered = inter_gap_ms - np.mean(inter_gap_ms)
        autocorr = np.correlate(ig_centered[:max_lag*2], ig_centered[:max_lag*2], mode='full')
        autocorr = autocorr[len(autocorr)//2:]  # Take positive lags only
        autocorr = autocorr / autocorr[0] if autocorr[0] > 0 else autocorr  # Normalize

        lags = np.arange(len(autocorr))
        ax.plot(lags, autocorr, color="steelblue", linewidth=0.8)
        ax.axhline(0, color="gray", linewidth=0.5)
        ax.axhline(2.0 / np.sqrt(len(inter_gap_ms)), color="red", linestyle=":", alpha=0.5)
        ax.axhline(-2.0 / np.sqrt(len(inter_gap_ms)), color="red", linestyle=":", alpha=0.5)
        ax.set_xlabel("Lag (gap index)")
        ax.set_ylabel("Autocorrelation")
        ax.set_title("Autocorrelation of Inter-gap Intervals")
        ax.set_xlim(0, min(100, max_lag))
    ax.grid(True, alpha=0.3)

    # Row 2, right: Gap duration vs flight phase (boxplot)
    ax = axes2[1, 2]
    phase_order = ["INITIALIZATION", "READY", "PRELAUNCH", "INFLIGHT", "LANDED"]
    phase_data = []
    phase_labels = []
    for pn in phase_order:
        if pn in phase_gaps:
            phase_data.append(phase_gaps[pn])
            phase_labels.append(f"{pn}\n(n={len(phase_gaps[pn])})")

    if phase_data:
        bp = ax.boxplot(phase_data, labels=phase_labels, patch_artist=True,
                        showfliers=True, flierprops=dict(markersize=2, alpha=0.3))
        for patch in bp['boxes']:
            patch.set_facecolor("steelblue")
            patch.set_alpha(0.6)
    ax.set_ylabel("Gap duration (ms)")
    ax.set_title("Gap Duration by Flight Phase")
    ax.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    path2 = os.path.join(output_dir, "detailed_gap_periodicity.png")
    fig2.savefig(path2, dpi=FIGURE_DPI, bbox_inches="tight")
    plot_paths.append(path2)
    print(f"  Saved: {path2}")
    plt.close(fig2)

    # -- Plot 3: Per-sensor detailed gap view --
    fig3, axes3 = plt.subplots(2, 2, figsize=(16, 12))
    fig3.suptitle(f"Per-Sensor Gap Detail: {name}", fontsize=14, fontweight="bold")

    # Panel 3a: Gap duration by sensor pair
    ax = axes3[0, 0]
    pairs_sorted = sorted(pair_counts.items(), key=lambda x: -x[1])[:12]
    if pairs_sorted:
        pair_names = [p for p, _ in pairs_sorted]
        pair_cnts = [c for _, c in pairs_sorted]
        y_pos = np.arange(len(pair_names))
        ax.barh(y_pos, pair_cnts, color="steelblue", alpha=0.8)
        ax.set_yticks(y_pos)
        ax.set_yticklabels(pair_names, fontsize=7)
        ax.set_xlabel("Number of gaps")
        ax.set_title("Gaps by Sensor Pair (before -> after)")
        ax.invert_yaxis()
    ax.grid(True, alpha=0.3, axis="x")

    # Panel 3b: Per-sensor inter-record delta analysis focusing on the gap range
    ax = axes3[0, 1]
    main_sensors = ["ISM6HG256", "BMP585", "MMC5983MA", "NonSensor"]
    colors = plt.cm.Set2(np.linspace(0, 1, len(main_sensors)))
    for i, sname in enumerate(main_sensors):
        if sname in per_sensor and len(per_sensor[sname]) > 2:
            deltas_s = np.diff(per_sensor[sname]) / 1000.0  # ms
            # Only show values in the gap range
            gap_range = (deltas_s > MIN_GAP_US / 1000) & (deltas_s < MAX_SMALL_GAP_US / 1000)
            if np.any(gap_range):
                ax.hist(deltas_s[gap_range], bins=50, alpha=0.5, label=f"{sname} ({np.sum(gap_range)})",
                        color=colors[i])

    ax.set_xlabel("Inter-record delta (ms)")
    ax.set_ylabel("Count")
    ax.set_title("Per-Sensor Delta Distribution (gap range only)")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Panel 3c: Gap timing zoom (first 5 seconds of active logging)
    ax = axes3[1, 0]
    if launch_us:
        zoom_start = (launch_us - t0) / 1e6 - 2.0  # 2s before launch
    else:
        zoom_start = 0
    zoom_end = zoom_start + 5.0
    zoom_mask = (gap_times_rel_s >= zoom_start) & (gap_times_rel_s <= zoom_end)
    if np.any(zoom_mask):
        ax.scatter(gap_times_rel_s[zoom_mask], gap_deltas_ms[zoom_mask],
                   s=30, c="red", alpha=0.7, edgecolors="darkred", linewidth=0.5,
                   zorder=10)
        # Also show all inter-record deltas in this window
        all_t_rel = (all_times[:-1] - t0) / 1e6
        all_d_ms = all_deltas / 1000.0
        window_mask = (all_t_rel >= zoom_start) & (all_t_rel <= zoom_end)
        if np.any(window_mask):
            ax.scatter(all_t_rel[window_mask], all_d_ms[window_mask],
                       s=2, c="steelblue", alpha=0.3)
        if launch_us:
            ax.axvline((launch_us - t0) / 1e6, color="orange", linewidth=2, label="Launch")
    ax.axhline(MIN_GAP_US / 1000, color="gray", linestyle=":", alpha=0.5, label=f"Gap threshold ({MIN_GAP_US/1000:.0f}ms)")
    ax.set_xlabel("Time from first record (s)")
    ax.set_ylabel("Inter-record delta (ms)")
    ax.set_title(f"Zoom: {zoom_start:.1f}s to {zoom_end:.1f}s (gaps in red)")
    ax.set_yscale("log")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Panel 3d: Cumulative gap time over flight
    ax = axes3[1, 1]
    cum_gap_ms = np.cumsum(gap_deltas_ms)
    ax.plot(gap_times_rel_s, cum_gap_ms, color="steelblue", linewidth=1.5)
    ax.set_xlabel("Time from first record (s)")
    ax.set_ylabel("Cumulative gap time (ms)")
    ax.set_title("Cumulative Time Lost to Gaps")
    if launch_us:
        ax.axvline((launch_us - t0) / 1e6, color="orange", linewidth=2, label="Launch")
    for trans_t, sname in state_timeline:
        t_s = trans_t / 1e6
        if t_s > 0:
            ax.axvline(t_s, color="gray", linestyle=":", linewidth=0.5, alpha=0.5)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path3 = os.path.join(output_dir, "detailed_gap_sensors.png")
    fig3.savefig(path3, dpi=FIGURE_DPI, bbox_inches="tight")
    plot_paths.append(path3)
    print(f"  Saved: {path3}")
    plt.close(fig3)

    # -- Plot 4: Zoomed inter-gap interval with period markers --
    fig4, axes4 = plt.subplots(2, 1, figsize=(16, 10))
    fig4.suptitle(f"Inter-Gap Interval Detail: {name}", fontsize=14, fontweight="bold")

    # Top: scatter of inter-gap intervals over time
    ax = axes4[0]
    if len(inter_gap_ms) > 0:
        ig_times = gap_times_rel_s[:-1]  # Time of first gap in each pair
        ax.scatter(ig_times, inter_gap_ms, s=6, alpha=0.4, c="steelblue", edgecolors="none")
        # Mark known periods
        for pname, period in KNOWN_PERIODS_MS.items():
            if period < 1500:
                ax.axhline(period, color="red", linestyle="--", linewidth=1, alpha=0.6)
                ax.annotate(pname, xy=(ig_times[-1], period), fontsize=7, color="red",
                            va="bottom", ha="right",
                            bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.8))
        # Also mark multiples of 250ms
        for mult in [250, 500, 750, 1000]:
            ax.axhline(mult, color="green", linestyle=":", linewidth=0.5, alpha=0.4)

        if launch_us:
            ax.axvline((launch_us - t0) / 1e6, color="orange", linewidth=2, label="Launch")
        ax.set_ylim(0, min(np.percentile(inter_gap_ms, 99) + 100, 2000))
    ax.set_xlabel("Time from first record (s)")
    ax.set_ylabel("Inter-gap interval (ms)")
    ax.set_title("Time Between Consecutive Gaps")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Bottom: histogram focused on 0-1000ms with finer bins
    ax = axes4[1]
    if len(inter_gap_ms) > 0:
        focused_ig = inter_gap_ms[inter_gap_ms < 1200]
        if len(focused_ig) > 0:
            ax.hist(focused_ig, bins=np.arange(0, 1200, 2), color="steelblue",
                    edgecolor="none", alpha=0.8)
            for pname, period in [("250ms", 250), ("500ms", 500), ("1000ms", 1000)]:
                ax.axvline(period, color="red", linestyle="--", linewidth=1.5, alpha=0.8)
                ax.annotate(pname, xy=(period, ax.get_ylim()[1] * 0.9),
                            fontsize=8, color="red", ha="center",
                            bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.8))
    ax.set_xlabel("Inter-gap interval (ms)")
    ax.set_ylabel("Count")
    ax.set_title("Inter-Gap Interval Distribution (0-1200ms, 2ms bins)")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path4 = os.path.join(output_dir, "detailed_gap_intergap.png")
    fig4.savefig(path4, dpi=FIGURE_DPI, bbox_inches="tight")
    plot_paths.append(path4)
    print(f"  Saved: {path4}")
    plt.close(fig4)

    return {
        "name": name,
        "gaps": gaps,
        "gap_deltas_ms": gap_deltas_ms,
        "gap_times_rel_s": gap_times_rel_s,
        "inter_gap_ms": inter_gap_ms,
        "families": families,
        "phase_gaps": dict(phase_gaps),
        "pair_counts": dict(pair_counts),
        "plot_paths": plot_paths,
    }


def main():
    results = []
    for flight in FLIGHTS:
        result = analyze_one_flight(flight)
        if result is not None:
            results.append(result)
        print()

    # -- Cross-flight comparison --
    if len(results) == 2:
        print("=" * 80)
        print("  CROSS-FLIGHT COMPARISON")
        print("=" * 80)
        r1, r2 = results
        print(f"\n  {r1['name']:30s}  vs  {r2['name']}")
        print(f"  Gap count:      {len(r1['gaps']):6d}         {len(r2['gaps']):6d}")
        print(f"  Mean gap (ms):  {np.mean(r1['gap_deltas_ms']):8.3f}       {np.mean(r2['gap_deltas_ms']):8.3f}")
        print(f"  Median gap (ms):{np.median(r1['gap_deltas_ms']):8.3f}       {np.median(r2['gap_deltas_ms']):8.3f}")
        if len(r1['inter_gap_ms']) > 0 and len(r2['inter_gap_ms']) > 0:
            print(f"  Mean inter-gap: {np.mean(r1['inter_gap_ms']):8.1f} ms     {np.mean(r2['inter_gap_ms']):8.1f} ms")
            print(f"  Median inter-gap:{np.median(r1['inter_gap_ms']):7.1f} ms     {np.median(r2['inter_gap_ms']):7.1f} ms")

        # Compare gap families
        print(f"\n  Gap families (peak durations):")
        print(f"    {r1['name']:30s}: {', '.join(f'{c:.1f}ms' for c, _ in r1['families'])}")
        print(f"    {r2['name']:30s}: {', '.join(f'{c:.1f}ms' for c, _ in r2['families'])}")

    print("\n" + "=" * 80)
    print("  FIRMWARE CORRELATION SUMMARY")
    print("=" * 80)
    print("""
  Known periodic operations on OutComputer (the logging ESP32):
    1. LoRa TX:      2 Hz (every 500ms) -- lora_comms.send() is blocking SPI
    2. Stats/BLE:    1 Hz (every 1000ms) -- printStats() + BLE telemetry
    3. NAND flush:   Continuous -- logger.service() flushes ring buffer to NAND via SPI
    4. I2C ingress:  Continuous -- serviceI2CIngress() reads from FlightComputer

  LoRa TX is the prime suspect for periodic gaps because:
    - LLCC68 SPI transactions block the main loop
    - send() configures registers + writes FIFO + triggers TX (~5-10ms for SF8 BW250)
    - Happens every 500ms, which creates a 2 Hz gap pattern
    - The gap duration (~8-10ms) matches typical LoRa register + FIFO SPI time

  Stats/BLE telemetry could add additional ~1-2ms stalls every 1000ms.

  The I2C ingress budget is capped at 1500us to prevent loop starvation,
  so it should not cause gaps > 1.5ms by itself.
    """)

    print("All plots saved.")
    for r in results:
        for p in r.get("plot_paths", []):
            print(f"  {p}")


if __name__ == "__main__":
    main()
