#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_gaps_post_fix.py

Post-fix gap analysis for TinkerRocket Mini binary log files.
Parses all frames, validates CRC, and reports inter-sample gap statistics
for ISM6HG256, BMP585, and MMC5983MA sensors.

Frame format: [0xAA][0x55][0xAA][0x55] + type(1) + length(1) + payload(N) + CRC16(2)
CRC-16: poly=0x8001, init=0x0000, no reflection, big-endian CRC bytes
CRC computed over [type, length, payload]

Pre-fix baseline (previous recording):
    ISM6HG256: 327 gaps >10ms at ~80ms intervals

Usage:
    python analyze_gaps_post_fix.py
"""

import struct
import sys
import os
import statistics

# ---- Configuration ----
BINARY_FILE = "/Users/christianpedersen/Downloads/flight_20260309_233258.bin"

# ---- Frame format constants ----
SYNC = b'\xAA\x55\xAA\x55'

# Message type IDs (from plot_flight_data_mini.py / RocketComputerTypes.h)
MSG_OUT_STATUS_QUERY = 0xA0
MSG_GNSS             = 0xA1
MSG_ISM6HG256        = 0xA2
MSG_BMP585           = 0xA3
MSG_MMC5983MA        = 0xA4
MSG_NON_SENSOR       = 0xA5
MSG_POWER            = 0xA6
MSG_START_LOGGING    = 0xA7
MSG_END_FLIGHT       = 0xA8
MSG_LORA             = 0xF1

MSG_NAMES = {
    MSG_OUT_STATUS_QUERY: "OUT_STATUS_QUERY",
    MSG_GNSS:             "GNSS",
    MSG_ISM6HG256:        "ISM6HG256",
    MSG_BMP585:           "BMP585",
    MSG_MMC5983MA:        "MMC5983MA",
    MSG_NON_SENSOR:       "NonSensor",
    MSG_POWER:            "POWER",
    MSG_START_LOGGING:    "StartLogging",
    MSG_END_FLIGHT:       "EndFlight",
    MSG_LORA:             "LoRa",
}

# Expected payload sizes
MSG_EXPECTED_LEN = {
    MSG_ISM6HG256:  22,
    MSG_BMP585:     12,
    MSG_MMC5983MA:  16,
    MSG_GNSS:       42,
    MSG_NON_SENSOR: 40,
    MSG_POWER:      10,
    MSG_OUT_STATUS_QUERY: 16,
    MSG_LORA:       49,
}

# Gap analysis threshold in microseconds
GAP_THRESHOLD_US = 10_000  # 10 ms


# ---- CRC-16 ----
def _build_crc16_table(poly=0x8001):
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ poly
            else:
                crc = crc << 1
            crc &= 0xFFFF
        table.append(crc)
    return table

_CRC16_TABLE = _build_crc16_table()

def crc16(data: bytes, init=0x0000) -> int:
    """CRC-16: poly=0x8001, init=0x0000, no reflection. Table-accelerated."""
    crc = init
    for byte in data:
        crc = _CRC16_TABLE[((crc >> 8) ^ byte) & 0xFF] ^ ((crc << 8) & 0xFFFF)
    return crc


# ---- Binary parser ----
def parse_frames(filepath):
    """
    Parse all frames from the binary file.

    Returns:
        frames: list of dicts with keys: type, payload, crc_ok, offset
        stats: dict with parsing statistics
    """
    with open(filepath, 'rb') as f:
        data = f.read()

    file_size = len(data)
    frames = []
    type_counts = {}
    good_crc = 0
    bad_crc = 0

    pos = 0
    while pos < file_size - 8:
        idx = data.find(SYNC, pos)
        if idx == -1:
            break
        pos = idx + 4  # skip past sync

        if pos + 2 > file_size:
            break

        msg_type = data[pos]
        msg_len = data[pos + 1]

        # Check we have enough data for payload + CRC
        if pos + 2 + msg_len + 2 > file_size:
            break

        payload = data[pos + 2: pos + 2 + msg_len]
        crc_bytes = data[pos + 2 + msg_len: pos + 2 + msg_len + 2]
        crc_received = (crc_bytes[0] << 8) | crc_bytes[1]

        # CRC over type + length + payload
        crc_data = bytes([msg_type, msg_len]) + payload
        crc_computed = crc16(crc_data)

        crc_ok = (crc_received == crc_computed)
        if crc_ok:
            good_crc += 1
        else:
            bad_crc += 1

        type_name = MSG_NAMES.get(msg_type, f"0x{msg_type:02X}")
        type_counts[type_name] = type_counts.get(type_name, 0) + 1

        frames.append({
            'type': msg_type,
            'type_name': type_name,
            'payload': payload,
            'length': msg_len,
            'crc_ok': crc_ok,
            'offset': idx,
        })

        pos = pos + 2 + msg_len + 2  # advance past this frame

    total_frames = good_crc + bad_crc
    stats = {
        'file_size': file_size,
        'total_frames': total_frames,
        'good_crc': good_crc,
        'bad_crc': bad_crc,
        'crc_pass_rate': (good_crc / total_frames * 100.0) if total_frames > 0 else 0.0,
        'type_counts': type_counts,
    }
    return frames, stats


def extract_timestamps(frames, msg_type):
    """
    Extract 4-byte little-endian timestamps from the start of each payload
    for frames of the given type. Only uses CRC-valid frames.

    Returns: sorted list of timestamps in microseconds
    """
    timestamps = []
    for f in frames:
        if f['type'] == msg_type and f['crc_ok'] and len(f['payload']) >= 4:
            ts_us = struct.unpack_from('<I', f['payload'], 0)[0]
            timestamps.append(ts_us)
    timestamps.sort()
    return timestamps


def analyze_gaps(timestamps, sensor_name, gap_threshold_us=GAP_THRESHOLD_US):
    """
    Analyze inter-sample intervals and report gap statistics.

    Returns dict with results.
    """
    if len(timestamps) < 2:
        print(f"\n  [{sensor_name}] Not enough samples to analyze ({len(timestamps)} samples)")
        return None

    # Compute inter-sample intervals
    intervals_us = []
    for i in range(1, len(timestamps)):
        dt = timestamps[i] - timestamps[i - 1]
        intervals_us.append(dt)

    intervals_ms = [dt / 1000.0 for dt in intervals_us]

    # Basic statistics
    mean_interval = statistics.mean(intervals_ms)
    median_interval = statistics.median(intervals_ms)
    min_interval = min(intervals_ms)
    max_interval = max(intervals_ms)
    stdev_interval = statistics.stdev(intervals_ms) if len(intervals_ms) > 1 else 0.0

    # Nominal rate estimation
    nominal_ms = median_interval
    nominal_hz = 1000.0 / nominal_ms if nominal_ms > 0 else 0.0

    # Find gaps exceeding threshold
    gap_threshold_ms = gap_threshold_us / 1000.0
    gaps = []
    for i, dt_ms in enumerate(intervals_ms):
        if dt_ms > gap_threshold_ms:
            gaps.append({
                'index': i,
                'size_ms': dt_ms,
                'timestamp_before_us': timestamps[i],
                'timestamp_after_us': timestamps[i + 1],
            })

    # Gap spacing analysis
    gap_spacings_ms = []
    if len(gaps) >= 2:
        for i in range(1, len(gaps)):
            spacing = (gaps[i]['timestamp_before_us'] - gaps[i - 1]['timestamp_before_us']) / 1000.0
            gap_spacings_ms.append(spacing)

    results = {
        'sensor_name': sensor_name,
        'num_samples': len(timestamps),
        'duration_s': (timestamps[-1] - timestamps[0]) / 1e6,
        'first_ts_us': timestamps[0],
        'last_ts_us': timestamps[-1],
        'mean_interval_ms': mean_interval,
        'median_interval_ms': median_interval,
        'min_interval_ms': min_interval,
        'max_interval_ms': max_interval,
        'stdev_interval_ms': stdev_interval,
        'nominal_hz': nominal_hz,
        'gap_threshold_ms': gap_threshold_ms,
        'num_gaps': len(gaps),
        'gaps': gaps,
        'gap_spacings_ms': gap_spacings_ms,
    }
    return results


def print_gap_report(results):
    """Print formatted gap analysis report for a sensor."""
    if results is None:
        return

    name = results['sensor_name']
    print(f"\n{'='*60}")
    print(f"  {name} Gap Analysis")
    print(f"{'='*60}")
    print(f"  Samples:          {results['num_samples']:,}")
    print(f"  Duration:         {results['duration_s']:.3f} s")
    print(f"  Nominal rate:     {results['nominal_hz']:.1f} Hz")
    print(f"  Interval stats:")
    print(f"    Mean:           {results['mean_interval_ms']:.3f} ms")
    print(f"    Median:         {results['median_interval_ms']:.3f} ms")
    print(f"    Min:            {results['min_interval_ms']:.3f} ms")
    print(f"    Max:            {results['max_interval_ms']:.3f} ms")
    print(f"    Stdev:          {results['stdev_interval_ms']:.3f} ms")

    gaps = results['gaps']
    print(f"\n  Gaps > {results['gap_threshold_ms']:.0f} ms:  {len(gaps)}")

    if len(gaps) > 0:
        gap_sizes = [g['size_ms'] for g in gaps]
        print(f"    Median size:    {statistics.median(gap_sizes):.3f} ms")
        print(f"    Mean size:      {statistics.mean(gap_sizes):.3f} ms")
        print(f"    Max size:       {max(gap_sizes):.3f} ms")
        print(f"    Min size:       {min(gap_sizes):.3f} ms")

        if len(gap_sizes) > 1:
            print(f"    Stdev size:     {statistics.stdev(gap_sizes):.3f} ms")

        # Show first few and last few gaps
        print(f"\n    First 5 gaps:")
        for g in gaps[:5]:
            t_s = g['timestamp_before_us'] / 1e6
            print(f"      t={t_s:10.4f}s  size={g['size_ms']:.3f}ms")
        if len(gaps) > 10:
            print(f"    ...")
            print(f"    Last 5 gaps:")
            for g in gaps[-5:]:
                t_s = g['timestamp_before_us'] / 1e6
                print(f"      t={t_s:10.4f}s  size={g['size_ms']:.3f}ms")
        elif len(gaps) > 5:
            print(f"    Last {len(gaps)-5} gaps:")
            for g in gaps[5:]:
                t_s = g['timestamp_before_us'] / 1e6
                print(f"      t={t_s:10.4f}s  size={g['size_ms']:.3f}ms")

    # Gap spacing pattern
    spacings = results['gap_spacings_ms']
    if len(spacings) > 0:
        print(f"\n  Gap Spacing Pattern (time between consecutive gaps):")
        print(f"    Count:          {len(spacings)}")
        print(f"    Median:         {statistics.median(spacings):.3f} ms")
        print(f"    Mean:           {statistics.mean(spacings):.3f} ms")
        print(f"    Min:            {min(spacings):.3f} ms")
        print(f"    Max:            {max(spacings):.3f} ms")
        if len(spacings) > 1:
            print(f"    Stdev:          {statistics.stdev(spacings):.3f} ms")

        # Distribution buckets for gap spacing
        buckets = [0, 10, 20, 50, 80, 100, 200, 500, 1000, float('inf')]
        bucket_labels = ['0-10ms', '10-20ms', '20-50ms', '50-80ms', '80-100ms',
                         '100-200ms', '200-500ms', '500-1000ms', '>1000ms']
        bucket_counts = [0] * (len(buckets) - 1)
        for s in spacings:
            for b in range(len(buckets) - 1):
                if buckets[b] <= s < buckets[b + 1]:
                    bucket_counts[b] += 1
                    break
        print(f"\n    Spacing distribution:")
        for label, count in zip(bucket_labels, bucket_counts):
            if count > 0:
                pct = count / len(spacings) * 100.0
                bar = '#' * max(1, int(pct / 2))
                print(f"      {label:>12s}: {count:5d} ({pct:5.1f}%) {bar}")

    print()


def print_interval_histogram(timestamps, sensor_name, num_bins=20):
    """Print a text histogram of inter-sample intervals."""
    if len(timestamps) < 2:
        return

    intervals_ms = []
    for i in range(1, len(timestamps)):
        dt = (timestamps[i] - timestamps[i - 1]) / 1000.0
        intervals_ms.append(dt)

    # Clip extreme outliers for histogram (show up to 99.5th percentile)
    sorted_intervals = sorted(intervals_ms)
    p995 = sorted_intervals[int(len(sorted_intervals) * 0.995)]
    clip_max = min(max(intervals_ms), p995 * 1.5)

    bin_width = clip_max / num_bins
    if bin_width <= 0:
        return

    bins = [0] * num_bins
    overflow = 0
    for dt in intervals_ms:
        b = int(dt / bin_width)
        if b >= num_bins:
            overflow += 1
        else:
            bins[b] += 1

    max_count = max(bins) if bins else 1
    bar_width = 40

    print(f"\n  Interval Histogram ({sensor_name}):")
    print(f"  {'Range (ms)':>18s}  {'Count':>7s}  Distribution")
    print(f"  {'-'*18}  {'-'*7}  {'-'*bar_width}")
    for i, count in enumerate(bins):
        lo = i * bin_width
        hi = (i + 1) * bin_width
        bar_len = int(count / max_count * bar_width) if max_count > 0 else 0
        bar = '#' * bar_len
        print(f"  {lo:8.2f} - {hi:7.2f}  {count:7d}  {bar}")
    if overflow > 0:
        print(f"  {'> ' + f'{clip_max:.2f}':>18s}  {overflow:7d}  (overflow)")


def main():
    print(f"{'='*60}")
    print(f"  Post-Fix Gap Analysis")
    print(f"  File: {BINARY_FILE}")
    print(f"{'='*60}")

    if not os.path.exists(BINARY_FILE):
        print(f"ERROR: File not found: {BINARY_FILE}")
        sys.exit(1)

    file_size = os.path.getsize(BINARY_FILE)
    print(f"\n  File size: {file_size:,} bytes ({file_size/1024:.1f} KB)")

    # Parse all frames
    print(f"\n  Parsing frames...")
    frames, stats = parse_frames(BINARY_FILE)

    # ---- Overall Summary ----
    print(f"\n{'='*60}")
    print(f"  Frame Summary")
    print(f"{'='*60}")
    print(f"  Total frames:     {stats['total_frames']:,}")
    print(f"  CRC pass:         {stats['good_crc']:,}")
    print(f"  CRC fail:         {stats['bad_crc']:,}")
    print(f"  CRC pass rate:    {stats['crc_pass_rate']:.2f}%")

    print(f"\n  Frames by type:")
    for type_name, count in sorted(stats['type_counts'].items(), key=lambda x: -x[1]):
        print(f"    {type_name:>20s}: {count:,}")

    # ---- Extract timestamps and compute total duration ----
    # Get timestamps from ISM6 (highest-rate sensor) for duration
    ism6_ts = extract_timestamps(frames, MSG_ISM6HG256)
    if len(ism6_ts) >= 2:
        total_duration_s = (ism6_ts[-1] - ism6_ts[0]) / 1e6
        print(f"\n  Total recording duration (ISM6): {total_duration_s:.3f} s")

    # ---- ISM6HG256 Gap Analysis ----
    ism6_results = analyze_gaps(ism6_ts, "ISM6HG256")
    print_gap_report(ism6_results)
    print_interval_histogram(ism6_ts, "ISM6HG256")

    # ---- BMP585 Gap Analysis ----
    bmp_ts = extract_timestamps(frames, MSG_BMP585)
    # BMP585 at lower rate; use proportional threshold
    if len(bmp_ts) >= 2:
        bmp_median_dt = statistics.median([(bmp_ts[i] - bmp_ts[i-1]) for i in range(1, len(bmp_ts))])
        bmp_threshold = max(GAP_THRESHOLD_US, bmp_median_dt * 2.5)
        bmp_results = analyze_gaps(bmp_ts, "BMP585", gap_threshold_us=bmp_threshold)
        print_gap_report(bmp_results)
        print_interval_histogram(bmp_ts, "BMP585")
    else:
        print(f"\n  [BMP585] Not enough samples ({len(bmp_ts)})")

    # ---- MMC5983MA Gap Analysis ----
    mmc_ts = extract_timestamps(frames, MSG_MMC5983MA)
    if len(mmc_ts) >= 2:
        mmc_median_dt = statistics.median([(mmc_ts[i] - mmc_ts[i-1]) for i in range(1, len(mmc_ts))])
        mmc_threshold = max(GAP_THRESHOLD_US, mmc_median_dt * 2.5)
        mmc_results = analyze_gaps(mmc_ts, "MMC5983MA", gap_threshold_us=mmc_threshold)
        print_gap_report(mmc_results)
        print_interval_histogram(mmc_ts, "MMC5983MA")
    else:
        print(f"\n  [MMC5983MA] Not enough samples ({len(mmc_ts)})")

    # ---- Comparison to Pre-Fix Baseline ----
    print(f"\n{'='*60}")
    print(f"  Comparison to Pre-Fix Baseline")
    print(f"{'='*60}")
    BASELINE_GAPS = 327
    BASELINE_SPACING_MS = 80.0

    if ism6_results:
        new_gaps = ism6_results['num_gaps']
        reduction = BASELINE_GAPS - new_gaps
        pct_reduction = (reduction / BASELINE_GAPS * 100.0) if BASELINE_GAPS > 0 else 0.0

        print(f"  ISM6HG256 gaps >10ms:")
        print(f"    Pre-fix baseline:   {BASELINE_GAPS} gaps at ~{BASELINE_SPACING_MS:.0f}ms intervals")
        print(f"    Post-fix:           {new_gaps} gaps")
        print(f"    Reduction:          {reduction} gaps ({pct_reduction:.1f}%)")

        if new_gaps > 0 and len(ism6_results['gap_spacings_ms']) > 0:
            new_median_spacing = statistics.median(ism6_results['gap_spacings_ms'])
            print(f"    Gap spacing:        {BASELINE_SPACING_MS:.0f}ms -> {new_median_spacing:.1f}ms (median)")
        elif new_gaps == 0:
            print(f"    *** ALL GAPS ELIMINATED ***")
        print()

        if new_gaps == 0:
            print(f"  RESULT: Fix is fully effective -- zero gaps detected!")
        elif pct_reduction > 90:
            print(f"  RESULT: Fix is highly effective -- {pct_reduction:.1f}% gap reduction")
        elif pct_reduction > 50:
            print(f"  RESULT: Fix is partially effective -- {pct_reduction:.1f}% gap reduction")
        elif pct_reduction > 0:
            print(f"  RESULT: Fix shows modest improvement -- {pct_reduction:.1f}% gap reduction")
        else:
            print(f"  RESULT: No improvement detected (or regression)")

    print(f"\n{'='*60}")
    print(f"  Analysis complete.")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
