#!/usr/bin/env python3
"""Analyze sensor update rates from TinkerRocket binary flight log."""

import struct
import sys
from collections import defaultdict

SOF = b'\xAA\x55\xAA\x55'

MSG_TYPES = {
    0xA0: ("OUT_STATUS_QUERY", 16),
    0xA1: ("GNSS", 42),
    0xA2: ("ISM6HG256", 22),
    0xA3: ("BMP585", 12),
    0xA4: ("MMC5983MA", 16),
    0xA5: ("NON_SENSOR", 42),
    0xA6: ("POWER", 10),
    0xA7: ("START_LOGGING", 0),
    0xA8: ("END_FLIGHT", 0),
    0xF1: ("LORA", 57),
}

def crc16(data):
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8001) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def parse_file(path):
    with open(path, 'rb') as f:
        data = f.read()

    print(f"File: {path}")
    print(f"File size: {len(data)} bytes")
    print()

    counts = defaultdict(int)
    crc_errors = 0
    pos = 0

    # Store timestamps (us) by type for sensors that have them
    timestamps = defaultdict(list)

    while pos < len(data) - 8:
        idx = data.find(SOF, pos)
        if idx < 0:
            break
        if idx + 6 > len(data):
            break

        msg_type = data[idx + 4]
        msg_len = data[idx + 5]
        frame_end = idx + 6 + msg_len + 2

        if frame_end > len(data):
            pos = idx + 1
            continue

        crc_data = data[idx+4 : idx+6+msg_len]
        crc_hi = data[idx+6+msg_len]
        crc_lo = data[idx+6+msg_len+1]
        expected_crc = (crc_hi << 8) | crc_lo
        actual_crc = crc16(crc_data)

        if actual_crc != expected_crc:
            crc_errors += 1
            pos = idx + 1
            continue

        payload = data[idx+6 : idx+6+msg_len]
        type_name = MSG_TYPES.get(msg_type, (f"UNK_0x{msg_type:02X}", None))[0]
        counts[type_name] += 1

        # Extract timestamp from first 4 bytes (uint32 microseconds)
        if msg_type in (0xA2, 0xA3, 0xA4, 0xA5, 0xA6) and msg_len >= 4:
            ts_us = struct.unpack_from('<I', payload, 0)[0]
            timestamps[msg_type].append(ts_us)

        pos = frame_end

    # === MESSAGE COUNTS ===
    print("=" * 60)
    print("MESSAGE COUNTS")
    print("=" * 60)
    total = sum(counts.values())
    for name in sorted(counts.keys()):
        print(f"  {name:20s}: {counts[name]:>8d}")
    print(f"  {'TOTAL':20s}: {total:>8d}")
    print(f"  {'CRC errors':20s}: {crc_errors:>8d}")
    print()

    # === RATE ANALYSIS FOR EACH SENSOR TYPE ===
    type_names = {
        0xA2: "ISM6HG256 (IMU)",
        0xA3: "BMP585 (Baro)",
        0xA4: "MMC5983MA (Mag)",
        0xA5: "NON_SENSOR (EKF)",
        0xA6: "POWER",
    }

    print("=" * 60)
    print("SENSOR RATE ANALYSIS  (timestamps are uint32 microseconds)")
    print("=" * 60)

    for type_code in [0xA2, 0xA3, 0xA4, 0xA5, 0xA6]:
        ts_list = timestamps.get(type_code, [])
        name = type_names[type_code]
        if len(ts_list) < 2:
            print(f"\n  {name}: insufficient data ({len(ts_list)} msgs)")
            continue

        first_ts = ts_list[0]
        last_ts = ts_list[-1]
        duration_us = last_ts - first_ts
        duration_s = duration_us / 1e6
        n = len(ts_list)

        if duration_s > 0:
            rate = (n - 1) / duration_s
        else:
            rate = 0

        print(f"\n  {name}")
        print(f"    Messages:     {n}")
        print(f"    First ts:     {first_ts} us  ({first_ts/1e6:.3f} s)")
        print(f"    Last ts:      {last_ts} us  ({last_ts/1e6:.3f} s)")
        print(f"    Duration:     {duration_s:.3f} s")
        print(f"    Rate:         {rate:.1f} Hz")

    # === DETAILED IMU (ISM6HG256) GAP ANALYSIS ===
    imu_ts = timestamps.get(0xA2, [])
    if len(imu_ts) > 1:
        print()
        print("=" * 60)
        print("ISM6HG256 (IMU) DETAILED GAP ANALYSIS")
        print("=" * 60)

        gaps_us = []
        for i in range(len(imu_ts) - 1):
            gap = imu_ts[i+1] - imu_ts[i]
            gaps_us.append(gap)

        avg_gap = sum(gaps_us) / len(gaps_us)
        sorted_gaps = sorted(gaps_us)
        median_gap = sorted_gaps[len(sorted_gaps) // 2]
        max_gap = max(gaps_us)
        min_gap = min(gaps_us)
        p95 = sorted_gaps[int(0.95 * len(sorted_gaps))]
        p99 = sorted_gaps[int(0.99 * len(sorted_gaps))]
        p999 = sorted_gaps[int(0.999 * len(sorted_gaps))]

        print(f"\n  Gap statistics (microseconds):")
        print(f"    Count:        {len(gaps_us)}")
        print(f"    Min gap:      {min_gap} us")
        print(f"    Max gap:      {max_gap} us")
        print(f"    Median gap:   {median_gap} us")
        print(f"    Mean gap:     {avg_gap:.1f} us")
        print(f"    P95 gap:      {p95} us")
        print(f"    P99 gap:      {p99} us")
        print(f"    P99.9 gap:    {p999} us")

        # Expected gap at 1 kHz = 1000 us
        expected_gap_us = 1000
        print(f"\n    Expected gap at 1 kHz: {expected_gap_us} us")

        # Histogram: bin by 50us increments up to 2000, then larger bins
        print(f"\n  Gap histogram (us):")
        bins = list(range(0, 1201, 50)) + [1500, 2000, 5000, 10000, 50000, 100000]
        bin_counts = [0] * (len(bins))
        for g in gaps_us:
            placed = False
            for i in range(len(bins) - 1):
                if bins[i] <= g < bins[i+1]:
                    bin_counts[i] += 1
                    placed = True
                    break
            if not placed:
                bin_counts[-1] += 1

        for i in range(len(bins)):
            if bin_counts[i] == 0:
                continue
            if i < len(bins) - 1:
                label = f"    [{bins[i]:>6d}, {bins[i+1]:>6d})"
            else:
                label = f"    [{bins[i]:>6d},    inf)"

            pct = 100.0 * bin_counts[i] / len(gaps_us)
            bar = '#' * min(50, int(pct * 0.5))
            print(f"{label} us: {bin_counts[i]:>7d} ({pct:5.1f}%) {bar}")

        # Key thresholds
        gaps_gt_1000 = sum(1 for g in gaps_us if g > 1000)
        gaps_gt_1500 = sum(1 for g in gaps_us if g > 1500)
        gaps_gt_2000 = sum(1 for g in gaps_us if g > 2000)
        gaps_gt_5000 = sum(1 for g in gaps_us if g > 5000)
        gaps_gt_10000 = sum(1 for g in gaps_us if g > 10000)

        print(f"\n  Threshold analysis:")
        print(f"    Gaps > 1000 us (1ms):   {gaps_gt_1000:>6d} ({100.0*gaps_gt_1000/len(gaps_us):.2f}%)")
        print(f"    Gaps > 1500 us (1.5ms): {gaps_gt_1500:>6d} ({100.0*gaps_gt_1500/len(gaps_us):.2f}%)")
        print(f"    Gaps > 2000 us (2ms):   {gaps_gt_2000:>6d} ({100.0*gaps_gt_2000/len(gaps_us):.2f}%)")
        print(f"    Gaps > 5000 us (5ms):   {gaps_gt_5000:>6d} ({100.0*gaps_gt_5000/len(gaps_us):.2f}%)")
        print(f"    Gaps > 10000 us (10ms): {gaps_gt_10000:>6d} ({100.0*gaps_gt_10000/len(gaps_us):.2f}%)")

        # Show worst gaps
        if max_gap > 2000:
            worst = sorted(enumerate(gaps_us), key=lambda x: -x[1])[:15]
            print(f"\n  Top 15 worst gaps:")
            for idx, gap in worst:
                print(f"    Sample {idx:>6d} at ts={imu_ts[idx]} us ({imu_ts[idx]/1e6:.3f}s): gap = {gap} us ({gap/1000:.1f} ms)")

        # Messages sharing same us timestamp
        msgs_per_ts = defaultdict(int)
        for ts in imu_ts:
            msgs_per_ts[ts] += 1
        dupes = {ts: c for ts, c in msgs_per_ts.items() if c > 1}
        if dupes:
            print(f"\n  Duplicate timestamps: {len(dupes)} timestamps have >1 message")
            top_dupes = sorted(dupes.items(), key=lambda x: -x[1])[:10]
            for ts, c in top_dupes:
                print(f"    ts={ts} us: {c} messages")
        else:
            print(f"\n  All {len(msgs_per_ts)} timestamps are unique (no duplicates)")

    # === BMP585 GAP ANALYSIS (brief) ===
    baro_ts = timestamps.get(0xA3, [])
    if len(baro_ts) > 1:
        print()
        print("=" * 60)
        print("BMP585 (Baro) GAP ANALYSIS")
        print("=" * 60)
        gaps = [baro_ts[i+1] - baro_ts[i] for i in range(len(baro_ts)-1)]
        print(f"  Min gap:    {min(gaps)} us ({min(gaps)/1000:.1f} ms)")
        print(f"  Max gap:    {max(gaps)} us ({max(gaps)/1000:.1f} ms)")
        print(f"  Median gap: {sorted(gaps)[len(gaps)//2]} us ({sorted(gaps)[len(gaps)//2]/1000:.1f} ms)")
        print(f"  Mean gap:   {sum(gaps)/len(gaps):.0f} us ({sum(gaps)/len(gaps)/1000:.1f} ms)")

    # === NON_SENSOR GAP ANALYSIS ===
    ns_ts = timestamps.get(0xA5, [])
    if len(ns_ts) > 1:
        print()
        print("=" * 60)
        print("NON_SENSOR (EKF) GAP ANALYSIS")
        print("=" * 60)
        gaps = [ns_ts[i+1] - ns_ts[i] for i in range(len(ns_ts)-1)]
        sorted_g = sorted(gaps)
        print(f"  Min gap:    {min(gaps)} us ({min(gaps)/1000:.1f} ms)")
        print(f"  Max gap:    {max(gaps)} us ({max(gaps)/1000:.1f} ms)")
        print(f"  Median gap: {sorted_g[len(sorted_g)//2]} us ({sorted_g[len(sorted_g)//2]/1000:.1f} ms)")
        print(f"  Mean gap:   {sum(gaps)/len(gaps):.0f} us ({sum(gaps)/len(gaps)/1000:.1f} ms)")

        dur_s = (ns_ts[-1] - ns_ts[0]) / 1e6
        rate = (len(ns_ts) - 1) / dur_s if dur_s > 0 else 0
        print(f"  Rate:       {rate:.1f} Hz")

    # === COMPARISON SUMMARY ===
    print()
    print("=" * 60)
    print("SUMMARY vs BASELINE (968 Hz)")
    print("=" * 60)

    imu_ts_list = timestamps.get(0xA2, [])
    if len(imu_ts_list) > 1:
        dur = (imu_ts_list[-1] - imu_ts_list[0]) / 1e6
        rate = (len(imu_ts_list) - 1) / dur if dur > 0 else 0
        improvement = rate - 968.0
        pct_improvement = 100.0 * improvement / 968.0

        print(f"  ISM6HG256 rate:      {rate:.1f} Hz")
        print(f"  Baseline rate:       968.0 Hz")
        print(f"  Delta:               {improvement:+.1f} Hz ({pct_improvement:+.1f}%)")
        print()
        if rate >= 1000:
            print(f"  --> TARGET MET: achieved >= 1 kHz !")
        elif rate > 968:
            print(f"  --> IMPROVED over baseline")
        else:
            print(f"  --> NO IMPROVEMENT or regression")

    baro_ts_list = timestamps.get(0xA3, [])
    if len(baro_ts_list) > 1:
        dur = (baro_ts_list[-1] - baro_ts_list[0]) / 1e6
        rate = (len(baro_ts_list) - 1) / dur if dur > 0 else 0
        print(f"  BMP585 rate:         {rate:.1f} Hz")

    ns_ts_list = timestamps.get(0xA5, [])
    if len(ns_ts_list) > 1:
        dur = (ns_ts_list[-1] - ns_ts_list[0]) / 1e6
        rate = (len(ns_ts_list) - 1) / dur if dur > 0 else 0
        print(f"  NON_SENSOR rate:     {rate:.1f} Hz")


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else '/Users/christianpedersen/Downloads/flight_20210307_000047.bin'
    parse_file(path)
