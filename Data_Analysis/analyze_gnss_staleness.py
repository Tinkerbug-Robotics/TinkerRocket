#!/usr/bin/env python3
"""Analyze GNSS data from a Mini FlightComputer binary recording.

Checks for:
  - Position freeze (lat/lon unchanged across consecutive frames)
  - Timestamp staleness (second+millisecond unchanged)
  - fix_mode going to 0 (staleness detection kicking in)
  - Overall GNSS frame rate and gap distribution
"""

import struct
import sys
import numpy as np
from pathlib import Path

BINARY_FILE = Path("/Users/christianpedersen/Downloads/flight_20260310_171206.bin")

# Sync pattern
SYNC = b'\xAA\x55\xAA\x55'

# Message types
MSG_GNSS      = 0xA1
MSG_ISM6HG256 = 0xA2

# CRC-16 (poly=0x8001, init=0x0000, no reflection)
def crc16(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x8001
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def parse_frames(data: bytes):
    frames = []
    pos = 0
    while pos < len(data) - 8:
        idx = data.find(SYNC, pos)
        if idx < 0:
            break
        hdr = idx + 4
        if hdr + 2 > len(data):
            break
        msg_type = data[hdr]
        msg_len  = data[hdr + 1]
        payload_start = hdr + 2
        payload_end   = payload_start + msg_len
        crc_end       = payload_end + 2
        if crc_end > len(data):
            pos = hdr + 1
            continue
        payload  = data[payload_start:payload_end]
        crc_recv = (data[payload_end] << 8) | data[payload_end + 1]
        crc_calc = crc16(bytes([msg_type, msg_len]) + payload)
        crc_ok   = (crc_recv == crc_calc)
        frames.append({
            'type': msg_type,
            'payload': payload,
            'crc_ok': crc_ok,
            'offset': idx,
        })
        pos = crc_end
    return frames

def parse_gnss(payload):
    """Parse 42-byte GNSSData struct."""
    if len(payload) < 42:
        return None
    fields = struct.unpack_from('<I HBB BBB H BB B iii iii BB', payload, 0)
    return {
        'time_us':      fields[0],
        'year':         fields[1],
        'month':        fields[2],
        'day':          fields[3],
        'hour':         fields[4],
        'minute':       fields[5],
        'second':       fields[6],
        'milli_second': fields[7],
        'fix_mode':     fields[8],
        'num_sats':     fields[9],
        'pdop_x10':     fields[10],
        'lat_e7':       fields[11],
        'lon_e7':       fields[12],
        'alt_mm':       fields[13],
        'vel_e_mmps':   fields[14],
        'vel_n_mmps':   fields[15],
        'vel_u_mmps':   fields[16],
        'h_acc_m':      fields[17],
        'v_acc_m':      fields[18],
    }

def main():
    data = BINARY_FILE.read_bytes()
    print(f"File: {BINARY_FILE.name}  ({len(data):,} bytes)")

    frames = parse_frames(data)
    gnss_frames = [f for f in frames if f['type'] == MSG_GNSS and f['crc_ok']]
    ism6_frames = [f for f in frames if f['type'] == MSG_ISM6HG256 and f['crc_ok']]

    print(f"Total valid frames: {len([f for f in frames if f['crc_ok']])}")
    print(f"GNSS frames: {len(gnss_frames)}")
    print(f"ISM6 frames: {len(ism6_frames)}")
    print()

    if not gnss_frames:
        print("No GNSS frames found!")
        return

    # Parse all GNSS records
    records = []
    for f in gnss_frames:
        r = parse_gnss(f['payload'])
        if r:
            records.append(r)

    if not records:
        print("Failed to parse GNSS records!")
        return

    # --- Time span ---
    t0 = records[0]['time_us']
    t_last = records[-1]['time_us']
    duration_s = (t_last - t0) / 1e6
    print(f"Recording duration: {duration_s:.1f} s")
    rate = len(records) / duration_s if duration_s > 0 else 0
    print(f"GNSS frame rate: {rate:.1f} Hz")
    print()

    # --- Fix mode distribution ---
    fix_modes = [r['fix_mode'] for r in records]
    fix_labels = {0: 'No Fix', 1: 'Dead Reckoning', 2: '2D', 3: '3D',
                  4: 'GNSS+DR', 5: 'Time Only'}
    print("Fix mode distribution:")
    for mode in sorted(set(fix_modes)):
        count = fix_modes.count(mode)
        pct = 100 * count / len(records)
        label = fix_labels.get(mode, f'Unknown({mode})')
        print(f"  {label:20s}: {count:6d} ({pct:5.1f}%)")
    stale_count = fix_modes.count(0)
    print(f"  → fix_mode=0 (stale detection): {stale_count} frames")
    print()

    # --- Position freeze analysis ---
    lats = [r['lat_e7'] for r in records]
    lons = [r['lon_e7'] for r in records]
    unique_positions = len(set(zip(lats, lons)))
    print(f"Unique lat/lon positions: {unique_positions} out of {len(records)} frames")

    # Count consecutive identical positions
    max_consecutive = 1
    current_consecutive = 1
    freeze_runs = []
    for i in range(1, len(records)):
        if lats[i] == lats[i-1] and lons[i] == lons[i-1]:
            current_consecutive += 1
        else:
            if current_consecutive > 1:
                freeze_runs.append(current_consecutive)
            current_consecutive = 1
    if current_consecutive > 1:
        freeze_runs.append(current_consecutive)
    max_consecutive = max(freeze_runs) if freeze_runs else 1

    print(f"Longest position freeze: {max_consecutive} consecutive frames", end="")
    if rate > 0:
        print(f" ({max_consecutive / rate:.1f} s)")
    else:
        print()

    if freeze_runs:
        print(f"Position freeze runs >1: {len(freeze_runs)}, "
              f"median={np.median(freeze_runs):.0f}, "
              f"max={max(freeze_runs)}")
    print()

    # --- Timestamp staleness ---
    stale_runs = []
    current_stale = 0
    for i in range(1, len(records)):
        if (records[i]['second'] == records[i-1]['second'] and
            records[i]['milli_second'] == records[i-1]['milli_second']):
            current_stale += 1
        else:
            if current_stale > 0:
                stale_runs.append(current_stale)
            current_stale = 0
    if current_stale > 0:
        stale_runs.append(current_stale)

    if stale_runs:
        print(f"GNSS timestamp stale runs: {len(stale_runs)}")
        print(f"  max consecutive stale: {max(stale_runs)}")
        print(f"  median: {np.median(stale_runs):.0f}")
        long_stale = [s for s in stale_runs if s >= 5]
        print(f"  runs >= 5 (would trigger detection): {len(long_stale)}")
    else:
        print("GNSS timestamp stale runs: NONE — every read got fresh data ✓")
    print()

    # --- GNSS inter-frame gaps ---
    timestamps = np.array([r['time_us'] for r in records], dtype=np.int64)
    deltas = np.diff(timestamps)
    gaps_over_100ms = np.sum(deltas > 100_000)
    gaps_over_1s = np.sum(deltas > 1_000_000)
    print(f"GNSS inter-frame timing:")
    print(f"  median interval: {np.median(deltas)/1000:.1f} ms")
    print(f"  max interval:    {np.max(deltas)/1000:.1f} ms")
    print(f"  gaps > 100ms:    {gaps_over_100ms}")
    print(f"  gaps > 1s:       {gaps_over_1s}")
    print()

    # --- ISM6 gap check (quick) ---
    if ism6_frames:
        ism6_ts = []
        for f in ism6_frames:
            if len(f['payload']) >= 4:
                ts = struct.unpack_from('<I', f['payload'], 0)[0]
                ism6_ts.append(ts)
        if len(ism6_ts) > 1:
            ism6_deltas = np.diff(np.array(ism6_ts, dtype=np.int64))
            ism6_gaps = np.sum(ism6_deltas > 10_000)
            ism6_dur = (ism6_ts[-1] - ism6_ts[0]) / 1e6
            print(f"ISM6 quick check: {len(ism6_ts)} frames in {ism6_dur:.1f}s "
                  f"({len(ism6_ts)/ism6_dur:.0f} Hz), "
                  f"gaps>10ms={ism6_gaps}, max={np.max(ism6_deltas)/1000:.1f}ms")

    # --- Sample of GNSS data (first + last 5) ---
    print()
    print("First 5 GNSS records:")
    for r in records[:5]:
        t_rel = (r['time_us'] - t0) / 1e6
        lat = r['lat_e7'] / 1e7
        lon = r['lon_e7'] / 1e7
        alt = r['alt_mm'] / 1000
        print(f"  t={t_rel:7.2f}s  fix={r['fix_mode']} sats={r['num_sats']:2d}  "
              f"{r['hour']:02d}:{r['minute']:02d}:{r['second']:02d}.{r['milli_second']:03d}  "
              f"lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}m")

    print("Last 5 GNSS records:")
    for r in records[-5:]:
        t_rel = (r['time_us'] - t0) / 1e6
        lat = r['lat_e7'] / 1e7
        lon = r['lon_e7'] / 1e7
        alt = r['alt_mm'] / 1000
        print(f"  t={t_rel:7.2f}s  fix={r['fix_mode']} sats={r['num_sats']:2d}  "
              f"{r['hour']:02d}:{r['minute']:02d}:{r['second']:02d}.{r['milli_second']:03d}  "
              f"lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}m")

if __name__ == '__main__':
    main()
