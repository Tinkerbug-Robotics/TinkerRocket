#!/usr/bin/env python3
"""Quick analysis of timestamps in a binary log to find stale data gaps."""
import struct, sys

SYNC = b'\xAA\x55\xAA\x55'

def _build_crc16_table(poly=0x8001):
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000: crc = (crc << 1) ^ poly
            else: crc = crc << 1
            crc &= 0xFFFF
        table.append(crc)
    return table

_CRC16_TABLE = _build_crc16_table()

def crc16(data, init=0x0000):
    crc = init
    for byte in data:
        crc = _CRC16_TABLE[((crc >> 8) ^ byte) & 0xFF] ^ ((crc << 8) & 0xFFFF)
    return crc

MSG_NAMES = {0xA0: "QUERY", 0xA1: "GNSS", 0xA2: "ISM6", 0xA3: "BMP",
             0xA4: "MMC", 0xA5: "NS", 0xA6: "PWR", 0xA7: "START",
             0xA8: "END", 0xF1: "LORA"}

filepath = sys.argv[1] if len(sys.argv) > 1 else "/Users/christianpedersen/Downloads/flight_20260325_182550.bin"
with open(filepath, 'rb') as f:
    data = f.read()

timestamps = []  # (file_offset, frame_idx, type, time_us)
idx = 0
frame_num = 0

while idx < len(data) - 8:
    pos = data.find(SYNC, idx)
    if pos < 0:
        break
    if pos + 8 > len(data):
        break
    msg_type = data[pos + 4]
    payload_len = data[pos + 5]
    frame_end = pos + 6 + payload_len + 2
    if frame_end > len(data):
        idx = pos + 1
        continue

    crc_payload = data[pos+4 : pos+6+payload_len]
    crc_stored = struct.unpack('>H', data[pos+6+payload_len : frame_end])[0]
    if crc16(crc_payload) != crc_stored:
        idx = pos + 1
        continue

    payload = data[pos+6 : pos+6+payload_len]

    # Extract time_us (first 4 bytes of payload for sensor msgs)
    time_us = None
    if msg_type in (0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6) and payload_len >= 4:
        time_us = struct.unpack('<I', payload[:4])[0]

    name = MSG_NAMES.get(msg_type, f"0x{msg_type:02X}")
    timestamps.append((pos, frame_num, name, time_us))
    frame_num += 1
    idx = frame_end

# Find time gaps
print(f"Total frames: {len(timestamps)}")
print(f"\nFirst 30 frames:")
for i in range(min(30, len(timestamps))):
    off, fn, name, t = timestamps[i]
    t_s = f"{t/1e6:.6f}s" if t is not None else "N/A"
    print(f"  #{fn:5d}  offset={off:8d}  {name:6s}  time={t_s}")

# Find the big gap
prev_t = None
print(f"\nLarge time gaps (>1s):")
for i, (off, fn, name, t) in enumerate(timestamps):
    if t is not None and prev_t is not None:
        gap = (t - prev_t) / 1e6
        if abs(gap) > 1.0:
            print(f"  Gap of {gap:+.3f}s between frame #{fn-1} and #{fn} (offset {off})")
            # Show surrounding frames
            start = max(0, i-3)
            end = min(len(timestamps), i+4)
            for j in range(start, end):
                o2, f2, n2, t2 = timestamps[j]
                t2_s = f"{t2/1e6:.6f}s" if t2 is not None else "N/A"
                marker = " <<< GAP" if j == i else ""
                print(f"    #{f2:5d}  {n2:6s}  time={t2_s}{marker}")
    if t is not None:
        prev_t = t

# Count frames in each time region
if timestamps:
    all_times = [t for _, _, _, t in timestamps if t is not None]
    if all_times:
        min_t = min(all_times) / 1e6
        max_t = max(all_times) / 1e6
        print(f"\nTime range: {min_t:.3f}s to {max_t:.3f}s (span: {max_t - min_t:.3f}s)")

        # Find clusters
        sorted_times = sorted(all_times)
        clusters = [[sorted_times[0]]]
        for t in sorted_times[1:]:
            if (t - clusters[-1][-1]) / 1e6 > 5.0:  # 5s gap = new cluster
                clusters.append([])
            clusters[-1].append(t)

        print(f"\nFound {len(clusters)} time cluster(s):")
        for ci, c in enumerate(clusters):
            t0 = c[0] / 1e6
            t1 = c[-1] / 1e6
            print(f"  Cluster {ci}: {len(c)} samples, t=[{t0:.3f}, {t1:.3f}]s (span: {t1-t0:.3f}s)")
