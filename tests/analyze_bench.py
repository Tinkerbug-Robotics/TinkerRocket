#!/usr/bin/env python3
"""
Unified TinkerRocket bench test analyzer.

Parses binary flight logs and/or serial monitor output to produce a
comprehensive report of sensor rates, EKF performance, I2C queue health,
and anomaly detection.

Usage:
    python3 analyze_bench.py <binary_file> [serial_file]
    python3 analyze_bench.py --serial <serial_file>
    python3 analyze_bench.py --bin <binary_file>
    python3 analyze_bench.py <binary_file> <serial_file>

Examples:
    python3 analyze_bench.py flight_20260317_102259.bin
    python3 analyze_bench.py flight_20260317_102259.bin serial_output.txt
    python3 analyze_bench.py --serial serial_output.txt
"""

import argparse
import math
import re
import struct
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ============================================================================
# Constants — keep in sync with config.h / RocketComputerTypes.h
# ============================================================================

# Target rates from config.h
TARGET_RATES = {
    "ISM6":       960,   # ISM6HG256_UPDATE_RATE
    "BMP":        500,   # BMP585_UPDATE_RATE (OSR x1/x1 ≈ 460 Hz actual)
    "MMC":        200,   # MMC5983MA_UPDATE_RATE (hw step: 200 Hz)
    "GNSS":        10,   # Realistic ceiling with 4 constellations (config asks 18)
    "NonSensor":  500,   # NON_SENSOR_UPDATE_RATE
    "Power":       -1,   # No fixed rate target
}

FLIGHT_LOOP_TARGET = 1000  # FLIGHT_LOOP_UPDATE_RATE
EKF_DECIMATION = 2         # EKF_DECIMATION

# Message types from RocketComputerTypes.h
MSG_INFO = {
    0xA0: ("StatusQuery",  16),
    0xA1: ("GNSS",         42),
    0xA2: ("ISM6",         22),
    0xA3: ("BMP",          12),
    0xA4: ("MMC",          16),
    0xA5: ("NonSensor",    42),
    0xA6: ("Power",        10),
    0xA7: ("StartLog",      0),
    0xA8: ("EndFlight",     0),
    0xF1: ("LoRa",         57),
}

# Frame framing
SOF = b'\xAA\x55\xAA\x55'
SOF_LEN = 4
HEADER_LEN = SOF_LEN + 2   # SOF + type + length
CRC_LEN = 2

# RocketState enum
ROCKET_STATES = {0: "INIT", 1: "READY", 2: "PRELAUNCH", 3: "INFLIGHT", 4: "LANDED"}


# ============================================================================
# CRC16 — polynomial 0x8001, init 0, no reversal, no XOR out
# Matches RobTillaart CRC library defaults used by TR_I2C_Interface
# ============================================================================

def crc16(data: bytes) -> int:
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8001) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ============================================================================
# Data classes
# ============================================================================

@dataclass
class BinaryStats:
    """Statistics extracted from binary flight log."""
    file_path: str = ""
    file_size: int = 0
    valid_frames: int = 0
    crc_errors: int = 0
    counts: dict = field(default_factory=dict)          # name -> count
    timestamps: dict = field(default_factory=dict)       # name -> [uint32 us, ...]
    parsed_frames: list = field(default_factory=list)    # [(name, parsed_dict), ...]


@dataclass
class SerialTimingWindow:
    """One [TIMING] line from serial output."""
    wall_time: str = ""         # HH:MM:SS.mmm from serial monitor
    ekf_avg_us: int = 0
    ekf_max_us: int = 0
    ekf_count: int = 0
    loop_count: int = 0
    ekf_running: bool = True


@dataclass
class SerialGapDiag:
    """One set of [GAP DIAG] lines from serial output."""
    wall_time: str = ""
    iter_max_us: int = 0
    gnss_max_us: int = 0
    bmp_max_us: int = 0
    mmc_max_us: int = 0
    ism6_max_us: int = 0
    gaps_gt_10ms: int = 0
    worst_gap_us: int = 0
    gnss_calls: int = 0
    gnss_gt_1ms: int = 0
    gnss_gt_5ms: int = 0
    gnss_gt_10ms: int = 0
    enqueue_ok: int = 0
    enqueue_drop: int = 0
    tx_ok: int = 0
    tx_fail: int = 0
    last_err: int = 0
    q_free: int = 0
    out_ready_max_us: int = 0
    query_ok: int = 0
    query_fail: int = 0


@dataclass
class SerialEkfDiag:
    """One set of [EKF DIAG] lines from serial output."""
    wall_time: str = ""
    quat_roll: float = 0.0
    euler_roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    cos2p: float = 0.0
    mag_ut: float = 0.0
    mag_status: str = ""
    gyro_bias: tuple = (0.0, 0.0, 0.0)


@dataclass
class SerialStats:
    """All statistics extracted from serial monitor output."""
    file_path: str = ""
    timing_windows: list = field(default_factory=list)   # [SerialTimingWindow]
    gap_diags: list = field(default_factory=list)         # [SerialGapDiag]
    ekf_diags: list = field(default_factory=list)         # [SerialEkfDiag]


# ============================================================================
# Binary log parser
# ============================================================================

def parse_binary(path: str) -> BinaryStats:
    """Parse a TinkerRocket binary flight log file."""
    data = Path(path).read_bytes()
    stats = BinaryStats(file_path=path, file_size=len(data))

    counts = defaultdict(int)
    timestamps = defaultdict(list)
    parsed = []

    pos = 0
    while pos < len(data) - (HEADER_LEN + CRC_LEN):
        idx = data.find(SOF, pos)
        if idx < 0 or idx + HEADER_LEN > len(data):
            break

        msg_type = data[idx + 4]
        msg_len = data[idx + 5]
        frame_end = idx + HEADER_LEN + msg_len + CRC_LEN

        if frame_end > len(data):
            pos = idx + 1
            continue

        # CRC over type + len + payload
        crc_data = data[idx + 4 : idx + HEADER_LEN + msg_len]
        stored = (data[frame_end - 2] << 8) | data[frame_end - 1]
        if crc16(crc_data) != stored:
            stats.crc_errors += 1
            pos = idx + 1
            continue

        payload = data[idx + HEADER_LEN : idx + HEADER_LEN + msg_len]
        name = MSG_INFO.get(msg_type, (f"0x{msg_type:02X}", None))[0]
        counts[name] += 1

        # Extract timestamp (first 4 bytes of every sensor payload)
        if msg_len >= 4:
            ts = struct.unpack_from('<I', payload, 0)[0]
            timestamps[name].append(ts)

        # Decode known payloads
        p = _decode_payload(msg_type, msg_len, payload)
        if p is not None:
            parsed.append((name, p))

        stats.valid_frames += 1
        pos = frame_end

    stats.counts = dict(counts)
    stats.timestamps = dict(timestamps)
    stats.parsed_frames = parsed
    return stats


def _decode_payload(msg_type, msg_len, payload):
    """Decode a payload into a dict of SI-ish values. Returns None on mismatch."""
    try:
        if msg_type == 0xA1 and msg_len == 42:
            f = struct.unpack('<I HBB BBB H BBB iii iii BB', payload)
            return {
                'time_us': f[0],
                'year': f[1], 'month': f[2], 'day': f[3],
                'hour': f[4], 'minute': f[5], 'second': f[6], 'ms': f[7],
                'fix': f[8], 'sats': f[9], 'pdop_x10': f[10],
                'lat': f[11] / 1e7, 'lon': f[12] / 1e7, 'alt_m': f[13] / 1000.0,
                'vel_e': f[14] / 1000.0, 'vel_n': f[15] / 1000.0,
                'vel_u': f[16] / 1000.0,
                'h_acc': f[17], 'v_acc': f[18],
            }
        elif msg_type == 0xA2 and msg_len == 22:
            f = struct.unpack('<I hhh hhh hhh', payload)
            return {
                'time_us': f[0],
                'acc_low': (f[1], f[2], f[3]),
                'acc_high': (f[4], f[5], f[6]),
                'gyro': (f[7], f[8], f[9]),
            }
        elif msg_type == 0xA3 and msg_len == 12:
            f = struct.unpack('<I i I', payload)
            return {
                'time_us': f[0],
                'temp_c': f[1] / 65536.0,
                'press_pa': f[2] / 64.0,
            }
        elif msg_type == 0xA4 and msg_len == 16:
            f = struct.unpack('<I III', payload)
            return {
                'time_us': f[0],
                'mag_x': f[1], 'mag_y': f[2], 'mag_z': f[3],
            }
        elif msg_type == 0xA5 and msg_len == 42:
            f = struct.unpack('<I hhhh h iii iii BB h', payload)
            return {
                'time_us': f[0],
                'q0': f[1] / 10000.0, 'q1': f[2] / 10000.0,
                'q2': f[3] / 10000.0, 'q3': f[4] / 10000.0,
                'roll_cmd_deg': f[5] / 100.0,
                'e_pos_m': f[6] / 100.0, 'n_pos_m': f[7] / 100.0,
                'u_pos_m': f[8] / 100.0,
                'e_vel_ms': f[9] / 100.0, 'n_vel_ms': f[10] / 100.0,
                'u_vel_ms': f[11] / 100.0,
                'flags': f[12], 'rocket_state': f[13],
                'baro_alt_rate_ms': f[14] / 10.0,
            }
        elif msg_type == 0xA6 and msg_len == 10:
            f = struct.unpack('<I Hhh', payload)
            return {
                'time_us': f[0],
                'voltage': f[1] * 10.0 / 65535.0,
                'current_ma': f[2] * 10000.0 / 32767.0,
                'soc': f[3] * 150.0 / 32767.0 - 25.0,
            }
    except struct.error:
        pass
    return None


# ============================================================================
# Serial output parser
# ============================================================================

# Regex patterns for serial diagnostic lines
_RE_WALL = r'(\d{2}:\d{2}:\d{2}\.\d{3})\s*->\s*'

_RE_TIMING = re.compile(
    _RE_WALL +
    r'\[TIMING\]\s*ekf:\s*avg=(\d+)\s+max=(\d+)\s+us\s+cnt=(\d+)\s*\|\s*loop:\s*cnt=(\d+)/s'
)
_RE_TIMING_NOEKF = re.compile(
    _RE_WALL +
    r'\[TIMING\]\s*ekf:\s*not running\s*\|\s*loop:\s*cnt=(\d+)/s'
)

_RE_GAP1 = re.compile(
    _RE_WALL +
    r'\[GAP DIAG\]\s*iter_max=(\d+)\s+gnss_max=(\d+)\s+bmp_max=(\d+)\s+mmc_max=(\d+)\s+ism6_max=(\d+)\s+us'
)
_RE_GAP2 = re.compile(
    _RE_WALL +
    r'\[GAP DIAG\]\s*gaps>10ms=(\d+)\s+worst=(\d+)\s+us\s*\|\s*gnss calls=(\d+)\s+>1ms=(\d+)\s+>5ms=(\d+)\s+>10ms=(\d+)'
)
_RE_GAP3 = re.compile(
    _RE_WALL +
    r'\[GAP DIAG\]\s*i2c enqueue ok/drop=(\d+)/(\d+)\s+tx ok/fail=(\d+)/(\d+)\s+last_err=(-?\d+)\s+q_free=(\d+)'
)
_RE_GAP4 = re.compile(
    _RE_WALL +
    r'\[GAP DIAG\]\s*getOutReady_max=(\d+)\s+us\s+query ok/fail=(\d+)/(\d+)'
)

_RE_EKF1 = re.compile(
    _RE_WALL +
    r'\[EKF DIAG\]\s*quat_roll=([\d.-]+)\s+euler_roll=([\d.-]+)\s+pitch=([\d.-]+)\s+yaw=([\d.-]+)\s+cos2p=([\d.-]+)'
)
_RE_EKF2 = re.compile(
    _RE_WALL +
    r'\[EKF DIAG\]\s*mag=([\d.]+)uT\((\w+)\)\s+gyro_bias=\[([\d.,-]+)\]dps'
)


def parse_serial(path: str) -> SerialStats:
    """Parse serial monitor output for diagnostic lines."""
    stats = SerialStats(file_path=path)
    lines = Path(path).read_text(errors='replace').splitlines()

    # Accumulate partial gap diags (4 lines per window)
    pending_gap = SerialGapDiag()
    gap_lines_seen = 0

    for line in lines:
        # --- [TIMING] ---
        m = _RE_TIMING.match(line)
        if m:
            stats.timing_windows.append(SerialTimingWindow(
                wall_time=m.group(1),
                ekf_avg_us=int(m.group(2)),
                ekf_max_us=int(m.group(3)),
                ekf_count=int(m.group(4)),
                loop_count=int(m.group(5)),
                ekf_running=True,
            ))
            continue

        m = _RE_TIMING_NOEKF.match(line)
        if m:
            stats.timing_windows.append(SerialTimingWindow(
                wall_time=m.group(1),
                loop_count=int(m.group(2)),
                ekf_running=False,
            ))
            continue

        # --- [GAP DIAG] line 1 ---
        m = _RE_GAP1.match(line)
        if m:
            pending_gap = SerialGapDiag(wall_time=m.group(1))
            pending_gap.iter_max_us = int(m.group(2))
            pending_gap.gnss_max_us = int(m.group(3))
            pending_gap.bmp_max_us = int(m.group(4))
            pending_gap.mmc_max_us = int(m.group(5))
            pending_gap.ism6_max_us = int(m.group(6))
            gap_lines_seen = 1
            continue

        # --- [GAP DIAG] line 2 ---
        m = _RE_GAP2.match(line)
        if m:
            pending_gap.gaps_gt_10ms = int(m.group(2))
            pending_gap.worst_gap_us = int(m.group(3))
            pending_gap.gnss_calls = int(m.group(4))
            pending_gap.gnss_gt_1ms = int(m.group(5))
            pending_gap.gnss_gt_5ms = int(m.group(6))
            pending_gap.gnss_gt_10ms = int(m.group(7))
            gap_lines_seen = 2
            continue

        # --- [GAP DIAG] line 3 ---
        m = _RE_GAP3.match(line)
        if m:
            pending_gap.enqueue_ok = int(m.group(2))
            pending_gap.enqueue_drop = int(m.group(3))
            pending_gap.tx_ok = int(m.group(4))
            pending_gap.tx_fail = int(m.group(5))
            pending_gap.last_err = int(m.group(6))
            pending_gap.q_free = int(m.group(7))
            gap_lines_seen = 3
            continue

        # --- [GAP DIAG] line 4 ---
        m = _RE_GAP4.match(line)
        if m:
            pending_gap.out_ready_max_us = int(m.group(2))
            pending_gap.query_ok = int(m.group(3))
            pending_gap.query_fail = int(m.group(4))
            gap_lines_seen = 4
            stats.gap_diags.append(pending_gap)
            pending_gap = SerialGapDiag()
            gap_lines_seen = 0
            continue

        # --- [EKF DIAG] line 1 ---
        m = _RE_EKF1.match(line)
        if m:
            diag = SerialEkfDiag(
                wall_time=m.group(1),
                quat_roll=float(m.group(2)),
                euler_roll=float(m.group(3)),
                pitch=float(m.group(4)),
                yaw=float(m.group(5)),
                cos2p=float(m.group(6)),
            )
            stats.ekf_diags.append(diag)
            continue

        # --- [EKF DIAG] line 2 ---
        m = _RE_EKF2.match(line)
        if m and stats.ekf_diags:
            d = stats.ekf_diags[-1]
            d.mag_ut = float(m.group(2))
            d.mag_status = m.group(3)
            biases = m.group(4).split(',')
            d.gyro_bias = tuple(float(b) for b in biases)
            continue

    return stats


# ============================================================================
# Rate computation from timestamps
# ============================================================================

def compute_rate_stats(ts_list: list, max_gap_us: int = 1_000_000) -> dict:
    """Compute rate statistics from a list of microsecond timestamps.

    Primary rate is computed as (count - 1) / span_s, where span is the
    elapsed time from the first to the last timestamp.  This matches what
    analyze_rates.py reports and is the only metric that's meaningful when
    the log contains non-chronological data (e.g., ring-buffer dumps).

    Interval statistics (median, P95, P99, gap counts) are computed from
    forward-going intervals only.  Negative intervals (timestamps going
    backwards) and intervals larger than max_gap_us are excluded from
    interval stats but counted separately — a non-zero non_monotonic count
    flags the log as out of order so the caller can warn the user.
    """
    empty = {
        'count': len(ts_list),
        'rate_hz': 0,
        'span_s': 0,
        'non_monotonic': 0,
        'valid_intervals': 0,
    }
    if len(ts_list) < 2:
        return empty

    # Classify every raw interval (signed, not uint32-masked).
    forward_diffs = []          # 0 < d <= max_gap_us
    large_forward = 0           # d > max_gap_us (dropouts, not stalls)
    non_monotonic = 0           # d <= 0 (duplicates or time going backwards)
    for i in range(1, len(ts_list)):
        d = ts_list[i] - ts_list[i - 1]
        if d <= 0:
            non_monotonic += 1
        elif d > max_gap_us:
            large_forward += 1
        else:
            forward_diffs.append(d)

    # Span: first-to-last timestamp.  If non-positive (e.g., the log ends
    # earlier than it started — ring-buffer wrap), fall back to the sum of
    # forward-going intervals as a best-effort "real elapsed time".
    raw_span_us = ts_list[-1] - ts_list[0]
    if raw_span_us > 0:
        span_us = raw_span_us
    else:
        span_us = sum(forward_diffs)  # may still be 0 if nothing is sane

    span_s = span_us / 1e6
    # Primary rate: observed throughput over the span the data actually covers.
    rate_hz = (len(ts_list) - 1) / span_s if span_s > 0 else 0

    result = {
        'count': len(ts_list),
        'rate_hz': rate_hz,
        'span_s': span_s,
        'non_monotonic': non_monotonic,
        'large_forward': large_forward,
        'valid_intervals': len(forward_diffs),
    }

    if not forward_diffs:
        return result

    diffs_sorted = sorted(forward_diffs)
    n = len(diffs_sorted)
    result.update({
        'avg_us': sum(forward_diffs) / n,
        'median_us': diffs_sorted[n // 2],
        'min_us': diffs_sorted[0],
        'max_us': diffs_sorted[-1],
        'p95_us': diffs_sorted[int(0.95 * n)],
        'p99_us': diffs_sorted[int(0.99 * n)] if n >= 100 else diffs_sorted[-1],
        'gaps_gt_2ms': sum(1 for d in forward_diffs if d > 2000),
        'gaps_gt_10ms': sum(1 for d in forward_diffs if d > 10000),
    })
    return result


# ============================================================================
# Report generation
# ============================================================================

def _bar(value, max_val, width=30):
    """ASCII bar for visualizing a value against a maximum."""
    if max_val <= 0:
        return ""
    filled = min(width, int(width * value / max_val))
    return "█" * filled + "░" * (width - filled)


def _status_icon(actual, target):
    """Return a status indicator comparing actual to target rate."""
    if target <= 0:
        return " "
    ratio = actual / target
    if ratio >= 0.95:
        return "✅"
    elif ratio >= 0.70:
        return "⚠️"
    else:
        return "❌"


def print_header(title: str):
    w = 70
    print()
    print("=" * w)
    print(f"  {title}")
    print("=" * w)


def report_binary(bs: BinaryStats):
    """Print analysis report from binary flight log."""
    print_header(f"BINARY LOG: {Path(bs.file_path).name}")
    print(f"  File size:    {bs.file_size:,} bytes")
    print(f"  Valid frames: {bs.valid_frames:,}")
    print(f"  CRC errors:   {bs.crc_errors:,}")

    # ── Message counts ──
    print_header("MESSAGE COUNTS")
    total = sum(bs.counts.values())
    for name in sorted(bs.counts, key=lambda n: -bs.counts[n]):
        cnt = bs.counts[name]
        pct = 100 * cnt / total if total else 0
        print(f"  {name:14s}  {cnt:>8,}  ({pct:5.1f}%)")
    print(f"  {'TOTAL':14s}  {total:>8,}")

    # ── Sensor rates ──
    print_header("LOGGED SENSOR RATES")
    print(f"  {'Sensor':<14s} {'Rate':>8s} {'Target':>8s} {'Status':>6s}  {'Median':>8s} {'P99':>8s} {'Msgs':>8s} {'Span':>6s}")
    print(f"  {'-'*14} {'-'*8} {'-'*8} {'-'*6}  {'-'*8} {'-'*8} {'-'*8} {'-'*6}")

    any_non_monotonic = False
    for name in ["ISM6", "BMP", "MMC", "GNSS", "NonSensor", "Power"]:
        ts = bs.timestamps.get(name, [])
        target = TARGET_RATES.get(name, -1)
        rs = compute_rate_stats(ts)

        if rs.get('non_monotonic', 0) > 0:
            any_non_monotonic = True

        if rs['rate_hz'] > 0:
            target_str = f"{target}" if target > 0 else "—"
            status = _status_icon(rs['rate_hz'], target) if target > 0 else " "
            median = rs.get('median_us', 0)
            p99    = rs.get('p99_us', 0)
            flag = " *" if rs.get('non_monotonic', 0) > 0 else ""
            print(f"  {name:<14s} {rs['rate_hz']:>7.1f}  {target_str:>7s}   {status}   "
                  f"{median:>7.0f}  {p99:>7.0f}  {rs['count']:>7,}  {rs['span_s']:>5.1f}s{flag}")
        elif len(ts) > 0:
            print(f"  {name:<14s}     —       —        —    (only {len(ts)} msgs)")
        # Skip if no messages at all

    if any_non_monotonic:
        print()
        print("  * Non-monotonic timestamps detected — log is not chronological.")
        print("    This is typical of a ring-buffer / recovery dump, not a live")
        print("    flight log.  Rate is computed as (count-1)/span and is reliable;")
        print("    individual gap stats reflect forward intervals only.")

    # ── Gap analysis for key sensors ──
    for sensor_name in ["ISM6", "BMP", "MMC", "NonSensor"]:
        ts = bs.timestamps.get(sensor_name, [])
        rs = compute_rate_stats(ts)
        if rs.get('valid_intervals', 0) < 10:
            continue

        print_header(f"{sensor_name} GAP ANALYSIS")
        print(f"  Intervals:  {rs['valid_intervals']:,}")
        skipped_nm = rs.get('non_monotonic', 0)
        skipped_lg = rs.get('large_forward', 0)
        if skipped_nm or skipped_lg:
            print(f"  Excluded:   {skipped_nm:,} non-monotonic, "
                  f"{skipped_lg:,} >1s (dropouts)")
        print(f"  Min gap:    {rs['min_us']:,.0f} µs  ({rs['min_us']/1000:.1f} ms)")
        print(f"  Median gap: {rs['median_us']:,.0f} µs  ({rs['median_us']/1000:.1f} ms)")
        print(f"  P95 gap:    {rs['p95_us']:,.0f} µs  ({rs['p95_us']/1000:.1f} ms)")
        print(f"  P99 gap:    {rs['p99_us']:,.0f} µs  ({rs['p99_us']/1000:.1f} ms)")
        print(f"  Max gap:    {rs['max_us']:,.0f} µs  ({rs['max_us']/1000:.1f} ms)")
        print(f"  Gaps > 2ms: {rs['gaps_gt_2ms']:,} ({100*rs['gaps_gt_2ms']/rs['valid_intervals']:.2f}%)")
        print(f"  Gaps >10ms: {rs['gaps_gt_10ms']:,} ({100*rs['gaps_gt_10ms']/rs['valid_intervals']:.2f}%)")

    # ── EKF / state analysis ──
    ns_frames = [(n, p) for n, p in bs.parsed_frames if n == "NonSensor"]
    if ns_frames:
        print_header("EKF / STATE ANALYSIS")

        # Quaternion health
        qnorms = [math.sqrt(p['q0']**2 + p['q1']**2 + p['q2']**2 + p['q3']**2)
                   for _, p in ns_frames]
        bad_q = sum(1 for q in qnorms if abs(q - 1.0) > 0.05)
        print(f"  Quat norm:      {min(qnorms):.4f} .. {max(qnorms):.4f}  (target: 1.0000)")
        if bad_q:
            print(f"  ⚠️  {bad_q} samples with |q|-1| > 0.05")
        else:
            print(f"  ✅  All quaternion norms within ±0.05 of 1.0")

        # Position drift
        u_pos = [p['u_pos_m'] for _, p in ns_frames]
        max_drift = max(abs(p) for p in u_pos)
        print(f"  Altitude range: {min(u_pos):.2f} .. {max(u_pos):.2f} m")

        # Velocity
        speeds = [math.sqrt(p['e_vel_ms']**2 + p['n_vel_ms']**2 + p['u_vel_ms']**2)
                  for _, p in ns_frames]
        print(f"  Speed range:    {min(speeds):.3f} .. {max(speeds):.3f} m/s")

        # State machine
        states_seen = set(p['rocket_state'] for _, p in ns_frames)
        state_names = [ROCKET_STATES.get(s, f"?{s}") for s in sorted(states_seen)]
        print(f"  States seen:    {', '.join(state_names)}")

        # State transitions
        prev = None
        for _, p in ns_frames:
            s = p['rocket_state']
            if s != prev:
                t_s = p['time_us'] / 1e6
                sname = ROCKET_STATES.get(s, f"?{s}")
                prev_name = ROCKET_STATES.get(prev, "None") if prev is not None else "—"
                print(f"    t={t_s:>8.2f}s  {prev_name} → {sname}")
                prev = s

    # ── Anomaly detection ──
    print_header("ANOMALY CHECK")
    anomalies = []

    if ns_frames:
        max_speed = max(math.sqrt(p['e_vel_ms']**2 + p['n_vel_ms']**2 + p['u_vel_ms']**2)
                        for _, p in ns_frames)
        max_alt = max(abs(p['u_pos_m']) for _, p in ns_frames)
        if max_speed > 2.0:
            anomalies.append(f"EKF speed drifted to {max_speed:.2f} m/s (bench: expect ~0)")
        if max_alt > 5.0:
            anomalies.append(f"EKF altitude drifted to {max_alt:.2f} m (bench: expect ~0)")
        if bad_q > 0:
            anomalies.append(f"{bad_q} quaternion norm violations (|q| > 5% from 1.0)")
        if 3 in states_seen:
            anomalies.append("Rocket entered INFLIGHT on bench!")

    # Sensor rate anomalies + non-chronological log detection
    log_non_chrono = False
    for name, target in TARGET_RATES.items():
        if target <= 0:
            continue
        ts = bs.timestamps.get(name, [])
        rs = compute_rate_stats(ts)
        if rs.get('non_monotonic', 0) > 0:
            log_non_chrono = True
        if rs['rate_hz'] > 0 and rs['rate_hz'] < target * 0.5:
            anomalies.append(f"{name} rate {rs['rate_hz']:.0f} Hz is <50% of target {target} Hz")
    if log_non_chrono:
        anomalies.append("Log contains non-monotonic timestamps — "
                         "likely a ring-buffer / recovery dump, not a live flight log")

    # IMU saturation
    imu_frames = [(n, p) for n, p in bs.parsed_frames if n == "ISM6"]
    if imu_frames:
        SAT = 32700
        for axis_name, idx_tuple in [('acc_low_x', ('acc_low', 0)),
                                      ('acc_low_y', ('acc_low', 1)),
                                      ('acc_low_z', ('acc_low', 2)),
                                      ('gyro_x', ('gyro', 0)),
                                      ('gyro_y', ('gyro', 1)),
                                      ('gyro_z', ('gyro', 2))]:
            key, i = idx_tuple
            sat = sum(1 for _, p in imu_frames if abs(p[key][i]) >= SAT)
            if sat > 0:
                anomalies.append(f"IMU {axis_name} saturated in {sat}/{len(imu_frames)} samples")

    if not anomalies:
        print("  ✅  No anomalies detected.")
    else:
        for a in anomalies:
            print(f"  ⚠️  {a}")


def report_serial(ss: SerialStats):
    """Print analysis report from serial monitor output."""
    print_header(f"SERIAL OUTPUT: {Path(ss.file_path).name}")

    # ── Timing ──
    if ss.timing_windows:
        print_header("EKF TIMING (from [TIMING] lines)")

        tw = ss.timing_windows
        running = [t for t in tw if t.ekf_running]

        if running:
            avg_of_avgs = sum(t.ekf_avg_us for t in running) / len(running)
            worst_max = max(t.ekf_max_us for t in running)
            total_ekf = sum(t.ekf_count for t in running)
            avg_loop = sum(t.loop_count for t in running) / len(running)
            avg_ekf_rate = sum(t.ekf_count for t in running) / len(running)

            print(f"  Windows:        {len(running)} (EKF active)")
            print(f"  EKF avg:        {avg_of_avgs:.0f} µs")
            print(f"  EKF worst max:  {worst_max} µs")
            print(f"  EKF runs/window:{avg_ekf_rate:.0f}  (≈ {avg_ekf_rate:.0f} Hz)")
            print(f"  Loop rate:      {avg_loop:.0f} Hz  (target: {FLIGHT_LOOP_TARGET})")
            print(f"  EKF rate:       ≈{avg_ekf_rate:.0f} Hz  (loop/{EKF_DECIMATION})")

            # Per-window table
            print(f"\n  {'Wall Time':>12s} {'EKF avg':>8s} {'EKF max':>8s} {'EKF cnt':>8s} {'Loop/s':>8s}")
            print(f"  {'-'*12} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")
            for t in running[-10:]:  # show last 10
                print(f"  {t.wall_time:>12s} {t.ekf_avg_us:>7d}  {t.ekf_max_us:>7d}  "
                      f"{t.ekf_count:>7d}  {t.loop_count:>7d}")
            if len(running) > 10:
                print(f"  ... ({len(running) - 10} earlier windows omitted)")

        not_running = [t for t in tw if not t.ekf_running]
        if not_running:
            print(f"\n  {len(not_running)} windows with EKF not running")

    # ── I2C Queue Health ──
    if ss.gap_diags:
        print_header("I2C QUEUE HEALTH (from [GAP DIAG] lines)")

        latest = ss.gap_diags[-1]
        total_enq = latest.enqueue_ok + latest.enqueue_drop
        drop_pct = 100 * latest.enqueue_drop / total_enq if total_enq > 0 else 0

        print(f"  Latest window ({latest.wall_time}):")
        print(f"    Enqueue OK:    {latest.enqueue_ok:>10,}")
        print(f"    Enqueue drop:  {latest.enqueue_drop:>10,}")
        print(f"    Drop rate:     {drop_pct:.1f}%  {'⚠️' if drop_pct > 5 else '✅'}")
        print(f"    TX OK/fail:    {latest.tx_ok:,} / {latest.tx_fail:,}")
        print(f"    Queue free:    {latest.q_free}")
        print(f"    Last I2C err:  {latest.last_err}")

        # Trend
        if len(ss.gap_diags) > 1:
            print(f"\n  {'Wall Time':>12s} {'Enq OK':>10s} {'Enq Drop':>10s} {'Drop%':>7s} {'Q Free':>7s}")
            print(f"  {'-'*12} {'-'*10} {'-'*10} {'-'*7} {'-'*7}")
            for g in ss.gap_diags[-8:]:  # last 8 windows
                t = g.enqueue_ok + g.enqueue_drop
                dp = 100 * g.enqueue_drop / t if t > 0 else 0
                print(f"  {g.wall_time:>12s} {g.enqueue_ok:>10,} {g.enqueue_drop:>10,} "
                      f"{dp:>6.1f}% {g.q_free:>6d}")

    # ── Sensor poll timing ──
    if ss.gap_diags:
        print_header("SENSOR POLL TIMING (worst-case per window)")
        print(f"  {'Wall Time':>12s} {'Iter max':>9s} {'GNSS':>7s} {'BMP':>7s} {'MMC':>7s} {'ISM6':>7s}")
        print(f"  {'-'*12} {'-'*9} {'-'*7} {'-'*7} {'-'*7} {'-'*7}")
        for g in ss.gap_diags[-8:]:
            print(f"  {g.wall_time:>12s} {g.iter_max_us:>8d}  {g.gnss_max_us:>6d}  "
                  f"{g.bmp_max_us:>6d}  {g.mmc_max_us:>6d}  {g.ism6_max_us:>6d}")

    # ── EKF diagnostics ──
    if ss.ekf_diags:
        print_header("EKF STATE (from [EKF DIAG] lines)")
        latest = ss.ekf_diags[-1]
        print(f"  Latest ({latest.wall_time}):")
        print(f"    Roll (quat):  {latest.quat_roll:.1f}°")
        print(f"    Roll (euler): {latest.euler_roll:.1f}°")
        print(f"    Pitch:        {latest.pitch:.1f}°")
        print(f"    Yaw:          {latest.yaw:.1f}°")
        print(f"    cos²(pitch):  {latest.cos2p:.4f}")
        print(f"    Mag field:    {latest.mag_ut:.1f} µT  ({latest.mag_status})")
        print(f"    Gyro bias:    [{latest.gyro_bias[0]:.3f}, {latest.gyro_bias[1]:.3f}, "
              f"{latest.gyro_bias[2]:.3f}] dps")

        # Check stability over time
        if len(ss.ekf_diags) >= 3:
            biases = [d.gyro_bias for d in ss.ekf_diags]
            for axis, name in enumerate(['X', 'Y', 'Z']):
                vals = [b[axis] for b in biases]
                spread = max(vals) - min(vals)
                status = "✅ stable" if spread < 0.1 else "⚠️ drifting"
                print(f"    Gyro bias {name} spread: {spread:.4f} dps  ({status})")


def report_combined(bs: Optional[BinaryStats], ss: Optional[SerialStats]):
    """Print combined comparison when both sources are available."""
    if not (bs and ss and ss.timing_windows):
        return

    print_header("COMBINED: SERIAL vs BINARY COMPARISON")

    running = [t for t in ss.timing_windows if t.ekf_running]
    if not running:
        return

    avg_loop = sum(t.loop_count for t in running) / len(running)
    avg_ekf_hz = sum(t.ekf_count for t in running) / len(running)

    print(f"\n  {'Metric':<30s} {'Serial':>12s} {'Binary Log':>12s}")
    print(f"  {'-'*30} {'-'*12} {'-'*12}")

    print(f"  {'Flight loop rate':<30s} {avg_loop:>11.0f}  {'—':>12s}")
    print(f"  {'EKF rate':<30s} {avg_ekf_hz:>11.0f}  {'—':>12s}")

    for name in ["ISM6", "BMP", "MMC", "NonSensor"]:
        ts = bs.timestamps.get(name, [])
        rs = compute_rate_stats(ts)
        binary_str = f"{rs['rate_hz']:.1f}" if rs['rate_hz'] > 0 else "—"
        target = TARGET_RATES.get(name, -1)
        target_str = f" (/{target})" if target > 0 else ""
        print(f"  {name + ' rate' + target_str:<30s} {'—':>12s} {binary_str:>12s}")

    # I2C throughput
    if ss.gap_diags:
        latest = ss.gap_diags[-1]
        total = latest.enqueue_ok + latest.enqueue_drop
        drop_pct = 100 * latest.enqueue_drop / total if total > 0 else 0
        print(f"\n  I2C enqueue total:   {total:>10,}")
        print(f"  I2C enqueue drops:   {latest.enqueue_drop:>10,}  ({drop_pct:.1f}%)")
        print(f"  I2C TX success:      {latest.tx_ok:>10,}")

        if running:
            avg_ekf_us = sum(t.ekf_avg_us for t in running) / len(running)
            worst = max(t.ekf_max_us for t in running)
            print(f"\n  EKF avg exec time:   {avg_ekf_us:>10.0f} µs")
            print(f"  EKF worst-case:      {worst:>10d} µs")
            loop_period_us = 1e6 / avg_loop if avg_loop > 0 else 0
            ekf_pct = 100 * avg_ekf_us / loop_period_us if loop_period_us > 0 else 0
            print(f"  Loop period:         {loop_period_us:>10.0f} µs")
            print(f"  EKF % of loop:       {ekf_pct:>10.1f}%")


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Analyze TinkerRocket bench test data (binary log and/or serial output).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('files', nargs='*',
                        help='Binary (.bin) and/or serial (.txt/.log) files')
    parser.add_argument('--bin', dest='bin_file',
                        help='Binary flight log file')
    parser.add_argument('--serial', dest='serial_file',
                        help='Serial monitor output file')
    args = parser.parse_args()

    bin_path = args.bin_file
    serial_path = args.serial_file

    # Auto-detect from positional args
    for f in (args.files or []):
        p = Path(f)
        if not p.exists():
            print(f"Error: {f} not found", file=sys.stderr)
            sys.exit(1)
        if p.suffix == '.bin':
            bin_path = str(p)
        else:
            serial_path = str(p)

    if not bin_path and not serial_path:
        parser.print_help()
        sys.exit(1)

    print("╔══════════════════════════════════════════════════════════════════════╗")
    print("║           TinkerRocket Bench Test Analysis                         ║")
    print("╚══════════════════════════════════════════════════════════════════════╝")

    bs = None
    ss = None

    if bin_path:
        bs = parse_binary(bin_path)
        report_binary(bs)

    if serial_path:
        ss = parse_serial(serial_path)
        report_serial(ss)

    if bs and ss:
        report_combined(bs, ss)

    print()


if __name__ == '__main__':
    main()
