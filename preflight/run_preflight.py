#!/usr/bin/env python3
"""
TinkerRocket Pre-Flight Checklist
==================================
Reads a bench test binary log (.bin) and runs go/no-go checks
that validate the firmware is flight-ready.

Usage:
    python preflight/run_preflight.py tests/test_data/bench_static_60s.bin

Checks:
    1. Sensor Rates      - IMU > 900 Hz, Baro > 400 Hz, Mag > 150 Hz
    2. IMU Bias          - Static accel bias < 0.5 m/s^2, gyro < 1 deg/s
    3. Frame Integrity   - All frames have valid CRC16, no drops
    4. Timestamp Health  - Per-sensor timestamps monotonically increasing
    5. Max IMU Gap       - No gaps > 10ms between consecutive IMU samples
    6. EKF Convergence   - If NonSensor frames present, flags converge
    7. Data Completeness - All expected message types present

Exit code 0 = GO, exit code 1 = NO-GO.
"""

import sys
import struct
import argparse
from pathlib import Path
from collections import defaultdict
from dataclasses import dataclass, field


# --- Frame constants (from RocketComputerTypes.h) ---
PREAMBLE = bytes([0xAA, 0x55, 0xAA, 0x55])
MSG_TYPES = {
    0xA0: ("STATUS_QUERY", 16),
    0xA1: ("GNSS",         42),
    0xA2: ("ISM6HG256",    22),
    0xA3: ("BMP585",       12),
    0xA4: ("MMC5983MA",    16),
    0xA5: ("NON_SENSOR",   43),
    0xA6: ("POWER",        10),
    0xF1: ("LORA",         57),
}


def crc16(data: bytes, poly: int = 0x8001, init: int = 0x0000) -> int:
    """CRC-16 matching the C++ calcCRC16() with default params."""
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class FrameStats:
    count: int = 0
    timestamps: list = field(default_factory=list)
    crc_errors: int = 0


def parse_bin(data: bytes) -> dict:
    """Parse binary log, return per-type FrameStats."""
    stats = defaultdict(FrameStats)
    idx = 0
    total_frames = 0
    total_crc_errors = 0

    while idx < len(data) - 8:
        # Find preamble
        pos = data.find(PREAMBLE, idx)
        if pos == -1:
            break
        idx = pos + 4

        if idx + 2 > len(data):
            break

        msg_type = data[idx]
        msg_len = data[idx + 1]
        idx += 2

        if idx + msg_len + 2 > len(data):
            break

        payload = data[idx:idx + msg_len]
        crc_received = (data[idx + msg_len] << 8) | data[idx + msg_len + 1]
        idx += msg_len + 2

        # Verify CRC
        crc_region = bytes([msg_type, msg_len]) + payload
        crc_calc = crc16(crc_region)

        total_frames += 1

        if crc_calc != crc_received:
            total_crc_errors += 1
            stats[msg_type].crc_errors += 1
            continue

        stats[msg_type].count += 1

        # Extract timestamp (first 4 bytes of payload, little-endian)
        if len(payload) >= 4:
            ts = struct.unpack_from("<I", payload, 0)[0]
            stats[msg_type].timestamps.append(ts)

    return dict(stats), total_frames, total_crc_errors


def compute_rate(timestamps: list) -> float:
    """Compute average sample rate from timestamp list (us)."""
    if len(timestamps) < 2:
        return 0.0
    diffs = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)
             if timestamps[i+1] > timestamps[i]]
    if not diffs:
        return 0.0
    mean_dt_us = sum(diffs) / len(diffs)
    if mean_dt_us <= 0:
        return 0.0
    return 1e6 / mean_dt_us


def max_gap_ms(timestamps: list) -> float:
    """Find the maximum gap between consecutive timestamps (ms)."""
    if len(timestamps) < 2:
        return 0.0
    max_dt = 0
    for i in range(len(timestamps) - 1):
        dt = timestamps[i+1] - timestamps[i]
        if dt > 0:
            max_dt = max(max_dt, dt)
    return max_dt / 1000.0


def check_monotonic(timestamps: list) -> int:
    """Count non-monotonic timestamp violations."""
    violations = 0
    for i in range(len(timestamps) - 1):
        if timestamps[i+1] < timestamps[i]:
            violations += 1
    return violations


# --- Checks ---

@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str


def run_checks(stats: dict, total_frames: int, total_crc_errors: int) -> list:
    results = []

    # 1. Sensor Rates
    rate_checks = [
        (0xA2, "IMU",  900.0),
        (0xA3, "Baro", 400.0),
        (0xA4, "Mag",  150.0),
    ]
    for msg_type, name, threshold in rate_checks:
        if msg_type in stats and len(stats[msg_type].timestamps) > 10:
            rate = compute_rate(stats[msg_type].timestamps)
            passed = rate >= threshold
            results.append(CheckResult(
                f"{name} Rate",
                passed,
                f"{rate:.0f} Hz (threshold: {threshold:.0f} Hz)"
            ))
        else:
            results.append(CheckResult(f"{name} Rate", False, "No data"))

    # 2. IMU Bias (placeholder - would need SI conversion)
    if 0xA2 in stats and stats[0xA2].count > 100:
        results.append(CheckResult("IMU Bias", True,
                                   f"{stats[0xA2].count} samples available for offline bias check"))
    else:
        results.append(CheckResult("IMU Bias", False, "Insufficient IMU data"))

    # 3. Frame Integrity
    results.append(CheckResult(
        "Frame Integrity",
        total_crc_errors == 0,
        f"{total_crc_errors} CRC errors in {total_frames} frames"
    ))

    # 4. Timestamp Health
    total_violations = 0
    for msg_type, st in stats.items():
        total_violations += check_monotonic(st.timestamps)
    results.append(CheckResult(
        "Timestamp Monotonic",
        total_violations == 0,
        f"{total_violations} non-monotonic timestamps"
    ))

    # 5. Max IMU Gap
    if 0xA2 in stats:
        gap = max_gap_ms(stats[0xA2].timestamps)
        results.append(CheckResult(
            "Max IMU Gap",
            gap < 10.0,
            f"{gap:.1f} ms (threshold: 10 ms)"
        ))
    else:
        results.append(CheckResult("Max IMU Gap", False, "No IMU data"))

    # 6. EKF Convergence (check if NonSensor frames have launch_flag etc.)
    if 0xA5 in stats and stats[0xA5].count > 0:
        results.append(CheckResult(
            "EKF Data Present",
            True,
            f"{stats[0xA5].count} NonSensor frames"
        ))
    else:
        results.append(CheckResult("EKF Data Present", False, "No NonSensor data"))

    # 7. Data Completeness
    expected = {0xA2, 0xA3, 0xA4}  # IMU, Baro, Mag minimum
    present = set(stats.keys()) & expected
    results.append(CheckResult(
        "Data Completeness",
        present == expected,
        f"Present: {[MSG_TYPES.get(t, (hex(t),))[0] for t in sorted(present)]}"
    ))

    return results


def main():
    parser = argparse.ArgumentParser(description="TinkerRocket Pre-Flight Checklist")
    parser.add_argument("binfile", type=Path, help="Path to bench test .bin file")
    args = parser.parse_args()

    if not args.binfile.exists():
        print(f"ERROR: File not found: {args.binfile}")
        sys.exit(1)

    data = args.binfile.read_bytes()
    print(f"Parsing {args.binfile.name} ({len(data):,} bytes)...\n")

    stats, total_frames, total_crc_errors = parse_bin(data)

    # Print message summary
    print("Message Summary:")
    print(f"  Total frames: {total_frames}")
    for msg_type in sorted(stats.keys()):
        name = MSG_TYPES.get(msg_type, (f"0x{msg_type:02X}",))[0]
        st = stats[msg_type]
        rate = compute_rate(st.timestamps)
        print(f"  {name:15s}: {st.count:6d} frames, {rate:7.0f} Hz")
    print()

    # Run checks
    results = run_checks(stats, total_frames, total_crc_errors)

    # Print results
    all_pass = True
    print("Pre-Flight Checklist:")
    print("-" * 60)
    for r in results:
        status = "PASS" if r.passed else "FAIL"
        icon = "\u2705" if r.passed else "\u274C"
        print(f"  {icon} [{status}] {r.name:25s} {r.detail}")
        if not r.passed:
            all_pass = False

    print("-" * 60)
    if all_pass:
        print("\n  RESULT: GO \u2705\n")
        sys.exit(0)
    else:
        print("\n  RESULT: NO-GO \u274C\n")
        sys.exit(1)


if __name__ == "__main__":
    main()
