#!/usr/bin/env python3
"""Plot inter-frame gap analysis for a TinkerRocket binary flight log.

Produces three PNGs in <bin_path>_analysis/ (configurable via --out):
  gap_overview.png    per-sensor inter-frame gap (log scale) vs. wall time
  gap_histogram.png   inter-frame gap distribution per sensor
  record_density.png  frames per 0.5 s bin (sensor-stall heatmap)

Useful for confirming that Core 0 stalls (NAND erase, prepareFlight, etc.)
do not bleed into Core 1 sensor capture, or to spot any sensor that drops
out during a bench/flight run.

Usage:
    python3 tests/plot_gaps.py tests/test_data/flight_<ts>.bin
    python3 tests/plot_gaps.py <bin> --out my_plots/
"""

import argparse
import struct
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SOF = b"\xAA\x55\xAA\x55"

# (msg_type_byte) -> short name. Sizes are validated by the inline header.
# Only sensors with a leading uint32 time_us field are useful for gap analysis;
# zero-payload control frames (StartLog, EndFlight) are ignored.
MSG_TYPES = {
    0xA0: "StatusQuery",
    0xA1: "GNSS",
    0xA2: "ISM6",
    0xA3: "BMP",
    0xA4: "MMC",
    0xA5: "NonSensor",
    0xA6: "Power",
    0xF1: "LoRa",
}

# Plot order and colors. Sensors not present in the file are skipped.
PLOT_ORDER = ["ISM6", "BMP", "MMC", "NonSensor", "Power"]
COLORS = {
    "ISM6": "#1f77b4",
    "BMP": "#ff7f0e",
    "MMC": "#2ca02c",
    "NonSensor": "#d62728",
    "Power": "#9467bd",
}


def crc16(data: bytes) -> int:
    """Match the firmware CRC: poly=0x8001, init=0, no reversal, no XOR-out."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8001) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_timestamps(path: Path) -> dict[str, list[int]]:
    """Walk the binary log frame-by-frame and return {sensor_name: [t_us, ...]}.

    CRC is checked over [type, len, payload] (matching parse_flight.py); CRC
    bytes themselves are big-endian on the wire. Frames with bad CRC or
    unknown type are skipped.
    """
    data = path.read_bytes()
    timestamps: dict[str, list[int]] = defaultdict(list)
    pos = 0
    while pos < len(data) - 8:
        idx = data.find(SOF, pos)
        if idx < 0 or idx + 6 > len(data):
            break
        msg_type = data[idx + 4]
        payload_len = data[idx + 5]
        if msg_type not in MSG_TYPES:
            pos = idx + 1
            continue
        end = idx + 6 + payload_len + 2
        if end > len(data):
            pos = idx + 1
            continue
        crc_data = data[idx + 4 : idx + 6 + payload_len]
        expected = (data[idx + 6 + payload_len] << 8) | data[idx + 6 + payload_len + 1]
        if crc16(crc_data) != expected:
            pos = idx + 1
            continue
        if payload_len >= 4:
            t_us = struct.unpack("<I", data[idx + 6 : idx + 10])[0]
            timestamps[MSG_TYPES[msg_type]].append(t_us)
        pos = end
    return timestamps


def plot_gap_overview(timestamps, sensors, t0_us, out_path, title):
    fig, axes = plt.subplots(
        len(sensors), 1, figsize=(11, 2 + 1.5 * len(sensors)),
        sharex=True, constrained_layout=True,
    )
    if len(sensors) == 1:
        axes = [axes]
    for ax, sensor in zip(axes, sensors):
        t = [(x - t0_us) / 1e6 for x in timestamps[sensor]]
        gaps = [(t[i + 1] - t[i]) * 1000.0 for i in range(len(t) - 1)]
        ax.plot(t[1:], gaps, ".", markersize=2, color=COLORS[sensor], alpha=0.6)
        ax.axhline(10.0, color="r", linestyle="--", alpha=0.4, label="10 ms")
        ax.set_ylabel(f"{sensor}\ngap (ms)")
        ax.set_yscale("log")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("time since first frame (s)")
    fig.suptitle(f"Inter-frame gaps — {title}")
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_histogram(timestamps, sensors, out_path, title):
    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    for sensor in sensors:
        t = timestamps[sensor]
        gaps = [(t[i + 1] - t[i]) / 1000.0 for i in range(len(t) - 1)]
        ax.hist(gaps, bins=80, alpha=0.5, label=sensor, color=COLORS[sensor])
    ax.axvline(10.0, color="r", linestyle="--", alpha=0.6, label="10 ms threshold")
    ax.set_xlabel("inter-frame gap (ms)")
    ax.set_ylabel("count")
    ax.set_yscale("log")
    ax.legend()
    ax.set_title(f"Gap histogram — {title}")
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_record_density(timestamps, sensors, t0_us, t1_us, out_path, title):
    fig, ax = plt.subplots(figsize=(11, 5), constrained_layout=True)
    bin_width_s = 0.5
    span_s = (t1_us - t0_us) / 1e6
    nbins = max(1, int(span_s / bin_width_s) + 1)
    edges = [i * bin_width_s for i in range(nbins + 1)]
    for sensor in sensors:
        counts = [0] * nbins
        for x in timestamps[sensor]:
            b = int(((x - t0_us) / 1e6) / bin_width_s)
            if 0 <= b < nbins:
                counts[b] += 1
        ax.plot(edges[:-1], counts, label=sensor, color=COLORS[sensor])
    ax.set_xlabel("time since first frame (s)")
    ax.set_ylabel(f"frames per {bin_width_s} s bin")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_title(f"Record density — {title}")
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bin_path", help="Path to a .bin flight log")
    ap.add_argument("--out", help="Output directory (default: <bin_path>_analysis/)")
    args = ap.parse_args()

    bin_path = Path(args.bin_path)
    if not bin_path.is_file():
        print(f"error: {bin_path} not found", file=sys.stderr)
        sys.exit(1)

    timestamps = parse_timestamps(bin_path)
    sensors = [s for s in PLOT_ORDER if len(timestamps.get(s, [])) > 1]
    if not sensors:
        print(f"error: no sensors with >1 frames found in {bin_path}", file=sys.stderr)
        sys.exit(1)

    out_dir = Path(args.out) if args.out else Path(
        bin_path.with_suffix("").as_posix() + "_analysis"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    t0_us = min(min(timestamps[s]) for s in sensors)
    t1_us = max(max(timestamps[s]) for s in sensors)
    title = bin_path.name

    plot_gap_overview(timestamps, sensors, t0_us, out_dir / "gap_overview.png", title)
    plot_histogram(timestamps, sensors, out_dir / "gap_histogram.png", title)
    plot_record_density(timestamps, sensors, t0_us, t1_us,
                        out_dir / "record_density.png", title)

    print(f"Wrote 3 plots to {out_dir}/")
    for name in ("gap_overview.png", "gap_histogram.png", "record_density.png"):
        print(f"  {name}")


if __name__ == "__main__":
    main()
