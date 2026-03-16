#!/usr/bin/env python3
"""Quick script to plot roll/pitch/yaw and mag from a binary flight log."""

import struct
import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

SYNC = b"\xAA\x55\xAA\x55"

# Message types (from RocketComputerTypes.h)
NON_SENSOR_MSG  = 0xA5
MMC5983MA_MSG   = 0xA4
ISM6HG256_MSG   = 0xA2

# NonSensorData: 40 bytes packed
# uint32 time_us, int16 roll/pitch/yaw/roll_cmd (centideg),
# int32 e/n/u_pos (cm), int32 e/n/u_vel (cm/s),
# uint8 flags, uint8 rocket_state, int16 baro_alt_rate_dmps
NS_FMT = "<IhhhhiiiiiiB Bh"
NS_SIZE = struct.calcsize(NS_FMT)

# MMC5983MAData: 16 bytes packed
# uint32 time_us, uint32 mag_x/y/z (raw counts, 18-bit centered at 131072)
MAG_FMT = "<IIII"
MAG_SIZE = struct.calcsize(MAG_FMT)

# ISM6HG256Data: 22 bytes packed
# uint32 time_us, 3x int16 acc_low, 3x int16 acc_high, 3x int16 gyro
IMU_FMT = "<Ihhh hhh hhh"
IMU_SIZE = struct.calcsize(IMU_FMT)

# MMC5983MA: 18-bit ADC, ±8 Gauss range = ±800 µT
# midpoint = 131072 (2^17), scale = 800 / 131072 = 0.006103515625 µT/count
# Actually the MMC5983MA is ±8 Gauss = ±800 µT over 18 bits (0 to 262143)
# So scale = 1600 / 262144 µT/count, offset = 131072
MAG_SCALE = 1600.0 / 262144.0  # µT per count from midpoint
MAG_MIDPOINT = 131072

def parse_bin(filepath):
    data = Path(filepath).read_bytes()
    n = len(data)
    i = 0

    ns_records = []
    mag_records = []
    imu_records = []

    while i < n - 8:
        if data[i:i+4] != SYNC:
            nxt = data.find(SYNC, i + 1)
            if nxt == -1:
                break
            i = nxt
            continue

        if i + 6 > n:
            break
        msg_type = data[i+4]
        payload_len = data[i+5]
        frame_end = i + 6 + payload_len + 2
        if frame_end > n:
            break
        payload = data[i+6 : i+6+payload_len]

        if msg_type == NON_SENSOR_MSG and payload_len >= NS_SIZE:
            vals = struct.unpack(NS_FMT, payload[:NS_SIZE])
            ns_records.append({
                "time_us": vals[0],
                "roll": vals[1] / 100.0,    # centideg → deg
                "pitch": vals[2] / 100.0,
                "yaw": vals[3] / 100.0,
                "roll_cmd": vals[4] / 100.0,
                "rocket_state": vals[12],
            })

        elif msg_type == MMC5983MA_MSG and payload_len >= MAG_SIZE:
            vals = struct.unpack(MAG_FMT, payload[:MAG_SIZE])
            mx = (vals[1] - MAG_MIDPOINT) * MAG_SCALE
            my = (vals[2] - MAG_MIDPOINT) * MAG_SCALE
            mz = (vals[3] - MAG_MIDPOINT) * MAG_SCALE
            mag_mag = np.sqrt(mx*mx + my*my + mz*mz)
            mag_records.append({
                "time_us": vals[0],
                "mag_x": mx, "mag_y": my, "mag_z": mz,
                "mag_mag": mag_mag,
            })

        elif msg_type == ISM6HG256_MSG and payload_len >= IMU_SIZE:
            vals = struct.unpack(IMU_FMT, payload[:IMU_SIZE])
            # Low-G accel scale: depends on FS setting.
            # For 16g: 16*9.81/32768 = 0.004788 m/s² per LSB
            # Just store raw for now, we mainly want the direction
            imu_records.append({
                "time_us": vals[0],
                "acc_lx": vals[1], "acc_ly": vals[2], "acc_lz": vals[3],
                "acc_hx": vals[4], "acc_hy": vals[5], "acc_hz": vals[6],
                "gyro_x": vals[7], "gyro_y": vals[8], "gyro_z": vals[9],
            })

        i = frame_end

    return ns_records, mag_records, imu_records


def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_attitude.py <file.bin>")
        sys.exit(1)

    filepath = sys.argv[1]
    ns, mag, imu = parse_bin(filepath)
    print(f"NonSensor: {len(ns)}, Mag: {len(mag)}, IMU: {len(imu)} records")

    if not ns:
        print("No NonSensor records found!")
        sys.exit(1)

    # Convert to arrays
    ns_t = np.array([r["time_us"] for r in ns]) / 1e6  # seconds
    ns_t -= ns_t[0]  # relative time
    roll = np.array([r["roll"] for r in ns])
    pitch = np.array([r["pitch"] for r in ns])
    yaw = np.array([r["yaw"] for r in ns])

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=False)

    # Plot 1: Roll/Pitch/Yaw vs time
    ax = axes[0]
    ax.plot(ns_t, roll, label="Roll", alpha=0.8)
    ax.plot(ns_t, pitch, label="Pitch", alpha=0.8)
    ax.plot(ns_t, yaw, label="Yaw", alpha=0.8)
    ax.set_ylabel("Angle (deg)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Euler Angles from FlightComputer")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axhline(90, color='gray', ls='--', alpha=0.5, label="90°")
    ax.axhline(0, color='gray', ls='--', alpha=0.3)

    # Plot 2: Pitch detail (first 10s or all)
    ax = axes[1]
    t_max = min(ns_t[-1], 10.0)
    mask = ns_t <= t_max
    ax.plot(ns_t[mask], pitch[mask], 'b-', label="Pitch")
    ax.axhline(90, color='r', ls='--', alpha=0.7, label="+90° (nose up)")
    ax.axhline(-90, color='orange', ls='--', alpha=0.7, label="-90° (nose down)")
    ax.set_ylabel("Pitch (deg)")
    ax.set_xlabel("Time (s)")
    ax.set_title(f"Pitch Convergence (first {t_max:.0f}s)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 3: Magnetometer magnitude
    if mag:
        mag_t = np.array([r["time_us"] for r in mag]) / 1e6
        mag_t -= mag_t[0]
        mag_mag = np.array([r["mag_mag"] for r in mag])
        mag_x = np.array([r["mag_x"] for r in mag])
        mag_y = np.array([r["mag_y"] for r in mag])
        mag_z = np.array([r["mag_z"] for r in mag])

        ax = axes[2]
        ax.plot(mag_t, mag_mag, 'k-', alpha=0.8, label="|B|")
        ax.axhline(15, color='r', ls='--', alpha=0.5, label="15 µT (low limit)")
        ax.axhline(80, color='r', ls='--', alpha=0.5, label="80 µT (high limit)")
        ax.axhspan(25, 65, alpha=0.1, color='green', label="Earth field range")
        ax.set_ylabel("Magnitude (µT)")
        ax.set_xlabel("Time (s)")
        ax.set_title("Magnetometer |B| (sanity check range)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes[3]
        ax.plot(mag_t, mag_x, label="Bx", alpha=0.7)
        ax.plot(mag_t, mag_y, label="By", alpha=0.7)
        ax.plot(mag_t, mag_z, label="Bz", alpha=0.7)
        ax.set_ylabel("Field (µT)")
        ax.set_xlabel("Time (s)")
        ax.set_title("Magnetometer XYZ (body frame, after sensor rotation)")
        ax.legend()
        ax.grid(True, alpha=0.3)
    else:
        axes[2].text(0.5, 0.5, "No magnetometer data", transform=axes[2].transAxes,
                     ha='center', va='center', fontsize=14)
        axes[3].text(0.5, 0.5, "No magnetometer data", transform=axes[3].transAxes,
                     ha='center', va='center', fontsize=14)

    plt.tight_layout()

    out = Path(filepath).with_suffix(".png")
    plt.savefig(out, dpi=150)
    print(f"Saved: {out}")

    # Print summary stats
    print(f"\n--- Summary ---")
    print(f"Duration: {ns_t[-1]:.1f}s, {len(ns)} samples")
    print(f"Pitch: first={pitch[0]:.1f}°, last={pitch[-1]:.1f}°, min={pitch.min():.1f}°, max={pitch.max():.1f}°")
    print(f"Roll:  first={roll[0]:.1f}°, last={roll[-1]:.1f}°, min={roll.min():.1f}°, max={roll.max():.1f}°")
    print(f"Yaw:   first={yaw[0]:.1f}°, last={yaw[-1]:.1f}°, min={yaw.min():.1f}°, max={yaw.max():.1f}°")
    if mag:
        print(f"\nMag |B|: mean={mag_mag.mean():.1f} µT, min={mag_mag.min():.1f}, max={mag_mag.max():.1f}")
        print(f"Mag valid (15-80 µT): {np.sum((mag_mag >= 15) & (mag_mag <= 80))}/{len(mag_mag)} samples")


if __name__ == "__main__":
    main()
