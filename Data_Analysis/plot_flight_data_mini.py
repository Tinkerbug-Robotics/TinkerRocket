#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_flight_data_mini.py

Parses binary log files from the TinkerRocket Mini (OutComputer) flight computer
and generates flight data plots.

Sensors: ISM6HG256 (low-g + high-g accel + gyro), BMP585 (baro),
         MMC5983MA (magnetometer), GNSS, Power ADC

Frame format: [0xAA][0x55][0xAA][0x55] + type(1) + length(1) + payload(N) + CRC16(2)
CRC-16: poly=0x8001, init=0x0000, no reflection, big-endian CRC bytes
CRC computed over [type, length, payload]

Data types defined in libraries/TR_RocketComputerTypes/RocketComputerTypes.h

Usage:
    1. Set BINARY_FILE, OUTPUT_DIR, SHOW_PLOTS, and PLOTS near the bottom
    2. Run:  python plot_flight_data_mini.py
"""

import sys
import struct
import os
import math
import numpy as np
import matplotlib.pyplot as plt

# ---------- User settings ----------
DOT_SIZE = 18
FIGURE_DPI = 150
# ------------------------------------

# ---------- Sensor conversion constants ----------
# ISM6HG256: configurable full-scale.  Defaults match FlightComputer config.h
# Sensitivity: value = raw * (full_scale / 32768)
G_MS2 = 9.80665

# Default full-scale settings (updated from OUT_STATUS_QUERY if present)
ISM6_LOW_G_FS_G   = 16     # ±16 g
ISM6_HIGH_G_FS_G  = 256    # ±256 g
ISM6_GYRO_FS_DPS  = 4000   # ±4000 dps
ISM6_ROT_Z_DEG    = -45.0  # sensor → board frame rotation about +Z
MMC_ROT_Z_DEG     = 180.0  # sensor → board frame rotation about +Z

def ism6_scales(low_g_fs=ISM6_LOW_G_FS_G, high_g_fs=ISM6_HIGH_G_FS_G,
                gyro_fs=ISM6_GYRO_FS_DPS):
    """Compute m/s² per LSB and dps per LSB for ISM6HG256."""
    denom = 32768.0
    acc_low  = (low_g_fs  * 1000.0 / denom) * 1e-3 * G_MS2
    acc_high = (high_g_fs * 1000.0 / denom) * 1e-3 * G_MS2
    gyro     = (gyro_fs   * 1000.0 / denom) * 1e-3
    return acc_low, acc_high, gyro

# MMC5983MA: 18-bit centered, Gauss = centered * (8 / 131072), uT = Gauss * 100
MMC_UT_PER_COUNT = (8.0 * 100.0) / 131072.0  # ≈ 0.006104

# BMP585: temp in Q16 (degC * 65536), pressure in Q6 (Pa * 64)
# ------------------------------------

# ---------- Message type IDs (Mini) ----------
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

# Expected payload sizes for validation
MSG_EXPECTED_LEN = {
    MSG_OUT_STATUS_QUERY: 16,   # OutStatusQueryData
    MSG_GNSS:             42,
    MSG_ISM6HG256:        22,
    MSG_BMP585:           12,
    MSG_MMC5983MA:        16,
    MSG_NON_SENSOR:       None,  # 42 (legacy) or 43 (with pyro_status byte)
    MSG_POWER:            10,
    MSG_START_LOGGING:    None,  # variable / no payload
    MSG_END_FLIGHT:       None,
    MSG_LORA:             49,
}

# Struct formats (little-endian, packed)
# GNSSData: 42 bytes
FMT_GNSS = '<I HBB BBB H BBB iii iii BB'
# ISM6HG256Data: 22 bytes (time_us + 3×Vec3i16)
FMT_ISM6 = '<I hhh hhh hhh'
# BMP585Data: 12 bytes (time_us, temp_q16:i32, press_q6:u32)
FMT_BMP585 = '<I i I'
# MMC5983MAData: 16 bytes (time_us, mag_x/y/z:u32)
FMT_MMC = '<I III'
# POWERData: 10 bytes (time_us, voltage_raw:u16, current_raw:i16, soc_raw:i16)
FMT_POWER = '<I Hhh'
# NonSensorData: 42 bytes (q0-q3 as int16*10000, roll_cmd centideg)
FMT_NONSENSOR_42 = '<I hhhhh iii iii BB h'     # legacy (no pyro_status)
FMT_NONSENSOR_43 = '<I hhhhh iii iii BB h B'  # current (with pyro_status byte)
# OutStatusQueryData: 16 bytes
FMT_STATUS_QUERY = '<B H H hh B hhh'

SYNC = b'\xAA\x55\xAA\x55'

# Rocket state names
ROCKET_STATES = {
    0: "INIT",
    1: "READY",
    2: "PRELAUNCH",
    3: "INFLIGHT",
    4: "LANDED",
}

# NonSensor flags
NSF_ALT_LANDED  = (1 << 0)
NSF_ALT_APOGEE  = (1 << 1)
NSF_VEL_APOGEE  = (1 << 2)
NSF_LAUNCH      = (1 << 3)
NSF_BURNOUT     = (1 << 4)
NSF_GUIDANCE    = (1 << 5)
NSF_PYRO1_ARMED = (1 << 6)
NSF_PYRO2_ARMED = (1 << 7)

# Pyro status byte bits
PSF_CH1_CONT  = (1 << 0)
PSF_CH2_CONT  = (1 << 1)
PSF_CH1_FIRED = (1 << 2)
PSF_CH2_FIRED = (1 << 3)


# ---------- CRC-16 (Rob Tillaart CRC library defaults) ----------
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


# ---------- Sensor decoding helpers ----------

def decode_bmp585_pressure(press_q6):
    """Decode BMP585 pressure from Q6 uint32 to Pa."""
    return float(press_q6) / 64.0

def decode_bmp585_temp(temp_q16):
    """Decode BMP585 temperature from Q16 int32 to °C."""
    return float(temp_q16) / 65536.0

def decode_mmc_centered(raw_u32):
    """Center an 18-bit MMC5983MA raw reading and convert to µT."""
    raw18 = raw_u32 & 0x3FFFF
    centered = raw18 - 131072
    return centered * MMC_UT_PER_COUNT

def decode_voltage(raw_u16):
    """Decode battery voltage from uint16 to V."""
    return (float(raw_u16) / 65535.0) * 10.0

def decode_current(raw_i16):
    """Decode battery current from int16 to mA."""
    return (float(raw_i16) / 32767.0) * 10000.0

def decode_soc(raw_i16):
    """Decode battery SOC from int16 to %."""
    return (float(raw_i16) * 150.0 / 32767.0) - 25.0

def pressure_to_altitude(p_pa, p0=101325.0):
    """Barometric altitude from pressure (Pa) using ISA model. Returns m."""
    if p_pa <= 0 or p0 <= 0:
        return 0.0
    return 44330.0 * (1.0 - (p_pa / p0) ** (1.0 / 5.255))


# ---------- Binary parser ----------
def parse_binary_file(filepath):
    """
    Parse a binary log file from the TinkerRocket Mini (OutComputer).

    Returns:
        records: dict mapping sensor name to list of parsed dicts
        stats:   dict with parsing statistics
        config:  dict with sensor configuration from OUT_STATUS_QUERY
    """
    with open(filepath, 'rb') as f:
        data = f.read()

    file_size = len(data)
    records = {
        "GNSS":       [],
        "ISM6HG256":  [],
        "BMP585":     [],
        "MMC5983MA":  [],
        "NonSensor":  [],
        "POWER":      [],
    }

    config = {
        "low_g_fs_g":    ISM6_LOW_G_FS_G,
        "high_g_fs_g":   ISM6_HIGH_G_FS_G,
        "gyro_fs_dps":   ISM6_GYRO_FS_DPS,
        "ism6_rot_z_deg": ISM6_ROT_Z_DEG,
        "mmc_rot_z_deg":  MMC_ROT_Z_DEG,
        "hg_bias": (0.0, 0.0, 0.0),
    }

    stats = {
        "file_size": file_size,
        "total_frames": 0,
        "good_crc": 0,
        "bad_crc": 0,
        "type_counts": {},
    }

    # We'll collect raw ISM6 data and apply conversion after parsing,
    # because the full-scale config may appear in an OUT_STATUS_QUERY frame.
    ism6_raw = []
    config_seen = False

    pos = 0
    while pos < file_size - 8:
        idx = data.find(SYNC, pos)
        if idx == -1:
            break
        pos = idx + 4

        if pos + 2 > file_size:
            break

        msg_type = data[pos]
        msg_len = data[pos + 1]
        pos += 2

        # Validate known types
        if msg_type in MSG_EXPECTED_LEN:
            expected = MSG_EXPECTED_LEN[msg_type]
            if expected is not None and msg_len != expected:
                pos -= 1
                continue

        if pos + msg_len + 2 > file_size:
            break

        payload = data[pos:pos + msg_len]

        # CRC (big-endian)
        crc_hi = data[pos + msg_len]
        crc_lo = data[pos + msg_len + 1]
        crc_received = (crc_hi << 8) | crc_lo
        crc_data = bytes([msg_type, msg_len]) + payload
        crc_computed = crc16(crc_data)

        stats["total_frames"] += 1

        if crc_received != crc_computed:
            stats["bad_crc"] += 1
            continue

        stats["good_crc"] += 1
        type_name = MSG_NAMES.get(msg_type, f"0x{msg_type:02X}")
        stats["type_counts"][type_name] = stats["type_counts"].get(type_name, 0) + 1

        pos += msg_len + 2

        try:
            if msg_type == MSG_OUT_STATUS_QUERY:
                if msg_len >= 16:
                    fields = struct.unpack(FMT_STATUS_QUERY, payload[:16])
                    config["low_g_fs_g"]  = fields[0]
                    config["high_g_fs_g"] = fields[1]
                    config["gyro_fs_dps"] = fields[2]
                    config["ism6_rot_z_deg"] = fields[3] / 100.0
                    config["mmc_rot_z_deg"]  = fields[4] / 100.0
                    fmt_ver = fields[5]
                    if fmt_ver >= 2:
                        config["hg_bias"] = (
                            fields[6] / 100.0,
                            fields[7] / 100.0,
                            fields[8] / 100.0,
                        )
                    config_seen = True

            elif msg_type == MSG_GNSS:
                fields = struct.unpack(FMT_GNSS, payload)
                records["GNSS"].append({
                    "time_us":   fields[0],
                    "year":      fields[1],
                    "month":     fields[2],
                    "day":       fields[3],
                    "hour":      fields[4],
                    "minute":    fields[5],
                    "second":    fields[6],
                    "milli_sec": fields[7],
                    "fix_mode":  fields[8],
                    "num_sats":  fields[9],
                    "pdop":      fields[10] / 10.0,
                    "lat":       fields[11] * 1e-7,
                    "lon":       fields[12] * 1e-7,
                    "alt_m":     fields[13] / 1000.0,
                    "vel_e":     fields[14] / 1000.0,
                    "vel_n":     fields[15] / 1000.0,
                    "vel_u":     fields[16] / 1000.0,
                    "h_acc_m":   fields[17],
                    "v_acc_m":   fields[18],
                })

            elif msg_type == MSG_ISM6HG256:
                fields = struct.unpack(FMT_ISM6, payload)
                # Store raw int16 values; convert after parsing to use final config
                ism6_raw.append({
                    "time_us": fields[0],
                    "lg_x": fields[1], "lg_y": fields[2], "lg_z": fields[3],
                    "hg_x": fields[4], "hg_y": fields[5], "hg_z": fields[6],
                    "gy_x": fields[7], "gy_y": fields[8], "gy_z": fields[9],
                })

            elif msg_type == MSG_BMP585:
                fields = struct.unpack(FMT_BMP585, payload)
                records["BMP585"].append({
                    "time_us":     fields[0],
                    "temperature": decode_bmp585_temp(fields[1]),
                    "pressure_pa": decode_bmp585_pressure(fields[2]),
                })

            elif msg_type == MSG_MMC5983MA:
                fields = struct.unpack(FMT_MMC, payload)
                # Store raw; will rotate after parsing when we have config
                records["MMC5983MA"].append({
                    "time_us":  fields[0],
                    "raw_x":    fields[1],
                    "raw_y":    fields[2],
                    "raw_z":    fields[3],
                })

            elif msg_type == MSG_NON_SENSOR and msg_len in (42, 43):
                if msg_len == 43:
                    fields = struct.unpack(FMT_NONSENSOR_43, payload)
                    pyro_status = fields[15]
                else:
                    fields = struct.unpack(FMT_NONSENSOR_42, payload)
                    pyro_status = 0
                q0 = fields[1] / 10000.0
                q1 = fields[2] / 10000.0
                q2 = fields[3] / 10000.0
                q3 = fields[4] / 10000.0
                # Derive Euler angles from quaternion (ZYX convention)
                import math as _m
                roll  = _m.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
                pitch = _m.asin(max(-1, min(1, 2*(q0*q2 - q3*q1))))
                yaw   = _m.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
                flags = fields[12]
                records["NonSensor"].append({
                    "time_us":       fields[0],
                    "q0":            q0,
                    "q1":            q1,
                    "q2":            q2,
                    "q3":            q3,
                    "roll":          _m.degrees(roll),
                    "pitch":         _m.degrees(pitch),
                    "yaw":           _m.degrees(yaw),
                    "roll_cmd":      fields[5] / 100.0,
                    "e_pos":         fields[6] / 100.0,  # cm → m
                    "n_pos":         fields[7] / 100.0,
                    "u_pos":         fields[8] / 100.0,
                    "e_vel":         fields[9] / 100.0,  # cm/s → m/s
                    "n_vel":         fields[10] / 100.0,
                    "u_vel":         fields[11] / 100.0,
                    "flags":         flags,
                    "rocket_state":  fields[13],
                    "baro_alt_rate": fields[14] * 0.1,  # dm/s → m/s
                    "alt_landed":    bool(flags & NSF_ALT_LANDED),
                    "alt_apogee":    bool(flags & NSF_ALT_APOGEE),
                    "vel_apogee":    bool(flags & NSF_VEL_APOGEE),
                    "launch":        bool(flags & NSF_LAUNCH),
                    "pyro1_armed":   bool(flags & NSF_PYRO1_ARMED),
                    "pyro2_armed":   bool(flags & NSF_PYRO2_ARMED),
                    "pyro1_cont":    bool(pyro_status & PSF_CH1_CONT),
                    "pyro2_cont":    bool(pyro_status & PSF_CH2_CONT),
                    "pyro1_fired":   bool(pyro_status & PSF_CH1_FIRED),
                    "pyro2_fired":   bool(pyro_status & PSF_CH2_FIRED),
                })

            elif msg_type == MSG_POWER:
                fields = struct.unpack(FMT_POWER, payload)
                if fields[0] != 0 or fields[1] != 0 or fields[2] != 0 or fields[3] != 0:
                    records["POWER"].append({
                        "time_us":  fields[0],
                        "voltage":  decode_voltage(fields[1]),
                        "current":  decode_current(fields[2]),
                        "soc":      decode_soc(fields[3]),
                    })

        except struct.error:
            pass

    # --- Post-process ISM6 raw data with final config ---
    acc_low_scale, acc_high_scale, gyro_scale = ism6_scales(
        config["low_g_fs_g"], config["high_g_fs_g"], config["gyro_fs_dps"])

    rot_rad = math.radians(config["ism6_rot_z_deg"])
    c_rot, s_rot = math.cos(rot_rad), math.sin(rot_rad)
    hg_bx, hg_by, hg_bz = config["hg_bias"]

    for r in ism6_raw:
        # Low-g accel → m/s², rotate to board frame
        lg_x = r["lg_x"] * acc_low_scale
        lg_y = r["lg_y"] * acc_low_scale
        lg_z = r["lg_z"] * acc_low_scale
        # High-g accel → m/s², rotate, subtract bias
        hg_x = r["hg_x"] * acc_high_scale
        hg_y = r["hg_y"] * acc_high_scale
        hg_z = r["hg_z"] * acc_high_scale
        # Gyro → dps, rotate
        gy_x = r["gy_x"] * gyro_scale
        gy_y = r["gy_y"] * gyro_scale
        gy_z = r["gy_z"] * gyro_scale

        records["ISM6HG256"].append({
            "time_us":     r["time_us"],
            "low_acc_x":   lg_x * c_rot - lg_y * s_rot,
            "low_acc_y":   lg_x * s_rot + lg_y * c_rot,
            "low_acc_z":   lg_z,
            "high_acc_x":  (hg_x * c_rot - hg_y * s_rot) - hg_bx,
            "high_acc_y":  (hg_x * s_rot + hg_y * c_rot) - hg_by,
            "high_acc_z":  hg_z - hg_bz,
            "gyro_x":      gy_x * c_rot - gy_y * s_rot,
            "gyro_y":      gy_x * s_rot + gy_y * c_rot,
            "gyro_z":      gy_z,
        })

    # --- Post-process MMC raw data with rotation ---
    mmc_rad = math.radians(config["mmc_rot_z_deg"])
    c_mmc, s_mmc = math.cos(mmc_rad), math.sin(mmc_rad)
    for rec in records["MMC5983MA"]:
        mx = decode_mmc_centered(rec.pop("raw_x"))
        my = decode_mmc_centered(rec.pop("raw_y"))
        mz = decode_mmc_centered(rec.pop("raw_z"))
        rec["mag_x"] = mx * c_mmc - my * s_mmc
        rec["mag_y"] = mx * s_mmc + my * c_mmc
        rec["mag_z"] = mz

    # Sort each sensor type by timestamp so MRAM ring-buffer drain
    # (which can interleave older pre-launch frames with newer ones)
    # doesn't produce backward time jumps in plots or analysis.
    for key in records:
        records[key].sort(key=lambda r: r["time_us"])

    return records, stats, config


# ---------- Plotting helpers ----------

def filter_records_by_time(records, t0_global, t_start, t_end):
    """Return a new records dict with only samples inside [t_start, t_end] (seconds)."""
    us_start = t0_global + t_start * 1e6
    us_end   = t0_global + t_end   * 1e6
    filtered = {}
    for name, recs in records.items():
        if recs and "time_us" in recs[0]:
            filtered[name] = [r for r in recs if us_start <= r["time_us"] <= us_end]
        else:
            filtered[name] = recs
    return filtered


def get_array(record_list, key):
    """Extract array of a single key from record list."""
    return np.array([r[key] for r in record_list])


def mark_endpoints(ax, x, y):
    """Mark start and end points."""
    if len(x) == 0:
        return
    ax.scatter([x[0]], [y[0]], s=48, marker='o', color='g', zorder=5)
    ax.scatter([x[-1]], [y[-1]], s=48, marker='o', color='r', zorder=5)


def mark_endpoints_3d(ax, x, y, z):
    """Mark start and end points on a 3D axis."""
    if len(x) == 0:
        return
    ax.scatter([x[0]], [y[0]], [z[0]], s=48, marker='o', color='g', zorder=5, label='Start')
    ax.scatter([x[-1]], [y[-1]], [z[-1]], s=48, marker='o', color='r', zorder=5, label='End')


def lla_to_ecef(lat_deg, lon_deg, h_m):
    """Convert lat/lon (deg) + height (m) arrays to ECEF X,Y,Z (m). WGS84."""
    a  = 6378137.0
    f  = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat, cos_lat = np.sin(lat), np.cos(lat)
    sin_lon, cos_lon = np.sin(lon), np.cos(lon)
    N = a / np.sqrt(1.0 - e2 * sin_lat**2)
    x = (N + h_m) * cos_lat * cos_lon
    y = (N + h_m) * cos_lat * sin_lon
    z = ((1.0 - e2) * N + h_m) * sin_lat
    return x, y, z


def gnss_to_enu(gnss_records):
    """Convert GNSS lat/lon/alt to local ENU (m) relative to first valid fix."""
    if not gnss_records:
        return np.array([]), np.array([]), np.array([])

    lat = get_array(gnss_records, "lat")
    lon = get_array(gnss_records, "lon")
    alt = get_array(gnss_records, "alt_m")

    valid = (lat != 0) & (lon != 0) & np.isfinite(lat) & np.isfinite(lon) & np.isfinite(alt)
    if not np.any(valid):
        return np.array([]), np.array([]), np.array([])

    x, y, z = lla_to_ecef(lat, lon, alt)

    i0 = int(np.argmax(valid))
    E = x - x[i0]
    N = y - y[i0]
    U = z - z[i0]

    E[~valid] = np.nan
    N[~valid] = np.nan
    U[~valid] = np.nan

    return E, N, U


# ---------- Plot functions ----------

def plot_acc_sats(records, t0_global):
    """ISM6 low-g acceleration X and GPS satellite count."""
    imu = records["ISM6HG256"]
    gnss = records["GNSS"]
    if not imu and not gnss:
        return None

    fig, ax1 = plt.subplots(figsize=(12, 5))

    if imu:
        t_imu = (get_array(imu, "time_us") - t0_global) / 1e6
        acc_x = get_array(imu, "low_acc_x")
        ax1.scatter(t_imu, acc_x, s=DOT_SIZE, alpha=0.6, label="Low-G Acc X (m/s²)")
        ax1.set_ylabel("Acceleration (m/s²)", color='tab:blue')
        ax1.tick_params(axis='y', labelcolor='tab:blue')

    if gnss:
        ax2 = ax1.twinx()
        t_gnss = (get_array(gnss, "time_us") - t0_global) / 1e6
        sats = get_array(gnss, "num_sats")
        ax2.scatter(t_gnss, sats, s=DOT_SIZE, color='tab:orange', alpha=0.6, label="Satellites")
        ax2.set_ylabel("Satellite Count", color='tab:orange')
        ax2.tick_params(axis='y', labelcolor='tab:orange')

    ax1.set_xlabel("Time (s)")
    ax1.set_title("ISM6HG256 Low-G Accel & GPS Satellites")
    ax1.grid(True, alpha=0.3)

    lines1, labels1 = ax1.get_legend_handles_labels()
    if gnss:
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    else:
        ax1.legend()

    fig.tight_layout()
    return fig


def plot_acc_low_g(records, t0_global):
    """ISM6HG256 low-g accelerometer X/Y/Z."""
    imu = records["ISM6HG256"]
    if not imu:
        return None

    t = (get_array(imu, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 5))

    for axis, color in [("x", "tab:blue"), ("y", "tab:orange"), ("z", "tab:green")]:
        vals = get_array(imu, f"low_acc_{axis}")
        ax.scatter(t, vals, s=8, alpha=0.4, color=color, label=f"Low-G {axis.upper()}")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.set_title("ISM6HG256 Low-G Accelerometer")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_acc_high_g(records, t0_global):
    """ISM6HG256 high-g accelerometer X/Y/Z."""
    imu = records["ISM6HG256"]
    if not imu:
        return None

    t = (get_array(imu, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 5))

    for axis, color in [("x", "tab:blue"), ("y", "tab:orange"), ("z", "tab:green")]:
        vals = get_array(imu, f"high_acc_{axis}")
        ax.scatter(t, vals, s=8, alpha=0.4, color=color, label=f"High-G {axis.upper()}")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.set_title("ISM6HG256 High-G Accelerometer")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_acc_overlay(records, t0_global):
    """Low-G vs High-G acceleration overlay (X axis)."""
    imu = records["ISM6HG256"]
    if not imu:
        return None

    t = (get_array(imu, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 5))

    ax.scatter(t, get_array(imu, "low_acc_x"), s=8, alpha=0.4,
               color='tab:blue', label="Low-G X")
    ax.scatter(t, get_array(imu, "high_acc_x"), s=8, alpha=0.4,
               color='tab:red', label="High-G X")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.set_title("Low-G vs High-G Accel (X axis)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_gyro_xyz(records, t0_global):
    """ISM6HG256 gyroscope X/Y/Z."""
    imu = records["ISM6HG256"]
    if not imu:
        return None

    t = (get_array(imu, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 5))

    for axis, color in [("x", "tab:blue"), ("y", "tab:orange"), ("z", "tab:green")]:
        vals = get_array(imu, f"gyro_{axis}")
        ax.scatter(t, vals, s=8, alpha=0.4, color=color, label=f"Gyro {axis.upper()}")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular Rate (deg/s)")
    ax.set_title("ISM6HG256 Gyroscope")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_pressure(records, t0_global):
    """BMP585 pressure and derived barometric altitude."""
    baro = records["BMP585"]
    if not baro:
        return None

    t = (get_array(baro, "time_us") - t0_global) / 1e6
    p = get_array(baro, "pressure_pa")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    ax1.scatter(t, p / 100.0, s=DOT_SIZE, alpha=0.5)  # Pa → hPa
    ax1.set_ylabel("Pressure (hPa)")
    ax1.set_title("BMP585 Pressure")
    ax1.grid(True, alpha=0.3)

    # Derive altitude using first sample as ground reference
    p0 = float(p[0]) if p[0] > 0 else 101325.0
    alt = np.array([pressure_to_altitude(pp, p0) for pp in p])
    ax2.scatter(t, alt, s=DOT_SIZE, alpha=0.5, color='tab:orange')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Baro Altitude (m AGL)")
    ax2.set_title("BMP585 Derived Altitude")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_temperature(records, t0_global):
    """BMP585 temperature."""
    baro = records["BMP585"]
    if not baro:
        return None

    t = (get_array(baro, "time_us") - t0_global) / 1e6
    temp = get_array(baro, "temperature")

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.scatter(t, temp, s=DOT_SIZE, alpha=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Temperature (°C)")
    ax.set_title("BMP585 Temperature")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_magnetometer(records, t0_global):
    """MMC5983MA magnetometer X/Y/Z."""
    mag = records["MMC5983MA"]
    if not mag:
        return None

    t = (get_array(mag, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 5))

    for axis, color in [("x", "tab:blue"), ("y", "tab:orange"), ("z", "tab:green")]:
        vals = get_array(mag, f"mag_{axis}")
        ax.scatter(t, vals, s=8, alpha=0.4, color=color, label=f"Mag {axis.upper()}")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Magnetic Field (µT)")
    ax.set_title("MMC5983MA Magnetometer")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_voltage(records, t0_global):
    """Battery voltage."""
    pwr = records["POWER"]
    if not pwr:
        return None

    t = (get_array(pwr, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.scatter(t, get_array(pwr, "voltage"), s=DOT_SIZE, alpha=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Voltage (V)")
    ax.set_title("Battery Voltage")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_current(records, t0_global):
    """Battery current."""
    pwr = records["POWER"]
    if not pwr:
        return None

    t = (get_array(pwr, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.scatter(t, get_array(pwr, "current"), s=DOT_SIZE, alpha=0.5, color='tab:red')
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Current (mA)")
    ax.set_title("Battery Current")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_soc(records, t0_global):
    """Battery state of charge."""
    pwr = records["POWER"]
    if not pwr:
        return None

    t = (get_array(pwr, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.scatter(t, get_array(pwr, "soc"), s=DOT_SIZE, alpha=0.5, color='tab:green')
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("State of Charge (%)")
    ax.set_title("Battery SOC")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_gps_altitude(records, t0_global):
    """GPS altitude and accuracy bands."""
    gnss = records["GNSS"]
    if not gnss:
        return None

    t = (get_array(gnss, "time_us") - t0_global) / 1e6
    alt = get_array(gnss, "alt_m")
    v_acc = get_array(gnss, "v_acc_m")

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.scatter(t, alt, s=DOT_SIZE, alpha=0.5, label="Alt")
    ax.fill_between(t, alt - v_acc, alt + v_acc, alpha=0.15, label="±v_acc")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("GPS Altitude")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_gps_alt_annotated(records, t0_global):
    """GPS altitude with apogee/landing annotations."""
    gnss = records["GNSS"]
    ns = records["NonSensor"]
    if not gnss:
        return None

    t_gnss = (get_array(gnss, "time_us") - t0_global) / 1e6
    alt = get_array(gnss, "alt_m")

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.scatter(t_gnss, alt, s=DOT_SIZE, alpha=0.5, label="GPS Alt")

    # Annotate apogee / landing from nav filter
    if ns:
        t_ns = (get_array(ns, "time_us") - t0_global) / 1e6
        for i, rec in enumerate(ns):
            if rec.get("alt_apogee") and i > 0 and not ns[i - 1].get("alt_apogee"):
                ax.axvline(t_ns[i], color='tab:red', linestyle='--', alpha=0.6,
                           label='Alt Apogee')
            if rec.get("alt_landed") and i > 0 and not ns[i - 1].get("alt_landed"):
                ax.axvline(t_ns[i], color='tab:green', linestyle='--', alpha=0.6,
                           label='Landed')
            if rec.get("launch") and i > 0 and not ns[i - 1].get("launch"):
                ax.axvline(t_ns[i], color='tab:orange', linestyle='--', alpha=0.6,
                           label='Launch')

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("GPS Altitude (Annotated)")
    # De-duplicate legend entries
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys())
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_gps_velocity(records, t0_global):
    """GPS velocity components (E/N/U) and speed."""
    gnss = records["GNSS"]
    if not gnss:
        return None

    t = (get_array(gnss, "time_us") - t0_global) / 1e6
    ve = get_array(gnss, "vel_e")
    vn = get_array(gnss, "vel_n")
    vu = get_array(gnss, "vel_u")
    spd = np.sqrt(ve**2 + vn**2 + vu**2)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    ax1.scatter(t, ve, s=8, alpha=0.5, label="East")
    ax1.scatter(t, vn, s=8, alpha=0.5, label="North")
    ax1.scatter(t, vu, s=8, alpha=0.5, label="Up")
    ax1.set_ylabel("Velocity (m/s)")
    ax1.set_title("GPS Velocity Components")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.scatter(t, spd, s=DOT_SIZE, alpha=0.5, color='tab:purple')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Speed (m/s)")
    ax2.set_title("GPS Speed")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_gps_ground_track(records):
    """2D ground track from GPS."""
    gnss = records["GNSS"]
    E, N, U = gnss_to_enu(gnss)
    if E.size == 0:
        return None

    valid = np.isfinite(E) & np.isfinite(N)
    if not np.any(valid):
        return None

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(E[valid], N[valid], s=DOT_SIZE, alpha=0.5)
    mark_endpoints(ax, E[valid], N[valid])
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("GPS Ground Track")
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_gps_enu_3d(records):
    """3D ENU trajectory from GPS."""
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    gnss = records["GNSS"]
    E, N, U = gnss_to_enu(gnss)
    if E.size == 0:
        return None

    valid = np.isfinite(E) & np.isfinite(N) & np.isfinite(U)
    if not np.any(valid):
        return None

    fig = plt.figure(figsize=(9, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(E[valid], N[valid], U[valid], s=12, alpha=0.5, label='Path')
    mark_endpoints_3d(ax, E[valid], N[valid], U[valid])
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Up (m)")
    ax.set_title("GPS ENU Trajectory (3D)")
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.legend()
    fig.tight_layout()
    return fig


def plot_gps_enu_2d(records):
    """2D ENU views from GPS: East-Up, North-Up, East-North."""
    gnss = records["GNSS"]
    E, N, U = gnss_to_enu(gnss)
    if E.size == 0:
        return None

    valid = np.isfinite(E) & np.isfinite(N) & np.isfinite(U)
    if not np.any(valid):
        return None
    Ev, Nv, Uv = E[valid], N[valid], U[valid]

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    titles = ["East vs Up", "North vs Up", "East vs North"]
    pairs = [(Ev, Uv), (Nv, Uv), (Ev, Nv)]
    xlabs = ["East (m)", "North (m)", "East (m)"]
    ylabs = ["Up (m)", "Up (m)", "North (m)"]

    for ax, title, (px, py), xl, yl in zip(axes, titles, pairs, xlabs, ylabs):
        ax.scatter(px, py, s=12, alpha=0.5)
        ax.axhline(0, linewidth=0.6, color='k')
        ax.axvline(0, linewidth=0.6, color='k')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)

    fig.suptitle("GPS ENU Views", fontsize=13)
    fig.tight_layout()
    return fig


def plot_attitude(records, t0_global):
    """Roll, Pitch, Yaw from nav filter."""
    ns = records["NonSensor"]
    if not ns:
        return None

    t = (get_array(ns, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 5))

    ax.scatter(t, get_array(ns, "roll"), s=8, alpha=0.5, color='tab:blue', label="Roll")
    ax.scatter(t, get_array(ns, "pitch"), s=8, alpha=0.5, color='tab:orange', label="Pitch")
    ax.scatter(t, get_array(ns, "yaw"), s=8, alpha=0.5, color='tab:green', label="Yaw")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Attitude (Roll / Pitch / Yaw)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_roll_cmd_gyro(records, t0_global):
    """Overlay: Roll, Roll Command, and Gyro X."""
    ns  = records["NonSensor"]
    imu = records["ISM6HG256"]
    if not ns and not imu:
        return None

    fig, ax1 = plt.subplots(figsize=(12, 5))

    lines, labels = [], []

    if ns:
        t_ns = (get_array(ns, "time_us") - t0_global) / 1e6
        l1 = ax1.scatter(t_ns, get_array(ns, "roll"), s=DOT_SIZE, alpha=0.5,
                         color='tab:blue', label="Roll (deg)")
        lines.append(l1)
        labels.append("Roll (deg)")

    if imu:
        t_imu = (get_array(imu, "time_us") - t0_global) / 1e6
        l2 = ax1.scatter(t_imu, get_array(imu, "gyro_x"), s=12, alpha=0.3,
                         color='tab:green', label="Gyro X (dps)")
        lines.append(l2)
        labels.append("Gyro X (dps)")

    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Roll / Gyro X (deg, deg/s)", color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax1.grid(True, alpha=0.3)

    # Roll command on separate right-hand axis with zero-aligned Y axes
    if ns:
        ax2 = ax1.twinx()
        roll_cmd = get_array(ns, "roll_cmd")
        l3 = ax2.scatter(t_ns, roll_cmd, s=DOT_SIZE, alpha=0.5,
                         color='tab:orange', label="Roll Command (deg)")

        cmd_min, cmd_max = float(np.nanmin(roll_cmd)), float(np.nanmax(roll_cmd))
        margin = max(0.5, (cmd_max - cmd_min) * 0.1)
        cmd_lo, cmd_hi = cmd_min - margin, cmd_max + margin

        ax1_lo, ax1_hi = ax1.get_ylim()

        # Fraction where 0 sits in each range
        f1 = -ax1_lo / (ax1_hi - ax1_lo) if ax1_hi != ax1_lo else 0.5
        f2 = -cmd_lo / (cmd_hi - cmd_lo) if cmd_hi != cmd_lo else 0.5

        f = max(f1, f2)
        f = max(0.05, min(0.95, f))

        # Scale each axis so 0 is at fraction f from the bottom
        ax1_range = max(abs(ax1_lo / f) if f > 0 else ax1_hi,
                        abs(ax1_hi / (1 - f)) if f < 1 else -ax1_lo)
        ax1.set_ylim(-f * ax1_range, (1 - f) * ax1_range)

        cmd_range = max(abs(cmd_lo / f) if f > 0 else cmd_hi,
                        abs(cmd_hi / (1 - f)) if f < 1 else -cmd_lo)
        ax2.set_ylim(-f * cmd_range, (1 - f) * cmd_range)

        ax2.set_ylabel("Roll Command (deg)", color='tab:orange')
        ax2.tick_params(axis='y', labelcolor='tab:orange')
        lines.append(l3)
        labels.append("Roll Command (deg)")

    ax1.legend(lines, labels, loc='upper right')
    ax1.set_title("Roll / Roll Cmd / Gyro X")
    fig.tight_layout()
    return fig


def plot_nav_altitude(records, t0_global):
    """Nav filter altitude (u_pos) and baro alt rate."""
    ns = records["NonSensor"]
    if not ns:
        return None

    t = (get_array(ns, "time_us") - t0_global) / 1e6
    u_pos = get_array(ns, "u_pos")
    alt_rate = get_array(ns, "baro_alt_rate")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    ax1.scatter(t, u_pos, s=DOT_SIZE, alpha=0.5)
    ax1.set_ylabel("Up Position (m)")
    ax1.set_title("Nav Filter Altitude")
    ax1.grid(True, alpha=0.3)

    ax2.scatter(t, alt_rate, s=DOT_SIZE, alpha=0.5, color='tab:orange')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Alt Rate (m/s)")
    ax2.set_title("Barometric Altitude Rate")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_nav_velocity(records, t0_global):
    """Nav filter velocity (E/N/U)."""
    ns = records["NonSensor"]
    if not ns:
        return None

    t = (get_array(ns, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 5))

    ax.scatter(t, get_array(ns, "e_vel"), s=8, alpha=0.5, color='tab:blue', label="East")
    ax.scatter(t, get_array(ns, "n_vel"), s=8, alpha=0.5, color='tab:orange', label="North")
    ax.scatter(t, get_array(ns, "u_vel"), s=8, alpha=0.5, color='tab:green', label="Up")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Nav Filter Velocity (ENU)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_rocket_state(records, t0_global):
    """Rocket state over time."""
    ns = records["NonSensor"]
    if not ns:
        return None

    t = (get_array(ns, "time_us") - t0_global) / 1e6
    states = get_array(ns, "rocket_state")

    fig, ax = plt.subplots(figsize=(12, 3))
    ax.scatter(t, states, s=DOT_SIZE, alpha=0.6)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("State")
    ax.set_title("Rocket State")
    ax.set_yticks(list(ROCKET_STATES.keys()))
    ax.set_yticklabels(list(ROCKET_STATES.values()))
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_nav_pos_3d(records, t0_global):
    """3D nav filter position (ENU)."""
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    ns = records["NonSensor"]
    if not ns:
        return None

    e = get_array(ns, "e_pos")
    n = get_array(ns, "n_pos")
    u = get_array(ns, "u_pos")

    fig = plt.figure(figsize=(9, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(e, n, u, s=8, alpha=0.4, label='Nav Path')
    mark_endpoints_3d(ax, e, n, u)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Up (m)")
    ax.set_title("Nav Filter Position (3D)")
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.legend()
    fig.tight_layout()
    return fig


def plot_nav_pos_2d(records, t0_global):
    """2D nav filter position views: East-Up, North-Up, East-North."""
    ns = records["NonSensor"]
    if not ns:
        return None

    e = get_array(ns, "e_pos")
    n = get_array(ns, "n_pos")
    u = get_array(ns, "u_pos")

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    titles = ["East vs Up", "North vs Up", "East vs North"]
    pairs = [(e, u), (n, u), (e, n)]
    xlabs = ["East (m)", "North (m)", "East (m)"]
    ylabs = ["Up (m)", "Up (m)", "North (m)"]

    for ax, title, (px, py), xl, yl in zip(axes, titles, pairs, xlabs, ylabs):
        ax.scatter(px, py, s=12, alpha=0.5, color='tab:blue')
        ax.axhline(0, linewidth=0.6, color='k')
        ax.axvline(0, linewidth=0.6, color='k')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)

    fig.suptitle("Nav Filter Position (ENU)", fontsize=13)
    fig.tight_layout()
    return fig


def plot_press_vs_gps_alt(records, t0_global):
    """Overlay: BMP585-derived altitude vs GPS altitude."""
    baro = records["BMP585"]
    gnss = records["GNSS"]
    if not baro and not gnss:
        return None

    fig, ax = plt.subplots(figsize=(12, 5))

    if baro:
        t_b = (get_array(baro, "time_us") - t0_global) / 1e6
        p = get_array(baro, "pressure_pa")
        p0 = float(p[0]) if p[0] > 0 else 101325.0
        alt_baro = np.array([pressure_to_altitude(pp, p0) for pp in p])
        ax.scatter(t_b, alt_baro, s=8, alpha=0.5, color='tab:blue', label="Baro Alt (m)")

    if gnss:
        t_g = (get_array(gnss, "time_us") - t0_global) / 1e6
        alt_gps = get_array(gnss, "alt_m")
        # Offset GPS alt to start at ~0 like baro
        valid = np.isfinite(alt_gps) & (alt_gps != 0)
        if np.any(valid):
            alt_gps_rel = alt_gps - alt_gps[np.argmax(valid)]
            ax.scatter(t_g, alt_gps_rel, s=8, alpha=0.5, color='tab:orange',
                       label="GPS Alt (m, rel)")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Baro vs GPS Altitude")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_nav_vs_gps_pos(records, t0_global):
    """Nav filter position vs GPS-derived ENU in 2D views."""
    gnss = records["GNSS"]
    ns = records["NonSensor"]
    if not gnss and not ns:
        return None

    gE, gN, gU = gnss_to_enu(gnss)
    nE = get_array(ns, "e_pos") if ns else np.array([])
    nN = get_array(ns, "n_pos") if ns else np.array([])
    nU = get_array(ns, "u_pos") if ns else np.array([])

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    titles = ["East vs Up", "North vs Up", "East vs North"]
    g_pairs = [(gE, gU), (gN, gU), (gE, gN)]
    n_pairs = [(nE, nU), (nN, nU), (nE, nN)]
    x_labs = ["East (m)", "North (m)", "East (m)"]
    y_labs = ["Up (m)", "Up (m)", "North (m)"]

    for ax, title, (gx, gy), (nx, ny), xl, yl in zip(
            axes, titles, g_pairs, n_pairs, x_labs, y_labs):
        if gx.size > 0:
            ax.scatter(gx, gy, s=12, alpha=0.4, color='tab:orange', label='GPS ENU')
        if nx.size > 0:
            ax.scatter(nx, ny, s=12, alpha=0.4, color='tab:blue', label='Nav Filter')
        ax.axhline(0, linewidth=0.6, color='k')
        ax.axvline(0, linewidth=0.6, color='k')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend()

    fig.suptitle("Nav Filter vs GPS-derived Position", fontsize=13)
    fig.tight_layout()
    return fig


# ---------- User Input Variables ----------
# Set these before running the script:

#BINARY_FILE = "/Users/christianpedersen/Documents/Hobbies/Model Rockets/TestFlights/2026_03_08/Raw Downloads/Goblin Flight 2 F52/flight_20260308_190239.bin"
#BINARY_FILE = "/Users/christianpedersen/Downloads/flight_20260309_233258.bin"
BINARY_FILE = "/Users/christianpedersen/Downloads/flight_20260331_192916.bin"
OUTPUT_DIR  = "/Users/christianpedersen/Downloads/simflight_20260331_analysis"
SHOW_PLOTS  = False    # True = show interactive matplotlib windows
TIME_WINDOW = None     # (t_start, t_end) in seconds, or None for all data

# ---------- Plot chooser (True = generate, False = skip) ----------
PLOTS = {
    # Sensor data
    "acc_sats":          True,
    "acc_low_g":         True,
    "acc_high_g":        True,
    "acc_overlay":       True,
    "gyro":              True,
    "pressure":          True,
    "temperature":       True,
    "magnetometer":      False,   # X axis broken
    # Power
    "voltage":           True,
    "current":           True,
    "soc":               True,
    # GPS
    "gps_altitude":      True,
    "gps_alt_annotated": True,
    "gps_velocity":      True,
    "ground_track":      True,
    "gps_enu_3d":        True,
    "gps_enu_2d":        True,
    # Nav filter
    "attitude":          True,
    "roll_cmd_gyro":     True,
    "nav_altitude":      True,
    "nav_velocity":      True,
    "rocket_state":      True,
    "nav_pos_3d":        True,
    "nav_pos_2d":        True,
    # Comparisons
    "press_vs_gps_alt":  True,
    "nav_vs_gps_pos":    True,
}
# ------------------------------------------


# ---------- Main ----------

def main(filepath=BINARY_FILE, output_dir=OUTPUT_DIR, show_plots=SHOW_PLOTS,
         time_window=TIME_WINDOW, plots=PLOTS):
    if not os.path.isfile(filepath):
        print(f"Error: File not found: {filepath}")
        sys.exit(1)

    print(f"Parsing: {filepath}")
    print(f"File size: {os.path.getsize(filepath):,} bytes")
    print()

    records, stats, config = parse_binary_file(filepath)

    # Print stats
    print("=" * 60)
    print("PARSING STATISTICS")
    print("=" * 60)
    print(f"  File size:     {stats['file_size']:>12,} bytes")
    print(f"  Total frames:  {stats['total_frames']:>12,}")
    print(f"  Good CRC:      {stats['good_crc']:>12,}")
    print(f"  Bad CRC:       {stats['bad_crc']:>12,}")
    if stats['total_frames'] > 0:
        pct = 100.0 * stats['good_crc'] / stats['total_frames']
        print(f"  CRC pass rate: {pct:>11.1f}%")
    print()
    print("  Message counts:")
    for name, count in sorted(stats["type_counts"].items(), key=lambda x: -x[1]):
        print(f"    {name:<24s} {count:>8,}")
    print()

    # Print sensor configuration
    print("  Sensor configuration:")
    print(f"    Low-G FS:      ±{config['low_g_fs_g']}g")
    print(f"    High-G FS:     ±{config['high_g_fs_g']}g")
    print(f"    Gyro FS:       ±{config['gyro_fs_dps']} dps")
    print(f"    ISM6 rot Z:    {config['ism6_rot_z_deg']:.1f}°")
    print(f"    MMC rot Z:     {config['mmc_rot_z_deg']:.1f}°")
    print(f"    HG bias:       ({config['hg_bias'][0]:.2f}, "
          f"{config['hg_bias'][1]:.2f}, {config['hg_bias'][2]:.2f}) m/s²")
    print()

    # Determine global t0 (earliest timestamp)
    all_t0 = []
    for recs in records.values():
        if recs and "time_us" in recs[0]:
            all_t0.append(recs[0]["time_us"])
    if not all_t0:
        print("No data to plot!")
        sys.exit(0)
    t0_global = min(all_t0)

    # Apply time window filter
    if time_window is not None:
        tw_start, tw_end = time_window
        records = filter_records_by_time(records, t0_global, tw_start, tw_end)
        print(f"  Time window:   [{tw_start:.1f}, {tw_end:.1f}] s")

    # Print data summaries
    for name, recs in records.items():
        if recs:
            t_start = (recs[0].get("time_us", 0) - t0_global) / 1e6
            t_end = (recs[-1].get("time_us", 0) - t0_global) / 1e6
            dur = t_end - t_start
            print(f"  {name:<12s}: {len(recs):>8,} samples, "
                  f"t=[{t_start:.1f}, {t_end:.1f}] s  ({dur:.1f}s)")
    print()

    # Output directory
    if output_dir:
        out_dir = output_dir
    else:
        out_dir = os.path.dirname(os.path.abspath(filepath))
    os.makedirs(out_dir, exist_ok=True)

    base_name = os.path.splitext(os.path.basename(filepath))[0]
    if time_window is not None:
        base_name += f"_t{time_window[0]:.0f}-{time_window[1]:.0f}s"

    # Generate all plots
    plot_funcs = [
        # --- Sensor data ---
        ("acc_sats",         lambda: plot_acc_sats(records, t0_global)),
        ("acc_low_g",        lambda: plot_acc_low_g(records, t0_global)),
        ("acc_high_g",       lambda: plot_acc_high_g(records, t0_global)),
        ("acc_overlay",      lambda: plot_acc_overlay(records, t0_global)),
        ("gyro",             lambda: plot_gyro_xyz(records, t0_global)),
        ("pressure",         lambda: plot_pressure(records, t0_global)),
        ("temperature",      lambda: plot_temperature(records, t0_global)),
        ("magnetometer",     lambda: plot_magnetometer(records, t0_global)),
        # --- Power ---
        ("voltage",          lambda: plot_voltage(records, t0_global)),
        ("current",          lambda: plot_current(records, t0_global)),
        ("soc",              lambda: plot_soc(records, t0_global)),
        # --- GPS ---
        ("gps_altitude",     lambda: plot_gps_altitude(records, t0_global)),
        ("gps_alt_annotated",lambda: plot_gps_alt_annotated(records, t0_global)),
        ("gps_velocity",     lambda: plot_gps_velocity(records, t0_global)),
        ("ground_track",     lambda: plot_gps_ground_track(records)),
        ("gps_enu_3d",       lambda: plot_gps_enu_3d(records)),
        ("gps_enu_2d",       lambda: plot_gps_enu_2d(records)),
        # --- Nav filter ---
        ("attitude",         lambda: plot_attitude(records, t0_global)),
        ("roll_cmd_gyro",    lambda: plot_roll_cmd_gyro(records, t0_global)),
        ("nav_altitude",     lambda: plot_nav_altitude(records, t0_global)),
        ("nav_velocity",     lambda: plot_nav_velocity(records, t0_global)),
        ("rocket_state",     lambda: plot_rocket_state(records, t0_global)),
        ("nav_pos_3d",       lambda: plot_nav_pos_3d(records, t0_global)),
        ("nav_pos_2d",       lambda: plot_nav_pos_2d(records, t0_global)),
        # --- Comparisons ---
        ("press_vs_gps_alt", lambda: plot_press_vs_gps_alt(records, t0_global)),
        ("nav_vs_gps_pos",   lambda: plot_nav_vs_gps_pos(records, t0_global)),
    ]

    # Filter to enabled plots
    total = len(plot_funcs)
    plot_funcs = [(n, f) for n, f in plot_funcs if plots.get(n, True)]
    print(f"  Generating {len(plot_funcs)} of {total} plots")
    print()

    saved = 0
    for name, func in plot_funcs:
        fig = func()
        if fig is not None:
            png_path = os.path.join(out_dir, f"{base_name}_{name}.png")
            fig.savefig(png_path, dpi=FIGURE_DPI, bbox_inches='tight')
            print(f"  Saved: {png_path}")
            saved += 1
            if not show_plots:
                plt.close(fig)

    print(f"\n{saved} plots saved to {out_dir}")

    if show_plots:
        plt.show(block=True)
        plt.close('all')


if __name__ == "__main__":
    main()
