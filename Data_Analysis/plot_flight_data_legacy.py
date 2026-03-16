#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_flight_data.py

Parses binary log files from the TinkerRocket Legacy flight computer
and generates flight data plots.

Sensors: ICM45686 (6-axis IMU), H3LIS331 (high-G accel), MS5611 (baro),
         LIS3MDL (magnetometer), I2C GNSS, Power ADC

Frame format: [0xAA][0x55][0xAA][0x55] + type(1) + length(1) + payload(N) + CRC16(2)
CRC-16: poly=0x8001, init=0x0000, no reflection, big-endian CRC bytes
CRC computed over [type, length, payload]

Data types defined in Legacy Code/libraries/TR_RocketComputerTypes/RocketComputerTypes.h

Usage:
    1. Set BINARY_FILE, OUTPUT_DIR, SHOW_PLOTS, and PLOT_SELECT near the bottom
    2. Run:  python plot_flight_data.py
"""

import sys
import struct
import os
import numpy as np
import matplotlib.pyplot as plt

# ---------- User settings ----------
DOT_SIZE = 18
FIGURE_DPI = 150
# ------------------------------------

# ---------- Sensor conversion constants ----------
# From TR_Sensor_Data_Converter.h (Legacy)
# ICM45686: ±32g accel, ±4000 dps gyro (int32 raw counts)
ICM_ACC_MS2_PER_LSB  = 0.00059855   # m/s² per LSB
ICM_GYRO_DPS_PER_LSB = 0.00762777   # deg/s per LSB
ICM_TEMP_SENSITIVITY = 333.87       # LSB/°C
ICM_TEMP_OFFSET      = 21.0         # °C

# H3LIS331: ±200g high-G accelerometer (int16 raw counts)
H3LIS_ACC_MS2_PER_LSB = 0.95788     # m/s² per LSB

# LIS3MDL: magnetometer (int16 raw counts)
LIS3MDL_UT_PER_LSB = 0.014          # µT per LSB

# MS5611: pressure and temperature encodings
MS5611_P_MIN = 10.0    # hPa
MS5611_P_MAX = 1200.0  # hPa
# ------------------------------------

# ---------- Message type IDs (Legacy) ----------
MSG_OUT_STATUS_QUERY = 0xA0
MSG_GNSS             = 0xA1
MSG_ICM45686         = 0xA2
MSG_MS5611           = 0xA3
MSG_LIS3MDL          = 0xA4
MSG_NON_SENSOR       = 0xA5
MSG_POWER            = 0xA6
MSG_START_LOGGING    = 0xA7
MSG_END_FLIGHT       = 0xA8
MSG_H3LIS331         = 0xA9
MSG_LORA             = 0xF3

MSG_NAMES = {
    MSG_OUT_STATUS_QUERY: "OUT_STATUS_QUERY",
    MSG_GNSS:             "GNSS",
    MSG_ICM45686:         "ICM45686",
    MSG_MS5611:           "MS5611",
    MSG_LIS3MDL:          "LIS3MDL",
    MSG_NON_SENSOR:       "NonSensor",
    MSG_POWER:            "POWER",
    MSG_START_LOGGING:    "StartLogging",
    MSG_END_FLIGHT:       "EndFlight",
    MSG_H3LIS331:         "H3LIS331",
    MSG_LORA:             "LoRa",
}

# Expected payload sizes for validation
MSG_EXPECTED_LEN = {
    MSG_OUT_STATUS_QUERY: 1,
    MSG_GNSS:             42,
    MSG_ICM45686:         36,
    MSG_MS5611:           10,
    MSG_LIS3MDL:          10,
    MSG_NON_SENSOR:       65,
    MSG_POWER:            10,
    MSG_START_LOGGING:    1,
    MSG_END_FLIGHT:       1,
    MSG_H3LIS331:         10,
}

# Struct formats (little-endian, packed)
# GNSSData: 42 bytes
FMT_GNSS = '<I HBB BBB H BBB iii iii BB'
# ICM45686Data: 36 bytes (time_us, time_sync, acc_xyz, gyro_xyz, temp — all int32)
FMT_ICM = '<II iii iii i'
# MS5611Data: 10 bytes (time_us, pressure_u32, temperature_i16)
FMT_MS5611 = '<I I h'
# LIS3MDLData: 10 bytes (time_us, mag_xyz — int16)
FMT_LIS3MDL = '<I hhh'
# H3LIS331Data: 10 bytes (time_us, acc_xyz — int16)
FMT_H3LIS = '<I hhh'
# POWERData: 10 bytes (time_us, voltage_i16, current_i16, soc_i16)
FMT_POWER = '<I hhh'
# NonSensorData: 65 bytes
FMT_NONSENSOR = '<I ffff iii iii iiii BBBBB'

SYNC = b'\xAA\x55\xAA\x55'

# Rocket state names
ROCKET_STATES = {
    0: "INIT",
    1: "READY",
    2: "PRELAUNCH",
    3: "INFLIGHT",
    4: "LANDED",
    5: "COMPLETE",
}


# ---------- CRC-16 (Rob Tillaart CRC library defaults) ----------
# Build lookup table for fast CRC computation
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
def decode_ms5611_pressure(raw_u32):
    """Decode MS5611 pressure from uint32 to hPa (mbar)."""
    span = MS5611_P_MAX - MS5611_P_MIN
    return MS5611_P_MIN + float(raw_u32) * span / 4294967295.0

def decode_ms5611_temp(raw_i16):
    """Decode MS5611 temperature from int16 to °C."""
    ratio = (float(raw_i16) + 32768.0) / 65535.0
    return ratio * 100.0 - 40.0

def decode_icm_temp(raw_i32):
    """Decode ICM45686 temperature from raw int32 to °C."""
    # Truncate to int16 range (temp register is 16-bit)
    raw_i16 = max(-32768, min(32767, raw_i32))
    return (raw_i16 / ICM_TEMP_SENSITIVITY) + ICM_TEMP_OFFSET

def decode_voltage(raw_i16):
    """Decode battery voltage from int16 (treated as uint16) to V."""
    raw_u16 = raw_i16 & 0xFFFF
    return (float(raw_u16) / 65535.0) * 10.0

def decode_current(raw_i16):
    """Decode battery current from int16 to mA."""
    return (float(raw_i16) / 32767.0) * 10000.0

def decode_soc(raw_i16):
    """Decode battery SOC from int16 to %."""
    return (float(raw_i16) * 150.0 / 32767.0) - 25.0


# ---------- Binary parser ----------
def parse_binary_file(filepath):
    """
    Parse a binary log file from the TinkerRocket Legacy computer.

    Returns:
        records: dict mapping sensor name to list of parsed dicts
        stats:   dict with parsing statistics
    """
    with open(filepath, 'rb') as f:
        data = f.read()

    file_size = len(data)
    records = {
        "GNSS":     [],
        "ICM45686": [],
        "H3LIS331": [],
        "MS5611":   [],
        "LIS3MDL":  [],
        "NonSensor":[],
        "POWER":    [],
    }

    stats = {
        "file_size": file_size,
        "total_frames": 0,
        "good_crc": 0,
        "bad_crc": 0,
        "type_counts": {},
    }

    pos = 0
    while pos < file_size - 8:  # Minimum: sync(4)+type(1)+len(1)+crc(2)
        # Find next sync pattern
        idx = data.find(SYNC, pos)
        if idx == -1:
            break
        pos = idx + 4  # Skip past sync

        if pos + 2 > file_size:
            break

        msg_type = data[pos]
        msg_len = data[pos + 1]
        pos += 2

        # Validate: known type with expected length
        if msg_type in MSG_EXPECTED_LEN:
            expected = MSG_EXPECTED_LEN[msg_type]
            if msg_len != expected:
                # Bad length — probably false sync, skip one byte
                pos -= 1
                continue

        # Check we have enough data for payload + CRC
        if pos + msg_len + 2 > file_size:
            break

        payload = data[pos:pos + msg_len]

        # CRC is stored big-endian (MSB first)
        crc_hi = data[pos + msg_len]
        crc_lo = data[pos + msg_len + 1]
        crc_received = (crc_hi << 8) | crc_lo

        # CRC is over [type, length, payload]
        crc_data = bytes([msg_type, msg_len]) + payload
        crc_computed = crc16(crc_data)

        stats["total_frames"] += 1

        if crc_received != crc_computed:
            stats["bad_crc"] += 1
            # pos is at payload start, past the sync — just continue
            # data.find(SYNC, pos) will find the next sync correctly
            continue

        stats["good_crc"] += 1
        type_name = MSG_NAMES.get(msg_type, f"0x{msg_type:02X}")
        stats["type_counts"][type_name] = stats["type_counts"].get(type_name, 0) + 1

        # Advance past payload + CRC
        pos += msg_len + 2

        # Parse payload based on type
        try:
            if msg_type == MSG_GNSS:
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
                    "pdop":      fields[10] / 100.0,
                    "lat":       fields[11] * 1e-7,
                    "lon":       fields[12] * 1e-7,
                    "alt_m":     fields[13] / 1000.0,
                    "vel_e":     fields[14] / 1000.0,
                    "vel_n":     fields[15] / 1000.0,
                    "vel_u":     fields[16] / 1000.0,
                    "h_acc_m":   fields[17],
                    "v_acc_m":   fields[18],
                })

            elif msg_type == MSG_ICM45686:
                fields = struct.unpack(FMT_ICM, payload)
                records["ICM45686"].append({
                    "time_us":    fields[0],
                    "time_sync":  fields[1],
                    "acc_x":      fields[2] * ICM_ACC_MS2_PER_LSB,
                    "acc_y":      fields[3] * ICM_ACC_MS2_PER_LSB,
                    "acc_z":      fields[4] * ICM_ACC_MS2_PER_LSB,
                    "gyro_x":     fields[5] * ICM_GYRO_DPS_PER_LSB,
                    "gyro_y":     fields[6] * ICM_GYRO_DPS_PER_LSB,
                    "gyro_z":     fields[7] * ICM_GYRO_DPS_PER_LSB,
                    "temp":       decode_icm_temp(fields[8]),
                })

            elif msg_type == MSG_H3LIS331:
                fields = struct.unpack(FMT_H3LIS, payload)
                records["H3LIS331"].append({
                    "time_us":     fields[0],
                    "acc_high_x":  fields[1] * H3LIS_ACC_MS2_PER_LSB,
                    "acc_high_y":  fields[2] * H3LIS_ACC_MS2_PER_LSB,
                    "acc_high_z":  fields[3] * H3LIS_ACC_MS2_PER_LSB,
                })

            elif msg_type == MSG_MS5611:
                fields = struct.unpack(FMT_MS5611, payload)
                records["MS5611"].append({
                    "time_us":     fields[0],
                    "pressure":    decode_ms5611_pressure(fields[1]),
                    "temperature": decode_ms5611_temp(fields[2]),
                })

            elif msg_type == MSG_LIS3MDL:
                fields = struct.unpack(FMT_LIS3MDL, payload)
                records["LIS3MDL"].append({
                    "time_us": fields[0],
                    "mag_x":   fields[1] * LIS3MDL_UT_PER_LSB,
                    "mag_y":   fields[2] * LIS3MDL_UT_PER_LSB,
                    "mag_z":   fields[3] * LIS3MDL_UT_PER_LSB,
                })

            elif msg_type == MSG_NON_SENSOR:
                fields = struct.unpack(FMT_NONSENSOR, payload)
                records["NonSensor"].append({
                    "time_us":        fields[0],
                    "roll":           fields[1],     # float, degrees
                    "pitch":          fields[2],
                    "yaw":            fields[3],
                    "roll_cmd":       fields[4],
                    "e_pos":          fields[5] / 100.0,  # cm → m
                    "n_pos":          fields[6] / 100.0,
                    "u_pos":          fields[7] / 100.0,
                    "e_vel":          fields[8] / 100.0,  # cm/s → m/s
                    "n_vel":          fields[9] / 100.0,
                    "u_vel":          fields[10] / 100.0,
                    "pressure_alt":   float(fields[11]),        # meters (direct cast in firmware)
                    "altitude_rate":  fields[12] / 10.0,       # decimeters/s → m/s (×10 in firmware)
                    "max_alt":        float(fields[13]),        # meters (direct cast)
                    "max_speed":      float(fields[14]),        # m/s (direct cast)
                    "alt_landed":     bool(fields[15]),
                    "alt_apogee":     bool(fields[16]),
                    "vel_apogee":     bool(fields[17]),
                    "launch":         bool(fields[18]),
                    "rocket_state":   fields[19],
                })

            elif msg_type == MSG_POWER:
                fields = struct.unpack(FMT_POWER, payload)
                # Skip all-zero frames (power monitor not connected)
                if fields[0] != 0 or fields[1] != 0 or fields[2] != 0 or fields[3] != 0:
                    records["POWER"].append({
                        "time_us":  fields[0],
                        "voltage":  decode_voltage(fields[1]),
                        "current":  decode_current(fields[2]),
                        "soc":      decode_soc(fields[3]),
                    })

        except struct.error:
            pass  # Malformed payload, skip

    return records, stats


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
    """Convert GNSS lat/lon/alt to local ENU (m) relative to first valid fix.
    Returns E, N, U arrays (same length as gnss_records)."""
    if not gnss_records:
        return np.array([]), np.array([]), np.array([])

    lat = get_array(gnss_records, "lat")
    lon = get_array(gnss_records, "lon")
    alt = get_array(gnss_records, "alt_m")

    valid = (lat != 0) & (lon != 0) & np.isfinite(lat) & np.isfinite(lon) & np.isfinite(alt)
    if not np.any(valid):
        return np.array([]), np.array([]), np.array([])

    x, y, z = lla_to_ecef(lat, lon, alt)

    # First valid point as origin
    i0 = int(np.argmax(valid))
    E = x - x[i0]
    N = y - y[i0]
    U = z - z[i0]

    # NaN-out invalid points
    E[~valid] = np.nan
    N[~valid] = np.nan
    U[~valid] = np.nan

    return E, N, U


# ---------- Plot functions ----------

def plot_acc_sats(records, t0_global):
    """ICM45686 acceleration X and GPS satellite count."""
    imu = records["ICM45686"]
    gnss = records["GNSS"]
    if not imu and not gnss:
        return None

    fig, ax1 = plt.subplots(figsize=(12, 5))

    if imu:
        t_imu = (get_array(imu, "time_us") - t0_global) / 1e6
        acc_x = get_array(imu, "acc_x")
        ax1.scatter(t_imu, acc_x, s=DOT_SIZE, marker='o', alpha=0.6, label="ICM Acc X (m/s²)")
        ax1.set_ylabel("Acceleration (m/s²)", color='tab:blue')
        ax1.tick_params(axis='y', labelcolor='tab:blue')

    if gnss:
        ax2 = ax1.twinx()
        t_gnss = (get_array(gnss, "time_us") - t0_global) / 1e6
        sats = get_array(gnss, "num_sats")
        ax2.scatter(t_gnss, sats, s=DOT_SIZE, marker='o', color='tab:orange', alpha=0.6, label="Satellites")
        ax2.set_ylabel("Satellite Count", color='tab:orange')
        ax2.tick_params(axis='y', labelcolor='tab:orange')

    ax1.set_xlabel("Time (s)")
    ax1.set_title("ICM45686 Acceleration & GPS Satellites")
    ax1.grid(True, alpha=0.3)

    lines1, labels1 = ax1.get_legend_handles_labels()
    if gnss:
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    else:
        ax1.legend()

    fig.tight_layout()
    return fig


def plot_acc_icm(records, t0_global):
    """ICM45686 accelerometer X/Y/Z."""
    imu = records["ICM45686"]
    if not imu:
        return None

    t = (get_array(imu, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 5))

    for axis, color in [("x", "tab:blue"), ("y", "tab:orange"), ("z", "tab:green")]:
        vals = get_array(imu, f"acc_{axis}")
        ax.scatter(t, vals, s=8, marker='o', alpha=0.4, color=color, label=f"Acc {axis.upper()}")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.set_title("ICM45686 Accelerometer")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_acc_high_g(records, t0_global):
    """H3LIS331 high-G accelerometer X/Y/Z."""
    hg = records["H3LIS331"]
    if not hg:
        return None

    t = (get_array(hg, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 5))

    for axis, color in [("x", "tab:blue"), ("y", "tab:orange"), ("z", "tab:green")]:
        vals = get_array(hg, f"acc_high_{axis}")
        ax.scatter(t, vals, s=8, marker='o', alpha=0.4, color=color, label=f"High-G {axis.upper()}")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.set_title("H3LIS331 High-G Accelerometer (±200g)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_gyro_xyz(records, t0_global):
    """ICM45686 gyroscope X/Y/Z."""
    imu = records["ICM45686"]
    if not imu:
        return None

    t = (get_array(imu, "time_us") - t0_global) / 1e6
    fig, ax = plt.subplots(figsize=(12, 5))

    for axis, color in [("x", "tab:blue"), ("y", "tab:orange"), ("z", "tab:green")]:
        vals = get_array(imu, f"gyro_{axis}")
        ax.scatter(t, vals, s=12, marker='o', alpha=0.4, color=color, label=f"Gyro {axis.upper()}")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular Rate (deg/s)")
    ax.set_title("ICM45686 Gyroscope (±4000 dps)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_temperature(records, t0_global):
    """MS5611 and ICM45686 temperatures."""
    ms = records["MS5611"]
    imu = records["ICM45686"]
    if not ms and not imu:
        return None

    fig, ax = plt.subplots(figsize=(12, 4))

    if ms:
        t_ms = (get_array(ms, "time_us") - t0_global) / 1e6
        temp_ms = get_array(ms, "temperature")
        ax.scatter(t_ms, temp_ms, s=DOT_SIZE, marker='o', alpha=0.6, label="MS5611 Temp")

    if imu:
        t_imu = (get_array(imu, "time_us") - t0_global) / 1e6
        temp_imu = get_array(imu, "temp")
        ax.scatter(t_imu, temp_imu, s=DOT_SIZE, marker='o', alpha=0.6, label="ICM45686 Temp")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Temperature (°C)")
    ax.set_title("Temperatures")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_pressure(records, t0_global):
    """MS5611 barometric pressure."""
    ms = records["MS5611"]
    if not ms:
        return None

    t = (get_array(ms, "time_us") - t0_global) / 1e6
    press = get_array(ms, "pressure")

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.scatter(t, press, s=DOT_SIZE, marker='o', alpha=0.6, color='tab:purple')
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pressure (hPa)")
    ax.set_title("MS5611 Barometric Pressure")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_voltage(records, t0_global):
    """Battery voltage."""
    pwr = records["POWER"]
    if not pwr:
        return None

    t = (get_array(pwr, "time_us") - t0_global) / 1e6
    v = get_array(pwr, "voltage")

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.scatter(t, v, s=DOT_SIZE, marker='o', alpha=0.6, color='tab:green')
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
    i = get_array(pwr, "current")

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.scatter(t, i, s=DOT_SIZE, marker='o', alpha=0.6, color='tab:red')
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
    soc = get_array(pwr, "soc")

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.scatter(t, soc, s=DOT_SIZE, marker='o', alpha=0.6, color='tab:cyan')
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("SOC (%)")
    ax.set_title("Battery State of Charge")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_gps_altitude(records, t0_global):
    """GPS altitude."""
    gnss = records["GNSS"]
    if not gnss:
        return None

    t = (get_array(gnss, "time_us") - t0_global) / 1e6
    alt = get_array(gnss, "alt_m")

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.scatter(t, alt, s=DOT_SIZE, marker='o', alpha=0.6, color='tab:brown')
    mark_endpoints(ax, t, alt)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("GPS Altitude")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_gps_velocity(records, t0_global):
    """GPS velocity components."""
    gnss = records["GNSS"]
    if not gnss:
        return None

    t = (get_array(gnss, "time_us") - t0_global) / 1e6

    fig, ax = plt.subplots(figsize=(12, 5))
    for comp, color, label in [("vel_e", "tab:blue", "East"),
                                ("vel_n", "tab:orange", "North"),
                                ("vel_u", "tab:green", "Up")]:
        vals = get_array(gnss, comp)
        ax.scatter(t, vals, s=12, marker='o', alpha=0.4, color=color, label=label)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("GPS Velocity")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_gps_ground_track(records):
    """GPS ground track (lat/lon)."""
    gnss = records["GNSS"]
    if not gnss:
        return None

    lat = get_array(gnss, "lat")
    lon = get_array(gnss, "lon")

    # Filter out zero positions (no fix)
    mask = (lat != 0) & (lon != 0)
    if not np.any(mask):
        return None
    lat = lat[mask]
    lon = lon[mask]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(lon, lat, s=8, marker='o', alpha=0.5, c=np.arange(len(lat)), cmap='viridis')
    ax.scatter([lon[0]], [lat[0]], s=80, marker='^', color='g', zorder=5, label='Start')
    ax.scatter([lon[-1]], [lat[-1]], s=80, marker='v', color='r', zorder=5, label='End')
    ax.set_xlabel("Longitude (deg)")
    ax.set_ylabel("Latitude (deg)")
    ax.set_title("GPS Ground Track")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    fig.tight_layout()
    return fig


def plot_attitude(records, t0_global):
    """Roll/Pitch/Yaw from NonSensorData (flight computer estimates)."""
    ns = records["NonSensor"]
    if not ns:
        return None

    t = (get_array(ns, "time_us") - t0_global) / 1e6

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    for ax, key, label, color in zip(axes,
                                     ["roll", "pitch", "yaw"],
                                     ["Roll", "Pitch", "Yaw"],
                                     ["tab:blue", "tab:orange", "tab:green"]):
        vals = get_array(ns, key)
        ax.scatter(t, vals, s=8, marker='o', alpha=0.4, color=color)
        ax.set_ylabel(f"{label} (deg)")
        ax.set_title(label)
        ax.grid(True, alpha=0.3)

    # Overlay roll command on roll axis
    roll_cmd = get_array(ns, "roll_cmd")
    axes[0].scatter(t, roll_cmd, s=4, marker='o', alpha=0.3, color='tab:red', label='Roll Cmd')
    axes[0].legend()

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Attitude (Flight Computer Estimate)", fontsize=13)
    fig.tight_layout()
    return fig


def plot_nav_altitude(records, t0_global):
    """Navigation altitude and baro altitude rate from NonSensorData."""
    ns = records["NonSensor"]
    if not ns:
        return None

    t = (get_array(ns, "time_us") - t0_global) / 1e6

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    p_alt = get_array(ns, "pressure_alt")
    ax1.scatter(t, p_alt, s=DOT_SIZE, marker='o', alpha=0.5, color='tab:brown')
    ax1.set_ylabel("Altitude (m)")
    ax1.set_title("Pressure Altitude")
    ax1.grid(True, alpha=0.3)

    alt_rate = get_array(ns, "altitude_rate")
    ax2.scatter(t, alt_rate, s=DOT_SIZE, marker='o', alpha=0.5, color='tab:olive')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Altitude Rate (m/s)")
    ax2.set_title("Altitude Rate")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


def plot_nav_velocity(records, t0_global):
    """Navigation velocity from NonSensorData."""
    ns = records["NonSensor"]
    if not ns:
        return None

    t = (get_array(ns, "time_us") - t0_global) / 1e6

    fig, ax = plt.subplots(figsize=(12, 5))
    for comp, color, label in [("e_vel", "tab:blue", "East"),
                                ("n_vel", "tab:orange", "North"),
                                ("u_vel", "tab:green", "Up")]:
        vals = get_array(ns, comp)
        ax.scatter(t, vals, s=12, marker='o', alpha=0.4, color=color, label=label)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Navigation Velocity (Flight Computer)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_rocket_state(records, t0_global):
    """Rocket state timeline from NonSensorData."""
    ns = records["NonSensor"]
    if not ns:
        return None

    t = (get_array(ns, "time_us") - t0_global) / 1e6
    states = get_array(ns, "rocket_state")

    fig, ax = plt.subplots(figsize=(12, 3))
    ax.scatter(t, states, s=DOT_SIZE, marker='o', alpha=0.6, color='tab:purple')
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("State")
    ax.set_title("Rocket State")
    state_vals = sorted(set(states.tolist()) | set(ROCKET_STATES.keys()))
    ax.set_yticks([v for v in state_vals if v in ROCKET_STATES])
    ax.set_yticklabels([ROCKET_STATES[v] for v in state_vals if v in ROCKET_STATES])
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_magnetometer(records, t0_global):
    """LIS3MDL magnetometer X/Y/Z."""
    mag = records["LIS3MDL"]
    if not mag:
        return None

    t = (get_array(mag, "time_us") - t0_global) / 1e6

    fig, ax = plt.subplots(figsize=(12, 5))
    for axis, color in [("mag_x", "tab:blue"), ("mag_y", "tab:orange"), ("mag_z", "tab:green")]:
        vals = get_array(mag, axis)
        label = axis.replace("mag_", "").upper()
        ax.scatter(t, vals, s=12, marker='o', alpha=0.4, color=color, label=f"Mag {label}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Magnetic Field (µT)")
    ax.set_title("LIS3MDL Magnetometer")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


# ---------- New plots (from flightDataPlot_lla.py reference) ----------

def plot_acc_overlay(records, t0_global):
    """High-G vs ICM45686 acceleration overlay — one subplot per axis."""
    imu = records["ICM45686"]
    hg  = records["H3LIS331"]
    if not imu and not hg:
        return None

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axis_labels = ["X", "Y", "Z"]
    icm_keys = ["acc_x", "acc_y", "acc_z"]
    hg_keys  = ["acc_high_x", "acc_high_y", "acc_high_z"]

    for ax, lbl, icm_k, hg_k in zip(axes, axis_labels, icm_keys, hg_keys):
        if imu:
            t_imu = (get_array(imu, "time_us") - t0_global) / 1e6
            ax.scatter(t_imu, get_array(imu, icm_k), s=8, alpha=0.4,
                       color='tab:blue', label=f"ICM45686 {lbl}")
        if hg:
            t_hg = (get_array(hg, "time_us") - t0_global) / 1e6
            ax.scatter(t_hg, get_array(hg, hg_k), s=8, alpha=0.4,
                       color='tab:orange', label=f"H3LIS331 {lbl}")
        ax.set_ylabel(f"Acc {lbl} (m/s²)")
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("High-G vs ICM45686 Acceleration — Per Axis", fontsize=13)
    fig.tight_layout()
    return fig


def plot_gps_alt_annotated(records, t0_global):
    """GPS altitude with max-altitude callout annotation."""
    gnss = records["GNSS"]
    if not gnss:
        return None

    t   = (get_array(gnss, "time_us") - t0_global) / 1e6
    alt = get_array(gnss, "alt_m")
    valid = np.isfinite(alt)
    if not np.any(valid):
        return None

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.scatter(t[valid], alt[valid], s=DOT_SIZE, marker='o', alpha=0.6, color='tab:brown')
    mark_endpoints(ax, t[valid], alt[valid])

    # Annotate max altitude
    i_max = int(np.nanargmax(alt[valid]))
    t_max = t[valid][i_max]
    a_max = alt[valid][i_max]
    ax.annotate(f"Max: {a_max:.1f} m @ {t_max:.1f} s",
                xy=(t_max, a_max), xytext=(12, 12),
                textcoords="offset points",
                arrowprops=dict(arrowstyle="->", lw=1),
                fontsize=10, fontweight='bold')

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("GPS Altitude (m)")
    ax.set_title("GPS Altitude (annotated max)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_pressure_vs_gps_alt(records, t0_global):
    """Overlay: pressure altitude (NonSensor) vs GPS altitude (GNSS)."""
    ns   = records["NonSensor"]
    gnss = records["GNSS"]
    if not ns and not gnss:
        return None

    fig, ax = plt.subplots(figsize=(12, 5))

    if ns:
        t_ns = (get_array(ns, "time_us") - t0_global) / 1e6
        p_alt = get_array(ns, "pressure_alt")
        ax.scatter(t_ns, p_alt, s=DOT_SIZE, marker='o', alpha=0.5,
                   color='tab:blue', label="Pressure Altitude (m)")

    if gnss:
        t_gps = (get_array(gnss, "time_us") - t0_global) / 1e6
        gps_alt = get_array(gnss, "alt_m")
        # Shift GPS altitude to AGL (relative to first reading)
        valid = np.isfinite(gps_alt)
        if np.any(valid):
            gps_agl = gps_alt - gps_alt[valid][0]
            ax.scatter(t_gps[valid], gps_agl[valid], s=DOT_SIZE, marker='o', alpha=0.5,
                       color='tab:orange', label="GPS Altitude AGL (m)")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Pressure Altitude vs GPS Altitude")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_roll_cmd_gyro(records, t0_global):
    """Overlay: Roll, Roll Command, and Gyro X on one plot."""
    ns  = records["NonSensor"]
    imu = records["ICM45686"]
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

    # Roll command on separate right-hand axis with its own scale
    # Both axes are set so that 0 is at the same vertical position
    if ns:
        ax2 = ax1.twinx()
        roll_cmd = get_array(ns, "roll_cmd")
        l3 = ax2.scatter(t_ns, roll_cmd, s=DOT_SIZE, alpha=0.5,
                         color='tab:orange', label="Roll Command (deg)")

        # Align zero on both axes: compute symmetric limits that keep
        # 0 at the same fractional position on each axis
        cmd_min, cmd_max = float(np.nanmin(roll_cmd)), float(np.nanmax(roll_cmd))
        margin = max(0.5, (cmd_max - cmd_min) * 0.1)
        cmd_lo, cmd_hi = cmd_min - margin, cmd_max + margin

        ax1_lo, ax1_hi = ax1.get_ylim()

        # Fraction where 0 sits in each range
        f1 = -ax1_lo / (ax1_hi - ax1_lo) if ax1_hi != ax1_lo else 0.5
        f2 = -cmd_lo / (cmd_hi - cmd_lo) if cmd_hi != cmd_lo else 0.5

        # Use the larger fraction so both ranges fit, then adjust
        f = max(f1, f2)
        f = max(0.05, min(0.95, f))  # clamp to avoid degenerate limits

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


def plot_gps_enu_3d(records):
    """3D ENU trajectory from GPS LLA (origin at first valid fix)."""
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
    ax.scatter(E[valid], N[valid], U[valid], s=12, marker='o', alpha=0.5, label='Path')
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
    """2D ENU views from GPS LLA: East-Up, North-Up, East-North."""
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
    x_data = [Ev, Nv, Ev]
    y_data = [Uv, Uv, Nv]
    x_labs = ["East (m)", "North (m)", "East (m)"]
    y_labs = ["Up (m)",   "Up (m)",    "North (m)"]

    for ax, title, xd, yd, xl, yl in zip(axes, titles, x_data, y_data, x_labs, y_labs):
        ax.scatter(xd, yd, s=12, marker='o', alpha=0.5, label='Path')
        mark_endpoints(ax, xd, yd)
        ax.axhline(0, linewidth=0.6, color='k')
        ax.axvline(0, linewidth=0.6, color='k')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend()

    fig.suptitle("GPS ENU Views (origin at first fix)", fontsize=13)
    fig.tight_layout()
    return fig


def plot_nav_pos_3d(records, t0_global):
    """3D position from flight computer nav filter (NonSensorData)."""
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    ns = records["NonSensor"]
    if not ns:
        return None

    E = get_array(ns, "e_pos")
    N = get_array(ns, "n_pos")
    U = get_array(ns, "u_pos")

    fig = plt.figure(figsize=(9, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(E, N, U, s=12, marker='o', alpha=0.5, label='Path')
    mark_endpoints_3d(ax, E, N, U)
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

    E = get_array(ns, "e_pos")
    N = get_array(ns, "n_pos")
    U = get_array(ns, "u_pos")

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    titles = ["East vs Up", "North vs Up", "East vs North"]
    x_data = [E, N, E]
    y_data = [U, U, N]
    x_labs = ["East (m)", "North (m)", "East (m)"]
    y_labs = ["Up (m)",   "Up (m)",    "North (m)"]

    for ax, title, xd, yd, xl, yl in zip(axes, titles, x_data, y_data, x_labs, y_labs):
        ax.scatter(xd, yd, s=12, marker='o', alpha=0.5, label='Path')
        mark_endpoints(ax, xd, yd)
        ax.axhline(0, linewidth=0.6, color='k')
        ax.axvline(0, linewidth=0.6, color='k')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend()

    fig.suptitle("Nav Filter Position ENU", fontsize=13)
    fig.tight_layout()
    return fig


def plot_nav_vs_gps_pos(records, t0_global):
    """Overlay: Nav filter Position ENU vs GPS-derived ENU (East-North view)."""
    ns   = records["NonSensor"]
    gnss = records["GNSS"]
    if not ns and not gnss:
        return None

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    titles = ["East vs Up", "North vs Up", "East vs North"]

    # GPS ENU
    gE, gN, gU = gnss_to_enu(gnss)
    gvalid = np.isfinite(gE) & np.isfinite(gN) & np.isfinite(gU)
    gE, gN, gU = gE[gvalid], gN[gvalid], gU[gvalid]

    # Nav ENU
    nE = get_array(ns, "e_pos") if ns else np.array([])
    nN = get_array(ns, "n_pos") if ns else np.array([])
    nU = get_array(ns, "u_pos") if ns else np.array([])

    g_pairs = [(gE, gU), (gN, gU), (gE, gN)]
    n_pairs = [(nE, nU), (nN, nU), (nE, nN)]
    x_labs = ["East (m)", "North (m)", "East (m)"]
    y_labs = ["Up (m)",   "Up (m)",    "North (m)"]

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

BINARY_FILE = "/Users/christianpedersen/Documents/Hobbies/Model Rockets/TestFlights/2026_03_08/Raw Downloads/Rolly Poly IV/flight_20260308_173449.bin"
OUTPUT_DIR  = None     # None = same directory as the binary file
SHOW_PLOTS  = True     # True = show interactive matplotlib windows
TIME_WINDOW = (462, 468) # (t_start, t_end) in seconds, or None for all data

# ---------- Plot chooser (True = generate, False = skip) ----------
PLOTS = {
    # Sensor data
    "acc_sats":          False,
    "acc_icm":           False,
    "acc_high_g":        False,
    "acc_overlay":       False,
    "gyro":              False,
    "temperature":       False,
    "pressure":          False,
    "magnetometer":      False,
    # Power
    "voltage":           False,
    "current":           False,
    "soc":               False,
    # GPS
    "gps_altitude":      True,
    "gps_alt_annotated": True,
    "gps_velocity":      False,
    "ground_track":      True,
    "gps_enu_3d":        False,
    "gps_enu_2d":        False,
    # Nav filter
    "attitude":          True,
    "roll_cmd_gyro":     True,
    "nav_altitude":      False,
    "nav_velocity":      True,
    "rocket_state":      False,
    "nav_pos_3d":        False,
    "nav_pos_2d":        False,
    # Comparisons
    "press_vs_gps_alt":  False,
    "nav_vs_gps_pos":    False,
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

    records, stats = parse_binary_file(filepath)

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

    # Determine global t0 (earliest timestamp across all data)
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

    # Output directory for saved plots
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
        ("acc_icm",          lambda: plot_acc_icm(records, t0_global)),
        ("acc_high_g",       lambda: plot_acc_high_g(records, t0_global)),
        ("acc_overlay",      lambda: plot_acc_overlay(records, t0_global)),
        ("gyro",             lambda: plot_gyro_xyz(records, t0_global)),
        ("temperature",      lambda: plot_temperature(records, t0_global)),
        ("pressure",         lambda: plot_pressure(records, t0_global)),
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
        ("press_vs_gps_alt", lambda: plot_pressure_vs_gps_alt(records, t0_global)),
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
