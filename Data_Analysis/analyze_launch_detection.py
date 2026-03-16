#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_launch_detection.py

Parses a binary flight recording and analyzes why launch was not detected.
Focuses on:
  - ISM6HG256 acceleration data (low-g and high-g)
  - BMP585 pressure/altitude profile
  - NonSensor flight state transitions and kinematics
  - Identifying the actual launch signature vs. the flight computer's state
"""

import struct
import math
import sys

# ── Constants from the existing parser ──────────────────────────────────────

G_MS2 = 9.80665

ISM6_LOW_G_FS_G  = 16
ISM6_HIGH_G_FS_G = 256
ISM6_GYRO_FS_DPS = 4000
ISM6_ROT_Z_DEG   = -45.0

SYNC = b'\xAA\x55\xAA\x55'

# Message type IDs
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

MSG_EXPECTED_LEN = {
    MSG_OUT_STATUS_QUERY: 16,
    MSG_GNSS:             42,
    MSG_ISM6HG256:        22,
    MSG_BMP585:           12,
    MSG_MMC5983MA:        16,
    MSG_NON_SENSOR:       40,
    MSG_POWER:            10,
    MSG_START_LOGGING:    None,
    MSG_END_FLIGHT:       None,
    MSG_LORA:             49,
}

FMT_ISM6       = '<I hhh hhh hhh'
FMT_BMP585     = '<I i I'
FMT_NONSENSOR  = '<I hhhh iii iii BB h'
FMT_STATUS_QUERY = '<B H H hh B hhh'

ROCKET_STATES = {
    0: "INIT",
    1: "READY",
    2: "PRELAUNCH",
    3: "INFLIGHT",
    4: "LANDED",
}

NSF_ALT_LANDED = (1 << 0)
NSF_ALT_APOGEE = (1 << 1)
NSF_VEL_APOGEE = (1 << 2)
NSF_LAUNCH     = (1 << 3)


# ── CRC-16 ──────────────────────────────────────────────────────────────────

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
    crc = init
    for byte in data:
        crc = _CRC16_TABLE[((crc >> 8) ^ byte) & 0xFF] ^ ((crc << 8) & 0xFFFF)
    return crc


# ── Conversion helpers ──────────────────────────────────────────────────────

def ism6_scales(low_g_fs, high_g_fs, gyro_fs):
    denom = 32768.0
    acc_low  = (low_g_fs  * 1000.0 / denom) * 1e-3 * G_MS2
    acc_high = (high_g_fs * 1000.0 / denom) * 1e-3 * G_MS2
    gyro     = (gyro_fs   * 1000.0 / denom) * 1e-3
    return acc_low, acc_high, gyro

def pressure_to_altitude(p_pa, p0=101325.0):
    if p_pa <= 0 or p0 <= 0:
        return 0.0
    return 44330.0 * (1.0 - (p_pa / p0) ** (1.0 / 5.255))


# ── Parser ──────────────────────────────────────────────────────────────────

def parse_file(filepath):
    with open(filepath, 'rb') as f:
        data = f.read()

    file_size = len(data)

    config = {
        "low_g_fs_g":    ISM6_LOW_G_FS_G,
        "high_g_fs_g":   ISM6_HIGH_G_FS_G,
        "gyro_fs_dps":   ISM6_GYRO_FS_DPS,
        "ism6_rot_z_deg": ISM6_ROT_Z_DEG,
        "hg_bias": (0.0, 0.0, 0.0),
    }

    ism6_raw = []
    bmp585 = []
    nonsensor = []
    stats = {"total": 0, "good_crc": 0, "bad_crc": 0, "type_counts": {}}

    pos = 0
    while pos < file_size - 8:
        idx = data.find(SYNC, pos)
        if idx == -1:
            break
        pos = idx + 4

        if pos + 2 > file_size:
            break

        msg_type = data[pos]
        msg_len  = data[pos + 1]
        pos += 2

        if msg_type in MSG_EXPECTED_LEN:
            expected = MSG_EXPECTED_LEN[msg_type]
            if expected is not None and msg_len != expected:
                pos -= 1
                continue

        if pos + msg_len + 2 > file_size:
            break

        payload = data[pos:pos + msg_len]
        crc_hi  = data[pos + msg_len]
        crc_lo  = data[pos + msg_len + 1]
        crc_received = (crc_hi << 8) | crc_lo
        crc_computed = crc16(bytes([msg_type, msg_len]) + payload)

        stats["total"] += 1
        if crc_received != crc_computed:
            stats["bad_crc"] += 1
            continue

        stats["good_crc"] += 1
        tname = MSG_NAMES.get(msg_type, f"0x{msg_type:02X}")
        stats["type_counts"][tname] = stats["type_counts"].get(tname, 0) + 1
        pos += msg_len + 2

        try:
            if msg_type == MSG_OUT_STATUS_QUERY and msg_len >= 16:
                fields = struct.unpack(FMT_STATUS_QUERY, payload[:16])
                config["low_g_fs_g"]    = fields[0]
                config["high_g_fs_g"]   = fields[1]
                config["gyro_fs_dps"]   = fields[2]
                config["ism6_rot_z_deg"] = fields[3] / 100.0
                fmt_ver = fields[5]
                if fmt_ver >= 2:
                    config["hg_bias"] = (
                        fields[6] / 100.0,
                        fields[7] / 100.0,
                        fields[8] / 100.0,
                    )

            elif msg_type == MSG_ISM6HG256:
                fields = struct.unpack(FMT_ISM6, payload)
                ism6_raw.append({
                    "time_us": fields[0],
                    "lg_x": fields[1], "lg_y": fields[2], "lg_z": fields[3],
                    "hg_x": fields[4], "hg_y": fields[5], "hg_z": fields[6],
                    "gy_x": fields[7], "gy_y": fields[8], "gy_z": fields[9],
                })

            elif msg_type == MSG_BMP585:
                fields = struct.unpack(FMT_BMP585, payload)
                press_pa = float(fields[2]) / 64.0
                bmp585.append({
                    "time_us":     fields[0],
                    "temperature": float(fields[1]) / 65536.0,
                    "pressure_pa": press_pa,
                })

            elif msg_type == MSG_NON_SENSOR:
                fields = struct.unpack(FMT_NONSENSOR, payload)
                flags = fields[11]
                nonsensor.append({
                    "time_us":       fields[0],
                    "roll":          fields[1] / 100.0,
                    "pitch":         fields[2] / 100.0,
                    "yaw":           fields[3] / 100.0,
                    "roll_cmd":      fields[4] / 100.0,
                    "e_pos":         fields[5] / 100.0,
                    "n_pos":         fields[6] / 100.0,
                    "u_pos":         fields[7] / 100.0,
                    "e_vel":         fields[8] / 100.0,
                    "n_vel":         fields[9] / 100.0,
                    "u_vel":         fields[10] / 100.0,
                    "flags":         flags,
                    "rocket_state":  fields[12],
                    "baro_alt_rate": fields[13] * 0.1,
                    "alt_landed":    bool(flags & NSF_ALT_LANDED),
                    "alt_apogee":    bool(flags & NSF_ALT_APOGEE),
                    "vel_apogee":    bool(flags & NSF_VEL_APOGEE),
                    "launch":        bool(flags & NSF_LAUNCH),
                })

        except struct.error:
            pass

    # Post-process ISM6 with config
    acc_low_scale, acc_high_scale, gyro_scale = ism6_scales(
        config["low_g_fs_g"], config["high_g_fs_g"], config["gyro_fs_dps"])

    rot_rad = math.radians(config["ism6_rot_z_deg"])
    c_rot, s_rot = math.cos(rot_rad), math.sin(rot_rad)
    hg_bx, hg_by, hg_bz = config["hg_bias"]

    ism6 = []
    for r in ism6_raw:
        lg_x = r["lg_x"] * acc_low_scale
        lg_y = r["lg_y"] * acc_low_scale
        lg_z = r["lg_z"] * acc_low_scale
        hg_x = r["hg_x"] * acc_high_scale
        hg_y = r["hg_y"] * acc_high_scale
        hg_z = r["hg_z"] * acc_high_scale
        gy_x = r["gy_x"] * gyro_scale
        gy_y = r["gy_y"] * gyro_scale
        gy_z = r["gy_z"] * gyro_scale

        ism6.append({
            "time_us":    r["time_us"],
            "low_acc_x":  lg_x * c_rot - lg_y * s_rot,
            "low_acc_y":  lg_x * s_rot + lg_y * c_rot,
            "low_acc_z":  lg_z,
            "high_acc_x": (hg_x * c_rot - hg_y * s_rot) - hg_bx,
            "high_acc_y": (hg_x * s_rot + hg_y * c_rot) - hg_by,
            "high_acc_z": hg_z - hg_bz,
            "gyro_x":     gy_x * c_rot - gy_y * s_rot,
            "gyro_y":     gy_x * s_rot + gy_y * c_rot,
            "gyro_z":     gy_z,
        })

    return ism6, bmp585, nonsensor, stats, config


# ── Analysis ────────────────────────────────────────────────────────────────

def accel_magnitude(rec, prefix="low_acc"):
    x = rec[f"{prefix}_x"]
    y = rec[f"{prefix}_y"]
    z = rec[f"{prefix}_z"]
    return math.sqrt(x*x + y*y + z*z)


def print_section(title):
    width = 80
    print()
    print("=" * width)
    print(f"  {title}")
    print("=" * width)


def main():
    filepath = (
        "/Users/christianpedersen/Documents/Hobbies/Model Rockets/"
        "TestFlights/2026_03_08/Raw Downloads/Goblin Flight 2 F52/"
        "flight_20260308_190239.bin"
    )

    print(f"Parsing: {filepath}")
    ism6, bmp585, nonsensor, stats, config = parse_file(filepath)

    # ── Parsing summary ─────────────────────────────────────────────────
    print_section("PARSING SUMMARY")
    print(f"  Total frames found:  {stats['total']}")
    print(f"  Good CRC:            {stats['good_crc']}")
    print(f"  Bad CRC:             {stats['bad_crc']}")
    print(f"  Frame type counts:")
    for name, count in sorted(stats["type_counts"].items()):
        print(f"    {name:25s} {count:>8d}")

    print(f"\n  Sensor config from OUT_STATUS_QUERY:")
    print(f"    Low-g FS:   +/-{config['low_g_fs_g']} g")
    print(f"    High-g FS:  +/-{config['high_g_fs_g']} g")
    print(f"    Gyro FS:    +/-{config['gyro_fs_dps']} dps")
    print(f"    ISM6 rot Z: {config['ism6_rot_z_deg']} deg")
    print(f"    HG bias:    {config['hg_bias']}")

    # Determine global t0 (earliest timestamp across all frame types)
    all_times = []
    if ism6:
        all_times.append(ism6[0]["time_us"])
    if bmp585:
        all_times.append(bmp585[0]["time_us"])
    if nonsensor:
        all_times.append(nonsensor[0]["time_us"])
    t0 = min(all_times) if all_times else 0

    def t_sec(time_us):
        return (time_us - t0) / 1e6

    # ── ISM6 acceleration summary ───────────────────────────────────────
    print_section("ISM6HG256 ACCELERATION SUMMARY")
    if ism6:
        print(f"  Total ISM6 samples: {len(ism6)}")
        t_first = t_sec(ism6[0]["time_us"])
        t_last  = t_sec(ism6[-1]["time_us"])
        dur = t_last - t_first
        rate = len(ism6) / dur if dur > 0 else 0
        print(f"  Time span:  {t_first:.3f} s  to  {t_last:.3f} s  ({dur:.3f} s)")
        print(f"  Avg rate:   {rate:.1f} Hz")

        low_mags  = [accel_magnitude(r, "low_acc") for r in ism6]
        high_mags = [accel_magnitude(r, "high_acc") for r in ism6]

        print(f"\n  Low-g accelerometer |a| (m/s^2):")
        print(f"    Min:   {min(low_mags):10.3f}")
        print(f"    Max:   {max(low_mags):10.3f}")
        print(f"    Mean:  {sum(low_mags)/len(low_mags):10.3f}")
        print(f"    1g =   {G_MS2:.3f}")

        print(f"\n  High-g accelerometer |a| (m/s^2):")
        print(f"    Min:   {min(high_mags):10.3f}")
        print(f"    Max:   {max(high_mags):10.3f}")
        print(f"    Mean:  {sum(high_mags)/len(high_mags):10.3f}")

        # Find the peak high-g acceleration
        peak_idx = high_mags.index(max(high_mags))
        peak_rec = ism6[peak_idx]
        print(f"\n  Peak high-g acceleration:")
        print(f"    Time:    {t_sec(peak_rec['time_us']):.4f} s")
        print(f"    |a|:     {high_mags[peak_idx]:.3f} m/s^2  ({high_mags[peak_idx]/G_MS2:.2f} g)")
        print(f"    X/Y/Z:   {peak_rec['high_acc_x']:.2f} / {peak_rec['high_acc_y']:.2f} / {peak_rec['high_acc_z']:.2f} m/s^2")

        # Find launch signature: sustained acceleration above a threshold
        # Looking for when |low_acc| first exceeds 2g (indicating motor ignition)
        launch_threshold_ms2 = 2.0 * G_MS2
        launch_candidates = []
        for i, mag in enumerate(low_mags):
            if mag > launch_threshold_ms2:
                launch_candidates.append(i)

        if launch_candidates:
            # Find the first sustained burst (at least 5 consecutive samples above threshold)
            burst_start = None
            run_start = launch_candidates[0]
            run_len = 1
            for j in range(1, len(launch_candidates)):
                if launch_candidates[j] == launch_candidates[j-1] + 1:
                    run_len += 1
                else:
                    if run_len >= 5 and burst_start is None:
                        burst_start = run_start
                    run_start = launch_candidates[j]
                    run_len = 1
            if run_len >= 5 and burst_start is None:
                burst_start = run_start

            if burst_start is not None:
                launch_time_us = ism6[burst_start]["time_us"]
                launch_t = t_sec(launch_time_us)
                print(f"\n  ** Detected actual launch signature (low-g > 2g sustained):")
                print(f"     Time:  {launch_t:.4f} s  (sample index {burst_start})")
                print(f"     |a|:   {low_mags[burst_start]:.3f} m/s^2  ({low_mags[burst_start]/G_MS2:.2f} g)")
            else:
                launch_time_us = None
                launch_t = None
                print(f"\n  No sustained burst above 2g found (only isolated spikes).")
                # Try with lower threshold
                launch_threshold_ms2 = 1.5 * G_MS2
                launch_candidates_15 = [i for i, mag in enumerate(low_mags) if mag > launch_threshold_ms2]
                if launch_candidates_15:
                    burst_start_15 = None
                    run_start = launch_candidates_15[0]
                    run_len = 1
                    for j in range(1, len(launch_candidates_15)):
                        if launch_candidates_15[j] == launch_candidates_15[j-1] + 1:
                            run_len += 1
                        else:
                            if run_len >= 5 and burst_start_15 is None:
                                burst_start_15 = run_start
                            run_start = launch_candidates_15[j]
                            run_len = 1
                    if run_len >= 5 and burst_start_15 is None:
                        burst_start_15 = run_start
                    if burst_start_15 is not None:
                        launch_time_us = ism6[burst_start_15]["time_us"]
                        launch_t = t_sec(launch_time_us)
                        print(f"  ** Found launch signature at 1.5g threshold:")
                        print(f"     Time:  {launch_t:.4f} s")
                        print(f"     |a|:   {low_mags[burst_start_15]:.3f} m/s^2")
                    else:
                        launch_time_us = None
                        launch_t = None
        else:
            launch_time_us = None
            launch_t = None
            print(f"\n  No samples above 2g found -- checking high-g accel...")
            hg_launch_thresh = 2.0 * G_MS2
            hg_candidates = [i for i, mag in enumerate(high_mags) if mag > hg_launch_thresh]
            if hg_candidates:
                burst_start = None
                run_start = hg_candidates[0]
                run_len = 1
                for j in range(1, len(hg_candidates)):
                    if hg_candidates[j] == hg_candidates[j-1] + 1:
                        run_len += 1
                    else:
                        if run_len >= 5 and burst_start is None:
                            burst_start = run_start
                        run_start = hg_candidates[j]
                        run_len = 1
                if run_len >= 5 and burst_start is None:
                    burst_start = run_start
                if burst_start is not None:
                    launch_time_us = ism6[burst_start]["time_us"]
                    launch_t = t_sec(launch_time_us)
                    print(f"  ** Found launch signature in high-g data:")
                    print(f"     Time:  {launch_t:.4f} s")
                    print(f"     |a|:   {high_mags[burst_start]:.3f} m/s^2  ({high_mags[burst_start]/G_MS2:.2f} g)")

    else:
        launch_time_us = None
        launch_t = None
        print("  No ISM6 data found!")

    # ── BMP585 altitude profile ─────────────────────────────────────────
    print_section("BMP585 PRESSURE / ALTITUDE PROFILE")
    if bmp585:
        print(f"  Total BMP585 samples: {len(bmp585)}")
        t_first = t_sec(bmp585[0]["time_us"])
        t_last  = t_sec(bmp585[-1]["time_us"])
        dur = t_last - t_first
        rate = len(bmp585) / dur if dur > 0 else 0
        print(f"  Time span:  {t_first:.3f} s  to  {t_last:.3f} s  ({dur:.3f} s)")
        print(f"  Avg rate:   {rate:.1f} Hz")

        pressures = [r["pressure_pa"] for r in bmp585]
        temps     = [r["temperature"] for r in bmp585]

        # Use first N samples as ground reference
        n_ground = min(50, len(pressures))
        p0 = sum(pressures[:n_ground]) / n_ground
        altitudes = [pressure_to_altitude(p, p0) for p in pressures]

        print(f"\n  Ground reference pressure (first {n_ground} samples): {p0:.2f} Pa")
        print(f"  Temperature range: {min(temps):.2f} C  to  {max(temps):.2f} C")
        print(f"\n  Altitude (barometric, AGL):")
        print(f"    Min:   {min(altitudes):8.2f} m")
        print(f"    Max:   {max(altitudes):8.2f} m  ({max(altitudes)*3.281:.1f} ft)")
        print(f"    Mean:  {sum(altitudes)/len(altitudes):8.2f} m")

        # Find apogee
        apogee_idx = altitudes.index(max(altitudes))
        apogee_t   = t_sec(bmp585[apogee_idx]["time_us"])
        print(f"\n  Barometric apogee:")
        print(f"    Time:     {apogee_t:.3f} s")
        print(f"    Altitude: {altitudes[apogee_idx]:.2f} m  ({altitudes[apogee_idx]*3.281:.1f} ft)")
        print(f"    Pressure: {pressures[apogee_idx]:.2f} Pa")

        # Print altitude profile in time bins
        print(f"\n  Altitude profile (1-second bins):")
        print(f"    {'Time (s)':>10s}  {'Alt (m)':>10s}  {'Pressure (Pa)':>14s}  {'Temp (C)':>10s}")
        bin_size = 1.0
        t_curr = 0.0
        bi = 0
        while t_curr <= t_last and bi < len(bmp585):
            # Find samples in this bin
            bin_alts = []
            bin_press = []
            bin_temps = []
            while bi < len(bmp585) and t_sec(bmp585[bi]["time_us"]) < t_curr + bin_size:
                bin_alts.append(altitudes[bi])
                bin_press.append(pressures[bi])
                bin_temps.append(temps[bi])
                bi += 1
            if bin_alts:
                avg_alt = sum(bin_alts) / len(bin_alts)
                avg_p   = sum(bin_press) / len(bin_press)
                avg_t   = sum(bin_temps) / len(bin_temps)
                print(f"    {t_curr:10.1f}  {avg_alt:10.2f}  {avg_p:14.2f}  {avg_t:10.2f}")
            t_curr += bin_size
    else:
        altitudes = []
        print("  No BMP585 data found!")

    # ── NonSensor flight state analysis ─────────────────────────────────
    print_section("NONSENSOR FLIGHT STATE ANALYSIS")
    if nonsensor:
        print(f"  Total NonSensor samples: {len(nonsensor)}")
        t_first = t_sec(nonsensor[0]["time_us"])
        t_last  = t_sec(nonsensor[-1]["time_us"])
        dur = t_last - t_first
        rate = len(nonsensor) / dur if dur > 0 else 0
        print(f"  Time span:  {t_first:.3f} s  to  {t_last:.3f} s  ({dur:.3f} s)")
        print(f"  Avg rate:   {rate:.1f} Hz")

        # State transitions
        print(f"\n  Flight state transitions:")
        prev_state = None
        for rec in nonsensor:
            state = rec["rocket_state"]
            if state != prev_state:
                state_name = ROCKET_STATES.get(state, f"UNKNOWN({state})")
                prev_name  = ROCKET_STATES.get(prev_state, "---") if prev_state is not None else "---"
                t = t_sec(rec["time_us"])
                print(f"    {t:10.4f} s :  {prev_name} --> {state_name}")
                prev_state = state

        # Check for launch flag
        launch_flag_times = [r for r in nonsensor if r["launch"]]
        if launch_flag_times:
            print(f"\n  Launch flag (NSF_LAUNCH) set in {len(launch_flag_times)} samples:")
            for r in launch_flag_times[:5]:
                print(f"    t={t_sec(r['time_us']):.4f} s  state={ROCKET_STATES.get(r['rocket_state'], '?')}")
            if len(launch_flag_times) > 5:
                print(f"    ... and {len(launch_flag_times) - 5} more")
        else:
            print(f"\n  *** Launch flag (NSF_LAUNCH) was NEVER set! ***")

        # Kinematics around detected launch time
        if launch_time_us is not None:
            print(f"\n  NonSensor kinematics around detected launch ({launch_t:.3f} s):")
            window = 2.0  # seconds
            nearby = [r for r in nonsensor
                      if abs(t_sec(r["time_us"]) - launch_t) <= window]
            print(f"    {'Time(s)':>10s}  {'State':>10s}  {'U_pos(m)':>10s}  {'U_vel(m/s)':>10s}  {'BaroRate':>10s}  {'Flags':>6s}  {'Launch':>6s}")
            for r in nearby:
                st = ROCKET_STATES.get(r["rocket_state"], "?")
                t = t_sec(r["time_us"])
                flag_str = f"0x{r['flags']:02X}"
                print(f"    {t:10.4f}  {st:>10s}  {r['u_pos']:10.3f}  {r['u_vel']:10.3f}  {r['baro_alt_rate']:10.3f}  {flag_str:>6s}  {'YES' if r['launch'] else 'no':>6s}")

        # Print unique states and how long in each
        state_durations = {}
        prev_state = nonsensor[0]["rocket_state"]
        prev_time  = nonsensor[0]["time_us"]
        for rec in nonsensor[1:]:
            if rec["rocket_state"] != prev_state:
                dur_s = (rec["time_us"] - prev_time) / 1e6
                sname = ROCKET_STATES.get(prev_state, f"?({prev_state})")
                state_durations[sname] = state_durations.get(sname, 0.0) + dur_s
                prev_time = rec["time_us"]
                prev_state = rec["rocket_state"]
        # Final state
        dur_s = (nonsensor[-1]["time_us"] - prev_time) / 1e6
        sname = ROCKET_STATES.get(prev_state, f"?({prev_state})")
        state_durations[sname] = state_durations.get(sname, 0.0) + dur_s

        print(f"\n  Time spent in each state:")
        for sname, dur in state_durations.items():
            print(f"    {sname:15s}  {dur:8.2f} s")

    else:
        print("  No NonSensor data found!")

    # ── Timeline: acceleration + state around launch ────────────────────
    print_section("DETAILED TIMELINE AROUND LAUNCH")

    if launch_time_us is not None and ism6:
        window_sec = 3.0
        launch_window_start = launch_time_us - int(window_sec * 1e6)
        launch_window_end   = launch_time_us + int(window_sec * 1e6)

        # ISM6 samples in window
        ism6_window = [r for r in ism6
                       if launch_window_start <= r["time_us"] <= launch_window_end]
        # NonSensor in window
        ns_window = [r for r in nonsensor
                     if launch_window_start <= r["time_us"] <= launch_window_end]

        # Merge into timeline
        events = []
        for r in ism6_window:
            low_mag  = accel_magnitude(r, "low_acc")
            high_mag = accel_magnitude(r, "high_acc")
            events.append((r["time_us"], "ISM6",
                           f"|low|={low_mag:7.2f} m/s^2 ({low_mag/G_MS2:5.2f}g)  "
                           f"|high|={high_mag:7.2f} m/s^2 ({high_mag/G_MS2:5.2f}g)  "
                           f"Z_low={r['low_acc_z']:7.2f}  Z_high={r['high_acc_z']:7.2f}"))
        for r in ns_window:
            st = ROCKET_STATES.get(r["rocket_state"], "?")
            events.append((r["time_us"], "STATE",
                           f"state={st}  u_pos={r['u_pos']:.2f}m  u_vel={r['u_vel']:.2f}m/s  "
                           f"launch_flag={'YES' if r['launch'] else 'no'}  flags=0x{r['flags']:02X}"))

        events.sort(key=lambda x: x[0])

        # Print with decimation (every Nth ISM6 sample to keep output manageable)
        ism6_count = sum(1 for e in events if e[1] == "ISM6")
        decimate = max(1, ism6_count // 60)

        print(f"  Window: {t_sec(launch_window_start):.3f} s  to  {t_sec(launch_window_end):.3f} s")
        print(f"  ISM6 samples in window: {ism6_count}  (showing every {decimate}th)")
        print(f"  NonSensor samples in window: {len(ns_window)}")
        print()

        ism6_printed = 0
        for time_us, etype, desc in events:
            t = t_sec(time_us)
            if etype == "ISM6":
                ism6_printed += 1
                if ism6_printed % decimate != 0:
                    continue
            marker = ">>>" if (launch_time_us is not None and
                               abs(time_us - launch_time_us) < 5000) else "   "
            print(f"  {marker} {t:10.4f}s  [{etype:5s}]  {desc}")

    elif ism6:
        # No clear launch detected, show first 5 seconds of data
        print("  No clear launch signature detected. Showing first 5 seconds of acceleration data:")
        t0_ism6 = ism6[0]["time_us"]
        first_5s = [r for r in ism6 if r["time_us"] - t0_ism6 <= 5e6]
        decimate = max(1, len(first_5s) // 40)
        count = 0
        for r in first_5s:
            count += 1
            if count % decimate != 0:
                continue
            t = t_sec(r["time_us"])
            low_mag = accel_magnitude(r, "low_acc")
            high_mag = accel_magnitude(r, "high_acc")
            print(f"    {t:8.4f}s  |low|={low_mag:7.2f} ({low_mag/G_MS2:5.2f}g)  "
                  f"|high|={high_mag:7.2f} ({high_mag/G_MS2:5.2f}g)")

    # ── Diagnosis ───────────────────────────────────────────────────────
    print_section("DIAGNOSIS: WHY LAUNCH WAS NOT DETECTED")

    if nonsensor:
        # What state was the computer in during the whole flight?
        states_seen = set(r["rocket_state"] for r in nonsensor)
        state_names = [ROCKET_STATES.get(s, f"?({s})") for s in sorted(states_seen)]
        print(f"  States seen throughout recording: {', '.join(state_names)}")

        launch_detected_by_fc = any(r["launch"] for r in nonsensor)
        entered_inflight = any(r["rocket_state"] == 3 for r in nonsensor)

        if not launch_detected_by_fc:
            print(f"  The flight computer NEVER set the launch flag (NSF_LAUNCH).")
        else:
            print(f"  The flight computer DID set the launch flag at some point.")

        if not entered_inflight:
            print(f"  The flight computer NEVER entered INFLIGHT state.")
        else:
            inflight_start = next(r for r in nonsensor if r["rocket_state"] == 3)
            print(f"  The flight computer entered INFLIGHT at t={t_sec(inflight_start['time_us']):.4f} s")

        if launch_time_us is not None:
            # What was the flight computer doing at actual launch time?
            ns_at_launch = None
            for r in nonsensor:
                if r["time_us"] <= launch_time_us:
                    ns_at_launch = r
                else:
                    break
            if ns_at_launch:
                st = ROCKET_STATES.get(ns_at_launch["rocket_state"], "?")
                print(f"\n  At the time of the actual launch signature ({launch_t:.4f} s):")
                print(f"    Flight computer state: {st}")
                print(f"    Launch flag:           {'SET' if ns_at_launch['launch'] else 'NOT SET'}")
                print(f"    U position (m):        {ns_at_launch['u_pos']:.3f}")
                print(f"    U velocity (m/s):      {ns_at_launch['u_vel']:.3f}")
                print(f"    Baro alt rate (m/s):   {ns_at_launch['baro_alt_rate']:.3f}")
                print(f"    Flags byte:            0x{ns_at_launch['flags']:02X}")

        # Check if the computer was stuck in a specific state
        if len(states_seen) == 1:
            only_state = ROCKET_STATES.get(list(states_seen)[0], "?")
            print(f"\n  ** The flight computer was STUCK in {only_state} state for the entire recording! **")
            if only_state == "READY":
                print(f"     The computer never transitioned from READY to PRELAUNCH or INFLIGHT.")
                print(f"     Possible causes:")
                print(f"       - Acceleration threshold for launch detection too high")
                print(f"       - Wrong axis being checked for launch")
                print(f"       - Launch detection algorithm not running")
                print(f"       - Sensor data not reaching the state machine")
            elif only_state == "PRELAUNCH":
                print(f"     The computer was in PRELAUNCH but never detected actual launch.")
                print(f"     Possible causes:")
                print(f"       - Launch detection threshold too high")
                print(f"       - Launch detection looking at wrong axis")
                print(f"       - Timer/debounce too long for the acceleration profile")
            elif only_state == "INIT":
                print(f"     The computer never left INIT state.")
                print(f"     Possible causes:")
                print(f"       - Sensor initialization failed")
                print(f"       - Configuration error")

    print()


if __name__ == "__main__":
    main()
