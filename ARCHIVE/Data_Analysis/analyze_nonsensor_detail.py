#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_nonsensor_detail.py

Deep-dive into NonSensor frames from a TinkerRocket Mini flight recording.
Prints raw field dumps, checks for non-zero values, verifies parsing around
the launch window, and summarizes all field ranges.
"""

import struct
import math
import sys

# ── Constants ────────────────────────────────────────────────────────────────

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

# Named field indices for the struct unpack result
NS_FIELD_NAMES = [
    "time_us",       # 0  I   uint32   microseconds
    "roll_raw",      # 1  h   int16    centideg
    "pitch_raw",     # 2  h   int16    centideg
    "yaw_raw",       # 3  h   int16    centideg
    "roll_cmd_raw",  # 4  h   int16    centideg
    "e_pos_raw",     # 5  i   int32    cm
    "n_pos_raw",     # 6  i   int32    cm
    "u_pos_raw",     # 7  i   int32    cm
    "e_vel_raw",     # 8  i   int32    cm/s
    "n_vel_raw",     # 9  i   int32    cm/s
    "u_vel_raw",     # 10 i   int32    cm/s
    "flags",         # 11 B   uint8
    "rocket_state",  # 12 B   uint8
    "baro_alt_rate_raw",  # 13 h int16  dm/s
]

NS_FIELD_FORMATS = [
    "I", "h", "h", "h", "h",
    "i", "i", "i",
    "i", "i", "i",
    "B", "B", "h",
]

NS_FIELD_UNITS = [
    "us", "centideg", "centideg", "centideg", "centideg",
    "cm", "cm", "cm",
    "cm/s", "cm/s", "cm/s",
    "bitfield", "enum", "dm/s",
]


# ── CRC-16 ───────────────────────────────────────────────────────────────────

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


# ── Conversion helpers ────────────────────────────────────────────────────────

def ism6_scales(low_g_fs, high_g_fs, gyro_fs):
    denom = 32768.0
    acc_low  = (low_g_fs  * 1000.0 / denom) * 1e-3 * G_MS2
    acc_high = (high_g_fs * 1000.0 / denom) * 1e-3 * G_MS2
    gyro     = (gyro_fs   * 1000.0 / denom) * 1e-3
    return acc_low, acc_high, gyro


# ── Parser ────────────────────────────────────────────────────────────────────

def parse_file(filepath):
    """Parse the binary file and return raw NonSensor + ISM6 data, plus raw bytes."""
    with open(filepath, 'rb') as f:
        data = f.read()

    file_size = len(data)
    print(f"File size: {file_size:,} bytes ({file_size / 1024 / 1024:.2f} MB)")

    config = {
        "low_g_fs_g":    ISM6_LOW_G_FS_G,
        "high_g_fs_g":   ISM6_HIGH_G_FS_G,
        "gyro_fs_dps":   ISM6_GYRO_FS_DPS,
        "ism6_rot_z_deg": ISM6_ROT_Z_DEG,
        "hg_bias": (0.0, 0.0, 0.0),
    }

    nonsensor_frames = []    # list of (fields_tuple, raw_payload_bytes)
    ism6_raw = []
    bmp585_raw = []
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
                ism6_raw.append(fields)

            elif msg_type == MSG_BMP585:
                fields = struct.unpack(FMT_BMP585, payload)
                bmp585_raw.append(fields)

            elif msg_type == MSG_NON_SENSOR:
                fields = struct.unpack(FMT_NONSENSOR, payload)
                nonsensor_frames.append((fields, bytes(payload)))

        except struct.error:
            pass

    return nonsensor_frames, ism6_raw, bmp585_raw, stats, config, data


# ── Printing helpers ──────────────────────────────────────────────────────────

def section(title):
    w = 90
    print()
    print("=" * w)
    print(f"  {title}")
    print("=" * w)


def print_ns_frame(idx, fields, raw_bytes, t0_us):
    """Print a single NonSensor frame with all raw and converted fields."""
    t_s = (fields[0] - t0_us) / 1e6 if t0_us else fields[0] / 1e6
    state_name = ROCKET_STATES.get(fields[12], f"UNKNOWN({fields[12]})")
    flags_val = fields[11]
    flag_bits = []
    if flags_val & NSF_ALT_LANDED: flag_bits.append("ALT_LANDED")
    if flags_val & NSF_ALT_APOGEE: flag_bits.append("ALT_APOGEE")
    if flags_val & NSF_VEL_APOGEE: flag_bits.append("VEL_APOGEE")
    if flags_val & NSF_LAUNCH:     flag_bits.append("LAUNCH")
    flags_str = "|".join(flag_bits) if flag_bits else "(none)"

    print(f"  --- NonSensor frame #{idx}  t={t_s:.4f}s  (time_us={fields[0]}) ---")
    print(f"    Raw bytes ({len(raw_bytes)}): {raw_bytes.hex(' ')}")
    print(f"    Struct unpack: {fields}")
    print(f"    ---- Field breakdown ----")
    print(f"    {'Field':>22s}  {'Raw':>12s}  {'Converted':>14s}  {'Unit':>10s}")
    print(f"    {'time_us':>22s}  {fields[0]:>12d}  {t_s:>14.4f}  {'sec':>10s}")
    print(f"    {'roll':>22s}  {fields[1]:>12d}  {fields[1]/100.0:>14.2f}  {'deg':>10s}")
    print(f"    {'pitch':>22s}  {fields[2]:>12d}  {fields[2]/100.0:>14.2f}  {'deg':>10s}")
    print(f"    {'yaw':>22s}  {fields[3]:>12d}  {fields[3]/100.0:>14.2f}  {'deg':>10s}")
    print(f"    {'roll_cmd':>22s}  {fields[4]:>12d}  {fields[4]/100.0:>14.2f}  {'deg':>10s}")
    print(f"    {'e_pos':>22s}  {fields[5]:>12d}  {fields[5]/100.0:>14.2f}  {'m':>10s}")
    print(f"    {'n_pos':>22s}  {fields[6]:>12d}  {fields[6]/100.0:>14.2f}  {'m':>10s}")
    print(f"    {'u_pos':>22s}  {fields[7]:>12d}  {fields[7]/100.0:>14.2f}  {'m':>10s}")
    print(f"    {'e_vel':>22s}  {fields[8]:>12d}  {fields[8]/100.0:>14.2f}  {'m/s':>10s}")
    print(f"    {'n_vel':>22s}  {fields[9]:>12d}  {fields[9]/100.0:>14.2f}  {'m/s':>10s}")
    print(f"    {'u_vel':>22s}  {fields[10]:>12d}  {fields[10]/100.0:>14.2f}  {'m/s':>10s}")
    print(f"    {'flags':>22s}  {flags_val:>12d}  0b{flags_val:08b}  {flags_str}")
    print(f"    {'rocket_state':>22s}  {fields[12]:>12d}  {state_name:>14s}")
    print(f"    {'baro_alt_rate':>22s}  {fields[13]:>12d}  {fields[13]*0.1:>14.2f}  {'m/s':>10s}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    filepath = (
        "/Users/christianpedersen/Documents/Hobbies/Model Rockets/"
        "TestFlights/2026_03_08/Raw Downloads/Goblin Flight 2 F52/"
        "flight_20260308_190239.bin"
    )

    print(f"Parsing: {filepath}")
    nonsensor_frames, ism6_raw, bmp585_raw, stats, config, raw_data = parse_file(filepath)

    # ── 0. Parsing summary ───────────────────────────────────────────────
    section("PARSING SUMMARY")
    print(f"  Total frames:   {stats['total']}")
    print(f"  Good CRC:       {stats['good_crc']}")
    print(f"  Bad CRC:        {stats['bad_crc']}")
    for name, count in sorted(stats["type_counts"].items()):
        print(f"    {name:25s}  {count:>8d}")
    print(f"\n  NonSensor frames:  {len(nonsensor_frames)}")
    print(f"  ISM6 frames:       {len(ism6_raw)}")
    print(f"  BMP585 frames:     {len(bmp585_raw)}")
    print(f"\n  Config: low_g_fs={config['low_g_fs_g']}g  high_g_fs={config['high_g_fs_g']}g"
          f"  gyro_fs={config['gyro_fs_dps']}dps  rot_z={config['ism6_rot_z_deg']}deg"
          f"  hg_bias={config['hg_bias']}")

    if not nonsensor_frames:
        print("\n  ** NO NonSensor frames found! Nothing to analyze. **")
        return

    # Determine t0 from earliest timestamp across all types
    all_t0_candidates = []
    if nonsensor_frames:
        all_t0_candidates.append(nonsensor_frames[0][0][0])
    if ism6_raw:
        all_t0_candidates.append(ism6_raw[0][0])
    if bmp585_raw:
        all_t0_candidates.append(bmp585_raw[0][0])
    t0_us = min(all_t0_candidates)
    print(f"\n  Global t0 = {t0_us} us")

    def t_sec(time_us):
        return (time_us - t0_us) / 1e6

    # ── 1. Verify struct format ──────────────────────────────────────────
    section("NONSENSOR BINARY FORMAT VERIFICATION")
    print(f"  Message type byte:  0x{MSG_NON_SENSOR:02X} (0xA5)")
    print(f"  Expected payload length: {MSG_EXPECTED_LEN[MSG_NON_SENSOR]} bytes")
    print(f"  Struct format string: '{FMT_NONSENSOR}'")
    ns_struct_size = struct.calcsize(FMT_NONSENSOR)
    print(f"  struct.calcsize(FMT_NONSENSOR) = {ns_struct_size} bytes")
    if ns_struct_size != 40:
        print(f"  *** WARNING: struct size {ns_struct_size} != expected 40! ***")
    else:
        print(f"  OK: struct size matches expected payload length (40 bytes)")

    print(f"\n  Field layout (little-endian):")
    print(f"    {'Offset':>6s}  {'Size':>4s}  {'Fmt':>3s}  {'Field':>22s}  {'Unit'}")
    offset = 0
    sizes = {"I": 4, "H": 2, "h": 2, "i": 4, "B": 1, "b": 1}
    for i, (name, fmt, unit) in enumerate(zip(NS_FIELD_NAMES, NS_FIELD_FORMATS, NS_FIELD_UNITS)):
        sz = sizes[fmt]
        signed = "signed" if fmt in ("h", "i", "b") else "unsigned"
        print(f"    {offset:>6d}  {sz:>4d}  {fmt:>3s}  {name:>22s}  {unit}  ({signed})")
        offset += sz

    # ── 2. Full raw dump of first 10 + every 1000th ──────────────────────
    section("RAW FIELD DUMP: FIRST 10 + EVERY 1000th FRAME")
    total_ns = len(nonsensor_frames)
    dump_indices = list(range(min(10, total_ns)))
    for i in range(1000, total_ns, 1000):
        if i not in dump_indices:
            dump_indices.append(i)

    for idx in dump_indices:
        fields, raw_bytes = nonsensor_frames[idx]
        print_ns_frame(idx, fields, raw_bytes, t0_us)
        print()

    # ── 3. Unique state values ───────────────────────────────────────────
    section("UNIQUE ROCKET STATE VALUES")
    state_values = set()
    state_counts = {}
    for fields, _ in nonsensor_frames:
        s = fields[12]
        state_values.add(s)
        state_counts[s] = state_counts.get(s, 0) + 1

    print(f"  Unique state values: {sorted(state_values)}")
    for s in sorted(state_values):
        name = ROCKET_STATES.get(s, f"UNKNOWN({s})")
        pct = 100.0 * state_counts[s] / total_ns
        print(f"    State {s} ({name:>10s}): {state_counts[s]:>8d} frames  ({pct:.1f}%)")

    # State transitions
    print(f"\n  State transitions:")
    prev_state = nonsensor_frames[0][0][12]
    for i, (fields, _) in enumerate(nonsensor_frames):
        s = fields[12]
        if s != prev_state:
            t = t_sec(fields[0])
            prev_name = ROCKET_STATES.get(prev_state, f"?({prev_state})")
            new_name  = ROCKET_STATES.get(s, f"?({s})")
            print(f"    Frame #{i}  t={t:.4f}s :  {prev_name} --> {new_name}")
            prev_state = s

    # ── 4. Unique flags values ───────────────────────────────────────────
    section("UNIQUE FLAGS VALUES")
    flags_values = set()
    flags_counts = {}
    for fields, _ in nonsensor_frames:
        f = fields[11]
        flags_values.add(f)
        flags_counts[f] = flags_counts.get(f, 0) + 1
    print(f"  Unique flags values: {sorted(flags_values)}")
    for f in sorted(flags_values):
        bits = []
        if f & NSF_ALT_LANDED: bits.append("ALT_LANDED")
        if f & NSF_ALT_APOGEE: bits.append("ALT_APOGEE")
        if f & NSF_VEL_APOGEE: bits.append("VEL_APOGEE")
        if f & NSF_LAUNCH:     bits.append("LAUNCH")
        bits_str = "|".join(bits) if bits else "(none)"
        pct = 100.0 * flags_counts[f] / total_ns
        print(f"    0x{f:02X} (0b{f:08b}): {flags_counts[f]:>8d} frames  ({pct:.1f}%)  [{bits_str}]")

    # ── 5. NonSensor frames around t=115s (launch) ───────────────────────
    section("RAW BYTES AROUND t=115s (EXPECTED LAUNCH)")
    # Find the actual launch time from ISM6 data
    launch_time_us = None
    if ism6_raw:
        acc_low_scale, acc_high_scale, _ = ism6_scales(
            config["low_g_fs_g"], config["high_g_fs_g"], config["gyro_fs_dps"])
        rot_rad = math.radians(config["ism6_rot_z_deg"])
        c_rot, s_rot = math.cos(rot_rad), math.sin(rot_rad)
        hg_bx, hg_by, hg_bz = config["hg_bias"]

        # Find where low-g magnitude > 2g sustained
        threshold = 2.0 * G_MS2
        run_start = None
        run_len = 0
        for i, raw_f in enumerate(ism6_raw):
            lg_x = raw_f[1] * acc_low_scale
            lg_y = raw_f[2] * acc_low_scale
            lg_z = raw_f[3] * acc_low_scale
            # Rotate
            lx = lg_x * c_rot - lg_y * s_rot
            ly = lg_x * s_rot + lg_y * c_rot
            lz = lg_z
            mag = math.sqrt(lx*lx + ly*ly + lz*lz)
            if mag > threshold:
                if run_start is None:
                    run_start = i
                    run_len = 1
                else:
                    run_len += 1
                if run_len >= 5:
                    launch_time_us = ism6_raw[run_start][0]
                    break
            else:
                run_start = None
                run_len = 0

    if launch_time_us is not None:
        launch_t = t_sec(launch_time_us)
        print(f"  Detected ISM6 launch signature at t={launch_t:.4f}s (time_us={launch_time_us})")
    else:
        launch_t = 115.0
        launch_time_us = t0_us + int(launch_t * 1e6)
        print(f"  No ISM6 launch detected; using t~115s as reference")

    # Find NonSensor frames in a +/- 2s window around launch
    window_s = 2.0
    window_us = int(window_s * 1e6)
    ns_around = [(i, f, b) for i, (f, b) in enumerate(nonsensor_frames)
                 if abs(f[0] - launch_time_us) <= window_us]

    print(f"  Showing NonSensor frames within +/-{window_s}s of launch ({len(ns_around)} frames):")
    for idx, fields, raw_bytes in ns_around:
        print_ns_frame(idx, fields, raw_bytes, t0_us)
        print()

    # Also print a few ISM6 frames right at launch for cross-reference
    if ism6_raw and launch_time_us:
        print(f"\n  Cross-reference: ISM6 frames near launch:")
        ism6_near = [(i, f) for i, f in enumerate(ism6_raw)
                     if abs(f[0] - launch_time_us) < 500000]  # +/- 0.5s
        acc_low_scale, acc_high_scale, _ = ism6_scales(
            config["low_g_fs_g"], config["high_g_fs_g"], config["gyro_fs_dps"])
        rot_rad = math.radians(config["ism6_rot_z_deg"])
        c_rot, s_rot = math.cos(rot_rad), math.sin(rot_rad)
        for i, raw_f in ism6_near[:10]:
            lg_x = raw_f[1] * acc_low_scale
            lg_y = raw_f[2] * acc_low_scale
            lg_z = raw_f[3] * acc_low_scale
            lx = lg_x * c_rot - lg_y * s_rot
            ly = lg_x * s_rot + lg_y * c_rot
            lz = lg_z
            mag = math.sqrt(lx*lx + ly*ly + lz*lz)
            t = t_sec(raw_f[0])
            print(f"    ISM6 #{i}  t={t:.4f}s  |low_acc|={mag:.2f} m/s^2 ({mag/G_MS2:.2f}g)")

    # ── 6. Are ALL NonSensor fields truly zero? ──────────────────────────
    section("FIELD-BY-FIELD ZERO CHECK")
    # For each field, count how many frames have non-zero values
    n_fields = len(NS_FIELD_NAMES)
    nonzero_counts = [0] * n_fields
    for fields, _ in nonsensor_frames:
        for j in range(n_fields):
            if fields[j] != 0:
                nonzero_counts[j] += 1

    print(f"  {'Field':>22s}  {'Non-zero':>10s}  {'Total':>10s}  {'%non-zero':>10s}  {'Status'}")
    for j in range(n_fields):
        pct = 100.0 * nonzero_counts[j] / total_ns
        status = "ALL ZERO" if nonzero_counts[j] == 0 else "has data"
        print(f"  {NS_FIELD_NAMES[j]:>22s}  {nonzero_counts[j]:>10d}  {total_ns:>10d}  {pct:>10.1f}%  {status}")

    # Grouped analysis
    print(f"\n  Grouped analysis:")
    orientation_fields = [1, 2, 3, 4]  # roll, pitch, yaw, roll_cmd
    position_fields = [5, 6, 7]         # e_pos, n_pos, u_pos
    velocity_fields = [8, 9, 10]         # e_vel, n_vel, u_vel
    meta_fields = [11, 12, 13]           # flags, state, baro_alt_rate

    groups = [
        ("Orientation (roll/pitch/yaw/roll_cmd)", orientation_fields),
        ("Position (e/n/u_pos)", position_fields),
        ("Velocity (e/n/u_vel)", velocity_fields),
        ("Metadata (flags/state/baro_alt_rate)", meta_fields),
    ]
    for gname, gfields in groups:
        any_nonzero = any(nonzero_counts[j] > 0 for j in gfields)
        all_zero = all(nonzero_counts[j] == 0 for j in gfields)
        if all_zero:
            print(f"    {gname}: ALL ZERO in every frame")
        elif any_nonzero:
            print(f"    {gname}: has non-zero data")
            for j in gfields:
                if nonzero_counts[j] > 0:
                    print(f"      {NS_FIELD_NAMES[j]}: {nonzero_counts[j]} non-zero frames")
        else:
            print(f"    {gname}: mixed")

    # ── 7. Field min/max ranges ──────────────────────────────────────────
    section("FIELD MIN/MAX RANGES (RAW VALUES)")
    mins = [None] * n_fields
    maxs = [None] * n_fields
    for fields, _ in nonsensor_frames:
        for j in range(n_fields):
            v = fields[j]
            if mins[j] is None or v < mins[j]:
                mins[j] = v
            if maxs[j] is None or v > maxs[j]:
                maxs[j] = v

    print(f"  {'Field':>22s}  {'Min (raw)':>14s}  {'Max (raw)':>14s}  {'Min (conv)':>14s}  {'Max (conv)':>14s}  {'Unit (conv)'}")
    # Conversion factors
    conv_factors = [
        (1e-6, "sec"),                # time_us -> seconds (relative to t0 makes more sense but show raw range)
        (1/100.0, "deg"),             # roll
        (1/100.0, "deg"),             # pitch
        (1/100.0, "deg"),             # yaw
        (1/100.0, "deg"),             # roll_cmd
        (1/100.0, "m"),              # e_pos
        (1/100.0, "m"),              # n_pos
        (1/100.0, "m"),              # u_pos
        (1/100.0, "m/s"),            # e_vel
        (1/100.0, "m/s"),            # n_vel
        (1/100.0, "m/s"),            # u_vel
        (1, "bitfield"),              # flags (no conversion)
        (1, "enum"),                  # rocket_state (no conversion)
        (0.1, "m/s"),                 # baro_alt_rate
    ]

    for j in range(n_fields):
        cf, cu = conv_factors[j]
        if j == 0:
            # For time, show relative range
            min_t = t_sec(mins[j])
            max_t = t_sec(maxs[j])
            print(f"  {NS_FIELD_NAMES[j]:>22s}  {mins[j]:>14d}  {maxs[j]:>14d}  {min_t:>14.4f}  {max_t:>14.4f}  sec (rel)")
        elif j in (11, 12):
            # flags and state: show as int
            print(f"  {NS_FIELD_NAMES[j]:>22s}  {mins[j]:>14d}  {maxs[j]:>14d}  {'---':>14s}  {'---':>14s}  {cu}")
        else:
            min_c = mins[j] * cf
            max_c = maxs[j] * cf
            print(f"  {NS_FIELD_NAMES[j]:>22s}  {mins[j]:>14d}  {maxs[j]:>14d}  {min_c:>14.4f}  {max_c:>14.4f}  {cu}")

    # ── 8. Detailed check: baro_alt_rate over time ───────────────────────
    section("BARO_ALT_RATE AND KINEMATICS TIMELINE (SAMPLED)")
    print(f"  Sampling every 500th frame to show how fields evolve over time:")
    print(f"  {'Frame#':>7s}  {'t(s)':>10s}  {'State':>10s}  {'roll':>8s}  {'pitch':>8s}  {'yaw':>8s}"
          f"  {'u_pos':>10s}  {'u_vel':>10s}  {'baro_rate':>10s}  {'flags':>6s}")
    for i in range(0, total_ns, 500):
        fields, _ = nonsensor_frames[i]
        t = t_sec(fields[0])
        state_name = ROCKET_STATES.get(fields[12], "?")
        print(f"  {i:>7d}  {t:>10.3f}  {state_name:>10s}  {fields[1]/100.0:>8.2f}  {fields[2]/100.0:>8.2f}  {fields[3]/100.0:>8.2f}"
              f"  {fields[7]/100.0:>10.3f}  {fields[10]/100.0:>10.3f}  {fields[13]*0.1:>10.3f}  0x{fields[11]:02X}")

    # ── 9. Check for first non-zero kinematics value ─────────────────────
    section("FIRST NON-ZERO VALUES IN KINEMATICS FIELDS")
    kinematics_fields = {
        "roll_raw": 1,
        "pitch_raw": 2,
        "yaw_raw": 3,
        "roll_cmd_raw": 4,
        "e_pos_raw": 5,
        "n_pos_raw": 6,
        "u_pos_raw": 7,
        "e_vel_raw": 8,
        "n_vel_raw": 9,
        "u_vel_raw": 10,
        "baro_alt_rate_raw": 13,
    }
    for name, fidx in kinematics_fields.items():
        found = False
        for i, (fields, _) in enumerate(nonsensor_frames):
            if fields[fidx] != 0:
                t = t_sec(fields[0])
                print(f"  {name:>22s}: first non-zero at frame #{i}  t={t:.4f}s  value={fields[fidx]}")
                found = True
                break
        if not found:
            print(f"  {name:>22s}: ALWAYS ZERO across all {total_ns} frames")

    # ── 10. Time_us sanity check ─────────────────────────────────────────
    section("TIMESTAMP SANITY CHECK")
    prev_t = nonsensor_frames[0][0][0]
    gaps = []
    backwards = 0
    for i in range(1, total_ns):
        cur_t = nonsensor_frames[i][0][0]
        dt = cur_t - prev_t
        if dt < 0:
            backwards += 1
            if backwards <= 5:
                print(f"  *** BACKWARDS timestamp at frame #{i}: {cur_t} < {prev_t}  (dt={dt} us)")
        gaps.append(dt)
        prev_t = cur_t

    if gaps:
        avg_gap = sum(gaps) / len(gaps)
        min_gap = min(gaps)
        max_gap = max(gaps)
        rate = 1e6 / avg_gap if avg_gap > 0 else 0
        print(f"  NonSensor frame intervals:")
        print(f"    Count:     {len(gaps)}")
        print(f"    Min dt:    {min_gap} us  ({min_gap/1000:.2f} ms)")
        print(f"    Max dt:    {max_gap} us  ({max_gap/1000:.2f} ms)")
        print(f"    Avg dt:    {avg_gap:.1f} us  ({avg_gap/1000:.2f} ms)")
        print(f"    Avg rate:  {rate:.1f} Hz")
        print(f"    Backwards: {backwards}")
        print(f"    Time span: {t_sec(nonsensor_frames[0][0][0]):.3f}s to {t_sec(nonsensor_frames[-1][0][0]):.3f}s")

    print()
    print("Done.")


if __name__ == "__main__":
    main()
