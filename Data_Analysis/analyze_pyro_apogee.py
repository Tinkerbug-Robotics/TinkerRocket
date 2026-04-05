#!/usr/bin/env python3
"""
Analyze pyro channel firing timing relative to apogee detection.
Parses the binary flight log and extracts NonSensor data with the
new 43-byte format (includes pyro_status byte).
"""

import sys
import struct

SYNC = b'\xAA\x55\xAA\x55'

MSG_BMP585     = 0xA3
MSG_NON_SENSOR = 0xA5

# NonSensor format: 43 bytes (was 42, added pyro_status byte)
# <I hhhhh iii iii BB h B
#  time_us(4), q0-q3+roll_cmd(5*2=10), e/n/u_pos(3*4=12), e/n/u_vel(3*4=12),
#  flags(1), rocket_state(1), baro_alt_rate_dmps(2), pyro_status(1) = 43
FMT_NONSENSOR_43 = '<I hhhhh iii iii BB h B'
FMT_NONSENSOR_42 = '<I hhhhh iii iii BB h'
FMT_BMP585 = '<I i I'

NSF_ALT_LANDED  = (1 << 0)
NSF_ALT_APOGEE  = (1 << 1)
NSF_VEL_APOGEE  = (1 << 2)
NSF_LAUNCH      = (1 << 3)
NSF_PYRO1_ARMED = (1 << 6)
NSF_PYRO2_ARMED = (1 << 7)

PSF_CH1_CONT  = (1 << 0)
PSF_CH2_CONT  = (1 << 1)
PSF_CH1_FIRED = (1 << 2)
PSF_CH2_FIRED = (1 << 3)

ROCKET_STATES = {0: "INIT", 1: "READY", 2: "PRELAUNCH", 3: "INFLIGHT", 4: "LANDED"}

def _build_crc16_table(poly=0x8001):
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = (crc << 1) ^ poly if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
        table.append(crc)
    return table

_CRC16_TABLE = _build_crc16_table()

def crc16(data, init=0x0000):
    crc = init
    for byte in data:
        crc = _CRC16_TABLE[((crc >> 8) ^ byte) & 0xFF] ^ ((crc << 8) & 0xFFFF)
    return crc

def pressure_to_altitude(p_pa, p0=101325.0):
    if p_pa <= 0: return 0.0
    return 44330.0 * (1.0 - (p_pa / p0) ** (1.0 / 5.255))

def parse(filepath):
    with open(filepath, 'rb') as f:
        data = f.read()

    ns_records = []
    bmp_records = []
    stats = {"frames": 0, "ns_ok": 0, "ns_skip": 0, "bmp_ok": 0}

    pos = 0
    while pos < len(data) - 8:
        idx = data.find(SYNC, pos)
        if idx == -1:
            break
        pos = idx + 4
        if pos + 2 > len(data):
            break

        msg_type = data[pos]
        msg_len = data[pos + 1]
        pos += 2

        if pos + msg_len + 2 > len(data):
            break

        payload = data[pos:pos + msg_len]
        crc_bytes = data[pos + msg_len:pos + msg_len + 2]
        pos += msg_len + 2

        # Validate CRC
        crc_data = bytes([msg_type, msg_len]) + payload
        expected_crc = (crc_bytes[0] << 8) | crc_bytes[1]
        if crc16(crc_data) != expected_crc:
            continue

        stats["frames"] += 1

        if msg_type == MSG_NON_SENSOR:
            if msg_len == 43:
                fields = struct.unpack(FMT_NONSENSOR_43, payload)
                pyro_status = fields[15]
            elif msg_len == 42:
                fields = struct.unpack(FMT_NONSENSOR_42, payload)
                pyro_status = 0
            else:
                stats["ns_skip"] += 1
                continue

            flags = fields[12]
            ns_records.append({
                "time_us":    fields[0],
                "u_pos_m":    fields[8] / 100.0,
                "u_vel_mps":  fields[11] / 100.0,
                "flags":      flags,
                "state":      fields[13],
                "alt_rate":   fields[14] * 0.1,
                "pyro_status": pyro_status,
                "alt_apogee": bool(flags & NSF_ALT_APOGEE),
                "vel_apogee": bool(flags & NSF_VEL_APOGEE),
                "launch":     bool(flags & NSF_LAUNCH),
                "landed":     bool(flags & NSF_ALT_LANDED),
                "p1_armed":   bool(flags & NSF_PYRO1_ARMED),
                "p2_armed":   bool(flags & NSF_PYRO2_ARMED),
                "p1_cont":    bool(pyro_status & PSF_CH1_CONT),
                "p2_cont":    bool(pyro_status & PSF_CH2_CONT),
                "p1_fired":   bool(pyro_status & PSF_CH1_FIRED),
                "p2_fired":   bool(pyro_status & PSF_CH2_FIRED),
            })
            stats["ns_ok"] += 1

        elif msg_type == MSG_BMP585 and msg_len == 12:
            fields = struct.unpack(FMT_BMP585, payload)
            bmp_records.append({
                "time_us":  fields[0],
                "temp_c":   float(fields[1]) / 65536.0,
                "press_pa": float(fields[2]) / 64.0,
            })
            stats["bmp_ok"] += 1

    return ns_records, bmp_records, stats

def main():
    filepath = sys.argv[1] if len(sys.argv) > 1 else "/Users/christianpedersen/Downloads/flight_20260327_135734.bin"
    print(f"Parsing: {filepath}")
    ns, bmp, stats = parse(filepath)
    print(f"Frames: {stats['frames']}, NonSensor: {stats['ns_ok']} (skip {stats['ns_skip']}), BMP: {stats['bmp_ok']}")

    if not ns:
        print("No NonSensor records found!")
        return

    # Find t0 (first launch flag)
    t0 = None
    for r in ns:
        if r["launch"]:
            t0 = r["time_us"]
            break

    if t0 is None:
        print("No launch detected in flight data")
        # Use first record as t0
        t0 = ns[0]["time_us"]

    print(f"\nt0 (launch): {t0} us")
    print(f"Total NonSensor records: {len(ns)}")
    print(f"Duration: {(ns[-1]['time_us'] - ns[0]['time_us']) / 1e6:.1f} s")

    # Find key events
    print("\n=== Key Events ===")

    # State transitions
    prev_state = None
    for r in ns:
        if r["state"] != prev_state:
            t_s = (r["time_us"] - t0) / 1e6
            print(f"  t={t_s:+8.3f}s  State → {ROCKET_STATES.get(r['state'], r['state'])}")
            prev_state = r["state"]

    # First vel_apogee flag
    for r in ns:
        if r["vel_apogee"]:
            t_s = (r["time_us"] - t0) / 1e6
            print(f"  t={t_s:+8.3f}s  vel_u_apogee_flag SET")
            break

    # First alt_apogee flag
    for r in ns:
        if r["alt_apogee"]:
            t_s = (r["time_us"] - t0) / 1e6
            print(f"  t={t_s:+8.3f}s  alt_apogee_flag SET")
            break

    # Both flags set (current pyro trigger)
    for r in ns:
        if r["vel_apogee"] and r["alt_apogee"]:
            t_s = (r["time_us"] - t0) / 1e6
            print(f"  t={t_s:+8.3f}s  BOTH apogee flags SET (pyro trigger point)")
            break

    # Pyro armed
    for r in ns:
        if r["p1_armed"]:
            t_s = (r["time_us"] - t0) / 1e6
            print(f"  t={t_s:+8.3f}s  CH1 ARMED")
            break

    # Pyro fired
    for r in ns:
        if r["p1_fired"]:
            t_s = (r["time_us"] - t0) / 1e6
            print(f"  t={t_s:+8.3f}s  CH1 FIRED")
            break

    for r in ns:
        if r["p2_fired"]:
            t_s = (r["time_us"] - t0) / 1e6
            print(f"  t={t_s:+8.3f}s  CH2 FIRED")
            break

    # Landing
    for r in ns:
        if r["landed"]:
            t_s = (r["time_us"] - t0) / 1e6
            print(f"  t={t_s:+8.3f}s  LANDED flag SET")
            break

    # Max altitude from EKF u_pos
    max_alt = max(ns, key=lambda r: r["u_pos_m"])
    t_s = (max_alt["time_us"] - t0) / 1e6
    print(f"  t={t_s:+8.3f}s  Peak altitude (EKF): {max_alt['u_pos_m']:.1f} m")

    # Baro altitude at key times
    if bmp:
        p0 = bmp[0]["press_pa"] if bmp else 101325.0
        max_baro_alt = 0
        max_baro_t = 0
        for b in bmp:
            alt = pressure_to_altitude(b["press_pa"], p0)
            if alt > max_baro_alt:
                max_baro_alt = alt
                max_baro_t = b["time_us"]
        t_s = (max_baro_t - t0) / 1e6
        print(f"  t={t_s:+8.3f}s  Peak baro altitude: {max_baro_alt:.1f} m")

    # Print a time series around apogee
    print("\n=== NonSensor around apogee (vel_u, alt_rate, flags) ===")
    # Find approximate apogee time (max u_pos)
    apogee_time = max_alt["time_us"]
    print(f"{'t(s)':>8s}  {'u_pos(m)':>8s}  {'u_vel':>7s}  {'alt_rate':>8s}  {'state':>10s}  {'vel_apo':>7s}  {'alt_apo':>7s}  {'p1_arm':>6s}  {'p1_fire':>7s}")
    for r in ns:
        dt = (r["time_us"] - apogee_time) / 1e6
        if -2.0 <= dt <= 10.0:
            t_s = (r["time_us"] - t0) / 1e6
            print(f"  {t_s:+7.3f}  {r['u_pos_m']:8.1f}  {r['u_vel_mps']:7.1f}  {r['alt_rate']:8.1f}  {ROCKET_STATES.get(r['state'], '?'):>10s}  {str(r['vel_apogee']):>7s}  {str(r['alt_apogee']):>7s}  {str(r['p1_armed']):>6s}  {str(r['p1_fired']):>7s}")

if __name__ == "__main__":
    main()
