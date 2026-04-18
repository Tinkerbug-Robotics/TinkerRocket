#!/usr/bin/env python3
"""Trace EKF divergence timeline from flight log."""

import struct
import sys
import math
from collections import defaultdict

SOF = b'\xAA\x55\xAA\x55'

def crc16(data):
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8001) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def parse_file(path):
    with open(path, 'rb') as f:
        data = f.read()

    frames = []
    pos = 0
    while pos < len(data) - 8:
        idx = data.find(SOF, pos)
        if idx < 0: break
        if idx + 6 > len(data): break
        msg_type = data[idx + 4]
        msg_len = data[idx + 5]
        frame_end = idx + 6 + msg_len + 2
        if frame_end > len(data):
            pos = idx + 1
            continue
        crc_data = data[idx+4 : idx+6+msg_len]
        expected_crc = (data[idx+6+msg_len] << 8) | data[idx+6+msg_len+1]
        if crc16(crc_data) != expected_crc:
            pos = idx + 1
            continue
        payload = data[idx+6 : idx+6+msg_len]
        frames.append((msg_type, msg_len, payload))
        pos = frame_end
    return frames

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/Users/christianpedersen/Downloads/flight_20260316_210654.bin'
    frames = parse_file(path)

    # Extract all frame types with timestamps
    ekf_data = []
    gnss_data = []
    imu_data = []
    baro_data = []

    first_time = None
    for msg_type, msg_len, payload in frames:
        if msg_type == 0xA5 and msg_len in (42, 43):  # NON_SENSOR (43=current, 42=legacy)
            # Current: uint32 time + i16*4 quat + i16 roll_cmd + i32*3 pos + i32*3 vel
            #          + u8 flags + u8 rocket_state + i16 baro_alt_rate_dmps + u8 pyro_status
            # Legacy (42 bytes) is the same minus the trailing pyro_status byte.
            fmt = '<I hhhh h iii iii BB h B' if msg_len == 43 else '<I hhhh h iii iii BB h'
            fields = struct.unpack(fmt, payload)
            t = fields[0]
            if first_time is None: first_time = t
            speed = math.sqrt((fields[9]/100.0)**2 + (fields[10]/100.0)**2 + (fields[11]/100.0)**2)
            ekf_data.append({
                'time_us': t,
                'dt_s': (t - first_time) / 1e6,
                'q0': fields[1]/10000.0, 'q1': fields[2]/10000.0,
                'q2': fields[3]/10000.0, 'q3': fields[4]/10000.0,
                'e_pos': fields[6]/100.0, 'n_pos': fields[7]/100.0, 'u_pos': fields[8]/100.0,
                'e_vel': fields[9]/100.0, 'n_vel': fields[10]/100.0, 'u_vel': fields[11]/100.0,
                'speed': speed,
                'state': fields[13],
            })

        elif msg_type == 0xA1 and msg_len == 42:  # GNSS
            fields = struct.unpack('<I HBB BBB H BBB iii iii BB', payload)
            t = fields[0]
            if first_time is None: first_time = t
            gnss_data.append({
                'time_us': t,
                'dt_s': (t - first_time) / 1e6,
                'fix': fields[8], 'sats': fields[9],
                'lat': fields[11]/1e7, 'lon': fields[12]/1e7, 'alt': fields[13]/1000.0,
                'vel_e': fields[14]/1000.0, 'vel_n': fields[15]/1000.0, 'vel_u': fields[16]/1000.0,
                'h_acc': fields[17], 'v_acc': fields[18],
            })

        elif msg_type == 0xA2 and msg_len == 22:  # IMU
            fields = struct.unpack('<I hhh hhh hhh', payload)
            t = fields[0]
            if first_time is None: first_time = t
            imu_data.append({
                'time_us': t,
                'dt_s': (t - first_time) / 1e6,
            })

        elif msg_type == 0xA3 and msg_len == 12:  # BMP585
            fields = struct.unpack('<I i I', payload)
            t = fields[0]
            if first_time is None: first_time = t
            baro_data.append({
                'time_us': t,
                'dt_s': (t - first_time) / 1e6,
            })

    print(f"Parsed: {len(ekf_data)} EKF, {len(gnss_data)} GNSS, {len(imu_data)} IMU, {len(baro_data)} BARO frames")
    print()

    # === FIRST 50 EKF frames (timeline) ===
    print("=== FIRST 50 EKF FRAMES (divergence timeline) ===")
    print(f"{'#':>4} {'t(s)':>8} {'speed':>12} {'u_pos':>10} {'e_vel':>10} {'n_vel':>10} {'u_vel':>10} {'q0':>7} {'q1':>7} {'q2':>7} {'q3':>7} {'st':>3}")
    for i, d in enumerate(ekf_data[:50]):
        print(f"{i:4d} {d['dt_s']:8.4f} {d['speed']:12.2f} {d['u_pos']:10.2f} "
              f"{d['e_vel']:10.2f} {d['n_vel']:10.2f} {d['u_vel']:10.2f} "
              f"{d['q0']:7.4f} {d['q1']:7.4f} {d['q2']:7.4f} {d['q3']:7.4f} {d['state']:3d}")
    print()

    # === GNSS frames around EKF init ===
    print("=== FIRST 20 GNSS FRAMES ===")
    print(f"{'#':>4} {'t(s)':>8} {'fix':>4} {'sats':>5} {'lat':>12} {'lon':>12} {'alt':>8} {'vel_e':>8} {'vel_n':>8} {'vel_u':>8} {'h_acc':>6} {'v_acc':>6}")
    for i, d in enumerate(gnss_data[:20]):
        print(f"{i:4d} {d['dt_s']:8.4f} {d['fix']:4d} {d['sats']:5d} {d['lat']:12.7f} {d['lon']:12.7f} {d['alt']:8.1f} "
              f"{d['vel_e']:8.3f} {d['vel_n']:8.3f} {d['vel_u']:8.3f} {d['h_acc']:6d} {d['v_acc']:6d}")
    print()

    # === Interleaved timeline of EKF + GNSS for first 5 seconds ===
    print("=== INTERLEAVED TIMELINE (first 5s): EKF speed + GNSS arrivals ===")
    events = []
    for d in ekf_data:
        if d['dt_s'] <= 5.0:
            events.append(('EKF', d['dt_s'], f"speed={d['speed']:10.2f} u_pos={d['u_pos']:10.2f} e_vel={d['e_vel']:8.2f} n_vel={d['n_vel']:8.2f} u_vel={d['u_vel']:8.2f}"))
    for d in gnss_data:
        if d['dt_s'] <= 5.0:
            events.append(('GPS', d['dt_s'], f"fix={d['fix']} sats={d['sats']} alt={d['alt']:.1f} h_acc={d['h_acc']} vel=[{d['vel_e']:.3f},{d['vel_n']:.3f},{d['vel_u']:.3f}]"))
    for d in baro_data:
        if d['dt_s'] <= 0.5:  # Only first 0.5s of baro to avoid clutter
            events.append(('BAR', d['dt_s'], "baro update"))

    events.sort(key=lambda x: x[1])
    for ev_type, t, desc in events[:100]:
        print(f"  {t:8.4f}s [{ev_type}] {desc}")

    # === Check: when does speed first exceed 1 m/s? ===
    print()
    for i, d in enumerate(ekf_data):
        if d['speed'] > 1.0:
            print(f"Speed first exceeds 1 m/s at EKF frame {i}, t={d['dt_s']:.4f}s, speed={d['speed']:.2f}")
            # How many GNSS frames before this?
            gnss_before = sum(1 for g in gnss_data if g['dt_s'] < d['dt_s'])
            print(f"  GNSS frames received before this: {gnss_before}")
            # How many IMU frames before this?
            imu_before = sum(1 for im in imu_data if im['dt_s'] < d['dt_s'])
            print(f"  IMU frames before this: {imu_before}")
            break

    # === Check: state transitions ===
    print()
    state_names = {0: 'INIT', 1: 'READY', 2: 'PRELAUNCH', 3: 'INFLIGHT', 4: 'LANDED'}
    prev_state = None
    for d in ekf_data:
        if d['state'] != prev_state:
            print(f"State: {state_names.get(prev_state, 'None')} -> {state_names.get(d['state'], '?')} at t={d['dt_s']:.4f}s")
            prev_state = d['state']

if __name__ == '__main__':
    main()
