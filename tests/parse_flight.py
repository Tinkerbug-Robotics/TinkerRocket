#!/usr/bin/env python3
"""Parse TinkerRocket binary flight log and print summary + anomalies."""

import struct
import sys
import math
from collections import defaultdict

SOF = b'\xAA\x55\xAA\x55'

MSG_TYPES = {
    0xA0: ("OUT_STATUS_QUERY", 16),
    0xA1: ("GNSS", 42),
    0xA2: ("ISM6HG256", 22),
    0xA3: ("BMP585", 12),
    0xA4: ("MMC5983MA", 16),
    0xA5: ("NON_SENSOR", 43),
    0xA6: ("POWER", 10),
    0xA7: ("START_LOGGING", 0),
    0xA8: ("END_FLIGHT", 0),
    0xF1: ("LORA", 57),
}

# NonSensor pyro_status bit masks (must match RocketComputerTypes.h PSF_* constants)
PSF_CH1_CONT         = 1 << 0
PSF_CH2_CONT         = 1 << 1
PSF_CH1_FIRED        = 1 << 2
PSF_CH2_FIRED        = 1 << 3
PSF_REBOOT_RECOVERY  = 1 << 4
PSF_GUIDANCE_ENABLED = 1 << 5  # FC's live guidance_enabled config (bit 5)

def crc16(data):
    """CRC-16 matching RobTillaart CRC library defaults: poly=0x8001, init=0, no reversal."""
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8001) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def parse_gnss(payload):
    fields = struct.unpack('<I HBB BBB H BBB iii iii BB', payload)
    return {
        'time_us': fields[0],
        'year': fields[1], 'month': fields[2], 'day': fields[3],
        'hour': fields[4], 'minute': fields[5], 'second': fields[6], 'ms': fields[7],
        'fix': fields[8], 'sats': fields[9], 'pdop_x10': fields[10],
        'lat': fields[11] / 1e7, 'lon': fields[12] / 1e7, 'alt_m': fields[13] / 1000.0,
        'vel_e': fields[14] / 1000.0, 'vel_n': fields[15] / 1000.0, 'vel_u': fields[16] / 1000.0,
        'h_acc': fields[17], 'v_acc': fields[18],
    }

def parse_imu(payload):
    fields = struct.unpack('<I hhh hhh hhh', payload)
    return {
        'time_us': fields[0],
        'acc_low': (fields[1], fields[2], fields[3]),
        'acc_high': (fields[4], fields[5], fields[6]),
        'gyro': (fields[7], fields[8], fields[9]),
    }

def parse_baro(payload):
    fields = struct.unpack('<I i I', payload)
    return {
        'time_us': fields[0],
        'temp_c': fields[1] / 65536.0,
        'press_pa': fields[2] / 64.0,
    }

def parse_mag(payload):
    fields = struct.unpack('<I III', payload)
    return {
        'time_us': fields[0],
        'mag_x': fields[1], 'mag_y': fields[2], 'mag_z': fields[3],
    }

def parse_nonsensor(payload):
    # Struct: uint32 time, i16*4 quat, i16 roll_cmd, i32*3 pos, i32*3 vel,
    #         u8 flags, u8 rocket_state, i16 baro_alt_rate_dmps, u8 pyro_status
    fields = struct.unpack('<I hhhh h iii iii BB h B', payload)
    pyro_status = fields[15]
    return {
        'time_us': fields[0],
        'q0': fields[1] / 10000.0, 'q1': fields[2] / 10000.0,
        'q2': fields[3] / 10000.0, 'q3': fields[4] / 10000.0,
        'roll_cmd_deg': fields[5] / 100.0,
        'e_pos_m': fields[6] / 100.0, 'n_pos_m': fields[7] / 100.0, 'u_pos_m': fields[8] / 100.0,
        'e_vel_ms': fields[9] / 100.0, 'n_vel_ms': fields[10] / 100.0, 'u_vel_ms': fields[11] / 100.0,
        'flags': fields[12],
        'rocket_state': fields[13],
        'baro_alt_rate_ms': fields[14] / 10.0,
        'pyro_status': pyro_status,
        'guidance_enabled': bool(pyro_status & PSF_GUIDANCE_ENABLED),
    }

def parse_power(payload):
    fields = struct.unpack('<I HhH', payload)
    # Wait - let me re-check the struct: uint32 time, uint16 voltage_raw, int16 current_raw, int16 soc_raw
    # Actually: uint32 time, uint16 voltage, int16 current, int16 soc = 4+2+2+2=10
    fields = struct.unpack('<I Hhh', payload)
    return {
        'time_us': fields[0],
        'voltage': fields[1] * 10.0 / 65535.0,
        'current_ma': fields[2] * 10000.0 / 32767.0,
        'soc': fields[3] * 150.0 / 32767.0 - 25.0,
    }

def decode_i24(b0, b1, b2):
    val = b0 | (b1 << 8) | (b2 << 16)
    if val & 0x800000:
        val -= 0x1000000
    return val

def parse_lora(payload):
    # Manual parse due to i24le_t fields
    off = 0
    num_sats_raw = payload[off]; off += 1
    num_sats = num_sats_raw & 0x7F
    logging = bool(num_sats_raw & 0x80)
    pdop_u8 = payload[off]; off += 1
    ecef_x = decode_i24(payload[off], payload[off+1], payload[off+2]); off += 3
    ecef_y = decode_i24(payload[off], payload[off+1], payload[off+2]); off += 3
    ecef_z = decode_i24(payload[off], payload[off+1], payload[off+2]); off += 3
    hacc = payload[off]; off += 1
    flags_state = payload[off]; off += 1
    acc_x, acc_y, acc_z = struct.unpack_from('<hhh', payload, off); off += 6
    gyro_x, gyro_y, gyro_z = struct.unpack_from('<hhh', payload, off); off += 6
    temp = struct.unpack_from('<h', payload, off)[0]; off += 2
    voltage_u8 = payload[off]; off += 1
    current_ma = struct.unpack_from('<h', payload, off)[0]; off += 2
    soc_i8 = struct.unpack_from('<b', payload, off)[0]; off += 1
    palt = decode_i24(payload[off], payload[off+1], payload[off+2]); off += 3
    alt_rate = struct.unpack_from('<h', payload, off)[0]; off += 2
    max_alt = decode_i24(payload[off], payload[off+1], payload[off+2]); off += 3
    max_speed = struct.unpack_from('<h', payload, off)[0]; off += 2
    roll_cd, pitch_cd, yaw_cd = struct.unpack_from('<hhh', payload, off); off += 6
    q0, q1, q2, q3 = struct.unpack_from('<hhhh', payload, off); off += 8
    speed = struct.unpack_from('<h', payload, off)[0]; off += 2

    rocket_state = (flags_state >> 4) & 0x07
    return {
        'num_sats': num_sats, 'logging': logging, 'pdop': pdop_u8,
        'ecef_x': ecef_x, 'ecef_y': ecef_y, 'ecef_z': ecef_z,
        'hacc': hacc,
        'rocket_state': rocket_state,
        'launch_flag': bool(flags_state & 0x01),
        'vel_apogee': bool(flags_state & 0x02),
        'alt_apogee': bool(flags_state & 0x04),
        'alt_landed': bool(flags_state & 0x08),
        'acc_x': acc_x / 10.0, 'acc_y': acc_y / 10.0, 'acc_z': acc_z / 10.0,
        'gyro_x': gyro_x / 10.0, 'gyro_y': gyro_y / 10.0, 'gyro_z': gyro_z / 10.0,
        'temp_c': temp / 10.0,
        'voltage_u8': voltage_u8,
        'current_ma': current_ma, 'soc': soc_i8,
        'pressure_alt': palt, 'alt_rate': alt_rate,
        'max_alt': max_alt, 'max_speed': max_speed,
        'roll': roll_cd / 100.0, 'pitch': pitch_cd / 100.0, 'yaw': yaw_cd / 100.0,
        'q0': q0 / 10000.0, 'q1': q1 / 10000.0, 'q2': q2 / 10000.0, 'q3': q3 / 10000.0,
        'speed': speed,
    }

def parse_file(path):
    with open(path, 'rb') as f:
        data = f.read()

    print(f"File size: {len(data)} bytes\n")

    frames = []
    counts = defaultdict(int)
    crc_errors = 0
    pos = 0

    while pos < len(data) - 8:
        # Find next SOF
        idx = data.find(SOF, pos)
        if idx < 0:
            break
        if idx + 6 > len(data):
            break

        msg_type = data[idx + 4]
        msg_len = data[idx + 5]

        frame_end = idx + 6 + msg_len + 2
        if frame_end > len(data):
            pos = idx + 1
            continue

        # CRC check
        crc_data = data[idx+4 : idx+6+msg_len]
        crc_hi = data[idx+6+msg_len]
        crc_lo = data[idx+6+msg_len+1]
        expected_crc = (crc_hi << 8) | crc_lo
        actual_crc = crc16(crc_data)

        if actual_crc != expected_crc:
            crc_errors += 1
            pos = idx + 1
            continue

        payload = data[idx+6 : idx+6+msg_len]
        type_name = MSG_TYPES.get(msg_type, (f"UNK_0x{msg_type:02X}", None))[0]
        counts[type_name] += 1

        parsed = None
        if msg_type == 0xA1 and msg_len == 42:
            parsed = parse_gnss(payload)
        elif msg_type == 0xA2 and msg_len == 22:
            parsed = parse_imu(payload)
        elif msg_type == 0xA3 and msg_len == 12:
            parsed = parse_baro(payload)
        elif msg_type == 0xA4 and msg_len == 16:
            parsed = parse_mag(payload)
        elif msg_type == 0xA5 and msg_len == 43:
            parsed = parse_nonsensor(payload)
        elif msg_type == 0xA6 and msg_len == 10:
            parsed = parse_power(payload)
        elif msg_type == 0xF1 and msg_len == 57:
            parsed = parse_lora(payload)

        frames.append((type_name, msg_type, parsed))
        pos = frame_end

    # === SUMMARY ===
    print("=== FRAME COUNTS ===")
    for name, count in sorted(counts.items()):
        print(f"  {name}: {count}")
    print(f"  CRC errors: {crc_errors}")
    print()

    # === TIME RANGE ===
    first_time = None
    last_time = None
    for name, mtype, p in frames:
        if p and 'time_us' in p:
            if first_time is None:
                first_time = p['time_us']
            last_time = p['time_us']
    if first_time and last_time:
        dur = (last_time - first_time) / 1e6
        print(f"=== TIME RANGE ===")
        print(f"  First: {first_time} us")
        print(f"  Last:  {last_time} us")
        print(f"  Duration: {dur:.2f} s")
        print()

    # === IMU ANALYSIS ===
    imu_frames = [(n, p) for n, mt, p in frames if n == 'ISM6HG256' and p]
    if imu_frames:
        print(f"=== IMU ({len(imu_frames)} samples) ===")
        # Check sample rate
        times = [p['time_us'] for _, p in imu_frames]
        if len(times) > 1:
            dts = [(times[i+1] - times[i]) / 1000.0 for i in range(len(times)-1)]
            avg_dt = sum(dts) / len(dts)
            max_dt = max(dts)
            min_dt = min(dts)
            print(f"  Avg dt: {avg_dt:.1f} ms ({1000.0/avg_dt:.1f} Hz)")
            print(f"  Min dt: {min_dt:.1f} ms, Max dt: {max_dt:.1f} ms")
            # Flag gaps > 5x average
            gaps = [(i, dt) for i, dt in enumerate(dts) if dt > avg_dt * 5]
            if gaps:
                print(f"  WARNING: {len(gaps)} gaps > 5x avg dt:")
                for i, dt in gaps[:10]:
                    print(f"    Sample {i}: {dt:.1f} ms gap")

        # Accel range
        low_x = [p['acc_low'][0] for _, p in imu_frames]
        low_y = [p['acc_low'][1] for _, p in imu_frames]
        low_z = [p['acc_low'][2] for _, p in imu_frames]
        high_x = [p['acc_high'][0] for _, p in imu_frames]
        high_y = [p['acc_high'][1] for _, p in imu_frames]
        high_z = [p['acc_high'][2] for _, p in imu_frames]
        gyro_x = [p['gyro'][0] for _, p in imu_frames]
        gyro_y = [p['gyro'][1] for _, p in imu_frames]
        gyro_z = [p['gyro'][2] for _, p in imu_frames]
        print(f"  Low-G accel raw range: X[{min(low_x)},{max(low_x)}] Y[{min(low_y)},{max(low_y)}] Z[{min(low_z)},{max(low_z)}]")
        print(f"  High-G accel raw range: X[{min(high_x)},{max(high_x)}] Y[{min(high_y)},{max(high_y)}] Z[{min(high_z)},{max(high_z)}]")
        print(f"  Gyro raw range: X[{min(gyro_x)},{max(gyro_x)}] Y[{min(gyro_y)},{max(gyro_y)}] Z[{min(gyro_z)},{max(gyro_z)}]")

        # Check for saturation (±32767)
        SAT = 32700
        for axis_name, vals in [('low_x', low_x), ('low_y', low_y), ('low_z', low_z),
                                 ('high_x', high_x), ('high_y', high_y), ('high_z', high_z),
                                 ('gyro_x', gyro_x), ('gyro_y', gyro_y), ('gyro_z', gyro_z)]:
            sat_count = sum(1 for v in vals if abs(v) >= SAT)
            if sat_count > 0:
                print(f"  WARNING: {axis_name} saturated in {sat_count} samples")
        print()

    # === BARO ANALYSIS ===
    baro_frames = [(n, p) for n, mt, p in frames if n == 'BMP585' and p]
    if baro_frames:
        print(f"=== BAROMETER ({len(baro_frames)} samples) ===")
        temps = [p['temp_c'] for _, p in baro_frames]
        press = [p['press_pa'] for _, p in baro_frames]
        print(f"  Temp: {min(temps):.1f} to {max(temps):.1f} °C")
        print(f"  Pressure: {min(press):.1f} to {max(press):.1f} Pa")
        # Check for unreasonable values
        if any(t < -40 or t > 85 for t in temps):
            print(f"  WARNING: Temperature out of sensor range!")
        if any(p < 10000 or p > 120000 for p in press):
            print(f"  WARNING: Pressure out of reasonable range!")

        times = [p['time_us'] for _, p in baro_frames]
        if len(times) > 1:
            dts = [(times[i+1] - times[i]) / 1000.0 for i in range(len(times)-1)]
            avg_dt = sum(dts) / len(dts)
            print(f"  Avg dt: {avg_dt:.1f} ms ({1000.0/avg_dt:.1f} Hz)")
        print()

    # === GNSS ANALYSIS ===
    gnss_frames = [(n, p) for n, mt, p in frames if n == 'GNSS' and p]
    if gnss_frames:
        print(f"=== GNSS ({len(gnss_frames)} samples) ===")
        fixes = [p['fix'] for _, p in gnss_frames]
        sats = [p['sats'] for _, p in gnss_frames]
        print(f"  Fix modes: {set(fixes)}")
        print(f"  Sats range: {min(sats)} to {max(sats)}")
        lats = [p['lat'] for _, p in gnss_frames if p['fix'] >= 2]
        lons = [p['lon'] for _, p in gnss_frames if p['fix'] >= 2]
        alts = [p['alt_m'] for _, p in gnss_frames if p['fix'] >= 2]
        if lats:
            print(f"  Lat: {min(lats):.7f} to {max(lats):.7f}")
            print(f"  Lon: {min(lons):.7f} to {max(lons):.7f}")
            print(f"  Alt: {min(alts):.1f} to {max(alts):.1f} m")
        speeds = [math.sqrt(p['vel_e']**2 + p['vel_n']**2 + p['vel_u']**2)
                  for _, p in gnss_frames if p['fix'] >= 2]
        if speeds:
            print(f"  GNSS speed: {min(speeds):.2f} to {max(speeds):.2f} m/s")
        print()

    # === NON-SENSOR (EKF/STATE) ANALYSIS ===
    ns_frames = [(n, p) for n, mt, p in frames if n == 'NON_SENSOR' and p]
    if ns_frames:
        print(f"=== EKF/STATE ({len(ns_frames)} samples) ===")
        states = [p['rocket_state'] for _, p in ns_frames]
        state_names = {0: 'INIT', 1: 'READY', 2: 'PRELAUNCH', 3: 'INFLIGHT', 4: 'LANDED'}
        state_set = set(states)
        print(f"  States seen: {[state_names.get(s, f'UNK({s})') for s in sorted(state_set)]}")

        # State transitions
        prev_state = None
        print("  State transitions:")
        for _, p in ns_frames:
            s = p['rocket_state']
            if s != prev_state:
                t_s = p['time_us'] / 1e6
                print(f"    t={t_s:.2f}s: {state_names.get(prev_state, 'None')} -> {state_names.get(s, f'UNK({s})')}")
                prev_state = s

        # Altitude
        u_pos = [p['u_pos_m'] for _, p in ns_frames]
        print(f"  EKF altitude: {min(u_pos):.2f} to {max(u_pos):.2f} m")

        # Velocity
        speeds = [math.sqrt(p['e_vel_ms']**2 + p['n_vel_ms']**2 + p['u_vel_ms']**2)
                  for _, p in ns_frames]
        print(f"  EKF speed: {min(speeds):.2f} to {max(speeds):.2f} m/s")

        u_vel = [p['u_vel_ms'] for _, p in ns_frames]
        print(f"  EKF vertical vel: {min(u_vel):.2f} to {max(u_vel):.2f} m/s")

        # Quaternion health
        qnorms = [math.sqrt(p['q0']**2 + p['q1']**2 + p['q2']**2 + p['q3']**2)
                   for _, p in ns_frames]
        print(f"  Quaternion norm: {min(qnorms):.4f} to {max(qnorms):.4f} (should be ~1.0)")
        bad_q = sum(1 for q in qnorms if abs(q - 1.0) > 0.05)
        if bad_q:
            print(f"  WARNING: {bad_q} samples with quaternion norm > 5% from 1.0")

        # Baro alt rate
        baro_rates = [p['baro_alt_rate_ms'] for _, p in ns_frames]
        print(f"  Baro alt rate: {min(baro_rates):.1f} to {max(baro_rates):.1f} m/s")

        # Flags
        launch_times = [p['time_us']/1e6 for _, p in ns_frames if p['flags'] & 0x08]
        apogee_times = [p['time_us']/1e6 for _, p in ns_frames if p['flags'] & 0x02]
        landed_times = [p['time_us']/1e6 for _, p in ns_frames if p['flags'] & 0x01]
        if launch_times:
            print(f"  Launch flag first set at t={launch_times[0]:.2f}s")
        if apogee_times:
            print(f"  Apogee flag first set at t={apogee_times[0]:.2f}s")
        if landed_times:
            print(f"  Landed flag first set at t={landed_times[0]:.2f}s")

        # Roll command (deg) — populated by servo_control PID in flight, and
        # by the standalone roll PID during ground test.
        roll_cmds = [p['roll_cmd_deg'] for _, p in ns_frames]
        nonzero_rc = sum(1 for r in roll_cmds if abs(r) > 0.005)
        print(f"  Roll cmd: {min(roll_cmds):+.2f} to {max(roll_cmds):+.2f} deg "
              f"({nonzero_rc} samples non-zero)")

        # Live guidance_enabled from FC (pyro_status bit 5). Should be stable
        # during a session; a flip mid-run indicates a runtime toggle.
        ge_vals = [p.get('guidance_enabled', False) for _, p in ns_frames]
        ge_on = sum(1 for v in ge_vals if v)
        ge_off = len(ge_vals) - ge_on
        if ge_on and ge_off:
            print(f"  FC guidance_enabled: MIXED ({ge_on} ON / {ge_off} OFF) — toggled during run")
        else:
            print(f"  FC guidance_enabled: {'ON' if ge_on else 'OFF'} (entire run)")
        print()

    # === LORA ANALYSIS ===
    lora_frames = [(n, p) for n, mt, p in frames if n == 'LORA' and p]
    if lora_frames:
        print(f"=== LORA ({len(lora_frames)} samples) ===")
        max_alts = [p['max_alt'] for _, p in lora_frames]
        max_speeds = [p['max_speed'] for _, p in lora_frames]
        speeds = [p['speed'] for _, p in lora_frames]
        palt = [p['pressure_alt'] for _, p in lora_frames]
        print(f"  Max alt reported: {min(max_alts)} to {max(max_alts)} m")
        print(f"  Max speed reported: {min(max_speeds)} to {max(max_speeds)} m/s")
        print(f"  Current speed: {min(speeds)} to {max(speeds)} m/s")
        print(f"  Pressure alt: {min(palt)} to {max(palt)} m")
        # Check for absurd values
        if any(abs(s) > 1000 for s in max_speeds):
            print(f"  *** ANOMALY: max_speed has unreasonable values! ***")
        if any(abs(s) > 1000 for s in speeds):
            print(f"  *** ANOMALY: speed has unreasonable values! ***")
        if any(abs(a) > 100000 for a in max_alts):
            print(f"  *** ANOMALY: max_alt has unreasonable values! ***")
        print()

    # === POWER ANALYSIS ===
    pwr_frames = [(n, p) for n, mt, p in frames if n == 'POWER' and p]
    if pwr_frames:
        print(f"=== POWER ({len(pwr_frames)} samples) ===")
        volts = [p['voltage'] for _, p in pwr_frames]
        currents = [p['current_ma'] for _, p in pwr_frames]
        socs = [p['soc'] for _, p in pwr_frames]
        print(f"  Voltage: {min(volts):.2f} to {max(volts):.2f} V")
        print(f"  Current: {min(currents):.1f} to {max(currents):.1f} mA")
        print(f"  SOC: {min(socs):.1f} to {max(socs):.1f} %")
        print()

    # === MAGNETOMETER ANALYSIS ===
    mag_frames = [(n, p) for n, mt, p in frames if n == 'MMC5983MA' and p]
    if mag_frames:
        print(f"=== MAGNETOMETER ({len(mag_frames)} samples) ===")
        mx = [p['mag_x'] for _, p in mag_frames]
        my = [p['mag_y'] for _, p in mag_frames]
        mz = [p['mag_z'] for _, p in mag_frames]
        print(f"  Raw X: {min(mx)} to {max(mx)}")
        print(f"  Raw Y: {min(my)} to {max(my)}")
        print(f"  Raw Z: {min(mz)} to {max(mz)}")
        # MMC5983MA midpoint is ~131072 (2^17), range 0-262143 (2^18-1)
        # Check if centered around midpoint
        mid = 131072
        print(f"  X offset from mid: {sum(mx)/len(mx) - mid:.0f}")
        print(f"  Y offset from mid: {sum(my)/len(my) - mid:.0f}")
        print(f"  Z offset from mid: {sum(mz)/len(mz) - mid:.0f}")
        print()

    # === DETAILED ANOMALY CHECK ===
    print("=== ANOMALY SUMMARY ===")
    anomalies = []

    # Check for EKF divergence on bench test
    if ns_frames:
        max_speed = max(math.sqrt(p['e_vel_ms']**2 + p['n_vel_ms']**2 + p['u_vel_ms']**2)
                        for _, p in ns_frames)
        max_alt = max(abs(p['u_pos_m']) for _, p in ns_frames)
        if max_speed > 1.0:
            anomalies.append(f"EKF speed reached {max_speed:.2f} m/s on bench test (should be ~0)")
        if max_alt > 2.0:
            anomalies.append(f"EKF altitude drifted to {max_alt:.2f} m on bench test (should be ~0)")

    # Check for state machine errors on bench
    if ns_frames:
        if 3 in state_set:  # INFLIGHT
            anomalies.append("Rocket entered INFLIGHT state during bench test!")

    # LoRa max_speed anomaly
    if lora_frames:
        lora_max_sp = max(abs(p['max_speed']) for _, p in lora_frames)
        if lora_max_sp > 10:
            anomalies.append(f"LoRa max_speed = {lora_max_sp} m/s on bench test (should be ~0)")
        lora_sp = max(abs(p['speed']) for _, p in lora_frames)
        if lora_sp > 10:
            anomalies.append(f"LoRa current speed = {lora_sp} m/s on bench test")

    if not anomalies:
        print("  No anomalies detected.")
    else:
        for a in anomalies:
            print(f"  * {a}")


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else '/Users/christianpedersen/Downloads/flight_20260316_173157.bin'
    parse_file(path)
