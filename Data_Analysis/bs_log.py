#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read a base-station log — binary (current) or CSV (legacy).

The base station used to write a 39-column CSV straight to storage, one row per
received packet, formatting every float on the MCU.  Since #850 it logs the
bytes it actually received, framed exactly like the rocket computer's log
(``AA 55 AA 55`` + type + len + payload + CRC16), and the CSV is generated
wherever it is needed.  That is ~174 B per packet down to ~70, and it keeps a
record of *what arrived* rather than a snapshot of an accumulator.

Both formats are read here so nothing already on disk is stranded — the repo's
own ``examples/flights/lora_20260705_173025.csv`` is a 1607-row CSV, and CSV
cannot be converted back to binary (it is derived and lossy).  Callers get the
same row dicts either way; ``fmt`` says which file they came from.

THE FORWARD-FILL.  A binary log holds two interleaved frame types: a 55-byte
FAST frame five times out of six and a 22-byte SLOW frame on the sixth.  Each
carries a subset, so a row is built by merging the frame into a running
accumulator per rocket, exactly as the base station does in RAM.  That is what
makes the rows rectangular, and it is why a slow frame does not blank the
position: an unpacker only writes what its frame carries.
"""

import csv
import math
import struct

# --- framing, shared with the rocket-computer log -------------------------
PREAMBLE = b'\xAA\x55\xAA\x55'

BS_LOG_MAGIC = b'TRBSLOG'
BS_LOG_FORMAT_VERSION = 1

BS_LORA_RX_MSG = 0xFC
BS_EVENT_MSG = 0xFD

# --- LoRa frames (#850) ---------------------------------------------------
SIZE_OF_LORA_FAST = 55
SIZE_OF_LORA_SLOW = 22
LORA_PROTO_VERSION = 5
LORA_FRAME_FAST = 0x0
LORA_FRAME_SLOW = 0x1

# LoRaFrameHeader: network_id, rocket_id, next_channel_idx, seq, ver_type, flags_state
FMT_LORA_HDR = '<BBBHBB'
LORA_HDR_LEN = 7

# BsLoRaRxHeader: time_ms, rssi_x10, snr_x10, rx_freq_hz
FMT_BS_RX_HDR = '<IhhI'
BS_RX_HDR_LEN = 12

# BsEventHeader: time_ms, rx_freq_hz, text_len
FMT_BS_EVT_HDR = '<IIB'
BS_EVT_HDR_LEN = 9

BS_RSSI_UNKNOWN = -32768

LORA_LAUNCH = 1 << 0
LORA_VEL_APOGEE = 1 << 1
LORA_ALT_APOGEE = 1 << 2
LORA_ALT_LANDED = 1 << 3
LORA_CAMERA_REC = 1 << 4
LORA_STATE_SHIFT = 4

LORA_NUM_SATS_MASK = 0x3F
LORA_SIM_BIT = 0x40
LORA_LOGGING_BIT = 0x80

STATE_NAMES = ['INIT', 'READY', 'PRELAUNCH', 'INFLIGHT', 'LANDED', 'MAG_CAL', '6', '7']


def _i24(b, off):
    """Signed little-endian 24-bit, matching i24le_t on the wire."""
    v = b[off] | (b[off + 1] << 8) | (b[off + 2] << 16)
    return v - 0x1000000 if v & 0x800000 else v


def _decode_voltage_2_10_01(u8):
    """encodeVoltage_2_10_01 inverse: 2.00-10.00 V in 1/32 V steps."""
    return 2.0 + (u8 / 255.0) * 8.0


def iter_frames(blob):
    """Yield (type, payload) for every length-plausible frame in a blob."""
    i = 0
    while True:
        p = blob.find(PREAMBLE, i)
        if p < 0:
            return
        j = p + len(PREAMBLE)
        if j + 3 >= len(blob):
            return
        mtype, length = blob[j], blob[j + 1]
        j += 2
        if j + length + 2 > len(blob):
            return
        yield mtype, blob[j:j + length]
        i = j + length + 2


def _unpack_lora_header(frame, row):
    nid, rid, nch, seq, ver_type, flags = struct.unpack_from(FMT_LORA_HDR, frame, 0)
    row['network_id'] = nid
    row['rocket_id'] = rid
    row['next_ch'] = nch
    row['seq'] = seq
    row['launch'] = 1 if flags & LORA_LAUNCH else 0
    row['vel_apo'] = 1 if flags & LORA_VEL_APOGEE else 0
    row['alt_apo'] = 1 if flags & LORA_ALT_APOGEE else 0
    row['landed'] = 1 if flags & LORA_ALT_LANDED else 0
    row['camera_rec'] = 1 if flags & LORA_CAMERA_REC else 0
    row['state'] = STATE_NAMES[(flags >> LORA_STATE_SHIFT) & 0x07]
    return ver_type


def _unpack_fast(frame, row):
    """Write ONLY the fields the fast frame carries — never clear the rest."""
    o = LORA_HDR_LEN
    sats = frame[o]
    row['num_sats'] = sats & LORA_NUM_SATS_MASK
    row['sim_active'] = bool(sats & LORA_SIM_BIT)
    row['logging'] = bool(sats & LORA_LOGGING_BIT)
    row['pdop'] = float(frame[o + 1])
    row['h_acc'] = float(frame[o + 2])
    row['ecef_x'] = float(_i24(frame, o + 3))
    row['ecef_y'] = float(_i24(frame, o + 6))
    row['ecef_z'] = float(_i24(frame, o + 9))
    ax, ay, az, gx, gy, gz, q0, q1, q2, q3 = struct.unpack_from('<10h', frame, o + 12)
    row['acc_x'], row['acc_y'], row['acc_z'] = ax / 10.0, ay / 10.0, az / 10.0
    row['gyro_x'], row['gyro_y'], row['gyro_z'] = gx / 10.0, gy / 10.0, gz / 10.0
    row['q0'], row['q1'] = q0 / 10000.0, q1 / 10000.0
    row['q2'], row['q3'] = q2 / 10000.0, q3 / 10000.0
    row['pressure_alt'] = float(_i24(frame, o + 32))
    (row['alt_rate'],) = struct.unpack_from('<h', frame, o + 35)
    ve, vn, vu = struct.unpack_from('<3h', frame, o + 37)
    row['vel_e'], row['vel_n'], row['vel_u'] = ve / 10.0, vn / 10.0, vu / 10.0
    (row['sensor_health'],) = struct.unpack_from('<I', frame, o + 43)
    row['flags2'] = frame[o + 47]
    # Derived on the ground since #191 — both inputs ride THIS frame, so the
    # derivation is always self-consistent rather than mixing a fresh
    # quaternion with a stale velocity.
    row['speed'] = math.sqrt(row['vel_e'] ** 2 + row['vel_n'] ** 2 + row['vel_u'] ** 2)


def _unpack_slow(frame, row):
    """Write ONLY the fields the slow frame carries."""
    o = LORA_HDR_LEN
    row['max_alt'] = float(_i24(frame, o))
    (row['max_speed'],) = struct.unpack_from('<h', frame, o + 3)
    (temp_x10,) = struct.unpack_from('<h', frame, o + 5)
    row['temp'] = temp_x10 / 10.0
    row['voltage'] = _decode_voltage_2_10_01(frame[o + 7])
    (row['current'],) = struct.unpack_from('<h', frame, o + 8)
    row['soc'] = struct.unpack_from('<b', frame, o + 10)[0]
    cam_ma, servo_ma = struct.unpack_from('<2H', frame, o + 11)
    row['cam_a'] = cam_ma / 1000.0
    row['servo_a'] = servo_ma / 1000.0


def read_bs_bin(path):
    """Parse a binary base-station log.

    Returns (rows, events).  Rows are forward-filled per rocket_id, so every
    row is a complete picture even though each frame carried only a subset.
    """
    with open(path, 'rb') as fh:
        blob = fh.read()

    if not blob.startswith(BS_LOG_MAGIC):
        raise ValueError(f'{path}: not a base-station binary log '
                         f'(missing {BS_LOG_MAGIC!r} magic)')
    ver = blob[len(BS_LOG_MAGIC)] if len(blob) > len(BS_LOG_MAGIC) else 0
    if ver != BS_LOG_FORMAT_VERSION:
        raise ValueError(f'{path}: log format v{ver}, this reader speaks '
                         f'v{BS_LOG_FORMAT_VERSION}')

    rows, events = [], []
    accum = {}           # rocket_id -> running row
    last_seq = {}        # rocket_id -> previous seq, for the gap column

    for mtype, payload in iter_frames(blob):
        if mtype == BS_EVENT_MSG:
            if len(payload) < BS_EVT_HDR_LEN:
                continue
            t_ms, freq_hz, tlen = struct.unpack_from(FMT_BS_EVT_HDR, payload, 0)
            text = payload[BS_EVT_HDR_LEN:BS_EVT_HDR_LEN + tlen].decode('utf-8', 'replace')
            events.append(dict(time_ms=t_ms, t_s=t_ms / 1000.0,
                               rx_freq_mhz=freq_hz / 1e6, event=text))
            continue

        if mtype != BS_LORA_RX_MSG or len(payload) <= BS_RX_HDR_LEN:
            continue

        t_ms, rssi_x10, snr_x10, freq_hz = struct.unpack_from(FMT_BS_RX_HDR, payload, 0)
        frame = payload[BS_RX_HDR_LEN:]
        if len(frame) not in (SIZE_OF_LORA_FAST, SIZE_OF_LORA_SLOW):
            continue

        probe = {}
        ver_type = _unpack_lora_header(frame, probe)
        if (ver_type >> 4) != LORA_PROTO_VERSION:
            continue
        ftype = ver_type & 0x0F

        rid = probe['rocket_id']
        row = dict(accum.get(rid, {}))          # forward-fill from this rocket
        _unpack_lora_header(frame, row)
        if ftype == LORA_FRAME_SLOW:
            _unpack_slow(frame, row)
        else:
            _unpack_fast(frame, row)

        row['time_ms'] = t_ms
        row['t_s'] = t_ms / 1000.0
        # NaN rather than a plausible-looking 0 dBm when the radio had no reading.
        row['rssi'] = float('nan') if rssi_x10 == BS_RSSI_UNKNOWN else rssi_x10 / 10.0
        row['snr'] = float('nan') if snr_x10 == BS_RSSI_UNKNOWN else snr_x10 / 10.0
        row['rx_freq_mhz'] = freq_hz / 1e6
        row['frame'] = 'slow' if ftype == LORA_FRAME_SLOW else 'fast'

        prev = last_seq.get(rid)
        row['gap'] = -1 if prev is None else ((row['seq'] - prev - 1) & 0xFFFF)
        last_seq[rid] = row['seq']

        # Derive the fields every consumer would otherwise recompute, so a row
        # from the reader is the same complete record the CSV carries. Tools
        # outside the app read the BINARY, not the rendered CSV, so anything the
        # CSV has must be reachable here or those tools quietly see nothing —
        # analyze_bs_gnss reported 100% "blanked position" for exactly that
        # reason, because it looked for `lat` and the rows only had ECEF.
        lat = lon = alt = float('nan')
        if row.get('num_sats', 0) > 0 and any(
                row.get(k, 0.0) for k in ('ecef_x', 'ecef_y', 'ecef_z')):
            lat, lon, alt = ecef_to_geodetic(row['ecef_x'], row['ecef_y'], row['ecef_z'])
        row['lat'], row['lon'], row['alt_m'] = lat, lon, alt

        if any(k in row for k in ('q0', 'q1', 'q2', 'q3')):
            row['roll'], row['pitch'], row['yaw'] = euler_from_quat(
                row.get('q0', 0.0), row.get('q1', 0.0),
                row.get('q2', 0.0), row.get('q3', 0.0))
        else:
            row['roll'] = row['pitch'] = row['yaw'] = 0.0

        accum[rid] = row
        rows.append(row)

    return rows, events


def read_bs_csv(path):
    """Parse a legacy CSV base-station log. Tolerates a truncated final row."""
    rows, events = [], []
    with open(path, newline='') as fh:
        for rec in csv.DictReader(fh):
            if (rec.get('state') or '').strip() == 'EVENT':
                try:
                    events.append(dict(time_ms=int(rec['time_ms']),
                                       t_s=float(rec['time_ms']) / 1000.0,
                                       rx_freq_mhz=float(rec.get('rx_freq_mhz') or 'nan'),
                                       event=rec.get('event') or ''))
                except (TypeError, ValueError):
                    pass
                continue
            try:
                row = dict(rec)
                row['time_ms'] = int(rec['time_ms'])
                row['t_s'] = row['time_ms'] / 1000.0
                for k in ('rssi', 'snr', 'pdop', 'voltage', 'current', 'soc'):
                    if rec.get(k) not in (None, ''):
                        row[k] = float(rec[k])
                row['seq'] = int(rec['seq'])
                row['frame'] = 'csv'
                rows.append(row)
            except (TypeError, ValueError, KeyError):
                continue    # truncated final row
    return rows, events


def read_bs_log(path):
    """Read a base-station log of either format.

    Detects by CONTENT, not by extension — a .bin that is really a CSV (or the
    reverse, after a rename) reads correctly rather than producing garbage.

    Returns (rows, events, fmt) with fmt in {'bin', 'csv'}.
    """
    with open(path, 'rb') as fh:
        head = fh.read(len(BS_LOG_MAGIC))
    if head == BS_LOG_MAGIC:
        rows, events = read_bs_bin(path)
        return rows, events, 'bin'
    rows, events = read_bs_csv(path)
    return rows, events, 'csv'


# ==========================================================================
#  CSV rendering
# ==========================================================================
# The base station used to write these columns itself.  It no longer does, so
# this is the definition of the format — and the apps carry ports of it, tested
# against the same golden.  The column set is the firmware's 39 verbatim, with
# `frame` appended: appending is how rocket_id and cam_a/servo_a were added, so
# anything keying columns by name is unaffected while the fast/slow information
# that only exists post-#850 is not thrown away.

CSV_COLUMNS = [
    'time_ms', 'state', 'num_sats', 'pdop', 'lat', 'lon', 'alt_m', 'h_acc',
    'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z',
    'pressure_alt', 'alt_rate', 'max_alt', 'max_speed',
    'voltage', 'current', 'soc', 'cam_a', 'servo_a', 'roll', 'pitch', 'yaw', 'speed',
    'launch', 'vel_apo', 'alt_apo', 'landed', 'rssi', 'snr',
    'next_ch', 'rx_freq_mhz', 'seq', 'gap', 'event', 'rocket_id', 'frame',
]

_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = 2 * _WGS84_F - _WGS84_F * _WGS84_F


def ecef_to_geodetic(x, y, z):
    """WGS84 ECEF -> (lat_deg, lon_deg, alt_m).

    Same iterative solution as TR_Coordinates::ecefToGeodetic, converging on
    latitude to 1e-10 rad (~0.1 mm).  Kept identical rather than swapped for a
    closed form so the three ports cannot drift in the last decimal.
    """
    lon = math.atan2(y, x)
    p = math.sqrt(x * x + y * y)
    lat = math.atan2(z, p * (1 - _WGS84_E2))
    alt = 0.0
    while True:
        prev = lat
        n = _WGS84_A / math.sqrt(1 - _WGS84_E2 * math.sin(lat) ** 2)
        alt = p / math.cos(lat) - n
        lat = math.atan2(z + n * _WGS84_E2 * math.sin(lat), p)
        if abs(lat - prev) <= 1e-10:
            break
    return math.degrees(lat), math.degrees(lon), alt


def euler_from_quat(qw, qx, qy, qz):
    """Port of SensorConverter::eulerFromQuat — roll is the body-Z azimuth
    (gimbal-lock-free), pitch and yaw are standard Euler."""
    z_n = 2.0 * (qx * qz + qw * qy)
    z_e = 2.0 * (qy * qz - qw * qx)
    roll = -math.degrees(math.atan2(z_e, z_n))
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(90.0, sinp)
    else:
        pitch = math.degrees(math.asin(sinp))
    yaw = math.degrees(math.atan2(2.0 * (qw * qz + qx * qy),
                                  1.0 - 2.0 * (qy * qy + qz * qz)))
    return roll, pitch, yaw


def _f(value, decimals):
    """printf("%.Nf") semantics, including the sign on a negative that rounds
    to zero.  Python's format() already rounds ties-to-even on the exact binary
    value, which is what C does."""
    if value is None:
        return ''
    if isinstance(value, float) and value != value:
        return 'nan'
    return f'{value:.{decimals}f}'


def rows_to_csv(rows, events=()):
    """Render reader output as the base-station CSV.

    Events are merged in by time so a single pass sees telemetry and events in
    arrival order — the property the firmware's padded EVENT rows provided.
    """
    out = [','.join(CSV_COLUMNS)]

    merged = [('rx', r) for r in rows] + [('ev', e) for e in events]
    merged.sort(key=lambda t: t[1].get('time_ms', 0))

    for kind, rec in merged:
        if kind == 'ev':
            cells = [''] * len(CSV_COLUMNS)
            cells[0] = str(int(rec.get('time_ms', 0)))
            cells[1] = 'EVENT'
            cells[CSV_COLUMNS.index('rx_freq_mhz')] = _f(rec.get('rx_freq_mhz'), 3)
            cells[CSV_COLUMNS.index('event')] = rec.get('event', '')
            out.append(','.join(cells))
            continue

        r = rec
        # Derived by the reader (lat/lon only where the rocket claims a fix —
        # nonzero ECEF with num_sats == 0 is a stale register read, #95).
        lat = r.get('lat', float('nan'))
        lon = r.get('lon', float('nan'))
        alt = r.get('alt_m', float('nan'))
        roll = r.get('roll', 0.0)
        pitch = r.get('pitch', 0.0)
        yaw = r.get('yaw', 0.0)

        out.append(','.join([
            str(int(r.get('time_ms', 0))),
            str(r.get('state', '')),
            str(int(r.get('num_sats', 0))),
            _f(r.get('pdop', 0.0), 1),
            _f(lat, 7), _f(lon, 7), _f(alt, 1),
            _f(r.get('h_acc', 0.0), 1),
            _f(r.get('acc_x', 0.0), 2), _f(r.get('acc_y', 0.0), 2), _f(r.get('acc_z', 0.0), 2),
            _f(r.get('gyro_x', 0.0), 1), _f(r.get('gyro_y', 0.0), 1), _f(r.get('gyro_z', 0.0), 1),
            _f(r.get('pressure_alt', 0.0), 1), _f(r.get('alt_rate', 0.0), 1),
            _f(r.get('max_alt', 0.0), 1), _f(r.get('max_speed', 0.0), 1),
            _f(r.get('voltage', 0.0), 2), _f(r.get('current', 0.0), 0), _f(r.get('soc', 0.0), 1),
            _f(r.get('cam_a', 0.0), 3), _f(r.get('servo_a', 0.0), 3),
            _f(roll, 1), _f(pitch, 1), _f(yaw, 1), _f(r.get('speed', 0.0), 1),
            str(int(r.get('launch', 0))), str(int(r.get('vel_apo', 0))),
            str(int(r.get('alt_apo', 0))), str(int(r.get('landed', 0))),
            _f(r.get('rssi', float('nan')), 0), _f(r.get('snr', float('nan')), 1),
            str(int(r.get('next_ch', 255))), _f(r.get('rx_freq_mhz', 0.0), 3),
            str(int(r.get('seq', 0))), str(int(r.get('gap', -1))),
            '',                                     # event column, empty on telemetry
            str(int(r.get('rocket_id', 0))),
            str(r.get('frame', '')),
        ]))

    return '\n'.join(out) + '\n'
