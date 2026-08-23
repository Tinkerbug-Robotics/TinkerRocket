#!/usr/bin/env python3
"""SkyTraq binary message parsing for the COCOM rig (#491).

The PX1125R on these boards emits binary rather than NMEA, and on the firmware
in hand it keeps doing so no matter what the message-type command is set to --
`0x09` ACKs and changes nothing, `0x1E` NACKs, and a factory reset plus power
cycle leaves it unchanged.  Rather than keep guessing at command ids, this reads
what the receiver is actually sending.

That turns out to be *better* for the measurement, not a fallback:

  * 0xDF carries an explicit **Navigation State** enum, so "no fix" is a stated
    value rather than something inferred from empty NMEA fields.
  * 0xDF carries ECEF velocity, so the speed at a transition -- the number the
    velocity-ramp test exists to find -- comes straight from the receiver
    instead of being reconstructed from position differences.
  * 0xE7 carries per-satellite C/N0 *and* a "used in fix" bit, which is exactly
    the tracking-vs-position split the COCOM verdict turns on.

Layouts are from SkyTraq Binary Protocol AN0039 v1.4.42, cross-checked against
bytes captured from the bench receiver:

  framing   A0 A1 | len(2, BE) | payload | XOR(payload) | 0D 0A
  numbers   big-endian throughout; DPFP = float64, SPFP = float32

  RCV_STATE (0xDF), payload 81 bytes
    [0] id  [1] IOD  [2] NavState  [3:5] WN  [5:13] TOW
    [13:21] ECEF X  [21:29] ECEF Y  [29:37] ECEF Z          (metres, f64)
    [37:41] VEL X   [41:45] VEL Y   [45:49] VEL Z           (m/s,    f32)
    [49:57] clock bias  [57:61] clock drift
    [61:65] GDOP [65:69] PDOP [69:73] HDOP [73:77] VDOP [77:81] TDOP

  GNSS_SV_CH_STATUS (0xE7), payload 4 + N*7 bytes
    [0] id  [1] version  [2] IOD  [3] NSVS
    then N 7-byte records:
      [0] channel  [1] type: bits0-3 GNSS, bits4-7 signal  [2] SVID
      [3] SV status  [4] URA/FT  [5] **C/N0, dBHz**  [6] channel status
      channel status bit 4 = used in normal fix, bit 5 = used in differential

NavState is the COCOM-relevant field: 0 = NO_FIX, 1 = FIX_PREDICTION,
2 = FIX_2D, 3 = FIX_3D, 4 = FIX_DIFFERENTIAL.
"""

import math
import struct

NAV_STATE = {
    0: "NO_FIX",
    1: "FIX_PREDICTION",
    2: "FIX_2D",
    3: "FIX_3D",
    4: "FIX_DIFFERENTIAL",
}

GNSS_TYPE = {0: "GPS", 1: "SBAS", 2: "GLO", 3: "GAL", 4: "QZSS", 5: "BDS", 6: "IRNSS"}

MSG_RCV_STATE = 0xDF
MSG_SV_CH_STATUS = 0xE7
MSG_SV_ELV_AZM = 0xE8

# WGS84
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


def iter_frames(buf: bytearray):
    """Pull complete A0A1 frames out of buf, consuming what it yields.

    Leaves any partial trailing frame in buf for the next call, and drops bytes
    before a preamble -- the stream is shared with the bridge's own '#' banner
    lines, so leading non-frame data is normal rather than an error.
    """
    while True:
        i = buf.find(b"\xa0\xa1")
        if i < 0:
            # Keep only a possible split preamble.
            del buf[:max(0, len(buf) - 1)]
            return
        if len(buf) < i + 4:
            del buf[:i]
            return
        n = int.from_bytes(buf[i + 2:i + 4], "big")
        end = i + 4 + n + 3
        if len(buf) < end:
            del buf[:i]
            return
        payload = bytes(buf[i + 4:i + 4 + n])
        checksum = buf[i + 4 + n]
        del buf[:end]
        if payload and checksum == _xor(payload):
            yield payload


def _xor(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c


def ecef_to_geodetic(x, y, z):
    """WGS84 ECEF -> (lat_deg, lon_deg, alt_m), Bowring's method.

    Returns None for the origin, which is what the receiver reports before it
    has a fix -- treating (0,0,0) as a position on the Gulf of Guinea would put
    a fake fix in the log at exactly the moments the test cares about.
    """
    if x == 0.0 and y == 0.0 and z == 0.0:
        return None
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    if p == 0.0:
        lat = math.copysign(math.pi / 2, z)
        return math.degrees(lat), math.degrees(lon), abs(z) - _A * (1 - _F)
    b = _A * (1.0 - _F)
    theta = math.atan2(z * _A, p * b)
    ep2 = (_A * _A - b * b) / (b * b)
    lat = math.atan2(z + ep2 * b * math.sin(theta) ** 3,
                     p - _E2 * _A * math.cos(theta) ** 3)
    n = _A / math.sqrt(1.0 - _E2 * math.sin(lat) ** 2)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt


def parse_rcv_state(payload: bytes):
    """0xDF -> dict, or None if the frame is the wrong size."""
    if len(payload) < 81 or payload[0] != MSG_RCV_STATE:
        return None
    nav_state = payload[2]
    x, y, z = struct.unpack_from(">ddd", payload, 13)
    vx, vy, vz = struct.unpack_from(">fff", payload, 37)
    gdop, pdop, hdop, vdop, tdop = struct.unpack_from(">fffff", payload, 61)
    geo = ecef_to_geodetic(x, y, z)
    return {
        "iod": payload[1],
        "nav_state": nav_state,
        "nav_state_name": NAV_STATE.get(nav_state, f"?{nav_state}"),
        "has_fix": nav_state >= 2,      # 2D or better; prediction is not a fix
        "week": struct.unpack_from(">H", payload, 3)[0],
        "tow": struct.unpack_from(">d", payload, 5)[0],
        "ecef": (x, y, z),
        "lat": geo[0] if geo else None,
        "lon": geo[1] if geo else None,
        "alt_m": geo[2] if geo else None,
        "vel": (vx, vy, vz),
        "speed_mps": math.sqrt(vx * vx + vy * vy + vz * vz),
        "gdop": gdop, "pdop": pdop, "hdop": hdop, "vdop": vdop, "tdop": tdop,
    }


def parse_sv_ch_status(payload: bytes):
    """0xE7 -> list of per-satellite dicts, or None if malformed."""
    if len(payload) < 4 or payload[0] != MSG_SV_CH_STATUS:
        return None
    nsvs = payload[3]
    if len(payload) < 4 + nsvs * 7:
        return None
    sats = []
    for i in range(nsvs):
        r = payload[4 + i * 7:11 + i * 7]
        cn0 = struct.unpack_from(">b", r, 5)[0]     # SINT8
        chan_status = r[6]
        gnss = r[1] & 0x0F
        sats.append({
            "channel": r[0],
            "gnss": gnss,
            "gnss_name": GNSS_TYPE.get(gnss, f"?{gnss}"),
            "signal": (r[1] >> 4) & 0x0F,
            "svid": r[2],
            "almanac": bool(r[3] & 0x01),
            "ephemeris": bool(r[3] & 0x02),
            "healthy": bool(r[3] & 0x04),
            "cn0": cn0,
            "used_in_fix": bool(chan_status & 0x10),
            "used_in_diff": bool(chan_status & 0x20),
        })
    return sats


def key(sat) -> str:
    """Stable per-satellite key across constellations (PRNs collide otherwise)."""
    return f"{sat['gnss_name']}:{sat['svid']}"
