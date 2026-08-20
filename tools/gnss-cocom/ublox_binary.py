#!/usr/bin/env python3
"""UBX parsing for the rocket-computer's u-blox receiver.

Sibling of skytraq_binary.py, deliberately the same shape, because the whole
point is that gnss_nmea_monitor's FIX / BLOCKED / NO_LOCK classifier stays the
single definition of "blocked" across both receivers. The two message pairs map
onto each other exactly:

    SkyTraq 0xDF  receiver state   <->  UBX-NAV-PVT  (0x01 0x07)
    SkyTraq 0xE7  channel status   <->  UBX-NAV-SAT  (0x01 0x35)

Needed because the flight computer is a u-blox SAM-M10Q, not a PX1125R, and its
firmware calls setUART1Output(COM_TYPE_UBX) -- NMEA is switched off on the wire,
so a passive tap on the receiver's TXD sees UBX and nothing else.

Two things here are easy to get wrong and both land on the measurement:

* **Speed must be the 3-D speed.** NAV-PVT carries gSpeed, which is *ground*
  speed -- horizontal only. COCOM acts on 3-D speed, and a rocket's velocity is
  almost entirely vertical, so gSpeed reads near zero through the exact part of
  the flight the limit is about. The 3-D speed is computed from velN/velE/velD.

* **A fix needs gnssFixOK, not just fixType.** u-blox reports a fixType with the
  validity flag clear when it is withholding, so reading fixType alone counts a
  withheld solution as a fix -- which is precisely the transition being measured.
"""

from __future__ import annotations

import math
import struct

SYNC = b"\xb5\x62"

CLS_NAV = 0x01
MSG_NAV_PVT = 0x07
MSG_NAV_SAT = 0x35
MSG_NAV_STATUS = 0x03

# NAV-PVT fixType. Only 2D and better count as a position; dead reckoning and
# time-only do not, the same call skytraq_binary makes at nav_state >= 2.
FIX_TYPE = {0: "NO_FIX", 1: "DR_ONLY", 2: "FIX_2D", 3: "FIX_3D",
            4: "GNSS_DR", 5: "TIME_ONLY"}

GNSS_ID = {0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 4: "IMES", 5: "QZSS",
           6: "GLO", 7: "NAVIC"}

_PVT_LEN = 92
_SAT_HDR = 8
_SAT_BLK = 12


def _checksum(data: bytes):
    """UBX 8-bit Fletcher over class, id, length and payload."""
    a = b = 0
    for byte in data:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def iter_frames(buf: bytearray):
    """Pull complete UBX frames out of buf, consuming what it yields.

    Yields (cls, msg_id, payload). Leaves a partial trailing frame in buf for
    the next call and drops bytes before a sync pair, so a stream that also
    carries NMEA or a bootloader banner is handled rather than treated as an
    error.
    """
    while True:
        i = buf.find(SYNC)
        if i < 0:
            del buf[:max(0, len(buf) - 1)]   # keep a possible split sync byte
            return
        if len(buf) < i + 6:
            del buf[:i]
            return
        cls = buf[i + 2]
        mid = buf[i + 3]
        n = int.from_bytes(buf[i + 4:i + 6], "little")
        end = i + 6 + n + 2
        if n > 4096:                          # not a real UBX length
            del buf[:i + 2]
            continue
        if len(buf) < end:
            del buf[:i]
            return
        payload = bytes(buf[i + 6:i + 6 + n])
        ck_a, ck_b = buf[end - 2], buf[end - 1]
        del buf[:end]
        body = bytes([cls, mid]) + n.to_bytes(2, "little") + payload
        if (ck_a, ck_b) == _checksum(body):
            yield cls, mid, payload


def parse_nav_pvt(payload: bytes):
    """UBX-NAV-PVT -> dict, or None if the payload is the wrong size."""
    if len(payload) < _PVT_LEN:
        return None
    (itow, year, month, day, hour, minute, sec, valid, t_acc, nano,
     fix_type, flags, flags2, num_sv, lon, lat, height, h_msl,
     h_acc, v_acc, vel_n, vel_e, vel_d, g_speed, head_mot,
     s_acc, head_acc, pdop) = struct.unpack_from(
        "<IHBBBBBBIiBBBBiiiiIIiiiiiIIH", payload, 0)

    # gnssFixOK. u-blox leaves fixType populated while clearing this bit when
    # it will not stand behind the solution, so both are required.
    gnss_fix_ok = bool(flags & 0x01)
    has_fix = gnss_fix_ok and fix_type in (2, 3, 4)

    # 3-D speed, not gSpeed: see the module docstring.
    speed_3d = math.sqrt(vel_n * vel_n + vel_e * vel_e + vel_d * vel_d) / 1000.0

    return {
        "itow_ms": itow,
        "utc": (year, month, day, hour, minute, sec),
        "utc_valid": (valid & 0x07) == 0x07,   # validDate|validTime|fullyResolved
        "fix_type": fix_type,
        "fix_type_name": FIX_TYPE.get(fix_type, f"?{fix_type}"),
        "gnss_fix_ok": gnss_fix_ok,
        "has_fix": has_fix,
        "num_sv": num_sv,
        "lat": lat * 1e-7,
        "lon": lon * 1e-7,
        "alt_m": height / 1000.0,        # above the ellipsoid, matching skytraq
        "alt_msl_m": h_msl / 1000.0,
        "h_acc_m": h_acc / 1000.0,
        "v_acc_m": v_acc / 1000.0,
        "vel": (vel_n / 1000.0, vel_e / 1000.0, vel_d / 1000.0),
        "speed_mps": speed_3d,
        "ground_speed_mps": g_speed / 1000.0,
        "s_acc_mps": s_acc / 1000.0,
        "pdop": pdop * 0.01,
        "hdop": None,                    # NAV-PVT carries pDOP only
    }


def parse_nav_sat(payload: bytes):
    """UBX-NAV-SAT -> list of per-satellite dicts, or None if malformed."""
    if len(payload) < _SAT_HDR:
        return None
    num_svs = payload[5]
    if len(payload) < _SAT_HDR + num_svs * _SAT_BLK:
        return None

    sats = []
    for i in range(num_svs):
        off = _SAT_HDR + i * _SAT_BLK
        gnss_id, sv_id, cno, elev = struct.unpack_from("<BBBb", payload, off)
        azim, pr_res = struct.unpack_from("<hh", payload, off + 4)
        flags = struct.unpack_from("<I", payload, off + 8)[0]
        sats.append({
            "gnss": gnss_id,
            "gnss_name": GNSS_ID.get(gnss_id, f"?{gnss_id}"),
            "svid": sv_id,
            "cn0": cno,
            "elev": elev,
            "azim": azim,
            "pr_res_m": pr_res * 0.1,
            "quality": flags & 0x07,
            "used_in_fix": bool(flags & 0x08),
            "health": (flags >> 4) & 0x03,
            "diff_corr": bool(flags & 0x40),
            "ephemeris": bool(flags & 0x0800),
            "almanac": bool(flags & 0x1000),
        })
    return sats


def key(sat) -> str:
    """Stable per-satellite key across constellations.

    Same contract as skytraq_binary.key, and the same trap behind it: a receiver
    reports one entry per *signal*, so a satellite can appear more than once in
    one NAV-SAT. Callers must keep the strongest C/N0 per key rather than the
    last seen, or a satellite tracked at 40 dBHz gets recorded at whatever its
    weakest secondary signal reads.
    """
    return f"{sat['gnss_name']}:{sat['svid']}"
