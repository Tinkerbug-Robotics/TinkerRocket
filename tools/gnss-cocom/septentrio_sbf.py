#!/usr/bin/env python3
"""SBF (Septentrio Binary Format) parsing for the mosaic-G5.

Sibling of skytraq_binary.py and ublox_binary.py. Layouts are from the mosaic-G5
Firmware v1.1.0 Reference Guide (section 4) and were checked on live blocks from a
mosaic-G5 P3 on firmware 1.0.0: every block CRC-valid, PVTGeodetic rev 2,
ReceiverStatus rev 1, MeasEpoch rev 1.

Three things here are easy to get wrong:

* **C/N0 has a +10 dB offset** on every signal except GPS L1P and L2P (signal
  numbers 1 and 2): C/N0 = CN0 * 0.25 + 10.
* **Signal numbers above 31** do not fit the 5-bit SigIdxLo field. SigIdxLo = 31
  means "read ObsInfo bits 3-7 and add 32" (QZSS L1C/L1S, BeiDou B2b, NavIC L1,
  QZSS L1CB, QZSS L5S). For GLONASS (signals 8-11) the same bits carry the
  frequency number + 8 instead, which the carrier wavelength needs.
* **GLONASS satellites tracked before their slot is known** use SVID 62, so
  several of them can share one SVID in a single epoch.

The CRC is CRC-CCITT (polynomial 0x1021, initial value 0) over everything after
the CRC field, which is exactly binascii.crc_hqx.
"""

from __future__ import annotations

import binascii
import struct

SYNC = b"$@"
C = 299792458.0

# Block numbers used by the mosaic tools.
MEAS_EPOCH = 4027
MEAS_EXTRA = 4000
PVT_GEODETIC = 4007
POS_COV_GEODETIC = 5906
VEL_COV_GEODETIC = 5908
DOP = 4001
RECEIVER_STATUS = 4014
QUALITY_IND = 4082
RF_STATUS = 4092
SAT_VISIBILITY = 4012
CHANNEL_STATUS = 4013
RECEIVER_TIME = 5914
RECEIVER_SETUP = 5902
COMMANDS = 4015
RX_MESSAGE = 4103

# Do-not-use values.
DNU_F8 = -2e10
DNU_F4 = -2e10

# Signal number -> (name, carrier MHz). GLONASS FDMA carriers are filled in per
# frequency number by carrier_hz().
SIGNALS = {
    0: ("GPS L1CA", 1575.42), 1: ("GPS L1P", 1575.42), 2: ("GPS L2P", 1227.60),
    3: ("GPS L2C", 1227.60), 4: ("GPS L5", 1176.45), 5: ("GPS L1C", 1575.42),
    6: ("QZS L1CA", 1575.42), 7: ("QZS L2C", 1227.60),
    8: ("GLO L1CA", None), 9: ("GLO L1P", None), 10: ("GLO L2P", None),
    11: ("GLO L2CA", None), 12: ("GLO L3", 1202.025),
    13: ("BDS B1C", 1575.42), 14: ("BDS B2a", 1176.45), 15: ("NavIC L5", 1176.45),
    17: ("GAL E1", 1575.42), 19: ("GAL E6", 1278.75), 20: ("GAL E5a", 1176.45),
    21: ("GAL E5b", 1207.14), 22: ("GAL E5", 1191.795), 23: ("MSS L-band", None),
    24: ("SBAS L1", 1575.42), 25: ("SBAS L5", 1176.45), 26: ("QZS L5", 1176.45),
    27: ("QZS L6", 1278.75), 28: ("BDS B1I", 1561.098), 29: ("BDS B2I", 1207.14),
    30: ("BDS B3I", 1268.52), 32: ("QZS L1C", 1575.42), 33: ("QZS L1S", 1575.42),
    34: ("BDS B2b", 1207.14), 37: ("NavIC L1", 1575.42), 38: ("QZS L1CB", 1575.42),
    39: ("QZS L5S", 1176.45),
}

# ReceiverStatus AGCState frontend codes.
FRONTENDS = {0: "L1/E1", 1: "GLO L1", 2: "E6", 3: "GPS L2", 4: "GLO L2", 5: "L5/E5a/B2a",
             6: "E5b/B2b", 7: "E5", 8: "L1 combined", 9: "L2 combined", 10: "L-band",
             11: "B1", 12: "B3", 13: "S-band", 14: "B3/E6"}

# PVTGeodetic Mode bits 0-3 and Error.
PVT_MODES = {0: "none", 1: "standalone", 2: "DGNSS", 3: "fixed", 4: "RTK fixed",
             5: "RTK float", 6: "SBAS", 7: "MB RTK fixed", 8: "MB RTK float", 10: "PPP"}
PVT_ERRORS = {0: "ok", 1: "not enough measurements", 2: "not enough ephemerides",
              3: "DOP too large", 4: "residuals too large", 5: "no convergence",
              6: "not enough measurements after outlier rejection",
              7: "position output prohibited due to export laws",
              8: "not enough differential corrections", 9: "base coordinates unavailable",
              10: "ambiguities not fixed"}


def crc_ok(block: bytes) -> bool:
    return binascii.crc_hqx(block[4:], 0) == struct.unpack_from("<H", block, 2)[0]


def block_id(block: bytes) -> tuple[int, int]:
    """(block number, revision)."""
    v = struct.unpack_from("<H", block, 4)[0]
    return v & 0x1FFF, v >> 13


def tow_wnc(block: bytes) -> tuple[int | None, int | None]:
    tow, wnc = struct.unpack_from("<IH", block, 8)
    return (None if tow == 0xFFFFFFFF else tow), (None if wnc == 0xFFFF else wnc)


class Splitter:
    """Split a receiver byte stream into CRC-valid SBF blocks and ASCII text.

    feed() returns ("S", block) and ("T", text) items in arrival order. A "$@"
    whose block fails the CRC is treated as text, one byte at a time, so a
    corrupt block costs only itself.
    """

    def __init__(self):
        self.buf = b""
        self.bad = 0                    # "$@" starts that failed length/CRC checks

    def feed(self, data: bytes) -> list[tuple[str, bytes]]:
        self.buf += data
        out = []
        while self.buf:
            i = self.buf.find(SYNC)
            if i < 0:
                # keep a trailing "$" in case the next read starts with "@"
                keep = 1 if self.buf.endswith(b"$") else 0
                text = self.buf[:len(self.buf) - keep]
                if text:
                    out.append(("T", text))
                self.buf = self.buf[len(self.buf) - keep:]
                break
            if i:
                out.append(("T", self.buf[:i]))
                self.buf = self.buf[i:]
            if len(self.buf) < 8:
                break
            length = struct.unpack_from("<H", self.buf, 6)[0]
            if length < 8 or length % 4 or length > 65535:
                self.bad += 1
                out.append(("T", self.buf[:1]))
                self.buf = self.buf[1:]
                continue
            if len(self.buf) < length:
                break
            blk = self.buf[:length]
            if crc_ok(blk):
                out.append(("S", blk))
                self.buf = self.buf[length:]
            else:
                self.bad += 1
                out.append(("T", self.buf[:1]))
                self.buf = self.buf[1:]
        return out


def svid_name(sv: int) -> str:
    """RINEX-style satellite name for an SBF SVID."""
    if 1 <= sv <= 37:
        return f"G{sv:02d}"
    if 38 <= sv <= 61:
        return f"R{sv - 37:02d}"
    if sv == 62:
        return "R??"
    if 63 <= sv <= 68:
        return f"R{sv - 38:02d}"
    if 71 <= sv <= 106:
        return f"E{sv - 70:02d}"
    if 107 <= sv <= 119:
        return f"L{sv - 106:02d}"
    if 120 <= sv <= 140:
        return f"S{sv:03d}"
    if 141 <= sv <= 180:
        return f"C{sv - 140:02d}"
    if 181 <= sv <= 190:
        return f"J{sv - 180:02d}"
    if 191 <= sv <= 197:
        return f"I{sv - 190:02d}"
    if 198 <= sv <= 215:
        return f"S{sv - 57:03d}"
    if 216 <= sv <= 222:
        return f"I{sv - 208:02d}"
    if 223 <= sv <= 245:
        return f"C{sv - 182:02d}"
    return f"?{sv}"


def signal_name(sig: int) -> str:
    return SIGNALS.get(sig, (f"sig{sig}", None))[0]


def carrier_hz(sig: int, glo_fn: int | None = None) -> float | None:
    if sig in (8, 9):
        return None if glo_fn is None else (1602.0 + glo_fn * 0.5625) * 1e6
    if sig in (10, 11):
        return None if glo_fn is None else (1246.0 + glo_fn * 0.4375) * 1e6
    mhz = SIGNALS.get(sig, (None, None))[1]
    return None if mhz is None else mhz * 1e6


def _sig_number(type_byte: int, obs_info: int) -> tuple[int, int | None]:
    """(signal number, GLONASS frequency number or None)."""
    lo = type_byte & 0x1F
    if lo == 31:
        return (obs_info >> 3) + 32, None
    if lo in (8, 9, 10, 11):
        return lo, (obs_info >> 3) - 8
    return lo, None


def _cn0(raw: int, sig: int) -> float | None:
    if raw == 255:
        return None
    return raw * 0.25 + (0.0 if sig in (1, 2) else 10.0)


def meas_epoch(block: bytes) -> dict:
    """Decode MeasEpoch 4027: one dict per signal, with pseudorange (m), Doppler (Hz),
    carrier phase (cycles), C/N0 (dB-Hz) and lock time (s). Missing values are None."""
    tow, wnc = tow_wnc(block)
    n1, sb1, sb2, flags, clk_jumps = struct.unpack_from("<BBBBB", block, 14)
    meas = []
    p = 20
    for _ in range(n1):
        (ch, typ, sv, misc, code_lsb, dop, car_lsb, car_msb, cn0, lock, obs,
         n2) = struct.unpack_from("<BBBBIiHbBHBB", block, p)
        sig, fn = _sig_number(typ, obs)
        f1 = carrier_hz(sig, fn)
        code_msb = misc & 0x0F
        pr1 = None if (code_msb == 0 and code_lsb == 0) else (code_msb * 4294967296 + code_lsb) * 0.001
        d1 = None if dop == -2147483648 else dop * 1e-4
        l1 = None
        if pr1 is not None and f1 and not (car_msb == -128 and car_lsb == 0):
            l1 = pr1 * f1 / C + (car_msb * 65536 + car_lsb) * 0.001
        meas.append({"ch": ch, "sv": svid_name(sv), "svid": sv, "sig": sig, "signal": signal_name(sig),
                     "glo_fn": fn, "pr": pr1, "doppler": d1, "carrier": l1, "cn0": _cn0(cn0, sig),
                     "lock": None if lock == 65535 else lock, "smoothed": bool(obs & 1),
                     "half_cycle": bool(obs & 4), "type1": True})
        q = p + sb1
        for _ in range(n2):
            typ2, lock2, cn02, off_msb, car_msb2, obs2, code_off_lsb, car_lsb2, dop_off_lsb = \
                struct.unpack_from("<BBBBbBHHH", block, q)
            sig2, fn2 = _sig_number(typ2, obs2)
            f2 = carrier_hz(sig2, fn2 if fn2 is not None else fn)
            code_off_msb = off_msb & 0x07
            code_off_msb -= 8 if code_off_msb >= 4 else 0
            dop_off_msb = off_msb >> 3
            dop_off_msb -= 32 if dop_off_msb >= 16 else 0
            pr2 = None
            if pr1 is not None and code_off_msb != -4:
                pr2 = pr1 + (code_off_msb * 65536 + code_off_lsb) * 0.001
            d2 = None
            if d1 is not None and f1 and f2 and dop_off_msb != -16:
                d2 = d1 * (f2 / f1) + (dop_off_msb * 65536 + dop_off_lsb) * 1e-4
            l2 = None
            if pr2 is not None and f2 and not (car_msb2 == -128 and car_lsb2 == 0):
                l2 = pr2 * f2 / C + (car_msb2 * 65536 + car_lsb2) * 0.001
            meas.append({"ch": ch, "sv": svid_name(sv), "svid": sv, "sig": sig2,
                         "signal": signal_name(sig2), "glo_fn": fn2 if fn2 is not None else fn,
                         "pr": pr2, "doppler": d2, "carrier": l2, "cn0": _cn0(cn02, sig2),
                         "lock": None if lock2 == 255 else lock2, "smoothed": bool(obs2 & 1),
                         "half_cycle": bool(obs2 & 4), "type1": False})
            q += sb2
        p = q
    return {"tow": tow, "wnc": wnc, "flags": flags, "scrambled": bool(flags & 0x80),
            "high_dynamics": bool(flags & 0x20), "clk_jumps": clk_jumps, "meas": meas}


def pvt_geodetic(block: bytes) -> dict:
    tow, wnc = tow_wnc(block)
    mode, err = block[14], block[15]
    lat, lon, h = struct.unpack_from("<ddd", block, 16)
    und, vn, ve, vu, cog = struct.unpack_from("<fffff", block, 40)
    clk_bias, = struct.unpack_from("<d", block, 60)
    clk_drift, = struct.unpack_from("<f", block, 68)
    nrsv = block[74]
    hacc, vacc = struct.unpack_from("<HH", block, 90)

    def f(x):
        return None if x == DNU_F8 else x
    return {"tow": tow, "wnc": wnc, "mode": mode & 0x0F, "mode_name": PVT_MODES.get(mode & 0x0F, str(mode & 0x0F)),
            "two_d": bool(mode & 0x40), "error": err, "error_name": PVT_ERRORS.get(err, str(err)),
            "lat": f(lat), "lon": f(lon), "h": f(h), "undulation": f(und), "vn": f(vn), "ve": f(ve),
            "vu": f(vu), "clk_bias_ms": f(clk_bias), "clk_drift_ppm": f(clk_drift),
            "nrsv": None if nrsv == 255 else nrsv,
            "hacc": None if hacc == 65535 else hacc / 100, "vacc": None if vacc == 65535 else vacc / 100}


def receiver_status(block: bytes) -> dict:
    tow, wnc = tow_wnc(block)
    cpu, ext_err, uptime, rx_state, rx_error, n, sbl, cmd_count, temp = \
        struct.unpack_from("<BBIIIBBBB", block, 14)
    agc = {}
    for k in range(n):
        fe, gain, var, blank = struct.unpack_from("<BbBB", block, 32 + k * sbl)
        agc[FRONTENDS.get(fe & 0x1F, str(fe & 0x1F))] = None if gain == -128 else gain
    return {"tow": tow, "wnc": wnc, "cpu": cpu, "ext_error": ext_err, "uptime": uptime,
            "rx_state": rx_state, "rx_error": rx_error, "active_antenna": bool(rx_state & 2),
            "cmd_count": cmd_count, "temp_c": None if temp == 255 else temp - 100, "agc_db": agc}


def sat_visibility(block: bytes) -> dict:
    """{satellite name: (elevation deg, azimuth deg)}."""
    n, sbl = block[14], block[15]
    out = {}
    for k in range(n):
        sv, fn, az, el, rise_set, info = struct.unpack_from("<BBHhBB", block, 16 + k * sbl)
        if sv and az != 65535 and el != -32768:
            out[svid_name(sv)] = (el / 100, az / 100)
    return out


# ChannelStatus 2-bit field positions per constellation (guide 4.2, ChannelStatus).
CHANNEL_FIELDS = {
    "G": ["L1CA", "L1P", "L2P", "L2C", "L5", "L1C"],
    "R": ["L1CA", "L1P", "L2P", "L2CA", "L3"],
    "E": [None, "E1", None, "E6", "E5a", "E5b", "E5"],
    "S": ["L1", "L5"],
    "C": ["B1I", "B2I", "B3I", "B1C", "B2a", "B2b"],
    "J": ["L1CA", "L2C", "L5", "L6", "L1C", "L1S", "L1CB", "L5S"],
    "I": ["L5", "L1"],
}
TRACKING = {0: "idle", 1: "search", 2: "sync", 3: "tracking"}
PVT_USE = {0: "not used", 1: "waiting for ephemeris", 2: "used", 3: "rejected"}


def channel_status(block: bytes) -> list[dict]:
    """Decode ChannelStatus 4013: per satellite, {signal: (health, tracking, pvt)} with the 2-bit codes."""
    n, sb1, sb2 = block[14], block[15], block[16]
    out = []
    p = 20
    for _ in range(n):
        sv, fn, sv_full, az_rs, health, el, n2, ch = struct.unpack_from("<BBHHHbBB", block, p)
        name = svid_name(sv if sv else sv_full)
        fields = CHANNEL_FIELDS.get(name[0], [])
        q = p + sb1
        for _ in range(n2):
            ant, _r, track, pvt, info = struct.unpack_from("<BBHHH", block, q)
            if ant == 0:
                sigs = {}
                for k, sig in enumerate(fields):
                    h, tr, pv = (health >> 2 * k) & 3, (track >> 2 * k) & 3, (pvt >> 2 * k) & 3
                    if sig and (h or tr or pv):
                        sigs[sig] = (h, tr, pv)
                out.append({"sv": name, "el": None if el == -128 else el, "az": az_rs & 0x1FF,
                            "ch": ch, "signals": sigs})
            q += sb2
        p = q
    return out
