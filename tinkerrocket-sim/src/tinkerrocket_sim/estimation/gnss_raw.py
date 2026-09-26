"""Raw GNSS front end: receiver bytes -> corrected per-satellite measurements.

Turns a receiver's raw output into the ``RawMeas`` list ``TcEkf.update_gnss_raw``
takes. GPS L1 C/A, Galileo E1 and BeiDou B1I (MEO/IGSO; the GEOs are not in view
from the Americas and their D2 message is not decoded).

  readers   SkyTraq binary (0xE5 extended raw measurements + 0xE0 GPS
            subframes, as the PX1125R sends them) and u-blox UBX
            (RXM-RAWX + RXM-SFRBX, as the NEO-M8T/ZED-F9P send them), from the
            COCOM rig's capture format ('<t> B|U <hex>' lines)
  ephemeris decoded here from the navigation data the receiver forwards:
            GPS LNAV (0xE0 / RXM-SFRBX), Galileo I/NAV word types 1-5
            (SkyTraq 0xE6 page pairs, CRC-24Q checked), BeiDou D1 subframes
            1-3 (SkyTraq 0xE2); layouts as RTKLIB rcvraw.c
  corrections satellite clock + relativity + TGD, broadcast (Klobuchar)
            ionosphere, optional Saastamoinen-style troposphere; satellite
            position at transmit time; Doppler -> range rate

Byte layouts follow RTKLIB (skytraq.c, ublox.c, rcvraw.c). Not flight code.
"""
from __future__ import annotations

import math
import struct

import numpy as np

from .tc_ekf import RawMeas, C_LIGHT, ecef2lla, t_e2ned

MU = 3.986005e14
OMGE = 7.2921151467e-5
PI_SC = 3.1415926535898
L1_WAVELENGTH = C_LIGHT / 1575.42e6
# per system: gravitational constant, earth rotation rate (each ICD's own
# value, used in the orbit), carrier frequency of the signal we use
SYS = {
    "G": dict(mu=3.986005e14, omge=7.2921151467e-5, freq=1575.42e6),     # L1 C/A
    "E": dict(mu=3.986004418e14, omge=7.2921151467e-5, freq=1575.42e6),  # E1
    "C": dict(mu=3.986004418e14, omge=7.292115e-5, freq=1561.098e6),     # B1I
}
SKY_GNSS = {0: "G", 3: "E", 5: "C"}      # SkyTraq 0xE5 GNSS type
BDT_GPST = 14.0                          # BDT = GPST - 14 s


# ------------------------------------------------------------------ bits
def getbitu(b, pos, n):
    v = 0
    for i in range(pos, pos + n):
        v = (v << 1) | ((b[i >> 3] >> (7 - (i & 7))) & 1)
    return v


def getbits(b, pos, n):
    v = getbitu(b, pos, n)
    return v - (1 << n) if v & (1 << (n - 1)) else v


def getbitu2(b, p1, l1, p2, l2):
    return (getbitu(b, p1, l1) << l2) + getbitu(b, p2, l2)


def getbits2(b, p1, l1, p2, l2):
    v = getbitu2(b, p1, l1, p2, l2)
    return v - (1 << (l1 + l2)) if getbitu(b, p1, 1) else v


def setbitu(b, pos, n, val):
    for i in range(n):
        bit = (val >> (n - 1 - i)) & 1
        k = pos + i
        if bit:
            b[k >> 3] |= 0x80 >> (k & 7)
        else:
            b[k >> 3] &= ~(0x80 >> (k & 7)) & 0xFF


def crc24q(data):
    crc = 0
    for x in data:
        crc ^= x << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


# ------------------------------------------------------------------ ephemeris
def decode_eph(sf):
    """Subframes 1-3 (30 bytes each: 10 words x 24 data bits) -> ephemeris."""
    b1, b2, b3 = sf[1], sf[2], sf[3]
    e = {}
    i = 24 + 19; id1 = getbitu(b1, i, 3); i += 5
    e["week"] = getbitu(b1, i, 10); i += 12; e["sva"] = getbitu(b1, i, 4); i += 4
    e["svh"] = getbitu(b1, i, 6); i += 6; iodc0 = getbitu(b1, i, 2); i += 2 + 1 + 87
    tgd = getbits(b1, i, 8); i += 8; iodc1 = getbitu(b1, i, 8); i += 8
    e["toc"] = getbitu(b1, i, 16) * 16.0; i += 16
    e["f2"] = getbits(b1, i, 8) * 2 ** -55; i += 8
    e["f1"] = getbits(b1, i, 16) * 2 ** -43; i += 16
    e["f0"] = getbits(b1, i, 22) * 2 ** -31
    i = 24 + 19; id2 = getbitu(b2, i, 3); i += 5
    e["iode"] = getbitu(b2, i, 8); i += 8
    e["crs"] = getbits(b2, i, 16) * 2 ** -5; i += 16
    e["deln"] = getbits(b2, i, 16) * 2 ** -43 * PI_SC; i += 16
    e["M0"] = getbits(b2, i, 32) * 2 ** -31 * PI_SC; i += 32
    e["cuc"] = getbits(b2, i, 16) * 2 ** -29; i += 16
    e["e"] = getbitu(b2, i, 32) * 2 ** -33; i += 32
    e["cus"] = getbits(b2, i, 16) * 2 ** -29; i += 16
    sqrtA = getbitu(b2, i, 32) * 2 ** -19; i += 32
    e["toe"] = getbitu(b2, i, 16) * 16.0
    i = 24 + 19; id3 = getbitu(b3, i, 3); i += 5
    e["cic"] = getbits(b3, i, 16) * 2 ** -29; i += 16
    e["OMG0"] = getbits(b3, i, 32) * 2 ** -31 * PI_SC; i += 32
    e["cis"] = getbits(b3, i, 16) * 2 ** -29; i += 16
    e["i0"] = getbits(b3, i, 32) * 2 ** -31 * PI_SC; i += 32
    e["crc"] = getbits(b3, i, 16) * 2 ** -5; i += 16
    e["omg"] = getbits(b3, i, 32) * 2 ** -31 * PI_SC; i += 32
    e["OMGd"] = getbits(b3, i, 24) * 2 ** -43 * PI_SC; i += 24
    iode3 = getbitu(b3, i, 8); i += 8
    e["idot"] = getbits(b3, i, 14) * 2 ** -43 * PI_SC
    e["A"] = sqrtA * sqrtA
    e["iodc"] = (iodc0 << 8) + iodc1
    e["tgd"] = 0.0 if tgd == -128 else tgd * 2 ** -31
    if (id1, id2, id3) != (1, 2, 3) or iode3 != e["iode"] or e["iode"] != (e["iodc"] & 0xFF):
        return None
    e["sys"], e["tgd_use"] = "G", e["tgd"]
    return e


def decode_gal_inav(w):
    """Word types 1-5 (128 bits each at offset 128*type) -> ephemeris.
    RTKLIB decode_gal_inav_eph. GST shares GPS's time of week."""
    e = {}
    i = 128; t1 = getbitu(w, i, 6); i += 6; iod1 = getbitu(w, i, 10); i += 10
    e["toe"] = getbitu(w, i, 14) * 60.0; i += 14
    e["M0"] = getbits(w, i, 32) * 2 ** -31 * PI_SC; i += 32
    e["e"] = getbitu(w, i, 32) * 2 ** -33; i += 32
    sqrtA = getbitu(w, i, 32) * 2 ** -19
    i = 256; t2 = getbitu(w, i, 6); i += 6; iod2 = getbitu(w, i, 10); i += 10
    e["OMG0"] = getbits(w, i, 32) * 2 ** -31 * PI_SC; i += 32
    e["i0"] = getbits(w, i, 32) * 2 ** -31 * PI_SC; i += 32
    e["omg"] = getbits(w, i, 32) * 2 ** -31 * PI_SC; i += 32
    e["idot"] = getbits(w, i, 14) * 2 ** -43 * PI_SC
    i = 384; t3 = getbitu(w, i, 6); i += 6; iod3 = getbitu(w, i, 10); i += 10
    e["OMGd"] = getbits(w, i, 24) * 2 ** -43 * PI_SC; i += 24
    e["deln"] = getbits(w, i, 16) * 2 ** -43 * PI_SC; i += 16
    e["cuc"] = getbits(w, i, 16) * 2 ** -29; i += 16
    e["cus"] = getbits(w, i, 16) * 2 ** -29; i += 16
    e["crc"] = getbits(w, i, 16) * 2 ** -5; i += 16
    e["crs"] = getbits(w, i, 16) * 2 ** -5; i += 16
    e["sva"] = getbitu(w, i, 8)
    i = 512; t4 = getbitu(w, i, 6); i += 6; iod4 = getbitu(w, i, 10); i += 10
    svid = getbitu(w, i, 6); i += 6
    e["cic"] = getbits(w, i, 16) * 2 ** -29; i += 16
    e["cis"] = getbits(w, i, 16) * 2 ** -29; i += 16
    e["toc"] = getbitu(w, i, 14) * 60.0; i += 14
    e["f0"] = getbits(w, i, 31) * 2 ** -34; i += 31
    e["f1"] = getbits(w, i, 21) * 2 ** -46; i += 21
    e["f2"] = getbits(w, i, 6) * 2 ** -59
    i = 640; t5 = getbitu(w, i, 6); i += 6 + 11 + 11 + 14 + 5
    bgd_e5a = getbits(w, i, 10) * 2 ** -32; i += 10
    bgd_e5b = getbits(w, i, 10) * 2 ** -32; i += 10
    e5b_hs = getbitu(w, i, 2); i += 2
    e1b_hs = getbitu(w, i, 2); i += 2
    e5b_dvs = getbitu(w, i, 1); i += 1
    e1b_dvs = getbitu(w, i, 1)
    if (t1, t2, t3, t4, t5) != (1, 2, 3, 4, 5) or len({iod1, iod2, iod3, iod4}) != 1:
        return None
    e["A"] = sqrtA * sqrtA
    e["iode"] = iod1
    e["svid"] = svid
    e["svh"] = (e1b_hs << 1) | e1b_dvs          # E1-B signal health only (we use E1)
    e["sys"], e["tgd_use"] = "E", bgd_e5b      # I/NAV clock: BGD(E1,E5b) for E1
    return e


def decode_bds_d1(sf):
    """D1 subframes 1-3 (300 bits each, 38-byte stride) -> ephemeris.
    RTKLIB decode_bds_d1_eph. Times are BDT; converted to GPS TOW (+14 s)."""
    e = {}
    i = 8 * 38 * 0
    frn1 = getbitu(sf, i + 15, 3); sow1 = getbitu2(sf, i + 18, 8, i + 30, 12)
    e["svh"] = getbitu(sf, i + 42, 1)
    e["sva"] = getbitu(sf, i + 48, 4)
    toc = getbitu2(sf, i + 73, 9, i + 90, 8) * 8.0
    tgd1 = getbits(sf, i + 98, 10) * 0.1e-9
    e["f2"] = getbits(sf, i + 214, 11) * 2 ** -66
    e["f0"] = getbits2(sf, i + 225, 7, i + 240, 17) * 2 ** -33
    e["f1"] = getbits2(sf, i + 257, 5, i + 270, 17) * 2 ** -50
    e["iode"] = getbitu(sf, i + 287, 5)
    i = 8 * 38 * 1
    frn2 = getbitu(sf, i + 15, 3); sow2 = getbitu2(sf, i + 18, 8, i + 30, 12)
    e["deln"] = getbits2(sf, i + 42, 10, i + 60, 6) * 2 ** -43 * PI_SC
    e["cuc"] = getbits2(sf, i + 66, 16, i + 90, 2) * 2 ** -31
    e["M0"] = getbits2(sf, i + 92, 20, i + 120, 12) * 2 ** -31 * PI_SC
    e["e"] = getbitu2(sf, i + 132, 10, i + 150, 22) * 2 ** -33
    e["cus"] = getbits(sf, i + 180, 18) * 2 ** -31
    e["crc"] = getbits2(sf, i + 198, 4, i + 210, 14) * 2 ** -6
    e["crs"] = getbits2(sf, i + 224, 8, i + 240, 10) * 2 ** -6
    sqrtA = getbitu2(sf, i + 250, 12, i + 270, 20) * 2 ** -19
    toe1 = getbitu(sf, i + 290, 2)
    i = 8 * 38 * 2
    frn3 = getbitu(sf, i + 15, 3); sow3 = getbitu2(sf, i + 18, 8, i + 30, 12)
    toe2 = getbitu2(sf, i + 42, 10, i + 60, 5)
    e["i0"] = getbits2(sf, i + 65, 17, i + 90, 15) * 2 ** -31 * PI_SC
    e["cic"] = getbits2(sf, i + 105, 7, i + 120, 11) * 2 ** -31
    e["OMGd"] = getbits2(sf, i + 131, 11, i + 150, 13) * 2 ** -43 * PI_SC
    e["cis"] = getbits2(sf, i + 163, 9, i + 180, 9) * 2 ** -31
    e["idot"] = getbits2(sf, i + 189, 13, i + 210, 1) * 2 ** -43 * PI_SC
    e["OMG0"] = getbits2(sf, i + 211, 21, i + 240, 11) * 2 ** -31 * PI_SC
    e["omg"] = getbits2(sf, i + 251, 11, i + 270, 21) * 2 ** -31 * PI_SC
    toe = ((toe1 << 15) + toe2) * 8.0
    if (frn1, frn2, frn3) != (1, 2, 3) or sow2 != sow1 + 6 or sow3 != sow2 + 6 or toc != toe:
        return None
    e["A"] = sqrtA * sqrtA
    e["toe"] = (toe + BDT_GPST) % 604800.0
    e["toc"] = (toc + BDT_GPST) % 604800.0
    e["sys"], e["tgd_use"] = "C", tgd1        # B1I: TGD1
    return e


def dt_week(t, ref):
    d = t - ref
    return d - 604800 if d > 302400 else d + 604800 if d < -302400 else d


class EphStore:
    """Collects navigation data per (system, PRN) and decodes ephemerides,
    plus the GPS Klobuchar ionosphere model."""

    def __init__(self):
        self.sub, self.eph, self.ion = {}, {}, None
        self.gal_crc_fail = 0

    def _keep(self, key, e):
        if e and not any(x["iode"] == e["iode"] and x["toe"] == e["toe"] for x in self.eph.get(key, [])):
            self.eph.setdefault(key, []).append(e)

    def add(self, prn, sb):
        """GPS LNAV subframe: 30 bytes = 10 words x 24 data bits."""
        if len(sb) < 30 or sb[0] != 0x8B:
            return
        sfid = (sb[5] >> 2) & 7
        if not 1 <= sfid <= 5:
            return
        key = ("G", prn)
        self.sub.setdefault(key, {})[sfid] = bytes(sb)
        s = self.sub[key]
        if sfid == 3 and 1 in s and 2 in s:
            self._keep(key, decode_eph(s))
        if sfid == 4 and getbitu(sb, 50, 6) == 56 and self.ion is None:
            sc = [2 ** -30, 2 ** -27, 2 ** -24, 2 ** -24, 2 ** 11, 2 ** 14, 2 ** 16, 2 ** 16]
            self.ion = [getbits(sb, 56 + 8 * k, 8) * sc[k] for k in range(8)]

    def add_gal(self, prn, dwords):
        """Galileo I/NAV even+odd page pair as 8 big-endian dwords (SkyTraq
        0xE6). Checks the CRC, files the 128-bit word by type, decodes on
        type 5 (RTKLIB decode_stqgene)."""
        buf = bytearray(32)
        for i, d in enumerate(dwords[:8]):
            buf[4 * i:4 * i + 4] = d.to_bytes(4, "big")
        if getbitu(buf, 0, 1) != 0 or getbitu(buf, 128, 1) != 1:
            return
        if getbitu(buf, 1, 1) or getbitu(buf, 129, 1):
            return                                        # alert page
        c = bytearray(26)                                 # 4 pad + 114 even + 82 odd bits
        for i in range(15):
            setbitu(c, 4 + 8 * i, 8, getbitu(buf, 8 * i, 8))
        for i in range(11):
            setbitu(c, 118 + 8 * i, 8, getbitu(buf, 128 + 8 * i, 8))
        if crc24q(c[:25]) != getbitu(buf, 128 + 82, 24):
            self.gal_crc_fail += 1
            return
        wtype = getbitu(buf, 2, 6)
        if wtype > 6:
            return
        key = ("E", prn)
        w = self.sub.setdefault(key, bytearray(16 * 7))
        for i in range(14):
            w[wtype * 16 + i] = getbitu(buf, 2 + 8 * i, 8)
        for i in range(14, 16):
            w[wtype * 16 + i] = getbitu(buf, 130 + 8 * (i - 14), 8)
        if wtype == 5:
            e = decode_gal_inav(w)
            if e and e.get("svid") == prn:
                self._keep(key, e)

    def add_bds_d1(self, prn, sfid, data):
        """BeiDou D1 subframe (SkyTraq 0xE2): word 1 as 26 bits, words 2-10 as
        22 bits each, parity stripped; re-spread to 30-bit words."""
        if not 1 <= sfid <= 5 or prn <= 5 or prn >= 59 or len(data) < 28:
            return                                        # GEO (D2) not handled
        key = ("C", prn)
        sub = self.sub.setdefault(key, bytearray(38 * 5))
        base = (sfid - 1) * 38 * 8
        j = 0
        setbitu(sub, base, 30, getbitu(data, j, 26) << 4); j += 26
        for i in range(1, 10):
            setbitu(sub, base + 30 * i, 30, getbitu(data, j, 22) << 8); j += 22
        if sfid == 3:
            self._keep(key, decode_bds_d1(sub))

    def pick(self, sys_or_prn, prn_or_t, t=None):
        """pick(sys, prn, t); pick(prn, t) keeps the GPS-only call working."""
        if t is None:
            key, t = ("G", sys_or_prn), prn_or_t
        else:
            key = (sys_or_prn, prn_or_t)
        best = None
        for e in self.eph.get(key, []):
            dt = abs(dt_week(t, e["toe"]))
            if dt < 7200 and (best is None or dt < best[0]):
                best = (dt, e)
        return best[1] if best else None


def sat_clock(e, t):
    ts = dt_week(t, e["toc"]); tt = ts
    for _ in range(2):
        tt = ts - (e["f0"] + e["f1"] * tt + e["f2"] * tt * tt)
    return e["f0"] + e["f1"] * tt + e["f2"] * tt * tt


def sat_pos(e, t):
    """ECEF position (m) and clock including relativity (s) at GPS TOW t."""
    k = SYS[e.get("sys", "G")]
    mu, omge = k["mu"], k["omge"]
    tk = dt_week(t, e["toe"])
    M = e["M0"] + (math.sqrt(mu / e["A"] ** 3) + e["deln"]) * tk
    E = M
    for _ in range(30):
        Ek = E; E -= (E - e["e"] * math.sin(E) - M) / (1 - e["e"] * math.cos(E))
        if abs(E - Ek) < 1e-13:
            break
    sE, cE = math.sin(E), math.cos(E)
    u = math.atan2(math.sqrt(1 - e["e"] ** 2) * sE, cE - e["e"]) + e["omg"]
    r = e["A"] * (1 - e["e"] * cE); inc = e["i0"] + e["idot"] * tk
    s2, c2 = math.sin(2 * u), math.cos(2 * u)
    u += e["cus"] * s2 + e["cuc"] * c2; r += e["crs"] * s2 + e["crc"] * c2; inc += e["cis"] * s2 + e["cic"] * c2
    x, y = r * math.cos(u), r * math.sin(u)
    toe_own = e["toe"] - (BDT_GPST if e.get("sys") == "C" else 0.0)   # BDS: Omega uses BDT toe
    O = e["OMG0"] + (e["OMGd"] - omge) * tk - omge * toe_own
    pos = np.array([x * math.cos(O) - y * math.cos(inc) * math.sin(O),
                    x * math.sin(O) + y * math.cos(inc) * math.cos(O), y * math.sin(inc)])
    tc = dt_week(t, e["toc"])
    dts = e["f0"] + e["f1"] * tc + e["f2"] * tc * tc - 2 * math.sqrt(mu * e["A"]) * e["e"] * sE / C_LIGHT ** 2
    return pos, dts


def klobuchar(ion, tow, lat, lon, az, el):
    """Broadcast ionosphere delay on L1 (m). With no parameters decoded, the
    model's night-time 5 ns term is all that is left."""
    a = ion[:4] if ion else [0, 0, 0, 0]; b = ion[4:] if ion else [0, 0, 0, 0]
    E = el / math.pi; phi_u = lat / math.pi; lam_u = lon / math.pi
    psi = 0.0137 / (E + 0.11) - 0.022
    phi_i = max(-0.416, min(0.416, phi_u + psi * math.cos(az)))
    lam_i = lam_u + psi * math.sin(az) / math.cos(phi_i * math.pi)
    phi_m = phi_i + 0.064 * math.cos((lam_i - 1.617) * math.pi)
    t = (4.32e4 * lam_i + tow) % 86400
    F = 1.0 + 16.0 * (0.53 - E) ** 3
    amp = max(0.0, sum(a[n] * phi_m ** n for n in range(4)))
    per = max(72000.0, sum(b[n] * phi_m ** n for n in range(4)))
    x = 2 * math.pi * (t - 50400) / per
    return F * (5e-9 + (amp * (1 - x * x / 2 + x ** 4 / 24) if abs(x) < 1.57 else 0.0)) * C_LIGHT


def tropo(h, el):
    """Standard-atmosphere zenith delay mapped by 1/sin(el) (Saastamoinen-like)."""
    if h < -100 or h > 1e4:
        return 0.0
    zd = 2.3 * math.exp(-0.000116 * h)
    return zd / max(math.sin(el), 0.1)


# ------------------------------------------------------------------ epochs
def corrected(tow, obs, eph, r_approx, use_tropo=True, el_mask_deg=5.0):
    """obs: [(sys, prn, pr_m | None, doppler_hz | None, cn0)] -> list[RawMeas]
    (a 4-tuple (prn, pr, dop, cn0) is taken as GPS).

    Satellite clock (with relativity and the signal's group delay: GPS TGD,
    Galileo BGD(E1,E5b), BeiDou TGD1) is added back to the pseudorange, the
    broadcast ionosphere (GPS Klobuchar, scaled to the signal's frequency) and
    optionally the troposphere are removed, and Doppler becomes range rate at
    the signal's own wavelength with the satellite clock drift removed.
    ``r_approx`` (ECEF) sets elevation for the atmosphere models and the mask;
    before the first fix pass None and nothing is masked."""
    out = []
    have_pos = r_approx is not None and np.linalg.norm(r_approx) > 6.0e6
    if have_pos:
        lat, lon, h = ecef2lla(np.asarray(r_approx, float))
        T = t_e2ned(lat, lon)
    for o in obs:
        sys, prn, pr, dop, cn0 = o if len(o) == 5 else ("G",) + tuple(o)
        if pr is None:
            continue
        e = eph.pick(sys, prn, tow)
        if not e or e["svh"] != 0:
            continue
        f = SYS[sys]["freq"]
        tt = tow - pr / C_LIGHT
        tt -= sat_clock(e, tt)
        rs, dts = sat_pos(e, tt)
        dts -= e["tgd_use"]
        rs2, dts2 = sat_pos(e, tt + 0.5)
        rs1, dts1 = sat_pos(e, tt - 0.5)
        vs = rs2 - rs1                      # ECEF-at-transmit velocity, 1 s central difference
        corr = C_LIGHT * dts
        if have_pos:
            los = rs - np.asarray(r_approx, float)
            nu = T @ (los / np.linalg.norm(los))
            el = math.asin(max(-1.0, min(1.0, -nu[2])))
            if math.degrees(el) < el_mask_deg:
                continue
            az = math.atan2(nu[1], nu[0])
            corr -= klobuchar(eph.ion, tow, lat, lon, az, el) * (1575.42e6 / f) ** 2
            if use_tropo:
                corr -= tropo(h, el)
        s_pr = 2.0 if cn0 >= 40 else 3.5 if cn0 >= 33 else 6.0
        s_rr = 0.08 if cn0 >= 40 else 0.15 if cn0 >= 33 else 0.4
        rr = None if dop is None else -(C_LIGHT / f) * dop + C_LIGHT * (dts2 - dts1)
        out.append(RawMeas(prn=prn, sat_pos=rs, sat_vel=vs, pr=pr + corr, rr=rr,
                           sigma_pr=s_pr, sigma_rr=s_rr, sys=sys))
    return out


# ------------------------------------------------------------------ readers
def read_capture(path, replay_source, systems="GEC"):
    """COCOM-rig capture -> (EphStore, [(tow, obs)], own_fixes, kind).

    obs = [(sys, prn, pr_m | None, doppler_hz | None, cn0)] for GPS L1 C/A,
    Galileo E1 and BeiDou B1I (SkyTraq; u-blox captures give GPS only here);
    own_fixes = [(tow, ecef, vel_ecef)] from the receiver's own solution.
    ``systems`` limits which are returned."""
    eph, epochs, own, kind = EphStore(), [], [], None
    for _t, k, d in replay_source(path):
        if k == "bin" and d:
            kind = "skytraq"
            if d[0] == 0xE0 and len(d) >= 33:
                eph.add(d[1], d[3:33])
            elif d[0] == 0xE6 and len(d) >= 37 and (d[2] & 0xF) == 3:
                eph.add_gal(d[3], [struct.unpack_from(">I", d, 5 + 4 * i)[0] for i in range(8)])
            elif d[0] == 0xE2 and len(d) >= 31:
                eph.add_bds_d1(d[1] - 200, d[2], d[3:31])
            elif d[0] == 0xE5 and len(d) >= 14:
                tow = struct.unpack_from(">I", d, 5)[0] * 1e-3
                obs = []
                for j in range(d[13]):
                    r = d[14 + 31 * j: 14 + 31 * (j + 1)]
                    if len(r) < 31 or (r[0] >> 4) != 0:
                        continue                      # first signal of each system only
                    sys = SKY_GNSS.get(r[0] & 0xF)
                    if sys is None or sys not in systems:
                        continue
                    ind = struct.unpack_from(">H", r, 27)[0]
                    obs.append((sys, r[1], struct.unpack_from(">d", r, 4)[0] if ind & 1 else None,
                                struct.unpack_from(">f", r, 20)[0] if ind & 2 else None, r[3]))
                epochs.append((tow, obs))
            elif d[0] == 0xDF and len(d) >= 61 and d[2] >= 2:
                own.append((struct.unpack_from(">d", d, 5)[0], np.array(struct.unpack_from(">ddd", d, 13)),
                            np.array(struct.unpack_from(">fff", d, 37), float)))
        elif k == "ubx" and len(d) >= 4:
            kind = "ubx"
            cid, p = d[:2], d[2:]
            if cid == b"\x02\x13" and len(p) >= 48 and p[0] == 0 and p[4] >= 10:
                words = struct.unpack_from("<10I", p, 8)
                eph.add(p[1], b"".join(((w >> 6) & 0xFFFFFF).to_bytes(3, "big") for w in words))
            elif cid == b"\x02\x15" and len(p) >= 16:
                tow = struct.unpack_from("<d", p, 0)[0]
                obs = []
                for j in range(p[11]):
                    b = p[16 + 32 * j: 16 + 32 * (j + 1)]
                    if len(b) < 32 or b[20] != 0 or "G" not in systems:
                        continue
                    obs.append(("G", b[21], struct.unpack_from("<d", b, 0)[0] if b[30] & 1 else None,
                                struct.unpack_from("<f", b, 16)[0], b[26]))
                epochs.append((tow, obs))
            elif cid == b"\x01\x07" and len(p) >= 92 and (p[21] & 1) and p[20] >= 3:
                from .tc_ekf import lla2ecef
                itow = struct.unpack_from("<I", p, 0)[0] / 1000.0
                lon, lat, hgt = struct.unpack_from("<iii", p, 24)
                vn, ve, vd = struct.unpack_from("<iii", p, 48)
                la, lo = math.radians(lat * 1e-7), math.radians(lon * 1e-7)
                x = lla2ecef(np.array([la, lo, hgt / 1000.0]))
                v = t_e2ned(la, lo).T @ np.array([vn, ve, vd]) / 1000.0
                own.append((itow, x, v))
    return eph, epochs, own, kind
