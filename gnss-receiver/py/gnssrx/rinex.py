"""RINEX 3 navigation file reader (GPS records, and Galileo I/NAV and BeiDou MEO/IGSO on request),
and satellite positions from it.

An independent Python implementation of IS-GPS-200's ephemeris model (Galileo's is the same
with its own GM; BeiDou's with CGCS2000's GM and Earth rate, in BDT), to check the C decoder and
orbit code against the broadcast files the simulators used.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

GPS_MU = 3.986005e14
GAL_MU = 3.986004418e14
BDS_MU = 3.986004418e14
OMEGA_E = 7.2921151467e-5
BDS_OMEGA_E = 7.2921150e-5
BDT_MINUS_GPST = -14.0
F_REL = -4.442807633e-10
C = 299792458.0


@dataclass
class Eph:
    prn: int
    toc_week_s: float      # toc as seconds of the GPS week (from the epoch line)
    af0: float
    af1: float
    af2: float
    iode: int
    crs: float
    delta_n: float
    m0: float
    cuc: float
    e: float
    cus: float
    sqrt_a: float
    toe: float
    cic: float
    omega0: float
    cis: float
    i0: float
    crc: float
    omega: float
    omega_dot: float
    idot: float
    week: int
    health: int
    tgd: float             # GPS TGD; Galileo BGD(E1,E5b), the I/NAV clock's E1 term; BeiDou TGD1 (B1I)
    iodc: int
    sys: str = "G"         # "G", "E" or "C"; Galileo's week and toe are GST, which runs with GPST;
                           # BeiDou's week, toe and toc are BDT, 14 s behind it (see sys_time)


def _num(s: str) -> float:
    return float(s.replace("D", "E").replace("d", "e"))


def _gps_sow(y, mo, d, h, mi, s) -> float:
    import datetime as dt
    t = dt.datetime(y, mo, d, h, mi, int(s)) - dt.datetime(1980, 1, 6)
    return (t.days * 86400 + t.seconds) % 604800 + (s - int(s))


def read_gps(path: str | Path) -> list[Eph]:
    return read_nav(path, "G")


def read_nav(path: str | Path, systems: str = "GE") -> list[Eph]:
    """The records of the given systems (G, E, C); Galileo's F/NAV records (an E5a clock) and BeiDou's
    GEO satellites (C01-C05, C59-C63: their orbits take another rotation) are left out. RINEX 2 files
    (GPS only) are read too."""
    lines = Path(path).read_text().splitlines()
    if lines and lines[0][:9].strip() and float(lines[0][:9]) < 3.0:
        return _read_nav_v2(lines) if "G" in systems else []
    i = 0
    while "END OF HEADER" not in lines[i]:
        i += 1
    i += 1
    out = []
    while i < len(lines):
        ln = lines[i]
        if not ln or ln[0] not in "GRECJIS":
            i += 1
            continue
        sys = ln[0]
        nrec = {"G": 8, "E": 8, "C": 8, "J": 8, "I": 8, "R": 4, "S": 4}.get(sys, 8)
        if sys not in "GEC" or sys not in systems:
            i += nrec
            continue
        prn = int(ln[1:3])
        if sys == "C" and (prn <= 5 or prn >= 59):
            i += nrec
            continue
        y, mo, d, h, mi, s = int(ln[4:8]), int(ln[9:11]), int(ln[12:14]), int(ln[15:17]), int(ln[18:20]), int(ln[21:23])
        vals = [_num(ln[23 + 19 * k: 42 + 19 * k]) for k in range(3)]
        for r in range(1, nrec):
            row = lines[i + r]
            for k in range(4):
                f = row[4 + 19 * k: 23 + 19 * k]
                vals.append(_num(f) if f.strip() else 0.0)
        af0, af1, af2 = vals[0:3]
        (iode, crs, dn, m0, cuc, e, cus, sqa, toe, cic, om0, cis, i0, crc, om, omd, idot, l2, week, _l2p,
         _sva, health, tgd, iodc) = vals[3:27]
        if sys == "E":
            if not (int(l2) & 5):
                i += nrec
                continue
            tgd, iodc = iodc, 0  # BGD(E1,E5b) sits where GPS keeps IODC
        elif sys == "C":
            iodc = 0  # TGD2 (B2I) sits there; TGD1 is in GPS's TGD place
        out.append(Eph(prn, _gps_sow(y, mo, d, h, mi, s), af0, af1, af2, int(iode), crs, dn, m0, cuc, e, cus, sqa, toe,
                       cic, om0, cis, i0, crc, om, omd, idot, int(week), int(health), tgd, int(iodc), sys))
        i += nrec
    return out


def _read_nav_v2(lines: list[str]) -> list[Eph]:
    """RINEX 2 GPS navigation: each record one column left of RINEX 3, with a two-digit PRN and year."""
    i = 0
    while "END OF HEADER" not in lines[i]:
        i += 1
    i += 1
    out = []
    while i + 7 < len(lines):
        ln = lines[i]
        if not ln.strip():
            i += 1
            continue
        prn = int(ln[0:2])
        y, mo, d, h, mi = (int(ln[c:c + 3]) for c in (2, 5, 8, 11, 14))
        s = float(ln[17:22])
        y += 2000 if y < 80 else 1900
        vals = [_num(ln[22 + 19 * k: 41 + 19 * k]) for k in range(3)]
        for r in range(1, 8):
            row = lines[i + r]
            for k in range(4):
                f = row[3 + 19 * k: 22 + 19 * k]
                vals.append(_num(f) if f.strip() else 0.0)
        af0, af1, af2 = vals[0:3]
        (iode, crs, dn, m0, cuc, e, cus, sqa, toe, cic, om0, cis, i0, crc, om, omd, idot, _l2, week, _l2p,
         _sva, health, tgd, iodc) = vals[3:27]
        out.append(Eph(prn, _gps_sow(y, mo, d, h, mi, s), af0, af1, af2, int(iode), crs, dn, m0, cuc, e, cus, sqa, toe,
                       cic, om0, cis, i0, crc, om, omd, idot, int(week), int(health), tgd, int(iodc), "G"))
        i += 8
    return out


def tdiff(t1: float, t0: float) -> float:
    d = t1 - t0
    if d > 302400:
        d -= 604800
    elif d < -302400:
        d += 604800
    return d


def sys_time(e: Eph, t_gps: float) -> float:
    """GPS time t_gps (s of week) in e's own system time: BDT for BeiDou, else the same."""
    return t_gps + BDT_MINUS_GPST if e.sys == "C" else t_gps


def sat_pos(e: Eph, t: float) -> tuple[tuple[float, float, float], float]:
    """ECEF position at time t (s of week, in e's system time: see sys_time) and the clock
    correction (s) for its signal (L1 C/A, E1, B1I)."""
    a = e.sqrt_a ** 2
    tk = tdiff(t, e.toe)
    n = math.sqrt((GPS_MU if e.sys == "G" else GAL_MU) / a ** 3) + e.delta_n
    m = e.m0 + n * tk
    ek = m
    for _ in range(30):
        ek = m + e.e * math.sin(ek)
    nu = math.atan2(math.sqrt(1 - e.e ** 2) * math.sin(ek), math.cos(ek) - e.e)
    phi = nu + e.omega
    du = e.cus * math.sin(2 * phi) + e.cuc * math.cos(2 * phi)
    dr = e.crs * math.sin(2 * phi) + e.crc * math.cos(2 * phi)
    di = e.cis * math.sin(2 * phi) + e.cic * math.cos(2 * phi)
    u, r, i = phi + du, a * (1 - e.e * math.cos(ek)) + dr, e.i0 + di + e.idot * tk
    xp, yp = r * math.cos(u), r * math.sin(u)
    we = BDS_OMEGA_E if e.sys == "C" else OMEGA_E
    om = e.omega0 + (e.omega_dot - we) * tk - we * e.toe
    x = xp * math.cos(om) - yp * math.cos(i) * math.sin(om)
    y = xp * math.sin(om) + yp * math.cos(i) * math.cos(om)
    z = yp * math.sin(i)
    tc = tdiff(t, e.toc_week_s)
    clk = e.af0 + e.af1 * tc + e.af2 * tc * tc + F_REL * e.e * e.sqrt_a * math.sin(ek) - e.tgd
    return (x, y, z), clk
