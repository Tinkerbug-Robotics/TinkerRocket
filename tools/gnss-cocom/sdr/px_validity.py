#!/usr/bin/env python3
"""Are the PX1105R's raw measurements right? Every 0xE5 GPS L1 measurement against
the truth: the receiver where gps-sdr-sim put it (the motion CSV), each satellite
where the ephemeris puts it at transmit time.

  pseudorange residual  PR - geometric range (+ satellite clock), per-epoch median
                        removed (receiver clock bias); spread in m
  Doppler residual      measured Doppler vs -(range rate)/lambda for the MOVING
                        receiver, per-epoch median removed (receiver clock drift);
                        spread in Hz
  flags                 validity word: 1 PR, 2 Doppler, 4 carrier, 8 slip possible,
                        32 half-cycle unresolved

    px_validity.py CAPTURE MOTION_CSV [WINDOW_S]
"""
import bisect
import math
import statistics as st
import struct
import sys
from pathlib import Path

SDR = Path(__file__).resolve().parent
sys.path.insert(0, str(SDR))
from msm7_clock import rinex2_nav, sat_pos, rx_ecef, C, WE     # noqa: E402

TOW0 = 203400.0
LAM = C / 1575.42e6
cap, motion = sys.argv[1], sys.argv[2]
WIN = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

mt, mp = [], []
for line in open(motion):
    f = line.strip().split(",")
    if len(f) == 4:
        mt.append(float(f[0])); mp.append(rx_ecef(float(f[1]), float(f[2]), float(f[3])))


def rx_at(t):
    i = min(max(bisect.bisect_left(mt, t), 1), len(mt) - 1)
    a, b = mp[i - 1], mp[i]
    fr = (t - mt[i - 1]) / (mt[i] - mt[i - 1])
    return tuple(a[k] + fr * (b[k] - a[k]) for k in range(3))


eph = rinex2_nav(SDR / "c8" / "BRDC_2026230.rx2.n")


def rng(e, t, rx):
    """Geometric range at receive time t (transmit-time satellite, Earth turned)
    minus the satellite clock, as gps-sdr-sim builds it (ionosphere not included)."""
    p = sat_pos(e, t)
    tau = math.dist(p, rx) / C
    p = sat_pos(e, t - tau)
    th = WE * tau
    p = (p[0] * math.cos(th) + p[1] * math.sin(th), -p[0] * math.sin(th) + p[1] * math.cos(th), p[2])
    return math.dist(p, rx) - C * (e["af0"] + e["af1"] * (t - e["toe"]))


rows = []      # (file t, n, pr spread, dop spread, flags seen)
for line in open(cap, errors="replace"):
    p = line.split(" ", 2)
    if len(p) < 3 or p[1] != "B" or not p[2].startswith("e5"):
        continue
    x = bytes.fromhex(p[2].strip())
    tow = struct.unpack_from(">I", x, 5)[0] * 1e-3
    ft = tow - TOW0
    if not 0 < ft < mt[-1] - 1:
        continue
    prr, dpr, flags = [], [], 0
    rx0 = rx_at(ft)
    for j in range(x[13]):
        r = x[14 + 31 * j: 14 + 31 * (j + 1)]
        if len(r) < 31 or (r[0] & 0xF) != 0 or (r[0] >> 4) != 0 or r[1] not in eph:
            continue                                        # GPS L1 C/A only
        ind = struct.unpack_from(">H", r, 27)[0]
        flags |= ind
        e = min(eph[r[1]], key=lambda q: abs(tow - q["toe"]))
        if ind & 1:
            prr.append(struct.unpack_from(">d", r, 4)[0] - rng(e, tow, rx0))
        if ind & 2:
            d = struct.unpack_from(">f", r, 20)[0]
            if d == int(d) and d != 0:
                d += 0.5 * math.copysign(1.0, d)             # whole-hertz truncation
            h = 0.05
            rr = (rng(e, tow + h, rx_at(ft + h)) - rng(e, tow - h, rx_at(ft - h))) / (2 * h)
            dpr.append(d + rr / LAM)                         # 0 if Doppler = -rr/lambda
    if len(prr) >= 3:
        mp_, md = st.median(prr), (st.median(dpr) if dpr else 0.0)
        rows.append((ft, len(prr), st.pstdev([v - mp_ for v in prr]),
                     max(abs(v - mp_) for v in prr),
                     st.pstdev([v - md for v in dpr]) if len(dpr) >= 3 else float("nan"),
                     max((abs(v - md) for v in dpr), default=float("nan")), flags))

print(f"{Path(cap).name}: {len(rows)} epochs with 3+ GPS measurements")
print(f"{'window (file s)':<16}{'epochs':>7}{'sats':>6}{'PR spread m':>13}{'PR worst m':>12}"
      f"{'Dop spread Hz':>15}{'Dop worst Hz':>14}  flags seen")
bins = {}
for r in rows:
    bins.setdefault(int(r[0] // WIN) * WIN, []).append(r)
for w in sorted(bins):
    v = bins[w]
    fl = 0
    for r in v:
        fl |= r[6]
    print(f"{w:6.0f}-{w + WIN:<9.0f}{len(v):7d}{st.median(r[1] for r in v):6.0f}"
          f"{st.median(r[2] for r in v):13.1f}{max(r[3] for r in v):12.1f}"
          f"{st.median(r[4] for r in v):15.2f}{max(r[5] for r in v):14.1f}  0x{fl:02X}")
