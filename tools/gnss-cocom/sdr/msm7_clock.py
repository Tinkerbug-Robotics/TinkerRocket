#!/usr/bin/env python3
"""The receiver clock as the LC86G's MSM7 shows it, for a spaceshot capture's pad.

    ./msm7_clock.py LABEL=CAPTURE [LABEL=CAPTURE ...]

Two numbers per capture, both against the broadcast ephemeris the file was made
from (c8/BRDC_2026230.rx2.n; the pad is 0.0, -119.0, 1200 m):

  Doppler offset  MSM7 phase-range rate minus the geometric range rate, common to
                  every satellite -- the HackRF's frequency error against the
                  receiver's oscillator, IF the receiver left it in. The LC86G does
                  not: it reads 0.0 +- 0.2 ppb in every run of 2026-09-26, with
                  0.02 m/s left per satellite.
  PR bias         MSM7 pseudorange minus the geometric range, common to every
                  satellite: the receiver clock bias. Arbitrary until the first
                  fix (~37 s), which steps it near zero; it then grows ~4 m/s
                  (~13 ppb) in held and lost boost runs alike.

So the pad clock does not separate runs that hold satellites through the burn
from runs that lose them (2026-09-26).
"""
import gzip
import math
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import rtcm3                                                      # noqa: E402
from rtcm3 import _Bits, message_type, MSM7_TYPES                 # noqa: E402

C = 299792458.0
MU, WE = 3.986005e14, 7.2921151467e-5
TOW0 = 203400.0


def rinex2_nav(path):
    lines = Path(path).read_text().splitlines()
    k = next(i for i, l in enumerate(lines) if "END OF HEADER" in l) + 1
    eph = {}
    while k + 7 < len(lines):
        rec = lines[k:k + 8]
        k += 8
        f = lambda s: float(s.replace("D", "E").replace("d", "e")) if s.strip() else 0.0
        prn = int(rec[0][0:2])
        v = [f(rec[0][22:41]), f(rec[0][41:60]), f(rec[0][60:79])]
        for r in rec[1:]:
            v += [f(r[3 + 19 * j: 3 + 19 * (j + 1)]) for j in range(4)]
        e = dict(af0=v[0], af1=v[1], af2=v[2], iode=v[3], crs=v[4], dn=v[5], m0=v[6],
                 cuc=v[7], ecc=v[8], cus=v[9], sqa=v[10], toe=v[11], cic=v[12], om0=v[13],
                 cis=v[14], i0=v[15], crc=v[16], w=v[17], omd=v[18], idot=v[19])
        eph.setdefault(prn, []).append(e)
    return eph


def sat_pos(e, t):
    a = e["sqa"] ** 2
    tk = t - e["toe"]
    tk -= 604800 * round(tk / 604800)
    n = math.sqrt(MU / a ** 3) + e["dn"]
    m = e["m0"] + n * tk
    E = m
    for _ in range(12):
        E = m + e["ecc"] * math.sin(E)
    nu = math.atan2(math.sqrt(1 - e["ecc"] ** 2) * math.sin(E), math.cos(E) - e["ecc"])
    phi = nu + e["w"]
    s2, c2 = math.sin(2 * phi), math.cos(2 * phi)
    u = phi + e["cus"] * s2 + e["cuc"] * c2
    r = a * (1 - e["ecc"] * math.cos(E)) + e["crs"] * s2 + e["crc"] * c2
    i = e["i0"] + e["cis"] * s2 + e["cic"] * c2 + e["idot"] * tk
    xp, yp = r * math.cos(u), r * math.sin(u)
    om = e["om0"] + (e["omd"] - WE) * tk - WE * e["toe"]
    return (xp * math.cos(om) - yp * math.cos(i) * math.sin(om),
            xp * math.sin(om) + yp * math.cos(i) * math.cos(om),
            yp * math.sin(i))


def geo_rate(e, t, rx):
    """Geometric range rate (m/s) minus the satellite clock drift, as gps-sdr-sim puts it."""
    def rng(tt):
        p = sat_pos(e, tt)
        tau = math.dist(p, rx) / C
        p = sat_pos(e, tt - tau)
        th = WE * tau                          # Earth turns during the transit
        p = (p[0] * math.cos(th) + p[1] * math.sin(th), -p[0] * math.sin(th) + p[1] * math.cos(th), p[2])
        return math.dist(p, rx)
    h = 0.05
    return (rng(t + h) - rng(t - h)) / (2 * h) - C * (e["af1"] + 2 * e["af2"] * (t - e["toe"]))


def rx_ecef(lat, lon, h):
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    la, lo = math.radians(lat), math.radians(lon)
    N = a / math.sqrt(1 - e2 * math.sin(la) ** 2)
    return ((N + h) * math.cos(la) * math.cos(lo), (N + h) * math.cos(la) * math.sin(lo),
            (N * (1 - e2) + h) * math.sin(la))


def msm7_full(body):
    mt = message_type(body)
    if mt not in MSM7_TYPES or MSM7_TYPES[mt] != "GPS":
        return None
    b = _Bits(body); b.u(12); b.u(12); epoch = b.u(30)
    b.u(1); b.u(3); b.u(7); b.u(2); b.u(2); b.u(1); b.u(3)
    sm, gm = b.u(64), b.u(32)
    sats = [i + 1 for i in range(64) if sm >> (63 - i) & 1]
    sigs = [i + 1 for i in range(32) if gm >> (31 - i) & 1]
    cells = [(s, g) for s in sats for g in sigs if b.u(1)]
    ns, nc = len(sats), len(cells)
    rms = [b.u(8) for _ in range(ns)]; [b.u(4) for _ in range(ns)]
    rmod = [b.u(10) for _ in range(ns)]; [b.s(14) for _ in range(ns)]
    fpr = [b.s(20) for _ in range(nc)]; fcp = [b.s(24) for _ in range(nc)]
    idx = {s: k for k, s in enumerate(sats)}
    out = []
    for c, (s, g) in enumerate(cells):
        k = idx[s]
        if rms[k] == 255 or fpr[c] == -524288:
            continue
        base = rms[k] + rmod[k] / 1024.0
        out.append((s, (base + fpr[c] * 2 ** -29) * C / 1000.0,
                    (base + fcp[c] * 2 ** -31) * C / 1000.0 if fcp[c] != -8388608 else None))
    return epoch, out

def georange(e, t, rx):
    p = sat_pos(e, t); tau = math.dist(p, rx) / C
    p = sat_pos(e, t - tau); th = WE * tau
    p = (p[0]*math.cos(th) + p[1]*math.sin(th), -p[0]*math.sin(th) + p[1]*math.cos(th), p[2])
    return math.dist(p, rx) - C * (e["af0"] + e["af1"] * (t - e["toe"]))


def lines(path):
    op = gzip.open if str(path).endswith(".gz") else open
    with op(path, "rt", errors="replace") as fh:
        for raw in fh:
            yield raw.rstrip("\n").partition(" ")[2]


def series(path, eph, rx):
    """[(file time, Doppler offset m/s, PR bias m, n)] from GPS MSM7 up to ignition."""
    out = []
    for rest in lines(path):
        if not rest.startswith("R "):
            continue
        try:
            body = bytes.fromhex(rest[2:])
            r = rtcm3.parse_msm7(body)
            full = msm7_full(body)
        except (ValueError, IndexError):
            continue
        if not r or r[0] != "GPS" or not full:
            continue
        ft = r[1] / 1000.0 - TOW0
        if not 0 <= ft <= 179.5:
            continue
        tow = TOW0 + ft
        best = {p: min(eph[p], key=lambda x: abs(tow - x["toe"])) for p in eph}
        dop = [c["rate_mps"] - geo_rate(best[c["prn"]], tow, rx) for c in r[2]
               if c.get("rate_mps") is not None and c["prn"] in best]
        prb = [pr - georange(best[p], tow, rx) for p, pr, _cp in full[1] if p in best]
        if len(dop) >= 4 and len(prb) >= 4:
            out.append((ft, st.median(dop), st.median(prb), len(dop)))
    return out


def main() -> int:
    eph = rinex2_nav(HERE / "c8" / "BRDC_2026230.rx2.n")
    rx = rx_ecef(0.0, -119.0, 1200.0)
    ppb = lambda v: v / C * 1e9
    print(f"{'run':<26}{'Doppler offset, ppb @60/120/175 s':>36}{'PR bias, m @30/100/175 s':>34}")
    for spec in sys.argv[1:]:
        label, path = spec.split("=", 1)
        s = series(path, eph, rx)
        if not s:
            print(f"{label:<26} no GPS MSM7 before ignition")
            continue

        def near(t, k):
            w = [x[k] for x in s if abs(x[0] - t) <= 2.5]
            return st.median(w) if w else float("nan")
        print(f"{label:<26}{ppb(near(60, 1)):12.1f}{ppb(near(120, 1)):12.1f}{ppb(near(175, 1)):12.1f}"
              f"{near(30, 2):12.0f}{near(100, 2):11.0f}{near(175, 2):11.0f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
