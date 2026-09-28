#!/usr/bin/env python3
"""A receiver's own fix against the injection, by flight phase: its height error (own
ellipsoidal height minus the truth's) and its east/north offset from the injected track,
which climbs straight up over the scenario origin. Written for A/B runs of one trajectory,
first the carrier-corrected IQ files (patch_carrier_offset.py) against the originals.

    ./own_fix.py SHIFT SCENARIO CAPTURE [CAPTURE ...]       # captures plain or .gz
    ./own_fix.py 420 gentle_alt results/lc86g_20260927_gentle_alt_pad600_smooth_balloon_msm7.log.gz \\
        results/lc86g_20260927_gentle_alt_pad600_smooth_cofs_balloon_msm7.log.gz

Reads whichever own fix the capture carries: the LC86G's $PQTMPVT (lc86_bench_run.py), a
u-blox's UBX NAV-PVT (run_radiated.py) or a SkyTraq's 0xDF (px1105r_run.py). File time =
GPS TOW - 203400 and truth time = file time - SHIFT (420 for *_pad600), as in
lc86_limits.py. The scenario JSON comes from scenarios/ (gitignored: ./make_flights.py
--lat 0 --lon -119 --only NAME, then --retime for signal timing). Fixes from the first 30 s
after the first 3-D fix are left out while the solution settles.
"""
import bisect
import gzip
import json
import math
import statistics as stt
import struct
import sys
from pathlib import Path

SDR = Path(__file__).resolve().parent
TOW0 = 203400.0
M_PER_DEG = 111319.49            # one degree of latitude, or of longitude at the equator
SETTLE_S = 30.0


def lla(x, y, z):
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - e2))
    for _ in range(6):
        n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - e2 * n / (n + h)))
    return math.degrees(lat), math.degrees(lon), h


def lines(path):
    op = gzip.open if str(path).endswith(".gz") else open
    with op(path, "rt", errors="replace") as fh:
        yield from fh


def own_fixes(path):
    """(file t, lat deg, lon deg, ellipsoidal height m) for every epoch with a 3-D fix."""
    out = []
    for line in lines(path):
        p = line.rstrip("\n").split(" ", 2)
        if len(p) < 2:
            continue
        if p[1].startswith("$PQTMPVT"):
            f = (p[1] + (" " + p[2] if len(p) == 3 else "")).split("*")[0].split(",")
            try:
                if int(f[5] or 0) > 0 and int(f[6] or 0) >= 3:
                    out.append((int(f[2]) / 1000.0 - TOW0, float(f[9]), float(f[10]),
                                float(f[11]) + float(f[12] or 0.0)))
            except (ValueError, IndexError):
                pass
        elif p[1] == "U" and len(p) == 3:
            d = bytes.fromhex(p[2])
            if len(d) >= 62 and d[0] == 0x01 and d[1] == 0x07:          # NAV-PVT
                pl = d[2:]
                if pl[21] & 0x01 and pl[20] >= 3:
                    lon, lat, h = struct.unpack_from("<iii", pl, 24)
                    out.append((struct.unpack_from("<I", pl, 0)[0] / 1000.0 - TOW0,
                                lat * 1e-7, lon * 1e-7, h / 1000.0))
        elif p[1] == "B" and len(p) == 3:
            x = bytes.fromhex(p[2].strip())
            if len(x) >= 61 and x[0] == 0xDF and x[2] >= 3:              # 3-D fix
                tow = struct.unpack(">d", x[5:13])[0]
                X, Y, Z = struct.unpack(">ddd", x[13:37])
                if tow and (X or Y or Z):
                    out.append((tow - TOW0, *lla(X, Y, Z)))
    return sorted(out)


def main() -> int:
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    shift = float(sys.argv[1])
    sc = json.loads((SDR / "scenarios" / f"{sys.argv[2]}.json").read_text())
    tr = sc["truth"]
    tt = [s["t"] for s in tr]
    lat0, lon0 = sc["origin"]["lat_deg"], sc["origin"]["lon_deg"]

    def sample(ft):
        x = ft - shift
        i = min(max(bisect.bisect_left(tt, x), 1), len(tt) - 1)
        a, b = tr[i - 1], tr[i]
        f = (x - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0.0
        return a["alt_m"] + f * (b["alt_m"] - a["alt_m"]), (a if f < 0.5 else b)["phase"]

    apo = max(range(len(tr)), key=lambda i: tr[i]["alt_m"])
    landing = shift + tt[-1]
    print(f"apogee {shift + tr[apo]['t']:.1f} s ({tr[apo]['alt_m'] / 1000:.1f} km), "
          f"landing {landing:.1f} s (file time)")

    for path in sys.argv[3:]:
        rows = []
        for ft, lat, lon, h in own_fixes(path):
            alt, phase = sample(ft)
            rows.append((ft, h - alt, (lon - lon0) * M_PER_DEG * math.cos(math.radians(lat0)),
                         (lat - lat0) * M_PER_DEG, alt, phase))
        print(f"\n{Path(path).name}")
        if not rows:
            print("  no 3-D fix")
            continue
        first = rows[0][0]
        print(f"  first 3-D fix at file {first:.1f} s")
        print(f"  {'':9} {'fixes':>6}  {'height error, m':^38}  {'east, m':^22}  north, m")
        print(f"  {'':9} {'':>6}  {'median':>7} {'p5':>7} {'p95':>7}  {'worst (file s, km)':<14}"
              f"  {'median':>7} {'p5':>6} {'p95':>6}  {'median':>7}")
        for phase, label in (("prologue", "pad"), ("boost", "boost"), ("coast", "coast"),
                             ("descent", "descent")):
            rr = [r for r in rows if r[5] == phase and r[0] >= first + SETTLE_S]
            if not rr:
                print(f"  {label:9} {0:6d}  no fix")
                continue
            dh = sorted(r[1] for r in rr)
            de = sorted(r[2] for r in rr)
            w = max(rr, key=lambda r: abs(r[1]))
            print(f"  {label:9} {len(rr):6d}  {stt.median(dh):+7.1f} {dh[len(dh) // 20]:+7.1f} "
                  f"{dh[19 * len(dh) // 20]:+7.1f}  {w[1]:+7.1f} ({w[0]:.0f}, {w[4] / 1000:.1f})"
                  f"  {stt.median(de):+7.1f} {de[len(de) // 20]:+6.1f} {de[19 * len(de) // 20]:+6.1f}"
                  f"  {stt.median(r[3] for r in rr):+7.1f}")
        desc = [r for r in rows if r[5] == "descent"]
        print("  on the descent, nearest fix to each injected altitude:")
        for km in (70, 50, 30, 20, 10, 5, 2):
            near = [r for r in desc if abs(r[4] - km * 1000) < 0.02 * km * 1000 + 50]
            if near:
                r = min(near, key=lambda r: abs(r[4] - km * 1000))
                print(f"    {km:3d} km  height {r[1]:+8.1f}  east {r[2]:+8.1f}  north {r[3]:+7.1f}"
                      f"   (file {r[0]:.1f} s)")
            else:
                print(f"    {km:3d} km  no fix")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
