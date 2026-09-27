#!/usr/bin/env python3
"""How fast code and carrier drift apart on this rig, from any raw capture.

Per GPS satellite and per continuous arc, code-minus-carrier (pseudorange minus
carrier phase times the L1 wavelength) grows at the rate the transmitted code and
carrier disagree. Geometry, satellite clocks and the receiver's own clock all cancel
in the difference, so no truth or ephemeris is needed: a straight line through each
arc of 60 s or more gives the rate, and the median over arcs is the answer.

On the HackRF rig as it has been since August it reads +4.19 m/s on every receiver:
the transmitted carrier sits 22.0 Hz above where its own code says it should be. On
the real sky it reads 0.00 m/s. With patch_carrier_offset.py's -DCARR_OFFSET_HZ=-22.0
it should read about 0 on the rig too; that is the check.

    ./code_carrier.py CAPTURE.log[.gz] [...]        [--from S] [--to S] (receiver TOW
                                                    minus 203400, i.e. file time)

Reads SkyTraq 0xE5 (PX1105R, PX1125R), UBX RXM-RAWX (NEO-M8T, ZED-F9P) and RTCM3 MSM7
(LC86G). Whole-millisecond receiver clock steps move the pseudorange alone and are
taken out; an arc ends at a gap, a cycle-slip flag or a jump over 50 m.

Carrier smoothing inside the receiver bends an arc's first ~20-40 s (a smoothed
pseudorange restarts at the raw code and settles rate x tau below it), so only the
part of each arc after --settle seconds (default 30) is fitted.

Needs a receiver whose pseudorange follows the code. One that drives its code with
its carrier between re-alignments reports code-minus-carrier flat and reads near 0
whatever the rig does: the PX1125R as configured in August (gentle_alt_eq reads
+0.39) and the SkyTraq parts in power save. The PX1105R in power normal, the
PX1125R in its September setup, the NEO-M8T and the LC86G all follow the code.
"""
from __future__ import annotations

import argparse
import gzip
import math
import statistics
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

C = 299792458.0
LAM = C / 1575.42e6
MS = C * 1e-3
TOW0 = 203400.0


def lines(path):
    op = gzip.open if str(path).endswith(".gz") else open
    with op(path, "rt", errors="replace") as fh:
        for raw in fh:
            ts, _, rest = raw.rstrip("\n").partition(" ")
            if rest[:2] in ("B ", "U ", "R "):
                try:
                    yield rest[0], bytes.fromhex(rest[2:])
                except ValueError:
                    pass


def epochs(path):
    """(tow_s, {prn: (pseudorange_m, carrier_cycles, slip)}) for GPS L1 C/A."""
    msm = None
    for tag, d in lines(path):
        if tag == "B" and d and d[0] == 0xE5 and len(d) >= 14:
            tow = struct.unpack_from(">I", d, 5)[0] * 1e-3
            out = {}
            for j in range(d[13]):
                r = d[14 + 31 * j: 14 + 31 * (j + 1)]
                if len(r) < 31 or r[0] != 0:                 # GPS, first signal
                    continue
                ind = struct.unpack_from(">H", r, 27)[0]
                if ind & 1 and ind & 4:
                    out[r[1]] = (struct.unpack_from(">d", r, 4)[0], struct.unpack_from(">d", r, 12)[0],
                                 bool(ind & 8))
            yield tow, out
        elif tag == "U" and d[:2] == b"\x02\x15" and len(d) >= 18:
            p = d[2:]
            out = {}
            for j in range(p[11]):
                b = p[16 + 32 * j: 16 + 32 * (j + 1)]
                if len(b) < 32 or b[20] != 0:
                    continue
                trk = b[30]
                if trk & 1 and trk & 2:
                    out[b[21]] = (struct.unpack_from("<d", b, 0)[0], struct.unpack_from("<d", b, 8)[0],
                                  not trk & 4)
            yield struct.unpack_from("<d", p, 0)[0], out
        elif tag == "R" and d:
            if msm is None:
                from msm7_clock import msm7_full as msm
            try:
                full = msm(d)
            except (ValueError, IndexError):
                continue
            if full:
                yield full[0] / 1000.0, {s: (pr, cp / LAM, False) for s, pr, cp in full[1] if cp is not None}


def arcs(path, t_from, t_to, gap):
    series = {}
    for tow, meas in epochs(path):
        t = tow - TOW0
        if not t_from <= t <= t_to:
            continue
        for prn, (pr, cp, slip) in meas.items():
            series.setdefault(prn, []).append((t, pr - LAM * cp, slip))
    for prn, s in series.items():
        s.sort()
        arc = []
        for t, cmc, slip in s:
            if arc:
                d = cmc - arc[-1][1]
                d -= round(d / MS) * MS                       # receiver clock step
                cmc = arc[-1][1] + d
                if t - arc[-1][0] > gap or slip or abs(d) > 50.0:
                    yield prn, arc
                    arc = []
            arc.append((t, cmc))
        if arc:
            yield prn, arc


def slope(arc, settle, min_len):
    t0 = arc[0][0]
    pts = [(t - t0, v) for t, v in arc if t - t0 >= settle]
    if len(pts) < 10 or pts[-1][0] - pts[0][0] < min_len:
        return None
    n = len(pts)
    mt = sum(p[0] for p in pts) / n
    mv = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mt) ** 2 for p in pts)
    return sum((p[0] - mt) * (p[1] - mv) for p in pts) / sxx if sxx > 0 else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--from", dest="t_from", type=float, default=-1e9, help="file time, s")
    ap.add_argument("--to", dest="t_to", type=float, default=1e9, help="file time, s")
    ap.add_argument("--settle", type=float, default=30.0, help="skip each arc's first S seconds")
    ap.add_argument("--min-len", type=float, default=30.0, help="fit at least S seconds of an arc")
    ap.add_argument("--gap", type=float, default=1.5, help="an arc ends at a gap longer than S")
    args = ap.parse_args()
    for cap in args.captures:
        rates = [r for _, a in arcs(cap, args.t_from, args.t_to, args.gap)
                 if (r := slope(a, args.settle, args.min_len)) is not None]
        name = Path(cap).name
        if not rates:
            print(f"{name}: no arc long enough (pseudorange + carrier phase needed)")
            continue
        rates.sort()
        med = statistics.median(rates)
        p10, p90 = rates[len(rates) // 10], rates[(9 * len(rates)) // 10]
        print(f"{name}: code-minus-carrier {med:+.3f} m/s over {len(rates)} arcs "
              f"(p10 {p10:+.3f}, p90 {p90:+.3f}) = carrier {med / LAM:+.1f} Hz off its code")


if __name__ == "__main__":
    main()
