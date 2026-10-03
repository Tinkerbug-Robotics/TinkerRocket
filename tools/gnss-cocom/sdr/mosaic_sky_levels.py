#!/usr/bin/env python3
"""The mosaic-G5's own C/N0 on the real sky, per system and signal, overall and by elevation.

The sky reference the C/N0 sweep sets each receiver's bench levels against (sky_levels.py does the
PX1105R): MeasEpoch C/N0 once a second joined to SatVisibility elevations by receiver time. "L1" is
the signal the sweep's wide files carry for each system (GPS L1 C/A, Galileo E1, BeiDou B1I), so it
lines up with sky_px.json / sky_m8t.json; the other signals keep their own names.

    mosaic_sky_levels.py CAPTURE [CAPTURE ...] [--mask DEG] [--json OUT]

--json writes the same rows as sky_px.json (system, band, sv, el, cn0: one per satellite and signal
per 120 s slice, the slice's median). Keep it out of the repo: elevations over time locate the antenna.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import septentrio_sbf as sbf                            # noqa: E402

SYSTEM = {"G": "GPS", "R": "GLONASS", "E": "Galileo", "C": "BeiDou", "J": "QZSS", "S": "SBAS", "I": "NavIC"}
L1 = {"GPS L1CA", "GLO L1CA", "GAL E1", "BDS B1I", "QZS L1CA", "SBAS L1"}
BANDS = [(10, 30), (30, 60), (60, 91)]
SLICE_S = 120.0


def band_of(signal: str) -> str:
    return "L1" if signal in L1 else signal.split(" ", 1)[1]


def read(caps):
    """Yield (tow_s, {sv: (el, az)}, [(sv, signal, cn0)]) once per second."""
    for cap in caps:
        el = {}
        for line in open(cap, errors="replace"):
            p = line.split(" ", 2)
            if len(p) < 3 or p[1] != "S" or len(p[2]) < 16:
                continue
            h = p[2]
            bid = int(h[10:12] + h[8:10], 16) & 0x1FFF       # block number, before decoding the rest
            if bid not in (sbf.SAT_VISIBILITY, sbf.MEAS_EPOCH):
                continue
            try:
                b = bytes.fromhex(h.strip())
            except ValueError:                               # the line still being written
                continue
            if bid == sbf.SAT_VISIBILITY:
                el = sbf.sat_visibility(b)
            elif bid == sbf.MEAS_EPOCH:
                tow, _ = sbf.tow_wnc(b)
                if tow is None or tow % 1000 or not el:
                    continue
                m = sbf.meas_epoch(b)
                yield tow / 1000, el, [(x["sv"], x["signal"], x["cn0"]) for x in m["meas"]
                                       if x["cn0"] is not None and x["sv"] != "R??"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--mask", type=float, default=10.0)
    ap.add_argument("--json", help="write sky_px.json-style rows here")
    args = ap.parse_args()

    vals = defaultdict(list)                     # (system, band, (lo, hi)) -> C/N0 samples
    slices = defaultdict(list)                   # (slice, system, band, sv) -> [(el, cn0)]
    seconds = 0
    t_first = t_last = None
    for tow, el, meas in read(args.captures):
        seconds += 1
        t_first = tow if t_first is None else t_first
        t_last = tow
        for sv, signal, c in meas:
            e = el.get(sv, (None, None))[0]
            system, band = SYSTEM.get(sv[0], sv[0]), band_of(signal)
            if e is not None and e >= args.mask:
                vals[(system, band, (args.mask, 91))].append(c)
                for lo, hi in BANDS:
                    if lo <= e < hi:
                        vals[(system, band, (lo, hi))].append(c)
            slices[(int(tow // SLICE_S), system, band, int(sv[1:]))].append((e, c))

    hours = (t_last - t_first) / 3600 if seconds else 0
    print("captures: " + ", ".join(Path(c).name for c in args.captures)
          + f"  ({seconds} s with elevations, {hours:.2f} h span)")
    print(f"mosaic-G5 C/N0 on the sky, dB-Hz: median [quartiles] (n channel-seconds); elevation >= {args.mask:g} deg")
    order = ["GPS", "Galileo", "BeiDou", "GLONASS", "QZSS", "SBAS", "NavIC"]
    keys = sorted({(s, b) for s, b, _ in vals}, key=lambda k: (order.index(k[0]) if k[0] in order else 9,
                                                                k[1] != "L1", k[1]))
    for s, b in keys:
        parts = []
        for rng in [(args.mask, 91)] + BANDS:
            v = vals.get((s, b, rng))
            if v and len(v) >= 4:
                q = statistics.quantiles(v, n=4)
                name = "all" if rng == (args.mask, 91) else f"{rng[0]}-{min(rng[1], 90)}"
                parts.append(f"{name}: {statistics.median(v):.1f} [{q[0]:.0f}-{q[2]:.0f}] ({len(v)})")
        if parts:
            print(f"  {s:8s} {b:5s} " + "; ".join(parts))

    if args.json:
        rows = []
        for (k, system, band, sv), xs in sorted(slices.items(), key=lambda kv: (kv[0][1], kv[0][2], kv[0][3], kv[0][0])):
            els = [e for e, _ in xs if e is not None]
            rows.append({"system": system, "band": band, "sv": sv,
                         "el": round(statistics.median(els), 1) if els else None,
                         "cn0": round(statistics.median(c for _, c in xs), 2)})
        Path(args.json).write_text(json.dumps({"capture": ", ".join(Path(c).name for c in args.captures),
                                               "kind": "sbf", "slice_s": SLICE_S, "rows": rows}))
        print(f"wrote {args.json} ({len(rows)} rows)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
