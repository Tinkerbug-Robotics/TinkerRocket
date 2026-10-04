#!/usr/bin/env python3
"""Same-number satellite pairs on a SkyTraq receiver (PX1105R) on the real sky: whenever a GPS satellite and
a BeiDou (or Galileo) satellite with the same number are both above the mask, does the receiver give both a
channel? Works from the receiver's own 0xE8 (satellites it knows are up, with elevation) and 0xE7 (its
channels, with C/N0 and lock status), so it needs no ephemeris and no position.

    ./pair_watch.py CAPTURE [--mask DEG] [--window S]

For each window (default 300 s), for every number shared by two constellations above the mask: the share of
0xE7 epochs in which each member held a channel with C/N0 > 0. A pair "split" when one member is tracked
most of the window and the other almost never.
"""
import sys

G = {0: "GPS", 1: "SBAS", 2: "GLONASS", 3: "Galileo", 4: "QZSS", 5: "BeiDou", 6: "NavIC"}
args = sys.argv[1:]
cap = args[0]
MASK = float(args[args.index("--mask") + 1]) if "--mask" in args else 10.0
WIN = float(args[args.index("--window") + 1]) if "--window" in args else 300.0


def frames(path):
    for line in open(path, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "B" or not p[2].startswith(("e7", "e8")):
            continue
        try:
            yield float(p[0]), bytes.fromhex(p[2].strip())
        except ValueError:
            continue


up, trk = [], []          # (t, {(sys, sv): elev}), (t, {(sys, sv) with a channel and C/N0 > 0})
for t, x in frames(cap):
    if x[0] == 0xE8 and len(x) >= 4:
        d = {}
        for j in range(x[3]):
            b = x[4 + 6 * j: 4 + 6 * (j + 1)]
            if len(b) == 6:
                el = int.from_bytes(b[2:4], "big", signed=True)
                d[(G.get(b[0] & 0x0F, "?"), b[1])] = el
        up.append((t, d))
    elif x[0] == 0xE7 and len(x) >= 4:
        s = set()
        for j in range(x[3]):
            b = x[4 + 7 * j: 4 + 7 * (j + 1)]
            if len(b) == 7 and (b[1] >> 4) in (0, 1) and int.from_bytes(b[5:6], "big", signed=True) > 0:
                s.add((G.get(b[1] & 0x0F, "?"), b[2]))       # L1-band signal on a channel, C/N0 > 0
        trk.append((t, s))
if not up or not trk:
    sys.exit("no 0xE7/0xE8 in the capture")

t0, t1 = min(up[0][0], trk[0][0]), max(up[-1][0], trk[-1][0])
print(f"{cap.split('/')[-1]}: {(t1 - t0) / 60:.0f} min, {len(up)} 0xE8 and {len(trk)} 0xE7 epochs; "
      f"mask {MASK:.0f} deg, windows of {WIN:.0f} s")
split_total, both_total = 0, 0
w = t0
while w < t1:
    U = [d for t, d in up if w <= t < w + WIN]
    T = [s for t, s in trk if w <= t < w + WIN]
    if U and T:
        last = U[-1]
        above = {k: e for k, e in last.items() if e >= MASK}
        by_sv = {}
        for (sy, sv), e in above.items():
            by_sv.setdefault(sv, []).append((sy, e))
        lines = []
        for sv, members in sorted(by_sv.items()):
            systems = {sy for sy, _ in members}
            if "GPS" not in systems or not ({"BeiDou", "Galileo"} & systems):
                continue
            share = {sy: sum(1 for s in T if (sy, sv) in s) / len(T) for sy, _ in members}
            el = {sy: e for sy, e in members}
            tag = ""
            if "BeiDou" in systems:
                hi = [sy for sy in ("GPS", "BeiDou") if share[sy] >= 0.7]
                lo = [sy for sy in ("GPS", "BeiDou") if share[sy] <= 0.1]
                if len(hi) == 1 and len(lo) == 1:
                    tag, split_total = f"  <- SPLIT: only {hi[0]}", split_total + 1
                elif len(hi) == 2:
                    tag, both_total = "  <- both tracked", both_total + 1
            lines.append(f"    {sv:2d}: " + ", ".join(f"{sy} el {el[sy]:2d} tracked {100 * share[sy]:3.0f}%"
                                                    for sy, _ in sorted(members)) + tag)
        n_gps = sum(1 for (sy, _), e in above.items() if sy == "GPS")
        n_bds = sum(1 for (sy, _), e in above.items() if sy == "BeiDou")
        n_trk = max(len(s) for s in T)
        print(f"  t={(w - t0) / 60:5.1f} min: up GPS {n_gps}, BeiDou {n_bds}; most on L1 channels {n_trk}; "
              f"shared numbers: {len(lines) if lines else 'none'}")
        for ln in lines:
            print(ln)
    w += WIN
print(f"GPS/BeiDou pairs: {split_total} window(s) split (one member only), {both_total} with both tracked")
