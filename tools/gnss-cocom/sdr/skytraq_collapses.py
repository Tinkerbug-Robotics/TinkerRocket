#!/usr/bin/env python3
"""Common-mode lock collapses in SkyTraq captures (PX1105R, PX1125R): 0xE7 epochs where three
or more GPS channels that had frame sync (flag F4) in the previous epoch lose it together, and
the receiver's own fix dropping out for more than 0.3 s (0xDF nav state < 2 between fixes).
Also reports the HackRF underruns from the capture's .hackrf.txt, since an underrun shifts the
whole simulated sky in time and voids the run from that moment.

On the bench (2026-09-26, pad 100-595 s): the PX1105R collapsed 30-36 times per pad, ~11-12 s
apart, in factory power save and once in power normal; the PX1125R ~25-30 times, ~15 s apart,
in both modes. The LC86G shows none. Runs with 30 collapses had 0 underruns.

    ./skytraq_collapses.py T0 T1 CAP...      # host seconds, e.g. 100 595 for a 600 s pad
"""
import statistics as stt
import sys
from pathlib import Path


def underruns(cap):
    """Largest running total hackrf_transfer printed, and the longest gap in bytes."""
    txt = Path(str(cap) + ".hackrf.txt")
    best = (None, None)
    if txt.exists():
        for line in open(txt, errors="replace"):
            p = line.split()
            if "underruns," in p:
                i = p.index("underruns,")
                n, longest = int(p[i - 1]), int(p[i + 2])
                if best[0] is None or n > best[0]:
                    best = (n, longest)
    return best


def main():
    t0, t1 = float(sys.argv[1]), float(sys.argv[2])
    for cap in sys.argv[3:]:
        prev, ev, df = None, [], []
        for line in open(cap, errors="replace"):
            p = line.split(" ", 2)
            if len(p) < 3 or p[1] != "B":
                continue
            h = float(p[0])
            if not t0 <= h <= t1:
                continue
            x = bytes.fromhex(p[2].strip())
            if x[0] == 0xDF and len(x) >= 3:
                df.append((h, x[2]))
            if x[0] != 0xE7 or len(x) < 4:
                continue
            fs = set()
            for i in range(x[3]):
                r = x[4 + 7 * i: 11 + 7 * i]
                if len(r) == 7 and (r[1] & 0x0F) == 0 and r[6] & 4:     # GPS, frame sync
                    fs.add(r[2])
            if prev is not None and len(prev - fs) >= 3:
                ev.append((h, len(prev - fs)))
            prev = fs
        fixes = [h for h, st in df if st >= 2]
        drops = [(a, b - a) for a, b in zip(fixes, fixes[1:]) if b - a > 0.3]
        if fixes and df and df[-1][0] - fixes[-1] > 0.3:
            drops.append((fixes[-1], df[-1][0] - fixes[-1]))                 # lost and never back
        gaps = [b - a for (a, _), (b, _) in zip(ev, ev[1:])]
        n_ur, longest = underruns(cap)
        ur = "no .hackrf.txt" if n_ur is None else f"{n_ur} HackRF underruns (longest {longest} bytes)"
        print(f"{Path(cap).name}\n  collapses {len(ev):3d}, median interval "
              f"{stt.median(gaps) if gaps else 0:4.1f} s; fix dropouts {len(drops):3d}; {ur}")
        if ev:
            print("  collapses at " + ", ".join(f"{h:.0f}({n})" for h, n in ev[:16]))
        if drops:
            print("  dropouts at  " + ", ".join(f"{a:.0f}+{g:.1f}" for a, g in drops[:16]))


if __name__ == "__main__":
    main()
