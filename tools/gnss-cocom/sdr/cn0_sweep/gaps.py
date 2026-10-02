#!/usr/bin/env python3
"""Gaps in a capture's raw measurement output (0xE5 epochs with none) after ignition, with what each system had
in the last epoch before the gap, and the replay underruns (from the .hackrf.txt beside it) for comparison.

    gaps.py CAPTURE SAMPLE_RATE_HZ [MIN_GAP_S] [PAD_S]     (PAD_S: the file's pad before ignition, default 600)
"""
import re
import struct
import sys

cap, fs = sys.argv[1], float(sys.argv[2])
min_gap = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
PAD = float(sys.argv[4]) if len(sys.argv) > 4 else 600.0
TOW0, IGN = 203400.0, 600.0
G = {0: "GPS", 3: "GAL", 5: "BDS"}
ep = []
for line in open(cap, errors="replace"):
    p = line.split(" ", 2)
    if len(p) < 3 or p[1] != "B":
        continue
    try:
        x = bytes.fromhex(p[2].strip())
    except ValueError:
        continue
    if x[0] == 0xE5 and len(x) >= 14:
        t = struct.unpack(">I", x[5:9])[0] / 1000.0 - TOW0 - IGN
        n = {"GPS": 0, "GAL": 0, "BDS": 0}
        for j in range(x[13]):
            r = x[14 + 31 * j: 14 + 31 * (j + 1)]
            if len(r) == 31 and (r[0] & 0x0F) in G and (r[0] >> 4) in (0, 1):
                n[G[r[0] & 0x0F]] += 1
        ep.append((t, n))
ep.sort(key=lambda e: e[0])
live = [(t, n) for t, n in ep if sum(n.values()) > 0]
print(f"{cap.split('/')[-1]}")
for (ta, na), (tb, nb) in zip(live, live[1:]):
    if tb - ta >= min_gap and ta > -400:
        print(f"  no raw measurements T{ta:+8.2f} .. T{tb:+8.2f} s ({tb - ta:6.1f} s); last epoch before: "
              f"{', '.join(f'{k} {v}' for k, v in na.items() if v)}; first after: "
              f"{', '.join(f'{k} {v}' for k, v in nb.items() if v)}")
rows = []
for s in open(cap + ".hackrf.txt", errors="replace"):
    m = re.search(r"(\d+) underruns, longest (\d+) bytes", s)
    if m:
        rows.append((int(m.group(1)), int(m.group(2))))
prev = 0
for k, (n, longest) in enumerate(rows):
    if n > prev:
        print(f"  underrun in replay second {k} (T{k - PAD:+.0f} s): +{n - prev}, longest so far "
              f"{longest / 2 / fs * 1e3:.1f} ms")
    prev = n
