#!/usr/bin/env python3
"""Did a transmitter underrun in the boost window spoil a u-blox flight? From the capture's .hackrf.txt: each underrun's
stats line (T = k - 178.2) and the 'longest' gap; from RAWX: valid satellites per second around it and up to ignition,
and the receiver's clock-reset flag (recStat bit 1); from an accuracy NPZ (optional): satellites with |error| > 50 m
between the underrun and T+0.5.    underrun_effect.py CAPTURE [ACC.npz]"""
import re
import struct
import sys

import numpy as np

cap = sys.argv[1]
prev, events = 0, []
for k, x in enumerate((ln for ln in open(cap + ".hackrf.txt", errors="replace") if "MB / " in ln), 1):
    m = re.search(r"(\d+) underruns, longest (\d+) bytes", x)
    if m and int(m.group(1)) > prev:
        events.append((round(k - 178.2, 1), int(m.group(1)) - prev, int(m.group(2))))
        prev = int(m.group(1))
for t, n, lg in events:
    print(f"underrun(s) at T{t:+.1f}: {n} new, longest so far {lg} bytes = {lg / 2 / 18.48e6 * 1e3:.3f} ms")
per, resets = {}, []
for line in open(cap, errors="replace"):
    p = line.split(" ", 2)
    if len(p) < 3 or p[1] != "U" or not p[2].startswith("0215"):
        continue
    try:
        b = bytes.fromhex(p[2].strip()[4:])
    except ValueError:
        continue
    t = struct.unpack_from("<d", b, 0)[0] - 204000.0
    if b[12] & 2:
        resets.append(round(t, 1))
    n = sum(1 for j in range(b[11]) if len(b) >= 48 + 32 * j and b[16 + 32 * j + 30] & 1)
    per[int(np.floor(t))] = n
t0 = int(min(t for t, _, _ in events)) - 3 if events else -30
print("valid measurements, last epoch of each second:",
      " ".join(f"T{s:+d}:{per.get(s, '-')}" for s in range(t0, 2)))
print("clock resets (recStat bit 1) at:", [t for t in resets if t0 <= t <= 5] or "none", f"({len(resets)} in the run)")
if len(sys.argv) > 2:
    z = np.load(sys.argv[2])
    m = (z["t"] >= t0) & (z["t"] <= 0.5) & (np.abs(z["pr"]) > 50)
    bad = sorted({("GEC"[int(s)], int(p)) for s, p in zip(z["sys"][m], z["prn"][m])})
    print("satellites over 50 m off between the underrun and T+0.5:",
          ", ".join(f"{c}{p:02d} (to T{z['t'][m & (z['sys'] == 'GEC'.index(c)) & (z['prn'] == p)].max():+.1f})"
                    for c, p in bad) or "none")
