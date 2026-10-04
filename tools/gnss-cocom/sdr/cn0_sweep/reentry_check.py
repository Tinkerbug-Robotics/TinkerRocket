#!/usr/bin/env python3
"""What stops the raw output near T+280? For one capture: every underrun in its .hackrf.txt placed on the
receiver's own time (host time -> GPS TOW through the 0xDF solutions; the TX starts at the host time the capture's
header gives), the per-system measurement counts and GPS C/N0 in 0.5 s steps over a window, and the truth's speed,
acceleration and jerk there (the re-entry deceleration).

    reentry_check.py CAPTURE SAMPLE_RATE_HZ T0 T1
"""
import math
import re
import struct
import sys

import numpy as np

sys.path.insert(0, sys.path[0])
from pr_accuracy import SignalSimTruth, SDR  # noqa: E402

cap, fs, w0, w1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
TOW_IGN = 204000.0
G = {0: "GPS", 3: "GAL", 5: "BDS"}
host_tow, ep, tx_launch = [], [], None
for line in open(cap, errors="replace"):
    p = line.split(" ", 2)
    if len(p) < 3:
        continue
    if p[1] == "#" and "TX launched" in line:
        tx_launch = float(re.search(r"TX launched ([0-9.]+) s", line).group(1))
        continue
    if p[1] != "B":
        continue
    try:
        th, x = float(p[0]), bytes.fromhex(p[2].strip())
    except ValueError:
        continue
    if x[0] == 0xDF and len(x) >= 13 and x[2] >= 1:
        tow = struct.unpack(">d", x[5:13])[0]
        if tow > 1000:
            host_tow.append((th, tow))
    elif x[0] == 0xE5 and len(x) >= 14:
        tow = struct.unpack(">I", x[5:9])[0] / 1000.0
        n, cn = {"GPS": 0, "GAL": 0, "BDS": 0}, []
        for j in range(x[13]):
            r = x[14 + 31 * j: 14 + 31 * (j + 1)]
            if len(r) == 31 and (r[0] & 0x0F) in G and (r[0] >> 4) in (0, 1):
                n[G[r[0] & 0x0F]] += 1
                if (r[0] & 0x0F) == 0:
                    cn.append(r[3])
        ep.append((tow - TOW_IGN, n, float(np.mean(cn)) if cn else math.nan, th))
h = np.array(host_tow)
# host -> TOW: robust line through the solutions (the receiver's time is the signal's)
A = np.vstack([h[:, 0], np.ones(len(h))]).T
coef, *_ = np.linalg.lstsq(A, h[:, 1], rcond=None)
res = h[:, 1] - A @ coef
keep = np.abs(res - np.median(res)) < 0.5
coef, *_ = np.linalg.lstsq(A[keep], h[keep, 1], rcond=None)
tow_of = lambda t: coef[0] * t + coef[1]           # noqa: E731
print(f"{cap.split('/')[-1]}")
print(f"host->TOW slope {coef[0]:.6f}; TX launched at host {tx_launch:.3f} s = T{tow_of(tx_launch) - TOW_IGN:+.2f} s")
rows = []
for s in open(cap + ".hackrf.txt", errors="replace"):
    m = re.search(r"(\d+) underruns, longest (\d+) bytes", s)
    if m:
        rows.append((int(m.group(1)), int(m.group(2))))
prev = 0
for k, (n, longest) in enumerate(rows):
    if n > prev:
        a, b = tx_launch + k, tx_launch + k + 1.0
        print(f"  underrun in stats line {k}: host {a:.1f}..{b:.1f} s = T{tow_of(a) - TOW_IGN:+.1f}..T{tow_of(b) - TOW_IGN:+.1f} s "
              f"(+{n - prev}; longest so far {longest / 2 / fs * 1e3:.1f} ms)")
    prev = n
tr = SignalSimTruth(str(SDR / "scenarios" / "traveler_soft25_pad600.csv"))
print(f"\n{'T+':>7} {'GPS':>4} {'GAL':>4} {'BDS':>4} {'GPS C/N0':>9}   {'speed':>7} {'accel':>8} {'jerk':>8}   (m/s, m/s^2, m/s^3)")
E = [e for e in ep if w0 <= e[0] < w1]
for a in np.arange(w0, w1, 0.5):
    s = [e for e in E if a <= e[0] < a + 0.5]
    t = 600.0 + a + 0.25
    v = tr.vup(t)
    acc = (tr.vup(t + 0.1) - tr.vup(t - 0.1)) / 0.2
    jerk = ((tr.vup(t + 0.6) - tr.vup(t + 0.4)) - (tr.vup(t - 0.4) - tr.vup(t - 0.6))) / 0.2 / 1.0
    if s:
        mx = {k: max(e[1][k] for e in s) for k in ("GPS", "GAL", "BDS")}
        cn = np.nanmean([e[2] for e in s]) if any(np.isfinite(e[2]) for e in s) else math.nan
        print(f"{a:+7.1f} {mx['GPS']:4d} {mx['GAL']:4d} {mx['BDS']:4d} {cn:9.1f}   {abs(v):7.0f} {acc:+8.1f} {jerk:+8.1f}")
    else:
        print(f"{a:+7.1f} {'--':>4} {'--':>4} {'--':>4} {'':>9}   {abs(v):7.0f} {acc:+8.1f} {jerk:+8.1f}   no raw output")
