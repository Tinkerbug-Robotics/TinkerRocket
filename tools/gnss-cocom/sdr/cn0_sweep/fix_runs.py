#!/usr/bin/env python3
"""Fix-state runs in a PX1105R traveler capture (0xDF navigation messages, pad600 timing as traveler_summary.py):
every stretch of constant fix mode after T0, with its start, end and length, and any gap between 0xDF messages
longer than MAXGAP s.      fix_runs.py CAPTURE [T0] [MAXGAP]"""
import struct
import sys

TOW0, IGN = 203400.0, 600.0
cap = sys.argv[1]
t_from = float(sys.argv[2]) if len(sys.argv) > 2 else 270.0
maxgap = float(sys.argv[3]) if len(sys.argv) > 3 else 0.35
df = []
for line in open(cap, errors="replace"):
    p = line.split(" ", 2)
    if len(p) < 3 or p[1] != "B":
        continue
    try:
        x = bytes.fromhex(p[2].strip())
    except ValueError:
        continue
    if x[0] == 0xDF and len(x) >= 61:
        t = round(struct.unpack(">d", x[5:13])[0] - TOW0 - IGN, 2)
        if t >= t_from:
            df.append((t, x[2]))
df.sort()
if not df:
    sys.exit("no 0xDF after T0")
runs, start, mode = [], df[0][0], df[0][1]
for (ta, ma), (tb, mb) in zip(df, df[1:]):
    if tb - ta > maxgap:
        print(f"  0xDF gap T+{ta:.2f} .. T+{tb:.2f} ({tb - ta:.2f} s)")
    if mb != mode:
        runs.append((start, ta, mode))
        start, mode = tb, mb
runs.append((start, df[-1][0], mode))
print(f"fix-mode runs from T+{t_from:g} ({len(df)} 0xDF messages):")
for a, b, m in runs:
    print(f"  mode {m}: T+{a:7.2f} .. T+{b:7.2f}  ({b - a:6.2f} s)")
