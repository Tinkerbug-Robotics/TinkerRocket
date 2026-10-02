#!/usr/bin/env python3
"""Where in a run the transmitter's underrun count went up: each -B stats line k (one per second after launch) whose
cumulative 'N underruns' grew, with the line's text trimmed.        underrun_lines.py ERRFILE"""
import re
import sys

prev, k = 0, 0
lines = open(sys.argv[1], errors="replace").read().splitlines()
stats = [ln for ln in lines if "MB / " in ln]
for k, ln in enumerate(stats, 1):
    m = re.search(r"(\d+) underruns, longest (\d+) bytes", ln)
    if not m:
        continue
    n = int(m.group(1))
    if n > prev:
        print(f"  stats line {k:4d}: underruns {prev} -> {n}, longest so far {int(m.group(2))} bytes | {ln[:60]}")
    prev = n
print(f"  {len(stats)} stats lines, final underrun count {prev}")
