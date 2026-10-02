#!/usr/bin/env python3
"""Per NEO-M8T run: each satellite's pseudorange and range-rate error from m8t_accuracy.py over the burn up to the raw
cut-off (hotshot T+0..2.9, traveler T+0..6.3), with its elevation and the line-of-sight Doppler rate it reached
(from the boost-chart JSON), sorted by that rate; then the run's clock steps and transmitter underruns.
    m8t_burn_sats.py"""
import json
import math
import os
import re
from pathlib import Path

import numpy as np

S = Path(__file__).resolve().parent / "data"
W = Path(__file__).resolve().parent / "work"
CAP = Path(os.environ.get("CAPTURES", str(Path(__file__).resolve().parents[1] / "captures")))
SYS = "GEC"
RUNS = [("mhs12", "+12 dB", "m8t_boost.json", 2.9), ("mhs6", "+6 dB", "m8t_boost.json", 2.9),
        ("mhs0", "0 dB", "m8t_boost.json", 2.9), ("mhsn6", "-6 dB", "m8t_boost.json", 2.9),
        ("mtr12", "+12 dB", "m8t_trav_boost.json", 6.3), ("mtr6", "+6 dB", "m8t_trav_boost.json", 6.3),
        ("mtr0", "0 dB", "m8t_trav_boost.json", 6.3), ("mtrn6b", "-6 dB", "m8t_trav_boost.json", 6.3)]
for tag, label, bj, cut in RUNS:
    z = np.load(W / f"m8tacc_{tag}.npz")
    t, s, prn, pr, rr = z["t"], z["sys"].astype(int), z["prn"].astype(int), z["pr"], z["rr"]
    recs = {(q["sys"], q["prn"]): q for r in json.load(open(S / bj)) if r["label"] == label for q in r["recs"]}
    rows = []
    for k in sorted({(int(a), int(b)) for a, b in zip(s, prn)}):
        m = (s == k[0]) & (prn == k[1]) & (t >= 0) & (t <= cut)
        if m.sum() < 3:
            continue
        q = recs.get((SYS[k[0]], k[1]), {})
        e, f = pr[m], rr[m][np.isfinite(rr[m])]
        rows.append((abs(q.get("rate", 0.0)), f"{SYS[k[0]]}{k[1]:02d}", q.get("el", math.nan), int(m.sum()),
                     math.sqrt(np.mean(e ** 2)), float(np.max(np.abs(e))),
                     math.sqrt(np.mean(f ** 2)) if len(f) else math.nan, q.get("lost", None)))
    rows.sort(reverse=True)
    print(f"##### {tag} ({'hotshot' if 'mhs' in tag else 'traveler'} {label}), T+0..{cut}: sat el rate(end) n "
          f"PR-RMS PR-max RR-RMS lost")
    for rate, name, el, n, rms, mx, rrr, lost in rows:
        print(f"  {name} {el:4.0f} deg {rate:6.0f} Hz/s  n {n:3d}  PR {rms:7.1f} m (max {mx:7.1f})  RR {rrr:6.2f} m/s"
              f"{'  LOST' if lost else ''}")
    out = ((W / f"m8t_runs_{tag}.out").read_text() if (W / f"m8t_runs_{tag}.out").exists() else "")
    steps = re.search(r"receiver clock: .*", out)
    print(f"  {steps.group(0) if steps else ''}")
    hk = next(CAP.glob(f"neo_m8t_wide{tag}_*.log.hackrf.txt"), None)
    if hk:
        und, prev = [], 0
        for i, line in enumerate(l for l in open(hk, errors="replace") if "MB/second" in l):
            mm = re.search(r"(\d+) underruns", line)
            if mm and int(mm.group(1)) > prev:
                und.append(f"line {i + 1} (~T{i + 1 - 178.2:+.0f})")
                prev = int(mm.group(1))
        print(f"  underruns: {', '.join(und) if und else 'none'}")
