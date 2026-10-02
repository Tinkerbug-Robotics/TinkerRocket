#!/usr/bin/env python3
"""Which time do the NEO-M8T's measurements belong to? Over the burn below 515 m/s (T+0.3 .. T+6.2, the speed and
acceleration make timing visible), evaluate the truth at the time tag minus DELTA and report the pseudorange spread
(per epoch and system, the median removed) and, separately, the Doppler against the truth range rate LAG earlier.
The pad clock (+2.96 ms on mtr0) would say DELTA = +2.96 ms if the tags were the receiver's raw clock.
    m8t_timing_scan.py CAPTURE [traveler|hotshot]"""
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import m8t_accuracy as M                                                  # noqa: E402
import pr_accuracy as P                                                   # noqa: E402

cap = sys.argv[1]
scen = sys.argv[2] if len(sys.argv) > 2 else "traveler"
csv = P.SDR / "scenarios" / ("traveler_soft25_pad600.csv" if scen == "traveler" else "hotshot_pad600.csv")
t_hi = 6.2 if scen == "traveler" else 2.7
eph = M.pooled_eph()
ep = M.m8t_epochs(cap)
keys = [k for k in sorted(ep) if P.IGN + 0.3 <= ep[k][0] - P.TOW_FILE0 <= P.IGN + t_hi]
print(f"{len(keys)} burn epochs (T+0.3 .. T+{t_hi})")


def spread(A):
    out = {0: [], 1: [], 2: []}
    ut, groups = P.by_epoch(A)
    for g in groups:
        for s in (0, 1, 2):
            gg = g[A[g, 1] == s]
            if len(gg) >= 3:
                r = A[gg, 3]
                out[s].extend(r - np.median(r))
    return {s: math.sqrt(np.mean(np.square(v))) if v else math.nan for s, v in out.items()}


print("pseudorange spread, m (GPS / Galileo / BeiDou), truth at tag minus DELTA:")
for d_ms in (-16, -12, -10, -8, -6, -4, -2, -1, 0, 1, 2, 3, 4, 6):
    M.SHIFT = (np.array([0.0, 5000.0]), np.array([d_ms / 1e3, d_ms / 1e3]))
    A = P.rows_for(M.ShiftedTruth(str(csv)), eph, ep, keys, {}, 0.0)
    s = spread(A)
    print(f"  DELTA {d_ms:+3d} ms: {s[0]:.3f} / {s[1]:.3f} / {s[2]:.3f}")
M.SHIFT = None
print("Doppler: range-rate error RMS, m/s (epoch median removed), truth LAG earlier:")
for lag_ms in (-20, -10, -5, 0, 5, 10, 20, 30, 40, 50, 60, 80):
    A = P.rows_for(M.ShiftedTruth(str(csv)), eph, ep, keys, {}, lag_ms / 1e3)
    ut, groups = P.by_epoch(A)
    e = []
    for g in groups:
        y = A[g, 4]
        f = np.isfinite(y)
        if f.sum() >= 3:
            e.extend(y[f] - np.median(y[f]))
    print(f"  LAG {lag_ms:3d} ms: {math.sqrt(np.mean(np.square(e))):.3f}")
