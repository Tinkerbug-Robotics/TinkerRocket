#!/usr/bin/env python3
"""Does a log have GNSS in flight? Its on-board raw epochs (0x96) after T+8 s with at least 4 pseudoranges used.

    aided_check.py RUN=OC_LOG [...]
"""
import sys
import numpy as np
sys.path.insert(0, ".")
import scorelib as S
sys.path.insert(0, S.WT + "/Data_Analysis")
import plot_flight_data_mini as P
for arg in sys.argv[1:]:
    run, log = arg.split("=", 1)
    base = float(S.read_meta(f"in/{run}.txt")["t_base_s"])
    recs, _s, _c = P.parse_binary_file(log)
    sh = recs["RawGnssEkf"]
    t = S.unwrap_us([r["time_us"] for r in sh]) * 1e-6 + base
    seen, n4, n_any = set(), 0, 0
    for ti, r in zip(t, sh):
        if ti > 8.0 and r["epoch_tow_s"] not in seen:
            seen.add(r["epoch_tow_s"])
            n_any += 1
            n4 += r["used_pr"] >= 4
    verdict = "GNSS-aided" if n4 >= 5 else "NO GNSS in flight (control)"
    print(f"{run}: {n_any} epochs after T+8 s, {n4} with >= 4 pseudoranges used -> {verdict}; log ends T+{t[-1]:.1f} s")
