"""How the GNSS records' stamps (time_us at publishFromParser) sit against the driver's clock line (0x9D): the
offset a line fitted to the records' lower envelope has to be moved by, for logs without 0x9D."""
import os
import sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Data_Analysis"))
import plot_flight_data_mini as P
import extract as X

for path in sys.argv[1:]:
    recs, _s, _c = P.parse_binary_file(path)
    ck = [r for r in recs["GnssClock"] if r["valid"]]
    gn = [r for r in recs["GNSS"] if r["year"] >= 2020 and r["fix_mode"] >= 2]
    if not ck or not gn:
        print(path.split("/")[-1], "no clock or fixes", len(ck), len(gn)); continue
    ct = np.array([r["time_us"] for r in ck], dtype=np.int64)
    res, tt = [], []
    for r in gn:
        wk, tow = X.gps_of_fix(r)
        g = wk * 604800 + tow * 1e-3
        k = np.searchsorted(ct, r["time_us"], side="right") - 1
        if k < 0: continue
        mcu = P.gnss_clock_mcu_us(ck[k], g)
        res.append(r["time_us"] - mcu); tt.append(r["time_us"])
    res = np.array(res) * 1e-3; tt = np.array(tt) * 1e-6
    # lower-envelope line through the records alone: per-second minima, robust line
    sec = np.floor(tt).astype(int)
    print(path.split("/")[-1], f"n {len(res)} stamp - line (ms): min {res.min():.2f} p5 {np.percentile(res,5):.2f} "
          f"p50 {np.median(res):.2f} p95 {np.percentile(res,95):.2f}")
