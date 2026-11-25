"""Where the driver's clock line puts an MSM7 epoch against the arrival of the frame that closed it (the last MSM
message of the epoch): line - arrival, ms. For logs with no clock line, the epoch's MCU time is its arrival plus this."""
import os
import sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Data_Analysis"))
import plot_flight_data_mini as P
import extract as X

def msm_type_tow(fr):
    p = fr[3:]
    bits = int.from_bytes(p[:8], "big")
    mt = bits >> 52
    tow = (bits >> 10) & ((1 << 30) - 1)
    mm = (p[6] >> 1) & 1 if False else None
    return mt, tow

allv = []
for path in sys.argv[1:]:
    recs, _s, _c = P.parse_binary_file(path)
    ck = [r for r in recs["GnssClock"] if r["valid"]]
    gn = [r for r in recs["GNSS"] if r["year"] >= 2020]
    ct = np.array([r["time_us"] for r in ck])
    frames, _ = P.join_gnss_rtcm(recs["GNSS_RTCM"])
    wk = X.gps_of_fix(gn[0])[0]
    last = {}
    for t, fr in frames:
        mt, tow = msm_type_tow(fr)
        if mt in (1077, 1097, 1127):
            g = tow + (14000 if mt == 1127 else 0)
            last[g] = t          # the epoch's last MSM frame (BeiDou comes last)
    v = []
    for g, t in sorted(last.items()):
        k = np.searchsorted(ct, t, side="right") - 1
        if k < 0: continue
        mcu = P.gnss_clock_mcu_us(ck[k], wk * 604800 + g * 1e-3)
        v.append((mcu - t) / 1000.0)
    v = np.array(v); allv += list(v)
    print(path.split("/")[-1], f"n {len(v)} line - last MSM arrival (ms): median {np.median(v):.1f} p10 {np.percentile(v,10):.1f} p90 {np.percentile(v,90):.1f} min {v.min():.1f} max {v.max():.1f}")
print("all", f"median {np.median(allv):.1f}")
