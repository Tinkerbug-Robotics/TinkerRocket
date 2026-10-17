"""Each apogee voter's first vote on board (NonSensor flags), on the flight's time, against the true apogee."""
import sys
import numpy as np
sys.path.insert(0, ".")
import scorelib as S
sys.path.insert(0, S.WT + "/Data_Analysis")
import plot_flight_data_mini as P
tr = S.Truth()
print(f"true apogee T+{tr.apogee:.3f}, truth pitch < -5 at T+{tr.pitch_vote:.3f}")
print("run   vel      baro     gps      pitch    apogee called (T+ s; margin = true apogee - call)")
import json, os
outj = json.load(open("voters.json")) if os.path.exists("voters.json") else {}
for arg in sys.argv[1:]:
    run, log = arg.split("=")
    base = float(S.read_meta(f"in/{run}.txt")["t_base_s"])
    recs, _s, _c = P.parse_binary_file(log)
    ns = recs["NonSensor"]
    t = S.unwrap_us([r["time_us"] for r in ns]) * 1e-6 + base
    out = []
    for k in ("vel_apogee", "alt_apogee", "gps_apogee", "pitch_apogee", "apogee_flag"):
        f = np.array([bool(r.get(k, False)) for r in ns])
        i = np.flatnonzero(f & (t > 0))
        out.append(t[i[0]] if i.size else np.nan)
    print(f"{run:5s} " + " ".join(f"{x:8.3f}" for x in out) + f"   margin {tr.apogee - out[-1]:+.3f} s")
    outj[run] = dict(zip(("vel", "baro", "gps", "pitch", "apogee"), [None if np.isnan(x) else float(x) for x in out]))
json.dump(outj, open("voters.json", "w"), indent=1)
