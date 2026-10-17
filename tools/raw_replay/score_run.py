#!/usr/bin/env python3
"""Score one log's on-board raw filter (0x96) and any harness CSVs of it, the same way.

    score_run.py EVENTS.txt OC_LOG [label=RUN.csv ...]
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402

sys.path.insert(0, os.path.join(S.WT, "Data_Analysis"))
import plot_flight_data_mini as P  # noqa: E402


def fmt3(x, nd=2):
    return "/".join(("-" if not np.isfinite(v) else f"{v:.{nd}f}") for v in x)


def main():
    ev, log = sys.argv[1], sys.argv[2]
    meta = S.read_meta(ev)
    base = float(meta["t_base_s"])
    tr = S.Truth()
    recs, _s, _c = P.parse_binary_file(log)
    runs = [("on-board", S.onboard_series(recs, base))]
    for a in sys.argv[3:]:
        lab, path = a.split("=", 1)
        runs.append((lab, S.harness_series(path, base)))
    print(f"truth: burnout {tr.burnout:.2f} s, apogee {tr.apogee:.2f} s, landed {tr.landed:.2f} s, "
          f"truth pitch < -5 deg at {tr.pitch_vote}")
    out = {}
    for lab, ser in runs:
        r = S.score(ser, tr)
        out[lab] = r
        print(f"\n== {lab}: {ser['t'][0]:+.2f} .. {ser['t'][-1]:+.2f} s, pitch < -5 at {r['pitch_vote_t']}, "
              f"unhealthy {r['unhealthy_samples']}/{r['samples']}" +
              (f", NonSensor pitch cross {r.get('ns_pitch_cross_t')}, voter flag {r.get('ns_pitch_flag_t')}, "
               f"apogee {r.get('ns_apogee_t')}" if 'ns_t' in ser else ""))
        print("  phase     n   total        nose         roll          heading       pitch        vz(m/s)      "
              "pos_h(m)     pos_v(m)     vel(m/s)")
        for ph, m in r["phases"].items():
            print(f"  {ph:8s} {m['n']:3d}  {fmt3(m['att_total'])}  {fmt3(m['nose'])}  {fmt3(m['roll'])}  "
                  f"{fmt3(m['heading'])}  {fmt3(m['pitch'])}  {fmt3(m['vz'])}  {fmt3(m['pos_h'], 1)}  "
                  f"{fmt3(m['pos_v'], 1)}  {fmt3(m['vel'])}")
    return out


if __name__ == "__main__":
    main()
