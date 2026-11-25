#!/usr/bin/env python3
"""The harness against the logs' own on-board raw filter: each log replayed with the configuration it was recorded
with, its trajectory differenced with the on-board records (0x96) and both scored against the truth.

    validate.py RUNS.json OUT_PREFIX
RUNS.json: [{"run": "r21", "log": ..., "events": ..., "binary": ..., "args": [...]}, ...]
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402

sys.path.insert(0, os.path.join(S.WT, "Data_Analysis"))
import plot_flight_data_mini as P  # noqa: E402


def f3(x, nd=2):
    return "/".join(("-" if not np.isfinite(v) else f"{v:.{nd}f}") for v in x)


def main():
    spec = json.load(open(sys.argv[1]))
    prefix = sys.argv[2]
    tr = S.Truth()
    results = []
    logs = {}
    for r in spec:
        meta = S.read_meta(r["events"])
        base = float(meta["t_base_s"])
        csv = f"{prefix}_{r['run']}_{r.get('tag', 'rec')}.csv"
        cmd = [r["binary"], "--in", r["events"], "--out", csv, "--t-base-s", repr(base)] + r["args"]
        p = subprocess.run(cmd, capture_output=True, text=True)
        if p.returncode != 0:
            print(p.stderr)
            raise SystemExit(f"{r['run']}: the harness failed")
        summary = p.stderr.strip().splitlines()[-1]
        if r["log"] not in logs:
            recs, _s, _c = P.parse_binary_file(r["log"])
            logs[r["log"]] = S.onboard_series(recs, base)
        ob = logs[r["log"]]
        hs = S.harness_series(csv, base)
        d = S.diff_vs(hs, ob, tr)
        sc_h = S.score(hs, tr)
        sc_o = S.score(ob, tr)
        results.append({"run": r["run"], "tag": r.get("tag", "rec"), "args": r["args"], "diff": d,
                        "score_harness": sc_h, "score_onboard": sc_o, "summary": summary, "csv": csv})
        print(f"\n=== {r['run']} [{r.get('tag', 'rec')}] {' '.join(r['args'])}\n    {summary}")
        print("  harness less on-board  n    att total      nose          heading        vel(m/s)       "
              "vz(m/s)        pos_h(m)       pos_v(m)")
        for ph, m in d.items():
            print(f"  {ph:8s}            {m['n']:4d}  {f3(m['att_total'])}  {f3(m['nose'])}  {f3(m['heading'])}  "
                  f"{f3(m['vel'])}  {f3(m['vz'])}  {f3(m['pos_h'], 1)}  {f3(m['pos_v'], 1)}")
        print("  against the truth (median/p90/max): nose, heading, total attitude")
        for ph in sc_o["phases"]:
            a, b = sc_o["phases"][ph], sc_h["phases"].get(ph)
            if b is None:
                continue
            print(f"    {ph:8s} on-board nose {f3(a['nose'])} hdg {f3(a['heading'])} tot {f3(a['att_total'])} | "
                  f"harness nose {f3(b['nose'])} hdg {f3(b['heading'])} tot {f3(b['att_total'])}")
        print(f"    pitch < -5 deg: on-board {sc_o['pitch_vote_t']} (loop {sc_o.get('ns_pitch_cross_t')}, voter "
              f"{sc_o.get('ns_pitch_flag_t')}), harness {sc_h['pitch_vote_t']}, truth {tr.pitch_vote}")
    json.dump(results, open(prefix + "_validation.json", "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
