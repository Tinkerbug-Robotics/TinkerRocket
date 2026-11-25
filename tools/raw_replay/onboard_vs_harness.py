#!/usr/bin/env python3
"""For each run: the on-board raw filter's score (the OC log's own 0x96 records), the harness's score with the
configuration the run was recorded with, and their difference -- the same metrics on the same phases, both
sampled at the on-board records' own times (the harness's nearest row, within 6 ms).

    onboard_vs_harness.py RUNS.json OUT.md
RUNS.json: [{"run": "f3c", "log": OC_LOG, "events": EVENTS (extract.py), "binary": raw_replay, "args": [...],
             "note": "..."}, ...]   (the events file is written first if it does not exist)
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

METRICS = [("nose", "nose-pointing error (deg)", 2), ("roll", "roll error about the nose (deg)", 2),
           ("att_total", "total attitude error (deg)", 2), ("heading", "heading error, diagnostic (deg)", 2),
           ("pitch", "pitch error (deg)", 2), ("vz", "vertical velocity error (m/s)", 2),
           ("pos_h", "horizontal position error (m)", 1), ("pos_v", "vertical position error (m)", 1),
           ("vel", "velocity error, 3-D (m/s)", 2)]


def f3(x, nd):
    if x is None or not np.all(np.isfinite(x)):
        return "-"
    return " / ".join(f"{v:.{nd}f}" for v in x)


def main():
    spec = json.load(open(sys.argv[1]))
    out_md = sys.argv[2]
    tr = S.Truth()
    L = []
    w = L.append
    w("# On-board against the harness, same configuration\n")
    w("Each cell: median / p90 / max over the phase, sampled at the on-board records' own times (10 Hz; the "
      "harness's nearest row within 6 ms), against the truth. Difference = harness less on-board, of each "
      "statistic. Boost = T+0 to burnout, coast = to apogee, descent = apogee to the end of the log (at most the "
      f"landing). Truth: burnout T+{tr.burnout:.2f}, apogee T+{tr.apogee:.2f}, its pitch below -5 deg at "
      f"T+{tr.pitch_vote:.3f}.\n")
    summary = []
    for r in spec:
        ev = r["events"]
        if not os.path.exists(ev):
            subprocess.run([sys.executable, os.path.join(HERE, "extract.py"), r["log"], ev] + r.get("extract_args", []),
                           check=True)
        meta = S.read_meta(ev)
        base = float(meta["t_base_s"])
        csv = os.path.splitext(out_md)[0] + f"_{r['run']}.csv"
        cmd = [r["binary"], "--in", ev, "--out", csv, "--t-base-s", repr(base)] + r["args"]
        p = subprocess.run(cmd, capture_output=True, text=True)
        if p.returncode != 0:
            raise SystemExit(p.stderr)
        recs, _s, _c = P.parse_binary_file(r["log"])
        ob = S.onboard_series(recs, base)
        hs = S.harness_series(csv, base)
        grid = ob["t"][ob["ok"]]
        so = S.score(ob, tr, grid=grid, tol=0.001)
        sh = S.score(hs, tr, grid=grid, tol=0.006)
        dv = S.diff_vs(hs, ob, tr)
        w(f"\n## {r['run']}: {r.get('note', '')}\n")
        w(f"Harness: `{' '.join(r['args'])}`. {p.stderr.strip().splitlines()[-1].split(': ', 1)[1]}\n")
        w("| metric | phase | on-board | harness | difference |")
        w("|---|---|---|---|---|")
        for k, name, nd in METRICS:
            for ph in ("boost", "coast", "descent"):
                a = so["phases"].get(ph, {}).get(k)
                b = sh["phases"].get(ph, {}).get(k)
                dd = tuple(np.subtract(b, a)) if a is not None and b is not None else None
                w(f"| {name} | {ph} | {f3(a, nd)} | {f3(b, nd)} | {f3(dd, nd) if dd else '-'} |")
        po, ph_ = so["pitch_vote_t"], sh["pitch_vote_t"]
        w(f"\nApogee pitch voter (first estimated pitch < -5 deg after burnout, T+ s): on-board records "
          f"{po:.3f} (10 Hz), on-board flight loop {so.get('ns_pitch_cross_t') or float('nan'):.3f} (its own "
          f"voter flag {so.get('ns_pitch_flag_t') or float('nan'):.3f}, apogee called "
          f"{so.get('ns_apogee_t') or float('nan'):.3f}); harness {ph_:.3f}; truth {tr.pitch_vote:.3f}. "
          f"Unhealthy samples: on-board {so['unhealthy_samples']}, harness {sh['unhealthy_samples']}.\n"
          if po is not None and ph_ is not None else "\nApogee pitch voter: not crossed in one of them.\n")
        w("Trajectory difference, harness less on-board at the on-board records (median / p90 / max):\n")
        w("| phase | attitude (deg) | nose (deg) | heading (deg) | velocity (m/s) | vertical velocity (m/s) | "
          "horizontal position (m) | vertical position (m) |")
        w("|---|---|---|---|---|---|---|---|")
        for ph, m in dv.items():
            w(f"| {ph} | {f3(m['att_total'], 2)} | {f3(m['nose'], 2)} | {f3(m['heading'], 2)} | "
              f"{f3(m['vel'], 2)} | {f3(m['vz'], 2)} | {f3(m['pos_h'], 1)} | {f3(m['pos_v'], 1)} |")
        de = S.diff_extra(hs, ob, tr)
        if de:
            w("\nThe filter's own 1-sigmas and IMU biases (0x9B), |harness less on-board| (median / p90 / max):\n")
            names = [(k, n) for k, n in S.EXTRA_DIFF if any(k in v for v in de.values())]
            w("| phase | " + " | ".join(n for _k, n in names) + " |")
            w("|---|" + "---|" * len(names))
            for ph, m in de.items():
                w(f"| {ph} | " + " | ".join(f3(m.get(k), 3 if k.startswith(("ab", "wb")) else 2) for k, _n in names)
                  + " |")
        summary.append({"run": r["run"], "onboard": so, "harness": sh, "diff": dv, "diff_extra": de})
    # the compact view: one row per run, the medians (p90 in brackets) that matter most
    K = [("nose", "coast"), ("nose", "descent"), ("roll", "coast"), ("roll", "descent"), ("pitch", "coast"),
         ("vz", "coast")]
    head = ["| run | " + " | ".join(f"{k} {ph}: on-board / harness / diff" for k, ph in K) +
            " | pitch < -5 deg (T+ s): on-board loop / harness / diff |",
            "|---|" + "---|" * (len(K) + 1)]
    rows = []
    for x in summary:
        cells = []
        for k, ph in K:
            a = x["onboard"]["phases"].get(ph, {}).get(k)
            b = x["harness"]["phases"].get(ph, {}).get(k)
            cells.append("-" if a is None or b is None else
                         f"{a[0]:.2f} [{a[1]:.2f}] / {b[0]:.2f} [{b[1]:.2f}] / {b[0] - a[0]:+.2f} [{b[1] - a[1]:+.2f}]")
        po = x["onboard"].get("ns_pitch_cross_t") or x["onboard"]["pitch_vote_t"]
        ph_ = x["harness"]["pitch_vote_t"]
        cells.append("-" if po is None or ph_ is None else f"{po:.3f} / {ph_:.3f} / {ph_ - po:+.3f}")
        rows.append(f"| {x['run']} | " + " | ".join(cells) + " |")
    i = next(k for k, ln in enumerate(L) if ln.startswith("\n## "))
    L[i:i] = ["\n## Summary: medians [p90], degrees (vz m/s)\n"] + head + rows
    open(out_md, "w").write("\n".join(L) + "\n")
    json.dump(summary, open(os.path.splitext(out_md)[0] + ".json", "w"), indent=1, default=float)
    print("wrote", out_md)


if __name__ == "__main__":
    main()
