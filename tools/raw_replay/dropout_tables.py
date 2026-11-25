#!/usr/bin/env python3
"""dropouts.md from dropout_sweep.py: for each configuration, how often it ends worse than unaided after a boost
dropout (the acceptance criterion), and what it buys when boost GNSS is good.

    dropout_tables.py SWEEP_DIR OUT.md [--imu cal] [--tol 1.0]

"Worse than unaided": in any phase (boost, coast, descent) its median |nose| or |roll| is more than tol deg above
unaided's on the same log, scenario and magnetometer error, or its heading, roll or nose in the last 2 s is.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

NAME = {"none": "no safeguard", "a4": "a4: inconsistent boost epoch dropped", "b2": "b2: heading step <= 5 deg",
        "a4b2": "a4 + b2", "b2c": "b2 + c (chi2 3.84 above 3 g)", "a4b2c": "a4 + b2 + c",
        "a5": "a5: as a4, threshold max(5 m/s, 3 sigma_v)", "a5b2": "a5 + b2", "a4c": "a4 + c",
        "a6": "a6: boost epoch dropped when the gate refuses most of its range rates", "a6b2": "a6 + b2",
        "a6h": "a6h: ... half or more of them", "a6hb2": "a6h + b2"}


def worse(r, u, tol):
    """The ways r ends worse than unaided's u."""
    out = []
    for ph in ("boost", "coast", "descent"):
        if ph in r["sum"] and ph in u["sum"]:
            for k in ("nose", "roll"):
                if r["sum"][ph][k][0] > u["sum"][ph][k][0] + tol:
                    out.append(f"{ph} {k} {r['sum'][ph][k][0]:.1f} vs {u['sum'][ph][k][0]:.1f}")
    if "end" in r["sum"] and "end" in u["sum"]:
        for k in ("heading", "roll", "nose"):
            if r["sum"]["end"][k] > u["sum"]["end"][k] + tol:
                out.append(f"end {k} {r['sum']['end'][k]:.1f} vs {u['sum']['end'][k]:.1f}")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sweep", help="sweep directory; several comma-separated, a later one's runs replacing an "
                                  "earlier one's of the same log, scenario, configuration and magnetometer error")
    ap.add_argument("out")
    ap.add_argument("--imu", default="cal")
    ap.add_argument("--tol", type=float, default=1.0)
    ap.add_argument("--lost", default="c30", help="logs whose receiver lost the sky in boost: as recorded, a real "
                                                 "dropout")
    a = ap.parse_args()
    byk = {}
    for sw in a.sweep.split(","):
        for f in sorted(glob.glob(os.path.join(sw, a.imu, "*", "dropout.json"))):
            for r in json.load(open(f))["results"]:
                byk[(r["run"], r["scn"], r["cfg"], r["mag_err"])] = r
    res = list(byk.values())
    order = ["H30_none", "H30_a4", "H30_a5", "H30_a6", "H30_a6h", "H30_b2", "H30_a4b2", "H30_a5b2", "H30_a6b2", "H30_a6hb2", "H30_b2c",
             "H30_a4c", "H30_a4b2c", "H10_none", "H10_a4", "H10_a5", "H10_a6", "H10_a6h", "H10_b2", "H10_a4b2", "H10_a5b2",
             "H10_a6b2", "H10_a6hb2", "H10_b2c", "H10_a4c", "H10_a4b2c",
             "A30_none", "C30_none", "U_ref"]
    res.sort(key=lambda r: (order.index(r["cfg"]) if r["cfg"] in order else 99, r["run"], r["scn"], r["mag_err"]))
    if not res:
        raise SystemExit("no results")
    logs = sorted({r["run"] for r in res})
    U = {(r["run"], r["scn"], r["mag_err"]): r for r in res if r["cfg"] == "U_ref"}
    cfgs = [c for c in dict.fromkeys(r["cfg"] for r in res) if c != "U_ref"]
    lost = set(x for x in a.lost.split(",") if x)

    def klass(r):
        s = r["scn"]
        if s == "real":
            return "lost" if r["run"] in lost else "real"
        return "corrupt+cut" if s.startswith("bad") else "cut"
    L = [f"# Boost dropouts against the safeguards ({'calibrated pad, 0 mg' if a.imu == 'cal' else 'uncalibrated pad'})\n",
         f"Logs: {', '.join(logs)}. Scenarios per log: as recorded; every epoch cut from T+0.5, 1.5 or 3 s until T+8, "
         "T+12 or the end; one epoch corrupted like c30's at T+1 s (two draws) then everything cut from T+1.5 s until "
         "T+8, T+12 or the end. Magnetometer 0, +10, +40 deg. H = C until the apogee call, then A; 30 / 10 = the launch "
         "heading prior. Safeguards in boost (IMU peak since the last epoch above 2 g): a4 = an epoch whose Dopplers "
         "point to a velocity correction over 5 m/s is dropped whole; b2 = no satellite may turn the heading more than "
         "5 deg in the epoch; c = innovation gate chi2 3.84 (2 sigma) above 3 g.\n",
         f"**Worse than unaided** (the acceptance criterion): in some phase the median nose or roll error is more than "
         f"{a.tol:g} deg above unaided's on the same log, scenario and magnetometer error, or the heading, roll or nose "
         "in the last 2 s is.\n",
         "\n## Runs that end worse than unaided\n",
         f"Counts: flagged / runs. As recorded: the GNSS-aided logs as flown (boost GNSS good). Lost in boost: "
         f"{', '.join(sorted(lost)) or '-'} as flown (the receiver gave a corrupted epoch at T+1.0 s and nothing "
         "after). Cut / corrupted then cut: the synthesized dropouts on the aided logs.\n",
         "| configuration | safeguards | as recorded (GNSS good) | lost in boost (real) | cut | corrupted then cut | "
         "all dropouts | all |",
         "|---|---|---|---|---|---|---|---|"]
    flagged = {}
    for c in cfgs:
        cnt = {"real": [0, 0], "lost": [0, 0], "cut": [0, 0], "corrupt+cut": [0, 0]}
        lst = []
        for r in res:
            if r["cfg"] != c:
                continue
            u = U.get((r["run"], r["scn"], r["mag_err"]))
            if u is None:
                continue
            k = klass(r)
            cnt[k][1] += 1
            w = worse(r, u, a.tol)
            if w:
                cnt[k][0] += 1
                lst.append(f"{r['run']} {r['scn']} {r['mag_err']:+.0f}: " + "; ".join(w))
        flagged[c] = lst
        g, sfx = c.split("_")
        tot = [sum(v[0] for v in cnt.values()), sum(v[1] for v in cnt.values())]
        dro = [tot[0] - cnt["real"][0], tot[1] - cnt["real"][1]]
        L.append(f"| {g} | {NAME.get(sfx, sfx)} | {cnt['real'][0]} / {cnt['real'][1]} | {cnt['lost'][0]} / "
                 f"{cnt['lost'][1]} | {cnt['cut'][0]} / {cnt['cut'][1]} | {cnt['corrupt+cut'][0]} / "
                 f"{cnt['corrupt+cut'][1]} | {dro[0]} / {dro[1]} | {tot[0]} / {tot[1]} |")

    # what the configurations buy with good boost GNSS (as recorded), and what dropouts cost
    for title, sel in (("as recorded (boost GNSS good)", lambda r: klass(r) == "real"),
                       ("with a boost dropout (lost in boost as flown, cut, or corrupted then cut)",
                        lambda r: klass(r) != "real")):
        L.append(f"\n## Attitude {title}: median of the per-run medians [worst run], deg\n")
        L.append("| configuration | safeguards | mag error | nose coast | nose descent | roll boost | roll coast | "
                 "roll descent | heading at the end | pitch vote (T+ s) |")
        L.append("|---|---|---|---|---|---|---|---|---|---|")
        for c in cfgs + ["U_ref"]:
            for m in (0.0, 10.0, 40.0):
                rs = [r for r in res if r["cfg"] == c and r["mag_err"] == m and sel(r)]
                if not rs:
                    continue

                def agg(ph, k):
                    v = [r["sum"][ph][k][0] for r in rs if ph in r["sum"]]
                    return f"{np.median(v):.1f} [{max(v):.1f}]" if v else "-"
                ends = [r["sum"]["end"]["heading"] for r in rs if "end" in r["sum"]]
                votes = [r["pitch_vote_t"] for r in rs if r.get("pitch_vote_t")]
                g, sfx = (c.split("_") + [""])[:2]
                L.append(f"| {'unaided' if c == 'U_ref' else g} | {NAME.get(sfx, '-' if c == 'U_ref' else sfx)} | "
                         f"{m:+.0f} | {agg('coast', 'nose')} | {agg('descent', 'nose')} | {agg('boost', 'roll')} | "
                         f"{agg('coast', 'roll')} | {agg('descent', 'roll')} | "
                         f"{(f'{np.median(ends):.1f} [{max(ends):.1f}]') if ends else '-'} | "
                         f"{(f'{np.median(votes):.3f} [{min(votes):.3f}..{max(votes):.3f}]') if votes else '-'} |")

    L.append("\n## Every run that ends worse than unaided\n")
    for c, lst in flagged.items():
        if lst:
            L.append(f"\n**{c}** ({len(lst)}):\n")
            L.extend(f"- {x}" for x in lst)
    bad = sum(1 for r in res if r["unhealthy"] > 0)
    L.append(f"\nHealth: {len(res)} runs, {bad} with an unhealthy or non-finite sample.\n")
    open(a.out, "w").write("\n".join(L) + "\n")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
