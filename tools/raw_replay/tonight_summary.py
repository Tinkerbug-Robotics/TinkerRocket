#!/usr/bin/env python3
"""A short summary of tonight's runs once tonight.py has run: per run, its on-board score against the harness's
with the same configuration, the attitude numbers per phase, and the apogee voters (on board, and the harness's
velocity and pitch votes), as markdown.

    tonight_summary.py TAG [TAG ...]
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402


def f3(x, nd=2):
    return "-" if x is None else " / ".join(f"{v:.{nd}f}" for v in x)


def main():
    tags = sys.argv[1:]
    tr = S.Truth()
    summ = {x["run"]: x for x in json.load(open(os.path.join(HERE, "val", "tonight_onboard_vs_harness.json")))}
    vot = json.load(open(os.path.join(HERE, "voters.json"))) if os.path.exists(os.path.join(HERE, "voters.json")) \
        else {}
    L = [f"Truth: burnout T+{tr.burnout:.2f}, apogee T+{tr.apogee:.3f}, the truth's pitch below -5 deg at "
         f"T+{tr.pitch_vote:.3f}. Cells: median / p90 / max over the phase, sampled at the on-board records' times.\n"]
    for tag in tags:
        x = summ.get(tag)
        if x is None:
            continue
        o, h = x["onboard"], x["harness"]
        L.append(f"### {tag}\n")
        L.append("| metric | phase | on-board | harness (same config) | difference |")
        L.append("|---|---|---|---|---|")
        for k, name in (("nose", "nose-pointing (deg)"), ("roll", "roll about the nose (deg)"),
                        ("att_total", "total attitude (deg)"), ("heading", "heading, diagnostic (deg)"),
                        ("pitch", "pitch (deg)"), ("vz", "vertical velocity (m/s)"), ("pos_h", "horiz. position (m)"),
                        ("vel", "velocity 3-D (m/s)")):
            for ph in ("boost", "coast", "descent"):
                a = o["phases"].get(ph, {}).get(k)
                b = h["phases"].get(ph, {}).get(k)
                d = tuple(np.subtract(b, a)) if a is not None and b is not None else None
                L.append(f"| {name} | {ph} | {f3(a)} | {f3(b)} | {f3(d)} |")
        v = vot.get(tag, {})
        L.append("")
        L.append(f"Apogee voters, on board (T+ s): velocity {v.get('vel') or float('nan'):.3f}, barometer "
                 f"{v.get('baro') or float('nan'):.3f}, GNSS {v.get('gps') or float('nan'):.3f}, pitch "
                 f"{v.get('pitch') or float('nan'):.3f}; apogee called {v.get('apogee') or float('nan'):.3f} "
                 f"(margin before the true apogee {tr.apogee - (v.get('apogee') or np.nan):+.3f} s). "
                 f"Harness: velocity {h.get('vel_vote_t') or float('nan'):.3f}, pitch "
                 f"{h.get('pitch_vote_t') or float('nan'):.3f}. Pitch < -5 deg: on-board flight loop "
                 f"{o.get('ns_pitch_cross_t') or float('nan'):.3f}, truth {tr.pitch_vote:.3f}.\n")
        dv = x["diff"]
        L.append("Trajectory difference, harness less on-board (median / p90 / max): " + "; ".join(
            f"{ph} attitude {f3(m['att_total'])} deg, velocity {f3(m['vel'])} m/s, horizontal {f3(m['pos_h'], 1)} m"
            for ph, m in dv.items() if ph != "pad") + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
