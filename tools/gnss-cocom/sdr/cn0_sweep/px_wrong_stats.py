#!/usr/bin/env python3
"""PX1105R wrong measurements (delivered, > 10 m off, Doppler-bridged clock rows left out) per flight, phase (up to the
first 515 m/s crossing / from there to burnout), level and system: count of 10 Hz rows, how many over 10 m, the
largest error, the satellites involved and the rate where each first passed 10 m.   px_wrong_stats.py"""
import json
from pathlib import Path

S = Path(__file__).resolve().parent / "data"
SDR = Path(__file__).resolve().parents[1]
for fn, scen in (("hot_boost.json", "hotshot"), ("boost_final.json", "traveler_soft25")):
    sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
    pro = sc["prologue_s"]
    over = sc["velocity_windows"][0][0] - pro
    burn = next(s["t"] for s in sc["truth"] if s["phase"] == "coast") - pro
    print(f"##### {fn}: 515 m/s at T+{over:.1f}, burnout T+{burn:.1f}")
    for lo, hi, ph in ((0.0, over, "to 515"), (over, burn, "515 to burnout")):
        tot = {c: [0, 0, 0.0, set()] for c in "GEC"}
        for r in json.load(open(S / fn)):
            parts = []
            for c in "GEC":
                n = nb = 0
                mx, sats = 0.0, []
                for q in r["recs"]:
                    if q["sys"] != c:
                        continue
                    pts = [p for p in q["series"] if len(p) > 5 and p[1] and p[5] is not None and lo <= p[0] < hi
                           and abs(p[0] * 10 - round(p[0] * 10)) < 0.01]
                    n += len(pts)
                    bad = [p for p in pts if abs(p[5]) > 10.0]
                    nb += len(bad)
                    if pts:
                        mx = max(mx, max(abs(p[5]) for p in pts))
                    if bad:
                        sats.append(f"{c}{q['prn']:02d}@{bad[0][2]:.0f}Hz/s")
                if n:
                    parts.append(f"{c} {nb}/{n} (max {mx:.0f} m{': ' + ' '.join(sats) if sats else ''})")
                    tot[c][0] += n
                    tot[c][1] += nb
                    tot[c][2] = max(tot[c][2], mx)
            print(f"  {ph:15s} {r['label'][:6]:6s} " + " | ".join(parts))
        print(f"  {ph:15s} ALL    " + " | ".join(f"{c} {v[1]}/{v[0]} (max {v[2]:.0f} m)" for c, v in tot.items() if v[0]))
