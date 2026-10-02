#!/usr/bin/env python3
"""Where the NEO-M8T's measurements go wrong, from m8t_boost_traces.py's JSONs (with errors): per flight and level,
each wrong satellite's rate when it first passed 10 m (and the time), the largest error below a few rate bands, and
which satellites make up the traveler's over-10-m points.   m8t_onset.py"""
import json
from pathlib import Path

S = Path(__file__).resolve().parent / "data"
for fn in ("m8t_boost.json", "m8t_trav_boost.json"):
    runs = json.load(open(S / fn))
    print(f"##### {fn}")
    bands = [300, 400, 450, 500, 600, 700, 800]
    worst = {b: (0.0, "") for b in bands}
    for r in runs:
        ons = []
        for q in r["recs"]:
            if q.get("wrong"):
                ons.append(f"{q['sys']}{q['prn']:02d} {q['wrong_rate']:.0f} Hz/s at T+{q['wrong_t']:.1f} "
                           f"(max {q['err_max']:.0f} m{', lost' if q['lost'] else ''})")
            for t, ok, v, x in q["series"]:
                if ok and x is not None and t >= 0:
                    for b in bands:
                        if v <= b and abs(x) > worst[b][0]:
                            worst[b] = (abs(x), f"{r['label']} {q['sys']}{q['prn']:02d} at {v:.0f} Hz/s T+{t:.1f}")
        n_bad = sum(1 for q in r["recs"] for t, ok, v, x in q["series"] if ok and x is not None and t >= 0
                    and abs(x) > 10)
        print(f"  {r['label']}: {n_bad} points over 10 m; first over 10 m: {'; '.join(ons) if ons else 'none'}")
    for b in bands:
        print(f"  largest error at or below {b} Hz/s: {worst[b][0]:.1f} m ({worst[b][1]})")
