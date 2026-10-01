#!/usr/bin/env python3
"""Facts for the report intro: per scenario (traveler_soft25, hotshot) the truth keys, burnout, apogee, the 515 m/s
windows, peak acceleration; the steepest line-of-sight Doppler rate in each boost chart JSON; what sky_m8t.json /
sky_px.json hold (C/N0 summaries only, never positions); and sweep_table.json's rows.   scen_facts.py"""
import json
import statistics as st
from pathlib import Path

S = Path(__file__).resolve().parent / "data"
SDR = Path(__file__).resolve().parents[1]
for scen in ("traveler_soft25", "hotshot"):
    sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
    tr, pro = sc["truth"], sc.get("prologue_s", 180.0)
    print(f"##### {scen}: top keys {sorted(sc.keys())}; truth keys {sorted(tr[0].keys())}; prologue {pro}")
    print(f"  velocity_windows (T): {[[round(a - pro, 2), round(b - pro, 2)] for a, b in sc['velocity_windows']]}")
    burn = next(s for s in tr if s["phase"] == "coast")
    print(f"  burnout T+{burn['t'] - pro:.2f} at {burn['speed_mps']:.0f} m/s; phases {sorted({s['phase'] for s in tr})}")
    altk = next((k for k in ("alt_m", "altitude_m", "alt", "h_m") if k in tr[0]), None)
    if altk:
        top = max(tr, key=lambda s: s[altk])
        print(f"  apogee {top[altk] / 1000:.1f} km at T+{top['t'] - pro:.1f}; end of truth T+{tr[-1]['t'] - pro:.1f}")
    acck = next((k for k in ("accel_mps2", "acc_mps2", "a_mps2", "accel") if k in tr[0]), None)
    if acck:
        pk = max(tr, key=lambda s: abs(s[acck]))
        print(f"  peak accel {pk[acck] / 9.80665:.1f} g at T+{pk['t'] - pro:.2f} ({acck})")
    else:
        rates = [((b["speed_mps"] - a["speed_mps"]) / (b["t"] - a["t"]), a["t"]) for a, b in zip(tr, tr[1:])
                 if b["t"] > a["t"] and a["phase"] != "pad"]
        pk = max(rates)
        print(f"  peak d(speed)/dt {pk[0] / 9.80665:.1f} g at T+{pk[1] - pro:.2f} (from speed)")
for fn in ("boost_final.json", "hot_boost.json"):
    runs = json.load(open(S / fn))
    mx = max(abs(p[2]) for r in runs for q in r["recs"] for p in q["series"] if p[2] is not None)
    print(f"##### {fn}: steepest line-of-sight rate {mx:.0f} Hz/s")
for fn in ("sky_m8t.json", "sky_px.json"):         # roof-log summaries: not committed (antenna geometry)
    if not (S / fn).exists():
        continue
    d = json.load(open(S / fn))
    print(f"##### {fn}: type {type(d).__name__}; keys {list(d.keys())[:12] if isinstance(d, dict) else len(d)}")
    if isinstance(d, dict):
        for k, v in list(d.items())[:12]:
            if isinstance(v, (int, float, str)):
                print(f"  {k}: {v}")
            elif isinstance(v, dict):
                print(f"  {k}: dict keys {list(v.keys())[:10]}")
            elif isinstance(v, list):
                print(f"  {k}: list len {len(v)}; first {json.dumps(v[0])[:160] if v else '-'}")
t = json.load(open(S / "sweep_table.json"))
print(f"##### sweep_table.json keys {t['keys']}")
for r in t["rows"]:
    print("  " + " | ".join(f"{k}={r[k]}" for _n, k in [("label", "label")] + t["keys"]))
