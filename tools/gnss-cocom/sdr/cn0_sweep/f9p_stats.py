#!/usr/bin/env python3
"""Every number the report's ZED-F9P part quotes, from the F9P boost-chart JSONs (f9p_boost.json, f9p_trav_boost.json,
m8t_boost_traces.py with errors), the accuracy NPZs (f9pacc_<tag>.npz) and outputs (f9p_acc_<tag>.out), and the
captures (the latest re-fly of each tag):
  1. per level: satellites still flagged valid at the raw cut-off, of those tracked at ignition; pad C/N0
  2. where measurements go wrong: each wrong satellite's rate and time at first > 10 m, its largest error, lost or not;
     largest error at or below rate bands; valid-flagged measurements over 10 m / all, per flight
  3. per satellite through the burn to the cut-off: elevation, end rate, PR RMS / max, Doppler RMS, lost
  4. fix: last fix before the cut-off with the receiver's speed against the truth; fix runs and raw gaps after T-10;
     fix periods above 80 km with the receiver's height range
  5. the accuracy tables' phase lines (above 80 km, descent) and clock steps
    f9p_stats.py"""
import json
import math
import os
import re
import struct
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
D, W = HERE / "data", HERE / "work"
SDR = Path(__file__).resolve().parents[1]
C = Path(os.environ.get("CAPTURES", str(SDR / "captures")))
IGN = 204000.0
SYS = "GEC"
RUNS = [("fhs12b", "+12 dB", "hotshot", "signalsim_hotshot_all_2026_57_w_p180"),
        ("fhs6b", "+6 dB", "hotshot", "signalsim_hotshot_all_2026_51_w_p180"),
        ("fhs0b", "0 dB", "hotshot", "signalsim_hotshot_all_2026_45_w_p180"),
        ("fhsn6b", "-6 dB", "hotshot", "signalsim_hotshot_all_2026_45_w_p180"),
        ("ftr12b", "+12 dB", "traveler_soft25", "signalsim_traveler_all_2026_57_w_p180"),
        ("ftr6b", "+6 dB", "traveler_soft25", "signalsim_traveler_all_2026_51_w_p180"),
        ("ftr0b", "0 dB", "traveler_soft25", "signalsim_traveler_all_2026_45_w_p180_cofs"),
        ("ftrn6b", "-6 dB", "traveler_soft25", "signalsim_traveler_all_2026_45_w_p180_cofs")]
JSON = {"hotshot": "f9p_boost.json", "traveler_soft25": "f9p_trav_boost.json"}


def capfor(tag, stem):
    for x in ("rrr", "rr", "r", ""):
        p = C / f"zed_f9p_wide{tag}{x}_{stem}.log"
        if p.exists() and Path(str(p) + ".hackrf.txt").exists():      # a finished flight (the log comes at the end)
            return p
    return None


def truth(scen):
    sc = json.loads((SDR / "scenarios" / f"{scen}.json").read_text())
    pro = sc["prologue_s"]
    return np.array([s["t"] - pro for s in sc["truth"]]), np.array([s["speed_mps"] for s in sc["truth"]]), \
        np.array([s["alt_m"] for s in sc["truth"]])


boost = {k: {r["label"]: r for r in json.load(open(D / v))} for k, v in JSON.items() if (D / v).exists()}
print("##### 1. still flagged valid at the raw cut-off / tracked at ignition; pad C/N0")
for scen, runs in boost.items():
    for lv, r in runs.items():
        cells = []
        for c in SYS:
            tot = [q for q in r["recs"] if q["sys"] == c]
            if tot:
                cells.append(f"{c} {sum(not q['lost'] for q in tot)}/{len(tot)} "
                             f"(within 10 m {sum(not q['lost'] and not q.get('wrong_end') for q in tot)})")
        print(f"  {scen:15s} {lv:7s} cut T+{r['cut']:.1f}  fix_end {r['fix_end']}  " + "  ".join(cells)
              + "  pad C/N0 " + ", ".join(f"{k} {v:.0f}" for k, v in r["pad_cn0"].items()))

print("\n##### 2. where it goes wrong")
for scen, runs in boost.items():
    bands = [300, 400, 450, 500, 600, 700, 800, 1000]
    worst = {b: (0.0, "") for b in bands}
    n_all = n_bad = 0
    for lv, r in runs.items():
        ons = []
        for q in r["recs"]:
            if q.get("wrong"):
                ons.append(f"{q['sys']}{q['prn']:02d} ({q['el']:.0f} deg) {q['wrong_rate']:.0f} Hz/s at "
                           f"T+{q['wrong_t']:.1f} (max {q['err_max']:.0f} m{', lost' if q['lost'] else ''}"
                           f"{', wrong at the end' if q.get('wrong_end') else ''})")
            for t, ok, v, x in q["series"]:
                if ok and x is not None and t >= 0:
                    n_all += 1
                    n_bad += abs(x) > 10
                    for b in bands:
                        if v <= b and abs(x) > worst[b][0]:
                            worst[b] = (abs(x), f"{lv} {q['sys']}{q['prn']:02d} at {v:.0f} Hz/s T+{t:.1f}")
        print(f"  {scen} {lv}: first over 10 m: {'; '.join(ons) if ons else 'none'}")
    print(f"  {scen}: {n_bad} of {n_all} valid-flagged measurements (T+0 .. cut-off) over 10 m")
    for b in bands:
        print(f"    largest error at or below {b} Hz/s: {worst[b][0]:.1f} m ({worst[b][1]})")

print("\n##### 3. per satellite through the burn (T+0 .. cut-off): el, end rate, n, PR RMS/max, Doppler RMS")
for tag, lv, scen, stem in RUNS:
    npz = W / f"f9pacc_{tag}.npz"
    if not npz.exists() or scen not in boost or lv not in boost[scen]:
        print(f"  {tag}: not available")
        continue
    r = boost[scen][lv]
    cut = r["cut"]
    z = np.load(npz)
    t, s, prn, pr, rr = z["t"], z["sys"].astype(int), z["prn"].astype(int), z["pr"], z["rr"]
    recs = {(q["sys"], q["prn"]): q for q in r["recs"]}
    rows = []
    for k in sorted({(int(a), int(b)) for a, b in zip(s, prn)}):
        m = (s == k[0]) & (prn == k[1]) & (t >= 0) & (t <= cut)
        if m.sum() < 3:
            continue
        q = recs.get((SYS[k[0]], k[1]), {})
        e, f = pr[m], rr[m][np.isfinite(rr[m])]
        rows.append((abs(q.get("rate", 0.0)), f"{SYS[k[0]]}{k[1]:02d}", q.get("el", math.nan), int(m.sum()),
                     math.sqrt(np.mean(e ** 2)), float(np.max(np.abs(e))),
                     math.sqrt(np.mean(f ** 2)) if len(f) else math.nan, q.get("lost")))
    rows.sort(reverse=True)
    print(f"  ## {tag} ({scen.split('_')[0]} {lv}) cut T+{cut:.1f}")
    for rate, name, el, n, rms, mx, rrr, lost in rows:
        print(f"    {name} {el:4.0f} deg {rate:6.0f} Hz/s  n {n:3d}  PR {rms:7.1f} m (max {mx:7.1f})  RR {rrr:6.2f} m/s"
              f"{'  LOST' if lost else ''}")

print("\n##### 4. fix: last fix before the cut-off vs truth; fix runs and raw gaps after T-10; fix above 80 km")
for tag, lv, scen, stem in RUNS:
    cap = capfor(tag, stem)
    if cap is None:
        print(f"  {tag}: no capture")
        continue
    tt, tv, ta = truth(scen)
    raw, pvt = [], []
    for line in open(cap, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "U":
            continue
        h = p[2].strip()
        try:
            b = bytes.fromhex(h[4:])
        except ValueError:
            continue
        if h.startswith("0215") and len(b) >= 16:
            nv = sum(1 for j in range(b[11]) if len(b) >= 48 + 32 * j and b[16 + 32 * j + 30] & 1)
            raw.append((struct.unpack_from("<d", b, 0)[0] - IGN, nv))
        elif h.startswith("0107") and len(b) >= 92:
            vn, ve, vd = struct.unpack_from("<iii", b, 48)
            pvt.append((struct.unpack_from("<I", b, 0)[0] / 1000.0 - IGN, b[20] >= 2 and bool(b[21] & 1),
                        (vn * vn + ve * ve + vd * vd) ** 0.5 / 1000.0, struct.unpack_from("<i", b, 36)[0] / 1000.0))
    ts = [t for t, nv in raw if nv > 0 and t >= -10]
    gaps = [(a, b) for a, b in zip(ts, ts[1:]) if b - a > 1.0]
    fr, cur = [], None
    for t, ok, v, h in pvt:
        if t < -10:
            continue
        if ok and cur is None:
            cur = [t, t, h, h]
        elif ok:
            cur[1], cur[2], cur[3] = t, min(cur[2], h), max(cur[3], h)
        elif cur is not None:
            fr.append(cur)
            cur = None
    if cur:
        fr.append(cur)
    first_cross = next(t for t, v in zip(tt, tv) if t > 0 and v > 515)
    last = max(((t, v) for t, ok, v, h in pvt if ok and 0 <= t <= first_cross + 1), default=None)
    lag = f"{float(np.interp(last[0], tt, tv)) - last[1]:+.0f} m/s behind" if last else "-"
    lf = f"T+{last[0]:.1f}" if last else "-"
    print(f"  {tag} ({cap.name.split('_signalsim')[0][12:]}): last fix before the cut {lf} ({lag}); "
          f"raw T{ts[0]:+.1f}..T{ts[-1]:+.1f}; no-raw gaps > 1 s: " + ", ".join(f"T{a:+.1f}..T{b:+.1f}" for a, b in gaps))
    print("     fix runs: " + ", ".join(f"T{a:+.1f}..T{b:+.1f} ({lo / 1000:.1f}-{hi / 1000:.1f} km)" for a, b, lo, hi in fr
                                       if b - a >= 0.5))

print("\n##### 5. accuracy tables (phase lines) and clock steps")
for tag, lv, scen, stem in RUNS:
    p = W / f"f9p_acc_{tag}.out"
    if not p.exists():
        continue
    txt = p.read_text()
    steps = re.search(r"receiver clock: .*", txt)
    print(f"  ## {tag}: {steps.group(0) if steps else ''}")
    for line in txt.splitlines():
        if re.match(r"^(above 80 km|descent, fix allowed|after 515 m/s|burn, below 515 m/s|burn, above 515 m/s|pad, last)"
                    r"|^\s{20,}(GPS|Galileo|BeiDou)", line):
            print("    " + line)
