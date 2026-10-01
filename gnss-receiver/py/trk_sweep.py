#!/usr/bin/env python3
"""Sweeps trksim over loop profiles, satellites, C/N0 and seeds, and tabulates the worst
satellite per profile and C/N0 (each metric averaged over seeds first).

    trk_sweep.py --truth DIR --prn 11,24,21,5,15,29 --cn0 45,40,35,32 \\
        --window 575,640 --score-from 595 --boost-at 595,607 \\
        --profile Q= --profile B1=10,15,2/10,15,1 ...

A profile is NAME=[Q:SPEC][;B:SPEC], SPEC being trksim's PF,PP,PD/LF,LP,LD[:MS]: Q sets the
quiet loops (default --quiet, else trk's), B the boost loops over --boost-at. NAME= alone runs
the default quiet loops throughout.
"""
from __future__ import annotations

import argparse
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

TRKSIM = Path(__file__).resolve().parents[1] / "build" / "host" / "trksim"
BASE = ("ferr_max", "ferr_rms", "unlocked_s", "slips", "phase_rms_deg", "code_max_m")


def run(args: list[str]) -> dict[str, float]:
    out = subprocess.run([str(TRKSIM), *args], capture_output=True, text=True, check=True).stdout
    return {k: float(v) for k, v in (kv.split("=") for kv in out.split())}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--truth", type=Path, required=True)
    ap.add_argument("--prn", required=True)
    ap.add_argument("--cn0", required=True)
    ap.add_argument("--window", required=True)
    ap.add_argument("--score-from", required=True)
    ap.add_argument("--boost-at", default="")
    ap.add_argument("--quiet", default="")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--extra", default="", help="more trksim options, space separated")
    ap.add_argument("--split", default="", help="T1,T2: tabulate the pad, boost and coast parts")
    ap.add_argument("--profile", action="append", required=True)
    a = ap.parse_args()

    t0, t1 = a.window.split(",")
    prns = [int(p) for p in a.prn.split(",")]
    cn0s = [float(c) for c in a.cn0.split(",")]
    profiles = [p.split("=", 1) for p in a.profile]
    jobs = []
    for name, spec in profiles:
        for cn0 in cn0s:
            for prn in prns:
                for seed in range(1, a.seeds + 1):
                    args = [str(a.truth / f"prn{prn:02d}.csv"), "--from", t0, "--to", t1, "--score-from", a.score_from,
                            "--cn0", str(cn0), "--seed", str(seed)]
                    parts = dict(x.split(":", 1) for x in spec.split(";") if x)
                    quiet = parts.get("Q", a.quiet)
                    if quiet:
                        args += ["--quiet", quiet]
                    if "B" in parts:
                        args += ["--boost", parts["B"], "--boost-at", a.boost_at]
                    if a.extra:
                        args += a.extra.split()
                    if a.split:
                        args += ["--split", a.split]
                    jobs.append(((name, cn0, prn, seed), args))
    with ThreadPoolExecutor(max_workers=8) as ex:
        results = dict(zip([j[0] for j in jobs], ex.map(run, [j[1] for j in jobs])))

    parts = ["pad_", "boost_", "coast_"] if a.split else [""]
    for part in parts:
        keys = [part + k for k in BASE]
        print(f"\n{part.rstrip('_') or 'window'}: worst satellite (seed mean) per profile and C/N0")
        print(f"{'profile':<8} {'C/N0':>5}  {'lost':>4}  {'ferr max':>8} {'ferr rms':>8}  {'unlocked s':>10}  "
              f"{'slips':>5}  {'phase rms':>9}  {'code max m':>10}   worst")
        for name, _spec in profiles:
            for cn0 in cn0s:
                per = {}
                for prn in prns:
                    rs = [results[(name, cn0, prn, s)] for s in range(1, a.seeds + 1)]
                    per[prn] = {k: sum(r[k] for r in rs) / len(rs) for k in keys}
                    per[prn]["lost"] = sum(r["off_at"] >= 0 for r in rs)
                lost = sum(v["lost"] for v in per.values())
                alive = {p: v for p, v in per.items() if v["lost"] == 0}
                if not alive:
                    print(f"{name:<8} {cn0:5.0f}  {lost:4d}  (every run lost the channel)")
                    continue
                k_ul, k_fm = part + "unlocked_s", part + "ferr_max"
                w = max(alive, key=lambda p: (alive[p][k_ul], alive[p][k_fm]))
                col = {k: max(v[part + k] for v in alive.values()) for k in BASE}
                print(f"{name:<8} {cn0:5.0f}  {lost:4d}  {col['ferr_max']:8.1f} {col['ferr_rms']:8.2f}  "
                      f"{col['unlocked_s']:10.2f}  {col['slips']:5.1f}  {col['phase_rms_deg']:9.1f}  "
                      f"{col['code_max_m']:10.1f}   PRN {w}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
