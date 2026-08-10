#!/usr/bin/env python3
"""Post-process the 100mm fin-tab CFD sweep.

Coordinate system (CFD frame):
  +X  = spanwise (fin root -> tip), root plane at X=0
  +Y  = lateral (thickness direction)
  +Z  = streamwise (flow direction)

Roll moment about the rocket axis (axis || Z at X = -D_AXIS, Y ~ 0):
  M_roll = Mz(origin) + Fy * D_AXIS
Tab deflection sign: delta > 0 moves the tab TE toward +Y.

Averages the last N_AVG iterations of each case; reports convergence
(std over the averaging window) so the report can quote it.
"""
import numpy as np
from pathlib import Path
import re
import sys
import json

D_AXIS = 0.044          # m (root plane -> axis; see roll_inertia.py)
RHO = 1.225
N_AVG = 200

RESULTS_DIR = Path(__file__).parent / "results"


def parse_force_dat(filepath):
    data = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            line = line.replace('(', '').replace(')', '')
            vals = line.split()
            if len(vals) >= 10:
                data.append([float(v) for v in vals[:10]])
    arr = np.array(data)
    if arr.size == 0:
        return None, None
    return arr[:, 0], arr[:, 1:4]


def load_cases(results_dir=RESULTS_DIR):
    cases = []
    for ff in sorted(results_dir.glob("*_force.dat")):
        name = ff.stem.replace("_force", "")
        mf = ff.parent / f"{name}_moment.dat"
        if not mf.exists():
            continue
        m = re.match(r"V(\d+)_(n?\d+)deg", name)
        if not m:
            continue
        vel = int(m.group(1))
        s = m.group(2)
        angle = -int(s[1:]) if s.startswith('n') else int(s)

        it_f, F = parse_force_dat(ff)
        it_m, M = parse_force_dat(mf)
        if F is None or M is None:
            print(f"  warning: empty data {name}")
            continue

        Fw, Mw = F[-N_AVG:], M[-N_AVG:]
        F_avg, M_avg = Fw.mean(axis=0), Mw.mean(axis=0)
        M_roll = M_avg[2] + F_avg[1] * D_AXIS
        M_roll_series = Mw[:, 2] + Fw[:, 1] * D_AXIS

        case = dict(
            name=name, vel=vel, angle=angle,
            F=F_avg, M=M_avg, M_roll=M_roll,
            q=0.5 * RHO * vel**2,
            F_std=Fw.std(axis=0), M_roll_std=M_roll_series.std(),
            n_iters=int(it_f[-1]),
        )
        fa = ff.parent / f"{name}_forceAll.dat"
        if fa.exists():
            _, FA = parse_force_dat(fa)
            if FA is not None:
                case['F_all'] = FA[-min(N_AVG, len(FA)):].mean(axis=0)
        cases.append(case)
    return cases


def fit_with_offset(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    A = np.vstack([x, np.ones_like(x)]).T
    m, b = np.linalg.lstsq(A, y, rcond=None)[0]
    r = y - (m * x + b)
    ss_tot = np.sum((y - y.mean())**2)
    r2 = 1 - np.sum(r**2) / ss_tot if ss_tot > 0 else 1.0
    return m, b, r2


def analyze(cases):
    out = {}
    for vel in sorted(set(c['vel'] for c in cases)):
        vc = sorted([c for c in cases if c['vel'] == vel], key=lambda c: c['angle'])
        ang = [c['angle'] for c in vc]
        mr = [c['M_roll'] for c in vc]
        slope, offset, r2 = fit_with_offset(ang, mr)     # N*m per deg
        out[vel] = dict(cases=vc, angles=ang, M_roll=mr,
                        Kt=slope, offset=offset, r2=r2,
                        q=0.5 * RHO * vel**2)
    return out


def main():
    cases = load_cases()
    print(f"Loaded {len(cases)} cases from {RESULTS_DIR}")
    if not cases:
        sys.exit(1)
    an = analyze(cases)
    summary = {}
    for vel, a in an.items():
        print(f"\n--- V = {vel} m/s (q = {a['q']:.0f} Pa) ---")
        print(f"{'ang':>5s} {'Fy (N)':>9s} {'Fz (N)':>8s} {'Mroll (mNm)':>12s} {'std':>7s} {'iters':>6s}")
        for c in a['cases']:
            print(f"{c['angle']:>+5d} {c['F'][1]:>9.3f} {c['F'][2]:>8.3f} "
                  f"{c['M_roll']*1e3:>12.2f} {c['M_roll_std']*1e3:>7.3f} {c['n_iters']:>6d}")
        print(f"  Kt = {a['Kt']*1e3:.3f} mN*m/deg  (offset {a['offset']*1e3:+.2f} mN*m, R^2={a['r2']:.5f})")
        summary[str(vel)] = dict(Kt=a['Kt'], offset=a['offset'], r2=a['r2'], q=a['q'])
    with open(RESULTS_DIR.parent / "sweep_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    print("\nsaved sweep_summary.json")


if __name__ == "__main__":
    main()
