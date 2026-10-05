#!/usr/bin/env python3
"""Runs mcsim over every Monte Carlo draw in DIR (from mc_scenario.py), per design and link offset, with each
run's own oscillator, clock-error and seeds, so every design sees the same skies.

    mc_run.py DIR [--designs Q,B,QA,BA,BG] [--offsets 0,3,6,9,12] [--jobs 8] [--clk-prior[-abs] SIGMA --tag T]

Writes DIR/out_NNN_DESIGN.csv at the link budget and DIR/out_NNN_DESIGN_mOFF.csv OFF dB under it. With
--clk-prior, each run's stored gamma is off by a fraction drawn from N(0, SIGMA); with --clk-prior-abs, by
N(0, SIGMA) ppb/g (a ground check's error, the same for any part). Each draw has its own generator, seeded by
the run, so the skies' draws stay as they are; the fractions go to DIR/clk_priorT.csv, and the outputs are
named DESIGN + T.
"""
from __future__ import annotations

import argparse
import csv
import subprocess

import numpy as np
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

MCSIM = Path(__file__).resolve().parents[1] / 'build' / 'host' / 'mcsim'


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dir', type=Path)
    ap.add_argument('--designs', default='Q,B,QA,BA,BG')
    ap.add_argument('--offsets', default='0,3,6,9,12')
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--clk-prior', type=float, help="the stored gamma's spread, a fraction (1 sigma)")
    ap.add_argument('--clk-prior-abs', type=float, help="the stored gamma's spread, ppb/g (1 sigma)")
    ap.add_argument('--tag', default='')
    a = ap.parse_args()
    jobs, priors = [], []
    for r in csv.DictReader(open(a.dir / 'runs.csv')):
        i = int(r['run'])
        extra = []
        if a.clk_prior is not None or a.clk_prior_abs is not None:
            z = float(np.random.default_rng(100000 + i).normal())
            g = abs(float(r['gamma_ppb']))
            e = z * a.clk_prior if a.clk_prior is not None else (z * a.clk_prior_abs / g if g > 0 else 0.0)
            extra = ['--clk-prior', f'{e:.5f}']
            priors.append(f'{i},{e:.5f}\n')
        for off in (int(x) for x in a.offsets.split(',')):
            for d in a.designs.split(','):
                name = d + a.tag
                out = a.dir / (f'out_{i:03d}_{name}.csv' if off == 0 else f'out_{i:03d}_{name}_m{off}.csv')
                jobs.append([str(MCSIM), str(a.dir / f'scen_{i:03d}.csv'), str(a.dir / 'flight.csv'), '--design', d,
                             '--gamma', r['gamma_ppb'], '--clk-err', r['clk_err'], '--seed', r['seed'],
                             '--imu-seed', r['imu_seed'], '--cn0-offset', str(-off), *extra, '--out', str(out)])
    if priors:
        (a.dir / f'clk_prior{a.tag}.csv').write_text('run,clk_prior\n' + ''.join(priors))
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        for p in ex.map(lambda j: subprocess.run(j, capture_output=True, text=True), jobs):
            if p.returncode:
                print(p.stderr.strip())
    print(f'{len(jobs)} runs')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
