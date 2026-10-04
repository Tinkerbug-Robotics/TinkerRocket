#!/usr/bin/env python3
"""Runs mcsim over every Monte Carlo draw in DIR (from mc_scenario.py), per design and link offset, with each
run's own oscillator, clock-error and seeds, so every design sees the same skies.

    mc_run.py DIR [--designs B,BA,BG] [--offsets 0,3,6,9,12] [--jobs 8]

Writes DIR/out_NNN_DESIGN.csv at the link budget and DIR/out_NNN_DESIGN_mOFF.csv OFF dB under it.
"""
from __future__ import annotations

import argparse
import csv
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

MCSIM = Path(__file__).resolve().parents[1] / 'build' / 'host' / 'mcsim'


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dir', type=Path)
    ap.add_argument('--designs', default='B,BA,BG')
    ap.add_argument('--offsets', default='0,3,6,9,12')
    ap.add_argument('--jobs', type=int, default=8)
    a = ap.parse_args()
    jobs = []
    for r in csv.DictReader(open(a.dir / 'runs.csv')):
        i = int(r['run'])
        for off in (int(x) for x in a.offsets.split(',')):
            for d in a.designs.split(','):
                out = a.dir / (f'out_{i:03d}_{d}.csv' if off == 0 else f'out_{i:03d}_{d}_m{off}.csv')
                jobs.append([str(MCSIM), str(a.dir / f'scen_{i:03d}.csv'), str(a.dir / 'flight.csv'), '--design', d,
                             '--gamma', r['gamma_ppb'], '--clk-err', r['clk_err'], '--seed', r['seed'],
                             '--imu-seed', r['imu_seed'], '--cn0-offset', str(-off), '--out', str(out)])
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        for p in ex.map(lambda j: subprocess.run(j, capture_output=True, text=True), jobs):
            if p.returncode:
                print(p.stderr.strip())
    print(f'{len(jobs)} runs')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
