#!/usr/bin/env python3
"""mcsim's Monte Carlo runs, counted: per run, design and link offset, the signals and satellites tracked at
ignition and those kept through the boost and through the 20 s after it.

    mc_summary.py DIR [--designs Q,B,QA,BA,BG] [--offsets 0,3,6,9,12]   -> DIR/summary.csv

Kept, per signal tracked (PLL locked) at ignition:
  tracked    never dropped (the channel never went off): code and Doppler kept coming;
  carrier    and the PLL never let go and never slipped: carrier phase kept as well;
over the boost (ignition to burnout) and over ignition to burnout + 20 s. A satellite is kept if any of its
signals is.
"""
from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

SIGS = ('L1CA', 'E1C', 'B1CP', 'L5Q', 'E5AQ', 'B2AP')
METRICS = ('trk', 'tb', 'ta', 'cb', 'ca')


def kept(r: dict) -> tuple[int, int, int, int, int]:
    """(tracked at ignition, tracked through the boost, tracked to burnout + 20 s, carrier through the boost,
    carrier to burnout + 20 s)."""
    if r['locked_at_ign'] != '1':
        return 0, 0, 0, 0, 0
    tb = r['boost_off'] == '0'
    ta = tb and r['after_off'] == '0'
    cb = tb and float(r['boost_unl']) == 0.0 and int(r['boost_slips']) == 0
    ca = cb and ta and float(r['after_unl']) == 0.0 and int(r['after_slips']) == 0
    return 1, int(tb), int(ta), int(cb), int(ca)


def summarize(path: Path) -> dict[str, list[int]]:
    per = defaultdict(lambda: [0] * 5)
    sats = defaultdict(lambda: [0] * 5)
    for r in csv.DictReader(open(path)):
        k = kept(r)
        v = per[r['sig']]
        s = sats[r['sat']]
        for j in range(5):
            v[j] += k[j]
            s[j] = max(s[j], k[j])
    out = {sig: per[sig] for sig in SIGS}
    for sys_ in 'GEC':
        out['SAT_' + sys_] = [sum(v[j] for name, v in sats.items() if name[0] == sys_) for j in range(5)]
    out['SAT'] = [sum(v[j] for v in sats.values()) for j in range(5)]
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dir', type=Path)
    ap.add_argument('--designs', default='Q,B,QA,BA,BG')
    ap.add_argument('--offsets', default='0,3,6,9,12')
    a = ap.parse_args()
    runs = [int(r['run']) for r in csv.DictReader(open(a.dir / 'runs.csv'))]
    rows = ['run,design,offset_db,what,' + ','.join(METRICS) + '\n']
    for off in (int(x) for x in a.offsets.split(',')):
        for d in a.designs.split(','):
            for i in runs:
                p = a.dir / (f'out_{i:03d}_{d}.csv' if off == 0 else f'out_{i:03d}_{d}_m{off}.csv')
                if not p.exists():
                    continue
                for what, v in summarize(p).items():
                    rows.append(f'{i},{d},{-off},{what},' + ','.join(str(x) for x in v) + '\n')
    (a.dir / 'summary.csv').write_text(''.join(rows))
    print(f"{len(rows) - 1} rows -> {a.dir / 'summary.csv'}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
