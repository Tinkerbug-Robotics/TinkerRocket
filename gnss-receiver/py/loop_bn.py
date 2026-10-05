#!/usr/bin/env python3
"""A receiver's carrier loop noise bandwidth from its own pad data: on a static pad a PLL's phase jitter is
sigma^2 = Bn / (C/N0) (rad^2, plus a squaring loss of 1 / (2 T C/N0) for a Costas loop on data). Per GPS L1 C/A
satellite, over file seconds S0..S1: the carrier phase in 2 s pieces, each less a cubic fit (the satellites' motion
and the clocks' drift), then less the mean over satellites at each epoch (what the clocks add faster than that);
Bn = sigma^2 x C/N0 / (1 + 1 / (2 T C/N0)), T = 1 ms.

    loop_bn.py mosaic CAPTURE.sbf ... | ours RUNDIR ...   [--from 120 --to 175]

On our own receiver (B1C hotshot file, 40 dB-Hz) it reads the quiet 10 Hz loops as 8.0 Hz, the design's 20 Hz as
18.2 and the 50 Hz fallback as 66 (its FLL and the three-dump command delay add noise): about right, low at the bottom.
The mosaic's SBF decoder is tools/gnss-cocom/septentrio_sbf.py (PR #1567). The p180 files start at GPS s 203820.
"""
import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/gnss-cocom'))   # septentrio_sbf (PR #1567)
TOW0 = 203820.0
T_INT = 1e-3


def series_mosaic(path: Path, s0, s1):
    import septentrio_sbf as sbf
    sp = sbf.Splitter()
    out = defaultdict(list)
    for kind, blk in sp.feed(path.read_bytes()):
        if kind != 'S':
            continue
        bid, _ = sbf.block_id(blk)
        if bid != 4027:
            continue
        me = sbf.meas_epoch(blk)
        if me['tow'] is None:
            continue
        t = me['tow'] / 1000.0 - TOW0
        if not (s0 <= t <= s1):
            continue
        for m in me['meas']:
            if m['signal'] == 'GPS L1CA' and m['carrier'] is not None and m['cn0'] is not None:
                out[m['sv']].append((round(t, 3), m['carrier'], m['cn0'], m['lock']))
    return out


def series_ours(run: Path, s0, s1):
    ini = dict(l.split(' = ', 1) for l in (run / 'run.ini').read_text().splitlines() if ' = ' in l)
    start = float(ini['start_s'])
    out = defaultdict(list)
    for r in csv.DictReader(open(run / 'obs.csv')):
        p = int(r['prn'])
        t = float(r['t_s']) + start
        if p < 100 and s0 <= t <= s1:
            out[p].append((round(t, 3), float(r['adr_cyc']), float(r['cn0']), float(r['lock_s'])))
    return out


def bn(series):
    """Per satellite (Bn Hz, C/N0 dB-Hz, phase sigma deg), from continuous, never-reset series."""
    res = {}
    for sv, v in series.items():
        v.sort()
        t = np.array([a[0] for a in v]); ph = np.array([a[1] for a in v]); cn = np.array([a[2] for a in v])
        lk = [a[3] for a in v if a[3] is not None]
        if t.size < 100 or np.diff(t).max() > 0.2 or any(b < a for a, b in zip(lk, lk[1:])):
            continue
        r = np.full(t.size, np.nan)
        edges = np.arange(t[0], t[-1] + 2.0, 2.0)
        for a, b in zip(edges[:-1], edges[1:]):
            m = (t >= a) & (t < b)
            if m.sum() >= 12:
                c = np.polyfit(t[m] - a, ph[m], 3)
                r[m] = ph[m] - np.polyval(c, t[m] - a)
        res[sv] = (t, r, float(np.median(cn)))
    if len(res) < 5:
        return {}
    epochs = sorted(set(np.concatenate([v[0] for v in res.values()])))
    idx = {e: i for i, e in enumerate(epochs)}
    grid = np.full((len(res), len(epochs)), np.nan)
    for k, (sv, (t, r, c)) in enumerate(res.items()):
        grid[k, [idx[x] for x in t]] = r
    n = np.sum(np.isfinite(grid), axis=0)
    common = np.where(n >= 5, np.nanmean(grid, axis=0), np.nan)
    out = {}
    for k, (sv, (t, r, c)) in enumerate(res.items()):
        d = grid[k] - common
        d = d[np.isfinite(d)]
        if d.size < 100:
            continue
        nn = float(np.median(n[np.isfinite(grid[k])]))
        var = np.var(d) * nn / (nn - 1) * (2 * np.pi) ** 2          # rad^2, the mean removal undone
        cn0 = 10 ** (c / 10)
        out[sv] = (var * cn0 / (1 + 1 / (2 * T_INT * cn0)), c, np.degrees(np.sqrt(var)))
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('kind', choices=('mosaic', 'ours'))
    ap.add_argument('paths', nargs='+')
    ap.add_argument('--from', dest='s0', type=float, default=120.0)
    ap.add_argument('--to', dest='s1', type=float, default=175.0)
    a = ap.parse_args()
    for p in a.paths:
        s = series_mosaic(Path(p), a.s0, a.s1) if a.kind == 'mosaic' else series_ours(Path(p), a.s0, a.s1)
        r = bn(s)
        if not r:
            print(f'{Path(p).name}: too few clean satellites'); continue
        b = np.array([x[0] for x in r.values()]); c = np.array([x[1] for x in r.values()]); sd = np.array([x[2] for x in r.values()])
        name = Path(p).name.split('_signalsim')[0]
        print(f'{name:28s} {len(b):2d} sats: Bn median {np.median(b):5.1f} Hz (quartiles {np.percentile(b, 25):.1f}-{np.percentile(b, 75):.1f}); '
              f'C/N0 median {np.median(c):.1f}; phase sigma median {np.median(sd):.1f} deg')
