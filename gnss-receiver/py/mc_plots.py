#!/usr/bin/env python3
"""Figures for mcsim's Monte Carlo runs (DIR/summary.csv from mc_summary.py, DIR/scen_*.csv):

    mc_plots.py DIR --flight Hotshot -o FIGDIR

  mc_sats.png     per run, the satellites maintained (tracked, and with carrier lock) through the boost and
                  to burnout + 20 s, by design and link margin;
  mc_signals.png  per signal, the share kept, by design, at the link budget and 12 dB under it;
  mc_cn0.png      the link budget: every signal's C/N0 at ignition against its elevation, all runs.
"""
from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

DESIGNS = (('Q', 'Unaided: 10 Hz quiet loops', '#7a828c'), ('B', 'Unaided: 50 Hz boost loops (the fallback)', '#b4442c'),
           ('QA', 'IMU + 10 Hz quiet loops', '#1a9bb0'), ('BA', 'IMU + 20 Hz loops (the design)', '#1d5fbf'),
           ('BG', 'Gated profile', '#1f7a4d'))
STEP = 0.17  # between the designs in a group
SIG_NAMES = {'L1CA': 'GPS L1 C/A', 'E1C': 'Galileo E1-C', 'B1CP': 'BeiDou B1C', 'L5Q': 'GPS L5',
             'E5AQ': 'Galileo E5a', 'B2AP': 'BeiDou B2a'}
METRICS = (('tb', 'Tracked through the boost'), ('ta', 'Tracked to burnout + 20 s'),
           ('cb', 'Carrier lock through the boost'), ('ca', 'Carrier lock to burnout + 20 s'))
INK, SLATE = '#17202a', '#566271'
plt.rcParams.update({'font.size': 9, 'axes.titlesize': 9.5, 'axes.labelsize': 9, 'axes.spines.top': False,
                     'axes.spines.right': False, 'axes.grid': True, 'grid.color': '#e4e7ea', 'grid.linewidth': 0.6,
                     'axes.titlelocation': 'left'})


def load(dirp: Path):
    d = defaultdict(list)
    for r in csv.DictReader(open(dirp / 'summary.csv')):
        d[(r['design'], int(r['offset_db']), r['what'])].append({m: int(r[m]) for m in ('trk', 'tb', 'ta', 'cb', 'ca')})
    return d


def fig_sats(d, offsets, flight, out):
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 7.6), sharey=True, constrained_layout=True)
    rng = np.random.default_rng(3)
    for ax, (m, title) in zip(axes.flat, METRICS):
        for gi, off in enumerate(offsets):
            for di, (dk, _, col) in enumerate(DESIGNS):
                v = d.get((dk, off, 'SAT'))
                if not v:
                    continue
                kept = np.array([x[m] for x in v])
                x0 = gi + (di - (len(DESIGNS) - 1) / 2) * STEP
                ax.scatter(x0 + rng.uniform(-0.05, 0.05, kept.size), kept + rng.uniform(-0.18, 0.18, kept.size), s=6,
                           color=col, alpha=0.45, linewidths=0)
                ax.plot([x0 - 0.065, x0 + 0.065], [np.median(kept)] * 2, color=col, lw=2.2)
                ax.plot([x0, x0], [np.percentile(kept, 5), np.percentile(kept, 95)], color=col, lw=1.0)
            trk = np.array([x['trk'] for x in d[('BA', off, 'SAT')]])
            ax.plot([gi - 0.46, gi + 0.46], [np.median(trk)] * 2, color=SLATE, lw=0.8, ls=':')
        ax.set_xticks(range(len(offsets)))
        ax.set_xticklabels(['link budget' if o == 0 else f'{o} dB' for o in offsets])
        ax.set_title(title, loc='left')
        ax.set_ylabel('satellites, of those tracked at ignition')
    for _, lab, col in DESIGNS:
        axes[0, 0].plot([], [], color=col, lw=2.2, label=lab)
    axes[0, 0].plot([], [], color=SLATE, lw=0.8, ls=':', label='median tracked at ignition')
    axes[0, 0].legend(frameon=False, loc='lower left', fontsize=8, ncol=2)
    fig.suptitle(f'{flight}: satellites maintained in each of 100 skies (dots; bar = median, line = 5th-95th '
                 'percentile), by design and link margin', x=0.01, ha='left', fontsize=10.5)
    fig.savefig(out / 'mc_sats.png', dpi=150)
    plt.close(fig)


def fig_signals(d, flight, out, offs=(0, -12)):
    sigs = list(SIG_NAMES)
    rows = (('ta', 'tracked to burnout + 20 s'), ('cb', 'carrier lock through the boost'),
            ('ca', 'carrier lock to burnout + 20 s'))
    fig, axes = plt.subplots(len(rows), len(offs), figsize=(12.5, 8.6), sharey=True, constrained_layout=True)
    for j, off in enumerate(offs):
        for i, (m, lab) in enumerate(rows):
            ax = axes[i, j]
            ax.set_axisbelow(True)
            for di, (dk, _, col) in enumerate(DESIGNS):
                fr = []
                for s in sigs:
                    v = d.get((dk, off, s), [])
                    n = sum(x['trk'] for x in v)
                    fr.append(100.0 * sum(x[m] for x in v) / n if n else np.nan)
                xs = np.arange(len(sigs)) + (di - (len(DESIGNS) - 1) / 2) * 0.165
                ax.bar(xs, fr, width=0.155, color=col)
            ax.set_xticks(range(len(sigs)))
            ax.set_xticklabels([SIG_NAMES[s] for s in sigs], fontsize=8.5)
            ax.set_ylim(0, 105)
            ax.set_ylabel('% of signals tracked at ignition')
            ax.set_title(f"{'Link budget' if off == 0 else f'{-off} dB under it'}: {lab}", loc='left')
    fig.legend(handles=[Patch(color=col, label=lab) for _, lab, col in DESIGNS], frameon=False,
               loc='outside lower center', ncol=len(DESIGNS), fontsize=8.5)
    fig.suptitle(f'{flight}: each signal kept, all 100 skies', x=0.01, ha='left', fontsize=10.5)
    fig.savefig(out / 'mc_signals.png', dpi=150)
    plt.close(fig)


def fig_cn0(dirp, out):
    fig, ax = plt.subplots(figsize=(8.5, 4.6), constrained_layout=True)
    cols = {'L1CA': '#1f6feb', 'E1C': '#c4561a', 'B1CP': '#1a7f37', 'L5Q': '#0b3d91', 'E5AQ': '#8a3b12', 'B2AP': '#0f4d22'}
    pts = defaultdict(lambda: ([], []))
    for p in sorted(dirp.glob('scen_*.csv')):
        seen = set()
        for r in csv.DictReader(open(p)):
            if r['ch'] in seen or abs(float(r['t_s'])) > 0.05:
                continue
            seen.add(r['ch'])
            pts[r['sig']][0].append(float(r['el_deg']))
            pts[r['sig']][1].append(float(r['cn0_dbhz']))
    for s, (el, c) in pts.items():
        l5 = s in ('L5Q', 'E5AQ', 'B2AP')
        ax.scatter(el, c, s=6 if l5 else 5, alpha=0.35, color=cols[s], linewidths=0, marker='^' if l5 else 'o',
                   label=SIG_NAMES[s])
    for y, lab in ((33.0, 'GPS L1 C/A, unaided 50 Hz loops: they start to slip near here'),
                   (31.0, 'GPS L1 C/A, the aided design: near here')):
        ax.axhline(y, color=SLATE, lw=0.8, ls='--')
        ax.text(6, y + 0.3, lab, ha='left', va='bottom', fontsize=8.5, color=SLATE)
    ax.set_xlabel('elevation at ignition, deg')
    ax.set_ylabel('C/N0 at ignition, dB-Hz')
    ax.set_title('The link budget: every tracked signal in every sky (antenna up, nominal power)', loc='left')
    ax.legend(frameon=False, fontsize=8, markerscale=2.5, ncol=2, loc='upper left')
    fig.savefig(out / 'mc_cn0.png', dpi=150)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dir', type=Path)
    ap.add_argument('--flight', default='Hotshot')
    ap.add_argument('-o', type=Path, required=True)
    a = ap.parse_args()
    a.o.mkdir(parents=True, exist_ok=True)
    d = load(a.dir)
    offsets = sorted({k[1] for k in d}, reverse=True)
    fig_sats(d, offsets, a.flight, a.o)
    fig_signals(d, a.flight, a.o)
    fig_cn0(a.dir, a.o)
    print('wrote', a.o)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
