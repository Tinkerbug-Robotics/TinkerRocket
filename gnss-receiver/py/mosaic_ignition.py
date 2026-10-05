#!/usr/bin/env python3
"""A commercial receiver and ours through the hotshot's ignition on the same file: the mosaic-G5's captures (SBF, from
the rig's mosaic_run.py) against gnssrx runs, GPS L1 C/A, per signal level.

    mosaic_ignition.py --captures DIR --runs DIR --traj SCEN.csv --nav BRDC.rnx -o FIG.png

For each satellite tracked at T-1 s, to T+3.0 s (the mosaic withholds everything past 600 m/s, at T+3.1 on the hotshot):
  holding lock  the receiver's own word: no 0.5 s gap and no lock-time reset (ours: PLL locked throughout);
  unbroken      judged against the truth: the carrier phase less the integral of the file's Doppler (the troposphere's
                thinning included), less the pad's clock offset and drift and the per-epoch median over satellites
                (what a receiver clock adds); a slip leaves a step, so unbroken = within 0.35 cycles from then on;
  Doppler error against the file's motion, each satellite's pad offset taken out.
The mosaic's SBF decoder is tools/gnss-cocom/septentrio_sbf.py (PR #1567).
"""
from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / 'tools/gnss-cocom'))
import boost_plots as bp  # noqa: E402

TOW0, IGN, END = 203820.0, 180.0, 3.0      # file second 0 in GPS s of week; ignition (file s); the window's end (s)
LAM = 299792458.0 / 1575.42e6
TT = np.round(np.arange(-1.0, 3.501, 0.05), 3)
LEVELS = (('0', '40.5', 'GPS 40.0 dB-Hz on the pad'), ('3', '38', '37.3 dB-Hz'), ('6', '35', '34.5 dB-Hz'))
NUDGE = {'B': -0.13, 'BA': 0.13}          # counts that coincide are drawn a hair apart, so every line shows
CASES = (('mosaic', 'M', 'mosaic-G5, default dynamics', '#b4442c', '--'),
         ('mosaic', 'H', 'mosaic-G5, High dynamics', '#b4442c', '-'),
         ('ours', 'Q', 'ours, quiet 10 Hz loops, unaided', '#7a828c', '-'),
         ('ours', 'B', 'ours, 50 Hz loops, unaided', '#c98a1c', '-'),
         ('ours', 'BA', 'ours, IMU + 20 Hz loops (the design)', '#1d5fbf', '-'))


class _T0:
    t0 = TOW0


def mosaic_series(cap_dir: Path, tag: str):
    """prn -> sorted [(t file s, Doppler Hz, carrier cycles, lock s)] from a b1chs<tag> capture."""
    import septentrio_sbf as sbf
    path = next(cap_dir.glob(f'mosaic_g5_b1chs{tag}_*.sbf'))
    s = defaultdict(list)
    for kind, blk in sbf.Splitter().feed(path.read_bytes()):
        if kind != 'S':
            continue
        bid, _ = sbf.block_id(blk)
        if bid != 4027:
            continue
        me = sbf.meas_epoch(blk)
        if me['tow'] is None:
            continue
        t = me['tow'] / 1000.0 - TOW0
        if IGN - 12 <= t <= IGN + 3.6:
            for m in me['meas']:
                if m['signal'] == 'GPS L1CA' and m['pr'] is not None and m['doppler'] is not None and m['carrier'] is not None:
                    s[int(m['sv'][1:])].append((t, m['doppler'], m['carrier'], m['lock']))
    return {p: sorted(v) for p, v in s.items()}


def ours_series(runs: Path, cfg: str, lv: str):
    """The same from a gnssrx run: Doppler and lock from trk.csv (PLL locked), the carrier from obs.csv."""
    run = runs / f'{cfg}_hot_{lv}'
    ini = dict(l.split(' = ', 1) for l in (run / 'run.ini').read_text().splitlines() if ' = ' in l)
    start = float(ini['start_s'])
    car = defaultdict(dict)
    for r in csv.DictReader(open(run / 'obs.csv')):
        p = int(r['prn'])
        car[p][round(float(r['t_s']) + start, 2)] = float(r['adr_cyc'])
    s = defaultdict(list)
    for r in csv.DictReader(open(run / 'trk.csv')):
        p = int(r['prn'])
        t = round(float(r['t_s']) + start, 2)
        if p < 100 and IGN - 12 <= t <= IGN + 3.6 and r['state'] == '2' and t in car[p]:
            s[p].append((t, float(r['dop_hz']), car[p][t], None))
    return {p: sorted(v) for p, v in s.items()}


class Judge:
    def __init__(self, tr):
        self.tr = tr

    def truth_dop(self, prn, t):
        keys = np.arange(bp.key(t.min()) - 1, bp.key(t.max()) + 2)
        d = self.tr.los(prn, keys, TOW0)[:, 1] - bp.tropo_rate(_T0, self.tr, prn, keys, 10000.0) / LAM
        return keys / 10.0, d

    def truth_phase(self, prn, t):
        kt, d = self.truth_dop(prn, t)
        fine = np.arange(kt[0], kt[-1], 0.001)
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (np.interp(fine[1:], kt, d) + np.interp(fine[:-1], kt, d)) * 1e-3)])
        return np.interp(t, fine, cum)

    def judge(self, series):
        """Per satellite tracked at T-1: (held-until, unbroken-until) in s from ignition, and |Doppler error| on TT."""
        rows = {}
        for prn, v in series.items():
            t = np.array([a[0] for a in v]); dop = np.array([a[1] for a in v]); L = np.array([a[2] for a in v])
            lk = [a[3] for a in v]
            if not np.any(np.abs(t - (IGN - 1.0)) < 0.06):
                continue
            pad = (t > IGN - 10) & (t < IGN - 1)
            kt, td = self.truth_dop(prn, t)
            de = dop - np.interp(t, kt, td)
            de -= np.median(de[pad])
            held = END + 0.5
            w = t >= IGN - 1.0
            tw = t[w]
            gaps = np.flatnonzero(np.diff(tw) > 0.5)
            if gaps.size:
                held = min(held, tw[gaps[0]] - IGN)
            lw = [c for c, ok in zip(lk, w) if ok]
            if all(c is not None for c in lw):
                for k in range(1, len(lw)):
                    if lw[k] < lw[k - 1]:
                        held = min(held, tw[k] - IGN)
                        break
            if tw[-1] < IGN + END:
                held = min(held, tw[-1] - IGN)
            ph = self.truth_phase(prn, t)
            best = None
            for sgn in (1, -1):
                r = L - sgn * ph
                c = np.polyfit(t[pad] - IGN, r[pad], 1)
                res = r - np.polyval(c, t - IGN)
                if best is None or np.std(res[pad]) < best[0]:
                    best = (np.std(res[pad]), res)
            rows[prn] = dict(t=np.round(t, 2), held=held, res=best[1], de=np.abs(de))
        # the per-epoch median over satellites: what the receiver clock adds to every carrier
        grid = sorted(set(np.concatenate([r['t'] for r in rows.values()]))) if rows else []
        idx = {g: k for k, g in enumerate(grid)}
        m = np.full((len(rows), len(grid)), np.nan)
        for i, r in enumerate(rows.values()):
            m[i, [idx[x] for x in r['t']]] = r['res']
        n = np.sum(np.isfinite(m), axis=0)
        med = np.full(len(grid), np.nan)
        ok = n >= 5
        med[ok] = np.nanmedian(m[:, ok], axis=0)
        out = []
        for i, r in enumerate(rows.values()):
            res = r['res'] - med[[idx[x] for x in r['t']]]
            tt = r['t'] - IGN
            sel = (tt >= -1.0) & (tt <= min(END, r['held'])) & np.isfinite(res)
            big = np.abs(res[sel]) > 0.35
            unbroken = r['held']
            if big.any():
                # the step: from the first sample after which it never comes back within 0.35 cycles
                last_good = np.flatnonzero(~big)
                k = (last_good[-1] + 1) if last_good.size else 0
                if k < big.size:
                    unbroken = min(unbroken, float(tt[sel][k]))
            e = np.interp(TT, tt, r['de'])
            e[TT > r['held']] = np.nan
            out.append((r['held'], unbroken, e))
        return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--captures', type=Path, required=True)
    ap.add_argument('--runs', type=Path, required=True, help='the gnssrx runs, <CFG>_hot_<LEVEL>')
    ap.add_argument('--traj', type=Path, required=True)
    ap.add_argument('--nav', type=Path, required=True)
    ap.add_argument('-o', type=Path, required=True)
    a = ap.parse_args()
    J = Judge(bp.Truth(a.traj, a.nav, 'signalsim'))
    fig, axes = plt.subplots(3, 3, figsize=(15.5, 10.4), sharex=True, constrained_layout=True)
    for j, (mtag, olv, title) in enumerate(LEVELS):
        for kind, cfg, lab, col, ls in CASES:
            s = mosaic_series(a.captures, cfg + mtag) if kind == 'mosaic' else ours_series(a.runs, cfg, olv)
            rows = J.judge(s)
            held = np.array([[h >= x for x in TT] for h, _, _ in rows]).sum(axis=0)
            unbr = np.array([[u >= x for x in TT] for _, u, _ in rows]).sum(axis=0)
            errs = np.array([e for _, _, e in rows])
            med = np.array([np.nanmedian(c) if np.isfinite(c).sum() >= 3 else np.nan for c in errs.T])
            nudge = NUDGE.get(cfg, 0.0) if kind == 'ours' else 0.0
            axes[0, j].step(TT, held + nudge, where='post', color=col, ls=ls, lw=1.8, label=lab)
            axes[1, j].step(TT, unbr + nudge, where='post', color=col, ls=ls, lw=1.8)
            axes[2, j].plot(TT, med, color=col, ls=ls, lw=1.5)
            k3 = np.argmin(np.abs(TT - END))
            print(f'{title:26s} {lab:40s} holding {held[k3]:2d}/{len(rows)}, unbroken {unbr[k3]:2d}/{len(rows)} at '
                  f'T+{END:.1f}; median Doppler error peak {np.nanmax(med):.1f} Hz')
        axes[0, j].set_title(f'{title}\nGPS satellites holding lock (the receiver\'s own word)', loc='left')
        axes[1, j].set_title('carrier unbroken (judged against the truth)', loc='left')
        axes[2, j].set_title('median Doppler error of those still holding, Hz', loc='left')
        axes[2, j].set_yscale('log')
        axes[2, j].set_ylim(0.3, 300)
        axes[2, j].set_xlabel('time from ignition, s')
        for i in (0, 1):
            axes[i, j].set_ylim(-0.5, 13.8)
        for ax in axes[:, j]:
            ax.axvline(0, color='#555', ls=':', lw=0.8)
            ax.axvline(3.1, color='#555', ls='-.', lw=0.8)
            ax.grid(True, color='#e4e7ea', lw=0.6)
            for sp in ('top', 'right'):
                ax.spines[sp].set_visible(False)
        axes[0, j].text(3.05, 0.3, "the mosaic's\n600 m/s cutoff", ha='right', va='bottom', fontsize=8, color='#555')
    axes[0, 0].legend(frameon=False, fontsize=8, loc='lower left')
    axes[0, 0].set_ylabel('satellites, of 13')
    axes[1, 0].set_ylabel('satellites, of 13')
    axes[2, 0].set_ylabel('Hz')
    fig.suptitle('The hotshot\'s ignition on one file: the mosaic-G5 in its two dynamics modes against our receiver\'s '
                 'loops (GPS L1 C/A; dotted: ignition)', x=0.01, ha='left', fontsize=10.5)
    a.o.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.o, dpi=150)
    print('wrote', a.o)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
