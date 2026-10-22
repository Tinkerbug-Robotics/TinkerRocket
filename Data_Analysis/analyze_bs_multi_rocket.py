#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Per-rocket link quality and transmit timing from base-station logs.

Built for the three-rocket bench: several rocket computers on one channel, one
base station.  For each rocket ID it reports

  * frames received vs expected (from the seq counter), loss %, the longest
    silence and how the misses cluster (one-off drops vs long streaks);
  * the TRANSMIT PHASE — where in the 500 ms cycle each frame started on air.
    The base station stamps a packet when it finishes arriving, so the start is
    that time minus the frame's time on air.  Free-running rockets drift
    against each other and collide in streaks; slotted rockets sit in separate
    phase clusters a third of a period apart.

and, for every pair of rockets, how close their phases came per window
(closer than one FAST frame's time on air = their frames overlap on air).

Accepts one or many logs (binary TRBSLOG or legacy CSV).  One shared log (the
base station's format before per-rocket files) already has one time base.
Per-rocket files are put on one time axis by the ``bs_ms=<millis>`` value the
base station writes in each file's opening event; files without it keep their
own time base, and the cross-rocket phase comparison then says so.

    python3 Data_Analysis/analyze_bs_multi_rocket.py lora_20261019_201500.bin
    python3 Data_Analysis/analyze_bs_multi_rocket.py lora_*_r*.bin --plot bench.png
"""

import argparse
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bs_log  # noqa: E402

# Seq is a u16 that restarts at 0 on every rocket boot.  A forward jump bigger
# than this is a reboot (a new segment), not 30 000 lost frames.
SEQ_RESET_JUMP = 30000

_BS_MS_RE = re.compile(r'bs_ms=(\d+)')
# The base station's slot controller logs each adjust as an event naming the
# rocket and its error, e.g. "slot adjust rid=2 slot=1 err=+14 ms".
_SLOT_EVT_RE = re.compile(r'rid=(\d+).*?err=([+-]?\d+)')


def time_on_air_ms(payload_len, sf=8, bw_khz=250.0, cr=5, preamble_syms=12):
    """Semtech AN1200.13, explicit header, CRC on.

    A transcription of loraTimeOnAirMs() in RocketComputerTypes.h, rounded the
    same way, so the phase here is computed from the number the firmware uses.
    """
    sf = min(max(sf, 6), 12)
    cr_idx = min(max(cr - 4, 1), 4)
    ts_ms = (1 << sf) / bw_khz
    preamble_ms = (preamble_syms + 4.25) * ts_ms
    de = 1 if ts_ms >= 16.0 else 0
    num = 8 * payload_len - 4 * sf + 28 + 16
    den = 4 * (sf - 2 * de)
    n_payload = ((num + den - 1) // den) * (cr_idx + 4) if num > 0 and den > 0 else 0
    return int(preamble_ms + (8 + n_payload) * ts_ms + 0.5)


def load(paths, events_out=None):
    """Read every log; return (rows, shared_time_base).

    Each row gains ``t_abs_ms``: the file's ``bs_ms`` offset plus its time_ms.
    shared_time_base is False when several files were given and any of them
    lacks the offset, so their phases can't be compared with each other.
    Events are appended to ``events_out`` when a list is given.
    """
    rows = []
    shared = True
    for path in paths:
        file_rows, events, _fmt = bs_log.read_bs_log(str(path))
        if events_out is not None:
            events_out.extend(events)
        offset = None
        for ev in events:
            m = _BS_MS_RE.search(ev.get('event', ''))
            if m:
                offset = int(m.group(1)) - int(ev['time_ms'])
                break
        if offset is None:
            offset = 0
            if len(paths) > 1:
                shared = False
        for r in file_rows:
            r = dict(r)
            r['t_abs_ms'] = offset + int(r['time_ms'])
            r['source'] = Path(path).name
            rows.append(r)
    rows.sort(key=lambda r: r['t_abs_ms'])
    return rows, shared


def _frame_toa(row, radio):
    size = bs_log.SIZE_OF_LORA_SLOW if row.get('frame') == 'slow' else bs_log.SIZE_OF_LORA_FAST
    return time_on_air_ms(size, **radio)


def _circular_mean_std(phases, period):
    """Circular mean and spread (ms) of phases on a circle of ``period``."""
    if not phases:
        return float('nan'), float('nan')
    s = sum(math.sin(2 * math.pi * p / period) for p in phases)
    c = sum(math.cos(2 * math.pi * p / period) for p in phases)
    mean = (math.atan2(s, c) / (2 * math.pi) * period) % period
    r = min(1.0, math.hypot(s, c) / len(phases))
    std = math.sqrt(-2.0 * math.log(r)) / (2 * math.pi) * period if r > 0 else float('inf')
    return mean, std


def _phase_distance(a, b, period):
    d = abs(a - b) % period
    return min(d, period - d)


def rocket_stats(rows, period_ms=500, radio=None):
    """{rocket_id: stats dict} for every rocket in ``rows`` (time-sorted)."""
    radio = radio or {}
    by_rid = {}
    for r in rows:
        by_rid.setdefault(int(r['rocket_id']), []).append(r)

    out = {}
    for rid, rs in sorted(by_rid.items()):
        received = len(rs)
        expected = 1
        missed_runs = []
        segments = 1
        longest_gap_s = 0.0
        for prev, cur in zip(rs, rs[1:]):
            delta = (int(cur['seq']) - int(prev['seq'])) & 0xFFFF
            if delta == 0:
                continue                      # duplicate receive of one frame
            if delta > SEQ_RESET_JUMP:
                segments += 1                 # the rocket rebooted
                expected += 1
            else:
                expected += delta
                if delta > 1:
                    missed_runs.append(delta - 1)
            longest_gap_s = max(longest_gap_s,
                                (cur['t_abs_ms'] - prev['t_abs_ms']) / 1000.0)
        fast_phases = [(r['t_abs_ms'] - _frame_toa(r, radio)) % period_ms
                       for r in rs if r.get('frame') != 'slow']
        phase, spread = _circular_mean_std(fast_phases, period_ms)
        out[rid] = dict(
            received=received,
            expected=expected,
            loss_pct=100.0 * (1.0 - received / expected) if expected else float('nan'),
            longest_gap_s=longest_gap_s,
            missed_1=sum(1 for m in missed_runs if m == 1),
            missed_2_5=sum(1 for m in missed_runs if 2 <= m <= 5),
            missed_6_20=sum(1 for m in missed_runs if 6 <= m <= 20),
            missed_over_20=sum(1 for m in missed_runs if m > 20),
            segments=segments,
            phase_ms=phase,
            phase_spread_ms=spread,
            first_s=rs[0]['t_abs_ms'] / 1000.0,
            last_s=rs[-1]['t_abs_ms'] / 1000.0,
            names=sorted({r.get('source', '') for r in rs}),
        )
    return out


def window_phases(rows, period_ms=500, window_s=30.0, radio=None):
    """[(window_start_s, {rid: (phase_ms, frames)})] over fixed windows."""
    radio = radio or {}
    if not rows:
        return []
    t0 = rows[0]['t_abs_ms']
    buckets = {}
    for r in rows:
        if r.get('frame') == 'slow':
            continue
        w = int((r['t_abs_ms'] - t0) / 1000.0 // window_s)
        buckets.setdefault(w, {}).setdefault(int(r['rocket_id']), []).append(
            (r['t_abs_ms'] - _frame_toa(r, radio)) % period_ms)
    out = []
    for w in sorted(buckets):
        per = {rid: (_circular_mean_std(ph, period_ms)[0], len(ph))
               for rid, ph in buckets[w].items()}
        out.append((t0 / 1000.0 + w * window_s, per))
    return out


def pair_separation(windows, period_ms=500, overlap_ms=None):
    """{(a, b): dict(min_ms, median_ms, overlap_windows, windows)} per rocket pair."""
    overlap_ms = time_on_air_ms(bs_log.SIZE_OF_LORA_FAST) if overlap_ms is None else overlap_ms
    seps = {}
    for _t, per in windows:
        rids = sorted(per)
        for i, a in enumerate(rids):
            for b in rids[i + 1:]:
                seps.setdefault((a, b), []).append(
                    _phase_distance(per[a][0], per[b][0], period_ms))
    out = {}
    for pair, ds in seps.items():
        ds_sorted = sorted(ds)
        out[pair] = dict(min_ms=ds_sorted[0],
                         median_ms=ds_sorted[len(ds_sorted) // 2],
                         overlap_windows=sum(1 for d in ds if d < overlap_ms),
                         windows=len(ds))
    return out


def slot_adjusts(events):
    """{rocket_id: (adjusts, last_err_ms)} from the base station's slot events."""
    out = {}
    for ev in events:
        text = ev.get('event', '')
        if 'slot' not in text:
            continue
        m = _SLOT_EVT_RE.search(text)
        if m:
            n, _last = out.get(int(m.group(1)), (0, None))
            out[int(m.group(1))] = (n + 1, int(m.group(2)))
    return out


def report(rows, shared, period_ms, window_s, radio, events=()):
    stats = rocket_stats(rows, period_ms, radio)
    lines = []
    span = (rows[-1]['t_abs_ms'] - rows[0]['t_abs_ms']) / 1000.0 if rows else 0.0
    lines.append(f'{len(rows)} frames from {len(stats)} rocket(s) over {span:.0f} s')
    lines.append('')
    lines.append('rocket  recv/expected   loss   longest gap   misses: 1 / 2-5 / 6-20 / >20'
                 '   phase (spread)   boots')
    for rid, s in stats.items():
        lines.append(
            f'  {rid:>3}   {s["received"]:>6}/{s["expected"]:<6}  {s["loss_pct"]:5.1f}%'
            f'   {s["longest_gap_s"]:8.1f} s   '
            f'{s["missed_1"]:>4} / {s["missed_2_5"]:>3} / {s["missed_6_20"]:>4} / {s["missed_over_20"]:>3}'
            f'   {s["phase_ms"]:5.0f} ms ({s["phase_spread_ms"]:4.0f})   {s["segments"]}')
    windows = window_phases(rows, period_ms, window_s, radio)
    pairs = pair_separation(windows, period_ms)
    if pairs:
        lines.append('')
        if not shared:
            lines.append('NOTE: these files have no common time base (no bs_ms= event), '
                         'so the phases below are not comparable across rockets.')
        overlap = time_on_air_ms(bs_log.SIZE_OF_LORA_FAST, **radio)
        lines.append(f'pair    phase separation per {window_s:.0f} s window   '
                     f'(< {overlap} ms = FAST frames overlap on air)')
        for (a, b), p in sorted(pairs.items()):
            lines.append(f'  {a}-{b}   min {p["min_ms"]:4.0f} ms   median {p["median_ms"]:4.0f} ms'
                         f'   overlapping in {p["overlap_windows"]}/{p["windows"]} windows')
    adjusts = slot_adjusts(events)
    if adjusts:
        lines.append('')
        lines.append('base-station slot adjusts')
        for rid, (n, last) in sorted(adjusts.items()):
            lines.append(f'  rocket {rid}: {n} adjust(s), last error {last:+d} ms')
    return '\n'.join(lines), stats, pairs


def plot(rows, stats, period_ms, window_s, radio, out_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, (ax_phase, ax_loss) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    t0 = rows[0]['t_abs_ms']
    for rid in stats:
        rs = [r for r in rows if int(r['rocket_id']) == rid and r.get('frame') != 'slow']
        ax_phase.scatter([(r['t_abs_ms'] - t0) / 1000.0 for r in rs],
                         [(r['t_abs_ms'] - _frame_toa(r, radio)) % period_ms for r in rs],
                         s=4, label=f'rocket {rid}')
        # Loss per window from the seq span inside it.
        xs, ys = [], []
        all_rs = [r for r in rows if int(r['rocket_id']) == rid]
        w0 = 0
        while w0 * window_s * 1000 <= all_rs[-1]['t_abs_ms'] - t0:
            lo = t0 + w0 * window_s * 1000
            win = [r for r in all_rs if lo <= r['t_abs_ms'] < lo + window_s * 1000]
            if len(win) >= 2:
                exp = ((int(win[-1]['seq']) - int(win[0]['seq'])) & 0xFFFF) + 1
                if 0 < exp < SEQ_RESET_JUMP:
                    xs.append((lo - t0) / 1000.0 + window_s / 2)
                    ys.append(100.0 * (1.0 - len(win) / exp))
            w0 += 1
        ax_loss.plot(xs, ys, marker='o', ms=3, label=f'rocket {rid}')
    ax_phase.set_ylabel('transmit start in cycle (ms)')
    ax_phase.set_ylim(0, period_ms)
    ax_phase.legend(loc='upper right', markerscale=3)
    ax_loss.set_ylabel(f'loss per {window_s:.0f} s (%)')
    ax_loss.set_xlabel('time (s)')
    ax_loss.set_ylim(0, 100)
    ax_loss.legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('logs', nargs='+', help='base-station log files (.bin or .csv)')
    ap.add_argument('--period-ms', type=int, default=500, help='rocket TX period (default 500)')
    ap.add_argument('--window-s', type=float, default=30.0, help='window for phase and loss (default 30)')
    ap.add_argument('--sf', type=int, default=8)
    ap.add_argument('--bw', type=float, default=250.0, help='bandwidth in kHz')
    ap.add_argument('--cr', type=int, default=5, help='coding rate denominator (5 = 4/5)')
    ap.add_argument('--preamble', type=int, default=12)
    ap.add_argument('--plot', metavar='PNG', help='write a phase + loss plot')
    args = ap.parse_args(argv)

    radio = dict(sf=args.sf, bw_khz=args.bw, cr=args.cr, preamble_syms=args.preamble)
    events = []
    rows, shared = load(args.logs, events)
    if not rows:
        print('no telemetry frames in these logs')
        return 1
    text, stats, _pairs = report(rows, shared, args.period_ms, args.window_s, radio, events)
    print(text)
    if args.plot:
        plot(rows, stats, args.period_ms, args.window_s, radio, args.plot)
        print(f'\nplot written to {args.plot}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
