#!/usr/bin/env python3
"""Summarize the flight computer's loop profile (-DTR_LOOP_PROFILE=1).

The firmware prints a report every 5 s (main/loop_profile.h): one [LP] window
line, a row per section and a counter total. The two Xtensa performance
counters rotate between three configurations from one window to the next, so a
single window shows only one pair. This merges every window of a capture into
one table: each section's mean cost, and each configuration's two shares,
weighted by the section's time.

    python3 tools/loop_profile_summary.py capture.log
    python3 tools/loop_profile_summary.py base.log change.log    # side by side
    python3 tools/loop_profile_summary.py capture.log --csv out.csv

A capture is the console text, with or without a host timestamp in front of
each line. The first window is skipped by default (--skip): it starts at the end
of the boot benchmark, before the filter has settled.
"""
import argparse
import csv
import re
import sys

WINDOW_RE = re.compile(
    r'\[LP\] window ([\d.]+) s cfg (\d) .*\| passes (\d+) = ([\d.]+)/s, mean ([\d.]+) us, '
    r'max (\d+) us \| gated (\d+) ekf (\d+) \| imu (\d+)/s ([\d.]+)/pass \| coverage ([\d.]+) %')
ROW_RE = re.compile(
    r'\[LP\]   (\S+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+(\d+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s*$')
TOTAL_RE = re.compile(r'\[LP\] total cfg (\d) pm0 ([\d.]+) % pm1 ([\d.]+) %')
CORE0_RE = re.compile(r'\[LP\] core0 idle ([\d.]+) %')

# What each configuration's two counters are (loop_profile.h, kPmuCfgs).
CFG_NAMES = {0: ('Istall%', 'Dstall%'), 1: ('IPCx100', 'int%'), 2: ('Ibusy%', 'waiti%')}

# Section groups for the per-gated-pass view (tr_loop_profile.h's sections).
GROUPS = [
    ('every pass', ['yield+wdt', 'pre', 'imu_drain', 'sensors', 'cfg+magcal',
                    'led+camera', 'diag_print', 'profile_report']),
    ('imu, baro, orientation', ['gate', 'imu_mean', 'baro_alt', 'orient', 'ekf_prep']),
    ('EKF', ['ekf_init', 'ekf.in', 'ekf.nav', 'ekf.cov', 'ekf.accel', 'ekf.mag',
             'ekf.gnss', 'ekf.out', 'ekf.zupt+logs', 'ekf.baro']),
    ('EKF output', ['ekf_extract']),
    ('I2C poll, commands', ['i2c_poll', 'cmd_dispatch', 'pad_cal']),
    ('kinematics, gates, state', ['kinematics', 'deploy_gates', 'test_modes',
                                  'state_machine', 'sim']),
    ('telemetry', ['telemetry']),
]
EKF_SECTIONS = GROUPS[2][1]


def parse(lines, skip=1):
    """Windows of a capture, in order, each {'cfg', header fields, 'rows', 'total'}."""
    windows = []
    cur = None
    for line in lines:
        m = WINDOW_RE.search(line)
        if m:
            cur = {'cfg': int(m.group(2)), 'seconds': float(m.group(1)),
                   'passes_per_s': float(m.group(4)), 'pass_mean_us': float(m.group(5)),
                   'pass_max_us': int(m.group(6)), 'imu_per_s': int(m.group(9)),
                   'imu_per_pass': float(m.group(10)), 'coverage': float(m.group(11)),
                   'rows': {}, 'total': None, 'core0_idle': None}
            windows.append(cur)
            continue
        if cur is None:
            continue
        m = TOTAL_RE.search(line)
        if m:
            cur['total'] = (float(m.group(2)), float(m.group(3)))
            continue
        m = CORE0_RE.search(line)
        if m:
            cur['core0_idle'] = float(m.group(1))
            continue
        m = ROW_RE.search(line)
        if m and m.group(1) != 'section':
            hz, upp, uph, mx, core, p0, p1 = (float(m.group(i)) for i in range(2, 9))
            cur['rows'][m.group(1)] = dict(hz=hz, upp=upp, uph=uph, max=int(mx),
                                           core=core, p0=p0, p1=p1)
    return windows[skip:]


def _mean(v):
    return sum(v) / len(v) if v else float('nan')


def merge(windows):
    """One summary: per-section means over all windows, counter shares per config
    weighted by the section's share of the core."""
    order = []
    for w in windows:
        for name in w['rows']:
            if name not in order:
                order.append(name)
    sections = {}
    for name in order:
        rows = [(w['cfg'], w['rows'][name]) for w in windows if name in w['rows']]
        s = {'hz': _mean([r['hz'] for _, r in rows]),
             'upp': _mean([r['upp'] for _, r in rows]),
             'uph': _mean([r['uph'] for _, r in rows]),
             'max': max(r['max'] for _, r in rows),
             'core': _mean([r['core'] for _, r in rows])}
        for cfg in CFG_NAMES:
            sel = [r for c, r in rows if c == cfg]
            weight = sum(r['core'] for r in sel)
            for k, key in enumerate(('p0', 'p1')):
                s[CFG_NAMES[cfg][k]] = (sum(r['core'] * r[key] for r in sel) / weight
                                        if weight else float('nan'))
        sections[name] = s
    totals = {cfg: tuple(_mean([w['total'][k] for w in windows
                                if w['cfg'] == cfg and w['total']]) for k in (0, 1))
              for cfg in CFG_NAMES}
    core0 = [w['core0_idle'] for w in windows if w['core0_idle'] is not None]
    return {'windows': len(windows), 'order': order, 'sections': sections, 'totals': totals,
            'passes_per_s': _mean([w['passes_per_s'] for w in windows]),
            'pass_mean_us': _mean([w['pass_mean_us'] for w in windows]),
            'pass_max_us': max((w['pass_max_us'] for w in windows), default=0),
            'imu_per_s': _mean([w['imu_per_s'] for w in windows]),
            'imu_per_pass': _mean([w['imu_per_pass'] for w in windows]),
            'coverage': _mean([w['coverage'] for w in windows]),
            'core0_idle': _mean(core0) if core0 else None}


def per_gated_pass(summary):
    """Group costs per gated (flight-logic) pass: us_per_hit x hits / gated hits."""
    secs = summary['sections']
    gated = secs.get('gate', {}).get('hz') or float('nan')
    out = []
    for group, names in GROUPS:
        out.append((group, sum(secs[n]['uph'] * secs[n]['hz'] / gated
                               for n in names if n in secs)))
    ekf_hz = secs.get('ekf.nav', {}).get('hz') or float('nan')
    ekf_tick = sum(secs[n]['uph'] * secs[n]['hz'] for n in EKF_SECTIONS if n in secs) / ekf_hz
    return out, gated, ekf_hz, ekf_tick


def print_summary(name, s, out=None):
    p = lambda *a: print(*a, file=out or sys.stdout)
    p(f"{name}: {s['windows']} windows; {s['passes_per_s']:.1f} passes/s, mean "
      f"{s['pass_mean_us']:.1f} us, max {s['pass_max_us']} us; imu {s['imu_per_s']:.0f}/s "
      f"({s['imu_per_pass']:.1f}/pass); coverage {s['coverage']:.1f} %"
      + (f"; core 0 idle {s['core0_idle']:.1f} %" if s['core0_idle'] is not None else ''))
    for cfg, (a, b) in s['totals'].items():
        p(f"  whole pass, cfg {cfg}: {CFG_NAMES[cfg][0]} {a:.1f}, {CFG_NAMES[cfg][1]} {b:.1f}")
    head = ''.join(f"{CFG_NAMES[c][k]:>9s}" for c in CFG_NAMES for k in (0, 1))
    p(f"  {'section':15s} {'hits/s':>8s} {'us/pass':>8s} {'us/hit':>8s} {'max_us':>7s} "
      f"{'core%':>6s}{head}")
    for n in s['order']:
        r = s['sections'][n]
        shares = ''.join(f"{r[CFG_NAMES[c][k]]:9.1f}" for c in CFG_NAMES for k in (0, 1))
        p(f"  {n:15s} {r['hz']:8.1f} {r['upp']:8.1f} {r['uph']:8.1f} {r['max']:7d} "
          f"{r['core']:6.1f}{shares}")


def print_compare(names, summaries, out=None):
    p = lambda *a: print(*a, file=out or sys.stdout)
    w = max(12, max(len(n) for n in names) + 2)
    p(f"{'us per hit':24s}" + ''.join(f"{n:>{w}s}" for n in names))
    order = []
    for s in summaries:
        for n in s['order']:
            if n not in order:
                order.append(n)
    for sec in order:
        p(f"{sec:24s}" + ''.join(
            f"{s['sections'][sec]['uph']:{w}.1f}" if sec in s['sections'] else f"{'-':>{w}s}"
            for s in summaries))
    p('')
    p(f"{'per gated pass, us':24s}" + ''.join(f"{n:>{w}s}" for n in names))
    groups = [per_gated_pass(s) for s in summaries]
    for i, (group, _) in enumerate(GROUPS):
        p(f"{group:24s}" + ''.join(f"{g[0][i][1]:{w}.1f}" for g in groups))
    p(f"{'total':24s}" + ''.join(f"{sum(v for _, v in g[0]):{w}.1f}" for g in groups))
    p(f"{'gated passes/s':24s}" + ''.join(f"{g[1]:{w}.1f}" for g in groups))
    p(f"{'EKF Hz':24s}" + ''.join(f"{g[2]:{w}.1f}" for g in groups))
    p(f"{'EKF us per tick':24s}" + ''.join(f"{g[3]:{w}.1f}" for g in groups))


def write_csv(path, s):
    cols = ['hz', 'upp', 'uph', 'max', 'core'] + [CFG_NAMES[c][k] for c in CFG_NAMES
                                                  for k in (0, 1)]
    with open(path, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['section', 'hits_per_s', 'us_per_pass', 'us_per_hit', 'max_us', 'core_pct']
                    + [CFG_NAMES[c][k] for c in CFG_NAMES for k in (0, 1)])
        for n in s['order']:
            wr.writerow([n] + [round(s['sections'][n][c], 2) for c in cols])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('logs', nargs='+', help='console captures of a TR_LOOP_PROFILE build')
    ap.add_argument('--skip', type=int, default=1, help='windows to drop at the start (default 1)')
    ap.add_argument('--csv', help='write the (first) merged table as CSV')
    a = ap.parse_args(argv)
    summaries = []
    for log in a.logs:
        with open(log, errors='replace') as f:
            windows = parse(f, a.skip)
        if not windows:
            sys.exit(f"{log}: no [LP] windows (built with -DTR_LOOP_PROFILE=1?)")
        summaries.append(merge(windows))
        print_summary(log.split('/')[-1], summaries[-1])
        print()
    if len(summaries) > 1:
        print_compare([l.split('/')[-1].rsplit('.', 1)[0] for l in a.logs], summaries)
    if a.csv:
        write_csv(a.csv, summaries[0])


if __name__ == '__main__':
    main()
