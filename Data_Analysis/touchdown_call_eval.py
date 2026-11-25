#!/usr/bin/env python3
"""Score the apps' predicted landing call against real flights.

The apps (iOS `TouchdownPredictor.swift`, Android `TouchdownPredictor.kt`)
call the landing at a predicted touchdown when the base station loses the
rocket before the landed frame gets through, which is what usually happens
at touchdown.  This replays the base station's own LoRa logs (what the phone
heard) through the same rule and scores it against each flight computer's
log of the same flight.

For every LoRa flight with a matching flight-computer CSV in the same folder
(or a sibling folder):
  - LoRa time is aligned to flight-computer time by the launch edge, refined
    by matching the descent pressure altitude;
  - touchdown = the first moment after apogee, low in the descent, after
    which the flight computer's baro moves less than 1 m in 1 s;
  - the call is made as the apps make it: remaining height (floored at pad
    level) over the mean descent rate of the last RATE_WINDOW_S of live
    frames, but not before SILENCE_S without a live frame.

Reported: the call time on flights that really lost the landing, the call
time had every flight lost the link after its last live frame, and the
prediction error had the link been cut at fixed heights.

Usage
-----
    python touchdown_call_eval.py <TestFlights dir>
    python touchdown_call_eval.py <dir> --main 04_Rolly_Polly=243.84:6.5

`--main NAME=ALT_M:RATE_MPS` gives a flight whose path contains NAME a main
charge at ALT_M on descent and a main descent rate, as the profile's pyro
channels and main rate would in the app.
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

import numpy as np
import pandas as pd

# Mirrors TouchdownPredictor (Swift and Kotlin).
RATE_WINDOW_S = 3.0
SILENCE_S = 1.5
MIN_DESCENT_RATE_MPS = 1.0
MAIN_OPEN_RATIO = 2.0

CUTS_M = [400, 300, 200, 150, 100, 60, 30, 15]


def lora_flights(path):
    """(rocket id, frames) per flight in one base-station LoRa CSV."""
    def ok(r):
        try:
            float(r['time_ms']); float(r['pressure_alt']); float(r['alt_rate'])
            return r.get('state') not in (None, '', 'EVENT')
        except (TypeError, ValueError, KeyError):
            return False
    rows = [r for r in csv.DictReader(open(path, newline='', errors='replace')) if ok(r)]
    by_rid = {}
    for r in rows:
        by_rid.setdefault(r.get('rocket_id') or '?', []).append(r)
    for rid, rr in by_rid.items():
        i = 0
        while i < len(rr):
            if rr[i].get('launch') != '1':
                i += 1
                continue
            j = i
            while j < len(rr) and not (rr[j]['state'] in ('READY', 'PRELAUNCH') and j > i + 5):
                j += 1
            fr = rr[i:j]
            alt = np.array([float(r['pressure_alt']) for r in fr])
            if alt.max() >= 30:
                yield rid, dict(
                    t=np.array([float(r['time_ms']) / 1000 for r in fr]),
                    alt=alt,
                    rate=np.array([float(r['alt_rate']) for r in fr]),
                    state=[r['state'] for r in fr],
                    landed=np.array([r['landed'] == '1' for r in fr]))
            i = j


def fc_truth(path):
    """Launch, apogee and touchdown from a flight-computer CSV, or None."""
    cols = ['Time (ms)', 'Pressure Altitude (m)', 'Landed Flag', 'Launch Flag']
    df = pd.read_csv(path, usecols=lambda c: c in cols).dropna(subset=['Pressure Altitude (m)'])
    t = df['Time (ms)'].to_numpy() / 1000.0
    alt = df['Pressure Altitude (m)'].to_numpy()
    launch = df['Launch Flag'].to_numpy() > 0
    landed = df['Landed Flag'].to_numpy() > 0
    if not launch.any():
        return None
    il = np.argmax(launch)
    # 20 Hz bins (median per bin), then a 5-bin running median
    g = pd.DataFrame({'b': np.floor((t - t[0]) / 0.05).astype(int), 'a': alt, 't': t}) \
        .groupby('b').agg(a=('a', 'median'), t=('t', 'mean'))
    bt = g['t'].to_numpy()
    sm = pd.Series(g['a'].to_numpy()).rolling(5, center=True, min_periods=1).median().to_numpy()
    ia = np.searchsorted(bt, t[il]) + np.argmax(sm[np.searchsorted(bt, t[il]):])
    # Logs stop about 2 s after the landed flag, so the stillness window is 1 s.
    lim = min(sm[ia] / 3, 60)
    td = next((bt[i] for i in range(ia, len(sm) - 20)
               if sm[i] < lim and abs(sm[i + 20] - sm[i]) < 1.0), None)
    if td is None:
        return None
    fl = t[landed]
    fl = fl[fl > bt[ia]]
    return dict(t_launch=t[il], apogee=sm[ia], touchdown=td,
                landed_flag=(fl[0] if len(fl) else np.nan), t=bt, sm=sm)


def align(lf, fc):
    """(offset to add to LoRa time for flight-computer time, rms metres)."""
    off0 = fc['t_launch'] - lf['t'][0]
    m = (np.arange(len(lf['t'])) > np.argmax(lf['alt'])) & (lf['alt'] > 10)
    if m.sum() < 5:
        return off0, np.nan
    best = (np.inf, off0)
    for d in np.arange(-2.0, 2.0, 0.01):
        a = np.interp(lf['t'][m] + off0 + d, fc['t'], fc['sm'])
        res = np.sqrt(np.mean((a - lf['alt'][m]) ** 2))
        if res < best[0]:
            best = (res, off0 + d)
    return best[1], best[0]


def predict(t, alt, rate, k, main=None):
    """Predicted touchdown from frames 0..k, or None (TouchdownPredictor.observe)."""
    win = (t >= t[k] - RATE_WINDOW_S) & (np.arange(len(t)) <= k)
    r = -np.mean(rate[win])
    if r < MIN_DESCENT_RATE_MPS:
        return None
    h = alt[k]
    if h <= 0:
        return t[k]
    if main and r > MAIN_OPEN_RATIO * main[1]:
        return t[k] + max(h - main[0], 0) / r + min(h, main[0]) / main[1]
    return t[k] + h / r


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('root')
    ap.add_argument('--main', action='append', default=[], metavar='NAME=ALT_M:RATE_MPS')
    args = ap.parse_args()
    mains = {}
    for spec in args.main:
        name, v = spec.split('=')
        a, r = v.split(':')
        mains[name.replace('_', ' ')] = (float(a), float(r))

    fc_cache = {}
    results = []
    seen = set()
    for lpath in sorted(glob.glob(os.path.join(args.root, '**', 'lora_*.csv'), recursive=True)):
        if '_rocket' in lpath:      # per-rocket splits of a log already read
            continue
        d = os.path.dirname(lpath)
        fcs = sorted(set(glob.glob(os.path.join(d, 'flight_*.csv')) +
                         glob.glob(os.path.join(os.path.dirname(d), '*', 'flight_*.csv'))))
        for rid, lf in lora_flights(lpath):
            best = None
            for fpath in fcs:
                if fpath not in fc_cache:
                    try:
                        fc_cache[fpath] = fc_truth(fpath)
                    except Exception as e:      # odd or truncated logs
                        print('skip', fpath, e, file=sys.stderr)
                        fc_cache[fpath] = None
                fc = fc_cache[fpath]
                if fc is None:
                    continue
                off, res = align(lf, fc)
                if not np.isnan(res) and (best is None or res < best[2]):
                    best = (fpath, fc, res, off)
            if not best or best[2] >= 8 or (os.path.basename(best[0]), rid) in seen:
                continue
            seen.add((os.path.basename(best[0]), rid))
            fpath, fc, _, off = best
            name = os.path.dirname(os.path.relpath(fpath, args.root))
            t = lf['t'] + off
            td = fc['touchdown']
            live = np.array([s == 'INFLIGHT' and not l for s, l in zip(lf['state'], lf['landed'])])
            desc = np.nonzero(live & (np.arange(len(t)) > np.argmax(lf['alt'])))[0]
            if len(desc) == 0:
                continue
            k = desc[-1]
            landed_t = t[lf['landed'] | np.array([s == 'LANDED' for s in lf['state']])]
            tp = predict(t, lf['alt'], lf['rate'], k)
            main_cfg = next((v for n, v in mains.items() if n in name), None)
            cuts = {}
            for c in CUTS_M:
                cand = [j for j in desc if lf['alt'][j] <= c]
                if not cand or lf['alt'][desc[0]] < c or lf['alt'][cand[0]] < c - 25:
                    continue
                j = cand[0]
                p = predict(t, lf['alt'], lf['rate'], j)
                pm = predict(t, lf['alt'], lf['rate'], j, main_cfg) if main_cfg else None
                cuts[c] = (None if p is None else p - td, td - t[j], None if pm is None else pm - td)
            results.append(dict(
                name=name, rid=rid,
                lost=not np.any(np.abs(landed_t - td) < 6),
                last_alt=lf['alt'][k], last_vs_td=t[k] - td,
                call=None if tp is None else max(tp, t[k] + SILENCE_S) - td,
                landed_flag=fc['landed_flag'] - td, cuts=cuts, main=main_cfg))

    fmt = lambda x: '   -  ' if x is None else f"{x:+6.1f}"
    print(f"rate window {RATE_WINDOW_S} s, silence before the call {SILENCE_S} s, "
          f"{len(results)} flights\n")
    print("LoRa lost the landing (no landed frame within 6 s of touchdown):")
    for o in results:
        if o['lost']:
            print(f"  {o['name'][:44]:44s} rid {o['rid']:>2}: last live {o['last_vs_td']:+5.1f} s "
                  f"at {o['last_alt']:+4.0f} m -> call {fmt(o['call'])} s vs touchdown")
    calls = [o['call'] for o in results if o['call'] is not None]
    print("\nHad every flight lost the link after its last live descent frame:")
    print(f"  n={len(calls)}  call vs touchdown: median {np.median(calls):+.1f} s, "
          f"min {min(calls):+.1f}, max {max(calls):+.1f}")
    print(f"  flight-computer landed flag vs touchdown: median "
          f"{np.nanmedian([o['landed_flag'] for o in results]):+.1f} s")
    print("\nPrediction error (s) had the link been cut at a height:")
    for c in CUTS_M:
        rows = [(o['cuts'][c][0], o['cuts'][c][1], o['name']) for o in results
                if c in o['cuts'] and o['cuts'][c][0] is not None]
        if rows:
            w = max(rows, key=lambda x: abs(x[0]))
            print(f"  {c:4d} m: n={len(rows):2d} median |err| "
                  f"{np.median([abs(x[0]) for x in rows]):4.1f} s, "
                  f"worst {w[0]:+6.1f} s of {w[1]:4.0f} s ({w[2][:32]})")
    for o in results:
        if o['main']:
            print(f"\nWith a main at {o['main'][0]} m, {o['main'][1]} m/s: {o['name']} rid {o['rid']}")
            for c, v in o['cuts'].items():
                print(f"  {c:4d} m: plain {fmt(v[0])} s, with the main leg {fmt(v[2])} s, "
                      f"of {v[1]:3.0f} s")


if __name__ == '__main__':
    main()
