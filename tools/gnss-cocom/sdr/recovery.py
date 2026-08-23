#!/usr/bin/env python3
"""Measure how long the receiver takes to give position back.

correlate.py answers "where is the threshold". This answers the other half:
once a flight stops exceeding a limit, how many seconds pass before there is a
fix again? That is the number that decides whether a rocket has usable position
during coast, and it cannot be read off a one-way ramp -- it needs a trajectory
that comes back, which is what make_flights.py produces.

For each blocked window the scenario predicts, this reports:

  shut      how long after the limit was first exceeded the fix actually went
            away. Negative would mean the receiver blanked early.
  clear     the moment the trajectory stopped exceeding any limit.
  recovered the first epoch after that with a fix again.
  latency   recovered - clear. The answer.

A window whose recovery never arrives before the next window opens is reported
as such rather than being silently attributed to the following window.
"""

from __future__ import annotations

import argparse
import statistics
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from correlate import Truth, collect, parse_start, trajectory_time  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", type=Path)
    ap.add_argument("-s", "--scenario", required=True, type=Path)
    ap.add_argument("-t", "--start", required=True)
    args = ap.parse_args()

    meta = json.loads(args.scenario.read_text())
    truth = Truth(meta)
    blocked = meta.get("blocked_windows") or []
    if not blocked:
        raise SystemExit(f"{args.scenario.name} has no blocked_windows; this "
                         "analysis needs a flight profile from make_flights.py")

    samples = collect(args.capture)
    if not samples:
        raise SystemExit(f"no receiver epochs in {args.capture}")
    start = parse_start(args.start)

    rows = []
    for s in samples:
        t, src = trajectory_time(s, start, None)
        if src == "host":
            continue                      # unanchored epochs cannot be placed
        rows.append((t, s[4], s[5]))
    rows.sort(key=lambda r: r[0])
    rows = [r for r in rows if r[0] <= truth.duration + 0.5]
    if not rows:
        raise SystemExit("no epoch carried a receiver clock")

    def fix_at(t):
        """Verdict of the epoch nearest t, or None if the capture has a hole."""
        best = min(rows, key=lambda r: abs(r[0] - t))
        return best if abs(best[0] - t) <= 3.0 else None

    print(f"scenario  : {meta['scenario']}  ({truth.duration:.0f} s, apogee "
          f"{meta['crossings']['peak_alt_m']/1000:.1f} km, peak "
          f"{meta['crossings']['peak_speed_mps']:.0f} m/s)")
    print(f"capture   : {args.capture.name}  ({len(rows)} placed epochs)")

    pre = [r for r in rows if r[0] < blocked[0][0]]
    locked = sum(1 for r in pre if r[1] == "FIX")
    print(f"prologue  : {locked}/{len(pre)} epochs with a fix before the first "
          f"limit was exceeded")
    if locked < 10:
        print("  WARNING: the receiver was barely locked before the flight "
              "started.\n           Recovery latencies below are not "
              "trustworthy -- re-run.")

    # Satellite count during the wait is printed alongside the latency because
    # without it the number is unreadable: every long latency measured here
    # turned out to be the receiver re-acquiring satellites, not the gate being
    # slow. A latency with satellites in hand means the gate; a latency with
    # none means the signal, and the two want opposite fixes.
    print(f"\n{'window':>7} {'limit':<10} {'shut at':>9} {'lag':>6} "
          f"{'clear at':>9} {'recovered':>10} {'latency':>8} {'sats in wait':>13}")
    print("-" * 84)

    results = []
    for i, (w_start, w_end) in enumerate(blocked, 1):
        nxt = blocked[i][0] if i < len(blocked) else truth.duration + 1e9

        tr = truth.at(w_start)
        limit = ("velocity" if tr and tr["speed_mps"] > meta["limits"]["velocity_mps"]
                 else "altitude")

        shut = next((r[0] for r in rows if r[0] >= w_start and r[1] != "FIX"), None)
        lag = (shut - w_start) if shut is not None else None

        rec = next((r[0] for r in rows
                    if r[0] > w_end and r[1] == "FIX" and r[0] < nxt), None)
        lat = (rec - w_end) if rec is not None else None

        wait_end = rec if rec is not None else min(nxt, truth.duration)
        during = [r[2].tracked_sats() for r in rows if w_end <= r[0] <= wait_end]
        sat_lo = min(during) if during else None
        sat_hi = max(during) if during else None
        # The bare minimum is not a safe basis for "was the receiver starved?".
        # One transient epoch drags it to zero: the Air530 tracks 12-14
        # satellites at 50 dBHz continuously and still drops a single GSV set
        # every ~70 s, which made a genuinely slow gate read as re-acquisition
        # and got a real finding dismissed. Judge on the median, and report how
        # much of the wait was actually starved.
        sat_med = statistics.median(during) if during else None
        starved = sum(1 for x in during if x < 4)
        starved_frac = (starved / len(during)) if during else 0.0
        sats_txt = (f"{sat_lo}-{sat_hi} (med {sat_med:.0f})"
                    if sat_lo is not None else "--")

        print(f"{i:>7} {limit:<10} "
              f"{(f'{shut:.1f}s' if shut is not None else 'never'):>9} "
              f"{(f'{lag:+.1f}' if lag is not None else '--'):>6} "
              f"{w_end:>8.1f}s "
              f"{(f'{rec:.1f}s' if rec is not None else 'never'):>10} "
              f"{(f'{lat:.1f}s' if lat is not None else '--'):>8} "
              f"{sats_txt:>13}")
        results.append(dict(window=i, limit=limit, shut_at=shut, shut_lag=lag,
                            clear_at=w_end, recovered_at=rec, latency_s=lat,
                            sats_min=sat_lo, sats_max=sat_hi, sats_med=sat_med,
                            starved_frac=starved_frac))

    print()
    got = [r for r in results if r["latency_s"] is not None]
    if got:
        for r in got:
            # State at the moment of recovery, not at clear_at: clear_at is the
            # last still-blocked sample, so quoting it prints a speed above the
            # limit next to the words "the limit cleared".
            tr = truth.at(r["recovered_at"])
            where = (f"{tr['alt_m']/1000:.1f} km, {tr['speed_mps']:.0f} m/s"
                     if tr else "")
            note = ""
            med, frac = r.get("sats_med"), r.get("starved_frac", 0.0)
            if med is not None and med < 4:
                note = (f"  -- but the median was only {med:.0f} satellite(s) "
                        f"during the wait,\n          so this is re-acquisition "
                        f"time, not the gate")
            elif frac > 0.25:
                note = (f"  -- {100*frac:.0f}% of the wait had fewer than 4 "
                        f"satellites,\n          so treat this latency as a "
                        f"mix of gate and re-acquisition")
            elif med is not None and med >= 4:
                note = (f"  -- {med:.0f} satellites held throughout, so this is "
                        f"the gate,\n          not re-acquisition")
            print(f"RECOVERY: {r['limit']} gate re-opened {r['latency_s']:.1f} s "
                  f"after the limit cleared,\n          at {where}{note}")
    missed = [r for r in results if r["latency_s"] is None
              and r["shut_at"] is not None]
    for r in missed:
        med = r.get("sats_med")
        held = (f" The receiver held a median of {med:.0f} satellites through "
                f"that whole\n              interval, so the position was being "
                f"withheld, not searched for."
                if med is not None and med >= 4 else "")
        print(f"NO RECOVERY in window {r['window']} ({r['limit']}): no fix "
              f"between {r['clear_at']:.1f}s and the next limit exceedance.{held}")
    if not got and not missed:
        print("The receiver never lost its fix. Either the flight never "
              "exceeded a limit\nor it was not locked to begin with.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
