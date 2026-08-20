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

    print(f"\n{'window':>7} {'limit':<10} {'shut at':>9} {'lag':>6} "
          f"{'clear at':>9} {'recovered':>10} {'latency':>8}")
    print("-" * 68)

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

        print(f"{i:>7} {limit:<10} "
              f"{(f'{shut:.1f}s' if shut is not None else 'never'):>9} "
              f"{(f'{lag:+.1f}' if lag is not None else '--'):>6} "
              f"{w_end:>8.1f}s "
              f"{(f'{rec:.1f}s' if rec is not None else 'never'):>10} "
              f"{(f'{lat:.1f}s' if lat is not None else '--'):>8}")
        results.append(dict(window=i, limit=limit, shut_at=shut, shut_lag=lag,
                            clear_at=w_end, recovered_at=rec, latency_s=lat))

    print()
    got = [r for r in results if r["latency_s"] is not None]
    if got:
        for r in got:
            tr = truth.at(r["clear_at"])
            where = (f"{tr['alt_m']/1000:.1f} km, {tr['speed_mps']:.0f} m/s"
                     if tr else "")
            print(f"RECOVERY: {r['limit']} gate re-opened {r['latency_s']:.1f} s "
                  f"after the limit cleared ({where})")
    missed = [r for r in results if r["latency_s"] is None
              and r["shut_at"] is not None]
    for r in missed:
        print(f"NO RECOVERY in window {r['window']} ({r['limit']}): no fix "
              f"between {r['clear_at']:.1f}s and the next limit exceedance.")
    if not got and not missed:
        print("The receiver never lost its fix. Either the flight never "
              "exceeded a limit\nor it was not locked to begin with.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
