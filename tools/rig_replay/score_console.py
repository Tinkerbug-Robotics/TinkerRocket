#!/usr/bin/env python3
"""Score a rig replay's console: the navigation filter's ENU state (the
[REPLAY] lines, once a second) against the flight's truth, with the state
machine's events.

    score_console.py FC_CONSOLE_LOG [--flight sim_rp5_0829 | --csv SENSORS_CSV]

The [REPLAY] position is the filter's ENU from its own pad reference, so its
"up" is relative: an absolute height error on the pad does not show here (the
flight log's score, score_log.py, has it).
"""
import argparse
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from truth import Truth, flight_files  # noqa: E402

RX = re.compile(r"\[REPLAY\] t=([+-][0-9.]+)(\(pad, no clock\))? phase=(\d) clock=(\d) fixes=(\d+) .*?ekf=(\d) "
                r"pos=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\) vel=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)")
EVENTS = ("[STATE]", "RIG REPLAY", "Apogee detected", "Cold start", "LAUNCH", "fix FLAGGED", "Ref pos frozen",
          "[EKF] Init", "Declination", "Mag reference", "Burnout", "DEPLOY", "Deploy", "LANDED")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("console")
    ap.add_argument("--flight", default="sim_rp5_0829", help="its files under $COCOM_SDR/scenarios")
    ap.add_argument("--csv", help="the sensors CSV, if not the flight's")
    args = ap.parse_args()
    tr = Truth(args.csv or flight_files(args.flight)[0])

    rows, events = [], []
    for ln in open(args.console, errors="replace"):
        m = RX.search(ln)
        if m:
            rows.append((float(m.group(1)), m.group(2) is None, int(m.group(3)), int(m.group(4)), int(m.group(5)),
                         int(m.group(6)), *[float(m.group(k)) for k in range(7, 13)]))
        elif any(k in ln for k in EVENTS):
            events.append(ln.rstrip())
    print(f"{len(rows)} [REPLAY] lines")
    for e in events[:60]:
        print("  " + e[:200])
    if not rows:
        return

    print("\n     t  ph clk  fixes ekf |  E err   N err   U err |  vE err  vN err  vU err | truth U   vU")
    errs = []
    for t, have, ph, clk, fixes, ekf, *s in rows:
        if not have or not ekf or t < -20 or t > tr.end:
            continue
        x = tr.at([t])[0]
        p = np.array([x[1], x[0], -x[2]])          # ENU
        v = np.array([x[4], x[3], -x[5]])
        ep, ev = np.array(s[0:3]) - p, np.array(s[3:6]) - v
        errs.append((t, *ep, *ev))
        print(f"{t:+7.2f} {ph:2d} {clk:3d} {fixes:6d} {ekf:3d} | {ep[0]:6.1f} {ep[1]:7.1f} {ep[2]:7.1f} | "
              f"{ev[0]:7.2f} {ev[1]:7.2f} {ev[2]:7.2f} | {p[2]:7.1f} {v[2]:6.1f}")
    if not errs:
        return
    e = np.array(errs)
    print("\nphase (from, to s)        horiz max   up max   vel max")
    for label, lo, hi in tr.windows():
        sel = (e[:, 0] >= lo) & (e[:, 0] < hi)
        if sel.any():
            h = np.hypot(e[sel, 1], e[sel, 2])
            print(f"  {label:7s} {lo:+7.2f} {hi:+7.2f}  {h.max():7.1f} m {np.abs(e[sel, 3]).max():6.1f} m "
                  f"{np.abs(e[sel, 4:7]).max():6.2f} m/s")


if __name__ == "__main__":
    main()
