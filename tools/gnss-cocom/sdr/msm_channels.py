#!/usr/bin/env python3
"""Per-channel tracking from a capture's RTCM MSM7 (the LC86G's raw measurements).

GSV says which satellites a receiver is tracking about once a second, as an
integer C/N0. MSM7 says, per satellite: C/N0 to 1/16 dB, the receiver's own
Doppler, and a lock-time counter that resets when that channel's loop drops.
That is what separates two very different ways to lose a fix under boost:

  * per-channel loop stress, where the satellites with the steepest Doppler
    rate (high elevation, for a vertical flight) go first and the rest hold;
  * a common-mode failure, where every channel drops at once regardless of
    its own dynamics -- the receiver's own aiding or clock, not the loops.

    ./msm_channels.py captures/lc86g_balloon_gentle_alt.log \\
        -s scenarios/gentle_alt.json -t 2026/08/18,08:30:00 --window 170 240

For each satellite it prints C/N0 and Doppler through the window, and marks a
channel LOST when its cell disappears or its lock time goes backwards.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import rtcm3                                                   # noqa: E402
from gnss_nmea_monitor import replay_source                    # noqa: E402
from correlate import Truth, parse_start                       # noqa: E402


def epochs(capture, tow0):
    """[(traj_t, {prn: cell})] from every GPS MSM7 in the capture."""
    out = []
    for _t, kind, data in replay_source(str(capture)):
        if kind != "rtcm" or rtcm3.message_type(data) != 1077:
            continue
        r = rtcm3.parse_msm7(data)
        if not r:
            continue
        _sys, ms, cells = r
        out.append((ms / 1000.0 - tow0, {c["prn"]: c for c in cells}))
    # The first frames after a cold start carry the previous run's clock; they
    # sit far from the rest in time and are simply out of order. Keep the run.
    out.sort(key=lambda e: e[0])
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", type=Path)
    ap.add_argument("-s", "--scenario", required=True, type=Path)
    ap.add_argument("-t", "--start", required=True)
    ap.add_argument("--window", nargs=2, type=float, metavar=("T0", "T1"),
                    default=(170.0, 240.0))
    args = ap.parse_args()

    meta = json.loads(args.scenario.read_text())
    truth = Truth(meta)
    _w, tow0, _sod = parse_start(args.start)
    ep = [e for e in epochs(args.capture, tow0)
          if 0.0 <= e[0] <= meta["duration_s"] + 1]
    if not ep:
        raise SystemExit("no GPS MSM7 (1077) in this capture")

    t0, t1 = args.window
    win = [e for e in ep if t0 <= e[0] <= t1]
    prns = sorted({p for _, cells in win for p in cells})
    print(f"{len(ep)} MSM7 epochs in the run, {len(win)} in {t0:.0f}-{t1:.0f} s, "
          f"{len(prns)} GPS satellites\n")

    # C/N0 grid: one column per epoch, '.' where the cell is absent.
    head = "  t (s)  " + " ".join(f"G{p:02d}" for p in prns) + \
           "   alt km  speed  accel g"
    print(head)
    prev = {}
    lost = []
    for t, cells in win:
        tr = truth.at(t)
        a = None
        if tr:
            b = truth.at(t + 0.5); c = truth.at(t - 0.5)
            if b and c and b["t"] != c["t"]:
                a = (b["speed_mps"] - c["speed_mps"]) / (b["t"] - c["t"]) / 9.80665
        row = []
        for p in prns:
            cell = cells.get(p)
            if cell is None or cell["cn0"] is None:
                row.append("  . ")
                if p in prev:
                    lost.append((t, p, "cell gone"))
                    prev.pop(p, None)
                continue
            if p in prev and cell["lock_ms"] < prev[p]["lock_ms"]:
                lost.append((t, p, f"lock reset {prev[p]['lock_ms']}->{cell['lock_ms']} ms"))
            row.append(f"{cell['cn0']:4.0f}")
            prev[p] = cell
        print(f"{t:7.1f}  " + " ".join(row) +
              (f"   {tr['alt_m']/1000:6.2f} {tr['speed_mps']:6.0f} "
               f"{'' if a is None else f'{a:6.1f}'}" if tr else ""))

    print("\nchannel events:")
    for t, p, why in lost:
        print(f"  {t:7.1f} s  G{p:02d}  {why}")
    if not lost:
        print("  none -- every channel held through the window")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
