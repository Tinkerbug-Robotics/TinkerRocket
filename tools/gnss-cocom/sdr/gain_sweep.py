#!/usr/bin/env python3
"""Find the working TX gain for a receiver by sweeping it against a static scene.

Every receiver on this rig has needed a different level, and the right one is
never the highest one. The PX1125R locked solidly at TX gain 22 and never fixed
at 32 -- its front end overdrives -- and when a bad connector later cost ~25 dB
the working point moved to 47. So the level has to be *measured* for each new
part rather than carried over.

**The tell for overdrive is higher C/N0 with fewer satellites.** A front end
being driven into compression reports a strong carrier while losing the weaker
ones, so ranking by C/N0 alone picks exactly the wrong setting. This ranks by
satellites tracked first and only uses C/N0 to break ties, and prints both so
the shape of the curve is visible rather than just its argmax.

The first dwell is longer than the rest: a cold receiver needs to download
ephemeris before it can report anything, but once it has done so re-acquisition
between gain steps takes seconds.
"""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

from ublox_binary import (iter_frames, parse_nav_pvt, parse_nav_sat,   # noqa: E402
                          CLS_NAV, MSG_NAV_PVT, MSG_NAV_SAT)
from ensure_hackrf import ensure_hackrf                                # noqa: E402

TX_ERR = "/tmp/hackrf_gain_sweep.err"


def start_tx(c8: Path, freq: int, rate: int, gain: int):
    errf = open(TX_ERR, "w")
    tx = subprocess.Popen(
        ["hackrf_transfer", "-t", str(c8), "-f", str(freq), "-s", str(rate),
         "-a", "0", "-x", str(gain)],
        stdout=errf, stderr=subprocess.STDOUT)
    time.sleep(3.0)
    out = Path(TX_ERR).read_text()
    if tx.poll() is not None or "MB / " not in out:
        try:
            tx.kill()
        except Exception:
            pass
        return None
    return tx


def stop_tx(tx):
    if tx is None:
        return
    tx.send_signal(signal.SIGINT)
    try:
        tx.wait(timeout=6)
    except subprocess.TimeoutExpired:
        tx.kill()


def dwell(ser, seconds: float):
    """Watch one gain step; return (best_sat_count, median C/N0, got_fix)."""
    import statistics
    buf = bytearray()
    sats, best_n, cn0s, fix = {}, 0, [], False
    ser.reset_input_buffer()
    t0 = time.time()
    while time.time() - t0 < seconds:
        buf.extend(ser.read(ser.in_waiting or 1))
        for cls, mid, pl in iter_frames(buf):
            if cls != CLS_NAV:
                continue
            if mid == MSG_NAV_SAT:
                best = {}
                for x in parse_nav_sat(pl) or []:
                    k = (x["gnss_name"], x["svid"])
                    best[k] = max(best.get(k, -99), x["cn0"])
                sats = best
            elif mid == MSG_NAV_PVT:
                r = parse_nav_pvt(pl)
                if not r:
                    continue
                fix = fix or r["has_fix"]
                live = [v for v in sats.values() if v >= 30]
                if len(live) >= best_n:
                    best_n = len(live)
                    if live:
                        cn0s = sorted(live)
    med = statistics.median(cn0s) if cn0s else 0.0
    return best_n, med, fix


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/cu.usbmodem101")
    ap.add_argument("-s", "--scenario", default="t00_static")
    ap.add_argument("-g", "--gains", default="8,14,20,26,32,38,44,47")
    ap.add_argument("--dwell", type=float, default=35.0)
    ap.add_argument("--first-dwell", type=float, default=75.0,
                    help="longer, to let a cold receiver download ephemeris")
    ap.add_argument("--freq", type=int, default=1575420000)
    ap.add_argument("--rate", type=int, default=2600000)
    args = ap.parse_args()

    import serial
    c8 = HERE / "c8" / f"{args.scenario}.C8"
    if not c8.exists():
        return f"no {c8}"
    if not ensure_hackrf():
        return 1

    gains = [int(g) for g in args.gains.split(",")]
    print(f"# {args.scenario}, {len(gains)} gain steps, "
          f"{args.first_dwell:.0f}s then {args.dwell:.0f}s each\n")
    print(f"  {'gain':>5} {'sats>=30dB':>11} {'median C/N0':>12} {'fix':>5}")
    print("  " + "-" * 40)

    rows = []
    with serial.Serial(args.port, 115200, timeout=0.3) as ser:
        for i, g in enumerate(gains):
            tx = start_tx(c8, args.freq, args.rate, g)
            if tx is None:
                print(f"  {g:>5}   hackrf_transfer would not start")
                continue
            try:
                n, med, fix = dwell(ser, args.first_dwell if i == 0 else args.dwell)
            finally:
                stop_tx(tx)
            rows.append((g, n, med, fix))
            print(f"  {g:>5} {n:>11} {med:>11.1f}  {'yes' if fix else 'no':>5}")

    if not rows:
        return 1
    # Satellites first, C/N0 only to break ties: see the module docstring.
    best = max(rows, key=lambda r: (r[1], r[2]))
    print(f"\n  best: gain {best[0]} -- {best[1]} satellites at {best[2]:.0f} dBHz")
    top = max(r[2] for r in rows)
    if best[2] < top:
        hot = [r for r in rows if r[2] == top][0]
        print(f"  note: gain {hot[0]} reported a higher C/N0 ({hot[2]:.0f} dBHz) with "
              f"{hot[1]} satellites.\n        Higher level, fewer satellites is the "
              "compression signature -- do not use it.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
