#!/usr/bin/env python3
"""Test whether a receiver's C/N0 blanking is tied to the COCOM gate.

The Air530 blanks every satellite's C/N0 in a single epoch on a tight ~18 s
cycle. Two observations already place that inside the receiver rather than on
the bench -- a control part on the same conducted path shows none of it, and
every satellite blanks and returns together rather than weakest-first. This
answers the remaining question: does it happen *only* while position is being
withheld?

The distinction matters because it decides what the artifact is evidence of. If
blanking is confined to blocked intervals it is part of the gate's behaviour --
something the receiver does while withholding. If it runs continuously it is an
unrelated housekeeping cycle that merely became visible during a long block, and
says nothing about the gate at all.

A "blank" here is an epoch reporting six or more satellites where at most one
carries a nonzero C/N0. That threshold matters: taking any single satellite at
zero would catch ordinary rise-and-set, and taking all of them would miss the
one satellite that survives each event.

    ./blanking.py -s scenarios/blockdur.json -t 2026/08/19,22:30:00 \\
        captures/air530_blockdur.log
"""

from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

from gnss_nmea_monitor import Parser, replay_source        # noqa: E402
from correlate import Truth, parse_start                   # noqa: E402

MIN_SATS = 6          # below this an epoch is too sparse to call
MAX_NONZERO = 1       # one satellite typically survives each event


def epochs(cap: Path):
    """(t, is_blank, n_tracked) per navigation epoch, protocol-agnostic."""
    p = Parser()
    out = []
    for t, kind, data in replay_source(str(cap)):
        if kind == "nmea":
            p.feed(data)
            if data[3:6] != "GGA":
                continue
        elif kind == "ubx":
            p.feed_ubx(data)
            if not (len(data) >= 2 and data[0] == 0x01 and data[1] == 0x07):
                continue
        elif kind == "bin":
            p.feed_binary(data)
            if not (data and data[0] == 0xDF):
                continue
        else:
            continue
        cn0 = p.epoch.cn0 or {}
        if not cn0:
            continue
        nonzero = sum(1 for v in cn0.values() if v)
        # State must come from the FIX FLAG, never from verdict(): a blank
        # epoch has every C/N0 at zero, so its tracked-satellite count is zero
        # and verdict() is forced to NO_LOCK. Splitting on the verdict therefore
        # asks whether a blank epoch is blank, and every bucket reads zero.
        out.append((t, len(cn0) >= MIN_SATS and nonzero <= MAX_NONZERO,
                    p.epoch.tracked_sats(),
                    "FIX" if (p.epoch.fix_quality or 0) > 0 else "BLOCKED"))
    return out


def in_any(t, windows, pad=0.0):
    return any(a - pad <= t <= b + pad for a, b in windows)


def period_of(times, cap=120.0):
    """Median spacing between distinct blanking events."""
    if len(times) < 3:
        return None, []
    ev = [times[0]]
    for t in times[1:]:
        if t - ev[-1] > 5.0:
            ev.append(t)
    gaps = [ev[i + 1] - ev[i] for i in range(len(ev) - 1)]
    inner = [g for g in gaps if g < cap]
    return (statistics.median(inner) if inner else None), ev


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("captures", nargs="+", type=Path)
    ap.add_argument("-s", "--scenario", required=True, type=Path)
    ap.add_argument("-t", "--start", required=True)
    ap.add_argument("--pad", type=float, default=2.0,
                    help="seconds of slack around a window edge, for gate latency")
    args = ap.parse_args()

    import json
    meta = json.loads(args.scenario.read_text())
    windows = meta.get("blocked_windows") or []
    if not windows:
        print("!! this scenario declares no blocked windows; nothing to compare against")
        return 1
    blocked_s = sum(b - a for a, b in windows)
    print(f"scenario {meta['scenario']}: {len(windows)} blocked window(s), "
          f"{blocked_s:.0f}s of {meta['duration_s']:.0f}s "
          f"({100*blocked_s/meta['duration_s']:.0f}%)\n")

    for cap in args.captures:
        if not cap.exists():
            print(f"{cap.name}: missing")
            continue
        ev = epochs(cap)
        if not ev:
            print(f"{cap.name}: no usable epochs")
            continue
        # Host time in the capture is already trajectory-relative for these runs.
        # Drop anything past the end of the transmitted file: when the .C8 runs
        # out the receiver coasts and then loses everything, which produces
        # blank epochs that have nothing to do with either the limit or the gate.
        ev = [e for e in ev if e[0] <= meta["duration_s"]]

        print(f"{cap.name}")
        print(f"  {'':<34} {'epochs':>8} {'blank':>7} {'rate':>8}")

        # (a) against the SCENARIO: was a limit being exceeded?
        inside = [e for e in ev if in_any(e[0], windows, args.pad)]
        outside = [e for e in ev if not in_any(e[0], windows, args.pad)]
        bi = [e for e in inside if e[1]]
        bo = [e for e in outside if e[1]]
        for lab, seg, bl in (("limit exceeded (scenario)", inside, bi),
                             ("limit not exceeded", outside, bo)):
            rate = (100 * len(bl) / len(seg)) if seg else 0.0
            print(f"  {lab:<34} {len(seg):>8} {len(bl):>7} {rate:>7.1f}%")

        # (b) against the RECEIVER: was IT actually withholding? For a part as
        # latent as the Air530 these two are very different questions -- it stays
        # blocked long after the limit clears -- and only this one asks whether
        # blanking accompanies withholding.
        held = [e for e in ev if e[3] == "FIX"]        # publishing a position
        with_held = [e for e in ev if e[3] == "BLOCKED"]  # not publishing one
        bh = [e for e in held if e[1]]
        bw = [e for e in with_held if e[1]]
        for lab, seg, bl in (("receiver withholding (BLOCKED)", with_held, bw),
                             ("receiver publishing (FIX)", held, bh)):
            rate = (100 * len(bl) / len(seg)) if seg else 0.0
            print(f"  {lab:<34} {len(seg):>8} {len(bl):>7} {rate:>7.1f}%")

        p_in, _ = period_of([e[0] for e in bw])
        p_out, _ = period_of([e[0] for e in bh])
        print(f"  period while withholding : {(f'{p_in:.0f}s' if p_in else '--')}")
        print(f"  period while publishing  : {(f'{p_out:.0f}s' if p_out else '--')}")
        bi, bo, inside, outside = bw, bh, with_held, held

        if not bo and bi:
            print("  -> blanking occurs ONLY while position is withheld: it is part\n"
                  "     of the gate's behaviour, not a free-running housekeeping cycle.")
        elif bo and bi and len(bo) / max(1, len(outside)) < 0.2 * (len(bi) / max(1, len(inside))):
            print("  -> overwhelmingly confined to blocked intervals, with a few\n"
                  "     events outside; consistent with the gate rather than a\n"
                  "     free-running cycle.")
        elif bo and not bi:
            print("  -> blanking happens only OUTSIDE the blocked windows, which does\n"
                  "     not fit the gate at all; look for another cause.")
        elif bo and bi:
            print("  -> blanking runs both inside and outside the windows at similar\n"
                  "     rates: it is a free-running cycle and is NOT evidence about\n"
                  "     the gate.")
        else:
            print("  -> no blanking anywhere in this capture.")
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
