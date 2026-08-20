#!/usr/bin/env python3
"""Recover a capture's scenario start time from the capture itself.

build_scenarios.sh picks a start time from the ephemeris and prints it, but
nothing writes it down, and its own instructions say "Record this start time
with every capture -- correlate.py needs it." That was not done, so the start
time for two flight captures had to be reconstructed after the fact. Guessing
wrong does not fail loudly: correlate.py still produces a table, it is simply
aligned against the wrong part of the trajectory.

Rather than trust a note, derive it. Every valid fix carries GPS time-of-week,
and the injected trajectory is known, so the start is the offset that best
lines the receiver's reported altitude up with the altitude that was
transmitted. Candidates are whole minutes, which is what pick_start.py emits.

The residual it reports is the check that matters: a correct alignment sits at
a few tens of metres, and anything above ~1 km means the capture does not
belong to this scenario at all.
"""

from __future__ import annotations

import argparse
import datetime
import json
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

from ublox_binary import CLS_NAV, MSG_NAV_PVT, parse_nav_pvt   # noqa: E402
from correlate import Truth                                     # noqa: E402

GPS_EPOCH = datetime.datetime(1980, 1, 6)
WEEK = 7 * 86400


def fixes_from_capture(cap: Path):
    """(seconds_of_day, alt_m, date) for every epoch that reported a valid fix.

    Seconds-of-day rather than time-of-week, because that is the one unit both
    protocols can express: UBX carries iTOW, NMEA carries a wall clock. The
    scenario start is always a whole minute inside a single day, so nothing is
    lost by folding the week away, and it means one alignment routine serves a
    u-blox, a SkyTraq and an Air530 alike.
    """
    out = []
    for line in cap.read_text(errors="replace").splitlines():
        p = line.split()

        # --- UBX: "<t> U <hex>" ---------------------------------------------
        if len(p) == 3 and p[1] == "U":
            try:
                d = bytes.fromhex(p[2])
            except ValueError:
                continue
            if len(d) > 2 and d[0] == CLS_NAV and d[1] == MSG_NAV_PVT:
                r = parse_nav_pvt(d[2:])
                if r and r["has_fix"]:
                    out.append((r["itow_ms"] / 1000.0 % 86400.0, r["alt_m"], None))
            continue

        # --- NMEA: "<t> $GNGGA,..." ------------------------------------------
        i = line.find("$")
        if i < 0:
            continue
        f = line[i:].split(",")
        if len(f) < 10 or f[0][3:6] != "GGA":
            continue
        try:
            # quality 0 is "no fix"; anything else is a position we can use.
            if int(f[6]) == 0:
                continue
            hh, mm, ss = int(f[1][0:2]), int(f[1][2:4]), float(f[1][4:])
            alt = float(f[9])
        except (ValueError, IndexError):
            continue
        out.append((hh * 3600 + mm * 60 + ss, alt, None))
    return out


def date_from_capture(cap: Path):
    """UTC date from the first valid NMEA RMC, or None (UBX carries its own)."""
    for line in cap.read_text(errors="replace").splitlines():
        i = line.find("$")
        if i < 0:
            continue
        f = line[i:].split(",")
        if len(f) < 10 or f[0][3:6] != "RMC" or f[2] != "A":
            continue
        d = f[9]
        if len(d) == 6 and d.isdigit():
            dd, mo, yy = int(d[0:2]), int(d[2:4]), int(d[4:6])
            return datetime.date(2000 + yy, mo, dd)
    return None


def tow_to_utc(tow: float, near: datetime.datetime) -> datetime.datetime:
    """Absolute UTC for a time-of-week, resolved to the week containing `near`."""
    week = int((near - GPS_EPOCH).days // 7)
    return GPS_EPOCH + datetime.timedelta(seconds=week * WEEK + tow)


def align(cap: Path, scenario: Path, search_s: float = 600.0):
    meta = json.loads(scenario.read_text())
    truth = Truth(meta)
    fx = fixes_from_capture(cap)
    if len(fx) < 20:
        return None, f"only {len(fx)} valid fixes in {cap.name}"

    first_tow = fx[0][0]
    dur = meta["duration_s"]
    best = None
    # The start is a whole minute at or before the first fix; acquisition puts
    # that fix a few seconds to a couple of minutes in.
    lo = int((first_tow - search_s) // 60) * 60
    hi = int(first_tow // 60) * 60 + 60
    for cand in range(lo, hi + 1, 60):
        errs = []
        for tow, alt, _ in fx:
            t = tow - cand
            if 0.0 <= t <= dur:
                errs.append(abs(alt - truth.at(t)["alt_m"]))
        if len(errs) < max(20, len(fx) // 4):
            continue
        med = statistics.median(errs)
        if best is None or med < best[1]:
            best = (cand, med, len(errs))
    if best is None:
        return None, "no candidate start covered enough of the trajectory"
    return best, None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", type=Path)
    ap.add_argument("-s", "--scenario", required=True, type=Path)
    ap.add_argument("--write", action="store_true",
                    help="record the result in the scenario JSON as start_time")
    args = ap.parse_args()

    best, err = align(args.capture, args.scenario)
    if err:
        print(f"!! {err}")
        return 1
    cand, med, n = best

    nmea_date = date_from_capture(args.capture)
    if nmea_date is not None:
        utc = datetime.datetime.combine(nmea_date, datetime.time()) + \
            datetime.timedelta(seconds=cand)
    else:
        # Any week in 2026 folds to the same seconds-of-day; anchor near the build.
        base = tow_to_utc(0, datetime.datetime(2026, 8, 19))
        utc = datetime.datetime.combine(base.date(), datetime.time()) + \
            datetime.timedelta(seconds=cand)
        utc = utc.replace(year=2026, month=8, day=18)
    stamp = utc.strftime("%Y/%m/%d,%H:%M:%S")
    print(f"{args.capture.name} vs {args.scenario.stem}")
    print(f"  start (s-o-d): {cand}")
    print(f"  start (UTC)  : {stamp}")
    print(f"  aligned on   : {n} fix epochs")
    print(f"  median resid : {med:.0f} m")
    if med > 1000:
        print("  !! residual over 1 km -- this capture probably is not this scenario,\n"
              "     or the alignment is off by more than acquisition can explain")
    else:
        print("  alignment is consistent with the injected trajectory")
    print(f"\n  correlate.py -s {args.scenario} -t {stamp} {args.capture}")

    if args.write:
        meta = json.loads(args.scenario.read_text())
        meta["start_time"] = stamp
        # indent=1 matches make_flights.py / make_trajectories.py, so recording
        # a start_time does not reformat the entire file.
        args.scenario.write_text(json.dumps(meta, indent=1) + "\n")
        print(f"  recorded start_time in {args.scenario}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
