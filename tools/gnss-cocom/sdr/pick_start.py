#!/usr/bin/env python3
"""Pick a scenario start time inside a RINEX 2 nav file's densest window.

Broadcast ephemeris files from the mirrors that serve without an account are
often *partial days*: the merge only holds what had been collected when it was
generated, so the early hours carry a handful of satellites and the late hours
carry the full constellation. Starting a scenario at a default noon can hand
gps-sdr-sim eight satellites when the same file offers thirty-one at 22:00.

That failure is silent -- the run completes, the receiver just never fixes, and
the obvious suspicion falls on RF levels rather than on the ephemeris.

Prints one `YYYY/MM/DD,hh:mm:ss` on stdout.
"""

from __future__ import annotations

import argparse
import datetime as dt
import sys
from collections import defaultdict
from pathlib import Path


def epochs(path: Path) -> dict:
    """Map each epoch to the set of PRNs with an ephemeris at that epoch."""
    lines = path.read_text(errors="replace").splitlines()
    try:
        start = next(i for i, l in enumerate(lines)
                     if l[60:73].strip() == "END OF HEADER") + 1
    except StopIteration:
        raise SystemExit(f"{path}: no END OF HEADER")

    found = defaultdict(set)
    i = start
    while i + 7 < len(lines):
        l = lines[i]
        # A RINEX 2 nav epoch line begins with a right-aligned PRN in cols 0-1.
        if l[:2].strip().isdigit():
            try:
                prn = int(l[:2])
                yy, mo, dd = int(l[3:5]), int(l[6:8]), int(l[9:11])
                hh, mi = int(l[12:14]), int(l[15:17])
                found[dt.datetime(2000 + yy, mo, dd, hh, mi)].add(prn)
            except ValueError:
                pass
            i += 8
        else:
            i += 1
    return found


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("nav", type=Path, help="RINEX 2 GPS navigation file")
    ap.add_argument("--min-duration", type=float, default=240.0,
                    help="seconds the scenario must fit before the last epoch")
    ap.add_argument("--report", action="store_true",
                    help="print the per-epoch satellite census to stderr")
    args = ap.parse_args()

    found = epochs(args.nav)
    if not found:
        raise SystemExit(f"{args.nav}: no navigation records found")

    if args.report:
        for when in sorted(found):
            print(f"  {when:%Y-%m-%d %H:%M}  {len(found[when]):2d} SV",
                  file=sys.stderr)

    last = max(found)
    # gps-sdr-sim picks the ephemeris set whose TOC is within an hour of the
    # start, so aim half an hour past a dense epoch: comfortably inside that
    # window, and clear of the boundary where the next set takes over.
    best, best_n = None, -1
    for when, prns in sorted(found.items()):
        target = when + dt.timedelta(minutes=30)
        if (last - target).total_seconds() < args.min_duration:
            continue
        if len(prns) > best_n:
            best, best_n = target, len(prns)

    if best is None:
        raise SystemExit(
            f"{args.nav}: no epoch leaves room for a {args.min_duration:.0f} s "
            f"run before the file ends at {last:%Y-%m-%d %H:%M}")

    print(f"{best:%Y/%m/%d,%H:%M:%S}")
    print(f"picked {best:%Y-%m-%d %H:%M} UTC -- {best_n} satellites in that "
          f"ephemeris set", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
