#!/usr/bin/env python3
"""Regenerate every figure the report uses, from the archived captures.

Shading thresholds come from results/receivers.json rather than being passed in
by hand, so the bands on a receiver's plot always agree with the row in the
comparison table. Two parts stop well below the export limit for reasons of
their own, and a plot shaded at a constant 515 m/s / 80 km would put the bands
nowhere near where their lock strip actually changes.

    ./replot_all.py            regenerate results/figures/
    ./replot_all.py --check    list what would be drawn, and with what limits
"""

from __future__ import annotations

import argparse
import gzip
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from receiver_table import velocity_threshold   # noqa: E402

HERE = Path(__file__).resolve().parent
RES = HERE / "results"
FIG = RES / "figures"

# receiver id -> (capture stem, scenario stem, start, output figure)
JOBS = [
    ("px1125r",  "spaceshot_eq",         "spaceshot_eq",        "2026/08/18,08:30:00", "spaceshot.svg"),
    ("px1125r",  "gentle_alt_eq2",       "gentle_alt_eq",       "2026/08/18,08:30:00", "gentle_alt.svg"),
    ("sam_m10q", "ublox_m10_spaceshot",  "ublox_m10_spaceshot", "2026/08/18,08:30:00", "ublox_m10_spaceshot.svg"),
    ("sam_m10q", "ublox_m10_gentle_alt", "ublox_m10_gentle_alt","2026/08/18,08:30:00", "ublox_m10_gentle_alt.svg"),
    ("zed_f9p",  "zed_f9p_spaceshot",    "zed_f9p_spaceshot",   "2026/08/18,08:30:00", "zed_f9p_spaceshot.svg"),
    ("zed_f9p",  "zed_f9p_gentle_alt",   "zed_f9p_gentle_alt",  "2026/08/18,08:30:00", "zed_f9p_gentle_alt.svg"),
    ("neo_m8t",  "neo_m8t_spaceshot",    "neo_m8t_spaceshot",   "2026/08/18,08:30:00", "neo_m8t_spaceshot.svg"),
    ("neo_m8t",  "neo_m8t_gentle_alt",   "neo_m8t_gentle_alt",  "2026/08/18,08:30:00", "neo_m8t_gentle_alt.svg"),
    ("air530",   "air530_spaceshot",     "air530_spaceshot",    "2026/08/18,08:30:00", "air530_spaceshot.svg"),
    ("air530",   "air530_gentle_alt",    "air530_gentle_alt",   "2026/08/18,08:30:00", "air530_gentle_alt.svg"),
    ("neo_m8t",  "neo_m8t_t2_altramp",   "neo_m8t_t2_altramp",  "2026/08/19,22:30:00", "neo_m8t_t2_altramp.svg"),
    ("quescan_m10", "quescan_m10_spaceshot",  "quescan_m10_spaceshot",  "2026/08/18,08:30:00", "quescan_m10_spaceshot.svg"),
    ("quescan_m10", "quescan_m10_gentle_alt", "quescan_m10_gentle_alt", "2026/08/18,08:30:00", "quescan_m10_gentle_alt.svg"),
    ("beitian_bn182", "beitian_bn182_spaceshot",  "beitian_bn182_spaceshot",  "2026/08/18,08:30:00", "beitian_bn182_spaceshot.svg"),
    ("beitian_bn182", "beitian_bn182_gentle_alt", "beitian_bn182_gentle_alt", "2026/08/18,08:30:00", "beitian_bn182_gentle_alt.svg"),
]


def limits(rec):
    """(velocity, altitude) thresholds to shade, rounded as the table shows them.

    The velocity estimate comes from receiver_table.velocity_threshold rather
    than being recomputed here: a second copy of the rule drifted the moment the
    table switched to a robust edge median, and a figure shaded at 510 beside a
    table row saying 515 is worse than either alone.

    A part with no velocity gate shades no speed band -- the Air530 held a fix
    to 900 m/s, so a band at 515 would mark something that never happened.
    """
    vel = velocity_threshold(rec)
    lo, hi = rec.get("altitude_fix_max_km"), rec.get("altitude_blocked_min_km")
    alt = None if lo is None or hi is None else round((lo + hi) / 2)
    return vel, alt


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    recs = {r["id"]: r for r in
            json.loads((RES / "receivers.json").read_text())["receivers"]}
    tmp = Path(tempfile.mkdtemp())
    rc = 0
    try:
        for rid, cap, scen, start, out in JOBS:
            vel, alt = limits(recs[rid])
            vs = "none" if vel is None else f"{vel:g}"
            as_ = "none" if alt is None else f"{alt:g}"
            if args.check:
                print(f"  {out:<30} {rid:<10} shade: {vs:>5} m/s, {as_:>5} km")
                continue
            gz = RES / f"{cap}.log.gz"
            if not gz.exists():
                print(f"  !! missing capture {gz.name}")
                rc = 1
                continue
            plain = tmp / f"{cap}.log"
            plain.write_text(gzip.open(gz, "rt", errors="replace").read())
            r = subprocess.run(
                [sys.executable, str(HERE / "plot_flight.py"),
                 "-s", str(RES / f"{scen}.scenario.json"), "-t", start,
                 "--vel-limit", vs, "--alt-limit", as_,
                 str(plain), "-o", str(FIG / out)],
                capture_output=True, text=True)
            tail = (r.stdout + r.stderr).strip().splitlines()
            print(f"  {tail[-1] if tail else out}   [{vs} m/s, {as_} km]")
            rc |= r.returncode
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
