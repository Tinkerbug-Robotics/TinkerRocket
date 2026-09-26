#!/usr/bin/env python3
"""Give a scenario a longer pad: the same flight, with ignition moved later.

A receiver cold-started on the pad needs time to decode ephemeris for every
satellite before the flight means anything. On the PX1105R the 360 s pad was not
always enough: the ephemeris decoded by ignition varied from 5 to 9 of 9 satellites
run to run on the same file, and a hot start can only use the ephemeris it holds.
A 600 s pad gave 14 of 14 (3 degree elevation mask) with room to spare.

Reads scenarios/NAME.csv (10 Hz time,lat,lon,height; gps-sdr-sim -x format) and
scenarios/NAME.json (its prologue_s is the ignition time), prepends pad rows so
ignition lands at PAD seconds, and stops at END seconds -- keep the row count
under the gps-sdr-sim build's USER_MOTION_SIZE.

    ./pad_scenario.py spaceshot 600 880       # -> scenarios/spaceshot_pad600.csv

Then generate from c8/ with SHORT relative paths: gps-sdr-sim copies -e/-x/-o into
100-byte buffers (MAX_CHAR) with strcpy, so a long absolute path overflows and the
build traps (SIGTRAP) before printing anything.

    cd c8 && gps-sdr-sim-smooth -e BRDC_2026230.rx2.n -x ../scenarios/spaceshot_pad600.csv \\
        -b 8 -s 2600000 -t 2026/08/18,08:30:00 -p -d 880 -o spaceshot_pad600_smooth.C8

File time maps to the scenario's own clock by PAD - prologue_s (px1105r_run.py
--pad-shift, px_limits.py's SHIFT argument): 420 s for spaceshot_pad600.
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    name, pad = sys.argv[1], float(sys.argv[2])
    end = float(sys.argv[3]) if len(sys.argv) > 3 else None
    src = HERE / "scenarios" / f"{name}.csv"
    prologue = json.loads((HERE / "scenarios" / f"{name}.json").read_text())["prologue_s"]
    extra = pad - prologue
    if extra < 0:
        sys.exit(f"{name} already ignites at {prologue:.0f} s; PAD must be at least that")
    rows = [line.strip() for line in src.read_text().splitlines() if line.strip()]
    where = rows[0].split(",")[1:]                    # the pad position
    out = [f"{k / 10:.1f}," + ",".join(where) for k in range(int(round(extra * 10)))]
    for r in rows:
        t, *rest = r.split(",")
        tn = float(t) + extra
        if end is not None and tn > end + 1e-6:
            break
        out.append(f"{tn:.1f}," + ",".join(rest))
    dst = HERE / "scenarios" / f"{name}_pad{pad:.0f}.csv"
    dst.write_text("\n".join(out) + "\n")
    print(f"{dst.name}: {len(out)} rows, ignition at {pad:.0f} s (pad shift {extra:.0f} s), "
          f"last row {out[-1].split(',')[0]} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
