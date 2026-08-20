#!/usr/bin/env python3
"""Patch gps-sdr-sim to use the real horizon at altitude, not a 0 degree mask.

gps-sdr-sim decides a satellite is visible when its elevation above the *local
horizontal plane* is positive (`elvmask = 0.0`, checkSatVisibility). That is
right on the ground and wrong in flight: from height h the horizon is depressed
by acos(R/(R+h)), so a vehicle at 82.5 km genuinely sees every satellite down to
9.2 degrees *below* horizontal. Holding the mask at zero withholds that whole
band, which is exactly the band a space shot gains by being up there.

    altitude      horizon dip
       1.2 km        1.11 deg
        18 km        4.30 deg
        40 km        6.40 deg
      82.5 km        9.17 deg

A fixed negative mask would be worse than the bug: at low altitude it would
transmit satellites that are behind the Earth. The mask has to follow the
vehicle, so this computes it per allocateChannel call from the position that
call is already given.

Idempotent. Run against a gps-sdr-sim checkout, then rebuild:

    python3 patch_horizon.py ~/src/gps-sdr-sim/gpssim.c
    gcc -O3 -DUSER_MOTION_SIZE=10000 -o gps-sdr-sim gpssim.c -lm
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

HELPER = """
/* --- gnss-cocom patch: real horizon at altitude ------------------------------
 * From height h the horizon sits acos(R/(R+h)) below the local horizontal, so a
 * vehicle at 82.5 km sees satellites down to -9.2 deg. A fixed 0 deg mask hides
 * them; a fixed negative one would transmit through the Earth at low altitude.
 * Compute it from the position each allocateChannel call already receives.
 */
double horizonDip(double *xyz)
{
\tdouble llh[3];
\tdouble h;

\txyz2llh(xyz, llh);
\th = llh[2];
\tif (h < 0.0)
\t\th = 0.0;

\treturn (-acos(6371000.0 / (6371000.0 + h)) * R2D);
}

"""

ANCHOR = "int allocateChannel("
MARK = "double horizonDip(double *xyz)"

# allocateChannel takes an elvMask parameter and then ignores it, calling
# checkSatVisibility with a hardcoded 0.0. Upstream bug: the parameter is dead.
# Passing the right value in is useless until this is fixed too.
DEAD_PARAM = "if(checkSatVisibility(eph[sv], grx, xyz, 0.0, azel)==1)"
LIVE_PARAM = "if(checkSatVisibility(eph[sv], grx, xyz, elvMask, azel)==1)"


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    src = Path(sys.argv[1])
    text = src.read_text()

    if MARK in text:
        print(f"{src.name}: already patched")
        return 0
    if ANCHOR not in text:
        raise SystemExit(f"{src}: cannot find allocateChannel; is this gpssim.c?")

    shutil.copy2(src, src.with_suffix(".c.orig"))

    text = text.replace(ANCHOR, HELPER.lstrip("\n") + ANCHOR, 1)

    # Both dynamic-mode call sites take the current position; the static-mode one
    # takes xyz[0], which is the fixed location and equally entitled to its dip.
    before = text
    text = text.replace("ionoutc, grx, xyz[iumd], elvmask)",
                        "ionoutc, grx, xyz[iumd], horizonDip(xyz[iumd]))")
    text = text.replace("ionoutc, grx, xyz[0], elvmask)",
                        "ionoutc, grx, xyz[0], horizonDip(xyz[0]))")
    n = sum(1 for a, b in zip(before.split("elvmask"), text.split("elvmask"))
            if a != b)
    if "elvmask))" in text or before == text:
        raise SystemExit("no allocateChannel call sites were rewritten")

    if DEAD_PARAM in text:
        text = text.replace(DEAD_PARAM, LIVE_PARAM, 1)
        print("  allocateChannel now honours its elvMask parameter "
              "(upstream hardcodes 0.0 and ignores it)")
    elif LIVE_PARAM not in text:
        raise SystemExit("cannot find the checkSatVisibility call in "
                         "allocateChannel; upstream may have changed")

    src.write_text(text)
    remaining = text.count("elvmask")
    print(f"{src.name}: patched  (original saved as {src.with_suffix('.c.orig').name})")
    print(f"  allocateChannel now masks on the real horizon at the vehicle's altitude")
    print(f"  {remaining} reference(s) to elvmask left (its declaration)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
