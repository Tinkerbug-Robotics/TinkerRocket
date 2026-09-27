#!/usr/bin/env python3
"""Add an opt-in CARR_OFFSET_HZ build to gps-sdr-sim: a constant offset on the
carrier alone, to cancel the HackRF's.

On this rig the transmitted carrier sits 22.0 Hz above where its own code says it
should be: code-minus-carrier grows at +4.19 m/s on the PX1105R, PX1125R, NEO-M8T
and LC86G alike, on every IQ build (stock, float, fixed-point ramp, smooth), from
2026-08-20 to 2026-09-27, and at 0.00 m/s on the real sky. gps-sdr-sim is not the
source: a static file it writes is code/carrier-consistent to 0.08 m/s. The HackRF
synthesizes its 1575.42 MHz LO fractionally while the 2.6 MHz sample clock comes out
exact, so the carrier alone carries the tuning error. Receivers read it as a clock
whose code and carrier disagree. See results/README.md.

With -DCARR_OFFSET_HZ=-22.0 every channel's carrier is generated 22.0 Hz low. The
code phase, code rate and data bits are untouched, so on air the HackRF's +22.0 Hz
brings the carrier back onto the code. Works with every build flavour: the stock
fixed-point carrier, FLOAT_CARR_PHASE, SMOOTH_CARRIER and SMOOTH_FIXED.

    python3 patch_horizon.py gpssim.c
    python3 patch_smooth_carrier.py gpssim.c       # (and patch_smooth_fixed.py) as needed
    python3 patch_carrier_offset.py gpssim.c       # last; in place
    gcc -O3 -DUSER_MOTION_SIZE=10000 -DFLOAT_CARR_PHASE -DSMOOTH_CARRIER \\
        -DCARR_OFFSET_HZ=-22.0 -o gps-sdr-sim-smooth-cofs gpssim.c -lm

Built without -DCARR_OFFSET_HZ the source writes the same file byte for byte as
before. To check a build on air, run any receiver that reports pseudorange and
carrier phase on a static signal: code-minus-carrier, per satellite, should grow at
about 0 m/s instead of +4.19 m/s.
"""
import sys
from pathlib import Path

p = Path(sys.argv[1])
src = p.read_text()
if "CARR_OFFSET_HZ" in src:
    raise SystemExit("already patched")

macro = ("double xyz[USER_MOTION_SIZE][3];\n",
         "double xyz[USER_MOTION_SIZE][3];\n"
         "\n/* Carrier frequency as generated: the channel's, plus CARR_OFFSET_HZ when it is\n"
         "   defined (patch_carrier_offset.py). The code is never offset. */\n"
         "#ifdef CARR_OFFSET_HZ\n"
         "#define CARR_F(f) ((f) + (CARR_OFFSET_HZ))\n"
         "#else\n"
         "#define CARR_F(f) (f)\n"
         "#endif\n")

# Every place the carrier phase is advanced, by build flavour. The first two exist in
# every source; the other two only after patch_smooth_carrier.py / patch_smooth_fixed.py.
sites = [
    ("chan[i].carr_phasestep = (int)round(512.0 * 65536.0 * chan[i].f_carr * delt);",
     "chan[i].carr_phasestep = (int)round(512.0 * 65536.0 * CARR_F(chan[i].f_carr) * delt);", True),
    ("chan[i].carr_phase += chan[i].f_carr * delt;",
     "chan[i].carr_phase += CARR_F(chan[i].f_carr) * delt;", True),
    ("chan[i].carr_phase += sm_f[i] * delt;",
     "chan[i].carr_phase += CARR_F(sm_f[i]) * delt;", False),
    ("sm_step[i] = 512.0 * 65536.0 * sm_f[i] * delt;",
     "sm_step[i] = 512.0 * 65536.0 * CARR_F(sm_f[i]) * delt;", False),
]

if src.count(macro[0]) != 1:
    raise SystemExit(f"anchor not found exactly once:\n{macro[0]}")
src = src.replace(macro[0], macro[1])
done = []
for old, new, required in sites:
    n = src.count(old)
    if n == 0 and not required:
        continue
    if n != 1:
        raise SystemExit(f"carrier site found {n} times (want 1):\n{old}")
    src = src.replace(old, new)
    done.append(old.split("=")[0].strip())
p.write_text(src)
print(f"{p}: CARR_OFFSET_HZ added (opt-in) at {len(done)} carrier sites: " + "; ".join(done))
