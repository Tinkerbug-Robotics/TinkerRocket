#!/usr/bin/env python3
"""Add an opt-in SMOOTH_CARRIER build to gps-sdr-sim.

gps-sdr-sim refreshes every channel once per 0.1 s block and holds the carrier
frequency constant across the block at the block's average range rate. Under a
13 g burn that is a staircase: each satellite's carrier jumps by (Doppler rate x
0.1 s) every 0.1 s -- 5 Hz for a satellite the burn barely moves, 50 Hz for one
along the thrust line -- where a real flight slides smoothly. With SMOOTH_CARRIER
the frequency is swept linearly across each block, from the rate at the block's
start to the rate at its end (each the mean of the two neighbouring blocks'
average rates, which is exact under constant acceleration), phase-continuous.
Code phase and data bits are untouched. Needs FLOAT_CARR_PHASE.

    python3 patch_smooth_carrier.py gpssim.c   # after patch_horizon.py; in place
    gcc -O3 -DUSER_MOTION_SIZE=10000 -DFLOAT_CARR_PHASE -DSMOOTH_CARRIER \\
        -o gps-sdr-sim-smooth gpssim.c -lm

Without -DSMOOTH_CARRIER the source compiles to the stock program.

The smooth spaceshot (2026-09-26), run from c8/ so the path arguments stay short:

    gps-sdr-sim-smooth -e BRDC_2026230.rx2.n -x ../scenarios/spaceshot.csv -b 8 \
        -s 2600000 -t 2026/08/18,08:30:00 -p -d 340 -o spaceshot_smooth.C8

Checked two ways: the patched source built without the flag writes a file
byte-identical to stock, and one satellite rendered alone (every other channel's
gain zeroed) and squared shows the stock build's 54 Hz jumps every 0.1 s on G24
where the smooth build ramps at 540 Hz/s. The ramp starts half a block early:
each block edge takes the mean of its two neighbours' rates, so ignition's kink is
spread over +-0.1 s, about a real motor's thrust rise.
"""
import sys
from pathlib import Path

p = Path(sys.argv[1])
src = p.read_text()
if "SMOOTH_CARRIER" in src:
    raise SystemExit("already patched")

edits = [
    ("double xyz[USER_MOTION_SIZE][3];\n",
     "double xyz[USER_MOTION_SIZE][3];\n"
     "\n#ifdef SMOOTH_CARRIER\n"
     "#ifndef FLOAT_CARR_PHASE\n#error \"SMOOTH_CARRIER needs FLOAT_CARR_PHASE\"\n#endif\n"
     "/* Per-channel sweep state: the previous block's average carrier frequency, the\n"
     "   current instantaneous frequency and its per-sample increment. */\n"
     "static double sm_fprev[MAX_CHAN], sm_f[MAX_CHAN], sm_df[MAX_CHAN];\n"
     "static int sm_prn[MAX_CHAN];\n"
     "#endif\n"),
    ("\t\t\t\tcomputeCodePhase(&chan[i], rho, 0.1);\n"
     "#ifndef FLOAT_CARR_PHASE\n"
     "\t\t\t\tchan[i].carr_phasestep = (int)round(512.0 * 65536.0 * chan[i].f_carr * delt);\n"
     "#endif\n",
     "\t\t\t\tcomputeCodePhase(&chan[i], rho, 0.1);\n"
     "#ifndef FLOAT_CARR_PHASE\n"
     "\t\t\t\tchan[i].carr_phasestep = (int)round(512.0 * 65536.0 * chan[i].f_carr * delt);\n"
     "#endif\n"
     "#ifdef SMOOTH_CARRIER\n"
     "\t\t\t\t{\n"
     "\t\t\t\t\t/* This block's average frequency, the next block's (one block of\n"
     "\t\t\t\t\t   look-ahead into the motion file) and the previous block's. */\n"
     "\t\t\t\t\tdouble fk = chan[i].f_carr, fn = fk, fp;\n"
     "\t\t\t\t\tif (iumd + 1 < numd)\n"
     "\t\t\t\t\t{\n"
     "\t\t\t\t\t\trange_t rn;\n"
     "\t\t\t\t\t\tgpstime_t gn = incGpsTime(grx, 0.1);\n"
     "\t\t\t\t\t\tcomputeRange(&rn, eph[ieph][sv], &ionoutc, gn,\n"
     "\t\t\t\t\t\t\tstaticLocationMode ? xyz[0] : xyz[iumd + 1]);\n"
     "\t\t\t\t\t\tfn = -((rn.range - rho.range) / 0.1) / LAMBDA_L1;\n"
     "\t\t\t\t\t}\n"
     "\t\t\t\t\tfp = (sm_prn[i] == chan[i].prn) ? sm_fprev[i] : fk;\n"
     "\t\t\t\t\tsm_prn[i] = chan[i].prn;\n"
     "\t\t\t\t\tsm_fprev[i] = fk;\n"
     "\t\t\t\t\tsm_f[i] = 0.5 * (fp + fk);\n"
     "\t\t\t\t\tsm_df[i] = (0.5 * (fk + fn) - sm_f[i]) / (double)iq_buff_size;\n"
     "#ifdef SMOOTH_DEBUG\n"
     "\t\t\t\t\tif ((chan[i].prn == 19 || chan[i].prn == 24) && iumd >= 1797 && iumd <= 1806)\n"
     "\t\t\t\t\t\tfprintf(stderr, \"SMOOTH prn %2d block %5d  avg %10.3f  start %10.3f  end %10.3f\\n\",\n"
     "\t\t\t\t\t\t\tchan[i].prn, iumd, fk, sm_f[i], 0.5 * (fk + fn));\n"
     "#endif\n"
     "\t\t\t\t}\n"
     "#endif\n"),
    ("#ifdef FLOAT_CARR_PHASE\n"
     "\t\t\t\t\tchan[i].carr_phase += chan[i].f_carr * delt;\n",
     "#ifdef FLOAT_CARR_PHASE\n"
     "#ifdef SMOOTH_CARRIER\n"
     "\t\t\t\t\tchan[i].carr_phase += sm_f[i] * delt;\n"
     "\t\t\t\t\tsm_f[i] += sm_df[i];\n"
     "#else\n"
     "\t\t\t\t\tchan[i].carr_phase += chan[i].f_carr * delt;\n"
     "#endif\n"),
]
for old, new in edits:
    if src.count(old) != 1:
        raise SystemExit(f"anchor not found exactly once:\n{old}")
    src = src.replace(old, new)
p.write_text(src)
print(f"{p}: SMOOTH_CARRIER added (opt-in; the stock build is unchanged)")
