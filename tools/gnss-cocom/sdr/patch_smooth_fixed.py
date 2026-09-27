#!/usr/bin/env python3
"""Add an opt-in SMOOTH_FIXED build to gps-sdr-sim: SMOOTH_CARRIER's frequency ramp on the
stock fixed-point carrier, without FLOAT_CARR_PHASE.

SMOOTH_CARRIER (patch_smooth_carrier.py) has to use gps-sdr-sim's floating-point carrier
path, and on a static pad that path -- not the ramp -- is what separates its file from
stock: stock and float-only IQ are only 10 % bit-identical (correlation 0.997), float-only
and smooth 68 % (0.9995). On the PX1125R (2022 firmware) the carrier builds acquired very
differently from a cold start in power save (2026-09-26, same pad and settings): stock fixed
by ~136 s, float-only ~326 s, SMOOTH_FIXED ~290 s, SMOOTH_CARRIER never in five runs. With
power save off the SMOOTH_CARRIER file fixes too (~53 s), so this build is an alternative,
not a requirement; the PX1105R and LC86G were fine on SMOOTH_CARRIER throughout.

SMOOTH_FIXED keeps SMOOTH_CARRIER's per-block computation (the ramp from the mean of the
previous and current block rates to the mean of the current and next) and turns it into the
fixed-point phase step, 512*65536 units per cycle, rounded every sample -- as stock rounds
its step once per block.

    python3 patch_smooth_carrier.py gpssim.c      # first
    python3 patch_smooth_fixed.py gpssim.c        # then this; in place
    gcc -O3 -DUSER_MOTION_SIZE=10000 -DSMOOTH_FIXED -o gps-sdr-sim-sfixed gpssim.c -lm

Built without -DSMOOTH_FIXED (and without -DSMOOTH_CARRIER) the source still writes a file
byte-identical to stock. Build with one of SMOOTH_FIXED or SMOOTH_CARRIER, not both.
"""
import sys
from pathlib import Path

p = Path(sys.argv[1])
src = p.read_text()
if "SMOOTH_FIXED" in src:
    raise SystemExit("already patched")
if "SMOOTH_CARRIER" not in src:
    raise SystemExit("apply patch_smooth_carrier.py first")

edits = [
    # state for the fixed-point ramp (SMOOTH_FIXED is built without SMOOTH_CARRIER)
    ("double xyz[USER_MOTION_SIZE][3];\n",
     "double xyz[USER_MOTION_SIZE][3];\n"
     "\n#ifdef SMOOTH_FIXED\n"
     "/* SMOOTH_CARRIER's frequency ramp on the stock fixed-point carrier: the per-sample\n"
     "   phase step (512*65536 units per cycle) ramps across each block and is rounded\n"
     "   every sample, as stock rounds it once per block. No FLOAT_CARR_PHASE. */\n"
     "static double sm_fprev[MAX_CHAN], sm_f[MAX_CHAN], sm_df[MAX_CHAN], sm_step[MAX_CHAN], sm_dstep[MAX_CHAN];\n"
     "static int sm_prn[MAX_CHAN];\n"
     "#endif\n"),
    # the per-block ramp computation runs for either flavour
    ("#ifdef SMOOTH_CARRIER\n\t\t\t\t{\n\t\t\t\t\t/* This block's average frequency",
     "#if defined(SMOOTH_CARRIER) || defined(SMOOTH_FIXED)\n\t\t\t\t{\n\t\t\t\t\t/* This block's average frequency"),
    ("\t\t\t\t\tsm_df[i] = (0.5 * (fk + fn) - sm_f[i]) / (double)iq_buff_size;\n",
     "\t\t\t\t\tsm_df[i] = (0.5 * (fk + fn) - sm_f[i]) / (double)iq_buff_size;\n"
     "#ifdef SMOOTH_FIXED\n"
     "\t\t\t\t\tsm_step[i] = 512.0 * 65536.0 * sm_f[i] * delt;\n"
     "\t\t\t\t\tsm_dstep[i] = 512.0 * 65536.0 * sm_df[i] * delt;\n"
     "#endif\n"),
    # per-sample carrier update on the fixed-point path
    ("\t\t\t\t\tchan[i].carr_phase += chan[i].carr_phasestep;\n",
     "#ifdef SMOOTH_FIXED\n"
     "\t\t\t\t\tchan[i].carr_phase += (int)lround(sm_step[i]);\n"
     "\t\t\t\t\tsm_step[i] += sm_dstep[i];\n"
     "#else\n"
     "\t\t\t\t\tchan[i].carr_phase += chan[i].carr_phasestep;\n"
     "#endif\n"),
]
for old, new in edits:
    if src.count(old) != 1:
        raise SystemExit(f"anchor not found exactly once:\n{old}")
    src = src.replace(old, new)
p.write_text(src)
print(f"{p}: SMOOTH_FIXED added (opt-in; the stock build is unchanged)")
