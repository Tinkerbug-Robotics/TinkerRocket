#!/usr/bin/env python3
"""Carrier-only frequency shift of an IQ8 file (interleaved int8 I, Q): every signal in it moves by SHIFT_HZ while
the sample sequence -- and with it every code's timing -- is untouched. The SignalSim counterpart of gps-sdr-sim's
patch_carrier_offset.py (-DCARR_OFFSET_HZ=-22.0), which cancels the HackRF's +22 Hz carrier-vs-code error at
1575.42 MHz (results/README, "The HackRF's carrier runs 22 Hz off its own code").

Each 1 s block is multiplied by exp(j 2 pi SHIFT_HZ n / fs) with n the file's sample index (phase continuous),
rounded back to int8 (clipped values counted). Writes OUT.partial and renames it when complete.

    carrier_shift.py IN.C8 OUT.C8 FS_HZ SHIFT_HZ
"""
import os
import sys
import time

import numpy as np

src, dst, fs, shift = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
total = os.path.getsize(src) // 2
block = int(round(fs))
step = shift / fs                                    # cycles per sample
clipped, n0, sum_in, sum_out = 0, 0, 0.0, 0.0
t_start = time.time()
tmp = dst + ".partial"
with open(src, "rb") as fi, open(tmp, "wb") as fo:
    while True:
        raw = np.fromfile(fi, dtype=np.int8, count=2 * block)
        if raw.size == 0:
            break
        m = raw.size // 2
        ph = 2.0 * np.pi * (((n0 * step) % 1.0) + np.arange(m) * step)
        c, s = np.cos(ph).astype(np.float32), np.sin(ph).astype(np.float32)
        i = raw[0::2].astype(np.float32)
        q = raw[1::2].astype(np.float32)
        out = np.empty(2 * m, np.float32)
        out[0::2] = i * c - q * s
        out[1::2] = i * s + q * c
        np.rint(out, out=out)
        clipped += int(np.count_nonzero((out > 127) | (out < -128)))
        np.clip(out, -128, 127, out=out)
        out.astype(np.int8).tofile(fo)
        if n0 == 0:
            sum_in = float(np.sqrt(np.mean(raw.astype(np.float32) ** 2)))
            sum_out = float(np.sqrt(np.mean(out ** 2)))
        n0 += m
        sec = n0 // block
        if sec % 120 == 0 or n0 >= total:
            print(f"{sec:5d} / {total // block} s  ({time.time() - t_start:5.0f} s elapsed)  clipped {clipped}",
                  flush=True)
os.replace(tmp, dst)
print(f"done: {n0} samples, shift {shift:+.3f} Hz, first-second RMS {sum_in:.2f} -> {sum_out:.2f} LSB, "
      f"clipped values {clipped} ({clipped / (2 * n0) * 100:.2e} %), {time.time() - t_start:.0f} s")
