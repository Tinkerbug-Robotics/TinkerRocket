#!/usr/bin/env python3
"""Check hackrf_tx_ram -N output (a -X dump) against the file it came from: fit y = a x + r, report a, the RMS of
the residual r (the added noise), the implied C/N0 drop 10 log10(1 + r^2 / (a^2 x^2)), the total RMS (should match
the file), and that r is white and unrelated between I and Q.      noise_check.py ORIG.C8 DUMP.bin EXPECT_DB
"""
import sys

import numpy as np

orig, dump, expect = sys.argv[1], sys.argv[2], float(sys.argv[3])
y = np.fromfile(dump, dtype=np.int8).astype(np.float64)
x = np.fromfile(orig, dtype=np.int8, count=y.size).astype(np.float64)
a = float(np.dot(x, y) / np.dot(x, x))
r = y - a * x
sx, sy, sr = (float(np.sqrt(np.mean(v ** 2))) for v in (x, y, r))
drop = 10 * np.log10(1 + sr ** 2 / (a * sx) ** 2)
print(f"{y.size / 2 / 1e6:.1f} M complex samples: a = {a:.4f} (expect {10 ** (-expect / 20):.4f}), "
      f"added noise RMS {sr:.2f} LSB per rail, implied C/N0 drop {drop:.2f} dB (asked {expect:g})")
print(f"total RMS per rail: file {sx:.2f}, output {sy:.2f} LSB; clipped values in output "
      f"{int(np.count_nonzero((y >= 127) | (y <= -128)))}")
ri, rq = r[0::2], r[1::2]
print(f"I/Q residual correlation {np.corrcoef(ri, rq)[0, 1]:+.4f}; residual mean {r.mean():+.3f}")
z = ri + 1j * rq
seg = 4096
k = z.size // seg
p = np.mean(np.abs(np.fft.fft(z[: k * seg].reshape(k, seg), axis=1)) ** 2, axis=0)
p = np.fft.fftshift(p)
bands = np.array_split(p, 8)
lv = [10 * np.log10(b.mean() / p.mean()) for b in bands]
print("residual spectrum, 8 bands across the file, dB re mean: " + " ".join(f"{v:+.2f}" for v in lv))
