#!/usr/bin/env python3
"""Scale a .C8 up toward full scale to buy transmit power.

gps-sdr-sim leaves a lot of the 8-bit range unused -- measured RMS |IQ| of 49
against a 127 full scale, clipping only 0.005% of samples. That is about 5 dB
of signal thrown away, which matters when the RF path has gone lossy and the
HackRF's TX gain is already at its 47 dB maximum.

Hard clipping is cheap here in a way it would not be for most waveforms: GPS
C/A is spread spectrum, and receivers work from 1-bit sampling at a cost of
roughly 2 dB, so driving a few percent of samples into the rails buys far more
than it loses. Default x2 gives +5.5 dB at ~13% clipping.

Streams in chunks -- the scenario files run to 2.5 GB.
"""
import argparse, sys
from pathlib import Path
import numpy as np

ap = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("files", nargs="+", type=Path)
ap.add_argument("-k", "--scale", type=float, default=2.0)
ap.add_argument("--chunk", type=int, default=1 << 24)
a = ap.parse_args()

for f in a.files:
    if not f.exists():
        print(f"  skip {f.name}: missing"); continue
    n = f.stat().st_size
    clipped = total = 0
    rms_acc = 0.0
    with open(f, "r+b") as fh:
        pos = 0
        while pos < n:
            fh.seek(pos)
            buf = np.frombuffer(fh.read(min(a.chunk, n - pos)), dtype=np.int8)
            if buf.size == 0:
                break
            v = buf.astype(np.float32) * a.scale
            clipped += int(np.count_nonzero(np.abs(v) > 127))
            total += v.size
            rms_acc += float((np.clip(v, -127, 127) ** 2).sum())
            fh.seek(pos)
            fh.write(np.clip(np.rint(v), -127, 127).astype(np.int8).tobytes())
            pos += buf.size
    rms = (rms_acc / total) ** 0.5
    print(f"  {f.name:<26} x{a.scale}  RMS/component {rms:5.1f}  "
          f"clipped {100*clipped/total:5.2f}%")
