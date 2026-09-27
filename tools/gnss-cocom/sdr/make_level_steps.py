#!/usr/bin/env python3
"""Write a stepped-level copy of a static .C8, for a level sweep below HackRF gain 0.

The HackRF's lowest transmit gain is 0, and every radiated run already uses it, so
a quieter signal has to be made in the file: each segment's I/Q is scaled by its
level and rounded back to 8 bits. The signal stays continuous -- one file, one GPS
clock -- so the receiver never sees its time step backwards between levels.

    ./make_level_steps.py c8/pad_static.C8 c8/pad_levels.C8 \\
        --hold 150 --step 3 --floor -36 --dwell 45

writes the file plus c8/pad_levels.json, the schedule lc86_tracking.py reads:
0 dB for --hold s, then --step dB every --dwell s down to --floor and back up.
"""
import argparse
import json
from pathlib import Path

import numpy as np

RATE = 2600000
BPS = 2 * RATE            # int8 I and Q per sample


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src"); ap.add_argument("out")
    ap.add_argument("--hold", type=float, default=150.0)
    ap.add_argument("--step", type=float, default=3.0)
    ap.add_argument("--floor", type=float, default=-36.0)
    ap.add_argument("--dwell", type=float, default=45.0)
    ap.add_argument("--start-time", default="2026/08/18,08:30:00")
    a = ap.parse_args()
    n = int(round(-a.floor / a.step))
    steps = [(0.0, a.hold)] + [(-a.step * k, a.dwell) for k in range(1, n + 1)] + \
            [(-a.step * k, a.dwell) for k in range(n - 1, -1, -1)]
    src = np.memmap(a.src, dtype=np.int8, mode="r")
    sched, t = [], 0.0
    for db, dur in steps:
        sched.append({"t0": t, "t1": t + dur, "db": db})
        t += dur
    if t * BPS > len(src):
        raise SystemExit(f"{a.src} holds {len(src) / BPS:.0f} s; the schedule needs {t:.0f} s")
    with open(a.out, "wb") as out:
        for seg in sched:
            lo, hi = int(seg["t0"] * BPS), int(seg["t1"] * BPS)
            g = 10 ** (seg["db"] / 20)
            for c in range(lo, hi, BPS * 5):
                x = src[c:min(c + BPS * 5, hi)]
                if seg["db"] == 0.0:
                    out.write(x.tobytes())
                else:
                    out.write(np.clip(np.rint(x.astype(np.float32) * g), -128, 127)
                              .astype(np.int8).tobytes())
    meta = {"source": Path(a.src).name, "start_time": a.start_time, "rate": RATE,
            "segments": sched}
    Path(a.out).with_suffix(".json").write_text(json.dumps(meta, indent=1) + "\n")
    print(f"{t:.0f} s in {len(sched)} segments -> {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
