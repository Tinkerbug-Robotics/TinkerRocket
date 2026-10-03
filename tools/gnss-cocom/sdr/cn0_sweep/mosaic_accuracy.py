#!/usr/bin/env python3
"""Pseudorange and range-rate accuracy of a mosaic-G5 SignalSim capture (mosaic_run.py lines 't S <hex>'), by
m8t_accuracy.py's method and with its tables: the same pooled ephemerides from the PX1105R captures of these
scenarios, the same truth, the same clock solution.

One signal per satellite, the one the wide files carry: GPS L1 C/A, Galileo E1, BeiDou B1I. MeasEpoch comes at
20 Hz; every second epoch is used (10 Hz, as the NEO-M8T's and ZED-F9P's RXM-RAWX). Only measurements with a
pseudorange are used; MeasEpoch carries none while the receiver withholds its output above the export limit.

Transmitter underruns step the signal's clock by the gap (1-3 ms, sometimes 200 ms). The mosaic re-acquires and
steps its own clock, but a channel that misses the step stays that far off -- hundreds of kilometres, a whole multiple
of the gap times c (790.9 km for a 2.638 ms gap). No tracking error comes near that, so pseudorange errors over
STEP_KM (100 km) are taken as the rig's and set to NaN (unknown) in the NPZ, and listed per satellite.

    mosaic_accuracy.py CAPTURE OUT.npz [--scenario traveler|hotshot|gentle_alt|spaceshot] [--rr-lag 0.0] [--retime]
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))
import m8t_accuracy as M                                                 # noqa: E402
import septentrio_sbf as sbf                                             # noqa: E402

SIGNALS = {"GPS L1CA": "G", "GAL E1": "E", "BDS B1I": "C"}
STEP_KM = 100.0                                                          # beyond this an error is a clock step


def mosaic_epochs(cap):
    """MeasEpoch -> {k: (tow_s, [(sys, prn, pr, doppler, cn0, None, 0)])} at 10 Hz, in log order."""
    ep = {}
    for line in open(cap, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "S" or len(p[2]) < 24:
            continue
        h = p[2]
        if (int(h[10:12] + h[8:10], 16) & 0x1FFF) != sbf.MEAS_EPOCH:
            continue
        tow = int.from_bytes(bytes.fromhex(h[16:24]), "little")
        if tow == 0xFFFFFFFF or tow % 100:
            continue
        try:
            m = sbf.meas_epoch(bytes.fromhex(h.strip()))
        except ValueError:
            continue
        obs = [(SIGNALS[x["signal"]], int(x["sv"][1:]), x["pr"], x["doppler"], x["cn0"], None, 0)
               for x in m["meas"] if x["signal"] in SIGNALS and x["pr"] is not None and x["doppler"] is not None
               and x["sv"][1:].isdigit()]
        if obs:
            ep[len(ep)] = (tow / 1000, obs)
    return ep


def drop_steps(path):
    """Errors over STEP_KM -> NaN in the NPZ, with a count per satellite."""
    import numpy as np
    z = dict(np.load(path))
    bad = np.isfinite(z["pr"]) & (np.abs(z["pr"]) > STEP_KM * 1000.0)
    if bad.any():
        per = {}
        for i in np.where(bad)[0]:
            k = f"{'GEC'[int(z['sys'][i])]}{int(z['prn'][i]):02d}"
            per[k] = per.get(k, 0) + 1
        z["pr"] = np.where(bad, np.nan, z["pr"])
        np.savez(path, **z)
        print(f"clock-step rows (|error| > {STEP_KM:.0f} km) set to NaN: {int(bad.sum())} -- "
              + ", ".join(f"{k} {n}" for k, n in sorted(per.items())))
    else:
        print(f"no clock-step rows (|error| > {STEP_KM:.0f} km)")


if __name__ == "__main__":
    M.m8t_epochs = mosaic_epochs
    if "--rr-lag" not in sys.argv:
        sys.argv += ["--rr-lag", "0.0"]
    out = sys.argv[2]
    M.main()
    drop_steps(out)
