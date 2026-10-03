#!/usr/bin/env python3
"""Where the mosaic-G5 withholds its fix and its raw measurements in a SignalSim flight, against the truth.

Reads a mosaic_run.py capture (20 Hz PVTGeodetic + MeasEpoch) and the scenario JSON (10 Hz truth). The receiver's
time is tied to the truth by its own climb rate: the offset that best matches PVTGeodetic's Vu to the truth's
v_up_mps while the fix is still allowed (Vu 20-450 m/s). Then, for every stretch where PVTGeodetic says
"position output prohibited due to export laws" (error 7), the last fix before and the first fix after, with the
truth speed and altitude there, and when MeasEpoch stops and resumes carrying measurements.

    mosaic_gate.py CAPTURE [--scenario hotshot|traveler_soft25]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import septentrio_sbf as sbf                            # noqa: E402


DESIGN_TIE = 203820.0                                  # TOW at the start of every p180 SignalSim file
FILE_SIGNALS = {"GPS L1CA": "GPS", "GAL E1": "Galileo", "BDS B1I": "BeiDou"}   # what the wide files carry


def read(cap):
    pvt, meas, cn0 = [], [], []
    for line in open(cap, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "S" or len(p[2]) < 20:
            continue
        bid = int(p[2][10:12] + p[2][8:10], 16) & 0x1FFF
        if bid not in (sbf.PVT_GEODETIC, sbf.MEAS_EPOCH):
            continue
        try:
            b = bytes.fromhex(p[2].strip())
        except ValueError:
            continue
        tow, _ = sbf.tow_wnc(b)
        if tow is None:
            continue
        if bid == sbf.PVT_GEODETIC:
            pvt.append((tow / 1000, sbf.pvt_geodetic(b)))
        else:
            m = sbf.meas_epoch(b)
            meas.append((tow / 1000, len({x["sv"] for x in m["meas"] if x["pr"] is not None})))
            if tow % 1000 == 0:
                cn0 += [(tow / 1000, FILE_SIGNALS[x["signal"]], x["cn0"]) for x in m["meas"]
                        if x["signal"] in FILE_SIGNALS and x["cn0"] is not None]
    return pvt, meas, cn0


def underruns(cap):
    """T of each transmitter underrun, from the -B per-second lines (ignition ~178.2 s into a p180 file)."""
    import re
    path = Path(str(cap) + ".hackrf.txt")
    if not path.exists():
        return None
    prev, out = 0, []
    lines = [x for x in path.read_text(errors="replace").splitlines() if "MB / " in x]
    for k, x in enumerate(lines[:-1], 1):
        m = re.search(r"(\d+) underruns", x)
        n = int(m.group(1)) if m else prev
        if n > prev:
            out.append(round(k - 178.2, 1))
        prev = n
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("--scenario", default=None, help="default: from the capture name")
    a = ap.parse_args()
    scen = a.scenario or ("hotshot" if "hotshot" in a.capture else "traveler_soft25")
    sc = json.loads((HERE / "scenarios" / f"{scen}.json").read_text())
    tr = sc["truth"]
    tt = np.array([r["t"] for r in tr])
    tv = np.array([r["v_up_mps"] for r in tr])
    ts = np.array([r["speed_mps"] for r in tr])
    th = np.array([r["alt_m"] for r in tr])
    pro = sc["prologue_s"]

    pvt, meas, cn0 = read(a.capture)
    fixes = [(t, p) for t, p in pvt if p["mode"] and p["vu"] is not None]
    # time tie: Vu of fixes in the early boost against the truth, offset scanned at 10 ms
    # the rising branch only: fixes up to the receiver's fastest one, the truth up to burnout
    t_top = max(fixes, key=lambda x: x[1]["vu"])[0] if fixes else 0
    fit = [(t, p["vu"]) for t, p in fixes if 20 <= p["vu"] <= 450 and t <= t_top]
    if len(fit) < 5:
        # every p180 file starts at TOW 203820 (ignition 08:40:00 GPST); underrun gaps can shift it slightly
        off = DESIGN_TIE
        print(f"{Path(a.capture).name}\nscenario {scen}: only {len(fit)} boost fixes -- truth t = receiver TOW - "
              f"{off:.2f} s (the files' design, not fitted); ignition at truth t {pro:.1f}")
    else:
        ft, fv = np.array(fit).T
        rise = (tt > pro - 5) & (tt <= tt[int(np.argmax(tv))])
        lift_rx = ft[0] - np.interp(fv[0], tv[rise], tt[rise])
        offs = np.arange(lift_rx - 2, lift_rx + 2, 0.01)
        err = [np.median(np.abs(np.interp(ft - o, tt, tv) - fv)) for o in offs]
        off = offs[int(np.argmin(err))]
        print(f"{Path(a.capture).name}\nscenario {scen}: truth t = receiver TOW - {off:.2f} s "
              f"(Vu fit median |error| {min(err):.2f} m/s over {len(fit)} fixes); ignition at truth t {pro:.1f}")

    def at(t):
        u = t - off
        return np.interp(u, tt, ts), np.interp(u, tt, th), u - pro

    # error-7 stretches
    blocked = [(t, p["error"] == 7) for t, p in pvt]
    runs, cur = [], None
    for t, b in blocked:
        if b and cur is None:
            cur = [t, t]
        elif b:
            cur[1] = t
        elif cur is not None:
            runs.append(cur)
            cur = None
    if cur:
        runs.append(cur)
    print(f"export-law stretches (error 7): {len(runs)}")
    for t0, t1 in runs:
        before = [(t, p) for t, p in fixes if t < t0]
        after = [(t, p) for t, p in fixes if t > t1]
        s0, h0, T0 = at(t0)
        s1, h1, T1 = at(t1)
        line = f"  T{T0:+8.2f} .. T{T1:+8.2f} s  (truth {s0:6.1f} -> {s1:6.1f} m/s, {h0 / 1000:6.2f} -> {h1 / 1000:6.2f} km)"
        if before:
            tb, pb = before[-1]
            sb, hb, Tb = at(tb)
            line += (f"\n      last fix before  T{Tb:+8.2f}: truth {sb:6.1f} m/s {hb / 1000:6.2f} km;"
                     f" receiver Vu {pb['vu']:6.1f}")
        if after:
            ta, pa = after[0]
            sa, ha, Ta = at(ta)
            line += (f"\n      first fix after  T{Ta:+8.2f}: truth {sa:6.1f} m/s {ha / 1000:6.2f} km;"
                     f" receiver Vu {pa['vu']:6.1f}")
        print(line)
    # raw measurements: stretches with none, after lift-off
    gaps, cur = [], None
    for t, n in meas:
        if at(t)[2] < -5:
            continue
        if n == 0 and cur is None:
            cur = [t, t]
        elif n == 0:
            cur[1] = t
        elif cur is not None:
            gaps.append(cur)
            cur = None
    if cur:
        gaps.append(cur)
    pad = {}
    for t, s, c in cn0:
        if -60 <= at(t)[2] <= -5:
            pad.setdefault(s, []).append(c)
    print("pad C/N0 (T-60..T-5, the file's signals), median: "
          + ", ".join(f"{s} {np.median(v):.1f} (n {len(v)})" for s, v in sorted(pad.items())))
    ur = underruns(a.capture)
    print("transmitter underruns at: " + ("no .hackrf.txt" if ur is None else
                                           (", ".join(f"T{x:+.1f}" for x in ur) or "none")))
    print("no raw measurements (MeasEpoch empty) from T-5 on:")
    for t0, t1 in gaps:
        if t1 - t0 >= 0.2:
            s0, h0, T0 = at(t0)
            s1, h1, T1 = at(t1)
            print(f"  T{T0:+8.2f} .. T{T1:+8.2f} s  (truth {s0:6.1f} -> {s1:6.1f} m/s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
