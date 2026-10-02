#!/usr/bin/env python3
"""Pseudorange and range-rate accuracy of a NEO-M8T SignalSim capture (run_radiated lines 't U <cls><id><hex>'),
GPS / Galileo / BeiDou, by pr_accuracy.py's method and with its tables.

The M8T flights logged no navigation data (RXM-SFRBX off), so the ephemerides come from PX1105R captures of the same
2026 SignalSim scenarios -- the same RINEX and epoch, so the same broadcast data -- pooled over several captures so
every satellite is covered (the PX1105R acquires few Galileo satellites in the wide files; the narrow GPS + Galileo
runs have them all), cached in work/m8t_eph.pkl. Only RXM-RAWX measurements with a valid pseudorange are used.

Timing (m8t_timing_scan.py, traveler burn): the M8T's clock runs ~3 ms ahead of GPS time, but re-timing the truth by
that clock makes the burn residuals worse (the pseudoranges fit best ~5 ms AFTER the tag), so by default the truth is
taken at the time tags, as pr_accuracy does for the PX1105R (--retime does the clock-based second pass anyway). The
Doppler fits the truth 0.01 s earlier (the PX1105R's: 0.03 s).

    m8t_accuracy.py CAPTURE OUT.npz [--scenario traveler|hotshot] [--rr-lag 0.01] [--retime]
"""
import argparse
import os
import pickle
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pr_accuracy as P                                                  # noqa: E402

CAPS = Path(os.environ.get("CAPTURES", str(P.SDR / "captures")))      # the PX1105R captures lending ephemerides
EPH_FROM = ["px1105r_signalsim_traveler_all_2026_45_w_p180_cofs_gain3_nav9_el3_traveler2026wcr.log",
            "px1105r_signalsim_traveler_all_2026_57_w_p180_gain3_nav9_el3_traveler2026wp12.log",
            "px1105r_signalsim_hotshot_all_2026_57_w_p180_gain3_nav9_el3_hotshot2026hs12.log",
            "px1105r_signalsim_hotshot_all_2026_45_w_p180_gain3_nav9_el3_hotshot2026hs0.log",
            "px1105r_signalsim_traveler_gpsgal_2026_45_n_gain3_nav9_el3_traveler2026.log",
            "px1105r_signalsim_traveler_gpsgal_2026_50e_n_cofs_gain3_nav9_el3_traveler2026ec.log"]
GNSS = {0: "G", 2: "E", 3: "C"}
C_MS = 299792458.0
SHIFT = None                                   # (file times, receiver clock s) once the first pass has solved it


def pooled_eph():
    cache = HERE / "work" / "m8t_eph.pkl"
    cache.parent.mkdir(exist_ok=True)
    if cache.exists():
        return pickle.loads(cache.read_bytes())
    eph = None
    for name in EPH_FROM:
        e = P.load_capture(str(CAPS / name), systems="GEC")[0]
        if eph is None:
            eph = e
            continue
        for key, es in e.eph.items():
            for x in es:
                eph._keep(key, x)
        if eph.ion is None:
            eph.ion = e.ion
    cache.write_bytes(pickle.dumps(eph))
    return eph


def m8t_epochs(cap):
    """RXM-RAWX -> {k: (rcvTow, [(sys, prn, pr, doppler, cn0, None, 0)])}, valid pseudoranges only, in log order;
    one signal per satellite, the first valid one (a ZED-F9P can report several signals of a satellite)."""
    ep = {}
    for line in open(cap, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3 or p[1] != "U" or not p[2].startswith("0215"):
            continue
        try:
            b = bytes.fromhex(p[2].strip()[4:])
        except ValueError:
            continue
        if len(b) < 16:
            continue
        obs, seen = [], set()
        for j in range(b[11]):
            m = b[16 + 32 * j: 48 + 32 * j]
            if len(m) == 32 and m[20] in GNSS and m[30] & 1 and (m[20], m[21]) not in seen:
                seen.add((m[20], m[21]))
                obs.append((GNSS[m[20]], m[21], struct.unpack_from("<d", m, 0)[0],
                            float(struct.unpack_from("<f", m, 16)[0]), m[26], None, 0))
        if obs:
            ep[len(ep)] = (struct.unpack_from("<d", b, 0)[0], obs)
    return ep


class ShiftedTruth(P.SignalSimTruth):
    """SignalSim's trajectory, evaluated at the true receive time once SHIFT holds the solved receiver clock."""

    def _t(self, t):
        return t if SHIFT is None else t - float(np.interp(t, SHIFT[0], SHIFT[1]))

    def vup(self, t):
        return super().vup(self._t(t))

    def alt(self, t):
        return super().alt(self._t(t))


def main():
    global SHIFT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("out")
    ap.add_argument("--scenario", default="traveler", choices=("traveler", "hotshot"))
    ap.add_argument("--rr-lag", dest="rr_lag", default="0.01")
    ap.add_argument("--retime", action="store_true")
    a = ap.parse_args()
    csv, scen = (("traveler_soft25_pad600.csv", "traveler_soft25.json") if a.scenario == "traveler" else
                 ("hotshot_pad600.csv", "hotshot.json"))
    eph = pooled_eph()
    print("pooled ephemerides:", {s: sorted(p for (q, p) in eph.eph if q == s) for s in "GEC"})
    ep = m8t_epochs(a.capture)
    seen = {(o[0], o[1]) for _, obs in ep.values() for o in obs}
    t_mid = ep[len(ep) // 2][0]
    missing = sorted(k for k in seen if eph.pick(k[0], k[1], t_mid) is None)
    print(f"RAWX epochs {len(ep)}, satellites {len(seen)}; no ephemeris for {missing or 'none'}")
    P.load_capture = lambda path, systems="GEC": (eph, ep, {}, "ubx")
    P.SignalSimTruth = ShiftedTruth
    argv = [a.capture, a.out, "--csv", str(P.SDR / "scenarios" / csv), "--scenario", str(P.SDR / "scenarios" / scen),
            "--stride", "1", "--rr-lag", a.rr_lag]
    sys.argv = ["pr_accuracy.py"] + argv
    print("\n######## truth at the M8T's time tags")
    P.main()
    z = np.load(a.out)
    pad = (z["ut"] > -170) & (z["ut"] < -5)
    print(f"\nreceiver clock on the pad: {np.median(z['clock'][pad]) / C_MS * 1e3:+.4f} ms (median), "
          f"{np.ptp(z['clock'][pad]) / C_MS * 1e6:.1f} us spread")
    if a.retime:
        SHIFT = (z["ut"] + P.IGN, z["clock"] / C_MS)
        print("\n######## pass 2 (truth at epoch minus receiver clock)")
        P.main()


if __name__ == "__main__":
    main()
