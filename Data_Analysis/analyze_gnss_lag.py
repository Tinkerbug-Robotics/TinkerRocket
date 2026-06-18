#!/usr/bin/env python3
"""Flag GPS position-vs-velocity inconsistency (#242).

On the 6/14 Rolly Polly flights the u-blox GPS solution was corrupted through
boost *while reporting healthy* (fix_mode=3, good h_acc/v_acc): the reported
position lagged ~4 s and dived, and the Doppler vertical velocity carried large
noise.  The two GPS-internal vertical-motion signals should agree —
d(altitude)/dt must equal vel_u — so this tool cross-correlates them and reports
the lag plus the RMS / worst disagreement, flagging a flight when GPS vertical
motion is internally inconsistent (the signature that makes GPS unusable for
apogee, see #237/#249).

Usage:
    python analyze_gnss_lag.py <flight.bin> [<flight.bin> ...]
"""
import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from plot_flight_data_mini import parse_binary_file

LAG_FLAG_S      = 1.0    # |pos-rate vs velocity| lag beyond this is suspect
RMS_FLAG_MPS    = 5.0    # RMS(d_alt/dt - vel_u) beyond this is suspect


def analyze(bin_path: str) -> bool:
    """Return True if the flight's GPS vertical motion is internally consistent."""
    name = Path(bin_path).name
    rec, _, _ = parse_binary_file(str(bin_path))
    g = [r for r in rec.get("GNSS", []) if "alt_m" in r and "vel_u" in r]
    if len(g) < 20:
        print(f"{name}: too few GNSS samples ({len(g)}) — skipped")
        return True

    t = np.array([r["time_us"] for r in g], float) / 1e6
    t -= t[0]
    alt = np.array([r["alt_m"] for r in g], float)
    vu  = np.array([r["vel_u"] for r in g], float)
    fix = sorted({r["fix_mode"] for r in g})

    dt = float(np.median(np.diff(t)))
    grid = np.arange(t[0], t[-1], dt)
    alt_rate = np.gradient(np.interp(grid, t, alt), grid)   # position-derived rate
    vu_i     = np.interp(grid, t, vu)                        # Doppler velocity

    # Cross-correlation lag: how much does position-rate trail velocity?
    a = (alt_rate - alt_rate.mean()) / (alt_rate.std() + 1e-9)
    b = (vu_i - vu_i.mean()) / (vu_i.std() + 1e-9)
    lags = np.arange(-5.0, 5.0 + dt, dt)
    corr = [float(np.mean(a * np.interp(grid + L, grid, b, left=b[0], right=b[-1])))
            for L in lags]
    lag = float(lags[int(np.argmax(corr))])

    disagree = alt_rate - vu_i
    rms  = float(np.sqrt(np.mean(disagree**2)))
    worst = float(np.max(np.abs(disagree)))

    ok = abs(lag) <= LAG_FLAG_S and rms <= RMS_FLAG_MPS
    verdict = "OK" if ok else "FLAG: GPS vertical motion internally inconsistent"
    print(f"{name}:")
    print(f"   pos-rate vs velocity: lag={lag:+.2f}s (corr {max(corr):.2f}), "
          f"RMS disagree={rms:.1f} m/s, worst={worst:.1f} m/s, fix_modes={fix}")
    print(f"   -> [{verdict}]")
    return ok


def main() -> int:
    paths = sys.argv[1:]
    if not paths:
        print(__doc__)
        return 2
    all_ok = all(analyze(p) for p in paths)
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
