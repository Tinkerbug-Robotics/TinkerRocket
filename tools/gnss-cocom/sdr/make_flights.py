#!/usr/bin/env python3
"""Realistic rocket flight profiles, for measuring how fast the gate re-opens.

The scenarios in make_trajectories.py are deliberately artificial: single-variable
ramps that answer *where* the COCOM thresholds are. This file answers a different
question -- once a flight stops exceeding a limit, how long before the receiver
gives position back? A ramp cannot answer that, because it never comes back.

Two profiles, both flown vertically and integrated from thrust, drag and gravity
rather than drawn by hand, so the velocity and altitude the receiver sees have
the shape a real flight has:

  boostthrough  A high-power flight that punches through 515 m/s on the way up
                and decelerates back below it well under 18 km. Isolates
                recovery from the *velocity* gate, with altitude never a factor.

  spaceshot     Apogee just above 80 km. Deliberately just above: the crossing
                speed at 80 km is sqrt(2*g*(apogee-80km)), so a 100 km apogee
                would cross at 626 m/s and the velocity gate would mask the
                altitude recovery entirely. Around 86 km it crosses near
                330 m/s, well clear of 515, so the altitude gate is isolated --
                while the boost phase still exceeds 515 earlier in the same
                flight, giving both recoveries in one run.

Atmosphere is the US Standard piecewise fit up to 32 km and an exponential
continuation above, which is far more than the drag above 30 km deserves but
keeps the ascent shape honest lower down.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from make_trajectories import (ALT_LIMIT_CANDIDATES_M, LAT0_DEG, LON0_DEG,
                               RATE_HZ, V_LIMIT_MPS, deg_lon_per_metre)

G0 = 9.80665
R_AIR = 287.05


def density(h: float) -> float:
    """US Standard Atmosphere density [kg/m^3], exponential above 32 km."""
    if h < 0:
        h = 0.0
    if h < 11_000:
        t = 288.15 - 0.0065 * h
        p = 101325.0 * (t / 288.15) ** 5.2559
    elif h < 20_000:
        t = 216.65
        p = 22632.1 * math.exp(-G0 * (h - 11_000) / (R_AIR * t))
    elif h < 32_000:
        t = 216.65 + 0.001 * (h - 20_000)
        p = 5474.89 * (t / 216.65) ** (-G0 / (0.001 * R_AIR))
    else:
        # Above the drag-relevant region; a single scale height is plenty.
        t = 228.65
        p = 868.02 * math.exp(-(h - 32_000) / 7_500.0)
    return p / (R_AIR * t)


def gravity(h: float) -> float:
    return G0 * (6_371_000.0 / (6_371_000.0 + h)) ** 2


def fly(burn_s: float, accel_mps2: float, cd_a_over_m: float,
        alt0_m: float, stop_alt_m: float, max_s: float):
    """Integrate a vertical flight. Returns [(t, alt, v_up)] at RATE_HZ.

    cd_a_over_m is the inverse ballistic coefficient, Cd*A/m [m^2/kg]: the only
    aerodynamic parameter that matters for a 1-D flight, so there is no point
    carrying mass, diameter and Cd separately.
    """
    dt = 1.0 / RATE_HZ
    t, h, v = 0.0, alt0_m, 0.0
    out = [(0.0, h, 0.0)]
    apogee_seen = False

    while t < max_s:
        thrust_a = accel_mps2 if t < burn_s else 0.0
        drag_a = 0.5 * density(h) * cd_a_over_m * v * abs(v)
        a = thrust_a - gravity(h) - drag_a       # drag opposes motion via v*|v|
        v += a * dt
        h += v * dt
        t += dt
        if v < 0:
            apogee_seen = True
        if apogee_seen and h <= stop_alt_m:
            break
        out.append((round(t, 1), h, v))
    return out


# burn_s, accel, Cd*A/m, alt0, stop_alt, max_s, purpose
FLIGHTS = {
    "boostthrough": dict(
        prologue_s=240.0,
        burn_s=9.0, accel_mps2=152.6, cd_a_over_m=1.5e-4,
        alt0_m=1_200.0, stop_alt_m=2_000.0, max_s=200.0,
        purpose="Apogee 40 km, peak 1068 m/s. Isolates recovery from the "
                "VELOCITY gate: 34 s above 515 m/s on the way up, then 106 s "
                "clear before the descent re-exceeds it, with altitude never "
                "within half the 80 km gate. Apogee is set at 40 km rather "
                "than under 18 km because the 18 km gate was shown not to "
                "exist -- staying under 80 km is the only constraint, and the "
                "extra energy buys a far longer blocked window to recover from.",
    ),
    # Same apogee as spaceshot, flown at 3 g instead of 15 g. This exists
    # because the realistic profiles answered a different question than the one
    # asked: at 15 g the receiver dropped from 7 satellites to 2 within a second
    # of ignition, at only 142 m/s, and spaceshot never re-acquired at all. That
    # is Doppler *rate*, not COCOM -- 15 g is 787 Hz/s on L1, past what the
    # tracking loops hold, while 3 g is 154 Hz/s. Flying the same trajectory
    # gently keeps the receiver tracking throughout, so what is left to measure
    # is the gate re-opening rather than a re-acquisition.
    "gentle_alt": dict(
        prologue_s=240.0,
        burn_s=60.3, accel_mps2=29.4, cd_a_over_m=1.0e-4,
        alt0_m=1_200.0, stop_alt_m=60_000.0, max_s=250.0,
        purpose="Apogee 82.8 km at 3 g. Gives both recoveries in one flight "
                "with the dynamics kept inside what the receiver can track: "
                "79 s blocked on velocity, 30 s clear, 48 s blocked on "
                "altitude above 80 km, 30 s clear on the way back down.",
    ),
    "spaceshot": dict(
        prologue_s=240.0,
        burn_s=12.0, accel_mps2=142.2, cd_a_over_m=1.0e-4,
        alt0_m=1_200.0, stop_alt_m=60_000.0, max_s=260.0,
        purpose="Apogee 82.5 km -- only just above the gate, and that is the "
                "point: crossing speed at 80 km is sqrt(2g(apogee-80km)), so a "
                "100 km apogee would cross at 626 m/s and the velocity gate "
                "would mask the altitude recovery entirely. At 82.5 km it "
                "crosses near 220 m/s and takes ~31 s to re-reach 515 m/s on "
                "the way down, which is the window the altitude recovery has "
                "to be measured in. Boost exceeds 515 m/s earlier, so one "
                "flight gives both recoveries: velocity, then altitude.",
    ),
}


def windows(samples, limit_fn):
    """Contiguous [start, end] spans where limit_fn(sample) is True."""
    out, run = [], None
    for s in samples:
        if limit_fn(s):
            if run is None:
                run = [s["t"], s["t"]]
            else:
                run[1] = s["t"]
        elif run is not None:
            out.append(run)
            run = None
    if run is not None:
        out.append(run)
    return out


def build(name: str, spec: dict):
    pro = spec["prologue_s"]
    flight = fly(spec["burn_s"], spec["accel_mps2"], spec["cd_a_over_m"],
                 spec["alt0_m"], spec["stop_alt_m"], spec["max_s"])

    rows, truth = [], []
    east_m = 0.0
    # Stationary prologue: the receiver needs 43-186 s to acquire, so the flight
    # cannot start until it is locked, or every run reads NO_LOCK for reasons
    # that have nothing to do with the gate.
    n_pro = int(round(pro * RATE_HZ))
    for i in range(n_pro):
        t = i / RATE_HZ
        rows.append(f"{t:.1f},{LAT0_DEG:.9f},{LON0_DEG:.9f},{spec['alt0_m']:.3f}")
        truth.append(dict(t=round(t, 1), alt_m=spec["alt0_m"], speed_mps=0.0,
                          v_east_mps=0.0, v_up_mps=0.0, phase="prologue"))

    for ft, h, v in flight:
        t = pro + ft
        lon = LON0_DEG + east_m * deg_lon_per_metre(LAT0_DEG, h)
        rows.append(f"{t:.1f},{LAT0_DEG:.9f},{lon:.9f},{h:.3f}")
        truth.append(dict(t=round(t, 1), alt_m=h, speed_mps=abs(v),
                          v_east_mps=0.0, v_up_mps=v,
                          phase="boost" if ft < spec["burn_s"] else
                                ("coast" if v >= 0 else "descent")))

    return rows, truth


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--outdir", default="scenarios", type=Path)
    ap.add_argument("--only", action="append", metavar="NAME")
    args = ap.parse_args()

    names = args.only or list(FLIGHTS)
    args.outdir.mkdir(parents=True, exist_ok=True)

    for name in names:
        spec = FLIGHTS[name]
        rows, truth = build(name, spec)
        dur = truth[-1]["t"]

        vel_w = windows(truth, lambda s: s["speed_mps"] > V_LIMIT_MPS)
        alt_w = {int(c / 1000): windows(truth, lambda s, c=c: s["alt_m"] > c)
                 for c in ALT_LIMIT_CANDIDATES_M}
        # A gate that is actually closed needs velocity OR altitude(80 km)
        # exceeded, which is what the previous runs established.
        blocked_w = windows(truth, lambda s: s["speed_mps"] > V_LIMIT_MPS
                            or s["alt_m"] > 80_000.0)

        (args.outdir / f"{name}.csv").write_text("\n".join(rows) + "\n")
        meta = dict(
            scenario=name, purpose=spec["purpose"], duration_s=dur,
            rate_hz=RATE_HZ, prologue_s=spec["prologue_s"],
            origin=dict(lat_deg=LAT0_DEG, lon_deg=LON0_DEG),
            limits=dict(velocity_mps=V_LIMIT_MPS,
                        altitude_candidates_m=list(ALT_LIMIT_CANDIDATES_M)),
            crossings=dict(
                velocity_515=vel_w[0][0] if vel_w else None,
                peak_speed_mps=max(s["speed_mps"] for s in truth),
                peak_alt_m=max(s["alt_m"] for s in truth),
                **{f"altitude_{k}km": (w[0][0] if w else None)
                   for k, w in alt_w.items()}),
            exceeds=("BOTH" if vel_w and alt_w.get(80) else
                     "VELOCITY only" if vel_w else
                     "ALTITUDE only" if alt_w.get(80) else "NEITHER"),
            velocity_windows=vel_w, altitude_80km_windows=alt_w.get(80, []),
            blocked_windows=blocked_w,
            truth=truth,
        )
        (args.outdir / f"{name}.json").write_text(json.dumps(meta, indent=1))

        peak_a = meta["crossings"]["peak_alt_m"] / 1000
        peak_v = meta["crossings"]["peak_speed_mps"]
        print(f"{name}: {dur:.0f}s total ({spec['prologue_s']:.0f}s prologue), "
              f"apogee {peak_a:.1f} km, peak {peak_v:.0f} m/s, "
              f"{dur*RATE_HZ*2*2.6e6/1e9/RATE_HZ:.2f} GB")
        for a, b in vel_w:
            print(f"    >515 m/s   t={a:.1f}..{b:.1f}s  ({b-a:.1f}s)")
        for a, b in alt_w.get(80, []):
            print(f"    >80 km     t={a:.1f}..{b:.1f}s  ({b-a:.1f}s)")
        for a, b in blocked_w:
            recov = next((s for s in truth if s["t"] > b), None)
            print(f"    GATE SHUT  t={a:.1f}..{b:.1f}s -> clear at {b:.1f}s"
                  + (f", then {recov['speed_mps']:.0f} m/s at "
                     f"{recov['alt_m']/1000:.1f} km" if recov else ""))
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
