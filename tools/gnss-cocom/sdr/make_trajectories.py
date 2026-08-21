#!/usr/bin/env python3
"""Trajectory generator for the #491 COCOM bench tests.

Emits gps-sdr-sim `-x` user-motion files: `t,lat,lon,height` at 10 Hz, which is
the rate gps-sdr-sim requires.  The `-x` (lat/lon/height) form is used rather
than the `-u` (ECEF) form the issue sketched, because it removes the pyproj
dependency entirely -- the only geodesy left is metres-east to degrees-longitude.

Each scenario is defined by *velocity* functions that are integrated, not by
position functions that are differentiated.  That ordering is deliberate: the
COCOM velocity limit acts on the receiver's 3-D speed, so the vertical
component of an altitude ramp counts against it.  Defining altitude directly
makes it easy to write a "pure altitude" test that quietly exceeds the velocity
limit as well -- which is exactly the flaw in the test-2 trajectory as filed
(10 km to 85 km in 120 s is a 625 m/s climb, over the 515 m/s limit, so it
would have tripped both gates and settled nothing).

Every scenario is checked against both limits and the summary prints which one
each run actually exceeds, so that class of mistake cannot survive quietly.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

# --- COCOM limits under test ------------------------------------------------
# 515 m/s is the one both readings agree on.  The altitude gate is the open
# question: 18 km (classic COCOM, SkyTraq's own FAQ) or 80 km (the figure
# printed in the PX11xx datasheets).  Both are tracked so the summary can say
# which candidate a given run brackets.
V_LIMIT_MPS = 515.0
ALT_LIMIT_CANDIDATES_M = (18_000.0, 80_000.0)

RATE_HZ = 10.0                    # gps-sdr-sim requires exactly 10 Hz
# The binary in bin/ reports "dynamic mode max: 1000" and has already produced
# an 848 s scenario (gentle_alt), so the old 600 s figure here was stale and was
# rejecting runs the simulator accepts. Keep this in step with `gps-sdr-sim -h`
# rather than with whatever it was last built as.
MAX_DURATION_S = 1000.0           # gps-sdr-sim -DUSER_MOTION_SIZE=10000

# Open-sky origin.  Nothing depends on the exact spot; a mid-latitude land
# location keeps the simulated constellation geometry unremarkable.
LAT0_DEG = 40.0
LON0_DEG = -119.0

# WGS84, for metres-east -> degrees-longitude.
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def deg_lon_per_metre(lat_deg: float, height_m: float) -> float:
    """Degrees of longitude per metre east, on the WGS84 ellipsoid at height."""
    lat = math.radians(lat_deg)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    return 180.0 / (math.pi * (n + height_m) * math.cos(lat))


def ramp(t: float, t0: float, t1: float, v0: float, v1: float) -> float:
    """Linear segment: v0 before t0, v1 after t1, interpolated between.

    Used to keep accelerations finite.  A step change in velocity is not just
    unphysical, it stresses the receiver's tracking loops and risks a genuine
    loss of lock landing in the middle of a run -- which is the one outcome
    this test cannot tell apart from the thing it is trying to measure.
    """
    if t <= t0:
        return v0
    if t >= t1:
        return v1
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0)



def _stair(t, v0, step, dwell, n):
    """Staircase in speed: n levels of `dwell` seconds, `step` apart.

    Transitions are ramped over 2 s rather than stepped. A true discontinuity
    is not merely unphysical, it stresses the tracking loops and can cause a
    genuine loss of lock in the middle of a dwell -- the one outcome this test
    cannot distinguish from the gate it is measuring.
    """
    edge = 2.0
    k = min(int(t // dwell), n - 1)
    frac = t - k * dwell
    lo = v0 + step * k
    if k == 0 or frac >= edge:
        return lo
    return ramp(frac, 0.0, edge, lo - step, lo)


def _stair_climb(t, dwell, n, climb):
    """Vertical rate that dwells, then climbs briefly to the next level.

    Returns climb rate rather than altitude, because the scenario integrates
    velocity. Each level holds for `dwell` seconds at zero vertical rate, then
    climbs at `climb` m/s for exactly as long as the next step needs.
    """
    steps_m = [2000.0, 1000.0, 1000.0, 1000.0, 1000.0]     # 76->78->79->80->81->82
    k = min(int(t // dwell), n - 1)
    frac = t - k * dwell
    if k >= len(steps_m):
        return 0.0
    climb_s = steps_m[k] / climb
    hold = dwell - climb_s
    return 0.0 if frac < hold else climb


def _blips(t, lengths, clear_s, v_low, v_high):
    """Excursions above the limit of the given lengths, separated by clear runs.

    v_low sits well under the velocity limit and v_high well over it, so each
    excursion is unambiguous, and the clear intervals are long enough for even
    the slowest recovery observed (134 s) to complete inside one.
    """
    edge = 2.0
    cursor = 0.0
    for n in lengths:
        if t < cursor + clear_s:
            return v_low
        cursor += clear_s
        if t < cursor + n:
            f = t - cursor
            if f < edge:
                return ramp(f, 0.0, edge, v_low, v_high)
            return v_high
        cursor += n
    return v_low


def windows(samples, limit_fn):
    """Contiguous [start, end] intervals where limit_fn(sample) holds.

    Emitted so recovery.py and correlate.py can work against these scenarios at
    all: both key off `blocked_windows`, which only make_flights.py used to
    write. Without it a staircase reports zero windows and recovery.py has
    nothing to measure -- which is exactly what the dwell scenarios are for.
    """
    out, start = [], None
    for smp in samples:
        hit = limit_fn(smp)
        if hit and start is None:
            start = smp["t"]
        elif not hit and start is not None:
            out.append([start, smp["t"]])
            start = None
    if start is not None:
        out.append([start, samples[-1]["t"]])
    return out



def _stair_climb_low(t, dwell, n, climb):
    """Vertical rate for the 12/14/16/18/20/22 km staircase.

    Separate from _stair_climb because the step sizes differ; sharing one
    helper with a steps list passed in would read more cleanly but this stays
    parallel to the scenario above it, where the levels are written out.
    """
    steps_m = [2000.0, 2000.0, 2000.0, 2000.0, 2000.0]
    k = min(int(t // dwell), n - 1)
    frac = t - k * dwell
    if k >= len(steps_m):
        return 0.0
    climb_s = steps_m[k] / climb
    hold = dwell - climb_s
    return 0.0 if frac < hold else climb



def _stair_climb_vlow(t, dwell, n, climb):
    """Vertical rate for the 8/9/10/11/12/13 km staircase (1 km steps)."""
    steps_m = [1000.0] * 5
    k = min(int(t // dwell), n - 1)
    frac = t - k * dwell
    if k >= len(steps_m):
        return 0.0
    climb_s = steps_m[k] / climb
    return 0.0 if frac < dwell - climb_s else climb


# --- Scenarios --------------------------------------------------------------
# Each entry: duration_s, alt0_m, v_east(t), v_up(t), and what it is for.
# v_east and v_up are in m/s and are integrated at RATE_HZ.

SCENARIOS = {
    # Level-setting warm-up. Stationary, so it is the one scenario where the
    # PortaPack's Loop checkbox (and hackrf_transfer's -R) may be left ON: a
    # static trajectory has no position to jump back to at EOF, only GPS time,
    # so it can repeat all through a gain sweep without being restarted by hand.
    # Every other scenario is a ramp, where looping teleports the receiver back
    # to t=0 mid-capture and destroys the run.
    "t00_static": dict(
        duration_s=120.0,
        alt0_m=100.0,
        v_east=lambda t: 0.0,
        v_up=lambda t: 0.0,
        purpose="Stationary warm-up for setting the injection level. Loopable, "
                "so one file covers a whole gain sweep. Prove a fix here before "
                "anything else is connected or concluded.",
    ),
    # --- Scenarios for a receiver whose gate is LATENT ----------------------
    # A ramp cannot bracket a threshold that the receiver reacts to slowly. The
    # Air530 closes its gate 1.6-8.9 s late, and on a 3 g climb the vehicle
    # spends only ~1 s within +/-15 m/s of the limit, so the lag smears the
    # bracket by 47-262 m/s: what comes back is the latency, not the threshold.
    # The fix is to stop ramping and dwell -- hold a speed steady for far longer
    # than the lag, and the receiver has time to make up its mind at each level.
    "vel_stair": dict(
        prologue_s=180.0,
        duration_s=740.0,
        alt0_m=5_000.0,
        v_east=lambda t: _stair(t, 495.0, 5.0, 90.0, 8),
        v_up=lambda t: 0.0,
        purpose="Velocity staircase: 90 s dwells at 495-530 m/s in 5 m/s steps, "
                "at a constant 5 km. Ninety seconds is ten times the worst close "
                "lag seen on any part, so the first level that blocks IS the "
                "threshold, bracketed to 5 m/s rather than to the latency.",
    ),
    # Same trick on the other axis. The Air530 has no measured altitude gate at
    # all -- no clean edge ever appeared on a flight profile -- so this is the
    # only way to get one. Speed is held at ~150 m/s, well clear of the velocity
    # limit, and the climbs between dwells are brief.
    # The Air530 turned out to have an altitude ceiling of its own, somewhere
    # near 10-16 km on flight profiles -- close enough to 18 km (60,000 ft, the
    # commonly-quoted COCOM altitude) to be worth bracketing properly. alt_stair
    # dwells at 76-82 km and is entirely above it, so it cannot see this at all.
    # Same dwell trick, an order of magnitude lower.
    "alt_stair_low": dict(
        prologue_s=180.0,
        duration_s=560.0,
        alt0_m=12_000.0,
        v_east=lambda t: 120.0,
        v_up=lambda t: _stair_climb_low(t, 90.0, 6, 150.0),
        purpose="Low altitude staircase: 90 s dwells at 12/14/16/18/20/22 km "
                "with speed held near 120 m/s. Brackets an altitude gate in the "
                "18 km region, which no ramp can do on a part this latent.",
    ),
    # alt_stair_low found nothing above 12 km, and the flights put the highest
    # Air530 fix at 10.50 km, so the ceiling sits in a 1.5 km window. One more
    # octave down, in 1 km steps, to close it.
    "alt_stair_vlow": dict(
        prologue_s=180.0,
        duration_s=560.0,
        alt0_m=8_000.0,
        v_east=lambda t: 120.0,
        v_up=lambda t: _stair_climb_vlow(t, 90.0, 6, 100.0),
        purpose="Very low altitude staircase: 90 s dwells at 8/9/10/11/12/13 km. "
                "Closes the bracket on an altitude ceiling that the flight "
                "profiles could only place somewhere between 10.5 and 12 km.",
    ),
    "alt_stair": dict(
        prologue_s=180.0,
        duration_s=560.0,
        alt0_m=76_000.0,
        v_east=lambda t: 150.0,
        v_up=lambda t: _stair_climb(t, 90.0, 6, 200.0),
        purpose="Altitude staircase: 90 s dwells at 76/78/79/80/81/82 km with "
                "speed held near 150 m/s. Brackets an altitude gate on a part "
                "too latent for a ramp to measure.",
    ),
    # Does the gate re-open slowly, or does the receiver drop its navigation
    # state when gated and have to converge again? The Air530's recoveries were
    # 31, 63 and 134 s and its WARM TTFF is ~40 s, which is suspiciously close.
    # Excursions of very different lengths separate the two: a fixed
    # re-convergence cost does not care how long the block lasted, and a slow
    # gate should.
    "blockdur": dict(
        prologue_s=180.0,
        duration_s=805.0,
        alt0_m=5_000.0,
        # 5/30/150 s spans a 30x range of block lengths, and 155 s of clear
        # after each exceeds the longest recovery yet observed (134 s), so a
        # slow recovery has room to finish inside its own interval instead of
        # running into the next excursion. Four blips would have overrun
        # gps-sdr-sim's 1000 s dynamic-mode cap and silently truncated the last.
        v_east=lambda t: _blips(t, [5.0, 30.0, 150.0], 155.0, 300.0, 560.0),
        v_up=lambda t: 0.0,
        purpose="Four excursions over the velocity limit lasting 5, 15, 45 and "
                "120 s, each followed by 150 s comfortably clear. If recovery "
                "time is flat regardless of block length the receiver is "
                "re-converging, not holding a gate shut.",
    ),
    "t0_baseline": dict(
        prologue_s=120.0,
        duration_s=60.0,
        alt0_m=1_000.0,
        v_east=lambda t: 50.0,
        v_up=lambda t: 0.0,
        purpose="Baseline. Neither limit approached. Proves the RF chain, the "
                "injection level and the UTC-to-trajectory time mapping before "
                "any threshold is probed.",
    ),
    # Velocity alone. Altitude parked at 5 km, far below either candidate gate.
    # 0 -> 900 m/s over 90 s is 10 m/s^2, about 1 g: gentle enough that the
    # tracking loops are never the limiting factor.
    "t1_velramp": dict(
        prologue_s=240.0,
        duration_s=90.0,
        alt0_m=5_000.0,
        v_east=lambda t: ramp(t, 0.0, 90.0, 0.0, 900.0),
        v_up=lambda t: 0.0,
        purpose="Velocity ramp with altitude held low. The core AND-vs-OR "
                "measurement: if position survives past 515 m/s the gate is not "
                "independent on velocity.",
    ),
    # Altitude alone. Climb rate 350 m/s keeps 3-D speed at ~354 m/s, 69% of the
    # velocity limit -- enough margin that a noisy velocity estimate cannot
    # nudge it over. One continuous ramp crosses BOTH candidate gates, so a
    # blank-out is never ambiguous between "gated" and "never acquired up here".
    "t2_altramp": dict(
        prologue_s=240.0,
        duration_s=240.0,
        alt0_m=5_000.0,
        v_east=lambda t: 50.0,
        v_up=lambda t: ramp(t, 0.0, 20.0, 0.0, 350.0),
        purpose="Altitude ramp with speed held well under the limit. Crosses "
                "18 km and 80 km in one continuous run, so it reads the "
                "altitude gate and its threshold in a single capture.",
    ),
    # Both limits, exceeded together, on the 18 km reading of the altitude gate.
    "t3a_both_18km": dict(
        prologue_s=240.0,
        duration_s=70.0,
        alt0_m=5_000.0,
        v_east=lambda t: ramp(t, 0.0, 70.0, 0.0, 700.0),
        v_up=lambda t: ramp(t, 0.0, 10.0, 0.0, 340.0),
        purpose="Confirmation run for an 18 km gate: climbs through 18 km while "
                "accelerating through 515 m/s, so both limits are exceeded "
                "simultaneously. Run only if t2 shows an 18 km threshold.",
    ),
    # Both limits, on the 80 km reading. Opens with a static prologue at 70 km
    # so acquisition happens below the candidate gate; if it cannot acquire up
    # here at all the run reads NO_LOCK from t=0 and is discarded, rather than
    # masquerading as a gate.
    "t3b_both_80km": dict(
        prologue_s=240.0,
        duration_s=110.0,
        alt0_m=70_000.0,
        v_east=lambda t: ramp(t, 30.0, 110.0, 0.0, 600.0),
        v_up=lambda t: ramp(t, 20.0, 30.0, 0.0, 300.0),
        purpose="Confirmation run for an 80 km gate: 20 s static acquisition at "
                "70 km, then climbs through 80 km while accelerating through "
                "515 m/s. Run only if t2 shows an 80 km threshold.",
    ),
}


def build(name: str, spec: dict) -> dict:
    """Integrate one scenario and return its samples plus ground truth.

    A scenario's `prologue_s` holds it stationary before the profile starts.
    This is not padding: measured time-to-first-fix on the bench receiver, warm
    restarted into a live injection, was about 100 s -- longer than the entire
    t1 ramp. Without a prologue the ramp would run while the receiver was still
    acquiring, and every run would read NO_LOCK for reasons that have nothing to
    do with the gate under test.

    Trajectory time stays absolute, measured from the start of the file, so the
    prologue simply shifts every limit crossing later by `prologue_s` and the
    correlator needs no special case.
    """
    pro = float(spec.get("prologue_s", 0.0))
    total = spec["duration_s"] + pro
    n = int(round(total * RATE_HZ))
    dt = 1.0 / RATE_HZ

    v_east_raw, v_up_raw = spec["v_east"], spec["v_up"]
    spec = dict(spec, duration_s=total,
                v_east=lambda t: 0.0 if t < pro else v_east_raw(t - pro),
                v_up=lambda t: 0.0 if t < pro else v_up_raw(t - pro))

    alt = spec["alt0_m"]
    east_m = 0.0
    rows, truth = [], []

    for i in range(n + 1):
        t = i * dt
        ve = float(spec["v_east"](t))
        vu = float(spec["v_up"](t))
        speed = math.hypot(ve, vu)

        lon = LON0_DEG + east_m * deg_lon_per_metre(LAT0_DEG, alt)
        rows.append(f"{t:.1f},{LAT0_DEG:.9f},{lon:.9f},{alt:.3f}")
        truth.append(dict(t=round(t, 1), alt_m=alt, speed_mps=speed,
                          v_east_mps=ve, v_up_mps=vu))

        # Trapezoid step to the next sample.
        ve_next = float(spec["v_east"](t + dt))
        vu_next = float(spec["v_up"](t + dt))
        east_m += 0.5 * (ve + ve_next) * dt
        alt += 0.5 * (vu + vu_next) * dt

    return dict(name=name, rows=rows, truth=truth,
                purpose=spec["purpose"], duration_s=spec["duration_s"])


def crossings(truth: list[dict]) -> dict:
    """First time each limit is exceeded, and the peak of each quantity."""
    out = {"velocity_515": None, "peak_speed_mps": 0.0, "peak_alt_m": 0.0}
    for cand in ALT_LIMIT_CANDIDATES_M:
        out[f"altitude_{int(cand / 1000)}km"] = None

    for s in truth:
        out["peak_speed_mps"] = max(out["peak_speed_mps"], s["speed_mps"])
        out["peak_alt_m"] = max(out["peak_alt_m"], s["alt_m"])
        if out["velocity_515"] is None and s["speed_mps"] > V_LIMIT_MPS:
            out["velocity_515"] = s["t"]
        for cand in ALT_LIMIT_CANDIDATES_M:
            key = f"altitude_{int(cand / 1000)}km"
            if out[key] is None and s["alt_m"] > cand:
                out[key] = s["t"]
    return out


def isolation(cx: dict) -> str:
    """One word for what a run actually exceeds -- the anti-footgun check."""
    v = cx["velocity_515"] is not None
    a = any(cx[f"altitude_{int(c / 1000)}km"] is not None
            for c in ALT_LIMIT_CANDIDATES_M)
    if v and a:
        return "BOTH"
    if v:
        return "VELOCITY only"
    if a:
        return "ALTITUDE only"
    return "NEITHER"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--outdir", default="scenarios", type=Path,
                    help="output directory (default: ./scenarios)")
    ap.add_argument("--only", action="append", metavar="NAME",
                    help="generate only this scenario (repeatable)")
    ap.add_argument("--sample-rate", type=float, default=2_600_000.0,
                    help="sample rate the .C8 will be built at, for the size "
                         "estimate only (default: 2.6e6)")
    args = ap.parse_args()

    names = args.only or list(SCENARIOS)
    unknown = [n for n in names if n not in SCENARIOS]
    if unknown:
        raise SystemExit(f"unknown scenario(s): {', '.join(unknown)}\n"
                         f"available: {', '.join(SCENARIOS)}")

    args.outdir.mkdir(parents=True, exist_ok=True)

    print(f"{'scenario':<16} {'dur':>6} {'peak alt':>10} {'peak spd':>9} "
          f"{'exceeds':<14} {'.C8 size':>9}")
    print("-" * 72)

    for name in names:
        spec = SCENARIOS[name]
        if spec["duration_s"] > MAX_DURATION_S:
            raise SystemExit(
                f"{name}: {spec['duration_s']:.0f} s exceeds gps-sdr-sim's "
                f"{MAX_DURATION_S:.0f} s dynamic-mode limit. Rebuild it with "
                f"-DUSER_MOTION_SIZE={int(spec['duration_s'] * RATE_HZ)} or "
                f"shorten the run.")

        built = build(name, spec)
        cx = crossings(built["truth"])

        csv_path = args.outdir / f"{name}.csv"
        csv_path.write_text("\n".join(built["rows"]) + "\n")

        # Ground truth for correlate.py: it must not re-derive the trajectory.
        meta = dict(
            scenario=name,
            purpose=built["purpose"],
            duration_s=built["duration_s"],
            rate_hz=RATE_HZ,
            origin=dict(lat_deg=LAT0_DEG, lon_deg=LON0_DEG),
            limits=dict(velocity_mps=V_LIMIT_MPS,
                        altitude_candidates_m=list(ALT_LIMIT_CANDIDATES_M)),
            crossings=cx,
            exceeds=isolation(cx),
            velocity_windows=windows(
                built["truth"], lambda x: x["speed_mps"] > V_LIMIT_MPS),
            altitude_80km_windows=windows(
                built["truth"], lambda x: x["alt_m"] > 80_000.0),
            blocked_windows=windows(
                built["truth"], lambda x: x["speed_mps"] > V_LIMIT_MPS
                or x["alt_m"] > 80_000.0),
            truth=built["truth"],
        )
        (args.outdir / f"{name}.json").write_text(json.dumps(meta, indent=1))

        bytes_c8 = int(built["duration_s"] * args.sample_rate * 2)
        print(f"{name:<16} {built['duration_s']:>5.0f}s "
              f"{cx['peak_alt_m'] / 1000:>8.1f}km {cx['peak_speed_mps']:>7.0f}m/s "
              f"{isolation(cx):<14} {bytes_c8 / 1e9:>7.2f}GB")

    print()
    for name in names:
        cx = crossings(build(name, SCENARIOS[name])["truth"])
        marks = [f"515 m/s at t={cx['velocity_515']:.1f}s"
                 if cx["velocity_515"] is not None else None]
        for cand in ALT_LIMIT_CANDIDATES_M:
            key = f"altitude_{int(cand / 1000)}km"
            if cx[key] is not None:
                marks.append(f"{int(cand / 1000)} km at t={cx[key]:.1f}s")
        marks = [m for m in marks if m]
        print(f"  {name}: " + ("; ".join(marks) if marks else "no limit crossed"))

    print(f"\nwritten to {args.outdir}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
