"""Deployment and recovery — did the chute come out on time, and come down safely?

Three questions, in the order a flyer asks them:

1. Did the flight computer call apogee at the right moment? Measured as the lag
   between the true altitude peak and the `alt_apogee` flag, plus how much
   altitude was already lost by then. A late call is what puts an ejection
   charge into a rocket that is already moving downwards fast.
2. Did the pyro channels do what they were configured to do? An enabled channel
   that never fired, or one that lost continuity, is the failure a flyer needs
   to see. Channels configured off are reported as motor-ejection, not as a
   fault.
3. Did it come down at a survivable rate? Mean descent rate, a drogue-to-main
   transition when there is one, and the touchdown rate.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array, pressure_to_altitude  # noqa: E402

from ..charts import COLORS, chart, trace
from ..flight import Flight
from ..maps import track_map
from ..registry import AnalysisResult
from ..units import Quantity, q

# A descent this fast under a deployed canopy is a partial or failed recovery.
HARD_DESCENT_MPS = 10.0
# Apogee called later than this leaves the vehicle meaningfully nose-down.
APOGEE_LAG_WARN_S = 1.0
# Late-phase rate must drop to this fraction of the early rate before we call it
# a main deployment rather than ordinary drag variation.
MAIN_TRANSITION_RATIO = 0.6


def _events(ns, t0_us) -> dict[str, Optional[float]]:
    out: dict[str, Optional[float]] = {}
    for label, flag in (("launch", "launch"), ("burnout", "burnout"),
                        ("apogee", "alt_apogee"), ("landed", "alt_landed")):
        out[label] = None
        for r in ns:
            if r.get(flag):
                out[label] = (r["time_us"] - t0_us) / 1e6
                break
    return out


def _baro_profile(recs, t0_us):
    """(time_s, altitude_agl_m) from the barometer, or (None, None)."""
    baro = recs.get("BMP585") or []
    if not baro:
        return None, None
    p = get_array(baro, "pressure_pa")
    if not p.size:
        return None, None
    p0 = float(np.mean(p[: min(200, p.size)]))
    alt = np.array([pressure_to_altitude(float(v), p0) for v in p])
    return (get_array(baro, "time_us") - t0_us) / 1e6, alt


def _descent_rate(t, alt, lo, hi) -> Optional[float]:
    """Least-squares vertical rate over [lo, hi] seconds. Negative = descending."""
    m = (t >= lo) & (t <= hi)
    if m.sum() < 10:
        return None
    return float(np.polyfit(t[m], alt[m], 1)[0])


def _pyro_config(sidecar) -> dict[int, bool]:
    """Which channels the flight computer was configured to fire, from the sidecar."""
    cfg = ((sidecar or {}).get("settings") or {}).get("pyro") or {}
    enabled = {}
    for ch in (1, 2, 3, 4):
        entry = cfg.get(f"ch{ch}")
        if isinstance(entry, dict) and "enabled" in entry:
            enabled[ch] = bool(entry["enabled"])
    return enabled


def _pyro_report(ns, t0_us, apogee_s, sidecar, result) -> dict[str, str]:
    """Per-channel fire time and continuity, with warnings for real faults.

    Only an *enabled* channel that failed to fire is a fault. Disabled channels
    mean the vehicle recovers on motor ejection, which is the normal case for
    smaller rockets — warning about it would train people to ignore warnings.
    """
    out: dict[str, str] = {}
    if not ns:
        return out

    enabled = _pyro_config(sidecar)
    fired_any = False
    for ch in (1, 2, 3, 4):
        fired_key, cont_key = f"pyro{ch}_fired", f"pyro{ch}_cont"
        if fired_key not in ns[0]:
            continue

        fire_t = None
        for r in ns:
            if r.get(fired_key):
                fire_t = (r["time_us"] - t0_us) / 1e6
                break
        had_continuity = any(bool(r.get(cont_key)) for r in ns)
        is_on = enabled.get(ch)

        if fire_t is not None:
            fired_any = True
            rel = f" ({fire_t - apogee_s:+.2f} s vs apogee)" if apogee_s is not None else ""
            out[f"Pyro {ch}"] = f"fired at {fire_t:.2f} s{rel}"
            if not had_continuity:
                result.warnings.append(
                    f"Pyro {ch} fired but never reported continuity — check the igniter "
                    "and its wiring; the charge may not have lit."
                )
        elif is_on is False:
            out[f"Pyro {ch}"] = "disabled"
        else:
            state = "continuity present" if had_continuity else "NO continuity"
            out[f"Pyro {ch}"] = f"enabled but did not fire ({state})"
            result.warnings.append(
                f"Pyro {ch} was enabled but never fired"
                + ("." if had_continuity
                   else " and never reported continuity — the igniter was probably "
                        "not connected.")
            )

    if out and not fired_any and enabled and not any(enabled.values()):
        out["Deployment"] = "motor ejection (no pyro channels enabled)"
    return out


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="deployment", title="Deployment & Recovery")
    recs = flight.records
    t0 = flight.t0_us
    if t0 is None:
        result.warnings.append("No timestamped records — nothing to analyze.")
        return result

    ns = recs.get("NonSensor") or []
    ev = _events(ns, t0)
    t, alt = _baro_profile(recs, t0)

    metrics: dict[str, object] = {}

    # --- 1. Was apogee called on time? --------------------------------------
    if t is not None and ev["apogee"] is not None:
        peak_i = int(np.argmax(alt))
        peak_t, peak_alt = float(t[peak_i]), float(alt[peak_i])
        lag = ev["apogee"] - peak_t
        metrics["True apogee"] = q(peak_alt, "m", 1, suffix=f" at {peak_t:.2f} s")

        # How far below the peak the vehicle was when the flag went up. Stated in
        # both directions: still climbing if the call was early, already falling
        # if it was late. (Not "altitude lost" — that would be wrong by sign for
        # an early call, which is the safer and more common tuning.)
        at_flag = float(np.interp(ev["apogee"], t, alt))
        below = peak_alt - at_flag
        when = "early" if lag < 0 else "late"
        still = "still climbing" if lag < 0 else "already descending"
        metrics["Apogee call"] = f"{abs(lag):.2f} s {when}"
        metrics["Altitude below peak when called"] = q(below, "m", 1, suffix=f", {still}")

        if lag > APOGEE_LAG_WARN_S:
            result.warnings.append(
                f"Apogee was detected {lag:.2f} s after the true peak, by which point "
                f"the rocket had descended {peak_alt - at_flag:.1f} m. Anything ejecting "
                "on that signal fires nose-down and faster than intended."
            )
        elif lag < -APOGEE_LAG_WARN_S:
            result.warnings.append(
                f"Apogee was declared {-lag:.2f} s before the true peak, {peak_alt - at_flag:.1f} m "
                "below it — an early ejection happens while the vehicle is still climbing."
            )

    # --- 2. What did the pyro channels do? ----------------------------------
    metrics.update(_pyro_report(ns, t0, ev["apogee"], flight.sidecar, result))

    # --- 3. How did it come down? -------------------------------------------
    if t is not None and ev["apogee"] is not None and ev["landed"] is not None:
        ap, land = ev["apogee"], ev["landed"]
        span = land - ap
        metrics["Descent time"] = q(span, "s", 1)

        if span > 4:
            # Skip the first second: the ejection transient is not a descent rate.
            overall = _descent_rate(t, alt, ap + 1.0, land - 0.5)
            early = _descent_rate(t, alt, ap + 1.0, ap + span / 2)
            late = _descent_rate(t, alt, ap + span / 2, land - 0.5)

            if overall is not None:
                metrics["Mean descent rate"] = q(abs(overall), "m/s", 2, conv="m/s:fps")

            two_stage = (
                early is not None and late is not None
                and early < 0 and late < 0
                and abs(late) < abs(early) * MAIN_TRANSITION_RATIO
            )
            if two_stage:
                mid = ap + span / 2
                metrics["Recovery"] = "two-stage (drogue, then main)"
                metrics["Under drogue"] = q(abs(early), "m/s", 2, conv="m/s:fps")
                metrics["Under main"] = q(abs(late), "m/s", 2, conv="m/s:fps")
                metrics["Main deployed near"] = q(float(np.interp(mid, t, alt)), "m", 0)
            else:
                metrics["Recovery"] = "single-stage"

            touchdown = _descent_rate(t, alt, max(ap, land - 2.0), land)
            if touchdown is not None:
                metrics["Touchdown rate"] = q(abs(touchdown), "m/s", 2, conv="m/s:fps")

            final = abs(late if two_stage else (overall or 0.0))
            if final > HARD_DESCENT_MPS:
                result.warnings.append(
                    f"Final descent rate was {final:.1f} m/s. Above about "
                    f"{HARD_DESCENT_MPS:.0f} m/s a landing is hard enough to damage "
                    "airframes and electronics — check for a tangled or undersized canopy."
                )

    result.metrics = metrics

    # --- Descent profile chart ----------------------------------------------
    if t is not None and ev["apogee"] is not None and ev["landed"] is not None:
        m = (t >= ev["apogee"] - 1.0) & (t <= ev["landed"] + 1.0)
        if m.sum() > 10:
            spec = chart(
                "chart-descent", "Descent profile",
                [trace(t[m], alt[m], "Barometric altitude", COLORS[0])],
                y_title="Altitude AGL", y_unit="m",
                events={k: v for k, v in ev.items() if k in ("apogee", "landed")},
                clip_spikes=True,  # the ejection charge throws the barometer hard
            )
            if spec:
                result.charts.append(spec)

    # --- Where to go looking for it -----------------------------------------
    gnss = recs.get("GNSS") or []
    if gnss and all(k in gnss[0] for k in ("lat", "lon")):
        spec = track_map(
            "map-track", "Ground track",
            get_array(gnss, "lat"), get_array(gnss, "lon"),
            note="Green marks the pad, red the last GNSS fix before touchdown.",
            alt=get_array(gnss, "alt_m") if "alt_m" in gnss[0] else None,
            # flight.name already carries the rocket name.
            name=flight.name,
        )
        if spec:
            result.maps.append(spec)

    if not metrics:
        result.warnings.append(
            "Not enough data to assess deployment (needs barometer plus apogee "
            "and landing detection)."
        )
    return result
