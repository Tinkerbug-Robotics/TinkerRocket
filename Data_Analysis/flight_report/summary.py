"""Headline flight numbers for the summary card.

Computed from the flight records rather than the `.json` sidecar, so the card is
identical whether or not a sidecar was saved alongside the log. Every value is
SI; formatting (and the eventual imperial toggle) belongs in the template.

Each entry is ``{"label", "value", "unit", "text", "hint"}``. `value` is the raw
number for downstream use, `text` is what the card shows, and `hint` says where
the number came from — which sensor disagreed with which matters when a flyer is
comparing an altitude claim against someone else's altimeter.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array, gnss_to_enu, pressure_to_altitude  # noqa: E402

from markupsafe import Markup

from .events import burnout_gap, ejection_channel, launch_gap, measured, phase_at, span as _span
from .imu import THRUST_HOLD_S, accel_body, sustained_peak
from .units import q

G = 9.80665

_COMPASS = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]


def _cell(label: str, value: Optional[float], unit: str, places: int = 1,
          hint: str = "", suffix: str = "") -> dict[str, Any]:
    """One card cell. `value` is SI; the Quantity renders and converts itself."""
    return {"label": label, "q": q(value, unit, places, suffix), "hint": hint}


def _event_times(flight) -> dict[str, Optional[float]]:
    """Seconds since t0 for each flight event, measured rather than declared.

    See events.py. The card used to read the flags, which meant "time to apogee"
    was the barometric detector's vote minus the launch declaration — two
    different kinds of lateness, one of them 1.15 s.
    """
    return measured(flight)


def _eject_hint(flight) -> str:
    """Where the ejection time came from: a channel that fired, or the trace.

    Worth saying on the card. A pyro time is recorded by the thing that fired;
    a motor ejection is inferred from the transient it leaves in the
    accelerometer, and a reader is entitled to know which they are looking at.

    The channel measured() took, not any channel that fired. A pyro that fired
    only the main, after a drogue on motor ejection, is not where the coast
    ended, and with two channels fired the first to fire is not always the
    lowest-numbered.
    """
    ch = ejection_channel(flight)
    return f"pyro {ch} firing" if ch is not None else "ejection, found in the accelerometer"


def _baro_agl(records) -> Optional[np.ndarray]:
    """Barometric altitude above the pad, using the pad-phase mean as reference."""
    baro = records.get("BMP585") or []
    if not baro:
        return None
    p = get_array(baro, "pressure_pa")
    if not p.size:
        return None
    p_ground = float(np.mean(p[: min(200, p.size)]))
    return np.array([pressure_to_altitude(float(v), p_ground) for v in p])


def _burn_window(flight, events) -> tuple[Optional[tuple[float, float]], str]:
    """The part of the record that is the motor burning, and how to say so.

    The bounds are the measured burn, not the flags, so the window opens when
    the motor lit rather than 0.2 s later when the flight computer agreed.

    When the motor lit inside a hole in the log, the window is the burn from
    the end of the hole on: still thrust, and still clear of the ejection and
    the landing. The hint says so, because the motor may have pushed harder
    before the log came back — 2026-07-05 195028 resumes at 5.2 g and falls
    from there.

    When the motor burned out inside one, the window is the burn up to the
    start of the hole, hinted the same way: the log does not hold the rest of
    the burn. 2026-03-14 224121 breaks off 0.69 s in, still at 8 g. A burn
    with a hole at each end keeps the part between them; one whose both ends
    fell in the same hole has no thrust in the record, and no window.
    """
    gap = launch_gap(flight)
    burnout_hole = burnout_gap(flight)
    if events["launch"] is not None and events["burnout"] is not None:
        return (events["launch"], events["burnout"]), "under thrust"
    if gap is not None and events["burnout"] is not None:
        return (gap[1], events["burnout"]), "under thrust, after the gap"
    if events["launch"] is not None and burnout_hole is not None:
        return (events["launch"], burnout_hole[0]), "under thrust, before the gap"
    if gap is not None and burnout_hole is not None:
        return (gap[1], burnout_hole[0]), "under thrust, between the gaps"
    if events["launch"] is not None and events["apogee"] is not None:
        # Coasting the specific force along the rocket is drag, negative, so
        # the held peak is still the motor's.
        return (events["launch"], events["apogee"]), "launch to apogee (no burnout detected)"
    # With no burn to bound it, any peak would be a guess at which knock was
    # the motor. The peak-shock cell still reports the largest one.
    return None, ""


def peak_shock(flight) -> Optional[dict[str, Any]]:
    """The hardest knock in the flight: {"g", "t", "phase", "hint"}, or None.

    The magnitude, from the part the flight loop would read at each sample,
    held on two samples in a row: a real shock rings for several, and a single
    corrupt sample cannot set it. The Piercer Mini's liftoff transient reads
    283 G on one sample and 229 G held, and that is what this reports.
    """
    recs = flight.records
    imu = recs.get("ISM6HG256") or []
    if not imu or flight.t0_us is None:
        return None
    a, railed, _which = accel_body(recs)
    if a is None or a.shape[1] < 2:
        return None
    t = (get_array(imu, "time_us") - flight.t0_us) / 1e6
    mag = np.sqrt((a ** 2).sum(axis=0)) / G
    held = np.minimum(mag[:-1], mag[1:])
    j = int(np.argmax(held))
    at = float(t[j])
    if railed[j] or railed[j + 1]:
        part = ("high-G accelerometer" if "high_acc_x" in imu[0]
                else "low-G accelerometer at its rail, so it reads low")
    else:
        part = "low-G accelerometer"
    phase = phase_at(flight, at)
    when = f"{phase}, {at:.1f} s" if phase else f"at {at:.1f} s"
    return {"g": float(held[j]), "t": at, "phase": phase, "hint": f"{when} · {part}"}


def compute_summary(flight) -> list[dict[str, Any]]:
    """The headline cells, in display order. Cells with no data are omitted."""
    recs = flight.records
    t0 = flight.t0_us
    if t0 is None:
        return []

    cells: list[dict[str, Any]] = []
    events = _event_times(flight)

    # --- Apogee altitude -----------------------------------------------------
    alt = _baro_agl(recs)
    gnss = recs.get("GNSS") or []
    gnss_apogee = None
    if gnss and "alt_m" in gnss[0]:
        g_alt = get_array(gnss, "alt_m")
        if g_alt.size:
            gnss_apogee = float(np.max(g_alt) - g_alt[0])

    if alt is not None and alt.size:
        baro_apogee = float(np.max(alt))
        hint = "barometric, above the pad"
        if gnss_apogee is not None:
            # A Quantity, not a formatted string: the hint is the one place a
            # number was baked into prose, so it stayed in metres while the value
            # above it switched to feet. Markup + Quantity works because Jinja's
            # escape() honours __html__, so the span survives autoescaping.
            hint = Markup(hint + " · GNSS says ") + q(gnss_apogee, "m", 0)
        cells.append(_cell("Apogee", baro_apogee, "m", 1, hint))
    elif gnss_apogee is not None:
        cells.append(_cell("Apogee", gnss_apogee, "m", 1, "GNSS (no barometer data)"))

    # --- Max speed -----------------------------------------------------------
    ns = recs.get("NonSensor") or []
    speed = None
    hint = ""
    if ns and all(k in ns[0] for k in ("e_vel", "n_vel", "u_vel")):
        v = np.sqrt(get_array(ns, "e_vel") ** 2
                    + get_array(ns, "n_vel") ** 2
                    + get_array(ns, "u_vel") ** 2)
        if v.size:
            speed, hint = float(np.max(v)), "EKF"
    if speed is None and gnss and all(k in gnss[0] for k in ("vel_e", "vel_n", "vel_u")):
        v = np.sqrt(get_array(gnss, "vel_e") ** 2
                    + get_array(gnss, "vel_n") ** 2
                    + get_array(gnss, "vel_u") ** 2)
        if v.size:
            speed, hint = float(np.max(v)), "GNSS"
    if speed is not None:
        cells.append(_cell("Max speed", speed, "m/s", 1, hint))

    # --- Acceleration -------------------------------------------------------
    # Two cells, because a flyer asks two different questions. What did the
    # motor do: the highest acceleration along the rocket that it held for at
    # least THRUST_HOLD_S under thrust. What was the hardest knock the airframe
    # took, and when: the peak shock, anywhere in the flight.
    #
    # They used to be one cell, the largest single sample in the burn, and it
    # answered neither. A liftoff transient rings for a couple of milliseconds
    # well past anything the motor does: on the 10/24 Piercer Mini that cell
    # read 283 G for a motor that held 10.5 G, and on 15 of the 26 archived
    # flights with a measured burn it overstated the motor by more than 2 G.
    imu = recs.get("ISM6HG256") or []
    if imu:
        t = (get_array(imu, "time_us") - t0) / 1e6
        window, hint_window = _burn_window(flight, events)
        if window is not None:
            mask = (t >= window[0]) & (t <= window[1])
            a, _railed, which = accel_body(recs, mask)
            held = sustained_peak(t[mask], a[0]) if a is not None and a.shape[1] else None
            if held is not None:
                how = f"along the rocket, held {THRUST_HOLD_S * 1e3:.0f} ms or more"
                if hint_window != "under thrust":
                    how = f"{hint_window} · {how}"
                cells.append(_cell("Max thrust acceleration", held[0] / G, "G", 1, f"{how} · {which}"))
        shock = peak_shock(flight)
        if shock is not None:
            cells.append(_cell("Peak shock", shock["g"], "G", 0 if shock["g"] >= 20 else 1, shock["hint"]))

    # --- Timing --------------------------------------------------------------
    # Chronological: the burn, then the coast, then the top, then the whole
    # thing. Every one of them is measured off the sensor record and starts from
    # the same instant of first motion, so they add up. When first motion or
    # burnout fell in a hole in the log the ones that count from it are left
    # off, and card_note() says why.
    burn = _span(events, "launch", "burnout")
    if burn is not None:
        cells.append(_cell("Burn time", burn, "s", 2, "first motion to thrust ending"))

    coast = _span(events, "burnout", "ejection")
    if coast is not None:
        cells.append(_cell("Coast time", coast, "s", 2, f"burnout to {_eject_hint(flight)}"))

    to_apogee = _span(events, "launch", "apogee")
    if to_apogee is not None:
        cells.append(_cell("Time to apogee", to_apogee, "s", 2, "first motion to peak altitude"))

    if events["launch"] is not None and events["landed"] is not None:
        cells.append(_cell("Flight time", events["landed"] - events["launch"], "s", 1,
                           "first motion to touchdown"))

    # --- Where it came down --------------------------------------------------
    if gnss:
        e, n, _u = gnss_to_enu(gnss)
        if e.size:
            dist = float(math.hypot(e[-1], n[-1]))
            bearing = math.degrees(math.atan2(float(e[-1]), float(n[-1]))) % 360.0
            point = _COMPASS[int((bearing + 11.25) // 22.5) % 16]
            cells.append(_cell(
                "Landing distance", dist, "m", 0,
                f"{bearing:.0f}° from the pad · last GNSS fix",
                suffix=f" {point}",
            ))

    # --- The pack ------------------------------------------------------------
    # Its lowest reading, which is the number a flyer checks against the
    # regulator's dropout; what the flight drew rides in the hint. Imported
    # here rather than at the top: the module imports the registry, and the
    # registry imports every module, so a top-level import would run before
    # the module had finished defining itself.
    from .modules.power import pack_summary
    pack = pack_summary(flight)
    if pack is not None:
        cells.append(_cell("Pack minimum", pack["v_min"], "V", 2, pack["hint"]))

    # --- Boost vibration -----------------------------------------------------
    # Same lazy import, same reason. The number is the longitudinal RMS above
    # 100 Hz under thrust: the motor's contribution, which is what a flyer
    # comparing two motors or two mounts wants first.
    from .modules.vibration import boost_summary
    vib = boost_summary(flight)
    if vib is not None:
        cells.append(_cell("Boost vibration", vib["rms_hp_x"], "m/s²", 1, vib["hint"]))

    return cells


def card_note(flight) -> str:
    """One quiet line under the card: what it leaves blank, and why.

    First motion or burnout fell in a hole in the log. Every span that counts
    from the lost event is left off rather than measured from a guess, and a
    card with cells silently missing reads as a report that forgot them.
    """
    holes = {"launch": launch_gap(flight), "burnout": burnout_gap(flight)}
    if not any(holes.values()):
        return ""
    events = measured(flight)
    # A cell is named only when a hole is what blanked it: each end it needs was
    # either measured or lost in a hole, and at least one was lost.
    cells = (("burn time", ("launch", "burnout")),
             ("coast time", ("burnout", "ejection")),
             ("time to apogee", ("launch", "apogee")),
             ("flight time", ("launch", "landed")))
    blank = [name for name, ends in cells
             if all(events[e] is not None or holes.get(e) for e in ends)
             and any(holes.get(e) for e in ends)]

    def where(hole: tuple[float, float]) -> str:
        a, b = hole
        return f"a {1e3 * (b - a):.0f} ms gap in the log ({a:.2f}–{b:.2f} s)"

    launch, burnout = holes["launch"], holes["burnout"]
    if launch and burnout:
        line = (f"First motion and burnout fell in {where(launch)}" if launch == burnout
                else f"First motion fell in {where(launch)} and burnout in {where(burnout)}")
    elif launch:
        line = f"First motion fell in {where(launch)}"
    else:
        line = f"Burnout fell in {where(burnout)}"
    if not blank:
        return line + "."
    listed = blank[0] if len(blank) == 1 else ", ".join(blank[:-1]) + " and " + blank[-1]
    return f"{line}, so {listed} {'is' if len(blank) == 1 else 'are'} not shown."
