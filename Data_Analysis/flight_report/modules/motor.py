"""Motor performance — what the motor actually did, versus what it says on the tube.

Removed in #751 because the section was gated behind two entry fields, liftoff
mass and motor designation, and most flyers have the log and not a scale
reading. Two required-looking inputs stood between dropping a file and reading a
report. #750 asked for the section back without the form.

So nothing here asks for anything. Every row is measured from the log, including
the impulse — quoted per kilogram of liftoff mass, which is what the
accelerometer can actually know. A flyer who wants newton-seconds multiplies by
a number they carry in their head; a flyer who does not still gets the burn.

Two things are kept from the removed implementation, both because they are about
honesty rather than code:

The accelerometer measures specific force, so what it integrates to is thrust
*minus* drag. Gravity does not enter it — an accelerometer in free fall reads
zero — so the integral really is the impulse the motor delivered net of drag,
and not the velocity the rocket gained. During a short boost the drag term is
small, but it is always an underestimate of total impulse and is labelled as one.
Quoting it as total impulse would flatter every motor on the shelf.

And a mass outside 0.01-500 kg is a typo, not a vehicle. The entry field is gone,
but `Flight.metadata` survived the removal and the web worker still forwards it,
so a caller that already knows the mass gets the newton-second rows for free.
Nothing in the shipped UI sends it today, and this module does not ask.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

from .. import events as ev
from ..flight import Flight
from ..imu import accel_magnitude
from ..registry import AnalysisResult
from ..units import q

G = 9.80665

# NAR/TRA impulse classes: (letter, upper bound in N·s). A class runs from the
# previous bound to its own, and each is double the one before — which is what
# makes the mass window for a class a clean factor of two as well.
_CLASSES = [
    ("1/4A", 0.625), ("1/2A", 1.25), ("A", 2.5), ("B", 5.0), ("C", 10.0),
    ("D", 20.0), ("E", 40.0), ("F", 80.0), ("G", 160.0), ("H", 320.0),
    ("I", 640.0), ("J", 1280.0), ("K", 2560.0), ("L", 5120.0), ("M", 10240.0),
    ("N", 20480.0), ("O", 40960.0),
]

# Liftoff masses worth naming a class for. Below this a hobby rocket is a
# streamer-recovery A motor and the impulse-per-kg figure is enormous; above it,
# nothing that flies on a certified motor.
_MASS_LO_KG, _MASS_HI_KG = 0.05, 100.0


def _impulse_class(total_ns: float) -> Optional[str]:
    for letter, upper in _CLASSES:
        if total_ns <= upper:
            return letter
    return None


def _declared_class(designation: str) -> Optional[str]:
    """Leading letter of a motor designation, e.g. 'I200' -> 'I'."""
    import re
    m = re.match(r"\s*(\d/\d|[A-Oa-o])", (designation or "").strip())
    return m.group(1).upper() if m else None


def _class_windows(j_per_kg: float) -> str:
    """Which class this burn was, as a function of liftoff mass.

    The measured quantity is impulse per kilogram, so the class is not knowable
    without a mass — but it is a *function* of one, and the class bounds double,
    so each class owns a mass window twice as wide as the one below it. Saying
    "H between 0.5 and 1.0 kg" hands the reader the answer in the units they
    actually use, without asking them for anything.
    """
    if j_per_kg <= 0:
        return ""
    parts: list[str] = []
    lower_ns = 0.0
    for letter, upper_ns in _CLASSES:
        # class applies for mass in (lower_ns/j, upper_ns/j]
        m_lo, m_hi = lower_ns / j_per_kg, upper_ns / j_per_kg
        lower_ns = upper_ns
        if m_hi < _MASS_LO_KG or m_lo > _MASS_HI_KG:
            continue
        if m_lo < _MASS_LO_KG:
            parts.append(f"{letter} up to {m_hi:,.2f} kg")
        else:
            parts.append(f"{letter} {m_lo:,.2f}-{m_hi:,.2f} kg")
    return " · ".join(parts[:6])


def _mass_kg(metadata) -> Optional[float]:
    """Liftoff mass in kg, if a caller supplied one and it looks sane."""
    try:
        mass = float((metadata or {}).get("mass_kg"))
    except (TypeError, ValueError):
        return None
    # A model rocket under 10 g or over 500 kg is a typo, not a vehicle.
    return mass if 0.01 <= mass <= 500.0 else None


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="motor", title="Motor Performance")
    recs = flight.records
    t0 = flight.t0_us
    ns = recs.get("NonSensor") or []
    imu = recs.get("ISM6HG256") or []

    if t0 is None or not imu:
        result.warnings.append("No accelerometer data — cannot assess the motor.")
        return result

    # Measured events, not the firmware's flags: `launch` is a flag the FC sets
    # on a detector threshold and `burnout` likewise, and both can be late by
    # more than the burn is long on a short motor. events.py derives them from
    # the acceleration trace itself, which is the same signal being integrated
    # here — so the window and the quantity come from one source.
    marks = ev.measured(flight)
    launch, burnout = marks.get("launch"), marks.get("burnout")
    if launch is None or burnout is None or burnout <= launch:
        result.warnings.append(
            "Launch and burnout were not both measurable from the acceleration "
            "trace, so the burn window is unknown and the motor cannot be "
            "characterised."
        )
        return result

    t = (get_array(imu, "time_us") - t0) / 1e6
    boost = (t >= launch) & (t <= burnout)
    if boost.sum() < 5:
        result.warnings.append("Too few accelerometer samples during the burn to measure it.")
        return result

    # Saturation is judged over the burn window only: a landing impact railing
    # the low-G part must not push the *motor* numbers onto the coarser sensor.
    mag, sensor = accel_magnitude(recs, flight.sidecar, boost)
    if mag is None:
        result.warnings.append("No 3-axis accelerometer channel found.")
        return result

    burn_s = burnout - launch
    peak = float(np.max(mag))
    mean = float(np.mean(mag))
    # Specific force integrated over the burn: impulse per unit mass, in N·s/kg.
    # Numerically identical to m/s, and deliberately NOT called that — the rocket
    # did not gain this much speed, because gravity is not in specific force.
    j_per_kg = float(np.trapezoid(mag, t[boost]))

    metrics: dict[str, object] = {
        "Burn time": q(burn_s, "s", 2),
        "Peak acceleration": q(peak / G, "G", 1, suffix=f" · {sensor}"),
        "Average acceleration": q(mean / G, "G", 1),
        # Specific force over gravity IS thrust-to-weight: the mass cancels,
        # because the accelerometer already divided by it.
        "Thrust-to-weight at peak": q(peak / G, "", 1, suffix=" : 1"),
        "Impulse per kg": q(j_per_kg, "N·s/kg", 0, suffix=" (net of drag)"),
    }

    if ns and all(k in ns[0] for k in ("e_vel", "n_vel", "u_vel")):
        tn = (get_array(ns, "time_us") - t0) / 1e6
        speed = np.sqrt(get_array(ns, "e_vel") ** 2
                        + get_array(ns, "n_vel") ** 2
                        + get_array(ns, "u_vel") ** 2)
        metrics["Speed at burnout"] = q(float(np.interp(burnout, tn, speed)), "m/s", 1)

    mass = _mass_kg(flight.metadata)
    if mass is None:
        windows = _class_windows(j_per_kg)
        if windows:
            metrics["Class by liftoff mass"] = windows
        result.note = (
            "Measured from the log alone — nothing here was typed in. The impulse "
            "is per kilogram of liftoff mass because that is what an accelerometer "
            "can know: multiply by your liftoff mass in kg for newton-seconds, "
            "net of drag."
        )
    else:
        # A caller that already knew the mass: same numbers, in the units a
        # motor is sold in.
        impulse = j_per_kg * mass
        metrics["Liftoff mass"] = q(mass, "kg", 3, suffix=" (supplied)")
        metrics["Measured impulse"] = q(impulse, "N·s", 0, suffix=" (net of drag)")
        metrics["Average thrust"] = q(impulse / burn_s, "N", 0)
        measured_class = _impulse_class(impulse)
        if measured_class:
            metrics["Measured class"] = measured_class
        designation = str((flight.metadata or {}).get("motor") or "").strip()
        if designation:
            metrics["Motor"] = designation
            declared = _declared_class(designation)
            if declared and measured_class and declared != measured_class:
                result.warnings.append(
                    f"The motor is marked {designation} (class {declared}) but the "
                    f"flight measured about {impulse:,.0f} N·s, which is class "
                    f"{measured_class}. Some of that gap is drag, which this method "
                    "subtracts from thrust — but a whole class is worth checking "
                    "the supplied mass for."
                )

    result.metrics = metrics
    return result
