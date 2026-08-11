"""Motor performance — what the motor actually did, versus what it says on the tube.

Structured so the section degrades rather than disappears. Burn time,
acceleration and thrust-to-weight need nothing but the log. Total impulse needs
the liftoff mass, and the class check needs the motor designation; without those
the section still renders everything else and says plainly what is missing.

A note on the impulse figure. The accelerometer measures specific force, so what
it integrates to is thrust *minus* drag, not thrust. During a short boost that
is close, but it is an underestimate and is labeled as one — quoting it as
total impulse would flatter every motor on the shelf.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

from ..flight import Flight
from ..imu import accel_magnitude
from ..registry import AnalysisResult
from ..units import q

G = 9.80665

# NAR/TRA impulse classes: (letter, upper bound in N·s). A class spans from the
# previous bound to its own, and each is double the one before.
_CLASSES = [
    ("1/4A", 0.625), ("1/2A", 1.25), ("A", 2.5), ("B", 5.0), ("C", 10.0),
    ("D", 20.0), ("E", 40.0), ("F", 80.0), ("G", 160.0), ("H", 320.0),
    ("I", 640.0), ("J", 1280.0), ("K", 2560.0), ("L", 5120.0), ("M", 10240.0),
    ("N", 20480.0), ("O", 40960.0),
]

_DESIGNATION = re.compile(r"^\s*(?:\d+/\d+|[A-Oa-o])\s*$|^\s*([A-Oa-o])\s*(\d+)", re.X)


def _impulse_class(total_ns: float) -> Optional[str]:
    for letter, upper in _CLASSES:
        if total_ns <= upper:
            return letter
    return None


def _declared_class(designation: str) -> Optional[str]:
    """Leading letter of a motor designation, e.g. 'I200' -> 'I'."""
    if not designation:
        return None
    m = re.match(r"\s*(\d/\d|[A-Oa-o])", designation.strip())
    return m.group(1).upper() if m else None


def _event(ns, t0_us, flag) -> Optional[float]:
    for r in ns:
        if r.get(flag):
            return (r["time_us"] - t0_us) / 1e6
    return None


def _mass_kg(metadata) -> Optional[float]:
    """Liftoff mass in kg from the entry form, if it looks sane."""
    raw = (metadata or {}).get("mass_kg")
    try:
        mass = float(raw)
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

    launch = _event(ns, t0, "launch")
    burnout = _event(ns, t0, "burnout")
    if launch is None or burnout is None or burnout <= launch:
        result.warnings.append(
            "Launch and burnout were not both detected, so the burn window is "
            "unknown and motor performance cannot be measured."
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

    metrics: dict[str, object] = {
        "Burn time": q(burn_s, "s", 2),
        "Peak acceleration": q(peak / G, "G", 1, suffix=f" · {sensor}"),
        "Average acceleration": q(mean / G, "G", 1),
        # Specific force over gravity IS thrust-to-weight, no mass needed: the
        # mass cancels. This is the one motor number that is always available.
        "Thrust-to-weight at peak": q(peak / G, "", 1, suffix=" : 1"),
    }

    ve = None
    if ns and all(k in ns[0] for k in ("e_vel", "n_vel", "u_vel")):
        tn = (get_array(ns, "time_us") - t0) / 1e6
        speed = np.sqrt(get_array(ns, "e_vel") ** 2
                        + get_array(ns, "n_vel") ** 2
                        + get_array(ns, "u_vel") ** 2)
        ve = float(np.interp(burnout, tn, speed))
        metrics["Speed at burnout"] = q(ve, "m/s", 1)

    # --- Everything below needs the entry form ------------------------------
    mass = _mass_kg(flight.metadata)
    designation = str((flight.metadata or {}).get("motor") or "").strip()

    if mass is not None:
        # ∫|a| dt over the burn, times mass. Specific force, so this is thrust
        # net of drag — an underestimate, and labeled as one.
        impulse = float(np.trapezoid(mag, t[boost])) * mass
        metrics["Liftoff mass"] = q(mass, "kg", 3, suffix=" (entered)")
        metrics["Measured impulse"] = q(impulse, "N·s", 0, suffix=" (net of drag)")
        metrics["Average thrust"] = q(impulse / burn_s, "N", 0)

        measured_class = _impulse_class(impulse)
        if measured_class:
            metrics["Measured class"] = measured_class
        if designation:
            metrics["Motor"] = designation
            declared = _declared_class(designation)
            if declared and measured_class and declared != measured_class:
                result.warnings.append(
                    f"The motor is marked {designation} (class {declared}) but the "
                    f"flight measured about {impulse:,.0f} N·s, which is class "
                    f"{measured_class}. Some of that gap is drag, which this method "
                    "subtracts from thrust — but a whole class is worth checking "
                    "the entered mass for."
                )
    else:
        missing = ["liftoff mass"]
        if not designation:
            missing.append("motor designation")
        result.warnings.append(
            "Total impulse, average thrust and the motor-class check need the "
            + " and ".join(missing)
            + ". Enter them on the first screen and re-run to get those; "
            "everything above is measured from the log alone."
        )
        if designation:
            metrics["Motor"] = designation

    result.metrics = metrics
    return result
