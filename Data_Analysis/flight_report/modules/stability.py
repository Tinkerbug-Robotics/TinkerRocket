"""Stability — how far the vehicle turned away from where you pointed it.

Tilt is measured against the vehicle's own attitude on the pad rather than
against an absolute vertical, for two reasons. It is what a flyer actually wants
("how far did it turn from the rail?", which already accounts for rail angle),
and it does not depend on the logged quaternion's world-frame convention. That
convention is not obvious from the data: positions are ENU, but the pad
attitude puts the body long axis along world -Z, so the attitude frame's third
axis is down-positive. Anchoring on the pad sidesteps the question entirely.

Attitude comes from the quaternion, never the Euler triple — sitting vertical is
an Euler singularity where roll and yaw trade off freely while the quaternion
stays steady (docs/architecture/flight-computer.md).
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

from ..charts import COLORS, chart, trace
from ..flight import Flight
from ..registry import AnalysisResult
from ..units import q

# Boost tilt beyond this is heavy weathercocking: it costs altitude and carries
# the vehicle downrange, and on a windy day it is the thing to fix.
TILT_WARN_DEG = 30.0
# Pad window used for the reference attitude, in seconds before launch.
PAD_WINDOW_S = (1.5, 0.1)


def _body_axis_world(ns) -> np.ndarray:
    """Unit vector of the vehicle's long axis, per sample, in the attitude frame."""
    w = get_array(ns, "q0")
    x = get_array(ns, "q1")
    y = get_array(ns, "q2")
    z = get_array(ns, "q3")
    v = np.stack([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)], axis=1)
    norms = np.linalg.norm(v, axis=1, keepdims=True)
    return np.divide(v, norms, out=np.zeros_like(v), where=norms > 1e-9)


def _event(ns, t0_us, flag) -> Optional[float]:
    for r in ns:
        if r.get(flag):
            return (r["time_us"] - t0_us) / 1e6
    return None


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="stability", title="Stability")
    recs = flight.records
    t0 = flight.t0_us
    ns = recs.get("NonSensor") or []

    if t0 is None or not ns or "q0" not in ns[0]:
        result.warnings.append("No logged attitude quaternion — cannot assess stability.")
        return result

    t = (get_array(ns, "time_us") - t0) / 1e6
    launch = _event(ns, t0, "launch")
    burnout = _event(ns, t0, "burnout")
    apogee = _event(ns, t0, "alt_apogee")

    if launch is None:
        result.warnings.append("No launch detected — no pad attitude to measure tilt against.")
        return result

    v = _body_axis_world(ns)
    pad = (t > launch - PAD_WINDOW_S[0]) & (t < launch - PAD_WINDOW_S[1])
    if pad.sum() < 10:
        result.warnings.append(
            "Too few samples on the pad before launch to establish a reference attitude."
        )
        return result

    ref = v[pad].mean(axis=0)
    ref /= np.linalg.norm(ref)
    tilt = np.degrees(np.arccos(np.clip(v @ ref, -1.0, 1.0)))

    metrics: dict[str, object] = {}

    # Pad steadiness doubles as a sanity check on the reference itself.
    metrics["Pad attitude scatter"] = q(float(np.std(tilt[pad])), "°", 2)

    if burnout is not None:
        boost = (t >= launch) & (t <= burnout)
        if boost.sum() > 5:
            metrics["Max tilt under thrust"] = q(float(np.max(tilt[boost])), "°", 1)
            metrics["Tilt at burnout"] = q(
                float(np.interp(burnout, t, tilt)), "°", 1,
                suffix=" from the rail",
            )
    if apogee is not None:
        metrics["Tilt at apogee"] = q(float(np.interp(apogee, t, tilt)), "°", 1)

    boost_max = None
    if burnout is not None:
        boost = (t >= launch) & (t <= burnout)
        if boost.sum() > 5:
            boost_max = float(np.max(tilt[boost]))
    if boost_max is not None and boost_max > TILT_WARN_DEG:
        result.warnings.append(
            f"The vehicle turned {boost_max:.0f}° away from the rail while still under "
            "thrust. That much weathercocking costs altitude and carries the flight "
            "downrange — expect it on a windy day with an underpowered motor, and check "
            "stability margin and rail length if it repeats in calm air."
        )

    result.metrics = metrics

    # Chart the powered and coasting phases; once the chute is out the airframe
    # swings freely and the angle stops describing the vehicle's flight.
    end = apogee if apogee is not None else float(t[-1])
    window = (t >= launch - 1.0) & (t <= end + 0.5)
    if window.sum() > 10:
        events = {k: val for k, val in
                  (("launch", launch), ("burnout", burnout), ("apogee", apogee))
                  if val is not None}
        spec = chart(
            "chart-tilt", "Tilt away from the pad attitude",
            [trace(t[window], tilt[window], "Tilt", COLORS[3])],
            y_title="Tilt", y_unit="°", events=events, height=320,
        )
        if spec:
            result.charts.append(spec)

    return result
