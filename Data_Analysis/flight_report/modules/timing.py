"""How evenly each sensor actually sampled.

The sample-rate chart earlier in the report gives one number per sensor, which
is a median and hides its own spread. This is the distribution behind it: a
sensor keeping time cleanly shows a single narrow spike, and one being starved
by a busy bus or a slow write shows a tail, or a second mode, that no median
would reveal.

It sits after the message counts because it answers the next question those
raise — the counts say how many frames arrived, this says whether they arrived
when they should have.
"""

from __future__ import annotations

import sys
from pathlib import Path

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from analyze_gaps import extract_per_sensor_timestamps  # noqa: E402

from ..flight import Flight
from ..registry import AnalysisResult
from .gaps import _plot_per_sensor_dt


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="timing", title="Per-Sensor Sample Timing")

    per_sensor = extract_per_sensor_timestamps(flight.records)
    if not per_sensor:
        result.warnings.append("No per-sensor timestamps, so timing cannot be measured.")
        return result

    fig = _plot_per_sensor_dt(per_sensor)
    if fig is None:
        result.warnings.append("Not enough samples to show a timing distribution.")
        return result

    result.figures = [fig]
    return result
