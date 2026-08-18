"""The flight computer's own state machine, as it stepped through the flight.

Read directly under the apogee section, because this is where the detectors'
votes end up: the transition out of coast is the master apogee flag latching, and
the step to LANDED is the touchdown declaration the flight time is measured to.
When a detector fires late, this is the trace that shows what the vehicle
believed it was doing at the time.
"""

from __future__ import annotations

import sys
from pathlib import Path

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from ..events import markers
from ..flight import Flight
from ..registry import AnalysisResult
from .overview import _state_chart


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="rocket_state", title="Rocket State")
    recs, t0 = flight.records, flight.t0_us
    if t0 is None:
        result.warnings.append("No timestamped records, so there is no state to plot.")
        return result

    events = {k: round(v, 3) for k, v in markers(flight).items() if v is not None}
    spec = _state_chart(recs, t0, events)
    if spec is None:
        result.warnings.append("The log carries no rocket-state channel.")
        return result

    # Two of the five lanes are always empty: INIT and READY are passed through
    # before logging starts, so they exist in the state enum and never in a log.
    spec["note"] = (
        "The state the flight computer believed it was in. The step out of "
        "PRELAUNCH is the launch declaration and the step to LANDED is the "
        "touchdown one; the markers show where those instants actually were. "
        "INIT and READY are passed through before logging begins, so those lanes "
        "stay empty."
    )
    result.charts = [spec]
    return result
