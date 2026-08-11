"""How fast each sensor actually logged.

A rate well below what a sensor is configured for means samples were dropped,
and everything downstream — apogee timing, the filter, the peak acceleration
figure — is reading a thinner record than it appears to. Worth a glance on any
flight, which is why it is here and not only in the detailed report.

The rates come from `gaps._per_sensor_summary`, the same function the detailed
report's table and figure use, so the two cannot disagree about what a sensor
managed.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from analyze_gaps import extract_per_sensor_timestamps  # noqa: E402

from ..flight import Flight
from ..registry import AnalysisResult
from .gaps import _per_sensor_summary

# Bars, so the comparison between sensors is a length rather than a number to be
# read off. charts.trace() only builds scatter traces, so this one is assembled
# directly — chart() itself is trace-type agnostic.
_BAR_COLOR = "#1f77b4"
_BAR_GAPPY = "#d62728"


def _chart(summary: dict[str, dict[str, float]]) -> Optional[dict[str, Any]]:
    from ..charts import chart

    if not summary:
        return None
    names = sorted(summary, key=lambda n: summary[n]["rate_hz"])
    rates = [summary[n]["rate_hz"] for n in names]
    gaps = [int(summary[n]["gap_count"]) for n in names]

    # A sensor that dropped samples is colored rather than annotated: the point
    # of the chart is that one bar is not like the others.
    colors = [_BAR_GAPPY if g else _BAR_COLOR for g in gaps]
    bar = {
        "type": "bar",
        "orientation": "h",
        "name": "Median rate",
        "x": rates,
        "y": names,
        "marker": {"color": colors},
        "text": [f"{r:,.0f} Hz" for r in rates],
        "textposition": "outside",
        "cliponaxis": False,
        "hovertemplate": "%{y}: %{x:,.0f} Hz<extra></extra>",
    }

    spec = chart("chart-sample-rates", "Per-sensor sample rate", [bar],
                 x_title="Median rate (Hz)", y_title="",
                 height=60 + 34 * len(names))
    if not spec:
        return None
    layout = spec["layout"]
    layout["showlegend"] = False
    layout["margin"] = {"l": 130, "r": 70, "t": 40, "b": 45}
    layout["xaxis"]["range"] = [0, max(rates) * 1.18]
    layout["yaxis"]["gridcolor"] = "#fff"

    dropped = [n for n, g in zip(names, gaps) if g]
    note = "Median logged rate for each sensor, measured from the timestamps themselves."
    if dropped:
        note += (" Red marks a sensor with a gap in its record: "
                 + ", ".join(f"{n} ({summary[n]['gap_count']})" for n in dropped) + ".")
    spec["note"] = note
    return spec


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="sample_rates", title="Sensor Sample Rates")

    per_sensor = extract_per_sensor_timestamps(flight.records)
    if not per_sensor:
        result.warnings.append("No per-sensor timestamps, so sample rates cannot be measured.")
        return result

    summary, _ = _per_sensor_summary(per_sensor)
    spec = _chart(summary)
    if spec is None:
        result.warnings.append("Not enough samples to measure a rate for any sensor.")
        return result
    result.charts = [spec]
    return result
