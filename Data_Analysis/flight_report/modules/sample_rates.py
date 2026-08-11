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
# read off, matching the figure in the detailed report.
#
# Each bar is a two-point horizontal line with a thick stroke, not a bar trace:
# the vendored Plotly build is the gl3d bundle, whose trace set is cone,
# isosurface, mesh3d, scatter, scatter3d, streamtube, surface and volume. There
# is no `bar` module in it, so a `{"type": "bar"}` spec silently renders as a
# scatter — which is what put a row of dots here. The apogee lane chart draws its
# spans the same way for the same reason.
_BAR_COLOR = "#1f77b4"
_BAR_GAPPY = "#d62728"

# Chart geometry. The bar stroke is a pixel width, so it has to be derived from
# how much vertical room a row actually got; hard-coding it overflows the row on
# a short log, where _per_sensor_summary can return one or two sensors.
_ROW_PX = 34
_MARGIN = {"l": 130, "r": 70, "t": 40, "b": 45}
_MIN_HEIGHT = 170
_BAR_FRACTION = 0.62


def _chart(summary: dict[str, dict[str, float]]) -> Optional[dict[str, Any]]:
    from ..charts import chart, trace

    if not summary:
        return None
    # Ascending, so row 0 is the slowest sensor at the bottom and the fastest
    # ends up on top. Sorted by rate rather than by name: the bar lengths then
    # run monotonically and the outliers sit at the ends.
    names = sorted(summary, key=lambda n: summary[n]["rate_hz"])
    rates = [summary[n]["rate_hz"] for n in names]
    gaps = [int(summary[n]["gap_count"]) for n in names]

    height = max(_MIN_HEIGHT, 60 + _ROW_PX * len(names))
    plot_px = height - _MARGIN["t"] - _MARGIN["b"]
    bar_px = round(min(20.0, max(4.0, _BAR_FRACTION * plot_px / len(names))))

    traces, labels = [], []
    for row, (name, rate, gap) in enumerate(zip(names, rates, gaps)):
        # A sensor that dropped samples is colored rather than annotated: the
        # point of the chart is that one bar is not like the others.
        color = _BAR_GAPPY if gap else _BAR_COLOR
        t = trace([0.0, rate], [row, row], name, color, mode="lines", width=bar_px)
        if not t:
            continue
        t["hovertemplate"] = f"{name}: {rate:,.0f} Hz<extra></extra>"
        traces.append(t)
        # The value goes on as an annotation, not the trace's own `text`: the
        # viewport renderer restyles x and y on zoom and never carries text
        # through, so a trace label would vanish on the first interaction.
        labels.append({
            "x": rate, "y": row, "text": f"{rate:,.0f} Hz",
            "showarrow": False, "xanchor": "left", "xshift": 6,
            "font": {"size": 11, "color": "#333"},
        })

    if not traces:
        return None

    spec = chart("chart-sample-rates", "Per-sensor sample rate", traces,
                 x_title="Median rate (Hz)", y_title="", height=height,
                 y_ticks=(list(range(len(names))), names))
    if not spec:
        return None
    layout = spec["layout"]
    # Load-bearing, not belt-and-braces: chart() turns the legend on for more
    # than one trace, and there is now one trace per sensor.
    layout["showlegend"] = False
    layout["margin"] = dict(_MARGIN)
    layout["annotations"] = labels
    layout["xaxis"]["range"] = [0, max(rates) * 1.18]
    layout["yaxis"]["range"] = [-0.6, len(names) - 0.4]
    layout["yaxis"]["showgrid"] = False
    # A two-point line only hovers near its endpoints under "closest", which
    # would leave the middle of every bar dead. Unified hover picks the row.
    layout["hovermode"] = "y unified"

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
