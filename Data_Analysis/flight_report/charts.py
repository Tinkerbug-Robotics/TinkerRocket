"""Interactive chart specs (Plotly) for the HTML report.

A chart is a plain dict — ``{"id", "title", "traces", "layout", "note"}`` — that
`render.py` serializes to JSON and the template hands to `Plotly.newPlot`. Keeping
them as dicts means neither the CLI nor the browser build needs the Plotly Python
package; only the vendored `plotly-gl3d.min.js` is required, and only at render
time.

Charts complement rather than replace the matplotlib figures: these give the
headline series zoom/pan, while the static figures remain the full reference set.

**Every sample is embedded at full resolution — do not decimate here.** Drawing
10^5 SVG points per trace would be slow, so the template's viewport renderer
draws a min/max envelope sized to the screen and re-slices from the full arrays
on every zoom. That keeps rendering cheap without ever hiding data: a min/max
envelope preserves extremes exactly, whereas a stride/nth-sample decimation
would silently drop the ejection transient and peak-G spikes — precisely what a
reader zooms in to find. Zoom far enough and you are looking at raw samples.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional, Sequence

import numpy as np

# Matplotlib's tab10 order, so interactive and static figures read alike.
COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
]

# Time is shipped at the log clock's own resolution. The boards stamp every
# record in integer microseconds, so six decimals is exact and never invents a
# digit. This used to be three: 1 ms was "well below sensor resolution" for the
# barometer and GNSS, but the IMU logs at 3.84 kHz under boost — samples
# 264 µs apart — so four consecutive readings rounded onto one x, and zooming
# into an acceleration chart showed columns of stacked dots where the trace
# should be. Whatever the rounding, it has to stay finer than the fastest
# stream's spacing, and the clock's own resolution is the one bound no sensor
# rate can get under.
TIME_DECIMALS = 6
# The hover reads the same digits. d3-format's `~` trims trailing zeros, so a
# 100 Hz barometer sample still shows "2.75" rather than "2.750000".
X_HOVER_FORMAT = f".{TIME_DECIMALS}~f"

# Chronological, which is the order they are drawn and listed in. Ejection sits
# between burnout and apogee because that is where it usually falls — but not
# always: a rocket can deploy before it stops climbing, and the sample flight
# does, which is exactly why the event is worth marking.
_EVENT_STYLE = {
    "launch": ("#2ca02c", "Launch"),
    "burnout": ("#ff7f0e", "Burnout"),
    "ejection": ("#9467bd", "Ejection"),
    "apogee": ("#d62728", "Apogee"),
    "landed": ("#7f7f7f", "Landed"),
}


def _clean(x: Sequence[float], y: Sequence[float]) -> tuple[list, list]:
    """Drop non-finite pairs; returns full-resolution JSON-safe Python lists.

    NaN/Inf are not valid JSON, so they must go before serialization; Plotly
    renders a gap for a `None`, but dropping is simpler and matches the
    scatter-style plots these mirror.

    Values are rounded: x to six decimals — the log clock's microsecond on the
    time axis nearly every chart has; see TIME_DECIMALS for why anything coarser
    stacks the IMU's samples — and y to 4 decimals, below what any sensor
    resolves. Digits that survive rounding are digits shipped in every report,
    and at ~10^5 points per report they add up to megabytes. No samples are
    dropped — see the module docstring on why decimating here would be wrong.
    """
    xa = np.asarray(x, dtype=float)
    ya = np.asarray(y, dtype=float)
    n = min(xa.size, ya.size)
    xa, ya = xa[:n], ya[:n]

    good = np.isfinite(xa) & np.isfinite(ya)
    xa, ya = xa[good], ya[good]

    return np.round(xa, TIME_DECIMALS).tolist(), np.round(ya, 4).tolist()


def trace(
    x: Sequence[float],
    y: Sequence[float],
    name: str,
    color: Optional[str] = None,
    mode: str = "markers",
    axis: str = "y",
    width: float = 1.5,
    size: float = 4,
) -> Optional[dict[str, Any]]:
    """One Plotly scatter trace. Returns None when there's nothing to draw.

    Markers with no connecting line by default, matching the static figures —
    every plot_* function in plot_flight_data_mini uses ax.scatter, none use
    ax.plot. A line drawn through the samples reads as continuous data and, at
    full-flight zoom, visually swallows the markers entirely; scatter shows where
    the samples actually fall and where they thin out. The viewport renderer
    scales marker size and opacity to how many are on screen.
    """
    xs, ys = _clean(x, y)
    if not xs:
        return None
    spec: dict[str, Any] = {
        "type": "scatter",
        "mode": mode,
        "name": name,
        "x": xs,
        "y": ys,
        "yaxis": axis,
        "hovertemplate": f"{name}: %{{y:.4g}}<br>%{{x:{X_HOVER_FORMAT}}}<extra></extra>",
    }
    if "lines" in mode:
        spec["line"] = {"width": width, "color": color}
    if "markers" in mode:
        spec["marker"] = {"size": size, "color": color}
    return spec


def trajectory_3d(
    chart_id: str,
    title: str,
    east: Sequence[float],
    north: Sequence[float],
    up: Sequence[float],
    name: str = "Trajectory",
    color: Optional[str] = None,
    height: int = 560,
    note: str = "",
) -> Optional[dict[str, Any]]:
    """The flight path as a rotatable 3D line, in meters from the pad.

    Marked `kind: "3d"` so the template plots it directly: the viewport renderer
    is a 1-D windowing scheme over an x axis and means nothing here. Callers are
    expected to pass the flight window rather than the whole log — pad idle is
    thousands of samples stacked at the origin, which is both slow to draw and
    not part of the trajectory.

    A ground-track shadow at z=0 is drawn alongside, because a bare line in space
    is genuinely hard to read: the shadow is what tells you where it drifted.
    """
    e = np.asarray(east, dtype=float)
    n = np.asarray(north, dtype=float)
    u = np.asarray(up, dtype=float)
    size = min(e.size, n.size, u.size)
    e, n, u = e[:size], n[:size], u[:size]

    good = np.isfinite(e) & np.isfinite(n) & np.isfinite(u)
    e, n, u = e[good], n[good], u[good]
    if e.size < 2:
        return None

    ex = [round(float(v), 2) for v in e]
    nx = [round(float(v), 2) for v in n]
    ux = [round(float(v), 2) for v in u]
    floor = [0.0] * len(ex)

    traces = [
        {
            "type": "scatter3d", "mode": "lines", "name": name,
            "x": ex, "y": nx, "z": ux,
            "line": {"width": 4, "color": color or COLORS[0]},
            "hovertemplate": "E %{x:.0f} m · N %{y:.0f} m · up %{z:.0f} m<extra></extra>",
        },
        {
            "type": "scatter3d", "mode": "lines", "name": "Ground track",
            "x": ex, "y": nx, "z": floor,
            "line": {"width": 2, "color": "#bbb"},
            "hoverinfo": "skip",
        },
        {
            "type": "scatter3d", "mode": "markers", "name": "Pad",
            "x": [ex[0]], "y": [nx[0]], "z": [ux[0]],
            "marker": {"size": 5, "color": "#2ca02c"}, "hoverinfo": "skip",
        },
        {
            "type": "scatter3d", "mode": "markers", "name": "Landing",
            "x": [ex[-1]], "y": [nx[-1]], "z": [ux[-1]],
            "marker": {"size": 5, "color": "#d62728"}, "hoverinfo": "skip",
        },
    ]

    axis = {"gridcolor": "#e6e6e6", "zerolinecolor": "#ccc", "showbackground": True,
            "backgroundcolor": "#fcfcfd"}
    return {
        "id": chart_id,
        "title": title,
        "kind": "3d",
        # All three axes are meters, so the metric/imperial toggle rescales the
        # whole scene. The unit lives in the chart title, not on the axes: a 3D
        # scene draws its axis titles rotated at the box corners, and with
        # aspectmode "data" a tall trajectory pushes "East (m)" and "North (m)"
        # off the edges however the margins and domain are set. Bare E/N/U fit.
        "sceneUnit": "m",
        "sceneTitles": ["East", "North", "Up"],
        "titleBase": title,
        "traces": traces,
        "layout": {
            "title": {"text": f"{title} — meters from the pad", "font": {"size": 14}},
            "height": height,
            "margin": {"l": 10, "r": 10, "t": 60, "b": 10},
            "showlegend": True,
            "legend": {"orientation": "h", "y": 1.0, "yanchor": "bottom",
                       "font": {"size": 11}},
            "scene": {
                "xaxis": {**axis, "title": {"text": "East"}},
                "yaxis": {**axis, "title": {"text": "North"}},
                "zaxis": {**axis, "title": {"text": "Up"}},
                # Equal aspect: without it Plotly stretches each axis to the box
                # and a 360 m climb over 200 m of drift looks like a gentle lean.
                "aspectmode": "data",
                # Camera pulled back deliberately. A 3D scene draws its axis
                # titles at the box corners and clips them at the WebGL
                # viewport edge, so with a tall trajectory "East" and
                # "North" get cut off. Margins and scene.domain do not
                # help — they shrink the viewport too. Zooming out is what
                # leaves room inside it.
                "camera": {"eye": {"x": 2.0, "y": -2.0, "z": 1.15}},
            },
            "paper_bgcolor": "#fff",
        },
        "note": note,
    }


def event_shapes(events: dict[str, float]) -> tuple[list[dict], list[dict]]:
    """Vertical marker lines + labels for launch/burnout/apogee/landed.

    Labels alternate between two rows: launch and burnout are often under a
    second apart, and single-row labels collide illegibly at that spacing.
    """
    shapes, annotations = [], []
    ordered = sorted(((t, k) for k, t in events.items() if t is not None))
    for i, (t, key) in enumerate(ordered):
        color, label = _EVENT_STYLE.get(key, ("#888", key))
        shapes.append({
            "type": "line", "xref": "x", "yref": "paper",
            "x0": t, "x1": t, "y0": 0, "y1": 1,
            "line": {"color": color, "width": 1, "dash": "dash"},
        })
        annotations.append({
            "x": t, "y": 1.0 if i % 2 == 0 else 1.07,
            "xref": "x", "yref": "paper",
            "text": label, "showarrow": False,
            "font": {"size": 10, "color": color},
            "yanchor": "bottom",
        })
    return shapes, annotations


def robust_range(traces: list[dict], pad: float = 0.08) -> Optional[list[float]]:
    """A y-range that ignores rare transient spikes, or None if none present.

    Barometric altitude takes a large excursion when the ejection charge fires;
    autoranging to it flattens the entire real flight into a few pixels. The
    range covers the 0.5-99.5 percentile of the data, and is only used when it
    is dramatically tighter than the full extent — otherwise autorange is right
    and this returns None. All samples stay in the chart; the toolbar's Autoscale
    button fits them.
    """
    ys = np.concatenate([np.asarray(t["y"], dtype=float) for t in traces if t.get("y")])
    if ys.size < 20:
        return None
    lo, hi = np.percentile(ys, [0.5, 99.5])
    full_lo, full_hi = float(np.min(ys)), float(np.max(ys))
    span, full_span = hi - lo, full_hi - full_lo
    if span <= 0 or full_span < span * 3:
        return None
    margin = span * pad
    return [float(lo - margin), float(hi + margin)]


def chart(
    chart_id: str,
    title: str,
    traces: Iterable[Optional[dict]],
    x_title: str = "Time (s)",
    y_title: str = "",
    events: Optional[dict[str, float]] = None,
    height: int = 380,
    y2_title: Optional[str] = None,
    equal_aspect: bool = False,
    y_ticks: Optional[tuple[list[float], list[str]]] = None,
    clip_spikes: bool = False,
    y_unit: Optional[str] = None,
    x_unit: Optional[str] = None,
) -> Optional[dict[str, Any]]:
    """Assemble a chart spec. Returns None when every trace was empty."""
    kept = [t for t in traces if t]
    if not kept:
        return None

    layout: dict[str, Any] = {
        "title": {"text": title, "font": {"size": 14}},
        "height": height,
        # Text tick labels (e.g. rocket-state names) need more room than numbers.
        "margin": {
            "l": 90 if y_ticks else 60,
            "r": 60 if y2_title else 20,
            "t": 56 if events else 40,
            "b": 45,
        },
        "xaxis": {"title": {"text": x_title}, "gridcolor": "#eee", "zeroline": False},
        "yaxis": {"title": {"text": y_title}, "gridcolor": "#eee", "zeroline": False},
        "hovermode": "closest",
        "showlegend": len(kept) > 1,
        "legend": {"orientation": "h", "y": -0.18, "font": {"size": 11}},
        "plot_bgcolor": "#fff",
        "paper_bgcolor": "#fff",
    }
    if y2_title:
        layout["yaxis2"] = {
            "title": {"text": y2_title},
            "overlaying": "y", "side": "right",
            "showgrid": False, "zeroline": False,
        }
    if equal_aspect:
        layout["yaxis"]["scaleanchor"] = "x"
        layout["yaxis"]["scaleratio"] = 1
    if y_ticks:
        vals, labels = y_ticks
        layout["yaxis"].update({"tickmode": "array", "tickvals": vals, "ticktext": labels})
    if events:
        shapes, annotations = event_shapes(events)
        if shapes:
            layout["shapes"] = shapes
            layout["annotations"] = annotations

    notes = []
    if clip_spikes and not y_ticks:
        r = robust_range(kept)
        if r:
            layout["yaxis"]["range"] = r
            notes.append("Axis clipped to the bulk of the data; Autoscale fits every sample.")
    # No per-chart sample-count note: the envelope behavior is explained once,
    # in the instructions at the top of the report, and repeating it under every
    # large chart was most of the prose in the document.

    return {
        "id": chart_id,
        "title": title,
        "traces": kept,
        "layout": layout,
        "note": " ".join(notes),
        # Axis units, so the viewer's metric/imperial toggle can rescale the data
        # and relabel the axes without regenerating the report. Titles carry the
        # base text (no unit) for the same reason.
        "yUnit": y_unit,
        "xUnit": x_unit,
        "yTitleBase": y_title,
        "xTitleBase": x_title,
    }
