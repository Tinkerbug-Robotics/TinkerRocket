"""Roll, as a flyer reads it.

Two shapes, depending on what the vehicle actually flew:

* **A roll-angle control flight** gets the four-panel control story — commanded
  angle against achieved, the tracking error, the rate the outer loop asked for
  against the rate the gyro saw, and the fin command that produced it — over the
  window the controller had authority, launch to ejection. After ejection the
  rocket is under canopy and the fins are doing nothing, so including it would
  only compress the part that matters.
* **Anything else** gets a single roll-rate trace, because "how much did it
  spin" is still worth knowing on a flight with no roll control at all.

Which of the two is not read off the sidecar. A sidecar can claim
`angle_profile` for a flight that flew pure rate-null — app exports before
2026-07-05 labeled any flight with a *stored* profile that way, even with
`use_angle_control` off — so `roll_series` cross-checks the claim against the
servo command with an R² fit and can overrule it. This module asks that shared
function rather than deciding for itself, so the flight and detailed reports
can never disagree about what the flight was.

The detailed report keeps the same four panels as static figures, plus the
plant-gain estimate, the FFT and the tuning recommendation. This is the readable
subset, made interactive.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

from ..charts import COLORS, chart, trace
from ..events import markers
from ..flight import Flight
from ..registry import AnalysisResult
from .roll_pid import roll_series

# Matching the detailed report's figures, so the two reports read as the same picture.
C_TARGET = "#ff7f0e"    # commanded
C_ACTUAL = "#2ca02c"    # achieved
C_ERROR = "#d62728"
C_RATECMD = "#1f77b4"


def _null_rate_segments(tw: np.ndarray, is_ang: np.ndarray) -> list[tuple[float, float]]:
    """Contiguous spans where NO angle was being commanded.

    The inverse of the angle segments, matching `roll_pid._shade_segments`: the
    detailed report's figures tint the *rate-null* stretches, leaving the angle-tracking
    ones on white, because those are the parts a reader is being asked to judge.
    Shading the other way round reads as "this is the interesting bit" pointed at
    exactly the wrong half.
    """
    if tw is None or is_ang is None or not len(tw):
        return []
    spans, start = [], None
    for i in range(len(tw)):
        null = not bool(is_ang[i])
        if null and start is None:
            start = float(tw[i])
        elif not null and start is not None:
            spans.append((start, float(tw[i])))
            start = None
    if start is not None:
        spans.append((start, float(tw[-1])))
    return [(a, b) for a, b in spans if b > a]


def _shade(spec: dict, spans: list[tuple[float, float]]) -> None:
    """Tint the rate-null spans, the way the detailed report's figures do.

    `yref: "paper"` rather than data coordinates on purpose: the metric/imperial
    toggle rescales trace data and the y-axis range but leaves shape coordinates
    literal, so a band anchored in data units would sit at the wrong height in
    imperial. Paper coordinates are fractions of the plot, so they are immune.

    Extends rather than assigns — chart() puts the event marker lines in the same
    list, and overwriting it would delete them.
    """
    if not spec or not spans:
        return
    spec["layout"].setdefault("shapes", []).extend({
        "type": "rect", "xref": "x", "yref": "paper",
        "x0": a, "x1": b, "y0": 0, "y1": 1,
        "fillcolor": "#d9d9d9", "opacity": 0.55,
        "line": {"width": 0}, "layer": "below",
    } for a, b in spans)


def _hline(spec: dict, y: float, color: str, dash: str = "dot") -> None:
    """A horizontal reference line in data coordinates.

    Safe against the units toggle only because every axis this is used on is in
    degrees or degrees per second, neither of which the toggle converts. Do not
    reuse this on a meters or m/s axis without anchoring it in paper space.
    """
    if not spec:
        return
    spec["layout"].setdefault("shapes", []).append({
        "type": "line", "xref": "paper", "yref": "y",
        "x0": 0, "x1": 1, "y0": y, "y1": y,
        "line": {"color": color, "width": 1, "dash": dash},
        # Under the traces. Plotly draws shapes above data by default, so a
        # reference line otherwise paints over the very curve it measures.
        "layer": "below",
    })


def _segmented(x, y, name, color, width) -> list[dict[str, Any]]:
    """One trace per contiguous run of finite y, so gaps stay gaps.

    `roll_series` marks two kinds of gap with NaN: the ±180 seam where a wrapped
    angle jumps, and the stretches where no angle was commanded so there is no
    target or error. The shared trace builder drops non-finite pairs rather than
    passing them through, which is harmless for a scatter but joins the two sides
    of a gap with a straight line once the series is drawn as one — turning the
    seam into a vertical stripe across the whole plot and inventing a target
    where none was commanded.
    """
    xa = np.asarray(x, dtype=float)
    ya = np.asarray(y, dtype=float)
    n = min(xa.size, ya.size)
    xa, ya = xa[:n], ya[:n]
    good = np.isfinite(xa) & np.isfinite(ya)
    if not good.any():
        return []

    out: list[dict[str, Any]] = []
    start = None
    for i in range(n + 1):
        inside = i < n and good[i]
        if inside and start is None:
            start = i
        elif not inside and start is not None:
            if i - start > 1:       # a single point draws nothing as a line
                t = trace(xa[start:i], ya[start:i], name, color,
                          mode="lines+markers", width=width)
                if t:
                    if out:
                        t["showlegend"] = False
                    out.append(t)
            start = None
    return out


def _rate_limit(t, g, eject_t, cap) -> tuple[float, float]:
    """Y-limit for the roll-rate panel, and the true peak it may be hiding.

    Ported from `roll_pid._clip_rate_axis` so both reports scale that axis the
    same way: the larger of a robust percentile of the in-flight magnitudes and a
    multiple of the rate cap, with the last second before ejection excluded so a
    tumble that starts early cannot inflate it.
    """
    g = np.asarray(g, dtype=float)
    t = np.asarray(t, dtype=float)
    n = min(g.size, t.size)
    # Mask both together. Dropping non-finite samples from g first and then
    # slicing t to the survivors' *count* would pair each reading with the
    # wrong timestamp, so the window below would select the wrong samples.
    fin = np.isfinite(g[:n])
    g, t = g[:n][fin], t[:n][fin]
    if not g.size:
        return 30.0, 0.0
    core = g
    if eject_t is not None and np.isfinite(eject_t):
        sel = t <= float(eject_t) - 1.0
        if sel.any():
            core = g[sel]
    lim = max(float(np.percentile(np.abs(core), 95)) * 1.4,
              (float(cap) * 2.5) if cap else 0.0, 30.0)
    return lim, float(np.max(np.abs(g)))


def _events_since_launch(flight, lo: float, hi: float) -> dict[str, float]:
    """The measured events, on the launch-relative axis these charts use.

    Filtered to the plotted window, as `roll_pid._event_list` is. Ejection is
    often after the window ends — the window stops just before the charge fires —
    and an event marker outside the range is not merely invisible: `chart()` gives
    it a data-referenced shape, which drags every panel's autorange out to reach
    it and leaves a second of dead space on the right of all four.

    Two things used to be wrong here. The times came from the roll-PID replay's
    own flag-derived values rather than the measured ones, so this section marked
    a burnout 71 ms away from the burnout line on every other chart in the report.
    And the ejection was drawn under the key "apogee" — the charge, labelled as
    the top of the flight, which on the sample flight is a second later and in
    the other direction.
    """
    ev = markers(flight)
    launch = ev.get("launch")
    if launch is None:
        return {}
    out: dict[str, float] = {}
    for key in ("burnout", "ejection", "apogee"):
        value = ev.get(key)
        if value is None or not np.isfinite(value):
            continue
        rel = float(value) - launch
        if lo <= rel <= hi:
            out[key] = round(rel, 3)
    return out


def _control_charts(flight, s: dict, facts: dict[str, Any]) -> list[dict[str, Any]]:
    """The four-panel roll-control story, launch to ejection.

    Drawn as four charts of equal height with the time axis labeled only on the
    last, so they stack and read as one figure rather than four unrelated plots.
    `chart()` cannot make subplots, so this is as close to a shared x axis as the
    spec allows — the ranges match because every panel covers the same window.

    Lines *and* markers. The detailed report's figure draws these with `ax.plot`, and
    the shape between samples is what a control signal is read for — but the
    individual samples matter too, so both are drawn. The template scales marker
    size and opacity to how many are on screen, so they stay out of the way at
    full-window zoom and resolve into points as you zoom in.
    """
    tw = s["track_tw"]
    win_end = s["win_end"]
    # One window, shared by all four panels. matplotlib gets this from sharex
    # plus its 5% margins; here it has to be set explicitly, and it is what makes
    # the four read as one stacked figure instead of four charts that happen to
    # be adjacent.
    x_lo, x_hi = -0.2, win_end + 0.2
    x_pad = 0.05 * (x_hi - x_lo)
    x_range = [x_lo - x_pad, x_hi + x_pad]

    events = _events_since_launch(flight, x_lo, x_hi)
    spans = _null_rate_segments(tw, s.get("track_is_ang"))
    specs: list[Optional[dict[str, Any]]] = []
    H = 260                      # one panel height, uniform across the stack
    NO_X = ""                    # x title only on the bottom panel

    # 1 — commanded angle against achieved. Both wrapped to ±180 so the vertical
    #     gap between them is the error the controller actually acts on; the
    #     target is absent wherever no angle was commanded.
    angle = chart(
        "chart-roll-angle", "Roll angle tracking vs profile plan (launch → ejection)",
        _segmented(tw, s["track_tgt"], "Profile Target", C_TARGET, 2.4)
        + _segmented(tw, s["track_act"], "EKF Roll", C_ACTUAL, 1.6),
        x_title=NO_X, y_title="Roll angle", y_unit="°",
        events=events, height=H,
        # Fixed to a full turn with ticks on the quarters, as the detailed
        # figure has it: autoscaling a wrapped angle hides how close a trace is
        # to the ±180 seam, which is exactly where tracking looks worst.
        y_ticks=([-180, -90, 0, 90, 180], ["−180", "−90", "0", "90", "180"]),
    )
    if angle:
        angle["layout"]["yaxis"]["range"] = [-189, 189]
        _shade(angle, spans)
    specs.append(angle)

    # 2 — the error itself, which is what the RMS and peak figures quote.
    err_spec = chart(
        "chart-roll-error", "Roll tracking error",
        _segmented(tw, s["track_err"], "Tracking Error", C_ERROR, 1.4),
        x_title=NO_X, y_title="Tracking error", y_unit="°",
        events=events, height=H,
    )
    if err_spec:
        # Framed to include zero with a 5% margin, as matplotlib does once its
        # zero rule is in the data limits. Without it the error curve grazes the
        # top of the panel and the zero line sits on the axis, so "the error
        # never came back to zero" — the point of this panel on some flights —
        # reads as the frame rather than as data.
        e = np.asarray(s["track_err"], dtype=float)
        e = e[np.isfinite(e)]
        if e.size:
            lo, hi = min(0.0, float(e.min())), max(0.0, float(e.max()))
            pad = 0.05 * (hi - lo) or 1.0
            err_spec["layout"]["yaxis"]["range"] = [lo - pad, hi + pad]
        _hline(err_spec, 0.0, "#999", dash="solid")
        _shade(err_spec, spans)
    specs.append(err_spec)

    # 3 — what the outer loop asked the vehicle to do against what it did. The
    #     rate cap is drawn because a setpoint beyond it is unreachable, and the
    #     error above then measures the plan rather than the controller.
    t_imu, g = s["t_imu"], s["g"]
    win = (t_imu >= -0.2) & (t_imu <= win_end + 0.2)
    rate_traces = [trace(t_imu[win], g[win], "Gyro X", C_ACTUAL,
                         mode="lines+markers", width=1.2)]
    if s.get("track_ratecmd") is not None:
        rate_traces.append(trace(tw, s["track_ratecmd"], "Commanded Rate",
                                 C_RATECMD, mode="lines+markers", width=2.0))
    rate_spec = chart(
        "chart-roll-rate", "Roll rate — commanded vs achieved", rate_traces,
        x_title=NO_X, y_title="Roll rate", y_unit="°/s",
        events=events, height=H,
    )
    cap = s.get("rate_cap_cfg")
    if rate_spec:
        # Scale the axis the way the detailed report's figure does, rather than with the
        # generic spike clipper: anchor on the rate cap, which is the natural size
        # of controlled roll, and exclude the last second before ejection so a
        # tumble starting early cannot inflate it. Without this the ~620°/s
        # off-the-rail spike flattens the entire controlled portion into a line.
        # win_end, not eject_t — the detailed report's helper is passed the end of the
        # plotting window, and using apogee instead would admit a second of
        # tumble into the percentile and quietly widen the axis.
        lim, peak = _rate_limit(t_imu[win], g[win], win_end, cap)
        rate_spec["layout"]["yaxis"]["range"] = [-lim, lim]
        _hline(rate_spec, 0.0, "#999", dash="solid")
        if cap:
            # Both rails, so a commanded rate sitting on one is visibly on it
            # rather than merely high.
            _hline(rate_spec, float(cap), C_RATECMD)
            _hline(rate_spec, -float(cap), C_RATECMD)
        facts["rate_clipped"] = peak if peak > lim * 1.05 else None
        _shade(rate_spec, spans)
    specs.append(rate_spec)

    # 4 — the fin command that produced all of the above.
    t_ns, cmd = s["t_ns"], s["cmd"]
    win_c = (t_ns >= -0.2) & (t_ns <= win_end + 0.2)
    cmd_spec = chart(
        "chart-roll-cmd", "Roll command to the fins",
        [trace(t_ns[win_c], cmd[win_c], "Fin Command", C_TARGET, mode="lines+markers", width=1.4)],
        x_title="Time since launch (s)", y_title="Roll command", y_unit="°",
        events=events, height=H,
    )
    if cmd_spec:
        _hline(cmd_spec, 0.0, "#999", dash="solid")
        limit = s["rc_cfg"].get("cmd_limit_max_deg") if s.get("rc_cfg") else None
        try:
            limit = float(limit) if limit is not None else None
        except (TypeError, ValueError):
            limit = None
        # Zoomed to the command's own range plus 5°, as the detailed report's panel is.
        # The fin command lives in a few degrees while its limit is ±20; scaling
        # to the limit would flatten every detail of what the controller did.
        cw = cmd[win_c]
        cw = cw[np.isfinite(cw)]
        if cw.size:
            lo, hi = float(np.min(cw)), float(np.max(cw))
            cmd_spec["layout"]["yaxis"]["range"] = [lo - 5.0, hi + 5.0]
            if limit:
                # Only the rail that is actually inside the frame. The axis is
                # zoomed to the command's own range, so on a flight that saturates
                # one way the other rail falls outside it — and a note promising
                # "the dotted lines are the limit" beside a single visible line
                # is worse than not drawing it.
                drawn = 0
                for rail in (limit, -limit):
                    if lo - 5.0 <= rail <= hi + 5.0:
                        _hline(cmd_spec, rail, "#999")
                        drawn += 1
                if drawn:
                    facts["fin_limit"] = limit
        _shade(cmd_spec, spans)
    specs.append(cmd_spec)

    kept = [sp for sp in specs if sp]

    # Make the four read as one figure rather than four charts.
    #
    # Shared window and a 1 s tick grid with 0.5 s minors, so a transition time
    # can be read off the bottom panel and traced straight up. Only that bottom
    # panel draws tick labels and a title for the axis — the same thing the
    # detailed report's figure does with `sharex` plus
    # `setp(ax.get_xticklabels(), visible=False)`.
    #
    # Only the first panel carries event *labels*; the rest keep the marker lines
    # but drop the annotations, which is both what the detailed report's figure does
    # (`_mark_events(ax)` without `annotate`) and what lets the top margin
    # collapse. Between them, the label suppression and the margin trim are what
    # actually close the gaps — CSS alone would leave four boxes of white space.
    for i, sp in enumerate(kept):
        first, last = i == 0, i == len(kept) - 1
        ax = sp["layout"]["xaxis"]
        ax["range"] = list(x_range)
        ax.update({
            "tick0": 0, "dtick": 1,
            "minor": {"dtick": 0.5, "showgrid": True, "gridcolor": "#f4f4f4"},
            "showticklabels": last,
        })
        if not first:
            sp["layout"]["annotations"] = []
        sp["layout"]["margin"] = {
            "l": 70, "r": 20,
            "t": 40 if first else 8,
            "b": 42 if last else 6,
        }
        if not first:
            sp["layout"]["title"] = {"text": "", "font": {"size": 14}}
        # Legend inside the axes, as the detailed report's figure has it. The report's
        # default is a horizontal strip below the plot, which in a stack is a band
        # of text sitting between two panels — the exact thing the stack is meant
        # not to have. Top-left on the first panel, top-right on the rest, so it
        # keeps clear of the traces that matter in each.
        if sp["layout"].get("showlegend"):
            sp["layout"]["legend"] = {
                "x": 0.008 if first else 0.992,
                "xanchor": "left" if first else "right",
                "y": 0.98, "yanchor": "top",
                "bgcolor": "rgba(255,255,255,0.72)",
                "borderwidth": 0, "font": {"size": 10},
            }
        sp["stack"] = "first" if first else ("last" if last else "mid")
        # Panels stop being self-describing once their titles are gone, so the
        # y-axis label has to carry the identification on its own.
        if not first:
            sp["layout"]["height"] = H - 40

    return [sp for sp in specs if sp]


def _rate_only_chart(flight: Flight) -> Optional[dict[str, Any]]:
    """Roll rate alone, for a flight that was not under roll control.

    On the same time base as the altitude, velocity and acceleration charts
    above it, rather than the launch-relative one the control charts use, so it
    lines up with them.
    """
    recs, t0 = flight.records, flight.t0_us
    imu = recs.get("ISM6HG256") or []
    if not imu or t0 is None or "gyro_x" not in imu[0]:
        return None

    t = (get_array(imu, "time_us") - t0) / 1e6
    g = get_array(imu, "gyro_x")

    # Measured, not the flags. The roll-control branch reads the same source
    # (see _events_since_launch), so the two branches of this module no longer
    # mark different instants depending on which one ran.
    events = {k: round(v, 3) for k, v in markers(flight).items() if v is not None}

    start, end = events.get("launch"), events.get("landed") or events.get("apogee")
    window = (t >= start) & (t <= end) if start is not None and end is not None \
        else np.ones(t.shape, dtype=bool)
    if window.sum() < 10:
        return None

    spec = chart(
        "chart-roll-rate", "Roll rate", [trace(t[window], g[window], "Gyro X", C_ACTUAL)],
        y_title="Roll rate", y_unit="°/s", events=events, height=340, clip_spikes=True,
    )
    if spec:
        spec["note"] = "Rotation about the long axis. This flight had no roll control."
    return spec


def _stack_note(s: dict, facts: dict) -> str:
    """The one piece of prose for the whole stack, placed under the last panel."""
    parts = [
        "Launch to ejection, four panels on one time axis. Gray spans are rate-null — "
        "no angle commanded; the unshaded stretches are the angle tracking."
    ]
    # The dotted rails have no other explanation now that this section carries no
    # metrics table, and an unlabelled reference line is worse than none.
    cap_v = s.get("rate_cap_cfg")
    lim_v = facts.get("fin_limit")
    if cap_v and lim_v:
        parts.append(f"Dotted lines mark the controller's ±{cap_v:g}°/s rate cap and the "
                     f"±{lim_v:g}° commanded fin limit.")
    elif cap_v:
        parts.append(f"The dotted lines mark the controller's ±{cap_v:g}°/s rate cap.")
    clipped = facts.get("rate_clipped")
    if clipped:
        parts.append(f"Roll rate peaked at {clipped:.0f}°/s off the rail, above the "
                     "clipped axis.")
    ramp, cap = s.get("profile_ramp"), s.get("rate_cap_cfg")
    if ramp and cap and ramp > cap + 1e-6:
        parts.append(f"The profile ramps at {ramp:.0f}°/s against a {cap:g}°/s cap, so the "
                     "setpoint was unreachable — the error measures the plan, not the "
                     "controller.")
    return " ".join(parts)


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="roll", title="Roll")

    s = roll_series(flight)
    controlled = (
        "abort" not in s
        and s.get("has_angle_profile")
        and s.get("track_tw") is not None
        and s["track_tw"].size > 5
    )

    if controlled:
        result.title = "Roll Control"
        facts: dict[str, Any] = {}
        result.charts = _control_charts(flight, s, facts)
        if not result.charts:
            result.warnings.append("Roll control ran, but produced no plottable series.")
            return result

        # One note, under the whole group, so nothing lands between the panels.
        # No metrics table: the flight report's roll section is a picture, and
        # the numbers behind it (plant gain, phase margin, suggested gains, the
        # steady-state spectrum) are tuning work that belongs in the detailed
        # report, where they already are.
        result.charts[-1]["note"] = _stack_note(s, facts)
        return result

    spec = _rate_only_chart(flight)
    if spec is None:
        result.warnings.append("No gyro data, so there is nothing to say about roll.")
        return result
    result.charts = [spec]
    return result
