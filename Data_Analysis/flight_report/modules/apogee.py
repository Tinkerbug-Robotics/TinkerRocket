"""How apogee was called — the four detectors, and which one got there first.

The flight computer does not have *an* apogee detector, it has four: vertical
velocity crossing zero, the barometer's altitude turning over, GNSS vertical
velocity, and the vehicle's pitch. Each votes, and a master flag latches from
those votes. That is what fires the ejection charge, so when a deployment is
early or late this is the picture that says which detector caused it.

Two charts, because a Plotly spec here is one pair of axes and this needs two:

* altitude through apogee, with each detector's fire marked at the altitude it
  fired at, plus the master flag and the true apogee, and
* a lane per detector showing every span where its condition was true — which is
  not the same as when it fired, since a detector can trip and recover.

Both come from replaying the log through the firmware's own kinematic-checks
code rather than from the logged flags, so a detector's *condition* history is
visible and not just the latched result. The replay lives outside this package
and may be absent, in which case this section says so and the rest of the report
is unaffected.
"""

from __future__ import annotations

from typing import Any, Optional

from ..charts import X_HOVER_FORMAT, chart, trace
from ..flight import Flight
from ..registry import AnalysisResult
from .kinematic_checks import (
    _DET_FIRE_FLAG,
    _DET_STYLE,
    _IMPORT_ERROR,
    _IMPORT_OK,
    _replay,
)

# The detector labels this report uses. kinematic_checks calls them "Vel" and
# "GPS"; renamed locally rather than in the shared constant, so the detailed
# figures keep their wording while this report stays consistent with the Data
# Sources table ("GNSS", never "GPS").
_DET_LABEL = {"vel": "Velocity", "baro": "Baro", "gps": "GNSS", "pitch": "Pitch"}

# The two reference lines on the altitude pane. charts._EVENT_STYLE only knows
# launch/burnout/apogee/landed and falls back to light grey with the raw key as
# the label, which for the two most important marks on this chart is the wrong
# way round — these are what the whole section is comparing. Styled explicitly
# below: near-black for the truth, red for the flag that fired the charge.
_TRUE_APOGEE = "True Apogee"
_MASTER_APOGEE = "Master Apogee"
_LANDED = "Landed"
_EVENT_EMPHASIS = {
    _TRUE_APOGEE: "#1a1a1a",
    _MASTER_APOGEE: "#c62828",
    _LANDED: "#6b4c3b",
    "Pyro 1": "#000000", "Pyro 2": "#444444",
    "Pyro 3": "#666666", "Pyro 4": "#888888",
}


def _log_events(recs) -> dict[str, float]:
    """Pyro fires and the landing declaration, on the replay's time base.

    The replay measures from the first IMU sample rather than `Flight.t0_us`, so
    these are read against the same origin — a few milliseconds either way would
    put the ejection charge on the wrong side of the apogee call.

    Pyro channels are almost always disabled on a motor-ejection flight, in which
    case nothing is drawn. The lines exist for the flights that do fire one.
    """
    ns = recs.get("NonSensor") or []
    imu = recs.get("ISM6HG256") or []
    if not ns or not imu:
        return {}
    t0_us = imu[0]["time_us"]

    out: dict[str, float] = {}
    for label, key in (("Landed", "alt_landed"),
                       ("Pyro 1", "pyro1_fired"), ("Pyro 2", "pyro2_fired"),
                       ("Pyro 3", "pyro3_fired"), ("Pyro 4", "pyro4_fired")):
        for r in ns:
            if r.get(key):
                out[label] = round((r["time_us"] - t0_us) / 1e6, 3)
                break
    return out


# Matplotlib's tab: names mean nothing to Plotly.
_TAB = {"tab:blue": "#1f77b4", "tab:green": "#2ca02c",
        "tab:orange": "#ff7f0e", "tab:purple": "#9467bd"}

# Seconds either side of the master apogee call. The interesting part is the few
# seconds around the turnover; the whole flight would compress it to nothing.
_BEFORE, _AFTER = 8.0, 5.0


def _emphasize_events(spec: dict, lo=None, hi=None) -> None:
    """Darken and enlarge the reference lines and their labels."""
    if not spec:
        return
    layout = spec["layout"]
    times = {}
    # Inside the plot, not in the margin above it. chart() puts event labels in
    # the top margin, which is also where the chart title lives — with a title
    # this long the two collide whatever the margin depth. Stacked at two heights
    # because the events are usually under a second apart, on white plates so
    # they stay readable over the altitude curve.
    for row, ann in enumerate(a for a in layout.get("annotations", [])
                              if a.get("text") in _EVENT_EMPHASIS):
        color = _EVENT_EMPHASIS[ann["text"]]
        times[round(float(ann["x"]), 3)] = color
        ann["font"] = {"size": 12, "color": color}
        ann["y"] = 0.97 - 0.09 * row
        ann["yanchor"] = "top"
        # Labels sit to the right of their line, except near the right edge,
        # where that runs them off the plot — the landing declaration is at the
        # far end of this window by construction.
        near_right = (lo is not None and hi is not None
                      and float(ann["x"]) > lo + 0.82 * (hi - lo))
        ann["xanchor"] = "right" if near_right else "left"
        ann["xshift"] = -4 if near_right else 4
        ann["bgcolor"] = "rgba(255,255,255,0.82)"
        ann["borderpad"] = 2
    for shape in layout.get("shapes", []):
        color = times.get(round(float(shape.get("x0", 0)), 3))
        if color and shape.get("type") == "line":
            shape["line"] = {"color": color, "width": 1.6, "dash": "dash"}


def _window(res) -> tuple[Optional[float], Optional[float]]:
    """The turnover: a few seconds either side of the apogee call.

    Deliberately tight. Landing is ~70 s later, and stretching this to reach it
    squeezes the four detector marks — the thing the chart exists to separate —
    into a couple of percent of the width. Anything outside this span is marked
    on the recovery chart below instead.
    """
    t_master = res["sim_fires"].get("apogee_flag")
    t_raw, _ = res["raw_palt"]
    if t_master is None or not t_raw:
        return None, None
    return max(0.0, t_master - _BEFORE), min(t_master + _AFTER, t_raw[-1])


def _crop(ts, vs, lo, hi):
    if lo is None:
        return list(ts), list(vs)
    pairs = [(t, v) for t, v in zip(ts, vs) if lo <= t <= hi]
    return [p[0] for p in pairs], [p[1] for p in pairs]


def _altitude_chart(res, marks: dict[str, float]) -> Optional[dict[str, Any]]:
    t_sim, v_sim = res["sim_alt"]
    if not t_sim:
        return None
    lo, hi = _window(res)
    t_raw, v_raw = res["raw_palt"]

    traces = []
    if t_raw:
        rt, rv = _crop(t_raw, v_raw, lo, hi)
        traces.append(trace(rt, rv, "Baro Raw", "#bbbbbb"))
    # "Baro Filtered", not just "Filtered": the replay's Kalman filter takes
    # pressure altitude as its only measurement (sim_kinematic_checks
    # `_update_alt_kf(pressure_altitude, ...)`), so this is a smoothed barometer
    # and not a fused solution. The EKF track is a different thing entirely.
    st, sv = _crop(t_sim, v_sim, lo, hi)
    traces.append(trace(st, sv, "Baro Filtered", "#333333"))

    # Each detector's fire, placed at the altitude the filter had at that moment,
    # so the vertical spread between them reads as "how much altitude was lost
    # between the first vote and the last".
    for det_key, flag_name in _DET_FIRE_FLAG.items():
        t = res["sim_fires"].get(flag_name)
        if t is None or not t_sim:
            continue
        i = min(range(len(t_sim)), key=lambda j: abs(t_sim[j] - t))
        _, color = _DET_STYLE[det_key]
        traces.append(trace([t], [v_sim[i]], f"{_DET_LABEL[det_key]} Fired",
                            _TAB.get(color, color), mode="markers", size=13))

    # Where the firmware discarded a barometer sample as untrustworthy. Drawn
    # along the floor of the pane so the run of rejections reads as a period
    # rather than as altitude data.
    rejects = [t for t in (res.get("baro_reject_t") or []) if lo is None or lo <= t <= hi]
    if rejects:
        floor = min(sv) - 0.06 * (max(sv) - min(sv) or 1.0)
        rj = trace(rejects, [floor] * len(rejects), "Baro Rejected", "#d62728",
                   mode="markers", size=6)
        if rj:
            rj["hovertemplate"] = f"Baro rejected<br>%{{x:{X_HOVER_FORMAT}}} s<extra></extra>"
            traces.append(rj)

    events: dict[str, float] = {}
    true_t = res.get("true_apogee_t")
    if true_t is not None:
        events[_TRUE_APOGEE] = round(float(true_t), 3)
    t_master = res["sim_fires"].get("apogee_flag")
    if t_master is not None:
        events[_MASTER_APOGEE] = round(float(t_master), 3)
    # Only the marks that fall inside the turnover — a pyro that fired here
    # belongs on this chart; the rest go on the recovery chart below.
    events.update({k: v for k, v in marks.items() if lo is None or lo <= v <= hi})

    spec = chart("chart-apogee-vote", "Apogee detection — which detector fired when",
                 traces, y_title="Altitude", y_unit="m", events=events, height=400)
    if spec and lo is not None:
        spec["layout"]["xaxis"]["range"] = [lo, hi]
    _emphasize_events(spec, lo, hi)
    if spec:
        note = ("Each marker is one detector calling apogee; the master flag, which fires "
                "the charge, latches from their votes.")
        if res.get("baro_reject_t"):
            note += (f" Baro was rejected {len(rejects)} times in this window, by "
                     "design, while the vehicle was too fast for pressure to be reliable.")
        spec["note"] = note
    return spec


def _recovery_chart(res, marks: dict[str, float]) -> Optional[dict[str, Any]]:
    """Apogee to touchdown, carrying the marks that fall outside the turnover.

    Its own chart rather than a wider window on the one above: the two spans
    differ by a factor of thirty, and one axis cannot serve both. This one answers
    "what happened after the charge fired and when did it decide it was down",
    which is a different question from "which detector called it".
    """
    t_sim, v_sim = res["sim_alt"]
    if not t_sim:
        return None
    _, turnover_hi = _window(res)
    t_master = res["sim_fires"].get("apogee_flag")
    if t_master is None:
        return None

    outside = {k: v for k, v in marks.items()
               if turnover_hi is None or v > turnover_hi}
    if not outside:
        return None

    lo = max(0.0, t_master - 2.0)
    hi = min(max(outside.values()) + 3.0, t_sim[-1])
    st, sv = _crop(t_sim, v_sim, lo, hi)
    if len(st) < 2:
        return None

    spec = chart("chart-recovery", "Descent and landing",
                 [trace(st, sv, "Baro Filtered", "#333333")],
                 y_title="Altitude", y_unit="m",
                 events={k: round(v, 3) for k, v in outside.items()}, height=300)
    if not spec:
        return None
    spec["layout"]["xaxis"]["range"] = [lo, hi]
    _emphasize_events(spec, lo, hi)
    named = ", ".join(sorted(outside))
    spec["note"] = (f"The descent to touchdown, carrying the events that fall outside the "
                    f"turnover above: {named}. Recovery runs tens of seconds while the "
                    f"turnover runs a few, so the two need separate axes.")
    return spec


def _lane_chart(res) -> Optional[dict[str, Any]]:
    """One row per detector, showing every span where its condition held."""
    lo, hi = _window(res)
    rows = [("vel", res["vel_periods"]), ("baro", res["baro_periods"]),
            ("gps", res["gps_periods"]), ("pitch", res["pitch_periods"])]

    traces, tickvals, ticktext = [], [], []
    for i, (det_key, periods) in enumerate(rows):
        _, color = _DET_STYLE[det_key]
        label = _DET_LABEL[det_key]
        y = len(rows) - 1 - i
        tickvals.append(y)
        ticktext.append(label)
        # One trace per span rather than one trace with None separators: the
        # shared trace builder drops non-finite pairs, so the separators would be
        # eaten and every span welded into one bar that never went false.
        for (a, b) in periods:
            b = max(b, a + 0.005)
            if lo is not None:
                if b < lo or a > hi:
                    continue
                # Clamp rather than keep whole: a condition that stays true to
                # touchdown would otherwise stretch this chart to the full flight
                # while the altitude chart above it stays on the apogee window.
                a, b = max(a, lo), min(b, hi)
            t = trace([a, b], [y, y], label, _TAB.get(color, color),
                      mode="lines", width=9)
            if t:
                t["hovertemplate"] = (f"{label} condition true<br>"
                                      f"%{{x:{X_HOVER_FORMAT}}} s<extra></extra>")
                traces.append(t)

    if not traces:
        return None

    spec = chart("chart-apogee-lanes", "Apogee detectors — when each condition held",
                 traces, y_title="", height=240,
                 y_ticks=(tickvals, ticktext))
    if spec:
        spec["layout"]["yaxis"]["range"] = [-0.7, len(rows) - 0.3]
        spec["layout"]["showlegend"] = False
        # The two charts are separate Plotly instances and so do not share a zoom.
        # Pinning both to the same x range at least makes them line up at rest,
        # which is the whole point of reading them one above the other.
        if lo is not None:
            spec["layout"]["xaxis"]["range"] = [lo, hi]
        spec["note"] = ("A bar is where that detector's condition held. They can trip and "
                        "recover; the flag latches on the first one that sticks.")
    return spec


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="apogee", title="Apogee Detection")

    if not _IMPORT_OK:
        result.warnings.append(
            "The apogee-detector breakdown needs the firmware replay, which is not "
            f"available here ({_IMPORT_ERROR}). Nothing else in this report depends "
            "on it."
        )
        return result

    recs = flight.records
    if not recs.get("NonSensor") or not recs.get("ISM6HG256"):
        result.warnings.append("Not enough logged data to replay the apogee detectors.")
        return result

    try:
        res = _replay(recs)
    except Exception as e:  # noqa: BLE001 — a failed replay costs this section only
        result.warnings.append(f"The apogee-detector replay did not run: {type(e).__name__}: {e}")
        return result
    if res is None:
        result.warnings.append("The apogee-detector replay produced no data.")
        return result

    marks = _log_events(recs)
    charts = [c for c in (_altitude_chart(res, marks), _lane_chart(res)) if c]
    if not charts:
        result.warnings.append("The replay ran but produced no altitude series to plot.")
        return result

    # Stack them as one figure, as the detailed report's version does with a shared-x
    # GridSpec: the lanes only mean anything read against the altitude above
    # them, and a gap plus a caption between the two breaks that reading.
    if len(charts) == 2:
        top, bottom = charts
        top["stack"], bottom["stack"] = "first", "last"
        top["layout"]["xaxis"]["showticklabels"] = False
        top["layout"]["xaxis"]["title"] = {"text": ""}
        top["layout"]["margin"] = {"l": 70, "r": 20, "t": 40, "b": 6}
        bottom["layout"]["margin"] = {"l": 70, "r": 20, "t": 8, "b": 42}
        bottom["layout"]["title"] = {"text": "", "font": {"size": 14}}
        # Top-left: over this window the vehicle is still climbing on the left
        # and descending on the right, so the upper left is the empty corner.
        # The rejection marks run along the floor, ruling out the bottom.
        top["layout"]["legend"] = {"x": 0.008, "xanchor": "left", "y": 0.98,
                                   "yanchor": "top", "font": {"size": 10},
                                   "bgcolor": "rgba(255,255,255,0.72)"}
        # One caption, under the pair.
        bottom["note"] = ((top.pop("note", "") or "") + " " +
                          (bottom.get("note") or "")).strip()
        top["note"] = ""

    # Standalone, not stacked: it covers a span thirty times longer, so it does
    # not share the axis above and must not look as though it does.
    recovery = _recovery_chart(res, marks)
    if recovery:
        charts.append(recovery)

    result.charts = charts
    return result
