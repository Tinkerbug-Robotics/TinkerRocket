"""Flight overview — the headline series as interactive (zoomable) charts.

Same data as the static figures further down the report, but rendered with
Plotly so a user can zoom into the launch transient, the apogee, or the
descent. Runs first so it lands at the top of the report.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

# Reuse the canonical accessors (parent dir is Data_Analysis/)
_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import (  # noqa: E402
    ROCKET_STATES,
    get_array,
    gnss_to_enu,
    pressure_to_altitude,
)

from ..charts import COLORS, chart, trace, trajectory_3d
from ..events import markers
from ..flight import Flight
from ..imu import G
from ..registry import AnalysisResult


# The vocabulary for every series name in this report. A reader should be able
# to look up "EKF" or "Low-G" once, here, and then recognize it in a legend
# without wondering whether "nav filter" is the same thing.
DATA_SOURCES = {
    "Baro": "BMP585 barometer. Altitude from air pressure, referenced to the pad.",
    "GNSS": "u-blox satellite receiver. Position, velocity and satellite count, ~18 Hz.",
    "EKF": "The flight computer's navigation filter, which fuses the accelerometers, "
           "gyro, barometer and GNSS into one solution for position, velocity and attitude.",
    "Low-G": "The finer of the two accelerometers (ISM6HG256, low range). Carries the "
             "boost profile, and clips at its limit during ejection and landing.",
    "High-G": "The coarser accelerometer (ISM6HG256, high range). Reads the peaks that "
              "the Low-G one clips.",
    "Gyro": "ISM6HG256 gyroscope. Angular rate; roll is its X axis.",
}


def _t(records, t0_us):
    """Record timestamps as seconds since the first frame."""
    return (get_array(records, "time_us") - t0_us) / 1e6


def _events(flight) -> dict[str, float]:
    """Every marker these charts draw, measured off the sensor record.

    See events.py. These used to be the flags, which put the "Apogee" line on
    the still-climbing part of the altitude trace — the barometric detector's
    vote fires before the peak, not after it.
    """
    return {k: round(v, 3) for k, v in markers(flight).items() if v is not None}


def _altitude_chart(recs, t0, events):
    """Baro, GNSS and nav-filter altitude on one axis — the key comparison."""
    traces = []
    baro = recs.get("BMP585") or []
    if baro:
        p = get_array(baro, "pressure_pa")
        if p.size:
            # Ground reference from the pad-phase mean, as the metrics do.
            p_ground = float(np.mean(p[: min(200, p.size)]))
            alt = np.array([pressure_to_altitude(float(v), p_ground) for v in p])
            traces.append(trace(_t(baro, t0), alt, "Baro Altitude", COLORS[0]))

    gnss = recs.get("GNSS") or []
    if gnss and "alt_m" in (gnss[0] if gnss else {}):
        alt = get_array(gnss, "alt_m")
        if alt.size:
            # GNSS altitude is MSL; offset to the first fix so it shares the AGL axis.
            traces.append(trace(_t(gnss, t0), alt - float(alt[0]),
                                "GNSS Altitude", COLORS[1]))

    ns = recs.get("NonSensor") or []
    if ns and "u_pos" in (ns[0] if ns else {}):
        traces.append(trace(_t(ns, t0), get_array(ns, "u_pos"),
                            "EKF Altitude", COLORS[2]))

    return chart("chart-altitude", "Altitude", traces,
                 y_title="Altitude AGL", y_unit="m", events=events, height=420,
                 clip_spikes=True)


def _velocity_chart(recs, t0, events):
    traces = []
    ns = recs.get("NonSensor") or []
    if ns:
        first = ns[0]
        for key, label, color in [("e_vel", "EKF East", COLORS[0]),
                                  ("n_vel", "EKF North", COLORS[1]),
                                  ("u_vel", "EKF Up", COLORS[2])]:
            if key in first:
                traces.append(trace(_t(ns, t0), get_array(ns, key), label, color))

    gnss = recs.get("GNSS") or []
    if gnss and all(k in (gnss[0] if gnss else {}) for k in ("vel_e", "vel_n", "vel_u")):
        speed = np.sqrt(get_array(gnss, "vel_e") ** 2
                        + get_array(gnss, "vel_n") ** 2
                        + get_array(gnss, "vel_u") ** 2)
        traces.append(trace(_t(gnss, t0), speed, "GNSS Speed", COLORS[3]))

    return chart("chart-velocity", "Velocity", traces,
                 y_title="Velocity", y_unit="m/s", events=events, clip_spikes=True)


# Axis picks the hue, sensor picks the intensity, so six traces stay readable:
# the two readings of the same axis sit together and the pair that disagrees is
# obvious. Ordered high-G first in each pair so the finer low-G trace paints on
# top of it — Plotly draws in trace order.
# Lead-in before the launch flag on the acceleration charts.
_PRE_LAUNCH_S = 1.0

_ACCEL_SERIES = [
    ("high_acc", "x", "High-G X", "#aec7e8"), ("low_acc", "x", "Low-G X", "#1f77b4"),
    ("high_acc", "y", "High-G Y", "#98df8a"), ("low_acc", "y", "Low-G Y", "#2ca02c"),
    ("high_acc", "z", "High-G Z", "#ff9896"), ("low_acc", "z", "Low-G Z", "#d62728"),
]


def _accel_chart_axes(recs, t0, events, chart_id, title, end_key, note):
    """Per-axis acceleration from both accelerometers, as the detailed report's plots show it.

    Three axes times two sensors. In G rather than m/s² because that is what the
    hobby quotes and what the summary card reports, and because at rest the
    vertical axis then reads a recognizable 1.

    Both sensors are drawn rather than one being chosen, for the same reason the
    altitude chart overlays baro, GNSS and nav filter: they disagree in ways
    worth seeing. The low-G part is the better instrument and carries the boost
    detail; it also visibly flat-tops wherever it rails. The high-G part is the
    only one that reads those peaks correctly and is coarser everywhere else.
    """
    imu = recs.get("ISM6HG256") or []
    if not imu:
        return None

    t = _t(imu, t0)
    launch, end = events.get("launch"), events.get(end_key)
    if launch is None or end is None:
        window = np.ones(t.shape, dtype=bool)
        shown = events
        note = ""
    else:
        # A second of pad before the launch flag. Starting exactly at launch
        # drops the 1 G reference the boost is read against, and reads as data
        # missing from the front of the chart.
        start = max(float(t[0]), launch - _PRE_LAUNCH_S)
        window = (t >= start) & (t <= end)
        # Only the events inside the window. An event marker outside it stretches
        # the x axis to reach — a "Landed" flag at 75 s would squeeze a
        # launch-to-apogee crop into the left eighth of its own chart.
        shown = {k: v for k, v in events.items() if start <= v <= end}
    if window.sum() < 10:
        return None

    traces = [
        trace(t[window], get_array(imu, f"{prefix}_{axis}")[window] / G, label, color)
        for prefix, axis, label, color in _ACCEL_SERIES
        if f"{prefix}_{axis}" in imu[0]
    ]
    if not traces:
        return None

    spec = chart(chart_id, title, traces, y_title="Acceleration", y_unit="G",
                 events=shown, clip_spikes=True)
    if spec and note:
        spec["note"] = note
    return spec


def _accel_chart_flight(recs, t0, events):
    return _accel_chart_axes(
        recs, t0, events, "chart-accel-g", "Acceleration",
        "landed" if events.get("landed") is not None else "apogee",
        "Both accelerometers, all three axes, from the pad to touchdown. Color is the axis; "
        "the paler trace of each pair is the high-G part.",
    )


def _accel_chart_boost(recs, t0, events):
    """The same series again, cropped to the powered and coasting climb.

    The full-flight view is dominated by the ejection and landing transients,
    which squash the part of the trace a flyer actually wants to read. Zooming
    reaches it, but the boost profile is worth having in front of you without
    having to go looking for it.
    """
    return _accel_chart_axes(
        recs, t0, events, "chart-accel-boost", "Acceleration — launch to apogee",
        "apogee",
        "The same six series before apogee, where the ejection and landing transients "
        "are not compressing the scale.",
    )


def _accel_chart(recs, t0, events):
    imu = recs.get("ISM6HG256") or []
    if not imu:
        return None
    t = _t(imu, t0)
    traces = [
        trace(t, get_array(imu, f"low_acc_{ax}"), f"Low-G {ax.upper()}", COLORS[i])
        for i, ax in enumerate(("x", "y", "z"))
        if f"low_acc_{ax}" in imu[0]
    ]
    return chart("chart-accel", "Acceleration (low-G)", traces,
                 y_title="Acceleration", y_unit="m/s²", events=events)


def _gyro_chart(recs, t0, events):
    imu = recs.get("ISM6HG256") or []
    if not imu:
        return None
    t = _t(imu, t0)
    traces = [
        trace(t, get_array(imu, f"gyro_{ax}"), f"Gyro {ax.upper()}", COLORS[i])
        for i, ax in enumerate(("x", "y", "z"))
        if f"gyro_{ax}" in imu[0]
    ]
    return chart("chart-gyro", "Angular rate", traces,
                 y_title="Rate (deg/s)", events=events)


def _ground_track_chart(recs):
    """GNSS track in local ENU meters, equal aspect so distances read true."""
    gnss = recs.get("GNSS") or []
    if not gnss:
        return None
    e, n, _u = gnss_to_enu(gnss)
    if e.size == 0:
        return None
    traces = [trace(e, n, "Ground track", COLORS[0], mode="lines+markers")]
    start = trace([e[0]], [n[0]], "Start", "#2ca02c", mode="markers", size=11)
    end = trace([e[-1]], [n[-1]], "Landing", "#d62728", mode="markers", size=11)
    traces += [start, end]
    return chart("chart-track", "GNSS ground track", traces,
                 x_title="East", y_title="North", x_unit="m", y_unit="m",
                 height=460, equal_aspect=True)


def _trajectory_chart(recs, t0, events):
    """Nav-filter position as a 3D path, windowed to the flight itself.

    Windowed on purpose: the nav filter runs for the whole session, and the
    minutes on the pad are thousands of samples piled on the origin — slow to
    draw and not part of the trajectory. Everything between launch and landing
    is kept at full resolution.

    Detailed-level, since the globe module draws the same path over real
    imagery in the flight report. Kept rather than deleted because it survives
    two things the globe does not: no network when the report is opened (Cesium
    fetches imagery and terrain at view time, and model rocketry happens in
    fields), and no GNSS fix at all — a flight with a dead receiver has no
    georeference but still has a complete trajectory in meters from the pad.
    """
    ns = recs.get("NonSensor") or []
    if not ns or not all(k in ns[0] for k in ("e_pos", "n_pos", "u_pos")):
        return None

    t = _t(ns, t0)
    start = events.get("launch")
    end = events.get("landed") or events.get("apogee")
    if start is None or end is None:
        window = np.ones(t.shape, dtype=bool)
        note = ""
    else:
        window = (t >= start) & (t <= end)
        note = "Launch to touchdown. Drag to rotate, scroll to zoom."
    if window.sum() < 10:
        return None

    return trajectory_3d(
        "chart-trajectory", "Flight path in 3D",
        get_array(ns, "e_pos")[window],
        get_array(ns, "n_pos")[window],
        get_array(ns, "u_pos")[window],
        name="Nav filter path",
        note=note,
    )


def _state_chart(recs, t0, events):
    ns = recs.get("NonSensor") or []
    if not ns or "rocket_state" not in ns[0]:
        return None
    t = trace(_t(ns, t0), get_array(ns, "rocket_state"), "State", COLORS[4])
    return chart("chart-state", "Rocket state", [t],
                 y_title="", events=events, height=260,
                 y_ticks=(list(ROCKET_STATES.keys()), list(ROCKET_STATES.values())))


def _build(result: AnalysisResult, builders) -> AnalysisResult:
    for build in builders:
        try:
            spec = build()
        except Exception as e:  # noqa: BLE001 — one bad chart shouldn't lose the rest
            result.warnings.append(f"chart failed — {type(e).__name__}: {e}")
            continue
        if spec:
            result.charts.append(spec)

    if not result.charts:
        result.warnings.append("No interactive charts produced (no usable sensor data).")
    return result


def analyze(flight: Flight) -> AnalysisResult:
    """Flight-level charts: the trajectory, as a flyer thinks about it.

    Deliberately excludes the raw IMU series. Accelerometer and gyro log at ~1 kHz
    for the whole session — several hundred thousand samples, most of them the
    rocket sitting on the pad — and carrying them at full resolution would make
    the flight report several times heavier than the detailed one. They live
    in `analyze_sensors` instead, where detail costs nothing.
    """
    result = AnalysisResult(name="overview", title="Flight Overview")
    result.metrics = dict(DATA_SOURCES)
    result.metric_headers = ("Source", "What it is")
    recs = flight.records
    t0 = flight.t0_us

    if t0 is None:
        result.warnings.append("No timestamped records — nothing to chart.")
        return result

    events = _events(flight)
    # No ground-track chart here: the flight report's geography is the 3D globe.
    # The ENU chart and the flat recovery map both live in the detailed report, along
    # with the 3D Plotly trajectory — see _trajectory_chart for why that one is
    # kept rather than deleted.
    # Position, then velocity, then acceleration — each the derivative of the one
    # before it. The 3D path is the position view and leads the whole report from
    # the globe module; altitude is that same position against time.
    return _build(result, [
        lambda: _altitude_chart(recs, t0, events),
        lambda: _velocity_chart(recs, t0, events),
        lambda: _accel_chart_flight(recs, t0, events),
        lambda: _accel_chart_boost(recs, t0, events),
    ])


def analyze_sensors(flight: Flight) -> AnalysisResult:
    """Detailed-level charts: raw inertial sensors at full logged rate."""
    result = AnalysisResult(name="sensor_charts", title="Inertial Sensors (interactive)")
    recs = flight.records
    t0 = flight.t0_us

    if t0 is None:
        result.warnings.append("No timestamped records — nothing to chart.")
        return result

    events = _events(flight)
    return _build(result, [
        lambda: _accel_chart(recs, t0, events),
        lambda: _gyro_chart(recs, t0, events),
        lambda: _state_chart(recs, t0, events),
        lambda: _trajectory_chart(recs, t0, events),
        lambda: _ground_track_chart(recs),
    ])
