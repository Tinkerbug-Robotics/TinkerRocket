"""Kinematics module — wraps the 26 plot_* functions in plot_flight_data_mini.

Produces the canonical "what happened on this flight" set of figures, plus
headline summary metrics (apogee altitude, max speed, flight duration, etc.).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

# Reuse the canonical parser/plotters (parent dir is Data_Analysis/)
_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import (  # noqa: E402
    plot_acc_sats, plot_acc_low_g, plot_acc_high_g, plot_acc_overlay,
    plot_gyro_xyz, plot_pressure, plot_temperature, plot_magnetometer,
    plot_voltage, plot_current, plot_soc,
    plot_gps_altitude, plot_gps_alt_annotated, plot_gps_velocity,
    plot_gps_ground_track, plot_gps_enu_3d, plot_gps_enu_2d,
    plot_attitude, plot_roll_cmd_gyro,
    plot_nav_altitude, plot_nav_velocity, plot_rocket_state,
    plot_nav_pos_3d, plot_nav_pos_2d,
    plot_press_vs_gps_alt, plot_nav_vs_gps_pos,
    get_array, pressure_to_altitude,
)

from ..flight import Flight
from ..registry import AnalysisResult


# (label, plot_function, takes_t0)
_PLOTS = [
    ("Accel + Sats",            plot_acc_sats,           True),
    ("Low-G Accel",             plot_acc_low_g,          True),
    ("High-G Accel",            plot_acc_high_g,         True),
    ("Accel Overlay",           plot_acc_overlay,        True),
    ("Gyro XYZ",                plot_gyro_xyz,           True),
    ("Pressure / Baro Alt",     plot_pressure,           True),
    ("Temperature",             plot_temperature,        True),
    ("Magnetometer",            plot_magnetometer,       True),
    ("Battery Voltage",         plot_voltage,            True),
    ("Battery Current",         plot_current,            True),
    ("State of Charge",         plot_soc,                True),
    ("GPS Altitude",            plot_gps_altitude,       True),
    ("GPS Alt Annotated",       plot_gps_alt_annotated,  True),
    ("GPS Velocity",            plot_gps_velocity,       True),
    ("GPS Ground Track",        plot_gps_ground_track,   False),
    ("GPS ENU 3D",              plot_gps_enu_3d,         False),
    ("GPS ENU 2D",              plot_gps_enu_2d,         False),
    ("Attitude",                plot_attitude,           True),
    ("Roll Command vs Gyro",    plot_roll_cmd_gyro,      True),
    ("Nav Altitude",            plot_nav_altitude,       True),
    ("Nav Velocity",            plot_nav_velocity,       True),
    ("Rocket State",            plot_rocket_state,       True),
    ("Nav Position 3D",         plot_nav_pos_3d,         True),
    ("Nav Position 2D",         plot_nav_pos_2d,         True),
    ("Pressure vs GPS Alt",     plot_press_vs_gps_alt,   True),
    ("Nav vs GPS Position",     plot_nav_vs_gps_pos,     True),
]


def _summary_metrics(flight: Flight) -> dict[str, float]:
    """Compute headline metrics from records (independent of .json sidecar)."""
    out: dict[str, float] = {}
    recs = flight.records
    t0 = flight.t0_us or 0

    # Apogee altitude (baro)
    baro = recs.get("BMP585") or []
    if baro:
        p_arr = get_array(baro, "pressure_pa")
        if p_arr.size:
            # Use first 200 samples as ground reference, fall back to median
            p_ground = float(np.mean(p_arr[: min(200, p_arr.size)]))
            alt = np.array([pressure_to_altitude(float(p), p_ground) for p in p_arr])
            out["max_baro_alt_m"] = float(np.max(alt))

    # GNSS max altitude + max speed
    gnss = recs.get("GNSS") or []
    if gnss:
        if "alt_m" in gnss[0]:
            gnss_alt = get_array(gnss, "alt_m")
            if gnss_alt.size:
                out["max_gnss_alt_m"] = float(np.max(gnss_alt))
        if all(k in gnss[0] for k in ("vel_e", "vel_n", "vel_u")):
            ve = get_array(gnss, "vel_e")
            vn = get_array(gnss, "vel_n")
            vu = get_array(gnss, "vel_u")
            speed = np.sqrt(ve ** 2 + vn ** 2 + vu ** 2)
            if speed.size:
                out["max_gnss_speed_mps"] = float(np.max(speed))

    # Duration
    dur = flight.duration_s
    if dur is not None:
        out["duration_s"] = round(dur, 2)

    # Launch / apogee / landed timing from NonSensor (parser exposes booleans).
    ns = recs.get("NonSensor") or []
    launch_us = apogee_us = landed_us = None
    for r in ns:
        if launch_us is None and r.get("launch"):
            launch_us = r["time_us"]
        if apogee_us is None and r.get("alt_apogee"):
            apogee_us = r["time_us"]
        if landed_us is None and r.get("alt_landed"):
            landed_us = r["time_us"]
    if launch_us is not None:
        out["launch_t_s"] = round((launch_us - t0) / 1e6, 3)
    if apogee_us is not None and launch_us is not None:
        out["apogee_after_launch_s"] = round((apogee_us - launch_us) / 1e6, 3)
    if landed_us is not None and launch_us is not None:
        out["flight_time_s"] = round((landed_us - launch_us) / 1e6, 3)

    return out


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="kinematics", title="Kinematics")
    records = flight.records
    t0 = flight.t0_us

    if t0 is None:
        result.warnings.append("No timestamped records — nothing to plot.")
        return result

    result.metrics = _summary_metrics(flight)

    for label, fn, takes_t0 in _PLOTS:
        try:
            fig = fn(records, t0) if takes_t0 else fn(records)
        except Exception as e:  # noqa: BLE001
            result.warnings.append(f"{label}: plot failed — {type(e).__name__}: {e}")
            continue
        if fig is not None:
            result.figures.append(fig)

    if not result.figures:
        result.warnings.append("No plots produced (records empty or all sensors missing).")

    return result
