"""Kinematic-checks module.

Replays the flight through `sim_kinematic_checks.KinematicChecks` (the Python
port of TR_KinematicChecks) and compares when each flag fires vs when the
firmware fired it (logged in NonSensor).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

try:
    from replay_kinematic_checks import (  # noqa: E402
        accel_norm_firmware,
        estimate_ground_pressure,
        build_events,
        logged_flag_times,
        pressure_to_altitude_firmware,
        BARO_MACH_LOCKOUT_ON,
        BARO_MACH_LOCKOUT_OFF,
        NSF_BURNOUT,
    )
    from sim_kinematic_checks import KinematicChecks  # noqa: E402
    _IMPORT_OK = True
    _IMPORT_ERROR = ""
except ImportError as e:
    _IMPORT_OK = False
    _IMPORT_ERROR = str(e)

from ..flight import Flight
from ..registry import AnalysisResult


_FLAGS_TO_CAPTURE = (
    "launch_flag", "vel_u_apogee_flag", "alt_apogee_flag",
    "gps_apogee_flag", "pitch_apogee_flag", "apogee_flag", "alt_landed_flag",
)


def _replay(records):
    """Drive KinematicChecks through merged event stream.

    Mirrors the inner loop of `replay_kinematic_checks.replay`. Returns dict
    {flag_name: fire_time_s_relative_to_t0}.
    """
    if not records["ISM6HG256"]:
        return {}

    t0_us = records["ISM6HG256"][0]["time_us"]
    ground_pa = estimate_ground_pressure(records["BMP585"], records["NonSensor"])

    kc = KinematicChecks()
    latest_palt = 0.0
    latest_acc_mag = 0.0
    latest_pos = (0.0, 0.0, 0.0)
    latest_vel = (0.0, 0.0, 0.0)
    latest_roll_rate = 0.0
    latest_gps_alt = 0.0
    latest_pitch_rad = math.pi / 2
    burnout = False
    mach_locked_out = False
    new_baro = False
    new_gps = False

    sim_fires: dict[str, float] = {}

    for t_us, kind, r in build_events(records):
        now_ms = (t_us - t0_us) // 1000
        if kind == "baro":
            latest_palt = pressure_to_altitude_firmware(r["pressure_pa"], ground_pa)
            new_baro = True
            continue
        if kind == "gnss":
            latest_gps_alt = float(r["alt_m"])
            new_gps = True
            continue
        if kind == "ns":
            latest_pos = (r["e_pos"], r["n_pos"], r["u_pos"])
            latest_vel = (r["e_vel"], r["n_vel"], r["u_vel"])
            latest_pitch_rad = math.radians(r["pitch"])
            burnout = bool(r["flags"] & NSF_BURNOUT)
            speed = math.sqrt(r["e_vel"]**2 + r["n_vel"]**2 + r["u_vel"]**2)
            if not mach_locked_out and speed > BARO_MACH_LOCKOUT_ON:
                mach_locked_out = True
            elif mach_locked_out and speed < BARO_MACH_LOCKOUT_OFF:
                mach_locked_out = False
            continue

        # imu tick
        low = (r["low_acc_x"], r["low_acc_y"], r["low_acc_z"])
        high = (r["high_acc_x"], r["high_acc_y"], r["high_acc_z"])
        latest_acc_mag = accel_norm_firmware(low, high)
        latest_roll_rate = r["gyro_x"]

        kc.kinematic_checks(
            pressure_altitude=latest_palt,
            acc_mag=latest_acc_mag,
            position=latest_pos,
            velocity=latest_vel,
            roll_rate=latest_roll_rate,
            new_baro=new_baro,
            gps_altitude=latest_gps_alt,
            new_gps=new_gps,
            pitch_rad=latest_pitch_rad,
            burnout_detected=burnout,
            baro_locked_out=mach_locked_out,
            now_ms=now_ms,
        )
        new_baro = False
        new_gps = False

        now_s = now_ms / 1000.0
        for name in _FLAGS_TO_CAPTURE:
            if getattr(kc, name) and name not in sim_fires:
                sim_fires[name] = now_s

    sim_fires["max_altitude_sim_m"] = round(float(kc.max_altitude), 2)
    sim_fires["max_speed_sim_mps"] = round(float(kc.max_speed), 2)
    return sim_fires


# Map sim flag name -> logged flag name produced by logged_flag_times()
_FLAG_PAIRS = (
    ("launch_flag",       "launch"),
    ("vel_u_apogee_flag", "vel_apogee"),
    ("alt_apogee_flag",   "alt_apogee"),
    ("gps_apogee_flag",   "gps_apogee"),
    ("pitch_apogee_flag", "pitch_apogee"),
    ("apogee_flag",       "apogee_flag"),
    ("alt_landed_flag",   "alt_landed"),
)


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="kinematic_checks", title="Kinematic Checks Replay")

    if not _IMPORT_OK:
        result.warnings.append(f"Module unavailable: {_IMPORT_ERROR}")
        return result

    recs = flight.records
    if not recs.get("NonSensor"):
        result.warnings.append("No NonSensor records — nothing to compare against.")
        return result
    if not recs.get("ISM6HG256"):
        result.warnings.append("No IMU records — replay needs IMU ticks.")
        return result

    try:
        sim = _replay(recs)
    except Exception as e:  # noqa: BLE001
        result.warnings.append(f"Replay failed: {type(e).__name__}: {e}")
        return result

    t0_us = recs["ISM6HG256"][0]["time_us"]
    logged = logged_flag_times(recs["NonSensor"], t0_us)

    metrics: dict[str, object] = {}
    for sim_name, log_name in _FLAG_PAIRS:
        sim_t = sim.get(sim_name)
        log_t = logged.get(log_name)
        metrics[f"{log_name}_logged_s"] = round(log_t, 3) if log_t is not None else "—"
        metrics[f"{log_name}_replay_s"] = round(sim_t, 3) if sim_t is not None else "—"
        if sim_t is not None and log_t is not None:
            dt = round(sim_t - log_t, 3)
            metrics[f"{log_name}_replay-logged_s"] = dt
            if abs(dt) > 0.5:
                result.warnings.append(
                    f"{log_name}: replay {dt:+.2f} s vs logged firmware decision."
                )
        else:
            metrics[f"{log_name}_replay-logged_s"] = "—"

    metrics["max_altitude_sim_m"] = sim.get("max_altitude_sim_m", "—")
    metrics["max_speed_sim_mps"] = sim.get("max_speed_sim_mps", "—")
    result.metrics = metrics
    return result
