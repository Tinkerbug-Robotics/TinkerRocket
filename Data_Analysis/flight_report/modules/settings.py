"""How the flight computer was configured for this flight, in the app's own words.

Last, because it is reference rather than reading: nobody opens a flight report
to look at settings, but the moment a number above looks wrong the first question
is what the vehicle was told to do. Keeping it here means that question is always
one scroll away and never in the way.

Two sections. **Rocket Settings** is what the flyer set in the app, grouped
under the app's own section headings and labeled with its words, so a value can
be checked against the screen it was typed into. **System Configuration** is
what the firmware and sensors were running underneath — full-scale ranges,
mounting rotations, controller internals the app never shows — kept below,
where a reader chasing a wrong number can still find it without it crowding the
settings they recognize.

One source: the FlightSettingsData frame the firmware writes into the log at
launch (see flight_settings.py), plus the status-query block for the sensor
mounting rotations. The sidecar .json used to be merged in as well; it is the
app's rendering of that same frame, so it added nothing the log did not already
say, and a log without the frame has no sidecar settings either.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import MAG_TYPE_NAMES  # noqa: E402

from .. import flight_settings as fsd
from ..flight import Flight
from ..registry import AnalysisResult
from ..units import Quantity

Rows = list[tuple[str, Any]]

# The app's picker words for the three fixed logging rates.
_RATE_NAMES = {960: "1k", 1920: "2k", 3840: "4k"}
_CAMERA_LABELS = {0: "None", 1: "GoPro", 2: "RunCam"}
_MOUNT_MODES = {
    0: "Default (+X nose, nothing configured)",
    1: "Manual",
    2: "Pad auto-detect (snapped to an axis)",
    3: "Pad auto-detect (exact vector)",
}
_GUIDANCE_TARGET = {0: "the pad, overhead", 1: "a geodetic point", 2: "a profile point"}
_OTP_GLOSS = {
    "UNKNOWN": "not evaluated",
    "NOT_M10": "receiver is not an M10, never eligible",
    "VERIFIED": "high-performance clock programmed",
    "PARTIAL": "OTP partly written, left alone",
    "BLANK": "unprogrammed, left that way",
    "BLOCKLISTED": "unprogrammed, on the never-program list",
    "PROGRAMMED": "written this boot and verified",
    "WRITE_FAILED": "written this boot, verification failed",
}


def _on(flag: bool) -> str:
    """A toggle reads On or Off, as it does in the app."""
    return "On" if flag else "Off"


def _num(value: float, unit: str = "") -> str:
    """Six significant figures, as the app's export rounds float32: a gain
    typed as 0.12 reads 0.12, not 0.11999999731779099."""
    return f"{float(value):.6g} {unit}".rstrip()


def _group(groups: list[dict[str, Any]], name: str, rows: Rows) -> None:
    """Add a group, dropping rows whose value is None (a field this firmware
    version did not record) and the group itself when nothing is left."""
    kept = [(k, v) for k, v in rows if v is not None]
    if kept:
        groups.append({"name": name, "rows": kept})


# --- Rocket settings: the app's tabs in the app's order --------------------

def _mounting_rows(fs: dict[str, Any]) -> Rows:
    code, mode = fs["b2r_code"], fs["b2r_mode"]
    if code is None:
        return []
    axis, _, clock = fsd.orientation_name(code).partition(" r")
    rows: Rows = [
        ("Mode", _MOUNT_MODES.get(mode, f"?{mode}")),
        ("Nose axis", axis),
        ("Fin clocking", f"{clock or 0}°"),
    ]
    if mode in (2, 3):
        rows.append(("Auto-detect residual", f"{fs['b2r_residual_deg']:.1f}°"))
    return rows


def _rate_name(hz: int) -> str:
    return f"{_RATE_NAMES[hz]} ({hz} Hz)" if hz in _RATE_NAMES else f"{hz} Hz"


def _imu_rate(fs: dict[str, Any]) -> Optional[str]:
    hz = fs["ism6_update_rate_hz"]
    if hz is None:
        return None
    if fs["imu_rate_dynamic"]:
        return (f"Dynamic — {_rate_name(hz)} through boost and coast, "
                f"{_rate_name(fsd.IMU_RATE_DYNAMIC_POST_HZ)} after deployment")
    return _rate_name(hz)


def _servo_rows(fs: dict[str, Any]) -> Rows:
    rows: Rows = [(f"Servo {i + 1}", f"{bias} µs") for i, bias in enumerate(fs["servo_bias_us"])]
    rows += [
        ("Frequency", f"{fs['servo_hz']} Hz"),
        ("Min Pulse", f"{fs['servo_min_us']} µs"),
        ("Max Pulse", f"{fs['servo_max_us']} µs"),
    ]
    lo, hi = fs["fin_min_deg"], fs["fin_max_deg"]
    if lo is not None and hi is not None:
        travel = hi - lo
        span = fs["servo_max_us"] - fs["servo_min_us"]
        rows.append(("Fin Travel", _num(travel, "deg")))
        if span > 0:
            # The app's read-only line under the servo fields, same format.
            rows.append(("Fin Range", f"{lo:+.1f}° … {hi:+.1f}°   ({travel / span:.3f}°/µs)"))
    return rows


def _profile_text(fs: dict[str, Any]) -> str:
    if not fs["waypoints"]:
        return "None"
    return "; ".join(f"{w['time_s']:.6g} s → {w['angle_deg']:.6g}°" for w in fs["waypoints"])


def _roll_rows(fs: dict[str, Any]) -> Rows:
    rows: Rows = [
        ("Mode", "Track Profile" if fs["use_angle_control"] else "Null Roll"),
        ("Activation Delay", f"{fs['roll_delay_ms']} ms"),
    ]
    if fs["roll_min_speed_mps"] is not None:
        rows.append(("Min Speed", _num(fs["roll_min_speed_mps"], "m/s")))
    rows += [
        ("Rate Cap", _num(fs["kp_angle_rate_cap_dps"], "°/s")),
        ("Angle Gain (Kp)", _num(fs["kp_angle"])),
        ("Roll Profile", _profile_text(fs)),
    ]
    return rows


def _guidance_rows(fs: dict[str, Any]) -> Rows:
    rows: Rows = [
        ("Guidance Law", "Station-Keep" if fs["guidance_station_keep"] else "Proportional Nav"),
        ("Activation Delay", f"{fs['roll_delay_ms']} ms"),
    ]
    if fs["roll_min_speed_mps"] is not None:
        rows.append(("Min Speed", _num(fs["roll_min_speed_mps"], "m/s")))
    if fs["guid_tgt_src"] is not None:
        rows.append(("Aim Point",
                     f"E {fs['guid_tgt_e_m']:.1f} m, N {fs['guid_tgt_n_m']:.1f} m from the pad "
                     f"({_GUIDANCE_TARGET.get(fs['guid_tgt_src'], '?')})"))
    return rows


def _pyro_rows(ch: dict[str, Any]) -> Rows:
    rows: Rows = [("Enabled", _on(ch["enabled"]))]
    if not ch["enabled"]:
        return rows
    if ch["mode"] == 1:
        rows.append(("Trigger", "Altitude on descent"))
        # The one setting the app shows in the reader's unit system, so it
        # follows the report's toggle too: 228.6 m is the 750 ft that was typed.
        rows.append(("Altitude", Quantity(ch["value"], "m", places=1, suffix=" on descent")))
    else:
        rows.append(("Trigger", "Time after apogee"))
        rows.append(("Delay", f"{_num(ch['value'])} s after apogee"))
    return rows


_SETTINGS_NOTE = (
    "What was set in the app, under its own headings, read from the settings frame the "
    "flight computer wrote into the log at launch. Guidance gains, the fin layout and the "
    "Recovery descent profile are not in the log: the flight computer does not snapshot "
    "the first two, and the last lives only in the app."
)


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="settings", title="Rocket Settings")

    fs, warnings = fsd.from_flight(flight)
    result.warnings.extend(warnings)
    if fs is None:
        result.warnings.append(
            "No settings frame in this log, so there is nothing to show. The firmware "
            "writes one at launch; this log predates that, or never launched."
        )
        return result

    result.note = _SETTINGS_NOTE
    g: list[dict[str, Any]] = []
    # General
    _group(g, "Rocket", [("Enable Sounds", _on(fs["sounds_enabled"]))])
    _group(g, "IMU Mounting", _mounting_rows(fs))
    _group(g, "IMU Logging Rate", [("Rate", _imu_rate(fs))])
    # Control
    _group(g, "Servo Control", [("Enable Servo Control", _on(fs["servo_enabled"]))])
    _group(g, "PID Gains", [
        ("Kp", _num(fs["kp"])),
        ("Ki", _num(fs["ki"])),
        ("Kd", _num(fs["kd"])),
        ("Min Deflection", _num(fs["min_cmd_deg"], "deg")),
        ("Max Deflection", _num(fs["max_cmd_deg"], "deg")),
        ("Velocity Gain Scheduling", _on(fs["gain_schedule_enabled"])),
    ])
    _group(g, "Servo", _servo_rows(fs))
    _group(g, "Control Mode", [("Mode", "Guidance" if fs["guidance_enabled"] else "Roll Control")])
    if fs["guidance_enabled"]:
        _group(g, "Guidance", _guidance_rows(fs))
    else:
        _group(g, "Roll Control", _roll_rows(fs))
    # Camera
    _group(g, "Camera", [("Camera Type", _CAMERA_LABELS.get(fs["camera_type"], f"?{fs['camera_type']}"))])
    # Pyro
    for i, ch in enumerate(fs["pyro"], 1):
        _group(g, f"Pyro Channel {i}", _pyro_rows(ch))
    result.groups = g
    return result


# --- System configuration: what ran underneath -------------------------------

def _sensor_mounting_rows(flight: Flight, fs: Optional[dict[str, Any]]) -> Rows:
    config = flight.config or {}
    rows: Rows = []
    if config.get("ism6_rot_z_deg") is not None:
        rows.append(("IMU rotation about Z", f"{config['ism6_rot_z_deg']:g}°"))
    if config.get("iis2mdc_rot_z_deg") is not None:
        rows.append(("Magnetometer rotation about Z", f"{config['iis2mdc_rot_z_deg']:g}°"))
    # The MMC5983MA rotation only means something on a board that carried one.
    if config.get("mmc_rot_z_deg") is not None and (flight.records.get("MMC5983MA") or []):
        rows.append(("MMC5983MA rotation about Z", f"{config['mmc_rot_z_deg']:g}°"))
    mag_type = config.get("mag_type")
    if mag_type is not None:
        rows.append(("Magnetometer",
                     f"{MAG_TYPE_NAMES.get(mag_type, f'type {mag_type}')}, "
                     f"{config.get('mag_ut_per_lsb', 0):g} µT/LSB"))
    hg = config.get("hg_bias")
    if hg and any(hg):
        rows.append(("High-G zero offset", ", ".join(f"{v:g}" for v in hg) + " m/s²"))
    quat = (fs or {}).get("b2r_quat") or config.get("b2r_quat")
    if quat:
        rows.append(("Board→rocket quaternion", ", ".join(f"{v:.4f}" for v in quat)))
    return rows


def analyze_system(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="system", title="System Configuration")
    fs, _ = fsd.from_flight(flight)   # the Settings section already reports the warnings

    g: list[dict[str, Any]] = []
    if fs is not None:
        t0 = flight.t0_us
        _group(g, "Firmware", [
            ("Version", fs["fw_git_sha"] + (" (built with uncommitted changes)" if fs["fw_dirty"] else "")),
            ("Settings frame", f"v{fs['version']}"),
            ("Snapshot taken", f"{(fs['time_us'] - t0) / 1e6:.3f} s into the log" if t0 is not None else None),
        ])
        rate = fs["ism6_update_rate_hz"]
        rate_text = None
        if rate is not None:
            rate_text = f"{rate} Hz"
            if fs["imu_rate_dynamic"]:
                rate_text += f", stepping down to {fsd.IMU_RATE_DYNAMIC_POST_HZ} Hz after deployment"
        _group(g, "IMU", [
            ("Low-G accelerometer range", f"±{fs['low_g_fs_g']} g"),
            ("High-G accelerometer range", f"±{fs['high_g_fs_g']} g"),
            ("Gyroscope range", f"±{fs['gyro_fs_dps']} °/s"),
            ("Logging rate", rate_text),
        ])
        _group(g, "Roll controller", [
            ("D-term low-pass", "Off" if fs["d_lpf_hz"] <= 0 else _num(fs["d_lpf_hz"], "Hz")),
            ("Roll-rate setpoint", _num(fs["roll_rate_set_point"], "°/s")),
            ("Gain schedule V_ref", _num(fs["gs_v_ref"], "m/s")),
            ("Gain schedule V_min", _num(fs["gs_v_min"], "m/s")),
            ("Gain schedule scale cap", f"{fs['gs_scale_cap']:.6g}×"),
            ("Profile semantics",
             "Ramp between waypoints" if fs["version"] >= 4
             else "Step to the next waypoint (pre-v4 firmware)"),
        ])
    _group(g, "Sensor mounting", _sensor_mounting_rows(flight, fs))
    if fs is not None and fs["gnss_otp_state"] is not None:
        name = fsd.gnss_otp_name(fs["gnss_otp_state"]) or "?"
        gloss = _OTP_GLOSS.get(name)
        _group(g, "GNSS", [("High-performance clock (OTP)", f"{name} — {gloss}" if gloss else name)])

    if g:
        result.note = ("What the firmware and sensors were running underneath the settings "
                       "above. Nothing here is set in the app.")
    result.groups = g
    return result
