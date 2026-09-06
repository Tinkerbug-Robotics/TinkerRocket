"""Decode the firmware's FlightSettingsData frame: the log's own record of settings.

The flight computer writes one at the PRELAUNCH→INFLIGHT transition (message
0xE1; layout in tinkerrocket-idf/components/TR_RocketComputerTypes/
RocketComputerTypes.h). The apps decode this same frame into the sidecar
.json's `settings` object. The report reads the binary directly, so what it
shows does not depend on which app version wrote the sidecar, or on a sidecar
existing at all — and a log without the frame has no sidecar settings either,
because the frame is where the app got them.

Versioned by appended tails: each version added fields at the end and never
moved one. The decoder reads what the frame's length covers and leaves the rest
None. Length rather than version gates each tail, because v4 changed the
roll-profile semantics without changing the layout, and a frame is exactly as
long as the firmware that wrote it.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path
from typing import Any, Optional

_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import GNSS_OTP_STATES  # noqa: E402

# Flag bit positions (FlightSettingsData::F_*).
F_USE_ANGLE_CONTROL = 0
F_GAIN_SCHEDULE = 1
F_GUIDANCE = 2
F_SERVO_ENABLED = 3
F_FW_DIRTY = 4
F_SOUNDS = 5
F_GUIDANCE_STATION_KEEP = 6
F_IMU_RATE_DYNAMIC = 7

# TR_Orientation's 24 codes: six nose axes, each at four fin clockings.
ORIENT_NAMES = [
    f"{axis}{'' if clock == 0 else f' r{clock}'}"
    for axis in ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
    for clock in (0, 90, 180, 270)
]
ORIENT_MODES = {0: "default", 1: "manual", 2: "auto_snap", 3: "auto_exact"}
CAMERA_TYPES = {0: "none", 1: "gopro", 2: "runcam"}
PYRO_MODES = {0: "time_after_apogee", 1: "altitude_on_descent"}
GUIDANCE_TARGET_SOURCES = {0: "overhead", 1: "geodetic_point", 2: "profile_point"}

# The dynamic logging rate steps down to this after deployment
# (IMU_RATE_DYNAMIC_POST_HZ in the firmware).
IMU_RATE_DYNAMIC_POST_HZ = 960

# Fixed head, bytes 0..76: time, version, flags, delay, the six rate-PID
# floats, the two outer-loop floats, the three gain-schedule floats, the rate
# setpoint, the IMU full scales, servo trim/timing and the camera type.
_HEAD = struct.Struct("<IBBH6f2f3ffBHH4hhhhB")
_PYRO = struct.Struct("<BBf")      # ×4 at 76
_SHA_AT = 100                      # char[12], NUL-terminated
_PROFILE_AT = 112                  # uint8 num_waypoints, 3 pad, then waypoints at 116
_WAYPOINT = struct.Struct("<ffB")  # ×8 at 116
_MAX_WAYPOINTS = 8
_B2R = struct.Struct("<BBh4h")     # v2 tail at 188
_FIN = struct.Struct("<2f")        # v3 tail at 200
_RATE = struct.Struct("<H")        # v5 tail at 208
_GUID = struct.Struct("<2fB")      # v6 tail at 210
_OTP = struct.Struct("<B")         # v7 tail at 219
_GATE = struct.Struct("<H")        # v8 tail at 220

MIN_LENGTH = 188
assert _HEAD.size == 76
assert _PROFILE_AT + 4 + _MAX_WAYPOINTS * _WAYPOINT.size == MIN_LENGTH


def decode(payload: bytes) -> dict[str, Any]:
    """One frame as a flat dict keyed by the struct's field names.

    Flags come out as booleans, the four pyro channels as a list of dicts,
    the roll profile as a list of waypoints, and every field a shorter frame
    does not carry as None.
    """
    if len(payload) < MIN_LENGTH:
        raise ValueError(f"FlightSettings frame is {len(payload)} bytes; needs {MIN_LENGTH}")

    h = _HEAD.unpack_from(payload, 0)
    flags = h[2]

    def flag(bit: int) -> bool:
        return bool(flags & (1 << bit))

    d: dict[str, Any] = {
        "time_us": h[0],
        "version": h[1],
        "flags": flags,
        "use_angle_control": flag(F_USE_ANGLE_CONTROL),
        "gain_schedule_enabled": flag(F_GAIN_SCHEDULE),
        "guidance_enabled": flag(F_GUIDANCE),
        "servo_enabled": flag(F_SERVO_ENABLED),
        "fw_dirty": flag(F_FW_DIRTY),
        "sounds_enabled": flag(F_SOUNDS),
        "guidance_station_keep": flag(F_GUIDANCE_STATION_KEEP),
        "imu_rate_dynamic": flag(F_IMU_RATE_DYNAMIC),
        "roll_delay_ms": h[3],
        "kp": h[4], "ki": h[5], "kd": h[6],
        "d_lpf_hz": h[7],
        "min_cmd_deg": h[8], "max_cmd_deg": h[9],
        "kp_angle": h[10], "kp_angle_rate_cap_dps": h[11],
        "gs_v_ref": h[12], "gs_v_min": h[13], "gs_scale_cap": h[14],
        "roll_rate_set_point": h[15],
        "low_g_fs_g": h[16], "high_g_fs_g": h[17], "gyro_fs_dps": h[18],
        "servo_bias_us": list(h[19:23]),
        "servo_hz": h[23], "servo_min_us": h[24], "servo_max_us": h[25],
        "camera_type": h[26],
    }

    pyro = []
    for i in range(4):
        enabled, mode, value = _PYRO.unpack_from(payload, 76 + i * _PYRO.size)
        pyro.append({"enabled": bool(enabled), "mode": mode, "value": value})
    d["pyro"] = pyro

    d["fw_git_sha"] = payload[_SHA_AT:_SHA_AT + 12].split(b"\0", 1)[0].decode("ascii", "replace")

    n = payload[_PROFILE_AT]
    d["num_waypoints"] = n
    d["waypoints"] = [
        dict(zip(("time_s", "angle_deg", "mode"),
                 _WAYPOINT.unpack_from(payload, _PROFILE_AT + 4 + i * _WAYPOINT.size)))
        for i in range(min(n, _MAX_WAYPOINTS))
    ]

    # Tails, each present only when the frame reaches it.
    d.update({
        "b2r_code": None, "b2r_mode": None, "b2r_residual_deg": None, "b2r_quat": None,
        "fin_min_deg": None, "fin_max_deg": None,
        "ism6_update_rate_hz": None,
        "guid_tgt_e_m": None, "guid_tgt_n_m": None, "guid_tgt_src": None,
        "gnss_otp_state": None,
        "roll_min_speed_mps": None,
    })
    if len(payload) >= 200:
        code, mode, residual_cdeg, *q = _B2R.unpack_from(payload, 188)
        d["b2r_code"], d["b2r_mode"] = code, mode
        d["b2r_residual_deg"] = residual_cdeg / 100.0
        d["b2r_quat"] = [v / 10000.0 for v in q]
    if len(payload) >= 208:
        d["fin_min_deg"], d["fin_max_deg"] = _FIN.unpack_from(payload, 200)
    if len(payload) >= 210:
        d["ism6_update_rate_hz"] = _RATE.unpack_from(payload, 208)[0]
    if len(payload) >= 219:
        d["guid_tgt_e_m"], d["guid_tgt_n_m"], d["guid_tgt_src"] = _GUID.unpack_from(payload, 210)
    if len(payload) >= 220:
        d["gnss_otp_state"] = _OTP.unpack_from(payload, 219)[0]
    if len(payload) >= 222:
        d["roll_min_speed_mps"] = _GATE.unpack_from(payload, 220)[0] / 10.0
    return d


def orientation_name(code: Optional[int]) -> str:
    if code is None or not 0 <= code < len(ORIENT_NAMES):
        return "?"
    return ORIENT_NAMES[code]


def gnss_otp_name(state: Optional[int]) -> Optional[str]:
    if state is None:
        return None
    return GNSS_OTP_STATES.get(state, f"?{state}")


def from_flight(flight) -> tuple[Optional[dict[str, Any]], list[str]]:
    """The settings the flight flew with, and any warnings about them.

    A log carries the frame many times. The flight computer writes a copy
    every 5 s while it is not in flight — on the pad, and again after landing
    — then three at launch plus 0, 100 and 200 ms so a dropped frame cannot
    lose it, and one more at launch plus 5 s, clear of the logging-activation
    rush that ate all three launch copies on one early flight.

    The one to show is the first written after launch: a pad copy can predate
    a setting changed on the pad seconds before the button, and the launch
    copy is what the vehicle flew. With no launch in the log (a bench
    recording), the last copy is the current setting. Copies from launch on
    are compared against the chosen one, and a field that differs is worth a
    warning because it means the vehicle changed its own mind about a
    setting. Pad copies are not compared — a flyer editing on the pad is not
    the vehicle changing its mind. The one expected difference is the IMU
    logging rate under the dynamic setting: the landing copy records the
    stepped-down rate, which is the setting working as designed.
    """
    from .events import _flag_time

    frames = (flight.config or {}).get("flight_settings_frames") or []
    decoded = []
    warnings: list[str] = []
    for i, raw in enumerate(frames):
        try:
            decoded.append(decode(raw))
        except ValueError as e:
            warnings.append(f"Settings frame {i + 1} could not be decoded: {e}")
    if not decoded:
        return None, warnings

    launch_s = _flag_time(flight.records, flight.t0_us, "launch") if flight.t0_us is not None else None
    if launch_s is None:
        chosen_at = len(decoded) - 1
    else:
        launch_us = flight.t0_us + launch_s * 1e6
        after = [i for i, d in enumerate(decoded) if d["time_us"] >= launch_us]
        chosen_at = after[0] if after else len(decoded) - 1
    chosen = decoded[chosen_at]

    ignore = {"time_us"}
    if chosen["imu_rate_dynamic"]:
        ignore.add("ism6_update_rate_hz")
    changed: list[str] = []
    for later in decoded[chosen_at + 1:]:
        for key, value in chosen.items():
            if key in ignore:
                continue
            if later.get(key) != value and key not in changed:
                changed.append(key)
    if changed:
        warnings.append(
            f"The log carries {len(decoded) - chosen_at} settings snapshots from launch on "
            f"and they disagree on {', '.join(changed)}; the launch copy is shown."
        )
    return chosen, warnings
