"""How the flight computer was configured for this flight.

Last, because it is reference rather than reading: nobody opens a flight report
to look at settings, but the moment a number above looks wrong the first question
is what the vehicle was told to do. Keeping it here means that question is always
one scroll away and never in the way.

Two sources are merged. The sidecar's `settings` object is what the ground tools
wrote — pyro channels, roll-control profile, servo travel. The log's own config
block is what the firmware reported at boot: full-scale ranges and the sensor
mounting rotations. They overlap on the ranges, and where they do the log wins,
because it is what the sensors were actually running.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from ..flight import Flight
from ..registry import AnalysisResult
from ..render import _flatten_settings

# Results, not settings. The summary card owns these; repeating them here invites
# the two to disagree, which is exactly the confusion this report has been
# working to remove.
_RESULT_KEYS = {"apogee_time_s", "max_altitude_m", "max_speed_mps", "burnout_time_s"}

# Config keys whose meaning is not obvious from the key alone.
_CONFIG_LABELS = {
    "low_g_fs_g": "Low-G accelerometer range (g)",
    "high_g_fs_g": "High-G accelerometer range (g)",
    "gyro_fs_dps": "Gyroscope range (°/s)",
    "ism6_rot_z_deg": "IMU mounting rotation (°)",
    "iis2mdc_rot_z_deg": "Magnetometer mounting rotation (°)",
    "mmc_rot_z_deg": "Magnetometer mounting rotation, old board (°)",
    "hg_bias": "High-G zero offset (m/s²)",
    "b2r_code": "Board-to-rocket frame code",
    "b2r_mode": "Board-to-rocket frame mode",
    "b2r_quat": "Board-to-rocket rotation (quaternion)",
}


def _config_rows(config: dict[str, Any]) -> list[tuple[str, str]]:
    """The sensor configuration the firmware reported, as readable rows.

    Rendered as a table rather than the JSON dump this used to be: it is a flat
    list of scalars and a wall of braces is not easier to read for being literal.
    """
    rows: list[tuple[str, str]] = []
    for key, value in config.items():
        label = _CONFIG_LABELS.get(key, key)
        for _k, text in _flatten_settings(value, label):
            rows.append((_k, text))
    return rows


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="settings", title="Settings Snapshot")
    result.metric_headers = ("Setting", "Value")

    rows: list[tuple[str, str]] = []

    settings = (flight.sidecar or {}).get("settings")
    if isinstance(settings, dict):
        rows.extend(_flatten_settings(
            {k: v for k, v in settings.items() if k not in _RESULT_KEYS}))

    config = flight.config or {}
    if config:
        rows.extend(_config_rows(config))

    if not rows:
        result.warnings.append(
            "No settings were recorded with this flight, so there is nothing to show. "
            "The sidecar .json carries them; a log opened on its own has none."
        )
        return result

    # Later wins, so the log's own config overrides a sidecar value of the same
    # name — it is what the sensors were actually running.
    result.metrics = dict(rows)
    return result
