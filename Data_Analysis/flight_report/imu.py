"""Choosing between the two accelerometers.

The board carries a low-G part (±16 g typical) and a high-G part (±256 g). The
low-G one is the better instrument — finer resolution and less noise — so it is
what we report, and the high-G one is a backstop for when the low-G rails.

Saturation is tested **per axis**, not on the magnitude. Each axis clips at its
own full scale, so a vehicle pulling 20 g along a diagonal can read |a| = 20 g
with no axis anywhere near its rail; testing the magnitude against the full
scale would declare that saturated and needlessly fall back to the coarser
sensor.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

G = 9.80665

# A reading this close to full scale is at the rail, not near it.
_SATURATION_FRACTION = 0.99
# With no declared full scale, a clipped signal shows as a plateau: the same
# extreme value on consecutive samples. A genuine peak is one sample.
_PLATEAU_SAMPLES = 3


def _axes(imu, prefix: str, mask=None) -> Optional[list[np.ndarray]]:
    if not imu or not all(f"{prefix}_{a}" in imu[0] for a in "xyz"):
        return None
    out = []
    for a in "xyz":
        v = get_array(imu, f"{prefix}_{a}")
        out.append(v[mask] if mask is not None else v)
    return out if out[0].size else None


def _declared_full_scale_mps2(sidecar, key: str) -> Optional[float]:
    """Full scale in m/s² from the sidecar's IMU settings, if it recorded one."""
    imu_cfg = ((sidecar or {}).get("settings") or {}).get("imu") or {}
    try:
        return float(imu_cfg[key]) * G
    except (KeyError, TypeError, ValueError):
        return None


def _is_saturated(axes: list[np.ndarray], full_scale: Optional[float]) -> bool:
    for v in axes:
        if not v.size:
            continue
        mag = np.abs(v)
        if full_scale is not None:
            if float(np.max(mag)) >= full_scale * _SATURATION_FRACTION:
                return True
            continue
        # No declared range: look for a flat top at the extreme value.
        peak = float(np.max(mag))
        if peak <= 0:
            continue
        at_peak = np.isclose(mag, peak, rtol=1e-6)
        run = 0
        for hit in at_peak:
            run = run + 1 if hit else 0
            if run >= _PLATEAU_SAMPLES:
                return True
    return False


def accel_magnitude(records, sidecar=None, mask=None) -> tuple[Optional[np.ndarray], str]:
    """|a| in m/s² over `mask`, preferring the low-G part. Returns (mag, source).

    Falls back to the high-G part only when the low-G one saturates within the
    window, and says so in the returned description.
    """
    imu = records.get("ISM6HG256") or []

    low = _axes(imu, "low_acc", mask)
    if low is not None:
        fs = _declared_full_scale_mps2(sidecar, "low_g_fs_g")
        if not _is_saturated(low, fs):
            return np.sqrt(sum(v ** 2 for v in low)), "low-G accelerometer"

    high = _axes(imu, "high_acc", mask)
    if high is not None:
        why = "low-G accelerometer saturated" if low is not None else "no low-G channel"
        return np.sqrt(sum(v ** 2 for v in high)), f"high-G accelerometer ({why})"

    if low is not None:
        # Low-G railed and there is no high-G part: report it, flagged, rather
        # than nothing — an under-read peak beats a blank cell.
        return np.sqrt(sum(v ** 2 for v in low)), "low-G accelerometer (SATURATED — reads low)"

    return None, ""
