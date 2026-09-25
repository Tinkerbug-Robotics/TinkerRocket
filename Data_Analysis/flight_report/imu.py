"""Choosing between the two accelerometers.

The board carries a low-G part (±16 g typical) and a high-G part (±256 g). The
low-G one is the better instrument — finer resolution and less noise — so it is
what we report, and the high-G one is a backstop for when the low-G rails.

The verdict is the flight computer's own: the parser's `low_g_near_rail`, set
when any low-G SENSOR axis is past full scale less 0.5 g in raw counts (#1191).
It cannot be taken from the body-frame values. The chip sits at 45° to the
thrust axis, so a boost can read 20 g on body X with no sensor axis near its
rail, while one railed sensor axis reads only 0.71× on two body axes.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

G = 9.80665


def _axes(imu, prefix: str, mask=None) -> Optional[list[np.ndarray]]:
    if not imu or not all(f"{prefix}_{a}" in imu[0] for a in "xyz"):
        return None
    out = []
    for a in "xyz":
        v = get_array(imu, f"{prefix}_{a}")
        out.append(v[mask] if mask is not None else v)
    return out if out[0].size else None


def _near_rail(imu, mask=None) -> bool:
    """Did any low-G sensor axis reach the flight computer's switch bar?"""
    flags = get_array(imu, "low_g_near_rail").astype(bool)
    if mask is not None:
        flags = flags[mask]
    return bool(flags.any())


def accel_magnitude(records, mask=None) -> tuple[Optional[np.ndarray], str]:
    """|a| in m/s² over `mask`, preferring the low-G part. Returns (mag, source).

    Falls back to the high-G part only when the low-G one reaches its rail
    within the window, and says so in the returned description.
    """
    imu = records.get("ISM6HG256") or []

    low = _axes(imu, "low_acc", mask)
    if low is not None and not _near_rail(imu, mask):
        return np.sqrt(sum(v ** 2 for v in low)), "low-G accelerometer"

    high = _axes(imu, "high_acc", mask)
    if high is not None:
        why = "low-G accelerometer at its rail" if low is not None else "no low-G channel"
        return np.sqrt(sum(v ** 2 for v in high)), f"high-G accelerometer ({why})"

    if low is not None:
        # Low-G railed and there is no high-G part: report it, flagged, rather
        # than nothing — an under-read peak beats a blank cell.
        return np.sqrt(sum(v ** 2 for v in low)), "low-G accelerometer (SATURATED — reads low)"

    return None, ""
