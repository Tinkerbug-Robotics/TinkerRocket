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


def accel_body(records, mask=None) -> tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
    """Body-frame acceleration in m/s², shape (3, n), and which part each sample came from.

    The flight loop's own pick (`firmware_accel_xyz`), sample by sample: the
    low-G part wherever it is clear of its rail, the high-G part only for the
    samples where it is not. `accel_magnitude` switches a whole window to the
    high-G part if one sample in it railed, which is right for a magnitude
    trace that must not under-read, and wrong for a peak: on the 10/24 Piercer
    Mini a 2 ms shock at liftoff railed 18 of 9,969 burn samples, the whole
    burn went to the coarser part, and its single largest sample, 283 G,
    became the motor's acceleration.

    Returns (axes, railed, description); `railed` marks the samples taken from
    the high-G part, or the low-G samples that clipped when there is no high-G
    part to take instead.
    """
    imu = records.get("ISM6HG256") or []
    low = _axes(imu, "low_acc", mask)
    high = _axes(imu, "high_acc", mask)
    if low is None and high is None:
        return None, None, ""
    if low is None:
        n = high[0].size
        return np.vstack(high), np.ones(n, dtype=bool), "high-G accelerometer (no low-G channel)"
    low_a = np.vstack(low)
    if "low_g_near_rail" in imu[0]:
        railed = get_array(imu, "low_g_near_rail").astype(bool)
        if mask is not None:
            railed = railed[mask]
    else:
        railed = np.zeros(low_a.shape[1], dtype=bool)
    if not railed.any():
        return low_a, railed, "low-G accelerometer"
    if high is None:
        return low_a, railed, "low-G accelerometer (at its rail in places, so it reads low there)"
    return (np.where(railed, np.vstack(high), low_a), railed,
            "low-G accelerometer, high-G where it railed")


# A level counts as the motor's only if the rocket held it this long. Shocks
# ring out in a few milliseconds: the 10/24 Piercer Mini's liftoff transient
# swung +275, -93, +22 G inside 2 ms, and the 2026-05-09 Goblin's burst to
# 28 G 30 ms after first motion was over in 8 ms. A thrust curve's fastest
# real feature, an ignition spike, lasts tens of milliseconds. On the 26
# archived flights with a measured burn, the held peak moves by 0.3 G at most
# between 10 and 20 ms holds; a 5 ms hold lets the Goblin's burst through,
# 13.8 G against the 10.3 its motor held.
THRUST_HOLD_S = 0.010

# Output positions per block of the running median, so its working copy stays
# a few megabytes on a long burn at 7 kHz rather than tens of them.
_MEDIAN_BLOCK = 4096


def sustained_peak(t: np.ndarray, x: np.ndarray, hold_s: float = THRUST_HOLD_S
                   ) -> Optional[tuple[float, float]]:
    """(level, time) of the highest level `x` held for at least `hold_s`.

    The peak of a running median over twice `hold_s`: the median of a window
    reaches a level only when at least half the window — `hold_s` of it — is
    at or above it. A spike shorter than that cannot move it however tall it
    is, which a moving mean cannot promise: averaged over 5 ms, the Piercer's
    liftoff shock still reads 22 G against a motor that held 10.5.
    """
    n = x.size
    if n == 0:
        return None
    steps = np.diff(t)
    steps = steps[steps > 0]
    if not steps.size:
        return float(np.median(x)), float(t[n // 2])
    k = int(round(2 * hold_s / float(np.median(steps))))
    k = max(3, k | 1)
    if n < k:
        return float(np.median(x)), float(t[n // 2])
    windows = np.lib.stride_tricks.sliding_window_view(x, k)
    best, at = -np.inf, 0
    for lo in range(0, windows.shape[0], _MEDIAN_BLOCK):
        med = np.median(windows[lo:lo + _MEDIAN_BLOCK], axis=1)
        i = int(np.argmax(med))
        if med[i] > best:
            best, at = float(med[i]), lo + i
    return best, float(t[at + k // 2])
