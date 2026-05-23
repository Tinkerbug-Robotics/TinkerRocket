"""Launch detection module.

Reports whether/when the launch flag fired in NonSensor records, and shows the
accel norm + baro altitude rate around the launch window so manual review is
possible when the firmware missed launch.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array, pressure_to_altitude  # noqa: E402

from ..flight import Flight
from ..registry import AnalysisResult

G_MS2 = 9.80665
NSF_LAUNCH = 1 << 3


def _accel_norm(imu_recs):
    """sqrt(ax² + ay² + az²) using low-G unless saturated (then high-G)."""
    LOW_FS = 16.0
    SAT = (LOW_FS - 0.5) * G_MS2
    ax = get_array(imu_recs, "low_acc_x")
    ay = get_array(imu_recs, "low_acc_y")
    az = get_array(imu_recs, "low_acc_z")
    hx = get_array(imu_recs, "high_acc_x")
    hy = get_array(imu_recs, "high_acc_y")
    hz = get_array(imu_recs, "high_acc_z")
    sat = (np.abs(ax) > SAT) | (np.abs(ay) > SAT) | (np.abs(az) > SAT)
    nx = np.where(sat, hx, ax)
    ny = np.where(sat, hy, ay)
    nz = np.where(sat, hz, az)
    return np.sqrt(nx * nx + ny * ny + nz * nz)


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="launch_detection", title="Launch Detection")
    recs = flight.records
    t0 = flight.t0_us or 0

    ns = recs.get("NonSensor") or []
    imu = recs.get("ISM6HG256") or []
    baro = recs.get("BMP585") or []

    # Firmware launch flag
    launch_us = None
    for r in ns:
        if r.get("flags", 0) & NSF_LAUNCH:
            launch_us = r["time_us"]
            break

    # Inferred launch from accel norm > 2.5g sustained for >50ms (heuristic only)
    inferred_us = None
    if imu:
        imu_t = get_array(imu, "time_us")
        a = _accel_norm(imu)
        above = a > 2.5 * G_MS2
        if above.any():
            # Find first run of >= 50 ms above threshold
            idx = np.where(above)[0]
            for i in idx:
                # window of next 50 ms
                window_end = imu_t[i] + 50_000
                seg = a[(imu_t >= imu_t[i]) & (imu_t <= window_end)]
                if seg.size > 5 and float(np.min(seg)) > 2.0 * G_MS2:
                    inferred_us = int(imu_t[i])
                    break

    result.metrics = {
        "firmware_launch_t_s": (launch_us - t0) / 1e6 if launch_us is not None else "—",
        "inferred_launch_t_s": (inferred_us - t0) / 1e6 if inferred_us is not None else "—",
    }
    if launch_us is None and inferred_us is not None:
        result.warnings.append(
            f"Firmware did NOT set launch flag, but accel suggests launch at "
            f"T+{(inferred_us - t0)/1e6:.2f}s."
        )
    if launch_us is not None and inferred_us is not None:
        dt = (launch_us - inferred_us) / 1e6
        result.metrics["firmware_vs_inferred_dt_s"] = round(dt, 3)

    # Figure: accel norm + baro alt rate around launch window
    if imu and baro:
        imu_t = (get_array(imu, "time_us") - t0) / 1e6
        an = _accel_norm(imu) / G_MS2  # in g

        baro_t = (get_array(baro, "time_us") - t0) / 1e6
        p = get_array(baro, "pressure_pa")
        if p.size > 50:
            p0 = float(np.mean(p[:50]))
            alt = np.array([pressure_to_altitude(float(pp), p0) for pp in p])
            # Smooth derivative: central diff with light filter
            dt_s = np.diff(baro_t)
            dt_s = np.where(dt_s <= 0, 1e-3, dt_s)
            alt_rate = np.concatenate([[0], np.diff(alt) / dt_s])

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
            ax1.plot(imu_t, an, lw=0.7, color="tab:blue")
            ax1.set_ylabel("|accel| (g)")
            ax1.axhline(2.5, color="orange", linestyle=":", alpha=0.6, label="2.5 g")
            ax1.grid(True, alpha=0.3)
            ax1.set_title("Launch-window kinematics")
            ax1.legend(loc="upper right", fontsize=8)

            ax2.plot(baro_t, alt_rate, lw=0.7, color="tab:green")
            ax2.set_ylabel("baro alt rate (m/s)")
            ax2.set_xlabel("Time (s)")
            ax2.grid(True, alpha=0.3)

            for ax in (ax1, ax2):
                if launch_us is not None:
                    ax.axvline((launch_us - t0) / 1e6, color="red", linestyle="--",
                               alpha=0.6, label="fw launch")
                if inferred_us is not None:
                    ax.axvline((inferred_us - t0) / 1e6, color="purple", linestyle=":",
                               alpha=0.6, label="inferred")

            fig.tight_layout()
            result.figures.append(fig)

    return result
