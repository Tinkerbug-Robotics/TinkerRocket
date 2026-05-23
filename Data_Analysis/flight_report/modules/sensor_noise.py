"""Sensor-noise module — pad-phase noise statistics for IMU / baro / mag / GNSS.

Validates the sim error models against real on-pad data. Mirrors analyze_sensor_noise.py.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array, pressure_to_altitude  # noqa: E402

from ..flight import Flight
from ..registry import AnalysisResult


def _pad_window(records, t0_us, pre_launch_pad_s=3.0, lead_s=0.5):
    """Pick a 3 s window ending 0.5 s before launch.

    Returns (pad_start_us, pad_end_us) or (None, None) if no usable pad data
    (e.g. log started after launch).
    """
    ns = records.get("NonSensor") or []
    if not ns:
        return None, None
    states = np.array([r.get("rocket_state", -1) for r in ns])
    times = np.array([r["time_us"] for r in ns])
    inflight = np.where(states == 3)[0]
    if inflight.size > 0:
        launch_us = int(times[inflight[0]])
    else:
        launch_us = int(times[-1])
    pad_end = launch_us - int(lead_s * 1e6)
    pad_start = pad_end - int(pre_launch_pad_s * 1e6)
    # If the log started after the pad window we want, give up.
    first_record_us = int(min(times[0], t0_us if t0_us is not None else times[0]))
    if pad_end <= first_record_us:
        return None, None
    return pad_start, pad_end


def _imu_stats(records, pad_start_us, pad_end_us):
    imu = records.get("ISM6HG256") or []
    if not imu:
        return {}
    imu_t = get_array(imu, "time_us")
    mask = (imu_t >= pad_start_us) & (imu_t <= pad_end_us)
    n = int(mask.sum())
    if n < 50:
        return {"imu_pad_samples": n, "imu_note": "too few pad samples"}

    out = {"imu_pad_samples": n}
    dt = np.diff(imu_t[mask]) / 1e6
    if dt.size:
        out["imu_rate_hz"] = round(1.0 / float(np.median(dt)), 1)

    for prefix, label in [("low_acc", "lg_accel"), ("high_acc", "hg_accel"), ("gyro", "gyro")]:
        for ax_ in ("x", "y", "z"):
            arr = get_array(imu, f"{prefix}_{ax_}")[mask]
            if arr.size:
                out[f"{label}_{ax_}_std"] = round(float(np.std(arr)), 4)
    return out


def _baro_stats(records, pad_start_us, pad_end_us):
    baro = records.get("BMP585") or []
    if not baro:
        return {}
    bt = get_array(baro, "time_us")
    mask = (bt >= pad_start_us) & (bt <= pad_end_us)
    n = int(mask.sum())
    out = {"baro_pad_samples": n}
    if n < 20:
        return out
    p = get_array(baro, "pressure_pa")[mask]
    out["baro_pressure_std_pa"] = round(float(np.std(p)), 2)
    # Convert to altitude std
    p0 = float(np.mean(p))
    alts = np.array([pressure_to_altitude(float(pp), p0) for pp in p])
    out["baro_alt_std_m"] = round(float(np.std(alts)), 3)
    return out


def _mag_stats(records, pad_start_us, pad_end_us):
    mag = records.get("MMC5983MA") or []
    if not mag:
        return {}
    mt = get_array(mag, "time_us")
    mask = (mt >= pad_start_us) & (mt <= pad_end_us)
    n = int(mask.sum())
    out = {"mag_pad_samples": n}
    if n < 20:
        return out
    for ax_ in ("x", "y", "z"):
        arr = get_array(mag, f"mag_{ax_}")[mask]
        if arr.size:
            out[f"mag_{ax_}_std_uT"] = round(float(np.std(arr)), 3)
    return out


def _gnss_stats(records, pad_start_us, pad_end_us):
    gnss = records.get("GNSS") or []
    if not gnss:
        return {}
    gt = get_array(gnss, "time_us")
    mask = (gt >= pad_start_us) & (gt <= pad_end_us)
    n = int(mask.sum())
    out = {"gnss_pad_samples": n}
    if n < 5:
        return out
    lat = get_array(gnss, "lat")[mask] if "lat" in gnss[0] else None
    lon = get_array(gnss, "lon")[mask] if "lon" in gnss[0] else None
    alt = get_array(gnss, "alt_m")[mask] if "alt_m" in gnss[0] else None
    if lat is not None and lat.size:
        # ~111 km per degree of latitude
        out["gnss_lat_std_m"] = round(float(np.std(lat)) * 111_000.0, 3)
    if lon is not None and lon.size and lat is not None and lat.size:
        out["gnss_lon_std_m"] = round(
            float(np.std(lon)) * 111_000.0 * float(np.cos(np.deg2rad(np.mean(lat)))), 3)
    if alt is not None and alt.size:
        out["gnss_alt_std_m"] = round(float(np.std(alt)), 3)
    return out


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="sensor_noise", title="Pad-Phase Sensor Noise")
    recs = flight.records
    t0 = flight.t0_us

    pad_start, pad_end = _pad_window(recs, t0)
    if pad_start is None:
        result.warnings.append(
            "Pad window unavailable (no NonSensor records, or log started after launch)."
        )
        return result

    result.metrics["pad_window_s"] = (
        f"[{(pad_start - t0)/1e6:.2f}, {(pad_end - t0)/1e6:.2f}]"
    )
    result.metrics.update(_imu_stats(recs, pad_start, pad_end))
    result.metrics.update(_baro_stats(recs, pad_start, pad_end))
    result.metrics.update(_mag_stats(recs, pad_start, pad_end))
    result.metrics.update(_gnss_stats(recs, pad_start, pad_end))

    return result
