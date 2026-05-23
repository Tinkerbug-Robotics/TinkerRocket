"""Shared EKF replay engine for binary flight logs.

Feeds parsed IMU/GNSS/baro/mag records through the firmware EKF
(`tinkerrocket_sim._ekf`) and returns time series of EKF state plus
GNSS truth, matching the same data flow the rocket uses in flight.

Both `replay_flight_ekf.py` (visualization) and `landing_predictor.py`
(landing prediction) build on top of this so any change to the replay
chain stays in one place.  The body of `replay_binary` was lifted from
the inner loop of `replay_flight_ekf.py:replay()`; behaviors preserved
include the FLU→FRD axis conversion, low-g/high-g saturation switch,
GNSS fix de-duplication, baro reference-pressure capture, and
phase-gated `use_ahrs_acc`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from tinkerrocket_sim._ekf import (GpsInsEKF, IMUData, GNSSDataLLA,
                                    MagData, BaroData)

G_MS2 = 9.80665
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
R_EARTH_M = 6378137.0

LOW_G_SAT_THRESH = 15.5 * G_MS2  # ±15.5g — matches ISM6 ±16g range minus margin


@dataclass
class FlightPhases:
    t0_us: int                       # first IMU sample timestamp
    boost_start_us: Optional[int] = None
    boost_end_us: Optional[int] = None
    apogee_us: Optional[int] = None


@dataclass
class EkfSeries:
    """EKF outputs logged at ~100 Hz."""
    t_us: np.ndarray             # (N,)
    lat_rad: np.ndarray
    lon_rad: np.ndarray
    alt_m: np.ndarray            # MSL
    vn: np.ndarray
    ve: np.ndarray
    vd: np.ndarray
    q: np.ndarray                # (N, 4) w,x,y,z
    cov_pos: np.ndarray          # (N, 3) per-axis variance
    cov_vel: np.ndarray


@dataclass
class GnssSeries:
    """Valid GNSS fixes (>=4 sats), used as truth."""
    t_us: np.ndarray
    lat_deg: np.ndarray
    lon_deg: np.ndarray
    alt_m: np.ndarray
    vn: np.ndarray
    ve: np.ndarray
    vu: np.ndarray


@dataclass
class BaroSeries:
    t_us: np.ndarray
    alt_msl_m: np.ndarray


@dataclass
class LaunchRef:
    """Local-ENU reference from first valid GNSS fix."""
    lat_deg: float
    lon_deg: float
    alt_m: float


@dataclass
class ReplayResult:
    phases: FlightPhases
    ekf: EkfSeries
    gnss: GnssSeries
    baro: BaroSeries
    launch_ref: LaunchRef


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def build_event_list(records):
    """Merge sensor records into a single time-sorted event list."""
    events = []
    for r in records["ISM6HG256"]:
        events.append((r["time_us"], "imu", r))
    for r in records["GNSS"]:
        events.append((r["time_us"], "gnss", r))
    for r in records["BMP585"]:
        events.append((r["time_us"], "baro", r))
    for r in records["MMC5983MA"]:
        events.append((r["time_us"], "mag", r))
    events.sort(key=lambda e: e[0])
    return events


def detect_flight_phases(records) -> FlightPhases:
    """Boost start/end from accel magnitude; apogee from GNSS altitude peak."""
    imu = records["ISM6HG256"]
    if not imu:
        return FlightPhases(t0_us=0)

    t0_us = imu[0]["time_us"]

    # Sample accel magnitude (downsample for speed)
    step = max(1, len(imu) // max(1, len(imu) // 20))
    accel_mag = []
    for i in range(0, len(imu), step):
        r = imu[i]
        a = math.sqrt(r["low_acc_x"]**2 + r["low_acc_y"]**2 + r["low_acc_z"]**2)
        accel_mag.append((r["time_us"], a))

    boost_start = None
    boost_end = None
    for t_us, a in accel_mag:
        if a > 3.0 * G_MS2 and boost_start is None:
            boost_start = t_us
        if boost_start is not None and a < 2.0 * G_MS2 and boost_end is None:
            boost_end = t_us

    apogee_us = None
    gnss = records["GNSS"]
    if gnss and boost_start:
        gnss_times = np.array([g["time_us"] for g in gnss])
        gnss_alt = np.array([g["alt_m"] for g in gnss])
        flight_mask = gnss_times > boost_start
        if flight_mask.any():
            apogee_us = int(gnss_times[flight_mask][np.argmax(gnss_alt[flight_mask])])
        elif boost_end:
            apogee_us = boost_end + 5_000_000

    return FlightPhases(t0_us=t0_us,
                        boost_start_us=boost_start,
                        boost_end_us=boost_end,
                        apogee_us=apogee_us)


def _pad_heading_init(imu_d) -> tuple[float, float, float, float]:
    """Quaternion from accel-only pad attitude, yaw=0 (heading unknown)."""
    g_mag = math.sqrt(imu_d.acc_x**2 + imu_d.acc_y**2 + imu_d.acc_z**2)
    if g_mag < 0.1:
        return (1.0, 0.0, 0.0, 0.0)
    pitch_rad = math.asin(max(-1.0, min(1.0, imu_d.acc_x / g_mag)))
    if abs(pitch_rad) > math.radians(80.0):
        roll_rad = 0.0
    else:
        roll_rad = math.atan2(-imu_d.acc_y, -imu_d.acc_z)
    yaw_rad = 0.0
    cy, sy = math.cos(yaw_rad/2), math.sin(yaw_rad/2)
    cp, sp = math.cos(pitch_rad/2), math.sin(pitch_rad/2)
    cr, sr = math.cos(roll_rad/2), math.sin(roll_rad/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qw, qx, qy, qz)


# ---------------------------------------------------------------------------
# Main replay
# ---------------------------------------------------------------------------

def replay_binary(records, log_decimation: int = 20,
                  verbose: bool = False) -> ReplayResult:
    """Run the firmware EKF over the parsed binary log records.

    Causal: output at time t only depends on sensor data <= t.

    Parameters
    ----------
    records : dict
        Output of `plot_flight_data_mini.parse_binary_file`.
    log_decimation : int
        Keep every Nth IMU sample in the EKF time series (1 = full rate ~2 kHz,
        20 ≈ 100 Hz).
    verbose : bool
        Print init / progress info.
    """
    phases = detect_flight_phases(records)
    if verbose and phases.boost_start_us:
        bs = (phases.boost_start_us - phases.t0_us) / 1e6
        be = (phases.boost_end_us - phases.t0_us) / 1e6 if phases.boost_end_us else None
        ap = (phases.apogee_us - phases.t0_us) / 1e6 if phases.apogee_us else None
        print(f"  Boost: {bs:.2f}s - {be:.2f}s   Apogee: {ap}")

    events = build_event_list(records)
    if verbose:
        print(f"  Total events: {len(events):,}")

    # ---- EKF state ----
    ekf = GpsInsEKF()
    ekf_initialized = False
    latest_gnss = None
    latest_mag = None
    gnss_counter = 0
    last_gnss_time_us = 0

    baro_ref_pa = None
    baro_samples_for_ref = []
    baro_alt_offset = 0.0
    baro_alt_offset_set = False

    # ---- Output buffers ----
    log_t = []
    log_lat = []; log_lon = []; log_alt = []
    log_vn = []; log_ve = []; log_vd = []
    log_q = []
    log_cov_pos = []; log_cov_vel = []

    gnss_t = []; gnss_lat = []; gnss_lon = []; gnss_alt = []
    gnss_vn = []; gnss_ve = []; gnss_vu = []

    baro_t = []; baro_alt = []

    n_imu = 0

    for time_us, etype, rec in events:
        if etype == "gnss":
            has_fix = rec.get("num_sats", 0) >= 4
            if has_fix:
                gnss_t.append(time_us)
                gnss_lat.append(rec["lat"])
                gnss_lon.append(rec["lon"])
                gnss_alt.append(rec["alt_m"])
                gnss_vn.append(rec["vel_n"])
                gnss_ve.append(rec["vel_e"])
                gnss_vu.append(rec["vel_u"])
            # De-dup: only count as new when second/milli changes
            if has_fix and (latest_gnss is None or
                            rec["second"] != latest_gnss["second"] or
                            rec["milli_sec"] != latest_gnss["milli_sec"]):
                latest_gnss = rec
                gnss_counter += 1

        elif etype == "mag":
            latest_mag = rec

        elif etype == "baro":
            if baro_ref_pa is None:
                baro_samples_for_ref.append(rec["pressure_pa"])
                if len(baro_samples_for_ref) >= 20:
                    baro_ref_pa = float(np.mean(baro_samples_for_ref))
                continue
            if not baro_alt_offset_set and latest_gnss is not None:
                baro_alt_offset = latest_gnss["alt_m"]
                baro_alt_offset_set = True
            alt = _pressure_to_altitude(rec["pressure_pa"], baro_ref_pa) + baro_alt_offset
            baro_t.append(time_us)
            baro_alt.append(alt)
            if ekf_initialized:
                bd = BaroData(); bd.time_us = time_us; bd.altitude_m = alt
                ekf.baro_meas_update(bd)

        elif etype == "imu":
            if latest_gnss is None:
                continue

            # Pick accel source (low-g/high-g saturation switch)
            lx, ly, lz = rec["low_acc_x"], rec["low_acc_y"], rec["low_acc_z"]
            if (abs(lx) > LOW_G_SAT_THRESH or abs(ly) > LOW_G_SAT_THRESH or
                    abs(lz) > LOW_G_SAT_THRESH):
                ax, ay, az = rec["high_acc_x"], rec["high_acc_y"], rec["high_acc_z"]
            else:
                ax, ay, az = lx, ly, lz

            # Board (FLU) → EKF body (FRD): Y and Z flip
            imu_d = IMUData()
            imu_d.time_us = time_us
            imu_d.acc_x = ax
            imu_d.acc_y = -ay
            imu_d.acc_z = -az
            imu_d.gyro_x = rec["gyro_x"]
            imu_d.gyro_y = -rec["gyro_y"]
            imu_d.gyro_z = -rec["gyro_z"]

            gnss_d = GNSSDataLLA()
            gnss_d.time_us = gnss_counter
            gnss_d.lat_rad = latest_gnss["lat"] * DEG2RAD
            gnss_d.lon_rad = latest_gnss["lon"] * DEG2RAD
            gnss_d.alt_m = latest_gnss["alt_m"]
            gnss_d.vel_n_mps = latest_gnss["vel_n"]
            gnss_d.vel_e_mps = latest_gnss["vel_e"]
            gnss_d.vel_d_mps = -latest_gnss["vel_u"]

            mag_d = MagData()
            if latest_mag is not None:
                mag_d.time_us = latest_mag["time_us"]
                mag_d.mag_x = latest_mag["mag_x"]
                mag_d.mag_y = -latest_mag["mag_y"]
                mag_d.mag_z = -latest_mag["mag_z"]

            if not ekf_initialized:
                ekf.init_lla(imu_d, gnss_d, mag_d)
                qw, qx, qy, qz = _pad_heading_init(imu_d)
                ekf.set_quaternion(qw, qx, qy, qz)
                ekf_initialized = True
                if verbose:
                    print(f"  EKF init at t={(time_us - phases.t0_us)/1e6:.2f}s")
                continue

            # Phase-gated use_ahrs_acc (off during boost+coast)
            if phases.boost_start_us and phases.boost_end_us and phases.apogee_us:
                in_bc = (phases.boost_start_us <= time_us <= phases.apogee_us)
                use_ahrs_acc = not in_bc
            else:
                use_ahrs_acc = True

            ekf.update_lla(use_ahrs_acc, imu_d, gnss_d, mag_d)
            n_imu += 1

            if gnss_d.time_us != last_gnss_time_us:
                last_gnss_time_us = gnss_d.time_us

            if n_imu % log_decimation == 0:
                pos = ekf.get_position()
                vel = ekf.get_velocity()
                q = ekf.get_quaternion()
                cp_ = ekf.get_cov_pos()
                cv_ = ekf.get_cov_vel()
                log_t.append(time_us)
                log_lat.append(pos[0]); log_lon.append(pos[1]); log_alt.append(pos[2])
                log_vn.append(vel[0]); log_ve.append(vel[1]); log_vd.append(vel[2])
                log_q.append(q)
                log_cov_pos.append(cp_); log_cov_vel.append(cv_)

    # ---- Pack results ----
    ekf_series = EkfSeries(
        t_us=np.asarray(log_t, dtype=np.int64),
        lat_rad=np.asarray(log_lat),
        lon_rad=np.asarray(log_lon),
        alt_m=np.asarray(log_alt),
        vn=np.asarray(log_vn),
        ve=np.asarray(log_ve),
        vd=np.asarray(log_vd),
        q=np.asarray(log_q),
        cov_pos=np.asarray(log_cov_pos),
        cov_vel=np.asarray(log_cov_vel),
    )
    gnss_series = GnssSeries(
        t_us=np.asarray(gnss_t, dtype=np.int64),
        lat_deg=np.asarray(gnss_lat),
        lon_deg=np.asarray(gnss_lon),
        alt_m=np.asarray(gnss_alt),
        vn=np.asarray(gnss_vn),
        ve=np.asarray(gnss_ve),
        vu=np.asarray(gnss_vu),
    )
    baro_series = BaroSeries(t_us=np.asarray(baro_t, dtype=np.int64),
                             alt_msl_m=np.asarray(baro_alt))

    # Launch ref: first valid GNSS fix
    if len(gnss_series.lat_deg) == 0:
        raise RuntimeError("No valid GNSS fixes — cannot establish launch reference")
    launch_ref = LaunchRef(lat_deg=float(gnss_series.lat_deg[0]),
                           lon_deg=float(gnss_series.lon_deg[0]),
                           alt_m=float(gnss_series.alt_m[0]))

    return ReplayResult(phases=phases, ekf=ekf_series, gnss=gnss_series,
                        baro=baro_series, launch_ref=launch_ref)


def _pressure_to_altitude(p_pa, p0):
    if p_pa <= 0 or p0 <= 0:
        return 0.0
    return 44330.0 * (1.0 - (p_pa / p0) ** (1.0 / 5.255))


# ---------------------------------------------------------------------------
# Frame conversions
# ---------------------------------------------------------------------------

def lla_rad_to_enu_m(lat_rad, lon_rad, alt_m, ref: LaunchRef):
    """Flat-earth ENU relative to launch reference.

    Matches the approximation used in replay_flight_ekf.py for short-range
    rocket flights (<~10 km from launch).  At higher latitudes or larger
    distances, replace with a proper ECEF→ENU rotation.
    """
    ref_lat_rad = ref.lat_deg * DEG2RAD
    ref_lon_rad = ref.lon_deg * DEG2RAD
    n = (lat_rad - ref_lat_rad) * R_EARTH_M
    e = (lon_rad - ref_lon_rad) * R_EARTH_M * math.cos(ref_lat_rad)
    u = alt_m - ref.alt_m
    return e, n, u


def enu_m_to_lla_deg(e_m, n_m, u_m, ref: LaunchRef):
    """Inverse of lla_rad_to_enu_m; returns lat/lon in degrees, alt in m."""
    ref_lat_rad = ref.lat_deg * DEG2RAD
    lat_rad = ref_lat_rad + n_m / R_EARTH_M
    lon_rad = (ref.lon_deg * DEG2RAD) + e_m / (R_EARTH_M * math.cos(ref_lat_rad))
    return lat_rad * RAD2DEG, lon_rad * RAD2DEG, ref.alt_m + u_m
