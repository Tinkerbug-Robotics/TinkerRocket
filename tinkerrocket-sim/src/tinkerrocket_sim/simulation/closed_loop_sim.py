"""Closed-loop simulation engine.

Multi-rate scheduling:
    Physics:  10 kHz (0.1 ms timestep)
    IMU:      1200 Hz
    Baro:     500 Hz
    Mag:      1000 Hz
    GNSS:     25 Hz
    Control:  at IMU rate (1200 Hz)

Pipeline per IMU tick:
    1. Run physics to advance to current sensor time
    2. Sample sensors
    3. Feed sensors to EKF
    4. PID controller reads EKF roll rate estimate
    5. PID outputs fin tab deflection command
    6. Actuator model rate-limits the command
"""
import math
import numpy as np
import pandas as pd
from dataclasses import dataclass, field
from typing import Optional

from ..physics.sixdof import (SixDOF, SimState, quat_to_dcm, quat_to_euler,
                               quat_normalize, quat_enu_flu_to_ned_frd)
from ..physics import atmosphere as atm
from ..sensors.imu_model import IMUModel
from ..sensors.baro_model import BaroModel
from ..sensors.mag_model import MagModel
from ..sensors.gnss_model import GNSSModel
from ..control.controller import RollController


@dataclass
class SimConfig:
    """Closed-loop simulation configuration."""
    # Timing
    physics_dt: float = 1e-4        # 10 kHz
    imu_rate: float = 1200.0
    baro_rate: float = 500.0
    mag_rate: float = 1000.0
    gnss_rate: float = 25.0
    duration: float = 30.0          # max sim time (after launch)
    pad_time: float = 0.0           # pre-launch pad warmup (s) for EKF convergence

    # Launch
    launch_angle_deg: float = 85.0
    heading_deg: float = 0.0

    # Control — flight-proven gains from Rolly Poly IV (2026-03-08)
    pid_kp: float = 0.04
    pid_ki: float = 0.001
    pid_kd: float = 0.0003
    roll_setpoint_dps: float = 0.0  # desired roll rate in deg/s (constant mode)
    control_enabled: bool = True

    # Roll angle profile: list of (time_s, angle_deg) waypoints, or None
    # When set, a cascaded controller tracks angle → rate → fin tab
    roll_profile: Optional[list] = None
    kp_angle: float = 5.0          # outer loop gain: (deg/s) per (deg error)

    # Gain scheduling — V_ref=50 gives 1.73x at 38 m/s, 0.51x at 70 m/s
    gain_V_ref: float = 50.0
    gain_V_min: float = 25.0
    gain_max_scale: float = 3.0

    # Actuator limits — PTK 7308 at 8.2V: 923 deg/s slew, +/-20 deg
    deflection_min: float = -20.0   # deg
    deflection_max: float = 20.0    # deg
    servo_rate_limit: float = 923.0 # deg/s

    # Wind (constant ENU, m/s)
    wind_speed: float = 0.0             # m/s
    wind_direction_deg: float = 0.0     # direction wind comes FROM (0=North, 90=East)

    # Launch site (for GNSS)
    ref_lat_deg: float = 38.0
    ref_lon_deg: float = -122.0
    ref_alt_m: float = 0.0

    # Sensor aiding (set False to test pure IMU dead-reckoning)
    enable_gnss_updates: bool = True
    enable_baro_updates: bool = True
    enable_mag_updates: bool = True

    # Truth injection: set the EKF quaternion to the true orientation at ignition
    # (removes AHRS convergence error, isolates pure IMU integration drift)
    inject_truth_orientation_at_ignition: bool = False

    # Pad heading initialization: when set (not None), the EKF quaternion is
    # initialized from the accelerometer (pitch/roll) + this known heading.
    # Simulates real hardware where the board Z-axis points in a known direction
    # (e.g., North → pad_heading_deg=0).  Superseded by inject_truth_orientation.
    # Set to None to skip (e.g. when using magnetometer or truth injection).
    pad_heading_deg: Optional[float] = 0.0

    # Perfect IMU: zero noise and bias (feeds truth sensor data to filter)
    perfect_imu: bool = False

    # Logging
    log_interval: float = 0.001     # 1 kHz log rate


@dataclass
class SimResult:
    """Results from a closed-loop simulation."""
    df: pd.DataFrame = field(default_factory=pd.DataFrame)
    apogee_m: float = 0.0
    max_speed_mps: float = 0.0
    flight_time_s: float = 0.0


def run_closed_loop(rocket_def, config: SimConfig = None) -> SimResult:
    """Run a full closed-loop simulation.

    Args:
        rocket_def: RocketDefinition.
        config: SimConfig (defaults to reasonable values).

    Returns:
        SimResult with DataFrame of logged data.
    """
    if config is None:
        config = SimConfig()

    # Initialize physics
    sim = SixDOF(rocket_def)
    state_obj = sim.initial_state(config.launch_angle_deg, config.heading_deg)
    state = state_obj.to_array()

    # Wind vector in ENU
    # wind_direction_deg is where wind comes FROM (meteorological convention)
    # so wind blows TOWARD the opposite direction
    wind_enu = None
    if config.wind_speed > 0:
        wind_from_rad = math.radians(config.wind_direction_deg)
        # Wind FROM north (0°) → blows south → negative North → negative ENU-y
        wind_east = -config.wind_speed * math.sin(wind_from_rad)
        wind_north = -config.wind_speed * math.cos(wind_from_rad)
        wind_enu = np.array([wind_east, wind_north, 0.0])

    # Initialize sensors
    if config.perfect_imu:
        imu = IMUModel(rate_hz=config.imu_rate,
                       accel_noise_sigma=0.0, gyro_noise_sigma=0.0,
                       accel_bias_sigma=0.0, gyro_bias_sigma=0.0)
    else:
        imu = IMUModel(rate_hz=config.imu_rate)
    baro = BaroModel(rate_hz=config.baro_rate)
    mag = MagModel(rate_hz=config.mag_rate)
    gnss = GNSSModel(
        rate_hz=config.gnss_rate,
        ref_lat_deg=config.ref_lat_deg,
        ref_lon_deg=config.ref_lon_deg,
        ref_alt_m=config.ref_alt_m,
    )

    # Initialize controller
    controller = RollController(
        kp=config.pid_kp, ki=config.pid_ki, kd=config.pid_kd,
        min_cmd=config.deflection_min, max_cmd=config.deflection_max,
        V_ref=config.gain_V_ref, V_min=config.gain_V_min,
        max_scale=config.gain_max_scale,
    )

    # Initialize EKF
    from tinkerrocket_sim._ekf import (GpsInsEKF, IMUData, GNSSData as EKFGNSSData,
                                        MagData as EKFMagData, BaroData as EKFBaroData)

    ekf = GpsInsEKF()

    # Reference LLA for EKF position → NED conversion
    ref_lat_rad = math.radians(config.ref_lat_deg)
    ref_lon_rad = math.radians(config.ref_lon_deg)

    # Sensor timing accumulators
    # Start at -pad_time so t=0 corresponds to motor ignition / launch
    t = -config.pad_time
    imu_dt = 1.0 / config.imu_rate
    baro_dt = 1.0 / config.baro_rate
    mag_dt = 1.0 / config.mag_rate
    gnss_dt = 1.0 / config.gnss_rate

    next_imu = t
    next_baro = t
    next_mag = t
    next_gnss = t
    next_log = t

    # Current actuator state
    fin_tab_cmd = 0.0
    fin_tab_actual = 0.0
    roll_target_deg = None  # current angle target (for profile mode logging)

    # EKF initialization flag
    ekf_initialized = False
    gnss_time_counter = 0
    truth_quat_injected = False

    # Logging
    log_rows = []
    max_alt = 0.0
    past_apogee = False

    # Latest IMU measurement and true specific force (for logging at log rate)
    latest_imu_meas = None
    latest_true_accel_body = None
    latest_baro_alt = None
    latest_mach = 0.0
    latest_gnss_valid = True
    latest_baro_valid = True
    latest_gnss_noise_scale = 1.0
    flight_phase = 'PAD'

    n_steps = int((config.pad_time + config.duration) / config.physics_dt)

    for step in range(n_steps):
        alt = state[2]
        pos = state[0:3]
        vel = state[3:6]
        q = state[6:10]
        omega = state[10:13]
        speed = np.linalg.norm(vel)

        in_pad_phase = (t < 0.0)

        # Termination conditions (only during flight, not pad phase)
        if not in_pad_phase:
            if alt < -1.0 and t > 1.0:
                break
            if past_apogee and alt < 0.0:
                break
            if alt > max_alt:
                max_alt = alt
            if max_alt > 10.0 and alt < max_alt - 5.0:
                past_apogee = True

        # --- Sensor sampling and control at sensor rates ---

        # IMU tick — primary control loop
        if t >= next_imu:
            next_imu += imu_dt

            # Compute acceleration for IMU
            if in_pad_phase:
                # Rocket is stationary on the pad.  True inertial acceleration
                # is zero (normal force cancels gravity).  The IMU model will
                # then compute specific_force = 0 - g_enu = [0, 0, +9.807],
                # giving the correct +1g reading that lets the AHRS level.
                accel_enu = np.zeros(3)
            else:
                # Use actual time: motor returns 0 thrust for t<0 (pad phase),
                # so we get gravity-only derivatives before ignition.
                deriv = sim.derivatives(state, t, fin_tab_actual, wind_enu)
                accel_enu = deriv[3:6]

            imu_meas = imu.measure(accel_enu, omega, q, imu_dt)
            latest_imu_meas = imu_meas

            # Body acceleration magnitude for flight phase detection & GNSS dropout
            accel_body_mag = math.sqrt(
                imu_meas['acc_x']**2 + imu_meas['acc_y']**2 + imu_meas['acc_z']**2)

            # Flight phase detection
            if in_pad_phase:
                flight_phase = 'PAD'
            elif accel_body_mag > 3.0 * 9.807:
                flight_phase = 'BOOST'
            elif not past_apogee:
                flight_phase = 'COAST'
            else:
                flight_phase = 'DESCENT'

            # True specific force in body frame (what a perfect accelerometer reads)
            R_b2e_imu = quat_to_dcm(q)
            R_e2b_imu = R_b2e_imu.T
            g_enu = np.array([0.0, 0.0, -9.807])
            true_sf_enu = accel_enu - g_enu   # same as imu_model.py
            latest_true_accel_body = R_e2b_imu @ true_sf_enu

            # EKF timestamps: offset by pad_time so uint32_t stays positive
            ekf_time_us = int((t + config.pad_time) * 1e6)

            # Initialize EKF on first GNSS + IMU
            if not ekf_initialized and t >= next_gnss - gnss_dt:
                gnss_meas = gnss.measure(pos, vel)
                mag_meas = mag.measure(q)

                imu_d = IMUData()
                imu_d.time_us = ekf_time_us
                # Convert 6DOF body frame (FLU) to EKF body frame (FRD).
                # The 6DOF uses ENU reference + ZYX Euler convention, which
                # produces a Forward-Left-Up body frame.  The EKF (ported from
                # the flight computer) expects Forward-Right-Down.  The
                # conversion is a 180° rotation about body X (nose axis):
                # Y_FRD = -Y_FLU, Z_FRD = -Z_FLU.
                imu_d.acc_x = imu_meas['acc_x']
                imu_d.acc_y = -imu_meas['acc_y']
                imu_d.acc_z = -imu_meas['acc_z']
                imu_d.gyro_x = imu_meas['gyro_x']
                imu_d.gyro_y = -imu_meas['gyro_y']
                imu_d.gyro_z = -imu_meas['gyro_z']

                gnss_d = EKFGNSSData()
                gnss_d.time_us = gnss_time_counter
                gnss_d.ecef_x = gnss_meas['ecef_x']
                gnss_d.ecef_y = gnss_meas['ecef_y']
                gnss_d.ecef_z = gnss_meas['ecef_z']
                gnss_d.ecef_vx = gnss_meas['ecef_vx']
                gnss_d.ecef_vy = gnss_meas['ecef_vy']
                gnss_d.ecef_vz = gnss_meas['ecef_vz']

                mag_d = EKFMagData()
                mag_d.time_us = ekf_time_us
                if config.enable_mag_updates:
                    mag_d.mag_x = mag_meas['mag_x']
                    mag_d.mag_y = -mag_meas['mag_y']
                    mag_d.mag_z = -mag_meas['mag_z']
                # else: zeros → Mahony skips mag correction

                ekf.init(imu_d, gnss_d, mag_d)

                # Pad heading init: compute initial quaternion from
                # accelerometer (pitch/roll) + known heading.  This is
                # what real hardware does when the board Z-axis is
                # aligned to a known direction (e.g. North) on the pad.
                if (config.pad_heading_deg is not None and
                        not config.inject_truth_orientation_at_ignition):
                    # Accelerometer in FRD body frame on a stationary pad
                    # measures specific force = +g in the "up" direction.
                    ax_frd = imu_meas['acc_x']        # FLU→FRD: x same
                    ay_frd = -imu_meas['acc_y']       # FLU→FRD: negate y
                    az_frd = -imu_meas['acc_z']       # FLU→FRD: negate z
                    g_mag = math.sqrt(ax_frd**2 + ay_frd**2 + az_frd**2)
                    if g_mag > 0.1:
                        # NED pitch = elevation from horizontal (positive = nose up).
                        # In FRD, specific force x-axis points along the body
                        # longitudinal axis.  On a nose-up pad ax_frd ≈ +g, so
                        # pitch = asin(ax/g) ≈ +90°.
                        pitch_rad = math.asin(
                            np.clip(ax_frd / g_mag, -1.0, 1.0))
                        # Roll from transverse specific-force components.
                        # At near-vertical pitch (|pitch| > 80°) the
                        # transverse components ay, az are tiny and
                        # dominated by noise, making atan2 unreliable
                        # (gimbal lock).  Default to roll=0 in that case.
                        if abs(pitch_rad) > math.radians(80.0):
                            roll_rad = 0.0
                        else:
                            roll_rad = math.atan2(-ay_frd, -az_frd)
                        yaw_rad = math.radians(config.pad_heading_deg)
                        # Build body→NED quaternion from ZYX Euler (yaw, pitch, roll)
                        cy = math.cos(yaw_rad / 2);   sy = math.sin(yaw_rad / 2)
                        cp = math.cos(pitch_rad / 2);  sp = math.sin(pitch_rad / 2)
                        cr = math.cos(roll_rad / 2);   sr = math.sin(roll_rad / 2)
                        qw = cr*cp*cy + sr*sp*sy
                        qx = sr*cp*cy - cr*sp*sy
                        qy = cr*sp*cy + sr*cp*sy
                        qz = cr*cp*sy - sr*sp*cy
                        ekf.set_quaternion(qw, qx, qy, qz)

                ekf_initialized = True
                next_baro = t  # sync baro timer with EKF init

            elif ekf_initialized:
                # Prepare EKF sensor data
                imu_d = IMUData()
                imu_d.time_us = ekf_time_us
                imu_d.acc_x = imu_meas['acc_x']
                imu_d.acc_y = -imu_meas['acc_y']   # FLU→FRD
                imu_d.acc_z = -imu_meas['acc_z']   # FLU→FRD
                imu_d.gyro_x = imu_meas['gyro_x']
                imu_d.gyro_y = -imu_meas['gyro_y']  # FLU→FRD
                imu_d.gyro_z = -imu_meas['gyro_z']  # FLU→FRD

                # Check if GNSS has new data (with dropout model)
                gnss_d = EKFGNSSData()
                gyro_mag_rps = math.sqrt(omega[0]**2 + omega[1]**2 + omega[2]**2)
                if config.enable_gnss_updates and t >= next_gnss:
                    next_gnss += gnss_dt
                    gnss_meas = gnss.measure(pos, vel,
                                             accel_magnitude=accel_body_mag,
                                             gyro_magnitude_rps=gyro_mag_rps,
                                             sim_time=t)
                    if gnss_meas is not None:
                        gnss_time_counter += 1
                        gnss_d.time_us = gnss_time_counter
                        gnss_d.ecef_x = gnss_meas['ecef_x']
                        gnss_d.ecef_y = gnss_meas['ecef_y']
                        gnss_d.ecef_z = gnss_meas['ecef_z']
                        gnss_d.ecef_vx = gnss_meas['ecef_vx']
                        gnss_d.ecef_vy = gnss_meas['ecef_vy']
                        gnss_d.ecef_vz = gnss_meas['ecef_vz']
                        latest_gnss_valid = True
                        # Set EKF GPS noise scaling based on recovery quality
                        latest_gnss_noise_scale = gnss_meas.get('noise_scale', 1.0)
                        ekf.set_gps_noise_scale(latest_gnss_noise_scale)
                    else:
                        # GNSS dropout — keep time_us unchanged so EKF skips
                        gnss_d.time_us = gnss_time_counter
                        latest_gnss_valid = False
                        latest_gnss_noise_scale = 0.0  # no measurement
                elif not config.enable_gnss_updates:
                    # GNSS disabled — keep time_us frozen so EKF never updates
                    gnss_d.time_us = gnss_time_counter
                    latest_gnss_valid = False
                    latest_gnss_noise_scale = 0.0
                    if t >= next_gnss:
                        next_gnss += gnss_dt
                else:
                    gnss_d.time_us = gnss_time_counter  # same as last

                # Mag sample (zeros if disabled → Mahony runs gyro+accel only)
                mag_d = EKFMagData()
                mag_d.time_us = ekf_time_us
                if config.enable_mag_updates:
                    mag_meas = mag.measure(q)
                    mag_d.mag_x = mag_meas['mag_x']
                    mag_d.mag_y = -mag_meas['mag_y']   # FLU→FRD
                    mag_d.mag_z = -mag_meas['mag_z']   # FLU→FRD

                # Sensor trust: only use accel gravity reference on the pad and
                # during descent.  During boost the accelerometer reads thrust,
                # and during coast aerodynamic drag can exceed 1g, both of
                # which corrupt the gravity reference.  This matches the flight
                # computer which disables accel corrections during INFLIGHT.
                use_ahrs_acc = (flight_phase in ('PAD', 'DESCENT'))
                ekf.update(use_ahrs_acc, imu_d, gnss_d, mag_d)

                # --- Truth state injection at ignition ---
                # Inject truth orientation, position, and velocity so that
                # dead-reckoning tests start from zero error at t=0.
                if (config.inject_truth_orientation_at_ignition and
                        not truth_quat_injected and t >= 0.0):
                    # Convert truth quat (FLU-to-ENU) to EKF convention (FRD-to-NED)
                    q_ned = quat_enu_flu_to_ned_frd(q)
                    ekf.set_quaternion(q_ned[0], q_ned[1], q_ned[2], q_ned[3])
                    # Inject truth position (ENU → LLA)
                    ekf.set_position(
                        ref_lat_rad + pos[1] / 6378137.0,             # North→lat
                        ref_lon_rad + pos[0] / (6378137.0 * math.cos(ref_lat_rad)),  # East→lon
                        config.ref_alt_m + pos[2])                     # Up→alt
                    # Inject truth velocity (ENU → NED)
                    ekf.set_velocity(vel[1], vel[0], -vel[2])
                    truth_quat_injected = True

                # --- Baro measurement + Mach lockout ---
                a_sound = atm.speed_of_sound(alt + config.ref_alt_m)
                latest_mach = speed / a_sound if a_sound > 0 else 0.0
                latest_baro_valid = (latest_mach < 0.5)

                if t >= next_baro:
                    next_baro += baro_dt
                    baro_meas = baro.measure(alt + config.ref_alt_m,
                                             mach=latest_mach)
                    # ISA pressure-to-altitude inversion
                    baro_alt = 44330.0 * (1.0 - (baro_meas['pressure']
                                                  / 101325.0) ** (1.0 / 5.255))
                    latest_baro_alt = baro_alt

                    if latest_baro_valid and config.enable_baro_updates:
                        baro_d = EKFBaroData()
                        baro_d.time_us = ekf_time_us
                        baro_d.altitude_m = baro_alt
                        ekf.baro_meas_update(baro_d)

                # PID control: use EKF roll rate estimate
                if config.control_enabled and speed > 5.0:
                    rot_rate = ekf.get_rot_rate_est()
                    roll_rate_dps = math.degrees(rot_rate[0])

                    if config.roll_profile is not None:
                        # --- Cascaded angle control ---
                        # 1. Look up target angle from profile
                        target_angle = config.roll_profile[0][1]
                        for wp_t, wp_angle in config.roll_profile:
                            if t >= wp_t:
                                target_angle = wp_angle
                            else:
                                break

                        # 2. Get current roll angle from EKF quaternion.
                        #    "Roll" = azimuth of body-Z in the NED horizontal
                        #    plane, which is gimbal-lock-free at all pitch
                        #    angles (singular only at pitch=0° / horizontal).
                        quat = ekf.get_quaternion()
                        qw, qx, qy, qz = quat
                        z_north = 2.0 * (qx*qz + qw*qy)
                        z_east  = 2.0 * (qy*qz - qw*qx)
                        current_roll_deg = -math.degrees(
                            math.atan2(z_east, z_north))

                        # 3. Compute angle error with wrapping to [-180, 180]
                        angle_error = target_angle - current_roll_deg
                        angle_error = (angle_error + 180.0) % 360.0 - 180.0

                        # 4. Outer loop: angle error → rate setpoint
                        rate_setpoint = config.kp_angle * angle_error

                        # 5. Inner loop: rate PID
                        fin_tab_cmd = controller.compute(
                            rate_setpoint,
                            roll_rate_dps,
                            imu_dt,
                            airspeed=speed,
                        )

                        # Store for logging
                        roll_target_deg = target_angle
                    else:
                        # --- Constant rate control ---
                        fin_tab_cmd = controller.compute(
                            config.roll_setpoint_dps,
                            roll_rate_dps,
                            imu_dt,
                            airspeed=speed,
                        )
                        roll_target_deg = None

            # Actuator model: rate-limit the command
            max_delta = config.servo_rate_limit * imu_dt
            delta = fin_tab_cmd - fin_tab_actual
            if abs(delta) > max_delta:
                delta = max_delta if delta > 0 else -max_delta
            fin_tab_actual += delta

        # --- Logging ---
        if t >= next_log:
            next_log += config.log_interval
            s = SimState.from_array(state, t)
            r, p, y = s.euler_deg

            # Compute AoA for logging — only meaningful when the rocket
            # has significant forward (axial) velocity.  At low axial speed
            # the wind dominates and AoA is near 90° but aerodynamically
            # irrelevant because the rocket hasn't built up speed yet.
            q_log = quat_normalize(q)
            R_e2b_log = quat_to_dcm(q_log).T
            if wind_enu is not None:
                v_air_log = vel - wind_enu
            else:
                v_air_log = vel.copy()
            v_air_body_log = R_e2b_log @ v_air_log
            v_axial_log = v_air_body_log[0]
            v_lat_log = np.sqrt(v_air_body_log[1]**2 + v_air_body_log[2]**2)
            # Only compute AoA when forward speed is meaningful (> 10 m/s)
            # Below this, AoA is dominated by wind and dynamic pressure is tiny
            if v_axial_log > 10.0:
                alpha_log = np.degrees(np.arctan2(v_lat_log, v_axial_log))
            else:
                alpha_log = 0.0

            # Use flight time for thrust/mass (0 during pad phase)
            flight_t = max(t, 0.0)

            row = {
                'time': t,
                'x': pos[0], 'y': pos[1], 'z': pos[2],
                'vx': vel[0], 'vy': vel[1], 'vz': vel[2],
                'speed': speed,
                'altitude': alt,
                'roll_deg': r, 'pitch_deg': p, 'yaw_deg': y,
                'roll_rate_dps': np.degrees(omega[0]),
                'pitch_rate_dps': np.degrees(omega[1]),
                'yaw_rate_dps': np.degrees(omega[2]),
                'alpha_deg': alpha_log,
                'thrust': rocket_def.motor.thrust_at(flight_t),
                'mass': rocket_def.mass_at(flight_t),
                'fin_tab_cmd': fin_tab_cmd,
                'fin_tab_actual': fin_tab_actual,
                # True NED state (for EKF comparison)
                'true_pn': pos[1],        # ENU-y = North
                'true_pe': pos[0],        # ENU-x = East
                'true_pd': -pos[2],       # -Up = Down
                'true_vn': vel[1],
                'true_ve': vel[0],
                'true_vd': -vel[2],
            }

            # Truth orientation in NED/FRD convention (for direct EKF comparison)
            q_ned = quat_enu_flu_to_ned_frd(q_log)
            ned_roll, ned_pitch, ned_yaw = quat_to_euler(q_ned)
            row['true_roll_ned_deg'] = np.degrees(ned_roll)
            row['true_pitch_ned_deg'] = np.degrees(ned_pitch)
            row['true_yaw_ned_deg'] = np.degrees(ned_yaw)
            row['true_q0_ned'] = q_ned[0]
            row['true_q1_ned'] = q_ned[1]
            row['true_q2_ned'] = q_ned[2]
            row['true_q3_ned'] = q_ned[3]

            # Gimbal-lock-free roll: azimuth of body-Z in NED horizontal
            # plane.  Well-conditioned at any pitch above a few degrees.
            tn = 2.0 * (q_ned[1]*q_ned[3] + q_ned[0]*q_ned[2])
            te = 2.0 * (q_ned[2]*q_ned[3] - q_ned[0]*q_ned[1])
            row['true_roll_quat_deg'] = -math.degrees(math.atan2(te, tn))

            # IMU measurements (body FRD frame, with noise/bias)
            if latest_imu_meas is not None:
                row['imu_acc_x'] = latest_imu_meas['acc_x']
                row['imu_acc_y'] = latest_imu_meas['acc_y']
                row['imu_acc_z'] = latest_imu_meas['acc_z']
                row['imu_gyro_x'] = latest_imu_meas['gyro_x']
                row['imu_gyro_y'] = latest_imu_meas['gyro_y']
                row['imu_gyro_z'] = latest_imu_meas['gyro_z']
            # True specific force in body frame (no noise, for comparison)
            if latest_true_accel_body is not None:
                row['true_acc_x'] = latest_true_accel_body[0]
                row['true_acc_y'] = latest_true_accel_body[1]
                row['true_acc_z'] = latest_true_accel_body[2]

            # Flight phase & sensor validity
            row['flight_phase'] = flight_phase
            row['mach'] = latest_mach
            row['gnss_valid'] = latest_gnss_valid
            row['gnss_noise_scale'] = latest_gnss_noise_scale
            row['baro_valid'] = latest_baro_valid
            if latest_baro_alt is not None:
                row['baro_alt'] = latest_baro_alt

            # Add roll angle profile data if active
            if roll_target_deg is not None:
                row['roll_target_deg'] = roll_target_deg

            if ekf_initialized:
                orient = ekf.get_orientation()
                ekf_vel = ekf.get_velocity()
                ekf_pos = ekf.get_position()  # (lat_rad, lon_rad, alt_m)
                row['ekf_roll_deg'] = math.degrees(orient[0])
                row['ekf_pitch_deg'] = math.degrees(orient[1])
                row['ekf_yaw_deg'] = math.degrees(orient[2])
                ekf_quat = ekf.get_quaternion()
                row['ekf_q0'] = ekf_quat[0]
                row['ekf_q1'] = ekf_quat[1]
                row['ekf_q2'] = ekf_quat[2]
                row['ekf_q3'] = ekf_quat[3]
                # Gimbal-lock-free roll (same formula used by controller)
                zn = 2.0*(ekf_quat[1]*ekf_quat[3] + ekf_quat[0]*ekf_quat[2])
                ze = 2.0*(ekf_quat[2]*ekf_quat[3] - ekf_quat[0]*ekf_quat[1])
                row['ekf_roll_quat_deg'] = -math.degrees(math.atan2(ze, zn))
                row['ekf_vn'] = ekf_vel[0]
                row['ekf_ve'] = ekf_vel[1]
                row['ekf_vd'] = ekf_vel[2]
                # EKF position: LLA → NED relative to launch site
                R_earth = 6378137.0
                row['ekf_pn'] = (ekf_pos[0] - ref_lat_rad) * R_earth
                row['ekf_pe'] = (ekf_pos[1] - ref_lon_rad) * R_earth * math.cos(ref_lat_rad)
                row['ekf_pd'] = -(ekf_pos[2] - config.ref_alt_m)

                # EKF covariances (for uncertainty bounds)
                cov_pos = ekf.get_cov_pos()
                cov_vel = ekf.get_cov_vel()
                row['ekf_pos_sigma_n'] = math.sqrt(max(0.0, cov_pos[0]))
                row['ekf_pos_sigma_e'] = math.sqrt(max(0.0, cov_pos[1]))
                row['ekf_pos_sigma_d'] = math.sqrt(max(0.0, cov_pos[2]))
                row['ekf_vel_sigma_n'] = math.sqrt(max(0.0, cov_vel[0]))
                row['ekf_vel_sigma_e'] = math.sqrt(max(0.0, cov_vel[1]))
                row['ekf_vel_sigma_d'] = math.sqrt(max(0.0, cov_vel[2]))

            log_rows.append(row)

        # --- Physics step (skip during pad phase — rocket is stationary) ---
        if not in_pad_phase:
            state = sim.step_rk4(state, t, config.physics_dt, fin_tab_actual, wind_enu)
        t += config.physics_dt

    # Build result
    result = SimResult()
    if log_rows:
        result.df = pd.DataFrame(log_rows)
        result.apogee_m = float(max_alt)
        result.max_speed_mps = float(result.df['speed'].max())
        result.flight_time_s = float(result.df['time'].iloc[-1])

    return result
