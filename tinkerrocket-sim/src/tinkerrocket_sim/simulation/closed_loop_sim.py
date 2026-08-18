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
from ..physics.dryden import DrydenGust


@dataclass
class SimConfig:
    """Closed-loop simulation configuration."""
    # Timing
    physics_dt: float = 1e-4        # 10 kHz
    imu_rate: float = 1200.0
    baro_rate: float = 500.0
    mag_rate: float = 1000.0
    gnss_rate: float = 25.0
    # Seed for the sensor-noise RNGs (None = nondeterministic). Each model
    # gets a distinct derived seed so streams are independent (#459 A/B).
    sensor_seed: int = None
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
    # Activation delay (s after launch) before fin control engages. SHARED by
    # roll control AND guidance, mirroring the firmware: both live in the same
    # post-delay branch gated by `t_since_launch_ms >= roll_delay_ms` (FC
    # main.cpp). This is the app's "Activation Delay" (BLE roll_delay_ms);
    # guidance engages at launch+delay, NOT at burnout.
    roll_delay_s: float = 0.25

    # Roll angle profile: list of (time_s, angle_deg) waypoints, or None
    # When set, a cascaded controller tracks angle → rate → fin tab
    roll_profile: Optional[list] = None
    kp_angle: float = 5.0          # outer loop gain: (deg/s) per (deg error)
    rate_cap_dps: float = 60.0     # outer-loop rate command cap (KP_ANGLE_RATE_CAP_DPS)
    # Real TR_ServoControl SIL (controlAngle / gain schedule / rate cap) vs the
    # legacy Python RollController. The real controller is negative-feedback in
    # the FIRMWARE sign convention, which is opposite the legacy Python cascade,
    # so it only stabilizes when the rocket's roll-tab fin->torque sign matches
    # the real vehicle (set fin_tabs.Kt_ref sign from CFD/flight; negative in the
    # sim body frame for the 67mm RollyPolly III testbed). Opt-in until that
    # per-vehicle plant sign is locked from CFD.
    use_firmware_roll_controller: bool = False
    roll_gain_schedule_enabled: bool = True   # servo V^2 gain schedule on/off (firmware path)
    d_lpf_hz: float = 0.0                      # servo PID derivative LPF cutoff Hz (0=off)
    integral_sep_threshold: float = 0.0        # PID integral-separation anti-windup; freeze I when |err|>thr (0=off)
    # Roll-profile targeting: "hold" = flown 6/14 firmware (hold the most-recent
    # waypoint); "endpoint" = pre-v4 firmware (803d23f) — command the segment's
    # END waypoint angle by the shortest path (no ramp), so a maneuver engages at
    # the segment START, not its end; "ramp" = v4 firmware — null-rate before the
    # first waypoint, target lerped along the shortest wrapped arc between
    # waypoints (per-waypoint modes ignored), last angle held after the profile.
    roll_targeting: str = "hold"
    # Impulsive roll-rate perturbation for the controller to null: add
    # roll_kick_dps to the body roll rate once at roll_kick_time_s (s after launch).
    roll_kick_time_s: float = 0.0
    roll_kick_dps: float = 0.0

    # Gain scheduling — V_ref=50 gives 1.73x at 38 m/s, 0.51x at 70 m/s
    gain_V_ref: float = 50.0
    gain_V_min: float = 25.0
    gain_max_scale: float = 3.0

    # Actuator limits — PTK 7308 at 8.2V: 923 deg/s slew, +/-20 deg
    deflection_min: float = -20.0   # deg
    deflection_max: float = 20.0    # deg
    # Servo actuator model (PTK 7308: 0.065 s/60deg, 2 us deadband).
    servo_rate_limit: float = 923.0 # deg/s slew rate (= 60 / 0.065 s-per-60deg)
    servo_tau_s: float = 0.0325     # first-order lag time const (s); ~half the 60deg-travel time; 0 -> pure slew (legacy)
    servo_deadband_us: float = 2.0  # servo PWM deadband (us); 0 -> none
    # Mechanical linkage lash between the servo horn and the fin: full gap
    # width in deg of fin travel. The fin only engages after the horn crosses
    # the half-gap; inside the gap the fin holds (linkage slack). 0 = rigid.
    # Default off — healthy-servo lash is unmeasured (bench wiggle-test to
    # calibrate); the known-bad servo 3 measured ~10 deg.
    servo_backlash_deg: float = 0.0

    # Wind (constant ENU, m/s)
    wind_speed: float = 0.0             # m/s
    wind_direction_deg: float = 0.0     # direction wind comes FROM (0=North, 90=East)

    # Dryden atmospheric turbulence (physics/dryden.py), added on top of the
    # steady wind above and independent of it — a gust may be flown with zero
    # mean wind. gust_w20_mps is the wind speed at the 20 ft reference height,
    # which sets the turbulence intensity: ~7.7 light, ~15.4 moderate, ~23.2
    # severe (W20_LIGHT / W20_MODERATE / W20_SEVERE). 0 = off, and with it off
    # no filter is built and the wind vector is exactly what it was before the
    # gust model existed. gust_vertical=False drops the Up component, which is
    # the one that perturbs axial velocity and so moves apogee.
    # NOTE: a uniform gust exerts NO roll moment on this symmetric airframe —
    # it stresses pitch/yaw, AoA, the EKF and dispersion, not the roll loop.
    # Use roll_misalign_deg / roll_kick_dps for roll disturbances.
    gust_w20_mps: float = 0.0
    gust_vertical: bool = True

    # Launch site (for GNSS)
    ref_lat_deg: float = 38.0
    ref_lon_deg: float = -122.0
    ref_alt_m: float = 0.0

    # Sensor aiding (set False to test pure IMU dead-reckoning)
    enable_gnss_updates: bool = True
    enable_baro_updates: bool = True
    enable_mag_updates: bool = True

    # IMU fidelity knobs (None = off, preserving the ideal-range default).
    # gyro_full_scale_dps: hard-clip the gyro at ±range (the 2026-05-17 flight
    # ran ±4000 dps and saturated). imu_mounting_rotation_deg: (roll,pitch,yaw)
    # ZYX Euler of the IMU relative to the airframe — injects a fixed
    # misalignment the firmware constants do not undo (degrades the EKF).
    gyro_full_scale_dps: Optional[float] = None
    accel_full_scale_g: Optional[float] = None
    imu_mounting_rotation_deg: Optional[tuple] = None

    # Truth injection: set the EKF quaternion to the true orientation at ignition
    # (removes AHRS convergence error, isolates pure IMU integration drift)
    inject_truth_orientation_at_ignition: bool = False

    # Heading-bias injection: at ignition, rotate the EKF attitude by this many
    # degrees about the world vertical (NED-down). Mimics a corrupted pad-mag
    # heading (e.g. steel launch rail) seeding a fixed heading error. Used to
    # test whether the in-flight magnetometer update recovers it.
    inject_heading_bias_deg: float = 0.0

    # Pad heading initialization: when set (not None), the EKF quaternion is
    # initialized from the accelerometer (pitch/roll) + this known heading.
    # Simulates real hardware where the board Z-axis points in a known direction
    # (e.g., North → pad_heading_deg=0).  Superseded by inject_truth_orientation.
    # Set to None to skip (e.g. when using magnetometer or truth injection).
    pad_heading_deg: Optional[float] = 0.0

    # Perfect IMU: zero noise and bias (feeds truth sensor data to filter)
    perfect_imu: bool = False

    # --- Guidance (PN + 3-axis control) ---
    guidance_enabled: bool = False  # False = roll-only (default), True = guided coast
    guidance_mode: str = 'pn'       # 'pn' = proportional navigation, 'attitude' = point-at-target
    guidance_debug: bool = False    # print the first 20 PN steps (interactive use)

    # PN guidance parameters
    pn_nav_gain: float = 3.0
    pn_max_tilt_deg: float = 10.0
    pn_max_accel_mps2: float = 5.0
    pn_blend_radius_m: float = 5.0
    pn_kp_pos: float = 0.5
    pn_kd_vel: float = 1.0
    # Station-keep horizontal aim point, ENU meters relative to the pad (#435;
    # mirrors the FC's cmd-28 GuidancePointData after its LLA->ENU conversion).
    # (0,0) = overhead.  Consumed only in STATION_KEEP; inert in MODE_PN,
    # exactly like the firmware.
    pn_target_e_m: float = 0.0
    pn_target_n_m: float = 0.0

    # Pitch/yaw outer loop angle P gains
    pn_kp_pitch_angle: float = 4.0
    pn_kp_yaw_angle: float = 4.0

    # Pitch/yaw inner loop rate PID gains
    pn_pitch_kp: float = 0.04
    pn_pitch_ki: float = 0.001
    pn_pitch_kd: float = 0.0003
    pn_yaw_kp: float = 0.04
    pn_yaw_ki: float = 0.001
    pn_yaw_kd: float = 0.0003

    # Guided mode limits
    pn_max_fin_deg: float = 15.0
    pn_min_speed_mps: float = 15.0
    # Tilt-limit safety gate (mirrors FC config::GUIDANCE_TILT_LIMIT_*): if the
    # off-vertical tilt from EKF attitude exceeds the phase limit, guidance is
    # LATCHED off (roll-only) for the rest of the flight.
    guidance_tilt_limit_boost_deg: float = 15.0
    guidance_tilt_limit_coast_deg: float = 20.0
    # LEGACY — no longer a guidance gate. The firmware dropped the post-burnout
    # "coast delay" in favour of the shared activation delay (roll_delay_s); kept
    # only so callers passing it don't break. See roll_delay_s above.
    pn_coast_delay_s: float = 0.0
    pn_target_alt_m: float = 600.0      # OVERHEAD aim-point altitude above pad (matches FC config::PN_TARGET_ALT_M)
    pn_accel_to_fin_deg: float = 4.0    # accel->fin scale; reconciled to the FC's config::PN_ACCEL_TO_FIN_DEG (was 5.0)

    # Gain scheduling for pitch/yaw (separate from roll)
    pn_gain_V_ref: float = 50.0
    pn_gain_V_min: float = 25.0

    # Logging
    log_interval: float = 0.001     # 1 kHz log rate


@dataclass
class SimResult:
    """Results from a closed-loop simulation."""
    df: pd.DataFrame = field(default_factory=pd.DataFrame)
    apogee_m: float = 0.0
    max_speed_mps: float = 0.0
    flight_time_s: float = 0.0


def _backlash_step(output_deg: float, input_deg: float, gap_deg: float) -> float:
    """One step of a mechanical backlash (linkage play) operator.

    The output (fin) engages the input (servo horn) only once the input has
    crossed the half-gap; inside the gap the linkage is slack and the output
    holds. On a direction reversal the input must traverse the full gap before
    the output moves again. gap_deg <= 0 is a rigid linkage (output = input).
    """
    if gap_deg <= 0.0:
        return input_deg
    half = 0.5 * gap_deg
    if input_deg - output_deg > half:
        return input_deg - half
    if output_deg - input_deg > half:
        return input_deg + half
    return output_deg


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
    wind_steady_enu = None
    if config.wind_speed > 0:
        wind_from_rad = math.radians(config.wind_direction_deg)
        # Wind FROM north (0°) → blows south → negative North → negative ENU-y
        wind_east = -config.wind_speed * math.sin(wind_from_rad)
        wind_north = -config.wind_speed * math.cos(wind_from_rad)
        wind_steady_enu = np.array([wind_east, wind_north, 0.0])
    # With the gust off this is never rebound, so wind_enu keeps its None
    # sentinel and every downstream consumer behaves exactly as before.
    wind_enu = wind_steady_enu

    # Initialize sensors
    def _seed(i):
        return None if config.sensor_seed is None else config.sensor_seed + i
    _imu_fidelity = dict(
        gyro_full_scale_dps=config.gyro_full_scale_dps,
        accel_full_scale_g=config.accel_full_scale_g,
        mounting_rotation_deg=config.imu_mounting_rotation_deg,
    )
    if config.perfect_imu:
        imu = IMUModel(rate_hz=config.imu_rate,
                       accel_noise_sigma=0.0, gyro_noise_sigma=0.0,
                       accel_bias_sigma=0.0, gyro_bias_sigma=0.0,
                       seed=_seed(0), **_imu_fidelity)
    else:
        imu = IMUModel(rate_hz=config.imu_rate, seed=_seed(0), **_imu_fidelity)
    baro = BaroModel(rate_hz=config.baro_rate, seed=_seed(1))
    mag = MagModel(rate_hz=config.mag_rate, seed=_seed(2))
    gnss = GNSSModel(
        rate_hz=config.gnss_rate,
        ref_lat_deg=config.ref_lat_deg,
        ref_lon_deg=config.ref_lon_deg,
        ref_alt_m=config.ref_alt_m,
        seed=_seed(3),
    )

    # Dryden turbulence. Seed index 4 — 0..3 are taken by the sensor models
    # above, and sharing one would both correlate the gust with sensor noise
    # and shift the existing scenarios' noise streams.
    gust = None
    if config.gust_w20_mps > 0.0:
        gust = DrydenGust(config.gust_w20_mps, seed=_seed(4),
                          vertical=config.gust_vertical)
        gust.reset(h_agl_m=0.0, v_mps=30.0)

    # Initialize controller
    controller = RollController(
        kp=config.pid_kp, ki=config.pid_ki, kd=config.pid_kd,
        min_cmd=config.deflection_min, max_cmd=config.deflection_max,
        V_ref=config.gain_V_ref, V_min=config.gain_V_min,
        max_scale=config.gain_max_scale,
    )

    # Real firmware roll controller (SIL): runs the exact TR_ServoControl
    # controlAngle cascade / V^2 gain schedule / KP_ANGLE rate cap that flies,
    # replacing the Python RollController when use_firmware_roll_controller.
    # Its PID derives dt from micros(); we advance a monotonic mock clock by
    # imu_dt each control tick, mirroring the real flight loop.
    from tinkerrocket_sim._servo import (ServoControl,
                                         set_mock_micros as _servo_set_micros)
    servo = ServoControl(
        kp=config.pid_kp, ki=config.pid_ki, kd=config.pid_kd,
        min_cmd=config.deflection_min, max_cmd=config.deflection_max,
    )
    if config.roll_gain_schedule_enabled:
        servo.enable_gain_schedule(config.gain_V_ref, config.gain_V_min)
    if config.d_lpf_hz > 0.0:
        servo.set_pid_derivative_filter_cutoff_hz(config.d_lpf_hz)
    if config.integral_sep_threshold > 0.0:
        servo.set_pid_integral_separation_threshold(config.integral_sep_threshold)
    # Seed the persistent rate setpoint ONCE, mirroring the firmware boot /
    # SERVO_CTRL_ENABLE path (main.cpp:2768,4078 -> setSetpoint(ROLL_RATE_SET_POINT)).
    # After this, the rate-null path never re-writes the setpoint — exactly like
    # firmware — so a stale setpoint left by controlAngle (#372) would manifest
    # here instead of being masked by a per-tick reset.
    servo.set_setpoint(config.roll_setpoint_dps)
    servo_clock_us = 0

    # Initialize EKF
    from tinkerrocket_sim._ekf import (GpsInsEKF, IMUData, GNSSData as EKFGNSSData,
                                        MagData as EKFMagData, BaroData as EKFBaroData)

    ekf = GpsInsEKF()

    # Initialize guidance and mixer (only used when guidance_enabled)
    guidance = None
    control_mixer = None
    roll_controller_standalone = None  # separate roll PID for guided mode
    if config.guidance_enabled:
        from tinkerrocket_sim._guidance import GuidancePN
        from tinkerrocket_sim._mixer import ControlMixer

        guidance = GuidancePN()
        if config.guidance_mode == 'station_keep':
            guidance.configure_station_keep(
                config.pn_kp_pos, config.pn_kd_vel, config.pn_max_accel_mps2)
        else:
            guidance.configure(
                config.pn_nav_gain, config.pn_max_accel_mps2,
                config.pn_target_alt_m)  # OVERHEAD: aim straight up over the pad
        # Horizontal aim point AFTER the law config, mirroring the FC's
        # applyGuidanceConfig() ordering (law first — configure* resets the
        # target; #435).  Inert in MODE_PN, exactly like the firmware.
        guidance.set_horizontal_target(config.pn_target_e_m, config.pn_target_n_m)

        control_mixer = ControlMixer()
        control_mixer.configure(
            config.pn_pitch_kp, config.pn_pitch_ki, config.pn_pitch_kd,
            config.pn_yaw_kp, config.pn_yaw_ki, config.pn_yaw_kd,
            config.pn_max_fin_deg,
            config.pn_gain_V_ref, config.pn_gain_V_min)
        control_mixer.enable_gain_schedule(config.pn_gain_V_ref, config.pn_gain_V_min)

        # Standalone roll PID for guided mode (same gains as main controller)
        roll_controller_standalone = RollController(
            kp=config.pid_kp, ki=config.pid_ki, kd=config.pid_kd,
            min_cmd=-config.pn_max_fin_deg, max_cmd=config.pn_max_fin_deg,
            V_ref=config.gain_V_ref, V_min=config.gain_V_min,
            max_scale=config.gain_max_scale)

    # Reference LLA for EKF position → NED conversion
    ref_lat_rad = math.radians(config.ref_lat_deg)
    ref_lon_rad = math.radians(config.ref_lon_deg)

    # Sensor timing accumulators
    # Start at -pad_time so t=0 corresponds to motor ignition / launch
    t = -config.pad_time
    imu_dt = 1.0 / config.imu_rate
    # Servo PWM deadband -> fin degrees. The servo maps its 1000 us pulse band
    # (1000-2000 us) across the full deflection range, so us/deg = 1000/span.
    _servo_deadband_deg = (config.servo_deadband_us *
                           (config.deflection_max - config.deflection_min) / 1000.0)
    _servo_tau_eff = max(config.servo_tau_s, imu_dt)  # tau<=dt -> pure slew limiter
    baro_dt = 1.0 / config.baro_rate
    mag_dt = 1.0 / config.mag_rate
    gnss_dt = 1.0 / config.gnss_rate

    next_imu = t
    next_baro = t
    next_mag = t
    next_gnss = t
    # #459: mag samples are generated at mag_rate and HELD between samples
    # with their sample timestamp, so the EKF sees the sensor's real cadence
    # (the FC stamps ekf_mag.time_us with the sensor sample time).
    held_mag_frd = None
    mag_sample_time_us = 0
    next_log = t

    # Current actuator state. servo_horn_deg is the servo-horn position (the
    # lag/slew dynamic state); fin_tab_actual is the fin behind the linkage
    # lash. With servo_backlash_deg=0 they are identical.
    fin_tab_cmd = 0.0
    fin_tab_actual = 0.0
    servo_horn_deg = 0.0
    roll_target_deg = None  # current angle target (for profile mode logging)
    current_roll_deg = None  # controller's regulated roll angle (azimuth, for logging)
    _roll_kicked = False

    # 4-fin actuator state (guided mode); fin_horns are the servo-horn
    # positions ahead of the linkage lash, like servo_horn_deg above.
    fin_cmds = np.zeros(4)
    fin_actuals = np.zeros(4)
    fin_horns = np.zeros(4)
    guidance_active = False
    guidance_tilt_inhibited = False  # tilt-limit safety latch (FC: guidance_tilt_inhibited)
    guidance_tilt_trip_t = None      # time the latch tripped (for reporting/plots)
    burnout_detected = False
    burnout_time = None
    guidance_cpa_reached = False  # closest point of approach — stop guidance

    # EKF initialization flag
    ekf_initialized = False
    gnss_time_counter = 0
    truth_quat_injected = False
    heading_bias_injected = False

    # Logging
    log_rows = []
    max_alt = 0.0
    past_apogee = False

    # Latest IMU measurement and true specific force (for logging at log rate)
    latest_imu_meas = None
    latest_true_accel_body = None
    # Seeded rather than left None: the first log row is emitted on the same
    # tick that initializes the EKF, and the baro is only sampled in the
    # *other* branch of that if/elif, so no reading exists yet. Left as None
    # the row omits the key entirely and pandas backfills NaN for it, poking a
    # one-sample hole in the column that any numpy-side consumer inherits.
    # The seed is the noiseless ISA inversion at the starting altitude —
    # exactly what the sampler computes, minus the sensor noise it has not
    # drawn yet — so it costs no RNG draw and shifts no baseline.
    latest_baro_alt = 44330.0 * (
        1.0 - (atm.pressure(state[2] + config.ref_alt_m) / 101325.0)
        ** (1.0 / 5.255))
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

        # Advance the gust before anything reads the wind this step, so the
        # IMU derivative, the logged row and the RK4 step all see the same
        # value at the same t. (Unlike the roll kick at the bottom of the
        # loop, which deliberately applies after t advances.)
        #
        # Held off during the pad phase: wind is inert there anyway (IMU accel
        # is forced to zero and the physics step is skipped), and integrating
        # through it would make the gust at t=0 depend on pad_time, coupling
        # two knobs that are currently independent. reset()'s burn-in already
        # starts the filter stationary.
        if gust is not None and not in_pad_phase:
            gust_enu = gust.step(config.physics_dt, float(speed), float(alt))
            wind_enu = (gust_enu if wind_steady_enu is None
                        else wind_steady_enu + gust_enu)

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
                    # #459: this is the first held sample
                    held_mag_frd = (mag_d.mag_x, mag_d.mag_y, mag_d.mag_z)
                    mag_sample_time_us = ekf_time_us
                    next_mag = t + mag_dt
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

                # Mag sample (zeros if disabled → Mahony runs gyro+accel only).
                # #459: a new sample is generated only at mag_rate; between
                # samples the last one is re-presented with its ORIGINAL
                # timestamp — matching the FC, where updateCore runs every EKF
                # tick but iis2mdc_latest_si only refreshes at ~98 Hz. The
                # EKF's mag freshness gate dedups on time_us.
                mag_d = EKFMagData()
                mag_d.time_us = ekf_time_us
                if config.enable_mag_updates:
                    if t >= next_mag:
                        next_mag += mag_dt
                        mag_meas = mag.measure(q)
                        held_mag_frd = (mag_meas['mag_x'],
                                        -mag_meas['mag_y'],   # FLU→FRD
                                        -mag_meas['mag_z'])   # FLU→FRD
                        mag_sample_time_us = ekf_time_us
                    if held_mag_frd is not None:
                        mag_d.time_us = mag_sample_time_us
                        mag_d.mag_x, mag_d.mag_y, mag_d.mag_z = held_mag_frd

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

                # --- Heading-bias injection at ignition (rail-mag mimic) ---
                if (config.inject_heading_bias_deg != 0.0 and
                        not heading_bias_injected and t >= 0.0):
                    dpsi = math.radians(config.inject_heading_bias_deg)
                    qd = (math.cos(dpsi / 2.0), 0.0, 0.0, math.sin(dpsi / 2.0))  # about NED-down
                    qe = ekf.get_quaternion()  # (w,x,y,z), FRD→NED
                    # q_new = qd ⊗ qe (rotate body azimuth by dpsi in the world)
                    w = qd[0]*qe[0] - qd[1]*qe[1] - qd[2]*qe[2] - qd[3]*qe[3]
                    x = qd[0]*qe[1] + qd[1]*qe[0] + qd[2]*qe[3] - qd[3]*qe[2]
                    y = qd[0]*qe[2] - qd[1]*qe[3] + qd[2]*qe[0] + qd[3]*qe[1]
                    z = qd[0]*qe[3] + qd[1]*qe[2] - qd[2]*qe[1] + qd[3]*qe[0]
                    ekf.set_quaternion(w, x, y, z)
                    heading_bias_injected = True

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

                # --- Burnout detection (for guidance activation) ---
                if (config.guidance_enabled and not burnout_detected and
                        t > 0.2 and flight_phase == 'COAST'):
                    burnout_detected = True
                    burnout_time = t
                    # Remove roll disturbance after burnout (motor spin stops)
                    if hasattr(sim.rocket, 'roll_disturbance_torque'):
                        sim.rocket.roll_disturbance_torque = 0.0

                # --- Determine if guided coast is active ---
                # Mirrors the firmware: guidance engages once past the shared
                # activation delay (launch+roll_delay_s), gated only by airspeed
                # and not-yet-CPA — NOT by burnout (FC main.cpp comment: "burnout
                # are no longer gates here").
                # Tilt-limit safety gate (mirrors FC main.cpp): off-vertical tilt
                # of the nose from EKF attitude. Latched — once tripped, guidance
                # stays roll-only for the rest of the flight. Only checked after
                # launch+activation delay (the FC checks it inside the post-launch
                # control block), so a pad-init attitude transient can't trip it.
                if (config.guidance_enabled and not guidance_tilt_inhibited
                        and t >= config.roll_delay_s):
                    eq = ekf.get_quaternion()  # (q0,q1,q2,q3) body->NED
                    nose_up = -2.0 * (eq[1]*eq[3] - eq[0]*eq[2])
                    nose_up = max(-1.0, min(1.0, nose_up))
                    tilt_deg = math.degrees(math.acos(nose_up))
                    tilt_limit = (config.guidance_tilt_limit_coast_deg if burnout_detected
                                  else config.guidance_tilt_limit_boost_deg)
                    if tilt_deg > tilt_limit:
                        guidance_tilt_inhibited = True
                        guidance_tilt_trip_t = t
                        print(f"  [GUID] tilt {tilt_deg:.1f} deg > {tilt_limit:.0f} "
                              f"limit ({'coast' if burnout_detected else 'boost'}) "
                              f"@t={t:.2f}s — guidance LATCHED off, roll-only")

                guidance_active = False
                if (config.guidance_enabled and guidance is not None and
                        t >= config.roll_delay_s and
                        not guidance_tilt_inhibited and
                        not guidance_cpa_reached and
                        speed > config.pn_min_speed_mps):
                    guidance_active = True

                # Fin control (roll + guidance) engages after the shared
                # activation delay, matching the firmware neutral-hold gate
                # `t_since_launch_ms >= roll_delay_ms`. speed > 5 is a rail-
                # clearance floor.
                if config.control_enabled and speed > 5.0 and t >= config.roll_delay_s:
                    rot_rate = ekf.get_rot_rate_est()
                    roll_rate_dps = math.degrees(rot_rate[0])

                    if guidance_active:
                        # ============================================
                        # GUIDED COAST MODE — 3D PN to target point
                        # ============================================

                        # 1. Build position ENU from EKF
                        ekf_pos_lla = ekf.get_position()  # (lat_rad, lon_rad, alt_m)
                        R_earth = 6378137.0
                        pos_e = (ekf_pos_lla[1] - ref_lon_rad) * R_earth * math.cos(ref_lat_rad)
                        pos_n = (ekf_pos_lla[0] - ref_lat_rad) * R_earth
                        pos_u = ekf_pos_lla[2] - config.ref_alt_m
                        pos_enu = [float(pos_e), float(pos_n), float(pos_u)]

                        # 2. Velocity NED from EKF
                        ekf_vel = ekf.get_velocity()  # (vn, ve, vd)
                        vel_ned = [float(ekf_vel[0]), float(ekf_vel[1]), float(ekf_vel[2])]

                        # 3. Quaternion from EKF (body-to-NED, scalar-first)
                        ekf_quat = ekf.get_quaternion()
                        quat_ned = [float(ekf_quat[0]), float(ekf_quat[1]),
                                    float(ekf_quat[2]), float(ekf_quat[3])]

                        # 4. PN guidance via C++ library (TR_GuidancePN)
                        #    Same textbook 3D PN (Yanushevsky Ch.2, eq 1.11/2.23)
                        #    Library outputs ENU acceleration commands.
                        target_alt = config.pn_target_alt_m

                        guid_active = guidance.update(
                            [float(pos_e), float(pos_n), float(pos_u)],
                            [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])],
                            imu_dt)

                        if guid_active:
                            a_cmd_e = guidance.get_accel_east_cmd()
                            a_cmd_n = guidance.get_accel_north_cmd()
                            a_cmd_u = guidance.get_accel_up_cmd()
                            los_angle_deg = guidance.get_los_angle_deg()
                            v_cl = guidance.get_closing_velocity()

                            if guidance.is_cpa_reached():
                                guidance_cpa_reached = True

                            a_cmd_mag = math.sqrt(a_cmd_e**2 + a_cmd_n**2 + a_cmd_u**2)

                            # Debug: print selected guidance steps
                            if not hasattr(config, '_guid_dbg_count'):
                                config._guid_dbg_count = 0
                                config._guid_dbg_t_next = 0.0
                            if (config.guidance_debug
                                    and t >= config._guid_dbg_t_next
                                    and config._guid_dbg_count < 20):
                                config._guid_dbg_count += 1
                                config._guid_dbg_t_next = t + 0.5
                                print(f"  PN[{config._guid_dbg_count:2d}] t={t:.3f}s "
                                      f"pos_enu=({pos_e:.1f}, {pos_n:.1f}, {pos_u:.1f}) "
                                      f"LOS={los_angle_deg:.2f}° v_cl={v_cl:.2f}m/s")
                                print(f"    a_cmd=({a_cmd_e:.3f}, {a_cmd_n:.3f}, {a_cmd_u:.3f}) m/s²"
                                      f"  |a|={a_cmd_mag:.3f} m/s²")
                                config._guid_dbg_print_fins = True

                            # --- Log ENU accel commands ---
                            guid_pn_a_e = a_cmd_e
                            guid_pn_a_n = a_cmd_n
                            guid_pn_a_u = a_cmd_u
                            guid_pn_los_angle = los_angle_deg
                            guid_pn_v_cl = v_cl

                            # --- Convert ENU accel to body pitch/yaw ---
                            # With heading=0 (North) and roll=0:
                            #   Body X (fwd) ≈ North+Up
                            #   Body Y (right) = East
                            #   Body Z (down) = ~toward ground
                            # So:
                            #   North accel → pitch (fins 1/3, right/left —
                            #     the elevator pair; tangential lift is
                            #     perpendicular to the fin's radial arm)
                            #   East accel  → yaw  (fins 0/2, top/bottom —
                            #     the rudder pair)
                            #
                            # Use full quaternion rotation for correctness:
                            # ENU → NED: [N, E, D] = [a_n, a_e, -a_u]
                            a_cmd_ned = np.array([a_cmd_n, a_cmd_e, -a_cmd_u])
                            q_ekf = np.array(quat_ned)
                            R_b2ned = quat_to_dcm(q_ekf)
                            a_cmd_body = R_b2ned.T @ a_cmd_ned
                            # FRD body: X=fwd, Y=right, Z=down.  Both channels
                            # take the body accel component directly, matching
                            # the FC coast-guidance path verbatim
                            # (pitch_accel = a_body_down, yaw_accel =
                            # a_body_right).  The yaw term used to carry an
                            # extra negation here, which silently cancelled the
                            # swapped fin pairing below — together they made the
                            # loop converge for the wrong reason.  With the
                            # pairing corrected the negation has to go, and the
                            # net closed-loop dynamics are unchanged.
                            pitch_accel = a_cmd_body[2]
                            yaw_accel = a_cmd_body[1]

                            # Direct accel → fin deflection (bypass PID)
                            # Scale: deg of fin per m/s² of accel command
                            accel_to_fin_deg = config.pn_accel_to_fin_deg  # reconciled to FC (was 5.0)
                            pitch_fin = accel_to_fin_deg * pitch_accel
                            yaw_fin = accel_to_fin_deg * yaw_accel

                            # Roll control (maintain zero roll, no gain sched)
                            roll_fin_cmd = roll_controller_standalone.compute(
                                config.roll_setpoint_dps, roll_rate_dps,
                                imu_dt, airspeed=None)

                            # Map to 4 fins directly (matches
                            # TR_ControlMixer's position→force mapping:
                            # pitch_mix = sin(az), yaw_mix = -cos(az)):
                            # Pitch: fins 1(right) and 3(left) differential
                            #        (elevator pair)
                            # Yaw: fins 0(top) and 2(bottom) differential,
                            #        negated (rudder pair) — the bench-
                            #        established sign, which is also what makes
                            #        a positive yaw_fin yaw the nose toward -Y
                            #        as this block's header documents.
                            # Roll: all fins same direction (common mode)
                            max_fin_deg = 20.0
                            fin_cmds = np.clip(np.array([
                                -yaw_fin   + roll_fin_cmd,    # fin 0 (top)
                                +pitch_fin + roll_fin_cmd,    # fin 1 (right)
                                +yaw_fin   + roll_fin_cmd,    # fin 2 (bottom)
                                -pitch_fin + roll_fin_cmd,    # fin 3 (left)
                            ]), -max_fin_deg, max_fin_deg)
                            guid_pitch_cmd = pitch_fin
                            guid_yaw_cmd = yaw_fin

                            # Debug: print body accel and fins
                            if (hasattr(config, '_guid_dbg_print_fins') and
                                    config._guid_dbg_print_fins):
                                config._guid_dbg_print_fins = False
                                print(f"    a_body=({a_cmd_body[0]:.3f}, "
                                      f"{a_cmd_body[1]:.3f}, "
                                      f"{a_cmd_body[2]:.3f}) FRD")
                                print(f"    pitch_accel={pitch_accel:.3f} "
                                      f"yaw_accel={yaw_accel:.3f}")
                                print(f"    fin_cmd: pitch={pitch_fin:.2f}° "
                                      f"yaw={yaw_fin:.2f}°")
                                print(f"    fins=[{fin_cmds[0]:.2f}, "
                                      f"{fin_cmds[1]:.2f}, {fin_cmds[2]:.2f}, "
                                      f"{fin_cmds[3]:.2f}]°")

                        else:
                            # Library returned inactive (too close or invalid)
                            a_cmd_body = np.array([0.0, 0.0, 0.0])
                            guid_pitch_cmd = 0.0
                            guid_yaw_cmd = 0.0
                            guid_pn_a_e = 0.0
                            guid_pn_a_n = 0.0
                            guid_pn_a_u = 0.0
                            guid_pn_los_angle = 0.0
                            guid_pn_v_cl = 0.0

                            roll_fin_cmd = roll_controller_standalone.compute(
                                config.roll_setpoint_dps, roll_rate_dps,
                                imu_dt, airspeed=None)
                            pitch_rate_dps = math.degrees(rot_rate[1])
                            yaw_rate_dps = math.degrees(rot_rate[2])
                            control_mixer.update(
                                0.0, 0.0, 0.0, 0.0,
                                pitch_rate_dps, yaw_rate_dps,
                                roll_fin_cmd, speed,
                                config.pn_kp_pitch_angle,
                                config.pn_kp_yaw_angle,
                                imu_dt)
                            fin_cmds = np.array(control_mixer.get_fin_deflections())

                    elif config.roll_profile is not None:
                        # --- Cascaded angle control ---
                        # 1. Look up target angle + segment mode per config.roll_targeting.
                        #    Waypoints are (time_s, angle_deg[, mode]), mode in
                        #    {"angle","null_rate"}.
                        _wps = config.roll_profile
                        if config.roll_targeting == "ramp":
                            # v4 firmware (roll_profile_query): null-rate before
                            # the first waypoint; lerp the target along the
                            # shortest wrapped arc between waypoints (per-wp
                            # modes ignored); hold the last angle afterwards.
                            def _wrap180(a):
                                while a > 180.0: a -= 360.0
                                while a < -180.0: a += 360.0
                                return a
                            if t < _wps[0][0]:
                                target_angle = 0.0
                                seg_mode = "null_rate"
                            elif t >= _wps[-1][0]:
                                target_angle = _wps[-1][1]
                                seg_mode = "angle"
                            else:
                                target_angle = _wps[-1][1]
                                seg_mode = "angle"
                                for i in range(len(_wps) - 1):
                                    (t0w, a0w), (t1w, a1w) = _wps[i][:2], _wps[i + 1][:2]
                                    if t < t1w:
                                        frac = ((t - t0w) / (t1w - t0w)
                                                if t1w - t0w > 1e-3 else 1.0)
                                        target_angle = _wrap180(
                                            a0w + _wrap180(a1w - a0w) * frac)
                                        break
                        elif config.roll_targeting == "endpoint" and len(_wps) > 1:
                            # Updated firmware (803d23f): command the END waypoint of
                            # the current segment; segment mode = the starting waypoint.
                            if t <= _wps[0][0]:
                                wp = _wps[0]
                                target_angle = wp[1]
                                seg_mode = wp[2] if len(wp) > 2 else "angle"
                            elif t >= _wps[-1][0]:
                                wp = _wps[-1]
                                target_angle = wp[1]
                                seg_mode = wp[2] if len(wp) > 2 else "angle"
                            else:
                                target_angle = _wps[-1][1]
                                seg_mode = "angle"
                                for i in range(len(_wps) - 1):
                                    if _wps[i][0] <= t < _wps[i + 1][0]:
                                        target_angle = _wps[i + 1][1]
                                        seg_mode = (_wps[i][2]
                                                    if len(_wps[i]) > 2 else "angle")
                                        break
                        else:
                            # Flown 6/14 firmware: hold the most-recent waypoint.
                            target_angle = _wps[0][1]
                            seg_mode = _wps[0][2] if len(_wps[0]) > 2 else "angle"
                            for wp in _wps:
                                if t >= wp[0]:
                                    target_angle = wp[1]
                                    seg_mode = wp[2] if len(wp) > 2 else "angle"
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

                        if config.use_firmware_roll_controller:
                            # Real firmware cascade. Mirror the flight call exactly —
                            # firmware passes -roll_rate_dps (= -gyro_x) as the rate
                            # arg (main.cpp). ROLL_SEG_ANGLE -> controlAngle (outer
                            # angle→rate + KP_ANGLE cap); ROLL_SEG_NULL_RATE ->
                            # controlWithGainSchedule holding ROLL_RATE_SET_POINT.
                            servo_clock_us += int(round(imu_dt * 1e6))
                            _servo_set_micros(servo_clock_us)
                            if seg_mode == "null_rate":
                                # Firmware-faithful: the rate-null path is a bare
                                # controlWithGainSchedule (main.cpp:5849). It does
                                # NOT reset the setpoint — it relies on the
                                # persistent ROLL_RATE_SET_POINT seeded at init.
                                # Pre-#372-fix, controlAngle would have left a
                                # residual rate command here, holding it instead
                                # of nulling.
                                servo.control_with_gain_schedule(
                                    -roll_rate_dps, speed)
                            else:
                                servo.control_angle(
                                    target_angle,
                                    current_roll_deg,
                                    -roll_rate_dps,
                                    speed,
                                    config.kp_angle,
                                    config.rate_cap_dps,
                                )
                            fin_tab_cmd = servo.roll_cmd_deg
                        else:
                            # Legacy Python cascade (re-implementation).
                            angle_error = target_angle - current_roll_deg
                            angle_error = (angle_error + 180.0) % 360.0 - 180.0
                            rate_setpoint = config.kp_angle * angle_error
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
            if guidance_active:
                # 4-fin servo rate-limit model; each fin follows its horn
                # through the linkage lash.
                for i in range(4):
                    delta_i = fin_cmds[i] - fin_horns[i]
                    if abs(delta_i) > max_delta:
                        delta_i = max_delta if delta_i > 0 else -max_delta
                    fin_horns[i] += delta_i
                    fin_actuals[i] = _backlash_step(
                        fin_actuals[i], fin_horns[i], config.servo_backlash_deg)
            else:
                # Roll-fin servo: PWM deadband, then a first-order lag toward the
                # command capped by the slew rate. Within the deadband the servo
                # holds; servo_tau_s<=imu_dt collapses to the prior slew limiter.
                # The deadband and dynamics act on the HORN (the servo's own
                # feedback loop); the fin follows the horn through the lash.
                err = fin_tab_cmd - servo_horn_deg
                if abs(err) >= _servo_deadband_deg:
                    rate = err / _servo_tau_eff
                    if rate > config.servo_rate_limit:
                        rate = config.servo_rate_limit
                    elif rate < -config.servo_rate_limit:
                        rate = -config.servo_rate_limit
                    servo_horn_deg += rate * imu_dt
                fin_tab_actual = _backlash_step(
                    fin_tab_actual, servo_horn_deg, config.servo_backlash_deg)

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
                'ekf_roll_rate_dps': np.degrees(ekf.get_rot_rate_est()[0]),
                'ekf_roll_bias_dps': np.degrees(ekf.get_rot_rate_bias()[0]),
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
            if current_roll_deg is not None:
                row['current_roll_deg'] = current_roll_deg

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
                # Attitude-error sigmas (deg) — index 2 ≈ yaw; used by the
                # #459 covariance-honesty check.
                cov_orient = ekf.get_cov_orient()
                row['ekf_att_sigma_x_deg'] = math.degrees(math.sqrt(max(0.0, cov_orient[0])))
                row['ekf_att_sigma_y_deg'] = math.degrees(math.sqrt(max(0.0, cov_orient[1])))
                row['ekf_att_sigma_z_deg'] = math.degrees(math.sqrt(max(0.0, cov_orient[2])))

            # Guidance telemetry
            row['guidance_active'] = guidance_active
            row['burnout_detected'] = burnout_detected
            if guidance_active and guidance is not None:
                row['guid_pitch_cmd_deg'] = guid_pitch_cmd
                row['guid_yaw_cmd_deg'] = guid_yaw_cmd
                row['guid_lateral_offset_m'] = guidance.get_lateral_offset()
                row['guid_accel_n'] = guidance.get_accel_north_cmd()
                row['guid_accel_e'] = guidance.get_accel_east_cmd()
                row['guid_target_alt'] = target_alt
                row['pn_a_e'] = guid_pn_a_e
                row['pn_a_n'] = guid_pn_a_n
                row['pn_a_u'] = guid_pn_a_u
                row['pn_los_angle'] = guid_pn_los_angle
                row['pn_v_cl'] = guid_pn_v_cl
                row['pn_pitch_cmd_deg'] = guid_pitch_cmd
                row['pn_yaw_cmd_deg'] = guid_yaw_cmd
                row['pn_a_body_fwd'] = float(a_cmd_body[0])
                row['pn_a_body_right'] = float(a_cmd_body[1])
                row['pn_a_body_down'] = float(a_cmd_body[2])
                # Fin deflections (1-indexed: 1=top, 2=right, 3=bottom, 4=left)
                for i in range(4):
                    row[f'fin{i+1}_cmd'] = float(fin_cmds[i])
                    row[f'fin{i+1}_actual'] = float(fin_actuals[i])

            log_rows.append(row)

        # --- Physics step (skip during pad phase — rocket is stationary) ---
        if not in_pad_phase:
            if guidance_active:
                state = sim.step_rk4(state, t, config.physics_dt,
                                     fin_tab_actual, wind_enu,
                                     fin_deflections=fin_actuals)
            else:
                state = sim.step_rk4(state, t, config.physics_dt,
                                     fin_tab_actual, wind_enu)
        t += config.physics_dt

        # Impulsive roll-rate kick for the controller to null (e.g. a launch /
        # staging perturbation): add roll_kick_dps to body roll rate (wx) once.
        if (config.roll_kick_dps != 0.0 and not _roll_kicked
                and t >= config.roll_kick_time_s):
            state[10] += math.radians(config.roll_kick_dps)
            _roll_kicked = True

    # Build result
    result = SimResult()
    if log_rows:
        result.df = pd.DataFrame(log_rows)
        result.apogee_m = float(max_alt)
        result.max_speed_mps = float(result.df['speed'].max())
        result.flight_time_s = float(result.df['time'].iloc[-1])

    return result
