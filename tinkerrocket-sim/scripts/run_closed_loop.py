#!/usr/bin/env python3
"""Run a closed-loop simulation with roll control.

Edit the variables in the CONFIG section below, then run:
    python3 scripts/run_closed_loop.py
"""
import sys
import os
import time
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from tinkerrocket_sim.rocket.definition import RocketDefinition, MotorConfig, from_ork
from tinkerrocket_sim.simulation.closed_loop_sim import run_closed_loop, SimConfig


# ============================================================================
# CONFIG — edit these to change the simulation
# ============================================================================

# --- Rocket source ---
# Option 1: Load from OpenRocket .ork file (motor thrust data from motors/*.eng)
ORK_FILE = "rockets/67mm Guidance Testbed.ork"
# Option 2: Set to None to use the manual definition in make_rocket() below
# ORK_FILE = None

# --- PID gains (inner rate loop) — flight-proven from Rolly Poly IV ---
KP = 0.04                  # proportional gain
KI = 0.001                 # integral gain
KD = 0.0003                # derivative gain

# --- Roll control mode ---
# Option A: Constant rate — set ROLL_PROFILE = None, use ROLL_SETPOINT_DPS
# Option B: Angle profile — set ROLL_PROFILE to a list of (time_s, angle_deg)
#           The controller tracks each angle until the next waypoint time.
ROLL_SETPOINT_DPS = 0.0    # desired roll rate (deg/s), only used if ROLL_PROFILE is None

# ROLL_PROFILE = [            # list of (time_s, target_angle_deg) waypoints
#     (0.0,   0.0),           # hold 0° from launch
#     (1.0, 180.0),           # at t=1.0s, roll to 180°
# ]
ROLL_PROFILE = None       # no roll during guidance debug

KP_ANGLE = 5.0             # outer angle loop gain: (deg/s rate cmd) per (deg angle error)

# --- Control ---
CONTROL_ENABLED = True      # False = passive flight (no fin tab actuation)
ROLL_DISTURBANCE_NM = 0.002  # constant roll torque disturbance (N-m)

# --- Gain scheduling — V_ref=50 gives 1.73x at 38 m/s, 0.51x at 70 m/s ---
GAIN_V_REF = 50.0           # reference airspeed for gain schedule (m/s)
GAIN_V_MIN = 25.0           # minimum airspeed for gain schedule (m/s)
GAIN_MAX_SCALE = 3.0        # max gain multiplier at low speed

# --- Actuator — PTK 7308 at 8.2V ---
DEFLECTION_MIN = -20.0      # min fin tab deflection (deg)
DEFLECTION_MAX = 20.0       # max fin tab deflection (deg)
SERVO_RATE_LIMIT = 923.0    # servo slew rate (deg/s)

# --- Wind ---
WIND_SPEED = 0.0            # m/s (0 = calm)
WIND_DIRECTION = 0.0        # direction wind comes FROM in degrees (0=N, 90=E)

# --- Simulation ---
PAD_TIME = 2.0              # pre-launch pad warmup (s) for EKF convergence
DURATION = 16.0             # sim duration after launch (s)
PHYSICS_DT = 0.001          # physics timestep (s), 0.001=1kHz, 0.0001=10kHz
LAUNCH_ANGLE_DEG = 87.0     # from horizontal (90 = straight up)
HEADING_DEG = 0.0           # launch heading (0 = North)

# --- Sensor rates ---
IMU_RATE = 1200.0           # Hz
BARO_RATE = 500.0           # Hz
MAG_RATE = 1000.0           # Hz
GNSS_RATE = 25.0            # Hz

# --- EKF debug modes ---
# These isolate different error sources for debugging the navigation filter.
# In normal operation, leave all False / True defaults.
PERFECT_IMU = False              # True = zero IMU noise & bias (truth sensor data)
INJECT_TRUTH_AT_IGNITION = False # True = reset EKF state to truth at ignition (t=0)
ENABLE_GNSS = True               # False = disable GNSS measurement updates
ENABLE_BARO = True               # False = disable barometer measurement updates
ENABLE_MAG = False                # False = disable magnetometer (no heading reference)
PAD_HEADING_DEG = 0.0            # Known heading on pad (°, 0=N). None = use mag/truth.
                                 # Default 0.0 = board Z-axis points North on the pad.
# Presets:
#   Normal flight:     PERFECT_IMU=False, INJECT=False, GNSS=True,  BARO=True,  MAG=True,  PAD=None
#   No-mag flight:     PERFECT_IMU=False, INJECT=False, GNSS=True,  BARO=True,  MAG=False, PAD=0.0
#   IMU-only:          PERFECT_IMU=False, INJECT=False, GNSS=False, BARO=False, MAG=False, PAD=0.0
#   Perfect IMU:       PERFECT_IMU=True,  INJECT=True,  GNSS=False, BARO=False, MAG=False, PAD=None

# --- Guidance — guided coast to point at target above pad ---
GUIDANCE_ENABLED = True        # True = guided coast, False = roll-only
GUIDANCE_MODE = 'pn'            # 'pn' = proportional nav, 'attitude' = point-at-target
PN_NAV_GAIN = 5.0               # navigation constant (3-5 typical)
PN_MAX_TILT_DEG = 15.0          # max body tilt command (deg)
PN_MAX_ACCEL_MPS2 = 20.0        # max lateral accel command (m/s^2)
PN_BLEND_RADIUS_M = 200.0       # PD blend radius (m) — large = always PD mode
PN_KP_POS = 0.8                 # PD position gain (s^-2)
PN_KD_VEL = 1.5                 # PD velocity gain (s^-1)
PN_KP_PITCH_ANGLE = 8.0         # outer-loop pitch angle P gain
PN_KP_YAW_ANGLE = 8.0           # outer-loop yaw angle P gain
PN_PITCH_KP = 0.06              # inner-loop pitch rate PID
PN_PITCH_KI = 0.002
PN_PITCH_KD = 0.0004
PN_YAW_KP = 0.06                # inner-loop yaw rate PID
PN_YAW_KI = 0.002
PN_YAW_KD = 0.0004
PN_MAX_FIN_DEG = 20.0           # per-fin max deflection (deg)
PN_MIN_SPEED_MPS = 10.0         # min speed for guidance (m/s)
PN_COAST_DELAY_S = 0.0          # delay after burnout (s)
PN_GAIN_V_REF = 50.0            # pitch/yaw gain-schedule V_ref (m/s)
PN_GAIN_V_MIN = 25.0            # pitch/yaw gain-schedule V_min (m/s)

# --- 3D Visualization ---
SHOW_3D = True              # True = show 3D rocket flight visualization
ANIMATE_3D = False          # True = animated playback (slower), False = static snapshots
N_SNAPSHOTS = 8             # number of rocket models along trajectory (static mode)
ANIM_SPEED = 2.0            # animation playback speed multiplier
EXPORT_HTML = True          # True = export interactive Three.js HTML visualization

# --- Output ---
SHOW_PLOT = True            # True = interactive window, False = save PNG
SAVE_CSV = None             # set to a filename like "results.csv" to export data


# ============================================================================
# ROCKET DEFINITION — used when ORK_FILE = None
# ============================================================================

def make_rocket_manual():
    """Define a rocket manually. Used when ORK_FILE is None."""
    burn_time = 2.0
    n_points = 20
    times = np.linspace(0, burn_time, n_points)
    forces = np.ones(n_points) * 40.0  # ~40 N average thrust
    forces[0] = 0.0
    forces[1] = 30.0
    forces[-2] = 25.0
    forces[-1] = 0.0

    motor = MotorConfig(
        thrust_times=times,
        thrust_forces=forces,
        total_mass=0.123,           # kg (casing + propellant)
        propellant_mass=0.06,       # kg
        designation="G40",
    )

    return RocketDefinition(
        name="TinkerRocket Mini",
        body_diameter=0.0574,       # m
        body_length=0.40,           # m
        nose_length=0.10,           # m
        dry_mass=0.770,             # kg (everything except propellant)
        Cd=0.5,
        motor=motor,
        I_roll_launch=8.3e-4,      # kg-m^2
        I_roll_burnout=8.237e-4,
        I_transverse_launch=0.02,
        I_transverse_burnout=0.019,
    )


def load_rocket():
    """Load rocket from .ork file or manual definition."""
    if ORK_FILE:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(script_dir)
        ork_path = os.path.join(project_dir, ORK_FILE)
        rd = from_ork(ork_path)
        print(f"Loaded rocket from: {ORK_FILE}")
        print(f"  Motor: {rd.motor.designation} ({len(rd.motor.thrust_times)} thrust points)")
    else:
        rd = make_rocket_manual()

    # Apply disturbance (not in .ork file)
    rd.roll_disturbance_torque = ROLL_DISTURBANCE_NM

    # Adjust stability to 3.5 cal (realistic with motor installed)
    target_stability_cal = 2.88  # with G80 motor installed
    rd.cg_from_nose = rd.cp_from_nose - target_stability_cal * rd.body_diameter

    # Set Kt_pitch/yaw from CFD Fy data:
    # CFD: Fy ≈ 47.5 mN/deg at 95 m/s per fin tab
    # Moment arm CG→fin tab CP ≈ 0.55 m (depends on CG position)
    # Kt_pitch = Fy_per_deg × arm ≈ 0.026 N-m/deg per tab
    fin_tab_cp_from_nose = (rd.nose_length + rd.body_length
                            - 0.25 * rd.fin_root_chord)  # tab at 75% chord
    moment_arm = fin_tab_cp_from_nose - rd.cg_from_nose
    Fy_per_deg_95 = 47.5e-3  # N/deg at V_ref=95 m/s (from CFD)
    Kt_pitch_cfd = Fy_per_deg_95 * moment_arm
    rd.cruciform_fins.Kt_pitch = Kt_pitch_cfd
    rd.cruciform_fins.Kt_yaw = Kt_pitch_cfd
    print(f"  Kt_pitch/yaw from CFD: {Kt_pitch_cfd:.4e} N-m/deg "
          f"(Fy={Fy_per_deg_95*1000:.1f} mN/deg, arm={moment_arm*1000:.0f} mm)")

    return rd


# ============================================================================
# PLOTTING
# ============================================================================

def plot_results(df, title, save_path=None, show=False):
    """Generate flight plots split across multiple pages.

    Page 1 — Flight Performance: altitude, speed, angle of attack
    Page 2 — Attitude & Control: pitch/yaw rates, roll rate, fin tab
    Page 3 — EKF Performance: orientation/velocity/position errors
    """
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # Derive save paths for each page
    if save_path:
        base, ext = os.path.splitext(save_path)
        save_p1 = f"{base}_flight{ext}"
        save_p2 = f"{base}_control{ext}"
        save_p3 = f"{base}_ekf{ext}"
        save_p4 = f"{base}_imu{ext}"
        save_p5 = f"{base}_guidance{ext}"
    else:
        save_p1 = save_p2 = save_p3 = save_p4 = save_p5 = None

    # Skip pages 1-4 when only guidance plots are needed
    GUIDANCE_PLOTS_ONLY = False

    # ---- Page 1: Flight Performance ----
    fig1, axes1 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig1.suptitle(f"{title} — Flight Performance")

    axes1[0].plot(df['time'], df['altitude'], 'b-')
    axes1[0].set_ylabel("Altitude (m)")
    axes1[0].grid(True, alpha=0.3)

    axes1[1].plot(df['time'], df['speed'], 'r-')
    axes1[1].set_ylabel("Speed (m/s)")
    axes1[1].grid(True, alpha=0.3)

    if 'alpha_deg' in df.columns:
        axes1[2].plot(df['time'], df['alpha_deg'], 'purple', alpha=0.7)
        axes1[2].set_ylabel("AoA (deg)")
    else:
        axes1[2].set_visible(False)
    axes1[2].set_xlabel("Time (s)")
    axes1[2].grid(True, alpha=0.3)

    fig1.tight_layout()

    if save_p1:
        fig1.savefig(save_p1, dpi=150)
        print(f"Plot saved to {save_p1}")
    if GUIDANCE_PLOTS_ONLY:
        plt.close(fig1)

    # ---- Page 2: Attitude & Control ----
    has_angle_profile = 'roll_target_deg' in df.columns
    n_panels = 4 if has_angle_profile else 3
    fig2, axes2 = plt.subplots(n_panels, 1, figsize=(12, 2.5 * n_panels + 1), sharex=True)
    fig2.suptitle(f"{title} — Attitude & Control")

    ax_idx = 0

    # Roll angle tracking (only when profile mode is active)
    if has_angle_profile:
        target = df['roll_target_deg'].values.copy()
        speed = df['speed'].values
        # Only show data after the controller activates (speed > 5 m/s)
        active_mask = (~np.isnan(target)) & (speed > 5.0)
        first_active = np.argmax(active_mask) if active_mask.any() else 0

        # Use quaternion-based roll (gimbal-lock-free) for both EKF and truth.
        # Falls back to Euler NED roll if quaternion columns not available.
        ekf_roll_col = 'ekf_roll_quat_deg' if 'ekf_roll_quat_deg' in df.columns else 'ekf_roll_deg'
        true_roll_col = 'true_roll_quat_deg' if 'true_roll_quat_deg' in df.columns else (
            'true_roll_ned_deg' if 'true_roll_ned_deg' in df.columns else 'roll_deg')

        # Unwrap target to get a continuous reference trajectory
        target_unwrap = target.copy()
        valid_target = ~np.isnan(target)
        if valid_target.any():
            first_valid = np.argmax(valid_target)
            target_unwrap[first_valid:] = np.degrees(np.unwrap(
                np.radians(target[first_valid:])))

        # Display angles on the same branch as the unwrapped target.
        # Compute wrapped error relative to target, then add back to target.
        # This avoids ±180° ambiguity without needing path-based unwrap.
        def on_target_branch(raw, tgt):
            err = (raw - tgt + 180.0) % 360.0 - 180.0  # wrap to [-180, 180]
            out = tgt + err
            out[:first_active] = np.nan
            return out

        if ekf_roll_col in df.columns:
            ekf_display = on_target_branch(
                df[ekf_roll_col].values, target_unwrap)
            axes2[ax_idx].plot(df['time'], ekf_display, 'b-',
                               label='EKF Estimate', alpha=0.7)

        true_display = on_target_branch(
            df[true_roll_col].values, target_unwrap)

        axes2[ax_idx].plot(df['time'], target_unwrap, 'r--',
                           label='Target', linewidth=2, alpha=0.8)
        axes2[ax_idx].plot(df['time'], true_display, 'g-',
                           label='True', alpha=0.5, linewidth=0.8)
        axes2[ax_idx].set_ylabel("Roll Angle (deg)")
        axes2[ax_idx].legend()
        axes2[ax_idx].grid(True, alpha=0.3)
        ax_idx += 1

    if 'pitch_rate_dps' in df.columns:
        axes2[ax_idx].plot(df['time'], df['pitch_rate_dps'], 'b-', label='Pitch', alpha=0.7)
        axes2[ax_idx].plot(df['time'], df['yaw_rate_dps'], 'r-', label='Yaw', alpha=0.7)
    axes2[ax_idx].set_ylabel("Pitch/Yaw Rate (deg/s)")
    axes2[ax_idx].legend()
    axes2[ax_idx].grid(True, alpha=0.3)
    ax_idx += 1

    axes2[ax_idx].plot(df['time'], df['roll_rate_dps'], 'b-', label='True', alpha=0.7)
    axes2[ax_idx].set_ylabel("Roll Rate (deg/s)")
    axes2[ax_idx].legend()
    axes2[ax_idx].grid(True, alpha=0.3)
    ax_idx += 1

    axes2[ax_idx].plot(df['time'], df['fin_tab_cmd'], 'g-', label='Command')
    axes2[ax_idx].plot(df['time'], df['fin_tab_actual'], 'r--', label='Actual', alpha=0.7)
    axes2[ax_idx].set_ylabel("Fin Tab (deg)")
    axes2[ax_idx].set_xlabel("Time (s)")
    axes2[ax_idx].legend()
    axes2[ax_idx].grid(True, alpha=0.3)

    fig2.tight_layout()

    if save_p2:
        fig2.savefig(save_p2, dpi=150)
        print(f"Plot saved to {save_p2}")
    if GUIDANCE_PLOTS_ONLY:
        plt.close(fig2)

    # ---- Page 3: EKF Estimation Performance ----
    has_ekf_data = all(c in df.columns for c in ['ekf_roll_deg', 'ekf_vn', 'ekf_pn'])
    fig3 = None
    if has_ekf_data:
        fig3, axes3 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        fig3.suptitle(f"{title} — EKF Estimation Error")

        # Panel 1: Orientation error via error quaternion
        # q_err = q_true^-1 ⊗ q_ekf — gives rotation from truth to EKF
        # Decompose into roll/pitch/yaw error angles (wrapping-free)
        has_quats = all(c in df.columns for c in
                        ['ekf_q0', 'ekf_q1', 'ekf_q2', 'ekf_q3',
                         'true_q0_ned', 'true_q1_ned', 'true_q2_ned', 'true_q3_ned'])
        if has_quats:
            # q_true inverse (conjugate): [w, -x, -y, -z]
            tw = df['true_q0_ned']; tx = df['true_q1_ned']
            ty = df['true_q2_ned']; tz = df['true_q3_ned']
            ew = df['ekf_q0']; ex = df['ekf_q1']
            ey = df['ekf_q2']; ez = df['ekf_q3']
            # q_err = q_true_inv ⊗ q_ekf  (Hamilton product)
            qe_w =  tw*ew + tx*ex + ty*ey + tz*ez
            qe_x = -tx*ew + tw*ex + tz*ey - ty*ez
            qe_y = -ty*ew - tz*ex + tw*ey + tx*ez
            qe_z = -tz*ew + ty*ex - tx*ey + tw*ez
            # Ensure positive scalar part (short rotation)
            sign = np.sign(qe_w).replace(0, 1)
            qe_w *= sign; qe_x *= sign; qe_y *= sign; qe_z *= sign
            # Small-angle: error ≈ 2*[qx, qy, qz] in radians
            # For larger errors, use proper Euler extraction
            roll_err = np.degrees(2.0 * qe_x)
            pitch_err = np.degrees(2.0 * qe_y)
            yaw_err = np.degrees(2.0 * qe_z)
        else:
            # Fallback to Euler difference
            roll_err = (df['ekf_roll_deg'] - df['true_roll_ned_deg'] + 180) % 360 - 180
            pitch_err = (df['ekf_pitch_deg'] - df['true_pitch_ned_deg'] + 180) % 360 - 180
            yaw_err = (df['ekf_yaw_deg'] - df['true_yaw_ned_deg'] + 180) % 360 - 180

        axes3[0].plot(df['time'], roll_err, 'r-', label='Roll', alpha=0.8)
        axes3[0].plot(df['time'], pitch_err, 'g-', label='Pitch', alpha=0.8)
        axes3[0].plot(df['time'], yaw_err, 'b-', label='Yaw', alpha=0.8)
        axes3[0].set_ylabel("Orientation Error (deg)")
        axes3[0].legend()
        axes3[0].grid(True, alpha=0.3)
        axes3[0].axhline(0, color='k', alpha=0.2)
        axes3[0].axvline(0, color='k', linestyle='--', alpha=0.3)

        # Panel 2: Velocity error (m/s) — NED frame
        vn_err = df['ekf_vn'] - df['true_vn']
        ve_err = df['ekf_ve'] - df['true_ve']
        vd_err = df['ekf_vd'] - df['true_vd']

        axes3[1].plot(df['time'], vn_err, 'r-', label='North', alpha=0.8)
        axes3[1].plot(df['time'], ve_err, 'g-', label='East', alpha=0.8)
        axes3[1].plot(df['time'], vd_err, 'b-', label='Down', alpha=0.8)
        axes3[1].set_ylabel("Velocity Error (m/s)")
        axes3[1].legend()
        axes3[1].grid(True, alpha=0.3)
        axes3[1].axhline(0, color='k', alpha=0.2)
        axes3[1].axvline(0, color='k', linestyle='--', alpha=0.3)

        # Panel 3: Position error (m) — NED frame
        pn_err = df['ekf_pn'] - df['true_pn']
        pe_err = df['ekf_pe'] - df['true_pe']
        pd_err = df['ekf_pd'] - df['true_pd']

        axes3[2].plot(df['time'], pn_err, 'r-', label='North', alpha=0.8)
        axes3[2].plot(df['time'], pe_err, 'g-', label='East', alpha=0.8)
        axes3[2].plot(df['time'], pd_err, 'b-', label='Down', alpha=0.8)
        axes3[2].set_ylabel("Position Error (m)")
        axes3[2].set_xlabel("Time (s)")
        axes3[2].legend()
        axes3[2].grid(True, alpha=0.3)
        axes3[2].axhline(0, color='k', alpha=0.2)
        axes3[2].axvline(0, color='k', linestyle='--', alpha=0.3)

        fig3.tight_layout()

        if save_p3:
            fig3.savefig(save_p3, dpi=150)
            print(f"Plot saved to {save_p3}")
        if GUIDANCE_PLOTS_ONLY:
            plt.close(fig3)

    # ---- Page 4: IMU Accelerometer vs Truth ----
    fig4 = None
    has_imu_data = 'imu_acc_x' in df.columns and 'true_acc_x' in df.columns
    if has_imu_data:
        fig4, axes4 = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        fig4.suptitle(f"{title} — IMU Accelerometer vs Truth (Body FRD)")

        axis_labels = [
            ('imu_acc_x', 'true_acc_x', 'Forward (X)'),
            ('imu_acc_y', 'true_acc_y', 'Right (Y)'),
            ('imu_acc_z', 'true_acc_z', 'Down (Z)'),
        ]

        for i, (imu_col, true_col, label) in enumerate(axis_labels):
            axes4[i].plot(df['time'], df[imu_col], 'b-', alpha=0.4,
                          linewidth=0.5, label='IMU (noisy)')
            axes4[i].plot(df['time'], df[true_col], 'r-', alpha=0.8,
                          linewidth=0.8, label='Truth')
            axes4[i].set_ylabel(f"{label} (m/s²)")
            axes4[i].legend(loc='upper right', fontsize=8)
            axes4[i].grid(True, alpha=0.3)
            axes4[i].axvline(0, color='k', linestyle='--', alpha=0.3)

        # Panel 4: Magnitude comparison
        imu_mag = np.sqrt(df['imu_acc_x']**2 + df['imu_acc_y']**2 + df['imu_acc_z']**2)
        true_mag = np.sqrt(df['true_acc_x']**2 + df['true_acc_y']**2 + df['true_acc_z']**2)
        axes4[3].plot(df['time'], imu_mag, 'b-', alpha=0.4, linewidth=0.5, label='IMU |a|')
        axes4[3].plot(df['time'], true_mag, 'r-', alpha=0.8, linewidth=0.8, label='Truth |a|')
        axes4[3].set_ylabel("|Accel| (m/s²)")
        axes4[3].set_xlabel("Time (s)")
        axes4[3].legend(loc='upper right', fontsize=8)
        axes4[3].grid(True, alpha=0.3)
        axes4[3].axvline(0, color='k', linestyle='--', alpha=0.3)

        fig4.tight_layout()

        if save_p4:
            fig4.savefig(save_p4, dpi=150)
            print(f"Plot saved to {save_p4}")
        if GUIDANCE_PLOTS_ONLY:
            plt.close(fig4)

    # ---- Page 5: PN Guidance + Fin Control ----
    fig5 = None
    has_pn = 'pn_a_n' in df.columns
    has_fins = 'fin1_actual' in df.columns
    guided = df[df['guidance_active'] == True] if 'guidance_active' in df.columns else df.iloc[0:0]
    # Show page if we have guidance data OR fin data
    if (has_pn and len(guided) > 0) or has_fins:
            fig5, axes5 = plt.subplots(3, 2, figsize=(16, 12))
            fig5.suptitle(f"{title} — PN Guidance + Fin Control")

            # Panel (0,0): PN accel commands ENU
            ax = axes5[0, 0]
            if has_pn and len(guided) > 0:
                ax.plot(guided['time'], guided['pn_a_e'].fillna(0),
                        'r-', label='East', alpha=0.8)
                ax.plot(guided['time'], guided['pn_a_n'].fillna(0),
                        'g-', label='North', alpha=0.8)
                ax.plot(guided['time'], guided['pn_a_u'].fillna(0),
                        'b-', label='Up', alpha=0.8)
                a_mag = np.sqrt(guided['pn_a_e'].fillna(0)**2 +
                                guided['pn_a_n'].fillna(0)**2 +
                                guided['pn_a_u'].fillna(0)**2)
                ax.plot(guided['time'], a_mag, 'k--', label='|a|', alpha=0.5)
            ax.set_ylabel("PN Accel Cmd ENU (m/s²)")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.axhline(0, color='k', alpha=0.2)

            # Panel (0,1): Commanded body-frame accel (FRD)
            ax = axes5[0, 1]
            if has_pn and len(guided) > 0:
                if 'pn_a_body_fwd' in guided.columns:
                    ax.plot(guided['time'], guided['pn_a_body_fwd'].fillna(0),
                            'r-', label='Cmd Fwd (X)', alpha=0.8)
                    ax.plot(guided['time'], guided['pn_a_body_right'].fillna(0),
                            'g-', label='Cmd Right (Y)', alpha=0.8)
                    ax.plot(guided['time'], guided['pn_a_body_down'].fillna(0),
                            'b-', label='Cmd Down (Z)', alpha=0.8)
                # Overlay true body accel (FLU→FRD: negate Y,Z)
                if 'true_acc_x' in guided.columns:
                    ax.plot(guided['time'], guided['true_acc_x'].fillna(0),
                            'r--', label='Actual Fwd', alpha=0.4)
                    ax.plot(guided['time'], -guided['true_acc_y'].fillna(0),
                            'g--', label='Actual Right', alpha=0.4)
                    ax.plot(guided['time'], -guided['true_acc_z'].fillna(0),
                            'b--', label='Actual Down', alpha=0.4)
            ax.set_ylabel("Body-Frame Accel (m/s²)")
            ax.set_title("Cmd (solid) vs Actual (dashed) — FRD")
            ax.legend(fontsize=7, ncol=2)
            ax.grid(True, alpha=0.3)
            ax.axhline(0, color='k', alpha=0.2)

            # Panel (1,0): LOS angle + Closing velocity (dual axis)
            ax = axes5[1, 0]
            if has_pn and len(guided) > 0:
                ax.plot(guided['time'], guided['pn_los_angle'].fillna(0),
                        'b-', linewidth=1.5, label='LOS angle')
                ax.set_ylabel("LOS Angle (°)", color='b')
                ax2 = ax.twinx()
                ax2.plot(guided['time'], guided['pn_v_cl'].fillna(0),
                         'r-', linewidth=1.5, alpha=0.7, label='v_cl')
                ax2.set_ylabel("Closing Velocity (m/s)", color='r')
                lines1, labels1 = ax.get_legend_handles_labels()
                lines2, labels2 = ax2.get_legend_handles_labels()
                ax.legend(lines1 + lines2, labels1 + labels2, fontsize=8)
            ax.grid(True, alpha=0.3)

            # Panel (1,1): Fin deflections (1-4) — use full df
            ax = axes5[1, 1]
            fin_colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3']
            fin_labels = ['Fin 1 Top (pitch+)',
                          'Fin 2 Right (yaw+)',
                          'Fin 3 Bottom (pitch-)',
                          'Fin 4 Left (yaw-)']
            for i in range(4):
                col = f'fin{i+1}_actual'
                if col in df.columns:
                    ax.plot(df['time'], df[col].fillna(0),
                            color=fin_colors[i], label=fin_labels[i],
                            linewidth=1.2, alpha=0.8)
            ax.set_ylabel("Fin Deflection (°)")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.axhline(0, color='k', alpha=0.2)

            # Panel (2,0): Position N/E (to correlate with accel direction)
            ax = axes5[2, 0]
            if 'ekf_pe' in df.columns:
                ax.plot(df['time'], df['ekf_pe'].fillna(0),
                        'r-', label='East', alpha=0.8)
                ax.plot(df['time'], df['ekf_pn'].fillna(0),
                        'g-', label='North', alpha=0.8)
            if len(guided) > 0:
                t_start = guided['time'].iloc[0]
                t_end = guided['time'].iloc[-1]
                ax.axvspan(t_start, t_end, alpha=0.1, color='green',
                           label='Guidance active')
            ax.set_ylabel("Position (m)")
            ax.set_xlabel("Time (s)")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.axhline(0, color='k', alpha=0.2)

            # Panel (2,1): Orientation (EKF pitch/yaw)
            ax = axes5[2, 1]
            ax.plot(df['time'], df['ekf_pitch_deg'], 'r-',
                    label='EKF Pitch', alpha=0.6, linewidth=0.8)
            ax.plot(df['time'], df['ekf_yaw_deg'], 'b-',
                    label='EKF Yaw', alpha=0.6, linewidth=0.8)
            # Mark guidance window
            if len(guided) > 0:
                t_start = guided['time'].iloc[0]
                t_end = guided['time'].iloc[-1]
                ax.axvspan(t_start, t_end, alpha=0.1, color='green',
                           label='Guidance active')
            ax.set_ylabel("EKF Euler Angle (°)")
            ax.set_xlabel("Time (s)")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

            fig5.tight_layout()

            if save_p5:
                fig5.savefig(save_p5, dpi=150)
                print(f"Plot saved to {save_p5}")

    if show:
        plt.show()
    else:
        plt.close(fig1)
        plt.close(fig2)
        if fig3 is not None:
            plt.close(fig3)
        if fig4 is not None:
            plt.close(fig4)
        if fig5 is not None:
            plt.close(fig5)


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    rd = load_rocket()

    cfg = SimConfig(
        pad_time=PAD_TIME,
        duration=DURATION,
        physics_dt=PHYSICS_DT,
        launch_angle_deg=LAUNCH_ANGLE_DEG,
        heading_deg=HEADING_DEG,
        wind_speed=WIND_SPEED,
        wind_direction_deg=WIND_DIRECTION,
        imu_rate=IMU_RATE,
        baro_rate=BARO_RATE,
        mag_rate=MAG_RATE,
        gnss_rate=GNSS_RATE,
        control_enabled=CONTROL_ENABLED,
        pid_kp=KP,
        pid_ki=KI,
        pid_kd=KD,
        roll_setpoint_dps=ROLL_SETPOINT_DPS,
        roll_profile=ROLL_PROFILE,
        kp_angle=KP_ANGLE,
        deflection_min=DEFLECTION_MIN,
        deflection_max=DEFLECTION_MAX,
        servo_rate_limit=SERVO_RATE_LIMIT,
        gain_V_ref=GAIN_V_REF,
        gain_V_min=GAIN_V_MIN,
        gain_max_scale=GAIN_MAX_SCALE,
        # Guidance
        guidance_enabled=GUIDANCE_ENABLED,
        guidance_mode=GUIDANCE_MODE,
        pn_nav_gain=PN_NAV_GAIN,
        pn_max_tilt_deg=PN_MAX_TILT_DEG,
        pn_max_accel_mps2=PN_MAX_ACCEL_MPS2,
        pn_blend_radius_m=PN_BLEND_RADIUS_M,
        pn_kp_pos=PN_KP_POS,
        pn_kd_vel=PN_KD_VEL,
        pn_kp_pitch_angle=PN_KP_PITCH_ANGLE,
        pn_kp_yaw_angle=PN_KP_YAW_ANGLE,
        pn_pitch_kp=PN_PITCH_KP,
        pn_pitch_ki=PN_PITCH_KI,
        pn_pitch_kd=PN_PITCH_KD,
        pn_yaw_kp=PN_YAW_KP,
        pn_yaw_ki=PN_YAW_KI,
        pn_yaw_kd=PN_YAW_KD,
        pn_max_fin_deg=PN_MAX_FIN_DEG,
        pn_min_speed_mps=PN_MIN_SPEED_MPS,
        pn_coast_delay_s=PN_COAST_DELAY_S,
        pn_gain_V_ref=PN_GAIN_V_REF,
        pn_gain_V_min=PN_GAIN_V_MIN,
        # EKF debug modes
        perfect_imu=PERFECT_IMU,
        inject_truth_orientation_at_ignition=INJECT_TRUTH_AT_IGNITION,
        enable_gnss_updates=ENABLE_GNSS,
        enable_baro_updates=ENABLE_BARO,
        enable_mag_updates=ENABLE_MAG,
        pad_heading_deg=PAD_HEADING_DEG,
    )

    # Print config summary
    control_str = f"Kp={KP}, Ki={KI}, Kd={KD}" if CONTROL_ENABLED else "DISABLED"
    print(f"\nClosed-loop simulation: {rd.name}")
    print(f"  Dry mass: {rd.dry_mass*1000:.1f} g | Launch mass: {rd.total_mass_launch*1000:.1f} g")
    print(f"  Physics: {1/PHYSICS_DT:.0f} Hz | IMU: {IMU_RATE:.0f} Hz | GNSS: {GNSS_RATE:.0f} Hz")
    print(f"  Control: {control_str}")
    if ROLL_PROFILE is not None:
        print(f"  Roll profile: {len(ROLL_PROFILE)} waypoints, Kp_angle={KP_ANGLE}")
        for wp_t, wp_a in ROLL_PROFILE:
            print(f"    t={wp_t:.1f}s → {wp_a:.0f}°")
    elif ROLL_SETPOINT_DPS != 0:
        print(f"  Roll setpoint: {ROLL_SETPOINT_DPS:.1f} deg/s")

    # Aerodynamic info
    if rd.CNa_total > 0:
        stability_cal = (rd.cp_from_nose - rd.cg_from_nose) / rd.body_diameter
        print(f"  CNa: {rd.CNa_total:.2f} /rad | CP: {rd.cp_from_nose*1000:.1f}mm | "
              f"CG: {rd.cg_from_nose*1000:.1f}mm | SM: {stability_cal:.1f} cal")

    if ROLL_DISTURBANCE_NM != 0.0:
        print(f"  Roll disturbance: {ROLL_DISTURBANCE_NM*1000:.1f} mN-m")
    if GUIDANCE_ENABLED:
        print(f"  Guidance: PN (N={PN_NAV_GAIN}, max_tilt={PN_MAX_TILT_DEG}°, "
              f"max_fin={PN_MAX_FIN_DEG}°)")
    else:
        print(f"  Guidance: OFF (roll-only)")
    if WIND_SPEED > 0:
        print(f"  Wind: {WIND_SPEED:.1f} m/s from {WIND_DIRECTION:.0f}°")

    # EKF mode summary
    ekf_flags = []
    if PERFECT_IMU:
        ekf_flags.append("perfect-IMU")
    if INJECT_TRUTH_AT_IGNITION:
        ekf_flags.append("truth-inject@ignition")
    if not ENABLE_GNSS:
        ekf_flags.append("no-GNSS")
    if not ENABLE_BARO:
        ekf_flags.append("no-baro")
    if not ENABLE_MAG:
        ekf_flags.append("no-mag")
    if PAD_HEADING_DEG is not None:
        ekf_flags.append(f"pad-heading={PAD_HEADING_DEG:.0f}°")
    if ekf_flags:
        print(f"  EKF mode: {', '.join(ekf_flags)}")
    else:
        print(f"  EKF mode: normal (all sensors)")

    print(f"  Pad warmup: {PAD_TIME:.1f}s | Launch angle: {LAUNCH_ANGLE_DEG}° | Duration: {DURATION}s")

    # Run
    t0 = time.time()
    result = run_closed_loop(rd, cfg)
    elapsed = time.time() - t0

    # Trim to apogee — discard descent phase
    df = result.df
    apogee_idx = df['altitude'].idxmax()
    df = df.iloc[:apogee_idx + 1].reset_index(drop=True)
    result.df = df
    print(f"  Trimmed to apogee at T+{df['time'].iloc[-1]:.2f}s ({len(df)} rows)")

    # Results
    print(f"\nResults:")
    print(f"  Apogee:      {result.apogee_m:.1f} m")
    print(f"  Max speed:   {result.max_speed_mps:.1f} m/s")
    print(f"  Flight time: {result.flight_time_s:.1f} s")
    print(f"  Wall-clock:  {elapsed:.1f} s")
    print(f"  Data points: {len(df)}")

    # Roll control stats
    burn_time = rd.motor.burn_time
    if 'roll_rate_dps' in df.columns:
        boost = df[(df['time'] > 0.5) & (df['time'] < burn_time)]
        if len(boost) > 0:
            print(f"\n  Boost roll rate:  mean={boost['roll_rate_dps'].mean():+.2f}, "
                  f"std={boost['roll_rate_dps'].std():.2f} deg/s")
            print(f"  Boost fin tab:    mean={boost['fin_tab_cmd'].mean():+.2f}, "
                  f"|max|={boost['fin_tab_cmd'].abs().max():.2f} deg")

        coast = df[(df['time'] > burn_time + 0.5) & (df['time'] < 8.0)]
        if len(coast) > 0:
            print(f"  Coast roll rate:  mean={coast['roll_rate_dps'].mean():+.2f}, "
                  f"std={coast['roll_rate_dps'].std():.2f} deg/s")

    # Roll angle profile tracking stats
    ekf_roll_stat = 'ekf_roll_quat_deg' if 'ekf_roll_quat_deg' in df.columns else 'ekf_roll_deg'
    if 'roll_target_deg' in df.columns and ekf_roll_stat in df.columns:
        active = df[df['speed'] > 5.0]
        if len(active) > 0:
            angle_err = active['roll_target_deg'] - active[ekf_roll_stat]
            # Wrap to [-180, 180]
            angle_err = (angle_err + 180.0) % 360.0 - 180.0
            print(f"\n  Roll angle tracking:")
            print(f"    Mean error:  {angle_err.mean():+.2f}°")
            print(f"    Std error:   {angle_err.std():.2f}°")
            print(f"    Max |error|: {angle_err.abs().max():.2f}°")

    # AoA stats
    if 'alpha_deg' in df.columns:
        flight = df[df['speed'] > 5.0]
        if len(flight) > 0:
            print(f"\n  Max AoA:    {flight['alpha_deg'].max():.2f}°")
            boost = flight[flight['time'] < burn_time]
            if len(boost) > 0:
                print(f"  Boost AoA:  mean={boost['alpha_deg'].mean():.2f}°, "
                      f"max={boost['alpha_deg'].max():.2f}°")

    # Pitch/yaw stats
    if 'pitch_rate_dps' in df.columns:
        print(f"  Max pitch rate: {df['pitch_rate_dps'].abs().max():.1f} deg/s")
        print(f"  Max yaw rate:   {df['yaw_rate_dps'].abs().max():.1f} deg/s")

    # Guidance stats
    if GUIDANCE_ENABLED and 'guidance_active' in df.columns:
        guided = df[df['guidance_active'] == True]
        if len(guided) > 0:
            print(f"\n  Guidance active: {guided['time'].iloc[0]:.2f}s - {guided['time'].iloc[-1]:.2f}s")
            if 'guid_lateral_offset_m' in guided.columns:
                print(f"  Max lateral offset: {guided['guid_lateral_offset_m'].max():.1f}m")
                print(f"  Final lateral offset: {guided['guid_lateral_offset_m'].iloc[-1]:.1f}m")
            if 'pn_pitch_cmd_deg' in guided.columns:
                print(f"  Max pitch cmd: {guided['pn_pitch_cmd_deg'].abs().max():.2f}°")
                print(f"  Max yaw cmd: {guided['pn_yaw_cmd_deg'].abs().max():.2f}°")
            # Fin deflection stats
            for i in range(4):
                col = f'fin{i+1}_actual'
                if col in guided.columns:
                    vals = guided[col].fillna(0)
                    print(f"  Fin {i+1} deflection: "
                          f"min={vals.min():.2f}°, max={vals.max():.2f}°")

    # Landing position / drift
    if len(df) > 0:
        end = df.iloc[-1]
        drift = np.sqrt(end['x']**2 + end['y']**2)
        if drift > 1.0:
            print(f"\n  Landing:   E={end['x']:.1f}m, N={end['y']:.1f}m (drift={drift:.1f}m)")

    # Save CSV
    if SAVE_CSV:
        df.to_csv(SAVE_CSV, index=False)
        print(f"\nData saved to {SAVE_CSV}")

    # Plot
    title = f"Closed-Loop Simulation — {rd.name}"
    if not CONTROL_ENABLED:
        title += " (passive)"
    else:
        title += f" (Kp={KP}, Ki={KI}, Kd={KD})"
    if PERFECT_IMU:
        title += " [Perfect IMU]"
    elif INJECT_TRUTH_AT_IGNITION:
        title += " [Truth Inject]"
    if not ENABLE_GNSS and not ENABLE_BARO:
        title += " [IMU-Only Nav]"

    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)
    png_path = os.path.join(project_dir, "closed_loop_test.png") if not SHOW_PLOT else None

    try:
        plot_results(df, title, save_path=png_path, show=SHOW_PLOT)
    except Exception as e:
        print(f"Plotting failed: {e}")

    # 3D visualization
    if SHOW_3D:
        try:
            from tinkerrocket_sim.visualization.rocket_3d import build_rocket_mesh
            from tinkerrocket_sim.visualization.flight_3d import plot_flight_3d, animate_flight_3d

            print("\nGenerating 3D visualization...")
            mesh = build_rocket_mesh(rd)

            save_3d = None
            if not SHOW_PLOT and save_path:
                base, ext = os.path.splitext(save_path)
                save_3d = f"{base}_3d{ext}"

            if ANIMATE_3D:
                anim_path = None
                if not SHOW_PLOT and save_path:
                    base, _ = os.path.splitext(save_path)
                    anim_path = f"{base}_3d.gif"
                animate_flight_3d(df, mesh, speed=ANIM_SPEED,
                                  title=f"{rd.name} — 3D Flight",
                                  save_path=anim_path, show=SHOW_PLOT)
            else:
                plot_flight_3d(df, mesh, n_snapshots=N_SNAPSHOTS,
                               title=f"{rd.name} — 3D Flight",
                               save_path=save_3d, show=SHOW_PLOT)
        except Exception as e:
            print(f"3D visualization failed: {e}")
            import traceback
            traceback.print_exc()

    # Interactive HTML visualization
    if EXPORT_HTML:
        try:
            from tinkerrocket_sim.visualization.rocket_3d import build_rocket_mesh
            from tinkerrocket_sim.visualization.flight_html import export_flight_html

            if 'mesh' not in dir():
                mesh = build_rocket_mesh(rd)

            html_path = os.path.join(project_dir, "flight_3d.html")
            export_flight_html(df, mesh, output_path=html_path,
                               title=f"{rd.name} — Interactive Flight")
        except Exception as e:
            print(f"HTML export failed: {e}")
            import traceback
            traceback.print_exc()
