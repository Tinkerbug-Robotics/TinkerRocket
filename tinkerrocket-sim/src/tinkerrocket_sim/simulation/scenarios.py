"""Canonical simulation scenarios for roll + guidance regression testing (#170).

Builds the validated RollyPolly III 67 mm plant and a set of canonical
SimConfigs so the same faithful vehicle is exercised by the checked-in
regression tests (`tests/test_scenarios.py`) and by the tuning sweep harness
(`scripts/tuning_sweep.py`).

Plant recipe (from the SIL roll-control work behind PR #252):
  - geometry/aero from RollyPolly_III_67mm.ork
  - motor swapped to G80T (the .ork default is G74W — far too weak)
  - 4 fin tabs, roll-tab Kt sign NEGATIVE in the sim body frame so the real
    firmware controller (negative feedback in the firmware convention) is
    stabilizing (see the SIL roll-sign convention notes)
  - roll damping + a small fin-tab build misalignment reproduce the flown
    boost-roll behavior
"""
import os

import numpy as np

from ..rocket.definition import from_ork
from ..rocket.motor_db import find_motor
from .closed_loop_sim import SimConfig

# rockets/ lives at the package-tree root (two levels above src/…/simulation/).
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
ROLLYPOLLY_ORK = os.path.join(_PKG_ROOT, "rockets", "RollyPolly_III_67mm.ork")

# Flight-proven roll-control tune shipped to firmware (config.h, PR #252).
FLOWN_ROLL_GAINS = dict(
    pid_kp=0.02, pid_ki=0.03, pid_kd=0.0,
    kp_angle=2.0, rate_cap_dps=60.0, integral_sep_threshold=40.0,
)


def build_rollypolly_iii(motor="G80T", firmware_roll_sign=True):
    """Return the validated RollyPolly III RocketDefinition.

    firmware_roll_sign=True sets the roll-tab Kt negative so the real firmware
    roll controller (use_firmware_roll_controller=True) is stabilizing; the
    guided path uses the cruciform fins and is sign-agnostic here.
    """
    rd = from_ork(ROLLYPOLLY_ORK)

    if motor:
        eng = find_motor(motor)
        if eng is None:
            raise ValueError(f"motor {motor!r} not found in motors/*.eng")
        rd.motor.thrust_times = np.array(eng.thrust_times, dtype=float)
        rd.motor.thrust_forces = np.array(eng.thrust_forces, dtype=float)
        rd.motor.total_mass = eng.total_mass
        rd.motor.propellant_mass = eng.propellant_mass
        rd.motor.designation = eng.designation

    # Mass / inertia (RollyPolly III measured; the .ork transverse MOI is low).
    rd.I_roll_launch = 2.0e-3
    rd.I_roll_burnout = 2.0e-3
    rd.I_transverse_launch = 0.10
    rd.I_transverse_burnout = 0.10

    # Roll-tab authority (CFD LS slope at V_ref=95 m/s, per tab).
    rd.fin_tabs.n_tabs = 4
    rd.fin_tabs.V_ref = 95.0
    rd.fin_tabs.Kt_ref = -5.34e-3 if firmware_roll_sign else 5.34e-3

    # Boost-roll plant: aerodynamic roll damping (Cl_p) + a small fin-tab build
    # misalignment that drives the V^2 disturbance seen on the flight.
    rd.roll_damping_K = 8.0e-5
    rd.roll_misalign_deg = -0.46

    # Cruciform pitch/yaw authority from CFD (used by the guided scenario).
    fin_tab_cp_from_nose = (rd.nose_length + rd.body_length
                            - 0.25 * rd.fin_root_chord)
    moment_arm = fin_tab_cp_from_nose - rd.cg_from_nose
    Kt_pitch = 47.5e-3 * moment_arm  # Fy≈47.5 mN/deg per tab at 95 m/s × arm
    rd.cruciform_fins.Kt_pitch = Kt_pitch
    rd.cruciform_fins.Kt_yaw = Kt_pitch

    return rd


def _base_roll_config(**overrides):
    """SimConfig common to the roll scenarios: real firmware controller, a
    settled EKF (long pad warmup so the gyro-bias transient decays), mag off
    with a known pad heading."""
    cfg = dict(
        pad_time=10.0,          # let the EKF gyro-bias estimate settle
        duration=8.0,
        physics_dt=1e-3,
        launch_angle_deg=87.0,
        control_enabled=True,
        use_firmware_roll_controller=True,
        roll_gain_schedule_enabled=True,
        guidance_enabled=False,
        enable_mag_updates=False,
        pad_heading_deg=0.0,
        sensor_seed=42,         # deterministic sensor noise for repeatable CI
    )
    cfg.update(FLOWN_ROLL_GAINS)
    cfg.update(overrides)
    return SimConfig(**cfg)


def roll_tracking_clean_config():
    """(b) Roll-angle profile tracking, no disturbance — sanity check that the
    controller holds a commanded angle. Null-rate to t=2.5 s, then hold 0°."""
    return _base_roll_config(
        roll_profile=[(0.0, 0.0, 'null_rate'), (2.5, 0.0, 'angle')],
        roll_targeting='hold',
    )


def roll_boost_disturbance_config():
    """(c) Roll tracking under a boost-roll perturbation — the 2026-05-17-like
    regression. A ~120 dps kick early in boost (plus the standing fin
    misalignment) that the controller must null."""
    return _base_roll_config(
        roll_profile=[(0.0, 0.0, 'null_rate'), (3.0, 0.0, 'angle')],
        roll_targeting='hold',
        roll_kick_time_s=0.5,
        roll_kick_dps=120.0,
    )


def sensor_degradation_config():
    """(d) Sensor degradation — gyro near full-scale saturation AND GNSS
    dropout under high-g/high-spin, on top of the boost-roll kick. The filter +
    controller must still keep the vehicle bounded (graceful degradation)."""
    return _base_roll_config(
        roll_profile=[(0.0, 0.0, 'null_rate'), (3.0, 0.0, 'angle')],
        roll_targeting='hold',
        roll_kick_time_s=0.5,
        roll_kick_dps=120.0,
        gyro_full_scale_dps=100.0,   # below the 120 dps kick, so the gyro clips
        enable_gnss_updates=True,    # dropout model still active in the sensor
    )


def guidance_coast_config():
    """(a) Nominal coast guidance to an overhead aim point — the real
    TR_GuidancePN cascade steers the vehicle back toward the pad axis during
    coast (CPA convergence). Vertical-ish launch with a small off-vertical
    angle so guidance has an error to null."""
    return SimConfig(
        pad_time=2.0,
        duration=16.0,
        physics_dt=1e-3,
        launch_angle_deg=85.0,      # 5° off vertical -> lateral drift to correct
        heading_deg=0.0,
        control_enabled=True,
        guidance_enabled=True,
        guidance_mode='pn',
        enable_mag_updates=False,
        pad_heading_deg=0.0,
        sensor_seed=7,
        # PN + pitch/yaw cascade (reconciled to firmware config).
        pn_nav_gain=5.0,
        pn_max_tilt_deg=15.0,
        pn_max_accel_mps2=20.0,
        pn_target_alt_m=600.0,
        pn_accel_to_fin_deg=4.0,
        pn_blend_radius_m=200.0,
        pn_kp_pos=0.8,
        pn_kd_vel=1.5,
        pn_kp_pitch_angle=8.0,
        pn_kp_yaw_angle=8.0,
        pn_pitch_kp=0.06, pn_pitch_ki=0.002, pn_pitch_kd=0.0004,
        pn_yaw_kp=0.06, pn_yaw_ki=0.002, pn_yaw_kd=0.0004,
        pn_max_fin_deg=20.0,
        pn_min_speed_mps=10.0,
        pn_gain_V_ref=50.0,
        pn_gain_V_min=25.0,
    )
