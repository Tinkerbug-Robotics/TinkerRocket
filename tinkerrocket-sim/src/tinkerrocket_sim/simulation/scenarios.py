"""Canonical simulation scenarios for roll + guidance regression testing (#170).

Builds the validated RollyPolly III 67 mm plant and a set of canonical
SimConfigs so the same faithful vehicle is exercised by the checked-in
regression tests (`tests/test_scenarios.py`) and by the tuning sweep harness
(`scripts/tuning_sweep.py`).

Two plants live here: the validated RollyPolly III (scenarios (a)-(d)) and,
since #549, the 54 mm roll-control vehicle FITTED to its 2026-08-29 flight
(scenario (e), `build_rollypolly_54` / `RP54_FLIGHT_20260829`).

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


ROLLYPOLLY_54_ORK = os.path.join(_PKG_ROOT, "rockets", "54mm_Roll_Control.ork")

# The RP-54's servo, as the sim models it (PTK 7308 numbers shared with the
# 67 mm testbed); the lag and transport delay are FITTED per flight, see
# scripts/fit_roll_flight.py and RP54_FLIGHT_20260829 below.
RP54_SERVO = dict(rate_limit=923.0, deadband_us=2.0)

# Roll inertia of the 54 mm airframe. Not measured: the RocketDefinition default
# (the 57.4/67 mm testbeds' value) carried over. It only ever appears in the
# product Kt/I, which is what a roll log constrains, so a better number here
# rescales Kt_ref below and changes nothing the scenario reproduces.
RP54_I_ROLL = 8.3e-4
RP54_N_TABS = 3
RP54_V_REF = 95.0


# ── (e) the 2026-08-29 Rolly Polly 54 mm flight, FITTED (#549) ──────────────
# Every number below comes out of scripts/fit_roll_flight.py run on
#   TestFlights/2026_08_29 BARC L2/Rolly Polly - 54 mm/flight_20260829_164814.bin
# (F52C, apogee 247 m, 71 m/s; the only active roll-control flight that day).
# Regenerate it from the binary rather than editing it.
#
# What one flight constrains, in order:
#   - the controller: Kp/Ki identified from the logged rate and PID output
#     through the firmware law (0.32 deg RMS over 3813 ticks) — near-exact;
#   - the roll trim: the standing +1.3..+2.3 deg tab in coast cancels a built-in
#     misalignment; the coast fit puts it at -2.0..-2.3 deg;
#   - the kick: +892 dps at T+0.50 s against a tab already pinned at +20 deg,
#     a torque burst during the burn (0.36-0.50 s); the sim's impulsive
#     roll_kick_dps stands in for it, sized to the flown peak and NOT fitted
#     (left free, the optimiser shrinks it to 320 dps to buy a better first
#     swing — the wrong trade for a regression that must exercise the kick);
#   - authority, damping and the servo lag/delay together (closed-loop fit:
#     A, B, tau, delay minimise the window-normalised rate error against the
#     flight over everything after the kick; individually they trade off, as
#     a set they reproduce the ring-down and the coast — see the windows
#     below).  I_roll is not measured; it appears only in Kt/I, so it
#     rescales Kt_ref and nothing else.
#
# Fit residual vs the flight, rate RMS per window (err / flight):
#   0.15-0.62 s (the kick)     331 / 362 dps  — impulse vs a 0.15 s torque burst; reported, not scored
#   0.62-1.20 s (first swing)   69 / 104 dps  — the sim under-swings (-600 flown)
#   1.20-2.50 s (ring-down)    7.5 / 20 dps
#   2.50-4.50 s (coast)        3.5 / 8.7 dps
#   4.50-7.70 s (late coast)   1.1 / 2.8 dps
# The scenario is a REGRESSION on the ring-down and the coast; the kick window
# is exercised, not matched.  What it does not constrain: the tab's Kt(delta)
# nonlinearity (the coast fit wants ~40 % less authority at 1-2 deg than the
# ring-down does at 20 deg), the kick's true shape, I_roll on its own, and
# anything about pitch/yaw — the flight arced to 24 deg off vertical by burnout
# and the sim simply launches at that angle so the coast airspeed (hence the
# V^2 authority) matches.
RP54_FLIGHT_20260829 = dict(
    pid_kp=0.1204, pid_ki=0.0100, pid_kd=0.0,          # identified
    A=-0.1220,                # deg/s^2 per deg of tab per (m/s)^2; negative = stabilising
    misalign_deg=-2.0,        # equivalent tab angle of the built-in roll trim
    B=0.687,                  # damping K/I, 1/m
    servo_tau_s=0.0278, servo_cmd_delay_s=0.0864,
    kick_time_s=0.45, kick_dps=892.0,                  # the flown peak
    launch_angle_deg=66.0,    # trajectory proxy: the flight's 24 deg tilt by burnout
    motor="F52C",
)


def rp54_kt_ref(authority_dps_per_deg_per_ms2):
    """Sim Kt_ref (N·m/deg per tab at V_ref) from the fitted roll authority
    A = n·Kt/(I·V_ref²), A in deg/s² per deg of tab per (m/s)²."""
    return (authority_dps_per_deg_per_ms2 * np.pi / 180.0
            * RP54_I_ROLL * RP54_V_REF ** 2 / RP54_N_TABS)


def build_rollypolly_54(motor="F52C", authority=None, misalign_deg=None, damping=None):
    """The 54 mm roll-control vehicle (rockets/54mm_Roll_Control.ork), the plant
    #549 is calibrated on.

    `authority` is A as fitted by scripts/fit_roll_flight.py (negative: the tab
    opposes the rate in the gyro frame, so the firmware's rate-null law is
    stabilising — the same sign convention build_rollypolly_iii uses for its
    Kt_ref), `misalign_deg` the built-in roll trim as an equivalent tab angle,
    `damping` B = K/I in 1/m. Any left None takes the 2026-08-29 fit.
    """
    fit = RP54_FLIGHT_20260829
    rd = from_ork(ROLLYPOLLY_54_ORK)
    if motor:
        eng = find_motor(motor)
        if eng is None:
            raise ValueError(f"motor {motor!r} not found in motors/*.eng")
        rd.motor.thrust_times = np.array(eng.thrust_times, dtype=float)
        rd.motor.thrust_forces = np.array(eng.thrust_forces, dtype=float)
        rd.motor.total_mass = eng.total_mass
        rd.motor.propellant_mass = eng.propellant_mass
        rd.motor.designation = eng.designation
    rd.I_roll_launch = RP54_I_ROLL
    rd.I_roll_burnout = RP54_I_ROLL
    rd.fin_tabs.n_tabs = RP54_N_TABS
    rd.fin_tabs.V_ref = RP54_V_REF
    rd.fin_tabs.Kt_ref = rp54_kt_ref(fit["A"] if authority is None else authority)
    rd.roll_misalign_deg = fit["misalign_deg"] if misalign_deg is None else misalign_deg
    rd.roll_damping_K = (fit["B"] if damping is None else damping) * RP54_I_ROLL
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


def roll_flight_20260829_config(**overrides):
    """(e) The 2026-08-29 Rolly Polly 54 mm flight as flown, on the fitted
    plant (RP54_FLIGHT_20260829, #549): rate-null from the rail with the
    identified gains, delay 0 ms and no speed gate (the gate did not exist
    yet), the firmware's stock gain schedule, and the burn-phase kick.  Run it
    against build_rollypolly_54()."""
    f = RP54_FLIGHT_20260829
    cfg = dict(
        duration=8.6,
        launch_angle_deg=f["launch_angle_deg"],
        pid_kp=f["pid_kp"], pid_ki=f["pid_ki"], pid_kd=f["pid_kd"],
        kp_angle=2.0, rate_cap_dps=60.0, integral_sep_threshold=40.0,
        gain_V_ref=50.0, gain_V_min=25.0,         # config.h defaults the flight ran
        roll_delay_s=0.0, roll_min_speed_mps=0.0,  # as flown: engaged at 7 m/s
        roll_profile=[(0.0, 0.0, 'null_rate')],
        roll_targeting='hold',
        roll_kick_time_s=f["kick_time_s"], roll_kick_dps=f["kick_dps"],
        servo_tau_s=f["servo_tau_s"], servo_cmd_delay_s=f["servo_cmd_delay_s"],
    )
    cfg.update(overrides)
    return _base_roll_config(**cfg)


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
