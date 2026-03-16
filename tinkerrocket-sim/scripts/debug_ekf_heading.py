#!/usr/bin/env python3
"""Debug EKF heading / body frame convention issues.

Dumps EKF quaternion, attitude, velocity, and bias at key timesteps
to identify the root cause of excess velocity error during boost.
"""

import sys, math
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from tinkerrocket_sim.rocket.definition import from_ork
from tinkerrocket_sim.simulation.closed_loop_sim import SimConfig, run_closed_loop
from tinkerrocket_sim.physics.sixdof import quat_to_dcm, quat_to_euler

# Load rocket
rocket_dir = Path(__file__).parent.parent / "rockets"
ork_file = rocket_dir / "54mm_Roll_Control.ork"
rocket = from_ork(ork_file)

# Simple config: no roll, GNSS enabled, 2s pad
cfg = SimConfig(
    duration=6.0,
    pad_time=2.0,
    launch_angle_deg=89.0,
    heading_deg=0.0,
    pid_kp=0.04,
    pid_ki=0.001,
    pid_kd=0.001,
    roll_profile=None,
    roll_setpoint_dps=0.0,
    kp_angle=5.0,
    enable_gnss_updates=True,
    enable_baro_updates=True,
)

print("Running sim...")
res = run_closed_loop(rocket, cfg)
df = res.df

# Key time points to examine
times = [-1.5, -1.0, -0.5, -0.1, 0.0, 0.1, 0.2, 0.5, 0.8, 1.0, 1.3, 1.5, 2.0, 3.0, 4.0]

print("\n" + "=" * 120)
print("EKF Attitude & Velocity Diagnostic")
print("=" * 120)

# Header
print(f"{'t':>6s} | {'phase':>7s} | {'GNSS':>4s} | "
      f"{'ekf_roll':>9s} {'ekf_pitch':>10s} {'ekf_yaw':>9s} | "
      f"{'tru_roll':>9s} {'tru_pitch':>10s} {'tru_yaw':>9s} | "
      f"{'ekf_vn':>8s} {'ekf_ve':>8s} {'ekf_vd':>8s} | "
      f"{'tru_vn':>8s} {'tru_ve':>8s} {'tru_vd':>8s} | "
      f"{'err_vn':>8s} {'err_ve':>8s} {'err_vd':>8s}")
print("-" * 120)

for ts in times:
    mask = df['time'] >= ts - 0.01
    if not mask.any():
        continue
    idx = (df['time'] - ts).abs().idxmin()
    r = df.loc[idx]

    t = r['time']
    phase = r.get('flight_phase', '?')
    gnss = 'Y' if r.get('gnss_valid', False) else 'N'

    # EKF Euler angles
    ekf_r = r.get('ekf_roll_deg', 0)
    ekf_p = r.get('ekf_pitch_deg', 0)
    ekf_y = r.get('ekf_yaw_deg', 0)

    # Truth Euler angles (NED convention, directly comparable to EKF)
    tru_r = r.get('true_roll_ned_deg', 0)
    tru_p = r.get('true_pitch_ned_deg', 0)
    tru_y = r.get('true_yaw_ned_deg', 0)

    # Velocities
    ekf_vn = r.get('ekf_vn', 0)
    ekf_ve = r.get('ekf_ve', 0)
    ekf_vd = r.get('ekf_vd', 0)
    tru_vn = r.get('true_vn', 0)
    tru_ve = r.get('true_ve', 0)
    tru_vd = r.get('true_vd', 0)

    err_vn = ekf_vn - tru_vn
    err_ve = ekf_ve - tru_ve
    err_vd = ekf_vd - tru_vd

    print(f"{t:6.2f} | {phase:>7s} | {gnss:>4s} | "
          f"{ekf_r:9.2f} {ekf_p:10.2f} {ekf_y:9.2f} | "
          f"{tru_r:9.2f} {tru_p:10.2f} {tru_y:9.2f} | "
          f"{ekf_vn:8.2f} {ekf_ve:8.2f} {ekf_vd:8.2f} | "
          f"{tru_vn:8.2f} {tru_ve:8.2f} {tru_vd:8.2f} | "
          f"{err_vn:8.2f} {err_ve:8.2f} {err_vd:8.2f}")

# Now compute the CORRECT NED-to-body quaternion from the truth quaternion
# and compare with the EKF's quaternion
print("\n" + "=" * 120)
print("Body Frame Convention Check at t=-0.1 (end of pad)")
print("=" * 120)

# Get the state at t ≈ -0.1
idx = (df['time'] - (-0.1)).abs().idxmin()
r = df.loc[idx]

# Truth and EKF Euler angles (both in NED convention now)
tru_r_rad = math.radians(r.get('true_roll_ned_deg', 0))
tru_p_rad = math.radians(r.get('true_pitch_ned_deg', 0))
tru_y_rad = math.radians(r.get('true_yaw_ned_deg', 0))

ekf_r_rad = math.radians(r.get('ekf_roll_deg', 0))
ekf_p_rad = math.radians(r.get('ekf_pitch_deg', 0))
ekf_y_rad = math.radians(r.get('ekf_yaw_deg', 0))

print(f"\nTruth (NED Euler): roll={math.degrees(tru_r_rad):8.3f}°  "
      f"pitch={math.degrees(tru_p_rad):8.3f}°  yaw={math.degrees(tru_y_rad):8.3f}°")
print(f"EKF   (NED Euler): roll={math.degrees(ekf_r_rad):8.3f}°  "
      f"pitch={math.degrees(ekf_p_rad):8.3f}°  yaw={math.degrees(ekf_y_rad):8.3f}°")

# For NED convention: nose-up rocket with heading=0 (North)
# Pitch should be -89° (nose 89° above horizon = 1° from vertical)
# Yaw should be 0° (heading North)
# Roll should be 0°
print(f"\nExpected NED: roll=0°, pitch=-89°, yaw=0° (heading North)")
print(f"Truth-EKF difference:  Δroll={math.degrees(tru_r_rad - ekf_r_rad):8.3f}°  "
      f"Δpitch={math.degrees(tru_p_rad - ekf_p_rad):8.3f}°  "
      f"Δyaw={math.degrees(tru_y_rad - ekf_y_rad):8.3f}°")

# Compute nose direction from EKF Euler angles (NED convention)
# In NED, nose direction = [cos(pitch)*cos(yaw), cos(pitch)*sin(yaw), -sin(pitch)]
nose_n = math.cos(ekf_p_rad) * math.cos(ekf_y_rad)
nose_e = math.cos(ekf_p_rad) * math.sin(ekf_y_rad)
nose_d = -math.sin(ekf_p_rad)
print(f"\nEKF nose direction (NED): [{nose_n:7.4f}, {nose_e:7.4f}, {nose_d:7.4f}]")
print(f"  Expected (near vertical): [~0.017, ~0.000, ~0.999]  (up = -Down)")

# Heading in NED = yaw
print(f"\nEKF Heading (NED yaw): {math.degrees(ekf_y_rad):8.3f}°  (expected: 0°)")
print(f"Heading error: {math.degrees(ekf_y_rad):8.3f}°")

# Check: what heading does the body Y axis point in?
# Body Y in NED = [-sin(yaw), cos(yaw), 0] for near-vertical
body_y_ned = [-math.sin(ekf_y_rad), math.cos(ekf_y_rad), 0]
print(f"EKF body Y direction (NED): [{body_y_ned[0]:7.4f}, {body_y_ned[1]:7.4f}, {body_y_ned[2]:7.4f}]")
print(f"  Expected FRD right = East: [0.000, 1.000, 0.000]")

# Check the IMU body frame convention
print("\n" + "=" * 120)
print("IMU Body Frame Convention Check")
print("=" * 120)
print("""
The 6DOF sim uses body-to-ENU quaternion with ZYX Euler convention.
For heading=0 (North) and pitch=89° (near vertical):
  - ENU yaw = 90° (North = 90° from East in ENU)
  - ENU pitch = -89° (nose above horizon)

  body-to-ENU DCM:
    Body X (nose) → ENU [sin(yaw)*cos(pitch), cos(yaw)*cos(pitch), sin(pitch)]
                       = [sin90°*cos89°, cos90°*cos89°, sin89°]
                       = [0.017, 0, 0.9998]  (mostly Up) ✓

    Body Y → ENU [-cos(yaw), sin(yaw), 0] ... no, this depends on full DCM.

  The ZYX Euler sequence: first yaw 90° about Z(Up), then pitch -89° about new Y.
  After yaw 90° about Z: [East, North, Up] → [North, -East, Up]
  So body X → North, body Y → -East (= West!), body Z → Up

  After pitch -89° about new Y (= -East direction = West):
  body X (North) rotates toward Up: body X → [sin(1°)*North + cos(1°)*Up] ≈ Up ✓
  body Z (Up) rotates toward -North: body Z → [cos(1°)*Up - sin(1°)*North]... wait

  Actually: pitch about body Y means nose (X) tilts down if positive pitch.
  pitch = -89° tilts nose UP by 89°.

  After yaw 90°: body frame is [N, W, U] (body X=N, body Y=W, body Z=U)
  After pitch -89° (about body Y = West axis):
    body X (N) → cos(89°)*N + sin(89°)*U = 0.017*N + 0.9998*U  ✓ nose up
    body Z (U) → -sin(89°)*N + cos(89°)*U = -0.9998*N + 0.017*U  ≈ South

  So: body X → Up, body Y → West, body Z → South
  Body frame is: Forward-LEFT-Up-ish? No:
    X = nose = Up
    Y = LEFT (West when heading North)
    Z = toward South (not Down!)

  This is NOT standard FRD! The 6DOF body Y is LEFT, not RIGHT.
  The EKF expects FRD body frame (body Y = right = East for heading=0).

  MISMATCH: IMU acc_y and acc_z (and gyro_y, gyro_z, mag_y, mag_z)
  have the WRONG SIGN for the EKF's FRD convention.
""")
