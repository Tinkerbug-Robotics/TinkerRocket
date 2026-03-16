#!/usr/bin/env python3
"""Pinpoint when and why attitude error grows in perfect-imu mode."""
import sys, math
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from tinkerrocket_sim.rocket.definition import from_ork
from tinkerrocket_sim.simulation.closed_loop_sim import SimConfig, run_closed_loop

rocket = from_ork(Path(__file__).parent.parent / "rockets" / "54mm_Roll_Control.ork")

cfg = SimConfig(
    duration=8.0,
    pad_time=2.0,
    launch_angle_deg=89.0,
    roll_profile=[(0.0, 0.0), (1.0, 180.0)],
    enable_gnss_updates=False,
    enable_baro_updates=False,
    inject_truth_orientation_at_ignition=True,
    perfect_imu=True,
)

print("Running perfect-imu test...")
res = run_closed_loop(rocket, cfg)
df = res.df

# Filter to flight phase only (t >= 0)
df = df[df['time'] >= 0.0].reset_index(drop=True)

# Compute quaternion attitude error
qt0 = df['true_q0_ned'].values
qt1 = df['true_q1_ned'].values
qt2 = df['true_q2_ned'].values
qt3 = df['true_q3_ned'].values
qe0 = df['ekf_q0'].values
qe1 = df['ekf_q1'].values
qe2 = df['ekf_q2'].values
qe3 = df['ekf_q3'].values
dw = qt0*qe0 + qt1*qe1 + qt2*qe2 + qt3*qe3
att_err = np.degrees(2.0 * np.arccos(np.clip(np.abs(dw), 0, 1)))

# Find when error first exceeds thresholds
for threshold in [1, 5, 10, 20, 30, 50]:
    idx = np.argmax(att_err > threshold)
    if att_err[idx] > threshold:
        t = df.loc[idx, 'time']
        print(f"  Error first exceeds {threshold:3d}° at t={t:.4f}s")
    else:
        print(f"  Error never exceeds {threshold:3d}°")

# Print detailed info around the error onset
print("\n--- Detailed timeline around error onset ---")
print(f"{'t':>8s} {'att_err':>8s} {'accel_mag':>10s} {'ekf_q0':>8s} {'ekf_q1':>8s} {'ekf_q2':>8s} {'ekf_q3':>8s} | {'tru_q0':>8s} {'tru_q1':>8s} {'tru_q2':>8s} {'tru_q3':>8s} | {'wbias_x':>8s} {'wbias_y':>8s} {'wbias_z':>8s}")

# Print at specific intervals
t_vals = np.concatenate([
    np.arange(0, 0.5, 0.1),
    np.arange(0.5, 2.5, 0.25),
    np.arange(2.5, 5.0, 0.1),
])

for ts in t_vals:
    idx = (df['time'] - ts).abs().idxmin()
    t = df.loc[idx, 'time']

    # Accel magnitude (body frame)
    if 'imu_acc_x' in df.columns:
        ax = df.loc[idx, 'imu_acc_x']
        ay = df.loc[idx, 'imu_acc_y']
        az = df.loc[idx, 'imu_acc_z']
        amag = math.sqrt(ax**2 + ay**2 + az**2)
    else:
        amag = float('nan')

    # Gyro bias
    if 'ekf_wbias_x' in df.columns:
        wbx = df.loc[idx, 'ekf_wbias_x']
        wby = df.loc[idx, 'ekf_wbias_y']
        wbz = df.loc[idx, 'ekf_wbias_z']
    else:
        wbx = wby = wbz = float('nan')

    # EKF and truth quats
    eq0 = df.loc[idx, 'ekf_q0']
    eq1 = df.loc[idx, 'ekf_q1']
    eq2 = df.loc[idx, 'ekf_q2']
    eq3 = df.loc[idx, 'ekf_q3']
    tq0 = df.loc[idx, 'true_q0_ned']
    tq1 = df.loc[idx, 'true_q1_ned']
    tq2 = df.loc[idx, 'true_q2_ned']
    tq3 = df.loc[idx, 'true_q3_ned']

    err = att_err[idx]
    amag_g = amag / 9.807

    print(f"{t:8.4f} {err:8.2f}° {amag_g:8.3f}g  {eq0:8.4f} {eq1:8.4f} {eq2:8.4f} {eq3:8.4f} | {tq0:8.4f} {tq1:8.4f} {tq2:8.4f} {tq3:8.4f} | {wbx:8.5f} {wby:8.5f} {wbz:8.5f}")

# Also check: what columns exist related to EKF bias?
bias_cols = [c for c in df.columns if 'bias' in c.lower() or 'wbias' in c.lower() or 'abias' in c.lower()]
print(f"\nBias-related columns: {bias_cols}")
print(f"All columns: {sorted(df.columns.tolist())}")
