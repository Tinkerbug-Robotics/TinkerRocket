#!/usr/bin/env python3
"""
Test EKF with IMU-only integration (no GNSS, no baro).
Two modes:
    1. Default: raw IMU dead reckoning (AHRS convergence error included)
    2. --truth-orient: inject perfect orientation at ignition, plot ignition→apogee only

Plots EKF estimates vs truth to show dead-reckoning drift.
"""

import sys, subprocess
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from tinkerrocket_sim.rocket.definition import from_ork
from tinkerrocket_sim.simulation.closed_loop_sim import SimConfig, run_closed_loop

# Parse args
truth_orient = '--truth-orient' in sys.argv

# Load rocket
rocket_dir = Path(__file__).parent.parent / "rockets"
ork_file = rocket_dir / "54mm_Roll_Control.ork"
rocket = from_ork(ork_file)

# IMU-only config
cfg = SimConfig(
    duration=16.0,
    pad_time=2.0,
    launch_angle_deg=89.0,
    roll_profile=[(0.0, 0.0), (1.0, 180.0)],
    enable_gnss_updates=False,
    enable_baro_updates=False,
    inject_truth_orientation_at_ignition=truth_orient,
)

mode_label = "IMU-Only (Perfect Orient @ Ignition)" if truth_orient else "IMU-Only"
file_tag = "imu_truth_orient" if truth_orient else "imu_only"

print(f"Running {mode_label} sim...")
res = run_closed_loop(rocket, cfg)
df = res.df

# If truth orientation mode, focus on ignition → apogee
if truth_orient:
    # Find apogee (max altitude)
    apogee_idx = df['altitude'].idxmax()
    apogee_time = df.loc[apogee_idx, 'time']
    df = df[(df['time'] >= 0.0) & (df['time'] <= apogee_time + 0.5)].copy()
    df = df.reset_index(drop=True)
    print(f"  Apogee at t={apogee_time:.1f}s, alt={df['altitude'].max():.0f}m")

t = df['time'].values

# ---- Figure 1: Position ----
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
fig.suptitle(f'{mode_label}: Position (EKF vs Truth)', fontsize=14, fontweight='bold')

for i, (comp, label) in enumerate([('n', 'North'), ('e', 'East'), ('d', 'Down (neg = up)')]):
    ax = axes[i]
    ax.plot(t, df[f'true_p{comp}'], 'k-', lw=2, label='Truth')
    ax.plot(t, df[f'ekf_p{comp}'], 'r-', lw=1.5, alpha=0.8, label=f'EKF ({mode_label})')
    # Covariance bands
    sigma = df[f'ekf_pos_sigma_{comp}']
    ax.fill_between(t, df[f'ekf_p{comp}'] - 2*sigma, df[f'ekf_p{comp}'] + 2*sigma,
                    color='red', alpha=0.1, label=r'$\pm 2\sigma$')
    ax.set_ylabel(f'{label} (m)')
    if not truth_orient:
        ax.axvline(0, color='green', lw=0.8, ls=':', alpha=0.5)
    ax.legend(fontsize=9, loc='best')
    ax.grid(True, alpha=0.3)

axes[-1].set_xlabel('Time (s)')
plt.tight_layout()
out1 = Path(__file__).parent.parent / "plots" / f"{file_tag}_position.png"
out1.parent.mkdir(exist_ok=True)
plt.savefig(out1, dpi=180, bbox_inches='tight')
plt.close()

# ---- Figure 2: Velocity ----
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
fig.suptitle(f'{mode_label}: Velocity (EKF vs Truth)', fontsize=14, fontweight='bold')

for i, (comp, label) in enumerate([('n', 'North'), ('e', 'East'), ('d', 'Down')]):
    ax = axes[i]
    ax.plot(t, df[f'true_v{comp}'], 'k-', lw=2, label='Truth')
    ax.plot(t, df[f'ekf_v{comp}'], 'r-', lw=1.5, alpha=0.8, label=f'EKF ({mode_label})')
    sigma = df[f'ekf_vel_sigma_{comp}']
    ax.fill_between(t, df[f'ekf_v{comp}'] - 2*sigma, df[f'ekf_v{comp}'] + 2*sigma,
                    color='red', alpha=0.1, label=r'$\pm 2\sigma$')
    ax.set_ylabel(f'{label} (m/s)')
    if not truth_orient:
        ax.axvline(0, color='green', lw=0.8, ls=':', alpha=0.5)
    ax.legend(fontsize=9, loc='best')
    ax.grid(True, alpha=0.3)

axes[-1].set_xlabel('Time (s)')
plt.tight_layout()
out2 = Path(__file__).parent.parent / "plots" / f"{file_tag}_velocity.png"
plt.savefig(out2, dpi=180, bbox_inches='tight')
plt.close()

# ---- Figure 3: 3D error + attitude ----
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
fig.suptitle(f'{mode_label}: Error Growth & Attitude', fontsize=14, fontweight='bold')

pos_err = np.sqrt((df['ekf_pn']-df['true_pn'])**2 +
                  (df['ekf_pe']-df['true_pe'])**2 +
                  (df['ekf_pd']-df['true_pd'])**2)
vel_err = np.sqrt((df['ekf_vn']-df['true_vn'])**2 +
                  (df['ekf_ve']-df['true_ve'])**2 +
                  (df['ekf_vd']-df['true_vd'])**2)

ax1.plot(t, pos_err, 'r-', lw=1.5)
ax1.set_ylabel('3D position error (m)')
if not truth_orient:
    ax1.axvline(0, color='green', lw=0.8, ls=':', alpha=0.5, label='Launch')
    ax1.legend(fontsize=9)
ax1.grid(True, alpha=0.3)

ax2.plot(t, vel_err, 'r-', lw=1.5)
ax2.set_ylabel('3D velocity error (m/s)')
if not truth_orient:
    ax2.axvline(0, color='green', lw=0.8, ls=':', alpha=0.5)
ax2.grid(True, alpha=0.3)

# Attitude error: quaternion angular error (gimbal-lock-free)
# Both truth and EKF quaternions are in NED/FRD convention
has_quat = all(c in df.columns for c in ['ekf_q0', 'true_q0_ned'])
if has_quat:
    # q_err = q_truth^{-1} ⊗ q_ekf  →  angle = 2*arccos(|q_err.w|)
    qt0 = df['true_q0_ned'].values
    qt1 = df['true_q1_ned'].values
    qt2 = df['true_q2_ned'].values
    qt3 = df['true_q3_ned'].values
    qe0 = df['ekf_q0'].values
    qe1 = df['ekf_q1'].values
    qe2 = df['ekf_q2'].values
    qe3 = df['ekf_q3'].values
    # Quaternion product: q_truth_conj ⊗ q_ekf  (conjugate = negate vector part)
    dw =  qt0*qe0 + qt1*qe1 + qt2*qe2 + qt3*qe3
    dx = -qt1*qe0 + qt0*qe1 + qt3*qe2 - qt2*qe3
    dy = -qt2*qe0 - qt3*qe1 + qt0*qe2 + qt1*qe3
    dz = -qt3*qe0 + qt2*qe1 - qt1*qe2 + qt0*qe3
    # Angular error in degrees
    att_err_deg = np.degrees(2.0 * np.arccos(np.clip(np.abs(dw), 0, 1)))
    ax3.plot(t, att_err_deg, 'r-', lw=1.5)
    ax3.set_ylabel('Attitude error (deg)')
    ax3.set_ylim(bottom=0)
else:
    # Fallback: just plot pitch comparison
    ax3.plot(t, df['true_pitch_ned_deg'], 'k-', lw=2, label='Truth pitch')
    ax3.plot(t, df['ekf_pitch_deg'], 'r-', lw=1.5, alpha=0.8, label='EKF pitch')
    ax3.set_ylabel('Angle (deg)')
    ax3.legend(fontsize=8, loc='best')
ax3.set_xlabel('Time (s)')
if not truth_orient:
    ax3.axvline(0, color='green', lw=0.8, ls=':', alpha=0.5)
ax3.grid(True, alpha=0.3)

plt.tight_layout()
out3 = Path(__file__).parent.parent / "plots" / f"{file_tag}_summary.png"
plt.savefig(out3, dpi=180, bbox_inches='tight')
plt.close()

# Print stats
print(f"\n=== {mode_label} Error Summary ===")
time_points = [0.5, 1.0, 2.0, 4.0, 6.0, 8.0]
if not truth_orient:
    time_points.extend([12.0, 16.0])
for ts in time_points:
    mask = df['time'] >= ts - 0.01
    if not mask.any():
        continue
    idx = (df['time'] - ts).abs().idxmin()
    print(f"  t={ts:5.1f}s:  pos_3d={pos_err[idx]:8.1f} m  "
          f"vel_3d={vel_err[idx]:7.1f} m/s  "
          f"sigma_pos={df.loc[idx, 'ekf_pos_sigma_n']:7.0f} m")

# Open plots
print(f"\nOpening plots...")
subprocess.run(['open', str(out1), str(out2), str(out3)])
