#!/usr/bin/env python3
"""EKF integration test with selectable modes for debugging.

Modes (pass as first argument):
    imu-only        AHRS-converged IMU dead reckoning (realistic noise)
    truth-orient    Inject perfect orientation at ignition, realistic IMU
    perfect-imu     Zero IMU noise/bias + truth orient (isolates filter errors)
    gnss            Full nav: GNSS + baro + noisy IMU (realistic flight)

Examples:
    python scripts/test_ekf_modes.py imu-only
    python scripts/test_ekf_modes.py truth-orient
    python scripts/test_ekf_modes.py perfect-imu
    python scripts/test_ekf_modes.py gnss
    python scripts/test_ekf_modes.py all          # run all four, overlay
"""

import sys, math, subprocess
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from tinkerrocket_sim.rocket.definition import from_ork
from tinkerrocket_sim.simulation.closed_loop_sim import SimConfig, run_closed_loop

# ---- Mode definitions ----
MODES = {
    'imu-only': dict(
        label='IMU-Only (AHRS)',
        color='C0',
        cfg_overrides=dict(
            enable_gnss_updates=False,
            enable_baro_updates=False,
            inject_truth_orientation_at_ignition=False,
            perfect_imu=False,
        ),
    ),
    'truth-orient': dict(
        label='Truth Orient + Noisy IMU',
        color='C1',
        cfg_overrides=dict(
            enable_gnss_updates=False,
            enable_baro_updates=False,
            inject_truth_orientation_at_ignition=True,
            perfect_imu=False,
        ),
    ),
    'perfect-imu': dict(
        label='Perfect IMU + Truth Orient',
        color='C3',
        cfg_overrides=dict(
            enable_gnss_updates=False,
            enable_baro_updates=False,
            inject_truth_orientation_at_ignition=True,
            perfect_imu=True,
        ),
    ),
    'gnss': dict(
        label='GNSS + Baro + Noisy IMU',
        color='C2',
        cfg_overrides=dict(
            enable_gnss_updates=True,
            enable_baro_updates=True,
            inject_truth_orientation_at_ignition=False,
            perfect_imu=False,
        ),
    ),
}

def run_mode(rocket, mode_name):
    """Run a single mode and return the dataframe."""
    mode = MODES[mode_name]
    cfg = SimConfig(
        duration=16.0,
        pad_time=2.0,
        launch_angle_deg=89.0,
        roll_profile=[(0.0, 0.0), (1.0, 180.0)],
        **mode['cfg_overrides'],
    )
    print(f"  Running: {mode['label']}...")
    res = run_closed_loop(rocket, cfg)
    df = res.df
    # For truth-injected modes, discard pad phase (state is reset at t=0)
    if mode['cfg_overrides'].get('inject_truth_orientation_at_ignition', False):
        df = df[df['time'] >= 0.0].reset_index(drop=True)
    return df


def compute_attitude_error(df):
    """Quaternion angular error in degrees (gimbal-lock-free)."""
    qt0 = df['true_q0_ned'].values
    qt1 = df['true_q1_ned'].values
    qt2 = df['true_q2_ned'].values
    qt3 = df['true_q3_ned'].values
    qe0 = df['ekf_q0'].values
    qe1 = df['ekf_q1'].values
    qe2 = df['ekf_q2'].values
    qe3 = df['ekf_q3'].values
    # q_err = q_truth_conj ⊗ q_ekf
    dw = qt0*qe0 + qt1*qe1 + qt2*qe2 + qt3*qe3
    return np.degrees(2.0 * np.arccos(np.clip(np.abs(dw), 0, 1)))


def plot_results(results, out_dir):
    """Generate all diagnostic plots."""
    out_dir = Path(out_dir)
    out_dir.mkdir(exist_ok=True)

    # ---- Figure 1: Position & Velocity errors ----
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle('EKF Mode Comparison: Position & Velocity Error', fontsize=14, fontweight='bold')

    # GNSS dropout shading + sigma bounds (if gnss mode present)
    if 'gnss' in results:
        gdf = results['gnss']
        gt = gdf['time'].values
        if 'gnss_valid' in gdf.columns:
            # Shade GNSS dropout regions
            valid = gdf['gnss_valid'].values.astype(bool)
            in_dropout = False
            for i in range(len(valid)):
                if gt[i] < 0:
                    continue
                if not valid[i] and not in_dropout:
                    drop_start = gt[i]
                    in_dropout = True
                elif valid[i] and in_dropout:
                    for ax in axes:
                        ax.axvspan(drop_start, gt[i], color='red',
                                   alpha=0.08, zorder=0)
                    in_dropout = False
            if in_dropout:
                for ax in axes:
                    ax.axvspan(drop_start, gt[-1], color='red',
                               alpha=0.08, zorder=0)
        # Plot EKF sigma bounds (3-sigma)
        if 'ekf_pos_sigma_n' in gdf.columns:
            sig_pos = np.sqrt(gdf['ekf_pos_sigma_n']**2 +
                              gdf['ekf_pos_sigma_e']**2 +
                              gdf['ekf_pos_sigma_d']**2)
            sig_vel = np.sqrt(gdf['ekf_vel_sigma_n']**2 +
                              gdf['ekf_vel_sigma_e']**2 +
                              gdf['ekf_vel_sigma_d']**2)
            axes[0].plot(gt, 3*sig_pos, '--', color='C2', lw=0.8,
                         alpha=0.5, label='GNSS 3-sigma')
            axes[1].plot(gt, 3*sig_vel, '--', color='C2', lw=0.8,
                         alpha=0.5, label='GNSS 3-sigma')

    for name, df in results.items():
        mode = MODES[name]
        t = df['time'].values
        pos_err = np.sqrt((df['ekf_pn']-df['true_pn'])**2 +
                          (df['ekf_pe']-df['true_pe'])**2 +
                          (df['ekf_pd']-df['true_pd'])**2)
        vel_err = np.sqrt((df['ekf_vn']-df['true_vn'])**2 +
                          (df['ekf_ve']-df['true_ve'])**2 +
                          (df['ekf_vd']-df['true_vd'])**2)
        axes[0].plot(t, pos_err, color=mode['color'], lw=1.5, label=mode['label'])
        axes[1].plot(t, vel_err, color=mode['color'], lw=1.5, label=mode['label'])

    axes[0].set_ylabel('3D position error (m)')
    axes[0].legend(fontsize=9)
    axes[0].grid(True, alpha=0.3)
    axes[0].axvline(0, color='green', lw=0.8, ls=':', alpha=0.5)
    axes[1].set_ylabel('3D velocity error (m/s)')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend(fontsize=9)
    axes[1].grid(True, alpha=0.3)
    axes[1].axvline(0, color='green', lw=0.8, ls=':', alpha=0.5)

    plt.tight_layout()
    out1 = out_dir / 'ekf_modes_pos_vel.png'
    plt.savefig(out1, dpi=180, bbox_inches='tight')
    plt.close()

    # ---- Figure 2: Attitude error (quaternion angular) ----
    fig, ax = plt.subplots(1, 1, figsize=(14, 5))
    fig.suptitle('EKF Mode Comparison: Attitude Error (quaternion)', fontsize=14, fontweight='bold')

    for name, df in results.items():
        mode = MODES[name]
        t = df['time'].values
        att_err = compute_attitude_error(df)
        ax.plot(t, att_err, color=mode['color'], lw=1.5, label=mode['label'])

    ax.set_ylabel('Attitude error (deg)')
    ax.set_xlabel('Time (s)')
    ax.set_ylim(bottom=0)
    ax.axvline(0, color='green', lw=0.8, ls=':', alpha=0.5)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out2 = out_dir / 'ekf_modes_attitude_err.png'
    plt.savefig(out2, dpi=180, bbox_inches='tight')
    plt.close()

    # ---- Figure 3: Roll / Pitch / Yaw (NED) — truth vs EKF per mode ----
    for name, df in results.items():
        mode = MODES[name]
        t = df['time'].values

        fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
        fig.suptitle(f'{mode["label"]}: Euler Angles NED (Truth vs EKF)', fontsize=14, fontweight='bold')

        for i, (ang, label) in enumerate([('roll', 'Roll'), ('pitch', 'Pitch'), ('yaw', 'Yaw / Heading')]):
            ax = axes[i]
            ax.plot(t, df[f'true_{ang}_ned_deg'], 'k-', lw=2, label='Truth')
            ax.plot(t, df[f'ekf_{ang}_deg'], '-', color=mode['color'], lw=1.5, alpha=0.8, label='EKF')
            ax.set_ylabel(f'{label} (deg)')
            ax.legend(fontsize=9, loc='best')
            ax.grid(True, alpha=0.3)
            ax.axvline(0, color='green', lw=0.8, ls=':', alpha=0.5)

        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        out_euler = out_dir / f'ekf_{name}_euler_ned.png'
        plt.savefig(out_euler, dpi=180, bbox_inches='tight')
        plt.close()

    # ---- Print table ----
    print("\n" + "=" * 100)
    print("Error Summary (3D position, 3D velocity, attitude at key times)")
    print("=" * 100)

    time_points = [0.5, 1.0, 2.0, 4.0, 8.0, 12.0]
    header = f"{'t':>6s}"
    for name in results:
        label_short = name[:12]
        header += f" | {'pos':>7s} {'vel':>7s} {'att':>6s}  [{label_short}]"
    print(header)
    print("-" * 100)

    for ts in time_points:
        row = f"{ts:6.1f}"
        for name, df in results.items():
            idx = (df['time'] - ts).abs().idxmin()
            pos_err = np.sqrt((df.loc[idx, 'ekf_pn']-df.loc[idx, 'true_pn'])**2 +
                              (df.loc[idx, 'ekf_pe']-df.loc[idx, 'true_pe'])**2 +
                              (df.loc[idx, 'ekf_pd']-df.loc[idx, 'true_pd'])**2)
            vel_err = np.sqrt((df.loc[idx, 'ekf_vn']-df.loc[idx, 'true_vn'])**2 +
                              (df.loc[idx, 'ekf_ve']-df.loc[idx, 'true_ve'])**2 +
                              (df.loc[idx, 'ekf_vd']-df.loc[idx, 'true_vd'])**2)
            att_err = compute_attitude_error(df.loc[[idx]])[0]
            row += f" | {pos_err:7.1f} {vel_err:7.1f} {att_err:6.1f}"
        print(row)

    plots = [out1, out2] + list(out_dir.glob('ekf_*_euler_ned.png'))
    return plots


# ---- Main ----
if __name__ == '__main__':
    mode_arg = sys.argv[1] if len(sys.argv) > 1 else 'all'

    rocket_dir = Path(__file__).parent.parent / "rockets"
    ork_file = rocket_dir / "54mm_Roll_Control.ork"
    rocket = from_ork(ork_file)

    if mode_arg == 'all':
        run_names = list(MODES.keys())
    elif mode_arg in MODES:
        run_names = [mode_arg]
    else:
        print(f"Unknown mode: {mode_arg}")
        print(f"Available: {', '.join(MODES.keys())}, all")
        sys.exit(1)

    print(f"Running EKF mode test(s): {', '.join(run_names)}")
    results = {}
    for name in run_names:
        results[name] = run_mode(rocket, name)

    out_dir = Path(__file__).parent.parent / "plots"
    plots = plot_results(results, out_dir)

    print(f"\nOpening plots...")
    subprocess.run(['open'] + [str(p) for p in plots])
