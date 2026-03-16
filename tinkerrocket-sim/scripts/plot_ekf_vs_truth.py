#!/usr/bin/env python3
"""EKF vs Truth comparison plots.

Generates detailed plots comparing EKF estimated state against sim truth,
with covariance uncertainty bands, GNSS dropout windows, and Mach/baro
lockout visualization.

Usage:
    python scripts/plot_ekf_vs_truth.py [--csv path/to/data.csv] [--show]

If no CSV is provided, runs the closed-loop sim with default config.
"""
import os
import sys
import argparse
import numpy as np

# ============================================================================
# CONFIG (used when running sim directly, not loading CSV)
# ============================================================================
PAD_TIME = 2.0
DURATION = 30.0
SHOW_PLOT = False


# ============================================================================
# PLOTTING
# ============================================================================

def add_phase_shading(ax, df):
    """Add subtle background shading for flight phases."""
    if 'flight_phase' not in df.columns:
        return

    phases = df['flight_phase'].values
    time = df['time'].values
    ylim = ax.get_ylim()

    colors = {
        'PAD': '#cccccc',
        'BOOST': '#ffcccc',
        'COAST': '#ccffcc',
        'DESCENT': '#ccccff',
    }

    prev_phase = phases[0]
    start = time[0]
    for i in range(1, len(phases)):
        if phases[i] != prev_phase or i == len(phases) - 1:
            if prev_phase in colors:
                ax.axvspan(start, time[i], alpha=0.08,
                           color=colors[prev_phase], zorder=0)
            prev_phase = phases[i]
            start = time[i]

    ax.set_ylim(ylim)


def add_gnss_dropout_strip(ax, df, y_pos=None):
    """Add red shading where GNSS is invalid."""
    if 'gnss_valid' not in df.columns:
        return

    time = df['time'].values
    valid = df['gnss_valid'].values.astype(bool)

    in_dropout = False
    start = 0.0
    for i in range(len(valid)):
        if not valid[i] and not in_dropout:
            start = time[i]
            in_dropout = True
        elif valid[i] and in_dropout:
            ax.axvspan(start, time[i], alpha=0.15, color='red',
                       zorder=0, label='GNSS dropout' if start == time[0] else None)
            in_dropout = False
    if in_dropout:
        ax.axvspan(start, time[-1], alpha=0.15, color='red', zorder=0)


def plot_ekf_vs_truth(df, title="EKF vs Truth", save_path=None, show=False):
    """Generate EKF comparison plots across 5 pages.

    Page 1 — Position: EKF NED vs true NED with ±2σ covariance bounds
    Page 2 — Velocity: EKF NED vs true NED with ±2σ covariance bounds
    Page 3 — Attitude: EKF roll/pitch/yaw vs true
    Page 4 — Altitude: EKF alt vs true alt vs baro alt, GNSS/baro strips
    Page 5 — Sensor status: Mach, GNSS valid, baro valid, flight phase
    """
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    has_ekf = all(c in df.columns for c in ['ekf_pn', 'ekf_vn', 'ekf_roll_deg'])
    has_cov = 'ekf_pos_sigma_n' in df.columns
    has_baro = 'baro_alt' in df.columns
    has_gnss = 'gnss_valid' in df.columns
    has_mach = 'mach' in df.columns

    if not has_ekf:
        print("No EKF data in DataFrame — skipping EKF plots.")
        return

    t = df['time'].values

    if save_path:
        base, ext = os.path.splitext(save_path)
    else:
        base, ext = None, None

    figs = []

    # ---- Page 1: Position (NED) ----
    fig1, axes1 = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fig1.suptitle(f"{title} — Position (NED)")

    for i, (axis, ekf_col, true_col, sigma_col, label) in enumerate([
        (axes1[0], 'ekf_pn', 'true_pn', 'ekf_pos_sigma_n', 'North (m)'),
        (axes1[1], 'ekf_pe', 'true_pe', 'ekf_pos_sigma_e', 'East (m)'),
        (axes1[2], 'ekf_pd', 'true_pd', 'ekf_pos_sigma_d', 'Down (m)'),
    ]):
        axis.plot(t, df[true_col], 'k-', linewidth=1.0, label='Truth', alpha=0.9)
        axis.plot(t, df[ekf_col], 'b-', linewidth=0.8, label='EKF', alpha=0.8)

        if has_cov and sigma_col in df.columns:
            sigma = df[sigma_col].values
            ekf_vals = df[ekf_col].values
            axis.fill_between(t, ekf_vals - 2*sigma, ekf_vals + 2*sigma,
                              alpha=0.15, color='blue', label='±2σ')

        add_gnss_dropout_strip(axis, df)
        add_phase_shading(axis, df)
        axis.set_ylabel(label)
        axis.legend(loc='upper right', fontsize=7)
        axis.grid(True, alpha=0.3)
        axis.axvline(0, color='k', linestyle='--', alpha=0.3)

    axes1[2].set_xlabel("Time (s)")
    fig1.tight_layout()
    figs.append(fig1)

    if base:
        p = f"{base}_pos{ext}"
        fig1.savefig(p, dpi=150)
        print(f"  Saved {p}")

    # ---- Page 2: Velocity (NED) ----
    fig2, axes2 = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fig2.suptitle(f"{title} — Velocity (NED)")

    for i, (axis, ekf_col, true_col, sigma_col, label) in enumerate([
        (axes2[0], 'ekf_vn', 'true_vn', 'ekf_vel_sigma_n', 'V_North (m/s)'),
        (axes2[1], 'ekf_ve', 'true_ve', 'ekf_vel_sigma_e', 'V_East (m/s)'),
        (axes2[2], 'ekf_vd', 'true_vd', 'ekf_vel_sigma_d', 'V_Down (m/s)'),
    ]):
        axis.plot(t, df[true_col], 'k-', linewidth=1.0, label='Truth', alpha=0.9)
        axis.plot(t, df[ekf_col], 'b-', linewidth=0.8, label='EKF', alpha=0.8)

        if has_cov and sigma_col in df.columns:
            sigma = df[sigma_col].values
            ekf_vals = df[ekf_col].values
            axis.fill_between(t, ekf_vals - 2*sigma, ekf_vals + 2*sigma,
                              alpha=0.15, color='blue', label='±2σ')

        add_gnss_dropout_strip(axis, df)
        add_phase_shading(axis, df)
        axis.set_ylabel(label)
        axis.legend(loc='upper right', fontsize=7)
        axis.grid(True, alpha=0.3)
        axis.axvline(0, color='k', linestyle='--', alpha=0.3)

    axes2[2].set_xlabel("Time (s)")
    fig2.tight_layout()
    figs.append(fig2)

    if base:
        p = f"{base}_vel{ext}"
        fig2.savefig(p, dpi=150)
        print(f"  Saved {p}")

    # ---- Page 3: Attitude ----
    fig3, axes3 = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fig3.suptitle(f"{title} — Attitude")

    for i, (axis, ekf_col, true_col, label) in enumerate([
        (axes3[0], 'ekf_roll_deg', 'roll_deg', 'Roll (deg)'),
        (axes3[1], 'ekf_pitch_deg', 'pitch_deg', 'Pitch (deg)'),
        (axes3[2], 'ekf_yaw_deg', 'yaw_deg', 'Yaw (deg)'),
    ]):
        true_vals = df[true_col].values
        ekf_vals = df[ekf_col].values

        # Unwrap angles for cleaner plotting
        true_unwrap = np.degrees(np.unwrap(np.radians(true_vals)))
        ekf_unwrap = np.degrees(np.unwrap(np.radians(ekf_vals)))

        axis.plot(t, true_unwrap, 'k-', linewidth=1.0, label='Truth', alpha=0.9)
        axis.plot(t, ekf_unwrap, 'b-', linewidth=0.8, label='EKF', alpha=0.8)

        add_gnss_dropout_strip(axis, df)
        add_phase_shading(axis, df)
        axis.set_ylabel(label)
        axis.legend(loc='upper right', fontsize=7)
        axis.grid(True, alpha=0.3)
        axis.axvline(0, color='k', linestyle='--', alpha=0.3)

    axes3[2].set_xlabel("Time (s)")
    fig3.tight_layout()
    figs.append(fig3)

    if base:
        p = f"{base}_att{ext}"
        fig3.savefig(p, dpi=150)
        print(f"  Saved {p}")

    # ---- Page 4: Altitude detail ----
    fig4, axes4 = plt.subplots(2, 1, figsize=(14, 8), sharex=True,
                                gridspec_kw={'height_ratios': [3, 1]})
    fig4.suptitle(f"{title} — Altitude Detail")

    ax_alt = axes4[0]
    # Truth altitude (convert from ENU Up to altitude)
    true_alt = -df['true_pd'].values  # -Down = Up = altitude
    ekf_alt = -df['ekf_pd'].values

    ax_alt.plot(t, true_alt, 'k-', linewidth=1.0, label='Truth', alpha=0.9)
    ax_alt.plot(t, ekf_alt, 'b-', linewidth=0.8, label='EKF', alpha=0.8)

    if has_cov:
        sigma_d = df['ekf_pos_sigma_d'].values
        ax_alt.fill_between(t, ekf_alt - 2*sigma_d, ekf_alt + 2*sigma_d,
                            alpha=0.15, color='blue', label='±2σ')

    if has_baro:
        baro_vals = df['baro_alt'].values
        ax_alt.plot(t, baro_vals, 'g.', markersize=0.5, alpha=0.3, label='Baro alt')

    add_gnss_dropout_strip(ax_alt, df)

    # Shade Mach lockout region (where baro is unreliable)
    if 'baro_valid' in df.columns:
        baro_v = df['baro_valid'].values.astype(bool)
        in_lockout = False
        start = 0.0
        for i in range(len(baro_v)):
            if not baro_v[i] and not in_lockout:
                start = t[i]
                in_lockout = True
            elif baro_v[i] and in_lockout:
                ax_alt.axvspan(start, t[i], alpha=0.12, color='orange',
                               zorder=0, label='Baro lockout' if start == t[0] else None)
                in_lockout = False
        if in_lockout:
            ax_alt.axvspan(start, t[-1], alpha=0.12, color='orange', zorder=0)

    ax_alt.set_ylabel("Altitude (m)")
    ax_alt.legend(loc='upper right', fontsize=7)
    ax_alt.grid(True, alpha=0.3)
    ax_alt.axvline(0, color='k', linestyle='--', alpha=0.3)

    # Altitude error subplot
    ax_err = axes4[1]
    alt_err = ekf_alt - true_alt
    ax_err.plot(t, alt_err, 'b-', linewidth=0.8, alpha=0.8)
    if has_cov:
        ax_err.fill_between(t, -2*sigma_d, 2*sigma_d, alpha=0.15, color='blue')
    ax_err.set_ylabel("Alt Error (m)")
    ax_err.set_xlabel("Time (s)")
    ax_err.axhline(0, color='k', alpha=0.3)
    ax_err.grid(True, alpha=0.3)
    ax_err.axvline(0, color='k', linestyle='--', alpha=0.3)
    add_gnss_dropout_strip(ax_err, df)

    fig4.tight_layout()
    figs.append(fig4)

    if base:
        p = f"{base}_alt{ext}"
        fig4.savefig(p, dpi=150)
        print(f"  Saved {p}")

    # ---- Page 5: Sensor status overview ----
    n_panels = 1 + has_mach + has_gnss + ('baro_valid' in df.columns)
    fig5, axes5 = plt.subplots(n_panels, 1, figsize=(14, 2 * n_panels + 1), sharex=True)
    if n_panels == 1:
        axes5 = [axes5]
    fig5.suptitle(f"{title} — Sensor Status")

    ax_idx = 0

    # Flight phase as categorical
    if 'flight_phase' in df.columns:
        phase_map = {'PAD': 0, 'BOOST': 1, 'COAST': 2, 'DESCENT': 3}
        phase_num = [phase_map.get(p, -1) for p in df['flight_phase']]
        axes5[ax_idx].plot(t, phase_num, 'k-', linewidth=1.5)
        axes5[ax_idx].set_ylabel("Flight Phase")
        axes5[ax_idx].set_yticks([0, 1, 2, 3])
        axes5[ax_idx].set_yticklabels(['PAD', 'BOOST', 'COAST', 'DESCENT'])
        axes5[ax_idx].grid(True, alpha=0.3)
        axes5[ax_idx].axvline(0, color='k', linestyle='--', alpha=0.3)
        ax_idx += 1

    if has_mach:
        axes5[ax_idx].plot(t, df['mach'], 'r-', linewidth=0.8)
        axes5[ax_idx].axhline(0.5, color='orange', linestyle='--', alpha=0.7,
                              label='Baro lockout (M=0.5)')
        axes5[ax_idx].axhline(1.0, color='red', linestyle='--', alpha=0.5,
                              label='Mach 1')
        axes5[ax_idx].set_ylabel("Mach")
        axes5[ax_idx].legend(loc='upper right', fontsize=7)
        axes5[ax_idx].grid(True, alpha=0.3)
        axes5[ax_idx].axvline(0, color='k', linestyle='--', alpha=0.3)
        ax_idx += 1

    if has_gnss:
        axes5[ax_idx].fill_between(t, 0, df['gnss_valid'].astype(float),
                                   step='mid', alpha=0.5, color='green', label='GNSS valid')
        axes5[ax_idx].set_ylabel("GNSS")
        axes5[ax_idx].set_ylim(-0.1, 1.3)
        axes5[ax_idx].set_yticks([0, 1])
        axes5[ax_idx].set_yticklabels(['Dropout', 'Valid'])
        axes5[ax_idx].legend(loc='upper right', fontsize=7)
        axes5[ax_idx].grid(True, alpha=0.3)
        axes5[ax_idx].axvline(0, color='k', linestyle='--', alpha=0.3)
        ax_idx += 1

    if 'baro_valid' in df.columns:
        axes5[ax_idx].fill_between(t, 0, df['baro_valid'].astype(float),
                                   step='mid', alpha=0.5, color='orange', label='Baro valid')
        axes5[ax_idx].set_ylabel("Baro")
        axes5[ax_idx].set_ylim(-0.1, 1.3)
        axes5[ax_idx].set_yticks([0, 1])
        axes5[ax_idx].set_yticklabels(['Lockout', 'Valid'])
        axes5[ax_idx].legend(loc='upper right', fontsize=7)
        axes5[ax_idx].grid(True, alpha=0.3)
        axes5[ax_idx].set_xlabel("Time (s)")
        axes5[ax_idx].axvline(0, color='k', linestyle='--', alpha=0.3)
        ax_idx += 1

    fig5.tight_layout()
    figs.append(fig5)

    if base:
        p = f"{base}_status{ext}"
        fig5.savefig(p, dpi=150)
        print(f"  Saved {p}")

    # Show or close
    if show:
        plt.show()
    else:
        for fig in figs:
            plt.close(fig)


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="EKF vs Truth comparison plots")
    parser.add_argument("--csv", type=str, default=None,
                        help="Path to CSV from a previous sim run")
    parser.add_argument("--show", action="store_true",
                        help="Show plots interactively instead of saving to file")
    args = parser.parse_args()

    import pandas as pd

    if args.csv:
        print(f"Loading data from {args.csv}")
        df = pd.read_csv(args.csv)
    else:
        print("No CSV provided — running closed-loop simulation...")
        # Add project root to path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(script_dir)
        sys.path.insert(0, os.path.join(project_dir, "src"))

        import time as _time
        from tinkerrocket_sim.simulation.closed_loop_sim import run_closed_loop, SimConfig

        # Import load_rocket from run_closed_loop
        sys.path.insert(0, script_dir)
        from run_closed_loop import load_rocket

        rd = load_rocket()
        cfg = SimConfig(pad_time=PAD_TIME, duration=DURATION)

        print(f"  Rocket: {rd.name}")
        print(f"  Pad warmup: {PAD_TIME:.1f}s")
        t0 = _time.time()
        result = run_closed_loop(rd, cfg)
        elapsed = _time.time() - t0

        df = result.df
        print(f"  Apogee: {result.apogee_m:.1f} m | Max speed: {result.max_speed_mps:.1f} m/s")
        print(f"  Sim took {elapsed:.1f}s ({len(df)} data points)")

    # Print EKF stats
    if 'ekf_pn' in df.columns:
        flight = df[df['time'] > 0.0]
        if len(flight) > 0:
            pn_err = flight['ekf_pn'] - flight['true_pn']
            pe_err = flight['ekf_pe'] - flight['true_pe']
            pd_err = flight['ekf_pd'] - flight['true_pd']
            vn_err = flight['ekf_vn'] - flight['true_vn']
            ve_err = flight['ekf_ve'] - flight['true_ve']
            vd_err = flight['ekf_vd'] - flight['true_vd']

            pos_err_3d = np.sqrt(pn_err**2 + pe_err**2 + pd_err**2)
            vel_err_3d = np.sqrt(vn_err**2 + ve_err**2 + vd_err**2)

            print(f"\nEKF Performance (flight phase only):")
            print(f"  Position 3D error: mean={pos_err_3d.mean():.1f}m, "
                  f"max={pos_err_3d.max():.1f}m")
            print(f"  Velocity 3D error: mean={vel_err_3d.mean():.1f}m/s, "
                  f"max={vel_err_3d.max():.1f}m/s")

            if 'gnss_valid' in df.columns:
                dropout = flight[~flight['gnss_valid'].astype(bool)]
                if len(dropout) > 0:
                    dropout_dur = dropout['time'].max() - dropout['time'].min()
                    dropout_pos = np.sqrt(
                        (dropout['ekf_pn'] - dropout['true_pn'])**2 +
                        (dropout['ekf_pe'] - dropout['true_pe'])**2 +
                        (dropout['ekf_pd'] - dropout['true_pd'])**2)
                    print(f"\n  During GNSS dropout ({dropout_dur:.1f}s):")
                    print(f"    Max position error: {dropout_pos.max():.1f}m")
                else:
                    print(f"\n  No GNSS dropout occurred (accel below threshold)")

    # Plot
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)

    if args.show:
        png_path = None
    else:
        png_path = os.path.join(project_dir, "ekf_vs_truth.png")

    try:
        print(f"\nGenerating plots...")
        plot_ekf_vs_truth(df, save_path=png_path, show=args.show)
        if png_path:
            print(f"Done! Plots saved with prefix: {os.path.splitext(png_path)[0]}_*.png")
    except Exception as e:
        print(f"Plotting failed: {e}")
        import traceback
        traceback.print_exc()
