#!/usr/bin/env python3
"""
Analyze roll control PID response from flight data and recommend tuned gains.

Flight: Rolly Poly IV, 2026-03-08
PID used: Kp=0.04, Ki=0.001, Kd=0.0001 (legacy computer)
Servo: PTK 7308 at 8.2V → 0.065s/60° = ~923 deg/s slew rate
Max speed: ~70 m/s

Plant model: dω/dt = K_plant * δ   (roll_accel = gain * fin_deflection)
K_plant depends on V² (aerodynamic moment coefficient).
The PID controls roll rate: δ = Kp*(ω_sp - ω) + Ki*∫(ω_sp - ω)dt + Kd*d(ω_sp - ω)/dt
With setpoint ω_sp = 0, the closed loop for rate is:
    dω/dt = -K_plant * (Kp*ω + Ki*∫ω dt + Kd*dω/dt)
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter1d
import os

CSV_PATH = "/Users/christianpedersen/Documents/Hobbies/Model Rockets/TestFlights/2026_03_08/Raw Downloads/Rolly Poly IV/flight_20260308_173449.csv"
OUT_DIR = os.path.dirname(CSV_PATH)

KP_FLIGHT = 0.04
KI_FLIGHT = 0.001
KD_FLIGHT = 0.0001

# Servo specs (PTK 7308 at 8.2V)
SERVO_SLEW_RATE = 60.0 / 0.065   # deg/s (≈923 deg/s)
SERVO_TAU = 0.065 / 2.0          # approx first-order time constant ~32ms


def load_data(csv_path):
    df = pd.read_csv(csv_path)
    df.columns = df.columns.str.strip()
    return df


def find_launch_time(df):
    col = 'High-G Acceleration Z (m/s2)'
    if col not in df.columns:
        col = 'Low-G Acceleration Z (m/s2)'
    acc = df[col].dropna()
    launch_idx = acc[acc > 30].index[0]
    return df.loc[launch_idx, 'Time (ms)'] / 1000.0


def main():
    print("=" * 72)
    print("ROLL CONTROL PID ANALYSIS — Rolly Poly IV 2026-03-08")
    print(f"Flight PID: Kp={KP_FLIGHT}, Ki={KI_FLIGHT}, Kd={KD_FLIGHT}")
    print("=" * 72)

    df = load_data(CSV_PATH)
    t_launch = find_launch_time(df)
    print(f"Launch at t={t_launch:.3f}s absolute")

    t_abs = df['Time (ms)'].values / 1000.0
    t = t_abs - t_launch  # time since launch

    gyro_x = df['Gyro X (deg/s)'].values
    roll_cmd = df['Roll Command (deg)'].values

    # Speed from nav (may start late)
    vel_e = pd.to_numeric(df['Velocity East (m/s)'], errors='coerce').values
    vel_n = pd.to_numeric(df['Velocity North (m/s)'], errors='coerce').values
    vel_u = pd.to_numeric(df['Velocity Up (m/s)'], errors='coerce').values
    speed = np.sqrt(np.nan_to_num(vel_e)**2 + np.nan_to_num(vel_n)**2 + np.nan_to_num(vel_u)**2)

    # Valid masks
    gv = np.isfinite(gyro_x) & np.isfinite(t)
    cv = np.isfinite(roll_cmd) & np.isfinite(t)
    sv = np.isfinite(speed) & (speed > 0.5) & np.isfinite(t)

    tg, g = t[gv], gyro_x[gv]
    tc, c = t[cv], roll_cmd[cv]
    ts, v = t[sv], speed[sv]

    # ================================================================
    # 1. IDENTIFY CONTROL ENGAGE TIME
    # ================================================================
    # Find first nonzero command
    nonzero = np.where(np.abs(c) > 0.05)[0]
    if len(nonzero) > 0:
        t_control_on = tc[nonzero[0]]
    else:
        t_control_on = 1.0
    print(f"Control appears active at t_rel = {t_control_on:.2f}s")

    # ================================================================
    # 2. ESTIMATE PLANT GAIN (K_plant) FROM TRANSIENT
    # ================================================================
    # During the initial control response, we can estimate K_plant from:
    # dω/dt = K_plant * δ
    # We need angular acceleration and the corresponding command
    print(f"\n--- PLANT IDENTIFICATION ---")

    # Window: from control-on to 1s after
    t0_plant = t_control_on
    t1_plant = t_control_on + 1.0

    mask_g = (tg >= t0_plant) & (tg <= t1_plant)
    mask_c = (tc >= t0_plant) & (tc <= t1_plant)

    if mask_g.sum() > 20 and mask_c.sum() > 5:
        tg_w = tg[mask_g]
        g_w = g[mask_g]

        # Remove duplicate timestamps
        unique_mask = np.diff(tg_w, prepend=-999) > 1e-6
        tg_w = tg_w[unique_mask]
        g_w = g_w[unique_mask]

        if len(tg_w) > 20:
            dt_mean = np.mean(np.diff(tg_w))
            smooth_window = max(3, int(0.02 / max(dt_mean, 1e-6)))
            g_smooth = uniform_filter1d(g_w, size=smooth_window)
            alpha = np.gradient(g_smooth, tg_w)

            cmd_interp = np.interp(tg_w, tc[mask_c], c[mask_c])

            sig = np.abs(cmd_interp) > 0.5
            if sig.sum() > 5:
                valid = np.isfinite(alpha) & np.isfinite(cmd_interp) & sig
                if valid.sum() > 5:
                    K_plant_est = np.sum(alpha[valid] * cmd_interp[valid]) / np.sum(cmd_interp[valid]**2)
                    residual = alpha[valid] - K_plant_est * cmd_interp[valid]
                    ss_res = np.sum(residual**2)
                    ss_tot = np.sum((alpha[valid] - alpha[valid].mean())**2)
                    r_sq = 1 - ss_res / ss_tot if ss_tot > 0 else 0
                    print(f"  K_plant ≈ {K_plant_est:.0f} deg/s² per deg deflection")
                    print(f"  R² = {r_sq:.2f}")
                else:
                    K_plant_est = 200
                    print(f"  K_plant: insufficient valid data, using {K_plant_est}")
            else:
                K_plant_est = 200
                print(f"  K_plant: insufficient cmd data, using {K_plant_est}")
        else:
            K_plant_est = 200
            print(f"  K_plant: insufficient unique timestamps, using {K_plant_est}")
    else:
        K_plant_est = 200
        print(f"  K_plant: insufficient data, using {K_plant_est}")

    # Sanity check K_plant — must be positive for our sign convention
    if not np.isfinite(K_plant_est) or K_plant_est == 0:
        K_plant_est = 200
        print(f"  K_plant fallback: {K_plant_est}")

    # ================================================================
    # 3. MEASURE OSCILLATION CHARACTERISTICS
    # ================================================================
    print(f"\n--- STEADY-STATE OSCILLATION ANALYSIS ---")

    # Analyze the "settled" portion (after initial transient dies down)
    t0_ss = t_control_on + 1.5
    t1_ss = min(t_control_on + 4.0, tg.max())

    mask_ss_g = (tg >= t0_ss) & (tg <= t1_ss)
    mask_ss_c = (tc >= t0_ss) & (tc <= t1_ss)

    if mask_ss_g.sum() > 50:
        g_ss = g[mask_ss_g]
        tg_ss = tg[mask_ss_g]

        # RMS and peak
        rms_rate = np.sqrt(np.mean(g_ss**2))
        peak_rate = np.max(np.abs(g_ss))

        print(f"  Roll rate RMS: {rms_rate:.1f} deg/s")
        print(f"  Roll rate peak: {peak_rate:.1f} deg/s")

        # FFT to find dominant oscillation frequency
        dt_ss = np.mean(np.diff(tg_ss))
        fs_ss = 1.0 / dt_ss
        N = len(g_ss)
        freqs = np.fft.rfftfreq(N, d=dt_ss)
        fft_mag = np.abs(np.fft.rfft(g_ss - g_ss.mean())) * 2.0 / N

        # Find peak frequency (above 0.5 Hz to avoid DC)
        mask_freq = freqs > 0.5
        if mask_freq.sum() > 0:
            peak_idx = np.argmax(fft_mag[mask_freq])
            osc_freq = freqs[mask_freq][peak_idx]
            osc_mag = fft_mag[mask_freq][peak_idx]
            print(f"  Dominant oscillation: {osc_freq:.1f} Hz ({osc_mag:.0f} deg/s amplitude)")
        else:
            osc_freq = 3.0
            print(f"  Using default oscillation frequency: {osc_freq} Hz")
    else:
        rms_rate = 50
        osc_freq = 3.0
        print(f"  Insufficient data for oscillation analysis")

    if mask_ss_c.sum() > 10:
        c_ss = c[mask_ss_c]
        rms_cmd = np.sqrt(np.mean(c_ss**2))
        peak_cmd = np.max(np.abs(c_ss))
        print(f"  Command RMS: {rms_cmd:.2f} deg")
        print(f"  Command peak: {peak_cmd:.2f} deg")
    else:
        rms_cmd = 2.0
        peak_cmd = 5.0

    # Speed during steady state
    mask_ss_s = (ts >= t0_ss) & (ts <= t1_ss)
    if mask_ss_s.sum() > 0:
        v_mean = v[mask_ss_s].mean()
        print(f"  Mean speed: {v_mean:.1f} m/s")
    else:
        v_mean = 50.0
        print(f"  Speed estimate: {v_mean:.0f} m/s (from flight profile)")

    # ================================================================
    # 4. STABILITY ANALYSIS
    # ================================================================
    print(f"\n--- STABILITY ANALYSIS ---")

    # Open-loop crossover frequency with current gains
    # L(s) = K_plant * Kp / s * H_servo(s)
    # |L(jω)| = K_plant * Kp / ω * 1/sqrt(1 + (ω*τ)²)
    # At crossover |L| = 1: ω_c ≈ K_plant * Kp (if servo is fast)
    omega_c = abs(K_plant_est) * KP_FLIGHT
    f_c = omega_c / (2 * np.pi)

    # Phase margin: PM = 90° - atan(ω_c * τ_servo)
    # Servo adds phase lag
    servo_lag = np.degrees(np.arctan(omega_c * SERVO_TAU))
    # Estimate computational delay (~2ms for legacy, ~1ms for mini)
    T_delay = 0.002  # 2ms
    delay_lag = np.degrees(omega_c * T_delay)

    pm = 90 - servo_lag - delay_lag

    print(f"  Open-loop crossover: ω_c ≈ {omega_c:.1f} rad/s ({f_c:.1f} Hz)")
    print(f"  Servo phase lag at ω_c: {servo_lag:.1f}°")
    print(f"  Delay phase lag at ω_c: {delay_lag:.1f}°")
    print(f"  Estimated phase margin: {pm:.1f}°")

    if pm < 30:
        print(f"  → LOW PHASE MARGIN — explains persistent oscillations!")
    elif pm < 45:
        print(f"  → Moderate phase margin — some oscillation expected")
    else:
        print(f"  → Good phase margin")

    # ================================================================
    # 5. PID TUNING RECOMMENDATIONS
    # ================================================================
    print(f"\n{'='*72}")
    print(f"PID TUNING RECOMMENDATIONS FOR MINI COMPUTER")
    print(f"{'='*72}")

    # Target: 45-60° phase margin for well-damped response
    # With derivative: the Kd term adds phase lead
    # L(s) = K_plant * (Kp + Kd*s) / s * H_servo(s)
    # The Kd*s adds +90° phase at high frequency, but limited by servo

    # Design approach: target crossover at f_target Hz
    # Choose f_target such that servo lag is manageable (< 30°)
    # atan(ω*τ) < 30° → ω < tan(30°)/τ = 0.577/0.032 ≈ 18 rad/s ≈ 3 Hz

    # For 45° PM:
    # PM = 90° + atan(ω_c * Kd/Kp) - atan(ω_c * τ) - ω_c * T_delay ≈ 45°
    # Need: atan(ω_c * Kd/Kp) ≈ 45° - 90° + atan(ω_c*τ) + ω_c*T_delay

    # Start with a target crossover frequency
    f_target = 2.0  # Hz
    omega_target = 2 * np.pi * f_target

    # Required Kp for this crossover (without derivative)
    Kp_new = omega_target / abs(K_plant_est)

    # Phase lag at target crossover
    servo_lag_target = np.degrees(np.arctan(omega_target * SERVO_TAU))
    delay_lag_target = np.degrees(omega_target * 0.001)  # mini computer: ~1ms

    pm_p_only = 90 - servo_lag_target - delay_lag_target

    print(f"\n  Target crossover: {f_target:.1f} Hz ({omega_target:.1f} rad/s)")
    print(f"  K_plant estimate: {K_plant_est:.0f} deg/s² per deg")
    print(f"  Servo time constant: {SERVO_TAU*1000:.0f} ms")
    print(f"  Mini computer delay: ~1 ms")

    print(f"\n  P-only at target crossover:")
    print(f"    Kp = ω_target / K_plant = {Kp_new:.4f}")
    print(f"    Phase margin = {pm_p_only:.1f}°")

    # Add derivative for additional phase lead
    # Target PM = 50°
    pm_target = 50.0
    phase_deficit = pm_target - pm_p_only  # need this much extra lead from Kd
    Kd_over_Kp = 0.0
    if phase_deficit > 0:
        # atan(ω_c * Kd/Kp) = phase_deficit
        Kd_over_Kp = np.tan(np.radians(phase_deficit)) / omega_target
        Kd_new = Kp_new * Kd_over_Kp
    else:
        Kd_new = 0.0001  # minimal derivative for noise filtering

    # Small Ki for steady-state offset elimination
    # Ki should be ~0.05-0.1 * Kp * ω_target to keep integral slow
    Ki_new = 0.05 * Kp_new * omega_target

    print(f"\n  With derivative (target PM = {pm_target:.0f}°):")
    print(f"    Phase deficit to fill: {max(0, phase_deficit):.1f}°")
    print(f"    Kd/Kp ratio: {Kd_over_Kp:.4f}")

    print(f"\n  ┌──────────────────────────────────────────┐")
    print(f"  │  RECOMMENDED GAINS (Mini Computer)        │")
    print(f"  ├──────────────────────────────────────────┤")
    print(f"  │  Kp = {Kp_new:.4f}                          │")
    print(f"  │  Ki = {Ki_new:.5f}                         │")
    print(f"  │  Kd = {Kd_new:.5f}                         │")
    print(f"  │  MIN_CMD = -20.0 deg                     │")
    print(f"  │  MAX_CMD = +20.0 deg                     │")
    print(f"  ├──────────────────────────────────────────┤")
    print(f"  │  Gain scheduling:                        │")
    print(f"  │    V_ref = {v_mean:.0f} m/s (flight speed)       │")
    print(f"  │    V_min = 25.0 m/s                      │")
    print(f"  │    Scale cap = 3.0                       │")
    print(f"  └──────────────────────────────────────────┘")

    # Verify: new open-loop crossover and PM
    omega_c_new = abs(K_plant_est) * Kp_new
    # With Kd, effective crossover shifts up slightly
    servo_lag_new = np.degrees(np.arctan(omega_c_new * SERVO_TAU))
    delay_lag_new = np.degrees(omega_c_new * 0.001)
    kd_lead = np.degrees(np.arctan(omega_c_new * Kd_new / Kp_new))
    pm_new = 90 + kd_lead - servo_lag_new - delay_lag_new

    print(f"\n  Verification:")
    print(f"    New crossover: {omega_c_new:.1f} rad/s ({omega_c_new/(2*np.pi):.1f} Hz)")
    print(f"    Servo lag: {servo_lag_new:.1f}°")
    print(f"    Delay lag: {delay_lag_new:.1f}°")
    print(f"    Kd phase lead: +{kd_lead:.1f}°")
    print(f"    Phase margin: {pm_new:.1f}°")

    print(f"\n  Comparison:")
    print(f"    {'Parameter':<12s} {'Flight':>10s} {'Recommended':>12s}")
    print(f"    {'─'*36}")
    print(f"    {'Kp':<12s} {KP_FLIGHT:>10.4f} {Kp_new:>12.4f}")
    print(f"    {'Ki':<12s} {KI_FLIGHT:>10.4f} {Ki_new:>12.5f}")
    print(f"    {'Kd':<12s} {KD_FLIGHT:>10.4f} {Kd_new:>12.5f}")
    print(f"    {'PM':<12s} {pm:>9.0f}° {pm_new:>11.0f}°")

    # ================================================================
    # 6. PLOTS
    # ================================================================
    print(f"\n--- Generating plots ---")

    fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=True)
    fig.suptitle('Roll Control Analysis — Rolly Poly IV (2026-03-08)\n'
                 f'Flight PID: Kp={KP_FLIGHT}, Ki={KI_FLIGHT}, Kd={KD_FLIGHT}',
                 fontsize=13, fontweight='bold')

    # Roll rate
    ax = axes[0]
    ax.plot(tg, g, 'g-', linewidth=0.4, alpha=0.4, label='Gyro X (raw)')
    if len(tg) > 50:
        g_sm = uniform_filter1d(g, size=min(50, len(g)//10))
        ax.plot(tg, g_sm, 'g-', linewidth=2, label='Roll rate (50pt avg)')
    ax.axhline(0, color='k', linewidth=0.5, alpha=0.3)
    ax.axvline(t_control_on, color='r', linewidth=1.5, linestyle='--', label=f'Control ON (t={t_control_on:.1f}s)')
    ax.set_ylabel('Roll Rate (deg/s)')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_title('Roll Rate (Gyro X)', fontsize=10)

    # Roll command
    ax = axes[1]
    ax.plot(tc, c, 'C1-', linewidth=0.8, label='Roll command')
    ax.axhline(0, color='k', linewidth=0.5, alpha=0.3)
    ax.axvline(t_control_on, color='r', linewidth=1.5, linestyle='--')
    ax.set_ylabel('Command (deg)')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_title('Roll Command (PID output → servo deflection)', fontsize=10)

    # Speed
    ax = axes[2]
    if len(ts) > 0:
        ax.plot(ts, v, 'k-', linewidth=1.5, label='Airspeed')
    ax.axvline(t_control_on, color='r', linewidth=1.5, linestyle='--')
    ax.set_ylabel('Speed (m/s)')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_title('Airspeed (nav solution)', fontsize=10)

    # Zoomed steady state: rate + cmd overlaid
    ax = axes[3]
    t0z, t1z = t0_ss, t1_ss
    mg = (tg >= t0z) & (tg <= t1z)
    mc = (tc >= t0z) & (tc <= t1z)
    ax.plot(tg[mg], g[mg], 'g-', linewidth=0.8, label='Roll rate (deg/s)')
    ax_r = ax.twinx()
    ax_r.plot(tc[mc], c[mc], 'C1-', linewidth=1.0, label='Roll cmd (deg)')
    ax.set_ylabel('Roll Rate (deg/s)', color='g')
    ax_r.set_ylabel('Command (deg)', color='C1')
    ax.axhline(0, color='k', linewidth=0.5, alpha=0.3)
    ax.grid(True, alpha=0.3)
    ax.set_title(f'Steady-State Detail (t={t0z:.1f}–{t1z:.1f}s)', fontsize=10)
    ax.set_xlabel('Time since launch (s)')

    # Legends
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax_r.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc='upper right')

    for a in axes:
        a.set_xlim(-0.5, 7.0)

    plt.tight_layout()
    p1 = os.path.join(OUT_DIR, "roll_pid_analysis.png")
    plt.savefig(p1, dpi=150, bbox_inches='tight')
    print(f"  Saved: {p1}")
    plt.close()

    # --- Transient detail ---
    fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    fig.suptitle(f'Transient Response (control ON at t={t_control_on:.1f}s)',
                 fontsize=13, fontweight='bold')

    t0t = t_control_on - 0.5
    t1t = t_control_on + 2.0

    ax = axes[0]
    m = (tg >= t0t) & (tg <= t1t)
    ax.plot(tg[m], g[m], 'g-', linewidth=1.0, label='Roll rate')
    ax.axhline(0, color='k', linewidth=0.5)
    ax.axvline(t_control_on, color='r', linewidth=1.5, linestyle='--', label='Control ON')
    ax.set_ylabel('Roll Rate (deg/s)')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    m = (tc >= t0t) & (tc <= t1t)
    ax.plot(tc[m], c[m], 'C1-', linewidth=1.0, label='Roll command')
    ax.axhline(0, color='k', linewidth=0.5)
    ax.axvline(t_control_on, color='r', linewidth=1.5, linestyle='--')
    ax.set_ylabel('Command (deg)')
    ax.set_xlabel('Time since launch (s)')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    p2 = os.path.join(OUT_DIR, "roll_pid_transient.png")
    plt.savefig(p2, dpi=150, bbox_inches='tight')
    print(f"  Saved: {p2}")
    plt.close()

    # --- FFT of steady-state ---
    fig, ax = plt.subplots(figsize=(10, 5))
    if mask_ss_g.sum() > 50:
        g_ss = g[mask_ss_g]
        tg_ss = tg[mask_ss_g]
        dt_ss = np.mean(np.diff(tg_ss))
        N = len(g_ss)
        freqs = np.fft.rfftfreq(N, d=dt_ss)
        fft_mag = np.abs(np.fft.rfft(g_ss - g_ss.mean())) * 2.0 / N
        ax.semilogy(freqs, fft_mag, 'b-', linewidth=0.8)
        ax.set_xlim(0, 50)
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Amplitude (deg/s)')
        ax.set_title('FFT of Roll Rate (steady-state window)')
        ax.grid(True, alpha=0.3)
        ax.axvline(osc_freq, color='r', linestyle='--', linewidth=1,
                   label=f'Peak: {osc_freq:.1f} Hz')
        ax.legend()
    plt.tight_layout()
    p3 = os.path.join(OUT_DIR, "roll_pid_fft.png")
    plt.savefig(p3, dpi=150, bbox_inches='tight')
    print(f"  Saved: {p3}")
    plt.close()

    print("\nDone.")


if __name__ == "__main__":
    main()
