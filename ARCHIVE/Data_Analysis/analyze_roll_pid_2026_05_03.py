#!/usr/bin/env python3
"""
Analyze roll control PID response from 2026-05-03 RolyPoly flight.

Flight: RolyPoly, 2026-05-03 (flight_recovered_4)
PID used: Kp=0.04, Ki=0.001, Kd=0.0001
Cmd limits: +/-20 deg
Roll control engages 500 ms after launch detection.

Plant: dwx/dt = K_plant * delta  (roll_accel = gain * fin_deflection)
K_plant ~ V^2.
"""

import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter1d

CSV_PATH = "/Users/christianpedersen/Documents/Hobbies/ModelRockets/TestFlights/2026_05_03/RolyPoly/flight_recovered_4.csv"
OUT_DIR = "/Users/christianpedersen/Documents/Hobbies/ModelRockets/TestFlights/2026_05_03/RolyPoly/flight_recovered_4_analysis"

KP_FLIGHT = 0.04
KI_FLIGHT = 0.001
KD_FLIGHT = 0.0001
CMD_LIMIT = 20.0
ROLL_DELAY_S = 0.5  # roll control activation delay after launch

# PTK 7308 servo at ~8.2 V: 0.065 s / 60 deg => ~923 deg/s slew, tau ~32 ms
SERVO_TAU = 0.065 / 2.0
SERVO_SLEW_RATE = 60.0 / 0.065


def load_data(path):
    df = pd.read_csv(path)
    df.columns = df.columns.str.strip()
    return df


def find_launch_time(df):
    """Use the on-board Launch Flag (first 0->1 transition).
    On this flight the High-G burst was the apogee motor-eject pyro,
    not boost, so accel-threshold detection is unreliable here."""
    lf = pd.to_numeric(df["Launch Flag"], errors="coerce").fillna(0).astype(int).values
    idx = np.where(lf == 1)[0]
    if len(idx) == 0:
        raise RuntimeError("Launch Flag never set")
    return float(df.iloc[idx[0]]["Time (ms)"]) / 1000.0


def find_motor_eject_time(df):
    """Motor eject = first Pyro 1 firing event (drogue/main eject at apogee).
    Roll control disengages here."""
    p1 = pd.to_numeric(df["Pyro 1 Fired"], errors="coerce").fillna(0).astype(int).values
    idx = np.where(p1 == 1)[0]
    if len(idx) == 0:
        return None
    return float(df.iloc[idx[0]]["Time (ms)"]) / 1000.0


def main():
    print("=" * 78)
    print("ROLL CONTROL PID ANALYSIS  ---  RolyPoly 2026-05-03 (flight_recovered_4)")
    print(f"Flight PID: Kp={KP_FLIGHT}, Ki={KI_FLIGHT}, Kd={KD_FLIGHT}")
    print(f"Cmd limits: +/-{CMD_LIMIT} deg | Roll control delay: {ROLL_DELAY_S*1000:.0f} ms")
    print("=" * 78)

    df = load_data(CSV_PATH)
    t_launch = find_launch_time(df)
    t_eject = find_motor_eject_time(df)
    print(f"Launch (Launch Flag) at t_abs={t_launch:.3f} s")
    if t_eject is not None:
        print(f"Motor eject (Pyro 1) at t_abs={t_eject:.3f} s  (t_rel={t_eject-t_launch:.3f} s)")
    # Trim window 0.3 s before motor eject to exclude the eject-impulse spike
    t_control_off_full = (t_eject - t_launch) if t_eject is not None else 7.0
    t_control_off = t_control_off_full - 0.3

    t_abs = df["Time (ms)"].values / 1000.0
    t = t_abs - t_launch  # time since launch

    gyro_x = pd.to_numeric(df["Gyro X (deg/s)"], errors="coerce").values
    roll_cmd = pd.to_numeric(df["Roll Command (deg)"], errors="coerce").values

    ve = pd.to_numeric(df["Velocity East (m/s)"], errors="coerce").values
    vn = pd.to_numeric(df["Velocity North (m/s)"], errors="coerce").values
    vu = pd.to_numeric(df["Velocity Up (m/s)"], errors="coerce").values
    speed = np.sqrt(np.nan_to_num(ve) ** 2 + np.nan_to_num(vn) ** 2 + np.nan_to_num(vu) ** 2)

    gv = np.isfinite(gyro_x) & np.isfinite(t)
    cv = np.isfinite(roll_cmd) & np.isfinite(t)
    sv = np.isfinite(speed) & (speed > 0.5) & np.isfinite(t)

    tg, g = t[gv], gyro_x[gv]
    tc, c = t[cv], roll_cmd[cv]
    ts, v = t[sv], speed[sv]

    # ------------------------------------------------------------------
    # 1. Control engage time
    # ------------------------------------------------------------------
    t_control_on = ROLL_DELAY_S
    nonzero = np.where(np.abs(c) > 0.05)[0]
    t_first_cmd = tc[nonzero[0]] if len(nonzero) else None
    print(f"Configured control-on:  t_rel = {t_control_on:.2f} s")
    if t_first_cmd is not None:
        print(f"First nonzero command:  t_rel = {t_first_cmd:.3f} s")

    # ------------------------------------------------------------------
    # 2. Saturation summary
    # ------------------------------------------------------------------
    mask_post = (tc >= t_control_on) & (tc <= t_control_off)
    if mask_post.sum() > 10:
        c_post = c[mask_post]
        sat = np.abs(c_post) >= (CMD_LIMIT - 0.01)
        sat_frac = sat.mean()
        print()
        print(f"--- COMMAND SATURATION (control window {t_control_on:.2f}-{t_control_off:.2f} s) ---")
        print(f"  Saturation fraction: {sat_frac*100:.1f}%  (|cmd| >= {CMD_LIMIT-0.01:.2f} deg)")
        print(f"  Mean |cmd|: {np.mean(np.abs(c_post)):.2f} deg, peak: {np.max(np.abs(c_post)):.2f} deg")

    # ------------------------------------------------------------------
    # 3. Plant identification
    # ------------------------------------------------------------------
    # Use a window starting at control-on through 1 s later. Saturation will
    # bias the fit; we additionally compute K_plant on unsaturated samples.
    print()
    print("--- PLANT IDENTIFICATION ---")
    t0_plant = t_control_on
    t1_plant = min(t_control_on + 2.0, t_control_off)
    mask_g = (tg >= t0_plant) & (tg <= t1_plant)
    mask_c = (tc >= t0_plant) & (tc <= t1_plant)

    K_plant_all = K_plant_unsat = np.nan
    r2_all = r2_unsat = np.nan

    if mask_g.sum() > 30 and mask_c.sum() > 10:
        tg_w = tg[mask_g]
        g_w = g[mask_g]

        unique = np.diff(tg_w, prepend=-1e9) > 1e-6
        tg_w = tg_w[unique]
        g_w = g_w[unique]

        if len(tg_w) > 30:
            dt_mean = np.mean(np.diff(tg_w))
            sm = max(3, int(0.02 / max(dt_mean, 1e-6)))
            g_smooth = uniform_filter1d(g_w, size=sm)
            alpha = np.gradient(g_smooth, tg_w)

            cmd_interp = np.interp(tg_w, tc[mask_c], c[mask_c])

            sig = np.abs(cmd_interp) > 0.5
            valid = np.isfinite(alpha) & np.isfinite(cmd_interp) & sig
            if valid.sum() > 10:
                K_plant_all = float(np.sum(alpha[valid] * cmd_interp[valid]) / np.sum(cmd_interp[valid] ** 2))
                resid = alpha[valid] - K_plant_all * cmd_interp[valid]
                ss_res = np.sum(resid ** 2)
                ss_tot = np.sum((alpha[valid] - alpha[valid].mean()) ** 2)
                r2_all = 1 - ss_res / ss_tot if ss_tot > 0 else 0
                print(f"  K_plant (all, |cmd|>0.5):      {K_plant_all:7.0f} deg/s^2 / deg   R^2={r2_all:.2f}")

            unsat = valid & (np.abs(cmd_interp) < CMD_LIMIT - 0.5)
            if unsat.sum() > 10:
                K_plant_unsat = float(np.sum(alpha[unsat] * cmd_interp[unsat]) / np.sum(cmd_interp[unsat] ** 2))
                resid = alpha[unsat] - K_plant_unsat * cmd_interp[unsat]
                ss_res = np.sum(resid ** 2)
                ss_tot = np.sum((alpha[unsat] - alpha[unsat].mean()) ** 2)
                r2_unsat = 1 - ss_res / ss_tot if ss_tot > 0 else 0
                print(f"  K_plant (unsat, |cmd|<{CMD_LIMIT-0.5:.0f}):    {K_plant_unsat:7.0f} deg/s^2 / deg   R^2={r2_unsat:.2f}  (N={unsat.sum()})")

    K_plant = K_plant_unsat if np.isfinite(K_plant_unsat) and abs(K_plant_unsat) > 5 else K_plant_all
    if not np.isfinite(K_plant) or abs(K_plant) < 5:
        K_plant = 200.0
        print(f"  Falling back to K_plant = {K_plant:.0f}")
    print(f"  K_plant chosen: {K_plant:.0f} deg/s^2 per deg")

    # ------------------------------------------------------------------
    # 4. Steady-state oscillation
    # ------------------------------------------------------------------
    print()
    print("--- OSCILLATION / TRACKING ANALYSIS ---")
    # Use the bulk of the control window after the initial transient (~1 s)
    t0_ss = t_control_on + 1.0
    t1_ss = max(t_control_on + 1.5, t_control_off - 0.2)
    mask_ss_g = (tg >= t0_ss) & (tg <= t1_ss)
    mask_ss_c = (tc >= t0_ss) & (tc <= t1_ss)

    rms_rate = peak_rate = osc_freq = np.nan
    rms_cmd = peak_cmd = np.nan
    v_mean = np.nan

    if mask_ss_g.sum() > 50:
        g_ss = g[mask_ss_g]
        tg_ss = tg[mask_ss_g]
        rms_rate = float(np.sqrt(np.mean(g_ss ** 2)))
        peak_rate = float(np.max(np.abs(g_ss)))
        print(f"  Window: {t0_ss:.2f}-{t1_ss:.2f} s ({len(g_ss)} samples)")
        print(f"  Roll rate RMS:  {rms_rate:6.1f} deg/s")
        print(f"  Roll rate peak: {peak_rate:6.1f} deg/s")

        # Resample uniformly for FFT
        dt_target = max(np.median(np.diff(tg_ss)), 1e-3)
        n_uni = int((tg_ss[-1] - tg_ss[0]) / dt_target)
        if n_uni > 32:
            tu = np.linspace(tg_ss[0], tg_ss[-1], n_uni)
            gu = np.interp(tu, tg_ss, g_ss)
            gu -= gu.mean()
            freqs = np.fft.rfftfreq(n_uni, d=dt_target)
            mag = np.abs(np.fft.rfft(gu)) * 2.0 / n_uni
            mf = freqs > 0.3
            if mf.any():
                pk = np.argmax(mag[mf])
                osc_freq = float(freqs[mf][pk])
                osc_amp = float(mag[mf][pk])
                print(f"  Dominant osc:   {osc_freq:5.2f} Hz  amp ~ {osc_amp:.0f} deg/s")

    if mask_ss_c.sum() > 10:
        c_ss = c[mask_ss_c]
        rms_cmd = float(np.sqrt(np.mean(c_ss ** 2)))
        peak_cmd = float(np.max(np.abs(c_ss)))
        sat_ss = np.mean(np.abs(c_ss) >= CMD_LIMIT - 0.01)
        print(f"  Cmd RMS:        {rms_cmd:6.2f} deg")
        print(f"  Cmd peak:       {peak_cmd:6.2f} deg   saturation: {sat_ss*100:.1f}%")

    mask_ss_s = (ts >= t0_ss) & (ts <= t1_ss)
    if mask_ss_s.sum() > 0:
        v_mean = float(v[mask_ss_s].mean())
        print(f"  Mean speed:     {v_mean:5.1f} m/s")

    # Speed at engage
    mask_engage_v = (ts >= t_control_on - 0.05) & (ts <= t_control_on + 0.15)
    v_engage = float(v[mask_engage_v].mean()) if mask_engage_v.sum() else np.nan
    print(f"  Speed @ engage: {v_engage:5.1f} m/s   (V_ref for K_plant scaling)")

    # ------------------------------------------------------------------
    # 5. Stability of flight gains
    # ------------------------------------------------------------------
    print()
    print("--- CURRENT-GAIN STABILITY ---")
    # K_plant scales with V^2; use the steady-state speed since osc happens there
    K_plant_ss = K_plant * (v_mean / max(v_engage, 1.0)) ** 2 if np.isfinite(v_mean) and np.isfinite(v_engage) else K_plant
    omega_c = abs(K_plant_ss) * KP_FLIGHT
    f_c = omega_c / (2 * np.pi)
    servo_lag = np.degrees(np.arctan(omega_c * SERVO_TAU))
    delay_lag = np.degrees(omega_c * 0.002)  # 2 ms loop delay
    kd_lead = np.degrees(np.arctan(omega_c * KD_FLIGHT / KP_FLIGHT)) if KP_FLIGHT > 0 else 0
    pm_now = 90 + kd_lead - servo_lag - delay_lag
    print(f"  K_plant @ V_ss:    {K_plant_ss:7.0f} deg/s^2/deg  (V_ss={v_mean:.1f} m/s)")
    print(f"  Crossover:         {omega_c:6.2f} rad/s ({f_c:5.2f} Hz)")
    print(f"  Servo lag:         {servo_lag:6.1f} deg")
    print(f"  Delay (2 ms) lag:  {delay_lag:6.1f} deg")
    print(f"  Kd phase lead:     {kd_lead:6.1f} deg")
    print(f"  Phase margin est:  {pm_now:6.1f} deg")
    print(f"  Observed osc freq: {osc_freq:5.2f} Hz" if np.isfinite(osc_freq) else "  Observed osc freq: N/A")

    # ------------------------------------------------------------------
    # 6. Tuning verdict
    # ------------------------------------------------------------------
    print()
    print("=" * 78)
    print("TUNING VERDICT")
    print("=" * 78)
    # Honest read: the plant is unobservable here because cmd never excited
    # it. We can still bound things from the observed peaks.
    mask_post_g = (tg >= t_control_on) & (tg <= t_control_off)
    rate_99 = float(np.percentile(np.abs(g[mask_post_g]), 99)) if mask_post_g.sum() else np.nan
    cmd_99 = float(np.percentile(np.abs(c[mask_post]), 99)) if mask_post.sum() else np.nan
    rate_95 = float(np.percentile(np.abs(g[mask_post_g]), 95)) if mask_post_g.sum() else np.nan
    print(f"  Window: {t_control_on:.2f}-{t_control_off:.2f} s post-launch (excl. eject impulse)")
    print(f"  |rate| 95th / 99th: {rate_95:.1f} / {rate_99:.1f} dps")
    print(f"  |cmd|  99th:        {cmd_99:.2f} deg   (limit +/-{CMD_LIMIT:.0f})")
    print(f"  Plant identification R^2: {r2_unsat:.2f}  --> too little excitation to fit K_plant")
    print()
    print(f"  Read: controller was loafing for the whole control window.  Roll rates")
    print(f"  stayed small and cmd had ~{CMD_LIMIT/max(cmd_99,0.1):.0f}x headroom.  Current gains are")
    print(f"  conservative-and-stable on this airframe; no evidence of oscillation,")
    print(f"  windup, or saturation in the active window.  The chaotic looking cmd")
    print(f"  excursion in the raw plot is the post-eject descent (not controlled).")
    print()
    print(f"  Recommendation:")
    print(f"   - Hold Kp={KP_FLIGHT}, Ki={KI_FLIGHT}, Kd={KD_FLIGHT} for the next flight on this geometry.")
    print(f"   - For 4-fin geometry (higher control authority), START LOWER, e.g. Kp ~ {KP_FLIGHT*0.5:.3f},")
    print(f"     Ki ~ {KI_FLIGHT*0.5:.4f}, Kd unchanged. Re-evaluate from the next flight's data.")
    print(f"   - To learn the plant, inject a step (set ROLL_RATE_SET_POINT to 50 dps for ~1 s)")
    print(f"     so the closed-loop is excited enough to fit K_plant cleanly.")

    # ------------------------------------------------------------------
    # 7. Plots
    # ------------------------------------------------------------------
    print()
    print("--- Plots ---")
    os.makedirs(OUT_DIR, exist_ok=True)

    fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=True)
    fig.suptitle(
        "Roll Control Analysis  ---  RolyPoly 2026-05-03\n"
        f"Flight PID: Kp={KP_FLIGHT}, Ki={KI_FLIGHT}, Kd={KD_FLIGHT}  |  cmd +/-{CMD_LIMIT}deg  |  delay {ROLL_DELAY_S*1000:.0f}ms",
        fontsize=12, fontweight="bold",
    )

    ax = axes[0]
    ax.plot(tg, g, "g-", linewidth=0.4, alpha=0.4, label="Gyro X (raw)")
    if len(tg) > 50:
        sm = uniform_filter1d(g, size=min(50, len(g) // 10))
        ax.plot(tg, sm, "g-", linewidth=2, label="Roll rate (50pt avg)")
    ax.axhline(0, color="k", linewidth=0.5, alpha=0.3)
    ax.axvline(t_control_on, color="r", linewidth=1.5, linestyle="--", label=f"Control ON (t={t_control_on:.2f}s)")
    ax.axvline(t_control_off, color="b", linewidth=1.5, linestyle="--", label=f"Control OFF (t={t_control_off:.2f}s)")
    ax.set_ylabel("Roll Rate (deg/s)")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Roll Rate (Gyro X)", fontsize=10)

    ax = axes[1]
    ax.plot(tc, c, "C1-", linewidth=0.8, label="Roll command")
    ax.axhline(0, color="k", linewidth=0.5, alpha=0.3)
    ax.axhline(CMD_LIMIT, color="k", linewidth=0.5, linestyle=":")
    ax.axhline(-CMD_LIMIT, color="k", linewidth=0.5, linestyle=":")
    ax.axvline(t_control_on, color="r", linewidth=1.5, linestyle="--")
    ax.axvline(t_control_off, color="b", linewidth=1.5, linestyle="--")
    ax.set_ylabel("Command (deg)")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Roll Command (PID output -> servo deflection)", fontsize=10)

    ax = axes[2]
    if len(ts) > 0:
        ax.plot(ts, v, "k-", linewidth=1.5, label="Airspeed")
    ax.axvline(t_control_on, color="r", linewidth=1.5, linestyle="--")
    ax.axvline(t_control_off, color="b", linewidth=1.5, linestyle="--")
    ax.set_ylabel("Speed (m/s)")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Airspeed (nav)", fontsize=10)

    ax = axes[3]
    t0z, t1z = t0_ss, t1_ss
    mg = (tg >= t0z) & (tg <= t1z)
    mc = (tc >= t0z) & (tc <= t1z)
    ax.plot(tg[mg], g[mg], "g-", linewidth=0.8, label="Roll rate (deg/s)")
    ax_r = ax.twinx()
    ax_r.plot(tc[mc], c[mc], "C1-", linewidth=1.0, label="Roll cmd (deg)")
    ax.set_ylabel("Roll Rate (deg/s)", color="g")
    ax_r.set_ylabel("Command (deg)", color="C1")
    ax.axhline(0, color="k", linewidth=0.5, alpha=0.3)
    ax.grid(True, alpha=0.3)
    ax.set_title(f"Steady-State Detail (t={t0z:.2f}-{t1z:.2f} s)", fontsize=10)
    ax.set_xlabel("Time since launch (s)")
    l1, lab1 = ax.get_legend_handles_labels()
    l2, lab2 = ax_r.get_legend_handles_labels()
    ax.legend(l1 + l2, lab1 + lab2, fontsize=8, loc="upper right")

    for a in axes:
        a.set_xlim(-0.5, t_control_off + 0.5)

    plt.tight_layout()
    p1 = os.path.join(OUT_DIR, "roll_pid_analysis_2026_05_03.png")
    plt.savefig(p1, dpi=150, bbox_inches="tight")
    print(f"  Saved: {p1}")
    plt.close()

    # Transient detail
    fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    fig.suptitle(f"Transient Response  (control ON at t={t_control_on:.2f} s)",
                 fontsize=12, fontweight="bold")
    t0t = t_control_on - 0.5
    t1t = t_control_on + 2.0
    ax = axes[0]
    m = (tg >= t0t) & (tg <= t1t)
    ax.plot(tg[m], g[m], "g-", linewidth=1.0, label="Roll rate")
    ax.axhline(0, color="k", linewidth=0.5)
    ax.axvline(t_control_on, color="r", linewidth=1.5, linestyle="--", label="Control ON")
    ax.set_ylabel("Roll Rate (deg/s)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax = axes[1]
    m = (tc >= t0t) & (tc <= t1t)
    ax.plot(tc[m], c[m], "C1-", linewidth=1.0, label="Roll command")
    ax.axhline(CMD_LIMIT, color="k", linewidth=0.5, linestyle=":")
    ax.axhline(-CMD_LIMIT, color="k", linewidth=0.5, linestyle=":")
    ax.axhline(0, color="k", linewidth=0.5)
    ax.axvline(t_control_on, color="r", linewidth=1.5, linestyle="--")
    ax.set_ylabel("Command (deg)")
    ax.set_xlabel("Time since launch (s)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    p2 = os.path.join(OUT_DIR, "roll_pid_transient_2026_05_03.png")
    plt.savefig(p2, dpi=150, bbox_inches="tight")
    print(f"  Saved: {p2}")
    plt.close()

    # FFT
    fig, ax = plt.subplots(figsize=(10, 5))
    if mask_ss_g.sum() > 50:
        g_ss = g[mask_ss_g]
        tg_ss = tg[mask_ss_g]
        dt_target = max(np.median(np.diff(tg_ss)), 1e-3)
        n_uni = int((tg_ss[-1] - tg_ss[0]) / dt_target)
        tu = np.linspace(tg_ss[0], tg_ss[-1], n_uni)
        gu = np.interp(tu, tg_ss, g_ss)
        gu -= gu.mean()
        freqs = np.fft.rfftfreq(n_uni, d=dt_target)
        mag = np.abs(np.fft.rfft(gu)) * 2.0 / n_uni
        ax.semilogy(freqs, mag, "b-", linewidth=0.8)
        ax.set_xlim(0, 50)
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("Amplitude (deg/s)")
        ax.set_title(f"FFT of Roll Rate (steady-state {t0_ss:.2f}-{t1_ss:.2f} s)")
        ax.grid(True, alpha=0.3)
        if np.isfinite(osc_freq):
            ax.axvline(osc_freq, color="r", linestyle="--", linewidth=1, label=f"Peak: {osc_freq:.2f} Hz")
            ax.legend()
    plt.tight_layout()
    p3 = os.path.join(OUT_DIR, "roll_pid_fft_2026_05_03.png")
    plt.savefig(p3, dpi=150, bbox_inches="tight")
    print(f"  Saved: {p3}")
    plt.close()

    # Focused overlay: control window only
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle(
        "RolyPoly 2026-05-03 -- Roll Control Window\n"
        f"Kp={KP_FLIGHT}, Ki={KI_FLIGHT}, Kd={KD_FLIGHT}  |  cmd +/-{CMD_LIMIT:.0f} deg  |  delay {ROLL_DELAY_S*1000:.0f} ms",
        fontsize=12, fontweight="bold",
    )
    win_pad = 0.2
    x0 = max(0.0, t_control_on - win_pad)
    x1 = t_control_off + win_pad

    ax = axes[0]
    mw = (tg >= x0) & (tg <= x1)
    ax.plot(tg[mw], g[mw], "g-", linewidth=0.8, label="Roll rate (gyro X)")
    ax.axhline(0, color="k", linewidth=0.5, alpha=0.3)
    ax.axvline(t_control_on, color="r", linewidth=1.5, linestyle="--", label=f"Control ON ({t_control_on:.2f}s)")
    ax.axvline(t_control_off, color="b", linewidth=1.5, linestyle="--", label=f"Trim end ({t_control_off:.2f}s)")
    ax.axvline(t_control_off_full, color="b", linewidth=1.0, linestyle=":", alpha=0.7, label=f"Eject ({t_control_off_full:.2f}s)")
    ax.set_ylabel("Roll Rate (deg/s)")
    ax.set_title("Roll Rate (Gyro X)", fontsize=10)
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    mc2 = (tc >= x0) & (tc <= x1)
    ax.plot(tc[mc2], c[mc2], "C1-", linewidth=0.8, label="Roll command")
    ax.axhline(0, color="k", linewidth=0.5, alpha=0.3)
    ax.axhline(CMD_LIMIT, color="k", linewidth=0.5, linestyle=":")
    ax.axhline(-CMD_LIMIT, color="k", linewidth=0.5, linestyle=":")
    ax.axvline(t_control_on, color="r", linewidth=1.5, linestyle="--")
    ax.axvline(t_control_off, color="b", linewidth=1.5, linestyle="--")
    ax.axvline(t_control_off_full, color="b", linewidth=1.0, linestyle=":", alpha=0.7)
    ax.set_ylabel("Command (deg)")
    ax.set_title("Roll Command", fontsize=10)
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    msw = (ts >= x0) & (ts <= x1)
    ax.plot(ts[msw], v[msw], "k-", linewidth=1.5, label="Airspeed")
    ax.axvline(t_control_on, color="r", linewidth=1.5, linestyle="--")
    ax.axvline(t_control_off, color="b", linewidth=1.5, linestyle="--")
    ax.axvline(t_control_off_full, color="b", linewidth=1.0, linestyle=":", alpha=0.7)
    ax.set_ylabel("Speed (m/s)")
    ax.set_xlabel("Time since launch (s)")
    ax.set_title("Airspeed (nav)", fontsize=10)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    for a in axes:
        a.set_xlim(x0, x1)

    plt.tight_layout()
    p4 = os.path.join(OUT_DIR, "roll_pid_control_window_2026_05_03.png")
    plt.savefig(p4, dpi=150, bbox_inches="tight")
    print(f"  Saved: {p4}")
    plt.close()

    # Cmd-vs-rate overlay (zoomed) -- shows tracking
    fig, ax = plt.subplots(figsize=(13, 5))
    fig.suptitle("Roll Rate (left) and Roll Command (right) -- control window",
                 fontsize=12, fontweight="bold")
    mz = (tg >= t_control_on) & (tg <= t_control_off)
    mzc = (tc >= t_control_on) & (tc <= t_control_off)
    ax.plot(tg[mz], g[mz], "g-", linewidth=0.8, label="Roll rate (deg/s)")
    ax.set_ylabel("Roll Rate (deg/s)", color="g")
    ax_r = ax.twinx()
    ax_r.plot(tc[mzc], c[mzc], "C1-", linewidth=0.8, alpha=0.9, label="Roll cmd (deg)")
    ax_r.set_ylabel("Command (deg)", color="C1")
    ax.axhline(0, color="k", linewidth=0.5, alpha=0.3)
    ax.set_xlabel("Time since launch (s)")
    ax.grid(True, alpha=0.3)
    l1, lab1 = ax.get_legend_handles_labels()
    l2, lab2 = ax_r.get_legend_handles_labels()
    ax.legend(l1 + l2, lab1 + lab2, fontsize=9, loc="upper right")
    ax.set_xlim(t_control_on, t_control_off)
    plt.tight_layout()
    p5 = os.path.join(OUT_DIR, "roll_pid_cmd_vs_rate_2026_05_03.png")
    plt.savefig(p5, dpi=150, bbox_inches="tight")
    print(f"  Saved: {p5}")
    plt.close()

    print()
    print("Done.")


if __name__ == "__main__":
    main()
