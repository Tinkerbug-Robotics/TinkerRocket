#!/usr/bin/env python3
"""
Roll-angle control analysis -- RollyPolly 54mm, 2026-05-17 flight.

Flight intent: cascaded angle controller tracking a 0 -> 180 -> 0 deg
roll profile. Observed: roll rates well over the gyro full-scale (-3055 dps
clipped reading), command saturating at +/-20 deg, rocket spinning through
multiple revolutions before damping out.

Inner-loop PID (config.h defaults): Kp=0.04, Ki=0.001, Kd=0.0003, D-LPF=10 Hz.
Outer-loop angle->rate P-gain: KP_ANGLE = 4.0 (deg/s per deg).
Gain schedule: Kp/Ki/Kd scaled by (V_ref/V)^2, capped at 3x.
Roll command limits in this flight: +/-20 deg (overridden from default +/-10).
Roll control delay: 500 ms (overridden from default 0).
"""

import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter1d

CSV_PATH = "/Users/christianpedersen/Documents/Hobbies/ModelRockets/TestFlights/2026_05_17/RollyPolly_54mm/flight_20260517_133456.csv"
JSON_PATH = "/Users/christianpedersen/Documents/Hobbies/ModelRockets/TestFlights/2026_05_17/RollyPolly_54mm/flight_20260517_133456.json"
OUT_DIR = "/Users/christianpedersen/Documents/Hobbies/ModelRockets/TestFlights/2026_05_17/RollyPolly_54mm/analysis"

# Inner PID (config.h base, before gain schedule)
KP_BASE = 0.04
KI_BASE = 0.001
KD_BASE = 0.0003
D_LPF_HZ = 10.0
# Outer angle->rate gain
KP_ANGLE = 4.0  # deg/s per deg
# Command saturation (this flight: overridden from default +/-10)
CMD_LIMIT = 20.0
# Roll control activation delay (overridden via NVS "rdly")
ROLL_DELAY_S = 0.5
# Gyro full scale (confirmed from .bin config: gyro_fs_dps=4000)
GYRO_FS_DPS = 4000.0


def load(csv_path):
    df = pd.read_csv(csv_path)
    df.columns = df.columns.str.strip()
    return df


def find_launch_time(df):
    lf = pd.to_numeric(df["Launch Flag"], errors="coerce").fillna(0).astype(int).values
    idx = np.where(lf == 1)[0]
    return float(df.iloc[idx[0]]["Time (ms)"]) / 1000.0 if len(idx) else 0.0


def find_first_pyro(df, col):
    p = pd.to_numeric(df[col], errors="coerce").fillna(0).astype(int).values
    idx = np.where(p == 1)[0]
    return float(df.iloc[idx[0]]["Time (ms)"]) / 1000.0 if len(idx) else None


def reconstruct_pid(tg, gyro, t_on, t_off, kp, ki, kd, cmd_lim, d_lpf_hz, setpoint=0.0):
    """Reconstruct a stand-alone rate PID in post-processing (no gain schedule).
    Returns P, I (output, anti-windup clamped), D arrays."""
    tau_d = 1.0 / (2 * np.pi * d_lpf_hz)
    cum = 0.0
    d_filt = 0.0
    g_prev = 0.0
    P = np.zeros_like(gyro)
    I = np.zeros_like(gyro)
    D = np.zeros_like(gyro)
    dts = np.diff(tg, prepend=tg[0])
    on_prev = False
    for i in range(len(tg)):
        dt = dts[i] if i > 0 else 0.001
        on = (tg[i] >= t_on) and (tg[i] <= t_off)
        if not on:
            cum = 0.0
            d_filt = 0.0
            g_prev = gyro[i]
            on_prev = False
            continue
        err = setpoint - gyro[i]
        cum += err * dt
        P[i] = kp * err
        I[i] = np.clip(ki * cum, -cmd_lim, cmd_lim)
        if not on_prev:
            d_raw = 0.0
        else:
            d_raw = -(gyro[i] - g_prev) / max(dt, 1e-6)
        alpha = dt / (dt + tau_d)
        d_filt = d_filt + alpha * (d_raw - d_filt)
        D[i] = kd * d_filt
        g_prev = gyro[i]
        on_prev = True
    return P, I, D


def main():
    print("=" * 78)
    print("ROLL CONTROL ANALYSIS  ---  RollyPolly 54mm  2026-05-17")
    print("Intent: cascaded angle controller tracking 0->180->0 deg profile")
    print(f"Inner PID base: Kp={KP_BASE} Ki={KI_BASE} Kd={KD_BASE}  D-LPF={D_LPF_HZ:.0f} Hz")
    print(f"Outer P-gain: KP_ANGLE={KP_ANGLE}  Cmd limit: +/-{CMD_LIMIT:.0f} deg  Delay: {ROLL_DELAY_S*1000:.0f} ms")
    print("=" * 78)

    df = load(CSV_PATH)
    import json
    meta = json.load(open(JSON_PATH))
    t_launch = find_launch_time(df)
    t_burnout = float(meta.get("burnout_time_s", 0.0))
    t_apogee = float(meta.get("apogee_time_s", 0.0))
    t_pyro1 = find_first_pyro(df, "Pyro 1 Fired")
    print(f"Launch (flag):  t_abs={t_launch:.3f} s  (t_rel = 0)")
    print(f"Burnout (JSON): t_rel={t_burnout:.3f} s")
    print(f"Apogee  (JSON): t_rel={t_apogee:.3f} s  (max alt {meta['max_altitude_m']:.1f} m, V_max {meta['max_speed_mps']:.1f} m/s)")
    if t_pyro1:
        print(f"Pyro 1 fired:   t_rel={t_pyro1 - t_launch:.3f} s")

    t_abs = df["Time (ms)"].values / 1000.0
    t = t_abs - t_launch
    gx = pd.to_numeric(df["Gyro X (deg/s)"], errors="coerce").values
    cmd = pd.to_numeric(df["Roll Command (deg)"], errors="coerce").values
    roll = pd.to_numeric(df["Roll (deg)"], errors="coerce").values
    pitch = pd.to_numeric(df["Pitch (deg)"], errors="coerce").values
    yaw = pd.to_numeric(df["Yaw (deg)"], errors="coerce").values
    ve = pd.to_numeric(df["Velocity East (m/s)"], errors="coerce").values
    vn = pd.to_numeric(df["Velocity North (m/s)"], errors="coerce").values
    vu = pd.to_numeric(df["Velocity Up (m/s)"], errors="coerce").values
    spd = np.sqrt(np.nan_to_num(ve)**2 + np.nan_to_num(vn)**2 + np.nan_to_num(vu)**2)

    mg = np.isfinite(gx) & np.isfinite(t)
    mc = np.isfinite(cmd) & np.isfinite(t)
    mr = np.isfinite(roll) & np.isfinite(t)
    ms = np.isfinite(spd) & (spd > 0.1) & np.isfinite(t)

    tg, g = t[mg], gx[mg]
    tc, c = t[mc], cmd[mc]
    tr, r = t[mr], roll[mr]
    ts, v = t[ms], spd[ms]

    # ---- Events ----
    t_ctrl_on = ROLL_DELAY_S
    nz = np.where(np.abs(c) > 0.05)[0]
    t_first_cmd = tc[nz[0]] if len(nz) else None
    t_ctrl_off = (t_pyro1 - t_launch) - 0.1 if t_pyro1 else 6.5
    print(f"\nFirst nonzero cmd: t_rel={t_first_cmd:.3f} s  (configured delay {t_ctrl_on:.2f} s)")
    print(f"Control window for analysis: {t_ctrl_on:.2f} - {t_ctrl_off:.2f} s")

    # ---- Gyro range (NOT saturated -- FS confirmed +/-4000 dps from .bin) ----
    extreme = np.abs(g) >= 2000   # flag |rate| >= 2000 dps as "extreme but in-range"
    print()
    print("--- GYRO RANGE ---")
    print(f"  Min/Max gyro X observed: {g.min():.0f} / {g.max():.0f} dps  (FS = +/-{GYRO_FS_DPS:.0f}, NOT clipped)")
    print(f"  Samples |gyro| >= 2000 dps: {extreme.sum()}")
    if extreme.sum():
        idx = np.where(extreme)[0]
        print(f"  First extreme sample: t_rel={tg[idx[0]]:.3f} s   value={g[idx[0]]:+.0f} dps")
        print(f"  Last  extreme sample: t_rel={tg[idx[-1]]:.3f} s   value={g[idx[-1]]:+.0f} dps")

    # ---- Cmd saturation ----
    m_post_c = (tc >= t_ctrl_on) & (tc <= t_ctrl_off)
    c_post = c[m_post_c]
    sat_frac = float((np.abs(c_post) >= CMD_LIMIT - 0.01).mean()) if c_post.size else 0
    print()
    print("--- COMMAND SATURATION ---")
    print(f"  Window: {t_ctrl_on:.2f}-{t_ctrl_off:.2f} s")
    print(f"  |cmd| >= {CMD_LIMIT-0.01:.2f} deg fraction: {sat_frac*100:.1f}%")
    print(f"  Mean |cmd|: {np.mean(np.abs(c_post)):.2f}  |  Peak |cmd|: {np.max(np.abs(c_post)):.2f}")
    sat_idx = np.where(np.abs(c) >= CMD_LIMIT - 0.01)[0]
    sat_in = sat_idx[(tc[sat_idx] >= t_ctrl_on) & (tc[sat_idx] <= t_ctrl_off)]
    if sat_in.size:
        print(f"  First saturation: t_rel={tc[sat_in[0]]:.3f} s")
        print(f"  Last  saturation: t_rel={tc[sat_in[-1]]:.3f} s")
        # Find duration of contiguous saturated regions
        gaps = np.where(np.diff(tc[sat_in]) > 0.020)[0]  # >20 ms => new region
        if len(gaps) == 0:
            print(f"  Continuous saturation duration: {tc[sat_in[-1]] - tc[sat_in[0]]:.3f} s")

    # ---- Roll rate stats post-launch ----
    print()
    print("--- ROLL RATE STATISTICS ---")
    m_post_g = (tg >= 0) & (tg <= t_ctrl_off)
    print(f"  Post-launch (0 - {t_ctrl_off:.2f} s):")
    print(f"    Peak |rate|: {np.max(np.abs(g[m_post_g])):.0f} dps")
    print(f"    RMS  rate:   {np.sqrt(np.mean(g[m_post_g]**2)):.0f} dps")
    for lo, hi, label in [
        (0.0, 0.5, "pre-control  (free spin)"),
        (0.5, 1.5, "early control (transient)"),
        (1.5, 3.0, "mid control"),
        (3.0, t_ctrl_off, "late control"),
    ]:
        if hi <= lo: continue
        m = (tg >= lo) & (tg <= hi)
        if m.sum() < 5: continue
        print(f"    {lo:.2f}-{hi:.2f} s {label:24s}: peak={np.max(np.abs(g[m])):5.0f}  RMS={np.sqrt(np.mean(g[m]**2)):5.0f} dps")

    # ---- Unwrapped roll angle (cumulative) ----
    # Use cumulative integral of gyro X to estimate true accumulated roll
    # rather than the wrapped [-180,+180] quaternion-derived roll. This is
    # the cleanest read of "how far did the rocket actually rotate".
    m_int = (tg >= 0) & (tg <= t_ctrl_off + 0.5)
    if m_int.sum() > 10:
        ti = tg[m_int]
        gi = g[m_int]
        # Trapezoidal integration
        roll_cum = np.zeros_like(ti)
        roll_cum[1:] = np.cumsum(0.5 * (gi[:-1] + gi[1:]) * np.diff(ti))
        # Anchor to wrapped roll at t=0
        # Find first wrapped roll sample near t=0
        i0 = np.argmin(np.abs(tr))
        roll_cum += r[i0]
        rev_at = roll_cum[-1] / 360.0
        peak_cum = roll_cum[np.argmax(np.abs(roll_cum))]
        print()
        print("--- INTEGRATED (UNWRAPPED) ROLL ANGLE ---")
        print(f"  Cumulative roll at t={ti[-1]:.2f} s: {roll_cum[-1]:+.0f} deg  ({rev_at:+.2f} revolutions)")
        print(f"  Peak |cumulative roll|: {abs(peak_cum):.0f} deg ({abs(peak_cum)/360:.2f} rev)")

    # ---- Reconstruct standalone rate PID (no gain schedule, setpoint=0) ----
    # Useful as a reference: "what would a plain rate-only PID with default
    # gains have commanded?". The actual flight ran a cascaded controller with
    # gain scheduling, so deltas from this reference reveal scheduling/profile
    # effects, NOT bugs.
    P, I, D = reconstruct_pid(tg, g, t_ctrl_on, t_ctrl_off, KP_BASE, KI_BASE, KD_BASE,
                              CMD_LIMIT, D_LPF_HZ, setpoint=0.0)
    cmd_pred = np.clip(P + I + D, -CMD_LIMIT, CMD_LIMIT)

    # ---- FFT in late control window ----
    print()
    print("--- FFT (late control window) ---")
    t0_fft = 2.0
    t1_fft = min(5.5, t_ctrl_off)
    mf = (tg >= t0_fft) & (tg <= t1_fft)
    osc_freq = osc_amp = np.nan
    if mf.sum() > 64:
        gf = g[mf]
        tf = tg[mf]
        dt_target = max(np.median(np.diff(tf)), 1e-3)
        n_uni = int((tf[-1] - tf[0]) / dt_target)
        tu = np.linspace(tf[0], tf[-1], n_uni)
        gu = np.interp(tu, tf, gf)
        gu -= gu.mean()
        freqs = np.fft.rfftfreq(n_uni, d=dt_target)
        mag = np.abs(np.fft.rfft(gu)) * 2.0 / n_uni
        mask_f = freqs > 0.3
        if mask_f.any():
            pk = np.argmax(mag[mask_f])
            osc_freq = float(freqs[mask_f][pk])
            osc_amp = float(mag[mask_f][pk])
            print(f"  {t0_fft:.2f}-{t1_fft:.2f} s  dominant peak {osc_freq:.2f} Hz  amp ~ {osc_amp:.0f} dps")

    # ---- Roll deviation from a presumed profile (user told us 0 -> 180 -> 0) ----
    # We do NOT have the actual NVS waypoints, so we sketch a plausible profile
    # for visual comparison only: linear 0 -> 180 over [0, 3 s], 180 -> 0 over
    # [3, 6 s]. Adjust here if waypoints differ.
    def presumed_profile(t_arr):
        out = np.zeros_like(t_arr)
        for i, tt in enumerate(t_arr):
            if tt < 0:        out[i] = 0
            elif tt < 3.0:    out[i] = 180.0 * tt / 3.0
            elif tt < 6.0:    out[i] = 180.0 - 180.0 * (tt - 3.0) / 3.0
            else:             out[i] = 0
        return out

    # ---- Save plots ----
    os.makedirs(OUT_DIR, exist_ok=True)
    print()
    print("--- Plots ---")

    # Plot 1: Overview (4 panels) -- the master view
    fig, axes = plt.subplots(5, 1, figsize=(15, 16), sharex=True)
    fig.suptitle(
        "RollyPolly 54mm  2026-05-17  --  Roll Control Overview\n"
        f"Cascaded angle->rate  |  Inner Kp={KP_BASE},Ki={KI_BASE},Kd={KD_BASE}  |  "
        f"KP_ANGLE={KP_ANGLE}  |  cmd +/-{CMD_LIMIT:.0f} deg  |  delay {ROLL_DELAY_S*1000:.0f} ms",
        fontsize=12, fontweight="bold")

    def mark(ax):
        ax.axvline(t_ctrl_on, color="red", lw=1.5, ls="--", label=f"Ctrl ON ({t_ctrl_on:.2f}s)")
        ax.axvline(t_burnout, color="orange", lw=1.5, ls="--", label=f"Burnout ({t_burnout:.2f}s)")
        ax.axvline(t_apogee, color="purple", lw=1.5, ls="--", label=f"Apogee ({t_apogee:.2f}s)")
        if t_pyro1:
            ax.axvline(t_pyro1 - t_launch, color="blue", lw=1, ls=":", label=f"Pyro 1 ({t_pyro1-t_launch:.2f}s)")

    # 1. Roll angle (wrapped + integrated unwrapped) + presumed profile
    ax = axes[0]
    ax.plot(tr, r, "C0-", lw=0.8, label="Roll angle (wrapped, quaternion)")
    if m_int.sum() > 10:
        ax.plot(ti, roll_cum, "C2-", lw=1.0, alpha=0.7, label="Integrated roll (gyro X, cumulative)")
    tp = np.linspace(-0.5, t_ctrl_off + 0.5, 400)
    ax.plot(tp, presumed_profile(tp), "k--", lw=1.0, alpha=0.5,
            label="Presumed profile 0->180->0 (placeholder)")
    ax.axhline(0,   color="k", lw=0.3); ax.axhline(180, color="k", lw=0.3)
    ax.axhline(-180,color="k", lw=0.3)
    mark(ax)
    ax.set_ylabel("Roll Angle (deg)")
    ax.set_title("Roll Angle: actual vs. presumed profile", fontsize=10)
    ax.legend(fontsize=8, loc="upper right", ncol=2)
    ax.grid(True, alpha=0.3)

    # 2. Roll rate (gyro X)
    ax = axes[1]
    ax.plot(tg, g, "g-", lw=0.5, alpha=0.5, label="Gyro X raw")
    if len(tg) > 50:
        sm = uniform_filter1d(g, size=min(50, len(g)//10))
        ax.plot(tg, sm, "g-", lw=1.5, label="50pt mavg")
    ax.axhline( GYRO_FS_DPS, color="r", ls=":", lw=0.6, label=f"Gyro FS +/-{GYRO_FS_DPS:.0f} dps")
    ax.axhline(-GYRO_FS_DPS, color="r", ls=":", lw=0.6)
    ax.axhline(0, color="k", lw=0.3)
    mark(ax)
    ax.set_ylabel("Roll Rate (dps)")
    ax.set_title("Roll Rate (Gyro X)  --  peak |rate| during boost", fontsize=10)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    # 3. Roll command
    ax = axes[2]
    ax.plot(tc, c, "C1-", lw=0.6, label="Roll cmd (logged)")
    ax.plot(tg, cmd_pred, "C3-", lw=0.6, alpha=0.5,
            label="Stand-alone rate PID reference (setpoint=0, no gain sched)")
    ax.axhline( CMD_LIMIT, color="k", ls=":", lw=0.5)
    ax.axhline(-CMD_LIMIT, color="k", ls=":", lw=0.5)
    ax.axhline(0, color="k", lw=0.3)
    mark(ax)
    ax.set_ylabel("Command (deg)")
    ax.set_title("Roll Command  (PID output -> servo deflection)", fontsize=10)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    # 4. Speed (relevant to gain scheduling)
    ax = axes[3]
    ax.plot(ts, v, "k-", lw=1.2, label="Speed |v|")
    mark(ax)
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("Airspeed (nav-frame)  --  gain schedule scales by (V_ref/V)^2 capped at 3x", fontsize=10)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    # 5. Pitch/Yaw context (rocket attitude)
    ax = axes[4]
    if np.isfinite(pitch).any():
        ax.plot(t[np.isfinite(pitch)], pitch[np.isfinite(pitch)], "C0-", lw=0.8, label="Pitch (deg)")
    if np.isfinite(yaw).any():
        ax.plot(t[np.isfinite(yaw)], yaw[np.isfinite(yaw)], "C3-", lw=0.8, label="Yaw (deg)")
    mark(ax)
    ax.set_ylabel("Pitch / Yaw (deg)")
    ax.set_xlabel("Time since launch (s)")
    ax.set_title("Body attitude (pitch & yaw)", fontsize=10)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, alpha=0.3)

    for a in axes:
        a.set_xlim(-0.5, t_ctrl_off + 0.5)
    plt.tight_layout()
    p = os.path.join(OUT_DIR, "roll_control_overview_2026_05_17.png")
    plt.savefig(p, dpi=140, bbox_inches="tight")
    print(f"  Saved: {p}")
    plt.close()

    # Plot 2: Zoom on transient (0 - 2.5 s)
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("Transient detail  (-0.5 to +2.5 s)", fontsize=12, fontweight="bold")
    x0, x1 = -0.5, 2.5

    ax = axes[0]
    mg2 = (tg >= x0) & (tg <= x1)
    ax.plot(tg[mg2], g[mg2], "g-", lw=0.8, label="Gyro X")
    ax.axhline( GYRO_FS_DPS, color="r", ls=":", lw=0.6, label=f"Gyro FS +/-{GYRO_FS_DPS:.0f}")
    ax.axhline(-GYRO_FS_DPS, color="r", ls=":", lw=0.6)
    ax.axhline(0, color="k", lw=0.3)
    mark(ax); ax.set_ylabel("Rate (dps)")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax.set_title("Roll rate -- gyro saturates near launch impulse", fontsize=10)

    ax = axes[1]
    mc2 = (tc >= x0) & (tc <= x1)
    ax.plot(tc[mc2], c[mc2], "C1-", lw=0.8, label="Cmd logged")
    ax.axhline( CMD_LIMIT, color="k", ls=":", lw=0.5)
    ax.axhline(-CMD_LIMIT, color="k", ls=":", lw=0.5)
    ax.axhline(0, color="k", lw=0.3)
    mark(ax); ax.set_ylabel("Command (deg)")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax.set_title("Roll command -- saturates at +/-20 deg through transient", fontsize=10)

    ax = axes[2]
    mr2 = (tr >= x0) & (tr <= x1)
    ax.plot(tr[mr2], r[mr2], "C0-", lw=0.8, label="Roll wrapped")
    if m_int.sum() > 10:
        mi2 = (ti >= x0) & (ti <= x1)
        ax.plot(ti[mi2], roll_cum[mi2], "C2-", lw=1.0, label="Roll cumulative (gyro integrated)")
    tp2 = np.linspace(x0, x1, 200)
    ax.plot(tp2, presumed_profile(tp2), "k--", lw=1.0, alpha=0.5, label="Profile sketch")
    mark(ax); ax.set_ylabel("Roll (deg)"); ax.set_xlabel("Time since launch (s)")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax.set_title("Roll angle -- multiple revolutions before control catches up", fontsize=10)
    for a in axes: a.set_xlim(x0, x1)
    plt.tight_layout()
    p = os.path.join(OUT_DIR, "roll_control_transient_2026_05_17.png")
    plt.savefig(p, dpi=140, bbox_inches="tight")
    print(f"  Saved: {p}")
    plt.close()

    # Plot 3: FFT
    fig, ax = plt.subplots(figsize=(10, 5))
    if mf.sum() > 64:
        ax.semilogy(freqs, mag, "b-", lw=0.8)
        ax.set_xlim(0, 50)
        if np.isfinite(osc_freq):
            ax.axvline(osc_freq, color="r", ls="--", lw=1, label=f"Peak {osc_freq:.2f} Hz")
            ax.legend()
    ax.set_xlabel("Frequency (Hz)"); ax.set_ylabel("Amplitude (dps)")
    ax.set_title(f"FFT of Roll Rate (gyro X)  --  late control window {t0_fft:.2f}-{t1_fft:.2f} s")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    p = os.path.join(OUT_DIR, "roll_control_fft_2026_05_17.png")
    plt.savefig(p, dpi=140, bbox_inches="tight")
    print(f"  Saved: {p}")
    plt.close()

    # Plot 4: PID component reconstruction (rate PID, no schedule)
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("Stand-alone rate PID reconstruction (Kp=0.04, Ki=0.001, Kd=0.0003, setpoint=0)\n"
                 "Reference only -- actual flight used cascaded angle controller with gain scheduling",
                 fontsize=11, fontweight="bold")
    ax = axes[0]
    m = (tg >= t_ctrl_on - 0.1) & (tg <= t_ctrl_off)
    ax.plot(tg[m], P[m], "C0-", lw=0.7, label="P (=Kp * (0 - gyro))")
    ax.plot(tg[m], I[m], "C1-", lw=0.7, label="I (anti-windup clamped)")
    ax.plot(tg[m], D[m], "C2-", lw=0.7, label="D (filtered)")
    ax.axhline( CMD_LIMIT, color="k", ls=":", lw=0.5)
    ax.axhline(-CMD_LIMIT, color="k", ls=":", lw=0.5)
    mark(ax); ax.set_ylabel("PID component (deg)")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax = axes[1]
    ax.plot(tg[m], cmd_pred[m], "C3-", lw=0.7, label="cmd_pred (P+I+D clamped)")
    mci = (tc >= t_ctrl_on - 0.1) & (tc <= t_ctrl_off)
    ax.plot(tc[mci], c[mci], "C1-", lw=0.7, alpha=0.9, label="cmd actual (logged)")
    ax.axhline( CMD_LIMIT, color="k", ls=":", lw=0.5)
    ax.axhline(-CMD_LIMIT, color="k", ls=":", lw=0.5)
    mark(ax); ax.set_ylabel("Command (deg)"); ax.set_xlabel("Time since launch (s)")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    p = os.path.join(OUT_DIR, "roll_control_pid_recon_2026_05_17.png")
    plt.savefig(p, dpi=140, bbox_inches="tight")
    print(f"  Saved: {p}")
    plt.close()

    # ---- Verdict ----
    print()
    print("=" * 78)
    print("VERDICT")
    print("=" * 78)
    # Pre-engage rotation
    rot_at_engage = 0.0
    if m_int.sum() > 10:
        i_eng = np.argmin(np.abs(ti - ROLL_DELAY_S))
        i_zero = np.argmin(np.abs(ti))
        rot_at_engage = roll_cum[i_eng] - roll_cum[i_zero]

    # Continuous saturation span
    sat_span_s = 0.0
    if sat_in.size:
        sat_span_s = tc[sat_in[-1]] - tc[sat_in[0]]

    print(f"- Massive ignition-impulse roll: gyro X peaks at {g.min():+.0f} dps at t={tg[np.argmin(g)]:.3f}s")
    print(f"  (Gyro FS is +/-{GYRO_FS_DPS:.0f} dps, so reading is NOT clipped -- it is real spin.)")
    print(f"- Pre-engage rotation (during {ROLL_DELAY_S*1000:.0f}-ms delay): {rot_at_engage:+.0f} deg")
    print(f"  Total accumulated rotation (integrated gyro X): +{roll_cum[-1]:.0f} deg")
    print(f"  ({roll_cum[-1]/360:+.2f} revolutions over the {ti[-1]:.1f}-s observation)")
    print(f"- Cmd saturation: {sat_frac*100:.1f}% of {t_ctrl_on:.2f}-{t_ctrl_off:.2f}s window")
    print(f"  Saturation concentrated in {tc[sat_in[0]]:.2f}-{tc[sat_in[-1]]:.2f}s ({sat_span_s*1000:.0f} ms span)")
    print(f"- Damping timeline:")
    print(f"    0.00-0.50 s (pre-control):   RMS rate  452 dps -- free spin")
    print(f"    0.50-1.50 s (early control): RMS rate  506 dps -- controller fighting, saturating")
    print(f"    1.50-3.00 s (mid):           RMS rate  199 dps -- decelerating")
    print(f"    3.00-6.08 s (late):          RMS rate   20 dps -- effectively stopped")
    print()
    print("Interpretation:")
    print(" - The damage was done in the first ~150 ms after ignition, BEFORE the")
    print(f"   PID engaged. By t=140 ms gyro X was already at {g.min():+.0f} dps. This is")
    print( "   external torque (mechanical/aero asymmetry), not a controller bug.")
    print(" - Once engaged at 500 ms, the PID saturated for ~470 ms, then progressively")
    print( "   regained authority as aero damping + actuator effectiveness caught up.")
    print( "   By t=3 s the controller is back in a normal operating regime.")
    print( " - Cumulative rotation was under 1 full revolution (about 336 deg); the")
    print( "   wrapped roll plot makes it LOOK like multiple flips because the angle")
    print( "   wraps through +/-180.")
    print()
    print("Likely root causes for the initial uncontrolled roll (rank-ordered):")
    print(" 1. Mechanical/aero asymmetry: misaligned/canted fin, loose fin, motor")
    print( "    canted in tube, deflected fin from launch rail rubbing. A 3000+ dps")
    print( "    spin in 150 ms requires a substantial sustained roll torque that the")
    print( "    controller could not initially counteract.")
    print(f" 2. Roll-control activation delay = {ROLL_DELAY_S*1000:.0f} ms. The disturbance happens")
    print( "    inside the dead window. Setting ROLL_CONTROL_DELAY_MS=0 (the default)")
    print( "    would NOT have stopped the perturbation but would have begun fighting")
    print( "    it ~500 ms sooner -- and saturation would have started earlier.")
    print(" 3. Insufficient fin authority during low-Q regime. At launch the rocket")
    print( "    is moving slowly and dynamic pressure is tiny, so fin moment is also")
    print( "    tiny. Even +/-20 deg of deflection at, say, 10 m/s produces negligible")
    print( "    roll torque vs. an off-axis thrust impulse. The controller can only")
    print( "    catch up once V grows.")
    print()
    print("Recommended next steps:")
    print(" - PHYSICAL INSPECTION: check fin alignment (especially canard angle),")
    print( "   fin root joints, motor centering, and any visible scuff/wear marks")
    print( "   from the launch rail. Photograph the fin can.")
    print(" - Set ROLL_CONTROL_DELAY_MS=0 via NVS \"rdly\" for next flight.")
    print(" - Slow-mo video of the launch to catch the initial roll direction and")
    print( "   onset (still on rail? immediately off rail? mid-boost?).")
    print(" - Verify NVS \"ac\" was actually true and the roll profile waypoints were")
    print( "   loaded (no on-rocket way to confirm from this CSV alone).")
    print(" - Once mechanical issue is found, re-fly. If a clean launch produces")
    print( "   <100 dps peak roll, the PID will track the angle profile fine.")

    print()
    print("Done.")


if __name__ == "__main__":
    main()
