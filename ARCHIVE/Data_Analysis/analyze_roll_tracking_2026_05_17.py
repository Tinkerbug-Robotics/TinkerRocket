#!/usr/bin/env python3
"""
Profile-tracking analysis for the 2026-05-17 RollyPolly_54mm flight.

Configuration (user-provided after-action):
  Mode:      cascaded angle->rate ("track profile")
  Inner PID: Kp=0.02 Ki=0.0005 Kd=0.0003   (HALF the config.h default of 0.04)
  Outer P:   KP_ANGLE = 4.0 deg/s per deg
  Roll delay: 500 ms
  Cmd limit: +/-20 deg
  Profile waypoints (time_s, angle_deg):
      WP1: (0.5, 0)
      WP2: (1.0, 180)
      WP3: (2.0, 0)
  Profile semantics (from firmware): before WP1 hold WP1; after WPn hold WPn;
  linear interpolation between consecutive waypoints.

Shows: 0 - 3 s window of actual roll angle vs. target profile, actual roll
rate vs. the rate the outer loop was asking for, and roll command (actual
vs. reconstructed cascaded controller output).
"""
import os
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

CSV = "/Users/christianpedersen/Documents/Hobbies/ModelRockets/TestFlights/2026_05_17/RollyPolly_54mm/flight_20260517_133456.csv"
OUT_DIR = "/Users/christianpedersen/Documents/Hobbies/ModelRockets/TestFlights/2026_05_17/RollyPolly_54mm/analysis"

# Flight settings (user-confirmed)
KP_INNER = 0.02
KI_INNER = 0.0005
KD_INNER = 0.0003
D_LPF_HZ = 10.0
KP_ANGLE = 4.0   # deg/s per deg
CMD_LIM  = 20.0
ROLL_DELAY_S = 0.5
WAYPOINTS = [(0.5, 0.0), (1.0, 180.0), (2.0, 0.0)]

# Plot interactively? (When run from CLI shows a native macOS window;
# also always saves a static PNG.)
INTERACTIVE = True


def profile_target_deg(t_s):
    """Linear interpolation between waypoints; clamp before/after."""
    wp = WAYPOINTS
    if t_s <= wp[0][0]:
        return wp[0][1]
    if t_s >= wp[-1][0]:
        return wp[-1][1]
    for i in range(len(wp) - 1):
        t0, a0 = wp[i]
        t1, a1 = wp[i + 1]
        if t_s < t1:
            frac = (t_s - t0) / (t1 - t0)
            return a0 + frac * (a1 - a0)
    return wp[-1][1]


def profile_target_rate(t_s):
    """Slope of the linear segment containing t_s (deg/s)."""
    wp = WAYPOINTS
    if t_s <= wp[0][0] or t_s >= wp[-1][0]:
        return 0.0
    for i in range(len(wp) - 1):
        t0, a0 = wp[i]
        t1, a1 = wp[i + 1]
        if t_s < t1:
            return (a1 - a0) / (t1 - t0)
    return 0.0


def wrap_pm180(x):
    return ((x + 180) % 360) - 180


def reconstruct_cascaded(tg, gx, roll_wrapped, t_on, kp_inner, ki_inner, kd_inner,
                         kp_angle, cmd_lim, d_lpf_hz):
    """Re-run the cascaded controller offline (no gain scheduling)."""
    tau_d = 1.0 / (2 * np.pi * d_lpf_hz)
    cum = 0.0
    d_filt = 0.0
    err_inner_prev = 0.0
    cmd = np.zeros_like(gx)
    rate_cmd_log = np.zeros_like(gx)
    angle_err_log = np.zeros_like(gx)
    pid_sp_log = np.zeros_like(gx)
    first = True
    dts = np.diff(tg, prepend=tg[0])
    for i in range(len(tg)):
        if tg[i] < t_on:
            cum = 0.0
            d_filt = 0.0
            err_inner_prev = 0.0
            first = True
            continue
        dt = dts[i] if i > 0 else 0.001
        # Outer loop
        target = profile_target_deg(tg[i])
        actual = roll_wrapped[i]
        ang_err = wrap_pm180(target - actual)
        rate_cmd = kp_angle * ang_err  # deg/s
        pid_setpoint = -rate_cmd       # firmware sign convention
        # Inner loop -- it receives -gyro_x as the "rate" measurement
        rate_meas = -gx[i]
        err_inner = pid_setpoint - rate_meas
        cum += err_inner * dt
        if first:
            d_raw = 0.0
            first = False
        else:
            d_raw = (err_inner - err_inner_prev) / max(dt, 1e-6)
        alpha = dt / (dt + tau_d)
        d_filt = d_filt + alpha * (d_raw - d_filt)
        P = kp_inner * err_inner
        I = np.clip(ki_inner * cum, -cmd_lim, cmd_lim)
        D = kd_inner * d_filt
        cmd[i] = np.clip(P + I + D, -cmd_lim, cmd_lim)
        err_inner_prev = err_inner
        rate_cmd_log[i] = rate_cmd
        angle_err_log[i] = ang_err
        pid_sp_log[i] = pid_setpoint
    return cmd, rate_cmd_log, angle_err_log, pid_sp_log


def main():
    print("=" * 78)
    print("PROFILE TRACKING ANALYSIS  --  RollyPolly_54mm 2026-05-17")
    print(f"Inner PID: Kp={KP_INNER}, Ki={KI_INNER}, Kd={KD_INNER}, D-LPF={D_LPF_HZ:.0f}Hz")
    print(f"Outer P:   KP_ANGLE={KP_ANGLE}  Cmd: +/-{CMD_LIM} deg  Delay: {ROLL_DELAY_S*1000:.0f}ms")
    print(f"Profile:   {WAYPOINTS}")
    print("=" * 78)

    df = pd.read_csv(CSV)
    df.columns = df.columns.str.strip()
    t = df["Time (ms)"].values / 1000.0
    lf = pd.to_numeric(df["Launch Flag"], errors="coerce").fillna(0).astype(int).values
    t_launch = t[np.where(lf == 1)[0][0]]
    t_rel = t - t_launch
    gx = pd.to_numeric(df["Gyro X (deg/s)"], errors="coerce").values
    rc = pd.to_numeric(df["Roll Command (deg)"], errors="coerce").values
    roll = pd.to_numeric(df["Roll (deg)"], errors="coerce").values

    m = np.isfinite(gx) & np.isfinite(rc) & np.isfinite(roll)
    tg = t_rel[m]
    g = gx[m]
    c = rc[m]
    r = roll[m]

    # Build target/profile arrays
    target = np.array([profile_target_deg(tt) for tt in tg])
    targ_rate = np.array([profile_target_rate(tt) for tt in tg])
    ang_err = wrap_pm180(target - r)

    # Reconstruct what the cascaded controller SHOULD have commanded
    cmd_recon, rate_cmd_outer, ang_err_log, pid_sp_log = reconstruct_cascaded(
        tg, g, r, ROLL_DELAY_S, KP_INNER, KI_INNER, KD_INNER, KP_ANGLE,
        CMD_LIM, D_LPF_HZ)

    # Print a snapshot table
    print()
    print("Snapshot (actual rocket vs profile, plus reconstructed cmd):")
    print("  t(s) | target | actual | err  | tgt_rate | gyro_x | cmd_act | cmd_recon")
    print("-" * 79)
    snap_times = [0.0, 0.25, 0.5, 0.6, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0]
    for tt in snap_times:
        i = int(np.argmin(np.abs(tg - tt)))
        print(f"  {tg[i]:+5.3f} |{target[i]:+7.1f} |{r[i]:+7.1f} |{ang_err[i]:+5.0f} | "
              f"{targ_rate[i]:+8.1f} | {g[i]:+7.0f} | {c[i]:+7.2f} | {cmd_recon[i]:+7.2f}")

    # Stats over 0.5 - 2.5 s (active profile window)
    mw = (tg >= ROLL_DELAY_S) & (tg <= 2.5)
    rms_err = float(np.sqrt(np.mean(ang_err[mw] ** 2))) if mw.any() else float("nan")
    max_err = float(np.max(np.abs(ang_err[mw])))
    print()
    print(f"Profile window {ROLL_DELAY_S:.2f}-2.5s:")
    print(f"  Angle tracking error: RMS {rms_err:5.1f} deg,  max {max_err:5.1f} deg")
    print(f"  Mean |target|={np.mean(np.abs(target[mw])):.1f} deg, mean |actual|={np.mean(np.abs(r[mw])):.1f} deg")

    # ---- Plots ----
    os.makedirs(OUT_DIR, exist_ok=True)

    if INTERACTIVE:
        try:
            matplotlib.use("MacOSX")
        except Exception:
            pass
    else:
        matplotlib.use("Agg")

    fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
    fig.suptitle("Profile tracking 0-3 s  --  Kp=0.02 Ki=0.0005 Kd=0.0003  "
                 "Profile: 0->180@1s ->0@2s (engage 0.5s)",
                 fontsize=12, fontweight="bold")

    # Panel 1: angle -- actual (wrapped) vs target profile
    ax = axes[0]
    ax.plot(tg, target, "r--", lw=1.6, label="Target (profile)")
    ax.plot(tg, r, "C0-", lw=1.0, label="Actual roll (wrapped)")
    # Mark waypoints
    for (tw, aw) in WAYPOINTS:
        ax.plot(tw, aw, "ro", ms=8, mfc="r", mec="k")
        ax.annotate(f"WP @ {tw:.1f}s, {aw:.0f}°", (tw, aw),
                    xytext=(6, 6), textcoords="offset points", fontsize=8)
    ax.axvline(ROLL_DELAY_S, color="red", ls="--", lw=1.0, label=f"Ctrl ON ({ROLL_DELAY_S}s)")
    ax.axhline(  180, color="k", lw=0.3)
    ax.axhline(    0, color="k", lw=0.3)
    ax.axhline( -180, color="k", lw=0.3)
    ax.set_ylabel("Roll angle (deg)")
    ax.set_ylim(-200, 220)
    ax.legend(fontsize=9, loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Roll ANGLE: actual vs. target profile", fontsize=10)

    # Panel 2: rate -- actual gyro vs target rate from outer loop
    ax = axes[1]
    ax.plot(tg, targ_rate, "r--", lw=1.6, label="Target rate (profile slope)")
    ax.plot(tg, g, "g-", lw=0.9, label="Actual roll rate (Gyro X)")
    ax.axhline(0, color="k", lw=0.3)
    ax.axvline(ROLL_DELAY_S, color="red", ls="--", lw=1.0)
    ax.set_ylabel("Roll rate (dps)")
    ax.legend(fontsize=9, loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Roll RATE: actual vs profile slope (= what outer loop wants)", fontsize=10)

    # Panel 3: command -- actual vs reconstructed cascaded controller
    ax = axes[2]
    ax.plot(tg, c, "C1-", lw=1.0, label="Cmd actual (logged)")
    ax.plot(tg, cmd_recon, "C3--", lw=1.0, alpha=0.7,
            label="Cmd reconstructed cascaded (no gain sched)")
    ax.axhline( CMD_LIM, color="k", ls=":", lw=0.5)
    ax.axhline(-CMD_LIM, color="k", ls=":", lw=0.5)
    ax.axhline(0, color="k", lw=0.3)
    ax.axvline(ROLL_DELAY_S, color="red", ls="--", lw=1.0)
    ax.set_ylabel("Roll cmd (deg)")
    ax.set_xlabel("Time since launch (s)")
    ax.legend(fontsize=9, loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Roll COMMAND: actual vs reconstructed cascaded PID", fontsize=10)

    for ax in axes:
        ax.set_xlim(-0.3, 3.0)

    plt.tight_layout()
    p = os.path.join(OUT_DIR, "roll_profile_tracking_0_3s_2026_05_17.png")
    plt.savefig(p, dpi=140, bbox_inches="tight")
    print()
    print(f"Saved static PNG: {p}")
    if INTERACTIVE:
        print("Opening interactive window...")
        plt.show()
    plt.close()


if __name__ == "__main__":
    main()
