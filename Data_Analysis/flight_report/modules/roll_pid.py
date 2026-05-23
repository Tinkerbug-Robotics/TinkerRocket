"""Roll-PID tracking & tuning module.

Driven from the binary log (not the legacy CSV input). Estimates the plant
gain K_plant, FFT-identifies any persistent oscillation, computes phase
margin given the current Kp, and recommends tuned gains for a target
crossover frequency.

Skips gracefully on flights without active roll control (`roll_cmd ≡ 0`).
Adapted from `analyze_roll_pid.py`.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

try:
    from scipy.ndimage import uniform_filter1d
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

from ..flight import Flight
from ..registry import AnalysisResult

# Default servo spec (PTK 7308 @ 8.2V) — overridden by sidecar when present.
_DEFAULT_SERVO_SLEW_DPS = 60.0 / 0.065      # ~923 deg/s
_DEFAULT_SERVO_TAU_S   = 0.065 / 2.0        # ~32 ms first-order time constant
_DEFAULT_COMPUTER_DELAY_S = 0.001           # 1 ms loop delay (Mini OC)
_K_PLANT_FALLBACK = 200.0                   # deg/s² per deg deflection
_F_TARGET_HZ = 2.0                          # target closed-loop crossover
_PM_TARGET_DEG = 50.0                       # target phase margin


def _moving_average(arr: np.ndarray, window: int) -> np.ndarray:
    if _HAS_SCIPY:
        return uniform_filter1d(arr, size=window)
    # numpy fallback — same-length convolution
    w = max(1, int(window))
    if w < 2:
        return arr.copy()
    pad = w // 2
    padded = np.pad(arr, (pad, w - pad - 1), mode="edge")
    return np.convolve(padded, np.ones(w) / w, mode="valid")


def _gain_from_sidecar(sidecar: dict) -> tuple[float | None, float | None, float | None]:
    """Best-effort extract of (Kp, Ki, Kd) from the per-flight .json sidecar."""
    # The post-#178 RocketProfile dump nests these — accept a few common shapes.
    for keyset in (("roll_kp", "roll_ki", "roll_kd"),
                   ("Kp", "Ki", "Kd")):
        if all(k in sidecar for k in keyset):
            return tuple(float(sidecar[k]) for k in keyset)  # type: ignore[return-value]
    profile = sidecar.get("roll_pid") or sidecar.get("guidance") or {}
    if isinstance(profile, dict):
        for keyset in (("kp", "ki", "kd"), ("Kp", "Ki", "Kd")):
            if all(k in profile for k in keyset):
                return tuple(float(profile[k]) for k in keyset)  # type: ignore[return-value]
    return None, None, None


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="roll_pid", title="Roll PID Tracking & Tuning")
    recs = flight.records
    t0 = flight.t0_us

    imu = recs.get("ISM6HG256") or []
    ns = recs.get("NonSensor") or []
    if not imu or not ns:
        result.warnings.append("Need IMU + NonSensor records for roll-PID analysis.")
        return result

    # ── Times relative to log start ──
    t_imu = (get_array(imu, "time_us") - t0) / 1e6
    g = get_array(imu, "gyro_x")  # roll rate, deg/s
    t_ns = (get_array(ns, "time_us") - t0) / 1e6
    cmd = get_array(ns, "roll_cmd")

    if cmd.size == 0 or np.max(np.abs(cmd)) < 0.05:
        result.warnings.append("No active roll command — not a guidance flight.")
        return result

    # ── Launch time (NonSensor flag, falls back to first |cmd|>0) ──
    launch_t = None
    for r in ns:
        if r.get("launch"):
            launch_t = (r["time_us"] - t0) / 1e6
            break
    if launch_t is None:
        launch_t = float(t_ns[np.argmax(np.abs(cmd) > 0.05)])

    # Re-zero time to launch
    t_imu -= launch_t
    t_ns -= launch_t

    # ── Speed envelope from NonSensor velocity ──
    speed = np.sqrt(
        get_array(ns, "e_vel") ** 2
        + get_array(ns, "n_vel") ** 2
        + get_array(ns, "u_vel") ** 2
    )

    # ── Control engagement ──
    nz_idx = np.where(np.abs(cmd) > 0.05)[0]
    if nz_idx.size == 0:
        result.warnings.append("Command samples all below 0.05° — no control engaged.")
        return result
    t_control_on = float(t_ns[nz_idx[0]])

    # ── Plant gain estimate (first 1 s of active control) ──
    mask_g = (t_imu >= t_control_on) & (t_imu <= t_control_on + 1.0)
    mask_c = (t_ns  >= t_control_on) & (t_ns  <= t_control_on + 1.0)

    K_plant = _K_PLANT_FALLBACK
    r_squared = float("nan")
    if mask_g.sum() > 20 and mask_c.sum() > 5:
        tg_w = t_imu[mask_g]
        g_w = g[mask_g]
        # Drop duplicate timestamps before differentiation
        keep = np.concatenate(([True], np.diff(tg_w) > 1e-6))
        tg_w, g_w = tg_w[keep], g_w[keep]
        if tg_w.size > 20:
            dt_mean = float(np.mean(np.diff(tg_w)))
            window = max(3, int(0.02 / max(dt_mean, 1e-6)))
            g_smooth = _moving_average(g_w, window)
            alpha = np.gradient(g_smooth, tg_w)
            cmd_interp = np.interp(tg_w, t_ns[mask_c], cmd[mask_c])
            sig = np.abs(cmd_interp) > 0.5
            valid = sig & np.isfinite(alpha) & np.isfinite(cmd_interp)
            if valid.sum() > 5:
                num = float(np.sum(alpha[valid] * cmd_interp[valid]))
                den = float(np.sum(cmd_interp[valid] ** 2))
                if den > 0:
                    K_plant = num / den
                    residual = alpha[valid] - K_plant * cmd_interp[valid]
                    ss_res = float(np.sum(residual ** 2))
                    ss_tot = float(np.sum((alpha[valid] - alpha[valid].mean()) ** 2))
                    r_squared = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0
    if not np.isfinite(K_plant) or K_plant == 0:
        K_plant = _K_PLANT_FALLBACK

    # ── Steady-state oscillation FFT (1.5–4 s after control on) ──
    ss_t0 = t_control_on + 1.5
    ss_t1 = min(t_control_on + 4.0, float(t_imu[-1]))
    mask_ss = (t_imu >= ss_t0) & (t_imu <= ss_t1)
    osc_freq = float("nan")
    rms_rate = float("nan")
    peak_rate = float("nan")
    fft_freqs = fft_mag = None
    if mask_ss.sum() > 50:
        g_ss = g[mask_ss]
        tg_ss = t_imu[mask_ss]
        rms_rate = float(np.sqrt(np.mean(g_ss ** 2)))
        peak_rate = float(np.max(np.abs(g_ss)))
        dt_ss = float(np.mean(np.diff(tg_ss)))
        N = len(g_ss)
        fft_freqs = np.fft.rfftfreq(N, d=dt_ss)
        fft_mag = np.abs(np.fft.rfft(g_ss - g_ss.mean())) * 2.0 / N
        peak_mask = fft_freqs > 0.5
        if peak_mask.any():
            peak_idx = int(np.argmax(fft_mag[peak_mask]))
            osc_freq = float(fft_freqs[peak_mask][peak_idx])

    # ── Mean speed in steady-state window ──
    mask_ss_v = (t_ns >= ss_t0) & (t_ns <= ss_t1)
    v_mean = float(np.mean(speed[mask_ss_v])) if mask_ss_v.any() else float("nan")

    # ── Current gains (from sidecar if available) ──
    kp_flight, ki_flight, kd_flight = _gain_from_sidecar(flight.sidecar)

    # ── Stability with current gains ──
    pm_current: float | None = None
    f_c_current: float | None = None
    if kp_flight is not None:
        omega_c = abs(K_plant) * kp_flight
        f_c_current = omega_c / (2 * np.pi)
        servo_lag = np.degrees(np.arctan(omega_c * _DEFAULT_SERVO_TAU_S))
        delay_lag = np.degrees(omega_c * _DEFAULT_COMPUTER_DELAY_S)
        pm_current = float(90.0 - servo_lag - delay_lag)
        if pm_current < 30:
            result.warnings.append(
                f"Low phase margin with flown gains: {pm_current:.0f}° "
                f"— oscillation expected."
            )

    # ── Tuning recommendation ──
    omega_target = 2 * np.pi * _F_TARGET_HZ
    kp_new = omega_target / abs(K_plant)
    servo_lag_t = np.degrees(np.arctan(omega_target * _DEFAULT_SERVO_TAU_S))
    delay_lag_t = np.degrees(omega_target * _DEFAULT_COMPUTER_DELAY_S)
    pm_p_only = 90.0 - servo_lag_t - delay_lag_t
    phase_deficit = max(0.0, _PM_TARGET_DEG - pm_p_only)
    kd_over_kp = np.tan(np.radians(phase_deficit)) / omega_target if phase_deficit > 0 else 0.0
    kd_new = max(kp_new * kd_over_kp, 1e-4)
    ki_new = 0.05 * kp_new * omega_target

    # ── Metrics ──
    metrics: dict[str, object] = {
        "control_engaged_t_s": round(t_control_on, 3),
        "K_plant_estimate_dps2_per_deg": round(K_plant, 1),
        "K_plant_R²": round(r_squared, 3) if np.isfinite(r_squared) else "—",
        "steady_state_window_s": f"[{ss_t0:.2f}, {ss_t1:.2f}]",
        "steady_rate_rms_dps": round(rms_rate, 1) if np.isfinite(rms_rate) else "—",
        "steady_rate_peak_dps": round(peak_rate, 1) if np.isfinite(peak_rate) else "—",
        "dominant_osc_hz": round(osc_freq, 2) if np.isfinite(osc_freq) else "—",
        "mean_speed_mps": round(v_mean, 1) if np.isfinite(v_mean) else "—",
        "current_Kp":             round(kp_flight, 4) if kp_flight is not None else "—",
        "current_Ki":             round(ki_flight, 5) if ki_flight is not None else "—",
        "current_Kd":             round(kd_flight, 5) if kd_flight is not None else "—",
        "current_crossover_hz":   round(f_c_current, 2) if f_c_current is not None else "—",
        "current_phase_margin_°": round(pm_current, 1)  if pm_current  is not None else "—",
        "recommended_Kp":         round(kp_new, 4),
        "recommended_Ki":         round(ki_new, 5),
        "recommended_Kd":         round(kd_new, 5),
        "target_crossover_hz":    round(_F_TARGET_HZ, 2),
        "target_phase_margin_°":  round(_PM_TARGET_DEG, 1),
    }
    result.metrics = metrics

    # ── Figures ──
    # 1) Full timeline: rate, cmd, speed
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t_imu, g, color="tab:green", lw=0.4, alpha=0.5, label="gyro x (raw)")
    if t_imu.size > 50:
        g_sm = _moving_average(g, min(50, max(2, t_imu.size // 10)))
        axes[0].plot(t_imu, g_sm, color="tab:green", lw=1.5, label="50-pt avg")
    axes[0].axhline(0, color="k", lw=0.5, alpha=0.3)
    axes[0].axvline(t_control_on, color="red", linestyle="--", lw=1.2,
                    label=f"control on ({t_control_on:.2f}s)")
    axes[0].set_ylabel("Roll rate (deg/s)")
    axes[0].set_title("Roll rate (gyro X)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_ns, cmd, color="tab:orange", lw=0.8)
    axes[1].axhline(0, color="k", lw=0.5, alpha=0.3)
    axes[1].axvline(t_control_on, color="red", linestyle="--", lw=1.2)
    axes[1].set_ylabel("Roll cmd (deg)")
    axes[1].set_title("Roll command (PID output → servo)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t_ns, speed, color="k", lw=1.2)
    axes[2].axvline(t_control_on, color="red", linestyle="--", lw=1.2)
    axes[2].set_xlabel("Time since launch (s)")
    axes[2].set_ylabel("Speed (m/s)")
    axes[2].set_title("Airspeed (nav)")
    axes[2].grid(True, alpha=0.3)
    fig.tight_layout()
    result.figures.append(fig)

    # 2) Transient detail (-0.5 s … +2 s around control on)
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    win = (t_imu >= t_control_on - 0.5) & (t_imu <= t_control_on + 2.0)
    win_c = (t_ns  >= t_control_on - 0.5) & (t_ns  <= t_control_on + 2.0)
    axes[0].plot(t_imu[win], g[win], color="tab:green", lw=1.0)
    axes[0].axhline(0, color="k", lw=0.5)
    axes[0].axvline(t_control_on, color="red", linestyle="--", lw=1.2, label="control on")
    axes[0].set_ylabel("Roll rate (deg/s)")
    axes[0].set_title("Transient response — first 2 s of active control")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=8)
    axes[1].plot(t_ns[win_c], cmd[win_c], color="tab:orange", lw=1.0)
    axes[1].axhline(0, color="k", lw=0.5)
    axes[1].axvline(t_control_on, color="red", linestyle="--", lw=1.2)
    axes[1].set_xlabel("Time since launch (s)")
    axes[1].set_ylabel("Roll cmd (deg)")
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    result.figures.append(fig)

    # 3) FFT of steady-state roll rate
    if fft_freqs is not None and fft_mag is not None:
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.semilogy(fft_freqs, fft_mag, color="tab:blue", lw=0.8)
        ax.set_xlim(0, 50)
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("Amplitude (deg/s)")
        ax.set_title(f"FFT of roll rate, steady-state window "
                     f"({ss_t0:.1f}–{ss_t1:.1f}s)")
        ax.grid(True, alpha=0.3)
        if np.isfinite(osc_freq):
            ax.axvline(osc_freq, color="red", linestyle="--", lw=1.0,
                       label=f"peak: {osc_freq:.1f} Hz")
            ax.legend()
        fig.tight_layout()
        result.figures.append(fig)

    return result
