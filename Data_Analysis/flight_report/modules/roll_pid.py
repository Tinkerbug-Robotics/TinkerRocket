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
from matplotlib.ticker import MultipleLocator
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

# Manual per-flight launch→ejection window overrides (seconds since launch),
# keyed by .bin stem. The window end is normally found automatically from the
# barometric ejection event (see _baro_eject_time); this map is only an escape
# hatch for flights where that detection misfires. Empty by default.
_WINDOW_TRIM_S: dict[str, float] = {}


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


def _roll_cfg(sidecar: dict) -> dict:
    """The `settings.roll_control` block from the #165 flight-settings snapshot, or {}."""
    rc = (sidecar.get("settings") or {}).get("roll_control")
    return rc if isinstance(rc, dict) else {}


def _gain_from_sidecar(sidecar: dict) -> tuple[float | None, float | None, float | None]:
    """Best-effort extract of (Kp, Ki, Kd) from the per-flight .json sidecar."""
    # Current firmware (#165 settings snapshot) nests the flown gains under
    # settings.roll_control.{kp,ki,kd}. Prefer that; fall back to older shapes.
    rc = _roll_cfg(sidecar)
    if all(k in rc for k in ("kp", "ki", "kd")):
        return float(rc["kp"]), float(rc["ki"]), float(rc["kd"])
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


def _parse_profile(rc: dict) -> list[tuple[float, float, bool]]:
    """Roll-control waypoints as [(time_s_after_launch, angle_deg, is_null_rate)], sorted by time."""
    out: list[tuple[float, float, bool]] = []
    for seg in rc.get("profile") or []:
        try:
            out.append((float(seg["time_s"]), float(seg["angle_deg"]),
                        str(seg.get("mode", "")).lower() == "null_rate"))
        except (KeyError, TypeError, ValueError):
            continue
    out.sort(key=lambda s: s[0])
    return out


def _profile_target(wps: list[tuple[float, float, bool]], t: float) -> tuple[float, bool]:
    """Replicate firmware `roll_profile_query` (main.cpp:1008) EXACTLY: returns
    (target_angle_deg, is_angle_mode) at flight time `t` (seconds since launch).
    Inside the profile the controller commands the segment's DESTINATION — the
    NEXT waypoint's absolute angle, as a STEP (no interpolation) — using the
    CURRENT waypoint's mode, and lets controlAngle take the shortest path to it.
    Holds the first / last waypoint outside the profile span.
    (Was previously interpolating a0→a1, which mislabels the target during
    multi-waypoint angle segments — the firmware does not ramp.)"""
    if not wps:
        return 0.0, False
    if t <= wps[0][0]:
        return wps[0][1], not wps[0][2]
    if t >= wps[-1][0]:
        return wps[-1][1], not wps[-1][2]
    for i in range(len(wps) - 1):
        if t < wps[i + 1][0]:
            return wps[i + 1][1], not wps[i][2]  # NEXT angle (step), CURRENT mode
    return wps[-1][1], not wps[-1][2]


def _quat_roll_deg(q0, q1, q2, q3):
    """Roll angle (deg) extracted exactly as the flight controller does in main.cpp:
    ``-atan2(z_east, z_north)`` from the body-Z axis projected into the nav frame.
    NOTE: the logged ``NonSensor.roll`` Euler field uses a *different* convention and is
    not what the roll controller regulates — always reconstruct from the quaternion."""
    z_north = 2.0 * (q1 * q3 + q0 * q2)
    z_east = 2.0 * (q2 * q3 - q0 * q1)
    return -np.degrees(np.arctan2(z_east, z_north))


def _wrap180(x):
    """Wrap angle(s) in degrees to (-180, 180]."""
    return (np.asarray(x, dtype=float) + 180.0) % 360.0 - 180.0


def _break_seam(y):
    """NaN-break a wrapped series where it jumps the ±180 seam, so a line plot
    doesn't draw a vertical connector across the discontinuity."""
    y = np.asarray(y, dtype=float).copy()
    d = np.abs(np.diff(y))
    y[1:][d > 180.0] = np.nan
    return y


def _shade_segments(ax, tw, is_ang, label="rate-null (no angle target)"):
    """Shade the contiguous time spans where the profile is in null-rate mode."""
    if tw is None or len(tw) == 0:
        return
    shown = False
    start = None
    for i in range(len(tw)):
        null = not bool(is_ang[i])
        if null and start is None:
            start = float(tw[i])
        elif not null and start is not None:
            ax.axvspan(start, float(tw[i]), color="0.85", alpha=0.6, lw=0,
                       label=(label if not shown else None))
            shown = True
            start = None
    if start is not None:
        ax.axvspan(start, float(tw[-1]), color="0.85", alpha=0.6, lw=0,
                   label=(label if not shown else None))


def _as_float(v):
    """Parse a sidecar value to float, or None if missing / non-numeric."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _clip_rate_axis(ax, tw, gw, eject_t, rate_cap=None):
    """Clip a roll-rate axis to in-flight magnitudes so the apogee tumble spike
    doesn't crush the controlled-flight detail; annotate if anything is clipped.
    Scale is anchored on the rate cap (the natural scale of controlled roll rate),
    with a robust percentile floor; the final second before ejection is excluded
    so a pre-apogee tumble ramp can't inflate the scale."""
    if gw is None or len(gw) == 0:
        return
    core = gw[tw <= eject_t - 1.0]
    core = core if core.size else gw
    lim = max(float(np.percentile(np.abs(core), 95)) * 1.4,
              (rate_cap * 2.5) if rate_cap else 0.0, 30.0)
    peak = float(np.max(np.abs(gw)))
    ax.set_ylim(-lim, lim)
    if peak > lim * 1.05:
        ax.text(0.01, 0.04, f"axis clipped — peak |rate| {peak:.0f}°/s",
                transform=ax.transAxes, ha="left", va="bottom", fontsize=8, color="0.4")


def _zoom_cmd_axis(ax, cmd_win, cmd_limit=None):
    """Zoom a roll-command axis to the command's actual range (the observed
    min/max padded by 5°), and note the range / ±cmd-limit authority."""
    if cmd_win is None or len(cmd_win) == 0:
        return
    lo, hi = float(np.nanmin(cmd_win)), float(np.nanmax(cmd_win))
    ax.set_ylim(lo - 5.0, hi + 5.0)
    note = f"cmd ∈ [{lo:.1f}, {hi:.1f}]°"
    if cmd_limit is not None:
        note += f"  ·  limit ±{cmd_limit:g}°"
    ax.text(0.99, 0.04, note, transform=ax.transAxes, ha="right", va="bottom",
            fontsize=8, color="0.4")


def _overlay_speed(ax, t, speed):
    """Overlay airspeed on a right-hand twin axis (roll authority scales with it).
    Keeps the primary (command) line drawn on top."""
    axb = ax.twinx()
    axb.plot(t, speed, color="0.45", lw=1.1, alpha=0.7, zorder=1)
    axb.set_ylabel("Speed (m/s)", color="0.45")
    axb.tick_params(axis="y", labelcolor="0.45")
    ax.set_zorder(axb.get_zorder() + 1)
    ax.patch.set_visible(False)
    return axb


def _fine_time_axis(ax):
    """1 s major / 0.5 s minor ticks with a two-level grid, so transition times
    are easy to read off the launch→eject figures."""
    ax.xaxis.set_major_locator(MultipleLocator(1.0))
    ax.xaxis.set_minor_locator(MultipleLocator(0.5))
    ax.grid(True, which="major", alpha=0.35)
    ax.grid(True, which="minor", alpha=0.15)


def _phase_label(wps, i):
    """Short label for the control phase that STARTS at waypoint i. Matches the
    firmware's step semantics: an angle segment commands the NEXT waypoint's
    angle immediately (no ramp)."""
    wt, wa, wnull = wps[i]
    if wnull:
        return "null-rate"
    if i < len(wps) - 1:
        return f"cmd {wps[i + 1][1]:g}° (step)"
    return f"hold {wa:g}°"


def _effective_plan(wps):
    """Human-readable EFFECTIVE command timeline under the firmware's step
    semantics (`roll_profile_query`): inside segment [i, i+1) the controller
    commands waypoint i+1's angle as a step, using waypoint i's mode. This is
    what actually drives the fins — the raw waypoint list reads misleadingly
    as \"reach angle X at time T\"."""
    if not wps:
        return "—"
    parts = []
    t_first, a_first, n_first = wps[0]
    if t_first > 0:
        parts.append(f"<{t_first:g}s {'null-rate' if n_first else f'cmd {a_first:g}°'}")
    for i in range(len(wps) - 1):
        (ta, _, n0), (tb, a1, _) = wps[i], wps[i + 1]
        parts.append(f"{ta:g}–{tb:g}s {'null-rate' if n0 else f'cmd {a1:g}°'}")
    t_last, a_last, n_last = wps[-1]
    parts.append(f"≥{t_last:g}s {'null-rate' if n_last else f'hold {a_last:g}°'}")
    return "; ".join(parts)


def _fit_r2(x, y):
    """R² of the least-squares line y ≈ kx + b (slope sign free), or NaN if x
    is ~constant. Can go negative for fits worse than the mean."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    if x.size < 10 or np.std(x) < 1e-9 or np.std(y) < 1e-9:
        return float("nan")
    k, b = np.polyfit(x, y, 1)
    ss_res = float(np.sum((y - (k * x + b)) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")


def _gain_schedule_scale(rc_cfg, speed):
    """Firmware gain-schedule multiplier per sample (TR_ServoControl::
    controlWithGainSchedule): min((v_ref / max(|v|, v_min))², scale_cap).
    Returns 1.0s when the schedule is disabled or unconfigured."""
    gs = rc_cfg.get("gain_schedule") or {}
    speed = np.asarray(speed, dtype=float)
    if not gs.get("enabled"):
        return np.ones_like(speed)
    v_ref = _as_float(gs.get("v_ref"))
    v_min = _as_float(gs.get("v_min"))
    cap = _as_float(gs.get("scale_cap"))
    if not v_ref or not v_min or not cap:
        return np.ones_like(speed)
    v = np.maximum(np.abs(speed), v_min)
    return np.minimum((v_ref / v) ** 2, cap)


def _baro_eject_time(flight, t0, launch_t, burnout_t, apogee_t):
    """Detect the ejection event from the barometric pressure spike (the charge
    gas pulse over-pressuring the baro port): the first post-burnout sample where
    |dP/dt| massively exceeds the coast baseline. The spike is ~1e6 Pa/s vs ~1e3
    in clean coast, so it separates cleanly. Returns launch-relative seconds, or
    None if no clear event (then we fall back to apogee)."""
    bmp = flight.records.get("BMP585") or []
    if len(bmp) < 50:
        return None
    tb = (get_array(bmp, "time_us") - t0) / 1e6 - launch_t
    p = get_array(bmp, "pressure_pa")
    order = np.argsort(tb, kind="stable")
    tb, p = tb[order], p[order]
    keep = np.concatenate(([True], np.diff(tb) > 1e-6)) & np.isfinite(p)
    tb, p = tb[keep], p[keep]
    if tb.size < 50:
        return None
    dpdt = np.abs(np.gradient(p, tb))
    bo = burnout_t if burnout_t is not None else 1.0
    coast = tb > bo + 0.5
    if coast.sum() < 20:
        return None
    # Threshold well above coast noise (absolute floor) yet far below the spike;
    # the median term lifts it for unusually noisy baro.
    thr = max(100000.0, 50.0 * float(np.median(dpdt[coast])))
    # Ejection is at/near apogee for these motors — ignore later descent/landing spikes.
    search = coast & (tb <= apogee_t + 2.0) & (dpdt > thr)
    idx = np.where(search)[0]
    return float(tb[idx[0]]) if idx.size else None


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

    # ── Roll-angle tracking vs profile plan (launch → ejection) ──
    # Reconstruct the commanded waypoint plan and the actual roll the controller
    # regulated, both exactly as the firmware computes them.
    rc_cfg = _roll_cfg(flight.sidecar)
    wps = _parse_profile(rc_cfg)
    claimed_mode = str(rc_cfg.get("mode", "")).lower()
    has_wp_angles = any(not null for (_, _, null) in wps)

    # Ejection ≈ apogee (end of roll authority), launch-relative.
    eject_t = None
    for r in ns:
        if r.get("apogee_flag"):
            eject_t = (r["time_us"] - t0) / 1e6 - launch_t
            break
    if eject_t is None or not np.isfinite(eject_t) or eject_t <= 0:
        apo = flight.sidecar.get("apogee_time_s")
        eject_t = (float(apo) if apo not in (None, "")
                   else float(min(t_control_on + 6.0, t_imu[-1])))
    eject_t = float(min(eject_t, t_imu[-1]))

    burnout_t = flight.sidecar.get("burnout_time_s")
    burnout_t = float(burnout_t) if burnout_t not in (None, "") else None

    # End the plot/analysis window 0.25 s before the barometric ejection event
    # (charge pressure spike), so the post-ejection tumble is excluded generally
    # rather than per-flight. Fall back to apogee if no clear baro event. eject_t
    # stays the real apogee for the metrics/marker.
    eject_baro = _baro_eject_time(flight, t0, launch_t, burnout_t, eject_t)
    if eject_baro is not None:
        win_end = max(eject_baro - 0.25, (burnout_t or 0.0) + 0.5)
    else:
        win_end = eject_t
    trim_t = _WINDOW_TRIM_S.get(flight.bin_path.stem)  # manual override (rare); wins if set
    if trim_t is not None:
        win_end = min(win_end, float(trim_t))
    win_end = float(min(win_end, t_imu[-1]))

    # Steepest commanded ramp rate across the angle segments (deg/s).
    rate_cap_cfg = _as_float(rc_cfg.get("rate_cap_dps"))
    kp_angle_cfg = _as_float(rc_cfg.get("kp_angle"))
    rate_setpt = _as_float(rc_cfg.get("roll_rate_set_point")) or 0.0
    profile_ramp = 0.0
    for i in range(len(wps) - 1):
        (t0w, a0w, n0), (t1w, a1w, _) = wps[i], wps[i + 1]
        if not n0 and t1w > t0w:
            profile_ramp = max(profile_ramp, abs(a1w - a0w) / (t1w - t0w))

    track_tw = track_tgt = track_act = track_err = track_is_ang = track_ratecmd = None
    track_cmd = None
    angle_rms = angle_peak = float("nan")
    if has_wp_angles:
        roll_act = _quat_roll_deg(*(get_array(ns, k) for k in ("q0", "q1", "q2", "q3")))
        wmask = (t_ns >= 0.0) & (t_ns <= win_end + 0.05) & np.isfinite(roll_act)
        track_tw = t_ns[wmask]
        track_cmd = cmd[wmask]
        act_w = roll_act[wmask]
        tq = [_profile_target(wps, float(tt)) for tt in track_tw]
        target_ang = np.array([x[0] for x in tq], dtype=float)
        track_is_ang = np.array([x[1] for x in tq], dtype=bool)
        # Wrapped shortest-path error, exactly like servo_control.controlAngle().
        err = _wrap180(target_ang - act_w)
        # Plot both target and actual wrapped to (-180,180] so the vertical gap
        # equals the true error the controller acts on; break the ±180 seam.
        # Actual roll is shown across the whole window (incl. the null-rate
        # period); the target/error exist only where an angle is commanded.
        track_tgt = _break_seam(np.where(track_is_ang, _wrap180(target_ang), np.nan))
        track_act = _break_seam(_wrap180(act_w))
        track_err = np.where(track_is_ang, err, np.nan)
        ea = err[track_is_ang]
        if ea.size:
            angle_rms = float(np.sqrt(np.mean(ea ** 2)))
            angle_peak = float(np.max(np.abs(ea)))
        # Reconstruct the firmware's outer-loop rate command: angle segments
        # → clip(kp_angle·wrapped_err, ±rate_cap); null-rate segments → setpoint.
        # Same sign frame as gyro_x (the inner loop drives gyro_x → rate_cmd).
        if kp_angle_cfg is not None and rate_cap_cfg is not None:
            rc_angle = np.clip(kp_angle_cfg * err, -rate_cap_cfg, rate_cap_cfg)
            track_ratecmd = np.where(track_is_ang, rc_angle, rate_setpt)

    # ── Which roll mode actually FLEW? ──
    # The sidecar `mode` can't be trusted on its own: app exports before
    # 2026-07-05 labeled any flight with a *stored* waypoint profile
    # "angle_profile" even when use_angle_control was off (Null Roll) — and the
    # firmware ignores a stored profile in that case and flies pure rate-null.
    # Cross-check against the log: the inner rate loop's error is
    # (gyro − rate_cmd) if the angle cascade ran, vs (gyro − setpoint) if
    # rate-null flew; the logged servo command is ~proportional to whichever
    # error was real, so correlation over the angle segments separates them.
    flown_mode_note = None
    has_angle_profile = has_wp_angles and claimed_mode != "rate"
    if has_wp_angles and claimed_mode != "rate" and track_ratecmd is not None:
        gyro_w = np.interp(track_tw, t_imu, g)
        # Only samples where the hypotheses actually differ are informative.
        sel = track_is_ang & (np.abs(track_ratecmd - rate_setpt) > 10.0)
        # The fin command is Kp_eff·error (+ small I/D); divide out the
        # velocity gain schedule so the fit slope is ~constant over the window.
        cmd_norm = track_cmd / _gain_schedule_scale(rc_cfg, speed[wmask])
        if sel.sum() < 20:
            flown_mode_note = ("undecidable from servo data (too few angle-segment "
                               "samples) — trusting sidecar mode")
        elif float(np.std(track_ratecmd[sel])) < 15.0:
            # Outer-loop cmd railed ~constant: the two error series differ only
            # by a constant, which the fit intercept absorbs — undecidable.
            flown_mode_note = ("undecidable from servo data (outer-loop cmd "
                               "~constant over angle segments) — trusting sidecar mode")
        else:
            r2_angle = _fit_r2(gyro_w[sel] - track_ratecmd[sel], cmd_norm[sel])
            r2_null = _fit_r2(gyro_w[sel] - rate_setpt, cmd_norm[sel])
            if not (np.isfinite(r2_null) and np.isfinite(r2_angle)):
                # Typically the fin command pinned at the ± limit throughout.
                flown_mode_note = ("undecidable from servo data (fin command "
                                   "~constant/saturated over angle segments) — "
                                   "trusting sidecar mode")
            elif r2_null - r2_angle > 0.25 and r2_null > 0.3:
                has_angle_profile = False
                result.warnings.append(
                    f"Sidecar claims roll mode '{claimed_mode}' but the servo "
                    f"command fits pure rate-null (R²={r2_null:.2f}) far better "
                    f"than the angle cascade (R²={r2_angle:.2f}) — the stored "
                    f"profile was NOT followed (use_angle_control off; app exports "
                    f"before 2026-07-05 mislabel this). Analyzing as rate-null."
                )
                flown_mode_note = (f"rate-null flew (servo fit R² {r2_null:.2f} "
                                   f"vs angle-cascade {r2_angle:.2f})")
            elif r2_angle - r2_null > 0.25 and r2_angle > 0.3:
                flown_mode_note = (f"angle cascade confirmed (servo fit R² "
                                   f"{r2_angle:.2f} vs rate-null {r2_null:.2f})")
            else:
                flown_mode_note = (f"inconclusive (servo fit R²: angle {r2_angle:.2f}, "
                                   f"rate-null {r2_null:.2f}) — trusting sidecar mode")

    # ── Current gains (from sidecar if available) ──
    kp_flight, ki_flight, kd_flight = _gain_from_sidecar(flight.sidecar)
    kp_angle_flight = rc_cfg.get("kp_angle")

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
        "current_Kp_angle":       kp_angle_flight if kp_angle_flight is not None else "—",
        "current_crossover_hz":   round(f_c_current, 2) if f_c_current is not None else "—",
        "current_phase_margin_°": round(pm_current, 1)  if pm_current  is not None else "—",
        "recommended_Kp":         round(kp_new, 4),
        "recommended_Ki":         round(ki_new, 5),
        "recommended_Kd":         round(kd_new, 5),
        "target_crossover_hz":    round(_F_TARGET_HZ, 2),
        "target_phase_margin_°":  round(_PM_TARGET_DEG, 1),
    }
    # Flown roll-control setup parameters (from the #165 settings snapshot).
    if rc_cfg:
        metrics["roll_mode"]      = rc_cfg.get("mode", "—")
        if flown_mode_note:
            metrics["flown_mode_check"] = flown_mode_note
        metrics["roll_delay_ms"]  = rc_cfg.get("delay_ms", "—")
        metrics["rate_cap_dps"]   = rc_cfg.get("rate_cap_dps", "—")
        metrics["cmd_limit_deg"]  = (f"[{rc_cfg.get('cmd_limit_min_deg', '?')}, "
                                     f"{rc_cfg.get('cmd_limit_max_deg', '?')}]")
        metrics["d_lpf_hz"]       = rc_cfg.get("d_lpf_hz", "—")
        metrics["guidance_enabled"] = rc_cfg.get("guidance_enabled", "—")
    # Window timing (applies to every active-roll flight's launch→eject figure).
    metrics["eject_time_s"] = round(eject_t, 2)
    if eject_baro is not None:
        metrics["baro_eject_s"] = round(eject_baro, 2)
    metrics["plot_window_s"] = f"[0, {win_end:.2f}]"
    if has_wp_angles and not has_angle_profile:
        # Profile stored on the FC but not followed (rate mode flew).
        metrics["profile_stored_NOT_followed"] = _effective_plan(wps)
    if has_angle_profile:
        metrics["profile_commanded"] = _effective_plan(wps)
        metrics["profile_waypoints"] = "; ".join(
            f"{wt:g}s→{'null-rate' if wn else f'{wa:g}°'}" for (wt, wa, wn) in wps)
        if profile_ramp > 0:
            feasible = (rate_cap_cfg is None) or (profile_ramp <= rate_cap_cfg + 1e-6)
            note = "" if feasible else f"  (EXCEEDS rate cap {rate_cap_cfg:g}°/s — target unreachable)"
            metrics["profile_ramp_dps"] = f"{profile_ramp:.0f}{note}"
        # NOTE: with an over-fast ramp the wrapped error reflects an unreachable
        # setpoint, not control performance — read alongside profile_ramp_dps.
        metrics["angle_track_rms_deg"]  = round(angle_rms, 1)  if np.isfinite(angle_rms)  else "—"
        metrics["angle_track_peak_deg"] = round(angle_peak, 1) if np.isfinite(angle_peak) else "—"

        # Per-segment breakdown over the analysis window: what was commanded,
        # how the angle error evolved, and the residual rate / fin activity.
        bounds = [0.0] + [wt for (wt, _, _) in wps if 0.0 < wt < win_end] + [win_end]
        for si in range(len(bounds) - 1):
            a, b = bounds[si], bounds[si + 1]
            if b - a < 0.05:
                continue
            mid = 0.5 * (a + b)
            tgt_mid, is_ang_mid = _profile_target(wps, mid)
            mi = (t_imu >= a) & (t_imu <= b)
            mc = (t_ns >= a) & (t_ns <= b)
            rate_rms_seg = float(np.sqrt(np.mean(g[mi] ** 2))) if mi.any() else float("nan")
            cmd_max_seg = float(np.max(np.abs(cmd[mc]))) if mc.any() else float("nan")
            if is_ang_mid and track_tw is not None:
                ms = (track_tw >= a) & (track_tw <= b) & track_is_ang
                e_seg = track_err[ms] if ms.any() else np.array([])
                e_seg = e_seg[np.isfinite(e_seg)]
                if e_seg.size:
                    desc = (f"cmd {tgt_mid:g}°: err {e_seg[0]:+.0f}→{e_seg[-1]:+.0f}° "
                            f"(RMS {np.sqrt(np.mean(e_seg**2)):.0f}°), ")
                else:
                    desc = f"cmd {tgt_mid:g}°: "
            else:
                desc = "null-rate: "
            desc += f"rate RMS {rate_rms_seg:.0f}°/s, |fin cmd| max {cmd_max_seg:.1f}°"
            metrics[f"segment {a:.2g}–{b:.2g}s"] = desc
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

    # Program events / phase transitions within the plotted window. With
    # annotate=True, each line gets a timestamped, vertical label (use on the
    # top panel only).
    def _event_list():
        evs = [(t_control_on, "control on", "red", "--")]
        if burnout_t is not None and -0.2 <= burnout_t <= win_end + 0.2:
            evs.append((burnout_t, "burnout", "tab:purple", "--"))
        if eject_t <= win_end + 0.2:
            evs.append((eject_t, "apogee/eject", "black", "--"))
        for i, (wt, _, wnull) in enumerate(wps):
            if -0.2 <= wt <= win_end + 0.2:
                # null-rate segments are already shown by the gray shading; only
                # label the meaningful mode-change transitions (angle / hold).
                evs.append((wt, None if wnull else _phase_label(wps, i), "gray", ":"))
        return evs

    def _mark_events(ax, annotate=False):
        for et, lbl, c, ls in _event_list():
            ax.axvline(et, color=c, linestyle=ls, lw=(0.8 if ls == ":" else 1.0))
        if annotate:
            for et, lbl, c, ls in _event_list():
                if lbl is None:
                    continue
                ax.annotate(f"{et:.2f}s  {lbl}", xy=(et, 0.0),
                            xycoords=("data", "axes fraction"),
                            xytext=(2, 3), textcoords="offset points",
                            rotation=90, va="bottom", ha="left",
                            fontsize=7, color=c)

    # Launch → ejection window (shared by the figures below), trimmed if configured.
    t_lo, t_hi = -0.2, win_end + 0.2
    win = (t_imu >= t_lo) & (t_imu <= t_hi)
    win_c = (t_ns >= t_lo) & (t_ns <= t_hi)
    rate_cap = _as_float(rc_cfg.get("rate_cap_dps"))

    # 2) Control timeline — launch → ejection (roll rate + servo command).
    #    For angle-profile flights this is folded into the 4-panel tracking
    #    figure below, so only render it standalone for rate-only flights.
    if not has_angle_profile:
        fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        axes[0].plot(t_imu[win], g[win], color="tab:green", lw=0.9)
        axes[0].axhline(0, color="k", lw=0.5)
        axes[0].set_ylabel("Roll rate (deg/s)")
        axes[0].set_title("Roll control — launch → ejection")
        _clip_rate_axis(axes[0], t_imu[win], g[win], win_end, rate_cap)
        _mark_events(axes[0], annotate=True)
        axes[1].plot(t_ns[win_c], cmd[win_c], color="tab:orange", lw=1.0, zorder=3)
        axes[1].axhline(0, color="k", lw=0.5)
        axes[1].set_xlabel("Time since launch (s)")
        axes[1].set_ylabel("Roll cmd (deg)", color="tab:orange")
        axes[1].tick_params(axis="y", labelcolor="tab:orange")
        _zoom_cmd_axis(axes[1], cmd[win_c], _as_float(rc_cfg.get("cmd_limit_max_deg")))
        _overlay_speed(axes[1], t_ns[win_c], speed[win_c])
        _mark_events(axes[1])
        for ax in axes:
            _fine_time_axis(ax)
        fig.tight_layout()
        result.figures.append(fig)

    # 3) Roll-angle tracking vs profile plan + control activity (launch → ejection)
    if has_angle_profile and track_tw is not None and track_tw.size > 5:
        fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

        # Panel 0 — roll angle: target vs actual (both wrapped ±180)
        _shade_segments(axes[0], track_tw, track_is_ang)
        axes[0].plot(track_tw, track_tgt, color="tab:orange", lw=2.2, label="profile target")
        axes[0].plot(track_tw, track_act, color="tab:green", lw=1.4, label="actual roll (quaternion)")
        axes[0].set_ylabel("Roll angle\n(deg, wrapped ±180)")
        axes[0].set_ylim(-189, 189)
        axes[0].set_yticks([-180, -90, 0, 90, 180])
        axes[0].set_title("Roll angle tracking vs profile plan (launch → ejection)")
        axes[0].grid(True, alpha=0.3)
        # Label each ANGLE segment with its effective command (the NEXT
        # waypoint's angle, per the firmware's step semantics), centred on the
        # visible part of the segment — not the waypoint angle at the waypoint
        # time, which reads as "hold this from here" and is wrong.
        seg_spans = [(wps[i][0], wps[i + 1][0], wps[i + 1][1], wps[i][2])
                     for i in range(len(wps) - 1)]
        if not wps[-1][2]:  # trailing hold segment if the last waypoint is angle-mode
            seg_spans.append((wps[-1][0], win_end, wps[-1][1], False))
        for (sa, sb, s_ang, s_null) in seg_spans:
            sa, sb = max(sa, 0.0), min(sb, win_end)
            if s_null or sb - sa < 0.05:
                continue
            axes[0].annotate(f"cmd {s_ang:g}°", xy=(0.5 * (sa + sb), _wrap180(s_ang)),
                             fontsize=8, color="tab:orange", ha="center", va="bottom",
                             xytext=(0, 3), textcoords="offset points")
        _mark_events(axes[0], annotate=True)
        axes[0].legend(loc="upper left", fontsize=8)

        # Panel 1 — tracking error
        _shade_segments(axes[1], track_tw, track_is_ang, label=None)
        axes[1].axhline(0, color="k", lw=0.5)
        axes[1].plot(track_tw, track_err, color="tab:red", lw=1.1)
        axes[1].set_ylabel("Tracking error\n(deg)")
        axes[1].grid(True, alpha=0.3)
        _mark_events(axes[1])
        if np.isfinite(angle_rms):
            txt = f"RMS {angle_rms:.1f}°   peak {angle_peak:.1f}°"
            if rate_cap is not None and profile_ramp > rate_cap + 1e-6:
                txt += (f"\nramp {profile_ramp:.0f}°/s > cap {rate_cap:g}°/s"
                        f"\n→ setpoint unreachable, error not a control metric")
            axes[1].text(0.99, 0.05, txt,
                         transform=axes[1].transAxes, ha="right", va="bottom", fontsize=8,
                         bbox=dict(boxstyle="round", fc="white", ec="0.7", alpha=0.85))

        # Panel 2 — roll rate: achieved (gyro) vs the firmware's commanded rate
        # (outer loop, clipped to ±rate cap), so the cap + the ±180° wrap flip
        # are visible against what the rocket actually did.
        _shade_segments(axes[2], track_tw, track_is_ang, label=None)
        axes[2].axhline(0, color="k", lw=0.5)
        axes[2].plot(t_imu[win], g[win], color="tab:green", lw=0.9, label="roll rate (gyro, achieved)")
        if track_ratecmd is not None:
            axes[2].plot(track_tw, track_ratecmd, color="tab:blue", lw=1.5,
                         label="commanded rate (outer loop)")
        axes[2].set_ylabel("Roll rate\n(deg/s)")
        axes[2].grid(True, alpha=0.3)
        _clip_rate_axis(axes[2], t_imu[win], g[win], win_end, rate_cap)
        if rate_cap is not None:
            axes[2].axhline(rate_cap, color="tab:blue", ls=":", lw=1.0,
                            label=f"rate cap ±{rate_cap:g}°/s")
            axes[2].axhline(-rate_cap, color="tab:blue", ls=":", lw=1.0)
        _mark_events(axes[2])
        axes[2].legend(loc="upper right", fontsize=8)

        # Panel 3 — roll command (PID output → servo), zoomed to its actual
        # range, with airspeed overlaid (roll authority scales with speed).
        _shade_segments(axes[3], track_tw, track_is_ang, label=None)
        axes[3].axhline(0, color="k", lw=0.5)
        axes[3].plot(t_ns[win_c], cmd[win_c], color="tab:orange", lw=1.0, zorder=3)
        axes[3].set_ylabel("Roll cmd (deg)", color="tab:orange")
        axes[3].tick_params(axis="y", labelcolor="tab:orange")
        axes[3].set_xlabel("Time since launch (s)")
        _zoom_cmd_axis(axes[3], cmd[win_c], _as_float(rc_cfg.get("cmd_limit_max_deg")))
        _overlay_speed(axes[3], t_ns[win_c], speed[win_c])
        _mark_events(axes[3])

        for ax in axes:
            _fine_time_axis(ax)
        fig.tight_layout()
        result.figures.append(fig)

    # 4) FFT of steady-state roll rate
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
