"""Roll-PID tracking & tuning module.

Driven from the binary log (not the legacy CSV input). Estimates the plant
gain K_plant, FFT-identifies any persistent oscillation, computes phase
margin given the current Kp, and recommends tuned gains for a target
crossover frequency.

Drawn with the report's interactive charts (2026-08-29; the matplotlib
figures this section carried before are retired). One deliberate omission:
the four-panel launch→ejection tracking figure is NOT reproduced here — the
Roll Control section directly above renders exactly those panels,
interactively, from the same `roll_series` data, and shipping the arrays
twice under colliding chart ids bought nothing. This section keeps what the
Roll section deliberately leaves out: the full-log timeline, the rate-only
close-up, the steady-state spectrum, and the tuning numbers.

Skips gracefully on flights without active roll control (`roll_cmd ≡ 0`).
Adapted from `analyze_roll_pid.py`.
"""

from __future__ import annotations

import sys
from pathlib import Path

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

from ..charts import chart, trace
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


def _profile_target(wps: list[tuple[float, float, bool]], t: float,
                    semantics: str = "step") -> tuple[float, bool]:
    """Replicate firmware `roll_profile_query` EXACTLY: returns
    (target_angle_deg, is_angle_mode) at flight time `t` (seconds since launch).

    semantics="step" (firmware pre-v4, sidecars without `profile_semantics`):
    inside the profile the controller commands the segment's DESTINATION — the
    NEXT waypoint's absolute angle, as a STEP (no interpolation) — using the
    CURRENT waypoint's per-waypoint mode; holds the first / last waypoint
    outside the profile span.

    semantics="ramp" (firmware v4+): pure (time, angle) waypoints — per-wp
    modes ignored; NULL_RATE before the first waypoint; target linearly
    interpolated along the shortest wrapped arc between waypoints; last angle
    held after the profile."""
    if not wps:
        return 0.0, False
    if semantics == "ramp":
        if t < wps[0][0]:
            return 0.0, False  # null-rate before the profile
        if t >= wps[-1][0]:
            return wps[-1][1], True
        for i in range(len(wps) - 1):
            (t0, a0, _), (t1, a1, _) = wps[i], wps[i + 1]
            if t < t1:
                frac = (t - t0) / (t1 - t0) if t1 - t0 > 1e-3 else 1.0
                tgt = float(_wrap180(a0 + float(_wrap180(a1 - a0)) * frac))
                return tgt, True
        return wps[-1][1], True
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


def _as_float(v):
    """Parse a sidecar value to float, or None if missing / non-numeric."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


# Chart colors — matplotlib tab-palette parity with the retired static figures.
_C_RATE = "#2ca02c"     # tab:green
_C_CMD = "#ff7f0e"      # tab:orange
_C_SPEED = "#555555"
_C_FFT = "#1f77b4"      # tab:blue
_C_PEAK = "#d62728"     # tab:red


def rate_axis_limit(t, g, eject_t, cap) -> tuple[float, float]:
    """Y-limit for a roll-rate panel, and the true peak it may be hiding.

    Shared with the flight-level roll section so both scale that axis the same
    way: the larger of a robust percentile of the in-flight magnitudes and a
    multiple of the rate cap, with the last second before ejection excluded so a
    tumble that starts early cannot inflate it.
    """
    g = np.asarray(g, dtype=float)
    t = np.asarray(t, dtype=float)
    n = min(g.size, t.size)
    # Mask both together. Dropping non-finite samples from g first and then
    # slicing t to the survivors' *count* would pair each reading with the
    # wrong timestamp, so the window below would select the wrong samples.
    fin = np.isfinite(g[:n])
    g, t = g[:n][fin], t[:n][fin]
    if not g.size:
        return 30.0, 0.0
    core = g
    if eject_t is not None and np.isfinite(eject_t):
        sel = t <= float(eject_t) - 1.0
        if sel.any():
            core = g[sel]
    lim = max(float(np.percentile(np.abs(core), 95)) * 1.4,
              (float(cap) * 2.5) if cap else 0.0, 30.0)
    return lim, float(np.max(np.abs(g)))


def _hline(spec: dict, y: float, color: str, dash: str = "dot") -> None:
    """A horizontal reference line in data coordinates (roll.py's idiom).

    Safe against the units toggle only because every axis this is used on is in
    degrees or degrees per second, neither of which the toggle converts.
    """
    if not spec:
        return
    spec["layout"].setdefault("shapes", []).append({
        "type": "line", "xref": "paper", "yref": "y",
        "x0": 0, "x1": 1, "y0": y, "y1": y,
        "line": {"color": color, "width": 1, "dash": dash},
        "layer": "below",
    })


def _stack(specs: list, x_range: list[float], dtick: float | None = None) -> list:
    """Make a run of charts read as one multi-panel figure.

    Same treatment as the Roll section's four-panel stack: a shared x window,
    tick labels and the axis title only on the bottom panel, event labels only
    on the top one, collapsed margins between, and legends inset so nothing
    lands in the seams. `dtick` adds the 1 s major / 0.5 s minor grid the
    launch→eject close-ups use.
    """
    kept = [sp for sp in specs if sp]
    for i, sp in enumerate(kept):
        first, last = i == 0, i == len(kept) - 1
        ax = sp["layout"]["xaxis"]
        ax["range"] = list(x_range)
        ax["showticklabels"] = last
        if dtick:
            ax.update({
                "tick0": 0, "dtick": dtick,
                "minor": {"dtick": dtick / 2, "showgrid": True,
                          "gridcolor": "#f4f4f4"},
            })
        if not first:
            sp["layout"]["annotations"] = []
            sp["layout"]["title"] = {"text": "", "font": {"size": 14}}
            sp["layout"]["height"] = sp["layout"]["height"] - 40
        sp["layout"]["margin"] = {
            # The y2 axis draws its labels in the right margin; collapsing it
            # to the stack's 20 px would clip them.
            "l": 70, "r": 60 if "yaxis2" in sp["layout"] else 20,
            "t": 40 if first else 8,
            "b": 42 if last else 6,
        }
        if sp["layout"].get("showlegend"):
            sp["layout"]["legend"] = {
                "x": 0.008 if first else 0.992,
                "xanchor": "left" if first else "right",
                "y": 0.98, "yanchor": "top",
                "bgcolor": "rgba(255,255,255,0.72)",
                "borderwidth": 0, "font": {"size": 10},
            }
        sp["stack"] = "first" if first else ("last" if last else "mid")
    return kept


def _fft_trace(freqs, mag):
    """The spectrum as one line trace, built by hand rather than via `trace()`.

    The shared builder rounds y to 4 decimals — right for sensor series, fatal
    for a spectrum drawn on a log axis, where the noise floor lives well below
    1e-4 °/s and would round to an unplottable 0. Zeros are dropped for the
    same reason (log of the DC bin's exact 0 has no pixel), and everything
    non-finite goes because NaN/Inf in a chart spec kills the whole document.
    """
    f = np.asarray(freqs, dtype=float)
    m = np.asarray(mag, dtype=float)
    n = min(f.size, m.size)
    f, m = f[:n], m[:n]
    # f > 0 also drops the DC bin: the window is mean-subtracted before the
    # FFT, so bin 0 is numerical dust (~1e-16) that would stretch the log
    # autorange down to femto-scale and flatten the real spectrum.
    good = np.isfinite(f) & np.isfinite(m) & (f > 0) & (m > 0)
    f, m = f[good], m[good]
    if f.size < 4:
        return None
    return {
        "type": "scatter", "mode": "lines", "name": "Amplitude",
        "x": np.round(f, 3).tolist(),
        "y": [float(f"{v:.5g}") for v in m],
        "yaxis": "y",
        "line": {"width": 1.2, "color": _C_FFT},
        "hovertemplate": "%{y:.3g} °/s @ %{x:.2f} Hz<extra></extra>",
    }


def _effective_plan(wps, semantics="step"):
    """Human-readable EFFECTIVE command timeline as the firmware flies it.
    step (pre-v4): inside segment [i, i+1) the controller commands waypoint
    i+1's angle as a STEP, using waypoint i's mode — the raw waypoint list
    reads misleadingly as \"reach angle X at time T\".
    ramp (v4+): null-rate before the first waypoint, target lerps between
    waypoints, last angle held."""
    if not wps:
        return "—"
    parts = []
    t_first, a_first, n_first = wps[0]
    if semantics == "ramp":
        if t_first > 0:
            parts.append(f"<{t_first:g}s null-rate")
        for i in range(len(wps) - 1):
            (ta, a0, _), (tb, a1, _) = wps[i], wps[i + 1]
            seg = f"hold {a0:g}°" if a1 == a0 else f"ramp {a0:g}→{a1:g}°"
            parts.append(f"{ta:g}–{tb:g}s {seg}")
        parts.append(f"≥{wps[-1][0]:g}s hold {wps[-1][1]:g}°")
        return "; ".join(parts)
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



def tuning(K_plant: float, kp_flight: float | None) -> dict:
    """Loop stability with the gains that flew, and the gains to fly next time.

    Shared by the detailed report's roll-PID section and the flight-level roll section
    so the two cannot recommend different numbers for the same flight. Phase
    margin is the first-order servo lag plus the loop delay subtracted from the
    90° a pure integrator would give; the recommendation places the crossover at
    `_F_TARGET_HZ` and adds whatever derivative term buys back the phase needed
    to reach `_PM_TARGET_DEG`.
    """
    pm_current: float | None = None
    f_c_current: float | None = None
    if kp_flight is not None:
        omega_c = abs(K_plant) * kp_flight
        f_c_current = omega_c / (2 * np.pi)
        servo_lag = np.degrees(np.arctan(omega_c * _DEFAULT_SERVO_TAU_S))
        delay_lag = np.degrees(omega_c * _DEFAULT_COMPUTER_DELAY_S)
        pm_current = float(90.0 - servo_lag - delay_lag)

    omega_target = 2 * np.pi * _F_TARGET_HZ
    kp_new = omega_target / abs(K_plant)
    servo_lag_t = np.degrees(np.arctan(omega_target * _DEFAULT_SERVO_TAU_S))
    delay_lag_t = np.degrees(omega_target * _DEFAULT_COMPUTER_DELAY_S)
    pm_p_only = 90.0 - servo_lag_t - delay_lag_t
    phase_deficit = max(0.0, _PM_TARGET_DEG - pm_p_only)
    kd_over_kp = np.tan(np.radians(phase_deficit)) / omega_target if phase_deficit > 0 else 0.0
    return {
        "pm_current": pm_current,
        "f_c_current": f_c_current,
        "kp_new": kp_new,
        "kd_new": max(kp_new * kd_over_kp, 1e-4),
        "ki_new": 0.05 * kp_new * omega_target,
        "pm_p_only": pm_p_only,
    }


def roll_series(flight: Flight) -> dict:
    """Every series and flag the roll analysis needs, computed once.

    Shared by the detailed report's `roll_pid` section and the flight-level `roll`
    section so both agree on what the flight was. In particular
    `has_angle_profile` — "did roll *angle* control actually fly?" — is not the
    sidecar's word for it: a mislabelled export is cross-checked against the
    servo command with an R² fit and can be overruled here.

    Returns `{"abort": reason}` when there is nothing to analyze.
    """
    warnings: list[str] = []
    recs = flight.records
    t0 = flight.t0_us

    imu = recs.get("ISM6HG256") or []
    ns = recs.get("NonSensor") or []
    if not imu or not ns:
        return {"abort": "Need IMU + NonSensor records for roll-PID analysis."}

    # ── Times relative to log start ──
    t_imu = (get_array(imu, "time_us") - t0) / 1e6
    g = get_array(imu, "gyro_x")  # roll rate, deg/s
    t_ns = (get_array(ns, "time_us") - t0) / 1e6
    cmd = get_array(ns, "roll_cmd")

    if cmd.size == 0 or np.max(np.abs(cmd)) < 0.05:
        return {"abort": "No active roll command — not a guidance flight."}

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
        return {"abort": "Command samples all below 0.05° — no control engaged."}
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
    # Profile semantics the FIRMWARE flew: "ramp" (v4+: lerp between waypoints,
    # per-wp modes ignored) vs "step" (pre-v4: step to NEXT waypoint's angle,
    # per-wp null_rate honored). Exports without the marker are pre-v4.
    semantics = str(rc_cfg.get("profile_semantics", "step")).lower()
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
            d_ang = abs(float(_wrap180(a1w - a0w)))  # shortest-path arc
            profile_ramp = max(profile_ramp, d_ang / (t1w - t0w))

    track_tw = track_tgt = track_act = track_err = track_is_ang = track_ratecmd = None
    track_cmd = None
    angle_rms = angle_peak = float("nan")
    if has_wp_angles:
        roll_act = _quat_roll_deg(*(get_array(ns, k) for k in ("q0", "q1", "q2", "q3")))
        wmask = (t_ns >= 0.0) & (t_ns <= win_end + 0.05) & np.isfinite(roll_act)
        track_tw = t_ns[wmask]
        track_cmd = cmd[wmask]
        act_w = roll_act[wmask]
        tq = [_profile_target(wps, float(tt), semantics) for tt in track_tw]
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
                warnings.append(
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
    return {
        "K_plant": K_plant,
        "angle_peak": angle_peak,
        "angle_rms": angle_rms,
        "burnout_t": burnout_t,
        "cmd": cmd,
        "eject_baro": eject_baro,
        "eject_t": eject_t,
        "fft_freqs": fft_freqs,
        "fft_mag": fft_mag,
        "flown_mode_note": flown_mode_note,
        "g": g,
        "has_angle_profile": has_angle_profile,
        "has_wp_angles": has_wp_angles,
        "osc_freq": osc_freq,
        "peak_rate": peak_rate,
        "profile_ramp": profile_ramp,
        "r_squared": r_squared,
        "rate_cap_cfg": rate_cap_cfg,
        "rc_cfg": rc_cfg,
        "rms_rate": rms_rate,
        "semantics": semantics,
        "speed": speed,
        "ss_t0": ss_t0,
        "ss_t1": ss_t1,
        "t_control_on": t_control_on,
        "t_imu": t_imu,
        "t_ns": t_ns,
        "track_act": track_act,
        "track_err": track_err,
        "track_is_ang": track_is_ang,
        "track_ratecmd": track_ratecmd,
        "track_tgt": track_tgt,
        "track_tw": track_tw,
        "v_mean": v_mean,
        "win_end": win_end,
        "wps": wps,
        "track_cmd": track_cmd,
        "kp_angle_cfg": kp_angle_cfg,
        "rate_setpt": rate_setpt,
        "claimed_mode": claimed_mode,
        "launch_t": launch_t,
        "warnings": warnings,
    }


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="roll_pid", title="Roll PID Tracking & Tuning")

    s = roll_series(flight)
    if "abort" in s:
        result.warnings.append(s["abort"])
        return result
    result.warnings.extend(s["warnings"])
    K_plant = s["K_plant"]
    angle_peak = s["angle_peak"]
    angle_rms = s["angle_rms"]
    burnout_t = s["burnout_t"]
    cmd = s["cmd"]
    eject_baro = s["eject_baro"]
    eject_t = s["eject_t"]
    fft_freqs = s["fft_freqs"]
    fft_mag = s["fft_mag"]
    flown_mode_note = s["flown_mode_note"]
    g = s["g"]
    has_angle_profile = s["has_angle_profile"]
    has_wp_angles = s["has_wp_angles"]
    osc_freq = s["osc_freq"]
    peak_rate = s["peak_rate"]
    profile_ramp = s["profile_ramp"]
    r_squared = s["r_squared"]
    rate_cap_cfg = s["rate_cap_cfg"]
    rc_cfg = s["rc_cfg"]
    rms_rate = s["rms_rate"]
    semantics = s["semantics"]
    speed = s["speed"]
    ss_t0 = s["ss_t0"]
    ss_t1 = s["ss_t1"]
    t_control_on = s["t_control_on"]
    t_imu = s["t_imu"]
    t_ns = s["t_ns"]
    track_act = s["track_act"]
    track_err = s["track_err"]
    track_is_ang = s["track_is_ang"]
    track_ratecmd = s["track_ratecmd"]
    track_tgt = s["track_tgt"]
    track_tw = s["track_tw"]
    v_mean = s["v_mean"]
    win_end = s["win_end"]
    wps = s["wps"]
    track_cmd = s["track_cmd"]
    kp_angle_cfg = s["kp_angle_cfg"]
    rate_setpt = s["rate_setpt"]
    claimed_mode = s["claimed_mode"]
    launch_t = s["launch_t"]


    # ── Current gains (from sidecar if available) ──
    kp_flight, ki_flight, kd_flight = _gain_from_sidecar(flight.sidecar)
    kp_angle_flight = rc_cfg.get("kp_angle")

    tune = tuning(K_plant, kp_flight)
    pm_current = tune["pm_current"]
    f_c_current = tune["f_c_current"]
    kp_new, ki_new, kd_new = tune["kp_new"], tune["ki_new"], tune["kd_new"]
    if pm_current is not None and pm_current < 30:
        result.warnings.append(
            f"Low phase margin with flown gains: {pm_current:.0f}° "
            f"— oscillation expected."
        )

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
    if wps:
        metrics["profile_semantics"] = semantics
    if has_wp_angles and not has_angle_profile:
        # Profile stored on the FC but not followed (rate mode flew).
        metrics["profile_stored_NOT_followed"] = _effective_plan(wps, semantics)
    if has_angle_profile:
        metrics["profile_commanded"] = _effective_plan(wps, semantics)
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
            eps = min(0.02, 0.25 * (b - a))
            tgt_a, is_ang_a = _profile_target(wps, a + eps, semantics)
            tgt_b, _ = _profile_target(wps, b - eps, semantics)
            _, is_ang_mid = _profile_target(wps, mid, semantics)
            if abs(float(_wrap180(tgt_b - tgt_a))) < 0.5:
                cmd_desc = f"cmd {tgt_a:g}°"
            else:
                cmd_desc = f"ramp {tgt_a:.0f}→{tgt_b:.0f}°"
            mi = (t_imu >= a) & (t_imu <= b)
            mc = (t_ns >= a) & (t_ns <= b)
            rate_rms_seg = float(np.sqrt(np.mean(g[mi] ** 2))) if mi.any() else float("nan")
            cmd_max_seg = float(np.max(np.abs(cmd[mc]))) if mc.any() else float("nan")
            if is_ang_mid and track_tw is not None:
                ms = (track_tw >= a) & (track_tw <= b) & track_is_ang
                e_seg = track_err[ms] if ms.any() else np.array([])
                e_seg = e_seg[np.isfinite(e_seg)]
                if e_seg.size:
                    desc = (f"{cmd_desc}: err {e_seg[0]:+.0f}→{e_seg[-1]:+.0f}° "
                            f"(RMS {np.sqrt(np.mean(e_seg**2)):.0f}°), ")
                else:
                    desc = f"{cmd_desc}: "
            else:
                desc = "null-rate: "
            desc += f"rate RMS {rate_rms_seg:.0f}°/s, |fin cmd| max {cmd_max_seg:.1f}°"
            metrics[f"segment {a:.2g}–{b:.2g}s"] = desc
    result.metrics = metrics

    # ── Charts ──
    # The "control on" instant is drawn on every panel. event_shapes styles the
    # keys it knows (launch/burnout/…) and falls back to gray for the rest, so
    # module-specific events like this one need no registration.
    ev_on = {"control on": round(float(t_control_on), 3)}

    # 1) Full-timeline stack: rate, command, speed over the whole log, so the
    #    pre-launch idle and the post-apogee behavior stay visible. The
    #    launch→ejection close-up is the Roll Control section's four panels
    #    (angle flights) or the stack below (rate-only flights).
    t_all_lo = float(min(t_imu[0], t_ns[0]))
    t_all_hi = float(max(t_imu[-1], t_ns[-1]))
    pad = 0.02 * (t_all_hi - t_all_lo) or 1.0
    full_range = [t_all_lo - pad, t_all_hi + pad]

    raw = trace(t_imu, g, "Gyro X (raw)", _C_RATE, mode="lines+markers", width=0.7)
    rate_traces = []
    if raw:
        raw["opacity"] = 0.45   # thin + faded, as the static figure drew it
        rate_traces.append(raw)
    if t_imu.size > 50:
        g_sm = _moving_average(g, min(50, max(2, t_imu.size // 10)))
        # Pure line: the average is derived, not samples, so no markers.
        rate_traces.append(trace(t_imu, g_sm, "50-pt average", _C_RATE,
                                 mode="lines", width=2.0))
    rate_spec = chart(
        "chart-roll-pid-rate", "Roll PID — full timeline", rate_traces,
        x_title="", y_title="Roll rate", y_unit="°/s", events=ev_on, height=250,
    )
    if rate_spec:
        _hline(rate_spec, 0.0, "#999", dash="solid")
    cmd_spec = chart(
        "chart-roll-pid-cmd", "",
        [trace(t_ns, cmd, "Fin Command", _C_CMD, mode="lines+markers", width=1.2)],
        x_title="", y_title="Roll command", y_unit="°", events=ev_on, height=250,
    )
    if cmd_spec:
        _hline(cmd_spec, 0.0, "#999", dash="solid")
    speed_spec = chart(
        "chart-roll-pid-speed", "",
        [trace(t_ns, speed, "Speed (nav)", _C_SPEED, mode="lines+markers", width=1.2)],
        x_title="Time since launch (s)", y_title="Speed", y_unit="m/s",
        events=ev_on, height=250,
    )
    timeline = _stack([rate_spec, cmd_spec, speed_spec], full_range)
    if timeline and has_angle_profile:
        timeline[-1]["note"] = (
            "Angle tracking, error, commanded rate and fin command for the "
            "launch→ejection window are the Roll Control section's four panels "
            "above; the table here carries the numbers behind them."
        )
    result.charts.extend(timeline)

    # 2) Control close-up — launch → ejection (roll rate + servo command).
    #    For angle-profile flights the Roll Control section's four panels are
    #    this view and more, so it renders only for rate-only flights. The
    #    static figure also drew a gray line per profile waypoint; on a
    #    rate-only flight a stored waypoint plan was by definition not
    #    followed, so those lines are dropped rather than ported.
    if not has_angle_profile:
        x_lo, x_hi = -0.2, win_end + 0.2
        xpad = 0.05 * (x_hi - x_lo)
        ctl_range = [x_lo - xpad, x_hi + xpad]
        ev_ctl = dict(ev_on)
        if burnout_t is not None and np.isfinite(burnout_t) \
                and x_lo <= burnout_t <= x_hi:
            ev_ctl["burnout"] = round(float(burnout_t), 3)
        if np.isfinite(eject_t) and eject_t <= x_hi:
            ev_ctl["apogee/eject"] = round(float(eject_t), 3)
        win = (t_imu >= x_lo) & (t_imu <= x_hi)
        win_c = (t_ns >= x_lo) & (t_ns <= x_hi)
        notes: list[str] = []

        ctl_rate = chart(
            "chart-roll-pid-ctl-rate", "Roll control — launch → ejection",
            [trace(t_imu[win], g[win], "Gyro X", _C_RATE,
                   mode="lines+markers", width=1.2)],
            x_title="", y_title="Roll rate", y_unit="°/s",
            events=ev_ctl, height=270,
        )
        if ctl_rate:
            lim, peak = rate_axis_limit(t_imu[win], g[win], win_end, rate_cap_cfg)
            ctl_rate["layout"]["yaxis"]["range"] = [-lim, lim]
            _hline(ctl_rate, 0.0, "#999", dash="solid")
            if rate_cap_cfg:
                _hline(ctl_rate, float(rate_cap_cfg), "#999")
                _hline(ctl_rate, -float(rate_cap_cfg), "#999")
                notes.append(f"Dotted lines on the rate panel mark the "
                             f"±{rate_cap_cfg:g}°/s rate cap.")
            if peak > lim * 1.05:
                notes.append(f"Rate axis clipped to the controlled flight; "
                             f"peak |rate| was {peak:.0f}°/s.")

        ctl_cmd = chart(
            "chart-roll-pid-ctl-cmd", "",
            [trace(t_ns[win_c], cmd[win_c], "Fin Command", _C_CMD,
                   mode="lines+markers", width=1.4),
             trace(t_ns[win_c], speed[win_c], "Speed", _C_SPEED,
                   mode="lines", width=1.1, axis="y2")],
            x_title="Time since launch (s)", y_title="Roll command", y_unit="°",
            y2_title="Speed (m/s)", events=ev_ctl, height=270,
        )
        if ctl_cmd:
            _hline(ctl_cmd, 0.0, "#999", dash="solid")
            cw = cmd[win_c]
            cw = cw[np.isfinite(cw)]
            if cw.size:
                # Zoomed to the command's own range plus 5°: it lives in a few
                # degrees while its limit is ±20, and scaling to the limit
                # would flatten everything the controller did.
                lo, hi = float(np.min(cw)), float(np.max(cw))
                ctl_cmd["layout"]["yaxis"]["range"] = [lo - 5.0, hi + 5.0]
                cmd_note = f"Fin command spans [{lo:.1f}, {hi:.1f}]°"
                limit = _as_float(rc_cfg.get("cmd_limit_max_deg"))
                if limit:
                    drawn = [r for r in (limit, -limit) if lo - 5.0 <= r <= hi + 5.0]
                    for rail in drawn:
                        _hline(ctl_cmd, rail, "#999")
                    if drawn:
                        cmd_note += f" against a ±{limit:g}° limit"
                notes.append(cmd_note + ".")

        ctl = _stack([ctl_rate, ctl_cmd], ctl_range, dtick=1.0)
        if ctl:
            ctl[-1]["note"] = " ".join(notes)
        result.charts.extend(ctl)

    # 3) Spectrum of the steady-state roll rate, on a log amplitude axis. This
    #    is where a limit-cycle oscillation shows as a spike and broadband
    #    airframe vibration as a floor — the dominant_osc_hz metric's evidence.
    if fft_freqs is not None and fft_mag is not None:
        fft_t = _fft_trace(fft_freqs, fft_mag)
        if fft_t:
            fft_spec = chart(
                "chart-roll-pid-fft",
                f"Roll-rate spectrum, steady-state window ({ss_t0:.1f}–{ss_t1:.1f} s)",
                [fft_t],
                x_title="Frequency (Hz)", y_title="Amplitude", y_unit="°/s",
                height=320,
            )
            if fft_spec:
                fft_spec["layout"]["yaxis"]["type"] = "log"
                # The window shows the control band; the full spectrum (out to
                # Nyquist) is shipped, and Autoscale reveals it.
                fft_spec["layout"]["xaxis"]["range"] = [0.0, 50.0]
                if np.isfinite(osc_freq):
                    fft_spec["layout"].setdefault("shapes", []).append({
                        "type": "line", "xref": "x", "yref": "paper",
                        "x0": float(osc_freq), "x1": float(osc_freq),
                        "y0": 0, "y1": 1,
                        "line": {"color": _C_PEAK, "width": 1, "dash": "dash"},
                    })
                    fft_spec["layout"].setdefault("annotations", []).append({
                        "x": float(osc_freq), "y": 1.0,
                        "xref": "x", "yref": "paper",
                        "text": f"peak {osc_freq:.1f} Hz", "showarrow": False,
                        "font": {"size": 10, "color": _C_PEAK},
                        "yanchor": "bottom",
                    })
                fft_spec["note"] = ("Log amplitude, windowed to 0–50 Hz; "
                                    "Autoscale shows the full spectrum.")
                result.charts.append(fft_spec)

    return result
