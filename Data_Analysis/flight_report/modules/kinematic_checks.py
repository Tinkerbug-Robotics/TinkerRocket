"""Kinematic-checks module.

Replays the flight through `sim_kinematic_checks.KinematicChecks` (the Python
port of TR_KinematicChecks) and compares when each flag fires vs when the
firmware fired it (logged in NonSensor).

Also renders two voting-timeline figures (when data permits):
  * Apogee: which sub-detector (vel / baro / gps / pitch) was passing at each
    tick + when each crossed its hysteresis threshold to fire.
  * Landing: when each landing sub-detector latched (impact / baro_stable /
    gyro_quiet / gps_stationary / accel_1g). Older flights predate the
    landing voting system; the figure is skipped if nothing fired.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

try:
    from replay_kinematic_checks import (  # noqa: E402
        accel_norm_firmware,
        estimate_ground_pressure,
        build_events,
        logged_flag_times,
        pressure_to_altitude_firmware,
        BARO_MACH_LOCKOUT_ON,
        BARO_MACH_LOCKOUT_OFF,
        NSF_BURNOUT,
    )
    from sim_kinematic_checks import KinematicChecks  # noqa: E402
    _IMPORT_OK = True
    _IMPORT_ERROR = ""
except ImportError as e:
    _IMPORT_OK = False
    _IMPORT_ERROR = str(e)

from ..flight import Flight
from ..registry import AnalysisResult


_APOGEE_FLAGS = (
    "launch_flag", "vel_u_apogee_flag", "alt_apogee_flag",
    "gps_apogee_flag", "pitch_apogee_flag", "apogee_flag", "alt_landed_flag",
)
_LANDING_SUBFLAGS = (
    "impact_flag", "baro_stable_flag", "gyro_quiet_flag",
    "gps_stationary_flag", "accel_1g_flag",
)


def _periods(times, states):
    """Collapse a per-tick bool sequence into list of (t_start, t_end) runs."""
    out = []
    in_run = False
    run_start = 0.0
    prev_t = 0.0
    for t, s in zip(times, states):
        if s and not in_run:
            in_run = True
            run_start = t
        elif not s and in_run:
            in_run = False
            out.append((run_start, prev_t))
        prev_t = t
    if in_run:
        out.append((run_start, prev_t))
    return out


def _replay(records):
    """Drive KinematicChecks through the merged event stream.

    Returns a dict with all the data the voting plots need. Mirrors the
    capture pattern in `replay_kinematic_checks.replay`.
    """
    if not records["ISM6HG256"]:
        return None

    t0_us = records["ISM6HG256"][0]["time_us"]
    ground_pa = estimate_ground_pressure(records["BMP585"], records["NonSensor"])

    kc = KinematicChecks()

    latest_palt = 0.0
    latest_acc_mag = 0.0
    latest_pos = (0.0, 0.0, 0.0)
    latest_vel = (0.0, 0.0, 0.0)
    latest_roll_rate = 0.0
    latest_gps_alt = 0.0
    latest_gps_vel_u = 0.0
    latest_pitch_rad = math.pi / 2
    burnout = False
    mach_locked_out = False
    new_baro = False
    new_gps = False
    gvu_t: list[float] = []   # GNSS Doppler vert-velocity (for true-apogee xing)
    gvu_v: list[float] = []

    sim_fires: dict[str, float] = {}
    raw_palt_t: list[float] = []
    raw_palt_v: list[float] = []
    sim_alt_t: list[float] = []
    sim_alt_v: list[float] = []
    pass_t: list[float] = []
    pass_vel: list[bool] = []
    pass_baro: list[bool] = []
    pass_pitch: list[bool] = []
    gps_pass_t: list[float] = []
    gps_pass_v: list[bool] = []
    baro_reject_t: list[float] = []

    for t_us, kind, r in build_events(records):
        now_ms = (t_us - t0_us) // 1000
        now_s = now_ms / 1000.0

        if kind == "baro":
            latest_palt = pressure_to_altitude_firmware(r["pressure_pa"], ground_pa)
            raw_palt_t.append((t_us - t0_us) / 1e6)
            raw_palt_v.append(latest_palt)
            new_baro = True
            continue
        if kind == "gnss":
            latest_gps_alt = float(r["alt_m"])
            latest_gps_vel_u = float(r["vel_u"])
            new_gps = True
            gps_pass_t.append((t_us - t0_us) / 1e6)
            gvu_t.append((t_us - t0_us) / 1e6)
            gvu_v.append(latest_gps_vel_u)
            continue
        if kind == "ns":
            latest_pos = (r["e_pos"], r["n_pos"], r["u_pos"])
            latest_vel = (r["e_vel"], r["n_vel"], r["u_vel"])
            latest_pitch_rad = math.radians(r["pitch"])
            burnout = bool(r["flags"] & NSF_BURNOUT)
            speed = math.sqrt(r["e_vel"]**2 + r["n_vel"]**2 + r["u_vel"]**2)
            if not mach_locked_out and speed > BARO_MACH_LOCKOUT_ON:
                mach_locked_out = True
            elif mach_locked_out and speed < BARO_MACH_LOCKOUT_OFF:
                mach_locked_out = False
            continue

        # imu tick — drives kc.kinematic_checks()
        low = (r["low_acc_x"], r["low_acc_y"], r["low_acc_z"])
        high = (r["high_acc_x"], r["high_acc_y"], r["high_acc_z"])
        latest_acc_mag = accel_norm_firmware(low, high)
        latest_roll_rate = r["gyro_x"]

        prev_rejects = kc._consec_baro_rejects
        kc.kinematic_checks(
            pressure_altitude=latest_palt,
            acc_mag=latest_acc_mag,
            position=latest_pos,
            velocity=latest_vel,
            roll_rate=latest_roll_rate,
            new_baro=new_baro,
            gps_altitude=latest_gps_alt,
            gps_vel_u=latest_gps_vel_u,
            new_gps=new_gps,
            pitch_rad=latest_pitch_rad,
            burnout_detected=burnout,
            baro_locked_out=mach_locked_out,
            now_ms=now_ms,
        )
        if kc._consec_baro_rejects > prev_rejects:
            baro_reject_t.append(now_s)
        new_baro_was = new_baro
        new_gps_was = new_gps
        new_baro = False
        new_gps = False

        # ── Capture per-tick state ────────────────────────────────────────
        sim_alt_t.append(now_s)
        sim_alt_v.append(float(kc.alt_est))
        pass_t.append(now_s)
        pass_vel.append(bool(kc.last_vel_pass))
        pass_baro.append(bool(kc.last_baro_pass))
        pass_pitch.append(bool(kc.last_pitch_pass))
        # GPS pass is only meaningful on new_gps ticks (one entry per gnss).
        if new_gps_was and len(gps_pass_t) > len(gps_pass_v):
            gps_pass_v.append(bool(kc.last_gps_pass))

        # First-fire snapshots
        for name in _APOGEE_FLAGS:
            if getattr(kc, name) and name not in sim_fires:
                sim_fires[name] = now_s
        for name in _LANDING_SUBFLAGS:
            if getattr(kc, name, False) and name not in sim_fires:
                sim_fires[name] = now_s

    # Align gps_pass arrays in case the final IMU tick happened before a
    # final GNSS sample's pass value got captured.
    if len(gps_pass_v) < len(gps_pass_t):
        gps_pass_t = gps_pass_t[: len(gps_pass_v)]

    return {
        "sim_fires":   sim_fires,
        "raw_palt":    (raw_palt_t, raw_palt_v),
        "sim_alt":     (sim_alt_t, sim_alt_v),
        "baro_reject_t": baro_reject_t,
        "vel_periods":   _periods(pass_t, pass_vel),
        "baro_periods":  _periods(pass_t, pass_baro),
        "pitch_periods": _periods(pass_t, pass_pitch),
        "gps_periods":   _periods(gps_pass_t, gps_pass_v),
        "max_altitude":  float(kc.max_altitude),
        "max_speed":     float(kc.max_speed),
        # True apogee = GNSS Doppler vertical-velocity zero-crossing (after
        # ascent).  Trusted over the raw-baro peak, which lags GNSS by 0.3-1.3 s
        # and is spike-prone (#112).  None when there is no usable GNSS.
        "true_apogee_t": _gnss_vel_zero_apogee(gvu_t, gvu_v),
    }


def _gnss_vel_zero_apogee(t, vu):
    """Apogee = descending GNSS Doppler vert-velocity zero-crossing, gated to
    after the vehicle clearly ascended (vu>10 m/s) so on-pad noise is skipped.
    A 5-sample median de-spikes vu first."""
    if len(t) < 6:
        return None
    half = 2
    sm = [sorted(vu[max(0, i - half):min(len(vu), i + half + 1)])[
              (min(len(vu), i + half + 1) - max(0, i - half)) // 2]
          for i in range(len(vu))]
    ascended = False
    for i in range(1, len(sm)):
        if sm[i - 1] > 10.0:
            ascended = True
        if ascended and sm[i - 1] >= 0.0 and sm[i] < 0.0:
            denom = sm[i - 1] - sm[i]
            f = (sm[i - 1] / denom) if denom else 0.0
            return float(t[i - 1] + f * (t[i] - t[i - 1]))
    return None


# Map sim flag name -> logged flag name produced by logged_flag_times()
_FLAG_PAIRS = (
    ("launch_flag",       "launch"),
    ("vel_u_apogee_flag", "vel_apogee"),
    ("alt_apogee_flag",   "alt_apogee"),
    ("gps_apogee_flag",   "gps_apogee"),
    ("pitch_apogee_flag", "pitch_apogee"),
    ("apogee_flag",       "apogee_flag"),
    ("alt_landed_flag",   "alt_landed"),
)


# ── Figures ──────────────────────────────────────────────────────────────────

_DET_STYLE = {
    "vel":   ("Vel",   "tab:blue"),
    "baro":  ("Baro",  "tab:green"),
    "gps":   ("GPS",   "tab:orange"),
    "pitch": ("Pitch", "tab:purple"),
}
_DET_FIRE_FLAG = {
    "vel":   "vel_u_apogee_flag",
    "baro":  "alt_apogee_flag",
    "gps":   "gps_apogee_flag",
    "pitch": "pitch_apogee_flag",
}


def _plot_apogee_voting(res, title_suffix=""):
    t_raw, v_raw = res["raw_palt"]
    t_sim, v_sim = res["sim_alt"]
    if not t_sim:
        return None

    fig = plt.figure(figsize=(12, 6), constrained_layout=True)
    gs = GridSpec(2, 1, figure=fig, height_ratios=[3, 1])
    ax_alt = fig.add_subplot(gs[0])
    ax_pass = fig.add_subplot(gs[1], sharex=ax_alt)

    # Altitude pane
    if t_raw:
        ax_alt.plot(t_raw, v_raw, color="lightgray", lw=0.7, label="raw palt")
    ax_alt.plot(t_sim, v_sim, color="black", lw=0.9, label="KF alt_est")

    # True apogee = GNSS Doppler vel=0 (trusted).  Fall back to the raw-baro
    # peak only when there is no GNSS; show the baro peak separately so its
    # lag vs GNSS (#112) is visible rather than mistaken for the truth.
    true_t = res.get("true_apogee_t")
    if true_t is not None:
        ax_alt.axvline(true_t, color="black", ls=":", lw=1.4,
                       label=f"true apogee (GNSS v=0) {true_t:.2f}s")
    if v_raw:
        peak_idx = max(range(len(v_raw)), key=lambda i: v_raw[i])
        baro_pk_t = t_raw[peak_idx]
        ax_alt.axvline(baro_pk_t, color="dimgray", ls=":", lw=0.8,
                       label=("baro peak (lags)" if true_t is not None
                              else "true apogee") + f" {baro_pk_t:.2f}s")

    # Sub-detector fire markers
    fire_marker = {"vel": "s", "baro": "^", "gps": "o", "pitch": "*"}
    for det_key, flag_name in _DET_FIRE_FLAG.items():
        t = res["sim_fires"].get(flag_name)
        if t is None:
            continue
        # Place marker at the kf-estimated altitude at that time
        i = min(range(len(t_sim)), key=lambda j: abs(t_sim[j] - t))
        y = v_sim[i]
        label, color = _DET_STYLE[det_key]
        ax_alt.scatter([t], [y], color=color, marker=fire_marker[det_key],
                       s=70, zorder=5, edgecolor="black", linewidth=0.6,
                       label=f"{label} fire {t:.2f}s")

    # Master apogee fire
    t_master = res["sim_fires"].get("apogee_flag")
    if t_master is not None:
        ax_alt.axvline(t_master, color="red", lw=1.4, alpha=0.7,
                       label=f"master {t_master:.2f}s")

    # Baro rejects
    if res["baro_reject_t"]:
        alt_floor = min(v_sim + v_raw) - 5 if v_raw else min(v_sim) - 5
        ax_alt.scatter(res["baro_reject_t"],
                       [alt_floor] * len(res["baro_reject_t"]),
                       color="red", marker="x", s=14, alpha=0.5,
                       label=f"baro reject (n={len(res['baro_reject_t'])})")

    ax_alt.set_ylabel("Altitude (m)")
    ax_alt.set_title(f"Apogee sub-detector voting{title_suffix}")
    ax_alt.grid(True, alpha=0.3)
    ax_alt.legend(loc="lower right", fontsize=7, ncol=2)
    plt.setp(ax_alt.get_xticklabels(), visible=False)

    # Pass-state pane (condition-true bands)
    det_rows = (
        ("vel",   res["vel_periods"]),
        ("baro",  res["baro_periods"]),
        ("gps",   res["gps_periods"]),
        ("pitch", res["pitch_periods"]),
    )
    for y_idx, (det_key, periods) in enumerate(det_rows):
        label, color = _DET_STYLE[det_key]
        y = len(det_rows) - 1 - y_idx
        for (t0, t1) in periods:
            ax_pass.hlines(y, t0, max(t1, t0 + 0.005), color=color,
                           lw=7, alpha=0.7)
        ax_pass.text(-0.01, y, label, transform=ax_pass.get_yaxis_transform(),
                     ha="right", va="center", fontsize=8, color=color)
    ax_pass.set_yticks([])
    ax_pass.set_ylim(-0.7, len(det_rows) - 0.3)
    ax_pass.set_xlabel("t (s)")
    ax_pass.set_ylabel("condition\ntrue", fontsize=8)
    ax_pass.grid(True, axis="x", alpha=0.3)

    # Zoom around the master apogee fire if we have one
    if t_master is not None and t_raw:
        ax_alt.set_xlim(max(0.0, t_master - 8.0), min(t_master + 5.0, t_raw[-1]))

    return fig


_LANDING_STYLE = {
    "impact_flag":         ("Impact (high-G)",   "tab:red"),
    "baro_stable_flag":    ("Baro stable",       "tab:green"),
    "gyro_quiet_flag":     ("Gyro quiet",        "tab:purple"),
    "gps_stationary_flag": ("GPS stationary",    "tab:orange"),
    "accel_1g_flag":       ("Accel ≈ 1g",        "tab:blue"),
}


def _plot_landing_voting(res):
    fires = res["sim_fires"]
    sublatches = [(name, fires[name]) for name in _LANDING_STYLE if name in fires]
    master_t = fires.get("alt_landed_flag")
    if not sublatches and master_t is None:
        return None

    t_raw, v_raw = res["raw_palt"]
    t_sim, v_sim = res["sim_alt"]
    if not t_sim:
        return None

    fig = plt.figure(figsize=(12, 6), constrained_layout=True)
    gs = GridSpec(2, 1, figure=fig, height_ratios=[3, 1])
    ax_alt = fig.add_subplot(gs[0])
    ax_pass = fig.add_subplot(gs[1], sharex=ax_alt)

    if t_raw:
        ax_alt.plot(t_raw, v_raw, color="lightgray", lw=0.7, label="raw palt")
    ax_alt.plot(t_sim, v_sim, color="black", lw=0.9, label="KF alt_est")

    # Sub-flag latch markers
    fire_marker = {"impact_flag": "X", "baro_stable_flag": "^",
                   "gyro_quiet_flag": "o", "gps_stationary_flag": "s",
                   "accel_1g_flag": "D"}
    for name, t in sublatches:
        i = min(range(len(t_sim)), key=lambda j: abs(t_sim[j] - t))
        y = v_sim[i]
        label, color = _LANDING_STYLE[name]
        ax_alt.scatter([t], [y], color=color, marker=fire_marker[name],
                       s=70, zorder=5, edgecolor="black", linewidth=0.6,
                       label=f"{label} {t:.2f}s")

    if master_t is not None:
        ax_alt.axvline(master_t, color="red", lw=1.4, alpha=0.7,
                       label=f"alt_landed {master_t:.2f}s")

    ax_alt.set_ylabel("Altitude (m)")
    ax_alt.set_title("Landing sub-detector voting")
    ax_alt.grid(True, alpha=0.3)
    ax_alt.legend(loc="upper right", fontsize=7, ncol=2)
    plt.setp(ax_alt.get_xticklabels(), visible=False)

    # Latched-state bands: each sub-flag is on from latch time to end
    end_t = t_sim[-1]
    rows = list(_LANDING_STYLE.items())
    for y_idx, (name, (label, color)) in enumerate(rows):
        y = len(rows) - 1 - y_idx
        if name in fires:
            ax_pass.hlines(y, fires[name], end_t, color=color, lw=7, alpha=0.7)
        ax_pass.text(-0.01, y, label, transform=ax_pass.get_yaxis_transform(),
                     ha="right", va="center", fontsize=8, color=color)
    ax_pass.set_yticks([])
    ax_pass.set_ylim(-0.7, len(rows) - 0.3)
    ax_pass.set_xlabel("t (s)")
    ax_pass.set_ylabel("latched", fontsize=8)
    ax_pass.grid(True, axis="x", alpha=0.3)

    # Zoom around landing if we have it
    if master_t is not None:
        ax_alt.set_xlim(max(0.0, master_t - 20.0), min(master_t + 5.0, end_t))

    return fig


# ── Entry point ──────────────────────────────────────────────────────────────

def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="kinematic_checks", title="Kinematic Checks Replay")

    if not _IMPORT_OK:
        result.warnings.append(f"Module unavailable: {_IMPORT_ERROR}")
        return result

    recs = flight.records
    if not recs.get("NonSensor"):
        result.warnings.append("No NonSensor records — nothing to compare against.")
        return result
    if not recs.get("ISM6HG256"):
        result.warnings.append("No IMU records — replay needs IMU ticks.")
        return result

    try:
        res = _replay(recs)
    except Exception as e:  # noqa: BLE001
        result.warnings.append(f"Replay failed: {type(e).__name__}: {e}")
        return result
    if res is None:
        result.warnings.append("Replay returned no data.")
        return result

    t0_us = recs["ISM6HG256"][0]["time_us"]
    logged = logged_flag_times(recs["NonSensor"], t0_us)
    sim = res["sim_fires"]

    metrics: dict[str, object] = {}
    for sim_name, log_name in _FLAG_PAIRS:
        sim_t = sim.get(sim_name)
        log_t = logged.get(log_name)
        metrics[f"{log_name}_logged_s"] = round(log_t, 3) if log_t is not None else "—"
        metrics[f"{log_name}_replay_s"] = round(sim_t, 3) if sim_t is not None else "—"
        if sim_t is not None and log_t is not None:
            dt = round(sim_t - log_t, 3)
            metrics[f"{log_name}_replay-logged_s"] = dt
            if abs(dt) > 0.5:
                result.warnings.append(
                    f"{log_name}: replay {dt:+.2f} s vs logged firmware decision."
                )
        else:
            metrics[f"{log_name}_replay-logged_s"] = "—"

    # Landing sub-flag latches (firmware does NOT log these for pre-#166
    # flights; the times here are the new sim's verdict on what *would* have
    # latched.)
    for name in _LANDING_SUBFLAGS:
        t = sim.get(name)
        metrics[f"sim_{name}_s"] = round(t, 3) if t is not None else "—"

    metrics["max_altitude_sim_m"] = round(res["max_altitude"], 2)
    metrics["max_speed_sim_mps"] = round(res["max_speed"], 2)
    metrics["baro_rejects"] = len(res["baro_reject_t"])
    result.metrics = metrics

    # ── Figures ─────────────────────────────────────────────────────────────
    # The apogee-voting figure moved to the flight report, where it is drawn
    # interactively by `modules/apogee.py` from this same replay. It is not
    # repeated here: the registry's rule is one thing in one place, and the
    # metrics above already carry the replay-vs-logged comparison it supported.
    fig = _plot_landing_voting(res)
    if fig is not None:
        result.figures.append(fig)

    return result
