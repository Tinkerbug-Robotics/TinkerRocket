"""PN guidance performance module.

Decodes GuidanceTelemData (GUIDANCE_TELEM_MSG 0xCA) from the binary log and
characterises the guided-coast phase: when guidance engaged (vs the shared
activation delay and burnout), the commanded pitch/yaw fin deflections, the PN
acceleration command, and the line-of-sight / closing-velocity / lateral-offset
geometry — all aligned to flight time (T=0 at launch).

Flags the target-too-low terminal singularity: a large fin/accel command driven
from a near-zero lateral offset as the vehicle transits a low overhead aim-point
(see project notes). Skips gracefully on flights with no guidance frames.

Sits below the roll-PID section; roll control and guidance run concurrently in
the firmware (shared activation delay).
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

from ..flight import Flight
from ..registry import AnalysisResult

# Below this airspeed the firmware drops guidance (config::PN_MIN_SPEED_MPS is
# app-tunable; this is only used to *label* a likely deactivation cause).
_SPEED_FLOOR_HINT_MPS = 20.0


def _launch_time_s(ns: list, t0: int) -> float | None:
    for r in ns:
        if r.get("launch"):
            return (r["time_us"] - t0) / 1e6
    return None


def _at_time(t_query, t_series, y_series):
    """Interpolate y(t_query) from a (possibly unsorted) NonSensor series."""
    order = np.argsort(t_series, kind="stable")
    return float(np.interp(t_query, t_series[order], y_series[order]))


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="guidance", title="Guidance (PN) Performance")
    recs = flight.records
    t0 = flight.t0_us
    g = recs.get("Guidance") or []
    ns = recs.get("NonSensor") or []

    if not g:
        result.warnings.append("No guidance frames (GUIDANCE_TELEM_MSG) — not a guidance flight.")
        return result
    if not ns:
        result.warnings.append("Need NonSensor records to align guidance to flight time.")
        return result

    sidecar = flight.sidecar
    rc = (sidecar.get("settings") or {}).get("roll_control") or {}
    servo = (sidecar.get("settings") or {}).get("servo") or {}
    delay_ms = rc.get("delay_ms")
    burnout_t = sidecar.get("burnout_time_s")
    burnout_t = float(burnout_t) if burnout_t not in (None, "") else None
    apogee_t = sidecar.get("apogee_time_s")
    apogee_t = float(apogee_t) if apogee_t not in (None, "") else None
    fin_max = servo.get("fin_max_deg")

    # ── Launch-relative time bases ──
    launch_t = _launch_time_s(ns, t0)
    if launch_t is None:
        launch_t = (g[0]["time_us"] - t0) / 1e6  # fall back to first guidance frame
    tg = (get_array(g, "time_us") - t0) / 1e6 - launch_t

    t_ns = (get_array(ns, "time_us") - t0) / 1e6 - launch_t
    alt = get_array(ns, "u_pos")
    speed_ns = np.sqrt(get_array(ns, "e_vel") ** 2 +
                       get_array(ns, "n_vel") ** 2 +
                       get_array(ns, "u_vel") ** 2)
    roll_cmd = get_array(ns, "roll_cmd")

    # ── Guidance channels ──
    pitch_fin = get_array(g, "pitch_fin_cmd")
    yaw_fin = get_array(g, "yaw_fin_cmd")
    los = get_array(g, "los_angle")
    vcl = get_array(g, "closing_vel")
    latoff = get_array(g, "lateral_offset")
    a_n = get_array(g, "accel_cmd_n")
    a_e = get_array(g, "accel_cmd_e")
    a_mag = np.hypot(a_n, a_e)

    # Legacy 15-byte logs have no fin-command fields (decoded as None → object array).
    has_fin = bool(g[0].get("pitch_fin_cmd") is not None)

    # ── Rate / window ──
    dt = np.diff(tg)
    rate_hz = float(1.0 / np.median(dt)) if dt.size and np.median(dt) > 0 else float("nan")
    t_engage, t_cutoff = float(tg.min()), float(tg.max())

    alt_engage = _at_time(t_engage, t_ns, alt)
    alt_cutoff = _at_time(t_cutoff, t_ns, alt)
    spd_cutoff = _at_time(t_cutoff, t_ns, speed_ns)

    # ── Deactivation-cause heuristic ──
    lat_cutoff = float(latoff[-1])
    fin_peak = float(max(np.max(np.abs(pitch_fin)), np.max(np.abs(yaw_fin)))) if has_fin else float("nan")
    before_apogee = (apogee_t is not None) and (t_cutoff < apogee_t - 1.0)
    # Singularity signature: fin command pinned high while lateral offset ~0,
    # cutoff well before apogee (vehicle transited a low overhead target,
    # range→0 → 1/range blow-up).
    singular = (before_apogee and abs(lat_cutoff) < 1.0 and has_fin and
                fin_max is not None and fin_peak >= 0.8 * float(fin_max))
    if spd_cutoff <= _SPEED_FLOOR_HINT_MPS and not singular:
        deact = f"airspeed floor (~{spd_cutoff:.0f} m/s, pn_min_speed)"
    elif singular:
        deact = "CPA at low overhead target (range→0 singularity)"
    elif apogee_t is not None and t_cutoff >= apogee_t - 1.0:
        deact = "near apogee"
    else:
        deact = f"CPA / other (alt {alt_cutoff:.0f} m, {spd_cutoff:.0f} m/s)"

    # ── Metrics ──
    metrics: dict[str, object] = {
        "guidance_frames": len(g),
        "log_rate_hz": round(rate_hz) if np.isfinite(rate_hz) else "—",
        "active_window_s": f"T+{t_engage:.2f} .. T+{t_cutoff:.2f}  ({t_cutoff - t_engage:.2f} s)",
        "engaged_after_launch_ms": round(t_engage * 1000),
        "activation_delay_ms": delay_ms if delay_ms is not None else "—",
    }
    if burnout_t is not None:
        metrics["engaged_vs_burnout"] = (
            f"during boost (burnout T+{burnout_t:.2f})" if t_engage < burnout_t
            else f"post-burnout (burnout T+{burnout_t:.2f})")
    metrics["engage_alt_m"] = round(alt_engage, 1)
    metrics["cutoff_alt_m"] = round(alt_cutoff, 1)
    metrics["cutoff_speed_mps"] = round(spd_cutoff, 1)
    metrics["deactivation_cause"] = deact
    if has_fin:
        metrics["pitch_fin_cmd_deg"] = f"[{pitch_fin.min():+.1f}, {pitch_fin.max():+.1f}]"
        metrics["yaw_fin_cmd_deg"] = f"[{yaw_fin.min():+.1f}, {yaw_fin.max():+.1f}]"
        metrics["peak_fin_cmd_deg"] = round(fin_peak, 1)
        if fin_max is not None:
            metrics["fin_limit_deg"] = fin_max
    else:
        metrics["fin_cmd"] = "— (legacy 15-byte log, no fin-command fields)"
    metrics["pn_accel_cmd_mps2"] = f"[{a_mag.min():.1f}, {a_mag.max():.1f}]"
    metrics["los_angle_deg"] = f"[{los.min():.1f}, {los.max():.1f}]"
    metrics["closing_vel_mps"] = f"[{vcl.min():.1f}, {vcl.max():.1f}]"
    metrics["lateral_offset_m"] = f"{float(latoff[0]):.1f} → {float(latoff[-1]):.1f}"
    metrics["max_lateral_offset_m"] = round(float(np.max(np.abs(latoff))), 2)

    # Lateral-drift effectiveness: slope of the offset over the first vs last
    # 0.5 s of guidance. If the late rate < early rate, guidance is slowing the
    # drift; if it grows, guidance is not arresting the lateral motion.
    def _drift_rate(t_lo, t_hi):
        m = (tg >= t_lo) & (tg <= t_hi)
        if m.sum() >= 3:
            return float(np.polyfit(tg[m], latoff[m], 1)[0])
        return float("nan")
    rate_early = _drift_rate(t_engage, t_engage + 0.5)
    rate_late = _drift_rate(t_cutoff - 0.5, t_cutoff)
    if np.isfinite(rate_early) and np.isfinite(rate_late):
        verdict = ("slowing drift" if rate_late < rate_early - 0.2
                   else "not arresting drift" if rate_late > 0.2
                   else "drift held")
        metrics["drift_rate_early_late_mps"] = (
            f"{rate_early:+.1f} → {rate_late:+.1f}  ({verdict})")
    if rc.get("guidance_enabled") is not None:
        metrics["guidance_enabled"] = rc.get("guidance_enabled")
    result.metrics = metrics

    # ── Warnings ──
    if singular:
        result.warnings.append(
            f"Command-saturation signature: peak fin cmd {fin_peak:.0f}° "
            f"(limit {fin_max}°) from a near-zero lateral offset, cutoff at "
            f"{alt_cutoff:.0f} m — well below apogee. Consistent with the PN "
            f"terminal singularity when the overhead target altitude is set "
            f"below apogee (vehicle transits the aim-point, range→0).")
    if has_fin and fin_max is not None and fin_peak >= float(fin_max):
        result.warnings.append(
            f"Pitch/yaw fin command saturated at the {fin_max}° servo limit.")

    # ── Figure 1: control output + guidance geometry/dynamics ──
    pad = 0.3
    win = (t_ns >= t_engage - pad) & (t_ns <= t_cutoff + pad)

    def _shade(ax):
        ax.axvspan(t_engage, t_cutoff, color="tab:green", alpha=0.10,
                   label="guidance active")
        if burnout_t is not None and t_engage - pad <= burnout_t <= t_cutoff + pad:
            ax.axvline(burnout_t, color="tab:purple", ls=":", lw=1.0, label="burnout")

    fig, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=True)

    # Panel 0 — control output: commanded fin deflections + roll cmd
    ax = axes[0]; _shade(ax)
    if has_fin:
        ax.plot(tg, pitch_fin, color="tab:red", lw=1.3, label="pitch fin cmd")
        ax.plot(tg, yaw_fin, color="tab:green", lw=1.3, label="yaw fin cmd")
    ax.plot(t_ns[win], roll_cmd[win], color="tab:orange", lw=0.9, alpha=0.7,
            label="roll cmd (NonSensor)")
    if fin_max is not None and has_fin and fin_peak >= 0.5 * float(fin_max):
        ax.axhline(float(fin_max), color="0.5", ls="--", lw=0.8)
        ax.axhline(-float(fin_max), color="0.5", ls="--", lw=0.8,
                   label=f"servo limit ±{fin_max}°")
    ax.set_ylabel("Commanded deflection (deg)")
    ax.set_title("Control output — guidance fin commands (pre-mix) + roll command")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 1 — geometry: LOS angle + lateral offset
    ax = axes[1]; _shade(ax)
    ax.plot(tg, los, color="tab:blue", lw=1.3, label="LOS angle (deg)")
    ax.set_ylabel("LOS angle (deg)", color="tab:blue")
    ax.tick_params(axis="y", labelcolor="tab:blue")
    axb = ax.twinx()
    axb.plot(tg, latoff, color="tab:purple", lw=1.3, label="lateral offset (m)")
    axb.set_ylabel("Lateral offset (m)", color="tab:purple")
    axb.tick_params(axis="y", labelcolor="tab:purple")
    ax.set_title("Guidance geometry — line-of-sight angle & lateral offset")
    ax.grid(True, alpha=0.3)

    # Panel 2 — dynamics: closing velocity + PN accel command magnitude
    ax = axes[2]; _shade(ax)
    ax.plot(tg, vcl, color="0.35", lw=1.3, label="closing velocity (m/s)")
    ax.set_ylabel("Closing velocity (m/s)", color="0.35")
    ax.set_xlabel("Time since launch (s)")
    axb = ax.twinx()
    axb.plot(tg, a_mag, color="tab:red", lw=1.3, label="|PN accel cmd| (m/s²)")
    axb.set_ylabel("|PN accel cmd| (m/s²)", color="tab:red")
    axb.tick_params(axis="y", labelcolor="tab:red")
    ax.set_title("Closing velocity & PN acceleration command")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    result.figures.append(fig)

    # ── Figure 2: ground-track drift (was guidance arresting the lateral drift?) ──
    e_pos = get_array(ns, "e_pos")
    n_pos = get_array(ns, "n_pos")
    gmask = (t_ns >= t_engage) & (t_ns <= t_cutoff)
    if gmask.sum() > 5:
        # Sorted NonSensor series for interpolating position at tick times.
        order = np.argsort(t_ns, kind="stable")
        ts_s, es_s, ns_s = t_ns[order], e_pos[order], n_pos[order]

        def _pos(tq):
            return (float(np.interp(tq, ts_s, es_s)),
                    float(np.interp(tq, ts_s, ns_s)))

        fig2, ax = plt.subplots(figsize=(8, 8))
        # Pre-guidance track (launch → guidance ON) then the guided segment, so
        # you can see the drift that was already present when guidance engaged.
        pre = (t_ns >= 0.0) & (t_ns <= t_engage)
        if pre.sum() > 1:
            ax.plot(e_pos[pre], n_pos[pre], color="0.7", lw=1.2,
                    label="pre-guidance (launch→ON)")
        ax.plot(e_pos[gmask], n_pos[gmask], color="tab:green", lw=2.2,
                label="guidance active")
        # Overhead target is straight up over the pad → origin in the E-N plane.
        ax.scatter([0], [0], marker="+", s=170, color="k", lw=1.6,
                   label="pad / overhead target")

        # 0.5 s ticks along the guided segment: tick SPACING = lateral drift
        # speed, so even spacing ⇒ constant drift (guidance not arresting it),
        # bunching ⇒ drift slowing.
        first_tick = np.ceil(t_engage / 0.5) * 0.5
        for tt in np.arange(first_tick, t_cutoff, 0.5):
            ex, nx = _pos(tt)
            ax.scatter([ex], [nx], color="tab:green", s=16, zorder=5)
            ax.annotate(f"{tt:.1f}s", (ex, nx), fontsize=7, color="0.3",
                        xytext=(4, 0), textcoords="offset points", va="center")

        # ON / OFF annotations with the lateral offset at each end.
        ex0, nx0 = _pos(t_engage)
        ex1, nx1 = _pos(t_cutoff)
        ax.annotate(
            f"guidance ON\nT+{t_engage:.2f} s · offset {float(latoff[0]):.1f} m",
            (ex0, nx0), xytext=(24, 12), textcoords="offset points", fontsize=8,
            color="tab:green", ha="left",
            arrowprops=dict(arrowstyle="->", color="tab:green"),
            bbox=dict(boxstyle="round", fc="white", ec="tab:green", alpha=0.9))
        ax.annotate(
            f"guidance OFF\nT+{t_cutoff:.2f} s · offset {float(latoff[-1]):.1f} m",
            (ex1, nx1), xytext=(24, -28), textcoords="offset points", fontsize=8,
            color="tab:red", ha="left",
            arrowprops=dict(arrowstyle="->", color="tab:red"),
            bbox=dict(boxstyle="round", fc="white", ec="tab:red", alpha=0.9))

        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_title("Ground track — lateral drift during guided coast\n"
                     "(0.5 s ticks; even spacing ⇒ drift not arrested)")
        ax.axis("equal")
        ax.legend(loc="best", fontsize=8)
        ax.grid(True, alpha=0.3)
        fig2.tight_layout()
        result.figures.append(fig2)

    return result
