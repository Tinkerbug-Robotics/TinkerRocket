#!/usr/bin/env python3
"""Live landing-point predictor for TinkerRocket flights.

Replays a binary flight log through the firmware EKF (`_ekf_replay`), then
at a chosen 'GPS-loss' time T_loss snapshots the EKF state and runs a
ballistic + drift-cast forward simulation in launch-relative ENU to
predict where the rocket lands.  Compares the prediction against the
actual landing recovered from the log.

This is the validation tool — once accuracy is acceptable, the same
algorithm gets ported to Swift (`LandingPredictor.swift`) and run live on
the iOS app.

Usage
-----
    python landing_predictor.py <flight_dir_or_bin> --loss-time 15.0
    python landing_predictor.py <flight_dir_or_bin> --sweep
"""

from __future__ import annotations

import sys
import math
import argparse
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "tinkerrocket-sim" / "src"))

from plot_flight_data_mini import parse_binary_file  # noqa: E402
from _ekf_replay import (replay_binary, ReplayResult, LaunchRef,  # noqa: E402
                          lla_rad_to_enu_m, G_MS2, DEG2RAD, RAD2DEG)
from _wind_profile import WindProfile, fetch_wind, KT_TO_MPS  # noqa: E402

FT_PER_M = 3.28084
M_PER_FT = 1.0 / FT_PER_M
LAYER_THICKNESS_FT = 1000.0    # matches DriftCastEngine.swift:280


# ---------------------------------------------------------------------------
# Rocket profile (mirrors RocketProfile fields planned for Swift)
# ---------------------------------------------------------------------------

@dataclass
class RocketProfile:
    drogue_rate_fps: float = 60.0
    main_rate_fps: float = 12.0
    main_deploy_alt_agl_ft: float = 700.0
    # Quadratic drag coefficient k (1/m) used during ascent ballistic propagation.
    # Default 5e-4 ≈ terminal velocity ~140 m/s, in the ballpark for typical
    # 54-65mm airframes.  Live-fit (fit_drag_k_from_coast) overrides this when
    # GNSS is healthy through coast — currently unreliable on legacy 1g GNSS
    # binaries, but will work on flights flown with the GNSS-4g fix (#176).
    ballistic_drag_k: float = 5e-4


# ---------------------------------------------------------------------------
# Snapshot + prediction
# ---------------------------------------------------------------------------

@dataclass
class EkfSnapshot:
    """EKF state at a single instant, expressed in launch-relative ENU."""
    t_s: float
    e_m: float
    n_m: float
    u_m: float
    ve_mps: float
    vn_mps: float
    vu_mps: float


@dataclass
class LandingPrediction:
    landing_e_m: float
    landing_n_m: float
    descent_track: list = field(default_factory=list)  # [(e, n, u, t_s)]
    phase: str = ""                                     # ASCENT / DESCENT_DROGUE / DESCENT_MAIN
    apogee_e_m: Optional[float] = None
    apogee_n_m: Optional[float] = None
    apogee_u_m: Optional[float] = None


def snapshot_at(result: ReplayResult, t_loss_s: float,
                prefer_gnss_for_descent: bool = True,
                gnss_fresh_threshold_s: float = 2.0) -> EkfSnapshot:
    """EKF state at `t_loss_s` with optional GNSS substitution for descent.

    By default, when (a) the EKF says we're descending (vu ≤ 0) and (b) a
    valid GNSS fix exists within `gnss_fresh_threshold_s` of t_loss_s, the
    snapshot position is taken from raw GNSS instead of the EKF.  This
    bypasses the EKF's slow Huber-down-weight recovery oscillation after
    a boost-phase GNSS gap (#176) — for ascent the EKF still wins because
    GNSS is usually missing.

    Velocity always comes from the EKF (smoother than raw GNSS, and the
    EKF velocity recovers faster than position after a gap).

    Set prefer_gnss_for_descent=False for a pure-EKF baseline comparison.
    """
    if len(result.ekf.t_us) == 0:
        raise RuntimeError("EKF series is empty")

    t_target_us = result.phases.t0_us + t_loss_s * 1e6
    ts = result.ekf.t_us
    if t_target_us <= ts[0]:
        idx = 0
        frac = 0.0
    elif t_target_us >= ts[-1]:
        idx = len(ts) - 2
        frac = 1.0
    else:
        idx = int(np.searchsorted(ts, t_target_us) - 1)
        idx = max(0, min(idx, len(ts) - 2))
        frac = (t_target_us - ts[idx]) / (ts[idx + 1] - ts[idx])

    def lerp(a, b):
        return float(a + (b - a) * frac)

    lat_rad = lerp(result.ekf.lat_rad[idx], result.ekf.lat_rad[idx + 1])
    lon_rad = lerp(result.ekf.lon_rad[idx], result.ekf.lon_rad[idx + 1])
    alt_m = lerp(result.ekf.alt_m[idx], result.ekf.alt_m[idx + 1])
    vn = lerp(result.ekf.vn[idx], result.ekf.vn[idx + 1])
    ve = lerp(result.ekf.ve[idx], result.ekf.ve[idx + 1])
    vd = lerp(result.ekf.vd[idx], result.ekf.vd[idx + 1])

    e, n, u = lla_rad_to_enu_m(lat_rad, lon_rad, alt_m, result.launch_ref)

    # GNSS substitution for descent (user spec: use GNSS for descent, EKF for ascent)
    if prefer_gnss_for_descent and -vd <= 0.5:
        latest = _latest_gnss_fix(result, t_loss_s, gnss_fresh_threshold_s)
        if latest is not None:
            g_e, g_n, g_u = lla_rad_to_enu_m(latest["lat_rad"], latest["lon_rad"],
                                              latest["alt_m"], result.launch_ref)
            e, n, u = g_e, g_n, g_u

    return EkfSnapshot(t_s=t_loss_s, e_m=e, n_m=n, u_m=u,
                       ve_mps=ve, vn_mps=vn, vu_mps=-vd)


def fit_drag_k_from_coast(result: ReplayResult, t_loss_s: float,
                          settle_s: float = 0.3,
                          min_samples: int = 20,
                          v_min_mps: float = 15.0
                          ) -> Optional[float]:
    """Fit isotropic quadratic drag coefficient k from coast-phase velocity.

    Uses the **vertical** channel only:  dv_u/dt = -g - k · |v| · v_u
    Horizontal velocity dead-reckons during the GNSS-out boost/coast, so
    its time derivative is dominated by IMU bias drift, not physics.  The
    EKF vertical velocity is baro-constrained and reliable.

    Window: [boost_end + settle_s, min(apogee, t_loss_s)] — drop the post-
    apogee samples (descent under chute is not coast).

    Causal: only uses data the predictor would have at T_loss.

    Returns None on insufficient data, low-v window, or implausible k.
    """
    if result.phases.boost_end_us is None:
        return None
    boost_end_s = (result.phases.boost_end_us - result.phases.t0_us) / 1e6
    t_start_s = boost_end_s + settle_s
    t_end_s = t_loss_s
    if result.phases.apogee_us is not None:
        apogee_s = (result.phases.apogee_us - result.phases.t0_us) / 1e6
        t_end_s = min(t_end_s, apogee_s)
    if t_end_s <= t_start_s + 0.2:
        return None

    ts = result.ekf.t_us
    t0_us = result.phases.t0_us
    t_start_us = t0_us + t_start_s * 1e6
    t_end_us = t0_us + t_end_s * 1e6
    mask = (ts >= t_start_us) & (ts <= t_end_us)
    idx = np.where(mask)[0]
    if len(idx) < min_samples + 1:
        return None

    ve = result.ekf.ve[idx]
    vn = result.ekf.vn[idx]
    vu = -result.ekf.vd[idx]
    t_s = (ts[idx] - t0_us) / 1e6

    dt = np.diff(t_s)
    valid = dt > 1e-4
    if valid.sum() < min_samples:
        return None

    dvu = (np.diff(vu) / dt)[valid]
    ve_m = ((ve[:-1] + ve[1:]) / 2)[valid]
    vn_m = ((vn[:-1] + vn[1:]) / 2)[valid]
    vu_m = ((vu[:-1] + vu[1:]) / 2)[valid]
    v_mag = np.sqrt(ve_m**2 + vn_m**2 + vu_m**2)

    keep = (v_mag > v_min_mps) & (vu_m > 1.0)  # still ascending
    if keep.sum() < min_samples:
        return None

    # Solve  -dvu/dt - g  =  k · |v| · vu   over coast samples.
    Y = -dvu[keep] - G_MS2
    X = v_mag[keep] * vu_m[keep]
    xx = float(np.dot(X, X))
    if xx < 1e-6:
        return None
    k = float(np.dot(X, Y) / xx)
    if not (0 < k < 0.01):
        return None
    return k


def _latest_gnss_fix(result: ReplayResult, t_s: float,
                     fresh_threshold_s: float) -> Optional[dict]:
    """Most recent valid GNSS fix at or before t_s, if within freshness window."""
    if len(result.gnss.t_us) == 0:
        return None
    t_target_us = result.phases.t0_us + t_s * 1e6
    gts = result.gnss.t_us
    idx = int(np.searchsorted(gts, t_target_us) - 1)
    if idx < 0:
        return None
    age_s = (t_target_us - gts[idx]) / 1e6
    if age_s < 0 or age_s > fresh_threshold_s:
        return None
    return {
        "t_s": (gts[idx] - result.phases.t0_us) / 1e6,
        "lat_rad": float(result.gnss.lat_deg[idx]) * DEG2RAD,
        "lon_rad": float(result.gnss.lon_deg[idx]) * DEG2RAD,
        "alt_m": float(result.gnss.alt_m[idx]),
        "age_s": age_s,
    }


def actual_landing_enu(result: ReplayResult,
                       lora_csv: Optional[Path] = None
                       ) -> tuple[float, float, float, float]:
    """Best estimate of where the rocket actually landed, in launch-relative ENU.

    Prefers the LoRa downlink CSV (BS keeps receiving telemetry long after the
    rocket binary stops — the onboard binary typically truncates mid-descent).
    Reads the last row with `landed=1` and a valid GNSS fix.

    Falls back to the EKF time-series tail if no LoRa CSV is supplied.
    Returns (e, n, u, t_s).
    """
    import csv
    if lora_csv is not None and lora_csv.exists():
        with open(lora_csv) as f:
            rows = list(csv.DictReader(f))
        # Last row with landed flag set and a usable fix
        landed = [r for r in rows
                  if r.get("landed", "0") == "1"
                  and r.get("lat", "nan") not in ("nan", "", "0.0")
                  and float(r.get("num_sats", 0) or 0) >= 4]
        if landed:
            last = landed[-1]
            lat = float(last["lat"])
            lon = float(last["lon"])
            alt = float(last["alt_m"])
            t_s = float(last["time_ms"]) / 1000.0
            e, n, u = lla_rad_to_enu_m(lat * DEG2RAD, lon * DEG2RAD, alt,
                                        result.launch_ref)
            return e, n, u, t_s

    # Fallback: tail of EKF series.  Note this is unreliable when the binary
    # truncates pre-landing (common — the rocket stops logging after a
    # configurable time post-launch but the BS keeps receiving over LoRa).
    idx = len(result.ekf.t_us) - 1
    e, n, u = lla_rad_to_enu_m(result.ekf.lat_rad[idx], result.ekf.lon_rad[idx],
                                result.ekf.alt_m[idx], result.launch_ref)
    return e, n, u, (result.ekf.t_us[idx] - result.phases.t0_us) / 1e6


# ---------------------------------------------------------------------------
# Predictor v0 (gravity-only ballistic, no wind)
# ---------------------------------------------------------------------------

def _descent_drift(e: float, n: float, u_start: float, u_end: float,
                   drogue_rate_mps: float, main_rate_mps: float,
                   main_deploy_u_m: float, ground_u_m: float,
                   wind: Optional[WindProfile],
                   observed_drogue_rate_mps: Optional[float],
                   observed_main_rate_mps: Optional[float],
                   t_s_start: float, track: list,
                   layer_thickness_m: float = LAYER_THICKNESS_FT * M_PER_FT
                   ) -> tuple[float, float, float, float]:
    """Layered descent from u_start down to u_end in launch-relative ENU.

    Mirrors DriftCastEngine.simulateDescent semantics:
    - Walk 1000 ft layers from top down
    - Switch drogue→main rate at main_deploy_u_m
    - Per-layer wind drift = wind_speed * layer_time, drift bearing = from+180°

    `observed_*_rate_mps` overrides the profile rate when the live EKF shows
    we're already descending at a different speed (user spec: "use the current
    fall rate if under main").
    """
    t_s = t_s_start
    u = u_start

    # Build descending layer boundaries, splitting at main_deploy_u_m
    boundaries = [u]
    a = u
    while a > u_end + 1e-6:
        next_a = max(a - layer_thickness_m, u_end)
        if main_deploy_u_m < a and main_deploy_u_m > next_a:
            boundaries.append(main_deploy_u_m)
            next_a = main_deploy_u_m
            a = next_a
            continue
        boundaries.append(next_a)
        a = next_a

    for i in range(len(boundaries) - 1):
        seg_hi = boundaries[i]
        seg_lo = boundaries[i + 1]
        seg = seg_hi - seg_lo
        if seg <= 0:
            continue
        mid_u = 0.5 * (seg_hi + seg_lo)
        mid_alt_agl_ft = (mid_u - ground_u_m) * FT_PER_M

        # Pick descent rate for this layer
        if mid_u > main_deploy_u_m:
            rate = (observed_drogue_rate_mps
                    if observed_drogue_rate_mps is not None
                    else drogue_rate_mps)
        else:
            rate = (observed_main_rate_mps
                    if observed_main_rate_mps is not None
                    else main_rate_mps)
        if rate < 0.1:
            continue
        dt = seg / rate

        # Wind drift over this layer
        if wind is not None and wind.layers:
            speed_kts, dir_from_deg = wind.interpolate(mid_alt_agl_ft)
            speed_mps = speed_kts * KT_TO_MPS
            drift_to_deg = (dir_from_deg + 180.0) % 360.0
            drift_to_rad = math.radians(drift_to_deg)
            de = math.sin(drift_to_rad) * speed_mps * dt
            dn = math.cos(drift_to_rad) * speed_mps * dt
            e += de
            n += dn

        u = seg_lo
        t_s += dt
        track.append((e, n, u, t_s))

    return e, n, u, t_s


def predict_landing(snap: EkfSnapshot, profile: RocketProfile,
                    ground_u_m: float = 0.0,
                    dt_s: float = 0.05,
                    wind: Optional[WindProfile] = None) -> LandingPrediction:
    """Predict landing in launch-relative ENU from EKF snapshot.

    Steps:
    1. Phase from EKF velocity & altitude.
    2. Ascent path: integrate ballistic-with-drag to apogee in ENU.
       Drag coefficient from profile.ballistic_drag_k (0 = gravity-only).
    3. Descent path: layered drogue→main with optional wind drift per layer.
       Observed-rate override when live VU disagrees with profile (per spec).
    """
    e, n, u = snap.e_m, snap.n_m, snap.u_m
    ve, vn, vu = snap.ve_mps, snap.vn_mps, snap.vu_mps

    track: list[tuple[float, float, float, float]] = [(e, n, u, snap.t_s)]
    t_s = snap.t_s

    main_deploy_u_m = profile.main_deploy_alt_agl_ft * M_PER_FT + ground_u_m
    drogue_rate_mps = profile.drogue_rate_fps * M_PER_FT
    main_rate_mps = profile.main_rate_fps * M_PER_FT

    if vu > 0.5:
        phase = "ASCENT"
    elif u > main_deploy_u_m:
        phase = "DESCENT_DROGUE"
    else:
        phase = "DESCENT_MAIN"

    apogee_e = apogee_n = apogee_u = None

    # ---- Ascent: ballistic propagate to apogee ----
    if phase == "ASCENT":
        k = profile.ballistic_drag_k
        max_steps = int(120.0 / dt_s)
        for _ in range(max_steps):
            v_mag = math.sqrt(ve * ve + vn * vn + vu * vu)
            ae = -k * v_mag * ve
            an = -k * v_mag * vn
            au = -G_MS2 - k * v_mag * vu
            ve += ae * dt_s; vn += an * dt_s; vu += au * dt_s
            e += ve * dt_s; n += vn * dt_s; u += vu * dt_s
            t_s += dt_s
            track.append((e, n, u, t_s))
            if vu <= 0.0:
                break
        apogee_e, apogee_n, apogee_u = e, n, u

    # ---- Descent: layered drift cast ----
    # Observed-rate override per spec: "use current fall rate if under main..."
    observed_drogue = None
    observed_main = None
    if phase == "DESCENT_DROGUE" and -vu > 0.5:
        observed_drogue = -vu
    elif phase == "DESCENT_MAIN" and -vu > 0.5:
        observed_main = -vu

    e, n, u, t_s = _descent_drift(
        e, n, u, ground_u_m,
        drogue_rate_mps, main_rate_mps,
        main_deploy_u_m, ground_u_m,
        wind, observed_drogue, observed_main, t_s, track,
    )

    return LandingPrediction(
        landing_e_m=e, landing_n_m=n,
        descent_track=track, phase=phase,
        apogee_e_m=apogee_e, apogee_n_m=apogee_n, apogee_u_m=apogee_u,
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _find_binary(path: Path) -> Path:
    if path.is_file() and path.suffix == ".bin":
        return path
    if path.is_dir():
        bins = sorted(path.glob("flight_*.bin"))
        if bins:
            return bins[0]
    raise FileNotFoundError(f"No flight_*.bin under {path}")


def _find_lora_csv(bin_path: Path) -> Optional[Path]:
    """Sibling lora_<timestamp>.csv next to the .bin (BS downlink log)."""
    candidates = sorted(bin_path.parent.glob("lora_2*.csv"))
    return candidates[0] if candidates else None


def _infer_flight_datetime_utc(bin_path: Path,
                                local_utc_offset_h: float = -4.0
                                ) -> Optional["datetime"]:
    """Parse local time from flight_YYYYMMDD_HHMMSS.bin and convert to UTC.

    Defaults to UTC-4 (EDT) — flights in this repo are in Maryland.  Override
    via CLI when running flights from other timezones / DST states.
    """
    from datetime import datetime, timedelta, timezone
    stem = bin_path.stem  # e.g. flight_20260517_153649
    parts = stem.split("_")
    if len(parts) < 3:
        return None
    try:
        dt_local = datetime.strptime(parts[1] + parts[2], "%Y%m%d%H%M%S")
    except ValueError:
        return None
    return (dt_local - timedelta(hours=local_utc_offset_h)
            ).replace(tzinfo=timezone.utc)


def _format_enu(e, n, u=None):
    if u is None:
        return f"E={e:+8.1f} m  N={n:+8.1f} m"
    return f"E={e:+8.1f} m  N={n:+8.1f} m  U={u:+7.1f} m"


def run_single(bin_path: Path, t_loss_s: float, profile: RocketProfile,
               wind: Optional[WindProfile] = None, fit_drag: bool = False):
    print(f"Parsing: {bin_path}")
    records, stats, _ = parse_binary_file(str(bin_path))
    print(f"  Frames: {stats['good_crc']:,} good, {stats['bad_crc']} bad CRC")
    print(f"  IMU: {len(records['ISM6HG256']):,}  GNSS: {len(records['GNSS']):,}  "
          f"Baro: {len(records['BMP585']):,}  Mag: {len(records['MMC5983MA']):,}")

    print("\nReplaying through EKF...")
    result = replay_binary(records, verbose=True)
    print(f"  EKF samples logged: {len(result.ekf.t_us):,}")
    print(f"  Launch ref: {result.launch_ref.lat_deg:.6f}, "
          f"{result.launch_ref.lon_deg:.6f}, alt={result.launch_ref.alt_m:.1f} m")

    if wind is not None:
        print(f"\nWind profile: {wind.source}  ground={wind.ground_elev_ft:.0f}ft  "
              f"{len(wind.layers)} layers")
        for L in wind.layers[:6]:
            print(f"    {L.alt_ft_agl:7.0f} ft AGL  {L.speed_kts:5.1f} kts  from {L.dir_from_deg:5.1f}°")

    # Snapshot
    snap = snapshot_at(result, t_loss_s)
    print(f"\nT_loss = {t_loss_s:.2f} s")
    print(f"  EKF position : {_format_enu(snap.e_m, snap.n_m, snap.u_m)}")
    print(f"  EKF velocity : VE={snap.ve_mps:+6.2f}  VN={snap.vn_mps:+6.2f}  "
          f"VU={snap.vu_mps:+6.2f} m/s")

    # Optional: refine drag k from observed coast deceleration (causal).
    # Off by default because pre-#176 (GNSS-1g) binaries have IMU-bias-driven
    # velocity drift through coast that corrupts the fit.  Enable on flights
    # logged with the GNSS-4g fix.
    if fit_drag:
        k = fit_drag_k_from_coast(result, t_loss_s)
        if k is not None:
            print(f"  drag k (fit): {k:.5f} 1/m  (was {profile.ballistic_drag_k:.5f})")
            profile.ballistic_drag_k = k
        else:
            print(f"  drag k (fit): no usable coast data — using profile default "
                  f"{profile.ballistic_drag_k:.5f}")

    # Predict
    pred = predict_landing(snap, profile, wind=wind)
    print(f"\n  Phase at loss: {pred.phase}")
    if pred.apogee_u_m is not None:
        print(f"  Predicted apogee:  {_format_enu(pred.apogee_e_m, pred.apogee_n_m, pred.apogee_u_m)}")
    print(f"  Predicted landing: {_format_enu(pred.landing_e_m, pred.landing_n_m)}")

    # Actual landing (prefer LoRa CSV — binary often truncates pre-landing)
    lora_csv = _find_lora_csv(bin_path)
    act_e, act_n, act_u, act_t = actual_landing_enu(result, lora_csv)
    src = f"LoRa {lora_csv.name}" if lora_csv else "EKF tail"
    print(f"\n  Actual landing:    {_format_enu(act_e, act_n, act_u)}  (at t={act_t:.1f}s, src={src})")

    err = math.sqrt((pred.landing_e_m - act_e) ** 2 + (pred.landing_n_m - act_n) ** 2)
    print(f"\n  Landing error: {err:.1f} m")

    return result, snap, pred, (act_e, act_n, act_u), err


def main():
    p = argparse.ArgumentParser()
    p.add_argument("flight_path", type=Path,
                   help="Path to flight binary (.bin) or directory containing one")
    p.add_argument("--loss-time", type=float, default=None,
                   help="GPS-loss time (s since first IMU sample)")
    p.add_argument("--drogue-fps", type=float, default=60.0)
    p.add_argument("--main-fps", type=float, default=12.0)
    p.add_argument("--main-alt-ft", type=float, default=700.0)
    p.add_argument("--drag-k", type=float, default=5e-4,
                   help="Quadratic drag coefficient (1/m); 0 = gravity-only")
    p.add_argument("--fit-drag", action="store_true",
                   help="Try to fit drag k from coast deceleration "
                        "(unreliable on pre-#176 GNSS-1g binaries)")
    p.add_argument("--no-wind", action="store_true",
                   help="Skip Open-Meteo fetch and predict with zero wind")
    p.add_argument("--utc-offset-h", type=float, default=-4.0,
                   help="Local-time offset from UTC for filename parsing "
                        "(default -4 = EDT)")
    args = p.parse_args()

    bin_path = _find_binary(args.flight_path)
    profile = RocketProfile(
        drogue_rate_fps=args.drogue_fps,
        main_rate_fps=args.main_fps,
        main_deploy_alt_agl_ft=args.main_alt_ft,
        ballistic_drag_k=args.drag_k,
    )

    # Wind fetch (using filename-inferred time + first GNSS fix location)
    wind = None
    if not args.no_wind:
        dt_utc = _infer_flight_datetime_utc(bin_path,
                                             local_utc_offset_h=args.utc_offset_h)
        if dt_utc is None:
            print("(skipping wind: couldn't parse flight time from filename)")
        else:
            # We need launch lat/lon — parse just the GNSS records, cheap.
            from plot_flight_data_mini import parse_binary_file as _pbf
            recs, _, _ = _pbf(str(bin_path))
            gnss = [g for g in recs["GNSS"] if g.get("num_sats", 0) >= 4]
            if not gnss:
                print("(skipping wind: no valid GNSS fix in flight)")
            else:
                lat0, lon0 = gnss[0]["lat"], gnss[0]["lon"]
                cache = _HERE / "test_data" / "wind_cache"
                try:
                    wind = fetch_wind(lat0, lon0, dt_utc, cache_dir=cache)
                except Exception as e:
                    print(f"(wind fetch failed: {e})")

    if args.loss_time is None:
        # Default: 1 second after apogee (descent case)
        print("Replaying to detect default T_loss...")
        records, stats, _ = parse_binary_file(str(bin_path))
        result = replay_binary(records, verbose=False)
        if result.phases.apogee_us is None:
            print("ERROR: Could not detect apogee; supply --loss-time")
            return 1
        t_loss = (result.phases.apogee_us - result.phases.t0_us) / 1e6 + 1.0
        print(f"  Defaulting to T_loss = apogee + 1s = {t_loss:.2f}s")
        snap = snapshot_at(result, t_loss)
        pred = predict_landing(snap, profile, wind=wind)
        act_e, act_n, act_u, act_t = actual_landing_enu(
            result, _find_lora_csv(bin_path))
        err = math.sqrt((pred.landing_e_m - act_e) ** 2 + (pred.landing_n_m - act_n) ** 2)
        print(f"\nT_loss={t_loss:.2f}s  phase={pred.phase}  error={err:.1f} m")
        return 0

    run_single(bin_path, args.loss_time, profile, wind=wind,
               fit_drag=args.fit_drag)
    return 0


if __name__ == "__main__":
    sys.exit(main())
