"""#552: the filter state the ROCKET logged, shaped like a replay result.

The landing-prediction sweep took its snapshots from `_ekf_replay.replay_binary`,
which re-runs the EKF on the host over the logged sensor stream. That measures
the replay's filter. The app never sees a replay — it predicts from the state
the rocket sends it, which is the same state the flight log records. So for a
skill measurement the logged state is not merely an acceptable substitute, it is
the honest input, and it removes the sweep's dependence on replay fidelity
(#1412) entirely.

Two things make this nearly free:

  * `EkfSnapshot` is already launch-relative ENU, and the NonSensor rows carry
    exactly that — `e_pos/n_pos/u_pos`, `e_vel/n_vel/u_vel` — at ~490 Hz.
  * The ENU origin those are relative to is in the log too, in every in-flight
    Snapshot (#1419). Using it rather than rebuilding one from pad fixes is the
    whole point: on Eagle Claw 2026-08-29 the reconstruction sat 38.6 m from the
    real origin, which reads as a bodily track shift.

So `launch_ref` here IS the firmware's own origin. The logged ENU then passes
through `snapshot_at` unchanged (its LLA round trip is this module's exact
inverse), and GNSS is converted into the same frame the firmware used, so the
two are comparable without a manufactured offset.
"""
from __future__ import annotations

import math

import numpy as np

from _ekf_replay import (BaroSeries, EkfSeries, GnssSeries, LaunchRef,
                         ReplayResult, detect_flight_phases,
                         _pressure_to_altitude, enu_m_to_lla_deg)

DEG2RAD = math.pi / 180.0

# A logged origin further than this from the GNSS stream's own pad mean cannot
# be the origin of those fixes — same bar and same reasoning as the flight
# report's globe module (#1419).
REF_PLAUSIBLE_M = 500.0


def _pad_mean_lla(records):
    """Mean of the pre-launch GNSS fixes, as a sanity check on the logged origin."""
    fixes = [g for g in records.get("GNSS", []) if g.get("num_sats", 0) >= 4]
    if not fixes:
        return None
    ns = records.get("NonSensor", [])
    launch_us = next((r["time_us"] for r in ns if r.get("launch")), None)
    pad = [g for g in fixes if launch_us is None or g["time_us"] <= launch_us]
    if not pad:
        pad = fixes[: min(len(fixes), 20)]
    return (sum(g["lat"] for g in pad) / len(pad),
            sum(g["lon"] for g in pad) / len(pad),
            sum(g["alt_m"] for g in pad) / len(pad))


def logged_reference(records):
    """The ENU origin the firmware used, from the log — (LaunchRef, note).

    Returns (None, note) when no Snapshot carries one, or when the one it
    carries is implausibly far from the pad fixes it should be the origin of.
    """
    snaps = records.get("Snapshot") or []
    pad = _pad_mean_lla(records)
    for s in snaps:
        lat, lon = s.get("ref_lat"), s.get("ref_lon")
        alt = s.get("ref_alt_m")
        if lat is None or lon is None or alt is None:
            continue
        if lat == 0.0 and lon == 0.0:
            continue        # the pre-freeze placeholder, not an origin
        if pad is not None:
            dn = (lat - pad[0]) * DEG2RAD * 6378137.0
            de = (lon - pad[1]) * DEG2RAD * 6378137.0 * math.cos(pad[0] * DEG2RAD)
            if math.hypot(dn, de) > REF_PLAUSIBLE_M:
                return None, (f"logged origin is {math.hypot(dn, de):.0f} m from the "
                              f"pad fixes — refused as implausible")
        conv = s.get("ref_datum_converged")
        return (LaunchRef(lat_deg=float(lat), lon_deg=float(lon), alt_m=float(alt)),
                f"logged origin (converged={conv})")
    return None, "no Snapshot record carries an ENU reference"


def logged_binary(records, verbose: bool = False) -> ReplayResult:
    """Build a ReplayResult from the state the rocket logged, not a re-run.

    Shape-compatible with `replay_binary`, so `snapshot_at`, the drag fit and
    the sweep all work unchanged.
    """
    phases = detect_flight_phases(records)

    ref, note = logged_reference(records)
    if ref is None:
        raise RuntimeError(f"cannot use the logged filter state: {note}")
    if verbose:
        print(f"  ENU origin: {note} "
              f"({ref.lat_deg:.6f}, {ref.lon_deg:.6f}, {ref.alt_m:.1f} m)")

    ns = [r for r in records.get("NonSensor", []) if r.get("time_us") is not None]
    if not ns:
        raise RuntimeError("no NonSensor rows — nothing was logged to measure")

    t = np.asarray([r["time_us"] for r in ns], dtype=np.int64)
    e = np.asarray([r.get("e_pos", 0.0) for r in ns], dtype=float)
    n = np.asarray([r.get("n_pos", 0.0) for r in ns], dtype=float)
    u = np.asarray([r.get("u_pos", 0.0) for r in ns], dtype=float)
    ve = np.asarray([r.get("e_vel", 0.0) for r in ns], dtype=float)
    vn = np.asarray([r.get("n_vel", 0.0) for r in ns], dtype=float)
    vu = np.asarray([r.get("u_vel", 0.0) for r in ns], dtype=float)

    # ENU -> LLA with the exact inverse of the conversion snapshot_at applies
    # on the way back, so the logged numbers survive the round trip untouched.
    lat_deg, lon_deg, alt_m = np.vectorize(
        lambda ee, nn, uu: enu_m_to_lla_deg(ee, nn, uu, ref))(e, n, u)

    q = np.asarray([[r.get("q0", 1.0), r.get("q1", 0.0),
                     r.get("q2", 0.0), r.get("q3", 0.0)] for r in ns], dtype=float)

    ekf_series = EkfSeries(
        t_us=t,
        lat_rad=np.asarray(lat_deg, dtype=float) * DEG2RAD,
        lon_rad=np.asarray(lon_deg, dtype=float) * DEG2RAD,
        alt_m=np.asarray(alt_m, dtype=float),
        vn=vn, ve=ve,
        vd=-vu,                       # the series is NED; the log is ENU
        q=q,
        # Not logged per-row and not read by the predictor. NaN rather than 0
        # so anything that starts reading it fails loudly instead of believing
        # the filter was perfectly certain.
        cov_pos=np.full((len(ns), 3), np.nan),
        cov_vel=np.full((len(ns), 3), np.nan),
    )

    g = [x for x in records.get("GNSS", []) if x.get("num_sats", 0) >= 4]
    gnss_series = GnssSeries(
        t_us=np.asarray([x["time_us"] for x in g], dtype=np.int64),
        lat_deg=np.asarray([x["lat"] for x in g], dtype=float),
        lon_deg=np.asarray([x["lon"] for x in g], dtype=float),
        alt_m=np.asarray([x["alt_m"] for x in g], dtype=float),
        vn=np.asarray([x["vel_n"] for x in g], dtype=float),
        ve=np.asarray([x["vel_e"] for x in g], dtype=float),
        vu=np.asarray([x["vel_u"] for x in g], dtype=float),
    )

    b = records.get("BMP585", [])
    baro_t, baro_alt = [], []
    if b:
        ref_pa = float(np.mean([x["pressure_pa"] for x in b[:20]]))
        for x in b:
            baro_t.append(x["time_us"])
            baro_alt.append(_pressure_to_altitude(x["pressure_pa"], ref_pa) + ref.alt_m)
    baro_series = BaroSeries(t_us=np.asarray(baro_t, dtype=np.int64),
                             alt_msl_m=np.asarray(baro_alt, dtype=float))

    if verbose:
        span = (t[-1] - t[0]) / 1e6 if len(t) > 1 else 0.0
        print(f"  logged filter state: {len(t):,} rows over {span:.1f} s "
              f"({len(t) / span if span else 0:.0f} Hz), "
              f"{len(g):,} GNSS fixes")

    return ReplayResult(phases=phases, ekf=ekf_series, gnss=gnss_series,
                        baro=baro_series, launch_ref=ref)
