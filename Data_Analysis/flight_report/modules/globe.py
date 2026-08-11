"""The flight path in 3D, over real satellite imagery and terrain.

Two tracks are drawn, deliberately:

* the **navigation filter** — the onboard fusion solution, the same series the
  engineering report plots as a 3D chart, and
* the **raw GNSS fixes** — what the receiver reported, unfiltered.

Showing one alone would be a claim. Showing both is a measurement. On the sample
flights they agree to well under a meter until the receiver loses lock during
boost, then part company and never reconverge — a disagreement large enough to
send someone to the wrong end of a field, and one this section exists to make
visible rather than average away.

Visible, but not narrated. Drawing both tracks on one globe lets a reader see the
divergence directly; the report does not put a number on it or advise which to
believe, because working out what causes it is issue #741 and not something a
flyer should have to adjudicate. The separations are still computed and carried
in the spec — `originResidualM` in particular is the health check on the
reconstructed ENU origin below, and a test bounds it.

Three things about the geometry are worth knowing before changing anything here.

**The two tracks share a time base but not a rate.** Both `time_us` fields come
from the same MCU clock and `Flight._compute_t0` takes the minimum across all
record lists, so no time alignment is needed. The nav filter logs at ~440 Hz and
GNSS at ~18 Hz; each track therefore carries its own timestamps and neither is
resampled onto the other. Resampling would invent samples and smooth away
exactly the disagreement the section is for.

**The filter's ENU origin is not in the log.** Firmware sets it to a running mean
of pad GNSS fixes, frozen 120 s after the first one, and never writes it to any
frame or to the sidecar. Python has to reconstruct it to place the nav track on
a map at all, and the reconstruction error is a real offset that has nothing to
do with filter quality. It is measured and published as `originResidualM` so a
reader can tell a datum error from a genuine disagreement.

**Altitudes are above the pad, resolved to the globe's datum in the browser.**
Cesium wants heights above the WGS84 ellipsoid; GNSS reports MSL; the difference
is the local geoid undulation, up to tens of meters, which would bury the track
under the terrain or float it above. Rather than embed a geoid model, both tracks
ship as height-above-pad and the page adds the pad's own terrain height once.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

from ..cesium_bundle import CesiumPatchError, CesiumUnavailable, cesium_source
from ..charts import chart, trace
from ..flight import Flight
from ..registry import AnalysisResult
from ..units import q

# The firmware's own spherical earth radius. Used to invert the firmware's own
# ENU conversion, so the round trip is exact against what the board actually did
# rather than "more correct" in a way that adds a second discrepancy.
R_EARTH_M = 6378137.0

# u-blox fixType: 0 no fix, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS+DR, 5 time-only.
# Firmware forces the field to 0 whenever the fix is not OK, so >= 3 is the
# "trust this position" test.
FIX_3D = 3

# Firmware freezes the ENU reference this long after the first qualifying fix.
REF_POS_MAX_AGE_S = 120.0

# Plotting a pre-fix (0, 0) sample runs a leg of the track through the Gulf of
# Guinea. It is the only sentinel the receiver emits.
_NULL_ISLAND_EPS = 1e-9

G_MS2 = 9.80665

TRACK_COLORS = {"filter": "#1f77b4", "gnss": "#ff9d3c"}


def _t(records, t0_us) -> np.ndarray:
    return (get_array(records, "time_us") - t0_us) / 1e6


def _events(recs, t0_us) -> dict[str, Optional[float]]:
    ns = recs.get("NonSensor") or []
    out: dict[str, Optional[float]] = {}
    for label, flag in (("launch", "launch"), ("apogee", "alt_apogee"),
                        ("landed", "alt_landed")):
        out[label] = None
        for r in ns:
            if r.get(flag):
                out[label] = (r["time_us"] - t0_us) / 1e6
                break
    return out


def _pad_reference(gnss, t, launch_s) -> Optional[tuple[float, float, float]]:
    """Mean pad position over the fixes the firmware would have averaged.

    Mirrors the firmware's reference: qualifying fixes from the first one through
    either 120 s later or launch, whichever comes first.

    It deliberately does *not* replicate the firmware's extra init gates (minimum
    satellites, horizontal-accuracy and velocity bounds). Those thresholds are
    config constants that the log does not carry, so copying them here would
    create a second source of truth that rots silently the next time firmware
    changes them. The rocket is stationary on the pad, so they reject
    substantially the same handful of fixes `fix_mode >= 3` already rejects.
    """
    if not gnss:
        return None
    lat = get_array(gnss, "lat")
    lon = get_array(gnss, "lon")
    alt = get_array(gnss, "alt_m")
    fix = get_array(gnss, "fix_mode") if "fix_mode" in gnss[0] else np.full(lat.shape, FIX_3D)

    ok = _valid_fix(lat, lon, alt, fix)
    if not ok.any():
        return None

    first = float(t[ok][0])
    if launch_s is None:
        # With no launch event the 120 s cap alone would average the whole
        # trajectory on any flight shorter than that, putting the "pad" somewhere
        # over the field. Fall back to the first second of fixes, which is pad
        # for any log that starts before the rocket leaves the rail.
        end = first + 1.0
    else:
        end = min(first + REF_POS_MAX_AGE_S, launch_s)

    window = ok & (t >= first) & (t <= end)
    if window.sum() < 1:
        # Launch at or before the first usable fix: nothing qualifies, so take
        # the earliest fixes available rather than returning no georeference at
        # all. Averaging a handful still beats a single cold-start sample.
        idx = np.where(ok)[0][:10]
        window = np.zeros(ok.shape, dtype=bool)
        window[idx] = True
    return (float(np.mean(lat[window])), float(np.mean(lon[window])),
            float(np.mean(alt[window])))


def _valid_fix(lat, lon, alt, fix) -> np.ndarray:
    """Finite, 3D-fixed, and not the receiver's pre-fix null-island sentinel.

    Both coordinates must be zero to count as the sentinel. Testing them
    separately would throw away every genuine fix on the equator or the prime
    meridian — rare for model rocketry, but the guard costs nothing to state
    correctly and a silently-dropped track is hard to diagnose.
    """
    null_island = (np.abs(lat) <= _NULL_ISLAND_EPS) & (np.abs(lon) <= _NULL_ISLAND_EPS)
    return (np.isfinite(lat) & np.isfinite(lon) & np.isfinite(alt)
            & ~null_island & (fix >= FIX_3D))


def _enu_to_geodetic(east, north, pad_lat, pad_lon, pad_alt) -> tuple[np.ndarray, np.ndarray]:
    """Invert the firmware's spherical small-angle ENU conversion.

    Matching `main.cpp:4429-4435` term for term, including the `+ ref_alt_m` on
    the radius and the fact that the longitude scaling uses the *pad* latitude
    rather than the current one. Both matter only slightly here, but the point of
    inverting the firmware's own formula rather than using a better one is that
    the round trip is exact against what the board actually computed.
    """
    radius = R_EARTH_M + pad_alt
    lat = pad_lat + np.degrees(north / radius)
    lon = pad_lon + np.degrees(east / (radius * math.cos(math.radians(pad_lat))))
    return lat, lon


def _flat(lat: np.ndarray, lon: np.ndarray, agl: np.ndarray) -> list[float]:
    """[lon, lat, height, ...] — exactly Cartesian3.fromDegreesArrayHeights' argument.

    Flat rather than nested so the page does no reshaping, and about 30% smaller
    as JSON. Six decimals of angle is ~0.11 m, matching maps.py and three orders
    of magnitude finer than the origin uncertainty this section reports.
    """
    out: list[float] = []
    for i in range(lat.size):
        out.extend((round(float(lon[i]), 6), round(float(lat[i]), 6), round(float(agl[i]), 1)))
    return out


def _dedupe(*arrays: np.ndarray) -> np.ndarray:
    """Mask dropping samples identical to their predecessor across every array.

    Not decimation — the position fields are logged faster than they are updated
    (the nav filter is logged at ~960 Hz but recomputed at ~440), so consecutive
    repeats are literally the same number twice. Dropping exact repeats halves
    the payload and cannot lose a sample that was never distinct. Everything
    that survives is kept: this section does not stride.
    """
    n = arrays[0].size
    if n == 0:
        return np.zeros(0, dtype=bool)
    keep = np.zeros(n, dtype=bool)
    keep[0] = True
    changed = np.zeros(n, dtype=bool)
    for a in arrays:
        changed[1:] |= a[1:] != a[:-1]
    keep |= changed
    # Always keep the last sample. A rocket sitting still after touchdown logs a
    # run of identical positions, so the final sample is usually a repeat and
    # would be dropped — silently moving the landing marker to whenever the
    # vehicle last twitched. Keeping it costs one vertex.
    keep[-1] = True
    return keep


def _nav_track(recs, t0, events, pad) -> Optional[dict[str, Any]]:
    ns = recs.get("NonSensor") or []
    if not ns or not all(k in ns[0] for k in ("e_pos", "n_pos", "u_pos")):
        return None
    t = _t(ns, t0)
    e, n, u = (get_array(ns, k) for k in ("e_pos", "n_pos", "u_pos"))

    window = _window(t, events)
    good = window & np.isfinite(e) & np.isfinite(n) & np.isfinite(u)
    if good.sum() < 2:
        return None
    e, n, u, t = e[good], n[good], u[good], t[good]

    keep = _dedupe(e, n, u)
    e, n, u, t = e[keep], n[keep], u[keep], t[keep]
    if t.size < 2:                 # a vehicle that never moved; nothing to draw
        return None

    lat, lon = _enu_to_geodetic(e, n, pad[0], pad[1], pad[2])
    # u_pos is already height above the firmware's pad-average altitude, so it is
    # AGL by construction and needs no offset.
    return {
        "key": "filter",
        "name": "EKF",
        "source": "Onboard fusion of IMU, barometer and GNSS",
        "color": TRACK_COLORS["filter"],
        "positions": _flat(lat, lon, u),
        "t": [round(float(v), 3) for v in t],
    }


def _gnss_track(recs, t0, events, pad) -> Optional[dict[str, Any]]:
    g = recs.get("GNSS") or []
    if not g:
        return None
    t = _t(g, t0)
    lat, lon, alt = (get_array(g, k) for k in ("lat", "lon", "alt_m"))
    fix = get_array(g, "fix_mode") if "fix_mode" in g[0] else np.full(lat.shape, FIX_3D)

    window = _window(t, events)
    good = window & _valid_fix(lat, lon, alt, fix)
    if good.sum() < 2:
        return None
    lat, lon, alt, t = lat[good], lon[good], alt[good], t[good]

    keep = _dedupe(lat, lon, alt)
    lat, lon, alt, t = lat[keep], lon[keep], alt[keep], t[keep]
    if t.size < 2:
        return None

    # Referenced to the same pad mean the filter uses, not to alt_m[0] — a single
    # first sample is exactly the one most likely to be a cold-start outlier.
    return {
        "key": "gnss",
        "name": "GNSS",
        "source": "Receiver position, unfiltered",
        "color": TRACK_COLORS["gnss"],
        "positions": _flat(lat, lon, alt - pad[2]),
        "t": [round(float(v), 3) for v in t],
    }


def _window(t: np.ndarray, events) -> np.ndarray:
    """Launch to touchdown. The pad minutes are thousands of samples on one point."""
    start, end = events.get("launch"), events.get("landed") or events.get("apogee")
    if start is None or end is None:
        return np.ones(t.shape, dtype=bool)
    return (t >= start) & (t <= end)


def _horizontal_m(a: dict, ai: int, b: dict, bi: int, pad_lat: float) -> float:
    """Ground distance between vertex `ai` of one track and `bi` of the other.

    Two indices, not one: the tracks are sampled at wildly different rates
    (~440 Hz against ~18 Hz), so the same ordinal is a different moment in each
    and comparing them would measure the rate difference, not the disagreement.
    """
    m_lat = 111132.92 - 559.82 * math.cos(2 * math.radians(pad_lat))
    m_lon = 111412.84 * math.cos(math.radians(pad_lat))
    de = (a["positions"][ai * 3] - b["positions"][bi * 3]) * m_lon
    dn = (a["positions"][ai * 3 + 1] - b["positions"][bi * 3 + 1]) * m_lat
    return math.hypot(de, dn)


def _nearest(track: dict, when_s: float) -> int:
    """Index of the sample closest in time to `when_s`."""
    return int(np.argmin(np.abs(np.asarray(track["t"], dtype=float) - when_s)))


def _apogee_m(track: dict) -> float:
    return max(track["positions"][2::3])


def _accel_sats_chart(recs, t0) -> Optional[dict[str, Any]]:
    """Acceleration against satellite count, on one time axis.

    The two tracks above diverge because the receiver loses lock under boost.
    This is that, directly: satellites on the right axis, |a| from both
    accelerometers on the left.
    """
    imu = recs.get("ISM6HG256") or []
    gnss = recs.get("GNSS") or []
    if not imu or not gnss or "num_sats" not in gnss[0]:
        return None

    events = _events(recs, t0)
    t_imu = _t(imu, t0)
    start = events.get("launch")
    end = events.get("landed") or events.get("apogee")
    win = ((t_imu >= start) & (t_imu <= end)) if start is not None and end is not None \
        else np.ones(t_imu.shape, dtype=bool)
    if win.sum() < 10:
        return None

    traces = []
    for prefix, label, color in (("high_acc", "High-G |a|", "#ff9896"),
                                  ("low_acc", "Low-G |a|", "#1f77b4")):
        if not all(f"{prefix}_{a}" in imu[0] for a in "xyz"):
            continue
        mag = np.sqrt(sum(get_array(imu, f"{prefix}_{a}")[win] ** 2 for a in "xyz"))
        traces.append(trace(t_imu[win], mag / G_MS2, label, color))

    t_g = _t(gnss, t0)
    gwin = ((t_g >= start) & (t_g <= end)) if start is not None and end is not None \
        else np.ones(t_g.shape, dtype=bool)
    traces.append(trace(t_g[gwin], get_array(gnss, "num_sats")[gwin],
                        "Satellites", "#2ca02c", axis="y2"))

    spec = chart("chart-accel-sats", "Acceleration and satellite count",
                 traces, y_title="Acceleration", y_unit="G",
                 y2_title="Satellites", events=events, height=340,
                 clip_spikes=True)
    if spec:
        # States the mechanism, not a reading of this particular flight. A caption
        # is generated for every report, so "this is where the tracks diverge" is
        # a conclusion that happens to hold here and would be wrong elsewhere —
        # the chart is there for the reader to draw it, or not.
        spec["note"] = ("Satellites on the right axis. GNSS receivers lose track under "
                        "high acceleration.")
    return spec


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="globe", title="Flight Path in 3D")
    recs = flight.records
    t0 = flight.t0_us

    if t0 is None:
        result.warnings.append("No timestamped records — nothing to place on the map.")
        return result

    events = _events(recs, t0)
    gnss = recs.get("GNSS") or []
    t_gnss = _t(gnss, t0) if gnss else np.zeros(0)
    pad = _pad_reference(gnss, t_gnss, events.get("launch"))
    if pad is None:
        result.warnings.append(
            "No usable GNSS fix on the pad, so there is no way to say where on "
            "Earth this flight happened. The 3D path is still in the engineering "
            "report, plotted in meters from the pad."
        )
        return result

    tracks = [tr for tr in (_nav_track(recs, t0, events, pad),
                            _gnss_track(recs, t0, events, pad)) if tr]
    if not tracks:
        result.warnings.append("No position samples inside the flight window.")
        return result

    # Ask for the Cesium bundle here, before emitting a spec, so that a bundle
    # which no longer takes its file:// worker patch costs this section and
    # nothing else. render_report inlines the same (cached) source once a globe
    # spec exists, and it has no module-level exception handling — if the raise
    # happened there it would take down the entire report over one section.
    try:
        cesium_source()
    except CesiumUnavailable as e:
        result.warnings.append(
            f"The 3D view was left out: the CesiumJS bundle could not be read ({e}). "
            "The flight path is still plotted in the engineering report."
        )
        return result
    except CesiumPatchError as e:
        result.warnings.append(
            f"The 3D view was left out: {e} The flight path is still plotted in "
            "the engineering report."
        )
        return result

    spec: dict[str, Any] = {
        "id": "globe-track",
        "title": "Flight path over imagery",
        "pad": {"lat": round(pad[0], 7), "lon": round(pad[1], 7), "mslM": round(pad[2], 1)},
        "tracks": tracks,
    }

    metrics: dict[str, Any] = {}
    for tr in tracks:
        metrics[f"Apogee — {tr['name']}"] = q(_apogee_m(tr), "m", 1)
    metrics["Samples"] = " · ".join(
        f"{tr['name']} {len(tr['t']):,}" for tr in tracks
    )

    if len(tracks) == 2:
        nav, gps = tracks
        # How far apart the two solutions are at the start and at the end of the
        # flight, compared at matched *times* rather than matched ordinals — see
        # _horizontal_m. Neither is shown in the report: the tracks are drawn on
        # the same globe, so the reader can see the divergence directly, and the
        # investigation into what causes it is issue #741 rather than something
        # a flyer needs narrated. The start figure stays in the spec because it
        # is the health check on the reconstructed ENU origin — a bad
        # reconstruction shifts the whole nav track bodily, and a test bounds it.
        t_start = max(nav["t"][0], gps["t"][0])
        t_end = min(nav["t"][-1], gps["t"][-1])
        spec["originResidualM"] = round(
            _horizontal_m(nav, _nearest(nav, t_start),
                          gps, _nearest(gps, t_start), pad[0]), 1)
        spec["endSeparationM"] = round(
            _horizontal_m(nav, _nearest(nav, t_end),
                          gps, _nearest(gps, t_end), pad[0]), 1)
        spec["endIsLanding"] = events.get("landed") is not None
        _warn_if_gnss_stalled(result, _moved_m(gps, pad[0]))

    result.metrics = metrics

    spec["note"] = ("Two position solutions: the onboard navigation filter and the raw "
                    "GNSS fixes. Imagery loads from Esri when the report is opened, so "
                    "this view needs a connection; the rest of the report does not.")
    result.globes = [spec]
    accel_sats = _accel_sats_chart(recs, t0)
    result.charts = [accel_sats] if accel_sats else []
    return result


def _moved_m(track: dict, pad_lat: float) -> float:
    """How far the track's position wanders over its whole length, in meters."""
    lons, lats = track["positions"][0::3], track["positions"][1::3]
    m_lat = 111132.92 - 559.82 * math.cos(2 * math.radians(pad_lat))
    m_lon = 111412.84 * math.cos(math.radians(pad_lat))
    return math.hypot((max(lons) - min(lons)) * m_lon, (max(lats) - min(lats)) * m_lat)


# A GNSS track that wanders less than this never really moved: the receiver was
# repeating a position rather than tracking the rocket.
_STATIONARY_M = 10.0


def _warn_if_gnss_stalled(result: AnalysisResult, gnss_span_m: float) -> None:
    """Flag a GNSS track that never moved — a fault, not a disagreement.

    The two solutions differing is expected and is left for the reader to see on
    the globe. A receiver that reports the same position for an entire flight is
    a different thing: the orange track is then not a measurement at all, and
    without saying so the view invites someone to trust it.
    """
    if gnss_span_m >= _STATIONARY_M:
        return
    result.warnings.append(
        f"The GNSS track moves only {gnss_span_m:,.0f} m across the entire flight, "
        "which a rocket that reached apogee cannot have done — the receiver was "
        "repeating a position rather than tracking it. Use the navigation filter "
        "track; treat the GNSS one as a diagnostic."
    )
