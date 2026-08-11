"""Headline flight numbers for the summary card.

Computed from the flight records rather than the `.json` sidecar, so the card is
identical whether or not a sidecar was saved alongside the log. Every value is
SI; formatting (and the eventual imperial toggle) belongs in the template.

Each entry is ``{"label", "value", "unit", "text", "hint"}``. `value` is the raw
number for downstream use, `text` is what the card shows, and `hint` says where
the number came from — which sensor disagreed with which matters when a flyer is
comparing an altitude claim against someone else's altimeter.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array, gnss_to_enu, pressure_to_altitude  # noqa: E402

from .imu import accel_magnitude
from .units import q

G = 9.80665

_COMPASS = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]


def _cell(label: str, value: Optional[float], unit: str, places: int = 1,
          hint: str = "", suffix: str = "") -> dict[str, Any]:
    """One card cell. `value` is SI; the Quantity renders and converts itself."""
    return {"label": label, "q": q(value, unit, places, suffix), "hint": hint}


def _event_times(records, t0_us) -> dict[str, Optional[float]]:
    """Seconds since t0 for the first occurrence of each flight event."""
    ns = records.get("NonSensor") or []
    out: dict[str, Optional[float]] = {}
    for label, flag in (("launch", "launch"), ("burnout", "burnout"),
                        ("apogee", "alt_apogee"), ("landed", "alt_landed")):
        out[label] = None
        for r in ns:
            if r.get(flag):
                out[label] = (r["time_us"] - t0_us) / 1e6
                break
    return out


def _baro_agl(records) -> Optional[np.ndarray]:
    """Barometric altitude above the pad, using the pad-phase mean as reference."""
    baro = records.get("BMP585") or []
    if not baro:
        return None
    p = get_array(baro, "pressure_pa")
    if not p.size:
        return None
    p_ground = float(np.mean(p[: min(200, p.size)]))
    return np.array([pressure_to_altitude(float(v), p_ground) for v in p])


def compute_summary(flight) -> list[dict[str, Any]]:
    """The headline cells, in display order. Cells with no data are omitted."""
    recs = flight.records
    t0 = flight.t0_us
    if t0 is None:
        return []

    cells: list[dict[str, Any]] = []
    events = _event_times(recs, t0)

    # --- Apogee altitude -----------------------------------------------------
    alt = _baro_agl(recs)
    gnss = recs.get("GNSS") or []
    gnss_apogee = None
    if gnss and "alt_m" in gnss[0]:
        g_alt = get_array(gnss, "alt_m")
        if g_alt.size:
            gnss_apogee = float(np.max(g_alt) - g_alt[0])

    if alt is not None and alt.size:
        baro_apogee = float(np.max(alt))
        hint = "barometric, above the pad"
        if gnss_apogee is not None:
            hint += f" · GNSS says {gnss_apogee:,.0f} m"
        cells.append(_cell("Apogee", baro_apogee, "m", 1, hint))
    elif gnss_apogee is not None:
        cells.append(_cell("Apogee", gnss_apogee, "m", 1, "GNSS (no barometer data)"))

    # --- Max speed -----------------------------------------------------------
    ns = recs.get("NonSensor") or []
    speed = None
    hint = ""
    if ns and all(k in ns[0] for k in ("e_vel", "n_vel", "u_vel")):
        v = np.sqrt(get_array(ns, "e_vel") ** 2
                    + get_array(ns, "n_vel") ** 2
                    + get_array(ns, "u_vel") ** 2)
        if v.size:
            speed, hint = float(np.max(v)), "navigation filter"
    if speed is None and gnss and all(k in gnss[0] for k in ("vel_e", "vel_n", "vel_u")):
        v = np.sqrt(get_array(gnss, "vel_e") ** 2
                    + get_array(gnss, "vel_n") ** 2
                    + get_array(gnss, "vel_u") ** 2)
        if v.size:
            speed, hint = float(np.max(v)), "GNSS"
    if speed is not None:
        cells.append(_cell("Max speed", speed, "m/s", 1, hint))

    # --- Peak acceleration (boost only) --------------------------------------
    # Measured over launch->burnout on purpose. Touchdown and the ejection
    # charge both slam the accelerometer far harder than the motor does — on the
    # sample flight the whole-flight peak is 146 G at the moment of landing,
    # against 5.1 G under thrust — so a whole-flight max would headline an impact
    # as though it were the rocket's acceleration.
    imu = recs.get("ISM6HG256") or []
    if imu:
        if events["launch"] is not None and events["burnout"] is not None:
            window, hint_window = (events["launch"], events["burnout"]), "under thrust"
        elif events["launch"] is not None and events["apogee"] is not None:
            window, hint_window = (events["launch"], events["apogee"]), "launch to apogee (no burnout detected)"
        else:
            window, hint_window = None, "whole flight — includes ejection and landing impacts"

        t = (get_array(imu, "time_us") - t0) / 1e6
        mask = None if window is None else (t >= window[0]) & (t <= window[1])
        # Low-G part unless it rails in this window; see units of imu.py.
        mag, which = accel_magnitude(recs, flight.sidecar, mask)
        if mag is not None and mag.size:
            peak_g = float(np.max(mag)) / G
            cells.append(_cell("Max acceleration", peak_g, "G", 1,
                               f"{hint_window} · {which}"))

    # --- Timing --------------------------------------------------------------
    if events["launch"] is not None and events["apogee"] is not None:
        cells.append(_cell("Time to apogee", events["apogee"] - events["launch"], "s", 2,
                           "launch to apogee detection"))
    if events["launch"] is not None and events["landed"] is not None:
        cells.append(_cell("Flight time", events["landed"] - events["launch"], "s", 1,
                           "launch to touchdown"))

    # --- Where it came down --------------------------------------------------
    if gnss:
        e, n, _u = gnss_to_enu(gnss)
        if e.size:
            dist = float(math.hypot(e[-1], n[-1]))
            bearing = math.degrees(math.atan2(float(e[-1]), float(n[-1]))) % 360.0
            point = _COMPASS[int((bearing + 11.25) // 22.5) % 16]
            cells.append(_cell(
                "Landing distance", dist, "m", 0,
                f"{bearing:.0f}° from the pad · last GNSS fix",
                suffix=f" {point}",
            ))

    return cells
