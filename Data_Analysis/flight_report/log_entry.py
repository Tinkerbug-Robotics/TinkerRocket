"""The facts a flight-log entry can take from the log itself.

The web tool's flight log asks the flyer for what only they know — the motor,
the field, how it went — and fills the rest from here, so the two columns a
flyer most often gets wrong by hand (which board and which firmware flew) come
from the settings frame the board wrote rather than from memory.

Every value is plain JSON and SI. None means the log does not carry it; the
form leaves that field for the flyer rather than inventing a default.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Any, Optional

import numpy as np

from . import flight_settings as fsd
from .events import measured
from .summary import compute_summary

# Fixes from this long before first motion make up the pad position. Long
# enough to average a receiver's wander on the rail, short enough that a fix
# taken walking out to the pad does not pull it off.
PAD_WINDOW_S = 30.0

# Summary-card labels whose raw value goes into the entry, keyed by the name
# the entry uses. The card is the one place these numbers are computed.
_CARD_VALUES = {
    "Apogee": "apogee_m",
    "Max speed": "max_speed_mps",
    "Max acceleration": "max_accel_g",
    "Flight time": "flight_time_s",
    "Landing distance": "landing_distance_m",
}


def _valid_fixes(gnss: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Fixes with a position: the receiver logs 0/0 until it has one."""
    out = []
    for r in gnss:
        lat, lon = r.get("lat", 0.0), r.get("lon", 0.0)
        if lat and lon and math.isfinite(lat) and math.isfinite(lon) and r.get("fix_mode", 0) >= 2:
            out.append(r)
    return out


def _utc(r: dict[str, Any]) -> Optional[datetime]:
    """The fix's own UTC stamp, or None before the receiver has the date."""
    try:
        if r.get("year", 0) < 2000:
            return None
        return datetime(r["year"], r["month"], r["day"], r["hour"], r["minute"],
                        r["second"], int(r.get("milli_sec", 0)) * 1000, tzinfo=timezone.utc)
    except (KeyError, ValueError, TypeError):
        return None


def _pad(fixes: list[dict[str, Any]], t0_us: int, launch_s: Optional[float]) -> Optional[dict[str, Any]]:
    """Median position of the fixes just before first motion.

    With no launch in the log (a bench recording, or one that missed it) the
    first fixes stand in, which is where the board sat when logging began.
    """
    if not fixes:
        return None
    if launch_s is not None:
        t_launch = t0_us + launch_s * 1e6
        pick = [r for r in fixes if t_launch - PAD_WINDOW_S * 1e6 <= r["time_us"] < t_launch]
        if not pick:
            pick = [r for r in fixes if r["time_us"] < t_launch][-10:]
    else:
        pick = fixes[:20]
    if not pick:
        return None
    return {
        "lat": round(float(np.median([r["lat"] for r in pick])), 6),
        "lon": round(float(np.median([r["lon"] for r in pick])), 6),
        "alt_m": round(float(np.median([r["alt_m"] for r in pick])), 1),
        "fixes": len(pick),
    }


def log_facts(flight) -> dict[str, Any]:
    """What the log says about itself, for a flight-log entry."""
    recs = flight.records
    t0 = flight.t0_us
    events = measured(flight) if t0 is not None else {}
    launch_s = events.get("launch")

    facts: dict[str, Any] = {
        "file": flight.bin_path.name,
        "rocket": flight.rocket_name,
        "date_utc": None,
        "date_source": None,
        "pad": None,
        "board": None,
        "board_asserted": None,
        "firmware": None,
        "firmware_dirty": None,
        "settings_version": None,
    }

    # --- When ---------------------------------------------------------------
    # The receiver's clock beats the filename's: the filename is stamped by
    # whatever clock the board had, which on a cold start is not the date.
    fixes = _valid_fixes(recs.get("GNSS") or [])
    stamped = [(r, _utc(r)) for r in (recs.get("GNSS") or [])]
    stamped = [(r, u) for r, u in stamped if u is not None]
    if stamped and t0 is not None:
        if launch_s is not None:
            t_launch = t0 + launch_s * 1e6
            r, u = min(stamped, key=lambda ru: abs(ru[0]["time_us"] - t_launch))
            u = u.timestamp() + (t_launch - r["time_us"]) / 1e6
            facts["date_utc"] = datetime.fromtimestamp(round(u), tz=timezone.utc).isoformat().replace("+00:00", "Z")
        else:
            facts["date_utc"] = stamped[0][1].replace(microsecond=0).isoformat().replace("+00:00", "Z")
        facts["date_source"] = "gnss"
    elif flight.date is not None:
        facts["date_utc"] = flight.date.isoformat()
        facts["date_source"] = "filename"

    # --- Where --------------------------------------------------------------
    if t0 is not None:
        facts["pad"] = _pad(fixes, t0, launch_s)

    # --- Which computer -----------------------------------------------------
    fs, _warnings = fsd.from_flight(flight)
    if fs is not None:
        facts["settings_version"] = fs["version"]
        facts["firmware"] = fs["fw_git_sha"] or None
        facts["firmware_dirty"] = bool(fs["fw_dirty"])
        code = fs["board_rev_code"]
        name = fsd.board_rev_name(code)
        if name and name != "unknown":
            facts["board"] = name.replace(" (asserted)", "")
            facts["board_asserted"] = bool(code & 0x80)

    # --- How it went, by the numbers ------------------------------------------
    for cell in compute_summary(flight):
        key = _CARD_VALUES.get(cell["label"])
        if key and cell.get("q") is not None:
            facts[key] = round(cell["q"].value, 2)
    return facts
