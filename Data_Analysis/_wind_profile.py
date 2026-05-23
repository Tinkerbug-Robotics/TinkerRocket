"""Open-Meteo wind profile fetcher (Python port of DriftCastEngine.fetchWinds).

Pulls hourly pressure-level wind data for a given lat/lon/UTC-hour and
converts it to a layered AGL profile compatible with the descent
simulator.  Caches responses on disk so repeated sweeps don't re-hit the
API.

Mirrors `DriftCastEngine.fetchWinds`:
  - pressure levels 200..1000 hPa
  - units: wind_speed_unit=kn
  - direction = FROM-bearing (degrees, 0=N, clockwise)
  - converts pressure level → AGL altitude via ISA troposphere model
"""

from __future__ import annotations

import json
import math
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

# Same pressure ladder as DriftCastEngine.swift:171
PRESSURE_LEVELS_HPA = [1000, 975, 950, 925, 900, 850, 800,
                       700, 600, 500, 400, 300, 250, 200]

FT_PER_M = 3.28084
M_PER_FT = 1.0 / FT_PER_M
KT_TO_MPS = 0.514444

# ISA constants (troposphere)
_P0 = 101325.0
_T0 = 288.15
_L = 0.0065
_G = 9.80665
_M = 0.0289644
_R = 8.31447


def _pressure_to_altitude_m(p_hpa: float) -> float:
    """Convert pressure (hPa) to geometric altitude (m MSL), troposphere only."""
    p_pa = p_hpa * 100.0
    exponent = _R * _L / (_G * _M)
    return (_T0 / _L) * (1.0 - (p_pa / _P0) ** exponent)


@dataclass
class WindLayer:
    alt_ft_agl: float
    speed_kts: float
    dir_from_deg: float   # FROM direction (0=N, clockwise)


@dataclass
class WindProfile:
    layers: list[WindLayer] = field(default_factory=list)
    ground_elev_ft: float = 0.0
    fetch_time_utc: str = ""
    location: tuple[float, float] = (0.0, 0.0)
    source: str = ""

    def interpolate(self, alt_ft_agl: float) -> tuple[float, float]:
        """(speed_kts, dir_from_deg) at given AGL altitude."""
        if not self.layers:
            return (0.0, 0.0)
        if alt_ft_agl <= self.layers[0].alt_ft_agl:
            return (self.layers[0].speed_kts, self.layers[0].dir_from_deg)
        if alt_ft_agl >= self.layers[-1].alt_ft_agl:
            return (self.layers[-1].speed_kts, self.layers[-1].dir_from_deg)
        for i in range(len(self.layers) - 1):
            lo, hi = self.layers[i], self.layers[i + 1]
            if lo.alt_ft_agl <= alt_ft_agl <= hi.alt_ft_agl:
                t = ((alt_ft_agl - lo.alt_ft_agl) /
                     max(1e-6, (hi.alt_ft_agl - lo.alt_ft_agl)))
                speed = lo.speed_kts + t * (hi.speed_kts - lo.speed_kts)
                # Angle interp with wraparound
                diff = ((hi.dir_from_deg - lo.dir_from_deg + 180.0) % 360.0) - 180.0
                d = (lo.dir_from_deg + t * diff) % 360.0
                if d < 0:
                    d += 360.0
                return (speed, d)
        return (self.layers[-1].speed_kts, self.layers[-1].dir_from_deg)


def _build_url(endpoint: str, lat: float, lon: float, date_utc: str) -> str:
    """endpoint: 'forecast' or 'archive'."""
    speed_vars = ",".join(f"wind_speed_{p}hPa" for p in PRESSURE_LEVELS_HPA)
    dir_vars = ",".join(f"wind_direction_{p}hPa" for p in PRESSURE_LEVELS_HPA)
    base = ("https://archive-api.open-meteo.com/v1/archive"
            if endpoint == "archive"
            else "https://api.open-meteo.com/v1/forecast")
    params = {
        "latitude": f"{lat}",
        "longitude": f"{lon}",
        "hourly": f"{speed_vars},{dir_vars}",
        "wind_speed_unit": "kn",
        "start_date": date_utc,
        "end_date": date_utc,
        "timezone": "UTC",
    }
    return base + "?" + urllib.parse.urlencode(params)


def _parse_response(json_data: dict, hour: int,
                    ground_elev_ft: Optional[float] = None) -> WindProfile:
    hourly = json_data.get("hourly")
    if not hourly or "time" not in hourly:
        raise ValueError("Open-Meteo response missing 'hourly.time'")
    times = hourly["time"]
    if not times:
        raise ValueError("Open-Meteo response has empty time series")
    idx = min(hour, len(times) - 1)

    api_elev_m = float(json_data.get("elevation", 0.0))
    ground_elev_ft_final = (ground_elev_ft if ground_elev_ft is not None
                            else api_elev_m * FT_PER_M)
    ground_elev_m = ground_elev_ft_final * M_PER_FT

    layers: list[WindLayer] = []
    for p in PRESSURE_LEVELS_HPA:
        sp_arr = hourly.get(f"wind_speed_{p}hPa")
        dr_arr = hourly.get(f"wind_direction_{p}hPa")
        if not sp_arr or not dr_arr or idx >= len(sp_arr) or idx >= len(dr_arr):
            continue
        sp = sp_arr[idx]
        dr = dr_arr[idx]
        if sp is None or dr is None:
            continue
        try:
            sp = float(sp)
            dr = float(dr)
        except (TypeError, ValueError):
            continue
        if not (0 <= sp < 500 and 0 <= dr <= 360):
            continue
        alt_msl_m = _pressure_to_altitude_m(p)
        alt_agl_ft = (alt_msl_m - ground_elev_m) * FT_PER_M
        if alt_agl_ft < -100:
            continue
        layers.append(WindLayer(alt_ft_agl=max(0.0, alt_agl_ft),
                                speed_kts=sp, dir_from_deg=dr))

    layers.sort(key=lambda L: L.alt_ft_agl)

    # Ensure a near-surface layer (matches Swift behavior)
    if layers and layers[0].alt_ft_agl > 100:
        first = layers[0]
        layers.insert(0, WindLayer(alt_ft_agl=0.0,
                                    speed_kts=first.speed_kts * 0.7,
                                    dir_from_deg=first.dir_from_deg))

    return WindProfile(layers=layers,
                        ground_elev_ft=ground_elev_ft_final,
                        fetch_time_utc=f"{times[idx]}Z",
                        location=(0.0, 0.0),  # filled in by caller
                        source="")


def fetch_wind(lat: float, lon: float, when_utc: datetime,
               ground_elev_ft: Optional[float] = None,
               cache_dir: Optional[Path] = None) -> WindProfile:
    """Fetch a wind profile for the given lat/lon/UTC time.

    Tries the archive API first (canonical for past dates), falls back to
    the forecast endpoint if archive is empty or returns 404 (recent dates
    that ERA5 hasn't ingested yet).

    Caches the raw JSON response in `cache_dir` (or no cache if None).
    """
    if when_utc.tzinfo is None:
        when_utc = when_utc.replace(tzinfo=timezone.utc)
    when_utc = when_utc.astimezone(timezone.utc)

    date_utc = when_utc.strftime("%Y-%m-%d")
    hour = when_utc.hour

    cache_key = f"wind_{lat:.4f}_{lon:.4f}_{date_utc}.json"
    cache_path = (cache_dir / cache_key) if cache_dir else None

    if cache_path and cache_path.exists():
        with open(cache_path) as f:
            payload = json.load(f)
        endpoint = payload.get("__endpoint__", "archive")
        json_data = payload.get("__data__", payload)
    else:
        json_data = None
        endpoint_used = None
        last_err = None
        for ep in ("archive", "forecast"):
            url = _build_url(ep, lat, lon, date_utc)
            try:
                req = urllib.request.Request(
                    url, headers={"User-Agent": "TinkerRocket-DriftCast/1.0"})
                with urllib.request.urlopen(req, timeout=20) as resp:
                    data = resp.read()
                    json_data = json.loads(data)
                # Some endpoints return an empty 'time' even on 200 — count
                # that as a miss so we fall through to the other endpoint.
                if (json_data.get("hourly") and
                        json_data["hourly"].get("time")):
                    endpoint_used = ep
                    break
            except Exception as e:
                last_err = e
        if json_data is None or endpoint_used is None:
            raise RuntimeError(f"Open-Meteo fetch failed: {last_err}")

        if cache_path:
            cache_path.parent.mkdir(parents=True, exist_ok=True)
            with open(cache_path, "w") as f:
                json.dump({"__endpoint__": endpoint_used,
                           "__data__": json_data}, f, indent=2)
        endpoint = endpoint_used

    profile = _parse_response(json_data, hour, ground_elev_ft)
    profile.location = (lat, lon)
    profile.source = f"open-meteo/{endpoint}"
    return profile
