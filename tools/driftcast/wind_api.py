"""
Wind data fetching from Open-Meteo pressure-level API.

Open-Meteo provides winds at standard pressure levels (1000–30 hPa).
We convert pressure → geometric altitude using the standard atmosphere model,
then build a WindProfile with altitude-indexed wind speed/direction for
interpolation during descent simulation.
"""

import math
import ssl
import urllib.error
import urllib.request
import json
from dataclasses import dataclass, field
from datetime import datetime, timezone


# Pressure levels to query (hPa), from surface to high altitude.
# These correspond roughly to 100 m .. 16 km.
PRESSURE_LEVELS_HPA = [
    1000, 975, 950, 925, 900, 850, 800, 700, 600, 500, 400, 300, 250, 200
]

# Standard atmosphere constants
P0 = 101325.0  # Pa (sea-level pressure)
T0 = 288.15    # K  (sea-level temperature)
L  = 0.0065    # K/m (lapse rate in troposphere)
G  = 9.80665   # m/s^2
M  = 0.0289644 # kg/mol (molar mass of air)
R  = 8.31447   # J/(mol·K) (universal gas constant)


def pressure_to_altitude_m(p_hpa: float) -> float:
    """Convert pressure (hPa) to geometric altitude (m ASL) using standard atmosphere.

    Valid in the troposphere (up to ~11 km / 36,000 ft).
    """
    p_pa = p_hpa * 100.0
    # Hypsometric formula for troposphere:
    # h = (T0 / L) * (1 - (p / P0)^(R*L / (g*M)))
    exponent = R * L / (G * M)
    return (T0 / L) * (1.0 - (p_pa / P0) ** exponent)


def altitude_m_to_ft(alt_m: float) -> float:
    """Convert meters to feet."""
    return alt_m / 0.3048


@dataclass
class WindLayer:
    """Wind at a single altitude."""
    alt_ft: float       # altitude AGL (feet)
    speed_kts: float    # wind speed (knots)
    direction_deg: float  # wind FROM direction (degrees, 0=N)


@dataclass
class WindProfile:
    """Wind profile from surface to upper atmosphere."""
    layers: list[WindLayer] = field(default_factory=list)
    ground_elev_ft: float = 0.0  # ground elevation ASL (feet)
    fetch_time: str = ""          # ISO time string used for the forecast
    location: tuple[float, float] = (0.0, 0.0)  # (lat, lon)

    def interpolate(self, alt_agl_ft: float) -> tuple[float, float]:
        """Interpolate wind speed (kts) and direction (deg) at a given AGL altitude.

        Args:
            alt_agl_ft: Altitude above ground level in feet.

        Returns:
            (speed_kts, direction_deg) interpolated between nearest layers.
        """
        if not self.layers:
            return 0.0, 0.0

        # Clamp to available range
        if alt_agl_ft <= self.layers[0].alt_ft:
            return self.layers[0].speed_kts, self.layers[0].direction_deg
        if alt_agl_ft >= self.layers[-1].alt_ft:
            return self.layers[-1].speed_kts, self.layers[-1].direction_deg

        # Find bracketing layers
        for i in range(len(self.layers) - 1):
            lo = self.layers[i]
            hi = self.layers[i + 1]
            if lo.alt_ft <= alt_agl_ft <= hi.alt_ft:
                if hi.alt_ft == lo.alt_ft:
                    t = 0.0
                else:
                    t = (alt_agl_ft - lo.alt_ft) / (hi.alt_ft - lo.alt_ft)

                speed = lo.speed_kts + t * (hi.speed_kts - lo.speed_kts)
                direction = _interp_angle(lo.direction_deg, hi.direction_deg, t)
                return speed, direction

        # Fallback (shouldn't reach here)
        return self.layers[-1].speed_kts, self.layers[-1].direction_deg


def _interp_angle(a1: float, a2: float, t: float) -> float:
    """Linearly interpolate between two angles (degrees), handling wraparound."""
    diff = ((a2 - a1 + 180) % 360) - 180  # shortest arc
    return (a1 + t * diff) % 360


def fetch_winds(lat: float, lon: float, time_utc: datetime | str,
                ground_elev_ft: float | None = None) -> WindProfile:
    """Fetch wind profile from Open-Meteo pressure-level API.

    Args:
        lat, lon: Location in decimal degrees.
        time_utc: Forecast time as datetime or ISO string.
        ground_elev_ft: Ground elevation ASL in feet. If None, uses API's elevation.

    Returns:
        WindProfile with layers sorted by altitude.
    """
    if isinstance(time_utc, datetime):
        # Extract just the date for the API
        date_str = time_utc.strftime("%Y-%m-%d")
        hour = time_utc.hour
    else:
        # Parse ISO string
        dt = datetime.fromisoformat(time_utc.replace("Z", "+00:00"))
        date_str = dt.strftime("%Y-%m-%d")
        hour = dt.hour

    # Build variable list for all pressure levels
    speed_vars = ",".join(f"wind_speed_{p}hPa" for p in PRESSURE_LEVELS_HPA)
    dir_vars = ",".join(f"wind_direction_{p}hPa" for p in PRESSURE_LEVELS_HPA)

    url = (
        f"https://api.open-meteo.com/v1/forecast"
        f"?latitude={lat}&longitude={lon}"
        f"&hourly={speed_vars},{dir_vars}"
        f"&wind_speed_unit=kn"
        f"&start_date={date_str}&end_date={date_str}"
        f"&timezone=UTC"
    )

    req = urllib.request.Request(url)
    req.add_header("User-Agent", "TinkerRocket-DriftCast/1.0")

    # Create SSL context (handles systems with outdated certificates)
    ctx = ssl.create_default_context()
    try:
        with urllib.request.urlopen(req, timeout=15, context=ctx) as resp:
            data = json.loads(resp.read().decode())
    except (ssl.SSLCertVerificationError, urllib.error.URLError):
        # Fallback: unverified context for macOS missing certs
        ctx = ssl._create_unverified_context()
        with urllib.request.urlopen(req, timeout=15, context=ctx) as resp:
            data = json.loads(resp.read().decode())

    # API returns elevation in meters
    api_elev_m = data.get("elevation", 0.0)
    if ground_elev_ft is None:
        ground_elev_ft = altitude_m_to_ft(api_elev_m)

    ground_elev_m = api_elev_m if ground_elev_ft is None else ground_elev_ft * 0.3048

    # Find the hourly index closest to requested hour
    times = data["hourly"]["time"]  # list of "YYYY-MM-DDTHH:MM" strings
    idx = min(hour, len(times) - 1)

    # Build layers from each pressure level
    layers = []
    for p in PRESSURE_LEVELS_HPA:
        speed_key = f"wind_speed_{p}hPa"
        dir_key = f"wind_direction_{p}hPa"

        speed_val = data["hourly"].get(speed_key, [None])[idx]
        dir_val = data["hourly"].get(dir_key, [None])[idx]

        if speed_val is None or dir_val is None:
            continue

        alt_msl_m = pressure_to_altitude_m(p)
        alt_agl_ft = altitude_m_to_ft(alt_msl_m - ground_elev_m)

        # Skip layers below ground or with negative AGL
        if alt_agl_ft < -100:
            continue

        layers.append(WindLayer(
            alt_ft=max(0.0, alt_agl_ft),
            speed_kts=float(speed_val),
            direction_deg=float(dir_val),
        ))

    # Sort by altitude and deduplicate
    layers.sort(key=lambda l: l.alt_ft)

    # Ensure a surface layer exists (extrapolate from lowest available)
    if layers and layers[0].alt_ft > 100:
        layers.insert(0, WindLayer(
            alt_ft=0.0,
            speed_kts=layers[0].speed_kts * 0.7,  # rough surface reduction
            direction_deg=layers[0].direction_deg,
        ))

    return WindProfile(
        layers=layers,
        ground_elev_ft=ground_elev_ft,
        fetch_time=f"{date_str}T{hour:02d}:00Z",
        location=(lat, lon),
    )


def print_wind_profile(wp: WindProfile) -> None:
    """Pretty-print a wind profile."""
    print(f"Wind profile for ({wp.location[0]:.4f}, {wp.location[1]:.4f})")
    print(f"  Ground elevation: {wp.ground_elev_ft:.0f} ft ASL")
    print(f"  Forecast time:    {wp.fetch_time}")
    print(f"  {'Alt AGL (ft)':>14s}  {'Speed (kts)':>12s}  {'From (deg)':>11s}")
    print(f"  {'-'*14}  {'-'*12}  {'-'*11}")
    for layer in wp.layers:
        print(f"  {layer.alt_ft:>14.0f}  {layer.speed_kts:>12.1f}  {layer.direction_deg:>11.0f}")
