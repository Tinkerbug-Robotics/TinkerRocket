"""
Geodesic utility functions for drift cast calculations.

Matches LocationManager.swift haversine/bearing implementations.
forward_project uses the Vincenty direct formula for great-circle projection.
"""

import math

# WGS-84 Earth radius (mean), meters
EARTH_RADIUS_M = 6371000.0


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distance between two points in meters (haversine formula).

    Args:
        lat1, lon1: Point 1 in decimal degrees.
        lat2, lon2: Point 2 in decimal degrees.

    Returns:
        Distance in meters.
    """
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1

    a = (math.sin(dlat / 2) ** 2
         + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_M * c


def bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from point 1 to point 2.

    Args:
        lat1, lon1: Origin in decimal degrees.
        lat2, lon2: Destination in decimal degrees.

    Returns:
        Bearing in degrees [0, 360).
    """
    rlat1, rlon1 = math.radians(lat1), math.radians(lon1)
    rlat2, rlon2 = math.radians(lat2), math.radians(lon2)
    dlon = rlon2 - rlon1

    x = math.sin(dlon) * math.cos(rlat2)
    y = (math.cos(rlat1) * math.sin(rlat2)
         - math.sin(rlat1) * math.cos(rlat2) * math.cos(dlon))

    brng = math.degrees(math.atan2(x, y))
    return brng % 360.0


def forward_project(lat: float, lon: float,
                    bearing_deg: float, distance_m: float) -> tuple[float, float]:
    """Project a point forward along a great-circle arc.

    Given a starting lat/lon, bearing, and distance, compute the destination.
    Uses the spherical-earth direct formula (Vincenty on sphere).

    Args:
        lat, lon: Starting point in decimal degrees.
        bearing_deg: Bearing in degrees (0 = north, 90 = east).
        distance_m: Distance in meters.

    Returns:
        (lat2, lon2) in decimal degrees.
    """
    rlat = math.radians(lat)
    rlon = math.radians(lon)
    rbrng = math.radians(bearing_deg)
    d_over_r = distance_m / EARTH_RADIUS_M

    lat2 = math.asin(
        math.sin(rlat) * math.cos(d_over_r)
        + math.cos(rlat) * math.sin(d_over_r) * math.cos(rbrng)
    )
    lon2 = rlon + math.atan2(
        math.sin(rbrng) * math.sin(d_over_r) * math.cos(rlat),
        math.cos(d_over_r) - math.sin(rlat) * math.sin(lat2)
    )

    return math.degrees(lat2), math.degrees(lon2)


def ft_to_m(ft: float) -> float:
    """Convert feet to meters."""
    return ft * 0.3048


def m_to_ft(m: float) -> float:
    """Convert meters to feet."""
    return m / 0.3048


def kts_to_mps(kts: float) -> float:
    """Convert knots to meters per second."""
    return kts * 0.514444


def fps_to_mps(fps: float) -> float:
    """Convert feet per second to meters per second."""
    return fps * 0.3048
