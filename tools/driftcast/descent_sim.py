"""
Layer-by-layer descent simulation for drift cast calculations.

Supports both forward (apogee → ground, drift downwind) and
reverse (ground → apogee, walk upwind) directions.

The reverse simulation starts at the desired landing point and traces
upwind through each altitude layer to find the apogee guidance point.
"""

from dataclasses import dataclass
from .geo import forward_project, kts_to_mps, ft_to_m
from .wind_api import WindProfile


# Default layer thickness for descent simulation
LAYER_THICKNESS_FT = 1000.0


@dataclass
class TrackPoint:
    """A single point on the descent/ascent track."""
    lat: float          # decimal degrees
    lon: float          # decimal degrees
    alt_agl_ft: float   # altitude AGL in feet
    time_s: float       # elapsed time from start of simulation (seconds)


def simulate_descent(
    start_lat: float,
    start_lon: float,
    apogee_agl_ft: float,
    drogue_rate_fps: float,
    main_rate_fps: float,
    main_deploy_agl_ft: float,
    wind_profile: WindProfile,
    direction: str = "forward",
    layer_thickness_ft: float = LAYER_THICKNESS_FT,
) -> list[TrackPoint]:
    """Simulate parachute descent through wind layers.

    Args:
        start_lat, start_lon: Starting position.
            - forward: this is the apogee position (rocket descends from here).
            - reverse: this is the landing position (we trace upwind from here).
        apogee_agl_ft: Apogee altitude above ground level (feet).
        drogue_rate_fps: Drogue parachute descent rate (feet/second, positive = descending).
        main_rate_fps: Main parachute descent rate (feet/second, positive = descending).
        main_deploy_agl_ft: Altitude AGL at which main deploys (feet).
        wind_profile: WindProfile from wind_api.
        direction: 'forward' (apogee → ground) or 'reverse' (ground → apogee).
        layer_thickness_ft: Altitude step per layer (feet).

    Returns:
        List of TrackPoints from start to end.
        - forward: first point is apogee, last is ground.
        - reverse: first point is ground (landing), last is apogee (guidance point).
    """
    if direction not in ("forward", "reverse"):
        raise ValueError(f"direction must be 'forward' or 'reverse', got '{direction}'")

    track = []
    time_s = 0.0
    lat, lon = start_lat, start_lon

    # Build altitude breakpoints that include the main deploy boundary
    # so we don't skip over the drogue→main transition.
    def _build_alt_steps(low_ft, high_ft, step_ft, deploy_ft):
        """Generate altitude step boundaries from low to high, splitting at deploy_ft."""
        alts = []
        a = low_ft
        while a < high_ft:
            next_a = min(a + step_ft, high_ft)
            # If this step straddles the deploy boundary, split it
            if a < deploy_ft < next_a:
                alts.append((a, deploy_ft))
                alts.append((deploy_ft, next_a))
            else:
                alts.append((a, next_a))
            a = next_a
        return alts

    if direction == "forward":
        # Descend from apogee to ground
        alt = apogee_agl_ft
        track.append(TrackPoint(lat, lon, alt, time_s))

        # Build steps from ground (0) up to apogee, then iterate in reverse
        steps = _build_alt_steps(0.0, apogee_agl_ft, layer_thickness_ft, main_deploy_agl_ft)
        for step_lo, step_hi in reversed(steps):
            step = step_hi - step_lo
            mid_alt = (step_lo + step_hi) / 2.0

            # Pick descent rate: above main_deploy → drogue, below → main
            descent_rate = drogue_rate_fps if mid_alt > main_deploy_agl_ft else main_rate_fps

            # Time to descend through this layer (ft / fps = seconds)
            dt = step / descent_rate

            # Wind at mid-layer altitude
            speed_kts, from_dir = wind_profile.interpolate(mid_alt)
            speed_mps = kts_to_mps(speed_kts)

            # Drift distance
            drift_m = speed_mps * dt

            # Forward: drift downwind (wind blows FROM from_dir, so drift TO from_dir + 180)
            drift_bearing = (from_dir + 180.0) % 360.0

            lat, lon = forward_project(lat, lon, drift_bearing, drift_m)
            alt = step_lo
            time_s += dt
            track.append(TrackPoint(lat, lon, alt, time_s))

    else:
        # Reverse: start at ground (landing), walk upwind to apogee
        alt = 0.0
        track.append(TrackPoint(lat, lon, alt, time_s))

        steps = _build_alt_steps(0.0, apogee_agl_ft, layer_thickness_ft, main_deploy_agl_ft)
        for step_lo, step_hi in steps:
            step = step_hi - step_lo
            mid_alt = (step_lo + step_hi) / 2.0

            # Pick descent rate: above main_deploy → drogue, below → main
            descent_rate = drogue_rate_fps if mid_alt > main_deploy_agl_ft else main_rate_fps

            # Time spent in this layer during descent (ft / fps = seconds)
            dt = step / descent_rate

            # Wind at mid-layer altitude
            speed_kts, from_dir = wind_profile.interpolate(mid_alt)
            speed_mps = kts_to_mps(speed_kts)

            # Drift distance
            drift_m = speed_mps * dt

            # Reverse: move upwind (INTO the wind = toward the FROM direction)
            drift_bearing = from_dir

            lat, lon = forward_project(lat, lon, drift_bearing, drift_m)
            alt = step_hi
            time_s += dt
            track.append(TrackPoint(lat, lon, alt, time_s))

    return track
