"""
Main orchestration for the reverse drift cast guidance point calculator.

Given a launch pad and desired landing point, computes the apogee guidance
point the rocket should steer toward so that wind drift during parachute
descent carries it to the target landing location.
"""

import math
from dataclasses import dataclass
from datetime import datetime

from .geo import haversine, bearing, ft_to_m
from .wind_api import fetch_winds, WindProfile
from .descent_sim import simulate_descent, TrackPoint


@dataclass
class GuidanceResult:
    """Result of a reverse drift cast computation."""

    # Apogee guidance point
    guidance_lat: float
    guidance_lon: float

    # Steering requirements
    steering_angle_deg: float    # off-vertical angle from pad to guidance point
    steering_bearing_deg: float  # compass heading from pad toward guidance point

    # Feasibility
    feasible: bool
    infeasible_reason: str | None

    # Descent track (forward verification: guidance point → ground)
    descent_track: list[TrackPoint]

    # Forward verification landing point
    forward_landing_lat: float
    forward_landing_lon: float
    landing_error_m: float       # distance between forward landing and target

    # Wind profile used
    wind_profile: WindProfile

    # Inputs echoed back
    launch_lat: float
    launch_lon: float
    landing_lat: float
    landing_lon: float
    apogee_ft: float

    # Misc
    total_descent_time_s: float
    total_drift_m: float  # horizontal distance from guidance point to landing


def compute_guidance_point(
    launch_lat: float,
    launch_lon: float,
    landing_lat: float,
    landing_lon: float,
    apogee_agl_ft: float,
    drogue_rate_fps: float,
    main_rate_fps: float,
    main_deploy_agl_ft: float,
    launch_time_utc: datetime | str,
    max_steering_deg: float = 15.0,
    ground_elev_ft: float | None = None,
    wind_profile: WindProfile | None = None,
) -> GuidanceResult:
    """Compute the apogee guidance point for a desired landing location.

    Algorithm:
        1. Fetch wind profile for the landing area.
        2. Reverse descent simulation: from landing point, walk upwind through
           each altitude layer to find the apogee position.
        3. Check feasibility: is the guidance point reachable from the pad
           with an acceptable steering angle?
        4. Forward verification: simulate descent from guidance point to
           verify it lands near the target.

    Args:
        launch_lat, launch_lon: Pad location (decimal degrees).
        landing_lat, landing_lon: Desired landing location (decimal degrees).
        apogee_agl_ft: Target apogee altitude AGL (feet).
        drogue_rate_fps: Drogue descent rate (feet/second).
        main_rate_fps: Main parachute descent rate (feet/second).
        main_deploy_agl_ft: Main deployment altitude AGL (feet).
        launch_time_utc: Forecast time (datetime or ISO string).
        max_steering_deg: Maximum allowed steering angle off vertical (degrees).
        ground_elev_ft: Ground elevation ASL (feet). None = use API value.
        wind_profile: Pre-fetched WindProfile. None = fetch from API.

    Returns:
        GuidanceResult with guidance point, feasibility, and verification.
    """
    # Step 0: Fetch winds if not provided
    if wind_profile is None:
        wind_profile = fetch_winds(landing_lat, landing_lon, launch_time_utc,
                                   ground_elev_ft)

    # Step 1: Reverse descent — landing → apogee guidance point
    reverse_track = simulate_descent(
        start_lat=landing_lat,
        start_lon=landing_lon,
        apogee_agl_ft=apogee_agl_ft,
        drogue_rate_fps=drogue_rate_fps,
        main_rate_fps=main_rate_fps,
        main_deploy_agl_ft=main_deploy_agl_ft,
        wind_profile=wind_profile,
        direction="reverse",
    )

    # The last point in reverse track is the guidance point at apogee
    guidance_pt = reverse_track[-1]
    guidance_lat = guidance_pt.lat
    guidance_lon = guidance_pt.lon

    # Step 2: Feasibility check
    horiz_dist_m = haversine(launch_lat, launch_lon, guidance_lat, guidance_lon)
    apogee_m = ft_to_m(apogee_agl_ft)
    steering_angle = math.degrees(math.atan2(horiz_dist_m, apogee_m))
    steering_brg = bearing(launch_lat, launch_lon, guidance_lat, guidance_lon)

    feasible = steering_angle <= max_steering_deg
    infeasible_reason = None
    if not feasible:
        infeasible_reason = (
            f"Steering angle {steering_angle:.1f}° exceeds maximum {max_steering_deg:.1f}°. "
            f"The guidance point is {horiz_dist_m:.0f} m from the pad at bearing {steering_brg:.0f}°. "
            f"Try a closer landing point, higher apogee, or increase max_steering_deg."
        )

    # Step 3: Forward verification — guidance point → ground
    forward_track = simulate_descent(
        start_lat=guidance_lat,
        start_lon=guidance_lon,
        apogee_agl_ft=apogee_agl_ft,
        drogue_rate_fps=drogue_rate_fps,
        main_rate_fps=main_rate_fps,
        main_deploy_agl_ft=main_deploy_agl_ft,
        wind_profile=wind_profile,
        direction="forward",
    )

    # Verify landing accuracy
    fwd_landing = forward_track[-1]
    landing_error_m = haversine(landing_lat, landing_lon,
                                fwd_landing.lat, fwd_landing.lon)

    # Total drift from guidance to landing
    total_drift_m = haversine(guidance_lat, guidance_lon,
                              fwd_landing.lat, fwd_landing.lon)

    total_time = forward_track[-1].time_s if forward_track else 0.0

    return GuidanceResult(
        guidance_lat=guidance_lat,
        guidance_lon=guidance_lon,
        steering_angle_deg=steering_angle,
        steering_bearing_deg=steering_brg,
        feasible=feasible,
        infeasible_reason=infeasible_reason,
        descent_track=forward_track,
        forward_landing_lat=fwd_landing.lat,
        forward_landing_lon=fwd_landing.lon,
        landing_error_m=landing_error_m,
        wind_profile=wind_profile,
        launch_lat=launch_lat,
        launch_lon=launch_lon,
        landing_lat=landing_lat,
        landing_lon=landing_lon,
        apogee_ft=apogee_agl_ft,
        total_descent_time_s=total_time,
        total_drift_m=total_drift_m,
    )


def print_result(result: GuidanceResult) -> None:
    """Pretty-print a guidance result."""
    print("=" * 50)
    print("  REVERSE DRIFT CAST — GUIDANCE RESULT")
    print("=" * 50)
    print(f"  Launch pad:     {result.launch_lat:.6f}°N, {abs(result.launch_lon):.6f}°W")
    print(f"  Landing target: {result.landing_lat:.6f}°N, {abs(result.landing_lon):.6f}°W")
    print(f"  Apogee:         {result.apogee_ft:,.0f} ft AGL")
    print()
    print(f"  Guidance point: {result.guidance_lat:.6f}°N, {abs(result.guidance_lon):.6f}°W")
    print(f"  Steering:       {result.steering_angle_deg:.1f}° off vertical, "
          f"bearing {result.steering_bearing_deg:.0f}°")
    status = "FEASIBLE" if result.feasible else "INFEASIBLE"
    print(f"  Status:         {status}")
    if result.infeasible_reason:
        print(f"  Reason:         {result.infeasible_reason}")
    print()
    print(f"  Forward verification error: {result.landing_error_m:.1f} m")
    print(f"  Total descent time:         {result.total_descent_time_s:.0f} s "
          f"({result.total_descent_time_s/60:.1f} min)")
    print(f"  Total wind drift:           {result.total_drift_m:.0f} m")
    print("=" * 50)
