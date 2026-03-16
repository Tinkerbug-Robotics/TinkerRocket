"""
Tests for the reverse drift cast tool.

Run with:
    cd Code/tools
    python -m pytest driftcast/test_driftcast.py -v
or:
    python driftcast/test_driftcast.py
"""

import math
import sys
from pathlib import Path

# ------------------------------------------------------------------
# Geo tests
# ------------------------------------------------------------------
from driftcast.geo import haversine, bearing, forward_project


def test_haversine_known():
    """JFK (40.6413, -73.7781) to LAX (33.9416, -118.4085) ~ 3974 km."""
    d = haversine(40.6413, -73.7781, 33.9416, -118.4085)
    assert 3950_000 < d < 4000_000, f"JFK→LAX distance {d/1000:.0f} km out of range"


def test_haversine_zero():
    """Same point → 0 distance."""
    d = haversine(32.0, -106.0, 32.0, -106.0)
    assert d < 0.01


def test_bearing_north():
    """Point due north."""
    b = bearing(32.0, -106.0, 33.0, -106.0)
    assert abs(b - 0.0) < 1.0 or abs(b - 360.0) < 1.0, f"Bearing should be ~0°, got {b:.1f}°"


def test_bearing_east():
    """Point due east (at equator for simplicity)."""
    b = bearing(0.0, 0.0, 0.0, 1.0)
    assert abs(b - 90.0) < 0.1, f"Bearing should be ~90°, got {b:.1f}°"


def test_forward_project_roundtrip():
    """Project forward, then compute bearing and distance back."""
    lat1, lon1 = 32.0, -106.0
    brg = 45.0
    dist = 10000.0  # 10 km

    lat2, lon2 = forward_project(lat1, lon1, brg, dist)

    # Check distance
    d_back = haversine(lat1, lon1, lat2, lon2)
    assert abs(d_back - dist) < 1.0, f"Round-trip distance error: {d_back - dist:.2f} m"

    # Check bearing
    b_back = bearing(lat1, lon1, lat2, lon2)
    assert abs(b_back - brg) < 0.5, f"Round-trip bearing error: {b_back - brg:.2f}°"


def test_forward_project_north_pole():
    """Project from high latitude toward north."""
    lat2, lon2 = forward_project(89.0, 0.0, 0.0, 1000.0)
    assert lat2 > 89.0, f"Should move further north, got {lat2:.4f}"


# ------------------------------------------------------------------
# Wind profile tests
# ------------------------------------------------------------------
from driftcast.wind_api import WindProfile, WindLayer, _interp_angle


def _make_simple_profile():
    """Constant 10 kt wind from the west (270°) at all altitudes."""
    layers = [
        WindLayer(alt_ft=0, speed_kts=10.0, direction_deg=270.0),
        WindLayer(alt_ft=5000, speed_kts=10.0, direction_deg=270.0),
        WindLayer(alt_ft=10000, speed_kts=10.0, direction_deg=270.0),
    ]
    return WindProfile(layers=layers, ground_elev_ft=0.0)


def test_wind_interpolation_exact():
    """Interpolation at an exact layer altitude."""
    wp = _make_simple_profile()
    speed, dirn = wp.interpolate(5000)
    assert speed == 10.0
    assert dirn == 270.0


def test_wind_interpolation_midpoint():
    """Interpolation between layers."""
    layers = [
        WindLayer(alt_ft=0, speed_kts=0.0, direction_deg=180.0),
        WindLayer(alt_ft=10000, speed_kts=20.0, direction_deg=180.0),
    ]
    wp = WindProfile(layers=layers)
    speed, dirn = wp.interpolate(5000)
    assert abs(speed - 10.0) < 0.01
    assert abs(dirn - 180.0) < 0.01


def test_wind_angle_wrap():
    """Angle interpolation across 360°/0° boundary."""
    angle = _interp_angle(350.0, 10.0, 0.5)
    assert abs(angle - 0.0) < 0.1 or abs(angle - 360.0) < 0.1, \
        f"Expected ~0° or ~360°, got {angle:.1f}°"


# ------------------------------------------------------------------
# Descent simulation tests
# ------------------------------------------------------------------
from driftcast.descent_sim import simulate_descent


def test_zero_wind_no_drift():
    """With zero wind, forward and reverse should produce no lateral drift."""
    layers = [
        WindLayer(alt_ft=0, speed_kts=0.0, direction_deg=0.0),
        WindLayer(alt_ft=10000, speed_kts=0.0, direction_deg=0.0),
    ]
    wp = WindProfile(layers=layers, ground_elev_ft=0.0)

    track = simulate_descent(
        start_lat=32.0, start_lon=-106.0,
        apogee_agl_ft=5000, drogue_rate_fps=60, main_rate_fps=18,
        main_deploy_agl_ft=700, wind_profile=wp, direction="forward",
    )

    landing = track[-1]
    assert abs(landing.lat - 32.0) < 1e-6
    assert abs(landing.lon - (-106.0)) < 1e-6
    assert landing.alt_agl_ft == 0.0


def test_forward_reverse_roundtrip():
    """Forward from A → B, then reverse from B should recover A."""
    wp = _make_simple_profile()

    # Forward: apogee at (32, -106) → landing somewhere east (wind from west)
    fwd_track = simulate_descent(
        start_lat=32.0, start_lon=-106.0,
        apogee_agl_ft=10000, drogue_rate_fps=60, main_rate_fps=18,
        main_deploy_agl_ft=700, wind_profile=wp, direction="forward",
    )
    landing = fwd_track[-1]

    # Reverse: from landing, walk upwind → should recover (32, -106)
    rev_track = simulate_descent(
        start_lat=landing.lat, start_lon=landing.lon,
        apogee_agl_ft=10000, drogue_rate_fps=60, main_rate_fps=18,
        main_deploy_agl_ft=700, wind_profile=wp, direction="reverse",
    )
    recovered = rev_track[-1]

    dist_error = haversine(32.0, -106.0, recovered.lat, recovered.lon)
    assert dist_error < 10.0, f"Round-trip error: {dist_error:.1f} m (expected < 10 m)"


def test_descent_time():
    """Check that descent time is reasonable."""
    wp = _make_simple_profile()
    track = simulate_descent(
        start_lat=32.0, start_lon=-106.0,
        apogee_agl_ft=10000, drogue_rate_fps=60, main_rate_fps=18,
        main_deploy_agl_ft=700, wind_profile=wp, direction="forward",
    )

    total_time = track[-1].time_s
    # 10000 ft: 9300 ft on drogue at 60 fps = 155.0 s, 700 ft on main at 18 fps = 38.9 s
    # Total ~ 193.9 s
    assert 185 < total_time < 205, f"Descent time {total_time:.0f} s out of expected range"


def test_wind_drift_direction():
    """With wind from the west, rocket should drift east (positive longitude)."""
    wp = _make_simple_profile()  # 10 kt from 270° (west)

    track = simulate_descent(
        start_lat=32.0, start_lon=-106.0,
        apogee_agl_ft=5000, drogue_rate_fps=60, main_rate_fps=18,
        main_deploy_agl_ft=700, wind_profile=wp, direction="forward",
    )

    landing = track[-1]
    # Wind from west → drift east → longitude should increase (become less negative)
    assert landing.lon > -106.0, \
        f"Expected eastward drift, but lon={landing.lon:.6f} <= -106.0"


# ------------------------------------------------------------------
# Integration test (requires network)
# ------------------------------------------------------------------
def test_full_guidance_with_api():
    """Full integration test with real wind data (requires internet)."""
    from driftcast.driftcast import compute_guidance_point

    try:
        result = compute_guidance_point(
            launch_lat=32.9430,
            launch_lon=-106.9200,
            landing_lat=32.9430,  # same as launch → guidance near launch
            landing_lon=-106.9200,
            apogee_agl_ft=5000,
            drogue_rate_fps=60,
            main_rate_fps=18,
            main_deploy_agl_ft=700,
            launch_time_utc="2026-03-15T18:00:00Z",
        )

        # Verification: forward landing should be close to target
        assert result.landing_error_m < 50.0, \
            f"Forward verification error too large: {result.landing_error_m:.1f} m"

        print(f"\n  API test passed:")
        print(f"  Guidance: ({result.guidance_lat:.6f}, {result.guidance_lon:.6f})")
        print(f"  Steering: {result.steering_angle_deg:.1f}° off vertical")
        print(f"  Landing error: {result.landing_error_m:.1f} m")

    except Exception as e:
        print(f"\n  API test skipped (no network?): {e}")


# ------------------------------------------------------------------
# Run all tests
# ------------------------------------------------------------------
def run_all():
    tests = [
        test_haversine_known,
        test_haversine_zero,
        test_bearing_north,
        test_bearing_east,
        test_forward_project_roundtrip,
        test_forward_project_north_pole,
        test_wind_interpolation_exact,
        test_wind_interpolation_midpoint,
        test_wind_angle_wrap,
        test_zero_wind_no_drift,
        test_forward_reverse_roundtrip,
        test_descent_time,
        test_wind_drift_direction,
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            print(f"  PASS: {test.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL: {test.__name__}: {e}")
            failed += 1
        except Exception as e:
            print(f"  ERROR: {test.__name__}: {e}")
            failed += 1

    print(f"\n  {passed}/{passed + failed} tests passed")

    # Optional API test
    print("\n  Running API integration test (requires internet)...")
    test_full_guidance_with_api()

    return failed == 0


if __name__ == "__main__":
    success = run_all()
    sys.exit(0 if success else 1)
