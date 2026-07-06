"""Unit tests for the roll-profile target reconstruction in the flight-report
roll_pid module — both firmware semantics generations:

* "step" (pre-v4): inside segment [i, i+1) the controller commands waypoint
  i+1's angle as a STEP, using waypoint i's per-waypoint mode.
* "ramp" (v4+): pure (time, angle) waypoints — null-rate before the first
  waypoint, target lerped along the shortest wrapped arc between waypoints,
  last angle held after the profile.

These must mirror firmware `roll_profile_query()` (flight_computer main.cpp)
exactly; the analysis reconstructs the commanded target from them.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "Data_Analysis"))

from flight_report.modules.roll_pid import (  # noqa: E402
    _effective_plan,
    _profile_target,
)

# (time_s, angle_deg, is_null_rate) — as _parse_profile produces
RAMP_WPS = [(1.5, 0.0, False), (3.0, 90.0, False), (5.0, 180.0, False)]
STEP_WPS = [(1.5, 0.0, True), (3.0, 90.0, False), (5.0, 180.0, True)]


class TestRampSemantics:
    def test_null_rate_before_first_waypoint(self):
        tgt, is_ang = _profile_target(RAMP_WPS, 0.5, "ramp")
        assert not is_ang

    def test_lerp_between_waypoints(self):
        assert _profile_target(RAMP_WPS, 2.25, "ramp") == (45.0, True)
        assert _profile_target(RAMP_WPS, 4.0, "ramp") == (135.0, True)

    def test_waypoints_hit_exactly(self):
        assert _profile_target(RAMP_WPS, 1.5, "ramp") == (0.0, True)
        assert _profile_target(RAMP_WPS, 3.0, "ramp") == (90.0, True)

    def test_hold_after_last_waypoint(self):
        assert _profile_target(RAMP_WPS, 7.0, "ramp") == (180.0, True)

    def test_equal_waypoints_hold(self):
        wps = [(1.0, 45.0, False), (3.0, 45.0, False)]
        assert _profile_target(wps, 2.0, "ramp") == (45.0, True)

    def test_wrap_shortest_path(self):
        # 170° → -170° must cross the ±180 seam (Δ=+20°), not sweep through 0.
        wps = [(0.0, 170.0, False), (2.0, -170.0, False)]
        tgt, _ = _profile_target(wps, 0.5, "ramp")
        assert tgt == pytest.approx(175.0)
        tgt, _ = _profile_target(wps, 1.5, "ramp")
        assert tgt == pytest.approx(-175.0)

    def test_zero_duration_segment(self):
        wps = [(1.0, 0.0, False), (1.0, 90.0, False)]
        tgt, is_ang = _profile_target(wps, 1.0, "ramp")
        assert is_ang

    def test_per_waypoint_null_flags_ignored(self):
        # Legacy null_rate flags must not matter under ramp semantics.
        assert _profile_target(STEP_WPS, 2.25, "ramp") == (45.0, True)

    def test_effective_plan(self):
        plan = _effective_plan(RAMP_WPS, "ramp")
        assert plan == "<1.5s null-rate; 1.5–3s ramp 0→90°; 3–5s ramp 90→180°; ≥5s hold 180°"


class TestStepSemantics:
    """Pre-v4 firmware: unchanged behavior — regression-guard the old parser."""

    def test_holds_first_waypoint_before_profile(self):
        tgt, is_ang = _profile_target(STEP_WPS, 0.5, "step")
        assert (tgt, is_ang) == (0.0, False)  # null-rate first waypoint

    def test_steps_to_next_angle_with_current_mode(self):
        # [1.5, 3): current wp null_rate → null, regardless of next angle
        assert _profile_target(STEP_WPS, 2.0, "step")[1] is False
        # [3, 5): current wp angle-mode → command NEXT waypoint's angle (180)
        assert _profile_target(STEP_WPS, 4.0, "step") == (180.0, True)

    def test_holds_last_waypoint_after_profile(self):
        tgt, is_ang = _profile_target(STEP_WPS, 6.0, "step")
        assert (tgt, is_ang) == (180.0, False)  # null-rate last waypoint

    def test_default_semantics_is_step(self):
        assert _profile_target(STEP_WPS, 4.0) == (180.0, True)

    def test_effective_plan(self):
        plan = _effective_plan(STEP_WPS, "step")
        assert plan == "<1.5s null-rate; 1.5–3s null-rate; 3–5s cmd 180°; ≥5s null-rate"
