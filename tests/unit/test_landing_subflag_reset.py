"""Regression test for issue #192.

The landing sub-detectors in TR_KinematicChecks accumulate evidence at 1 Hz
regardless of flight state, so a rocket flying straight pre-apogee (low roll
rate, ~0 m/s in EKF frame at burnout, ~1g during coast) can latch
gyro_quiet / gps_stationary / accel_1g before apogee. The master vote is
gated on apogee_flag so this didn't (yet) cause a false LANDED, but the
latched sub-flags carried pre-apogee history into the post-apogee vote.

Fix: zero the sub-flag counters + flags on the apogee_flag rising edge so
post-apogee voting reflects only post-apogee evidence.

This test verifies the Python mirror in `sim_kinematic_checks.py`. The
firmware mirror in tinkerrocket-idf/components/TR_KinematicChecks is
covered by `tests_cpp/test_kinematic_checks.cpp::Landing_SubflagsResetOnApogeeRisingEdge`.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys

# Pull in sim_kinematic_checks from the Data_Analysis directory.
_SIM_DIR = Path(__file__).resolve().parent.parent.parent / "Data_Analysis"
sys.path.insert(0, str(_SIM_DIR))

from sim_kinematic_checks import KinematicChecks  # noqa: E402


def _drive(kc, now_ms, **kw):
    """Wrapper around kc.kinematic_checks() with sensible test defaults."""
    defaults = dict(
        pressure_altitude=30.0,
        acc_mag=9.80665,
        position=(0.0, 0.0, 30.0),
        velocity=(0.0, 0.0, 0.0),
        roll_rate=0.1,
        new_baro=True,
        gps_altitude=0.0,
        new_gps=False,
        pitch_rad=math.pi / 2,
        burnout_detected=False,
        baro_locked_out=False,
        now_ms=now_ms,
    )
    defaults.update(kw)
    kc.kinematic_checks(**defaults)


def test_sub_flag_counters_reset_on_apogee_rising_edge():
    kc = KinematicChecks()
    # Bypass launch detection (covered by other tests) and force
    # max_altitude so baro_stable's > 15 m gate is satisfied.
    kc.launch_flag = True
    kc.max_altitude = 50.0

    # Seed KF at 30 m so alt_est tracks above 15 m (needed for vel/baro
    # apogee detectors below).
    for i in range(80):
        _drive(kc, now_ms=i * 2)

    # 6 s of "quiet coast" — should latch gyro_quiet, accel_1g, baro_stable.
    for second in range(6):
        base = 200 + second * 1000
        for i in range(50):
            _drive(kc, now_ms=base + i * 2)

    # Pre-apogee: sub-flags latched, apogee_flag still False.
    assert kc.gyro_quiet_flag is True, "quiet inputs should latch gyro_quiet_flag pre-apogee"
    assert kc.accel_1g_flag is True,  "quiet inputs should latch accel_1g_flag pre-apogee"
    assert kc.baro_stable_flag is True, "quiet inputs should latch baro_stable_flag pre-apogee"
    assert kc.apogee_flag is False, "apogee_flag should still be False"

    # Counters at their cap (LANDING_SLOW_COUNT_MAX = 8 in the sim).
    assert kc.gyro_quiet_count_ >= 4
    assert kc.accel_1g_count_ >= 4
    assert kc.baro_stable_count_ >= 4

    # Drive apogee-triggering inputs to flip apogee_flag inside the
    # function (the rising edge our reset hooks on).
    #   vel_pass: position[2] > 15 and velocity[2] < 0
    #   baro_pass: alt_est > 15 and alt_est < max_altitude - 5 and d_alt < 20
    #   pitch_pass: pitch_rad < -0.087
    # APOGEE_COUNT_HI = 6 → need 6+ passing calls to latch sub-flags, then
    # voting fires on the next call.
    for i in range(20):
        alt = 30.0 - i * 0.5
        _drive(
            kc,
            now_ms=6500 + i * 2,
            pressure_altitude=alt,
            acc_mag=5.0,
            position=(0.0, 0.0, alt),
            velocity=(0.0, 0.0, -10.0),
            pitch_rad=-0.5,
            burnout_detected=True,
        )

    assert kc.apogee_flag is True, "apogee_flag should have transitioned"

    # The rising edge in this loop triggers the reset: counters back to 0,
    # flags back to False.
    assert kc.gyro_quiet_count_ == 0, f"gyro_quiet_count_ not reset (got {kc.gyro_quiet_count_})"
    assert kc.accel_1g_count_ == 0,   f"accel_1g_count_ not reset (got {kc.accel_1g_count_})"
    assert kc.gps_stationary_count_ == 0
    assert kc.baro_stable_count_ == 0
    assert kc.gyro_quiet_flag is False, "gyro_quiet_flag should reset on apogee rising edge"
    assert kc.accel_1g_flag is False
    assert kc.gps_stationary_flag is False
    assert kc.baro_stable_flag is False


def test_apogee_flag_already_set_does_not_reset_counters():
    """If apogee_flag is True at the start of the tick, no transition → no reset.

    Guards against accidentally resetting on every post-apogee tick.
    """
    kc = KinematicChecks()
    kc.launch_flag = True
    kc.max_altitude = 50.0
    kc.apogee_flag = True  # already set before this tick

    # Drive a single quiet tick; counters should tick normally (or not at
    # all if outside the 1 Hz window), but they MUST NOT be force-reset.
    kc.gyro_quiet_count_ = 5
    kc.gyro_quiet_flag = True
    kc.accel_1g_count_ = 5
    kc.accel_1g_flag = True

    _drive(kc, now_ms=10000)

    # Counters preserved (the 1-Hz block may or may not have ticked; the
    # important thing is that the reset block did NOT zero them).
    assert kc.gyro_quiet_count_ >= 4, (
        f"gyro_quiet_count_ should not be reset when apogee was already set "
        f"(got {kc.gyro_quiet_count_})"
    )
    assert kc.gyro_quiet_flag is True
