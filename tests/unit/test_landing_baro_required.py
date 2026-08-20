"""Regression tests for issue #824 (Python mirror).

With GPS stale the landing vote drops to 2-of-3, and the two remaining
detectors are both altitude-blind: gyro_quiet and accel_1g each pass under a
canopy at terminal velocity (an accelerometer reads 1 g in steady descent
exactly as it does sitting on the ground).  The pair could therefore latch
LANDED at altitude and cut the main deploy.

Fix, in two halves:
  * baro_stable is now mandatory in the vote — every latch must include the
    one detector that knows the rocket is near the ground; and
  * a new extended-quiescence detector provides a baro-independent path to
    LANDED so a dead or out-of-band barometer cannot strand the flight.

The firmware mirror is covered by the `Landing_NoGPS_*` / `Landing_Quiescent_*`
cases in tests_cpp/test_kinematic_checks.cpp.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys

_SIM_DIR = Path(__file__).resolve().parent.parent.parent / "Data_Analysis"
sys.path.insert(0, str(_SIM_DIR))

from sim_kinematic_checks import KinematicChecks  # noqa: E402


def _drive(kc, now_ms, **kw):
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


def _seed_flown(kc):
    kc.launch_flag = True
    kc.max_altitude = 500.0
    for i in range(80):
        _drive(kc, now_ms=i * 2)
    kc.apogee_flag = True


def _soak(kc, seconds, start_ms=1000, **kw):
    """Run `seconds` of 1 Hz slow-detector ticks with the given inputs."""
    for second in range(seconds):
        base = start_ms + second * 1000
        for i in range(50):
            _drive(kc, now_ms=base + i * 2, **kw)


def test_altitude_blind_pair_cannot_latch_landed_aloft():
    """The #824 failure: quiet + 1 g at 300 m, GPS stale, must not fire."""
    kc = KinematicChecks()
    _seed_flown(kc)

    # 60 s under canopy, descending 3 m/s from 300 m.
    for second in range(60):
        base = 1000 + second * 1000
        alt = 300.0 - 3.0 * second
        for i in range(50):
            _drive(kc, now_ms=base + i * 2, pressure_altitude=alt,
                   acc_mag=9.81, roll_rate=1.0,
                   position=(0.0, 0.0, alt), velocity=(0.0, 0.0, -3.0))

    assert kc.gyro_quiet_flag is True      # the altitude-blind pair passes...
    assert kc.accel_1g_flag is True
    assert kc.baro_stable_flag is False    # ...the altitude-aware one does not
    assert kc.alt_landed_flag is False, "altitude-blind evidence must not latch LANDED"


def test_quiescence_ends_flight_when_baro_cannot_satisfy_the_vote():
    """Landing 400 m off the pad reference: baro_stable can never latch."""
    kc = KinematicChecks()
    _seed_flown(kc)
    _soak(kc, 40, pressure_altitude=400.0, acc_mag=9.80665, roll_rate=0.2,
          position=(0.0, 0.0, 400.0))

    assert kc.baro_stable_flag is False
    assert kc.quiescent_flag is True
    assert kc.alt_landed_flag is True


def test_quiescence_does_not_fire_under_a_rolling_canopy():
    """40 dps is the low end of what the flight logs show under chute."""
    kc = KinematicChecks()
    _seed_flown(kc)
    for second in range(40):
        base = 1000 + second * 1000
        alt = 300.0 - 3.0 * second
        for i in range(50):
            _drive(kc, now_ms=base + i * 2, pressure_altitude=alt,
                   acc_mag=9.81, roll_rate=40.0,
                   position=(0.0, 0.0, alt), velocity=(0.0, 0.0, -3.0))

    assert kc.quiescent_flag is False
    assert kc.alt_landed_flag is False


def test_quiescence_does_not_fire_on_a_pendulum_swing():
    """The case the roll gate alone would miss: descent without roll."""
    kc = KinematicChecks()
    _seed_flown(kc)
    for second in range(40):
        base = 1000 + second * 1000
        alt = 300.0 - 3.0 * second
        acc = 11.77 if second % 2 == 0 else 7.85      # +/- 0.2 g
        for i in range(50):
            _drive(kc, now_ms=base + i * 2, pressure_altitude=alt,
                   acc_mag=acc, roll_rate=0.5,
                   position=(0.0, 0.0, alt), velocity=(0.0, 0.0, -3.0))

    assert kc.quiescent_flag is False
    assert kc.alt_landed_flag is False


def test_quiescence_not_armed_before_apogee():
    """A rocket on the pad is quiescent by definition; the counter must not
    carry off the pad and fire on the first post-apogee tick.

    max_altitude is earned by climbing rather than assigned, because
    baro_stable's own > 15 m gate is what keeps the *vote* off the pad — a
    forced max_altitude would let the vote fire here for unrelated reasons.
    """
    kc = KinematicChecks()
    kc.launch_flag = True

    # 60 s sitting on the pad: perfectly still, but never flown.
    _soak(kc, 60, start_ms=0, pressure_altitude=0.0, acc_mag=9.80665,
          roll_rate=0.0, position=(0.0, 0.0, 0.0))

    # Climb, so max_altitude is real.
    for i in range(80):
        _drive(kc, now_ms=61000 + i * 2, pressure_altitude=float(i),
               acc_mag=25.0, roll_rate=0.0,
               position=(0.0, 0.0, float(i)), velocity=(0.0, 0.0, 10.0))
    assert kc.max_altitude > 15.0

    kc.apogee_flag = True
    _drive(kc, now_ms=62000, pressure_altitude=300.0, acc_mag=9.80665,
           roll_rate=0.0, position=(0.0, 0.0, 300.0))
    assert kc.quiescent_flag is False
    assert kc.alt_landed_flag is False


def test_frozen_baro_cannot_supply_the_mandatory_voter():
    """baro_stable is mandatory because it is the only altitude-aware voter,
    so an unhealthy barometer must not be able to satisfy it.  A frozen sensor
    retains its last reading, making landing_alt_change exactly 0 — maximally
    "stable" — which would hand the vote its mandatory voter mid-descent."""
    kc = KinematicChecks()
    _seed_flown(kc)

    for second in range(20):
        base = 1000 + second * 1000
        for i in range(50):
            _drive(kc, now_ms=base + i * 2, pressure_altitude=30.0,
                   acc_mag=9.81, roll_rate=1.0, baro_healthy=False,
                   position=(0.0, 0.0, 30.0), velocity=(0.0, 0.0, -6.0))

    assert kc.gyro_quiet_flag is True
    assert kc.accel_1g_flag is True
    assert kc.baro_stable_flag is False, "an unhealthy baro must not satisfy baro_stable"
    assert kc.alt_landed_flag is False


def test_healthy_baro_still_latches_normally():
    """The baro_healthy gate must not break the ordinary landing."""
    kc = KinematicChecks()
    _seed_flown(kc)
    _soak(kc, 7, pressure_altitude=5.0, acc_mag=9.81, roll_rate=1.0,
          position=(0.0, 0.0, 5.0))

    assert kc.baro_stable_flag is True
    assert kc.alt_landed_flag is True


def test_dead_baro_quiescence_still_needs_a_quiet_imu():
    """With baro_healthy false, quiescence runs on the IMU alone.  Real canopy
    accel scatter misses the 0.05 g gate most ticks, and a leaky +1/-1 counter
    cannot climb on a sub-50% duty cycle."""
    kc = KinematicChecks()
    _seed_flown(kc)

    for second in range(60):
        base = 1000 + second * 1000
        alt = 300.0 - 3.0 * second
        acc = 9.80665 if second % 3 == 0 else 10.8
        for i in range(50):
            _drive(kc, now_ms=base + i * 2, pressure_altitude=alt,
                   acc_mag=acc, roll_rate=1.0, baro_healthy=False,
                   position=(0.0, 0.0, alt), velocity=(0.0, 0.0, -3.0))

    assert kc.quiescent_flag is False
    assert kc.alt_landed_flag is False
