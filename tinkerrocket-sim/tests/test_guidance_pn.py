"""
Pybind11 GuidancePN tests.
Mirrors C++ test_guidance_pn.cpp to ensure sim matches flight code.
"""
import pytest
import math

from tinkerrocket_sim._guidance import GuidancePN


TARGET_ALT = 600.0
DT = 0.002


@pytest.fixture
def guid():
    g = GuidancePN()
    g.configure(3.0, 20.0, TARGET_ALT)
    return g


def test_directly_below_target_near_zero_lateral(guid):
    pos_enu = [0.0, 0.0, 300.0]
    vel_ned = [0.0, 0.0, -50.0]  # upward
    guid.update(pos_enu, vel_ned, DT)

    assert guid.is_active()
    assert abs(guid.get_accel_east_cmd()) < 0.5
    assert abs(guid.get_accel_north_cmd()) < 0.5
    assert abs(guid.get_lateral_offset()) < 0.1


def test_offset_from_pad_lateral_correction(guid):
    pos_enu = [50.0, 0.0, 300.0]
    vel_ned = [0.0, 0.0, -50.0]
    guid.update(pos_enu, vel_ned, DT)

    assert guid.is_active()
    assert guid.get_accel_east_cmd() < 0.0  # push back toward pad
    assert abs(guid.get_lateral_offset() - 50.0) < 0.1


def test_accel_clamping(guid):
    pos_enu = [200.0, 200.0, 300.0]
    vel_ned = [0.0, 0.0, -50.0]
    guid.update(pos_enu, vel_ned, DT)

    ae = guid.get_accel_east_cmd()
    an = guid.get_accel_north_cmd()
    au = guid.get_accel_up_cmd()
    a_mag = math.sqrt(ae**2 + an**2 + au**2)
    assert a_mag <= 20.0 + 0.01


def test_close_range_deactivates(guid):
    pos_enu = [0.0, 0.0, TARGET_ALT - 0.5]
    vel_ned = [0.0, 0.0, -1.0]
    guid.update(pos_enu, vel_ned, DT)

    assert not guid.is_active()
    assert guid.get_accel_east_cmd() == 0.0


def test_closing_velocity_approaching(guid):
    pos_enu = [0.0, 0.0, 300.0]
    vel_ned = [0.0, 0.0, -50.0]
    guid.update(pos_enu, vel_ned, DT)

    assert guid.get_closing_velocity() > 0.0
    assert not guid.is_cpa_reached()


def test_cpa_when_receding(guid):
    pos_enu = [0.0, 0.0, 300.0]
    vel_ned = [0.0, 0.0, 50.0]  # downward = away
    guid.update(pos_enu, vel_ned, DT)

    assert guid.get_closing_velocity() <= 0.0
    assert guid.is_cpa_reached()


def test_range_computed(guid):
    pos_enu = [30.0, 40.0, 100.0]
    vel_ned = [0.0, 0.0, -10.0]
    guid.update(pos_enu, vel_ned, DT)

    expected = math.sqrt(30**2 + 40**2 + 500**2)
    assert abs(guid.get_range() - expected) < 0.1


def test_reset(guid):
    guid.update([50.0, 50.0, 300.0], [0.0, 0.0, -50.0], DT)
    guid.reset()

    assert guid.get_accel_east_cmd() == 0.0
    assert guid.get_range() == 0.0
    assert not guid.is_active()
    assert not guid.is_cpa_reached()


def test_configure_updates_params(guid):
    guid.configure(5.0, 30.0, 800.0)
    pos_enu = [0.0, 0.0, 799.5]
    vel_ned = [0.0, 0.0, -1.0]
    guid.update(pos_enu, vel_ned, DT)
    assert not guid.is_active()  # range < 1m
