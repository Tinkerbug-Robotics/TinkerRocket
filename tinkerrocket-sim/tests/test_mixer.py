"""
Pybind11 ControlMixer tests.
Mirrors C++ test_control_mixer.cpp to ensure sim matches flight code.
"""
import pytest
import math

from tinkerrocket_sim._mixer import ControlMixer


DT = 0.002
MAX_FIN = 15.0


@pytest.fixture
def mixer():
    m = ControlMixer()
    m.configure(0.04, 0.001, 0.0003,   # pitch P/I/D
                0.04, 0.001, 0.0003,   # yaw P/I/D
                MAX_FIN, 95.0, 30.0)
    return m


def test_zero_command_zero_deflections(mixer):
    mixer.update(0, 0, 0, 0, 0, 0, 0, 95.0, 1.0, 1.0, DT)  # init
    mixer.update(0, 0, 0, 0, 0, 0, 0, 95.0, 1.0, 1.0, DT)
    d = mixer.get_fin_deflections()
    for v in d:
        assert abs(v) < 0.01


def test_pure_pitch_mixing(mixer):
    mixer.update(10.0, 0, 0, 0, 0, 0, 0, 95.0, 1.0, 1.0, DT)  # init
    mixer.update(10.0, 0, 0, 0, 0, 0, 0, 95.0, 1.0, 1.0, DT)
    d = mixer.get_fin_deflections()
    # PITCH_MIX = [+1, 0, -1, 0]
    assert abs(d[0]) > 0  # top
    assert abs(d[1]) < 0.01  # right (yaw only)
    assert abs(d[0] + d[2]) < 0.01  # top = -bottom
    assert abs(d[3]) < 0.01  # left (yaw only)


def test_pure_yaw_mixing(mixer):
    mixer.update(0, 10.0, 0, 0, 0, 0, 0, 95.0, 1.0, 1.0, DT)
    mixer.update(0, 10.0, 0, 0, 0, 0, 0, 95.0, 1.0, 1.0, DT)
    d = mixer.get_fin_deflections()
    # YAW_MIX = [0, +1, 0, -1]
    assert abs(d[0]) < 0.01
    assert abs(d[1]) > 0
    assert abs(d[2]) < 0.01
    assert abs(d[1] + d[3]) < 0.01


def test_pure_roll_all_same_sign(mixer):
    mixer.update(0, 0, 0, 0, 0, 0, 5.0, 95.0, 1.0, 1.0, DT)
    d = mixer.get_fin_deflections()
    for v in d:
        assert abs(v - 5.0) < 0.01


def test_fin_deflection_clamped(mixer):
    mixer.update(0, 0, 0, 0, 0, 0, 100.0, 95.0, 1.0, 1.0, DT)
    d = mixer.get_fin_deflections()
    for v in d:
        assert v <= MAX_FIN
        assert v >= -MAX_FIN


def test_gain_schedule_high_speed(mixer):
    mixer.enable_gain_schedule(95.0, 30.0)

    # Reference speed
    mixer.reset()
    mixer.update(10, 0, 0, 0, 0, 0, 0, 95.0, 1.0, 1.0, DT)
    mixer.update(10, 0, 0, 0, 0, 0, 0, 95.0, 1.0, 1.0, DT)
    d_ref = mixer.get_fin_deflections()

    # 2x reference speed
    mixer.reset()
    mixer.update(10, 0, 0, 0, 0, 0, 0, 190.0, 1.0, 1.0, DT)
    mixer.update(10, 0, 0, 0, 0, 0, 0, 190.0, 1.0, 1.0, DT)
    d_fast = mixer.get_fin_deflections()

    assert abs(d_fast[0]) < abs(d_ref[0])


def test_gain_schedule_low_speed(mixer):
    mixer.enable_gain_schedule(95.0, 30.0)

    mixer.reset()
    mixer.update(10, 0, 0, 0, 0, 0, 0, 95.0, 1.0, 1.0, DT)
    mixer.update(10, 0, 0, 0, 0, 0, 0, 95.0, 1.0, 1.0, DT)
    d_ref = mixer.get_fin_deflections()

    mixer.reset()
    mixer.update(10, 0, 0, 0, 0, 0, 0, 47.5, 1.0, 1.0, DT)
    mixer.update(10, 0, 0, 0, 0, 0, 0, 47.5, 1.0, 1.0, DT)
    d_slow = mixer.get_fin_deflections()

    assert abs(d_slow[0]) > abs(d_ref[0])


def test_reset_clears_all(mixer):
    for _ in range(100):
        mixer.update(10, 5, 0, 0, 0, 0, 3.0, 95.0, 1.0, 1.0, DT)
    mixer.reset()
    d = mixer.get_fin_deflections()
    for v in d:
        assert v == 0.0
    assert mixer.get_pitch_fin_cmd() == 0.0
    assert mixer.get_yaw_fin_cmd() == 0.0
