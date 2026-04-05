"""
Pybind11 PID controller tests.

Mirrors the C++ test_pid.cpp test suite to ensure the simulation PID
produces identical results to what the flight firmware uses.

IMPORTANT: The sim PID uses derivative-on-ERROR (line 35 of pid_controller.h):
    D = Kd * (error - last_error) / dt
while the flight PID uses derivative-on-MEASUREMENT (TR_PID.cpp line 74):
    D = -Kd * (actual - last_measurement) / dt

This means the D-term will differ when setpoint changes. Both behaviors
are correct for their context (sim uses D-on-error for simplicity;
flight uses D-on-measurement to avoid setpoint kick).
"""
import pytest
import math

from tinkerrocket_sim._pid import PIDController


DT = 0.01  # 100 Hz


def test_first_call_returns_zero():
    pid = PIDController(1.0, 0.5, 0.1, -10.0, 10.0)
    out = pid.compute(10.0, 0.0, DT)
    assert out == 0.0


def test_zero_error_zero_output():
    pid = PIDController(1.0, 0.5, 0.1, -10.0, 10.0)
    pid.compute(5.0, 5.0, DT)  # init
    out = pid.compute(5.0, 5.0, DT)
    assert abs(out) < 1e-5


def test_proportional_only():
    pid = PIDController(2.0, 0.0, 0.0, -10.0, 10.0)
    pid.compute(0.0, 0.0, DT)  # init
    out = pid.compute(3.0, 0.0, DT)
    # P = 2.0 * 3.0 = 6.0
    # D = Kd * (error - last_error) / dt = 0 (Kd=0)
    assert abs(out - 6.0) < 0.01


def test_integral_accumulation():
    pid = PIDController(0.0, 1.0, 0.0, -10.0, 10.0)
    pid.compute(0.0, 0.0, DT)  # init
    out = 0.0
    for _ in range(10):
        out = pid.compute(5.0, 0.0, DT)
    # cumulative_error = 5.0 * 0.01 * 10 = 0.5
    # I = 1.0 * 0.5 = 0.5
    assert abs(out - 0.5) < 0.05


def test_integral_antiwindup():
    pid = PIDController(0.0, 100.0, 0.0, -10.0, 10.0)
    pid.compute(0.0, 0.0, DT)
    for _ in range(1000):
        out = pid.compute(100.0, 0.0, DT)
    assert out <= 10.0
    assert out >= -10.0


def test_output_clamping():
    pid = PIDController(100.0, 0.0, 0.0, -5.0, 5.0)
    pid.compute(0.0, 0.0, DT)
    out = pid.compute(100.0, 0.0, DT)
    assert abs(out - 5.0) < 0.01

    out = pid.compute(-100.0, 0.0, DT)
    assert abs(out - (-5.0)) < 0.01


def test_reset_clears_state():
    pid = PIDController(1.0, 0.5, 0.1, -10.0, 10.0)
    pid.compute(10.0, 0.0, DT)
    pid.compute(10.0, 0.0, DT)
    pid.reset()
    out = pid.compute(10.0, 0.0, DT)
    assert out == 0.0


def test_negative_dt_returns_zero():
    pid = PIDController(1.0, 0.5, 0.1, -10.0, 10.0)
    pid.compute(10.0, 0.0, DT)  # init
    pid.compute(10.0, 0.0, DT)  # normal
    out = pid.compute(10.0, 0.0, -0.01)
    assert out == 0.0


def test_gain_setters():
    pid = PIDController(1.0, 0.0, 0.0, -10.0, 10.0)
    pid.compute(0.0, 0.0, DT)
    pid.compute(5.0, 0.0, DT)
    pid.set_kp(2.0)
    assert pid.kp == 2.0
    out = pid.compute(5.0, 0.0, DT)
    assert out == 10.0  # clamped at max


def test_derivative_on_error():
    """Sim PID uses D-on-error, NOT D-on-measurement like flight code.
    A step change in setpoint WILL produce a D-kick in the sim."""
    pid = PIDController(0.0, 0.0, 1.0, -1000.0, 1000.0)
    pid.compute(0.0, 0.0, DT)  # init

    # Step setpoint from 0 to 100, measurement stays at 0
    out = pid.compute(100.0, 0.0, DT)
    # D = Kd * (error - last_error) / dt = 1.0 * (100 - 0) / 0.01 = 10000
    # This IS the D-kick that flight code avoids via D-on-measurement
    assert abs(out - 1000.0) < 1.0  # clamped


def test_properties_readable():
    pid = PIDController(1.0, 2.0, 3.0, -5.0, 5.0)
    assert pid.kp == 1.0
    assert pid.ki == 2.0
    assert pid.kd == 3.0
    assert pid.min_cmd == -5.0
    assert pid.max_cmd == 5.0
    assert pid.cumulative_error == 0.0
    assert pid.last_error == 0.0
