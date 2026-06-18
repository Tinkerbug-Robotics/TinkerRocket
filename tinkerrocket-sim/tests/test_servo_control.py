"""SIL test: the REAL firmware roll controller (TR_ServoControl) on the host.

Exercises the actual flight binary's controlAngle cascade, V^2 gain schedule,
and KP_ANGLE rate cap — the behaviors PR #169 added and that the old Python
re-implementation could not represent. LEDC PWM is stubbed; we read the
controller's computed roll_cmd_deg (the fin command the sim turns into torque).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from tinkerrocket_sim._servo import ServoControl, set_mock_micros


def _step_angle(servo, t_us, **kw):
    # The controller's PID reads dt from micros(); advance the shim clock first.
    set_mock_micros(t_us)
    servo.control_angle(**kw)


def test_zero_error_holds_zero():
    s = ServoControl(kp=0.05, ki=0.0, kd=0.0, min_cmd=-20.0, max_cmd=20.0)
    for i in range(5):
        _step_angle(s, (i + 1) * 1000,
                    target_roll_deg=0.0, actual_roll_deg=0.0,
                    roll_rate_dps=0.0, velocity_ms=80.0,
                    kp_angle=4.0, rate_cap_dps=60.0)
    assert abs(s.roll_cmd_deg) < 1e-3


def test_rate_cap_limits_command():
    # Same large angle error into two controllers; the small rate cap must
    # produce a smaller-magnitude fin command than an effectively-infinite cap.
    common = dict(target_roll_deg=90.0, actual_roll_deg=0.0,
                  roll_rate_dps=0.0, velocity_ms=80.0, kp_angle=4.0)
    big = ServoControl(kp=0.05, ki=0.0, kd=0.0, min_cmd=-20.0, max_cmd=20.0)
    small = ServoControl(kp=0.05, ki=0.0, kd=0.0, min_cmd=-20.0, max_cmd=20.0)
    for i in range(4):
        t = (i + 1) * 1000
        _step_angle(big, t, rate_cap_dps=1000.0, **common)
        _step_angle(small, t, rate_cap_dps=5.0, **common)
    assert abs(small.roll_cmd_deg) < abs(big.roll_cmd_deg)
    # Both correct in the same direction.
    assert small.roll_cmd_deg <= 0.0 and big.roll_cmd_deg <= 0.0


def test_gain_schedule_scales_and_caps():
    # Below v_min the V^2 schedule saturates at GAIN_SCHEDULE_SCALE_CAP (3.0),
    # so the scheduled command is ~3x the unscheduled one for the same rate error.
    base_kp = 0.1
    sched = ServoControl(kp=base_kp, ki=0.0, kd=0.0, min_cmd=-50.0, max_cmd=50.0)
    sched.enable_gain_schedule(v_ref=95.0, v_min=30.0)
    plain = ServoControl(kp=base_kp, ki=0.0, kd=0.0, min_cmd=-50.0, max_cmd=50.0)
    for i in range(4):
        t = (i + 1) * 1000
        set_mock_micros(t)
        sched.control_with_gain_schedule(roll_rate=10.0, velocity_ms=10.0)
        set_mock_micros(t)
        plain.control(roll_rate=10.0)
    assert plain.roll_cmd_deg != 0.0
    ratio = sched.roll_cmd_deg / plain.roll_cmd_deg
    assert abs(ratio - 3.0) < 0.05
