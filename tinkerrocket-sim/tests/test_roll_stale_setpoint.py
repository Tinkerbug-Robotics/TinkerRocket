"""SIL regression for #372: controlAngle() must not leave a stale rate setpoint.

Post-v4 roll profiles are ANGLE-mode for the whole profile (incl. hold-last-
angle), so the ANGLE -> rate-null transition happens exactly on the #265
EKF-health gate: when the EKF goes unhealthy, `angle_mode_active` drops and the
firmware falls from controlAngle() to the bare rate-null path
(controlWithGainSchedule / control) — main.cpp:5816-5857. Pre-#372, controlAngle
persisted its outer-loop rate command into pid_setpoint, so that rate-null path
HELD the residual rate (up to ±rate_cap) instead of nulling — corrupting the
safety fallback itself.

These tests drive the *real firmware controller* (the SIL _servo binding, the
exact object closed_loop_sim.py flies) through that sequence, mirroring the
firmware call convention: advance the servo's mock clock each tick, pass the
rate arg negated (-roll_rate_dps), seed the persistent setpoint ONCE at init
(as the hardened sim now does, matching the firmware boot / SERVO_CTRL_ENABLE
path). On pre-#372 firmware the null command sticks at ≈ -rate_cap; with the
fix it nulls to the configured setpoint.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from tinkerrocket_sim._servo import ServoControl, set_mock_micros

# Mirror the sim's firmware-controller wiring.
IMU_DT = 1.0 / 1200.0            # closed_loop_sim imu_rate
KP, KI, KD = 1.0, 0.0, 0.0      # P-only inner loop: fin cmd == rate error
MIN_CMD, MAX_CMD = -60.0, 60.0
GAIN_V_REF, GAIN_V_MIN = 50.0, 25.0
KP_ANGLE = 4.0
RATE_CAP_DPS = 60.0
SPEED_MS = 40.0                 # < V_ref, so the V^2 schedule is active (>1x)


class _Servo:
    """A firmware-controller instance clocked exactly like closed_loop_sim."""

    def __init__(self, setpoint_dps=0.0, gain_schedule=True):
        self.s = ServoControl(kp=KP, ki=KI, kd=KD,
                              min_cmd=MIN_CMD, max_cmd=MAX_CMD)
        if gain_schedule:
            self.s.enable_gain_schedule(GAIN_V_REF, GAIN_V_MIN)
        # One-time setpoint seed — mirrors firmware boot (main.cpp:2768) and the
        # hardened sim init. The rate-null path relies on this persistent value.
        self.s.set_setpoint(setpoint_dps)
        self._t_us = 0
        self._tick()  # TR_PID returns 0 on its first call; burn the dt bootstrap
        self.s.control(0.0)

    def _tick(self):
        self._t_us += int(round(IMU_DT * 1e6))
        set_mock_micros(self._t_us)

    def angle_tick(self, target_deg, actual_deg, roll_rate_dps):
        self._tick()
        self.s.control_angle(target_deg, actual_deg, -roll_rate_dps,
                             SPEED_MS, KP_ANGLE, RATE_CAP_DPS)
        return self.s.roll_cmd_deg

    def gain_sched_null_tick(self, roll_rate_dps):
        # Firmware ROLL_SEG_NULL_RATE / EKF-healthy fallback (main.cpp:5849).
        self._tick()
        self.s.control_with_gain_schedule(-roll_rate_dps, SPEED_MS)
        return self.s.roll_cmd_deg

    def gyro_null_tick(self, roll_rate_dps):
        # Firmware pure-gyro fallback when the gain schedule is off / EKF
        # unhealthy (main.cpp:5856).
        self._tick()
        self.s.control(-roll_rate_dps)
        return self.s.roll_cmd_deg


def _drive_unconverged_angle_segment(servo):
    """An ANGLE segment that ends unconverged: a 180° error the vehicle can't
    close, so every tick's outer loop saturates the rate command at ±rate_cap.
    Returns the last angle-tick fin command (should be the saturated cmd)."""
    cmd = 0.0
    for _ in range(20):
        cmd = servo.angle_tick(target_deg=180.0, actual_deg=0.0,
                               roll_rate_dps=0.0)
    return cmd


def test_ekf_health_drop_after_angle_nulls_cleanly():
    # Angle segment saturates the outer loop at the +rate_cap; the inner-loop
    # setpoint for that tick is -rate_cap.
    servo = _Servo(setpoint_dps=0.0, gain_schedule=True)
    ang_cmd = _drive_unconverged_angle_segment(servo)
    assert abs(ang_cmd) > 30.0, f"angle segment should saturate, got {ang_cmd}"

    # #265 EKF-health drop: fall to the rate-null path with the measured rate
    # already at the setpoint (0). A correct null commands ~0; the pre-#372 bug
    # would hold ≈ -rate_cap here.
    null_cmd = servo.gain_sched_null_tick(roll_rate_dps=0.0)
    assert abs(null_cmd) < 1e-3, (
        f"rate-null after ANGLE held a residual command ({null_cmd:.3f} deg) — "
        f"stale pid_setpoint (#372)")


def test_pure_gyro_fallback_after_angle_nulls_cleanly():
    # Same transition but into the pure-gyro fallback (gain schedule off) — the
    # deepest safety path (EKF unhealthy AND schedule off).
    servo = _Servo(setpoint_dps=0.0, gain_schedule=False)
    _drive_unconverged_angle_segment(servo)
    null_cmd = servo.gyro_null_tick(roll_rate_dps=0.0)
    assert abs(null_cmd) < 1e-3, (
        f"pure-gyro rate-null after ANGLE held a residual command "
        f"({null_cmd:.3f} deg) — stale pid_setpoint (#372)")


def test_configured_setpoint_survives_angle_segment():
    # A non-zero constant-rate setpoint must still govern the rate-null path
    # after an ANGLE segment — the angle tick must neither clobber it (bug) nor
    # zero it. P-only: fin cmd == setpoint - measured.
    servo = _Servo(setpoint_dps=5.0, gain_schedule=False)
    _drive_unconverged_angle_segment(servo)
    null_cmd = servo.gyro_null_tick(roll_rate_dps=0.0)
    assert abs(null_cmd - 5.0) < 1e-3, (
        f"configured setpoint not honored after ANGLE segment: {null_cmd:.3f}")


def test_angle_segment_still_tracks():
    # Guard the fix didn't perturb angle-mode behavior: a converging angle tick
    # commands the proportional rate. target-actual = 10°, kp_angle=4 -> 40 dps
    # cap-clear; inner P-only, measured 0 -> fin cmd -40.
    servo = _Servo(setpoint_dps=0.0, gain_schedule=False)
    servo._tick()
    cmd = servo.s.roll_cmd_deg  # unused; just keep clock aligned
    cmd = servo.angle_tick(target_deg=10.0, actual_deg=0.0, roll_rate_dps=0.0)
    assert abs(cmd - (-40.0)) < 1e-4, f"angle tracking changed: {cmd}"
