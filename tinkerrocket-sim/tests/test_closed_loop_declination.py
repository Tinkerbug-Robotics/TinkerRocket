"""The closed-loop sim gives the flight filter the magnetic declination, as the
flight computer does after setQuaternion.

Without it the filter took the sim field's magnetic north (12.8 deg east of
true north) for true north: ~15.6 deg of heading error on the pad of every
closed-loop flight, which on a thrust 3 deg off vertical became ~1.1 m/s of
horizontal velocity error by burnout.
"""
import math

import numpy as np

from tinkerrocket_sim.estimation.tc_ekf import quat2dcm
from tinkerrocket_sim.simulation.closed_loop_sim import SimConfig, run_closed_loop
from tinkerrocket_sim.simulation.scenarios import build_rollypolly_iii


def _error_about_vertical_deg(q_true, q_est):
    """Rotation from the estimate to the truth about NED down, degrees."""
    E = quat2dcm(q_true).T @ quat2dcm(q_est) - np.eye(3)
    return math.degrees(0.5 * (E[1, 0] - E[0, 1]))


def test_the_flight_filter_holds_true_north_on_the_pad():
    cfg = SimConfig(pad_time=10.0, duration=0.2, physics_dt=1e-3, imu_rate=1000.0, log_interval=0.01,
                    launch_angle_deg=87.0, control_enabled=False, guidance_enabled=False, sensor_seed=42)
    df = run_closed_loop(build_rollypolly_iii(motor="G80T"), cfg).df
    row = df[df["time"] <= -0.01].iloc[-1]
    q_true = row[["true_q0_ned", "true_q1_ned", "true_q2_ned", "true_q3_ned"]].to_numpy(float)
    q_ekf = row[["ekf_q0", "ekf_q1", "ekf_q2", "ekf_q3"]].to_numpy(float)
    assert abs(_error_about_vertical_deg(q_true, q_ekf)) < 1.5
