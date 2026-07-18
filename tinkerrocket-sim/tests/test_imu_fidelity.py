"""Unit tests for the IMU fidelity additions: full-scale clipping and
configurable mounting rotation (#170)."""
import numpy as np
import pytest

from tinkerrocket_sim.sensors.imu_model import IMUModel, _euler_zyx_to_matrix

# Stationary, level: body-to-ENU identity quaternion. A perfect IMU then reads
# +1 g on the up axis (acc_z ≈ +9.807 in the model's sign convention) and zero
# rate.
Q_LEVEL = np.array([1.0, 0.0, 0.0, 0.0])


def _clean_imu(**kw):
    return IMUModel(accel_noise_sigma=0.0, gyro_noise_sigma=0.0,
                    accel_bias_sigma=0.0, gyro_bias_sigma=0.0, seed=0, **kw)


def test_euler_identity_is_identity():
    assert np.allclose(_euler_zyx_to_matrix(0, 0, 0), np.eye(3))


def test_no_clip_by_default_passes_high_rate():
    imu = _clean_imu()
    omega = np.radians([6000.0, 0.0, 0.0])  # 6000 dps roll, above any range
    m = imu.measure(np.zeros(3), omega, Q_LEVEL, dt=1e-3)
    assert m['gyro_x'] == pytest.approx(6000.0, abs=1e-6)


def test_gyro_full_scale_clips():
    imu = _clean_imu(gyro_full_scale_dps=4000.0)
    omega = np.radians([6000.0, -6000.0, 100.0])
    m = imu.measure(np.zeros(3), omega, Q_LEVEL, dt=1e-3)
    assert m['gyro_x'] == pytest.approx(4000.0)    # clipped high
    assert m['gyro_y'] == pytest.approx(-4000.0)   # clipped low
    assert m['gyro_z'] == pytest.approx(100.0)     # within range, untouched


def test_accel_full_scale_clips():
    imu = _clean_imu(accel_full_scale_g=8.0)
    # 20 g of specific force along ENU-up -> body; should clip to 8 g = 78.456
    big = np.array([0.0, 0.0, 20.0 * 9.807])  # a - only; g added inside model
    m = imu.measure(big, np.zeros(3), Q_LEVEL, dt=1e-3)
    assert abs(m['acc_z']) <= 8.0 * 9.807 + 1e-6


def test_mounting_rotation_identity_matches_none():
    imu_a = _clean_imu()
    imu_b = _clean_imu(mounting_rotation_deg=(0.0, 0.0, 0.0))
    omega = np.radians([100.0, 50.0, -20.0])
    accel = np.array([1.0, 2.0, 3.0])
    ma = imu_a.measure(accel, omega, Q_LEVEL, dt=1e-3)
    mb = imu_b.measure(accel, omega, Q_LEVEL, dt=1e-3)
    for k in ('gyro_x', 'gyro_y', 'gyro_z', 'acc_x', 'acc_y', 'acc_z'):
        assert ma[k] == pytest.approx(mb[k], abs=1e-9)


def test_mounting_rotation_90deg_swaps_gyro_axes():
    # +90° yaw (about body z) maps body-x -> +y sensor axis under Rz.
    imu = _clean_imu(mounting_rotation_deg=(0.0, 0.0, 90.0))
    omega = np.radians([100.0, 0.0, 0.0])  # pure roll rate
    m = imu.measure(np.zeros(3), omega, Q_LEVEL, dt=1e-3)
    assert m['gyro_x'] == pytest.approx(0.0, abs=1e-6)
    assert m['gyro_y'] == pytest.approx(100.0, abs=1e-6)


def test_mounting_rotation_preserves_magnitude():
    imu = _clean_imu(mounting_rotation_deg=(15.0, -30.0, 45.0))
    omega = np.radians([100.0, 50.0, -20.0])
    m = imu.measure(np.zeros(3), omega, Q_LEVEL, dt=1e-3)
    mag_out = np.linalg.norm([m['gyro_x'], m['gyro_y'], m['gyro_z']])
    assert mag_out == pytest.approx(np.linalg.norm([100.0, 50.0, -20.0]), abs=1e-6)
