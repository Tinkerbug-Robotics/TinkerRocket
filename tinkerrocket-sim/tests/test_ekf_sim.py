"""
EKF regression tests via pybind11 bindings.

Tests the same C++ TR_GpsInsEKF code used in flight, but exercised
through the Python simulation path. Validates convergence, stability,
and numerical health under simulated sensor conditions.
"""
import pytest
import math
import numpy as np

from tinkerrocket_sim._ekf import (
    GpsInsEKF, IMUData, GNSSData, GNSSDataLLA, MagData, BaroData
)


# Approximate launch site (LA area)
LAT_DEG = 33.7
LON_DEG = -118.4
ALT_M = 100.0
LAT_RAD = math.radians(LAT_DEG)
LON_RAD = math.radians(LON_DEG)


def make_stationary_imu(time_us: int) -> IMUData:
    imu = IMUData()
    imu.time_us = time_us
    imu.acc_x = 0.0
    imu.acc_y = 0.0
    imu.acc_z = 9.807  # gravity in FRD (nose up, Z=down)
    imu.gyro_x = 0.0
    imu.gyro_y = 0.0
    imu.gyro_z = 0.0
    return imu


def make_stationary_gnss_lla(time_us: int) -> GNSSDataLLA:
    gnss = GNSSDataLLA()
    gnss.time_us = time_us
    gnss.lat_rad = LAT_RAD
    gnss.lon_rad = LON_RAD
    gnss.alt_m = ALT_M
    gnss.vel_n_mps = 0.0
    gnss.vel_e_mps = 0.0
    gnss.vel_d_mps = 0.0
    return gnss


def make_stationary_mag(time_us: int) -> MagData:
    mag = MagData()
    mag.time_us = time_us
    mag.mag_x = 22.0
    mag.mag_y = 0.0
    mag.mag_z = 42.0
    return mag


def make_baro(time_us: int, alt: float = ALT_M) -> BaroData:
    baro = BaroData()
    baro.time_us = time_us
    baro.altitude_m = alt
    return baro


class TestEKFStationary:
    """Tests with stationary (on-pad) sensor data."""

    def setup_method(self):
        self.ekf = GpsInsEKF()
        self.ekf.init_lla(
            make_stationary_imu(0),
            make_stationary_gnss_lla(0),
            make_stationary_mag(0),
        )

    def test_init_sets_position(self):
        pos = self.ekf.get_position()
        assert abs(pos[0] - LAT_RAD) < 1e-4
        assert abs(pos[1] - LON_RAD) < 1e-4
        assert abs(pos[2] - ALT_M) < 1.0

    def test_quaternion_norm_preserved(self):
        t = 0
        for _ in range(1000):
            t += 2000
            self.ekf.update_lla(True,
                                make_stationary_imu(t),
                                make_stationary_gnss_lla(t),
                                make_stationary_mag(t))
        q = self.ekf.get_quaternion()
        norm = math.sqrt(sum(x * x for x in q))
        assert abs(norm - 1.0) < 0.01

    def test_stationary_convergence(self):
        """Position covariance should decrease over 10 seconds."""
        cov_initial = self.ekf.get_cov_pos()

        t = 0
        for i in range(5000):  # 10s at 500 Hz
            t += 2000
            self.ekf.update_lla(True,
                                make_stationary_imu(t),
                                make_stationary_gnss_lla(t),
                                make_stationary_mag(t))
            if i % 4 == 0:  # baro at ~125 Hz
                self.ekf.baro_meas_update(make_baro(t))
        cov_final = self.ekf.get_cov_pos()
        for i in range(3):
            assert cov_final[i] < cov_initial[i]

    def test_velocity_accuracy(self):
        """Stationary velocity should converge to near-zero."""
        t = 0
        for i in range(15000):  # 30s at 500 Hz
            t += 2000
            self.ekf.update_lla(True,
                                make_stationary_imu(t),
                                make_stationary_gnss_lla(t),
                                make_stationary_mag(t))
            if i % 4 == 0:
                self.ekf.baro_meas_update(make_baro(t))
        vel = self.ekf.get_velocity()
        for v in vel:
            assert abs(v) < 0.5  # < 0.5 m/s

    def test_baro_reduces_alt_covariance(self):
        # EKF is 15-state (baro offset dropped); baro updates now write
        # directly into the altitude (down) component of position.
        t = 0
        for _ in range(100):
            t += 2000
            self.ekf.update_lla(True,
                                make_stationary_imu(t),
                                make_stationary_gnss_lla(t),
                                make_stationary_mag(t))
        cov_before = self.ekf.get_cov_pos()[2]  # altitude covariance
        self.ekf.baro_meas_update(make_baro(t, ALT_M))
        cov_after = self.ekf.get_cov_pos()[2]
        assert cov_after <= cov_before

    def test_no_nan_after_60s(self):
        """60 seconds of continuous updates should produce no NaN."""
        t = 0
        for i in range(30000):
            t += 2000
            self.ekf.update_lla(True,
                                make_stationary_imu(t),
                                make_stationary_gnss_lla(t),
                                make_stationary_mag(t))
            if i % 10 == 0:
                self.ekf.baro_meas_update(make_baro(t))

        for name, getter in [
            ("vel", self.ekf.get_velocity),
            ("pos", self.ekf.get_position),
            ("orient", self.ekf.get_orientation),
            ("quat", self.ekf.get_quaternion),
            ("accel_bias", self.ekf.get_accel_bias),
            ("gyro_bias", self.ekf.get_rot_rate_bias),
        ]:
            vals = getter()
            for i, v in enumerate(vals):
                assert math.isfinite(v), f"{name}[{i}] is not finite: {v}"

    def test_covariance_positive_definite(self):
        """All covariance diagonals should remain positive."""
        t = 0
        for _ in range(5000):
            t += 2000
            self.ekf.update_lla(True,
                                make_stationary_imu(t),
                                make_stationary_gnss_lla(t),
                                make_stationary_mag(t))
        for name, getter in [
            ("pos", self.ekf.get_cov_pos),
            ("vel", self.ekf.get_cov_vel),
            ("orient", self.ekf.get_cov_orient),
        ]:
            vals = getter()
            for i, v in enumerate(vals):
                assert v > 0, f"cov_{name}[{i}] = {v} is not positive"


class TestEKFECEFPath:
    """Test the ECEF init/update path (used by simulation)."""

    def test_ecef_init_and_update(self):
        ekf = GpsInsEKF()

        # Convert LLA to ECEF
        cos_lat = math.cos(LAT_RAD)
        sin_lat = math.sin(LAT_RAD)
        cos_lon = math.cos(LON_RAD)
        sin_lon = math.sin(LON_RAD)
        a = 6378137.0
        e2 = 6.69437999014e-3
        N = a / math.sqrt(1.0 - e2 * sin_lat ** 2)

        gnss = GNSSData()
        gnss.time_us = 0
        gnss.ecef_x = (N + ALT_M) * cos_lat * cos_lon
        gnss.ecef_y = (N + ALT_M) * cos_lat * sin_lon
        gnss.ecef_z = (N * (1 - e2) + ALT_M) * sin_lat
        gnss.ecef_vx = 0.0
        gnss.ecef_vy = 0.0
        gnss.ecef_vz = 0.0

        ekf.init(make_stationary_imu(0), gnss, make_stationary_mag(0))

        # Run a few updates
        t = 0
        for _ in range(100):
            t += 2000
            gnss.time_us = t
            ekf.update(True, make_stationary_imu(t), gnss, make_stationary_mag(t))

        vel = ekf.get_velocity()
        for v in vel:
            assert math.isfinite(v)


class TestEKFStateInjection:
    """Test state injection methods."""

    def setup_method(self):
        self.ekf = GpsInsEKF()
        self.ekf.init_lla(
            make_stationary_imu(0),
            make_stationary_gnss_lla(0),
            make_stationary_mag(0),
        )

    def test_set_quaternion(self):
        self.ekf.set_quaternion(1.0, 0.0, 0.0, 0.0)
        # Run one update
        self.ekf.update_lla(True,
                            make_stationary_imu(2000),
                            make_stationary_gnss_lla(2000),
                            make_stationary_mag(2000))
        orient = self.ekf.get_orientation()
        # Roll and pitch should be near zero with identity quat
        assert abs(orient[0]) < 0.2
        assert abs(orient[1]) < 0.2

    def test_set_position(self):
        new_lat = math.radians(34.0)
        new_lon = math.radians(-117.0)
        self.ekf.set_position(new_lat, new_lon, 500.0)
        pos = self.ekf.get_position()
        assert abs(pos[0] - new_lat) < 1e-6
        assert abs(pos[1] - new_lon) < 1e-6
        assert abs(pos[2] - 500.0) < 1e-6

    def test_set_velocity(self):
        self.ekf.set_velocity(10.0, -5.0, 2.0)
        vel = self.ekf.get_velocity()
        assert abs(vel[0] - 10.0) < 1e-4
        assert abs(vel[1] - (-5.0)) < 1e-4
        assert abs(vel[2] - 2.0) < 1e-4

    def test_gps_noise_scale(self):
        self.ekf.set_gps_noise_scale(5.0)
        assert self.ekf.get_gps_noise_scale() == 5.0
