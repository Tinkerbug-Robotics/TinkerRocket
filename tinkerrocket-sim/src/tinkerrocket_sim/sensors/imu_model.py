"""IMU sensor model (accelerometer + gyroscope).

Models the ISM6HG256 (formerly ICM45686) IMU with:
- True specific force (accel - gravity in body frame)
- Additive Gaussian noise
- Slowly varying Markov bias
- Output at configurable rate (default 1200 Hz)

Body frame: FRD (Forward-Right-Down), matching the flight computer convention.
"""
import numpy as np

from ..physics.sixdof import quat_to_dcm


class IMUModel:
    """Simulated IMU producing accelerometer and gyroscope readings."""

    def __init__(self,
                 accel_noise_sigma: float = 0.05,    # m/s^2
                 gyro_noise_sigma: float = 0.00175,   # rad/s (~0.1 deg/s)
                 accel_bias_sigma: float = 0.01,       # m/s^2
                 gyro_bias_sigma: float = 0.00025,     # rad/s
                 accel_bias_tau: float = 100.0,        # s
                 gyro_bias_tau: float = 50.0,          # s
                 rate_hz: float = 1200.0):
        self.accel_noise_sigma = accel_noise_sigma
        self.gyro_noise_sigma = gyro_noise_sigma
        self.accel_bias_sigma = accel_bias_sigma
        self.gyro_bias_sigma = gyro_bias_sigma
        self.accel_bias_tau = accel_bias_tau
        self.gyro_bias_tau = gyro_bias_tau
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz

        # Markov bias state
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

        self._rng = np.random.default_rng()

    def measure(self, true_accel_enu: np.ndarray,
                true_omega_body: np.ndarray,
                quaternion: np.ndarray,
                dt: float = None) -> dict:
        """Generate IMU measurement from true state.

        Args:
            true_accel_enu: True acceleration in ENU frame (m/s^2), NOT including gravity.
            true_omega_body: True angular velocity in body frame (rad/s).
            quaternion: Body-to-ENU quaternion [q0, q1, q2, q3].
            dt: Timestep for bias propagation (defaults to 1/rate_hz).

        Returns:
            Dict with acc_x/y/z (m/s^2) and gyro_x/y/z (deg/s), time_us.
        """
        if dt is None:
            dt = self.dt

        # Rotation matrix: ENU-to-body
        R_b2e = quat_to_dcm(quaternion)
        R_e2b = R_b2e.T

        # Specific force in body frame = R_e2b @ (accel + g_enu)
        # IMU measures specific force: accel_body = R_e2b @ (a_enu - g_enu)
        # where g_enu = [0, 0, -9.807] (gravity pulls down, negative z in ENU)
        # Specific force = a_inertial - g = a_enu + [0, 0, 9.807]
        g_enu = np.array([0.0, 0.0, -9.807])
        specific_force_enu = true_accel_enu - g_enu  # a - g
        specific_force_body = R_e2b @ specific_force_enu

        # Propagate Markov biases
        self._propagate_bias(dt)

        # Add noise and bias
        accel_meas = specific_force_body + self.accel_bias + \
                     self._rng.normal(0, self.accel_noise_sigma, 3)
        gyro_meas = true_omega_body + self.gyro_bias + \
                    self._rng.normal(0, self.gyro_noise_sigma, 3)

        return {
            'acc_x': accel_meas[0],
            'acc_y': accel_meas[1],
            'acc_z': accel_meas[2],
            'gyro_x': np.degrees(gyro_meas[0]),  # deg/s (matches sensor output)
            'gyro_y': np.degrees(gyro_meas[1]),
            'gyro_z': np.degrees(gyro_meas[2]),
        }

    def _propagate_bias(self, dt: float):
        """First-order Gauss-Markov bias propagation."""
        # accel bias
        alpha_a = dt / self.accel_bias_tau
        noise_a = np.sqrt(2 * self.accel_bias_sigma**2 * dt / self.accel_bias_tau)
        self.accel_bias = (1 - alpha_a) * self.accel_bias + \
                          self._rng.normal(0, noise_a, 3)

        # gyro bias
        alpha_w = dt / self.gyro_bias_tau
        noise_w = np.sqrt(2 * self.gyro_bias_sigma**2 * dt / self.gyro_bias_tau)
        self.gyro_bias = (1 - alpha_w) * self.gyro_bias + \
                         self._rng.normal(0, noise_w, 3)

    def reset(self):
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
