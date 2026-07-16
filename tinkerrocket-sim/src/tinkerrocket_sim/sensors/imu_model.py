"""IMU sensor model (accelerometer + gyroscope).

Models the ISM6HG256 (formerly ICM45686) IMU with:
- True specific force (accel - gravity in body frame)
- Additive Gaussian noise
- Slowly varying Markov bias
- Full-scale clipping (gyro/accel saturate at the configured range)
- Configurable mounting rotation (IMU physically rotated w.r.t. the airframe)
- Output at configurable rate (default 1200 Hz)

The IMU is polled once per output sample in the sim loop, so the discrete
sampling at `rate_hz` *is* the sensor's zero-order sample-and-hold — there is
no sub-sampling to latch as there is for the slower magnetometer.

Body frame: FRD (Forward-Right-Down), matching the flight computer convention.
"""
import numpy as np

from ..physics.sixdof import quat_to_dcm


def _euler_zyx_to_matrix(roll_deg, pitch_deg, yaw_deg):
    """Rotation matrix from ZYX (yaw-pitch-roll) Euler angles in degrees."""
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


class IMUModel:
    """Simulated IMU producing accelerometer and gyroscope readings."""

    def __init__(self,
                 accel_noise_sigma: float = 0.05,    # m/s^2
                 gyro_noise_sigma: float = 0.00175,   # rad/s (~0.1 deg/s)
                 accel_bias_sigma: float = 0.01,       # m/s^2
                 gyro_bias_sigma: float = 0.00025,     # rad/s
                 accel_bias_tau: float = 100.0,        # s
                 gyro_bias_tau: float = 50.0,          # s
                 rate_hz: float = 1200.0,
                 gyro_full_scale_dps: float = None,    # gyro range (deg/s); None = no clip
                 accel_full_scale_g: float = None,     # accel range (g); None = no clip
                 mounting_rotation_deg=None,           # (roll,pitch,yaw) IMU vs airframe
                 seed: int = None):
        self.accel_noise_sigma = accel_noise_sigma
        self.gyro_noise_sigma = gyro_noise_sigma
        self.accel_bias_sigma = accel_bias_sigma
        self.gyro_bias_sigma = gyro_bias_sigma
        self.accel_bias_tau = accel_bias_tau
        self.gyro_bias_tau = gyro_bias_tau
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz

        # Full-scale saturation (a real gyro clips hard at its range — the
        # 2026-05-17 flight ran the gyro at ±4000 dps and saw clipping).
        self.gyro_full_scale_dps = gyro_full_scale_dps
        self.accel_full_scale_g = accel_full_scale_g

        # Mounting rotation: the IMU sees body-frame vectors rotated by R_mount.
        # Default identity = perfectly aligned to the airframe.  A non-identity
        # rotation injects a fixed misalignment the firmware constants do not
        # undo, so it degrades the EKF — useful for testing mounting-constant
        # sensitivity, analogous to inject_heading_bias_deg.
        if mounting_rotation_deg is None:
            self.R_mount = None
        else:
            self.R_mount = _euler_zyx_to_matrix(*mounting_rotation_deg)

        # Markov bias state
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

        self._rng = np.random.default_rng(seed)

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

        # Mounting rotation: the sensor axes are rotated relative to the
        # airframe body frame (applied to the true vectors before noise/bias).
        omega_body = np.asarray(true_omega_body, dtype=float)
        if self.R_mount is not None:
            specific_force_body = self.R_mount @ specific_force_body
            omega_body = self.R_mount @ omega_body

        # Propagate Markov biases
        self._propagate_bias(dt)

        # Add noise and bias
        accel_meas = specific_force_body + self.accel_bias + \
                     self._rng.normal(0, self.accel_noise_sigma, 3)
        gyro_meas = omega_body + self.gyro_bias + \
                    self._rng.normal(0, self.gyro_noise_sigma, 3)

        # Full-scale clipping (hard saturation at the sensor range).
        if self.accel_full_scale_g is not None:
            fs = self.accel_full_scale_g * 9.807
            accel_meas = np.clip(accel_meas, -fs, fs)
        gyro_deg = np.degrees(gyro_meas)
        if self.gyro_full_scale_dps is not None:
            gyro_deg = np.clip(gyro_deg, -self.gyro_full_scale_dps,
                               self.gyro_full_scale_dps)

        return {
            'acc_x': accel_meas[0],
            'acc_y': accel_meas[1],
            'acc_z': accel_meas[2],
            'gyro_x': gyro_deg[0],  # deg/s (matches sensor output)
            'gyro_y': gyro_deg[1],
            'gyro_z': gyro_deg[2],
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
