"""Magnetometer sensor model.

Models the MMC5983MA magnetometer with:
- Earth's magnetic field rotated to body frame
- Additive Gaussian noise
"""
import numpy as np
from ..physics.sixdof import quat_to_dcm


class MagModel:
    """Simulated magnetometer."""

    def __init__(self,
                 noise_sigma: float = 0.5,  # uT
                 rate_hz: float = 1000.0,
                 field_ned: np.ndarray = None):
        """
        Args:
            noise_sigma: Measurement noise std dev in uT.
            rate_hz: Output rate in Hz.
            field_ned: Earth magnetic field in NED frame (uT).
                       Default is approximate for mid-latitudes US.
        """
        self.noise_sigma = noise_sigma
        self.rate_hz = rate_hz
        if field_ned is None:
            # Approximate field for ~38N latitude (NED, uT)
            self.field_ned = np.array([22.0, 5.0, 42.0])
        else:
            self.field_ned = np.array(field_ned)
        self._rng = np.random.default_rng()

    def measure(self, quaternion: np.ndarray) -> dict:
        """Generate magnetometer measurement from true orientation.

        Args:
            quaternion: Body-to-ENU quaternion [q0, q1, q2, q3].

        Returns:
            Dict with mag_x/y/z in uT (body frame).
        """
        R_b2e = quat_to_dcm(quaternion)
        R_e2b = R_b2e.T

        # Convert NED field to ENU: [E, N, U] = [field_ned[1], field_ned[0], -field_ned[2]]
        field_enu = np.array([self.field_ned[1], self.field_ned[0], -self.field_ned[2]])

        # Rotate to body frame
        field_body = R_e2b @ field_enu

        # Add noise
        field_body += self._rng.normal(0, self.noise_sigma, 3)

        return {
            'mag_x': field_body[0],
            'mag_y': field_body[1],
            'mag_z': field_body[2],
        }
