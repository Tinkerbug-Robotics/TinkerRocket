"""Barometric pressure sensor model.

Models the BMP585 barometer with:
- ISA pressure from altitude
- Additive Gaussian noise
- Temperature output
- Mach-dependent static port position error
"""
import numpy as np
from ..physics import atmosphere as atm


class BaroModel:
    """Simulated barometric pressure sensor."""

    def __init__(self,
                 pressure_noise_sigma: float = 1.0,    # Pa
                 temp_noise_sigma: float = 0.5,         # K
                 rate_hz: float = 500.0,
                 ground_pressure_pa: float = 101325.0):
        self.pressure_noise_sigma = pressure_noise_sigma
        self.temp_noise_sigma = temp_noise_sigma
        self.rate_hz = rate_hz
        self.ground_pressure_pa = ground_pressure_pa
        self._rng = np.random.default_rng()

    def measure(self, altitude_m: float, mach: float = 0.0) -> dict:
        """Generate barometric measurement from true altitude.

        Args:
            altitude_m: True altitude above sea level in meters.
            mach: Current Mach number for static port error modeling.
                  At transonic/supersonic speeds, shock waves and
                  compressibility distort the pressure at the static
                  ports on the rocket body.

        Returns:
            Dict with pressure (Pa) and temperature (K).
        """
        true_pressure = atm.pressure(altitude_m, self.ground_pressure_pa)
        true_temp = atm.temperature(altitude_m)

        # Static port position error — compressibility distorts local
        # pressure at the port openings on the rocket body.
        # Modeled as a fraction of dynamic pressure added to the static
        # reading.  The coefficient peaks at transonic (Mach ~1.0) where
        # the shock sits on or near the ports.
        if mach > 0.3:
            rho = atm.air_density(altitude_m)
            a = atm.speed_of_sound(altitude_m)
            q_dyn = 0.5 * rho * (mach * a) ** 2  # dynamic pressure (Pa)
            # Position error coefficient: ramps from 0 at M=0.3, peaks
            # near M=1, then decays above M=1.
            if mach < 1.0:
                Kp = 0.05 * ((mach - 0.3) / 0.7) ** 2
            else:
                Kp = 0.05 * max(0.0, 1.0 - (mach - 1.0) / 2.0)
            true_pressure += Kp * q_dyn  # port reads higher than freestream

        return {
            'pressure': true_pressure + self._rng.normal(0, self.pressure_noise_sigma),
            'temperature': true_temp + self._rng.normal(0, self.temp_noise_sigma),
        }
