"""Fin tab aerodynamic torque model based on CFD analysis.

Model: tau_single = Kt_ref * (V / V_ref)^2 * delta
       tau_total  = n_tabs * tau_single

Where:
    Kt_ref  = 5.34e-3 N-m/deg at V_ref = 95 m/s (per tab, from 67mm fin can CFD)
    delta   = fin tab deflection in degrees
    V       = airspeed in m/s
    n_tabs  = number of fin tabs (3 for 67mm fin can)

The V^2 scaling is confirmed by CFD at 50, 95, and 160 m/s within ~4%.
Valid for |delta| <= 20 degrees (linear range, R^2 > 0.98).

CFD analysis: 67mm-fin-tab-analysis/ (March 2026)
Previous design: Kt_ref = 4.20e-3 at 95 m/s (57.4mm fin can, 4 tabs)
"""
import math


class FinTabAeroModel:
    """Fin tab roll torque model from CFD data."""

    def __init__(self,
                 Kt_ref: float = 5.34e-3,
                 V_ref: float = 95.0,
                 n_tabs: int = 3,
                 deflection_min: float = -20.0,
                 deflection_max: float = 20.0):
        """
        Args:
            Kt_ref: Torque coefficient at V_ref, N-m per degree per tab.
                    Default from 67mm fin can CFD (March 2026).
            V_ref: Reference velocity for Kt_ref, m/s.
            n_tabs: Number of fin tabs.
            deflection_min: Minimum deflection angle, degrees.
            deflection_max: Maximum deflection angle, degrees.
        """
        self.Kt_ref = Kt_ref
        self.V_ref = V_ref
        self.n_tabs = n_tabs
        self.deflection_min = deflection_min
        self.deflection_max = deflection_max

    def torque(self, airspeed: float, deflection_deg: float) -> float:
        """Compute total roll torque from all fin tabs.

        Args:
            airspeed: Airspeed magnitude in m/s.
            deflection_deg: Fin tab deflection in degrees (positive = positive roll).

        Returns:
            Total roll torque in N-m.
        """
        if abs(airspeed) < 1.0:
            return 0.0

        # Clamp deflection to valid range
        delta = max(self.deflection_min, min(self.deflection_max, deflection_deg))

        V_ratio_sq = (airspeed / self.V_ref) ** 2
        tau_single = self.Kt_ref * V_ratio_sq * delta
        return tau_single * self.n_tabs

    def torque_per_tab(self, airspeed: float, deflection_deg: float) -> float:
        """Compute roll torque from a single fin tab."""
        if abs(airspeed) < 1.0:
            return 0.0
        delta = max(self.deflection_min, min(self.deflection_max, deflection_deg))
        V_ratio_sq = (airspeed / self.V_ref) ** 2
        return self.Kt_ref * V_ratio_sq * delta

    def Kt_at_speed(self, airspeed: float) -> float:
        """Torque coefficient at a given airspeed, N-m/deg per tab."""
        if abs(airspeed) < 1.0:
            return 0.0
        return self.Kt_ref * (airspeed / self.V_ref) ** 2

    def roll_acceleration(self, airspeed: float, deflection_deg: float,
                          I_roll: float) -> float:
        """Compute roll angular acceleration from fin tab torque.

        Args:
            airspeed: m/s
            deflection_deg: degrees
            I_roll: Roll moment of inertia in kg-m^2

        Returns:
            Roll angular acceleration in rad/s^2
        """
        tau = self.torque(airspeed, deflection_deg)
        return tau / I_roll * math.pi / 180.0  # Convert because tau is in N-m/deg*deg
