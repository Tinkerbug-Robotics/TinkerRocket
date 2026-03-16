"""Python wrapper for C++ PID controller with gain scheduling."""
from tinkerrocket_sim._pid import PIDController


class RollController:
    """Roll rate PID controller with velocity-based gain scheduling.

    Gain schedule (from TR_ServoControl_ledc_mult):
        Kp_eff = Kp_base * min((V_ref / max(V, V_min))^2, max_scale)

    At high speed the gains are at their base values. At low speed
    the gains are scaled up (capped at max_scale) to compensate for
    reduced fin tab effectiveness.
    """

    def __init__(self,
                 kp: float = 0.10,
                 ki: float = 0.0,
                 kd: float = 0.0,
                 min_cmd: float = -20.0,
                 max_cmd: float = 20.0,
                 V_ref: float = 95.0,
                 V_min: float = 30.0,
                 max_scale: float = 3.0):
        self.kp_base = kp
        self.ki_base = ki
        self.kd_base = kd
        self.V_ref = V_ref
        self.V_min = V_min
        self.max_scale = max_scale
        self._pid = PIDController(kp, ki, kd, min_cmd, max_cmd)

    def compute(self, setpoint: float, measurement: float,
                dt: float, airspeed: float = None) -> float:
        """Compute roll command with optional gain scheduling.

        Args:
            setpoint: Desired roll rate (deg/s).
            measurement: Measured roll rate (deg/s).
            dt: Timestep in seconds.
            airspeed: Current airspeed in m/s (enables gain scheduling).

        Returns:
            Fin tab deflection command in degrees.
        """
        if airspeed is not None and airspeed > 1.0:
            scale = min((self.V_ref / max(airspeed, self.V_min)) ** 2,
                        self.max_scale)
            self._pid.set_kp(self.kp_base * scale)
            self._pid.set_ki(self.ki_base * scale)
            self._pid.set_kd(self.kd_base * scale)
        else:
            self._pid.set_kp(self.kp_base)
            self._pid.set_ki(self.ki_base)
            self._pid.set_kd(self.kd_base)

        return self._pid.compute(setpoint, measurement, dt)

    def reset(self):
        self._pid.reset()

    def set_gains(self, kp: float, ki: float, kd: float):
        self.kp_base = kp
        self.ki_base = ki
        self.kd_base = kd
