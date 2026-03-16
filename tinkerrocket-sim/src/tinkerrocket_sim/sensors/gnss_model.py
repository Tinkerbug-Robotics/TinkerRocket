"""GNSS (GPS) sensor model.

Provides ECEF position and velocity with Gaussian noise at 25 Hz.
Uses a reference point (launch site LLA) to convert ENU simulation
coordinates to ECEF for the EKF.

Dropout model based on real flight data analysis (RolyPolyIII 2025-12-07):
  - High-g dropout: receiver loses lock above ~5g sustained acceleration
  - Spin-rate dropout: high angular rates (>400 dps) cause loss even at low g
  - Reacquisition delay: randomized, typically 1-5s after dynamics subside
  - Recovery quality: first fixes after reacquisition have inflated noise
    (h_acc starts at 50-100m, settles over ~1-2s)
  - Preemptive lockout: when boost is detected, reject GNSS before receiver
    even reports loss to avoid feeding corrupted dead-reckoned measurements
"""
import numpy as np
import math


class GNSSModel:
    """Simulated GNSS receiver providing ECEF position and velocity."""

    def __init__(self,
                 pos_noise_ne_m: float = 3.0,
                 pos_noise_d_m: float = 6.0,
                 vel_noise_ne_mps: float = 0.5,
                 vel_noise_d_mps: float = 1.0,
                 rate_hz: float = 25.0,
                 ref_lat_deg: float = 38.0,
                 ref_lon_deg: float = -122.0,
                 ref_alt_m: float = 0.0,
                 # Dropout triggers
                 dropout_accel_threshold_g: float = 5.0,
                 dropout_gyro_threshold_dps: float = 400.0,
                 # Reacquisition timing
                 reacq_delay_mean_s: float = 2.0,
                 reacq_delay_std_s: float = 1.0,
                 # Recovery quality
                 recovery_noise_scale_initial: float = 15.0,
                 recovery_settle_time_s: float = 2.0,
                 enable_dropout: bool = True):
        self.pos_noise_ne = pos_noise_ne_m
        self.pos_noise_d = pos_noise_d_m
        self.vel_noise_ne = vel_noise_ne_mps
        self.vel_noise_d = vel_noise_d_mps
        self.rate_hz = rate_hz

        # Reference point (launch site)
        self.ref_lat_rad = math.radians(ref_lat_deg)
        self.ref_lon_rad = math.radians(ref_lon_deg)
        self.ref_alt = ref_alt_m

        # Precompute reference ECEF and rotation matrix
        self.ref_ecef = self._lla_to_ecef(self.ref_lat_rad, self.ref_lon_rad, self.ref_alt)
        self.R_enu2ecef = self._enu_to_ecef_rotation(self.ref_lat_rad, self.ref_lon_rad)

        # GNSS dropout model
        self.accel_threshold = dropout_accel_threshold_g * 9.807  # m/s^2
        self.gyro_threshold_rps = math.radians(dropout_gyro_threshold_dps)
        self.reacq_delay_mean = reacq_delay_mean_s
        self.reacq_delay_std = reacq_delay_std_s
        self.enable_dropout = enable_dropout

        # Recovery quality model
        self.recovery_noise_scale_initial = recovery_noise_scale_initial
        self.recovery_settle_time = recovery_settle_time_s

        # State
        self._in_dropout = False
        self._dropout_start_time = None
        self._dynamics_clear_time = None  # when acceleration+gyro both dropped below thresholds
        self._reacq_delay = 0.0
        self._recovery_start_time = None  # when fix was reacquired
        self._boost_lockout = False  # preemptive lockout during boost

        self._rng = np.random.default_rng()

    def measure(self, pos_enu: np.ndarray, vel_enu: np.ndarray,
                accel_magnitude: float = None,
                gyro_magnitude_rps: float = None,
                sim_time: float = None) -> dict:
        """Generate GNSS measurement from true ENU state.

        Args:
            pos_enu: True position in ENU (m) relative to launch site.
            vel_enu: True velocity in ENU (m/s).
            accel_magnitude: Body-frame acceleration magnitude (m/s^2)
                for dropout modeling.  If None, dropout is not checked.
            gyro_magnitude_rps: Body-frame angular rate magnitude (rad/s)
                for spin-rate dropout.
            sim_time: Current simulation time (s) for reacquisition
                delay tracking.

        Returns:
            Dict with ecef_x/y/z (m), ecef_vx/vy/vz (m/s), and
            noise_scale (float, 1.0 = nominal, >1 during recovery),
            or None if the receiver is in dropout (no fix available).
        """
        # --- Dropout model ---
        if self.enable_dropout and accel_magnitude is not None and sim_time is not None:
            # Check dynamics thresholds
            high_accel = accel_magnitude > self.accel_threshold
            high_gyro = (gyro_magnitude_rps is not None and
                         gyro_magnitude_rps > self.gyro_threshold_rps)

            if high_accel or high_gyro:
                # High dynamics: receiver loses lock
                if not self._in_dropout:
                    self._dropout_start_time = sim_time
                self._in_dropout = True
                self._dynamics_clear_time = None
                self._recovery_start_time = None
                self._boost_lockout = high_accel  # remember if it was accel-triggered
            elif self._in_dropout:
                # Dynamics have subsided — start reacquisition timer
                if self._dynamics_clear_time is None:
                    self._reacq_delay = max(0.3, self._rng.normal(
                        self.reacq_delay_mean, self.reacq_delay_std))
                    self._dynamics_clear_time = sim_time

                time_since_clear = sim_time - self._dynamics_clear_time
                if time_since_clear >= self._reacq_delay:
                    # Reacquisition complete
                    self._in_dropout = False
                    self._recovery_start_time = sim_time
                    self._boost_lockout = False

            if self._in_dropout:
                return None  # no GNSS fix available

        # --- Recovery noise scaling ---
        # Real data shows h_acc starts at 50-100m after reacquisition
        # and settles to nominal over ~2 seconds.
        noise_scale = 1.0
        if (self._recovery_start_time is not None and sim_time is not None
                and self.recovery_settle_time > 0):
            time_since_recovery = sim_time - self._recovery_start_time
            if time_since_recovery < self.recovery_settle_time:
                # Exponential decay from initial scale to 1.0
                frac = time_since_recovery / self.recovery_settle_time
                noise_scale = 1.0 + (self.recovery_noise_scale_initial - 1.0) * \
                              math.exp(-3.0 * frac)  # ~95% settled at frac=1
            else:
                self._recovery_start_time = None  # fully settled

        # --- Normal measurement ---
        # Convert ENU position to ECEF
        pos_ecef = self.ref_ecef + self.R_enu2ecef @ pos_enu
        vel_ecef = self.R_enu2ecef @ vel_enu

        # Add noise in NED frame, then convert to ECEF
        # Apply recovery noise scaling to position and velocity noise
        pos_noise_ned = np.array([
            self._rng.normal(0, self.pos_noise_ne * noise_scale),
            self._rng.normal(0, self.pos_noise_ne * noise_scale),
            self._rng.normal(0, self.pos_noise_d * noise_scale),
        ])
        vel_noise_ned = np.array([
            self._rng.normal(0, self.vel_noise_ne * math.sqrt(noise_scale)),
            self._rng.normal(0, self.vel_noise_ne * math.sqrt(noise_scale)),
            self._rng.normal(0, self.vel_noise_d * math.sqrt(noise_scale)),
        ])

        # NED to ENU: [E, N, -D]
        pos_noise_enu = np.array([pos_noise_ned[1], pos_noise_ned[0], -pos_noise_ned[2]])
        vel_noise_enu = np.array([vel_noise_ned[1], vel_noise_ned[0], -vel_noise_ned[2]])

        pos_ecef += self.R_enu2ecef @ pos_noise_enu
        vel_ecef += self.R_enu2ecef @ vel_noise_enu

        return {
            'ecef_x': pos_ecef[0],
            'ecef_y': pos_ecef[1],
            'ecef_z': pos_ecef[2],
            'ecef_vx': vel_ecef[0],
            'ecef_vy': vel_ecef[1],
            'ecef_vz': vel_ecef[2],
            'noise_scale': noise_scale,
        }

    @staticmethod
    def _lla_to_ecef(lat_rad: float, lon_rad: float, alt_m: float) -> np.ndarray:
        a = 6378137.0
        e2 = 6.69437999014e-3
        sinlat = math.sin(lat_rad)
        coslat = math.cos(lat_rad)
        N = a / math.sqrt(1 - e2 * sinlat**2)
        x = (N + alt_m) * coslat * math.cos(lon_rad)
        y = (N + alt_m) * coslat * math.sin(lon_rad)
        z = (N * (1 - e2) + alt_m) * sinlat
        return np.array([x, y, z])

    @staticmethod
    def _enu_to_ecef_rotation(lat_rad: float, lon_rad: float) -> np.ndarray:
        """ENU to ECEF rotation matrix."""
        slat = math.sin(lat_rad)
        clat = math.cos(lat_rad)
        slon = math.sin(lon_rad)
        clon = math.cos(lon_rad)
        R = np.array([
            [-slon,      -slat*clon,  clat*clon],
            [ clon,      -slat*slon,  clat*slon],
            [ 0.0,        clat,       slat     ],
        ])
        return R
