"""Python port of TR_KinematicChecks.cpp for offline replay.

Line-for-line mirror of tinkerrocket-idf/components/TR_KinematicChecks/
TR_KinematicChecks.{cpp,h}, with experimental detection changes flagged
as ``CHANGED:`` to keep the diff visible for back-porting to C++.

Active experiments (all behind no flag — edit to disable):
  1. Baro rate-gate at ingestion (reject |Δpalt|/dt > 300 m/s, with a
     consecutive-reject safety valve so a wedged sensor reseeds).
  2. GPS max-altitude spike-reject window removed (boost-phase GPS
     dropouts on big flights leave a 500 m+ post-recovery jump that
     the 100 m window permanently rejected — Eagle Claw 5/17 was
     silently no-vote because of this).
  3. Apogee block gated on ``launch_flag && burnout_detected`` (was
     burnout only; defensive against pre-launch noise).
  4. Sub-detector flags are leaky-counter hysteresis rather than
     single-write latches, so a false-positive sample can un-vote
     once the underlying condition reverses. Master ``apogee_flag``
     still latches once true.

Run replay_kinematic_checks.py after editing to validate against
known flights before porting any change back to C++ source.

C++ line references appear as ``# TRKC.cpp:NN`` for cross-reference.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


# TRKC.cpp:10-13 (anonymous namespace constants for landing fast path)
LANDING_IMPACT_G       = 15.0
LANDING_IMPACT_ALT_M   = 20.0
LANDING_IMPACT_COUNT   = 5
G_MS2                  = 9.80665

# CHANGED (1): Baro rate-gate.
# 300 m/s headroom above max plausible rocket dynamic (~200 m/s peak
# ascent or terminal). Ejection spikes are O(10–100 km/s) so they're
# rejected by 2+ orders of magnitude. self-scales with dt: at a 1 s
# dropout the allowed Δ is 300 m, at 5 ms it's 1.5 m.
MAX_BARO_RATE_MS = 300.0
# After this many consecutive rejects the gate accepts the next sample
# unconditionally and reseeds — prevents a permanently wedged sensor
# from leaving us baro-blind for the rest of the flight.
MAX_CONSEC_BARO_REJECTS = 10

# CHANGED (4): Apogee sub-detector hysteresis thresholds.
# Each test runs a leaky counter (+1 on pass, -1 on miss, clamped).
# Flag latches true at HI, un-latches at zero. The HI thresholds are
# chosen to preserve the original C++ time-to-fire behaviour:
#   Baro/Pitch/Vel run at kinematicChecks rate (~1 kHz), so HI=6 →
#     ~6 ms to fire (same as the original ``apogee_count > 5``).
#   GPS runs at GNSS rate (~5 Hz), so HI=4 → ~800 ms to fire (same
#     as the original ``gps_apogee_count_ > 3``).
APOGEE_COUNT_MAX     = 10
APOGEE_COUNT_HI      = 6        # baro/pitch/vel latch-on threshold
GPS_APOGEE_COUNT_MAX = 8
GPS_APOGEE_COUNT_HI  = 4

# CHANGED (5): Landing sub-detector hysteresis thresholds.
# Slow detectors evaluated at 1 Hz inside the existing landing_check_dt
# gate. HI=4 → ~4 s to fire (matches the original ``landing_checks > 4``).
LANDING_SLOW_COUNT_MAX = 8
LANDING_SLOW_COUNT_HI  = 4

# Landing sub-detector thresholds.
# Lower bound is −50 m, not 0, because ground-pressure drift across a
# 60-90 s flight can put a settled palt several m below the pre-launch
# reference (Roly Poly GTV 5/9 settled at palt ≈ −8 m). The rate-gate
# upstream filters deep spikes, so this is just a defensive backstop.
BARO_STABLE_PALT_MIN     = -50.0
BARO_STABLE_PALT_MAX     = 50.0
BARO_STABLE_DELTA_MAX    = 2.0     # |Δpalt| over 1 s
BARO_STABLE_MAX_ALT_MIN  = 15.0    # rocket must have flown
GYRO_QUIET_DPS           = 20.0    # |roll_rate| < this
GPS_STATIONARY_SPEED_MS  = 1.0     # |velocity| < this (EKF proxy)
ACCEL_1G_TOLERANCE       = 2.94    # |acc_mag − 1g| < this (≈ 0.3 g)


def _constrain(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class KinematicChecks:
    """Mirror of TR_KinematicChecks (public + private fields)."""

    # ── Public output flags / values ──────────────────────────────────
    launch_flag: bool = False
    alt_landed_flag: bool = False
    alt_apogee_flag: bool = False      # Test 2: baro altitude decreasing
    vel_u_apogee_flag: bool = False    # Test 1: EKF velocity negative
    gps_apogee_flag: bool = False      # Test 3: GPS altitude decreasing
    pitch_apogee_flag: bool = False    # Test 4: pitch below horizontal
    apogee_flag: bool = False          # Voted master
    # Landing sub-detector flags (#166) — exposed for diagnostics/logging.
    impact_flag: bool = False          # one-shot: high-G ground impact
    baro_stable_flag: bool = False     # palt low and stable
    gyro_quiet_flag: bool = False      # roll-rate quiescent
    gps_stationary_flag: bool = False  # EKF speed near zero (GPS fresh)
    accel_1g_flag: bool = False        # acc_mag near 1 g
    max_altitude: float = 0.0
    max_speed: float = 0.0
    alt_est: float = 0.0
    d_alt_est_: float = 0.0

    # ── Private state ─────────────────────────────────────────────────
    launch_count: int = 0
    landing_check_time: int = 0
    landing_look_back_alt: float = 0.0
    landing_check_dt: int = 1000
    apogee_count: int = 0             # Baro test counter (now leaky)
    landing_checks: int = 0
    impact_seen_count: int = 0

    # CHANGED (4): new per-detector counters for un-latching hysteresis.
    vel_apogee_count_: int = 0
    pitch_apogee_count_: int = 0

    # CHANGED (5): landing sub-detector counters.
    baro_stable_count_: int = 0
    gyro_quiet_count_: int = 0
    gps_stationary_count_: int = 0
    accel_1g_count_: int = 0

    max_gps_altitude_: float = 0.0
    gps_apogee_count_: int = 0
    gps_available_: bool = False
    last_gps_time_ms_: int = 0

    # CHANGED (1): baro rate-gate state.
    _baro_gate_init: bool = False
    _palt_accepted: float = 0.0
    _last_baro_gate_ms: int = 0
    _consec_baro_rejects: int = 0

    # Per-tick pass-state outputs (exposed for replay/plot — not in C++).
    last_vel_pass: bool = False
    last_baro_pass: bool = False
    last_gps_pass: bool = False    # only meaningful on new_gps ticks
    last_pitch_pass: bool = False

    # KF state
    kf_init_: bool = False
    P00_: float = 10.0
    P01_: float = 0.0
    P10_: float = 0.0
    P11_: float = 10.0
    last_ms_: int = 0

    # KF tunables (TRKC.cpp:49-55)
    R_: float  = 1.0
    Qz_: float = 0.05
    Qv_: float = 5.0

    _now_ms: int = field(default=0, repr=False)

    def reset(self) -> None:
        """TRKC.cpp:58 reset()."""
        self.launch_flag = False
        self.alt_landed_flag = False
        self.alt_apogee_flag = False
        self.vel_u_apogee_flag = False
        self.gps_apogee_flag = False
        self.pitch_apogee_flag = False
        self.apogee_flag = False
        self.launch_count = 0
        self.max_altitude = 0.0
        self.max_speed = 0.0
        self.landing_check_time = 0
        self.landing_look_back_alt = 0.0
        self.apogee_count = 0
        self.landing_checks = 0
        self.impact_seen_count = 0
        self.vel_apogee_count_ = 0
        self.pitch_apogee_count_ = 0
        self.impact_flag = False
        self.baro_stable_flag = False
        self.gyro_quiet_flag = False
        self.gps_stationary_flag = False
        self.accel_1g_flag = False
        self.baro_stable_count_ = 0
        self.gyro_quiet_count_ = 0
        self.gps_stationary_count_ = 0
        self.accel_1g_count_ = 0
        self.max_gps_altitude_ = 0.0
        self.gps_apogee_count_ = 0
        self.gps_available_ = False
        self.last_gps_time_ms_ = 0
        self._baro_gate_init = False
        self._palt_accepted = 0.0
        self._last_baro_gate_ms = 0
        self._consec_baro_rejects = 0
        self.kf_init_ = False
        self.alt_est = 0.0
        self.d_alt_est_ = 0.0
        self.P00_ = 10.0; self.P01_ = 0.0; self.P10_ = 0.0; self.P11_ = 10.0

    # TRKC.cpp:86
    def kinematic_checks(
        self,
        pressure_altitude: float,
        acc_mag: float,
        position: tuple[float, float, float],
        velocity: tuple[float, float, float],
        roll_rate: float,
        new_baro: bool,
        gps_altitude: float,
        new_gps: bool,
        pitch_rad: float,
        burnout_detected: bool,
        baro_locked_out: bool,
        now_ms: int,
    ) -> None:
        self._now_ms = now_ms

        # Snapshot apogee_flag so the rising-edge reset below sees the
        # state *before* this tick's apogee voting fires (#192).
        apogee_was_set = self.apogee_flag

        # ── CHANGED (1): Baro rate-gate at ingestion ──────────────────
        # Only the new_baro sample is judged. Once accepted (or substituted
        # with the last good value) all downstream paths — KF update,
        # landing checks, baro apogee test — see the gated value.
        if new_baro:
            if self._baro_gate_init:
                dt = max(0.001, (now_ms - self._last_baro_gate_ms) * 0.001)
                rate = abs(pressure_altitude - self._palt_accepted) / dt
                if (rate > MAX_BARO_RATE_MS
                        and self._consec_baro_rejects < MAX_CONSEC_BARO_REJECTS):
                    # Spike: don't pass to KF, hold last good for downstream.
                    self._consec_baro_rejects += 1
                    new_baro = False
                    pressure_altitude = self._palt_accepted
                else:
                    # Accept (either truly OK, or reseed after wedge).
                    self._consec_baro_rejects = 0
                    self._palt_accepted = pressure_altitude
                    self._last_baro_gate_ms = now_ms
            else:
                self._palt_accepted = pressure_altitude
                self._last_baro_gate_ms = now_ms
                self._baro_gate_init = True
        else:
            pressure_altitude = self._palt_accepted

        # TRKC.cpp:105 KF predict (every call) / update (when new_baro).
        self._update_alt_kf(pressure_altitude, new_baro)

        # TRKC.cpp:112 ─ Launch detection
        if not self.launch_flag:
            if acc_mag > 20.0:
                self.launch_count += 1
                if self.launch_count > 50 and self.d_alt_est_ > 1.0:
                    self.launch_flag = True
            else:
                self.launch_count = 0

        # TRKC.cpp:133 ─ max_altitude with 50 m spike-reject window
        if self.max_altitude == 0.0 or abs(self.alt_est - self.max_altitude) < 50.0:
            self.max_altitude = max(self.max_altitude, self.alt_est)

        # ── CHANGED (5): Landing voting system (#166) ────────────────
        # Fast path: high-G impact gated on apogee + ground proximity.
        # One-shot latch — once a real impact is detected, the flag stays
        # true so it can participate in voting alongside slower detectors.
        # Defensive ``palt > -10`` lower bound mirrors the rate-gate intent.
        if (self.apogee_flag
                and pressure_altitude > -10.0
                and pressure_altitude < LANDING_IMPACT_ALT_M
                and acc_mag > LANDING_IMPACT_G * G_MS2):
            self.impact_seen_count += 1
            if self.impact_seen_count >= LANDING_IMPACT_COUNT:
                self.impact_flag = True
        else:
            self.impact_seen_count = 0

        # Slow detectors evaluated at 1 Hz inside the existing time-gate.
        # Each is a leaky-counter sub-flag like the apogee tests.
        if self._now_ms > self.landing_check_time + self.landing_check_dt:
            landing_alt_change = abs(pressure_altitude - self.landing_look_back_alt)
            self.landing_look_back_alt = pressure_altitude
            self.landing_check_time = self._now_ms

            # Sub 1: Baro-stable (was the original slow path; +lower bound)
            baro_stable_pass = (BARO_STABLE_PALT_MIN <= pressure_altitude <= BARO_STABLE_PALT_MAX
                                and landing_alt_change < BARO_STABLE_DELTA_MAX
                                and self.max_altitude > BARO_STABLE_MAX_ALT_MIN)
            if baro_stable_pass:
                self.baro_stable_count_ = min(LANDING_SLOW_COUNT_MAX,
                                              self.baro_stable_count_ + 1)
            else:
                self.baro_stable_count_ = max(0, self.baro_stable_count_ - 1)
            if self.baro_stable_count_ >= LANDING_SLOW_COUNT_HI:
                self.baro_stable_flag = True
            elif self.baro_stable_count_ == 0:
                self.baro_stable_flag = False

            # Sub 2: Gyro-quiet (roll_rate; future: generalize to all axes)
            gyro_quiet_pass = abs(roll_rate) < GYRO_QUIET_DPS
            if gyro_quiet_pass:
                self.gyro_quiet_count_ = min(LANDING_SLOW_COUNT_MAX,
                                             self.gyro_quiet_count_ + 1)
            else:
                self.gyro_quiet_count_ = max(0, self.gyro_quiet_count_ - 1)
            if self.gyro_quiet_count_ >= LANDING_SLOW_COUNT_HI:
                self.gyro_quiet_flag = True
            elif self.gyro_quiet_count_ == 0:
                self.gyro_quiet_flag = False

            # Sub 3: GPS-stationary (EKF speed magnitude as proxy)
            if (self.gps_available_
                    and (self._now_ms - self.last_gps_time_ms_) < 5000):
                speed = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
                gps_stationary_pass = speed < GPS_STATIONARY_SPEED_MS
                if gps_stationary_pass:
                    self.gps_stationary_count_ = min(LANDING_SLOW_COUNT_MAX,
                                                     self.gps_stationary_count_ + 1)
                else:
                    self.gps_stationary_count_ = max(0, self.gps_stationary_count_ - 1)
                if self.gps_stationary_count_ >= LANDING_SLOW_COUNT_HI:
                    self.gps_stationary_flag = True
                elif self.gps_stationary_count_ == 0:
                    self.gps_stationary_flag = False

            # Sub 4: Accel-1g floor
            accel_1g_pass = abs(acc_mag - G_MS2) < ACCEL_1G_TOLERANCE
            if accel_1g_pass:
                self.accel_1g_count_ = min(LANDING_SLOW_COUNT_MAX,
                                           self.accel_1g_count_ + 1)
            else:
                self.accel_1g_count_ = max(0, self.accel_1g_count_ - 1)
            if self.accel_1g_count_ >= LANDING_SLOW_COUNT_HI:
                self.accel_1g_flag = True
            elif self.accel_1g_count_ == 0:
                self.accel_1g_flag = False

        # Voting: impact alone fires master; otherwise N-1 of N over the
        # slow detectors (GPS excluded if stale). Master latches once true.
        # Gated on apogee_flag — pre-flight a static rocket trips
        # gyro_quiet + gps_stationary + accel_1g, which would otherwise
        # be 3 of 4 votes and fire landed on the pad.
        if not self.alt_landed_flag and self.apogee_flag:
            if self.impact_flag:
                self.alt_landed_flag = True
            else:
                available = 0
                passed = 0

                available += 1
                if self.baro_stable_flag: passed += 1

                available += 1
                if self.gyro_quiet_flag: passed += 1

                if (self.gps_available_
                        and (self._now_ms - self.last_gps_time_ms_) < 5000):
                    available += 1
                    if self.gps_stationary_flag: passed += 1

                available += 1
                if self.accel_1g_flag: passed += 1

                if available >= 2 and passed >= 2 and passed >= (available - 1):
                    self.alt_landed_flag = True

        # TRKC.cpp:189 ─ GPS max altitude tracking (post-launch)
        # CHANGED (2): no spike-reject window. The 100 m gate left
        # Eagle Claw silently no-voting because GPS dropped out
        # through boost and re-acquired at +500 m past the gate.
        if new_gps and self.launch_flag:
            self.gps_available_ = True
            self.last_gps_time_ms_ = self._now_ms
            self.max_gps_altitude_ = max(self.max_gps_altitude_, gps_altitude)

        # Per-tick pass states default false (overwritten if apogee block runs).
        self.last_vel_pass = False
        self.last_baro_pass = False
        self.last_pitch_pass = False
        # last_gps_pass is only updated on new_gps ticks; keep prior value.

        # TRKC.cpp:209 ─ Apogee detection
        # CHANGED (3): added launch_flag gate (was burnout only).
        if self.launch_flag and burnout_detected:
            # ─ Test 1: Velocity (CHANGED (4): leaky counter) ─
            vel_pass = position[2] > 15.0 and velocity[2] < 0.0
            self.last_vel_pass = vel_pass
            if vel_pass:
                self.vel_apogee_count_ = min(APOGEE_COUNT_MAX,
                                              self.vel_apogee_count_ + 1)
            else:
                self.vel_apogee_count_ = max(0, self.vel_apogee_count_ - 1)
            if self.vel_apogee_count_ >= APOGEE_COUNT_HI:
                self.vel_u_apogee_flag = True
            elif self.vel_apogee_count_ == 0:
                self.vel_u_apogee_flag = False

            # ─ Test 2: Baro (CHANGED (4): leaky counter; same HI=6) ─
            baro_pass = (self.alt_est > 15.0
                         and self.alt_est < self.max_altitude - 5.0
                         and self.d_alt_est_ < 20.0)
            self.last_baro_pass = baro_pass
            if baro_pass:
                self.apogee_count = min(APOGEE_COUNT_MAX, self.apogee_count + 1)
            else:
                self.apogee_count = max(0, self.apogee_count - 1)
            if self.apogee_count >= APOGEE_COUNT_HI:
                self.alt_apogee_flag = True
            elif self.apogee_count == 0:
                self.alt_apogee_flag = False

            # ─ Test 3: GPS (CHANGED (4): leaky counter; same HI=4) ─
            if new_gps and self.gps_available_ and gps_altitude > 15.0:
                gps_pass = gps_altitude < self.max_gps_altitude_ - 10.0
                self.last_gps_pass = gps_pass
                if gps_pass:
                    self.gps_apogee_count_ = min(GPS_APOGEE_COUNT_MAX,
                                                  self.gps_apogee_count_ + 1)
                else:
                    self.gps_apogee_count_ = max(0, self.gps_apogee_count_ - 1)
                if self.gps_apogee_count_ >= GPS_APOGEE_COUNT_HI:
                    self.gps_apogee_flag = True
                elif self.gps_apogee_count_ == 0:
                    self.gps_apogee_flag = False

            # ─ Test 4: Pitch (CHANGED (4): leaky counter) ─
            pitch_pass = pitch_rad < -0.087
            self.last_pitch_pass = pitch_pass
            if pitch_pass:
                self.pitch_apogee_count_ = min(APOGEE_COUNT_MAX,
                                                self.pitch_apogee_count_ + 1)
            else:
                self.pitch_apogee_count_ = max(0, self.pitch_apogee_count_ - 1)
            if self.pitch_apogee_count_ >= APOGEE_COUNT_HI:
                self.pitch_apogee_flag = True
            elif self.pitch_apogee_count_ == 0:
                self.pitch_apogee_flag = False

            # N-1 of N voting (TRKC.cpp:264) — master still latches once true.
            if not self.apogee_flag:
                available = 0
                passed = 0

                available += 1
                if self.vel_u_apogee_flag: passed += 1

                if not baro_locked_out:
                    available += 1
                    if self.alt_apogee_flag: passed += 1

                if (self.gps_available_
                        and (self._now_ms - self.last_gps_time_ms_) < 5000):
                    available += 1
                    if self.gps_apogee_flag: passed += 1

                available += 1
                if self.pitch_apogee_flag: passed += 1

                if available >= 2 and passed >= 2 and passed >= (available - 1):
                    self.apogee_flag = True

        # ── Apogee rising edge: reset landing sub-flag counters (#192) ──
        # The 1 Hz sub-detectors above accumulate evidence regardless of
        # flight state, so a rocket flying straight pre-apogee (low roll
        # rate, ~0 m/s in EKF frame at burnout, ~1g during coast) latches
        # gyro_quiet / gps_stationary / accel_1g before apogee. The
        # master-vote apogee gate ([landing voting block above]) prevents
        # a false LANDED, but the latched sub-flags carry pre-apogee
        # history into the post-apogee vote — eroding the 3-of-4 margin.
        # Zero the counters + flags at the transition so post-apogee
        # voting reflects only post-apogee evidence.
        if self.apogee_flag and not apogee_was_set:
            self.baro_stable_count_ = 0
            self.gyro_quiet_count_ = 0
            self.gps_stationary_count_ = 0
            self.accel_1g_count_ = 0
            self.baro_stable_flag = False
            self.gyro_quiet_flag = False
            self.gps_stationary_flag = False
            self.accel_1g_flag = False

        # TRKC.cpp:302 ─ Pre-apogee max speed
        speed = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
        if not self.apogee_flag and speed > self.max_speed:
            self.max_speed = speed

    # TRKC.cpp:328 ─ 1D CV Kalman filter (unchanged)
    def _update_alt_kf(self, z_meas: float, new_measurement: bool) -> float:
        now = self._now_ms
        dt = 0.02
        if self.kf_init_:
            dms = now - self.last_ms_
            dt = _constrain(dms * 0.001, 0.0005, 0.200)
        self.last_ms_ = now

        if not self.kf_init_:
            self.alt_est = z_meas
            self.d_alt_est_ = 0.0
            self.kf_init_ = True
            return self.alt_est

        z_pred  = self.alt_est + self.d_alt_est_ * dt
        vz_pred = self.d_alt_est_

        F00, F01 = 1.0, dt
        F10, F11 = 0.0, 1.0

        P00p = F00*self.P00_ + F01*self.P10_
        P01p = F00*self.P01_ + F01*self.P11_
        P10p = F10*self.P00_ + F11*self.P10_
        P11p = F10*self.P01_ + F11*self.P11_

        P00pp = P00p*F00 + P01p*F01
        P01pp = P00p*F10 + P01p*F11
        P10pp = P10p*F00 + P11p*F01
        P11pp = P10p*F10 + P11p*F11

        P00pp += self.Qz_ * dt
        P11pp += self.Qv_ * dt

        if new_measurement:
            S  = P00pp + self.R_
            K0 = P00pp / S
            K1 = P10pp / S

            innov = z_meas - z_pred
            self.alt_est    = z_pred  + K0 * innov
            self.d_alt_est_ = vz_pred + K1 * innov

            P00n = (1.0 - K0) * P00pp
            P01n = (1.0 - K0) * P01pp
            P10n = P10pp - K1 * P00pp
            P11n = P11pp - K1 * P01pp

            self.P00_ = P00n
            self.P11_ = P11n
            self.P01_ = 0.5 * (P01n + P10n)
            self.P10_ = self.P01_
        else:
            self.alt_est    = z_pred
            self.d_alt_est_ = vz_pred
            self.P00_ = P00pp
            self.P11_ = P11pp
            self.P01_ = 0.5 * (P01pp + P10pp)
            self.P10_ = self.P01_

        return self.alt_est
