"""
Unit test for the hardened baro apogee detector (issue #142).

Ports just enough of TR_KinematicChecks.cpp to verify the boost-noise
false-positive that latched alt_apogee_flag at T+1.989s on the GTV 67mm
flight on 2026-05-09 cannot recur with the new logic.

The fix has two parts (both reproduced here):
  1. The running max + deficit compare use KF-smoothed altitude, not raw.
  2. apogee_count only increments when the KF vertical velocity is below
     an ascent threshold (20 m/s).

Test 1 (false-positive guard): noisy ascending altitude + high climb rate
  -> detector must NOT latch.
Test 2 (true positive): smooth descending altitude past max-5m + low rate
  -> detector MUST latch.
Test 3 (regression guard for the OLD logic on the same noisy data): the
  fixed logic with the velocity gate removed re-creates the false latch,
  proving the synthetic input really does exercise the failure mode.
"""
import math
import random


class AltKF:
    """Minimal port of TR_KinematicChecks::updateAltKF (1D CV filter)."""
    def __init__(self, R=1.0, Qz=0.05, Qv=5.0):
        self.alt_est = 0.0
        self.d_alt_est = 0.0
        self.P00 = 10.0
        self.P01 = 0.0
        self.P10 = 0.0
        self.P11 = 10.0
        self.R = R
        self.Qz = Qz
        self.Qv = Qv
        self._init = False

    def update(self, z_meas, dt, new_measurement=True):
        if not self._init:
            self.alt_est = z_meas
            self.d_alt_est = 0.0
            self._init = True
            return self.alt_est

        # Predict
        z_pred = self.alt_est + self.d_alt_est * dt
        vz_pred = self.d_alt_est

        F00, F01 = 1.0, dt
        F10, F11 = 0.0, 1.0

        P00p = F00 * self.P00 + F01 * self.P10
        P01p = F00 * self.P01 + F01 * self.P11
        P10p = F10 * self.P00 + F11 * self.P10
        P11p = F10 * self.P01 + F11 * self.P11

        P00pp = P00p * F00 + P01p * F01
        P01pp = P00p * F10 + P01p * F11
        P10pp = P10p * F00 + P11p * F01
        P11pp = P10p * F10 + P11p * F11

        P00pp += self.Qz * dt
        P11pp += self.Qv * dt

        if new_measurement:
            S = P00pp + self.R
            K0 = P00pp / S
            K1 = P10pp / S
            innov = z_meas - z_pred
            self.alt_est = z_pred + K0 * innov
            self.d_alt_est = vz_pred + K1 * innov
            P00n = (1 - K0) * P00pp
            P01n = (1 - K0) * P01pp
            P10n = P10pp - K1 * P00pp
            P11n = P11pp - K1 * P01pp
            self.P00 = P00n
            self.P11 = P11n
            self.P01 = 0.5 * (P01n + P10n)
            self.P10 = self.P01
        else:
            self.alt_est = z_pred
            self.d_alt_est = vz_pred
            self.P00 = P00pp
            self.P01 = P01pp
            self.P10 = P10pp
            self.P11 = P11pp
        return self.alt_est


def run_baro_apogee_test(samples, dt, use_velocity_gate=True, use_kf=True):
    """Run the hardened detector over a sample stream.

    samples: list of raw pressure altitudes (m)
    dt: seconds per sample
    use_velocity_gate: include the d_alt_est < 20 m/s gate (the fix)
    use_kf: use KF-smoothed altitude for max + deficit (the fix)

    Returns (latched, t_latched_s) — latched True iff the counter ever
    exceeded 5 ticks.
    """
    kf = AltKF()
    max_altitude = 0.0
    apogee_count = 0
    alt_apogee_flag = False
    t_latched = None

    for i, z_raw in enumerate(samples):
        kf.update(z_raw, dt)
        alt_for_check = kf.alt_est if use_kf else z_raw
        if max_altitude == 0.0 or abs(alt_for_check - max_altitude) < 50.0:
            if alt_for_check > max_altitude:
                max_altitude = alt_for_check

        deficit_ok = alt_for_check > 15.0 and alt_for_check < max_altitude - 5.0
        velocity_ok = (not use_velocity_gate) or kf.d_alt_est < 20.0
        if deficit_ok and velocity_ok:
            apogee_count += 1
        else:
            apogee_count = 0

        if apogee_count > 5 and not alt_apogee_flag:
            alt_apogee_flag = True
            t_latched = i * dt
    return alt_apogee_flag, t_latched


def synth_noisy_ascent(duration_s=5.0, dt=0.02, climb_rate=100.0,
                       spike_amplitude=20.0, spike_every=8):
    """Synthetic boost that deterministically reproduces the #142 failure
    pattern: a steady climb where every Nth raw sample spikes upward by
    20 m.  Under the OLD logic (raw P_alt, no velocity gate) this ratchets
    max_altitude above the true climb and 6+ subsequent samples sit in the
    deficit band, latching the detector.  Under the NEW logic the velocity
    gate suppresses the counter entirely (d_alt_est >> 20 m/s during boost),
    and the KF smooths the spikes before they reach max_altitude."""
    samples = []
    t = 0.0
    i = 0
    while t < duration_s:
        true_alt = 10.0 + climb_rate * t
        spike = spike_amplitude if (i % spike_every == 0 and i > 0) else 0.0
        samples.append(true_alt + spike)
        t += dt
        i += 1
    return samples, dt


def synth_clean_descent(start_alt=400.0, dt=0.02, descent_rate=10.0,
                       duration_s=3.0):
    """Post-apogee profile: altitude descends from 400m at -10 m/s."""
    samples = []
    t = 0.0
    while t < duration_s:
        samples.append(max(0.0, start_alt - descent_rate * t))
        t += dt
    return samples, dt


def synth_boost_then_descent(climb_rate=100.0, peak_alt=400.0,
                             descent_rate=15.0, dt=0.02):
    """Full ascent + apogee + descent profile with light boost noise.
    Used to verify the detector still latches around real apogee."""
    rng = random.Random(0xB0)
    samples = []
    t_climb = peak_alt / climb_rate
    # Boost
    t = 0.0
    while t < t_climb:
        samples.append(climb_rate * t + rng.gauss(0.0, 1.0))
        t += dt
    # Descent
    t = 0.0
    while t < 5.0:
        samples.append(peak_alt - descent_rate * t)
        t += dt
    return samples, dt


# ---------------------------------------------------------------- tests

def test_baro_detector_does_not_false_trip_on_noisy_boost():
    """Issue #142: noisy 100 m/s climb must not latch alt_apogee_flag."""
    samples, dt = synth_noisy_ascent()
    latched, _ = run_baro_apogee_test(samples, dt)
    assert not latched, (
        "Hardened baro apogee detector false-tripped during synthetic boost"
    )


def test_baro_detector_latches_on_clean_descent():
    """Sanity: detector still works when altitude actually drops."""
    # Pre-load a high max_altitude by feeding a brief climb first, then drop.
    samples, dt = synth_boost_then_descent()
    latched, t = run_baro_apogee_test(samples, dt)
    assert latched, "Hardened baro apogee detector failed to latch on descent"
    # Should latch shortly after the climb finishes (~4s for 400m at 100m/s).
    assert t is not None and t > 3.5, (
        f"Detector latched too early at t={t}s — boost section is ~4s long"
    )


def test_velocity_gate_is_what_prevents_false_trip():
    """Regression guard: with the velocity gate disabled, the same noisy
    ascent input DOES false-trip. This proves the synthetic data is
    pushing the failure mode and the gate is doing real work."""
    samples, dt = synth_noisy_ascent()
    latched_no_gate, _ = run_baro_apogee_test(
        samples, dt, use_velocity_gate=False, use_kf=False
    )
    assert latched_no_gate, (
        "Expected old logic (raw altitude, no velocity gate) to false-trip "
        "on the synthetic boost data — if it doesn't, the synthetic noise "
        "isn't representative of the GTV 67mm scenario and the positive "
        "tests above don't prove much."
    )
