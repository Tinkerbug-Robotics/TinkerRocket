"""#741 — a drifted filter must keep being pulled back at full strength.

The GNSS position/velocity update de-weights inconsistent measurements instead
of rejecting them (the #174 split-NIS gate). That is the right idea, but the
inflation law decides whether a drifted filter can ever recover.

NIS goes as y^2, so inflating R by (NIS/gate) makes the applied correction

    K*y  ~  P*y / (P + R*y^2/gate)   ->   ~ 1/y   for large y

— a REDESCENDING influence function: the further the estimate has drifted, the
LESS each good fix pulls it back. Huber's defining property is the opposite,
influence BOUNDED rather than vanishing, which is what sqrt(NIS/gate) gives.

This is not academic. On the CENJARS flight the nav filter and GNSS diverge
during a 1.9 s boost satellite outage and land 81 m apart, and the gap GROWS
over the remaining 72 s even though the receiver recovers to 21 satellites and
PDOP 1.10.

The invariant pinned here is the bounded-influence one: as the error grows, the
absolute distance recovered per unit time must NOT go down. The fractional
recovery legitimately falls (bounded pull over a bigger gap); the absolute pull
must not.
"""
import math
import pytest

from tinkerrocket_sim._ekf import GpsInsEKF, IMUData, GNSSDataLLA, MagData

LAT, LON, ALT = math.radians(33.7), math.radians(-118.4), 100.0
R_E = 6371000.0
EKF_HZ, GNSS_HZ = 486.0, 18.0


def _imu(t):
    d = IMUData()
    d.time_us = t
    d.acc_x = d.acc_y = 0.0
    d.acc_z = 9.807            # gravity, FRD, nose up
    d.gyro_x = d.gyro_y = d.gyro_z = 0.0
    return d


def _gnss(t, east_m=0.0):
    g = GNSSDataLLA()
    g.time_us = t
    g.lat_rad = LAT
    g.lon_rad = LON + east_m / (R_E * math.cos(LAT))
    g.alt_m = ALT
    g.vel_n_mps = g.vel_e_mps = g.vel_d_mps = 0.0
    return g


def _mag(t):
    m = MagData()
    m.time_us = t
    m.mag_x, m.mag_y, m.mag_z = 22.0, 0.0, 42.0
    return m


def _east_of(ekf):
    return (ekf.get_position()[1] - LON) * R_E * math.cos(LAT)


def _recovered_m(offset_m, settle_s=20.0, after_s=10.0):
    """Metres of a step offset the filter closes in `after_s`.

    Settling first matters: the filter must be CONFIDENT (small P) when the
    large innovation arrives, which is the post-outage condition. A freshly
    initialised filter has P large enough that the gate never trips and the
    bug is invisible — that is why this reproduces only after a settle.
    """
    ekf = GpsInsEKF()
    ekf.init_lla(_imu(0), _gnss(0), _mag(0))
    dt, gi = int(1e6 / EKF_HZ), int(1e6 / GNSS_HZ)
    t, nxt, g = 0, int(1e6 / GNSS_HZ), _gnss(0)

    for _ in range(int(settle_s * EKF_HZ)):
        t += dt
        if t >= nxt:
            g = _gnss(t)
            nxt += gi
        ekf.update_lla(False, _imu(t), g, _mag(t))

    before = _east_of(ekf)
    for _ in range(int(after_s * EKF_HZ)):
        t += dt
        if t >= nxt:
            g = _gnss(t, offset_m)
            nxt += gi
        ekf.update_lla(False, _imu(t), g, _mag(t))
    return _east_of(ekf) - before


# 81 m is the measured CENJARS nav-vs-GNSS landing separation.
OFFSETS = [30.0, 50.0, 81.0]


@pytest.mark.parametrize("offset", OFFSETS)
def test_a_drifted_filter_still_recovers(offset):
    """Some meaningful fraction must come back, at every magnitude."""
    got = _recovered_m(offset)
    assert got > 0.10 * offset, (
        f"{offset:.0f} m step: only {got:.1f} m recovered in 10 s — the filter "
        f"is barely responding to good fixes"
    )


def test_recovery_does_not_slow_as_the_error_grows():
    """THE invariant: bounded influence, not redescending.

    Under the old (NIS/gate) law the absolute recovery FELL as the error grew
    — 30 m -> 7.6 m recovered, 50 m -> 4.5 m, 81 m -> 2.8 m — so a filter that
    had drifted furthest was pulled back least. That is the mechanism by which
    an 81 m gap opens and never closes.
    """
    recovered = [_recovered_m(o) for o in OFFSETS]
    for (o_small, r_small), (o_big, r_big) in zip(
        zip(OFFSETS, recovered), zip(OFFSETS[1:], recovered[1:])
    ):
        assert r_big >= r_small * 0.9, (
            f"recovery SHRANK as the error grew: {o_small:.0f} m -> "
            f"{r_small:.1f} m recovered, but {o_big:.0f} m -> {r_big:.1f} m. "
            f"A redescending de-weight cannot pull a drifted filter back."
        )


def test_below_the_gate_recovery_is_offset_independent():
    """Small innovations must be untouched by any of this.

    Below the NIS gate no inflation happens at all, so recovery is a property
    of the filter, not of the offset.  If this ever starts scaling with the
    offset, the gate has begun firing on ordinary measurements.
    """
    small, medium = _recovered_m(2.0), _recovered_m(8.0)
    assert small / 2.0 == pytest.approx(medium / 8.0, rel=0.05)
