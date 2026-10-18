"""tools/raw_replay/sim_roll.py: the spinning body the simulated flights take (sim_events.py --roll), pinned to the
conventions the firmware's wind-up correction (TR_RawGnssEKF/RawGnssWindup.h) and the spin rig share."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "tools", "raw_replay"))
import sim_roll  # noqa: E402

LAM = sim_roll.LAMBDA_L1


def dcm(q):
    """body -> NED of a scalar-first quaternion (rows)."""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat_from_dcm(R):
    w = 0.5 * math.sqrt(max(1.0 + R[0, 0] + R[1, 1] + R[2, 2], 1e-12))
    return np.array([w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)])


def test_the_rigs_clockwise_turn_is_plus_one_cycle_range_like():
    # Nose up (body +x up), a clockwise turn seen from above is a negative turn about the boresight: one turn at
    # 1 rev/s for 1 s winds the phase range by +k cycles (the rig: +0.991 / +0.994).
    k = 0.99
    p = -sim_roll.TWO_PI                      # right-handed about the nose (up): clockwise from above
    cycles = sim_roll.roll_rate_windup(LAM, k, p) * 1.0 / LAM
    assert cycles == pytest.approx(+k)


def test_roll_alone_winds_every_satellite_the_same_at_any_angle():
    rng = np.random.default_rng(1)
    e = rng.normal(size=(500, 3))
    e /= np.linalg.norm(e, axis=1, keepdims=True)
    e = e[e[:, 0] > -0.85]                      # off the null behind the antenna
    p = 10 * sim_roll.TWO_PI
    w = np.tile([p, 0.0, 0.0], (len(e), 1))
    r = sim_roll.windup_rate(LAM, 0.99, w, e)
    assert np.allclose(r, -0.99 * LAM * p / sim_roll.TWO_PI)
    assert r[0] == pytest.approx(-1.8838, abs=1e-3)     # 0.19 m/s per rev/s on L1


def test_the_rate_across_the_boresight_is_each_satellites_own_and_bounded():
    rng = np.random.default_rng(2)
    for _ in range(200):
        e = rng.normal(size=3)
        e /= np.linalg.norm(e)
        if e[0] < -0.9:
            continue
        wp = rng.normal(size=3)
        wp[0] = 0.0
        theta = math.acos(e[0])
        bound = LAM / sim_roll.TWO_PI * np.linalg.norm(wp) * math.tan(theta / 2.0)
        assert abs(sim_roll.windup_rate(LAM, 1.0, wp, e)) <= bound * (1 + 1e-9) + 1e-12


def test_wu_against_rtklibs_dipole_for_a_turning_body():
    # RTKLIB's windupcorr (Wu et al. 1993), the receiver's part: with k the unit vector satellite -> receiver and the
    # antenna's axes a1, a2 (a1 x a2 the boresight: body y, z), the effective dipole d = a1 - k (k.a1) + k x a2; the
    # wind-up is d's turn about k, in cycles (range-like). Turned along a path, the rate form's integral matches it.
    rng = np.random.default_rng(3)
    compared = 0
    for trial in range(6):
        e_ned = rng.normal(size=3)
        e_ned /= np.linalg.norm(e_ned)
        k = -e_ned
        ref = np.cross(k, rng.normal(size=3))
        ref /= np.linalg.norm(ref)
        R = np.eye(3)
        w_b = rng.normal(size=3) * 2.0
        w_b[0] += 6.0
        dt = 2e-4
        ang, integral = [], 0.0
        for i in range(6000):
            e_b = R.T @ e_ned
            if 1.0 + e_b[0] < 0.2:             # near the null behind the antenna: no comparison there
                break
            d = R[:, 1] - k * (k @ R[:, 1]) + np.cross(k, R[:, 2])
            ang.append(math.atan2(np.cross(ref, d) @ k, ref @ d))
            integral += sim_roll.windup_rate(1.0, 1.0, w_b, e_b) * dt
            th = w_b * dt
            a = np.linalg.norm(th)
            K = np.array([[0, -th[2], th[1]], [th[2], 0, -th[0]], [-th[1], th[0], 0]]) / a
            R = R @ (np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K)
        if len(ang) < 1000:
            continue
        compared += 1
        a_un = np.unwrap(np.array(ang))
        # (the integral runs one step past the last angle: 2e-4 cycles at most)
        assert integral == pytest.approx((a_un[-1] - a_un[0]) / sim_roll.TWO_PI, abs=0.01)
        assert abs(integral) > 0.1                 # the path wound it by a tenth of a cycle or more
    assert compared >= 2


def test_the_rolled_bodys_rate_is_its_attitudes():
    # q' = q (x) qx(phi): the forward difference of q' is Rx(phi_mid)^T w + phi_dot x
    t = np.arange(0.0, 0.2, 0.001)
    w = np.array([0.3, -0.2, 0.1])
    q = [np.array([1.0, 0.0, 0.0, 0.0])]
    for _ in t[1:]:
        th = w * 0.001
        a = np.linalg.norm(th)
        dq = np.concatenate([[math.cos(a / 2)], math.sin(a / 2) * th / a])
        q.append(sim_roll.qmul(q[-1], dq))
    q = np.array(q)
    rate = np.full_like(t, 15 * sim_roll.TWO_PI)
    phi = sim_roll.integrate(t, rate)
    q1 = sim_roll.roll_quat(q, phi)
    for i in (10, 100, 150):
        R0, R1 = dcm(q1[i]), dcm(q1[i + 1])
        dC = R0.T @ R1
        wf = np.array([dC[2, 1] - dC[1, 2], dC[0, 2] - dC[2, 0], dC[1, 0] - dC[0, 1]]) / 2.0
        ang = math.asin(min(1.0, np.linalg.norm(wf)))
        wf = wf / np.linalg.norm(wf) * ang / 0.001
        phi_mid = 0.5 * (phi[i] + phi[i + 1])
        expect = sim_roll.turn(sim_roll.rx(np.array([phi_mid])), w[None, :])[0]
        expect[0] += (phi[i + 1] - phi[i]) / 0.001
        assert np.allclose(wf, expect, atol=2e-3 * np.linalg.norm(expect))


def test_pure_roll_reaches_the_chips_rail_at_6487_dps():
    rail = math.radians(sim_roll.GYRO_RAIL_DPS)
    under = np.array([[math.radians(6400.0), 0.0, 0.0]])
    over = np.array([[math.radians(7200.0), 0.0, 0.0]])
    w, c = sim_roll.chip_clip(under, rail)
    assert not c[0] and np.allclose(w, under)
    w, c = sim_roll.chip_clip(over, rail)
    assert c[0]
    # both axes clip alike: the roll reads the rail's sqrt 2, still about the nose
    assert math.degrees(w[0, 0]) == pytest.approx(sim_roll.GYRO_RAIL_DPS * math.sqrt(2.0))
    assert abs(w[0, 1]) < 1e-9 and abs(w[0, 2]) < 1e-9
    # with a rate across the nose one axis clips first: the excess reads as a false rate across it
    w, c = sim_roll.chip_clip(np.array([[math.radians(6400.0), math.radians(300.0), 0.0]]), rail)
    assert c[0] and abs(w[0, 1] - math.radians(300.0)) > math.radians(10.0)


def test_profiles():
    ph = dict(thrust_start_s=0.0, burnout_s=1.3, apogee_s=8.5, deploy_s=9.5)
    t = np.array([-1.0, 0.05, 0.2, 1.3, 5.0, 9.5, 10.5])
    r = sim_roll.profile_rate(t, sim_roll.parse_profile("const:10"), ph) / sim_roll.TWO_PI
    assert r == pytest.approx([0.0, 5.0, 10.0, 10.0, 10.0, 10.0, 10.0 * math.exp(-1.0)])
    r = sim_roll.profile_rate(t, sim_roll.parse_profile("spinup:10"), ph) / sim_roll.TWO_PI
    assert r[1] == pytest.approx(10.0 * 0.05 / 1.3) and r[3] == pytest.approx(10.0) and r[4] == pytest.approx(10.0)
    rv = sim_roll.parse_profile("rev:10")
    mid = 0.5 * (1.3 + 8.5)
    r = sim_roll.profile_rate(np.array([mid - 0.1, mid + 0.5, mid + 1.1]), rv, ph) / sim_roll.TWO_PI
    assert r == pytest.approx([10.0, 0.0, -10.0])
    assert sim_roll.parse_profile("none") is None


def test_a_roll_axis_tilted_off_the_nose_cones_the_nose_about_it():
    # --roll-axis-tilt-deg: the roll about an axis tilt deg off the nose, toward body +y. Untilted, the nose itself
    # (rot is rx, exactly). Tilted 2 deg, the nose turned by it stays 2 deg off the axis and sweeps a cone, so its own
    # rate is p sin(tilt) across it; the quaternion turns the attitude as the matrix does.
    phi = np.linspace(0.0, sim_roll.TWO_PI, 721)
    assert sim_roll.roll_axis(0.0) is None
    assert np.array_equal(sim_roll.rot(phi, None), sim_roll.rx(phi))
    np.testing.assert_allclose(sim_roll.rot(phi, np.array([1.0, 0.0, 0.0])), sim_roll.rx(phi), atol=1e-15)
    tilt = 2.0
    a = sim_roll.roll_axis(tilt)
    assert a == pytest.approx([math.cos(math.radians(tilt)), math.sin(math.radians(tilt)), 0.0])
    R = sim_roll.rot(phi, a)
    np.testing.assert_allclose(np.einsum("nij,nkj->nik", R, R), np.tile(np.eye(3), (len(phi), 1, 1)), atol=1e-14)
    nose = R[:, :, 0]                                   # +x turned about a
    np.testing.assert_allclose(nose @ a, math.cos(math.radians(tilt)), atol=1e-15)
    rate = np.linalg.norm(np.gradient(nose, phi, axis=0), axis=1)[1:-1]   # per radian of roll
    np.testing.assert_allclose(rate, math.sin(math.radians(tilt)), rtol=1e-3)
    assert np.max(np.linalg.norm(nose - nose[0], axis=1)) == pytest.approx(2.0 * math.sin(math.radians(tilt)),
                                                                            rel=1e-3)
    q0 = np.array([0.9, 0.1, -0.3, 0.2])
    q0 /= np.linalg.norm(q0)
    q1 = sim_roll.roll_quat(np.tile(q0, (len(phi), 1)), phi, a)
    for i in (0, 200, 500):
        np.testing.assert_allclose(dcm(q1[i]), dcm(q0) @ R[i], atol=1e-12)
