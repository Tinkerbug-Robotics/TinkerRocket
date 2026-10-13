"""#1600 -- the raw-GNSS (tightly coupled) filter's bindings, tinkerrocket_sim._rawgnss.

The gnss repo's bench scorer (sim/tc_ekf_cocom.py, modes ins-cpp and ins-cpp64) drives the flight
computer's TR_RawGnssEKF and RawMeasBuilder through these bindings to show they reproduce the prototype
they were ported from. This pins the bindings themselves: the parameters the LC86G flies with, a filter
that starts its clock from the first epoch and converges on a synthetic sky, and the parser and builder
objects the scorer feeds. The filter's behaviour is tested in C++ (tests_cpp/test_raw_gnss_ekf.cpp).
"""
import math

import numpy as np
import pytest

rg = pytest.importorskip("tinkerrocket_sim._rawgnss")

C = 299792458.0
OMEGA_E = 7.2921151467e-5
LAT, LON, H = math.radians(32.99), math.radians(-106.97), 1400.0
E2 = 0.0066943799901


def ecef(lat, lon, h):
    n = 6378137.0 / math.sqrt(1.0 - E2 * math.sin(lat) ** 2)
    return np.array([(n + h) * math.cos(lat) * math.cos(lon), (n + h) * math.cos(lat) * math.sin(lon),
                     (n * (1.0 - E2) + h) * math.sin(lat)])


def sky(r):
    """Eight satellites 26,560 km from the Earth's centre, spread over the sky."""
    sl, cl, so, co = math.sin(LAT), math.cos(LAT), math.sin(LON), math.cos(LON)
    e, n, u = np.array([-so, co, 0.0]), np.array([-sl * co, -sl * so, cl]), np.array([cl * co, cl * so, sl])
    out = []
    for k, (sys_, prn, az, el) in enumerate([(rg.SYS_GPS, 2, 0, 70), (rg.SYS_GPS, 9, 70, 30), (rg.SYS_GPS, 17, 150, 25),
                                             (rg.SYS_GPS, 21, 220, 55), (rg.SYS_GPS, 29, 300, 35),
                                             (rg.SYS_GAL, 5, 40, 20), (rg.SYS_GAL, 19, 180, 45),
                                             (rg.SYS_BDS, 30, 260, 60)]):
        a, b = math.radians(az), math.radians(el)
        d = e * math.cos(b) * math.sin(a) + n * math.cos(b) * math.cos(a) + u * math.sin(b)
        ru = r @ d
        rho = -ru + math.sqrt(ru * ru - r @ r + 26.56e6 ** 2)
        pos = r + rho * d
        vel = np.cross(pos, [0.3, -0.2, 1.0])
        out.append((sys_, prn, pos, 3870.0 * vel / np.linalg.norm(vel)))
    return out


def measure(sats, r, bias, isb):
    """Still receiver at r: pseudorange with the Earth's rotation over the travel time, the clock's bias and
    the system's offset; range rate with no clock drift."""
    out = []
    for sys_, prn, pos, vel in sats:
        rho = np.linalg.norm(pos - r)
        for _ in range(2):
            th = OMEGA_E * rho / C
            c, s = math.cos(th), math.sin(th)
            x = np.array([c * pos[0] + s * pos[1], -s * pos[0] + c * pos[1], pos[2]])
            rho = np.linalg.norm(x - r)
        v = np.array([c * vel[0] + s * vel[1], -s * vel[0] + c * vel[1], vel[2]])
        rate = float((x - r) @ v / rho)
        out.append(rg.RawMeas(sys_, prn, list(pos), list(vel), [0.0, 0.0, 0.0], rho + bias + isb.get(sys_, 0.0),
                              rate, 1.0, 0.0, 15.0, 0.05, 0.1))
    return out


def test_defaults_are_the_lc86g_bench_settings():
    p = rg.RawGnssEKFParams()
    assert p.clk_drift_psd == 100.0          # the LC86G steers its clock to its own fix
    assert p.rr_lag_s == pytest.approx(0.0555)
    assert p.gate_chi2 == pytest.approx(10.83)
    assert p.reject_persist == 3
    b = rg.BuilderConfig()
    assert (b.pr_corr_scale, b.relock_sigma_m, b.relock_gap_s, b.rr_dyn_sigma, b.rr_relock_sigma) == \
        (0.5, 40.0, 2.5, 0.5, 3.0)


@pytest.mark.parametrize("cls", ["RawGnssEKF32", "RawGnssEKF64"])
def test_starts_its_clock_and_converges_on_a_still_receiver(cls):
    p = rg.RawGnssEKFParams()
    p.rr_lag_s = 0.0
    f = getattr(rg, cls)(p)
    truth = ecef(LAT, LON, H)
    sats = sky(truth)
    f.init([LAT + 10.0 / 6.37e6, LON, H - 8.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], 0.0)
    f.set_quaternion([1.0, 0.0, 0.0, 0.0])
    assert not f.clock_ready
    isb = {rg.SYS_GAL: 15.0, rg.SYS_BDS: -20.0}
    for _ in range(15):
        for _ in range(200):
            f.propagate([0.0, 0.0, -9.807], [0.0, 0.0, 0.0], 0.005)
        st = f.update_gnss_raw(measure(sats, truth, 777.0, isb))
        assert st["used_pr"] == 8 and st["rejected_pr"] == 0
    assert f.clock_ready
    r, v = f.receiver_ecef()
    assert np.linalg.norm(np.array(r) - truth) < 0.5
    assert np.linalg.norm(v) < 0.05
    assert f.clk_bias == pytest.approx(777.0, abs=0.5)
    assert f.isb_gal == pytest.approx(15.0, abs=0.5)
    assert f.isb_bds == pytest.approx(-20.0, abs=0.5)
    P = f.P
    assert P.shape == (19, 19)
    assert np.allclose(P, P.T)
    assert f.t == pytest.approx(15.0)


def test_parser_and_builder_objects():
    fe = rg.Lc86FrontEnd()
    assert fe.feed_payload(b"\x00\x00\x00") == []          # not an MSM7 or an ephemeris
    assert fe.store.size() == 0
    ep = rg.RawEpoch(345600.0, [(rg.SYS_GPS, 5, 2.2e7, -310.0, 41.0), (rg.SYS_GAL, 7, None, 12.0, 35.0)])
    assert ep.tow_s == 345600.0
    assert ep.obs[1][2] is None
    b = rg.RawMeasBuilder()
    assert b.build(ep, fe.store, None) == []                # no ephemeris yet: nothing to place
    assert b.relock_age(rg.SYS_GPS, 5, 345601.0) == pytest.approx(1.0)
