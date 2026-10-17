"""#1600 -- the raw-GNSS (tightly coupled) filter's bindings, tinkerrocket_sim._rawgnss.

The gnss repo's bench scorer (sim/tc_ekf_cocom.py, modes ins-cpp and ins-cpp64) drives the flight
computer's TR_RawGnssEKF and RawMeasBuilder through these bindings to show they reproduce the prototype
they were ported from. This pins the bindings themselves: the parameters the LC86G flies with, a filter
that starts its clock from the first epoch and converges on a synthetic sky, and the parser and builder
objects the scorer feeds; and the heading from thrust (HeadingFit, its Doppler velocity, the filter's
hand-off calls), which the gnss repo's heading runs (sim/heading_seed_sim.py --fit-cpp) fly beside the
prototype. And the receivers' profiles and the carrier range rate -- the observations' phase, the
measurements' rate, the filter's carrier update whole and in its parts -- for a log replayed through the flight
code with the carrier observed or fused. The behaviour is tested in C++ (tests_cpp/test_raw_gnss_ekf.cpp,
test_raw_gnss_carrier.cpp, test_receiver_profile.cpp, test_heading_fit.cpp).
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


def test_the_heading_fit_finds_a_turned_seed():
    # a level body pushed north at 8 m/s^2, its seed turned 40 deg about the vertical: the fit turns it back
    err = math.radians(40.0)
    f = rg.HeadingFit()
    f.start([math.cos(err / 2), 0.0, 0.0, math.sin(err / 2)], [0.0, 0.0, 0.0], 0.0)
    assert f.started and f.epochs == 0
    for k in range(1, 501):
        f.imu([8.0, 0.0, -9.807], [0.0, 0.0, 0.0], 0.01)
        if k % 100 == 0:
            assert f.gnss(f.t, [8.0 * f.t, 0.0], 0.01)
    r = f.solve()
    assert r["ok"] and r["n"] == 5 and r["n_cut"] == 0
    assert r["psi"] == pytest.approx(-err, abs=math.radians(0.3))
    assert r["sigma"] is not None and r["sigma"] < math.radians(1.0)
    assert r["rival"] is None
    g = f.solve_grid()
    assert g["psi"] == pytest.approx(r["psi"], abs=1e-9) and g["sigma"] == r["sigma"]
    q = f.attitude(r["psi"])                           # turned back: the truth's (level, nose north)
    assert abs(q[0]) == pytest.approx(1.0, abs=1e-4)
    assert rg.heading_between(q, [1.0, 0.0, 0.0, 0.0]) == pytest.approx(0.0, abs=math.radians(0.3))


def test_the_doppler_velocity_of_a_moving_receiver():
    truth = ecef(LAT, LON, H)
    sl, cl, so, co = math.sin(LAT), math.cos(LAT), math.sin(LON), math.cos(LON)
    ned2ecef = np.array([[-sl * co, -so, -cl * co], [-sl * so, co, -cl * so], [cl, 0.0, -sl]])
    v_ned = np.array([12.0, -30.0, -150.0])
    v = ned2ecef @ v_ned
    meas = []
    for m in measure(sky(truth), truth, 0.0, {}):
        u = np.array(m.sat_pos) - truth
        u /= np.linalg.norm(u)
        meas.append(rg.RawMeas(m.sys, m.prn, list(m.sat_pos), list(m.sat_vel), [0.0, 0.0, 0.0], m.pr,
                               m.rr - float(u @ v) + 40.0, 1.0, 0.0, 15.0, 0.05, 0.1))     # and a clock drift
    out = rg.doppler_velocity_ned(meas, list(truth), LAT, LON)
    assert out is not None
    vn, var_h, used = out
    assert used == 8
    assert np.allclose(vn, v_ned, atol=0.02)
    assert 0.0 < var_h < 0.01
    assert rg.doppler_velocity_ned(meas[:4], list(truth), LAT, LON) is None     # under five: none


def test_the_hand_off_calls():
    f = rg.RawGnssEKF32(rg.RawGnssEKFParams())
    f.init([LAT, LON, H], [0.0, 0.0, 0.0], [0.7071068, 0.0, 0.7071068, 0.0], 0.0)    # nose up
    q0 = f.q
    f.turn_heading(0.5)
    assert rg.heading_between(f.q, q0) == pytest.approx(0.5, abs=1e-5)
    f.set_heading_sigma(0.3)
    assert f.att_sigma_ned()[2] == pytest.approx(0.3, rel=1e-4)
    f.set_att_gate_vertical(True)
    assert f.params.att_gate_vertical
    Pb = np.diag([1e-4] * 3 + [1e-6] * 3)
    f.set_biases([0.01, -0.02, 0.03], [1e-3, 0.0, -1e-3], Pb)
    assert np.allclose(f.ab, [0.01, -0.02, 0.03], atol=1e-7)
    assert np.allclose(f.bias_covariance(), Pb, atol=1e-10)
    f.release_velocity(3.0)
    assert np.all(np.diag(f.P)[3:6] >= 9.0 - 1e-5)
    f.clear_rejects()


# ---- the receivers' profiles and the carrier range rate

BUILDER_FIELDS = ("el_mask_deg", "troposphere", "doppler_step_hz", "pr_corr_scale", "rr_scale", "relock_sigma_m",
                  "relock_tau_s", "relock_gap_s", "rr_dyn_sigma", "rr_relock_sigma", "sat_refresh_s", "pr_white_a",
                  "pr_white_b", "pr_corr_a", "pr_corr_b", "pr_tau_s", "rr_white_a", "rr_white_b", "rr_tau_s",
                  "carrier", "cr_min_lock_ms", "epoch_period_s", "cr_max_interval_s", "cr_sigma_floor_mps")
EKF_FIELDS = ("clk_bias_psd", "clk_drift_psd", "isb_psd", "clock_step_m", "clock_step_spread_m", "drift_step_mps",
              "drift_step_spread_mps", "rr_lag_s", "p0_clk_bias", "p0_clk_drift", "p0_isb", "cr_mode",
              "cr_sigma_per_rev_s", "cr_common_sigma_mps", "cr_screen_k", "cr_screen_floor_mps", "cr_half_cycle",
              "cr_carrier_hz", "cr_half_cycle_unflagged")


def test_the_lc86g_profile_is_the_bound_defaults():
    p = rg.lc86g_profile()
    assert p.name == "LC86G" and p.measured and p.type == rg.RX_LC86G
    assert p.cr_mode == rg.CARRIER_OBSERVE and p.cr_half_cycle == rg.HALF_CYCLE_REJECT
    assert [s.cn0_min_cr for s in p.sig] == [32.5, 35.0, 32.5, 32.5]
    assert [s.half_cycle_unflagged for s in p.sig] == [True, False, True, True]
    assert p.sig[rg.SYS_GPS].phase_sigma(40.0) == pytest.approx(math.sqrt(0.0045 * 0.0014), rel=1e-5)
    # applied, the builder's and the filter's defaults: what they flew with, and the carrier's
    b0, b1 = rg.BuilderConfig(), rg.BuilderConfig()
    rg.apply_profile(p, b1)
    for f in BUILDER_FIELDS:
        assert getattr(b1, f) == getattr(b0, f), f
    assert [s.carrier_hz for s in b1.sig] == [s.carrier_hz for s in b0.sig]
    e0, e1 = rg.RawGnssEKFParams(), rg.RawGnssEKFParams()
    rg.apply_profile(p, e1)
    for f in EKF_FIELDS:
        assert getattr(e1, f) == getattr(e0, f), f
    # the others: the LC86G's numbers, UNMEASURED
    for make, rx in ((rg.neo_m8t_profile, rg.RX_NEO_M8T), (rg.px1105r_profile, rg.RX_PX1105R),
                     (rg.mosaic_g5_profile, rg.RX_MOSAIC_G5), (rg.generic_profile, rg.RX_GENERIC)):
        q = make()
        assert not q.measured and q.type == rx and rg.profile_for(rx).name == q.name
        assert q.cr_sigma_floor_mps == p.cr_sigma_floor_mps and q.rr_lag_s == p.rr_lag_s
    # the carrier off: the builder builds no rates
    p.cr_mode = rg.CARRIER_OFF
    rg.apply_profile(p, b1)
    assert not b1.carrier


def test_an_epoch_carries_its_carrier_phase():
    ep = rg.RawEpoch(345600.0, [(rg.SYS_GPS, 5, 2.2e7, -310.0, 41.0, 60000, 2.2e7 + 3.5, False),
                                (rg.SYS_GAL, 7, 2.3e7, 12.0, 35.0),
                                (rg.SYS_BDS, 9, 2.4e7, 3.0, 38.0, 1200, 2.4e7 - 1.0, True)])
    assert ep.obs_carrier == [(2.2e7 + 3.5, False), (None, False), (2.4e7 - 1.0, True)]
    assert len(ep.obs[0]) == 6                              # the observations' old shape


def sagnac(pos, vel, r):
    """Range and still-receiver range rate, the satellite turned by the Earth's rotation over the travel."""
    rho = np.linalg.norm(pos - r)
    for _ in range(2):
        th = OMEGA_E * rho / C
        c, s = math.cos(th), math.sin(th)
        x = np.array([c * pos[0] + s * pos[1], -s * pos[0] + c * pos[1], pos[2]])
        rho = np.linalg.norm(x - r)
    v = np.array([c * vel[0] + s * vel[1], -s * vel[0] + c * vel[1], vel[2]])
    return rho, float((x - r) @ v / rho)


def carrier_epochs(n, r, rng):
    """A still receiver's epochs a second apart: its satellites moving, its clock drifting, each satellite's
    carrier phase exact to 3 mm beside a 1 m pseudorange and a 0.1 m/s Doppler, a rate from the second epoch."""
    sats = sky(r)
    clk0, drift = 777.0, -1.2
    isb = {rg.SYS_GAL: 15.0, rg.SYS_BDS: -20.0}
    out, prev = [], None
    for k in range(n):
        t = float(k + 1)
        clk = clk0 + drift * t
        meas, ends = [], []
        for sys_, prn, pos0, vel in sats:
            pos = pos0 + vel * t
            rho, rate = sagnac(pos, vel, r)
            lam = C / (1561.098e6 if sys_ == rg.SYS_BDS else 1575.42e6)
            phase = rho + clk + lam * (100 + prn) + 0.003 * rng.standard_normal()
            kw = {}
            if prev is not None:
                ph0, pos_prev = prev[len(meas)]
                kw = dict(cr_dphi_m=phase - ph0, cr_dt=1.0, sat_pos_prev=list(pos_prev), sigma_cr=0.008)
            meas.append(rg.RawMeas(sys_, prn, list(pos), list(vel), [0.0, 0.0, 0.0],
                                   rho + clk + isb.get(sys_, 0.0) + rng.standard_normal(),
                                   rate + drift + 0.1 * rng.standard_normal(), 1.0, 0.0, 15.0, 0.1, 0.1, **kw))
            ends.append((phase, pos))
        prev = ends
        out.append(meas)
    return out


def carrier_filter(mode):
    p = rg.RawGnssEKFParams()
    p.rr_lag_s = 0.0
    p.cr_mode = mode
    p.p0_abias, p.p0_wbias = 0.05, 0.0005                  # the IMU's biases as the pad leaves them
    f = rg.RawGnssEKF32(p)
    f.init([LAT + 5.0 / 6.37e6, LON, H - 3.0], [0.2, -0.1, 0.1], [1.0, 0.0, 0.0, 0.0], 0.0)
    f.set_quaternion([1.0, 0.0, 0.0, 0.0])
    return f


def second(f):
    for _ in range(200):
        f.propagate([0.0, 0.0, -9.807], [0.0, 0.0, 0.0], 0.005)


def test_the_carrier_observed_changes_nothing_and_fused_holds_the_velocity():
    epochs = carrier_epochs(25, ecef(LAT, LON, H), np.random.default_rng(1600))
    off, obs, fuse = (carrier_filter(m) for m in (rg.CARRIER_OFF, rg.CARRIER_OBSERVE, rg.CARRIER_FUSE))
    err = {id(obs): [], id(fuse): []}
    for k, meas in enumerate(epochs):
        for f in (off, obs, fuse):
            second(f)
        st_off, st_obs, st_fuse = (f.update_gnss_raw(meas) for f in (off, obs, fuse))
        assert np.array_equal(off.P, obs.P) and off.v == obs.v and off.q == obs.q   # observed: untouched
        assert st_off["cr_built"] == 0
        if k > 0:
            assert st_obs["cr_built"] == 8 and st_obs["cr_in"] == 8 and st_obs["used_cr"] == 0
            assert st_fuse["used_cr"] == 8
            assert [c["verdict"] for c in fuse.carrier_info()] == ["in"] * 8
            # the common rate: the clock's drift (the integers do not move)
            assert st_obs["cr_common_mps"] == pytest.approx(-1.2, abs=0.05)
        if k >= 10:
            for f in (obs, fuse):
                err[id(f)].append(np.linalg.norm(f.v))
    e_obs, e_fuse = np.sqrt(np.mean(np.square(err[id(obs)]))), np.sqrt(np.mean(np.square(err[id(fuse)])))
    assert e_fuse < 0.5 * e_obs and e_fuse < 0.02


def test_the_carrier_in_parts_is_the_epoch_at_once():
    epochs = carrier_epochs(6, ecef(LAT, LON, H), np.random.default_rng(7))
    whole, parts = carrier_filter(rg.CARRIER_FUSE), carrier_filter(rg.CARRIER_FUSE)
    for meas in epochs:
        second(whole)
        second(parts)
        sw = whole.update_gnss_raw(meas, 0.0)
        st = rg.RawGnssStats()
        assert parts.begin_gnss_raw(meas, 0.0, st)
        parts.update_gnss_raw_part(meas, 0, 4, 0.0, st)
        parts.update_gnss_raw_part(meas, 4, 8, 0.0, st)
        n = parts.begin_carrier(meas, 0.0, st)
        if n > 0:
            parts.fuse_carrier(meas, st)
        parts.end_carrier(st)
        # each part takes the epoch back with the velocity the parts before it left: to rounding
        d = st.as_dict()
        for key in ("used_pr", "used_rr", "used_cr", "cr_built", "cr_in", "cr_other", "cr_unmatched"):
            assert d[key] == sw[key], key
        assert d["cr_common_mps"] == pytest.approx(sw["cr_common_mps"], abs=1e-4)
        assert np.allclose(parts.P, whole.P, rtol=1e-4, atol=1e-9)
        assert np.allclose(parts.v, whole.v, atol=1e-5)
    assert sw["used_cr"] == 8


def test_the_tilt_floor_and_the_cos4_switch():
    f = carrier_filter(rg.CARRIER_OBSERVE)
    f.set_tilt_sigma_floor(math.radians(2.0))
    s = f.att_sigma_ned()
    assert s[0] >= math.radians(2.0) * (1 - 1e-4) and s[1] >= math.radians(2.0) * (1 - 1e-4)
    assert f.params.att_gate_cos4
    f.set_att_gate_cos4(False)
    assert not f.params.att_gate_cos4
