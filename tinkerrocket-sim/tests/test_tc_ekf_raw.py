"""Raw-GNSS pieces of the standalone EKF (estimation/tc_ekf.py, gnss_raw.py) and
the raw measurement model (sensors/raw_gnss_model.py).

Decoder tests use real SkyTraq navigation-data frames from a PX1105R
(tests/data/skytraq_nav_frames.txt: broadcast satellite data only). Filter tests
use the synthetic constellation, so they need no capture and run in seconds.
"""
import math
import pathlib
import struct

import numpy as np

from tinkerrocket_sim.estimation import gnss_raw as GR
from tinkerrocket_sim.estimation.tc_ekf import (C_LIGHT, G, TcEkf, TcEkfParams, corr_inflation,
                                                ecef2lla, lla2ecef, quat2dcm, quat_from_accel_heading,
                                                spp_fix, t_e2ned)
from tinkerrocket_sim.sensors.raw_gnss_model import RawGNSSModel

DATA = pathlib.Path(__file__).parent / "data" / "skytraq_nav_frames.txt"
REF = (38.0, -122.0, 100.0)


def _frames():
    return [bytes.fromhex(line.strip()) for line in DATA.read_text().splitlines()
            if line.strip() and not line.startswith("#")]


def _dwords(b):
    return [struct.unpack_from(">I", b, 5 + 4 * i)[0] for i in range(8)]


def _store():
    st = GR.EphStore()
    for b in _frames():
        if b[0] == 0xE0:
            st.add(b[1], b[3:33])
        elif b[0] == 0xE6:
            st.add_gal(b[3], _dwords(b))
        elif b[0] == 0xE2:
            st.add_bds_d1(b[1] - 200, b[2], b[3:31])
    return st


# ---------------------------------------------------------------- decoders
def test_ephemerides_decode_for_gps_galileo_and_beidou():
    st = _store()
    assert {key[0] for key in st.eph} == {"G", "E", "C"}
    band_km = {"G": (26400, 26700), "E": (29500, 29700), "C": (27800, 42300)}
    for (sys, prn), ephs in st.eph.items():
        e = ephs[0]
        lo, hi = band_km[sys]
        assert lo < e["A"] / 1e3 < hi, (sys, prn, e["A"])
        r, dts = GR.sat_pos(e, e["toe"])
        assert abs(np.linalg.norm(r) - e["A"]) <= e["A"] * e["e"] + 2e3, (sys, prn)
        # not a millisecond: Galileo E11 really runs 5.8 ms off (its pseudorange
        # fits to 0.1 m with it); the af0 fields span tens of milliseconds
        assert abs(dts) < 0.02, (sys, prn, dts)


def test_galileo_page_with_a_flipped_bit_fails_its_crc():
    st = GR.EphStore()
    bad = bytearray(next(b for b in _frames() if b[0] == 0xE6))
    bad[10] ^= 0x04
    st.add_gal(bad[3], _dwords(bad))
    assert st.gal_crc_fail == 1 and not st.eph


def test_beidou_times_are_moved_to_gps_time():
    e = next(v[0] for k, v in _store().eph.items() if k[0] == "C")
    assert (e["toe"] - GR.BDT_GPST) % 8.0 == 0.0     # BDT toe is a multiple of 8 s


# ---------------------------------------------------------------- solutions
def _truth():
    return lla2ecef(np.array([math.radians(REF[0]), math.radians(REF[1]), REF[2]]))


def _static(model, t):
    meas, _ = model.measure(t, np.zeros(3), np.zeros(3), np.zeros(3), 1.0)
    return meas


def _gnss_only_ekf(first):
    fx = spp_fix(first)
    lla = ecef2lla(fx[0])
    ekf = TcEkf(TcEkfParams())
    ekf.init(lla, t_e2ned(lla[0], lla[1]) @ fx[1], [1.0, 0.0, 0.0, 0.0])
    ekf.freeze_imu_states()
    ekf.init_clock_from(first)
    return ekf


def test_spp_recovers_a_static_position_and_zero_velocity():
    m = RawGNSSModel(*REF, sigma_pr_m=0.05, sigma_rr_mps=0.005, atmo_sigma_m=0.0,
                     pr_wander_sigma_m=0.0, seed=1)
    fx = spp_fix(_static(m, 100.0))
    assert fx is not None
    assert np.linalg.norm(fx[0] - _truth()) < 1.0
    assert np.linalg.norm(fx[1]) < 0.05


def test_a_receiver_clock_step_is_absorbed_not_gated():
    """SkyTraq and u-blox step the receiver clock by whole milliseconds: every
    pseudorange jumps 299.8 km at once. The filter must shift its clock."""
    m = RawGNSSModel(*REF, atmo_sigma_m=0.0, pr_wander_sigma_m=0.0, seed=2)
    ekf = _gnss_only_ekf(_static(m, 0.0))
    steps, rejected = [], 0
    for t in range(1, 60):
        meas = _static(m, float(t))
        if t >= 30:
            for x in meas:
                x.pr += C_LIGHT * 1e-3
        ekf.propagate_kinematic(1.0, 0.01)
        st = ekf.update_gnss_raw(meas)
        steps.append(st.clock_jump_ms)
        rejected += st.rejected_pr + st.rejected_rr
    assert steps.count(1) == 1 and set(steps) <= {0, 1}
    assert rejected <= 2
    assert np.linalg.norm(ekf.receiver_state_ecef()[0] - _truth()) < 3.0


def test_an_inter_system_bias_is_learned_without_moving_the_position():
    """Galileo/BeiDou pseudoranges carry their own constant offset from GPS."""
    m = RawGNSSModel(*REF, atmo_sigma_m=0.0, pr_wander_sigma_m=0.0, seed=3)

    def relabelled(t):
        meas = _static(m, t)
        for x in meas:
            if x.prn % 2 == 0:
                x.sys = "E"
                x.pr += 7.0
        return meas
    ekf = _gnss_only_ekf(relabelled(0.0))
    for t in range(1, 90):
        ekf.propagate_kinematic(1.0, 0.01)
        ekf.update_gnss_raw(relabelled(float(t)))
    assert abs(ekf.isb["E"] - 7.0) < 1.0
    assert np.linalg.norm(ekf.receiver_state_ecef()[0] - _truth()) < 3.0


# ---------------------------------------------------------------- measurement model
def test_line_of_sight_acceleration_drops_satellites_and_they_relock():
    m = RawGNSSModel(*REF, lock_los_accel_mps2=30.0, reacq_mean_s=1.0, reacq_std_s=0.0, seed=4)
    zero = np.zeros(3)
    _, d0 = m.measure(0.0, zero, zero, zero, 1.0)
    assert d0["tracked"] == d0["visible"] > 4
    up_10g = np.array([0.0, 0.0, -98.07])          # NED: up is -D
    _, d1 = m.measure(0.1, zero, zero, up_10g, 11.0)
    assert 0 < len(d1["lost"]) and d1["tracked"] < d1["visible"]
    _, d_1g = RawGNSSModel(*REF, seed=4).measure(0.0, zero, zero, np.array([0.0, 0.0, -9.807]), 2.0)
    assert not d_1g["lost"]                         # 1 g along any line of sight < 30 m/s^2
    t = 0.2
    while t < 5.0:
        _, d = m.measure(t, zero, zero, zero, 1.0)
        t += 0.1
    assert d["tracked"] == d["visible"]


# ---------------------------------------------------------------- error model
def test_correlated_errors_are_inflated_by_their_sampling_rate():
    """A 15 s correlated error sampled at 20 Hz carries ~1/600 of an independent
    sample's information each epoch; sampled a minute apart, nearly all of it."""
    assert abs(corr_inflation(0.05, 15.0) - 2 * 15.0 / 0.05) / 600.0 < 0.01
    assert 1.0 <= corr_inflation(60.0, 15.0) < 1.05
    assert corr_inflation(1.0, 0.0) == 1.0


def test_skytraq_whole_hertz_doppler_moves_to_the_middle_of_its_step():
    assert GR.skytraq_doppler_fix(1605.0) == 1605.5       # truncated toward zero
    assert GR.skytraq_doppler_fix(-751.0) == -751.5
    assert GR.skytraq_doppler_fix(0.0) == 0.0
    assert GR.skytraq_doppler_fix(123.25) == 123.25       # not quantised: untouched


def test_sigmas_fall_with_cn0():
    lo, hi = 32.0, 47.0
    assert GR._cn0_sigma(GR.PR_CORR, hi) < GR._cn0_sigma(GR.PR_CORR, lo)
    assert GR._cn0_sigma(GR.PR_WHITE, hi) < GR._cn0_sigma(GR.PR_WHITE, lo)
    assert GR._cn0_sigma(GR.RR_WHITE, hi) < GR._cn0_sigma(GR.RR_WHITE, lo)


def test_sim_pseudorange_error_wanders_like_the_measured_receiver():
    """Gauss-Markov wander: ~2 m, still correlated after a second, gone after a minute."""
    m = RawGNSSModel(*REF, sigma_pr_m=0.0, atmo_sigma_m=0.0, pr_wander_sigma_m=2.0,
                     pr_wander_tau_s=15.0, clk_h0=0.0, clk_hm2=0.0, clk_bias0_m=0.0,
                     clk_drift0_mps=0.0, seed=5)
    zero = np.zeros(3)
    series = {}
    for k in range(600):
        t = 0.1 * k
        for x in m.measure(t, zero, zero, zero, 1.0)[0]:
            series.setdefault(x.prn, []).append(m.wander[x.prn][0])
    w = np.concatenate([np.array(v) for v in series.values() if len(v) == 600])
    assert 1.0 < np.std(w) < 3.5
    one = [np.corrcoef(np.array(v)[:-10], np.array(v)[10:])[0, 1] for v in series.values() if len(v) == 600]
    assert np.median(one) > 0.8                     # 1 s apart: exp(-1/15) = 0.94


# ---------------------------------------------------------------- carrier delta-range
def _vel_err(ekf, v_ned_true):
    """Against the sim's truth, which flies straight in the reference NED frame
    (3 km out, the local frame has turned half a milliradian: 2 cm/s at 50 m/s)."""
    _, v, _ = ekf.receiver_state_ecef()
    return float(np.linalg.norm(v - t_e2ned(math.radians(REF[0]), math.radians(REF[1])).T @ np.asarray(v_ned_true)))


def _run_moving(v_ned, carrier, n=60, slip_at=None, slip_prn=None, seed=6):
    """GNSS-only, 1 Hz, flying straight at ``v_ned``. The process model says
    so (1e-4 m^2/s^3): what the carrier pins is the AVERAGE velocity over each
    second, and only a dynamics model that ties it to the velocity at the end
    of the second (the truth here, an IMU in flight) turns that into a better
    velocity. Returns the filter, its carrier stats and the velocity error
    (m/s rms) over the last 40 epochs."""
    m = RawGNSSModel(*REF, atmo_sigma_m=0.0, pr_wander_sigma_m=0.0, seed=seed)
    v_ned = np.asarray(v_ned, float)
    first, _ = m.measure(0.0, np.zeros(3), v_ned, np.zeros(3), 1.0)
    ekf = _gnss_only_ekf(first)
    ekf.update_carrier(first)                       # stores the first carrier epoch
    stats, err = [], []
    for t in range(1, n):
        meas, _ = m.measure(float(t), v_ned * t, v_ned, np.zeros(3), 1.0)
        if slip_at is not None and t >= slip_at:
            for x in meas:
                if slip_prn == "all":
                    x.cr += 1.0                         # every satellite at once
                elif x.prn == slip_prn:
                    x.cr += 0.19029                     # one L1 cycle, not flagged
        ekf.propagate_kinematic(1.0, 1e-4)
        ekf.update_gnss_raw(meas)
        if carrier:
            stats.append(ekf.update_carrier(meas))
        err.append(_vel_err(ekf, v_ned))
    return ekf, stats, float(np.sqrt(np.mean(np.square(err[-40:]))))


def test_carrier_delta_range_pins_a_static_velocity_to_millimetres():
    _, st, err_c = _run_moving([0.0, 0.0, 0.0], carrier=True)
    _, _, err_d = _run_moving([0.0, 0.0, 0.0], carrier=False)
    assert sum(s.used_cr for s in st) > 50 * 5 and sum(s.rejected_cr for s in st) <= 2
    assert 0.5 < np.mean(np.concatenate([s.nis_cr for s in st])) < 1.5
    assert err_c < 0.012                             # ~8 mm/s vs ~21 mm/s on Doppler alone
    assert err_c < 0.6 * err_d


def test_carrier_delta_range_follows_a_moving_receiver():
    v = [0.0, 50.0, -20.0]                           # 50 m/s east, climbing 20 m/s
    ekf, _, err = _run_moving(v, carrier=True)
    assert err < 0.012
    r, _, _ = ekf.receiver_state_ecef()
    truth = _truth() + t_e2ned(math.radians(REF[0]), math.radians(REF[1])).T @ (np.asarray(v) * 59)
    assert np.linalg.norm(r - truth) < 5.0


def test_an_unflagged_cycle_slip_is_gated_once_and_forgotten():
    """The slip must be pinned on its own satellite: a satellite-by-satellite
    gate lets it through on the clock and then rejects every clean one."""
    prn = RawGNSSModel(*REF, seed=6).measure(0.0, np.zeros(3), np.zeros(3), np.zeros(3), 1.0)[0][0].prn
    _, st, err = _run_moving([0.0, 0.0, 0.0], carrier=True, slip_at=30, slip_prn=prn)
    hit = [k for k, s in enumerate(st, start=1) if (prn, "cr") in s.rejected_prns]
    assert 30 in hit and 31 not in hit              # the jump epoch; the pair after is clean
    assert sum(s.rejected_cr for s in st) <= 3      # the gate's 0.1 % on the rest, no pile-up
    assert err < 0.012


def test_a_carrier_jump_on_every_satellite_drops_the_epoch():
    """A shared jump is invisible satellite by satellite -- the clock would
    absorb it -- so the joint test has to drop the epoch."""
    _, st, err = _run_moving([0.0, 0.0, 0.0], carrier=True, slip_at=30, slip_prn="all")
    assert st[29].used_cr == 0 and st[29].cr_common_rejected >= 5
    assert st[30].used_cr >= 5 and st[30].cr_common_rejected == 0
    assert sum(s.cr_common_rejected > 0 for s in st) <= 2    # that epoch, and at most one chance drop
    assert err < 0.012


def _dcm_b2n(yaw_deg, pitch_deg, roll_deg):
    """Body-to-NED DCM from ZYX Euler angles, built from plain rotations."""
    y, p, r = np.radians([yaw_deg, pitch_deg, roll_deg])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return Rz @ Ry @ Rx


def test_the_pad_seed_recovers_the_attitude_from_gravity_and_heading():
    """quat_from_accel_heading is TR_Orientation's quatFromAccelHeading: pitch
    and roll from a stationary specific force, yaw from the pad heading, and
    roll left at zero within 10 deg of vertical, where it is ill-conditioned."""
    for yaw, pitch, roll in ((30.0, 40.0, -20.0), (-100.0, -60.0, 150.0), (170.0, 5.0, 90.0)):
        C = _dcm_b2n(yaw, pitch, roll)
        f_b = C.T @ np.array([0.0, 0.0, -G])         # a still accelerometer reads -gravity
        q = quat_from_accel_heading(f_b, math.radians(yaw))
        assert np.allclose(quat2dcm(q), C.T, atol=1e-9)
    f_b = _dcm_b2n(0.0, 87.0, 25.0).T @ np.array([0.0, 0.0, -G])
    q = quat_from_accel_heading(f_b, 0.0)
    assert np.allclose(quat2dcm(q), _dcm_b2n(0.0, 87.0, 0.0).T, atol=1e-9)


def test_the_pad_seed_takes_the_attitude_as_known_like_the_flight_filter():
    """TcEkf.set_quaternion is GpsInsEKF::setQuaternion, the flight computer's
    pad seed: attitude variance 1e-6 and its cross-covariances zeroed, so the
    first velocity updates cannot trade tilt for accelerometer bias."""
    import pytest
    ekf_cpp = pytest.importorskip("tinkerrocket_sim._ekf")
    f_pad = _dcm_b2n(0.0, 87.0, 0.0).T @ np.array([0.0, 0.0, -G])
    ekf = TcEkf()
    ekf.init(np.array([math.radians(REF[0]), math.radians(REF[1]), REF[2]]), [0, 0, 0], [1.0, 0, 0, 0])
    for _ in range(200):                              # builds attitude/velocity/bias cross-covariances
        ekf.propagate(f_pad, np.zeros(3), 0.001)
    assert np.abs(ekf.P[6:9, 3:6]).max() > 1e-6
    q = quat_from_accel_heading(f_pad, math.radians(30.0))
    ekf.set_quaternion(q)
    assert np.allclose(np.diag(ekf.P)[6:9], 1e-6)
    assert not np.delete(ekf.P[6:9], [6, 7, 8], axis=1).any()
    cpp = ekf_cpp.GpsInsEKF()
    cpp.set_quaternion(*q)
    assert np.allclose(cpp.get_cov_orient(), np.diag(ekf.P)[6:9], rtol=1e-6)
    assert np.allclose(cpp.get_quaternion(), ekf.q, atol=1e-6)



# ---------------------------------------------------------------- late Doppler, clock-rate offset
def _climbing(rr_lag_s, lag_in_filter, a_up=20.0, secs=10.0, rate=20.0, rate_ofs=0.0, p0_rate_ofs=0.0):
    """GNSS-only constant-acceleration filter on a receiver climbing at a_up
    from rest, 20 Hz. The Doppler is the range rate ``rr_lag_s`` before the
    epoch, built with the filter's own geometry so only the lag is under test
    (a PX1105R reports it ~0.22 s late); ``rate_ofs`` is added to every range
    rate, as the COCOM rig's carrier shift does. Returns (filter, vertical
    velocity error at the end, m/s)."""
    from tinkerrocket_sim.estimation.tc_ekf import los_predict, sat_range_accel
    m = RawGNSSModel(*REF, atmo_sigma_m=0.0, pr_wander_sigma_m=0.0, lock_los_accel_mps2=1e3, seed=9)
    acc = np.array([0.0, 0.0, -a_up])
    T = t_e2ned(math.radians(REF[0]), math.radians(REF[1]))

    def at(t):
        p, v = 0.5 * acc * t * t, acc * t
        meas, _ = m.measure(t, p, v, acc, 1.0 + a_up / G)
        r, a_e = _truth() + T.T @ p, T.T @ acc
        v_lag = T.T @ (acc * max(0.0, t - rr_lag_s))
        for x in meas:
            u = los_predict(x, r, T.T @ v)[2]
            x.rr += float(u @ (T.T @ v - v_lag)) - rr_lag_s * sat_range_accel(x, r, v_lag) + rate_ofs
        return meas
    first = at(0.0)
    fx = spp_fix(first)
    lla = ecef2lla(fx[0])
    ekf = TcEkf(TcEkfParams(rr_lag_s=rr_lag_s if lag_in_filter else 0.0, p0_rate_ofs=p0_rate_ofs))
    ekf.init(lla, t_e2ned(lla[0], lla[1]) @ fx[1], [1.0, 0.0, 0.0, 0.0])
    ekf.freeze_imu_states()
    ekf.enable_kinematic_acceleration()
    ekf.init_clock_from(first)
    n = int(secs * rate)
    for k in range(1, n + 1):
        ekf.propagate_kinematic(1.0 / rate, 0.01, 1.0)
        ekf.update_gnss_raw(at(k / rate))
    _, v_e, _ = ekf.receiver_state_ecef()
    return ekf, float((T @ v_e)[2] - acc[2] * n / rate)


def test_a_late_doppler_is_predicted_at_its_own_instant():
    """Under 2 g a Doppler 0.2 s late reads a*tau = 4 m/s slow along the line of
    sight. Predicted at t - tau from the acceleration state, it is right (to
    tau times the acceleration estimate's noise: a steady climb here)."""
    _, err_model = _climbing(0.2, True)
    _, err_naive = _climbing(0.2, False)
    assert abs(err_model) < 0.4                      # the filter's own sigma is ~0.15
    assert abs(err_naive) > 3.0


def test_a_doppler_clock_rate_offset_is_learned():
    """The COCOM rig shifts the carrier alone: every range rate reads 4.2 m/s
    off the rate the code's clock bias moves at. The offset state takes it."""
    ekf, err = _climbing(0.0, True, a_up=0.0, secs=60.0, rate=2.0, rate_ofs=4.2, p0_rate_ofs=10.0)
    assert abs(ekf.rate_ofs - 4.2) < 0.2
    assert abs(err) < 0.2


# ---------------------------------------------------------------- Earth model in the mechanization
def _coast_ins(lat_deg, v_ned, f_ned, w_ned, secs, **prm):
    """Unaided IMU propagation, body = NED (level), constant specific force and
    rate as the IMU would read them. Returns the final velocity error (NED)."""
    ekf = TcEkf(TcEkfParams(**prm))
    ekf.init(np.array([math.radians(lat_deg), 0.0, 1000.0]), v_ned, [1.0, 0.0, 0.0, 0.0])
    for _ in range(int(secs * 100)):
        ekf.propagate(f_ned, w_ned, 0.01)
    return ekf.v - np.asarray(v_ned, float)


def test_the_earth_rate_is_not_a_gyro_bias_when_modelled():
    """A still IMU at 38 N reads the Earth's rotation. The flight filter's
    mechanization takes it as vehicle rotation, tilts, and gravity leaks into
    the horizontal (tens of m/s in 5 min); with the Earth rate it stays still."""
    from tinkerrocket_sim.estimation.tc_ekf import OMGE
    lat = math.radians(38.0)
    w = OMGE * np.array([math.cos(lat), 0.0, -math.sin(lat)])
    f = np.array([0.0, 0.0, -G])
    assert np.linalg.norm(_coast_ins(38.0, [0, 0, 0], f, w, 300.0, earth_rate=True)) < 0.05
    assert np.linalg.norm(_coast_ins(38.0, [0, 0, 0], f, w, 300.0)) > 5.0
    # ... and the pad's stationary-mean gyro bias seed must not take it as bias too
    for er, expect in ((True, np.zeros(3)), (False, w)):
        ekf = TcEkf(TcEkfParams(earth_rate=er))
        ekf.init(np.array([math.radians(38.0), 0.0, 1000.0]), [0, 0, 0], [1.0, 0.0, 0.0, 0.0])
        ekf.seed_gyro_bias(w)                     # body = NED: the still gyro reads w
        assert np.allclose(ekf.wb, expect, atol=1e-12)


def test_coriolis_is_in_the_mechanization_when_modelled():
    """Climbing straight up at 1 km/s on the equator, the vehicle must push
    east with 2 Omega v = 0.15 m/s^2 to stay on its ground track; the IMU reads
    that, and a mechanization without Coriolis turns it into 8.75 m/s of east
    velocity in a minute -- 10.0 with the Earth rate tilting the platform too."""
    from tinkerrocket_sim.estimation.tc_ekf import OMGE
    v = np.array([0.0, 0.0, -1000.0])
    w = OMGE * np.array([1.0, 0.0, 0.0])
    f = np.array([0.0, 0.0, -G]) + np.cross(2.0 * w, v)
    assert abs(_coast_ins(0.0, v, f, w, 60.0, earth_rate=True)[1]) < 0.05
    assert abs(_coast_ins(0.0, v, f, w, 60.0)[1] - 10.04) < 0.3


def test_wgs84_normal_gravity_matches_the_published_values():
    """Somigliana on the ellipsoid (NIMA TR8350.2): 9.7803253359 on the equator,
    9.8321849378 at the pole, 9.8061978 at 45 deg; the free-air gradient on the
    equator is -2 gamma/a (1 + f + m) = -3.0877e-6 s^-2, so 80 km up is 2.5 % lighter."""
    from tinkerrocket_sim.estimation.tc_ekf import gravity_wgs84
    assert abs(gravity_wgs84(0.0, 0.0)[2] - 9.7803253359) < 1e-9
    assert abs(gravity_wgs84(math.pi / 2, 0.0)[2] - 9.8321849378) < 1e-8
    assert abs(gravity_wgs84(math.pi / 4, 0.0)[2] - 9.8061978) < 1e-6
    grad = (gravity_wgs84(0.0, 100.0)[2] - gravity_wgs84(0.0, 0.0)[2]) / 100.0
    assert abs(grad + 3.0877e-6) < 0.001e-6
    assert 0.974 < gravity_wgs84(0.0, 80_000.0)[2] / gravity_wgs84(0.0, 0.0)[2] < 0.976


def test_wgs84_gravity_follows_a_free_fall_from_80_km():
    """In vacuum the accelerometer reads zero. Gravity at 80 km is 2.5 % weaker
    than on the pad: a constant-G mechanization falls ~430 m too far in a minute
    (the WGS84 model's few metres are the 100 Hz Euler step)."""
    from tinkerrocket_sim.estimation.tc_ekf import gravity_wgs84
    lat, h0, secs = 0.3, 80_000.0, 60.0
    h, v = h0, 0.0
    for _ in range(int(secs * 1000)):              # the truth, finely integrated
        v -= gravity_wgs84(lat, h)[2] * 1e-3
        h += v * 1e-3
    out = {}
    for model in ("wgs84", "const"):
        ekf = TcEkf(TcEkfParams(gravity_model=model))
        ekf.init(np.array([lat, 0.0, h0]), [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        for _ in range(int(secs * 100)):
            ekf.propagate(np.zeros(3), np.zeros(3), 0.01)
        out[model] = ekf.lla[2] - h
    assert abs(out["wgs84"]) < 5.0
    assert out["const"] < -300.0


def test_pad_levelling_uses_the_mechanization_gravity():
    """Levelled against the gravity it propagates with, a still IMU leaves the
    accelerometer bias alone; the flight filter's constant G on the equator
    (WGS84 9.780 m/s^2) parks 0.027 m/s^2 of it there instead."""
    from tinkerrocket_sim.estimation.tc_ekf import gravity_wgs84
    lat, h = 0.0, 1200.0
    f = np.array([0.0, 0.0, -gravity_wgs84(lat, h)[2]])        # body = NED, level and still
    out = {}
    for model in ("wgs84", "const"):
        ekf = TcEkf(TcEkfParams(gravity_model=model))
        ekf.init(np.array([lat, 0.0, h]), [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        for _ in range(3000):
            ekf.propagate(f, np.zeros(3), 0.01)
            ekf.update_accel_level(f)
        out[model] = ekf.ab[2]
    assert abs(out["wgs84"]) < 0.003
    assert abs(out["const"] - (G - gravity_wgs84(lat, h)[2])) < 0.005


def test_the_heading_gate_keeps_tilt_and_drops_rotation_about_the_thrust():
    """Nose up under 3 g, a GNSS update may correct the tilt (rotation about the
    body's Y and Z, across the thrust) but not the rotation about the thrust
    axis, which it cannot see. The flight filter's cos^4 pitch gate drops both."""
    ekf = TcEkf(TcEkfParams(gnss_att_gate="heading"))
    ekf.init(np.array([0.3, 0.0, 100.0]), [0, 0, 0], quat_from_accel_heading([G, 0.0, 0.0], 0.0))
    ekf.f_b = np.array([3.0 * G, 0.0, 0.0])                    # thrust along the nose (body X)
    A = ekf._att_gate()
    assert np.allclose(A @ [1.0, 0.0, 0.0], 0.0)
    assert np.allclose(A @ [0.0, 1.0, 0.0], [0.0, 1.0, 0.0]) and np.allclose(A @ [0.0, 0.0, 1.0], [0.0, 0.0, 1.0])
    ekf.p.gnss_att_gate = "cos4"
    assert np.abs(ekf._att_gate()).max() < 1e-6
