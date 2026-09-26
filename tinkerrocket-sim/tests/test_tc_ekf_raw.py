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
from tinkerrocket_sim.estimation.tc_ekf import (C_LIGHT, TcEkf, TcEkfParams, corr_inflation,
                                                ecef2lla, lla2ecef, spp_fix, t_e2ned)
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
