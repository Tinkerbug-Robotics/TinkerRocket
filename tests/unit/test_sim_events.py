"""tools/raw_replay/sim_events.py: Grid B's simulated receiver, the parts its review found wrong (E3).

- The receiver's fix comes at its own 20 Hz (the mosaic streams PVTGeodetic at msec50), while only every second
  epoch, on the fused 10 Hz grid, goes to the filter.
- The fix's velocity is the truth's of a few ms before the epoch plus the solution's error: solved at the epoch's
  own velocity, which the range rates carry, not at the lagged one (that gave back the epoch's velocity).
- Two stale satellites tell the filter nothing (no firmware path widens one satellite's Doppler sigma).
- settle40 carries the rig's common term (tc_ekf_eval --post-thrust 4,1.5,2: 2 m/s rms common to every Doppler
  over the first 5 s of coast); settle15 has none.
- The short cuts (gapD) start at burnout so GNSS comes back in coast; the slow coherent ramps never start before
  burnout.

The module needs the gnss repo's receiver model and tinkerrocket-sim's extension (as sim_grid.py does); without
them on PYTHONPATH these tests skip:
    PYTHONPATH=<simbuild>/tinkerrocket-sim/src:<gnss-hf-sim>/sim python3 -m pytest tests/unit/test_sim_events.py
"""
import json
import math
import os
import pickle
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools" / "raw_replay"))

try:
    import sim_events  # noqa: E402
    import tinkerrocket_sim._ekf  # noqa: E402,F401
    HAVE_SIM = True
except ImportError as e:  # pragma: no cover - only where the sim is built
    HAVE_SIM = False
    WHY = str(e)

# synthetic: open ocean, nowhere anyone flies
REF = (-45.0, -150.0, 10.0)


def los_geometry(n=8, seed=3):
    """n unit lines of sight spread over the sky (ECEF-like axes), satellite velocities, ranges."""
    rng = np.random.default_rng(seed)
    u = rng.normal(size=(n, 3))
    u[:, 2] = np.abs(u[:, 2]) + 0.3
    u /= np.linalg.norm(u, axis=1)[:, None]
    vs = rng.normal(0.0, 3000.0, size=(n, 3))
    rho = rng.uniform(2.0e7, 2.6e7, size=n)
    return u, vs, rho


def meas_for(u, vs, rho, v, clk=30.0, drift=0.2):
    meas, info = [], dict(rho=list(rho), u=list(u), vs_r=list(vs))
    for k in range(len(u)):
        meas.append(dict(k=k, pr=rho[k] + clk, rr=float(u[k] @ (vs[k] - v)) + drift, s_pr=0.3, s_corr=1.0,
                         s_rr=0.05, rr_sig_add=0.0))
    return meas, info


def synthetic_flight(path, phases_path):
    """A tiny drogue-less flight as E0's pickles hold it: 3 s on the pad, 1 s of 10 g straight up, a ballistic coast
    to apogee and a fall, at 1 kHz; the nose up throughout."""
    import pandas as pd
    dt = 1e-3
    t = np.arange(-3.0, 14.0 + dt / 2, dt)
    g = 9.807
    a = np.where((t >= 0) & (t < 1.0), 10.0 * g - g, np.where(t >= 1.0, -g, 0.0))
    vu = np.cumsum(a) * dt
    vu[t < 0] = 0.0
    h = np.cumsum(vu) * dt
    n = len(t)
    nose_up = np.array([math.cos(math.pi / 4), 0.0, -math.sin(math.pi / 4), 0.0])   # body x up (pitch +90)
    sf = np.where((t >= 0) & (t < 1.0), 10.0 * g, np.where(t >= 1.0, 0.0, g))      # along the nose
    df = pd.DataFrame({
        "time": t, "true_pn": np.zeros(n), "true_pe": np.zeros(n), "true_pd": -h,
        "true_vn": np.zeros(n), "true_ve": np.zeros(n), "true_vd": -vu,
        "true_q0_ned": np.full(n, nose_up[0]), "true_q1_ned": np.zeros(n), "true_q2_ned": np.full(n, nose_up[2]),
        "true_q3_ned": np.zeros(n),
        "imu_acc_x": sf, "imu_acc_y": np.zeros(n), "imu_acc_z": np.zeros(n),
        "imu_gyro_x": np.zeros(n), "imu_gyro_y": np.zeros(n), "imu_gyro_z": np.zeros(n),
        "baro_alt": h,
    })
    with open(path, "wb") as fh:
        pickle.dump(dict(df=df, cfg=dict(ref_lat_deg=REF[0], ref_lon_deg=REF[1], ref_alt_m=REF[2])), fh)
    t_apo = float(t[np.flatnonzero((t > 1.0) & (vu <= 0.0))[0]])
    name = Path(path).stem
    ph = dict(thrust_start_s=0.0, burnout_s=1.0, thrust_end_s=1.0, apogee_s=t_apo, vz_below_2mps_s=t_apo - 0.2,
              deploy_s=t_apo + 1.0, landing_s=14.0)
    with open(phases_path, "w") as fh:
        json.dump({"flights": {name: ph}}, fh)
    return ph


@unittest.skipUnless(HAVE_SIM, "the sim's modules are not on PYTHONPATH")
class FixTest(unittest.TestCase):
    def test_the_fix_is_the_epochs_error_on_the_lagged_truth(self):
        u, vs, rho = los_geometry()
        v_te = np.array([5.0, -3.0, -80.0])
        meas, info = meas_for(u, vs, rho, v_te)
        dpos, dvel = sim_events.solve_fix(meas, info, v_te, np.eye(3))
        # no noise: no error, so the fix reports the lagged truth exactly (the caller adds dvel to it)
        np.testing.assert_allclose(dvel, 0.0, atol=1e-6)
        np.testing.assert_allclose(dpos, 0.0, atol=1e-3)
        # linearized at a velocity 1 m/s off, the solution hands that 1 m/s back: the old way returned v(te)
        _, d_off = sim_events.solve_fix(meas, info, v_te - np.array([0.0, 0.0, 1.0]), np.eye(3))
        np.testing.assert_allclose(d_off, [0.0, 0.0, 1.0], atol=1e-6)

    def test_a_coherent_bias_reaches_the_fix(self):
        u, vs, rho = los_geometry()
        v_te = np.array([0.0, 0.0, -30.0])
        meas, info = meas_for(u, vs, rho, v_te)
        b = np.array([0.0, 0.0, 4.0])   # the receiver's velocity 4 m/s down (these axes: +z down)
        for m in meas:
            m["rr"] -= float(info["u"][m["k"]] @ b)
        _, dvel = sim_events.solve_fix(meas, info, v_te, np.eye(3))
        np.testing.assert_allclose(dvel, b, atol=1e-6)


@unittest.skipUnless(HAVE_SIM, "the sim's modules are not on PYTHONPATH")
class PatternTest(unittest.TestCase):
    def test_settle40_has_the_rigs_common_term_and_settle15_none(self):
        p40 = sim_events.pattern_params("settle40", 1.2, 7.0)["post"]
        p15 = sim_events.pattern_params("settle15", 1.2, 7.0)["post"]
        self.assertEqual(p40, (4.0, 1.5, 2.0, 5.0))
        self.assertEqual(p15[2], 0.0)
        rng = np.random.default_rng(7)
        n = 16
        # late in the window the per-satellite term has decayed (4 exp(-4.5/1.5) = 0.2 m/s): the epoch's mean is the
        # common draw, 2 m/s rms
        means = [np.mean(sim_events.settle_offsets(rng, n, 4.5, p40)) for _ in range(2000)]
        self.assertAlmostEqual(float(np.std(means)), 2.0, delta=0.15)
        # past the 5 s window there is no common term
        late = [np.mean(sim_events.settle_offsets(rng, n, 5.5, p40)) for _ in range(2000)]
        self.assertLess(float(np.std(late)), 0.1)
        # settle15 at the same time: no common term either
        s15 = [np.mean(o) for o in (sim_events.settle_offsets(rng, n, 4.5, p15) for _ in range(2000)) if o]
        self.assertLess(float(np.std(s15)), 0.1)
        # and nothing left at all once the settling has decayed
        self.assertIsNone(sim_events.settle_offsets(rng, n, 20.0, p15))

    def test_stale_satellites_carry_no_sigma(self):
        self.assertEqual(sim_events.pattern_params("stale_sats", 1.2, 7.0)["stale_sats"], (2, 4.0))

    def test_the_short_cuts_start_at_burnout_and_the_long_ones_run_to_t8(self):
        for d in ("05", "15", "30"):
            p = sim_events.pattern_params("gap" + d, 1.29, 8.4)
            self.assertEqual(p["cut"], (1.29, 1.29 + int(d) / 10.0))
            self.assertIsNotNone(p["corrupt"])
            self.assertEqual(sim_events.pattern_params("cut" + d, 1.29, 8.4)["cut"], (int(d) / 10.0, 8.0))

    def test_the_slow_ramps(self):
        t_burn, t_apo = 1.29, 8.4
        coh = sim_events.pattern_params("slow15", t_burn, t_apo)["coh"]
        self.assertAlmostEqual(sim_events.coherent_bias(t_apo, coh), 15.0)
        self.assertAlmostEqual(sim_events.coherent_bias(t_apo - 6.0, coh), 0.0)
        self.assertAlmostEqual(sim_events.coherent_bias(t_apo - 3.0, coh), 7.5)
        self.assertAlmostEqual(sim_events.coherent_bias(t_apo - 2.0, sim_events.pattern_params(
            "slow12", t_burn, t_apo)["coh"]), 8.0)
        # a short coast: the ramp starts at burnout, not in the boost, at the same slope
        coh = sim_events.pattern_params("slow15", 1.19, 6.72)["coh"]
        self.assertEqual(coh[2], 1.19)
        self.assertAlmostEqual(sim_events.coherent_bias(1.19, coh), 0.0)
        self.assertAlmostEqual(sim_events.coherent_bias(6.72, coh), 15.0 * (6.72 - 1.19) / 6.0)
        self.assertAlmostEqual(sim_events.coherent_bias(t_apo, sim_events.pattern_params(
            "slow08", t_burn, t_apo)["coh"]), 8.0)
        coh = sim_events.pattern_params("slow15b", t_burn, t_apo)["coh"]
        self.assertAlmostEqual(sim_events.coherent_bias(t_burn + 6.0, coh), 15.0)
        self.assertAlmostEqual(sim_events.coherent_bias(t_apo + 5.0, coh), 15.0)   # held
        # the plan's coherent ramp and the fast step are unchanged
        self.assertAlmostEqual(sim_events.coherent_bias(t_apo, sim_events.pattern_params(
            "coherent", t_burn, t_apo)["coh"]), 5.0)
        self.assertAlmostEqual(sim_events.coherent_bias(t_apo - 2.0, sim_events.pattern_params(
            "coherent15", t_burn, t_apo)["coh"]), 15.0)


@unittest.skipUnless(HAVE_SIM, "the sim's modules are not on PYTHONPATH")
class RolledAntennaTest(unittest.TestCase):
    def test_the_rolled_truth_turns_an_antenna_off_the_axis_at_5_rev_s(self):
        # --roll with --lever-arm-b: the antenna's truth (antenna_truth) takes the rolled body's attitude and rate
        # (roll_overlay), so the Goblin's antenna (r_b FRD, 12.6 mm off the nose's axis) circles the axis at the roll
        # rate while its part along the nose stays put, and its velocity is its offset's rate of change
        import sim_roll
        dt = 1e-3
        t = np.arange(-1.0, 3.0 + dt / 2, dt)
        n = len(t)
        nose_up = np.array([math.cos(math.pi / 4), 0.0, -math.sin(math.pi / 4), 0.0])   # synthetic_flight's
        p = np.zeros((n, 3))
        p[:, 2] = -50.0 * t
        f = dict(t=t, q=np.tile(nose_up, (n, 1)), v=np.tile([0.0, 0.0, -50.0], (n, 1)), acc=np.zeros((n, 3)),
                 gyr=np.zeros((n, 3)))
        ph = dict(thrust_start_s=0.0, burnout_s=1.0, apogee_s=10.0, deploy_s=11.0)
        w_body, C_n2b, facts = sim_events.roll_overlay(f, ph, sim_roll.parse_profile("const:5"))
        self.assertAlmostEqual(facts["peak_rps"], 5.0, places=6)
        r_b = np.array([0.04696, -0.01138, 0.00548])
        rho = math.hypot(r_b[1], r_b[2])
        p_ant, v_ant = sim_events.antenna_truth(p, f["v"], C_n2b, w_body, r_b)
        on = (t > 0.2) & (t < 2.8)                    # past the profile's 0.1 s ramp
        d = (p_ant - p)[on]
        nose = C_n2b[on, 0, :]                        # body x in NED, as build takes it (vertical here)
        np.testing.assert_allclose(np.abs(nose[:, 2]), 1.0, atol=1e-12)
        np.testing.assert_allclose(np.einsum("ni,ni->n", d, nose), r_b[0], atol=1e-9)   # along the nose: r_a
        np.testing.assert_allclose(np.hypot(d[:, 0], d[:, 1]), rho, atol=1e-9)       # across it: rho
        ang = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
        rate = np.polyfit(t[on], ang, 1)[0]
        self.assertAlmostEqual(abs(rate) / sim_roll.TWO_PI, 5.0, places=6)
        # its velocity: the offset's turn, rho times the roll rate (0.40 m/s), as the offset's own derivative
        dv = (v_ant - f["v"])[on]
        np.testing.assert_allclose(np.linalg.norm(dv, axis=1), rho * 5.0 * sim_roll.TWO_PI, rtol=1e-6)
        dd = np.gradient(p_ant - p, t, axis=0)[on]     # (a central difference: its chord 1.6e-4 short at 1 ms)
        np.testing.assert_allclose(dv, dd, atol=1e-3 * rho * 5.0 * sim_roll.TWO_PI)
        # with no roll the same antenna stays where the body put it
        f0 = dict(t=t, q=np.tile(nose_up, (n, 1)), v=f["v"], acc=np.zeros((n, 3)), gyr=np.zeros((n, 3)))
        w0, C0, _ = sim_events.roll_overlay(f0, ph, None)
        p0, v0 = sim_events.antenna_truth(p, f["v"], C0, w0, r_b)
        np.testing.assert_allclose(np.ptp(p0 - p, axis=0), 0.0, atol=1e-9)
        np.testing.assert_allclose(v0 - f["v"], 0.0, atol=1e-9)


@unittest.skipUnless(HAVE_SIM, "the sim's modules are not on PYTHONPATH")
class EventsFileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        d = Path(cls.tmp.name)
        cls.flight = str(d / "synth_flight.pkl")
        cls.phases = synthetic_flight(cls.flight, d / "truth_phases.json")
        cls.all_phases = json.load(open(d / "truth_phases.json"))["flights"]
        cls.out = str(d / "ev")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def lines(self, pattern):
        path, counts = sim_events.build(self.flight, pattern, self.out, self.all_phases, lever_arm=0.0)
        with open(path) as fh:
            return [ln.split() for ln in fh if not ln.startswith(("#", "META"))], counts

    def test_fixes_at_20_hz_epochs_at_10(self):
        rows, counts = self.lines("none")
        v = [int(r[1]) for r in rows if r[0] == "V"]
        x = [int(r[2]) for r in rows if r[0] == "X"]   # each epoch's own GPS instant (MCU us)
        self.assertGreater(len(x), 100)
        self.assertEqual(set(np.diff(v).tolist()), {50000})
        self.assertEqual(set(np.diff(x).tolist()), {100000})
        self.assertEqual(len(v), 2 * len(x))
        self.assertEqual(counts["sim_fixes"], len(v))
        self.assertEqual(counts["sim_epochs"], len(x))
        # the truth for the quorum on it: a row for each 1 kHz row of the flight, in time order
        nav = Path(self.out) / "truth" / "synth_flight.nav"
        t_us = np.loadtxt(nav, usecols=0, dtype=np.int64)
        self.assertTrue(np.all(np.diff(t_us) > 0))
        self.assertGreater(len(t_us), 15000)

    def test_stale_satellites_are_not_flagged_to_the_filter(self):
        rows, counts = self.lines("stale_sats")
        self.assertGreater(counts["sim_stale_epochs"], 0)
        extra = [float(r[6 + 17 * k + 16]) for r in rows if r[0] == "X" for k in range(int(r[5]))]
        self.assertTrue(extra)
        self.assertEqual(max(extra), 0.0)

    def test_a_short_cut_ends_in_coast(self):
        rows, counts = self.lines("gap15")
        x = np.array([int(r[2]) for r in rows if r[0] == "X"]) * 1e-6 - sim_events.T_OFS
        v = np.array([int(r[1]) for r in rows if r[0] == "V"]) * 1e-6 - sim_events.T_OFS - sim_events.FIX_ARRIVAL_S
        t_b = self.phases["burnout_s"]
        self.assertFalse(np.any((x >= t_b - 1e-6) & (x < t_b + 1.5 - 1e-6)))
        self.assertFalse(np.any((v >= t_b - 1e-6) & (v < t_b + 1.5 - 1e-6)))
        self.assertTrue(np.any((x >= t_b + 1.5) & (x < self.phases["apogee_s"])))   # back before apogee
        self.assertEqual(counts["sim_epochs_cut"], 15)
        self.assertGreater(counts["sim_sats_corrupted"], 0)


if __name__ == "__main__":
    unittest.main()
