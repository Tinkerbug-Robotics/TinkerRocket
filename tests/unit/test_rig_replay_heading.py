"""tools/rig_replay/score_heading.py: the heading from thrust on a rig replay (HEADING_FIT_MSG 0x9F).

The scorer's own arithmetic: a heading turned about NED down whatever the tilt (as the flight computer's
headingBetween), the fits read from a log's records, when the 1-sigma came under 10 and 5 deg and the hand-off, and
two runs' turns set side by side -- they differ by the bench error one was built with.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
for p in (HERE, REPO_ROOT / "tools" / "rig_replay"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import score_heading as sh  # noqa: E402


def qmul(a, b):
    return np.array([a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                     a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                     a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                     a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]])


def qaxis(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return np.concatenate(([math.cos(h)], math.sin(h) * a))


def record(t_us, psi, sigma, flags=0x02, turn=0.0, filt=0.0, n=3):
    return {"time_us": t_us, "psi_deg": psi, "sigma_deg": sigma, "filt_deg": filt, "turn_deg": turn,
            "n": n, "n_cut": 0, "sats": 9, "vel_us": 400, "solve_us": 600, "seed_ms": 110, "speed_mps": 12.0,
            "act": bool(flags & 0x01), "handed_now": bool(flags & 0x04), "rival": bool(flags & 0x10)}


class TestTwist(unittest.TestCase):
    def test_a_turn_about_down_is_its_twist_at_any_tilt(self):
        for tilt in (0.0, 5.0, 60.0, 89.0):
            b = qmul(qaxis([0, 0, 1], 70.0), qaxis([0, 1, 0], 90.0 - tilt))      # nose up, leaning
            for turn in (-150.0, -40.0, 0.0, 25.0, 179.0):
                a = qmul(qaxis([0, 0, 1], turn), b)
                self.assertAlmostEqual(float(sh.twist_deg(a, b)[0]), turn, places=6)
                self.assertAlmostEqual(float(sh.twist_deg(b, a)[0]), -turn, places=6)


class TestFits(unittest.TestCase):
    def recs(self):
        return {"HeadingFit": [record(4_000_000, -12.0, None), record(5_000_000, -38.5, 9.0),
                               record(6_000_000, -39.7, 4.75, flags=0x07, turn=39.6, filt=-39.6),
                               record(7_000_000, -40.1, 3.5, flags=0x0B)]}

    def test_the_fits_on_the_flights_time_and_their_milestones(self):
        f = sh.fits(self.recs(), base=-2.0)
        np.testing.assert_allclose(f["t"], [2.0, 3.0, 4.0, 5.0])
        self.assertTrue(np.isnan(f["sigma"][0]))
        s = sh.summary(f)
        self.assertEqual(s["sig10"], (3.0, -38.5))
        self.assertEqual(s["sig5"], (4.0, -39.7))
        self.assertEqual(s["handed"], (4.0, 39.6, 4.75))
        self.assertEqual(s["rivals"], 0)

    def test_two_runs_differ_by_the_bench_error(self):
        f = sh.fits(self.recs(), base=0.0)
        ref = {"HeadingFit": [record(4_050_000, 1.5, None), record(5_000_000, 1.0, 9.25),
                              record(6_100_000, 0.4, 5.0), record(7_000_000, 0.0, 3.5)]}
        c = sh.compare(f, sh.fits(ref, base=0.0))
        self.assertEqual(len(c), 4)
        np.testing.assert_allclose([x[1] for x in c], [-13.5, -39.5, -40.1, -40.1])


if __name__ == "__main__":
    unittest.main()
