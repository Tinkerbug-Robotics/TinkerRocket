"""tools/rig_replay/signalsim_sensors.py: a vertical SignalSim scene's sensors for a rig replay of it (#1600).

A small scene in the rig's own format (a Const pad, VerticalAcc segments up, a deceleration through apogee, a fall
that ends in the air) through the writer and on through tools/sim_replay_blob.py: the specific force along the nose,
the phases from the trajectory's own knots, the landed tail at the last height, the ignition's GPS time.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
for p in (REPO_ROOT / "tools", REPO_ROOT / "tools" / "rig_replay"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import signalsim_sensors as ss  # noqa: E402
import sim_replay_blob as srb  # noqa: E402


def scene():
    """30 s of pad, 2 s at 50 m/s^2 up, 2 s at -20 m/s^2, then -10 m/s^2 through apogee at 10 s into a fall that
    the file cuts at 20 s after ignition, 60 m/s down."""
    return {
        "time": {"type": "UTC", "year": 2026, "month": 8, "day": 18, "hour": 8, "minute": 37, "second": 42},
        "trajectory": {
            "name": "tiny_vertical",
            "initPosition": {"type": "LLA", "format": "d", "longitude": -119.0, "latitude": 0.0, "altitude": 1200},
            "initVelocity": {"type": "SCU", "speed": 0, "course": 0},
            "trajectoryList": [
                {"type": "Const", "time": 30.0},
                {"type": "VerticalAcc", "time": 2.0, "speed": 100.0},
                {"type": "VerticalAcc", "time": 2.0, "speed": 60.0},
                {"type": "VerticalAcc", "time": 16.0, "speed": -100.0},
            ],
        },
    }


class SignalSimSensors(unittest.TestCase):
    def setUp(self):
        self.cols, self.scen = ss.build(scene(), rate_hz=1000, tail_s=6.0, t_from=-20.0)
        self.t = self.cols["t_s"]

    def at(self, t_s):
        return int(np.argmin(np.abs(self.t - t_s)))

    def test_the_nose_feels_the_trajectorys_acceleration_and_gravity(self):
        g = ss.gravity(0.0, 1200.0)
        k = self.at(-5.0)
        self.assertAlmostEqual(self.cols["fx_mps2"][k], g, places=3)        # nose up: +g on the pad
        self.assertLess(abs(self.cols["fy_mps2"][k]) + abs(self.cols["fz_mps2"][k]), 1e-6)
        self.assertAlmostEqual(self.cols["fx_mps2"][self.at(1.0)], 50.0 + g, delta=0.01)
        self.assertAlmostEqual(self.cols["fx_mps2"][self.at(3.0)], -20.0 + g, delta=0.01)

    def test_phases_come_from_the_knots(self):
        ph = self.cols["phase"]
        self.assertEqual(ph[self.at(-0.5)], ss.PHASE_PAD)
        self.assertEqual(ph[self.at(1.0)], ss.PHASE_BOOST)
        self.assertEqual(ph[self.at(3.0)], ss.PHASE_COAST)
        tr = self.scen["truth"]
        self.assertAlmostEqual(tr["burnout_s"], 2.0)
        self.assertAlmostEqual(tr["apogee_s"], 10.0)       # 60 m/s at 4 s, -10 m/s^2
        self.assertEqual(ph[self.at(12.0)], ss.PHASE_DROGUE)
        self.assertEqual(ph[self.at(21.0)], ss.PHASE_LANDED)

    def test_the_tail_stands_still_at_the_last_height(self):
        h_end = float(ss.Trajectory(scene()).at([50.0])[0][0])    # the file's last instant
        k_tail = self.at(23.0)
        self.assertAlmostEqual(self.cols["baro_p_pa"][k_tail], ss.pressure_pa(h_end), places=6)
        self.assertAlmostEqual(self.cols["fx_mps2"][k_tail], ss.gravity(0.0, 1200.0 + self.cols["baro_alt_m"][k_tail]),
                               places=6)
        self.assertEqual(self.cols["acc_hg"][k_tail], 0.0)

    def test_the_blob_carries_it_at_the_scenes_gps_time(self):
        blob, s = srb.build(self.cols, self.scen, t_from=-20.0, port_error=False, land_loop_s=5.0)
        self.assertEqual(s["rate_hz"], 500)
        # 08:37:42 UTC = 08:38:00 GPST on a Tuesday (2026-08-18), +30 s of pad
        self.assertAlmostEqual(s["ignition_tow_s"], 2 * 86400 + 8 * 3600 + 38 * 60 + 30.0, places=6)
        self.assertEqual(s["gps_week"], 2432)
        self.assertAlmostEqual(s["peak_acc_mps2"], 50.0 + ss.gravity(0.0, 1200.0 + 50.0), delta=0.1)

    def test_pressure_is_the_standard_atmosphere(self):
        self.assertAlmostEqual(ss.pressure_pa(0.0), 101325.0, places=3)
        self.assertAlmostEqual(ss.pressure_pa(11000.0), 22632.06, delta=1.0)
        self.assertTrue(math.isclose(ss.pressure_pa(25000.0), 2511.0, rel_tol=0.002))   # the 1976 table


if __name__ == "__main__":
    unittest.main()
