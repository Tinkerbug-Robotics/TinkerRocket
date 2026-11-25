"""tools/rig_replay/truth.py: the ignition edge the rig scorers put a flight log's clock on (#1600, #1626 T4).

The scorers line a replay's log up with its flight by where the replayed accelerometer crosses 20 m/s^2. That edge
is the flight's own, read from its replay file or worked out from its CSV the way sim_replay_blob.py writes the file,
not one flight's number for every flight.
"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
for p in (HERE, REPO_ROOT / "tools" / "rig_replay"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import truth  # noqa: E402
from test_sim_replay_blob import tiny_flight  # noqa: E402

KW = dict(t_from=-2.0, pad_loop_s=1.0, land_loop_s=0.5)


def ramp_flight():
    """The tiny flight with its specific force along the nose only: 10 m/s^2 on the pad, then 10 + 20 t, which
    crosses 20 m/s^2 at t = 0.5 s exactly, also in the pairs (a straight line's pair mean is its mid-time value)."""
    cols, scen = tiny_flight()
    t = cols["t_s"]
    cols["fx_mps2"] = np.where(t > 0, 10.0 + 20.0 * t, 10.0)
    cols["fy_mps2"] = np.zeros_like(t)
    cols["fz_mps2"] = np.zeros_like(t)
    return cols, scen


class IgnitionEdgeTest(unittest.TestCase):
    def test_the_replay_files_edge_is_where_its_pairs_cross(self):
        cols, scen = ramp_flight()
        blob, _ = truth.srb.build(cols, scen, **KW)
        self.assertAlmostEqual(truth.blob_edge_t(blob), 0.5, delta=1e-6)

    def test_from_the_csv_or_the_file_it_is_the_same_edge(self):
        cols, scen = ramp_flight()
        blob, _ = truth.srb.build(cols, scen, **KW)
        with tempfile.TemporaryDirectory() as d:
            csv = Path(d) / "sensors.csv"
            names = list(cols)
            with open(csv, "w") as f:
                f.write("# a made-up flight\n" + ",".join(names) + "\n")
                np.savetxt(f, np.stack([cols[n] for n in names], axis=1), delimiter=",", fmt="%.9g")
            scen_path = Path(d) / "scenario.json"
            scen_path.write_text(json.dumps(scen))
            blob_path = Path(d) / "replay.bin"
            blob_path.write_bytes(blob)
            from_csv = truth.edge_t(str(csv), str(scen_path), **KW)
            from_file = truth.edge_t(None, None, str(blob_path))
        self.assertAlmostEqual(from_csv, truth.blob_edge_t(blob), delta=1e-9)
        self.assertAlmostEqual(from_file, truth.blob_edge_t(blob), delta=1e-9)

    def test_a_flight_that_never_crosses_has_no_edge(self):
        cols, scen = tiny_flight()          # about 10-12 m/s^2 throughout
        blob, _ = truth.srb.build(cols, scen, **KW)
        with self.assertRaises(ValueError):
            truth.blob_edge_t(blob)


if __name__ == "__main__":
    unittest.main()
