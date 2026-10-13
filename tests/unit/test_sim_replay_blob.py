"""tools/sim_replay_blob.py: a flight's sensors as a TR_SIM_REPLAY flight computer carries them (#1600).

A small made-up flight (100 Hz, 4 s: pad, boost, ground) through the writer: the rocket frame and attitude in FLU, the
pairs averaged, the static-port error as a pressure, the header's GPS time. The same flight's file is the C++ reader's
fixture (tests_cpp/fixtures/sim_replay_tiny.bin, test_sim_replay.cpp), so the two cannot drift apart: rewrite it with

    python3 tests/unit/test_sim_replay_blob.py --write-fixture
"""
import struct
import sys
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "tools"))

import sim_replay_blob as srb  # noqa: E402

FIXTURE = REPO_ROOT / "tests_cpp" / "fixtures" / "sim_replay_tiny.bin"
PAD_Q = np.array([0.68973575, -0.25330126, 0.61621194, 0.28352410])   # rp5's rail, body FRD -> NED


def tiny_flight():
    t = np.round(np.arange(-200, 200) * 0.01, 2)
    up = t > 0
    h = np.where(up, 50.0 * t, 0.0)
    p = 100000.0 * (1.0 - h / 44330.0) ** srb.BARO_N
    q = np.tile(PAD_Q / np.linalg.norm(PAD_Q), (len(t), 1))
    cols = {
        "t_s": t,
        "phase": np.where(t < 0, 0, np.where(t < 1.0, 2, 6)).astype(float),
        "fx_mps2": 10.0 + t, "fy_mps2": np.full_like(t, 0.5), "fz_mps2": -1.0 + 0.25 * t,
        "wx_rps": 0.1 * t, "wy_rps": np.full_like(t, 0.2), "wz_rps": np.full_like(t, -0.3),
        "qw": q[:, 0], "qx": q[:, 1], "qy": q[:, 2], "qz": q[:, 3],
        "acc_hg": ((t >= 0.5) & (t < 0.6)).astype(float),
        "baro_p_pa": p, "baro_alt_m": h, "baro_alt_port_m": h - 2.0 * up,
    }
    scen = {"scenario": "tiny_pad3", "prologue_s": 3.0, "duration_s": 5.0, "start_utc": "2026-08-29T15:19:48",
            "tow0_file": 573606.0, "origin": {"lat_deg": 39.4683565, "lon_deg": -75.29286495, "alt_m": -0.712},
            "flight": {"id": "tiny"}}
    return cols, scen


def tiny_blob(**kw):
    cols, scen = tiny_flight()
    args = dict(t_from=-2.0, pad_loop_s=1.0, land_loop_s=0.5)
    args.update(kw)
    return srb.build(cols, scen, **args)


def samples(blob):
    hdr = struct.unpack_from(srb.HEADER_FMT, blob, 0)
    n = hdr[5]
    return hdr, [struct.unpack_from(srb.SAMPLE_FMT, blob, srb.HEADER_BYTES + i * srb.SAMPLE_BYTES) for i in range(n)]


class SimReplayBlobTest(unittest.TestCase):
    def test_header(self):
        blob, s = tiny_blob()
        hdr, recs = samples(blob)
        self.assertEqual(hdr[0], b"TRRP")
        self.assertEqual(hdr[1:6], (1, 128, 24, 50, 200))
        self.assertEqual(len(blob), 128 + 200 * 24)
        self.assertAlmostEqual(hdr[6], -1.995, places=9)
        # ignition: the file's first instant (TOW 573606, week 2433) plus the 3 s prologue
        self.assertEqual(hdr[7], 2433 * 604800 + 573609)
        self.assertEqual(s["gps_week"], 2433)
        self.assertEqual(hdr[11:13], (-3.0, 2.0))
        self.assertEqual(hdr[17:19], (1.0, 0.5))
        self.assertAlmostEqual(hdr[19], 2026 + 240 / 365, delta=2e-4)   # 29 August: day 241 (a float)
        self.assertEqual(hdr[20].rstrip(b"\0"), b"tiny")

    def test_rocket_frame_is_flu(self):
        blob, _ = tiny_blob()
        hdr, recs = samples(blob)
        acc_lsb, gyro_lsb = hdr[13], hdr[14]
        # sample 0: the mean of t = -2.00 and -1.99
        t = -1.995
        f, w, q = np.array(recs[0][0:3]) * acc_lsb, np.array(recs[0][3:6]) * gyro_lsb, np.array(recs[0][6:10]) / 32767
        np.testing.assert_allclose(f, [10.0 + t, -0.5, 1.0 - 0.25 * t], atol=acc_lsb)
        np.testing.assert_allclose(w, [0.1 * t, -0.2, 0.3], atol=gyro_lsb)
        # the FLU attitude turns a FLU vector where the FRD one turns the same vector in FRD
        f_frd = f * np.array([1.0, -1.0, -1.0])
        np.testing.assert_allclose(srb.rotate(q[None, :], f[None, :]), srb.rotate(PAD_Q[None, :], f_frd[None, :]),
                                   atol=1e-3)

    def test_phase_and_high_g(self):
        _, recs = samples(tiny_blob()[0])
        phases = [r[11] for r in recs]
        self.assertEqual(phases[:100], [0] * 100)
        self.assertEqual(phases[100:150], [2] * 50)
        self.assertEqual(phases[150:], [6] * 50)
        hg = [i for i, r in enumerate(recs) if r[12] & 1]
        self.assertEqual(hg, list(range(125, 130)))       # 0.50 .. 0.59 s

    def test_static_port_error_is_a_pressure(self):
        with_port, _ = tiny_blob()
        without, _ = tiny_blob(port_error=False)
        hdr, a = samples(with_port)
        _, b = samples(without)
        lsb = hdr[16]
        i = 180                                           # t = 1.605 s, the port reading 2 m low
        dp = (a[i][10] - b[i][10]) * lsb
        p = 100000.0 * (1.0 - 50.0 * 1.605 / 44330.0) ** srb.BARO_N
        self.assertAlmostEqual(dp, srb.BARO_N * p / (44330.0 - 80.25) * 2.0, delta=lsb)
        self.assertEqual(a[50][10], b[50][10])            # on the pad: none

    def test_refuses_a_file_without_its_pad(self):
        with self.assertRaises(ValueError):
            tiny_blob(t_from=-0.5)                        # the first second is not all pad

    def test_fixture_is_fresh(self):
        blob, _ = tiny_blob()
        self.assertTrue(FIXTURE.exists(), f"{FIXTURE} missing: run this file with --write-fixture")
        self.assertEqual(FIXTURE.read_bytes(), blob, "the C++ reader's fixture is stale: --write-fixture")


if __name__ == "__main__":
    if "--write-fixture" in sys.argv:
        FIXTURE.write_bytes(tiny_blob()[0])
        print(f"wrote {FIXTURE}")
    else:
        unittest.main()
