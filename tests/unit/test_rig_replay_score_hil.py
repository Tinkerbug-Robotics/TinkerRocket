"""tools/rig_replay/score_hil.py: the HIL scorer's math -- the scene's truth from a SignalSim config, the error
statistics by phase, the headline source per phase, the apogee, the console's lines, the raw filter's GNSS use,
the board's 4001 fix and epoch, and a log's whole path on records made from the truth.

A small scene in the rig's format (the B1C files' made-up pad, 0 N 119 W): 30 s of pad, 2 s at 50 m/s^2 up, 2 s at
-20 m/s^2, then -10 m/s^2 through apogee at T+10 s (440 m up) to the file's end at T+16 s, 260 m up and falling at
60 m/s.
"""
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
for p in (REPO_ROOT / "Data_Analysis", REPO_ROOT / "tools", REPO_ROOT / "tools" / "rig_replay"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import extract_gnss_rtcm as XR  # noqa: E402
import score_hil as H  # noqa: E402

PAD = (0.0, -119.0, 1200.0)


def scene():
    return {
        "time": {"type": "UTC", "year": 2026, "month": 8, "day": 18, "hour": 8, "minute": 37, "second": 42},
        "trajectory": {
            "name": "tiny_vertical",
            "initPosition": {"type": "LLA", "format": "d", "longitude": PAD[1], "latitude": PAD[0],
                             "altitude": PAD[2]},
            "initVelocity": {"type": "SCU", "speed": 0, "course": 0},
            "trajectoryList": [
                {"type": "Const", "time": 30.0},
                {"type": "VerticalAcc", "time": 2.0, "speed": 100.0},
                {"type": "VerticalAcc", "time": 2.0, "speed": 60.0},
                {"type": "VerticalAcc", "time": 12.0, "speed": -60.0},
            ],
        },
    }


def truth():
    return H.SceneTruth(scene())


def shifted(tr, t, name="log-nav", dpos=(0.0, 0.0, 0.0), dvel=(0.0, 0.0, 0.0), frame="test"):
    """The truth at t, plus constant NED offsets."""
    p, v = tr.at(t)
    return H.Series(name, np.asarray(t, float), p + np.array(dpos), v + np.array(dvel), frame)


class TestSceneTruth(unittest.TestCase):
    def test_phases_and_heights(self):
        tr = truth()
        self.assertAlmostEqual(tr.burnout, 2.0)
        self.assertAlmostEqual(tr.apogee, 10.0)
        self.assertAlmostEqual(tr.end, 16.0)
        self.assertAlmostEqual(tr.apogee_m, 440.0)
        self.assertAlmostEqual(tr.end_m, 260.0)
        self.assertAlmostEqual(tr.end_vu, -60.0)
        self.assertEqual(tr.name, "tiny_vertical")

    def test_ignition_gps_time(self):
        # 2026-08-18 08:37:42 UTC = GPS week 2432, Tuesday 08:38:00 (18 leap seconds); ignition 30 s later
        tr = truth()
        week, tow = divmod(tr.ign_gps_s, H.WEEK_S)
        self.assertEqual(week, 2432)
        self.assertAlmostEqual(tow, 2 * 86400 + 8 * 3600 + 38 * 60 + 30.0)

    def test_at_ned(self):
        tr = truth()
        p, v = tr.at([-5.0, 1.0, 10.0, 16.0, 16.5, -31.0])
        np.testing.assert_allclose(p[0], [0, 0, 0], atol=1e-9)
        np.testing.assert_allclose(p[1], [0, 0, -25.0], atol=1e-9)          # 0.5 * 50 * 1^2 up
        np.testing.assert_allclose(v[1], [0, 0, -50.0], atol=1e-9)
        np.testing.assert_allclose(p[2], [0, 0, -440.0], atol=1e-9)
        np.testing.assert_allclose(v[2], [0, 0, 0.0], atol=1e-9)
        np.testing.assert_allclose(v[3], [0, 0, 60.0], atol=1e-9)
        self.assertTrue(np.isnan(p[4]).all() and np.isnan(v[5]).all())      # after the file, before it

    def test_windows(self):
        w = truth().windows(20.0)
        self.assertEqual([x[0] for x in w], ["pad", "boost", "coast", "descent"])
        self.assertEqual(w[0], ("pad", -20.0, 0.0))
        self.assertEqual(w[-1], ("descent", 10.0, 16.0))
        self.assertEqual(truth().windows(99.0)[0], ("pad", -30.0, 0.0))     # no longer than the file's pad


class TestGeodesy(unittest.TestCase):
    def test_up_is_minus_down(self):
        ned = H.lla_to_ned([PAD[0]], [PAD[1]], [PAD[2] + 1000.0], PAD)
        np.testing.assert_allclose(ned[0], [0.0, 0.0, -1000.0], atol=1e-6)

    def test_north_and_east(self):
        ned = H.lla_to_ned([PAD[0] + 1e-4, PAD[0]], [PAD[1], PAD[1] + 1e-4], [PAD[2]] * 2, PAD)
        self.assertGreater(ned[0, 0], 11.0)
        self.assertLess(abs(ned[0, 1]), 1e-6)
        self.assertGreater(ned[1, 1], 11.0)
        self.assertLess(abs(ned[1, 0]), 1e-6)


class TestStats(unittest.TestCase):
    def test_abs_nan_and_percentile(self):
        s = H.stats([1.0, -2.0, 3.0, float("nan")])
        self.assertEqual(s["n"], 3)
        self.assertAlmostEqual(s["median"], 2.0)
        self.assertAlmostEqual(s["p90"], 2.8)
        self.assertAlmostEqual(s["max"], 3.0)
        self.assertIsNone(H.stats([float("nan")]))

    def test_constant_offsets_by_phase(self):
        tr = truth()
        w = tr.windows(20.0)
        t = np.arange(-20.0, 16.0, 0.01)
        r = H.score_series(shifted(tr, t, dpos=(3.0, 4.0, -12.0), dvel=(0.0, 0.0, 2.0)), tr, w)
        for ph in ("pad", "boost", "coast", "descent"):
            m = r[ph]
            self.assertAlmostEqual(m["pos_h"]["max"], 5.0)
            self.assertAlmostEqual(m["pos_v"]["median"], 12.0)
            self.assertAlmostEqual(m["pos_3d"]["p90"], 13.0)
            self.assertAlmostEqual(m["vel_v"]["max"], 2.0)
            self.assertAlmostEqual(m["vel_h"]["max"], 0.0)
            self.assertGreater(m["cover"], 0.99)
        self.assertEqual(r["boost"]["n"], 200)

    def test_vertical_sign_is_up(self):
        tr = truth()
        e = H.errors(shifted(tr, [5.0], dpos=(0, 0, -7.0), dvel=(0, 0, 1.5)), tr)
        self.assertAlmostEqual(e["pos_v"][0], 7.0)        # 7 m too high
        self.assertAlmostEqual(e["vel_v"][0], -1.5)       # sinking 1.5 m/s too fast

    def test_cover_and_gap(self):
        tr = truth()
        w = [("boost", 0.0, 2.0)]
        r = H.score_series(shifted(tr, [0.5, 1.0]), tr, w)["boost"]
        self.assertEqual(r["n"], 2)
        self.assertAlmostEqual(r["cover"], 0.5)
        self.assertAlmostEqual(r["max_gap_s"], 1.0)
        self.assertEqual(H.score_series(shifted(tr, [5.0]), tr, w)["boost"]["n"], 0)

    def test_headline_falls_back_to_the_console_on_the_pad(self):
        tr = truth()
        w = tr.windows(20.0)
        scored = {"log-nav": H.score_series(shifted(tr, np.arange(-0.4, 16.0, 0.01)), tr, w),
                  "console": H.score_series(shifted(tr, np.arange(-19.5, 16.0, 1.0), "console"), tr, w)}
        self.assertEqual(H.headline(scored, w), {"pad": "console", "boost": "log-nav", "coast": "log-nav",
                                                 "descent": "log-nav"})


class TestApogee(unittest.TestCase):
    def test_call_and_peak(self):
        tr = truth()
        a = H.apogee_scores(tr, call_t=10.5, s=shifted(tr, np.arange(-1.0, 16.0, 0.01), dpos=(0, 0, -10.0)))
        self.assertAlmostEqual(a["call_dt"], 0.5)
        self.assertAlmostEqual(a["call_true_vu"], -5.0)
        self.assertAlmostEqual(a["call_true_m"], 440.0 - 1.25)
        self.assertAlmostEqual(a["peak_dm"], 10.0, places=3)
        self.assertAlmostEqual(a["peak_dt"], 0.0, places=6)
        self.assertAlmostEqual(a["at_truth_dm"], 10.0, places=3)
        self.assertAlmostEqual(a["call_est_m"], 450.0 - 1.25, places=3)

    def test_no_call(self):
        a = H.apogee_scores(truth())
        self.assertNotIn("call_t", a)
        self.assertEqual(a["truth_m"], 440.0)


def console_text():
    """A reset's two ROM banners, the replay's boot line, [REPLAY] lines (ENU) on the clock from t = -2 s (stamp
    - 100 s), the apogee line 0.45 s of stamp after its millis, then a second boot that must end the reading."""
    L = ["ESP-ROM:esp32s3-20210327", "rst:0x15 (USB_UART_CHIPESP-ROM:esp32s3-20210327",
         "I (441) app_init: App version:      abc12345-v10-bench",
         "W (3427) SIM: RIG REPLAY: 'tiny_vertical', 100 samples at 500 Hz, t -30.00 .. +20.00 s; ignition at GPS "
         "week 2432 TOW 203910.000; the rig's file spans",
         "I (50000) FC: [REPLAY] t=+0.00(pad, no clock) phase=0 clock=0 fixes=0 (last t=+0.00, outside=0, "
         "restarts=0) ekf=1 pos=(9.0,9.0,9.0) vel=(0.00,0.00,0.00)"]
    for k in range(-2, 15):
        ms = 100000 + k * 1000
        L.append(f"I ({ms}) FC: [REPLAY] t={k:+.2f} phase=2 clock=2 fixes=5 (last t={k:+.2f}, outside=0, restarts=0) "
                 f"ekf=1 pos=(1.0,2.0,{30.0 + k:.1f}) vel=(0.10,0.20,{3.0 + k:.2f})")
    L.append("I (110450) FC: [PYRO] Apogee detected at t=110000 ms (vel=1 baro=0 gps=0 pitch=1 backstop=0)")
    L.append("I (99300) FC: [STATE] READY -> INFLIGHT (ground_p=87716)")
    L.append("I (101000) FC: [GAP DIAG] iter_max=2065 mI (101001) FC: [EKF] canopy mode on: velocity follows at "
             "T+9.50 s (#1580)")
    L.append("ESP-ROM:esp32s3-20210327")
    L.append("I (120000) FC: [REPLAY] t=+99.00 phase=6 clock=2 fixes=5 (last t=+0.00, outside=0, restarts=0) ekf=1 "
             "pos=(0.0,0.0,0.0) vel=(0.00,0.00,0.00)")
    return "\n".join(L) + "\n"


class TestConsole(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.dir.name, "fc_console.log")
        Path(self.path).write_text(console_text())

    def tearDown(self):
        self.dir.cleanup()

    def test_series_enu_to_ned_and_the_clock(self):
        series, ev = H.parse_console([self.path])
        s = series["console"]
        self.assertIsNone(series["console-nav"])
        self.assertEqual(len(s.t), 17)                     # the no-clock line and the second boot's left out
        np.testing.assert_allclose(s.pos[0], [2.0, 1.0, -28.0])
        np.testing.assert_allclose(s.vel[3], [0.2, 0.1, -4.0])
        self.assertAlmostEqual(ev["t_off_s"], -100.0)
        self.assertEqual(ev["version"], "abc12345-v10-bench")
        self.assertEqual(ev["ignition_gps"], ("tiny_vertical", 2432, 203910.0))

    def test_events_on_flight_time(self):
        _s, ev = H.parse_console([self.path])
        self.assertAlmostEqual(ev["apogee_t"], 10.45)
        self.assertEqual(ev["apogee_stamp_minus_millis_ms"], 450)
        self.assertAlmostEqual(ev["state_INFLIGHT_t"], -0.7)
        self.assertAlmostEqual(ev["canopy_t"], 9.5)          # the second message on a line

    def test_replay_nav_lines(self):
        # main.cpp's [REPLAY-NAV] printf (a TR_RAW_NAV_RIG_REPLAY image), on and off the replay's clock
        fmt = ("I ({ms}) FC: [REPLAY-NAV] t={t:+.4f}{nc} st=2 ekf=1 vote=1 q=(0.70711,0.00000,0.70711,0.00000) "
               "pos=(1.25,-0.50,{u:.2f}) vel=(0.010,0.020,{vu:.3f}) baro=(25.10,+50.00) gps_vu=+50.00 votes=0000 "
               "call=0 backstop=0 burnout=0 gps_admitted=1 guard=0 mach=0")
        lines = [fmt.format(ms=1000, t=0.0, nc="(pad, no clock)", u=0.0, vu=0.0)]
        lines += [fmt.format(ms=200000 + k * 20, t=k * 0.02, nc="", u=0.01 * k, vu=1.0 * k) for k in range(10)]
        Path(self.path).write_text("\n".join(lines) + "\n")
        series, ev = H.parse_console([self.path])
        s = series["console-nav"]
        self.assertIsNone(series["console"])
        self.assertEqual(len(s.t), 10)
        np.testing.assert_allclose(s.pos[3], [-0.5, 1.25, -0.03])
        np.testing.assert_allclose(s.vel[3], [0.02, 0.01, -3.0])
        self.assertAlmostEqual(ev["t_off_s"], -200.0)
        self.assertEqual(ev["lines"], {"replay": 0, "replay_nav": 11})

    def test_score_run_on_the_console(self):
        tr = truth()
        r = H.score_run("console only", tr, tr.windows(20.0), consoles=[self.path])
        self.assertAlmostEqual(r["time"]["console"]["ignition_gps_minus_scene_s"], 0.0)
        self.assertEqual(r["headline"]["boost"], "console")
        self.assertAlmostEqual(r["apogee"]["call_dt"], 0.45)
        self.assertTrue(any("no raw-GNSS filter records" in n for n in r["notes"]))
        self.assertTrue(any(n.startswith("NIS: not logged") for n in r["notes"]))


def raw_rec(t_us, tow_s, used=(4, 6), rej=(0, 0), age_s=0.03, running=True, lla=(0.0, -119.0, 1200.0),
            vel=(0.0, 0.0, 0.0)):
    return {"time_us": t_us, "running": running, "clock_set": True, "epoch_tow_s": tow_s, "used_pr": used[0],
            "used_rr": used[1], "rejected_pr": rej[0], "rejected_rr": rej[1], "epoch_age_s": age_s,
            "stale_epoch": False, "clock_step": False, "lat_deg": lla[0], "lon_deg": lla[1], "alt_m": lla[2],
            "vel_n": vel[0], "vel_e": vel[1], "vel_d": vel[2]}


class TestGnssUse(unittest.TestCase):
    def test_epochs_deduped_and_placed_by_gps_time(self):
        tr = truth()
        w = tr.windows(20.0)
        tow0 = tr.ign_gps_s % H.WEEK_S
        base = -100.0                                       # flight s = MCU s - 100
        recs = []
        # the pad: epochs at T-1.0, -0.9 each seen by two records; boost: one fully refused, one with nothing
        for k, te in enumerate((-1.0, -0.9)):
            for d in (0.04, 0.09):
                recs.append(raw_rec(int((te + d + 100.0) * 1e6), round(tow0 + te, 3)))
        recs.append(raw_rec(int(100.55e6), round(tow0 + 0.5, 3), used=(0, 0), rej=(3, 5)))
        recs.append(raw_rec(int(100.65e6), round(tow0 + 0.6, 3), used=(0, 0), rej=(0, 0)))
        recs.append(raw_rec(int(100.75e6), round(tow0 + 0.7, 3), used=(5, 7), rej=(1, 0)))
        g = H.gnss_use({"RawGnssEkf": recs}, base, tr, w)
        self.assertEqual(g["epochs_seen"], 5)
        pad, boost = g["phases"]["pad"], g["phases"]["boost"]
        self.assertEqual((pad["epochs"], pad["pr_used"], pad["rr_used"]), (2, 8, 12))
        self.assertEqual(pad["records"], 4)
        self.assertEqual((boost["epochs"], boost["pr_used"], boost["pr_refused"], boost["rr_used"],
                          boost["rr_refused"]), (3, 5, 4, 7, 5))
        self.assertEqual((boost["epochs_any_refused"], boost["epochs_all_refused"], boost["epochs_empty"]),
                         (2, 1, 1))
        self.assertEqual(boost["expected_10hz"], 20)
        self.assertGreater(g["map_check_s"], 0.0)
        self.assertLess(g["map_check_s"], 0.1)

    def test_week_of(self):
        ref = 2432 * H.WEEK_S + 10.0
        self.assertAlmostEqual(float(H.week_of(H.WEEK_S - 5.0, ref)), 2431 * H.WEEK_S + H.WEEK_S - 5.0)
        self.assertAlmostEqual(float(H.week_of(20.0, ref)), 2432 * H.WEEK_S + 20.0)

    def test_find_nis(self):
        self.assertEqual(H.find_nis({"RawGnssEkf": [raw_rec(0, 0.0)]}), {})
        self.assertEqual(H.find_nis({"X": [{"nis_max": 2.5, "time_us": 1}]}), {"X.nis_max": [2.5]})


class BitWriter:
    def __init__(self):
        self.bits = []

    def u(self, k, v):
        self.bits += [(int(v) >> (k - 1 - i)) & 1 for i in range(k)]

    def s(self, k, v):
        self.u(k, int(v) & ((1 << k) - 1))

    def frame(self):
        while len(self.bits) % 8:
            self.bits.append(0)
        payload = bytes(int("".join(map(str, self.bits[i:i + 8])), 2) for i in range(0, len(self.bits), 8))
        head = bytes([0xD3, (len(payload) >> 8) & 3, len(payload) & 0xFF]) + payload
        return head + XR.crc24q(head).to_bytes(3, "big")


def tr_fix_frame(gps_s, pos, vel, valid=True, nsat=29):
    """A 4001 sub-type 2 as the gnss repo's rtcm_tr_fix_encode packs it."""
    week, tow = divmod(gps_s, H.WEEK_S)
    tow_ms = round(tow * 1e3)                          # the remainder within +-0.5 ms, as the 20-bit field holds
    b = BitWriter()
    b.u(12, 4001), b.u(4, 2), b.u(16, week), b.u(30, tow_ms), b.s(20, round((tow * 1e3 - tow_ms) * 1e6))
    b.u(1, valid), b.u(1, 1), b.u(6, nsat)
    for x in pos:
        b.s(38, round(x * 1e4))
    for x in vel:
        b.s(28, round(x * 1e4))
    b.s(32, -12345), b.s(28, 2500), b.u(10, 17), b.u(16, 250)
    return b.frame()


def tr_epoch_frame(gps_s, nobs, mode=1):
    week, tow = divmod(gps_s, H.WEEK_S)
    b = BitWriter()
    b.u(12, 4001), b.u(4, 1), b.u(16, week), b.u(30, round(tow * 1e3)), b.s(20, 0), b.u(16, 0), b.s(48, 0)
    b.u(2, mode), b.u(6, nobs), b.u(48, 0)
    return b.frame()


class TestBoardStream(unittest.TestCase):
    def test_fix_decode(self):
        tr = truth()
        xyz = H.lla_to_ecef(PAD[0], PAD[1], PAD[2] + 25.0)
        f = tr_fix_frame(tr.ign_gps_s + 1.0, xyz, (1.5, -2.25, 140.0))
        self.assertTrue(XR.frame_ok(f))
        d = H.decode_tr_fix(f)
        self.assertAlmostEqual(d["gps_s"], tr.ign_gps_s + 1.0, delta=2e-6)
        np.testing.assert_allclose(d["pos"], xyz, atol=1e-4)
        np.testing.assert_allclose(d["vel"], (1.5, -2.25, 140.0), atol=1e-4)
        self.assertEqual((d["valid"], d["nsat"]), (True, 29))
        self.assertAlmostEqual(d["clk_bias_m"], -12.345)
        self.assertAlmostEqual(d["pdop"], 1.7)
        self.assertIsNone(H.decode_tr_epoch(f))

    def test_epoch_decode_and_phases(self):
        tr = truth()
        # the truth at T+1: 25 m up, climbing at 50 m/s (NED put into ECEF by the pad's rotation)
        up = H.ecef_to_ned_rot(PAD).T @ np.array([0.0, 0.0, -50.0])
        frames = [tr_epoch_frame(tr.ign_gps_s + t, n) for t, n in ((-1.0, 12), (-0.99, 3), (1.0, 20))]
        frames.append(tr_fix_frame(tr.ign_gps_s + 1.0, H.lla_to_ecef(PAD[0], PAD[1], PAD[2] + 25.0), up))
        frames.append(tr_fix_frame(tr.ign_gps_s + 1.5, H.lla_to_ecef(*PAD), up, valid=False))
        s, b = H.board_stream(frames, tr, tr.windows(20.0))
        self.assertEqual((b["fixes"], b["epochs"]), (2, 3))
        self.assertEqual((b["phases"]["pad"]["epochs"], b["phases"]["pad"]["epochs_under_4"]), (2, 1))
        self.assertEqual(b["phases"]["boost"]["fixes_valid"], 1)
        self.assertEqual(b["phases"]["boost"]["fixes"], 2)
        e = H.errors(s, tr)
        self.assertLess(e["pos_3d"][0], 1e-3)
        self.assertLess(e["vel_3d"][0], 1e-3)

    def test_read_frames_skips_the_v10s_own(self):
        tr = truth()
        own = BitWriter()
        own.u(12, 4002), own.u(4, 3), own.u(16, 0)
        good = tr_epoch_frame(tr.ign_gps_s, 10)
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "rec.rtcm3")
            Path(p).write_bytes(b"\x00junk" + good + own.frame() + good[:5] + good)
            self.assertEqual(H.read_frames(p), [good, good])


def log_records(tr, base, edge, call_t=10.205, launch_t=0.255, rate=100.0, dpos=(0.0, 0.0, 0.0)):
    """A log's records made from the truth: MCU s = flight s - base; NonSensor (ENU, 100 Hz from T-0.4 s, its
    flags from the first sample at or after launch_t and call_t) on the truth plus dpos (NED); RawGnssEkf (LLA) on
    the truth, each record 50 ms after its epoch (epochs at T+0.05 + 0.1 k); an accelerometer ramp through
    20 m/s^2 at the edge."""
    t = np.arange(-0.4, 16.0, 1.0 / rate)
    p, v = tr.at(t)
    p = p + np.array(dpos)
    us = np.rint((t - base) * 1e6).astype(int)
    ns = [{"time_us": int(u), "n_pos": float(pp[0]), "e_pos": float(pp[1]), "u_pos": float(-pp[2]),
           "n_vel": float(vv[0]), "e_vel": float(vv[1]), "u_vel": float(-vv[2]), "launch": bool(tt >= launch_t),
           "burnout": bool(tt >= 2.1), "apogee_flag": bool(tt >= call_t), "deployed": False, "alt_landed": False}
          for u, tt, pp, vv in zip(us, t, p, v)]
    ti = np.arange(-0.4, 3.0, 0.002)
    acc = np.minimum(9.81 + np.maximum(0.0, ti - (edge - 0.01)) * (20.0 - 9.81) / 0.01, 60.0)
    imu = [{"time_us": int(round((tt - base) * 1e6)), "low_acc_x": float(a), "low_acc_y": 0.0, "low_acc_z": 0.0,
            "high_acc_x": float(a), "high_acc_y": 0.0, "high_acc_z": 0.0, "low_g_near_rail": False}
           for tt, a in zip(ti, acc)]
    te = np.arange(0.05, 15.9, 0.1)
    pr, vr = tr.at(te + 0.05)
    tow0 = tr.ign_gps_s % H.WEEK_S
    raw = [raw_rec(int(round((tt + 0.05 - base) * 1e6)), round(tow0 + tt, 3), lla=(PAD[0], PAD[1], PAD[2] - pp[2]),
                   vel=tuple(vv)) for tt, pp, vv in zip(te, pr, vr)]
    return {"NonSensor": ns, "ISM6HG256": imu, "RawGnssEkf": raw}


class TestLogPath(unittest.TestCase):
    def test_time_map_errors_and_events(self):
        tr = truth()
        w = tr.windows(20.0)
        base, edge = -123.456, 0.0037
        recs = log_records(tr, base, edge, dpos=(0.0, 2.0, 0.0))
        r = H.score_run("synthetic", tr, w, log="synthetic", recs=recs, edge=edge)
        self.assertAlmostEqual(r["time"]["log"]["base_s"], base, places=5)
        self.assertIn("ignition edge", r["time"]["log"]["how"])
        self.assertEqual(r["headline"]["boost"], "log-nav")
        self.assertNotIn("pad", r["headline"])               # the log starts at T-0.4
        for ph in ("boost", "coast", "descent"):
            m = r["sources"]["log-nav"]["phases"][ph]
            self.assertAlmostEqual(m["pos_h"]["max"], 2.0, places=4)
            self.assertLess(m["vel_3d"]["max"], 1e-3)
            self.assertLess(r["sources"]["log-raw"]["phases"][ph]["pos_3d"]["max"], 0.01)
        self.assertAlmostEqual(r["apogee"]["call_t"], 10.21, places=4)
        self.assertEqual(r["apogee"]["call_from"], "the log's apogee flag (NonSensor)")
        self.assertAlmostEqual(r["events"]["log"]["launch_t"], 0.26, places=4)
        g = r["gnss_use"]
        self.assertEqual(g["phases"]["boost"]["epochs"], 20)
        self.assertEqual(g["phases"]["coast"]["epochs"], 80)
        self.assertAlmostEqual(g["map_check_s"], 0.05 - 0.03, places=4)

    def test_launch_flag_fallback(self):
        tr = truth()
        recs = log_records(tr, -50.0, 0.0)
        recs["ISM6HG256"] = []
        base, how = H.log_time_map(recs, 0.0)
        self.assertAlmostEqual(base, -50.0 - 0.26, places=4)   # the flag's sample (T+0.26) taken as ignition
        self.assertIn("COARSE", how)


class TestReport(unittest.TestCase):
    def test_side_by_side_and_plots(self):
        tr = truth()
        w = tr.windows(20.0)
        a = H.score_run("raw", tr, w, log="a", recs=log_records(tr, -10.0, 0.0, dpos=(1.0, 0.0, 0.0)), edge=0.0)
        b = H.score_run("absent", tr, w, log="b", recs=log_records(tr, -20.0, 0.0, dpos=(30.0, 0.0, 5.0)),
                        edge=0.0)
        md = H.report(tr, w, [a, b])
        self.assertIn("## Side by side: raw against absent", md)
        row = next(ln for ln in md.splitlines() if ln.startswith("| coast | position horizontal"))
        self.assertIn("| 1.0 | 30.0 | -29.0 | better |", row)
        with tempfile.TemporaryDirectory() as d:
            paths = H.plot(tr, w, [a, b], os.path.join(d, "x"))
            self.assertEqual([os.path.basename(p) for p in paths], ["x_flight.png", "x_phases.png"])
            self.assertTrue(all(os.path.getsize(p) > 10000 for p in paths))


if __name__ == "__main__":
    unittest.main()
