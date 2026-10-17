"""tools/rig_replay/score_v9.py: a V9 + mosaic-G5 rig replay's console scored against its flight's truth.

A made-up flight (a sensors CSV in sim_flight_3d.py's columns: boost, coast, drogue, landed) and a console written the
way the V9's replay image prints it -- the [REPLAY-NAV] line 10 a second in flight with the attitude turned a known
1.5 deg off the truth and the velocity 0.3 m/s east of it, the replay clock, the apogee voters with one early velocity
vote, and the 5 s [RAWGNSS], driver, PPS, [RIG], [CPU] and [LP] lines in the firmware's formats -- must come back
as those numbers: the plan's attitude error per phase, each voter against the true apogee, the run's epoch ages and
CPU, the PPS check. No real place: the pad is 0, 0.
"""
import gzip
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
for p in (REPO_ROOT / "tools" / "rig_replay", REPO_ROOT / "tools" / "raw_replay"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import score_v9  # noqa: E402

G = 9.80665
T_OFF = -200.0          # replay t = FC ms / 1000 + T_OFF
ERR_DEG = 1.5
DVE = 0.3


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def q_ypr(yaw, pitch):
    """Body FRD -> NED: yaw about down, then pitch about the body's y."""
    qz = np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
    qy = np.array([math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0])
    return qmul(qz, qy)


def flight():
    """Straight up: 50 m/s^2 for 2 s, a coast to apogee, then down at g until 10 m/s under the drogue, on the
    ground about a minute in."""
    t = np.round(np.arange(-40.0, 75.0, 0.01), 2)
    bo = 2.0
    v_bo = (50.0 - G) * bo
    t_apo = bo + v_bo / G
    h_bo = 0.5 * (50.0 - G) * bo ** 2
    h_apo = h_bo + v_bo ** 2 / (2 * G)
    t_dr = t_apo + 10.0 / G                       # 10 m/s down: the drogue holds it there
    h_dr = h_apo - 0.5 * G * (t_dr - t_apo) ** 2
    t_land = t_dr + h_dr / 10.0
    vu = np.where(t < 0, 0.0, np.where(t < bo, (50.0 - G) * t, np.where(t < t_dr, v_bo - G * (t - bo),
                                                                        np.where(t < t_land, -10.0, 0.0))))
    h = np.where(t < 0, 0.0, np.where(t < bo, 0.5 * (50.0 - G) * t ** 2,
                                      np.where(t < t_dr, h_bo + v_bo * (t - bo) - 0.5 * G * (t - bo) ** 2,
                                               np.where(t < t_land, h_dr - 10.0 * (t - t_dr), 0.0))))
    phase = np.where(t < 0, 0, np.where(t < bo, 2, np.where(t < t_apo, 3, np.where(t < t_land, 4, 6))))
    pitch = np.radians(np.where(t < t_apo, 85.0 - 95.0 * np.clip(t, 0, t_apo) / t_apo, -10.0))
    q = np.array([q_ypr(math.radians(30.0), p) for p in pitch])
    cols = {"t_s": t, "pn_m": np.zeros_like(t), "pe_m": np.zeros_like(t), "pd_m": -h, "vn_mps": np.zeros_like(t),
            "ve_mps": np.zeros_like(t), "vd_mps": -vu, "phase": phase.astype(float),
            "qw": q[:, 0], "qx": q[:, 1], "qy": q[:, 2], "qz": q[:, 3]}
    return cols, t_apo, t_land


def console(cols, t_apo, t_land):
    t, q = cols["t_s"], np.stack([cols[k] for k in ("qw", "qx", "qy", "qz")], axis=1)
    off = np.array([math.cos(math.radians(ERR_DEG) / 2), 0.0, math.sin(math.radians(ERR_DEG) / 2), 0.0])

    def ms(tt):
        return int(round((tt - T_OFF) * 1000.0))

    L = ["I (398) app_init: App version:      5dcc302f-v9-bench+20261017-1700",
         "I (2559) GNSS: mosaic-G5 set up: \"mosaic-G5 P3\" firmware 1.0.0 on COM1 at 2000000 baud (setup 1)",
         "W (2559) GNSS: BENCH: mosaic-G5 cold start (erst,Soft,PVTData+SatData) queued",
         "W (4146) SIM: RIG REPLAY: 'sim_test', 100 samples at 500 Hz",
         "I (5064) GNSS: mosaic-G5 set up: \"mosaic-G5 P3\" firmware 1.0.0 on COM1 at 2000000 baud (setup 2)",
         f"W ({ms(-130.0)}) SIM: RIG REPLAY clock: on the receiver's clock (the receiver's timed epoch at t -130.00 s)",
         f"W ({ms(-5.0)}) SIM: RIG REPLAY clock: frozen for the flight (the receiver's timed epoch at t -5.00 s)",
         f"I ({ms(-100.0)}) FC: [STATE] READY -> PRELAUNCH",
         f"I ({ms(0.05)}) FC: [STATE] PRELAUNCH -> INFLIGHT (ground_p=101325)",
         f"I ({ms(0.05)}) FC: [LAUNCH] latched by accel + baro-confirmed climb",
         f"I ({ms(2.1)}) FC: [GUID] Burnout detected at T+2050 ms",
         f"I ({ms(t_apo + 0.6)}) FC: [PYRO] Apogee detected at t={ms(t_apo + 0.6)} ms (vel=1 baro=0 gps=0 pitch=1 backstop=0)",
         f"I ({ms(t_land + 4)}) FC: [STATE] INFLIGHT -> LANDED"]
    # the 5 s lines, one on the pad and one in flight (the firmware's formats, made-up numbers)
    for tt, age in ((-20.0, 57), (5.0, 81)):
        L.append(f"I ({ms(tt)}) FC: [RAWGNSS] running (starts 1, stops 0, last 0) | imu 486/s, step 552 us (max 2168) "
                 f"| epochs 747: applied 242, no time 504, idle 0, stale {0 if tt < 0 else 3} | last: 11 meas, pr 0+0 "
                 f"rej, rr 11+0 rej, age {age} ms, build 20585 us, update 6019 us (max 43711/8588) | eph 11 sats, 0 "
                 f"tracked without | from flight filter N 0.2 E 0.3 D 0.2 m | iono none (0 days) | pad survey 2 "
                 f"settling, 0 corrected (rms 0.00 m), 0 used, raw puts the pad 0.0 m off (not vouched for) | busy "
                 f"16.7% of core 0 (wall: with preemption) | log 694, dropped 0, ring drops 0 | internal heap 230060 B "
                 f"free, min 209636 B")
        L.append(f"I ({ms(tt)}) FC: [RAWGNSS] nav: the loop read a state at most 10.2 ms old, brought forward up to "
                 f"23.9 ms from the filter's (0 read as published) | last epoch in 3 slices over 21 ms (longest slice "
                 f"5.5 ms)")
        L.append(f"I ({ms(tt)}) FC: [RAWGNSS] gnss task: built 243, not taken 0 | last build 15.7 ms (max 42.7, wall) "
                 f"| stack free: filter 1884 B, gnss 4300 B | eph dropped 0")
        L.append(f"I ({ms(tt)}) FC: [RAWGNSS] mosaic-G5: gnss_aided 1, baro_aided 1 | fused 10.0 Hz (100 ms grid), cap "
                 f"16 sats | epochs 747 seen: 504 untimed, 1395 off the grid, 0 queue drops, 0 sats capped out | "
                 f"tracked 11, 0 without an ephemeris | eph 11 sats, 11 of 128 store entries, 0 blocks refused, 0 nav "
                 f"queue drops | imu ring drops 0, gaps bridged 0 | PPS tag max 32 us, 0 over 200 of 5 judged (20 "
                 f"settling, 0 edges late) | iono none (0 sets) | pad datum the raw filter's, 483 fixes | GNSS log "
                 f"DETAILED | in flight: restarts 0 of 3, raw votes cast, transonic lockout off | flight task stack "
                 f"free 11688 B")
        L.append(f"I ({ms(tt)}) GNSS: [RIG] tracked: GPS L1CA {5 if tt < 0 else 4} (C/N0 med 37.5 max 39.5) | GAL E1 4 "
                 f"(36.0 / 36.8) | BDS B1I 2 (37.5 / 37.5) | 11 signals, 0 other")
        L.append(f"I ({ms(tt)}) FC: [CPU]  flight@1 98.6% IDLE0@0 54.0% Poll IMU Data@0 17.5% Raw GNSS@0 13.3%")
        L.append(f"I ({ms(tt)}) FC: [TIMING] ekf: avg=25 max=147 us cnt=484 | loop: cnt={968 if tt < 0 else 950}/s")
        L.append("[LP] window 5.0 s cfg 1 (pm0=instructions(100xIPC) pm1=cycles@CINTLEVEL>0) | passes 209986 = "
                 "41997.1/s, mean 23.8 us, max 6053 us | gated 4867 ekf 0 | imu 3784/s 0.1/pass | coverage 100.1 %")
        L.append("[LP]   imu_drain        41997.1       2.0       2.0      106    8.4    0.0    0.0")
    L.append(f"I ({ms(30.0)}) GNSS: mosaic raw: epochs on the 100 ms grid: 1271 handed over (504 untimed, 26 without "
             f"a PVT bias), 0 dropped, 249 empty; 2046 off the grid; tagged on the PPS line")
    L.append(f"I ({ms(30.0)}) GNSS: mosaic PPS tag check: 53 whole seconds judged, |residual| max 39 us, 0 over 200 "
             f"us, last -8 us | 20 before the line had the crystal's drift (max 441 us, not judged) | 0 edges stamped "
             f"late (the interrupt held off), up to 0 us past due: judged where due")
    L.append(f"I ({ms(30.0)}) GNSS: mosaic PPS line: MCU crystal +16.28 ppm against GPS, rms 0.002 ms; receiver delay "
             f"to the block's last byte: MeasEpoch mean 18.3 ms (max 32.8, n 600), PVTGeodetic mean 20.8 ms (max "
             f"34.4, n 600)")
    # [REPLAY-NAV]: 1 Hz on the pad, 10 Hz in flight; the replay's IMU sample 2 ms before the line
    times = list(np.arange(-60.0, 0.0, 1.0)) + list(np.arange(0.0, t_land + 3.0, 0.1))
    for tt in times:
        k = int(np.argmin(np.abs(t - tt)))
        qe = qmul(q[k], off)
        vu = -cols["vd_mps"][k]
        h = -cols["pd_m"][k]
        v_vel = int(t_apo + 0.3 <= tt or abs(tt - 5.0) < 0.05)   # one early vote at T+5, climbing
        v_baro = int(tt >= t_apo + 0.8)
        v_pitch = int(tt >= t_apo + 0.5)
        call = int(tt >= t_apo + 0.6)
        st = 3 if 0.05 <= tt < t_land + 4 else (2 if tt < 0.05 else 4)
        L.append(f"I ({ms(tt) + 2}) FC: [REPLAY-NAV] t={tt:+.3f} st={st} ekf=1 vote=1 q=({qe[0]:.5f},{qe[1]:.5f},"
                 f"{qe[2]:.5f},{qe[3]:.5f}) pos=(0.00,0.00,{h:.2f}) vel=({DVE:.3f},0.000,{vu:.3f}) baro=({h:.2f},"
                 f"{vu:+.2f}) gps_vu={vu:+.2f} votes={v_vel}{v_baro}0{v_pitch} call={call} backstop=0 "
                 f"burnout={int(tt > 2.05)} gps_admitted={int(tt > 1.0)} guard=0 mach=0")
    for tt in np.arange(-60.0, t_land + 3.0, 1.0):
        L.append(f"I ({ms(tt) + 3}) FC: [REPLAY] t={tt:+.2f} phase=0 clock=2 fixes=1300 (last t={tt:+.2f}, "
                 f"outside=0, restarts=0) ekf=1 pos=(0.0,0.0,0.0) vel=(0.00,0.00,0.00)")
    return "\n".join(L) + "\n"


class ScoreV9ConsoleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        d = Path(cls.tmp.name)
        cols, cls.t_apo, cls.t_land = flight()
        names = list(cols)
        with gzip.open(d / "sim_test_sensors.csv.gz", "wt") as f:
            f.write("# made up\n" + ",".join(names) + "\n")
            np.savetxt(f, np.stack([cols[n] for n in names], axis=1), delimiter=",", fmt="%.9g")
        (d / "scenario.json").write_text(json.dumps({"origin": {"lat_deg": 0.0, "lon_deg": 0.0, "alt_m": 0.0},
                                                     "start_utc": "2026-08-29T15:19:48", "prologue_s": 180.0,
                                                     "duration_s": 256.7}))
        run = d / "run_x"
        run.mkdir()
        (run / "fc_console.txt").write_text(console(cols, cls.t_apo, cls.t_land))
        (run / "run.json").write_text(json.dumps({"flight": "sim_test", "tag": "x", "gain": 7, "pad_s": 180.0,
                                                  "void_by_file_s": 170.0, "void": False}))
        cls.out = d / "score.json"
        score_v9.main([str(run), "--csv", str(d / "sim_test_sensors.csv.gz"), "--scenario", str(d / "scenario.json"),
                       "--json", str(cls.out)])
        cls.r = json.loads(cls.out.read_text())

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_the_attitude_error_is_the_plans_and_per_phase(self):
        for ph in ("boost", "coast", "descent"):
            a = self.r["nav"]["phases"][ph]["att"]
            self.assertGreater(a["n"], 5, ph)
            self.assertAlmostEqual(a["p90"], ERR_DEG, delta=0.02, msg=ph)
            self.assertAlmostEqual(a["max"], ERR_DEG, delta=0.02, msg=ph)

    def test_the_velocity_error_and_vz_at_apogee(self):
        for ph in ("boost", "coast", "descent"):
            self.assertAlmostEqual(self.r["nav"]["phases"][ph]["vel"]["p90"], DVE, delta=0.06, msg=ph)
        self.assertAlmostEqual(self.r["nav"]["vz_at_apogee"], 0.0, delta=0.6)

    def test_each_voter_against_the_true_apogee(self):
        v = self.r["votes"]
        self.assertAlmostEqual(v["true_apogee"], self.t_apo, delta=0.02)
        self.assertAlmostEqual(v["pitch"]["latency"], 0.5, delta=0.11)
        self.assertAlmostEqual(v["baro"]["latency"], 0.8, delta=0.11)
        self.assertIsNone(v["gps"]["first"])
        # the early velocity vote at T+5 s, climbing at about 60 m/s: G5's hard bar sees it
        self.assertEqual(v["vel"]["climbing_samples"], 1)
        self.assertAlmostEqual(v["vel"]["first_climbing"], 5.0, delta=0.06)
        self.assertEqual(v["pitch"]["climbing_samples"], 0)
        # the call, from the [PYRO] line on the replay's time
        self.assertAlmostEqual(v["call_exact"]["latency"], 0.6, delta=0.01)
        self.assertEqual(v["call_voters"]["pitch"], 1)

    def test_the_clock_and_the_run(self):
        c = self.r["clock"]
        self.assertAlmostEqual(c["armed_t"], -130.0)
        self.assertAlmostEqual(c["armed_file_s"], 50.0)
        self.assertFalse(c["void"])
        self.assertTrue(self.r["console"]["cold_start"])
        self.assertEqual(self.r["console"]["setups"], 2)
        self.assertAlmostEqual(self.r["console"]["t_offset_s"], T_OFF - 0.002, delta=0.002)

    def test_the_epochs_cpu_and_pps(self):
        tm = self.r["timing"]
        self.assertEqual(tm["epoch_age_ms"]["pad"]["median"], 57.0)
        self.assertEqual(tm["epoch_age_ms"]["flight"]["max"], 81.0)
        self.assertEqual(tm["stale_epochs_in_flight"], 3)
        self.assertEqual(tm["loop_per_s"]["flight"]["min"], 950.0)
        self.assertAlmostEqual(tm["cpu_pct"]["Raw GNSS@0"]["pad"]["median"], 13.3)
        self.assertAlmostEqual(tm["loop_profile"]["imu_drain.us_per_hit"]["pad"]["median"], 2.0)
        rx = self.r["receiver"]
        self.assertEqual(rx["pps_tag_check"]["judged"], 53)
        self.assertTrue(rx["pps_tag_check"]["pass"])
        self.assertAlmostEqual(rx["pps_line"]["crystal_ppm"], 16.28)
        self.assertEqual(rx["driver_epochs"]["untimed"], 504)
        self.assertEqual(rx["at_launch"]["eph_sats"], 11)
        self.assertEqual(rx["tracked"]["pad"]["gps"], 5.0)
        self.assertEqual(rx["tracked"]["coast"]["gps"], 4.0)

    def test_a_late_clock_voids_the_run(self):
        con = score_v9.Console.__new__(score_v9.Console)
        con.clock = [(1000, "on the receiver's clock", -8.0)]
        con.rep = None
        c = score_v9.score_clock(con, {"pad_s": 180.0, "void_by_file_s": 170.0})
        self.assertTrue(c["void"])
        con.clock = [(1000, "MISSED (the first fix came after the pad)", 3.0)]
        self.assertTrue(score_v9.score_clock(con, None)["void"])


if __name__ == "__main__":
    unittest.main()
