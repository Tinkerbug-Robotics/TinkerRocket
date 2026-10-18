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


def console(cols, t_apo, t_land, early_in_samples=True, pps=((-20.0, 0, 5), (5.0, 0, 30)),
            iono_at_launch="none (0 sets)", extra=(), untimed=None, raw_votes="cast", time_tag=False):
    """The console; `pps` the 5 s lines' (t, over, judged) PPS tag check counts and `untimed` their untimed epochs,
    `extra` lines as (t, text); `time_tag`: the driver's lines as an image with the time-tag fix prints them."""
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
    for tt, text in extra:
        L.append(f"I ({ms(tt)}) {text}")
    # the 5 s lines, one on the pad and one in flight (the firmware's formats, made-up numbers)
    for (tt, over, judged), age, unt in zip(pps, (57, 81, 81), untimed or (504,) * len(pps)):
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
                 f"16 sats | epochs 747 seen: {unt} untimed, 1395 off the grid, 0 queue drops, 0 sats capped out | "
                 f"tracked 11, 0 without an ephemeris | eph 11 sats, 11 of 128 store entries, 0 blocks refused, 0 nav "
                 f"queue drops | imu ring drops 0, gaps bridged 0 | PPS tag max 32 us, {over} over 200 of {judged} "
                 f"judged (20 settling, 0 edges late) | iono {iono_at_launch} | pad datum the raw filter's, 483 fixes "
                 f"| GNSS log "
                 f"DETAILED | in flight: restarts 0 of 3, raw votes {raw_votes}, transonic lockout off | dv check: "
                 f"0 epochs checked, 0 failed, latches 0 let go 0, 0 Dopplers held | heading share 0.00 | rotation "
                 f"allowance max 0.00 m/s | flight task stack free 11688 B")
        L.append(f"I ({ms(tt)}) GNSS: [RIG] tracked: GPS L1CA {5 if tt < 0 else 4} (C/N0 med 37.5 max 39.5) | GAL E1 4 "
                 f"(36.0 / 36.8) | BDS B1I 2 (37.5 / 37.5) | 11 signals, 0 other")
        L.append(f"I ({ms(tt)}) FC: [CPU]  flight@1 98.6% IDLE0@0 54.0% Poll IMU Data@0 17.5% Raw GNSS@0 13.3%")
        L.append(f"I ({ms(tt)}) FC: [TIMING] ekf: avg=25 max=147 us cnt=484 | loop: cnt={968 if tt < 0 else 950}/s")
        L.append("[LP] window 5.0 s cfg 1 (pm0=instructions(100xIPC) pm1=cycles@CINTLEVEL>0) | passes 209986 = "
                 "41997.1/s, mean 23.8 us, max 6053 us | gated 4867 ekf 0 | imu 3784/s 0.1/pass | coverage 100.1 %")
        L.append("[LP]   imu_drain        41997.1       2.0       2.0      106    8.4    0.0    0.0")
    if time_tag:
        L.append(f"I ({ms(30.0)}) GNSS: mosaic raw: epochs on the 100 ms grid: 1271 handed over (504 untimed: no line "
                 f"300, not FINETIME 120, no edge 0, no bias 26, edge off the line 2, edge off the chain 0, held 56), "
                 f"0 dropped, 249 empty; 2046 off the grid; last whole second timed | nav queued/filtered/dropped: GPS "
                 f"40/0/0, GAL 30/2/0, BDS 20/1/0; too long 0, short buffer 0 (queue 64 in PSRAM) | GPSIon 0 (0 "
                 f"refused) | clock jumps 0, restarts 1 (clock lost 0; jumps placed past 256 ms 0), PVTs not carried 0")
        L.append(f"I ({ms(30.0)}) GNSS: mosaic PPS tag check: 53 whole seconds judged, |residual| max 39 us, 0 over 200 "
                 f"us, last -8 us | 20 before the line had the crystal's drift (max 441 us, not judged) | 0 edges "
                 f"stamped late (the interrupt held off), up to 0 us past due: judged where due | 7 held back "
                 f"(checked, not judged: max 6855 us, last -12 us)")
    else:
        L.append(f"I ({ms(30.0)}) GNSS: mosaic raw: epochs on the 100 ms grid: 1271 handed over (504 untimed, 26 "
                 f"without a PVT bias), 0 dropped, 249 empty; 2046 off the grid; tagged on the PPS line")
        L.append(f"I ({ms(30.0)}) GNSS: mosaic PPS tag check: 53 whole seconds judged, |residual| max 39 us, 0 over 200 "
                 f"us, last -8 us | 20 before the line had the crystal's drift (max 441 us, not judged) | 0 edges "
                 f"stamped late (the interrupt held off), up to 0 us past due: judged where due")
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
        v_vel = int(t_apo + 0.3 <= tt or (early_in_samples and abs(tt - 5.0) < 0.05))   # one early vote at T+5
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



def vote_edges(t_apo):
    """[REPLAY-VOTE] lines: a velocity vote of 6 ms at T+5.003, between two 10 Hz samples, then the votes as the
    samples have them (the call, the pitch, the barometer after the true apogee)."""
    def line(tt, vel, baro, pitch, call):
        return (tt, f"FC: [REPLAY-VOTE] t={tt:+.4f} st=3 votes={vel}{baro}0{pitch} call={call} backstop=0 burnout=1 "
                    f"gps_admitted=1 guard=0 mach=0 vote=1 unprinted=0")
    return [line(5.003, 1, 0, 0, 0), line(5.009, 0, 0, 0, 0), line(t_apo + 0.3, 1, 0, 0, 0),
            line(t_apo + 0.5, 1, 0, 1, 0), line(t_apo + 0.6, 1, 0, 1, 1), line(t_apo + 0.8, 1, 1, 1, 1)]


def score_run(d, name, text, cols_files, run_extra=None, hackrf=None, argv=()):
    run = d / name
    run.mkdir()
    (run / "fc_console.txt").write_text(text)
    meta = {"flight": "sim_test", "tag": name, "gain": 7, "pad_s": 180.0, "void_by_file_s": 170.0, "void": False}
    meta.update(run_extra or {})
    (run / "run.json").write_text(json.dumps(meta))
    if hackrf is not None:
        (run / "hackrf.txt").write_text(hackrf)
    out = d / f"{name}.json"
    csv, scen = cols_files
    score_v9.main([str(run), "--csv", str(csv), "--scenario", str(scen), "--json", str(out), *argv])
    return json.loads(out.read_text())


def hackrf_log(underrun_at_file_s=None, seconds=257):
    """hackrf_tx_ram -B's per-second lines for a whole file at 36.96 MB/s, with one 6.86 ms underrun in the second
    that ends at file second `underrun_at_file_s`."""
    L = ["hackrf_tx_ram: signalsim_real_sim_test.C8, 9.5 GB; ring 2048 MiB (wired), 512 MiB read ahead in 0.30 s"]
    under = longest = 0
    for k in range(1, seconds + 1):
        if underrun_at_file_s is not None and k == int(underrun_at_file_s):
            under, longest = 1, 253312
        L.append(f"37.0 MB / 1.000 sec = 37.0 MB/second, average power -11.2 dBfs, 19584 bytes filled in buffer, "
                 f"{under} underruns, longest {longest} bytes, ring 63 MiB ahead, host shortfalls 0, slowest callback "
                 f"0.40 ms")
    L.append(f"hackrf_tx_ram: sent {seconds * 0.037:.3f} GB of {seconds * 0.037:.3f} GB, host shortfalls 0")
    return "\n".join(L) + "\n"


class ScoreV9RunRulesTest(unittest.TestCase):
    """What makes a run count: the [REPLAY-VOTE] edges for G5, the freeze's check and the 1PPS tag check after it,
    the transmitter's slips, the ionosphere."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.d = Path(cls.tmp.name)
        cls.cols, cls.t_apo, cls.t_land = flight()
        names = list(cls.cols)
        csv = cls.d / "sim_test_sensors.csv.gz"
        with gzip.open(csv, "wt") as f:
            f.write("# made up\n" + ",".join(names) + "\n")
            np.savetxt(f, np.stack([cls.cols[n] for n in names], axis=1), delimiter=",", fmt="%.9g")
        scen = cls.d / "scenario.json"
        scen.write_text(json.dumps({"origin": {"lat_deg": 0.0, "lon_deg": 0.0, "alt_m": 0.0},
                                    "start_utc": "2026-08-29T15:19:48", "prologue_s": 180.0, "duration_s": 256.7}))
        cls.files = (csv, scen)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def checked(self, residual, moved, judged, over, held=None):
        what = f"MOVED onto the edge by {residual:+d}" if moved else f"kept, the edge within {residual:+d}"
        tail = "" if held is None else f"; {held} held back (untimed, not judged)"
        return (-4.9, f"SIM: RIG REPLAY clock: the freeze checked against the 1PPS edge of the second at t -5 s: "
                      f"residual {residual:+d} us, {what} us; PPS tag check so far {judged} judged, {over} over 200 "
                      f"us{tail}")

    def test_a_six_ms_vote_between_two_samples_fails_g5_from_the_edges(self):
        text = console(self.cols, self.t_apo, self.t_land, early_in_samples=False, extra=vote_edges(self.t_apo))
        r = score_run(self.d, "edges", text, self.files)
        v = r["votes"]
        self.assertTrue(v["from"].startswith("the [REPLAY-VOTE]"))
        self.assertTrue(v["complete"])
        self.assertAlmostEqual(v["vel"]["first"], 5.003, delta=1e-6)
        self.assertEqual(v["vel"]["climbing_samples"], 1)
        self.assertFalse(v["g5_hard"]["pass"])
        # the latencies to the loop's instant, not to a 10 Hz sample (the truth's apogee is a 10 ms sample's)
        self.assertAlmostEqual(v["pitch"]["latency"], 0.5, delta=0.011)
        self.assertAlmostEqual(v["baro"]["latency"] - v["pitch"]["latency"], 0.3, delta=1e-4)
        # the samples alone never see that vote: from them G5 is not judged at all
        r2 = score_run(self.d, "samples", console(self.cols, self.t_apo, self.t_land, early_in_samples=False),
                       self.files)
        self.assertEqual(r2["votes"]["vel"]["climbing_samples"], 0)
        self.assertFalse(r2["votes"]["g5_judged"])
        self.assertNotIn("g5_hard", r2["votes"])

    def test_a_second_over_200_us_after_the_freeze_voids_the_run_and_nothing_is_scored(self):
        text = console(self.cols, self.t_apo, self.t_land, pps=((-20.0, 0, 20), (5.0, 3, 40)),
                       extra=[self.checked(-6856, True, 30, 0)])
        r = score_run(self.d, "ppsvoid", text, self.files)
        self.assertTrue(r["clock"]["void"])
        self.assertIn("1PPS tag check judged 3 second(s) over 200 us", r["clock"]["void_why"])
        self.assertTrue(r["sky_time"]["pps_tag_check"]["freeze_check"]["moved"])
        self.assertFalse(r["scored"])
        self.assertNotIn("nav", r)
        self.assertNotIn("votes", r)
        # --score-void: scored, for diagnosis only
        r2 = score_run(self.d, "ppsvoid2", text, self.files, argv=["--score-void"])
        self.assertTrue(r2["scored"] and r2["clock"]["void"])
        self.assertIn("nav", r2)

    def test_a_moved_freeze_with_a_clean_check_after_it_counts(self):
        # the line had caught up by the freeze's next second: overs only before the freeze
        text = console(self.cols, self.t_apo, self.t_land, pps=((-20.0, 4, 20), (5.0, 4, 40)),
                       extra=[self.checked(-6856, True, 30, 4)])
        r = score_run(self.d, "moved", text, self.files)
        self.assertFalse(r["clock"]["void"])
        self.assertEqual(r["sky_time"]["pps_tag_check"]["over_after_freeze"], 0)
        self.assertEqual(r["sky_time"]["pps_tag_check"]["judged_after_freeze"], 10)
        self.assertTrue(any("MOVED -6856 us" in m for m in r["sky_time"]["marks"]))
        self.assertIn("nav", r)

    def test_without_the_check_line_the_seconds_around_the_freeze_are_judged(self):
        # an over at t -5.5 s, in the 5 s line after it: the last line a second before the freeze is the base
        text = console(self.cols, self.t_apo, self.t_land, pps=((-20.0, 0, 20), (5.0, 1, 40)))
        r = score_run(self.d, "nocheck", text, self.files)
        self.assertTrue(r["clock"]["void"])
        self.assertTrue(any("no freeze-check line" in m for m in r["sky_time"]["marks"]))

    def test_an_image_with_the_time_tag_fix_is_read_whole(self):
        # the driver's lines in the time-tag fix's formats; a hold in flight (26 epochs untimed between the freeze
        # and the landing) and the log-only vote guard latched: each a mark or an event, the run still counts
        guard = (8.0, "FC: [RAWGNSS] raw vote guard latched at T+8.00 s: the filter's vertical velocity +30.0 m/s "
                      "against the barometer's +10.0 m/s for 1 s -- log only: its votes are still cast "
                      "(config::RAW_VOTE_GUARD_ACTS)")
        text = console(self.cols, self.t_apo, self.t_land, pps=((-20.0, 0, 20), (5.0, 0, 30), (self.t_land + 5, 0, 70)),
                       untimed=(504, 510, 530), raw_votes="cast (guard latched, log only)", time_tag=True,
                       extra=[self.checked(-12, False, 18, 0, held=7), guard])
        r = score_run(self.d, "timetag", text, self.files)
        self.assertFalse(r["clock"]["void"])
        fc = r["sky_time"]["pps_tag_check"]
        self.assertEqual(fc["freeze_check"]["held"], 7)
        self.assertFalse(fc["freeze_check"]["moved"])
        self.assertEqual(fc["over_after_freeze"], 0)
        self.assertEqual(fc["untimed_after_freeze"], 26)
        self.assertTrue(any("26 epoch(s) went to the filter untimed" in m for m in r["sky_time"]["marks"]))
        rx = r["receiver"]
        self.assertEqual(rx["driver_epochs"]["untimed"], 504)
        self.assertEqual(rx["driver_epochs"]["untimed_why"]["held"], 56)
        self.assertEqual(rx["driver_epochs"]["untimed_why"]["edge_off_the_line"], 2)
        self.assertEqual(rx["driver_epochs"]["no_pvt_bias"], 26)
        self.assertEqual(rx["driver_epochs"]["dropped"], 0)
        self.assertEqual(rx["driver_epochs"]["empty"], 249)
        self.assertEqual(rx["pps_tag_check"]["held_back"], 7)
        self.assertEqual(rx["pps_tag_check"]["held_back_max_us"], 6855)
        self.assertEqual(rx["end"]["raw_votes"], "cast (guard latched, log only)")
        self.assertTrue(any(e["what"] == "raw vote guard latched (log only)" and abs(e["t"] - 8.0) < 0.1
                            for e in r["events"]))
        # the lines before the fix still read as they did
        r2 = score_run(self.d, "oldtag", console(self.cols, self.t_apo, self.t_land), self.files)
        self.assertEqual(r2["receiver"]["driver_epochs"]["no_pvt_bias"], 26)
        self.assertNotIn("untimed_why", r2["receiver"]["driver_epochs"])
        self.assertNotIn("held_back", r2["receiver"]["pps_tag_check"])

    def test_a_transmitter_underrun_after_the_freeze_voids_the_run(self):
        text = console(self.cols, self.t_apo, self.t_land)
        r = score_run(self.d, "txvoid", text, self.files, hackrf=hackrf_log(190))
        self.assertTrue(r["clock"]["void"])
        self.assertEqual(r["sky_time"]["transmitter"]["verdict"], "VOID")
        self.assertIn("after the replay clock froze", r["clock"]["void_why"])
        self.assertFalse(r["scored"])
        # before the clock: absorbed, the run counts
        r2 = score_run(self.d, "txearly", text, self.files, hackrf=hackrf_log(20))
        self.assertFalse(r2["clock"]["void"])
        self.assertEqual(r2["sky_time"]["transmitter"]["verdict"], "clean")

    def test_a_second_boot_before_the_landing_voids_the_run(self):
        text = console(self.cols, self.t_apo, self.t_land)
        lines = text.splitlines()
        # the FC crashes at T+20: its console starts over from the ROM banner, ms from 0
        cut = next(i for i, ln in enumerate(lines) if "[REPLAY-NAV] t=+20.000" in ln)
        rebooted = lines[:cut] + ["ESP-ROM:esp32p4-eco2-20240710", "I (398) app_init: App version:      x"] + lines[cut:]
        r = score_run(self.d, "reboot", "ESP-ROM:esp32p4-eco2-20240710\n" + "\n".join(rebooted) + "\n", self.files)
        self.assertTrue(r["clock"]["void"])
        self.assertIn("booted again", r["clock"]["void_why"])
        self.assertAlmostEqual(r["sky_time"]["reboot_t"], 19.9, delta=0.2)

    def test_the_ionosphere_in_use_is_flagged_when_it_is_not_the_skys(self):
        sky = (-170.0, "SIM: RIG REPLAY ionosphere: the rig sky's own GPS Klobuchar (14 3 -1 -2 / 51 6 -2 -5, GPS "
                       "day 17037) stands in for the GPSIon its file never broadcasts: the filter takes it as the "
                       "receiver's")
        r = score_run(self.d, "iono_ok", console(self.cols, self.t_apo, self.t_land,
                                                 iono_at_launch="the receiver's GPSIon (1 sets)", extra=[sky]),
                      self.files)
        self.assertTrue(r["ionosphere"]["sky_set"])
        self.assertNotIn("flag", r["ionosphere"])
        r2 = score_run(self.d, "iono_none", console(self.cols, self.t_apo, self.t_land, extra=[sky]), self.files)
        self.assertIn("none", r2["ionosphere"]["flag"])
        r3 = score_run(self.d, "iono_old", console(self.cols, self.t_apo, self.t_land), self.files)
        self.assertIn("before the RIG REPLAY ionosphere line", r3["ionosphere"]["flag"])

if __name__ == "__main__":
    unittest.main()
