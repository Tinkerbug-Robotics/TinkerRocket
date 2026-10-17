#!/usr/bin/env python3
"""Score a V9 + mosaic-G5 rig replay (run_v9.sh) against its flight's truth: the console, and the OC's log if it was
downloaded, and on request the same log through the offline harness.

    score_v9.py RUN_DIR|FC_CONSOLE [--flight sim_rp5_0829 | --csv SENSORS_CSV --scenario JSON] [--blob REPLAY.bin]
                [--log OC_LOG [--harness RAW_REPLAY_MOSAIC]] [--json OUT.json]

From the console alone (a TR_SIM_REPLAY_BLOB build of the mosaic-only image prints [REPLAY-NAV] 10 a second in flight):
  - the attitude error per phase, as the plan's section 4 defines it: e = 2 acos|q_est . q_true| (deg), p90, p99 and
    max for boost (T+0 to burnout), coast (to the true apogee) and descent (to the landing); the pitch error within
    2 s of the true apogee; the nose, heading and roll parts beside it;
  - the velocity and position errors per phase (the position from the flight computer's own pad reference, so its
    "up" is relative, as the [REPLAY] line's);
  - each apogee voter (velocity, barometer, GPS, pitch), the call and the backstop: first vote, latency against the
    true apogee, the true vertical velocity then, and any vote while the rocket still climbs faster than 2 m/s
    (gate G5's hard bar); the vertical velocity estimate at the true apogee;
  - the receiver's epochs' age and build, the filter's step and busy share, [CPU] and [LP] where the image has them,
    the loop and IMU rates -- pad, flight and ground apart;
  - the bias-corrected PPS tag check, the 1PPS line's drift, the driver's untimed epochs and drops;
  - what the receiver tracked through each phase (the [RIG] C/N0 line), and the replay clock (VOID if it was not on
    the receiver's clock by file second 170, as run_v9.sh judges).
With --log, the OC's flight log too: the on-board filter's records (0x96, 10 Hz) on the same metrics, each voter's
flag (NonSensor), the epoch age the filter recorded, and the receiver's own PVT (its SBF) against the truth with its
clock drift (about +0.1 ppm on the rig) and its clock bias at ignition (how far the replayed IMU sits from the
receiver's GPS time: at most 0.5 ms). With --harness, the log through the offline harness from the on-board filter's
own start (extract.py --sbf --rig): the harness against the on-board records, and the aided filter against its unaided
twin (att_gnss_off) by the plan's section 4 bars.

Positions are only ever printed as errors; the console, the log and every intermediate stay in the run folder.
"""
import argparse
import datetime as dt
import json
import math
import os
import re
import struct
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "raw_replay"))
import scorelib as S  # noqa: E402
from truth import Truth as RigTruth, edge_t, flight_files  # noqa: E402

PHASES = ("boost", "coast", "descent")
# plan section 4: a run passes a phase if p90(aided) <= max(p90(unaided) + delta, floor)
BARS = {"boost": (1.0, 2.0), "coast": (2.0, 3.0), "descent": (3.0, 5.0)}
CLIMB_MPS = 2.0                 # G5: no vote while the true vertical velocity is above this
VOID_BY_FILE_S = 170.0          # run_v9.sh's: the replay clock on the receiver's by this file second
VOTERS = ("vel", "baro", "gps", "pitch")
LEAP_S = 18

# ------------------------------------------------------------------ the console

ANSI = re.compile(r"\x1b\[[0-9;]*m")
STAMP = re.compile(r"^[IWEDV] \((\d+)\) ([^:]+): (.*)$")
F = r"([-+]?(?:[0-9.]+|nan|inf))"
RX_NAV = re.compile(r"\[REPLAY-NAV\] t=" + F + r"(\(pad, no clock\))? st=(\d+) ekf=(\d) vote=(\d) q=\(([^)]*)\) "
                    r"pos=\(([^)]*)\) vel=\(([^)]*)\) baro=\(([^)]*)\) gps_vu=" + F + r" votes=(\d)(\d)(\d)(\d) "
                    r"call=(\d) backstop=(\d) burnout=(\d) gps_admitted=(\d) guard=(\d) mach=(\d)")
RX_REPLAY = re.compile(r"\[REPLAY\] t=" + F + r"(\(pad, no clock\))? phase=(\d) clock=(\d) fixes=(\d+) \(last t=" + F
                       + r", outside=(\d+), restarts=(\d+)\) ekf=(\d)")
RX_CLOCK = re.compile(r"RIG REPLAY clock: (.*?) \((?:the receiver's timed epoch|a fix) at t " + F + r" s\)")
RX_STATE = re.compile(r"\[STATE\] (\S+) -> (\S+)")
RX_BURNOUT = re.compile(r"\[GUID\] Burnout detected at T\+(\d+) ms")
RX_APOGEE = re.compile(r"\[PYRO\] Apogee detected at t=(\d+) ms \(vel=(\d) baro=(\d) gps=(\d) pitch=(\d) backstop=(\d)\)")
RX_ADMIT = re.compile(r"\[GNSS\] Vertical admitted to the EKF at T\+" + F + r" s: (.*)")
RX_WITHHELD = re.compile(r"\[RAWGNSS\] raw votes withheld at T\+" + F + r" s")
RX_RAW5 = re.compile(r"\[RAWGNSS\] (\w+) \(starts (\d+), stops (\d+), last (\d+)\) \| imu (\d+)/s, step (\d+) us \(max "
                     r"(\d+)\) \| epochs (\d+): applied (\d+), no time (\d+), idle (\d+), stale (\d+) \| last: (\d+) meas, "
                     r"pr (\d+)\+(\d+) rej, rr (\d+)\+(\d+) rej, age (\d+) ms, build (\d+) us, update (\d+) us \(max "
                     r"(\d+)/(\d+)\) \| eph (\d+) sats, (\d+) tracked without .*?\| iono ([^|]*?) \|.*?busy " + F
                     + r"% of core 0")
RX_NAVAGE = re.compile(r"\[RAWGNSS\] nav: the loop read a state at most " + F + r" ms old, brought forward up to " + F
                       + r" ms from the filter's \((\d+) read as published\) \| last epoch in (\d+) slices over (\d+) "
                       r"ms \(longest slice " + F + r" ms\)")
RX_TASK = re.compile(r"\[RAWGNSS\] gnss task: built (\d+), not taken (\d+) \| last build " + F + r" ms \(max " + F
                     + r", wall\) \| stack free: filter (\d+) B, gnss (\d+) B \| eph dropped (\d+)")
RX_MOSAIC = re.compile(r"\[RAWGNSS\] mosaic-G5: gnss_aided (\d+), baro_aided (\d+) \|.*?epochs (\d+) seen: (\d+) "
                       r"untimed, (\d+) off the grid, (\d+) queue drops, (\d+) sats capped out \| tracked (\d+), (\d+) "
                       r"without an ephemeris \| eph (\d+) sats.*?\| PPS tag max (\d+) us, (\d+) over 200 of (\d+) "
                       r"judged \((\d+) settling, (\d+) edges late\) \| iono ([^|]*?) \| pad datum ([^|]*?) \|.*?in "
                       r"flight: restarts (\d+) of 3, raw votes (\w+), transonic lockout (\w+)")
RX_RAWDRV = re.compile(r"mosaic raw: epochs on the (\d+) ms grid: (\d+) handed over \((\d+) untimed, (\d+) without a "
                       r"PVT bias\), (\d+) dropped, (\d+) empty")
RX_PPSCHK = re.compile(r"mosaic PPS tag check: (\d+) whole seconds judged, \|residual\| max (\d+) us, (\d+) over 200 "
                       r"us, last ([-+]?\d+) us \| (\d+) before the line had the crystal's drift \(max (\d+) us, not "
                       r"judged\) \| (\d+) edges stamped late")
RX_PPSLINE = re.compile(r"mosaic PPS line: MCU crystal " + F + r" ppm against GPS, rms " + F + r" ms; receiver delay "
                        r"to the block's last byte: MeasEpoch mean " + F + r" ms \(max " + F + r", n (\d+)\), "
                        r"PVTGeodetic mean " + F + r" ms \(max " + F)
RX_PPS = re.compile(r"mosaic PPS: (\d+) edges, (\d+) paired, (\d+) glitches; line (\w+)")
RX_LINK = re.compile(r"mosaic: " + F + r" MeasEpoch/s.*?UART overflows (\d+); relinks (\d+); nav re-sends (\d+)")
RX_RIG = re.compile(r"\[RIG\] tracked: GPS L1CA (\d+) \(C/N0 med " + F + r" max " + F + r"\) \| GAL E1 (\d+) \(" + F
                    + r" / " + F + r"\) \| BDS B1I (\d+) \(" + F + r" / " + F + r"\)")
RX_CPU = re.compile(r"\[CPU\]\s+(.*)")
RX_CPU_TASK = re.compile(r"\s*(.+?)@(\d) " + F + r"%")
RX_LP_HEAD = re.compile(r"\[LP\] window " + F + r" s cfg (\d).*?passes (\d+) = " + F + r"/s, mean " + F + r" us, max "
                        r"(\d+) us \| gated (\d+) ekf (\d+) \| imu (\d+)/s")
RX_LP_ROW = re.compile(r"\[LP\]\s+(\S+)\s+" + F + r"\s+" + F + r"\s+" + F + r"\s+(\d+)\s+" + F + r"\s+" + F + r"\s+"
                       + F + r"\s*$")
RX_TIMING = re.compile(r"\[TIMING\] ekf: (.*?) \| loop: cnt=(\d+)/s")
RX_SENSOR = re.compile(r"\[SENSOR\] reads/s: ism6=(\d+)")
RX_GAP = re.compile(r"imu_q_drops=(\d+)")
RX_VERSION = re.compile(r"App version: *(\S+)")
RX_NAME = re.compile(r"RIG REPLAY: '([^']*)'")
RX_SETUP = re.compile(r"mosaic-G5 set up: .*\(setup (\d+)\)")


def fl(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def vec(s):
    return [fl(v) for v in s.split(",")]


class Console:
    """A run's console (bench_capture_boot.py: the FC's bytes as they came), each line with the FC's log stamp (ms
    since boot; a line without one takes the last), parsed into the series below."""

    def __init__(self, path):
        raw = open(path, "rb").read().decode("utf-8", "replace")
        self.lines = []
        last = None
        for ln in raw.splitlines():
            ln = ANSI.sub("", ln).rstrip()
            m = STAMP.match(ln)
            if m:
                last = int(m.group(1))
                self.lines.append((last, m.group(3)))
            elif ln:
                self.lines.append((last, ln))
        self.parse()

    def parse(self):
        nav, rep, self.clock, self.states, self.events = [], [], [], [], []
        self.raw5, self.navage, self.task, self.mosaic, self.rawdrv = [], [], [], [], []
        self.ppschk, self.ppsline, self.pps, self.link, self.rig = [], [], [], [], []
        self.cpu, self.lp, self.timing, self.sensor, self.gap = [], [], [], [], []
        self.version = self.name = None
        self.setups = 0
        self.cold_start = False
        self.apogee = None
        self.burnout_ms = None
        lp_head = None
        for ms, msg in self.lines:
            m = RX_NAV.search(msg)
            if m:
                g = m.groups()
                nav.append([ms, fl(g[0]), g[1] is None, int(g[2]), int(g[3]), int(g[4]), *vec(g[5]), *vec(g[6]),
                            *vec(g[7]), *vec(g[8]), fl(g[9]), *[int(x) for x in g[10:20]]])
                continue
            m = RX_REPLAY.search(msg)
            if m:
                g = m.groups()
                rep.append([ms, fl(g[0]), g[1] is None, int(g[2]), int(g[3]), int(g[4]), fl(g[5]), int(g[6]),
                            int(g[7]), int(g[8])])
                continue
            for rx, out in ((RX_RAW5, self.raw5), (RX_NAVAGE, self.navage), (RX_TASK, self.task),
                            (RX_MOSAIC, self.mosaic), (RX_RAWDRV, self.rawdrv), (RX_PPSCHK, self.ppschk),
                            (RX_PPSLINE, self.ppsline), (RX_PPS, self.pps), (RX_LINK, self.link), (RX_RIG, self.rig),
                            (RX_TIMING, self.timing), (RX_SENSOR, self.sensor)):
                m = rx.search(msg)
                if m:
                    out.append((ms, m.groups()))
                    break
            else:
                m = RX_GAP.search(msg)
                if m and "[GAP DIAG]" in msg:
                    self.gap.append((ms, int(m.group(1))))
                m = RX_CPU.search(msg)
                if m:
                    self.cpu.append((ms, {f"{n.strip()}@{c}": fl(p) for n, c, p in RX_CPU_TASK.findall(m.group(1))}))
                m = RX_LP_HEAD.search(msg)
                if m:
                    lp_head = {"ms": ms, "passes_per_s": fl(m.group(4)), "imu_per_s": int(m.group(9)), "rows": {}}
                    self.lp.append(lp_head)
                m = RX_LP_ROW.search(msg)
                if m and lp_head is not None and m.group(1) not in ("section",):
                    g = m.groups()
                    lp_head["rows"][g[0]] = {"hits_per_s": fl(g[1]), "us_per_pass": fl(g[2]), "us_per_hit": fl(g[3]),
                                             "max_us": int(g[4]), "core_pct": fl(g[5])}
                m = RX_CLOCK.search(msg)
                if m:
                    self.clock.append((ms, m.group(1), fl(m.group(2))))
                m = RX_STATE.search(msg)
                if m:
                    self.states.append((ms, m.group(1), m.group(2)))
                m = RX_BURNOUT.search(msg)
                if m:
                    self.burnout_ms = (ms, int(m.group(1)))
                m = RX_APOGEE.search(msg)
                if m and self.apogee is None:
                    self.apogee = (int(m.group(1)), [int(x) for x in m.groups()[1:]])
                for rx, kind in ((RX_ADMIT, "gps admitted"), (RX_WITHHELD, "raw votes withheld")):
                    m = rx.search(msg)
                    if m:
                        self.events.append((ms, kind, fl(m.group(1)), msg))
                for key in ("[LAUNCH]", "transonic lockout", "[BARO] Judged stuck", "NO NAVIGATION FILTER",
                            "[RAWGNSS] the receiver's week", "[PADCAL]", "NOT restarted", "restarted from"):
                    if key in msg:
                        self.events.append((ms, key.strip("[]"), None, msg))
                m = RX_VERSION.search(msg)
                if m and self.version is None:
                    self.version = m.group(1)
                m = RX_NAME.search(msg)
                if m and self.name is None:
                    self.name = m.group(1)
                m = RX_SETUP.search(msg)
                if m:
                    self.setups = max(self.setups, int(m.group(1)))
                if "BENCH: mosaic-G5 cold start" in msg:
                    self.cold_start = True
        cols = ["ms", "t", "have_t", "st", "ekf", "vote", "qw", "qx", "qy", "qz", "pe", "pn", "pu", "ve", "vn", "vu",
                "baro_alt", "baro_rate", "gps_vu", "v_vel", "v_baro", "v_gps", "v_pitch", "call", "backstop",
                "burnout", "gps_admitted", "guard", "mach"]
        self.nav = {c: np.array([r[i] for r in nav], float) for i, c in enumerate(cols)} if nav else None
        rc = ["ms", "t", "have_t", "phase", "clock", "fixes", "last_fix_t", "outside", "restarts", "ekf"]
        self.rep = {c: np.array([r[i] for r in rep], float) for i, c in enumerate(rc)} if rep else None
        self.t_off = self.time_offset()

    def time_offset(self):
        """Replay time = FC ms / 1000 + this: the samples' offset once the clock is on the receiver's (after the
        freeze both run on the MCU's crystal, so one offset holds; the log stamp is a few ms after the sample)."""
        ts, ms = [], []
        for d in (self.nav, self.rep):
            if d is not None:
                m = d["have_t"] > 0
                ts.append(d["t"][m])
                ms.append(d["ms"][m])
        if not ts or not np.concatenate(ts).size:
            return None
        t, m = np.concatenate(ts), np.concatenate(ms)
        late = t >= -5.0
        sel = late if late.sum() >= 5 else np.ones_like(late)
        return float(np.median(t[sel] - m[sel] * 1e-3))

    def t_of_ms(self, ms):
        return None if ms is None or self.t_off is None else ms * 1e-3 + self.t_off

    def launch_ms(self):
        for ms, a, b in self.states:
            if b == "INFLIGHT":
                return ms
        return None


# ------------------------------------------------------------------ statistics

def pstats(x, pcts=(90, 99)):
    x = np.abs(np.asarray(x, float))
    x = x[np.isfinite(x)]
    if not x.size:
        return None
    return {"n": int(x.size), **{f"p{p}": float(np.percentile(x, p)) for p in pcts}, "max": float(x.max()),
            "median": float(np.median(x))}


def phase_windows(st):
    """The plan's phases from the truth (scorelib.Truth): boost T+0 to burnout, coast to the true apogee, descent
    to the landing."""
    return [("boost", 0.0, st.burnout), ("coast", st.burnout, st.apogee), ("descent", st.apogee, st.landed)]


def f3(x, nd=2):
    """scorelib's (median, p90, max)."""
    return "-" if x is None or not all(np.isfinite(x)) else " / ".join(f"{v:.{nd}f}" for v in x)


def jsonable(o):
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def fmt(d, keys=("p90", "p99", "max"), nd=2):
    if not d:
        return "-"
    return " / ".join(f"{d[k]:.{nd}f}" for k in keys)


# ------------------------------------------------------------------ scoring

def score_nav(con, st, rt):
    """The [REPLAY-NAV] state against the truth, per phase."""
    n = con.nav
    if n is None:
        return {"error": "no [REPLAY-NAV] lines: not a V9 mosaic replay image (TR_RAW_NAV_RIG_REPLAY)"}
    m = (n["have_t"] > 0) & np.isfinite(n["qw"])
    t = n["t"][m]
    q = np.stack([n["qw"], n["qx"], n["qy"], n["qz"]], axis=1)[m]
    pos_t, vel_t, q_t = st.at(t)
    total, nose, roll, heading, pitch = S.att_errors(q, q_t)
    enu_p = np.stack([n["pe"], n["pn"], n["pu"]], axis=1)[m]
    enu_v = np.stack([n["ve"], n["vn"], n["vu"]], axis=1)[m]
    true_p = np.stack([pos_t[:, 1], pos_t[:, 0], -pos_t[:, 2]], axis=1)
    true_v = np.stack([vel_t[:, 1], vel_t[:, 0], -vel_t[:, 2]], axis=1)
    ep, ev = enu_p - true_p, enu_v - true_v
    out = {"samples": int(m.sum()), "phases": {}}
    for name, lo, hi in phase_windows(st):
        w = (t >= lo) & (t < hi)
        out["phases"][name] = {
            "from": lo, "to": hi, "n": int(w.sum()),
            "att": pstats(total[w]), "nose": pstats(nose[w]), "heading": pstats(heading[w]), "roll": pstats(roll[w]),
            "pitch": pstats(pitch[w]), "vel": pstats(np.linalg.norm(ev[w], axis=1)), "vz": pstats(ev[w, 2]),
            "pos_h": pstats(np.hypot(ep[w, 0], ep[w, 1])), "pos_v": pstats(ep[w, 2]),
            "unhealthy": int((n["ekf"][m][w] < 1).sum()),
        }
    w = np.abs(t - st.apogee) <= 2.0
    out["pitch_near_apogee"] = pstats(pitch[w])
    if t.size >= 2 and t[0] <= st.apogee <= t[-1]:
        out["vz_at_apogee"] = float(np.interp(st.apogee, t, enu_v[:, 2]))   # the truth's is 0
    pad = (t >= -30.0) & (t < 0.0)
    out["pad"] = {"att": pstats(total[pad]), "heading": pstats(heading[pad]), "vel": pstats(np.linalg.norm(ev[pad], axis=1))}
    return out


def first_rise(t, on):
    k = np.flatnonzero(on)
    return float(t[k[0]]) if k.size else None


def intervals(t, on):
    out, start = [], None
    for ti, o in zip(t, on):
        if o and start is None:
            start = ti
        elif not o and start is not None:
            out.append((float(start), float(ti)))
            start = None
    if start is not None:
        out.append((float(start), None))
    return out


def score_votes(t, flags, st, true_vu, call_t=None):
    """Each voter's first vote, its latency against the true apogee and the true vertical velocity then; any vote
    while the rocket climbs faster than CLIMB_MPS (G5's hard bar)."""
    out = {"true_apogee": st.apogee}
    for k, on in flags.items():
        on = np.asarray(on, bool)
        fr = first_rise(t, on)
        climb = on & (true_vu > CLIMB_MPS) & (t > 0.0)
        out[k] = {"first": fr, "latency": None if fr is None else fr - st.apogee,
                  "true_vu_at_first": None if fr is None else float(np.interp(fr, st.t, -st.vel[:, 2])),
                  "on": intervals(t, on)[:6], "climbing_samples": int(climb.sum()),
                  "first_climbing": first_rise(t, climb)}
    if call_t is not None:
        out["call_exact"] = {"t": call_t, "latency": call_t - st.apogee,
                             "true_vu": float(np.interp(call_t, st.t, -st.vel[:, 2]))}
    return out


def console_votes(con, st):
    n = con.nav
    if n is None:
        return None
    m = n["have_t"] > 0
    t = n["t"][m]
    true_vu = np.interp(t, st.t, -st.vel[:, 2])
    flags = {"vel": n["v_vel"][m], "baro": n["v_baro"][m], "gps": n["v_gps"][m], "pitch": n["v_pitch"][m],
             "call": n["call"][m], "backstop": n["backstop"][m], "guard": n["guard"][m], "mach": n["mach"][m],
             "gps_admitted": n["gps_admitted"][m]}
    call_t = con.t_of_ms(con.apogee[0]) if con.apogee else None
    out = score_votes(t, flags, st, true_vu, call_t)
    if con.apogee:
        out["call_voters"] = dict(zip(("vel", "baro", "gps", "pitch", "backstop"), con.apogee[1]))
    return out


def windows_of(con, st, items):
    """Split (ms, value) items into pad (t < 0), flight (T+0 to landing) and ground, by the replay's time."""
    w = {"pad": [], "flight": [], "ground": []}
    for ms, v in items:
        t = con.t_of_ms(ms)
        if t is None:
            continue
        w["pad" if t < 0 else ("flight" if t < st.landed else "ground")].append(v)
    return w


def med_max(vals):
    v = np.asarray([x for x in vals if x is not None and np.isfinite(x)], float)
    return None if not v.size else {"n": int(v.size), "median": float(np.median(v)), "max": float(v.max()),
                                    "min": float(v.min())}


def score_timing(con, st):
    out = {}
    for name, items in (
            ("epoch_age_ms", [(ms, fl(g[17])) for ms, g in con.raw5 if g[0] == "running"]),
            ("epoch_build_us", [(ms, fl(g[18])) for ms, g in con.raw5 if g[0] == "running"]),
            ("epoch_build_max_us", [(ms, fl(g[20])) for ms, g in con.raw5 if g[0] == "running"]),
            ("update_max_us", [(ms, fl(g[21])) for ms, g in con.raw5 if g[0] == "running"]),
            ("filter_step_max_us", [(ms, fl(g[6])) for ms, g in con.raw5 if g[0] == "running"]),
            ("filter_busy_pct", [(ms, fl(g[25])) for ms, g in con.raw5 if g[0] == "running"]),
            ("nav_read_age_ms", [(ms, fl(g[0])) for ms, g in con.navage]),
            ("nav_forward_ms", [(ms, fl(g[1])) for ms, g in con.navage]),
            ("epoch_slices", [(ms, fl(g[3])) for ms, g in con.navage]),
            ("gnss_task_build_max_ms", [(ms, fl(g[3])) for ms, g in con.task]),
            ("loop_per_s", [(ms, fl(g[1])) for ms, g in con.timing]),
            ("ism6_per_s", [(ms, fl(g[0])) for ms, g in con.sensor]),
    ):
        out[name] = {k: med_max(v) for k, v in windows_of(con, st, items).items()}
    # counters: their growth over the flight window (stale epochs, not taken, read as published)
    def growth(items):
        w = [(con.t_of_ms(ms), v) for ms, v in items if con.t_of_ms(ms) is not None]
        a = [v for t, v in w if t < 0]
        b = [v for t, v in w if 0 <= t < st.landed + 10]
        return None if not a or not b else b[-1] - a[-1]
    out["stale_epochs_in_flight"] = growth([(ms, int(g[11])) for ms, g in con.raw5])
    out["not_taken_in_flight"] = growth([(ms, int(g[1])) for ms, g in con.task])
    out["read_as_published_in_flight"] = growth([(ms, int(g[2])) for ms, g in con.navage])
    out["imu_queue_drops_in_flight"] = growth(con.gap)
    out["filter_stops"] = int(con.raw5[-1][1][2]) if con.raw5 else None
    # [CPU] (an image with FreeRTOS run-time stats) and [LP] (TR_LOOP_PROFILE)
    if con.cpu:
        tasks = sorted({k for _ms, d in con.cpu for k in d})
        out["cpu_pct"] = {k: {w: med_max(v) for w, v in windows_of(con, st, [(ms, d.get(k)) for ms, d in con.cpu]).items()}
                          for k in tasks}
    if con.lp:
        rows = {}
        for sec in ("imu_drain", "sensors", "ekf_extract", "kinematics", "sim"):
            for fld in ("us_per_hit", "core_pct"):
                items = [(h["ms"], h["rows"].get(sec, {}).get(fld)) for h in con.lp]
                rows[f"{sec}.{fld}"] = {w: med_max(v) for w, v in windows_of(con, st, items).items()}
        rows["imu_per_s"] = {w: med_max(v) for w, v in windows_of(con, st, [(h["ms"], h["imu_per_s"]) for h in con.lp]).items()}
        rows["passes_per_s"] = {w: med_max(v) for w, v in
                                windows_of(con, st, [(h["ms"], h["passes_per_s"]) for h in con.lp]).items()}
        out["loop_profile"] = rows
    return out


def score_receiver(con, st):
    out = {}
    if con.ppschk:
        g = con.ppschk[-1][1]
        out["pps_tag_check"] = {"judged": int(g[0]), "max_us": int(g[1]), "over_200us": int(g[2]), "last_us": int(g[3]),
                                "settling": int(g[4]), "settling_max_us": int(g[5]), "late_edges": int(g[6]),
                                "pass": int(g[2]) == 0}
    if con.ppsline:
        g = con.ppsline[-1][1]
        out["pps_line"] = {"crystal_ppm": fl(g[0]), "rms_ms": fl(g[1]), "meas_delay_mean_ms": fl(g[2]),
                           "meas_delay_max_ms": fl(g[3]), "pvt_delay_mean_ms": fl(g[5]), "pvt_delay_max_ms": fl(g[6])}
    if con.pps:
        g = con.pps[-1][1]
        out["pps"] = {"edges": int(g[0]), "paired": int(g[1]), "glitches": int(g[2]), "line": g[3]}
    if con.rawdrv:
        g = con.rawdrv[-1][1]
        out["driver_epochs"] = {"grid_ms": int(g[0]), "handed_over": int(g[1]), "untimed": int(g[2]),
                                "no_pvt_bias": int(g[3]), "dropped": int(g[4]), "empty": int(g[5])}
    if con.link:
        g = con.link[-1][1]
        out["link"] = {"meas_per_s": fl(g[0]), "uart_overflows": int(g[1]), "relinks": int(g[2]), "nav_resends": int(g[3])}
    # the filter's view at launch: the last 5-s line before T+0
    pre = [(ms, g) for ms, g in con.mosaic if con.t_of_ms(ms) is not None and con.t_of_ms(ms) < 0]
    last = pre[-1][1] if pre else (con.mosaic[-1][1] if con.mosaic else None)
    if last is not None:
        out["at_launch"] = {"gnss_aided": int(last[0]), "baro_aided": int(last[1]), "untimed": int(last[3]),
                            "tracked": int(last[7]), "without_eph": int(last[8]), "eph_sats": int(last[9]),
                            "iono": last[15].strip(), "pad_datum": last[16].strip()}
    if con.mosaic:
        g = con.mosaic[-1][1]
        out["end"] = {"restarts": int(g[17]), "raw_votes": g[18], "transonic_lockout": g[19]}
    # what the receiver tracked through each phase ([RIG] line, every 5 s)
    if con.rig:
        ph = [("pad", -1e9, 0.0)] + [(n, lo, hi) for n, lo, hi in RIG_PHASES(st)] + [("ground", st.landed, 1e9)]
        rows = {}
        for name, lo, hi in ph:
            sel = [g for ms, g in con.rig if con.t_of_ms(ms) is not None and lo <= con.t_of_ms(ms) < hi]
            if not sel:
                continue
            a = np.array([[fl(x) for x in g] for g in sel])
            rows[name] = {"lines": len(sel), "gps": float(np.median(a[:, 0])), "gps_cn0_med": float(np.median(a[:, 1])),
                          "gps_cn0_max": float(a[:, 2].max()), "gal": float(np.median(a[:, 3])),
                          "gal_cn0_med": float(np.median(a[:, 4])), "bds": float(np.median(a[:, 6])),
                          "bds_cn0_med": float(np.median(a[:, 7]))}
        out["tracked"] = rows
    return out


def RIG_PHASES(st):
    """The flight's phases for what the receiver tracked: boost, coast, under the drogue, under the main."""
    e = st.edges
    out = [("boost", 0.0, st.burnout), ("coast", st.burnout, e.get("drogue", st.apogee))]
    if "drogue" in e:
        out.append(("drogue", e["drogue"], e.get("main", st.landed)))
    if "main" in e:
        out.append(("main", e["main"], st.landed))
    return out


def score_clock(con, run):
    out = {"transitions": [{"t_fc_s": None if ms is None else ms * 1e-3, "state": s, "at_t": t} for ms, s, t in con.clock]}
    armed = [t for _ms, s, t in con.clock if s.startswith("on the receiver's clock")]
    missed = any(s.startswith("MISSED") for _ms, s, _t in con.clock)
    out["armed_t"] = armed[0] if armed else None
    pad_s = float(run.get("pad_s", 180.0)) if run else 180.0
    by = float(run.get("void_by_file_s", VOID_BY_FILE_S)) if run else VOID_BY_FILE_S
    out["armed_file_s"] = None if not armed else armed[0] + pad_s
    out["void"] = bool(missed or not armed or armed[0] + pad_s > by)
    out["void_why"] = ("MISSED" if missed else "no clock" if not armed else
                       f"armed at file second {armed[0] + pad_s:.1f}" if armed[0] + pad_s > by else "")
    if con.rep is not None:
        out["fixes"] = int(con.rep["fixes"][-1])
        out["outside"] = int(con.rep["outside"][-1])
        out["restarts"] = int(con.rep["restarts"][-1])
    if run and run.get("void"):
        out["void"] = True
        out["void_why"] = out["void_why"] or run.get("void_why", "")
    return out


def score_events(con, st):
    """The flight computer's own events on the replay's time."""
    out = []
    lms = con.launch_ms()
    for ms, a, b in con.states:
        out.append({"t": con.t_of_ms(ms), "what": f"{a} -> {b}"})
    if con.burnout_ms and lms is not None:
        out.append({"t": con.t_of_ms(lms + con.burnout_ms[1]), "what": "burnout (GUID)"})
    if con.apogee:
        v = con.apogee[1]
        out.append({"t": con.t_of_ms(con.apogee[0]), "what": "apogee called (vel %d baro %d gps %d pitch %d backstop %d)" % tuple(v)})
    for ms, kind, tplus, _msg in con.events:
        t = (con.t_of_ms(lms) + tplus) if (tplus is not None and lms is not None) else con.t_of_ms(ms)
        out.append({"t": t, "what": kind if kind != "PADCAL" else _msg[_msg.find("[PADCAL]"):][:120]})
    out.sort(key=lambda e: (e["t"] is None, e["t"] if e["t"] is not None else 0.0))
    truth = [{"t": 0.0, "what": "TRUTH ignition"}, {"t": st.burnout, "what": "TRUTH burnout"},
             {"t": st.apogee, "what": "TRUTH apogee"}, {"t": st.landed, "what": "TRUTH landed"}]
    return sorted(out + truth, key=lambda e: (e["t"] is None, e["t"] if e["t"] is not None else 0.0))


# ------------------------------------------------------------------ the OC log

def sbf_pvt(recs):
    """PVTGeodetic from the log's SBF (0x95): GPS s, mode, error, lat, lon (rad), height (ellipsoidal), undulation,
    vN vE vU, clock bias (ms), drift (ppm)."""
    import plot_flight_data_mini as P
    frames, _dropped = P.join_gnss_rtcm(recs.get("GNSS_RTCM") or [])
    sbf = P.split_gnss_raw_frames(frames)["sbf"]
    out = []
    for _tu, b in sbf:
        b = bytes(b)
        if P.sbf_block_number(b) != 4007 or len(b) < 75:
            continue
        tow, wnc = struct.unpack_from("<IH", b, 8)
        mode, err = b[14], b[15]
        lat, lon, h = struct.unpack_from("<ddd", b, 16)
        und, vn, ve, vu = struct.unpack_from("<ffff", b, 40)
        bias, = struct.unpack_from("<d", b, 60)
        drift, = struct.unpack_from("<f", b, 68)
        if tow == 0xFFFFFFFF or wnc == 0xFFFF:
            continue
        out.append((wnc * 604800.0 + tow * 1e-3, mode & 0x0F, err, lat, lon, h, und, vn, ve, vu, bias, drift))
    return np.array(out) if out else None


def gps_seconds(utc):
    return (utc + dt.timedelta(seconds=LEAP_S) - dt.datetime(1980, 1, 6)).total_seconds()


def score_log(log, st, rt, edge):
    sys.path.insert(0, os.path.join(S.WT, "Data_Analysis"))
    import plot_flight_data_mini as P
    from truth import log_time_base, unwrap_us
    recs, _stats, _cfg = P.parse_binary_file(log)
    base = log_time_base(recs, edge)
    out = {"time_base_s": base, "records": {k: len(v) for k, v in recs.items() if v}}
    # the on-board filter's records (0x96), the plan's metrics
    if recs.get("RawGnssEkf"):
        ob = S.onboard_series(recs, base)
        out["onboard"] = phase_metrics(ob, st)
        ages = np.array([r["epoch_age_s"] for r in recs["RawGnssEkf"]], float) * 1e3
        rt_ = ob["t"]
        for name, lo, hi in [("pad", -1e9, 0.0), ("flight", 0.0, st.landed)]:
            w = (rt_ >= lo) & (rt_ < hi) & np.isfinite(ages) & (ages > 0)
            out.setdefault("epoch_age_ms", {})[name] = None if not w.any() else {
                "median": float(np.median(ages[w])), "p95": float(np.percentile(ages[w], 95)), "max": float(ages[w].max())}
    # each voter's flag, the loop's own (NonSensor)
    ns = recs.get("NonSensor") or []
    if ns:
        nt = unwrap_us([r["time_us"] for r in ns]) * 1e-6 + base
        flags = {"vel": [r["vel_apogee"] for r in ns], "baro": [r["alt_apogee"] for r in ns],
                 "gps": [r["gps_apogee"] for r in ns], "pitch": [r["pitch_apogee"] for r in ns],
                 "call": [r["apogee_flag"] for r in ns], "burnout": [r["burnout"] for r in ns]}
        out["votes"] = score_votes(nt, flags, st, np.interp(nt, st.t, -st.vel[:, 2]))
    # the receiver's own PVT, against the truth
    pvt = sbf_pvt(recs)
    if pvt is not None and rt.scenario:
        ign = gps_seconds(rt.ignition_utc())
        t = pvt[:, 0] - ign
        fix = (pvt[:, 1] > 0) & (pvt[:, 2] == 0) & (np.abs(pvt[:, 3]) < 4) & (np.abs(pvt[:, 7]) < 1e4)
        res = {"pvt": int(len(t)), "fixes": int(fix.sum())}
        if fix.any():
            p = rt.ned_from_lla(np.degrees(pvt[fix, 3]), np.degrees(pvt[fix, 4]), pvt[fix, 5])
            v = np.stack([pvt[fix, 7], pvt[fix, 8], -pvt[fix, 9]], axis=1)   # NED
            tp, tv, _q = st.at(t[fix])
            ep, ev = p - tp, v - tv
            res["phases"] = {}
            for name, lo, hi in [("pad", -60.0, 0.0)] + phase_windows(st):
                w = (t[fix] >= lo) & (t[fix] < hi)
                res["phases"][name] = {"n": int(w.sum()), "pos_h": pstats(np.hypot(ep[w, 0], ep[w, 1])),
                                       "pos_v": pstats(ep[w, 2]), "vel": pstats(np.linalg.norm(ev[w], axis=1)),
                                       "vu": pstats(ev[w, 2])}
            res["clock_drift_ppm_median"] = float(np.median(pvt[fix, 11]))
            k = np.argmin(np.abs(t[fix]))
            res["clock_bias_ms_at_ignition"] = float(pvt[fix, 10][k])   # the replayed IMU's offset from GPS time
        out["receiver"] = res
    return out, recs, base


def phase_metrics(series, st):
    """The plan's metrics of a series (S.onboard_series / S.harness_series) per phase, p90/p99/max."""
    t = series["t"]
    m = series["ok"] & (t >= 0.0) & (t <= st.landed)
    pt, vt, qt = st.at(t[m])
    total, nose, roll, heading, pitch = S.att_errors(series["q"][m], qt)
    ev = series["vel"][m] - vt
    ep = series["pos"][m] - pt
    tt = t[m]
    out = {}
    for name, lo, hi in phase_windows(st):
        w = (tt >= lo) & (tt < hi)
        out[name] = {"n": int(w.sum()), "att": pstats(total[w]), "nose": pstats(nose[w]), "heading": pstats(heading[w]),
                     "roll": pstats(roll[w]), "pitch": pstats(pitch[w]), "vel": pstats(np.linalg.norm(ev[w], axis=1)),
                     "vz": pstats(ev[w, 2]), "pos_h": pstats(np.hypot(ep[w, 0], ep[w, 1])), "pos_v": pstats(ep[w, 2])}
    w = np.abs(tt - st.apogee) <= 2.0
    out["pitch_near_apogee"] = pstats(pitch[w])
    out["unhealthy_samples"] = int((~series["ok"][(t >= 0.0) & (t <= st.landed)]).sum())
    return out


def run_harness(binary, log, blob, outdir, recs, base, st):
    """The log through the offline harness from the on-board filter's own start: aided (as flown) and its unaided
    twin (att_gnss_off), each with the four-voter quorum."""
    os.makedirs(outdir, exist_ok=True)
    ev = os.path.join(outdir, "onboard.events")
    if not os.path.exists(ev):   # written once per log (about a minute); delete it to extract again
        ext = [sys.executable, os.path.join(HERE, "..", "raw_replay", "extract.py"), "--sbf", "--rig", log, ev]
        if blob:
            ext += ["--blob", blob]
        p = subprocess.run(ext, capture_output=True, text=True)
        if p.returncode != 0:
            raise SystemExit(f"extract.py --sbf --rig failed: {p.stderr.strip().splitlines()[-1:]}")
    meta = S.read_meta(ev)
    hbase = float(meta["t_base_s"])
    res = {"t_base_s": hbase, "log_time_base_s": base}
    series = {}
    for arm, extra in (("aided", []), ("unaided", ["--unaided"])):
        csv = os.path.join(outdir, f"harness_{arm}.csv")
        p = subprocess.run([binary, "--in", ev, "--out", csv, "--receiver", "mosaic", "--quorum",
                            "--t-base-s", repr(hbase)] + extra, capture_output=True, text=True)
        open(csv + ".stderr", "w").write(p.stderr)
        if p.returncode not in (0, 3):
            raise SystemExit(f"raw_replay ({arm}) failed ({p.returncode}): {p.stderr.strip().splitlines()[-1:]}")
        series[arm] = S.harness_series(csv, hbase)
        res[arm] = phase_metrics(series[arm], st)
        q = series[arm].get("events", {}).get("quorum") or {}
        res[arm]["quorum"] = {k: q.get(k) for k in ("launch_t", "burnout_t", "gnss_admitted_t", "guard_latch_t",
                                                    "apogee_t", "backstop_t") if k in q}
        res[arm]["quorum"]["at_call"] = q.get("at_call")
        res[arm]["exit"] = p.returncode
    # the plan's bar per phase: p90(aided) <= max(p90(unaided) + delta, floor)
    res["acceptance"] = {}
    for ph in PHASES:
        a = (res["aided"].get(ph) or {}).get("att")
        u = (res["unaided"].get(ph) or {}).get("att")
        if a and u:
            d, floor = BARS[ph]
            bar = max(u["p90"] + d, floor)
            res["acceptance"][ph] = {"aided_p90": a["p90"], "unaided_p90": u["p90"], "bar": bar,
                                     "pass": a["p90"] <= bar}
    # the twins alike to the last digit: no GNSS reached the attitude (no ephemeris, no time, a dead sky)
    res["twins_identical"] = all(
        (res["aided"].get(ph) or {}).get("att") == (res["unaided"].get(ph) or {}).get("att") for ph in PHASES)
    if recs.get("RawGnssEkf"):
        ob = S.onboard_series(recs, base)
        res["harness_less_onboard"] = S.diff_vs(series["aided"], ob, st)
    return res


# ------------------------------------------------------------------ the report

def report(r):
    L = []
    w = L.append
    meta = r.get("run") or {}
    w(f"# Rig replay {meta.get('tag', '')}: {r['flight']}  (image {r['console'].get('version')}, carrying "
      f"'{r['console'].get('flight')}', gain {meta.get('gain', '?')})")
    c = r["clock"]
    w(f"replay clock: {'on the receiver at t %+.2f s (file second %.1f)' % (c['armed_t'], c['armed_file_s']) if c['armed_t'] is not None else 'never'}"
      f"; fixes {c.get('fixes')} outside {c.get('outside')} restarts {c.get('restarts')}"
      f"{'  ** VOID: ' + c['void_why'] + ' **' if c['void'] else ''}")
    w(f"cold start: {'yes' if r['console']['cold_start'] else 'NO'}, receiver set up {r['console']['setups']} times")
    w("\n## Events (replay time, s from ignition)")
    for e in r["events"]:
        w(f"  {e['t']:+9.3f}  {e['what']}" if e["t"] is not None else f"      ?     {e['what']}")
    nav = r.get("nav") or {}
    if "phases" in nav:
        w("\n## Navigation state against the truth ([REPLAY-NAV]; p90 / p99 / max)")
        w("  phase      n   attitude e (deg)      nose (deg)          heading (deg)       velocity (m/s)     vz (m/s)")
        for ph in PHASES:
            p = nav["phases"][ph]
            w(f"  {ph:8s} {p['n']:4d}   {fmt(p['att'])}   {fmt(p['nose'])}   {fmt(p['heading'])}   "
              f"{fmt(p['vel'])}   {fmt(p['vz'])}")
        w(f"  pitch error within 2 s of the true apogee: {fmt(nav.get('pitch_near_apogee'))} deg; "
          f"vz at the true apogee {nav.get('vz_at_apogee', float('nan')):+.2f} m/s; pad attitude "
          f"{fmt((nav.get('pad') or {}).get('att'))}, heading {fmt((nav.get('pad') or {}).get('heading'))}")
    v = r.get("votes")
    if v:
        w(f"\n## Apogee voters (console, 10 Hz; the true apogee T+{v['true_apogee']:.2f})")
        for k in VOTERS + ("call", "backstop", "guard", "mach"):
            x = v.get(k)
            if not x:
                continue
            fr = "never" if x["first"] is None else f"T+{x['first']:.2f} ({x['latency']:+.2f} s, true vz {x['true_vu_at_first']:+.2f})"
            w(f"  {k:9s} first {fr}; votes while climbing > {CLIMB_MPS:g} m/s: {x['climbing_samples']}"
              f"{'' if x['first_climbing'] is None else ' (first T+%.2f)' % x['first_climbing']}")
        if v.get("call_exact"):
            ce = v["call_exact"]
            w(f"  call (exact, [PYRO]) T+{ce['t']:.3f}: {ce['latency']:+.3f} s from the true apogee, true vz "
              f"{ce['true_vu']:+.2f} m/s; voters {v.get('call_voters')}")
    tm = r.get("timing") or {}
    if tm:
        w("\n## Epochs and CPU (median / max per window: pad, flight, ground)")
        for k in ("epoch_age_ms", "epoch_build_us", "epoch_build_max_us", "update_max_us", "filter_step_max_us",
                  "filter_busy_pct", "nav_read_age_ms", "nav_forward_ms", "gnss_task_build_max_ms", "loop_per_s",
                  "ism6_per_s"):
            row = tm.get(k) or {}
            cells = []
            for wnd in ("pad", "flight", "ground"):
                x = row.get(wnd)
                cells.append("-" if not x else f"{x['median']:.1f}/{x['max']:.1f}")
            w(f"  {k:24s} " + "   ".join(f"{c:>14s}" for c in cells))
        w(f"  in flight: stale epochs +{tm.get('stale_epochs_in_flight')}, not taken +{tm.get('not_taken_in_flight')}, "
          f"read as published +{tm.get('read_as_published_in_flight')}, IMU queue drops +{tm.get('imu_queue_drops_in_flight')};"
          f" filter stops {tm.get('filter_stops')}")
        for k, row in (tm.get("cpu_pct") or {}).items():
            cells = ["-" if not row.get(wnd) else f"{row[wnd]['median']:.1f}/{row[wnd]['max']:.1f}" for wnd in ("pad", "flight", "ground")]
            w(f"  [CPU] {k:20s} " + "   ".join(f"{c:>14s}" for c in cells))
        for k, row in (tm.get("loop_profile") or {}).items():
            cells = ["-" if not row.get(wnd) else f"{row[wnd]['median']:.1f}/{row[wnd]['max']:.1f}" for wnd in ("pad", "flight", "ground")]
            w(f"  [LP] {k:21s} " + "   ".join(f"{c:>14s}" for c in cells))
    rx = r.get("receiver") or {}
    if rx:
        w("\n## Receiver and its time")
        for k in ("pps_tag_check", "pps_line", "pps", "driver_epochs", "link", "at_launch", "end"):
            if k in rx:
                w(f"  {k}: {rx[k]}")
        for ph, x in (rx.get("tracked") or {}).items():
            w(f"  tracked {ph:7s}: GPS {x['gps']:.0f} (C/N0 {x['gps_cn0_med']:.1f}, max {x['gps_cn0_max']:.1f}), "
              f"GAL {x['gal']:.0f} ({x['gal_cn0_med']:.1f}), BDS {x['bds']:.0f} ({x['bds_cn0_med']:.1f}) over "
              f"{x['lines']} lines")
    lg = r.get("log")
    if lg:
        w(f"\n## OC log (time base {lg['time_base_s']:+.4f} s)")
        if "onboard" in lg:
            w("  on-board 0x96   phase    n   attitude e p90/p99/max   velocity   vz")
            for ph in PHASES:
                p = lg["onboard"][ph]
                w(f"                  {ph:8s} {p['n']:4d}   {fmt(p['att'])}   {fmt(p['vel'])}   {fmt(p['vz'])}")
            w(f"  epoch age (0x96): {lg.get('epoch_age_ms')}")
        if "votes" in lg:
            for k in VOTERS + ("call",):
                x = lg["votes"][k]
                fr = "never" if x["first"] is None else f"T+{x['first']:.3f} ({x['latency']:+.3f} s, true vz {x['true_vu_at_first']:+.2f})"
                w(f"  {k:6s} (NonSensor) first {fr}; while climbing: {x['climbing_samples']}")
        if "receiver" in lg:
            rr = lg["receiver"]
            w(f"  receiver PVT: {rr['fixes']} fixes of {rr['pvt']}; clock drift {rr.get('clock_drift_ppm_median', float('nan')):+.3f} ppm, "
              f"clock bias at ignition {rr.get('clock_bias_ms_at_ignition', float('nan')):+.3f} ms (the replayed IMU's offset from GPS time)")
            for ph, x in (rr.get("phases") or {}).items():
                w(f"    {ph:8s} n {x['n']:5d}  pos h {fmt(x['pos_h'])} m  v {fmt(x['pos_v'])} m  vel {fmt(x['vel'])} m/s")
    hz = r.get("harness")
    if hz:
        w("\n## Offline harness from the on-board start (aided = as flown; unaided = att_gnss_off)")
        for ph in PHASES:
            a, u = hz["aided"][ph], hz["unaided"][ph]
            acc = hz["acceptance"].get(ph)
            w(f"  {ph:8s} aided {fmt(a['att'])}  unaided {fmt(u['att'])}  -> "
              f"{'PASS' if acc and acc['pass'] else 'FAIL' if acc else '-'}"
              f"{'' if not acc else ' (bar %.2f)' % acc['bar']}")
        if hz.get("twins_identical"):
            w("  !! aided and unaided are identical: no GNSS reached the attitude in the harness (its stderr says why)")
        for arm in ("aided", "unaided"):
            w(f"  quorum {arm}: {hz[arm]['quorum']}")
        if "harness_less_onboard" in hz:
            for ph, m in hz["harness_less_onboard"].items():
                w(f"  harness - on-board {ph:8s} (median / p90 / max): attitude {f3(m['att_total'])} deg, "
                  f"velocity {f3(m['vel'])} m/s, vz {f3(m['vz'])} m/s")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run", help="the run folder (run_v9.sh) or its fc_console.txt")
    ap.add_argument("--flight", help="sim_<id>, its files under $COCOM_SDR/scenarios (default: the run's)")
    ap.add_argument("--csv", help="the sensors CSV, if not the flight's")
    ap.add_argument("--scenario", help="the scenario JSON, if not the flight's")
    ap.add_argument("--blob", help="the replay file the image carried (the log's time base; default: the run's)")
    ap.add_argument("--log", help="the OC's flight log of this run (.bin), if downloaded")
    ap.add_argument("--harness", help="raw_replay_mosaic (tools/raw_replay's build): the log through the offline harness")
    ap.add_argument("--json", help="write the scores here (default: score_v9.json in the run folder)")
    args = ap.parse_args(argv)

    run_dir = args.run if os.path.isdir(args.run) else os.path.dirname(os.path.abspath(args.run))
    console = os.path.join(args.run, "fc_console.txt") if os.path.isdir(args.run) else args.run
    run = json.load(open(os.path.join(run_dir, "run.json"))) if os.path.exists(os.path.join(run_dir, "run.json")) else None
    con = Console(console)
    flight = args.flight or (run or {}).get("flight") or con.name
    if not flight and not args.csv:
        raise SystemExit("which flight? --flight sim_<id>")
    csv, scen = flight_files(flight) if flight else (None, None)
    csv, scen = args.csv or csv, args.scenario or scen
    st = S.Truth(csv)
    st.path = csv
    rt = RigTruth(csv, scen)
    st.edges = rt.edges
    blob = args.blob
    if not blob and run and run.get("image_dir") and run.get("blob"):
        cand = os.path.join(run["image_dir"], run["blob"])
        blob = cand if os.path.exists(cand) else None
    r = {"flight": flight, "run": run,
         "console": {"version": con.version, "flight": con.name, "cold_start": con.cold_start, "setups": con.setups,
                     "t_offset_s": con.t_off},
         "clock": score_clock(con, run), "events": score_events(con, st)}
    if con.name and flight and flight.replace("sim_", "") not in con.name:
        r["warning"] = f"the image carried '{con.name}', not {flight}"
    r["nav"] = score_nav(con, st, rt)
    r["votes"] = console_votes(con, st)
    r["timing"] = score_timing(con, st)
    r["receiver"] = score_receiver(con, st)
    if args.log:
        edge = edge_t(csv, scen, blob)
        r["log"], recs, base = score_log(args.log, st, rt, edge)
        if args.harness:
            r["harness"] = run_harness(args.harness, args.log, blob, os.path.join(run_dir, "harness"), recs, base, st)
    print(report(r))
    if r.get("warning"):
        print("\n!! " + r["warning"])
    out = args.json or os.path.join(run_dir, "score_v9.json")
    with open(out, "w") as f:
        json.dump(r, f, indent=1, default=jsonable)
    print(f"\nscores: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
