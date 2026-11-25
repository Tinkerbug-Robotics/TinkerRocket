#!/usr/bin/env python3
"""Score a hardware-in-the-loop run on the RF rig: the V10's navigation solution against the scene's truth.

In a HIL run the V10 runs a rig-replay image. It plays the scene's own IMU, barometer and magnetometer at their GPS
times (signalsim_sensors.py made them from the scene's config), while the HackRF plays the same scene's signals to
the gnss-sdr-dev board, whose measurements the V10's navigation filter takes. The truth is the scene's SignalSim
config: a vertical trajectoryList (a Const pad, then VerticalAcc segments) from its initPosition, the pad.

What it scores, by phase (pad, boost to burnout, coast to apogee, descent to the scene's end):
  - position (3D, horizontal, vertical) and velocity (3D, horizontal, vertical) error: median, 90th percentile,
    largest;
  - the apogee: the flight computer's call against the true apogee's time, and the solution's highest point
    against the true apogee's altitude;
  - the filter's use of GNSS (the raw-GNSS filter's records, 0x96): epochs, pseudoranges and range rates fused and
    refused; NIS wherever a log carries it;
  - the board's own fixes against the truth, and its epochs' observables (--rtcm, or the log's 0x95 records);
  - with --ref-*: a second run (a GNSS-absent one) side by side.

The sources, each scored on its own:
  log-nav      the navigation state the flight computer logs (NonSensor: ENU from its own pad reference)
  log-raw      the raw-GNSS filter's records (0x96: ellipsoidal, scored from the scene's pad)
  console-nav  the console's [REPLAY-NAV] lines (a raw-nav rig image: 50 Hz in boost, 10 Hz after, 1 Hz on the pad)
  console      the console's [REPLAY] lines (once a second, every replay image; ENU from the filter's pad reference)
  board-fix    the board's own fixes (4001 sub-type 2: ECEF, scored from the scene's pad)
Each phase's headline row takes the first of log-nav, console-nav, console that covers half of the phase: the OC
log starts at launch, so the pad comes from the console.

Time. The console's replay lines carry the replay's flight time (t=); its other lines go on it by their stamps.
The log's MCU clock goes on it by the ignition edge in its accelerometer, as score_log.py does (the replay file's
edge: --blob, else the file sim_replay_blob.py makes from the config). The board's frames go on it by their GPS
time against the scene's ignition.

    score_hil.py SCENE.json --log V10.bin --console fc_console.log [--rtcm rec.rtcm3] [--blob REPLAY.bin]
        [--ref-console REF_console.log] [--ref-log REF.bin] [--label L --ref-label L] [--out-dir DIR --name NAME]
"""
import argparse
import datetime as dt
import json
import math
import os
import re
import sys
from dataclasses import dataclass

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "..", "Data_Analysis"))
import signalsim_sensors as ss  # noqa: E402

GPS_EPOCH = dt.datetime(1980, 1, 6)
LEAP_S = 18                                  # GPS - UTC since 2017
WEEK_S = 604800.0
A_WGS, E2_WGS = 6378137.0, 6.69437999014e-3
PAD_S = 60.0                                 # the pad's window: this long before ignition
COVER = 0.5                                  # a headline source covers this much of a phase
HEADLINE = ("log-nav", "console-nav", "console")
METRICS = ("pos_3d", "pos_h", "pos_v", "vel_3d", "vel_h", "vel_v")
METRIC_NAMES = {"pos_3d": "position 3D (m)", "pos_h": "position horizontal (m)", "pos_v": "position vertical (m)",
                "vel_3d": "velocity 3D (m/s)", "vel_h": "velocity horizontal (m/s)",
                "vel_v": "velocity vertical (m/s)"}
TR_MESSAGE, FC_MESSAGE = 4001, 4002


# ------------------------------------------------------------------ the truth

class SceneTruth:
    """A vertical SignalSim scene's truth on flight time (s from ignition): NED from its pad, its phases' edges, and
    the ignition's GPS time."""

    def __init__(self, cfg):
        self.tr = ss.Trajectory(cfg)
        ign, burn, apo = self.tr.phases()
        self.name = cfg["trajectory"].get("name", "signalsim")
        self.pad = self.tr.pad
        self.ign_file_s = ign
        self.burnout = None if burn is None else burn - ign
        self.apogee = None if apo is None else apo - ign
        self.end = self.tr.duration - ign
        self.apogee_m = None if apo is None else float(self.tr.at([apo])[0][0] - self.pad[2])
        h_end, v_end, _ = self.tr.at([self.tr.duration])
        self.end_m, self.end_vu = float(h_end[0] - self.pad[2]), float(v_end[0])
        c = cfg["time"]
        utc = dt.datetime(c["year"], c["month"], c["day"], c["hour"], c["minute"], int(c["second"]))
        self.ign_gps_s = (utc - GPS_EPOCH).total_seconds() + LEAP_S + ign

    def at(self, t):
        """NED position and velocity (n x 3 each) at flight times t; NaN outside the scene's file."""
        t = np.asarray(t, float)
        h, v, _ = self.tr.at(t + self.ign_file_s)
        z = np.zeros_like(t)
        pos = np.stack([z, z, -(h - self.pad[2])], axis=1)
        vel = np.stack([z, z, -v], axis=1)
        out = (t > self.end) | (t < -self.ign_file_s)
        pos[out] = np.nan
        vel[out] = np.nan
        return pos, vel

    def windows(self, pad_s=PAD_S):
        """(phase, from, to) in flight s: the pad, boost, coast and descent (to the scene's end)."""
        w = [("pad", -min(pad_s, self.ign_file_s), 0.0)]
        if self.burnout is None:
            return w
        w.append(("boost", 0.0, self.burnout))
        if self.apogee is None:
            return w + [("coast", self.burnout, self.end)]
        return w + [("coast", self.burnout, self.apogee), ("descent", self.apogee, self.end)]


class CsvTruth(SceneTruth):
    """A sim_flight_3d.py flight (its sensors CSV and scenario JSON, truth.py's Truth) behind SceneTruth's
    interface: the descent runs from the drogue to the landing."""

    def __init__(self, csv_path, scenario_path):
        import truth as rig_truth
        self.ft = rig_truth.Truth(csv_path, scenario_path)
        e, s = self.ft.edges, self.ft.scenario
        self.name = s.get("scenario", os.path.basename(csv_path))
        self.pad = self.ft.pad()
        self.ign_file_s = float(s["prologue_s"])
        self.burnout = e.get("coast")
        self.apogee = e.get("drogue")
        self.end = e.get("landed", self.ft.end)
        x = self.ft.at([self.apogee])[0] if self.apogee is not None else None
        self.apogee_m = None if x is None else float(-x[2])
        x = self.ft.at([self.end])[0]
        self.end_m, self.end_vu = float(-x[2]), float(-x[5])
        self.ign_gps_s = (self.ft.ignition_utc() - GPS_EPOCH).total_seconds() + LEAP_S

    def at(self, t):
        t = np.asarray(t, float)
        x = self.ft.at(t)
        pos, vel = x[:, 0:3].copy(), x[:, 3:6].copy()
        out = (t > self.end) | (t < self.ft.t[0])
        pos[out] = np.nan
        vel[out] = np.nan
        return pos, vel


# ------------------------------------------------------------------ geodesy

def lla_to_ecef(lat_deg, lon_deg, h):
    lat, lon = np.radians(np.asarray(lat_deg, float)), np.radians(np.asarray(lon_deg, float))
    h = np.asarray(h, float)
    n = A_WGS / np.sqrt(1 - E2_WGS * np.sin(lat) ** 2)
    return np.stack([(n + h) * np.cos(lat) * np.cos(lon), (n + h) * np.cos(lat) * np.sin(lon),
                     (n * (1 - E2_WGS) + h) * np.sin(lat)], axis=-1)


def ecef_to_ned_rot(pad):
    """The rotation taking ECEF vectors to NED at the pad (lat, lon deg)."""
    la, lo = math.radians(pad[0]), math.radians(pad[1])
    return np.array([[-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
                     [-math.sin(lo), math.cos(lo), 0.0],
                     [-math.cos(la) * math.cos(lo), -math.cos(la) * math.sin(lo), -math.sin(la)]])


def ecef_to_ned(xyz, pad):
    """NED (m) from the pad (lat, lon, h) of ECEF positions (n x 3)."""
    return (np.asarray(xyz, float) - lla_to_ecef(*pad)) @ ecef_to_ned_rot(pad).T


def lla_to_ned(lat, lon, h, pad):
    return ecef_to_ned(lla_to_ecef(lat, lon, h), pad)


# ------------------------------------------------------------------ scoring

@dataclass
class Series:
    """A navigation solution on flight time: NED position and velocity."""
    name: str
    t: np.ndarray
    pos: np.ndarray
    vel: np.ndarray
    frame: str                  # "the filter's pad reference" or "the scene's pad"


def stats(x):
    """Median, 90th percentile and largest of |x| (NaN left out); None for no samples."""
    x = np.abs(np.asarray(x, float))
    x = x[np.isfinite(x)]
    if not x.size:
        return None
    return {"n": int(x.size), "median": float(np.median(x)), "p90": float(np.percentile(x, 90)),
            "max": float(x.max())}


def errors(s, truth):
    """Each metric's error series at s's own times; vertical ones signed (up positive)."""
    p, v = truth.at(s.t)
    ep, ev = s.pos - p, s.vel - v
    return {"pos_3d": np.linalg.norm(ep, axis=1), "pos_h": np.hypot(ep[:, 0], ep[:, 1]), "pos_v": -ep[:, 2],
            "vel_3d": np.linalg.norm(ev, axis=1), "vel_h": np.hypot(ev[:, 0], ev[:, 1]), "vel_v": -ev[:, 2]}


def score_phase(s, err, lo, hi):
    """One source over one window: its samples, how much of the window they cover, the largest gap, each metric's
    stats."""
    m = (s.t >= lo) & (s.t < hi) & np.isfinite(err["pos_3d"])
    t = s.t[m]
    out = {"n": int(m.sum()), "cover": 0.0, "max_gap_s": float(hi - lo)}
    if not t.size:
        return out
    out["cover"] = float(min(1.0, (t.max() - t.min() + np.median(np.diff(t)) if t.size > 1 else 0.0) / (hi - lo)))
    out["max_gap_s"] = float(np.diff(np.concatenate([[lo], t, [hi]])).max())
    out.update({k: stats(err[k][m]) for k in METRICS})
    return out


def score_series(s, truth, windows):
    err = errors(s, truth)
    return {ph: score_phase(s, err, lo, hi) for ph, lo, hi in windows}


def headline(scored, windows):
    """Each phase's source: the first of HEADLINE scored there that covers COVER of it."""
    out = {}
    for ph, _lo, _hi in windows:
        for name in HEADLINE:
            r = scored.get(name, {}).get(ph)
            if r and r["n"] and r["cover"] >= COVER:
                out[ph] = name
                break
    return out


def apogee_scores(truth, call_t=None, s=None):
    """The flight computer's apogee call against the true apogee's time, and the solution's highest point (its own
    height, from its frame's origin) against the true apogee's height."""
    out = {"truth_t": truth.apogee, "truth_m": truth.apogee_m}
    if truth.apogee is None:
        return out
    if call_t is not None:
        p, v = truth.at([call_t])
        out.update(call_t=float(call_t), call_dt=float(call_t - truth.apogee), call_true_m=float(-p[0, 2]),
                   call_true_vu=float(-v[0, 2]))
    if s is not None:
        m = (s.t >= (truth.burnout or 0.0)) & (s.t <= truth.end)
        if m.any():
            up = -s.pos[m, 2]
            k = int(np.argmax(up))
            out.update(peak_t=float(s.t[m][k]), peak_m=float(up[k]), peak_dt=float(s.t[m][k] - truth.apogee),
                       peak_dm=float(up[k] - truth.apogee_m), peak_source=s.name)
            if s.t[m].min() <= truth.apogee <= s.t[m].max():
                at = float(np.interp(truth.apogee, s.t[m], up))
                out.update(at_truth_m=at, at_truth_dm=at - truth.apogee_m)
            if call_t is not None and s.t[m].min() <= call_t <= s.t[m].max():
                out["call_est_m"] = float(np.interp(call_t, s.t[m], up))
    return out


# ------------------------------------------------------------------ the console

STAMP = re.compile(r"([IWEDV]) \((\d+)\) ([\w-]+): ")
F = r"([-+]?(?:[0-9.]+|nan|inf))"
RX_REPLAY = re.compile(r"\[REPLAY\] t=" + F + r"(\(pad, no clock\))? phase=(\d) clock=(\d) fixes=(\d+) .*?ekf=(\d) "
                       r"pos=\(([^)]*)\) vel=\(([^)]*)\)")
RX_NAV = re.compile(r"\[REPLAY-NAV\] t=" + F + r"(\(pad, no clock\))? st=(\d+) ekf=(\d) .*?pos=\(([^)]*)\) "
                    r"vel=\(([^)]*)\)")
RX_APOGEE = re.compile(r"\[PYRO\] Apogee detected at t=(\d+) ms(?: \(([^)]*)\))?")
RX_STATE = re.compile(r"\[STATE\] (\S+) -> (\S+)")
RX_BURNOUT = re.compile(r"\[GUID\] Burnout detected at T\+(\d+) ms")
RX_DEPLOY = re.compile(r"\[DEPLOY\] detected T\+" + F + r" s")
RX_IGN = re.compile(r"RIG REPLAY: '([^']*)'.*?ignition at GPS week (\d+) TOW ([0-9.]+)")
RX_FROZEN = re.compile(r"RIG REPLAY clock: frozen for the flight \((?:the receiver's timed epoch|a fix) at t " + F)
RX_VERSION = re.compile(r"App version: *(\S+)")
RX_CANOPY = re.compile(r"canopy mode on.*? at T\+" + F + r" s")
RX_NIS = re.compile(r"\bNIS\b(?: max)?[ =:]+" + F, re.I)


def console_lines(paths):
    """(stamp ms, message) of the console logs in order, each message cut at the next stamp (a line can hold two),
    read to a second boot (a ROM banner after the app has logged: a reset prints two before it does)."""
    out = []
    for path in paths:
        for ln in open(path, "rb").read().decode("utf-8", "replace").splitlines():
            if "ESP-ROM:" in ln and out:
                return out
            ms = [(m.start(), int(m.group(2))) for m in STAMP.finditer(ln)]
            for k, (i, stamp) in enumerate(ms):
                end = ms[k + 1][0] if k + 1 < len(ms) else len(ln)
                out.append((stamp, STAMP.sub("", ln[i:end], count=1)))
    return out


def vec3(s):
    return [float(x) for x in s.split(",")]


def parse_console(paths):
    """The console's navigation series ([REPLAY], [REPLAY-NAV]: ENU -> NED), the stamps' offset onto flight time
    and the events, on flight time."""
    rep, nav, raw = [], [], {"apogee": None, "states": [], "burnout_ms": None, "deploy_t": None, "ign": None,
                             "frozen_t": None, "version": None, "nis": [], "rawgnss": [], "canopy_t": None}
    for ms, msg in console_lines(paths):
        m = RX_NAV.search(msg)
        if m:
            nav.append((ms, float(m.group(1)), m.group(2) is None, int(m.group(4)), vec3(m.group(5)), vec3(m.group(6))))
            continue
        m = RX_REPLAY.search(msg)
        if m:
            rep.append((ms, float(m.group(1)), m.group(2) is None, int(m.group(6)), vec3(m.group(7)), vec3(m.group(8))))
            continue
        m = RX_APOGEE.search(msg)
        if m and raw["apogee"] is None:
            raw["apogee"] = (ms, int(m.group(1)), m.group(2))
        m = RX_STATE.search(msg)
        if m:
            raw["states"].append((ms, m.group(1), m.group(2)))
        m = RX_BURNOUT.search(msg)
        if m and raw["burnout_ms"] is None:
            raw["burnout_ms"] = ms
        m = RX_DEPLOY.search(msg)
        if m and raw["deploy_t"] is None:
            raw["deploy_t"] = float(m.group(1))
        m = RX_IGN.search(msg)
        if m:
            raw["ign"] = (m.group(1), int(m.group(2)), float(m.group(3)))
        m = RX_FROZEN.search(msg)
        if m:
            raw["frozen_t"] = float(m.group(1))
        m = RX_CANOPY.search(msg)
        if m and raw["canopy_t"] is None:
            raw["canopy_t"] = float(m.group(1))
        m = RX_VERSION.search(msg)
        if m and raw["version"] is None:
            raw["version"] = m.group(1)
        if "[RAWGNSS]" in msg:
            raw["rawgnss"].append((ms, msg.strip()))
            m = RX_NIS.search(msg)
            if m:
                raw["nis"].append((ms, float(m.group(1))))

    def series(rows, name):
        rows = [r for r in rows if r[2] and r[3]]          # on the replay's clock, the filter up
        if not rows:
            return None
        t = np.array([r[1] for r in rows])
        enu = np.array([r[4] for r in rows])
        venu = np.array([r[5] for r in rows])
        return Series(name, t, np.stack([enu[:, 1], enu[:, 0], -enu[:, 2]], axis=1),
                      np.stack([venu[:, 1], venu[:, 0], -venu[:, 2]], axis=1), "the filter's pad reference")

    timed = [(r[0], r[1]) for r in rep + nav if r[2]]
    t_off = None
    if timed:
        st = np.array(timed, float)
        late = st[:, 1] >= -5.0                    # after the replay clock's freeze, one offset holds
        sel = late if late.sum() >= 5 else np.ones(len(st), bool)
        t_off = float(np.median(st[sel, 1] - st[sel, 0] * 1e-3))

    def t_of(ms):
        return None if ms is None or t_off is None else ms * 1e-3 + t_off

    ev = {"t_off_s": t_off, "lines": {"replay": len(rep), "replay_nav": len(nav)}, "version": raw["version"],
          "ignition_gps": raw["ign"], "frozen_t": raw["frozen_t"], "deploy_t": raw["deploy_t"],
          "canopy_t": raw["canopy_t"],
          "burnout_t": t_of(raw["burnout_ms"]), "nis": raw["nis"], "rawgnss_lines": len(raw["rawgnss"]),
          "rawgnss_last": raw["rawgnss"][-1][1] if raw["rawgnss"] else None}
    if raw["apogee"]:
        ms, millis, voters = raw["apogee"]
        ev.update(apogee_t=t_of(ms), apogee_voters=voters, apogee_stamp_minus_millis_ms=ms - millis)
    for ms, a, b in raw["states"]:
        ev.setdefault(f"state_{b}_t", t_of(ms))
    return {"console-nav": series(nav, "console-nav"), "console": series(rep, "console")}, ev


# ------------------------------------------------------------------ the flight log

def edge_from_config(cfg):
    """The ignition edge (s) of the replay file sim_replay_blob.py makes from the scene (the README's options)."""
    import sim_replay_blob as srb
    import truth as rig_truth
    cols, scen = ss.build(cfg)
    blob, _ = srb.build(cols, scen, t_from=-30.0, port_error=False, log_pressure=True)
    return rig_truth.blob_edge_t(blob)


def log_time_map(recs, edge):
    """(base, how): flight s = MCU s + base. The ignition edge in the logged accelerometer against the replay
    file's; else (no IMU records, or no edge) the launch flag, which latches some 0.2 s after ignition."""
    import truth as rig_truth
    imu = recs.get("ISM6HG256") or []
    if imu and edge is not None:
        lo = np.array([[r["low_acc_x"], r["low_acc_y"], r["low_acc_z"]] for r in imu])
        hi = np.array([[r["high_acc_x"], r["high_acc_y"], r["high_acc_z"]] for r in imu])
        rail = np.array([r["low_g_near_rail"] for r in imu])
        amag = np.where(rail, np.linalg.norm(hi, axis=1), np.linalg.norm(lo, axis=1))
        above = amag > rig_truth.EDGE_MPS2
        if above.any() and int(np.argmax(above)) > 0:
            return rig_truth.log_time_base(recs, edge), "ignition edge in the logged accelerometer"
    ns = recs.get("NonSensor") or []
    t = rig_truth.unwrap_us([r["time_us"] for r in ns]) * 1e-6 if ns else np.array([])
    k = next((i for i, r in enumerate(ns) if r["launch"]), None)
    if k is not None:
        return -float(t[k]), "the launch flag (COARSE: it latches ~0.2 s after ignition)"
    return None, "no time map: no accelerometer edge and no launch flag"


def log_series(recs, base, truth):
    import truth as rig_truth
    out = {}
    ns = recs.get("NonSensor") or []
    if ns:
        t = rig_truth.unwrap_us([r["time_us"] for r in ns]) * 1e-6 + base
        out["log-nav"] = Series("log-nav", t, np.array([[r["n_pos"], r["e_pos"], -r["u_pos"]] for r in ns]),
                                np.array([[r["n_vel"], r["e_vel"], -r["u_vel"]] for r in ns]),
                                "the filter's pad reference")
    rg = [r for r in recs.get("RawGnssEkf") or [] if r["running"]]
    if rg:
        t = rig_truth.unwrap_us([r["time_us"] for r in rg]) * 1e-6 + base
        out["log-raw"] = Series("log-raw", t, lla_to_ned([r["lat_deg"] for r in rg], [r["lon_deg"] for r in rg],
                                                         [r["alt_m"] for r in rg], truth.pad),
                                np.array([[r["vel_n"], r["vel_e"], r["vel_d"]] for r in rg]), "the scene's pad")
    return out


def log_events(recs, base):
    import truth as rig_truth
    ns = recs.get("NonSensor") or []
    ev = {}
    if not ns:
        return ev
    t = rig_truth.unwrap_us([r["time_us"] for r in ns]) * 1e-6 + base
    for key, name in (("launch", "launch_t"), ("burnout", "burnout_t"), ("apogee_flag", "apogee_t"),
                      ("deployed", "deploy_t"), ("alt_landed", "landed_t")):
        k = next((i for i, r in enumerate(ns) if r.get(key)), None)
        if k is not None:
            ev[name] = float(t[k])
    ev["span"] = (float(t.min()), float(t.max()))
    return ev


def week_of(tow_s, ref_gps_s):
    """GPS seconds of a time of week, in the week that puts it nearest ref_gps_s."""
    g = math.floor(ref_gps_s / WEEK_S) * WEEK_S + np.asarray(tow_s, float)
    return g + WEEK_S * np.round((ref_gps_s - g) / WEEK_S)


def gnss_use(recs, base, truth, windows):
    """The raw-GNSS filter's epochs by phase, from its records (0x96): each record carries the last epoch it
    applied, so an epoch is counted once (by its time of week) and placed by its own GPS time. An epoch applied
    between two records is not seen."""
    import truth as rig_truth
    rg = recs.get("RawGnssEkf") or []
    if not rg:
        return None
    tr = rig_truth.unwrap_us([r["time_us"] for r in rg]) * 1e-6 + base
    rows, last = [], None
    for r, t in zip(rg, tr):
        if not r["clock_set"] or r["epoch_tow_s"] == last:
            continue
        last = r["epoch_tow_s"]
        te = float(week_of(r["epoch_tow_s"], truth.ign_gps_s) - truth.ign_gps_s)
        rows.append((te, t, r["used_pr"], r["rejected_pr"], r["used_rr"], r["rejected_rr"], r["epoch_age_s"]))
    flags = np.array([[r["stale_epoch"], r["clock_step"], r["running"]] for r in rg], bool)
    out = {"records": len(rg), "epochs_seen": len(rows), "phases": {}}
    a = np.array(rows, float).reshape(-1, 7)
    if len(a):
        # the record's time less its epoch's GPS time less the epoch's age: when it was applied after its record
        # time on the map -- 0 to a record's interval when the map and the GPS time agree
        out["map_check_s"] = float(np.median(a[:, 1] - a[:, 0] - a[:, 6]))
    for ph, lo, hi in windows:
        m = (a[:, 0] >= lo) & (a[:, 0] < hi)
        mr = (tr >= lo) & (tr < hi)
        e = a[m]
        out["phases"][ph] = {
            "epochs": int(m.sum()), "expected_10hz": int(round((hi - lo) * 10)),
            "pr_used": int(e[:, 2].sum()), "pr_refused": int(e[:, 3].sum()),
            "rr_used": int(e[:, 4].sum()), "rr_refused": int(e[:, 5].sum()),
            "epochs_any_refused": int(((e[:, 3] + e[:, 5]) > 0).sum()),
            "epochs_all_refused": int((((e[:, 2] + e[:, 4]) == 0) & ((e[:, 3] + e[:, 5]) > 0)).sum()),
            "epochs_empty": int(((e[:, 2] + e[:, 3] + e[:, 4] + e[:, 5]) == 0).sum()),
            "age_median_s": float(np.median(e[:, 6])) if len(e) else None,
            "records": int(mr.sum()), "not_running": int((~flags[mr, 2]).sum()),
            "stale": int(flags[mr, 0].sum()), "clock_steps": int(flags[mr, 1].sum())}
    return out


def find_nis(recs):
    """Every record field named like NIS: {record.field: values}."""
    out = {}
    for name, rows in recs.items():
        if rows and isinstance(rows, list) and isinstance(rows[0], dict):
            for k in rows[0]:
                if "nis" in k.lower().split("_"):
                    out[f"{name}.{k}"] = [r[k] for r in rows]
    return out


# ------------------------------------------------------------------ the board's own stream

def read_frames(path):
    """Every CRC-good RTCM3 frame of the board's output in a .rtcm3 file or a V10 log (0x95), in order, the V10's
    own 4002s left out (from sdr_rig_events.py, fc/sdr-raw-offline)."""
    import extract_gnss_rtcm as XR
    if path.endswith(".bin"):
        good, _ = XR.extract(__import__("pathlib").Path(path))
        return [bytes(f) for _t, f in good if XR.message_type(bytes(f)) != FC_MESSAGE]
    data = open(path, "rb").read()
    out, i = [], 0
    while i + 6 <= len(data):
        if data[i] != 0xD3:
            i += 1
            continue
        n = ((data[i + 1] & 3) << 8) | data[i + 2]
        f = data[i:i + 6 + n]
        if len(f) == 6 + n and XR.frame_ok(f):
            if XR.message_type(f) != FC_MESSAGE:
                out.append(bytes(f))
            i += 6 + n
        else:
            i += 1
    return out


class Bits:
    """RTCM3's bit order over a frame's payload."""

    def __init__(self, frame):
        p = frame[3:-3]
        self.v, self.n, self.p = int.from_bytes(p, "big"), len(p) * 8, 0

    def u(self, k):
        self.p += k
        return (self.v >> (self.n - self.p)) & ((1 << k) - 1)

    def s(self, k):
        x = self.u(k)
        return x - (1 << k) if x >> (k - 1) else x


def tr_sub(frame):
    return frame[4] & 0x0F if len(frame) > 6 and ((frame[3] << 4) | (frame[4] >> 4)) == TR_MESSAGE else 0


def decode_tr_fix(frame):
    """A 4001 sub-type 2 (the gnss repo's rtcm_tr_fix_encode): the fix's GPS time, ECEF position and velocity."""
    if tr_sub(frame) != 2 or (len(frame) - 6) * 8 < 374:
        return None
    b = Bits(frame)
    b.u(16)
    week, tow_ms, sub_ns = b.u(16), b.u(30), b.s(20)
    valid, vel_valid, nsat = b.u(1), b.u(1), b.u(6)
    pos = [b.s(38) * 1e-4 for _ in range(3)]
    vel = [b.s(28) * 1e-4 for _ in range(3)]
    return {"gps_s": week * WEEK_S + tow_ms * 1e-3 + sub_ns * 1e-9, "valid": bool(valid), "vel_valid": bool(vel_valid),
            "nsat": nsat, "pos": pos, "vel": vel, "clk_bias_m": b.s(32) * 1e-3, "clk_drift_mps": b.s(28) * 1e-4,
            "pdop": b.u(10) * 0.1, "resid_rms_m": b.u(16) * 0.01}


def decode_tr_epoch(frame):
    """A 4001 sub-type 1: the epoch's GPS time, its mode and observables."""
    if tr_sub(frame) != 1 or (len(frame) - 6) * 8 < 202:
        return None
    b = Bits(frame)
    b.u(16)
    week, tow_ms = b.u(16), b.u(30)
    b.s(20), b.u(16), b.s(48)
    return {"gps_s": week * WEEK_S + tow_ms * 1e-3, "mode": b.u(2), "nobs": b.u(6)}


def board_stream(frames, truth, windows):
    """The board's fixes as a series (valid ones) and its epochs and fixes by phase."""
    fixes = [x for x in (decode_tr_fix(f) for f in frames) if x]
    epochs = [x for x in (decode_tr_epoch(f) for f in frames) if x]
    out = {"fixes": len(fixes), "epochs": len(epochs), "phases": {}}
    ft = np.array([x["gps_s"] - truth.ign_gps_s for x in fixes])
    et = np.array([x["gps_s"] - truth.ign_gps_s for x in epochs])
    nobs = np.array([x["nobs"] for x in epochs])
    valid = np.array([x["valid"] for x in fixes], bool)
    nsat = np.array([x["nsat"] for x in fixes])
    for ph, lo, hi in windows:
        me = (et >= lo) & (et < hi)
        mf = (ft >= lo) & (ft < hi)
        out["phases"][ph] = {"epochs": int(me.sum()), "nobs_median": float(np.median(nobs[me])) if me.any() else None,
                             "nobs_min": int(nobs[me].min()) if me.any() else None,
                             "epochs_under_4": int((nobs[me] < 4).sum()),
                             "fixes": int(mf.sum()), "fixes_valid": int((mf & valid).sum()),
                             "nsat_median": float(np.median(nsat[mf & valid])) if (mf & valid).any() else None}
    s = None
    if valid.any():
        v = [x for x in fixes if x["valid"]]
        rot = ecef_to_ned_rot(truth.pad)
        vel = np.array([x["vel"] if x["vel_valid"] else [np.nan] * 3 for x in v]) @ rot.T
        s = Series("board-fix", ft[valid], ecef_to_ned(np.array([x["pos"] for x in v]), truth.pad), vel,
                   "the scene's pad")
    return s, out


# ------------------------------------------------------------------ one run

def score_run(label, truth, windows, log=None, consoles=(), rtcm=None, edge=None, recs=None):
    """Everything one run's inputs give: each source's scores, the headline per phase, the apogee, the GNSS use,
    the board's stream, the time maps and what could not be scored. recs: the log already parsed (log names it)."""
    series, notes, times, events = {}, [], {}, {}
    nis = {}
    gu = None
    if consoles:
        cs, cev = parse_console(consoles)
        series.update({k: v for k, v in cs.items() if v is not None})
        times["console"] = {"t_off_s": cev["t_off_s"], "lines": cev["lines"], "version": cev["version"]}
        if cev["ignition_gps"]:
            name, week, tow = cev["ignition_gps"]
            times["console"]["ignition_gps_minus_scene_s"] = week * WEEK_S + tow - truth.ign_gps_s
            if name not in truth.name and truth.name not in name:
                notes.append(f"the replay image carries '{name}', not the scene '{truth.name}'")
        events["console"] = cev
        if cev["nis"]:
            nis["console"] = [v for _ms, v in cev["nis"]]
    if log:
        if recs is None:
            import plot_flight_data_mini as P
            recs, _st, _cfg = P.parse_binary_file(log)
        base, how = log_time_map(recs, edge)
        times["log"] = {"base_s": base, "how": how, "records": {k: len(v) for k, v in recs.items()
                                                                  if isinstance(v, list) and v}}
        if base is None:
            notes.append(f"{os.path.basename(log)}: {how}; the log is not scored")
        else:
            if "COARSE" in how:
                notes.append(f"{os.path.basename(log)}: time from {how}")
            series.update(log_series(recs, base, truth))
            events["log"] = log_events(recs, base)
            gu = gnss_use(recs, base, truth, windows)
            nis.update(find_nis(recs))
    if not series:
        raise SystemExit(f"{label}: no navigation series in its inputs")
    board = None
    frames = read_frames(rtcm) if rtcm else (read_frames(log) if log and log.endswith(".bin") else [])
    if frames:
        s, board = board_stream(frames, truth, windows)
        board["source"] = os.path.basename(rtcm or log)
        if s is not None:
            series["board-fix"] = s
        if not board["fixes"] and not board["epochs"]:
            notes.append(f"the board's own stream: no 4001 in {board['source']} (not the gnss-sdr-dev board's)")
            board = None
    scored = {name: score_series(s, truth, windows) for name, s in series.items()}
    head = headline(scored, windows)
    for ph, lo, hi in windows:
        if ph not in head:
            notes.append(f"{ph}: no navigation source covers half of it")

    # the apogee: the call from the log's flags, else the console's line; the peak from the flight source
    call_t, call_from = None, None
    if events.get("log", {}).get("apogee_t") is not None:
        call_t, call_from = events["log"]["apogee_t"], "the log's apogee flag (NonSensor)"
    elif events.get("console", {}).get("apogee_t") is not None:
        call_t, call_from = events["console"]["apogee_t"], "the console's [PYRO] line (its stamp)"
    peak_src = next((series[n] for n in HEADLINE if n in series and scored[n].get("coast", {}).get("n")), None)
    apo = apogee_scores(truth, call_t, peak_src)
    apo["call_from"] = call_from
    if "log" in events and "console" in events and events["log"].get("apogee_t") is not None \
            and events["console"].get("apogee_t") is not None:
        times["apogee_log_minus_console_s"] = events["log"]["apogee_t"] - events["console"]["apogee_t"]
    if call_t is None:
        notes.append("the apogee call: no apogee flag in the log and no [PYRO] apogee line in the console")

    if gu is None:
        notes.append("GNSS use: no raw-GNSS filter records (0x96)" + ("" if log else " -- no flight log given")
                     + "; the filter's fused and refused counts are not scored")
    if not nis:
        notes.append("NIS: not logged (the raw filter's record 0x96 carries used/refused counts, no NIS)")
    if board is None and not frames:
        notes.append("the board's own fixes: no RTCM given (--rtcm, or a log with 0x95 records)")
    for name in ("console", "console-nav"):
        if name in series:
            break
    else:
        if consoles:
            notes.append("console: no [REPLAY] or [REPLAY-NAV] line on the replay's clock")
    if "log-nav" in series and scored["log-nav"]["pad"]["cover"] < COVER and head.get("pad", "").startswith("console"):
        notes.append("pad: the log starts at launch; the pad is scored from the console")
    nis_out = {k: stats(v) for k, v in nis.items()}
    return {"label": label, "inputs": {"log": log, "console": list(consoles), "rtcm": rtcm},
            "sources": {n: {"frame": s.frame, "samples": int(len(s.t)), "phases": scored[n]}
                        for n, s in series.items()},
            "headline": head, "apogee": apo, "gnss_use": gu, "nis": nis_out, "board": board, "time": times,
            "events": {k: {kk: vv for kk, vv in v.items() if kk not in ("nis",)} for k, v in events.items()},
            "notes": notes, "_series": series}


# ------------------------------------------------------------------ the report

def f3(st, nd=1):
    if not st:
        return "-"
    return "/".join(f"{st[k]:.{nd}f}" for k in ("median", "p90", "max"))


def nd_of(metric):
    return 2 if metric.startswith("vel") else 1


def table(rows, head):
    out = ["| " + " | ".join(head) + " |", "|" + "|".join("---" for _ in head) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def phase_rows(run, windows, source=None):
    rows = []
    for ph, lo, hi in windows:
        src = source or run["headline"].get(ph)
        r = run["sources"].get(src, {}).get("phases", {}).get(ph) if src else None
        if not r or not r["n"]:
            rows.append([ph, src or "-", 0, f"{0.0:.0%}"] + ["-"] * len(METRICS))
            continue
        rows.append([ph, src, r["n"], f"{r['cover']:.0%}"] + [f3(r[k], nd_of(k)) for k in METRICS])
    return rows


def fmt_t(x, sign=True):
    return "-" if x is None else (f"T{x:+.2f} s" if sign else f"{x:.2f} s")


def apogee_rows(runs, truth):
    def g(run, k, fmt):
        v = run["apogee"].get(k)
        return "-" if v is None else fmt(v)
    rows = [["the call", f"T+{truth.apogee:.2f} s"] + [g(r, "call_t", lambda v: f"T{v:+.2f} s") + " ("
                                                       + g(r, "call_dt", lambda v: f"{v:+.2f} s") + ")" for r in runs],
            ["true height and climb at the call", "-"]
            + [g(r, "call_true_m", lambda v: f"{v:,.0f} m") + ", " + g(r, "call_true_vu", lambda v: f"{v:+.1f} m/s")
               for r in runs],
            ["the solution's height at the call", "-"] + [g(r, "call_est_m", lambda v: f"{v:,.0f} m") for r in runs],
            ["the solution's highest point", f"{truth.apogee_m:,.0f} m at T+{truth.apogee:.2f} s"]
            + [g(r, "peak_m", lambda v: f"{v:,.0f} m") + " (" + g(r, "peak_dm", lambda v: f"{v:+,.0f} m") + ") at "
               + g(r, "peak_t", lambda v: f"T{v:+.2f} s") for r in runs],
            ["the solution's height at the true apogee", f"{truth.apogee_m:,.0f} m"]
            + [g(r, "at_truth_m", lambda v: f"{v:,.0f} m") + " (" + g(r, "at_truth_dm", lambda v: f"{v:+,.0f} m") + ")"
               for r in runs]]
    return rows


def report(truth, windows, runs):
    """The Markdown report: the truth, each run's headline table, the side by side, the apogee, the GNSS use, the
    board, every source, the time maps and what was not scored."""
    L = [f"# HIL score: {truth.name}", ""]
    L.append(f"Truth (the scene's config): ignition at GPS {truth.ign_gps_s:.1f} s, burnout T+{truth.burnout:.2f} s, "
             f"apogee T+{truth.apogee:.2f} s at {truth.apogee_m:,.0f} m above the pad; the scene ends at "
             f"T+{truth.end:.2f} s, {truth.end_m:,.0f} m up and {truth.end_vu:+.0f} m/s. Vertical all the way: the "
             "true horizontal position and velocity are 0. Phases: " +
             ", ".join(f"{ph} T{lo:+.2f}..T{hi:+.2f} s" for ph, lo, hi in windows) + ".")
    L.append("")
    L.append("Each cell is median/p90/max of the absolute error. Positions from the solution's frame: the log's "
             "NonSensor and the console's lines are ENU from the filter's own pad reference (an absolute height "
             "error on the pad does not show); log-raw and board-fix are from the scene's pad.")
    head = ["phase", "source", "n", "cover"] + [METRIC_NAMES[k] for k in METRICS]
    for run in runs:
        L += ["", f"## {run['label']}: position and velocity error", "", table(phase_rows(run, windows), head)]
    if len(runs) == 2:
        a, b = runs
        L += ["", f"## Side by side: {a['label']} against {b['label']} (p90; the headline source of each)", ""]
        rows = []
        for ph, _lo, _hi in windows:
            ra = a["sources"].get(a["headline"].get(ph), {}).get("phases", {}).get(ph)
            rb = b["sources"].get(b["headline"].get(ph), {}).get("phases", {}).get(ph)
            for k in METRICS:
                xa = ra.get(k) if ra else None
                xb = rb.get(k) if rb else None
                nd = nd_of(k)
                d = (xa["p90"] - xb["p90"]) if xa and xb else None
                mark = "" if d is None or round(d, nd) == 0 else ("worse" if d > 0 else "better")
                rows.append([ph, METRIC_NAMES[k], "-" if not xa else f"{xa['p90']:.{nd}f}",
                             "-" if not xb else f"{xb['p90']:.{nd}f}", "-" if d is None else f"{d:+.{nd}f}", mark])
        L.append(table(rows, ["phase", "metric", a["label"], b["label"], "difference", ""]))
        L.append("")
        L.append(f"Sources: {a['label']} " + ", ".join(f"{ph} {a['headline'].get(ph, '-')}" for ph, *_ in windows)
                 + f"; {b['label']} " + ", ".join(f"{ph} {b['headline'].get(ph, '-')}" for ph, *_ in windows) + ".")
    L += ["", "## Apogee", "", table(apogee_rows(runs, truth), ["", "truth"] + [r["label"] for r in runs])]
    L.append("")
    for r in runs:
        a = r["apogee"]
        L.append(f"- {r['label']}: the call from {a.get('call_from') or '-'}; the highest point from "
                 f"{a.get('peak_source', '-')}.")
    for r in runs:
        gu = r["gnss_use"]
        L += ["", f"## {r['label']}: the filter's use of GNSS"]
        if gu is None:
            L += ["", "No raw-GNSS filter records (0x96) in this run's inputs."]
        else:
            rows = [[ph, f"{p['epochs']} ({p['expected_10hz']} at 10 Hz)", p["pr_used"], p["pr_refused"],
                     p["rr_used"], p["rr_refused"], p["epochs_any_refused"], p["epochs_all_refused"],
                     p["epochs_empty"], "-" if p["age_median_s"] is None else f"{p['age_median_s'] * 1e3:.0f} ms",
                     p["stale"], p["clock_steps"], p["not_running"]] for ph, p in gu["phases"].items()]
            L += ["", table(rows, ["phase", "epochs seen", "PR fused", "PR refused", "RR fused", "RR refused",
                                   "epochs with a refusal", "epochs all refused", "epochs with nothing to fuse",
                                   "age (median)", "stale records", "clock steps", "records not running"])]
            L.append("")
            L.append(f"{gu['records']} records, {gu['epochs_seen']} distinct epochs. Each record carries the last "
                     "epoch applied, so an epoch applied between two records is not counted. Map check (record "
                     "time less the epoch's GPS time less its age, median): "
                     + ("-" if gu.get("map_check_s") is None else f"{gu['map_check_s'] * 1e3:+.0f} ms") + ".")
        if r["nis"]:
            L.append("")
            L.append("NIS: " + "; ".join(f"{k} {f3(v, 2)} (median/p90/max of {v['n']})" for k, v in r["nis"].items()
                                         if v))
        else:
            L += ["", "NIS: not logged."]
    for r in runs:
        b = r["board"]
        if not b:
            continue
        L += ["", f"## {r['label']}: the board's own stream ({b['source']})", ""]
        fx = r["sources"].get("board-fix", {}).get("phases", {})
        rows = [[ph, p["epochs"], "-" if p["nobs_median"] is None else f"{p['nobs_median']:.0f}",
                 "-" if p["nobs_min"] is None else p["nobs_min"], p["epochs_under_4"],
                 f"{p['fixes_valid']}/{p['fixes']}",
                 "-" if p["nsat_median"] is None else f"{p['nsat_median']:.0f}",
                 f3((fx.get(ph) or {}).get("pos_3d")), f3((fx.get(ph) or {}).get("vel_3d"), 2)]
                for ph, p in b["phases"].items()]
        L.append(table(rows, ["phase", "epochs", "observables (median)", "observables (min)", "epochs under 4",
                              "fixes valid", "satellites (median)", "fix position 3D (m)", "fix velocity 3D (m/s)"]))
    L += ["", "## Every source"]
    for r in runs:
        for name in r["sources"]:
            L += ["", f"### {r['label']}: {name} ({r['sources'][name]['frame']}, {r['sources'][name]['samples']} "
                      "samples)", "", table(phase_rows(r, windows, name), head)]
    L += ["", "## Time"]
    for r in runs:
        t = r["time"]
        if "console" in t:
            c = t["console"]
            s = (f"- {r['label']}, console ({c.get('version') or 'image ?'}): replay t = stamp "
                 f"{c['t_off_s']:+.3f} s from {c['lines']['replay']} [REPLAY] and {c['lines']['replay_nav']} "
                 "[REPLAY-NAV] lines" if c["t_off_s"] is not None else f"- {r['label']}, console: no line on the "
                 "replay's clock")
            if "ignition_gps_minus_scene_s" in c:
                s += f"; the image's ignition against the scene's: {c['ignition_gps_minus_scene_s'] * 1e3:+.1f} ms"
            L.append(s + ".")
        if "log" in t:
            lg = t["log"]
            L.append(f"- {r['label']}, log: " + ("no map" if lg["base_s"] is None else
                                                  f"flight s = MCU s {lg['base_s']:+.4f}") + f" ({lg['how']}).")
        if "apogee_log_minus_console_s" in t:
            L.append(f"- {r['label']}: the apogee call by the log's map less by the console's: "
                     f"{t['apogee_log_minus_console_s'] * 1e3:+.0f} ms.")
        ev = r["events"]
        for src in ("log", "console"):
            e = ev.get(src) or {}
            got = [f"{k[:-2]} {fmt_t(v)}" for k, v in e.items() if k.endswith("_t") and isinstance(v, float)]
            if got:
                L.append(f"- {r['label']}, events from the {src}: " + ", ".join(got) + ".")
    L += ["", "## Not scored, and why"]
    for r in runs:
        L += [f"- {r['label']}: {n}" for n in r["notes"]]
    return "\n".join(L) + "\n"


# ------------------------------------------------------------------ the plots

INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
COLORS = ("#2a78d6", "#eb6834")              # this run, the reference (the categorical slots 1 and 2)
PLOT_METRICS = (("pos_h", "horizontal position\nerror (m)"), ("pos_v", "vertical position\nerror, up (m)"),
                ("vel_h", "horizontal velocity\nerror (m/s)"), ("vel_v", "vertical velocity\nerror, up (m/s)"))


def stitched(run, windows, truth):
    """Each phase's headline source over that phase, joined: (t, errors) for plotting."""
    ts, es = [], {k: [] for k in METRICS}
    for ph, lo, hi in windows:
        name = run["headline"].get(ph)
        if not name:
            continue
        s = run["_series"][name]
        e = errors(s, truth)
        m = (s.t >= lo) & (s.t < hi)
        ts.append(s.t[m])
        for k in METRICS:
            es[k].append(e[k][m])
    if not ts:
        return np.array([]), {k: np.array([]) for k in METRICS}
    return np.concatenate(ts), {k: np.concatenate(v) for k, v in es.items()}


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.6)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=8)


def plot(truth, windows, runs, out_prefix):
    """Two figures, linear axes only: the whole flight (height, then each error) and each phase on its own scale."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    data = [(r, *stitched(r, windows, truth)) for r in runs]
    t0, t1 = windows[0][1], windows[-1][2]
    paths = []

    fig, axes = plt.subplots(5, 1, figsize=(11, 12.5), sharex=True, facecolor=SURFACE)
    tt = np.linspace(t0, t1, 2000)
    axes[0].plot(tt, -truth.at(tt)[0][:, 2] / 1e3, color=INK2, linewidth=1.5, linestyle="--", label="truth")
    for (r, t, e), c in zip(data, COLORS):
        name_src = {ph: r["headline"].get(ph) for ph, *_ in windows}
        up = []
        for ph, lo, hi in windows:
            s = r["_series"].get(name_src[ph]) if name_src[ph] else None
            if s is not None:
                m = (s.t >= lo) & (s.t < hi)
                up.append((s.t[m], -s.pos[m, 2]))
        if up:
            axes[0].plot(np.concatenate([u[0] for u in up]), np.concatenate([u[1] for u in up]) / 1e3, color=c,
                         linewidth=1.5, label=r["label"])
        for ax, (k, _lab) in zip(axes[1:], PLOT_METRICS):
            ax.plot(t, e[k], color=c, linewidth=1.5, marker="o" if len(t) < 400 else None, markersize=3,
                    label=r["label"])
    axes[0].set_ylabel("height above\nthe pad (km)", color=INK, fontsize=9)
    for ax, (_k, lab) in zip(axes[1:], PLOT_METRICS):
        ax.set_ylabel(lab, color=INK, fontsize=9)
        ax.axhline(0.0, color=INK2, linewidth=0.8)
    for ax in axes:
        style(ax)
        for _ph, lo, _hi in windows[1:]:
            ax.axvline(lo, color=INK2, linewidth=0.8, linestyle=":")
    for ph, lo, hi in windows:
        axes[0].text((lo + hi) / 2, 1.02, ph, transform=axes[0].get_xaxis_transform(), ha="center", va="bottom",
                     fontsize=9, color=INK2)
    axes[0].legend(loc="upper left", fontsize=8, frameon=False)
    axes[-1].set_xlabel("flight time from ignition (s)", color=INK, fontsize=9)
    axes[-1].set_xlim(t0, t1)
    fig.suptitle(f"{truth.name}: the V10's navigation solution against the truth "
                 f"({' vs '.join(r['label'] for r in runs)})", color=INK, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    p = out_prefix + "_flight.png"
    fig.savefig(p, dpi=120, facecolor=SURFACE)
    plt.close(fig)
    paths.append(p)

    fig, axes = plt.subplots(len(PLOT_METRICS), len(windows), figsize=(4 * len(windows), 10), facecolor=SURFACE,
                             squeeze=False)
    for j, (ph, lo, hi) in enumerate(windows):
        for i, (k, lab) in enumerate(PLOT_METRICS):
            ax = axes[i, j]
            for (r, t, e), c in zip(data, COLORS):
                m = (t >= lo) & (t < hi)
                if m.any():
                    ax.plot(t[m], e[k][m], color=c, linewidth=1.5, marker="o" if m.sum() < 200 else None,
                            markersize=3, label=r["label"])
            ax.axhline(0.0, color=INK2, linewidth=0.8)
            ax.set_xlim(lo, hi)
            style(ax)
            if j == 0:
                ax.set_ylabel(lab, color=INK, fontsize=9)
            if i == 0:
                ax.set_title(f"{ph} (T{lo:+.1f}..T{hi:+.1f} s)", color=INK, fontsize=10)
            if i == len(PLOT_METRICS) - 1:
                ax.set_xlabel("flight time (s)", color=INK, fontsize=9)
    hs, ls = axes[0, 0].get_legend_handles_labels()
    for ax in axes.flat:
        h, lab = ax.get_legend_handles_labels()
        if len(h) > len(hs):
            hs, ls = h, lab
    fig.legend(hs, ls, loc="upper right", fontsize=9, frameon=False, ncol=len(hs))
    fig.suptitle(f"{truth.name}: each phase on its own scale (linear)", color=INK, fontsize=11, x=0.35)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p = out_prefix + "_phases.png"
    fig.savefig(p, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    paths.append(p)
    return paths


# ------------------------------------------------------------------ main

def jsonable(o):
    if isinstance(o, dict):
        return {str(k): jsonable(v) for k, v in o.items() if not str(k).startswith("_")}
    if isinstance(o, (list, tuple)):
        return [jsonable(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if not np.isfinite(o) else float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", nargs="?", help="the scene's SignalSim config (sdr_hotshot_b1c_p120.json)")
    ap.add_argument("--truth-csv", help="instead of a config: a sim_flight_3d.py flight's sensors CSV (with "
                    "--scenario its JSON)")
    ap.add_argument("--scenario")
    ap.add_argument("--log", help="the V10's flight log (.bin)")
    ap.add_argument("--console", action="append", default=[],
                    help="the V10's console log; give it twice for the reset log and the run's, in that order")
    ap.add_argument("--rtcm", help="the board's recording (.rtcm3); default the log's 0x95 records")
    ap.add_argument("--blob", help="the replay file the image carries (its ignition edge); default made from the "
                    "config")
    ap.add_argument("--label", default="this run")
    ap.add_argument("--ref-log")
    ap.add_argument("--ref-console", action="append", default=[])
    ap.add_argument("--ref-rtcm")
    ap.add_argument("--ref-label", default="GNSS-absent")
    ap.add_argument("--pad-s", type=float, default=PAD_S, help="the pad's window before ignition (s)")
    ap.add_argument("--out-dir", default=".")
    ap.add_argument("--name", default="hil_score", help="the outputs' prefix in --out-dir")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args(argv)
    if not (a.log or a.console):
        ap.error("give --log and/or --console")
    if bool(a.config) == bool(a.truth_csv) or bool(a.truth_csv) != bool(a.scenario):
        ap.error("give the scene's config, or --truth-csv with --scenario")
    truth = SceneTruth(json.load(open(a.config))) if a.config else CsvTruth(a.truth_csv, a.scenario)
    windows = truth.windows(a.pad_s)
    edge = None
    if a.log or a.ref_log:
        import truth as rig_truth
        if a.blob:
            edge = rig_truth.blob_edge_t(open(a.blob, "rb").read())
        elif a.config:
            edge = edge_from_config(json.load(open(a.config)))
        else:
            edge = rig_truth.edge_t(a.truth_csv, a.scenario)
    runs = [score_run(a.label, truth, windows, a.log, a.console, a.rtcm, edge)]
    if a.ref_log or a.ref_console:
        runs.append(score_run(a.ref_label, truth, windows, a.ref_log, a.ref_console, a.ref_rtcm, edge))
    os.makedirs(a.out_dir, exist_ok=True)
    prefix = os.path.join(a.out_dir, a.name)
    md = report(truth, windows, runs)
    if not a.no_plot:
        pngs = plot(truth, windows, runs, prefix)
        md += "\n" + "\n".join(f"![{os.path.basename(p)}]({os.path.basename(p)})" for p in pngs) + "\n"
    open(prefix + ".md", "w").write(md)
    out = {"scene": truth.name, "config": os.path.abspath(a.config or a.truth_csv), "edge_s": edge,
           "truth": {"ign_gps_s": truth.ign_gps_s, "burnout": truth.burnout, "apogee": truth.apogee,
                     "apogee_m": truth.apogee_m, "end": truth.end, "pad": truth.pad},
           "windows": windows, "runs": runs}
    with open(prefix + ".json", "w") as fh:
        json.dump(jsonable(out), fh, indent=1)
    print(md)
    print(f"wrote {prefix}.md, {prefix}.json" + ("" if a.no_plot else ", the plots"))
    return out


if __name__ == "__main__":
    main()
