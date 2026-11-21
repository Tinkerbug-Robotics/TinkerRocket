#!/usr/bin/env python3
"""Score PX1105R replays of a simulated rig flight (px_events.py) by phase, against the sim's truth (scorelib), and
the PX1105R's own fix (0xDF, from the capture) the same way.

    px_score.py EVENTS --run NAME=RUN.csv [--run ...] [--capture CAPTURE] [--json OUT.json]

Phases (flight time, s from ignition): pad (-60 to 0), boost (0 to burnout), coast (burnout to apogee - 2),
apogee (apogee +- 2), descent (apogee + 2 to landing). Each: median / p90 / max of the 3-D velocity error, the
vertical velocity error, the horizontal and vertical position error, and (runs only) the attitude's total error,
nose direction and heading, sampled every 0.1 s (the run's nearest sample, the truth at its own time).
"""
import argparse
import json
import math
import os
import re
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402


class JsonTruth:
    """A real flight's scenario truth (scenarios/real_<flight>_sky3d_pad180.json: 10 Hz north, east, height and their
    rates, the KF/RTS of its own log): no attitude, so only the velocity and position are scored."""

    def __init__(self, path):
        sc = json.load(open(path))
        pro = float(sc.get("prologue_s", 180.0))
        tr = sc["truth"]
        self.t = np.array([r["t"] - pro for r in tr])
        a0 = tr[0]["alt_m"]
        self.pos = np.array([[r["north_m"], r["east_m"], -(r["alt_m"] - a0)] for r in tr])
        self.vel = np.array([[r["v_north_mps"], r["v_east_mps"], -r["v_up_mps"]] for r in tr])
        o = sc["origin"]
        self.pad = (float(o["lat_deg"]), float(o["lon_deg"]), float(o["alt_m"]))
        f = sc["flight"]
        self.burnout = float(f["burnout_s"]) - pro
        self.apogee = float(f["apogee_s"]) - pro
        self.landed = float(f["landing_s"]) - pro
        self.end = float(self.t[-1])
        if f.get("descent_extrapolated"):
            self.end = float(f["log_end_s"]) - pro   # the truth past the log is an extrapolation

    def at(self, t):
        t = np.asarray(t, float)
        pos = np.stack([np.interp(t, self.t, self.pos[:, i]) for i in range(3)], axis=1)
        vel = np.stack([np.interp(t, self.t, self.vel[:, i]) for i in range(3)], axis=1)
        return pos, vel, None


def phases(tr, t_end):
    end = min(t_end, tr.landed)
    return [("pad", -60.0, 0.0), ("boost", 0.0, tr.burnout), ("coast", tr.burnout, tr.apogee - 2.0),
            ("apogee", tr.apogee - 2.0, tr.apogee + 2.0), ("descent", tr.apogee + 2.0, end)]


def errors(series, tr, grid):
    t = series["t"]
    idx = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
    idx = np.where(np.abs(t[idx - 1] - grid) <= np.abs(t[idx] - grid), idx - 1, idx)
    near = np.abs(t[idx] - grid) <= 0.06
    idx, grid = idx[near], grid[near]
    ts = t[idx]
    pt, vt, qt = tr.at(ts)
    perr = series["pos"][idx] - pt
    verr = series["vel"][idx] - vt
    out = {"t": grid, "vel": np.linalg.norm(verr, axis=1), "vz": verr[:, 2], "pos_h": np.hypot(perr[:, 0], perr[:, 1]),
           "pos_v": perr[:, 2], "vh": np.hypot(verr[:, 0], verr[:, 1])}
    # the position relative to the run's own pad (its mean over T-30..T-1 less the truth's there): the on-board
    # [REPLAY-NAV] position is relative to the flight computer's pad reference, so this is the one to compare
    pm = (ts >= -30.0) & (ts < -1.0)
    if pm.any():
        rel = perr - perr[pm].mean(axis=0)
        out["pos_h_rel"] = np.hypot(rel[:, 0], rel[:, 1])
        out["pos_v_rel"] = rel[:, 2]
    if series.get("q") is not None and qt is not None:
        total, nose, _roll, heading, pitch = S.att_errors(series["q"][idx], qt)
        out.update(att=total, nose=nose, heading=heading, pitch=pitch)
    return out


def table(series, tr, t_end):
    grid = np.arange(-60.0, min(t_end, tr.end) + 1e-9, 0.1)
    e = errors(series, tr, grid)
    res = {}
    for name, lo, hi in phases(tr, t_end):
        m = (e["t"] >= lo) & (e["t"] < hi)
        if not m.any():
            continue
        r = {"n": int(m.sum())}
        for k in ("vel", "vh", "vz", "pos_h", "pos_v", "pos_h_rel", "pos_v_rel", "att", "nose", "heading"):
            if k in e:
                r[k] = S.stats3(e[k][m])
        # signed means: a bias shows
        r["vz_mean"] = float(np.mean(e["vz"][m]))
        r["pos_v_mean"] = float(np.mean(e["pos_v"][m]))
        res[name] = r
    return res


def fix_series(capture, gps_ign):
    """The PX1105R's own fixes (0xDF with a fix), at their GPS instants, flight time."""
    rows = []
    op = open
    if capture.endswith(".gz"):
        import gzip
        op = gzip.open
    with op(capture, "rt", errors="replace") as f:
        for ln in f:
            p = ln.split(None, 2)
            if len(p) < 3 or p[1] != "B" or not p[2].startswith("df"):
                continue
            b = bytes.fromhex(p[2].strip())
            if len(b) < 81 or b[2] < 2:
                continue
            week = struct.unpack_from(">H", b, 3)[0]
            tow = struct.unpack_from(">d", b, 5)[0]
            x = struct.unpack_from(">3d", b, 13)
            v = struct.unpack_from(">3f", b, 37)
            rows.append((week * 604800.0 + tow - gps_ign, x, v))
    t = np.array([r[0] for r in rows])
    X = np.array([r[1] for r in rows])
    V = np.array([r[2] for r in rows], float)
    a, e2 = 6378137.0, 6.69437999014e-3
    lon = np.arctan2(X[:, 1], X[:, 0])
    p = np.hypot(X[:, 0], X[:, 1])
    lat = np.arctan2(X[:, 2], p * (1 - e2))
    for _ in range(6):
        n = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
        h = p / np.cos(lat) - n
        lat = np.arctan2(X[:, 2], p * (1 - e2 * n / (n + h)))
    sl, cl, so, co = np.sin(lat), np.cos(lat), np.sin(lon), np.cos(lon)
    vn = -sl * co * V[:, 0] - sl * so * V[:, 1] + cl * V[:, 2]
    ve = -so * V[:, 0] + co * V[:, 1]
    vd = -(cl * co * V[:, 0] + cl * so * V[:, 1] + sl * V[:, 2])
    pos = S.ned(np.degrees(lat), np.degrees(lon), h)
    return {"t": t, "pos": pos, "vel": np.stack([vn, ve, vd], axis=1), "q": None, "ok": np.ones(len(t), bool)}


F = r"([-+]?(?:[0-9.]+|nan|inf))"
RX_NAV = re.compile(r"\[REPLAY-NAV\] t=" + F + r"(\(pad, no clock\))? st=(\d+) ekf=(\d) vote=(\d) q=\(([^)]*)\) "
                    r"pos=\(([^)]*)\) vel=\(([^)]*)\)")


def onboard_series(console):
    """The V9 rig image's own navigation state ([REPLAY-NAV], tools/rig_replay/score_v9.py's reading): flight time,
    q (body FRD -> NED), position ENU from the flight computer's pad reference, velocity ENU."""
    rows = []
    for ln in open(console, errors="replace"):
        m = RX_NAV.search(ln)
        if not m or m.group(2) is not None:
            continue
        try:   # a console line another line's output broke into is skipped
            q = [float(x) for x in m.group(6).split(",")]
            p = [float(x) for x in m.group(7).split(",")]
            v = [float(x) for x in m.group(8).split(",")]
        except ValueError:
            continue
        if len(q) != 4 or len(p) != 3 or len(v) != 3:
            continue
        rows.append([float(m.group(1))] + q + [p[1], p[0], -p[2], v[1], v[0], -v[2], int(m.group(4))])
    d = np.array(rows)
    d = d[np.argsort(d[:, 0], kind="stable")]
    return {"t": d[:, 0], "q": d[:, 1:5], "pos": d[:, 5:8], "vel": d[:, 8:11], "ok": d[:, 11] > 0}


def fmt(r, k, nd=2):
    if k not in r:
        return "-"
    m, p9, mx = r[k]
    return f"{m:.{nd}f}/{p9:.{nd}f}/{mx:.{nd}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("events")
    ap.add_argument("--run", action="append", default=[], help="NAME=RUN.csv")
    ap.add_argument("--capture")
    ap.add_argument("--onboard", action="append", default=[], help="NAME=FC_CONSOLE: a V9 rig run's [REPLAY-NAV]")
    ap.add_argument("--json")
    ap.add_argument("--truth-json", help="a real flight's scenario JSON as the truth (velocity and position only)")
    args = ap.parse_args()
    meta = S.read_meta(args.events)
    t_base = float(meta["t_base_s"])
    gps_ign = float(meta["gps_ignition_s"])
    tr = JsonTruth(args.truth_json) if args.truth_json else S.Truth()
    if args.truth_json:
        S.PAD = tr.pad   # positions about this flight's pad
        S.ned.__defaults__ = (tr.pad,)
    print(f"truth: burnout T+{tr.burnout:.2f}, apogee T+{tr.apogee:.2f}, landed T+{tr.landed:.2f}")
    out = {}
    series = []
    for spec in args.run:
        name, path = spec.split("=", 1)
        series.append((name, S.harness_series(path, t_base)))
    for spec in args.onboard:
        name, path = spec.split("=", 1)
        series.append((name, onboard_series(path)))
    if args.capture:
        series.append(("px_own_fix", fix_series(args.capture, gps_ign)))
    for name, s in series:
        res = table(s, tr, float(s["t"][-1]))
        out[name] = res
        print(f"\n== {name}  (median/p90/max)")
        print(f"  {'phase':8s} {'n':>4s}  {'|dv| m/s':>16s} {'dvz m/s':>16s} {'pos_h m':>16s} {'pos_v m':>16s} "
              f"{'pos_h_rel':>16s} {'pos_v_rel':>16s} {'att deg':>16s} {'nose deg':>16s} {'hdg deg':>16s}  vz_mean "
              f"pos_v_mean")
        for ph, r in res.items():
            print(f"  {ph:8s} {r['n']:4d}  {fmt(r, 'vel'):>16s} {fmt(r, 'vz'):>16s} {fmt(r, 'pos_h'):>16s} "
                  f"{fmt(r, 'pos_v'):>16s} {fmt(r, 'pos_h_rel'):>16s} {fmt(r, 'pos_v_rel'):>16s} {fmt(r, 'att'):>16s} "
                  f"{fmt(r, 'nose'):>16s} {fmt(r, 'heading'):>16s}  {r['vz_mean']:+.3f} {r['pos_v_mean']:+.2f}")
        if s.get("events"):
            ev = s["events"]
            q = ev.get("quorum") or {}
            if q:
                print(f"  quorum: {json.dumps({k: q[k] for k in list(q)[:12]})[:400]}")
    if args.json:
        json.dump(out, open(args.json, "w"), indent=1)


if __name__ == "__main__":
    main()
