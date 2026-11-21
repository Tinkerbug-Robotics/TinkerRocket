#!/usr/bin/env python3
"""Score raw_replay runs of an RF-rig events file (sdr_rig_events.py) against its flight's truth.

    score_sdr_rig.py RUN.events label=RUN.csv [label=RUN2.csv ...] [--json OUT.json]

The truth is the events file's own (META sdr_rig_truth beside it; phase times META truth_*_s, flight time 0 =
ignition). Phases: boost (T+0 to burnout), coast (burnout to apogee, or to the file's end before one), descent (apogee to the
file's end); a static
file has one, static (its whole length). Each metric every 0.1 s of flight time, scorelib's way: the run's nearest
sample, the truth at that sample's time; median / 90th percentile / largest of the absolute error. Attitude in deg,
velocity in m/s, position in m (NED from the pad).
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scorelib as S  # noqa: E402


class RigTruth:
    """scorelib.Truth's interface on sdr_rig_events.py's truth (.nav) and META."""

    def __init__(self, events_path, meta):
        base = float(meta["t_base_s"])
        self.pad = tuple(float(x) for x in meta["pad_lla"].split())
        nav = os.path.join(os.path.dirname(os.path.abspath(events_path)), meta["sdr_rig_truth"])
        d = np.loadtxt(nav, comments="#")
        self.t = d[:, 0] * 1e-6 + base
        self.vel = d[:, 1:4]
        self.q = S.qnorm(d[:, 4:8])
        self.pos = np.stack([np.zeros(len(d)), np.zeros(len(d)), -(d[:, 8] - self.pad[2])], axis=1)
        self.burnout = float(meta["truth_burnout_s"])
        self.apogee = float(meta["truth_apogee_s"])
        self.end = float(self.t[-1])
        self.landed = self.end
        self.static = self.burnout < 0
        self.pitch_vote = None

    def at(self, t):
        t = np.asarray(t, float)
        pos = np.stack([np.interp(t, self.t, self.pos[:, i]) for i in range(3)], axis=1)
        vel = np.stack([np.interp(t, self.t, self.vel[:, i]) for i in range(3)], axis=1)
        i = np.clip(np.searchsorted(self.t, t), 0, len(self.t) - 1)
        return pos, vel, self.q[i]

    def phases(self, t_end):
        end = min(t_end, self.end)
        if self.static:
            return [("static", 0.0, end)]
        if self.apogee < 0:                     # the file ends before apogee
            return [("boost", 0.0, self.burnout), ("coast", self.burnout, end)]
        return [("boost", 0.0, self.burnout), ("coast", self.burnout, self.apogee), ("descent", self.apogee, end)]


def series(csv_path, base, pad):
    """A raw_replay CSV as scorelib's series, its position NED from this file's pad."""
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    q = np.stack([d["qw"], d["qx"], d["qy"], d["qz"]], axis=1)
    return dict(t=d["t_mcu_s"] + base, q=q, pos=S.ned(d["lat_deg"], d["lon_deg"], d["h_m"], pad=pad),
                vel=np.stack([d["vn"], d["ve"], d["vd"]], axis=1), ok=(d["healthy"] > 0) & (d["finite"] > 0),
                kind="harness")


def fmt3(x, nd=2):
    return "/".join(("-" if not np.isfinite(v) else f"{v:.{nd}f}") for v in x)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--json")]
    out_json = next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--json=")), None)
    if len(args) < 2:
        raise SystemExit(__doc__)
    ev = args[0]
    meta = S.read_meta(ev)
    tr = RigTruth(ev, meta)
    base = float(meta["t_base_s"])
    print(f"{meta.get('sdr_rig_config', '?')}: " + ("static" if tr.static else
          f"burnout T+{tr.burnout:.2f} s, apogee T+{tr.apogee:.2f} s") + f", the file to T+{tr.end:.1f} s")
    res = {}
    for a in args[1:]:
        lab, path = a.split("=", 1) if "=" in a else (os.path.basename(a), a)
        r = S.score(series(path, base, tr.pad), tr)
        res[lab] = r
        print(f"\n== {lab}: {r['t_first']:+.1f} .. {r['t_last']:+.1f} s, unhealthy {r['unhealthy_samples']}/"
              f"{r['samples']}, largest gap {r['max_gap_s']:.2f} s")
        print("  phase     n    nose(deg)        heading(deg)      vz(m/s)          vel(m/s)         pos_h(m)"
              "           pos_v(m)")
        for ph, m in r["phases"].items():
            print(f"  {ph:8s} {m['n']:4d}  {fmt3(m['nose']):15s}  {fmt3(m['heading']):16s}  {fmt3(m['vz']):15s}  "
                  f"{fmt3(m['vel']):15s}  {fmt3(m['pos_h'], 1):17s}  {fmt3(m['pos_v'], 1)}")
    if out_json:
        with open(out_json, "w") as fh:
            json.dump(res, fh, indent=1, default=float)
    return res


if __name__ == "__main__":
    main()
