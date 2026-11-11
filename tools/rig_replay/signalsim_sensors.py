#!/usr/bin/env python3
"""A vertical SignalSim scene's sensors, for a rig replay of it (#1600; the P1 rig's boost through the V10).

The gnss-sdr-dev board's rig scenes (the gnss repo's cocom-rig/sdr/signalsim configs: hotshot, traveler) are made
from a trajectoryList of Const and VerticalAcc segments, with no sim_flight_3d.py run and so no sensors CSV beside
them. This writes the two files tools/sim_replay_blob.py takes, from the scene's own config:

  OUT_sensors.csv.gz  1 kHz, body FRD, NED at the pad: the specific force (the trajectory's acceleration, Coriolis
                      and normal gravity at the height), the earth's rate, the attitude (nose up all the way, the belly
                      toward --heading-deg, no roll), the barometer (the 1976 standard atmosphere at the height, the
                      pad's height taken as above sea level, no port error), the high-g channel in use over 15 g, and
                      the phase (pad, boost to the first deceleration, coast to apogee, drogue after it);
  OUT_scenario.json   the file's start (UTC), the ignition's offset into it, its length and the pad.

A scene that ends in the air (the hotshot is falling at 145 m/s, 20 km up, when it ends at T+120) gets --tail-s of
landed samples at the last height after it: the rocket stopped where the signals stopped, so the flight computer's
landing comes from a still barometer with the board's signals already gone. Only the samples after the scene's end
are made up; the board has nothing to track there anyway.

    tools/rig_replay/signalsim_sensors.py CONFIG.json OUT [--heading-deg 0] [--tail-s 15]
    tools/sim_replay_blob.py OUT_sensors.csv.gz OUT_scenario.json OUT.bin --no-port-error --from -30

The replay carries 500 Hz at 24 bytes a sample in the image: a V10's 3 MB app slot holds about 180 s of it beside
the flight image.
"""
import argparse
import datetime as dt
import gzip
import json
import math
import sys
from pathlib import Path

import numpy as np

OMEGA_E = 7.2921151467e-5           # rad/s
A_WGS, F_WGS = 6378137.0, 1.0 / 298.257223563
E2_WGS = F_WGS * (2.0 - F_WGS)
G0 = 9.80665
HIGH_G = 15.0 * G0                   # the low-g accelerometer (16 g) hands over to the high-g channel near here
PHASE_PAD, PHASE_BOOST, PHASE_COAST, PHASE_DROGUE, PHASE_LANDED = 0, 2, 3, 4, 6


class Trajectory:
    """A trajectoryList's vertical motion from rest: the velocity piecewise linear between the segments' knots."""

    def __init__(self, cfg):
        tr = cfg["trajectory"]
        p = tr["initPosition"]
        if p.get("type") != "LLA" or p.get("format", "d") != "d":
            raise SystemExit(f"initPosition {p}: only LLA in degrees")
        if float(tr.get("initVelocity", {}).get("speed", 0.0)) != 0.0:
            raise SystemExit("initVelocity: only a start at rest")
        self.pad = (float(p["latitude"]), float(p["longitude"]), float(p["altitude"]))
        tk, vk, self.ignition = [0.0], [0.0], None
        for s in tr["trajectoryList"]:
            if s["type"] == "Const":
                v_end = vk[-1]
            elif s["type"] == "VerticalAcc":
                v_end = float(s["speed"])
                if self.ignition is None:
                    self.ignition = tk[-1]
            else:
                raise SystemExit(f"segment type {s['type']}: only Const and VerticalAcc")
            tk.append(tk[-1] + float(s["time"]))
            vk.append(v_end)
        if self.ignition is None:
            raise SystemExit("a static scene: no flight to replay")
        self.tk, self.vk = np.array(tk), np.array(vk)
        self.hk = self.pad[2] + np.concatenate([[0.0], np.cumsum(np.diff(self.tk) * (self.vk[1:] + self.vk[:-1]) / 2)])
        self.duration = float(self.tk[-1])

    def at(self, t):
        """Height (m), up velocity (m/s) and up acceleration (m/s^2) at file times t (held after the end)."""
        t = np.asarray(t, float)
        i = np.clip(np.searchsorted(self.tk, t, side="right") - 1, 0, len(self.tk) - 2)
        dtk = self.tk[i + 1] - self.tk[i]
        a = np.where(dtk > 0, (self.vk[i + 1] - self.vk[i]) / np.where(dtk > 0, dtk, 1.0), 0.0)
        tau = np.clip(t - self.tk[i], 0.0, None)
        v = self.vk[i] + a * tau
        h = self.hk[i] + self.vk[i] * tau + 0.5 * a * tau * tau
        return h, v, np.where(t >= self.tk[-1], 0.0, a)

    def phases(self):
        """Ignition, burnout (the first deceleration after it) and apogee (the up velocity's zero), file s."""
        dv = np.diff(self.vk)
        k = np.flatnonzero(self.tk[:-1] >= self.ignition)
        burn = next((float(self.tk[j]) for j in k if dv[j] < 0), None)
        apo = None
        for j in k:
            if self.vk[j] > 0 >= self.vk[j + 1]:
                apo = float(self.tk[j] + (self.tk[j + 1] - self.tk[j]) * self.vk[j] / (self.vk[j] - self.vk[j + 1]))
                break
        return self.ignition, burn, apo


def pressure_pa(h):
    """The 1976 standard atmosphere's pressure (Pa) at a geopotential height (m), to 86 km."""
    layers = ((0.0, 101325.0, 288.15, -0.0065), (11000.0, 22632.06, 216.65, 0.0), (20000.0, 5474.889, 216.65, 0.001),
              (32000.0, 868.0187, 228.65, 0.0028), (47000.0, 110.9063, 270.65, 0.0), (51000.0, 66.93887, 270.65, -0.0028),
              (71000.0, 3.956420, 214.65, -0.002))
    M, R = 0.0289644, 8.3144598
    hb, pb, tb, lb = next(L for L in reversed(layers) if h >= L[0]) if h >= 0 else layers[0]
    if lb == 0.0:
        return pb * math.exp(-G0 * M * (h - hb) / (R * tb))
    return pb * (tb / (tb + lb * (h - hb))) ** (G0 * M / (R * lb))


def gravity(lat_rad, h):
    """Normal gravity (Somigliana) with the free-air terms, m/s^2, down."""
    s2 = math.sin(lat_rad) ** 2
    g0 = 9.7803253359 * (1 + 0.00193185265241 * s2) / math.sqrt(1 - E2_WGS * s2)
    return g0 * (1 - 2 * h / A_WGS * (1 + F_WGS + 0.00344978600308 - 2 * F_WGS * s2) + 3 * h * h / A_WGS ** 2)


def nose_up(heading_deg):
    """C_b2n, body FRD -> NED, for the nose up (body x = -D) and the belly (body z) toward heading_deg from north,
    and its quaternion (scalar first)."""
    p = math.radians(heading_deg)
    x = np.array([0.0, 0.0, -1.0])
    z = np.array([math.cos(p), math.sin(p), 0.0])
    c = np.stack([x, np.cross(z, x), z], axis=1)
    w = math.sqrt(max(0.0, 1 + np.trace(c))) / 2
    q = np.array([w,
                  math.copysign(math.sqrt(max(0.0, 1 + c[0, 0] - c[1, 1] - c[2, 2])) / 2, c[2, 1] - c[1, 2]),
                  math.copysign(math.sqrt(max(0.0, 1 - c[0, 0] + c[1, 1] - c[2, 2])) / 2, c[0, 2] - c[2, 0]),
                  math.copysign(math.sqrt(max(0.0, 1 - c[0, 0] - c[1, 1] + c[2, 2])) / 2, c[1, 0] - c[0, 1])])
    return c, q / np.linalg.norm(q)


def build(cfg, heading_deg=0.0, rate_hz=1000, tail_s=15.0, t_from=-30.0):
    """The sensors' columns (t_s from ignition) and the scenario for sim_replay_blob.py."""
    tr = Trajectory(cfg)
    ign, burn, apo = tr.phases()
    t = np.round(np.arange(int(round(t_from * rate_hz)), int(round((tr.duration - ign + tail_s) * rate_hz)))
                 / rate_hz, 9)
    in_scene = t + ign < tr.duration
    h, v, a = tr.at(t + ign)
    h_end = float(tr.at([tr.duration])[0][0])
    h = np.where(in_scene, h, h_end)
    v = np.where(in_scene, v, 0.0)
    a = np.where(in_scene, a, 0.0)
    lat = math.radians(tr.pad[0])
    c_b2n, q = nose_up(heading_deg)
    w_n = OMEGA_E * np.array([math.cos(lat), 0.0, -math.sin(lat)])
    v_n = np.stack([np.zeros_like(v), np.zeros_like(v), -v], axis=1)
    g = np.array([gravity(lat, x) for x in h])
    f_n = np.stack([np.zeros_like(a), np.zeros_like(a), -a - g], axis=1) + 2.0 * np.cross(w_n, v_n)
    f_b = f_n @ c_b2n            # C_n2b f = (f^T C_b2n)^T
    w_b = np.tile(c_b2n.T @ w_n, (len(t), 1))
    p = np.array([pressure_pa(x) for x in h])
    phase = np.full(len(t), PHASE_PAD)
    phase[t >= 0.0] = PHASE_BOOST
    if burn is not None:
        phase[t >= burn - ign] = PHASE_COAST
    if apo is not None:
        phase[t >= apo - ign] = PHASE_DROGUE
    phase[~in_scene] = PHASE_LANDED
    cols = {
        "t_s": t,
        "fx_mps2": f_b[:, 0], "fy_mps2": f_b[:, 1], "fz_mps2": f_b[:, 2],
        "wx_rps": w_b[:, 0], "wy_rps": w_b[:, 1], "wz_rps": w_b[:, 2],
        "qw": np.full(len(t), q[0]), "qx": np.full(len(t), q[1]),
        "qy": np.full(len(t), q[2]), "qz": np.full(len(t), q[3]),
        "phase": phase.astype(float),
        "acc_hg": (np.linalg.norm(f_b, axis=1) > HIGH_G).astype(float),
        "baro_p_pa": p, "baro_alt_m": h - tr.pad[2], "baro_alt_port_m": h - tr.pad[2],
    }
    c = cfg["time"]
    if c.get("type") != "UTC":
        raise SystemExit(f"time {c}: only UTC")
    start = dt.datetime(c["year"], c["month"], c["day"], c["hour"], c["minute"], int(c["second"]))
    scen = {
        "scenario": cfg["trajectory"].get("name", "signalsim"),
        "start_utc": start.isoformat(),
        "prologue_s": ign,
        "duration_s": tr.duration,
        "origin": {"lat_deg": tr.pad[0], "lon_deg": tr.pad[1], "alt_m": tr.pad[2]},
        "truth": {"ignition_s": 0.0, "burnout_s": None if burn is None else burn - ign,
                  "apogee_s": None if apo is None else apo - ign, "scene_end_s": tr.duration - ign,
                  "apogee_m": float(tr.at([apo])[0][0] - tr.pad[2]) if apo is not None else None},
    }
    return cols, scen


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("config", help="the SignalSim scene's config (JSON)")
    ap.add_argument("out", help="the output prefix: OUT_sensors.csv.gz and OUT_scenario.json")
    ap.add_argument("--heading-deg", type=float, default=0.0, help="the belly's (body z) azimuth from north")
    ap.add_argument("--tail-s", type=float, default=15.0, help="s of landed samples after the scene's end")
    ap.add_argument("--from", dest="t_from", type=float, default=-30.0, help="the first sample, s from ignition")
    a = ap.parse_args(argv)
    cols, scen = build(json.loads(Path(a.config).read_text()), a.heading_deg, tail_s=a.tail_s, t_from=a.t_from)
    names = list(cols)
    data = np.stack([cols[n] for n in names], axis=1)
    with gzip.open(f"{a.out}_sensors.csv.gz", "wt") as f:
        f.write(f"# {scen['scenario']}: signalsim_sensors.py from {Path(a.config).name}, body FRD, NED at the pad\n")
        f.write(",".join(names) + "\n")
        np.savetxt(f, data, delimiter=",", fmt="%.9g")
    Path(f"{a.out}_scenario.json").write_text(json.dumps(scen, indent=1) + "\n")
    tr = scen["truth"]
    print(f"{a.out}_sensors.csv.gz: {len(cols['t_s'])} samples, t {cols['t_s'][0]:.3f} .. {cols['t_s'][-1]:.3f} s; "
          f"burnout {tr['burnout_s']:.2f} s, apogee {tr['apogee_s']:.2f} s ({tr['apogee_m']:.0f} m), the scene ends "
          f"at {tr['scene_end_s']:.2f} s; peak {np.abs(cols['fx_mps2']).max():.1f} m/s^2")
    return 0


if __name__ == "__main__":
    sys.exit(main())
