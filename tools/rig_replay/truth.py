"""A rig-replay flight's truth, for the scorers in this folder.

A flight is two files the gnss repo's cocom-rig/sdr/sim_flight_3d.py writes
(TinkerRocket-GNSS): its sensors CSV (scenarios/sim_<id>_sensors.csv.gz: 1 kHz,
NED at the pad, body FRD, with a phase column) and its scenario JSON
(scenarios/real_sim_<id>_sky3d_pad180.json: the pad, the file's start and
prologue). Everything flight-specific comes from them: the pad, the ignition's
GPS time and the phases' edges.
"""
import datetime as dt
import gzip
import json
import math
import os
import subprocess

import numpy as np

A, E2 = 6378137.0, 0.0066943799901          # WGS84
PHASES = ("pad", "rail", "boost", "coast", "drogue", "main", "landed")


def cocom_sdr():
    """The rig's folder: $COCOM_SDR, else the monorepo's gnss submodule."""
    if os.environ.get("COCOM_SDR"):
        return os.environ["COCOM_SDR"]
    top = subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True,
                         cwd=os.path.dirname(os.path.abspath(__file__))).stdout.strip()
    return os.path.join(top, "gnss", "cocom-rig", "sdr")


def flight_files(flight):
    """(sensors CSV, scenario JSON) for a flight id such as sim_rp5_0829."""
    scen = os.path.join(cocom_sdr(), "scenarios")
    return (os.path.join(scen, f"{flight}_sensors.csv.gz"),
            os.path.join(scen, f"real_{flight}_sky3d_pad180.json"))


class Truth:
    """The flight's NED position and velocity at any t (s from ignition), its
    phases, and the scenario's pad and ignition time."""

    def __init__(self, csv_path, scenario_path=None):
        lines = [ln for ln in gzip.open(csv_path, "rt") if not ln.startswith("#")]
        names = lines[0].strip().split(",")
        cols = ("t_s", "pn_m", "pe_m", "pd_m", "vn_mps", "ve_mps", "vd_mps", "phase")
        self.d = np.loadtxt(lines[1:], delimiter=",", usecols=[names.index(c) for c in cols])
        self.t = self.d[:, 0]
        ph = self.d[:, 7].astype(int)
        # the first instant of each phase (pad first, at the file's start)
        self.edges = {PHASES[0]: float(self.t[0])}
        for i in np.flatnonzero(np.diff(ph)) + 1:
            self.edges.setdefault(PHASES[ph[i]], float(self.t[i]))
        self.end = float(self.t[-1])
        self.scenario = json.load(open(scenario_path)) if scenario_path else None

    def at(self, t):
        """NED position and velocity (n x 6) at the times t."""
        t = np.asarray(t, float)
        return np.stack([np.interp(t, self.t, self.d[:, k]) for k in range(1, 7)], axis=1)

    def windows(self):
        """(label, from, to) for each flown phase, with the ground after landing."""
        e = self.edges
        out = [("pad", max(e["pad"], -60.0), e.get("rail", 0.0)),
               ("boost", e.get("rail", 0.0), e["coast"]),
               ("coast", e["coast"], e["drogue"]),
               ("drogue", e["drogue"], e.get("main", e["landed"]))]
        if "main" in e:
            out.append(("main", e["main"], e["landed"]))
        out.append(("ground", e["landed"], self.end))
        return out

    def pad(self):
        """lat, lon (deg), height above the ellipsoid (m) of the scenario's pad."""
        o = self.scenario["origin"]
        return float(o["lat_deg"]), float(o["lon_deg"]), float(o["alt_m"])

    def ignition_utc(self):
        """The ignition as a UTC datetime: the file's start plus its prologue."""
        s = self.scenario
        start = dt.datetime.fromisoformat(s["start_utc"].replace("Z", ""))
        return start + dt.timedelta(seconds=float(s["prologue_s"]))

    def ned_from_lla(self, lat, lon, h):
        """NED (m) at the pad of positions given in deg, deg, m (ellipsoidal)."""
        lat0, lon0, h0 = self.pad()
        s0 = math.sin(math.radians(lat0))
        w0 = math.sqrt(1 - E2 * s0 * s0)
        rn = A * (1 - E2) / w0 ** 3 + h0
        re = A / w0 + h0
        return np.stack([np.radians(np.asarray(lat) - lat0) * rn,
                         np.radians(np.asarray(lon) - lon0) * re * math.cos(math.radians(lat0)),
                         -(np.asarray(h) - h0)], axis=1)
