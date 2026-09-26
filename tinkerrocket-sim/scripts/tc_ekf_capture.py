#!/usr/bin/env python3
"""Run the standalone EKF, GNSS-only, on a raw-measurement capture.

For receiver-only data -- a PX1105R on the bench, the COCOM rig's NEO-M8T --
where there is no IMU: the filter propagates a constant-velocity model and
takes every satellite's pseudorange and range rate as its own update
(``TcEkf.update_gnss_raw``), initialised from a least-squares fix.

Input is the rig's capture format ('<t> B <hex>' SkyTraq binary, '<t> U <hex>'
UBX), plain or .gz. It needs raw measurements AND navigation subframes: 0xE5 +
0xE0 on SkyTraq, RXM-RAWX + RXM-SFRBX on u-blox.

    PYTHONPATH=src python3 scripts/tc_ekf_capture.py bench_px1105r.log
    PYTHONPATH=src python3 scripts/tc_ekf_capture.py rig.log.gz \\
        --scenario rig.scenario.json --tow0 203400 --accel-psd 400 --no-tropo

With a scenario (the rig's truth), errors are scored against it and the
scenario's COCOM-blocked windows are skipped entirely. Without one, the filter
is compared with the receiver's own fix, and a static capture reports its
scatter about the mean.
"""
from __future__ import annotations

import argparse
import bisect
import gzip
import json
import math
import os
import sys
import tempfile

import numpy as np

from tinkerrocket_sim.estimation.tc_ekf import TcEkf, TcEkfParams, ecef2lla, lla2ecef, t_e2ned, spp_fix
from tinkerrocket_sim.estimation.gnss_raw import read_capture, corrected

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools", "gnss-cocom"))
from gnss_nmea_monitor import replay_source  # noqa: E402


class Truth:
    """The rig scenario's trajectory: fixed latitude, east drift, altitude."""

    def __init__(self, path):
        S = json.load(open(path))
        self.S, self.tr = S, S["truth"]
        self.ts = [s["t"] for s in self.tr]
        self.lat0 = math.radians(S["origin"]["lat_deg"]); self.lon0 = math.radians(S["origin"]["lon_deg"])
        self.east = [0.0]
        for a, b in zip(self.tr, self.tr[1:]):
            self.east.append(self.east[-1] + 0.5 * (a["v_east_mps"] + b["v_east_mps"]) * (b["t"] - a["t"]))

    def at(self, t):
        i = max(1, min(len(self.ts) - 1, bisect.bisect_left(self.ts, t)))
        a, b = self.tr[i - 1], self.tr[i]; w = (t - a["t"]) / (b["t"] - a["t"])
        f = lambda k: a[k] + w * (b[k] - a[k])
        e = self.east[i - 1] + w * (self.east[i] - self.east[i - 1])
        rew = 6378137.0 / math.sqrt(1 - 0.0066943799901 * math.sin(self.lat0) ** 2)
        lon = self.lon0 + e / ((rew + f("alt_m")) * math.cos(self.lat0))
        return lla2ecef(np.array([self.lat0, lon, f("alt_m")])), np.array([0.0, f("v_east_mps"), -f("v_up_mps")])

    def blocked(self, t, pad=2.0):
        return any(a - pad <= t <= b + pad for a, b in self.S.get("blocked_windows", []))


def open_capture(path):
    if path.endswith(".gz"):
        tmp = tempfile.NamedTemporaryFile("wb", suffix=".log", delete=False)
        tmp.write(gzip.open(path, "rb").read()); tmp.close()
        return tmp.name
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("--scenario", help="rig scenario JSON (truth)")
    ap.add_argument("--tow0", type=float, help="GPS time of week of the scenario start")
    ap.add_argument("--accel-psd", dest="accel_psd", type=float, default=4.0,
                    help="white-acceleration PSD, m^2/s^3 (static/walking ~1-4, a 3 g boost ~400)")
    ap.add_argument("--no-tropo", dest="tropo", action="store_false",
                    help="skip the troposphere model (the gps-sdr-sim rig has none)")
    ap.add_argument("--clk-bias-psd", dest="clk_bias_psd", type=float, default=None,
                    help="receiver clock-bias PSD, m^2/s (default: the TCXO value). Raise it "
                         "(~1) where code and Doppler disagree on the clock rate, as they do on "
                         "the HackRF rig (0.5 m/s on the M8T, 4 m/s on the LC86G)")
    ap.add_argument("--stride", type=int, default=1,
                    help="use every Nth raw epoch (a 20 Hz hour is 72,000 epochs)")
    ap.add_argument("--systems", default="GEC", help="constellations to use: any of G, E, C")
    ap.add_argument("--pr-model", dest="pr_model", choices=("tuned", "white"), default="tuned",
                    help="tuned: white + correlated pseudorange error, inflated per update "
                         "interval; white: the same total sigma fused as if independent")
    ap.add_argument("--rr-b", dest="rr_b", type=float, help="override the range-rate C/N0 coefficient (m/s)")
    ap.add_argument("--rr-tau", dest="rr_tau", type=float, help="override the range-rate correlation time (s)")
    ap.add_argument("--no-doppler-fix", dest="doppler_fix", action="store_false",
                    help="leave SkyTraq's truncated whole-hertz Doppler as reported")
    ap.add_argument("--csv", help="write the filter track here")
    args = ap.parse_args()

    import tinkerrocket_sim.estimation.gnss_raw as _gr
    if args.rr_b is not None:
        _gr.RR_WHITE = (_gr.RR_WHITE[0], args.rr_b)
    if args.rr_tau is not None:
        _gr.RR_TAU_S = args.rr_tau
    eph, epochs, own, kind = read_capture(open_capture(args.capture), replay_source, systems=args.systems,
                                          fix_doppler_truncation=args.doppler_fix)
    epochs = epochs[::max(1, args.stride)]
    print(f"{os.path.basename(args.capture)}: {kind}, {len(epochs)} raw epochs, ephemeris for "
          f"{len(eph.eph)} PRNs, iono model {'decoded' if eph.ion else 'absent'}, "
          f"{len(own)} own fixes")
    truth = Truth(args.scenario) if args.scenario else None
    if truth and args.tow0 is None:
        sys.exit("--scenario needs --tow0")

    prm = TcEkfParams()
    if args.clk_bias_psd is not None:
        prm.clk_bias_psd = args.clk_bias_psd
    ekf = TcEkf(prm)
    started, t_prev, track, rejects = False, None, [], 0
    nis_pr, nis_rr, sig = [], [], []
    for tow, obs in epochs:
        t_s = tow - args.tow0 if truth else tow
        if truth and (t_s < 0 or truth.blocked(t_s)):
            continue                                   # COCOM-blocked windows: not processed
        if not started:
            meas = corrected(tow, obs, eph, None, use_tropo=False)
            fx = spp_fix(meas)
            if fx is None or fx[1] is None:
                continue
            meas = corrected(tow, obs, eph, fx[0], use_tropo=args.tropo)
            fx = spp_fix(meas, fx[0])
            if fx is None or fx[1] is None:
                continue
            lla = ecef2lla(fx[0])
            ekf.init(lla, t_e2ned(lla[0], lla[1]) @ fx[1], [1.0, 0.0, 0.0, 0.0])
            ekf.freeze_imu_states()
            ekf.init_clock_from(meas)
            started, t_prev = True, tow
            continue
        ekf.propagate_kinematic(tow - t_prev, args.accel_psd)
        t_prev = tow
        r, _, _ = ekf.receiver_state_ecef()
        meas = corrected(tow, obs, eph, r, use_tropo=args.tropo,
                         doppler_step_hz=1.0 if kind == "skytraq" else 0.0)
        if args.pr_model == "white":
            import math as _m
            for x in meas:
                x.sigma_pr = _m.hypot(x.sigma_pr, x.sigma_pr_corr)
                x.sigma_pr_corr, x.tau_rr = 0.0, 0.0
        st = ekf.update_gnss_raw(meas)
        nis_pr += st.nis_pr; nis_rr += st.nis_rr
        rejects += st.rejected_pr + st.rejected_rr
        if st.clock_jump_ms:
            print(f"  receiver clock stepped {st.clock_jump_ms:+d} ms at TOW {tow:.1f}; clock state shifted")
        r, v, _ = ekf.receiver_state_ecef()
        track.append((tow, t_s, r.copy(), v.copy(), len(meas), st.rejected_pr + st.rejected_rr))
        sig.append((math.hypot(math.sqrt(ekf.P[0, 0]), math.sqrt(ekf.P[1, 1])), math.sqrt(ekf.P[2, 2])))

    if not track:
        sys.exit("never got a first fix: need >= 4 satellites with pseudorange, Doppler and ephemeris")
    print(f"filter ran {len(track)} epochs, {rejects} measurements gated out")
    for name, v in (("pseudorange", nis_pr), ("range rate", nis_rr)):
        if v:
            a = np.array(v)
            print(f"  {name:11s} NIS: mean {a.mean():6.2f} (1.0 if consistent), median {np.median(a):5.2f} "
                  f"(0.45), > 10.83: {100 * np.mean(a > 10.83):5.2f} % (0.1)")

    own_t = [o[0] for o in own]
    def own_at(tow):
        if not own_t:
            return None
        i = min(range(max(0, bisect.bisect_left(own_t, tow) - 1), min(len(own_t), bisect.bisect_left(own_t, tow) + 1)),
                key=lambda k: abs(own_t[k] - tow))
        return own[i] if abs(own_t[i] - tow) < 0.6 else None

    def stats(label, rows):
        if not rows:
            return
        a = np.array(rows)
        hz, up, v = np.hypot(a[:, 0], a[:, 1]), np.abs(a[:, 2]), a[:, 3]
        p = lambda x, q: np.sort(x)[min(len(x) - 1, int(q * len(x)))]
        print(f"  {label:30s} n {len(a):4d}  horiz p50/p95 {p(hz,.5):6.1f} {p(hz,.95):7.1f} m   "
              f"up p50/p95 {p(up,.5):6.1f} {p(up,.95):7.1f} m   |dv| p50/p95 {p(v,.5):5.2f} {p(v,.95):6.2f} m/s")

    if truth:
        pro = truth.S.get("prologue_s", 0.0)
        segs = {"pad": [], "flight": []}
        segs_own = {"pad": [], "flight": []}
        for tow, t_s, r, v, n, _ in track:
            xt, vt_ned = truth.at(t_s)
            lla = ecef2lla(xt); T = t_e2ned(lla[0], lla[1])
            d = T @ (r - xt); dv = np.linalg.norm(T @ v - vt_ned)
            seg = "pad" if t_s < pro - 2 else "flight"
            segs[seg].append((d[1], d[0], -d[2], dv))
            o = own_at(tow)
            if o is not None:
                do = T @ (o[1] - xt); dvo = np.linalg.norm(T @ o[2] - vt_ned)
                segs_own[seg].append((do[1], do[0], -do[2], dvo))
        print("\nagainst the scenario truth (COCOM-blocked windows skipped):")
        for seg in ("pad", "flight"):
            stats(f"{seg}: standalone EKF (raw)", segs[seg])
            stats(f"{seg}: receiver's own fix", segs_own[seg])
    else:
        diffs = []
        for tow, _, r, v, n, _ in track:
            o = own_at(tow)
            if o is not None:
                lla = ecef2lla(r); T = t_e2ned(lla[0], lla[1]); d = T @ (r - o[1])
                diffs.append((d[1], d[0], -d[2], np.linalg.norm(T @ (v - o[2]))))
        print("\nagainst the receiver's own fix:")
        stats("EKF minus own fix", diffs)
        R = np.array([x[2] for x in track]); mean = R.mean(axis=0)
        lla = ecef2lla(mean); T = t_e2ned(lla[0], lla[1])
        sc = np.array([T @ (x - mean) for x in R])
        sh = np.median([x[0] for x in sig]); sv = np.median([x[1] for x in sig])
        print(f"\nthe filter's own sigma (median): horiz {sh:.2f} m, vert {sv:.2f} m "
              f"(scatter/sigma: {np.sqrt(np.mean(sc[:,0]**2 + sc[:,1]**2)) / sh:.2f} horiz, "
              f"{np.sqrt(np.mean(sc[:,2]**2)) / sv:.2f} vert -- 1 if the filter's uncertainty is honest)")
        print(f"scatter about the mean position (meaningful only if the antenna was still): "
              f"horiz rms {np.sqrt(np.mean(sc[:,0]**2 + sc[:,1]**2)):.2f} m, "
              f"vert rms {np.sqrt(np.mean(sc[:,2]**2)):.2f} m; mean lat/lon/h "
              f"{math.degrees(lla[0]):.7f} {math.degrees(lla[1]):.7f} {lla[2]:.1f} m")
    if args.csv:
        with open(args.csv, "w") as f:
            f.write("tow,x,y,z,vx,vy,vz,n_sats,rejected\n")
            for tow, _, r, v, n, rj in track:
                f.write(f"{tow:.3f},{r[0]:.3f},{r[1]:.3f},{r[2]:.3f},{v[0]:.3f},{v[1]:.3f},{v[2]:.3f},{n},{rj}\n")


if __name__ == "__main__":
    main()
