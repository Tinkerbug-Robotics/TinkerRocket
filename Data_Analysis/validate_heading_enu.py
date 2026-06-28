#!/usr/bin/env python3
"""#313: validate the EKF's initial heading + body->world (ENU) transform.

The EKF dead-reckons an ENU trajectory seeded with a pad attitude/heading; GPS
gives a largely independent ENU track. If the EKF horizontal track is rotated by
a near-constant angle from the GPS track, that angle is (to first order) the
initial-heading error. A vertical early boost in ENU + agreement on coast/descent
says the body->ENU rotation is pointed the right way.

GPS is corrupted during boost (post-#174 position dives), so the heading offset
is fit on the coast+descent window only.

Usage: validate_heading_enu.py <flight1.bin> [flight2.bin ...]
Reuses the replay + LLA->ENU helpers from _ekf_replay (shares TR_GpsInsEKF.cpp
with firmware) and parse_binary_file from plot_flight_data_mini.
"""
import sys
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
from plot_flight_data_mini import parse_binary_file          # noqa: E402
from _ekf_replay import replay_binary, lla_rad_to_enu_m, DEG2RAD  # noqa: E402


def enu_track(t, lat, lon, alt, ref, lat_in_deg=False):
    e, n, u = [], [], []
    for i in range(len(t)):
        la = lat[i] * DEG2RAD if lat_in_deg else lat[i]
        lo = lon[i] * DEG2RAD if lat_in_deg else lon[i]
        ee, nn, uu = lla_rad_to_enu_m(la, lo, alt[i], ref)
        e.append(ee); n.append(nn); u.append(uu)
    return np.array(e), np.array(n), np.array(u)


def best_fit_rotation_deg(a_e, a_n, b_e, b_n):
    """Least-squares angle theta (deg) such that R(theta)*A ~ B about the launch
    origin (closed-form 2D Procrustes; translation is fixed at the launch point)."""
    num = float(np.sum(a_e * b_n - a_n * b_e))
    den = float(np.sum(a_e * b_e + a_n * b_n))
    return math.degrees(math.atan2(num, den))


def analyze(bin_path: Path):
    records, _, _ = parse_binary_file(str(bin_path))
    res = replay_binary(records, verbose=False)
    t0 = res.phases.t0_us

    ekf_t = (np.asarray(res.ekf.t_us, float) - t0) / 1e6
    ekf_e, ekf_n, _ = enu_track(res.ekf.t_us, res.ekf.lat_rad, res.ekf.lon_rad,
                                res.ekf.alt_m, res.launch_ref)
    ekf_vu = -np.asarray(res.ekf.vd, float)
    ekf_ve = np.asarray(res.ekf.ve, float)
    ekf_vn = np.asarray(res.ekf.vn, float)

    g_t = (np.asarray(res.gnss.t_us, float) - t0) / 1e6
    g_e, g_n, _ = enu_track(res.gnss.t_us, res.gnss.lat_deg, res.gnss.lon_deg,
                            res.gnss.alt_m, res.launch_ref, lat_in_deg=True)

    boost_end_s = ((res.phases.boost_end_us - t0) / 1e6
                   if res.phases.boost_end_us else 2.0)
    apogee_s = ((res.phases.apogee_us - t0) / 1e6
                if res.phases.apogee_us else None)

    # ---- heading offset: rotate EKF onto GPS on the GPS-reliable window ----
    # skip boost (+0.5 s margin) and require meaningful horizontal displacement
    win = (g_t > boost_end_s + 0.5) & (np.hypot(g_e, g_n) > 5.0)
    theta = rms_b = rms_a = float("nan")
    n_pts = int(win.sum())
    re = rn = None
    if n_pts >= 10:
        gt_w, ge_w, gn_w = g_t[win], g_e[win], g_n[win]
        ee_w = np.interp(gt_w, ekf_t, ekf_e)
        en_w = np.interp(gt_w, ekf_t, ekf_n)
        theta = best_fit_rotation_deg(ee_w, en_w, ge_w, gn_w)
        rms_b = float(np.sqrt(np.mean((ee_w - ge_w) ** 2 + (en_w - gn_w) ** 2)))
        th = math.radians(theta)
        rew = ee_w * math.cos(th) - en_w * math.sin(th)
        rnw = ee_w * math.sin(th) + en_w * math.cos(th)
        rms_a = float(np.sqrt(np.mean((rew - ge_w) ** 2 + (rnw - gn_w) ** 2)))

    # ---- net-displacement bearing offset (launch -> apogee), cross-check ----
    bearing_off = float("nan")
    if apogee_s is not None and len(ekf_t) and len(g_t):
        ia = int(np.argmin(np.abs(ekf_t - apogee_s)))
        ig = int(np.argmin(np.abs(g_t - apogee_s)))
        be = math.degrees(math.atan2(ekf_e[ia], ekf_n[ia]))   # bearing E of N
        bg = math.degrees(math.atan2(g_e[ig], g_n[ig]))
        bearing_off = (be - bg + 180.0) % 360.0 - 180.0

    # ---- velocity-heading offset: EKF vs GPS horizontal velocity direction ----
    # Velocity is NOT integrated, so this isolates a true heading error from
    # accumulated position dead-reckoning drift (which the rotation fit conflates).
    gve = np.asarray(res.gnss.ve, float); gvn = np.asarray(res.gnss.vn, float)
    vh_off = float("nan")
    vwin = (g_t > boost_end_s + 0.5) & (np.hypot(gve, gvn) > 2.0)
    if vwin.sum() >= 10:
        eve = np.interp(g_t[vwin], ekf_t, ekf_ve)
        evn = np.interp(g_t[vwin], ekf_t, ekf_vn)
        d = (np.degrees(np.arctan2(eve, evn))
             - np.degrees(np.arctan2(gve[vwin], gvn[vwin])) + 180.0) % 360.0 - 180.0
        vh_off = float(np.median(d))

    # ---- transform check: early-boost velocity should be ~vertical in ENU ----
    bwin = (ekf_t > 0.2) & (ekf_t < boost_end_s)
    boost_elev = float("nan")
    if bwin.sum() >= 5:
        horiz = np.hypot(ekf_ve[bwin], ekf_vn[bwin])
        boost_elev = float(np.median(np.degrees(np.arctan2(ekf_vu[bwin], horiz))))

    # ---- plot ground-track overlay ----
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(g_e, g_n, ".-", color="C0", ms=3, lw=0.7, label="GPS")
    ax.plot(ekf_e, ekf_n, "-", color="C3", lw=1.3, label="EKF")
    if not math.isnan(theta):
        th = math.radians(theta)
        ax.plot(ekf_e * math.cos(th) - ekf_n * math.sin(th),
                ekf_e * math.sin(th) + ekf_n * math.cos(th),
                "--", color="C2", lw=1.0, label=f"EKF de-rotated {theta:+.1f}°")
    ax.plot(0, 0, "k+", ms=12)
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.legend()
    ax.set_title(f"{bin_path.parent.name.strip()} / {bin_path.name}\n"
                 f"heading offset {theta:+.1f}° (n={n_pts}), boost elev {boost_elev:.0f}°")
    outdir = _HERE.parent / "plots"; outdir.mkdir(exist_ok=True)
    out = outdir / f"heading_enu_{bin_path.stem}.png"
    fig.savefig(out, dpi=110, bbox_inches="tight"); plt.close(fig)

    return dict(flight=f"{bin_path.parent.name.strip()}/{bin_path.name}",
                theta=theta, n=n_pts, rms_b=rms_b, rms_a=rms_a,
                bearing_off=bearing_off, vh_off=vh_off, boost_elev=boost_elev,
                plot=str(out))


def main():
    if len(sys.argv) < 2:
        print("usage: validate_heading_enu.py <flight.bin> [...]", file=sys.stderr)
        sys.exit(1)
    rows = []
    for p in sys.argv[1:]:
        try:
            rows.append(analyze(Path(p)))
        except Exception as e:  # noqa: BLE001
            print(f"  {p}: FAILED ({type(e).__name__}: {e})", file=sys.stderr)

    print(f"\n{'flight':40s} {'rot_fit':>8s} {'rms_a':>7s} {'net_brg':>8s} "
          f"{'vel_hdg':>8s} {'boost_el':>9s}")
    print("  (rot_fit/rms_a = position-track rotation + residual; net_brg = "
          "launch→apogee bearing; vel_hdg = velocity-heading offset, the clean one)")
    for r in rows:
        print(f"{r['flight'][-40:]:40s} {r['theta']:+7.1f}° {r['rms_a']:6.1f}m "
              f"{r['bearing_off']:+7.1f}° {r['vh_off']:+7.1f}° {r['boost_elev']:7.0f}°")
    valid = [r for r in rows if not math.isnan(r["vh_off"])]
    if valid:
        vh = np.array([r["vh_off"] for r in valid])
        print(f"\nVELOCITY-heading offset (drift-free, the clean metric): "
              f"mean {np.mean(vh):+.1f}°, |mean| {np.mean(np.abs(vh)):.1f}°, "
              f"spread {np.std(vh):.1f}° (n={len(valid)})")
    be = np.array([r["boost_elev"] for r in rows if not math.isnan(r["boost_elev"])])
    if len(be):
        print(f"early-boost ENU elevation: mean {np.mean(be):.0f}° "
              f"(90° = perfectly vertical → transform up-axis correct)")


if __name__ == "__main__":
    main()
