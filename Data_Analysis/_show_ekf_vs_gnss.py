#!/usr/bin/env python3
"""EKF vs GNSS truth comparison for a flight binary.

Replays the binary through the (rebuilt) firmware EKF, then overlays
EKF position/velocity against the raw GNSS fixes used as truth.  Useful
for spotting dead-reckoning divergence during GNSS dropouts.
"""
import sys, math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("MacOSX")
import matplotlib.pyplot as plt

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "tinkerrocket-sim" / "src"))

from plot_flight_data_mini import parse_binary_file
from _ekf_replay import replay_binary, lla_rad_to_enu_m, DEG2RAD


def render(bin_path: Path, title: str):
    records, _, _ = parse_binary_file(str(bin_path))
    res = replay_binary(records, verbose=False)
    t0 = res.phases.t0_us

    ekf_t = (res.ekf.t_us - t0) / 1e6
    ekf_e, ekf_n, ekf_u = [], [], []
    for i in range(len(res.ekf.t_us)):
        e, n, u = lla_rad_to_enu_m(res.ekf.lat_rad[i], res.ekf.lon_rad[i],
                                    res.ekf.alt_m[i], res.launch_ref)
        ekf_e.append(e); ekf_n.append(n); ekf_u.append(u)
    ekf_e = np.array(ekf_e); ekf_n = np.array(ekf_n); ekf_u = np.array(ekf_u)
    ekf_vn = res.ekf.vn; ekf_ve = res.ekf.ve; ekf_vu = -res.ekf.vd

    g_t = (res.gnss.t_us - t0) / 1e6
    g_e, g_n, g_u = [], [], []
    for i in range(len(res.gnss.t_us)):
        e, n, u = lla_rad_to_enu_m(res.gnss.lat_deg[i] * DEG2RAD,
                                    res.gnss.lon_deg[i] * DEG2RAD,
                                    res.gnss.alt_m[i], res.launch_ref)
        g_e.append(e); g_n.append(n); g_u.append(u)
    g_e = np.array(g_e); g_n = np.array(g_n); g_u = np.array(g_u)

    boost_s = ((res.phases.boost_start_us - t0)/1e6
               if res.phases.boost_start_us else None)
    boost_e = ((res.phases.boost_end_us - t0)/1e6
               if res.phases.boost_end_us else None)
    apogee_s = ((res.phases.apogee_us - t0)/1e6
                if res.phases.apogee_us else None)

    def shade(ax):
        if boost_s and boost_e:
            ax.axvspan(boost_s, boost_e, color="red", alpha=0.08)
        if apogee_s:
            ax.axvline(apogee_s, color="blue", ls=":", lw=0.8, alpha=0.6)

    fig, axes = plt.subplots(3, 2, figsize=(15, 11), sharex=True)
    fig.suptitle(title, fontsize=13, fontweight="bold")

    # Left column: position E / N / U
    for ax, (label, ekf_v, gnss_v) in zip(
        axes[:, 0],
        [("East (m)", ekf_e, g_e),
         ("North (m)", ekf_n, g_n),
         ("Up (m)", ekf_u, g_u)],
    ):
        ax.plot(ekf_t, ekf_v, "-", color="C3", lw=1.2, label="EKF")
        ax.scatter(g_t, gnss_v, c="C0", s=10, alpha=0.6, label="GNSS")
        ax.set_ylabel(label)
        shade(ax)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9, loc="best")

    # Right column: velocity VE / VN / VU
    g_vn = res.gnss.vn; g_ve = res.gnss.ve; g_vu = res.gnss.vu
    for ax, (label, ekf_v, gnss_v) in zip(
        axes[:, 1],
        [("VE (m/s)", ekf_ve, g_ve),
         ("VN (m/s)", ekf_vn, g_vn),
         ("VU (m/s)", ekf_vu, g_vu)],
    ):
        ax.plot(ekf_t, ekf_v, "-", color="C3", lw=1.2, label="EKF")
        ax.scatter(g_t, gnss_v, c="C0", s=10, alpha=0.6, label="GNSS")
        ax.set_ylabel(label)
        shade(ax)
        ax.grid(True, alpha=0.3)

    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    plt.tight_layout()

    out = _HERE.parent / "plots" / f"ekf_vs_gnss_{bin_path.parent.name.strip().replace(' ', '_')}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160, bbox_inches="tight")
    print(f"Saved {out}")
    return fig


BASE = Path("/Users/christianpedersen/Documents/Hobbies/ModelRockets/TestFlights/2026_05_17")

for fdir, label in [("Eagle Claw", "Eagle Claw"),
                     ("RIM-66_65mm ", "RIM-66")]:
    bins = sorted((BASE / fdir).glob("flight_*.bin"))
    if not bins:
        continue
    render(bins[0], f"{label} (2026-05-17) — EKF vs GNSS truth   (red shading=boost, blue dashed=apogee)")

plt.show()
