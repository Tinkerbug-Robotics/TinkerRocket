#!/usr/bin/env python3
"""#552 — why the filter's horizontal velocity goes wrong in coast.

Takes flight binaries, and for each one plots the filter's horizontal velocity
against the GNSS fixes it was being fed, the resulting error, and — the part
that turned out to matter — the filter's OWN declared velocity uncertainty
beside the GNSS quality it was reading.

Two things this exists to separate, because they look identical in a single
error number and have opposite fixes:

  TUMBLE      the filter flies apart while the GNSS stays smooth and healthy.
              Sustained coast error tracks peak body rate.
  BAD GNSS    the GNSS teleports while the filter stays smooth, and the filter
              is then dragged toward it over a few seconds.

Usage:
    python3 analyze_coast_velocity_552.py <flight.bin> [<flight.bin> ...] \
        [--labels "Name A,Name B"] [--outdir DIR]

Written for the 2026-08-29 BARC set (four flights, one launch day, so weather
and site are controlled). Nothing here is specific to those flights.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from plot_flight_data_mini import parse_binary_file

# Skip this much after burnout before measuring: the burnout transient is a
# real, brief, well-understood disagreement (GNSS velocity lags a step change
# in acceleration) and it is not what #552 is about.
SETTLE_S = 0.5

# A GNSS gap longer than this is drawn as an outage band.
GAP_S = 0.5


def load(path: Path):
    rec, _, _ = parse_binary_file(str(path))
    ns = [r for r in rec.get("NonSensor", []) if r.get("time_us") is not None]
    if not ns:
        raise SystemExit(f"{path}: no NonSensor rows")
    t0 = next((r["time_us"] for r in ns if r.get("launch")), ns[0]["time_us"])
    return rec, ns, t0


def col(rows, key, t0=None):
    if key == "t":
        return (np.array([r["time_us"] for r in rows], float) - t0) / 1e6
    return np.array([float(r.get(key) or 0.0) for r in rows], float)


def analyse(path: Path, label: str):
    rec, ns, t0 = load(path)
    tns = col(ns, "t", t0)
    ve, vn, upos = col(ns, "e_vel"), col(ns, "n_vel"), col(ns, "u_pos")

    # Coast runs burnout -> the MEASURED altitude peak.
    #
    # NOT burnout -> apogee_flag. The master apogee vote fires deliberately
    # early because it drives pyro, and early beats late: 1.9 s early on
    # RIM-66 and 2.3 s on Eagle Claw. Using the flag cuts the window to a
    # sliver that misses the excursion entirely and makes the worst flight in
    # this set look like the best.
    t_apogee = tns[int(np.argmax(upos))]
    t_burn = next(((r["time_us"] - t0) / 1e6 for r in ns if r.get("burnout")), 0.0)
    t_dep = next(((r["time_us"] - t0) / 1e6 for r in ns if r.get("deployed")), None)
    lo, hi = t_burn + SETTLE_S, t_apogee

    g = [r for r in rec.get("GNSS", []) if (r.get("num_sats") or 0) >= 4]
    tg = col(g, "t", t0)
    gve, gvn = col(g, "vel_e"), col(g, "vel_n")
    sats, pdop, hacc = col(g, "num_sats"), col(g, "pdop"), col(g, "h_acc_m")
    err = np.hypot(np.interp(tg, tns, ve) - gve, np.interp(tg, tns, vn) - gvn)

    # The filter's own declared 1-sigma velocity uncertainty.
    snap = [r for r in rec.get("Snapshot", []) if r.get("time_us") is not None]
    tsn = col(snap, "t", t0) if snap else np.array([])
    sig = (np.sqrt(np.maximum(col(snap, "p_vel_e"), 0) + np.maximum(col(snap, "p_vel_n"), 0))
           if snap else np.array([]))

    im = rec.get("ISM6HG256") or []
    ti = col(im, "t", t0) if im else np.array([])
    rate = (np.sqrt(sum(col(im, f"gyro_{a}") ** 2 for a in "xyz")) if im else np.array([]))

    m = (tg >= lo) & (tg <= hi)
    mi = (ti >= lo) & (ti <= hi) if ti.size else np.array([], bool)
    e = np.sort(err[m]) if m.any() else np.array([])

    return dict(
        label=label, tns=tns, ve=ve, vn=vn, tg=tg, gve=gve, gvn=gvn, err=err,
        sats=sats, pdop=pdop, hacc=hacc, tsn=tsn, sig=sig, coast=m,
        t_burn=t_burn, t_apogee=t_apogee, t_dep=t_dep, lo=lo, hi=hi,
        # "sustained" = median of the larger half. A single-epoch spike the
        # filter shrugs off does not dead-reckon into a landing miss; a level
        # it holds for seconds does.
        sustained=float(np.median(e[len(e) // 2:])) if e.size else float("nan"),
        median=float(np.median(err[m])) if m.any() else float("nan"),
        peak_rate=float(rate[mi].max()) if mi.any() else float("nan"),
        min_sats=float(sats[m].min()) if m.any() else float("nan"),
        max_hacc=float(hacc[m].max()) if m.any() else float("nan"),
        gnss_jump=float(np.max(np.hypot(np.diff(gve[m]), np.diff(gvn[m]))))
        if m.sum() > 2 else float("nan"),
        gaps=[(tg[i], tg[i + 1]) for i in np.where(np.diff(tg) > GAP_S)[0]] if tg.size > 1 else [],
    )


def plot_one(r, outdir: Path):
    tg, err, coast = r["tg"], r["err"], r["coast"]
    s_at = np.interp(tg, r["tsn"], r["sig"]) if r["tsn"].size else np.full_like(tg, np.nan)
    xlim = (max(-1.0, r["lo"] - 3), r["hi"] + 3)

    fig, ax = plt.subplots(4, 1, figsize=(12, 12), sharex=True,
                           gridspec_kw=dict(height_ratios=[3, 2, 2, 2]))
    fig.suptitle(f"#552  {r['label']}   coast = burnout+{SETTLE_S:g}s .. measured apogee (shaded)",
                 fontsize=13, weight="bold")
    for a in ax:
        a.axvspan(r["lo"], r["hi"], color="0.88", zorder=0)
        a.axvline(r["t_burn"], color="tab:orange", lw=1, ls="--")
        if r["t_dep"] is not None:
            a.axvline(r["t_dep"], color="tab:brown", lw=1.2, ls="-.")
        for g0, g1 in r["gaps"]:
            a.axvspan(g0, g1, color="tab:red", alpha=.18, zorder=0)

    ax[0].plot(r["tns"], r["ve"], color="tab:blue", lw=1, label="EKF vE")
    ax[0].plot(r["tns"], r["vn"], color="tab:cyan", lw=1, label="EKF vN")
    ax[0].plot(tg, r["gve"], "o", ms=3, color="tab:red", label="GNSS vE")
    ax[0].plot(tg, r["gvn"], "o", ms=3, color="tab:orange", label="GNSS vN")
    ax[0].set_ylabel("horizontal velocity (m/s)"); ax[0].legend(ncol=4, fontsize=8)

    ax[1].plot(tg, err, "-o", ms=3, color="tab:red", label="|EKF − GNSS|")
    ax[1].fill_between(tg, 0, s_at, color="tab:green", alpha=.35,
                       label="filter's own 1σ (note the scale)")
    ax[1].set_ylabel("velocity error (m/s)"); ax[1].legend(fontsize=8)

    with np.errstate(divide="ignore", invalid="ignore"):
        ax[2].plot(tg, err / np.where(s_at > 1e-6, s_at, np.nan), "-o", ms=3, color="tab:purple")
    ax[2].axhline(3, color="k", ls="--", lw=.8); ax[2].set_yscale("log")
    ax[2].set_ylabel("error / σ")

    ax[3].plot(tg, r["sats"], color="tab:green", label="num_sats")
    ax[3].plot(tg, r["pdop"], color="tab:blue", label="PDOP")
    ax[3].plot(tg, r["hacc"], color="tab:red", label="h_acc (m)")
    ax[3].set_ylabel("GNSS quality"); ax[3].set_xlabel("t since launch (s)"); ax[3].legend(fontsize=8)
    for a in ax:
        a.grid(alpha=.3)
    ax[0].set_xlim(*xlim)

    out = outdir / f"552_{r['label'].replace(' ', '_').replace('/', '_')}.png"
    fig.tight_layout(); fig.savefig(out, dpi=110); plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bins", nargs="+", type=Path)
    ap.add_argument("--labels", default="", help="comma-separated display names")
    ap.add_argument("--outdir", type=Path, default=Path("."))
    a = ap.parse_args()
    labels = [s.strip() for s in a.labels.split(",")] if a.labels else []
    a.outdir.mkdir(parents=True, exist_ok=True)

    rows = []
    for i, b in enumerate(a.bins):
        label = labels[i] if i < len(labels) else b.stem
        r = analyse(b, label)
        rows.append(r)
        print("wrote", plot_one(r, a.outdir))

    print(f"\n{'flight':<22} {'median':>7} {'sustained':>10} {'peak rate':>10} "
          f"{'m/s /1000dps':>13} {'GNSS jump':>10} {'min sats':>9} {'max h_acc':>10} {'deploy−apogee':>14}")
    for r in rows:
        ratio = r["sustained"] / r["peak_rate"] * 1000 if r["peak_rate"] else float("nan")
        dep = (r["t_dep"] - r["t_apogee"]) if r["t_dep"] is not None else float("nan")
        print(f"{r['label']:<22} {r['median']:>7.1f} {r['sustained']:>10.1f} {r['peak_rate']:>10.0f} "
              f"{ratio:>13.1f} {r['gnss_jump']:>10.1f} {r['min_sats']:>9.0f} "
              f"{r['max_hacc']:>10.1f} {dep:>+14.2f}")
    print("\nA flight ON the tumble line (a consistent m/s per 1000 dps) is attitude-driven.")
    print("A flight OFF it with a large GNSS jump at low sats was dragged by its own fixes.")


if __name__ == "__main__":
    main()
