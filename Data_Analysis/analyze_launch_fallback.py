#!/usr/bin/env python3
"""Replay the launch latch over logged boosts: primary vs accel-only fallback (#1102).

The FC latches launch either on accel + a baro-confirmed climb (primary) or on
LAUNCH_ACCEL_FALLBACK_COUNT uninterrupted flight-loop samples above
LAUNCH_ACCEL_FALLBACK_MS2 (the #258 fallback, ungated from baro health in
#1102).  This script answers, per logged flight:

  * when the FC's own primary latched (the NonSensor launch flag), relative to
    the first sustained >20 m/s2 sample;
  * when the fallback WOULD have latched, replaying its counters over the
    logged |a| at the flight loop's ~1 kHz cadence with the FC's own
    accelerometer channel pick -- i.e. what a blocked static port costs;
  * how long the longest uninterrupted >3 g run inside the boost is (the
    margin the 250-sample window has against vibration dropouts);
  * optionally (--gyro-reject) what a pitch/yaw-rate reset on the fallback
    would do, the option-C variant that was evaluated and not adopted.

--export-fixture writes one flight's |a| trace as a C header for the host
tests, so the 2026-05-17 54 mm boost (the noisiest logged) pins the fallback
behaviour on real data.

Usage:
    python analyze_launch_fallback.py                 # every log under TestFlights
    python analyze_launch_fallback.py <flight.bin>... # specific logs
    python analyze_launch_fallback.py --gyro-reject
    python analyze_launch_fallback.py <flight.bin> --export-fixture OUT.h [--pre-ms 50] [--post-ms 700]
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from plot_flight_data_mini import NSF_BURNOUT, parse_binary_file  # noqa: E402
from replay_deployment_detector import accel_norm_firmware  # noqa: E402

DEFAULT_ROOT = Path.home() / "Documents" / "Hobbies" / "ModelRockets" / "TestFlights"

# Mirror TR_KinematicChecks.cpp.  If these move, so must this.
LAUNCH_RESET_MS2 = 20.0
LAUNCH_ACCEL_FALLBACK_MS2 = 30.0
LAUNCH_ACCEL_FALLBACK_COUNT = 250
LOOP_TICK_US = 1000

def load(path):
    rec, _stats, cfg = parse_binary_file(str(path))
    imu, ns = rec["ISM6HG256"], rec["NonSensor"]
    if not imu or not ns:
        return None
    t = np.array([r["time_us"] for r in imu], dtype=np.int64)
    low = np.array([[r["low_acc_x"], r["low_acc_y"], r["low_acc_z"]] for r in imu])
    high = np.array([[r["high_acc_x"], r["high_acc_y"], r["high_acc_z"]] for r in imu])
    gyr = np.array([[r["gyro_x"], r["gyro_y"], r["gyro_z"]] for r in imu])
    acc = np.array([accel_norm_firmware(l, h, cfg["low_g_fs_g"]) for l, h in zip(low, high)])
    # One tick per IMU sample, never faster than the loop (same rule as
    # replay_deployment_detector.build_ticks).
    keep = [0]
    last = t[0]
    for i in range(1, len(t)):
        if t[i] - last >= LOOP_TICK_US:
            keep.append(i)
            last = t[i]
    keep = np.array(keep)
    hz = 1e6 / np.median(np.diff(t))
    launch = [r["time_us"] for r in ns if r["launch"]]
    burnout = [r["time_us"] for r in ns if r["flags"] & NSF_BURNOUT]
    return dict(
        t_s=(t[keep] - t[0]) / 1e6, acc=acc[keep], gyr=gyr[keep], low=low[keep], hz=hz,
        t_launch=(launch[0] - t[0]) / 1e6 if launch else None,
        t_burnout=(burnout[0] - t[0]) / 1e6 if burnout else None)

def boost_start(acc):
    """Index of the first sample of the first 50-sample run above the 20 m/s2 floor."""
    run = 0
    for i, a in enumerate(acc > LAUNCH_RESET_MS2):
        run = run + 1 if a else 0
        if run == 50:
            return i - 49
    return None

def replay_fallback(acc, extra_ok=None):
    """Index at which launch_count_hi exceeds the bar, replaying the firmware counters."""
    hi = 0
    for i, a in enumerate(acc):
        ok = a > LAUNCH_RESET_MS2 and a > LAUNCH_ACCEL_FALLBACK_MS2
        if ok and extra_ok is not None:
            ok = bool(extra_ok[i])
        hi = hi + 1 if ok else 0
        if hi > LAUNCH_ACCEL_FALLBACK_COUNT:
            return i
    return None

def longest_run_s(mask, t_s):
    best, start = 0.0, None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if start is not None and (not m or i == len(mask) - 1):
            end = i if m else i - 1
            best = max(best, t_s[end] - t_s[start])
            start = None
    return best

def analyse(path, gyro_reject=()):
    d = load(path)
    if d is None:
        return None
    b0 = boost_start(d["acc"])
    if b0 is None:
        return None
    t, acc = d["t_s"], d["acc"]
    tb0 = t[b0]
    t_end = d["t_burnout"] if d["t_burnout"] else tb0 + 3.0
    burn = (t >= tb0) & (t < t_end)
    win = (t >= tb0) & (t < tb0 + 0.25)
    fb = replay_fallback(acc)
    row = dict(
        hz=d["hz"], preroll_s=tb0,
        primary_ms=(d["t_launch"] - tb0) * 1e3 if d["t_launch"] is not None else None,
        fallback_ms=(t[fb] - tb0) * 1e3 if fb is not None else None,
        longest_hi_ms=longest_run_s(list(acc[burn] > LAUNCH_ACCEL_FALLBACK_MS2), t[burn]) * 1e3,
        win_mean=float(acc[win].mean()), win_min=float(acc[win].min()),
        burn_s=(t_end - tb0) if d["t_burnout"] else None, truncated=(b0 == 0))
    if gyro_reject:
        thrust_ax = int(np.argmax(np.abs(d["low"][win].mean(axis=0))))
        lat_axes = [a for a in range(3) if a != thrust_ax]
        lat = np.hypot(d["gyr"][:, lat_axes[0]], d["gyr"][:, lat_axes[1]])
        row["lat250_dps"] = float(lat[win].max())
        row["roll250_dps"] = float(np.abs(d["gyr"][win][:, thrust_ax]).max())
        for thr in gyro_reject:
            i = replay_fallback(acc, extra_ok=lat < thr)
            row[f"fb<{thr}"] = (t[i] - tb0) * 1e3 if i is not None and t[i] < t_end + 0.5 else None
    return row

def fmt(v, w=7, d=0):
    return f"{v:{w}.{d}f}" if v is not None else f"{'-':>{w}}"

def export_fixture(path, out, pre_ms, post_ms):
    d = load(path)
    b0 = boost_start(d["acc"])
    t, acc = d["t_s"], d["acc"]
    tb0 = t[b0]
    sel = (t >= tb0 - pre_ms / 1e3) & (t < tb0 + post_ms / 1e3)
    vals = acc[sel]
    boost_idx = int(np.searchsorted(t[sel], tb0))
    fb = replay_fallback(vals)
    name = "kBoostAccel" + out.stem.split("boost_accel_")[-1].replace("_", " ").title().replace(" ", "")
    lines = [
        "// GENERATED by Data_Analysis/analyze_launch_fallback.py --export-fixture; do not hand-edit.",
        f"// Source log: {path.name}",
        f"//   {Path(path).parent.name}, IMU {d['hz']:.0f} Hz, burnout at +{(d['t_burnout'] - tb0) * 1e3:.0f} ms"
        if d["t_burnout"] else f"//   {Path(path).parent.name}, IMU {d['hz']:.0f} Hz",
        "// |a| in m/s2 per flight-loop tick (one IMU sample, never faster than 1 kHz),",
        "// with the FC's own low-g/high-g channel pick, from %d ms before the first" % pre_ms,
        "// sustained >20 m/s2 sample through %d ms after it." % post_ms,
        f"// Replayed fallback latch: +{(t[sel][fb] - tb0) * 1e3:.0f} ms after ignition." if fb is not None
        else "// Replayed fallback latch: never.",
        "#pragma once",
        f"static const int   {name}_n = {len(vals)};",
        f"static const int   {name}_boost0 = {boost_idx};   // index of ignition (first sustained >20 m/s2 sample)",
        f"static const float {name}[{len(vals)}] = {{",
    ]
    for i in range(0, len(vals), 10):
        lines.append("    " + ", ".join(f"{v:.2f}f" for v in vals[i:i + 10]) + ",")
    lines.append("};")
    out.write_text("\n".join(lines) + "\n")
    print(f"wrote {out} ({len(vals)} samples, ignition at index {boost_idx}, "
          f"fallback at +{(t[sel][fb] - tb0) * 1e3:.0f} ms)" if fb is not None else f"wrote {out}")

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bins", nargs="*", help="flight_*.bin files; default: every log under TestFlights")
    ap.add_argument("--root", default=str(DEFAULT_ROOT))
    ap.add_argument("--gyro-reject", nargs="*", type=float, metavar="DPS",
                    help="also replay the fallback with a pitch/yaw-rate reset at these thresholds (default 150 200 250 300)")
    ap.add_argument("--export-fixture", type=Path, metavar="OUT.h")
    ap.add_argument("--pre-ms", type=int, default=50)
    ap.add_argument("--post-ms", type=int, default=700)
    args = ap.parse_args()

    if args.export_fixture:
        if len(args.bins) != 1:
            ap.error("--export-fixture takes exactly one flight")
        export_fixture(Path(args.bins[0]), args.export_fixture, args.pre_ms, args.post_ms)
        return

    paths = [Path(b) for b in args.bins] or sorted(
        Path(p) for p in glob.glob(os.path.join(args.root, "**", "flight_*.bin"), recursive=True)
        if os.path.getsize(p) > 1024)
    thr = tuple(args.gyro_reject) if args.gyro_reject is not None else ()
    if args.gyro_reject is not None and not thr:
        thr = (150.0, 200.0, 250.0, 300.0)

    hdr = f"{'flight':36} {'IMU Hz':>6} {'pre s':>5} {'primary':>7} {'fallbk':>7} {'run>3g':>7} {'a.mean':>6} {'a.min':>6} {'burn s':>6}"
    if thr:
        hdr += f" {'lat250':>6} {'roll250':>7} " + " ".join(f"{'fb<'+str(int(x)):>7}" for x in thr)
    print(hdr)
    rows = []
    for p in paths:
        try:
            r = analyse(p, thr)
        except Exception as e:  # a corrupt or pre-format log should not end the sweep
            print(f"{p.name:36} parse error: {e}")
            continue
        if r is None:
            print(f"{p.name:36} no boost in log")
            continue
        rows.append(r)
        label = (p.parent.parent.name[:10] + " " + p.parent.name[:25]) if p.parent.parent.name else p.name
        line = (f"{label:36} {r['hz']:6.0f} {r['preroll_s']:5.2f} {fmt(r['primary_ms'])} {fmt(r['fallback_ms'])}"
                f" {fmt(r['longest_hi_ms'])} {r['win_mean']:6.1f} {r['win_min']:6.1f} {fmt(r['burn_s'], 6, 2)}")
        if thr:
            line += f" {r['lat250_dps']:6.0f} {r['roll250_dps']:7.0f} " + " ".join(fmt(r[f'fb<{x}']) for x in thr)
        if r["truncated"]:
            line += "  (log starts inside the boost)"
        print(line)
    if not rows:
        return
    prim = [r["primary_ms"] for r in rows if r["primary_ms"] is not None]
    fb = [r["fallback_ms"] for r in rows if r["fallback_ms"] is not None]
    print(f"\n{len(rows)} boosts.  Primary latched: median {np.median(prim):.0f} ms, max {max(prim):.0f} ms after ignition.")
    print(f"Fallback would latch on {len(fb)}/{len(rows)}: median {np.median(fb):.0f} ms, max {max(fb):.0f} ms.")
    if thr:
        for x in thr:
            never = sum(1 for r in rows if r[f"fb<{x}"] is None)
            delayed = sum(1 for r in rows if r[f"fb<{x}"] is not None and r["fallback_ms"] is not None
                          and r[f"fb<{x}"] > r["fallback_ms"] + 1)
            print(f"  lateral-rate reset at {x:.0f} dps: never latches on {never}, delayed on {delayed}")

if __name__ == "__main__":
    main()
