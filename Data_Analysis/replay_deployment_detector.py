#!/usr/bin/env python3
"""Replay a binary flight log through the REAL recovery-deployment detector.

Drives ``tr::deploymentDetectStep`` (TR_KinematicChecks/DeploymentDetector.h)
with the shipped ``config::DEPLOY_*`` tunables, both compiled straight out of
the firmware tree by ``_deployment_detector_shim.cpp``. Nothing here is a
Python port of the detector: the header exists specifically so the host tests
and this tool exercise the same code the vehicle flies, and a mirror would
reintroduce exactly the silent divergence it was written to avoid.

Reports when the detector latches, which path fired, how the latch lines up
against the firmware's own logged events, and how much margin the thresholds
had — the peak acceleration and largest baro step in each flight phase, which
is what says whether a threshold is comfortable or lucky. ``--sweep`` walks
each tunable to show where the detection window's edges actually are.

Usage:
    python replay_deployment_detector.py <binary> [<binary>...] [--sweep]
                                         [--loop-hz N] [--verbose]
"""
from __future__ import annotations

import argparse
import ctypes
import math
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from plot_flight_data_mini import parse_binary_file  # noqa: E402

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
_DETECTOR_DIR = _REPO / "tinkerrocket-idf" / "components" / "TR_KinematicChecks"
_FC_MAIN_DIR = _REPO / "tinkerrocket-idf" / "projects" / "flight_computer" / "main"
_SHIM_SRC = _HERE / "_deployment_detector_shim.cpp"

G_MS2 = 9.80665
NSF_BURNOUT = 1 << 4
NSF2_DEPLOYED = 1 << 7

# Reason bitmask, mirroring tr::kDeployReason* in DeploymentDetector.h.
REASON_SHOCK_BARO = 1 << 0
REASON_DESCENT_COLLAPSE = 1 << 1


def reason_names(mask: int) -> str:
    parts = []
    if mask & REASON_SHOCK_BARO:
        parts.append("shock+baro")
    if mask & REASON_DESCENT_COLLAPSE:
        parts.append("descent-collapse")
    return " + ".join(parts) if parts else "none"


# --------------------------------------------------------------------------
# Firmware binding
# --------------------------------------------------------------------------

def _build_shim() -> Path:
    """Compile the shim if it is older than any firmware source it pulls in.

    The staleness check is the no-drift guarantee: edit a DEPLOY_* threshold or
    the detector and the next replay rebuilds instead of quietly reporting the
    previous build's answer.
    """
    out = _HERE / "_deployment_detector_shim.so"
    deps = [_SHIM_SRC, _DETECTOR_DIR / "DeploymentDetector.h", _FC_MAIN_DIR / "config.h"]
    deps += sorted((_FC_MAIN_DIR / "board").glob("*.h"))
    if out.exists() and all(out.stat().st_mtime >= d.stat().st_mtime for d in deps):
        return out
    cmd = [
        "c++", "-std=c++17", "-O2", "-shared", "-fPIC",
        f"-I{_DETECTOR_DIR}", f"-I{_FC_MAIN_DIR}",
        str(_SHIM_SRC), "-o", str(out),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        sys.exit(f"failed to build {out.name}:\n{proc.stderr}\ncommand: {' '.join(cmd)}")
    return out


class Detector:
    """ctypes binding to the compiled firmware detector."""

    def __init__(self):
        self._lib = ctypes.CDLL(str(_build_shim()))
        self._lib.tr_deploy_shipped_cfg.argtypes = [
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_uint)]
        self._lib.tr_deploy_run.restype = ctypes.c_int
        self._lib.tr_deploy_run.argtypes = [
            ctypes.c_float, ctypes.c_uint, ctypes.c_float, ctypes.c_uint,
            ctypes.c_float, ctypes.c_float, ctypes.c_uint, ctypes.c_uint,
            ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(ctypes.c_ubyte)]

        f = (ctypes.c_float * 4)()
        u = (ctypes.c_uint * 5)()
        self._lib.tr_deploy_shipped_cfg(f, u)
        self.shipped = {
            "shock_ms2": f[0], "baro_step_m": f[1],
            "ballistic_mps": f[2], "canopy_mps": f[3],
            "shock_count": u[0], "coincidence_ms": u[1],
            "canopy_count": u[2], "launch_lockout_ms": u[3],
        }
        self.loop_hz = u[4]

    def run(self, ticks, **overrides):
        """Run one tick stream. Returns (latched_ms|None, reason, ballistic)."""
        cfg = dict(self.shipped)
        cfg.update(overrides)
        out_t = ctypes.c_uint(0)
        out_reason = ctypes.c_ubyte(0)
        out_ballistic = ctypes.c_ubyte(0)
        latched = self._lib.tr_deploy_run(
            ctypes.c_float(cfg["shock_ms2"]), ctypes.c_uint(int(cfg["shock_count"])),
            ctypes.c_float(cfg["baro_step_m"]), ctypes.c_uint(int(cfg["coincidence_ms"])),
            ctypes.c_float(cfg["ballistic_mps"]), ctypes.c_float(cfg["canopy_mps"]),
            ctypes.c_uint(int(cfg["canopy_count"])),
            ctypes.c_uint(int(cfg["launch_lockout_ms"])),
            ticks.t, ticks.acc, ticks.palt, ticks.new_baro,
            ticks.rate, ticks.burnout, ticks.stop, ticks.n,
            ctypes.byref(out_t), ctypes.byref(out_reason), ctypes.byref(out_ballistic))
        return (out_t.value if latched else None,
                out_reason.value, bool(out_ballistic.value))


# --------------------------------------------------------------------------
# Flight-loop reconstruction
# --------------------------------------------------------------------------

class Ticks:
    """One entry per reconstructed flight-loop iteration, as ctypes arrays."""

    __slots__ = ("t", "acc", "palt", "new_baro", "rate", "burnout", "stop", "n", "py")

    def __init__(self, rows):
        self.n = len(rows)
        self.py = rows
        self.t = (ctypes.c_uint * self.n)(*[r[0] for r in rows])
        self.acc = (ctypes.c_float * self.n)(*[r[1] for r in rows])
        self.palt = (ctypes.c_float * self.n)(*[r[2] for r in rows])
        self.new_baro = (ctypes.c_ubyte * self.n)(*[r[3] for r in rows])
        self.rate = (ctypes.c_float * self.n)(*[r[4] for r in rows])
        self.burnout = (ctypes.c_ubyte * self.n)(*[r[5] for r in rows])
        self.stop = (ctypes.c_ubyte * self.n)(*[r[6] for r in rows])


def pressure_to_altitude_firmware(p_pa: float, p_ground: float) -> float:
    return 44330.0 * (1.0 - (p_pa / p_ground) ** (1.0 / 5.255))


def accel_norm_firmware(low_xyz, high_xyz, low_g_fs_g: float) -> float:
    """Mirror of the flight loop's channel pick, not of the detector.

    The FC switches to the high-g accelerometer as any low-g axis nears
    saturation, which is the only reason an ejection shock reads tens of g
    instead of clipping at full scale.
    """
    sat = (low_g_fs_g - 0.5) * G_MS2
    ax, ay, az = high_xyz if any(abs(v) > sat for v in low_xyz) else low_xyz
    return math.sqrt(ax * ax + ay * ay + az * az)


def estimate_ground_pressure(baro_recs, nonsensor_recs) -> float:
    """Pre-launch mean, matching replay_kinematic_checks.py."""
    if not baro_recs or not nonsensor_recs:
        return 101325.0
    launch_us = next((r["time_us"] for r in nonsensor_recs if r["launch"]), None)
    cutoff = launch_us if launch_us is not None else baro_recs[-1]["time_us"]
    pre = [r["pressure_pa"] for r in baro_recs if r["time_us"] < cutoff]
    return sum(pre) / len(pre) if pre else sum(
        r["pressure_pa"] for r in baro_recs[:50]) / min(50, len(baro_recs))


def build_ticks(records, loop_hz: int):
    """Merge the logged streams back into the flight loop the FC actually ran.

    A tick is emitted per IMU sample but never faster than the loop rate: the
    IMU can log at up to 3840 Hz while loop_fc() runs near 1 kHz and consumes
    only the freshest sample, so stepping per IMU sample would compress every
    tick-count threshold. Logs whose IMU rate is below the loop rate (anything
    predating the deepened handoff queue) simply tick per sample.
    """
    imu, baro, ns = records["ISM6HG256"], records["BMP585"], records["NonSensor"]
    if not imu or not ns:
        return None, {}

    ground_pa = estimate_ground_pressure(baro, ns)
    launch_us = next((r["time_us"] for r in ns if r["launch"]), None)
    if launch_us is None:
        return None, {"no_launch": True}

    events = ([(r["time_us"], 0, r) for r in baro] +
              [(r["time_us"], 1, r) for r in ns] +
              [(r["time_us"], 2, r) for r in imu])
    events.sort(key=lambda e: (e[0], e[1]))

    low_g_fs = records.get("_low_g_fs_g", 16.0)
    min_dt_us = 1_000_000.0 / loop_hz

    palt = 0.0
    new_baro = False
    rate = 0.0
    burnout = False
    landed = False
    last_tick_us = None
    rows = []
    for t_us, kind, r in events:
        if kind == 0:
            palt = pressure_to_altitude_firmware(r["pressure_pa"], ground_pa)
            new_baro = True
            continue
        if kind == 1:
            rate = r["baro_alt_rate"]
            burnout = bool(r["flags"] & NSF_BURNOUT)
            landed = landed or bool(r["alt_landed"])
            continue
        if t_us < launch_us:
            continue
        if last_tick_us is not None and (t_us - last_tick_us) < min_dt_us:
            continue
        last_tick_us = t_us
        acc = accel_norm_firmware(
            (r["low_acc_x"], r["low_acc_y"], r["low_acc_z"]),
            (r["high_acc_x"], r["high_acc_y"], r["high_acc_z"]), low_g_fs)
        rows.append(((t_us - launch_us) // 1000, acc, palt,
                     1 if new_baro else 0, rate, 1 if burnout else 0,
                     1 if landed else 0))
        new_baro = False

    def first(pred):
        return next((( r["time_us"] - launch_us) / 1000.0
                     for r in ns if pred(r)), None)

    logged = {
        "burnout_ms": first(lambda r: r["flags"] & NSF_BURNOUT),
        "apogee_ms": first(lambda r: r["apogee_flag"]),
        "landed_ms": first(lambda r: r["alt_landed"]),
        # The firmware's own latch, present only on logs flown with the
        # detector. None means "never reported", which for a log predating the
        # detector is not the same claim as "the flight never deployed" —
        # has_apogee_flags separates the two.
        "deployed_ms": first(lambda r: r.get("deployed")),
        "has_apogee_flags": bool(ns and ns[0].get("has_apogee_flags")),
        "ground_pa": ground_pa,
        "tick_hz": (len(rows) / ((rows[-1][0] - rows[0][0]) / 1000.0)
                    if len(rows) > 1 and rows[-1][0] > rows[0][0] else 0.0),
    }
    return Ticks(rows), logged


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------

def phase_margins(ticks, logged, latch_ms):
    """Peak |a| and largest baro step per phase.

    This is the part that says whether a threshold is comfortable or merely
    lucky: the detection thresholds have to sit above everything boost and
    coast produce and below what the ejection produces.
    """
    burnout = logged.get("burnout_ms") or 0.0
    landed = logged.get("landed_ms")
    ej = latch_ms if latch_ms is not None else None
    bounds = [("boost  (launch->burnout)", 0.0, burnout)]
    if ej is not None:
        bounds += [("coast  (burnout->latch-100ms)", burnout, ej - 100),
                   ("latch  (+/-300 ms)", ej - 300, ej + 300),
                   ("descent(latch+2s->landing)", ej + 2000,
                    landed if landed else float("inf"))]
    else:
        bounds += [("post-burnout (all)", burnout,
                    landed if landed else float("inf"))]

    out = []
    for name, lo, hi in bounds:
        sub = [r for r in ticks.py if lo <= r[0] < hi]
        if not sub:
            continue
        pk = max(sub, key=lambda r: r[1])
        prev = None
        step = (0.0, 0)
        for r in sub:
            if r[3] and prev is not None and r[2] != prev:
                d = abs(r[2] - prev)
                if d > step[0]:
                    step = (d, r[0])
            if r[3]:
                prev = r[2]
        out.append((name, len(sub), pk[1] / G_MS2, pk[0], step[0], step[1]))
    return out


def sweep(det, ticks):
    print("\n  threshold sweep (one tunable at a time, others shipped):")
    plans = [
        ("shock_ms2", "shock", "g", [2, 4, 6, 8, 12, 20, 30, 40, 60],
         lambda v: v * G_MS2),
        ("baro_step_m", "baro step", "m", [0.5, 1, 2, 3, 5, 20, 80, 120],
         lambda v: v),
        ("shock_count", "shock count", "ticks", [1, 3, 10, 30, 100],
         lambda v: v),
        ("canopy_count", "canopy count", "ticks", [50, 200, 500, 2000],
         lambda v: v),
        ("ballistic_mps", "ballistic", "m/s", [5, 10, 18, 30],
         lambda v: v),
        ("coincidence_ms", "coincidence", "ms", [5, 25, 100, 250, 1000],
         lambda v: v),
    ]
    for key, label, unit, values, conv in plans:
        cells = []
        for v in values:
            t, reason, _ = det.run(ticks, **{key: conv(v)})
            cells.append(f"{v:g}{unit}:" + ("never" if t is None else f"{t}ms"))
        print(f"    {label:12s} " + "  ".join(cells))


def replay(path: str, loop_hz_override: int | None, do_sweep: bool, verbose: bool) -> bool:
    records, _stats, cfg = parse_binary_file(path)
    records["_low_g_fs_g"] = float(cfg.get("low_g_fs_g", 16))

    det = Detector()
    loop_hz = loop_hz_override or det.loop_hz
    ticks, logged = build_ticks(records, loop_hz)

    print(f"\n=== {Path(path).name} ===")
    if ticks is None:
        print("  no launch / no usable streams — skipping")
        return False
    print(f"  IMU={len(records['ISM6HG256']):,}  Baro={len(records['BMP585']):,}  "
          f"NS={len(records['NonSensor']):,}   ground={logged['ground_pa']:.0f} Pa")
    print(f"  reconstructed {ticks.n:,} loop ticks at {logged['tick_hz']:.0f} Hz "
          f"(loop rate cap {loop_hz} Hz)")
    print("  shipped config: " + ", ".join(
        f"{k}={v:g}" for k, v in det.shipped.items()))

    for name, key in (("burnout", "burnout_ms"), ("apogee", "apogee_ms"),
                      ("landed", "landed_ms")):
        v = logged.get(key)
        print(f"    firmware {name:8s} T{'+%.0f ms' % v if v is not None else '  (never)'}")

    latch_ms, reason, ballistic = det.run(ticks)
    if latch_ms is None:
        print("\n  >>> detector NEVER latched")
    else:
        print(f"\n  >>> latched T+{latch_ms} ms   via {reason_names(reason)}")
        apo = logged.get("apogee_ms")
        if apo is not None:
            print(f"      {latch_ms - apo:+.0f} ms relative to the firmware's apogee flag")
    print(f"      descent-collapse path armed: {'yes' if ballistic else 'NO (never ballistic)'}")

    # The firmware's own latch, when the flight was flown with the detector.
    if logged.get("deployed_ms") is not None:
        d = logged["deployed_ms"]
        print(f"      firmware logged NSF2_DEPLOYED at T+{d:.0f} ms "
              f"(replay differs by {latch_ms - d:+.0f} ms)"
              if latch_ms is not None else
              f"      firmware logged NSF2_DEPLOYED at T+{d:.0f} ms but the replay did NOT latch")
    elif not logged.get("has_apogee_flags"):
        print("      firmware NSF2_DEPLOYED: log predates the apogee_flags byte entirely")
    else:
        # The bit's absence alone can't separate these two: the log records
        # whether the detector fired, not whether the build had one.
        print("      firmware NSF2_DEPLOYED: never set "
              "(flight predates the detector, or it genuinely never fired)")

    print("\n  phase margins (thresholds must clear every pre-ejection row):")
    print(f"    {'phase':30s} {'ticks':>7s} {'peak |a|':>10s} {'at':>10s} "
          f"{'max baro step':>14s} {'at':>10s}")
    for name, n, pk_g, pk_t, st_m, st_t in phase_margins(ticks, logged, latch_ms):
        print(f"    {name:30s} {n:7,d} {pk_g:8.1f} g  T+{pk_t:6.0f}ms "
              f"{st_m:12.1f} m  T+{st_t:6.0f}ms")

    if do_sweep:
        sweep(det, ticks)
    if verbose and latch_ms is not None:
        print("\n  ticks around the latch:")
        for r in ticks.py:
            if abs(r[0] - latch_ms) <= 12:
                print(f"    T+{r[0]:6d} ms  |a|={r[1]/G_MS2:6.1f} g  palt={r[2]:8.1f} m"
                      f"  new_baro={r[3]}  rate={r[4]:+6.1f} m/s  burnout={r[5]}")
    return latch_ms is not None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("binaries", nargs="+", help="flight .bin log(s)")
    ap.add_argument("--sweep", action="store_true",
                    help="walk each tunable to show the detection window's edges")
    ap.add_argument("--loop-hz", type=int, default=None,
                    help="override the reconstructed loop rate "
                         "(default: config::FLIGHT_LOOP_UPDATE_RATE)")
    ap.add_argument("--verbose", action="store_true",
                    help="dump the loop ticks around the latch")
    args = ap.parse_args()

    any_latch = False
    for b in args.binaries:
        any_latch |= replay(b, args.loop_hz, args.sweep, args.verbose)
    return 0 if any_latch else 1


if __name__ == "__main__":
    sys.exit(main())
