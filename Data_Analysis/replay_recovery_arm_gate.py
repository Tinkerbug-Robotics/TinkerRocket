#!/usr/bin/env python3
"""Replay binary flight logs through the REAL recovery arming interlock (#1176).

Drives ``RecoveryArmGate::step`` together with the shipped
``TR_KinematicChecks``, both compiled straight out of the firmware tree by
``_recovery_arm_gate_shim.cpp``. Nothing here is a Python port: a mirror would
reintroduce exactly the silent divergence the shim pattern exists to avoid.

WHY THE FILTER IS RE-RUN RATHER THAN READ BACK. Two of the gate's inputs --
``quiescent_flag`` and the barometric rate ``d_alt_est_`` -- are computed inside
the kinematics module and never reach the log, so they cannot be read back at
all. And a recovery boot restarts that filter COLD, so replaying against the
log's own warm ``baro_alt_rate`` would be optimistic and would never exercise
the gate's settle window. The shim therefore builds a fresh filter at the
simulated reboot instant and feeds it only samples from there on.

THE QUESTION THIS ANSWERS. Not "does the gate open on one descent" but: for a
reboot at ANY moment of any flight we have flown, does the interlock open, how
long does it take, and which arm carries it? ``--sweep`` walks the reboot
instant across the whole flight at a fixed step and reports the distribution,
including the ticks where it never opens at all.

Usage:
    python replay_recovery_arm_gate.py <binary> [<binary>...]
    python replay_recovery_arm_gate.py --all            # every log under TestFlights
    python replay_recovery_arm_gate.py <binary> --sweep [--step-s 2.0]
    python replay_recovery_arm_gate.py <binary> --pad   # pre-launch ground segment
"""
from __future__ import annotations

import argparse
import ctypes
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from plot_flight_data_mini import parse_binary_file  # noqa: E402
from replay_deployment_detector import (  # noqa: E402
    accel_norm_firmware,
    estimate_ground_pressure,
    pressure_to_altitude_firmware,
)

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
_KIN_DIR = _REPO / "tinkerrocket-idf" / "components" / "TR_KinematicChecks"
_SHIM_DIR = _REPO / "tests_cpp" / "host_shim"
_SHIM_SRC = _HERE / "_recovery_arm_gate_shim.cpp"

ARM_NAMES = {0: "none", 1: "descent", 2: "boost", 3: "gnss", 4: "free-fall", 5: "spin"}

# GNSS validity, mirroring the FC's own predicate at the MainDeployGate step.
GNSS_MIN_SATS = 6
GNSS_STALE_US = 2_000_000


def _build_shim() -> Path:
    """Compile the shim if it is older than any firmware source it pulls in.

    The staleness check is the no-drift guarantee: edit a threshold in
    RecoveryArmGate.h and the next replay rebuilds instead of quietly
    reporting the previous build's answer.
    """
    out = _HERE / "_recovery_arm_gate_shim.so"
    deps = [_SHIM_SRC,
            _KIN_DIR / "RecoveryArmGate.h",
            _KIN_DIR / "TR_KinematicChecks.h",
            _KIN_DIR / "TR_KinematicChecks.cpp"]
    if out.exists() and all(out.stat().st_mtime >= d.stat().st_mtime for d in deps):
        return out
    cmd = ["c++", "-O2", "-std=c++17", "-shared", "-fPIC",
           f"-I{_KIN_DIR}", f"-I{_SHIM_DIR}",
           str(_SHIM_SRC), str(_KIN_DIR / "TR_KinematicChecks.cpp"),
           "-o", str(out)]
    subprocess.run(cmd, check=True)
    return out


class Gate:
    def __init__(self) -> None:
        self._lib = ctypes.CDLL(str(_build_shim()))
        self._lib.tr_rag_shipped_cfg.argtypes = [
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_uint)]
        self._lib.tr_rag_replay.restype = ctypes.c_int
        self._lib.tr_rag_replay.argtypes = [
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint),                                  # t_ms
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_ubyte), # palt, baro_new
            ctypes.POINTER(ctypes.c_ubyte),                                 # baro_healthy
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), # accel, roll
            ctypes.POINTER(ctypes.c_ubyte),                                 # imu_fresh
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), # gnss alt, vel
            ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(ctypes.c_ubyte), # gnss new, ok
            ctypes.c_uint, ctypes.c_uint, ctypes.c_uint,                     # reboot_at, elapsed, gnss_cold
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_uint),  # cfg
            ctypes.POINTER(ctypes.c_long), ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(ctypes.c_long), ctypes.POINTER(ctypes.c_long),
            ctypes.POINTER(ctypes.c_ubyte)]
        f = (ctypes.c_float * 5)()
        u = (ctypes.c_uint * 9)()
        self._lib.tr_rag_shipped_cfg(f, u)
        self.shipped = {
            "descent_mps": f[0], "boost_ms2": f[1], "gnss_mps": f[2],
            "freefall_ms2": f[3], "spin_dps": f[4],
            "descent_hold_ms": u[0], "boost_hold_ms": u[1], "gnss_hold_ms": u[2],
            "freefall_hold_ms": u[3], "spin_hold_ms": u[4],
            "baro_settle_ms": u[5], "refute_hold_ms": u[6],
            "max_flight_ms": u[7], "apogee_arm_ms": u[8],
        }

    def replay(self, ticks, reboot_at_ms: int, elapsed_at_reboot_ms: int,
               gnss_cold_ms: int = 30000):
        n = ticks.n
        open_ms = ctypes.c_long(-1)
        arm = ctypes.c_ubyte(0)
        apogee_ms = ctypes.c_long(-1)
        refute_ms = ctypes.c_long(-1)
        stepped = self._lib.tr_rag_replay(
            n, ticks.t, ticks.palt, ticks.baro_new, ticks.baro_ok,
            ticks.accel, ticks.roll, ticks.imu_fresh,
            ticks.gnss_alt, ticks.gnss_vel, ticks.gnss_new, ticks.gnss_ok,
            reboot_at_ms, elapsed_at_reboot_ms, gnss_cold_ms, None, None,
            ctypes.byref(open_ms), ctypes.byref(arm),
            ctypes.byref(apogee_ms), ctypes.byref(refute_ms), None)
        return {
            "stepped": stepped,
            "open_ms": None if open_ms.value < 0 else open_ms.value,
            "arm": ARM_NAMES.get(arm.value, "?"),
            "apogee_armed_ms": None if apogee_ms.value < 0 else apogee_ms.value,
            "refute_ms": None if refute_ms.value < 0 else refute_ms.value,
        }


class Ticks:
    """Merged tick stream, launch-relative, INCLUDING the pre-launch pad phase.

    replay_deployment_detector.build_ticks drops everything before launch
    because the deployment detector cannot fire there. Here the pad phase is
    the point: it is a real recording of a rocket sitting still, and it is what
    says whether the interlock can be opened by something that is not a flight.
    """

    def __init__(self, rows, launch_ms):
        self.n = len(rows)
        self.launch_ms = launch_ms
        arr_u = ctypes.c_uint * self.n
        arr_f = ctypes.c_float * self.n
        arr_b = ctypes.c_ubyte * self.n
        self.t = arr_u(*[r[0] for r in rows])
        self.palt = arr_f(*[r[1] for r in rows])
        self.baro_new = arr_b(*[r[2] for r in rows])
        self.baro_ok = arr_b(*[r[3] for r in rows])
        self.accel = arr_f(*[r[4] for r in rows])
        self.roll = arr_f(*[r[5] for r in rows])
        self.imu_fresh = arr_b(*[r[6] for r in rows])
        self.gnss_alt = arr_f(*[r[7] for r in rows])
        self.gnss_vel = arr_f(*[r[8] for r in rows])
        self.gnss_new = arr_b(*[r[9] for r in rows])
        self.gnss_ok = arr_b(*[r[10] for r in rows])
        self.t_first = rows[0][0]
        self.t_last = rows[-1][0]


def build_ticks(records, loop_hz: int = 1000):
    """Merge the logged streams back into the flight loop the FC actually ran.

    A tick per IMU sample but never faster than the loop rate, matching the
    deployment replay: the IMU logs at up to 3840 Hz while loop_fc runs near
    1 kHz and consumes only the freshest sample, so stepping per IMU sample
    would compress every hold threshold in the gate.
    """
    imu = records["ISM6HG256"]
    baro = records["BMP585"]
    ns = records["NonSensor"]
    gnss = records["GNSS"]
    if not imu or not ns:
        return None, {"reason": "no IMU or NonSensor stream"}

    ground_pa = estimate_ground_pressure(baro, ns)
    launch_us = next((r["time_us"] for r in ns if r["launch"]), None)
    if launch_us is None:
        return None, {"reason": "no launch in this log"}

    low_g_fs = records.get("_low_g_fs_g", 16.0)
    min_dt_us = 1_000_000.0 / loop_hz

    events = ([(r["time_us"], 0, r) for r in baro] +
              [(r["time_us"], 1, r) for r in gnss] +
              [(r["time_us"], 2, r) for r in imu])
    events.sort(key=lambda e: (e[0], e[1]))

    palt = 0.0
    new_baro = False
    baro_us = None
    g_alt, g_vel, g_ok, g_us = 0.0, 0.0, 0, None
    new_gnss = False
    last_tick_us = None
    rows = []
    for t_us, kind, r in events:
        if kind == 0:
            palt = pressure_to_altitude_firmware(r["pressure_pa"], ground_pa)
            new_baro = True
            baro_us = t_us
            continue
        if kind == 1:
            g_alt = float(r.get("alt", 0.0) or 0.0)
            g_vel = float(r.get("vel_u", 0.0) or 0.0)
            g_ok = 1 if (int(r.get("fix_mode", 0) or 0) >= 3 and
                         int(r.get("num_sats", 0) or 0) >= GNSS_MIN_SATS) else 0
            g_us = t_us
            new_gnss = True
            continue
        if last_tick_us is not None and (t_us - last_tick_us) < min_dt_us:
            continue
        last_tick_us = t_us
        acc = accel_norm_firmware(
            (r["low_acc_x"], r["low_acc_y"], r["low_acc_z"]),
            (r["high_acc_x"], r["high_acc_y"], r["high_acc_z"]), low_g_fs)
        # #257 baro health, as the FC computes it: a sample recently enough to
        # be believed. Range is implicit in the log (a stored sample was in
        # range when it was taken).
        baro_ok = 1 if (baro_us is not None and (t_us - baro_us) < 200_000) else 0
        gnss_ok = 1 if (g_ok and g_us is not None and
                        (t_us - g_us) < GNSS_STALE_US) else 0
        rows.append(((t_us - launch_us) // 1000 + 600_000,  # bias so pad time is >= 0
                     palt, 1 if new_baro else 0, baro_ok,
                     acc, r["gyro_x"], 1,
                     g_alt, g_vel, 1 if new_gnss else 0, gnss_ok))
        new_baro = False
        new_gnss = False

    if len(rows) < 10:
        return None, {"reason": "too few ticks"}

    def first(pred):
        return next(((r["time_us"] - launch_us) / 1000.0 for r in ns if pred(r)), None)

    meta = {
        "launch_ms": 600_000,
        "apogee_ms": first(lambda r: r["apogee_flag"]),
        "landed_ms": first(lambda r: r["alt_landed"]),
        "ground_pa": ground_pa,
        "n_ticks": len(rows),
        "pad_ms": rows[0][0],
        "end_ms": rows[-1][0],
    }
    return Ticks(rows, 600_000), meta


def _fmt(ms, launch_ms):
    return "never" if ms is None else f"T{(ms - launch_ms) / 1000.0:+.2f}s"


def _kill_baro(ticks):
    """Model a barometer that does not survive the reboot (or a taped port).

    Sets baro_healthy false for every tick, which removes the descent arm and
    the barometric half of the quiescence test. This is the degraded case the
    owner's no-sensorless-backstop ruling bears on, so it is worth measuring
    rather than reasoning about.
    """
    for i in range(ticks.n):
        ticks.baro_ok[i] = 0
    return ticks


def replay_one(path: Path, gate: Gate, do_sweep: bool, step_s: float,
               gnss_cold_ms: int = 30000, no_baro: bool = False) -> dict:
    try:
        records, _stats, cfg = parse_binary_file(str(path))
        records["_low_g_fs_g"] = float(cfg.get("low_g_fs_g", 16))
    except Exception as exc:  # noqa: BLE001
        return {"path": path, "error": f"parse failed: {exc}"}
    ticks, meta = build_ticks(records)
    if ticks is None:
        return {"path": path, "error": meta.get("reason", "unusable")}
    if no_baro:
        ticks = _kill_baro(ticks)

    launch = meta["launch_ms"]
    out = {"path": path, "meta": meta, "results": []}

    # 1. The ground case: reboot during the pad phase. Must never open.
    if meta["pad_ms"] < launch - 5000:
        r = gate.replay(ticks, meta["pad_ms"], 0, gnss_cold_ms)
        r["label"] = "pad (pre-launch)"
        r["reboot_ms"] = meta["pad_ms"]
        out["results"].append(r)

    # 2. The incident case: reboot just after launch.
    r = gate.replay(ticks, launch + 500, 500, gnss_cold_ms)
    r["label"] = "T+0.5s (the 2026-08-29 case)"
    r["reboot_ms"] = launch + 500
    out["results"].append(r)

    # 3. Mid-descent, where the main charge still has to fire.
    apo = meta.get("apogee_ms")
    if apo is not None:
        reboot = launch + int(apo) + 3000
        if reboot < meta["end_ms"] - 5000:
            r = gate.replay(ticks, reboot, int(apo) + 3000, gnss_cold_ms)
            r["label"] = "apogee + 3s (under drogue)"
            r["reboot_ms"] = reboot
            out["results"].append(r)

    if do_sweep:
        sweep = []
        t = ticks.t_first
        step_ms = int(step_s * 1000)
        while t < ticks.t_last - 3000:
            r = gate.replay(ticks, t, max(0, t - launch), gnss_cold_ms)
            sweep.append((t, r["open_ms"], r["arm"]))
            t += step_ms
        out["sweep"] = sweep
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="*")
    ap.add_argument("--all", action="store_true",
                    help="every flight_*.bin under the TestFlights tree")
    ap.add_argument("--sweep", action="store_true",
                    help="walk the reboot instant across the whole flight")
    ap.add_argument("--step-s", type=float, default=2.0)
    ap.add_argument("--no-baro", action="store_true",
                    help="model a barometer that never comes back after the "
                         "reboot (dead part, or a blocked static port)")
    ap.add_argument("--gnss-cold-s", type=float, default=30.0,
                    help="GNSS acquisition time after a reboot (0 = fix "
                         "available immediately, which is NOT realistic)")
    args = ap.parse_args()

    paths = [Path(p) for p in args.paths]
    if args.all:
        root = Path(os.path.expanduser(
            "~/Documents/Hobbies/ModelRockets/TestFlights"))
        paths = sorted(p for p in root.rglob("flight_*.bin")
                       if p.stat().st_size > 512 * 1024)

    if not paths:
        ap.error("give at least one log, or --all")

    gate = Gate()
    print(f"GNSS cold-start window after a reboot: {args.gnss_cold_s:.0f}s")
    print("Shipped thresholds:")
    for k, v in gate.shipped.items():
        print(f"    {k:18s} {v}")
    print()

    ok = skipped = 0
    never_open_pad = 0
    pad_total = 0
    for p in paths:
        res = replay_one(p, gate, args.sweep, args.step_s,
                         int(args.gnss_cold_s * 1000), args.no_baro)
        if "error" in res:
            print(f"-- {p.name}: SKIPPED ({res['error']})")
            skipped += 1
            continue
        ok += 1
        m = res["meta"]
        launch = m["launch_ms"]
        print(f"== {p.parent.name}/{p.name}")
        print(f"   {m['n_ticks']} ticks, pad {(launch - m['pad_ms'])/1000.0:.0f}s, "
              f"apogee {m['apogee_ms']/1000.0 if m['apogee_ms'] else float('nan'):.1f}s, "
              f"end T+{(m['end_ms']-launch)/1000.0:.0f}s")
        for r in res["results"]:
            if r["label"].startswith("pad"):
                pad_total += 1
                if r["open_ms"] is None:
                    never_open_pad += 1
            opened = (f"OPEN {_fmt(r['open_ms'], r['reboot_ms']).replace('T','+')} "
                      f"via {r['arm']}" if r["open_ms"] is not None else "never opened")
            apo = ("" if r["apogee_armed_ms"] is None else
                   f", apogee armed +{(r['apogee_armed_ms']-r['reboot_ms'])/1000.0:.2f}s")
            ref = ("" if r["refute_ms"] is None else
                   f", REFUTED +{(r['refute_ms']-r['reboot_ms'])/1000.0:.1f}s")
            print(f"     {r['label']:28s} -> {opened}{apo}{ref}")
        if "sweep" in res:
            n = len(res["sweep"])
            never = [t for t, o, _ in res["sweep"] if o is None]
            lat = [(o - t) / 1000.0 for t, o, _ in res["sweep"] if o is not None]
            arms = {}
            for _, o, a in res["sweep"]:
                if o is not None:
                    arms[a] = arms.get(a, 0) + 1
            if lat:
                lat.sort()
                print(f"     sweep {n} reboots: opened {len(lat)}, never {len(never)}; "
                      f"latency min {lat[0]:.2f}s med {lat[len(lat)//2]:.2f}s "
                      f"max {lat[-1]:.2f}s; arms {arms}")
            else:
                print(f"     sweep {n} reboots: NEVER opened")
        print()

    print(f"replayed {ok}, skipped {skipped}")
    if pad_total:
        print(f"pad-phase reboots that never opened the gate: "
              f"{never_open_pad}/{pad_total}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
