#!/usr/bin/env python3
"""Run the #491 scenario matrix unattended, defending every run.

The bench RF path is a conducted chain through an SMA that has already worked
loose once, costing ~25 dB with no warning and silently invalidating two full
scenarios. Unattended, that failure mode would produce a night of NO_LOCK
captures indistinguishable from real gate closures unless each run is guarded.

So every scenario is preceded by a preflight on the *static* file, which must
reach a real fix before the scenario is allowed to run. The preflight also
re-searches TX gain, because the working point moves when the connector shifts:
it was 22 early in the session and 47 after the connector loosened.

Results land in results.json as they complete, so an interrupted night still
yields whatever finished.
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
RIG = HERE.parent
sys.path.insert(0, str(RIG))

import serial                                        # noqa: E402
from gnss_nmea_monitor import Parser, _demux         # noqa: E402
from skytraq_binary import (MSG_RCV_STATE, MSG_SV_CH_STATUS,   # noqa: E402
                            parse_rcv_state, parse_sv_ch_status)

# Never hardcode the port: the receiver enumerates as usbmodemNNNN and the
# number changes when it is moved to another USB socket. autodetect_port
# matches on USB vendor instead, which is stable.
from gnss_nmea_monitor import autodetect_port   # noqa: E402
from ensure_hackrf import ensure_hackrf         # noqa: E402
PORT = autodetect_port()
START = "2026/08/19,22:30:00"
STATIC = HERE / "c8" / "t00_long.C8"
CAPS = HERE / "captures"
RESULTS = HERE / "captures" / "results.json"

# Highest first: the path is lossy right now, and a run that fixes at a lower
# gain will also fix at a higher one, but not the reverse.
GAIN_LADDER = [47, 44, 40, 33, 26, 22]

SCENARIOS = [
    ("t1_velramp",    5000.0),
    ("t2_altramp",    5000.0),
    ("t3a_both_18km", 5000.0),
    ("t3b_both_80km", 18300.0),   # seed field is capped at 18300 m
    ("t0_baseline",   1000.0),
]


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def start_tx(c8: Path, gain: int):
    if not ensure_hackrf(quiet=True):
        return None
    errf = open("/tmp/hackrf_tx.err", "w")
    tx = subprocess.Popen(
        ["hackrf_transfer", "-t", str(c8), "-f", "1575420000",
         "-s", "2600000", "-a", "0", "-x", str(gain)],
        stdout=errf, stderr=subprocess.STDOUT)
    time.sleep(4.0)
    out = Path("/tmp/hackrf_tx.err").read_text()
    if tx.poll() is not None or "MB / " not in out:
        try:
            tx.kill()
        except Exception:
            pass
        return None
    return tx


def stop_tx(tx):
    if tx is None:
        return
    import signal as sg
    tx.send_signal(sg.SIGINT)
    try:
        tx.wait(timeout=6)
    except subprocess.TimeoutExpired:
        tx.kill()
    time.sleep(2.0)


def seed(alt_m: float):
    subprocess.run([sys.executable, str(HERE / "seed_restart.py"),
                    "-t", START, "--lat", "40.0", "--lon", "-119.0",
                    "--alt", str(alt_m), "--mode", "warm",
                    "--nav-mode", "airborne"],
                   capture_output=True)


def observe(seconds: float, logfile: Path | None):
    """Watch the receiver. Returns (fix_epochs, total_epochs, peak_sats)."""
    p = Parser()
    fh = open(logfile, "w") if logfile else None
    fix = total = peak = 0
    cur = 0
    try:
        with serial.Serial(PORT, 115200, timeout=0.5) as s:
            buf = bytearray()
            t0 = time.time()
            while time.time() - t0 < seconds:
                chunk = s.read(s.in_waiting or 1)
                if not chunk:
                    continue
                buf.extend(chunk)
                for kind, data in _demux(buf):
                    t = time.time() - t0
                    if fh:
                        fh.write(f"{t:.3f} B {data.hex()}\n" if kind == "bin"
                                 else f"{t:.3f} {data}\n")
                    if kind != "bin":
                        p.feed(data)
                        continue
                    p.feed_binary(data)
                    if data[0] == MSG_SV_CH_STATUS:
                        ss = parse_sv_ch_status(data) or []
                        cur = sum(1 for x in ss if x["cn0"] >= 30)
                        peak = max(peak, cur)
                    if data[0] == MSG_RCV_STATE:
                        total += 1
                        if p.epoch.verdict() == "FIX":
                            fix += 1
    finally:
        if fh:
            fh.close()
    return fix, total, peak


def preflight():
    """Find a TX gain that reaches a real fix on the static file."""
    for gain in GAIN_LADDER:
        log(f"  preflight @ gain {gain} ...")
        tx = start_tx(STATIC, gain)
        if tx is None:
            log("  !! hackrf_transfer would not start; waiting 30 s")
            time.sleep(30)
            continue
        seed(100.0)
        fix, total, peak = observe(110, None)
        stop_tx(tx)
        log(f"     {fix}/{total} FIX epochs, peak {peak} sats")
        # A weak bar here is worse than no bar: it green-lights a 500 s
        # scenario onto a path that will flap in and out of lock and produce a
        # capture whose transitions mean nothing. Demand a solidly held fix.
        if fix >= 55 and peak >= 5:
            return gain
    return None


def correlate(name: str, cap: Path):
    r = subprocess.run(
        [sys.executable, str(HERE / "correlate.py"),
         "-s", str(HERE / "scenarios" / f"{name}.json"),
         "-t", START, str(cap)],
        capture_output=True, text=True)
    return r.stdout + r.stderr


def valid(report: str) -> bool:
    bad = ("never produced a fix", "INCONCLUSIVE", "stale\n           solution",
           "That is a stale")
    return not any(b in report for b in bad)


def main():
    CAPS.mkdir(exist_ok=True)
    results = json.loads(RESULTS.read_text()) if RESULTS.exists() else {}

    for name, alt0 in SCENARIOS:
        c8 = HERE / "c8" / f"{name}.C8"
        if not c8.exists():
            log(f"{name}: no {c8.name}, skipping")
            continue
        dur = c8.stat().st_size / (2 * 2_600_000)

        for attempt in (1, 2, 3):
            log(f"=== {name} attempt {attempt} ({dur:.0f}s scenario) ===")
            gain = preflight()
            if gain is None:
                log("  preflight failed at every gain; RF path is down. "
                    "Waiting 5 min.")
                time.sleep(300)
                continue
            log(f"  preflight OK at gain {gain}; running scenario")

            tx = start_tx(c8, gain)
            if tx is None:
                log("  !! transmitter would not start")
                continue
            seed(alt0)
            cap = CAPS / f"{name}_a{attempt}.log"
            fix, total, peak = observe(dur + 15, cap)
            stop_tx(tx)
            log(f"  captured {total} epochs, {fix} FIX, peak {peak} sats")

            report = correlate(name, cap)
            ok = valid(report)
            results[name] = {"attempt": attempt, "gain": gain, "valid": ok,
                             "fix_epochs": fix, "epochs": total,
                             "peak_sats": peak, "capture": cap.name,
                             "report": report}
            RESULTS.write_text(json.dumps(results, indent=1))
            log(f"  -> {'VALID' if ok else 'invalid'}")
            for line in report.splitlines():
                if "VERDICT" in line or "threshold brackets" in line \
                        or "last fix" in line or "first block" in line:
                    log(f"     {line.strip()}")
            if ok:
                break

    log("=== batch complete ===")
    log(f"results in {RESULTS}")


if __name__ == "__main__":
    main()
