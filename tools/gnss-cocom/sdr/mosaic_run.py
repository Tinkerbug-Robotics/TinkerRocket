#!/usr/bin/env python3
"""Fly one scenario into the mosaic-G5 and log its SBF: run_radiated.py for a Septentrio.

Per flight: optional cold start (erst, Soft, PVTData+SatData: ephemeris, almanac and
last position erased, configuration kept), the test setup from mosaic_log.py (20 Hz
MeasEpoch/MeasExtra/PVTGeodetic, srd High, Unlimited) plus Galileo OSNMA off, so the
simulator's Galileo is not dropped as unauthenticated (1.0.0 already defaults to off and
has no signal-authentication checks to relax), then the transmitter, then SBF for --seconds.

The transmitter is the sweep's buffered hackrf_tx_ram (HACKRF_TX_RAM in the environment,
or on the PATH; TX_NOISE_DB and TX_CARRIER_HZ pass -N and -C, as px_run_ram.py does), or
hackrf_transfer when neither is there. Its per-second log (-B: underruns) is kept next to
the capture as .hackrf.txt, and the radio is left idle afterwards. The IQ files come from
--c8-dir (C8_DIR in the environment, default c8/ beside this script).

    mosaic_run.py -s signalsim_hotshot_all_2026_57_w_p180 -x 3 --seconds 305 \\
        --rate 18480000 --freq 1568286000 --bb-filter 20000000 --tag mosaic_g5_wide_mhs12
"""

from __future__ import annotations

import argparse
import os
import shutil
import statistics
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

import serial                                           # noqa: E402
import septentrio_sbf as sbf                            # noqa: E402
import mosaic_log                                       # noqa: E402
from ensure_hackrf import ensure_hackrf, hackrf_idle    # noqa: E402

C8_DIR = os.environ.get("C8_DIR", str(HERE / "c8"))
TX_BIN = os.environ.get("HACKRF_TX_RAM") or shutil.which("hackrf_tx_ram") or "hackrf_transfer"
RIG_CMDS = ["sou, off, , off"]       # fw 1.0.0 has no setSignalAuthentication (sna); OSNMA is off by default there


def start_tx(binary: str, c8: Path, freq: int, rate: int, gain: int, errfile: str, extra=()):
    """Start the transmitter and confirm it streams (same checks as px_run_ram.start_tx_ram)."""
    if not ensure_hackrf():
        return None
    errf = open(errfile, "w")
    noise = os.environ.get("TX_NOISE_DB", "")
    carrier = os.environ.get("TX_CARRIER_HZ", "")
    ram = Path(binary).name == "hackrf_tx_ram"
    cmd = [binary, "-t", str(c8), "-f", str(freq), "-s", str(rate), "-a", "0", "-x", str(gain), *extra]
    if ram:
        cmd += [*(["-N", noise] if noise else []), *(["-C", carrier] if carrier else [])]
    elif noise or carrier:
        print("!! TX_NOISE_DB / TX_CARRIER_HZ need hackrf_tx_ram")
        return None
    tx = subprocess.Popen(cmd, stdout=errf, stderr=subprocess.STDOUT)
    time.sleep(5.0)
    out = Path(errfile).read_text()
    if ("hackrf_open() failed" in out or "not found" in out.lower() or tx.poll() is not None
            or "MB / " not in out):
        try:
            tx.kill()
        except Exception:
            pass
        print(f"!! {Path(binary).name} is not streaming:\n{out[:400] or '(no output)'}")
        return None
    for key, want in (("added noise", noise), ("carrier shift -C", carrier)):
        lines = [ln for ln in out.splitlines() if key in ln]
        if want and not lines:
            tx.kill()
            print(f"!! asked for {key} {want} but the transmitter does not report it")
            return None
        if lines:
            print(f"# {lines[0]}")
    print(f"# TX confirmed ({Path(binary).name}): {out.strip().splitlines()[-1]}")
    return tx


def stop_tx(tx):
    if tx is None:
        return
    tx.send_signal(2)
    try:
        tx.wait(timeout=6)
    except subprocess.TimeoutExpired:
        tx.kill()


def connect(log, port_arg, wait_s=60.0):
    """Open the receiver; returns (link, port name in its prompt) or (None, None)."""
    t_end = time.time() + wait_s
    while time.time() < t_end:
        port = port_arg or mosaic_log.find_port()
        if port:
            try:
                link = mosaic_log.Link(port, log)
                link.command("")
                if link.prompt:
                    log.note(f"opened {port}")
                    return link, link.prompt.rstrip(">").strip()
                link.s.close()
            except (serial.SerialException, OSError):
                pass
        time.sleep(1)
    return None, None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-s", "--scenario", required=True, help="C8 stem, e.g. signalsim_hotshot_all_2026_57_w_p180")
    ap.add_argument("--c8-dir", default=C8_DIR)
    ap.add_argument("-p", "--port", help="default: the first Septentrio port")
    ap.add_argument("-x", "--gain", type=int, default=3)
    ap.add_argument("--freq", type=int, default=1568286000)
    ap.add_argument("--rate", type=int, default=18480000)
    ap.add_argument("--bb-filter", type=int, help="hackrf -b baseband filter, Hz")
    ap.add_argument("--seconds", type=float, required=True)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--tx-bin", default=TX_BIN, help="hackrf_tx_ram, or hackrf_transfer")
    ap.add_argument("--cold-start", action="store_true")
    ap.add_argument("--dynamics", default="High, Unlimited")
    ap.add_argument("--interval", default="msec50")
    ap.add_argument("--cmd", action="append", default=[], help="extra command after the rig setup")
    ap.add_argument("--keep-streams", action="store_true")
    ap.add_argument("--listen-only", action="store_true",
                    help="no transmitter (real sky): the cold-start timing check; -s only names the capture")
    args = ap.parse_args()
    args.cmd = ([] if args.listen_only else RIG_CMDS) + args.cmd

    c8 = Path(args.c8_dir) / f"{args.scenario}.C8"
    if not args.listen_only and not c8.exists():
        print(f"!! no {c8}")
        return 1
    cap = HERE / "captures" / f"{args.tag}_{args.scenario}.log"
    if cap.exists():
        print(f"!! {cap.name} exists")
        return 1
    cap.parent.mkdir(exist_ok=True)
    t0 = time.time()
    log = mosaic_log.Log(cap, t0)
    log.note((f"listen-only {args.scenario}" if args.listen_only else f"tx {c8.name} gain {args.gain}")
             + f"; mosaic_run.py {' '.join(sys.argv[1:])}")

    link, cd = connect(log, args.port)
    if link is None:
        print("!! no mosaic-G5 answering on USB")
        return 1
    if args.cold_start:
        t_reset = time.time() - t0
        log.note("cold start: erst, Soft, PVTData+SatData")
        link.s.write(b"erst, Soft, PVTData+SatData\r")
        time.sleep(1.0)
        try:
            link.pump()
            link.s.close()
        except (serial.SerialException, OSError):
            pass
        time.sleep(8.0)                              # the firmware restarts and USB re-enumerates
        link, cd = connect(log, args.port, wait_s=60.0)
        if link is None:
            print("!! the receiver did not come back after the cold start")
            return 1
        print(f"# cold start done ({cd})")
    if not mosaic_log.configure(link, cd, args):
        print("!! the mosaic did not take the configuration")
        return 1
    link.command("lif, Identification")
    log.note("configured")

    errfile = f"/tmp/hackrf_tx_{args.scenario}.err"
    extra = ["-B"] + (["-b", str(args.bb_filter)] if args.bb_filter else [])
    tx = None
    if not args.listen_only:
        print(f"# {c8.name}: {c8.stat().st_size / 1e9:.2f} GB, {c8.stat().st_size / (2 * args.rate):.0f} s, "
              f"gain {args.gain}")
        tx = start_tx(args.tx_bin, c8, args.freq, args.rate, args.gain, errfile, extra)
        if tx is None:
            return 1
    t_tx = time.time() - t0
    log.note(f"tx started at {t_tx:.3f}")
    print(f"# logging {args.seconds:.0f} s to {cap.name}")

    meas_t, pvt = [], []
    last_print = 0.0
    try:
        while time.time() - t0 - t_tx < args.seconds:
            try:
                blocks = link.pump()
            except (serial.SerialException, OSError) as exc:
                log.note(f"serial dropped ({exc}); reopening")
                link, cd = connect(log, args.port, wait_s=20.0)
                if link is None:
                    log.note("receiver did not come back in 20 s")
                    break
                continue
            t = time.time() - t0 - t_tx
            for blk in blocks:
                bid = sbf.block_id(blk)[0]
                if bid == sbf.MEAS_EPOCH:
                    m = sbf.meas_epoch(blk)
                    meas_t.append((t, len({x["sv"] for x in m["meas"]})))
                elif bid == sbf.PVT_GEODETIC:
                    pvt.append((t, sbf.pvt_geodetic(blk)))
            if t - last_print >= 10 and pvt:
                last_print = t
                p = pvt[-1][1]
                fix = (f"{p['mode_name']} {p['nrsv']} sv h {p['h']:.0f} m vu {p['vu']:.0f} m/s"
                       if p["mode"] else f"none ({p['error_name']})")
                print(f"{t:6.0f} | sats {meas_t[-1][1] if meas_t else 0:2d} | {fix}", flush=True)
    except KeyboardInterrupt:
        log.note("interrupted")
    finally:
        stop_tx(tx)
        if not args.keep_streams:
            try:
                for k in (1, 2, 3):
                    link.command(f"sso, Stream{k}, {cd}, none, off")
            except Exception:
                pass
        log.note("end; blocks " + " ".join(f"{k}:{v}" for k, v in sorted(log.counts.items())))
        log.flush()
        try:
            link.s.close()
        except Exception:
            pass
        if tx is not None:
            shutil.copyfile(errfile, str(cap) + ".hackrf.txt")
            hackrf_idle()                            # radio idle between scenarios (owner's rule)

    first_fix = next((t for t, p in pvt if p["mode"]), None)
    if first_fix is None:
        print("# no fix")
    else:
        since = f", {first_fix + t_tx - t_reset:.1f} s after the cold start" if args.cold_start else ""
        print(f"# first fix {first_fix:.1f} s after the transmitter/log start{since}")
    pad_end = min(170.0, args.seconds - 1)                # the flights' pad; a short listen-only run ends sooner
    pad = [n for t, n in meas_t if 5 <= t <= pad_end]
    errs = sorted({p["error"] for _, p in pvt if p["mode"] == 0})
    print(f"# pad: {len(pad) / (pad_end - 5) / 20 * 100:.0f} % of 20 Hz epochs, most satellites {max(pad) if pad else 0}; "
          f"PVT errors seen {errs}")
    print(f"# capture {cap}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
