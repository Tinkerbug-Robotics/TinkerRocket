#!/usr/bin/env python3
"""Run a scenario radiated into a Faraday cage, tapping the receiver's UART.

Differs from the conducted runs in three ways that matter:

* **The receiver is not configured by us.** The flight computer owns it and sets
  it up at boot, including DYN_MODEL_AIRBORNE4g. This tool only listens, on a
  passive tap of the receiver's TXD line, so it cannot perturb the thing it is
  measuring. There is no seeded restart here; the SkyTraq `0x01` trick has a
  u-blox equivalent (UBX-CFG-RST) but injecting it would mean driving a line the
  flight computer is also driving.
* **The tap is a generic USB-serial adapter**, so it cannot be found by USB
  vendor the way the passthrough board can. Pass --port, or run --list.
* **Level is set by pad and geometry, not by a cable.** Free-space loss at L1 is
  only 26 dB at 30 cm, so inside a cage the separation is a first-order term. It
  is also fixed by the box, which leaves TX gain as the only live adjustment.

Always confirm the transmitter is actually streaming before believing a null
result: a run where hackrf_transfer failed to open the device looks exactly like
a run where the receiver heard nothing.
"""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import serial                                          # noqa: E402
from serial.tools import list_ports                    # noqa: E402
from gnss_nmea_monitor import Parser, _demux           # noqa: E402
from ublox_binary import (CLS_NAV, MSG_NAV_PVT, MSG_NAV_SAT,   # noqa: E402
                          parse_nav_pvt, parse_nav_sat)
sys.path.insert(0, str(HERE))
from ensure_hackrf import ensure_hackrf                    # noqa: E402

TX_ERR = "/tmp/hackrf_tx_radiated.err"


def list_candidate_ports():
    print("serial ports:")
    for p in sorted(list_ports.comports(), key=lambda x: x.device):
        vid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "----:----"
        print(f"  {p.device:<28} {vid}  {p.manufacturer or ''} {p.product or ''}")
    print("\nThe tap is whichever adapter is clipped to the receiver's TXD.\n"
          "A u-blox on the flight computer runs at 460800 (38400 factory default).")


def start_tx(c8: Path, freq: int, rate: int, gain: int):
    # The PortaPack boots into Mayhem and hides the radio; this is a no-op when
    # the HackRF is already there.
    if not ensure_hackrf():
        return None
    errf = open(TX_ERR, "w")
    tx = subprocess.Popen(
        ["hackrf_transfer", "-t", str(c8), "-f", str(freq),
         "-s", str(rate), "-a", "0", "-x", str(gain)],
        stdout=errf, stderr=subprocess.STDOUT)
    time.sleep(4.0)
    out = Path(TX_ERR).read_text()
    if tx.poll() is not None or "MB / " not in out:
        try:
            tx.kill()
        except Exception:
            pass
        print("!! hackrf_transfer is not streaming:")
        print(out[-800:] or "(no output)")
        return None
    print(f"# TX confirmed: {out.strip().splitlines()[-1]}")
    return tx


def stop_tx(tx):
    if tx is None:
        return
    tx.send_signal(signal.SIGINT)
    try:
        tx.wait(timeout=6)
    except subprocess.TimeoutExpired:
        tx.kill()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("-s", "--scenario", help="scenario name, e.g. spaceshot")
    ap.add_argument("-p", "--port", help="serial tap on the receiver's TXD")
    ap.add_argument("-b", "--baud", type=int, default=460800)
    ap.add_argument("-x", "--gain", type=int, default=40, help="HackRF TX gain 0-47")
    ap.add_argument("--freq", type=int, default=1575420000)
    ap.add_argument("--rate", type=int, default=2600000)
    ap.add_argument("--seconds", type=float, help="override capture length")
    ap.add_argument("--listen-only", action="store_true",
                    help="tap the receiver without transmitting, to check wiring")
    args = ap.parse_args()

    if args.list:
        list_candidate_ports()
        return 0
    if not args.port:
        list_candidate_ports()
        return "\n--port is required (see the list above)"

    c8 = meta = None
    if not args.listen_only:
        if not args.scenario:
            return "--scenario is required unless --listen-only"
        c8 = HERE / "c8" / f"{args.scenario}.C8"
        sc = HERE / "scenarios" / f"{args.scenario}.json"
        if not c8.exists():
            return f"no {c8}"
        meta = json.loads(sc.read_text()) if sc.exists() else None

    dur = args.seconds or (meta["duration_s"] + 15 if meta else 120)

    tx = None
    if not args.listen_only:
        print(f"# {args.scenario}: {c8.stat().st_size/1e9:.2f} GB, "
              f"{meta['duration_s']:.0f}s, gain {args.gain}")
        tx = start_tx(c8, args.freq, args.rate, args.gain)
        if tx is None:
            return 1
    else:
        print("# listen-only: not transmitting")

    cap = HERE / "captures" / (
        f"{args.scenario}_radiated.log" if args.scenario else "tap_check.log")
    cap.parent.mkdir(exist_ok=True)
    print(f"# tap {args.port} @ {args.baud}, logging to {cap.name}\n")
    print(f"{'t':>6} {'fix':<10} {'verdict':<8} {'sats':>5} {'used':>5} "
          f"{'lat':>10} {'lon':>11} {'alt m':>9} {'3D m/s':>8}")
    print("-" * 78)

    p = Parser()
    nsat = used = 0
    last = -99.0
    nbytes = nframes = 0
    try:
        with serial.Serial(args.port, args.baud, timeout=0.5) as ser, \
                open(cap, "w") as fh:
            buf = bytearray()
            t0 = time.time()
            while time.time() - t0 < dur:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                nbytes += len(chunk)
                buf.extend(chunk)
                for kind, data in _demux(buf):
                    t = time.time() - t0
                    nframes += 1
                    if kind == "ubx":
                        fh.write(f"{t:.3f} U {data.hex()}\n")
                        p.feed_ubx(data)
                    elif kind == "bin":
                        fh.write(f"{t:.3f} B {data.hex()}\n")
                        p.feed_binary(data)
                    else:
                        fh.write(f"{t:.3f} {data}\n")
                        p.feed(data)
                        continue
                    if kind == "ubx" and len(data) >= 2 and \
                            data[0] == CLS_NAV and data[1] == MSG_NAV_SAT:
                        ss = parse_nav_sat(data[2:]) or []
                        best = {}
                        for x in ss:
                            k = (x["gnss_name"], x["svid"])
                            best[k] = max(best.get(k, -99), x["cn0"])
                        nsat = sum(1 for v in best.values() if v >= 30)
                        used = len({(x["gnss_name"], x["svid"])
                                    for x in ss if x["used_in_fix"]})
                    if kind == "ubx" and len(data) >= 2 and \
                            data[0] == CLS_NAV and data[1] == MSG_NAV_PVT:
                        r = parse_nav_pvt(data[2:])
                        if not r or t - last < 5:
                            continue
                        last = t
                        e = p.epoch
                        print(f"{t:>6.0f} {r['fix_type_name']:<10} "
                              f"{e.verdict():<8} {nsat:>5} {used:>5} "
                              f"{r['lat']:>10.5f} {r['lon']:>11.5f} "
                              f"{r['alt_m']:>9.0f} {r['speed_mps']:>8.0f}")
    except serial.SerialException as exc:
        print(f"\n!! serial: {exc}")
    except KeyboardInterrupt:
        print("\n# interrupted")
    finally:
        stop_tx(tx)

    print(f"\n# {nbytes} bytes, {nframes} frames -> {cap}")
    if nframes == 0:
        print("# Nothing arrived. Check the tap is on the receiver's TXD (not the\n"
              "# flight computer's), that ground is common, and the baud rate --\n"
              "# a u-blox on this firmware runs 460800, not the 38400 default.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
