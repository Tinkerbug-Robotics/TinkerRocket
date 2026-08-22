#!/usr/bin/env python3
"""Run a scenario radiated at the rocket computer, capturing its console.

The rocket-computer's u-blox cannot be tapped the way the PX1125R was: its
firmware sets UART1 to UBX-only and the rocket computer consumes the stream, so
nothing raw reaches USB. Instead the FC is built with TR_GNSS_COCOM_DIAG=1 and
logs fix state and per-satellite C/N0 to the console that is already attached.
This drives the transmitter, records that console, and converts it into the same
capture format the conducted rig produced, so correlate.py / recovery.py /
plot_flight.py all work unchanged.

Unlike run_radiated.py there is no serial tap and no receiver configuration --
the rocket computer owns the receiver end to end. The only live adjustment is TX
gain, because the cage fixes the geometry.
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
sys.path.insert(0, str(HERE))
from ensure_hackrf import ensure_hackrf                    # noqa: E402

TX_ERR = "/tmp/hackrf_tx_fc.err"


def start_tx(c8: Path, freq: int, rate: int, gain: int):
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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-s", "--scenario", required=True)
    ap.add_argument("-p", "--port", default="/dev/cu.usbmodem101",
                    help="rocket-computer console")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-x", "--gain", type=int, default=12, help="HackRF TX gain 0-47")
    ap.add_argument("--freq", type=int, default=1575420000)
    ap.add_argument("--rate", type=int, default=2600000)
    ap.add_argument("--margin", type=float, default=20.0,
                    help="extra seconds of capture after the scenario ends")
    args = ap.parse_args()

    import serial

    c8 = HERE / "c8" / f"{args.scenario}.C8"
    sc = HERE / "scenarios" / f"{args.scenario}.json"
    if not c8.exists():
        return f"no {c8}"
    meta = json.loads(sc.read_text())
    dur = meta["duration_s"] + args.margin

    con = HERE / "captures" / f"fc_{args.scenario}_console.log"
    out = HERE / "captures" / f"fc_{args.scenario}.log"
    con.parent.mkdir(exist_ok=True)

    print(f"# {args.scenario}: {c8.stat().st_size/1e9:.2f} GB, "
          f"{meta['duration_s']:.0f}s, gain {args.gain}")
    tx = start_tx(c8, args.freq, args.rate, args.gain)
    if tx is None:
        return 1

    print(f"# capturing {args.port} for {dur:.0f}s -> {con.name}")
    n = fixes = 0
    t0 = time.time()
    try:
        with serial.Serial(args.port, args.baud, timeout=0.5) as ser, \
                open(con, "wb") as fh:
            buf = b""
            while time.time() - t0 < dur:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                fh.write(chunk)
                fh.flush()
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode(errors="replace")
                    if "[COCOM] P " not in s:
                        continue
                    n += 1
                    ok = " ok=1 " in s
                    fixes += ok
                    if n % 30 == 0:
                        el = time.time() - t0
                        print(f"  t={el:>5.0f}s  {n:>4} epochs  {fixes:>4} with a fix"
                              f"  ({100*fixes/n:>4.0f}%)  last: {'FIX' if ok else 'no fix'}")
    except KeyboardInterrupt:
        print("\n# interrupted")
    finally:
        tx.send_signal(signal.SIGINT)
        try:
            tx.wait(timeout=6)
        except subprocess.TimeoutExpired:
            tx.kill()

    print(f"\n# {n} epochs, {fixes} with a fix -> {con}")
    r = subprocess.run([sys.executable, str(HERE / "cocom_fcdiag.py"),
                        str(con), "-o", str(out)], capture_output=True, text=True)
    print(r.stdout + r.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
