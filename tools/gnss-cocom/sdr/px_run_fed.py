#!/usr/bin/env python3
"""px1105r_run.py unchanged, except that hackrf_transfer is fed through c8_feeder.py (a read-ahead buffer in memory)
instead of reading the C8 itself: run_radiated.start_tx/stop_tx are swapped before px1105r_run.py runs (it imports
them from run_radiated). Same arguments as px1105r_run.py. The step before hackrf_tx_ram (px_run_ram.py), which
does the same job inside one process and is what the wide flights use.

    px_run_fed.py GAIN C8 SECONDS NAV [px1105r_run.py options]
"""
import runpy
import subprocess
import sys
import time
from pathlib import Path

SDR = Path(__file__).resolve().parent
FEEDER = SDR / "c8_feeder.py"
sys.path.insert(0, str(SDR))
import run_radiated  # noqa: E402


def start_tx_fed(c8, freq, rate, gain, errfile, extra=()):
    if not run_radiated.ensure_hackrf():
        return None
    errf = open(errfile, "w")
    feeder = subprocess.Popen([sys.executable, str(FEEDER), str(c8)], stdout=subprocess.PIPE)
    tx = subprocess.Popen(["hackrf_transfer", "-t", "-", "-f", str(freq), "-s", str(rate), "-a", "0",
                           "-x", str(gain), *extra], stdin=feeder.stdout, stdout=errf, stderr=subprocess.STDOUT)
    feeder.stdout.close()                         # hackrf_transfer holds the pipe; EOF reaches it when the file ends
    time.sleep(4.0)
    out = Path(errfile).read_text()
    failed = ("hackrf_open() failed" in out or "not found" in out.lower()
              or tx.poll() is not None or "MB / " not in out)
    if failed:
        for p in (tx, feeder):
            try:
                p.kill()
            except Exception:
                pass
        print("!! hackrf_transfer is not streaming (fed):")
        print(out[:400] or "(no output)")
        return None
    print(f"# TX confirmed, fed from a read-ahead buffer: {out.strip().splitlines()[-1]}")
    tx._feeder = feeder
    return tx


_stop = run_radiated.stop_tx


def stop_tx_fed(tx):
    _stop(tx)
    f = getattr(tx, "_feeder", None) if tx is not None else None
    if f is not None:
        try:
            f.kill()
        except Exception:
            pass


run_radiated.start_tx = start_tx_fed
run_radiated.stop_tx = stop_tx_fed
sys.argv = [str(SDR / "px1105r_run.py")] + sys.argv[1:]
runpy.run_path(str(SDR / "px1105r_run.py"), run_name="__main__")
