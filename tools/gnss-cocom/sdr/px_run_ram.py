#!/usr/bin/env python3
"""px1105r_run.py unchanged, except that the transmitter is hackrf_tx_ram (hackrf_tx_ram/: the file read ahead into
a locked 2 GiB ring by its own thread, the USB callback only copies from memory) instead of hackrf_transfer:
run_radiated.start_tx is swapped before px1105r_run.py imports it. Same options, same -B lines, same checks.
TX_NOISE_DB=D in the environment passes -N D: the file's C/N0 lowered by D dB with added noise (level and noise
density unchanged). TX_CARRIER_HZ=F passes -C F: every carrier shifted by F Hz as the file is read (the rig's -23 Hz
correction for an uncorrected wide file).

    px_run_ram.py GAIN C8 SECONDS NAV [px1105r_run.py options]
"""
import os
import runpy
import subprocess
import sys
import time
from pathlib import Path

SDR = Path(__file__).resolve().parent
TX_BIN = SDR / "hackrf_tx_ram" / "hackrf_tx_ram"
sys.path.insert(0, str(SDR))
import run_radiated  # noqa: E402


def start_tx_ram(c8, freq, rate, gain, errfile, extra=()):
    if not run_radiated.ensure_hackrf():
        return None
    errf = open(errfile, "w")
    noise = os.environ.get("TX_NOISE_DB", "")
    carrier = os.environ.get("TX_CARRIER_HZ", "")
    tx = subprocess.Popen([str(TX_BIN), "-t", str(c8), "-f", str(freq), "-s", str(rate), "-a", "0",
                           "-x", str(gain), *extra, *(["-N", noise] if noise else []),
                           *(["-C", carrier] if carrier else [])],
                          stdout=errf, stderr=subprocess.STDOUT)
    time.sleep(5.0)
    out = Path(errfile).read_text()
    failed = ("hackrf_open() failed" in out or "not found" in out.lower()
              or tx.poll() is not None or "MB / " not in out)
    if failed:
        try:
            tx.kill()
        except Exception:
            pass
        print("!! hackrf_tx_ram is not streaming:")
        print(out[:400] or "(no output)")
        return None
    added = [ln for ln in out.splitlines() if "added noise" in ln]
    if noise and not added:
        try:
            tx.kill()
        except Exception:
            pass
        print(f"!! TX_NOISE_DB={noise} asked but hackrf_tx_ram reports no added noise")
        return None
    if added:
        print(f"# {added[0]}")
    shifted = [ln for ln in out.splitlines() if "carrier shift -C" in ln]
    if carrier and not shifted:
        try:
            tx.kill()
        except Exception:
            pass
        print(f"!! TX_CARRIER_HZ={carrier} asked but hackrf_tx_ram reports no carrier shift")
        return None
    if shifted:
        print(f"# {shifted[0]}")
    print(f"# TX confirmed (hackrf_tx_ram): {out.strip().splitlines()[-1]}")
    return tx


if not TX_BIN.exists():
    sys.exit(f"!! {TX_BIN} not built: run hackrf_tx_ram/build.sh")
run_radiated.start_tx = start_tx_ram
sys.argv = [str(SDR / "px1105r_run.py")] + sys.argv[1:]
runpy.run_path(str(SDR / "px1105r_run.py"), run_name="__main__")
