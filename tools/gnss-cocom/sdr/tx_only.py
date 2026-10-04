#!/usr/bin/env python3
"""Transmit a C8 exactly as px1105r_run.py does (run_radiated.start_tx: hackrf_transfer -t FILE -f FREQ -s RATE -a 0
-x GAIN -B [-b BBF]) but with no receiver on the Mac -- for underrun tests with the receiver board on a charger.
Pre-caches the file, switches the PortaPack to HackRF mode, transmits to the end of the file, then leaves the radio
idle (PortaPack back in Mayhem). The transmitter's log is the result: underrun times as T+ (the file's second k
plays at about T = k + 1 - PAD). TX_BIN=hackrf_tx_ram/hackrf_tx_ram runs the buffered transmitter instead (same
options, same -B lines).

    tx_only.py C8 FREQ RATE GAIN BBF PAD LOG          (C8: a file name in c8/)
"""
import os
import re
import subprocess
import sys
import time
from pathlib import Path

TX_BIN = os.environ.get("TX_BIN", "hackrf_transfer")

SDR = Path(__file__).resolve().parent
sys.path.insert(0, str(SDR))
from ensure_hackrf import ensure_hackrf, hackrf_idle  # noqa: E402

c8, freq, rate, gain, bbf, pad, log = (sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]),
                                       int(sys.argv[5]), float(sys.argv[6]), sys.argv[7])
path = SDR / "c8" / c8
size = path.stat().st_size
secs = size / (2 * rate)
print(f"# {time.strftime('%H:%M:%S')} pre-caching {c8} ({size / 1e9:.1f} GB, {secs:.0f} s)", flush=True)
subprocess.run(["cat", str(path)], stdout=subprocess.DEVNULL)
if not ensure_hackrf():
    sys.exit("!! no HackRF")
print(f"# {time.strftime('%H:%M:%S')} transmitting with {Path(TX_BIN).name}, receiver not on the Mac", flush=True)
with open(log, "w") as f:
    tx = subprocess.Popen([TX_BIN, "-t", str(path), "-f", str(freq), "-s", str(rate), "-a", "0",
                           "-x", str(gain), "-B", "-b", str(bbf)], stdout=f, stderr=subprocess.STDOUT)
    try:
        tx.wait(timeout=secs + 30)
    except subprocess.TimeoutExpired:
        tx.send_signal(2)
        tx.wait(timeout=10)
time.sleep(1.0)
hackrf_idle()
prev, k = 0, -1
for s in open(log, errors="replace"):
    m = re.search(r"(\d+) underruns, longest (\d+) bytes", s)
    if not m:
        continue
    k += 1
    n, longest = int(m.group(1)), int(m.group(2))
    if n != prev:
        print(f"  underrun in stats line {k} (about T{k + 1 - pad:+.0f} s): total {n}, longest so far "
              f"{longest / 2 / rate * 1e3:.1f} ms")
    prev = n
print(f"# {time.strftime('%H:%M:%S')} done: {k + 1} s transmitted, {prev} underruns")
