#!/usr/bin/env python3
"""Block until the PortaPack/HackRF is on the USB bus, then hand back its state.

Detection is by **USB vendor id 0x1D50**, not by product name. Both faces of the
unit live under that vendor -- the Mayhem console (0x1D50:0x6018, a CDC serial
port) and the radio itself (0x1D50:0x6089, a libusb device with no serial port
at all) -- and a name-matching watcher silently fails on whichever one it did
not think of. The two look completely different to the host, which is the whole
reason this is easy to get wrong:

    HackRF mode : no serial port anywhere; only hackrf_info / ioreg see it
    Mayhem mode : a serial port appears, and hackrf_info reports nothing

So "no serial port" is not evidence of absence, and neither is "hackrf_info
found nothing". Both have to be checked, which is what this does.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

VENDOR = 0x1D50


def bus_has_vendor(vid: int = VENDOR) -> bool:
    """True if any device with this vendor id is enumerated, mode-independent."""
    try:
        out = subprocess.run(["ioreg", "-p", "IOUSB", "-l", "-w0"],
                             capture_output=True, text=True, timeout=15).stdout
    except Exception:
        return False
    return f'"idVendor" = {vid}' in out


def serial_ports_for(vid: int = VENDOR):
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [p.device for p in list_ports.comports() if p.vid == vid]


def hackrf_ready() -> bool:
    try:
        r = subprocess.run(["hackrf_info"], capture_output=True, text=True,
                           timeout=15)
    except Exception:
        return False
    return "Found HackRF" in (r.stdout + r.stderr)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timeout", type=float, default=1800.0)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    def say(m):
        if not args.quiet:
            print(m, flush=True)

    say(f"# waiting for USB vendor 0x{VENDOR:04X} (PortaPack or HackRF), "
        f"up to {args.timeout/60:.0f} min")
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        if hackrf_ready():
            say("# HackRF already in radio mode")
            return 0
        if bus_has_vendor() or serial_ports_for():
            ports = serial_ports_for()
            say(f"# device on the bus (console: {ports or 'none -- already radio mode'})")
            return 0
        time.sleep(4)
    say("# timed out: nothing with vendor 0x1D50 ever appeared")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
