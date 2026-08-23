#!/usr/bin/env python3
"""Put the PortaPack into HackRF mode, so hackrf_transfer can find the radio.

A PortaPack boots into Mayhem every time and presents its own USB serial console
(1d50:6018) instead of a HackRF. Mayhem's `hackrf` console command drops out of
the application and hands USB to the radio, but it is session-only: there is no
persistent-memory flag to boot that way, and `cmd_hackrf` is just
m4_request_shutdown() followed by EventDispatcher::request_stop(). So it reverts
on every power cycle, which during this work silently turned into "no HackRF
boards found" three separate times, once mid-session.

Rather than remember to do it by hand, every entry point that transmits calls
ensure_hackrf() first. It is a no-op when the radio is already available.

Run standalone to fix it once:   ./ensure_hackrf.py
"""

from __future__ import annotations

import subprocess
import sys
import time

MAYHEM_VID = 0x1D50
MAYHEM_PID = 0x6018


def hackrf_present() -> bool:
    try:
        r = subprocess.run(["hackrf_info"], capture_output=True, text=True,
                           timeout=15)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return "Found HackRF" in (r.stdout + r.stderr)


def find_mayhem_console():
    """Device node of a PortaPack running Mayhem, or None."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    for p in list_ports.comports():
        if p.vid == MAYHEM_VID and p.pid == MAYHEM_PID:
            return p.device
        if p.product and "portapack" in p.product.lower():
            return p.device
    return None


def ensure_hackrf(timeout: float = 25.0, quiet: bool = False) -> bool:
    """Return True once a HackRF is available, switching modes if needed."""
    def say(msg):
        if not quiet:
            print(msg)

    if hackrf_present():
        return True

    port = find_mayhem_console()
    if port is None:
        say("# no HackRF and no PortaPack console -- is anything plugged in?")
        return False

    say(f"# PortaPack console on {port}; switching to HackRF mode")
    try:
        import serial
        with serial.Serial(port, 115200, timeout=0.5) as ser:
            time.sleep(0.3)
            ser.reset_input_buffer()
            ser.write(b"hackrf\r\n")
            time.sleep(1.0)
    except Exception as exc:
        # The port disappearing mid-write is the expected outcome, not a fault:
        # the device re-enumerates as soon as it takes the command.
        say(f"#   console closed ({type(exc).__name__}) -- expected on switch")

    deadline = time.time() + timeout
    while time.time() < deadline:
        if hackrf_present():
            say("# HackRF mode active")
            return True
        time.sleep(1.5)

    say(f"# still no HackRF after {timeout:.0f}s. Power-cycle the unit and retry.")
    return False


def main() -> int:
    ok = ensure_hackrf()
    if ok:
        r = subprocess.run(["hackrf_info"], capture_output=True, text=True)
        for line in (r.stdout + r.stderr).splitlines():
            if any(k in line for k in ("Board ID", "Firmware", "Hardware Rev",
                                       "Serial number")):
                print("  " + line.strip())
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
