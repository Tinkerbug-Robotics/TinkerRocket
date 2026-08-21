#!/usr/bin/env python3
"""Diagnose a silent UART receiver: is it the wiring, the baud, or the module?

A dead serial link gives you exactly one symptom -- zero bytes -- and at least
four causes that produce it identically: crossed vs straight wiring, wrong baud,
no power, no common ground. Software alone cannot separate reversed TX/RX from
an unpowered module, because both are silence. What it *can* do is test the half
of the path that ends at the adapter, which narrows the search to one side.

    ./serial_probe.py                       # sweep bauds, report what arrives
    ./serial_probe.py --loopback            # after shorting the adapter TX to RX

The loopback is the discriminator. Pull the module's wires off, bridge the
adapter's own TX and RX pins, and run --loopback: if the bytes come back, the
adapter, its driver and the USB cable are all good and the fault is on the
module side of the connector. If they do not, stop looking at the module.
"""

from __future__ import annotations

import argparse
import time

BAUDS = (9600, 4800, 19200, 38400, 57600, 115200, 230400)


def _ubx_frames(buf: bytes) -> int:
    """Count checksum-valid UBX frames, so a binary stream is not read as noise."""
    n = i = 0
    while True:
        i = buf.find(b"\xb5\x62", i)
        if i < 0 or i + 6 > len(buf):
            return n
        ln = int.from_bytes(buf[i + 4:i + 6], "little")
        end = i + 6 + ln + 2
        if ln > 4096 or end > len(buf):
            i += 2
            continue
        a = b = 0
        for byte in buf[i + 2:end - 2]:
            a = (a + byte) & 0xFF
            b = (b + a) & 0xFF
        if (a, b) == (buf[end - 2], buf[end - 1]):
            n += 1
            i = end
        else:
            i += 2

# Receivers seen on this bench and where they idle. Air530 (AT6558R) is 9600 and
# NMEA by default; u-blox native USB ignores baud entirely.
KNOWN = {9600: "u-blox M8 UART default, Air530/AT6558R, most NMEA modules",
         38400: "u-blox F9/M9 UART default",
         115200: "common after reconfiguration",
         460800: "flight-computer u-blox on this project"}


def sweep(port: str, dwell: float = 2.5):
    import serial
    print(f"  {'baud':>7} {'bytes':>7} {'printable':>10} {'$':>4} {'UBX':>5}  sample")
    print("  " + "-" * 74)
    best = None
    for baud in BAUDS:
        try:
            with serial.Serial(port, baud, timeout=0.4) as s:
                time.sleep(0.25)
                s.reset_input_buffer()
                buf = b""
                t0 = time.time()
                while time.time() - t0 < dwell:
                    buf += s.read(4096)
        except Exception as exc:
            print(f"  {baud:>7}  !! {type(exc).__name__}: {exc}")
            continue
        pr = sum(1 for b in buf if 32 <= b < 127 or b in (10, 13))
        frac = (100 * pr // len(buf)) if buf else 0
        # A binary-only receiver is not garbage, it just is not ASCII. Counting
        # UBX sync pairs alongside NMEA '$' stops a perfectly good UBX stream
        # from being reported as noise at every baud -- which is what happened
        # with the NEO-M8T, whose UART carries UBX and no NMEA at all.
        n_ubx = _ubx_frames(buf)
        smp = buf[:40].decode("ascii", "replace").replace("\r", "\\r").replace("\n", "\\n")
        note = f"   <- {KNOWN[baud]}" if baud in KNOWN and not buf else ""
        print(f"  {baud:>7} {len(buf):>7} {frac:>9}% {buf.count(b'$'):>4} "
              f"{n_ubx:>5}  {smp}{note}")
        score = buf.count(b"$") + n_ubx
        if score > 2 and (best is None or score > best[1]):
            best = (baud, score)
    return best


def loopback(port: str, baud: int = 9600) -> bool:
    import serial
    probe = b"LOOPBACK-CHECK-0123456789\r\n"
    with serial.Serial(port, baud, timeout=1.0) as s:
        time.sleep(0.2)
        s.reset_input_buffer()
        s.write(probe)
        s.flush()
        got = b""
        t0 = time.time()
        while time.time() - t0 < 1.5 and probe not in got:
            got += s.read(256)
    ok = probe.strip() in got
    print(f"  loopback @ {baud}: {'ECHOED' if ok else 'silent'} "
          f"({len(got)} bytes back)")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/cu.usbserial-0001")
    ap.add_argument("--loopback", action="store_true",
                    help="test the adapter with its TX shorted to its RX")
    ap.add_argument("--dwell", type=float, default=2.5)
    args = ap.parse_args()

    if args.loopback:
        ok = loopback(args.port)
        print("\n  " + ("The adapter, driver and USB cable are good. The fault is\n"
                        "  on the module side: power, ground, or TX/RX orientation."
                        if ok else
                        "Nothing came back. Either the short is not actually across\n"
                        "  the adapter's own TX and RX pins, or the adapter/driver is\n"
                        "  at fault -- do not go on suspecting the module yet."))
        return 0 if ok else 1

    best = sweep(args.port, args.dwell)
    if best:
        print(f"\n  data at {best[0]} baud")
        return 0
    print("\n  Nothing at any baud. Software cannot tell these apart -- all four\n"
          "  produce identical silence:\n"
          "    * TX/RX straight through instead of crossed (adapter RX <- module TX)\n"
          "    * module unpowered\n"
          "    * no common ground\n"
          "    * module outputting at a baud not swept\n"
          "  Run --loopback with the adapter's TX shorted to its RX to rule the\n"
          "  adapter and cable in or out before touching the module.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
