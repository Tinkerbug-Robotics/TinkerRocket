#!/usr/bin/env python3
"""Raise an Air530 / AT6558R off its 9600-baud default before measuring it.

9600 baud is 960 bytes/s. An Air530 tracking 13 GPS + BeiDou satellites emits
GGA, RMC, VTG, two GSA and four-plus GSV sentences every second, and that does
not fit. It does not fail loudly either -- sentences are simply dropped, and the
capture that results looks like a receiver losing satellites. On the first
gentle_alt run it produced 52 epochs where the 180 s prologue should have given
about 180, and satellite counts that collapsed to 0-1 inside every blocked
window while the other three receivers on this bench held 10-14. That reads as a
dramatic receiver difference and is an artifact of the wire.

CASIC parts take NMEA-style PCAS commands rather than a binary protocol:

    $PCAS01,<rate>   0=4800 1=9600 2=19200 3=38400 4=57600 5=115200
    $PCAS02,<ms>     navigation rate
    $PCAS03,...      per-sentence output rates

The change takes effect immediately, so the host has to reopen at the new speed;
this verifies by doing exactly that and reading real sentences back, because a
receiver that ignored the command and one that accepted it are indistinguishable
until you look.
"""

from __future__ import annotations

import argparse
import time

RATES = {4800: 0, 9600: 1, 19200: 2, 38400: 3, 57600: 4, 115200: 5}


def nmea(body: str) -> bytes:
    """Wrap a PCAS body in $...*CC\\r\\n with an XOR checksum."""
    ck = 0
    for ch in body:
        ck ^= ord(ch)
    return f"${body}*{ck:02X}\r\n".encode()


def read_for(ser, seconds: float) -> bytes:
    buf = b""
    t0 = time.time()
    while time.time() - t0 < seconds:
        buf += ser.read(4096)
    return buf


def looks_alive(buf: bytes) -> tuple[bool, int]:
    lines = [l for l in buf.decode("ascii", "replace").split("\r\n")
             if l.startswith("$") and "*" in l]
    return (len(lines) > 3, len(lines))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/cu.usbserial-0001")
    ap.add_argument("--from-baud", type=int, default=9600)
    ap.add_argument("--to-baud", type=int, default=115200, choices=sorted(RATES))
    args = ap.parse_args()

    import serial
    if args.to_baud not in RATES:
        return f"unsupported rate {args.to_baud}"

    with serial.Serial(args.port, args.from_baud, timeout=0.4) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()
        ok, n = looks_alive(read_for(ser, 2.0))
        print(f"  at {args.from_baud}: {n} sentences  {'ok' if ok else '-- silent'}")
        if not ok:
            return ("nothing at the starting baud; not sending a rate change "
                    "blind")
        cmd = nmea(f"PCAS01,{RATES[args.to_baud]}")
        print(f"  sending {cmd.decode().strip()}")
        ser.write(cmd)
        ser.flush()
        time.sleep(0.5)

    time.sleep(0.5)
    with serial.Serial(args.port, args.to_baud, timeout=0.4) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()
        buf = read_for(ser, 3.0)
        ok, n = looks_alive(buf)
        rate = len(buf) / 3.0
        print(f"  at {args.to_baud}: {n} sentences, {rate:.0f} B/s  "
              f"{'ok' if ok else '-- silent'}")
        if not ok:
            print("  !! the receiver did not take the rate change; it is probably\n"
                  "     still at the old baud. Nothing was lost -- retry, or leave\n"
                  "     it at 9600 and trim sentences with $PCAS03 instead.")
            return 1
        head = (args.to_baud / 10.0)
        print(f"  headroom: {rate:.0f} of {head:.0f} B/s "
              f"({100*rate/head:.0f}% used)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
