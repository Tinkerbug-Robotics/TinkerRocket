#!/usr/bin/env python3
"""Reset an ESP32-S3 over its USB-Serial-JTAG port and log its console.

Each line is stamped with the host's seconds since the reset. Opening the port
resets the chip on macOS anyway; the EN pulse makes the reset deliberate, so a
rig replay starts its boot clock here.

    console.py PORT SECONDS OUT_LOG
"""
import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("port", help="e.g. /dev/cu.usbmodem1101 -- check it is the FC by its MAC first")
    ap.add_argument("seconds", type=float)
    ap.add_argument("out")
    args = ap.parse_args()

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = args.port, 115200, 0.2
    s.dtr = False
    s.rts = False
    s.open()
    s.rts = True                     # EN low through the USB-JTAG bridge
    time.sleep(0.1)
    s.rts = False
    t0 = time.time()
    with open(args.out, "w") as fh:
        while time.time() - t0 < args.seconds:
            try:
                line = s.readline()
            except serial.SerialException:
                # the chip re-enumerates after its reset: reopen and go on
                time.sleep(0.5)
                try:
                    s.close()
                    s.open()
                except Exception:   # noqa: BLE001 - keep trying until the window ends
                    pass
                continue
            if line:
                fh.write("%8.3f %s\n" % (time.time() - t0, line.decode("utf-8", "replace").rstrip()))
                fh.flush()


if __name__ == "__main__":
    main()
