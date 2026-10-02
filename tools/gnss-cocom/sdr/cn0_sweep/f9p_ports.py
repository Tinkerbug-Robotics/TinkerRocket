#!/usr/bin/env python3
"""ZED-F9P: read what the ports other than USB are set to send (RAM layer), then turn their output off, RAM only.

Why: at 10 Hz with GPS + Galileo + BeiDou (about 22 measurements) the F9P sends RXM-RAWX for ~45 s after a cold start
and then almost never, while NAV-PVT and RXM-SFRBX keep flowing -- the big messages starve and the small ones get
through. ubx_config.py configures only the port it talks on (USB), so UART1, UART2, I2C and SPI keep whatever the
board was saved with (an ArduSimple ships with NMEA on UART1); if a slow or unread port holds the transmit buffer the
receiver shares between ports, USB has no room for a 700-byte RAWX. One VALSET per port (VALSET is all-or-nothing).

    f9p_ports.py PORT [--read-only]
"""
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ubx_config import (LAYER_RAM, collect, parse_valget, poll, read_for, valget, valset,   # noqa: E402
                        CLS_CFG, MSG_VALGET)

PORTS = {   # port: (enabled key, baud key or None, output-protocol key base)
    "UART1": (0x10520005, 0x40520001, 0x10740000),
    "UART2": (0x10530005, 0x40530001, 0x10760000),
    "I2C": (0x10510003, None, 0x10720000),
    "SPI": (0x10640006, None, 0x107a0000),
    "USB": (None, None, 0x10780000),
}
PROTO = {1: "UBX", 2: "NMEA", 4: "RTCM3X"}


def read_port(ser, name):
    en, baud, base = PORTS[name]
    keys = [k for k in (en, baud) if k] + [base + b for b in PROTO]
    pl = poll(ser, CLS_CFG, MSG_VALGET, valget(keys)[6:-2], seconds=1.0)
    if pl is None:
        return f"{name:<6} no answer"
    v = parse_valget(pl)
    parts = []
    if en:
        parts.append("enabled" if v.get(en) else "DISABLED" if en in v else "enabled ?")
    if baud and baud in v:
        parts.append(f"{v[baud]} baud")
    out = [PROTO[b] for b in PROTO if v.get(base + b)]
    parts.append("output: " + (" + ".join(out) if out else "none"))
    return f"{name:<6} " + ", ".join(parts)


def main():
    port = sys.argv[1]
    ser = serial.Serial(port, 460800, timeout=0.05)
    time.sleep(0.3)
    print("  ports before:")
    for name in PORTS:
        print("    " + read_port(ser, name))
    if "--read-only" in sys.argv:
        return 0
    bad = []
    for name in ("UART1", "UART2", "I2C", "SPI"):
        base = PORTS[name][2]
        ser.reset_input_buffer()
        ser.write(valset([(base + b, 0) for b in PROTO], LAYER_RAM))
        acks = [m for c, m, _ in collect(read_for(ser, 1.0)) if c == 0x05]
        if 0x01 not in acks:
            bad.append(name)
    print("  ports after:")
    for name in PORTS:
        print("    " + read_port(ser, name))
    print("  port output: UART1, UART2, I2C, SPI off (RAM)" + (f" -- no ACK for {', '.join(bad)}" if bad else
                                                             " -- ACK"))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
