#!/usr/bin/env python3
"""SkyTraq navigation (dynamics) mode, AN0037 0x64/0x17; query 0x64/0x18 -> 0x64/0x8B.
   nav_mode.py set N   |   nav_mode.py sweep   (cold start, try every mode, read back)"""
import sys, time
from skytraq_raw import Link, frame

MODES = {0: "auto", 1: "pedestrian", 2: "car", 3: "marine", 4: "balloon",
         5: "airborne", 7: "quadcopter", 9: "SLR"}


def cold_start(link):
    import struct
    pl = bytes([0x01, 0x03]) + struct.pack(">H", 2026) + bytes([8, 18, 8, 29, 0]) + struct.pack(">hhh", 0, -11900, 1200)
    link.command(pl, timeout=3.0); time.sleep(3.0); link.frames()


def set_mode(link, mode, flash=False):
    ack, _ = link.command(bytes([0x64, 0x17, mode, 1 if flash else 0]), timeout=2.5)
    return ack


def read_mode(link):
    # 0x64/0x18 query; reply is a 0x64 message whose sub-id is 0x8B, mode in the next byte
    ack, r = link.command(bytes([0x64, 0x18]), want_reply=0x64, timeout=2.5)
    if r and len(r) >= 3 and r[1] == 0x8B:
        return r[2]
    return None


def main():
    port = sys.argv[sys.argv.index("-p") + 1] if "-p" in sys.argv else "/dev/cu.usbmodem101"
    link = Link(port); time.sleep(0.3); link.frames()
    try:
        if sys.argv[1] == "set":
            m = int(sys.argv[2])
            print(f"set nav mode {m} ({MODES.get(m, '?')}):", "ACK" if set_mode(link, m) else "NACK/none")
            print("read back:", read_mode(link))
        elif sys.argv[1] == "sweep":
            cold_start(link)
            print(f"{'mode':<12}{'set':>6}{'reads back':>12}")
            for m, name in MODES.items():
                ack = set_mode(link, m); time.sleep(0.4)
                rb = read_mode(link)
                print(f"{m} {name:<10}{('ACK' if ack else 'NACK' if ack is False else '-'):>6}{str(rb):>12}")
    finally:
        link.close()


if __name__ == "__main__":
    main()
