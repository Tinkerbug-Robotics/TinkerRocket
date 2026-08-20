#!/usr/bin/env python3
"""Resolve a u-blox receiver's serial port by USB vendor id, not by path.

Device paths on this bench are not stable. The ZED-F9P came up as
/dev/cu.usbmodem101 on one boot and /dev/cu.usbmodem2101 on the next, and the
PortaPack takes a usbmodem name of its own whenever it boots into Mayhem -- so
a hardcoded path is not merely fragile, it can name the *wrong device*, and
writing configuration frames at a transmitter is a good way to waste an
afternoon. u-blox receivers all enumerate under vendor 0x1546.

Run standalone to see what is present:   ./find_ublox.py
"""

from __future__ import annotations

UBLOX_VID = 0x1546


def find_ublox(prefer: str | None = None) -> str | None:
    """Device node of an attached u-blox receiver, or None.

    `prefer` short-circuits an explicit --port; passing "auto" or None searches.
    """
    if prefer and prefer != "auto":
        return prefer
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    hits = [p for p in list_ports.comports() if p.vid == UBLOX_VID]
    if not hits:
        return None
    return sorted(hits, key=lambda p: p.device)[0].device


def main() -> int:
    from serial.tools import list_ports
    found = False
    for p in sorted(list_ports.comports(), key=lambda x: x.device):
        vid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "----:----"
        mark = "  <- u-blox" if p.vid == UBLOX_VID else ""
        found = found or p.vid == UBLOX_VID
        print(f"  {p.device:<34} {vid}  {p.product or ''}{mark}")
    if not found:
        print("\n  no u-blox receiver on the bus")
        return 1
    print(f"\n  resolved: {find_ublox()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
