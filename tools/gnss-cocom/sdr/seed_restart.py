#!/usr/bin/env python3
"""Restart the receiver seeded with the injected scenario's time and position.

Why this exists: on the bench the receiver comes up holding its *real* last-known
time and position, and it fights the injected signal rather than accepting it. A
watch run showed the receiver's clock sitting 618 s away from the injected one
for three minutes, tracking satellites and decoding ephemeris the whole time but
never using any of them in a fix -- then snapping to within 1 s the moment it
finally accepted the signal's time.

Seeding removes that fight: tell it where and when it is, restart, and it
acquires into the injected scenario directly.

Message layouts are from the AN0037 table in SkyTraqGit/skytraq-cmd-api:

  0x01 SYSTEM RESTART   15 bytes
     [1] id  [2] start mode 1=hot 2=warm 3=cold
     [3-4] UTC year U16   [5] month  [6] day  [7] hour  [8] min  [9] sec
     [10-11] lat S16 1/100 deg   [12-13] lon S16 1/100 deg   [14-15] alt S16 m
  0x64/0x17 CONFIGURE GNSS NAVIGATION MODE   4 bytes
     [3] 0=Auto 1=Pedestrian 2=Car 3=Marine 4=Balloon 5=Airborne
         7=Quadcopter 9=SLR (Speed Lag Reduced)
     [4] 0=SRAM 1=SRAM+FLASH
  0x0C CONFIGURE SYSTEM POWER MODE   3 bytes
     [2] 0=Normal 1=Power Save   [3] 0=SRAM 1=SRAM+FLASH 2=temporary

This is the closest thing the Phoenix protocol has to u-blox's dynamic model
knobs. There is no exposed tracking-loop bandwidth or Doppler search window:
navigation mode is the only dynamics lever, and "Configure GPS Parameter Search
Engine Number" (0x64/0x0A) was removed from the protocol in a later revision,
leaving only the query. Power mode matters for a different reason -- AN0037 says
power save is enabled by default "to reduce current consumption by the search
engine", which is exactly the engine that has to re-acquire after a boost.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
try:
    import serial
    from skytraq_cmd import frame, send
    from gnss_nmea_monitor import autodetect_port
except ImportError as exc:  # pragma: no cover
    raise SystemExit(f"needs the receiver half and pyserial ({exc})")

MODES = {"hot": 1, "warm": 2, "cold": 3}
NAV_MODES = {"auto": 0, "pedestrian": 1, "car": 2, "marine": 3,
             "balloon": 4, "airborne": 5, "quadcopter": 7, "slr": 9}


def restart_payload(mode: int, when, lat: float, lon: float, alt: float) -> bytes:
    y, mo, d, h, mi, s = when
    # Altitude is a signed 16-bit metre field the doc bounds at -1000..18300.
    # It is only an acquisition hint, so clamping a high-altitude scenario to
    # the top of the range costs nothing -- but silently wrapping would seed
    # the receiver somewhere absurd, so clamp explicitly.
    alt_i = max(-1000, min(18300, int(round(alt))))
    return struct.pack(">BBHBBBBBhhh", 0x01, mode, y, mo, d, h, mi, s,
                       int(round(lat * 100)), int(round(lon * 100)), alt_i)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port")
    ap.add_argument("-t", "--start", required=True,
                    help="scenario start, YYYY/MM/DD,hh:mm:ss (same as gps-sdr-sim -t)")
    ap.add_argument("--lat", type=float, default=40.0)
    ap.add_argument("--lon", type=float, default=-119.0)
    ap.add_argument("--alt", type=float, default=100.0)
    ap.add_argument("--mode", choices=list(MODES), default="warm",
                    help="warm keeps the almanac and uses the seed; cold discards more")
    ap.add_argument("--nav-mode", choices=list(NAV_MODES),
                    help="platform dynamics. 'airborne' for the ramps; 'slr' "
                         "(Speed Lag Reduced) is worth trying against high "
                         "acceleration")
    ap.add_argument("--power-mode", choices=("normal", "save"),
                    help="'normal' disables the default power-save mode, which "
                         "throttles the search engine")
    args = ap.parse_args()

    date, clock = args.start.split(",")
    y, mo, d = (int(x) for x in date.split("/"))
    h, mi, s = (int(float(x)) for x in clock.split(":"))

    port = args.port or autodetect_port()
    print(f"# {port}")
    with serial.Serial(port, 115200, timeout=1) as ser:
        if args.power_mode:
            send(ser, bytes([0x0C, 0 if args.power_mode == "normal" else 1, 0x00]),
                 f"power mode -> {args.power_mode}", expect_id=0x0C)
        if args.nav_mode:
            code = NAV_MODES[args.nav_mode]
            send(ser, bytes([0x64, 0x17, code, 0x00]),
                 f"nav mode -> {args.nav_mode}", expect_id=0x64)
        payload = restart_payload(MODES[args.mode], (y, mo, d, h, mi, s),
                                  args.lat, args.lon, args.alt)
        print(f"  seed: {y:04d}-{mo:02d}-{d:02d} {h:02d}:{mi:02d}:{s:02d} UTC, "
              f"{args.lat:.4f}, {args.lon:.4f}, {args.alt:.0f} m")
        print(f"  frame: {frame(payload).hex(' ')}")
        send(ser, payload, f"{args.mode} restart", expect_id=0x01)
    return 0


if __name__ == "__main__":
    sys.exit(main())
