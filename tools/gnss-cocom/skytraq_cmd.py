#!/usr/bin/env python3
"""Send SkyTraq binary configuration frames through the passthrough bridge (#491).

The PX1125R on the TinkerNav boards is often left in **binary** output mode by
earlier RTK work, in which case it emits SkyTraq navigation frames and not a
single NMEA sentence -- so gnss_nmea_monitor.py sees a live, correctly-tuned
link with zero parseable data.  This flips it back and turns on the four
sentences the COCOM measurement needs.

Wire facts, taken from Tinkerbug's TR_SkyTraqNMEA (sendFrame / setNmeaSentence)
rather than transcribed from a datasheet:

  framing   A0 A1 | len_hi | len_lo | payload | XOR(payload) | 0D 0A
  ACK       message id 0x83, body = the id being acked
  NACK      message id 0x84
  msg type  id 0x09: [09][type][attr]   type 0=none 1=NMEA 2=binary
  sentence  id 0x64 sub 0x3B: [64][3B][3 ASCII][on][attr]
  rate      id 0x0E: [0E][hz][attr]
  version   id 0x02: [02][kind]  -> replies 0x80 with the version string
  attr      0 = SRAM only (reverts on power cycle), 1 = SRAM + flash

**Defaults to SRAM only.** These boards carry a stored RTK configuration that
someone's workflow depends on; a bench measurement has no business burning
itself into flash. Pass --flash only if you mean it, and know a power cycle will
no longer undo it.

Examples:
  python3 tools/gnss-cocom/skytraq_cmd.py --version
  python3 tools/gnss-cocom/skytraq_cmd.py --nmea          # the usual fix
  python3 tools/gnss-cocom/skytraq_cmd.py --msg-type nmea
  python3 tools/gnss-cocom/skytraq_cmd.py --sentence GGA:on --sentence GSV:on
  python3 tools/gnss-cocom/skytraq_cmd.py --rate 10
  python3 tools/gnss-cocom/skytraq_cmd.py --nmea --flash  # persist it

Notes:
  * This talks *through* firmware/gnss_passthrough, which forwards host bytes to
    the receiver verbatim. It needs a build whose TX pin is bound -- an RP2040
    or ESP32-S3 build, or one given an explicit -DGNSS_UART_TX_PIN. A C3 build
    on default pins is listen-only by design and cannot send anything.
  * ACK/NACK is reported per command. A timeout usually means TX is not wired
    to the receiver's RXD, not that the command was rejected.
"""

import argparse
import sys
import time

ACK, NACK = 0x83, 0x84
SENTENCES = ("GGA", "GSA", "GSV", "RMC", "VTG", "GLL", "ZDA")

# What the COCOM classifier needs: position (GGA/RMC) plus tracking (GSA/GSV).
# Without GSV a withheld fix is indistinguishable from a lost one.
COCOM_SENTENCES = ("GGA", "RMC", "GSA", "GSV")


def frame(payload: bytes) -> bytes:
    """A0 A1 | len | payload | XOR | 0D 0A"""
    cs = 0
    for b in payload:
        cs ^= b
    return b"\xa0\xa1" + len(payload).to_bytes(2, "big") + payload + bytes([cs, 0x0D, 0x0A])


def rtk_mode_payload(rover: bool, attr: int) -> bytes:
    """0x6A/0x01 Configure RTK Mode.  0 = rover, 1 = base."""
    return bytes([0x6A, 0x01, 0x00 if rover else 0x01, attr])


def rtcm_payload(enable: bool, attr: int) -> bytes:
    """0x20 Configure RTCM output, 16-byte body (TR_SkyTraqNMEA layout).

    Only byte 0 (master enable) and byte 15 (save target) matter for switching
    the stream off; the per-message enables are left at a sane 1005 + GPS/GLO/GAL
    MSM4 set so that re-enabling gives something useful rather than silence.
    """
    body = bytearray(16)
    body[0]  = 0x01 if enable else 0x00   # RTCM master enable
    body[1]  = 0x00                       # MSM rate code -> 1 Hz
    body[2]  = 0x01 if enable else 0x00   # 1005 ARP
    body[3]  = 0x01 if enable else 0x00   # GPS MSM  (1074)
    body[4]  = 0x01 if enable else 0x00   # GLO MSM  (1084)
    body[5]  = 0x01 if enable else 0x00   # GAL MSM  (1094)
    body[13] = 0x01                       # MSM type 1 -> MSM4
    body[14] = 0x02                       # version
    body[15] = attr
    return bytes([0x20]) + bytes(body)


def read_frames(ser, timeout=3.0):
    """Collect complete SkyTraq frames arriving within the window."""
    out, buf = [], bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
        while True:
            i = buf.find(b"\xa0\xa1")
            if i < 0 or len(buf) < i + 4:
                break
            n = int.from_bytes(buf[i + 2:i + 4], "big")
            end = i + 4 + n + 3
            if len(buf) < end:
                break
            out.append(bytes(buf[i + 4:i + 4 + n]))
            del buf[:end]
    return out


def send(ser, payload: bytes, label: str, expect_id=None, quiet=False):
    """Send one frame and report whether *this* command was acknowledged.

    The ACK body carries the id being acknowledged, and it must be checked: the
    receiver is streaming continuously, so an ACK for some earlier command is
    almost always in the buffer.  Accepting any ACK reports success for commands
    the receiver actually ignored -- which is indistinguishable from the setting
    silently not applying, and sends you looking in the wrong place entirely.
    """
    ser.reset_input_buffer()
    ser.write(frame(payload))
    ser.flush()
    got = read_frames(ser)
    want = payload[0]
    for f in got:
        if not f:
            continue
        if f[0] == ACK and len(f) > 1 and f[1] == want:
            if not quiet:
                print(f"  {label:28} ACK")
            return True
        if f[0] == NACK and len(f) > 1 and f[1] == want:
            print(f"  {label:28} NACK  (receiver rejected it)")
            return False
    # Report an unmatched ACK/NACK rather than hiding it -- it says the link is
    # alive and the receiver is answering, just not about this command.
    others = sorted({f[1] for f in got if f and f[0] in (ACK, NACK) and len(f) > 1})
    if others:
        ids = ", ".join(f"0x{o:02X}" for o in others)
        print(f"  {label:28} NO REPLY for 0x{want:02X}  (saw replies for {ids})")
        return False
    # A version reply (0x80) also counts as a response for --version.
    for f in got:
        if f and f[0] == 0x80:
            print(f"  {label:28} replied: {f[1:].decode('ascii', 'replace').strip()}")
            return True
    print(f"  {label:28} TIMEOUT  (no ACK seen)")
    return False


def main():
    ap = argparse.ArgumentParser(
        description="Configure a SkyTraq receiver through the passthrough (#491)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("-p", "--port", help="serial port (default: autodetect)")
    ap.add_argument("--nmea", action="store_true",
                    help="the usual fix: leave base mode, stop RTCM, output NMEA, "
                         "enable GGA/RMC/GSA/GSV")
    ap.add_argument("--rtk-mode", choices=("rover", "base"),
                    help="RTK role. A receiver left in 'base' streams RTCM corrections "
                         "and reports no position at all.")
    ap.add_argument("--rtcm", choices=("on", "off"), help="RTCM correction output")
    ap.add_argument("--msg-type", choices=("none", "nmea", "binary"),
                    help="set the receiver's output type")
    ap.add_argument("--sentence", action="append", metavar="XXX:on|off",
                    help="enable/disable one NMEA sentence (repeatable)")
    ap.add_argument("--rate", type=int, metavar="HZ", help="set position update rate")
    ap.add_argument("--version", action="store_true", help="query software version")
    ap.add_argument("--flash", action="store_true",
                    help="persist to flash (default: SRAM only, reverts on power cycle)")
    ap.add_argument("--factory-reset", action="store_true",
                    help="DESTRUCTIVE: restore defaults (0x64/0x1E) + cold restart "
                         "(0x04/0x02). Wipes the receiver's stored configuration, "
                         "including any RTK base survey position and RTCM setup. "
                         "Requires --yes-wipe-config.")
    ap.add_argument("--yes-wipe-config", action="store_true",
                    help="confirm --factory-reset; without it the reset is refused")
    args = ap.parse_args()

    if args.factory_reset and not args.yes_wipe_config:
        ap.error("--factory-reset discards the receiver's stored configuration "
                 "(RTK base survey position, RTCM settings). It is not undone by a "
                 "power cycle. Re-run with --yes-wipe-config if that is what you want.")

    if not any((args.nmea, args.msg_type, args.sentence, args.rate, args.version,
                args.rtk_mode, args.rtcm, args.factory_reset)):
        ap.error("nothing to do; --nmea is the usual one")

    try:
        import serial
    except ImportError:
        raise SystemExit("pyserial not installed:  python3 -m pip install pyserial")

    port = args.port
    if not port:
        sys.path.insert(0, __file__.rsplit("/", 1)[0])
        from gnss_nmea_monitor import autodetect_port
        port = autodetect_port()

    attr = 0x01 if args.flash else 0x00
    where = "SRAM + FLASH (persistent)" if args.flash else "SRAM only (reverts on power cycle)"
    print(f"# {port} -> {where}")

    try:
        ser = serial.Serial(port, 115200, timeout=0.2)
    except serial.SerialException as exc:
        raise SystemExit(f"could not open {port}: {exc}")

    with ser:
        time.sleep(0.3)
        ok = True
        if args.version:
            ok &= send(ser, bytes([0x02, 0x01]), "query version")

        if args.factory_reset:
            # Restore defaults then cold restart, per TR_SkyTraqNMEA's
            # resetToDefaultsAndCheck().  The restart drops the link briefly.
            ok &= send(ser, bytes([0x64, 0x1E]), "restore defaults")
            ok &= send(ser, bytes([0x04, 0x02]), "cold restart")
            print("  (receiver is restarting; give it a few seconds to reacquire)")

        if args.rtk_mode or args.nmea:
            rover = (args.rtk_mode or "rover") == "rover"
            ok &= send(ser, rtk_mode_payload(rover, attr),
                       f"RTK mode -> {'rover' if rover else 'base'}")

        if args.rtcm or args.nmea:
            on = (args.rtcm or "off") == "on"
            ok &= send(ser, rtcm_payload(on, attr), f"RTCM output -> {'on' if on else 'off'}")

        if args.msg_type or args.nmea:
            kind = args.msg_type or "nmea"
            code = {"none": 0, "nmea": 1, "binary": 2}[kind]
            ok &= send(ser, bytes([0x09, code, attr]), f"output type -> {kind}")

        wanted = list(args.sentence or [])
        if args.nmea:
            wanted += [f"{s}:on" for s in COCOM_SENTENCES]

        for spec in wanted:
            name, _, state = spec.partition(":")
            name = name.upper().strip()
            on = state.strip().lower() not in ("off", "0", "false")
            if len(name) != 3:
                print(f"  skipping {spec!r}: need a 3-letter sentence name")
                ok = False
                continue
            payload = bytes([0x64, 0x3B]) + name.encode("ascii") + bytes([1 if on else 0, attr])
            ok &= send(ser, payload, f"{name} {'on' if on else 'off'}")

        if args.rate:
            ok &= send(ser, bytes([0x0E, args.rate, attr]), f"update rate -> {args.rate} Hz")

    print("\n" + ("all commands acknowledged" if ok else "some commands failed -- see above"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
