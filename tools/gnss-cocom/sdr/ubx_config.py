#!/usr/bin/env python3
"""Identify and configure a u-blox F9/M9 receiver over its own USB port.

Written for the ArduSimple ZED-F9P, which unlike the rocket computer's SAM-M10Q
is ours to configure: it sits on its own USB CDC (1546:01A9) with nothing else
driving it. Out of the box it emits NMEA only, and NMEA cannot carry this
measurement -- GGA has no flag distinguishing "withholding a solution" from "no
solution", and its speed is *ground* speed, which is near zero through the exact
part of a rocket flight the velocity limit is about. So UBX-NAV-PVT and
UBX-NAV-SAT have to be turned on before the receiver can be tested at all.

**Everything is written to the RAM layer only.** A power cycle restores whatever
the owner had configured. That is deliberate: this is someone's surveying
receiver, and a bench test has no business permanently rewriting it. It also
means the configuration must be re-applied after every power cycle, which
run_f9p.py does on its own.

F9 uses the configuration interface (CFG-VALSET/VALGET) rather than the
deprecated per-message CFG-MSG. Output protocols are switched with the three
CFG-USBOUTPROT keys rather than by disabling six individual NMEA sentences --
one key per protocol, and no list of message IDs to get wrong.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from ublox_binary import (iter_frames, _checksum,        # noqa: E402
                          CLS_NAV, MSG_NAV_PVT as MSG_NAV_PVT_L,
                          MSG_NAV_SAT as MSG_NAV_SAT_L)
sys.path.insert(0, str(Path(__file__).resolve().parent))
from find_ublox import find_ublox, UBLOX_VID              # noqa: E402


def UBLOX_VID_PORT(dev: str) -> bool:
    """True if this port IS the receiver's own USB CDC rather than a bridge."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return False
    return any(p.device == dev and p.vid == UBLOX_VID for p in list_ports.comports())

CLS_CFG, CLS_MON = 0x06, 0x0A
MSG_CFG_MSG = 0x01        # legacy per-message rate (M8 and earlier)
MSG_CFG_NAV5 = 0x24       # legacy dynamic model
MSG_CFG_PRT = 0x00        # legacy port / protocol masks
MSG_VALSET, MSG_VALGET = 0x8A, 0x8B
MSG_RST = 0x04
MSG_MON_VER = 0x04

# CFG-RST. Clearing all of battery-backed RAM is the u-blox equivalent of the
# PX1125R's seeded restart, and it is needed for the same reason: a receiver
# that keeps its last real position fights an injected signal from somewhere
# else. This unit came to the bench holding a fix in New Jersey and tracked six
# simulated satellites at the equator for 400 s with used_in_fix stuck at 0.
BBR_COLD = 0xFFFF
# 0x02 is a controlled software reset of the GNSS subsystem only. A hardware
# reset (0x00) also works but drops and re-enumerates the USB port, which loses
# the handle mid-run for no benefit.
RESET_GNSS_ONLY = 0x02

LAYER_RAM, LAYER_BBR, LAYER_FLASH = 0x01, 0x02, 0x04

# Storage size is encoded in bits 30:28 of the key.
SIZE_OF = {0x1: 1, 0x2: 1, 0x3: 2, 0x4: 4, 0x5: 8}

# The configuration keys are PER PORT, and picking the wrong port's set does not
# fail politely: VALSET is all-or-nothing, so one inapplicable key NAKs the whole
# frame and nothing is written. A module on a USB-UART bridge has no USB port at
# all, and the USB keys that worked on the F9P's native USB were rejected
# wholesale by an M10 on a CP2102.
KEYS = {
    # --- native USB (the receiver enumerates as its own CDC) ---
    "CFG-USBOUTPROT-UBX":       0x10780001,
    "CFG-USBOUTPROT-NMEA":      0x10780002,
    "CFG-USBOUTPROT-RTCM3X":    0x10780004,
    "CFG-MSGOUT-UBX_NAV_PVT_USB": 0x20910009,
    "CFG-MSGOUT-UBX_NAV_SAT_USB": 0x20910018,
    # --- UART1 (the receiver is behind a USB-UART bridge) ---
    "CFG-UART1OUTPROT-UBX":     0x10740001,
    "CFG-UART1OUTPROT-NMEA":    0x10740002,
    "CFG-UART1OUTPROT-RTCM3X":  0x10740004,
    "CFG-MSGOUT-UBX_NAV_PVT_UART1": 0x20910007,
    "CFG-MSGOUT-UBX_NAV_SAT_UART1": 0x20910016,
    # dynamics: 8 = airborne <4 g, matching the rocket computer's receiver
    "CFG-NAVSPG-DYNMODEL":      0x20110021,
    # navigation rate, ms between solutions
    "CFG-RATE-MEAS":            0x30210001,
    # 0 = normal navigation. THIS IS THE ONE THAT BITES: an ArduSimple ships
    # configured as an RTK base, and in fixed mode (2) or survey-in (1) the
    # receiver does not navigate at all -- it holds its surveyed position and
    # emits corrections. The symptoms look exactly like a signal problem and
    # are not: satellites track at 41 dBHz with valid ephemeris, 335 epochs out
    # of 420 have four or more of them, and used_in_fix sits at 0 the whole
    # time while the reported position stays nailed to the base coordinates.
    # No amount of cold-starting or gain-hunting touches it.
    "CFG-TMODE-MODE":           0x20030001,
}

TMODE_NAME = {0: "disabled (normal navigation)", 1: "survey-in base",
              2: "fixed-position base"}


def frame(cls: int, mid: int, payload: bytes) -> bytes:
    body = bytes([cls, mid]) + len(payload).to_bytes(2, "little") + payload
    a, b = _checksum(body)
    return b"\xb5\x62" + body + bytes([a, b])


def valset(items, layers=LAYER_RAM) -> bytes:
    """CFG-VALSET for {key: value}, sizes inferred from the key encoding."""
    p = struct.pack("<BBBB", 0, layers, 0, 0)
    for key, val in items:
        n = SIZE_OF[(key >> 28) & 0x7]
        p += struct.pack("<I", key)
        p += int(val).to_bytes(n, "little", signed=False)
    return frame(CLS_CFG, MSG_VALSET, p)


def valget(keys, layer=0) -> bytes:
    p = struct.pack("<BBBB", 0, layer, 0, 0)
    for k in keys:
        p += struct.pack("<I", k)
    return frame(CLS_CFG, MSG_VALGET, p)


# --- generation split -------------------------------------------------------
# CFG-VALSET/VALGET arrived with protocol 27 (F9, M9). An M8 -- including the
# NEO-M8T -- speaks protocol 15-20 and answers a VALSET with a NAK, or with
# nothing at all, which reads exactly like a wiring fault if you are not
# expecting it. The two generations therefore need different configuration
# paths, chosen from the PROTVER the receiver reports in MON-VER rather than
# from what the box says.
VALSET_MIN_PROTVER = 27.0


def cfg_msg(cls: int, mid: int, rate: int, port: int = 1) -> bytes:
    """Legacy CFG-MSG: set the output rate of one message on one port.

    Port 1 is UART1 on every u-blox that has one. The 6-byte form addresses a
    single port; the 8-byte form sets all six at once and is easy to get wrong.
    """
    p = bytearray(8)
    p[0], p[1] = cls, mid
    p[2 + port] = rate
    return frame(CLS_CFG, MSG_CFG_MSG, bytes(p))


def cfg_nav5_dynmodel(model: int) -> bytes:
    """Legacy CFG-NAV5, applying the dynamic model and nothing else."""
    p = bytearray(36)
    p[0:2] = (0x0001).to_bytes(2, "little")   # mask: dyn model only
    p[2] = model
    return frame(CLS_CFG, MSG_CFG_NAV5, bytes(p))


def protver_from_monver(payload: bytes):
    """PROTVER as a float from a MON-VER payload, or None."""
    for k in range(40, len(payload), 30):
        ext = payload[k:k + 30].split(b"\x00")[0].decode(errors="replace")
        if ext.startswith("PROTVER="):
            try:
                return float(ext.split("=", 1)[1])
            except ValueError:
                return None
    return None


def cold_start(ser, settle: float = 12.0) -> None:
    """Discard position, time, ephemeris and almanac; take it all from the signal.

    CFG-RST is deliberately not acknowledged by the receiver -- it resets
    instead of replying -- so there is nothing to wait for but the stream
    coming back. The caller must re-apply configuration afterwards and verify
    it, which is why this is not exposed as a standalone command.
    """
    ser.write(frame(CLS_CFG, MSG_RST,
                    struct.pack("<HBB", BBR_COLD, RESET_GNSS_ONLY, 0)))
    time.sleep(settle)
    ser.reset_input_buffer()


def read_for(ser, seconds: float) -> bytearray:
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds:
        buf.extend(ser.read(ser.in_waiting or 1))
    return buf


def collect(buf: bytearray):
    """(cls, mid, payload) list, leaving buf consumed."""
    return list(iter_frames(buf))


def parse_valget(payload: bytes):
    """CFG-VALGET response -> {key: value}."""
    out = {}
    i = 4
    while i + 4 <= len(payload):
        key = struct.unpack_from("<I", payload, i)[0]
        n = SIZE_OF.get((key >> 28) & 0x7)
        if n is None or i + 4 + n > len(payload):
            break
        out[key] = int.from_bytes(payload[i + 4:i + 4 + n], "little")
        i += 4 + n
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="auto",
                    help="serial port, or 'auto' to resolve by u-blox vendor id")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="ignored on a native-USB receiver; M8 UARTs vary")
    ap.add_argument("--rate-hz", type=float, default=1.0)
    ap.add_argument("--dynmodel", type=int, default=8,
                    help="8 = airborne <4 g (default), 7 = <2 g, 6 = <1 g")
    ap.add_argument("--iface", choices=("auto", "usb", "uart1"), default="auto",
                    help="which port's config keys to write. 'auto' picks USB "
                         "for a receiver on its own CDC and UART1 for one behind "
                         "a serial bridge")
    ap.add_argument("--keep-nmea", action="store_true",
                    help="leave NMEA on; the capture then carries both protocols")
    ap.add_argument("--identify-only", action="store_true")
    ap.add_argument("--cold-start", action="store_true",
                    help="clear position/time/ephemeris/almanac before configuring")
    ap.add_argument("--persist", action="store_true",
                    help="also write BBR+Flash, so the settings survive a power "
                         "cycle (default is RAM only, which does not)")
    args = ap.parse_args()

    import serial
    port = find_ublox(args.port)
    if port is None:
        return "no u-blox on the bus (run ./find_ublox.py to see what is)"
    print(f"  port     : {port}")
    with serial.Serial(port, args.baud, timeout=0.2) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()

        # --- identify -------------------------------------------------------
        ser.write(frame(CLS_MON, MSG_MON_VER, b""))
        buf = read_for(ser, 2.0)
        protver = None
        for cls, mid, pl in collect(buf):
            if cls == CLS_MON and mid == MSG_MON_VER:
                protver = protver_from_monver(pl)
                sw = pl[0:30].split(b"\x00")[0].decode(errors="replace")
                hw = pl[30:40].split(b"\x00")[0].decode(errors="replace")
                print(f"  software : {sw}")
                print(f"  hardware : {hw}")
                for k in range(40, len(pl), 30):
                    ext = pl[k:k + 30].split(b"\x00")[0].decode(errors="replace")
                    if ext:
                        print(f"  ext      : {ext}")
                break
        else:
            print("  !! no MON-VER response -- is this a u-blox on the right port?")
            if not args.keep_nmea:
                print("     (a receiver emitting NMEA only still answers MON-VER;\n"
                      "      no answer usually means the wrong device or port)")
            return 1
        legacy = protver is not None and protver < VALSET_MIN_PROTVER
        if protver is not None:
            print(f"  config   : {'legacy CFG-MSG (M8 or older)' if legacy else 'CFG-VALSET'}"
                  f"  [PROTVER {protver}]")
        else:
            print("  config   : PROTVER unknown; assuming CFG-VALSET")
        if args.identify_only:
            return 0

        # --- M8 and older: legacy messages, no configuration database --------
        if legacy:
            if args.cold_start:
                print("  cold start: clearing BBR")
                cold_start(ser)
            if args.persist:
                print("  note: --persist has no effect on a legacy part here; "
                      "CFG-CFG\n     would be needed to save, and is not sent.")
            ser.reset_input_buffer()
            for cls, mid, name in ((CLS_NAV, MSG_NAV_PVT_L, "NAV-PVT"),
                                   (CLS_NAV, MSG_NAV_SAT_L, "NAV-SAT")):
                ser.write(cfg_msg(cls, mid, 1))
                time.sleep(0.25)
            ser.write(cfg_nav5_dynmodel(args.dynmodel))
            time.sleep(0.4)
            if not args.keep_nmea:
                # Silence the standard NMEA set message by message. CFG-PRT
                # would do it with one frame by clearing the output protocol
                # mask, but that frame also carries the baud rate, and getting
                # it wrong takes the receiver off the wire mid-session.
                for mid in range(0x00, 0x06):    # GGA GLL GSA GSV RMC VTG
                    ser.write(cfg_msg(0xF0, mid, 0))
                    time.sleep(0.12)
            acks = [(c, m) for c, m, _ in collect(read_for(ser, 1.5)) if c == 0x05]
            n_ack = sum(1 for c, m in acks if m == 0x01)
            n_nak = sum(1 for c, m in acks if m == 0x00)
            print(f"  CFG-MSG/NAV5: {n_ack} ACK, {n_nak} NAK")
            ser.reset_input_buffer()
            raw = bytes(read_for(ser, 3.0))
            n_ubx = len(collect(bytearray(raw)))
            print(f"  stream   : {len(raw)} B in 3 s -- {n_ubx} UBX frames, "
                  f"{raw.count(b'$G')} NMEA sentences")
            if n_ubx == 0:
                print("  !! no UBX on the wire; NAV-PVT/NAV-SAT did not take")
                return 1
            # NMEA is left on deliberately: an M8T carries both without trouble
            # at 9600, and the shared demuxer reads either.
            return 0

        # --- what mode did we find it in? -----------------------------------
        ser.reset_input_buffer()
        ser.write(valget([KEYS["CFG-TMODE-MODE"]]))
        buf = read_for(ser, 1.2)
        for cls, mid, pl in collect(buf):
            if cls == CLS_CFG and mid == MSG_VALGET:
                mode = parse_valget(pl).get(KEYS["CFG-TMODE-MODE"])
                if mode:
                    print(f"  !! found in {TMODE_NAME.get(mode, mode)} -- it was not "
                          f"navigating.\n     Switching to normal navigation (RAM "
                          f"only; a power cycle restores the base setup).")
                break

        # --- optionally discard retained state ------------------------------
        if args.cold_start:
            print("  cold start: clearing BBR (position, time, ephemeris, almanac)")
            cold_start(ser)

        # --- configure, RAM only --------------------------------------------
        # Native-USB receivers enumerate under u-blox's own vendor id; anything
        # reached through a bridge (CP2102, FTDI, CH340) is on UART1.
        iface = args.iface
        if iface == "auto":
            iface = "usb" if UBLOX_VID_PORT(port) else "uart1"
        P = "USB" if iface == "usb" else "UART1"
        OUT = "CFG-USBOUTPROT" if iface == "usb" else "CFG-UART1OUTPROT"
        print(f"  iface    : {P}"
              f"{'  (auto-detected)' if args.iface == 'auto' else ''}")
        items = [
            (KEYS[f"{OUT}-UBX"], 1),
            (KEYS[f"CFG-MSGOUT-UBX_NAV_PVT_{P}"], 1),
            (KEYS[f"CFG-MSGOUT-UBX_NAV_SAT_{P}"], 1),
            (KEYS["CFG-NAVSPG-DYNMODEL"], args.dynmodel),
            (KEYS["CFG-RATE-MEAS"], int(round(1000.0 / args.rate_hz))),
        ]
        if not args.keep_nmea:
            items.append((KEYS[f"{OUT}-NMEA"], 0))

        # Defensive settings that only exist on RTK-capable parts. A plain
        # navigation M10 has neither an RTCM3 output nor a time mode, and NAKs
        # them -- which, because VALSET is all-or-nothing, took the whole frame
        # down with it. Applied separately so an absent feature costs nothing.
        optional = [
            (KEYS[f"{OUT}-RTCM3X"], 0, "RTCM3 output off"),
            (KEYS["CFG-TMODE-MODE"], 0, "base/time mode off"),
        ]

        layers = LAYER_RAM | (LAYER_BBR | LAYER_FLASH if args.persist else 0)
        if args.persist:
            print("  persisting to BBR+Flash: these settings will survive a power "
                  "cycle\n     (the unit's original base-station setup is being "
                  "overwritten, not shadowed)")
        for key, val, what in optional:
            ser.reset_input_buffer()
            ser.write(valset([(key, val)], layers))
            r = [m for c, m, _ in collect(read_for(ser, 0.8)) if c == 0x05]
            if 0x00 in r:
                print(f"  skipped  : {what} -- this receiver has no such feature")

        ser.reset_input_buffer()
        ser.write(valset(items, layers))
        buf = read_for(ser, 1.5)
        acks = [(c, m, p) for c, m, p in collect(buf) if c == 0x05]
        if any(m == 0x01 for _, m, _ in acks):
            where = "RAM+BBR+Flash" if args.persist else "RAM layer only"
            print(f"  VALSET   : ACK ({len(items)} keys, {where})")
        elif any(m == 0x00 for _, m, _ in acks):
            print("  VALSET   : NAK -- the receiver rejected the configuration")
            # VALSET is all-or-nothing, so a NAK says nothing about WHICH key was
            # wrong. Retry them one at a time; the ones that NAK alone are the
            # ones this receiver does not have.
            names = {v: k for k, v in KEYS.items()}
            bad = []
            for key, val in items:
                ser.reset_input_buffer()
                ser.write(valset([(key, val)], LAYER_RAM))
                r = [m for c, m, _ in collect(read_for(ser, 0.8)) if c == 0x05]
                if 0x00 in r:
                    bad.append(names.get(key, hex(key)))
            if bad:
                print("  rejected individually:")
                for b in bad:
                    print(f"    {b}")
                print("  A key the receiver does not implement -- usually the "
                      "wrong port's\n     key set. Try --iface uart1 or --iface usb.")
            return 1
        else:
            print("  VALSET   : no ACK seen (continuing; verification below is "
                  "what counts)")

        # --- verify by reading it back --------------------------------------
        ser.reset_input_buffer()
        ser.write(valget([k for k, _ in items]))
        buf = read_for(ser, 1.5)
        got = {}
        for cls, mid, pl in collect(buf):
            if cls == CLS_CFG and mid == MSG_VALGET:
                got.update(parse_valget(pl))
        names = {v: k for k, v in KEYS.items()}
        bad = 0
        for key, want in items:
            have = got.get(key)
            ok = have == want
            bad += not ok
            print(f"  {'ok ' if ok else '!! '}{names.get(key, hex(key)):<30} "
                  f"want {want:<6} got {have if have is not None else '--'}")
        if bad:
            print(f"  !! {bad} key(s) did not read back; do not trust a run until "
                  "this is clean")
            return 1

        # --- confirm the stream actually changed ----------------------------
        ser.reset_input_buffer()
        buf = read_for(ser, 3.0)
        raw = bytes(buf)
        n_ubx = len(collect(bytearray(raw)))
        n_nmea = raw.count(b"$G")
        print(f"  stream   : {len(raw)} B in 3 s -- {n_ubx} UBX frames, "
              f"{n_nmea} NMEA sentences")
        if n_ubx == 0:
            print("  !! still no UBX on the wire")
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
