#!/usr/bin/env python3
"""Turn on (and log) a SkyTraq receiver's raw measurements through the bridge.

For the raw-measurement collect: queries the receiver, switches its binary
measurement output on -- 0xE5 extended raw measurements (pseudorange, carrier,
Doppler, C/N0 per channel), 0xE0 GPS subframes (the ephemeris the host needs to
use them), 0xDF the receiver's own fix for comparison -- and captures the stream
in the rig's replay format ('<t> B <hex>' per frame, '<t> <text>' for NMEA), so
it replays through gnss_nmea_monitor.replay_source and
tinkerrocket-sim/scripts/tc_ekf_capture.py.

Configuration goes to SRAM only unless --flash: a power cycle undoes it.

RTK receivers (PX1105R, PX1125R) are different, and this is what the first
PX1105R collect (2026-09-26) ran into: AN0037 marks 0x09 "configure message
type" as NOT supported in RTK receivers -- binary NACKs, NMEA works -- and says
raw measurement output is what the receiver sends in RTK BASE mode. In rover
mode with 0x1E fully enabled it emits NMEA and no binary at all. So:

    --kinematic-base   0x6A/0x06 RTK base, KINEMATIC function (the base
                       re-solves every epoch, so it may move), SRAM only.
                       NMEA stops; 0xDF/0xE7/0xE8 + 0xE5 + subframes flow.
                       A power cycle returns it to its stored mode.

    python3 skytraq_raw.py -p /dev/cu.usbmodem2101 --query
    python3 skytraq_raw.py -p /dev/cu.usbmodem2101 --baud 921600
    python3 skytraq_raw.py -p /dev/cu.usbmodem2101 --kinematic-base --enable-bin --rate 20 \
        --log px1105r_20hz.log --seconds 1800

The passthrough must score SkyTraq binary frames as a live link (it does since
2026-09-26); the older NMEA-only bridge re-probed and dropped ~6 s in 10.

Frames per SkyTraq AN0039 v1.4.x (0x1E configure binary measurement output,
0x1F query it, 0x89 its reply, 0x02/0x80 software version, 0x09 message type,
0x83/0x84 ACK/NACK).
"""
from __future__ import annotations

import argparse
import sys
import time

import serial

RATES = {1: 0, 2: 1, 4: 2, 5: 3, 10: 4, 20: 5, 8: 6}


def frame(payload: bytes) -> bytes:
    cs = 0
    for b in payload:
        cs ^= b
    return b"\xa0\xa1" + len(payload).to_bytes(2, "big") + payload + bytes([cs]) + b"\r\n"


class Link:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.05)
        self.buf = bytearray()
        self.t0 = time.time()

    def close(self):
        self.s.close()

    def frames(self):
        """Pull complete, checksum-valid SkyTraq frames out of the buffer.
        Yields (kind, bytes): ('B', payload) or ('T', text line)."""
        self.buf += self.s.read(8192)
        out = []
        while True:
            i = self.buf.find(b"\xa0\xa1")
            j = self.buf.find(b"\n")
            if i < 0 and j < 0:
                break
            if i >= 0 and (j < 0 or i < j):
                if i > 0:                      # text before a frame
                    txt = bytes(self.buf[:i]).strip()
                    if txt:
                        out.append(("T", txt))
                    del self.buf[:i]
                    continue
                if len(self.buf) < 4:
                    break
                n = (self.buf[2] << 8) | self.buf[3]
                if len(self.buf) < 7 + n:
                    break
                pl = bytes(self.buf[4:4 + n])
                cs = 0
                for b in pl:
                    cs ^= b
                if cs == self.buf[4 + n] and self.buf[5 + n:7 + n] == b"\r\n":
                    out.append(("B", pl))
                    del self.buf[:7 + n]
                else:
                    del self.buf[:2]
            else:
                line = bytes(self.buf[:j]).strip()
                del self.buf[:j + 1]
                if line:
                    out.append(("T", line))
        return out

    def command(self, payload: bytes, want_reply: int | None = None, timeout=2.0):
        """Send one frame; wait for the ACK/NACK that names it (strict id
        match -- the receiver may be streaming) and an optional reply id."""
        self.s.write(frame(payload))
        ack, reply = None, None
        deadline = time.time() + timeout
        while time.time() < deadline and (ack is None or (want_reply and reply is None)):
            for k, f in self.frames():
                if k != "B" or not f:
                    continue
                if f[0] in (0x83, 0x84) and len(f) >= 2 and f[1] == payload[0]:
                    ack = f[0] == 0x83
                elif want_reply is not None and f[0] == want_reply:
                    reply = f
        return ack, reply


def query(link):
    ack, r = link.command(bytes([0x02, 0x01]), want_reply=0x80)
    if r:
        kv = ".".join(str(b) for b in r[3:6]); odm = ".".join(str(b) for b in r[7:10])
        rev = "%02d%02d%02d" % (r[11], r[12], r[13]) if len(r) >= 14 else "?"
        print(f"software: kernel {kv}  ODM {odm}  revision {rev}")
    else:
        print(f"software version: ack={ack}, no 0x80 reply")
    ack, r = link.command(bytes([0x1F]), want_reply=0x89)
    if r:
        names = ["rate", "meas_time(DC)", "raw_meas(DD)", "sv_ch_status(DE/E7/E8)", "rcv_state(DF)",
                 "subframe bits", "ext_raw(E5)"]
        rate = {v: k for k, v in RATES.items()}.get(r[1], r[1])
        vals = [f"{rate} Hz"] + [str(x) for x in r[2:8]]
        print("binary measurement output: " + ", ".join(f"{n}={v}" for n, v in zip(names, vals)))
    else:
        print(f"binary measurement output status: ack={ack}, no 0x89 reply")
    ack, r = link.command(bytes([0x10]), want_reply=0x86)
    if r:
        print(f"position update rate: {r[1]} Hz")


def enable(link, rate_hz, flash, subframes):
    attr = 1 if flash else 0
    steps = [
        ("message type -> binary", bytes([0x09, 0x02, attr])),
        (f"binary measurements {rate_hz} Hz: E5 raw, subframes 0x{subframes:02X}, DF fix, E7/E8 status",
         bytes([0x1E, RATES[rate_hz], 0x01, 0x00, 0x01, 0x01, subframes, 0x01, attr])),
    ]
    ok = True
    for what, pl in steps:
        ack, _ = link.command(pl)
        print(f"  {what:70s} {'ACK' if ack else 'NACK' if ack is False else 'no reply'}")
        ok &= bool(ack)
    return ok


def kinematic_base(link, flash):
    """RTK base mode, kinematic operational function (AN0037 0x6A/0x06, 37 B)."""
    import struct
    pl = (bytes([0x6A, 0x06, 0x01, 0x00]) + struct.pack(">II", 2000, 30)
          + struct.pack(">dd", 0.0, 0.0) + struct.pack(">ff", 0.0, 0.0) + bytes([1 if flash else 0]))
    ack, _ = link.command(pl, timeout=2.5)
    print(f"  RTK base mode, kinematic ({'flash' if flash else 'SRAM'}): "
          f"{'ACK' if ack else 'NACK' if ack is False else 'no reply'}")
    return bool(ack)


def enable_bin(link, rate_hz, flash, subframes):
    """0x1E only (the message type is not configurable on RTK receivers)."""
    attr = 1 if flash else 0
    ack, _ = link.command(bytes([0x1E, RATES[rate_hz], 0x00, 0x00, 0x01, 0x01, subframes, 0x01, attr]))
    print(f"  binary measurements {rate_hz} Hz (E5, subframes 0x{subframes:02X}, DF, E7/E8): "
          f"{'ACK' if ack else 'NACK' if ack is False else 'no reply'}")
    if not ack:
        return False
    # Raw output only runs at the requested rate if the position update rate
    # matches: with 0x1E at 20 Hz and 0x0E left at 1 Hz, 0xE5 came at 0.4 Hz.
    # The 0x0E ACK is easy to miss while the receiver restarts, so read it back.
    link.command(bytes([0x0E, rate_hz, attr]), timeout=2.5)
    _, r = link.command(bytes([0x10]), want_reply=0x86, timeout=3.0)
    got = r[1] if r else None
    print(f"  position update rate {rate_hz} Hz: reads back {got}")
    return got == rate_hz


BAUDS = {4800: 0, 9600: 1, 19200: 2, 38400: 3, 57600: 4, 115200: 5, 230400: 6, 460800: 7, 921600: 8}


def set_baud(link, baud, flash, wait_s=40.0):
    """0x05: move the receiver's UART to ``baud`` (SRAM unless --flash), then
    wait for the passthrough to notice the silence, re-probe and re-lock.
    Raw measurements at 20 Hz from ~36 channels are ~31 kB/s: 460800 is the
    minimum, 921600 leaves headroom (measured 2026-09-26)."""
    ack, _ = link.command(bytes([0x05, 0x00, BAUDS[baud], 1 if flash else 0]))
    print(f"  UART -> {baud} ({'flash' if flash else 'SRAM'}): {'ACK' if ack else 'NACK' if ack is False else 'no reply'}")
    if not ack:
        return False
    deadline = time.time() + wait_s
    while time.time() < deadline:
        for k, x in link.frames():
            if k == "T" and x.startswith(b"# re-locked"):
                print("  " + x.decode("ascii", "replace"))
                return str(baud).encode() in x
    print("  the bridge did not re-lock in time")
    return False


def log(link, path, seconds):
    ids, n_text, t_end = {}, 0, time.time() + seconds
    with open(path, "w") as f:
        while time.time() < t_end:
            for k, x in link.frames():
                t = time.time() - link.t0
                if k == "B":
                    f.write(f"{t:.3f} B {x.hex()}\n")
                    ids[x[0]] = ids.get(x[0], 0) + 1
                else:
                    s = x.decode("ascii", "replace")
                    if s.startswith("$"):
                        f.write(f"{t:.3f} {s}\n")
                    n_text += 1
    print(f"logged {seconds:.0f} s to {path}: " + ", ".join(f"0x{k:02X} x{v}" for k, v in sorted(ids.items()))
          + f"; {n_text} text lines")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", required=True)
    ap.add_argument("--query", action="store_true")
    ap.add_argument("--enable", action="store_true",
                    help="non-RTK receivers: message type binary + 0x1E (SRAM unless --flash)")
    ap.add_argument("--kinematic-base", dest="kbase", action="store_true",
                    help="RTK receivers: switch to RTK base mode, kinematic (needed for raw output)")
    ap.add_argument("--enable-bin", dest="enable_bin", action="store_true",
                    help="RTK receivers: 0x1E only (E5 raw, subframes, DF, E7/E8)")
    ap.add_argument("--rate", type=int, default=1, choices=sorted(RATES))
    ap.add_argument("--baud", type=int, choices=sorted(BAUDS),
                    help="move the receiver UART to this rate first (the bridge re-locks by itself)")
    ap.add_argument("--subframes", type=lambda x: int(x, 0), default=0x3D,
                    help="subframe bits: 1 GPS, 2 GLONASS, 4 Galileo, 8 BeiDou, 0x10 SBAS, "
                         "0x20 NavIC (default 0x3D, the PX1105R's own setting)")
    ap.add_argument("--flash", action="store_true")
    ap.add_argument("--log", help="capture the stream here")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()
    link = Link(args.port)
    try:
        time.sleep(0.3)
        link.frames()
        if args.query:
            query(link)
        if args.baud and not set_baud(link, args.baud, args.flash):
            sys.exit("baud change failed")
        if args.kbase and not kinematic_base(link, args.flash):
            sys.exit("receiver refused RTK base mode")
        if args.enable_bin and not enable_bin(link, args.rate, args.flash, args.subframes):
            sys.exit("receiver refused the binary measurement configuration")
        if args.enable:
            if not enable(link, args.rate, args.flash, args.subframes):
                sys.exit("receiver refused part of the configuration")
        if args.log:
            log(link, args.log, args.seconds)
    finally:
        link.close()


if __name__ == "__main__":
    main()
