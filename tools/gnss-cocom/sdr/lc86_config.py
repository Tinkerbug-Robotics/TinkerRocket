#!/usr/bin/env python3
"""Configure and verify the Tinker-Beetle's Quectel LC86G through lc86_bridge.

The Beetle (rocket-computer-mini, M1) keeps its GNSS on the flight computer's
UART, so the rig reaches it through the bench image in
../firmware/lc86_bridge, which copies bytes between that UART and USB.

The default action applies EXACTLY what the flight driver applies at boot
(TR_GNSSReceiverLC86_Serial::begin() as of PR #1500, at the M1's
GNSS_UPDATE_RATE of 18 clamped to the LC86G's 10 Hz ceiling), then reads every
setting back. That is the receiver as it flies. Nothing is saved to the
module's flash: this is RAM configuration, as in flight, and it lasts until the
flight computer's rail cycles -- which is also when the Beetle's own firmware
would re-apply it.

    ./lc86_config.py --identify          # read only: version, mode, rates
    ./lc86_config.py                     # the flight configuration, Balloon mode
    ./lc86_config.py --navmode 0         # ... in Normal mode instead

**The navigation mode is the lever that matters.** Until PR #1500 the flight
driver never sent $PAIR080, so the Beetle flew in the module's default, Normal
mode, and that is how the first flights here were made (2026-09-24). PR #1500
makes begin() send $PAIR080,3 straight after the fix rate, so the flight
configuration is now Balloon; --navmode 0 puts the module back in Normal to
repeat the earlier measurements.
Quectel's protocol spec (LC26G/LC76G/LC86G GNSS Protocol Specification V1.4,
section 2.4.24, Tables 7 and 8) gives every mode but Balloon a 10 km altitude
limitation, calls 10-50 km "cannot be guaranteed", and stops ALL output above
50 km; Balloon mode moves the limit to 80 km. That is a dynamic-model ceiling
of the NEO-M8T kind, not an export gate, and the way to show it is the M8T's
way: move the mode and watch whether the ceiling moves.

Acks are matched strictly by command id (a $PAIR001 for a DIFFERENT command
proves nothing -- the module streams continuously and acks arrive among the
sentences), and result 1 ("being processed") is waited through, not accepted.
"""

from __future__ import annotations

import argparse
import re
import sys
import time

import serial
from serial.tools import list_ports

# The Beetle's flight computer, by USB serial = MAC. The OUT computer on the
# same USB-C (S1 = O) is 9C:13:9E:28:9E:88 and does not carry the receiver.
BEETLE_FC_MAC = "9C:13:9E:28:9E:8C"
BEETLE_OC_MAC = "9C:13:9E:28:9E:88"
ESPRESSIF_VID = 0x303A

# The flight driver's fix rate on this board: config.h asks for 18 Hz and the
# driver clamps to the LC86G's 10 Hz maximum. GSV goes out every N fixes with
# N = the fix rate, i.e. about once a second.
FLIGHT_RATE_HZ = 10
# The $PAIR080 navigation mode begin() sends from PR #1500 on: 3 = Balloon.
# Before it nothing set the mode, and the Beetle flew in Normal (0).
FLIGHT_NAV_MODE = 3

NAV_MODES = {0: "Normal", 1: "Fitness", 3: "Balloon", 4: "Stationary",
             5: "Drone", 7: "Swimming"}
# Quectel's Table 7. Every mode but Balloon is limited to 10 km.
NAV_ALT_LIMIT_M = {0: 10000, 1: 10000, 3: 80000, 4: 10000, 5: 10000, 7: 10000}

NMEA_TYPES = {0: "GGA", 1: "GLL", 2: "GSA", 3: "GSV", 4: "RMC", 5: "VTG",
              6: "ZDA", 7: "GRS", 8: "GST", 9: "GNS"}

# $PAIR432 <Mode>. MSM7 is the one worth having: per-satellite C/N0 to 1/16 dB,
# the receiver's own Doppler (phase-range rate) and lock-time counters. It comes
# once a second whatever the fix rate (checked at 1, 5 and 10 Hz, 2026-09-26), and
# its Doppler has the receiver's own clock drift already taken out. It changes
# what the module OUTPUTS, not how it navigates.
RTCM_MODES = {"off": -1, "msm4": 0, "msm7": 1}


def flight_plan(rate_hz: int = FLIGHT_RATE_HZ):
    """(body, kind, what) in the flight driver's order. kind is the PAIR
    command id whose $PAIR001 acks it, or 'QTM' for a $PQTMCFGMSGRATE."""
    return [
        ("PAIR062,1,0", 62, "GLL off"),
        ("PAIR062,2,0", 62, "GSA off"),
        ("PAIR062,5,0", 62, "VTG off"),
        ("PAIR062,0,1", 62, "GGA every fix"),
        ("PAIR062,4,0", 62, "RMC off"),
        (f"PAIR062,3,{rate_hz}", 62, f"GSV every {rate_hz} fixes"),
        ("PQTMCFGMSGRATE,W,PQTMPVT,1,1", "QTM", "PQTMPVT every fix"),
        ("PQTMCFGMSGRATE,W,PQTMEPE,1,2", "QTM", "PQTMEPE every fix"),
        (f"PAIR050,{1000 // rate_hz}", 50, f"fix interval {1000 // rate_hz} ms"),
        (f"PAIR080,{FLIGHT_NAV_MODE}", 80, f"nav mode {NAV_MODES[FLIGHT_NAV_MODE]}"),
    ]


def expected_rates(rate_hz: int = FLIGHT_RATE_HZ) -> dict:
    """NMEA type -> output rate the flight configuration leaves behind."""
    return {0: 1, 1: 0, 2: 0, 3: rate_hz, 4: 0, 5: 0}


# --------------------------------------------------------------------------
# Port
# --------------------------------------------------------------------------

def find_bridge(port: str | None) -> str | None:
    """The Beetle FC's port, identified by MAC. Refuses the OC and strangers.

    Every native-USB board on this Mac enumerates as usbmodemNNNN with the same
    Espressif ids, and S1 decides which of the Beetle's two processors is on
    the cable. The MAC is the only thing that says which one answered.
    """
    ports = list(list_ports.comports())
    if port and port != "auto":
        for p in ports:
            if p.device == port and (p.serial_number or "").upper() == BEETLE_OC_MAC:
                print(f"!! {port} is the Beetle's OUT computer ({BEETLE_OC_MAC}); "
                      f"slide S1 to F for the flight computer")
                return None
        return port
    for p in ports:
        if p.vid == ESPRESSIF_VID and (p.serial_number or "").upper() == BEETLE_FC_MAC:
            return p.device
    for p in ports:
        if (p.serial_number or "").upper() == BEETLE_OC_MAC:
            print(f"!! only the Beetle's OUT computer is on USB ({p.device}); "
                  f"slide S1 to F")
    return None


def open_bridge(port: str) -> serial.Serial:
    """Open WITHOUT resetting the flight computer.

    On USB-Serial-JTAG, (DTR 0, RTS 1) is the chip-reset pattern, and pyserial
    passes through it when it drops DTR after macOS has raised both on open.
    Opening with both already high leaves the running bridge alone. A reset
    would not touch the receiver (it is on the rail, not on this chip's EN)
    but it drops a second of stream.
    """
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200      # irrelevant over USB CDC; the bridge fixes 115200
    ser.timeout = 0.05
    ser.dtr = True
    ser.rts = True
    ser.open()
    return ser


# --------------------------------------------------------------------------
# Sentences
# --------------------------------------------------------------------------

def checksum(body: str) -> int:
    c = 0
    for ch in body:
        c ^= ord(ch)
    return c


def frame(body: str) -> bytes:
    return f"${body}*{checksum(body):02X}\r\n".encode("ascii")


def valid(line: str) -> str | None:
    """Body of a checksum-valid sentence, else None."""
    m = re.match(r"^\$([^*$]+)\*([0-9A-Fa-f]{2})$", line.strip())
    if not m or checksum(m.group(1)) != int(m.group(2), 16):
        return None
    return m.group(1)


class Link:
    def __init__(self, ser: serial.Serial, verbose: bool = False):
        self.ser = ser
        self.buf = b""
        self.verbose = verbose

    def lines(self, seconds: float):
        """Checksum-valid sentence bodies arriving within `seconds`."""
        end = time.time() + seconds
        while time.time() < end:
            self.buf += self.ser.read(4096)
            while b"\n" in self.buf:
                raw, self.buf = self.buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if line.startswith("#"):
                    if self.verbose:
                        print(f"   {line}")
                    continue
                body = valid(line)
                if body:
                    yield body

    def send(self, body: str):
        if self.verbose:
            print(f"   -> ${body}")
        self.ser.write(frame(body))

    def pair(self, body: str, cmd_id: int, timeout=1.5, reply: str | None = None,
             retries: int = 2):
        """Send a PAIR command; return (result, reply_fields or None).

        Result 0 is success; None means no matching ack at all. `reply` names
        the query-result sentence (e.g. 'PAIR081') to collect after the ack.
        """
        for _ in range(retries + 1):
            self.send(body)
            result, got = None, None
            for s in self.lines(timeout):
                f = s.split(",")
                if f[0] == "PAIR001" and len(f) >= 3 and _int(f[1]) == cmd_id:
                    r = _int(f[2])
                    if r == 1:          # being processed: wait for the verdict
                        continue
                    result = r
                elif reply and f[0] == reply:
                    got = f[1:]
                if result is not None and (reply is None or got is not None):
                    return result, got
            if result is not None:
                return result, got
        return None, None

    def qtm(self, body: str, timeout=1.5, retries: int = 2):
        """$PQTMCFGMSGRATE: ('OK', fields) / ('ERROR', code) / (None, None)."""
        for _ in range(retries + 1):
            self.send(body)
            for s in self.lines(timeout):
                f = s.split(",")
                if f[0] == "PQTMCFGMSGRATE" and len(f) >= 2 and f[1] in ("OK", "ERROR"):
                    return f[1], f[2:]
        return None, None

    def version(self, timeout=1.5):
        self.send("PQTMVERNO")
        for s in self.lines(timeout):
            f = s.split(",")
            if f[0] == "PQTMVERNO":
                return f[1:]
        return None


def _int(s):
    try:
        return int(s)
    except (TypeError, ValueError):
        return None


# --------------------------------------------------------------------------
# Read-back
# --------------------------------------------------------------------------

def read_state(link: Link) -> dict:
    st = {}
    st["version"] = link.version()
    r, f = link.pair("PAIR081", 81, reply="PAIR081")
    st["navmode"] = _int(f[0]) if r == 0 and f else None
    r, f = link.pair("PAIR051", 51, reply="PAIR051")
    st["fix_interval_ms"] = _int(f[0]) if r == 0 and f else None
    r, f = link.pair("PAIR067", 67, reply="PAIR067")
    st["search"] = [_int(x) for x in f[:5]] if r == 0 and f else None
    rates = {}
    for t in NMEA_TYPES:
        r, f = link.pair(f"PAIR063,{t}", 63, reply="PAIR063")
        if r == 0 and f and len(f) >= 2 and _int(f[0]) == t:
            rates[t] = _int(f[1])
        else:
            rates[t] = None
    st["nmea_rates"] = rates
    r, f = link.pair("PAIR433", 433, reply="PAIR433")
    st["rtcm"] = _int(f[0]) if r == 0 and f else None
    for name, ver in (("PQTMPVT", 1), ("PQTMEPE", 2)):
        ok, f = link.qtm(f"PQTMCFGMSGRATE,R,{name},{ver}")
        st[name] = _int(f[1]) if ok == "OK" and len(f) >= 2 else None
    return st


def show_state(st: dict):
    v = st.get("version")
    print(f"  firmware      {', '.join(v) if v else '(no reply)'}")
    nm = st.get("navmode")
    if nm is None:
        print("  nav mode      (no reply)")
    else:
        print(f"  nav mode      {nm} = {NAV_MODES.get(nm, '?')}"
              f"  (Quectel altitude limitation {NAV_ALT_LIMIT_M.get(nm, 0)/1000:.0f} km)")
    fi = st.get("fix_interval_ms")
    print(f"  fix interval  {fi} ms" + (f" ({1000/fi:.0f} Hz)" if fi else ""))
    s = st.get("search")
    if s:
        names = [n for n, on in zip(("GPS", "GLONASS", "Galileo", "BDS", "QZSS"), s) if on]
        print(f"  constellations {'+'.join(names)}")
    rates = st.get("nmea_rates") or {}
    on = [f"{NMEA_TYPES[t]}/{r}" for t, r in rates.items() if r]
    print(f"  NMEA          {' '.join(on) or '(none)'}   (type/every-N-fixes)")
    print(f"  PQTMPVT       {st.get('PQTMPVT')}   PQTMEPE {st.get('PQTMEPE')}")
    rt = {v: k for k, v in RTCM_MODES.items()}.get(st.get("rtcm"), st.get("rtcm"))
    print(f"  RTCM          {rt}")


def check_flight(st: dict, rate_hz: int, navmode: int | None) -> list[str]:
    """Everything that differs from the flight configuration (+ nav mode)."""
    bad = []
    if st.get("fix_interval_ms") != 1000 // rate_hz:
        bad.append(f"fix interval {st.get('fix_interval_ms')} ms, "
                   f"want {1000 // rate_hz}")
    for t, want in expected_rates(rate_hz).items():
        got = (st.get("nmea_rates") or {}).get(t)
        if got != want:
            bad.append(f"{NMEA_TYPES[t]} rate {got}, want {want}")
    if st.get("PQTMPVT") != 1:
        bad.append(f"PQTMPVT rate {st.get('PQTMPVT')}, want 1")
    want_nav = FLIGHT_NAV_MODE if navmode is None else navmode
    if st.get("navmode") != want_nav:
        bad.append(f"nav mode {st.get('navmode')}, want {want_nav} "
                   f"({NAV_MODES.get(want_nav)})")
    return bad


# --------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="auto",
                    help="bridge port, or 'auto' to find the Beetle FC by MAC")
    ap.add_argument("--identify", action="store_true",
                    help="read the module's state; change nothing")
    ap.add_argument("--navmode", type=int, choices=sorted(NAV_MODES),
                    help="after the flight configuration, set this navigation "
                         "mode ($PAIR080) instead of the flight's 3 (Balloon, as "
                         "flown from PR #1500). 0 = Normal repeats the "
                         "measurements made before it")
    ap.add_argument("--rate", type=int, default=FLIGHT_RATE_HZ,
                    help=f"fix rate in Hz (default {FLIGHT_RATE_HZ}, as flown)")
    ap.add_argument("--rtcm", choices=sorted(RTCM_MODES),
                    help="also set RTCM3 raw-measurement output ($PAIR432): msm7 "
                         "adds per-satellite Doppler, fine C/N0 and lock time. "
                         "Output only; the flight image never enables it")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    port = find_bridge(args.port)
    if not port:
        return "no Beetle flight computer on USB (want MAC " + BEETLE_FC_MAC + ")"

    with open_bridge(port) as ser:
        link = Link(ser, args.verbose)
        heard = sum(1 for _ in link.lines(1.5))
        print(f"# {port}: {heard} valid sentences in 1.5 s")
        if heard == 0:
            print("!! nothing from the LC86G. Is the flight computer's rail on "
                  "(cmd 8), and is the bridge image flashed?")
            return 1

        if not args.identify:
            print(f"# applying the flight configuration ({args.rate} Hz)")
            failed = []
            for body, kind, what in flight_plan(args.rate):
                if kind == "QTM":
                    ok, f = link.qtm(body)
                    if ok != "OK" and "PQTMEPE" in body:
                        ok, f = link.qtm(body.replace(",1,2", ",1,1"))
                    good = ok == "OK"
                else:
                    r, _ = link.pair(body, kind)
                    good = r == 0
                print(f"   {'ok ' if good else 'NO '} {what:<22} ${body}")
                if not good and "PQTMEPE" not in body:
                    failed.append(what)
            if args.navmode is not None:
                r, _ = link.pair(f"PAIR080,{args.navmode}", 80)
                print(f"   {'ok ' if r == 0 else 'NO '} "
                      f"{'nav mode ' + NAV_MODES[args.navmode]:<22} "
                      f"$PAIR080,{args.navmode}  (result {r})")
                if r != 0:
                    failed.append("nav mode")
            if args.rtcm:
                r, _ = link.pair(f"PAIR432,{RTCM_MODES[args.rtcm]}", 432)
                print(f"   {'ok ' if r == 0 else 'NO '} {'RTCM ' + args.rtcm:<22} "
                      f"$PAIR432,{RTCM_MODES[args.rtcm]}  (result {r})")
                if r != 0:
                    failed.append("RTCM")
            if failed:
                print(f"!! not applied: {', '.join(failed)}")

        print("# read-back")
        st = read_state(link)
        show_state(st)
        if args.identify:
            return 0
        bad = check_flight(st, args.rate, args.navmode)
        if args.rtcm and st.get("rtcm") != RTCM_MODES[args.rtcm]:
            bad.append(f"RTCM mode {st.get('rtcm')}, want {RTCM_MODES[args.rtcm]}")
        if bad:
            print("!! the module is NOT in the configuration asked for:")
            for b in bad:
                print(f"     {b}")
            return 1
        print("# verified: flight configuration"
              + (f" + {NAV_MODES[args.navmode]} mode" if args.navmode is not None
                 else f" ({NAV_MODES[FLIGHT_NAV_MODE]} mode, as the Beetle flies)"))
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
