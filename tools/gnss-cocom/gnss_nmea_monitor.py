#!/usr/bin/env python3
"""Host monitor for the PX1105R COCOM bench rig (#491).

Reads the NMEA stream a TinkerNav V25 puts on USB via
firmware/gnss_passthrough, timestamps it, and classifies each epoch as one of
the three states #491 has to tell apart:

  FIX       position reported
  BLOCKED   satellites still tracked at healthy C/N0, position withheld
  NO_LOCK   C/N0 collapsed / satellites dropped out first

That distinction is the whole measurement.  A COCOM gate withholds the *fix*
while tracking continues, so GGA quality falls to 0 and RMC status to 'V' while
GSV still lists strong satellites -- a sharp, reversible transition at the
limit.  Losing the signal instead takes the satellites down first.  Reading
only GGA cannot separate them, so GSV/GSA are parsed alongside and the verdict
is computed from both.

For step one of the rig this doubles as plain bring-up: run it with no
arguments and it finds the board, proves sentences arrive with good checksums,
and prints what the receiver is seeing.

Wire facts (TinkerNav V25 netlist + firmware/gnss_passthrough):
  transport  ESP32-S3 native USB CDC, VID 0x303A -- `Serial` is the USB port,
             there is no bridge chip, so the port only exists while the sketch
             runs (or in the ROM bootloader, which enumerates differently)
  host baud  irrelevant on native USB CDC; the receiver-side rate is
             autodetected by the sketch and is not set from here
  banner     the sketch prefixes human-readable output with '#', which is not a
             legal NMEA start character, and those lines are dropped below

Examples:
  python3 tools/gnss-cocom/gnss_nmea_monitor.py --list
  python3 tools/gnss-cocom/gnss_nmea_monitor.py                     # autodetect, live
  python3 tools/gnss-cocom/gnss_nmea_monitor.py --seconds 30
  python3 tools/gnss-cocom/gnss_nmea_monitor.py --raw
  python3 tools/gnss-cocom/gnss_nmea_monitor.py --log t1_velramp.nmea
  python3 tools/gnss-cocom/gnss_nmea_monitor.py --replay t1_velramp.nmea

Notes:
  * --log writes the capture that the COCOM analysis reads back: one line per
    sentence, prefixed with a host monotonic timestamp.  Correlating the
    blank-out sample against the injected trajectory's altitude/velocity at
    that instant is what yields the threshold, so the timestamp is the point.
  * --replay re-runs the classifier over a capture with no hardware attached,
    which is how a run gets re-analysed after the bench is packed up.
  * Checksum failures are counted, never silently dropped.  A rising error rate
    is the tell for a baud mismatch or a marginal USB cable, and it would
    otherwise masquerade as intermittent GNSS trouble.
  * Nothing here writes to the receiver.  Configuration frames are a separate
    concern; the bridge forwards host bytes verbatim if that is wanted later.
"""

import argparse
import sys
import time
from collections import Counter
from dataclasses import dataclass, field, replace

from skytraq_binary import (MSG_RCV_STATE, MSG_SV_CH_STATUS, iter_frames, key,
                            parse_rcv_state, parse_sv_ch_status)

# USB vendor IDs of the MCUs the passthrough firmware supports.  Espressif
# covers the ESP32-S3/C3 builds; Raspberry Pi covers the RP2040 build, which is
# the one that matters on the dual-MCU TinkerNav boards where the receiver hangs
# off the RP2040 rather than the ESP32.
ESP32_VID  = 0x303A
RP2040_VID = 0x2E8A
BOARD_VIDS = (ESP32_VID, RP2040_VID)

# A satellite at or above this C/N0 counts as "tracked" for the BLOCKED test.
# 30 dBHz is comfortably above the ~25 dBHz where a receiver starts dropping
# satellites, and well below the 40-45 dBHz the injection is tuned for, so the
# classifier is not sensitive to the exact injection level.
TRACKED_CN0_DBHZ = 30.0

# Satellites needed at TRACKED_CN0_DBHZ before "still tracking" is claimed.
TRACKED_MIN_SATS = 4


# --------------------------------------------------------------------------
# NMEA
# --------------------------------------------------------------------------

def nmea_checksum_ok(sentence: str) -> bool:
    """True if sentence is '$<body>*HH' with HH == XOR of body."""
    if not sentence.startswith("$"):
        return False
    star = sentence.rfind("*")
    if star < 0 or star + 3 > len(sentence):
        return False
    body = sentence[1:star]
    try:
        want = int(sentence[star + 1:star + 3], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def _f(value: str):
    """Float or None; NMEA leaves fields empty rather than zeroed."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _i(value: str):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _latlon(raw: str, hemi: str):
    """ddmm.mmmm + hemisphere -> signed degrees."""
    v = _f(raw)
    if v is None or not hemi:
        return None
    deg = int(v / 100.0)
    minutes = v - deg * 100.0
    dec = deg + minutes / 60.0
    return -dec if hemi in ("S", "W") else dec


@dataclass
class Epoch:
    """Rolling receiver state, updated sentence by sentence."""

    # GGA
    fix_quality: int = 0
    sats_used: int = 0
    hdop: float = None
    lat: float = None
    lon: float = None
    alt_m: float = None
    utc: str = ""

    # RMC
    rmc_status: str = ""       # 'A' valid, 'V' warning (no fix)
    speed_mps: float = None

    # Binary only: the receiver's stated navigation state, which is better
    # evidence than an inferred one -- NMEA leaves you guessing from empty
    # fields, while 0xDF says NO_FIX / FIX_2D / FIX_3D / FIX_DIFFERENTIAL.
    nav_state: str = ""

    # GSA / GSV
    fix_type: int = 0          # 1 none, 2 = 2D, 3 = 3D
    sats_in_view: int = 0
    cn0: dict = field(default_factory=dict)   # prn -> C/N0 dBHz

    def snapshot(self) -> "Epoch":
        """Deep-enough copy for the transition log.

        The parser mutates one Epoch in place, so a transition record that held
        a reference would read back as the *final* receiver state instead of the
        state at the transition -- silently destroying the one measurement this
        rig exists to take.
        """
        return replace(self, cn0=dict(self.cn0))

    def tracked_sats(self, threshold=TRACKED_CN0_DBHZ) -> int:
        return sum(1 for v in self.cn0.values() if v is not None and v >= threshold)

    def max_cn0(self):
        vals = [v for v in self.cn0.values() if v is not None]
        return max(vals) if vals else None

    def mean_cn0(self):
        vals = [v for v in self.cn0.values() if v is not None and v > 0]
        return sum(vals) / len(vals) if vals else None

    def has_position(self) -> bool:
        return self.fix_quality > 0 and self.lat is not None and self.lon is not None

    def verdict(self) -> str:
        """FIX / BLOCKED / NO_LOCK -- the #491 measurement."""
        if self.has_position():
            return "FIX"
        if self.tracked_sats() >= TRACKED_MIN_SATS:
            return "BLOCKED"
        return "NO_LOCK"


class Parser:
    """Feeds sentences into a rolling Epoch.

    GSV arrives as a multi-message set describing all satellites in view.  The
    C/N0 map is rebuilt per set rather than mutated in place, so a satellite
    that stops being reported disappears instead of lingering at its last value
    -- which matters here, because a stale strong satellite would make a real
    loss of lock read as a COCOM block.
    """

    def __init__(self):
        self.epoch = Epoch()
        self.counts = Counter()
        self.good = 0
        self.bad = 0
        self._gsv_acc = {}
        self._gsv_talkers_open = set()

    def feed(self, sentence: str) -> bool:
        """Parse one sentence. Returns True if the checksum was valid."""
        sentence = sentence.strip()
        if not sentence.startswith("$"):
            return False
        if not nmea_checksum_ok(sentence):
            self.bad += 1
            return False
        self.good += 1

        star = sentence.rfind("*")
        fields = sentence[1:star].split(",")
        talker, kind = fields[0][:2], fields[0][2:]
        self.counts[fields[0]] += 1

        if kind == "GGA":
            self._gga(fields)
        elif kind == "RMC":
            self._rmc(fields)
        elif kind == "GSA":
            self._gsa(fields)
        elif kind == "GSV":
            self._gsv(talker, fields)
        return True

    def feed_binary(self, payload: bytes) -> bool:
        """Update the epoch from one SkyTraq binary frame."""
        if not payload:
            return False
        mid = payload[0]

        if mid == MSG_RCV_STATE:
            r = parse_rcv_state(payload)
            if not r:
                self.bad += 1
                return False
            self.counts["0xDF RCV_STATE"] += 1
            self.good += 1
            e = self.epoch
            e.nav_state = r["nav_state_name"]
            e.fix_quality = 1 if r["has_fix"] else 0
            e.lat, e.lon, e.alt_m = r["lat"], r["lon"], r["alt_m"]
            e.speed_mps = r["speed_mps"]
            e.hdop = r["hdop"]
            # Map the enum onto the 1/2/3 the display already speaks.
            e.fix_type = {0: 1, 1: 1, 2: 2, 3: 3, 4: 3}.get(r["nav_state"], 1)
            return True

        if mid == MSG_SV_CH_STATUS:
            sats = parse_sv_ch_status(payload)
            if sats is None:
                self.bad += 1
                return False
            self.counts["0xE7 SV_CH_STATUS"] += 1
            self.good += 1
            e = self.epoch
            # Replace wholesale, as with GSV: a satellite that drops out must
            # disappear rather than linger at its last C/N0, or a real loss of
            # lock reads as a COCOM block.
            e.cn0 = {key(s): float(s["cn0"]) for s in sats}
            e.sats_in_view = len(sats)
            e.sats_used = sum(1 for s in sats if s["used_in_fix"])
            return True

        self.counts[f"0x{mid:02X} (unparsed)"] += 1
        return False

    def _gga(self, f):
        e = self.epoch
        if len(f) < 10:
            return
        e.utc = f[1]
        e.lat = _latlon(f[2], f[3])
        e.lon = _latlon(f[4], f[5])
        e.fix_quality = _i(f[6]) or 0
        e.sats_used = _i(f[7]) or 0
        e.hdop = _f(f[8])
        e.alt_m = _f(f[9])

    def _rmc(self, f):
        e = self.epoch
        if len(f) < 8:
            return
        e.rmc_status = f[2]
        knots = _f(f[7])
        e.speed_mps = knots * 0.514444 if knots is not None else None

    def _gsa(self, f):
        if len(f) < 3:
            return
        self.epoch.fix_type = _i(f[2]) or 0

    def _gsv(self, talker, f):
        # $xxGSV,total,msg,inview,(prn,elev,az,cn0)*4
        if len(f) < 4:
            return
        total, msg = _i(f[1]), _i(f[2])
        if total is None or msg is None:
            return

        if msg == 1:
            # First message of this talker's set: start its contribution over.
            self._gsv_acc = {k: v for k, v in self._gsv_acc.items()
                             if not k.startswith(talker)}
            self._gsv_talkers_open.add(talker)

        for i in range(4, len(f) - 3, 4):
            prn = f[i]
            if not prn:
                continue
            self._gsv_acc[f"{talker}:{prn}"] = _f(f[i + 3])

        if msg == total:
            self._gsv_talkers_open.discard(talker)
            if not self._gsv_talkers_open:
                # Every talker has closed its set: publish an atomic snapshot.
                self.epoch.cn0 = dict(self._gsv_acc)
                self.epoch.sats_in_view = len(self._gsv_acc)


# --------------------------------------------------------------------------
# Ports
# --------------------------------------------------------------------------

def list_ports():
    from serial.tools import list_ports as lp
    return list(lp.comports())


# Enough USB vendors to name what is actually plugged in.  Identifying by vendor
# rather than by the port name matters: on macOS *every* native-USB CDC device
# enumerates as /dev/cu.usbmodemNNNN, so matching that substring cheerfully
# reports a Raspberry Pi Pico as an ESP32 and sends you debugging the wrong
# board.  (It did exactly that here before this was tightened.)
KNOWN_VENDORS = {
    0x303A: "Espressif",
    0x2E8A: "Raspberry Pi",
    0x10C4: "SiLabs CP210x",
    0x1A86: "WCH CH34x",
    0x0403: "FTDI",
    0x239A: "Adafruit",
}


def _is_board(p) -> bool:
    """A device whose vendor matches an MCU the passthrough runs on.

    Matched by vendor ID, never by port name: on macOS every native-USB CDC
    device is /dev/cu.usbmodemNNNN, so a name match happily reports somebody
    else's board as this one.
    """
    return p.vid in BOARD_VIDS


def _vendor(p) -> str:
    if p.vid is None:
        return ""
    return KNOWN_VENDORS.get(p.vid, f"vendor {p.vid:04X}")


def describe_ports():
    ports = list_ports()
    if not ports:
        print("no serial ports found")
        return
    for p in ports:
        vid = f"{p.vid:04X}" if p.vid is not None else "----"
        pid = f"{p.pid:04X}" if p.pid is not None else "----"
        star = " *" if _is_board(p) else "  "
        print(f"{star}{p.device:28} {vid}:{pid}  {_vendor(p):14} {p.description}")
    print("\n* = a board the passthrough runs on (Espressif 303A / Raspberry Pi 2E8A).")


def autodetect_port() -> str:
    cands = [p for p in list_ports() if _is_board(p)]
    if not cands:
        others = [p for p in list_ports() if p.vid is not None]
        detail = ""
        if others:
            seen = ", ".join(f"{_vendor(p)} on {p.device}" for p in others)
            detail = f"\n  what IS attached: {seen}"
        raise SystemExit(
            "no passthrough board found (Espressif 303A / Raspberry Pi 2E8A)." + detail + "\n"
            "  - is the board plugged in and powered?\n"
            "  - native USB means the port vanishes when no firmware is running\n"
            "  - `--list` shows every port; force one with `-p` if you disagree"
        )
    if len(cands) > 1:
        names = ", ".join(c.device for c in cands)
        raise SystemExit(f"multiple candidate ports ({names}); pick one with -p")

    chosen = cands[0]
    print(f"# found {chosen.device}  ({chosen.description})")
    # The S3 exposes two different USB personalities on the same VID:PID, and
    # only the description tells them apart.  The built-in USB-Serial-JTAG
    # bridge enumerates whether or not any sketch is running, so finding it is
    # not evidence the passthrough is loaded -- and on a board flashed without
    # "USB CDC On Boot" it is all you will ever get.
    if "jtag" in (chosen.description or "").lower():
        print("# note: this is the S3's built-in USB-Serial-JTAG, not a sketch's USB CDC.")
        print("#       if no sentences arrive, the passthrough may not be running, or")
        print("#       it was built without CDCOnBoot=cdc.")
    return chosen.device


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------

VERDICT_NOTE = {
    "FIX":     "position reported",
    "BLOCKED": "sats tracked, position withheld  <-- COCOM signature",
    "NO_LOCK": "no position, no strong sats",
}


def status_line(t: float, e: Epoch) -> str:
    v = e.verdict()
    cn0 = e.mean_cn0()
    mx = e.max_cn0()
    pos = f"{e.lat:10.6f},{e.lon:11.6f}" if e.has_position() else f"{'--':>10},{'--':>11}"
    alt = f"{e.alt_m:8.1f}" if e.alt_m is not None else f"{'--':>8}"
    spd = f"{e.speed_mps:6.1f}" if e.speed_mps is not None else f"{'--':>6}"
    state = f" {e.nav_state}" if e.nav_state else ""
    return (
        f"{t:7.1f}s {v:<8}{state} q={e.fix_quality} {e.fix_type}D "
        f"used={e.sats_used:2d} trk={e.tracked_sats():2d}/{e.sats_in_view:2d} "
        f"C/N0 {cn0 or 0:4.1f}/{mx or 0:4.1f} "
        f"pos {pos} alt {alt} m  v {spd} m/s"
    )


def print_summary(p: Parser, elapsed: float, transitions, verdict_time):
    print()
    print("=" * 78)
    print(f"ran {elapsed:.1f} s   sentences ok={p.good} bad-checksum={p.bad}", end="")
    if p.good + p.bad:
        print(f"  ({100.0 * p.bad / (p.good + p.bad):.2f}% bad)")
    else:
        print()

    if not p.good:
        print()
        print("NOTHING PARSED. Things to check, in order:")
        print("  1. is the passthrough running? its banner lines start with '#'")
        print("  2. receiver powered, and on the MCU you are talking to --")
        print("     on dual-MCU TinkerNav boards the receiver is on the RP2040")
        print("  3. baud: the sketch prints its probe results as '# ...' lines")
        print("  4. --raw shows the bytes; D3.. is RTCM, A0A1.. SkyTraq binary")
        return

    print("\nmessages seen:")
    for name, n in sorted(p.counts.items(), key=lambda kv: -kv[1]):
        print(f"    {name:8} {n:6d}")

    binary = any(k.startswith("0x") for k in p.counts)
    if binary:
        need = [n for n in ("0xDF RCV_STATE", "0xE7 SV_CH_STATUS") if n not in p.counts]
        if need:
            print(f"\n  WARNING: no {', '.join(need)}. The COCOM verdict needs 0xDF")
            print("  (navigation state, position, velocity) and 0xE7 (per-SV C/N0)")
            print("  to tell a withheld fix from a lost one.")
    else:
        missing = [s for s in ("GGA", "GSV", "GSA", "RMC")
                   if not any(k.endswith(s) for k in p.counts)]
        if missing:
            print(f"\n  WARNING: no {', '.join(missing)} seen. The COCOM verdict needs")
            print("  GGA+RMC (position) and GSV+GSA (tracking) to tell a block from a")
            print("  loss of lock. Enable them on the receiver before a real run.")

    total = sum(verdict_time.values()) or 1.0
    print("\ntime by verdict:")
    for v in ("FIX", "BLOCKED", "NO_LOCK"):
        secs = verdict_time.get(v, 0.0)
        print(f"    {v:<8} {secs:7.1f} s  {100.0 * secs / total:5.1f}%   {VERDICT_NOTE[v]}")

    if transitions:
        print("\ntransitions:")
        for t, a, b, e, good in transitions:
            detail = (f"trk={e.tracked_sats()}/{e.sats_in_view} "
                      f"maxC/N0={e.max_cn0() or 0:.1f}")
            print(f"    {t:7.1f}s  {a} -> {b}   {detail}")
            if good is not None:
                alt = f"{good.alt_m:.0f} m" if good.alt_m is not None else "--"
                spd = f"{good.speed_mps:.0f} m/s" if good.speed_mps is not None else "--"
                print(f"                last good fix: alt {alt}, speed {spd}")
        print("\n  Correlate each timestamp against the injected trajectory's")
        print("  altitude/velocity at that instant -- that is the threshold.")
    else:
        print("\nno state transitions (steady state for the whole run)")
    print("=" * 78)


# --------------------------------------------------------------------------
# Run loops
# --------------------------------------------------------------------------

def run(source, args, log_fh):
    """Drive the parser over an iterator of (timestamp, line).

    Ctrl-C is caught here rather than at the call site so an interrupted run
    still yields its analysis -- stopping by hand is the normal way a bench run
    ends, and discarding the summary would mean re-running the whole injection.
    """
    parser = Parser()
    transitions = []
    verdict_time = Counter()
    last_verdict = None
    last_t = 0.0
    next_print = 0.0
    # The last epoch that actually had a fix.  A COCOM block withholds velocity
    # along with position, so the state *at* the transition reads 0 m/s -- the
    # speed that mattered is the one from the sample just before it blanked.
    last_good = None

    try:
        for t, kind, data in source:
            if kind == "bin":
                if log_fh is not None:
                    log_fh.write(f"{t:.3f} B {data.hex()}\n")
                if args.raw:
                    print(f"{t:8.3f} [bin 0x{data[0]:02X}] {data.hex()}")
                if not parser.feed_binary(data):
                    continue
            else:
                line = data
                if log_fh is not None:
                    log_fh.write(f"{t:.3f} {line}\n")
                if args.raw:
                    print(f"{t:8.3f} {line}")
                if line.startswith("#"):
                    if not args.raw:
                        print(line)
                    continue
                if not parser.feed(line):
                    continue

            e = parser.epoch
            v = e.verdict()

            # Attribute elapsed time to the verdict that was in force during it.
            if last_verdict is not None:
                verdict_time[last_verdict] += t - last_t
            last_t = t

            if e.has_position():
                last_good = e.snapshot()

            if v != last_verdict:
                if last_verdict is not None:
                    transitions.append((t, last_verdict, v, e.snapshot(), last_good))
                    if not args.raw:
                        print(f"\n>>> {t:7.1f}s  {last_verdict} -> {v}   "
                              f"{VERDICT_NOTE[v]}")
                last_verdict = v

            if not args.raw and t >= next_print:
                next_print = t + args.interval
                print(status_line(t, e))
    except KeyboardInterrupt:
        print("\n# interrupted")

    return parser, transitions, verdict_time, last_t


def _demux(buf: bytearray):
    """Split a raw stream into ('bin', payload) and ('nmea', line) events.

    Binary frames are extracted first because they are self-delimiting and end
    with 0D 0A -- split the stream on newlines first and every frame shatters
    into unreadable fragments.  Whatever survives frame extraction is text.
    """
    for payload in iter_frames(buf):
        yield "bin", payload
    while True:
        nl = buf.find(b"\n")
        if nl < 0:
            break
        # Stop if a frame preamble begins before this newline: that frame is
        # still arriving, and its body may legitimately contain 0x0A.
        pre = buf.find(b"\xa0\xa1")
        if 0 <= pre < nl:
            break
        line = bytes(buf[:nl]).decode("ascii", errors="replace").strip()
        del buf[:nl + 1]
        if line:
            yield "nmea", line


def serial_source(ser, deadline):
    """Yield (elapsed, kind, data) until the deadline, the port dies, or Ctrl-C.

    A port that drops mid-run is reported and ends the iteration rather than
    raising: an unplugged cable ten minutes into an injection run should still
    leave the analysis of what was captured, not a traceback.
    """
    import serial as _serial

    t0 = time.monotonic()
    buf = bytearray()
    while True:
        if deadline is not None and time.monotonic() - t0 >= deadline:
            return
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except _serial.SerialException as exc:
            print(f"\n# serial read failed: {exc}")
            print("# port dropped -- cable, board reset, or another program holding it")
            return
        if chunk:
            buf.extend(chunk)
            for kind, data in _demux(buf):
                yield time.monotonic() - t0, kind, data


def replay_source(path):
    """Replay a capture. Lines are 'TS text' or 'TS B <hex>' for binary frames."""
    with open(path, "r", errors="replace") as fh:
        for raw in fh:
            raw = raw.rstrip("\n")
            if not raw:
                continue
            ts, _, rest = raw.partition(" ")
            try:
                t = float(ts)
            except ValueError:
                # A capture without timestamps still replays, just untimed.
                yield 0.0, "nmea", raw
                continue
            if rest.startswith("B "):
                try:
                    yield t, "bin", bytes.fromhex(rest[2:])
                except ValueError:
                    pass
                continue
            yield t, "nmea", rest


def main():
    ap = argparse.ArgumentParser(
        description="PX1105R NMEA monitor / COCOM classifier (#491)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("-p", "--port", help="serial port (default: autodetect)")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--seconds", type=float, default=None,
                    help="run for N seconds then summarise (default: until Ctrl-C)")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="status line period in seconds (default: 1.0)")
    ap.add_argument("--raw", action="store_true",
                    help="dump every line instead of the status display")
    ap.add_argument("--log", metavar="FILE",
                    help="write a timestamped capture for later --replay")
    ap.add_argument("--replay", metavar="FILE",
                    help="re-analyse a capture instead of reading hardware")
    args = ap.parse_args()

    if args.list:
        describe_ports()
        return 0

    log_fh = open(args.log, "w") if args.log else None
    try:
        if args.replay:
            print(f"# replaying {args.replay}")
            # Elapsed comes from the capture's own timestamps, not the wall clock.
            p, tr, vt, last_t = run(replay_source(args.replay), args, None)
            print_summary(p, last_t, tr, vt)
            return 0

        try:
            import serial
        except ImportError:
            raise SystemExit("pyserial not installed:  python3 -m pip install pyserial")

        port = args.port or autodetect_port()
        print(f"# opening {port}")
        try:
            # Native USB CDC ignores the rate; 115200 is conventional.
            ser = serial.Serial(port, 115200, timeout=0.1)
        except serial.SerialException as exc:
            raise SystemExit(
                f"could not open {port}: {exc}\n"
                "  - another program (screen, idf.py monitor, Arduino IDE) may hold it\n"
                "  - the board may have reset; `--list` re-checks what is present"
            )
        with ser:
            t0 = time.monotonic()
            p, tr, vt, _ = run(serial_source(ser, args.seconds), args, log_fh)
            print_summary(p, time.monotonic() - t0, tr, vt)
        if log_fh:
            print(f"\ncapture written to {args.log}")
        return 0
    finally:
        if log_fh:
            log_fh.close()


if __name__ == "__main__":
    sys.exit(main())
