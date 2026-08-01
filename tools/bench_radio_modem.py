#!/usr/bin/env python3
"""Bench host for the LoRa daughterboard's UART modem link (#409).

The daughterboard is a transparent modem: it never parses the air frames it
radiates, so the thing that has to be proven on the bench is the *link* --
framing, the credit window, capability reporting -- and then that an air frame
pushed through it comes out of an unmodified base station unchanged.  The OC
(#410) and the BS (#414) are the production hosts; this is the bench one, so
that acceptance step does not need a rocket wired up to run.

Wire facts, all extracted from the firmware rather than transcribed:
  framing   AA 55 AA 55 | type | len | payload | CRC16-BE   (TR_MsgCodec.h)
  CRC       poly 0x8001, init 0x0000, no reflection, no xorout, over
            type+len+payload  (components/CRC defaults, as TR_I2C_Interface)
  protocol  components/TR_UART_Link/RadioModemProtocol.h, PROTOCOL_VERSION 1
  baud      921600 8N1, no RTS/CTS -- flow control is the in-band credit
            window (<= TX_QUEUE_CAPACITY unacknowledged TX_FRAME seqs)

Bench wiring to J6 on the daughterboard (3.3 V TTL adapter only -- the S3 is
not 5 V tolerant):
  J6.1 VSS   -> adapter GND
  J6.2 +BATT -> 6.4-8.4 V bench supply (or leave it and power over USB-C)
  J6.3       -> adapter RX     (board GPIO6, the modem's TX)
  J6.4       -> adapter TX     (board GPIO5, the modem's RX)

Examples:
  python3 tools/bench_radio_modem.py --selftest          # codec, no hardware
  python3 tools/bench_radio_modem.py -p /dev/tty.usbserial-X --identity
  python3 tools/bench_radio_modem.py -p PORT --status
  python3 tools/bench_radio_modem.py -p PORT --config 915 125 10 7 20
  python3 tools/bench_radio_modem.py -p PORT --tx DEADBEEF --repeat 20
  python3 tools/bench_radio_modem.py -p PORT --listen 60
  python3 tools/bench_radio_modem.py -p PORT --scan 902 928 500 30

Notes:
  * The modem emits BOOT (an identity payload) once at startup and it may
    arrive at any time -- every mode below decodes unsolicited frames, so a
    board that resets mid-run is visible rather than confusing.
  * --tx payloads are opaque to the modem by design.  To exercise a *real*
    base station you have to hand it bytes the deployed LORA_PROTO_VERSION
    parser accepts; --tx of arbitrary hex proves the tunnel and the credit
    return, not application interop.
  * The modem has no NVS: every setting is pushed by the host and is gone on
    reset.  --config is not persistent, and that is deliberate.
"""

import argparse
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Codec -- must stay byte-identical to TR_MsgCodec.cpp
# ---------------------------------------------------------------------------

SOF = bytes([0xAA, 0x55, 0xAA, 0x55])
MAX_PAYLOAD = 255

PROTOCOL_VERSION = 1
TX_QUEUE_CAPACITY = 8
MAX_AIR_FRAME = 247

# host -> modem
MSG_TX_FRAME     = 0x01
MSG_SET_CONFIG   = 0x02
MSG_HOP_FREQ     = 0x03
MSG_START_RX     = 0x04
MSG_GET_STATUS   = 0x05
MSG_GET_IDENTITY = 0x06
MSG_START_SCAN   = 0x07
# modem -> host
MSG_RX_FRAME     = 0x81
MSG_TX_RESULT    = 0x82
MSG_STATUS       = 0x83
MSG_IDENTITY     = 0x84
MSG_SCAN_RESULT  = 0x85
MSG_BOOT         = 0x86

TYPE_NAMES = {
    MSG_TX_FRAME: "TX_FRAME", MSG_SET_CONFIG: "SET_CONFIG",
    MSG_HOP_FREQ: "HOP_FREQ", MSG_START_RX: "START_RX",
    MSG_GET_STATUS: "GET_STATUS", MSG_GET_IDENTITY: "GET_IDENTITY",
    MSG_START_SCAN: "START_SCAN", MSG_RX_FRAME: "RX_FRAME",
    MSG_TX_RESULT: "TX_RESULT", MSG_STATUS: "STATUS",
    MSG_IDENTITY: "IDENTITY", MSG_SCAN_RESULT: "SCAN_RESULT",
    MSG_BOOT: "BOOT",
}

CFG_FLAG_CRC_ON           = 1 << 0
CFG_FLAG_RX_BOOSTED_GAIN  = 1 << 1
CFG_FLAG_SYNCWORD_PRIVATE = 1 << 2

CHIP_NAMES = {0: "UNKNOWN", 1: "LLCC68"}


def crc16(data: bytes) -> int:
    """CRC16 with the components/CRC library defaults (poly 0x8001, init 0)."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8001) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pack(msg_type: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)} > {MAX_PAYLOAD}")
    body = bytes([msg_type, len(payload)]) + payload
    return SOF + body + struct.pack(">H", crc16(body))


class Deframer:
    """Byte-at-a-time port of tr_msg::MsgDeframer (TR_MsgCodec.cpp).

    This is a state machine and not a `buf.find(SOF)` scan on purpose, because
    the two are NOT equivalent and the difference is observable on a real
    link:

      * `resync_bytes` differs. The firmware counts bytes its state machine
        discards, and a stray 0xAA that becomes a SOF candidate is accounted
        for differently than a byte simply skipped over. On the canonical
        `00 FF AA AA 13 55` garbage prefix the firmware counts 5, a find-based
        scan counts 6. That number is reported to hosts in
        STATUS.uart_rx_resync_bytes, so a bench tool that computes it
        differently mis-reports link health.

      * Recovery after a TRUNCATED frame differs, which is the one that
        matters. This framer is length-driven: a frame cut short leaves it
        consuming the *next* frame's bytes as payload/CRC, so a truncation
        costs two frames, not one. A find-based scan would re-anchor on the
        next SOF and lose only one -- i.e. it would quietly report better
        behaviour than the modem actually has.

    Both divergences were found by running the on-chip bench self-test
    (TR_BENCH_SELFTEST) against this file's expectations.
    """

    SOF_BYTES = (0xAA, 0x55, 0xAA, 0x55)

    # States, mirroring St:: in TR_MsgCodec.h
    _HUNT0, _HUNT1, _HUNT2, _HUNT3, _TYPE, _LEN, _PAYLOAD, _CRC0, _CRC1 = range(9)

    def __init__(self):
        self.state = self._HUNT0
        self.body = bytearray()   # type + len + payload, what the CRC covers
        self.payload_len = 0
        self.crc_hi = 0
        self.frames = 0
        self.crc_fails = 0
        self.resync_bytes = 0

    def _rehunt(self, b: int):
        """tr_msg::MsgDeframer::rehunt — a stray 0xAA restarts one byte in."""
        self.resync_bytes += 1
        self.state = self._HUNT1 if b == 0xAA else self._HUNT0

    def feed(self, data: bytes):
        """Yield (type, payload) for each complete CRC-valid frame."""
        for b in data:
            if self.state == self._HUNT0:
                if b == 0xAA:
                    self.state = self._HUNT1
                else:
                    self.resync_bytes += 1
            elif self.state == self._HUNT1:
                if b == 0x55:
                    self.state = self._HUNT2
                elif b == 0xAA:
                    # AA AA 55 ... — the second AA may start the real SOF.
                    self.resync_bytes += 1
                else:
                    self._rehunt(b)
            elif self.state == self._HUNT2:
                if b == 0xAA:
                    self.state = self._HUNT3
                else:
                    self._rehunt(b)
            elif self.state == self._HUNT3:
                if b == 0x55:
                    self.state = self._TYPE
                else:
                    self._rehunt(b)
            elif self.state == self._TYPE:
                self.body = bytearray([b])
                self.state = self._LEN
            elif self.state == self._LEN:
                self.body.append(b)
                self.payload_len = b
                self.state = self._PAYLOAD if b > 0 else self._CRC0
            elif self.state == self._PAYLOAD:
                self.body.append(b)
                if len(self.body) - 2 >= self.payload_len:
                    self.state = self._CRC0
            elif self.state == self._CRC0:
                self.crc_hi = b
                self.state = self._CRC1
            elif self.state == self._CRC1:
                expected = (self.crc_hi << 8) | b
                self.state = self._HUNT0
                if crc16(bytes(self.body)) == expected:
                    self.frames += 1
                    yield self.body[0], bytes(self.body[2:])
                else:
                    self.crc_fails += 1


# ---------------------------------------------------------------------------
# Payload structs -- layouts asserted in RadioModemProtocol.h
# ---------------------------------------------------------------------------

FMT_CONFIG      = "<ffBBbBHBB"        # 16 B
FMT_HOP         = "<f"                # 4 B
FMT_RX_HEADER   = "<ff"               # 8 B
FMT_TX_RESULT   = "<BB"               # 2 B
FMT_IDENTITY    = "<BBbBff32s"        # 44 B
FMT_STATUS      = "<IBBBBIIIIIIIfffB3s"  # 52 B
FMT_SCAN_REQ    = "<ffHH"             # 12 B
FMT_SCAN_HEADER = "<B3sff"            # 12 B

for _fmt, _size in ((FMT_CONFIG, 16), (FMT_HOP, 4), (FMT_RX_HEADER, 8),
                    (FMT_TX_RESULT, 2), (FMT_IDENTITY, 44), (FMT_STATUS, 52),
                    (FMT_SCAN_REQ, 12), (FMT_SCAN_HEADER, 12)):
    assert struct.calcsize(_fmt) == _size, f"{_fmt} != {_size} B"


def describe(msg_type: int, payload: bytes) -> str:
    name = TYPE_NAMES.get(msg_type, f"0x{msg_type:02X}")
    if msg_type in (MSG_IDENTITY, MSG_BOOT) and len(payload) == 44:
        ver, chip, pwr, _, fmin, fmax, fw = struct.unpack(FMT_IDENTITY, payload)
        fw = fw.split(b"\0", 1)[0].decode("ascii", "replace")
        warn = "" if ver == PROTOCOL_VERSION else f"  ** expected v{PROTOCOL_VERSION} **"
        return (f"{name} protocol=v{ver} chip={CHIP_NAMES.get(chip, chip)} "
                f"max_tx={pwr} dBm band={fmin:.3f}-{fmax:.3f} MHz fw={fw}{warn}")
    if msg_type == MSG_STATUS and len(payload) == 52:
        (up, en, rxm, qu, qc, tx_ok, tx_fail, rx_n, rx_crc, wd, u_crc, u_res,
         rssi, snr, freq, sf, _) = struct.unpack(FMT_STATUS, payload)
        return (f"{name} up={up / 1000:.1f}s radio={'up' if en else 'DOWN'} "
                f"rx_mode={rxm} txq={qu}/{qc} tx={tx_ok}/{tx_fail}f rx={rx_n} "
                f"rx_crc_fail={rx_crc} tx_wdog={wd} uart_crc_fail={u_crc} "
                f"uart_resync={u_res}B last={rssi:.0f}dBm/{snr:.1f}dB "
                f"@{freq:.3f}MHz SF{sf}")
    if msg_type == MSG_TX_RESULT and len(payload) == 2:
        seq, ok = struct.unpack(FMT_TX_RESULT, payload)
        return f"{name} seq={seq} {'ok' if ok else 'FAILED'}"
    if msg_type == MSG_RX_FRAME and len(payload) >= 8:
        rssi, snr = struct.unpack(FMT_RX_HEADER, payload[:8])
        air = payload[8:]
        return (f"{name} rssi={rssi:.0f} dBm snr={snr:.1f} dB "
                f"{len(air)} B air={air.hex().upper()}")
    if msg_type == MSG_SCAN_RESULT and len(payload) >= 12:
        count, _, start, step = struct.unpack(FMT_SCAN_HEADER, payload[:12])
        if len(payload) < 12 + count:
            return f"{name} TRUNCATED: header claims {count} samples, got {len(payload) - 12}"
        rssi = struct.unpack(f"<{count}b", payload[12:12 + count])
        peak = max(range(count), key=lambda i: rssi[i]) if count else 0
        return (f"{name} {count} samples from {start:.3f} MHz step {step:.0f} kHz; "
                f"peak {rssi[peak] if count else 0} dBm at "
                f"{start + peak * step / 1000:.3f} MHz")
    return f"{name} {len(payload)} B {payload.hex().upper()}"


# ---------------------------------------------------------------------------
# Link
# ---------------------------------------------------------------------------

class ModemLink:
    def __init__(self, port: str, baud: int, verbose: bool = True):
        import serial  # imported here so --selftest needs no pyserial
        self.ser = serial.Serial(port, baud, timeout=0)
        self.deframer = Deframer()
        self.verbose = verbose

    def send(self, msg_type: int, payload: bytes = b""):
        self.ser.write(pack(msg_type, payload))

    def pump(self, timeout_s: float):
        """Yield (type, payload) for `timeout_s`, printing each as it lands."""
        deadline = time.monotonic() + timeout_s
        while True:
            data = self.ser.read(4096)
            if data:
                for msg_type, payload in self.deframer.feed(data):
                    if self.verbose:
                        print(f"  [{time.strftime('%H:%M:%S')}] "
                              f"{describe(msg_type, payload)}")
                    yield msg_type, payload
            if time.monotonic() >= deadline:
                return
            if not data:
                time.sleep(0.002)

    def request(self, msg_type: int, want: int, timeout_s: float = 2.0,
                payload: bytes = b""):
        """Send a request and return the first matching reply payload."""
        self.send(msg_type, payload)
        for got_type, got_payload in self.pump(timeout_s):
            if got_type == want:
                return got_payload
        return None


def cmd_identity(link: ModemLink) -> int:
    if link.request(MSG_GET_IDENTITY, MSG_IDENTITY) is None:
        print("no IDENTITY reply -- check wiring/baud, and that TX/RX are not swapped",
              file=sys.stderr)
        return 1
    return 0


def cmd_status(link: ModemLink) -> int:
    if link.request(MSG_GET_STATUS, MSG_STATUS) is None:
        print("no STATUS reply", file=sys.stderr)
        return 1
    return 0


def cmd_config(link: ModemLink, freq, bw, sf, cr, pwr, start_rx: bool) -> int:
    flags = CFG_FLAG_CRC_ON | CFG_FLAG_RX_BOOSTED_GAIN | CFG_FLAG_SYNCWORD_PRIVATE
    payload = struct.pack(FMT_CONFIG, freq, bw, sf, cr, pwr, flags, 16,
                          1 if start_rx else 0, 0)
    # The modem acks every SET_CONFIG with a STATUS -- that ack is the only
    # proof the radio actually came up (radio_enabled in the status line).
    if link.request(MSG_SET_CONFIG, MSG_STATUS, payload=payload) is None:
        print("no STATUS ack for SET_CONFIG", file=sys.stderr)
        return 1
    return 0


def cmd_tx(link: ModemLink, air: bytes, repeat: int, gap_s: float,
           stall_s: float = 10.0) -> int:
    """Push `repeat` copies of `air`, respecting the in-band credit window.

    The contract being exercised: every accepted seq comes back in a
    TX_RESULT, so `unanswered` at the end should always be 0.  A nonzero
    count means the modem dropped a frame silently, which #409 says must not
    happen -- that is the failure this loop is here to catch.
    """
    if len(air) > MAX_AIR_FRAME:
        print(f"air frame {len(air)} B > MAX_AIR_FRAME {MAX_AIR_FRAME}", file=sys.stderr)
        return 1

    outstanding = set()
    sent = ok = failed = 0
    seq = 0
    last_progress = time.monotonic()

    while sent < repeat or outstanding:
        if sent < repeat and len(outstanding) < TX_QUEUE_CAPACITY:
            link.send(MSG_TX_FRAME, bytes([seq]) + air)
            outstanding.add(seq)
            seq = (seq + 1) & 0xFF
            sent += 1
            last_progress = time.monotonic()
            if gap_s:
                time.sleep(gap_s)
            continue

        # Either the window is full or everything is submitted: wait for
        # credits.  LoRa airtime dominates, so poll in short slices.
        for msg_type, payload in link.pump(0.1):
            if msg_type == MSG_TX_RESULT and len(payload) == 2:
                got_seq, good = struct.unpack(FMT_TX_RESULT, payload)
                if got_seq in outstanding:
                    outstanding.discard(got_seq)
                    last_progress = time.monotonic()
                    ok, failed = ok + bool(good), failed + (not good)

        if time.monotonic() - last_progress > stall_s:
            print(f"stalled {stall_s:.0f}s with {len(outstanding)} credit(s) "
                  f"unreturned: seq {sorted(outstanding)}", file=sys.stderr)
            break

    print(f"tx: {sent} submitted, {ok} radiated, {failed} rejected, "
          f"{len(outstanding)} unanswered")
    return 0 if failed == 0 and not outstanding else 1


def cmd_scan(link: ModemLink, start, stop, step_khz, dwell_ms) -> int:
    payload = struct.pack(FMT_SCAN_REQ, start, stop, step_khz, dwell_ms)
    # Sweep time is samples * dwell; give it that plus slack.
    budget = ((stop - start) * 1000 / max(step_khz, 1)) * dwell_ms / 1000.0 + 5.0
    if link.request(MSG_START_SCAN, MSG_SCAN_RESULT, timeout_s=budget,
                    payload=payload) is None:
        print("no SCAN_RESULT", file=sys.stderr)
        return 1
    return 0


# ---------------------------------------------------------------------------
# Self-test: codec only, no hardware
# ---------------------------------------------------------------------------

# Generated by the firmware codec itself (tr_msg::pack via the host build), so
# a divergence here means this file drifted from TR_MsgCodec.cpp.
GOLDEN = [
    (0x42, "010203", "AA55AA5542030102030066"),
    (MSG_GET_IDENTITY, "", "AA55AA5506000400"),
    (MSG_GET_STATUS, "", "AA55AA5505000600"),
    (MSG_TX_FRAME, "07DEADBEEF", "AA55AA55010507DEADBEEF2DA7"),
]


def selftest() -> int:
    failures = 0

    for msg_type, payload_hex, expect in GOLDEN:
        got = pack(msg_type, bytes.fromhex(payload_hex)).hex().upper()
        if got != expect:
            print(f"FAIL pack(0x{msg_type:02X}, {payload_hex or '-'}): "
                  f"{got} != {expect}")
            failures += 1
    print(f"ok  {len(GOLDEN)} golden frames match the firmware codec")

    # Round trip, including a max-length payload.
    d = Deframer()
    cases = [(MSG_BOOT, bytes(range(44))), (MSG_START_RX, b""),
             (0x77, bytes(MAX_PAYLOAD))]
    stream = b"".join(pack(t, p) for t, p in cases)
    got = list(d.feed(stream))
    if got != cases:
        print(f"FAIL round trip: {got!r}")
        failures += 1
    else:
        print(f"ok  {len(cases)} frames round-trip (incl. {MAX_PAYLOAD} B payload)")

    # Split across chunk boundaries, one byte at a time.
    d = Deframer()
    got = [f for b in pack(MSG_STATUS, bytes(52)) for f in d.feed(bytes([b]))]
    if got != [(MSG_STATUS, bytes(52))]:
        print(f"FAIL byte-at-a-time reassembly: {got!r}")
        failures += 1
    else:
        print("ok  frame reassembles across arbitrary chunk boundaries")

    # Leading garbage (including a false SOF prefix) then a good frame.
    # resync_bytes == 5, not 6: the firmware counts what its state machine
    # discards, and one 0xAA is consumed as a SOF candidate rather than
    # dropped. Verified against the on-chip self-test — do not "fix" to 6.
    d = Deframer()
    good = pack(0x33, b"\x01\x02")
    got = list(d.feed(b"\x00\xFF\xAA\xAA\x13\x55" + good))
    if got != [(0x33, b"\x01\x02")] or d.resync_bytes != 5:
        print(f"FAIL resync: {got!r} resync={d.resync_bytes} (want 5)")
        failures += 1
    else:
        print(f"ok  resynchronizes past garbage ({d.resync_bytes} B discarded)")

    # A truncated frame costs TWO frames, not one. This framer is
    # length-driven, so a frame cut short keeps consuming: it eats the next
    # frame's SOF as its own payload/CRC, fails the CRC, and only recovers on
    # the frame after that. Documented in TR_MsgCodec.h as "a rare drop is
    # recovered by the next frame"; pinned here because a find-based deframer
    # would lose only one and thereby overstate link quality.
    d = Deframer()
    truncated = pack(0x66, bytes(range(1, 9)))[:-4]
    got = list(d.feed(truncated + pack(0x77, b"\xEE") + pack(0x78, b"\xEF")))
    if got != [(0x78, b"\xEF")] or d.crc_fails != 1:
        print(f"FAIL truncation: {got!r} crc_fails={d.crc_fails} "
              f"(want only 0x78 through, 1 CRC fail)")
        failures += 1
    else:
        print("ok  truncated frame swallows exactly one follower, then recovers")

    # A corrupted frame is dropped and counted; the next one still parses.
    d = Deframer()
    bad = bytearray(pack(0x44, b"\x0A\x14\x1E"))
    bad[7] ^= 0xFF
    got = list(d.feed(bytes(bad) + pack(0x55, b"\x2A")))
    if got != [(0x55, b"\x2A")] or d.crc_fails != 1:
        print(f"FAIL corruption: {got!r} crc_fails={d.crc_fails}")
        failures += 1
    else:
        print("ok  CRC-corrupted frame dropped and counted; next frame parses")

    # Struct sizes are asserted at import; say so explicitly.
    print("ok  8 payload struct layouts match RadioModemProtocol.h")

    # Every decoder runs on a synthetic payload of the right size: a field
    # order that drifts from RadioModemProtocol.h shows up as a wrong value
    # here rather than as nonsense on the bench.
    decoders = [
        (MSG_IDENTITY,
         struct.pack(FMT_IDENTITY, PROTOCOL_VERSION, 1, 22, 0, 850.0, 930.0,
                     b"abc123+20260731-1200"),
         ["protocol=v1", "chip=LLCC68", "max_tx=22", "850.000-930.000",
          "fw=abc123+20260731-1200"]),
        (MSG_STATUS,
         struct.pack(FMT_STATUS, 12345, 1, 1, 2, 8, 100, 3, 55, 1, 0, 0, 0,
                     -91.0, 7.5, 915.0, 10, b""),
         ["up=12.3s", "radio=up", "txq=2/8", "tx=100/3f", "rx=55",
          "-91dBm/7.5dB", "@915.000MHz", "SF10"]),
        (MSG_TX_RESULT, struct.pack(FMT_TX_RESULT, 7, 0), ["seq=7", "FAILED"]),
        (MSG_RX_FRAME, struct.pack(FMT_RX_HEADER, -104.0, -3.25) + b"\xDE\xAD",
         ["-104 dBm", "-3.2 dB", "2 B", "DEAD"]),
        (MSG_SCAN_RESULT,
         struct.pack(FMT_SCAN_HEADER, 3, b"", 902.0, 500.0) +
         struct.pack("<3b", -120, -60, -110),
         ["3 samples", "902.000 MHz", "step 500 kHz", "peak -60 dBm",
          "at 902.500 MHz"]),
    ]
    for msg_type, payload, expect in decoders:
        line = describe(msg_type, payload)
        missing = [e for e in expect if e not in line]
        if missing:
            print(f"FAIL decode {TYPE_NAMES[msg_type]}: missing {missing}\n     {line}")
            failures += 1
    print(f"ok  {len(decoders)} payload decoders read back the fields they packed")

    # A short SCAN_RESULT must be reported, not crash on the sample unpack.
    truncated = describe(MSG_SCAN_RESULT,
                         struct.pack(FMT_SCAN_HEADER, 64, b"", 902.0, 500.0))
    if "TRUNCATED" not in truncated:
        print(f"FAIL truncated SCAN_RESULT not flagged: {truncated}")
        failures += 1
    else:
        print("ok  truncated SCAN_RESULT is reported rather than crashing")

    print("FAILED" if failures else "\nAll codec self-tests passed.")
    return 1 if failures else 0


# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Bench host for the LoRa daughterboard UART modem link (#409)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Examples:", 1)[1] if "Examples:" in __doc__ else None)
    ap.add_argument("-p", "--port", help="serial device of the USB-UART adapter")
    ap.add_argument("-b", "--baud", type=int, default=921600,
                    help="must match config::HOST_UART_BAUD (default 921600)")
    ap.add_argument("--selftest", action="store_true",
                    help="verify the codec against golden firmware frames; no hardware")
    ap.add_argument("--identity", action="store_true", help="GET_IDENTITY")
    ap.add_argument("--status", action="store_true", help="GET_STATUS")
    ap.add_argument("--config", nargs=5, metavar=("FREQ_MHZ", "BW_KHZ", "SF", "CR", "DBM"),
                    help="SET_CONFIG, e.g. --config 915 125 10 7 20")
    ap.add_argument("--no-rx", action="store_true",
                    help="with --config: do not enter RX after applying")
    ap.add_argument("--hop", type=float, metavar="MHZ", help="HOP_FREQ (no reply)")
    ap.add_argument("--start-rx", action="store_true", help="START_RX")
    ap.add_argument("--tx", metavar="HEX", help="tunnel these air bytes (hex)")
    ap.add_argument("--repeat", type=int, default=1, help="with --tx (default 1)")
    ap.add_argument("--gap", type=float, default=0.0,
                    help="with --tx: seconds between submissions (default 0)")
    ap.add_argument("--scan", nargs=4, type=float,
                    metavar=("START_MHZ", "STOP_MHZ", "STEP_KHZ", "DWELL_MS"),
                    help="START_SCAN, e.g. --scan 902 928 500 30")
    ap.add_argument("--listen", type=float, metavar="SECONDS",
                    help="decode everything the modem sends for N seconds")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.port:
        ap.error("--port is required (or use --selftest)")

    try:
        link = ModemLink(args.port, args.baud)
    except ImportError:
        print("pyserial not installed:  python3 -m pip install pyserial", file=sys.stderr)
        return 2

    rc = 0
    did_something = False
    if args.identity:
        rc |= cmd_identity(link); did_something = True
    if args.config:
        freq, bw, sf, cr, pwr = args.config
        rc |= cmd_config(link, float(freq), float(bw), int(sf), int(cr), int(pwr),
                         not args.no_rx)
        did_something = True
    if args.hop is not None:
        link.send(MSG_HOP_FREQ, struct.pack(FMT_HOP, args.hop))
        print(f"  HOP_FREQ {args.hop:.3f} MHz sent (fire-and-forget, no reply)")
        did_something = True
    if args.start_rx:
        link.send(MSG_START_RX); did_something = True
    if args.tx:
        rc |= cmd_tx(link, bytes.fromhex(args.tx), args.repeat, args.gap)
        did_something = True
    if args.scan:
        rc |= cmd_scan(link, args.scan[0], args.scan[1], int(args.scan[2]),
                       int(args.scan[3]))
        did_something = True
    if args.status:
        rc |= cmd_status(link); did_something = True
    if args.listen:
        print(f"listening {args.listen:.0f}s (Ctrl-C to stop)")
        try:
            for _ in link.pump(args.listen):
                pass
        except KeyboardInterrupt:
            pass
        did_something = True

    if not did_something:
        ap.error("nothing to do: pick an action (--identity/--status/--config/...)")
    return rc


if __name__ == "__main__":
    sys.exit(main())
