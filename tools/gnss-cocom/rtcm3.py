"""RTCM 10403.3 framing and MSM7 decoding, for the LC86G's raw measurements.

The Quectel LC86G has no binary navigation protocol -- position, velocity and
fix state exist only as NMEA and $PQTM sentences -- but it will emit RTCM3
multiple-signal messages ($PAIR432,1 = MSM7). Those carry, per satellite and
signal, at the full fix rate:

    C/N0                 DF408, 2^-4 dB-Hz        (GSV: integer, ~1 Hz here)
    phase-range rate     DF399 + DF404, m/s       -> the receiver's own Doppler
    lock-time indicator  DF407                    -> resets when a loop drops

which is what section 06's Doppler analysis took from the NEO-M8T's RXM-RAWX,
and what says whether a loss of lock hit every channel at once or the
steepest-Doppler ones first.

Frames are pulled out of a mixed NMEA + RTCM byte stream WITHOUT disturbing the
text around them: only a frame whose CRC-24Q checks is removed. NMEA is ASCII,
so 0xD3 never occurs in it; a 0xD3 that does not start a valid frame is left
where it is.
"""

from __future__ import annotations

PREAMBLE = 0xD3
L1_WAVELENGTH_M = 299792458.0 / 1575.42e6

MSM7_TYPES = {1077: "GPS", 1087: "GLONASS", 1097: "Galileo", 1117: "QZSS",
              1127: "BDS"}


def crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def iter_frames(buf: bytearray):
    """Yield each complete, CRC-valid RTCM3 message body from `buf`, removing
    exactly those bytes. Everything else stays, for the NMEA splitter."""
    i = 0
    while True:
        i = buf.find(bytes([PREAMBLE]), i)
        if i < 0 or i + 3 > len(buf):
            return
        if buf[i + 1] & 0xFC:            # six reserved bits must be zero
            i += 1
            continue
        n = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
        end = i + 3 + n + 3
        if end > len(buf):
            return                        # still arriving
        frame = bytes(buf[i:end])
        if crc24q(frame[:-3]) != int.from_bytes(frame[-3:], "big"):
            i += 1
            continue
        del buf[i:end]
        yield frame[3:-3]


def pending_frame_start(buf: bytearray) -> int:
    """Index of a frame that has started arriving but is not complete, or -1.

    The NMEA splitter must not cut a line at a 0x0A inside such a frame.
    """
    i = 0
    while True:
        i = buf.find(bytes([PREAMBLE]), i)
        if i < 0 or i + 3 > len(buf):
            return i if i >= 0 else -1
        if not (buf[i + 1] & 0xFC):
            n = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
            if i + 6 + n > len(buf):
                return i
        i += 1


class _Bits:
    def __init__(self, data: bytes):
        self.v = int.from_bytes(data, "big")
        self.n = len(data) * 8
        self.p = 0

    def u(self, k: int) -> int:
        self.p += k
        return (self.v >> (self.n - self.p)) & ((1 << k) - 1)

    def s(self, k: int) -> int:
        x = self.u(k)
        return x - (1 << k) if x & (1 << (k - 1)) else x


def message_type(body: bytes) -> int | None:
    return (body[0] << 4 | body[1] >> 4) if len(body) >= 2 else None


# DF407 lock-time indicator -> minimum lock time in ms (RTCM 10403.3, DF407):
# 0-63 map one to one, then 32-wide bands each doubling the step, continuous at
# every band edge (64 -> 64, 96 -> 128, 128 -> 256 ...), saturating at 704.
def lock_time_ms(i: int) -> int:
    if i < 64:
        return i
    if i >= 704:
        return 67108864
    b = (i - 64) // 32
    return 2 ** (b + 1) * (i - (64 + 32 * b)) + 2 ** (6 + b)


def parse_msm7(body: bytes):
    """Decode an MSM7 message into (constellation, epoch_ms, [cells]).

    Each cell: dict(prn, sig, cn0, rate_mps, doppler_hz, lock_ms, halfcyc).
    The epoch field is GPS time of week in ms for GPS/Galileo/QZSS (BDS counts
    from its own epoch, GLONASS packs a day number); only GPS is injected here.
    Returns None for anything that is not an MSM7.
    """
    mt = message_type(body)
    if mt not in MSM7_TYPES:
        return None
    b = _Bits(body)
    b.u(12)                       # DF002 message number
    b.u(12)                       # DF003 station
    epoch = b.u(30)               # GNSS epoch time
    b.u(1); b.u(3); b.u(7); b.u(2); b.u(2); b.u(1); b.u(3)
    sat_mask = b.u(64)
    sig_mask = b.u(32)
    sats = [i + 1 for i in range(64) if sat_mask >> (63 - i) & 1]
    sigs = [i + 1 for i in range(32) if sig_mask >> (31 - i) & 1]
    ns, ng = len(sats), len(sigs)
    cells = [(s, g) for s in sats for g in sigs if b.u(1)]
    nc = len(cells)

    rough_ms = [b.u(8) for _ in range(ns)]           # DF397
    [b.u(4) for _ in range(ns)]                      # DF419 ext info
    [b.u(10) for _ in range(ns)]                     # DF398
    rough_rate = [b.s(14) for _ in range(ns)]        # DF399, m/s
    [b.s(20) for _ in range(nc)]                     # DF405 fine pseudorange
    [b.s(24) for _ in range(nc)]                     # DF406 fine phase-range
    lock = [b.u(10) for _ in range(nc)]              # DF407
    half = [b.u(1) for _ in range(nc)]               # DF420
    cnr = [b.u(10) for _ in range(nc)]               # DF408, 2^-4 dB-Hz
    fine_rate = [b.s(15) for _ in range(nc)]         # DF404, 0.0001 m/s

    idx = {s: k for k, s in enumerate(sats)}
    out = []
    for c, (s, g) in enumerate(cells):
        k = idx[s]
        valid_rate = rough_rate[k] != -8192 and fine_rate[c] != -16384
        rate = rough_rate[k] + fine_rate[c] * 1e-4 if valid_rate else None
        out.append(dict(
            prn=s, sig=g, cn0=cnr[c] / 16.0 if cnr[c] else None,
            rate_mps=rate,
            doppler_hz=(-rate / L1_WAVELENGTH_M) if rate is not None else None,
            lock_ms=lock_time_ms(lock[c]), halfcyc=half[c],
            rough_ms=rough_ms[k]))
    return MSM7_TYPES[mt], epoch, out
