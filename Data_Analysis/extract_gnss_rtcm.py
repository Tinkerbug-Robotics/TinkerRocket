#!/usr/bin/env python3
"""Pull the GNSS receiver's raw RTCM3 frames (GNSS_RTCM_MSG, 0x95) out of a mini log.

The flight computer logs every RTCM3 frame the receiver sends, verbatim (on the
mini: the LC86G's MSM7 raw measurements, one epoch a second). Joined in log
order they are a standard RTCM3 stream, so this writes exactly that:

    ./extract_gnss_rtcm.py flight_20261012_143000.bin
        -> flight_20261012_143000.rtcm3   (any RTCM3 tool reads it; RTKLIB's
                                           convbin turns it into RINEX)

    ./extract_gnss_rtcm.py flight.bin --csv
        -> also flight.msm7.csv, one row per satellite-signal per MSM7 epoch:
           the frame's FC time, GPS time of week, constellation, satellite,
           signal, pseudorange, carrier phase, Doppler (phase-range rate),
           C/N0, lock time, half-cycle flag, GLONASS channel

The summary also names each broadcast-ephemeris message ($PAIR436,1: 1019 GPS,
1020 GLONASS, 1042 BeiDou, 1044 QZSS, 1046 Galileo) by satellite, issue of data
and reference time, with sqrt(A) as a sanity check on the decode.

Every frame written passes its own CRC-24Q; a frame missing a part, or failing
its CRC, is left out and counted. On the V9 the same record carries the
mosaic-G5's SBF blocks instead: those are not RTCM3, so they are left out and
counted on a line of their own, not as bad CRCs (extract_gnss_sbf.py writes
them). Field references are RTCM 10403.3 (DFnnn).

On a V9 or V10 with TinkerRocket's gnss-sdr-dev board the record carries that
board's RTCM3 (MSM7 at up to 100 Hz, TinkerRocket's message 4001, its
ephemerides), and the same .rtcm3 comes out. The summary then also gives the
MSM7 epochs' step and any missing between them, and the 4001 sub-types:
1 after each epoch, 2 after each fix (RTKLIB passes 4001 over). The flight
computer logs beside them each frame it sent the board, TinkerRocket's message
4002: a flight image its flight phase (sub-type 3, about one a second and
three more after each change), its attitude (sub-type 1, ten a second) and its
rests (sub-type 5, still seconds, one every 5 s), a bench one
(TR_BENCH_SDR_AID) its P1 script. The summary counts those by sub-type (1
attitude, 2 navigation state, 3 phase, 4 geometry, 5 rest). They stay in the .rtcm3, which the board's rtcmdump reads; RTKLIB
passes 4002 over too.

The board's own IMU into its tracking loops (the aided-receiver plan's P2)
comes first in the summary when the epochs carry the aiding block's version 2
(the board's receiver/README.md, "The IMU's watch"): how long the IMU aided
the loops, and every time the board dropped it (when, in which of the flight
computer's phases, why, and how long until it aided again), or "never
dropped"; the pad check (the IMU's output at rest, where the truth is 0); and
the loops' own share, what the IMU missed as the GPS channels' loops saw it,
by phase. The board drops the IMU only when its data stop or cannot be read,
never because the GPS channels disagree with it, so a drop is always worth a
look:

    ./extract_gnss_rtcm.py flight.bin --aid-csv --aid-plot
        -> also flight.aid.csv, one row per epoch: the block's fields and the
           loops' own share north, east and down (m/s^2); and flight.aid.png,
           the share and what aided the loops against the log's time, the
           drops marked
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_flight_data_mini as pfd          # noqa: E402

LIGHT_MS_M = 299792458.0 / 1000.0            # MSM ranges are in light-milliseconds

# MSM7 message type -> (UBX gnssId as GNSSSatBlock numbers them, name)
MSM7_TYPES = {1077: (0, "GPS"), 1087: (6, "GLONASS"), 1097: (2, "Galileo"),
              1117: (5, "QZSS"), 1127: (3, "BeiDou")}


def crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def frame_ok(frame: bytes) -> bool:
    """A whole RTCM3 frame whose length and CRC-24Q check."""
    if len(frame) < 6 or frame[0] != 0xD3 or frame[1] & 0xFC:
        return False
    if len(frame) != ((frame[1] & 0x03) << 8 | frame[2]) + 6:
        return False
    return crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big")


def message_type(frame: bytes) -> int:
    return (frame[3] << 4) | (frame[4] >> 4)


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
        return x - (1 << k) if x >> (k - 1) else x


def lock_time_ms(i: int) -> int:
    """DF407 lock-time indicator -> minimum lock time, ms."""
    if i < 64:
        return i
    if i >= 704:
        return 67108864
    b = (i - 64) // 32
    return 2 ** (b + 1) * (i - (64 + 32 * b)) + 2 ** (6 + b)


def decode_msm7(frame: bytes):
    """(constellation, GPS TOW ms or None, [cell dicts]) for an MSM7 frame, else None.

    Invalid fields come back as None. GLONASS's epoch is UTC(SU) and needs the
    leap seconds, so its GPS TOW is None; BeiDou's is shifted by its 14 s.
    """
    mt = message_type(frame)
    if mt not in MSM7_TYPES:
        return None
    gnss_id, name = MSM7_TYPES[mt]
    b = _Bits(frame[3:-3])
    b.u(12); b.u(12)
    epoch = b.u(30)
    b.u(1); b.u(3); b.u(7); b.u(2); b.u(2); b.u(1); b.u(3)
    sat_mask, sig_mask = b.u(64), b.u(32)
    sats = [i + 1 for i in range(64) if sat_mask >> (63 - i) & 1]
    sigs = [i + 1 for i in range(32) if sig_mask >> (31 - i) & 1]
    cells = [(s, g) for s in sats for g in sigs if b.u(1)]
    ns, nc = len(sats), len(cells)
    rough_ms = [b.u(8) for _ in range(ns)]
    ext = [b.u(4) for _ in range(ns)]
    rough_mod = [b.u(10) for _ in range(ns)]
    rough_rate = [b.s(14) for _ in range(ns)]
    fine_pr = [b.s(20) for _ in range(nc)]
    fine_cp = [b.s(24) for _ in range(nc)]
    lock = [b.u(10) for _ in range(nc)]
    half = [b.u(1) for _ in range(nc)]
    cnr = [b.u(10) for _ in range(nc)]
    fine_rate = [b.s(15) for _ in range(nc)]

    if gnss_id == 6:
        tow = None
    elif gnss_id == 3:
        tow = (epoch + 14000) % 604800000
    else:
        tow = epoch
    k = {s: i for i, s in enumerate(sats)}
    out = []
    for c, (s, g) in enumerate(cells):
        i = k[s]
        rough = rough_ms[i] + rough_mod[i] / 1024.0 if rough_ms[i] != 255 else None
        out.append({
            "gnss_id": gnss_id, "sv_id": s, "sig_id": g,
            "pseudorange_m": None if rough is None or fine_pr[c] == -524288
                             else (rough + fine_pr[c] * 2.0 ** -29) * LIGHT_MS_M,
            "phase_range_m": None if rough is None or fine_cp[c] == -8388608
                             else (rough + fine_cp[c] * 2.0 ** -31) * LIGHT_MS_M,
            "rate_mps": None if rough_rate[i] == -8192 or fine_rate[c] == -16384
                        else rough_rate[i] + fine_rate[c] * 1e-4,
            "cn0_dbhz": cnr[c] / 16.0,
            "lock_ms": lock_time_ms(lock[c]),
            "half_cycle": half[c],
            "ext_info": ext[i],
        })
    return name, tow, out


# Broadcast ephemeris ($PAIR436,1). Where each message keeps the fields that say
# WHICH ephemeris it is, as (bit offset, width) after the 12-bit message number:
# satellite, issue of data, reference time (and its unit, s), sqrt(A) (2^-19).
EPH_LAYOUT = {
    #        name       sv        iod        toe (bits, scale s)  sqrtA
    1019: ("GPS",     (12, 6), (48, 8),  (288, 16, 16.0),  (256, 32)),   # DF009, DF071 IODE, DF093, DF092
    1044: ("QZSS",    (12, 4), (78, 8),  (246, 16, 16.0),  (214, 32)),   # DF429, DF434 IODE, DF442, DF441
    1046: ("Galileo", (12, 6), (30, 10), (294, 14, 60.0),  (262, 32)),   # DF252, DF290 IODnav, DF304, DF303
    1042: ("BeiDou",  (12, 6), (49, 5),  (299, 17, 8.0),   (267, 32)),   # DF488, DF492 AODE, DF505, DF504
    1020: ("GLONASS", (12, 6), None,     (41, 7, 900.0),   None),        # DF038 slot, DF110 tb (15 min)
}


def decode_eph(frame: bytes):
    """{constellation, sv, iod, toe_s, sqrt_a} for an ephemeris frame, else None.

    toe_s is the ephemeris reference time in the constellation's own time of
    week (GLONASS: tb, time of day, Moscow time); sqrt_a is None for GLONASS,
    whose message carries positions, not Keplerian elements.
    """
    mt = message_type(frame)
    if mt not in EPH_LAYOUT:
        return None
    name, sv, iod, toe, sqa = EPH_LAYOUT[mt]
    v = int.from_bytes(frame[3:-3], "big")
    n = (len(frame) - 6) * 8

    def field(off, width):
        return (v >> (n - off - width)) & ((1 << width) - 1)

    return {"constellation": name, "sv": field(*sv),
            "iod": field(*iod) if iod else None,
            "toe_s": field(toe[0], toe[1]) * toe[2],
            "sqrt_a": field(*sqa) * 2.0 ** -19 if sqa else None}


TR_MESSAGE = 4001                             # TinkerRocket's own (the gnss-sdr-dev board's)
FC_MESSAGE = 4002                             # TinkerRocket's from the flight computer to the board
FC_SUBTYPES = {1: "attitude", 2: "state", 3: "phase", 4: "geometry", 5: "rest"}
WEEK_MS = 604800000


def msm_gps_tow(frame: bytes):
    """An MSM frame's epoch as GPS time of week, ms (BeiDou's 14 s added back); None for GLONASS or another message."""
    mt = message_type(frame)
    if mt not in MSM7_TYPES or MSM7_TYPES[mt][0] == 6:
        return None
    epoch = (int.from_bytes(frame[3:11], "big") >> (64 - 24 - 30)) & ((1 << 30) - 1)
    return (epoch + 14000) % WEEK_MS if mt == 1127 else epoch


TR_WHY = {0: "aiding", 1: "let go (turned off)", 2: "stale (none given for 30 ms)", 3: "a gap in its samples",
          4: "its attitude lapsed (no flight computer attitude)", 5: "not a number",
          6: "its sensor at the end of its range", 7: "the pad check failed"}
TR_PHASE = {0: "no phase", 1: "pad", 2: "boost", 3: "coast", 4: "descent", 5: "landed"}
TR_CHECK = {0: "not run", 1: "passing", 2: "FAILED (the IMU refused)"}
TR_SRC = {0: "nothing", 1: "the GPS channels", 2: "the IMU"}


def decode_epoch_aid(frame: bytes):
    """A 4001 epoch's (sub-type 1) aiding block as a dict, or None: another message, or an epoch without one.

    The block sits after as many 10-bit marks as the 6-bit count at payload bit
    208 says, at a = 214 + 10 * nmark: version, what aids the loops, the
    check, ..., 8 bits of the flight computer's frames (32 bits); version 2
    puts why the IMU does not aid in version 1's reserved bits and appends 56:
    its drops (mod 256), the last one's reason, the loops' own share's channels
    and north, east, down (10 bits signed, 0.2 m/s^2), the pad check's window
    (8 bits, 0.02 m/s^2). A frame cut short of the 56 keeps the first 32.
    """
    if message_type(frame) != TR_MESSAGE or len(frame) < 6:
        return None
    pay = frame[3:-3]
    nb = 8 * len(pay)
    v = int.from_bytes(pay, "big")

    def get(pos, n, signed=False):
        x = (v >> (nb - pos - n)) & ((1 << n) - 1)
        return x - (1 << n) if signed and x >> (n - 1) else x

    if nb < 214 or get(12, 4) != 1:
        return None
    at = 214 + 10 * get(208, 6)
    if nb < at + 32 or get(at, 4) == 0:
        return None
    d = {"week": get(16, 16), "tow_ms": get(32, 30), "ver": get(at, 4), "src": get(at + 4, 2),
         "check": get(at + 6, 2), "boost": get(at + 8, 1), "wide": get(at + 9, 1), "by": get(at + 10, 2),
         "planned": get(at + 12, 1), "phase": get(at + 13, 3), "att": get(at + 16, 1), "nav": get(at + 17, 1),
         "late": get(at + 18, 1), "fc_frames": get(at + 24, 8),
         "why": 0, "drops": None, "last": 0, "lo_n": 0, "lo": None, "pad": None}
    if d["ver"] >= 2:
        d["why"] = get(at + 19, 4)
    if d["ver"] >= 2 and nb >= at + 88:
        d["drops"], d["last"], d["lo_n"] = get(at + 32, 8), get(at + 40, 4), get(at + 44, 4)
        if d["lo_n"]:
            d["lo"] = tuple(0.2 * get(at + 48 + 10 * j, 10, signed=True) for j in range(3))
        d["pad"] = 0.02 * get(at + 78, 8)
    return d


def imu_watch(frames):
    """The board IMU's watch over a log's epochs: [(time_us, block)] with version 2's, the drops (each: time_us, GPS
    time of week ms, phase, reason, seconds until the IMU aided again or None), the pad check's turns, the loops' own
    share by phase (epochs, mean and largest |share|, m/s^2), the epochs with the IMU aiding, and the last block.
    Durations are the epochs' own GPS time (the log's stamps are when each frame arrived).

    The block's drop count runs from the board's receiver start, mod 256: the first epoch's count is the drops before
    the log ("before"), a count that goes back is the receiver started again (a restart: "restarts", the count taken
    from there), and an epoch whose block was cut short of it counts nothing."""
    rows = [(t, d) for t, f in frames if (d := decode_epoch_aid(f)) is not None and d["ver"] >= 2]
    drops, pad, share, restarts = [], [], {}, []
    prev = None
    imu = 0
    before = None
    last_drops = None
    for t, d in rows:
        aiding = d["src"] == 2
        imu += aiding
        if d["drops"] is not None:
            if last_drops is None:
                before = d["drops"]
            elif d["drops"] < last_drops and last_drops - d["drops"] < 128:
                # the count went back: the receiver started again (its count from 0, these since)
                restarts.append({"time_us": t, "tow_ms": d["tow_ms"], "lost": last_drops, "now": d["drops"]})
                for _ in range(d["drops"]):
                    drops.append({"time_us": t, "tow_ms": d["tow_ms"], "phase": d["phase"], "why": d["last"],
                                  "back_s": None})
            else:
                for _ in range((d["drops"] - last_drops) % 256):
                    drops.append({"time_us": t, "tow_ms": d["tow_ms"], "phase": d["phase"], "why": d["last"],
                                  "back_s": None})
            last_drops = d["drops"]
        if prev is not None:
            if d["check"] != prev["check"]:
                pad.append({"time_us": t, "tow_ms": d["tow_ms"], "phase": d["phase"], "check": d["check"],
                            "pad": d["pad"]})
        if aiding and drops and drops[-1]["back_s"] is None:
            for e in drops:
                if e["back_s"] is None:
                    e["back_s"] = ((d["tow_ms"] - e["tow_ms"]) % WEEK_MS) / 1e3
        if aiding and d["lo"] is not None:
            a = sum(x * x for x in d["lo"]) ** 0.5
            ph = share.setdefault(d["phase"], {"n": 0, "sum": 0.0, "max": 0.0, "t_max": t})
            ph["n"] += 1
            ph["sum"] += a
            if a > ph["max"]:
                ph["max"], ph["t_max"] = a, t
        prev = d
    return {"rows": rows, "drops": drops, "pad": pad, "share": share, "imu": imu, "before": before or 0,
            "restarts": restarts, "last": rows[-1][1] if rows else None}


def print_imu_watch(w, t0_us: int) -> None:
    """The watch's lines for the summary: a flight whose IMU was dropped says so on the first."""
    rows = w["rows"]
    if not rows:
        return
    span = ((rows[-1][1]["tow_ms"] - rows[0][1]["tow_ms"]) % WEEK_MS) / 1e3
    step = span / (len(rows) - 1) if len(rows) > 1 else 0.0
    span += step
    n = w["before"] + len(w["drops"])
    refused = any(d["check"] == 2 for _, d in rows)
    if n:
        before = f", {w['before']} of them before the log" if w["before"] else ""
        print(f"  IMU AIDING: DROPPED {n} TIME{'S' if n > 1 else ''}{before} (the board's IMU into its loops)")
    elif w["imu"] == 0 and refused:
        print("  IMU AIDING: REFUSED BY THE PAD CHECK, never aided (the board's IMU into its loops)")
    elif w["imu"] == 0:
        print("  IMU aiding: never aided (the board's IMU into its loops: off, or never given)")
    else:
        print("  IMU aiding: never dropped (the board's IMU into its loops)")
    last = w["last"]
    pad = f"{last['pad']:.2f} m/s^2" if last["pad"] is not None else "none"
    now = "aiding" if last["src"] == 2 else ("not aiding" if last["why"] == 0 else TR_WHY.get(last["why"], last["why"]))
    print(f"    aided {w['imu'] * step:.1f} s of {span:.1f} s of epochs ({w['imu']} of {len(rows)}); the pad check "
          f"{TR_CHECK.get(last['check'], last['check'])} (its last still window {pad}); at the end {now}")
    for e in w["restarts"]:
        print(f"    the board's receiver started again at {(e['time_us'] - t0_us) / 1e6:.3f} s into the log (its drop "
              f"count back from {e['lost']} to {e['now']}): drops counted from there")
    for k, e in enumerate(w["drops"], 1):
        back = f"back {e['back_s']:.2f} s later" if e["back_s"] is not None else "never back"
        print(f"    drop {k}: {(e['time_us'] - t0_us) / 1e6:.3f} s into the log (GPS {e['tow_ms'] / 1e3:.3f} s of "
              f"week), {TR_PHASE.get(e['phase'], e['phase'])}: {TR_WHY.get(e['why'], e['why'])}; {back}")
    for e in w["pad"]:
        win = f"{e['pad']:.2f} m/s^2" if e["pad"] is not None else "-"
        print(f"    the pad check {TR_CHECK.get(e['check'], e['check'])} at {(e['time_us'] - t0_us) / 1e6:.3f} s "
              f"({TR_PHASE.get(e['phase'], e['phase'])}; the window {win})")
    if w["share"]:
        parts = [f"{TR_PHASE.get(ph, ph)} {v['sum'] / v['n']:.2f} mean, {v['max']:.2f} most (at "
                 f"{(v['t_max'] - t0_us) / 1e6:.2f} s)" for ph, v in sorted(w["share"].items())]
        print("    the loops' own share while the IMU aided (m/s^2): " + "; ".join(parts))


def write_aid_csv(path: Path, w) -> int:
    with path.open("w", newline="") as fh:
        c = csv.writer(fh)
        c.writerow(["time_us", "gps_tow_ms", "ver", "src", "check", "why", "drops", "last", "phase", "boost",
                    "wide", "att", "nav", "late", "fc_frames", "lo_n", "lo_n_ms2", "lo_e_ms2", "lo_d_ms2",
                    "pad_ms2"])
        for t, d in w["rows"]:
            lo = ["", "", ""] if d["lo"] is None else [f"{x:.1f}" for x in d["lo"]]
            c.writerow([t, d["tow_ms"], d["ver"], d["src"], d["check"], d["why"], d["drops"], d["last"],
                        d["phase"], d["boost"], d["wide"], d["att"], d["nav"], d["late"], d["fc_frames"],
                        d["lo_n"], *lo, "" if d["pad"] is None else f"{d['pad']:.2f}"])
    return len(w["rows"])


def plot_aid(path: Path, w, t0_us: int, title: str) -> None:
    """The loops' own share north, east and down, and what aided the loops, against the epochs' GPS time from the
    first (linear axes); each drop a dashed line, the flight computer's boost shaded."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = w["rows"]
    tow0 = rows[0][1]["tow_ms"]
    t = [((d["tow_ms"] - tow0) % WEEK_MS) / 1e3 for _, d in rows]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    for j, (name, col) in enumerate((("north", "C0"), ("east", "C1"), ("down", "C2"))):
        ax1.plot(t, [float("nan") if d["lo"] is None else d["lo"][j] for _, d in rows], color=col, lw=1.0,
                 label=name)
    ax1.set_ylabel("the loops' own share (m/s$^2$)")
    ax1.set_title(f"{title}: what the board's IMU missed, as the GPS channels' loops saw it")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")
    ax2.step(t, [d["src"] for _, d in rows], where="post", color="0.25", lw=1.0)
    ax2.set_yticks([0, 1, 2], [TR_SRC[0], TR_SRC[1], TR_SRC[2]])
    ax2.set_ylabel("aided by")
    ax2.set_xlabel(f"GPS time from the first epoch (s; {tow0 / 1e3:.3f} s of week)")
    ax2.grid(True, alpha=0.3)
    for ax in (ax1, ax2):
        on = None
        for x, (_, d) in zip(t, rows):
            if d["phase"] == 2 and on is None:
                on = x
            elif d["phase"] != 2 and on is not None:
                ax.axvspan(on, x, color="0.85", lw=0)
                on = None
        if on is not None:
            ax.axvspan(on, t[-1], color="0.85", lw=0)
        for e in w["drops"]:
            ax.axvline(((e["tow_ms"] - tow0) % WEEK_MS) / 1e3, color="C3", ls="--", lw=1.0)
    note = "grey: the flight computer's boost"
    if w["drops"]:
        note += f"; {len(w['drops'])} drop(s): dashed"
    ax1.text(0.01, 0.95, note, transform=ax1.transAxes, color="C3" if w["drops"] else "0.3", va="top")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def epoch_summary(frames):
    """The MSM7 epochs in log order: how many, their usual step, and the ones missing between them.

    An epoch is a run of MSM7 frames with the same GPS time of week (the
    multiple-message bit ties them). Also TinkerRocket's 4001 by sub-type, and
    the 4002 frames this flight computer sent the board, by sub-type.
    """
    tows = []
    tr = Counter()
    fc = Counter()
    fixes_valid = 0
    for _, f in frames:
        tow = msm_gps_tow(f)
        if tow is not None and (not tows or tows[-1] != tow):
            tows.append(tow)
        if message_type(f) == TR_MESSAGE and len(f) > 6:
            sub = f[4] & 0x0F
            tr[sub] += 1
            if sub == 2 and len(f) >= 3 + 11 + 3:
                fixes_valid += (f[3 + 10] >> 5) & 1      # bit 82 of the payload
        elif message_type(f) == FC_MESSAGE and len(f) > 6:
            fc[f[4] & 0x0F] += 1
    steps = [(b - a) % WEEK_MS for a, b in zip(tows, tows[1:])]
    forward = [d for d in steps if 0 < d < WEEK_MS // 2]
    step = Counter(forward).most_common(1)[0][0] if forward else None
    missing = sum(-(-d // step) - 1 for d in forward if d > step) if step else 0
    return {"epochs": len(tows), "step_ms": step, "missing": missing,
            "backward": len(steps) - len(forward), "tr_epochs": tr[1], "tr_fixes": tr[2],
            "tr_fixes_valid": fixes_valid, "fc_sent": dict(fc)}


def extract(bin_path: Path):
    """([(time_us, frame)], stats) for every good frame in a log, in log order."""
    records, _stats, _config = pfd.parse_binary_file(str(bin_path))
    joined, dropped = pfd.join_gnss_rtcm(records["GNSS_RTCM"])
    good = [(t, f) for t, f in joined if frame_ok(f)]
    sbf = sum(1 for _, f in joined if f[:2] == pfd.SBF_SYNC)   # a mosaic-G5's, not bad RTCM3
    return good, {"records": len(records["GNSS_RTCM"]), "frames": len(joined),
                  "dropped_parts": dropped, "bad_crc": len(joined) - len(good) - sbf,
                  "sbf_blocks": sbf}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path, help="mini .bin flight log")
    ap.add_argument("-o", "--out", type=Path, help="output .rtcm3 (default: beside the log)")
    ap.add_argument("--csv", action="store_true", help="also write the decoded MSM7 as .msm7.csv")
    ap.add_argument("--aid-csv", action="store_true",
                    help="also write the gnss-sdr-dev board's aiding block, one row an epoch, as .aid.csv")
    ap.add_argument("--aid-plot", action="store_true",
                    help="also plot the board IMU's watch (the loops' own share, what aided them) as .aid.png")
    args = ap.parse_args(argv)

    frames, st = extract(args.log)
    out = args.out or args.log.with_suffix(".rtcm3")
    out.write_bytes(b"".join(f for _, f in frames))

    types = Counter(message_type(f) for _, f in frames)
    print(f"{args.log.name}: {st['records']} records -> {st['frames']} frames "
          f"({st['dropped_parts']} incomplete, {st['bad_crc']} bad CRC)")
    if st["sbf_blocks"]:
        print(f"  {st['sbf_blocks']} SBF blocks (mosaic-G5), not RTCM3: see extract_gnss_sbf.py")
    print(f"  wrote {out} ({out.stat().st_size} bytes)")
    watch = imu_watch(frames)
    t0 = frames[0][0] if frames else 0
    print_imu_watch(watch, t0)
    print("  message types: " + (", ".join(f"{t} x{n}" for t, n in sorted(types.items())) or "none"))
    if frames:
        span = (frames[-1][0] - frames[0][0]) / 1e6
        print(f"  FC time span {span:.1f} s")
    es = epoch_summary(frames)
    if es["epochs"]:
        rate = f" ({1000.0 / es['step_ms']:.1f} Hz)" if es["step_ms"] else ""
        print(f"  MSM7 epochs: {es['epochs']} at a {es['step_ms']} ms step{rate}, "
              f"{es['missing']} missing, {es['backward']} out of order")
    if es["tr_epochs"] or es["tr_fixes"]:
        print(f"  TinkerRocket 4001 (gnss-sdr-dev): {es['tr_epochs']} epochs, "
              f"{es['tr_fixes']} fixes ({es['tr_fixes_valid']} valid)")
    if es["fc_sent"]:
        sent = ", ".join(f"{n} {FC_SUBTYPES.get(sub, f'sub-type {sub}')}" for sub, n in sorted(es["fc_sent"].items()))
        print(f"  TinkerRocket 4002 (this flight computer to the gnss-sdr-dev board): {sent}")
    seen = {}
    for t, f in frames:
        e = decode_eph(f)
        if e is not None:
            seen.setdefault((e["constellation"], e["sv"]), []).append((t, e))
    if seen:
        print("  ephemeris (satellite: messages, median repeat, issue of data, toe, sqrt(A)):")
        for (name, sv), msgs in sorted(seen.items()):
            ts = [t for t, _ in msgs]
            gaps = sorted(b - a for a, b in zip(ts, ts[1:]))
            rep = f"{gaps[len(gaps) // 2] / 1e6:5.1f} s" if gaps else "    -  "
            last = msgs[-1][1]
            sqa = f"{last['sqrt_a']:.3f}" if last["sqrt_a"] is not None else "-"
            print(f"    {name:<8s}{sv:3d}: {len(msgs):3d}, {rep}, iod {last['iod']}, "
                  f"toe {last['toe_s']:.0f} s, sqrtA {sqa}")

    if args.csv:
        path = out.with_suffix(".msm7.csv")
        n = 0
        with path.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["time_us", "gps_tow_ms", "constellation", "gnss_id", "sv_id", "sig_id",
                        "pseudorange_m", "phase_range_m", "rate_mps", "cn0_dbhz",
                        "lock_ms", "half_cycle", "ext_info"])
            for t, f in frames:
                d = decode_msm7(f)
                if d is None:
                    continue
                name, tow, cells = d
                for c in cells:
                    w.writerow([t, "" if tow is None else tow, name, c["gnss_id"], c["sv_id"],
                                c["sig_id"],
                                *("" if c[k] is None else f"{c[k]:.4f}"
                                  for k in ("pseudorange_m", "phase_range_m", "rate_mps")),
                                c["cn0_dbhz"], c["lock_ms"], c["half_cycle"], c["ext_info"]])
                    n += 1
        print(f"  wrote {path} ({n} satellite-signal rows)")
    if args.aid_csv and watch["rows"]:
        path = out.with_suffix(".aid.csv")
        print(f"  wrote {path} ({write_aid_csv(path, watch)} epochs)")
    if args.aid_plot and watch["rows"]:
        path = out.with_suffix(".aid.png")
        plot_aid(path, watch, t0, args.log.name)
        print(f"  wrote {path}")
    if (args.aid_csv or args.aid_plot) and not watch["rows"]:
        print("  no aiding block of version 2 in the log: no .aid.csv or .aid.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
