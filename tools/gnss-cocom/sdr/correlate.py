#!/usr/bin/env python3
"""Correlate a receiver capture against the trajectory that was injected (#491).

This is the step that turns "the fix went away" into a number: the altitude and
3-D speed the receiver was being *told* it had at the instant it stopped
reporting a position.

Works on either wire format, because `gnss_nmea_monitor.py` captures either --
and on this bench board it is SkyTraq binary, since its firmware
(`SKYTRAQ03.00.01,01.07.33`) ignores every documented NMEA-output setting.

Timing is taken from the signal, never the wall clock:

* **Binary** (preferred): the `0xDF` receiver-state frame carries GPS week and
  time-of-week directly. gps-sdr-sim's `-t` maps to the same week/TOW pair with
  no leap-second offset -- verified against its own printout -- so trajectory
  time is exact integer arithmetic with no string parsing and no UTC ambiguity.
* **NMEA**: GGA/RMC report UTC, so trajectory time is `UTC - scenario start`.
* **Neither**: fall back to the host's monotonic timestamps, offset-fitted
  against every epoch that did carry a receiver clock. The spread of that fit is
  reported; a wide spread means a playback underrun and the run should be redone.

The FIX / BLOCKED / NO_LOCK classifier is imported from the receiver half rather
than reimplemented, so there is exactly one definition of "blocked" in the rig.

Usage:
  ./correlate.py -s scenarios/t1_velramp.json -t 2026/08/19,22:30:00 capture.log
"""

from __future__ import annotations

import argparse
import bisect
import datetime as dt
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
try:
    from gnss_nmea_monitor import Parser, replay_source
    from skytraq_binary import MSG_RCV_STATE, parse_rcv_state
    from ublox_binary import CLS_NAV, MSG_NAV_PVT, parse_nav_pvt
except ImportError as exc:  # pragma: no cover - depends on the receiver half
    raise SystemExit(
        f"cannot import the receiver half from the parent directory ({exc}).\n"
        "correlate.py deliberately shares its FIX/BLOCKED/NO_LOCK classifier and\n"
        "SkyTraq parser instead of carrying second copies of them.")

SECONDS_IN_WEEK = 604800
GPS_EPOCH = dt.datetime(1980, 1, 6)


def to_gps(when: dt.datetime):
    """UTC datetime -> (week, TOW), matching gps-sdr-sim's date2gps.

    No leap-second offset, because gps-sdr-sim applies none: it prints
    2026/08/19,22:30:00 as 2432:340200, which is this arithmetic exactly. The
    receiver is reading the clock out of the signal gps-sdr-sim built, so the
    two agree by construction.
    """
    delta = when - GPS_EPOCH
    total = delta.days * 86400 + delta.seconds
    return total // SECONDS_IN_WEEK, total % SECONDS_IN_WEEK


def parse_start(start: str):
    """'YYYY/MM/DD,hh:mm:ss' -> (week, tow, seconds-of-day)."""
    try:
        date, clock = start.split(",")
        y, mo, d = (int(x) for x in date.split("/"))
        h, mi, s = (int(float(x)) for x in clock.split(":"))
    except ValueError:
        raise SystemExit(f"bad -t value {start!r}; want YYYY/MM/DD,hh:mm:ss")
    when = dt.datetime(y, mo, d, h, mi, s)
    week, tow = to_gps(when)
    return week, tow, h * 3600 + mi * 60 + s


def utc_to_sod(utc: str):
    """NMEA 'hhmmss.ss' -> seconds of day, or None if the field is empty."""
    utc = (utc or "").strip()
    if len(utc) < 6:
        return None
    try:
        return int(utc[0:2]) * 3600 + int(utc[2:4]) * 60 + float(utc[4:])
    except ValueError:
        return None


class Truth:
    """Ground truth lookup: trajectory time -> what was injected."""

    def __init__(self, meta: dict):
        self.meta = meta
        self.samples = meta["truth"]
        self.times = [s["t"] for s in self.samples]
        self.duration = meta["duration_s"]

    def at(self, t: float):
        if t < self.times[0] - 1.0 or t > self.times[-1] + 1.0:
            return None
        i = bisect.bisect_left(self.times, t)
        if i == 0:
            return self.samples[0]
        if i >= len(self.samples):
            return self.samples[-1]
        lo, hi = self.samples[i - 1], self.samples[i]
        return lo if (t - lo["t"]) <= (hi["t"] - t) else hi


def collect(log_path: Path):
    """One sample per epoch: (host_t, gps_wt, utc_sod, verdict, snapshot, wire).

    An epoch is closed out when the *next* epoch marker arrives -- `0xDF` in
    binary, GGA in NMEA -- not when its own does. Both markers lead their
    burst, so the satellite frames that complete the same epoch have not been
    parsed yet at that point. Sampling on the marker itself pairs each solution
    with the *previous* epoch's C/N0, reporting every receiver column one epoch
    late: invisible in a static run, and squarely on the answer in a ramp.
    """
    parser = Parser()
    out = []
    pending = None

    for host_t, kind, data in replay_source(str(log_path)):
        if kind == "bin":
            marker = bool(data) and data[0] == MSG_RCV_STATE
        elif kind == "ubx":
            # NAV-PVT is the u-blox analogue of SkyTraq's 0xDF and, like it,
            # trails the burst -- NAV-SAT for the same epoch arrives first.
            marker = len(data) >= 2 and data[0] == CLS_NAV and data[1] == MSG_NAV_PVT
        else:
            data = data.strip()
            marker = data[3:6] == "GGA"

        # The two wire formats order their burst oppositely, and getting this
        # backwards silently pairs each solution with the wrong epoch's
        # satellites -- which on this bench made a clean gate closure (7 sats
        # held, used_in_fix 7 -> 0) read as a signal collapse to 3 satellites.
        #   NMEA:   GGA leads, GSA/GSV follow -> close the epoch at the NEXT GGA
        #   binary: 0xE7 leads, 0xDF trails   -> close the epoch AT the 0xDF
        if marker and kind != "bin" and pending is not None:
            e = parser.epoch
            out.append((*pending, e.verdict(), e.snapshot()))

        if kind == "bin":
            parser.feed_binary(data)
        elif kind == "ubx":
            parser.feed_ubx(data)
        else:
            parser.feed(data)

        if marker:
            if kind in ("bin", "ubx"):
                gps_wt = None
                if kind == "bin":
                    r = parse_rcv_state(data)
                    gps_wt = (r["week"], r["tow"]) if r else None
                else:
                    r = parse_nav_pvt(data[2:])
                    if r:
                        if r["utc_valid"]:
                            y, mo, d, hh, mi, ss = r["utc"]
                            gps_wt = to_gps(dt.datetime(y, mo, d, hh, mi, ss))
                        else:
                            # NAV-PVT carries no week number, so with the date
                            # unresolved the only anchor left is the scenario's
                            # own week. Correct for any run shorter than a week,
                            # which every scenario here is.
                            gps_wt = (None, r["itow_ms"] / 1000.0)
                e = parser.epoch
                out.append((host_t, gps_wt, utc_to_sod(e.utc),
                            "bin" if kind == "bin" else "ubx",
                            e.verdict(), e.snapshot()))
                pending = None
            else:
                pending = (host_t, None, utc_to_sod(parser.epoch.utc), "nmea")

    if pending is not None:       # NMEA only; binary epochs close inline
        e = parser.epoch
        out.append((*pending, e.verdict(), e.snapshot()))
    return out


def trajectory_time(sample, start, k):
    """Map one epoch onto trajectory time. Returns (t, source-of-clock)."""
    host_t, gps_wt, sod, _wire, _verdict, _e = sample
    w0, tow0, sod0 = start

    if gps_wt is not None:
        week, tow = gps_wt
        if week is None:
            week = w0
        return (week - w0) * SECONDS_IN_WEEK + (tow - tow0), "gps"

    if sod is not None:
        t = sod - sod0
        if t < -43200:            # scenario crossed midnight UTC
            t += 86400
        return t, "utc"

    return (host_t + k if k is not None else host_t), "host"


def fmt(v, unit="", nd=1):
    return "  --  " if v is None else f"{v:.{nd}f}{unit}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", type=Path, help="log from gnss_nmea_monitor.py --log")
    ap.add_argument("-s", "--scenario", required=True, type=Path,
                    help="scenario .json from make_trajectories.py")
    ap.add_argument("-t", "--start", required=True,
                    help="gps-sdr-sim scenario start, YYYY/MM/DD,hh:mm:ss")
    ap.add_argument("--all", action="store_true",
                    help="print every epoch, not just transitions")
    args = ap.parse_args()

    meta = json.loads(args.scenario.read_text())
    truth = Truth(meta)
    samples = collect(args.capture)
    if not samples:
        raise SystemExit(f"no receiver epochs in {args.capture} "
                         "(no 0xDF frames and no GGA sentences)")

    start = parse_start(args.start)

    # Fit host-clock offset from every epoch that carried a receiver clock, so
    # epochs that carried none can still be placed.
    offsets = []
    for s in samples:
        t, src = trajectory_time(s, start, None)
        if src != "host":
            offsets.append(t - s[0])
    k = statistics.median(offsets) if offsets else None
    spread = (max(offsets) - min(offsets)) if offsets else None

    wires = {s[3] for s in samples}
    print(f"scenario  : {truth.meta['scenario']}  ({truth.duration:.0f} s, "
          f"exceeds {truth.meta['exceeds']})")
    print(f"capture   : {args.capture}  ({len(samples)} epochs, "
          f"{'/'.join(sorted(wires))})")

    if k is None:
        raise SystemExit(
            "no epoch carried a receiver clock, so the capture cannot be tied "
            "to the trajectory.\nThe receiver never locked -- check the "
            "injection level before reading anything into this run.")

    n_host = sum(1 for s_ in samples
                 if trajectory_time(s_, start, None)[1] == "host")
    print(f"clock fit : traj_t = host_t + {k:.2f}s   (spread {spread:.2f}s, "
          f"{n_host} epoch(s) needed it)")
    if spread > 2.0 and n_host:
        print("  WARNING: the receiver-to-host offset moved by more than 2 s "
              "across the run.\n           Suspect a playback underrun or a "
              "receiver clock step; treat the\n           thresholds below as "
              "approximate.")

    rows = []
    for s in samples:
        t, src = trajectory_time(s, start, k)
        rows.append((t, src, s[4], s[5], truth.at(t)))

    # Stale-solution guard.
    #
    # A receiver that has lost the signal may keep republishing its last
    # solution with the fix flag still set: nav_state reads FIX_3D, lat/lon/alt
    # are present, and the classifier scores it FIX. Seen on this bench holding
    # a frozen 4.99 km while the injected trajectory climbed to 85 km.
    #
    # The test is divergence from ground truth, not a frozen position. A frozen
    # position is normal and correct whenever the trajectory is stationary, and
    # at low speed the reported position moves so little that a naive
    # freeze-detector fires on perfectly good runs -- it did, on a run whose
    # reported speed tracked the injected ramp to within 5 m/s on 7 satellites.
    # Since the injected altitude is known exactly, compare against it.
    ALT_TOL_M = 2000.0
    stale = [False] * len(rows)
    for i, (t, src, verdict, e, tr) in enumerate(rows):
        if verdict != "FIX" or tr is None or e.alt_m is None:
            continue
        if abs(e.alt_m - tr["alt_m"]) > ALT_TOL_M:
            stale[i] = True
    n_stale = sum(stale)
    if n_stale:
        first = next(i for i, x in enumerate(stale) if x)
        r = rows[first]
        print(f"\n  WARNING: {n_stale} epoch(s) claim a fix whose altitude has "
              f"diverged from the\n           injected trajectory by more than "
              f"{ALT_TOL_M/1000:.0f} km -- first at t={r[0]:.1f}s, reporting "
              f"{r[3].alt_m/1000:.2f} km\n           against an injected "
              f"{r[4]['alt_m']/1000:.2f} km. That is a stale solution being\n"
              f"           republished, not a live fix. Do not read a gate "
              f"result out of this run.")

    # Drop epochs past the end of the scenario. The capture deliberately runs a
    # little longer than the file so the last seconds are not clipped, but once
    # the transmitter stops the receiver loses lock -- an artifact of the capture
    # window, not a result. Left in, it reads as a signal failure and invalidates
    # any run whose gate never closes, which is exactly what the control run is.
    n_before = len(rows)
    rows = [r for r in rows if r[0] <= truth.duration + 0.5]
    if not rows:
        raise SystemExit("every epoch fell outside the scenario window; check -t")
    if n_before != len(rows):
        print(f"  ({n_before - len(rows)} epoch(s) after the scenario ended, "
              f"dropped)")

    transitions = [(i, rows[i]) for i in range(1, len(rows))
                   if rows[i][2] != rows[i - 1][2]]

    print(f"\n{'t_traj':>8} {'clk':>4} {'verdict':<8} {'inj alt':>10} "
          f"{'inj spd':>9} {'rx alt':>9} {'rx spd':>8} {'sats':>5} {'C/N0':>6} "
          f"{'nav':<8}")
    print("-" * 88)

    def line(t, src, verdict, e, tr):
        print(f"{t:>8.1f} {src:>4} {verdict:<8} "
              f"{fmt(tr['alt_m'] / 1000 if tr else None, 'km', 2):>10} "
              f"{fmt(tr['speed_mps'] if tr else None, '', 0):>9} "
              f"{fmt(e.alt_m / 1000 if e.alt_m is not None else None, '', 2):>9} "
              f"{fmt(e.speed_mps, '', 0):>8} "
              f"{e.tracked_sats():>5} {fmt(e.mean_cn0(), '', 1):>6} "
              f"{(e.nav_state or '-'):<8}")

    if args.all:
        for r in rows:
            line(*r)
    else:
        line(*rows[0])
        for i, r in transitions:
            prev = rows[i - 1]
            print(f"{'':>8} {'':>4} --- {prev[2]} -> {r[2]} ---")
            line(*prev)
            line(*r)
        line(*rows[-1])

    # --- verdict -----------------------------------------------------------
    print()
    ever_fixed = any(r[2] == "FIX" for r in rows)
    # The gate closure is the transition the receiver never recovers from.
    #
    # Two weaker rules were tried and both misreport. "First BLOCKED anywhere"
    # catches the innocent pre-acquisition BLOCKED, where satellites are tracked
    # but no position has been computed yet. "First BLOCKED after any FIX"
    # catches transient drop-outs, which a marginal RF path produces throughout
    # the stationary prologue -- on this bench that reported the threshold as
    # 0 m/s at t=78 s while the real closure sat at t=292 s.
    #
    # A COCOM gate does not flicker: once the trajectory is past the limit the
    # position stays withheld. So anchor on the LAST epoch that held a fix.
    # A flight profile recovers by design and lands holding a fix, so "the last
    # epoch that held one" is the touchdown, not the gate -- which reported a
    # flight that blocked three times as never withheld at all. When the
    # scenario declares blocked windows, anchor on the first closure inside the
    # first of them instead; recovery.py handles the rest window by window.
    windows = meta.get("blocked_windows") or []
    last_fix_i = None
    if windows:
        w_start, w_end = windows[0]
        for i in range(1, len(rows)):
            if (rows[i - 1][2] == "FIX" and rows[i][2] != "FIX"
                    and w_start - 2.0 <= rows[i][0] <= w_end + 2.0):
                last_fix_i = i - 1
                break
    if last_fix_i is None:
        last_fix_i = max((i for i, r in enumerate(rows) if r[2] == "FIX"),
                         default=None)
    transient = sum(1 for i in range(1, len(rows))
                    if rows[i - 1][2] == "FIX" and rows[i][2] != "FIX") - 1
    first_block = None
    if last_fix_i is not None and last_fix_i + 1 < len(rows):
        nxt = rows[last_fix_i + 1]
        if nxt[2] == "BLOCKED":
            first_block = nxt

    if not ever_fixed:
        print("VERDICT: the receiver never produced a fix in this run. Nothing "
              "can be concluded\n         about the COCOM gate -- fix the "
              "injection level and re-run.")
        return 1

    if first_block is None:
        # A run that ends in NO_LOCK did not "hold a fix throughout" -- the
        # signal went away. Saying otherwise would report a lost link as
        # evidence that the gate never fires, which is the exact false
        # positive this rig is built to avoid.
        # Only a NO_LOCK that follows a FIX is a lost link. Before the first
        # fix the receiver is simply still acquiring, and treating that as a
        # signal failure throws away otherwise good runs -- the same mistake as
        # counting a pre-acquisition BLOCKED as the gate closing.
        lost, seen = None, False
        for r in rows:
            if r[2] == "FIX":
                seen = True
            elif r[2] == "NO_LOCK" and seen:
                lost = r
                break
        if lost is not None:
            tr = lost[4]
            where = (f" at {tr['alt_m'] / 1000:.2f} km, {tr['speed_mps']:.0f} m/s"
                     if tr else "")
            print(f"VERDICT: INCONCLUSIVE -- the receiver lost lock at "
                  f"t={lost[0]:.1f}s{where}.")
            print("         Satellites fell out with the position, so this is a "
                  "signal problem, not\n         a COCOM gate. Raise the "
                  "injection level or check the RF chain, then re-run.")
            return 1

        peak_a = truth.meta["crossings"]["peak_alt_m"] / 1000
        peak_v = truth.meta["crossings"]["peak_speed_mps"]
        print(f"VERDICT: position was never withheld. The receiver held a fix "
              f"to {peak_a:.1f} km\n         and {peak_v:.0f} m/s "
              f"({truth.meta['exceeds']} exceeded).")
        return 0

    last_fix = rows[last_fix_i]
    tr_b = first_block[4]
    print("VERDICT: position withheld while satellites stayed tracked -- the "
          "COCOM signature.")
    if transient > 0:
        kind = ("the first closure inside a declared blocked window"
                if (meta.get("blocked_windows") or [])
                else "the transition the receiver never came back from")
        print(f"         ({transient} other drop-out(s) in this run; the one "
              f"reported is\n          {kind}.)")
    if last_fix and last_fix[4] and tr_b:
        print(f"         last fix at  {last_fix[4]['alt_m'] / 1000:8.2f} km, "
              f"{last_fix[4]['speed_mps']:6.0f} m/s  (t={last_fix[0]:.1f}s)")
        print(f"         first block  {tr_b['alt_m'] / 1000:8.2f} km, "
              f"{tr_b['speed_mps']:6.0f} m/s  (t={first_block[0]:.1f}s)")
        print(f"         threshold brackets: altitude "
              f"{last_fix[4]['alt_m'] / 1000:.2f}-{tr_b['alt_m'] / 1000:.2f} km, "
              f"speed {last_fix[4]['speed_mps']:.0f}-{tr_b['speed_mps']:.0f} m/s")
    print(f"         run exceeded: {truth.meta['exceeds']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
