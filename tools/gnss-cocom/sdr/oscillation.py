#!/usr/bin/env python3
"""Characterise the periodic C/N0 oscillation in a capture.

The bench carries a slow, large swing in reported C/N0 that is not the
trajectory, not acceleration, not clipping and not the injection level -- see the
report. Every satellite-count dip in every flight sits in one of its troughs, so
it is the dominant confound on this rig and worth a number rather than an
adjective.

This exists so the question can be settled by an A/B rather than by argument.
Mayhem's `config_disable_external_tcxo()` forces the HackRF's own crystal in
place of the PortaPack TCXO; if the period or depth moves when that is flipped,
the oscillation is clock-related after all. If they do not move, the clock is
cleared and the remaining suspects are the receiver's AGC or C/N0 estimator.

Period is estimated by autocorrelation of the detrended series rather than by
eye. The series is median C/N0 across tracked satellites, which is robust to one
satellite rising or setting mid-run in a way a mean is not.
"""

from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from gnss_nmea_monitor import replay_source                    # noqa: E402
from skytraq_binary import (MSG_RCV_STATE, MSG_SV_CH_STATUS,   # noqa: E402
                            parse_sv_ch_status)
from ublox_binary import (CLS_NAV, MSG_NAV_PVT, MSG_NAV_SAT,   # noqa: E402
                          parse_nav_sat)


def series(cap: Path):
    """(t, median C/N0, n>=30) per epoch, from either receiver's frames."""
    sats: dict = {}
    out = []
    t0 = None
    for t, kind, data in replay_source(str(cap)):
        if kind == "bin":
            if data and data[0] == MSG_SV_CH_STATUS:
                ss = parse_sv_ch_status(data) or []
                best = {}
                for x in ss:
                    k = (x["gnss_name"], x["svid"])
                    best[k] = max(best.get(k, -99), x["cn0"])
                sats = best
            elif data and data[0] == MSG_RCV_STATE:
                vals = [v for v in sats.values() if v > 0]
                if t0 is None:
                    t0 = t
                out.append((t - t0, statistics.median(vals) if vals else 0.0,
                            sum(1 for v in vals if v >= 30)))
        elif kind == "ubx" and len(data) >= 2 and data[0] == CLS_NAV:
            if data[1] == MSG_NAV_SAT:
                ss = parse_nav_sat(data[2:]) or []
                best = {}
                for x in ss:
                    k = (x["gnss_name"], x["svid"])
                    best[k] = max(best.get(k, -99), x["cn0"])
                sats = best
            elif data[1] == MSG_NAV_PVT:
                vals = [v for v in sats.values() if v > 0]
                if t0 is None:
                    t0 = t
                out.append((t - t0, statistics.median(vals) if vals else 0.0,
                            sum(1 for v in vals if v >= 30)))
    return out


def estimate_period(times, values, lo=15.0, hi=200.0, min_cycles=2.5,
                    min_r=0.20):
    """Autocorrelation peak of the detrended series, in seconds.

    Returns (None, reason) when the record cannot support an estimate. A run
    covering less than a couple of cycles will still produce an autocorrelation
    maximum somewhere, and it will be meaningless -- a 74-epoch capture of an
    80 s oscillation reported 14 s at r=0.08 before this guard existed.
    """
    if len(values) < 40:
        return None, 0.0
    mean = sum(values) / len(values)
    x = [v - mean for v in values]
    dt = (times[-1] - times[0]) / max(1, len(times) - 1)
    energy = sum(v * v for v in x) or 1.0

    span = times[-1] - times[0]
    hi = min(hi, span / min_cycles)          # need several cycles to be believed
    if hi <= lo:
        return None, 0.0

    # Compute the whole autocorrelation, then take the first prominent peak
    # AFTER the first minimum -- not the global maximum. At short lags the
    # series is merely smooth, and a plain maximum lands there: two 777-epoch
    # flight captures whose sparklines plainly show an ~80 s cycle both reported
    # 14 s that way, which is the autocorrelation of slow variation, not period.
    hi_lag = min(int(hi / dt), len(x) // 2)
    acf = []
    for lag in range(1, hi_lag):
        acf.append((lag, sum(x[i] * x[i + lag] for i in range(len(x) - lag)) / energy))
    if not acf:
        return None, 0.0

    first_min = 0
    for i in range(1, len(acf) - 1):
        if acf[i][1] <= acf[i - 1][1] and acf[i][1] < acf[i + 1][1]:
            first_min = i
            break
    if first_min == 0:                      # monotone: no cycle resolvable
        return None, 0.0

    lo_lag = max(first_min, int(lo / dt))
    best_lag, best_r = None, 0.0
    for lag, r in acf[lo_lag:]:
        if r > best_r:
            best_r, best_lag = r, lag
    if best_lag is None or best_r < min_r:
        return None, best_r
    return best_lag * dt, best_r


def sparkline(values, lo, hi, width=64):
    blocks = "▁▂▃▄▅▆▇█"
    step = max(1, len(values) // width)
    out = []
    for i in range(0, len(values), step):
        v = values[i]
        f = 0.0 if hi <= lo else (v - lo) / (hi - lo)
        out.append(blocks[max(0, min(len(blocks) - 1, int(f * len(blocks))))])
    return "".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("captures", nargs="+", type=Path)
    ap.add_argument("--from-t", type=float, default=0.0,
                    help="ignore the first N seconds (acquisition)")
    ap.add_argument("--label", default="", help="note printed with the summary")
    args = ap.parse_args()

    if args.label:
        print(f"{args.label}\n")
    print(f"{'capture':<28} {'epochs':>7} {'median':>7} {'p10':>6} {'p90':>6} "
          f"{'swing':>7} {'period':>8} {'r':>6}")
    print("-" * 84)

    periods = []
    for cap in args.captures:
        if not cap.exists():
            print(f"{cap.name:<28}  missing")
            continue
        s = [r for r in series(cap) if r[0] >= args.from_t and r[1] > 0]
        if len(s) < 40:
            print(f"{cap.name:<28}  too few epochs ({len(s)})")
            continue
        t = [r[0] for r in s]
        v = [r[1] for r in s]
        vs = sorted(v)
        p10, p90 = vs[len(vs) // 10], vs[-len(vs) // 10 - 1]
        period, r = estimate_period(t, v)
        if period:
            periods.append(period)
        span = t[-1] - t[0]
        note = ""
        if period is None:
            note = ("  record spans only %.0fs -- too short for a ~80s cycle"
                    % span) if span < 200 else "  no periodicity above threshold"
        print(f"{cap.name:<28} {len(s):>7} {statistics.median(v):>7.0f} "
              f"{p10:>6.0f} {p90:>6.0f} {p90 - p10:>6.0f}dB "
              f"{(f'{period:.0f}s' if period else '--'):>8} {r:>6.2f}{note}")
        print(f"   {sparkline(v, min(v), max(v))}")

    if len(periods) > 1:
        print(f"\nacross {len(periods)} captures: period "
              f"{statistics.median(periods):.0f}s "
              f"(spread {min(periods):.0f}-{max(periods):.0f}s)")
    print("\nCompare against a run with the external TCXO disabled. A period or\n"
          "depth that moves means the clock is implicated; identical numbers\n"
          "clear it and point at the receiver.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
