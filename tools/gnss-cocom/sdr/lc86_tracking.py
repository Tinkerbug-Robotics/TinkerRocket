#!/usr/bin/env python3
"""Carrier-lock and dropout metrics for LC86G captures (RTCM MSM7), bench and sky.

The LC86G's MSM7 carries, per satellite, C/N0 to 1/16 dB, its own Doppler, a
lock-time counter and a half-cycle-ambiguity flag. Three things are read from it:

  lock reset   the lock-time counter went backwards while the satellite stayed:
               the carrier loop lost phase and started again
  half-cycle   the flag that the carrier phase still has an unresolved half-cycle
               ambiguity; a loop that holds phase clears it within seconds
  dropout      a satellite that was measured, then was not

Most of these receivers' carrier is too strong to be the question, so every table
also reports the "strong" satellites alone (median C/N0 >= 38 dBHz in the window):
there a loop that is working holds phase, and a reset means the loop, not the sky.

    ./lc86_tracking.py aba    results/lc86g_aba_pad_static.log.gz
    ./lc86_tracking.py levels results/lc86g_levels_pad_static.log.gz \\
                              results/lc86g_levels_pad_static.schedule.json
    ./lc86_tracking.py modes  results/lc86g_mode{3,0,1,4,5,7}_pad_static.log.gz
    ./lc86_tracking.py sky    results/lc86g_sky_20260925.log.gz
    ./lc86_tracking.py overnight captures/lc86g_sky_drone.log --csv OUT.csv
    ./lc86_tracking.py boost  results/lc86g_20260926_smooth_run{1,2,3,4}_spaceshot.log.gz
    ./lc86_tracking.py ignition results/lc86g_20260926_smooth_run1_spaceshot.log.gz

MSM7 comes out once a second at every fix rate, so every "held" and every lock
reset here has one-second resolution whatever the module's navigation rate.

Everything runs on the capture's host clock, because GLONASS and BeiDou MSM7
epochs keep their own time scales. A bench capture also gets the offset to its
file's GPS time (the static files start at 2026/08/18 08:30:00), from $PQTMPVT.
"""
from __future__ import annotations

import argparse
import gzip
import json
import re
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import rtcm3                                                   # noqa: E402

TOW0 = 203400.0          # 2026/08/18 08:30:00 GPS, t = 0 of every static file
SYS_TAG = {"GPS": "G", "GLONASS": "R", "Galileo": "E", "QZSS": "J", "BDS": "C"}
GSV_TALK = {"GP": "G", "GL": "R", "GA": "E", "GB": "C", "GQ": "J", "BD": "C"}
MODE_NAME = {0: "Normal", 1: "Fitness", 3: "Balloon", 4: "Stationary", 5: "Drone",
             7: "Swimming"}
STRONG_DBHZ = 38.0


def _lines(path):
    op = gzip.open if str(path).endswith(".gz") else open
    with op(path, "rt", errors="replace") as fh:
        for raw in fh:
            ts, _, rest = raw.rstrip("\n").partition(" ")
            try:
                yield float(ts), rest
            except ValueError:
                continue


class Capture:
    """One capture, read once.

    sats[(tag, prn)] = [(t, cell)]   one signal per satellite, host time
    epochs[tag]      = [t]           every MSM7 epoch of that constellation
    pvt              = [(t, file_t, fixmode, used)]
    elev[(tag, prn)] = degrees       latest GSV elevation
    modes            = [(t, mode)]   the mode READ BACK after each $PAIR080
    colds            = [t]           every cold start sent
    """

    def __init__(self, path, start_mode=None, t1=None):
        self.path = Path(path)
        self.sats, self.epochs, self.pvt, self.elev = {}, {}, [], {}
        self.modes = [] if start_mode is None else [(float("-inf"), start_mode)]
        self.colds = []
        sent = None
        for t, rest in _lines(path):
            if t1 is not None and t > t1:
                break
            if rest.startswith("R "):
                try:
                    r = rtcm3.parse_msm7(bytes.fromhex(rest[2:]))
                except ValueError:
                    continue
                if not r:
                    continue
                tag = SYS_TAG.get(r[0], "?")
                self.epochs.setdefault(tag, []).append(t)
                seen = set()
                for c in r[2]:
                    if c["prn"] in seen:
                        continue
                    seen.add(c["prn"])
                    self.sats.setdefault((tag, c["prn"]), []).append((t, c))
            elif rest.startswith("$PQTMPVT"):
                f = rest.split("*")[0].split(",")
                try:
                    self.pvt.append((t, int(f[2]) / 1000.0 - TOW0,
                                     int(f[6] or 0), int(f[7] or 0)))
                except (ValueError, IndexError):
                    pass
            elif re.match(r"\$(G[PLABQ]|BD)GSV", rest):
                f = rest.split("*")[0].split(",")
                tag = GSV_TALK.get(f[0][1:3], "?")
                body = f[4:]
                if len(body) % 4 == 1:
                    body = body[:-1]
                for i in range(0, len(body) - 3, 4):
                    try:
                        sid = int(body[i])
                        if tag == "R" and sid > 64:     # NMEA GLONASS ids are slot + 64
                            sid -= 64
                        self.elev[(tag, sid)] = int(body[i + 1])
                    except ValueError:
                        pass
            elif rest.startswith("# host: sent $PAIR080"):
                sent = t
            elif rest.startswith("$PAIR081,") and sent is not None:
                try:
                    self.modes.append((sent, int(rest.split(",")[1].split("*")[0])))
                except ValueError:
                    pass
                sent = None
            elif rest.startswith("# host: sent $PAIR006"):
                self.colds.append(t)
        # host -> file time, from epochs that carry GPS time after the first fix
        offs = [ft - t for t, ft, fm, _ in self.pvt if fm >= 2]
        self.offset = st.median(offs[len(offs) // 4:]) if offs else None

    def mode_at(self, t):
        m = None
        for s, mm in self.modes:
            if s <= t:
                m = mm
        return m

    def first_fix(self, after=float("-inf")):
        return next((t for t, _ft, fm, _u in self.pvt if t >= after and fm >= 2), None)

    def end(self):
        return max(t for v in self.epochs.values() for t in v)


def metrics(cap: Capture, a: float, b: float) -> dict:
    """Tracking between host times a and b."""
    P = [p for p in cap.pvt if a <= p[0] < b]
    out = dict(a=a, b=b, mins=(b - a) / 60,
               fix=100 * sum(1 for p in P if p[2] >= 2) / len(P) if P else 0.0,
               used=st.median(p[3] for p in P) if P else 0)
    resets = drops = hc = n = 0
    s_resets = s_hc = s_n = 0
    s_mins = 0.0
    strong, cn = [], []
    for key, rows in cap.sats.items():
        R = [(t, c) for t, c in rows if a <= t < b]
        if not R:
            continue
        good = [c["cn0"] for _, c in R if c.get("cn0")]
        cn += good
        is_strong = len(R) >= 30 and good and st.median(good) >= STRONG_DBHZ
        if is_strong:
            strong.append(key)
            s_mins += (R[-1][0] - R[0][0]) / 60
        prev = last = None
        for t, c in R:
            if last is not None and t - last > 1.6:
                drops += 1
                prev = None
            if prev is not None and c["lock_ms"] < prev:
                resets += 1
                s_resets += is_strong
            prev, last = c["lock_ms"], t
            hc += c["halfcyc"]
            n += 1
            if is_strong:
                s_hc += c["halfcyc"]
                s_n += 1
    mins = out["mins"] or 1e-9
    out.update(cn0=st.median(cn) if cn else None, resets_min=resets / mins,
               halfcyc=100 * hc / n if n else None, drops_min=drops / mins,
               strong=strong,
               s_resets=s_resets / s_mins if s_mins else None,
               s_halfcyc=100 * s_hc / s_n if s_n else None)
    return out


def row(name, m, ttff=None):
    f = lambda v, s: "     -" if v is None else s.format(v)
    return (f"{name:<24}{('' if ttff is None else f'{ttff:5.0f}s'):>6} {m['fix']:5.1f}% "
            f"{m['used']:5.1f} {f(m['cn0'], '{:5.1f}')} {len(m['strong']):4d} "
            f"{f(m['s_resets'], '{:9.2f}')} {f(m['s_halfcyc'], '{:7.1f}')}%  "
            f"{m['resets_min']:7.1f} {f(m['halfcyc'], '{:6.1f}')}% {m['drops_min']:7.1f}")


HEAD = (f"{'window':<24}{'TTFF':>6} {'fix':>6} {'used':>5} {'C/N0':>5} {'strng':>4} "
        f"{'resets/':>9} {'half-':>7}   {'resets':>7} {'half-':>6}  {'drops':>7}\n"
        f"{'':<24}{'':>6} {'':>6} {'':>5} {'':>5} {'sats':>4} "
        f"{'sat-min':>9} {'cycle':>7}   {'/min':>7} {'cycle':>6}  {'/min':>7}")


def cmd_aba(a):
    cap = Capture(a.capture, start_mode=3)
    ff = cap.first_fix()
    marks = [(ff + 30, cap.mode_at(ff))] + [(t + 5, m) for t, m in cap.modes if t > ff]
    print(HEAD)
    for i, (t, m) in enumerate(marks):
        u = marks[i + 1][0] - 5 if i + 1 < len(marks) else cap.end()
        print(row(f"{MODE_NAME.get(m, m)} {t + cap.offset:4.0f}-{u + cap.offset:4.0f} s",
                  metrics(cap, t, u)))


def cmd_levels(a):
    cap = Capture(a.capture, start_mode=3)
    sched = json.loads(Path(a.schedule).read_text())["segments"]
    print(HEAD)
    for s in sched:
        t0 = s["t0"] + (10 if s["t0"] > 0 else 60)
        m = metrics(cap, t0 - cap.offset, s["t1"] - cap.offset)
        print(row(f"{s['db']:+4.0f} dB  {s['t0']:4.0f}-{s['t1']:4.0f} s", m))


def cmd_modes(a):
    print(HEAD)
    for path in a.capture:
        cap = Capture(path)
        mode = next((mm for _t, mm in cap.modes), None) or int(
            re.search(r"mode(\d)", Path(path).name).group(1))
        ff = cap.first_fix()
        end = 184.0 - cap.offset
        print(row(f"{MODE_NAME.get(mode, mode)} ({mode})", metrics(cap, ff + 10, end),
                  ttff=ff + cap.offset))


def phases(cap: Capture):
    """(start, end, mode, cold) for the sky log: a new phase at every switch and
    every cold start; a switch counts from 20 s after it, a cold start from 10 s
    after its first fix, and a cold-started phase ends at the next switch or 300 s
    after its cold start, whichever is first -- the length every cold-start phase
    was run for."""
    edges = sorted([(t, "switch") for t, _m in cap.modes if t > float("-inf")] +
                   [(t, "cold") for t in cap.colds])
    first = cap.first_fix()
    out, start, cold, cap_at = [], first + 30, True, None
    for t, kind in edges + [(cap.end(), "end")]:
        if kind == "cold" and out and abs(t - out[-1][1]) < 10:
            continue
        end = min(t, cap_at) if cap_at else t
        if end > start:
            out.append((start, end, cap.mode_at(start), cold))
        if kind == "cold":
            ff = cap.first_fix(after=t + 3)
            start, cold, cap_at = (ff + 10 if ff else t), True, t + 300
        else:
            nxt = [c for c in cap.colds if 0 <= c - t < 10]
            if nxt:
                ff = cap.first_fix(after=nxt[0] + 3)
                start, cold, cap_at = (ff + 10 if ff else t), True, nxt[0] + 300
            else:
                start, cold, cap_at = t + 20, False, None
    return out


def cmd_sky(a):
    cap = Capture(a.capture, start_mode=a.start_mode, t1=a.t1)
    print(HEAD)
    for s, e, m, cold in phases(cap):
        if e - s < 60:
            continue
        ttff = None
        if cold:
            c = max((x for x in cap.colds if x < s), default=None)
            ttff = (cap.first_fix(after=c + 3) - c) if c is not None else None
        name = f"{MODE_NAME.get(m, m)}{' cold' if cold else ' switched'} {s/60:4.1f}-{e/60:4.1f}m"
        print(row(name, metrics(cap, s, e), ttff=ttff))


def cmd_overnight(a):
    cap = Capture(a.capture, start_mode=a.start_mode)
    c = cap.colds[-1]
    t, rows = c + 60, []
    while t + a.step <= cap.end():
        m = metrics(cap, t, t + a.step)
        rows.append((round((t - c) / 60, 1), round((t + a.step - c) / 60, 1), m))
        t += a.step
    print(HEAD)
    for x, y, m in rows:
        print(row(f"{cap.mode_at(c) and MODE_NAME[cap.mode_at(c)]} {x:5.0f}-{y:5.0f} min", m))
    if a.csv:
        with open(a.csv, "w") as fh:
            fh.write("min_from_cold_start,min_to,mode,fix_pct,used_median,cn0_median_dbhz,"
                     "strong_sats,strong_resets_per_sat_min,strong_halfcyc_pct,"
                     "resets_per_min,halfcyc_pct,dropouts_per_min\n")
            for x, y, m in rows:
                fh.write(f"{x},{y},{MODE_NAME[cap.mode_at(c)]},{m['fix']:.1f},{m['used']},"
                         f"{m['cn0'] or 0:.1f},{len(m['strong'])},"
                         f"{m['s_resets'] if m['s_resets'] is not None else ''},"
                         f"{m['s_halfcyc'] if m['s_halfcyc'] is not None else ''},"
                         f"{m['resets_min']:.2f},{m['halfcyc'] or 0:.1f},{m['drops_min']:.2f}\n")
        print(f"wrote {a.csv}")


BURN = (180.0, 192.1)          # spaceshot: 13.5 g peak, file time
PAD = (150.0, 179.0)


def boost_run(path):
    """Per GPS satellite through the spaceshot burn, on the file's own clock (MSM7
    epochs carry GPS time), plus the run's fix availability from ignition on."""
    ep = []
    for _t, rest in _lines(path):          # gzip-aware, unlike msm_channels
        if not rest.startswith("R "):
            continue
        try:
            r = rtcm3.parse_msm7(bytes.fromhex(rest[2:]))
        except ValueError:
            continue
        if r and r[0] == "GPS":
            ft = r[1] / 1000.0 - TOW0
            if 0 <= ft <= 900:              # a cold start's first frames carry an old clock
                ep.append((ft, {c["prn"]: c for c in r[2]}))
    ep.sort(key=lambda e: e[0])
    sats = {}
    for prn in sorted({p for _t, c in ep for p in c}):
        pad = [c[prn]["cn0"] for t, c in ep if PAD[0] <= t < PAD[1] and prn in c
               and c[prn].get("cn0")]
        if len(pad) < 10:
            continue                       # not tracked on the pad
        after = [(t, c.get(prn)) for t, c in ep if t >= BURN[0]]
        held = all(x is not None and x.get("cn0") for t, x in after
                   if BURN[0] + 3 <= t < BURN[1])
        lost = next((t for t, x in after if x is None or not x.get("cn0")), None)
        back = next((t for t, x in after if lost is not None and t > lost
                     and x is not None and x.get("cn0")), None)
        burn = [x["cn0"] for t, x in after if t < BURN[1] and x and x.get("cn0")]
        resets, prev = 0, None
        for t, x in after:
            if t >= BURN[1]:
                break
            if x is not None and prev is not None and x["lock_ms"] < prev:
                resets += 1
            prev = x["lock_ms"] if x is not None else None
        sats[prn] = dict(pad=st.median(pad), held=held, lost=lost, back=back,
                         dmin=(min(burn) - st.median(pad)) if burn else None, resets=resets)
    import bisect
    truth = json.loads((HERE / "scenarios" / "spaceshot.json").read_text())["truth"]
    tt = [s["t"] for s in truth]

    def at(x, k):
        i = min(max(bisect.bisect_left(tt, x), 1), len(tt) - 1)
        a_, b_ = truth[i - 1], truth[i]
        fr = (x - a_["t"]) / (b_["t"] - a_["t"]) if b_["t"] > a_["t"] else 0.0
        return a_[k] + fr * (b_[k] - a_[k])

    pvt, wrong, worst = [], 0, 0.0
    for _t, rest in _lines(path):
        if rest.startswith("$PQTMPVT"):
            f = rest.split("*")[0].split(",")
            try:
                ft, ok, fm = int(f[2]) / 1000.0 - TOW0, int(f[5] or 0), int(f[6] or 0)
            except (ValueError, IndexError):
                continue
            valid = ok > 0 and fm >= 2
            pvt.append((ft, valid))
            # the reports' WRONG: valid-flagged and > 5 km or > 50 m/s off the truth
            if valid and BURN[0] + 1 <= ft < 320 and f[11]:
                da = abs(float(f[11]) - at(ft, "alt_m"))
                dv = abs(-float(f[15] or 0) - at(ft, "v_up_mps")) if f[15] else 0.0
                if da > 5000 or dv > 50:
                    wrong += 1
                    worst = max(worst, da)
    P = [v for ft, v in pvt if BURN[0] + 1 <= ft < 320]
    first = next((ft for ft, v in pvt if ft >= BURN[0] + 1 and v), None)
    fixpct = 100 * sum(1 for v in P if v) / len(P) if P else 0.0
    return sats, fixpct, first, wrong, worst


def cmd_boost(a):
    ref = json.loads((HERE / "results" / "doppler_ref_spaceshot.json").read_text())["sats"]
    rate = {int(k.split(":")[1]): v.get("rate_hzs") for k, v in ref.items()
            if k.startswith("0:")}
    runs = [(Path(p).name.split("_spaceshot")[0], boost_run(p)) for p in a.capture]
    prns = sorted({p for _n, (s, *_r) in runs for p in s},
                  key=lambda p: (rate.get(p) is None, rate.get(p) or 0))
    w = 17
    print(f"{'sat':<5}{'Hz/s':>6}  " + "".join(f"{n[:w - 1]:<{w}}" for n, _ in runs))
    for p in prns:
        cells = []
        for _n, (s, *_r) in runs:
            x = s.get(p)
            if x is None:
                cells.append("not on pad")
            elif x["held"]:
                cells.append(f"held {x['dmin']:+.0f} dB" + (f" r{x['resets']}" if x["resets"] else ""))
            else:
                cells.append(f"lost {x['lost']:.0f}" + (f">{x['back']:.0f}" if x["back"] else ""))
        r = rate.get(p)
        print(f"G{p:02d}  {('-' if r is None else f'{r:.0f}'):>6}  " + "".join(f"{c:<{w}}" for c in cells))
    print(f"{'pad C/N0':<13}" + "".join(f"{st.median(x['pad'] for x in s.values()):<{w}.1f}" for _n, (s, *_r) in runs))
    print(f"{'held':<13}" + "".join(f"{sum(1 for x in s.values() if x['held']):<{w}}" for _n, (s, *_r) in runs))
    print(f"{'fix% 181-320':<13}" + "".join(f"{r[0]:<{w}.1f}" for _n, (_s, *r) in runs))
    print(f"{'1st fix >181':<13}" + "".join(f"{('-' if r[1] is None else f'{r[1]:.1f}'):<{w}}" for _n, (_s, *r) in runs))
    print(f"{'WRONG fixes':<13}" + "".join(f"{(f'{r[2]} ({r[3]/1000:.1f} km)' if r[2] else '0'):<{w}}" for _n, (_s, *r) in runs))


IGNITION_PRNS = (29, 12, 30, 19, 14, 15)   # the spaceshot burn's gentlest, 54-199 Hz/s


def cmd_ignition(a):
    """Second by second through ignition: the fix collapsing ($PQTMPVT fix mode /
    satellites used, 180-181 s) and, for the satellites the burn moves least, each
    MSM7 epoch as C/N0 / lock-time counter (s) / half-cycle flag."""
    for path in a.capture:
        ep, pvt, head = {}, [], ""
        for _t, rest in _lines(path):
            if rest.startswith("# host: tx"):
                head = rest
            elif rest.startswith("R "):
                try:
                    r = rtcm3.parse_msm7(bytes.fromhex(rest[2:]))
                except ValueError:
                    continue
                if r and r[0] == "GPS":
                    ft = round(r[1] / 1000.0 - TOW0, 1)
                    if 179 <= ft <= 186:
                        ep[ft] = {c["prn"]: c for c in r[2]}
            elif rest.startswith("$PQTMPVT"):
                f = rest.split("*")[0].split(",")
                try:
                    ft = int(f[2]) / 1000.0 - TOW0
                except (ValueError, IndexError):
                    continue
                if 179.95 <= ft <= 181.05:
                    pvt.append(f"{ft:.1f}:{f[6]}/{int(f[7] or 0)}")
        print(f"== {Path(path).name}\n   {head[:170]}")
        print("   fix/used " + " ".join(pvt))
        for p in IGNITION_PRNS:
            cells = []
            for t in sorted(ep):
                c = ep[t].get(p)
                cells.append(f"{t:.0f}:" + ("  --  " if c is None else
                             f"{c['cn0'] or 0:4.1f}/{c['lock_ms'] / 1000:5.1f}/{c['halfcyc']}"))
            print(f"   G{p:02d} " + "  ".join(cells))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("aba"); p.add_argument("capture"); p.set_defaults(fn=cmd_aba)
    p = sub.add_parser("levels"); p.add_argument("capture"); p.add_argument("schedule")
    p.set_defaults(fn=cmd_levels)
    p = sub.add_parser("modes"); p.add_argument("capture", nargs="+"); p.set_defaults(fn=cmd_modes)
    p = sub.add_parser("sky"); p.add_argument("capture")
    p.add_argument("--start-mode", type=int, default=5); p.add_argument("--t1", type=float)
    p.set_defaults(fn=cmd_sky)
    p = sub.add_parser("overnight"); p.add_argument("capture")
    p.add_argument("--start-mode", type=int, default=5)
    p.add_argument("--step", type=float, default=1800.0); p.add_argument("--csv")
    p.set_defaults(fn=cmd_overnight)
    p = sub.add_parser("boost"); p.add_argument("capture", nargs="+"); p.set_defaults(fn=cmd_boost)
    p = sub.add_parser("ignition"); p.add_argument("capture", nargs="+")
    p.set_defaults(fn=cmd_ignition)
    a = ap.parse_args()
    a.fn(a)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
