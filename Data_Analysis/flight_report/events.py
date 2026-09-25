"""Measured flight events — what the vehicle did, not what it declared.

The flight computer's flags are *declarations*. Each one latches only after its
detector is confident, so it lands a little after the thing it names: on the
sample flight the launch flag is 0.20 s after the motor lit and the burnout flag
is 0.05 s after thrust ended. That lag is correct for firing a charge and wrong
for a report that quotes a burn time — a fifth of a second on a 1.5 s burn is 13
per cent.

Apogee is worse than late, because there is no single apogee flag to be late.
The log carries five: one vote each from the barometric, velocity, pitch and
GNSS detectors, plus the master that latches from them and fires the charge. The
report used to read the barometric vote — the earliest of the four — and call it
"apogee", which is how the summary card came to disagree with the flight
computer's own record by 1.15 s.

So every event here is measured from the sensor record, and every module that
needs one calls the same function. Two sections cannot then disagree about when
the rocket left the pad.

Where a flag is genuinely the best available answer it is still used, and said
so: touchdown has no distinctive signature in the accelerometer once the vehicle
is down, so `landed` is the declaration.

Everything returned is seconds since `flight.t0_us`, or None when the flight
does not contain the event. None is normal — a bench log has no launch — and
callers must render a blank rather than a guess. A launch or a burnout the log
lost is None too: a motor can light, or burn out, inside a hole in the record,
and launch_gap() and burnout_gap() say where the hole was so the blank can carry
its reason.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array, pressure_to_altitude  # noqa: E402

from .imu import accel_magnitude

G = 9.80665

# Sitting on the pad the accelerometer reads 1 g plus noise. Above this it is
# being pushed by something.
_PAD_G = 1.2
# Unambiguously under thrust — used only to find the burn, never as the launch
# instant itself, which is walked back to from here.
_IGNITION_G = 2.0
# Thrust has ended when the specific force along the body's long axis turns
# negative: the air is pushing back harder than the motor pushes. The flight
# computer's burnout detector watches the same sign (BurnoutDetector.h).
_COAST_AXIAL_G = 0.0
# ...and stays there. A momentary dip mid-burn (chuffing, a stager) is not
# burnout. The flight computer waits the same 50 ms, as 50 flight-loop ticks.
_COAST_HOLD_S = 0.05
# Search the launch flag's neighbourhood rather than the whole log, so handling
# the rocket on the pad cannot be mistaken for ignition.
_LAUNCH_SEARCH_BEFORE_S = 3.0
_LAUNCH_SEARCH_AFTER_S = 1.0
# Two consecutive records further apart than this have a hole between them, not
# a sample step. First motion is interpolated between the last pad sample and
# the first one off the pad, and across a hole that is wherever a straight line
# happens to cut 1.2 g: on 2026-07-05 195028 the motor lit inside a 743 ms gap,
# and the line put first motion 37 ms after the last pad sample, where the log
# holds nothing at all. 10 ms is the card's resolution (burn time prints to
# 0.01 s), so a step up to that wide cannot move a printed figure by more than
# its last digit. The widest step a clean walk-back crosses in the flight
# archive is 4.2 ms; the two launches that fell in a gap sit in holes of 743
# and 746 ms.
#
# Burnout is interpolated the same way, between the last sample under thrust
# and the first one coasting, and holds to the same bar. On 2026-03-14 224121
# the log lost 990 ms with body X still at 8.2 g and came back at -0.35 g, and
# the line put burnout 96 per cent of the way across. The widest step a clean
# burnout crosses is 3.1 ms. The March 2026 logs also stall for 10-20 ms at a
# time, and some for longer, dozens to thousands of times a log. Those are
# holes by this bar too, but only the step a crossing sits in is tested, and no
# crossing in the archive sits in one.
_GAP_S = 0.010
# Snapshot frames read for the flight computer's own launch time. The median of
# a few, because each frame's stamp and count are taken a few ms apart.
_SNAPSHOTS_READ = 5

# An ejection charge is impulsive and violent: 38 g against a 0.2 g coast on the
# sample flight. Both bars must be cleared — an absolute floor so ordinary
# tumbling under a canopy cannot qualify, and a ratio so a quiet log cannot have
# its own noise promoted to an event.
_EJECT_MIN_G = 4.0
_EJECT_MIN_RATIO = 8.0
# The charge cannot fire into the burn, and the touchdown impact is not an
# ejection.
_EJECT_AFTER_BURNOUT_S = 0.2
_EJECT_BEFORE_LANDING_S = 1.0

# Below this the barometer is looking at weather, not a flight. Guards the
# fallback peak: a log recorded on a bench has an argmax like any other, and
# without this it is reported as an apogee.
_MIN_FLIGHT_ALT_M = 20.0

_CACHE_KEY = "_measured_events"
_GAP_CACHE_KEY = "_launch_gap"
_BURNOUT_GAP_CACHE_KEY = "_burnout_gap"


def _flags(recs) -> list:
    return recs.get("NonSensor") or []


def _flag_time(recs, t0_us, name: str) -> Optional[float]:
    """Seconds since t0 for the first record with `name` set."""
    for r in _flags(recs):
        if r.get(name):
            return (r["time_us"] - t0_us) / 1e6
    return None


def _snapshot_launch(recs, t0_us) -> Optional[float]:
    """The flight computer's own launch time, from its Snapshot frames.

    A Snapshot carries `flight_elapsed_ms`, counted from the moment the flight
    computer declared launch, beside a stamp on the same clock as every other
    stream. Stamp minus count is the declaration itself. It lands within 3 ms of
    the launch flag on the logs that lost nothing.
    """
    snaps = [s for s in recs.get("Snapshot") or []
             if s.get("flight_elapsed_ms") and s.get("time_us") is not None]
    if not snaps:
        return None
    return float(np.median([(s["time_us"] - t0_us) / 1e6 - s["flight_elapsed_ms"] / 1e3
                            for s in snaps[:_SNAPSHOTS_READ]]))


def declared_launch(flight) -> Optional[float]:
    """When the flight computer declared launch, in seconds since t0.

    Normally the first record with the launch flag set. But records can be lost
    on their way into the log, and when the first flagged ones are, the flag in
    the log is late by however long the hole lasted. 2026-07-05 195028 lost
    745 ms of every flight-computer stream, launch included: its first flagged
    record comes 0.47 s after the flight computer called launch, by which time
    the rocket was 7 m up and doing 24 m/s.

    So when a hole sits right before the first flagged record and the
    Snapshot's time falls inside it, the Snapshot is the answer. Everywhere else
    it is the flag: no hole, no Snapshot (firmware before it had one), or a
    Snapshot that disagrees with the log around it.
    """
    t0_us = flight.t0_us
    if t0_us is None:
        return None
    ns = _flags(flight.records)
    k = next((i for i, r in enumerate(ns) if r.get("launch")), None)
    if k is None:
        return None
    flag = (ns[k]["time_us"] - t0_us) / 1e6
    if k == 0:
        return flag
    before = (ns[k - 1]["time_us"] - t0_us) / 1e6
    if flag - before <= _GAP_S:
        return flag
    snap = _snapshot_launch(flight.records, t0_us)
    if snap is not None and before < snap < flag:
        return snap
    return flag


def _accel_series(flight) -> tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    """(seconds since t0, |a| in g, body X in g) from the IMU.

    |a| is the low-G part's unless that saturated anywhere in the log. Body X is
    always the low-G part's, for the reasons true_burnout gives.
    """
    recs = flight.records
    imu = recs.get("ISM6HG256") or []
    if not imu or flight.t0_us is None:
        return None, None, None
    t = (get_array(imu, "time_us") - flight.t0_us) / 1e6
    mag, _which = accel_magnitude(recs, None)
    if mag is None or not mag.size or mag.size != t.size:
        return None, None, None
    ax = get_array(imu, "low_acc_x") / G if "low_acc_x" in imu[0] else None
    return t, mag / G, ax


def _cross(t, y, i, level) -> float:
    """Time at which y crosses `level` between samples i-1 and i."""
    y0, y1 = float(y[i - 1]), float(y[i])
    if y1 == y0:
        return float(t[i])
    f = (level - y0) / (y1 - y0)
    return float(t[i - 1] + f * (float(t[i]) - float(t[i - 1])))


def _first_thrust(flight, t, g) -> Optional[int]:
    """Index of the first unambiguous thrust sample near the launch call."""
    if t is None:
        return None
    call = declared_launch(flight)
    if call is not None:
        window = (t >= call - _LAUNCH_SEARCH_BEFORE_S) & (t <= call + _LAUNCH_SEARCH_AFTER_S)
    else:
        window = np.ones(t.size, dtype=bool)
    idx = np.flatnonzero(window & (g > _IGNITION_G))
    return int(idx[0]) if idx.size else None


def _first_motion(flight, t, g) -> tuple[Optional[float], Optional[tuple[float, float]]]:
    """(first motion, the hole it fell in). At most one of the two is set."""
    i = _first_thrust(flight, t, g)
    if i is None:
        return None, None
    while i > 0 and g[i] > _PAD_G:
        i -= 1
    # Walked all the way to the start of the record and never found the pad: the
    # log begins with the vehicle already under thrust, so first motion happened
    # before anything was written down and cannot be measured. One of the bench
    # logs is like this, opening at 8.1 g.
    if i == 0 and g[0] > _PAD_G:
        return None, None
    if i + 1 >= t.size:
        return None, None
    # The pad band was left inside a hole in the log. Only this step matters: a
    # hole further up the climb, with the trace above the band on both sides,
    # does not move the crossing.
    if t[i + 1] - t[i] > _GAP_S:
        return None, (float(t[i]), float(t[i + 1]))
    return _cross(t, g, i + 1, _PAD_G), None


def true_launch(flight, t, g) -> Optional[float]:
    """First motion: the moment the accelerometer leaves the pad band.

    Found by locating an unambiguous thrust sample and walking *back* to where
    the trace left 1 g, which is the instant the motor started pushing rather
    than the instant it became obvious.

    None when the log cannot say: it opens already under thrust, or the trace
    left the pad band inside a hole in the log (see launch_gap).
    """
    return _first_motion(flight, t, g)[0]


def _burnout(flight, t, g, ax) -> tuple[Optional[float], Optional[tuple[float, float]]]:
    """(burnout, the hole it fell in). At most one of the two is set."""
    if ax is None:
        return None, None
    i0 = _first_thrust(flight, t, g)
    if i0 is None:
        return None, None
    idx = np.flatnonzero((t > t[i0]) & (ax < _COAST_AXIAL_G))
    for i in idx:
        i = int(i)
        if i == 0:
            continue
        held = (t >= t[i]) & (t <= t[i] + _COAST_HOLD_S)
        if held.any() and float(np.max(ax[held])) < _COAST_AXIAL_G:
            # Body X turned negative inside a hole in the log. Only this step
            # matters, as for first motion. A hole inside the hold is not
            # tested: the crossing before it is still a measurement, and on the
            # two logs that have one (50 ms on 224121, 71 ms on 2026-05-03
            # RolyPoly flight_recovered_4) body X is negative on both sides.
            if t[i] - t[i - 1] > _GAP_S:
                return None, (float(t[i - 1]), float(t[i]))
            return _cross(t, ax, i, _COAST_AXIAL_G), None
    return None, None


def true_burnout(flight, t, g, ax) -> Optional[float]:
    """Thrust ends: the specific force along the body's long axis turns negative
    and stays there.

    The accelerometer reads thrust minus drag, over the mass. Along the long
    axis that is positive while the motor out-pushes the air and negative once
    it does not. The flight computer's burnout detector watches exactly this,
    the low-G part's body X below zero, so the two answer one question and
    differ only in when they say so. The flag waits until the sign has held: for
    50 flight-loop ticks since #197 (2026-05-23), which puts it 40-54 ms after
    this crossing on every flight in the archive since; before that, for one
    negative sample, which puts it within 6 ms of the crossing on ten flights of
    eleven. The eleventh, 2026-05-09 Journey 75, hovered at zero for 0.1 s at
    the end of a long tail, and the flag took the first dip.

    The magnitude |a| used to be the test, against a 1 g bar, and it cannot see
    the sign. After burnout it reads drag, and drag above 1 g held it over the
    bar for seconds: 1.30 s late on the 2026-05-09 Goblin. It read a sideways
    force as thrust: 3 g across the airframe as it swung at burnout put the
    2026-05-17 54 mm Rolly Polly 0.28 s late. It read the high-G part's offset,
    and |a| comes from the high-G part on most logs: 0.43 g on X put the
    2026-05-17 65 mm RIM-66 2.81 s late. And it cut short every burn with a
    sustain or a long tail, while the motor still pushed up to 1 g net: by
    0.16-0.44 s on eight flights.

    The low-G part, never the high-G: it is the finer instrument, a sign change
    is nowhere near its rail, and the high-G part's X offset can be as large as
    the drag on a slow coast.

    Walking forward from the burn rather than back from its peak, because the
    largest acceleration in the flight is usually the ejection charge, not the
    motor — 38 g against 9 g on the sample flight.

    Walking from the first unambiguous thrust sample, not from first motion,
    because the end of a burn can be in the log when its start is not:
    2026-07-05 195028 lit its motor inside a 743 ms hole and burned out 0.6 s
    after the log came back. Nothing between the two can end a burn — that
    stretch is the thrust building to 2 g — so where both are known the answer
    is the same.

    None when the log cannot say: body X turned negative inside a hole in the
    log (see burnout_gap). 2026-03-14 224121 lost 990 ms with the motor still
    at 8 g and came back coasting, so the burn ended somewhere in that second,
    and a crossing drawn across it is only where a straight line happens to
    cut zero.
    """
    return _burnout(flight, t, g, ax)[0]


def ejection(flight, t, g, burnout: Optional[float], landed: Optional[float]) -> Optional[float]:
    """When the recovery system was deployed.

    A fired pyro channel records its own time, so that is used when there is
    one. Otherwise the vehicle recovers on motor ejection, which logs nothing —
    but the charge is the most violent thing that happens between burnout and
    touchdown, so it is found as the dominant transient in that span.
    """
    recs = flight.records
    ns = _flags(recs)
    fired = [_flag_time(recs, flight.t0_us, f"pyro{ch}_fired") for ch in (1, 2, 3, 4)
             if ns and f"pyro{ch}_fired" in ns[0]]
    fired = [f for f in fired if f is not None]
    if fired:
        return min(fired)

    if t is None or burnout is None:
        return None
    hi = (landed - _EJECT_BEFORE_LANDING_S) if landed is not None else float(t[-1])
    window = (t > burnout + _EJECT_AFTER_BURNOUT_S) & (t < hi)
    if not window.any():
        return None

    tw, gw = t[window], g[window]
    i = int(np.argmax(gw))
    peak = float(gw[i])
    if peak < _EJECT_MIN_G:
        return None
    # Background is the coast before the peak, which is the quiet part; taking it
    # after would include the canopy ringing the charge itself caused.
    before = gw[:i]
    background = float(np.median(before)) if before.size else float(np.median(gw))
    if peak < _EJECT_MIN_RATIO * max(background, 0.05):
        return None
    return float(tw[i])


def gnss_vertical_zero(t, vu) -> Optional[float]:
    """Apogee from the GNSS Doppler vertical velocity crossing zero downwards.

    Gated to after the vehicle clearly ascended (vu > 10 m/s) so on-pad noise is
    skipped, with a 5-sample median to de-spike vu first.

    Preferred over the barometric peak, which lags and is spike-prone (#112) —
    and on a flight that ejects at or before apogee the pressure transient lands
    directly on the peak the barometer is being asked to find.
    """
    if t is None or len(t) < 6:
        return None
    half = 2
    sm = [sorted(vu[max(0, i - half):min(len(vu), i + half + 1)])[
              (min(len(vu), i + half + 1) - max(0, i - half)) // 2]
          for i in range(len(vu))]
    ascended = False
    for i in range(1, len(sm)):
        if sm[i - 1] > 10.0:
            ascended = True
        if ascended and sm[i - 1] >= 0.0 and sm[i] < 0.0:
            denom = sm[i - 1] - sm[i]
            f = (sm[i - 1] / denom) if denom else 0.0
            return float(t[i - 1] + f * (t[i] - t[i - 1]))
    return None


def _baro_peak(flight, launch: Optional[float]) -> Optional[float]:
    """Time of maximum barometric altitude, median-filtered.

    The fallback when there is no usable GNSS. Filtered because an ejection
    charge drives the pressure sensor hard for a few samples, and an unfiltered
    argmax will happily return the top of that spike.
    """
    recs = flight.records
    baro = recs.get("BMP585") or []
    if not baro or flight.t0_us is None:
        return None
    p = get_array(baro, "pressure_pa")
    tb = (get_array(baro, "time_us") - flight.t0_us) / 1e6
    if p.size < 20:
        return None
    ground = float(np.mean(p[: min(200, p.size)]))
    alt = np.array([pressure_to_altitude(float(v), ground) for v in p])
    if float(np.max(alt)) < _MIN_FLIGHT_ALT_M:
        return None

    k = 25
    if alt.size > k:
        pad = k // 2
        padded = np.concatenate([np.full(pad, alt[0]), alt, np.full(pad, alt[-1])])
        alt = np.array([np.median(padded[i:i + k]) for i in range(alt.size)])

    mask = tb > launch if launch is not None else np.ones(tb.size, dtype=bool)
    if not mask.any():
        return None
    return float(tb[mask][int(np.argmax(alt[mask]))])


def true_apogee(flight, launch: Optional[float]) -> Optional[float]:
    """The top of the trajectory, measured rather than declared."""
    recs = flight.records
    gnss = recs.get("GNSS") or []
    if gnss and flight.t0_us is not None and "vel_u" in gnss[0]:
        tg = list((get_array(gnss, "time_us") - flight.t0_us) / 1e6)
        vu = list(get_array(gnss, "vel_u"))
        found = gnss_vertical_zero(tg, vu)
        if found is not None:
            return found
    return _baro_peak(flight, launch)


def measured(flight) -> dict[str, Optional[float]]:
    """Every event this module can find, in seconds since t0. Cached per flight.

    Keys: launch, burnout, apogee, ejection, landed. A value of None means the
    flight does not contain that event — render nothing rather than a guess.
    Launch and burnout are also None when they fell in a hole in the log;
    launch_gap() and burnout_gap() say where.
    """
    cached = flight.__dict__.get(_CACHE_KEY)
    if cached is not None:
        return cached

    out: dict[str, Optional[float]] = {
        "launch": None, "burnout": None, "apogee": None,
        "ejection": None, "landed": None,
    }
    if flight.t0_us is None:
        flight.__dict__[_CACHE_KEY] = out
        return out

    t, g, ax = _accel_series(flight)
    out["landed"] = _flag_time(flight.records, flight.t0_us, "alt_landed")
    out["launch"], gap = _first_motion(flight, t, g)
    out["burnout"], burnout_hole = _burnout(flight, t, g, ax)
    # A burn that ended inside a hole was over by the time the log came back, so
    # the charge is looked for from there.
    coasting = out["burnout"] if burnout_hole is None else burnout_hole[1]
    out["ejection"] = ejection(flight, t, g, coasting, out["landed"])
    out["apogee"] = true_apogee(flight, out["launch"])

    flight.__dict__[_GAP_CACHE_KEY] = gap
    flight.__dict__[_BURNOUT_GAP_CACHE_KEY] = burnout_hole
    flight.__dict__[_CACHE_KEY] = out
    return out


def launch_gap(flight) -> Optional[tuple[float, float]]:
    """The hole in the log that first motion fell in, or None if there was none.

    (last sample on the pad, first sample off it), in seconds since t0. When it
    is set, measured() has no launch and every figure that counts from first
    motion is blank. This is what lets a caller say why, rather than leave the
    reader to wonder what the report forgot.
    """
    measured(flight)
    return flight.__dict__.get(_GAP_CACHE_KEY)


def burnout_gap(flight) -> Optional[tuple[float, float]]:
    """The hole in the log that burnout fell in, or None if there was none.

    (last sample under thrust, first sample coasting), in seconds since t0. When
    it is set, measured() has no burnout, and burn time and coast time are
    blank. Sections that bound a window by burnout stop it at the hole's edge
    instead.
    """
    measured(flight)
    return flight.__dict__.get(_BURNOUT_GAP_CACHE_KEY)


def markers(flight) -> dict[str, Optional[float]]:
    """Measured events, but with the flight computer's launch call standing in
    when first motion cannot be measured.

    For anchoring, not for quoting. Chart windows and the globe's pad reference
    are all pinned to launch, and a log that opens mid-boost has no measurable
    one — which would silently widen every window to the whole record and take
    the pad datum from a second of powered flight. At chart scale the call's
    fifth of a second does not show, so it is the right answer there.

    The call is declared_launch(), not simply the first flagged record. Where
    records were lost ahead of the flag, the flag in the log is late by the
    whole hole, and on 2026-07-05 195028 it would pin every window 0.47 s into
    the boost.

    The summary card must NOT use this: a burn time started from a flag 0.2 s
    late is wrong by 13 per cent, which is the whole reason events.py exists.
    """
    out = dict(measured(flight))
    if out["launch"] is None:
        out["launch"] = declared_launch(flight)
    return out


def span(events: dict[str, Optional[float]], a: str, b: str) -> Optional[float]:
    """Seconds from event `a` to event `b`, or None if either is missing.

    Negative spans are returned as-is: a rocket really can eject before apogee,
    and hiding that would be the opposite of useful.
    """
    ta, tb = events.get(a), events.get(b)
    if ta is None or tb is None:
        return None
    return tb - ta
