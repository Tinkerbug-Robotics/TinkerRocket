"""Measured flight events — what the vehicle did, not what it declared.

The flight computer's flags are *declarations*. Each one latches only after its
detector is confident, so it lands a little after the thing it names: on the
sample flight the launch flag is 0.20 s after the motor lit and the burnout flag
is 0.07 s after thrust ended. That lag is correct for firing a charge and wrong
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
callers must render a blank rather than a guess.
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
# Thrust has ended when specific force falls back through 1 g: the motor is no
# longer holding the vehicle up against its own weight.
_COAST_G = 1.0
# ...and stays there. A momentary dip mid-burn (chuffing, a stager) is not
# burnout.
_COAST_HOLD_S = 0.05
# Search the launch flag's neighbourhood rather than the whole log, so handling
# the rocket on the pad cannot be mistaken for ignition.
_LAUNCH_SEARCH_BEFORE_S = 3.0
_LAUNCH_SEARCH_AFTER_S = 1.0

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


def _flags(recs) -> list:
    return recs.get("NonSensor") or []


def _flag_time(recs, t0_us, name: str) -> Optional[float]:
    """Seconds since t0 for the first record with `name` set."""
    for r in _flags(recs):
        if r.get(name):
            return (r["time_us"] - t0_us) / 1e6
    return None


def _accel_series(flight) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """(seconds since t0, |a| in g) from the IMU, low-G unless it saturated."""
    recs = flight.records
    imu = recs.get("ISM6HG256") or []
    if not imu or flight.t0_us is None:
        return None, None
    t = (get_array(imu, "time_us") - flight.t0_us) / 1e6
    mag, _which = accel_magnitude(recs, flight.sidecar, None)
    if mag is None or not mag.size or mag.size != t.size:
        return None, None
    return t, mag / G


def _cross(t, y, i, level) -> float:
    """Time at which y crosses `level` between samples i-1 and i."""
    y0, y1 = float(y[i - 1]), float(y[i])
    if y1 == y0:
        return float(t[i])
    f = (level - y0) / (y1 - y0)
    return float(t[i - 1] + f * (float(t[i]) - float(t[i - 1])))


def true_launch(flight, t, g) -> Optional[float]:
    """First motion: the moment the accelerometer leaves the pad band.

    Found by locating an unambiguous thrust sample and walking *back* to where
    the trace left 1 g, which is the instant the motor started pushing rather
    than the instant it became obvious.
    """
    if t is None:
        return None
    flag = _flag_time(flight.records, flight.t0_us, "launch")
    if flag is not None:
        window = (t >= flag - _LAUNCH_SEARCH_BEFORE_S) & (t <= flag + _LAUNCH_SEARCH_AFTER_S)
    else:
        window = np.ones(t.size, dtype=bool)
    idx = np.flatnonzero(window & (g > _IGNITION_G))
    if not idx.size:
        return None

    i = int(idx[0])
    while i > 0 and g[i] > _PAD_G:
        i -= 1
    # Walked all the way to the start of the record and never found the pad: the
    # log begins with the vehicle already under thrust, so first motion happened
    # before anything was written down and cannot be measured. One of the bench
    # logs is like this, opening at 8.1 g.
    if i == 0 and g[0] > _PAD_G:
        return None
    if i + 1 >= t.size:
        return None
    return _cross(t, g, i + 1, _PAD_G)


def true_burnout(flight, t, g, launch: Optional[float]) -> Optional[float]:
    """Thrust ends: specific force falls back through 1 g and stays there.

    Walking forward from launch rather than back from the burn's peak, because
    the largest acceleration in the flight is usually the ejection charge, not
    the motor — 38 g against 9 g on the sample flight.
    """
    if t is None or launch is None:
        return None
    idx = np.flatnonzero((t > launch) & (g < _COAST_G))
    for i in idx:
        i = int(i)
        if i == 0:
            continue
        held = (t >= t[i]) & (t <= t[i] + _COAST_HOLD_S)
        if held.any() and float(np.max(g[held])) < _COAST_G:
            return _cross(t, g, i, _COAST_G)
    return None


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

    t, g = _accel_series(flight)
    out["landed"] = _flag_time(flight.records, flight.t0_us, "alt_landed")
    out["launch"] = true_launch(flight, t, g)
    out["burnout"] = true_burnout(flight, t, g, out["launch"])
    out["ejection"] = ejection(flight, t, g, out["burnout"], out["landed"])
    out["apogee"] = true_apogee(flight, out["launch"])

    flight.__dict__[_CACHE_KEY] = out
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
