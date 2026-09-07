"""What the flight pack did, sample by sample.

The health section reduces the pack to one verdict. This is the trace behind
it: voltage and discharge current through the flight with the events marked,
and the handful of numbers that let a flyer compare one flight's electrical
load against the next — energy used, how hard the pack sagged under load, what
each pyro firing pulled, and whether the record ends the way a landed flight
should.

Two of those numbers exist because of the 2026-08-29 Rolly Polly V flight,
where the av-bay computer's power was severed 0.45 s into boost. The pack was
healthy to its last sample. A single 7.6 V reading with no matching change in
current, 170 ms before the end, was the only warning — a connector losing
contact under vibration — and the log simply stopped mid-cadence with no
landing. Both were found by hand that evening; both are flagged here.

Sign convention: the parser returns milliamps with discharge negative (the
firmware inverts the INA230's sign). This module flips it back, so every
current here is amps of discharge, positive when the pack is being drawn on.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import ROCKET_STATES, get_array  # noqa: E402

from ..charts import COLORS, chart, trace
from ..events import markers
from ..flight import Flight
from ..registry import AnalysisResult
from ..units import q

# The firmware rejects readings outside 3.0–9.0 V and re-logs the previous good
# sample, and a pre-init frame can carry zeros; anything under this is not a pack.
_PRE_INIT_V = 1.0

# A connector losing contact drops the bus for one sample and recovers. What
# separates that from load is the current, and the direction matters: a real
# 100 mV sag on a pack of a few hundred milliohms needs the draw to RISE by
# several hundred milliamps at that sample. A dip with the current flat, or
# falling with it, is contact resistance — the 2026-08-29 V8 sample read 7.61 V
# between 7.8 V neighbours with the current 0.37 A LOWER, which is exactly what
# a joint going high-resistance does to both. Both neighbours must agree, so a
# genuine step (a rail switching on) is not flagged either.
_CHATTER_DV = 0.10      # V, against both neighbours
_CHATTER_DI = 0.30      # A, the smallest load change that could explain it

# A V-vs-I fit is only a source impedance when the current actually varied. The
# nose computer on 2026-08-29 drew 0.11–0.20 A for the whole flight; a slope
# fitted through that is noise.
_IMPEDANCE_MIN_SPREAD_A = 0.30
_IMPEDANCE_MIN_SAMPLES = 20
# And the fit has to explain the voltage: 37 samples of a chattering pack
# through 0.8 s of boost fit an 11 mΩ slope with r² of 0.005, which is noise
# wearing a unit. The sample flight's real 170 mΩ fits at 0.82.
_IMPEDANCE_MIN_R2 = 0.30

# A pyro firing pulls for tens of milliseconds; the pack is sampled every ~10 ms.
# Baseline is the half-second before the fire, the dip is the deepest sample in
# the half-second after it.
_PYRO_BEFORE_S = 0.50
_PYRO_GUARD_S = 0.02
_PYRO_AFTER_S = 0.50

# The tail examined when a log ends without landing: enough samples to say
# whether the pack was sagging into the cut or simply vanished.
_END_TAIL_S = 0.10
_END_SAG_V = 0.05

_STATE_LANDED = 4
_STATES_NEVER_FLEW = (0, 1)   # INIT, READY

_SEC_PER_HOUR = 3600.0


# ---------------------------------------------------------------------------
# Series
# ---------------------------------------------------------------------------

def _series(recs, t0) -> Optional[dict[str, Any]]:
    """The POWER record as arrays: t (s since t0), v (V), i (A discharge),
    soc (%), cam/servo (A or None). Pre-init zeros are dropped."""
    pw = recs.get("POWER") or []
    if not pw or t0 is None or "voltage" not in pw[0]:
        return None
    t = (get_array(pw, "time_us") - t0) / 1e6
    v = get_array(pw, "voltage")
    i = -get_array(pw, "current") / 1000.0
    keep = np.isfinite(t) & np.isfinite(v) & np.isfinite(i) & (v > _PRE_INIT_V)
    if keep.sum() < 2:
        return None
    out: dict[str, Any] = {"t": t[keep], "v": v[keep], "i": i[keep]}
    if "soc" in pw[0]:
        soc = get_array(pw, "soc")
        out["soc"] = soc[keep] if soc.size == keep.size else None
    else:
        out["soc"] = None
    for key in ("cam_a", "servo_a"):
        if pw[0].get(key) is not None:
            arr = get_array(pw, key)
            out[key] = arr[keep] if arr.size == keep.size else None
        else:
            out[key] = None
    return out


def _pyro_fires(recs, t0) -> dict[int, float]:
    """Seconds since t0 at which each pyro channel's fired flag first rose."""
    ns = recs.get("NonSensor") or []
    if not ns or t0 is None:
        return {}
    t = (get_array(ns, "time_us") - t0) / 1e6
    out: dict[int, float] = {}
    for ch in (1, 2, 3, 4):
        key = f"pyro{ch}_fired"
        if key not in ns[0]:
            continue
        fired = get_array(ns, key).astype(bool)
        if fired.any():
            out[ch] = float(t[int(np.argmax(fired))])
    return out


# ---------------------------------------------------------------------------
# The numbers
# ---------------------------------------------------------------------------

def find_chatter(t: np.ndarray, v: np.ndarray, i: np.ndarray,
                 after_s: Optional[float] = None) -> list[dict[str, float]]:
    """Single-sample voltage excursions with no load change to explain them.

    Each hit is {t, v, around, di}: when, the odd sample, what both neighbours
    agreed on, and how much the current moved. `after_s` restricts the search
    to samples at or after that time (the launch, so pad handling — plugging
    in, switching on — is not reported as a fault).
    """
    hits: list[dict[str, float]] = []
    n = t.size
    if n < 3:
        return hits
    for k in range(1, n - 1):
        if after_s is not None and t[k] < after_s:
            continue
        dv_prev = v[k] - v[k - 1]
        dv_next = v[k] - v[k + 1]
        if abs(dv_prev) < _CHATTER_DV or abs(dv_next) < _CHATTER_DV:
            continue
        if np.sign(dv_prev) != np.sign(dv_next):
            continue
        if abs(v[k - 1] - v[k + 1]) >= _CHATTER_DV:
            continue  # the neighbours disagree: a step, not a blip
        around_i = 0.5 * (i[k - 1] + i[k + 1])
        di = i[k] - around_i
        # A dip is explained only by more draw, a spike only by less.
        if dv_prev < 0 and di >= _CHATTER_DI:
            continue
        if dv_prev > 0 and di <= -_CHATTER_DI:
            continue
        hits.append({"t": float(t[k]), "v": float(v[k]),
                     "around": float(0.5 * (v[k - 1] + v[k + 1])), "di": float(di)})
    return hits


def fit_impedance(v: np.ndarray, i: np.ndarray) -> Optional[dict[str, float]]:
    """Source impedance from V = V0 − R·I, or None when the current never
    varied enough to carry a slope. R in milliohms."""
    if v.size < _IMPEDANCE_MIN_SAMPLES:
        return None
    spread = float(np.max(i) - np.min(i))
    if spread < _IMPEDANCE_MIN_SPREAD_A:
        return None
    a = np.vstack([-i, np.ones(i.size)]).T
    coef, *_ = np.linalg.lstsq(a, v, rcond=None)
    r_ohm, v0 = float(coef[0]), float(coef[1])
    resid = v - a @ coef
    total = float(np.sum((v - np.mean(v)) ** 2))
    r2 = 1.0 - float(np.sum(resid ** 2)) / total if total > 0 else 0.0
    return {"r_mohm": r_ohm * 1000.0, "v0": v0, "spread": spread, "r2": r2}


def energy_used(t: np.ndarray, v: np.ndarray, i: np.ndarray) -> tuple[float, float]:
    """(joules, milliamp-hours) drawn from the pack over the record."""
    joules = float(np.trapezoid(v * i, t))
    amp_s = float(np.trapezoid(i, t))
    return joules, amp_s * 1000.0 / _SEC_PER_HOUR


def pyro_dip(t: np.ndarray, v: np.ndarray, t_fire: float) -> Optional[dict[str, float]]:
    """How far the bus fell when a charge fired: baseline before, deepest
    sample after. None when the record does not span the firing."""
    before = (t >= t_fire - _PYRO_BEFORE_S) & (t < t_fire - _PYRO_GUARD_S)
    after = (t >= t_fire - _PYRO_GUARD_S) & (t <= t_fire + _PYRO_AFTER_S)
    if not before.any() or not after.any():
        return None
    base = float(np.median(v[before]))
    low_idx = int(np.argmin(v[after]))
    return {"dip": base - float(v[after][low_idx]),
            "t": float(t[after][low_idx]), "baseline": base}


def log_end(recs, t0, t: np.ndarray, v: np.ndarray, i: np.ndarray) -> dict[str, Any]:
    """Whether the record ends the way a flight should.

    Returns {clean: bool|None, text: str}. `clean` is None for a log that never
    launched — a bench run has no landing to reach.
    """
    ns = recs.get("NonSensor") or []
    if not ns or t0 is None:
        return {"clean": None, "text": "no state stream, so the end of the log cannot be judged"}
    launched = "launch" in ns[0] and any(bool(r.get("launch")) for r in ns)
    landed = "alt_landed" in ns[0] and any(bool(r.get("alt_landed")) for r in ns)
    last_state = int(ns[-1].get("rocket_state", -1))
    state_name = ROCKET_STATES.get(last_state, str(last_state))
    t_end = (ns[-1]["time_us"] - t0) / 1e6

    if not launched and last_state in _STATES_NEVER_FLEW:
        return {"clean": None, "text": f"no launch in this log; it ends in {state_name}"}
    if landed or last_state == _STATE_LANDED:
        return {"clean": True,
                "text": f"landed; last pack sample at {t[-1]:.1f} s reads {v[-1]:.2f} V, {i[-1]:.2f} A"}

    tail = (t >= t[-1] - _END_TAIL_S) & (t < t[-1])
    if tail.any():
        dv = float(v[-1] - np.median(v[tail]))
        if dv <= -_END_SAG_V:
            trend = f"sagging {-dv * 1000:.0f} mV over the final {_END_TAIL_S * 1000:.0f} ms"
        else:
            trend = f"no sag over the final {_END_TAIL_S * 1000:.0f} ms"
    else:
        trend = "too few samples to read a trend"
    return {
        "clean": False,
        "text": (f"the log stops at {t_end:.2f} s in {state_name} with no landing — "
                 f"last pack sample at {t[-1]:.3f} s reads {v[-1]:.2f} V, {i[-1]:.2f} A, {trend}"),
    }


def chatter_in_flight(flight: Flight) -> list[dict[str, float]]:
    """The contact-chatter hits after launch, for the health verdict."""
    s = _series(flight.records, flight.t0_us)
    if s is None:
        return []
    launch = markers(flight).get("launch")
    return find_chatter(s["t"], s["v"], s["i"], after_s=launch)


def pack_summary(flight: Flight) -> Optional[dict[str, Any]]:
    """The one card cell: the pack's lowest reading, with the flight's draw as
    the hint. None when there is no pack data."""
    s = _series(flight.records, flight.t0_us)
    if s is None:
        return None
    t, v, i = s["t"], s["v"], s["i"]
    _, mah = energy_used(t, v, i)
    k = int(np.argmin(v))
    return {
        "v_min": float(v[k]),
        "hint": (f"{v[0]:.2f} V at the start, {v[-1]:.2f} V at the end · "
                 f"{float(np.mean(i)):.2f} A average · {mah:.1f} mAh used"),
    }


# ---------------------------------------------------------------------------
# Charts
# ---------------------------------------------------------------------------

def _pack_chart(s, events) -> Optional[dict[str, Any]]:
    spec = chart("chart-power", "Flight pack", [
        trace(s["t"], s["v"], "Voltage", COLORS[0]),
        trace(s["t"], s["i"], "Discharge current", COLORS[1], axis="y2"),
    ], y_title="Voltage (V)", y2_title="Discharge current (A)", events=events, height=360)
    if not spec:
        return None
    spec["note"] = (spec["note"] + " " if spec["note"] else "") + (
        "Bus voltage and discharge current from the pack monitor, about every 10 ms. "
        "The firmware re-logs the previous sample when a reading falls outside 3–9 V, so "
        "a brownout is a flat trace and then a gap, never a low voltage. Current clamps at "
        "10 A, so a pyro firing harder than that reads flat."
    )
    return spec


def _rails_chart(s, events) -> Optional[dict[str, Any]]:
    cam, servo = s.get("cam_a"), s.get("servo_a")
    if cam is None and servo is None:
        return None
    traces = []
    if cam is not None and np.any(cam != 0):
        traces.append(trace(s["t"], cam, "Camera rail", COLORS[2]))
    if servo is not None and np.any(servo != 0):
        traces.append(trace(s["t"], servo, "Servo rail", COLORS[3]))
    if not traces:
        return None
    spec = chart("chart-power-rails", "Camera and servo rails", traces,
                 y_title="Current (A)", events=events, height=300)
    if spec:
        spec["note"] = (spec["note"] + " " if spec["note"] else "") + (
            "The two switched rails' own monitors. A rail that is fitted but off reads zero."
        )
    return spec


# ---------------------------------------------------------------------------
# Section
# ---------------------------------------------------------------------------

def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="power", title="Flight Pack")
    recs = flight.records
    s = _series(recs, flight.t0_us)
    if s is None:
        result.warnings.append("No pack telemetry in this log, so there is nothing to show.")
        return result

    t, v, i = s["t"], s["v"], s["i"]
    events = {k: round(val, 3) for k, val in markers(flight).items() if val is not None}
    launch = markers(flight).get("launch")
    metrics: dict[str, Any] = {}

    span = float(t[-1] - t[0])
    if span > 0:
        metrics["Sampled at"] = f"{(t.size - 1) / span:.0f} Hz"

    v_launch = float(np.interp(launch, t, v)) if launch is not None else float(v[0])
    metrics["Pack at launch" if launch is not None else "Pack at the start"] = q(v_launch, "V", 2)
    k = int(np.argmin(v))
    metrics["Pack minimum"] = q(float(v[k]), "V", 2, suffix=f" at {t[k]:.1f} s")
    metrics["Pack at the end"] = q(float(v[-1]), "V", 2)

    metrics["Discharge current, mean"] = q(float(np.mean(i)), "A", 2)
    k = int(np.argmax(i))
    metrics["Discharge current, peak"] = q(float(i[k]), "A", 2, suffix=f" at {t[k]:.1f} s")

    joules, mah = energy_used(t, v, i)
    metrics["Energy used"] = q(mah, "mAh", 1, suffix=f" ({joules:.0f} J)")

    soc = s.get("soc")
    if soc is not None and soc.size and np.isfinite(soc).all() and float(np.max(soc)) > 0.5:
        metrics["State of charge"] = (f"{float(soc[0]):.0f} % → {float(soc[-1]):.0f} % "
                                      f"(looked up from the bus voltage, not counted)")

    fit = fit_impedance(v, i)
    if fit is None:
        spread = float(np.max(i) - np.min(i))
        metrics["Source impedance"] = (f"not measurable — the current only varied by {spread:.2f} A, "
                                       f"and a slope needs at least {_IMPEDANCE_MIN_SPREAD_A:.1f} A")
    elif fit["r_mohm"] <= 0 or fit["r2"] < _IMPEDANCE_MIN_R2:
        metrics["Source impedance"] = (f"no consistent sag with load over a {fit['spread']:.2f} A spread "
                                       f"(the fit explains {fit['r2'] * 100:.0f} % of the voltage)")
    else:
        metrics["Source impedance"] = q(fit["r_mohm"], "mΩ", 0,
                                        suffix=f" from a {fit['spread']:.2f} A current spread, "
                                               f"r² {fit['r2']:.2f}")

    for ch, t_fire in sorted(_pyro_fires(recs, flight.t0_us).items()):
        d = pyro_dip(t, v, t_fire)
        if d is None:
            metrics[f"Pyro {ch} firing"] = f"at {t_fire:.2f} s, outside the pack record"
        elif d["dip"] < 0.005:
            metrics[f"Pyro {ch} firing"] = (f"at {t_fire:.2f} s — no dip resolved at this sample rate "
                                            f"({d['baseline']:.2f} V before and after)")
        else:
            metrics[f"Pyro {ch} firing"] = (f"at {t_fire:.2f} s — bus dropped {d['dip'] * 1000:.0f} mV "
                                            f"from {d['baseline']:.2f} V")

    hits = find_chatter(t, v, i, after_s=launch)
    if not hits:
        metrics["Single-sample drops in flight"] = "none"
    else:
        worst = min(hits, key=lambda h: h["v"] - h["around"])
        detail = (f"{len(hits)} — worst at {worst['t']:.3f} s: {worst['v']:.2f} V between "
                  f"{worst['around']:.2f} V neighbours with the draw not rising to match "
                  f"({worst['di'] * 1000:+.0f} mA)")
        metrics["Single-sample drops in flight"] = detail
        result.warnings.append(
            f"The bus dropped for a single sample {len(hits)} time{'s' if len(hits) != 1 else ''} "
            f"in flight with no change in load — a connector or harness losing contact, not the pack. "
            f"Worst: {worst['v']:.2f} V at {worst['t']:.3f} s."
        )

    end = log_end(recs, flight.t0_us, t, v, i)
    metrics["Log end"] = end["text"]
    if end["clean"] is False:
        result.warnings.append("The log ends in flight with no landing. The pack state at the cut is "
                               "in the table: a healthy last sample means the power was severed, "
                               "not exhausted.")

    cam, servo = s.get("cam_a"), s.get("servo_a")
    if cam is not None or servo is not None:
        fitted = [(name, arr) for name, arr in (("Camera rail", cam), ("Servo rail", servo))
                  if arr is not None and np.any(arr != 0)]
        if not fitted:
            metrics["Camera and servo rails"] = "not fitted on this board (both read 0 A)"
        for name, arr in fitted:
            metrics[name] = q(float(np.mean(arr)), "A", 2, suffix=f", peak {float(np.max(arr)):.2f} A")

    result.metrics = metrics
    result.charts = [c for c in (_pack_chart(s, events), _rails_chart(s, events)) if c]
    return result
