"""How hard the airframe shook, at what frequency, and whether it was shaking
or knocking.

Everything upstream of this section treats the accelerometer as a measure of
the vehicle's motion. Under thrust it is mostly a measure of the motor: on the
2026-08-29 Rolly Polly V flight the two computers recorded a single ~800 Hz
tone at 36 and 51 m/s² RMS, coherent between the nose and the av-bay, that
grew while thrust fell and died at half thrust — a combustion oscillation,
not the rocket moving. The same flight's nose computer then took an 87 g
burst from something loose, which railed the gyro and left the attitude 38°
wrong through apogee. None of that was visible on the report; it took a day
of scripting to find. This section is those numbers, on every flight, so the
next motor, mount or joint change can be judged against the last one.

Three choices worth knowing:

- The high-g accelerometer is the instrument. The ±16 g part rails under
  thrust on this class of motor (23 % of boost samples on that flight); its
  rail fraction is reported, its spectrum is not.
- Samples are snapped to the sensor's own clock before any spectrum. The
  timestamps carry ±125 µs of read jitter and about 1 % of slots are dropped
  in boost; a spectrum taken on the raw stamps smears a tone into a hump.
- Kurtosis separates shaking from knocking. A steady oscillation is Gaussian
  (kurtosis ≈ 0); impacts are heavy-tailed (8–35 on the nose that flight).
  That is the difference between "the motor buzzes" and "something is loose".

No scipy: the hosted tool runs on numpy, pandas, matplotlib and jinja2 only.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

from ..charts import COLORS, chart, trace
from ..events import markers, measured
from ..flight import Flight
from ..registry import AnalysisResult
from ..units import q

G = 9.80665

# "Vibration" is everything above this; below it is the vehicle moving.
_HP_HZ = 100.0
# Bands, in the order a reader thinks about them: airframe bending, local
# mounts, the motor. The last runs to Nyquist.
_BANDS: tuple[tuple[float, Optional[float]], ...] = (
    (2.0, 50.0), (50.0, 200.0), (200.0, 500.0), (500.0, 1000.0), (1000.0, None))
# The envelope chart: RMS above 20 Hz in 20 ms windows, plotted every 5 ms.
_ENV_HP_HZ = 20.0
_ENV_WINDOW_S = 0.020
_ENV_STEP_S = 0.005
# Rails, as fractions of the configured full scale, tested in sensor axes.
_RAIL_LOW_G = 0.99
_RAIL_GYRO = 0.95
# Above this the >100 Hz content is impacts, not oscillation — but only
# when there is enough of it to mean anything: at the sensor floor a few
# counts of noise make any kurtosis, and the first quarter second of boost
# is ignition and the rail buttons leaving the rail, which knock on every
# flight. Below the floor no tone is reported either.
_IMPACT_KURTOSIS = 8.0
_KNOCK_MIN_RMS = 5.0        # m/s² above 100 Hz on that axis
_TONE_MIN_RMS = 1.0         # m/s² above 100 Hz
_RAIL_EXIT_S = 0.25
# The pad window: a second and a half ending just before first motion.
_PAD_BEFORE_S = 1.5
_PAD_GUARD_S = 0.1
_MIN_SAMPLES = 256
_WELCH_NPERSEG = 1024
# A tone's share is the energy within this fraction of its frequency.
_TONE_HALF_WIDTH = 0.10


# ---------------------------------------------------------------------------
# Signal helpers (numpy only)
# ---------------------------------------------------------------------------

def uniform(t: np.ndarray, chans: dict[str, np.ndarray]) -> Optional[dict[str, Any]]:
    """Snap samples to the sensor's ODR slots and fill dropped slots.

    Each interval is rounded to a whole number of slots at the median rate —
    one for a normal step, two where a sample was dropped — so read jitter of
    a fraction of a slot vanishes and a rate that is off by a part in a
    thousand cannot accumulate into a slot error down the window (it would,
    over 2,000 samples, if every stamp were placed against t[0]). The rate is
    then refined from the slot count over the whole span. A slot nothing
    landed in is filled by linear interpolation. Returns None when there is
    not enough to work with, or the stamps are not a sampled stream at all.
    """
    if t.size < 16:
        return None
    dt = np.diff(t)
    if np.any(dt <= 0):
        order = np.argsort(t, kind="stable")
        t = t[order]
        chans = {k: v[order] for k, v in chans.items()}
        dt = np.diff(t)
        keep = np.concatenate([[True], dt > 0])
        t = t[keep]
        chans = {k: v[keep] for k, v in chans.items()}
        dt = np.diff(t)
    if dt.size < 8:
        return None
    slot = float(np.median(dt))
    steps = np.maximum(1, np.round(dt / slot).astype(np.int64))
    idx = np.concatenate([[0], np.cumsum(steps)])
    n = int(idx[-1]) + 1
    if n < _MIN_SAMPLES or n > 4 * idx.size:
        return None
    fs = float(idx[-1]) / float(t[-1] - t[0])
    tu = t[0] + np.arange(n) / fs
    out: dict[str, np.ndarray] = {}
    for key, v in chans.items():
        full = np.full(n, np.nan)
        full[idx] = v
        gaps = np.isnan(full)
        if gaps.any():
            full[gaps] = np.interp(tu[gaps], tu[~gaps], full[~gaps])
        out[key] = full
    return {"t": tu, "chans": out, "fs": fs, "filled": int(n - idx.size)}


def welch(x: np.ndarray, fs: float, nperseg: int = _WELCH_NPERSEG) -> Optional[tuple[np.ndarray, np.ndarray]]:
    """One-sided PSD in (unit²)/Hz, Hann window, 50 % overlap, mean removed
    per segment."""
    n = min(nperseg, x.size)
    if n < 64:
        return None
    step = max(1, n // 2)
    win = np.hanning(n)
    scale = 1.0 / (fs * float(np.sum(win ** 2)))
    starts = range(0, x.size - n + 1, step)
    acc = None
    for s0 in starts:
        seg = x[s0:s0 + n]
        seg = seg - np.mean(seg)
        p = (np.abs(np.fft.rfft(seg * win)) ** 2) * scale
        p[1:-1] *= 2.0
        acc = p if acc is None else acc + p
    if acc is None:
        return None
    return np.fft.rfftfreq(n, 1.0 / fs), acc / len(starts)


def band_rms(f: np.ndarray, p: np.ndarray, lo: float, hi: Optional[float]) -> float:
    df = float(f[1] - f[0]) if f.size > 1 else 0.0
    m = (f >= lo) & ((f < hi) if hi is not None else np.ones(f.size, bool))
    return float(math.sqrt(max(0.0, float(np.sum(p[m])) * df)))


def dominant_tone(f: np.ndarray, p: np.ndarray, fmin: float = _HP_HZ) -> Optional[tuple[float, float]]:
    """(frequency, share of the energy above fmin within ±10 % of it)."""
    m = f >= fmin
    if m.sum() < 8:
        return None
    ff, pp = f[m], p[m]
    total = float(np.sum(pp))
    if total <= 0:
        return None
    k = int(np.argmax(pp))
    f0 = float(ff[k])
    near = (ff >= f0 * (1 - _TONE_HALF_WIDTH)) & (ff <= f0 * (1 + _TONE_HALF_WIDTH))
    return f0, float(np.sum(pp[near])) / total


def highpass(x: np.ndarray, fs: float, fc: float) -> np.ndarray:
    """Brick-wall high-pass by FFT; fine for a whole-window statistic."""
    x = x - np.mean(x)
    spec = np.fft.rfft(x)
    f = np.fft.rfftfreq(x.size, 1.0 / fs)
    spec[f < fc] = 0.0
    return np.fft.irfft(spec, n=x.size)


def kurtosis(x: np.ndarray) -> float:
    """Excess kurtosis: 0 for Gaussian, large and positive for impacts."""
    m = x - np.mean(x)
    s2 = float(np.mean(m ** 2))
    if s2 <= 0:
        return 0.0
    return float(np.mean(m ** 4) / (s2 * s2) - 3.0)


def envelope(x: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    """20 ms RMS of the >20 Hz content, every 5 ms. Returns (sample index, rms)."""
    hp = highpass(x, fs, _ENV_HP_HZ)
    n = max(2, int(round(_ENV_WINDOW_S * fs)))
    step = max(1, int(round(_ENV_STEP_S * fs)))
    c = np.concatenate([[0.0], np.cumsum(hp ** 2)])
    ends = np.arange(n, hp.size + 1, step)
    rms = np.sqrt((c[ends] - c[ends - n]) / n)
    centre = ends - n / 2.0
    return centre, rms


def to_sensor_axes(bx: np.ndarray, by: np.ndarray, rot_z_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Undo the converter's Z rotation so a rail test sees the chip's own
    axes. The converter applies body = R(θ)·sensor with R = [[c,−s],[s,c]]."""
    th = math.radians(rot_z_deg)
    c, s = math.cos(th), math.sin(th)
    return c * bx + s * by, -s * bx + c * by


def rail_fraction(x: np.ndarray, y: np.ndarray, z: np.ndarray, full_scale: float,
                  rot_z_deg: float, frac: float) -> tuple[float, str]:
    """Fraction of samples with any sensor axis at the rail, and which axis
    did it most."""
    sx, sy = to_sensor_axes(x, y, rot_z_deg)
    lim = full_scale * frac
    hits = {"X": np.abs(sx) >= lim, "Y": np.abs(sy) >= lim, "Z": np.abs(z) >= lim}
    any_hit = hits["X"] | hits["Y"] | hits["Z"]
    if x.size == 0:
        return 0.0, ""
    worst = max(hits, key=lambda k: int(hits[k].sum()))
    return float(np.mean(any_hit)), worst


# ---------------------------------------------------------------------------
# Phases
# ---------------------------------------------------------------------------

def _imu(recs, t0):
    imu = recs.get("ISM6HG256") or []
    if not imu or t0 is None or "time_us" not in imu[0]:
        return None
    t = (get_array(imu, "time_us") - t0) / 1e6
    prefix = "high_acc" if all(f"high_acc_{a}" in imu[0] for a in "xyz") else "low_acc"
    if not all(f"{prefix}_{a}" in imu[0] for a in "xyz"):
        return None
    out = {"t": t, "prefix": prefix}
    for a in "xyz":
        out[f"a{a}"] = get_array(imu, f"{prefix}_{a}")
    if all(f"low_acc_{a}" in imu[0] for a in "xyz"):
        for a in "xyz":
            out[f"l{a}"] = get_array(imu, f"low_acc_{a}")
    if all(f"gyro_{a}" in imu[0] for a in "xyz"):
        for a in "xyz":
            out[f"g{a}"] = get_array(imu, f"gyro_{a}")
    return out


def phases(flight: Flight, t_first: float, t_last: float) -> list[tuple[str, float, float, str]]:
    """(name, start, end, how the window was bounded), in flight order."""
    ev = measured(flight)
    launch = markers(flight).get("launch")
    out: list[tuple[str, float, float, str]] = []
    if launch is None:
        out.append(("Whole log", t_first, t_last, "no launch found, so the whole record"))
        return out
    pad_lo, pad_hi = max(t_first, launch - _PAD_BEFORE_S), launch - _PAD_GUARD_S
    if pad_hi - pad_lo >= 0.3:
        out.append(("Pad", pad_lo, pad_hi, "the 1.5 s before first motion"))
    if ev["burnout"] is not None:
        out.append(("Boost", launch, ev["burnout"], "first motion to thrust ending"))
        if ev["apogee"] is not None and ev["apogee"] > ev["burnout"]:
            out.append(("Coast", ev["burnout"], ev["apogee"], "burnout to apogee"))
    elif ev["apogee"] is not None:
        out.append(("Boost", launch, ev["apogee"], "first motion to apogee (no burnout found)"))
    else:
        out.append(("Boost", launch, t_last, "first motion to the end of the log (no burnout found)"))
    top = ev["apogee"] if ev["apogee"] is not None else ev["burnout"]
    if top is not None and ev["landed"] is not None and ev["landed"] > top:
        out.append(("Descent", top, ev["landed"], "apogee to touchdown"))
    return out


def phase_stats(u: dict[str, Any], skip_start_s: float = 0.0) -> Optional[dict[str, Any]]:
    """Per-axis numbers for one uniformly sampled window. `skip_start_s`
    excludes the opening of the window from the kurtosis only."""
    fs = u["fs"]
    stats: dict[str, Any] = {"fs": fs, "n": u["t"].size, "axes": {}}
    k0 = min(int(round(skip_start_s * fs)), max(0, u["t"].size - _MIN_SAMPLES))
    for a in "xyz":
        x = u["chans"][f"a{a}"]
        w = welch(x, fs)
        if w is None:
            return None
        f, p = w
        hp = highpass(x[k0:], fs, _HP_HZ)
        stats["axes"][a] = {
            "rms_hp": band_rms(f, p, _HP_HZ, None),
            "bands": [band_rms(f, p, lo, hi) for lo, hi in _BANDS],
            "tone": dominant_tone(f, p),
            "kurtosis": kurtosis(hp),
            "peak": float(np.max(np.abs(x))),
            "psd": (f, p),
        }
    return stats


# ---------------------------------------------------------------------------
# Presentation
# ---------------------------------------------------------------------------

def _fmt_axes(ax: dict, key: str, places: int = 1) -> str:
    return " / ".join(f"{ax[a][key]:.{places}f}" for a in "xyz")


def _fmt_bands(vals: list[float]) -> str:
    labels = [f"{int(lo)}–{int(hi)}" if hi is not None else f"{int(lo)}+" for lo, hi in _BANDS]
    return " · ".join(f"{lab}: {v:.1f}" for lab, v in zip(labels, vals))


def _tone_text(ax: dict) -> str:
    parts = []
    tx = ax["x"]["tone"]
    if tx and ax["x"]["rms_hp"] >= _TONE_MIN_RMS:
        parts.append(f"longitudinal {tx[0]:.0f} Hz ({tx[1] * 100:.0f} % of the energy above {_HP_HZ:.0f} Hz)")
    lat = max(("y", "z"), key=lambda a: ax[a]["rms_hp"])
    tl = ax[lat]["tone"]
    if tl and ax[lat]["rms_hp"] >= _TONE_MIN_RMS:
        parts.append(f"lateral {tl[0]:.0f} Hz ({tl[1] * 100:.0f} %, {lat.upper()})")
    return " · ".join(parts) if parts else "none — at the sensor floor"


def _knocking_axis(ax: dict) -> Optional[str]:
    """The axis whose >100 Hz content is impact-like AND loud enough to matter."""
    loud = [a for a in "xyz" if ax[a]["rms_hp"] >= _KNOCK_MIN_RMS and ax[a]["kurtosis"] >= _IMPACT_KURTOSIS]
    return max(loud, key=lambda a: ax[a]["kurtosis"]) if loud else None


def _character(ax: dict) -> str:
    knock = _knocking_axis(ax)
    if knock:
        return f"impact-like on {knock.upper()} — something knocking, not just shaking"
    if max(ax[a]["rms_hp"] for a in "xyz") < _TONE_MIN_RMS:
        return "at the sensor floor"
    if max(ax[a]["kurtosis"] for a in "xyz") >= 3.0:
        return "mostly steady, with some transients"
    return "steady oscillation"


def _envelope_chart(imu, lo: float, hi: float, events) -> Optional[dict[str, Any]]:
    m = (imu["t"] >= lo) & (imu["t"] <= hi)
    if m.sum() < _MIN_SAMPLES:
        return None
    u = uniform(imu["t"][m], {f"a{a}": imu[f"a{a}"][m] for a in "xyz"})
    if u is None:
        return None
    traces = []
    names = {"x": "X (longitudinal)", "y": "Y (lateral)", "z": "Z (lateral)"}
    for k, a in enumerate("xyz"):
        idx, rms = envelope(u["chans"][f"a{a}"], u["fs"])
        t = u["t"][0] + idx / u["fs"]
        traces.append(trace(t, np.maximum(rms, 1e-3), names[a], COLORS[k], mode="lines", width=1.2))
    spec = chart("chart-vibration-envelope", "Vibration envelope through the burn", traces,
                 y_title="RMS above 20 Hz, 20 ms windows", y_unit="m/s²", events=events, height=360)
    if not spec:
        return None
    spec["layout"]["yaxis"]["type"] = "log"
    spec["note"] = (spec["note"] + " " if spec["note"] else "") + (
        "How hard each axis shook, 20 ms at a time, on a log scale. Vibration that follows "
        "the thrust curve is the airframe answering the motor; vibration that grows while "
        "thrust falls, or that stops while the motor is still burning, is the motor's own "
        "combustion oscillation. A spike with nothing around it is a knock."
    )
    return spec


def _spectrum_chart(stats: dict[str, Any], title_phase: str) -> Optional[dict[str, Any]]:
    traces = []
    names = {"x": "X (longitudinal)", "y": "Y (lateral)", "z": "Z (lateral)"}
    for k, a in enumerate("xyz"):
        f, p = stats["axes"][a]["psd"]
        keep = f >= 1.0
        traces.append(trace(f[keep], np.maximum(p[keep], 1e-6), names[a], COLORS[k], mode="lines", width=1.1))
    spec = chart("chart-vibration-spectrum", f"{title_phase} spectrum", traces,
                 x_title="Frequency (Hz)", y_title="Power spectral density ((m/s²)²/Hz)", height=340)
    if not spec:
        return None
    spec["layout"]["yaxis"]["type"] = "log"
    spec["note"] = (spec["note"] + " " if spec["note"] else "") + (
        "Where the energy sits. A narrow peak is a tone — the motor's chamber, or a mount "
        "ringing; a broad hump is a structure answering broadband noise. The 2–50 Hz region "
        "is airframe bending, 200–500 Hz is usually the sled, 500–1000 Hz the motor."
    )
    return spec


def boost_summary(flight: Flight) -> Optional[dict[str, Any]]:
    """The summary-card cell: longitudinal RMS above 100 Hz in boost, with the
    dominant tone and the gyro rail in the hint."""
    imu = _imu(flight.records, flight.t0_us)
    if imu is None:
        return None
    ph = [p for p in phases(flight, float(imu["t"][0]), float(imu["t"][-1])) if p[0] == "Boost"]
    if not ph:
        return None
    _, lo, hi, _how = ph[0]
    m = (imu["t"] >= lo) & (imu["t"] <= hi)
    if m.sum() < _MIN_SAMPLES:
        return None
    u = uniform(imu["t"][m], {f"a{a}": imu[f"a{a}"][m] for a in "xyz"})
    if u is None:
        return None
    st = phase_stats(u)
    if st is None:
        return None
    ax = st["axes"]
    hint = f"above {_HP_HZ:.0f} Hz, longitudinal, under thrust"
    tx = ax["x"]["tone"]
    if tx:
        hint += f" · dominant {tx[0]:.0f} Hz"
    if all(f"g{a}" in imu for a in "xyz"):
        fs_dps = float(flight.config.get("gyro_fs_dps") or 0)
        rot = float(flight.config.get("ism6_rot_z_deg") or 0.0)
        if fs_dps > 0:
            frac, _ = rail_fraction(imu["gx"][m], imu["gy"][m], imu["gz"][m], fs_dps, rot, _RAIL_GYRO)
            if frac > 0:
                hint += f" · gyro at full scale {frac * 100:.1f} % of samples"
    return {"rms_hp_x": ax["x"]["rms_hp"], "hint": hint}


# ---------------------------------------------------------------------------
# Section
# ---------------------------------------------------------------------------

def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="vibration", title="Vibration")
    recs = flight.records
    imu = _imu(recs, flight.t0_us)
    if imu is None:
        result.warnings.append("No IMU records, so there is no vibration to measure.")
        return result

    t_first, t_last = float(imu["t"][0]), float(imu["t"][-1])
    metrics: dict[str, Any] = {}
    events = {k: round(v, 3) for k, v in markers(flight).items() if v is not None}
    rot = float(flight.config.get("ism6_rot_z_deg") or 0.0)

    sensor = ("±256 g accelerometer" if imu["prefix"] == "high_acc"
              else "±16 g accelerometer only — content above its rail is clipped")
    boost_stats: Optional[dict[str, Any]] = None
    boost_mask = None
    boost_how = ""

    for name, lo, hi, how in phases(flight, t_first, t_last):
        m = (imu["t"] >= lo) & (imu["t"] <= hi)
        if m.sum() < _MIN_SAMPLES:
            continue
        u = uniform(imu["t"][m], {f"a{a}": imu[f"a{a}"][m] for a in "xyz"})
        if u is None:
            continue
        st = phase_stats(u, skip_start_s=_RAIL_EXIT_S if name == "Boost" else 0.0)
        if st is None:
            continue
        ax = st["axes"]
        key = name
        if name == "Boost":
            boost_stats, boost_mask, boost_how = st, m, how
            metrics["Sensor"] = (f"{sensor}, {st['fs']:.0f} Hz, snapped to its own clock "
                                 f"({u['filled']} dropped slots filled)")
            metrics["Boost window"] = f"{how}, {hi - lo:.2f} s"
        metrics[f"{key} · RMS above {_HP_HZ:.0f} Hz, X / Y / Z"] = _fmt_axes(ax, "rms_hp") + " m/s²"
        metrics[f"{key} · dominant tone"] = _tone_text(ax)
        peak = max(ax[a]["peak"] for a in "xyz")
        metrics[f"{key} · peak |a|, X / Y / Z"] = (_fmt_axes(ax, "peak", 0)
                                                    + f" m/s² ({peak / G:.0f} g)")
        if name == "Boost":
            lat = [math.sqrt(ax["y"]["bands"][i] ** 2 + ax["z"]["bands"][i] ** 2)
                   for i in range(len(_BANDS))]
            metrics[f"{key} · band RMS, longitudinal (Hz: m/s²)"] = _fmt_bands(ax["x"]["bands"])
            metrics[f"{key} · band RMS, lateral (Hz: m/s²)"] = _fmt_bands(lat)
            metrics[f"{key} · kurtosis above {_HP_HZ:.0f} Hz, X / Y / Z"] = (
                _fmt_axes(ax, "kurtosis") + f" (after the first {_RAIL_EXIT_S:.2f} s) — {_character(ax)}")

    if boost_stats is None:
        if not metrics:
            result.warnings.append("Not enough IMU samples in any flight phase to measure vibration.")
        result.metrics = metrics
        return result

    # Rails, boost only: that is where thrust puts the ±16 g part at its limit,
    # and where a railed gyro corrupts the attitude for the rest of the flight.
    if all(f"l{a}" in imu for a in "xyz"):
        fs_g = float(flight.config.get("low_g_fs_g") or 0)
        if fs_g > 0:
            frac, worst = rail_fraction(imu["lx"][boost_mask], imu["ly"][boost_mask], imu["lz"][boost_mask],
                                        fs_g * G, rot, _RAIL_LOW_G)
            metrics["Boost · ±16 g accelerometer at its rail"] = (
                f"{frac * 100:.1f} % of samples" + (f" (mostly {worst}, sensor axes)" if frac > 0 else ""))
    if all(f"g{a}" in imu for a in "xyz"):
        fs_dps = float(flight.config.get("gyro_fs_dps") or 0)
        if fs_dps > 0:
            frac, worst = rail_fraction(imu["gx"][boost_mask], imu["gy"][boost_mask], imu["gz"][boost_mask],
                                        fs_dps, rot, _RAIL_GYRO)
            metrics["Boost · gyro at full scale"] = (
                f"{frac * 100:.2f} % of samples" + (f" (mostly {worst}, sensor axes)" if frac > 0 else ""))
            if frac > 0:
                sx, sy = to_sensor_axes(imu["gx"][boost_mask], imu["gy"][boost_mask], rot)
                lim = fs_dps * _RAIL_GYRO
                hit = (np.abs(sx) >= lim) | (np.abs(sy) >= lim) | (np.abs(imu["gz"][boost_mask]) >= lim)
                t_hit = float(imu["t"][boost_mask][int(np.argmax(hit))])
                result.warnings.append(
                    f"The gyro hit full scale on {frac * 100:.2f} % of boost samples, first at "
                    f"{t_hit:.2f} s. A railed sample is not a measurement, but the attitude filter "
                    f"integrates it as one — treat pitch and yaw from that moment on as suspect.")

    ax = boost_stats["axes"]
    worst = _knocking_axis(ax)
    if worst is not None:
        result.warnings.append(
            f"Boost vibration on {worst.upper()} is impact-like (kurtosis {ax[worst]['kurtosis']:.0f}): "
            f"something was knocking, not just shaking. Check for a loose sled, battery or ballast, "
            f"and the fit of any joint near the computer.")

    result.metrics = metrics
    ev = measured(flight)
    launch = markers(flight).get("launch")
    if launch is not None:
        lo = max(t_first, launch - 0.5)
        end = ev["burnout"] if ev["burnout"] is not None else (ev["apogee"] if ev["apogee"] is not None else t_last)
        hi = min(t_last, end + 0.5)
    else:
        lo, hi = t_first, t_last
    result.charts = [c for c in (_envelope_chart(imu, lo, hi, events),
                                 _spectrum_chart(boost_stats, "Boost")) if c]
    return result
