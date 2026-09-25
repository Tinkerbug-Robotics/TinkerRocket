#!/usr/bin/env python3
"""
replay_imu_drain.py -- #1191: what the EKF and the roll controller are fed,
before and after averaging the drained IMU samples.

The flight loop drains every queued ISM6HG256 sample each pass (~500 Hz) and
used to convert only the freshest one for the estimators: a 1-in-N decimation
with no anti-alias step, so the 2026-08-29 boost's ~800 Hz motor tone reached
the EKF and the roll P-term folded to ~190 Hz at full amplitude.  #1191 hands
them the window MEAN instead, an N-tap boxcar at the ODR.

This replays a recorded ODR-rate stream through the REAL accumulator
(imu_drain_window.h, compiled on the fly via _imu_drain_shim.cpp) at the
flight's own EKF tick rate, and compares the two ~500 Hz streams the
estimators would have seen: the freshest sample (as flown) and the window
mean (as of #1191).  The tick grid is uniform at the rate derived from the
logged ekf_ticks counter; the real loop jitters around it, which smears the
alias lines a little but does not move the band figures.

Usage:
    python3 replay_imu_drain.py FLIGHT.bin [--rate-hz 496] [--plot-dir DIR]

Per analysis window (pad / boost / coast) and per channel (high-g X, low-g X,
gyro X = the roll-rate input), two tables:

  "as the estimators see it" -- the two tick-rate streams themselves: their
  sigma, the RMS in the 100 Hz-Nyquist band, and the strongest line above
  100 Hz as flown.  Both streams contain real sub-Nyquist content the boxcar
  rightly leaves alone, so this table shows the net effect, not the filter.

  "band-passed through the same windows" -- the filter itself.  The full-rate
  stream is brick-wall band-passed (the 700-900 Hz tone, 25-35 Hz airframe
  bending, everything above the tick Nyquist), then decimated BOTH ways with
  exactly the windows the accumulator formed.  Each decimation's RMS over the
  ODR-rate band RMS is its gain on that band alone: the acceptance numbers in
  #1191 (tone down >= 12 dB, 30 Hz within 1 %) are read from the mean's.
  The band-pass is an FFT mask, which treats its segment as a circle, so each
  window is band-passed with a quarter second of context on either side (its
  margin) to keep the wrap and the edge ringing off the measured ticks.  The
  margin is the window's own run wherever that has samples.  Where it has
  none, the margin is made up and the row says how (band_segment).

Plus the mean samples per tick (the N behind the boxcar) and the low-g
near-rail duty: the old body-frame test on the freshest sample against the
new per-sensor-axis test on the window's worst sample.

Each window's ODR-rate work runs at that window's own ODR.  A log flown at
the DYNAMIC IMU rate changes ODR in flight: the FC reprograms the chip at
deployment (applyImuRateForFlightPhase), and the 2026-08-29 log steps from
3817 Hz to 952 Hz at T+3.049 s.  So the stream is cut into runs at one rate,
a window is analysed at the median-spacing ODR of the run it lies in, and a
window that spans a step is split at it, with a note saying which parts were
analysed and which were not.
"""

import argparse
import ctypes
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from plot_flight_data_mini import parse_binary_file, get_array, quat_to_matrix, ism6_scales  # noqa: E402
from replay_flight_ekf import derive_ekf_rate_hz  # noqa: E402

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
_COLL_DIR = _REPO / "tinkerrocket-idf" / "components" / "TR_Sensor_Collector"
_TYPES_DIR = _REPO / "tinkerrocket-idf" / "components" / "TR_RocketComputerTypes"
_SHIM_SRC = _HERE / "_imu_drain_shim.cpp"

G_MS2 = 9.80665
NEAR_RAIL_MARGIN_G = 0.5

# Analysis windows in seconds since the launch flag (VIBRATION_ANALYSIS.md
# table 1 uses the same boost and coast spans).
WINDOWS = [("pad", -1.5, -0.1), ("boost", 0.0, 0.75), ("coast", 3.0, 10.0)]
# (label, column in the 9-wide body-frame array, unit)
CHANNELS = [("high-g X", 3, "m/s2"), ("low-g X", 0, "m/s2"), ("gyro X", 6, "dps")]
# Bands the filter is measured on; None means "the tick Nyquist to ODR/2".
BANDS = [("tone 700-900 Hz", 700.0, 900.0), ("bending 25-35 Hz", 25.0, 35.0), ("above tick Nyquist", None, None)]
# The ODR-rate PSD length the tone is read from, and the band it is looked for in.
NPERSEG_ODR = 1024
TONE_SEARCH_HZ = (500.0, 1000.0)
# Fewest ticks a window, or the part of one at a single ODR, needs to be analysed.
MIN_TICKS = 64
# Rate runs: the ISM6HG256 ODR ladder (IMU_RATE_OPTIONS_HZ is 960 Hz x 2^k), the
# intervals in the median that sets each interval's local rate, and the fewest
# intervals a stretch on one rung needs to count as a rate, not the switch.
ODR_LADDER_HZ = 960.0
RATE_MEDIAN_W = 33
RATE_RUN_MIN = 64

# NumPy 2.4 dropped np.trapz; older installs lack np.trapezoid.
_trapz = getattr(np, "trapezoid", None) or getattr(np, "trapz")


# ----------------------------------------------------------------------------
# The firmware accumulator, via the shim
# ----------------------------------------------------------------------------

def _build_shim() -> Path:
    """Compile the shim if older than any firmware source it pulls in (no-drift guarantee)."""
    out = _HERE / "_imu_drain_shim.so"
    deps = [_SHIM_SRC, _COLL_DIR / "imu_drain_window.h", _TYPES_DIR / "RocketComputerTypes.h"]
    if out.exists() and all(out.stat().st_mtime >= d.stat().st_mtime for d in deps):
        return out
    subprocess.run(["c++", "-O2", "-std=c++17", "-shared", "-fPIC",
                    f"-I{_COLL_DIR}", f"-I{_TYPES_DIR}",
                    str(_SHIM_SRC), "-o", str(out)],
                   check=True)
    return out


def near_rail_lsb(fs_g: float, margin_g: float = NEAR_RAIL_MARGIN_G) -> int:
    """imu_drain::nearRailLsb, mirrored: (FS - margin) / FS * 32768 in raw LSB."""
    return int(((fs_g - margin_g) / fs_g) * 32768.0)


class DrainReplay:
    """The shipped drain-window accumulator, driven over a sample stream."""

    def __init__(self):
        self._lib = ctypes.CDLL(str(_build_shim()))
        U64 = ctypes.POINTER(ctypes.c_ulonglong)
        I16 = ctypes.POINTER(ctypes.c_short)
        U32 = ctypes.POINTER(ctypes.c_uint)
        I32 = ctypes.POINTER(ctypes.c_int)
        U8 = ctypes.POINTER(ctypes.c_ubyte)
        self._lib.tr_imu_drain_replay.restype = ctypes.c_int
        self._lib.tr_imu_drain_replay.argtypes = [
            ctypes.c_int, U64, I16, I16, I16,
            ctypes.c_int, U64, ctypes.c_int,
            U32, U32, I16, I16, I32, U8]

    def run(self, t_us, raw9, tick_us, bar_lsb):
        """raw9: (n, 9) int16 [lg xyz, hg xyz, gy xyz]; returns dict of per-tick arrays."""
        n, k = len(t_us), len(tick_us)
        arr = lambda a, ct: np.ascontiguousarray(a, dtype=ct).ctypes.data_as(ctypes.POINTER(ct))
        lg = np.ascontiguousarray(raw9[:, 0:3], dtype=np.int16)
        hg = np.ascontiguousarray(raw9[:, 3:6], dtype=np.int16)
        gy = np.ascontiguousarray(raw9[:, 6:9], dtype=np.int16)
        out_n = np.zeros(k, dtype=np.uint32)
        out_stamp = np.zeros(k, dtype=np.uint32)
        out_mean = np.zeros((k, 9), dtype=np.int16)
        out_last = np.zeros((k, 9), dtype=np.int16)
        out_idx = np.zeros(k, dtype=np.int32)
        out_rail = np.zeros(k, dtype=np.uint8)
        consumed = self._lib.tr_imu_drain_replay(
            n, arr(t_us, ctypes.c_ulonglong),
            lg.ctypes.data_as(ctypes.POINTER(ctypes.c_short)),
            hg.ctypes.data_as(ctypes.POINTER(ctypes.c_short)),
            gy.ctypes.data_as(ctypes.POINTER(ctypes.c_short)),
            k, arr(tick_us, ctypes.c_ulonglong), int(bar_lsb),
            out_n.ctypes.data_as(ctypes.POINTER(ctypes.c_uint)),
            out_stamp.ctypes.data_as(ctypes.POINTER(ctypes.c_uint)),
            out_mean.ctypes.data_as(ctypes.POINTER(ctypes.c_short)),
            out_last.ctypes.data_as(ctypes.POINTER(ctypes.c_short)),
            out_idx.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
            out_rail.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)))
        return dict(consumed=consumed, n=out_n, stamp=out_stamp, mean=out_mean,
                    last=out_last, last_idx=out_idx, near_rail=out_rail.astype(bool))


# ----------------------------------------------------------------------------
# Log -> raw counts
# ----------------------------------------------------------------------------

def unwrap_u32_us(t):
    """uint32 micros() -> monotonic int64 (the FC clock wraps every 71.6 min)."""
    t = np.asarray(t, dtype=np.int64)
    d = np.diff(t)
    wraps = np.cumsum(np.concatenate([[0], (d < -(1 << 31)).astype(np.int64)]))
    return t + wraps * (1 << 32)


def conversion_matrices(cfg):
    """The affine maps SensorConverter applies, as the parser mirrors them.

    Column-vector form:
        body_low  = B2R . Rz . (raw * s_low)
        body_high = B2R . (Rz . (raw * s_high) - bias)
        body_gyro = B2R . Rz . (raw * s_gyro)
    """
    s_low, s_high, s_gyro = ism6_scales(cfg["low_g_fs_g"], cfg["high_g_fs_g"], cfg["gyro_fs_dps"])
    r = math.radians(cfg["ism6_rot_z_deg"])
    c, s = math.cos(r), math.sin(r)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    b2r = np.array(quat_to_matrix(cfg["b2r_quat"]), dtype=float) if cfg.get("b2r_quat") else np.eye(3)
    bias = np.array(cfg.get("hg_bias") or (0.0, 0.0, 0.0), dtype=float)
    return dict(s_low=s_low, s_high=s_high, s_gyro=s_gyro, rz=rz, b2r=b2r, bias=bias)


def to_body(raw9, cm):
    """Raw counts -> body-frame SI: the affine map the converter applies to the mean.

    Rows are samples, so a column-vector product M . v is v_row @ M.T here.
    """
    r = raw9.astype(float)
    low = (r[:, 0:3] * cm["s_low"]) @ cm["rz"].T @ cm["b2r"].T
    high = ((r[:, 3:6] * cm["s_high"]) @ cm["rz"].T - cm["bias"]) @ cm["b2r"].T
    gyro = (r[:, 6:9] * cm["s_gyro"]) @ cm["rz"].T @ cm["b2r"].T
    return np.column_stack([low, high, gyro])


def recover_raw(imu, cm):
    """Invert the parser's conversion back to the logged int16 counts.

    Column form: raw * s = Rz^T . (B2R^T . body + bias); in row form that is
    (body_row @ B2R + bias) @ Rz.  Rotation inverses are transposes, so the
    forward map's transposed factors come back untransposed here.

    The conversion is affine and the counts are integers, so the inverse lands
    on integers to floating-point precision.  Integer-ness alone is not proof
    (a 90-degree slip keeps integers integers), so the recovered counts are also
    pushed forward again and must reproduce the parser's values.  Anything else
    means the config the parser applied is not the one it reported, and we
    refuse rather than analyse a stream that is not the vehicle's.
    """
    b2r, rz = cm["b2r"], cm["rz"]
    low = np.column_stack([get_array(imu, k) for k in ("low_acc_x", "low_acc_y", "low_acc_z")])
    high = np.column_stack([get_array(imu, k) for k in ("high_acc_x", "high_acc_y", "high_acc_z")])
    gyro = np.column_stack([get_array(imu, k) for k in ("gyro_x", "gyro_y", "gyro_z")])
    raw_low = (low @ b2r @ rz) / cm["s_low"]
    raw_high = ((high @ b2r) + cm["bias"]) @ rz / cm["s_high"]
    raw_gyro = (gyro @ b2r @ rz) / cm["s_gyro"]
    raw = np.column_stack([raw_low, raw_high, raw_gyro])
    err = np.max(np.abs(raw - np.round(raw)))
    if err > 0.01:
        sys.exit(f"raw-count recovery is off by {err:.3f} LSB: the parser's config is not "
                 f"the one it applied, refusing to analyse a stream that is not the vehicle's")
    raw = np.round(raw).astype(np.int16)
    back = to_body(raw, cm)
    rt = np.max(np.abs(back - np.column_stack([low, high, gyro])))
    if rt > 1e-6:
        sys.exit(f"raw-count round trip is off by {rt:.3g}: the inverse does not undo the parser")
    return raw


# ----------------------------------------------------------------------------
# Rate runs
# ----------------------------------------------------------------------------

def rate_runs(t_us, t_launch):
    """Split the samples into runs at one ODR: [dict(i0, i1, odr, t0, t1)].

    A run is samples [i0, i1), from T+t0 to T+t1 s, at the ODR its own median
    spacing gives.  Each interval's local spacing is the median of the
    RATE_MEDIAN_W around it, which a dropped sample or a late read does not
    move, and it goes on the nearest rung of the ODR ladder.  The rungs are 2x
    apart and the oscillator is within a few % of nominal, so a rate would have
    to be ~30 % off its rung to land on the next.

    A stretch on one rung shorter than RATE_RUN_MIN intervals is not a rate.
    Between two runs on one rung it is a burst of drops and joins them, and at
    either end of the span it joins the run beside it.  Between two rates it is
    the switch (the 736 us gap at T+3.049 s on 2026-08-29) and belongs to
    neither.  So a fixed-rate log is always one run over every interval, and its
    ODR is the median over the whole span, as it always was.
    """
    t_us = np.asarray(t_us, dtype=np.int64)
    dt = np.diff(t_us)
    spans = [(0, len(dt))]
    if len(dt) >= RATE_MEDIAN_W:
        h = RATE_MEDIAN_W // 2
        local = np.median(np.lib.stride_tricks.sliding_window_view(dt, RATE_MEDIAN_W), axis=1)
        local = np.concatenate([np.full(h, local[0]), local, np.full(h, local[-1])])
        rung = np.round(np.log2(1e6 / local / ODR_LADDER_HZ)).astype(int)
        cuts = np.flatnonzero(np.diff(rung)) + 1
        merged = []
        for a, b in zip(np.concatenate([[0], cuts]), np.concatenate([cuts, [len(dt)]])):
            if b - a < RATE_RUN_MIN:
                continue
            if merged and merged[-1][2] == rung[a]:
                merged[-1][1] = b
            else:
                merged.append([a, b, rung[a]])
        if merged:
            merged[0][0], merged[-1][1] = 0, len(dt)
            spans = [(a, b) for a, b, _ in merged]
    # Intervals [a, b) join samples a..b.  A sample two runs share goes to the later one.
    bounds = [[int(a), int(b) + 1] for a, b in spans]
    for prev, nxt in zip(bounds, bounds[1:]):
        prev[1] = min(prev[1], nxt[0])
    return [dict(i0=i0, i1=i1, odr=1e6 / np.median(dt[a:b]),
                 t0=(t_us[i0] - t_launch) / 1e6, t1=(t_us[i1 - 1] - t_launch) / 1e6)
            for (i0, i1), (a, b) in zip(bounds, spans)]


def runs_text(runs):
    """'3817 Hz to T+3.048 s, 952 Hz from T+3.049 s'; a single run is its ODR alone."""
    last = len(runs) - 1
    return ", ".join(f"{run['odr']:.0f} Hz" + (f" from T+{run['t0']:.3f} s" if k else "")
                     + (f" to T+{run['t1']:.3f} s" if k < last else "") for k, run in enumerate(runs))


def tick_runs(r, runs):
    """Per tick, the index of the run that holds every sample it drained, else -1."""
    last = r["last_idx"].astype(np.int64)
    first = last - r["n"].astype(np.int64) + 1
    out = np.full(len(last), -1)
    for k, run in enumerate(runs):
        out[(r["n"] > 0) & (first >= run["i0"]) & (last < run["i1"])] = k
    return out


def split_at_rate_steps(name, win, r, tick_run, runs, t_rel):
    """The window as parts at one ODR each, [(label, ticks, run)], and the notes to print.

    A window inside one run comes back whole under its own name, whatever its
    length.  One that spans a rate step is split there.  Each part with
    MIN_TICKS ticks is analysed at its own ODR, under the window's name when it
    is the only one, and the notes say which parts were analysed and which were
    not.  A tick that drains samples from both sides of the step is in neither
    part: its mean mixes two rates.
    """
    ids = tick_run[win]
    if not win.any():
        return [(name, win, None)], []
    if ids[0] >= 0 and (ids == ids[0]).all():
        return [(name, win, runs[ids[0]])], []
    idx, n = r["last_idx"][win], r["n"][win]
    a, b = int(idx[0]) - int(n[0]) + 1, int(idx[-1]) + 1
    touched = [run for run in runs if run["i0"] < b and run["i1"] > a]
    notes = [f"[{name}] spans an ODR step ({runs_text(touched)}): split there, each part at its own ODR"]
    parts = []
    for k in dict.fromkeys(ids[ids >= 0].tolist()):
        sel = win & (tick_run == k)
        t = t_rel[sel]
        say = f"  T+{t[0]:.3f}..{t[-1]:.3f} s at {runs[k]['odr']:.0f} Hz: {len(t)} ticks, "
        if len(t) < MIN_TICKS:
            notes.append(say + f"too few to analyse (fewer than {MIN_TICKS})")
        else:
            notes.append(say + "analysed below")
            parts.append((sel, runs[k]))
    mixed = int((ids < 0).sum())
    if mixed:
        notes.append(f"  {mixed} tick{'s drain' if mixed > 1 else ' drains'} samples from both sides "
                     f"of the step: in neither part")
    labels = [name] if len(parts) == 1 else [f"{name}-{i + 1}" for i in range(len(parts))]
    return [(label, sel, run) for label, (sel, run) in zip(labels, parts)], notes


# ----------------------------------------------------------------------------
# Spectra and bands
# ----------------------------------------------------------------------------

def welch(x, fs, nperseg):
    """Hann-windowed averaged periodogram, 50 % overlap, one-sided, mean removed."""
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    step = nperseg // 2
    win = np.hanning(nperseg)
    scale = fs * np.sum(win ** 2)
    segs = [x[i:i + nperseg] for i in range(0, len(x) - nperseg + 1, step)]
    if not segs:
        return None, None
    psd = np.zeros(nperseg // 2 + 1)
    for s in segs:
        spec = np.fft.rfft(s * win)
        psd += (np.abs(spec) ** 2) / scale
    psd /= len(segs)
    psd[1:-1] *= 2.0
    f = np.fft.rfftfreq(nperseg, 1.0 / fs)
    return f, psd


def band_rms(f, psd, lo, hi):
    m = (f >= lo) & (f <= hi)
    if not m.any():
        return 0.0
    return math.sqrt(_trapz(psd[m], f[m]))


def bandpass(x, fs, lo, hi):
    """Brick-wall band-pass by FFT masking; the segment carries margins
    (band_segment) so the edge ringing stays outside the ticks that are measured."""
    x = np.asarray(x, dtype=float)
    spec = np.fft.rfft(x - x.mean())
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    spec[(f < lo) | (f > hi)] = 0.0
    return np.fft.irfft(spec, n=len(x))


def across_step(d, k, before, count):
    """Up to `count` samples continuing run k across its rate step, on run k's spacing.

    The step changes the rate, not the signal, so the samples beyond it are
    real context.  When they are the faster run they are first averaged over
    the rate ratio, so the slower grid does not alias them; then they are
    interpolated onto run k's spacing.  Only the grid points the neighbouring
    run covers come back.
    """
    run, nb = d["runs"][k], d["runs"][k - 1 if before else k + 1]
    t = d["t_us"][nb["i0"]:nb["i1"]].astype(float)
    x = to_body(d["raw"][nb["i0"]:nb["i1"]], d["cm"])
    w = int(round(nb["odr"] / run["odr"]))
    if w > 1:
        # A centred w-sample mean whose window shrinks at the ends, so the
        # samples next to the step are not averaged with zeros.
        cs = np.vstack([np.zeros((1, x.shape[1])), np.cumsum(x, axis=0)])
        start = np.arange(len(x)) - w // 2
        lo, hi = np.clip(start, 0, len(x)), np.clip(start + w, 0, len(x))
        x = (cs[hi] - cs[lo]) / (hi - lo)[:, None]
    dt = 1e6 / run["odr"]
    if before:
        tq = d["t_us"][run["i0"]] - dt * np.arange(count, 0, -1)
        tq = tq[tq >= t[0]]
    else:
        tq = d["t_us"][run["i1"] - 1] + dt * np.arange(1, count + 1)
        tq = tq[tq <= t[-1]]
    return np.column_stack([np.interp(tq, t, x[:, c]) for c in range(x.shape[1])])


def band_segment(d, run, a, b, margin, name, pre_launch):
    """Samples [a, b) of `run` with `margin` samples of context each side, body frame.

    Returns (seg, i0, notes): the rows, the stream index the first row stands
    for, and a line for each side whose margin had to be made up.  The margin
    is the run's own samples wherever it has them.  Where it does not:

    - Past a rate step, the samples beyond it (across_step).  The FC steps the
      ODR at deployment, so a run after a step starts on the ejection shock
      (on 2026-08-29 its first samples are the 1102-1162 m/s2 peaks), and a
      mirror image would double the shock.
    - Past the log's first or last sample, the samples at the edge, mirrored.
    - A pre-launch window takes none past its own end and mirrors instead.
      What follows the pad is the motor start (the launch flag trails first
      motion by up to 0.4 s), and in the pad's segment it put a 30-70 m/s2
      step at the FFT's wrap, which rang into the pad's gains by up to 16 dB.
    """
    runs = d["runs"]
    k = runs.index(run)
    i0 = max(run["i0"], a - margin)
    i1 = b if pre_launch else min(run["i1"], b + margin)
    seg = to_body(d["raw"][i0:i1], d["cm"])
    notes, mirror = [], [0, 0]
    for side, have in (("before", a - i0), ("after", i1 - b)):
        short = margin - have
        if short <= 0:
            continue
        before = side == "before"
        head = f"band-pass margin {side}: {have} of {margin} samples"
        nb = k - 1 if before else k + 1          # the run beyond this edge, if there is one
        if pre_launch and not before:
            notes.append(f"band-pass margin after: none (past the {name} is the motor start); all mirrored")
        elif 0 <= nb < len(runs):
            fill = across_step(d, k, before, short)
            seg = np.vstack([fill, seg] if before else [seg, fill])
            if before:
                i0 -= len(fill)
            short -= len(fill)
            step = runs[k if before else nb]["t0"]
            notes.append(f"{head} (a rate step at T{step:+.3f} s); the rest from the {runs[nb]['odr']:.0f} Hz "
                         f"run, resampled" + (f"; {short} more mirrored" if short else ""))
        else:
            edge = (d["t_us"][0 if before else -1] - d["t_launch"]) / 1e6
            notes.append(f"{head} (no samples {side} T{edge:+.3f} s); {'the rest' if have else 'all'} mirrored")
        mirror[0 if before else 1] = short
    if any(mirror):
        seg = np.pad(seg, (tuple(mirror), (0, 0)), mode="reflect")
        i0 -= mirror[0]
    return seg, i0, notes


def rms(v):
    v = np.asarray(v, dtype=float)
    return math.sqrt(np.mean(v * v)) if len(v) else 0.0


def db(ratio):
    return 10.0 * math.log10(ratio) if ratio > 0 else float("-inf")


def alias_of(f_hz, fs):
    """Where a tone at f_hz lands after sampling at fs (the folded frequency)."""
    f_mod = math.fmod(f_hz, fs)
    return min(f_mod, fs - f_mod)


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def load(path, rate_override=None, span=(-2.0, 12.0)):
    records, stats, cfg = parse_binary_file(str(path))
    imu = records.get("ISM6HG256") or []
    ns = records.get("NonSensor") or []
    if len(imu) < 1000:
        sys.exit(f"{path}: only {len(imu)} IMU records")
    launch_idx = [i for i, r in enumerate(ns) if r.get("launch")]
    if not launch_idx:
        sys.exit(f"{path}: no launch flag in the log")
    t_imu = unwrap_u32_us(get_array(imu, "time_us"))
    # NonSensor stamps share the FC clock; unwrapped the same way.
    t_ns = unwrap_u32_us(get_array(ns, "time_us"))
    t_launch = int(t_ns[launch_idx[0]])
    rate = rate_override or derive_ekf_rate_hz(records) or 496.0

    cm = conversion_matrices(cfg)
    raw = recover_raw(imu, cm)

    # Keep one tick of lead-in so the first window is an ordinary one, not
    # the whole pad sitting in the queue (the accumulator's sums are sized
    # for a drain pass, not for twenty minutes of pad).
    lo_us = t_launch + int(span[0] * 1e6) - int(1e6 / rate)
    hi_us = t_launch + int(span[1] * 1e6)
    m = (t_imu >= lo_us) & (t_imu <= hi_us)
    t_sel, raw_sel = t_imu[m], raw[m]

    n_ticks = int((hi_us - lo_us) * rate / 1e6)
    tick_us = lo_us + (np.arange(1, n_ticks + 1) * (1e6 / rate)).astype(np.int64)
    return dict(path=path, cfg=cfg, cm=cm, t_us=t_sel, raw=raw_sel,
                tick_us=tick_us, t_launch=t_launch, rate=rate, runs=rate_runs(t_sel, t_launch),
                n_imu=len(imu))


def analyse(d, replay):
    bar = near_rail_lsb(d["cfg"]["low_g_fs_g"])
    r = replay.run(d["t_us"].astype(np.uint64), d["raw"], d["tick_us"].astype(np.uint64), bar)
    fresh = to_body(r["last"], d["cm"])
    mean = to_body(r["mean"], d["cm"])
    t_rel = (d["tick_us"] - d["t_launch"]) / 1e6
    fs = d["rate"]
    nyq = fs / 2.0
    tick_run = tick_runs(r, d["runs"])

    # The old verdict, exactly as flown: the body-frame low-g values of the
    # freshest sample against (FS - 0.5) g on any axis.
    old_bar = (d["cfg"]["low_g_fs_g"] - NEAR_RAIL_MARGIN_G) * G_MS2
    old_rail = np.max(np.abs(fresh[:, 0:3]), axis=1) > old_bar

    rows = []
    for name, lo, hi in WINDOWS:
        win = (t_rel >= lo) & (t_rel < hi) & (r["n"] > 0)
        parts, notes = split_at_rate_steps(name, win, r, tick_run, d["runs"], t_rel)
        if notes:
            rows.append(dict(window=name, notes=notes))
        for label, sel, run in parts:
            if sel.sum() < MIN_TICKS:
                continue
            odr = run["odr"]
            idx = r["last_idx"][sel].astype(np.int64)
            n = r["n"][sel].astype(np.int64)
            a, b = int(idx[0] - n[0] + 1), int(idx[-1]) + 1
            # The band-pass segment: the drained samples plus a quarter second
            # of context each side, at this run's spacing only (a segment
            # across a rate step would take samples at two spacings for one).
            body_seg, i0, margin_notes = band_segment(d, run, a, b, int(0.25 * odr), name, hi <= 0.0)
            in_win = np.zeros(len(body_seg), dtype=bool)
            in_win[a - i0:b - i0] = True

            nperseg = 1 << int(math.log2(max(16, min(256, sel.sum() // 2))))
            # The ODR-rate PSD is NPERSEG_ODR points when the window holds that
            # many samples, else shrunk to fit like the tick-rate one above, which
            # keeps the plot's reference line.  The tone is named only from the
            # full-length PSD, and only when the search band lies below the ODR
            # Nyquist: an argmax over a band with no bins lands on bin 0, a 0 Hz
            # "tone" nobody measured.
            n_odr = int(in_win.sum())
            nperseg_odr = NPERSEG_ODR if n_odr >= NPERSEG_ODR else 1 << int(math.log2(max(16, n_odr // 2)))
            if odr / 2.0 < TONE_SEARCH_HZ[0]:
                tone_na = (f"the {TONE_SEARCH_HZ[0]:.0f}-{TONE_SEARCH_HZ[1]:.0f} Hz search band is above "
                           f"the ODR Nyquist ({odr / 2.0:.0f} Hz)")
            elif nperseg_odr < NPERSEG_ODR:
                tone_na = f"{n_odr} ODR samples in the window, short of one {NPERSEG_ODR}-point segment"
            else:
                tone_na = None
            row = dict(window=label, odr=odr, span=(t_rel[sel][0], t_rel[sel][-1]) if notes else None,
                       ticks=int(sel.sum()), n_mean=float(np.mean(n)), n_max=int(np.max(n)),
                       old_rail_duty=float(np.mean(old_rail[sel])),
                       new_rail_duty=float(np.mean(r["near_rail"][sel])), tone_na=tone_na,
                       margin_notes=margin_notes, channels=[])
            for cname, col, unit in CHANNELS:
                # --- the filter itself: band-passed full-rate, decimated both ways
                x = body_seg[:, col]
                f_full, p_full = welch(x[in_win], odr, nperseg_odr)
                f_tone = None
                if tone_na is None:
                    hi_full = (f_full >= TONE_SEARCH_HZ[0]) & (f_full <= TONE_SEARCH_HZ[1])
                    f_tone = float(f_full[int(np.argmax(np.where(hi_full, p_full, -1.0)))])
                cs_bands = []
                for bname, blo, bhi in BANDS:
                    blo_, bhi_ = (nyq, odr / 2.0) if blo is None else (blo, bhi)
                    xbp = bandpass(x, odr, blo_, bhi_)
                    fresh_bp = xbp[idx - i0]
                    cs = np.concatenate([[0.0], np.cumsum(xbp)])
                    mean_bp = (cs[idx - i0 + 1] - cs[idx - i0 - n + 1]) / n
                    full = rms(xbp[in_win])
                    # Gains are referenced to the ODR-rate band RMS: the freshest
                    # pick of a line near a multiple of half the tick rate has a
                    # phase-dependent RMS, so it is not a fixed reference.
                    cs_bands.append(dict(name=bname, lo=blo_, hi=bhi_, full_rms=full,
                                         fresh_rms=rms(fresh_bp), mean_rms=rms(mean_bp),
                                         pick=(rms(fresh_bp) / full) if full > 0 else float("nan"),
                                         gain=(rms(mean_bp) / full) if full > 0 else float("nan")))
                # --- what the estimators see: the tick-rate streams
                xf, xm = fresh[sel, col], mean[sel, col]
                f, pf = welch(xf, fs, nperseg)
                _, pm = welch(xm, fs, nperseg)
                hi_band = (f >= 100.0) & (f <= nyq)
                ipk = int(np.argmax(np.where(hi_band, pf, -1.0)))
                row["channels"].append(dict(
                    name=cname, unit=unit,
                    sigma_fresh=float(np.std(xf)), sigma_mean=float(np.std(xm)),
                    hi_rms_fresh=band_rms(f, pf, 100.0, nyq), hi_rms_mean=band_rms(f, pm, 100.0, nyq),
                    peak_hz=float(f[ipk]), tone_hz=f_tone,
                    tone_alias_hz=alias_of(f_tone, fs) if f_tone is not None else None,
                    bands=cs_bands, f=f, pf=pf, pm=pm,
                    f_full=f_full[f_full <= nyq], p_full=p_full[f_full <= nyq]))
            rows.append(row)
    return rows, dict(fresh=fresh, mean=mean, t_rel=t_rel, r=r)


def report(d, rows):
    print(f"{Path(d['path']).name}: {d['n_imu']} IMU records, ODR {runs_text(d['runs'])}, "
          f"EKF ticks {d['rate']:.1f} Hz (Nyquist {d['rate']/2:.0f} Hz), "
          f"low-g bar {near_rail_lsb(d['cfg']['low_g_fs_g'])} LSB")
    for row in rows:
        if "notes" in row:
            print("\n" + "\n".join(row["notes"]))
            continue
        span = f" (T+{row['span'][0]:.3f}..{row['span'][1]:.3f} s at {row['odr']:.0f} Hz)" if row["span"] else ""
        print(f"\n[{row['window']}]  {row['ticks']} ticks{span}, samples/tick mean {row['n_mean']:.2f} "
              f"max {row['n_max']}; low-g near-rail duty: old body-frame test {100*row['old_rail_duty']:.1f} %, "
              f"new worst-sample test {100*row['new_rail_duty']:.1f} %")
        print("  as the estimators see it (tick-rate streams):")
        print(f"    {'channel':10s} {'sigma as-flown':>14s} {'sigma mean':>11s} "
              f"{'100Hz-Nyq RMS':>14s} {'after':>7s} {'net dB':>7s} {'strongest line >100 Hz':>24s}")
        for c in row["channels"]:
            net = db(c["hi_rms_mean"] ** 2 / c["hi_rms_fresh"] ** 2) if c["hi_rms_fresh"] > 0 else float("nan")
            tone = (f"ODR tone {c['tone_hz']:.0f} Hz folds to {c['tone_alias_hz']:.0f} Hz"
                    if c["tone_hz"] is not None else "ODR tone n/a")
            print(f"    {c['name']:10s} {c['sigma_fresh']:9.2f} {c['unit']:>5s} {c['sigma_mean']:11.2f} "
                  f"{c['hi_rms_fresh']:14.2f} {c['hi_rms_mean']:7.2f} {net:7.1f} "
                  f"{c['peak_hz']:8.0f} Hz ({tone})")
        if row["tone_na"]:
            print(f"    ODR tone n/a: {row['tone_na']}")
        print("  band-passed through the same windows: band RMS at the ODR, then what each")
        print("  decimation keeps of it (as-flown pick / window mean, relative to the ODR-rate RMS):")
        for note in row["margin_notes"]:
            print(f"    {note}")
        print(f"    {'channel':10s} " + " ".join(f"{b['name']:>40s}" for b in row["channels"][0]["bands"]))
        for c in row["channels"]:
            cells = []
            for b in c["bands"]:
                if b["lo"] >= row["odr"] / 2.0:
                    # Nothing of the band exists at this ODR, so there is no
                    # RMS to take a gain against (it is not a -inf dB cut).
                    cells.append(f"n/a: above the {row['odr'] / 2.0:.0f} Hz ODR Nyquist")
                else:
                    cells.append(f"{b['full_rms']:8.2f}  pick x{b['pick']:.3f}  mean x{b['gain']:.3f} "
                                 f"({20*math.log10(b['gain']) if b['gain'] > 0 else float('-inf'):6.1f} dB)")
            print(f"    {c['name']:10s} " + " ".join(f"{cell:>40s}" for cell in cells))


def plot(d, rows, plot_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plot_dir = Path(plot_dir)
    plot_dir.mkdir(parents=True, exist_ok=True)
    for row in rows:
        if "notes" in row:
            continue
        fig, axes = plt.subplots(1, len(row["channels"]), figsize=(5.2 * len(row["channels"]), 4.2))
        for ax, c in zip(np.atleast_1d(axes), row["channels"]):
            ax.semilogy(c["f_full"], c["p_full"], color="0.6", lw=0.8, label="true content (ODR-rate PSD)")
            ax.semilogy(c["f"], c["pf"], lw=1.0, label="freshest sample (as flown)")
            ax.semilogy(c["f"], c["pm"], lw=1.0, label="window mean (#1191)")
            ax.set_title(f"{row['window']}: {c['name']}")
            ax.set_xlabel("Hz at the EKF tick rate")
            ax.set_ylabel(f"PSD ({c['unit']})^2/Hz")
            ax.grid(True, which="both", alpha=0.3)
            ax.legend(fontsize=8)
        span = f", T+{row['span'][0]:.3f}..{row['span'][1]:.3f} s at {row['odr']:.0f} Hz" if row["span"] else ""
        fig.suptitle(f"{Path(d['path']).name} — what the EKF is fed, {row['window']} window{span}")
        fig.tight_layout()
        out = plot_dir / f"imu_drain_{row['window']}.png"
        fig.savefig(out, dpi=110)
        plt.close(fig)
        print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("binary_file")
    ap.add_argument("--rate-hz", type=float, default=None,
                    help="EKF tick rate to emulate (default: derived from the logged ekf_ticks)")
    ap.add_argument("--plot-dir", default=None, help="write PSD overlays here (needs matplotlib)")
    args = ap.parse_args()

    d = load(Path(args.binary_file), args.rate_hz)
    rows, _ = analyse(d, DrainReplay())
    report(d, rows)
    if args.plot_dir:
        plot(d, rows, args.plot_dir)


if __name__ == "__main__":
    main()
