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

Plus the mean samples per tick (the N behind the boxcar) and the low-g
near-rail duty: the old body-frame test on the freshest sample against the
new per-sensor-axis test on the window's worst sample.
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
    """Brick-wall band-pass by FFT masking; the segment carries margins so the
    edge ringing stays outside the ticks that are measured."""
    x = np.asarray(x, dtype=float)
    spec = np.fft.rfft(x - x.mean())
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    spec[(f < lo) | (f > hi)] = 0.0
    return np.fft.irfft(spec, n=len(x))


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
    odr = 1e6 / np.median(np.diff(t_sel))
    return dict(path=path, cfg=cfg, cm=cm, t_us=t_sel, raw=raw_sel,
                tick_us=tick_us, t_launch=t_launch, rate=rate, odr=odr, n_imu=len(imu))


def analyse(d, replay):
    bar = near_rail_lsb(d["cfg"]["low_g_fs_g"])
    r = replay.run(d["t_us"].astype(np.uint64), d["raw"], d["tick_us"].astype(np.uint64), bar)
    fresh = to_body(r["last"], d["cm"])
    mean = to_body(r["mean"], d["cm"])
    t_rel = (d["tick_us"] - d["t_launch"]) / 1e6
    fs, odr = d["rate"], d["odr"]
    nyq = fs / 2.0
    margin = int(0.25 * odr)

    # The old verdict, exactly as flown: the body-frame low-g values of the
    # freshest sample against (FS - 0.5) g on any axis.
    old_bar = (d["cfg"]["low_g_fs_g"] - NEAR_RAIL_MARGIN_G) * G_MS2
    old_rail = np.max(np.abs(fresh[:, 0:3]), axis=1) > old_bar

    rows = []
    for name, lo, hi in WINDOWS:
        sel = (t_rel >= lo) & (t_rel < hi) & (r["n"] > 0)
        if sel.sum() < 64:
            continue
        idx = r["last_idx"][sel].astype(np.int64)
        n = r["n"][sel].astype(np.int64)
        i0 = max(0, int(idx[0] - n[0] + 1) - margin)
        i1 = min(len(d["raw"]), int(idx[-1]) + 1 + margin)
        body_seg = to_body(d["raw"][i0:i1], d["cm"])
        in_win = np.zeros(i1 - i0, dtype=bool)
        in_win[int(idx[0] - n[0] + 1) - i0:int(idx[-1]) + 1 - i0] = True

        nperseg = 1 << int(math.log2(max(16, min(256, sel.sum() // 2))))
        row = dict(window=name, ticks=int(sel.sum()), n_mean=float(np.mean(n)), n_max=int(np.max(n)),
                   old_rail_duty=float(np.mean(old_rail[sel])),
                   new_rail_duty=float(np.mean(r["near_rail"][sel])), channels=[])
        for cname, col, unit in CHANNELS:
            # --- the filter itself: band-passed full-rate, decimated both ways
            x = body_seg[:, col]
            f_full, p_full = welch(x[in_win], odr, 1024)
            hi_full = (f_full >= 500.0) & (f_full <= 1000.0)
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
                peak_hz=float(f[ipk]), tone_hz=f_tone, tone_alias_hz=alias_of(f_tone, fs),
                bands=cs_bands, f=f, pf=pf, pm=pm,
                f_full=f_full[f_full <= nyq], p_full=p_full[f_full <= nyq]))
        rows.append(row)
    return rows, dict(fresh=fresh, mean=mean, t_rel=t_rel, r=r)


def report(d, rows):
    print(f"{Path(d['path']).name}: {d['n_imu']} IMU records, ODR {d['odr']:.0f} Hz, "
          f"EKF ticks {d['rate']:.1f} Hz (Nyquist {d['rate']/2:.0f} Hz), "
          f"low-g bar {near_rail_lsb(d['cfg']['low_g_fs_g'])} LSB")
    for row in rows:
        print(f"\n[{row['window']}]  {row['ticks']} ticks, samples/tick mean {row['n_mean']:.2f} "
              f"max {row['n_max']}; low-g near-rail duty: old body-frame test {100*row['old_rail_duty']:.1f} %, "
              f"new worst-sample test {100*row['new_rail_duty']:.1f} %")
        print("  as the estimators see it (tick-rate streams):")
        print(f"    {'channel':10s} {'sigma as-flown':>14s} {'sigma mean':>11s} "
              f"{'100Hz-Nyq RMS':>14s} {'after':>7s} {'net dB':>7s} {'strongest line >100 Hz':>24s}")
        for c in row["channels"]:
            net = db(c["hi_rms_mean"] ** 2 / c["hi_rms_fresh"] ** 2) if c["hi_rms_fresh"] > 0 else float("nan")
            print(f"    {c['name']:10s} {c['sigma_fresh']:9.2f} {c['unit']:>5s} {c['sigma_mean']:11.2f} "
                  f"{c['hi_rms_fresh']:14.2f} {c['hi_rms_mean']:7.2f} {net:7.1f} "
                  f"{c['peak_hz']:8.0f} Hz (ODR tone {c['tone_hz']:.0f} Hz folds to {c['tone_alias_hz']:.0f} Hz)")
        print("  band-passed through the same windows: band RMS at the ODR, then what each")
        print("  decimation keeps of it (as-flown pick / window mean, relative to the ODR-rate RMS):")
        print(f"    {'channel':10s} " + " ".join(f"{b['name']:>40s}" for b in row["channels"][0]["bands"]))
        for c in row["channels"]:
            cells = []
            for b in c["bands"]:
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
        fig.suptitle(f"{Path(d['path']).name} — what the EKF is fed, {row['window']} window")
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
