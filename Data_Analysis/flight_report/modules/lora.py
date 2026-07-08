"""LoRa link / telemetry module.

Looks for sibling `lora_*.csv` files in the flight directory (the ground-station
packet log) and reports link quality (RSSI, SNR), packet rate / loss / gaps,
frequency hopping, and the telemetry that actually made it over LoRa
(altitude, speed, attitude, power, ground track).

Skips gracefully when no `lora_*.csv` is present.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from ..flight import Flight
from ..registry import AnalysisResult


_M_PER_DEG_LAT = 111_132.0


def _discover_lora_csvs(flight: Flight) -> list[Path]:
    """Find lora_*.csv siblings next to the .bin (excluding flight_*.csv)."""
    return sorted(p for p in flight.bin_path.parent.glob("lora_*.csv") if p.is_file())


def _load(lora_csvs: list[Path]) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Concat all lora CSVs; split telemetry rows from EVENT rows."""
    frames = []
    for p in lora_csvs:
        try:
            frames.append(pd.read_csv(p, dtype={"event": str}).fillna({"event": ""}))
        except Exception:  # noqa: BLE001
            continue
    if not frames:
        return pd.DataFrame(), pd.DataFrame()
    df = pd.concat(frames, ignore_index=True)
    events = df[df.state == "EVENT"].copy()
    telem = df[df.state != "EVENT"].reset_index(drop=True)
    # Drop obvious garbage rows (no time)
    telem = telem[telem.time_ms.notna()].reset_index(drop=True)
    return telem, events


def _pad_position(df: pd.DataFrame) -> tuple[float, float, float] | None:
    """Median GPS during PRELAUNCH/READY/INIT (or first 50 packets)."""
    pre = df[df.state.isin(["PRELAUNCH", "READY", "INIT"])]
    if len(pre) < 5:
        pre = df.head(min(50, len(df)))
    pre = pre[pre.num_sats > 0]
    pre = pre[pre.lat.notna() & pre.lon.notna() & (pre.lat != 0)]
    if len(pre) < 5:
        return None
    return float(pre.lat.median()), float(pre.lon.median()), float(pre.alt_m.median())


def _slant_distance_m(df: pd.DataFrame, pad: tuple[float, float, float]) -> np.ndarray:
    """Pad-relative slant distance (m) per packet, NaN where GPS invalid."""
    plat, plon, palt = pad
    lat = df.lat.to_numpy()
    lon = df.lon.to_numpy()
    alt = df.alt_m.to_numpy()
    valid = np.isfinite(lat) & np.isfinite(lon) & (lat != 0) & (df.num_sats.to_numpy() > 0)
    m_per_lon = 111_320.0 * math.cos(math.radians(plat))
    dx = (lon - plon) * m_per_lon
    dy = (lat - plat) * _M_PER_DEG_LAT
    dz = alt - palt
    d = np.sqrt(dx * dx + dy * dy + dz * dz)
    d[~valid] = np.nan
    return d


# ── Figures ──────────────────────────────────────────────────────────────────

def _fig_rssi_snr(df: pd.DataFrame, t: np.ndarray):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 5.5), sharex=True)
    ax1.plot(t, df.rssi, color="tab:blue", lw=0.6, alpha=0.7)
    ax1.set_ylabel("RSSI (dBm)")
    ax1.set_title("LoRa link quality")
    ax1.grid(True, alpha=0.3)
    ax1.axhline(-110, color="orange", ls="--", lw=0.8, alpha=0.5, label="-110 dBm")
    ax1.axhline(-123, color="red",    ls="--", lw=0.8, alpha=0.5, label="sensitivity (-123 dBm)")
    ax1.legend(loc="lower right", fontsize=8)
    ax2.plot(t, df.snr, color="tab:green", lw=0.6, alpha=0.7)
    ax2.set_ylabel("SNR (dB)")
    ax2.set_xlabel("Time since first packet (s)")
    ax2.axhline(0, color="k", lw=0.4, alpha=0.4)
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def _fig_rssi_hist(df: pd.DataFrame):
    fig, ax = plt.subplots(figsize=(10, 3.5))
    rssi = df.rssi.to_numpy()
    rssi = rssi[np.isfinite(rssi)]
    if rssi.size == 0:
        plt.close(fig)
        return None
    ax.hist(rssi, bins=50, color="tab:blue", alpha=0.7)
    ax.set_xlabel("RSSI (dBm)")
    ax.set_ylabel("Packets")
    ax.set_title("RSSI distribution")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    return fig


def _fig_rssi_vs_distance(df: pd.DataFrame, distance_m: np.ndarray):
    rssi = df.rssi.to_numpy()
    ok = np.isfinite(rssi) & np.isfinite(distance_m) & (distance_m > 0)
    if ok.sum() < 10:
        return None
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.scatter(distance_m[ok], rssi[ok], s=4, alpha=0.4, color="tab:blue")
    ax.set_xscale("log")
    ax.set_xlabel("Slant distance to pad (m, log scale)")
    ax.set_ylabel("RSSI (dBm)")
    ax.set_title("RSSI vs slant distance")
    ax.axhline(-123, color="red", ls="--", lw=0.8, alpha=0.5, label="-123 dBm sensitivity")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    return fig


def _fig_packet_rate(df: pd.DataFrame, t: np.ndarray):
    if t.size < 30:
        return None
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 3.8), gridspec_kw={"width_ratios": [3, 1]})
    # 1 s rolling rate
    bin_s = 1.0
    bins = np.arange(t[0], t[-1] + bin_s, bin_s)
    counts, edges = np.histogram(t, bins=bins)
    centers = 0.5 * (edges[:-1] + edges[1:])
    ax1.plot(centers, counts / bin_s, color="tab:purple", lw=1.0)
    ax1.set_xlabel("Time since first packet (s)")
    ax1.set_ylabel("Packets / s")
    ax1.set_title("LoRa packet rate (1 s window)")
    ax1.grid(True, alpha=0.3)
    # Inter-packet gap histogram (skipped-sequence count from `gap` column)
    gaps = df["gap"].to_numpy()
    gaps = gaps[gaps >= 0]
    if gaps.size:
        ax2.hist(gaps, bins=range(0, int(min(gaps.max(), 30)) + 2), color="tab:red", alpha=0.7)
        ax2.set_xlabel("Missed packets between")
        ax2.set_ylabel("Count")
        ax2.set_title("Sequence-gap distribution")
        ax2.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    return fig


def _fig_telem_altitude(df: pd.DataFrame, t: np.ndarray):
    fig, ax = plt.subplots(figsize=(11, 4))
    have = False
    if "alt_m" in df:
        gps_alt = df.alt_m.to_numpy()
        ok = np.isfinite(gps_alt) & (df.num_sats.to_numpy() > 0)
        if ok.any():
            ax.plot(t[ok], gps_alt[ok], color="tab:blue", lw=0.9, label="GPS alt")
            have = True
    if "pressure_alt" in df:
        palt = df.pressure_alt.to_numpy()
        ok = np.isfinite(palt)
        if ok.any():
            ax.plot(t[ok], palt[ok], color="tab:green", lw=0.9, label="Pressure alt")
            have = True
    if not have:
        plt.close(fig)
        return None
    ax.set_xlabel("Time since first packet (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Telemetered altitude (over LoRa)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    return fig


def _fig_telem_speed_power(df: pd.DataFrame, t: np.ndarray):
    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(t, df.get("speed", pd.Series(dtype=float)), color="tab:orange", lw=0.9, label="speed")
    if "max_speed" in df:
        axes[0].plot(t, df.max_speed, color="tab:red", lw=0.7, alpha=0.6, label="max_speed")
    axes[0].set_ylabel("Speed (m/s)")
    axes[0].set_title("Telemetered speed / power")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    if "voltage" in df:
        axes[1].plot(t, df.voltage, color="tab:blue", lw=0.9)
    axes[1].set_ylabel("Voltage (V)")
    axes[1].grid(True, alpha=0.3)

    if "soc" in df:
        axes[2].plot(t, df.soc, color="tab:green", lw=0.9)
    axes[2].set_ylabel("SoC (%)")
    axes[2].set_xlabel("Time since first packet (s)")
    axes[2].grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def _fig_telem_attitude(df: pd.DataFrame, t: np.ndarray):
    if not all(c in df for c in ("roll", "pitch", "yaw")):
        return None
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, df.roll,  color="tab:blue",   lw=0.7, label="roll")
    ax.plot(t, df.pitch, color="tab:green",  lw=0.7, label="pitch")
    ax.plot(t, df.yaw,   color="tab:orange", lw=0.7, label="yaw")
    ax.set_xlabel("Time since first packet (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Telemetered attitude")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, ncol=3)
    fig.tight_layout()
    return fig


def _fig_ground_track(df: pd.DataFrame):
    if "lat" not in df or "lon" not in df:
        return None
    lat = df.lat.to_numpy()
    lon = df.lon.to_numpy()
    ok = np.isfinite(lat) & np.isfinite(lon) & (lat != 0) & (df.num_sats.to_numpy() > 0)
    if ok.sum() < 5:
        return None
    fig, ax = plt.subplots(figsize=(7, 7))
    rssi = df.rssi.to_numpy()[ok]
    sc = ax.scatter(lon[ok], lat[ok], c=rssi, s=8, cmap="viridis_r", alpha=0.8)
    cb = fig.colorbar(sc, ax=ax)
    cb.set_label("RSSI (dBm)")
    ax.set_xlabel("Longitude (deg)")
    ax.set_ylabel("Latitude (deg)")
    ax.set_title("LoRa ground track (color = RSSI)")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    return fig


def _fig_freq_hopping(df: pd.DataFrame, t: np.ndarray):
    if "rx_freq_mhz" not in df:
        return None
    f = df.rx_freq_mhz.to_numpy()
    if not np.isfinite(f).any() or (np.unique(f[np.isfinite(f)]).size < 2):
        return None  # not hopping
    fig, ax = plt.subplots(figsize=(11, 3.5))
    ax.scatter(t, f, s=3, alpha=0.5, color="tab:cyan")
    ax.set_xlabel("Time since first packet (s)")
    ax.set_ylabel("RX freq (MHz)")
    ax.set_title("Frequency hopping timeline")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


# ── Module entry point ─────────────────────────────────────────────────────────

def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="lora", title="LoRa Link & Telemetry")

    lora_csvs = _discover_lora_csvs(flight)
    if not lora_csvs:
        result.warnings.append("No lora_*.csv found in flight directory.")
        return result

    df, events = _load(lora_csvs)
    if df.empty:
        result.warnings.append("LoRa CSV(s) found but contain no telemetry rows.")
        return result

    # Multi-rocket demux (#381): post-#381 BS CSVs carry a rocket_id column
    # because one file can interleave several rockets' telemetry (e.g. a
    # landed rocket still transmitting while the next one preps). Analyze the
    # dominant rocket and say so, rather than silently blending trajectories.
    # Older CSVs without the column (or single-rocket files) are unaffected.
    if "rocket_id" in df.columns:
        rid = pd.to_numeric(df.rocket_id, errors="coerce")
        ids = rid.dropna().unique()
        if len(ids) > 1:
            dominant = int(rid.value_counts().idxmax())
            n_before = len(df)
            df = df[rid == dominant].reset_index(drop=True)
            others = sorted(int(i) for i in ids if int(i) != dominant)
            result.warnings.append(
                f"LoRa CSV contains {len(ids)} rockets; analyzing rocket_id="
                f"{dominant} ({len(df)}/{n_before} rows) — other rocket(s) "
                f"{others} excluded."
            )
        if df.empty:
            result.warnings.append("No telemetry rows left after rocket_id demux.")
            return result

    t_ms = df.time_ms.to_numpy()
    t = (t_ms - t_ms[0]) / 1000.0

    # ── Link-quality metrics ─────────────────────────────────────────────────
    rssi = df.rssi.to_numpy()
    rssi = rssi[np.isfinite(rssi)]
    snr = df.snr.to_numpy()
    snr = snr[np.isfinite(snr)]

    # ── Packet-rate / loss ───────────────────────────────────────────────────
    duration_s = float((t_ms[-1] - t_ms[0]) / 1000.0)
    rate_hz = round(len(df) / duration_s, 2) if duration_s > 0 else 0
    # Sequence-based loss: sum of `gap` column (consecutive missing packets per row)
    gaps = df["gap"].to_numpy()
    gaps = gaps[gaps >= 0]
    seq_skipped = int(gaps.sum()) if gaps.size else 0
    seq_span = int(df["seq"].iloc[-1] - df["seq"].iloc[0]) if len(df) >= 2 else 0
    seq_received = len(df)
    loss_pct = round(100.0 * seq_skipped / max(seq_span, 1), 2) if seq_span > 0 else 0.0

    # ── Pad-relative slant distance ──────────────────────────────────────────
    pad = _pad_position(df)
    max_range_m = None
    distance_m = None
    if pad is not None:
        distance_m = _slant_distance_m(df, pad)
        if np.isfinite(distance_m).any():
            max_range_m = float(np.nanmax(distance_m))

    # ── Metrics ──────────────────────────────────────────────────────────────
    metrics: dict[str, object] = {
        "lora_csv_files":           ", ".join(p.name for p in lora_csvs),
        "lora_packets_received":    seq_received,
        "lora_duration_s":          round(duration_s, 1),
        "lora_packet_rate_hz":      rate_hz,
        "lora_seq_span":            seq_span if seq_span > 0 else "—",
        "lora_packets_skipped":     seq_skipped,
        "lora_packet_loss_pct":     loss_pct,
        "lora_rssi_min/med/max":    (
            f"{int(rssi.min())}/{int(np.median(rssi))}/{int(rssi.max())}"
            if rssi.size else "—"
        ),
        "lora_snr_min/med/max":     (
            f"{snr.min():.1f}/{float(np.median(snr)):.1f}/{snr.max():.1f}"
            if snr.size else "—"
        ),
        "lora_max_range_to_pad_m":  round(max_range_m, 1) if max_range_m is not None else "—",
        "lora_events_logged":       len(events),
    }
    result.metrics = metrics

    # ── Warnings ─────────────────────────────────────────────────────────────
    if loss_pct > 10:
        result.warnings.append(
            f"Packet loss {loss_pct:.1f}% ({seq_skipped} missed of {seq_span} expected)."
        )
    if rssi.size and rssi.min() < -120:
        result.warnings.append(
            f"Min RSSI {int(rssi.min())} dBm — within 3 dB of sensitivity floor."
        )
    if len(events):
        ev_summary = ", ".join(events["event"].dropna().astype(str).head(5).tolist())
        result.warnings.append(f"{len(events)} EVENT row(s) in LoRa log: {ev_summary}")

    # ── Figures ──────────────────────────────────────────────────────────────
    for fn in (
        _fig_rssi_snr,
        _fig_rssi_hist,
        _fig_packet_rate,
        _fig_telem_altitude,
        _fig_telem_speed_power,
        _fig_telem_attitude,
        _fig_ground_track,
        _fig_freq_hopping,
    ):
        try:
            if fn is _fig_rssi_snr:
                fig = fn(df, t)
            elif fn is _fig_rssi_hist:
                fig = fn(df)
            elif fn is _fig_packet_rate:
                fig = fn(df, t)
            elif fn is _fig_ground_track:
                fig = fn(df)
            elif fn is _fig_freq_hopping:
                fig = fn(df, t)
            else:
                fig = fn(df, t)
        except Exception as e:  # noqa: BLE001
            result.warnings.append(f"{fn.__name__}: {type(e).__name__}: {e}")
            continue
        if fig is not None:
            result.figures.append(fig)

    if distance_m is not None:
        fig = _fig_rssi_vs_distance(df, distance_m)
        if fig is not None:
            result.figures.append(fig)

    return result
