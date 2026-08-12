"""What came down the radio, and how well the link held up.

Three questions a flyer asks about telemetry, in order:

1. Did the numbers arrive? The altitude the ground station saw, against the
   altitude the flight computer recorded. They are the same measurement taking
   two different routes, so where they diverge, packets were lost.
2. Why did the link fade? Received strength against how far away the rocket was,
   with the low passes marked.

The second is the point of the section. There is no ground track here: the 3D
view at the top of the report already draws where the rocket went, from the same
fixes and against imagery, and a flat repeat of it added nothing. Free-space loss alone predicts a
smooth curve against range, and the measured points do not lie on one: the ones
that fall short are the ones taken at a low look-angle, where the path grazes the
ground and the direct ray arrives with a reflection on top of it. So the chart
colours by elevation angle rather than by time, and the answer to "why was my
link bad at 400 m" turns out to be "because the rocket was low, not because it
was far".

The ground station does not record its own position. The pad is used instead —
the receiver is normally within a few metres of it, which is nothing against the
ranges here. Ranges are honest to about that few metres; the elevation angle is
only meaningful once the rocket is well clear of the pad, which is why the
geometry is computed over the flight and not over the whole session (most packets
are the rocket sitting a couple of metres away, where a metre of GNSS noise
swings the angle through tens of degrees).
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

from ..charts import COLORS, chart, trace
from ..flight import Flight
from ..units import q
from ..registry import AnalysisResult

# Below this the path grazes terrain and the ground reflection starts to matter.
# A round number, not a derived one: the transition is gradual.
_LOW_ELEVATION_DEG = 10.0

_M_PER_DEG_LAT = 111_132.0
_M_PER_DEG_LON_EQ = 111_320.0

# States the flight computer reports before it is off the pad. Used to pick the
# packets that establish where the pad is.
_ON_PAD = ("INIT", "READY", "PRELAUNCH")
_IN_FLIGHT = "INFLIGHT"

_LOW_COLOR = "#d62728"
_HIGH_COLOR = "#1f77b4"


def _read(flight: Flight):
    """The ground-station packet log as a DataFrame, or None if there isn't one."""
    try:
        import pandas as pd
    except ImportError:                                    # pragma: no cover
        return None
    paths = sorted(p for p in flight.bin_path.parent.glob("lora_*.csv") if p.is_file())
    if not paths:
        return None
    frames = []
    for p in paths:
        try:
            frames.append(pd.read_csv(p))
        except Exception:
            continue
    if not frames:
        return None
    df = pd.concat(frames, ignore_index=True) if len(frames) > 1 else frames[0]
    return df if len(df) else None


def _fixes(df):
    """Rows carrying a usable position. Null island is a receiver with no fix."""
    if not {"lat", "lon", "alt_m"} <= set(df.columns):
        return None
    d = df.dropna(subset=["lat", "lon", "alt_m"])
    d = d[(d["lat"].abs() > 0.001) & (d["lon"].abs() > 0.001)]
    return d if len(d) else None


def _pad(d):
    """Ground-station position, taken as the pad. See the module docstring."""
    on_pad = d[d["state"].isin(_ON_PAD)] if "state" in d.columns else d.iloc[:0]
    src = on_pad if len(on_pad) >= 5 else d
    return float(src["lat"].median()), float(src["lon"].median()), float(src["alt_m"].median())


def _enu(d, lat0, lon0, alt0):
    m_lon = _M_PER_DEG_LON_EQ * math.cos(math.radians(lat0))
    east = (d["lon"].to_numpy() - lon0) * m_lon
    north = (d["lat"].to_numpy() - lat0) * _M_PER_DEG_LAT
    up = d["alt_m"].to_numpy() - alt0
    return east, north, up


def _altitude_chart(df) -> Optional[dict[str, Any]]:
    """Altitude as the ground station received it."""
    if "time_ms" not in df.columns:
        return None
    t = df["time_ms"].to_numpy() / 1000.0
    traces = []
    if "pressure_alt" in df.columns:
        traces.append(trace(t, df["pressure_alt"].to_numpy(),
                            "Telemetered Baro Altitude", COLORS[0]))
    if "alt_m" in df.columns:
        alt = df["alt_m"].to_numpy(dtype=float)
        base = np.nanmin(alt) if np.isfinite(alt).any() else 0.0
        traces.append(trace(t, alt - base, "Telemetered GNSS Altitude", COLORS[1]))
    spec = chart("chart-lora-alt", "Altitude received over the radio", traces,
                 x_title="Ground station time (s)", y_title="Altitude", y_unit="m",
                 height=330, clip_spikes=True)
    if not spec:
        return None
    spec["note"] = (
        "The flight as the ground station saw it, on the receiver's own clock. "
        "Each point is one packet that arrived; a flat stretch across the boost "
        "is packets that did not. Compare the peak against the apogee on the "
        "summary card — telemetry is downsampled and lossy, so it reads low."
    )
    return spec


def _rssi_chart(d, lat0, lon0, alt0) -> tuple[Optional[dict[str, Any]], dict[str, Any]]:
    """Received strength against slant range, low look-angles picked out."""
    facts: dict[str, Any] = {}
    if "rssi" not in d.columns:
        return None, facts

    # Flight only. On the pad the rocket is a couple of metres from the receiver,
    # where a metre of GNSS noise swings the elevation angle through tens of
    # degrees and the geometry means nothing.
    fl = d[d["state"] == _IN_FLIGHT] if "state" in d.columns else d
    if len(fl) < 5:
        return None, facts

    east, north, up = _enu(fl, lat0, lon0, alt0)
    horizontal = np.hypot(east, north)
    slant = np.hypot(horizontal, up)
    elevation = np.degrees(np.arctan2(up, np.maximum(horizontal, 1e-6)))
    rssi = fl["rssi"].to_numpy(dtype=float)

    low = elevation < _LOW_ELEVATION_DEG
    facts = {
        "n": int(len(fl)), "low": int(low.sum()),
        "max_range": float(np.max(slant)),
        "rssi_low": float(np.median(rssi[low])) if low.any() else None,
        "rssi_high": float(np.median(rssi[~low])) if (~low).any() else None,
    }

    traces = []
    for mask, name, color in ((~low, f"Above {_LOW_ELEVATION_DEG:.0f}° elevation", _HIGH_COLOR),
                              (low, f"Below {_LOW_ELEVATION_DEG:.0f}° elevation", _LOW_COLOR)):
        if not mask.any():
            continue
        t = trace(slant[mask], rssi[mask], name, color, mode="markers", size=7)
        if t:
            t["hovertemplate"] = f"{name}<br>%{{x:.0f}} m · %{{y:.0f}} dBm<extra></extra>"
            traces.append(t)

    spec = chart("chart-lora-rssi", "Signal strength against distance", traces,
                 x_title="Slant range from the pad", x_unit="m",
                 y_title="Received strength", y_unit="dBm", height=380)
    if not spec:
        return None, facts
    spec["layout"]["hovermode"] = "closest"

    note = (
        f"Every packet of the flight, placed by how far away the rocket was rather "
        f"than by when it arrived. Red marks the passes taken below "
        f"{_LOW_ELEVATION_DEG:.0f}° above the horizon, where the path grazes the "
        f"ground and the direct ray arrives with a reflection on top of it."
    )
    if facts["rssi_low"] is not None and facts["rssi_high"] is not None:
        note += (
            f" Here the low passes ran {facts['rssi_high'] - facts['rssi_low']:.0f} dB "
            f"weaker than the rest ({facts['rssi_low']:.0f} against "
            f"{facts['rssi_high']:.0f} dBm), at no greater range — which is the "
            f"case for getting the antenna up rather than for more power."
        )
    spec["note"] = note
    return spec, facts


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="lora_link", title="Radio Link")

    df = _read(flight)
    if df is None:
        # Not a warning. Most flights are flown without a ground station, and a
        # missing telemetry log is not a fault in the flight.
        return result

    result.charts.append(_altitude_chart(df))
    d = _fixes(df)
    if d is not None:
        lat0, lon0, alt0 = _pad(d)
        rssi_spec, facts = _rssi_chart(d, lat0, lon0, alt0)
        result.charts.append(rssi_spec)
        if facts.get("n"):
            result.metrics["Packets in flight"] = f"{facts['n']:,}"
            result.metrics["Longest range"] = q(facts["max_range"], "m", 0)
            if facts["low"]:
                result.metrics["Below 10° elevation"] = (
                    f"{facts['low']} packets ({100.0 * facts['low'] / facts['n']:.0f}%)"
                )
    elif "rssi" in df.columns:
        result.warnings.append(
            "Telemetry arrived but never carried a position, so range and look-angle "
            "cannot be worked out — only the altitude chart above is available."
        )

    result.charts = [c for c in result.charts if c]
    if not result.charts:
        result.warnings.append("The ground-station log carried no plottable telemetry.")
    return result
