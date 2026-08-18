"""What the barometer actually measured, before it became an altitude.

Every altitude in this report is derived from this one pressure trace, so it is
worth being able to look at the raw signal: a sensor that saturated, drifted or
took a hit from the ejection charge shows up here as a shape, and nowhere else as
anything but a number that looks slightly wrong.

The temperature is the sensor die, not the air. It sits 15-20 °C above ambient
because the board heats itself, and it falls through the flight as airflow cools
it. That is normal and the chart says so — a flyer who reads 45 °C as an outside
air temperature would reasonably conclude the sensor is broken.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

from ..charts import COLORS, chart, trace
from ..events import markers
from ..flight import Flight
from ..registry import AnalysisResult

# hPa is what altimeters, weather stations and field boxes all quote, and it puts
# the numbers in a range a flyer recognises. The stored value is Pa.
_PA_PER_HPA = 100.0


def _series(recs, t0) -> tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    baro = recs.get("BMP585") or []
    if not baro or t0 is None:
        return None, None, None
    t = (get_array(baro, "time_us") - t0) / 1e6
    p = get_array(baro, "pressure_pa")
    temp = get_array(baro, "temperature") if "temperature" in baro[0] else None
    if not t.size or not p.size:
        return None, None, None
    return t, p, temp


def _smooth_hpa(v, k: int = 25):
    """Median filter, so the apogee reading is the flight and not the charge."""
    if v.size <= k:
        return v
    pad = k // 2
    padded = np.concatenate([np.full(pad, v[0]), v, np.full(pad, v[-1])])
    return np.array([np.median(padded[i:i + k]) for i in range(v.size)])


def _pressure_chart(t, p, events) -> Optional[dict[str, Any]]:
    hpa = p / _PA_PER_HPA
    spec = chart("chart-pressure", "Barometric pressure", [
        trace(t, hpa, "Pressure", COLORS[0]),
    ], y_title="Pressure", y_unit="hPa", events=events, height=320, clip_spikes=True)
    if not spec:
        return None
    # Pad to apogee, not min-to-max: the ejection charge is the deepest point on
    # this trace and it is a pressure event, not a height. Quoting it here would
    # measure the very spike the next sentence tells the reader to discount.
    pad = float(np.median(hpa[: min(200, hpa.size)]))
    apogee_p = float(np.min(_smooth_hpa(hpa)))
    spec["note"] = (
        f"The raw signal every altitude in this report is derived from. It falls "
        f"{pad - apogee_p:.1f} hPa from the pad to the top of the flight. The ejection "
        f"charge drives it hard for a few samples — that spike is the charge "
        f"venting, not the vehicle changing height."
    )
    return spec


def _temperature_chart(t, temp, events) -> Optional[dict[str, Any]]:
    if temp is None or not temp.size:
        return None
    spec = chart("chart-baro-temp", "Barometer die temperature", [
        trace(t, temp, "Die Temperature", COLORS[1]),
    ], y_title="Temperature", y_unit="°C", events=events, height=300)
    if not spec:
        return None
    spec["note"] = (
        "The sensor's own die, not the outside air — the board warms it well above "
        "ambient, and it cools through the flight as air moves over the airframe. "
        "It is here because pressure sensors drift with temperature, so a reading "
        "that swings hard is a reason to distrust the altitude at that moment."
    )
    return spec


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="barometer", title="Barometer")
    t, p, temp = _series(flight.records, flight.t0_us)
    if t is None:
        result.warnings.append("No barometer records, so there is no pressure to show.")
        return result

    events = {k: round(v, 3) for k, v in markers(flight).items() if v is not None}
    result.charts = [c for c in (_pressure_chart(t, p, events),
                                 _temperature_chart(t, temp, events)) if c]
    if not result.charts:
        result.warnings.append("Barometer records carry no usable pressure series.")
    return result
