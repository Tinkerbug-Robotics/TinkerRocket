"""Pyro / apogee timing module.

Reports when each apogee detector fired vs when each pyro channel fired.
Uses NonSensor records directly (the canonical parser populates the flags).
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array, pressure_to_altitude  # noqa: E402

from ..flight import Flight
from ..registry import AnalysisResult


# NonSensor flag bits
NSF_ALT_LANDED  = 1 << 0
NSF_ALT_APOGEE  = 1 << 1
NSF_VEL_APOGEE  = 1 << 2
NSF_LAUNCH      = 1 << 3
NSF_BURNOUT     = 1 << 4
NSF_GPS_APOGEE  = 1 << 5
NSF_PYRO1_ARMED = 1 << 6
NSF_PYRO2_ARMED = 1 << 7

# pyro_status byte
PSF_CH1_CONT  = 1 << 0
PSF_CH2_CONT  = 1 << 1
PSF_CH1_FIRED = 1 << 2
PSF_CH2_FIRED = 1 << 3


def _first_time_us(ns_recs, predicate):
    for r in ns_recs:
        if predicate(r):
            return r["time_us"]
    return None


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="pyro_apogee", title="Pyro & Apogee Timing")
    recs = flight.records
    t0 = flight.t0_us or 0

    ns = recs.get("NonSensor") or []
    baro = recs.get("BMP585") or []

    if not ns:
        result.warnings.append("No NonSensor records.")
        return result

    # Parser exposes most NonSensor flags as direct booleans; only burnout
    # still needs the raw flags byte.
    launch_us  = _first_time_us(ns, lambda r: r.get("launch"))
    apogee_us  = _first_time_us(ns, lambda r: r.get("alt_apogee"))
    velapg_us  = _first_time_us(ns, lambda r: r.get("vel_apogee"))
    gpsapg_us  = _first_time_us(ns, lambda r: r.get("gps_apogee"))
    burnout_us = _first_time_us(ns, lambda r: r.get("flags", 0) & NSF_BURNOUT)
    landed_us  = _first_time_us(ns, lambda r: r.get("alt_landed"))

    pyro1_fired_us = _first_time_us(ns, lambda r: r.get("pyro1_fired"))
    pyro2_fired_us = _first_time_us(ns, lambda r: r.get("pyro2_fired"))

    def rel(us):
        return round((us - t0) / 1e6, 3) if us is not None else "—"

    def rel_to_apogee(us):
        if us is None or apogee_us is None:
            return "—"
        return round((us - apogee_us) / 1e6, 3)

    result.metrics = {
        "launch_t_s":            rel(launch_us),
        "burnout_t_s":           rel(burnout_us),
        "apogee_baro_t_s":       rel(apogee_us),
        "apogee_velocity_t_s":   rel(velapg_us),
        "apogee_gps_t_s":        rel(gpsapg_us),
        "landed_t_s":            rel(landed_us),
        "pyro1_fired_t_s":       rel(pyro1_fired_us),
        "pyro2_fired_t_s":       rel(pyro2_fired_us),
        "pyro1_delay_vs_apg_s":  rel_to_apogee(pyro1_fired_us),
        "pyro2_delay_vs_apg_s":  rel_to_apogee(pyro2_fired_us),
    }

    if apogee_us and pyro1_fired_us and (pyro1_fired_us - apogee_us) > 500_000:
        result.warnings.append(
            f"Pyro 1 fired {(pyro1_fired_us - apogee_us)/1e6:.2f}s after apogee — slow.")
    if apogee_us and pyro1_fired_us is None:
        result.warnings.append("Apogee detected but Pyro 1 never fired in log.")

    # Figure: baro alt + event markers
    if baro:
        baro_t = (get_array(baro, "time_us") - t0) / 1e6
        p = get_array(baro, "pressure_pa")
        if p.size > 10:
            p0 = float(np.mean(p[: min(200, p.size)]))
            alt = np.array([pressure_to_altitude(float(pp), p0) for pp in p])
            fig, ax = plt.subplots(figsize=(11, 5))
            ax.plot(baro_t, alt, lw=0.8, color="tab:blue", label="Baro alt (AGL)")

            for label, us, color, style in [
                ("launch",        launch_us,  "green",   "--"),
                ("burnout",       burnout_us, "orange",  ":"),
                ("apogee (baro)", apogee_us,  "red",     "--"),
                ("apogee (vel)",  velapg_us,  "magenta", ":"),
                ("apogee (gps)",  gpsapg_us,  "purple",  ":"),
                ("pyro1",         pyro1_fired_us, "black",  "-"),
                ("pyro2",         pyro2_fired_us, "gray",   "-"),
                ("landed",        landed_us,  "brown",   "--"),
            ]:
                if us is not None:
                    ax.axvline((us - t0) / 1e6, color=color, linestyle=style,
                               alpha=0.7, label=label, lw=1.2)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Altitude AGL (m)")
            ax.set_title("Apogee detection & pyro events")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right", fontsize=8, ncol=2)
            fig.tight_layout()
            result.figures.append(fig)

    return result
