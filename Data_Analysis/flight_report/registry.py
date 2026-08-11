"""Analysis module registry and shared result type."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, TYPE_CHECKING

if TYPE_CHECKING:
    import matplotlib.figure
    from .flight import Flight


@dataclass
class AnalysisResult:
    """Output of one analysis module for one flight."""

    name: str                              # short id, e.g. "gaps"
    title: str                             # display title, e.g. "Frame Gaps"
    metrics: dict[str, Any] = field(default_factory=dict)
    # Column headings for the metrics table. "Metric"/"Value" suits a list of
    # measurements; a table that is not measurements needs to say so.
    metric_headers: tuple[str, str] = ("Metric", "Value")
    figures: list["matplotlib.figure.Figure"] = field(default_factory=list)
    charts: list[dict[str, Any]] = field(default_factory=list)  # Plotly specs, see charts.py
    maps: list[dict[str, Any]] = field(default_factory=list)    # Leaflet specs, see maps.py
    globes: list[dict[str, Any]] = field(default_factory=list)  # Cesium specs, see modules/globe.py
    warnings: list[str] = field(default_factory=list)
    text: str = ""                         # optional pre-formatted block (monospace)
    error: str | None = None               # if module crashed, captured here


AnalyzeFn = Callable[["Flight"], AnalysisResult]

# Report levels. FLIGHT answers "how did my rocket fly?" and is what a flyer
# reads; DETAILED is board bring-up and firmware validation. A module belongs
# to exactly one — anything a flyer needs from an detailed-level module should be
# rolled up into a FLIGHT-level module rather than shown twice.
LEVEL_FLIGHT = "flight"
LEVEL_DETAILED = "detailed"
LEVELS = (LEVEL_FLIGHT, LEVEL_DETAILED)


def _build_module_list() -> list[tuple[str, AnalyzeFn, str]]:
    """Import each module and collect its `analyze` function. Order = report order."""
    from .modules import (
        overview,
        deployment,
        globe,
        roll,
        apogee,
        sample_rates,
        stability,
        motor,
        health,
        kinematics,
        gaps,
        launch_detection,
        pyro_apogee,
        sensor_noise,
        gnss_staleness,
        kinematic_checks,
        roll_pid,
        guidance,
        lora,
        log_buffer,
        timestamps,
    )

    return [
        # The 3D path leads: it is the one view that shows the whole flight at
        # once, and it is what a flyer wants to see first. The charts that follow
        # then run position -> velocity -> acceleration.
        ("globe",             globe.analyze,             LEVEL_FLIGHT),
        ("overview",          overview.analyze,          LEVEL_FLIGHT),
        # Directly below the kinematic charts: roll is the one axis those three
        # say nothing about, and on a roll-control flight it is the whole story.
        ("roll",              roll.analyze,              LEVEL_FLIGHT),
        ("apogee",            apogee.analyze,            LEVEL_FLIGHT),
        ("stability",         stability.analyze,         LEVEL_FLIGHT),
        # Above Motor Performance: a sensor that logged short makes every
        # number below it thinner than it looks. Still in the detailed report
        # too, alongside the gap analysis it comes from.
        ("sample_rates",      sample_rates.analyze,      LEVEL_FLIGHT),
        ("motor",             motor.analyze,             LEVEL_FLIGHT),
        ("health",            health.analyze,            LEVEL_FLIGHT),
        ("sensor_charts",     overview.analyze_sensors,  LEVEL_DETAILED),
        ("kinematics",        kinematics.analyze,        LEVEL_DETAILED),
        ("gaps",              gaps.analyze,              LEVEL_DETAILED),
        ("timestamps",        timestamps.analyze,        LEVEL_DETAILED),
        ("launch_detection",  launch_detection.analyze,  LEVEL_DETAILED),
        # Deployment & recovery — descent profile, the flat ground-track map and
        # the KML links. Detailed-level by decision: the flat map is the one that
        # still draws with no network (it degrades to graph paper with the track
        # on it), so it lives with the rest of the diagnostic set rather than
        # duplicating the globe in the flight report.
        ("deployment",        deployment.analyze,        LEVEL_DETAILED),
        ("pyro_apogee",       pyro_apogee.analyze,       LEVEL_DETAILED),
        ("sensor_noise",      sensor_noise.analyze,      LEVEL_DETAILED),
        ("gnss_staleness",    gnss_staleness.analyze,    LEVEL_DETAILED),
        ("kinematic_checks",  kinematic_checks.analyze,  LEVEL_DETAILED),
        ("roll_pid",          roll_pid.analyze,          LEVEL_DETAILED),
        ("guidance",          guidance.analyze,          LEVEL_DETAILED),
        ("lora",              lora.analyze,              LEVEL_DETAILED),
        ("log_buffer",        log_buffer.analyze,        LEVEL_DETAILED),
    ]


MODULES: list[tuple[str, AnalyzeFn, str]] = _build_module_list()


def modules_for(level: str | None) -> list[tuple[str, AnalyzeFn, str]]:
    """Modules for one level, or all of them when `level` is None."""
    if level is None:
        return list(MODULES)
    if level not in LEVELS:
        raise ValueError(f"unknown report level {level!r}; expected one of {LEVELS}")
    return [m for m in MODULES if m[2] == level]


def run_module(name: str, fn: AnalyzeFn, flight: "Flight") -> AnalysisResult:
    """Invoke a module; capture any exception into `result.error`."""
    try:
        return fn(flight)
    except Exception as e:  # noqa: BLE001  — by design, modules don't take down report
        import traceback
        return AnalysisResult(
            name=name,
            title=name.replace("_", " ").title(),
            error=f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}",
        )
