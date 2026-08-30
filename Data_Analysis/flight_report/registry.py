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
    # Raw channel data for the Explore panel to plot on demand. Shipped through
    # the same gzip transport as charts, but it is not a chart: it has no traces
    # and nothing draws until the reader asks for something.
    datasets: list[dict[str, Any]] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    text: str = ""                         # optional pre-formatted block (monospace)
    error: str | None = None               # if module crashed, captured here


AnalyzeFn = Callable[["Flight"], AnalysisResult]

# One report. There used to be a second, "detailed" level for board bring-up and
# firmware validation, and the two drifted: the same word meant different
# instants in each, and a number a flyer read on one contradicted the other. The
# sections worth keeping were folded into this one and the split was removed.
#
# The level is still carried because the renderer and the CLI name it, and
# because a second document may come back for a different audience — but nothing
# should read it as "which of two reports is this".
LEVEL_FLIGHT = "flight"
LEVELS = (LEVEL_FLIGHT,)


def _build_module_list() -> list[tuple[str, AnalyzeFn, str]]:
    """Import each module and collect its `analyze` function. Order = report order.

    Only modules that appear in the table below are imported. `modules/` also
    holds `guidance.py`, which is not: no committed log carries a guidance frame,
    so at flight level it would print "not a guidance flight" on every report. It
    is kept as an offline tool because it derives engage and cutoff conditions
    that its standalone script does not — import it by hand to use it.
    """
    from .modules import (
        overview,
        deployment,
        globe,
        explore,
        roll,
        apogee,
        sample_rates,
        stability,
        health,
        rocket_state,
        barometer,
        lora_link,
        parser_stats,
        timing,
        settings,
        gaps,
        kinematic_checks,
        roll_pid,
        timestamps,
    )

    return [
        # The 3D path leads: it is the one view that shows the whole flight at
        # once, and it is what a flyer wants to see first. The charts that follow
        # then run position -> velocity -> acceleration.
        ("globe",             globe.analyze,             LEVEL_FLIGHT),
        # Directly under the 3D path: that view answers "where did it go", and
        # this answers "show me anything else", before the curated sections
        # start answering questions somebody else chose.
        ("explore_time",      explore.analyze_time,      LEVEL_FLIGHT),
        # Against another channel rather than the clock. A separate section, not
        # a mode toggle: it needs an X picker and is confined to one stream, and
        # putting that behind a radio made the common case carry it too.
        ("explore_xy",        explore.analyze_xy,        LEVEL_FLIGHT),
        ("overview",          overview.analyze,          LEVEL_FLIGHT),
        # Directly below the kinematic charts: roll is the one axis those three
        # say nothing about, and on a roll-control flight it is the whole story.
        ("roll",              roll.analyze,              LEVEL_FLIGHT),
        # The control-loop view of the same axis: plant gain, oscillation,
        # phase margin, and recommended gains. Skips itself when roll_cmd ≡ 0,
        # so non-control flights never see it.
        ("roll_pid",          roll_pid.analyze,          LEVEL_FLIGHT),
        ("apogee",            apogee.analyze,            LEVEL_FLIGHT),
        # Straight after the detectors: this is where their votes land, and the
        # step out of coast is the master flag latching.
        ("rocket_state",      rocket_state.analyze,      LEVEL_FLIGHT),
        ("stability",         stability.analyze,         LEVEL_FLIGHT),
        # Late, but on the flight report rather than only in the detailed one:
        # a sensor that logged short makes every number above it thinner than
        # it looks, so it belongs where the numbers are read.
        ("sample_rates",      sample_rates.analyze,      LEVEL_FLIGHT),
        ("health",            health.analyze,            LEVEL_FLIGHT),
        # The tail of the report: what happened after the charge fired, then the
        # raw signal behind every altitude, then the radio, then the record of
        # the record itself. Reference rather than reading, in that order.
        ("deployment",        deployment.analyze,        LEVEL_FLIGHT),
        ("barometer",         barometer.analyze,         LEVEL_FLIGHT),
        ("lora_link",         lora_link.analyze,         LEVEL_FLIGHT),
        ("parser_stats",      parser_stats.analyze,      LEVEL_FLIGHT),
        ("timing",            timing.analyze,            LEVEL_FLIGHT),
        ("settings",          settings.analyze,          LEVEL_FLIGHT),
        # Deployment & recovery — descent profile, the flat ground-track map and
        # the KML links. Detailed-level by decision: the flat map is the one that
        # still draws with no network (it degrades to graph paper with the track
        # on it), so it lives with the rest of the diagnostic set rather than
        # duplicating the globe in the flight report.
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
