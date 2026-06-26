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
    figures: list["matplotlib.figure.Figure"] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    text: str = ""                         # optional pre-formatted block (monospace)
    error: str | None = None               # if module crashed, captured here


AnalyzeFn = Callable[["Flight"], AnalysisResult]


def _build_module_list() -> list[tuple[str, AnalyzeFn]]:
    """Import each module and collect its `analyze` function. Order = report order."""
    from .modules import (
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
        ("kinematics",        kinematics.analyze),
        ("gaps",              gaps.analyze),
        ("timestamps",        timestamps.analyze),
        ("launch_detection",  launch_detection.analyze),
        ("pyro_apogee",       pyro_apogee.analyze),
        ("sensor_noise",      sensor_noise.analyze),
        ("gnss_staleness",    gnss_staleness.analyze),
        ("kinematic_checks",  kinematic_checks.analyze),
        ("roll_pid",          roll_pid.analyze),
        ("guidance",          guidance.analyze),
        ("lora",              lora.analyze),
        ("log_buffer",        log_buffer.analyze),
    ]


MODULES: list[tuple[str, AnalyzeFn]] = _build_module_list()


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
