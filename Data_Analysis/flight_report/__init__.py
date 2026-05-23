"""TinkerRocket unified flight-analysis suite.

Discovers `flight_*.bin` logs, runs a set of analysis modules over each,
and renders a self-contained per-flight HTML report.

Usage:
    python -m flight_report run                       # default discovery root
    python -m flight_report run /path/to/flights/     # walk a directory
    python -m flight_report run flight_XXX.bin        # single flight
"""

from .flight import Flight
from .registry import AnalysisResult, MODULES

__all__ = ["Flight", "AnalysisResult", "MODULES"]
