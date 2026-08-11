"""Unit-aware display values, so the report can switch metric/imperial at view time.

Everything upstream computes in SI. A `Quantity` carries the SI number plus its
unit, and renders as ``<span class="q" data-si="357.1" data-unit="m">357.1 m</span>``.
The template's toggle rewrites those spans in place — no regeneration, and the
downloaded file keeps working either way.

Values that are *not* convertible (seconds, G, degrees, counts) can still be
Quantities; they simply render the same in both systems, which keeps call sites
uniform.
"""

from __future__ import annotations

from typing import Optional

from markupsafe import Markup, escape

# conversion key -> (factor from SI, imperial label, decimal places in imperial).
# Keyed by conversion rather than by unit because one SI unit can want different
# imperial idioms: rocketry quotes airspeed in mph but descent rate in ft/s, and
# printing a chute's sink rate in mph reads as an error to anyone in the hobby.
# An explicit identity entry is required for non-convertible units so that a typo
# is a loud KeyError rather than a silent no-op.
CONVERSIONS = {
    "m": (3.280839895, "ft", 0),
    "km": (0.621371192, "mi", 2),
    "m/s": (2.236936292, "mph", 1),      # airspeed
    "m/s:fps": (3.280839895, "ft/s", 1),  # vertical / descent rate
    "m/s²": (3.280839895, "ft/s²", 1),
    # Identical in both systems.
    "s": None,
    "G": None,
    "N·s": None,   # impulse: SI either way
    "N": None,     # thrust: SI either way
    "kg": (2.204622622, "lb", 3),
    "°": None,
    "": None,
}


def _fmt(value: float, places: int) -> str:
    return f"{value:,.{places}f}"


class Quantity:
    """One measured value: SI number, unit, and how many decimals to show.

    `conv` names the row in CONVERSIONS; it defaults to `unit` and is only passed
    explicitly when one SI unit needs more than one imperial idiom.
    """

    __slots__ = ("value", "unit", "places", "suffix", "conv")

    def __init__(self, value: float, unit: str, places: int = 1, suffix: str = "",
                 conv: Optional[str] = None):
        key = conv or unit
        if key not in CONVERSIONS:
            raise KeyError(
                f"unknown unit {key!r} — add it to units.CONVERSIONS, with None "
                "if it reads the same in both systems"
            )
        self.value = float(value)
        self.unit = unit
        self.places = places
        self.suffix = suffix  # trailing prose, e.g. a compass point
        self.conv = key

    def metric_text(self) -> str:
        return f"{_fmt(self.value, self.places)} {self.unit}{self.suffix}".rstrip()

    def imperial_text(self) -> str:
        conv = CONVERSIONS[self.conv]
        if conv is None:
            return self.metric_text()
        factor, label, places = conv
        return f"{_fmt(self.value * factor, places)} {label}{self.suffix}".rstrip()

    def __html__(self) -> str:
        """Rendered by Jinja directly (autoescape calls this)."""
        return str(Markup(
            '<span class="q" data-si="{si}" data-conv="{conv}" data-unit="{unit}" '
            'data-places="{places}" data-suffix="{suffix}">{text}</span>'
        ).format(
            si=repr(round(self.value, 6)),
            conv=self.conv,
            unit=self.unit,
            places=self.places,
            suffix=self.suffix,
            text=escape(self.metric_text()),
        ))

    def __str__(self) -> str:  # plain-text contexts (logs, tests)
        return self.metric_text()


def q(value: Optional[float], unit: str, places: int = 1, suffix: str = "",
      conv: Optional[str] = None) -> Optional[Quantity]:
    """Quantity, or None when there's no value — call sites can skip the row."""
    if value is None:
        return None
    return Quantity(value, unit, places, suffix, conv)
