"""Explore — plot any channel in the log, from inside the report.

This is the section #752 exists for. Every other section answers a question
somebody chose in advance; this one ships the data and lets the reader ask.

**Why the data is here rather than fetched.** The report is a single file you
can mail, archive, or open off a USB stick two years later with no network and
no Python. A picker that needs a server would not survive that, and it is the
property that makes these reports worth keeping.

**The encoding, and why it is affordable.** Measured on a 154k-frame flight,
embedding all 97 channels the obvious way — a time vector alongside each series,
the way `charts.py` builds a trace — costs 10.6 MB gzipped. Three changes take
it to 3.9 MB, and the first is nearly all of it:

  · One time vector per *stream*, not per channel. Every channel of a stream
    shares its frame timestamps, so 56 NonSensor channels were shipping 56
    copies of the same array. This alone is 10.6 MB -> 3.9 MB.
  · Constant channels carry their value, not 34,144 copies of it. Barely
    changes the gzipped size — gzip already collapses a run — but it halves the
    raw JSON, which is what the browser holds after parsing.
  · Booleans and coded states ship as runs: the index and value at each change.
    Same reasoning, and a run table is what a step plot wants anyway.

Nothing is decimated. The viewport renderer already draws a min/max envelope
sized to the screen and re-slices from the full arrays on zoom, so the samples
are all here and reachable — see the note at the top of `charts.py`.
"""

from __future__ import annotations

import numbers
from typing import Any, Optional

from ..catalog import KIND_BOOL, KIND_COUNTER, KIND_ENUM, build
from ..charts import TIME_DECIMALS
from ..flight import Flight
from ..registry import AnalysisResult

import re

_DISCRETE = (KIND_BOOL, KIND_ENUM, KIND_COUNTER)

# "(#112)" and ", #514" and bare "#514" — the reference, not the sentence.
_ISSUE_REF = re.compile(r"\s*[(\[]?,?\s*#\d+\s*[)\]]?")
_SPACES = re.compile(r"\s{2,}")

# What the time panel opens showing, in preference order; the first one this log
# actually carries wins. A blank plot asks the reader to go and find something
# before the panel has shown it can plot anything at all, and GNSS altitude is
# the channel every flight has and everyone recognises.
_DEFAULT_CHANNELS = ("GNSS.alt_m", "NonSensor.u_pos", "BMP585.pressure_pa")


def _first_present(channels: dict, wanted: tuple[str, ...]) -> list[str]:
    for key in wanted:
        if key in channels:
            return [key]
    return []

# Both Explore sections read one embedded payload. It is the largest thing in
# the document, so the second section references this id rather than carrying
# its own copy — see `analyze_xy`.
DATASET_ID = "explore-data"

# Time at the log clock's microsecond and samples to 4 decimals, matching what
# `charts._clean` ships — see TIME_DECIMALS there for why anything coarser
# stacks the IMU's samples. Digits that survive rounding are digits in every
# copy of the report.
_T_DP = TIME_DECIMALS
_V_DP = 4

# Except where a unit is enormous. A degree of latitude is 111 km, so the default
# four decimals quantises a position to 11 m: on the sample flight that collapsed
# 1,397 GNSS fixes into 21 distinct points, and plotting lat against lon drew a
# staircase rather than a track. Seven decimals is 1.1 cm, comfortably finer than
# any receiver, and costs about 40 kB on that flight.
_V_DP_BY_KEY = {"GNSS.lat": 7, "GNSS.lon": 7}


def _public(text: Optional[str]) -> Optional[str]:
    """A caution with its issue references removed.

    The channel catalog is written for whoever is debugging the firmware, and
    its cautions cite the issues that established each quirk. The substance is
    exactly what a reader plotting that channel needs; the issue numbers mean
    nothing outside the repository and read as leaked internals in a report a
    customer opens.
    """
    if not text:
        return text
    out = _ISSUE_REF.sub("", text)
    return _SPACES.sub(" ", out).replace(" ,", ",").replace(" .", ".").strip()


def _values(rows: list[dict], field: str, kind: str, decimals: int = _V_DP) -> list:
    """One channel as int / float / None, ready to serialize.

    Kept as Python objects rather than a numpy array on purpose: run detection
    below compares consecutive entries, and `nan != nan` would make every
    missing sample look like a change. `None == None` does the right thing.
    """
    out: list = []
    for r in rows:
        v = r.get(field)
        if v is None:
            out.append(None)
        elif kind in _DISCRETE:
            out.append(int(v))
        else:
            f = float(v)
            # Non-finite is not valid JSON and render.py serializes with
            # allow_nan=False, outside any error containment — one of these
            # would take down the whole document rather than one channel.
            out.append(round(f, decimals) if f == f and abs(f) != float("inf") else None)
    return out


def _shape(rows: list[dict], field: str) -> str:
    """What a channel's values are: "number", "text" or "other".

    Not every field a parser writes is a reading. Newer streams carry named
    states as text (Snapshot.gnss_admission's "held_out", PadCalStatus's
    "idle"), and some carry a whole table per row: GNSS_SAT.sats is a list of
    satellites, PadContext a calibration matrix, GNSS_RTCM raw bytes. The
    catalog lists them all, and the section used to put each through float(),
    which took the whole section down on every log that had one.
    """
    shape = "number"
    for r in rows:
        v = r.get(field)
        if v is None:
            continue
        if isinstance(v, str):
            shape = "text"
        elif not isinstance(v, numbers.Number):
            return "other"
    return shape


def _codes(rows: list[dict], field: str) -> tuple[list, list[str]]:
    """A text channel as integer codes, numbered in the order the states first
    appear, and the names those codes stand for.

    First appearance rather than alphabetical, so a state machine's step plot
    climbs as the flight goes on (held_out, then admitted_baro_agreed) instead
    of jumping around in an order nobody chose.
    """
    levels: list[str] = []
    index: dict[str, int] = {}
    out: list = []
    for r in rows:
        v = r.get(field)
        if v is None:
            out.append(None)
            continue
        s = str(v)
        if s not in index:
            index[s] = len(levels)
            levels.append(s)
        out.append(index[s])
    return out, levels


def _legend(levels: list[str]) -> str:
    return "Plotted as numbers: " + ", ".join(f"{i} = {name}" for i, name in enumerate(levels)) + "."


def _runs(vals: list) -> list[list]:
    """[[index, value], ...] at each change, including index 0."""
    out = [[0, vals[0]]]
    for i in range(1, len(vals)):
        if vals[i] != vals[i - 1]:
            out.append([i, vals[i]])
    return out


def _dataset(flight: Flight) -> Optional[dict[str, Any]]:
    cat = build(flight)
    if not cat.channels or flight.t0_us is None:
        return None

    recs = flight.records
    t0 = flight.t0_us

    streams: dict[str, Any] = {}
    for name in cat.streams():
        rows = recs[name]
        streams[name] = {
            "t": [round((r["time_us"] - t0) / 1e6, _T_DP) for r in rows],
            "n": len(rows),
        }

    channels: dict[str, Any] = {}
    skipped: list[str] = []
    for c in cat.channels:
        rows = recs[c.stream]
        shape = _shape(rows, c.field)
        if shape == "other":
            skipped.append(c.key)
            continue
        kind = c.meta.kind
        caution = _public(c.meta.caution)
        if shape == "text":
            # Named states, whatever the catalog guessed for an undocumented
            # field: step-drawn, with the names beside the plot.
            vals, levels = _codes(rows, c.field)
            kind = KIND_ENUM
            caution = " ".join(x for x in (caution, _legend(levels)) if x)
        else:
            vals = _values(rows, c.field, kind, _V_DP_BY_KEY.get(c.key, _V_DP))
        entry: dict[str, Any] = {
            "stream": c.stream,
            "label": c.meta.label,
            "unit": c.meta.unit,
            # Which CONVERSIONS row drives the imperial toggle, when it differs
            # from the display unit. Vertical rates are "m/s" on screen but
            # convert to ft/s, not mph — units.py keeps a separate row for that
            # and says quoting a sink rate in mph "reads as an error to anyone
            # in the hobby". Omitted when it adds nothing, to save the bytes.
            "conv": c.meta.conv if c.meta.conv and c.meta.conv != c.meta.unit else "",
            "kind": kind,
            # Carried so the picker can warn in place rather than making the
            # reader go and find the inventory section.
            "caution": caution,
            "shown": list(c.meta.shown_in),
        }
        if len(set(vals)) == 1:
            entry["const"] = vals[0]
        elif kind in (KIND_BOOL, KIND_ENUM):
            entry["runs"] = _runs(vals)
        else:
            entry["y"] = vals
        channels[c.key] = entry

    return {
        "id": DATASET_ID,
        "streams": streams,
        "channels": channels,
        # Step-drawn kinds, so the panel does not have to re-derive the rule.
        "stepKinds": [KIND_BOOL, KIND_ENUM],
        # Fields that hold a table per row rather than one value. Said in the
        # section rather than dropped silently; the time panel pops it off
        # before the payload ships.
        "skipped": skipped,
    }


def analyze_time(flight: Flight) -> AnalysisResult:
    """Section one: any channel against time.

    The common case by a wide margin, so it leads and carries no controls it
    does not need — no X picker, because the X axis is the clock.
    """
    result = AnalysisResult(name="explore_time", title="Plot Data Channels")

    data = _dataset(flight)
    if data is None:
        result.warnings.append("No channels parsed from this log — nothing to plot.")
        return result

    n_const = sum(1 for c in data["channels"].values() if "const" in c)
    skipped = data.pop("skipped")
    result.metrics["Channels available"] = f"{len(data['channels']):,}"
    result.metrics["Held one value all flight"] = f"{n_const:,}"
    result.metrics["Streams"] = ", ".join(data["streams"])
    if skipped:
        result.metrics["Not plottable"] = (
            f"{len(skipped)} field{'s' if len(skipped) != 1 else ''} hold a list or raw bytes "
            f"in each row, not one value: {', '.join(skipped)}"
        )

    data["panel"] = "time"
    data["default"] = _first_present(data["channels"], _DEFAULT_CHANNELS)
    result.datasets = [data]
    return result


def analyze_xy(flight: Flight) -> AnalysisResult:
    """Section two: one channel against another.

    References the payload the time section embedded instead of building its
    own. The dataset is ~3.9 MB gzipped on a real flight and duplicating it
    would double the document to save one dictionary lookup in the browser.

    Kept a separate section rather than a mode toggle because the two answer
    different questions and want different controls: this one needs an X axis
    and is confined to a single stream, and putting that behind a radio button
    made the common case carry the rarer one's machinery.
    """
    result = AnalysisResult(name="explore_xy", title="Plot One Channel Against Another")

    if flight.t0_us is None or not flight.records:
        result.warnings.append("No channels parsed from this log — nothing to plot.")
        return result

    result.metrics["Pairing"] = (
        "Both axes must come from the same stream, so each point is one logged "
        "frame rather than an interpolated pair."
    )
    # A reference, not a payload: render.py emits no script tag for these.
    result.datasets = [{"id": DATASET_ID, "panel": "xy", "ref": True}]
    return result
