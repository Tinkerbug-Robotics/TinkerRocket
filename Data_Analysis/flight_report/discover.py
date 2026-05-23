"""Find flight_*.bin files (and their sidecars) under a path."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

from .flight import Flight


DEFAULT_DISCOVERY_ROOT = Path.home() / "Documents" / "Hobbies" / "ModelRockets" / "TestFlights"


def discover(path: Path | str | None = None) -> list[Flight]:
    """
    Walk `path` (or DEFAULT_DISCOVERY_ROOT) for `flight_*.bin` files.

    If `path` is a single .bin file, returns a one-element list.
    Returns Flights sorted by .bin path.
    """
    if path is None:
        root = DEFAULT_DISCOVERY_ROOT
    else:
        root = Path(path).expanduser().resolve()

    if not root.exists():
        return []

    if root.is_file() and root.suffix == ".bin":
        return [Flight.from_bin(root)]

    bins = sorted(p for p in root.rglob("flight_*.bin") if p.is_file())
    return [Flight.from_bin(p) for p in bins]


def filter_flights(
    flights: Iterable[Flight],
    min_size_bytes: int = 1024,
) -> list[Flight]:
    """Drop obvious junk: empty / truncated files. Reports only what was kept."""
    out = []
    for f in flights:
        try:
            if f.bin_path.stat().st_size < min_size_bytes:
                continue
        except OSError:
            continue
        out.append(f)
    return out
