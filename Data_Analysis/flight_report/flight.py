"""Flight data container.

Holds the file-trio (`.bin` / `.csv` / `.json`) for one flight and lazily
parses the binary on first access via `plot_flight_data_mini.parse_binary_file`.
"""

from __future__ import annotations

import json
import os
import re
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

# Reuse the canonical parser
_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))
from plot_flight_data_mini import parse_binary_file  # noqa: E402


_FLIGHT_STEM_RE = re.compile(r"^flight_(?:(\d{8})_(\d{6})|recovered_(\d+))$")


@dataclass
class Flight:
    """One flight's on-disk artifacts + parsed records (lazy)."""

    bin_path: Path
    csv_path: Optional[Path] = None
    json_path: Optional[Path] = None

    # Cached parse results (populated by .load())
    _records: Optional[dict[str, list[dict[str, Any]]]] = field(default=None, repr=False)
    _stats: Optional[dict[str, Any]] = field(default=None, repr=False)
    _config: Optional[dict[str, Any]] = field(default=None, repr=False)
    _sidecar: Optional[dict[str, Any]] = field(default=None, repr=False)
    _t0_us: Optional[int] = field(default=None, repr=False)

    @classmethod
    def from_bin(cls, bin_path: os.PathLike[str] | str) -> "Flight":
        """Build a Flight from a .bin path; pair with .csv / .json sidecars by stem."""
        bin_path = Path(bin_path).resolve()
        stem = bin_path.stem
        parent = bin_path.parent
        csv = parent / f"{stem}.csv"
        js = parent / f"{stem}.json"
        return cls(
            bin_path=bin_path,
            csv_path=csv if csv.exists() else None,
            json_path=js if js.exists() else None,
        )

    # ---- Identity ----------------------------------------------------------
    @property
    def name(self) -> str:
        """Human-readable name: parent-directory + filename stem."""
        return f"{self.bin_path.parent.name} / {self.bin_path.stem}"

    @property
    def rocket_name(self) -> str:
        """Best-effort rocket name from the parent directory."""
        return self.bin_path.parent.name.strip()

    @property
    def date(self) -> Optional[datetime]:
        """Parse flight datetime from `flight_YYYYMMDD_HHMMSS.bin` filename, else None."""
        m = _FLIGHT_STEM_RE.match(self.bin_path.stem)
        if not m:
            return None
        ymd, hms, _recov = m.groups()
        if ymd and hms:
            try:
                return datetime.strptime(f"{ymd}_{hms}", "%Y%m%d_%H%M%S")
            except ValueError:
                return None
        return None

    # ---- Lazy parse --------------------------------------------------------
    def load(self) -> None:
        """Parse the binary and load the JSON sidecar (idempotent)."""
        if self._records is not None:
            return
        self._records, self._stats, self._config = parse_binary_file(str(self.bin_path))
        self._t0_us = self._compute_t0()
        if self.json_path and self.json_path.exists():
            try:
                with self.json_path.open() as f:
                    self._sidecar = json.load(f)
            except (OSError, json.JSONDecodeError) as e:
                self._sidecar = {"_parse_error": str(e)}
        else:
            self._sidecar = {}

    def _compute_t0(self) -> Optional[int]:
        if not self._records:
            return None
        t0s = []
        for recs in self._records.values():
            if recs and "time_us" in recs[0]:
                t0s.append(recs[0]["time_us"])
        return min(t0s) if t0s else None

    # ---- Accessors (auto-load) --------------------------------------------
    @property
    def records(self) -> dict[str, list[dict[str, Any]]]:
        self.load()
        assert self._records is not None
        return self._records

    @property
    def stats(self) -> dict[str, Any]:
        self.load()
        assert self._stats is not None
        return self._stats

    @property
    def config(self) -> dict[str, Any]:
        self.load()
        assert self._config is not None
        return self._config

    @property
    def sidecar(self) -> dict[str, Any]:
        """Parsed `.json` sidecar (may only contain summary metrics for older flights)."""
        self.load()
        assert self._sidecar is not None
        return self._sidecar

    @property
    def t0_us(self) -> Optional[int]:
        self.load()
        return self._t0_us

    @property
    def duration_s(self) -> Optional[float]:
        recs = self.records
        ts = [r["time_us"] for sensor in recs.values() for r in sensor if "time_us" in r]
        if not ts:
            return None
        return (max(ts) - min(ts)) / 1e6
