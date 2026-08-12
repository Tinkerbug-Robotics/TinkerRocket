"""What the parser found in the file, and how much of it survived the trip.

The counts are a completeness check on everything above them. A stream whose
count is far below what its configured rate implies did not record what it was
asked to, and every number derived from it is thinner than it looks. A bad-CRC
count above a handful means frames were corrupted in flash or over the wire, and
the affected samples are simply missing.
"""

from __future__ import annotations

from ..flight import Flight
from ..registry import AnalysisResult

# Under this it is background noise from flash wear, not a data problem.
_BAD_CRC_WARN_FRACTION = 0.001


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="parser_stats", title="Log Contents")
    result.metric_headers = ("Message", "Frames")

    stats = flight.stats or {}
    total = stats.get("total_frames")
    good = stats.get("good_crc")
    bad = stats.get("bad_crc")

    counts = stats.get("type_counts") or {}
    if counts:
        result.metrics = {name: f"{count:,}" for name, count
                          in sorted(counts.items(), key=lambda kv: -kv[1])}
    if not result.metrics:
        result.warnings.append("The parser reported no message counts for this log.")
        return result

    bits = []
    if stats.get("file_size") is not None:
        bits.append(f"{stats['file_size'] / 1024 / 1024:.1f} MB on disk")
    if total is not None:
        bits.append(f"{total:,} frames")
    if bad is not None and total:
        pct = 100.0 * (good or 0) / total
        bits.append(f"{pct:.2f}% passed CRC" + (f" ({bad:,} did not)" if bad else ""))
    result.text = " · ".join(bits)

    # A handful of bad frames is ordinary on a full flash and not worth a warning
    # banner; a rate that starts costing real samples is.
    if bad and total and bad / total > _BAD_CRC_WARN_FRACTION:
        result.warnings.append(
            f"{bad:,} of {total:,} frames failed their checksum and were dropped "
            f"({100.0 * bad / total:.2f}%). Those samples are missing from the "
            "record the rest of this report is built on."
        )
    return result
