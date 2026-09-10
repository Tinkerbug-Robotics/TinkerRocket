"""What the parser found in the file, and how much of it survived the trip.

The counts are a completeness check on everything above them. A stream whose
count is far below what its configured rate implies did not record what it was
asked to, and every number derived from it is thinner than it looks. A bad-CRC
count above a handful means frames were corrupted in flash or over the wire, and
the affected samples are simply missing.
"""

from __future__ import annotations

from .. import catalog
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
        # #752: a row here used to read "0xD2 — 774 frames" and stop, which reads
        # as a sensor nobody recognised rather than as data this tool throws
        # away. Name the ones the parser produced nothing from, and why.
        try:
            _unreadable_list = catalog.unreadable_for(flight)
        except Exception:
            _unreadable_list = []
        reasons = {u.name: u.reason for u in _unreadable_list}
        result.metrics = {
            name: (f"{count:,} — {reasons[name]}" if name in reasons else f"{count:,}")
            for name, count in sorted(counts.items(), key=lambda kv: -kv[1])
        }
        n_undecoded = sum(1 for u in _unreadable_list if not u.decoded_elsewhere)
        if n_undecoded:
            result.note = (
                f"{n_undecoded} message type(s) in this log have no decoder — their frames "
                "are counted but nothing in this report is built from them."
            )
    if not result.metrics:
        result.warnings.append("The parser reported no message counts for this log.")
        return result

    bits = []
    if stats.get("file_size") is not None:
        bits.append(f"{stats['file_size'] / 1024 / 1024:.1f} MB on disk")
    if total is not None:
        bits.append(f"{total:,} frames")
    if bad is not None and total:
        # Say it one way or the other. Rounding 99.998% to "100.00%" and then
        # appending "(3 did not)" reads as a contradiction; below the rounding
        # threshold the count is the honest form.
        pct = 100.0 * (good or 0) / total
        bits.append(f"{bad:,} of {total:,} frames failed CRC" if bad
                    else f"all {total:,} frames passed CRC")
    result.text = " · ".join(bits)

    # A handful of bad frames is ordinary on a full flash and not worth a warning
    # banner; a rate that starts costing real samples is.
    if bad and total and bad / total > _BAD_CRC_WARN_FRACTION:
        result.warnings.append(
            f"{bad:,} of {total:,} frames failed their checksum and were dropped "
            f"({100.0 * bad / total:.2f}%). Those samples are missing from the "
            "record the rest of this report is built on."
        )

    # #752: a snapshot frame that fails its magic or CRC is corruption, not a
    # data point — this is the frame the FC restores ITSELF from after an
    # in-flight reboot, so it is worth saying out loud when one is refused.
    rej = stats.get("snapshot_rejects") or {}
    n_rej = sum(rej.values())
    if n_rej:
        parts = [f"{v:,} {k}" for k, v in rej.items() if v]
        result.warnings.append(
            f"{n_rej:,} flight-snapshot frame(s) were refused ({', '.join(parts)}) "
            "and are absent from the snapshot stream. A magic or checksum failure "
            "means the frame was corrupted; the flight computer would have "
            "rejected the same frame as a recovery source."
        )

    # Fields the parser reads off the wire and then discards. They are absent
    # from every record, so no amount of looking will find them — saying so is
    # the difference between a known gap and an invisible one.
    if catalog.DROPPED_FIELDS:
        result.groups.append({
            "name": "Decoded, then discarded",
            "rows": [(text.split(" — ")[0], text.split(" — ", 1)[1]
                      if " — " in text else "")
                     for text in catalog.DROPPED_FIELDS],
        })
    return result
