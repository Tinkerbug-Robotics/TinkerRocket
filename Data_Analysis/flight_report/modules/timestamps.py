"""Timestamps module — large jumps in time_us across the merged stream.

Lighter cousin of `gaps`: just calls out >1 s positive or any negative jumps.
"""

from __future__ import annotations

import numpy as np

from ..flight import Flight
from ..registry import AnalysisResult


# Frame types stamped with the OutComputer's own micros() rather than the
# FC clock the sensor stream uses.  The clocks are offset by however long one
# board booted before the other (minutes), so mixing them into the merged
# stream manufactures one giant fake "jump" per flight.  Their internal
# cadence is still valid — they are only excluded from the MERGED analysis.
OC_CLOCK_TYPES = {"LogBufferStats"}


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="timestamps", title="Timestamp Anomalies")
    recs = flight.records

    all_ts = []
    for sname, rlist in recs.items():
        if sname in OC_CLOCK_TYPES:
            continue
        for r in rlist:
            if "time_us" in r:
                all_ts.append((r["time_us"], sname))
    all_ts.sort(key=lambda x: x[0])

    if len(all_ts) < 2:
        result.warnings.append("Not enough timestamped records.")
        return result

    times = np.array([t for t, _ in all_ts], dtype=np.int64)
    deltas = np.diff(times)

    # Negative jumps shouldn't exist after the sort; if they did before sort, that's the bug.
    # Look at raw per-sensor monotonicity instead.
    nonmono_per_sensor = {}
    for sname, rlist in recs.items():
        ts = np.array([r["time_us"] for r in rlist if "time_us" in r], dtype=np.int64)
        if ts.size < 2:
            continue
        bad = int(np.sum(np.diff(ts) < 0))
        if bad > 0:
            nonmono_per_sensor[sname] = bad

    big = int(np.sum(deltas > 1_000_000))   # >1 s in merged stream
    huge = int(np.sum(deltas > 10_000_000))  # >10 s — likely reboot/log split

    excluded = sorted(s for s in OC_CLOCK_TYPES if recs.get(s))
    result.metrics = {
        "merged_records": len(all_ts),
        "merged_span_s": round((times[-1] - times[0]) / 1e6, 2),
        "merged_jumps_gt_1s": big,
        "merged_jumps_gt_10s": huge,
        "nonmono_sensors": ", ".join(f"{k}={v}" for k, v in nonmono_per_sensor.items()) or "none",
        "excluded_oc_clock_types": ", ".join(excluded) or "none",
    }

    if huge:
        result.warnings.append(f"{huge} jump(s) > 10 s in merged stream — possible reboot or split log.")
    if nonmono_per_sensor:
        for k, v in nonmono_per_sensor.items():
            result.warnings.append(f"{k}: {v} non-monotonic timestamps.")

    return result
