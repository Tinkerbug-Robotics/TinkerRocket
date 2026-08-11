"""Vehicle health — one verdict per subsystem, so the detail stays optional.

The detailed report already dissects battery, GNSS, logging and radio at
length. A flyer needs only to know whether anything needs attention before the
next flight, so each subsystem reduces to OK / CHECK / PROBLEM with the number
that decided it. Anything worse than OK also raises a warning, which surfaces in
the report's table of contents.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent.parent
if str(_PARENT) not in sys.path:
    sys.path.insert(0, str(_PARENT))

from plot_flight_data_mini import get_array  # noqa: E402

from ..flight import Flight
from ..registry import AnalysisResult

OK, CHECK, PROBLEM = "OK", "CHECK", "PROBLEM"

# A 2S lithium pack: 7.4 V nominal, 6.0 V is empty and unhealthy under load.
BATT_LOW_V = 7.0
BATT_CRITICAL_V = 6.6
# Below this many satellites a fix is not trustworthy for recovery.
SATS_MIN = 6
PDOP_MAX = 5.0


def _verdict(status: str, detail: str) -> dict[str, Any]:
    return {"status": status, "detail": detail}


def _battery(recs) -> Optional[dict[str, Any]]:
    pw = recs.get("POWER") or []
    if not pw or "voltage" not in pw[0]:
        return None
    v = get_array(pw, "voltage")
    v = v[np.isfinite(v) & (v > 1.0)]  # drop pre-init zeros
    if not v.size:
        return None

    low = float(np.min(v))
    detail = f"{low:.2f} V lowest, {float(v[-1]):.2f} V at the end"
    soc = get_array(pw, "soc")
    if soc.size:
        detail += f" · {float(soc[-1]):.0f}% charge remaining"

    if low < BATT_CRITICAL_V:
        return _verdict(PROBLEM, detail + " — the pack sagged below a safe level under load")
    if low < BATT_LOW_V:
        return _verdict(CHECK, detail + " — charge before the next flight")
    return _verdict(OK, detail)


def _gnss(recs) -> Optional[dict[str, Any]]:
    g = recs.get("GNSS") or []
    if not g or "num_sats" not in g[0]:
        return None
    sats = get_array(g, "num_sats")
    fixed = sats >= SATS_MIN
    pct = 100.0 * float(np.mean(fixed)) if sats.size else 0.0

    detail = f"{int(np.max(sats))} satellites at best, usable fix for {pct:.0f}% of the log"
    if "pdop" in g[0]:
        pdop = get_array(g, "pdop")
        good = pdop[(pdop > 0) & np.isfinite(pdop)]
        if good.size:
            detail += f" · best PDOP {float(np.min(good)):.1f}"

    if pct < 50:
        return _verdict(PROBLEM, detail + " — recovery coordinates may be unreliable")
    if pct < 85:
        return _verdict(CHECK, detail)
    return _verdict(OK, detail)


def _logging(recs, stats) -> Optional[dict[str, Any]]:
    total = int(stats.get("total_frames", 0) or 0)
    if not total:
        return None
    bad = int(stats.get("bad_crc", 0) or 0)
    detail = f"{total:,} frames recorded, {bad} with a bad checksum"

    # ring_overruns is monotonic since boot and is dominated by the rolling
    # prelaunch window dropping its oldest frame on every push while the ring
    # sits at the pad cap — by design, not data loss. Only the delta after
    # logging activation means live flight frames were discarded, so measure
    # from the second sample onwards, exactly as the log_buffer module does.
    lb = recs.get("LogBufferStats") or []
    inflight_overruns = 0
    if len(lb) > 1 and "ring_overruns" in lb[0]:
        inflight_overruns = max(0, int(lb[-1]["ring_overruns"]) - int(lb[1]["ring_overruns"]))
        if inflight_overruns:
            detail += f" · {inflight_overruns:,} frames dropped in flight"

    # A handful of bad frames in a hundred thousand is flash and radio being
    # ordinary; an in-flight overrun is not.
    if inflight_overruns > 0:
        return _verdict(CHECK, detail + " — the log buffer could not keep up")
    if bad > total * 0.001:
        return _verdict(CHECK, detail + " — more corrupt frames than usual")
    return _verdict(OK, detail)


def _radio(flight) -> Optional[dict[str, Any]]:
    csvs = sorted(flight.bin_path.parent.glob("lora_*.csv"))
    if not csvs:
        return None
    return _verdict(OK, f"telemetry captured in {len(csvs)} ground-station log"
                        f"{'s' if len(csvs) != 1 else ''}")


def analyze(flight: Flight) -> AnalysisResult:
    result = AnalysisResult(name="health", title="Vehicle Health")
    recs = flight.records

    checks = [
        ("Battery", _battery(recs)),
        ("GNSS", _gnss(recs)),
        ("Data logging", _logging(recs, flight.stats)),
        ("Radio link", _radio(flight)),
    ]

    metrics: dict[str, str] = {}
    for name, verdict in checks:
        if verdict is None:
            continue
        metrics[name] = f"{verdict['status']} — {verdict['detail']}"
        if verdict["status"] == PROBLEM:
            result.warnings.append(f"{name}: {verdict['detail']}")
        elif verdict["status"] == CHECK:
            result.warnings.append(f"{name} needs a look: {verdict['detail']}")

    if not metrics:
        result.warnings.append("No health telemetry in this log.")
    result.metrics = metrics
    return result
