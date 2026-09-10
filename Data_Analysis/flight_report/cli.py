"""Command-line entry point for the flight-analysis suite."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # render off-screen; no display required
import matplotlib.pyplot as plt
plt.rcParams["figure.max_open_warning"] = 0  # we batch ~30 figures intentionally

from . import catalog
from .discover import DEFAULT_DISCOVERY_ROOT, discover, filter_flights
from .flight import Flight
from .registry import LEVEL_FLIGHT, modules_for, run_module
from .render import write_report


_LEVEL_SUFFIX = {LEVEL_FLIGHT: "_report.html"}


def _out_path_for(flight: Flight, out: Path | None, level: str) -> Path:
    suffix = _LEVEL_SUFFIX[level]
    if out is None:
        return flight.bin_path.with_name(f"{flight.bin_path.stem}{suffix}")
    if out.is_dir() or (not out.exists() and out.suffix == ""):
        return out / f"{flight.bin_path.stem}{suffix}"
    # An explicit filename names the first report; the other gets a sibling.
    if level == LEVEL_FLIGHT:
        return out
    return out.with_name(f"{out.stem}_detailed{out.suffix or '.html'}")


def _process_one(flight: Flight, out: Path | None, levels: list[str]) -> list[Path]:
    """Run one flight. `levels` is a list for historical reasons — there is one."""
    print(f"  Parsing: {flight.bin_path}")
    t0 = time.time()
    flight.load()
    print(f"    parsed in {time.time()-t0:.1f}s — {flight.stats.get('total_frames', 0):,} frames")

    written = []
    for level in levels:
        print(f"    [{level} report]")
        results = []
        for name, fn, _lvl in modules_for(level):
            m_t0 = time.time()
            result = run_module(name, fn, flight)
            elapsed = time.time() - m_t0
            marker = "ERR" if result.error else ("WARN" if result.warnings else "OK ")
            made = f"{len(result.figures)} figs"
            if result.charts:
                made += f", {len(result.charts)} charts"
            print(f"      [{marker}] {name:<20s} ({elapsed:.1f}s, {made})")
            results.append(result)

        out_path = _out_path_for(flight, out, level)
        write_report(flight, results, out_path, level=level)
        print(f"      -> {out_path}")
        written.append(out_path)

    return written


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="flight_report", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    p_run = sub.add_parser("run", help="Run analysis on a flight (or directory of flights).")
    p_run.add_argument(
        "path",
        nargs="?",
        default=None,
        help=f"File or directory. Default: {DEFAULT_DISCOVERY_ROOT}",
    )
    p_run.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output file (single flight) or directory. Default: next to the .bin.",
    )
    p_run.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Stop after N flights (useful for testing).",
    )
    p_list = sub.add_parser("list", help="Discover flights without running analysis.")
    p_list.add_argument("path", nargs="?", default=None)

    # #752: what is actually IN one log, built by walking its parsed records
    # rather than from a static schema — so it can never advertise a channel this
    # firmware version did not write. Greps, and pastes into an issue.
    p_fields = sub.add_parser(
        "fields", help="List every channel in one flight, with provenance.")
    p_fields.add_argument("path", help="A flight_*.bin file.")
    p_fields.add_argument("--json", action="store_true",
                          help="Machine-readable output instead of the text block.")

    args = p.parse_args(argv)

    if args.cmd == "fields":
        bin_path = Path(args.path)
        if not bin_path.is_file():
            print(f"Not a file: {bin_path}", file=sys.stderr)
            return 1
        flight = Flight.from_bin(bin_path)
        flight.load()
        cat = catalog.build(flight)
        if args.json:
            import json
            print(json.dumps({
                "flight": str(bin_path),
                "channels": [
                    {
                        "stream": c.stream, "field": c.field, "label": c.label,
                        "unit": c.meta.unit, "kind": c.meta.kind,
                        "documented": c.documented, "empty": c.is_empty,
                        "n_present": c.n_present, "n_total": c.n_total,
                        "rate_hz": c.rate_hz,
                        "t_start_s": c.t_start_s, "t_end_s": c.t_end_s,
                        "vmin": c.vmin, "vmax": c.vmax,
                        "shown_in": list(c.meta.shown_in),
                        "derived_in": list(c.meta.derived_in),
                        "note": c.meta.note, "caution": c.meta.caution,
                    }
                    for c in cat.channels
                ],
                "unreadable": [
                    {"name": u.name, "count": u.count, "reason": u.reason}
                    for u in cat.unreadable
                ],
                "dropped": list(cat.dropped),
            }, indent=2))
        else:
            print(catalog.format_text(cat))
        return 0

    if args.cmd == "list":
        flights = filter_flights(discover(args.path))
        for f in flights:
            print(f"{f.bin_path}")
        print(f"\n{len(flights)} flight(s) found.")
        return 0

    flights = filter_flights(discover(args.path))
    if not flights:
        print(f"No flight_*.bin files found under {args.path or DEFAULT_DISCOVERY_ROOT}",
              file=sys.stderr)
        return 1

    if args.limit:
        flights = flights[: args.limit]

    levels = [LEVEL_FLIGHT]

    print(f"Processing {len(flights)} flight(s)...")
    failed = 0
    for f in flights:
        print(f"\n[{f.name}]")
        try:
            _process_one(f, args.out, levels)
        except Exception as e:  # noqa: BLE001
            print(f"  FAILED: {type(e).__name__}: {e}", file=sys.stderr)
            failed += 1

    print(f"\nDone. {len(flights) - failed} succeeded, {failed} failed.")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
