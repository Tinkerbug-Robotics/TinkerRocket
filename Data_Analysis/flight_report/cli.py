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

from .discover import DEFAULT_DISCOVERY_ROOT, discover, filter_flights
from .flight import Flight
from .registry import LEVEL_ENGINEERING, LEVEL_FLIGHT, modules_for, run_module
from .render import write_report


_LEVEL_SUFFIX = {LEVEL_FLIGHT: "_report.html", LEVEL_ENGINEERING: "_report_engineering.html"}


def _out_path_for(flight: Flight, out: Path | None, level: str) -> Path:
    suffix = _LEVEL_SUFFIX[level]
    if out is None:
        return flight.bin_path.with_name(f"{flight.bin_path.stem}{suffix}")
    if out.is_dir() or (not out.exists() and out.suffix == ""):
        return out / f"{flight.bin_path.stem}{suffix}"
    # An explicit filename names the first report; the other gets a sibling.
    if level == LEVEL_FLIGHT:
        return out
    return out.with_name(f"{out.stem}_engineering{out.suffix or '.html'}")


def _process_one(flight: Flight, out: Path | None, levels: list[str]) -> list[Path]:
    """Run the requested levels against one flight. Returns the report paths."""
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
        # Cross-link the two reports, but only when both are being written —
        # a link to a file that was never generated is worse than no link.
        other = next((lv for lv in levels if lv != level), None)
        counterpart = _out_path_for(flight, out, other).name if other else None
        write_report(flight, results, out_path, level=level, counterpart=counterpart)
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
    p_run.add_argument(
        "--level",
        choices=[LEVEL_FLIGHT, LEVEL_ENGINEERING, "both"],
        default="both",
        help=(
            "Which report(s) to write. 'flight' is the headline read — summary "
            "card and interactive charts; 'engineering' is the full diagnostic "
            "set. Default: both."
        ),
    )

    p_list = sub.add_parser("list", help="Discover flights without running analysis.")
    p_list.add_argument("path", nargs="?", default=None)

    args = p.parse_args(argv)

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

    levels = [LEVEL_FLIGHT, LEVEL_ENGINEERING] if args.level == "both" else [args.level]

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
