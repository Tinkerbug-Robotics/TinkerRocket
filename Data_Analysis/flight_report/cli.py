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
from .registry import MODULES, run_module
from .render import write_report


def _process_one(flight: Flight, out: Path | None) -> Path:
    """Run all modules against one flight and write the report. Returns report path."""
    print(f"  Parsing: {flight.bin_path}")
    t0 = time.time()
    flight.load()
    print(f"    parsed in {time.time()-t0:.1f}s — {flight.stats.get('total_frames', 0):,} frames")

    results = []
    for name, fn in MODULES:
        m_t0 = time.time()
        result = run_module(name, fn, flight)
        elapsed = time.time() - m_t0
        marker = "ERR" if result.error else ("WARN" if result.warnings else "OK ")
        print(f"    [{marker}] {name:<20s} ({elapsed:.1f}s, {len(result.figures)} figs)")
        results.append(result)

    if out is None:
        out_path = flight.bin_path.with_name(f"{flight.bin_path.stem}_report.html")
    elif out.is_dir() or (not out.exists() and out.suffix == ""):
        out_path = out / f"{flight.bin_path.stem}_report.html"
    else:
        out_path = out

    write_report(flight, results, out_path)
    print(f"    -> {out_path}")
    return out_path


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

    print(f"Processing {len(flights)} flight(s)...")
    failed = 0
    for f in flights:
        print(f"\n[{f.name}]")
        try:
            _process_one(f, args.out)
        except Exception as e:  # noqa: BLE001
            print(f"  FAILED: {type(e).__name__}: {e}", file=sys.stderr)
            failed += 1

    print(f"\nDone. {len(flights) - failed} succeeded, {failed} failed.")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
