#!/usr/bin/env python3
"""#916: enforce the boot-NOR declaration policy, statically.

POLICY (owner decision, 2026-09-09): every build declares the SMALLEST standard
flash size that fits its own partition table — not the size of the part fitted
to the board.

Why that way round. Over-declaring is the only direction that bricks: the
bootloader reads a size the chip does not have and the board loops before
app_main, where no runtime assertion can reach it. Under-declaring is harmless;
it just leaves the top of the part unaddressable. Sizing to the TABLE rather
than to the PART means an image is safe on any board carrying at least that
much flash, which removed the per-revision fork entirely for flight_computer
and out_computer — they share one partition table across V7/V8/V9/M1, so the
16 MB overlays were buying nothing at all.

That fork is what went wrong twice: the original V7 boot loop (ec2a0728), and
rocket-computer-mini's two processors declaring different sizes for one board.

So this asserts, for every project x board overlay:
  1. the declaration covers its own partition table, and
  2. it is the SMALLEST standard size that does — no headroom, because headroom
     is the failure direction, and
  3. processors sharing a physical board agree.

If a partition table ever grows past its current size, this fails and the fix
is to raise that project's declaration to the next standard size. That is the
intended workflow, not a bypass.

Run from anywhere; paths are resolved relative to this file.
"""
from __future__ import annotations

import csv
import re
import sys
from pathlib import Path

PROJECTS = Path(__file__).resolve().parent.parent / "projects"

SIZE_RE = re.compile(r"^CONFIG_ESPTOOLPY_FLASHSIZE_(\d+)MB=y\s*$", re.M)
TABLE_RE = re.compile(r'^CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="([^"]+)"\s*$', re.M)

# (label, project, [sdkconfig files applied in order])
# Mirrors each project's CMakeLists overlay selection; keep in step with it.
BUILDS = [
    ("flight_computer V7/V8", "flight_computer", ["sdkconfig.defaults"]),
    ("flight_computer V9", "flight_computer", ["sdkconfig.defaults", "sdkconfig.defaults.v9"]),
    ("flight_computer M1", "flight_computer", ["sdkconfig.defaults", "sdkconfig.defaults.m1"]),
    ("out_computer V8", "out_computer", ["sdkconfig.defaults"]),
    ("out_computer V9", "out_computer", ["sdkconfig.defaults", "sdkconfig.defaults.v9"]),
    ("out_computer M1", "out_computer", ["sdkconfig.defaults", "sdkconfig.defaults.v9"]),
    ("base_station V1/V2", "base_station", ["sdkconfig.defaults"]),
    ("base_station V3", "base_station", ["sdkconfig.defaults", "sdkconfig.defaults.v3"]),
    ("radio_board", "radio_board", ["sdkconfig.defaults"]),
    ("rocket_computer_mini", "rocket_computer_mini", ["sdkconfig.defaults"]),
]

# Physical boards carrying two processors that boot from their own NOR parts.
# rocket-computer-mini's BOM line 59 fits TWO W25Q128JVYIQ (U13 and U33), so
# both are 16 MB and the two declarations must agree.
BOARD_PAIRS = [("rocket-computer-mini", "flight_computer M1", "out_computer M1")]

# The sizes esptool/IDF can declare. The policy picks the smallest that fits.
STANDARD_MB = (1, 2, 4, 8, 16, 32, 64)


def smallest_fitting_mb(end_bytes: int) -> int | None:
    for mb in STANDARD_MB:
        if mb * 1024 * 1024 >= end_bytes:
            return mb
    return None


def resolve(project: str, files: list[str]) -> tuple[int | None, str | None]:
    """Last declaration wins, exactly as IDF applies SDKCONFIG_DEFAULTS."""
    size_mb, table = None, None
    for name in files:
        path = PROJECTS / project / name
        if not path.exists():
            continue
        text = path.read_text()
        for m in SIZE_RE.finditer(text):
            size_mb = int(m.group(1))
        for m in TABLE_RE.finditer(text):
            table = m.group(1)
    return size_mb, table


def table_end(project: str, table: str) -> int:
    """Highest offset+size in the CSV, in bytes."""
    path = PROJECTS / project / table
    end = 0
    with path.open() as fh:
        for row in csv.reader(fh):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) < 5:
                continue
            try:
                off = int(row[3].strip(), 0)
                size = int(row[4].strip(), 0)
            except ValueError:
                continue
            end = max(end, off + size)
    return end


def main() -> int:
    failures: list[str] = []
    declared: dict[str, int] = {}

    print(f"{'build':26s} {'declared':>9s} {'table ends':>11s}  table")
    for label, project, files in BUILDS:
        size_mb, table = resolve(project, files)
        if size_mb is None:
            failures.append(f"{label}: no CONFIG_ESPTOOLPY_FLASHSIZE_*MB anywhere")
            continue
        declared[label] = size_mb
        if table is None:
            print(f"{label:26s} {size_mb:8d}M {'(built-in)':>11s}")
            continue
        end = table_end(project, table)
        want = smallest_fitting_mb(end)
        flag = "" if want == size_mb else f"   <-- policy says {want}M"
        print(f"{label:26s} {size_mb:8d}M {end:#11x}  {table}{flag}")
        if want is None:
            failures.append(f"{label}: table ends at {end:#x}, beyond any standard flash size")
        elif size_mb < want:
            failures.append(
                f"{label}: partition table ends at {end:#x} but only "
                f"{size_mb} MB is declared — the image will not fit. "
                f"Raise it to {want} MB."
            )
        elif size_mb > want:
            failures.append(
                f"{label}: declares {size_mb} MB for a table ending at {end:#x}, "
                f"which needs only {want} MB. Headroom is the direction that "
                f"boot-loops a board carrying a smaller part — declare {want} MB. "
                f"If the table is meant to grow, grow it first."
            )

    for board, a, b in BOARD_PAIRS:
        if a in declared and b in declared and declared[a] != declared[b]:
            failures.append(
                f"{board}: {a} declares {declared[a]} MB but {b} declares "
                f"{declared[b]} MB — one physical board, two processors, and "
                f"they disagree about the part fitted to it (#916)"
            )

    if failures:
        print("\nFAIL", file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1
    print("\nOK — every declaration is the smallest standard size fitting its own\n"
          "     partition table, and paired processors agree")
    return 0


if __name__ == "__main__":
    sys.exit(main())
