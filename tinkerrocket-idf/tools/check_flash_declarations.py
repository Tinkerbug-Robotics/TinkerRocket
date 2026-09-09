#!/usr/bin/env python3
"""#916: check every board's declared boot-NOR size, statically.

Two failure modes, opposite directions and very different consequences:

  UNDER-declared (smaller than the partition table needs) — IDF's own
  gen_esp32part.py catches this at build time, but only for configurations CI
  actually builds, and only once the table is generated.

  OVER-declared (larger than the part fitted) — the bootloader reads a flash
  size the chip does not have and the board boot-loops before app_main, so no
  runtime assertion can catch it. This is what ec2a0728 fixed by forking the
  declaration per board revision. A static check cannot know what part is
  fitted either, but it CAN catch the tell that preceded the last occurrence:
  two processors on one board disagreeing about the same board's flash.

So this asserts:
  1. every project x board overlay declares at least what its own partition
     table ends at, and
  2. the processor pairs that share a physical board declare the same size.

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
        print(f"{label:26s} {size_mb:8d}M {end:#11x}  {table}")
        if end > size_mb * 1024 * 1024:
            failures.append(
                f"{label}: partition table ends at {end:#x} but only "
                f"{size_mb} MB is declared — the image will not fit"
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
    print("\nOK — every declaration covers its own table, and paired processors agree")
    return 0


if __name__ == "__main__":
    sys.exit(main())
