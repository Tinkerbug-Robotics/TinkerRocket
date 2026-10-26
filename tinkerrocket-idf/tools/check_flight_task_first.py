#!/usr/bin/env python3
"""The flight computer's app_main must create the flight task before any other
task. Statically enforced, because nothing else can catch it.

THE BUG SHAPE. The flight task's 16 KB stack is one block of internal RAM, and
setup_fc() leaves little of it. A task that app_main creates ahead of the
flight task can take the block the flight task needs, and then no flight loop
runs: no NonSensorData, no command service, no flight.

That shipped on a bench image (Beetle, 2026-10-26): the multi-band benchmark
of #1675 (TR_BENCH_MB_BLOCKS) created its 12 KB "mb_bench" task first, which
left 22408 B free with a largest block of 11264 B. The flight task was never
created, and the apps said "Flight computer ready" over a dashboard stuck on
"Syncing…". The FC now reports FCB_NO_FLIGHT_LOOP when that happens; this
check keeps the cause from coming back.

THE RULE. Inside app_main, the task named "flight" is the first one created.
The one exception is "rescue": rescue mode creates it INSTEAD of the flight
task and returns. Anything else goes after the flight task, bench or not.

LIMITS: the scan is textual and covers app_main's own body. A task created in
a helper that app_main calls before the flight task is not seen, and neither
is one setup_fc() creates (those are the design, and their sizes are budgeted
there). Every xTaskCreate* call needs its name as a string literal.

Run from anywhere; paths resolve relative to this file. Pass a source file as
an argument to check it instead of the flight computer's main.cpp.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

FC_MAIN = Path(__file__).resolve().parent.parent / "projects/flight_computer/main/main.cpp"

FLIGHT = "flight"
# Created instead of the flight task, never beside it.
ALLOWED_BEFORE = {"rescue"}

APP_MAIN_RE = re.compile(r'^extern\s+"C"\s+void\s+app_main\s*\(\s*(?:void)?\s*\)', re.M)
CREATE_RE = re.compile(r"\bxTaskCreate\w*\s*\(")
NAME_RE = re.compile(r'"([^"\\]*)"')


def strip_comments(src: str) -> str:
    """Blank out // and /* */ comments, keeping newlines and string literals
    (the task names are string literals)."""
    out = []
    i, n = 0, len(src)
    while i < n:
        two = src[i : i + 2]
        if two == "//":
            j = src.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif two == "/*":
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", src[i:j]))
            i = j
        elif src[i] == '"':
            j = i + 1
            while j < n and src[j] != '"':
                j += 2 if src[j] == "\\" else 1
            out.append(src[i : j + 1])
            i = j + 1
        else:
            out.append(src[i])
            i += 1
    return "".join(out)


def matching(src: str, open_at: int, open_ch: str, close_ch: str) -> int:
    """Offset just past the bracket that closes the one at open_at."""
    depth = 0
    i = open_at
    while i < len(src):
        c = src[i]
        if c == '"':
            i += 1
            while i < len(src) and src[i] != '"':
                i += 2 if src[i] == "\\" else 1
        elif c == open_ch:
            depth += 1
        elif c == close_ch:
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1
    sys.exit(f"unbalanced {open_ch}{close_ch} at offset {open_at} -- cannot scan")


def line_of(src: str, offset: int) -> int:
    return src.count("\n", 0, offset) + 1


def task_creations(body: str):
    """Yield (offset, name) for every xTaskCreate* call in body. The name is
    the call's first string literal -- the task name, since the entry point
    and everything before it are identifiers or a lambda without strings."""
    for m in CREATE_RE.finditer(body):
        args_end = matching(body, m.end() - 1, "(", ")")
        args = body[m.end() : args_end]
        # Skip a lambda entry point's body: its strings are not the name.
        if args.lstrip().startswith("["):
            brace = args.find("{")
            args = args[matching(args, brace, "{", "}") :]
        name = NAME_RE.search(args)
        yield m.start(), (name.group(1) if name else None)


def check(path: Path) -> list[str]:
    src = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
    m = APP_MAIN_RE.search(src)
    if not m:
        return [f"{path}: no app_main() found"]
    start = src.find("{", m.end())
    body = src[start : matching(src, start, "{", "}")]
    errors: list[str] = []
    order = []
    flight_seen = False
    for off, name in task_creations(body):
        where = f"{path.name}:{line_of(src, start + off)}"
        if name is None:
            errors.append(f"{where}: a task created with no string-literal name -- cannot check its order")
            continue
        order.append(name)
        if name == FLIGHT:
            flight_seen = True
        elif not flight_seen and name not in ALLOWED_BEFORE:
            errors.append(
                f'{where}: app_main creates "{name}" before the flight task -- its stack can '
                f"take the internal RAM the flight task's 16 KB stack needs, and then no flight loop runs"
            )
    if not flight_seen:
        errors.append(f'{path.name}: app_main creates no task named "{FLIGHT}" -- has it been renamed?')
    if not errors:
        print(f"ok   {path.name}: app_main creates {', '.join(order)}")
    return errors


def main(argv: list[str]) -> int:
    path = Path(argv[1]).resolve() if len(argv) > 1 else FC_MAIN
    if not path.exists():
        print(f"FAIL {path}: missing -- has it moved? Update FC_MAIN in {Path(__file__).name}")
        return 1
    errors = check(path)
    for e in errors:
        print(f"FAIL {e}")
    if errors:
        print('\nFix: create the task after the flight task (see the TR_BENCH_MB_BLOCKS block in app_main).')
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
