#!/usr/bin/env python3
"""No default-partition NVS open may run ahead of nvs_flash_init() in a boot
function. Statically enforced, because nothing else can catch it.

THE BUG SHAPE. Every project's boot function opens Preferences namespaces on
the default `nvs` partition, and every one of them calls nvs_flash_init() for
that partition somewhere in the same function. Order matters and nothing
checks it: an open that runs first fails with ESP_ERR_NVS_NOT_INITIALIZED,
the wrapper logs one line and returns false, every read hands back the
caller's default, every write is dropped — and the firmware keeps booting.

That is not hypothetical. It has now shipped twice:

  * The base station had no nvs_flash_init() at all (#500, fixed in #521):
    every setting read back as its default and the PHY recalibrated on every
    boot, silently.
  * The out computer's #773 board-revision read (PR #1323) sat at the top of
    setup_oc(), two hundred lines above its own nvs_flash_init() — which is
    deliberately late, behind the #825 rail-restore race. The read failed on
    every boot, so a provisioned board reported "unprovisioned" forever and
    the reflash protection the value exists for was inert. Found on the V9
    bench on 2026-09-18, nine days after it merged.

No host test can compile those functions (they are hardware setup in a
monolithic main.cpp), and the wrapper only logs. What a static check CAN
catch is the shape: inside any function that calls nvs_flash_init(), a
`prefs.begin(` or `nvs_open(` that textually precedes the first init call.
Opens of a NAMED partition (nvs_open_from_partition, the #1176 flight-token
store) are deliberately not matched — they have their own init and are
allowed, and needed, ahead of the default one.

LIMITS, stated so nobody over-trusts it: the scan is per function body and
textual. An open buried in a helper called before the init is not seen, and
only the identifier `prefs` (the name every project uses for its global
Preferences) and the raw nvs_open() are matched. It catches the two shapes
that actually shipped and the obvious re-hoist of a block; it is not a proof.

Run from anywhere; paths resolve relative to this file. Pass source files as
arguments to check those instead of the built-in list.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

PROJECTS = Path(__file__).resolve().parent.parent / "projects"

# Every translation unit in the repo that calls nvs_flash_init(). A file that
# opens the default partition and never initialises it is the #500 shape, so a
# listed file with opens and no init is an error too.
FILES = [
    PROJECTS / "out_computer/main/main.cpp",
    PROJECTS / "flight_computer/main/main.cpp",
    PROJECTS / "base_station/main/main.cpp",
    PROJECTS / "legacy/rocket_computer_mini/main/main.cpp",
    PROJECTS / "legacy/rocket_computer_mini/main/flight.cpp",
]

INIT_RE = re.compile(r"\bnvs_flash_init\s*\(\s*\)")
# Default-partition opens only. nvs_open_from_partition() is a different
# function name, so `\bnvs_open\s*\(` cannot match it.
OPEN_RE = re.compile(r"\b(?:prefs\.begin|nvs_open)\s*\(")
# A top-level function signature: starts at column 0, has a parameter list,
# and is not a statement. Bodies are found by brace-matching from here.
SIG_RE = re.compile(r"^[A-Za-z_][\w:<>,*&\s]*?\b([A-Za-z_]\w*)\s*\([^;{}]*\)\s*(?:const\s*)?\{?\s*$")


def strip_comments(src: str) -> str:
    """Blank out // and /* */ comments, keeping every newline so offsets and
    line numbers survive. Comments talk about nvs_flash_init() a lot."""
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
            # Skip string literals so a quoted "nvs_flash_init()" in a log
            # message is not mistaken for the call.
            j = i + 1
            while j < n and src[j] != '"':
                j += 2 if src[j] == "\\" else 1
            out.append('"' + " " * (j - i - 1) + '"')
            i = j + 1
        else:
            out.append(src[i])
            i += 1
    return "".join(out)


def function_bodies(src: str):
    """Yield (name, body_start, body_end) for every top-level function."""
    lines = src.split("\n")
    pos = 0
    offsets = []
    for line in lines:
        offsets.append(pos)
        pos += len(line) + 1
    i = 0
    while i < len(lines):
        m = SIG_RE.match(lines[i])
        if not m:
            i += 1
            continue
        name = m.group(1)
        brace = src.find("{", offsets[i])
        # The signature must be followed by its opening brace before any
        # statement terminator, or it was a declaration / macro line.
        semi = src.find(";", offsets[i])
        if brace < 0 or (0 <= semi < brace):
            i += 1
            continue
        depth = 0
        end = -1
        for j in range(brace, len(src)):
            if src[j] == "{":
                depth += 1
            elif src[j] == "}":
                depth -= 1
                if depth == 0:
                    end = j + 1
                    break
        if end < 0:
            sys.exit(f"unbalanced braces after {name}() -- cannot scan")
        yield name, brace, end
        # Resume after the body.
        while i < len(lines) and offsets[i] < end:
            i += 1


def line_of(src: str, offset: int) -> int:
    return src.count("\n", 0, offset) + 1


def check_file(path: Path) -> list[str]:
    raw = path.read_text(encoding="utf-8", errors="replace")
    src = strip_comments(raw)
    rel = path.relative_to(PROJECTS.parent.parent) if path.is_relative_to(PROJECTS.parent.parent) else path
    errors: list[str] = []
    inits_seen = 0
    checked = []
    for name, start, end in function_bodies(src):
        body = src[start:end]
        init = INIT_RE.search(body)
        if not init:
            continue
        inits_seen += 1
        early = [m for m in OPEN_RE.finditer(body) if m.start() < init.start()]
        for m in early:
            errors.append(
                f"{rel}:{line_of(src, start + m.start())}: {name}() opens the "
                f"default NVS partition here, but its nvs_flash_init() is not "
                f"until line {line_of(src, start + init.start())} -- this open "
                f"fails with ESP_ERR_NVS_NOT_INITIALIZED on every boot"
            )
        checked.append(f"{name}():{line_of(src, start + init.start())}")
    if inits_seen == 0 and OPEN_RE.search(src):
        errors.append(
            f"{rel}: opens the default NVS partition but never calls "
            f"nvs_flash_init() -- the #500 base-station shape"
        )
    if not errors:
        print(f"ok   {rel}: init before every open in {', '.join(checked) or 'no boot function'}")
    return errors


def main(argv: list[str]) -> int:
    files = [Path(a).resolve() for a in argv[1:]] or FILES
    errors: list[str] = []
    for f in files:
        if not f.exists():
            errors.append(f"{f}: missing -- has it moved? Update FILES in {Path(__file__).name}")
            continue
        errors.extend(check_file(f))
    if errors:
        print()
        for e in errors:
            print(f"FAIL {e}")
        print(
            "\nFix: move the open below nvs_flash_init() (the init is late on "
            "purpose where a rail decision precedes it -- do not hoist the "
            "init instead without reading the comments above it)."
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
