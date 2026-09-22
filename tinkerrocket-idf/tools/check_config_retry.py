#!/usr/bin/env python3
"""Every flight-computer command handler that reads a config frame must retry
on a miss. Statically enforced, because the gap has shipped twice and the
comment that said it was closed was wrong.

THE BUG SHAPE. The FC executes each OC command once per serving window: the
dedup key (OcCmdDedup in flight_computer/main/oc_cmd_dedup.h) takes the command
on its first delivery. When readConfigFrame() finds no frame in that poll's
read, the handler has to call cfgRetryOnNextPoll(), which re-arms the key for
the next poll so the OC's remaining repeat deliveries, each re-staging the
frame, get another attempt (#1112). A handler without that call keeps the
command in the key, the repeats are skipped as duplicates, and the setting is
lost for good. Where the app's readback is the OC's cache (IMU rate, roll
control), nothing even shows it.

That is not hypothetical:

  * #1117 (2026-09-07): the pyro handler had no else, so a dropped frame left
    the FC flying on whatever its NVS held, which on a fresh board is all four
    deployment channels disabled. That fix called pyro "the only
    config-pending handler with no else".
  * It was not. On 2026-09-22 orientation, IMU rate and camera type were found
    with no else either, and six more (both calibration applies, roll control,
    guidance config, guidance point, fin layout) with an else that only
    logged.

No host test can reach these handlers: they are branches of the main loop in a
monolithic main.cpp. What a static check CAN see is the shape. Every
`if (readConfigFrame(...)) {...}` must be followed by an `else {...}` that
calls cfgRetryOnNextPoll().

EXCEPTIONS. DELIBERATE lists the commands that must not retry, each with the
reason. KNOWN_GAPS lists a handler whose fix is being done in other work. An
entry that stops matching (the handler now retries, or is gone) is an error
too, so neither list can outlive the reason it gives.

LIMITS. The check is textual and covers this one file. It sees the if/else
around each call, attributes the call to the nearest preceding
`if (out_pending_command == X`, and fails on any call it cannot parse that way
rather than guessing. A retry hidden in a helper called from the else is not
seen, and none exists today. The single-MCU rocket_computer_mini is not
scanned: its command queue is exactly-once and has no dedup key. The M1 mini's
flight computer builds this same file.

Run from anywhere; paths resolve relative to this file. Pass a source file as
an argument to check it instead of the flight computer's main.cpp.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
FC_MAIN = REPO / "tinkerrocket-idf/projects/flight_computer/main/main.cpp"

# Commands that read a frame and deliberately do not retry. None of them is a
# setting: nothing is cached on the OC, and the operator sees the miss.
DELIBERATE = {
    "OTA_BEGIN_PENDING": (
        "a failed header read is reported to the relay as VERIFY_FAILED, so "
        "the app shows the OTA failing and the operator starts it again"
    ),
    "PYRO_CONT_TEST": (
        "a one-shot ground test, not a setting: a miss reads no channel and "
        "the operator taps again"
    ),
    "PYRO_FIRE_TEST": (
        "a one-shot ground test: a retry would fire the channel on a later "
        "delivery, at a moment the operator did not choose; a miss fires "
        "nothing"
    ),
}

# Real gaps whose fix is being done in other work. Delete the entry when that
# fix lands; the check fails until you do.
KNOWN_GAPS = {
    "CAMERA_CONFIG_PENDING": (
        "being fixed with the camera-type readback work (found 2026-09-22)"
    ),
}

CALL_RE = re.compile(r"\breadConfigFrame\s*\(")
IF_OPEN_RE = re.compile(r"\bif\s*\(\s*$")
HANDLER_RE = re.compile(r"\bif\s*\(\s*out_pending_command\s*==\s*([A-Z][A-Z0-9_]*)")
RETRY_RE = re.compile(r"\bcfgRetryOnNextPoll\s*\(")


def strip_code(src: str) -> str:
    """Blank out comments and string/char literals, keeping every newline so
    offsets and line numbers survive. Comments and log strings mention
    readConfigFrame() and contain braces."""
    out = []
    i, n = 0, len(src)
    while i < n:
        two = src[i : i + 2]
        c = src[i]
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
        elif c == '"' or (c == "'" and not (i > 0 and src[i - 1].isalnum())):
            # A ' after a digit is a separator (60'000'000), not a literal.
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            out.append(c + re.sub(r"[^\n]", " ", src[i + 1 : j]) + c)
            i = j + 1
        else:
            out.append(c)
            i += 1
    return "".join(out)


def matching(code: str, start: int) -> int:
    """Index of the bracket that closes the one at `start`."""
    opener = code[start]
    closer = {"(": ")", "{": "}"}[opener]
    depth = 0
    for j in range(start, len(code)):
        if code[j] == opener:
            depth += 1
        elif code[j] == closer:
            depth -= 1
            if depth == 0:
                return j
    return -1


def skip_ws(code: str, i: int) -> int:
    while i < len(code) and code[i].isspace():
        i += 1
    return i


def line_of(code: str, offset: int) -> int:
    return code.count("\n", 0, offset) + 1


def scan(code: str):
    """Yield (handler, line, retries, problem) for each readConfigFrame call.
    `problem` is set when the call's shape could not be parsed."""
    for m in CALL_RE.finditer(code):
        line = line_of(code, m.start())
        if re.search(r"\bbool\s*$", code[: m.start()]):
            continue  # the definition
        handlers = HANDLER_RE.findall(code, 0, m.start())
        handler = handlers[-1] if handlers else "?"
        if_open = IF_OPEN_RE.search(code, 0, m.start())
        if not if_open:
            yield handler, line, False, "not the `if (readConfigFrame(...))` shape"
            continue
        cond_end = matching(code, code.index("(", if_open.start()))
        k = skip_ws(code, cond_end + 1) if cond_end >= 0 else len(code)
        then_end = matching(code, k) if k < len(code) and code[k] == "{" else -1
        if then_end < 0:
            yield handler, line, False, "the success branch is not a braced block"
            continue
        k = skip_ws(code, then_end + 1)
        if not re.match(r"else\b", code[k:]):
            yield handler, line, False, None  # no else at all
            continue
        k = skip_ws(code, k + len("else"))
        else_end = matching(code, k) if k < len(code) and code[k] == "{" else -1
        if else_end < 0:
            yield handler, line, False, "the else is not a braced block"
            continue
        yield handler, line, bool(RETRY_RE.search(code, k, else_end)), None


def main(argv: list[str]) -> int:
    path = Path(argv[1]).resolve() if len(argv) > 1 else FC_MAIN
    if not path.exists():
        print(f"FAIL {path}: missing -- has it moved? Update FC_MAIN in {Path(__file__).name}")
        return 1
    rel = path.relative_to(REPO) if path.is_relative_to(REPO) else path
    code = strip_code(path.read_text(encoding="utf-8", errors="replace"))

    errors: list[str] = []
    retrying: list[str] = []
    missing: dict[str, int] = {}  # handler -> line, for the calls with no retry
    for handler, line, retries, problem in scan(code):
        if problem:
            errors.append(
                f"{rel}:{line}: {handler}: {problem} -- the check cannot tell "
                f"whether a missing frame is retried; restructure it or "
                f"extend {Path(__file__).name}"
            )
        elif retries:
            retrying.append(handler)
        else:
            missing.setdefault(handler, line)

    for handler, line in missing.items():
        if handler in DELIBERATE or handler in KNOWN_GAPS:
            continue
        errors.append(
            f"{rel}:{line}: {handler} reads a config frame but does not call "
            f"cfgRetryOnNextPoll() when the frame is missing. The dedup key "
            f"keeps the command, the OC's remaining repeats are skipped, and "
            f"the setting is lost for good. Add "
            f'`else {{ cfgRetryOnNextPoll("..."); }}` like its siblings, or '
            f"list it in DELIBERATE with the reason it must not retry."
        )
    for name, table in (("DELIBERATE", DELIBERATE), ("KNOWN_GAPS", KNOWN_GAPS)):
        for handler in table:
            if handler not in missing:
                state = "now retries" if handler in retrying else "reads no config frame"
                errors.append(
                    f"{Path(__file__).name}: {name} lists {handler}, which "
                    f"{state} in {rel} -- delete the entry"
                )

    if errors:
        for e in errors:
            print(f"FAIL {e}")
        return 1
    print(
        f"ok   {rel}: {len(retrying)} config-frame reads retry on a miss; "
        f"deliberately not: {', '.join(sorted(DELIBERATE))}; "
        f"known gap: {', '.join(sorted(KNOWN_GAPS)) or 'none'}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
