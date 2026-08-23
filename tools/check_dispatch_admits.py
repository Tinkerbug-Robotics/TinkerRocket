#!/usr/bin/env python3
"""
Every OC<->FC message type the out computer HANDLES must also be ADMITTED.

`processFrame()` in projects/out_computer/main/main.cpp opens with

    if (!isKnownMessageType(type)) return;

a deliberate guard so CRC false positives from I2S noise cannot reach a
handler.  That means a new message type has to be added in TWO places: the
`type == X` branch that handles it, and the `case X:` in isKnownMessageType().
Add only the handler and the frame is dropped one line before it -- the handler
becomes dead code that no test notices.

That is not hypothetical.  FC_BOOT_STATUS_MSG (0xFA, FC boot progress) shipped
with a handler and no whitelist entry on 2026-08-19.  Every host test passed:
the gtest registry in tests_cpp/test_rocket_computer_types.cpp checks that
message codes are UNIQUE, and tools/check_ble_command_ids.py checks BLE command
numbers -- neither checks membership in this whitelist.  It was only caught by
watching the app report "waiting for flight computer" on the bench, which is a
slow and lucky way to find a two-list bug.

The reverse direction (admitted but never handled) is reported as INFO only:
some codes are legitimately admitted so they reach the logger via
enqueueFrame() without a dedicated branch.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OC = ROOT / "tinkerrocket-idf/projects/out_computer/main/main.cpp"


def known_message_types(src: str) -> set[str]:
    """The `case X:` labels inside isKnownMessageType()'s switch."""
    m = re.search(r"bool\s+isKnownMessageType\s*\([^)]*\)\s*\{", src)
    if not m:
        sys.exit("could not find isKnownMessageType() -- has it been renamed?")
    # Walk to the end of the function by brace depth.
    i = src.index("{", m.start())
    depth = 0
    for j in range(i, len(src)):
        if src[j] == "{":
            depth += 1
        elif src[j] == "}":
            depth -= 1
            if depth == 0:
                body = src[i : j + 1]
                break
    else:
        sys.exit("isKnownMessageType() has unbalanced braces")
    return set(re.findall(r"case\s+([A-Z][A-Z0-9_]+)\s*:", body))


def handled_message_types(src: str) -> dict[str, int]:
    """`type == X` branches, mapped to their 1-based line number."""
    out: dict[str, int] = {}
    for n, line in enumerate(src.splitlines(), 1):
        for name in re.findall(r"\btype\s*==\s*([A-Z][A-Z0-9_]+)\b", line):
            out.setdefault(name, n)
    return out


def main() -> int:
    src = OC.read_text()
    admitted = known_message_types(src)
    handled = handled_message_types(src)

    print("=== OC dispatch admission guard ===")
    print(f"  isKnownMessageType(): {len(admitted)} admitted")
    print(f"  processFrame():       {len(handled)} handled")

    missing = sorted(set(handled) - admitted)
    for name in missing:
        print(
            f"  DEAD HANDLER: {name} is handled at "
            f"{OC.relative_to(ROOT)}:{handled[name]} but is NOT a case in "
            f"isKnownMessageType() -- processFrame() drops the frame before "
            f"reaching it."
        )

    unhandled = sorted(admitted - set(handled))
    if unhandled:
        print(
            f"  info: {len(unhandled)} admitted without a dedicated branch "
            f"(they still reach the logger): {', '.join(unhandled)}"
        )

    if missing:
        print(
            f"\nRESULT: FAIL -- {len(missing)} handler(s) can never run. Add the "
            f"matching `case X:` to isKnownMessageType()."
        )
        return 1
    print("\nRESULT: PASS -- every handled message type is admitted.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
