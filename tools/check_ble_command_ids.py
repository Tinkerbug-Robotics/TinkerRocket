#!/usr/bin/env python3
"""
Wire-code uniqueness guard: app<->device BLE command numbers (#132 / #148).

Both the Out Computer and the Base Station dispatch an incoming BLE command
byte with a flat `if (ble_cmd == N) ... else if (ble_cmd == M) ...` chain.
The chain takes the FIRST matching branch, so two branches sharing a value
silently turn the later one into dead code -- no compiler error, no link
error, no test failure.  #132/#148 hit exactly this: ble_cmd 56/57/58 were
each assigned to two features, and the second handler never ran.

This script makes that class of bug a hard CI failure.  Within each device's
dispatch, every `ble_cmd == <value>` must be unique.  Right-hand sides that
are named constants (e.g. `LORA_CMD_SET_HOP_DISABLED`) are resolved to their
numeric value from RocketComputerTypes.h, so a bare literal can't silently
collide with a named one.

Why a script and not a C++ unit test (the I2C message types ARE a gtest):
the dispatch lives in `projects/*/main/main.cpp`, which pulls in app_main,
FreeRTOS and the full IDF -- not host-linkable -- and the cases are if/else
branches, not constants.  Grepping the chain is the maintainable option.

Why per-device and not one global space: the app talks to EITHER an OC or a
BS, so the two command spaces are independent.  cmd 50 is "mag-cal start" to
the OC but "relay to rocket" to the BS; that overlap is correct.  The Swift
app therefore mixes both spaces and is reported for information only -- the
firmware dispatch is the authority where a duplicate actually causes a bug.

Usage (runs from anywhere):
    python3 tools/check_ble_command_ids.py
Exit 0 = OK, exit 1 = duplicate (or unresolved RHS) found.
"""

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

HEADER = REPO / "tinkerrocket-idf/components/TR_RocketComputerTypes/RocketComputerTypes.h"
DISPATCHES = [
    ("Out Computer", REPO / "tinkerrocket-idf/projects/out_computer/main/main.cpp"),
    ("Base Station", REPO / "tinkerrocket-idf/projects/base_station/main/main.cpp"),
]
SWIFT = REPO / "TinkerRocketApp/TinkerRocketApp/Models/BLEDevice.swift"

# `static constexpr uint8_t NAME = 0xNN;`  /  `= 17;`
CONST_RE = re.compile(
    r"static\s+constexpr\s+uint8_t\s+(\w+)\s*=\s*(0[xX][0-9A-Fa-f]+|\d+)\s*;"
)
# RHS of `ble_cmd ==` is either a numeric literal or an identifier.
BLE_CMD_RE = re.compile(r"ble_cmd\s*==\s*(0[xX][0-9A-Fa-f]+|\d+|[A-Za-z_]\w*)")
# Swift senders: sendCommand(N) / sendRawCommand(N, ...)
SWIFT_RE = re.compile(r"send(?:Raw)?Command\(\s*(\d+)")

NUMERIC_RE = re.compile(r"\A(?:0[xX][0-9A-Fa-f]+|\d+)\Z")


def load_constants(path):
    """Map every `static constexpr uint8_t` name -> int value in the header."""
    consts = {}
    if not path.exists():
        return consts
    for line in path.read_text().splitlines():
        m = CONST_RE.search(line)
        if m:
            consts[m.group(1)] = int(m.group(2), 0)
    return consts


def resolve(token, consts):
    """Return the int value of a `ble_cmd ==` RHS token, or None if unknown."""
    if NUMERIC_RE.match(token):
        return int(token, 0)
    return consts.get(token)


def check_dispatch(name, path, consts):
    """Return (failed: bool, lines: list[str]) for one device's dispatch."""
    out = []
    if not path.exists():
        return True, [f"{name}: MISSING source file {path}"]

    rel = path.relative_to(REPO)
    # value -> list of (lineno, token); also track unresolved tokens.
    by_value = {}
    unresolved = []
    total = 0
    named = {}
    for lineno, line in enumerate(path.read_text().splitlines(), 1):
        for m in BLE_CMD_RE.finditer(line):
            token = m.group(1)
            total += 1
            val = resolve(token, consts)
            if val is None:
                unresolved.append((lineno, token))
                continue
            if not NUMERIC_RE.match(token):
                named[token] = val
            by_value.setdefault(val, []).append((lineno, token))

    failed = False
    out.append(f"{name}  ({rel})")
    out.append(f"  {total} ble_cmd branches, {len(by_value)} distinct values")
    for tok, val in sorted(named.items(), key=lambda kv: kv[1]):
        out.append(f"  resolved named constant: {tok} = {val}")

    dups = {v: hits for v, hits in by_value.items() if len(hits) > 1}
    if dups:
        failed = True
        for val, hits in sorted(dups.items()):
            where = ", ".join(f"line {ln} (`{tok}`)" for ln, tok in hits)
            out.append(
                f"  DUPLICATE: ble_cmd == {val} appears {len(hits)}x -> {where}"
            )
            out.append(
                "            first match wins; the later branch(es) are dead code."
            )
    if unresolved:
        failed = True
        for ln, tok in unresolved:
            out.append(
                f"  UNRESOLVED: ble_cmd == {tok} (line {ln}) is not a literal and "
                f"is not defined in {HEADER.name}; cannot verify uniqueness."
            )
    if not failed:
        out.append("  -> OK (all values unique)")
    return failed, out


def swift_cross_reference(oc_values):
    """Informational only: list the app's command numbers vs the OC dispatch."""
    out = ["App cross-reference (informational, not enforced):"]
    if not SWIFT.exists():
        out.append(f"  (skipped: {SWIFT} not found)")
        return out
    rel = SWIFT.relative_to(REPO)
    nums = sorted({int(m.group(1)) for m in SWIFT_RE.finditer(SWIFT.read_text())})
    out.append(f"  {rel}: {len(nums)} distinct send(Raw)Command numbers")
    not_oc = [n for n in nums if n not in oc_values]
    if not_oc:
        out.append(
            "  not in the OC ble_cmd dispatch (expected: base-station / relay / "
            "OTA / out-of-band handlers) -> " + ", ".join(map(str, not_oc))
        )
    return out


def main():
    consts = load_constants(HEADER)
    if not consts:
        print(f"ERROR: could not read constants from {HEADER}", file=sys.stderr)
        return 1

    print("=== BLE command-number uniqueness guard ===\n")
    any_failed = False
    oc_values = set()
    for name, path in DISPATCHES:
        failed, lines = check_dispatch(name, path, consts)
        any_failed = any_failed or failed
        print("\n".join(lines))
        print()
        if name == "Out Computer" and path.exists():
            for m in BLE_CMD_RE.finditer(path.read_text()):
                v = resolve(m.group(1), consts)
                if v is not None:
                    oc_values.add(v)

    print("\n".join(swift_cross_reference(oc_values)))
    print()

    if any_failed:
        print("RESULT: FAIL -- fix the duplicate/unresolved command number(s) above.")
        return 1
    print("RESULT: PASS -- every device's ble_cmd dispatch is internally unique.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
