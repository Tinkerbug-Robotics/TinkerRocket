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
the OC but "relay to rocket" to the BS; that overlap is correct.  The apps
therefore mix both spaces; the firmware dispatch is the authority where a
duplicate actually causes a bug.

App-parity check (android-port plan section 2.2): the iOS app's literal
send(Raw)Command numbers -- swept across ALL app Swift files, since
SimulationView sends cmds 5/6 itself -- plus the KNOWN_RAW_WRITE_CMDS
(file ops sent as raw Data([N, ...]) writes the regex cannot see) must
EXACTLY equal the Kotlin BleCommandId const-val set.  A command added to one
app but not the other is the new silent-drift failure mode a second app
introduces; this makes it a hard CI failure.  While Android is mid-port,
ALLOWED_DIVERGENCE holds the burn-down list.  Every app number must also
exist in some firmware dispatch (OC, BS, or the OTA trio handled inside
TR_BLE_To_APP.cpp) -- the "app sends a number nobody handles" hole.

Usage (runs from anywhere):
    python3 tools/check_ble_command_ids.py
Exit 0 = OK, exit 1 = duplicate / unresolved RHS / app-parity failure.
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
SWIFT_APP_DIR = REPO / "TinkerRocketApp/TinkerRocketApp"
KOTLIN = (
    REPO / "TinkerRocketAndroid/core/protocol/src/main/kotlin/"
    "com/tinkerbug/tinkerrocket/protocol/BleCommandId.kt"
)

# File-op commands sent as raw `Data([N, ...])` + writeValue in
# BLEDevice.swift (requestFileList / deleteFile / downloadFile) -- invisible
# to SWIFT_RE.  If a new command is ever added via a raw write instead of
# send(Raw)Command, it must be added here or the parity check will (rightly)
# scream.
KNOWN_RAW_WRITE_CMDS = {2, 3, 4}

# Dispatched inside TR_BLE_To_APP.cpp (OTA begin/finish/abort), not the
# main.cpp chains grepped above -- legitimate app commands with no
# `ble_cmd ==` branch.
BLE_TO_APP_CMDS = {70, 71, 72}

# Burn-down list for the Android port: SWIFT-ONLY command numbers whose Kotlin
# port hasn't landed yet (the only excusable direction — a Kotlin-only number
# is always an error).  Every entry needs a trailing comment naming the
# feature; entries that stop being divergent are flagged stale and must be
# removed, so the list mechanically trends to empty.
ALLOWED_DIVERGENCE: set[int] = set()

# Guard-the-guard: SWIFT_RE only sees integer literals at call sites.  If a
# refactor moved the Swift app to named constants, the extracted set would
# silently shrink and the parity diff would degrade to noise.  The app has
# ~50 distinct literal command numbers; well under that means the extraction
# broke, not the protocol.
MIN_SWIFT_DISTINCT = 40

# `static constexpr uint8_t NAME = 0xNN;`  /  `= 17;`
CONST_RE = re.compile(
    r"static\s+constexpr\s+uint8_t\s+(\w+)\s*=\s*(0[xX][0-9A-Fa-f]+|\d+)\s*;"
)
# RHS of `ble_cmd ==` is either a numeric literal or an identifier.
BLE_CMD_RE = re.compile(r"ble_cmd\s*==\s*(0[xX][0-9A-Fa-f]+|\d+|[A-Za-z_]\w*)")
# Swift senders: sendCommand(N) / sendRawCommand(N, ...) — decimal or hex.
SWIFT_RE = re.compile(r"send(?:Raw)?Command\(\s*(0[xX][0-9A-Fa-f]+|\d+)")
# Kotlin command table: `const val NAME: Int = N` (BleCommandId.kt only).
KOTLIN_CONST_RE = re.compile(
    r"const\s+val\s+(\w+)(?:\s*:\s*Int)?\s*=\s*(0[xX][0-9A-Fa-f]+|\d+)"
)

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


def collect_swift_commands():
    """Literal send(Raw)Command numbers across the whole iOS app tree."""
    nums = set()
    files = 0
    for path in sorted(SWIFT_APP_DIR.rglob("*.swift")):
        found = {int(m.group(1), 0) for m in SWIFT_RE.finditer(path.read_text())}
        if found:
            files += 1
            nums |= found
    return nums, files


def collect_kotlin_commands():
    """const-val values from the single mandated Kotlin command table."""
    if not KOTLIN.exists():
        return None
    return {
        int(m.group(2), 0) for m in KOTLIN_CONST_RE.finditer(KOTLIN.read_text())
    }


def app_parity_check(oc_values, bs_values):
    """HARD check: Swift command set == Kotlin command set, and every app
    number is handled by some firmware dispatch.  Returns (failed, lines)."""
    out = ["App parity (Swift <-> Kotlin <-> firmware, enforced):"]
    failed = False

    swift_nums, files = collect_swift_commands()
    out.append(
        f"  Swift: {len(swift_nums)} distinct literal numbers across {files} files"
        f" + raw-write cmds {sorted(KNOWN_RAW_WRITE_CMDS)}"
    )
    if len(swift_nums) < MIN_SWIFT_DISTINCT:
        failed = True
        out.append(
            f"  FAIL: only {len(swift_nums)} distinct Swift literals found "
            f"(< {MIN_SWIFT_DISTINCT}).  Either commands moved behind named "
            "constants (teach SWIFT_RE the new shape) or the extraction broke."
        )
    swift_all = swift_nums | KNOWN_RAW_WRITE_CMDS

    kotlin_nums = collect_kotlin_commands()
    if kotlin_nums is None:
        failed = True
        out.append(f"  FAIL: Kotlin command table missing at {KOTLIN.relative_to(REPO)}")
        return failed, out
    out.append(f"  Kotlin: {len(kotlin_nums)} distinct const-val values (BleCommandId.kt)")

    raw_only_swift = swift_all - kotlin_nums
    raw_only_kotlin = kotlin_nums - swift_all
    # ALLOWED_DIVERGENCE excuses ONE direction only: Swift has a command the
    # Kotlin port hasn't reached yet (the legitimate mid-port state).  A
    # Kotlin-only number is never excusable — it's a typo or a stale entry.
    only_swift = raw_only_swift - ALLOWED_DIVERGENCE
    only_kotlin = raw_only_kotlin
    if only_swift:
        failed = True
        out.append(
            "  FAIL: in Swift but not Kotlin -> "
            + ", ".join(map(str, sorted(only_swift)))
            + "  (add to BleCommandId.kt, or to ALLOWED_DIVERGENCE with a comment)"
        )
    if only_kotlin:
        failed = True
        out.append(
            "  FAIL: in Kotlin but not Swift -> "
            + ", ".join(map(str, sorted(only_kotlin)))
            + "  (missing Swift sender, or stale Kotlin entry)"
        )
    # Stale entries would otherwise suppress FUTURE drift of that number
    # forever — the burn-down list must actually burn down.
    stale = ALLOWED_DIVERGENCE - raw_only_swift
    if stale:
        failed = True
        out.append(
            "  FAIL: stale ALLOWED_DIVERGENCE entries (no longer divergent) -> "
            + ", ".join(map(str, sorted(stale)))
            + "  (remove them)"
        )
    if ALLOWED_DIVERGENCE:
        out.append(
            "  allowed divergence (burn-down): "
            + ", ".join(map(str, sorted(ALLOWED_DIVERGENCE)))
        )

    handled = oc_values | bs_values | BLE_TO_APP_CMDS
    unhandled = sorted((swift_all | kotlin_nums) - handled)
    if unhandled:
        failed = True
        out.append(
            "  FAIL: app command(s) with no firmware handler (not in OC or BS "
            "dispatch, not in the TR_BLE_To_APP OTA trio) -> "
            + ", ".join(map(str, unhandled))
        )

    if not failed:
        out.append("  -> OK (Swift == Kotlin, all numbers firmware-handled)")
    return failed, out


def main():
    consts = load_constants(HEADER)
    if not consts:
        print(f"ERROR: could not read constants from {HEADER}", file=sys.stderr)
        return 1

    print("=== BLE command-number uniqueness guard ===\n")
    any_failed = False
    dispatch_values = {}
    for name, path in DISPATCHES:
        failed, lines = check_dispatch(name, path, consts)
        any_failed = any_failed or failed
        print("\n".join(lines))
        print()
        values = set()
        if path.exists():
            for m in BLE_CMD_RE.finditer(path.read_text()):
                v = resolve(m.group(1), consts)
                if v is not None:
                    values.add(v)
        dispatch_values[name] = values

    parity_failed, lines = app_parity_check(
        dispatch_values["Out Computer"], dispatch_values["Base Station"]
    )
    any_failed = any_failed or parity_failed
    print("\n".join(lines))
    print()

    if any_failed:
        print("RESULT: FAIL -- fix the finding(s) above.")
        return 1
    print(
        "RESULT: PASS -- dispatches internally unique; Swift and Kotlin "
        "command sets match and every number has a firmware handler."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
