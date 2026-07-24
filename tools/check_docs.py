#!/usr/bin/env python3
"""
Documentation consistency checks for prose that CI can actually verify.

The generated artifacts (section maps, protocol reference) already cannot drift
-- their generators run with --check in CI. This covers the other half: the
hand-written prose, where drift is silent and reads as authoritative.

Every check here corresponds to a defect found by hand in the 2026-07-24 README
audit. None of them is hypothetical:

  * links       3 image tags pointed at files that do not exist, so they
                rendered as broken icons at the top of a public README
  * idf         README said "ESP-IDF v5.x" long after every project moved to v6
  * workflows   README said "Three GitHub Actions workflows", listed four, and
                seven existed
  * struct      README's message table said NonSensor was 43 B (it is 50) and
                LoRa telemetry 59 B (it is 65)

What this deliberately does NOT check: anything requiring judgement about
whether prose is still *true*. A checker that guesses produces false failures,
and a CI check people learn to ignore is worse than no check.

Scope note: docs/plans/ is excluded. Those are design plans written before the
work and explicitly not kept in sync afterwards (see docs/plans/README.md), so
their stale links are correct history, not defects.

Usage:
    python3 tools/check_docs.py           # report and exit 1 on any problem
    python3 tools/check_docs.py --quiet   # only print problems
"""

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
README = REPO / "README.md"
DOCS = REPO / "docs/architecture"
WORKFLOWS = REPO / ".github/workflows"
FW_BUILD = WORKFLOWS / "firmware-build.yml"
TYPES_H = REPO / "tinkerrocket-idf/components/TR_RocketComputerTypes/RocketComputerTypes.h"

# README message-type row -> the struct whose static_assert defines its size.
# There is no mechanical link from a wire code to its struct, so this mapping is
# explicit. A row with no entry is REPORTED rather than skipped, so a new row
# cannot quietly opt out of the check.
CODE_TO_STRUCT = {
    0xA1: "GNSSData",
    0xA2: "ISM6HG256Data",
    0xA3: "BMP585Data",
    0xA4: "MMC5983MAData",
    0xA5: "NonSensorData",
    0xA6: "POWERData",
    0xD1: "IIS2MDCData",
    0xF1: "LoRaData",
}

HTML_COMMENT = re.compile(r"<!--.*?-->", re.S)
LINK = re.compile(r"!?\[([^\]]*)\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")
SIZE_ASSERT = re.compile(r"static_assert\(\s*sizeof\((\w+)\)\s*==\s*(\d+)")
# | 0xA5 | NonSensor (EKF) | 50 B | 500 Hz |
MSG_ROW = re.compile(r"^\|\s*(0[xX][0-9A-Fa-f]{2})\s*\|([^|]*)\|\s*(\d+)\s*B\s*\|")
# A code + size mentioned in running prose, e.g. "`0xA4` (MMC5983MA, 16 B)"
MSG_PROSE = re.compile(r"`?(0[xX][0-9A-Fa-f]{2})`?\s*\([^)]*?(\d+)\s*B\)")


def markdown_files():
    """Root-level docs plus the architecture pages. See the scope note above.

    CONTRIBUTING.md and CLA.md are included because they are the first thing a
    new contributor reads -- a broken link there costs someone their first
    half hour. hardware/README.md is included for the same reason.
    """
    files = [README,
             REPO / "CONTRIBUTING.md",
             REPO / "CLA.md",
             REPO / "hardware/README.md"]
    if DOCS.is_dir():
        files += sorted(DOCS.rglob("*.md"))
    return [f for f in files if f.exists()]


def check_links():
    """Every relative link and image target must resolve on disk."""
    problems = []
    for md in markdown_files():
        # Commented-out blocks are not rendered, so they are not broken links.
        text = HTML_COMMENT.sub("", md.read_text())
        for label, target in LINK.findall(text):
            if target.startswith(("http://", "https://", "mailto:", "#")):
                continue
            # Strip an #anchor and a trailing :line (a repo-wide convention for
            # pointing at a specific line of source).
            path = re.sub(r":\d+(-\d+)?$", "", target.split("#")[0])
            if not path:
                continue
            if not (md.parent / path).exists():
                rel = md.relative_to(REPO)
                problems.append(f"{rel}: [{label}]({target}) -> no such file")
    return problems


def check_idf_version():
    """The IDF version the README tells you to install must be the one CI uses."""
    if not FW_BUILD.exists():
        return [f"{FW_BUILD.relative_to(REPO)} missing; cannot verify the IDF version"]
    m = re.search(r"espressif/idf:(v[\d.]+)", FW_BUILD.read_text())
    if not m:
        return [f"no 'container: espressif/idf:vX.Y.Z' in "
                f"{FW_BUILD.relative_to(REPO)}; update this checker"]
    version = m.group(1)                      # e.g. v6.0.1
    major_minor = ".".join(version.lstrip("v").split(".")[:2])   # 6.0
    text = README.read_text()
    if version in text:
        return []
    if re.search(rf"ESP-IDF v{re.escape(major_minor)}\b", text):
        return []
    return [f"README does not mention ESP-IDF {version} (or v{major_minor}), "
            f"but firmware-build.yml builds on espressif/idf:{version}"]


def check_workflows():
    """The README's CI list must name every workflow, and no phantom ones."""
    problems = []
    on_disk = {p.name for p in sorted(WORKFLOWS.glob("*.yml"))}
    text = README.read_text()
    cited = set(re.findall(r"[\w-]+\.yml", text))

    for name in sorted(on_disk - cited):
        problems.append(f"README does not mention .github/workflows/{name} — "
                        f"add it to the CI table or the list is misleading")
    for name in sorted(cited - on_disk):
        problems.append(f"README mentions {name}, which does not exist in "
                        f".github/workflows/")
    return problems


def check_struct_sizes():
    """Byte sizes quoted in the README must match the header's static_asserts."""
    if not TYPES_H.exists():
        return [f"{TYPES_H.relative_to(REPO)} missing; cannot verify struct sizes"]
    sizes = {name: int(n) for name, n in SIZE_ASSERT.findall(TYPES_H.read_text())}
    problems = []
    text = README.read_text()

    claims = []                                   # (code, claimed_bytes, where)
    for line in text.splitlines():
        m = MSG_ROW.match(line)
        if m:
            claims.append((int(m.group(1), 16), int(m.group(3)), line.strip()))
    for m in MSG_PROSE.finditer(text):
        claims.append((int(m.group(1), 16), int(m.group(2)), m.group(0)))

    for code, claimed, where in claims:
        struct = CODE_TO_STRUCT.get(code)
        if struct is None:
            problems.append(f"README quotes a size for message 0x{code:02X} but "
                            f"CODE_TO_STRUCT in this checker has no entry for it")
            continue
        actual = sizes.get(struct)
        if actual is None:
            problems.append(f"no static_assert(sizeof({struct})) in the header, "
                            f"so 0x{code:02X} cannot be verified")
        elif actual != claimed:
            problems.append(f"README says message 0x{code:02X} is {claimed} B, but "
                            f"sizeof({struct}) == {actual}  [{where}]")
    return problems


CHECKS = [
    ("links resolve", check_links),
    ("ESP-IDF version matches CI", check_idf_version),
    ("CI workflow list is complete", check_workflows),
    ("quoted struct sizes match the wire", check_struct_sizes),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quiet", action="store_true",
                    help="print only failures")
    args = ap.parse_args()

    if not README.exists():
        print(f"ERROR: {README} not found", file=sys.stderr)
        return 1

    failed = 0
    for label, fn in CHECKS:
        problems = fn()
        if problems:
            failed += len(problems)
            print(f"  X  {label}")
            for p in problems:
                print(f"       {p}")
        elif not args.quiet:
            print(f"  ok {label}")

    if failed:
        print(f"\n{failed} documentation problem(s). These are mechanical "
              f"mismatches between the docs and the source, not style opinions "
              f"-- each one means a reader would be misled.", file=sys.stderr)
        return 1
    if not args.quiet:
        print("\nDocumentation consistent with source.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
