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
    # Variable-length on the wire (10 + 6 * num_blocks); the README quotes the
    # struct's full size, which is what the static_assert pins.
    0x90: "GNSSSatData",
    0xA1: "GNSSData",
    0xA2: "ISM6HG256Data",
    0xA3: "BMP585Data",
    0xA4: "MMC5983MAData",
    0xA5: "NonSensorData",
    0xA6: "POWERData",
    0xD1: "IIS2MDCData",
    # #850: 0xF1 is TWO frames now. A tuple means "the README must quote a
    # size that matches ONE of these", which is what a mixed-frame log holds.
    0xF1: ("LoRaFastData", "LoRaSlowData"),
    0xF9: "LoRaUplinkData",
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


# Spelled-out counts the README uses for its CI table.  #838 item 9: the
# README said "Nine" above a table listing ten, and the checker could not see
# it — it only diffed *names*.  A wrong count is what a reader skims.
_COUNT_WORDS = {
    "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
    "seven": 7, "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12,
    "thirteen": 13, "fourteen": 14, "fifteen": 15, "sixteen": 16,
}


def check_workflows():
    """The README's CI list must name every workflow, count them right, and
    both docs must describe the firmware-build matrix as it actually is."""
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

    # The count word above the table.
    m = re.search(r"(\w+) GitHub Actions workflows run automatically", text)
    if m is None:
        problems.append("README no longer states how many workflows there are; "
                        "the count sentence above the CI table is the thing "
                        "readers skim")
    else:
        word = m.group(1).lower()
        stated = _COUNT_WORDS.get(word)
        if stated is None:
            problems.append(f'README says "{m.group(1)} GitHub Actions workflows" '
                            f"— not a count word this checker knows; spell it out")
        elif stated != len(on_disk):
            problems.append(f'README says "{m.group(1)} GitHub Actions workflows" '
                            f"but {len(on_disk)} exist in .github/workflows/")

    # CONTRIBUTING deliberately lists a run-these-locally SUBSET, so it is not
    # checked for completeness — only for phantoms.
    contributing = REPO / "CONTRIBUTING.md"
    if contributing.exists():
        for name in sorted(set(re.findall(r"[\w-]+\.yml", contributing.read_text())) - on_disk):
            problems.append(f"CONTRIBUTING.md mentions {name}, which does not "
                            f"exist in .github/workflows/")

    # Every project in the firmware-build matrix must be named in both docs.
    problems += _check_firmware_matrix(text, contributing)
    return problems


def _check_firmware_matrix(readme_text, contributing_path):
    """Both docs enumerate the firmware-build projects; the matrix is truth."""
    fw = WORKFLOWS / "firmware-build.yml"
    if not fw.exists():
        return []
    # Deliberately a regex, not a YAML parse: check_docs.py has no third-party
    # dependencies and runs before anything is installed.
    projects = sorted(set(re.findall(r"^\s*-?\s*project:\s*([\w-]+)",
                                     fw.read_text(), re.MULTILINE)))
    if not projects:
        return ["firmware-build.yml has no matrix `project:` entries; the "
                "matrix check cannot verify the docs"]
    problems = []

    # README's CI table is the enumerating one: it must name every project.
    row = next((ln for ln in readme_text.splitlines() if "firmware-build.yml" in ln), None)
    if row is not None:
        missing = [p for p in projects if p not in row]
        if missing:
            problems.append(
                f"README.md's firmware-build row omits {', '.join(missing)} — the "
                f"matrix builds {len(projects)} projects, so a reader concludes "
                f"CI does not cover the ones left out")

    # CONTRIBUTING's table is a deliberately terse run-these-locally summary,
    # so it says "all N firmware projects" rather than listing them. Check the
    # COUNT — "all four" was the #838 item 9 defect, and a wrong number there
    # is what makes a contributor skip a local run.
    if contributing_path.exists():
        ctext = contributing_path.read_text()
        crow = next((ln for ln in ctext.splitlines() if "firmware-build.yml" in ln), None)
        if crow is not None:
            m = re.search(r"all (\w+) firmware projects", crow)
            if m is None:
                missing = [p for p in projects if p not in crow]
                if missing:
                    problems.append(
                        f"CONTRIBUTING.md's firmware-build row neither says "
                        f'"all N firmware projects" nor names {", ".join(missing)}')
            else:
                stated = _COUNT_WORDS.get(m.group(1).lower())
                if stated is None:
                    problems.append(f'CONTRIBUTING.md says "all {m.group(1)} firmware '
                                    f'projects" — not a count word this checker knows')
                elif stated != len(projects):
                    problems.append(
                        f'CONTRIBUTING.md says "all {m.group(1)} firmware projects" but '
                        f"firmware-build.yml builds {len(projects)}")
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
        # One code may map to several wire structs (#850: 0xF1 is a fast frame
        # OR a slow one). Every named struct must still be pinned by a
        # static_assert, and the README's number must match one of them.
        structs = struct if isinstance(struct, tuple) else (struct,)
        missing = [n for n in structs if sizes.get(n) is None]
        if missing:
            problems.append(f"no static_assert(sizeof({', '.join(missing)})) in the "
                            f"header, so 0x{code:02X} cannot be verified")
            continue
        actual = [sizes[n] for n in structs]
        if claimed not in actual:
            expect = " or ".join(f"sizeof({n}) == {sizes[n]}" for n in structs)
            problems.append(f"README says message 0x{code:02X} is {claimed} B, but "
                            f"{expect}  [{where}]")
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
