#!/usr/bin/env python3
"""Every @Test the Android suites declare must actually have run.

JUnit 5 does not execute a test method whose return type is not void, and
neither it nor Gradle says so: the suite is green, the report just holds
fewer cases than the sources declare. Kotlin gets there in one keyword. An
expression-bodied function takes the type of its last expression, and
`kotlin.test.assertIs<T>` and `assertNotNull` both RETURN the value they
narrowed, so

    @Test fun `x`() = runSync { ...; assertIs<Foo>(got) }

is a method returning Foo, and JUnit skips it. FirmwareReleaseTest declared
13 of these and the report said 11 (2026-09-10, #773 step 4b); both missing
cases would have passed, so nothing looked wrong. The same silence covers a
`private fun` test, a test inside a private class, and a JUnit 4
`org.junit.Test` on the JUnit Platform with no vintage engine.

The check is a count. Per module, the number of `@Test` annotations under
src/test/**/*.kt must equal the number of <testcase> elements across that
module's JUnit XML reports. Per MODULE deliberately, not per file or class:
one .kt file can hold two test classes (PreflightTest/PreflightStoreTest,
KnownDeviceStoreTest/DeviceTypeFromNameTest), and each class writes its own
report, so a per-file comparison flags those as mismatches. Module totals are
exact. Disabled tests still appear in the XML as skipped cases, so they do
not disturb the count.

What it does not model, and says so rather than guessing: @ParameterizedTest,
@RepeatedTest, @TestFactory and @TestTemplate each run one annotation as many
cases, and a @Test inherited from a base class runs once per subclass. None
exist today; if one arrives, teach this checker rather than skipping it.

Run after `./gradlew test`, from anywhere; paths resolve relative to this file:

    python3 tools/check_android_tests_ran.py
"""
from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

ANDROID = Path(__file__).resolve().parent.parent / "TinkerRocketAndroid"
SETTINGS = ANDROID / "settings.gradle.kts"

# `include(":core:protocol")` -- one Gradle module per line.
INCLUDE_RE = re.compile(r'^\s*include\(\s*"(:[\w:.-]+)"\s*\)', re.M)

# The annotation, bare or fully qualified. The \b keeps @TestFactory out.
TEST_RE = re.compile(
    r"@(?:kotlin\.test\.|org\.junit\.jupiter\.api\.|org\.junit\.)?Test\b")
# Annotations that run one declaration as several cases; see the docstring.
MULTI_RE = re.compile(
    r"@(?:[\w.]+\.)?(ParameterizedTest|RepeatedTest|TestFactory|TestTemplate)\b")
# The function a @Test annotates: the next `fun` after it, backticked or
# plain, allowing a type-parameter list and a receiver in between.
FUN_RE = re.compile(
    r"\bfun\s+(?:<[^>]*>\s*)?(?:[\w.]+\.)?(?:`([^`]+)`|(\w+))\s*\(")

# Where Gradle writes JUnit XML under build/test-results: `test` for a
# kotlin("jvm") module, one `test<Variant>UnitTest` per build variant for an
# Android module. Each holds a full run of the module's suite, so each must
# match the declared count on its own.
REPORT_DIR_PATTERNS = ("test", "test*UnitTest")

# What a shortfall means, in the words of the person who has to fix it.
SKIPPED_WHY = (
    "JUnit 5 silently skips a test method that is not void. In Kotlin that is "
    "an expression body whose last expression is non-Unit -- assertIs and "
    "assertNotNull both return the narrowed value; write `(): Unit =` -- or a "
    "`private fun`, a private class, or a JUnit 4 `org.junit.Test` import."
)
SURPLUS_WHY = (
    "more cases ran than the sources declare: stale reports from a deleted "
    "or renamed test (run `./gradlew test` again), or a parameterized, "
    "repeated or inherited test this checker does not model -- see its "
    "docstring."
)


def strip_comments_and_strings(src: str) -> str:
    """Blank out comments (Kotlin nests /* */) and string literals, keeping
    newlines. A `//` inside a URL literal must not eat the rest of its line,
    and a `@Test` quoted in prose must not count as a declaration. A string
    template nesting quotes (`"${m["k"]}"`) mis-pairs here, but the worst
    case is a fragment of one line leaking through, never a lost annotation.
    """
    out: list[str] = []
    i, n = 0, len(src)
    while i < n:
        pair = src[i:i + 2]
        if pair == "//":
            end = src.find("\n", i)
            i = n if end < 0 else end
        elif pair == "/*":
            depth, i = 1, i + 2
            while i < n and depth:
                pair = src[i:i + 2]
                if pair == "/*":
                    depth, i = depth + 1, i + 2
                elif pair == "*/":
                    depth, i = depth - 1, i + 2
                else:
                    if src[i] == "\n":
                        out.append("\n")
                    i += 1
        elif src.startswith('"""', i):
            end = src.find('"""', i + 3)
            end = n if end < 0 else end + 3
            out.append('""' + "\n" * src.count("\n", i, end))
            i = end
        elif src[i] == '"':
            j = i + 1
            while j < n and src[j] not in '"\n':
                j += 2 if src[j] == "\\" else 1
            out.append('""')
            i = min(j + 1, n)
        else:
            out.append(src[i])
            i += 1
    return "".join(out)


def declared_tests(kt: Path) -> tuple[list[str], list[str]]:
    """Names of the functions this file's @Test annotations sit on, plus any
    annotation kind this checker does not model."""
    code = strip_comments_and_strings(kt.read_text(encoding="utf-8"))
    names = []
    for m in TEST_RE.finditer(code):
        f = FUN_RE.search(code, m.end())
        names.append((f.group(1) or f.group(2)) if f else "?")
    unmodelled = [m.group(1) for m in MULTI_RE.finditer(code)]
    return names, unmodelled


def executed_tests(report_dir: Path) -> tuple[list[str], list[str]]:
    """Test-case names across one report directory, plus anything about the
    reports that does not look like the JUnit XML this checker expects."""
    names, problems = [], []
    for xml in sorted(report_dir.glob("TEST-*.xml")):
        try:
            root = ET.parse(xml).getroot()
        except ET.ParseError as e:
            problems.append(f"{xml.name}: not parseable JUnit XML ({e})")
            continue
        cases = root.findall(".//testcase")
        stated = root.get("tests")
        if stated is not None and int(stated) != len(cases):
            problems.append(f'{xml.name}: tests="{stated}" but '
                            f"{len(cases)} <testcase> elements -- the report "
                            f"format is not what this checker expects")
        for case in cases:
            # `name()` in the XML is the Kotlin `name`; strip the call suffix.
            names.append(re.sub(r"\([^()]*\)$", "", case.get("name", "")))
    return names, problems


def report_dirs(module: Path) -> list[Path]:
    base = module / "build" / "test-results"
    found: list[Path] = []
    for pat in REPORT_DIR_PATTERNS:
        found += [d for d in sorted(base.glob(pat)) if d.is_dir() and d not in found]
    return found


def modules() -> list[str]:
    return [m.lstrip(":").replace(":", "/")
            for m in INCLUDE_RE.findall(SETTINGS.read_text())]


def rel(path: Path) -> str:
    return str(path.relative_to(ANDROID))


def main() -> int:
    if not SETTINGS.exists():
        print(f"ERROR: {SETTINGS} not found", file=sys.stderr)
        return 1
    mods = modules()
    if not mods:
        print(f"ERROR: no include(...) lines in {SETTINGS}; update INCLUDE_RE",
              file=sys.stderr)
        return 1

    failures: list[str] = []
    total_ran = 0
    print(f"{'module':16s} {'declared':>9s} {'ran':>6s}  reports")
    for mod in mods:
        root = ANDROID / mod
        test_src = root / "src" / "test"
        sources = sorted(test_src.rglob("*.kt")) if test_src.is_dir() else []

        declared: Counter[str] = Counter()
        for kt in sources:
            names, unmodelled = declared_tests(kt)
            declared.update(names)
            for kind in unmodelled:
                failures.append(
                    f"{rel(kt)}: @{kind} runs one declaration as several "
                    f"cases, which this checker does not model -- teach it "
                    f"before relying on the count")
        n_declared = sum(declared.values())

        dirs = report_dirs(root)
        if not dirs:
            if n_declared == 0:
                print(f"{mod:16s} {0:9d} {0:6d}  (no JVM tests)")
            else:
                print(f"{mod:16s} {n_declared:9d} {'none':>6s}  (no reports)   <-- FAIL")
                failures.append(
                    f"{mod}: {n_declared} @Test declared under src/test but "
                    f"no JUnit XML under build/test-results -- the test task "
                    f"did not run for this module; run `./gradlew test` first")
            continue

        for d in dirs:
            ran, problems = executed_tests(d)
            failures += [f"{mod}: {p}" for p in problems]
            n_ran = len(ran)
            total_ran += n_ran
            flag = "" if n_ran == n_declared else "   <-- FAIL"
            print(f"{mod:16s} {n_declared:9d} {n_ran:6d}  {rel(d)}{flag}")
            if n_ran == n_declared:
                continue
            missing = sorted((declared - Counter(ran)).elements())
            surplus = sorted((Counter(ran) - declared).elements())
            msg = (f"{mod}: {n_declared} @Test declared under src/test, "
                   f"{n_ran} ran ({rel(d)}).")
            if missing:
                msg += ("\n      declared but in no report: "
                        + ", ".join(f"`{m}`" for m in missing)
                        + "\n      " + SKIPPED_WHY)
            if surplus:
                msg += ("\n      ran but not declared: "
                        + ", ".join(f"`{s}`" for s in surplus)
                        + "\n      " + SURPLUS_WHY)
            failures.append(msg)

    # Test sources that no included module owns never compile, let alone run.
    for kt in sorted(ANDROID.rglob("*.kt")):
        parts = kt.relative_to(ANDROID).parts
        if "build" in parts or "src" not in parts:
            continue
        if parts[parts.index("src") + 1:][:1] != ("test",):
            continue
        if not any(kt.is_relative_to(ANDROID / m) for m in mods):
            failures.append(f"{rel(kt)}: test source outside every module "
                            f"in settings.gradle.kts -- nothing runs it")

    if failures:
        print("\nFAIL", file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1
    print(f"\nOK -- every @Test declared under TinkerRocketAndroid ran: "
          f"{total_ran} cases across {len(mods)} modules")
    return 0


if __name__ == "__main__":
    sys.exit(main())
