#!/usr/bin/env python3
"""#1086: ban the bare Kotlin `String.format` EXTENSION under the Android app
and core modules.

`"%.1f".format(x)` calls `kotlin.text.String.format`, which takes
`Locale.getDefault()` — a Danish or German phone then prints `55,3` where iOS
(C printf, always '.') prints `55.3`, and the two apps' screenshots stop being
comparable. The fix at every call site is `String.format(Locale.ROOT, "%.1f", x)`.

This guard flags a string-literal receiver immediately followed by `.format(`
(the form the whole codebase uses); it does NOT flag `String.format(...)`,
`Locale`-qualified calls, java time/BigDecimal `.format`, or variable
receivers. Run from the repo root. Exit 1 lists offenders.
"""
import re, sys, pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
DIRS = [
    ROOT / "TinkerRocketAndroid" / "app" / "src" / "main",
    ROOT / "TinkerRocketAndroid" / "core",
]
# a string literal (possibly interpolated) or its concatenation, then `.format(`
#   "…".format(     or     …}".format(     or     …").format(
PAT = re.compile(r'"\s*\)?\.format\(|\}"\)?\.format\(')

def main() -> int:
    offenders = []
    for base in DIRS:
        for path in base.rglob("*.kt"):
            if "/src/test/" in str(path) or "/src/androidTest/" in str(path):
                continue
            for i, line in enumerate(path.read_text().splitlines(), 1):
                stripped = line.lstrip()
                if stripped.startswith(("*", "//", "/*")):
                    continue  # KDoc / comment mentioning .format is not a call
                if "String.format" in line:
                    continue
                if PAT.search(line):
                    offenders.append((path.relative_to(ROOT), i, line.strip()))
    if offenders:
        print(f"{len(offenders)} default-locale String.format extension call(s) "
              f"— use String.format(Locale.ROOT, ...) instead (#1086):")
        for rel, ln, txt in offenders:
            print(f"  {rel}:{ln}  {txt[:100]}")
        return 1
    print("OK — no default-locale String.format extension under app/ or core/.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
