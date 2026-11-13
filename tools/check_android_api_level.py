#!/usr/bin/env python3
"""No module of the Android app may call a Java or Android API above its minSdk.

The app's minSdk is 31. Its :core:protocol, :core:session and :core:maps
modules are plain kotlin("jvm") modules, compiled against JDK 21's class
library, so any Java 9+ API compiles there and nothing checked the level.
That let `InputStream.readNBytes(int)`, Android API 33, into HttpFirmwareFetch.
On Android 12 it throws NoSuchMethodError, an Error, which the fetch's
`catch (Exception)` does not catch (#1612).

Android Lint's NewApi check cannot see those modules. Lint 32 (AGP 9.0.0)
runs NewApi only in a module that is an Android project, and AGP analyzes
each module on its own, so a kotlin("jvm") module is skipped even with the
com.android.lint plugin applied. Tried: `:app:lintDebug` with
`checkDependencies = true` analyzed all three modules and did not flag the
readNBytes call.

So this reads the bytecode. For every class compiled from a module's main
sources it takes the constant pool's Class, Fieldref, Methodref and
InterfaceMethodref entries, which name every class, field and method the code
can reach, and looks each one up in the SDK's api-versions.xml, the database
lint uses. A member resolves through the supertypes listed there, and through
our own classes' supertypes on the way. A reference fails if it needs an API
level above minSdk, or names a java/javax/android class or member that
Android lacks at every level, unless:

  - D8 backports it into the APK (D8's BackportedMethodList for minSdk);
  - it is LambdaMetafactory or StringConcatFactory, the bootstrap methods of
    Kotlin's lambdas and string templates, which D8 desugars; or
  - it is listed in GUARDED below: an Android call behind a
    Build.VERSION.SDK_INT check, which bytecode cannot see.

What it does not model: a member our class inherits through a library
class. A call names the receiver's static type, so MainActivity's calls to
methods it inherits from Activity name MainActivity, whose superclass is
androidx's ComponentActivity. Library class files are not read, so the walk
stops there (six calls on 2026-10-13, none above API 23). Nor does it check
the libraries' own calls.

Run after `./gradlew test`, which compiles every module's main classes, from
anywhere. It needs ANDROID_HOME, and java on PATH or JAVA_HOME to run D8:

    python3 tools/check_android_api_level.py
"""
from __future__ import annotations

import os
import re
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import NamedTuple

ANDROID = Path(__file__).resolve().parent.parent / "TinkerRocketAndroid"
SETTINGS = ANDROID / "settings.gradle.kts"
APP_GRADLE = ANDROID / "app" / "build.gradle.kts"

# `include(":core:protocol")` -- one Gradle module per line.
INCLUDE_RE = re.compile(r'^\s*include\(\s*"(:[\w:.-]+)"\s*\)', re.M)

# Where Gradle writes a module's compiled main classes: under build/classes
# for a kotlin("jvm") module; under build/intermediates for an Android module,
# from AGP 9's built-in Kotlin and from javac (BuildConfig). Debug, because
# `./gradlew test` compiles it; release compiles the same sources.
CLASS_DIRS = (
    "build/classes/kotlin/main",
    "build/classes/java/main",
    "build/intermediates/built_in_kotlinc/debug/compileDebugKotlin/classes",
    "build/intermediates/javac/debug/compileDebugJavaWithJavac/classes",
)

# The D8 whose backport list says which calls end up rewritten in the APK.
# AGP 9.0.0 dexes with its own D8 (9.0.32), which at minSdk 31 backports
# everything this one does and 7 methods more, so the difference can only
# fail a call that is safe, never pass one that is not. The CI runner image
# has this version too.
D8_BUILD_TOOLS = "36.1.0"

# Kotlin compiles lambdas and string templates to invokedynamic with these as
# bootstrap methods. D8 desugars both; neither class exists on Android.
DESUGARED = {
    "java/lang/invoke/LambdaMetafactory",
    "java/lang/invoke/StringConcatFactory",
}

# Android calls above minSdk that sit behind a Build.VERSION.SDK_INT check.
# Each entry allows the call from one class, its nested classes and lambdas
# included; the same call from anywhere else still fails. An entry that no
# longer matches a call fails too, so a removed call takes its entry with it.
GUARDED = {
    # enableNotifications and write: SDK_INT >= 33, else the deprecated
    # set-the-value-then-write pair.
    "com/tinkerbug/tinkerrocket/ble/RealBleTransport": {
        "android/bluetooth/BluetoothGatt#writeCharacteristic"
        "(Landroid/bluetooth/BluetoothGattCharacteristic;[BI)I",
        "android/bluetooth/BluetoothGatt#writeDescriptor"
        "(Landroid/bluetooth/BluetoothGattDescriptor;[B)I",
    },
    # locationCallback: SDK_INT >= 34, else the ellipsoid altitude.
    "com/tinkerbug/tinkerrocket/app/PhoneLocationManager": {
        "android/location/Location#hasMslAltitude()Z",
        "android/location/Location#getMslAltitudeMeters()D",
    },
}

# Android supplies these packages, and every package api-versions.xml names
# (org/json, say). A class or member there that the database lacks is missing
# on every phone, whatever the JDK it compiled against had.
PLATFORM_PREFIXES = ("android/", "dalvik/", "java/", "javax/")

NEVER = 10**6  # "API level" of something Android has at no level

WHAT_TO_DO = (
    "On a phone below that level each is a NoSuchMethodError or "
    "NoClassDefFoundError, an Error that `catch (e: Exception)` lets "
    "through. Use an API that exists at minSdk -- copyCapped for capped "
    "InputStream reads -- or put the call behind "
    "`Build.VERSION.SDK_INT >= <level>` and list it in GUARDED in "
    "tools/check_android_api_level.py."
)


class ClassFile(NamedTuple):
    name: str
    source: str            # SourceFile attribute, e.g. HttpFirmwareFetch.kt
    supers: list[str]      # superclass, then interfaces
    declared: set[str]     # own methods (name + descriptor) and fields (name)
    classes: set[str]      # Class constants
    members: set[tuple[str, str]]  # (owner, member) of Field/Method refs


def read_class(data: bytes) -> ClassFile:
    """The parts of a class file this check needs (JVM spec chapter 4)."""
    def u2(at: int) -> int:
        return struct.unpack_from(">H", data, at)[0]

    def u4(at: int) -> int:
        return struct.unpack_from(">I", data, at)[0]

    if data[:4] != b"\xca\xfe\xba\xbe":
        raise ValueError("not a class file")
    count, pos, i = u2(8), 10, 1
    utf8: dict[int, str] = {}
    class_of: dict[int, int] = {}            # Class -> its name's Utf8
    refs: list[tuple[int, int]] = []         # Field/Method refs: class, NameAndType
    name_type: dict[int, tuple[int, int]] = {}
    while i < count:
        tag = data[pos]
        if tag == 1:                         # Utf8 (modified; names are ASCII)
            n = u2(pos + 1)
            utf8[i] = data[pos + 3:pos + 3 + n].decode("utf-8", "replace")
            pos += 3 + n
        elif tag == 7:                       # Class
            class_of[i] = u2(pos + 1)
            pos += 3
        elif tag in (9, 10, 11):             # Field, Method, InterfaceMethod ref
            refs.append((u2(pos + 1), u2(pos + 3)))
            pos += 5
        elif tag == 12:                      # NameAndType
            name_type[i] = (u2(pos + 1), u2(pos + 3))
            pos += 5
        elif tag in (5, 6):                  # Long, Double: two slots
            pos += 9
            i += 1
        elif tag in (3, 4, 17, 18):          # Integer, Float, (Invoke)Dynamic
            pos += 5
        elif tag == 15:                      # MethodHandle
            pos += 4
        elif tag in (8, 16, 19, 20):         # String, MethodType, Module, Package
            pos += 3
        else:
            raise ValueError(f"unknown constant pool tag {tag}")
        i += 1

    def cls(index: int) -> str:
        return utf8[class_of[index]]

    this, sup = u2(pos + 2), u2(pos + 4)
    n = u2(pos + 6)
    supers = ([cls(sup)] if sup else []) + [cls(u2(pos + 8 + 2 * k))
                                            for k in range(n)]
    pos += 8 + 2 * n

    declared: set[str] = set()
    for is_method in (False, True):          # fields, then methods
        n, pos = u2(pos), pos + 2
        for _ in range(n):
            name, desc = utf8[u2(pos + 2)], utf8[u2(pos + 4)]
            declared.add(name + desc if is_method else name)
            attributes, pos = u2(pos + 6), pos + 8
            for _ in range(attributes):
                pos += 6 + u4(pos + 2)
    source = "?"
    attributes, pos = u2(pos), pos + 2
    for _ in range(attributes):
        if utf8[u2(pos)] == "SourceFile":
            source = utf8[u2(pos + 6)]
        pos += 6 + u4(pos + 2)

    members = set()
    for owner, nt in refs:
        name, desc = name_type[nt]
        # A field's member is its name; a method's is name + descriptor,
        # the form api-versions.xml and D8's list both use.
        member = utf8[name] + (utf8[desc] if utf8[desc].startswith("(") else "")
        members.add((cls(owner), member))
    return ClassFile(cls(this), source, supers, declared,
                     {cls(k) for k in class_of}, members)


class ApiDatabase:
    """api-versions.xml: the API level each platform class and member
    arrived at, and which supertypes each class has had since when."""

    def __init__(self, path: Path):
        self.classes: dict[str, tuple[int, dict[str, int],
                                      list[tuple[str, int]]]] = {}
        for _, el in ET.iterparse(path):
            if el.tag != "class":
                continue
            since = int(el.get("since", "1"))
            members: dict[str, int] = {}
            supers: list[tuple[str, int]] = []
            for child in el:
                level = int(child.get("since", since))
                if child.tag in ("extends", "implements"):
                    supers.append((child.get("name"), level))
                elif child.tag in ("method", "field"):
                    members[child.get("name")] = level
            self.classes[el.get("name")] = (since, members, supers)
            el.clear()
        self.packages = {c.rpartition("/")[0] for c in self.classes}
        self._memo: dict[tuple[str, str], int | None] = {}

    def is_platform(self, name: str) -> bool:
        return (name.startswith(PLATFORM_PREFIXES)
                or name.rpartition("/")[0] in self.packages)

    def class_level(self, name: str) -> int | None:
        entry = self.classes.get(name)
        return entry[0] if entry else None

    def member_level(self, owner: str, member: str) -> int | None:
        """The lowest level at which `owner.member` resolves, the way lint's
        ApiLookup reckons it: the class's own declaration, or a supertype's
        from when the class gained that supertype. Constructors are not
        inherited. None if no level has it."""
        key = (owner, member)
        if key not in self._memo:
            self._memo[key] = None           # cuts a cycle, if one exists
            entry = self.classes.get(owner)
            best = None
            if entry:
                _, members, supers = entry
                best = members.get(member)
                if not member.startswith("<init>"):
                    for sup, edge in supers:
                        level = self.member_level(sup, member)
                        if level is not None:
                            level = max(level, edge)
                            best = level if best is None else min(best, level)
            self._memo[key] = best
        return self._memo[key]


def gradle_int(name: str) -> int:
    m = re.search(rf"^\s*{name}\s*=\s*(\d+)\b", APP_GRADLE.read_text(), re.M)
    if not m:
        sys.exit(f"ERROR: no `{name} = <n>` in {APP_GRADLE}; update gradle_int")
    return int(m.group(1))


def d8_backports(sdk: Path, min_sdk: int) -> set[str]:
    jar = sdk / "build-tools" / D8_BUILD_TOOLS / "lib" / "d8.jar"
    if not jar.is_file():
        sys.exit(f"ERROR: {jar} not found -- install build-tools "
                 f"{D8_BUILD_TOOLS}, or move D8_BUILD_TOOLS to one installed")
    java = Path(os.environ["JAVA_HOME"]) / "bin" / "java" \
        if os.environ.get("JAVA_HOME") else "java"
    try:
        run = subprocess.run(
            [str(java), "-cp", str(jar), "com.android.tools.r8.BackportedMethodList",
             "--min-api", str(min_sdk)],
            capture_output=True, text=True)
    except OSError as e:
        sys.exit(f"ERROR: cannot run {java} for D8 ({e}) -- set JAVA_HOME")
    if run.returncode != 0 or not run.stdout.strip():
        sys.exit(f"ERROR: D8's BackportedMethodList failed:\n{run.stderr}")
    return set(run.stdout.split())


def main() -> int:
    sdk_env = os.environ.get("ANDROID_HOME") or os.environ.get("ANDROID_SDK_ROOT")
    if not sdk_env:
        print("ERROR: set ANDROID_HOME to the Android SDK", file=sys.stderr)
        return 1
    sdk = Path(sdk_env)
    min_sdk, compile_sdk = gradle_int("minSdk"), gradle_int("compileSdk")
    api_xml = sdk / "platforms" / f"android-{compile_sdk}" / "data" / "api-versions.xml"
    if not api_xml.is_file():
        print(f"ERROR: {api_xml} not found -- install platform android-{compile_sdk}",
              file=sys.stderr)
        return 1
    api = ApiDatabase(api_xml)
    backports = d8_backports(sdk, min_sdk)

    failures: list[str] = []
    ours: dict[str, ClassFile] = {}
    module_of: dict[str, str] = {}
    mods = [m.lstrip(":").replace(":", "/")
            for m in INCLUDE_RE.findall(SETTINGS.read_text())]
    if not mods:
        print(f"ERROR: no include(...) lines in {SETTINGS}; update INCLUDE_RE",
              file=sys.stderr)
        return 1
    for mod in mods:
        files = [f for d in CLASS_DIRS for f in sorted((ANDROID / mod / d).rglob("*.class"))]
        if not files:
            failures.append(
                f"{mod}: no compiled main classes under any of {', '.join(CLASS_DIRS)} "
                f"-- run `./gradlew test` first, or the build moved them; "
                f"update CLASS_DIRS")
        for f in files:
            try:
                cf = read_class(f.read_bytes())
            except (ValueError, KeyError, struct.error) as e:
                failures.append(f"{f.relative_to(ANDROID)}: not a class file "
                                f"this checker can read ({e})")
                continue
            ours[cf.name] = cf
            module_of[cf.name] = mod
        print(f"{mod:16s} {len(files):5d} classes")

    def resolve(owner: str, member: str) -> int | None:
        """The API level a reference to `owner.member` needs: None when our
        own code or a library supplies it, NEVER when Android lacks it."""
        cf = ours.get(owner)
        if cf is not None:
            if member in cf.declared or member.startswith("<init>"):
                return None
            for sup in cf.supers:
                level = resolve(sup, member)
                if level is not None and level != NEVER:
                    return level
            return None
        if owner in api.classes:
            level = api.member_level(owner, member)
            return NEVER if level is None else level
        return NEVER if api.is_platform(owner) else None

    def class_needs(name: str) -> int | None:
        if name.startswith("["):
            name = name.lstrip("[")
            if not name.startswith("L"):
                return None                  # a primitive array
            name = name[1:-1]                # an array's element class
        if name in ours or not api.is_platform(name):
            return None
        level = api.class_level(name)
        return NEVER if level is None else level

    found: list[tuple[str, str, str, str, int]] = []
    unused = {(c, r) for c, refs in GUARDED.items() for r in refs}
    for name, cf in sorted(ours.items()):
        top = name.split("$", 1)[0]
        needs = [(c, class_needs(c)) for c in cf.classes
                 if c not in DESUGARED]
        needs += [(f"{owner}#{member}", resolve(owner, member))
                  for owner, member in cf.members
                  if not owner.startswith("[") and owner not in DESUGARED]
        for ref, level in needs:
            if level is None or level <= min_sdk or ref in backports:
                continue
            if ref in GUARDED.get(top, ()):
                unused.discard((top, ref))
                continue
            found.append((module_of[name], top, cf.source, ref, level))

    for top, ref in sorted(unused):
        failures.append(f"GUARDED lists {ref} for {top}, which makes no such "
                        f"call above minSdk {min_sdk} -- remove the entry")
    for mod, top, source, ref, level in sorted(set(found)):
        why = (f"not on Android at any level (android-{compile_sdk})"
               if level == NEVER else f"needs API {level}")
        failures.append(f"{mod}: {source} ({top.replace('/', '.')}) -- "
                        f"{ref} {why}")

    if failures:
        sys.stdout.flush()                   # the table above, before the list
        print(f"\nFAIL -- minSdk is {min_sdk}", file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        if found:
            print(f"\n  {WHAT_TO_DO}", file=sys.stderr)
        return 1
    print(f"\nOK -- {len(ours)} classes across {len(mods)} modules call nothing "
          f"above minSdk {min_sdk} (android-{compile_sdk} api-versions.xml, "
          f"D8 {D8_BUILD_TOOLS} backports, {sum(map(len, GUARDED.values()))} "
          f"guarded calls)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
