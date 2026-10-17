#!/usr/bin/env bash
# m1_identity.sh BASE [HEAD] -- does the Beetle's flight computer (M1) compile the same main.cpp in two trees?
#
# The mosaic-G5 work adds to main.cpp, config.h, the other files of the
# flight computer's main/ and the sensor collectors behind switches that are
# 0 on the M1. This checks that the M1 sees none of it: both trees are
# configured for the M1 (idf.py reconfigure, -DTR_BOARD_M1=1), and the compile
# command of each file below is re-run with -E (line markers kept) on each
# tree:
#
#   tinkerrocket-idf/projects/flight_computer/main/main.cpp
#   tinkerrocket-idf/components/TR_Sensor_Collector/TR_Sensor_Collector.cpp
#   tinkerrocket-idf/components/TR_Sensor_Collector_Sim/TR_Sensor_Collector_Sim.cpp
#
# Of the output, only the lines of the file itself and of every file under
# tinkerrocket-idf/projects/flight_computer/main/ it includes (config.h,
# board/*.h, raw_nav_adapter.h, the *_policy.h ...) are kept, file by file:
# those whose preceding line marker names that file. Other headers are not
# kept (shared headers gain defaulted fields and methods by design; the
# filter's behavior is bitident.sh's job). Blank lines go, so a block compiled
# out is no difference, and a file with no lines left is none either; the tree
# roots, FW_GIT_SHA (and FW_GIT_DIRTY) and every integer __LINE__ produced are
# normalized, so lines moved by such a block are none either. Then each kept
# file's lines are diffed, and so are the TR_* and CONFIG_* macros main.cpp
# sees at its end (-dM). Any difference fails.
#
# __LINE__'s integers are found exactly: each compile is run a second time
# from a copy of the file and of main/ in which every file starts with
# #line 1000001 under its own name, and an integer that moved by 1000000
# between the two came from __LINE__.
#
# BASE and HEAD are commits (archived) or directories holding a source tree
# (copied, without build directories). A commit is named as the caller's
# repository sees it: HEAD, the default for HEAD, is the HEAD of the worktree
# the script is run from, not of the one holding the script (outside any
# worktree of this repository, the script's own). Either way the configure
# runs on a copy in the scratch directory, so no tree's dependencies.lock or
# managed_components is touched. The private TR_GuidancePN submodule is never
# copied: both sides build with its stub.
#
# Not compared: compile flags other than the macros above, linker fragments
# (main/*.lf), CMake files, and component sources other than the collectors.
#
# Environment:
#   M1_IDENTITY_TMP  the scratch directory (default: a new one under $TMPDIR); kept for inspection.
#   IDF_EXPORT       ESP-IDF's export.sh, sourced when idf.py is not on the PATH
#                    (default ~/esp/esp-idf-v6.0/export.sh).
#
# Exit status: 0 identical, 1 a difference, 2 the check could not run.
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 2 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
    exit 2
fi
BASE_ARG=$1
HEAD_ARG=${2:-HEAD}

die() { echo "m1_identity: $*" >&2; exit 2; }

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(git -C "$HERE" rev-parse --show-toplevel)
COMMON=$(cd "$(git -C "$HERE" rev-parse --path-format=absolute --git-common-dir)" && pwd -P)

# A commit is named as the caller's worktree sees it, so HEAD is the caller's
# HEAD and not this script's worktree's; a caller in no worktree of this
# repository names it as this script's worktree does. The object store is
# shared, so REPO archives the commit either way.
CALLER=$(git rev-parse --show-toplevel 2>/dev/null) || CALLER=""
if [ -n "$CALLER" ] &&
   [ "$(cd "$(git rev-parse --path-format=absolute --git-common-dir)" && pwd -P)" != "$COMMON" ]; then
    CALLER=""   # another repository's worktree
fi
resolve() {
    git -C "${CALLER:-$REPO}" rev-parse --verify --quiet "$1^{commit}"
}
TMP=${M1_IDENTITY_TMP:-$(mktemp -d "${TMPDIR:-/tmp}/m1_identity.XXXXXX")}
mkdir -p "$TMP"
TMP=$(cd "$TMP" && pwd -P)
[ -z "$(ls -A "$TMP")" ] || die "the scratch directory $TMP is not empty: give M1_IDENTITY_TMP a new one"

if ! command -v idf.py > /dev/null 2>&1; then
    export_sh=${IDF_EXPORT:-$HOME/esp/esp-idf-v6.0/export.sh}
    [ -f "$export_sh" ] || die "idf.py is not on the PATH and there is no $export_sh (set IDF_EXPORT)"
    # shellcheck disable=SC1090
    . "$export_sh" > /dev/null 2>&1 || die "sourcing $export_sh failed"
fi

# A tree in the scratch directory: a commit's tinkerrocket-idf archived, or a directory's copied.
tree() {
    local side=$1 arg=$2
    local d="$TMP/$side"
    mkdir -p "$d"
    if [ -d "$arg" ]; then
        local src
        src=$(cd "$arg" && pwd)
        [ -d "$src/tinkerrocket-idf/projects/flight_computer" ] || die "$arg is not a source tree"
        rsync -a --exclude '.git' --exclude 'build/' --exclude 'build_*/' --exclude 'managed_components/' \
            --exclude '/components/TR_GuidancePN/' "$src/tinkerrocket-idf/" "$d/tinkerrocket-idf/"
        echo "$side: a copy of the directory $src" >&2
    else
        local rev
        rev=$(resolve "$arg") || die "$arg is neither a commit nor a directory"
        git -C "$REPO" archive "$rev" tinkerrocket-idf | tar -x -C "$d"
        echo "$side: $arg in ${CALLER:-$REPO} = $(git -C "$REPO" log -1 --format='%h %s' "$rev" | cut -c1-100)" >&2
    fi
}

configure() {
    local side=$1
    echo "configuring the M1 flight computer ($side)..." >&2
    if ! idf.py -C "$TMP/$side/tinkerrocket-idf/projects/flight_computer" -B "$TMP/m1_$side" -DTR_BOARD_M1=1 \
             reconfigure > "$TMP/m1_$side.log" 2>&1; then
        tail -30 "$TMP/m1_$side.log" >&2
        die "idf.py reconfigure failed for $side (log: $TMP/m1_$side.log)"
    fi
    [ -f "$TMP/m1_$side/compile_commands.json" ] || die "no compile_commands.json in $TMP/m1_$side"
}

tree base "$BASE_ARG"
tree head "$HEAD_ARG"
configure base
configure head

# Each file preprocessed, the lines of it and of the main/ files it includes kept and normalized:
# OUT/<file>/<kept file>.own, and main.cpp's macros: OUT/main.cpp.dM
cat > "$TMP/own_lines.py" <<'PY'
import json, os, re, shlex, subprocess, sys

root, build, out = sys.argv[1], sys.argv[2], sys.argv[3]
FILES = ["tinkerrocket-idf/projects/flight_computer/main/main.cpp",
         "tinkerrocket-idf/components/TR_Sensor_Collector/TR_Sensor_Collector.cpp",
         "tinkerrocket-idf/components/TR_Sensor_Collector_Sim/TR_Sensor_Collector_Sim.cpp"]
MAIN = "tinkerrocket-idf/projects/flight_computer/main"
SOURCES = (".h", ".hh", ".hpp", ".hxx", ".inc", ".inl", ".ipp", ".def", ".c", ".cc", ".cpp", ".cxx")
OFFSET = 1000000
MARKER = re.compile(r'^# (\d+) "((?:[^"\\]|\\.)*)"')
# the build stamps, the same on both sides
STAMPS = ["-UFW_GIT_SHA", '-DFW_GIT_SHA="FW_GIT_SHA"', "-UFW_GIT_DIRTY", "-DFW_GIT_DIRTY=0"]

def fail(msg):
    sys.stderr.write("m1_identity: " + msg + "\n")
    sys.exit(2)

def command(entry, src, extra):
    args = entry.get("arguments") or shlex.split(entry["command"])
    keep, skip = [], False
    for a in args:
        if skip:
            skip = False
            continue
        if a in ("-o", "-MF", "-MT", "-MQ"):
            skip = True
            continue
        if a in ("-c", "-MD", "-MMD", "-MP") or a == entry["file"]:
            continue
        keep.append(a)
    return keep + STAMPS + extra + [src]

def run(argv, cwd):
    p = subprocess.run(argv, cwd=cwd, capture_output=True, text=True)
    if p.returncode != 0:
        fail("preprocessing failed: %s\n%s" % (" ".join(argv[-3:]), p.stderr[-3000:]))
    return p.stdout

def unquote(name):
    return re.sub(r"\\(.)", r"\1", name)

def own(text, key):
    # the kept files' lines, each under its key, in order; and the name each was first marked by
    cur, lines, names = None, {}, {}
    for ln in text.split("\n"):
        m = MARKER.match(ln)
        if m:
            name = unquote(m.group(2))
            cur = key(name)
            if cur is not None:
                names.setdefault(cur, name)
            continue
        if cur is not None and ln.strip():
            lines.setdefault(cur, []).append(ln.rstrip())
    return lines, names

def normalize(s, roots):
    for r, tag in roots:
        s = s.replace(r, tag)
    return s

db = json.load(open(os.path.join(build, "compile_commands.json")))
idf = os.environ.get("IDF_PATH", "")
roots = [(os.path.realpath(build), "<BUILD>"), (build, "<BUILD>"), (os.path.realpath(root), "<ROOT>"),
         (root, "<ROOT>")] + ([(os.path.realpath(idf), "<IDF>"), (idf, "<IDF>")] if idf else [])
main_dir = os.path.realpath(os.path.join(root, MAIN))
for rel in FILES:
    entry = [e for e in db if os.path.realpath(e["file"]) == os.path.realpath(os.path.join(root, rel))]
    if len(entry) != 1:
        fail("%s: %d compile commands in %s" % (rel, len(entry), build))
    e = entry[0]
    name, cwd = e["file"], e["directory"]
    base = os.path.basename(rel)
    real = os.path.realpath(os.path.join(cwd, name))
    seen = {}

    def key(n):
        # the file itself, or a file under main/ (by its path there); None for every other
        if n not in seen:
            rp = os.path.realpath(os.path.join(cwd, n))
            if rp == real and not rp.startswith(main_dir + os.sep):
                seen[n] = base
            elif rp.startswith(main_dir + os.sep) and os.path.isfile(rp):
                seen[n] = os.path.relpath(rp, main_dir)
            else:
                seen[n] = None
        return seen[n]

    text = run(command(e, name, ["-E"]), cwd)
    a, names = own(text, key)
    # The same from a copy numbered from 1000001 under each file's own name (as the first run's markers
    # spelled it): the file, and for main.cpp every source file of main/ beside it, so its quoted includes
    # find their copies first (the includer's directory), and anything else the file's directory (-iquote).
    shifted_dir = os.path.join(out, "shifted", base)
    copies = []
    if real.startswith(main_dir + os.sep):
        for d, _, fs in os.walk(main_dir):
            for f in fs:
                if f.endswith(SOURCES):
                    src = os.path.join(d, f)
                    copies.append((src, os.path.join(shifted_dir, os.path.relpath(src, main_dir))))
        shifted = os.path.join(shifted_dir, os.path.relpath(real, main_dir))
    else:
        shifted = os.path.join(shifted_dir, base)   # its own name: the driver goes by the extension
        copies.append((real, shifted))
    for src, dst in copies:
        k = base if src == real and not real.startswith(main_dir + os.sep) else os.path.relpath(src, main_dir)
        spelled = names.get(k, name if src == real else src)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "w", encoding="utf-8", errors="surrogateescape") as f:
            f.write('#line %d "%s"\n' % (OFFSET + 1, spelled.replace("\\", "\\\\").replace('"', '\\"')))
            f.write(open(src, encoding="utf-8", errors="surrogateescape").read())
    text2 = run(command(e, shifted, ["-E", "-iquote", os.path.dirname(real)]), cwd)
    b, _ = own(text2, key)
    me = base if base in a else os.path.relpath(real, main_dir)
    if not a.get(me):
        fail("%s: no lines of its own in its preprocessed output" % rel)
    if sorted(a) != sorted(b):
        fail("%s: the shifted copy keeps %s, the file %s" % (rel, sorted(b), sorted(a)))
    for k in sorted(a):
        if len(a[k]) != len(b[k]):
            fail("%s: %s: the shifted copy gives %d lines, the file %d" % (rel, k, len(b[k]), len(a[k])))
        res = []
        for la, lb in zip(a[k], b[k]):
            pa, pb = re.split(r"(\d+)", la), re.split(r"(\d+)", lb)
            if len(pa) != len(pb):
                fail("%s: %s: cannot align a line with its shifted copy:\n  %s\n  %s" % (rel, k, la, lb))
            for i in range(len(pa)):
                if pa[i] == pb[i]:
                    continue
                if i % 2 == 1 and int(pb[i]) - int(pa[i]) == OFFSET:
                    pa[i] = "__LINE__"
                else:
                    fail("%s: %s: a line differs from its shifted copy other than by __LINE__:\n  %s\n  %s"
                         % (rel, k, la, lb))
            res.append(normalize("".join(pa), roots))
        path = os.path.join(out, base, k + ".own")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write("\n".join(res) + "\n")
        print("%s: %s: %d lines" % (rel, k, len(res)))
    if base == "main.cpp":
        dm = run(command(e, name, ["-E", "-dM"]), cwd)
        defs = sorted(normalize(ln, roots) for ln in dm.split("\n") if re.match(r"#define (TR_|CONFIG_)", ln))
        with open(os.path.join(out, base + ".dM"), "w") as f:
            f.write("\n".join(defs) + "\n")
        print("%s: %d TR_*/CONFIG_* macros" % (rel, len(defs)))
PY
for side in base head; do
    mkdir -p "$TMP/out_$side"
    echo "$side:" >&2
    python3 -I "$TMP/own_lines.py" "$TMP/$side" "$TMP/m1_$side" "$TMP/out_$side" >&2
done

# Every kept file on either side: one only one side keeps is a difference too.
status=0
: > "$TMP/m1_identity.diff"
kept=$( { (cd "$TMP/out_base" && find . -path ./shifted -prune -o -type f \( -name '*.own' -o -name '*.dM' \) -print)
          (cd "$TMP/out_head" && find . -path ./shifted -prune -o -type f \( -name '*.own' -o -name '*.dM' \) -print)
        } | sed 's|^\./||' | sort -u)
for f in $kept; do
    b="$TMP/out_base/$f"
    h="$TMP/out_head/$f"
    note=""
    [ -f "$b" ] || { b=/dev/null; note=" (head only)"; }
    [ -f "$h" ] || { h=/dev/null; note=" (base only)"; }
    if diff -u --label "base/$f" --label "head/$f" "$b" "$h" >> "$TMP/m1_identity.diff"; then
        echo "same     $f"
    else
        status=1
        echo "DIFFERS  $f$note"
    fi
done
if [ "$status" -ne 0 ]; then
    echo "--- the differences (all of them: $TMP/m1_identity.diff)"
    head -80 "$TMP/m1_identity.diff"
    echo "m1_identity: DIFFERENT (scratch: $TMP)"
    exit 1
fi
echo "m1_identity: IDENTICAL (scratch: $TMP)"
