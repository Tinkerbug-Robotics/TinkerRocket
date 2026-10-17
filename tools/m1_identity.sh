#!/usr/bin/env bash
# m1_identity.sh BASE [HEAD] -- does the Beetle's flight computer (M1) compile the same main.cpp in two trees?
#
# The mosaic-G5 work adds to main.cpp, config.h and the sensor collectors
# behind switches that are 0 on the M1. This checks that the M1 sees none of
# it: both trees are configured for the M1 (idf.py reconfigure,
# -DTR_BOARD_M1=1), and the compile command of each file below is re-run with
# -E (line markers kept) on each tree:
#
#   tinkerrocket-idf/projects/flight_computer/main/main.cpp
#   tinkerrocket-idf/components/TR_Sensor_Collector/TR_Sensor_Collector.cpp
#   tinkerrocket-idf/components/TR_Sensor_Collector_Sim/TR_Sensor_Collector_Sim.cpp
#
# Of the output, only the file's own lines are kept: those whose preceding
# line marker names that file, not its headers (shared headers gain defaulted
# fields and methods by design; the filter's behavior is bitident.sh's job).
# Blank lines go, so a block compiled out is no difference; the tree roots,
# FW_GIT_SHA (and FW_GIT_DIRTY) and every integer __LINE__ produced are
# normalized, so lines moved by such a block are none either. Then the own
# lines are diffed, and so are the TR_* and CONFIG_* macros main.cpp sees at its
# end (-dM). Any difference fails.
#
# __LINE__'s integers are found exactly: each file is preprocessed a second
# time from a copy that starts with #line 1000001 under the file's own name,
# and an integer that moved by 1000000 between the two came from __LINE__.
#
# BASE and HEAD are commits (archived) or directories holding a source tree
# (copied, without build directories). Either way the configure runs on a
# copy in the scratch directory, so no tree's dependencies.lock or
# managed_components is touched. The private TR_GuidancePN submodule is never
# copied: both sides build with its stub. HEAD defaults to HEAD.
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
        rev=$(git -C "$REPO" rev-parse --verify "$arg^{commit}" 2>/dev/null) || die "$arg is neither a commit nor a directory"
        git -C "$REPO" archive "$rev" tinkerrocket-idf | tar -x -C "$d"
        echo "$side: $arg = $(git -C "$REPO" log -1 --format='%h %s' "$rev" | cut -c1-100)" >&2
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

# Each file preprocessed, its own lines kept and normalized: OUT/<name>.own, and main.cpp's macros: OUT/main.cpp.dM
cat > "$TMP/own_lines.py" <<'PY'
import json, os, re, shlex, subprocess, sys

root, build, out = sys.argv[1], sys.argv[2], sys.argv[3]
FILES = ["tinkerrocket-idf/projects/flight_computer/main/main.cpp",
         "tinkerrocket-idf/components/TR_Sensor_Collector/TR_Sensor_Collector.cpp",
         "tinkerrocket-idf/components/TR_Sensor_Collector_Sim/TR_Sensor_Collector_Sim.cpp"]
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

def own(text, name):
    cur, lines = None, []
    for ln in text.split("\n"):
        m = MARKER.match(ln)
        if m:
            cur = m.group(2)
            continue
        if cur == name and ln.strip():
            lines.append(ln.rstrip())
    return lines

def normalize(s, roots):
    for r, tag in roots:
        s = s.replace(r, tag)
    return s

db = json.load(open(os.path.join(build, "compile_commands.json")))
idf = os.environ.get("IDF_PATH", "")
roots = [(os.path.realpath(build), "<BUILD>"), (build, "<BUILD>"), (os.path.realpath(root), "<ROOT>"),
         (root, "<ROOT>")] + ([(os.path.realpath(idf), "<IDF>"), (idf, "<IDF>")] if idf else [])
for rel in FILES:
    entry = [e for e in db if os.path.realpath(e["file"]) == os.path.realpath(os.path.join(root, rel))]
    if len(entry) != 1:
        fail("%s: %d compile commands in %s" % (rel, len(entry), build))
    e = entry[0]
    name = e["file"]
    base = os.path.basename(rel)
    text = run(command(e, name, ["-E"]), e["directory"])
    # the same, from a copy numbered from 1000001 under the file's own name: quoted includes still
    # resolve from the file's directory (-iquote), as they do for the file itself
    os.makedirs(os.path.join(out, "shifted"), exist_ok=True)
    shifted = os.path.join(out, "shifted", base)   # its own name: the driver goes by the extension
    with open(shifted, "w") as f:
        f.write('#line %d "%s"\n' % (OFFSET + 1, name.replace("\\", "\\\\").replace('"', '\\"')))
        f.write(open(name, encoding="utf-8", errors="surrogateescape").read())
    text2 = run(command(e, shifted, ["-E", "-iquote", os.path.dirname(name)]), e["directory"])
    a, b = own(text, name), own(text2, name)
    if len(a) != len(b) or not a:
        fail("%s: the shifted copy gives %d own lines, the file %d" % (rel, len(b), len(a)))
    res = []
    for la, lb in zip(a, b):
        pa, pb = re.split(r"(\d+)", la), re.split(r"(\d+)", lb)
        if len(pa) != len(pb):
            fail("%s: cannot align a line with its shifted copy:\n  %s\n  %s" % (rel, la, lb))
        for i in range(len(pa)):
            if pa[i] == pb[i]:
                continue
            if i % 2 == 1 and int(pb[i]) - int(pa[i]) == OFFSET:
                pa[i] = "__LINE__"
            else:
                fail("%s: a line differs from its shifted copy other than by __LINE__:\n  %s\n  %s" % (rel, la, lb))
        res.append(normalize("".join(pa), roots))
    with open(os.path.join(out, base + ".own"), "w") as f:
        f.write("\n".join(res) + "\n")
    print("%s: %d own lines" % (rel, len(res)))
    if base == "main.cpp":
        dm = run(command(e, name, ["-E", "-dM"]), e["directory"])
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

status=0
: > "$TMP/m1_identity.diff"
for f in main.cpp.own TR_Sensor_Collector.cpp.own TR_Sensor_Collector_Sim.cpp.own main.cpp.dM; do
    if diff -u --label "base/$f" --label "head/$f" "$TMP/out_base/$f" "$TMP/out_head/$f" >> "$TMP/m1_identity.diff"; then
        echo "same     $f"
    else
        status=1
        echo "DIFFERS  $f"
    fi
done
if [ "$status" -ne 0 ]; then
    echo "--- the differences (all of them: $TMP/m1_identity.diff)"
    head -80 "$TMP/m1_identity.diff"
    echo "m1_identity: DIFFERENT (scratch: $TMP)"
    exit 1
fi
echo "m1_identity: IDENTICAL (scratch: $TMP)"
