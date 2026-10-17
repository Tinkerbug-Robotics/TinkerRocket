#!/usr/bin/env bash
# bitident.sh BASE [HEAD] -- is the Beetle's raw-GNSS filter bit-identical in two trees?
#
# The harness frozen at the commit named below (FROZEN) is built twice,
# through SRC_ROOT: against BASE's firmware sources and against HEAD's, with
# the vertical check and its latch compiled in (the design the Beetle flies).
# Both replay every Beetle events file in the inputs directory, in act
# (--heading-mode 2) and observe (1), at magnetometer errors 0, 10 and 40 deg,
# with the firmware's own defaults (--fw-defaults: what startRawShadow() builds
# on the M1 without bench flags), and write a digest: a line of hashes per
# filter step over a fixed list of named fields (raw_replay.cpp, --digest).
# Each run is made without a clock (each epoch worked in one step) and with
# the emulated FC clock at 100 and 300 us a reading (--fc-unit-us), so epochs
# are worked a slice a step as on the M1. Any byte of difference in any run
# fails.
#
# The emulated clock counts readings: a tree that reads the clock more or
# less often on the M1's path slices its epochs differently and can differ in
# those legs alone. Read such a difference; it is not noise.
#
# The harness is frozen so that later harness work, which may need the new
# tree's interfaces, never has to compile against BASE.
#
# BASE and HEAD are commits (archived, so only what is committed counts) or
# directories holding a source tree (a worktree's path tests what is not
# committed yet). A commit is named as the caller's repository sees it: HEAD,
# the default for HEAD, is the HEAD of the worktree the script is run from,
# not of the one holding the script (outside any worktree of this repository,
# the script's own).
#
# Environment:
#   BITIDENT_IN       the events files (*.txt with a META t_base_s line) and an optional recorded.txt
#                     ("tag deg": the magnetometer error each log's image was built with; 0 if not listed).
#                     Default: gnss-attitude-study/rig-replay/bitident-in beside the repository.
#   BITIDENT_TMP      the scratch directory (default: a new one under $TMPDIR); kept for inspection.
#   BITIDENT_HARNESS  the commit whose tools/raw_replay is built (default: the frozen one below).
#   BITIDENT_MAGS     the magnetometer errors, deg (default "0 10 40").
#   BITIDENT_UNITS    the emulated FC clock's cost of a reading, us; 0 is no clock (default "0 100 300").
#   BITIDENT_JOBS     runs at once (default: the CPU count).
#   BITIDENT_EXTRA_SRC, BITIDENT_EXTRA_INC
#                     ';'-separated source files and include directories, relative to the tree, that a later
#                     tree's filter needs and the frozen CMakeLists does not name; each is used where it exists.
#
# Exit status: 0 identical, 1 a difference, 2 the check could not run (a build or a run failed).
set -euo pipefail

FROZEN=771e837449dd1ed5315a6c8376cf4f12b7d8aeee
FROZEN_SUBJECT="the frozen harness for the Beetle's bit-identity check"

if [ $# -lt 1 ] || [ $# -gt 2 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
    exit 2
fi
BASE_ARG=$1
HEAD_ARG=${2:-HEAD}

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(git -C "$HERE" rev-parse --show-toplevel)
COMMON=$(cd "$(git -C "$HERE" rev-parse --path-format=absolute --git-common-dir)" && pwd -P)
IN=${BITIDENT_IN:-$(dirname "$COMMON")/../gnss-attitude-study/rig-replay/bitident-in}
MAGS=${BITIDENT_MAGS:-"0 10 40"}
UNITS=${BITIDENT_UNITS:-"0 100 300"}
JOBS=${BITIDENT_JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)}
TMP=${BITIDENT_TMP:-$(mktemp -d "${TMPDIR:-/tmp}/bitident.XXXXXX")}
die() { echo "bitident: $*" >&2; exit 2; }

mkdir -p "$TMP"
TMP=$(cd "$TMP" && pwd)
[ -z "$(ls -A "$TMP")" ] || die "the scratch directory $TMP is not empty: give BITIDENT_TMP a new one"

[ -d "$IN" ] || die "no inputs directory $IN (set BITIDENT_IN)"
IN=$(cd "$IN" && pwd)

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

# The frozen harness: the given commit, else the one this script names, else
# (after a rebase rewrote it) the newest commit whose message carries its subject.
if [ -n "${BITIDENT_HARNESS:-}" ]; then
    harness=$(resolve "$BITIDENT_HARNESS") || die "no commit $BITIDENT_HARNESS"
elif git -C "$REPO" cat-file -e "$FROZEN^{commit}" 2>/dev/null; then
    harness=$FROZEN
else
    harness=$(git -C "$REPO" log --format=%H -1 -F --grep="$FROZEN_SUBJECT" HEAD || true)
fi
[ -n "$harness" ] || die "the frozen harness $FROZEN is not in this repository: set BITIDENT_HARNESS"
mkdir -p "$TMP/harness"
git -C "$REPO" archive "$harness" tools/raw_replay/CMakeLists.txt tools/raw_replay/raw_replay.cpp |
    tar -x -C "$TMP/harness"

# A tree to build against: a commit's firmware sources archived, or a directory as it is.
tree() {
    local side=$1 arg=$2
    if [ -d "$arg" ]; then
        local d
        d=$(cd "$arg" && pwd)
        [ -d "$d/tinkerrocket-idf/components/TR_RawGnssShadow" ] || die "$arg is not a source tree"
        echo "$side: the directory $d" >&2
        echo "$d"
        return
    fi
    local rev
    rev=$(resolve "$arg") || die "$arg is neither a commit nor a directory"
    mkdir -p "$TMP/$side"
    git -C "$REPO" archive "$rev" tinkerrocket-idf/components tinkerrocket-idf/projects/flight_computer/main \
        tests_cpp/host_shim | tar -x -C "$TMP/$side"
    echo "$side: $arg in ${CALLER:-$REPO} = $(git -C "$REPO" log -1 --format='%h %s' "$rev" | cut -c1-100)" >&2
    echo "$TMP/$side"
}

# The extra sources and include directories that exist in a tree, as CMake lists.
extra() {
    local root=$1 list=$2 out="" p
    local IFS=';'
    for p in $list; do
        [ -n "$p" ] && [ -e "$root/$p" ] && out="${out:+$out;}$p"
    done
    echo "$out"
}

build() {
    local side=$1 root=$2
    echo "building the frozen harness against $side..." >&2
    if ! { cmake -S "$TMP/harness/tools/raw_replay" -B "$TMP/build_$side" -DSRC_ROOT="$root" \
               -DCMAKE_BUILD_TYPE=Release -DRAW_REPLAY_VCHECK=ON -DRAW_REPLAY_LATCH=ON \
               -DRAW_REPLAY_EXTRA_SRC="$(extra "$root" "${BITIDENT_EXTRA_SRC:-}")" \
               -DRAW_REPLAY_EXTRA_INC="$(extra "$root" "${BITIDENT_EXTRA_INC:-}")" &&
           cmake --build "$TMP/build_$side" -j "$JOBS"; } > "$TMP/build_$side.log" 2>&1; then
        tail -30 "$TMP/build_$side.log" >&2
        die "the frozen harness does not build against $side (log: $TMP/build_$side.log)"
    fi
}

echo "frozen harness: $(git -C "$REPO" log -1 --format='%h %s' "$harness" | cut -c1-100)"
base_root=$(tree base "$BASE_ARG")
head_root=$(tree head "$HEAD_ARG")
build base "$base_root"
build head "$head_root"

# One run: SIDE TAG CONFIG MAG UNIT. The CSV is not wanted (--every keeps it to its header).
run_one() {
    local side=$1 tag=$2 cfg=$3 mag=$4 unit=$5
    local f="$IN/$tag.txt" mode=2
    [ "$cfg" = observe ] && mode=1
    local tb rec clock=()
    tb=$(awk '$1=="META" && $2=="t_base_s" {print $3; exit}' "$f")
    rec=$( { [ -f "$IN/recorded.txt" ] && awk -v t="$tag" '$1==t {print $2; exit}' "$IN/recorded.txt"; } || true)
    [ "$unit" = 0 ] || clock=(--fc-unit-us "$unit")
    local out="$TMP/run_$side/${tag}_${cfg}_m${mag}_u${unit}"
    if "$TMP/build_$side/raw_replay" --in "$f" --out "$out.csv" --every 1000000000 --t-base-s "$tb" \
           --fw-defaults --heading-mode "$mode" --mag-error-deg "$mag" --recorded-mag-error-deg "${rec:-0}" \
           ${clock[@]+"${clock[@]}"} --digest "$out.digest" 2> "$out.err" &&
       grep -q '^end ' "$out.digest"; then
        echo 0 > "$out.rc"
    else
        echo 1 > "$out.rc"
    fi
}
export -f run_one
export IN TMP

mkdir -p "$TMP/run_base" "$TMP/run_head"
tags=""
for f in "$IN"/*.txt; do
    grep -q '^META t_base_s ' "$f" 2>/dev/null || continue
    tags="$tags $(basename "$f" .txt)"
done
[ -n "$tags" ] || die "no events files in $IN"
ntags=$(echo $tags | wc -w | tr -d ' ')
echo "inputs: $ntags events files in $IN; magnetometer errors $MAGS deg; act and observe;" \
     "FC clock $UNITS us a reading (0: none)"
for side in base head; do
    for tag in $tags; do
        for cfg in act observe; do
            for mag in $MAGS; do
                for unit in $UNITS; do
                    printf '%s\0%s\0%s\0%s\0%s\0' "$side" "$tag" "$cfg" "$mag" "$unit"
                done
            done
        done
    done
done | xargs -0 -n 5 -P "$JOBS" bash -c 'run_one "$@"' _

# Compare: every run must have run on both sides, and match byte for byte.
same=0
differ=0
failed=0
groups="step time nav rec_0x96 att_0x9B heading_fit pad_context stats state"
runs=""
for tag in $tags; do
    for cfg in act observe; do
        for mag in $MAGS; do
            for unit in $UNITS; do
                runs="$runs ${tag}_${cfg}_m${mag}_u${unit}"
            done
        done
    done
done
for k in $runs; do
    b="$TMP/run_base/$k"
    h="$TMP/run_head/$k"
    if [ "$(cat "$b.rc")" != 0 ] || [ "$(cat "$h.rc")" != 0 ]; then
        failed=$((failed + 1))
        echo "FAILED  $k (base rc $(cat "$b.rc"), head rc $(cat "$h.rc"); stderr in $b.err, $h.err)"
        continue
    fi
    if cmp -s "$b.digest" "$h.digest"; then
        same=$((same + 1))
        continue
    fi
    differ=$((differ + 1))
    echo "$k" >> "$TMP/differ_u${k##*_u}"
    # the first line that differs, and which of its hashes do
    n=$({ cmp "$b.digest" "$h.digest" 2>/dev/null || true; } | sed -n 's/.* line \([0-9]*\).*/\1/p')
    n=${n:-1}
    lb=$(sed -n "${n}p" "$b.digest")
    lh=$(sed -n "${n}p" "$h.digest")
    what=$(awk -v a="$lb" -v b="$lh" -v g="$groups" 'BEGIN {
        na = split(a, x, " "); nb = split(b, y, " "); split(g, name, " ")
        if (na != nb || x[1] == "end" || y[1] == "end") { print "the step count (one stopped or ran longer)"; exit }
        s = ""; for (i = 1; i <= na; i++) if (x[i] != y[i]) s = s (s ? ", " : "") name[i]
        print s }')
    [ -n "$what" ] || what="the step count (one digest is longer)"
    echo "DIFFERS $k: from step $(echo "$lb" | cut -d' ' -f1) (digest line $n): $what"
done
total=$((same + differ + failed))
echo "bitident: $total runs, $same identical, $differ differ, $failed failed (scratch: $TMP)"
if [ "$differ" -gt 0 ]; then
    # differences with the clock alone are the slicing's: the clock read more or less often, or a slice's work
    tally=""
    for unit in $UNITS; do
        nd=0
        [ ! -f "$TMP/differ_u$unit" ] || nd=$(wc -l < "$TMP/differ_u$unit" | tr -d ' ')
        tally="${tally:+$tally, }$([ "$unit" = 0 ] && echo "no clock" || echo "$unit us a reading") $nd"
    done
    echo "bitident: the runs that differ, by FC clock: $tally"
fi
if [ "$failed" -gt 0 ]; then
    exit 2
fi
if [ "$differ" -gt 0 ]; then
    exit 1
fi
echo "bitident: IDENTICAL"
