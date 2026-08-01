#!/usr/bin/env bash
# Plot a board's gerbers + drill files with git provenance recorded alongside.
#
# The source .kicad_pcb is never modified. The board's marketing version lives in
# its title block as (rev "V3") and reaches the silkscreen through ${REVISION};
# this script only checks that it agrees with the git tag, and writes the full
# descriptor into a README.txt beside the gerbers.
#
#   tools/plot_gerbers.sh <board> [--allow-dirty] [--outdir DIR]
#
# See docs/board-versioning.md for what major/minor/patch mean for a PCB.
set -euo pipefail

KICAD_CLI="${KICAD_CLI:-/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

board="${1:-}"; shift || true
allow_dirty=0; outdir=""
while [ $# -gt 0 ]; do
  case "$1" in
    --allow-dirty) allow_dirty=1 ;;
    --outdir) outdir="$2"; shift ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac; shift
done

if [ -z "$board" ]; then
  echo "usage: $(basename "$0") <board> [--allow-dirty] [--outdir DIR]" >&2
  echo "boards:" >&2; ls -1 "$REPO/hardware" | while read -r d; do
    [ -f "$REPO/hardware/$d/$d.kicad_pcb" ] && echo "  $d" >&2; done
  exit 2
fi

pcb="$REPO/hardware/$board/$board.kicad_pcb"
[ -f "$pcb" ] || { echo "no such board: $board" >&2; exit 2; }
[ -x "$KICAD_CLI" ] || { echo "kicad-cli not found at $KICAD_CLI" >&2; exit 2; }
outdir="${outdir:-$REPO/hardware/$board/gerbers}"

cd "$REPO"

# ---- provenance ------------------------------------------------------------
dirty=""
if ! git diff --quiet -- "hardware/$board" || ! git diff --cached --quiet -- "hardware/$board"; then
  dirty="-dirty"
fi
describe="$(git describe --tags --match "${board}-v*" --always --dirty 2>/dev/null || git rev-parse --short HEAD)"
commit="$(git rev-parse HEAD)"
tag_ver="$(git describe --tags --match "${board}-v*" --abbrev=0 2>/dev/null || echo '<untagged>')"

if [ -n "$dirty" ] && [ "$allow_dirty" -eq 0 ]; then
  echo "REFUSING: hardware/$board has uncommitted changes." >&2
  echo "  Gerbers plotted now could not be reproduced from any commit." >&2
  echo "  Commit first, or re-run with --allow-dirty to plot anyway." >&2
  exit 1
fi

# ---- silkscreen version vs tag --------------------------------------------
rev="$(sed -n 's/.*(rev "\([^"]*\)").*/\1/p' "$pcb" | head -1)"
if [ -z "$rev" ]; then
  echo "WARNING: $board has no (rev \"...\") in its title block — silkscreen \${REVISION} will be blank." >&2
elif [ "$tag_ver" != "<untagged>" ]; then
  # tag  <board>-v3.1.0  should agree with rev  V3.1  (or V3)
  semver="${tag_ver#${board}-v}"
  major="${semver%%.*}"; rest="${semver#*.}"; minor="${rest%%.*}"
  if [ "$rev" != "V${major}" ] && [ "$rev" != "V${major}.${minor}" ]; then
    echo "WARNING: title-block rev '$rev' disagrees with tag '$tag_ver'." >&2
    echo "  The silkscreen will read '$rev'. Fix one of them before fabricating." >&2
  fi
fi

# ---- plot ------------------------------------------------------------------
mkdir -p "$outdir"
echo "plotting $board  rev=${rev:-<none>}  $describe"
"$KICAD_CLI" pcb export gerbers --output "$outdir/" "$pcb" >/dev/null
"$KICAD_CLI" pcb export drill   --output "$outdir/" "$pcb" >/dev/null

# ---- provenance record -----------------------------------------------------
cat > "$outdir/README.txt" <<EOF
TinkerRocket — $board
Silkscreen revision : ${rev:-<none>}
Git tag             : $tag_ver
Git describe        : $describe
Commit              : $commit
Plotted             : $(date -u '+%Y-%m-%dT%H:%M:%SZ')
Tool                : $("$KICAD_CLI" --version 2>/dev/null | head -1)
$( [ -n "$dirty" ] && echo "
WARNING: plotted from a DIRTY working tree. These files do not correspond
to any commit and cannot be reproduced. Do not fabricate from this set." )
EOF

echo "wrote $(ls -1 "$outdir" | wc -l | tr -d ' ') files to $outdir"
[ -n "$dirty" ] && echo "NOTE: dirty tree — see the warning in README.txt" >&2
exit 0
