#!/usr/bin/env bash
# Build PortaPack / HackRF GPS baseband files for the #491 COCOM tests.
#
# For each trajectory CSV this produces the pair the Mayhem GPS Sim app needs:
#
#   NAME.C8    signed 8-bit interleaved I/Q  (gps-sdr-sim -b 8)
#   NAME.TXT   center_frequency= / sample_rate=   (Mayhem metadata sidecar)
#
# The .C8 also plays straight out of hackrf_transfer; see the tail of the run.
#
# Usage:  ./build_scenarios.sh -e <ephemeris> [-o out] [-S scen] [-s rate]
#                              [-t YYYY/MM/DD,hh:mm:ss] [names...]
#
# The ephemeris may be RINEX 2 (brdc<DOY>0.<YY>n) or RINEX 3 mixed
# (BRDC..._MN.rnx); a RINEX 3 file is auto-converted with rinex3to2.py.

set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

EPH=""
OUTDIR="c8"
SCENDIR="scenarios"
RATE=2600000
CENTER=1575420000
START=""

usage() { sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    -e|--ephemeris) EPH="$2"; shift 2;;
    -o|--outdir)    OUTDIR="$2"; shift 2;;
    -S|--scenarios) SCENDIR="$2"; shift 2;;
    -s|--rate)      RATE="$2"; shift 2;;
    -t|--start)     START="$2"; shift 2;;
    -h|--help)      usage 0;;
    -*) echo "unknown option: $1" >&2; usage 1;;
    *)  break;;
  esac
done

command -v gps-sdr-sim >/dev/null || {
  cat >&2 <<'MSG'
gps-sdr-sim not found on PATH. Build it:

  git clone https://github.com/osqzss/gps-sdr-sim
  cd gps-sdr-sim && gcc -O3 -o gps-sdr-sim gpssim.c -lm
  sudo cp gps-sdr-sim /usr/local/bin/

Runs longer than 300 s additionally need -DUSER_MOTION_SIZE=<10*seconds>.
MSG
  exit 1
}

[[ -n "$EPH" ]] || { echo "-e <ephemeris> is required" >&2; usage 1; }
[[ -f "$EPH" ]] || { echo "ephemeris not found: $EPH" >&2; exit 1; }

# Absolute paths now, relative arg strings later -- see the CRITICAL note below.
EPH="$(cd "$(dirname "$EPH")" && pwd)/$(basename "$EPH")"
SCENDIR="$(cd "$SCENDIR" && pwd)"
mkdir -p "$OUTDIR"; OUTDIR="$(cd "$OUTDIR" && pwd)"

# --- RINEX 3 -> RINEX 2 ------------------------------------------------------
# gps-sdr-sim's nav reader is strictly RINEX 2 (fixed-column). The ephemeris
# sources that serve without a login publish RINEX 3 mixed; convert if so.
ver_line="$(head -1 "$EPH")"
if [[ "$ver_line" == *"3."*"RINEX VERSION"* || "$ver_line" == *"MIXED"* ]]; then
  conv="$OUTDIR/$(basename "${EPH%.*}").rx2.n"
  echo "converting RINEX 3 -> 2 ..."
  python3 "$HERE/rinex3to2.py" "$EPH" -o "$conv"
  EPH="$conv"
  echo
fi

# --- pick a start time in the ephemeris's dense window -----------------------
# Two traps this avoids, both of which silently cost satellites rather than
# erroring:
#   * A partial-day broadcast file (common from the no-login mirrors) is sparse
#     at the start of the day. Defaulting to noon can land on 8 SVs.
#   * gps-sdr-sim's -T (overwrite TOC/TOE) shifts every record relative to the
#     file's *earliest* epoch, so on a multi-set file it aligns the wrong set to
#     the start and drops the constellation to ~1 SV. We never pass -T; instead
#     we place the scenario start inside the file's own validity window, which
#     is what -T exists to avoid needing. A same-day ephemeris makes -T moot.
# Longest requested scenario, so the whole run fits before the file's last TOC:
max_dur=0
for name in "$@"; do
  d=$(awk -F, 'END{print int($1)+1}' "$SCENDIR/$name.csv" 2>/dev/null || echo 0)
  (( d > max_dur )) && max_dur=$d
done
[[ ${max_dur} -gt 0 ]] || max_dur=240

if [[ -z "$START" ]]; then
  START="$(python3 "$HERE/pick_start.py" "$EPH" --min-duration "$max_dur")" || {
    echo "could not pick a start time from $EPH; pass -t explicitly" >&2; exit 1; }
fi

echo "ephemeris : $EPH"
echo "start     : $START  (UTC; trajectory t=0)"
echo "rate      : $RATE Sa/s   center: $CENTER Hz"
echo

names=("$@")
if [[ ${#names[@]} -eq 0 ]]; then
  shopt -s nullglob
  for f in "$SCENDIR"/*.csv; do names+=("$(basename "${f%.csv}")"); done
fi
[[ ${#names[@]} -gt 0 ]] || { echo "no scenarios in $SCENDIR" >&2; exit 1; }

# CRITICAL: gps-sdr-sim strcpy()s each path argument into a 100-byte buffer with
# no bounds check (gpssim.c:1824). A long absolute path -- a session scratchpad
# is already ~180 chars -- smashes the stack and aborts. The lengths that matter
# are the *argument strings*, not the resolved paths, so run from OUTDIR and pass
# short relative names. Guard it anyway in case someone points -o somewhere deep.
cd "$OUTDIR"
rel_eph="$(python3 -c 'import os,sys;print(os.path.relpath(*sys.argv[1:3]))' "$EPH" "$OUTDIR")"

for name in "${names[@]}"; do
  csv="$SCENDIR/$name.csv"
  [[ -f "$csv" ]] || { echo "skip $name: no $csv" >&2; continue; }
  rel_csv="$(python3 -c 'import os,sys;print(os.path.relpath(*sys.argv[1:3]))' "$csv" "$OUTDIR")"
  out="$name.C8"

  for arg in "$rel_eph" "$rel_csv" "$out"; do
    (( ${#arg} < 90 )) || { echo "path arg too long for gps-sdr-sim (${#arg}>90): $arg" >&2; exit 1; }
  done

  echo "--- $name"
  # -x  lat,lon,height user motion (10 Hz)     -b 8  signed 8-bit == C8
  # -p  hold channel power constant: a flat C/N0 makes BLOCKED unambiguous
  #     against NO_LOCK, which is the one call this whole test rests on.
  gps-sdr-sim -e "$rel_eph" -x "$rel_csv" -b 8 -s "$RATE" -t "$START" -p -o "$out" \
    2>&1 | tr '\r' '\n' | grep -vE '^Time into run' | sed 's/^/    /'

  printf 'center_frequency=%s\nsample_rate=%s\n' "$CENTER" "$RATE" > "$name.TXT"

  bytes=$(wc -c < "$out" | tr -d ' ')
  secs=$(python3 -c "print(f'{$bytes/(2*$RATE):.1f}')")
  echo "    -> $out  $(python3 -c "print(f'{$bytes/1e9:.2f}')") GB  ${secs}s"
done

cat <<MSG

Files are in $OUTDIR/ .

PortaPack:  copy each NAME.C8 + NAME.TXT to the SD card under /GPS/ , then
            GPS Sim -> Open -> pick the .C8 (leave Loop unchecked).

HackRF from this Mac (no SD card):
    hackrf_transfer -t $OUTDIR/NAME.C8 -f $CENTER -s $RATE -a 0 -x 0
    # raise -x (0..47) from cold until the receiver sees ~40-45 dBHz; no -R.

Record this start time with every capture -- correlate.py needs it:
    $START
MSG
