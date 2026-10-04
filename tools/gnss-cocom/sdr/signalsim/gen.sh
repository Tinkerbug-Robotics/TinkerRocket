#!/usr/bin/env bash
# Generate SignalSim IQ files from configs/<NAME>.json into ../c8/. SignalSim has no license, so it is never in this
# repository: SIGNALSIM points at a local IFdataGen build (README.md; the default is the bench-backups build every
# current file came from). The configs hold short relative paths (EphData/..., c8/...), so IFdataGen runs from this
# folder, which gets EphData/ (the rig's day-230 multi-GNSS broadcast file, gunzipped from ../results/) and a c8 link
# to ../c8 on first use, both ignored by git. Refuses to run beside a transmitter (CPU load starves it) and never
# overwrites a file. The _cofs files need signalsim/carrier_shift.py afterwards: ../regen_c8.py build does both.
#
#   ./gen.sh NAME [NAME ...]          e.g. ./gen.sh hotshot_all_2026_45_w_p180   -> logs/gen_NAME.log
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SIGNALSIM="${SIGNALSIM:-$HOME/Projects/ModelRockets/bench-backups/signalsim-builds/2026-09-30_gps-almanac-fix/globsky-SignalSim-c83ae92/build/IFdataGen}"
cd "$HERE" || exit 1
[ $# -gt 0 ] || { sed -n '2,10p' "$0"; exit 2; }
[ -x "$SIGNALSIM" ] || { echo "!! no IFdataGen at $SIGNALSIM (set SIGNALSIM; see README.md)"; exit 1; }
if pgrep -f "hackrf_tx_ram -t|hackrf_transfer" > /dev/null; then
    echo "!! a transmitter is running -- not generating"; exit 1
fi
mkdir -p EphData logs ../c8
[ -s EphData/BRDC_2026230_MN.rnx ] || gunzip -c ../results/BRDC_2026230_MN.rnx.gz > EphData/BRDC_2026230_MN.rnx
[ -e c8 ] || ln -s ../c8 c8
for n in "$@"; do
    cfg="configs/$n.json"
    [ -f "$cfg" ] || { echo "!! no $cfg"; exit 1; }
    out=$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["output"]["name"])' "$cfg")
    [ -e "$out" ] && { echo "!! $out exists -- not overwriting"; continue; }
    echo "# $(date +%H:%M:%S) generating $n"
    "$SIGNALSIM" -c "$cfg" > "logs/gen_$n.log" 2>&1
    echo "# $(date +%H:%M:%S) $n done (exit $?): $(grep -E 'Data generated' "logs/gen_$n.log" | tr -s '\t ' ' ')"
    ls -l "$out"
done
