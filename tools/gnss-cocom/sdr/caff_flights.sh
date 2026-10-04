#!/usr/bin/env bash
# Keep-awake A/B (2026-10-01): the 0 dB wide traveler (the corrected 180 s-pad file, HackRF gain 3, px_fly_ram.sh),
# flown back to back once per TAG, with the runner's keep-awake (run_radiated.keep_awake). A watcher snapshots the
# power assertions every 60 s while flying; afterwards powerd's PerfMode / activity events in the flight window, then
# per flight the underruns, raw-output gaps, the traveler summary and the fix runs (cn0_sweep/ tools).
# Notes go to captures/caff_*.txt.
#
#   ./caff_flights.sh [TAG ...]      (default: wka1 wka2)
set -uo pipefail
SDR="$(cd "$(dirname "$0")" && pwd)"
CAP="$SDR/captures"
NAME=traveler_all_2026_45_w_p180_cofs
TAGS="${*:-wka1 wka2}"
capof() { echo "$CAP/px1105r_signalsim_${NAME}_gain3_nav9_el3_traveler2026$1.log"; }

for t in $TAGS; do
  [ -e "$(capof "$t")" ] && { echo "!! capture for $t exists -- not overwriting"; exit 1; }
done
if pgrep -f "hackrf_tx_ram|hackrf_transfer|IFdataGen -c" > /dev/null; then
  echo "!! a transmitter or SignalSim is already running:"; pgrep -fl "hackrf_tx_ram|hackrf_transfer|IFdataGen -c"; exit 1
fi
[ -e "$SDR/c8/signalsim_${NAME}.C8" ] || { echo "!! c8/signalsim_${NAME}.C8 missing (./regen_c8.py build)"; exit 1; }
mkdir -p "$CAP"
echo "# USB before the flights:"; ioreg -p IOUSB -w0 | grep -E "PortaPack|HackRF|USB JTAG" | sed 's/<class.*//'

START=$(date "+%Y-%m-%d %H:%M:%S")
FLAG="$CAP/.caff_flying"
touch "$FLAG"
( while [ -e "$FLAG" ]; do
    echo "== $(date +%H:%M:%S)"
    pmset -g assertions | grep -E "caffeinate|^ *(UserIsActive|PreventUserIdleSystemSleep|PreventSystemSleep|PreventUserIdleDisplaySleep|PreventDiskIdle) " | head -12
    sleep 60
  done ) > "$CAP/caff_assertions.txt" 2>&1 &
for t in $TAGS; do
  echo "######## $t ($(date +%H:%M:%S))"
  bash "$SDR/px_fly_ram.sh" "$NAME:w:540:traveler2026$t" || { echo "FLIGHT $t FAILED"; break; }
  if grep -q "average power -99.0" "$(capof "$t").hackrf.txt" 2>/dev/null; then
    echo "!! $t: transmitter stopped mid-flight -- stopping"; break
  fi
done
rm -f "$FLAG"
END=$(date "+%Y-%m-%d %H:%M:%S")
echo "# flights $START .. $END"
echo "# transmitter processes left: $(pgrep -fl 'hackrf_tx_ram|hackrf_transfer' || echo none)"

echo; echo "== powerd PerfMode / activity events during the flights"
log show --style compact --start "$START" --end "$END" \
  --predicate 'process == "powerd" AND (eventMessage CONTAINS "PerfMode" OR eventMessage CONTAINS "Activity changes" OR eventMessage CONTAINS "locked/inactive" OR eventMessage CONTAINS "unlocked/active")' \
  > "$CAP/caff_perfmode.txt" 2>&1
grep -c -E "PerfMode|Activity changes|locked/inactive|unlocked/active" "$CAP/caff_perfmode.txt"
grep -E "PerfMode|Activity changes|locked/inactive|unlocked/active" "$CAP/caff_perfmode.txt" | cut -c1-200 | head -20

for t in $TAGS; do
  NEW=$(capof "$t")
  [ -e "$NEW" ] || continue
  echo; echo "######## $t analysis"
  echo "== underruns"; python3 "$SDR/cn0_sweep/underrun_lines.py" "$NEW.hackrf.txt"
  echo "== gaps"; python3 "$SDR/cn0_sweep/gaps.py" "$NEW" 18480000 2 180
  echo "== traveler summary"; python3 "$SDR/cn0_sweep/traveler_summary.py" "$NEW"
  echo "== fix runs"; python3 "$SDR/cn0_sweep/fix_runs.py" "$NEW" -10 0.35
done
