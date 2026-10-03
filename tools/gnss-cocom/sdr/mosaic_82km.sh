#!/usr/bin/env bash
# The rig's two 82 km flights on the mosaic-G5 (owner 10-02: "let's also run the 82km shots we ran a while ago
# against this receiver"): gentle_alt (3 g, 82.5 km apogee, 997 m/s) and spaceshot (15 g, 82.5 km, 1341 m/s),
# rebuilt as wide all-three SignalSim files at 51 dB-Hz (+6 dB) with a 180 s pad (configs from
# signalsim_wide_flight.py, run in GEN_DIR, the SignalSim working folder). Waits for mosaic_sweep.sh to finish and the transmitter to stop -- generation is CPU-heavy
# and starves the transmitter -- then generates both side by side, then flies each from a cold start
# (mosaic_run.py, carrier -23 Hz, gain 3). A flight with any underrun from T-60 to its end is flown once more
# (tag + "r"): these flights have gates spread over the whole trajectory, not one boost window.
#   mosaic_82km.sh GEN_DIR SWEEP_OUT      -> stdout
set -uo pipefail
SDR="$(cd "$(dirname "$0")" && pwd)"
GEN="$1"; SWEEP_OUT="$2"
TAGP=mosaic_g5_wide
FLIGHTS="${FLIGHTS:-gentle_alt_all_2026_51_w_p180:480:mga6 spaceshot_all_2026_51_w_p180:460:mss6}"

echo "# $(date +%H:%M:%S) waiting for the sweep to finish"
until grep -q MOSAICSWEEPDONE "$SWEEP_OUT"; do
  grep -q -- "-- stopping" "$SWEEP_OUT" && { echo "!! the sweep stopped on an error -- not continuing"; exit 1; }
  sleep 15
done
while pgrep -f "hackrf_tx_ram -t|hackrf_transfer" > /dev/null; do sleep 5; done
echo "# $(date +%H:%M:%S) sweep done, transmitter idle; generating"

cd "$GEN"
for f in $FLIGHTS; do
  IFS=: read -r n secs tag <<< "$f"
  [ -e "c8/signalsim_$n.C8" ] && { echo "# c8/signalsim_$n.C8 exists -- not regenerating"; continue; }
  ( ./IFdataGen -c "$n.json" > "gen_$n.log" 2>&1
    echo "# $(date +%H:%M:%S) $n generated (exit $?): $(grep -E 'Data generated' "gen_$n.log" | tr -s '\t ' ' ')" ) &
  echo "# $(date +%H:%M:%S) started $n"
done
wait
for f in $FLIGHTS; do
  IFS=: read -r n secs tag <<< "$f"
  want=$((secs * 18480000 * 2))
  have=$(stat -f %z "c8/signalsim_$n.C8" 2>/dev/null || echo 0)
  echo "# c8/signalsim_$n.C8: $have bytes (want $want)"
  # SignalSim writes 1 ms less than duration x rate (the hotshot file: 36960 bytes short)
  [ $((want - have)) -ge 0 ] && [ $((want - have)) -le 100000 ] || { echo "!! $n is the wrong size -- stopping"; exit 1; }
done

cd "$SDR"
any_hit() {   # underruns from T-60 (file second ~118) to the end
  python3 - "$1" <<'EOF'
import re, sys
prev, hit = 0, []
lines = [x for x in open(sys.argv[1], errors="replace") if "MB / " in x]
for k, x in enumerate(lines[:-1], 1):
    m = re.search(r"(\d+) underruns", x)
    n = int(m.group(1)) if m else prev
    if n > prev and k >= 118:
        hit.append(round(k - 178.2, 1))
    prev = n
print(" ".join(f"T{t:+.1f}" for t in hit))
sys.exit(1 if hit else 0)
EOF
}
for f in $FLIGHTS; do
  IFS=: read -r n secs tag <<< "$f"
  scen=${n%%_all_*}
  for try in "" r; do
    t="$tag$try"
    C="$SDR/captures/${TAGP}${t}_signalsim_$n.log"
    [ -e "$C" ] && { echo "!! $t exists -- skipping"; break; }
    echo "######## $(date +%H:%M:%S) $t: signalsim_$n TX_CARRIER_HZ=-23"
    TX_CARRIER_HZ=-23 caffeinate -dimsu python3 -u mosaic_run.py -s "signalsim_$n" --c8-dir "$SDR/c8" -x 3 \
        --cold-start --seconds "$((secs + 5))" --rate 18480000 --freq 1568286000 --bb-filter 20000000 \
        --tag "$TAGP$t" 2>&1 | grep --line-buffered -E "^# |^!!|Traceback|Error"
    [ -e "$C" ] || { echo "!! no capture for $t -- stopping"; exit 1; }
    # zero power on any per-second line but the last (the last is the partial second after the file ends)
    if awk '/MB \// {n++; l[n]=$0} END {for (i = 1; i < n; i++) if (l[i] ~ /average power -99\.0/) exit 1}' "$C.hackrf.txt"; then :; else echo "!! $t: radio stopped mid-flight -- stopping"; exit 1; fi
    python3 mosaic_gate.py "$C" --scenario "$scen" | sed "s/^/# $t: /"
    if hits=$(any_hit "$C.hackrf.txt"); then echo "# $t: no underruns from T-60 on"; break; fi
    echo "# $t: underruns at $hits"
    [ -z "$try" ] && echo "# re-flying $tag once"
  done
done
echo "# $(date +%H:%M:%S) 82 km flights done"
echo MOSAIC82DONE
