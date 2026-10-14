#!/usr/bin/env bash
# One rig replay (#1600): reset the flight computer (a TR_SIM_REPLAY build carrying the flight's sensors), log its
# console, play the same flight's GNSS on the COCOM rig, then leave the radio idle (the PortaPack back in Mayhem).
#
#   tools/rig_replay/run.sh PORT [GAIN] [TAG]
#
#   PORT        the FC's USB port -- check it is the FC by its MAC (USB serial number) before anything
#   GAIN        HackRF TX gain (46 for the LC86G in the cage, 2026-10)
#   TAG         the run's name: its files go to $RIG_OUT/run_TAG/
# Environment:
#   COCOM_SDR   the rig's folder (TinkerRocket-GNSS cocom-rig/sdr); default: this repo's gnss submodule
#   RIG_FLIGHT  the flight id (default sim_rp5_0829): its IQ file c8/signalsim_real_<id>_sky3d_all_45_w_p180.C8
#               and its scenario JSON (for the file's length)
#   RIG_C8      the IQ file, if not that
#   RIG_OUT     where runs go (default ./rig_runs)
#   TX_NOISE    DB@SECONDS: bury the signal from that file second on (an outage; needs TinkerRocket-GNSS#7's
#               hackrf_tx_ram), e.g. 30@220 is +40 s into rp5_0829's flight
#   DRY_RUN=1   print what it would do
set -u
PORT="$1"; GAIN="${2:-46}"; TAG="${3:-r1}"
HERE="$(cd "$(dirname "$0")" && pwd)"
SDR="${COCOM_SDR:-$(git -C "$HERE" rev-parse --show-toplevel)/gnss/cocom-rig/sdr}"
FLIGHT="${RIG_FLIGHT:-sim_rp5_0829}"
C8="${RIG_C8:-$SDR/c8/signalsim_real_${FLIGHT}_sky3d_all_45_w_p180.C8}"
SCEN="$SDR/scenarios/real_${FLIGHT}_sky3d_pad180.json"
OUT="${RIG_OUT:-$PWD/rig_runs}/run_$TAG"
TX="$SDR/hackrf_tx_ram/hackrf_tx_ram"
BOOT_S=15           # FC boot, the receiver's configuration and cold start, the replay on its pad loop
TAIL_S=120          # after the file: landing detection, the ground
FILE_S=$(python3 -c "import json, math; print(math.ceil(json.load(open('$SCEN'))['duration_s']))" 2>/dev/null || echo 257)

for f in "$C8" "$TX" "$SDR/ensure_hackrf.py"; do
    [ -e "$f" ] || { echo "!! missing: $f (set COCOM_SDR / RIG_C8)"; exit 1; }
done
TX_ARGS=(-t "$C8" -f 1568286000 -s 18480000 -a 0 -x "$GAIN" -B -b 20000000 -C -23 ${TX_NOISE:+-N "$TX_NOISE"})
if [ -n "${DRY_RUN:-}" ]; then
    echo "out:      $OUT"
    echo "console:  python3 -u $HERE/console.py $PORT $((BOOT_S + FILE_S + TAIL_S)) $OUT/fc_console.log  (then ${BOOT_S} s)"
    echo "transmit: caffeinate -dimsu $TX ${TX_ARGS[*]}"
    exit 0
fi
mkdir -p "$OUT"

echo "# $(date +%H:%M:%S) pre-caching $(basename "$C8")"
cat "$C8" > /dev/null
python3 -c "import sys; sys.path.insert(0, '$SDR'); from ensure_hackrf import ensure_hackrf; sys.exit(0 if ensure_hackrf() else 1)" \
    || { echo "!! no HackRF"; exit 1; }
echo "# $(date +%H:%M:%S) resetting the FC and logging its console ($PORT)"
python3 -u "$HERE/console.py" "$PORT" $((BOOT_S + FILE_S + TAIL_S)) "$OUT/fc_console.log" &
CPID=$!
sleep $BOOT_S
echo "# $(date +%H:%M:%S) transmitting, gain $GAIN, carrier -23 Hz${TX_NOISE:+, noise $TX_NOISE dB@s}"
caffeinate -dimsu "$TX" "${TX_ARGS[@]}" > "$OUT/hackrf.txt" 2>&1
echo "# $(date +%H:%M:%S) file done: $(tail -2 "$OUT/hackrf.txt" | head -1)"
wait $CPID
python3 -c "import sys; sys.path.insert(0, '$SDR'); from ensure_hackrf import hackrf_idle; hackrf_idle()"
echo "# $(date +%H:%M:%S) done: $OUT"
