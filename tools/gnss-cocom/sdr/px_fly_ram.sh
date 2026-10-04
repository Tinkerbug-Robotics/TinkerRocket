#!/usr/bin/env bash
# SignalSim flights into the PX1105R through the buffered transmitter (px_run_ram.py -> hackrf_tx_ram/), with the
# L1 sweep's receiver settings (nav mode 9, 3 deg elevation mask, power normal, cold start, HackRF gain 3) and its
# capture naming. Each argument is  NAME:BAND:SECONDS:TAG  for c8/signalsim_NAME.C8, BAND one of
#   n   narrow L1: 8.184 Msps at 1575.42 MHz
#   w   wide L1 + B1I: 18.48 Msps at 1568.286 MHz, 20 MHz baseband filter
#   l5  the L5 band: 18.48 Msps at 1176.45 MHz, 20 MHz baseband filter
# TX_CARRIER_HZ=-23 for the uncorrected wide files (c8_manifest.json says which), TX_NOISE_DB=D lowers C/N0;
# PX_MAC picks the receiver (default: the TinkerNav's PX1105R). After each flight, signalsim/sat_summary.py when the
# file's generation log signalsim/logs/gen_NAME.log is there (signalsim/gen.sh writes it).
#
#   TX_CARRIER_HZ=-23 ./px_fly_ram.sh traveler_all_2026_51_w_p180:w:540:t51 hotshot_all_2026_45_w_p180:w:300:h45
set -uo pipefail
SDR="$(cd "$(dirname "$0")" && pwd)"
cd "$SDR" || exit 1
[ $# -gt 0 ] || { sed -n '2,12p' "$0"; exit 2; }
MAC="${PX_MAC:-9C:13:9E:A2:F8:CC}"
for spec in "$@"; do
    IFS=: read -r name band secs tag <<< "$spec"
    case "$band" in
        n)  OPTS="--rate 8184000 --freq 1575420000" ;;
        w)  OPTS="--rate 18480000 --freq 1568286000 --bb-filter 20000000" ;;
        l5) OPTS="--rate 18480000 --freq 1176450000 --bb-filter 20000000" ;;
        *)  echo "!! band '$band' in $spec: n, w or l5"; exit 1 ;;
    esac
    c8="signalsim_${name}.C8"
    [ -f "c8/$c8" ] || { echo "!! no c8/$c8 (./regen_c8.py build $c8)"; exit 1; }
    log="captures/px1105r_signalsim_${name}_gain3_nav9_el3_${tag}.log"
    echo "# $(date +%H:%M:%S) flying ${c8} (${tag}) with hackrf_tx_ram"
    python3 -u px_run_ram.py 3 "$c8" "$secs" 9 --elev-mask 3 --rx px1105r --mac "$MAC" --power-mode normal \
        $OPTS --tag "$tag" 2>&1 | grep --line-buffered -E "^# |^!!|Traceback|Error" \
        || { echo "# ${name} FAILED"; exit 1; }
    if [ -f "signalsim/logs/gen_${name}.log" ]; then
        python3 signalsim/sat_summary.py "$log" "signalsim/logs/gen_${name}.log"
    else
        echo "# no signalsim/logs/gen_${name}.log, so no satellite summary"
    fi
done
echo "# $(date +%H:%M:%S) flights done"
