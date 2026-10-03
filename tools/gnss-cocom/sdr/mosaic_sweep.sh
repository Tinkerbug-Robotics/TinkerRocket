#!/usr/bin/env bash
# The C/N0 sweep's eight flights on the mosaic-G5, as flown on the ZED-F9P (f9p_sweep2.sh): hotshot at +12/+6/0/-6 dB,
# then the traveler (180 s pad, to T+360) at +12/+6/0/-6 dB, on the wide all-three SignalSim files (GPS + Galileo +
# BeiDou, 1568.286 MHz, 18.48 Msps). Carrier -23 Hz and the -6 dB noise in hackrf_tx_ram, HackRF gain 3. The RF path
# should have the same 10 dB more attenuation than the PX1105R's that the M8T and F9P had.
# Receiver: cold start before each flight, MeasEpoch + MeasExtra + PVTGeodetic at 20 Hz, srd High, Unlimited, Galileo
# OSNMA off (mosaic_run.py). Stops if the first pad shows almost no satellites (RF not
# connected) or the pad delivers under 90 % of the 20 Hz epochs; a flight with an underrun in T-60..T+30 is flown once
# more (tag + "r").
# The IQ files from C8_DIR (default c8/ here), the transmitter from HACKRF_TX_RAM (see mosaic_run.py).
#   mosaic_sweep.sh [SPEC ...]   SPEC = STEM:SECONDS:NOISE_DB:CARRIER_HZ:TAG      -> stdout
set -uo pipefail
SDR="$(cd "$(dirname "$0")" && pwd)"
TAGP=mosaic_g5_wide
SPECS="${*:-signalsim_hotshot_all_2026_57_w_p180:300:0:-23:mhs12 signalsim_hotshot_all_2026_51_w_p180:300:0:-23:mhs6 signalsim_hotshot_all_2026_45_w_p180:300:0:-23:mhs0 signalsim_hotshot_all_2026_45_w_p180:300:6:-23:mhsn6 signalsim_traveler_all_2026_57_w_p180:540:0:-23:mtr12 signalsim_traveler_all_2026_51_w_p180:540:0:-23:mtr6 signalsim_traveler_all_2026_45_w_p180_cofs:540:0:0:mtr0 signalsim_traveler_all_2026_45_w_p180_cofs:540:6:0:mtrn6}"
cd "$SDR"
capof() { echo "$SDR/captures/${TAGP}$2_$1.log"; }
window_hit() {   # underrun (hackrf -B per-second lines) between T-60 and T+30: ignition is ~178.2 s into the file
  python3 - "$1" <<'EOF'
import re, sys
prev, hit = 0, []
lines = [x for x in open(sys.argv[1], errors="replace") if "MB / " in x]
for k, x in enumerate(lines[:-1], 1):
    m = re.search(r"(\d+) underruns", x)
    n = int(m.group(1)) if m else prev
    if n > prev and 118 <= k <= 208:
        hit.append(round(k - 178.2, 1))
    prev = n
print(" ".join(f"T{t:+.1f}" for t in hit))
sys.exit(1 if hit else 0)
EOF
}

first=1
for sp in $SPECS; do
  IFS=: read -r stem secs noise carr tag <<< "$sp"
  envs=()
  [ "$noise" != 0 ] && envs+=("TX_NOISE_DB=$noise")
  [ "$carr" != 0 ] && envs+=("TX_CARRIER_HZ=$carr")
  for try in "" r; do
    t="$tag$try"
    C=$(capof "$stem" "$t")
    [ -e "$C" ] && { echo "!! $t exists -- skipping"; break; }
    echo "######## $(date +%H:%M:%S) $t: $stem ${envs[*]:-}"
    out=$(env ${envs[@]+"${envs[@]}"} caffeinate -dimsu python3 -u mosaic_run.py -s "$stem" --c8-dir "${C8_DIR:-$SDR/c8}" -x 3 --cold-start \
        --seconds "$((secs + 5))" --rate 18480000 --freq 1568286000 --bb-filter 20000000 --tag "$TAGP$t" 2>&1 \
        | tee /dev/stderr | grep -E "^# |^!!|Traceback|Error")
    [ -e "$C" ] || { echo "!! no capture for $t -- stopping"; exit 1; }
    # zero power on any per-second line but the last (the last is the partial second after the file ends)
    if awk '/MB \// {n++; l[n]=$0} END {for (i = 1; i < n; i++) if (l[i] ~ /average power -99\.0/) exit 1}' "$C.hackrf.txt"; then :; else echo "!! $t: radio stopped mid-flight -- stopping"; exit 1; fi
    pad=$(echo "$out" | grep "^# pad:" | tail -1)
    echo "# $t: $pad"
    pct=$(echo "$pad" | sed -E 's/^# pad: ([0-9]+) %.*/\1/')
    n=$(echo "$pad" | sed -E 's/.*most satellites ([0-9]+).*/\1/')
    if [ "$first" = 1 ] && [ "${n:-0}" -lt 4 ]; then echo "!! the mosaic saw almost nothing on the pad: is its RF connected? -- stopping"; exit 1; fi
    if [ "${pct:-0}" -lt 90 ]; then echo "!! $t: pad delivered under 90 % of the 20 Hz epochs -- stopping"; exit 1; fi
    first=0
    if hits=$(window_hit "$C.hackrf.txt"); then echo "# $t: boost window clean"; break; fi
    echo "# $t: underrun in the boost window at $hits"
    [ -z "$try" ] && echo "# re-flying $tag once"
  done
done
echo "# $(date +%H:%M:%S) mosaic flights done"
echo MOSAICSWEEPDONE
