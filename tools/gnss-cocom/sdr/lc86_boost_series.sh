#!/usr/bin/env bash
# LC86G (Balloon mode) through the spaceshot boost to apogee, a series of runs in
# the sealed cage. Per entry: a blind cold start, the flight configuration at the
# entry's fix rate, then lc86_bench_run.py transmitting the file and logging.
#
#   ./lc86_boost_series.sh TAG C8 ENTRY [ENTRY ...]        ENTRY = RATE[:GAIN]
#
#   ./lc86_boost_series.sh smooth c8/spaceshot_smooth.C8 10 10 10 10
#   ./lc86_boost_series.sh rate   c8/spaceshot_smooth.C8 1 10 5 1 10 5
#   ./lc86_boost_series.sh level  c8/spaceshot.C8 10:0 10:1 10:3 10:3 10:1 10:0
#
# Run N writes captures/lc86g_TAG_runN_spaceshot.log, plus .hackrf.txt (the
# transmitter's own log, underruns included), .config.txt and .runner.txt beside
# it. Gain is 0 unless an entry says otherwise; the runner refuses above 6 dB.
#
# The blind $PAIR006 is not optional: a run that crosses 80 km leaves the module
# in Balloon's altitude mute, answering nothing, so the configuration step would
# fail without it. c8/spaceshot.C8 steps each carrier every 0.1 s (stock
# gps-sdr-sim); c8/spaceshot_smooth.C8 is the same flight swept smoothly
# (patch_smooth_carrier.py) and is the one whose runs repeat (2026-09-26).
set -u
cd "$(dirname "$0")" || exit 1
TAG=${1:?tag}; C8=${2:?c8 file}; shift 2
[ $# -gt 0 ] || { echo "no runs given"; exit 1; }
[ -f "$C8" ] || { echo "!! no $C8"; exit 1; }
mkdir -p captures
n=0
for entry in "$@"; do
  n=$((n + 1)); rate=${entry%%:*}; gain=0
  [ "$entry" != "$rate" ] && gain=${entry#*:}
  base=captures/lc86g_${TAG}_run${n}_spaceshot
  echo "=== run $n: $rate Hz, gain $gain, $(basename "$C8")  $(date +%H:%M:%S)"
  pgrep -x hackrf_transfer >/dev/null && { echo "!! a transmitter is still running; stopping"; exit 1; }
  python3 -c "
import time
from lc86_config import find_bridge, open_bridge, frame
s = open_bridge(find_bridge('auto')); s.write(frame('PAIR006')); time.sleep(3); s.close()" \
      || { echo "!! the Beetle FC (lc86_bridge) is not on USB"; exit 1; }
  python3 lc86_config.py --rate "$rate" --navmode 3 --rtcm msm7 > "$base.config.txt" 2>&1
  grep -q "verified" "$base.config.txt" || { echo "!! configuration not verified"; tail -3 "$base.config.txt"; exit 1; }
  python3 -u lc86_bench_run.py --c8 "$C8" --seconds 320 --start-mode 3 --rate "$rate" --gain "$gain" \
      --out "$base.log" > "$base.runner.txt" 2>&1
  echo "    exit $? : $(grep -E 'capture ->|!!|refus|not at|capped' "$base.runner.txt" | tail -1)"
  sleep 3
done
echo "=== done $(date +%H:%M:%S)"
