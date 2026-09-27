#!/usr/bin/env bash
# SAM-M10Q on the V9 rocket computer through the spaceshot boost to apogee, one
# u-blox navigation rate per run. The rate is a compile-time constant, so each
# rate is its own image: build them once, then flash and run each in turn.
#
#   ./m10_rate_series.sh build 1 5 10 18
#   ./m10_rate_series.sh run TAG C8 GAIN RATE [RATE ...]
#   ./m10_rate_series.sh run smooth c8/spaceshot_smooth.C8 0 1 10 5 18 1 10 5 18
#
# Images go to tinkerrocket-idf/projects/flight_computer/build_cocom_v9_rR, built
# with TR_BOARD_V9=1 TR_GNSS_COCOM_DIAG=1 TR_GNSS_RATE_HZ=R
# TR_GNSS_BENCH_COLD_START=1. The last sends the receiver a cold start at boot, so
# every run starts from nothing as the LC86G's $PAIR006 does. All three are
# bench-only; no release build sets them.
#
# Before the first flash, back up the board's flash (and restore it after the
# series). The script refuses any board but the V9 flight computer, by its USB
# MAC (override with V9_FC_MAC=...). Captures: captures/m10q_TAG_rateR_runN_spaceshot.log,
# with the console log and the transmitter's log beside it.
# Staged 2026-09-26: the images built; the series itself has not run yet.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
FW="$HERE/../../../tinkerrocket-idf/projects/flight_computer"
V9_FC_MAC=${V9_FC_MAC:-80:F1:B2:D0:94:A7}
. ~/esp/esp-idf-v6.0/export.sh >/dev/null 2>&1 || { echo "!! ESP-IDF v6.0 not found"; exit 1; }

port() {
  python3 - "$V9_FC_MAC" <<'PY'
import sys
from serial.tools import list_ports
for p in list_ports.comports():
    if (p.serial_number or "").upper() == sys.argv[1]:
        print(p.device); break
PY
}

case "${1:-}" in
build)
  shift
  for r in "$@"; do
    d=build_cocom_v9_r$r
    echo "=== $d $(date +%H:%M:%S)"
    (cd "$FW" && idf.py -B "$d" -DTR_BOARD_V9=1 -DTR_GNSS_COCOM_DIAG=1 -DTR_GNSS_RATE_HZ="$r" \
        -DTR_GNSS_BENCH_COLD_START=1 build > "/tmp/m10_$d.log" 2>&1) \
      || { echo "!! build failed: /tmp/m10_$d.log"; exit 1; }
    grep -E "TR_GNSS_RATE_HZ=|TR_GNSS_BENCH_COLD_START=" "/tmp/m10_$d.log" | head -2
  done ;;
run)
  TAG=${2:?tag}; C8=${3:?c8 file}; GAIN=${4:?gain}; shift 4
  [ -f "$HERE/$C8" ] || [ -f "$C8" ] || { echo "!! no $C8"; exit 1; }
  n=0
  for r in "$@"; do
    n=$((n + 1)); tag=m10q_${TAG}_rate${r}_run${n}
    echo "=== run $n: $r Hz, gain $GAIN  $(date +%H:%M:%S)"
    pgrep -x hackrf_transfer >/dev/null && { echo "!! a transmitter is still running; stopping"; exit 1; }
    [ -d "$FW/build_cocom_v9_r$r" ] || { echo "!! no image for $r Hz: ./m10_rate_series.sh build $r"; exit 1; }
    P=$(port); [ -n "$P" ] || { echo "!! V9 flight computer ($V9_FC_MAC) not on USB"; exit 1; }
    (cd "$FW" && idf.py -B "build_cocom_v9_r$r" -p "$P" flash > "/tmp/m10_${tag}_flash.log" 2>&1) \
      || { echo "!! flash failed: /tmp/m10_${tag}_flash.log"; exit 1; }
    sleep 4
    P=$(port); [ -n "$P" ] || { echo "!! the flight computer did not come back after the flash"; exit 1; }
    (cd "$HERE" && python3 -u run_fc.py -s spaceshot --c8 "$C8" -p "$P" -x "$GAIN" --seconds 330 \
        --tag "$tag" > "captures/${tag}_runner.txt" 2>&1)
    echo "    exit $? : $(grep -E 'epochs,|!!|No such|failed' "$HERE/captures/${tag}_runner.txt" | tail -1)"
    sleep 3
  done
  echo "=== done $(date +%H:%M:%S)" ;;
*)
  sed -n '2,20p' "$0"; exit 1 ;;
esac
