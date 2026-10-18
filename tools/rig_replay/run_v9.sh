#!/usr/bin/env bash
# One rig replay on the V9 with the mosaic-G5 (#1600 on the Goblin's V9): reset the flight computer (a
# TR_SIM_REPLAY_BLOB build of the mosaic-only image, firmware-ota/v9-rig-replay/<flight>/), log its console, play
# the same flight's GNSS conducted into the mosaic from the COCOM rig's HackRF (PortaPack Mayhem), then leave the
# radio idle. Flashing is not here: each flash needs the owner's OK after the MAC check.
#
#   tools/rig_replay/run_v9.sh FLIGHT GAIN TAG
#
#   FLIGHT   sim_rp5_0829 | sim_eagleclaw_0517 | sim_gtv67_0509: the IQ file c8/signalsim_real_<FLIGHT>_sky3d_all_45_w_p180.C8
#            (180 s of pad, then the flight) and the scenario JSON for its length and landing; the image must carry
#            the same flight (its boot line "RIG REPLAY: '<name>'"), or nothing is transmitted
#   GAIN     HackRF TX gain (-x), conducted: -x 3 read GPS 40 dB-Hz on the pad (10/17); the rig caps C/N0 at
#            about 38-40 dB-Hz whatever the gain
#   TAG      the run's name: its files go to $RIG_OUT/run_TAG/
#
# Environment:
#   COCOM_SDR        the rig's folder (TinkerRocket-GNSS cocom-rig/sdr); default: this repo's gnss submodule
#   RIG_C8           the IQ file, if not $COCOM_SDR/c8/signalsim_real_<FLIGHT>_sky3d_all_45_w_p180.C8
#   RIG_OUT          where runs go (default gnss-attitude-study/rig-replay/v9-runs beside the repository); never inside a
#                    checkout: the console carries the rig pad's position, and runs stay out of git
#   RIG_IMAGE_DIR    the staged image folder (default firmware-ota/v9-rig-replay/<FLIGHT> beside the repo): its
#                    SHA256SUMS and the blob's are recorded with the run
#   RIG_FC_MAC       the flight computer's USB serial number = its MAC (default the Goblin V9's, 80:F1:B2:D0:94:A7)
#   RIG_STATE_FILE   the rig's busy flag other sessions read (default the rf1015 session's RIG_STATE)
#   TX_NOISE         DB@SECONDS: bury the signal from that file second on (hackrf_tx_ram -N), e.g. 30@220
#   READY_TIMEOUT_S  how long to wait for the receiver's setup after its cold start (default 40)
#   VOID_BY_S        the file second by which the replay clock must be on the receiver's clock (default 170: the
#                    clock freezes at T-5 s, file second 175, and a clock that comes later never starts the flight)
#   TAIL_S           console after the file ends (default 120: landing detection, the ground)
#   KEEP_ON_VOID=1   transmit the whole file even when the run is void
#   ANY_IMU_RATE=1   transmit whatever the IMU rate (default: only the flight's, about 3780 reads a second)
#   ANY_GNSS_LOG=1   transmit whatever the GNSS logging (default: only Detailed, what the flight logs)
#   DRY_RUN=1        check what can be checked without a port or the radio, print what it would do
#
# It refuses while a build runs (an underrun slips the whole sky) or while another session has the rig. It finds the
# FC's port from its USB serial number without opening any port (usb_ports.py: the port under the USB device whose
# serial is the FC's MAC; the PortaPack the only other device allowed), and finds it again just before the reset
# pulses RTS on it and once the image has booted. It marks the rig active (back to idle on any exit, with the radio
# left idle: the PortaPack back in Mayhem). It transmits only when the image carries the flight, the receiver was set
# up again after its cold start (or READY_TIMEOUT_S passed), the IMU reads at the flight's rate and GNSS logging is
# Detailed. The run folder: fc_console.txt; hackrf.txt; tx.json (tx_log.py: each underrun on the file's timeline,
# the exit status, the bytes sent); run.json (the image's version and ionosphere, the IMU rate, the blob's SHA-256,
# the gain, the times, the transmitter's verdict, VOID and why); and VOID when void -- no replay clock by VOID_BY_S,
# the clock frozen only after ignition (a time-tag hold across T-5 s and T+0: the replayed IMU steps in flight), an
# underrun after the replay clock froze (the sky late on the replayed IMU for the rest of the flight), or a
# transmitter that failed or stopped short of the file. An underrun while the clock still followed the 1PPS line
# leaves the run DEGRADED: score_v9.py judges the freeze's check against the 1PPS edge and the PPS tag check after
# it. A console capture that ends early (the FC's USB went away: a reset or a crash) or a second boot of the FC
# before the file is done voids the run too. Score it with score_v9.py.
# The FC is left on the RIG REPLAY image: it never sees a real launch -- flash the flight image back before a flight.
set -u
FLIGHT="${1:?FLIGHT}"; GAIN="${2:?GAIN}"; TAG="${3:?TAG}"
HERE="$(cd "$(dirname "$0")" && pwd)"
TOP="$(git -C "$HERE" rev-parse --show-toplevel 2>/dev/null || echo "$HERE/../..")"
# the main checkout (a worktree's gnss submodule is often not checked out) and the folder beside it
MAIN="$(dirname "$(git -C "$HERE" rev-parse --path-format=absolute --git-common-dir 2>/dev/null || echo "$TOP/.git")")"
SDR="${COCOM_SDR:-$TOP/gnss/cocom-rig/sdr}"
[ -n "${COCOM_SDR:-}" ] || [ -e "$SDR/ensure_hackrf.py" ] || SDR="$MAIN/gnss/cocom-rig/sdr"
C8="${RIG_C8:-$SDR/c8/signalsim_real_${FLIGHT}_sky3d_all_45_w_p180.C8}"
SCEN="$SDR/scenarios/real_${FLIGHT}_sky3d_pad180.json"
OUT="${RIG_OUT:-$(dirname "$MAIN")/gnss-attitude-study/rig-replay/v9-runs}/run_$TAG"
IMG="${RIG_IMAGE_DIR:-$(dirname "$MAIN")/firmware-ota/v9-rig-replay/$FLIGHT}"
FC_MAC="${RIG_FC_MAC:-80:F1:B2:D0:94:A7}"
RIG="${RIG_STATE_FILE:-/private/tmp/claude-501/-Users-christianpedersen-Projects-ModelRockets/2bc47118-da48-4f6f-a411-bb72574fe97a/scratchpad/rf1015/RIG_STATE}"
TX="$SDR/hackrf_tx_ram/hackrf_tx_ram"
CAPTURE="$TOP/tools/bench_capture_boot.py"
USB="$HERE/usb_ports.py"
TXLOG="$HERE/tx_log.py"
READY_S="${READY_TIMEOUT_S:-40}"
VOID_BY="${VOID_BY_S:-170}"
TAIL="${TAIL_S:-120}"
BUILD_RX='ninja|cc1plus|clang.*-cc1|riscv32-esp-elf-g|xtensa-esp32s3-elf-g|cmake --build|make .*-j|idf\.py .*build'

say() { echo "# $(date +%H:%M:%S) $*"; }
die() { echo "!! $*" >&2; exit 1; }

# ---- what must be there
for f in "$C8" "$TX" "$SDR/ensure_hackrf.py" "$SCEN" "$CAPTURE" "$USB" "$TXLOG"; do
    [ -e "$f" ] || die "missing: $f (set COCOM_SDR / RIG_C8)"
done
# the run's folder is never inside a checkout (this one, the main one or any worktree under it)
python3 - "$OUT" "$TOP" "$MAIN" <<'PY' || die "RIG_OUT is inside a checkout ($OUT): its console carries the rig pad's position -- put runs outside"
import os, sys
out = os.path.realpath(sys.argv[1])
sys.exit(1 if any(os.path.commonpath([out, os.path.realpath(r)]) == os.path.realpath(r) for r in sys.argv[2:]) else 0)
PY
FILE_S=$(python3 -c "import json, math; print(math.ceil(json.load(open('$SCEN'))['duration_s']))") || die "no duration in $SCEN"
PAD_S=$(python3 -c "import json; print(json.load(open('$SCEN'))['prologue_s'])") || die "no prologue in $SCEN"
LAND_S=$(python3 -c "import json; print(json.load(open('$SCEN'))['flight']['landing_s'])") || die "no landing in $SCEN"
FREEZE_S=$(python3 -c "print($PAD_S - 5.0)")   # the replay clock freezes at T-5 s
BLOB=$(ls "$IMG"/sim_replay_*.bin 2>/dev/null | head -1)
BLOB_SHA=$([ -n "$BLOB" ] && shasum -a 256 "$BLOB" | cut -d' ' -f1 || echo unknown)
TX_ARGS=(-t "$C8" -f 1568286000 -s 18480000 -a 0 -x "$GAIN" -B -b 20000000 -C -23 ${TX_NOISE:+-N "$TX_NOISE"})

# ---- a quiet Mac, a free rig
BUSY=$(pgrep -fl "$BUILD_RX" | grep -v -e pgrep -e run_v9.sh || true)
[ -z "$BUSY" ] || die "a build is running -- an underrun slips the whole sky; wait for it:
$BUSY"
[ "$(tr -d ' \n' < "$RIG" 2>/dev/null)" != "active" ] || die "RIG_STATE says active: another session has the rig"

# ---- the FC by its USB serial number (= its MAC), no port opened: its port is the one under the USB device with
# that serial (usb_ports.py). The PortaPack is the only other USB device with a serial number allowed (Mayhem's
# "Transceiver", or the HackRF's 32 hex digits if a run left it in HackRF mode), and no other usbmodem port.
fc_port() { python3 "$USB" --fc "$FC_MAC"; }
PORT=$(fc_port) || die "MAC check failed (above): want exactly the FC $FC_MAC, on one port"
say "FC $FC_MAC on $PORT; flight $FLIGHT ($FILE_S s, ignition at file second $PAD_S, landing at $LAND_S); gain $GAIN; image folder $IMG"

if [ -n "${DRY_RUN:-}" ]; then
    echo "out:      $OUT"
    echo "blob:     ${BLOB:-none} ($BLOB_SHA)"
    echo "console:  python3 -u $CAPTURE $PORT <boot + file + tail s> $OUT/fc_console.txt (the port found again first)"
    echo "ready:    'mosaic-G5 set up: ... (setup 2)' after 'BENCH: mosaic-G5 cold start', at most $READY_S s"
    echo "refuse:   no 'RIG REPLAY' line of $FLIGHT; the cold start NOT queued; the console capture ended; the FC's"
    echo "          port moved; [SENSOR] ism6 outside 3400-4200/s; GNSS log not DETAILED"
    echo "transmit: caffeinate -dimsu $TX ${TX_ARGS[*]}"
    echo "void:     no 'RIG REPLAY clock: on the receiver's clock' by file second $VOID_BY, or MISSED; 'frozen for the"
    echo "          flight' at t >= 0 (a time-tag hold across T-5 s and T+0); the console capture"
    echo "          ending or the FC booting again before the file is done; an underrun or host shortfall after file"
    echo "          second $FREEZE_S (the freeze) and before $LAND_S (the landing); the transmitter failing or short"
    echo "          (score_v9.py adds: a 1PPS tag check over 200 us from the freeze to the landing)"
    echo "tail:     $TAIL s, then the radio idle"
    exit 0
fi

mkdir -p "$OUT" || die "cannot make $OUT"
[ ! -e "$OUT/fc_console.txt" ] || die "$OUT already holds a run: another TAG"
CON="$OUT/fc_console.txt"
CPID=""; TPID=""; STOPPED=""; RESET=""; VERSION=""
stop_tx() {   # caffeinate and the transmitter under it
    [ -n "$TPID" ] || return 0
    pkill -TERM -P "$TPID" 2>/dev/null; kill "$TPID" 2>/dev/null; wait "$TPID" 2>/dev/null
    TPID=""; STOPPED=1
}
cleanup() {
    stop_tx
    [ -n "$CPID" ] && kill "$CPID" 2>/dev/null
    python3 -c "import sys; sys.path.insert(0, '$SDR'); from ensure_hackrf import hackrf_idle; hackrf_idle()"
    echo idle > "$RIG"
    say "rig idle"
    # the board is the Goblin's own V9: a replay image plays a canned pad from boot and never sees a real launch
    [ -z "$RESET" ] || echo "!! DO NOT FLY: the FC $FC_MAC runs the RIG REPLAY image ${VERSION:-?} -- flash the flight image back before any flight" >&2
}
echo active > "$RIG"
trap cleanup EXIT
trap 'exit 130' INT TERM

say "pre-caching $(basename "$C8")"
cat "$C8" > /dev/null
python3 -c "import sys; sys.path.insert(0, '$SDR'); from ensure_hackrf import ensure_hackrf; sys.exit(0 if ensure_hackrf() else 1)" \
    || die "no HackRF"

# ---- the FC: a deliberate reset (a fresh start, so no crash rule skips the receiver), its console from boot. The
# pre-cache and ensure_hackrf took half a minute and re-enumerated the PortaPack: the FC's port is found again right
# before the reset pulses RTS on it
PORT_NOW=$(fc_port) || die "MAC check failed just before the reset (above) -- nothing reset, nothing transmitted"
[ "$PORT_NOW" = "$PORT" ] || die "the FC moved from $PORT to $PORT_NOW since the first check -- nothing reset"
CAP_S=$((READY_S + 30 + 15 + FILE_S + TAIL + 30))
say "resetting the FC and logging its console ($PORT, at most $CAP_S s)"
T_RESET=$(python3 -c 'import time; print(f"{time.time():.3f}")')
python3 -u "$CAPTURE" "$PORT" "$CAP_S" "$CON" > "$OUT/capture.txt" 2>&1 &
CPID=$!
RESET=1

# ---- the right flight on board, and the receiver set up again after its cold start
NAME=""; READY=""; COLD="BENCH: mosaic-G5 cold start"
for _ in $(seq 1 $((READY_S * 2))); do
    sleep 0.5
    [ -z "$NAME" ] && NAME=$(grep -a -o "RIG REPLAY: '[^']*'" "$CON" 2>/dev/null | head -1 | sed "s/RIG REPLAY: '\(.*\)'/\1/")
    grep -a -q -F "$COLD (erst,Soft,PVTData+SatData) NOT queued" "$CON" 2>/dev/null && break
    if grep -a -q -F "$COLD (erst,Soft,PVTData+SatData) queued" "$CON" 2>/dev/null \
            && grep -a -q "mosaic-G5 set up: .*(setup 2)" "$CON"; then
        READY=1; break
    fi
    kill -0 "$CPID" 2>/dev/null || die "the console capture ended during the boot ($(tail -1 "$OUT/capture.txt" 2>/dev/null)) -- nothing transmitted"
done
VERSION=$(grep -a -o "App version: *[^ ]*" "$CON" | head -1 | awk '{print $3}')
say "image $VERSION carries '${NAME:-?}'"
if [ -z "$NAME" ]; then
    die "no 'RIG REPLAY:' line: not a replay image, or the flight did not load -- nothing transmitted"
fi
case "$NAME" in *"${FLIGHT#sim_}"*) ;; *) die "the image carries '$NAME', not $FLIGHT -- nothing transmitted";; esac
if [ -z "$READY" ]; then
    grep -a -q -F "$COLD (erst,Soft,PVTData+SatData) NOT queued" "$CON" \
        && die "the cold start was NOT queued: the receiver keeps the real sky's time and ephemerides -- nothing transmitted"
    grep -a -q -F "$COLD (erst,Soft,PVTData+SatData) queued" "$CON" \
        || die "no cold start queued on the console in $READY_S s -- nothing transmitted"
    say "no second setup after the cold start in $READY_S s: transmitting anyway"
fi
# the reset re-enumerated the FC: the console still comes from the FC's own port
PORT_NOW=$(fc_port) || die "MAC check failed after the reset (above) -- nothing transmitted"
[ "$PORT_NOW" = "$PORT" ] || die "after the reset the FC is on $PORT_NOW, the console on $PORT -- nothing transmitted"

# ---- what the flight flies: the IMU at its rate (the app's 4k Dynamic reads about 3780 a second; the V9 was left at
# 960 Hz once) and GNSS logging Detailed. [SENSOR] comes every second (its first is a partial window), the [RAWGNSS]
# mosaic-G5 line every 5 s
IMU=""; GLOG=""
for _ in $(seq 1 30); do
    IMU=$(grep -a -o "\[SENSOR\] reads/s: ism6=[0-9]*" "$CON" | tail -n +2 | tail -3 | sed 's/.*=//' | sort -n | sed -n 2p)
    GLOG=$(grep -a "\[RAWGNSS\] mosaic-G5:" "$CON" | tail -1 | grep -o "GNSS log [A-Z]*" | awk '{print $3}')
    [ -n "$IMU" ] && [ -n "$GLOG" ] && break
    sleep 0.5
done
say "IMU ${IMU:-?} reads/s, GNSS log ${GLOG:-?}"
if [ -z "${ANY_IMU_RATE:-}" ] && { [ -z "$IMU" ] || [ "$IMU" -lt 3400 ] || [ "$IMU" -gt 4200 ]; }; then
    die "the IMU reads ${IMU:-?}/s, not the flight's ~3780 (4k Dynamic in the app, or ANY_IMU_RATE=1) -- nothing transmitted"
fi
if [ -z "${ANY_GNSS_LOG:-}" ] && [ "$GLOG" != "DETAILED" ]; then
    die "GNSS logging is ${GLOG:-?}, not DETAILED (set it in the app, or ANY_GNSS_LOG=1) -- nothing transmitted"
fi
IONO=$(grep -a -o "RIG REPLAY ionosphere: [^:]*" "$CON" | head -1 | sed 's/RIG REPLAY ionosphere: //')
say "ionosphere: ${IONO:-? (no 'RIG REPLAY ionosphere' line: an image from before it)}"

# ---- the sky
say "transmitting gain $GAIN, carrier -23 Hz${TX_NOISE:+, noise $TX_NOISE dB@s}"
T_TX=$(python3 -c 'import time; print(f"{time.time():.3f}")')
caffeinate -dimsu "$TX" "${TX_ARGS[@]}" > "$OUT/hackrf.txt" 2>&1 &
TPID=$!

# ---- the replay clock must be on the receiver's clock by file second VOID_BY; each underrun said as it comes
VOID=""; WHY=""; ARMED_T=""; FROZEN_T=""; UNDER=0
while kill -0 "$TPID" 2>/dev/null; do
    sleep 1
    FS=$(python3 -c "import time; print(int(time.time() - $T_TX))")
    U=$(grep -a -o "[0-9]* underruns" "$OUT/hackrf.txt" 2>/dev/null | tail -1 | awk '{print $1}')
    if [ -n "$U" ] && [ "$U" -gt "$UNDER" ]; then
        say "transmitter underrun ($UNDER -> $U) near TX second $FS: the sky slipped; judged when the file is done"
        UNDER=$U
    fi
    if grep -a -q "RIG REPLAY clock: MISSED" "$CON"; then
        VOID=1; WHY="the replay clock's first time came after T-5 s (MISSED): the flight never starts"; break
    fi
    # the clock frozen in flight: only a timed epoch moves it, so a time-tag hold across T-5 s and T+0 freezes it
    # on the late sky at the first timed epoch after the release, stepping the replayed IMU during the flight
    if [ -z "$FROZEN_T" ]; then
        FROZEN_T=$(grep -a -o "RIG REPLAY clock: frozen for the flight ([^)]*t [-+0-9.]*" "$CON" | head -1 \
            | sed 's/.* t //')
        if [ -n "$FROZEN_T" ]; then
            say "replay clock frozen at t $FROZEN_T s"
            if FT="$FROZEN_T" python3 -c 'import os, sys; sys.exit(0 if float(os.environ["FT"]) >= 0 else 1)'; then
                VOID=1; WHY="the replay clock froze at t $FROZEN_T s, in flight (a time-tag hold across T-5 s and T+0)"
                break
            fi
        fi
    fi
    if ! kill -0 "$CPID" 2>/dev/null; then
        VOID=1; WHY="the console capture ended at TX second $FS (the FC's USB went away: a reset or a crash)"; break
    fi
    if [ "$(grep -a -c "ESP-ROM:" "$CON")" -gt 1 ]; then
        VOID=1; WHY="the FC booted again at about TX second $FS (a crash or a reset): the replay starts over"; break
    fi
    if grep -a -q "RIG REPLAY clock: on the receiver's clock" "$CON"; then
        [ -z "$ARMED_T" ] && ARMED_T=$(grep -a -o "RIG REPLAY clock: on the receiver's clock ([^)]*t [-+0-9.]*" "$CON" \
            | head -1 | sed 's/.* t //') && say "replay clock on the receiver's at t $ARMED_T s (file second ~$FS)"
    elif [ "$FS" -ge "$VOID_BY" ]; then
        RAW=$(grep -a "mosaic raw: epochs" "$CON" | tail -1 | sed 's/.*handed over (\([^)]*\)).*/\1/')
        VOID=1; WHY="no replay clock by file second $VOID_BY (the driver's last count: ${RAW:-none})"; break
    fi
done
if [ -n "$VOID" ]; then
    say "VOID: $WHY"
    echo "$WHY" > "$OUT/VOID"
    if [ -z "${KEEP_ON_VOID:-}" ]; then
        stop_tx
        TAIL=10
    fi
fi
TX_EXIT=""
if [ -n "$TPID" ]; then
    wait "$TPID"; TX_EXIT=$?
fi
TPID=""
T_END=$(python3 -c 'import time; print(f"{time.time():.3f}")')
say "file done: $(grep -a 'sent' "$OUT/hackrf.txt" | tail -1)${TX_EXIT:+ (exit $TX_EXIT)}"
say "console tail $TAIL s"
sleep "$TAIL"
# a reboot or a lost console in the tail: after the file, so the flight stands; said in run.json, and the scorer
# reads the console only to the second boot
TAIL_NOTE=""
kill -0 "$CPID" 2>/dev/null || TAIL_NOTE="the console capture ended in the tail"
[ "$(grep -a -c "ESP-ROM:" "$CON")" -gt 1 ] && TAIL_NOTE="${TAIL_NOTE:+$TAIL_NOTE; }the FC booted again after the file"
[ -z "$TAIL_NOTE" ] || say "note: $TAIL_NOTE"
kill "$CPID" 2>/dev/null; wait "$CPID" 2>/dev/null; CPID=""

# ---- the transmitter's verdict on the file's timeline (tx_log.py): an underrun after the freeze, a failed or short
# transmission voids the run
ARMED_FS=$(python3 -c "import math; t = float('${ARMED_T:-nan}'); print('' if math.isnan(t) else t + $PAD_S)")
python3 "$TXLOG" "$OUT/hackrf.txt" ${ARMED_FS:+--armed-file-s "$ARMED_FS"} --freeze-file-s "$FREEZE_S" \
    --landing-file-s "$LAND_S" ${TX_EXIT:+--exit "$TX_EXIT"} ${STOPPED:+--stopped} --json "$OUT/tx.json"
TXV=$?
if [ "$TXV" = 3 ] && [ -z "$VOID" ]; then
    VOID=1
    WHY="the transmitter: $(python3 -c "import json; print('; '.join(json.load(open('$OUT/tx.json'))['void_why']))")"
    say "VOID: $WHY"
    echo "$WHY" > "$OUT/VOID"
fi

# ---- the run's record (the values through the environment, so no console text is ever quoted into code)
FLIGHT="$FLIGHT" TAG="$TAG" GAIN="$GAIN" TX_ARGS="${TX_ARGS[*]}" TX_NOISE="${TX_NOISE:-}" FC_MAC="$FC_MAC" \
PORT="$PORT" VERSION="$VERSION" NAME="$NAME" IMG="$IMG" BLOB="$(basename "${BLOB:-none}")" BLOB_SHA="$BLOB_SHA" \
FILE_S="$FILE_S" PAD_S="$PAD_S" LAND_S="$LAND_S" VOID_BY="$VOID_BY" T_RESET="$T_RESET" T_TX="$T_TX" \
T_END="$T_END" ARMED_T="${ARMED_T:-nan}" VOID="$VOID" WHY="$WHY" TAIL="$TAIL" IMU="${IMU:-}" GLOG="${GLOG:-}" \
IONO="${IONO:-}" TAIL_NOTE="$TAIL_NOTE" python3 - "$OUT" <<'PY'
import json, os, sys
out, e = sys.argv[1], os.environ
tx = json.load(open(out + "/tx.json")) if os.path.exists(out + "/tx.json") else {}
json.dump({
    "flight": e["FLIGHT"], "tag": e["TAG"], "gain": float(e["GAIN"]), "tx_args": e["TX_ARGS"],
    "tx_noise": e["TX_NOISE"], "fc_mac": e["FC_MAC"], "port": e["PORT"],
    "image_version": e["VERSION"], "image_flight": e["NAME"], "image_dir": e["IMG"],
    "image_ionosphere": e["IONO"], "imu_reads_per_s": int(e["IMU"]) if e["IMU"] else None, "gnss_log": e["GLOG"],
    "blob": e["BLOB"], "blob_sha256": e["BLOB_SHA"],
    "file_s": float(e["FILE_S"]), "pad_s": float(e["PAD_S"]), "landing_file_s": float(e["LAND_S"]),
    "void_by_file_s": float(e["VOID_BY"]),
    "host_reset_s": float(e["T_RESET"]), "host_tx_start_s": float(e["T_TX"]), "host_tx_end_s": float(e["T_END"]),
    "clock_armed_t_s": float(e["ARMED_T"]), "void": bool(e["VOID"]), "void_why": e["WHY"],
    "underruns": tx.get("underruns"), "tx_verdict": tx.get("verdict"), "tx_degraded_why": tx.get("degraded_why"),
    "tail_s": float(e["TAIL"]), "tail_note": e["TAIL_NOTE"],
    "fc_left_with": "the RIG REPLAY image " + e["VERSION"] + ": DO NOT FLY",
}, open(out + "/run.json", "w"), indent=1)
PY
[ -f "$IMG/SHA256SUMS" ] && cp "$IMG/SHA256SUMS" "$OUT/image_SHA256SUMS"
say "done: $OUT${VOID:+ (VOID)}$([ "$TXV" = 2 ] && echo ' (DEGRADED: underruns while the clock followed the 1PPS line; the score judges the PPS check)')"
echo "score: python3 $HERE/score_v9.py $OUT --flight $FLIGHT${BLOB:+ --blob $BLOB}"
