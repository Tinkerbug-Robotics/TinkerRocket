#!/usr/bin/env bash
# One rig replay on the V9 with the mosaic-G5 (#1600 on the Goblin's V9): reset the flight computer (a
# TR_SIM_REPLAY_BLOB build of the mosaic-only image, firmware-ota/v9-rig-replay/<flight>/), log its console, play
# the same flight's GNSS conducted into the mosaic from the COCOM rig's HackRF (PortaPack Mayhem), then leave the
# radio idle. Flashing is not here: each flash needs the owner's OK after the MAC check.
#
#   tools/rig_replay/run_v9.sh FLIGHT GAIN TAG
#
#   FLIGHT   sim_rp5_0829 | sim_eagleclaw_0517 | sim_gtv67_0509: the IQ file c8/signalsim_real_<FLIGHT>_sky3d_all_45_w_p180.C8
#            (180 s of pad, then the flight) and the scenario JSON for its length; the image must carry the same
#            flight (its boot line "RIG REPLAY: '<name>'"), or nothing is transmitted
#   GAIN     HackRF TX gain (-x), conducted: -x 3 read GPS 40 dB-Hz on the pad (10/17); the rig caps C/N0 at
#            about 38-40 dB-Hz whatever the gain
#   TAG      the run's name: its files go to $RIG_OUT/run_TAG/
#
# Environment:
#   COCOM_SDR        the rig's folder (TinkerRocket-GNSS cocom-rig/sdr); default: this repo's gnss submodule
#   RIG_C8           the IQ file, if not $COCOM_SDR/c8/signalsim_real_<FLIGHT>_sky3d_all_45_w_p180.C8
#   RIG_OUT          where runs go (default ./rig_runs)
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
#   DRY_RUN=1        check what can be checked without a port or the radio, print what it would do
#
# It refuses while a build runs (an underrun slips the whole sky) or while another session has the rig, finds the
# FC by its USB serial number without opening any port, marks the rig active (back to idle on any exit, with the
# radio left idle: the PortaPack back in Mayhem), and writes the run folder: fc_console.txt, hackrf.txt,
# run.json (the image's version, the blob's SHA-256, the gain, the times, VOID and why), and VOID when void.
# Score it with tools/rig_replay/score_v9.py.
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
OUT="${RIG_OUT:-$PWD/rig_runs}/run_$TAG"
IMG="${RIG_IMAGE_DIR:-$(dirname "$MAIN")/firmware-ota/v9-rig-replay/$FLIGHT}"
FC_MAC="${RIG_FC_MAC:-80:F1:B2:D0:94:A7}"
RIG="${RIG_STATE_FILE:-/private/tmp/claude-501/-Users-christianpedersen-Projects-ModelRockets/2bc47118-da48-4f6f-a411-bb72574fe97a/scratchpad/rf1015/RIG_STATE}"
TX="$SDR/hackrf_tx_ram/hackrf_tx_ram"
CAPTURE="$TOP/tools/bench_capture_boot.py"
READY_S="${READY_TIMEOUT_S:-40}"
VOID_BY="${VOID_BY_S:-170}"
TAIL="${TAIL_S:-120}"
BUILD_RX='ninja|cc1plus|clang.*-cc1|riscv32-esp-elf-g|xtensa-esp32s3-elf-g|cmake --build|make .*-j|idf\.py .*build'

say() { echo "# $(date +%H:%M:%S) $*"; }
die() { echo "!! $*" >&2; exit 1; }

# ---- what must be there
for f in "$C8" "$TX" "$SDR/ensure_hackrf.py" "$SCEN" "$CAPTURE"; do
    [ -e "$f" ] || die "missing: $f (set COCOM_SDR / RIG_C8)"
done
FILE_S=$(python3 -c "import json, math; print(math.ceil(json.load(open('$SCEN'))['duration_s']))") || die "no duration in $SCEN"
PAD_S=$(python3 -c "import json; print(json.load(open('$SCEN'))['prologue_s'])") || die "no prologue in $SCEN"
BLOB=$(ls "$IMG"/sim_replay_*.bin 2>/dev/null | head -1)
BLOB_SHA=$([ -n "$BLOB" ] && shasum -a 256 "$BLOB" | cut -d' ' -f1 || echo unknown)
TX_ARGS=(-t "$C8" -f 1568286000 -s 18480000 -a 0 -x "$GAIN" -B -b 20000000 -C -23 ${TX_NOISE:+-N "$TX_NOISE"})

# ---- a quiet Mac, a free rig
BUSY=$(pgrep -fl "$BUILD_RX" | grep -v -e pgrep -e run_v9.sh || true)
[ -z "$BUSY" ] || die "a build is running -- an underrun slips the whole sky; wait for it:
$BUSY"
[ "$(tr -d ' \n' < "$RIG" 2>/dev/null)" != "active" ] || die "RIG_STATE says active: another session has the rig"

# ---- the FC by its USB serial number (= its MAC), no port opened. The PortaPack is the only other USB device with
# a serial number allowed (Mayhem's "Transceiver", or the HackRF's 32 hex digits if a run left it in HackRF mode):
# exactly one serial besides it, and it must be the FC's.
SER=$(ioreg -p IOUSB -l -w0 | grep -o '"USB Serial Number" = "[^"]*"' | grep -vi transceiver | sed 's/.*= "\(.*\)"/\1/' \
      | grep -v -E '^[0-9a-fA-F]{32}$' || true)
PORTS=$(ls /dev/cu.usbmodem* 2>/dev/null | grep -v Transceiver || true)
NSER=$(printf '%s\n' "$SER" | grep -c . || true)
NPORT=$(printf '%s\n' "$PORTS" | grep -c . || true)
if [ "$NSER" != 1 ] || [ "$(printf '%s' "$SER" | tr a-f A-F)" != "$(printf '%s' "$FC_MAC" | tr a-f A-F)" ] || [ "$NPORT" != 1 ]; then
    die "MAC check failed: USB serials [$(echo $SER)], ports [$(echo $PORTS)]; want exactly the FC $FC_MAC on one port"
fi
PORT="$PORTS"
say "FC $FC_MAC on $PORT; flight $FLIGHT ($FILE_S s, ignition at file second $PAD_S); gain $GAIN; image folder $IMG"

if [ -n "${DRY_RUN:-}" ]; then
    echo "out:      $OUT"
    echo "blob:     ${BLOB:-none} ($BLOB_SHA)"
    echo "console:  python3 -u $CAPTURE $PORT <boot + file + tail s> $OUT/fc_console.txt"
    echo "ready:    'mosaic-G5 set up: ... (setup 2)' after 'BENCH: mosaic-G5 cold start', at most $READY_S s"
    echo "transmit: caffeinate -dimsu $TX ${TX_ARGS[*]}"
    echo "void:     no 'RIG REPLAY clock: on the receiver's clock' by file second $VOID_BY, or MISSED"
    echo "tail:     $TAIL s, then the radio idle"
    exit 0
fi

mkdir -p "$OUT" || die "cannot make $OUT"
[ ! -e "$OUT/fc_console.txt" ] || die "$OUT already holds a run: another TAG"
CON="$OUT/fc_console.txt"
CPID=""; TPID=""
stop_tx() {   # caffeinate and the transmitter under it
    [ -n "$TPID" ] || return 0
    pkill -TERM -P "$TPID" 2>/dev/null; kill "$TPID" 2>/dev/null; wait "$TPID" 2>/dev/null
    TPID=""
}
cleanup() {
    stop_tx
    [ -n "$CPID" ] && kill "$CPID" 2>/dev/null
    python3 -c "import sys; sys.path.insert(0, '$SDR'); from ensure_hackrf import hackrf_idle; hackrf_idle()"
    echo idle > "$RIG"
    say "rig idle"
}
echo active > "$RIG"
trap cleanup EXIT
trap 'exit 130' INT TERM

say "pre-caching $(basename "$C8")"
cat "$C8" > /dev/null
python3 -c "import sys; sys.path.insert(0, '$SDR'); from ensure_hackrf import ensure_hackrf; sys.exit(0 if ensure_hackrf() else 1)" \
    || die "no HackRF"

# ---- the FC: a deliberate reset (a fresh start, so no crash rule skips the receiver), its console from boot
CAP_S=$((READY_S + 30 + FILE_S + TAIL + 30))
say "resetting the FC and logging its console ($PORT, at most $CAP_S s)"
T_RESET=$(python3 -c 'import time; print(f"{time.time():.3f}")')
python3 -u "$CAPTURE" "$PORT" "$CAP_S" "$CON" > "$OUT/capture.txt" 2>&1 &
CPID=$!

# ---- the right flight on board, and the receiver set up again after its cold start
NAME=""; READY=""
for _ in $(seq 1 $((READY_S * 2))); do
    sleep 0.5
    [ -z "$NAME" ] && NAME=$(grep -a -o "RIG REPLAY: '[^']*'" "$CON" 2>/dev/null | head -1 | sed "s/RIG REPLAY: '\(.*\)'/\1/")
    if grep -a -q "BENCH: mosaic-G5 cold start" "$CON" 2>/dev/null && grep -a -q "mosaic-G5 set up: .*(setup 2)" "$CON"; then
        READY=1; break
    fi
done
VERSION=$(grep -a -o "App version: *[^ ]*" "$CON" | head -1 | awk '{print $3}')
say "image $VERSION carries '${NAME:-?}'"
if [ -z "$NAME" ]; then
    die "no 'RIG REPLAY:' line: not a replay image, or the flight did not load -- nothing transmitted"
fi
case "$NAME" in *"${FLIGHT#sim_}"*) ;; *) die "the image carries '$NAME', not $FLIGHT -- nothing transmitted";; esac
if [ -z "$READY" ]; then
    grep -a -q "BENCH: mosaic-G5 cold start" "$CON" || die "no cold start on the console in $READY_S s -- nothing transmitted"
    say "no second setup after the cold start in $READY_S s: transmitting anyway"
fi

# ---- the sky
say "transmitting gain $GAIN, carrier -23 Hz${TX_NOISE:+, noise $TX_NOISE dB@s}"
T_TX=$(python3 -c 'import time; print(f"{time.time():.3f}")')
caffeinate -dimsu "$TX" "${TX_ARGS[@]}" > "$OUT/hackrf.txt" 2>&1 &
TPID=$!

# ---- the replay clock must be on the receiver's clock by file second VOID_BY
VOID=""; WHY=""; ARMED_T=""
while kill -0 "$TPID" 2>/dev/null; do
    sleep 1
    FS=$(python3 -c "import time; print(int(time.time() - $T_TX))")
    if grep -a -q "RIG REPLAY clock: MISSED" "$CON"; then
        VOID=1; WHY="the replay clock's first time came after T-5 s (MISSED): the flight never starts"; break
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
[ -n "$TPID" ] && wait "$TPID"
TPID=""
T_END=$(python3 -c 'import time; print(f"{time.time():.3f}")')
say "file done: $(grep -a 'sent' "$OUT/hackrf.txt" | tail -1)"
say "console tail $TAIL s"
sleep "$TAIL"
kill "$CPID" 2>/dev/null; wait "$CPID" 2>/dev/null; CPID=""

# ---- the run's record
python3 - "$OUT" <<EOF
import json, re, sys
out = sys.argv[1]
hk = open(out + "/hackrf.txt", errors="replace").read().splitlines()
under = [int(m.group(1)) for ln in hk for m in [re.search(r"(\d+) underruns", ln)] if m]
json.dump({
    "flight": "$FLIGHT", "tag": "$TAG", "gain": $GAIN, "tx_args": "${TX_ARGS[*]}",
    "tx_noise": "${TX_NOISE:-}", "fc_mac": "$FC_MAC", "port": "$PORT",
    "image_version": "$VERSION", "image_flight": "$NAME", "image_dir": "$IMG",
    "blob": "$(basename "${BLOB:-none}")", "blob_sha256": "$BLOB_SHA",
    "file_s": $FILE_S, "pad_s": $PAD_S, "void_by_file_s": $VOID_BY,
    "host_reset_s": $T_RESET, "host_tx_start_s": $T_TX, "host_tx_end_s": $T_END,
    "clock_armed_t_s": float("${ARMED_T:-nan}"), "void": bool("$VOID"), "void_why": "$WHY",
    "underruns": max(under) if under else None, "tail_s": $TAIL,
}, open(out + "/run.json", "w"), indent=1)
EOF
[ -f "$IMG/SHA256SUMS" ] && cp "$IMG/SHA256SUMS" "$OUT/image_SHA256SUMS"
say "done: $OUT ${VOID:+(VOID)}"
echo "score: python3 $HERE/score_v9.py $OUT --flight $FLIGHT${BLOB:+ --blob $BLOB}"
