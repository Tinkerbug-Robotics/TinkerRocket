#!/usr/bin/env bash
# Offline checks of hackrf_tx_ram: no radio is opened. Give it one wide IQ file (18.48 Msps) and one at 2.6 Msps from
# c8/ (any will do):
#  1. a -X dump of the 2.6 Msps file equals the file, also with -K (the exit path)
#  2. -N 6: noise_check.py measures a 6.00 dB C/N0 drop with the file's own RMS kept and a white residual
#  3. -C -23 on the first 64 MiB of the wide file agrees with ../signalsim/carrier_shift.py to rounding
#  4. a 10 s dry run (-D, a consumer paced at 18.48 Msps) with -N 6 -C -23: the ring never runs short
#  5. a 5 s -L loop dry run, and -L with -N refused
#
#   ./check_offline.sh WIDE.C8 NARROW.C8    e.g. ../c8/signalsim_traveler_all_2026_51_w_p180.C8 ../c8/cal_g8.C8
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SDR="$(dirname "$HERE")"
BIN="$HERE/hackrf_tx_ram"
[ $# -eq 2 ] || { sed -n '2,11p' "$0"; exit 2; }
WIDE="$1"; NARROW="$2"
[ -x "$BIN" ] || { echo "!! build it first: $HERE/build.sh"; exit 1; }
for f in "$WIDE" "$NARROW"; do [ -f "$f" ] || { echo "!! no $f"; exit 1; }; done
T="$(mktemp -d -t hackrf_tx_ram_check)"
trap 'rm -rf "$T"' EXIT
export RING_MIB=256 PREFILL_MIB=64
fail=0

echo "== 1. -X dump equals the file ($(basename "$NARROW"), whole file; then with -K)"
ref=$(md5 -q "$NARROW")
d1=$("$BIN" -t "$NARROW" -X 2>/dev/null | md5 -q)
d2=$("$BIN" -t "$NARROW" -X -K 2>/dev/null | md5 -q)
if [ "$d1" = "$ref" ] && [ "$d2" = "$ref" ]; then echo "   identical, with and without -K"
else echo "   DIFFERENT: file $ref, dump $d1, dump -K $d2"; fail=1; fi

echo "== 2. -N 6 on $(basename "$WIDE"): added noise, level kept"
"$BIN" -t "$WIDE" -X -N 6 2>/dev/null | head -c 67108864 > "$T/n6.bin"
python3 "$HERE/noise_check.py" "$WIDE" "$T/n6.bin" 6

echo "== 3. -C -23 against ../signalsim/carrier_shift.py (first 64 MiB)"
head -c 67108864 "$WIDE" > "$T/head.C8"
python3 "$SDR/signalsim/carrier_shift.py" "$T/head.C8" "$T/head_py.C8" 18480000 -23.0 | tail -1
"$BIN" -t "$T/head.C8" -X -s 18480000 -C -23 2>/dev/null > "$T/head_c.C8"
python3 - "$T/head_py.C8" "$T/head_c.C8" <<'EOF'
import sys
import numpy as np
a = np.fromfile(sys.argv[1], dtype=np.int8).astype(int)
b = np.fromfile(sys.argv[2], dtype=np.int8).astype(int)
n = min(a.size, b.size)
d = np.abs(a[:n] - b[:n])
print(f"   {n} bytes compared: {int((d > 0).sum())} differ ({(d > 0).mean() * 100:.4f} %), max |diff| {int(d.max())}")
EOF

echo "== 4. paced dry run, 10 s at 18.48 Msps with -N 6 -C -23"
"$BIN" -t "$WIDE" -s 18480000 -B -D -N 6 -C -23 > "$T/dry.txt" 2>&1 &
P=$!
sleep 10
kill -INT "$P"
wait "$P"
grep -E "added noise|carrier shift|read ahead" "$T/dry.txt" | sed 's/^/   /'
grep "MB / " "$T/dry.txt" | tail -2 | cut -c1-200 | sed 's/^/   /'
tail -1 "$T/dry.txt" | sed 's/^/   /'

echo "== 5. -L 5 loop (dry), then -L with -N"
"$BIN" -t "$WIDE" -s 18480000 -D -B -L 5 > "$T/loop.txt" 2>&1
echo "   loop: exit $?, $(grep -c 'MB / ' "$T/loop.txt") stats lines"
if "$BIN" -t "$NARROW" -s 2600000 -D -L 5 -N 3 > /dev/null 2>&1; then echo "   -L with -N was NOT refused"; fail=1
else echo "   -L with -N refused"; fi
exit $fail
