#!/usr/bin/env bash
# Build hackrf_tx_ram next to this script (the binary is ignored by git). Needs libhackrf: brew install hackrf.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PREFIX="$(brew --prefix 2>/dev/null || echo /opt/homebrew)"
clang -O2 -Wall -o "$HERE/hackrf_tx_ram" "$HERE/hackrf_tx_ram.c" \
    -I"$PREFIX/include/libhackrf" -L"$PREFIX/lib" -lhackrf -lpthread
echo "built $HERE/hackrf_tx_ram"
