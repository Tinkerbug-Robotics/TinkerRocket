#!/usr/bin/env bash
# TinkerRocket ESP-IDF build helper
# Usage: ./tools/build.sh <project> [command]
#   project: out_computer | flight_computer | base_station
#   command: build (default) | flash | monitor | menuconfig | fullclean
#
# Examples:
#   ./tools/build.sh out_computer build
#   ./tools/build.sh out_computer flash
#   ./tools/build.sh base_station menuconfig

set -e

# Ensure arm64 homebrew tools (cmake, ninja) are on PATH first
export PATH="/opt/homebrew/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

# Source ESP-IDF environment if not already loaded.
# Pinned to the v5.5.x parallel install for the #88 I2C-V2 bench branch.
IDF_EXPORT="$HOME/esp/esp-idf-v5.5/export.sh"
if ! command -v idf.py &>/dev/null; then
    if [ -f "$IDF_EXPORT" ]; then
        . "$IDF_EXPORT" >/dev/null 2>&1
    else
        echo "ERROR: ESP-IDF v5.5 not found. Run: . $IDF_EXPORT"
        exit 1
    fi
fi

PROJECT="${1:?Usage: $0 <out_computer|flight_computer|base_station> [build|flash|monitor|menuconfig|fullclean]}"
COMMAND="${2:-build}"
PROJECT_DIR="$ROOT_DIR/projects/$PROJECT"

if [ ! -d "$PROJECT_DIR" ]; then
    echo "ERROR: Project '$PROJECT' not found in $ROOT_DIR/projects/"
    echo "Available: $(ls "$ROOT_DIR/projects/")"
    exit 1
fi

cd "$PROJECT_DIR"

# Set target if not already done. Read the desired target from
# sdkconfig.defaults so each project picks its own chip (OC/BS=esp32s3,
# FC=esp32p4) without idf.py silently defaulting to esp32s3 and producing
# a binary for the wrong target.
if [ ! -f "sdkconfig" ]; then
    TARGET="$(grep -E '^CONFIG_IDF_TARGET=' sdkconfig.defaults 2>/dev/null | sed -E 's/.*"([^"]+)".*/\1/' | head -1)"
    TARGET="${TARGET:-esp32s3}"
    echo "First build — setting target to ${TARGET}..."
    idf.py set-target "${TARGET}"
fi

case "$COMMAND" in
    build)     idf.py build ;;
    flash)     idf.py flash ;;
    monitor)   idf.py monitor ;;
    fm)        idf.py flash monitor ;;
    menuconfig) idf.py menuconfig ;;
    fullclean) idf.py fullclean ;;
    *)
        echo "Unknown command: $COMMAND"
        echo "Available: build | flash | monitor | fm | menuconfig | fullclean"
        exit 1
        ;;
esac
