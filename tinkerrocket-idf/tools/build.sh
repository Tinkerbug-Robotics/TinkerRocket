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

# Source ESP-IDF environment if not already loaded
if ! command -v idf.py &>/dev/null; then
    if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
        . "$HOME/esp/esp-idf/export.sh" >/dev/null 2>&1
    else
        echo "ERROR: ESP-IDF not found. Run: . ~/esp/esp-idf/export.sh"
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

# Set target if not already done
if [ ! -f "sdkconfig" ]; then
    echo "First build — setting target to esp32s3..."
    idf.py set-target esp32s3
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
