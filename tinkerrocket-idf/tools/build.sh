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

# Source ESP-IDF environment. Pinned to the v6.0.x parallel install for the
# #88 6.0 migration branch. FORCE it even when a different IDF is already
# sourced: if a stray older IDF reconfigures the build dir it can silently
# drop/alter sdkconfig options (defaults are only applied on a fresh
# set-target). So we re-source whenever the active IDF isn't exactly this one.
IDF_HOME="$HOME/esp/esp-idf-v6.0"
if [ "$IDF_PATH" != "$IDF_HOME" ] || ! command -v idf.py &>/dev/null; then
    if [ -f "$IDF_HOME/export.sh" ]; then
        . "$IDF_HOME/export.sh" >/dev/null 2>&1
    else
        echo "ERROR: ESP-IDF v6.0 not found at $IDF_HOME"
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

# No explicit set-target. Every project pins its chip with CONFIG_IDF_TARGET in
# its own sdkconfig.defaults (OC/BS/radio=esp32s3, FC=esp32p4, mini=esp32s3),
# and idf.py honours that on the first configure of a build dir, so the target
# is already correct without help.
#
# This used to run `idf.py set-target` guarded by `[ ! -f sdkconfig ]`, testing
# for a generated config at the PROJECT ROOT. Every project now keeps its
# generated sdkconfig in the build dir instead (see each CMakeLists), so that
# guard would never be satisfied — and `set-target` DELETES the build directory,
# so the helper would have thrown away and re-run the whole build on every
# single invocation.

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
