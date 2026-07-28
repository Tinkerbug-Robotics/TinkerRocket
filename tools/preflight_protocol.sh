#!/usr/bin/env bash
# Local protocol-change gate (docs/protocol-change-checklist.md §C).
# Runs every parity check that works without Xcode:
#   1. wire-code checker (dispatch uniqueness + Swift<->Kotlin parity)
#   2. C++ host tests incl. golden-fixture freshness
#   3. Android JVM tests incl. the Kotlin golden walk
# iOS tests still need Xcode/CI.
set -euo pipefail
cd "$(dirname "$0")/.."

echo "== 1/3 wire-code checker =="
python3 tools/check_ble_command_ids.py

echo "== 2/3 C++ host tests (incl. fixture freshness) =="
cmake -S tests_cpp -B tests_cpp/build -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build tests_cpp/build -j"$(sysctl -n hw.ncpu 2>/dev/null || nproc)" >/dev/null
ctest --test-dir tests_cpp/build --output-on-failure -Q || {
    ctest --test-dir tests_cpp/build --output-on-failure --rerun-failed; exit 1; }
echo "   OK"

echo "== 3/3 Android JVM tests (golden walk) =="
if [ -z "${JAVA_HOME:-}" ] && [ -d /opt/homebrew/opt/openjdk@21 ]; then
    export JAVA_HOME=/opt/homebrew/opt/openjdk@21
fi
(cd TinkerRocketAndroid && ./gradlew --no-daemon -q test)
echo "   OK"

echo "PASS — protocol surfaces consistent (iOS tests: run via Xcode/CI)"
