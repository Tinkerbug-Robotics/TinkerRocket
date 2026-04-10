#!/usr/bin/env bash
# Run bench integration tests against binary flight logs in tests/test_data/
#
# Usage:
#   ./tests/run_bench_tests.sh                       # run on all .bin in test_data/
#   ./tests/run_bench_tests.sh path/to/flight.bin    # copy file to test_data/ first, then run
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TEST_DATA="$SCRIPT_DIR/test_data"

mkdir -p "$TEST_DATA"

# If a .bin path was passed, copy it into test_data/
if [[ $# -ge 1 && -f "$1" ]]; then
    cp "$1" "$TEST_DATA/"
    echo "Copied $(basename "$1") to test_data/"
fi

count=$(find "$TEST_DATA" -name "*.bin" 2>/dev/null | wc -l | tr -d ' ')
if [[ "$count" -eq 0 ]]; then
    echo "No .bin files in $TEST_DATA"
    echo "Usage: $0 [path/to/flight.bin]"
    exit 1
fi

echo "Running bench integration tests on $count .bin file(s)..."
echo ""

cd "$SCRIPT_DIR/.."
python3 -m pytest tests/integration/ -v --tb=short "$@"
