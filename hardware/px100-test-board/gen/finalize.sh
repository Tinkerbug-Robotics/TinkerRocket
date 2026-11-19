#!/bin/bash
# After route.sh: close what the autorouter left (patch_route.py), stitch GND (stitch.py), then DRC + parity.
set -e
cd "$(dirname "$0")"
K=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
P=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
F="traits\|Fontconfig\|Debug\|assert"
$P stitch.py 2>&1 | grep -v "$F"
python3 check.py > /dev/null
$P patch_route.py ${MAX:-200} 2>&1 | grep -v "$F"
$P stitch.py 2>&1 | grep -v "$F"
python3 check.py
(cd .. && python3 ../../tools/check_board_parity.py . 2>&1 | tail -2) || true
