#!/bin/bash
# After route_best.sh: field/keep-out fixes, patch the router's leftovers, re-stitch, and check.
set -e
cd "$(dirname "$0")"
P=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
$P post_fix.py 2>&1 | grep -v "traits\|Fontconfig"
python3 check.py > /dev/null
$P patch_route.py 2>&1 | grep -v "traits\|Fontconfig"
python3 check.py > /dev/null
$P fixup.py 2>&1 | grep -v "traits\|Fontconfig" | grep -v "^fanout.*via at\|^island.*via added"
python3 check.py > /dev/null
$P patch_route.py 2>&1 | grep -v "traits\|Fontconfig"
python3 check.py
