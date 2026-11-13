#!/bin/bash
# Place (layout.py), route with Freerouting (one or more item-selection strategies) and keep the result that leaves
# the fewest unconnected items. FREEROUTING_JAR: a Freerouting 2.x jar (github.com/freerouting/freerouting).
set -e
cd "$(dirname "$0")"
K=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
P=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
JAVA=${JAVA:-/opt/homebrew/opt/openjdk/bin/java}
JAR=${FREEROUTING_JAR:?set FREEROUTING_JAR to a Freerouting 2.x jar}
PASSES=${PASSES:-8}       # it stops gaining after about five
PCB=../px100-test-board.kicad_pcb
mkdir -p out
$P layout.py 2>&1 | grep -v "traits\|Fontconfig\|Debug\|assert"
$P route.py prep 2>&1 | grep -v "traits\|Fontconfig\|Debug\|assert"
cp $PCB out/prepped.kicad_pcb
# the item-selection strategies only differ in Freerouting's clean-up pass, so STRATEGIES defaults to one
for s in ${STRATEGIES:-prioritized}; do
  (cd out && FREEROUTING__GUI__ENABLED=false FREEROUTING__USAGE_AND_DIAGNOSTIC_DATA__DISABLE_ANALYTICS=true \
    $JAVA -Djava.awt.headless=true -jar "$JAR" -de px100-test-board.dsn -do route-$s.ses -mp $PASSES -is $s \
    -mt ${THREADS:-8} > freerouting-$s.log 2>&1) &
done
wait
best=""; bestn=99999
for s in ${STRATEGIES:-prioritized}; do
  [ -f out/route-$s.ses ] || { echo "$s: no result"; continue; }
  cp out/prepped.kicad_pcb $PCB
  $P route.py finish out/route-$s.ses 2>&1 | grep -v "traits\|Fontconfig\|Debug\|assert"
  $K pcb drc --format json --severity-all -o out/drc-$s.json $PCB > /dev/null 2>&1 || true
  n=$(python3 -c "import json; print(len(json.load(open('out/drc-$s.json'))['unconnected_items']))")
  echo "$s: unconnected $n"
  cp $PCB out/routed-$s.kicad_pcb
  if [ "$n" -lt "$bestn" ]; then bestn=$n; best=$s; fi
done
echo "best: $best ($bestn unconnected)"
cp out/routed-$best.kicad_pcb $PCB
cp out/route-$best.ses out/px100-test-board.ses
