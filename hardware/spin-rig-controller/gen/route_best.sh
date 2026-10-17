#!/bin/bash
# Place once, route three ways in parallel (Freerouting item-selection strategies), finish + fix each on a copy,
# keep the one with the fewest unconnected items.
set -e
cd "$(dirname "$0")"
K=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
P=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
JAVA=${JAVA:-/opt/homebrew/opt/openjdk/bin/java}
JAR=${FREEROUTING_JAR:?set FREEROUTING_JAR to a Freerouting 2.x jar (github.com/freerouting/freerouting)}
PASSES=${PASSES:-25}
mkdir -p out
$K sch export netlist --format kicadsexpr -o out/spin-rig-controller.net ../spin-rig-controller.kicad_sch 2>&1 | grep -v Fontconfig || true
$P make_pcb.py place 2>&1 | grep -v "traits\|Fontconfig"
cp ../spin-rig-controller.kicad_pcb out/placed.kicad_pcb
for s in prioritized random sequential; do
  (cd out && FREEROUTING__GUI__ENABLED=false FREEROUTING__USAGE_AND_DIAGNOSTIC_DATA__DISABLE_ANALYTICS=true \
    $JAVA -Djava.awt.headless=true -jar "$JAR" -de spin-rig-controller.dsn -do route-$s.ses -mp $PASSES -is $s -mt 3 \
    > freerouting-$s.log 2>&1) &
done
wait
best=""; bestn=99999
for s in prioritized random sequential; do
  [ -f out/route-$s.ses ] || continue
  cp out/placed.kicad_pcb ../spin-rig-controller.kicad_pcb
  cp out/route-$s.ses out/spin-rig-controller.ses
  $P make_pcb.py finish > /dev/null 2>&1
  python3 check.py > /dev/null
  $P fixup.py > out/fixup-$s.log 2>&1 || true
  python3 check.py > out/check-$s.txt
  n=$(grep "^unconnected_items:" out/check-$s.txt | awk '{print $2}')
  e=$(grep -c " error " out/check-$s.txt || true)
  echo "$s: unconnected $n, error lines $e"
  cp ../spin-rig-controller.kicad_pcb out/routed-$s.kicad_pcb
  if [ "$n" -lt "$bestn" ]; then bestn=$n; best=$s; fi
done
echo "best: $best ($bestn unconnected)"
cp out/routed-$best.kicad_pcb ../spin-rig-controller.kicad_pcb
cp out/route-$best.ses out/spin-rig-controller.ses
python3 check.py
