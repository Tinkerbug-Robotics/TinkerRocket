#!/bin/bash
# Local Docker OpenFOAM sweep for the 100mm L2 rocket fin tab.
# 33 cases: 11 deflection angles x 3 velocities (40/80/120 m/s).
# 2 concurrent cases x 4 MPI procs. endTime=1000, forces on finTab patch.
#
# Usage:
#   bash run_sweep.sh          # full sweep (skips DONE cases)
#   bash run_sweep.sh pilot    # single case V80_10deg with diagnostics
set -u
ROOT="/Users/christianpedersen/Documents/Hobbies/ModelRockets/CFD/100mm-L2-Rocket-Fin-Tab-Analysis"
TPL="$ROOT/case_template"
WORK="/private/tmp/claude-501/cfd_run_100mm"
RES="$ROOT/results"
IMG="opencfd/openfoam-default:2412"
NPROCS=4; MAXJOBS=2
VELS="80 120 40"
ANGS="0 10 n10 20 n20 5 n5 15 n15 2 n2"
mkdir -p "$WORK" "$RES" "$RES/logs"

run_one(){
  local V=$1 A=$2
  local name="V${V}_${A}deg"
  local C="$WORK/$name"
  [ -f "$C/DONE" ] && { echo "skip $name"; return; }
  rm -rf "$C"; cp -r "$TPL" "$C"; mkdir -p "$C/constant/triSurface"
  cp "$ROOT/geometry/fincan_${A}deg.stl" "$C/constant/triSurface/fincan.stl"
  local K=$(python3 -c "print(f'{1.5*($V*0.01)**2:.4f}')")
  local OM=$(python3 -c "import math;k=$K;print(f'{math.sqrt(k)/(0.09**0.25*0.01):.1f}')")
  sed -i '' "s/uniform (0 0 95)/uniform (0 0 $V)/g" "$C/0/U"
  sed -i '' "s/uniform 1.35/uniform $K/g" "$C/0/k"
  sed -i '' "s/uniform 212/uniform $OM/g" "$C/0/omega"
  docker run --rm -v "$C":/case -w /case -e HOME=/case "$IMG" bash /case/Allrun $NPROCS > "$C/log.docker" 2>&1
  local L=$(ls -1 "$C/postProcessing/forces_finTab/" 2>/dev/null | sort -n | tail -1)
  if [ -n "$L" ] && [ -f "$C/postProcessing/forces_finTab/$L/force.dat" ]; then
    cp "$C/postProcessing/forces_finTab/$L/force.dat"  "$RES/${name}_force.dat"
    cp "$C/postProcessing/forces_finTab/$L/moment.dat" "$RES/${name}_moment.dat"
    # also keep total fin+tab forces for drag/report use
    local LA=$(ls -1 "$C/postProcessing/forces_all/" 2>/dev/null | sort -n | tail -1)
    [ -n "$LA" ] && cp "$C/postProcessing/forces_all/$LA/force.dat" "$RES/${name}_forceAll.dat" 2>/dev/null
    # convergence evidence for the report
    { grep -c "^Time = " "$C/log.simpleFoam" 2>/dev/null | sed 's/^/iterations: /';
      grep "Solving for Ux" "$C/log.simpleFoam" 2>/dev/null | tail -1;
      grep -A3 "Mesh OK\|cells:" "$C/log.snappyHexMesh" 2>/dev/null | tail -5;
      tail -30 "$C/log.docker"; } > "$RES/logs/${name}.log" 2>&1
    touch "$C/DONE"; echo "DONE $name"
  else
    echo "FAIL $name"; tail -5 "$C/log.docker"
    cp "$C/log.docker" "$RES/logs/${name}.FAIL.log" 2>/dev/null
  fi
}
export -f run_one; export ROOT TPL WORK RES IMG NPROCS

if [ "${1:-}" = "pilot" ]; then
  echo "=== PILOT: V80_10deg ==="
  time run_one 80 10
  C="$WORK/V80_10deg"
  echo "--- mesh ---";  grep -E "cells:|Cells:" "$C/log.docker" | head -5
  echo "--- snappy tail ---"; tail -15 "$C/log.snappyHexMesh" 2>/dev/null
  echo "--- simpleFoam tail ---"; grep "Solving for Ux" "$C/log.simpleFoam" | tail -3
  echo "--- last force rows (finTab) ---"
  L=$(ls -1 "$C/postProcessing/forces_finTab/" 2>/dev/null | sort -n | tail -1)
  tail -3 "$C/postProcessing/forces_finTab/$L/force.dat" 2>/dev/null
  tail -3 "$C/postProcessing/forces_finTab/$L/moment.dat" 2>/dev/null
  exit 0
fi

N=0
for V in $VELS; do for A in $ANGS; do
  run_one "$V" "$A" &
  N=$((N+1)); [ $((N % MAXJOBS)) -eq 0 ] && wait
done; done
wait
echo "SWEEP COMPLETE $(date)"
