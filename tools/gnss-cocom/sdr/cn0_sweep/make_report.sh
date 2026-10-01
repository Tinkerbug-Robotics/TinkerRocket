#!/usr/bin/env bash
# The C/N0 sweep report, end to end.   make_report.sh [STEP ...]   (default: page)
#   scenarios the two flights' truth -> ../scenarios (gitignored; accuracy, table and charts make them if missing)
#   accuracy  pseudorange/Doppler errors per run -> work/acc_*.npz (PX1105R), work/m8tacc_*.npz (NEO-M8T)
#   table     the traveler flight table -> data/sweep_table.json (needs accuracy)
#   charts    boost charts and their data -> figures/, data/; error-vs-rate charts; run plots (needs accuracy)
#   page      page/index.html from report_template.html, report_text.json, data/ and figures/
#   export    page/ as one self-contained HTML file -> export/
# Captures from $CAPTURES (default ../captures, gitignored), scenarios from ../scenarios.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
C=${CAPTURES:-$HERE/../captures}
SC=$HERE/../scenarios
W=$HERE/work
D=$HERE/data
F=$HERE/figures
mkdir -p "$W" "$D" "$F"
cd "$HERE"

PX_TRAV="wp12 wp6b wcr wn3t wn6 wn9"
PX_HOT="hs12 hs6 hs0 hsn6"
M8T_TRAV="mtr12 mtr6 mtr0 mtrn6b"
M8T_HOT="mhs12 mhs6 mhs0 mhsn6"

cap() {     # run tag -> its capture
  case $1 in
    wp12) echo "$C/px1105r_signalsim_traveler_all_2026_57_w_p180_gain3_nav9_el3_traveler2026wp12.log" ;;
    wp6b) echo "$C/px1105r_signalsim_traveler_all_2026_51_w_p180_gain3_nav9_el3_traveler2026wp6b.log" ;;
    wcr|wn3t|wn6|wn9) echo "$C/px1105r_signalsim_traveler_all_2026_45_w_p180_cofs_gain3_nav9_el3_traveler2026$1.log" ;;
    hs12) echo "$C/px1105r_signalsim_hotshot_all_2026_57_w_p180_gain3_nav9_el3_hotshot2026hs12.log" ;;
    hs6) echo "$C/px1105r_signalsim_hotshot_all_2026_51_w_p180_gain3_nav9_el3_hotshot2026hs6.log" ;;
    hs0|hsn6) echo "$C/px1105r_signalsim_hotshot_all_2026_45_w_p180_gain3_nav9_el3_hotshot2026$1.log" ;;
    mtr12) echo "$C/neo_m8t_widemtr12_signalsim_traveler_all_2026_57_w_p180.log" ;;
    mtr6) echo "$C/neo_m8t_widemtr6_signalsim_traveler_all_2026_51_w_p180.log" ;;
    mtr0|mtrn6b) echo "$C/neo_m8t_wide$1_signalsim_traveler_all_2026_45_w_p180_cofs.log" ;;
    mhs12) echo "$C/neo_m8t_widemhs12_signalsim_hotshot_all_2026_57_w_p180.log" ;;
    mhs6) echo "$C/neo_m8t_widemhs6_signalsim_hotshot_all_2026_51_w_p180.log" ;;
    mhs0|mhsn6) echo "$C/neo_m8t_wide$1_signalsim_hotshot_all_2026_45_w_p180.log" ;;
    *) echo "unknown run $1" >&2; exit 1 ;;
  esac
}

level() {   # run tag -> "level:how the level was made"
  case $1 in
    wp12|hs12|mtr12|mhs12) echo "+12:file generated at 57 dB-Hz" ;;
    wp6b|hs6|mtr6|mhs6) echo "+6:file generated at 51 dB-Hz" ;;
    wcr|hs0|mtr0|mhs0) echo "0:file generated at 45 dB-Hz" ;;
    wn3t) echo "-3:45 dB-Hz file with 3 dB of added noise" ;;
    wn6|hsn6|mtrn6b|mhsn6) echo "-6:45 dB-Hz file with 6 dB of added noise" ;;
    wn9) echo "-9:45 dB-Hz file with 9 dB of added noise" ;;
  esac
}

scenarios() {   # the files SignalSim was given and the truth every script reads (origin 0 N, 119 W)
  python3 ../make_flights.py -o "$SC" --only traveler_soft25 --only hotshot --lat 0 --lon -119 > "$W/make_flights.log"
  python3 ../pad_scenario.py traveler_soft25 600 >> "$W/make_flights.log"
  python3 ../pad_scenario.py hotshot 600 1060 >> "$W/make_flights.log"     # the flown file stops at 1060 s
}

need_scenarios() {
  [ -e "$SC/traveler_soft25_pad600.csv" ] && [ -e "$SC/hotshot_pad600.csv" ] || scenarios
}

accuracy() {
  need_scenarios
  for t in $PX_TRAV; do
    python3 pr_accuracy.py "$(cap $t)" "$W/acc_$t.npz" > "$W/acc_$t.txt"
  done
  for t in $PX_HOT; do
    python3 pr_accuracy.py "$(cap $t)" "$W/acc_$t.npz" --csv "$SC/hotshot_pad600.csv" \
      --scenario "$SC/hotshot.json" > "$W/acc_$t.txt"
  done
  for t in $M8T_TRAV; do python3 m8t_accuracy.py "$(cap $t)" "$W/m8tacc_$t.npz" --scenario traveler > "$W/m8t_runs_$t.out"; done
  for t in $M8T_HOT; do python3 m8t_accuracy.py "$(cap $t)" "$W/m8tacc_$t.npz" --scenario hotshot > "$W/m8t_runs_$t.out"; done
}

table() {
  local labels=("+12 dB (57 dB-Hz file)" "+6 dB (51 dB-Hz file)" "0 dB (45 dB-Hz file)" "-3 dB" "-6 dB" "-9 dB")
  local args=() i=0 t c
  need_scenarios
  for t in $PX_TRAV; do
    c=$(cap $t)
    { echo "== gaps and underruns"; python3 gaps.py "$c" 18480000 2 180
      echo; echo "== re-entry and descent, second by second"; python3 reentry_check.py "$c" 18480000 255 362
      echo; echo "== traveler summary"; python3 traveler_summary.py "$c"
      echo; echo "== fix runs"; python3 fix_runs.py "$c" -10 0.35
      echo; echo "== accuracy"; cat "$W/acc_$t.txt"; } > "$W/sweep_post_$t.out" 2>&1
    args+=("${labels[$i]}=$W/sweep_post_$t.out=$W/acc_$t.npz")
    i=$((i + 1))
  done
  python3 sweep_table.py "$D/boost_final.json" "${args[@]}" --json "$D/sweep_table.json"
}

charts() {
  local t lab how scen end xm
  need_scenarios
  python3 boost_traces_wide.py "$W/boost_final" \
    "+12 dB (57 dB-Hz file)=$(cap wp12)=$W/acc_wp12.npz" "+6 dB (51 dB-Hz file)=$(cap wp6b)=$W/acc_wp6b.npz" \
    "0 dB (45 dB-Hz file)=$(cap wcr)=$W/acc_wcr.npz" "-3 dB=$(cap wn3t)=$W/acc_wn3t.npz" \
    "-6 dB=$(cap wn6)=$W/acc_wn6.npz" "-9 dB=$(cap wn9)=$W/acc_wn9.npz"
  BOOST_SCEN=hotshot python3 boost_traces_wide.py "$W/hot_boost" \
    "+12 dB=$(cap hs12)=$W/acc_hs12.npz" "+6 dB=$(cap hs6)=$W/acc_hs6.npz" \
    "0 dB=$(cap hs0)=$W/acc_hs0.npz" "-6 dB=$(cap hsn6)=$W/acc_hsn6.npz"
  mv "$W/boost_final.json" "$W/hot_boost.json" "$D/"
  mv "$W/boost_final.png" "$W/hot_boost.png" "$F/"
  BOOST_SCEN=hotshot python3 m8t_boost_traces.py "$W/m8t_boost" "$D/hot_boost.json" \
    "+12 dB=$(cap mhs12)=$W/m8tacc_mhs12.npz" "+6 dB=$(cap mhs6)=$W/m8tacc_mhs6.npz" \
    "0 dB=$(cap mhs0)=$W/m8tacc_mhs0.npz" "-6 dB=$(cap mhsn6)=$W/m8tacc_mhsn6.npz"
  BOOST_SCEN=traveler_soft25 python3 m8t_boost_traces.py "$W/m8t_trav_boost" "$D/boost_final.json" \
    "+12 dB=$(cap mtr12)=$W/m8tacc_mtr12.npz" "+6 dB=$(cap mtr6)=$W/m8tacc_mtr6.npz" \
    "0 dB=$(cap mtr0)=$W/m8tacc_mtr0.npz" "-6 dB=$(cap mtrn6b)=$W/m8tacc_mtrn6b.npz"
  mv "$W/m8t_boost.json" "$W/m8t_trav_boost.json" "$D/"
  mv "$W/m8t_boost.png" "$W/m8t_boost_compare.png" "$W/m8t_trav_boost.png" "$W/m8t_trav_boost_compare.png" "$F/"
  python3 px_err_rate.py "$F/px_err_rate.png"
  python3 m8t_err_rate.py "$F/m8t_err_rate.png"
  for t in $PX_TRAV; do
    IFS=: read -r lab how <<< "$(level $t)"
    python3 plot_traveler_run.py "$(cap $t)" "$F/traveler_acc_$t.png" \
      "PX1105R, SignalSim traveler, C/N0 $lab dB ($how), carrier corrected, buffered transmitter: GPS + Galileo + BeiDou, wide, 180 s pad, file ends T+360" \
      "$W/acc_$t.npz" end=360
  done
  for t in $PX_HOT; do
    IFS=: read -r lab how <<< "$(level $t)"
    python3 plot_traveler_run.py "$(cap $t)" "$F/hotshot_acc_$t.png" \
      "PX1105R, SignalSim hotshot, C/N0 $lab dB ($how), carrier corrected, buffered transmitter: GPS + Galileo + BeiDou, wide, 180 s pad, file ends T+120" \
      hotshot "$W/acc_$t.npz" end=120 xmax=125
  done
  for t in $M8T_TRAV $M8T_HOT; do
    IFS=: read -r lab how <<< "$(level $t)"
    case $t in
      mtr*) scen=traveler_soft25; end=360; xm=375 ;;
      *) scen=hotshot; end=120; xm=125 ;;
    esac
    python3 plot_m8t_run.py "$(cap $t)" "$F/m8t_run_$t.png" \
      "NEO-M8T, SignalSim ${scen%%_*}, C/N0 $lab dB ($how), carrier corrected, buffered transmitter: GPS + Galileo + BeiDou, wide, 180 s pad, file ends T+$end" \
      "$scen" "$W/m8tacc_$t.npz" "end=$end" "xmax=$xm"
  done
}

page() {
  python3 build_report.py
}

export_html() {
  mkdir -p "$HERE/export"
  python3 export_standalone.py "$HERE/export/px1105r-neo-m8t-cn0-sweep.html"
}

for step in ${@:-page}; do
  case $step in
    scenarios) scenarios ;;
    accuracy) accuracy ;;
    table) table ;;
    charts) charts ;;
    page) page ;;
    export) export_html ;;
    *) echo "unknown step: $step (scenarios accuracy table charts page export)" >&2; exit 1 ;;
  esac
done
