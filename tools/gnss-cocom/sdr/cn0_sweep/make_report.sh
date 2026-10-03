#!/usr/bin/env bash
# The C/N0 sweep report, end to end.   make_report.sh [STEP ...]   (default: page)
#   scenarios the two flights' truth -> ../scenarios (gitignored; accuracy, table and charts make them if missing)
#   accuracy  pseudorange/Doppler errors per run -> work/acc_*.npz (PX1105R), work/m8tacc_*.npz (NEO-M8T),
#             work/f9pacc_*.npz (ZED-F9P), work/mosacc_*.npz (mosaic-G5)
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
F9P_TRAV="ftr12b ftr6b ftr0b ftrn6b"         # second F9P sweep; cap() names the flight kept for each level
F9P_HOT="fhs12b fhs6b fhs0b fhsn6b"
MOS_TRAV="g5tr12 g5tr6 g5tr0 g5trn6"            # mosaic-G5 (2026-10-02); cap() names the flight kept for each level
MOS_HOT="g5hs12 g5hs6 g5hs0 g5hsn6"
MOS_82="g5ga6 g5ss6"                             # the two 82 km flights, +6 dB

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
    ftr12b) echo "$C/zed_f9p_wideftr12b_signalsim_traveler_all_2026_57_w_p180.log" ;;
    ftr6b) echo "$C/zed_f9p_wideftr6br_signalsim_traveler_all_2026_51_w_p180.log" ;;
    ftr0b) echo "$C/zed_f9p_wideftr0br_signalsim_traveler_all_2026_45_w_p180_cofs.log" ;;
    ftrn6b) echo "$C/zed_f9p_wideftrn6b_signalsim_traveler_all_2026_45_w_p180_cofs.log" ;;
    fhs12b) echo "$C/zed_f9p_widefhs12br_signalsim_hotshot_all_2026_57_w_p180.log" ;;
    fhs6b) echo "$C/zed_f9p_widefhs6br_signalsim_hotshot_all_2026_51_w_p180.log" ;;
    fhs0b) echo "$C/zed_f9p_widefhs0brr_signalsim_hotshot_all_2026_45_w_p180.log" ;;
    fhsn6b) echo "$C/zed_f9p_widefhsn6br_signalsim_hotshot_all_2026_45_w_p180.log" ;;
    g5tr12) echo "$C/mosaic_g5_widemtr12_signalsim_traveler_all_2026_57_w_p180.log" ;;
    g5tr6) echo "$C/mosaic_g5_widemtr6_signalsim_traveler_all_2026_51_w_p180.log" ;;
    g5tr0) echo "$C/mosaic_g5_widemtr0r_signalsim_traveler_all_2026_45_w_p180_cofs.log" ;;
    g5trn6) echo "$C/mosaic_g5_widemtrn6b_signalsim_traveler_all_2026_45_w_p180_cofs.log" ;;
    g5trg) echo "$C/mosaic_g5_widemtr12r_signalsim_traveler_all_2026_57_w_p180.log" ;;     # the gate chart's traveler
    g5hs12) echo "$C/mosaic_g5_widemhs12_signalsim_hotshot_all_2026_57_w_p180.log" ;;
    g5hs6) echo "$C/mosaic_g5_widemhs6_signalsim_hotshot_all_2026_51_w_p180.log" ;;
    g5hs0) echo "$C/mosaic_g5_widemhs0r_signalsim_hotshot_all_2026_45_w_p180.log" ;;
    g5hsn6) echo "$C/mosaic_g5_widemhsn6r_signalsim_hotshot_all_2026_45_w_p180.log" ;;
    g5ga6) echo "$C/mosaic_g5_widemga6_signalsim_gentle_alt_all_2026_51_w_p180.log" ;;
    g5ss6) echo "$C/mosaic_g5_widemss6_signalsim_spaceshot_all_2026_51_w_p180.log" ;;
    *) echo "unknown run $1" >&2; exit 1 ;;
  esac
}

level() {   # run tag -> "level:how the level was made"
  case $1 in
    wp12|hs12|mtr12|mhs12|ftr12b|fhs12b|g5tr12|g5hs12) echo "+12:file generated at 57 dB-Hz" ;;
    wp6b|hs6|mtr6|mhs6|ftr6b|fhs6b|g5tr6|g5hs6|g5ga6|g5ss6) echo "+6:file generated at 51 dB-Hz" ;;
    wcr|hs0|mtr0|mhs0|ftr0b|fhs0b|g5tr0|g5hs0) echo "0:file generated at 45 dB-Hz" ;;
    wn3t) echo "-3:45 dB-Hz file with 3 dB of added noise" ;;
    wn6|hsn6|mtrn6b|mhsn6|ftrn6b|fhsn6b|g5trn6|g5hsn6) echo "-6:45 dB-Hz file with 6 dB of added noise" ;;
    wn9) echo "-9:45 dB-Hz file with 9 dB of added noise" ;;
  esac
}

scenarios() {   # the files SignalSim was given and the truth every script reads (origin 0 N, 119 W)
  python3 ../make_flights.py -o "$SC" --only traveler_soft25 --only hotshot --only gentle_alt --only spaceshot \
    --lat 0 --lon -119 > "$W/make_flights.log"
  python3 ../pad_scenario.py traveler_soft25 600 >> "$W/make_flights.log"
  python3 ../pad_scenario.py hotshot 600 1060 >> "$W/make_flights.log"     # the flown file stops at 1060 s
  python3 ../pad_scenario.py gentle_alt 600 >> "$W/make_flights.log"
  python3 ../pad_scenario.py spaceshot 600 >> "$W/make_flights.log"
}

need_scenarios() {
  [ -e "$SC/traveler_soft25_pad600.csv" ] && [ -e "$SC/hotshot_pad600.csv" ] && [ -e "$SC/gentle_alt_pad600.csv" ] \
    && [ -e "$SC/spaceshot_pad600.csv" ] || scenarios
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
  # the ZED-F9P's Doppler fits the truth with no delay (m8t_timing_scan.py on the traveler burn)
  for t in $F9P_TRAV; do
    python3 m8t_accuracy.py "$(cap $t)" "$W/f9pacc_$t.npz" --scenario traveler --rr-lag 0.0 > "$W/f9p_acc_$t.out"
  done
  for t in $F9P_HOT; do
    python3 m8t_accuracy.py "$(cap $t)" "$W/f9pacc_$t.npz" --scenario hotshot --rr-lag 0.0 > "$W/f9p_acc_$t.out"
  done
  # the mosaic-G5: errors over 100 km (channels left on a clock step by a transmitter underrun) set to NaN
  for t in $MOS_TRAV; do python3 mosaic_accuracy.py "$(cap $t)" "$W/mosacc_$t.npz" --scenario traveler > "$W/mosacc_$t.out"; done
  for t in $MOS_HOT; do python3 mosaic_accuracy.py "$(cap $t)" "$W/mosacc_$t.npz" --scenario hotshot > "$W/mosacc_$t.out"; done
  python3 mosaic_accuracy.py "$(cap g5ga6)" "$W/mosacc_g5ga6.npz" --scenario gentle_alt > "$W/mosacc_g5ga6.out"
  python3 mosaic_accuracy.py "$(cap g5ss6)" "$W/mosacc_g5ss6.npz" --scenario spaceshot > "$W/mosacc_g5ss6.out"
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
  RX=ZED-F9P BOOST_SCEN=hotshot python3 m8t_boost_traces.py "$W/f9p_boost" "$D/hot_boost.json" \
    "+12 dB=$(cap fhs12b)=$W/f9pacc_fhs12b.npz" "+6 dB=$(cap fhs6b)=$W/f9pacc_fhs6b.npz" \
    "0 dB=$(cap fhs0b)=$W/f9pacc_fhs0b.npz" "-6 dB=$(cap fhsn6b)=$W/f9pacc_fhsn6b.npz"
  RX=ZED-F9P BOOST_SCEN=traveler_soft25 python3 m8t_boost_traces.py "$W/f9p_trav_boost" "$D/boost_final.json" \
    "+12 dB=$(cap ftr12b)=$W/f9pacc_ftr12b.npz" "+6 dB=$(cap ftr6b)=$W/f9pacc_ftr6b.npz" \
    "0 dB=$(cap ftr0b)=$W/f9pacc_ftr0b.npz" "-6 dB=$(cap ftrn6b)=$W/f9pacc_ftrn6b.npz"
  mv "$W/f9p_boost.json" "$W/f9p_trav_boost.json" "$D/"
  mv "$W/f9p_boost.png" "$W/f9p_boost_compare.png" "$W/f9p_trav_boost.png" "$W/f9p_trav_boost_compare.png" "$F/"
  RX=mosaic-G5 RX_LIMIT=600 BOOST_SCEN=hotshot python3 m8t_boost_traces.py "$W/mos_boost" "$D/hot_boost.json" \
    "+12 dB=$(cap g5hs12)=$W/mosacc_g5hs12.npz" "+6 dB=$(cap g5hs6)=$W/mosacc_g5hs6.npz" \
    "0 dB=$(cap g5hs0)=$W/mosacc_g5hs0.npz" "-6 dB=$(cap g5hsn6)=$W/mosacc_g5hsn6.npz"
  RX=mosaic-G5 RX_LIMIT=600 BOOST_SCEN=traveler_soft25 python3 m8t_boost_traces.py "$W/mos_trav_boost" "$D/boost_final.json" \
    "+12 dB=$(cap g5tr12)=$W/mosacc_g5tr12.npz" "+6 dB=$(cap g5tr6)=$W/mosacc_g5tr6.npz" \
    "0 dB=$(cap g5tr0)=$W/mosacc_g5tr0.npz" "-6 dB=$(cap g5trn6)=$W/mosacc_g5trn6.npz"
  mv "$W/mos_boost.json" "$W/mos_trav_boost.json" "$D/"
  mv "$W/mos_boost.png" "$W/mos_boost_compare.png" "$W/mos_trav_boost.png" "$W/mos_trav_boost_compare.png" "$F/"
  python3 mosaic_gate_chart.py "$F/mos_gate.png" "$D/mos_gate.json" \
    "Hotshot (10 to 40 g), +6 dB=$(cap g5hs6)=hotshot=120" "Traveler (102 km), +12 dB=$(cap g5trg)=traveler_soft25=360" \
    "gentle_alt (3 g, 82.5 km), +6 dB=$(cap g5ga6)=gentle_alt=300" \
    "spaceshot (15 g, 82.5 km), +6 dB=$(cap g5ss6)=spaceshot=280"
  python3 px_err_rate.py "$F/px_err_rate.png"
  python3 m8t_err_rate.py "$F/m8t_err_rate.png"
  python3 m8t_err_rate.py "$F/f9p_err_rate.png" "$D/f9p_boost.json" "$D/f9p_trav_boost.json" ZED-F9P
  python3 m8t_err_rate.py "$F/mos_err_rate.png" "$D/mos_boost.json" "$D/mos_trav_boost.json" mosaic-G5 600
  python3 cmp3_boost.py "$F/cmp3_boost.png" "$D/hot_boost.json" "$D/m8t_boost.json" "$D/f9p_boost.json" \
    "$D/mos_boost.json" "$D/boost_final.json" "$D/m8t_trav_boost.json" "$D/f9p_trav_boost.json" "$D/mos_trav_boost.json"
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
  for t in $F9P_TRAV $F9P_HOT; do
    IFS=: read -r lab how <<< "$(level $t)"
    case $t in
      ftr*) scen=traveler_soft25; end=360; xm=375 ;;
      *) scen=hotshot; end=120; xm=125 ;;
    esac
    python3 plot_m8t_run.py "$(cap $t)" "$F/f9p_run_$t.png" \
      "ZED-F9P, SignalSim ${scen%%_*}, C/N0 $lab dB ($how), carrier corrected, buffered transmitter: GPS + Galileo + BeiDou, wide, 180 s pad, file ends T+$end" \
      "$scen" "$W/f9pacc_$t.npz" "end=$end" "xmax=$xm"
  done
  for t in $MOS_TRAV $MOS_HOT $MOS_82; do
    IFS=: read -r lab how <<< "$(level $t)"
    case $t in
      g5tr*) scen=traveler_soft25; end=360; xm=375 ;;
      g5ga6) scen=gentle_alt; end=300; xm=310 ;;
      g5ss6) scen=spaceshot; end=280; xm=290 ;;
      *) scen=hotshot; end=120; xm=125 ;;
    esac
    python3 plot_m8t_run.py "$(cap $t)" "$F/mos_run_$t.png" \
      "mosaic-G5, SignalSim ${scen%%_soft25}, C/N0 $lab dB ($how), carrier corrected, buffered transmitter: GPS + Galileo + BeiDou, wide, 180 s pad, file ends T+$end" \
      "$scen" "$W/mosacc_$t.npz" "end=$end" "xmax=$xm" limit=600
  done
}

page() {
  python3 build_report.py
}

export_html() {
  mkdir -p "$HERE/export"
  python3 export_standalone.py "$HERE/export/px1105r-neo-m8t-zed-f9p-mosaic-g5-cn0-sweep.html"
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
