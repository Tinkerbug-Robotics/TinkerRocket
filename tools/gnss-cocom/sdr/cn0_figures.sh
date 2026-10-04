#!/usr/bin/env bash
# The Doppler-rate trace figures of cn0_boost_report.html, from the bench captures in captures/
# (2026-09-28/29; conducted HackRF gain 3). Writes results/figures/cn0_*.svg; the PNG/JSON side
# outputs go to a temp directory.
#   PX1105R: traveler (step start) C/N0 ladder, 10 dB pad (~75 = no pad), and the 38.4 dB-Hz checks
#            (0.25 s / 0.5 s soft start, stock gps-sdr-sim build, spread power); hotshot, 10 dB pad,
#            fixed gain 64 down to 4 (2026-09-29, the same files as the NEO-M8T's hotshot); 44 is the
#            repeat (_r2: the first flight had the SkyTraq 13 s cycle strike 1 s before ignition) and
#            41 was flown twice, both shown.
#   NEO-M8T: traveler 0.25 s soft start and hotshot, 30 dB in the cable, fixed gain 64 down to 4.
# Levels: "expected" dB-Hz on the PX1105R-calibrated scale (65 + 20 log10(gain/128) - extra pad);
# "reads" = the M8T's own pad reading. A * marks a PX1105R capture with its 13 s collapse cycle.
set -euo pipefail
cd "$(dirname "$0")/captures"
TMP=$(mktemp -d)
FIGS=../results/figures
PT=px1105r_traveler_pad600_smooth_cofs
M=neo_m8t_night30db

../plot_doppler_traces.py "$TMP/px_ladder" --compare --svg \
    "~75 dB-Hz (no pad):75=${PT}_gain3_nav9_el3_pmnormal.log" \
    "48 dB-Hz:48.0=${PT}_g18_gain3_nav9_el3_pad10db.log" \
    "45 dB-Hz*:45.1=${PT}_g13_gain3_nav9_el3_pad10db.log" \
    "41 dB-Hz*:40.9=${PT}_g8_gain3_nav9_el3_pad10db.log" \
    "38.4 dB-Hz:38.4=${PT}_g6_gain3_nav9_el3_pad10db.log" \
    "34.9 dB-Hz:34.9=${PT}_g4_gain3_nav9_el3_pad10db.log" | grep "tracked at ignition"
cp "$TMP/px_ladder_traces.svg" "$FIGS/cn0_px1105r_traveler.svg"

../plot_doppler_traces.py "$TMP/px_checks" --compare --svg \
    "step start, smooth build:38.4=${PT}_g6_gain3_nav9_el3_pad10db.log" \
    "0.25 s soft start:38.4:traveler_soft25=px1105r_h5_soft25_g6_gain3_nav9_el3_h5.log" \
    "0.5 s soft start:38.4:traveler_soft=px1105r_h5_soft_g6_gain3_nav9_el3_h5.log" \
    "step start, stock build:38.4=px1105r_h5_stock_g6_gain3_nav9_el3_h5.log" \
    "spread power* (dB-Hz at right):pl-17.3=px1105r_h5_spread_gain3_nav9_el3_h5.log" | grep "tracked at ignition"
cp "$TMP/px_checks_traces.svg" "$FIGS/cn0_px1105r_checks.svg"

PH=px1105r_hotshot_pad600_smooth_cofs
../plot_doppler_traces.py "$TMP/px_hotshot" --compare --svg \
    "59 dB-Hz:59.0:hotshot=${PH}_g64_gain3_nav9_el3_hotshot10db.log" \
    "56 dB-Hz:55.9:hotshot=${PH}_g45_gain3_nav9_el3_hotshot10db.log" \
    "53 dB-Hz:53.0:hotshot=${PH}_g32_gain3_nav9_el3_hotshot10db.log" \
    "50 dB-Hz:50.1:hotshot=${PH}_g23_gain3_nav9_el3_hotshot10db.log" \
    "47 dB-Hz:46.9:hotshot=${PH}_g16_gain3_nav9_el3_hotshot10db.log" \
    "44 dB-Hz (repeat):43.7:hotshot=${PH}_g11_gain3_nav9_el3_hotshot10db_r2.log" \
    "41 dB-Hz:40.9:hotshot=${PH}_g8_gain3_nav9_el3_hotshot10db.log" \
    "41 dB-Hz (repeat):40.9:hotshot=${PH}_g8_gain3_nav9_el3_hotshot10db_r2.log" \
    "38.4 dB-Hz:38.4:hotshot=${PH}_g6_gain3_nav9_el3_hotshot10db.log" \
    "34.9 dB-Hz:34.9:hotshot=${PH}_g4_gain3_nav9_el3_hotshot10db.log" | grep "tracked at ignition"
cp "$TMP/px_hotshot_traces.svg" "$FIGS/cn0_px1105r_hotshot.svg"
# its own speed against the truth, over the satellites it lists: strong, middling and sky-like signal
../plot_own_lag.py "$FIGS/cn0_px1105r_own_lag.svg" hotshot 4.2 \
    "59 dB-Hz=${PH}_g64_gain3_nav9_el3_hotshot10db.log" \
    "53 dB-Hz=${PH}_g32_gain3_nav9_el3_hotshot10db.log" \
    "38.4 dB-Hz=${PH}_g6_gain3_nav9_el3_hotshot10db.log" > /dev/null

for traj in traveler_soft25 hotshot; do
    args=()
    for lv in "64 39 44" "45 36 42" "32 33 40" "23 30 37" "16 27 37" "11 24 34" "8 21 32" "6 18 29" "4 15 26"; do
        read -r g px rd <<< "$lv"
        args+=("gain $g + 30 dB, $px expected, reads $rd:$rd:$traj=${M}_${traj}_pad600_smooth_cofs_g$g.log")
    done
    ../plot_doppler_traces.py "$TMP/m8t_$traj" --compare --svg "${args[@]}" | grep "tracked at ignition"
    cp "$TMP/m8t_${traj}_traces.svg" "$FIGS/cn0_m8t_${traj%_soft25}.svg"
done
# the real sky: both receivers on the same outdoor antenna, 20 min each (2026-09-29). The captures
# hold the antenna position and stay local; the JSON and the figure carry only elevation and C/N0.
../sky_cn0.py px1105r_sky_20260929_1238.log --json "$TMP/sky_px.json" > /dev/null
../sky_cn0.py neo_m8t_sky_20260929.log --json "$TMP/sky_m8t.json" > /dev/null
python3 ../plot_sky_cn0.py "$TMP/sky_px.json" "$TMP/sky_m8t.json"
rm -rf "$TMP"
# matplotlib writes a fixed size in points on the root <svg>; drop it and keep the viewBox so the
# figure scales to the report's column like the hand-drawn ones
for f in "$FIGS"/cn0_*.svg; do
    perl -0pi -e 's/(<svg[^>]*?) width="[0-9.]+pt" height="[0-9.]+pt"/$1/' "$f"
done
ls -la "$FIGS"/cn0_*.svg
