# C/N0 sweep: PX1105R, NEO-M8T and ZED-F9P through two boosts

The report "PX1105R, NEO-M8T and ZED-F9P C/N0 Sweep" (COCOM bench, #491, flown 2026-09-30 and 10-01): how each
receiver keeps tracking through a simulated rocket boost, and how much that depends on signal strength. Two SignalSim
flights -- the traveler (space shot, burnout T+13 at 1498 m/s, apogee 102 km) and the hotshot (thrust 10 to 40 g over
4 s) -- were played into the PX1105R, the NEO-M8T and the ZED-F9P at levels from 9 dB under to 12 dB over the file's
own.

## What is here

- `figures/` -- every chart and run plot in the report.
- `data/` -- what the charts and the page are built from: the boost-chart JSONs (per satellite and epoch: line-of-sight
  Doppler rate, lock, delivery, pseudorange error) and the traveler flight table.
- `report_template.html` + `report_text.json` -- the page's layout and text; `build_report.py` fills them in.
- `make_report.sh` -- runs the pipeline. It writes `work/` (accuracy files, ephemeris cache, logs), `page/` (the
  report: open `page/index.html`) and `export/` (the same page as one self-contained HTML file); none is committed.

| Script | Does |
| --- | --- |
| `pr_accuracy.py` | PX1105R pseudorange and Doppler errors against the SignalSim truth, receiver clock solved per epoch |
| `m8t_accuracy.py` | the same for a u-blox receiver's RXM-RAWX (NEO-M8T, ZED-F9P; one signal per satellite); ephemerides from PX1105R captures of the same scenarios |
| `boost_traces_wide.py` | PX1105R boost chart: each satellite's line-of-sight Doppler rate, lock, delivery, wrong measurements |
| `m8t_boost_traces.py` | u-blox boost chart (`RX=ZED-F9P` for the F9P), and its comparison with the PX1105R at 515 m/s |
| `cmp3_boost.py` | all three receivers: satellites still tracked and within 10 m at 515 m/s, both flights |
| `plot_traveler_run.py`, `plot_m8t_run.py` | full-run plots (PX1105R; NEO-M8T and ZED-F9P) |
| `px_err_rate.py`, `m8t_err_rate.py` | pseudorange error against line-of-sight Doppler rate, all runs (the latter takes the receiver's JSONs) |
| `build_report.py`, `export_standalone.py` | the page, and the page as one HTML file with its images inlined |
| `traveler_summary.py`, `fix_runs.py`, `gaps.py`, `reentry_check.py`, `underrun_lines.py`, `sweep_table.py` | the traveler flight table |
| `m8t_summary.py`, `m8t_timing_scan.py`, `m8t_onset.py`, `m8t_burn_sats.py`, `m8t_intervals.py`, `px_wrong_stats.py`, `scen_facts.py`, `compare_px_json.py` | the numbers quoted in the text |
| `f9p_stats.py`, `f9p_alt_fix.py`, `f9p_hidden_alt.py`, `aug_f9p_alt.py`, `underrun_effect.py`, `f9p_comms.py` | the ZED-F9P numbers: per-level counts and errors, its fix above 80 km (and August's gate run), underruns, raw delivery and transmit buffers |
| `f9p_signals.py`, `f9p_ports.py` | ZED-F9P setup before each flight, after `../ubx_config.py --cold-start --raw --rate-hz 10 --dynmodel 8 --mon-comms`: GPS + Galileo + BeiDou only, and every port but USB silenced (RAM) |

## Rebuilding

```bash
./make_report.sh                                    # the page from data/ and figures/
./make_report.sh accuracy table charts page export  # everything, from the captures
CAPTURES=/path/to/captures ./make_report.sh accuracy charts page
```

Inputs: the flight captures (`../captures`, gitignored; archived with the bench data) and the two flights' truth in
`../scenarios` (gitignored), which `make_report.sh scenarios` regenerates byte for byte from `../make_flights.py`
(`traveler_soft25`, `hotshot`; origin 0 N, 119 W) and `../pad_scenario.py`. The accuracy step takes a minute or two
per run; `table` and `charts` read its output from `work/`.

| Flight | PX1105R runs | NEO-M8T runs | ZED-F9P runs |
| --- | --- | --- | --- |
| traveler | `wp12` +12, `wp6b` +6, `wcr` 0, `wn3t` -3, `wn6` -6, `wn9` -9 dB | `mtr12`, `mtr6`, `mtr0`, `mtrn6b` (+12 to -6 dB) | `ftr12b`, `ftr6b`, `ftr0b`, `ftrn6b` |
| hotshot | `hs12`, `hs6`, `hs0`, `hsn6` (+12 to -6 dB) | `mhs12`, `mhs6`, `mhs0`, `mhsn6` (+12 to -6 dB) | `fhs12b`, `fhs6b`, `fhs0b`, `fhsn6b` |

The ZED-F9P names are its second sweep's levels; `make_report.sh`'s `cap()` gives the flight kept for each (a trailing
`r` or `rr` on the capture is a re-flight).

## Choices worth knowing

- **Wrong** = a measurement the receiver flagged valid (NEO-M8T) or delivered (PX1105R) that is more than 10 m off the
  truth. Good satellites never passed 6.3 m. PX1105R rows whose receiver clock had to be bridged by the Doppler are not
  judged, and the run plots mark wrong stretches only from T-5 to T+30: elsewhere transmitter underruns make brief
  errors of the rig's own.
- The NEO-M8T's errors are taken at its own time tags, as the PX1105R's are: its clock runs about 3 ms ahead of GPS
  time, but re-timing the truth by that clock fitted the burn worse (`m8t_timing_scan.py`). Its Doppler fits the truth
  0.01 s earlier, the PX1105R's 0.03 s.
- The ZED-F9P's too: its pseudoranges fit best about 11 ms after their tags (the M8T's about 5 ms), left in, which
  adds a few metres that grow with speed; its Doppler fits with no delay (`--rr-lag 0.0`).
- The ZED-F9P must have every port but USB silenced (`f9p_ports.py`): the board's saved NMEA on UART1 (38400 baud) and
  I2C backed up its transmit buffers at 10 Hz and starved the raw messages on USB. Its first sweep, flown that way, is
  not used.
- An underrun before the T-60..T+30 window can still spoil an F9P flight: the receiver steps its clock by the gap and a
  channel that misses the step stays that far off (`underrun_effect.py`); the hotshot +12 dB flight was flown again for
  that.
- The sky reference levels (C/N0 on the roof antenna) are quoted in `report_text.json`; the roof logs behind them are
  not committed, because their satellite geometry is enough to locate the antenna.

Not here: the SignalSim files and the bench tooling that flew them (the buffered transmitter with noise injection and
carrier correction, and the flight queues). `f9p_signals.py` and `f9p_ports.py` are here because the F9P's results
depend on them.
