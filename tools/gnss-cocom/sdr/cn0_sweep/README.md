# C/N0 sweep: PX1105R and NEO-M8T through two boosts

The report "PX1105R and NEO-M8T C/N0 Sweep" (COCOM bench, #491, flown 2026-09-30 and 10-01): how each receiver keeps
tracking through a simulated rocket boost, and how much that depends on signal strength. Two SignalSim flights -- the
traveler (space shot, burnout T+13 at 1498 m/s, apogee 102 km) and the hotshot (thrust 10 to 40 g over 4 s) -- were
played into the PX1105R and the NEO-M8T at levels from 9 dB under to 12 dB over the file's own.

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
| `m8t_accuracy.py` | the same for the NEO-M8T's RXM-RAWX; its ephemerides come from PX1105R captures of the same scenarios |
| `boost_traces_wide.py` | PX1105R boost chart: each satellite's line-of-sight Doppler rate, lock, delivery, wrong measurements |
| `m8t_boost_traces.py` | NEO-M8T boost chart, and the PX1105R-against-NEO-M8T comparison at 515 m/s |
| `plot_traveler_run.py`, `plot_m8t_run.py` | full-run plots (PX1105R, NEO-M8T) |
| `px_err_rate.py`, `m8t_err_rate.py` | pseudorange error against line-of-sight Doppler rate, all runs |
| `build_report.py`, `export_standalone.py` | the page, and the page as one HTML file with its images inlined |
| `traveler_summary.py`, `fix_runs.py`, `gaps.py`, `reentry_check.py`, `underrun_lines.py`, `sweep_table.py` | the traveler flight table |
| `m8t_summary.py`, `m8t_timing_scan.py`, `m8t_onset.py`, `m8t_burn_sats.py`, `m8t_intervals.py`, `px_wrong_stats.py`, `scen_facts.py`, `compare_px_json.py` | the numbers quoted in the text |

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

| Flight | PX1105R runs | NEO-M8T runs |
| --- | --- | --- |
| traveler | `wp12` +12, `wp6b` +6, `wcr` 0, `wn3t` -3, `wn6` -6, `wn9` -9 dB | `mtr12`, `mtr6`, `mtr0`, `mtrn6b` (+12 to -6 dB) |
| hotshot | `hs12`, `hs6`, `hs0`, `hsn6` (+12 to -6 dB) | `mhs12`, `mhs6`, `mhs0`, `mhsn6` (+12 to -6 dB) |

## Choices worth knowing

- **Wrong** = a measurement the receiver flagged valid (NEO-M8T) or delivered (PX1105R) that is more than 10 m off the
  truth. Good satellites never passed 6.3 m. PX1105R rows whose receiver clock had to be bridged by the Doppler are not
  judged, and the run plots mark wrong stretches only from T-5 to T+30: elsewhere transmitter underruns make brief
  errors of the rig's own.
- The NEO-M8T's errors are taken at its own time tags, as the PX1105R's are: its clock runs about 3 ms ahead of GPS
  time, but re-timing the truth by that clock fitted the burn worse (`m8t_timing_scan.py`). Its Doppler fits the truth
  0.01 s earlier, the PX1105R's 0.03 s.
- The sky reference levels (C/N0 on the roof antenna) are quoted in `report_text.json`; the roof logs behind them are
  not committed, because their satellite geometry is enough to locate the antenna.

Not here: the SignalSim files and the bench tooling that flew them (the buffered transmitter with noise injection and
carrier correction, and the flight queues).
