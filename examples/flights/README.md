# Example flight logs

Four real flights from a single day at CENJARS, 2026-07-05, kept here so the tooling in
[`Data_Analysis/flight_report/`](../../Data_Analysis/flight_report/) has something to run
against without asking you to fly first.

Vehicles also get per-rocket folders here as they move through build and analysis:

| Folder | Vehicle | Status |
|--------|---------|--------|
| [`L2_100mm_Rolly_Polly_V/`](L2_100mm_Rolly_Polly_V/) | Rolly Polly V, 100 mm L2 | Pre-flight: CFD fin-tab sweep + first-flight roll PID tune |

| File | Vehicle | Size | Max altitude | Max speed | Apogee | Burnout | Firmware |
|------|---------|------|--------------|-----------|--------|---------|----------|
| [`flight_20260705_174532.bin`](flight_20260705_174532.bin) | Rolly Polly, 54 mm | 5.2 MB | 357 m | 98 m/s | 8.0 s | 1.4 s | `0c8c132` |
| [`flight_20260705_183745.bin`](flight_20260705_183745.bin) | V2, 2.6 in — flight 1 | 1.6 MB | 109 m | 50 m/s | 6.1 s | 0.7 s | `0c8c132` |
| [`flight_20260705_191300.bin`](flight_20260705_191300.bin) | Rolly Polly, 67 mm large fin | 2.5 MB | 175 m | 55 m/s | 5.8 s | 0.9 s | `bbd2630` |
| [`flight_20260705_195028.bin`](flight_20260705_195028.bin) | V2, 2.6 in — flight 2 | 1.0 MB | 74 m | 38 m/s | 3.1 s | 0.7 s | `—` |

The figures come from each flight's `.json` sidecar, which the rocket writes alongside the
log. `Firmware` is the git SHA the flight was flown on — worth checking before drawing
conclusions from a log, since analysis code and flight code both move.

## What's here, and what isn't

| | |
|---|---|
| **`.bin`** | The log itself. Framed binary as written to the rocket's flash — this is the source of truth |
| **`.json`** | Flight summary: apogee, burnout, maxima, and the settings the flight was flown with |
| ~~`.csv`~~ | **Not committed.** Regenerated from the `.bin`, and about four times its size |
| ~~`_report.html`~~ | **Not committed.** Same reason — regenerate it |

`.gitignore` enforces that split, so a generated CSV or report sitting in this folder cannot
be committed by accident.

## Running the tooling

```bash
python -m Data_Analysis.flight_report run examples/flights/flight_20260705_174532.bin
python -m Data_Analysis.flight_report run examples/flights          # all four
```

Each report lands next to its `.bin` as `<stem>_report.html`.

## Provenance

These are unedited captures from real flights, not synthetic data — which is the point. They
carry the sensor noise, dropouts, and timing jitter that synthetic data does not, and several
of the analysis modules exist because of behaviour first seen in logs like these.

Vehicle names come from the flight archive's own folder layout; the timestamps in the
filenames are the rocket's, set from the phone at the pad.
