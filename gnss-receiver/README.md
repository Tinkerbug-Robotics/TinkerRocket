# GNSS receiver, stage 0: the software receiver on IQ files

Our own receiver, written from the ICDs and the textbooks, run first on recorded and
simulated IQ files. The design is in `docs/plans/gnss-receiver-review.md` and
`docs/plans/gnss-receiver-architecture.md`:
- a MAX2769B front end (2-bit I/Q at 27 MS/s from a 27 MHz reference);
- an FPGA that decimates L1 to 6.75 MS/s and runs the correlators;
- an ESP32-P4 that does everything else.

Pocket SDR is the reference and cross-check. It runs as a separate program on the same
input, and none of its code is copied here (see [Licences](#licences)).

## Layout

| Path | What | Runs on |
|---|---|---|
| `core/` | The P4 firmware-to-be. Portable C99: no OS calls, no malloc after init. Only the signal/band IDs exist so far | Host now; an ESP-IDF component later |
| `fpga/model/` | Bit-exact C model of the FPGA: the sample format (`fe_format.h`), the 27 → 6.75 MS/s decimator (provisional), and later the correlator bank with every width in `corr_params.h`. The HDL will be verified against it | Host only |
| `host/` | IQ files, the front-end emulation, the CLI tools | Host only |
| `py/` | Analysis: readers, a reference acquisition and C/N0 estimator, comparison scripts | Host only |
| `tests/` | Unit tests (GoogleTest, fetched as `tests_cpp/` does) | Host |
| `data/iq_files.ini` | Metadata for the rig's IQ files. **Never IQ itself** | — |
| `runs/` | Outputs; ignored by git | — |

## Build and test

```bash
cmake -S gnss-receiver -B gnss-receiver/build -DCMAKE_BUILD_TYPE=Release
cmake --build gnss-receiver/build -j8
ctest --test-dir gnss-receiver/build --output-on-failure
```

## The test files

The IQ files are in the COCOM rig's `tools/gnss-cocom/sdr/c8/`, which lives in that rig's worktree. Point
`GNSS_IQ_DIR` at it; tools then take bare file names. `data/iq_files.ini` holds what each file needs:
- its rate and centre frequency (most large files have no `.TXT` sidecar);
- its DC and carrier fixes;
- its measured signal power and noise density;
- its truth.

Things about the files that are not obvious, all measured 2026-09-30:
- **SignalSim's "45 dB-Hz" files measure 41.0 dB-Hz** (41.3 for the 18.48 MS/s wide file).
- **gps-sdr-sim files carry no noise.**
  - Read directly they behave like 50 dB-Hz. The floor comes from the other satellites' codes and the
    8-bit rounding.
  - `--cn0` adds thermal noise. With `--cn0 45` the correlator measures 43.7 dB-Hz unquantized and
    42.8 in the 2-bit stream.
  - They also carry −0.47 LSB of DC from truncation (`dc = auto`).
- **The `_cofs` files have their carrier 22.0 Hz below their code.** That cancels the HackRF on the rig.
  Read directly, they need `carrier_fix_hz = 22.0`.
- **Sample convention:** I + jQ, with Doppler signs as the generators log them.

## Front-end emulation

`iqtool emul` turns a file into what the correlators will see. In the default, `direct` mode:
1. DC and carrier fixes.
2. L1 to 0 Hz.
3. Resample to 6.75 MS/s (Kaiser polyphase, 80 dB).
4. Noise to a set C/N0.
5. The MAX2769B's 4.2 MHz IF filter (5th-order Butterworth).
6. L1 to the IF.
7. 2-bit sign/magnitude with an AGC holding 33 % magnitude bits.

The other modes:
- **`adc27`** does the quantization at 27 MS/s and then applies an FPGA decimator model. It is
  provisional until the hardware design picks the decimator.
- **`native`** keeps the file's rate in float.

```bash
export GNSS_IQ_DIR=.../tools/gnss-cocom/sdr/c8
build/host/iqtool info signalsim_static_gpsgal_2026_45_n.C8
build/host/iqtool emul hotshot_pad600_smooth_cofs_g8.C8 -o runs/hs.u2 --start 100 --dur 10 --cn0 45
python3 py/emul_check.py signalsim_static_gpsgal_2026_45_n.C8 --start 5 --dur 0.5
```

Every output gets a `.ini` beside it giving its rate, IF, source segment and sample mapping. Formats:
- **`u2`:** packed nibbles, as the FPGA receives them;
- **`cs8`:** the ±1/±3 weights as int8, readable by Pocket SDR;
- **`cf32`:** float.

The IF of the 6.75 MS/s stream defaults to +1.2 MHz. **That is a placeholder until the hardware design
sets the IF.**

Measured on the SignalSim static file, the 2-bit 6.75 MS/s stream costs 0.43 dB of C/N0 against the
native file. Doppler is unchanged to 0.01 Hz on every satellite.

## The receiver (GPS L1 C/A)

```bash
build/host/gnssrx signalsim_static_gpsgal_2026_45_n.C8 --out runs/static
build/host/gnssrx hotshot_pad600_smooth_cofs_g8.C8 --dur 120 --cn0 45 --truth static:0,-119,1200
python3 py/compare_pocketsdr.py runs/static runs/pocketsdr/<run>/trk.log
```

`gnssrx` feeds the emulated stream to the float correlator bank (`host/corr_float.c`) and gives the core
(`core/`) a 1 ms tick with the dumps, exactly as the FPGA's interrupt will on the P4. It writes:
- `trk.csv`: per channel at 10 Hz;
- `obs.csv`: pseudorange, carrier phase, Doppler, C/N0, elevation and residual per satellite;
- `pvt.csv`: position and error against the manifest's static truth;
- `eph.csv`: every decoded ephemeris.

How each stage works:
- **Acquisition:** FFT on a 10 ms snapshot block-averaged to 2.048 MS/s, with Doppler steps of
  250 Hz. A fine stage on the full-rate snapshot then hands over within ~0.05 chip and ~20 Hz.
- **Tracking:** Costas 3rd-order PLL, assisted by a 2nd-order FLL during pull-in (15 Hz PLL,
  10 Hz FLL), then 10 Hz once locked. The DLL is 1st order and carrier aided, with early/late at
  ±0.25 chip: 2 Hz in pull-in, 0.25 Hz once locked. These are static settings; the boost loops are
  milestone 5.
- **Navigation data:** bit sync by a transition histogram, then LNAV with parity, ephemeris and
  page 18.
- **Observables and PVT:** observables come from the exact integer NCO state. PVT is least
  squares with Sagnac, Klobuchar and Saastamoinen corrections, plus Doppler velocity.

Results, 2026-09-30, on the 2-bit 6.75 MS/s stream:

| File | Result |
|---|---|
| SignalSim static, 240 s | 13/13 GPS locked by 10 s, frame sync by 20 s, first fix at ~36 s. 2039 fixes: mean error E −0.04, N +0.01, U +0.06 m; sd 0.43, 0.28, 0.80 m |
| Same stream, Pocket SDR | 191 fixes: E +0.04, N −0.03, U −0.01 m; sd 0.25, 0.29, 0.87 m |
| Ours against Pocket SDR | Pseudoranges agree to 0.62 m rms (after fitting the two receivers' epoch-label offset); Doppler to 0.2 Hz; C/N0 to 0.3 dB |
| gps-sdr-sim hotshot pad, `--cn0 45` | 14/14 locked; E +0.03, N −0.04, U −0.41 m; sd 0.20, 0.15, 0.45 m |

Other checks:
- **Ephemerides:** every decoded ephemeris equals the RINEX broadcast record to print precision.
- **Doppler:** within ±2 Hz of geometric truth on every satellite.
- **Code and carrier:** they drift apart at under 1 cm/s.

Known limits:
- **Ionosphere parameters arrive slowly.** They come in subframe 4 page 18, every 12.5 minutes.
  Until they arrive, the manifest's `iono_params` stand in (as the flight computer could preload
  them). Uncorrected, SignalSim's ionosphere costs +3.6 m of height.
- **Low code Doppler adds code noise.** A satellite with little code Doppler (PRN 22, 0.14 chips/s)
  keeps the code samples lined up with the chips for long stretches: 1.7 m of code noise against
  0.6 m for the others.
- **The 4.5 detection threshold is set for 10 ms snapshots at ~40 dB-Hz.** Weaker signals need
  longer snapshots.
- **Speed:** one core, about 1.9× real time with 13 channels.

## The FPGA correlator model (milestone 4)

`fpga/model/corr_model.c` is the correlator bank bit for bit: 2-bit sign/magnitude codes in, integer
carrier table and accumulators, the contract's integer NCOs and epoch latching. Every width lives in
`core/include/gnss/corr_params.h`. `gnssrx` uses it by default on 2-bit streams (`--corr float` for the
float bank), so the receiver core already runs against the FPGA's arithmetic.

| Proposed | Value | Why |
|---|---|---|
| Sample input | 2-bit sign/magnitude, weights 1 and 3, 6.75 MS/s | The MAX2769B's output |
| Carrier NCO | 32-bit phase; signed 32-bit word | 1.6 mHz steps |
| Code NCO | chip index plus a 40-bit fraction | 2 mm/s steps: no DLL bias from carrier aiding |
| Carrier mixer | 3-bit phase, 8 sectors, levels 1 and 2 (the GP2021's) | −0.10 dB; 16 sectors buys only 0.03 dB more |
| Taps | early/prompt/late at ±0.25 chip | As tracked |
| Accumulators | 24-bit signed | 1 ms needs 18 bits, 10 ms B1C 21; the largest seen is 4,058 |
| Dump and tick | at each code epoch; a 1 ms tick reads them | As tracked |
| NCO commands | tagged with the period they take effect after; 2 pending per channel | A fixed loop delay; see below |
| Decimator 27 → 6.75 | every 4th sample | Costs nothing measurable; sum-of-4 to 2-bit costs 0.4 dB |

**Test vectors for the HDL.**

```bash
gnssrx FILE --dur 0.3 --vectors DIR --vectors-ms 300
vecreplay DIR
```

The vectors are three files:
- `samples.u2`: the correlator input;
- `commands.csv`: every command, with the sample at which it is presented;
- `dumps.csv`: every dump the correlator must produce.

`vecreplay` feeds a fresh model only those files and checks every dump bit for bit, the same job an
HDL testbench does. 300 ms of the static file gives 3,757 dumps with no mismatches.

**Tagged commands (owner decision, 2026-09-30).**
- Each NCO command names the period whose closing code epoch switches the channel to its words.
- The P4 tags a command computed from dump s with s + 2. That leaves at least a full period for its
  own latency. Each channel holds two pending commands (`CORR_CMD_QUEUE`).
- The delay from a measurement to its correction is then fixed: period s steers period s + 3,
  whatever the 1 ms tick's phase against the channel's epochs.
- A command that arrives after its tagged epoch applies at the next epoch and sets `CORR_DUMP_LATE`.
- `gnssrx --p4-latency-us T` delivers commands T µs after each tick. With 0 and 600 µs the dumps,
  observables and PVT are byte-identical, and a unit test holds that.

## Pocket SDR as the cross-check

- **Build:** a source-only checkout at `~/Projects/ModelRockets/bench-backups/pocketsdr/` (commit
  03787da5), built with its default PocketFFT. No FFTW.
- **Two traps:**
  - Its `INT8X2` format negates Q for its own front end. These files are plain I + jQ, so use
    `-fmt CS8`. With `INT8X2`, Doppler flips sign, and carrier aiding then drives the code loop the
    wrong way.
  - It cannot hold lock when the rate is a whole number of samples per chip:
    - At 8.184 MS/s (8 per chip) it loses every satellite within 2 s, with 8-bit or 2-bit samples,
      at 0 Hz or 1.2 MHz IF.
    - At 4.092 MS/s (4 per chip, the PSAS flight file's rate) it loses them repeatedly.
    - At 6.75 and 8.0 MS/s it tracks 13 of 13. On our 6.75 MS/s output its PVT lands within 0.1 m of
      truth (sd 0.2–0.6 m).

    **So cross-check it on the emulated stream:**

```bash
iqtool emul FILE -o runs/x.cs8 --format cs8 --dur 90
pocket_trk -sig L1CA -prn 1-32 -fmt CS8 -f 6.75 -fo 1574.22 -log trk.log -nmea pvt.nmea runs/x.cs8
```

`-fo` is the LO: L1 minus the IF.

## Number formats

Integers wherever the FPGA is. On the P4, float32 loops and double PVT are proposed, pending the owner's
decision. C builds with `-ffp-contract=off` so that host and P4 float results can match bit for bit.

## Licences

- **This directory** is GPL-3.0 like the rest of the repository's software.
- **Pocket SDR:**
  - It is BSD-2, but its README adds an export notice outside the licence: no military, WMD or
    delivery-system development use.
  - The owner's decision (2026-09-30) is to use it **only as a black-box cross-check**. No code is
    copied from it.
- **SignalSim** has no licence. Use its output files only, never its code.
- **xoshiro256\*\*** (`host/rng.c`) is written here from its public-domain description.
