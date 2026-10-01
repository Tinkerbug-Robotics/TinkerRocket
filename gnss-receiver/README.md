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

**The IF follows the hardware session's frequency plan (provisional until the owner confirms it):**
- The MAX2769B's LO sits at 1571.328052 MHz (fractional-N from 27 MHz). That puts L1 at +4.091948 MHz
  in the 27 MS/s ADC stream.
- Keeping every 4th sample folds it to −2.658052 MHz at 6.75 MS/s. That is the default for streams
  made at the correlators' rate. `--mode adc27` places L1 at +4.092 MHz and folds it the same way.
- The IF's sign rests on the chip's I/Q convention, and first light settles it. Every tool takes
  `--if`.
- At the correlators, the plan's IF costs nothing measurable against the old +1.2 MHz placeholder.
  The static file gives 40.76 against 40.80 dB-Hz and the same 239 fixes in 60 s.

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
  ±0.25 chip: 2 Hz in pull-in, 0.25 Hz once locked. Under a boost the receiver switches to wider
  loops; see [Boost dynamics](#boost-dynamics-milestone-5).
- **Navigation data:** bit sync by a transition histogram, then LNAV with parity, ephemeris and
  page 18.
- **Observables and PVT:** observables come from the exact integer NCO state. Doppler is the NCO's
  mean frequency over the last 20 ms, moved forward by the loop's rate. PVT is least squares with
  Sagnac, Klobuchar and Saastamoinen corrections, plus Doppler velocity.

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
| Carrier mixer | 3-bit phase, 8 sectors, levels 1 and 2 (the GP2021's) | −0.10 dB on 2-bit samples (−0.25 dB on white noise); 16 sectors buys only 0.03 dB more |
| Taps | early/prompt/late at ±0.25 chip | As tracked |
| Accumulators | 24-bit signed | 1 ms needs 18 bits, 10 ms B1C 21; the largest seen is 4,058 |
| Dump and tick | at each code epoch; a 1 ms tick reads them | As tracked |
| NCO commands | tagged with the period they take effect after; 2 pending per channel | A fixed loop delay; see below |
| Counters | sample count 48-bit, period count (`seq`, `apply_seq`) 16-bit, both compared modulo | The hardware session's widths; the P4 extends them |
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
HDL testbench does. 300 ms of the static file gives 3,744 dumps with no mismatches, with or without the P4's latency.

**Tagged commands (owner decision, 2026-09-30).**
- Each NCO command names the period whose closing code epoch switches the channel to its words.
- The P4 tags a command computed from dump s with s + 2. That leaves at least a full period for its
  own latency. Each channel holds two pending commands (`CORR_CMD_QUEUE`).
- The delay from a measurement to its correction is then fixed: period s steers period s + 3,
  whatever the 1 ms tick's phase against the channel's epochs.
- A command that arrives after its tagged epoch applies at the next epoch and sets `CORR_DUMP_LATE`.
- `gnssrx --p4-latency-us T` delivers commands T µs after each tick. With 0 and 600 µs the dumps,
  observables and PVT are byte-identical, and a unit test holds that.

**Counters (hardware session, 2026-09-30).**
- The FPGA's sample counter is 48 bits, and each channel's period count is 16 bits. Both wrap: the
  sample counter after 1.3 years, the period count every 65.5 s.
- The model wraps them as the HDL will. The P4 code (`rx_tick`) extends them to 64 and 32 bits.
- Two tests guard this:
  - A unit test runs a stream across the sample-counter wrap and gets identical dumps.
  - A 140 s `gnssrx` run, in which every channel's period count wraps twice, is byte-identical to
    the same run before the change.

**The carrier mixer around the IF** (`IfPlan.MixerAroundTheIf`). The 8-sector table was swept ±100 kHz
around −2.658 MHz in 100 Hz steps.
- Against white noise it loses 0.246 dB everywhere, flat to 0.001 dB.
- A DC offset in the samples can reach the correlators unspread only where a table harmonic folds it
  onto 0 Hz. That is the simple ratios of 6.75 MS/s.
- The nearest is 2 fs / 5, 42 kHz from the IF. Its leakage is −21 dB, a feature a few hundred hertz
  wide.
- Inside the ±15 kHz that satellites occupy (Doppler, vehicle velocity and the reference's
  ±0.5 ppm), the leakage stays below −44 dB.

## Boost dynamics (milestone 5)

The question: where does our tracking hold through a rocket boost, and with which loops? The two
test flights are the rig's files, the ones the bought receivers flew in the rig's reports. Their
truth is in the rig's `scenarios/`:
- **traveler** (`traveler_soft25`): a 13 s burn, 6 → 18 g, 0.25 s soft start. The steepest
  satellite slides at 835 Hz/s.
- **hotshot:** a 4 s burn, 10 → 40 g. The steepest satellite slides at 1,638 Hz/s, then drops to
  −190 Hz/s within 0.2 s at burnout.

### Tools

| Tool | What it does |
|---|---|
| `py/los_truth.py` | Each satellite's line-of-sight truth along a scenario: Doppler, Doppler rate, elevation |
| `trksim` | One channel's real tracking code (`core/trk`) against that truth, at the level of correlator dumps. It includes the contract's command delay, data bits, and correlated early/prompt/late noise at any C/N0, and runs about 1000× real time. On the IQ files it matches `gnssrx` channel for channel: unlock time within 0.1 s, the same slips |
| `py/trk_sweep.py` | Sweeps `trksim` over loop profiles, C/N0, satellites and seeds; tabulates the worst satellite |
| `gnssrx --boost-at S0,S1` | The boost profile over those file seconds, as the flight computer would call it. With `--loops-quiet` and `--loops-boost` (bandwidths), `--cn0-at S:DBHZ` (a level change mid-file) and `run.ini` (what made the run) |
| `py/boost_track.py` | A run against truth, per satellite: frequency error, unlock time, carrier slips, code error, losses as the bench counts them (no pseudorange for 0.5 s), and the fix |

`boost_track.py` judges the carrier against the integral of the true Doppler. That integral is how
the rig's smoothed gps-sdr-sim builds the carrier. Range from linearly interpolated positions differs
from it by a·dt²/8 inside a 0.1 s trajectory step, which at burnout is whole cycles.

### What the stage-0 loops did

On the hotshot at 42.6 dB-Hz, every channel's PLL let go at liftoff and at burnout:
- unlocked 0.6–3.5 s per satellite;
- 1–3 cycles slipped.

The FLL held code and frequency, so no satellite was lost and the fix never gapped more than
0.1 s.

Below about 37 dB-Hz the stage-0 machinery failed on the pad, before any boost. Five things were
wrong, all fixed in `core/trk` for every profile:
- **The lock indicator read SNR / (SNR + 1).** It was a per-dump cos 2φ, which never reaches its
  0.85 threshold below 38 dB-Hz. It now takes the noise power out (1 in lock at any C/N0).
- **Pull-in channels gave no measurements.** A channel whose PLL lets go now still reports
  pseudorange and Doppler, with `lock_s` = 0 to void its carrier phase.
- **Bit sync was dropped with PLL lock.** Bit edges follow the code, so it now stays.
- **The 1 ms FLL was too noisy below 37 dB-Hz.** At 35 dB-Hz it was about ±20 Hz. The FLL now
  compares 2 ms blocks:
  - folded before bit sync;
  - inside a data bit with a full atan2 after it (±250 Hz).
- **The C/N0 estimate could not see a lost signal.** The 200-dump moments estimate reads about
  27 dB-Hz on noise alone. So a loop that lost its signal at burnout ramped its NCO away (11 kHz off
  in one run) and was never dropped. Three changes:
  - After bit sync the estimate is narrowband over wideband power in 5 ms blocks. It reads near
    zero on noise. A 50 Hz frequency error pulls it down (45 reads 35) but nowhere near the
    25 dB-Hz loss threshold, where 20 ms blocks would read nothing.
  - The loops' rate state is clamped at 3 kHz/s.
  - A channel whose signal has gone coasts on its last frequency until it is dropped.

### Findings

- **The fixed command delay costs nothing.** Period s steers period s + 3. Every design tried gave
  the same result with a delay of 1 as with 3, up to a 50 Hz PLL.
- **Unaided, the burnout needs a ≥ 40 Hz PLL** (review §7's figure). On the hotshot's steepest
  satellite:
  - 20–30 Hz loops slip 10–16 half cycles in the 0.2 s of burnout;
  - 40 Hz slips none at 45 dB-Hz;
  - 50 Hz slips none at 45 or 40 dB-Hz, and one at 35.
- **Narrowing must be gradual.** Stepping a wide loop straight into a narrow one hands it the wide
  loop's frequency noise. The 10 Hz PLL slipped on it at 606 s, 2 s after burnout. The receiver
  now widens at once and narrows with a 0.5 s time constant (`trk_profile_step`).
- **Doppler from the phase, not the NCO word.** A 50 Hz loop jitters its NCO word by tens of hertz,
  while the phase it holds stays steady. The Doppler observable is now the mean NCO frequency
  over 20 ms from the exact phase, moved forward by the loop's own rate:
  - the boost's vertical velocity error falls four- to sixfold, to 0.19 m/s rms at 42.6 dB-Hz;
  - the static velocity error falls 4.5-fold, to 0.02–0.05 m/s rms per axis.

### The proposed design (under discussion with the owner)

| Profile | Pull-in FLL / PLL / DLL | Locked FLL / PLL / DLL | FLL block | When |
|---|---|---|---|---|
| quiet | 10 / 15 / 2 Hz | — / 10 / 0.25 Hz | 2 ms | pad, coast, descent |
| boost | 10 / 50 / 2 Hz | 5 / 50 / 1 Hz | 2 ms | from the flight computer's launch arming (or launch detect) to 2 s after burnout |

The P4 calls `rx_set_boost()` from the flight computer's phase. Narrowing back tapers over about
1 s.

### Results through the boost

On the IQ chain: `gnssrx`, the golden correlator, the 2-bit 6.75 MS/s stream. The signal is
stepped down from 45 to the level shown 10 s before liftoff. C/N0 is what our receiver measures.
"Lost" is the bench's rule: no pseudorange for 0.5 s or more, from ignition to 0.5 s past burnout.

| C/N0 (measured) | Hotshot, boost profile | Hotshot, quiet loops only | Traveler, boost profile | Traveler, quiet loops only |
|---|---|---|---|---|
| 42.6 | 0 lost; 0 of 14 lost carrier lock | 0 lost; 14 of 14 lost carrier lock | 0 lost; 0 of 14 | 0 lost; 14 of 14 |
| 36.4 | 0 lost; 0 of 14 | 0 lost; 14 of 14 | 0 lost; 0 of 14 | 0 lost; 14 of 14 |
| 33.4 | 0 lost; 3 of 14 | 0 lost; 14 of 14 | 0 lost; 1 of 14 | 0 lost; 14 of 14 |
| 31.4 | 0 lost; 13 of 14 | 0 lost; 14 of 14 | 0 lost; 14 of 14 | 0 lost; 14 of 14 |
| 29.3 | 0 lost; 14 of 14 | 0 lost; 14 of 14 | 0 lost; 14 of 14 | 0 lost; 14 of 14 |

The fix never gapped more than 0.1 s.

Fix errors through the hotshot burn with the boost profile:

| C/N0 (measured) | Height error, max | Vertical velocity error, rms (max) |
|---|---|---|
| 42.6 | 3.4 m | 0.19 m/s (1.0) |
| 36.4 | 8.3 m | 0.33 m/s (1.1) |
| 33.4 | 9.7 m | 0.59 m/s (1.8) |
| 31.4 | 14.5 m | — |
| 29.3 | 21.5 m | — |

`trksim` agrees, and says the same for every satellite in the sky. With the boost profile, all 14
keep carrier phase through the traveler at 45, 40 and 35 dB-Hz, and through the hotshot at 45 and
40. At 35 only the overhead PRN 11 (1,638 Hz/s) slips, once.

**Against the bought receivers** (the rig's C/N0 boost report, the same files):

| Receiver | Hotshot | Traveler |
|---|---|---|
| SkyTraq PX1105R (SLR mode) | 10–13 of 13 lost by T+2.8 s at every level flown | Half lost by 135–145 Hz/s at 35–38 expected dB-Hz, by 330 Hz/s at 48 |
| u-blox NEO-M8T (airborne) | None lost before its 515 m/s cutoff (T+2.8 s) down to 18 expected dB-Hz, where it reads 29. Raw withheld after | The same |
| Ours, boost profile | None lost through the whole burn and burnout down to 29.3 dB-Hz. Carrier phase on all 14 at ≥ 36 dB-Hz | The same |

The two C/N0 scales are not the same instrument:
- the bench quotes an "expected" level calibrated on the PX1105R;
- ours is our own estimate on the emulated stream.

The comparison that holds is in kind. Ours, like the NEO-M8T, follows every satellite on the
bench, and it does so through the whole burn and burnout, carrier phase included.

### Next: IMU feed-forward

`trk_ch_t.ff_rate` takes a predicted line-of-sight Doppler rate. It drives the frequency directly,
so the loop tracks only what the prediction misses. `trksim --aid` tests it with an IMU's typical
faults: 5 ms late, 3 % scale error, and a 1 g bias along the line of sight. With those, the quiet
10 Hz loops ride both boosts:
- no slips at 45 dB-Hz, 0.3 at 40, 2.7 at 35;
- a quarter of the boost profile's frequency noise.

Feed-forward makes the wide loops unnecessary, and with them most of the boost's C/N0 cost. It
needs two things on the P4:
- the flight computer's acceleration and attitude, to project onto each line of sight;
- the oscillator's g-sensitivity (milestone 7), to correct the clock.

### Limits

- **Static sensitivity ends near 31 dB-Hz.** Pull-in fails below about 32 dB-Hz, boost or not.
  Weaker signals need longer coherent integration with data wipe-off and a two-stage pull-in.
- **Real motors add what the files lack.** Vibration on the oscillator, plume, spin and antenna
  phase are milestone 7.

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
