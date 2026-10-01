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
| `core/` | The P4 firmware-to-be. Portable C99: no OS calls, no malloc after init | Host now; an ESP-IDF component later |
| `fpga/model/` | Bit-exact C model of the FPGA: the sample format (`fe_format.h`), the 27 → 6.75 MS/s decimator (keep every 4th sample), and later the correlator bank with every width in `corr_params.h`. The HDL will be verified against it | Host only |
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
- **`adc27`** does the quantization at 27 MS/s and then applies an FPGA decimator model. The design
  keeps every 4th sample (owner decision, 2026-10-01); sum-of-4 stays for comparison.
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

**The IF follows the hardware session's frequency plan (owner decision, 2026-10-01):**
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
- **Observables and PVT:** observables come from the exact integer NCO state.
  - Doppler is the NCO's mean frequency over the last 20 ms, moved forward by the loop's rate.
  - Pseudoranges are carrier-smoothed (Hatch, 100 s, `rx_cfg_t.hatch_s`) while the PLL holds,
    and restart when it lets go.
  - PVT is weighted least squares with Sagnac, Klobuchar and Saastamoinen corrections, plus
    Doppler velocity. It tests its own residuals; see [The fix's weights and residual test](#the-fixs-weights-and-residual-test).

Results, 2026-09-30, on the 2-bit 6.75 MS/s stream, before carrier smoothing:

| File | Result |
|---|---|
| SignalSim static, 240 s | 13/13 GPS locked by 10 s, frame sync by 20 s, first fix at ~36 s. 2039 fixes: mean error E −0.04, N +0.01, U +0.06 m; sd 0.43, 0.28, 0.80 m |
| Same stream, Pocket SDR | 191 fixes: E +0.04, N −0.03, U −0.01 m; sd 0.25, 0.29, 0.87 m |
| Ours against Pocket SDR | Pseudoranges agree to 0.62 m rms (after fitting the two receivers' epoch-label offset); Doppler to 0.2 Hz; C/N0 to 0.3 dB |
| gps-sdr-sim hotshot pad, `--cn0 45` | 14/14 locked; E +0.03, N −0.04, U −0.41 m; sd 0.20, 0.15, 0.45 m |

With carrier smoothing (2026-10-01), the SignalSim static file's GPS gives 2039 fixes with mean
error E +0.02, N +0.05, U −0.08 m and sd 0.05, 0.06, 0.14 m.

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

### The fix's weights and residual test

Each measurement carries a 1-sigma, and the fix weighs it by 1/σ²:
- **Pseudorange** (`pvt_sigma_pr`): a 0.2 m floor, plus the code's thermal noise from C/N0. That
  is 0.6 m at 40 dB-Hz on one epoch, and it climbs faster below 32 dB-Hz as the squaring loss sets
  in. Carrier smoothing averages it down over the smoothing's age; the code noise decorrelates in
  about 2 s.
- **Range rate** (`pvt_sigma_dop`): 0.06 m/s at 43 dB-Hz behind a 10 Hz PLL, scaling with the
  carrier's phase noise. It grows as (bandwidth / 10 Hz)^0.75 for wider loops, because their rate
  state feeds the observable's lag correction. Measured on the pad, 20 Hz loops are 1.2–1.3 times
  noisier and 50 Hz loops 3.1–3.6 times. It is five times larger for a channel whose PLL is
  pulling in, since its Doppler is the FLL's.
- **Each satellite's own spread** (`rx_cfg_t.adapt_tau_s`, 30 s). The receiver low-passes each
  satellite's squared normalized residual. Where that runs above 1, the satellite's sigma grows
  with it, up to five times. A satellite worse than its C/N0 suggests then counts for less, instead
  of being left out one epoch and back in the next. The case in point is E19 on the SignalSim
  static files: its code wanders with the sample phase.

The model comes from the residuals of the boost runs on the pad, set a little on the safe side:

| Pseudorange | Measured rms |
|---|---|
| Raw, 33 dB-Hz | 1.1 m |
| Raw, 31 dB-Hz | 2.4 m |
| Raw, 29 dB-Hz | 4.9 m |
| Smoothed 10–40 s | 0.3–0.7 m |
| Smoothed over 40 s | 0.12–0.28 m |

The test is a chi-square on the weighted residuals, at a false-alarm probability of 10⁻⁴ per
epoch (`pvt_opt_t.raim_pfa`):
- When it fails, the measurement with the largest normalized residual is left out and the fix
  solved again, up to twice.
- A fix that still fails is withheld: `pvt_solve` returns −2, and the receiver keeps its last good
  fix for aiding.
- The velocity is tested the same way. A failure there leaves the position standing, with
  `vel_valid` = 0.

`pvt.csv` records `nexcl`, `chi2`, `chi2_lim` and `vel_valid` per fix. `obs.csv` records `excl`
(bit 0 the range, bit 1 the Doppler).

On the static files, the same code with each layer added in turn (sd E / N / U; mean in brackets):

| Fix | GPS + Galileo, 240 s | GPS + Galileo + BeiDou, 90 s |
|---|---|---|
| Unweighted, no test (before) | 0.22 / 0.05 / 0.19 m (U −0.23) | 0.18 / 0.10 / 0.19 m (U +0.20) |
| Weighted by C/N0 and smoothing age | 0.17 / 0.05 / 0.15 m | 0.13 / 0.10 / 0.16 m |
| + the residual test | 0.11 / 0.04 / 0.15 m; E19 left out in 719 of 2039 fixes | 0.10 / 0.08 / 0.17 m; 70 of 539 |
| + each satellite's own spread (the default) | **0.06 / 0.07 / 0.11 m (U −0.08)**; nothing left out | **0.09 / 0.07 / 0.10 m (U +0.03)**; 1 fix |

`--no-raim`, `--pvt-unweighted` and `--pvt-adapt-tau 0` turn the layers off for comparisons.

Through the boosts (`runs/m7c`, `runs/m7g`; 54 runs, compared with the same runs unweighted):
- **No false alarms on the pad** in any configuration at 31.4 dB-Hz and above. At 29 dB-Hz, only
  the configurations past their limit flag anything: the 50 Hz loops (4 fixes with a measurement
  left out) and the quiet loops (1 velocity).
- **The runaway is caught.** With quiet loops and no aiding, every channel's NCO ran away
  together after the hotshot's burnout (`Q_hot_38`). Unweighted, the fix climbed 97 m in 0.6 s
  while its residuals reached only 33 m rms. Now the worst height error is 12 m, the vertical
  velocity 1.2 m/s rms (from 58), and the epochs that can't be mended are withheld.
- **Weak satellites stop moving the fix.** On SignalSim, one weak Galileo satellite's 7.5 m code
  error moved the aided fix 2.9 m. Now the worst height error through the burn is 0.3 m.
- **Velocity at the edge.** IMU + 20 Hz loops at 29.3 dB-Hz: 0.80 → 0.42 m/s rms (hotshot) and
  0.79 → 0.39 (traveler). The 50 Hz fallback at 31.4 dB-Hz: 2.2 → 1.5 and 1.8 → 1.2 m/s.
- At 33.4 dB-Hz and above, the aided runs were already clean, and they stay the same.

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
| `gnssrx --imu SCEN.csv` | IMU aiding from the scenario's trajectory, with the IMU's faults (`--imu-err`); see [IMU aiding](#imu-aiding-milestone-7) |
| `py/boost_plots.py` | The figures, drawn like the rig's reports on the bought receivers. `rates` draws each satellite's line-of-sight Doppler rate through the burn, by what the receiver delivered, as a grid of runs (signal levels × loop configurations). `timeline` draws one run in full: speed, altitude and acceleration (with the IMU's input), per-satellite output, measurements per epoch, pseudorange and range-rate errors, and the fix's errors |

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

### The design (owner decision, 2026-10-01)

The P4 detects launch and burnout from the dev board's own IMU, which sits on its SPI with
data-ready stamped by the FPGA on the sample counter. It feeds the loops the predicted
line-of-sight Doppler rate (see [IMU aiding](#imu-aiding-milestone-7) below), so they can stay narrow. The
boost profile is the fallback when there is no aiding. Profiles:

| Profile | Pull-in FLL / PLL / DLL | Locked FLL / PLL / DLL | FLL block | When |
|---|---|---|---|---|
| quiet | 10 / 15 / 2 Hz | — / 10 / 0.25 Hz | 2 ms | pad, coast, descent |
| boost | 10 / 50 / 2 Hz | 5 / 50 / 1 Hz | 2 ms | without aiding: from launch to 2 s after burnout |

Without aiding, the P4 calls `rx_set_boost()` from the launch and burnout it detects. Narrowing
back tapers over about 1 s.

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

With carrier smoothing (milestone 6), the PLL holds through the burn and smoothing never restarts.
At 42.6 dB-Hz the hotshot's position error falls to rms 0.17 / 0.11 / 0.28 m, with a worst height
error of 0.6 m (from 3.4 m).

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

### IMU aiding (milestone 7)

The owner's design (2026-10-01): the P4 hands each loop the line-of-sight Doppler rate it
predicts from the board's IMU. The loops then track only what the prediction misses, and can
stay narrow through the burn.

**In the receiver:**
- `rx_set_accel(rx, acc, valid)` takes the acceleration in ECEF (m/s², gravity taken out) at each
  tick.
- Each tracked satellite's line of sight comes from the last fix. That includes satellites the
  fix leaves out: the rig's PRN 13 is flagged unhealthy, so it never enters the fix, and it went
  unaided until this was fixed.
- The loop gets `ff_rate` = a·u / λ, in Hz/s. It drives the loop's frequency directly. Each
  command carries the frequency predicted for when it lands, three periods on. The Doppler
  observable's lag correction counts it too.

**The emulated IMU.** `gnssrx --imu SCEN.csv` takes the acceleration from the scenario's
trajectory, constant over each 0.1 s step, as the smoothed carrier shows it. `--imu-err
LAG,SF,BIAS,NOISE[,TILT]` adds the faults:
- latency, ms;
- scale error, as a fraction;
- bias along local up, m/s²;
- white noise per axis, m/s²;
- an attitude error, in degrees, that tips the acceleration from up toward east.

The default is 5 ms, 3 %, 0.5 m/s², 0.1 m/s² and 0°. `--imu-at S0,S1` limits the aiding to a
window. `imu.csv` logs what the loops were given.

**Why 20 Hz loops and not the quiet 10 Hz ones.** At burnout, an IMU 5 ms late and 3 % off leaves
the loop 50–100 Hz/s that it was not told about. An acceleration step's phase error is about its
size over ωn². So a 10 Hz PLL stays within ±45° only up to about 20 Hz/s, while a 20 Hz PLL
manages about 80 Hz/s. `TrkBoost.ImuAidingCarriesTheBurnout` shows all three cases:
- exact aiding carries the quiet loops;
- the realistic IMU makes them slip;
- 20 Hz loops ride the realistic IMU.

| Profile | Pull-in FLL / PLL / DLL | Locked FLL / PLL / DLL | FLL block | When |
|---|---|---|---|---|
| aided boost | 10 / 20 / 2 Hz | — / 20 / 0.5 Hz | 2 ms | with the IMU: from T−5 s to 2 s after burnout |

**Results through the boost** (`runs/m7c`; figures from `py/boost_plots.py`). The set-up is the
one above: the IQ chain, the signal stepped down 10 s before liftoff, C/N0 as our receiver measures
it. Each cell gives the satellites (of 14) whose PLL let go between ignition and 2.5 s past
burnout, with the unlocked satellite-seconds. No run lost a pseudorange, and the fix never gapped.

Hotshot:

| C/N0 (measured) | Quiet loops | 50 Hz boost loops | Quiet loops + IMU | IMU + 20 Hz loops |
|---|---|---|---|---|
| 42.6 | 14 (27 s) | 0 | 3 (1.0 s) | 0 |
| 36.4 | 14 (39 s) | 0 | 3 (1.2 s) | 0 |
| 33.4 | 14 (53 s) | 2 (0.7 s) | 5 (9.8 s) | 0 |
| 31.4 | 14 (68 s) | 9 (6.0 s) | 8 (11 s) | 0 |
| 29.3 | 14 (77 s) | 14 (45 s) | 9 (22 s) | 7 (9.4 s) |

Traveler:

| C/N0 (measured) | Quiet loops | 50 Hz boost loops | Quiet loops + IMU | IMU + 20 Hz loops |
|---|---|---|---|---|
| 42.6 | 14 (13 s) | 0 | 0 | 0 |
| 36.4 | 14 (20 s) | 0 | 0 | 0 |
| 33.4 | 14 (42 s) | 0 | 2 (3.2 s) | 0 |
| 31.4 | 14 (46 s) | 10 (9.8 s) | 2 (3.5 s) | 0 |
| 29.3 | 14 (180 s) | 14 (121 s) | 6 (12 s) | 7 (19 s) |

The fix through the same window, 50 Hz boost loops against IMU + 20 Hz loops:

| C/N0 (measured) | Hotshot: vertical velocity rms (max), m/s | Hotshot: height error max | Traveler: vertical velocity rms (max) | Traveler: height error max |
|---|---|---|---|---|
| 42.6 | 0.22 (1.0) → 0.07 (0.22) | 0.7 → 0.7 m | 0.21 (0.59) → 0.08 (0.22) | 0.5 → 0.5 m |
| 36.4 | 0.38 (0.91) → 0.13 (0.39) | 0.7 → 0.6 m | 0.43 (1.2) → 0.16 (0.46) | 0.5 → 0.6 m |
| 33.4 | 0.67 (2.0) → 0.23 (0.74) | 1.4 → 1.1 m | 0.69 (1.9) → 0.24 (0.79) | 0.8 → 1.1 m |
| 31.4 | 1.5 (5.3) → 0.32 (0.89) | 2.6 → 1.2 m | 1.2 (4.9) → 0.31 (0.85) | 6.0 → 1.2 m |
| 29.3 | 4.7 (17) → 0.42 (1.3) | 15.5 → 5.0 m | 4.6 (18) → 0.39 (1.2) | 24.1 → 5.4 m |

So aiding with 20 Hz loops keeps carrier phase on every satellite through both burns down to
31.4 dB-Hz, where the unaided boost profile holds only to 36.4 (hotshot) and 33.4 (traveler).
The vertical velocity is three to twelve times better. Both columns use the weighted, tested fix
([The fix's weights and residual test](#the-fixs-weights-and-residual-test)); velocities the fix's
own test rejected are not counted.

**How good the IMU must be** (`runs/m7i`; IMU + 20 Hz loops at 31.4 dB-Hz; satellites that let go):

| IMU | Hotshot | Traveler |
|---|---|---|
| Ideal | 0 | 0 |
| 5 ms, 3 %, 0.5 m/s², 0.1 m/s² (the default) | 0 | 0 |
| The default, plus a 2° attitude error | 0 | 0 |
| The default, plus a 5° attitude error | 3 (2.7 s), all back within 1 s | 0 |
| 20 ms, 10 %, 2 m/s², 0.5 m/s² | 9 (9.7 s), 2 not back | 4 (4.8 s), 1 not back |

What the P4 needs from the board:
- the IMU's samples stamped on the sample counter, with latency under about 5 ms;
- its scale factor good to about 3 %;
- the attitude good to about 2° through the burn. A wrong attitude turns part of the thrust
  sideways: 5° of the hotshot's 35 g is 3 g.

**Galileo through the boost** (`runs/m7g`): the SignalSim traveler with GPS and Galileo
(`signalsim_traveler_gpsgal_2026_50e_n_cofs.C8`). Its sky is 46.5 dB-Hz at the zenith, faded by
elevation. The table gives each Galileo satellite's PLL-unlocked time, from ignition to 2.5 s
past burnout:

| Satellite | Pad C/N0 (E1-C) | Quiet loops | 50 Hz boost loops | Quiet loops + IMU | IMU + 20 Hz loops |
|---|---|---|---|---|---|
| E26 | 41.2 | 3.1 s, not back | 1.5 s | 0.3 s | 0 |
| E13 | 39.6 | 3.0 s, not back | 1.1 s | 0 | 0 |
| E29 | 36.5 | 0.9 s | 1.0 s | 0.3 s | 0 |
| E27 | 35.2 | 0.9 s | 0.9 s | 0 | 0 |
| E21 | 34.0 | 0.9 s | 1.1 s | 0.3 s | 0 |
| E19 | 32.0 | 3.1 s | 2.5 s | 0.6 s | 1.2 s |
| E07 | 31.6 | 5.5 s, not back | 5.7 s | 3.8 s | 5.3 s |
| E33 | 30.6 | 9.9 s | 10.0 s | 7.4 s | lost |

- **The 50 Hz fallback cannot help Galileo.** A pilot's 4 ms dumps and the three-dump command
  delay cap its loops at 12.5 Hz, so every Galileo satellite lets go at burnout for about a
  second.
- **With aiding**, the five Galileo satellites at 34 dB-Hz and above hold carrier through the burn
  and burnout. The three weak ones flicker on the pad already.
- **GPS** holds carrier on all 8 satellites in every configuration but the quiet loops.
- **The fix**, ignition to 2.5 s past burnout, weighted and tested:
  - The vertical velocity error is 0.15 m/s rms with IMU + 20 Hz loops, against 0.21 with the
    boost profile and 3.1 with quiet loops.
  - The height error peaks at 0.3 m. Unweighted, it reached 2.9 m, where one weak Galileo
    satellite's code ran 7.5 m off.

**Fixed on the way:**
- **A noise step unlocked the PLL** (`TrkBoost.StaysLockedThroughANoiseStep`). A 14 dB step in
  the noise floor, as the AGC passes it on, puts the lock indicator's noise power wrong until the
  next C/N0 estimate, up to 0.2 s later. At 31 dB-Hz that dropped 12 of 14 channels into pull-in,
  where they took seconds to relock. LOCKED now falls back only after 0.1 s below the line.
- **The troposphere moves when the receiver climbs** (`Pvt.TroposphereAboveAClimbingReceiver`).
  Found on SignalSim, which has a troposphere; gps-sdr-sim has none. Two changes:
  - The velocity solution now carries the delay's change with height. At 1 km/s through 5 km the
    delay falls by 0.15 m/s at the zenith and 0.4 m/s at 20°. The boost's vertical velocity error
    on SignalSim falls from 0.38 to 0.18 m/s rms.
  - The model no longer stops at 10 km, where the zenith delay is still 0.6 m. It now runs to
    40 km. SignalSim's troposphere, like the real one, carries on.
- **Satellites outside the fix went unaided.** The rig's PRN 13 is flagged unhealthy, so the fix
  leaves it out, and it got no line of sight. Every tracked satellite now gets one.


### Limits

- **Static sensitivity ends near 31 dB-Hz.** Pull-in fails below about 32 dB-Hz, boost or not.
  Weaker signals need longer coherent integration with data wipe-off and a two-stage pull-in.
  Even aided, the burn costs carrier at 29 dB-Hz: with an ideal IMU and quiet loops, 5 of 14 on
  the hotshot.
- **The fix's sigmas know noise, not tracking stress.** Two cases follow:
  - With the 50 Hz loops at 29 dB-Hz (all 14 satellites already losing carrier), the weighted
    fix trusts the wrong channels: worst height error 15–24 m, against 16–17 m unweighted.
  - The quiet loops without aiding run 0.5–2 m/s worse in velocity than unweighted.

  Both are configurations nothing flies. A lock-quality term in the sigma would cover them.
- **Real motors add what the files lack.** Vibration on the oscillator, plume, spin and antenna
  phase are milestone 7.

## Galileo E1 and BeiDou B1C (milestone 6)

```bash
build/host/gnssrx signalsim_static_gpsgalb1c_2026_45_n.C8 --dur 90 --out runs/m6
```

**Codes** (`core/sig/`):
- **B1C:** the data, pilot and pilot-secondary codes are Weil codes generated here. All 189 match
  the ICD's printed first and last 24 chips.
- **E1:** the E1-B and E1-C memory codes come from the Galileo ICD's attachments. They are not
  under this repository's licence: see [Licences](#licences).

**Correlator** (corr_if.h, the golden model and the float bank):
- A START names the tracked signal: L1 C/A, E1-C or the B1C pilot.
- E1 and B1C replicas carry sine-phased BOC(1,1).
- Each channel has five taps on the tracked code (very early, early, prompt, late, very late)
  plus a prompt on the data code.
- Code periods are 1, 4 or 10 ms.
- Secondary codes are wiped off on the P4, one chip per period.
- Measured on synthetic signals, against the theory:
  - E1: early/prompt 0.698 (theory 0.700); very early −0.501 (−0.5); E1-B at E1-C's power, in
    phase.
  - B1C: data/pilot 0.613 (0.616), in quadrature as SignalSim builds it.
- The GPS path is unchanged: a 65 s boost run is byte-identical, and the HDL vectors replay with
  no mismatches.

**Aided starts** (`rx_aid`):
- Galileo and BeiDou ephemerides are preloaded from the generator's RINEX (the manifest's `nav`),
  as the flight computer could provide them. Their navigation messages are not decoded yet.
- After the first GPS fix, every Galileo and BeiDou satellite above 10° gets a channel started
  at the predicted code phase and Doppler. The predicted B1C Dopplers match SignalSim's within
  1 Hz.
- The transmit time comes from the prediction rounded to the code epoch, and the secondary-code
  phase from it. On the file, Galileo's CS25 alignment agrees on every dump.
- A start that never locks is dropped after 5 s and held off for 30 s. Until a pilot has locked
  (lock indicator and 30 dB-Hz), it gives no measurements.

**Tracking a pilot:**
- full-range PLL and FLL discriminators;
- the BOC(1,1) DLL gain;
- C/N0 by moments over 0.2 s;
- loop bandwidths capped at Bn·T ≤ 0.05 (12.5 Hz for E1, 5 Hz for B1C). With 10 ms dumps and
  the three-period command delay, the 15 Hz pull-in PLL could not lock B1C.

**PVT:** one time offset per extra system. A GPS-only solve does the same arithmetic as before.

Results on the static files (GPS 13, Galileo 8, BeiDou 8 satellites):

| | GPS only | GPS + Galileo + BeiDou, raw | GPS + Galileo + BeiDou, carrier-smoothed |
|---|---|---|---|
| Satellites, PDOP | 13, 1.34 | 29, 0.92 | 29, 0.92 |
| Position sd E / N / U | 0.43 / 0.28 / 0.81 m (raw); 0.05 / 0.06 / 0.14 m (smoothed) | 0.95 / 0.25 / 0.93 m | 0.15 / 0.03 / 0.17 m (mean within 0.1 m) |
| Pseudorange residual sd | 0.45–1.3 m (raw) | GPS 0.55–1.5; Galileo 0.55–1.7 (E19 7.2); B1C 0.35–1.1 m | E19 1.3 m |
| C/N0 | 40.3 dB-Hz | GPS 40.3; E1-C 37.0; B1C pilot 38.4 (SignalSim predicts 39.2) | |

Galileo and BeiDou time offsets solve to about +1.5 m against GPS, steady to a few decimetres.

**Fixed since the first runs:**
- **Low code Doppler.** E19 (+9 Hz) wandered ±11 m with a 25 s period. That is the time its code
  takes to slide one sample spacing (0.15 chip) at 0.006 chips/s, so the code tracking error
  repeats with the sample phase. Faster satellites average it out; this one followed it.
  Carrier smoothing over 100 s cuts it to 1.3 m sd, and the position sd from 0.95 / 0.25 / 0.93
  to 0.15 / 0.03 / 0.17 m.
- **Side peaks** (`TrkPilot.JumpsOffASidePeak`). On a BOC channel, a very early or very late tap
  with more power than the prompt over 0.2 s moves the code half a chip its way, over one
  commanded period. A channel parked on either side peak finds the main peak; one near the main
  peak never jumps.
- **A pilot's C/N0** comes from 0.2 s of moments, which read up to 28 dB-Hz on noise. So a pilot
  measures, and counts as locked, only at 30 dB-Hz or more, and is dropped below it.
  Narrowband/wideband on the wiped pilot would allow a lower line.

**Against Pocket SDR on the same stream** (150 s; its E1-B against our E1-C pilot):
- **Position:** ours sd 0.15 / 0.03 / 0.17 m; Pocket SDR's 0.32 / 0.23 / 0.68 m.
- **Raw pseudoranges, ours minus Pocket SDR's:** GPS sd 0.5–0.7 m, Galileo 1.2–1.7 m. E19's 8.7 m
  is our low-Doppler wander, before smoothing.
- **C/N0** agrees within 0.5 dB on every satellite. Doppler has a common −0.4 Hz, which is the
  rounding of the oscillator frequency Pocket SDR was given.
- **B1C:** Pocket SDR found no B1C signal in 150 s, on its own pilot and data searches or beside
  GPS. Ours tracks all nine with ICD-checked codes, a correlation shape that matches the theory,
  and SignalSim's documented data/pilot ratio. The cause is on Pocket SDR's side, and as a black
  box it isn't examined further.

**Limit:** E1-B I/NAV and B-CNAV1 are not decoded; ephemerides are preloaded.

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

- **This directory** is GPL-3.0 like the rest of the repository's software, except
  `core/sig/gal_e1_codes.c`.
- **Galileo E1 codes** (`core/sig/gal_e1_codes.c`):
  - The codes are Technical Data of the Galileo OS SIS ICD (© European Union), from the
    attachments of Issue 2.0's PDF. They are unchanged in Issue 2.1.
  - The ICD's Authorisation lets anyone store them, with the source acknowledged, and build
    them into receivers.
  - That Authorisation is "non-transferable and non-licensable", so the file is not under
    GPL-3.0. Each user holds the Authorisation directly from the EU, and the file's header
    says so.
  - `py/gen_codes.py` regenerates the file from the ICD.
- **BeiDou B1C codes** are generated here from the ICD's formula. Only the per-PRN parameters
  (`core/sig/b1c_params.c`) and the test values come from BDS-SIS-ICD-B1C-1.0, which sets no
  terms on their use.
- **Pocket SDR:**
  - It is BSD-2, but its README adds an export notice outside the licence: no military, WMD or
    delivery-system development use.
  - The owner's decision (2026-09-30) is to use it **only as a black-box cross-check**. No code is
    copied from it.
- **SignalSim** has no licence. Use its output files only, never its code.
- **xoshiro256\*\*** (`host/rng.c`) is written here from its public-domain description.
