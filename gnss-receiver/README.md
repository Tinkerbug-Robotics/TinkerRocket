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
- **The rig's C/N0 sweep files** (`signalsim_*_all_2026_*_w_p180*.C8`) carry GPS, Galileo and
  BeiDou B1I at 18.48 MS/s centred on 1568.286 MHz, every satellite at one level.
  - They hold the pad for 180 s, not 600. Ignition is at file second 180, so their truth is the
    scenarios shifted 420 s earlier.
  - Their 45, 51 and 57 dB-Hz files measure 41.3, 45.8 and 48.5 dB-Hz.
  - Only the traveler's 45 has the carrier offset in the file: 23.0 Hz at this centre.
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

`--jam` adds an interferer and `--mitig` a stage against it; see
[Narrowband interference](#narrowband-interference-milestone-7).

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
  - The time of week comes from a subframe's first two words (TLM, HOW: 1.2 s of clean bits).
  - It counts only once the next subframe's HOW lands 6 s later with the count one up. Two real
    words in a row always pass parity. On PSAS's flight, an almanac page (the same on every
    satellite) began a word like a preamble, and every satellite took the same wrong time from it.
  - A whole subframe that arrived before that confirmation is held, and its data is used once
    confirmed.
  - The receiver also drops any satellite whose time disagrees with the others' by more than 0.1 s.
  - Where a seed or a fix has resolved the milliseconds, the decoded message times vote against
    them; see [Seeded starts and coarse time](#seeded-starts-and-coarse-time).
  - The 10-bit week resolves against a reference week (`rx_set_week_ref`; gnssrx takes it from the
    file's date).
- **Observables and PVT:** observables come from the exact integer NCO state.
  - Doppler is the NCO's mean frequency over the last 20 ms, moved forward by the loop's rate.
  - Pseudoranges are carrier-smoothed (Hatch, 100 s, `rx_cfg_t.hatch_s`; `gnssrx --hatch S`, 0
    off) while the PLL holds, and restart when it lets go.
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

### Seeded starts and coarse time

The flight computer can hand the receiver a position and a time (`rx_set_seed`), each with a
1-sigma (owner, 2026-10-01). With it:
- **Each GPS satellite is timed from its code phase alone.** The code leaves the millisecond open;
  the predicted range settles it. The rounding is done on differences against a reference
  satellite, so an error in the seed's time, common to all of them, cancels. The position must be
  good to tens of km.
- **A time good to 0.125 ms** (prediction errors included) gives the absolute millisecond. The fix
  is ordinary from the first epoch, and Galileo and BeiDou start aided at once.
- **A coarser time** leaves every transmit time, and the receiver's clock, one whole number of
  milliseconds out together. The fix then solves that offset as one more unknown
  (`pvt_opt_t.coarse_time`, five satellites or more). The unknown's partial is each satellite's own
  range rate, and the satellites are placed at the time it gives. Aided starts wait, since a
  pilot's code would be whole milliseconds out.
- **The navigation messages settle it.** When two satellites' decoded times agree on an offset from
  the resolved milliseconds, and outnumber those that agree with them, every channel and the clock
  move by it; the pseudoranges stay as they are. Since all satellites' subframes arrive together,
  the second comes with the first.
- **A seed that claimed better than it had** is overruled by three satellites agreeing. Galileo
  restarts once the time moves.
- **Once settled,** a satellite whose message time still disagrees has its bit sync wrong (it
  decodes, shifted), and it restarts.
- **The offset is not rounded from the fixes.** Its sd is 0.25 ms on the static file, but it reads
  2–5 ms high in flight (below).
- **Only channels that agree get a millisecond (the integrity gate).** A tone near L1 leaks
  through the C/A code's spectral lines into channels that track nothing real. A seed's
  millisecond would turn them into ranges; the navigation message never would, since they decode
  nothing. So, with a seed or a fix:
  - satellites predicted below −5° aren't searched;
  - before a fix, a channel is resolved only within a group of three or more whose code phases
    (less whole milliseconds) and Dopplers agree pairwise. The windows come from the seed's
    position and velocity sigmas (`rx_set_seed_vel`; unknown velocity means no Doppler window);
    the seed's own time and clock-rate errors are common and cancel;
  - after a fix, each channel's code phase and Doppler must agree with the fix's prediction:
    5 µs and 100 Hz, widening with the fix's age at up to 30 g;
  - a channel failing for 2 s is dropped, and its PRN rests 10 s;
  - a fix using ranges no navigation message has confirmed needs a spare degree of freedom in
    its position and in its velocity, and must pass both tests. Otherwise it is withheld. Any
    handful of false channels solves exactly; it can't also agree on Doppler.
  - `rx_set_interference` (the stage's power in over out; gnssrx flags over 0.5 dB) applies that
    rule to every fix. `pvt.csv` records `dof`, `vdof` and `jam`.

`gnssrx --prior ERR_M,ERR_MS[,SIGMA_MS[,VEL_SIGMA[,POS_SIGMA]]]` seeds from the manifest's static
truth and start time, moved by those errors. It claims the time good to SIGMA_MS, rest good to
VEL_SIGMA m/s (default 1) and the position good to POS_SIGMA m. PSAS, seeded on the pad but
acquiring in flight, takes `--prior 0,0,0.001,400,1000`. `pvt.csv` records `coarse` and
`time_off_ms` per fix, and the run ends with a line on when the time settled and by how much.

The SignalSim static file, GPS + Galileo, orbits preloaded (`runs/seed`):

| Seed | First fix | Time settled | Worst error before then |
|---|---|---|---|
| None | 13.3 s | — | — |
| True time | 1.1 s | — (Galileo in at 1.5 s) | 1.5 m |
| 0.3 ms out, claimed 10 µs | 1.1 s | — (the rounding absorbs it) | 1.5 m |
| 5 ms or 0.61 s out | 1.1 s, 122 coarse-time fixes | 13.3 s, moved −5 or −610 ms | 1.4 m |
| 3 ms out, claimed 10 µs | 1.1 s | 13.3 s, moved −3 ms | 2.8 m; Galileo back at 14.5 s |
| 5 ms out, orbits not preloaded | 36.1 s (the ephemerides first) | at the first fix | — |

From 20 s on, the seeded runs with preloaded orbits match the unseeded one to a few cm: sd E
0.02–0.04, N 0.03–0.04, U 0.03–0.05 m, against 0.02, 0.03 and 0.07. The solved offset starts
within 0.8 ms of the truth and is within 0.15 ms by 6 s.

On PSAS's flight (`runs/psas81`; below), with the 50 Hz boost loops:
- **First fix at T+2.1 s** with a seed, true time or the TeleMetrum's (0.53 s early), against
  T+25.9 s without.
- **The coarse-time fixes** sit within 2.0 m of the true-time fixes (median; 95 % 7.2 m, worst
  16 m, at the join). The time settled at T+25.8 s, moved +530 ms.
- **The offset reads 2–5 ms high after the join**, smoothing on or off: about 2 m of error that
  follows each satellite's range rate, from the real sky and antenna.
- **The seed found the files' join 80 ms off** (below).

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
| `py/los_truth.py` | Each satellite's line-of-sight truth along a scenario: Doppler, Doppler rate, elevation, azimuth |
| `trksim` | One channel's real tracking code (`core/trk`) against that truth, at the level of correlator dumps. It includes the contract's command delay, data bits, and correlated early/prompt/late noise at any C/N0, and runs about 1000× real time. On the IQ files it matches `gnssrx` channel for channel: unlock time within 0.1 s, the same slips. With `--spin`, the rolling antenna's phase and gain; see [Spin and the antenna](#spin-and-the-antenna-milestone-7) |
| `py/trk_sweep.py` | Sweeps `trksim` over loop profiles, C/N0, satellites and seeds; tabulates the worst satellite |
| `gnssrx --boost-at S0,S1` | The boost profile over those file seconds, as the flight computer would call it. With `--loops-quiet` and `--loops-boost` (bandwidths), `--cn0-at S:DBHZ` (a level change mid-file) and `run.ini` (what made the run) |
| `py/boost_track.py` | A run against truth, per satellite: frequency error, unlock time, carrier slips, code error, losses as the bench counts them (no pseudorange for 0.5 s), and the fix |
| `gnssrx --imu SCEN.csv` | IMU aiding from the scenario's trajectory, with the IMU's faults (`--imu-err`); see [IMU aiding](#imu-aiding-milestone-7) |
| `py/boost_plots.py` | The figures, drawn like the rig's reports on the bought receivers. `rates` draws each satellite's line-of-sight Doppler rate through the burn on the rig's key, as a grid of runs (signal levels × loop configurations): coloured by constellation where a pseudorange was delivered, magenta where one was over 10 m off the truth, markers on carrier lock. `timeline` draws one run in full: speed, altitude and acceleration (with the IMU's input), per-satellite output, measurements per epoch, pseudorange and range-rate errors, and the fix's errors |

`boost_track.py` judges the carrier against the integral of the true Doppler. That integral is how
the rig's smoothed gps-sdr-sim builds the carrier. Range from linearly interpolated positions differs
from it by a·dt²/8 inside a 0.1 s trajectory step, which at burnout is whole cycles.

`boost_plots.py` judges each delivered pseudorange by the rig's rule for the bought receivers: over
10 m off the truth is wrong.
- **The truth:** the geometric range, plus the troposphere where the manifest says the file carries
  one (`tropo`; the receiver's own Saastamoinen model).
- **Taken out:** each satellite's pre-launch level, then the per-epoch median over satellites (the
  receiver clock).
- **The troposphere term matters on SignalSim:** it takes the aided traveler's burn from 0.93 to
  0.67 m rms.

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

**The four configurations compared.** Every comparison below runs these:

| Configuration | Pull-in FLL / PLL / DLL | Locked FLL / PLL / DLL | IMU feed-forward | When |
|---|---|---|---|---|
| quiet loops | 10 / 15 / 2 Hz | — / 10 / 0.25 Hz | none | throughout |
| 50 Hz boost loops (the fallback) | 10 / 50 / 2 Hz | 5 / 50 / 1 Hz | none | T−5 s to 2 s past burnout |
| quiet loops + IMU | 10 / 15 / 2 Hz | — / 10 / 0.25 Hz | throughout | throughout |
| IMU + 20 Hz loops (the design) | 10 / 20 / 2 Hz | — / 20 / 0.5 Hz | throughout | T−5 s to 2 s past burnout |

- **The loops.** The carrier loop is a 3rd-order Costas PLL with a 2nd-order FLL to help it. The
  code loop is a carrier-aided 1st-order DLL (`core/include/gnss/trk.h`).
- **The states.** A channel pulls in with both carrier loops wide, and narrows once its PLL locks.
  If the PLL lets go, it falls back to the FLL and keeps delivering pseudorange and Doppler.
- **The trade.** Within its 45° a 3rd-order PLL rides a Doppler-rate step of about 20 Hz/s at
  10 Hz, 80 at 20 Hz and 500 at 50 Hz. Its 3σ phase noise at 31.4 dB-Hz is 17°, 24° and 38°.
  - The fallback buys the burnout with noise.
  - The design lets the IMU predict the dynamics, and keeps the noise of a 20 Hz loop.

**What aiding buys, in short.** The 40 runs were repeated on 2026-10-01 with today's receiver
(`runs/aid`). They reproduce the tables below, except one cell, now updated.
- **Sensitivity:** carrier on every satellite through both burns down to 31.4 dB-Hz. Without
  aiding, the 50 Hz boost loops manage that only to 36.4 (hotshot) and 33.4 (traveler): aiding is
  worth 5 and 2 dB. At 31.4 dB-Hz the unaided loops keep 5 and 4 satellites of 14.
- **Vertical velocity:** three times better from 42.6 to 33.4 dB-Hz, four to five times at 31.4,
  eleven at 29.3.
- **Height:** at 31.4 dB-Hz the worst error halves on the hotshot (2.6 → 1.2 m) and falls fivefold
  on the traveler (6.0 → 1.2 m).
- **Narrow loops need both:** the quiet loops without aiding lose carrier on every satellite at
  every level, and aiding alone doesn't rescue them; the IMU's residual needs 20 Hz loops.
- **Measurement integrity** (a delivered pseudorange over 10 m off the truth is wrong):
  - The aided design delivers nothing over 1.6 m down to 31.4 dB-Hz on both flights.
  - At 29.3 dB-Hz, 5 and 7 satellites go 10–18 m off. Every one had lost carrier lock first, and
    unsmoothed code at 27–31 dB-Hz is that noisy.
  - The 50 Hz loops stay clean to 33.4 dB-Hz. They go over on 1 and 7 satellites at 31.4, and on
    13 and 14 at 29.3.
  - On the SignalSim traveler, only the quiet loops without aiding go over.
- **Figures** (`runs/aid/fig`):
  - `aid_summary.png`: satellites kept, velocity, height and unlocked time, against C/N0;
  - `aid_velocity_35.png`: the velocity error through each burn at 33.4 dB-Hz;
  - `rates_hotshot.png` and `rates_traveler.png`: per satellite.

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
| 29.3 | 14 (77 s) | 14 (45 s) | 10 (24 s) | 7 (9.4 s) |

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


### On the bought receivers' files (milestone 7)

The rig's C/N0 sweep (report "PX1105R and NEO-M8T C/N0 Sweep", 2026-09-30 and 10-01) flew the
PX1105R through both boosts on SignalSim files.
- **The files:** GPS, Galileo and BeiDou B1I, from 12 dB over to 9 dB under the 45 dB-Hz file.
- **Our runs:** `gnssrx` ran the same files in the four loop configurations (`runs/wide`), judged by
  the same truth and rule.
- **What we see:** their GPS and Galileo. Their BeiDou is B1I, outside the L1 front end's band.

The table gives the satellites locked at burnout (the PX1105R) or 1 s after it (ours). Each cell is
GPS · Galileo, plus the PX1105R's BeiDou. The "over 10 m" columns count the satellites that
delivered a pseudorange that far off.

| Flight, level | PX1105R | Over 10 m | IMU + 20 Hz loops | Over 10 m | 50 Hz boost loops | Over 10 m |
|---|---|---|---|---|---|---|
| Traveler, +12 dB | 11/12 · 2/6 · 4/4 | GPS 5, GAL 1, BDS 1 | 13/13 · 8/8 | none | 13/13 · 7/8 | none |
| Traveler, +6 dB | 9/13 · 1/7 · 7/8 | GPS 5, GAL 1, BDS 3 | 13/13 · 8/8 | none | 13/13 · 6/8 | none |
| Traveler, 0 dB | 1/13 · 0/2 · 7/7 | GPS 4, BDS 3 | 13/13 · 7/8 | none | 13/13 · 8/8 | none |
| Traveler, −3 dB | 1/12 · 0/1 · 6/9 | GPS 3, BDS 6 | 13/13 · 6/8 | none | 13/13 · 8/8 | none |
| Traveler, −6 dB | 1/13 · – · 1/7 | GPS 3 | 13/13 · 4/8 | none | 13/13 · 3/8 | none |
| Traveler, −9 dB | 0/6 · – · 0/1 | none | 11/13 · 0/8 | GPS 1 | 12/13 · 0/8 | GPS 8 |
| Hotshot, +12 dB | 5/13 · 0/7 · 6/6 | GPS 2, BDS 3 | 13/13 · 8/8 | none | 13/13 · 8/8 | none |
| Hotshot, +6 dB | 4/13 · 0/6 · 6/6 | GPS 1, GAL 1, BDS 2 | 13/13 · 8/8 | none | 13/13 · 8/8 | none |
| Hotshot, 0 dB | 0/13 · 0/4 · 5/6 | GPS 2, GAL 1, BDS 2 | 13/13 · 8/8 | none | 13/13 · 8/8 | none |
| Hotshot, −3 dB | not flown | | 13/13 · 8/8 | none | 13/13 · 8/8 | none |
| Hotshot, −6 dB | 0/9 · – · 0/8 | none | 13/13 · 6/8 | none | 13/13 · 3/8 | none |
| Hotshot, −9 dB | not flown | | 12/13 · 0/8 | GPS 2 | 11/13 · 0/8 | GPS 2 |

- **Every GPS satellite through both burns to −6 dB,** aided or on the 50 Hz loops. The PX1105R
  keeps 11 of 12 at best on the traveler and 5 of 13 on the hotshot, and from 0 dB down 1 and none.
- **No wrong pseudorange from either to −6 dB.** The PX1105R delivered them on 3–9 satellites at
  every level where it still held any. Its strength is BeiDou B1I, which it keeps where it loses
  GPS.
- **Aiding shows here as on our own files.** Count the satellites that never lost carrier from
  ignition to 2.5 s past burnout:
  - both configurations keep all 13 GPS satellites to −6 dB;
  - at −9 dB (31 dB-Hz) the aided design keeps 11 and 10, the 50 Hz loops none and 3;
  - only the aided design carries Galileo through either burn: 6–8 of 8 down to −3 dB, against
    none.
- **What differs:**
  - the PX1105R took the files as RF through the HackRF, and at 0 dB reads about 2 dB less C/N0
    than ours (38–39 against 40.4);
  - the rig attenuated whole files for −3 to −9 dB. We add noise from T−10 s, so acquisition there
    isn't tested;
  - its counts are at burnout, ours 1 s later, through the burnout transient.
- **Figures:** `runs/wide/fig/rates_wide_traveler.png` and `rates_wide_hotshot.png`.

One run, from `gnss-receiver` with `GNSS_IQ_DIR` at the rig's `c8/` (the truth shifted to the
180 s pad first):

```bash
SC=.../tools/gnss-cocom/sdr/scenarios
awk -F, -v OFS=, '$1 >= 420 { $1 = sprintf("%.1f", $1 - 420); print }' $SC/hotshot_pad600.csv \
    > runs/wide/hotshot_pad180.csv
build/host/gnssrx signalsim_hotshot_all_2026_45_w_p180.C8 --start 80 --dur 110 \
    --cn0 41.0 --cn0-at 170:35.3 --imu runs/wide/hotshot_pad180.csv --boost-at 175,186 \
    --loops-boost 10,20,2/0,20,0.5:2 --out runs/wide/BA_hot_m6
```

### Oscillator g-sensitivity (milestone 7)

The 27 MHz TCXO (Epson TG2520SMN) moves with acceleration. Its sensitivity is 0.07 ppb/g typical
on Y and Z, 0.4 on X, and 2 ppb/g the spec bound. The LO and the sample clock share it, so every
satellite's carrier shifts together, by −f_L1 times Γ times the specific force.

**How big it is.** At the hotshot's burnout the specific force falls 34 g in 0.3 s, up to 147 g/s.
That adds a Doppler rate to every channel, on top of its line of sight's:

| Sensitivity | Hotshot burnout (147 g/s) | Traveler burnout (86 g/s) |
|---|---|---|
| 2 ppb/g | 463 Hz/s | 271 Hz/s |
| 0.4 ppb/g | 93 Hz/s | 54 Hz/s |
| 0.07 ppb/g | 16 Hz/s | 9.5 Hz/s |

The aided 20 Hz loops hold about 80 Hz/s.

**The emulation.** `--osc-g GAMMA[,COMP]` turns everything received by the oscillator's phase. That
phase follows the trajectory's specific force along the thrust axis (1 g on the pad), and
`--osc-vib F_HZ,A_G` adds a vibration tone. The fix's clock drift moves by c·Γ·f as it should: at
2 ppb/g, 0.607 m/s at 1 g and 18.587 m/s at 31 g.

**Results.** IMU-aided 20 Hz loops, the realistic IMU. Each cell is the number of satellites that
lost carrier between liftoff and burnout + 2.5 s, of 14. "33" is set at 33 dB-Hz and measures 31.
Runs are in `runs/osc`; the figure is `runs/osc/fig/osc_hot_33.png`.

| Sensitivity | Hotshot 45 | Traveler 45 | Hotshot 33 | Traveler 33 |
|---|---|---|---|---|
| 0 or 0.07 ppb/g (typical: thrust on Y or Z) | 0 | 0 | 0 | 0 |
| 0.4 ppb/g (thrust on X) | 0 | 0 | 14, 2 not back | 6 |
| 1 ppb/g | 14 | 14 | 14 | 14, 2 not back |
| 2 ppb/g (the bound) | 14 | 14 | 14, 3 not back | 14 |
| 2 ppb/g, fed forward exactly | 0 | 0 | 0 | 0 |
| 2 ppb/g, fed forward at 80 % | 0 | 0 | 11 | 4 |
| 2 ppb/g, fed forward at 50 % | 14 | 14 | 14 | 14 |
| 2 ppb/g, learnt in flight | 0 | 0 | 9, at ignition | 1 |
| 0.4 or 1 ppb/g, learnt in flight | — | — | 0 | 0 |

- **The 50 Hz fallback without aiding** loses nothing at 45 dB-Hz, even at 2 ppb/g. At 33 dB-Hz it
  is at its own limit, losing 9–10 of 14 with no oscillator at all.
- **Vibration does nothing.** An 800 Hz tone of 30 g peak (Rolly Polly V's boost) through 2 ppb/g
  swings the carrier 95 Hz, 0.12 rad. Nothing was lost, aided or on the fallback.

**The feed-forward.** `rx_set_clock_rate(rx, hz_per_s, valid)` adds the oscillator's predicted
rate, −f_L1 Γ·(df/dt), to every channel's feed-forward. The P4 has the specific force from the
IMU. The words stay on the same IF, so the observables keep the true drift and the fix's drift
state takes it.

**Learning Γ in flight.** Once the burn starts, the fix's clock drift follows Γ·c·f.
- A least-squares fit against the IMU's specific force, both less their pad values, finds Γ
  within 0.5 s of ignition: 2.000 for 2 ppb/g at 45 dB-Hz, 0.066 for 0.07, and within 3.5 % at
  33 dB-Hz.
- `--osc-g GAMMA,-1` emulates the P4 doing this. It carries burnout completely.
- At 33 dB-Hz, a hotshot-like ignition (35 g/s) at 2 ppb/g still costs carrier before the
  estimate exists.

**Findings:**
- **The TCXO's X axis must stay across the thrust axis** (the board's layout rule). At 33 dB-Hz it
  is the difference between losing nothing and losing every satellite.
- **The P4 should feed the oscillator forward**, with Γ learnt in flight. A bench calibration
  (2 g flips on each axis) would cover ignition too, should a part near the bound turn up.
- **Vibration needs nothing.**

### Spin and the antenna (milestone 7)

A spinning rocket turns its antenna. The received carrier of the right-hand circular signal turns
with it, a cycle a revolution (the antenna's wind-up), and its amplitude follows the antenna's
pattern toward each satellite.

**The model** is `trksim`, per satellite:
- `--spin HZ[,T0,T1]` sets the roll rate, ramping up through the burn;
- `--antenna nose|side` puts the patch in the nose, looking up the roll axis, or on the side,
  looking out;
- the patch is two crossed dipoles fed 90° apart, the second weaker by the axial ratio (`--ar`, on
  the boresight and 90° off it). The signal's voltage on them gives the carrier its phase and
  amplitude, pattern included;
- behind a side patch the body blocks it (`--floor`);
- `--aid-spin` feeds the predicted wind-up forward, as the gyro and attitude would.

`py/los_truth.py` now gives each satellite's azimuth for this. The runs use the hotshot and
traveler skies, IMU-aided 20 Hz loops, and spin ramping up through the burn. The figure is
`runs/spin/fig/spin_antenna.png`.

**On the roll axis, spin costs nothing at good signal.**
- A perfect patch there sees exactly a cycle a revolution from every satellite at every
  elevation, at a steady gain. That is a common frequency offset of the spin rate. Every loop
  tracks it, and the fix's clock drift takes it: 8 Hz is 1.5 m/s of drift, and the velocity is
  untouched.
- A real patch (axial ratio 1 dB on axis, 8 dB at the horizon) ripples at twice the spin rate for
  low satellites: ±28° and 5 dB at 10° elevation.
- At 45 dB-Hz all 14 satellites track clean through 8 and 20 Hz of spin, on both flights.
- At 35 dB-Hz on the boresight, the low satellites are already marginal from the pattern: 6 of
  14 are clean without spin, and the ripple leaves 2–3.

**On the side, spin is fatal.**
- The patch faces each satellite for only part of each turn: fades of 15–40 dB, and phase swings
  of up to ±170° a revolution.
- Without spin it already loses the 6 satellites behind the body.
- At 2 Hz it loses all 14, even at 45 dB-Hz. Feeding the wind-up forward from the gyro doesn't
  help through the fades.

**For the vehicle:** the GNSS antenna belongs on the roll axis (in the nose, looking up), with as
good an axial ratio toward the horizon as can be had. A side mount would need an array around the
body, which isn't studied here. Traveler IV's 6–8 Hz spin is no problem for a nose patch.

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
- **Real motors add what the files lack.** The oscillator's g-sensitivity, vibration, spin and
  antenna phase are covered above. The plume's attenuation is not.

## Real flight data: PSAS Launch-12 (milestone 7)

Portland State Aerospace Society's LV2 flew on 2015-07-19 at Brothers, Oregon. Its jGPS v3 recorded
the raw signal: a passive antenna, SAW filters, a MAX2769B at 4.092 MS/s, zero IF, 2-bit. The
recording runs from T−32 s to T+34.7 s ([psas/Launch-12](https://github.com/psas/Launch-12),
`data/GPS`). The flight:
- 33 g peak;
- burnout at T+5.7 s and 378 m/s (Mach 1.1);
- apogee 4,781 m above the pad at T+30.7 s, by the TeleMetrum altimeter on the same rocket.

The data stays outside the repo. The manifest describes the files; point `$GNSS_IQ_DIR` at them.

**What kept receivers off it.** PSAS's own software never tracked it, and their COTS receiver
called it "pretty much garbage". `py/psas_condition.py` fixes four things and writes an 8-bit
file:

| Impairment | Measured | Done |
|---|---|---|
| An on-board carrier near +418 kHz, drifting a few kHz | 39 dB over the noise in a 250 Hz bin, with the 2-bit quantizer's harmonics at 832 kHz and 1.26 MHz | Excised: 4096-point sqrt-Hann frames, bins standing 8× over the median zeroed (7.5 to 17 per frame) |
| DC on Q | −0.14 to −0.20 | Removed |
| I/Q imbalance | I and Q correlated +0.17 to +0.26, 0.4–0.5 dB apart | Balanced |
| The spectrum inverted against ours | Each satellite at minus its predicted Doppler | Conjugated |

Without the inversion fixed, carrier-aided code tracking runs the wrong way.

**The gap between the two files is 81 ms + 296 samples** (81.072 ms), against the 73.166 ms the
file names imply:
- **The 296 samples** (0.0723 ms) come from six satellites' code phase.
- **The bit alignment narrows the whole milliseconds** to 1, 21, 41, 61 or 81. The receiver's C/N0
  estimate, which needs its blocks inside data bits, runs smoothly through any of those and dips
  at 72, 73 and 74 ms. PSAS's README says "about 1 ms", and the first join used 1 ms.
- **Coarse time settled the 80.** The first file alone, seeded with the time the second file's
  fixes give (carried back across a 1 ms join), solves its offset at +78.7 ± 0.9 ms. Its plain fixes
  fail the residual test, or come out 4 m rms on 5–6 satellites. With the start 80 ms earlier the
  offset reads −1.3 ms, and all 25 fixes pass at 1.1 m rms on 7–9 satellites.
- **On the 1 ms join** the carrier also missed 80 ms of range change. The smoothed pseudoranges
  carried that through the second file, decaying over 20 s.

`PSAS_L12_cond_g81.C8` joins them at 81 ms + 296 samples of zeros, and the loops coast through.

**What our receiver does with it** (`gnssrx PSAS_L12_cond_g81.C8 --if 0 --preload-gps --acq-interval 1`;
`--if 0` because the 4 MHz-wide band would alias at the plan's IF):
- **On the pad the antenna saw almost nothing.** A 200 ms search reads 1.4–2.8 against a noise
  floor of 1.35. At liftoff every satellite comes up about 10 dB within a second.
- **All 9 visible satellites are acquired by T+1–2 s,** in the middle of the burn.
- **The 50 Hz boost loops hold carrier through the burn** at 37–41 dB-Hz, where the quiet loops
  flicker in and out of lock.
- **C/N0 falls 4–6 dB at the join of the files.** The second file's interference is stronger:
  17 bins excised against 7.5.
- **Without a seed, the first fix comes at T+25.9 s** with the boost loops kept for the whole
  flight; the first confirmed time is at T+19.8 s.
- **Near apogee the height reads about 270 m above the barometric altimeter** (5,053 m above the
  pad against its 4,781). The accelerometer's integration gives 5,091 m. A barometer reads low on
  a hot day by about that much.
- **The vertical velocity follows the TeleMetrum's integrated accelerometer.**

**What holds the fix back is time.** The receiver gets time only from the navigation message,
which needs two clean subframe headers 6 s apart. That is rare in flight.

**IMU aiding needs a line of sight**, which needs a fix. With the pad blocked, neither came until
T+25 s.

The owner chose (2026-10-01) to let the flight computer seed position and time; see
[Seeded starts and coarse time](#seeded-starts-and-coarse-time). With the seed the first fix
comes at T+2.1 s, even from the TeleMetrum's time, 0.53 s early. One or two satellites per run
decode a time the rest contradict (their bit sync off by whole milliseconds), and restart.

**Also fixed for it:**
- GPS ephemerides can be preloaded (`--preload-gps`), as the flight computer could hand them over.
- `--acq-interval` sets how often the receiver searches.
- The week rollover is resolved: a 2015 recording read 1024 weeks late.
- RINEX 2 broadcast files (the IGS's for that day) load in C and Python.
- PSAS's packed 2-bit format reads directly (`format = max2769_2bit`).

**The on-board carrier through our own stages:** see
[Narrowband interference](#narrowband-interference-milestone-7). Either stage gives back what the
offline excision did.

## Narrowband interference (milestone 7)

The owner's question (2026-10-01): what does a narrowband interferer cost our 2-bit chain, and what
would an FPGA notch or excision stage buy? It is studied here in software, before anything goes to
the hardware session.

**The model.**
- **`--jam`** adds the interferer at complex baseband, together with the thermal noise and ahead of
  the IF filter and the MAX2769B's 2-bit AGC quantizer, where a real one arrives (`host/jam.c`):
  - a carrier, `cw:F_HZ:JNR_DB`;
  - band-limited noise, `nb:F_HZ:JNR_DB:BW_HZ`;
  - a swept carrier, `chirp:F_HZ:JNR_DB:SPAN_HZ:PERIOD_S`.

  JNR is its power over the noise in the 4.2 MHz IF band; 0 dB is a tone of about −106 dBm at
  the LNA. `--jam-at S` switches it on S seconds into the run.
- **`--mitig`** runs float models of two candidate FPGA stages (`host/mitig.c`). Both sit on the
  2-bit samples after the decimator and requantize to 2 bits with their own AGC, so the
  correlators stay as they are:
  - `anf`: adaptive notches in cascade. A complex zero is adapted by normalized LMS onto the
    interferer, with a pole at 0.99 times it (about 20 kHz wide).
  - `fde`: frequency-domain excision. It takes 1024-point sqrt-Hann frames at 50 % overlap and
    zeroes the bins whose power, averaged over 50 ms, stands 8 times over the median.
- **The test:**
  - the gps-sdr-sim hotshot pad: 14 GPS satellites at 42.7 dB-Hz, 30 s, judged over 15–30 s;
  - "locked" means a real satellite at its true Doppler, PLL locked;
  - the direct 6.75 MS/s path. The 27 MS/s chain with its decimator agrees: with a notch at 20 dB,
    36.5 dB-Hz both ways.
  - Runs are in `runs/jam`; the figure is `runs/jam/interference_study.png`.

**An unmitigated tone near L1 captures the receiver at about the noise power.**
- The C/A code's spectrum is a comb of 1 kHz lines, the strongest only about 20 dB below the
  code's whole power. A tone leaks through them into every PRN's acquisition, at some Doppler.
- 300 kHz from L1:
  - false channels appear from −5 dB JNR;
  - at 0 dB, 11 of 14 satellites lock, with 19 false channels;
  - at +5 dB, none.
- Placement matters:
  - 20 kHz from L1 leaves 2 of 14 at 0 dB;
  - at 700 kHz all 14 hold, with 3 false channels;
  - at 1 MHz (the C/A spectrum's first null) and at 1.9 MHz, nothing happens.

**Once the tone is removed, what remains is the 2-bit quantizer's loss.** The tone holds the AGC's
thresholds, and the weak signal crosses them less often. No stage after the ADC recovers it.
- Either stage measures the same loss, and it follows the theory: the quantizer's small-signal
  gain and noise, averaged over the tone's phase.
- The loss is 0.4 dB at 0 dB JNR, 2.3 at 10, 6.1 at 20 and 11 at 30.

**The AGC target sets that loss above 20 dB.**
- The MAX2769B holds its magnitude bits at a target density: 33 %, its GAINREF register.
- Under a strong tone, a lower target puts the thresholds where the tone crosses them slowly. At
  30 dB JNR, a 0.10 target gives 34.7 dB-Hz, matching the theory's 34.3.
- It costs 0.3 dB with no tone (theory) and measured 0.7 dB at 15 dB JNR, so it belongs only
  where a tone is detected.

**Tracking holds further than acquisition.** Our 10 ms snapshots need about 36 dB-Hz.
- With the tone present from the start, the notch acquires to 20 dB JNR (excision to 15).
- Switched on after every satellite was locked:

| JNR | No stage | Notch | Notch, AGC target 0.10 | Excision | 100 kHz noise, excision |
|---|---|---|---|---|---|
| 15 dB | 3 of 14; 26 false | 14 at 39.2 dB-Hz | 14 at 38.5 | 14 at 39.0 | 14 at 36.6 (no stage: none) |
| 20 dB | none | 14 at 36.5 | 14 at 37.5 | 14 at 35.0 | 14 at 32.8 |
| 25 dB | none | 14 at 32.6 | 14 at 36.3 | 14 at 30.9 | 6 at 28.0 |
| 30 dB | none | 6 at 28.6 | 14 at 34.7 | none | none |
| 35 dB | none | none | 14 at 32.7 | none | none |
| 40 dB | none | none | 10 at 29.6 | none | none |

**Wider and moving interference:**
- **100 kHz-wide noise raises the noise floor.** It doesn't leak through lines: there are no false
  channels, but at 5 dB JNR every satellite goes. A 20 kHz notch can't cover it; excision holds to
  15 dB from the start and to 20 dB after lock.
- **A swept carrier** (±1 MHz every 10 µs, 20 dB) defeats the notch. Excision keeps 7 of 14.
- **More notches or slower adaptation don't move the limits.** Four notches catch the quantizer's
  images of the tone too: one sits 1.09 MHz below L1 for a tone 300 kHz above. Neither that nor
  a step four times smaller moves the 20–25 dB limit; the quantizer sets it.

**On real data: PSAS's on-board carrier.**
- It sits 411–419 kHz below L1, at −4 to +4 dB JNR in their 2-bit recording. That is the regime
  where an unmitigated receiver is captured.
- **Left in, ours locks nothing in flight.** On the pad, where the antenna saw no real satellite,
  it reported 56 fixes, 95–5,126 km off (median 4,791). They came from 4–6 false satellites,
  leaving the residual test 0–2 degrees of freedom: little or nothing to test. With the
  integrity gate (see [Seeded starts](#seeded-starts-and-coarse-time)) it reports none. Only
  satellites above the horizon are searched, and their captured channels never agree.
- **One notch finds the carrier unaided** (−416.7 kHz) and gives back what the offline float
  excision did: 9 satellites against 10, 32.8 dB-Hz against 33.5, all 327 fixes, 3.5 m (median)
  from those fixes.
- **Excision:** 10 satellites, 33.2 dB-Hz.

**FPGA cost** *(estimates)*, against the ECP5-25's 28 multipliers and 24k LUTs:
- **The notch, per notch:**
  - two complex multiplies per sample at 6.75 MS/s;
  - the pole factor is a shift (k = 1 − 2⁻⁷), and so is the step: the AGC holds the input power,
    so there is no normalizing divide;
  - at 108 MHz one 18×18 multiplier covers it in 8 of the 16 cycles per sample, and the
    recursion's one-multiply latency fits;
  - with the requantizer's comparators and density counter: about 1–2 multipliers and 0.5k LUTs.
- **Excision, 1024 points:**
  - forward and inverse transforms, each at 13.5 MS/s with the overlap: about 135 M butterflies
    a second, two memory-based butterfly units at 108 MHz (6–8 multipliers);
  - 150–200 kbit of block RAM: frames, twiddles, overlap and averaged bin powers, with a running
    threshold in place of a median;
  - 3–5k LUTs.

**In fixed point** (`--mitig anfq`; formats approved by the owner, 2026-10-01):
- the pole factor 1 − 2⁻⁷ and the step 2⁻¹⁰ are shifts;
- the step is normalized by the leading bit of the pole state's power (no divider);
- the zero is accumulated in Q1.28 and multiplied as Q1.16, so it fits the ECP5's 18×18
  multipliers. The 12 guard bits matter: without them, updates under one bit of z were lost and
  the notch stalled against strong tones;
- the pole state is 16 bits (11.4), saturating.

In the receiver it matches the float notch: C/N0 within 0.05 dB and the same locks, from no tone
to 25 dB JNR after lock.

**The bit-exact model** is `fpga/model/notch.{c,h}`. The header spells out every operation and
its rounding for the HDL.
- **What it has:** two notches in cascade; the requantizer, with an integer AGC
  (T += T·(bits − 169) >> 16 per 256 samples, the MAX2769B's third at a 10 ms time constant); and
  the power counters the P4 reads each millisecond.
- **How it's checked:** `Notch.EqualsTheWordLengthStudy` holds its output to the study's sample
  for sample, and `Notch.OutputIsPinned` pins a checksum over an integer-only input.
- **Running it:** `gnssrx --mitig notch` runs it. With `--vectors DIR` it writes its own vectors
  beside the correlator's: `notch_in.u2`, `notch_out.u2`, `notch_power.csv` (counters, threshold
  and each zacc, per ms) and `notch.ini`. `vecreplay` checks both sets: 300 ms, 2,025,000
  samples, 0 mismatches each.
- **In the receiver:** it gives the float notch's C/N0 within 0.1 dB. At 20 dB JNR it gives
  0.5 dB more (14 satellites, not 13, from the start), because the second notch takes the
  quantizer's image of the tone.

**The interference flag** is the stage's power in over power out.
- With no interferer, either stage takes out 0.02 dB. The notch's own depth can't serve as the
  detector: with nothing to remove it settles on the IF passband's hump near L1 (|z| 0.99), which
  is harmless (42.68 dB-Hz either way).
- A tone at −10 dB JNR takes out 0.33 dB, and PSAS's carrier up to 8.2 dB. gnssrx flags over
  0.5 dB, and the flag makes every fix meet the gate's redundancy rule.

**Open:**
- **Longer acquisition snapshots** would let acquisition reach as far as tracking under a tone.
- **The AGC-target policy** (lower it while the flag holds) is the P4's, not yet emulated here.

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
