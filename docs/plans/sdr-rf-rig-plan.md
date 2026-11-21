# The gnss-sdr-dev board and the V10 on the HackRF rig: plan

**Written 2026-11-02, branch `fc/sdr-raw-offline`; set up 2026-11-09 (below: "Ready").** The next step of `sdr-raw-filter-plan.md`: the board's real
tracking of a rocket flight it is played over RF, with truth, through the raw-GNSS filter. Nothing here has run. The
HackRF rig (RFRig) belongs to the "PX1105R firmware with external IMU" session, which runs every test on it; the
board and the V10 sit on the open-sky rig (SkyTraqRig, the "PX1105R clean firmware study" session's); the board is
the "SDR dev board bring up" session's. Moving hardware is the user's.

## Two stages

- **A, now, no firmware change:** the board tracks the played flight; the V10 (the image of PR #1688) and the
  board's own NAND recorder log its RTCM3; the V10 stands still. Offline, the logged stream is paired with an IMU
  synthesized from the same flight's truth, and `raw_replay --receiver sdr` is scored against that truth.
- **B, later, full hardware in the loop:** the V10 flies a TinkerRocket-sim flight's IMU live (TR_SIM_REPLAY) while
  the board tracks that flight's RF, the raw filter on board. It needs the FC's SDR raw outputs and a replay clock
  fed from the board's epochs (`sdr-raw-filter-plan.md` step 4), and a B1C file of such a flight (rp5's recipe is in
  `bench-backups/c8-recipes-2026-10-19.tar.gz`; today's rp5 files carry B1I, which the board does not track).

## The files (stage A)

Made for this receiver (gnss repo `cocom-rig/sdr/signalsim/make_b1c_flights.py`, configs beside it): GPS L1 C/A,
Galileo E1 and BeiDou B1C, narrow, 8.184 Msps at 1575.42 MHz, every satellite at 45 dB-Hz nominal (40.8 measured on
GPS), no carrier offset in the file, the scenario starting 2026-08-18 08:36:42 UTC, ignition at file second 180.

| File | Length | Flight | Truth (gnss `cocom-rig/sdr/scenarios/`, 420 s later) |
|---|---|---|---|
| `signalsim_static_gpsgalb1c_2026_45_n.C8` | 300 s | static | the config's position |
| `signalsim_hotshot_gpsgalb1c_2026_45_n_p180.C8` | 300 s, 4.9 GB | 10-40 g, 4 s burn, to T+120 | `hotshot_pad600.csv` |
| `signalsim_traveler_gpsgalb1c_2026_45_n_p180.C8` | 540 s, 8.8 GB | 6-18 g, 13 s burn, to T+360 | `traveler_soft25_pad600.csv` |

None is on the Mac now (the 10-19 clean-up). If the rig host lacks them, they regenerate from those configs.

## Setup

- **RF:** one HackRF board tuned to 1575.42 MHz at 8.184 Msps (a single-band play, not the dual L1/L5 setup), through
  attenuators and a **DC block** into the board's antenna input: the board biases its antenna port. Level: the
  board's own C/N0 at 40-45 dB-Hz on the pad. The rig's carrier fix (`dual_tx.py` CARRIER_FIX) is measured only for
  1568.286 / 1176.45 MHz at 17.5 Msps: the static run measures this tuning's from the board's code minus carrier.
  Nothing else on that RF: the rig's PX1105R does not track B1C, and the files carry no B1I.
- **The board:** its USB on a host for its console. Before each run a cold start (the files' sky is 2026-08-18, and
  the board keeps time, position and ephemerides from the real sky: how, the bring-up session knows), measurement
  mode 100 Hz, UART on at 2 Mbaud, recorder on.
- **The V10:** J1 to the board's J401 as now, on USB; the FC image of PR #1688 as flashed. Logging by BLE cmd 23 from
  the Mac (the app closed) on at least one run: it is what checks the flight computer's own timing and logging at a
  flight's data rate. A 300 s OC log at ~200 kB/s is ~60 MB, ~37 min over BLE; the board's recorder (~155 kB/s over
  USB) covers every run's RTCM.

## Runs

1. Static, 300 s: acquisition, the level, the carrier offset, the time sync.
2. Hotshot, 300 s, three times.
3. Traveler, 540 s, twice.

With the cold starts and downloads, about three hours.

## Offline (stage A)

- **An IMU from the truth** (`tools/raw_replay/sdr_rig_events.py`, written 2026-11-09): the trajectory's specific force and an
  attitude along the flight path (vertical on the pad, no roll), ISM6-like noise and bias, the barometer from the
  altitude, the magnetometer from WMM at the pad, as I/B/M events at the flight computer's tick rate.
- **Merged with the logged frames by GPS time:** the V10's run places them by its time sync as in flight; a
  recorder-only run gets arrivals from GPS time plus a nominal delay.
- **Scored** against the truth by phase (pad, boost, coast, descent): position, velocity, attitude; the unaided twin,
  and GNSS cut at ignition (the user's rule: assume GNSS lost in boost; the bar is not meaningfully worse than
  unaided). Beside the rig's earlier hotshot numbers for the mosaic-G5 and the PX1105R.
- **The board's own tracking through boost**, from its stream alone: locks lost and regained, C/N0, its marks,
  against what its core predicted offline on the same flights (gnss repo, receiver/README.md "Boost dynamics").

## Ready (2026-11-09, the board and the V10 on the open-sky rig's Pi, no antenna)

- **The V10:** the image of PR #1688 merged with main, `76d77c31-v10+20261109-1758`, in the FC's ota_1 (otadata seq
  2); it validated itself (rollback cancelled), finds no receiver at boot, remembers the board and listens at 2 Mbaud,
  "nothing heard yet" every 30 s (the board sends from its first fix). Before it, ota_1 held the PX100 carrier's
  bridge: saved on the Pi as `~/fc-sdr-rtcm/before-20261109/ota1_px100_bridge.bin` (= `~/px100-bringup/fc_px100.bin`)
  with that otadata (`otadata.bin`, both slots valid, seq 1 and 2). The flight image (600569bb) in ota_0 untouched.
- **The board:** firmware a60b606a (2026-11-03). Its boot: antenna bias on, the measurements mode at 100 Hz, the
  recorder on, the UART off. GNSS main's 91c2efb (the L5 bank) needs a USB flash (its 3 MB partitions; S601 to
  USB-Serial/JTAG and back) and changes nothing the FC sees on this V1 board (its L5 chain is dead); its stream
  decodes in the FC's code, L5 cells and all (`bench-backups/v10-sdr-logs/l5-crosscheck`).
- **The run kit** on the Pi, `~/fc-sdr-rtcm/rfrig/` (copy in `bench-backups/v10-sdr-logs/rfrig-kit/`), dry-run twice
  without a sky:
  - `preflight.sh`: both boards on USB (the FC by its MAC, the board by its serial), the FC's console line, the
    board's state. Changes nothing.
  - `run_start.sh NAME`: the board's navigation store cleared (its Galileo/BeiDou ephemerides are of today's sky,
    the files' 2026-08-18; it refills on the next real sky; `--keep-store` keeps it), a reboot (cold; GPS always is),
    antenna bias off (the HackRF feeds the port), the 100 Hz mode checked, the recorder's first page noted, the UART
    on at 2 Mbaud only with the FC on USB, the FC's console logged. Then start the file.
  - `run_end.sh NAME`: the UART off (`rx out uart off`; `rx out off` is refused), the board's state, the recorder's
    pages of the run read back (`runs/NAME/rec_sNNN.rtcm3`), the recorder on again.
  - The OC log is the Mac's (BLE cmd 23 on and off, then the download): only within BLE range of the V10.
- **Offline:** `tools/raw_replay/sdr_rig_events.py` makes each run's events file from the config and the board's
  stream, the IMU from the file's own trajectory (the plan's `scenario_imu.py`; the config is the truth, so the
  traveler needs no `traveler_soft25_pad600.csv`), `score_sdr_rig.py` scores aided and GNSS-lost runs against it
  (README, "An RF-rig run"). Checked on the hotshot's IMU alone and on the bench log re-timed (above).
- **The truth's facts** (from the configs): hotshot burnout T+3.9 s, apogee 27.4 km at T+73.1 s, 35 g, the file to
  T+119.9; traveler burnout T+13.0 s, apogee 102 km at T+152.0 s, 17.8 g, -942 m/s on the way down. The pad is at
  0 N 119 W, 1200 m: there the filter's constant gravity is 0.03 m/s^2 off, which its pad bias takes, but not the
  height term or the Coriolis force of the climb (README).

The board flies a60b606a on the rig (user, 2026-11-09): no USB flash first. Still open: the rig's slot and the
answers below (asked 2026-11-02, no reply yet); the OC log over BLE needs the Mac near the V10.

## Results, stage A (2026-11-09, the HackRF rig, the user's go-ahead)

The board's input cabled to the HackRF rig's L1 board (2b815b13, TX gain 40, amp and antenna power off), the board and
the V10 on the open-sky rig's Pi. The rig's SD card had 4.5 GB free, so the scenes were made there one at a time in
`~/sdr-rig` with a 120 s pad (start 60 s later, ignition still TOW 204000.1) and the flights cut at T+120:
`sdr_static_b1c_200`, `sdr_hotshot_b1c_p120`, `sdr_traveler_b1c_p120` (configs in `bench-backups/v10-sdr-logs/
rfrig-20261109/configs`). Each run: the board cold (`rx stop`, store cleared, `rx start`), the scene's Galileo/BeiDou
ephemerides (BRDC_2026230 08:00-09:00) loaded at its first fix (a cleared store has no almanac: without them GPS
only), every second of the TX at 16.3 MB/s (no underrun in any run). Everything is in `bench-backups/v10-sdr-logs/
rfrig-20261109/`; the analysis scripts in `rfrig-kit/analysis/` (`analyze_run.sh`; the waterfall and the run comparison
are the PX1105R 0xE5 work's `plot_e5_cn0.py` and `compare_runs.py`, adapted).

- **Level:** GPS 37-39 dB-Hz at gain 30 and 40 alike (the file's 45 sets it); Galileo E1-C ~34, BeiDou B1C pilot ~35.
  First fix 63-82 s after the TX starts; Galileo/BeiDou locked ~10 s after the ephemerides.
- **The V10's link:** 100.0 epochs/s, up to 29 observables, 402 frames/s, no epoch gaps or CRC fails in flight.
- **The board through boost** (availability of the pad's satellites, the 0xE5 work's phases):

  | | GPS | Galileo | BeiDou |
  |---|---|---|---|
  | hotshot (35 g, 3.9 s), 3 runs: boost / coast 4-22 s / after | 100 / 100 / 100 % | 100 / 63-87 / 93-98 % | 41-66 / 25-50 / 88-92 % |
  | traveler (18 g, 13 s), 1 run: boost / coast 13-30 s / after | 100 / 100 / 100 % | 100 / 72 / 89 % | 100 / 38 / 88 % |
  | PX1105R on its hotshot scene (r3-r6): boost / coast | 7-10 / 1-9 % | 4-6 / 0-9 % | 65-86 / 29-67 % |

  The losses are at the jerks: BeiDou at the hotshot's ignition (35 g onset) and both pilots at burnout; GPS L1 C/A
  never. Lost satellites come back 32-34 s later (median 32.7 s, settled 36.6 s): the receiver holds an aided start
  that stopped for 30 s (see "Change 1" below; this line first said 10-20 s, read off the waterfall).
- **The V10's filter on them** (raw_replay --receiver sdr, the IMU from the scene; aided / GNSS cut at T+0, median):
  hotshot coast 0.10-0.12 vs 3.7-4.0 m/s, 0.6-1.1 vs 96-118 m horizontal, 0.4-2.2 vs 11-14 m vertical; descent 0.05
  vs 6.5 m/s; traveler boost 0.14 vs 1.5 m/s, coast to 97 km 0.08 vs 11.2 m/s, 0.4 vs 309 m. Every boost Doppler and
  pseudorange of the board used (0 refused, NIS <= 9.7); 0.5-0.8 % of the Dopplers refused in coast and descent.
- **Open:** the vertical carries a 1.5-6 m bias (no ionosphere in the stream); in the traveler 4 satellites dropped on
  the pad at T-10 s (not dynamics: the same sky held in the hotshot); the harness's apogee quorum never calls on these
  files (no receiver fix for its GPS voter; the barometer above its range).

## Change 1: fast re-acquisition, and the sample-wrap bug it uncovered (2026-11-09, the HackRF rig)

The user's pick of the improvement round. Board firmware on a60b606 (GNSS branch `receiver/fast-reacq`, worktree
`Code/.claude/worktrees/gnss-fast-reacq`, uncommitted), sent over the console's OTA; images, patches and figures in
`bench-backups/v10-sdr-logs/rfrig-20261109/fast-reacq/`. Every run the hotshot as above (cold, scene ephemerides,
no TX underrun but rq4's).

- **Why they came back late:** an aided start whose channel stops waits 30 s (rx.c), so a pilot lost at a jerk sat
  out most of the coast whatever the sky.
- **The change:** a pilot lost after it had tracked goes into the next aiding plan (1 Hz) and, for 60 s, a start that
  finds nothing tries again a second later; the plan carries the fix to the start's time (its velocity and the
  acceleration from the last two fixes: the fix is 0.25 s old, 210 m and 45 Hz off at 840 m/s and -35 m/s^2), and the
  loop's rate starts at the acceleration along the line of sight.
- **The bug:** the tick extended each dump's 48-bit sample count as if no dump were newer than the tick. One closing a
  few samples after the tick's own count (31 on the rig) read as 2^48 samples back: the loop's next update ran over
  -4e7 s, its Doppler went to 8.2e15 Hz (the NCO pinned at fs/2) and the channel was lost for good. The black box
  (`rx bb status`, `sdrdev.py rx-blackbox`, `host/rx_blackbox.py`) caught it on stock a60b606a (hb2) and on the
  re-acquisition build (rq6), each time on the one channel with a dump newer than its tick. It is the "GPS fades"
  (2-5 GPS lost at once, never back: trav1, hb2, and every rq run, where the extra starts made it likelier) and the
  bring-up's PRN 28 at fs/2. Fixed with the signed difference (`corr_tsamp_diff`); host test `RxTick.*`. GNSS main
  91c2efb has the same line.
- **Results** (lost Galileo/BeiDou satellites, from the last cell before the gap):

  | | runs | back after (median) | settled (median) | GPS lost for good |
  |---|---|---|---|---|
  | stock a60b606a | hot1-3, hb1-2 | 32.5-32.9 s | 36.4-36.8 s | hb2 (G11 at burnout), trav1 (3 on the pad) |
  | re-acquisition, backoff to 4 s (v3) | rq1-3 | 2.3-9.6 s | 6.8-13.6 s | every run (the wrap bug) |
  | re-acquisition, 1 s retries (v4) | rq4-6 | 5.2-5.8 s | 9.7-9.8 s | every run (rq4 also lost the HackRF's USB) |
  | **re-acquisition + the wrap fix (v5)** | **rs1-2** | **2.6-5.4 s** | **7.0-10.0 s** | **none; black box never triggered** |

  Satellites lost at burnout come back in 2-3 s; BeiDou lost at the hotshot's ignition in 6-9 s (the starts during
  the boost fail: the board's IMU is still on the rig and the board does not use it). Coast (4-22 s) availability,
  stock hb1-2 vs v5 rs1-2: Galileo 63-87 % vs 96-98 %, BeiDou 38-50 % vs 87-89 %; GPS 92-100 % vs 100 %
  (`figs/compare_stock_vs_fast_reacq.png`). The V10 filter's scores barely move on this scene: it takes 16
  satellites an epoch and GPS alone nearly fills them; the gain is the margin for a flight that loses GPS.
- **P4 traps met on the way:** rx_live.c keeps two 126-entry rx_aid_t arrays in internal BSS, so 8 bytes on the
  struct (+2 kB) stopped `rx start` at boot (ESP_ERR_NO_MEM on the l1_675 stream); esptool's segment list does not
  show BSS. v5 is byte-for-byte a60b606a's internal RAM.

## Why the pilots drop at the jerks (2026-11-09, root cause)

Tools and data in `bench-backups/v10-sdr-logs/rfrig-20261109/fast-reacq/rootcause/` (truth Doppler from the scene's
own ephemeris `BRDC_2026230_MN.rnx` and trajectory; black boxes; `pilotsim.cpp`, the receiver core's trk.c on
synthetic pilot dumps with the scene's acceleration).

- **The Doppler values are right.** The board's MSM7 Doppler against the truth (the clock's common drift out):
  0.000 / 0.10 m/s GPS, 0.00 / 0.25 Galileo, 0.00 / 0.35 BeiDou (median / 90 %) on the pad and in coast, so the scene,
  the signs, the carrier and BeiDou's time are right. The fix's velocity is within 0.4 m/s in coast and 1.3 m/s
  through the boost. An aided start's Doppler is right in steady flight (E21 at T+6.3: 2.6 Hz) but 22-39 Hz out
  (x sin el) in the first second after burnout: the plan's acceleration, from two 1 Hz fixes, spans the burnout
  (rs1: +34 against -36 m/s^2) or is missing (rq6: no velocity in the T+4.0 fix). That is why the first retry
  after burnout fails and the second takes.
- **Nothing aids the loops in the boost.** On the board `rx_set_boost(rx, 0)` runs every tick, `rx_set_accel` is
  never called and vector tracking is off: every channel runs the quiet profile (locked: PLL 10 Hz, no FLL; pull-in:
  FLL 10 Hz + PLL 15 Hz), only the start is aided. Pilots are capped further by their long dumps (Bn T <= 0.05:
  E1-C 4 ms 12.5 Hz, B1C 10 ms 5 Hz) and their discriminators are unambiguous only to +-125 / +-50 Hz.
- **The mechanism.** A jerk (burnout: +344 -> +153 -> -40 m/s^2 in two 0.1 s steps, the scene's segments, 384 m/s^2,
  ~2,020 Hz/s x sin el; ignition: 0 -> 73 m/s^2, ~380 Hz/s x sin el) drives the frequency error past that range; the loop cannot pull back, the pilot's C/N0 estimate (a
  product of consecutive dumps, blind to a constant frequency error but not to a ramping one) falls under 25 dB-Hz,
  the channel coasts on the boost's rate and is dropped a second later. rq6's black box: every pilot overshoots
  +30-120 Hz at burnout; E21 and E27 passed +200 Hz and ran away.
- **Galileo's upside-down elevation pattern.** The 25 g ramp knocks the high satellites (E13, E26, E29) out of PLL
  lock, so at burnout they are in pull-in with the FLL on and ride it (0 of 13 runs lost); the lower ones re-lock
  in the boost and meet burnout on the bare 10 Hz PLL. Every Galileo burnout loss on the rig was a channel in lock
  (7 of 7, from the MSM7 lock time). BeiDou is in pull-in through the whole boost, its FLL capped at 5 Hz: the jerk
  along the line of sight is 1,170-1,820 Hz/s for C31-C39 (all lost), 670-740 for C22/C26 (3 of 11-13), 450-500 for
  C19/C29 (never). At ignition the locked BeiDou meet 380 Hz/s x sin el on a 5 Hz PLL.
- **The simulator reproduces it** (quiet profile, 34 dB-Hz, 10 seeds): Galileo sin el 0.29 / 0.43 / 0.58 / 0.82 /
  0.93 drop 1 / 6 / 6 / 0 / 0 of 10; BeiDou 8-10 of 10 from sin el 0.33, none at 0.22-0.25.
- **Fixes, in the simulator:** the FLL kept on in lock (5 Hz) loses no Galileo at any elevation (BeiDou unchanged;
  the loop's pad Doppler noise 1.2 -> 1.8 Hz E1, 0.4 -> 0.8 Hz B1C); acceleration aiding along each line of sight
  (the core's `ff_rate`) 50 ms late and 10 % off loses no Galileo, 20 ms / 5 % no BeiDou up to sin el 0.75, and
  10 ms / 2 % none at all. The aiding can come from the board's IMU in flight (the rig cannot show it: the board
  sits still) or from its GPS channels (which the rig can).

## Change 2: GPS aiding and the boost loops (2026-11-09, the user's pick: "FLL on + GPS aiding")

On top of v5 (same branch, uncommitted; images and patches `fast-reacq/gnss_sdr_dev_p4_gps_aid_v*.bin`, v9 the one).
`rx_gps_accel` (receiver core, `cfg.gps_aid`, on for the board): every 10 ms a least squares over the clean GPS
channels' frequency changes over 40 ms (their loops' integrators) gives the vehicle's acceleration and the clock's
drift rate (one outlier out by leave-one-out when the fit fails); its low-pass over 50 ms above 20 m/s^2 takes the
boost profile (the core's own: 50 Hz GPS PLLs, a locked pilot's FLL on at 5 Hz) until 2 s after; in the boost each
pilot gets a.u + the drift rate as rate aiding (its rate state handing over, so nothing steps), restarts carry the fix
with it and start from the plan's line of sight. A pilot FLL always on in lock was tried and dropped: it cost weak
pilots their lock (27 dB-Hz: 78 % / 65 % locked against 97 %).

- **On the way:** v6 stopped `rx start` at boot (580 B more of the tick in internal RAM): `rx_aid_order` went to flash
  (v7, 360 B under a60b606a). v7's leave-one-out on every noisy solution made the tick late (LATE 197): only for a
  failing fit now. v8 aided with the GPS loops' frequency including the PLL's proportional step: the noise passed
  20 m/s^2 at rest, took the boost loops and kept them (noisier still): the pilots' Doppler 10-20x worse on the pad,
  BeiDou mostly out of the stream, the filter's coast 0.72 m/s. v9: the integrators, the low-passed boost decision,
  aiding only in the boost.
- **v9 on the hotshot (ra4, ra5):** Galileo lost nothing (8 of 8 through both jerks, 100 % in every phase; stock 63-87 %
  in coast, v5 lost 1-5 a run). BeiDou: the high four still drop at ignition (0 -> 73 m/s^2 arrives before the
  aiding: 16 ms to take the boost, 40 ms of window); two came back inside the boost (T+2.9 s) and rode burnout,
  two took 10-16 s; burnout lost 2 in ra4, 0 in ra5. Coast availability BeiDou 76-85 % (stock 38-50 %, v5 87-89 %),
  and BeiDou's count flickers in coast (short dips, T+10-25 s) that stock and v5 do not show: not understood yet.
- **Costs:** in the boost and its 2 s hold the wider loops' Doppler is 2-5x noisier (coast to 20 s: GPS 0.23 vs
  0.08 m/s median, Galileo 0.29 vs 0.12, BeiDou 0.77 vs 0.16); the filter's coast velocity 0.10-0.13 vs 0.07-0.09 m/s
  median, descent the same; pad and descent Doppler as v5's. The tick: LATE 51-59, as before.

## Change 3: a simulated IMU (2026-11-09, the user: "in the real system we would be feeding in the IMU acceleration")

The board sits still on the rig, so its own IMU cannot aid the flight. `rx imusim` (rx_imusim.c, in flash) stands in:
`host/imusim_profile.py SCENE.json OUT.ims` writes the scene's vertical acceleration segment by segment from ignition
(its GPS time, the pad), `sdrdev.py put` it, `rx imusim load`, `rx imusim on LATENCY_MS SCALE NOISE BIAS`. Every 2 ms,
once the receiver has GPS time, the profile's acceleration at that flight time less the latency, times the scale,
plus the bias and white noise, along the pad's up, goes to `rx_set_accel`, the core's IMU aiding: every channel, GPS
and pilots, gets it along its line of sight as feed-forward (the GPS-derived aiding steps aside; the boost loops still
come with the GPS acceleration). Board image v11 (`fast-reacq/gnss_sdr_dev_p4_imusim_v11.bin`), profile
`fast-reacq/hotshot.ims`; the profile is in PSRAM, so reload it after a reboot.

- **ri1, ri2 (5 ms late, scale 1.02, 0.3 m/s^2 noise, 0.1 m/s^2 bias):** nothing lost in either run: GPS, Galileo and
  BeiDou 100 % in boost, coast and after (stock: Galileo 63-87 %, BeiDou 38-52 % in boost and coast; v9: BeiDou lost
  at ignition). The Doppler against the truth as v5's on the pad and in descent; in the boost and its hold the boost
  loops' noise (0.23-0.32 m/s median), BeiDou in boost 0.26-0.32 m/s (v9 0.94, v5 0.59). The filter: coast 0.12-0.17,
  descent 0.05 m/s median.
- **Cost:** LATE 576-876 a run (v9 51-59): the core's IMU path works each dump's line of sight in doubles (software on
  the P4) for every channel, and the first simulator also every tick (now 500 Hz, its time in floats). 0.02 % of the
  commands; the tracking shows no harm, but a flight build should take that path to floats.
- **What it says for the real system:** the board needs an acceleration feed (the V10's IMU over J401's RX, or its
  own ISM6 with an attitude) at about this quality (milliseconds late, a few percent); with it the pilots hold.

## The baseline: the receiver without aiding (2026-11-09, the user: "closer to how a typical commercial receiver would")

`rx aid off` (board image v12 = v11 + the switch; with the receiver stopped, kept across starts, not boots) starts the
receiver with no aiding: no GPS aiding, so no boost loops, and no simulated IMU. Its loops run alone, the quiet
profile all the way. It still starts and re-acquires satellites from its own fix and ephemerides, and gets the
scene's ephemerides at its first fix (`--eph`, as every run), as a commercial receiver's own would be after its time
on the pad. Same image as the aided runs' code, same scene.

| | stock a60b606a (hb1, hb2) | no aiding (rn1, rn2) | GPS aiding (ra4, ra5) | simulated IMU (ri1, ri2) |
|---|---|---|---|---|
| satellites lost a run | 5-9 | 6-7 | 4-5 | 0 |
| back after (median) | 32.5-32.9 s | 5.1-6.6 s | 4.5-6.4 s | - |
| GPS, coast | 92.4-100 % | 92.4-99.9 % | 100 % | 100 % |
| Galileo, coast | 62.8-87.2 % | 99.9-100 % | 100 % | 100 % |
| BeiDou, boost | 52.1-99.0 % | 51.2-63.8 % | 55.7-67.7 % | 100 % |
| BeiDou, coast | 37.5-50.0 % | 85.6-86.3 % | 75.6-84.8 % | 100 % |
| range rate vs truth, 90th pct T+10-25 s (G/E/C) | - | 0.15/0.25/0.34 m/s | 0.56/0.82/1.97 (ra4) | 0.57/0.81/0.75 |
| V10 filter velocity error, mean T+14-25 s | - | 0.07-0.12 m/s | 1.08-1.37 | 1.16-1.30 |

- **Without aiding:** BeiDou's pilots drop at both jerks (ignition: back in 6-8 s; burnout: 3-4 s). At burnout the
  most stressed GPS satellite dips too (G11: 27 dB-Hz in rs1, lost in rn1 and not back: GPS has no fast
  re-acquisition, only the pilots have it). v12 without aiding matches v5 (rs1, rs2), as it should.
- **With the simulated IMU:** nothing lost, every system 100 %.
- **But the aided runs' measurements are noisier in early coast:** T+10-25 s, the range rate 3-6x noisier against the
  truth (median about 0: noise, not a bias), and the V10 filter's velocity error about 1.2 m/s instead of 0.1. GPS
  shows it under GPS aiding alone, where its channels get no feed-forward: the likely cause is the boost loop profile,
  chosen by |a| > 20 m/s^2 with a 2 s hold, which the coast's drag (|a| 12-35 m/s^2 until about T+25 s) keeps on.
  Next change for the rig: the boost loops only around the jerks.
- Figure: `fast-reacq/figs/compare_no_aiding_vs_sim_imu.png` (`fast-reacq/compare_aiding.py`; truth Doppler
  `rootcause/truth_dop.py`).

## Asks of the rig's session

1. A slot, and whether retuning one HackRF to 1575.42 MHz / 8.184 Msps disturbs the dual-band setup (its carrier
   fix, its per-boot start grid).
2. Whether the three B1C C8 files are on the rig host.
3. A free TX port, attenuators and a DC block.
4. How it wants the runs handed over (a script it runs, as its protocol says), and where the board's and the V10's
   USB go on its host, or whether the RF comes to them.
