# GNSS receiver: architecture proposal (v0)

**Date:** 2026-09-30.
**Status:** decisions recorded below. The dev board's schematic is in `hardware/gnss-sdr-dev` (PR #1561).
**Builds on:** [gnss-receiver-review.md](gnss-receiver-review.md), referred to below as "the review".

**How to read it.**
- *(estimate)*: my arithmetic.
- *(to confirm)*: needs a datasheet check. ADI's site was unreachable on 2026-09-30, so the MAX2769B sheet itself was not read.

## Decisions so far (owner, 2026-09-30)

1. **A flight receiver.**
2. **L1 multi-GNSS first, on the MAX2769B.**
3. **Room in the architecture for an L5 band later, on a MAX2771.** A couple of MAX2771s are in hand.
4. **The first board is a development board.**
5. **Size the FPGA for 32 channels.**
6. **Our own code, with Pocket SDR as the reference.** Stage 0 runs in its own session.
7. **27 MHz reference.** Every vibration-sensitive part is chosen for vibration load. The oscillator, loop-filter capacitors and mounting searches are running.
8. **L1 IF 4.092 MHz, decimated by keeping every 4th sample** (2026-10-01). See the sample-clock and L1 IF paragraphs below.

## What those decisions settle

**The correlators go in an FPGA.** This is the review's option C:
- front end → FPGA correlator bank → ESP32-P4 for everything else.

Two reasons:
- **Channel count.** L1 multi-GNSS means ~24 signals, and most of them are data + pilot pairs. Software correlation on the P4 was estimated at ~5–12 channel-equivalents at 7–8 MS/s (review §8).
- **L5 later.** L5 needs correlation at ≥ 24 MS/s against 10.23 Mcps codes, which only hardware does.

So the P4 software benchmark is no longer on the critical path.

## Requirements for version 1

| Item | Requirement |
|---|---|
| Signals | GPS L1 C/A; Galileo E1 B/C (data + pilot); BeiDou B1C (data + pilot). Later, for little extra: GPS L1C (GPS III), which uses the same Weil-code machinery as B1C; QZSS where visible. BeiDou B1I and GLONASS G1 are out: they sit off the L1 centre and beside the P4's 1560/1600 MHz harmonics |
| Channels | ≥ 24 satellite channels; each tracks a data + pilot pair where the signal has one |
| Front end | MAX2769B, low-IF complex I/Q, 4.2 MHz IF filter *(to confirm on the B)*, 2-bit I + 2-bit Q |
| Dynamics | Hold carrier-aided code tracking through the rig's boost profiles (15–20 g, ignition and burnout jerk). Unaided: frequency lock throughout, phase lock at ≥ 35 dB-Hz. Aided by the on-board IMU: phase lock down to ~30 dB-Hz (review §7) |
| Measurements | Pseudorange, Doppler, carrier phase, C/N0, lock time, per signal per epoch; prompt I/Q for debugging |
| Rates | PVT at ≥ 10 Hz to the flight computer; tracking loops at 1 kHz; raw measurements at the PVT rate or higher |
| Time | PPS output; timestamped event input |
| Gate | None built in (see the export note in review §12) |
| Aiding | From the on-board IMU; optional flight-phase and aiding messages from the flight computer |
| Size | Flight version in the Space Bug class, 35 × 35 mm, 8 layers |
| Power | GNSS RF on an LDO (owner rule); the FPGA and P4 on their own regulators |
| Environment | −40 to +85 °C; boost shock and vibration; the oscillator chosen and mounted for them |

## Block diagram (v1, with the L5 provisions dashed)

```
 L1 patch ─ ESD ─ SAW ─ LNA ─► MAX2769B ─ I1 I0 Q1 Q0 @ 27 MS/s ──────┐
 (or L1/L5 stacked patch)      ▲ SPI  │ CLKOUT 27 MHz (sample clock) ─┤
                               │      │                               ▼
 27 MHz TCXO ─► buffer ────────┼──────┼──────────────────► ┌───────────────────────┐
  (low g-sens)   │ spare out ┄┄┼┄┄┐   │                    │ FPGA                  │
                 │             │  ┊   ▼                    │  capture + time base  │
                 │             │  ┊  ADC_CLKIN ┄┄┐         │  L1 decimate          │
                 │             │  ┊              ┊         │  correlator bank      │
 ┄┄ L5 feed ┄ SAW ┄ LNA ┄► MAX2771 (low band) ┄┄ 4 bits ┄┄►│  raw-sample tap       │
    (later)                    ▲ SPI                       │  PPS out, event in    │
                               │                           └──┬─────────────┬──────┘
                               │                   QSPI (regs, dumps)   PARLIO (samples)
                               │                              ▼             ▼
                               └────────────────────── ESP32-P4 ◄── IMU (on board)
                                                         │   │
                                            UART to flight   USB-C high speed
                                            computer (PVT,   (dev: sample stream,
                                            raw, aiding)     bitstream, logs)
```

## Frequency plan

**One 27 MHz reference. Every clock that leaves a chip is 27 MHz or an integer multiple of it.**

With that rule, every harmonic on the board lands on a multiple of 27 MHz *(estimate)*:
- L1: the nearest harmonics are 1566 and 1593 MHz, 9.4 and 17.6 MHz away.
- L5: the nearest are 1161 and 1188 MHz. They are 15.5 and 11.6 MHz away, which is clear of the ±10.23 MHz main lobe and at the edge of the MAX2771's 23.4 MHz filter.

Why 27 MHz, compared with common reference frequencies:

| Reference | Nearest harmonic to L1 | Nearest harmonic to L5 | Notes |
|---|---|---|---|
| 16.368 MHz | 4.1 MHz | **2.0 MHz, in band** | MAX2769 default |
| 24 MHz | 8.6 MHz | **0.45 MHz, on L5** | Pocket SDR's known spur |
| 26 MHz | 10.6 MHz | **6.5 MHz, in band** | Fallback if 27 MHz parts are poor |
| **27 MHz** | **9.4 MHz** | **11.6 MHz** | Best common frequency ≤ 32 MHz |
| 30 / 32 MHz | 14.6 / 7.4 MHz | **6.5 / 7.6 MHz, in band** | |
| 40 MHz | 15.4 MHz | 16.5 MHz | Cleanest, and the P4's own crystal. Above the MAX2769B's 32 MHz reference limit *(to confirm; 8–32 MHz per the MAX2769-family sheets)* |

**Sample clock.** Both front ends sample at 27 MS/s. The FPGA decimates L1 by 4 to 6.75 MS/s for correlation, by keeping every 4th sample *(decided 2026-10-01)*:
- the receiver model measured a 0.04–0.05 dB loss for it, because the 4.2 MHz IF filter has already removed what lies outside the band;
- summing four samples and re-rounding to 2 bits measured about 0.5 dB, and costs adders and a requantizer.

Why run the ADC at 27 and decimate, rather than sample slower:
- A 9 MHz sample clock would put its 175th harmonic at 1575.0 MHz, 0.42 MHz from L1.
- 13.5 and 6.75 MHz clear L1 but land inside L5.
- 6.75 MS/s is 6.6 samples per chip, not commensurate with the code rate (review §4.3).

**Other clocks.**
- The FPGA fabric runs at 108 MHz (4 × 27) from its PLL.
- The P4's 40 MHz crystal and its derived clocks (360/400 MHz CPU, PSRAM, USB) put harmonics at 1560/1600 MHz and 1160/1200 MHz. Those are ≥ 15 MHz from L1 and ≥ 16 MHz from L5.
- **Any other clock on the board** (IMU SPI, UART) must keep its harmonics > 2.1 MHz from L1 (> 10.2 MHz from L5 later), or be edge-rate limited. A 10 MHz SPI clock, for example, has harmonics 4.6 MHz from L1: fine now, but it would hit L5.

**L1 IF** *(decided 2026-10-01)*. Low-IF complex I/Q with the 4.2 MHz filter, IF = **4.092 MHz**:
- the LO is fractional-N from the 27 MHz reference: LO = 27 MHz × (58 + 206921 / 2²⁰) = 1571.328052 MHz, so the IF is 4.091948 MHz, the chip's default filter centre;
- the filter passes about 2–6 MHz, which holds the L1 main lobes (about ±2 MHz) clear of DC;
- after keep-every-4th the IF sits at 4.091948 − 6.75 = **−2.658052 MHz** at the correlators. With complex samples the wrap is harmless: the 4.2 MHz band fits inside 6.75 MHz without overlapping itself;
- Holme's fs/4 lesson (review §4.3): the nearest simple ratio of 6.75 MS/s (2fs/5) is 42 kHz away, outside any flight Doppler, and the receiver's ±100 kHz sweep through its 8-sector mixer was clean;
- the sign of the IF depends on the I/Q convention and is set at first light. The IF is a register setting, so a later change needs no board change.

## Clocking and synchronization

- **TCXO requirements:**
  - 27 MHz, ≤ 0.5 ppm;
  - **g-sensitivity specified, ≤ ~0.2 ppb/g**;
  - covered against airflow;
  - mounted with its most sensitive axis across the thrust.
- **Distribution.** The TCXO feeds a low-jitter buffer with three outputs: MAX2769B XTAL, the FPGA PLL input, and a spare routed toward the L5 front-end site.
- **Sampling.** The MAX2769B samples at the undivided 27 MHz reference, and its CLKOUT is the sample clock into the FPGA. For L5, the MAX2771 takes the same clock on ADC_CLKIN, so both bands sample on one edge. The MAX2769 has no ADC-clock input, so it has to be the master (review §4.2).
- **Time base.** The FPGA keeps a 64-bit sample counter at 27 MHz. Every correlator dump, raw-sample block, PPS edge and event timestamp is stated in that counter. The P4 maps it to GPS time through the receiver clock estimate.

## Data path

- **Front end → FPGA:** 4 CMOS data lines at 27 MS/s = 108 Mbit/s, plus the clock. L5 later adds another 4 lines on the same clock.
- **FPGA → P4, raw samples.** The P4's parallel-IO receiver (PARLIO), 4–8 bits wide, carries:
  - acquisition snapshots (e.g. 20–40 ms of decimated L1);
  - in development, a continuous stream: 3.4 MB/s decimated or 13.5 MB/s full rate. The P4 relays it to the Mac over USB high speed, so the board doubles as a sampler for Pocket SDR, GNSS-SDR or our own code.
- **FPGA ↔ P4, registers.** QSPI with the P4 as master, plus one interrupt per 1 ms. Load *(estimate)*: 24 channels × ~12 accumulators × 3 bytes ≈ 0.9 kB of dumps per ms and ~0.2 kB of NCO updates, about 1 MB/s. SPI carries that easily.

## What runs where

| FPGA | ESP32-P4 | Flight computer |
|---|---|---|
| Sample capture, time base, L1 decimation | Configures the front ends over SPI; loads the FPGA at boot from its own flash (bitstream updatable with the firmware) | Consumes PVT and raw measurements |
| ≥ 24 correlator channels, each with: carrier NCO, code NCO, code generator, BOC(1,1) subcarrier; very-early/early/prompt/late/very-late taps on the pilot plus a data prompt; integrate-and-dump at the code period | Acquisition: FFT on snapshots. GPS first, then its time and position narrow the E1/B1C search, whose 4 ms and 10 ms codes are costly to search blind | Sends flight phase, and optionally its own aiding |
| Code generation: GPS/QZSS C/A by LFSR. Galileo E1B/C memory codes in block RAM, loaded by the P4. B1C (and L1C) from a shared Legendre sequence plus per-PRN phase. L5-band codes later are all LFSR-based | Loops at 1 kHz: FLL-assisted PLL and carrier-aided DLL, scheduled by flight phase, with IMU feed-forward (review §7) | |
| Raw-sample tap; PPS out; event timestamping | Bit/frame sync; LNAV, I/NAV and B-CNAV1 decoding; observables; PVT/EKF with inter-system and, later, inter-band bias states | |
| | Messages to the flight computer; USB development interface | |

The P4 has ample headroom for this *(estimate)*. 24 channels of loop updates at 1 kHz is ~10 % of one core, and PVT at 10–100 Hz is small.

## FPGA: requirements and candidates

**Requirements:**
- ≥ 24 L1 channels at 6.75 MS/s, plus room for ~12 L5 channels at 27 MS/s later.
  - Each channel is roughly 600–900 LUT4 as a parallel design *(estimate)*: two NCOs, a code generator, 12 accumulators, control. 24 channels is 15–22k LUTs.
  - A time-multiplexed design needs a fraction of that: fabric at 108 MHz runs 16 L1 channels or 4 L5 channels per physical engine, with channel state in block RAM.
- ~200–400 kbit of block RAM for Galileo codes and channel state.
- ~40 I/O: two front ends, QSPI, PARLIO, IRQ, PPS, event, configuration.
- Configurable over SPI by the P4, so no separate configuration flash.
- Industrial temperature grade, a package that fits a 35 × 35 mm board, and a toolchain you are happy to live with.

**Candidates** (DigiKey Canada stock, 2026-09-30):

| Device class | Capacity | Package | Power | Toolchain | Stock |
|---|---|---|---|---|---|
| Lattice iCE40 UltraPlus 5K | 5.3k LUT, 120 kbit block RAM + 1 Mbit SRAM, 8 DSP | 7 × 7 mm QFN48 (39 I/O) | Milliwatts | Open (yosys / nextpnr) | In stock (industrial), CA$17 |
| Lattice ECP5-25 | 24k LUT, ~1 Mbit block RAM, 28 multipliers | 14 × 14 mm BGA256 at 0.8 mm (or 10 × 10 mm BGA285 at 0.5 mm) | 0.1–0.3 W; 1.1 / 2.5 / 3.3 V | Open (yosys / nextpnr) | BGA256 commercial grade 75 pcs; industrial and BGA285: 0 |
| Others to check: Efinix Trion, Gowin (the LCSC route) | 10–20k class | Various | — | Vendor tools, free | Not checked |

**My recommendation: an ECP5-25-class device**, time-multiplexed so the first L1 design uses well under half of it and L5 fits later without a new FPGA.
- The iCE40 UP5K can do L1 multi-GNSS time-multiplexed. It is tiny, low-power and in stock, but it leaves no room for L5, and 39 I/O is tight for two front ends.
- ECP5's catch is sourcing: the industrial grade is out of stock today. This needs the same availability check we did for the front end before committing.

## L5 provisions: what we design in now

- **FPGA:** capacity for L5 (or a pin-compatible density step up in the same package) and reserved I/O for a second front end: 4 data lines, SPI chip select, shutdown, lock detect.
- **Reference:** 27 MHz TCXO, with a spare buffer output routed toward the L5 site.
- **Sample clock:** routed so the MAX2771's ADC_CLKIN can take the common 27 MHz sample clock.
- **Firmware and messages:** everything carries a band/signal ID. The navigation filter has an inter-band bias state. The front-end driver layer takes two chip types.
- **Power:** the GNSS LDO sized for a second front end and LNA, about +35 mA.
- **Antenna:** decide whether v1 flies an L1 patch or an L1/L5 stacked patch (a 25 × 25 mm class exists), so L5 later doesn't change the mechanics (question 3 below).
- **Board area:** reserving the L5 chain's area (SAW + LNA + MAX2771 + loop filter, ~10 × 15 mm) on the 35 × 35 mm flight board is optional. The architecture doesn't need it; a later layout can add it.

## Power estimate *(estimate)*

| Block | Estimate |
|---|---|
| Front end | ~20–30 mA at 3.3 V on the LDO *(to confirm for the MAX2769B)* |
| LNA and TCXO | ~5–10 mA |
| FPGA (ECP5-25 class) | ~0.1–0.3 W |
| P4 | ~0.3–0.5 W |
| **Total** | **~0.5–0.9 W** |
| Adding L5 later | +~0.1 W |

## Development plan (updated)

0. **Software receiver on the rig files. No hardware; can start now.**
   - C for the loop/nav/PVT core, which becomes the P4 firmware, with Python for plots.
   - Pocket SDR serves as the reference and cross-check.
   - Requantize the rig's 8-bit I/Q to 2-bit at 6.75 MS/s, so the software sees what the MAX2769B + FPGA will deliver.
   - Order: GPS L1 C/A to PVT, then the boost scenarios, then Galileo E1 and BeiDou B1C, then PSAS's flight IQ and the oscillator/spin injections.
1. **Correlator golden model and HDL.**
   - A bit-exact C model of the FPGA correlator, run inside the software receiver.
   - The HDL is written against it and simulated on the same files, before any FPGA board exists.
2. **Development board (recommended before the flight board).**
   - The v1 chip set on a relaxed outline, with an SMA input, USB-C, JTAG, and test points on the sample bus and clocks.
   - Sampler mode first (real sky and rig replays to the Mac), then real-time tracking.
3. **Flight board** in the Space Bug outline. The owner lays it out.
4. **L5:** second front end on a MAX2771, L5 correlators, dual-band antenna.

## Questions for the owner

1. **First board.** A development board first (recommended), or straight to the 35 × 35 mm flight outline?
2. **FPGA.** Is a BGA acceptable (ECP5-25 class, room for L5), or do you want to stay in QFN (iCE40 UP5K, L1 only, a different FPGA for L5)? Do you prefer an open toolchain?
3. **Antenna.** L1 patch now, or an L1/L5 stacked patch from the start?
4. **Code.** Our own C with Pocket SDR as the reference and cross-check (default), or build on Pocket SDR's library directly?
5. **Reference frequency.** Is 27 MHz acceptable, pending a search for a low-g-sensitivity part? The fallback is 26 MHz, which leaves a spur inside L5.
