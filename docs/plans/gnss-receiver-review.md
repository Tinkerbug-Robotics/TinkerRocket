# A GNSS receiver from scratch: review and options

**Date:** 2026-09-30.
**Status:** review for discussion; nothing is decided.
**Scope:** everything from the RF front end through the tracking loops.
- Published designs and what each one teaches.
- Front-end chips and whether they can be bought today.
- What a rocket boost asks of the tracking loops.
- The architecture options for a receiver of our own, and an order of work.

**How to read it.** Facts were read on 2026-09-30 from each project's own repository, paper or datasheet. Other claims are tagged:
- *(secondary)*: from a search result or a third party.
- *(estimate)*: my arithmetic.

Sources are listed at the end.

## 1. Summary

**This is well-trodden ground up to one point.** Every design follows the same chain:
1. A front-end IC turns L-band into 1–3-bit samples.
2. Correlators in an FPGA or in tight software strip off carrier and code at the sample rate.
3. C code closes the loops at 1 kHz, decodes the navigation message and solves PVT.

Open code exists for every block. What nobody has published is an open, real-time receiver on a small MCU or FPGA that is built to fly. That part would be ours.

**The rocket flight record is consistent.** Generic receivers drop satellites at ignition, burnout and separation. The receivers that held on had four things in common:
- wide or FLL-assisted carrier loops;
- trajectory or IMU aiding;
- bandwidth scheduled by flight phase;
- attention to the crystal oscillator.

Oscillator shock or vibration is named as a cause in the flights of 2001, 2014 and 2024 (§5.3).

**Findings that shape the design**

- **Front end.** The MAX2771 is the front end in the current designs: Pocket SDR, jmfriedt, the GMV launcher receiver and GNSS-SDR's FPGA path (§4).
  - **It is hard to buy today:**
    - DigiKey: 0 in stock, backorders refused.
    - LCSC: tube stock sold out.
    - Mouser reportedly will not sell it, and ADI direct quotes 22–23 weeks *(secondary)*.
  - **The L1-only MAX2769B is in stock** and has the most flight and open-hardware history.
  - **The alternatives fail:**
    - NT1065: sanctions exposure.
    - SE4150L: obsolete.
    - SDR transceiver chips: too much power and area.
    - Direct RF sampling: impractical at our size.
- **Samplers.** Pocket SDR (BSD-2) is the reference design: hardware, a library of MAX2771 register settings, and the best software to learn from. jmfriedt's board is a cheap GPL-3.0 derivative with an honest record of the pitfalls (§5.1).
- **Real time.** Most real-time receivers split the work: an FPGA does the correlation and a CPU does everything else (Holme, KiwiSDR, Piksi, DLR Kodiak).
  - CPU-only also works at modest sample rates. An ESA-funded launcher receiver runs 12 GPS + Galileo channels at 8 MS/s, 1-bit, on a dual-core ARM at 72 % load.
  - **Whether an ESP32-P4 can do the same is the deciding unknown.** My estimate is about a dozen L1 C/A channels at ~4 MS/s. One benchmark on an existing board would settle it (§8).
- **Dynamics.** Our bench losses (2.6–6.4 g line of sight) match the "about 4 g" that a 2006 sounding-rocket paper attributes to COTS loop settings.
  - A frequency-locked loop holds through a 15–20 g boost.
  - A phase-locked loop holds only in one of two ways:
    - wide (~40 Hz) with a strong signal (≥ 35 dB-Hz);
    - narrow, with IMU aiding.
  - Oscillator g-sensitivity is part of the budget, not a footnote (§7).
- **Test path.** We already own most of what testing needs (§9):
  - the rig's IQ files with truth;
  - a public raw-IQ recording of a real rocket boost;
  - the HackRF for real-sky recording and conducted replay.

**Suggested order** (§10):
1. Build the receiver in software first, against the rig's files.
2. Benchmark the P4.
3. Build one front-end + P4 board on the Space Bug outline. It works as a USB sampler first and as the embedded receiver later.
4. Choose between P4-only and adding an FPGA from the benchmark.
5. Add IMU aiding last.

The decisions needed from you are in §11.

## 2. Why build one: what the bench already shows

- **Export gate.** Every receiver we tested that has one withholds the fix at 514–516 m/s or 80 km.
- **Boost losses.** The PX1105R loses half its satellites at 134–327 Hz/s of line-of-sight Doppler rate (2.6–6.4 g) at 35–48 dB-Hz. Part of that is its own lagging navigation solution dragging the loops.
- **Raw data.** Raw measurements come from only some parts, at limited rates and with caveats.

A receiver of our own would change each of these:
- **No gate.** It has no export gate unless we add one (see the note in §12).
- **Loops.** Its loop order, bandwidth and schedule can be designed for the boost.
- **Aiding.** It can take the flight computer's IMU as an input.
- **Output.** It can emit correlator-level data (prompt I/Q, C/N0, lock state) as well as pseudorange, Doppler and carrier phase, at whatever rate the link carries.

**What a first flight unit would have to do.** These are stated as requirements so they can be checked:
- **Signals.** GPS L1 C/A. Galileo E1 and BeiDou B1C share the L1 centre frequency and add satellites for little RF cost.
- **Tracking.** Carrier-aided code tracking that holds through a 15–20 g boost, including the jerk at ignition and burnout.
- **Outputs.** PVT at ≥ 10 Hz, plus raw measurements.
- **Aiding.** An input from the flight computer.
- **Size and power.** The Space Bug class: 35 × 35 mm, GNSS supplied from an LDO.

## 3. What a receiver is made of

| Block | Job | Where it can live |
|---|---|---|
| Antenna, LNA, SAW | Gain and out-of-band rejection. On our boards the threats are LoRa, the P4's own clock harmonics and cellular | Board. The Space Bug already has this chain |
| RF front end | Down-conversion, IF filter, AGC, 1–3-bit ADC | Front-end IC |
| Reference oscillator | One TCXO for the LO and the sample clock. Its stability, phase noise and g-sensitivity set the floor for the carrier loop | Board |
| Sample transport | Tens of Mbit/s of 2–4-bit samples to wherever correlation happens | USB bridge, MCU parallel port or FPGA |
| Acquisition | 2-D search over code phase × Doppler per satellite; FFT-based is standard | CPU (PC or MCU) or FPGA |
| Tracking channels | Carrier and code NCOs, replica generation, early/prompt/late correlators, 1 ms or longer accumulations | FPGA or SIMD software |
| Loop closure | Discriminators, FLL/PLL/DLL filters, lock detectors, C/N0 estimate, bit and frame sync, all at 1 kHz per channel | CPU |
| Navigation data | Decode GPS LNAV, Galileo I/NAV and BDS B-CNAV1; ephemeris; time of week | CPU |
| Observables and PVT | Pseudorange, Doppler, carrier phase; least squares or EKF with clock, iono/tropo and Sagnac terms | CPU. Our tightly coupled EKF work (#1523) already takes pseudorange, Doppler and carrier delta-range |
| Aiding (later) | Line-of-sight dynamics projected from the IMU and fed into the NCOs | CPU, sharing the flight computer's IMU |

## 4. The front end

### 4.1 MAX2771: the datasheet (Rev 2, 4/25) and field experience

**Bands.** It has two LNA inputs and one synthesizer.
- The LO tunes 1525–1610 MHz for L1, E1, B1 and G1, or 1160–1290 MHz for L2, L5, E5a/b, E6, B2 and B3.
- **One chip receives one band at a time.** L1 + L5 therefore needs two chips on one reference, which is how the multi-channel Pocket SDR boards are built.

**Noise and gain.**
- Cascaded NF is 1.4 dB at L1 and 1.6 dB at L2/L5, using the internal LNA (0.9 dB NF, 18 dB gain).
- LNA-out and mixer-in are separate pins, so a SAW can sit between them. The datasheet puts the cost of a 1 dB SAW there at about 0.15 dB of NF.
- The mixer-input NF is 10.3 dB. An external LNA that bypasses the internal one therefore needs about 20 dB of gain.
- Up to 96 dB voltage gain; 59 dB of PGA range; 25 dB of image rejection.
- Mixer in-band input P1dB is −85 dBm and out-of-band IIP3 is −9 dBm.

**IF filter.**
- Complex band-pass at 2.5, 4.2 or 8.7 MHz (centre ≤ 9 MHz, 3rd- or 5th-order).
- Low-pass for zero-IF at 16.4, 23.4 or 36 MHz two-sided. The 36 MHz setting takes GPS and GLONASS L1 together.

**ADC and data out.**
- 1 or 2 bits on each of I and Q, or up to 3 bits on I alone. Sample clock up to 44 MHz.
- On-chip AGC works by servoing the density of magnitude bits.
- The samples come out in parallel on four pins plus a clock. A serial "DSP interface" also exists, but it needs a clock 2–4 times the sample rate.
- The ADC can be bypassed to bring analog I/Q out.

**Clock.**
- Reference 8–44 MHz.
- Fractional-N synthesizer: LO within about ±30 Hz for a reference ≤ 32 MHz. Integer-N also works; 1.023 MHz is the convenient comparison frequency.
- A fractional divider produces the ADC clock.
- An ADC-clock input keeps several chips in lock-step.

**Control, power, package.**
- Configured over 3-wire SPI in 48-bit transactions. Two reserved fields must be rewritten after every power-up.
- 2.7–3.3 V; 26–27 mA running, 200 µA in shutdown; −40 to +85 °C.
- 5 × 5 mm 28-pin TQFN. There is no antenna-bias pin, so a bias tee is built externally if one is needed.

**The EV kit does not stream samples.** Its MCU only bridges SPI to a Windows GUI. The ADC bits come out on a header. It carries a 16.368 MHz TCXO and unpopulated SAW sites between LNA and mixer. ADI confirmed to jmfriedt that it cannot stream IQ. It costs about $570–620.

**Lessons from jmfriedt's and Pocket SDR's issue trackers:**
- **Q sign.** The MAX2771 outputs I−jQ. If Q is not negated, Doppler flips sign, which in GNSS-SDR showed up as velocities wrong by about 1 km/s.
- **LNA compression.** The internal LNA goes non-linear above about −83 dBm. Keep HackRF injection well below that.
- **Two chips.** Their relative carrier phase is random after power-up or a band change, holds within a band, and drifts during warm-up.
- **Sample clock.** Use ADC rates that are integer divisions of the TCXO, e.g. 24 MHz / n. The fractional divider adds jitter because the ADC clock has no PLL. The FX2LP is unreliable above 32 MS/s.
- **TCXO harmonics show up in band.** With 24 MHz: 24 × 49 = 1176 MHz sits on L5 and 24 × 65 = 1560 MHz sits beside B1I. Our P4's 40 MHz harmonics at 1560 and 1600 MHz are the same kind of problem.
- **Antenna feed.** A DC-passing splitter shorts the antenna feed unless DC blocks are fitted.

### 4.2 Alternatives and whether they can be bought (checked 2026-09-30)

| Front end | What it is | Availability | Verdict |
|---|---|---|---|
| **MAX2771** | Multi-band, one band per chip, 1–3-bit ADC at up to 44 MS/s | Production, but DigiKey has 0 and refuses backorders ("temporary constrained supply"). LCSC tube stock 0. These *(secondary)*: 6 LCSC reel pieces at ~$33; Mouser "restricted availability", ECCN 7A994; ADI list $6.36 at 1k, 22–23 weeks; brokers ~$17–22 | Best part. Sourcing is the risk |
| **MAX2769B** | L1 only, same architecture, AEC-Q100 | DigiKey 31 in stock + 780 at the factory (10 weeks); Mouser 110 and LCSC 86 per the review | In stock. The most-flown open-hardware front end: Piksi, KiwiSDR v2, PSAS, OreSat |
| MAX2769 / MAX2769C | L1 only | Orderable in small numbers; the C is discontinued at DigiKey | Fallback |
| NT1065 / NT1066 (NTLab) | Four channels, multi-band, 2-bit ADC up to 99 MS/s | No franchised distributor. Design office in Minsk; sales through a Vilnius entity; its web shop returns 404. Amungo's boards are discontinued | Not an option: sanctions exposure and no supply |
| SE4150L / SE4110L | L1 only | Obsolete | No |
| Chinese GNSS RF front ends | Exist | No datasheets or prices online | No |
| AD9361/63/64, LMS7002M, ADRV9002 | General transceivers, 12-bit | $124–476, BGA, ~0.35 W per receive chain, needs an FPGA and still an external LNA/SAW | Wrong size class |
| Direct RF sampling | ADC samples L1 directly | GSPS JESD204 ADCs at 1.6 W and $440+; a jitter budget of ~1.7 ps; noise folding. Monta considered it for the Firehose and chose a tuner | No |

If the MAX2771 can't be had in time, an L1-only first board on the MAX2769B loses nothing for GPS L1 C/A, Galileo E1 or BeiDou B1C. Only L5 waits.

**MAX2771ETI+T** is the same part as MAX2771ETI+, supplied on tape and reel instead of in tubes: same die, footprint and datasheet. It does not get around the shortage. DigiKey sells it as cut tape from one piece but has 0, with 2,500 due 2027-03-04 and a 23-week lead time. Broker listings that claim reel stock carry counterfeit risk.

**L1 on a MAX2769B, L5 on a MAX2771.** Nothing rules this out. It halves the MAX2771 count, and NSL Stereo already paired a MAX2769B with a different front end on one TCXO. What it costs:

- **Clocking.**
  - One TCXO feeds both chips.
  - The MAX2769 has no external ADC-clock input; its EV kit shows only the reference input and CLKOUT. For common sampling the MAX2769 must be the master, with its CLKOUT driving the MAX2771's ADC_CLKIN, and both bands then sample at L5's rate.
  - The alternative is two sample-clock domains off the one TCXO. The timing offset between them is constant per power-up, and the position solution absorbs it as a receiver bias; it drops out of the iono-free combination too.
- **Frequency plan.** Reference harmonics must miss both bands:
  - 24 MHz puts 1176.0 MHz on L5 (Pocket SDR's known spur).
  - 16.368 MHz puts 1178.5 MHz inside L5 and 1571.3 MHz beside L1.
  - 27 MHz is clear of the whole L5 main lobe and of the L1 main lobes; its nearest harmonic is 1566 MHz *(estimate; check each chip's reference range)*.
- **Two drivers.** The chips have different register maps, SPI formats and AGCs. The group delays also differ, and the LO phase is random after each power-up. That is a constant inter-band bias, the same as with two MAX2771s.
- **The bigger commitment is L5 itself.**
  - It needs its own chain: a dual-band antenna with a diplexer or separate feeds, plus an L5 SAW and LNA.
  - L5 is ~20 MHz wide, so it needs ≥ ~24 MS/s and 10.23 Mcps correlation. That puts it in FPGA territory (option C, §8).
  - In return:
    - 1 ms pilot codes (L5Q, E5aQ, B2a pilot) for a pure PLL at 1 ms;
    - 25 % less Doppler per g;
    - 10 × the code rate;
    - an iono-free combination;
    - a band away from the P4's 1560/1600 MHz harmonics.
  - GPS L5 is still flagged unhealthy, but Galileo E5a and BeiDou B2a are usable.

### 4.3 Around the chip

- **Ahead of the chip: LNA and SAW.** No Pocket SDR or jmfriedt board has anything ahead of the MAX2771, because they assume an active antenna. The flight designs added filtering and gain: PSAS put a SAW between LNA and mixer to keep its own Wi-Fi out, and GMV tested its receiver behind an added LNA. Our boards carry LoRa and a P4, so keep an L1 LNA + SAW chain like the Space Bug's ahead of the chip. Alternatively, use the chip's own LNA with a SAW in its LNA→mixer gap.
- **Reference oscillator requirements:**
  - frequency: harmonics clear of the IF passband and of any band in use, and the sample clock an integer division of it;
  - stability ≤ 0.5 ppm;
  - **g-sensitivity specified, ≤ ~0.2 ppb/g** (§7);
  - shielded from airflow;
  - mounted with its sensitive axis across the thrust.

  Typical quartz is 0.1–1 ppb/g, with 0.25–4 ppb/g quoted in one survey. MEMS-resonator TCXOs are marketed well below 0.1 ppb/g (vendor claim). Frequencies in use today: 16.368 MHz (EV kit, Piksi, KiwiSDR), 24 MHz (Pocket SDR, jmfriedt), 38.88 MHz (Firehose).
- **Sample rate.** Prefer a rate that is not a small-integer multiple of the 1.023 MHz chip rate; 4.092 MS/s, for example, is exactly 4 samples per chip. A commensurate rate quantizes code phase, and when code Doppler is small, as in a static test, that can bias the DLL. Pocket SDR's 24 MHz / n rates avoid it.
- **IF placement.** Holme's fs/4 IF put NCO spurs on his carriers, and moving it by 100 kHz fixed that. In zero-IF, put the DC spur where no signal is.

## 5. Projects reviewed

### 5.1 Samplers: front end to a PC

| Project | Front end and clock | Transport | Processing | Licence, state |
|---|---|---|---|---|
| **jmfriedt/max2771_fx2lp** | Two MAX2771 on a 4-layer daughterboard (~31 × 34 mm). Internal LNA only, no SAW. 24 MHz with an external-reference input. 5 V antenna bias | Plugs onto a cheap FX2LP breakout board. SDCC firmware that speaks Pocket SDR's protocol, so no Keil needed. 8–16 MS/s loss-free | Pocket SDR tools to capture; GNSS-SDR, GNU Radio or Octave to process. Shown: GPS L1 C/A PVT, Galileo E1. L5 "always challenging" | GPL-3.0, active (2026-02) |
| **Pocket SDR FE** (T. Takasu) | 2 / 4 / 8 × MAX2771. 24 MHz 0.5 ppm TCXO; one chip's ADC clock goes to all. No LNA or SAW (active antenna assumed) | FX2LP, 2CH, ≤ 32 MS/s; FX3 USB 3, 4CH/8CH, ≤ 48 MS/s (above the datasheet's 44) | Pocket SDR (§6). Parts ~$60 (2CH), ~$130 (4CH); sold assembled by a third party at $279 (4CH, in stock) and $668 (8CH, pre-order) | BSD-2 including hardware; v0.20, 2026-08. Firmware needs Windows-only Keil / Cypress tools |
| **GNSS Firehose** (P. Monta; your download) | 3 × wideband satellite-TV tuner (now NRND) + dual 8-bit ADC at 69.984 MS/s. One 38.88 MHz TCXO for all. LNA with a current-limited bias tee; no SAW | Spartan-6 LX9 with a soft RISC-V; raw frames over gigabit Ethernet (~840 Mbit/s) | Host software receivers. 4-layer, 118 × 99 mm, 5 V 2 A | TAPR OHL + GPLv2. Last push 2024; sold by a reseller |
| SiGe GN3S v2/v3 | SE4120L, 16.368 MS/s 2-bit | FX2 | GNSS-SDR, SoftGNSS | Retired |
| NSL Stereo | MAX2769B + wideband tuner, Spartan-6 | USB 2 | GNSS-SDRLIB | Closed |

What the Firehose teaches:
- One reference for everything.
- Dithered 2-bit samples with histogram AGC are enough.
- A 3D-printed cap over the TCXO "dramatically" improved the clock by keeping air currents off it.
- Current-limit the antenna feed.
- A 10 MHz/PPS input is kept for timing checks.
- Obsolescence bites: the PHY and the tuner both went.

**Off-the-shelf SDRs.** These are recording and reference tools, not the design:

| SDR | Samples | Clock | Notes |
|---|---|---|---|
| HackRF One (we have it) | 8-bit, 2–20 MS/s | ~20 ppm crystal *(secondary)*, 10 MHz clock input | Records real sky through an active antenna |
| RTL-SDR V4 | 8-bit, ~2.4 MS/s | 1 ppm TCXO | L1 C/A main lobe only |
| ADALM-Pluto | 12-bit, ~4–6 MS/s over USB 2 | — | |
| bladeRF 2.0 micro, LimeSDR Mini 2.0, USRP B2xx | 12-bit, up to 56 MHz | — | $540–2400 |

Pocket SDR and GNSS-SDR support all of them.

### 5.2 Real-time receivers

| Project | Front end | Correlation | Loops, nav, PVT | Channels and notes | Licence, state |
|---|---|---|---|---|---|
| **Andrew Holme** | All discrete: MMIC, mixer, a fractional-N LO whose loop is closed in the FPGA, 22.6 MHz IF, comparator as a 1-bit ADC | Spartan-3: NCOs, C/A generators, E/P/L integrate-and-dump at 1 ms | Soft CPU in the FPGA runs the loops. A Raspberry Pi does FFT acquisition (4 ms, 250 Hz bins), ephemeris and PVT | 12 channels = 67 % of an XC3S400. Handover trap: an FFT bin can be 250 Hz off, outside Costas pull-in, so he recomputes carrier frequency from the locked code NCO | GPL |
| **KiwiSDR GPS** | SE4150L (v1), MAX2769B (v2); 16.368 MS/s, 1-bit | Artix-7 A35; each extra channel costs 2 DSP slices, no block RAM | Soft CPU loops; the ARM does FFTW acquisition, nav and Kalman PVT. Galileo E1B and QZSS added | 12 channels. Uses Xilinx primitives | GPL-3.0+, active |
| **Swift Piksi v1/v2** | MAX2769 + SAW, 16.368 MHz, 1-bit at 16.368 MS/s | Spartan-6 LX9 "NAP": acquisition RAM + 12 tracking channels | STM32F4 under ChibiOS. Profiles: 1 ms, 10 Hz PLL + FLL aid, 1 Hz carrier-aided DLL; then 5 ms at 50 Hz after bit sync. "FAST" profiles at 40–100 Hz | New NCO rates take effect only after the next dump, a pipelining trap every FPGA design meets | Firmware GPL-3.0 (archived); hardware CC BY-SA; NAP HDL closed |
| Namuru (UNSW) | GP2015 (obsolete) | Cyclone II, GP2021-style Verilog | NIOS II | 12 channels | LGPL-2.1 per the Verilog headers |
| GNSS-SDR FPGA (CTTC) | AD9361, or the MAX2771 EV kit | Zynq: FFT acquisition + multicorrelator tracking in HDL | ARM/Linux | 40 signals in real time, 5.4–6.5 W | Host GPL-3.0; the HDL is sold commercially |
| **DLR Kodiak** | Plug-in front-end modules; 10 MHz TCXO or external OCXO/CSAC | Cyclone V SoC FPGA: 32 channels (pilot + data), 256 correlators | Dual ARM. Loop bandwidth widened for boost and re-entry; loops aided with Doppler from the IMU solution | GPS L1 C/A + Galileo E1, 4.5 W, cold TTFF 34 s | Closed |
| **GMV GNSSW-MLMSC** (ESA, microlaunchers) | **One MAX2771**, 16 MHz, **8 MS/s, 1-bit I**, 4.2 MHz bandwidth | **None: everything on the CPU** of a dual-core Zynq-7030 under RTEMS | FFT acquisition, tracking and PVT on the CPU | **12 GPS + Galileo channels at ~72 % CPU.** Launcher simulation: mean position error ~15 m, velocity error < 0.15 m/s. Passed vibration; radiated emissions needed filtering on the communication lines | Closed |
| iliasam STM32F4 / ESP32 | MAX2769, 1-bit at 16.368 MS/s into an SPI slave (the chip's clock drives SCK) with circular DMA | Bit-packed XOR + popcount table: 450 µs of CPU per channel per 1 ms | Same MCU | ~2 continuous channels at 168 MHz, so 4 are time-shared; ~60 s per acquisition | Labelled MIT but carries GPL-2 code |
| Cornell (2006) | GP2015, 2-bit at 5.7 MS/s | Bit-wise parallel correlation on a 720 MHz DSP | Same DSP | Equivalent of 43 L1 C/A channels, tracking to 25 dB-Hz | Paper |
| **PSAS jGPS v3** (flew 2015) | Passive antenna → SAW → LNA/SAW module → MAX2769B; 4.092 MS/s zero-IF 2-bit | CPLD → STM32F4 → Ethernet (recording only) | Post-processing | **Raw IQ from T−32 s to T+34.7 s is public.** Their software never tracked it; the COTS receiver alongside was "pretty much garbage" | BSD-3 board, GPL software |

**Open FPGA building blocks.** We found no complete open real-time tracker for an iCE40, ECP5 or Gowin part. The pieces that exist:
- **osqzss/gps-fpga** (MIT, 2026): a one-channel L1 C/A correlator. Its author is T. Ebinuma, who wrote gps-sdr-sim and the 2006 sounding-rocket receiver paper below.
- **j-core/gnss-baseband** (BSD-style, VHDL): up to 7 time-sliced channels.
- **JuliaGNSS/gnss-m2sdr** (BSD-2, LiteX on Artix-7, active): a multi-channel bank with BOC support.
- Resource guide: KiwiSDR spends 2 DSP slices per channel.

### 5.3 Receivers that flew on rockets

| Flight | Dynamics | What happened | Lesson |
|---|---|---|---|
| Maxus-4, 2001: DLR-modified Mitel Orion | 17–18 g for 6 s, 3.8 Hz spin, 1100 m/s, 81 km | The Orion fell 9 → 5 satellites and the NASA reference receiver 8 → 6; both were full again in 8–9 s. The unmodified receiver on the same chipset lost everything. **The Orion's oscillator spiked at ignition and settled ~100 Hz lower at L1** | Trajectory aiding solved reacquisition; the crystal was stressed at high jerk |
| Ebinuma & Nakasuka 2006: GP4020 running GPL-GPS, built for the PSAS rocket | Simulated 12 g peak, 40 g/s jerk, −75 g/s at burnout | Held lock; 3 Hz fixes, 8 m (1σ) | Decode NAV with a PLL on the pad, then track with a 2nd-order FLL in flight. COTS loop settings cap out near 4 g |
| DLR Phoenix: GP4020, flying since 2004 | Rockets, launchers, LEO | Wide 3rd-order PLL with FLL assist and a narrow carrier-aided DLL *(secondary)*; trajectory aiding | Unrestricted units need a German export licence |
| Ariane 5 VA219, 2014 (OCAM-G experiment) | Booster separation | A generic receiver lost all 22 signals and took ~6 s to recover. Phoenix-S with launcher loops lost 3 of 9 and kept navigating | Pyro shock on the crystal oscillators |
| PSAS LV2, 2015 | Amateur | COTS output unusable | The recorded IQ is a free test vector |
| PMWE2, 2021: DLR Kodiak | Sounding rocket | Continuous except a brief loss of some high-elevation satellites at burnout | Phase-scheduled bandwidth and IMU aiding work |
| MAPHEUS-15, 2024: commercial dual-band receiver in fixed-loop mode (PLL 17 Hz, DLL 0.1 Hz, 1 ms) | 309 km, roll up to 2000 °/s | Lost everything at tower exit; navigating again by T+4.5 s. Vibration on the LO raised the noise. A second commercial receiver never reacquired | Generic loops plus vibration on the reference |

## 6. Software and HDL we could reuse

The repository is GPL-3.0 (software) and CERN-OHL-S (hardware). The CLA keeps relicensing open, so what we copy matters.
- **Permissive code (BSD/MIT)** keeps every option open.
- **GPL code** is compatible with our GPL-3.0 repository. But files copied from it stay GPL, which closes off relicensing of those files.

| Code | Licence | What it is | Use to us |
|---|---|---|---|
| **Pocket SDR** software | BSD-2 | C/C++ + Python. Acquisition, tracking, nav, PVT (RTKLIB). Very wide signal set including pilots. Reads HackRF-style interleaved int8 I/Q. Real time on a Raspberry Pi 5: 10 channels at 6 MS/s on ~1.2 cores | **Best code to learn from and lift.** Its tracking defaults (5 Hz PLL, FLL 5 → 2 Hz, 20 ms) suit a static receiver, not a boost. Build without FFTW, which would make it GPL. The README adds an export notice outside the licence: research/educational use, and a prohibition on military, WMD or delivery-system development use |
| GNSS-SDR | GPL-3.0+ | C++ on GNU Radio; the widest signal set; reads `ibyte` (HackRF) files. `gnss-sdr-vtl` is a vector-tracking variant built for a spinning rocket | Reference receiver on the Mac. Too heavy to lift from |
| SoftGNSS (Borre et al.) | GPL-2.0+ | MATLAB, L1 C/A, written alongside the textbook | Teaching |
| FGI-GSRx | GPL-3.0 | MATLAB, multi-GNSS, alongside the 2022 Borre book | Teaching |
| GNSS-DSP-tools (Monta) | MIT | Python code generators and trackers for nearly every signal | Code tables and cross-checks |
| JuliaGNSS; gnss-rcv (Rust) | MIT | Modern receivers | Prototyping |
| KiwiSDR / Holme | GPL-3.0+ | C++ + Verilog, FPGA/CPU split | Architecture reference |
| libswiftnav-legacy | LGPL-3.0 | Piksi loop filters | Reference |
| osqzss/gps-fpga, j-core, gnss-m2sdr | MIT / BSD | Correlator HDL | Starting points for option C |

## 7. Tracking through a boost

**Units.**
- 1 g of line-of-sight acceleration = **51.5 Hz/s** of Doppler rate at L1 (38.5 at L5).
- 1 g/s of line-of-sight jerk = 51.5 Hz/s².

**Unaided carrier loops.**
- A 3rd-order PLL tracks constant acceleration with no steady error, so jerk sets its limit: θe = 0.4828·J/Bn³.
- Thermal jitter: σ = √(Bn/(C/N0)·(1 + 1/(2T·C/N0))).
- The rule is jitter + θe/3 ≤ 15° for a Costas loop (data channels), or 30° for a pure PLL on a pilot channel.
- Largest sustained jerk allowed, Costas, T = 1 ms *(estimate)*:

| PLL Bn | 30 dB-Hz | 35 dB-Hz | 40 dB-Hz | 45 dB-Hz |
|---|---|---|---|---|
| 10 Hz | 3 g/s | 4 g/s | 4 g/s | 5 g/s |
| 15 Hz | 7 g/s | 12 g/s | 14 g/s | 16 g/s |
| 25 Hz | 20 g/s | 50 g/s | 63 g/s | 70 g/s |
| 40 Hz | 21 g/s | 173 g/s | 242 g/s | 278 g/s |
| 60 Hz | — | 470 g/s | 756 g/s | 903 g/s |

- A 2nd-order FLL at Bn = 10 Hz and T = 1 ms:
  - its error at 150 g/s is 22 Hz, against a 250 Hz tracking limit (1/4T);
  - its jitter at 30 dB-Hz is 45 Hz.
- **So frequency lock survives the boost. Phase lock needs a wide loop and a strong signal, or aiding.**
- Loops must be designed in discrete time once Bn·T is not small.

**What the working rocket receivers did:**
- FLL-assisted 3rd-order PLL at 1 ms, wide during powered flight (Phoenix). For scale, Piksi's high-dynamics profiles run 40–100 Hz.
- An FLL through the boost: Ebinuma.
- Bandwidth scheduled by flight phase: Kodiak.
- Trajectory aiding (Orion, Phoenix) or IMU Doppler aiding (Kodiak) for tracking and fast reacquisition.
- A carrier-aided DLL near 0.1–1 Hz.

The closest templates in the recent literature:
- A 3-state Kalman tracking loop for launchers: 5 ms, holding to 28 dB-Hz through a 5 g step and 30.7 dB-Hz through a 20 g step, real time on a LEON3 with FPGA correlators (Llorente et al. 2026).
- A vector tracking loop that held at 50 g and 50 g/s where a scalar 18 Hz 3rd-order PLL lost lock (Mu & Long 2021).
- MEMS-IMU-aided PLLs holding phase at 100 g with a 20 Hz loop (Ban et al. 2017).

With aiding, the receiver clock (including vibration) becomes the limit (Chiou 2005).

**The oscillator.** A quartz reference moves by Γ·a:

| Γ (per g) | Shift for a 15 g step at L1 |
|---|---|
| 1 ppb/g | 24 Hz |
| 0.5 ppb/g | 12 Hz |
| 0.2 ppb/g | 5 Hz |
| 0.1 ppb/g | 2 Hz |

- **The step is common to every channel,** so a shared clock state (vector tracking) absorbs it. Per-channel loops each see it as dynamics.
- **Random vibration adds phase noise the PLL cannot track** *(estimate)*: integrated over 20–2000 Hz, 1 ppb/g at 0.04 g²/Hz (8.9 grms) costs 4.0° of the 15° budget, and 6.4° at 0.1 g²/Hz. At 0.2 ppb/g the same vibration costs 0.8° and 1.3°.
- **The flights confirm it:** oscillator spikes and a ~100 Hz offset at ignition (2001), pyro shock (2014), LO vibration (2024).
- **The bench cannot show any of this.** IQ replay never accelerates the receiver's own oscillator.

**Spin and the antenna** *(estimate)*.
- **Off-axis antenna:** 27 mm off the spin axis (the skin of a 54 mm airframe) at 8 Hz roll swings the carrier ±51° × sin(angle between line of sight and axis). That is ±7 Hz of Doppler and about 350 g/s of peak jerk.
- **Side-mounted blades:** a spinning rocket with blade antennas saw fades of up to 20 dB; DLR switched antennas using a gyro.
- **Wind-up:** an antenna spinning about its own boresight adds one cycle per revolution, common to all channels.
- **Either way, the gyro can feed this term forward.**

**Pilot signals.** Galileo E1C, the BeiDou B1C pilot, GPS L1CP and L5Q carry no data bits, which buys three things:
- a pure PLL can use the 30° threshold, up to 6 dB better than Costas;
- the FLL gets its full ±1/(2T) pull-in;
- no bit-sync wait.

The catch on L1 is that pilot codes are 4–10 ms long, so a 1 ms boost loop means partial-code correlation. E1C, at 4 ms, is the practical L1 pilot. L5Q has 1 ms codes but needs an L5 chain.

## 8. Architecture options

|   | A. Sampler → PC | B. Front end → P4, software correlators | C. Front end → small FPGA → P4 | D. SDR SoC (Zynq + transceiver) | E. Discrete RF |
|---|---|---|---|---|---|
| Precedent | Pocket SDR, jmfriedt | GMV (dual ARM), iliasam (STM32) | Piksi, KiwiSDR, Holme, Kodiak | GNSS-SDR FPGA | Holme |
| Flyable | No | Yes | Yes | No: 5–6.5 W | Possible, not sensible |
| Channels | PC-limited | ~5–25 at L1 *(estimate below)* | 12–48, by FPGA size | 40+ | — |
| Signals | Anything | L1: GPS C/A, then Galileo E1; not L5 | L1 multi-GNSS; L5 later with a second chip | All | L1 C/A |
| New work | Board + capture firmware | Board + P4 DSP code | Board + HDL + P4 code | Integration | RF design |
| Starting code | Pocket SDR end to end | Pocket SDR loops and nav; the inner loop is ours | MIT/BSD correlator cores, the rest as B | Host GPL; HDL closed | GPL |
| Main risk | None (proven) | CPU budget, capture rate, P4 harmonics near L1 | HDL verification effort, area, FPGA power | Size class | NF, effort |

**Can a P4 correlate in software?** This is B's deciding unknown. The table scales measured costs from other CPUs; nothing here was measured on a P4. It assumes two cores at 360–400 MHz with 60 % of their time given to correlation. Channel counts *(estimate)*:

| Cost basis (cycles per sample per channel) | 4.092 MS/s | 8.184 MS/s | 16.368 MS/s |
|---|---|---|---|
| Bit-packed 1-bit XOR/popcount: iliasam, measured 4.6 | ~24 | ~12 | ~6 |
| GMV's dual-ARM receiver, ~12 including its navigation | ~9 | ~5 | ~2 |
| 16-bit SIMD at ESP-DSP's P4 dot-product rate, ~13 | ~9 | ~4 | ~2 |

A dozen GPS L1 C/A channels at ~4 MS/s is a fair planning number. Galileo E1 needs ≥ 4 MHz of bandwidth and costs more per channel.

**A benchmark on an existing P4 board, with no RF hardware, settles it.** It would measure:
- the inner correlator loop, in cycles per sample;
- parallel capture at the front end's clock rate (the IDF docs don't state PARLIO's maximum external-clock rate);
- sustained USB high-speed bulk throughput.

**Why the P4 is attractive as the bridge.** It has USB 2.0 high speed and a 1–16-bit parallel capture unit with DMA streaming. A front end + P4 board can therefore be:
- a sampler (A), streaming to the Mac for Pocket SDR, GNSS-SDR or our own code;
- the embedded receiver (B);
- a host for an FPGA later (C).

An FX2LP/FX3 sampler can never become flight hardware.

## 9. Test path we already own

- **Scenario files with truth.**
  - The COCOM/boost rig writes signed 8-bit interleaved I/Q: gps-sdr-sim at 2.6 MS/s, SignalSim at 8.184 and 18.48 MS/s. Each file has a JSON truth sidecar.
  - Pocket SDR (`INT8X2`) and GNSS-SDR (`ibyte`) read the format as is, so a software receiver can be judged on exactly the trajectories the bought receivers failed.
  - The rig's smooth-carrier gps-sdr-sim build and SignalSim's 1 ms updates avoid the 0.1 s Doppler staircase. The stock-build A/B already ruled that out as the cause of the PX1105R losses.
- **What replay can't do, written into the files:**
  - the receiver's own oscillator under acceleration: a common-mode carrier and code-rate offset Γ·a(t)·f_L;
  - spin: a roll-rate phase term for an off-axis antenna.
- **A real boost.** PSAS's 2015 flight recording (MAX2769B, 4.092 MS/s zero-IF 2-bit, T−32 s to T+34.7 s), published with IGS precise orbits.
- **Real sky.** The HackRF records through an active antenna as 8-bit I/Q.
- **Hardware in the loop.** Once a front end exists, conducted HackRF replay drives it the same way it drives the bought receivers. Keep the level well below the LNA's ~−83 dBm compression.

## 10. Suggested order of work

**Stage 0: software receiver on files. No hardware needed; can start now.**
- Our own C (with Python for plots), with Pocket SDR as the reference implementation and cross-check. Covers acquisition, tracking, LNAV, observables and PVT for GPS L1 C/A.
- Then the rig's boost scenarios, to design and prove the boost loops against truth.
- Then the PSAS flight IQ, and the oscillator and spin injections.
- **Output:** a loop design proven on the same trajectories the bought receivers failed, in C that ports to the P4.

**Stage 0b: P4 benchmark.** About a day on an existing board, as described in §8. Decides B or C.

**Stage 1: one front-end board.**
- The front-end IC, our L1 LNA + SAW, the TCXO and the P4 on the Space Bug outline.
- It streams samples to the Mac over USB high speed first.
- **The chip depends on sourcing:** the MAX2771 if a handful can be had, otherwise the MAX2769B (L1-only).
- **Optional:** buy an assembled Pocket SDR front end now for real-sky MAX2771 recordings while our board is designed.

**Stage 2: real time on the target.** P4 software correlators, or an FPGA added if the benchmark says so.

**Stage 3: flight.**
- IMU aiding and phase-scheduled loops.
- The oscillator chosen and mounted for g-sensitivity.
- Bench qualification on the same rig as the bought receivers.

## 11. Decisions needed from you

**Decided 2026-09-30:**
- A flight receiver.
- L1 multi-GNSS first, on the MAX2769B.
- Room in the architecture for L5 later, on a MAX2771 (a couple are in hand).

The architecture that follows is proposed in [gnss-receiver-architecture.md](gnss-receiver-architecture.md). The original questions:

1. **Goal.** A flight receiver in the Space Bug class, or a lab instrument first? This review assumes flight eventually.
2. **Version 1 signals.** GPS L1 C/A only, or L1 multi-GNSS (+ Galileo E1, + BeiDou B1C)? L5 would come later with a second chip.
3. **Front-end chip, given supply.** MAX2771 (sourcing risk) or MAX2769B (L1-only, in stock)? Or design for the MAX2771 and buy a few at broker or reel prices?
4. **First hardware.**
   - Our own P4-based sampler board (the recommendation).
   - A clone of the Pocket SDR / jmfriedt sampler.
   - An assembled Pocket SDR front end, bought now.
5. **Code.** Write our own with Pocket SDR as the reference, or build directly on its BSD-2 library?

## 12. A note on export rules

A receiver we write has no COCOM-style gate unless we add one.
- **US ECCN 7A105.b.1 and the MTCR** cover receivers designed or modified for airborne use that navigate above 600 m/s.
- **Related rules.** Mouser reportedly lists the MAX2771 under ECCN 7A994 *(secondary)*. DLR's unrestricted Phoenix units need a German export licence. Pocket SDR's README carries its own notice.
- **Get a read before shipping units abroad or publishing flight-ready designs.** Building one and flying it is a different question from distributing it. This note is not legal advice.

## Sources

**Local references**
- `~/Downloads/`: MAX2771 datasheet Rev 2, MAX2771 EV kit guide, PLL loop-filter calculator guide, and GNSS_Firehose-master.
- Rig: `tools/gnss-cocom/sdr/README.md`.

**Samplers and front ends**
- Friedt's sampler: https://github.com/jmfriedt/max2771_fx2lp (and its issues).
- Pocket SDR:
  - https://github.com/tomojitakasu/PocketSDR (README, `conf/`, `src/sdr_ch.c`, FE READMEs, issues);
  - seminar slides https://gpspp.sakura.ne.jp/paper2005/pocketsdr_seminar_202411_revA.pdf;
  - assembled boards https://www.datagnss.com/products/pocketsdr-gnss-receiver.
- GNSS Firehose: https://github.com/pmonta/GNSS_Firehose; http://www.pmonta.com/gnss-firehose.html.
- Analog Devices product pages: https://www.analog.com/en/products/max2771.html, max2769b.html, max2769c.html.
- Distributor pages read 2026-09-30:
  - DigiKey MAX2771: https://www.digikey.ca/en/products/detail/analog-devices-inc-maxim-integrated/MAX2771ETI/9599235.
  - LCSC MAX2771: https://www.lcsc.com/product-detail/C400143.html.
  - DigiKey MAX2769B: https://www.digikey.co.uk/en/products/detail/analog-devices-inc-maxim-integrated/MAX2769BETI-V/2793249.
- NTLab: https://www.ion.org/gnss/upload/files/1526_Leaflet_NT1065_v2.pdf; https://intergeo25-hintefairs.expoplatform.com/product/10429; https://www.crowdsupply.com/amungo-navigation/nut2nt-plus.
- Direct sampling: Lamontagne et al. 2012, https://www.scirp.org/html/2-8501038_24409.htm.
- Low-g-sensitivity oscillators (vendor claim): https://www.sitime.com/company/newsroom/blog/what-g-sensitivity.

**Real-time receivers**
- Holme: http://www.aholme.co.uk/GPS/Main.htm.
- KiwiSDR: https://github.com/jks-prv/Beagle_SDR_GPS; http://kiwisdr.com/docs/KiwiSDR/KiwiSDR.design.review.pdf.
- Piksi: https://github.com/swift-nav/piksi_firmware; https://github.com/swift-nav/piksi_hardware; https://github.com/swift-nav/libswiftnav-legacy.
- Namuru: https://github.com/kristianpaul/gnsssdr; https://elib.dlr.de/56548/1/S11_02_Grillenberger.pdf.
- GNSS-SDR FPGA: https://gnss-sdr.org/ip-cores-available/; https://pmc.ncbi.nlm.nih.gov/articles/PMC10708737/.
- DLR Kodiak: https://elib.dlr.de/187866/1/A-082aicher.pdf.
- GMV launcher receiver: https://navisp.esa.int/uploads/files/project_documents/GNSSW-MLMSC_FINAL_PRESENTATION_El2-022.pdf.
- iliasam: https://github.com/iliasam/STM32F4_SDR_GPS; https://github.com/iliasam/ESP32_SDR_GPS; https://habr.com/ru/articles/789382/.
- Cornell: https://gps.mae.cornell.edu/humphreys_etal_iongnss2006.pdf.
- PSAS: https://github.com/psas/gps-rf-board; https://github.com/psas/Launch-12/tree/gh-pages/data/GPS.
- OreSat: https://github.com/oresat/oresat-gps-hardware.
- Open HDL: https://github.com/osqzss/gps-fpga; https://github.com/j-core/gnss-baseband; https://github.com/JuliaGNSS/gnss-m2sdr.

**Rocket flights**
- Maxus-4: https://www.dlr.de/de/rb/medien/publikationen/publikationen/gsoc-dokumente/eurock_xv.pdf/@@download/file/EuRock_XV.pdf; https://ntrs.nasa.gov/api/citations/20020060112/downloads/20020060112.pdf.
- Ebinuma & Nakasuka 2006: https://www.jstage.jst.go.jp/article/jjsass/54/635/54_635_542/_pdf.
- Phoenix: https://elib.dlr.de/137677/1/WISEE20-Markgraf-Phoenix.pdf.
- Ariane 5 VA219: https://elib.dlr.de/105403/1/Paper_JAE2015_OCAM-G_20160728.pdf.
- MAPHEUS-15: https://elib.dlr.de/214653/1/engproc-126-00039.pdf.
- IGAS: https://elib.dlr.de/82066/1/IGAS.pdf.

**Software**
- https://github.com/gnss-sdr/gnss-sdr; https://github.com/gnss-sdr/gnss-sdr-vtl.
- https://github.com/kristianpaul/SoftGNSS; https://github.com/nlsfi/FGI-GSRx.
- https://github.com/pmonta/GNSS-DSP-tools; https://github.com/JuliaGNSS; https://github.com/mx4/gnss-rcv; https://github.com/taroz/GNSS-SDRLIB.
- ESP-DSP benchmarks: https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-benchmarks.html.
- ESP32-P4 PARLIO RX: https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/peripherals/parlio/parlio_rx.html.

**Tracking and oscillators**
- Loop coefficients: Kaplan & Hegarty, *Understanding GPS/GNSS* (the coefficients, via GNSS-SDR's `tracking_loop_filter.cc`).
- Theses: Lian 2004, https://www.ucalgary.ca/engo_webdocs/GL/04.20208.PLian.pdf; Muthuraman 2010, https://www.ucalgary.ca/engo_webdocs/GL/10.20303.KMuthuraman.pdf.
- Navipedia tracking-loop pages.
- Llorente et al. 2026, https://arxiv.org/abs/2606.23925.
- Mu & Long 2021, https://pmc.ncbi.nlm.nih.gov/articles/PMC8402518/.
- Ban et al. 2017, https://pmc.ncbi.nlm.nih.gov/articles/PMC6189995/.
- Chiou 2005, https://web.stanford.edu/group/scpnt/gpslab/pubs/papers/Chiou_IONGNSS_2005.pdf.
- Abedi et al. 2015, https://pmc.ncbi.nlm.nih.gov/articles/PMC4610460/.
- Fry 2014, https://www.mwrf.com/technologies/components/active-components/article/21845512/manage-quartz-crystals-under-high-vibration.

**Export**
- https://www.space.commerce.gov/wp-content/uploads/2022-03-US-export-controls-GPS-GNSS-equipment.pdf.
