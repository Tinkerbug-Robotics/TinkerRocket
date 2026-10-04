# gnss-sdr-dev: development board spec (v0)

**Date:** 2026-09-30.
**Status:** approved by the owner 2026-09-30.
- Schematic captured the same day in `hardware/gnss-sdr-dev/` (ERC clean).
- Board placed by the owner, and routed on eight layers 2026-10-01/02.
- Review pass 2026-10-02: DRC clean.
- Ordered from JLCPCB 2026-10-04, tagged `gnss-sdr-dev-v0.0.0`. Fabrication notes: `hardware/gnss-sdr-dev/FABRICATION-NOTES.md`.

**Parents:**
- [gnss-receiver-architecture.md](gnss-receiver-architecture.md)
- [gnss-receiver-review.md](gnss-receiver-review.md)

**Project folder:** `hardware/gnss-sdr-dev/`.

## What the board is for

It is the first hardware for our own GNSS receiver.
- **Sampler first.** It streams raw samples to the Mac over USB, both for the software receiver and for recording real sky and rig replays.
- **Receiver next.** It is the real-time receiver with FPGA correlators and the ESP32-P4.
- **Test bed.** The oscillator, the L5 chain and IMU aiding get tested on it before the 35 × 35 mm flight board.

## Decisions (owner, 2026-09-30)

| Area | Decision |
|---|---|
| Role | Development board first; the flight board follows |
| L1 | MAX2769B |
| L5 | A MAX2771 low-band chain **on the board, populate-optional** (a couple of MAX2771s are in hand) |
| RF input | One SMA → switchable, current-limited antenna bias → **two-way divider** → L1 chain and L5 chain. Each chain has our own SAW + LNA ahead of its front-end IC, with a bypass for A/B tests |
| Reference | 27 MHz, chosen for vibration. **Part pending**: a search for parts under US$20 each is running |
| FPGA | **Lattice ECP5, caBGA256** (14 × 14 mm, 0.8 mm pitch). One footprint takes the 12F/25F/45F: first builds with the 25F, the 45F when stock returns. Sized for 32 L1 channels + L5 |
| Processor | ESP32-P4 chip-down, reusing the Space Bug's P4 subsystem |
| IMU | The Space Bug's ISM6HG256X |
| Host link | The Space Bug's host connector: J4, 5-pin JST-SH, pin-for-pin with the Mantis J1 |
| Power | USB-C 5 V + pack input. GNSS RF on its own LDO |
| Board | 8 layers, JLC08161H-2116 (sig/gnd/sig/gnd/pwr/sig/sig/gnd), the Beetle and Mantis stack. Outline ~80 × 60 mm, mounting holes. The board started on 6 layers (JLC06161H-3313); the owner allowed 8 on 2026-10-01 so the P4's pins could fan out |
| Dev features | FPGA JTAG header + SPI configuration-flash footprint; logic-analyzer header (sample bus, sample clock, 1 ms tick) |
| Shielding | Solderable fence + removable lid over the RF section |
| Software | Our own code, with Pocket SDR as the reference (separate session) |

## Block diagram

```
SMA ─ ESD ─ bias tee (switchable, current-limited) ─ 2-way divider ─┬─ L1 SAW ─ LNA ─► MAX2769B (L1)  ─ 4 bits ─┐
                                                                     │   (bypass ► LNA1)       │ CLKOUT 27 MHz   │
                                                                     └─ L5 SAW ─ LNA ─► MAX2771 (L5) ─ 4 bits ┄┐ │
                                                                         [populate-optional]   ▲ ADC_CLKIN     ┊ │
27 MHz reference ─► clock buffer ─┬─► MAX2769B XTAL                                 buffered sample clock ─────┊─┤
   (under the shield)             ├─► MAX2771 XTAL                                                             ▼ ▼
                                  └─► FPGA PLL ──────────────────────────────────────────────────► ECP5 (caBGA256)
                                                                                                    │  │  │   │
                                                               JTAG + config-flash footprint ───────┘  │  │   │
                                                               logic-analyzer header ──────────────────┘  │   │
                                                                                         QSPI + PARLIO + IRQ  │
                                                                                                  ▼           │
 J4 host link (5-pin JST-SH) ◄──────── ESP32-P4 ◄── ISM6HG256X (INT lines also to the FPGA) ◄──────────────────┘
 USB-C (high speed) ◄───────────────────┘   └── SPI to both front ends; PPS/event via the FPGA
```

## Subsystems

### RF input and chains

- **Input:** one edge-launch SMA (SMA rather than U.FL: vibration retention). ESD protection sized for an RF line.
- **Antenna bias:**
  - selectable 3.3 V or 5 V, off by default;
  - current-limited, with a fault flag readable by the P4;
  - DC-blocked from the divider.
- **Divider:** a broadband two-way divider after the bias tee, about 3 dB per path; negligible behind an active antenna.
- **L1 chain:** L1 SAW → LNA, the same topology as the Space Bug's one-stage chain, into the MAX2769B. The bypass path feeds the MAX2769B's own LNA for A/B tests. The schematic stage decides between:
  - two LNA inputs selected in software (LNA1 direct, LNA2 behind our LNA);
  - or 0 Ω links.
- **L5 chain (populate-optional):** L5 SAW → LNA → MAX2771 low-band LNA input. A SAW position sits in the MAX2771's LNA-out → mixer-in gap.
- **Supply and layout:** all RF, both front ends, the reference and its buffer sit on the **GNSS LDO rail(s)**, under the shield fence. Grounded coplanar RF lines; return vias beside signal vias; filled vias.

### Front-end ICs

- **MAX2769B**, from the Rev 1 datasheet via an archived copy; check against Rev 2 before release:
  - PGM tied low, so it runs in SPI mode.
  - SPI is write-only: 32-bit words, 28 data bits + 4 address bits. The P4 writes all 10 registers at power-up.
  - The reference at 27 MHz is within its 8–32 MHz range, AC-coupled (10 nF), ≥ 0.5 Vpp.
  - Sample clock = CLKOUT = the undivided 27 MHz reference (REFDIV ×1, ADCCLK = 0, FCLKIN = 0).
  - 2-bit I + 2-bit Q, sign/magnitude, 27 MS/s (the part is rated to ~50 MS/s).
  - IF filter 4.2 MHz, complex. The IF centre word is calibrated on the bench.
  - The synthesizer runs fractional-N from 27 MHz (ratio ~58, within its ≤ 251 limit). **Recalculate the loop filter** for 27 MHz; the datasheet values assume a 1.023 MHz comparison frequency and 50 kHz bandwidth.
  - SHDN and IDLE driven by the P4; LD read back.
- **CLKOUT drives capacitive loads weakly** (~2.2 Vpp at 32 MHz into 40 pF). Put a single-gate CMOS buffer beside it. Fan out from the buffer to the FPGA sample-clock input and the MAX2771 ADC_CLKIN.
- **Data changes a few ns after CLKOUT's rising edge,** so the FPGA samples on the falling edge. Confirm on the bench; the logic-analyzer header is there for it.
- **MAX2771** (L5, optional):
  - low band (LO 1160–1290 MHz), zero-IF, 23.4 MHz low-pass;
  - ADC clocked from ADC_CLKIN (the common 27 MHz sample clock), 2-bit I/Q;
  - 48-bit readable SPI; the two reserved fields rewritten after every power-up;
  - its own loop filter, recalculated for 27 MHz;
  - Q is inverted relative to the usual convention (the Pocket SDR/jmfriedt lesson).

### Reference oscillator

- **27.000 MHz.** Part chosen at schematic capture (Y201). Requirements:
  - ±0.5 ppm;
  - g-sensitivity specified and better than a standard GNSS TCXO's ~2 ppb/g;
  - clipped sine or LVCMOS output; 1.8–3.3 V.
- **Placement:** under the shield lid (keeps airflow off), near a mounting standoff rather than mid-span, with its most sensitive axis across the thrust axis.
- **Capacitors:** C0G/NP0 for every capacitor on the oscillator's supply filter, its AC coupling, and both synthesizer loop filters (class-II ceramics are microphonic).
- **Buffer:** a low-jitter fan-out buffer with three outputs (MAX2769B, MAX2771, FPGA), powered from the GNSS LDO.

### FPGA (ECP5, caBGA256)

- **Rails:** 1.1 V core, 2.5 V aux, 3.3 V I/O, each from its own regulator. The core supply is sized for the 45F.
- **Configuration:**
  - The P4 loads it over slave SPI (the configuration-mode pins set for slave SPI; the P4 drives PROGRAMN, INITN and DONE).
  - A SPI configuration-flash footprint allows a standalone boot.
  - A JTAG header.
- **I/O plan** (all 3.3 V banks):

| Group | Signals |
|---|---|
| L1 front end | 4 data + sample clock |
| L5 front end | 4 data |
| P4 | QSPI (CS, CLK, 4 data), PARLIO sample stream (8 data + clock + valid), IRQ (1 ms tick), spare GPIO |
| Timing | IMU INT1/INT2 (timestamped on the FPGA time base), PPS out, event in |
| Debug | Logic-analyzer header: sample bus, sample clock, 1 ms tick, 2–4 debug lines; status LEDs (DONE, PLL lock, PPS) |

- **Clock input:** a PLL-capable pin fed from the reference buffer (27 MHz → 108 MHz fabric).
- **Placement:** keep the BGA off the board centre (vibration: board curvature peaks mid-span).

### ESP32-P4, IMU, USB, host link

- **Reused sheets:** the Space Bug M8T's `esp32_p4`, `imu` and `usb` sheets, taken from the owner's current version at capture time. They are being edited in the main checkout (decoupling redraw 2026-09-30), so they are copied, not linked.
- **P4 duties:**
  - SPI to both front ends;
  - loads the FPGA;
  - QSPI master to the FPGA; PARLIO RX from it;
  - USB high speed to the Mac;
  - host UART on J4 (HOST_TX, HOST_RX, HOST_TX2 as on the Space Bug);
  - IMU on SPI as on the Space Bug.
- **J4:** 5-pin JST-SH, pin-for-pin with the Mantis J1 and the Space Bug. PPS and event also go to the logic-analyzer/debug header.
- **USB (owner, 2026-10-01):**
  - One USB-C, behind its ESD and common-mode choke, feeds a USB 2.0 switch as on the Mantis.
  - A slide switch picks the P4's USB-Serial/JTAG (flashing, console) or its high-speed PHY (sample streaming).
  - The second USB-C port was removed.
  - The pairs are routed to 90 Ω differential.
- **Boot button (owner, 2026-10-02):** on GPIO35, as on the Space Bug.
- **Data flash (owner, 2026-10-01):** the Mantis's flight-data NAND, on the P4's four free GPIOs, so a flight on this board can be recorded.

### Power

- **Inputs:** USB-C 5 V and the pack input (as the flight boards take it), OR-ed with Schottky diodes as on the Space Bug.
- **GNSS LDO:** a dedicated LDO chain for the RF, both front ends, the reference and its buffer. **No switcher feeds it** (owner rule). Budget *(estimate)*:

| Load | Estimate |
|---|---|
| MAX2769B (max) | ~31 mA |
| MAX2771 (optional) | ~27 mA |
| Two LNAs | ~10 mA |
| Reference + buffer | ≤ ~15 mA |
| **Total** | **≤ ~85 mA**, within a 200 mA LDO class |

- **Digital:** separate regulators for the P4 (per the Space Bug/HDG), the FPGA core (1.1 V), aux (2.5 V) and 3.3 V I/O.
- **Coexistence:** keep the switchers away from the RF section, outside the shield, with their own ground-return planning.

## Frequency rules for every clock on the board

- **Front end and FPGA:** 27 MHz and integer multiples only; the fabric runs at 108 MHz. No divided-down clock leaves the FPGA. A 9 MHz clock, for example, has a harmonic 0.42 MHz from L1.
- **P4 side:** 40 MHz multiples, i.e. the crystal and 40 or 80 MHz SPI/QSPI clocks. Their harmonics sit ≥ 15 MHz from L1 and ≥ 16 MHz from L5. **Avoid 20 MHz and 10 MHz** buses, which land inside L5.
- **Slow buses** (I²C, UART, IMU SPI ≤ a few MHz): keep edges slow and traces short, and check the harmonics of their clock rates against L1 ±2.1 MHz and L5 ±10.2 MHz.

## Open items before or during capture

1. Reference part: from the < $20 search.
2. L5 SAW, L5 LNA, divider and bias-switch parts, specified by requirement first.
3. Loop-filter values for both synthesizers at 27 MHz (ADI's calculator is in `~/Downloads`).
4. Confirm the MAX2769B against its Rev 2 datasheet (pins, reference range, registers).
5. FPGA pin assignment against the ECP5 caBGA256 bank map. Keep the sample buses on one bank, next to a PLL input.
6. J4 pinout copied from the Space Bug; decide which spare signals (PPS, event) go where.

## Next steps

1. ~~Owner approves or edits this spec.~~ Approved 2026-09-30.
2. ~~Create `hardware/gnss-sdr-dev/`: a KiCad 10 project with the shared libraries, the reused P4/IMU/USB sheets, and new RF, front-end, clock, FPGA and power sheets.~~ Done 2026-09-30.
3. ~~Schematic capture and ERC.~~ Done 2026-09-30 (0 errors).
4. ~~Placement and routing.~~ Placed by the owner; routed on eight layers 2026-10-01/02 at the owner's request.
5. ~~Design-review pass.~~ Done 2026-10-02:
   - DRC clean;
   - return vias added;
   - the USB pairs re-routed to 90 Ω;
   - 3D models for every part.
6. ~~The owner's review, then the fab package.~~ Ordered 2026-10-04 (tag `gnss-sdr-dev-v0.0.0`).
7. Bring-up.
