# PX1105R GNSS board ("TinkerRocket Space Bug") — design review, 2026-09-27

**Board:** `hardware/legacy/gnss-px1105r-18mm-highpower-ext-ant`, the owner's working copy as saved at 11:34 on
2026-09-27 (md5: PCB `de11f1ad`, root sheet `3859f4f7`, `esp32_p4.kicad_sch` `3f1e62f4`, `imu.kicad_sch` `bdce69bc`,
`usb.kicad_sch` `ca9449c9`, project `22b7929d`, `.kicad_dru` `a062c9bf`). **None of these files is committed.** The
board in git is still the 22 × 27.5 mm, 4-layer V2 carrier. The working copy was re-checked unchanged at 12:58, with
KiCad closed.

**What the board is now:** 35 × 35 mm, 8 layers (JLC08161H-2116 stack, 1.63 mm, ENIG). It carries 124 parts on four sheets:
- **Receiver:** the PX1105R L1/L5 receiver on its own 3.3 V LDO (U2).
- **Front end:** filter-first dual-band (SAW, LNA, SAW, LNA) for a 25 × 25 mm passive patch mounted on the back.
- **Processor:** an ESP32-P4 (chip revision v3.x) with 16 MB boot flash, placed between the host connector and the receiver.
- **Other:** an ISM6HG256X IMU, and USB-C with a Schottky OR into the supply.
- **Host connector:** the same 5-pin JST-SH as before (J4).

**Who drew what:** Claude sessions on 2026-09-26/27 generated the P4, IMU and USB sheets and most of the routing. The
owner then edited them; the 11:14 to 11:34 saves added 67 GND vias and re-routed the host lines. Several findings below
are defects in that Claude-drawn copper, and are reported as such.

**Method:**
1. Snapshot the saved files, together with the shared libraries, into a scratch tree.
2. Export with `kicad-cli`: netlist, ERC, DRC with schematic parity, BOM, gerbers, drill.
3. Measure geometry with pcbnew and shapely: zones, vias, pads, tracks, clearances, plane overlaps. One 2-D field solve for the RF line.
4. Run seven parallel specialist passes against the datasheets: power tree, RF front end, P4 circuit, interfaces, general
   layout, footprints and assembly, fab package and BOM.
5. Re-check every finding kept here against the files, and the load-bearing numbers by hand.

Datasheets used:
- PX1105R DS rev 2.
- ESP32-P4 datasheet v0.7, plus Espressif's *ESP32-P4 Hardware Design Guidelines* (HWDG).
- TPS6215x, TLV62569, LP5907.
- ADP7142 Rev A (package drawing) and Rev PrD (thermal table).
- BGA855N6 rev 2.2, plus Infineon AN596.
- B8389 SAW, GVLB258.A, TPD1E0B04, ISM6HG256X, W25Q128JV, CUS10S30.
- Murata DLW21S series.

**State of the files:**
- **DRC:** 0 errors, 0 unconnected, 11 old silk/library warnings. The zone fills are current: a scratch refill changed no area.
- **Parity:** the 8 warnings are MPN fields waiting for Update PCB.
- **ERC:** 4 errors, all on U1 pins left open without no-connect flags.

---

## Status after the fixes, 2026-09-28

The owner worked through the findings one at a time. The body of this review is left as written on 2026-09-27.

| Outcome | Items |
|---|---|
| Fixed in the board or its footprints | L1–L4, L6–L13, L15, S2, S4, S8–S11 |
| Fixed in firmware | S3: the P4 bridge image pulls its idle TX lines up from the bootloader on, and the Mantis and Beetle GNSS drivers pull their RX pins up |
| Recorded in the docs | S7 (`hardware/cables.md`); D1 and D7 (`FABRICATION-NOTES.md`); D2 (sheet title blocks, `docs/board-versioning.md`) |
| Checked, nothing to change | D4's RF width: 0.18 mm is 50 Ω on this stack. The RF netclass still says 0.34 mm, the 50 Ω width on a 4-layer 7628 stack; it only sets the default for new routing |
| Kept on purpose | L5 (the route under U5), S5, S6 |
| Skipped | S1 (the crystal works); L14 (the fab notes ask for a placement check on U6 and U10 instead) |
| Still open | S12–S14, D3, the rest of D4, D6, D8, M1–M3 (first article), F2–F6 |

D5 is the commit that carries this board. The parity gap it named was closed by #1544.

After the fixes: DRC 0 errors and 0 unconnected; the zone fills equal a fresh kicad-cli refill; the parity gate
passes (128 symbols linked by path).

**V3, 2026-09-29: six layers.** V2 (8 layers) was ordered; V3 is the same board on JLC06161H-3313, for cost, in the
order sig/gnd/sig/gnd/pwr/sig. Each old layer's copper moved whole: In5's routing became In2; In2's (the MCU pour, the
power tracks and nine signals) and In3's receiver plane became In4; In4's ground became In3; In6 was dropped. The
receiver's and the processor's 3.3 V now sit side by side on In4, which settles L1 by construction, and the lines
that cross that split return in the bottom pour 0.10 mm below. The RF traces are 0.16 mm (50 Ω on this stack) and
the RF netclass matches, which settles D4's width. DRC 0 errors, 0 unconnected; the fills are a kicad-cli fixed
point; 128 symbols linked by path.

---

## Verdict

**No blockers.** The circuit is right where it most often goes wrong.

- **Footprints:** every footprint's pad numbering matches its datasheet. This includes the WLCSP flash, the LGA IMU, the TSNP LNAs, the SAWs and the QFN-104.
- **P4 circuit:** it matches Espressif's v3.x reference:
  - pin 54 is on the core rail;
  - the 499 k / 499 k / 22 pF feedback network is fitted;
  - there is no DP pull-down;
  - the TLV62569's spare pin 6 may legally go to ground.

  The flash on +3V3_MCU is correct with default eFuses.
- **RF chain:** it copies Infineon AN596 Option B exactly, and the DC paths are right. The LNA outputs are DC-blocked inside the package (datasheet Fig. 1), every SAW port sits at 0 V, and C26 keeps the receiver's own 3.3 V antenna bias off U4.
- **Power-up order:** the receiver's rail is up at 1.6 ms, the P4's at 5 ms, and the P4 leaves reset about 11 ms after that. The P4 cannot drive the receiver's inputs before the receiver is powered.
- **Host link:** J4 matches the Mantis J1 header pin for pin. The P4 talks to the receiver on the Mantis P4's own GNSS pins (GPIO2/3/4), so drivers port over.

What needs attention before an order:

1. **The GNSS 3.3 V plane is the path by which P4 noise can reach the receiver.** On In3 it runs under the P4's
   +3V3_MCU pour for 318 mm², 0.15 mm apart (72 pF). It also comes to within 0.2 mm of the antenna feed barrel. Two
   layout changes close the path (L1, L2); this is what the "GNSS stays on the LDO" rule depends on.
2. **Three risks that only the first article can settle:**
   - P4 clock harmonics fall inside BDS B1I and GLONASS L1, with the P4 pads 2.6 mm from the antenna feed pad (M1).
   - The patch sits on a quarter of its tuned ground plane (M2).
   - About 1.5 W heats a 35 × 35 mm board (M3).

   Each has a bench test below.
3. **The release package is not ready:**
   - the fab notes record only the stencil;
   - the revision still says V2;
   - there are no fiducials;
   - the rules have no minimums.

   See D1–D8.
4. **The host link now depends on P4 firmware, which does not exist yet.** The Mantis flight computer speaks only UBX (F1).

---

## Summary

| ID | Severity | Finding | Who |
|---|---|---|---|
| **L1** | **major** | The In3 GNSS +3V3 plane lies under the +3V3_MCU pour for 318 mm² (72 pF); In2 signals return in it | layout |
| **L2** | **major** | The antenna feed keep-out is missing on In3–In6: the GNSS plane and GND come within 0.2 mm of the feed barrel | layout / footprint |
| **M1** | **major (risk)** | P4 harmonics at 1560 MHz (B1I) and 1600 MHz (GLONASS L1) pass both SAWs. The P4 is 2.6 mm from the feed pad, with no shield | test |
| **M2** | **major (risk)** | The patch sits on 25 % of its tuned 70 × 70 mm ground plane: expect detuning and several dB less gain | test |
| **M3** | **major (risk)** | About 1.5 W on 35 × 35 mm. By estimate, the 85 °C parts reach their limit near 40 °C ambient | test |
| **D1** | **major** | FABRICATION-NOTES.md records only the stencil; no Block A/B for an 8-layer assembled order | docs |
| L3 | minor | 12 of 63 signal vias still have no GND via within 1 mm; none of the 67 new GND vias fixed one | layout |
| L4 | minor | U4 pin 4 (second LNA ground) reaches a via only through 1.3 mm of pour. The other three LNA ground pins each have a via about 0.45 mm away | layout |
| L5 | minor | The LNA1-to-SAW2 line runs under U5, 0.21 mm from its +1V8 and +3V3 pins | layout |
| L6 | minor | B.Cu test-pad traces cut an 11.6 mm slot in the patch ground, 0.5 mm from the patch edge; UART0 TX runs there | layout |
| L7 | minor | U2's exposed pad has no vias (the SAM carrier's identical U2 has 9); U2 dissipates up to 0.58 W | layout |
| L8 | minor | U9's input caps reach PVIN via the EN pin; PGND has no via within 2 mm; the exposed pad has 1 via | layout |
| L9 | minor | ESP_VDD_HP is 0.15–0.3 mm on every path (Espressif asks ≥ 20 mil); VIN has 15 mm of 0.15 mm on In2 | layout |
| L10 | minor | P4 decoupling: pins 75/77 (VDD_LDO/VDD_DCDCC) have no close 10 µF; pins 9 and 96 have no cap within 3 mm | layout |
| L11 | minor | The flash's only GND ball (E3) reaches ground through 1.6 mm of 0.09 mm track, 4.5 mm from C32's ground | layout |
| L12 | minor | R11 sits at the P4 end, so the host-driven side of HOST_TX2 crosses the board unresisted (R8/R9 sit at J4) | layout |
| L13 | minor | No fiducials (Beetle and Mantis carry 4; the fab asks two per side) | layout |
| L14 | minor | Pin-1 marks: none on U6, U10's is on the back silk, U9's is 65 % clipped, U2's is 0.08 mm from the edge | layout / library |
| L15 | minor | 12 different-net via pairs sit at 0.20–0.25 mm hole to hole (Beetle's rule is 0.25) | layout / fab |
| S1 | minor | No series-part footprints on FLASH_CK, SENS_SCLK or the crystal, which Espressif recommends; they are the only edge-rate knob for M1 | schematic |
| S2 | minor | J4's mounting tabs are on no net (stale 5-pin symbol); library, Mantis J1 and SAM J3 tie them to GND | schematic |
| S3 | minor | The receiver's RXD and the host's RX line float while the P4 is in reset, boot or download mode | schematic |
| S4 | minor | No access to the receiver's BOOT_SEL, so recovering a corrupted receiver flash means wiring pad 18 | schematic / layout |
| S5 | minor | On pack power, VBUS floats to about 6 V through D9's reverse leakage, which can block USB-C attach | schematic |
| S6 | minor | FL2 is rated 320 mA; the board draws 0.20–0.25 A typical and 0.31–0.48 A at Espressif's 380 mA P4 sizing | schematic |
| S7 | minor | A reversed jumper sends the whole board return (~0.2 A) into the Mantis GPIO3 clamp; cables.md calls it silent | docs / host |
| S8 | minor | U1 paste is cut to 64 % by a −0.1 ratio, with no recorded reason; V2 printed 100 % | footprint |
| S9 | minor | U9 exposed-pad paste: an old polygon and four new panes merge into one accidental 70.6 % blob | footprint |
| S10 | minor | U3/U4/D6 mask openings equal the copper; Infineon and TI specify NSMD | footprint |
| S11 | minor | U2's exposed-pad land (2.41 × 3.05 mm) is larger than the package pad (2.29 × 2.29 mm) and pasted 100 % | footprint |
| S12 | minor | U8's footprint has no courtyard, and missing courtyards are set to "ignore" | library |
| S13 | minor | CHIP_PU is RC-only; a partly held-up rail can skip a clean reset (Espressif suggests a ~3.0 V supervisor) | schematic |
| D2 | minor | Revision: "V2" (title block, silk, gerbers) on a design unrelated to the committed V2; all schematic title blocks empty | docs |
| D3 | minor | Board minimum track width and clearance are 0 (unset); Beetle/Mantis use 0.09/0.09 | rules |
| D4 | minor | Netclasses don't match the copper: RF width 0.34 mm (≈ 35 Ω here) vs 0.18 drawn; class membership by auto-generated net names | rules |
| D5 | minor | Commit set: the untracked `.kicad_dru` is load-bearing (428 DRC errors without it); the parity gate can't see missing sheets | docs |
| D6 | minor | BOM: 8 footprints lack MPN/Mfr; D2/D5 values differ; no committed unpriced bom.csv | docs |
| D7 | minor | Reflow limits (U1 peak 240 °C, MSL4) and SAW ESD sensitivity are not recorded for the assembler | docs |
| D8 | minor | Doc drift: legacy/README, hardware/README, board-versioning, cables.md, 3dmodels/README | docs |
| S14 | minor | ERC hygiene: no-connect flags on U1 pins 5/13/14, a 0.2 mm dangling wire, symbol-cache refresh changes J4 | schematic |
| F1 | note (system) | The host link is P4-to-P4 and the Mantis flight computer speaks only UBX: until the P4 has an image the host hears nothing | firmware |
| F2–F6 | note | Firmware obligations, the programming procedure, cold starts, J4.5 direction, PPS | firmware |
| N1–N14 | note | Observations that need no action (listed at the end) | — |

---

## 1. Fix before fab — layout (the owner's copper; everything here is a proposal)

### L1. The GNSS +3V3 plane on In3 lies under the +3V3_MCU pour on In2 for 318 mm² — major

- **Where:** In3 zone named "In2 +3V3 (GNSS)" (net +3V3, full board, 885 mm², one piece); In2 zone "+3V3_MCU (P4)" (337 mm²).
  The overlap covers x 56.3–70.6, under U6, U7, U8/L9, U9/L10, U10 and Y1.
- **Evidence:**
  - **Coupling capacitance.** 318.2 mm² of the +3V3_MCU fill sits directly over the +3V3 fill, through 0.1528 mm of
    1080 × 2 prepreg (εr 3.91). That makes 72 pF between the P4 rail and the receiver rail (measured twice,
    independently).
  - **Plane capacitance to GND.** The +3V3 plane has about 230 pF to ground: 115 pF to In4 and 115 pF to the In2 GND
    fill. At L-band the 0402/0805 capacitors are inductive, so this plane capacitance sets the divider. The receiver
    plane follows +3V3_MCU noise at about −12 dB at 1.6 GHz, and about −40 dB at 100 MHz. FL3 (470 Ω at 100 MHz) is
    meant to be the high-frequency barrier; 72 pF is 22 Ω at 100 MHz and 1.4 Ω at 1575 MHz.
  - **In2 returns in the GNSS plane.** In2 is 0.153 mm from In3 but 0.300 mm from In1 GND, so about two thirds of every
    In2 trace's return current flows in the GNSS plane. Nets routed on In2 over it: VIN 14.0 mm (U9's whole input),
    HOST_TX2 10.7, HOST_TX 5.2, GNSS_PPS 5.2, ESP_VDD_HP 5.8, J4-Pad1 3.3 mm.
  - **Nothing east of x = 56 needs the plane.** Every +3V3 load is west of x = 56: U1 pins 6/8, U5, C7/C8/C27, R12, and
    FL3.2 (the source, x 55.94). The only exception is R10, the power LED, at (62.81, 35.66).
  - **The same plane lies under both buck switch nodes** (U8 SW, L10.1), with In1 GND between.
  - PX1105R DS p. 12 asks for under 50 mVpp ripple and says "power supply noise can affect the receiver's sensitivity".
- **Consequence:** above about 100 MHz, P4 and buck noise reaches U1's VCC and U5's input (and so the LNA supply)
  without passing U2 or FL3. This is the same mechanism behind the rule that keeps the receiver on an LDO. How much
  C/N0 it costs cannot be computed from the files; M1 measures it.
- **Proposal:** keep the GNSS plane out of the P4 half. Restrict the In3 +3V3 zone to FL3 and everything west of it
  (x ≲ 56). The existing lower-priority In3 GND zone then fills the east half. Feed R10 with a track, or move the power
  LED to the west side.
  - **Result:** the P4's In2 signals get GND on both sides, and +3V3_MCU gains about 72 pF of plane decoupling to ground.
  - **Equivalent alternative:** make In3 all GND and pour the GNSS +3V3 on the west half of In2. The two supply pours
    then couple edge to edge instead of face to face.

### L2. The antenna feed keep-out is missing on In3–In6 — major

- **Where:** AE1 pad 1 at (53.63, 49.78). The footprint's rule areas are "feed clearance, top and inner" (B.Cu, In1,
  In2 only) and "feed clearance, bottom" (F.Cu).
- **Evidence:** radius from the feed centre to the nearest other copper:

  | Layer | Measured | Required (GVLB258.A §4.3, antenna on the back) |
  |---|---|---|
  | F.Cu | 1.750 mm | 1.75 mm (Ø3.5, feed-trace side) — OK |
  | B.Cu, In1, In2 | 1.500 mm | 1.50 mm (Ø3.0) — OK |
  | **In3 (+3V3, the GNSS rail)** | **0.801 mm** | 1.50 mm |
  | **In4, In5, In6 (GND)** | **0.801 mm** | 1.50 mm |

  §4.3 says "the top copper keep out area applies to all other layers". With the antenna on the back, "top" means
  every layer except the F.Cu feed side.

  Two effects follow:
  - **Noise coupling.** The Ø2.2 mm F.Cu feed pad overlaps the In3 plane, and In3 comes 0.2 mm from the feed's 1.2 mm
    inner pad. Together they couple about 0.2 pF from the antenna port to the GNSS plane. That is about −26 dB at L1
    into the port node, ahead of the ESD part, the SAW and the first LNA.
  - **Detuning.** The extra shunt capacitance at the feed is about 0.2–0.4 pF over the reference layout, which gives
    |Γ| ≈ 0.07–0.15 at L1.
- **Consequence:** whatever noise is on the GNSS plane (see L1) reaches the most sensitive node on the board, before any
  filtering or gain. For example, 100 µV in band on the plane is about −93 dBm at the port, around 37 dB above a
  satellite. There is also a small detuning of the feed.
- **Proposal:**
  - Make the footprint's inner rule area cover all inner layers (Ø3.0 mm keep-out: no fill, no tracks, no vias), and
    keep the 19-via GND ring.
  - Optionally drop the unused inner pads of the feed pin.
  - Fix the library footprint and the board copy together. They already disagree (D5); a later "update footprint"
    would otherwise change the keep-out silently.

### L3. 12 of 63 signal vias still have no GND via within 1 mm — minor (owner's rule)

- **Evidence:** the metric is the same as at 11:14 (12/63). Of the 67 GND vias added since then, 27 went into the RF
  area, 8 into the P4 courtyard and 32 elsewhere, and only 2 landed within 1 mm of any signal via. Moving R11 created
  one new violator; re-routing HOST_TX removed one. Distance from each violating via to the nearest GND via:

  | Via | at | nearest GND via |
  |---|---|---|
  | SENS_SDI | (61.375, 46.625) | 1.78 mm |
  | SENS_SCLK | (60.775, 46.575) | 1.75 |
  | HOST_TX2 (at R11.1) | (56.560, 51.810) | 1.60 |
  | ISM6HG256_CS | (62.400, 46.350) | 1.28 |
  | CHIP_PU | (58.450, 48.050) | 1.25 |
  | HOST_TX2 | (47.125, 46.100) | 1.25 |
  | CHIP_PU (B.Cu–In5) | (66.900, 44.100) | 1.22 |
  | ISM6HG256_INT2 | (60.450, 47.850) | 1.15 |
  | Net-(J4-Pad1) | (49.575, 39.800) | 1.15 |
  | USB_D- (at R22.1) | (55.200, 57.250) | 1.08 |
  | Net-(J4-Pad1) | (49.575, 43.875) | 1.05 |
  | FLASH_HD | (59.750, 56.350) | 1.05 |

  The two U2 SENSE vias, at (54.35, 38.275) and (54.35, 40.20), also exceed 1 mm. They are DC nets, and both have room
  for a GND via at (55.03, 38.28) and (54.95, 40.20). A 5° ring search with the board's clearances found no GND via
  site for the other 12 without moving copper.
- **Proposal:** move each listed via, or the copper beside it, so that a GND via fits within 1 mm. Do the fast nets
  first: the IMU SPI cluster above the P4's top pin row, FLASH_HD in the via row at y 56.35, and USB_D- at R22.1.
  Two caveats:
  - The J4-side vias carry host-referenced signals whose return is VSS through FL2, so a board GND via does not close
    that loop (L12).
  - An F.Cu–In2 transition returns partly through the In3 +3V3 plane (L1).

### L4. U4 pin 4 (second LNA ground) reaches a via only through the F.Cu pour — minor

- **Evidence:** U4.4 at (39.30, 57.80). Its nearest GND via is 1.33 mm away, reached through pour necks about
  0.2–0.25 mm wide (≈ 0.5 nH). U3.1, U3.4 and U4.1 each have a via 0.43–0.46 mm away on a direct track. There is a legal
  0.4/0.3 via site at (39.00, 57.95), clear by at least 0.246 mm on all layers and at least 0.2 mm hole to hole.
- **Consequence:** extra common-lead inductance on one LNA: a small gain and match shift, and a little less stability
  margin.
- **Proposal:** add the via at (39.00, 57.95). Optionally give U4's VCC pin a direct F.Cu path to C25; it now crosses
  1.5 mm of In5.

### L5. FL5-IN (LNA1 output to SAW2) runs under U5 between its pins — minor

- **Evidence:** segment (42.60, 59.775)–(42.60, 61.175) passes under the SOT-23 body of U5, between pad 5 (+1V8) and
  pad 4. It keeps 0.21 mm from the +1V8 and +3V3 pads and crosses the In1 antipad of a +3V3 via. At 6.5 mm with 12
  bends, it is the longest RF net; no other is over 2.7 mm. PX1105R DS p. 13: "do not route the RF signal under or over
  any other components".
- **Proposal:** route around U5 with at least 0.35 mm (3h) to other copper and solid In1 underneath, or move U5. The
  LP5907 tolerates a remote output capacitor, so U5 can move away from the RF path. Either change also shortens the net.

### L6. The B.Cu test-pad traces cut an 11.6 mm slot in the patch ground, 0.5 mm from the patch edge — minor

- **Evidence:** BOOT (9.15 mm on B.Cu), RXD0, TXD0, CHIP_PU and a +3V3_MCU stub run on B.Cu from vias at
  x 66.6–66.9 to the test pads at x 69.4. The patch body edge is at x 66.13. The resulting B.Cu void is 11.61 mm long
  (19.2 mm²), 0.50–0.72 mm from the patch outline. TXD0 is UART0 TX: the ROM boot log and the default ESP-IDF console,
  with no series resistor. B.Cu is the patch's ground and the only copper on the antenna side.
- **Consequence:** a small extra detuning from the slot. Every boot, and every in-flight log line if the console stays
  on UART0, launches edges 0.55 mm from the patch edge.
- **Proposal:** route the test-pad feeds on In5 (between GND layers) and drop to B.Cu only at each pad, keeping B.Cu
  solid within about 3 mm of the patch outline. In firmware, keep UART0 quiet in flight (F2).

### L7. U2's exposed pad has no vias — minor

- **Evidence:** pad 9 is 2.41 × 3.05 mm and has 0 vias inside it. Six GND vias ring it 0.41–0.44 mm outside its short
  edges. The committed SAM-M10Q carrier, with the same part and footprint, has 9 vias in the pad. ADI recommends
  connecting the pad to the ground plane. U2 dissipates up to 0.58 W at 8.4 V: (7.95 − 3.3) V × 125 mA. The thermal
  table gives ψJB 32.7 °C/W for the SOIC-8.
- **Proposal:** a 3 × 3 GND via array in the pad, as on the SAM carrier. The vias are filled, so this is a thermal
  asset only. It also helps M3.

### L8. U9 (P4 buck): input capacitors not at the pins, PGND not stitched — minor

- **Evidence:**
  - **Input capacitor path.** C33.1 reaches PVIN pin 12 through 1.5 mm of 0.15 mm track via the EN pad; C34.1 through
    1.3 mm. TI (TPS6215x §9.2.2 and §11.1) wants the input capacitor directly across PVIN and PGND.
  - **Grounding.** PGND pins 15/16 reach the exposed pad by one 0.2 mm track, and the nearest GND via is 2.2 mm away
    (centre to centre). The exposed pad has 1 via, and there are only 3 GND vias within 3 mm of U9's centre.
  - **AVIN.** Pin 10 has no 100 nF of its own.
- **Consequence:** a pulsed input loop of a few nH (about 0.7–0.8 A peaks) instead of under 1 nH, so more switch-node
  ringing and emission. U9 is well placed: 15.3 mm from the feed and 21 mm from U3, with L10 outside the patch outline.
  So this costs coexistence margin, not function.
- **Proposal:**
  - Put the input ceramic directly across pins 11/12 and 15/16, with connections of at least 0.3 mm.
  - Add 2–3 GND vias at PGND and 3–4 in the exposed pad.
  - Add a 100 nF from AVIN to AGND.
  - Fix the pad's paste in the same pass (S9).

### L9. Supply copper narrower than Espressif asks — minor

- **ESP_VDD_HP.** 0.15–0.3 mm on every path, 56.7 mm in total (F.Cu 22.6, In2 8.4, In5 25.7), 11 vias. HWDG §1.4.1
  asks for at least 20 mil (0.51 mm), star-routed. Network resistance from L9.2 is 35 / 56 / 120 / 152 mΩ to pins
  91 / 76 / 54 / 26. At 0.3–0.5 A that is a 9–37 mV uncompensated drop, because R17 taps the rail near L9. That is
  1–3 % of 1.1 V, inside the 0.99–1.3 V range. The thin 20–25 mm branches to pins 54 and 26 also add about 8–10 nH.
  Mantis WORKLIST H-10 flags the same issue on Mantis.
- **VIN on In2.** 15.2 mm, of which 14.9 mm is 0.15 mm wide on 15.2 µm copper (≈ 0.107 Ω). It is U9's entire input
  and passes under Y1 and U6. On USB at Espressif's 380 mA P4 budget it carries about 0.31 A, above the ≈ 0.2 A IPC-2221
  internal-layer figure for that cross-section. Typical is about 0.12 A.
- **Proposal:** widen the VIN In2 run to the 0.4 mm used on the rest of VIN; the +3V3_MCU pour gives way. Widen the
  ESP_VDD_HP inner runs toward 0.5 mm, at least the branches to pins 54 and 26, or pour ESP_VDD_HP as a region on In5
  under U6.

### L10. P4 decoupling placement — minor

- **Evidence:**
  - **Pin 62:** the only +3V3_MCU pin with a cap directly on it (C44, 0.62 mm).
  - **Other +3V3_MCU pins** reach their nearest 100 nF through the In2 pour, with two vias each:

    | Pin | Function | Path to nearest 100 nF |
    |---|---|---|
    | 9 | | 4.0 mm |
    | 21 | | 3.0 mm |
    | 51 | | 2.8 mm |
    | 75 | VDD_LDO | 3.5 mm |
    | 77 | VDD_DCDCC | 2.5 mm |
    | 85 | | 2.7 mm |
    | 96 | | 5.0 mm |
    | 101/102 | | 3.0 mm, one shared via |

  - **Espressif** (HWDG §1.4.1) asks for a 10 µF close to each of VDD_LDO and VDD_DCDCC, because they carry the internal
    flash and PSRAM regulators. The nearest 10 µF is C38, 4.2–5.0 mm away, and it is also U8's input capacitor.
  - **Mantis** had its 100 nF about 1.5 mm from these pins.
  - **VDD_FLASHIO (pin 30):** its C61 is 3.7 mm away through In5.
  - **C50:** its GND pad has no via within 1.3 mm.
  - **Fine:** the core, PSRAM, VDDO_3/4 and VO1 output pins all have caps within 0.6–1.3 mm with no via.
- **Proposal:**
  - Give pins 75 and 77 a 100 nF within about 1.5 mm on F.Cu, plus a 10 µF nearby.
  - Move one of C42/C47 next to pin 9, and one of C53/C54 next to pin 96.
  - Put C61 on F.Cu to pin 30.
  - Add a via at C50's GND pad.

### L11. The flash's only GND ball reaches ground through 1.6 mm of 0.09 mm track — minor

- **Evidence:**
  - **Ground path.** U7.E3 at (59.50, 59.60) reaches its via at (59.45, 58.30) through 1.62 mm of 0.09 mm track. The
    via is 4.5 mm from C32's GND via.
  - **Supply path.** U7.B2 (VCC) is fed by 2.0 mm of 0.09 mm track from C32.
  - **No via in the pour.** The 0.15 mm² F.Cu GND piece at E3 is the board's only pour piece without a via.
  - **Loop inductance.** The two tracks add about 1.5 nH, so the supply loop is about 2 nH.
  - **Why no via fits.** FLASH_WP occupies the gap where a via next to E3 would fit.
- **Consequence:** in quad reads, 30–65 mA edge currents into about 2 nH give an estimated 60–130 mV of ground and supply
  bounce. That eats timing margin at high QSPI clocks.
- **Proposal:** a GND via within about 0.5 mm of E3 (re-route FLASH_WP around it). Widen the VCC and GND stubs once they
  leave the ball field, and place C32's GND via next to it.

### L12. R11 sits at the P4 end, so the host side of HOST_TX2 crosses the whole board — minor

- **Evidence:**
  - **R11.** Now at (56.56, 52.32), next to GPIO12. HOST_TX2 runs 25.9 mm from J4.5 (F.Cu 9.9, In2 11.6, In5 4.4,
    3 vias), 10.7 mm of it on In2 over the GNSS plane and 3.5 mm within 1 mm (plan view) of RF copper.
  - **R8/R9.** Both sit about 10 mm from J4, so the long runs of the other two host lines are on the 1 kΩ side.
  - **The 11:34 move** lengthened the unprotected run from 24.1 to 25.9 mm.
  - **Naming.** HOST_TX2 names the J4 side of its resistor, while HOST_TX/HOST_RX name the P4 side.
- **Consequence:** the host's full-drive UART edges, once RTCM is sent, cross the board with their return current in the
  GNSS plane and back through FL2. An ESD hit on J4.5 also meets no resistance for 26 mm.
- **Proposal:** put R11 next to J4 like R8/R9, and route the long side on F.Cu or In5. Name the GPIO12 side consistently.

### L13. No fiducials — minor

- **Evidence:** 0 fiducial footprints. Beetle, Mantis and V9 carry 4 each. Mantis WORKLIST H-5 records "two per side,
  which is this fab's requirement". This board adds a 0.35 mm-pitch QFN, a 0.5 mm WLCSP and two 0.4 mm TSNPs.
- **Proposal:** 2–3 top-side fiducials spread as far apart as possible. Candidate copper-clear spots are near
  (48.45, 48.32) and (58.42, 65.89); check the silk there. Otherwise confirm in writing that the assembler's panel rails
  carry fiducials.

### L14. Pin-1 marks — minor

- **U6:** the front silk is 8 identical corner brackets; its only pin-1 circle is on F.Fab.
- **U10:** the circle is on B.SilkS, so it prints on the back under AE1. This library issue is shared with Mantis and
  Beetle.
- **U9:** the dot loses 65 % of its area to L10's mask opening.
- **U2:** the dot is 0.08 mm from the board edge (DRC is silent because min silk clearance is 0).
- **FL2 and U8:** weak marks.

These parts have symmetric pad patterns, so a rotated part still lands on pads. For example, U2 rotated 180° puts its
GND pin on VIN. Placement follows the CPL; the marks are how inspection catches a mistake.

- **Proposal:**
  - U6: front-silk dot near (57.25, 46.85).
  - U10: move the circle to F.SilkS.
  - U9: shrink the dot into the gap by L10.
  - U2: move the dot to about (52.39, 36.66).
  - Fix the P4 and IMU library footprints too.

### L15. 12 different-net via pairs sit between 0.20 and 0.25 mm hole to hole — minor (confirm with the fab)

- **Evidence:**
  - **Rules.** The board rule is 0.20 mm edge to edge (V2 used 0.25). Beetle, the only 8-layer board this repo has
    fabbed, sets 0.25 mm between different nets "for this board being eight layers at 1.63 mm".
  - **Count.** There are 12 pairs below 0.25 mm; for example FLASH_HD/FLASH_CK and VDDO_FLASH/FLASH_HD at 0.200 mm in
    the row at y 56.35, and ESP_VDD_HP/FB_DCDC at 0.202 mm.
  - **Gate.** The check is a warning, so the release gate never fails on it.
- **Proposal:** either state 0.20 mm different-net spacing in Block A and ask the fab, or add Beetle's rule to the
  `.kicad_dru` and move the 12 vias.

---

## 2. Fix before fab — schematic and footprints (proposals)

### S1. No series-part footprints on the flash clock, the IMU SPI clock or the crystal — minor

- **Evidence:** Espressif recommends zero-ohm series footprints on the flash SPI lines, citing "mitigating RF
  interference" (HWDG §1.3.5), and a series resistor plus capacitor to ground on SPI clocks, close to the chip (§1.3.7).
  Espressif's v3 reference also has 0 Ω placeholders on XTAL_P/N. On this board the flash nets connect only U6 and U7
  (0.09 mm traces beside a 0.5 mm-pitch WLCSP), and SENS_SCLK and the XTAL nets have no series parts.
- **Consequence:** if M1 shows desense, firmware drive strength is the only knob, and adding parts later means a respin.
- **Proposal:** fit 0 Ω footprints at the P4 end of FLASH_CK (at least), SENS_SCLK and XTAL_N.

### S2. J4's mounting tabs are on no net — minor

- **Evidence:**
  - **Tabs.** J4 pads 6/7 (1.30 × 1.91 mm, soldered and pasted) have no net. The schematic holds a stale 5-pin copy of
    the connector symbol. The `Custom` library symbol has pins 6/7 = GND, and so do Mantis J1 and SAM J3.
  - **Neighbours.** Tab 7 sits 0.66 mm from Net-(J4-Pad1), and tab 6 0.87 mm from HOST_TX2.
  - **FL2 unaffected.** The tabs never touch the cable, so grounding them does not bypass FL2.
- **Proposal:** update J4 from the library, wire pins 6/7 to GND, Update PCB, refill.

### S3. The receiver's RXD and the host's RX line float while the P4 is not driving them — minor

- **Evidence:**
  - **P4 pin states** (datasheet Table 2-1): after reset GPIO4 (→ U1 RXD) is input-only with no pull, and GPIO10
    (→ R8 → host RX) is high-impedance. GPIO2 has a weak pull-up, so RXD2 stays high.
  - **Receiver.** PX1105R DS: RXD "should be driven HIGH" when idle.
  - **Mantis.** Its RX input has no pull-up on the board side either.
  - **Duration.** Both lines float for 0.1–0.5 s at every P4 boot, and indefinitely with blank flash or in download mode.
- **Consequence:** framing errors at the receiver, and garbage at the host parser at every power-up. An accidental
  receiver command is very unlikely: it needs the 0xA0 0xA1 header and a checksum.
- **Proposal:** a pull-up of at least 47 k on GNSS_RX to +3V3 (the receiver's rail), and 10–100 k on HOST_RX at the P4
  side of R8. Otherwise document that firmware enables the GPIO4/GPIO10 pull-ups first.

### S4. No access to the receiver's BOOT_SEL — minor

- **Evidence:** PX1105R pin 18: "pull-low for loading firmware into empty or corrupted Flash memory". Here pad 18 has no
  net; its castellation sits 1.6 mm from C22 in the RF chain. SkyTraq's evaluation board brings BOOT_SEL to a header.
  The six test pads serve only the P4.
- **Consequence:** a receiver firmware update interrupted by power loss leaves the receiver unrecoverable without a
  hand-soldered wire beside the RF chain, plus a P4 bridge image.
- **Proposal:** a BOOT_SEL test pad with a GND pad beside it, away from the RF path: the back strip at x < 40.4, or the
  test-pad column at x 69.4. Do not tie BOOT_SEL to a P4 GPIO unless RSTN is held low until the P4 rail is up. R12
  releases the receiver as soon as +3V3 rises, and an unpowered P4's clamp would hold BOOT_SEL low at that moment.

### S5. On pack power, VBUS floats to about 6 V through D9's reverse leakage — minor

- **Evidence:**
  - **D9 leakage.** With the pack on VSYS (about 8 V), D9 is reverse-biased. Its leakage (CUS10S30 datasheet: 0.2 mA
    typical at 30 V and 25 °C, tens of µA at a few volts, rising steeply with temperature) charges VBUS.
  - **No load on VBUS.** The only DC path is D7's clamp channel (5.5 V standoff); there is no bleed resistor. So VBUS
    rises until D7 conducts, about 6 V on the receptacle.
  - **USB-C attach rule.** A compliant USB-C source applies VBUS only after it sees VBUS at vSafe0V (≤ 0.8 V). This is
    recalled from the Type-C specification, not from a repo document.
- **Consequence:** a pack-powered board plugged into a USB-C port may get no attach and no USB-Serial/JTAG. When warm,
  it back-drives up to about 2 mA into the host's VBUS. There is no damage, and USB-A-to-C cables are unaffected.
- **Proposal:** requirement: VBUS stays at or below 0.8 V with no source, at worst-case D9 leakage. Either a bleed sized
  for it or a lower-leakage blocking element meets this. At minimum, document "USB before pack".

### S6. FL2's 320 mA rating — minor

- **Evidence:**
  - **Rating.** DLW21SN670HQ2L: 320 mA, 0.31 Ω per line (Murata table).
  - **Board input current,** iterated through the drops (the Mantis switch and bead, the Schottky, FL2, U2 at 125 mA,
    U9 at 88–91 % efficiency):

    | Case | Input current | FL2 load |
    |---|---|---|
    | Pack 8.4 V, P4 150 mA (the datasheet's dual-core maximum, Table 5-7) | 0.196 A | 61 % |
    | Pack at the Mantis UVLO (6.34 V), P4 150 mA | 0.221 A | 69 % |
    | USB 5.0 V, P4 150 mA | 0.245 A | 77 % |
    | Espressif's 380 mA supply-sizing figure | 0.31–0.48 A | 96–151 % |

  - **Other series elements.** Every other element in the input path covers the 380 mA case.
- **Consequence:** within rating in realistic use, with no margin on USB or if firmware pushes the P4 hard. The board is
  already warm (M3).
- **Proposal (the owner's call):** keep the choke and record a firmware budget (+3V3_MCU at or below about 0.2 A).
  Alternatively, specify a common-mode choke rated at least 0.6 A per line with at most 0.15 Ω per line. The M3 test
  measures the real input current.

### S7. A reversed jumper sends the whole board return into the Mantis GPIO3 clamp — minor

- **Evidence:**
  - **The mapping.** A B-suffix (pad 1 to pad 5) jumper keeps J4.3 on J1.3, so the board is still powered. J4.4 (VSS),
    the board's only supply return, lands on Mantis J1.2 = GNSS_TX = Mantis P4 GPIO3, the host's UART RX. That net has
    no series resistor and no TVS.
  - **Other return paths.** Every other path back to the host goes through 1 kΩ (R8, R9, R11).
  - **What follows.** Board ground rises about 4 V above host ground. The board still runs on the remaining ≈ 4 V and
    draws its full ≈ 0.2 A, and most of that current enters Mantis GPIO3 through its upper clamp into the Mantis 3.3 V
    rail.
  - **cables.md is wrong here.** It says the reversed link "fails silently" with the module ground on the host's TX
    line. On Mantis that pin is the host's RX, and for this board the failure is not silent.
  - **Not a new hazard.** V2 had the same pin functions, but it drew a fraction of the current. This mistake happened
    once before, with the LoRa jumper, on 2026-07-31.
- **Consequence:** probable damage to the Mantis P4's GPIO3 pad; that chip-down QFN-104 cannot be reworked. If the Mantis
  draws less than the injected current, its 3.3 V rail also rises.
- **Proposal:**
  - Correct cables.md (D8).
  - Add a pin-1 mark at J4 pad 1 (49.63, 36.26) so the cable can be checked by eye.
  - To discuss: at least 1 kΩ in series on Mantis J1.2, so a reversed carrier cannot power up; or a board-side gate
    that draws no current from J4.3 unless VSYS − VSS exceeds about 5.5 V (normal ≥ 6 V, reversed 2–4 V).

### S8. U1 (PX1105R) paste is cut to 64 % by a −0.1 paste ratio, with no recorded reason — minor

- **Evidence:**
  - **Where the ratio comes from.** The footprint carries `(solder_paste_margin_ratio -0.1)`, the board's only paste
    override. It has been in the library copy since the #600 import. The fabricated V2 board copy has no ratio and
    printed 100 %.
  - **Apertures.** 0.56 × 0.80 mm on 0.70 × 1.00 mm pads: 64 % coverage, 0.036 mm³ per pad at 80 µm (0.070 mm³ at
    full pad on 100 µm). The convention's standoff estimate is about 26 µm, against 40 µm at full pad.
  - **The rule.** SOLDER-PASTE-CONVENTION §2 says full pad unless a named mechanism applies.
  - **The part.** U1 is the heaviest SMD part (1.7 g) and has castellated pads.
- **Proposal:** drop the ratio in the library footprint and the board copy (full pad gives AR 2.57 at 80 µm), or record
  the mechanism.

### S9. U9 exposed-pad paste: two patterns stacked into one accidental blob — minor

- **Evidence:**
  - **Footprint history.** The footprint is V9/Mantis U18 plus four new 0.65 mm roundrect paste pads. The old
    1.06 mm-square F.Paste polygon was left in.
  - **What plots.** One 1.49 × 1.49 mm aperture (70.6 % of the pad) with notches and no escape channels. The panes
    alone would give 56.7 %; V9's polygon alone gives 40.6 %.
  - **TI's example.** One 1.55 mm square at 85 %, on a 125 µm stencil.
- **Proposal:** keep one pattern: a single square of about 1.45 mm, or the four panes alone. Record the choice.

### S10. U3/U4 (and D6) mask openings equal the copper — minor

- **Evidence:**
  - **Here.** Pad mask margin 0, board pad-to-mask 0, so each opening equals the copper, with 0.150 mm webs between the
    0.25 mm LNA pads.
  - **Infineon.** The TSNP-6-10 recommendation is NSMD: one window around the pad array.
  - **TI.** The DPY0002A recommendation is NSMD, preferred, with up to 0.07 mm margin.
  - **History.** U7, U8 and U10 use the same 1:1 style and are proven on V9 and the mini; U3, U4 and D6 have never been
    built.
- **Proposal:** give U3/U4 a single window around all six pads (or about +0.05 mm per pad), and D6 about +0.05 mm. Or
  record the intended treatment in the fab notes.

### S11. U2's exposed-pad land is larger than the package's pad and printed at 100 % — minor (confirm)

- **Evidence:** the land is 2.41 × 3.05 mm (7.35 mm²) with 1:1 paste. The ADP7142 package drawing (RD-8-1, datasheet
  Rev A) gives the exposed pad as 2.29 × 2.29 mm (5.24 mm²), so the paste is 1.4 × the package pad. The same library
  footprint is on the SAM carrier.
- **Consequence:** excess solder can float or tilt the part, form solder balls, and cause voids under the hottest
  regulator.
- **Proposal:** confirm against ADI's current land recommendation. If the land stays, print no more than about 5.2 mm²,
  for example a 2 × 2 window pane, and fix it in the library.

### S12. U8's footprint (DRL0006A) has no courtyard — minor

- **Evidence:** the footprint has no F.CrtYd, and the project sets missing courtyards to "ignore". The `.kicad_dru`
  fine-pitch rule (`intersectsCourtyard`) therefore cannot include U8. A synthetic courtyard would clear U10 by only
  0.017 mm and C71 by 0.026 mm. V9 and Mantis are the same.
- **Proposal:** add a courtyard to the library footprint. Consider setting missing courtyards to "warning".

### S13. CHIP_PU relies on the RC alone — minor

- **Evidence:**
  - **Espressif** (HWDG §1.3.3): the 10 k / 1 µF RC can fail with frequent power cycling or slow edges; it suggests a
    reset chip with a threshold around 3.0 V. Reset needs CHIP_PU below 0.25 × VDD for at least 1 ms.
  - **Back-feed.** Host lines idling high can feed up to about 2.7 mA each through R9/R11 into an unpowered +3V3_MCU.
    The rail then rests part-way up, and so does CHIP_PU.
  - **Today's exposure.** Current Mantis firmware raises GPS_ACT once at boot and never lowers it
    (flight_computer `main.cpp`, #700), so the host-side case is not exposed. Only the bench case is: this board on USB
    with the Mantis off, feeding up to 2.7 mA into Mantis GPIO3 through R8.
- **Proposal (the owner's call):** a supervisor function that holds CHIP_PU low while +3V3_MCU is under about 3.0 V and
  releases it at least 1 ms after recovery. Keep the Mantis rule of never lowering GPS_ACT while its UART idles high.
  Enable the P4's brown-out detector.

### S14. ERC hygiene — minor

- **U1 pins.** Add no-connect flags on U1 pins 5 (TRIG), 13 (NC) and 14 (VCC_RF). All three are open per the datasheet
  application circuit; these flags account for all 4 ERC errors.
- **Dangling wire.** Delete the 0.2 mm wire left from the old RF_IN stub at sheet (205.1, 101.6).
- **Symbol caches.** Refresh the PX1105R and DLW21 caches; they differ only in the default Footprint field. A blanket
  refresh changes J4 (S2), so do J4 deliberately.

---

## 3. Measure on the first article (risks the files cannot settle)

### M1. P4 clock harmonics inside BDS B1I and GLONASS L1, with the P4 next to the antenna feed — major (risk)

- **Frequency plan (fixed).** The P4 supports only a 40 MHz crystal (HWDG §1.3.5), and its PLLs (CPLL 400, SPLL 480,
  MPLL 500 MHz) are locked to it.
  - **1560.000 MHz** = 39 × 40 = 13 × 120 = 26 × 60 = 78 × 20. This is B1I (centre 1561.098 MHz, main lobe ±2.046 MHz),
    1.098 MHz below centre. It was a known item and is confirmed.
  - **1600.000 MHz** = 40 × 40 = 4 × 400 (CPU) = 20 × 80 (flash) = 10 × 160. This sits in GLONASS L1, between channels
    k = −4 (1599.750) and k = −3 (1600.3125), each with a lobe of ±0.511 MHz. It matters only with SkyTraq's GLONASS
    firmware build. The PX1105R units on the bench report rev 230831, the GPS/Galileo/BDS/NavIC build.
  - **1176 MHz** = 98 × 12 (USB full-speed, bench only) and **1180 MHz** = 59 × 20. Both fall inside L5, but they are
    high harmonics.
  - **Crystal error** (±10 ppm tolerance plus ±10 ppm stability) moves the lines by at most ±31 kHz, so they cannot leave
    those lobes.
  - **The SAWs pass them.** B8389 passband 2 is 1559–1607 MHz with at most 2.5 dB loss, so 1560 and 1600 reach both
    LNAs.
- **Proximity.** The P4 pads (pins 4–9) are 2.6 mm from the antenna feed pad, and XTAL_P/C30 3.8 mm. GNSS_RSTN (P4
  GPIO8) has a via 0.47 mm from the feed copper. There is no shield. Solid In1/In4/In6/B.Cu GND separates layers, but
  the feed pad and the P4 are both on F.Cu, and L1/L2 open a supply-plane path.
- **Test.**
  1. Log per-constellation C/N0 (GSV plus raw 0xE5) on the same sky, first with the P4 held in reset (short TP3 EN to
     TP2 GND).
  2. Log again with the P4 running a worst-case load: both cores at 400 MHz, PSRAM and flash traffic, IMU SPI, all
     UARTs, and USB both in and out.
  3. Repeat at 360 MHz CPU. Compare B1I (and GLONASS, if that build is loaded) against GPS L1.
- **Levers if it shows:**
  - L1 and L2 first.
  - S1 series parts.
  - Lowest workable GPIO drive strength.
  - A 360 MHz CPU clock, whose harmonics miss 1559–1607 MHz; it removes the CPU's 1600 line but not the 40 MHz ×40.
  - Deselect B1I if 1560 is measurable.
  - A shield can over U6/U7/Y1 if a respin allows.

### M2. The patch sits on 25 % of its tuned ground plane — major (risk)

- **Evidence:**
  - **The antenna's reference plane.** GVLB258.A: "tuned and tested on a 70 x 70 mm ground plane"; Taoglas recommends
    at least 70 × 70 mm.
  - **This board.** 35 × 35 mm (1222 mm² against 4900 mm²); the B.Cu ground is 997 mm² in one piece.
  - **Datasheet performance on 70 × 70.** The L5 gain hump is only about ±15 MHz wide to −3 dB, and the best axial ratio
    is 7.5 dB (L5) and 8.3 dB (L1).
  - **Sibling part.** Taoglas's AGVLB256.A drops from 1.73 dBi to −3.14 dBi at L1 without the reference plane.
- **Consequence:** the resonance shifts, peak gain falls by several dB, and the axial ratio worsens. A 10–15 MHz shift
  costs several dB across part of L5. This probably outweighs every other RF item on the board. It is a form-factor
  limit, so the action is to characterise and tune, not redesign.
- **Test:**
  - Measure S11 through a coax pigtail on C19's pads (pad 1 is the feed, pad 2 is GND with a via), with C20 lifted.
  - Compare per-band C/N0 against a reference antenna.
  - If L5 is off: rematch with C19, or ask Taoglas for a tune to a 35 × 35 mm plane (the datasheet offers customer
    tuning), or choose an L1/L5 patch characterised on a plane of 35 × 35 mm or smaller.
  - Keep bench injection at the port at −40 dBm or less.

### M3. The board heats itself: about 1.5 W on 35 × 35 mm — major (risk)

- **Dissipation at 8.4 V:**

  | Source | Dissipation |
  |---|---|
  | U2 | 0.58 W (0.46 W at 7.4 V, 0.17 W on USB) |
  | U1 | 0.38 W |
  | P4 | up to 0.50 W (150 mA at 3.3 V) |
  | U9 | 0.07 W |
  | D8 | 0.05 W |
  | FL2, LNAs, U5 | 0.05 W |
  | **Total** | about 1.6 W (1.4 W with the P4 at 100 mA) |

- **Cooling estimate.** Two faces in still air come to about 28 K/W (±30 %), so the board sits about 45 °C above
  ambient. An enclosed bay is worse, and the patch shades the back.
- **Limits.** PX1105R, P4 (ambient), W25Q128JVYIQ and FL2 are all 85 °C parts, so by this estimate they reach their
  limit near 40 °C ambient. A rocket on a sunny pad is plausibly there. U2's junction runs about 20 °C above the board.
- **Test:** 8.4 V input, representative P4 firmware, board in a closed tube. Read temperatures on the U1 shield, U2 and
  U6, measure the input current (S6), and document the maximum ambient.
- **Levers:** the P4's power budget, and vias under U2 (L7). U2's share follows the pack voltage by design; it cannot
  drop without changing the receiver's supply source, which is the owner's architecture.

### Other first-article checks (small)

- **Crystal frequency.** C30/C31 = 12 pF against the crystal's 10 pF CL assumes 4 pF of stray, which is probably high;
  the crystal may then run 10–30 ppm fast. The P4 has no radio, so this is harmless. Measure it against PPS on GPIO6
  and raise C30/C31 if it is outside ±10 ppm.
- **Receiver logic levels.** The PX1105R guarantees VOH ≥ 2.4 V, while the P4 needs VIH ≥ 0.75 × VDD = 2.475 V. That is
  a paper shortfall only; scope TXD and PPS at the P4 pins.
- **Back-feed.** With the Mantis on USB and GPS_ACT off (a future firmware case), measure where +3V3_MCU settles.
- **USB-C with the pack on:** does the port attach? (S5)

---

## 4. Release package

### D1. FABRICATION-NOTES.md records only the stencil — major (for the order)

- **What is there:** one table row (80 µm stencil) and a pointer to SOLDER-PASTE-CONVENTION.md. Its own header says
  every other Block A item is "still unrecorded". Beetle has A1–A7 and B1–B8 on the same stack; use Beetle as the
  template, because Mantis's notes still describe its old 6-layer stack.
- **Block A values (measured):**
  - JLC08161H-2116 with the layer use: F signal + GND; In1 GND; In2 signal + +3V3_MCU + GND; In3 +3V3 (GNSS); In4 GND;
    In5 signal + GND; In6 GND; B GND + test pads.
  - Copper 35 µm outer, 15.2 µm inner; ENIG; 1.63 mm.
  - 50 Ω GCPW: 0.18 mm track, 0.20 mm gap on F.Cu over In1. 14 nets, 20 mm total. Ask for the impedance service or a
    confirmation.
  - Minimum track 0.09 mm, minimum spacing 0.10 mm.
  - 414 vias, all 0.40/0.30 mm (0.05 mm annular ring).
  - Holes: plated 414 × 0.30, 1 × 1.00 (AE1), 4 × 2.20; non-plated 2 × 0.65 (J5).
  - Hole to hole 0.20 mm (see L15).
  - Via process: record it as Beetle and Mantis do.
- **Block B:**
  - Stencil 80 µm (move it here from Block A).
  - One reflow pass; all SMD parts on the top.
  - AE1 (18 g) adhesive-mounted on the back, its pin hand-soldered from the top after reflow.
  - **U6 must be chip revision v3.x (ESP32-P4NRW32X).** This is the opposite of Mantis note B8, the nearest sibling.
  - U1: MSL4 and its reflow limit (D7).
  - C19 is DNP. Block its apertures or inspect it for a bridge (N-notes).
  - U10/C70/C71 sit at 45°/135°; check them in the placement preview.
  - J4 cable per cables.md.
  - Programming pads on the back.
- **Wrong sentence:** "the patch antenna, the only part on the back" omits the six test pads.

### D2. Revision and title blocks — minor

- **What says V2.** The PCB title block, the bottom silk ("PX1105R V2" via `${REVISION}`) and the gerber project ID. The
  committed V2 is a different 4-layer board.
- **Tags.** Only `…-v1.0.0` and `…-v1.1.0` exist, and v1.1.0's title block reads V1. The repo has no record of a V2
  order. `plot_gerbers.sh` already warns that rev V2 disagrees with the tag.
- **Schematic title blocks.** All four sheets have empty title blocks.
- **Decision (the owner's, from the fab history):**
  - If V2 was ordered: this board is V3. Tag the V2 that was sent, and tag v3.0.0 at release.
  - If V2 was never ordered: keep V2 and tag v2.0.0.

  Record the decision in `docs/board-versioning.md`. Optionally fill the schematic title blocks, as Beetle and the SAM
  carrier do.

### D3. Board-level minimum track and clearance are unset — minor

`min_track_width` and `min_clearance` are 0.0 in `.kicad_pro`; Beetle and Mantis use 0.09/0.09. The board already
complies (minimum track 0.09 mm, minimum spacing 0.1019 mm), but without a floor a later 0.05 mm track or a sub-0.1 mm
footprint clearance passes the release gate. Set 0.09/0.09.

### D4. Netclasses don't match the copper — minor

- **RF class.** 0.34 mm, which a 2-D field solve puts at about 35–36 Ω on this stack. The drawn 0.18 mm is 49–52 Ω
  (0.20 mm gap, with or without mask). The RF clearance of 0.20 is load-bearing: it sets the coplanar gap at every
  refill.
- **Power class.** 0.5 mm, against 0.15–0.4 drawn.
- **Netclass vias.** 0.6/0.3, against 0.4/0.3 drawn.
- **Class membership.** RF membership comes from 14 auto-generated net names (for example `Net-(C20-Pad2)`,
  `Net-(U3-AI)`), so re-annotating C20, FL4 or U3 silently drops that net to Default.
- **Proposal:**
  - RF class 0.18 mm track, 0.20 mm clearance.
  - Netclass via 0.4/0.3.
  - Assign classes through labels or netclass directives rather than auto-generated names.

### D5. What must be committed together — minor

- **Commit together:**
  - the board, `.kicad_pro` and root sheet;
  - `esp32_p4`, `imu` and `usb` sheets;
  - the `.kicad_dru` (DRC reports 428 errors without it);
  - FABRICATION-NOTES.md;
  - `symbols/Custom.kicad_sym` (three new symbols; ESP32-P4NRW32 pin 54, which already matches Mantis's cache);
  - `footprints/PX1105R.kicad_mod`;
  - SOLDER-PASTE-CONVENTION.md;
  - the 4 new footprints, and the 4 new STEP models with their `.build.py`.
- **Keep out:** `legacy/base-station/base-station.kicad_pro`, `tinker-mantis/tinker-mantis.kicad_pcb`,
  `hardware/Claude outputs/` and the two `.log` files.
- **Tooling gaps:**
  - `check_board_parity.py` passes even if the three sheets are missing (47/47 with the root sheet alone).
  - `plot_gerbers.sh`'s dirty check misses untracked files.
- **Library housekeeping:**
  - AE1's library and board copies disagree on the keep-out layers (L2).
  - Five footprints carry two model paths, one of which is always missing.

### D6. BOM — minor

- **Missing fields.** 8 footprints (C7, C8, D2, J4, R3, R8, R9, U1) lack MPN/Mfr until Update PCB.
- **Duplicate lines.** D2 "Green LED" and D5 "Green" (both XL-1005UGC) make two BOM lines for one reel.
- **Duplicate field.** FL2 carries both `Mfr` and `Manufacturer`.
- **No bom.csv.** Mantis, Beetle, the SAM carrier, Tinker-Base and base-station each commit an unpriced one; this board
  doesn't. Priced BOMs are never committed.
- **Placement file.** No repo tool writes it; note `kicad-cli pcb export pos` in Block B.

### D7. Reflow and handling limits are not recorded — minor

- **U1.** PX1105R DS: peak 240 ± 0.5 °C for 25–35 s, above 220 °C for 60–80 s, "should not be exceeded". MSL4: 72 h
  floor life, then bake at 85 °C for 8–12 h.
- **SAWs.** B8389: peak 250 °C; ESD 250 V HBM.
- **The squeeze.** An 8-layer board with six GND planes and a 7.5 mm P4 pad tempts a hotter profile, so the usable peak
  is about 235–240 °C at U1.
- **Proposal:** an assembly block with that window, a thermocouple check on the first article, the MSL4/bake rule and
  ESD handling for the SAWs.

### D8. Documentation drift — minor

- **legacy/README.md.** Still reads "GNSS carrier, PX1105R, external antenna", tree V2, "superseded by the SAM-M10Q
  carrier" and "carries no firmware". A P4 now needs firmware; `tinkerrocket-idf` has no project for it.
- **hardware/README.md.** Still lists the gone `2337019-1` part under known issues.
- **board-versioning.md.** Has no boot-NOR row for U7 (16 MB), although over-declaring flash size is fatal. Tag list
  and legacy status are stale.
- **cables.md.**
  - It says "both GNSS carriers' J3", but this board's header is J4.
  - The reversed-cable paragraph is wrong for this board (S7).
- **3dmodels/README.md.** Says every model comes from the manufacturers; the four new ones are generated by CadQuery
  scripts.

---

## 5. Firmware and system (no P4 firmware exists yet)

### F1. The host link is now P4-to-P4, and the Mantis speaks only UBX — note (system)

- **Evidence:**
  - **Wiring.** Every J4 signal goes only to the P4, and the receiver's UART goes only to the P4. The GNSS_* nets have
    two nodes each.
  - **Host driver.** `board_v9.h` (Mantis V9/V10): "the board header hard-selects the UBlox driver". The repo has no
    SkyTraq driver.
  - **Receiver protocols.** The PX1105R speaks NMEA, SkyTraq binary and RTCM.
- **Consequence:** with a blank P4 the host hears nothing, and even a transparent bridge would not be parsed by today's
  Mantis driver.
- **Plan:**
  1. The first P4 image is a bridge: USB-Serial/JTAG ↔ receiver UART, and host UART ↔ receiver UART.
  2. Then choose: the P4 emits UBX, or the Mantis gets a driver for this carrier.

### F2. Constraints this schematic puts on P4 firmware

- **Build and silicon.** Build for silicon v3.x; Mantis is a v1.3 build, so its configuration cannot be reused.
- **eFuses never to burn:**
  - `EFUSE_0PXA_TIEH_SEL_0`: flash IO would drop to 1.8 V against a 3.3 V flash.
  - `EFUSE_JTAG_SEL_ENABLE` and `EFUSE_DIS_USB_JTAG`: JTAG would move onto GPIO2–5, the receiver UART pins.
  - Anything that disables USB-Serial/JTAG, the only programming path once the board is mounted.
- **UART map.** All fit at once:
  - Host on UART1 (GPIO11 RX, GPIO10 TX).
  - Receiver on another UART (GPIO3 RX, GPIO4 TX).
  - RTCM relay on a third (GPIO12 RX from host TX2, GPIO2 TX to RXD2).
  - Console on UART0 (GPIO37/38) or USB. Keep UART0 silent in flight (L6).
- **RSTN (GPIO8).** Open-drain only; R12 is the pull-up. It is high-impedance after reset, so a P4 reboot does not
  reset the receiver.
- **PPS (GPIO6).** Use hardware capture.
- **At boot.** Enable the GPIO4/GPIO10 pull-ups first (S3), and the brown-out detector (S13).
- **CPU clock.** Prefer 360 MHz if M1 shows a GLONASS problem.
- **Keep GPIO24/25 and the USB PHY alone.**
- **IMU:**
  - Set `IO_PAD_STRENGTH` = 00 (datasheet: lowest strength recommended for VDDIO ≥ 3.0 V). The fleet driver never
    writes `PIN_CTRL`.
  - Enable the SDO pull-up (or the P4's pull on GPIO51) so MISO doesn't float between transfers.

### F3. Programming procedure (no button)

- **Download mode:** power from USB-C, short TP4 (BOOT) to TP2 (GND), pulse TP3 (EN) to TP2, then flash over USB.
  R16 holds GPIO36 high, so the TP4 short alone selects download mode.
- **TP1 (3V3) is not a supply input.** Driving it back-feeds VIN through U9's body diode and starts U2 and the receiver
  in dropout.
- **Once mounted:** only USB auto-download works, and only while the application keeps USB-Serial/JTAG alive.
- **Test pad labels.** The six test pads have no silk labels, so bring-up needs the PCB file open. Label them.

### F4. Every power cycle is a cold start

- **Evidence:** V_BCKP is tied to VCC. PX1105R DS: with both removed, "all user configuration set is lost", and TTFF is
  1 s hot against 29 s cold. The board has about 0.4 ms of hold-up, so any supply gap longer than that (including a pack
  bounce in flight) means a cold start. Configuration is re-sent at every boot anyway.
- **Consideration only:** a Schottky and resistor from +3V3 into a reservoir on V_BCKP (13 µA with VCC off). 0.1 F
  holds about 3.6 h; a small rechargeable cell holds days.

### F5. J4.5 direction and PPS

- **Direction.** J4.5 is an input here (host TX2 → R11 → GPIO12). The SAM carrier puts PPS on the same pin as an
  output. `board_v9.h` records pin 5 as carrier-dependent and keeps Mantis GPIO2 unused.
- **Option.** This board could mirror PPS out through GPIO12 → R11 → J4.5 in firmware alone, SAM-carrier style. R11
  limits any contention to 3.3 mA.
- **To do.** Choose the direction for this carrier and add it to `board_v9.h`'s carrier table.

---

## 6. Notes (no action needed)

- **N1 — Filter-first NF.** The front end computes to NF 2.9 dB at L1 and 2.0 dB at L5 (worst case 4.2 / 3.2 dB),
  against the receiver's "< 2 dB" antenna guidance. Gain is 27 dB (L1) and 33 dB (L5), at most 35.4 dB, under the 40 dB
  limit. LNA-first would give about 1.1 dB, but the pre-filter buys ≥ 20 dB at 600–1112 MHz against the nearby LoRa
  transmitter. That trade is the owner's and is defensible.
- **N2 — LNA outside its datasheet band.** The BGA855N6 datasheet covers 1164–1300 MHz only. L1 operation rests on
  AN596 Option B (15.7 dB, NF 0.70 at 1575 MHz), which the design copies exactly, so there are no guaranteed limits at
  L1.
- **N3 — USB routing.** D+/D− are not routed as a pair, with 4/5 vias, and the 22 Ω resistors sit 13–16 mm from the P4.
  Irrelevant at 12 Mbit/s. The order connector → ESD → resistor → P4 is right.
- **N4 — Crystal spacing.** Y1 is 1.2 mm body-to-body from the P4, against Espressif's "at least 4.5 mm". The short
  traces (2.1 / 3.4 mm, no vias) are the better trade on a GNSS board; keep it. Keep SENS_SCLK at least 0.5 mm from
  XTAL_N if the area is reworked.
- **N5 — Mounting hardware.** An M2 pan head (4 mm) touches only mask-covered pour. A 5 mm washer on top reaches the
  FB_DCDC track at H2 and C36 at H4, so specify top-side hardware of 4.6 mm or less. The holes are netless on purpose:
  screws cannot bypass FL2.
- **N6 — Heights.** The patch is 8.1 mm (plus adhesive) on the back; J4 is about 4.25 mm, J5 3.26 mm and U1 2.9 mm on
  top. The envelope is about 14.5 mm. The test pads are 2.6–2.8 mm outside the patch wall, reachable with vertical
  probes.
- **N7 — VBUS capacitance.** About 20–25 µF effective sits behind D9, about twice the USB 2.0 attach limit (10 µF /
  50 µC). Hot-plug inrush is about 2×; acceptable for a bench port.
- **N8 — IMU orientation.** The IMU at 45° has the same axis relation to the silk X/Y marker as Mantis and Beetle, so
  Mantis's 45° handling carries over. If the board mounts with the patch looking along the rocket axis, thrust falls on
  sensor Z and the low-g range loses its √2 headroom (the high-g range still covers boost).
- **N9 — Mask webs.** 0.06 mm between the P4's pins, 0.073 mm at U9's corners, 0.10 mm at J5, all inherited from built
  boards (V9, mini). Expect the fab to open the P4 rows as one window.
- **N10 — P4 exposed-pad paste.** 40.4 % (nine panes), identical to V9. On 80 µm that is 20 % less solder than V9 got if
  V9's stencil was 100 µm. V9's foil is still unrecorded.
- **N11 — D8/D9 land.** Pads are 0.45 mm wide against Toshiba's 0.9 mm reference land (the same footprint as V9). Widen
  them the next time the library footprint is edited.
- **N12 — Rework access.** Rework is tight: neighbours sit 0.39–0.70 mm from the fine-pitch parts, and AE1 covers the
  back of the RF chain, U1 and U6. Consistent with #906, "rework cannot be assumed".
- **N13 — Panel edge.** J5 overhangs the edge by about 0.14–0.19 mm, which is correct for plug seating, so keep panel
  rails off that edge. MLCCs sit 0.52–0.82 mm from the right and top edges; keep breakaway tabs away from them.
- **N14 — Smaller items:**
  - C19 (DNP) still gets paste apertures (D1).
  - AE1's inner 1.2 mm feed rings leave 0.1 mm annular rings.
  - Zone names don't match their layers: "In2 +3V3 (GNSS)" is on In3.
  - The IMU axis text is 0.75 mm, against the 0.8 mm rule.
  - FL2 carries unbalanced DC on USB power, which is harmless beyond S6.
  - L10 is 2.2 µH (TI's minimum at the 1.25 MHz setting), so U9 always runs in power-save mode; stability is fine.
  - Procurement: 15 of 42 MPNs have never been bought by this project. The 22 µF 0805 part is recorded as NRND in the
    SAM carrier's BOM, and the 1 µF reel is a 10 V part where the fleet stocks 16 V.

---

## 7. Checked and fine

**Power**

- **U2 wiring** matches ADI Table 4: VOUT/SENSE tied, EN on VIN, SS 2.2 nF, EP to GND; the BOM part is the fixed
  3.3 V variant.
- **U2 operating points:**
  - Soft start 1.64 ms.
  - Inrush 68 mA + 115 mA, under the 220 mA minimum current limit.
  - Dropout headroom 1.3 V on USB.
  - Every capacitor inside ADI's window; C3 is 0.6 mm from VIN.
- **FL3 with C7/C27** resonates near 50 kHz at Q ≈ 2. A 20 mA step gives 12–18 mV, inside the receiver's 50 mVpp.
- **U5 (LP5907):** pinout, EN, C27/C28 (about 0.9 µF effective against a 0.7 µF minimum), 14 mW.
- **U9 straps:** EN on VIN; DEF to GND gives 3.3 V; FSW to VOS gives 1.25 MHz; PG open (allowed).
- **U9 operating points:** soft start 5.0 ms; VIN absolute maximum 20 V against the 17 V TVS clamp.
- **U9 output filter:** 2.2 µH with about 83 µF nominal (34–46 µF effective) is inside TI Table 9-2.
- **U8 (TLV62569DRL):**
  - pin 6 is NC and legal to ground;
  - EN_DCDC has no pull-down, as in Espressif Fig. 5;
  - 499 k / 499 k / 22 pF gives 1.200 V (1.164–1.236 V), within the P4's trim reach;
  - 30.4 µF output capacitance, near Espressif's 32.4 µF;
  - C38 is 1.3 mm from VIN, and U8 is 3.7 mm from the P4;
  - the FB trace is shielded on In5.
- **P4 LDO outputs:** VO1–VO4 each have 1 µF within 1.1–1.6 mm, plus 0.1 + 1 µF at the flash and PSRAM IO pins.
- **D4:** polarity right, 10 V standoff against VSYS of at most 8.15 V, on the connector side of FL2.
- **D8/D9:** polarity right; the pack always wins; the handover is seamless.
- **Input paths:** VSYS/+BATT/VSS 0.5 mm (up to 48 mΩ); VBUS 0.4 mm.
- **+3V3_MCU pour:** one piece, 2.6–10.8 mΩ from the buck to the loads.
- **Sequencing:** +3V3 up at 1.64 ms, +3V3_MCU at 5.05 ms, P4 out of reset about 11.5 ms later (Espressif needs 50 µs).
- **Pack range:** 2S, 6.4–8.4 V; the Mantis eFuse has UVLO 6.34 V and OVLO 16.9 V.

**RF**

- **LNA DC paths:**
  - AO is internally DC-blocked with an on-chip collector choke (datasheet Fig. 1), so the output-side shunts only hold
    0 V nodes.
  - AI has no internal block (AN596; absolute maximum 0.9 V), and C22/C24 provide it.
- **SAW ports** all sit at 0 V, as the B8389 requires ("blocking capacitors are mandatory").
- **Matching:**
  - L1/L2/L5/L6 are the SAW's specified 5.1 nH shunts.
  - The high-pass corner is about 780 MHz: 2.4 dB at 915 MHz, 6.3 dB at 433 MHz.
  - D6 adds 0.15 pF (≤ 0.01 dB).
  - The 33 pF blocks are within 1.6 Ω of series resonance in band.
- **LNA supply:** C23/C25 1 nF meet AN596's "≥ 1 nF", backed by C28. PON is on VCC.
- **RF_IN:** C26 keeps the receiver's 3.3 V bias off U4 AO (absolute maximum 2.1 V). VCC_RF is open, as in the
  datasheet circuit.
- **Stability:** k > 1 from 20 MHz to 10 GHz (datasheet). Board isolation between the chain's ends is 64–77 dB against
  at most 35 dB of loop gain.
- **Routing:** 14 nets, all 0.18 mm on F.Cu, 20.0 mm, no vias, 45° bends.
- **Ground under the RF:** In1 is solid under all RF except the designed feed keep-out and L5. Fencing is 1.5 vias/mm,
  every RF point is within 1.43 mm of a GND via, and there are 19 vias around the feed.
- **SAW and inductor grounds:** SAW GND pads have vias within 0.11–0.17 mm; the shunt inductors have vias in pad.
- **Feed surroundings:** the patch is centred. The nearest non-GND copper outside the keep-outs is at least 2.3 mm from
  the feed on every layer. The switch nodes are 12.8 and 15.3 mm from the feed and 17–29 mm from the LNAs, with both
  inductors outside the patch outline.

**P4, flash, IMU, USB**

- **U6 pins:** all 105 checked against the datasheet table. 16 GPIOs are used, and every unused pin has a no-connect
  flag. Exposed pad on GND with 27 vias.
- **Power domains:** the 3.3 V IO banks are on +3V3_MCU; flash IO comes from VO1 and PSRAM from VO2.
- **GPIO choice:** no interface pin is a strapping, flash or USB pin. GPIO2–5 are pad-JTAG pins, but JTAG defaults to
  USB.
- **Flash:**
  - The WLCSP ball map matches datasheet §3.8/§10.7 and is not mirrored.
  - The "Q" part fixes QE = 1, so /WP and /HOLD need no pull-ups.
  - The CS pull-up is present, and C32 is 1.7 mm from VCC.
  - Flash on +3V3_MCU is correct with default eFuses: VO1 passes VDD_LDO through about 3 Ω in 3.3 V mode.
- **Straps:** normal boot by default. R15 on GPIO35 follows HWDG; R16 makes download mode reachable.
- **CHIP_PU timing:** 10 k / 1 µF crosses its threshold 6.7–11.5 ms after the rail settles.
- **USB:** complete: 22 Ω series resistors, ESD array, internal D+ pull-up, polarity right (GPIO24 → D−, GPIO25 → D+),
  5.11 k Rd on both CC pins, both rows wired, and no DP pull-down needed on v3.x.
- **IMU (ISM6HG256X):** every pin matches the datasheet table: SDx/SCx to GND, the aux pins open, CS pull-up,
  push-pull INTs. VDD and VDDIO share a rail (no sequencing rule). C70/C71 sit 1.1 mm from pins 8/5, each with its own
  GND via. The footprint and 45° orientation are identical to Mantis and Beetle.
- **Host link:** J4 ↔ Mantis J1 pin for pin with an A-type cable. Names follow the Mantis convention (HOST_* from the
  host's side, GNSS_* from the receiver's). Levels are compatible. The 1 kΩ resistors bound back-feed, contention and a
  pin 2–3 bridge from +BATT to 2.7–8.4 mA.
- **PX1105R pins:** all per the datasheet: TXD → GPIO3 (input at reset), RXD/RXD2 driven only after the receiver is
  powered, PPS → GPIO6 + LED (0.09 mA), RSTN pull-up + GPIO8 (high-Z at reset), pins 5/7/13/14/16/17 open.
- **U1 decoupling and ground:** C8 sits 0.40 mm from VCC pin 8, and every U1 GND pin has a GND via within 0.3 mm.

**Layout, planes, assembly**

- **Planes:**
  - In1, In4, In6 and B.Cu GND are each one piece.
  - No zone island floats: the 131 GND pieces on In3 and 54 on In2 are via donuts inside the supply pours.
  - In5 has GND on both sides.
  - No signal crosses a plane void for more than about 0.3–0.5 mm outside its own antipads.
- **QSPI:** lengths 2.8–8.9 mm, CK-to-data mismatch at most 3.5 mm (23 ps).
- **Copper balance** across the 8 layers is 73–89 %, so there is no warp concern at 35 mm.
- **Edge clearances:** pours are 0.501 mm from the edge, tracks at least 0.625 mm and vias at least 0.595 mm. No pad is
  closer than 0.5 mm.
- **Paste:** 455 apertures, none off-pad and none bridging. The minimum area ratio is 0.78 at 80 µm; the notes'
  numbers reproduce (U6 is really 0.91, not 0.88). B.Paste is empty.
- **Courtyards:** no overlaps; closest pad-to-pad between parts is 0.293 mm.
- **3D models:** all resolve.
- **Stackup:** matches JLC08161H-2116 exactly, the same as Beetle A1.

---

## 8. Not covered, or not checkable from the files

- Spur amplitudes, antenna S11 and gain on this board, rail noise at L-band, real P4 current, and board temperature.
  These are what M1–M3 measure.
- ADP7142: the current released datasheet did not load (analog.com timed out). Rev A (package) and Rev PrD (thermal)
  were used.
- Manufacturer datasheets were unavailable for SP0503BAHT (403 everywhere), SMF10A, VLS3012CX, BLM18PG and USB4110. For
  these, the review relied on footprints and pinouts already built on V9, the mini or the SAM carrier.
- JLC's 8-layer capabilities: no capability document is in the repo, so every fab number above is marked "confirm".
- The host enclosure, the mounting of this board in an airframe, and its vibration behaviour with the 18 g patch.
- Whether V2 was ever ordered (D2).
- Firmware: none exists for this board, so F1–F5 are requirements, not reviews.
