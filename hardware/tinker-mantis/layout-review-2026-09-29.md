# Tinker-Mantis — layout review, 2026-09-29

**Board:** `hardware/tinker-mantis`, rev **V10** (title block), 8 layers, never fabricated. This review covers the owner's
working copy as saved at 15:47 on 2026-09-29, after placement and routing were finished.
- PCB md5 `dcc32030`. The schematics, project and libraries are identical to `main` at `693570b8`.
- The only difference from the board merged in #1549 (`0a5ea144`) is about 700 re-routed In2 segments on ~30 nets, one
  `PYRO3_FIRE` via nudged to (89.73, 147.17), and one GND via added at (90.64, 162.93). No footprint moved.
- **Nothing in the board was edited for this review.** Every fix below is a proposal with coordinates, for the owner to
  make in KiCad.

**State of the files:**
- **DRC** (kicad-cli, all severities, schematic parity): 1 error, 46 warnings, 0 unconnected, 0 parity. The error is the
  deliberate C15/J1 courtyard overlap (0.046 × 6.37 mm; the bodies are 0.70 mm apart).
- **Fills are current.** A kicad-cli refill of a project-mirror copy is geometrically identical to the stored fills
  (0 mm² difference on every zone and layer).
- **Footprint links:** `tools/check_board_parity.py` passes, 276 of 276 footprints linked by path.
- **BOM:** `bom.csv`, the board and the netlist agree on all 264 references, values and MPNs. C12, C93, R74, R75 and R76
  are DNP in all three.
- **`.gbrjob`** (as plotted from the board): 8 layers, 1.6301 mm, ENIG, the JLC08161H-2116 stack.

**Method:**
1. Snapshot the saved project into a scratch mirror; export netlist, ERC, DRC with parity, and every layer as an image.
2. Export exact copper (pad polygons per layer, zone fills, tracks, vias) and measure it with shapely: cross-sections,
   gaps, parallel runs, return-via distances, copper under parts.
3. Run six specialist passes against the datasheets: power conversion; battery, eFuse and pyro; processors, memories and
   clocks; signals, USB and RF; sensors, mechanics and thermal; fabrication and assembly.
4. Re-check every finding kept here against the files. Every proposed via site was checked on all 8 layers: at least
   0.10 mm to other-net pads, tracks and vias; at least 0.20 mm hole to hole; lands on GND copper; inner planes and inner
   pours take an ordinary antipad; no outer pyro or power pour is holed.

**Datasheets read:** Espressif ESP32-P4 and ESP32-S3 Hardware Design Guidelines (HDG); TI TPS62152, TPS61094, TLV62569,
TPS2121, TPS22810, TPS22811, TPS25982 (TPS259824), INA230, FSUSB63; AOS AONR21321 and AON7534; Winsok WSD20L50DN33;
Abracon AANI-CH-0070 rev A; Winbond W25Q128JV; GigaDevice GD5F2GQ5UE; Bosch BMP581; QST QMC5883P; GCT USB4110; JST VH.
ST's IMU land-pattern notes would not download; the IMU item uses ST's community PCB-design article instead.

## Updates since the review (2026-09-30)

- **P1 is downgraded from MAJOR to a note.** The MAJOR grade followed Espressif's generic 20 mil wording, not the
  current.
  - The P4 core rail draws roughly 0.1–0.3 A, a few hundred mA at peak.
  - The three core pins share the load through the package, so the routed feeds give about 3 mV of drop at 0.3 A.
  - The fabbed V9, which has flown, used thinner and longer traces.
  - The note keeps one suggestion: a 100 nF at pins 26 and 76. See P1 under *Minor — layout*.
- **F1, F2 and F9 are done:**
  - `FABRICATION-NOTES.md` is rewritten for V10: block A for the fab, block B for assembly, including the 0.08 mm foil,
    the reflow order, C130 and the mounting hardware.
  - `SOLDER-PASTE-CONVENTION.md` has a `tinker-mantis` 80 µm row.
  - The board's `User.Drawings` text now holds block A (board md5 `a8dd953f`; only that text changed).
- **F3 was done by the owner** on 2026-09-29 at 21:04: U18, U27, U29 and U30 were updated from the library.
- **The owner's 2026-09-29 20:48 save also changed:**
  - VDD3P3 widened to 0.3 mm (M1);
  - 22 GND vias stitching the antenna cutout (R1);
  - the C53 GND via (P3a) and the moved V_SCAP_ADC via (P3b);
  - an In2 ribbon re-route (S2);
  - ESP_VDD_HP widened to 0.3–0.4 mm.
  
  Those items have not been re-checked against their findings yet.
- **Second pass, 2026-09-30**, applied to the owner's board, which is now md5 `bb98fb66`:
  - M1 verified done by the owner. P3a and P3b are done; P3c is accepted as-is.
  - Six S1 return vias added.
  - X4: the U4 via moved into pad 9.
  - F8: FID3 moved.
  - F6: thermal relief on R41, R47 and R22.
  - C27 and C145 swapped.
  - The SERVO_ACT stub deleted.
  - Partly done: F7 (the OC_ARM_EN via), F5 (the orphan '+' marks) and F4 (U2's pin-1 dot, fixed in the library too).
  - Not feasible as proposed: X3's via move (J5 pads under the IMU) and F7's VDDO_PSRAM nudge.
  - DRC: 1 error, 34 warnings, 0 unconnected, 0 parity.
  - The remaining items are listed as decisions on #1556.
- **Third pass, 2026-09-30**, on the owner's answers to the #1556 decision list. The board is now md5 `516a8f8a`.
  - **X3:** the via under U2 is deleted. The pad 5 → 8 link waits until the area is next touched.
  - **M2:** INT1 re-routed, since XTAL_P's jog can't move (it hits C63.1).
    - INT1 within 0.30 mm of XTAL_N: 4.50 → 0.07 mm.
    - INT1 within 0.50 mm of XTAL_P: 4.46 → 0.01 mm.
    - The INT1 via stays 0.10 mm from XTAL_N at one point.
  - **F4:** pin-1 dots on the board copies of U17, U15, U23, U1, U6, U7, U8, U10, U9, J1 and J3. U26's and U28's dots
    moved. J3's dot sits at the housing's pin-1 corner, because its pads run under the housing.
  - **F5:** the "F" moved clear of C44.
  - **Accepted:** F7 (VDDO_PSRAM), R1 (the rest) and C17. The C17 note is corrected in the power-parity note.
  - **M3:** the waiver is re-affirmed at 3 + 1 (WORKLIST M-17).
  - **Settings and notes:** the four DRC severities are raised, there is still no `.kicad_dru`, and FAB A9 (the top
    edge) is added.
  - **Top edge:** the measured distances correct the note under X3: R83 is 0.25 mm from the edge and U3 0.485 mm.
  - **Skipped for V10:** P1's cap, the P4-half caps and the NAND bulk cap.
  - **Closed as noted:** the informational notes.
  - DRC: 1 error, 46 warnings, 0 unconnected, 0 parity. The 13 dotted footprints now report `lib_footprint_mismatch`,
    and the raised severities add 4 warnings.
  - The fab package is re-plotted.
  - **The owner's calls on the last three:**
    - S1's "O" legend: the owner first chose to leave it, then moved it to (92.64, 142.29), below R134 and beside
      R135. That spot is clear of pads and silk, but it is 3.8 mm from the switch end. The move clears the "O"'s six
      silk warnings, so DRC is now 1 error, 40 warnings, 0 unconnected, 0 parity.
    - S3 is accepted as it is.
    - S2 is handled in firmware. There is no board change; the re-route waits for the next layout. The S3 I2C slave
      gets the master's 7-cycle glitch filter, set by register because the IDF slave driver has none. The I2S
      master's four outputs drop to the weakest drive. A bench soak (SCL at the S3 with the stream running, and a
      count of the link's CRC rejects) is still owed.
- **Tracking:** everything open is in #1556. D1 is split into #1553 (firmware trip) and #1554 (fuses); P2 is #1555.

## Verdict

The layout is in good shape. Every net is connected, the fills are current, the footprints are linked and the BOM agrees
with the netlist. The safety-critical geometry checks out:
- Q11's reverse-polarity orientation is right.
- All four pyro neck floors are met.
- The fire path's vias share the current.
- The switch nodes sit on F.Cu over solid In1.
- In1, In3, In4 and In6 are each one unbroken plane.
- The antenna window is clean on all 8 layers.
- There are no hairline joints.

Most of issue #1365's old problems are gone: the Q3–Q6 shorts, the listed crosstalk pairs and the buck input cap on the
back (details in the #1365 table below). So is the 0.66 mm In5 VBATT neck at U28 noted during routing.

Before ordering, fix two things. One design question also needs a decision.

| # | Severity | Item |
|---|---|---|
| **F1** | MAJOR | A 0.10 mm stencil fails both WLCSP boot flashes; order 0.08 mm, as the sibling boards do. |
| **F2** | MAJOR | `FABRICATION-NOTES.md` and the board's `User.Drawings` text still describe the V9. |
| **D1** | for discussion | A shorted e-match during the 200 ms fire has no current limit, and the fault current runs through the board's own supply path. The Beetle is the same. |

Then 13 minor layout items (P2, P3, R1, M1–M3, S1–S3, X1–X4), 7 minor fab/assembly items (F3–F9) and notes. P1, the
P4 core supply, was first graded MAJOR here and is now a note; see *Updates since the review*. The cheapest high-value
ones are:
- F3: update four footprints from the library.
- M1: widen the S3 RF supply trace.
- X4: move one via under the barometer.
- S1: add the six return vias.
- P2: move the pyro gate pull-ups to the FETs.

---

## Fix before ordering

### F1 [MAJOR, done 2026-09-30] A 0.10 mm stencil fails both WLCSP boot flashes: order 0.08 mm

**Where:** U13 (S3 NOR) and U16 (P4 NOR), 24 apertures each; `FABRICATION-NOTES.md` B0.

**Evidence:** From the plotted F.Paste and B.Paste (1017 apertures), the area ratio is AR = area / (perimeter × foil).
- U13/U16 ball apertures are 0.254 mm circles: AR 0.635 at 0.10 mm, 0.794 at 0.08 mm. The IPC-7525 floor is 0.66, so 48
  apertures fall below it.
- The next worst clear the floor at either foil:

  | Aperture | AR at 0.10 mm | AR at 0.08 mm |
  |---|---|---|
  | U23 EP corner windows | 0.663 | 0.829 |
  | U17 0.18 × 0.65 pins | 0.731 | 0.914 |
  | U22 and U4 | 0.750 | 0.938 |

- `SOLDER-PASTE-CONVENTION.md` already puts the Beetle, the Base and the LoRa board on 80 µm for this WLCSP ("AR 0.64
  at 100 µm"). The Mantis is not in the table.
- B0 is also stale in its own right: it lists Q9 (gone), gives U23 as AR 0.70 (it is 0.663) and says every aperture
  clears.

**Why it matters:** these are both processors' boot flashes. Poor paste release on a 0.25 mm ball land means an open
joint under the chip that can't be inspected.

**Proposed fix:** a flat 0.08 mm foil for both sides. The worst aperture is then AR 0.794 and everything else is ≥ 0.83.
Rewrite B0 with these numbers and add a `tinker-mantis` row to the convention table.

### F2 [MAJOR, done 2026-09-30] The fab notes and the board's User.Drawings text still describe the V9

**Where:** `FABRICATION-NOTES.md` and the text block at (127.19, 106.75) on `User.Drawings`, which is Block A verbatim.

**Evidence** — each statement below is wrong for this board:
- **Header and A1:** "6 LAYER, 1.546 mm STACK", "JLC06161H-3313", and In1/In4 GND with In3 routing. This board is 8
  layers, 1.630 mm, JLC08161H-2116, with In1/In6 GND, In3 +3V3 and In4 V_MCU_SWTCH.
- **A2:** the fine-pitch list gives U21 as 0.4 mm (it is 0.5 mm). It omits the 0.5 mm WLCSP flashes U13/U16 and U1
  (0.4 mm).
- **A4:** "0.18 mm ≈ 48.7 Ω on 0.0994 mm 3313". The dielectric is now 0.1164 mm 2116. A field solve of the feed as
  built (grounded CPW, 0.13 mm gaps) gives about 47 Ω, still close to 50.
- **A7:** the THT parts are J8 and C130. J2 is SMT, and C12 is now a DNP 0402.
- **B0:** the foil (F1).
- **B1–B4:** the C12 can, interposer and RTV instructions are obsolete. They also cite C56 and a SOIC-8 U13.
- **C130 has no note** (2 F radial, B side, THT, pad 1 = V_SCAP +, about 1 g on two leads, 12 mm tall; stake it).
- **B9:** the P4 strap lines are J3.11 = EXP_09, J3.12 = EXP_10 and J3.13 = EXP_11, not pins 12/11/10.
- **"Still open":**
  - The U19 DVDT mask bridge is gone.
  - The VBATT corridor uses V9 coordinates.
  - There are 4 fiducials.
  - The C12 "+" is now an orphan mark (F5).
- **Plotting:** the board's saved plot set omits User.Drawings, so a `--board-plot-params` plot never ships the block.
  But `tools/plot_gerbers.sh` passes no layer list, and it was verified to emit `User_Drawings.gbr` carrying the 6-layer
  text.

**Why it matters:** an 8-layer order could go out with a 6-layer stack call-out, a failing foil and can instructions for
a part that no longer exists.

**Proposed fix:** rewrite Blocks A and B against this board, using the figures here plus F9's assembly items, then update
or delete the User.Drawings block.

---

## For discussion (design level)

### D1 A shorted e-match or harness during the 200 ms fire has no current limit, and the fault current runs through Q11 and R72

**The fire loop:** J8 → Q11 → R72 → B.Cu VBAT_CON trunk → pyro FET (U6/U7/U8/U10) → J2 → e-match → PYRO_GND → U9 → GND →
J8.1. VBAT_CON is upstream of the eFuse. The V9's series limit, the 150 Ω charge resistor and its store, left with the
pack-fire rework, and nothing replaced it.

**Estimate** (DC solve on the exact copper, plus datasheet on-resistances):

| Element | Resistance |
|---|---|
| Board copper, per channel | 8.8–9.9 mΩ |
| R72 | 2 mΩ |
| Q11 | 16.5 mΩ max (29.5 mΩ at −4.5 V) |
| Pyro FET | 11 mΩ max |
| U9 | 5–8.5 mΩ |
| Contacts (J8 + J2), assumed | ~10 mΩ |
| 2S pack and leads, assumed | 20–60 mΩ |
| **Loop** | **73–130 mΩ** |

- A short at the terminal therefore draws about **60–115 A** at 8.4 V. A normal fire is 6.5–9.6 A.
- That exceeds the single-pulse ratings: Q11 IDM −66 A, pyro FET IDM −70 A. Both are short pulses at TJ(max); a
  200 ms pulse is far worse. Expect the MOSFETs to fail first, within about a millisecond.
- The B.Cu trunk's narrowest stretch (2.2–2.4 mm × 35 µm at y 136.3–138.2) reaches +250 K adiabatically in roughly
  20–60 ms. Its 200 ms fusing current is about 50 A.

**Why it matters:**
- Q11 and R72 also carry the whole board's supply. If either fails open, the board is left on the hold-up supercap for a
  few seconds and every later channel is lost, including the main.
- A pyro FET that fails short leaves its channel live until U9 disarms.
- A shorted harness reads like a good e-match on the continuity sense (EXT ≈ 0.14 V either way), so nothing shows it
  before the fire.
- The Tinker-Beetle has the identical topology (pyro FET sources on VBAT_CON, the same Q11/R72/U19 chain; checked in its
  netlist).

**Options:**
1. **Firmware on existing hardware.**
   - Use the INA230's shunt over-limit alert (SOL; ±81.92 mV = ±41 A full scale; 140 µs conversion). INA_ALERT already
     goes to the S3, which on an alert drops OC_ARM_EN and so turns off U9.
   - U9's gate discharges through R22 = 100 kΩ, so it lingers 0.2–0.5 ms in its linear region; a smaller R22 or an active
     pull-down would shorten that.
   - Also trim PYRO_FIRE_DURATION_MS to the e-match's all-fire time.
2. **Hardware:**
   - a fast fuse or PTC per channel at J2;
   - 0.2–0.3 Ω of pulse-rated series resistance per channel, which cuts a terminal short to 20–30 A and normal fire
     current by about 20%;
   - current-limited high-side switches.
3. Accept and document.

**Confidence:** medium. The copper numbers are solid; the pack and contact resistances and the MOSFETs' transient curves
are assumptions.

---

## Minor — layout

### P1 [NOTE — downgraded from MAJOR on 2026-09-30] The P4 core supply is a thin chain; a 100 nF at pins 26 and 76 is cheap insurance

**Where:** `ESP_VDD_HP`, from the U20/L8 output pour (F.Cu, 91.6–93.5 × 104.4–108.4) to U17 pins 76 (88.51, 108.08),
91 (83.76, 106.83) and 26 (78.66, 116.13).

**Why it was downgraded.** The review first graded this MAJOR against the wording of the P4 HDG §1.4.1: VDD_HP traces
≥ 20 mil, star routing, a 0.1 µF per pin. That guideline is generic, and at the rail's actual current the width is not
a problem.
- **The current is small.**
  - The core rail draws roughly 0.1–0.3 A in operation, and a few hundred mA at peak. P4 datasheet Table 5-7 gives
    35–123 mA at 3.3 V for the whole chip at 360 MHz, converted here through the buck.
  - Espressif's whole-chip supply budget (≥ 380 mA at 3.3 V, about 1.25 W) keeps the core well under 1 A even at the
    extreme.
  - U20 is a 2 A part.
- **The drop is negligible.** The three core pins are tied together inside the package, so their feeds act in parallel.
  - As routed on 2026-09-29, the feeds are roughly 15 mΩ to pin 76, 40 mΩ to pin 91 and 110–130 mΩ to pin 26, about
    10 mΩ combined.
  - That is about 3 mV at 0.3 A, under 1% of the rail, and the P4 drives U20's FB pin itself.
- **Heating is negligible.** The busiest segment, the In2 trunk (now 0.4 mm and about 4 mm long, between planes), rises
  under 10 °C at 0.3 A by IPC-2221's conservative inner-layer rule.
- **The V9 flew with less.** The fabbed V9 fed the same pins through 56 mm of mostly 0.2 mm trace (some 0.1 mm) with 6
  vias.

**As routed now** (owner, 2026-09-29 20:48): In5 26.3 mm at 0.3 mm; In2 3.9 mm at 0.4 mm and 4.4 mm at 0.3 mm; the rest
0.1–0.3 mm. Six vias, one of them out of the output pour at (92.89, 107.34), and the feed is still a chain (76 → 91 → 26).

**What would matter** is the core's fast current steps. Those come from local capacitance, and trace width barely changes
a trace's inductance.
- **Suggested, optional:** a 100 nF within about 1 mm of pins 26 and 76. Today the nearest are C73, 2.30 mm from pin 26
  through 0.10 mm trace, and C67, 3.33 mm from pin 76 through two vias. No drop-in site exists: at pin 26 it means
  re-arranging the R46 / FLASH_CS / ESP_SDA / ESP_SCL escapes; at pin 76, the C79/C84 column (the B side there is J3's
  body).
- **Optional on top:** replace pin 26's 0.10 mm F.Cu tail (5.6 mm of 0.10 mm trace on the net) with a via near the pin.
  A second output via at (92.00, 105.20), inside C55.1, is proven legal (0.23 mm clearance, 0.32 mm hole to hole) if the
  area is touched.
- No further widening is needed.

### P2 The pyro FET gate pull-ups sit at the drivers, not the FETs (CH2: 36 mm and 4 vias away)

**Where** (each pull-up and the gate it serves):

| Channel | Pull-up | FET gate | Gate net |
|---|---|---|---|
| CH2 | R16 (81.33, 142.33) | U6.4 (81.45, 150.92) | Net-(Q3-C): 36.1 mm (In2 19.8, In5 10.8, B.Cu 5.5), 4 vias |
| CH3 | R17 | U7.4 | 10.5 mm, 2 vias |
| CH4 | R23 | U10.4 | 12.4 mm, 2 vias |
| CH1 | R15 | U8.4 | 9.2 mm, no vias |

**Evidence:** WSD20L50DN33: VGS(th) −0.5 to −1.0 V, Ciss 1620 pF, Crss 290 pF.

**Why it matters:**
- If a gate net opens (a cracked via or a bad joint), the gate floats. The Crss/Ciss divider (≈ 0.18) then sets it at
  about −1.4 V with an 8 V pack, which is past threshold, so the channel conducts as soon as U9 arms.
- With the pull-up at the FET, the same open fails safe. U9 already has this: R22 sits 1.6 mm from its gate with no via.
- CH2's gate net also runs 11.1 mm beside M_MOSI at 0.10 mm on In2; a local pull-up stiffens that too.

**Proposed fix:** put a gate-to-source resistor at each FET, from pin 4 to the VBAT_CON source copper. Either move
R15/R16/R17/R23 there or add one each.
- Space is tight: U6.4 to U7.1 is 1.41 mm, and anything placed in the VBAT_CON band above y 150.46 cuts U7's and U10's
  necks (1.90 and 1.65 mm against the 1.40 floor).
- An 0201, or the F side with a via, may be needed. Not space-proven.

### P3 [P3a and P3b done by the owner; P3c accepted 2026-09-30] Smaller power-layout items

- **C53's ground** (TPS62152 output cap) is cut off from U18's PGND by the VOS trace; its nearest GND via is 3.4 mm away.
  Proposed: a GND via at (88.30, 141.47). Checked: 0.12 mm minimum clearance (In2); lands on GND on six layers.
- **V_SCAP_ADC's via at (92.691, 144.472)** sits 0.10 mm from U47's PGND pin 7, inside C144's return path. Moving it
  about 0.8 mm east or south restores the direct return.
- **Three signal vias sit between L11's lands**, 0.155–0.455 mm from the switch land: PYRO1_CONT (90.65, 147.10),
  PYRO3_FIRE (89.73, 147.17) and M_SCK (90.175, 147.40). U47 switches only during supercap top-ups and hold-up, and the
  lines are driven, so the risk is low. If this area is touched, move the pyro lines first.

### R1 [stitched by the owner; the rest accepted 2026-09-30] Antenna clearance: the east wall is a battery pour and unstitched signal lines, not a stitched ground

**Where:** U22's clearance, x 73.113–76.113 × y 133.471–138.071 (rule areas on all layers), and the feed U15.1 → L2/C23 →
U22.

**Evidence:**
- **What matches the datasheet** (Abracon AANI-CH-0070 rev A, p. 4, "Recommended PCB layout"):
  - clearance 4.60 × 3.55 mm with a 0.55 mm GND edge strip (datasheet 4.6 × 3.5 with 0.50);
  - feed-to-strip gap 0.27 mm (0.30);
  - antenna in the middle of the long edge (its "option 1");
  - matching topology as on the evaluation board;
  - no copper at all inside the clearance on In1 to B.Cu, and none under the feed pads.
- **The east wall (x 76.11) has no GND via within 2 mm.** The 1 mm band along it holds:
  - on B.Cu, 81% VBAT_CON pour;
  - on In2, from 0.25 mm out, SERVO_IMON, INA_ALERT, the four I2S lines and ESP_SCL;
  - on In5, PWR_SCL/SDA.
- The datasheet asks for "a robust via structure around the cutout and along the edge of the ground plane".
- L2's shunt ground pad (75.81, 140.50) is 1.82 mm from the nearest GND via.
- The feed has a via fence on its west side only (3 vias at 0.95 mm pitch).
- **Root cause:** the B.Cu VBAT_CON pour covers the whole S3 block (x 75.47–88.83, y 127.35–151.24). A site search finds
  no legal GND via on that wall or near L2 without holing it.

**Why it matters:** for a loop chip antenna the clearance boundary is part of the radiator. A wall made of a battery pour
and digital striplines detunes it. It also couples transmit power into the I2S and I2C lines and their harmonics into the
receiver (the WORKLIST M-25 mechanism, now with I2S 0.65 mm from the wall).

**Proposed fix** (for discussion, because it reshapes the pyro pour):
1. Move the In2 bus and In5 PWR_SCL/SDA to x ≥ 76.9 along this wall.
2. Pull the B.Cu VBAT_CON edge back to x ≥ 77.0 over y 133–138.5 and let GND fill the band. This leaves 1.7–2.1 mm of pour
   there: above the 1.40 mm floor, but it trades against D1's trunk margin.
3. Then stitch the wall with GND vias at x 76.45, y 133.9 / 134.8 / 135.7 / 136.6 / 137.5. Not site-proven until steps 1
   and 2 are done.

Otherwise plan a VNA re-tune at bring-up: C23, L2 and the DNP C12 position are the trim points.

**Note:** the datasheet detail also shows a 0.10 mm slit through the edge strip beside the antenna; this board's strip is
continuous. The Beetle's strip is continuous too, and its BLE works, so treat the slit as a tuning note for the VNA
session, not a defect.

### M1 [done by the owner 2026-09-29] The S3 RF supply (VDD3P3, pins 2/3) is still 0.127 mm; #1365 asked for ≥ 0.25 mm

**Where:** F.Cu, pins 2/3 (75.53, 142.87 / 143.27) → junction (74.83, 143.07) → C148.1 (73.96, 142.96) → L4.1.

**Evidence:**
- All 4.90 mm of Net-(U15-VDD3P3) is 0.127 mm.
- The S3 HDG asks for ≥ 20 mil on these pins; it carries the ~340 mA TX bursts.
- POWER_SWITCH runs through the filter column 0.10 mm away; harmless, since it is a DC node with C105 (10 µF).

**Proposed fix:** widen the common run (x 74.07→74.83, y 143.07) and the junction diagonals to 0.25 mm, and the pin stubs
to 0.20 mm. Checked clearances: 0.218 mm to the POWER_SWITCH via, 0.19 mm to U15.1 (LNA_IN) and U15.4, 0.211 mm to
CHIP_PU.

### M2 [done 2026-09-30: INT1 re-routed] The P4 crystal's XTAL_N now runs 0.10 mm from the IMU's INT1 (the #1365 item moved to a different net)

**Where:** F.Cu around (81.0, 101.8–106.8), between U17 pin 99 and Y4.

**Evidence:**
- XTAL_N to ISM6HG256_INT1: minimum gap 0.100 mm at (81.01, 105.42). The lines run within 0.12 mm for 1.78 mm and within
  0.30 mm for 4.50 mm.
- The INT1 via's In1 antipad clips the plane under XTAL_N.
- XTAL_N also passes Y4's other pads at 0.10 mm, and a U28-DVDT via sits 0.10 mm from it.
- Legs are 11.87 mm (XTAL_N) and 7.92 mm (XTAL_P).
- The old problem is fixed: SENS_SCLK is now 0.44 mm from XTAL_N (was 0.10).

**Why it matters:** INT1 edges inject charge into the P4's only clock node, which drives its PLL and USB timing. Failure
is unlikely, but this is the coupling #1365 asked to remove.

**Proposed fix:** drop XTAL_P's 0.11 mm jog at (80.61→80.72, 106.72), run XTAL_N at x ≈ 80.85, and offset INT1's F.Cu run
0.2–0.3 mm north-east into the open pour east of x 81.4. Aim for ≥ 0.3 mm with pour between. Watch the LoRa_RX via at
(80.38, 101.34).

### M3 [waiver re-affirmed 2026-09-30] The S3's exposed pad has 3 GND vias (the V9 waiver was 5; the HDG asks for 9)

- U15 pad 57 (4.1 × 4.1 mm) carries 3 in-pad GND vias, plus one in the ring pour.
- Under the pad, B.Cu is 81% VBAT_CON pour, and In2 carries 11 nets across it; no legal site exists without holing the
  pyro pour.
- Electrically, 4 vias with In1 only 0.116 mm below are adequate for the S3's dissipation and RF return.
- **Proposed:** re-affirm the WORKLIST M-17 waiver at "3 + 1" in the fab/design notes. If R1's pour pull-back happens,
  revisit.

### S1 [done 2026-09-30] Return vias: where a GND via actually helps on this stack, and six proven sites

**Evidence:**
- Census: 222 signal vias; 95 have a GND via within 1 mm and 169 within 2 mm.
- A 2-D field solve of this stack shows why the owner's rule helps some hops and not others:
  - An In2 trace returns about 63% of its current on In3 (+3V3, 0.153 mm away) and 37% on In1 (GND, 0.300 mm). In5 splits
    the same way between In4 and In6.
  - So a GND via carries the whole return only for **F.Cu ↔ B.Cu** hops, and about 37% for F↔In5, B↔In2 and In2↔In5.
  - For F↔In2 and B↔In5 hops the shared GND plane needs no via; the power-plane share needs a +3V3 or V_MCU_SWTCH
    decoupling cap near the via.
- By type: F↔In2 59, F↔B 24 (+2), B↔In2 14, F↔In5 11, B↔In5 9, In2↔In5 6.
- The only fast F↔B hops are the P4 flash lines.

**Proposed GND vias**, each checked on all 8 layers:

| Serves | Proposed via | Notes |
|---|---|---|
| U16 flash FLASH_CS (74.415, 117.412) | **(74.340, 117.912)** | 0.51 mm from it; 0.103 mm clearance; 0.206 mm hole to hole |
| FLASH_CK (75.590, 116.160) and U16's GND ball E3 (no via today) | **(75.465, 114.860)** | 1.31 mm from the CK via (was 1.93), 1.13 mm from E3; 0.101 mm clearance |
| FLASH_HD (75.599, 116.780) | **(75.849, 118.130)** | lands in J1.7's GND pad on B.Cu; 0.115 mm clearance |
| NAND M_MOSI (88.70, 156.95) | **(89.45, 156.25)** | 1.56 → 1.03 mm |
| M_SCK (90.175, 147.40), In2↔In5 | **(91.700, 147.050)** | 2.63 → 1.56 mm; also near PYRO1_CONT and PYRO3_FIRE |
| P4_EN_HOLD / P4_EN_S3 (73.33, 146.61 / 73.17, 147.10) | **(72.780, 145.811)** | in C25.2's GND pad; ring 0.22 mm from the edge |

- The three U16 vias each take an antipad in the In5 VBATT pour. The pour's narrowest section is unchanged (2.48 mm), and
  the local cuts stay ≥ 3.35 mm.
- **Not proposed:** a PG_RAIL site at (80.175, 131.925). PG_RAIL is a DC line, and that site sits 0.100 mm from the B.Cu
  VBAT_CON pour, whose 0.127 mm zone clearance would pull the pour back.
- **No legal site exists** for:
  - the flash lines' P4-end vias (81.83, 117.27) and (81.22, 116.05);
  - the I2S hops at both ends;
  - the IMU SPI hops at (81.65–82.66, 105.55–105.98);
  - the U13 flash fan-out (its +3V3 cap C26 is 1.6–3.9 mm away, which covers the power-plane share);
  - CEN_D± by slot 2, INA_ALERT between the pyro pours, and D+ at (88.25, 160.05).

### S2 [firmware mitigation 2026-09-30; the re-route waits for the next layout] In2 inter-MCU ribbon: I2S BCLK runs 36 mm at 0.10 mm beside I2C SCL; SDA runs 37 mm beside PYRO4_CONT

**Where:** In2 from U17 (y ≈ 112.3) to U15 (y ≈ 146.3), mostly x 77.2–77.9. Ribbon order from west to east:
SERVO_IMON, INA_ALERT, WS, SD, POWER_SWITCH, FSYNC, **BCLK, SCL, SDA, PYRO4_CONT**, PYRO3_CONT.

**Evidence:**
- BCLK runs within 0.10 mm of ESP_SCL for 35.99 mm; ESP_SDA runs within 0.10 mm of PYRO4_CONT for 36.68 mm.
- A field solve gives mutual capacitance equal to 24% of each line's self-capacitance.
- ESP_SCL is open-drain with a 5.11 k pull-up (~11.5 pF), so each BCLK edge steps a released SCL by about 0.29 V
  (τ ≈ 59 ns).
- PYRO4_CONT is a 96 mm, 100 k, uncapped ADC line; each SDA edge puts about 0.2 V on it (τ ≈ 1.5 µs).

**Why it matters:**
- A BCLK edge during SCL's ~130 ns rise can cross the threshold twice and add a clock on the inter-MCU link, unless both
  ends filter I2C glitches.
- The CONT reading also picks up I2C-correlated noise.

**Proposed fix:**
- Swap a quiet line between BCLK and SCL: POWER_SWITCH (10 µF on it) or V_SCAP_ADC (100 nF). Or open the BCLK/SCL and
  SDA/PYRO4_CONT gaps to ≥ 0.30 mm, which takes the coupling from 0.24 to 0.057.
- Space along the full run was not checked.
- At minimum, confirm the I2C glitch filter is enabled on both MCUs.

### S3 [accepted 2026-09-30] PYRO2_FIRE hugs both slot faces; a few other lines run flush with plane edges

- **PYRO2_FIRE** runs 15.1 mm on In2 at 0.21–0.24 mm from the slot cut faces: y 119.26 at slot 1, x 88.02, and y 132.09
  at slot 2. The planes stop 0.20 mm short of every routed edge, so this gate command line has no plane beyond its outer
  edge at an exposed face (straps or wires through the slots, ESD).
- **Others:**
  - In2 LoRa_ACT: 28.5 mm at x 94.46.
  - The SW2 boot line Net-(R54-Pad1): 14.6 mm at x 94.46.
  - In2 PYRO4_CONT: 19.8 mm at y 171.03.
  - SERVO_IMON: 15.2 mm along x 72.6–72.9.
- **Proposed:** move PYRO2_FIRE ≥ 0.5 mm in from both slot faces. Where the ribbon allows, move the others 0.3 mm in.

### X1 [hardware rule in FAB B11, 2026-09-30] The magnetometer is 0.64 mm from the H3 screw head: specify non-magnetic hardware at H3 and H1

**Where:** U3 (F, 87.93, 98.29) and H3 (91.97, 99.14).

**Evidence:**
- U3's body is 0.64 mm from H3's Ø3.8 pad / DIN 965 head circle. On the V9 the magnetometer was 7.94 mm from H3, so a
  screw's pole field is about 3.7× stronger here.
- A steel M2 screw magnetised by a driver bit (10–25 kA/m) is estimated at 180–460 µT at U3; an unmagnetised one still
  distorts about 10 µT.
- The firmware runs U3 at ±8 G = ±800 µT.
- The QST datasheet (§4.3) says to keep ferrous parts away on both sides of the board.

**Why it matters:** a hard-iron offset of several Earth fields eats range, and it changes whenever the screw is swapped
or re-magnetised, which invalidates the calibration.

**Proposed fix:** no layout change. Specify brass, titanium or nylon M2 hardware at H3, and at H1 (12.7 mm away), with
nylon or brass standoffs and nuts. Put it in the assembly notes.

**Also checked:** fields from board currents are not worse than the V9's.

| Path | V10 at U3 | V9 at its magnetometer |
|---|---|---|
| LoRa | 40 µT/A | 50 µT/A |
| Servo | 11.5 µT/A | 89 µT/A |
| Core buck | 9.5 µT/A | 31 µT/A |

The M-11 firmware gating stays as decided.

### X2 [hardware rules in FAB B11, 2026-09-30] A standoff or washer larger than the Ø3.8 pad lands on live copper at every hole

**Evidence:**
- At every hole, other-net copper starts only 0.10–0.13 mm outside the Ø3.8 pad.
- B.Cu copper inside a Ø5.0 annulus:

  | Hole | Copper | Area |
  |---|---|---|
  | H2, the only GND-bonded hole | **VBAT_CON** (upstream of the eFuse) | 1.07 mm² |
  | H8 | VBAT_CON | 0.84 mm² |
  | H6 | PYRO1_EXT | 3.48 mm² |
  | H4 | VBATT | 1.53 mm² |

- Parts on B sit 2.22–2.50 mm from the hole centres, so a Ø5 washer can't sit flat anywhere, and a 4 mm hex standoff can
  hit parts at H1, H3, H4, H5 and H7.

**Why it matters:** solder mask is the only insulation between a clamped, vibrating standoff and pack or pyro copper. At
H2, one breach shorts the unfused pack to GND through the bonded hardware.

**Proposed fix:** a hardware specification.
- Heads no larger than Ø3.8.
- Round standoffs no larger than Ø4.0, or 3.5 mm hex; no metal washers.
- Nylon standoffs at H2, H4, H6 and H8 on B. At H2, an insulating washer under a metal standoff keeps the F-side GND bond.
- Pulling copper back to r 2.5 works at H4, H6 and H8, but not at H2, where the VBAT_CON section is only 1.74–3.27 mm.

### X3 [via deleted 2026-09-30] The IMU sits directly over J5, near H1 and J3, with a via and a trace under its body

**Evidence:**
- U2 (F, 79.92, 99.03, 45°) lies over J5 (B, a vertical-mate LoRa SH connector): 7.64 of its 7.74 mm² body area is above
  J5's body.
- U2's body is 0.83 mm from H1's head circle and 1.37 mm from J3's body.
- Under the body on F.Cu: a GND via at (79.675, 99.175), 0.29 mm from centre and not in a pad, and a 0.127 mm V_MCU_SWTCH
  link from pad 5 to pad 8 (1.06 mm of it under the body).
- ST's guidance: no routing or vias under the device, and sensors kept away from external-force sources and between
  fasteners.
- This is the V9's position; the V10 is better under the body, but not clear.

**Why it matters:** board strain from mating J5, a tugging cable and the screw clamp shifts IMU offsets, and the flight
estimator depends on U2.

**Proposed fix:**
- Move the via into GND pad 6 or 7; its ring then stays 0.15 mm from pads 5 and 8.
- Route the pad 5 → 8 link outside the body.
- Calibrate U2 with J3 and J5 mated and the screws torqued, and tie the LoRa and servo cables to the sled.
- For discussion only: swapping U2 and U4 puts the IMU 0.7 mm off the centreline and away from H1 and J5, at the cost of
  re-routing both SPI fan-outs.

### X4 [done 2026-09-30] The GND via under the barometer is back in the open gap (the V9's M-12 fix did not carry over)

- The GND via at (84.350, 99.350) sits under U4's body, 0.50 mm from its centre; it overlaps pad 9 by 10% of its ring.
- BMP581 datasheet §8.2: "We do not recommend vias or traces under the BMP581." WORKLIST M-12 closed exactly this on the
  V9.
- **Proposed fix:** centre the via on pad 9 at (84.28, 99.07). Checked: its ring clears pad 10 (V_MCU_SWTCH) by 0.13 mm.
  Alternatively delete it and tie pad 9 to pad 8 on F.Cu; pads 3 and 8 already have rim vias.

## Minor — fabrication and assembly

### F3 [done by the owner 2026-09-29] U18, U27, U29 and U30 are older library copies: the decided U18 window panes are not on the board

This is the cause of 4 of the 6 lib_footprint_mismatch warnings.
- **U18** (TPS62152) has only the old 1.06 × 1.06 mm paste polygon: 40% of the exposed pad as one island. The library
  has the four 0.65 mm panes the owner chose on 2026-09-28 (~57%).
- **U27, U29 and U30** carry the library's two 1.0 × 0.7 panes plus the old 0.632 × 1.012 polygon, which merge into one
  island of about 95%. The library (panes only) matches TI's 88% example.
- All four keep a 0.05 mm EP mask margin where the library now has 0. That leaves EP-to-pin mask webs of 0.065 mm (U18)
  and 0.075 mm (the SONs), which a fab will not hold.
- **Fix:** *Update Footprints from Library* on U18, U27, U29 and U30. The diff is paste and EP-margin only, with no pad
  moves; then re-run DRC.
- Y2 and Y4, the other two mismatches, differ only by a library silk rectangle.

### F4 [done 2026-09-30] Pin-1 and polarity marks

- **U2's pin-1 circle is on B.SilkS** in its library footprint (`U_LGA-14L_2p5X3p0X0p83_STM`). The top shows a
  symmetric outline only, and the dot prints on the bottom at (77.95, 99.97), on J5's outline beside J5 pin 4, where it
  reads as J5's pin 1. Fix it in the library and update U2.
- **No mark:** U17, U15, U23, U1, U6, U7, U8, U10, U9, J1 and J3.
  - J3's pegs are symmetric at ±6.0 mm, so a 180° placement fits. That puts the servo supply at the J3.15/16 GND end,
    which is B7's reverse-polarity case.
- **Marks lost to masks or the edge:**
  - U26 keeps 7% of its dot (R86.1's opening covers it) and U28 9% (off the x 72.36 edge). Their land is symmetric under
    180°, which would swap IN and OUT.
  - U29 keeps 26%, U11 34% and U19 38%.
- **Fix:**
  - Add pin-1 dots outside the pin-1 corner of U15, U17, U23 and U1, and marks for J3 and J1.
  - Move U26's and U28's dots clear.
  - Check the result against the mask plot, or plot with "subtract soldermask from silkscreen", which is off today.

### F5 [done 2026-09-30; the owner moved the "O" to (92.64, 142.29)] S1's "F"/"O" legends print over pads, and two "+" marks are orphans

- The owner's F/O convention stays; only the placement is a problem. The B-side "F" (83.8–85.1, 135.2–137.1) is 56% over
  C44's pad openings, and the "O" (92.7–94.2, 133.8–135.6) is 34% over R52/R63.
- **"+" at (83.9, 138.87)** is the V9's C12 mark; it is now 44% over R79.1, beside U1.
- **"+" at (75.43, 150.0)** overlaps U8's and D1's outlines beside D1's cathode, so it reads as reversed polarity.
- **Fix:** move F/O clear and delete both "+" marks.

### F6 [done 2026-09-30] Tombstone risk on the USB CC resistors and U9's gate pull-down

- 94 of 187 0402 two-pad parts have one pad solid in a pour and the other on a track. The ones that matter:

  | Part | Role | Pour pad | Track pad |
  |---|---|---|---|
  | R41 | USB CC1 | 98% | 6% |
  | R47 | USB CC2 | 100% | 6% |
  | R22 | U9 gate pull-down | 100% | 13% |

  (Figures are the share of each pad's perimeter bordered by same-net copper.)
- A tombstoned CC resistor means no VBUS from a host, and USB-C is the only programming path. Tombstoned CC pull-downs were
  the second-ranked cause in the Beetle first-article investigation.
- **Fix:** set a pad-level thermal-relief connection on the GND pads of R41, R47 and R22, and optionally the crystal load
  caps. No copper moves. Leave L2's GND pad solid, because spokes add shunt inductance.

### F7 [OC_ARM_EN moved; VDDO_PSRAM accepted 2026-09-30] Different-net via rings sit 0.008–0.03 mm outside J4/J2 pad mask openings

J4 and J2 carry a 0.102 mm mask margin, so a slight mask misregistration exposes part of a ring inside a neighbour's
opening. The worst:
- OC_ARM_EN at (90.66, 160.04), in Q14.1: 0.008 mm from J2.5 (PYRO_GND).
- VDDO_PSRAM at (90.09, 114.46): 0.008 mm from J4.2.
- Camera_TX and GND at x 86.10: 0.018 mm from J4.4.
- EXP_07 and FC_ARM at x 88.10: 0.018–0.021 mm from J4.3.
- Eight more at 0.024–0.028 mm.

**Fix:** move OC_ARM_EN's via to (90.72, 160.04), where it stays inside Q14.1, and VDDO_PSRAM's to (90.19, 114.46). Or cut
J4/J2's mask margin to 0.05 in the library, which also affects the Beetle's J2.

### F8 [done 2026-09-30] The bottom-side fiducials are 8.6 mm apart and nearly collinear

- FID3 (93.36, 118.25) and FID4 (92.48, 126.85): dx 0.88, dy 8.60 mm. A ±0.02 mm fiducial error is about 0.12 mm at U28
  (0.45 mm pitch) 25 mm away.
- The top pair spans 36.7 mm, and all four fiducials are clean.
- **Proposed:** move FID3 to (73.3, 97.3) on B, which gives a 35.3 mm diagonal. It is clear of pads, tracks, vias,
  non-GND pours, courtyards and silk, and is 0.76 mm from H1's pad.

### F9 [done 2026-09-30] What the rewritten fab/assembly notes (F2) should also say

- **Reflow order:** top side first, bottom second. The heavy bottom parts (J2 about 3 g against its land, J3, J4) then
  reflow once, upright.
- **Hand-solder** J8 and C130 after both passes, and leave them out of the position file.
- **Mask webs:** U17 0.060 mm (0.35 mm pitch), U19 0.056, U15 0.070 and U23 0.029 mm (diagonal). If the fab can't hold
  them, gang the opening per row; do not shrink the pads.
- **C130:** stake it.
- **Hardware:** non-magnetic at H1/H3 (X1), and the standoff and washer limits of X2.
- **Position file:** `kicad-cli pcb export pos --exclude-dnp` drops exactly C12, C93, R74, R75 and R76.

---

## Notes

- **C17**, the second 22 µF on U47's output, sits at U30's input, 25 mm from U47. Only C144 is local, although the v10
  power-parity note and #1365 A put C144 beside C17 at U47's VOUT. Through the In3 plane the aggregate output capacitance
  still meets TPS61094 §6.3; correct the note or specify C144 for ≥ 20 µF effective at 3.4 V.
- **The P4 half has only two +3V3–GND caps**, C40 and C17. The IMU SPI on In2 returns 63% on +3V3, and the nearest +3V3
  cap is 15–22 mm away. A 100 nF near (82, 105) and one near (79.4, 113) would give it a local path (not space-checked).
  The SPI runs at ≤ 10 MHz, so the risk is EMI rather than function.
- **USB is full-speed only.** The P4 uses pins 52/53; its HS pins 49/50 are unused.
  - J6 → U1: D+ and D− are not routed as a pair. They run 6.2–6.7 mm apart (≈ 125 Ω), with 2 and 4 vias.
  - S3 side: coupled at 94 Ω.
  - P4 side: CEN_D+ is 33.2 mm against 27 mm for CEN_D−, because of the R77 detour, and there are no series resistors
    (known).
  - CR3 now sits directly behind J6. D− reaches it on a 3.25 mm stub; U1 itself is rated IEC 61000-4-2 ±8 kV contact.
  - None of this is critical at full speed.
- **Crystal Y2 (S3)** improved: legs 11.2 and 7.3 mm (was 14 and 5.4). XTAL_N runs 0.100 mm along C43's (buck input) pads
  for about 1.3 mm; keep ≥ 0.3 mm if the area is touched.
- **Other pyro-line neighbours:**
  - PYRO4_FIRE runs 15.5 mm at 0.14 mm beside M_MOSI.
  - OC_ARM_EN runs 13.7 mm at 0.10 mm beside D+ on In2, from (79.23, 153.01) to (86.78, 160.56).
  - FC_ARM runs 14.6 mm beside LoRa_RX.
  - None of these can fire a channel, which needs FIRE, FC_ARM and OC_ARM_EN together; worst coupled steps are about
    0.1–0.2 V into low-impedance nodes.
- **CH1's output neck** is 1.37 mm (B.Cu PYRO1_EXT, set by the INA_ALERT via pair at (78.43, 154.90)/(78.96, 154.44)).
  The other outputs are 2.63–3.21 mm. Normal fire rises ≤ 40 K; moving INA_ALERT's hop ~0.6 mm east would recover ~2 mm.
- **C145 (100 nF) sits farther from S3 pin 29 than C27 (1 µF)**; swapping their positions costs nothing.
- **R141** (the S3 GPIO0 pull-up) sits at SW3, 21 mm from pin 5; acceptable.
- **NAND U11** has C16 (100 nF) 1.5 mm from VCC; its nearest bulk is C144 (22 µF) 12 mm away. #1365 F's bulk cap is still
  a schematic item.
- **The X/Y axis arrows** by the Boot legend describe the board frame, not U2's 45° axes. They are 0.75 mm text against
  the 0.8 mm minimum. Suggest 0.8 mm and a "BOARD" label, or move them near U2.
- **DRC hygiene:**
  - Delete the SERVO_ACT stub (73.300, 104.525 → 73.375, 104.875).
  - Raising the four ignored severities (#1365 E) surfaces only U20's missing courtyard and J6's pegs inside J2's
    courtyard (they stop 0.90 mm into the board), so it is safe to raise.
  - There is still no `.kicad_dru`: the Beetle's 0.25 mm different-net hole rule would flag 73 pairs here (min 0.200).
  - Copper-to-edge is exactly 0.200 mm at the slots and corners.
- **Sensors hug the top edge.** U3 is 0.47 mm and U2 0.73 mm from y 96.32; ask the fab for no break tab or V-score on that
  edge.
- **Mounting orientation:** the B side is 12–16.5 mm tall (C130; J8 when mated), so the board likely mounts F-side down.
  SW2/SW3 can't be reached then, but S1 and J6 can.
- **The bottom 18 mm beyond H6/H8** carries J2, J6 and the e-match leads. Its estimated first mode (0.9–1.6 kHz) is near
  the ~800 Hz boost tone, so tie the pyro leads to the sled close to J2 (low confidence).
- **IMU lever arm:** U2 is 3.6 mm off the long centreline. A 6–8 Hz spin about that axis gives 0.5–0.9 g lateral;
  compensate ω×(ω×r) with r from this layout.
- **Thermal:** the board is nearly isothermal, +24–34 K in still air at ~1.1 W. U2, U3 and U4 sit within 1.5 K of the P4
  area. Q11 is the largest single loss (0.18–0.25 W at 3 A). Allow gyro warm-up before bias capture.
- **H2's hardware is 6.75 mm from U22.** Tune the match with the flight hardware fitted.
- **3D models (cosmetic):** J8 has none, so the render hides the tallest mated part. D9 and U47 point at KiCad paths that
  don't exist, although the repo has both STEPs. J2's model is offset 3.7 mm from its Fab outline.

## Issue #1365 on this layout

| #1365 item | Status now |
|---|---|
| B: Q3–Q6 shorts, clearance errors, mask bridges | **Gone** (DRC has no clearance or mask errors) |
| B: courtyard overlaps Q4/Q6, FID2/U16, J4/FID3 | **Gone**; only C15/J1 remains (deliberate) |
| B: 3 unconnected items | **0** |
| B: hairline pad joints (C89, R87) | **Clean**: sweep of every pad's sole connection; shallowest overlap 45 µm deep at full track width |
| C: ESP_VDD_HP pour | **Not needed at this current** → P1, now a note. Widened to 0.3–0.4 mm on 2026-09-29; a 100 nF at pins 26/76 remains optional |
| C: S3 VDD3P3 ≥ 0.25 mm | **Open** → M1 |
| C: C43 buck input cap on the back | **Resolved**: all four input caps on F.Cu, 1.5–6.0 mm from PVIN |
| C: ribbon spacing, pyro vs clocks/USB | **The listed pairs are gone**; new In2 pairings in S2 and the notes |
| C: P4 XTAL_N vs SENS_SCLK | **Fixed** (0.44 mm), but XTAL_N now sits 0.10 mm from INT1 → M2 |
| C: USB D+/D− routing, CR3 placement | CR3 **fixed** (directly behind J6); the pair is still unmatched (note; full speed) |
| C: Y2 leg asymmetry | **Improved** (11.2 / 7.3 mm) |
| C: INA230 Kelvin tap | **Verified**: IN+ = VBAT_Terminal, IN− = VBAT_CON, each touching its net only inside R72's pads |
| D: silk | S1 F/O kept by the owner but now over pads (F5); J2 pin-1 circle still clipped at the edge; X/Y text still 0.75 mm |
| E: thickness cache | **Fixed** (1.63008) |
| E: `.kicad_dru` | **Still none** |
| E: ignored severities | **Still ignored**; safe to raise |
| E: EP mask margins 0.05 | **Still** on U18/U27/U29/U30 → F3 |
| E: P4 EP paste | 9 windows, 40% |
| E: gerbers, 3D models | `gerbers/` empty (plot at release); 3D items in the notes |
| F: NAND bulk cap | Still a schematic item (note) |
| H: refill, DRC, parity checks | **Done** for this review (see *State of the files*) |

## Verified OK

- **Battery input:**
  - Q11: drain (pins 5–8 + EP) on VBAT_J8, source (1–3) on VBAT_Terminal, gate (4) on GND. The body diode blocks a
    reversed pack; VGS = −Vpack, within ±25 V. The pyro FETs and U9 are also the right polarity and orientation.
  - Necks: VBAT_J8 2.08 mm, VBAT_Terminal 1.47 mm.
  - R72.1 → U19 IN goes through 4 vias, each ≤ 1.2 A at 3 A and ≤ 4.6 A at the 11.6 A limit over the 4.7 ms ITIMER.
  - J8's GND pin has thermal spokes and its VBAT_J8 pin is solid on B.Cu.
- **INA230:** IN+ is on the supply side and IN− on the load side; VBUS is tied to IN− as in TI's example. The tap error is
  about 0.1% of 2 mΩ for VBATT current.
- **U19 eFuse:**
  - ILIM 11.6 A (R48 127 Ω), fast trip ≈ 24 A, ITIMER ≈ 4.7 ms (C50).
  - UVLO 6.91 / 6.34 V with C94's deglitch.
  - All its parts are 1.0–1.35 mm from their pins.
  - The GND exposed pad has no vias (Q11 is below) but sits solid in a 138 mm² F.Cu GND area with 24 vias.
  - 24 mW at 3 A.
- **VBATT distribution:** 12.9 mΩ to U28 (39 mV at 3 A). The old 0.66 mm In5 neck at U28 is gone; the In5 section is now
  ≥ 2.9 mm. Load-switch outputs to J3/J4 are ~2 mΩ, and the J3 GND return is 0.89 mΩ through ≥ 10 vias.
- **Pyro necks** (pad-inclusive, owner floors): U8 1.45 mm (floor 0.89), U6 1.75, U7 1.90, U10 1.65 (floor 1.40).
  - The trunk minimum is 2.22 mm.
  - The fire path's high side stays on B.Cu. PYRO_GND drops to U9 through 5 vias, and U9's source has 3 in-pad vias; no
    via carries more than 3.2 A at 9.6 A.
  - Normal-fire adiabatic rises are ≤ 36 K.
  - Drive and arm parts (Q12–Q14, R21, R22, R132, R139) sit within 5 mm of U9; R78–R81 are 1.7–2.7 mm from their
    drivers.
- **Power conversion:**
  - U21 feeds: VBATT neck 0.59 mm; V_MCU_2S min 0.655 mm, median 1.4 mm.
  - U18: loop tight, AVIN pour 1.78 mm, VOS senses at C53, 3 EP vias.
  - U47: bypass path ≥ 1.85 mm, 5 EP vias, R134–R136 at their pins. L11's lands match the Bourns footprint.
  - U20: C47, C55 and C92 within 0.26–0.45 mm, with in-pad GND vias.
  - All three switch nodes are F.Cu-only over solid In1, with no signal track within 0.5 mm.
  - In3 is fed by 2 vias at U47's VOUT and In4 through U30's in-pad via.
  - Every converter runs cool: U18 rises ~6 °C at 0.35 A, U20 ~10 °C, U47 ≤ 10 °C while boosting, U21 and U30 a few
    tens of mW.
- **Processors:**
  - P4 EP: 27 GND vias (the V9 had 10).
  - The v1.3 DNP provisions (R74/R75/C93 on FB_DCDC, R76 on pin 54) exist and are wired.
  - Every other P4/S3 supply pin has its cap within 1.1–3.8 mm; the memory VCC caps are at 1.4–2.3 mm.
  - Crystals: 4.77 mm (Y4) and ~5.4 mm (Y2) from their pins, meeting the HDG; no vias on any clock net; In1 covers 94–100%
    under all four crystals.
  - Both WLCSP ball maps match Winbond's.
  - Reset RCs and straps are at their pins (R42/C39, R36/C25, R50, R53/R54).
  - D9/C105/R84 are placed per the enable design.
  - S3 USB series resistors R38/R39 sit 1.6–2.2 mm from the pins.
- **RF feed:** 0.18 mm GCPW with 0.13 mm gaps over In1, about 47 Ω, constant width, 45° bends, no vias. Segments are
  2.3 / 0.9 / 2.6 mm; the C12 DNP stub is 0.96 mm.
- **Mechanics:**
  - J6's receptacle face is 0.25 mm behind the edge, identical to the fabbed and working V9.
  - No 3D bodies overlap.
  - J3's pegs stop 0.9 mm into the board, and J8's and C130's leads come through clear of top-side parts.
  - Every footprint's pads are inside the outline.
- **Fab data:**
  - Drills: 490 × 0.30 (487 vias at 0.40/0.30, 3 at 0.45/0.30), 2 × 0.90, 2 × 1.70, 8 × 2.20; NPTH 2 × 0.65 and
    2 × 1.041; hole-to-hole minimum 0.1999 mm edge to edge.
  - No isolated outer copper.
  - No mask opening merges two nets, and no different-net via ring sits inside an SMD opening.
  - Copper balance by layer is 77–91%, with a top/bottom ratio of 0.96.
- **EP paste coverage:**

  | Part | Coverage |
  |---|---|
  | U17 | 40% |
  | U15 | 55% |
  | U19 | 80% |
  | U26/U28 | 86% |
  | U47 | 84% (TI 83%) |
  | U11 | 76% |
  | U23 | 40% |
  | Q11 | 65% |
  | U18 / SONs | see F3 |

## Not checked

- Real part masses (the bottom-side weight budget uses land area only).
- The J2 and J3 drawings for the fitted parts.
- J8 and J2 contact ratings, and R72's pulse rating.
- The pyro FETs' and U9's transient thermal curves. D1 uses IDM plus Q11's single-pulse class.
- Space for P2's gate resistors, S2's ribbon re-order and the extra +3V3 caps along their full runs.
- The antenna's actual tuning (needs a VNA), and the effect of the edge-strip slit or wall stitching.
- U20's stability with the remote FB_DCDC sense.
- The real P4 VDD_HP current. P1 estimates 0.1–0.3 A typical and a few hundred mA peak from the datasheet and
  Espressif's supply budget; it has not been measured.
- L8's leakage field at the magnetometer.
- JLC capability figures (mask dam, copper-to-edge): none are captured in the repo, so none are asserted here.
