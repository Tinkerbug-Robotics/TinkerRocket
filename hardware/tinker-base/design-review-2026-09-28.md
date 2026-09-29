# Tinker-Base — design review, 2026-09-28

**Board:** `hardware/tinker-base`, rev **V1** (title block), never fabricated. This review covers the owner's working copy
as saved at 16:17 on 2026-09-28.
- PCB md5 `da86a82b`, `esp32_p3.kicad_sch` `a6971682`, project `3eefd31a`.
- `power`, `battery` and the root sheet are unchanged from `4a7904c3`.
- **None of the 2026-09-27/28 work is committed.** That covers the Beetle/Mantis parts and their layout, the J2/R13/R16
  relink, and the owner's 16:17 edits (logo, silk text, the `U22-ON` and `L_RXEN` track moves).

**Method:**
1. Snapshot the saved files, with the shared libraries, into a scratch project.
2. Export with `kicad-cli`: netlist, ERC, DRC with schematic parity, BOM, gerbers. Refill a copy and compare its zone
   fills with the stored ones.
3. Measure geometry with pcbnew and shapely: pours, vias, pads, clearances, return-via distances, copper under the antennas.
4. Run four specialist passes against the datasheets: power tree; ESP32-S3, flash, USB and firmware; RF; footprints,
   assembly, fab package and BOM.
5. Re-check every finding kept here against the files. Every layout fix that gives coordinates was added to a scratch
   copy, refilled and DRC'd: 0 errors, 0 unconnected, no new warnings.

**Datasheets:**
- ESP32-S3 datasheet v2.2 and Espressif's *ESP32-S3 Hardware Design Guidelines* (HWDG; schematic checklist, PCB layout,
  download guidelines; fetched 2026-09-28).
- W25Q128JV rev F.
- TI: BQ21040 SLUSCE2D rev D, TPS22918 SLVSD76C, TPS63020 SLVS916I.
- TDK VLS3012CX-1, Littelfuse SP05, DW01A (PUOLOP rev B), FS8205A (TECH PUBLIC).
- Ebyte E220-900MM22S manual v1.3, Semtech LLCC68 rev 1.0.
- Abracon AANI-CH-0070 rev A and its EVB sheet, RF Solutions DS-SMA-EDGE-4.
- HRO TYPE-C-31-M-12 drawing rev A, MYOUNG WD-CP-0336, E-Switch PB400, Mitsumi R-667995.
- ECS ECX-1637B2, Abracon ABS07, Murata LQG15HS.

**State of the files:**
- **DRC on the saved board:** 1 error (a stale fill, L1), 41 warnings (silk and library), 0 unconnected, 0 parity.
  After a refill: 0 errors.
- **ERC:** 405 items, all hygiene (D4).
- **Footprint links:** all 85 schematic parts' footprints are linked by path, so the J2/R13/R16 relink holds. The new
  `G***` logo is board-only, as logos are.

## Fixes applied (2026-09-28, owner: "let's fix these")

All of it went into the owner's checkout on 2026-09-28 and is committed with this review. The V1 JLC package was
plotted from exactly these files.
- **Checks on the result:** DRC 0 errors, 26 warnings (was 41; all silk plus the logo's library nickname),
  0 unconnected, 0 parity. All 91 schematic parts' footprints are linked by path (90 before C26). A fresh `kicad-cli` refill
  reproduces the stored fills exactly.
- **Method:** each layout edit was applied to an unfilled copy, filled in a separate process and DRC'd.

**Done:**

| Item | What changed |
|---|---|
| L1 | Zones refilled. |
| L2 | The F.Cu window keeps the feed notch; a second rule area covers In1/In2/B.Cu without it. The window was trimmed to 4.6 × 3.0 mm. Six fence vias at x 51.40 and three at y 152.55 replace (51.9, 151.75/152.6). Nothing sits under U15's feed pads on any layer now. |
| L3 | Eight GND vias beside signal vias, plus the one L13 adds; signal vias over 1 mm from a ground via: 14 → 5. The remaining five are Q1-G1 and CHIP_PU (static), L_BUSY at 1.17 mm, and the two SPI hop vias, which have no legal site. |
| L4 | Nine GND vias in U3's exposed pad, in the paste-window gaps. |
| L5 | A GND via for C16/C24, now at (53.52, 149.60) with a stub to the C24.2–C16.2 strap. It also serves C8.2 (see *Found at packaging*). |
| L7 | GND vias at (70.97, 109.045) and (75.25, 116.98) for U16's ground pads; the first has a stub to pad 2. |
| L8 | `C25` 100 nF V_SWITCH→GND at (48.85, 171.8), grounded to U9 pin 13, which sits in the PowerPAD pour. `C26` 100 nF +3V3→GND at (54.68, 171.48), added at the owner's word: its +3V3 pad sits on the output pour beside VOUT, U9's FB line runs under its body between the pads, and its GND pad links to U9 pin 2's via, moved to (53.77, 172.12). |
| L9 | J8 moved to (62.885, 105.695); its pads start 0.30 mm inside the edge. |
| L10 | J2's 16 signal pads trimmed at the rear to HRO's land, in the library and on the board: BT2.P to J2 is now 0.84 mm, was 0.40. |
| L12 (part) | J2's shell legs and D6's cathode connect to GND through thermal reliefs. |
| L13 | S3's GPIO0 via moved from under the switch to (52.226, 145.0), with a return via at (52.80, 145.00). F.Cu keep-out over the switch body. |
| S2 | `L5` 330 nH (0603) from the J8 line to GND at (62.0975, 108.8), per Ebyte §5.1. Two fence vias under its ground pad removed. |
| S6 / P1-A | `R52` 680 Ω → 1.2 kΩ (0.40–0.50 A). |
| S5 | Documented in the README: USB charges the cell and does not run the board; charge with S1 off; the 10-hour timer. |
| S7 | A B.Silk "−" outside the holder at the N end, and the holder orientation in the fab notes. |
| F1, F10, L11 | Fab notes: gang U3's mask, top stencil only, trim S1's posts, fit J8 and J2's legs by hand, check U1's placement. |
| F2 | The library double-pasted three exposed pads. `IC_ESP32-S3` and `8L_WSON_8x6x3p4x4p3_MAC` were reverted to their pre-4078070b window polygons, which is the pattern every built board carries; that cleared U3's library mismatch. `QFN50P300X300X100-17N` went the other way at 20:09, with the PX1105R carrier work: the owner kept the four 4078070b panes (about 57 %, 0.19 mm gaps) and removed the old polygon, which had merged them into a 1.49 mm blob on the PX1105R board. The owner decided to leave the ESP32-S3 and WSON on their window polygons, the pattern every built board carries. |
| F3 | U15's footprint description now states the real clearance and the notch rule. |
| F4 | Pin-1 dots on the silk for U3, U5, U22, CR1 and U16. |
| F5 | `FID1`–`FID3` on the root sheet and the top side. |
| F7 | R13/R16 references hidden. |
| D1 | FABRICATION-NOTES written in the house format. |
| D3 | README, `hardware/README.md`, `docs/board-versioning.md` and the base_station firmware comments corrected. |
| Title blocks | Beetle-style title blocks (rev V1) on all four schematic sheets. |
| X1 | Mantis antenna fixed the same way, in the owner's Mantis working copy. It lands with the Mantis layout, not with this board; see below. |

**Not done, and why:**

| Item | Why |
|---|---|
| S1 (CLC at LNA_IN) | No legal spot for three 0402s within reach of U3 pin 1. C13/C11, the +3V3 via at (56.625, 148.035) and the CHIP_PU route fill that corner. It needs a re-layout of the S3's RF/decoupling corner. |
| S3 (UART test pads) | Pins 49/50 sit between the crystal lines, R2 and pin 46's decoupling. Any route to a pad crosses Y2's ground ring or the B.Cu LoRa diagonals, and Espressif asks to keep TX away from the crystal. |
| S4 (SPI_CLK series part) | No room between pin 33 and the flash lanes. |
| L6 (C1 next to the flash) | The SPI_HD escape (x 69.07, then y 154.02) boxes in the flash's VCC via. |
| L12, SMD reliefs | That is a board-wide zone setting; accepted, as on the Beetle. |
| L14 (In2 VCC slot) | Moving the line to B.Cu would cross the LoRa bundle. |
| F6 (logo strokes) | Withdrawn: the owner says the logo prints fine. |
| S7 FET, S8, S9, VBUS sense | Optional design changes, left for discussion. |
| D4 (ERC hygiene), F8/F9 (lands), D5 (BOM) | Notes only. |

**Found at packaging (2026-09-28, after the fixes).** The earlier via-in-land check tested via centres. It missed three
vias whose rings overlap the edge of an SMD land. Each hole stays outside the land's copper:

| Via | Net | Land | Hole vs the land's mask opening | Origin |
|---|---|---|---|---|
| (54.20, 149.55) | GND | C24.2, C16.2 | 0.014 mm outside C24's | added for finding L5 |
| (70.50, 109.00) | GND | C20.1 | 0.058 mm outside | added for finding L7 |
| (51.16, 178.13) | VCC | U22.1 | 0.06 mm inside the pad's 0.10 mm mask margin | original layout |

**Fixed the same evening** (owner: "address the vias and any other issues you found"):
- The first moves to (53.52, 149.60), the pocket between C8.1, C8.2, C24.2 and C16.2, with a 0.2 mm stub to the strap.
  GPIO0's B.Cu diagonal, a static strap line, moves 0.5 mm down-right to free that pocket.
- The second moves to (70.97, 109.045) with a 0.25 mm stub to U16 pad 2.
- The third moves to (50.44, 178.25) beside the trace, with a 0.25 mm F.Cu stub to it. The In2 VCC track, which carries
  µA, drops straight down to it.

Afterwards no via ring touches an SMD pad's mask opening except U3's nine. The closest via hole is 0.10 mm from one,
at C88.
DRC is unchanged (0 errors, 26 warnings), with 0 unconnected, 0 parity, and fills equal to a fresh refill.

**Also found at packaging:** BT2's flat base ends at y 187.72, over the first 0.27 mm of J2's two rear shell-leg pads
(y 187.45–189.55). Those legs are soldered from the bottom, so a domed fillet props up that end of the holder. The fab
notes now say to keep those two fillets flat. Check the holder sits flat on the first article.

---

## Verdict

**One blocker, which one refill fixes, and two majors.** One major is a defect in Claude-drawn copper. The other needs a
decision.

- **L1 — the saved B.Cu ground fill is stale** after the 16:17 track edits. As saved, it would plot `U22-ON` shorted to
  GND. **Refill before plotting.**
- **L2 — the BLE antenna sits over copper on In1, In2 and B.Cu.** This is my layout error from this morning. One extra
  rule area fixes it, and the fix is DRC-proven.
- **P1 — the charger heats the board past its own NTC's hot trip at room temperature.** Charging stalls from about 25 °C
  ambient, and above about 32 °C a running base station drains its cell on USB. Options are below; this is the item
  that needs your call.

**The circuit itself checks out** where boards usually go wrong:
- Every IC pinout and footprint numbering matches its datasheet.
- Straps give SPI boot, with Joint Download on the button. VDD_SPI is 3.3 V, as the RH2's PSRAM needs.
- The flash's I/O levels work both ways across the +3V3 / VDD_SPI split.
- The radio's RF switch cannot have both enables high.
- The TS thresholds are what earlier work computed.
- `board_v4.h` matches the netlist constant for constant.
- The J8 feed, crystal layout and decoupling values match their references.

Everything else is minor or a note.

## Summary

| # | Finding | Severity | Who / where |
|---|---|---|---|
| L1 | Stale B.Cu GND fill; `U22-ON` would short to GND | **blocker** | owner: refill |
| L2 | BLE antenna over In1/In2/B.Cu copper (feed notch cut on all layers) | **major** | layout — Claude's error, fix proven |
| P1 | Charger dissipation vs board-mounted NTC: charging stalls from ~25 °C | **major** | decision: R52, TS, wired NTC or procedure |
| L3 | 14 of 41 signal vias have no GND via within 1 mm (5 are Claude's) | minor | layout, sites proven |
| L4 | U3 exposed pad still has no vias (B1 of 08-12) | minor | layout, sites proven |
| L5 | VDD3P3 filter caps lost their ground vias when R1 moved (Claude's) | minor | layout, site proven |
| L6 | C1 is not local to the flash (Claude's) | minor | layout |
| L7 | E220 GND pads 2 and 16 have no nearby via | minor | layout, sites proven |
| L8 | U9 switching loops close through In1; no PowerPAD vias | minor | layout |
| L9 | J8 pin engages 2.37 of 3.5 mm of pad | minor | layout, optional |
| L10 | BT2 + pad is 0.40 mm from J2 GND | minor | layout / assembly note |
| L11 | S1's snap-in posts protrude under the battery holder | minor | mechanical, first article |
| L12 | Pads solid in pours: 39 lopsided 0402s; J2 legs and D6 solid to 3 planes | minor | layout / DFM |
| L13 | Copper under S3's body; paste above the vendor's reference | minor | layout |
| L14 | In2 VCC track slots the +3V3 plane under U3; B.Cu signals ride +3V3 | note | layout, optional |
| S1 | No chip-side CLC at LNA_IN (Espressif requires one; fleet-wide) | minor | schematic proposal |
| S2 | E220 ANT pin lacks the protective shunt inductor Ebyte asks for | minor | schematic |
| S3 | No UART0 or test access | minor | schematic proposal |
| S4 | No series-part footprints on the SPI flash lines | minor | schematic proposal |
| S5 | Charging while the board runs: no power path | minor | docs + firmware; optional VBUS sense |
| S6 | ISET at the datasheet limit: 0.713–0.876 A vs 0.8 A max | minor | schematic (same fix as P1-A) |
| S7 | A reversed cell is possible and unprotected | minor | assembly note / optional FET |
| S8 | CR1's VBUS channel caps VBUS tolerance near 7 V | note | optional |
| S9 | PS/SYNC hard-wired to power-save: no forced-PWM option | note | optional jumper |
| F1–F10 | Footprints, silk, fiducials, mask, pin-1, logo | minor/note | library + layout |
| D1–D6 | Fab notes, JLC placement check, docs drift, BOM, ERC | minor | docs |
| M1–M7 | First-article measurements | — | bench |
| X1–X3 | Mantis antenna defect; library traps | — | other boards |

---

## 1. Layout (the owner's copper; everything here is a proposal)

### L1. The saved B.Cu GND fill is stale — blocker until refilled

- **What:** DRC on the saved board reports clearance 0.000 mm (rule 0.127) between the `Net-(U22-ON)` track on B.Cu
  at (76.07, 153.14) and the GND pour.
  - The 16:17 save moved that track from x 76.25 to 76.07 over (59.91, 169.3)→(76.07, 153.14)→(76.07, 142.16)→(72.97, 139.06).
  - It also re-routed `L_RXEN` at (64.45–67.39, 146.6–149.6).
  - The pour was not refilled after either edit.
- **Refill check:** a refill of a copy differs from the stored fill only on B.Cu GND.
  - 0.42 mm² of the stored pour overlaps the moved track.
  - 2.88 mm² is new copper in the vacated corridor and around the `L_RXEN` edit.
  - After the refill: 0 errors, 0 unconnected.
- **Why:** gerbers are plotted from the stored fill. As saved, the ON pin is shorted to GND, so the board never turns on.
  Pressing S1 would then connect the cell to GND through the switch, and the DW01A would trip at best.
- **Fix:** press **B** (refill) and save, then run DRC before any plot.

### L2. The BLE antenna sits over copper on In1, In2 and B.Cu — major (Claude-drawn, 2026-09-28)

- **What:** the rule area "U15 antenna clearance" covers x 48.15–51.35 × y 147.61–153.20 on all four copper layers. A
  notch at x 48.15–48.80 × y 147.61–149.25 is cut out of it so the feed trace can reach the antenna.
  - That notch is cut on **every** layer. U15, whose feed pads sit at (48.50, 148.55/148.95), lies inside it.
  - Probing the refilled board, In1 GND, In2 +3V3 and B.Cu GND are all present under the antenna body and both feed pads.
  - The copper-free window starts beside the antenna, not under it.
- **Why:** Abracon rev A p.4 puts the feed pads inside the window, 0.30 mm from the ground band. It says the cutout
  "must extend through all layers". A loop antenna with planes 0.21 mm under its feed is detuned by an amount no one can
  predict, and the EVB match values assume the clear window.
- **Other boards:** the Beetle does this right. Its notch is covered again by a second rule area on B.Cu and In1–In6,
  so it is F.Cu-only. The Mantis has the same all-layer notch as this board (X1). I copied the Mantis pattern.
- **Fix (proven):** add a second rule area on In1, In2 and B.Cu over the notch:
  - polygon (48.15, 147.61) (48.80, 147.61) (48.80, 149.25) (48.15, 149.25);
  - no tracks, no vias, no pours.
  After the refill, the antenna and its feed pads have no copper under them on any layer. The GND pads keep the band,
  and DRC shows 0 errors with no new warnings.
- **Optional, closer to the datasheet:**
  - The window is 5.59 mm along the edge against Abracon's 4.60, and 3.20 mm deep against 3.00. The antenna sits
    0.84 mm from the feed-side end (Abracon 0.75).
  - Stitching vias exist on only three sides, five of them within 1 mm of the window.
  - The Beetle's window is 4.59 × 2.89 with 20 vias around it. Shortening the window to ~4.6 mm (end at y ≈ 152.21) and
    adding a via row at x ≈ 51.45 brings it to the EVB shape. The match needs a VNA tune either way (M1).

### L3. 14 of 41 signal vias have no GND via within 1 mm — minor (owner's rule; 5 are Claude's)

- **What:** every signal via changes between F.Cu (over In1 GND) and B.Cu (over In2 +3V3). The committed board had
  11 vias over 1 mm; this one has 14.
  - My flash re-route removed the GND via at (65.14, 150.54). That pushed three LoRa vias further out:
    - `L_RXEN` (64.45, 149.56) 1.20 → 1.93 mm;
    - `L_BUSY` (64.29, 150.18) 0.93 → 1.64;
    - `L_MISO` (64.17, 150.84) 1.02 → 1.46.
  - My two flash hop vias are at 1.10 mm (`SPI_CLK`, 64.40, 152.82) and 1.34 mm (`SPI_WP`, 64.40, 153.62).
  - The others are:
    - `GPIO0` (50.42, 143.73) 2.70; `Net-(Q1-G1)` (52.99, 114.32) 2.15;
    - `L_RST` (63.07, 147.90) 1.90; `Volt_Read` (50.34, 176.25) 1.80;
    - `L_SCK` (72.34, 120.02) 1.66; `CHIP_PU` (55.76, 150.50) 1.32;
    - `L_MOSI` (63.17, 156.33) 1.19; `D-` (62.32, 188.94) 1.11; `Net-(D9-K)` (64.29, 178.20) 1.09.
- **Fix (proven):** nine GND vias (0.45/0.3) bring nine of these to 0.56 mm:
  - (64.35, 149.01) L_RXEN · (64.42, 151.34) L_MISO · (63.32, 148.40) L_RST
  - (49.87, 143.63) GPIO0 · (50.84, 176.00) Volt_Read · (71.79, 119.92) L_SCK
  - (63.72, 156.43) L_MOSI · (61.77, 188.84) D- · (63.74, 178.10) D9-K
- **What remains after that:** Q1-G1 (a static protection gate), CHIP_PU (static), L_BUSY at 1.17, and the two SPI hop
  vias. SPI_Q/CS0/HD sit at 0.4 mm pitch there, so there is no legal site without re-routing.
- **Stackup note:** on this S-G-P-S stack a B.Cu trace returns on the **+3V3** plane. The GND via ties In1 to the B.Cu
  pour; the plane-to-plane hop goes through the nearest decoupling cap.

### L4. U3's exposed pad still has no vias — minor (B1 of the 2026-08-12 review, still open)

- **What:** there are no vias inside pad 57 (58.03–62.13 × 149.77–153.87). Fifteen GND vias ring it 0.09–0.43 mm outside
  the pad edge, all four sides, joined through the F.Cu pour.
  - HWDG PCB layout: "The ground pad at the bottom of the chip should be connected to the ground plane through at least
    nine ground vias."
  - The fabbed parent board works with the ring, so this needs a recorded decision rather than a rescue.
- **Fix (proven):** nine GND vias 0.45/0.3, all in the gaps between the nine paste windows (none under paste), at least
  1.06 mm apart so the In2 antipads don't merge:
  - (58.575, 151.07), (58.575, 152.57)
  - (59.325, 151.82), (59.325, 153.32)
  - (60.075, 151.07), (60.075, 152.57)
  - (60.825, 150.32), (60.825, 151.82), (60.825, 153.32)
  Avoid (59.325, 150.32): the In2 VCC track passes 0.015 mm from it.

### L5. The VDD3P3 filter caps lost their ground vias when R1 moved — minor (Claude-drawn)

- **What:** moving R1 to make room for C24 removed the GND via column at x ≈ 52 that the committed board had:
  (52.00, 149.61), (52.01, 148.91), (52.02, 150.38), (52.06, 151.07).
  - C16.2's nearest reachable GND via is now 2.46 mm away, C24.2's 2.04 mm and C8.2's 2.07 mm.
  - HWDG PCB layout asks for "ground vias … close to the capacitor's ground pad" and, at pins 2/3, "maximizing the
    placement of ground vias".
- **Fix (proven):** a GND via at (54.20, 149.55) on the C24.2–C16.2 ground strap.

### L6. C1 is not local to the flash — minor (Claude-drawn)

- **What:** C1.1 at (70.65, 151.72) is 2.2 mm from U1's VCC ball B2 (68.55, 152.45), but the two meet only through the
  In2 plane:
  - B2 escapes 1.5 mm to a via at (68.50, 153.55); C1 has its own via at (70.65, 151.00), 3.3 mm away.
  - C1.2's nearest GND via is 2.2 mm away.
  - The Beetle's caps on the same part are 1.35 mm from B2.
- **Why:** at 80 MHz the flash's supply loop runs through two vias and about 6 mm of the In1/In2 pair.
- **Fix:** move C1 into the free pour right of column 1, below row A (x 69.4–71.0, y 153.1–154.0), sharing or sitting
  beside the (68.50, 153.55) via, with a GND via at pad 2. Or accept it.

### L7. The E220's GND pads 2 and 16 have no nearby via — minor

- **What:** pad 2 (71.01, 109.79) is 2.86 mm from the nearest GND via and pad 16 (74.37, 116.98) is 4.11 mm. Pad 2 is
  the return for VCC, which carries 118 mA at +22 dBm (LLCC68 Table 3-6).
- **Fix (proven):** GND vias at (70.50, 109.00) and (75.25, 116.98).

### L8. U9's switching loops close through In1, and the PowerPAD has no vias — minor

- **What:** pad 15 has no vias inside it. Four GND vias sit 0.48–0.73 mm off its tabs: (51.35, 167.80),
  (51.87, 167.76), (50.44, 172.65), (50.71, 173.40). That is much better than the parent board's 2.89 mm.
  - C59's GND pad (48.74, 168.69) is walled off from PGND on F.Cu by the switch-node and V_SWITCH pours. It returns
    through (48.61/49.24, 167.67) → In1 → (51.35/51.87, 167.8), a loop of about 3.0 × 2.9 mm.
  - The output caps' GND pads (x 54.7–58.7, y ≈ 168.1) are cut off by the L2 zone and return through (55.03, 166.97).
  - TPS63020 §10.1 asks for input and output caps "as close as possible" to the VIN/VOUT and PGND pins.
- **Why:** edge ringing and EMI on a board whose job is weak-signal LoRa reception. In1 at 0.21 mm keeps it tolerable.
- **Fix, if you touch it:**
  - An 0402 100 nF–1 µF V_SWITCH→PGND cap at about (49.6, 171.8), its GND end at the (50.44, 172.65) via.
  - An 0402 +3V3→GND cap at about (54.15, 170.9). This needs the FB trace at y 171.0 re-routed.
  - In-pad vias would need the four paste windows reshaped: the gaps between them are 0.19–0.25 mm, narrower than a
    via, so skip them.

### L9. J8's pin engages only 2.37 mm of its 3.5 mm pad — minor, optional

- **What:** the edge-launch SMA's pads start 1.44 mm inside the edge (pad 1 y 105.08–108.58; edge y 103.64).
  - The flange butts the edge, and the pin reaches 3.81 mm from the flange face (DS-SMA-EDGE-4 p.2). It lands on
    2.37 mm of pad.
  - The first ~1 mm of pin lies on solder mask over the F.Cu GND pour (y 103.94–104.93).
  - The Beetle has the same footprint 1.435 mm in, and the fabbed LoRa board 1.30 mm in; both work.
- **Why:** a shorter joint, and a small shunt capacitance, about 0.25 pF (≈ 700 Ω at 915 MHz), so RF-wise negligible.
  The real risk is a mask breach shorting the feed.
- **Fix, if you want full engagement:** move J8 to (62.885, 105.69) (−1.14 mm) and extend the feed. The pads then start
  0.30 mm from the edge. Check the fence vias at x 61.56/64.26 afterwards. Keeping fleet parity is also reasonable.

### L10. The battery's + pad is 0.40 mm from a J2 ground pad — minor

- **What:** BT2.P (VCC, Ø3.4 THT at (58.845, 184.915)) sits 0.40 mm from J2 A1/B12 (GND) and 0.80 mm from J2's VBUS
  pad on F.Cu, inside J2's courtyard.
  - BT2 is a B-side part, so P is hand-soldered on top after J2 is reflowed.
  - The parent board's gap was 0.88 mm.
- **Why:** a bridge is a dead short of the cell.
- **Fix:** trim J2's twelve SMD pads at the rear to HRO's own land (rear end 0.53 mm shorter), which gives ≈ 0.86 mm. Or
  shrink BT2.P toward Ø3.0 on that side. At minimum, add an assembly caution.

### L11. S1's snap-in posts protrude under the battery holder — minor (mechanical)

- **What:** S1's two Ø1.9 NPTH posts at (59.03, 134.45) and (59.03, 138.85) sit inside BT2's body outline on the back
  (x 48.4–69.3, y 109.8–187.7).
  - The vendor model puts the post tips 2.81 mm below the top surface, so about 1.2 mm protrudes below a 1.59 mm board.
  - MYOUNG's drawing shows a flat base. DRC is silent because `npth_inside_courtyard` is set to ignore.
- **Fix:** add an assembly note to trim S1's posts flush before fitting BT2 (S1 is held by its four soldered pins). Or
  confirm on the first article that the holder floor clears them.

### L12. Pads solid in pours — minor (DFM)

- **What:** every zone connects pads solid (FULL).
  - **Two-pad parts:** 39 of the 64 two-pad 0402/0603 parts have one pad solid in the 2107 mm² F.Cu GND pour and the other
    on a track. That includes the CC Rd pair R14/R15, L4 and the crystal caps. It is the tombstoning asymmetry the
    Beetle review flagged.
  - **Through-hole:** J2's four shell legs and D6's cathode are solid to GND on F.Cu, In1 and B.Cu.
  - **J2 shell legs also get no paste:** J2 is `attr smd`, and its plated slots S1–S4 have no paste layer. So an
    SMT-only build leaves the charging port held by its 0.3 mm signal pads alone. The footprint's own description says
    to fill those holes.
- **Fix:**
  - Thermal-relief connection for SMD pads on the GND pour. Or accept it: 80 µm paste and convection reflow usually cope.
  - For J2, choose paste-in-hole apertures on S1–S4 or "hand/selective-solder S1–S4, barrels filled", and write it into
    the fab notes (D1).

### L13. Copper under S3's body, and more paste than Mitsumi specifies — minor

- **What:** Mitsumi's spec marks a 3.2 × 2.2 mm "no pattern / no silk" area under the R-667995 body. Its reference
  stencil is 2 × 0.42 × 0.60 per terminal, against the board's full 0.72 mm² pads (+44 %).
  - Between the lands (x 49.15–51.95, y 142.3–144.5) there is F.Cu GND pour, a `GPIO0` track and the `GPIO0` via at
    (50.43, 143.73).
  - Mitsumi warns about flux ingress and float.
- **Why:** this is the boot button.
- **Fix:** a 3.2 × 2.2 mm F.Cu keep-out centred on S3, with the via moved out. Optionally use the vendor's split paste.

### L14. The In2 VCC track slots the +3V3 plane, and B.Cu signals ride +3V3 — note

- **What:** a 0.5 mm VCC track on In2 runs from (51.16, 178.13) along x 47.98, diagonally under U3 at y 149.83, and on
  to S1 pin 4 (70.43, 139.60). It carries only µA: S1's ON-pin current and the DW01A supply. It leaves the plane one
  piece, but C-shaped.
  - Crossing that slot on B.Cu are the LoRa SPI bundle (`L_CS`, `L_MOSI`, `L_MISO`, `L_BUSY`, `L_RXEN`, `L_RST`, and
    `L_DI01` 4×), plus `CHIP_PU` and `GPIO0`.
  - B.Cu signal lengths run up to 63 mm (`L_DI01`). USB D+/D− have 35/46 mm on B.Cu, over +3V3.
- **Fix, optional:** move that µA line to a 0.2 mm B.Cu track and give In2 back to the plane. Full-speed USB and
  ≤ 16 MHz SPI tolerate it as is.

---

## 2. Schematic, footprints and library

### S1. No chip-side CLC match at LNA_IN — minor (proposal; the Beetle and Mantis are the same)

- **What:** the path is U3.1 → 4.8 mm of line → L4 shunt 2.2 nH → C7 series 5.1 pF → C5 (DNP) → antenna. That is
  Abracon's antenna match only.
  - HWDG PCB layout: "A CLC matching circuit is required for chip tuning … place them close to the pin".
  - HWDG schematic checklist: "The CLC structure is mainly used to adjust the impedance point and suppress harmonics".
    The tuning target is S11 ≈ 35 + j0 with S21 < −35 dB at 4.8 and 7.2 GHz.
  - With ideal parts, the existing network passes 4.88 GHz at −0.4 dB.
- **Why:** 2nd and 3rd harmonics reach the antenna essentially unfiltered. That is an emissions and compliance question,
  not a link problem, since the Beetle's BLE works the same way. Footprints can't be added after fab.
- **Fix:** three 0402 footprints (shunt C / series L / shunt C) within ~1.5 mm of U3.1 on the 135° segment
  (56.63, 149.02)→(55.45, 147.84), fitted per Espressif's value range and tuned on a VNA. Or record a decision to ship
  without them, fleet-wide.

### S2. The E220 antenna pin lacks the protective inductor Ebyte asks for — minor

- **What:** Ebyte manual §5.1 shows L1 from ANT to GND. "The L1 inductor is a protective device used to prevent the
  device from being damaged due to excessive input power of the antenna. The user should add this inductor when using
  the module."
  - `Net-(U16-ANT)` has only J8.1 and U16.6.
  - The LoRa daughterboard's review left the same item open.
- **Fix:** an 0402 shunt footprint from the trace near (64.3, 111.1) to the pour, with a GND via within 0.5 mm, at the
  value Ebyte's circuit shows.

### S3. No UART0 or test access — minor (proposal)

- **What:** U0TXD/U0RXD (pins 49/50) are unconnected, and there is no pad for +3V3, GND, VCC, CHIP_PU or GPIO0.
  - HWDG download guidelines: "It is recommended to retain the UART download interface, as the current RF test firmware
    only supports the UART interface."
  - The console is USB-Serial-JTAG only.
- **Why:** if USB doesn't enumerate (J2, CR1, R3/R4 or assembly), there is no log and no download path, and the crystal
  can't be trimmed Espressif's way.
- **Fix:** ≥1 mm SMD pads for GPIO43, GPIO44, GND and +3V3, optionally CHIP_PU and VCC. Route TX away from the XTAL
  lines at x 58.3–58.8.

### S4. No series-part footprints on the SPI flash lines — minor (proposal)

- **What:** all six SPI nets run U3→U1 directly. HWDG asks for zero-ohm footprints in series on the off-package flash
  lines, and an R/C option on CLK.
  - The flash runs at 80 MHz, and 31 × 80 MHz = 2480 MHz, the top BLE channel.
- **Fix:** a 0 Ω footprint on SPI_CLK within ~1 mm of pin 33 (needs a re-route). Or accept it, with a 40 MHz flash
  fallback in firmware if BLE desense shows up.

### S5. Charging while the board runs: no power path — minor (docs, firmware; optional hardware)

- **What:** the switched load hangs on VCC, the same node as the cell and the charger output.
  - **Termination:** the charger terminates at 9–11 % of ISET, i.e. 64–96 mA (§8.4.8). The running board draws about
    55–90 mA from VCC, so termination is a per-unit coin-toss. Until it happens, the cell floats at 4.2 V.
  - **Safety timer:** the 10-hour fast-charge timer (§8.4.7) then ends charging. It is reset only by disabling the IC,
    cycling power or passing through TTDM. R53 blocks TTDM and nothing pulls TS low, so only re-plugging USB re-arms
    it. Whether a VRCH refresh restarts it is not stated — bench-check.
  - **Precharge:** precharge is 18–22 % of ISET with a 30-minute timer. With the board on, a flat cell's load takes most
    of that, so precharge can time out.
  - **No cell:** without a cell, the board runs from precharge current plus a battery-detect routine. Expect no boot or
    hiccups.
  - **Visibility:** firmware can see neither VBUS nor /CHG. On USB, `Volt_Read` reports the charger's output, not the
    cell.
- **Why:** a full cell carries a launch day after the timer ends. Multi-day or overnight USB operation drains it. P1
  dominates at warm ambient.
- **Fix:**
  - Document "USB charges the cell; it does not run the base station". Advise charging with S1 off.
  - Firmware: don't treat `Volt_Read` as the cell's resting voltage while charging.
  - Optional: a VBUS-present divider into a free ADC pin, so the app can say "USB present, not charging".

### S6. ISET sits at the datasheet limit — minor

- **What:** R52 680 Ω ±1 % with K_ISET 490/540/590 AΩ gives 0.713/0.794/0.876 A.
  - The recommended range is R_ISET ≥ 0.675 kΩ, I_IN ≤ 0.8 A and I_OUT ≤ 0.8 A (§7.3). At −1 % R52 is 673 Ω.
  - CC carries only Rd, so the board can't tell a 500 mA port from a 3 A one.
- **Fix:** the same change as P1-A.

### S7. A reversed cell is possible and unprotected — minor

- **What:** the MYOUNG holder has identical contacts at both ends, with ⊕/⊖ moulded only into the housing. Being
  symmetric, it can also be fitted rotated.
  - After assembly, the only visible board mark is the B.Silk "+" at (52.08, 191.73). The "−" at (53.64, 110.47) and
    BT2's own marks sit under the holder.
  - Reversed, the one guaranteed current path is the DW01A's substrate diode plus R55: about 35 mA, about 0.12 W in an
    0402 (inferred; no vendor states reverse behaviour). Whether U5 and U22 are exposed depends on how the unpowered
    protection FETs sit.
- **Fix:**
  - An assembly note fixing the holder's orientation (⊕ toward P at (58.845, 184.915)).
  - A visible "−" outside the holder (y ≈ 108.5).
  - A keyed enclosure.
  - Design option: a P-channel MOSFET in the BT2.P→VCC lead, gate to GND.

### S8. CR1's VBUS channel limits VBUS tolerance to about 7 V — note

- **What:** the SP05 has a 5.5 V standoff, 7.0–8.5 V clamping and a 0.225 W package. The BQ21040 IN is rated 30 V with
  6.5–6.8 V OVP. A sustained source above ~7.5 V would burn CR1, typically short, where U5 alone would sit in OVP.
- **Fix, optional:** take CR1 pin 2 off VBUS, or use a VBUS clamp with ≥ 6.8 V standoff. C27 already takes VBUS ESD.

### S9. PS/SYNC is hard-wired to power-save — note

- **What:** below about 100 mA the TPS63020 bursts, holding +2.5 to +3.5 % (§7.4.4). The base station's 60–100 mA sits
  right at that boundary, and the effect on the LoRa receiver is unquantified.
- **Fix, optional:** a solder-jumper option so pin 13 (PS/SYNC, (50.32, 171.50)) can tie to pin 12 (EN = V_SWITCH,
  (50.32, 171.00)) for forced PWM. That enables an A/B sensitivity test (M3).
  - The same jumper matters for eFuse burning. Power-save raises +3V3 up to +5 % (3.54 V worst case), and eFuse
    programming wants VDD3P3_CPU ≤ 3.3 V (DS Table 5-2 note 3).

### Footprints, silk and library

- **F1. U3's solder-mask webs are 0.070 mm on all 55 pin gaps — minor.** The fp_poly mask openings are 0.76 × 0.33 mm on
  0.4 mm pitch, below JLC's 0.10 mm bridge. The fab will gang them. Either shrink the openings to ≤ 0.30 wide, or
  write "gang U3's opening, don't shrink pads" into the fab notes. The Beetle was built with the same footprint.
- **F2. The library `IC_ESP32-S3` pastes pad 57 twice — minor, latent.**
  - The nine F.Paste fp_poly windows (1.0 mm on 1.5 mm, 55 %) have been there since import. Commit 4078070b (2026-09-04)
    added nine more paste-only pads (1.06 mm on 1.36 mm), because a pad-only scan missed the polygons.
  - No board carries both: the Beetle U15/U32, Mantis U15, LoRa U28 and this U3 all have the polygons only.
  - The next "update footprint from library" would stack the two arrays (≈ 69 %, merged L-shapes). This is also the
    whole of U3's `lib_footprint_mismatch`.
  - Fix: delete the added pads from the library, then check `QFN50P300X300X100-17N`, which got the same treatment.
    Don't update U3 until then.
- **F3. The U15 footprint promises a clearance drawing it doesn't contain — minor.** Its description says "The
  4.60 x 3.50 mm rectangle on Dwgs.User is the required ground clearance". The file has no Dwgs.User item, and the three
  boards drew three different keep-outs. Add the outline, or better, footprint-level rule areas in the Beetle's
  two-layer-set pattern.
- **F4. Pin-1 marks — minor.**
  - U3 has only corner brackets. U5 and U22 (library `SOT95P280X145-6N`) have no pin-1 mark on silk or fab. CR1 relies on
    its wide pad. The E220 (U16) has no mark at all.
  - A 180° error swaps U5 VIN↔TS and U22 VIN↔VOUT unseen.
  - Fix: add a ≥ 0.15 mm dot in each library footprint. On this board: U3 (56.65, 149.22), U5 (62.40, 179.52),
    U22 (51.16, 177.34), CR1 (68.14, 180.93), U16 near (75.2, 109.0).
- **F5. No fiducials — minor.** The board carries a 0.4 mm-pitch QFN and a 0.5 mm WLCSP. Three are free at
  (53.9, 108.1), (74.4, 121.6) and (51.4, 187.6), clear of copper, silk and edge; DRC them after placing.
- **F6. The logo — withdrawn.** This flagged the logo's fine strokes against JLC's 0.15 mm silk minimum. The owner says
  the logo prints fine (2026-09-28). Its library nickname `footprints` isn't in fp-lib-table; that is the harmless
  `lib_footprint_issues` warning.
- **F7. Silk warnings that matter — minor.**
  - R13 and R16 show their references on their own pads and on each other. The fab will clip them to fragments; hide
    them, as the rest of the board does.
  - 214 silk lines are drawn at 0.12–0.127 mm, under JLC's 0.15. U1's outline is 0.05 mm, so only its 0.26 mm pin-1 dot
    will print.
  - The rest are cosmetic: edge-clipped J2/J8/S1 outlines, 0402 overlaps, and symmetric "+"/"−" glyphs flagged as
    non-mirrored.
- **F8. CR1's land is smaller than Littelfuse's recommended layout — note.** The pads are 0.85 × 0.56 on 2.39 mm
  centres, against 1.0–1.2 × 0.8–1.0 on 2.20. The heel fillet is about zero.
- **F9. Y1's land is short of Abracon's ABS07 recommendation — note.** It is 0.85 × 1.7 with a 1.45 gap against
  1.1 × 1.9 with a 1.4 gap, so the terminal ends overhang. Firmware doesn't use Y1 anyway (see the firmware section).
- **F10. J2 sits 0.65 mm further out than HRO's layout — note.**
  - The KiCad land's pads run 0.53 mm further rearward than HRO's, and the front slots are 1.2 mm against HRO's 1.4 mm.
  - The plug still seats.
  - The B.Paste layer holds only J8's two bottom ground pads. Say "top stencil only" or drop B.Paste from J8.

---

## 3. P1 — the charger heats the board past its own NTC — major (decision)

**What:**
- **Dissipation:** U5 (BQ21040 at (63.35, 178.27)) dissipates (VBUS − V_cell) × I at the programmed 0.794 A. From a
  5.0 V port that is:

  | Cell voltage | Dissipation |
  |---|---|
  | 3.0 V | 1.59 W |
  | 3.7 V | 1.03 W |
  | 4.2 V | 0.64 W |

  At 5.25 V add about 0.2 W.
- **Package:** RθJA is 130.8 °C/W and thermal regulation starts at 125 °C. TI §11.3 "recommended that the design not run
  in thermal regulation for typical operating conditions".
- **Board model:** a steady-state four-layer model built from the real copper, in still air with no enclosure, gives
  about 19 K per watt at TH1. The board reads essentially its mean temperature, because the solid planes spread the heat.
  The README's distance-weighted placement argument does not hold on this board.
- **Results at 25 °C ambient:**
  - U5 folds back to about 0.89 W.
  - Charge current is 0.56 A at a 3.4 V cell and 0.69 A at 3.7 V.
  - TH1 settles near **42 °C with the board off**, against a 40.6 °C hot trip (37.8–44.0 over tolerance; it re-arms at
    38.5 °C).
  - With the board on, add about 6 K.
- **Average charge current** as TH1 cycles, at a 3.7 V cell:

  | Ambient | Board off | Board on, net into the cell |
  |---|---|---|
  | 25 °C | 0.59 A | +0.24 A |
  | 30 °C | 0.39 A | +0.04 A |
  | 35 °C | 0.18 A | −0.09 A |

  Above about 32 °C with the board on, charging never re-arms, and the load drains the cell with USB plugged in.
- **The LED:** the blue /CHG LED stays lit through a temperature fault on the first cycle (Table 1), so it looks like
  charging.
- **In an enclosure:** every threshold drops by about 15 °C.
- The model is ±25 %, but the direction is simple physics: about 1 W into a 27 cm² board raises it about 15–19 K.

**Why:** topping up the base station from a power bank at a warm launch field is the use case.

**Options:**
- **A.** R52 680 Ω → **1.2 kΩ**: 0.40–0.50 A, P ≤ 0.72 W. There is then no thermal regulation at 25 °C, TH1 rises only
  11–14 K, and S6 is fixed too. On its own it still trips from about 30 °C, or about 24 °C with the board on.
- **B.** 680 Ω–1 kΩ in series with TH1 on the 19 mm TS run. That moves the hot trip to 44–46 °C, and the cold trip by
  under 1 °C. It trades away some of the cell's hot margin; standard cells allow 45 °C charging, so check the chosen cell.
- **C.** The README's own fallback: a wired NTC on the cell. It is the only option that fixes charging with the board on
  above about 30 °C.
- **D.** Procedure: "charge with the base station switched off", plus the S5 note.

**Recommendation:** A now. Then run the first-article test the README already lists at 25 and 35 °C, with S1 both off
and on (M2), and decide C (or B) from those numbers. Don't improve U5's heatsinking without A: better cooling only lets
U5 dissipate more before regulating, which heats TH1 more.

---

## 4. Measure on the first article

- **M1. BLE match.** Fit the cell, holder, S1 and enclosure: the holder's body covers 2.9 of the window's 3.2 mm depth
  from behind. Lift C7, fit a pigtail at the C5.1/C7.1 node, and tune C5/C7/L4 for S11 < −10 dB over 2400–2483.5 MHz
  after L2's fix. If S1 (the CLC) goes in, check S11 ≈ 35 + j0 and S21 < −35 dB at 4.8/7.2 GHz. Then compare
  over-the-air RSSI with a Beetle.
- **M2. Charging thermals (P1).** Log TS voltage (TH1), U5's case temperature and the cell voltage while charging from
  5.0 V. Run at 25 and 35 °C, with S1 off and on, and note when /CHG lies. Probe the cell as well; that is also the
  README's cold-side check.
- **M3. LoRa, conducted at J8.** TX power at 915.0 MHz, harmonics at 1830 and 2745 MHz, and ±5 MHz for switcher
  sidebands (the TPS63020 runs at 2.2–2.6 MHz). Measure the RX noise floor with the S3 idle, with BLE active, and on
  USB. If S9's jumper exists, A/B power-save against PWM.
  - The 40 MHz crystal's 23rd harmonic is 920 MHz, 5 MHz from the 915 MHz channel; revisit if the channel moves.
- **M4. Power cycling.** Toggle S1 off/on 50 times at 0.2–1 s and confirm every boot.
  - The CHIP_PU RC (10 ms) meets the datasheet with V_SWITCH rising in 3.6–5.4 ms.
  - HWDG warns that an RC alone can miss slow or frequent power cycles; the fallback is a ~3.0 V open-drain supervisor.
- **M5. Crystal.** Y2's effective load is estimated at 7–8 pF against its 10 pF CL, so it runs fast. Measure the ppm
  offset and trim C3/C6 if needed. The measure-and-trim decision already stands.
- **M6. No-power-path behaviour (S5).** Leave the board on USB with S1 on past the 10-hour timer, and plug USB into a
  flat cell with S1 on.
- **M7. Mechanics.** Check the holder seats flat over S1's posts (L11), the SMA seats flush (L9), and the USB plug seats
  (F10).

---

## 5. Release package

- **D1. FABRICATION-NOTES records only the stencil — minor, blocks the order.** Add, from the board:
  - Stackup `JLC04161H-7628`, 1 oz outer / 0.5 oz inner, 1.586 mm.
  - ENIG.
  - Vias: 0.45/0.3, tented, none in SMD pads.
  - Controlled impedance on the RF class (0.36/0.15 GCPW on F.Cu over In1: `Net-(U16-ANT)`, `Net-(C5-Pad1)`,
    `Net-(U3-LNA_IN)`), and "do not fill the U15 clearance on any layer".
  - Minimum track/space 0.10 (U1 escape) and annular 0.075.
  - Mixed technology: S1, D6, D9 THT top; BT2 THT bottom; J2 legs (L12); J8 edge part.
  - Assembly block: holder orientation (S7), S1 post trim (L11), J2 legs, J8 hand-fit, C5 DNP, gang U3's mask (F1),
    top stencil only (F10), routed tabs not V-score on the left edge (S1 overhangs 5.4 mm).
  The JLC package recipe expects a fenced "paste verbatim" block; there isn't one yet.
- **D2. Check JLC's placement preview for U1 — minor.**
  - The WLCSP ball array is 180°-symmetric, so a part rotated 180° still fits. It puts VCC (B2) on the GND land (E3).
  - Only U1's 0.26 mm pin-1 dot will print. A1 is the pad at (69.35, 152.95), bottom-right as placed.
  - The LoRa dead-flash history is exactly this class of fault.
- **D3. Docs drift — minor.** Several of these are my own text from this morning.
  - **README BLE bullet:** it says "0.5 mm deep" band and "4.6 × 3.5 mm". As built, the band is 0.75 mm from the edge
    and the window is 5.59 × 3.20 with the notch. Fix the text after L2.
  - **README Status:** "no DRC errors" is true only after L1's refill.
  - **`docs/board-versioning.md`:** the Tinker-Base flash row says revision "first article"; it should say "V1 (not
    fabricated)".
  - **`hardware/README.md`:** it still says the board "has no map of its own yet" (`board_v4.h` landed in #1499), and
    that the silk reads "TinkerRocket Base Station Mini" (it now reads "TinkerRocket / Base Station V1").
  - **2026-08-12 review:** its "Verified correct" section is superseded. The Molex U15 "needs no keep-out" and the
    fixed-output TPS63021 are both gone.
  - **Firmware comments:** `CMakeLists.txt`, `sdkconfig.defaults.v3` and `partitions_v3.csv` still call U1 a
    GD25Q128ESIG. The size is the same, so builds are unaffected.
- **D4. ERC hygiene — note.** 405 items:
  - 275 off-grid, 90 library-symbol mismatches.
  - 28 unconnected pins with no no-connect flags: U3 spares, J2 SBU, U16 DIO3, U9 PG.
  - 5 "power pin not driven": +3V3, VBUS, VCC, B− and `Net-(U14-VCC)` lack PWR_FLAGs.
  - 2 output-to-output pairs, both legitimate: U22 QOD = VOUT, U9's VOUT pins.
- **D5. BOM — note.**
  - All 81 fitted refs match the netlist on value, footprint, MPN and DNP.
  - Only 5 of 45 lines carry LCSC numbers.
  - J8's manufacturer is "generic", but the land is specific to RF Solutions' 1.65 mm-slot, 3.81 mm-prong part.
  - R-667995 is listed as discontinued by distributors (fleet-wide), with a 6-month use-by note.
  - Export positions with `--exclude-dnp`.
- **D6. Before tagging:**
  - L1 refill, and `kicad-cli pcb drc --severity-error --schematic-parity` clean.
  - `tools/check_board_parity.py`. A path-link check is being added to it in a separate session; until it lands,
    prove every footprint path against a fresh netlist.
  - Bump nothing (V1, never fabricated), tag `tinker-base-v1.0.0`, and plot with `tools/plot_gerbers.sh` from a refilled
    board.

---

## 6. Firmware and system

- **Floating pins.** GPIO3, GPIO9–14, GPIO36/37, MTDO/MTDI/MTMS and GPIO47/48 are unconnected, input-enabled after reset
  and unpulled. HWDG asks for them to be pulled. Enable internal pulls in the board-4 init.
- **Straps.** Never burn `STRAP_JTAG_SEL`: GPIO3 is ignored only while it is 0. GPIO45 relies on its ~45 kΩ internal
  pull-down; a misread would select 1.8 V VDD_SPI, which is no boot here. Burning VDD_SPI_FORCE = 3.3 V is the belt and
  braces; see S9 for the supply condition.
- **Y1 is unused.** `CONFIG_RTC_CLK_SRC_INT_RC` means Y1, C2 and C4 are fitted but unused. Either use them or mark them DNP.
- **Battery sense** is fine for a low-battery warning: ±120 mV RSS at the cell, and allow 0.25 s after switch-on
  (τ = 50 ms). On USB, it reads the charger, not the cell (S5).
- **Recovery without a reset button:** hold S3 and switch S1 on. S1 cuts +3V3 even with USB plugged in. USB
  auto-download works while the app keeps USB-Serial-JTAG.

---

## 7. Outside this board

- **X1. The Mantis U22 antenna has the same keep-out defect as L2.**
  - Its all-layer notch at 73.26–73.69 × 136.52–138.07 lets GND, In3 +3V3 and In4 V_MCU_SWTCH fill under the feed pads,
    on both the committed file and today's 16:42 main save.
  - It also has no fence vias.
  - Fix it with the Beetle's two-area pattern.
- **X2. Library.** The `IC_ESP32-S3` double paste (F2) and the same treatment on `QFN50P300X300X100-17N`; U15's missing
  clearance drawing (F3); no pin-1 marks in `SOT95P280X145-6N` (F4).
- **X3. Record correction.** The L9 inductor's 2.83 A figure quoted in the parent's records is TDK's thermal current.
  Isat is 1.70 A rated and 1.89 A typical. The worst-case peak is about 0.9 A (1.08 A with TI's +20 %), so there is
  still 1.6× margin.

---

## 8. Checked and fine

**Pinouts:**
- U5, U9 (TPS63020 on the shared DSJ land), U14, U22, Q1–Q3, CR1 (pin 1 = common anode on the wide lead), U1 (all eight
  balls, W25Q128JV Fig 1e), U15 and U16 (all 20 pins, Ebyte §3) match their datasheets and footprints.
- J2's pad map matches the HRO drawing. The BOM matches the netlist ref for ref.

**MCU:**
- **Boot:** GPIO0 has its pull-up + R5 (1); GPIO46 has its pull-down + R2 (0). That is SPI boot, and S3 selects Joint
  Download. GPIO45 floats to 0, so VDD_SPI = 3.3 V, which the RH2 needs. SPICS1 (PSRAM CE) is correctly left open.
- **Flash levels:** S3→flash gives VOH ≥ 2.5 V against VIH 2.30 V. Flash→S3 gives VOH ≥ 3.08 V, within VDD_SPI + 0.3 V.
  /CS follows SPICS0's reset pull-up. The firmware never powers VDD_SPI down.
- **Decoupling:** values match the HWDG reference. L3 (2.0 nH, 900 mA) exceeds HWDG's 500 mA. Pin-to-cap distances are
  1.2–1.8 mm.
- **Crystals:** Y2 is 40 MHz, 10 pF, ±10 ppm, ESR ≤ 40 Ω. L2 24 nH is HWDG's initial value. The crystal layout has no
  vias, solid In1 under it, and nothing foreign within 0.5 mm; it is identical to the fabbed parent.
- **USB:** GPIO20 = D+, GPIO19 = D−. R3/R4 sit 2.7–3.0 mm from the pins. The pair solves to 95–107 Ω differential
  against the 90 Ω target, which is fine at full speed.
- **Firmware:** every `board_v4.h` constant matches the netlist. TR_BS_BOARD=4 selects 16 MB, DIO at 80 MHz, and a
  partition table ending at exactly 0x1000000.

**Radio:**
- **Enables:** GPIO35/38 have no reset pulls, so R6 holds RXEN low. DIO2/TXEN is low through reset and high only in TX.
  RadioLib drives RXEN low for TX and idle, so both enables are never high together.
- **Module:** it uses a 32 MHz crystal, not a TCXO, so DIO3 is correctly left open. 3.28 V avoids the LLCC68's
  low-supply power drop.
- **Layout:** C20 22 µF sits 1.65 mm from VCC and C22 100 nF 3.23 mm, both with vias. Only DIO1, DIO2 and NRST run
  under the module, on B.Cu, with F.Cu and In1 solid.
- **Feed:** 0.36 mm, 5.99 mm long, one 45° bend, solid In1 under it, 0.150 mm minimum gap, 19 fence vias at about
  0.85 mm pitch. It solves to 45 Ω (with copper thickness), which is harmless at 11°. It fits the 1.2–1.6 mm board range.

**Power:**
- **TS thresholds:** hot 40.6 °C (37.8–44.0), cold 0.25 °C (−1.3 to +2.3), both matching earlier work.
- **DW01A:**
  - Values are the application values; R57 is 2 k against the app note's 1 k.
  - Over-charge 4.25–4.35 V clears the 4.16–4.23 V charger.
  - Over-discharge 2.40–2.60 V clears the TPS63020's 1.8 V minimum.
  - Over-current trips at about 6–13 A across three FS8205A pairs; the FETs are rated 6 A continuous and 24 A pulsed.
- **TPS22918:** rise 3.3–4.6 ms, inrush about 20 mA, QOD legal, ≤ 60 mV drop at 0.8 A.
- **TPS63020:**
  - +3V3 is 3.19–3.37 V in PWM.
  - Output caps: 4 × 22 µF plus the rest, meeting TI's Table 1 for 2.2 µH.
  - Input: 22 µF.
  - Switch-node copper is 3.7 and 2.9 mm², on F.Cu over In1. The FB divider is shielded by the +3V3 zone.
  - L9's worst-case peak is about 0.9 A against a 1.70 A Isat.
- **Copper:** VBUS, VCC, B−, V_SWITCH and the +3V3 feed are adequate for 0.9 A; there is no neck of concern.
- **Standby with S1 off:** about 4.5 µA.
- **Run time:** roughly 30–45 h on a 3 Ah cell.

**Fab:**
- **Area ratio at 80 µm:** the minimum is U1 at 0.79, then U15 0.94, U3 1.03, U9 1.12, J2 1.60 (all 320 apertures,
  including fp_poly windows).
- **Exposed-pad paste:** U3 55 %, U9 82 %.
- **Vias:** none centred in an SMD pad.
- **Copper to edge:** the minimum is 0.33 mm.
- **Courtyards:** no overlaps.
- **Text:** every visible text is ≥ 1.0 mm high with ≥ 0.15 mm stroke.
- **`gerbers/`:** holds only the `.gitkeep` placeholder; there is no stale package.

**Mechanical:**
- Mounting holes are ≥ 2.55 mm from any courtyard.
- The J2, J8 and S1 overhangs are intended.
- Vias are tented both sides.

## 9. Not covered, or not checkable from the files

- No EM simulation of either antenna. The P1 figures come from a steady-state model (±25 %), with no enclosure or sun.
- **Missing datasheet facts:**
  - the E220's internal RF-switch truth table;
  - whether the BQ21040 restarts at VRCH after a timer expiry;
  - the S3's XTAL pin capacitance and 915 MHz blocking;
  - MLCC DC-bias curves;
  - LED forward voltages;
  - the holder floor geometry;
  - the TX06 NTC's R–T table (the B-equation was used; its cold trip is probably ~1 °C colder).
- FCC/CE limits.
- JLC's actual handling of J2's legs, J8 and the bottom-side THT.
- No bench data exists: this board has never been built.
