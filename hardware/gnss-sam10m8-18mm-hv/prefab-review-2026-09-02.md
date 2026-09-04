# SAM-M10Q GNSS carrier, LDO redraw — pre-manufacturing review

**Board:** `hardware/gnss-sam10m8-18mm-hv`, working tree of the main checkout as saved on 2026-09-02 (schematic 06:03, PCB and project 06:58). KiCad had the project open during the review, so anything not yet saved is not covered.
**What changed since the fabricated v2.1.0 board:** the TPS62913 buck and its filter are gone; an ADP7142ARDZ-3.3 linear regulator with a 470 Ω bead feeds the module (the architecture of the 2026-06-10 LDO board that tracked 27 satellites); every pre-fab fix from 2026-08-04 that does not involve the buck is kept; the layout was re-done with a dense stitching-via carpet and vias inside the module pads; the board file now declares six copper layers.
**Method:** netlist, ERC, DRC (with schematic parity) and BOM exported with kicad-cli from a byte-identical snapshot of the saved files; copper geometry measured from the board file (zones, vias, pad positions) against the fabricated v2.1.0 board and the 2026-06-10 LDO board; two independent specialist passes (schematic and electrical against the ADP7142 and u-blox documents; layout against DRC, JLC capabilities, TI/ADI/u-blox layout rules) with their load-bearing claims re-checked here. Every number below comes from the files, not from a plot.

## Verdict

> **Update, 2026-09-02 16:40.** The verdict below describes the files as saved that morning. Since then the blockers B1–B8 were closed (see section 0): the board is 6-layer by decision with the JLC06161H-3313 stackup, via rules match the 0.4/0.3 mm vias, zones are refilled, the two open connections and the duplicate C16 are fixed, bom.csv and the priced workbook describe the LDO board, and the revision is V3. After the 16:40 pass: DRC 0 errors, 0 unconnected, 0 parity, 3 library-courtyard warnings (R1/R2/D1); ERC 0 errors. What remains open is listed in section 0.

**Schematic: electrically correct, release to layout.** The ADP7142ARDZ-3.3 is wired exactly per its datasheet Table 4 (VIN 7/8 with EN tied to VIN, VOUT 1/2 with SENSE 3 tied to VOUT ahead of the bead, SS through 1 nF, GND 4 and EP 9 to ground), the SMF10A clamps VSYS↔VSS on the connector side of the common-mode choke, V_BCKP is open with a no-connect flag, every unused module pin is flagged, the UART and PPS series resistors and J3.5 = PPS are in, and every buck part is gone from both the schematic and the PCB. The five ERC errors are hygiene (no PWR_FLAG anywhere, two "Output" pins tied), not defects. What is not ready is the release package: the committed `bom.csv` still orders a TPS62913 board, nothing stops an assembler fitting the adjustable ADP7142ARDZ (which would regulate at 1.2 V on this divider-less board), the soft-start capacitor puts start-up inrush at the regulator's current limit, and there is no title block or revision.

**PCB: not fab-ready.** As saved, a gerber plot would ship a +3V3-to-GND short (the In2 +3V3 plane fill is stale and runs unbroken through 56 ground vias), every one of the 260 vias is 0.4/0.3 mm with a 0.05 mm ring (half the board's own minimum and below JLC's 0.45 mm via floor), two connections are unrouted (the PPS LED can never light), C16 is placed twice, and the regulator's enable pin reaches VIN only through a track on In3.Cu. Whether that last item is a defect depends on one decision that has to be made explicitly first: **is this a 4-layer or a 6-layer board?** The file says six; the fabricated board and its tag say four. Everything about via-in-pad, the In3 track and the stackup follows from that answer, so it is section 1.

RF-wise the antenna environment is unchanged or slightly better than the fabricated board (B.Cu ground under the module 81 % solid vs 73 %, 158 vs 19 stitching vias in the body, only a 1.15 mm RXD stub on B.Cu under the module vs 74 mm of routing before). Nothing in this layout would move the satellite count either way; the LDO is the design change that matters, and it is drawn correctly.

## 0. Status after the 2026-09-02 fix pass — the complete list

Decided: **the board is 6-layer** (JLCPCB standard JLC06161H-3313), one +3V3 plane on In2 and ground on In1, In3, In4 plus the outer pours. Vias 0.4/0.3 mm are legal on that process (JLC 6-layer floor is 0.25 mm on a 0.15 mm hole) and are filled and capped by default, so via-in-pad is accepted. Everything below is the full list; nothing else is tracked elsewhere for this board.

| # | Item | Status |
|---|---|---|
| B1 | Six copper layers, stackup, EN strap on In3 | **Done (decision: 6-layer).** Stackup set to JLC06161H-3313 (0.035 / 3313 0.0994 / 0.0152 / core 0.55 / 0.0152 / 2116 0.1088 / 0.0152 / core 0.55 / 0.0152 / 3313 0.0994 / 0.035, ENIG, 1.5994 mm with masks); gerber job file reads 6 layers. The In3 enable strap stays; optional: move it to F.Cu so nothing depends on the layer count. |
| B2 | Vias 0.4/0.3 vs the 4-layer rules | **Done.** Board rules now min via 0.4 mm, min annular 0.05 mm (the smallest via used); 0 via errors. |
| B3 | Stale In2 +3V3 fill | **Done.** All zones refilled and saved from pcbnew; In2 plane 371 mm², one region; 0 clearance errors. |
| B4 | R2 and R1–D1 unrouted | **Done by the owner** (saved 15:50); DRC unconnected = 0. |
| B5 | Duplicate C16 | **Done by the owner**; parity clean. |
| B6 | Via-in-pad under U1 | **Accepted** on the filled-and-capped 6-layer process; board file now declares `(filling yes)(capping yes)`. |
| B7 | bom.csv described the buck board | **Done.** 14 rows, matches the schematic designator for designator, constraints carried. |
| B8 | Fixed-variant constraint | **Done.** In bom.csv (U2 row), in the U2 symbol Description, and in the priced workbook comment. Schematic note text still to be placed (section 5). |
| S-Sch1 | PWR_FLAG on VSYS, VSS, GND, +3V3 | **Done.** Five PWR_FLAGs placed (VSYS, VSS, GND, +3V3 and the regulator input net, which the retyped VIN pin now needs); ERC power errors 4 → 0. |
| S-Sch2 | U2 pin types (VOUT/VOUT Output–Output) | **Done** in the library and the schematic cache: VOUT power_out + passive, VIN power_in + passive; the pin-to-pin error is gone. |
| S-Sch3 | 0.635 mm stub wire at C136 | **Done** (removed; netlist identical). |
| S-Sch4 | Soft-start C7 1 nF → 2.2 nF | **Done** (schematic value + MPN CC0402KRX7R9BB222; BOM; workbook). |
| S-Sch5 | MPN/value hygiene | **Done.** Every part carries MPN + Mfr from the fleet donors; 22 µF unified on CL21A226MOQNNNE; D3 Value "LED" → "Green"; BOM export now 13 lines. |
| S-Sch6 | R3 10 k → 1.5 k | **Declined by the owner** (2026-09-02): R3 stays 10 k. |
| S-Sch7 | Title block / revision | **Done.** Schematic title block added (rev V3, 2026-09-02); PCB title block rev V2 → V3, silk `${REVISION}` follows. Tag as `gnss-sam10m8-18mm-hv-v3.0.0` when committed. |
| S-Sch8 | Cosmetic ERC (J3 pin types, off-grid, lib mismatch) | **Done except off-grid**: J3 pins typed passive (library + cache), cached symbols synced to the library (lib-symbol mismatch 8 → 0, pin-to-pin 7 → 0). ERC now reports 0 errors and only the 96 endpoint-off-grid warnings (the drawing sits on a 0.635 mm half-grid; connectivity is proven by the netlist). |
| S1 | In2 as +3V3 plane | **Accepted** on 6 layers (In3/In4 GND under the module). |
| S2 | Thermal reliefs on U1 GND pads | **Closed, not applied**: with filled vias inside the pads the pour relief no longer isolates the joint thermally (the vias still sink heat into four planes), so it would cost antenna ground for nothing. |
| S3 | V_IO decoupling at pin 2 | **Done by the owner**: C16 now sits at (57.80, 51.32), 3.9 mm from pin 2 with its own via and stub; the stale fill its move left behind was refilled. |
| S4 | Via-in-pad on U2/0402s/J3 tabs | **Accepted** (filled and capped). |
| S5 | Silk: rev V3, patch-direction marker, J3 pin-1 mark | **Done.** "PATCH ANT ON BACK" (0.8 mm) along the bottom edge at (52.85, 56.35), clear of D2's outline and the edge; "1" beside J3 pin 1 at (57.55, 49.30). |
| S6 | FL1 footprint has no F.Paste | **Withdrawn**: the footprint's paste is drawn as four `fp_poly` on F.Paste (one per pad), exactly the SnapEDA convention the SAM-M10Q footprint uses; the layout pass misread it. Nothing to change. |
| S7 | Three same-net GND via pairs at 0.235–0.241 mm hole-to-hole | **Done**: one via of each pair deleted (257 vias remain, 157 GND in the module body). |
| S8 | Test pads | **Not placed**: a grid search inside the +3V3 pour found no spot with more than 0.12 mm copper margin (the module's B.Cu pads sit under it), and the UART tracks run at the 0.2 mm minimum pitch. Probe +3V3 at C1/C5/C6 or FL2's pad, VIN at C136, the UART at R7/R8's outer pads or J3. |
| C1–C9 | Consider items | Unchanged; C7 (refdes C135/C136, D2 vs CR1) noted in bom.csv. PCB footprints now carry the schematic's MPN/Mfr fields and values (parity 24 → 0). |
| — | Priced workbook | **Done.** `TinkerRocket_CrossBoard_BOM.xlsx`: GNSS $16.71 → $16.44/bd, ADP7142ARDZ-3.3 row added (price ESTIMATED $2.08, confirm at order), buck lines set to LoRa-only, BLM18PG471SN1D now Rocket+GNSS; snapshot `…_pre-gnss-ldo_20260902a.xlsx`; other boards unchanged to the cent. |
| — | FABRICATION-NOTES.md | **Done**: `hardware/gnss-sam10m8-18mm-hv/FABRICATION-NOTES.md` (Block A order parameters incl. the filled-and-capped via requirement; Block B module reflow, MSL-4, rework ban, fixed-variant U2, polarity/identity checks). |

## 1. Layer count (decided 2026-09-02: six)

Kept for the record: the comparison that framed the decision. The board is now the 6-layer column.

| | If the board stays 4-layer (as fabricated, tag v2.1.0) | If the board becomes 6-layer (what the file currently says) |
|---|---|---|
| Stackup | Board Setup → Physical Stackup → 4 copper layers, JLC04161H-7628: 0.2104 prepreg / 1.065 core / 0.2104 prepreg, inner copper 15.2 µm, ENIG, 1.5862 mm. Remove In3.Cu/In4.Cu. | Enter a published JLC 6-layer profile (JLC06161H-3313 is 0.0994 / 0.55 / 0.1088 / 0.55 / 0.0994 mm) with **all four** inner layers at the same copper weight; today In1/In2 are 15.2 µm and In3/In4 are 35 µm, which matches no process. |
| The In3 track | **KiCad deletes every In3 item when the layer count is reduced, so route EN first** (fix B1). | Keep, but a 2.54 mm enable strap on an inner layer is pointless; route it on F.Cu anyway so the design does not depend on the layer count. |
| Via-in-pad (54 vias inside U1's mask openings, more under U2, the 0402s, J3's tabs) | Not acceptable with `(filling no)(capping no)`: move them out (fix B6). | Acceptable if the order includes filled and capped vias, which is your standing rule for 6-or-more-layer orders. Then set `(filling yes)(capping yes)` in Board Setup so the file records the process, and keep the paste windowpane as is. |
| In2 as a +3V3 plane | Make In2 GND (the pre-fab A2/B3 decision); route +3V3 on F.Cu (S1). | Tolerable: In3 and In4 are solid GND directly under the module, so the plane sits between grounds. Still refill it (B3). |
| Cost / lead time | as before | JLC 6-layer, small: modestly more per board; filled vias are included on 6-layer orders. |

The rest of this review is written so that it holds under either answer; items that differ say so.

## 2. Blockers

### B1. Layer count and the enable strap on In3.Cu

`(layers)` lists F.Cu, In1.Cu, In2.Cu, In3.Cu, In4.Cu, B.Cu; `(thickness 1.6472)`; the GND zone fills F, B, In1, In3 and In4 (549–550 mm² on each inner layer). A default `kicad-cli pcb export gerbers` writes a job file with LayerNumber 6 and six copper files, and In3_Cu/In4_Cu are no longer empty as they were at HEAD: they carry the full ground fill. Restricting the export with `--layers F.Cu,In1.Cu,In2.Cu,B.Cu` still writes LayerNumber 6.

The layout now depends on In3: Net-(U2-EN) exists only as a 0.127 mm, 2.54 mm track on In3.Cu from (50.285, 41.635) to (50.285, 44.175), between a via in U2 pin 5 and a via in U2 pin 7. On a 4-layer build that track does not exist, EN floats, and the ADP7142 (precision enable, on above 1.16–1.28 V, off below 1.13 V) is most likely off. Fix regardless of the decision: route EN on F.Cu from pin 5's outboard end (49.43, 41.635) west to x ≈ 47.15, south to y ≈ 44.9, east into the VIN zone (west edge x 47.48); C7 pad 2 at 47.46–48.02 × 41.88–42.50 is cleared by 0.25 mm at x 47.15. About 5.5 mm of 0.127–0.2 mm track, no vias.

### B2. All 260 vias are 0.4 mm / 0.3 mm

Every via in the file is `(size 0.4)(drill 0.3)`: a 0.05 mm annular ring. Board rules (`min_via_diameter 0.5`, `min_via_annular_width 0.1`) and JLC's capability (0.45 mm minimum via diameter on a 0.3 mm hole) both reject it. DRC shows 199 annular-width and 199 via-diameter errors; 199 is the DRC engine's per-rule report cap, not the count (enlarging 98 vias on a scratch copy brought the count to 162 = 260 − 98). The fabricated board used 0.55/0.35 and 0.6/0.3.

Fix: select all vias → 0.55/0.35 (the project's second preset). That resize creates conflicts the carpet is too dense for; computed for 0.55/0.35:

- Hole-to-hole under 0.25 mm on 11 pairs (3 today): (51.27, 50.73)–(50.77, 50.92); (55.16, 47.43)–(54.64, 47.58); (56.03, 50.31)–(56.28, 50.79); (54.29, 50.85)–(54.47, 51.37); (54.29, 50.85)–(53.76, 50.70); (55.90, 49.10)–(56.09, 49.62); (57.07, 41.93)–(57.09, 41.36); (50.30, 48.26)–(50.24, 48.83); (55.94, 51.27)–(56.28, 50.79); (54.29, 50.85)–(54.86, 50.69); (58.00, 45.62)–(58.41, 46.05). Delete one of each.
- Cross-net via pair: +3V3 (54.73, 48.20) vs GND (54.64, 47.58), copper gap 0.078 mm.
- 19 vias closer than 0.2 mm to a foreign pad, among them R7.2 vs the GND via at (56.23, 48.48) at 0.077 mm, U1.2 (+3V3) vs GND vias at (55.90, 49.10) 0.125 mm and (54.64, 47.58) 0.14 mm, FL2.1 vs the RESET via (59.66, 42.89) 0.125 mm, J3.1 vs the +3V3 via (54.73, 48.20) 0.175 mm. Nearest-neighbour spacing of the GND carpet: median 0.776 mm, minimum 0.536 mm (fabricated board 2.01 / 1.16 mm).

### B3. The In2 +3V3 plane fill is stale: a short in any plot from the saved file

The saved In2 polygon (400.2 mm², 8623 vertices) contains 56 GND vias with no anti-pad. DRC: 60 clearance and 60 hole-clearance errors at 0.0000 mm, e.g. at (59.55, 49.29), (47.01, 36.35), (45.70, 36.62), (54.47, 51.37), (52.91, 41.55), (61.45, 46.87). `kicad-cli pcb export gerbers` plots stored fills, so this is the px1105r trap of 2026-08-09 again. Fix: Edit → Fill All Zones (B), save, then DRC and plot. Verified on a copy with `--refill-zones`: zero clearance errors remain; the plane shrinks to 372.7 mm² with 0.4 mm vias (324.9 mm² with 0.55/0.35) and stays one region that reaches all six +3V3 vias, so V_IO's single via keeps its connection. Add `--refill-zones` to the DRC step of `tools/plot_gerbers.sh` so this class cannot ship again.

### B4. Two unrouted connections

R2 pad 1 (+3V3) at (62.46, 43.54) is 1.67 mm below the F.Cu +3V3 zone (bottom edge y 41.87) with no track; R1 pad 2 (43.13, 42.44) to D1 pad 2 (43.14, 41.565), 0.875 mm apart, no track, so the TIMEPULSE LED chain is open. Two 0.127 mm tracks fix both (or delete R2, per the earlier review's C5, and re-annotate).

### B5. Duplicate footprint C16

Two 100 nF footprints carry the reference C16, at (60.48, 42.06) and (62.43, 42.09), both on +3V3/GND; the schematic has one C16 (8 capacitors in the netlist, 9 on the board; DRC parity "Duplicate footprints"). Delete one, or add C17 in the schematic and use the spare for V_IO (S3).

### B6. Via-in-pad on the module, with the file declaring unfilled vias

54 vias lie inside U1's 20 mask openings, 41 of them under paste windows; the board setup says `(filling no)(capping no)(plugging no)`. Per pad: GND pads 1 ×6, 4 ×4, 5 ×5, 6 ×4, 10 ×5, 11 ×5, 15 ×5, 16 ×5, 20 ×7 (46 GND vias, several dead-centre); V_IO pad 2 ×1 at (54.73, 48.20), 0.69 mm off-centre and under paste; TIMEPULSE pad 7 ×1 dead-centre; TXD pad 13 ×1 dead-centre; VCC pad 17 ×4, all under paste; RESET_N pad 18 ×1 under paste. The fabricated board had five, and the field review asked to move those 0.6 mm outward; seven vias now sit in the 0.28 mm paste cross that review singled out.

- 6-layer with filled and capped vias ordered: acceptable; set `(filling yes)(capping yes)` in Board Setup so the file matches the order, and write the option into FABRICATION-NOTES.md.
- 4-layer: move every via at least 0.3 mm outside the openings (opening = pad + 0.1 mm per side). The eight signal and power vias get short B.Cu stubs exactly as RXD already has (pad 14 → 2.39 mm stub → via at (54.75, 33.31), 1.23 mm outside the body); the 46 GND vias add nothing inside the pads because the pour connects them.

### B7. The committed BOM orders the buck board

`bom.csv` still lists U3 TPS62913RPUR, L8, FB1, R5/R6, C10 and eight 22 µF. Exact edits:

- Delete rows 5 (C10 470 nF), 8 (FB1 782853200), 12 (L8 VLS3012CX-2R2M-1), 15 (R5 15.4 k), 16 (R6 4.87 k), 18 (U3 TPS62913RPUR).
- Row 1 (22 µF): designators `C1, C8, C9, C11–C15` → `C1, C135, C136` (or the renumbered C1/C2/C4). Rewrite the constraint: C136 sits on raw pack voltage behind FL1 (8.4 V full pack, ≤ 17 V TVS clamp), so 16 V stays the floor; delete the TI 40–80 µF window text, which was TPS62913-specific. Note that Samsung lists both CL21A226MOQNNN and CL21A226KOQNNN as not recommended for new designs; the named successor for the M part is CL21A226MOYNNN.
- Row 2 (100 nF): `C5, C16` → `C3, C5, C16`; C3 is the regulator input HF cap.
- Row 4 (C7): value and note per S-Sch4 below (2.2 nF, "ADP7142 soft-start capacitor, sets start-up time"); the fleet CC0402KRX7R9BB222 already on this row is the right part.
- Row 6 (TVS): refdes `CR1` → `D2` (or re-annotate the schematic to CR1 to match the sibling board). Replace "stays under the TPS62913's 18 V absolute maximum" with "ADP7142 VIN absolute maximum 44 V; C136 (16 V) sees the ≤ 17 V clamp only for the pulse".
- Row 13 (R1, R3): R3 → 1.5 k RC0402FR-071K5L (field review C4; a fleet part on both rocket computers); R1 stays 10 k.
- New rows: U2 `ADP7142ARDZ-3.3` (tube) or `ADP7142ARDZ-3.3-R7` (reel), 8-lead SOIC with exposed pad, footprint `SOIC127P600X175-9N`, with the constraint in B8; FL2 `BLM18PG471SN1D`, footprint `BEADC1608X95N` (a fleet part on the rocket computer).
- Unchanged: rows 7 (D1, D3 XL-1005UGC), 9 (FL1), 10 (H1–H4), 11 (J3), 14 (R2, R7, R8, R9), 17 (U1).

### B8. Nothing protects the fixed-variant assumption

The board has no feedback divider. If the adjustable `ADP7142ARDZ` (the 2026-06-10 board's part, and still the MPN in the library symbol's fields) is fitted with pin 3 tied to VOUT, the loop regulates at the 1.2 V reference and the module never starts. Datasheet Table 4: pin 3 is the sense input; an external divider "may also be used to set the output voltage higher than the fixed output voltage". Fix: a BOM constraint on U2 ("fixed 3.3 V variant only; SENSE tied to VOUT; do not substitute the adjustable ADP7142ARDZ"), the symbol's Description field corrected (it still reads "Adjustable o/p, 1.2v–39v" and is exported into the BOM), and a schematic note (text proposed in section 5).

## 3. Should fix before sign-off

### Schematic

- **S-Sch1. PWR_FLAG.** ERC: 4 × power-pin-not-driven (#PWR01 VSYS, #PWR05 VSS, H1 pin 1 on GND, U1 pin 2 on +3V3). The schematic contains zero PWR_FLAG symbols. No net is really undriven: VSYS/VSS/GND enter at J3, whose custom symbol pins are typed "Unspecified", and +3V3 is fed from U2 pins 1/2 typed "Output" through the passive FL2. Place PWR_FLAG on VSYS (at J3.3), VSS (J3.4), GND and +3V3.
- **S-Sch2. U2 pin types.** ERC: 1 × pin-to-pin error, U2 pins 1 and 2 (both VOUT, both "Output") tied together. In `Custom:ADP7142ARDZ` make pin 1 power_out and pin 2 passive, pin 7 power_in and pin 8 passive. Library fix, so it propagates.
- **S-Sch3. Stub wire.** A 0.635 mm vertical wire from C136 pin 1's connection point (77.47, 132.715) to (77.47, 133.35) ends on nothing (ERC unconnected-wire-endpoint). C136.1 is properly on Net-(U2-EN) through the horizontal wires. Delete it.
- **S-Sch4. Soft-start capacitor: 1 nF puts inrush at the current limit.** ADP7142 soft-start time = 320 µs + 0.6 µs/pF → 0.92 ms for 1 nF, i.e. 3.6 V/ms. Output-side capacitance is now 44 µF nominal (C135 + C1; the old board had 20 µF), so the ramp draws 111–158 mA, plus the module's start-up inrush of up to 100 mA (data sheet 4.3): 211–258 mA against ILIMIT minimum 220 mA. The regulator simply ramps in current limit for a few hundred µs, but there is no reason to run at the edge: C7 = 2.2 nF gives 1.64 ms (2.0 V/ms) → 162–189 mA total, uses the fleet CC0402KRX7R9BB222, and stays inside the module's V_IO ramp window (25–35 000 µs/V; 1.64 ms is 497 µs/V). Never leave SS open (320 µs → ~450 mA plus inrush) and never ground it (Table 4).
- **S-Sch5. MPN and value hygiene.** C135/C136 carry `CL21A226KOQNNNE` (±10 %), C1 has no MPN, the fleet part is `CL21A226MOQNNNE` (±20 %): pick one for all three (or the Samsung successor for every board). D3's Value is "LED" while D1 is "Green" (both XL-1005UGC): kicad-cli's BOM splits them. C3/C5/C16/C6/C7 and all resistors have no MPN field: fill from the fleet list (100 nF CL05B104KO5NNNC, 100 pF CL05C101JB5NNNC, 1 k RC0402FR-071KL, 10 k RC0402FR-0710KL).
- **S-Sch6. R3 (power LED) still 10 k** → 0.12 mA. Change to 1.5 k (0.8 mA). R1 correctly stays 10 k until the SAFEBOOT node is scoped (IM 3.2.3.3; Vih = 0.68 × V_IO = 2.24 V; D1's Vf is the lever).
- **S-Sch7. No title block.** `(paper "A4")` and no `title_block` in the schematic; the PCB title block says rev "V2". A regulator change on an already-fabricated V2 is V3 / tag v3.0.0 (docs/board-versioning.md): two electrically different carriers with the same marking is exactly the identity problem the field review hit.
- **S-Sch8. Cosmetic ERC.** 7 pin-to-pin warnings from J3's "Unspecified" pins (type them passive in `Custom:BM05B-SRSS-TB`); 92 endpoint-off-grid warnings because the drawing sits on a 0.635 mm half-grid (connectivity proven by the netlist); 8 lib-symbol-mismatch warnings (FL1, J3, U2, U1, FL2, 3 × +3V3) from the library defaults fixed in PR #992: run Tools → Update Symbols from Library once, instance fields survive.

### PCB

- **S1. In2 as a +3V3 plane (400 mm²) reverses the pre-fab decision.** Measured RF effect: none worth a dB (on the JLC 4-layer stack the plane is 0.21 mm below B.Cu, ≈ 74 pF to B.Cu ground ≈ 1.4 Ω at 1.575 GHz; B.Cu ground under the module is 81 % solid). What it costs: every one of the 248 GND vias needs an anti-pad (B3), the 158-via carpet stitches B.Cu to In1 through a perforated power plane, and the F.Cu +3V3 zone cannot reach R3 (55.13, 32.05) or V_IO without it. On a 4-layer build make In2 GND and route +3V3 on F.Cu: pad 17 from the zone directly above it; pad 2 by an 8–9 mm track from the zone at (60.2, 41.9) to a via at ≈ (54.75, 47.5) with a 0.6 mm B.Cu stub (the Net-(J3-Pad2) track at 54.35–54.77 × 46.92–47.22 must move); R3/D3 relocated beside the zone (x 61–62.5, y 37–38.5) because a track to (55.13, 32.05) would cross the UART runs at y 33.44/33.77. On a 6-layer build the plane is tolerable.
- **S2. Thermal reliefs on U1's ground pads (field review B2) not done.** GND zone `connect_pads thru_hole_only`, no footprint override: the nine GND pads tie solid into 508 mm² of B.Cu plus 46 in-pad vias. U1 → Footprint Properties → Clearance Overrides → Pad connection: Thermal relief, gap 0.2 mm (IM 4.4 Fig. 22), spokes 0.4–0.5 mm; effective only where the vias are out of the pads.
- **S3. V_IO (pad 2) has no local decoupling (field review A5).** Nearest +3V3 capacitor pad is 9.3 mm away (C16 at (60.48, 41.58)); V_IO hangs on one 0.3 mm barrel from the In2 plane. VCC (pad 17) is fine: C1 1.5 mm, C16 1.6 mm, C5 2.4 mm, C6 3.0 mm through the four in-pad vias. Put a 100 nF (A5's 10–100 pF is equally good) on F.Cu opposite pad 2 with its own GND via at ≈ (54.3, 47.3) once the J3-Pad2 track moves; the spare C16 footprint (schematic C17) is the part.
- **S4. Via-in-pad elsewhere.** Dead-centre in 0402 pads or regulator pins: R3.1 (55.13, 32.05); U2 pins 4 (55.395, 41.635), 5 (50.285, 41.635), 7 (50.285, 44.175); C1.2 (60.99, 38.50); C3.2 (48.66, 47.51); C5.2 (61.46, 42.58); C6.2 (62.46, 39.75); C7.2 (47.74, 42.19); FL1.3 (49.94, 47.62). Partial: FL2.2 vias (58.88, 40.80) 96 % and (58.84, 39.99) 34 %; C16(60.48).2 via (60.75, 42.57) 67 %. Inside 0805 or tab pads: C136.2 ×3, C135.2 ×4, J3 tabs 6 and 7 ×4 each. Keep the nine GND vias in U2's exposed pad — that is what ADI asks for and the old board had none. Same rule as B6: filled-and-capped order, or move them ≥ 0.3 mm outside the copper with a stub.
- **S5. Silk and identity.** `${REVISION}` = "V2" in the title block and in "SAM-M10Q / V2" at (46.92, 32.73): identical to the fabricated buck board's marking. Rev V3 (S-Sch7), optionally "LDO" and a date code. No patch-direction marker (field review C8): add F.SilkS text such as "GNSS PATCH ON BACK · SKY ↓" in the y ≈ 56.2 band between H3 and H4 (D2 occupies 50.2–54.5 × 53.9–55.3) or beside "TinkerRocket" at y ≈ 30.6. The J3 pin-1 "1" at (56.3, 48.1) from the tag is gone and the footprint has none; R7 now sits at 55.54–57.10 × 47.52–48.16, so place "1" at ≈ (57.5, 49.4). All 27 refdes are hidden, acceptable at this size; U2's pin-1 dot at (57.105, 46.045) and D2's cathode bar exist. Text 0.85/0.15 mm meets JLC's 0.8/0.15; "PWR" ends 0.28 mm above R3's pads; DRC reports no silk over copper.
- **S6. FL1 has no F.Paste** — **wrong, withdrawn 2026-09-02.** The pads do list only F.Cu and F.Mask, but the footprint draws its paste as four `fp_poly` on F.Paste (one per pad), the same convention the SAM-M10Q footprint uses; the apertures plot normally. Nothing to change.
- **S7. Near-duplicate GND vias** with hole gaps 0.235–0.241 mm (DRC warnings today): the first three pairs in B2; delete one of each now, the other eight after resizing.
- **S8. Test access (field review C6): none.** No test pads and no jumper in series with FL2. Add TPs on +3V3 (zone at ≈ (61.5, 40.5)), VIN (the 9.5 mm² zone) and GNSS_TX/RX on their y 47.5–47.8 F.Cu runs at x 58–62 (the east-edge runs at x 63.03/63.36 leave under 0.5 mm to the edge for a pad).

## 4. Consider

- **C1. U2 under the module.** U2 at (52.84, 43.54) is 1.3 mm off U1's axis on the far side, dissipating 77–204 mW (5.1 V × 15–40 mA; data sheet Table 14 gives ~15 mA for the module at 3.0 V) into nine EP vias that share the module's ground. θJA 89–145 °C/W → +7 to +30 °C, TJ ≤ 115 °C at 85 °C ambient. The old LDO board had the same placement with zero EP vias and tracked 27 satellites; IM 4.4 only warns about temperature changes near the TCXO. If the floorplan is ever reopened, U2 belongs at the connector end (y > 50).
- **C2. F.Cu ground is split** north 296 mm² / south 137 mm² by the VIN zone (47.48–51.15 × 43.87–47.03), U2, the VOUT zone (54.53–59.30 × 41.48–45.72), the PPS track at x 43.31 and the UART tracks at x 63.03/63.36, plus two slivers of 3.0 and 0.7 mm²; no F.Cu ground east of the UART tracks for 14 mm. Electrically fine (In1 and B.Cu continuous).
- **C3. UART routing:** TXD 29.4 mm and RXD 26.9 mm on F.Cu around the east edge, 9.6 mm parallel at 0.327 mm pitch (0.2 mm gap, the net-class minimum), 0.73 mm from the edge. Fine at 460800 baud; it is what blocks S1(iii) and edge test pads.
- **C4. FL2 DCR vs IM 4.1.1** ("no series resistance > 0.2 Ω on the supply line"): the BLM18PG471SN1D is nominally 0.2–0.25 Ω (Murata's page could not be fetched; verify), so at or over the rule; the drop at 40 mA is ≤ 10 mV, C1/C5/C6/C16 sit after it, and the identical bead fed the 27-satellite board. Accept, but record the deviation in the BOM constraint. Do not "fix" it with the fleet Würth 782853200 (8 mΩ but only 20 Ω at 100 MHz).
- **C5. EN = VIN** is correct per Table 4 ("for automatic startup, connect EN to VIN"); turn-on is then set by UVLO (rising ≤ 2.69 V), not by the 1.16–1.28 V precision threshold. A pack-level UVLO (100 k / 400 k divider for 6.0 V on, 5.5 V off) is possible but the host already gates the rail.
- **C6. Field review C5/C7 not taken** (optional): R2 kept (parallels the module's internal 7–13 kΩ RESET_N pull-up; harmless); no Schottky + supercap on V_BCKP (host gating → cold start each cycle).
- **C7. Reference designators.** C135/C136 are leftovers of the buck numbering (`used_designators` in the project file); renumber with the PCB (C2/C4 would match the 2026-06-10 board). The TVS is D2 here but CR1 on the sibling px1105r board and in `bom.csv`; pick one.
- **C8. Footprint-library mismatch warnings** on R1, R2 (R_0402), D1 (LED_0402): 4-line courtyards vs the library's fp_rect. Update when convenient. Small zones declare clearance 0.117/0.127 mm below the 0.2 mm net class (KiCad applies the larger; harmless).
- **C9. Data sheet edition.** The extract used is the preliminary ADP7142 text; re-read the soft-start coefficient, θJA and current-limit figures against the released revision before the BOM note quotes them.

## 5. Design-intent notes to add to the schematic (proposed text, not applied)

(a) At U2: "ADP7142ARDZ-3.3 (fixed). SENSE tied to VOUT ahead of FL2 by design; no divider on the board. The adjustable ADP7142ARDZ would regulate at 1.2 V here." (b) At C7: "Soft start: t = 320 µs + 0.6 µs/pF. Never ground SS." (c) At D1: "D1's Vf holds TIMEPULSE/SAFEBOOT_N above Vih at start-up (IM 3.2.3.3); do not replace with a resistor to GND or a low-Vf LED." (d) At U1.3: "V_BCKP open per IM 4.1.3; the host's power gating gives a cold start each cycle." (e) At D2: "TVS across VSYS–VSS on the connector side of FL1 (pre-fab review B1)." (f) At J3.5: "1 PPS to host GPIO2 via R9; the host pin must remain an input."

## 6. Verified correct; do not re-litigate

**Schematic**
- U2 pinout per ADP7142 Table 4: 1/2 VOUT and 3 SENSE on one net with C135 22 µF and FL2.1 (sensing before the bead keeps its inductance out of the loop and costs ≤ 10 mV); 4 GND and 9 EP to ground; 5 EN to VIN; 6 SS to C7 to ground; 7/8 VIN with C136 22 µF and C3 100 nF. Absolute maxima (SENSE ≤ 6 V, EN ≤ VIN, SS ≤ 6 V) met. ADP7142ARDZ-3.3 and -3.3-R7 are orderable parts.
- Capacitors: minimum 1.5 µF (CMIN) / 2.2 µF recommended, ESR 1 mΩ–0.3 Ω. C136 at 8.4 V ≈ 6.6 µF nominal, 4.5 µF worst case (3× margin); C135 at 3.3 V ≈ 15.4 µF nominal, 10.5 µF worst; 0805 X5R ESR ≈ 3–5 mΩ. 16 V rating correct for C136 (the 17 V clamp is pulse-only). Dropout ≤ 60 mV at 10 mA against 3.1 V of headroom at the 6.4 V pack floor.
- Input path: J3.3 VSYS → D2 SMF10A cathode on VSYS, anode on VSS (correct unidirectional polarity, same pins as the 2026-06-10 board), clamp ≤ 17 V vs the ADP7142's 44 V absolute maximum; VSYS → FL1 1→2 → VIN; GND → FL1 3→4 → VSS → J3.4. FL1 rated 320 mA vs ≤ 60 mA steady.
- Output path: VOUT → FL2 → +3V3 → C1, C5, C6, C16, U1.17 VCC and U1.2 V_IO tied together (IM 4.1.1), R2 1 k RESET_N pull-up with no capacitor (IM 3.2.3.1), R3 → D3 power LED. V_IO ramp inside the 25–35 000 µs/V window.
- Module PIOs: V_BCKP open with a no-connect (IM 4.1.3); SAFEBOOT_N, SDA, SCL, EXTINT open and flagged (5 flags, 5 open pins, no pin-not-connected in ERC); TIMEPULSE → R1 10 k → D1 → GND and → R9 1 k → J3.5 (0.13 mA plus the host pull-up, well under the 2 mA drive limit).
- UART direction: U1.13 TXD → R8 → J3.2 (GNSS_TX), U1.14 RXD ← R7 ← J3.1 (GNSS_RX), matching the host map (J1.1 = P4 TX → module RXD).
- Grounding: J3 tabs 6/7 and H1–H4 (MountingHole_Pad) on GND; exactly one VSYS, VSS, +3V3 and GND net; no stray single-pin nets.
- Buck removal complete: no U3, L8, FB1, R5, R6, C8–C15, C10; 26 schematic components = 26 board footprints bar the duplicate C16.

**PCB**
- U1 unchanged at (52.85, 42.2925) rot 90 on B.Cu, the only B.Cu part; pads 1.5 × 1.8 mm on B.Cu only; mask openings 1.6 × 1.9 mm (pad + 0.1 mm, IM 4.4.1) as 20 fp_poly; paste 80 windows of 0.58 × 0.72 mm = 33.41/54.00 mm² = 61.9 %, the 2 × 2 windowpane intact (80 regions in the plotted B_Paste); pin-1 dot on B.SilkS at (56.65, 50.79) outside the body.
- Under the module on B.Cu: only the 1.15 mm RXD stub (the fabricated board had 37.0 mm of +3V3, 14.2 RX, 12.4 TX, 8.8 PPS and 1.9 RESET there). No inner-layer tracks except the In3 enable strap (B1).
- Outline identical to the tag (42.15–64.15 × 29.54–57.04, r = 1); H1–H4 on GND, 3.8 mm pads on 2.2 mm drills, 0.55 mm from the edge. Copper-to-edge: fills 0.500 mm; nearest track 0.726 mm (RXD); nearest via 1.2 mm.
- J3 pinout and position unchanged from both working boards; R7/R8 1.6–2.6 mm and R9 3.2 mm from their J3 pins. D2 at (52.35, 54.57) clamps J3.3↔J3.4 on the connector side of FL1, 4.4 mm south of the J3 pins; FL1 at (50.79, 47.22) with the same footprint and orientation as both working boards; VSYS/VSS tracks 0.4 mm.
- U2: exposed pad to GND through nine vias; C136 → pin 8 2.09 mm, C3 → pin 8 1.97 mm; C135 → pin 2 2.24 mm; C7 → SS pin 6 1.74 mm, SS not grounded; SENSE pin 3 in the 14.3 mm² VOUT zone with pins 1/2; FL2 pad 1 in the VOUT zone, pad 2 in the +3V3 zone; every capacitor's GND pad has a ground via within 0.5 mm.
- Ground copper vs fabricated / 2026-06-10 board: F 437 (426 / 418) mm²; B 508 (456 / 425); In1 550 (549 / 493); under the module B.Cu 81 % (73 / 75 %), In1 98 % (99 / 99 %). GND vias in the 15.5 mm body 158 (19 / 11), in the 16 mm courtyard 180 (21 / 12).
- Zone priorities GND 0 < In2 +3V3 1 < VIN 2 < VOUT 3 < F.Cu +3V3 4; the tag's two B.Cu +3V3 zones are gone; no netless pads or zones (U1 pads 3, 8, 9, 12, 19 intentionally open); no courtyard overlaps; no silk over exposed copper; no dangling tracks in DRC as saved or refilled.
- PPS: pad 7 → via → 13.3 mm F.Cu → R1/D1 and R9 → J3.5. RESET_N: pad 18 → via → 2.2 mm → R2.2. Vias tented both sides; `pad_to_mask_clearance 0`.

## 7. Order of work

1. Decide 4 or 6 layers (section 1) and set the stackup accordingly; route EN on F.Cu either way (B1).
2. Resize all vias to 0.55/0.35 and clear the 11 hole-to-hole and 20 clearance conflicts that creates (B2, S7); on a 4-layer build move the in-pad vias out (B6, S4), on a 6-layer build set filled + capped in Board Setup.
3. Fill all zones, save, run DRC with `--refill-zones`, and expect zero errors (B3).
4. Route R2.1 and R1–D1 (B4); resolve the duplicate C16 and place the spare at V_IO (B5, S3); thermal reliefs on U1's ground pads if the vias are out (S2).
5. Schematic: PWR_FLAGs, U2 pin types, the stub wire, C7 → 2.2 nF, R3 → 1.5 k, MPNs, title block and rev V3, the design-intent notes; Update Symbols from Library (S-Sch1–8, section 5).
6. Rewrite `bom.csv` (B7) with the fixed-variant constraint (B8); write FABRICATION-NOTES.md (via option, U1 reflow profile, no hot-air rework, FL1 paste).
7. Silk: rev V3, patch-direction marker, J3 pin-1 (S5); test pads (S8).
8. Plot from a tag, compare the job file's layer count against the intended stackup before uploading.

## Sources

- ADI ADP7142 data sheet: Table 1 (ILIMIT 220–600 mA, start-up 380 µs, ISS 1.2 µA), Table 2 (absolute maxima), Table 4 (pin functions, "DO NOT ground" SS), Table 5 (θJA), Input and Output Capacitor Recommended Specifications (CMIN 1.5 µF, ESR 0.001–0.3 Ω), Capacitor Selection, Soft Start, Ordering Guide.
- u-blox SAM-M10Q Integration Manual UBX-22020019 R02: 3.2.3.1 (RESET_N), 3.2.3.3 (TIMEPULSE/SAFEBOOT_N), 4.1.1 (VCC/V_IO, ≤ 0.2 Ω), 4.1.3 (V_BCKP open), 4.3.1 (ground plane), 4.4 and Fig. 21/22 (layout, reliefs), 4.4.1 (mask/paste). Data Sheet UBX-22013293 R05: Table 9, Table 11 (V_IO ramp), Table 13, Table 14, 4.3 (100 mA inrush).
- JLCPCB capabilities (via 0.45 mm on 0.3 mm hole, 4-layer JLC04161H-7628, 6-layer JLC06161H-3313, silk 0.8/0.15 mm); docs/board-versioning.md; hardware/gnss-sam10m8-18mm-hv/prefab-review-2026-08-04.md; field-review-2026-09-01.md section 5.0.
- Snapshot outputs: netlist, erc.json, drc.json (as saved), drc_refilled.json (copy with `--refill-zones`), bom_export.csv, per-layer gerbers and renders of the saved file, all in the session scratchpad.
