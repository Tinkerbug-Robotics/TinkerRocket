# PX1105R GNSS carrier (gnss-px1105r-18mm-hv) — design review, 2026-10-12

**Board:** `hardware/gnss-px1105r-18mm-hv`, committed and clean at `c942cfd1` (2026-10-12 05:49, "the carrier routed and
its DRC errors fixed"). md5: PCB `05bfbf9d`, schematic `0ffee18b`, project `f9699ebd`, `.kicad_dru` `a5ad8f3f`. The live
files equal the reviewed state. KiCad had the project open during the review (lock files dated 06:46), so I reviewed the
committed state only, from a scratch copy; the live project was never opened.

**What the board is:** the SAM-M10Q carrier (`hardware/gnss-sam10m8-18mm-hv`, same J3 host connector, same "hv" pack input)
with the u-blox module replaced:
- **Receiver:** SkyTraq PX1105R L1/L5 (U1), fed by the SAM carrier's ADP7142ARDZ-3.3 LDO (U2) at about 125 mA.
- **Front end:** the Space Bug's filter-first chain with its reference designators kept: AE1 → D6 / C20–L1–C21 ESD
  high-pass → FL4 (B8389 SAW) → U3 (BGA855N6) → FL5 → U4 → C26 → U1 RF_IN; U5 (LP5907-1.8) supplies the LNAs.
- **Antenna:** Taoglas GVLB208.A, a passive 20 × 20 × 8.8 mm stacked L1/L5 patch on the back (B side), feed pin 1.2 mm
  off center.
- **Host:** 5-pin JST-SH J3 to the Tinker-Mantis J1; four M2 holes H1–H4 at 16.75 × 25.00 mm.
- **PCB:** 22.00 × 30.00 mm on the Edge.Cuts centerline (22.05 × 30.05 over the line), 6 layers on JLC 3313, 1.5994 mm,
  ENIG; 49 symbols.

**Who drew what:** Claude sessions transplanted the schematic onto the SAM carrier on 2026-10-08. The owner placed and
routed the board (`c942cfd1`); the schematic has not changed since `1cf7862d`. The project backups also show one schematic
edit made in the KiCad GUI on 2026-10-08, between Claude sessions (see S1).

**Method:**
1. Snapshot the committed files and the shared libraries into a scratch tree.
2. Export with `kicad-cli` 10.0.3: netlists, ERC, DRC with schematic parity, BOM, gerbers, drill, placement, schematic plot.
3. Measure geometry with pcbnew and shapely: pads, mask openings, zones, vias, clearances, plane coverage. 2-D field solves
   for the RF line and lands; Laplace sheet solves for ground paths; thermal, plate and loop models; parses of the owner's
   flight logs, receiver captures and firmware.
4. Nine specialist passes against the datasheets (power, RF schematic, RF layout, receiver, host system, layout/DFM,
   footprints, release, regression against the Space Bug and SAM-carrier reviews), then four gap passes (host faults,
   flight loads, receiver configuration, first-article test).
5. An independent verifier re-derived every kept finding; the high-impact ones also got an impact check and a skeptic
   pass. Where a verifier corrected a number or a proposal, this report uses the corrected one. Details in the appendix.

**Datasheets used:**
- PX1105R DS rev 2 (2024-12-27); PX1125R DS rev 1 (sibling, for context).
- Taoglas GVLB208.A SPE-25-8-318-B rev B (2026-03-31); Taoglas APN-12-8-002/B (generic L1 patch note).
- SkyTraq AN0037 v1.4.69 (2025-03-26) and AN0039 v1.4.42 (2022-07-22).
- ADI ADP7142 Rev H (mirror; analog.com timed out), cross-checked against Rev A.
- TI LP5907 SNVS798Q; TI TPD1E0B04 SLVSDG9C; TI TPS22810 SLVSDH0C and TPS25982 SLVSEI3D (host side).
- Infineon BGA855N6 Rev 2.2 and AN596 Rev 1.3; Qualcomm RF360 B8389 v2.1.
- Murata DLW21S (2014.5.12), BLM18PG (2019/11/27), LQW15AN, GRM1555; Vishay SMF 85881 Rev 2.4 and Littelfuse SMF (05/15/20);
  JST SH catalog; XINGLIGHT XL-1005UGC; 3M 9077 TDS (Nov 2006).
- ESP32-P4 datasheet v1.2 (chip rev v1.3) for the Mantis host; Rakon RST2016A Issue A (TCXO reference).

**State of the files:**
- **DRC:** 0 errors, 0 unconnected, 0 parity; 7 warnings (4 library mismatches on U2, R1, R2, D1; 3 silk). A scratch
  refill gives the same result, so the fills are current.
- **Parity:** `tools/check_board_parity.py` finds 49 board symbols, all present and linked by path.
- **ERC:** 91 entries: 84 off-grid (inherited), 5 pin_to_pin (all correct connections), 2 errors, both on U1 pin 18 (S4).
- **Release package:** no FABRICATION-NOTES.md, no bom.csv, `gerbers/` holds only `.gitkeep`, no tag (D1, D4).

---

## Verdict

**No blockers.** The circuit is right where it most often goes wrong:
- **Front end:** pin for pin and value for value it is the Space Bug's (0 pin differences over AE1, C19–C28, D6,
  FL4/FL5, L1–L8, U3–U5), which copies Infineon AN596 Option B. Every SAW port sits at 0 V DC, C22/C24 block the LNA
  inputs (abs max 0.9 V), and C26 keeps U1's 3.3 V antenna bias off U4's output.
- **Space Bug fixes carried over:** the feed keep-out now covers every layer (its L2), U4.4 has a via (its L4), U2's
  exposed pad has 6 vias (its L7), J3's tabs are on GND (its S2), U1 paste is 100 % (its S8), U3/U4 have a single NSMD
  window (its S10), the rule floors are set (its D3) and the RF class width equals the drawn 0.16 mm (its D4 width).
- **Power:** U2 is wired per ADI, soft start 1.53 ms, inrush below the minimum current limit; D2 cannot conduct at
  8.4 V; FL1 is oriented as a common-mode choke; +3V3 at U1 is 3.22–3.36 V.
- **Host link:** J3 pins 1–4 and 6/7 match the Mantis J1 and the SAM carrier; logic levels are compatible apart from a
  paper margin (M8).

What needs attention before an order:

1. **J3 pin 5: confirm RXD2 or PPS (S1, decision).** The copper takes J3.5 through R9 to the receiver's RXD2 input; 1PPS
   reaches only the LED, so the flight computer gets no hardware time mark. The project backups show this was a GUI edit
   on 2026-10-08 (both PPS labels renamed to RXD2 and R4 deleted between the 08:29:52 and 08:31:55 saves), and it agrees
   with the Mantis net name GNSS_RXD2 and the legacy-carrier row in `board_v9.h`. The README's "pin 5 is PPS out" came from
   my session notes, which never saw that edit. If RXD2 is intended, only the documents change. If PPS is wanted, it is a
   local re-route that is cheap now and a cut-and-jumper after the order.
2. **The release package (D1, D2, D3, D4).** There are no fab notes. The two TSNP LNAs need an 80 µm foil that the fab will
   not pick by itself; via fill, U1's 240 °C peak and MSL4, SAW ESD handling and the AE1 hand-fit are recorded nowhere.
   U2's fixed-3.3 V guard has no BOM or fab-note row, while the only stocked LCSC listing is the adjustable part, which
   regulates at 1.2 V here. Several RF parts need consignment or global sourcing.
3. **The antenna feed joint (K1, M5).** The GVLB208.A's 1.2 mm pin stops about 0.42 mm below F.Cu on this 1.6 mm board, so
   the only RF path into the receiver is a blind barrel fill. Write the procedure, qualify it on a spare-board coupon, and
   check every unit with a DMM through the pin.
4. **Two first-article risks (M1, M2).** The patch sits on 13.5 % of its 70 × 70 mm minimum ground plane: by estimate L5
   loses 4–7 dB of RHCP gain. About 0.8–1.05 W heats 22 × 30 mm: in a closed bay the 85 °C parts reach their limit at
   roughly 35–50 °C bay air (estimate). Both have bench tests in section 3, several of which must happen before AE1 is
   fitted (M13, M19).
5. **Cheap layout changes while the board is open:** move +3V3, +1V8 and the VSYS jog off B.Cu under the patch (L2, a
   layer change only); pin-1 marks for U1 and U2 (L1); one GND via at U3 pin 1 (L3) or the U3 VCC re-route that includes
   it (L4); fiducials (L6); C5 next to U1 VCC (L7); three return vias (L8); U2's paste decision (S3); the U1 pin-18
   no-connect flag (S4); AE1's 0.10 mm ring (S5).
6. **System (F2, F3, F4, F5).** Mantis V11 should get 1 kΩ in series on the three GNSS lines at the P4 and a current-limited
   GNSS branch. A SkyTraq driver with a written configuration spec must exist before this carrier flies; today's UBX-only
   image stalls every boot by at least 60 s with it fitted.

---

## Summary

| ID | Severity | Finding | Who |
|---|---|---|---|
| **S1** | **decision** | J3 pin 5 goes through R9 to U1's RXD2 input (an owner edit of 2026-10-08); the host gets no 1PPS, and the README says PPS out | owner |
| **K1** | **major** | AE1's feed pin stops about 0.42 mm below F.Cu: the RF feed is a blind barrel fill 0.24 mm from U1's body, and nothing records how to make it | mechanical / docs |
| **D1** | **major** | No FABRICATION-NOTES.md: the TSNP LNAs need an 80 µm foil; via fill, U1 reflow and MSL4, SAW ESD and the AE1 hand-fit are recorded nowhere | docs |
| **D2** | **major** | U2's fixed-3.3 V guard is in no release file; a stray MP field and the only stocked LCSC listing name the adjustable part (1.2 V here) | docs / library |
| **M1** | **major (risk)** | The patch sits on 13.5 % of its 70 × 70 mm minimum plane, with about 0.8 mm of ground beyond its long sides: L5 likely loses 4–7 dB RHCP | test |
| **M2** | **major (risk)** | About 0.8–1.05 W on 22 × 30 mm: in a closed bay the 85 °C parts reach their limit at about 35–50 °C bay air (estimate) | test |
| L1 | minor | Pin-1 marks: U1 has none (symmetric under 180°), U2's dot prints on C1's pad; small marks below the legend minimum | layout / library |
| L2 | minor | +3V3 (21.1 mm), +1V8 (13.9 mm) and a VSYS jog run on B.Cu under the patch, slotting its ground | layout |
| L3 | minor | U3 pin 1 (first LNA ground) reaches ground only through F.Cu pour: 1.84 squares against 0.03 for U3.4 | layout |
| L4 | minor | The LNA 1 nF bypasses sit at PON, not VCC; U3's VCC reaches C23 through two barrels and 2.4 mm of B.Cu | layout |
| L5 | minor | 26 mask openings on U1, U2, FL1, FL2 leave about 0.025 mm of mask over the adjacent pour (JLC asks ≥ 0.09 mm) | footprint |
| L6 | minor | No fiducials on a board with 0.4 mm-pitch LNAs and 0.3 mm SAW lands | layout |
| L7 | minor | U1 VCC decoupling is 1.8–2.0 mm from pin 8 (SkyTraq: directly adjacent; Space Bug 0.40 mm) | layout |
| L8 | minor | Nine non-GND vias have no GND return via within 1 mm (owner's rule) | layout |
| L9 | minor | C136 (22 µF on the pack rail) sits beside H3 and near the left edge; a resistive crack would not be interrupted | layout / docs |
| S2 | minor | R7 and R9 are 1 kΩ where SkyTraq offers 10 kΩ: a host UART idling high back-feeds the dark carrier with up to about 2.7 mA | schematic |
| S3 | minor | U2's board copy prints its full 2.41 × 3.05 mm EP land; the library's panes never reached it. A paste decision is needed | footprint |
| S4 | minor | U1 pin 18 has no no-connect flag (both ERC errors); the PX1105R symbol's pin types are loose | schematic |
| S5 | minor | AE1 feed hole: 0.10 mm annular ring on B.Cu and inner layers, below JLC's 0.15 mm minimum | footprint |
| S6 | minor | D6's footprint description inverts TI's mask preference; the pour makes its two lands asymmetric | footprint |
| M3 | minor | IM2 in U3 from host BLE minus LoRa can land at L1/E1/B1I; FL4 and the high-pass are the only selectivity ahead of it | test |
| M4 | minor | Mantis clock harmonics land at 1560 MHz (B1I) and 1600 MHz (G1); the S3 and its crystal sit under the patch when stacked | test |
| M5 | minor | The feed-joint fill cannot be verified on a built unit; a coupon qualifies the method, a DMM through the pin screens each unit | test |
| K2 | minor | Mounting-hardware margins are tight on both faces and recorded nowhere; a metal bond at Mantis H2 bypasses FL1's return | mechanical / docs |
| K3 | minor | The 16 g patch hangs on 3M 9077 tape and the blind joint; the logs show 118–146 g shocks (about 1.5–2× peel margin) | mechanical |
| K4 | minor | Stack geometry is undefined: the matching Mantis holes put the carrier over J8/C130/S1 (B side) or the S3 and its antenna (F side); only V11 matches | system / mechanical |
| K5 | minor | J3 is friction-retained, with no staking or strain-relief rule | mechanical / docs |
| D3 | minor | Four assembler lines (U1, U2, U3/U4, FL4/FL5), maybe more, cannot come from LCSC stock | docs |
| D4 | minor | No committed unpriced bom.csv; inconsistent fields; the default CPL carries AE1 on the bottom and H1–H4 | docs |
| D5 | minor | cables.md covers the reversed-jumper hazard only for the legacy carrier; this one runs reversed and looks alive but deaf | docs |
| D6 | minor | Doc drift: the README row says "placement in progress"; 3dmodels/README misdescribes the CadQuery models | docs |
| F1 | minor | V_BCKP is tied to VCC: every carrier power cut is a 29 s cold start that loses SRAM configuration | schematic (owner's call) |
| F2 | minor | A J3.2–J3.3 short puts the pack onto Mantis GPIO3, which has no series resistor | host (Mantis V11) / test |
| F3 | minor | The Mantis GNSS branch has no current limit below the 11.6 A eFuse; a hard VSYS short draws about 9–25 A | host (Mantis V11) |
| F4 | minor | No consolidated configuration spec for the future SkyTraq driver; the factory state is not fit for flight | firmware |
| S7 | note | The receiver's RXD floats from carrier power-up until the FC configures its UART | firmware |
| S8 | note | U3/U4 single mask windows also expose pour, via caps and track stubs (about 29 % extra wettable copper) | footprint |
| M6 | note | Interference sources next to the unshielded feed node (U1 0.21–0.24 mm away, J3 on the far face) | test |
| M7 | note | Conducted injection above about −23 dBm (L1) or −26 dBm (L5) at the port can overdrive RF_IN | test |
| M8 | note | On paper, TXD's 2.4 V minimum high is below the P4's VIH (about 2.60 V on this host's 3.465 V rail) | test |
| M9 | note | Eleven two-terminal parts pair a solid via-in-pad GND land with a thin track (tombstone or skew watch list) | test |
| M10 | note | A signal simulator cannot reproduce the common-mode TCXO transient at deployment and landing shocks | test / firmware |
| M11 | note | Patch S11 is reachable only through C19 with C20 lifted | test |
| M12 | note | The cascade-output tap at C26; scan with C26 fitted first, then lift it | test |
| M13 | note | AE1 fit is the gate for patch-off tests: X-ray and conducted injection first | test / docs |
| M14 | note | The patch-bottom temperature cannot be measured after the fit; bound it from U2's copper | test |
| M15 | note | U1 castellation wetting: the under-body half of every land is hidden; inspect fillets, profile to the joint | test / docs |
| M16 | note | U1 and the LNAs cannot be switched off separately as drawn; "RSTN low" is not a proven quiet state | test |
| M17 | note | Equipment gaps: no VNA in the listed kit, no calibrated noise source | owner |
| M18 | note | No test points and tented vias: every probe ground is a pad | docs |
| M19 | note | First-article verification plan (table) | test |
| K6 | note | First mode about 1.5–4.7 kHz with the patch bonded; keep the stack short, stiff and insulating, no isolators | mechanical |
| D7 | note | C19 (DNP) still prints paste on both pads, one on the feed | docs / footprint |
| D8 | note | RF netclass membership rests on 13 auto-generated net names | rules |
| D9 | note | Second 3D-model paths fail by design on this board; R1/R2/D1 mismatches are cosmetic | library |
| D10 | note | The silk ID "PX1105R V1" reads like an early revision of the Space Bug | docs |
| D11 | note | Panelization and DFM queries that follow from the 0.2 mm edge and 0.1 mm hole-clearance rules | docs |
| D12 | note | Mantis power-eco.md still gives R48 = 100 Ω / 14.7 A; the board has 127 Ω / 11.6 A | host docs |
| F5 | note | On today's UBX-only image every FC boot stalls at least 60 s in GNSS bring-up, delaying in-flight reboot recovery | firmware |
| F6 | note | The Tinker-Beetle cannot host this carrier | docs |
| F7 | note | The Mantis hold-up docs size GNSS ride-through for a 25–40 mA carrier; this one draws 105–127 mA | host docs |
| F8 | note | The pad wait drains about 65–112 mA more than with the SAM carrier | system |
| F9 | note | The patch puts B1I about 4 dB and G1 4–8 dB below L1 (RHCP): keep mask 0x001D, GLONASS off | firmware |
| F10 | note | GLONASS is absent from the owner's firmware build (rev 2023-08-31), not from the product | firmware |
| F11 | note | Saving configuration to flash has no power-loss protection; keep it SRAM-only | firmware |
| N1–N5 | note | Cascade NF, pad parasitics, resistor placement, ESD posture, U2-short containment (section 7) | — |

---

## 1. Fix before fab — layout

The copper is the owner's; everything here is a proposal.

### L1. Pin-1 marks: U1 has none, U2's dot prints on C1's pad, and several small marks are below the legend minimum — minor

- **Where:** U1 (Footprints:PX1105R, board and library), pad 1 (GND) at (48.575, 30.060). U2 pin-1 dot (Ø0.4) at
  (47.385, 42.385), on C1 pad 2. U3 triangle (60.53–60.99, 40.39–40.85); U4 triangle (60.67–61.13, 36.57–37.03).
  Ø0.2 dots on D1, D3, FL4, FL5; J3's "1" is 0.8 mm tall.
- **Evidence:**
  - **U1.** F.SilkS is two 9.7 mm lines plus four corner stubs that mask clipping trims to equal length, so the print is
    symmetric under 180°. F.Fab holds only `${REFERENCE}`. PX-DS rev 2 p11 has no pin-1 mark and draws all 18
    castellations (9 + 9), so counting cannot orient the part. The p1 render shows the shield dot top-left, a corner the
    p5 top view gives to pin 10. Rotated 180°, pin n lands on pad n + 9: VCC (8) on open pad 17, RF_IN (11) on pad 2
    (GNSS_TX), pins 15/17 on the +3V3 pads 6/8.
  - **U2.** 0.1226 of the dot's 0.1249 mm² (98 %) lies in C1 pad 2's mask opening; 0.0022 mm² survives (DRC
    silk_over_copper). The remaining silk is two 180°-symmetric side lines. Rotated, U2's GND pin 4 lands on pad 8 (VIN)
    and VIN pins 7/8 on pads 3/4.
  - **U3/U4.** U3's triangle overlaps FL5's top line by 0.0166 mm² (12.4 %, x 60.536–60.759); U4's loses 0.0150 mm²
    (11.2 %) to L8 pad 2. These are the three DRC silk warnings. 88–89 % of each triangle still prints and each outline
    leaves its pin-1 corner open, so they read. Rotated, either LNA puts +1V8 on AI (abs max 0.9 V, BGA855N6 Rev 2.2
    Table 2).
  - **Small marks.** JLC: lines under 0.15 mm or text under 1.0 mm "will be unidentifiable"; pad-to-silk 0.15 mm. The
    D1/D3/FL4/FL5 dots are 0.03 mm² arcs drawn with a 0.10 mm aperture. The board's minimum silk clearance is 0, so DRC
    flags silk on an opening but not silk within 0.15 mm of one: 54 of 94 F-silk items are within 0.15 mm (39 are stock
    0402 ticks at 0.100 mm).
- **Consequence:** placement follows the CPL, but inspection has nothing to check U1 and U2 against.
  - A rotated U2 is a hard VSYS-to-GND short through its GND pin and exposed pad, about 8–14 A by estimate. The casualty is
    FL1 (rated 320 mA) or the Mantis eFuse (limit about 11.6 A); the ADP7142 current limit plays no part.
  - A rotated U1 leaves the board dead with 3.3 V on two unpowered module inputs (above VCC + 0.5 V, PX-DS p6).
  - If the module's shield dot is really at pin 10's corner, aligning dot to dot would cause exactly the error a mark is
    meant to prevent.
- **Proposal:**
  1. **U1, board and library:** a Ø0.4 F.SilkS dot outside pad 1 at (47.60, 30.06) (0.173 mm from the opening) or
     (47.60, 29.35) (0.295 mm), and an F.Fab outline with a pin-1 chamfer. Before ordering, find which corner of the real
     module is pin 1 relative to its shield dot: ask SkyTraq, or ohm a sample. Pins 1, 10 and 12 are GND and common with
     the shield; pin 12 is the third castellation from the pin-10 end, and its 180° partner is pin 3 (RXD), which is not
     GND. Write the result into FABRICATION-NOTES.md and check U1 in the placement preview.
  2. **U2, board and library:** move the dot to (47.65, 43.60), library local (−3.05, −2.77): 0.263 mm from pad 1's
     opening and 0.71 mm from the PWR text. Not (47.45, 44.10), which is 0.063 mm from the opening and would be clipped.
  3. **Optional:** nudge the U3/U4 triangles to clear the three silk warnings. Enlarging the small dots, making J3's "1"
     1.0 mm tall and setting a 0.1 mm minimum silk clearance are housekeeping against the fleet's 0.8 mm text convention,
     which is the owner's call.
- **Prior:** Space Bug L14 (closed there with a fab-note placement check).

### L2. +3V3, +1V8 and a VSYS jog run on B.Cu under the patch, slotting its ground — minor

- **Where:**
  - **+3V3**, 0.4 mm on B.Cu: via in C1.1 (46.84, 40.31) → (48.14, 41.61) → R3.1 via (48.14, 47.33) → (48.14, 48.75) →
    (49.93, 50.54) → (59.42, 50.54) → via in U5.1 (59.4675, 50.5875).
  - **+1V8**, 0.2 mm on B.Cu: from the C28 via-in-pad (59.39, 46.84) by a 0.28 mm stub, then (59.67, 46.84) →
    (61.35, 45.16) → (61.35, 41.54) → (61.82, 41.07) → vias at (62.70, 41.07) and (60.83, 42.21); and (61.82, 41.07) →
    (61.82, 39.30) → (62.41, 38.71) → via (62.41, 36.09).
  - **VSYS**, 0.4 mm on B.Cu: via (53.75, 51.76) → (51.55, 53.96) → FL1.1 via (49.45, 53.95).
  - AE1 body x 43.19–63.19, y 32.02–52.02, with a C1.7 chamfer at the bottom right.
- **Evidence:**
  - All 21.07 mm of B.Cu +3V3 and all 13.93 mm of B.Cu +1V8 lie inside the patch outline. The +3V3 run feeds only
    U5/C27 and R3/D3; U1 is fed by the F.Cu +3V3 zone. VSYS has 5.22 mm on B.Cu, 0.37 mm of it inside the outline; its
    via sits 0.26 mm inside the lower edge, near the middle (about 7.7 mm west of the chamfer).
  - B.Cu GND under the patch is 92.3 % filled. Non-feed voids total 23.6 mm² (5.9 %): +3V3 14.13 mm² (an L-slot 0.654 mm
    wide: 5.72 + 1.42 mm up x 48.14 and 9.49 mm along y 50.54), +1V8 6.84 mm², VSYS 0.41 mm², six 0.34 mm² via antipads.
    In1, In3 and In4 are each 97 % solid there; In4 GND is 0.0994 mm above B.Cu.
  - Coupling, quasi-static estimate: the horizontal +3V3 run lies under the silver border just outside the 16.5 mm tape
    square (about 0.036 pF/mm), so +3V3 couples about 1.4 pF to the patch electrode and +1V8 about 0.6 pF. Against the
    electrode's roughly 60–70 pF to B.Cu, that is a divider of about −34 dB and −41 dB.
  - GV-DS rev B p12 (and p7) shows a solid antenna-side pour on the reference board.
  - Moving the routes is free: copied unchanged to In2, +3V3 crosses nothing (minimum 0.14 mm to Net-(J3-Pad2), above
    the 0.1 mm Default), +1V8 keeps ≥ 3.0 mm and VSYS ≥ 1.7 mm. In4 has no tracks at all.
- **Consequence:** small by estimate. The slots sit under the patch's own electrode with solid In4 GND behind them, and
  the SAM field review found +3V3 under a GNSS antenna 20–70 dB short of mattering. The +1V8 leg is the more interesting
  one: it carries LNA VCC (see L4) into the antenna ground ahead of about 30 dB of gain. It breaks the fleet's "keep the
  antenna face solid" rule for no reason.
- **Proposal:** change the layer of the existing segments only; every end is already a through via, so no new vias.
  - **+1V8** (10 segments) → In2 (stripline between In1 and In3, preferred because it keeps two planes between it and the
    antenna) or In4.
  - **+3V3** (6 segments) → In4, or In2 exactly as drawn (no re-ordering needed).
  - **VSYS** (3 segments) → In4 or In2. Drop the "F.Cu beside VSS" idea: the pair's order flips between J3 and FL1, which is
    why the jog exists.
  - After a refill, B.Cu under the patch is about 98 % solid (non-feed voids 23.6 → 2.6 mm²). If the routes stay, list
    the slots in the RF first-article checks.
- **Prior:** Space Bug L6; SAM field-review A5; SAM prefab-review-2026-09-02 §6.

### L3. U3 pin 1 (first LNA ground) reaches ground only through F.Cu pour — minor

- **Where:** U3.1 (GND) at (61.34, 41.20); candidate via sites (61.20, 41.05) and (61.32, 40.80).
- **Evidence:**
  - U3.1 has no track; it joins the F.Cu pour (SMD pads connect solid). Two parallel paths: to the FL5.3 via at
    (60.37, 41.85), 0.87 mm pad edge to via copper through a 0.20 mm neck at about (60.72, 41.58) between FL5.4's RF
    clearance and the U3.2 +1V8 diagonal; and east to C23.2's via at (62.70, 40.11), 1.47 mm with a 0.37 mm neck. No GND
    via lies within 1.0 mm.
  - Laplace sheet solve, pad to all GND vias: U3.1 1.84 squares, U3.4 0.03, U4.1 0.37, U4.4 0.02. At 0.125–0.156 nH per
    square that is about 0.25–0.3 nH for U3.1 (about 2.5–3 Ω at 1575 MHz), against 0.03–0.09 nH for the others. U3.4's
    via-in-pad sits on the same die ground (BGA855N6 Rev 2.2 Fig. 1), so the effective common-lead change is only about
    0.01–0.05 nH (estimate).
  - AN596 Rev 1.3 Fig. 11 p14: single-stage k minimum about 1.15–1.2 at 1–2 GHz. Cascade gain about 29.6 dB at L1.
  - Sites tested, each a 0.4/0.3 GND via with refill and DRC with parity (0 errors, 0 unconnected, the same 7 warnings):
    (61.20, 41.05): drill overlapping the pad corner as on U3.4/U4.4, worst clearance 0.153 mm to B.Cu +1V8, 0.03 squares.
    (61.32, 40.80): 0.200 mm to B.Cu +1V8, outside U3's mask window, 0.28 squares.
  - The Space Bug's equivalent (its L4, on U4.4) is fixed here (0.184 mm).
- **Consequence:** the RF effect is probably below part tolerance (k > 1 "guaranteed by device design", DS Table 4), but
  this is the one LNA ground without a via, on the stage that sets NF. Taoglas TFM.100A: "Ground vias should be placed
  beside each ground pad".
- **Proposal:** one 0.4/0.3 GND via at (61.20, 41.05), relying on the filled-and-capped via setting the board declares;
  or (61.32, 40.80) if via-in-pad is not wanted. Refill and re-run DRC. If L4's U3 re-route is made, its pin-1 via at about
  (61.34, 40.85) replaces this one. A stub to U3.4's via cannot be routed: U3.4 is an island inside the AI/AO tracks.
- **Prior:** Space Bug L4.

### L4. The LNA 1 nF bypasses sit at the PON pins, not at VCC; U3's VCC reaches C23 through two barrels and 2.4 mm of B.Cu — minor

- **Where:** U3 pin 2 VCC (61.34, 41.60), pin 6 PON (61.74, 41.20), C23.1 via-in-pad (62.70, 41.07), VCC via
  (60.83, 42.21). U4 pin 2 (61.48, 37.78), pin 6 (61.88, 37.38), C25.1 (62.41, 36.09).
- **Evidence:**
  - Netlist: +1V8 = C23.1, C25.1, C28.1, U3.2/6, U4.2/6, U5.5.
  - U3: PON → C23.1 is 1.01 mm of 0.2 mm F.Cu. VCC → 0.92 mm of F.Cu → through via (60.83, 42.21) → 2.43 mm of 0.2 mm
    B.Cu → up C23.1's via-in-pad: 3.35 mm plus two 1.6 mm barrels.
  - U4: PON → C25.1 1.56 mm of 0.2 mm F.Cu; VCC → C25.1 2.98 mm of 0.1 mm F.Cu, no vias.
  - Estimates (microstrip on 0.0994 mm prepreg): 0.27 nH/mm at 0.2 mm, 0.37 nH/mm at 0.1 mm; a via 1.30 nH isolated,
    0.72–0.87 nH as a loop to a nearby GND via. U3's VCC path 2.5–3.5 nH (25–35 Ω at L1, 19–26 Ω at L5); U4's 1.1 nH.
  - BGA855N6 Rev 2.2 Fig. 1 p3: VCC feeds the output through an internal inductor. Fig. 2 / Table 6 p9: C2 ≥ 1 nF at VCC
    pin 2, "(optional)", "RF bypass recommended to mitigate power supply noise". AN596 Rev 1.3 p10: C2 "not necessary if a
    clean and stable DC supply can be ensured".
  - The B.Cu run stays at least 8.2 mm from the feed with four GND planes between it and the F.Cu chain; its only
    coupling path is to the patch (L2).
  - Supply-loop estimate around U4: about −19 dB worst case (first-pass estimates spanned −15 to −45 dB), so up to about
    ±1 dB of gain ripple and no oscillation.
- **Consequence:** each LNA's supply node becomes part of its output network and a shared path between the stages; expect
  small gain and match shifts. The LP5907 rail is clean, so supply noise is not the concern.
- **Proposal:** a capacitor within 0.5–1.0 mm of either pin 2 does not fit without moving FL5/L4 or L8 (clearance search
  with the board's rules). Instead:
  - **U3, re-route only**, copying the pattern U4 already uses: pin 2 west to x ≈ 60.95 (0.1 mm track), north to
    y ≈ 40.50, east to about (62.15, 40.50), then diagonally into C23.1 (0.2 mm from C23.2). Add a 0.4/0.3 GND via for
    pin 1 inside the loop at about (61.34, 40.85), which also settles L3. Keep the PON stub. Delete the VCC via
    (60.83, 42.21) and its B.Cu stub from (61.35, 41.54). Result: about 3 mm of F.Cu, about 1.1 nH, no barrels. I checked
    it by hand against the 0.1/0.2/0.15 mm rules only; DRC must confirm.
  - **U4:** widen pin 2's track to 0.15–0.2 mm where L8 (RF, 0.2 mm) and the pin-1 via allow (about 0.8–0.9 nH).
  - **If left unchanged:** A/B test at first article. Chain S21 and a spur scan at C26 (M12); then tack a 0201 33 pF
    from the U3 VCC via (60.83, 42.21) to the GND via at (60.37, 41.85), 0.58 mm apart, and remeasure. S21 against the
    27.7/32.6 dB budget alone cannot resolve a sub-dB effect (the budget's own spread is about 4 dB).
- **Prior:** Space Bug L4 (its note on the U4 VCC path).

### L5. Mask openings on U1, U2, FL1 and FL2 leave about 0.025 mm of mask over the adjacent pour (JLC asks ≥ 0.09 mm) — minor

- **Where:** U1 pads 2–9 and 13–18; U2 pads 1, 2, 3, 5, 6, 7 (0.025 mm) and 8 (0.041 mm); FL1 pads 1, 2, 4; FL2 pad 2
  (0.024 mm) and pad 1 (0.047 mm).
- **Evidence:**
  - 26 of 140 F.Cu openings are under 0.05 mm from other-net copper (and the same 26 under 0.09 mm). All come from the
    0.102 mm per-pad mask margin in these four vendor footprints against the 0.127 mm pour clearance; the board's
    pad-to-mask is 0. U1.7 and U1.9 face a +3V3 pour, the rest GND. Nominally no copper is exposed (minimum 0.024 mm).
  - JLC: "Keep at least 0.09 mm clearance between soldermask openings and neighboring traces"; 1:1 openings are supported.
  - The SAM carrier, with the same footprints and a 0.200 mm pour gap, had 0.097–0.114 mm, so this is a regression from
    the 0.127 mm pour. Mantis, Beetle, the flown V9 and the Space Bug V2 carry the same or worse (down to 0.001 mm).
- **Consequence:** a mask shift of more than about 0.025 mm toward the pour, plus solder wetting across 0.127 mm, would short
  VSYS or VIN to GND at FL1 or U2 (a bridge at FL1.4 only bypasses the choke's ground leg; one at U1.9 ties the pulled-up
  RSTN to 3.3 V). It would show at the bring-up continuity check. The fleet's flown boards are evidence that the fab
  tolerates it.
- **Proposal:** set the per-pad mask margin to 0 (1:1) on this board's U1, U2, FL1 and FL2 instances, leaving 0.127 mm of
  mask. Do not use a local zone clearance of 0.2 mm or more: it overrides the approved 0.127 mm pour clearance and would
  remove U1's GND fingers between castellations. Changing the shared library is a house-wide decision (Mantis U19, J2, S1,
  J4, FL1/FL2; Beetle J2, S1, J3 have the same condition), outside this board's scope.
- **Prior:** none.

### L6. No fiducials — minor

- **Where:** whole board, F side.
- **Evidence:**
  - 0 fiducials. U3/U4 have 0.25 mm pads at 0.4 mm pitch; FL4/FL5 have 0.30 × 0.375 mm lands. AE1 is the only B-side part
    (through-hole, no B.Paste), so B needs none.
  - Space Bug V3 has FID1–FID3; Mantis and Beetle 4 each. Mantis WORKLIST H-5: "two per side, which is this fab's
    requirement"; Beetle's review (item 23) says JLC publishes no on-board rule; JLC's Assembly T&C (2026-09-09) require
    fiducials and add them on rails by default. The SAM carrier has none and was hand-assembled.
  - Three Fiducial_0.5mm_Mask1mm at (47.35, 28.34), (58.65, 55.74) and (62.85, 33.94) pass DRC after a refill (the same
    17 entries). The first two are point-symmetric about the board center (53.15, 42.04), so the third is required. An
    alternative set (43.2, 33.5), (57.3, 56.0), (62.85, 33.94) also passes, with more clearance to exposed copper
    (2.35–2.42 mm against 1.08 mm to the H1/H4 rings) but a shorter baseline.
- **Consequence:** low; placement is referenced from the rails or done by eye.
- **Proposal:** add the three, Reference hidden (the stock "FIDn" silk would land outside the edge for FID1). Add them to the
  schematic excluded from the BOM, or set "Not in schematic", so parity stays clean. Or record in the fab notes that
  assembly relies on JLC's rail fiducials.
- **Prior:** Space Bug L13.

### L7. U1 VCC decoupling is 1.8–2.0 mm from pin 8; SkyTraq asks for it directly adjacent — minor

- **Where:** U1.8 VCC (48.575, 37.760), U1.6 V_BCKP (48.575, 35.560); C5 100 nF (45.93, 38.65); C1 22 µF (46.84, 41.26);
  F.Cu +3V3 zone 44.63–48.74 × 35.22–41.24.
- **Evidence:**
  - Pad edge to pad edge: C1.1–U1.8 1.843 mm, C5.1–U1.8 2.022 mm, C5.1–U1.6 3.521 mm. The Space Bug's C8 was 0.400 mm.
  - PX-DS rev 2 p12: "Bypass capacitors should be placed close to the module VCC pin" (ripple under 50 mVpp); p13:
    "located directly adjacent to the pin they are for". The p7 application circuit draws no VCC capacitor at all.
  - Laplace solve on the pour over In1 at 0.0994 mm: C5.1 → U1.8 about 1.65 squares ≈ 0.21 nH; C1 ≈ 0.20 nH; to U1.6
    ≈ 0.45 nH. The module's GND pins are at least 7.7 mm from pin 8 wherever the capacitor goes. V_BCKP draws about 54 µA.
- **Consequence:** low: about 0.2 nH of plane-pair inductance, not the 1–2 nH first estimated. It departs from the datasheet
  text and regresses from the Space Bug; the SAM prefab review (09-02 S3) judged 1.5–3.0 mm fine. File it as a note if the
  owner prefers not to touch the layout.
- **Proposal:** move C5 (no BOM change), horizontal, centered at about (46.97, 37.30):
  - +3V3 pad at x ≈ 47.45, 0.345 mm from U1.8; the pour stays one piece.
  - GND pad at x ≈ 46.49 in the F.Cu GND pour, with a 0.4/0.3 GND via in it at about (46.35, 37.30): 0.17 mm pad to track
    and 0.22 mm hole to track against In2 Net-(J3-Pad2). Keep the old via at (45.93, 38.17) as a stitch.
  - Keep-out: no through via in x 46.42–47.60 between y 34 and 39.7, where In2 carries J3 Pad2/Pad1/Pad5 at x
    46.783/47.010/47.237 (S1's fix keeps the Pad5 lane below y 33.6). The first-pass spot at x 47.3–47.45 fails: its via
    lands on the J3-Pad5 track and its pad splits the pour.
  - Refill and re-run DRC.
- **Prior:** Space Bug §7 "U1 decoupling"; SAM prefab-review-2026-08-04 B5; SAM field-review A5.

### L8. Nine non-GND vias have no GND return via within 1 mm (the owner's rule) — minor

- **Where (distance to the nearest GND via):** R7.2 (46.05, 32.94) 3.49 mm; R8.2 (46.03, 31.94) 3.16; J3.2
  (54.01, 49.425) 1.48; J3.5 (51.01, 49.425) 2.72; C1.1 1.90; R3.1 2.18; U2.5 1.89; U2.7 1.80; FL1.1 2.02.
- **Evidence:**
  - 146 vias, all 0.4/0.3: 129 GND, 17 not; 9 of the 17 fail the rule.
  - Stack F / 0.0994 / In1 / 0.55 / In2 / 0.1088 / In3. A Laplace solve of the In2 trace puts 83–89 % of its return off In1
    at each F→In2 hop, so a GND via is the right fix on this stack.
  - Roles: only Net-(J3-Pad1) (host TX) is host-driven, and its offending via is at the receiver end behind R7 (about
    3 mA). J3.2 carries receiver TX through R8 (slow edges). J3.5 is undriven until S1. The EN vias carry bias current,
    R3.1 0.15 mA DC; VSYS returns on VSS through FL1; C1.1's nearest GND via is C1.2's own pad via.
  - The detour adds about 0.4 nH (estimate), about 1 mV transients, between solid In1/In3 stitched by 129 vias, 8.6–13.7 mm
    from the feed.
- **Consequence:** rule compliance; close to note level in RF impact.
- **Proposal:** add 0.4/0.3 GND vias in this order:
  1. (46.52, 32.40) for R7.2, 0.72 mm away (not (46.50, 32.40), which is 0.098 mm from the R8.2 pad).
  2. (45.43, 31.94–31.99) for R8.2, 0.60 mm away.
  3. J3.5: decide after the S1 re-route; if the via stays, (50.37, 49.54) is clean at 0.65 mm.
  4. Rule-only, optional: VSYS (49.26, 53.43); R3 (47.58, 47.54); EN (43.58, 48.85) and (44.91, 48.85) (not y 48.86,
     which is 0.095 mm from the In2 EN track).
  - Leave J3.2 and C1.1: no legal site without moving copper. All seven vias together: DRC 0 errors, 0 unconnected, the 7
    baseline warnings.
- **Prior:** Space Bug L3.

### L9. C136 (22 µF on the pack rail) sits beside H3 and near the left edge; a resistive crack would not be interrupted — minor

- **Where:** C136 at (45.73, 51.29), F, rot 180: pad 1 Net-(U2-EN) at (46.68, 51.29), pad 2 GND at (44.78, 51.29). H3 at
  (44.60, 54.49) with a Ø3.8 GND land. Left edge x = 42.15. The Net-(U2-EN) pour comes within 2.075 mm of H3's center.
- **Evidence:**
  - **Geometry.** C136's center is 3.39 mm from H3; its pad 2 edge 2.48 mm from the hole center (0.58 mm outside the
    land). Its long axis is tangential to H3 (the favorable orientation for screw-boss bending) and perpendicular to the
    left edge (unfavorable for tab snapping); the pad edge is 2.13 mm from the edge. On the SAM carrier C136 is at least
    8 mm from every hole and 5.3 mm from the edge, so this exposure is new. The cause is U2 sitting at the left edge (pin 8
    4.64 mm from it); C136 is already 2.09 mm from pin 8, as on the SAM carrier. The same edge carries C135 at 0.53 mm, U2
    at 0.55 mm and C7 at about 1.1 mm.
  - **Part:** CL21A226MOQNNNE, 16 V X5R, standard terminations.
  - **Bolted crack (estimate):** series 0.80–1.27 Ω including the carrier FL1's two windings (2 × 0.31 Ω max) gives
    5.1–10.5 A. FL1 runs at 16–33× its 320 mA rating, 8–22 W per winding (adiabatic estimate: fuses in about 4–66 ms).
    The host's U27 dissipates 2.9–8.7 W and reaches its 160 °C thermal shutdown (30 °C hysteresis, SLVSDH0C) in every
    corner, then cycles. VBAT_CON stays at 5.2–7.1 V.
  - **Resistive crack, 5–50 Ω:** 0.12–1.45 A and 0.8–10.5 W in the part over 6.4–8.4 V. Nothing interrupts it in the
    10–50 Ω band (U27 ≤ 0.06 W).
- **Consequence:** low probability: clamping onto standoffs about 0.2 mm out of plane gives roughly 4e-4 to 1e-3 surface
  strain (estimate), below the AEC-Q200 bend-test level; edge snapping needs tabs or a V-score on the left edge. A bolted
  crack is cleared by FL1 or by U27's thermal shutdown. A resistive crack can char the board inside the airframe and shows
  only as extra pack current.
- **Proposal:**
  1. **Now, no layout change:** fab notes "routed single boards, or tabs only on the top and right edges; no tabs, mouse
     bites or V-score on the left or bottom edge" (left: C135 0.53, U2 0.55, C7 about 1.1, C136 2.13 mm; bottom: D2 1.93,
     FL1 2.11 mm), after Mantis A9. Every F-side bearing face at H3 round and at most 3.8–4.0 mm across (no DIN 125
     washers, no 4 mm AF hex standoffs); screws torque-limited onto standoffs of equal height. See K2 and D11.
  2. **If the layout is reopened:** move C136 to the interior, for example centered at (50.6, 46.7), rot 90 (at least
     8.2 mm from every hole and 7.7 mm from every edge, no courtyard conflict). Keep C3 at U2 pins 7/8 as the local bypass
     and route Net-(U2-EN) on F.Cu between R3 and C3 or on In2, which already carries it.
  3. I found no catalogued 22 µF / 16 V / 0805 soft-termination part. A smaller soft-termination substitute must be checked
     against the ADP7142's 1.5 µF CMIN at 8.4 V DC bias.
  4. Optional firmware check: pack current with GPS_ACT on and off at boot (about 125 mA expected). A running threshold in
     flight would be masked by servo and radio swings.
- **Prior:** none (new exposure relative to the SAM carrier).

---

## 2. Fix before fab — schematic and footprints

### S1. J3 pin 5 is wired through R9 to the receiver's RXD2 input, not to 1PPS; 1PPS reaches only the LED — decision for the owner

- **Where:** J3.5 (51.01, 49.425) → Net-(J3-Pad5), 45.1 mm of 0.127 mm track on In2 → R9.2; R9 (1 kΩ) at (60.11, 33.30);
  R9.1 → global label RXD2 → U1.15. U1.4 1PPS (48.575, 33.36) → Net-(U1-PPS), 5.37 mm on F.Cu → R1.1 → D1 only.
  `hardware/README.md` line 45.
- **Evidence:**
  - **Netlist:** Net-(J3-Pad5) = {J3.5, R9.2}; RXD2 = {R9.1, U1.15 "RXD2_15", input}; Net-(U1-PPS) = {U1.4, R1.1};
    Net-(D1-A) = {R1.2, D1.2}. The schematic plot shows the RXD2 label, then R9, then J3 pin 5. The schematic is unchanged
    since `1cf7862d` (`git diff` to `c942cfd1` touches only the .kicad_pcb).
  - **Datasheet:** PX-DS rev 2 p6, pin 15: "RXD2: UART serial data input"; in the idle condition it "should be driven
    HIGH". p5 pin 4 is 1PPS; p12: 100 ms high, rising edge on the UTC second, LOW before a fix.
  - **Documented intent:** my session notes and README line 45 (`95b14623`, 2026-10-11) say "pin 5 is PPS out, as on
    the SAM carrier". On the SAM carrier
    Net-(J3-Pad5) = {J3.5, R9.2} and PPS = {U1.7 TIMEPULSE, R1.1, R9.1}; its FABRICATION-NOTES line 37 and bom.csv row 10
    define pin 5 as 1PPS out with the host pin an input (row 10 also notes that the older PX1105R carrier uses the pin as
    RXD2). This board follows the legacy convention.
  - **Host:** Mantis J1.5 = GNSS_RXD2 → U17 pad 2 (P4 GPIO2), no external pull, unused by any firmware (git grep).
    `board_v9.h` lines 68–91 list the SAM carrier (pin 5 = PPS) and the legacy carrier (pin 5 → RXD2). GPIO2 resets to
    input with a weak pull-up, 45 kΩ typ (ESP32-P4 DS v1.2 Table 2-1, Table 5-4), while EFUSE_DIS_PAD_JTAG = 0.
- **Consequence:**
  - The flight computer gets no hardware time mark. Without it, GNSS/INS fusion has to timestamp NMEA on UART arrival:
    tens of ms of variable latency is several meters along track at 250–300 m/s (estimate).
  - Firmware written to the README would read a constant high (the 45 kΩ host pull-up through R9) while D1 still
    blinks; on the bench that looks like a firmware bug.
  - U1.15 idles high on Mantis only through that host pull-up; on a 4-wire bench cable it floats, against the datasheet.
  - Nothing is damaged, and the UART on pins 1/2 works. As built, pin 5 has the legacy carrier's direction (module input),
    so it adds a row to `board_v9.h` but not a third direction. After the order, PPS would need a cut and a 14–16 mm wire,
    which is not flight-grade.
- **Who changed it.** The "pin 5 = PPS out" wiring was mine: the Claude session of 2026-10-08 transplanted the SAM
  carrier's PPS on pin 5 and added R4, a 10 k RXD2 pull-up, saying it could be deleted. I exported netlists from the
  project's own backups with `kicad-cli`. `2026-10-08_082952` has PPS = {U1.4, R1.1, R9.1} and R4 on U1.15.
  `2026-10-08_083155` has RXD2 = {R9.1, U1.15}, Net-(U1-PPS) = {U1.4, R1.1} and no R4. Both PPS global labels became RXD2
  labels at the same positions in that two-minute window, when no agent touched the schematic. The board was then routed
  J3.5 → R9 → U1.15 and kept that way through about 75 saves. The README line and my session notes predate that knowledge
  (the notes still list R4), so the documents are what is stale, most likely not the copper. A GUI edit's intent cannot be
  proven from the files, so this is a question, not a defect.
- **If RXD2 is intended (the copper stays):** correct README line 45 and add a `board_v9.h` row "gnss-px1105r-18mm-hv: pin
  5 = RXD2 (FC output, module input)", the same direction as the legacy carrier. Record in the fab notes that U1.15 idles
  high only through the host's weak pull-up and floats on a 4-wire bench cable. See S2 for R9's value. Accept that the
  flight computer has no hardware time mark (the consequence above).
- **Proposal if PPS out is wanted instead:**
  1. **Schematic:** replace the RXD2 global label on R9.1 with a PPS label and put the same label on the U1.4–R1 wire, so
     the net is {U1.4, R1.1, R9.1}, as on the SAM carrier. R9 stays 1 kΩ as the source resistor, keeping the "R9 = pin 5"
     identity that `board_v9.h` quotes.
  2. **U1.15:** a no-connect flag, no copper. PX-DS p7 takes RXD2 only to a test terminal, and p8 "Rover Mode
     Configuration 2" leaves it open with RTCM entering on RXD (pin 3). Add a 100 kΩ pull-up only if the bench shows RXD2
     floating or active; no +3V3 copper is near U1.15 today, so it would need about 15 mm of new route and 2 vias.
  3. **Layout:** R9 at about (46.3, 34.0), rot 0. Pad 1 faces west and takes a ~1.8 mm F.Cu stub to the PPS track; pad 2
     faces east onto a 0.4 mm via at about (47.24, 33.6) on the existing In2 lane. That is the only DRC-clean via spot on
     the lane: from y 33.9 to 39.36 the lane runs 0.227 mm center to center from Net-(J3-Pad1), so a via there shorts to it,
     and above y 33.5 R7.1 is in the way. Margins are 0.11–0.14 mm. Then delete the lane above y 33.6, the y 28.68 run, the
     right-hand legs, the via-in-pad at (60.62, 33.30) and the RXD2 stub. Net-(J3-Pad5) drops from 45.1 mm to about 24 mm
     and nothing of it is left near RF_IN. Re-run DRC with parity; the final placement is the router's call.
  4. **Firmware docs:** add a `board_v9.h` row "gnss-px1105r-18mm-hv: J3.5 → R9 (1 k) → U1.4 1PPS, module output, FC
     input", under the existing rules (GPIO2 stays an input; the JTAG eFuses are never burned).
  5. **Fab notes:** record the J3 pin map with directions in the new FABRICATION-NOTES.md (D1).
- **First-article check:**
  - Unpowered DMM: J3.5 to U1.4 reads 1 kΩ and J3.5 to the U1.15 castellation reads open (the as-drawn board reads the
    reverse).
  - On the Mantis, outdoors: scope J3.5 or GPIO2. Expect low before the fix, then a 1 Hz, ~100 ms high pulse. VOL about
    0.47 V or less (0.4 V + 72 µA × 1 kΩ); VOH must reach the P4's 2.475 V VIH (PX's 2.4 V minimum is a paper shortfall,
    M8). The rising edge falls on the second in GNZDA/RMC.
  - Ten power cycles through GPS_ACT with the GPIO2 pull-up present; NMEA must appear every time (1PPS is not a documented
    strap).
  - With pin 5 open, read U1.15 with a 10 MΩ DMM and compare supply current open against a 100 kΩ pull-up; add the pull-up
    next spin only if it floats or the current differs.
- **Prior:** Space Bug F5; SAM prefab-review-2026-08-04 B8; SAM field-review §6 item 5; Mantis prefab-review-2026-08-05
  (#837 item 4).

### S2. R7 and R9 are 1 kΩ: a host UART idling high back-feeds the dark carrier with up to about 2.7 mA per pin — minor

- **Where:** R7 (46.56, 32.94): J3.1 → R7 → GNSS_RX → U1.3. R9 (60.11, 33.30): J3.5 → R9 → RXD2 → U1.15. R8: U1.2 TXD →
  R8 → J3.2. U2 VOUT (pins 1–3) against VIN (7/8). Host: Mantis J1.1 = GPIO4 (FC TX), J1.5 = GPIO2; U27 TPS22810 with
  EN = GPS_ACT (R82 100 k to GND) and QOD on VOUT.
- **Evidence:**
  - BOM: R7–R9 = 1 k (RC0402FR-071KL). R7 and R9 sit at the U1 end, so the cable capacitance is on the host side.
  - PX-DS rev 2 p5 (RXD): "ensure that this pin is not driven to HIGH when primary power ... is removed, or a 10K-ohm
    series resistor can be added to minimize leakage current". p6 (RXD2): the same text, conditioned on sleep. Input abs
    max VCC + 0.5 V (p6). SkyTraq offers 10 k as the alternative; it does not require it.
  - ADP7142 Rev H Table 3: VOUT −0.3 V to VIN; the pass device is PMOS (p15), so a VOUT→VIN body diode exists, and no
    reverse-current protection is stated.
  - **When the carrier is dark with the host UART alive:** GPS_ACT low; the Mantis on USB only (power-eco.md lines
    142–144: VBATT dead); VBATT interruptions while the P4 rides its supercap (v10-power-parity, table at lines 174–176: a
    bounce under 0.27 s is dark for the bounce, a 0.3–1.0 s bounce for 1.8–4.3 s, eFuse retries blink at about 92 ms, a
    severed or flat pack indefinitely). The #700 rule in main.cpp (~4018–4030, GPS_ACT raised once) covers none of these.
  - **The UBX driver makes it worse:** `uartBegin()` never parks the pins and a failed `begin()` leaves the driver
    installed, so on a USB-only session TX idles high for the whole session. Its swapped-orientation probes also drive
    GPIO3 push-pull into R8 and U1.2.
  - **Numbers (estimate):** (3.3 − 0.6 − V(+3V3)) / 1 kΩ ≤ 2.7 mA into a push-pull-driven pin (R7 today; R8 after a
    swapped probe). R9 sees only the 45 kΩ GPIO2 pull-up: about 59 µA. With 10 kΩ: ≤ 0.27 mA.
  - The SAM prefab review (08-04 C7) accepted 1 kΩ against u-blox's rule.
- **Consequence:** U1's inputs sit above VCC + 0.5 V and U2 runs with VOUT above VIN, both unrated but limited to
  milliamps, so damage is unlikely. +3V3 is partly pre-biased (not measured). RSTN is tied to the same rail by R2 and the
  host has no reset line, so a receiver that does not restart cleanly from a pedestal (plausible, not shown) stays down for
  the session. The owner already accepts a cold start after any VBATT interruption (F1).
- **Proposal:**
  - **R7 → 10 kΩ** (RC0402FR-0710KL, already on the BOM as R1–R3). 10 kΩ × 4–10 pF (pad, 1.8 mm of trace, module input;
    estimate) is 40–100 ns against 8.68 µs per bit at 115200 and 1.085 µs at 921600; ±10 µA × 10 kΩ = 0.1 V keeps VIH.
  - **R9 → 10 kΩ** only if pin 5 stays host-driven RXD2 (optional: 59 → 49 µA). Moot if S1 makes R9 a PPS source resistor.
  - **R8:** either 4.7 kΩ (its host-side node carries about 21 mm of In2 plus the harness, so 10 kΩ × 10–15 pF gets tight at
    921600), or remove the swapped-orientation probe for boards with a fixed pinout.
  - If a carrier pull-up is ever added on GNSS_RX (S7), make it ≥ 100 kΩ.
  - **Firmware:** on `begin()` failure, delete the UART driver and set GNSS_TX/GNSS_RX to inputs with pulls off
    (`gpio_reset_pin()` re-enables the 45 kΩ pull-up, acceptable at about 60 µA). Optionally skip GNSS bring-up when
    VBATT is absent (eFuse PG or INA230 via the OC, #721).
  - **Bench:** Mantis on USB only with the carrier fitted: measure +3V3 at C1 and the current into J3.1 (mV across R7).
    Plug the pack and confirm NMEA restarts by scoping J3.2 directly (the FC will not retry and has no SkyTraq driver).
    A faint glow on D3 means a pedestal above about 2.4 V. Then, with the host UART idle high, open VSYS for 1 s and 5 s,
    50 cycles, log +3V3 and U1.3, and confirm NMEA resumes every time.
- **Prior:** SAM prefab-review-2026-08-04 C7; Space Bug S13 (back-feed analysis) and its "Other first-article checks".

### S3. U2's board copy prints its full 2.41 × 3.05 mm exposed-pad land; the library's 2 × 2 panes never reached it — minor

- **Where:** U2 SOIC127P600X175-9N at (44.88, 46.65), rot −90. Pad 9 (GND) spans x 43.355–46.405, y 45.445–47.855, with six
  GND vias. DRC reports lib_footprint_mismatch on U2.
- **Evidence:**
  - **Board copy:** pad 9 on F.Cu/F.Mask/F.Paste, 2.41 × 3.05 mm, no paste margin. F_Paste plots one 3.05 × 2.41 flash,
    7.35 mm². The package EP is 2.29 × 2.29 = 5.24 mm² (ADP7142 Rev H p22, RD-8-1): the land is 1.40× by area.
  - **Library** (md5 `544bf2e1`, changed in `7654e58c`): pad 9 on F.Cu/F.Mask only, plus four 1.0 × 1.0 mm roundrect paste
    panes at (±0.625, ±0.625): 3.785 mm², 72 % of the EP and 51.5 % of the land. That commit's message already says the
    SAM carrier's U2 shows a mismatch until updated; this board inherited the SAM V3 copy. On the board the panes would land
    at (44.255/45.505, 46.025/47.275); four of the six EP vias have their centers under panes, and (44.90, 47.48) and
    (44.84, 45.99) straddle the 0.25 mm gaps. The board declares filled and capped vias.
  - **Volume, not area:** on a leaded EP SOIC the leads set the height. Filling the EP gap (A1 0.05 mm nominal, 0.10 max,
    plus the lead joint, about 0.07–0.09 mm in all) takes about 0.37–0.47 mm³ of solder (estimate). Full-land paste
    delivers about 0.29 / 0.37 / 0.44 mm³ of solder at 80 / 100 / 120 µm (0.5 metal fraction); the panes 0.15 / 0.19 mm³ at
    80 / 100 µm. TI's DDA0008J example stencil (TPS5430, SLVS632L) prints 100 % of a larger land and scales to about
    0.82 mm³ of paste for this EP.
  - The SAM V3 carriers were fabbed with the full land, but U2 there dissipated 77–204 mW on a 120 µm foil; I found no
    solder-quality record for it.
  - Emulating Update-from-Library on U2 alone: DRC 7 → 6 warnings, 0 errors, 0 parity issues; the pin-1 dot is unchanged.
- **Consequence:** the excess-solder / float mechanism the first pass expected does not hold at this board's likely 80 µm
  foil (D1); if anything the panes risk under-filling the thermal joint under the hottest part (0.39–0.64 W, M2). The real
  risk is the unrecorded divergence: a bulk "Update footprints from library" would silently change the paste under U2.
- **Proposal:**
  - Settle the foil first (D1: 80 µm).
  - At 80–100 µm, keep the full-land print (0.59–0.74 mm³ of paste) and record why in FABRICATION-NOTES: leaded EP,
    foil-limited volume, SOLDER-PASTE-CONVENTION §2 ("do not spend standoff"). Clear the warning without changing the
    paste: point U2 at a full-paste library variant (for example `SOIC127P600X175-9N_FullPaste`), or exclude the U2
    mismatch with that note.
  - Use the library's panes only with a 120 µm foil and a voiding concern; at 80 µm, enlarge them to about 1.15–1.2 mm
    (about 5 mm²) rather than using the 3.8 mm² set.
  - Either way keep the six filled-and-capped vias and X-ray U2's EP on the first article (M19, row 4).
- **Prior:** Space Bug S11.

### S4. U1 pin 18 has no no-connect flag (both ERC errors), and the PX1105R symbol's pin types are loose — minor

- **Where:** U1 pin 18 at sheet (135.255, 68.58) mm, net unconnected-(U1-Boot-Pad18); symbol Custom:PX1105R (library and
  schematic cache identical).
- **Evidence:**
  - ERC (KiCad 10.0.3), reproduced: 91 entries, 84 endpoint_off_grid, 5 pin_to_pin and 2 errors (pin_not_connected and
    pin_not_driven), both on U1 pin 18 [Boot, Input]. Pins 5, 13 and 14 already carry flags.
  - The SAM carrier gives 96 entries, all off-grid, with 0 errors and 0 pin_to_pin: the two errors are what remained after
    TP1 was removed; only the off-grid warnings are inherited.
  - PX-DS rev 2 p6, pin 18: "No connection for normal use". Open is correct and is the owner's choice.
  - Symbol pin types: pins 1–6 unspecified; 8, 10, 12 input; 13 (NC) input; 14 VCC_RF output; 15 RXD2 input (correct);
    16/17 no_connect but named NC (datasheet: SDA/SCL); pin 6 named V_BCKUP (datasheet V_BCKP), pin 18 named Boot
    (BOOT_SEL). The 5 pin_to_pin warnings (U1.6/U1.8 on +3V3, U1.4–R1.1, U1.3–R7.1, U1.2–R8.1, U1.1–#PWR042) are all
    correct connections.
  - `docs/board-versioning.md` lines 76–78: the release gate is DRC with parity plus `check_board_parity.py`; ERC is not
    part of it.
- **Consequence:** no circuit fault. With two standing errors, ERC cannot work as a regression check: a new real error does
  not change the "errors present" state.
- **Proposal:**
  - Place a no-connect flag at (135.255, 68.58); commit it as recording the deliberate TP1 removal. No copper change.
    Tested in a copy: 0 errors.
  - Optional retype of U1 only (refresh only U1's cache: the FL4/FL5 and U3/U4 caches also differ cosmetically from the
    library): GND, VCC, V_BCKP power_in; TXD, PPS output; RXD, RXD2, TRIG, RSTN, BOOT_SEL input; VCC_RF power_out; pin 13
    no_connect (7, 16, 17 already are); rename pin 6 V_BCKP, 16/17 SDA/SCL, 18 BOOT_SEL. Tested: 0 errors, 0 pin_to_pin.
  - Optionally add "ERC 0 errors" to the release checklist and record the 84 off-grid warnings as inherited.
- **Prior:** Space Bug S14; SAM prefab-review-2026-08-04 C8.

### S5. AE1 feed hole: the Ø1.2 mm pads on B.Cu and In1–In4 leave a 0.10 mm ring on the 1.0 mm hole — minor

- **Where:** AE1 pad 1 at (53.19, 40.82), front-inner-back padstack: F.Cu Ø2.2, In1–In4 and B.Cu Ø1.2, drill 1.0.
- **Evidence:** ring (1.2 − 1.0)/2 = 0.10 mm. JLC (fetched 2026-10-12), multilayer 1 oz PTH: "Recommended 0.20 mm or
  above; absolute minimum 0.15 mm"; hole tolerance +0.13/−0.08 mm. AE1 is the only PTH pad under 0.15 mm; the board's via
  annular minimum (0.05 mm) cannot flag it. GV-DS rev B p9 draws exactly Ø1.20 (top) and Ø2.20 (bottom) on Ø1.00. The
  inner pads connect to nothing inside the Ø3.0 keep-out. The Space Bug V2 (ordered 2026-09-29) carried the identical
  padstack; no DFM query is recorded.
- **Consequence:** a DFM hold or a silent CAM change; breakout of the non-functional rings is likely. Electrically harmless:
  the joint is on the Ø2.2 F.Cu pad with a 0.6 mm ring.
- **Proposal:** in the library footprint ANT_GVLB208.A_TAOGLAS (antenna side = front), change the front and inner sizes
  1.2 → 1.4 mm (0.20 mm ring), leave B.Cu 2.2 and the drill 1.0, then update AE1 from the library. The Ø3.0/Ø3.5 keep-outs
  stay (0.80 mm gap remains); the added fringe capacitance is hundredths of a pF (estimate). Alternatively set the pad to
  remove unconnected layers. Or accept it with one FABRICATION-NOTES line ("AE1 1.00 mm plated hole: 0.10 mm ring on B.Cu
  and inner layers per the antenna maker's land; non-functional; fab may enlarge or remove them"). The GVLB258.A footprint
  shares the land.
- **Prior:** Space Bug N14.

### S6. D6's footprint description inverts TI's mask preference — minor

- **Where:** D6 Texas_DPY0002A at (55.61, 44.85), rot 90; both pads 0.3 × 0.5 mm roundrect, mask margin +0.05 mm.
- **Evidence:**
  - Footprint descr: "TI: pads not mask-defined, preferred, up to 0.07 mm". TI SLVSDG9C p24 (drawing 4224561/C):
    "NON SOLDER MASK DEFINED ... 0.07 MAX ALL AROUND" and "SOLDER MASK DEFINED (PREFERRED) ... 0.07 MIN ALL AROUND".
    Copper size and pitch and the p25 1:1 paste match TI. The wording came from the Space Bug review's S10; the descr also
    cites mpss034c and tpd6e05u06, neither of which has this land. The same footprint is on space-bug-m8t and the legacy
    Space Bug.
  - On this board the GND pour connects SMD pads solid and fills the 0.05 mm ring around pad 1 (which has a via in pad), so
    pad 1's wettable land is mask-defined at 0.4 × 0.6 mm (0.227 mm²) while pad 2 exposes 0.155 mm² (pad plus trace
    neck). Paste is 0.145 mm² per pad: about 1.46:1.
- **Consequence:** low physical risk (ESD-only part; NSMD is an allowed option), but the library text misstates the
  datasheet, and the pour makes the lands asymmetric on a 1.0 × 0.6 mm two-pin part.
- **Proposal:** correct the descr to cite SLVSDG9C p24 and drop the wrong references; update the board copy. For this board
  prefer TI's SMD option: copper about 0.44 × 0.64 mm, solder_mask_margin −0.07 and solder_paste_margin −0.07 (without the
  paste margin KiCad prints 0.28 mm² per pad instead of TI's 0.15 mm²). Refill and DRC; estimated gaps 0.26 mm pad to pad
  and 0.29 mm to C20.2, with the pour back at 0.2 mm. If NSMD stays, record the asymmetry.
- **Prior:** Space Bug S10.

### S7. The receiver's RXD floats from carrier power-up until the FC configures its UART — note

- **Where:** GNSS_RX = {R7.1, U1.3}; Mantis J1.1 → U17 pad 4 (GPIO4).
- **Evidence:** no pull on either side of R7. ESP32-P4 DS v1.2 (chip v1.3) Table 2-1, after reset: GPIO4 "IE", GPIO3 "IE",
  GPIO2 "IE, WPU". PX-DS p5: RXD "should be driven HIGH" when idle; IIL/IIH ±10 µA (no strong internal pull, by
  inference). main.cpp:4027–4031 raises GPS_ACT; GPIO4 is first configured in `uartBegin()` via main.cpp:4647 →
  TR_Sensor_Collector.cpp:715, after the baro, magnetometer and IMU init. The window is unmeasured, about 0.1–1 s
  (estimate). TR_GNSSReceiverUBlox_Serial.cpp:40 pulls up only the host RX pin.
- **Consequence:** framing noise at a floating input on each boot. A false SkyTraq command needs 0xA0 0xA1, a length, a
  payload, a checksum and 0x0D 0x0A, so it is implausible; today's UBX probing sends far more foreign bytes anyway.
- **Proposal:** firmware: `gpio_pullup_en(GPIO_NUM_4)` right after main.cpp:4030 raises GPS_ACT (45 kΩ). Not output-high:
  +3V3 is still ramping (about 1.5 ms soft start), and a hard high through R7 would back-feed the receiver (S2). Optional
  hardware: 100 kΩ (47 kΩ works) from U1.3 to the carrier's own +3V3; a host low through R7 then reads at most 0.07 V (1 kΩ)
  or 0.58 V (10 kΩ), below VIL 0.8 V.
- **Prior:** Space Bug S3.

### S8. U3/U4 single mask windows also expose pour, via caps and track stubs: about 29 % more wettable copper than the pads — note

- **Where:** U3 window 61.165–61.915 × 41.025–42.175; U4 window 61.305–62.055 × 37.205–38.355.
- **Evidence:**
  - Copper in each window 0.457 / 0.454 mm² against 0.354 mm² of pads (+0.10 mm² each), of which about 0.06 mm² is GND pour
    solid-tied to pins 1 and 4. GND lands come to 1.5× a bare pad and 1.2–1.4× a signal land (signal stubs are exposed too).
  - The pin-4 via discs are mostly capped drill (only about 0.005 mm² is ring); track stubs are 0.0014–0.015 mm² per
    segment, about 0.04 mm² per part.
  - The extras are point-symmetric: the copper centroid is within 0.01 mm of the part center. Exposed GND is 0.151–0.157 mm
    from the signal pads.
  - The single window follows Infineon BGA855N6 Rev 2.2 p10 Fig. 4 (NSMD, one opening, round Ø0.25 apertures, 100 µm).
- **Consequence:** no net pull; GND joints about two thirds the height of the others (estimate: about 22 against 33 µm) and a
  little more heat-sinking at pins 1 and 4. The foil choice (D1) matters far more.
- **Proposal (optional):** an F.Cu no-pour rule area the size of the window (or 0.02 mm larger) in
  Infineon_PG-TSNP-6-10, so U3 and U4 inherit it; then give each pin 1 a short 0.2 mm GND track out to the pour, because
  neither U3.1 nor U4.1 has a via or track today (see L3), or DRC flags it unconnected. Keep pin 4 and its via; do not move
  the pin-4 vias, which are the LNAs' lowest-inductance ground. The excess drops from about 29 % to 12 %. Record "via-in-pad,
  filled and capped" in the fab notes.
- **Prior:** Space Bug S10.

---

## 3. Measure on the first article

Risks the files cannot settle.

### M1. The patch sits on 13.5 % of Taoglas's 70 × 70 mm minimum ground plane — major (risk)

- **Where:** AE1 on B at (53.19, 42.02); body x 43.19–63.19, y 32.02–52.02. B.Cu GND fill x 42.35–63.95, y 27.24–56.84
  (604.2 mm², one piece). H1–H4 Ø3.8 GND pads 0.57–0.63 mm from the body.
- **Evidence:**
  - **Plane size.** GV-DS rev B p7/p12: "a minimum of 70x70mm ground plane". The carrier alone is 22.00 × 30.00 mm
    (659 mm² with R1 corners), 13.45 % of 4900 mm². The Space Bug was at 25 % (its M2).
  - **Margins.** Ground beyond the nominal 20 mm body: 0.84 mm left, 0.76 right, 4.78 top, 4.82 bottom (1.04 / 0.96 mm to
    the board edge). At the maximum body size (20 ± 0.2 mm is ±0.1 per side) that is 0.74/0.66 mm; the pin-position
    tolerance and hole play could take one side to about 0.3–0.4 mm. The patch is centered within 0.04/0.02 mm.
  - **The antenna's own curves** (GV-DS p16–18, re-digitized): −10 dB band 1564.1–1585.2 MHz at L1 and 1168.2–1181.1 MHz at
    L5 (13 MHz). The return-loss dip midpoints (1171/1178 → 1174.5 MHz; 1568.5/1580 → 1574.3 MHz) match the axial-ratio
    minima (about 4.1 dB near 1174 and 1574 MHz), so S11 can stand in for the CP frequency. Even on 70 × 70 mm, L5 AR is
    below 10 dB only from about 1170 to 1178 MHz, and about 6–7 dB at 1176.45 MHz. L1's gain peak is at 1585 MHz;
    1575.42 MHz sits in an efficiency dip.
  - **Small-plane trend** (APN-12-8-002/B, a generic L1 patch, indicative only): Fig. 1 center 1543 MHz on an 18 mm plane,
    1573 on 30 mm, 1580.5 on 70 mm; Fig. 2 gain 0.1 dBic at 18.5 mm, 1.2 at 25 mm, 3.3 at 60 mm. Interpolated for this
    board's 21.6 × 29.6 mm copper: roughly −8 to −28 MHz at L1. The direction rests on this one generic curve.
  - **Receiver bandwidth.** FL4/FL5's L5 passband is 1166.22–1186.68 MHz: the receiver uses the whole L5 main lobe.
  - **Installed condition.** The carrier hangs under the Mantis on nylon standoffs (Mantis FAB B11), its F side toward a
    22.4 × 77.55 mm board with solid GND planes. That extends the ground in y only (31.7 mm north, 15.8 mm south), which
    adds to the x/y asymmetry; it must be measured installed.
  - **Tuning hooks.** C19 (DNP shunt), C20 (33 pF series) and L1 (5.1 nH shunt) form a pi at the feed; C20–L1–C21 is also
    the ESD high-pass.
- **Consequence (estimate):** a 5–10 MHz shift costs about 1.7–2 dB of L5 gain at 1176.45 MHz and takes the AR past
  15–20 dB (polarization loss −2.2 to −2.7 dB instead of −0.5). With 1–3 dB of small-plane efficiency loss that is roughly
  4–7 dB of RHCP gain lost at the L5 center, and 1–3 dB at L1 (the 1585 MHz peak moves toward L1 while the AR worsens).
  That threatens the reason for an L1/L5 receiver and the 37 dB-Hz RTK guideline (PX-DS p13); L1 tracking survives. It is
  a form-factor limit set by owner decisions (antenna and outline), so the action is to characterize and tune.
- **Proposal (no antenna, outline or layout change):**
  1. **S11.** Lift C20 and solder a pigtail to C19: pad 1 at (54.66, 44.61) is the feed, pad 2 at (54.66, 45.57) is GND
     through the via in the pad. D6 stays in circuit, as in service. Sweep 1100–1700 MHz with port extension to C19 pad 1,
     a ferrite sleeve on the cable and the cable routed away from the patch. Measure twice: the carrier alone, and installed
     under the Mantis on its standoffs with the J3 cable and the flight screws. Read both dips in each band; an L5 midpoint
     off by more than about 4 MHz (half the AR < 10 dB window) means several dB lost. Procedure details: M11.
  2. **Sky.** Log GSV with signal IDs (PX-DS Table 3) beside a reference PX1105R on a known-good L1/L5 antenna under the
     same sky, for 15 min or more. Per PRN, compute (carrier − reference) at L5 (GPS 7/8, Galileo E5a, BDS B2a) minus the
     same at L1/E1/B1, and count signals above 37 dB-Hz. An L5 deficit more than about 3 dB worse than at L1 confirms the
     finding. Optional: a temporary 70 × 70 mm copper skirt with a 21 mm cut-out screwed to the H1–H4 GND pads; repeat both
     tests to separate the plane effect from the rest of the front end.
  3. **Remedies, honestly.** A C19/C20/L1 rematch recovers only mismatch loss (about 0.5–1.3 dB for a −10 to −6 dB return
     loss), moves the high-pass corner, and cannot retune L1 and L5 independently or move the CP frequency. If the midpoint
     is off, the real fix is a GVLB208.A that Taoglas tunes for this plane (GV-DS p11 offers custom tuning). Ask Taoglas now,
     in parallel with the order, and expect an MOQ or NRE; it keeps the part family but needs the owner's OK. Judge any
     rematch by C/N0, not S11 alone.
  4. Record the screw material at H1–H4 (heads 0.57–0.63 mm from the walls) and test with the flight hardware. Keep any
     conducted injection with C20 fitted at −40 dBm or less (M7).
- **Prior:** Space Bug M2; SAM prefab-review-2026-08-04 C1.

### M2. The carrier heats itself: about 0.8–1.05 W on 22 × 30 mm — major (risk)

- **Where:** whole board. U2's EP (43.355–46.405 × 45.445–47.855) with 6 GND vias; U1 body around (53.485, 34.43).
  85 °C-class parts: U1, AE1, FL4/FL5, U3/U4, FL1, J3, the X5R capacitors C1/C135/C136/C27/C28.
- **Evidence:**
  - **Load.** U1 115 mA in acquisition, 95 mA tracking, typical only (PX-DS rev 2 p7); U3 + U4 2 × 4.8 mA typ, 5.8 max
    (BGA855N6 Table 4); small loads. About 125 / 105 mA on +3V3, and the same at the pack (linear regulator).
  - **Power,** acquisition at 6.34 / 7.4 / 8.4 V: U2 0.37 / 0.50 / 0.63 W, board 0.79 / 0.93 / 1.05 W (1.6 mW/mm² against
    the Space Bug's 1.3). Tracking on a loaded pack at about 8.0–8.1 V: board about 0.85 W, U2 about 0.49 W. The SAM
    carrier's U2 ran at 77–204 mW.
  - **ADP7142 Rev H thermal data:** Table 4 θJA 52.7 °C/W is on a 4-layer 4 × 3 in board (about 7700 mm²) and does not
    apply; Table 6 gives about 87 °C/W at 600 mm² of copper; ψJB 32.7 °C/W (Table 4) or 38.8 (Table 7); TJ "must not exceed
    125 °C" (p17); thermal shutdown 150 °C.
  - **Models (estimates).** A 2-D sheet model (Gsq 0.048 W/K) at 8.4 V in acquisition, open still air (h ≈ 15 W/m²K):
    board +33.6 K with the patch credited, +51.5 K without; TJ − TA +57 to +79 K. Closed bay (h ≈ 10): board +50 to +77 K,
    TJ − TA +74 to +105 K. An independent lumped estimate with the patch credited: about 28 K/W (±30 %), +29 K. The 16 g
    patch is nearly isothermal with the board (the 3M 9077 pad is about 1.7 K/W), so the no-patch case is a pessimistic
    bound. Heat capacity 13–17 J/K gives a time constant of 8–19 min: a normal pad wait reaches steady state.
  - **EP vias:** about 18 K/W each (20 µm barrel to In1), about 3 K/W for six, 2 K/W for nine. Board-to-air (30–75 K/W) is
    the bottleneck.
- **Consequence (estimates, patch credited, 8.4 V, acquisition):**
  - Open still air: the 85 °C parts reach their limit at about 50–56 °C ambient; U2's junction reaches 125 °C at about
    66–72 °C.
  - Closed bay, the flight case: about 35 °C and 47–51 °C bay air. Realistic pad tracking (about 0.85 W) raises each
    threshold by about 6–10 K. The bay air can sit above outdoor air from sun and, if it shares the bay, from the Mantis's
    own 2–3 W.
  - Above 85 °C, C/N0 and lock degrade first (TCXO, LNAs, SAWs, patch). U2's thermal shutdown (a cold start, F1) is a
    remote tail: it needs TJ 150 °C, after U1 and the patch are already 20–40 K over their ratings.
- **Proposal (no supply change; owner decision):**
  1. **Before the order:** nothing required. A middle row of three EP vias (x ≈ 43.8 / 44.87 / 45.96, y ≈ 46.74; 0.43 mm
     hole to hole) is worth less than 1 K; add it only if the layout is reopened.
  2. **First-article soak, one run.** The carrier mounted as in flight inside a closed airframe section in still air. If
     the Mantis shares the bay, power it in the same section and feed the carrier from its J1; otherwise 8.40 V through the
     A-type cable. Indoors, so the receiver stays in acquisition (log about 125 mA). Fine 36–40 AWG thermocouples on the U1
     shield center, the patch top center, board copper 1 mm off U2's long side (the ψJB reference; the EP itself cannot be
     reached) and the bay air about 10 mm from the board, shaded; log the room air as well. Measure V at C136 for
     PD(U2) = (V_C136 − 3.3) × I. Log until dT/dt < 0.5 K per 10 min (45–90 min).
  3. **Derive** ΔT(part − bay air), ΔT(bay − room) and TJ(U2) = T_board + (32.7 to 38.8) × PD. Scale to tracking by measured
     power. Maximum permitted bay air = 80 °C − ΔT of the hotter of the U1 shield and the patch, with TJ ≤ 125 °C at that
     air temperature. Add a solar margin (10–20 K in a dark airframe, estimate) or repeat once outdoors in sun inside the
     real nose cone.
  4. **Record** the envelope (maximum bay air, maximum powered pad time on hot days) in the fab notes and the launch
     checklist. GPS_ACT lets the host power the carrier late, at the cost of a cold start (F1).
  5. **Optional pre-order proxy:** a fabbed SAM V3 carrier at 8.4 V with about 27 Ω (≥ 0.5 W) on +3V3 reproduces U2's
     0.63 W and the 1.05 W total. It reads pessimistic (smaller patch, 9 % less area).
  - Not recommended: a series dropper (it removes no board heat: P_board = VSYS × I). Metal standoffs help only onto a real
    heat sink, bypass FL1's return winding (H1–H4 are carrier GND) and conflict with Mantis B11's nylon rule. A
    thermocouple on FL1 is of little use (it dissipates about 5 mW).
- **Prior:** Space Bug M3 and L7.

### M3. Second-order intermodulation in U3 from host BLE minus LoRa can land at L1/E1/B1I — minor

- **Where:** U3 input behind FL4 and the C20/L1/C21 tee. Aggressors: the Mantis ESP32-S3 BLE (chip antenna U22) and the
  LoRa E220-900MM22S on the separate LoRa board.
- **Evidence:**
  - **Firmware:** LoRa is accepted from 850 to 930 MHz (RocketComputerTypes.h:254–255) with a fixed 902–928 MHz hop plan;
    default 915 MHz at 12 dBm, 2 Hz (out_computer config.h); up to 22 dBm on the radio board. BLE defaults to +9 dBm.
  - **Arithmetic:** f_BLE − f_LoRa spans 1472–1630 MHz over 850–930 MHz, 1474–1578 MHz on the hop plan and 1487–1565 MHz
    at 915 MHz. Examples: 2480 − 904.6 = 1575.4 MHz (GPS L1, E1); 2463 − 902 = 1561 MHz (B1I). GLONASS L1 needs LoRa at
    882 MHz or below. The PX1105R does not track B1C.
  - **LNA:** AN596 Rev 1.3 Table 1, Option B: OOB IIP2 +1 dBm and OOB OIM2 −41 dBm at −20 dBm per tone (825 + 2400 MHz).
    Fig. 17 p17 (output-referred) shows the IM2 at about −25 dBm, so IIP2 ≈ +1 dBm input-referred and the table's −41 dBm
    is the input-referred level.
  - **Selectivity ahead of U3:** B8389 v2.1 p7 minimum 20 dB at 600–1112 MHz and 24 dB at 2000–3000 MHz; Fig. 4 typical
    about 26 dB at 902–915 MHz and 37–41 dB at 2402–2480 MHz (digitized). The tee adds about 0 dB beyond the datasheet's
    50 Ω ∥ 5.1 nH test condition, because L1 is that shunt.
  - **Threshold:** noise at U3's output in 2 MHz is about −96 dBm. Keeping the IM2 10 dB below it allows about −38 dBm per
    equal tone at the patch port with minimum rejection, −27 dBm with typical.
  - **Proximity:** stacked on Mantis H2/H4/H6/H8 (K4), U22 is about 12 mm laterally from the patch center; the LoRa
    antenna position is not in the files.
- **Consequence:** with BLE and LoRa transmitting together on coincident channels, U3 can make an in-band product at or
  above the noise floor that FL5 and the receiver cannot remove: brief, duty-cycle-weighted C/N0 dips on part of L1/E1 or
  B1I. L5 is untouched. The V9 field data showed no LoRa-correlated degradation (different receiver, no simultaneous BLE).
- **Proposal:** a first-article coexistence test, no change before the order:
  - (a) LoRa fixed at 915 MHz, 22 dBm, continuous packets; BLE forced to 2474–2478 MHz and advertising on channel 39
    (2480 MHz): IM2 at 1559–1565 MHz (B1I).
  - (b) LoRa fixed at 904.6 MHz with BLE advertising on channel 39: IM2 at 1575.4 MHz (L1/E1).
  - (c) Normal operation: LoRa hopping 902–928 MHz at 22 dBm, BLE connected and advertising.
  - Log per-satellite C/N0 for L1/E1/B1I and L5/E5a against a both-off baseline, from the PX1105R itself (GSV or raw output
    via a PC tool; no SkyTraq host driver yet).
  - If it fails: a static BLE data-channel map without 2462–2478 MHz (data channels 28–36), and channel 39 dropped from
    advertising, clears L1/E1/B1I for every LoRa channel in 902–928 MHz; or stop BLE after launch detection. Per-pair
    masking is not practical because the BLE controller hops. Change hardware only if firmware is not enough.
- **Prior:** Space Bug N1 (the LoRa motivation for filter-first; the IM2 mechanism is new).

### M4. Mantis clock harmonics land in BDS B1I (1560 MHz) and GLONASS G1 (1600 MHz); the receiver can drop B1I only with all of BeiDou — minor

- **Where:** host Mantis: Y4 (P4) and Y2 (S3), both ECS-400-10-37B2-CKY-TR 40 MHz crystals; P4 flash at 80 MHz; S3 CPU at
  160 MHz and flash at 80 MHz. Carrier L1 chain: AE1 → D6/C20/L1/C21 → FL4 → U3 → FL5 → U4 → C26 → U1.11.
- **Evidence:**
  - **Frequencies:** B1I is centered at 1561.098 MHz with a ±2.046 MHz main lobe; 39 × 40 = 1560.000 MHz sits 1.098 MHz
    below center, where the code PSD is −4.6 dB from peak (13 × 120 also gives 1560). G1: 40 × 40 = 20 × 80 = 10 × 160 =
    1600.000 MHz falls between k = −4 (1599.750) and k = −3 (1600.3125), inside both lobes. The P4 CPU at 360 MHz has no
    harmonic in 1559–1607 MHz. Crystal error (±10 + ±10 ppm) moves the lines by at most ±31 kHz. B8389 PB2
    (1559–1607 MHz, ≤ 2.5 dB) passes both.
  - **Threshold** (spectral-density estimate with N0 ≈ −172 dBm/Hz from N1's NF): a CW at 1560.000 MHz costs B1I about 1 dB
    at about −110 dBm at the patch port and 3 dB at about −104 dBm; G1 k = −4 loses 3 dB at about −111 dBm.
  - **Levers:** AN0037 v1.4.69 p80 (0x64/0x19) selects whole constellations only; neither AN0037 nor AN0039 has a per-signal
    or per-band selection. In the owner's outdoor capture BeiDou supplied 8 B1I satellites and 7 B2a, against 6 GPS and 6
    Galileo; B1I was used in the fix in 84 % of channel-epochs.
  - **Geometry when stacked:** the S3 (U15) is 5.1 mm, its flash (U13) 4.4 mm and its crystal Y2 9.4 mm laterally from the
    projected patch center, all inside the patch's shadow in every orientation. The P4 (U17) is at least 25.7 mm away and
    Y4 about 41 mm.
- **Consequence:** if a line couples above about −110 dBm, every B1I channel loses C/N0 together, on top of the patch's
  roughly 2.3 dB B1I deficit (F9). G1 matters only with a GLONASS firmware build (F10). The coupled level depends on the
  stack geometry, which K4 leaves open.
- **Proposal:** a first-article test on the Mantis in the flight stack geometry:
  - **Control:** the Mantis unpowered and the carrier fed through J3 from a bench supply, or both the P4 and the S3 held in
    reset. Holding only the P4 can give a false negative, because the S3 parts sit under the patch.
  - **Loaded:** the S3 on its flight firmware (160 MHz CPU, flash traffic) and the P4 on its flight load.
  - Alternate on and off in blocks of about 2 minutes in one outdoor session, so sky geometry and multipath cancel.
    Per satellite, compute (B1I − B2a) − (L1CA − L5); B2a and L5 share 1176.45 MHz, so the L5-band antenna terms cancel. A
    consistent on-minus-off drop of 1 dB or more with L1CA − L5 unchanged points to the spur. Use the absolute value
    (about +3.7 dB from the patch alone; the reference capture used another antenna) only as a sanity check.
  - Near-field probe at 1560 and 1600 MHz over U15, U13 and Y2 first, then U17, U16 and Y4. The receiver's interference flag
    (0x64/0x06, query 0x64/0x07, reply 0x64/0x83) is optional and coarse.
  - When K4 defines the stack, face the patch away from the Mantis (the carrier's ground stack then sits between the
    boards) and maximize the standoff height. Host levers as in Space Bug M1 (series parts, drive strength, clock choices).
    Keep BeiDou enabled unless the B1I loss is gross, because dropping it costs B2a too.
- **Prior:** Space Bug M1.

### M5. The feed-joint fill (K1) cannot be verified on a built unit; open joints can still be screened per unit — minor

- **Where:** AE1 pad 1, plated Ø1.0 mm at (53.19, 40.82): F.Cu Ø2.2, B.Cu and inner Ø1.2. U1 body edge y 39.48. C20 pad 1
  at (54.94, 43.68).
- **Evidence:**
  - The pin tip ends 0.25–0.65 mm below F.Cu (K1), so the lead end is never visible from the solder side.
  - A Ø0.8 ± 0.1 mm pin in a Ø1.0 mm hole leaves a 0.05–0.15 mm annulus, closed at B by the tape, venting only to F
    (inferred).
  - The feed net is DC-isolated from GND (C20 in series, D6 a TVS), so a pin-to-ground DMM proves nothing. GV-DS p6's top
    view shows the feed as a ringed bump on the top electrode ("0.6Max"); the owner's model script reads it the same way.
    If that bump is the pin (to confirm on a loose patch), a DMM from it to C20 pad 1 tests pin, joint, barrel and trace.
  - X-ray: every oblique path crosses the 8 mm ceramic; views from −y also cross U1 (2.9 mm tall), while +y and ±x clear
    the F side.
  - Coax model (estimate, 1.15 mm engagement): a fully unwetted centered pin couples through about 0.29 pF (about 11 dB loss
    at L1, 14 dB at L5); an off-center pin at a 0.01 mm gap through about 0.66 pF (5–7 dB). A C/N0 check catches gross
    cold joints, not marginal ones.
  - The 16 g patch is held by its 272 mm² adhesive pad; a wetted joint mainly carries the RF path while the tape holds.
- **Consequence:** partial fill or voids pass every non-destructive check the owner has. An unwetted pin sealed in solder
  can pass a bench check and open under vibration.
- **Proposal:**
  1. **Before the order:** keep spare bare boards from the same panel unassembled (JLC's 5-board minimum normally leaves
     them).
  2. **One-time qualification:** on a loose patch, confirm < 1 Ω from the pin tip to the top bump. Make one or two coupons
     from spare bare boards with real patches: liner off, firm pressure, and a 2.9 mm dummy block glued on U1's outline
     (edge at y 39.48) so the iron access matches. Solder with the frozen method (GV-DS p14: 360–380 °C, 3–4 s, from F;
     try a 100–120 °C preheat only on a coupon first). Take a DC reading from the bump to C20 pad 1, then saw about 1 mm off
     the pin axis, sand to the axis and inspect at 10–20×. Accept when the pin's end face and upper side are wetted, the
     barrel is wetted over at least half of the about 1.15 mm engagement, and there is no blowhole. Write the method (iron,
     tip, temperature, dwell, solder, flux, approach from S/SW) into the fab notes (D1).
  3. **Every unit, no rework:** before soldering, a loupe check that the tip is visible 0.25–0.65 mm down; afterwards a
     concave plug wetting all 360° of the Ø2.2 land with no pinhole; a DMM from the top bump to C20 pad 1 while tapping the
     patch (low and steady); an open-sky GSV C/N0 check on L1 and L5 against the first-article unit while tapping. Fail
     anything more than about 3 dB below the reference's top-4 average.
  4. **First article only:** S11 at C19 with C20 lifted (M11), then refit C20. Not on every flight unit; 0402 rework next to
     D6 carries its own risk.
  - Dropped from the first pass: per-unit CT (not available to the owner) and a sacrificial assembled unit for sectioning
    (the coupon with a dummy block shows the same joint).
- **Prior:** none (follows from K1).

### M6. Interference sources next to the unshielded feed node — note

- **Where:** feed pad (F.Cu Ø2.2) at (53.19, 40.82); U1 body edge at y 39.48–39.51 (courtyard about 39.71); J3 at
  (53.01, 50.853), top entry on F.
- **Evidence:** the feed pad edge (y 39.72) is 0.21–0.24 mm from the module body and the feed center 1.31–1.34 mm. The node
  precedes about 27.5 dB (L1) and 33 dB (L5) of gain. PX-DS rev 2 p3: "Active antenna is required"; it gives no emission
  data, and the module sits under a shield can (p1 photo). The feed via has a full GND ring on In1–In4 and B.Cu from r 1.5
  to 2.5 mm. J3's signal pads are 7.83 mm from the feed center; the nearest RF copper to a J3 signal pad is 3.76 mm (C19.1);
  VSYS is unfiltered ahead of FL1. The J3 UART nets on In2 pass 3.17–3.73 mm from the feed axis, outside that ring; In2 GND
  covers only 56 % of the J3 pad-row footprint. 40 MHz harmonics miss L1 and L5/E5a (nearest 1560/1600 and 1160/1200 MHz)
  and hit B1I and G1 (M4).
- **Consequence:** not computable from the files; mitigated by the shield can, the GND ring and the short feed.
- **Proposal:** a bench test. Lift C26 and tap C26 pad 1 (Net-(C26-Pad1), the front-end output on the L8 side; pad 2 is
  RF_IN with U1's 3.3 V bias), shield to adjacent GND, with a DC block anyway. Output noise is about −143 dBm/Hz at L1
  (estimate), so use the analyzer preamp (DANL below −150 dBm/Hz) and 100 Hz–1 kHz RBW. Sweep 1559–1610 and 1164–1189 MHz:
  U1 running; U1 in reset (short R2 pad 2 to GND); the host connected and running against a clean bench supply on
  J3.3/J3.4 with UART traffic. Decisive metric: with C26 refitted, L1/L5 C/N0 with the host idle and active. No layout
  change. Tap and quiet-state details: M12, M16.
- **Prior:** Space Bug M1 (analogous proximity risk).

### M7. Conducted injection above about −23 dBm (L1) or −26 dBm (L5) at the patch port can overdrive RF_IN — note

- **Where:** C19 pads / AE1 feed → … → U4 → C26 → U1.11 RF_IN.
- **Evidence:** port-to-RF_IN gain 27.7 dB at L1 and 32.6 dB at L5, typical (calc). U4 output P1dB about +7.7 dBm (L1) and
  +4.6 dBm (L5) (AN596 Rev 1.3 Table 1, Option B, 1.8 V). RF_IN abs max +5 dBm (PX-DS rev 2 p6). At L1, −22.7 dBm at the
  port reaches it (about −24.7 dBm with the LNAs' +1 dB gain spread). At L5 the limit is above U4's output P1dB, so it takes
  about −26 dBm with U4 compressing, and only if U4 can deliver +5 dBm at all (Psat at 1.8 V is unspecified). Nothing else
  is overstressed at 0 dBm (BGA855N6 input +25 dBm, B8389 18 dBm). In flight the port sees about −125 to −130 dBm; host
  transmitters are out of band and cut by the high-pass and SAWs; a re-radiator delivers open-sky levels.
- **Consequence:** only a conducted measurement with the receiver connected and the source at −20 dBm or above, for example
  a VNA left at 0 dBm during an S11 sweep at C19 with C20 fitted.
- **Proposal:** for any in-band conducted measurement through the chain, keep the source at −40 dBm or less at the port (at
  least 12 dB below U1's limit, and U4 at least 13 dB below its L5 input P1dB), whether or not U1 is connected. Patch S11
  with C20 lifted (M11) makes the level irrelevant. For chain S21, lift C26 and take the output from pad 1 (L8 side, no
  DC); never probe the U1 side without a DC block. Put it in the first-article list beside M1.
- **Prior:** Space Bug M2 (its −40 dBm rule).

### M8. On paper the PX1105R's minimum TXD high is below the P4's VIH — note

- **Where:** U1.2 TXD → R8 → J3.2 → Mantis P4 GPIO3.
- **Evidence:** PX-DS rev 2 p7: VOH ≥ 2.4 V, with no load condition. ESP32-P4 DS v1.2 Table 5-4: VIH ≥ 0.75 × VDD. GPIO3
  is in the VDD_LP/VDD_BAT domain on V_MCU_SWTCH, fed from a TPS62152 with DEF tied high (+5 %, SLVSAL5E): 3.465 V nominal,
  a recorded Mantis decision. VIH is therefore about 2.60 V nominal (2.55–2.67 V across the rail tolerance). The paper
  shortfall is about 155 mV with the 46 mV ground credit across FL1 (taken at its maximum DCR) and up to about 270 mV
  without it. The other directions keep margin at 3.465 V. TXD sees R8, a 50 nA input and the 45 kΩ firmware pull-up; U1's
  VCC is 3.22–3.36 V.
- **Consequence:** probably none: a lightly loaded push-pull output sits near VCC, at least 0.55 V above the worst-case VIH.
  Compliance is not guaranteed on paper.
- **Proposal:** scope the TXD high at Mantis J1.2/GPIO3 against Mantis GND (not carrier GND), with NMEA streaming at
  115200 and the pull-up on, and record V_MCU_SWTCH in the same session (expect 3.44–3.47 V). Pass at ≥ 0.75 ×
  V_MCU_SWTCH with margin; I suggest ≥ 2.9 V and expect 3.2–3.3 V. No board change.
- **Prior:** Space Bug "Other first-article checks".

### M9. Eleven two-terminal parts pair a solid via-in-pad GND land with a thin track (tombstone and skew watch list) — note

- **Where:** C7, D3, D1, L1, C23, D6, L6, L5, L2, C28, C25 (the 11 most asymmetric; C19 is DNP).
- **Evidence:** every zone uses `connect_pads thru_hole_only`, so SMD pads join their pours solid; all four inner layers and
  B.Cu are GND. Perimeter copper: the GND pad is 79–100 % surrounded by pour, the other pad 4–18 %. Example L1: pad 1 has
  no pour and one 0.16 mm track; pad 2 has 1.625 mm² of pour, a via in the pad and another 0.483 mm away. D1/D3 use KiCad
  LED_0402 lands (0.59 × 0.64 mm, paste 0.358 mm²) against XL-1005UGC p7's 0.3 × 0.55 mm (0.165 mm²): about 2.2× the
  paste. Vias are filled and capped. Fleet: the same zone convention; Mantis made pad-level thermal-relief exceptions for
  three 0402s (layout-review F6) and kept its RF shunt solid. Beetle's first-article CC fault lists lifted pull-downs among
  its hypotheses (unresolved).
- **Consequence:** the non-RF candidates are harmless if they lift (LEDs; an open C7 gives the ADP7142's default soft start,
  about 380 µs, Rev H). A lifted C28 would leave the LP5907 below its 0.7 µF minimum output capacitance; it is the least
  asymmetric part on the list.
- **Proposal:** no change before the order. Inspect all 11 on the first article, C28, L1 and D6 first. If a non-RF part
  lifts: pad-level thermal relief (zone_connect 1) on the GND pads of D3 and C7; for D1 also move its GND via about 0.3 mm
  out of the pad; optionally trim the LED paste toward the maker's land (a named exception under SOLDER-PASTE-CONVENTION
  §2). Keep the RF shunts, D6, C23, C25 and C28 solid and rework by hand. Add "profile to the joint, prefer a longer soak"
  to Block B, as the SAM carrier has; a turnkey assembler may not honor a custom profile.
- **Prior:** none.

### M10. A signal simulator cannot reproduce the flight's common-mode TCXO transient — note

- **Where:** U1's internal 0.5 ppm TCXO (PX-DS rev 2 p3 block diagram). The effect is common to every channel and does not
  depend on the carrier layout or mounting.
- **Evidence:**
  - PX-DS gives no g-sensitivity and no shock rating; its operating limits are altitude < 80,000 m and velocity < 515 m/s
    (p2). Comparable parts: Rakon RST2016A (Issue A, p1) 2 ppb/g max; Howe et al. (NIST, IFCS 2005, p494) "of the order of
    1 × 10⁻⁹ / g".
  - At 1 / 2 ppb/g: 1.58 / 3.15 Hz per g at L1, 1.18 / 2.35 Hz per g at L5; the owner's 13.5 g bench ignition gives 21 /
    43 Hz at L1.
  - The four 2026-07-05 logs: boost peaks 5.2–9.4 g; 14 events above 25 g (27.5–146 g). The signed specific-force integral
    per event is 0.053–0.786 g·s, which is 0.08–1.24 cycles of phase at 1 ppb/g and 0.17–2.48 cycles at 2 ppb/g (L1).
  - Generic loops (2nd order Bn 15 Hz, 3rd order Bn 18 Hz; Kaplan coefficients quoted from memory): with the line of sight
    along each event's velocity change, the real dynamics alone exceed 90° at 10 of the 14 events, and the TCXO term is
    larger at 9 of them (1 ppb/g). At ignition and burnout the term adds roughly 15–40 % (1 ppb/g) to 30–80 % (2 ppb/g).
    Random-vibration phase jitter is 0.4–1.3° rms, negligible.
  - The owner's documents already state that IQ replay cannot accelerate the receiver's oscillator
    (gnss/docs/gnss-receiver-review.md §7; receiver README "Oscillator g-sensitivity").
- **Consequence:** what sets the term apart is that it hits every channel at once at deployment, snatch and landing:
  carrier slips or a reacquisition of about 1 s, and RTK reconvergence under 10 s (PX-DS p2). Not a carrier defect; no
  mount can remove a quasi-static acceleration.
- **Proposal:**
  1. **Measure Γ on the first article.** Quick ABAB tilts of ±X and ±Y with the patch at the horizon (2 g change per axis),
     and patch-up against patch-at-horizon for Z. Fit the step at each flip in the 20 Hz common-mode Doppler (0xE5) or the
     0xDF Clock Drift field (AN0039 PDF p52; 0.300 m/s per ppb). Do not average long dwells: U2's heat shifts the TCXO with
     orientation. Alternatively regress clock drift against the Mantis IMU specific force over the first boosts.
  2. **On the COCOM rig**, apply one common phasor exp(−j·2π·Γ·f_c·∫a dt) to the IQ stream at the sample rate, driven by the
     logged specific force (the 0.1 s trajectory blocks cannot carry 5–40 ms events).
  3. Orientation is not a lever: the 16.75 × 25 mm pattern allows only 0° or 180°.
  4. Keep the mount stiff (K6).
  5. **Firmware:** treat carrier phase as suspect on all channels for about 1–2 s after any event above 25 g, gated on the
     integrated specific-force change rather than the peak.
- **Prior:** SAM field-review-2026-09-01 §3.7 (random term only); the owner's receiver README (osc-g).

### M11. Patch S11 is reachable only through C19 with C20 lifted — note

- **Where:** C19 (DNP, 0402): pad 1 feed at (54.66, 44.61), pad 2 GND at (54.66, 45.57). C20 at (54.94 / 55.90, 43.68).
  D6 at (55.61, 44.85).
- **Evidence:** Net-(AE1-FEED) = AE1.1, C19.1, C20.1, D6.2, with no connector. C19.2 is fully in the F.Cu GND pour with a
  via in the pad and two more GND vias 0.45 and 0.65 mm away. D6.1's land lies entirely under the D6 body (not usable as a
  shield point); L1.2's toe is the nearest other exposed GND. The series path from C19.1 to the AE1 pin is about 4.26 mm
  (4.93 mm is the net total, including the 0.67 mm D6 branch); with C20 lifted its pad 1 land is in-line, not a stub. With
  C20 fitted, the VNA sees the patch in parallel with the high-pass and SAW input (about 50 Ω in band, B8389 §6), and the
  over-drive case of M7 applies if the LNAs are powered. On a 22 × 30 mm plane the cable shield becomes part of the
  radiator.
- **Proposal:** no board change and no test connector. Lift C20 on one unit with hot tweezers; pigtail center to C19.1,
  shield to C19.2 plus a scraped strip of GND pour west or south of it toward the GND vias at (53.91, 45.18) and
  (53.91, 44.42). Clip-on ferrites within a few centimeters; repeat with two cable routings. For a matching decision no
  de-embedding is needed (C19 is within about 1 mm of C20.1); to compare with Taoglas's curves, port-extend about 4.3 mm of
  0.16 mm line and accept that D6 and the C20.1 land stay in. Measure unpowered, on the Mantis with the harness. Refit C20
  (GRM1555C1H330JA01D).
- **Prior:** Space Bug M2.

### M12. The cascade-output tap at C26: scan with C26 fitted first, lift it only for a 50 Ω measurement — note

- **Where:** C26 at (59.55, 37.61): pad 1 Net-(C26-Pad1) at (59.55, 37.13), pad 2 /RF_IN at (59.55, 38.09). U1.12 GND land
  at (58.275, 36.66); C25.2 GND at (61.45, 36.09).
- **Evidence:**
  - C26.1 is shared only with L8.2, after U4's internally DC-blocked output (BGA855N6 Fig. 1): no DC. C26.2 is U1.11, "RF
    input with 3.3V active antenna bias voltage" (PX-DS p6).
  - Nearest exposed GND to C26.1, edge to edge: U1.12 0.47 mm, U1.10 1.24, C25.2 1.47, U4.1 1.50 (under the LNA). A solid
    F.Cu GND pour lies under the mask 0.20 mm from C26.1 over x 59.1–60.9, y 35.9–36.65, stitched by GND vias at
    (59.29, 36.34), (60.01, 36.32) and (60.69, 36.35).
  - C26's copper is 0.465 mm from U1.11/U1.12 (courtyard to U1's lands 0.32 mm).
  - An oscillating cascade could deliver about +5 to +9 dBm (estimate from AN596 Table 1) against the HackRF One's −5 dBm
    maximum input.
- **Consequence:** a 50 Ω gain result drops U1's real input load and the reverse path from the measurement.
- **Proposal:**
  1. Scan for spurs and oscillation first with C26 fitted: a near-field probe, or a resistive tap (0402 470 Ω in series with
     the coax center, about −20 dB) on the DC-free C26.1/L8.2 node.
  2. Then lift C26 on one assembled unit with hot tweezers (no hot air near U1's castellations 10–12) for the 50 Ω gain
     measurement, and refit it. A "unit A only" DNP does not work if one JLC BOM applies to every assembled board, as I
     understand it.
  3. Tap pad 1 only. Shield to scraped mask on that pour at about (59.5–60.3, 36.3–36.6), away from the capped vias; C25.2
     as a fallback; not U1.12.
  4. Put a DC block and at least 20 dB of attenuation ahead of the HackRF.
  5. State that the gain result is into 50 Ω. Optional at no cost: a 0.5 × 0.5 mm F.Mask opening on that pour patch as a
     ready-made ground land.
- **Prior:** none.

### M13. AE1 fit is the gate for patch-off tests: order the bring-up so X-ray and conducted injection come first — note

- **Where:** AE1 (B) at (53.19, 42.02), 20 × 20 mm (20.4 mm courtyard), 8.8 mm tall.
- **Evidence:**
  - Shadow by courtyard polygon: 33 parts fully inside at 20.0 mm (35 at 20.4 mm, adding C7 and L7); partly inside U1
    73.3 %, U2 81.4 %, J3 76.2 %, C136 87 %, C7 80 %, C135 60 %, C27 55 %, R8 41 %. Under the patch: 7 of U2's 9 pads
    including the EP, all of U3's and U4's, and 12 of U1's 18 (lands 1, 2, 17, 18 and part of 3 and 16 are outside).
  - Mounting: 3M 9077 over 16.5 × 16.5 mm; the pin is hand-soldered at 360–380 °C for 3–4 s (GV-DS p6, p14). The feed joint
    is on F and stays reachable; removal costs peeling and re-taping. With AE1 fitted, lifting C20 still disconnects the
    patch, and a pigtail on C20 pad 2 injects into the chain from F.
  - The Space Bug's notes fit AE1 "after reflow ... soldered from the top". This board has no fab notes, and the BOM lists
    AE1 as a fitted part.
- **Consequence:** with AE1 on, X-ray of the U2 EP, U3/U4 and most of U1 looks through the 8 mm ceramic (degraded, by
  estimate), and injection at C19 becomes ambiguous.
- **Proposal** (in D1's notes, not a standalone change):
  - AE1 fits after reflow, by hand, on the bottom (Space Bug wording; GV-DS p14). First-article units ship with AE1 loose;
    if it is ever assembler-fitted, any X-ray of U1–U4 comes first.
  - Bring-up order: reflow → X-ray (if ordered) → visual → patch-off conducted tests at C19 (then remove any C19 test part)
    → fit AE1 → S11 at C19 with C20 lifted (M11) → C19 rematch if needed → refit C20 → radiated C/N0 → U2 thermal from F
    with the patch fitted (M2, M14).
  - Fallback for later conducted tests with the patch on: lift C20 and inject on its pad 2 through a 33 pF series part.
  - The B-face IR map is optional.
- **Prior:** none.

### M14. The patch-bottom temperature cannot be measured after the fit, and U2 sits 81 % under the patch — note

- **Where:** U2 (F) at (44.88, 46.65): EP 2.41 × 3.05 mm with 6 vias. AE1 body x 43.19–63.19.
- **Evidence:** U2's courtyard is 81.4 % inside the patch body (85.1 % inside its 20.4 mm courtyard) and the whole EP is under
  the body. The tape square starts at x 44.94, so 48 % of the EP and 2 of its 6 vias are under the tape; the other 4 are
  under the untaped silver border. GV-DS rev B p5: −40 to 85 °C; the datasheet gives no tape temperature rating. The B face
  is all mask (vias tented), with the B.Cu tracks of L2 under the patch. Through-board drop at U2 is a few K (estimate: six
  vias about 40 K/W for about 0.1 W).
- **Consequence:** the patch and tape temperature can only be bounded on a flight-like unit.
- **Proposal:** in the M2 soak, a fine thermocouple on U2's GND pin-4 copper or the pour at the EP edge (x ≈ 43.4,
  y ≈ 46.6), leads out on F. Treat it as a conservative upper bound on the patch bottom and tape: pass at ≤ 85 °C with margin
  at M2's worst-case ambient. Only if that bound comes within about 10 K of 85 °C, put a 0.08 mm thermocouple under the tape
  edge (x ≈ 45, y ≈ 46.6) on a sacrificial unit. IR mapping only in open air, before the fit, to relate the B-face hot spot
  to the F-side thermocouple. No side-wall thermocouple: it reads low and its lead sits at the radiating edge.
- **Prior:** Space Bug M3.

### M15. U1 castellation wetting: the under-body half of every land is hidden; inspect the fillets and profile to the joint — note

- **Where:** U1 at (53.485, 34.43), rot 180; GND lands 1 (48.575, 30.06), 10 (58.275, 38.86) and 12 (58.275, 36.66),
  1.0 × 0.7 mm. C26 at (59.55, 37.61).
- **Evidence:**
  - All zones are THT_THERMAL (pcbnew pad connection 3), so U1's SMD lands connect solid. U1.1/10/12 have 100 % of their
    ring in GND pour, a capped via in the pad and 2/5/4 more GND vias within 1 mm; GND is filled on all six layers. PX-DS
    p13 asks for exactly that via.
  - The module edge lies on the land centerline (silk and STEP): every land is 0.50 mm outside the body and 0.50 mm under
    it.
  - C26's body is 1.025 mm from the castellation face. Only land 11 (RF_IN, a signal land with no heat-sinking) is squarely
    behind it; GND land 12 clears C26 by 0.10 mm in y and land 10 by 0.40 mm, both visible straight-on at about 15° and
    above (limited by L6 and C25); land 1 is clear.
  - PX-DS p12 profile: peak 240 °C, 60–80 s above 220 °C, 25–35 s near peak, cooling ≤ 3 °C/s, "should not be exceeded";
    MSL4 (p13). The #906 cold-joint mechanism (a dense ceramic patch over LGA joints) does not apply: U1 is a 1.7 g
    castellated module and the patch goes on after reflow. GND pins 1/10/12 are common inside the module, so continuity
    cannot detect an open one.
- **Proposal:** no design change. Block B: PX-DS p12 as the ceiling; "profile to the joint, not to the board" with the
  thermocouple on U1.10 or U1.1 (the SAM carrier's wording); the long end of the soak; ask for the profile record. First
  article: inspect all 18 fillets at 30–45° (from −x for lands 1–9, from +x for 10–18; land 11 needs about 26° or more),
  before AE1 and the mounting hardware. X-ray is optional; cross-section only if a GND fillet looks marginal or C/N0 is low.
- **Prior:** none.

### M16. U1 and the LNAs cannot be switched off separately as drawn; "RSTN held low" is not a proven quiet state — note

- **Where:** +3V3 = U1.6, U1.8, U5.1, U5.3, C1, C5, C27, R2.1, R3.1, FL2.2. One 0.4 mm B.Cu chain links the two +3V3 islands
  (C1.1 via → U5.1 via, also feeding R3/D3). R2.2 (RSTN) at (48.34, 40.14); C1.2 (GND) 1.50 mm away, edge to edge.
- **Evidence:** FL2 feeds U1 and U5 together. The LNAs draw 2 × 4.8 mA typ (5.8 max) inside 105–125 mA, invisible in the
  supply current. PX-DS p6: RSTN is an "External active-low reset input to the baseband"; nothing is said about the TCXO or
  the RF section in reset. LP5907 SNVS798Q Table 4-2: EN has an internal 1 MΩ pull-down, and with EN low the output
  discharges through 230 Ω. C28.1 (+1V8) is an F-side pad at (59.39, 46.84).
- **Consequence:** a reset baseline may still contain U1's own LO and TCXO leakage, understating its emissions.
- **Proposal:**
  1. Use the unpowered board as the floor (a); then (b) battery only, no UART, RSTN held low (tack R2.2 to C1.2); then (c)
     running. U1's in-reset emissions are (b) − (a).
  2. For LNA oscillation, compare LNAs on against off with U1 powered, by lifting U5 pin 3 (EN): U1's emissions cancel and
     U4 still sees its real load. Use the RF scan, not the current; record the LNA current as the drop when EN is lifted
     (expect about 9.6 mA typ, 11.6 mA max).
  3. If U1 must be off with the LNAs on: lift U5 pin 5, feed 1.8 V into C28.1 from a metered bench supply and remove FL2.
     All on F and possible after the AE1 fit; it is weaker evidence on stability, since U1's RF_IN load changes.
  - Do not cut the B.Cu track. Optional for a future revision: an 0402 0 Ω between U5.5 and C28 to break and meter +1V8.
- **Prior:** Space Bug M1 (its P4-in-reset method).

### M17. Equipment gaps: no VNA in the listed kit, and no calibrated noise source — note

- **Evidence:** the owner's listed assets are the HackRF COCOM rig, the IQ files (ModelRockets/gnss-c8, L1/B1 only), the
  2026-09-28 PX1105R C/N0 ladder and the mosaic-G5. The attitude study's bench.md asks whether a NanoVNA exists and budgets
  "+$80 VNA" (a NanoVNA-H4, prices from memory); I found no VNA measurement record under ModelRockets. A NanoVNA-H4 runs
  fundamental only to about 300 MHz and on harmonics to its rated 1.5 GHz top, so L1 needs extended firmware with a
  degraded floor; L5 is inside its rated range. An absolute noise figure needs a known-ENR source; PX-DS gives no NF.
- **Consequence:** without a VNA, M1 can be settled only indirectly through C/N0 asymmetry. N1 stays a relative bound, and
  the L5 part of that bound needs an L5 IQ file generated first.
- **Proposal:** buy one V2-class VNA (a LiteVNA-64 or NanoVNA V2 Plus4; prices approximate and unchecked) instead of the H4.
  It covers L1/L5 here, the attitude bench's rotary-joint check, and the 2.4 GHz tunes the Mantis (layout-review-2026-09-29)
  and Tinker-Base (design-review-2026-09-28 M1) reviews ask for. Use it on one first-article unit per M11, mounted on the
  Mantis, to decide whether to fit C19 or ask Taoglas for a tune. Accept the relative N1 bound.
- **Prior:** none.

### M18. No test points, and vias are tented and capped: every probe ground is a component pad — note

- **Evidence:** no TP references (the owner removed TP1); the board sets tenting on both sides, capping and filling, and no
  via overrides it. Nearest exposed GND to U1.8 is C5.2 (2.35 mm from the pad center, 1.84 mm edge to edge); to U1.6 it is
  3.36 mm. Every rail has a capacitor pair at 0.96 mm pitch: +3V3 at C5, C1, C27; +1V8 at C28, C23, C25. Good clip grounds:
  the H1–H4 Ø3.8 GND rings (exposed on both faces) and J3's tabs 6/7. J3.4 is VSS, upstream of FL1's common-mode winding, so
  it is not a probe reference. The SAM carrier's notes (line 39) list probe pads instead of test points.
- **Proposal:** no board change. A "Test points" paragraph in the fab notes: +3V3 at U1, tip on the U1.8 castellation fillet
  and spring ground on C5.2, or across C5; +3V3 elsewhere across C1 or C27; +1V8 across C28, C23 or C25; clip ground on
  H1–H4 or J3.6/7, never J3.4; a spring ground, not the long lead, for the < 50 mVpp check (PX-DS p12).
- **Prior:** none.

### M19. First-article verification plan — note

The table is ordered by risk retired per bench hour, with the one gate at the AE1 fit (M13). **Unit A** is the RF and thermal
unit and ends flight-like; **unit B** is optional and sacrificial (M5's coupon replaces it for the feed joint); **coupons**
are spare bare boards from the same order. Order unit A with C20 fitted (it is lifted only for row 9) and AE1 not fitted by
the assembler; C26 may be ordered unfitted, which saves the row-7 lift (it is fitted for row 8).

| # | Findings | What to measure | Pass / fail (source) | Equipment | Access (side, mm) | Bench h |
|---|---|---|---|---|---|---|
| 1 | D2, assembly | 7.4 V, 200 mA limit. U2 VOUT, +3V3, +1V8; DC on C26.1, C26.2, C19.1; VSYS current in acquisition and tracking; J3.1/J3.2/J3.5 to GND | VOUT 3.24–3.36 V (ADP7142 Rev H Table 1, ±1.8 %); 1.2 V = adjustable part, ~5 V = −5.0. +1V8 1.764–1.836 V (LP5907 ±2 %). C26.1 0 V (U4 AO abs max 2.1 V); C26.2 ≈ 3.3 V (PX-DS p6); C19.1 0 V. ~125 / 105 mA; investigate > 150 mA. J3.2 ≈ 3.3 V, none of the signal pins at VSYS (F2) | PSU, DMM, JST-SH breakout | J3.3/J3.4; C135.1 (43.38, 42.20); C1.1 (46.84, 40.31); C28.1 (59.39, 46.84); C26 (59.55, 37.13 / 38.09); C19.1 (54.66, 44.61), all F | 0.25 |
| 2 | S2 | Carrier unpowered, host TX idling high: mV across R7, R9, R8 (1 mV = 1 µA); V at C1.1, C135.1, C136.1 | ≤ 0.33 mA per pin (3.3 V across SkyTraq's 10 k, PX-DS p5/p6); V(C135.1) ≤ V(C136.1) (ADP7142 Table 3) | DMM, 3.3 V USB-UART or the Mantis | R7 (46.05 / 47.07, 32.94); R9 (59.60 / 60.62, 33.30); R8 (46.03 / 47.05, 31.94) | 0.25 |
| 3 | L3, L4 | No-rework oscillation sniff: H-loop over U3, U4, L8, `hackrf_sweep` 1 MHz–6 GHz, patch absent; then 0x64/0x06 and 0x64/0x07 on J3 | No line that appears and disappears with VSYS; 0x64/0x83 status 1 if supported | HackRF, small H-loop | over U3 (61.54, 41.60), U4 (61.68, 37.78); UART on J3.1/J3.2 | 0.25 |
| 4 | S3, M9, M15 | 2-D X-ray before the AE1 fit: U2 EP, U3/U4, FL4/FL5, U1 lands; the assembler's profile record | U2 EP voids ≤ 25 % (my threshold); no bridges; solder on U1.1/10/12; peak ≤ 240 °C, 60–70 s above 220 °C (PX-DS p12 and B8389 Table 3 together) | Lab X-ray | F, under the patch; U1 lands 1, 2, 17, 18 are outside its shadow | 0.5 + lab |
| 5 | M9, M15 | 10–20× visual of every two-terminal part and all 18 U1 fillets | No tombstone or lift; castellation fillet per IPC-A-610 §8.3.4.6 (number from the owner's copy) | Stereo microscope | F; U1.10–12 from +x at ≥ 15°, U1.11 at ≥ 26° | 0.5 |
| 6 | L7 | Ripple at 6.4 and 8.4 V during a cold-start acquisition with maximum host traffic, full bandwidth and 20 MHz | +3V3 < 50 mVpp (PX-DS p12); +1V8 ≤ 5 mVpp (my threshold) | Scope, spring ground | across C5, C1, C28, C23, C25; grounds per M18 | 0.5 |
| 7 | L3, L4, M6, M12 | Before the AE1 fit: C19 with 49.9 Ω, 0 Ω and open; scan C26.1 (fitted: probe or 470 Ω tap; then lifted: pigtail, shield to scraped pour or C25.2); `hackrf_sweep` 1 MHz–6 GHz, 10 kHz RBW captures at 1575.42 and 1176.45 MHz; U1 in reset against running, host on and off | No line ≥ 10 dB above the output floor (100 kHz RBW) in any C19 state; in-band floor rise ≤ 0.5 dB; no CW inside the L1, B1I or L5 main lobes more than 10 dB above the floor (my thresholds) | HackRF, DC block, ≥ 20 dB pad | C19 (54.66, 44.61 / 45.57); C26.1 (59.55, 37.13) | 2 |
| 8 | N1 | Before the AE1 fit, C26 fitted, C19 empty: pigtail on C19 behind a fixed pad of ≥ 45 dB; inject regenerated L1 files and an L5 file into the carrier and into a bare PX1105R through the same chain (keep the rig's DC block on the bare-board leg); step the level | C/N0 tracks the level 1:1; carrier median ≥ bare − 1.0 dB at L1 and L5 (my threshold); port ≤ −40 dBm (M7) | HackRF rig, regenerated IQ files, bare PX1105R board | C19 pads 1 and 2 | 2 |
| GATE | M13 | Remove any C19 part; optional B-face IR map at 8.4 V; fit AE1 with the qualified method (M5) | — | Iron; IR camera (optional) | B | 0.5 |
| 9 | M1, K1 | S11 with C20 lifted, pigtail on C19, port extension, ferrites, carrier on the Mantis with the harness; re-route the cable and repeat; refit C20 | \|S11\| ≤ −10 dB at 1575.42 and 1176.45 MHz; dips within 5 MHz of Taoglas's (≈ 1568.5 / 1580 and ≈ 1171 / 1178 MHz, GV-DS p16–17 digitized); no dip at all = open feed | V2-class VNA (M17) | C19; C20 (54.94 / 55.90, 43.68) | 2 |
| 10 | M2, M14 | Closed-tube soak at 6.4 / 7.4 / 8.4 V: thermocouples on the U1 shield, board copper by U2.4, FL4, C136, C23.2, the patch top and the bay air; then a cold start (AN0037 0x01) on the pad_static file for TTFF | Part rise + T_amb,max ≤ 85 °C for the 85 °C parts; TJ(U2) = TB + PD × 38.8 °C/W ≤ 125 °C (TB ≤ 100 °C at 0.64 W); cold TTFF ≤ 35 s and ≤ 1.2× the unit's room value (PX-DS p2 typ 29 s; my margin) | Logger, PSU, tube, cage | U1 shield; U2.4 land (42.975, 44.095); FL4 (59.37, 44.37); C136 (45.73, 51.29) | 4 |
| 11 | M1, M3, M4, M6 | Sky: carrier on the Mantis beside a bare PX1105R and the mosaic-G5, per-satellite C/N0 against elevation at L1 and L5; coexistence and harmonic on/off blocks; in the cage, GSV only at 1 Hz against raw at 20 Hz | Δ = [L5 − L1]carrier − [L5 − L1]reference ≥ −1.5 dB above 30° elevation (my threshold); host traffic costs ≤ 0.5 dB median | Sky loggers, mosaic-G5, cage | J3 UART | 3 |
| 12 | K1, M5, M15 | Destructive: section the coupons through the feed joint; section unit B through U1.10–12 and the U2 EP only if a fillet or X-ray looks marginal | Feed: M5's acceptance; continuous intermetallic on U1's GND lands | Lab, or saw and polish | — | 1 + lab |

Notes to the table:
- The IQ files the plan uses were moved to the Trash on 2026-10-07 (gnss-c8 README); regenerate them from their recipes
  (`c8_manifest.json`, `signalsim/make_l5_config.py`, `lc86_bench_run.py`). The warm-start L5 file in the manifest is
  `signalsim_static_l5w_2026_45.C8`.
- Row 8: raise the HackRF TX gain by the extra pad (45 − 10 = 35 dB), or treat the 09-28 dB-Hz scale as void and rely on the
  relative carrier-against-bare comparison. Step levels on `pad_static.C8`, not the 150 s `cal_g8`. `make_level_steps.py`
  hard-codes 2.6 Msps; add a rate option before using it on the 18.48 Msps L5 file.
- Row 4: 60–70 s above 220 °C satisfies both PX-DS p12 (60–80 s) and B8389 Table 3 (30–70 s).
- **Prior:** Space Bug M1, M2, M3 (the same test families).

---

## 4. Mechanical

### K1. AE1's feed pin stops about 0.42 mm below the F.Cu surface: the RF feed is a blind barrel fill — major

- **Where:** AE1 pad 1 (Net-(AE1-FEED)) at (53.19, 40.82): plated Ø1.00 mm hole, Ø2.2 pad on F.Cu (the solder side), Ø1.2
  on B.Cu and In1–In4. U1's body edge at y 39.48 (silk 39.51); the F pad edge at y 39.72.
- **Evidence:**
  - **Pin.** GV-DS rev B p6: the pin protrudes 1.2 ± 0.2 mm below the ceramic and 1.06 mm beyond the outer face of the
    release liner; the tape stack is 0.05 mm adhesive plus a 0.09 mm liner (3M 9077 TDS). With the liner peeled, the pin
    stands 1.15 ± 0.2 mm proud of the adhesive face. Pin Ø0.8 ± 0.1 mm.
  - **Board.** 1.5994 mm, which already includes both 0.0305 mm masks (copper to copper 1.5384 mm); from the B-mask face to
    the F.Cu surface is 1.569 mm.
  - **Tip depth.** 0.42 mm below the F.Cu surface nominally, 0.22–0.62 mm over the pin tolerance, 0.06–0.78 mm with JLC's
    ±10 % thickness (quoted from memory). The pin never protrudes; about 1.12 mm of it sits inside the 1.54 mm barrel. My
    `kicad-cli` STEP export agrees (the model's pin top is 0.39 mm short of the F pad top before the tape is added). The
    first pass's 0.45–0.57 mm double-counted the mask and the liner.
  - **Assembly data.** GV-DS p14: hand soldering at 360–380 °C for 3–4 s, no reflow profile; GV-DS gives no board
    thickness. The repo's model scripts use a 1.2 mm pin for the GVLB208.A and 2.4 mm for the GVLB258.A, which protruded
    through the Space Bug's 1.63 mm board; that board's notes say "pin goes through the 1.00 mm plated hole and is soldered
    from the top". Nothing records this step for this board, and AE1 is in the BOM as a fitted through-hole part.
  - **Access.** U1's body is 0.24 mm from the F pad (its courtyard overlaps the pad by 0.015 mm, cross-side and cosmetic);
    U1's nearest castellations (pins 9 and 10) are 4.4–5.5 mm away. The south and south-west approach is open: the feed
    trace is under mask, C20's courtyard is about 2.7 mm and D6's about 3.9 mm away.
  - **Continuity.** GV-DS p6's top view shows the feed as a bump on the top electrode ("0.6Max"; inferred), so a DC check
    through the pin is possible after mounting (M5).
  - IPC-A-610 (general knowledge, not a repo file): a lead end not discernible in the solder is a reject, with an
    exception only for boards thicker than 2.3 mm.
- **Consequence:** the only RF path into the receiver becomes a joint whose pin wetting cannot be seen. A non-wetted tip
  sealed in solder can pass a bench C/N0 check and open under flight vibration, losing GNSS. It is also a quiet regression
  from the Space Bug, so copying that board's procedure would mislead. The skeptic added a failure mode: with a recessed
  tip, solder fed from F can run down the 0.05–0.15 mm annulus to the exposed Ø1.2 B.Cu pad under the tape, where the
  patch's electrode sits 0.05 mm above. The iron near U1's can is a handling point (use Kapton), not a 240 °C overheating
  risk: PX-DS p12's 240 °C is a reflow-profile peak.
- **Proposal (no copper change):**
  1. **Before the order:** write Block B for this board (D1): AE1 hand-fitted after reflow, excluded from the JLC BOM and
     CPL (or DNP for assembly); its pin stops about 0.4 mm (0.06–0.78) below F and is soldered by filling the barrel from F;
     the AE1 pad stays paste-free (it is). Order spare bare boards with the assembled ones (for example 5 PCBs, 2
     assembled). Optionally ask Taoglas what board thickness the 1.2 mm pin assumes and whether a longer-pin build exists;
     this does not block the order.
  2. **Assembly procedure:** measure the pin diameter and length on a real part and the finished hole with a pin gauge;
     dry-fit with the liner off and gauge the recess from F; flux the barrel (do not pre-tin it, because a solder plug
     blocks the pin; tin the pin tip only if it still fits); press the patch onto clean B mask; Kapton over U1's south edge;
     a fine conical tip from S or SW, 0.3–0.5 mm wire, until the barrel fills and the Ø2.2 pad wets flush or concave; 3–4 s
     at 360–380 °C per attempt with cooling between attempts; do not overfeed.
  3. **Qualification and per-unit checks:** M5 (coupon section, DMM through the pin, C/N0 under tapping, S11 on the first
     article).
  4. **Staking** per K3, after the joint has passed.
  - Dropped from the first pass: moving U1 north (a reroute of a finished board for an access problem the south approach
    does not have) and a thinner board (it changes the 50 Ω stack, and the pin would still only be about flush).
- **Severity:** the verifier rated it minor (mitigable by procedure, nothing to change in copper); the impact check and the
  skeptic rated it major (the only RF path, a recognized workmanship nonconformance, and an unrecorded regression). I keep
  it at major as a risk to close before the order with documentation.
- **Prior:** Space Bug D1 (its Block B: "pin hand-soldered from the top after reflow").

### K2. Mounting-hardware margins are tight on both faces and recorded nowhere — minor

- **Where:** H1 (44.60, 29.49), H2 (61.35, 29.49), H3 (44.60, 54.49), H4 (61.35, 54.49): plated 2.2 mm, Ø3.8 GND pads on all
  layers, mask open on both faces. AE1's body; Mantis H2 (75.22, 130.50), the only Mantis carrier hole on GND.
- **Evidence:**
  - **Patch side.** Hole center to the nominal body: 2.53 mm (H1/H2), 2.47 mm (H3/H4); to the courtyard 2.335 / 2.275 mm.
    The patch is located only by its pin; one degree of rotation moves a corner 0.23–0.26 mm and the edge point nearest a
    hole 0.14–0.15 mm. Clearance from fastener to ceramic, nominal / worst at 1° / worst at 2°:

    | Fastener | Clearance (mm) |
    |---|---|
    | Ø3.8 head | +0.57 / +0.17 / +0.01 |
    | Ø4.0 pan head | +0.47 / +0.07 / −0.09 |
    | 3.5 mm AF hex | +0.45 / +0.05 / −0.11 |
    | Real M2 nut (corners ≈ 4.38 mm) | +0.28 / −0.12 / −0.28 |
    | 4.0 mm AF hex | +0.16 / −0.24 / −0.40 |
    | Ø5 washer | −0.03 at nominal (H3/H4) |

    Adding the feed pin's ±0.2 mm position tolerance (GV-DS p6) makes the Ø3.8 head about −0.03 mm at 1° in a full linear
    worst case, which is pessimistic.
  - **F side** (non-GND copper from the hole center): H3, the Net-(U2-EN) pack-voltage pour at 2.075 mm; H1, R8.2
    (Net-(J3-Pad2), an exposed pad) at 2.475 mm; H2, U1.18 at 2.584 mm; H4, the +3V3 pour at 2.669 mm. GND pads C27.2
    (2.377 mm from H4) and C136.2 (2.475 mm from H3). A Ø5 washer lands on R8.2 at H1 (a UART-to-GND short with no mask in
    between) and overlaps C27.2 by 0.12 mm and C136.2 by 0.03 mm. A 4.0 mm AF hex corner (2.309 mm) leaves 0.068 mm to C27.2.
  - **Bonding.** All four holes are on GND, as on the SAM carrier (its prefab review 08-04 B4). Mantis FAB B11: heads ≤ Ø3.8,
    round standoffs ≤ Ø4.0 or 3.5 mm AF hex, no metal washers, nylon standoffs at H2/H4/H6/H8 on the bottom; it still lets
    H2 take a metal standoff over an insulating washer, which keeps a GND bond. A metal bond would carry roughly 85–98 % of
    the 125 mA return around FL1 (estimate: 0.25–0.38 Ω through FL1 and the cable against 0.005–0.05 Ω through a brass
    standoff). The Space Bug made its holes netless for that reason (its N5). This board has no FABRICATION-NOTES.md.
- **Consequence:** a wrong fastener or a visibly rotated patch: a chipped ceramic under vibration; a bypassed FL1 return;
  a pack-voltage short after mask wear, but only if B11 is broken with a 4.0 mm AF hex or Ø4.15 round metal standoff at H3.
- **Proposal:** a short hardware block in a new FABRICATION-NOTES.md, citing Mantis B11:
  1. **Patch side (B):** M2 heads ≤ Ø3.8 (ISO 4762 socket head or ISO 1207 cheese head), or nylon screws; no washers and no
     nuts on the patch side. Fasteners "from the F side" do not help: the carrier is the outer board, so any through
     fastener ends in a head or a nut on the patch side.
  2. **F side (toward the host):** nylon standoffs ≤ Ø4.0 round or 3.5 mm AF hex, no washers, stating why (U2's VIN pour is
     2.075 mm from H3; R8.2 is 2.475 mm from H1).
  3. **Bonding intent, one line:** the holes are GND for plane area; the carrier reaches host ground only through J3 and
     FL1, so no metal standoff at the hole that mates Mantis H2, despite B11's exception. If the owner prefers the bond (it
     also protects against the reversed-cable fault in cables.md), say so and accept that FL1's return winding is bypassed.
  4. **Patch placement:** within about 0.2 mm per edge of the B.SilkS outline (about 0.6°); a jig is optional. With Ø3.8
     heads the worst case still clears by 0.17 mm at 1°.
- **Prior:** Space Bug N5; SAM prefab-review-2026-08-04 B4.

### K3. The 16 g patch is held by 3M 9077 tape and the blind feed joint; the logs show 118–146 g shocks — minor

- **Where:** AE1 (B), centered at (53.19, 42.02); the 16.5 × 16.5 mm tape square spans x 44.94–61.44, y 33.77–50.27.
- **Evidence:**
  - GV-DS rev B p5: "Weight 16g" (the p13 packaging gives 20 g gross); p6: 3M 9077 over 16.5 × 16.5 mm; no shock or vibration
    rating anywhere.
  - 3M 9077 TDS (Nov 2006): a 0.05 mm double-coated non-woven acrylic on a 0.09 mm liner, "Ideal for Flexible Printed
    Circuit (FPC) attachments"; 90° peel 11 N/cm on glass-epoxy (9.1 after reflow), measured with a flexible pull tab;
    p3 asks for clean surfaces, firm pressure and application at 21–38 °C.
  - Flight logs (ISM6HG256 high-g channel, about 960 Hz, no railed samples): boost mean 7.4 g, peak 9.4 g; ejection 39, 44,
    45 and 118 g axial; landing 54–77 g and 146 g (140 g lateral). The first pass's "deployment shocks are unmeasured" was
    wrong.
  - Edge-peel heuristic (CG about 3.9 mm up, uniform density): 2.3–2.5 N/cm at 50 g; 5.3–7.4 N/cm at 118–146 g, against
    11 N/cm: about 1.5–2× margin. This is an order-of-magnitude comparison, not a rating on solder mask or ceramic.
  - Under the tape, B.Cu is 95.7 % copper; the tracks are flush with the GND pour, and the only relief is the 0.127 mm
    clearance grooves (1.8 % of the tape area). Vias are filled and capped.
  - The SAM field review lists "a cracked or delaminated patch" as a failure mode.
- **Consequence:** a patch that creeps or peels loads the hidden feed joint (K1); a cracked joint loses GNSS, the board's
  only job.
- **Proposal:** make staking mandatory in the fab and assembly notes, in Mantis B3's words: non-corrosive alkoxy-cure RTV
  silicone to MIL-A-46146, no acetoxy and no epoxy.
  - Small, low (about 1.5 mm or less up the lower ceramic) and symmetric fillets. The patch corners are only about
    0.9–1.2 mm from the H1–H4 lands and its x-edges about 1.0 mm from the board edges: stake the corners only if the RTV
    stays off the lands and inside the outline; otherwise put fillets at mid-span of the two y-edges (y ≈ 32.0 and 52.0,
    x ≈ 47–59).
  - Tape step per the TDS: IPA-clean the B mask, apply at 21–38 °C with firm pressure, and let the bond dwell before
    soldering the feed.
  - Compare C/N0 (or S11) before and after staking; the L5 band is only about 13 MHz wide.
  - Qualify one built unit at the fleet's measured levels, not 50 g: at least 120–150 g, millisecond-scale, axial and
    lateral (a logged drop test or an ejection-charge ground test). Then inspect the tape edge and the feed joint and
    re-check C/N0.
  - The B.Cu tracks under the tape are not a mechanical reason to reroute; L2's RF case stands on its own.
- **Prior:** Space Bug review §8 (vibration with the 18 g patch not covered).

### K4. The stack geometry is undefined, and neither side of the matching Mantis hole set takes the carrier cleanly — minor

- **Where:** Mantis H2 (75.22, 130.50), H4 (91.97, 130.50), H6 (75.22, 155.50), H8 (91.97, 155.50); C130 (87.50, 147.03, B),
  S1 (89.72, 137.71, B), J8 (B), U15 (78.95, 145.07, F), U22 (73.19, 136.93, F).
- **Evidence:**
  - **Hole sets.** The carrier's pattern is 16.75 × 25.00 mm. On the Mantis HEAD (`95b14623`), H2/H4/H6/H8 are 16.75 ×
    25.00 mm, the only match; H1/H3/H5/H7 are 16.75 × 22.50 mm (as on the SAM carrier and the LoRa board); the Beetle's are
    17.72 × 48.64 mm (F6). The Mantis V10 as ordered (tag `tinker-mantis-v10.0.0`, `86aa08de`) is the pre-stretch file:
    its H6/H8 sit at y 153.00, so its pattern is 16.75 × 22.50 mm and the carrier fits only the stretched V11 now in work.
    That settles the reviewers' disagreement: one of them had read the V10 tag.
  - Mantis FAB B11 puts nylon standoffs at H2/H4/H6/H8 on the bottom.
  - **B side under the carrier:** C130 (an 8 mm supercap can, 12 mm tall), S1 (the USB SEL0 slide switch, 5.6 mm), and J8,
    the JST B2P-VH battery header, which runs about 2.9 mm under the carrier's outline (pin 2 is 0.93 mm inside); its mated
    height is 16.5 mm (JST VH catalog).
  - **F side under the carrier:** U15 (the S3) inside the rectangle; U22 (its 2.4 GHz loop chip antenna) 1.19 mm outside
    it but under the carrier's edge strip. The tallest F-side part under it is L11 at about 2 mm.
  - **Pointing.** J3 is top-entry on F, so F faces the host and the patch faces outward. A stack parallel to a Mantis lying
    along the airframe looks radially, about 90° off zenith on the pad; GV-DS p21's pattern is about 2–3 dB down at 90°,
    and the far half of the sky falls behind the stack. The SAM V9 installation had the patch "looking straight up"
    (field-review-2026-09-01 line 15); that review's C8 asked to record the mount.
  - **Thickness (estimate):** on the B side, J8's mated plug and wire bend need a gap of at least about 17 mm, so the stack
    is about 29–31 mm thick and 37–38 mm across the diagonal. No stack model exists (both tinker-mantis STEP files are the
    same V9 assembly, flight computer only).
- **Consequence:** the carrier circuit is unaffected and nothing on it has to change. But the mount is the largest GNSS
  performance lever (the SAM field review traced its 27-against-about-10 satellite gap to the installation), the B side
  needs standoffs of 17 mm or more or is ruled out, the F side lays the carrier over the S3 and its antenna (M3, M4), and
  V10 hosts do not match at all.
- **Proposal:** decide and model the mount before the Mantis V11 order: which side, the standoff height, or nose-mounted and
  cable-connected.
  - Include J8 with its mated plug and wire exit in the clearance check; on the B side it, not C130, sets the gap.
  - For an F-side stack, check whether the carrier's ground a few mm away detunes U22.
  - State that stacking needs the V11 Mantis; a V10 installation is cable-connected (the 51 mm A05SR05SR30K51A jumper) or
    uses an adapter.
  - Record the patch direction and the mount (SAM field review C8).
  - Do not move the carrier's holes or change its outline; both are owner decisions.
- **Prior:** SAM field-review-2026-09-01 C8.

### K5. J3 is friction-retained, with no staking or strain-relief rule — minor

- **Where:** J3 BM05B-SRSS-TB on F at (53.01, 50.853): signal pads 1–5 at y 49.425, tabs 6/7 at (49.71 / 56.31, 51.95);
  top entry. The A05SR05SR30K51A jumper to the Mantis J1.
- **Evidence:**
  - cables.md (pin mapping and the reversed-jumper trap) and the SAM carrier's fab notes have no staking or tie-down line,
    and this board has no fab notes. The Mantis prefab review (2026-08-05) already carries a LOW item, "All peripheral
    connectors are friction-retention only" (WORKLIST:245). The fleet's only staking rules are for C130 (Mantis B3,
    Beetle B8).
  - JST's SH catalog names only a "Housing lance system" and gives no lock and no withdrawal force; I found no published
    figure.
  - Flight logs: ejection main pulses stay above 25 g for 8–12 ms (39–45 g and 118 g, mostly along the thrust axis and, with
    the patch facing up, in J3's withdrawal direction); landings 54–146 g.
  - The one carrier flown on this link (V9, 2026-08-29) saw 66 g, 37.5 g, a 22.7 g landing and a 94 g boost burst with no
    GNSS gap (1378 records, none over 0.5 s apart). On the same flight the V8 main computer lost power at T+0.44 s, which
    its recovery notes attribute to a connector or harness letting go under boost vibration (an 800 Hz tone with ±30 g
    peaks; "Strain-relieve and tie down the battery connector regardless").
  - Inertial pull (estimate): plug plus half the jumper is about 0.2–0.3 g, so 0.3–0.45 N at 146 g; with the cable
    unsupported, 0.4–0.65 N.
- **Consequence:** a de-mate or chatter cuts VSYS and the UART. With V_BCKP on VCC (F1) every dropout longer than about
  150 µs (C136's ride-through, estimate) is a 29 s cold start in factory defaults, losing the descent and the landing
  position.
- **Proposal:** for flight builds only (an airframe-build line in cables.md and this board's Block B):
  - A small, removable RTV fillet (neutral or alkoxy cure, never acetoxy or epoxy, per Mantis B3 and Beetle B8) bridging the
    plug housing to the header body on one long side. Cut it before unplugging; prying a staked plug can lift the 1.0 mm
    pads. Kapton across the mated plug, pressed down on both sides, is an alternative.
  - Strain relief as a principle: both jumper ends on one rigid mass (carrier and Mantis on one sled), one tie near the
    carrier end, and enough slack that the tie never preloads the plug toward withdrawal.
  - Bench: force-gauge the straight-pull withdrawal force of a mated pair, new and after about 10 cycles, against
    0.3–0.65 N. Check after each flight that the bead is intact and the plug fully seated.
  - Changing connector family is not proposed; it would change the host, the cable and the fleet convention.
- **Prior:** Mantis prefab-review-2026-08-05 (LOW, friction retention) and WORKLIST:245; the V8 recovery notes of 2026-08-29.

### K6. The first mode is about 1.5–4.7 kHz with the patch bonded; keep the stack short, stiff and insulating, never on isolators — note

- **Where:** H1–H4; AE1 (16 g) centered within 0.217 mm of the hole-pattern center.
- **Evidence:**
  - Mass about 20.6 g (the patch is 78 %); laminate bending stiffness about 10 N·m (9.0–11.1).
  - With the patch bonded: f1 about 1.5–4.7 kHz (Mindlin FE 2.13–3.34 kHz pinned on the hole rings, 2.7–4.7 kHz clamped;
    my Kirchhoff FE 1.5–7 kHz). Treating the patch as an unbonded point mass gives a 902 Hz bounce mode and lower rocking
    modes (698–767 Hz and still falling with mesh refinement): a singular idealization with no valid lower bound, since the
    patch is bonded over 16.5 mm.
  - **Input.** The July example logs sample at about 963 Hz (interpolation suppresses their top band; raw-sample estimates
    reach 4.8e-4 g²/Hz near 422 Hz). The owner's archive has faster data: the 2026-08-29 BARC L2 flights logged about
    3.8 kHz in boost (to about 1.9 kHz) and 2026-03-08 logged 1938 Hz. On `flight_20260829_152400`: axial 0.10 g²/Hz at
    855 Hz and 1.9e-2 at 907 Hz; at most 4.7e-4 g²/Hz above 1.3 kHz; boost 0.45–1.15 g rms over 5–470 Hz. Shocks in the
    archive reach 210–276 g on five flights and 338.5 g (railed) once.
  - **Margins** (Steinberg's formulae as commonly quoted, conservative constants; I could not open the book): allowable
    deflection U1 74 µm, C1/C135 166 µm, C136 122 µm, J3 116 µm. With f1 at 1.5–2 kHz or above and the measured inputs,
    Z/Z_allow stays at or below about 0.4, and board strain (bonded, at most 0.9 µε/g at the components) stays well under
    MLCC flex levels.
  - **Standoffs** (beam estimate, 3.5 mm AF hex nylon, fixed-guided, no rocking): 5 mm tall sways at 2.6–3.4 kHz, 10 mm at
    0.9–1.2 kHz, 15 mm at 0.5–0.65 kHz.
- **Consequence:** no fatigue or overstress at known levels with a short, stiff mount. A tall plain-nylon stack or soft
  isolators would put a system mode on the L2 flight's 0.85–0.91 kHz axial peaks or into the boost band.
- **Proposal:** when K4 defines the stack: insulating standoffs at those holes within Mantis B11's limits (≤ 3.5 mm AF hex or
  Ø4.0 round, no metal washers), as short as J3's mated plug and K4's clearances allow. Above about 6 mm use a stiffer
  insulator (glass-filled nylon, PEEK, G10/FR-4) rather than plain nylon 6/6. No grommets or foam. The first pass proposed
  metal standoffs, which conflict with B11 (pack, pyro and VBATT copper starts 0.10–0.13 mm outside those Mantis pads).
  Run the PSD on the existing 4k Dynamic L2 logs first (raw samples or Lomb-Scargle, not interpolation), and fly "8k
  Dynamic" (7680 Hz, RocketComputerTypes.h ~L2885–2895) only to cover 1.9–3.8 kHz. An optional tap test finds f1.
- **Prior:** Space Bug review §8 (not covered there).

---

## 5. Release package

### D1. No FABRICATION-NOTES.md — major (for the order)

- **Where:** `hardware/gnss-px1105r-18mm-hv/`. `git ls-files` lists only fp-lib-table, `gerbers/.gitkeep`, `.kicad_dru`,
  `.kicad_pcb`, `.kicad_pro`, `.kicad_sch` and sym-lib-table.
- **Evidence:**
  - **Convention.** `hardware/README.md` lines 14–17: every board's FABRICATION-NOTES.md points to
    SOLDER-PASTE-CONVENTION.md. The SAM carrier, the Space Bug and seven other boards have one. A fresh `.gbrjob` carries the
    layer count, thickness, ENIG, rev V1, the stackup and the rules, but no via fill or cap field. The board setup declares
    filling and capping (`.kicad_pcb` lines 144–145), which no fab reads.
  - **Foil.** U3/U4 print 12 true Ø0.25 mm round apertures (0.0491 mm², 0.785 mm perimeter): area ratio 0.78 at 80 µm,
    0.625 at 100 µm, 0.52 at 120 µm. The convention's 0.66 floor needs a foil of 94.7 µm or less. Infineon's own BGA855N6
    Rev 2.2 Fig. 4 specifies these apertures on a 100 µm stencil, so 100 µm is marginal against the house floor, not
    against the maker. The next tightest apertures: FL4/FL5 1.04 (0.83 at 100 µm), D6 1.23 (0.99), FL1 1.58 (1.26); all
    others ≥ 1.94; U1 2.57. B.Paste is empty. The convention's table lists the Space Bug at 80 µm for the same apertures;
    this board has no row. JLC's default "Select by JLCPCB" picks from its free 0.10/0.12/0.15/0.18/0.20 mm foils; 0.08 mm
    is a paid special thickness that must be named in the order. Copying the SAM order would give 120 µm.
  - **Vias.** 146 × 0.40/0.30 mm. 43 via centers sit inside 38 SMD lands: U2's EP (6), U2.5 and U2.7, U1.1/10/12 dead
    center, J3.1/2/5/6/7, L1.2, L2.2, L5.2, L6.2, FL5.3, C19.2, D6.1, FL1.1, U5.1/2, R7, R8, R9, R3.1, D1.1 and the
    decoupling capacitors. Four more drills cut into FL4.5, FL5.5, U3.4 and U4.4. JLC: epoxy filled and capped is the free
    default on 6-layer boards. The SAM notes (line 12): "epoxy filled & capped ... never tented-only or plugged-only".
  - **Assembly limits.** PX-DS rev 2 p12: peak 240 ± 0.5 °C held 25–35 s, 60–80 s above 220 °C, preheat 150–200 °C for
    120–150 s, cooling ≤ 3 °C/s, "should not be exceeded"; p13: MSL4, 72 h floor life, bake 85 °C for 8–12 h, "ESD
    sensitive device". B8389 v2.1: MSL3 (p4), 250 V HBM / 125 V MM (§7), peak 250 +0/−5 °C with ≤ 20 s above 245 °C and
    30–70 s above 220 °C (Table 3). XL-1005UGC MSL3. The BGA855N6's MSL is not stated in Rev 2.2.
  - **Other unrecorded items:** AE1 hand-fit (K1), U2 fixed variant (D2), C19 DNP with paste (D7), panel rules (D11, L9),
    the J3 pin map (S1), mounting hardware (K2), the thermal envelope (M2).
  - **History.** The SAM carrier's #906 first article had U1 cold joints and was recovered by an unrecorded rework (method,
    bake and the MSL4 floor life all unrecorded); its field review lists that rework as one of two unmeasured candidates
    for its satellite deficit, and its §6 item 11 notes that the via option ordered is unrecorded.
- **Consequence:** ordered from the gerbers alone, JLC prints the TSNP LNAs at an area ratio of 0.625 or less (0.52 on
  120 µm). Those two series parts are the only RF path and can be inspected only by X-ray; an open AI or AO stops all
  tracking, a starved ground costs several dB. Unfilled barrels (unlikely at JLC, but nothing written prevents a different
  option or fab) would wick solder from U1, J3 and the U2 EP. A standard 245–250 °C lead-free profile exceeds U1's
  ceiling (out of spec, probably survivable). Nothing tells the assembler to bake U1, handle the 250 V SAWs, or leave AE1
  off. This matches the Space Bug's D1 rating, "major (for the order)"; it is not a blocker because the artwork is right
  and the fix is a document.
- **Proposal:** write the notes from the Space Bug V3 block (same stack, receiver and front end), not from the SAM V3
  Block B, which is u-blox-specific (120 µm, two stencils, a 245 °C peak, D1 holding SAFEBOOT_N, "pin 5 carries the 1 PPS
  output"). From the SAM notes take only the U2 fixed-variant line, D2's cathode toward VSYS and the FL1 polygon wording.
  Write it last, from the release commit, and re-measure every number.
  - **The minimum before ordering:**
    1. **Stencil:** 0.08 mm, flat, laser-cut, electropolished, top only, apertures 1:1, ordered as "Select by Customer"
       with "do not substitute 0.10/0.12" and the area-ratio table (U3/U4 0.78, or 0.625 at 0.10 mm; FL4/FL5 1.04; D6 1.23;
       FL1 1.58). Add the board's row to SOLDER-PASTE-CONVENTION §1. Do not copy the Space Bug's "U2 51 % (4 windows)"
       line: here U2 prints one full aperture (S3).
    2. **Vias:** 146 × 0.40/0.30 mm, epoxy filled and capped (IPC-4761 Type VII), never tented-only or plugged-only; 47 of
       them are in or cut into SMD lands.
    3. **Reflow and handling:** peak 235–240 °C measured at U1, 60–70 s above 220 °C (meets both PX-DS p12 and B8389
       Table 3); U1 MSL4 with the bake rule; FL4/FL5 MSL3 and 250 V HBM handling; BGA855N6 MSL "not stated".
    4. **AE1** hand-fitted after reflow per K1 and M5, excluded from assembly.
  - **Block A:** JLC06161H-3313, 1.6 mm (1.5994); copper 35 / 15.2 / 15.2 / 15.2 / 15.2 / 35 µm; 3313 0.0994 mm (εr 4.05),
    core 0.55 (4.38), 2116 0.1088 (4.16), core 0.55, 3313 0.0994. Layer use: F RF, parts and GND; In1 GND, the RF reference;
    In2 UART/EN plus GND; In3/In4 GND; B supply tracks plus the patch ground (L2). RF: 0.16 mm CPWG with a 0.20 mm gap over
    In1, 50 Ω, 14 nets, 23.5 mm, "do not adjust widths" (ask for impedance confirmation if wanted). ENIG. Outline
    22.00 × 30.00 mm with R1 corners. Plated holes 146 × 0.30, 1 × 1.00 (AE1), 4 × 2.20. Minimum drawn track 0.10 mm (rule
    0.09), spacing 0.09/0.10, RF gap 0.20, hole to hole 0.218 mm same net / 0.284 mm different nets, copper to edge 0.20 mm
    (0.199 as filled). U3/U4 single NSMD windows on purpose. Panel per D11 and L9.
  - **Block B:** one top-side reflow; U2 ADP7142ARDZ-3.3 only (D2); C19 DNP (D7); placement-preview checks (U1 pin 1 per L1,
    U2's dot, D2's cathode on pad 1 = VSYS, D1/D3 polarity); fiducials or rail fiducials (L6); CPL front SMD only, DNP
    excluded (D4); J3 pin map from the netlist after S1; hardware (K2); staking (K3, K5); thermal envelope (M2); probe
    points (M18); bring-up order (M13).
  - **First-article qualification block:** under 10× on bare boards, every in-pad via shows a flat cap (U2 EP, U1.1/10/12,
    J3); confirm 0.08 mm on the stencil frame or the order; X-ray U3/U4 (12 joints, no bridge inside the shared window, the
    pin-4 joint comparable to the others) and U2's EP voids; inspect U1's castellation fillets and J3's tab fillets; get the
    profile record at U1; a 15–45 min sky log of L1/L5 C/N0 beside a reference (losing one LNA stage costs about 15.7 dB at
    L1; an open AI/AO shows no tracking); record U1's firmware (0x02 01) and serial, and the foil, profile and any rework or
    bake actually used.
- **Prior:** Space Bug D1, D7 (and L13); SAM field-review B3 and §6 item 11; SAM prefab-review-2026-09-02 B6;
  SOLDER-PASTE-CONVENTION table.

### D2. U2's fixed-3.3 V guard is in no release file; a stray MP field and the only stocked LCSC listing name the adjustable part — major

- **Where:** U2 at (44.88, 46.65), F; symbol Custom:ADP7142ARDZ; schematic and footprint fields.
- **Evidence:**
  - **Correct fields:** U2's Value and MPN are "ADP7142ARDZ-3.3" (Value is printed on the sheet and fills every BOM's
    Designation column), and its Description says "FIXED variant only ... the adjustable ADP7142ARDZ would regulate at
    1.2 V".
  - **Stray SnapEDA fields** on the instance and the footprint: MP "ADP7142ARDZ", PRICE "2.08 USD", AVAILABILITY "Warning",
    PARTREV "D", MF, PACKAGE, STANDARD.
  - **Library symbol,** shared by this board, the SAM carrier, space-bug-m8t and the legacy Space Bug (all four U2 instances
    carry MP "ADP7142ARDZ"): Value "ADP7142ARDZ", Description "Ldo Voltage Regulator Adjustable o/p, 1.2v-39v, 0.2a,
    Nsoic-8". Instance fields persist, so editing the library alone changes no placed part.
  - **Electrical:** SENSE/ADJ (U2.3) is tied to VOUT with no divider. ADI Rev H p23 lists ADP7142ARDZ and -R7 as
    "Adjustable (1.2 V)"; Eq. 3 gives 1.2 V with ADJ on VOUT. The PX1105R needs VCC ≥ 3.0 V (p7), the LP5907 VIN ≥ 2.2 V.
  - **Sourcing (2026-10-12):** LCSC C3745280 is ADP7142ARDZ, 206 in stock, with the parametric "Output Type: Fixed" beside
    "1.2V~39V". The skeptic also found LCSC C3752812 (ADP7142ARDZ-3.3) and JLC C659365 (-3.3-R7), both with no stock. No
    fleet schematic carries LCSC numbers; the assembler picks parts.
  - **Release files:** this board has no bom.csv and no FABRICATION-NOTES.md. The SAM carrier closed the same hazard,
    rated a blocker (B8, prefab-review-2026-09-02), with its FABRICATION-NOTES line 31 and bom.csv item 14. The default BOM
    exports (`kicad-cli`, the project preset, the repo bom.csv format) do not export MP; an all-fields export shows MP
    "ADP7142ARDZ" beside MPN "ADP7142ARDZ-3.3".
- **Consequence:** an accepted matcher suggestion, or an all-fields BOM read by a parser, fits the adjustable part. +3V3
  then sits at about 1.2 V and every board in the batch is dead at bring-up (D3 dark), without damage; the fix is reworking
  an SOIC-8-EP stitched to six planes. A fixed ADP7142ARDZ-5.0 would destroy U1 (abs max 3.6 V).
- **Severity:** the verifier rated it minor (it needs an unusual path, and the owner knows the hazard); the impact check and
  the skeptic rated it major to minor-plus. I keep major because the guard has to exist before the order and the
  predecessor treated the same gap as a blocker.
- **Proposal:**
  1. **Required before the order:** a bom.csv row and a FABRICATION-NOTES Block B line: "U2 FIXED 3.3 V only (tube
     ADP7142ARDZ-3.3, reel ADP7142ARDZ-3.3-R7). SENSE is tied to VOUT, no divider. The adjustable ADP7142ARDZ / -R7, and
     LCSC C3745280 (the adjustable part although LCSC lists it as 'Fixed'), would regulate at 1.2 V; ADP7142ARDZ-5.0 would
     destroy U1. No LCSC stock of -3.3: consign or global-source." Put "DO NOT match C3745280" in the BOM's supplier field,
     where JLC's matcher acts.
  2. **Hygiene:** delete MP and AVAILABILITY (preferably also PRICE, PARTREV, MF, PACKAGE, STANDARD) from U2 in the
     schematic, then Update PCB.
  3. **Durability, optional:** in Custom.kicad_sym set the shared symbol's Value to ADP7142ARDZ-3.3, give it the
     fixed-variant Description and drop the SnapEDA fields (safe, since all four users fit -3.3; expect library-mismatch
     churn in the other three projects, whose instances still carry MP), or add a separate -3.3 symbol. Optionally place
     the schematic note at U2 that SAM B8 proposed.
  4. **Bring-up:** read the tube or reel label for -3.3 before reflow. At first power (7.4 V, about 250 mA limit) measure
     VOUT at C135: 3.24–3.36 V (±1.8 %). About 1.2 V means the adjustable part, about 5 V the -5.0. D3 lit is the quick
     go/no-go.
- **Prior:** SAM-carrier FABRICATION-NOTES U2 line and bom.csv item 14; SAM prefab-review-2026-09-02 B8.

### D3. BOM sourcing: four assembler lines cannot come from LCSC stock, and probably more — minor

- **Where:** U1 (PX1105R), U2 (ADP7142ARDZ-3.3), U3/U4 (BGA855N6E6327XTSA1), FL4/FL5 (B39162B8389P810); possibly L1/L2/L5/L6
  (LQW15AN5N1C00D), L3/L7 (LQW15AN9N1G00D), FL1 (DLW21SN670HQ2L) and C20/C21/C26 (GRM1555C1H330JA01D); AE1 (hand-fit);
  C1/C135/C136 (CL21A226MOQNNNE).
- **Evidence (LCSC and JLC, 2026-10-12):**
  - C5556161 (B8389, the Dist_PN on FL4/FL5) out of stock at $1.63; C534178 (BGA855N6) out of stock; JLC's library shows 0
    for both. RS stocks the BGA855N6 (about 11k, from a search snippet). I found no confirmed stock of the B8389 anywhere.
  - ADP7142ARDZ-3.3: listed but out of stock (LCSC C3752812, JLC C659365); the only stocked listing is the adjustable
    C3745280 (D2). DigiKey lists the -3.3 as active with a 10-week lead (search snippet).
  - PX1105R: JLC has a zero-stock "JLCPCB Assembly" entry (C9900164713, from earlier global sourcing); NavSpark sells it at
    $13.50.
  - The skeptic also found C76964 (GRM1555C1H330JA01D) "Not available now" and C521084 (DLW21SN670HQ2L) out of stock; the
    LCSC status of the two LQW15 inductors is unconfirmed.
  - In stock: C1975594 TPD1E0B04DPYR (1,105), C113113 LQW15AN2N4B00D (7,460), C92498 LP5907MFX-1.8 (30,515).
  - AE1 is a B-side through-hole part fitted by hand after reflow anyway (the GVLB208.A has a Mouser new-product page and a
    Westward Sales listing; LCSC carries the GVLB258.A, apparently not the 208.A).
  - CL21A226MOQNNNE: Samsung "NRND; replacement CL21A226MOYNNN#". LCSC C98190 has 547,470; the successor appears at LCSC
    only in a trivial quantity. The SAM bom.csv already says "qualify before the next reel buy". Electrically the part meets
    ADP7142 CMIN (C136 about 2.65 µF worst at 8.4 V, C135 about 7.1 µF).
  - The first pass's findchips "No results" evidence was invalid (findchips returns nothing for parts that are sold, too).
  - The Space Bug V2, with the same U1/U2/U3/U4/FL4/FL5 MPNs, was ordered from JLC on 2026-09-29 ("No BOM is included"), so
    a sourcing channel exists outside the repo.
- **Consequence:** order logistics, not a design defect. JLC's matcher shows unmatched and zero-stock lines at upload,
  before payment, so an unplanned order stalls rather than ships wrong. The one sharp edge is U2 (D2). Gate the order on
  having B8389s in hand or confirmed.
- **Proposal:** add a supplier column and a short sourcing table to the new FABRICATION-NOTES.md and bom.csv:
  - U2 per D2.
  - U1 consigned from SkyTraq/NavSpark in vacuum dry-pack; the assembler checks the humidity card and bakes 85 °C for
    8–12 h if the bag is open (PX-DS p13).
  - U3/U4 and FL4/FL5 by JLC global sourcing or consigned (Space Bug V2 leftovers first), machine-placed only; FL4/FL5 MSL3
    and 250 V HBM.
  - AE1 owner-supplied, hand-fitted, excluded from the assembler BOM.
  - On order day run the whole BOM through JLC's matcher and treat the four as a floor (expect the LQW15, DLW21SN and 33 pF
    lines too).
  - Keep CL21A226MOQNNNE for this order and carry the SAM NRND note; move to MOYNNN fleet-wide later.
  - Bring-up per D2 step 4, plus +1V8 1.76–1.84 V at C28 (LP5907 ±2 %) and about 125 mA input.
- **Prior:** SAM prefab-review-2026-09-02 B7 (NRND); Space Bug N14 (procurement).

### D4. No committed unpriced bom.csv, inconsistent fields, and a default CPL with AE1 on the bottom and H1–H4 in it — minor

- **Where:** the board folder (no bom.csv); fields on FL1, U2, D2, AE1 and D6/FL4/FL5/L4/L8/U5; the placement export.
- **Evidence:**
  - The SAM carrier and Mantis commit a bom.csv (SAM: Id; Designator; Footprint; Quantity; Designation; "Supplier and ref";
    MPN; Constraint. Mantis: MPN, Mfr, Fit).
  - Every fitted symbol has MPN and Mfr. `out/bom.csv`'s Manufacturer column is filled only for FL1, through its extra
    MANUFACTURER field. Six symbols carry Dist="LCSC" / Dist_PN: D6 C1975594, FL4/FL5 C5556161 (the right listing, out of
    stock), L4/L8 C113113, U5 C92498. No symbol has an "LCSC" field.
  - FL1 carries MANUFACTURER, MAXIMUM_PACKAGE_HEIGHT and STANDARD; U2 carries more SnapEDA residue, including PRICE
    "2.08 USD" (the fleet never commits priced BOMs) (D2).
  - D2: Mfr PTTC with the Vishay 85881 datasheet link (the same pair is on the SAM carrier and the Space Bug). Their IR
    (0.1 against 2.5 µA at 10 V) does not matter at VSYS ≤ 8.4 V.
  - AE1's Description says "BeiDou B1/B2a". GV-DS p4 marks B1I (1561 MHz) not covered (B1C and B2a are), while the PX1105R
    tracks B1I.
  - Placement: `--side both` gives 49 rows including AE1 "bottom" (through-hole), H1–H4 (only exclude_from_bom, as on
    Mantis and the SAM carrier) and C19 (DNP). `--exclude-dnp` gives 48; `--side front --exclude-dnp` 47 (H1–H4 remain);
    `--side front --smd-only --exclude-dnp` 43, exactly the 43 fitted SMD parts.
- **Consequence:** a reorder by value or MPN loses the constraints (U2, C136's 16 V floor, the NRND successor, SAW
  handling). A default CPL carries a bottom-side through-hole row and four hole rows (JLC's matcher would likely surface
  them; estimate).
- **Proposal:**
  - Commit an unpriced bom.csv in the SAM format (22 fitted lines plus a C19 DNP line) with constraints: U2 fixed only; C136
    16 V is the floor behind the ≤ 17 V SMF10A clamp; CL21A226MOQNNNE NRND → CL21A226MOYNNN; FL4/FL5 MSL3, 250 V HBM,
    global-source; AE1 B-side through-hole, hand-fit; J3 pin 5 per S1.
  - Keep C5556161. If LCSC numbers are wanted, move all six Dist pairs to one "LCSC" field, or drop all six.
  - Delete FL1's MANUFACTURER, MAXIMUM_PACKAGE_HEIGHT and STANDARD, and U2's SnapEDA fields.
  - Make D2's Mfr and Datasheet name one maker (fleet-wide).
  - AE1 Description: "BeiDou B1C/B2a (B1I 1561 MHz not specified)".
  - CPL: set exclude_from_pos_files on H1–H4 and AE1 and export with `--exclude-dnp`, or document
    `kicad-cli pcb export pos --side front --smd-only --exclude-dnp --format csv --units mm` in the fab notes. Not
    `--side front --exclude-dnp` alone, which still emits H1–H4.
- **Prior:** Space Bug D6; SAM prefab-review-2026-09-02 B7, S-Sch5.

### D5. cables.md covers the reversed-jumper hazard only for the legacy carrier; this one runs reversed and looks alive but deaf — minor

- **Where:** a reversed (B-type, pad 1 to pad 5) jumper puts carrier J3.4 (VSS) on Mantis J1.2 (GNSS_TX = U17 pad 3, P4
  GPIO3, no series resistor). `hardware/cables.md` lines 5–7 and 34–55.
- **Evidence:**
  - cables.md was last changed in `7654e58c` (2026-09-29, the Space Bug S7 fix), before this board's first commit
    (`1cf7862d`, 2026-10-11). It says "both GNSS carriers' J3", names a "light carrier (tens of mA)" and "the PX1105R
    carrier (legacy/gnss-px1105r-18mm-highpower-ext-ant) draws about 0.2 A". Four boards now plug into the Mantis J1 with VSS
    on pin 4: the SAM carrier, this one, the legacy Space Bug (J4) and the Space Bug M8T (J4.3 +BATT, J4.4 VSS).
  - Netlists: this carrier's J3.1–J3.5 map is identical to the SAM carrier's. Mantis GNSS_TX = {J1.2, U17.3}, two nodes.
    Mantis B11's nylon standoffs keep the GND holes from tying the grounds.
  - Reversed, every other return runs through 1 kΩ (at most about 3.3 mA each), so the carrier's ground rises to about
    3.3 V plus the clamp drop (about 0.8–1.1 V at 120 mA, estimate): 4.1–4.4 V. The load is 105–125 mA, about
    115–120 mA of it into the GPIO3 clamp.
  - U2 (UVLO ≤ 2.69 V rising; dropout about 0.14 V at 125 mA typical from Fig. 22, Table 1 maximum 420 mV at 200 mA):
    regulates near 8.4 V, sits at or in dropout at 7.4 V (VOUT about 2.8–3.3 V against U1's 3.0 V minimum), browns out or
    cycles near 6.4 V.
  - ESP32-P4 DS v1.2 Table 5-1 rates no injection current; latch-up is qualified at ±200 mA pulsed (Table 5-10).
  - A bench supply swapped on VSYS/VSS forward-biases D2 and puts U2's VIN about 0.6–0.9 V below GND (ADP7142 Table 3
    allows −0.3 V); the SAM carrier shares this.
  - No carrier-side gate can tell the cases apart: before U2 starts, the carrier ground sits only about 0.6 V above the
    host's. J3's "1" silk is present at (57.02–58.08, 48.57–50.03).
- **Consequence:** a wrong-type jumper puts about 0.12 A continuously into a pad of the chip-down P4, which cannot be
  reworked. Worse, on the bench the carrier looks alive (D3 lit, D1 may blink) but deaf (its TXD lands on host GND through
  R8), which is the symptom cables.md assigns to the harmless light carrier and easy to mistake for the missing SkyTraq
  driver while the current keeps flowing.
- **Proposal:** keep the cable rule as the mitigation and rewrite cables.md's 5-pin section:
  - List every board on the Mantis J1 by folder and load: the SAM carrier (light); `gnss-px1105r-18mm-hv` J3 (about
    0.105–0.125 A); the legacy Space Bug J4 (about 0.2 A); the Space Bug M8T J4.
  - For this carrier: reversed, it still powers up on a 2S pack and looks alive but deaf while about 0.12 A flows into
    GPIO3; a deaf PX1105R carrier gets a cable check before any driver debugging and is not left powered.
  - Optionally correct Mantis FAB B10's wording for the GNSS link (there the carrier's ground lands on the host's GPIO3).
  - Put the current-limited bench-supply note for flying leads in this carrier's own fab or bring-up notes; it is not a
    jumper hazard. F2's host-side 1 kΩ resistors would also bound this case to a few mA.
- **Prior:** Space Bug S7.

### D6. Documentation drift: the README row and the 3D-model README — minor

- **Where:** `hardware/README.md` line 45; `hardware/3dmodels/README.md` line 17; `docs/board-versioning.md` line 35.
- **Evidence:**
  - The README row (from `95b14623`, 2026-10-11) says "Schematic done 2026-10-08 ... placement in progress". HEAD
    `c942cfd1` is routed: DRC 0 errors, 0 unconnected, parity clean, 7 warnings; `check_board_parity.py` links all 49
    symbols. Title blocks say rev V1; no tag exists; board-versioning line 35 says "never fabricated, no tag yet".
  - 3dmodels/README line 17: "These files are supplied by the component manufacturers", and lines 21–23 place them outside
    `../LICENSE`. Four of this board's eight vendored models are drawn by this project in CadQuery from committed `build.py`
    scripts (ANT_GVLB208.A_TAOGLAS, PX1105R, FIL_SAW-5_1.4x1.1mm_RF360, Texas_DPY0002A). The README is unchanged since
    2026-08-02; the folder now holds 17 build scripts, and its table lists 55 models against 89 files. Space Bug D8 flagged
    the same wording; it is still open.
  - The README's pin-5 text belongs to S1.
- **Consequence:** a wrong build state in the release record and misstated licensing of project-drawn models. No copper
  effect.
- **Proposal:** at release, the README row: "routed; DRC 0 errors / 0 unconnected / parity clean (7 warnings: 4 library
  mismatches, 3 silk)", rev V1, tag `gnss-px1105r-18mm-hv-v1.0.0`, host "for Tinker-Mantis" (F6), pin-5 text as the owner
  settles S1. Update board-versioning line 35 with a dated entry. In 3dmodels/README, split the provenance: models with a
  `build.py` beside them are drawn by this project from the cited drawings, and the owner states their license; refresh
  the table.
- **Prior:** Space Bug D8.

### D7. C19 (DNP) still prints paste on both pads, one of them the feed — note

- **Where:** C19 at (54.66, 45.09), F, rot −90: pad 1 on the feed, pad 2 GND with a via in the pad.
- **Evidence:** schematic dnp yes, Value "DNP", no MPN; footprint attribute `smd dnp`. F_Paste flashes the full
  0.62 × 0.56 mm on both pads at (54.66, 44.61) and (54.66, 45.57); the gap between them is 0.40 mm with a mask web. KiCad
  10 has no option that strips paste from DNP parts (only fab-layer DNP options). Nothing covers C19; its neighbors' pads
  are 0.34 mm (C20) and 0.39 mm (D6) away. D6 (VRWM ±3.6 V, leakage under 10 nA) leaves the pads open to an ohmmeter. The
  owner dispositioned the identical Space Bug item (its D1/N14) by keeping the paste ("APERTURES 1:1") and writing "C19 IS
  DO-NOT-FIT".
- **Consequence:** a low-probability bridge, easy to see and to rework, that would kill the RF on that unit.
- **Proposal:** "C19 IS DO-NOT-FIT" in the fab notes, and a first-article step after reflow and before AE1: measure across
  C19, expect open. Optional zero-cost removal of both flashes: a footprint-level solder-paste absolute clearance of −0.3 mm
  or more negative on C19 (tested at −0.4 mm: both flashes gone, DRC unchanged). Do not strip F.Paste from the pad layers;
  that adds a library-mismatch warning. Either way the pads stay usable for M11.
- **Prior:** Space Bug D1 (C19 note), N14.

### D8. RF netclass membership rests on 13 auto-generated net names — note

- **Where:** `.kicad_pro` netclass patterns: Net-(AE1-FEED), Net-(C20-Pad2), Net-(FL4-IN), Net-(FL4-OUT), Net-(C22-Pad2),
  Net-(U3-AI), Net-(U3-AO), Net-(FL5-IN), Net-(FL5-OUT), Net-(C24-Pad2), Net-(U4-AI), Net-(U4-AO), Net-(C26-Pad1), and
  /RF_IN.
- **Evidence:**
  - RF class: clearance 0.2 mm, track 0.16 mm, via 0.6/0.3 (no RF vias exist; all 146 are 0.4/0.3). All 42 RF segments are
    0.16 mm on F.Cu. The minimum fill gap is 0.2005 mm (median 0.28 mm; 28 % of the length at 0.21 mm or less). The GND
    zone clearance is 0.127 mm.
  - Reproduced in a copy: renaming the FL5, C20 and U3 nets as a re-annotation would, then refilling, drops 5 nets to
    Default and their gap to 0.127 mm, with DRC unchanged (0 errors, the same 17 warnings). A net is also renamed if another
    node is re-annotated to sort earlier, if a part with a named pin joins the net, or if a library update renames pins.
  - My 2-D solve: the narrower gap lowers Z0 by about 1.7–2.2 Ω (49.3 → 47.1 Ω with mask, 51.9 → 50.2 Ω bare), under
    0.001 dB of mismatch loss on these lengths. A dropped net re-routed at the Default 0.09 mm would be about 60–64 Ω. The
    `.kicad_dru` calls the gap load-bearing; on this 0.0994 mm prepreg it sets Z0 only to within about 2 Ω.
- **Consequence:** release hygiene with a small RF effect. The owner keeps the Space Bug designators on purpose, so a
  re-annotation is unlikely.
- **Proposal:** at the next schematic edit, local labels on the 13 auto-named RF nets (RF_FEED, RF_HP, RF_SAW1_IN, ...)
  with the single pattern `/RF_*`, which also covers /RF_IN. Not `RF_*`: patterns are anchored and root-sheet label nets
  carry the "/" prefix (tested). Alternatively, netclass directive labels on the 13 wires. Not a schematic rule area, which
  would pull GND and +1V8 wires into the class. Changing the class via to 0.4/0.3 is optional and cosmetic.
- **Prior:** Space Bug D4 (still open there).

### D9. Five second 3D-model paths fail on this board by design; R1/R2/D1 differ from the library cosmetically — note

- **Where:** U1, AE1, FL4, FL5, D6: a second model entry `${KIPRJMOD}/../../3dmodels/...`; R1, R2, D1 (DRC
  lib_footprint_mismatch).
- **Evidence:** a STEP export prints exactly five "Could not add 3D model" lines, all for the second entries; the first
  path resolves for all five and every model is centered. The second entry is deliberate: added in `7654e58c` after
  `c97ecb8b` moved retired boards into `hardware/legacy/`, it is the only path that resolves from a legacy board (the legacy
  Space Bug uses PX1105R, FIL_SAW-5 and Texas_DPY0002A from this library). Six library files use the two-path convention.
  R1/R2/D1: pads, mask and paste are identical to KiCad 10's; they differ in courtyard/fab drawn as lines, a descr suffix,
  the `${KICAD9_3DMODEL_DIR}` path (still resolves), a missing hidden KiLib_Generator property and a 0.15 mm font thickness
  on two hidden fields.
- **Consequence:** five expected warnings per STEP export, and three DRC warnings beside U2's real one (S3).
- **Proposal:** do not delete the `../../` entry: it would silently drop models from the legacy boards, and deleting it on
  this board only would add five library mismatches. Optionally note the two-depth convention in 3dmodels/README (one
  "File not found" per such part is expected). Clean exports need a library-wide change (embedded STEP files, about 1.5 MB
  for the four used here, or one tested path variable). Updating R1/R2/D1 from KiCad 10 is fine and cosmetic.
- **Prior:** Space Bug D5.

### D10. The silk ID "PX1105R V1" reads like an early revision of the Space Bug — note

- **Where:** B.SilkS "TinkerRocket" at (50.96, 28.08) and "PX1105R \n${REVISION}" at (54.07, 30.24), resolving to
  TinkerRocket / PX1105R / V1.
- **Evidence:** the legacy carrier's tags v1.0.0 and v1.1.0 carry "TinkerRocket / GNSS PX1105R / V1", apparently never
  fabricated (v1.1.0 was "reference plot only"). The Space Bug v2.0.0, ordered 2026-09-29, reads "TinkerRocket Space Bug /
  PX1105R V2" (V3 in work). Pin 5 differs across the three: RXD2 on the legacy carrier, HOST_TX2 on the Space Bug, and on
  this board per S1.
- **Consequence:** ambiguity in logs and issues only; the boards look completely different (this one has a 20 mm patch on
  the back).
- **Proposal (optional, before the first order; no version bump, since the board is untagged and unfabricated):** for
  example "PX1105R-HV ${REVISION}" or "PX1105R CARRIER ${REVISION}" (avoid "PATCH", which reads like the semver field); drop
  the trailing space. Keep it in the top strip between H1 and H2 (about 12.4 mm free at x 46.77–59.17, y < 31.8; move the
  mirrored anchor right) and re-run DRC so no new silk warning appears. Record the ID in the fab notes.
- **Prior:** Space Bug D2.

### D11. Panelization and DFM queries that follow from the 0.2 mm edge and 0.1 mm hole-clearance rules — note

- **Where:** GND pours on all six layers at the outline; the left and right edges; the non-GND vias.
- **Evidence:**
  - Outline 22.000 × 30.000 mm on the centerline with 1 mm arcs. Pour to edge 0.2005 mm on the straight edges (0.1987 mm at
    the arcs) on every layer. JLC: routed edges ≥ 0.2 mm; V-cut ≥ 0.4 mm (±0.4 mm tolerance, 25° groove).
  - Near the edges: C135 (0805) 0.505 mm from the left edge, U2's pads 0.525 mm, the U2 pad-5 via hole wall 0.675 mm; on the
    right, the GND via fence (hole walls 0.51–0.62 mm), H1–H4 pads 0.55–0.65 mm, and the RF 0402s L7, C23, L3, C22
    (courtyards 0.93–1.0 mm). In board terms: C135/U2/C7 lie 12.5–25.1 mm below the top edge on the left; the right-edge
    parts 10.5–17.5 mm below the top.
  - Hole to other-net copper below JLC's 0.2 mm: on the inner layers 68 pairs (17 non-GND vias × 4 layers), 65 at 0.1775 mm
    to the pour and 3 at 0.150 mm to In2 tracks (the J3-Pad1 via at (46.05, 32.94) to a J3-Pad2 track; the J3-Pad1 and
    J3-Pad2 vias at (55.01 / 54.01, 49.425) to J3-Pad5 tracks); on the outer layers 21 more, 20 at 0.1775 mm and one GND
    via at (62.01, 38.31) 0.150 mm from a B.Cu +1V8 track. These rules are the owner's, matching Mantis and Beetle.
  - JLC Economic PCBA takes 6-layer single boards down to 10 × 10 mm without rails; Standard PCBA needs rails and a panel of
    at least 70 × 70 mm. The owner hand-assembled the SAM carrier.
- **Consequence:** a V-score would breach JLC's 0.4 mm rule and could expose or tear GND copper; breakout tabs near C135
  risk flex cracks (L9); expect a DFM query or antipad edits.
- **Proposal:** in Block A: "GND copper is 0.20 mm from the routed edge on all six layers: route the outline (single boards
  or a mouse-bite panel); no V-score (also impossible on the 1 mm radii). Break tabs only on the two 22 mm short edges (the
  top edge between H1 and H2 is clear; on the bottom D2 is 1.9 mm in); none on the long edges (left: C135 and U2 0.5 mm
  in, 12.5–25 mm below the top; right: the via fence and RF 0402s, 10.5–17.5 mm below the top)" (after Mantis A9). "Via hole
  to copper is 0.1775 mm at the 17 non-GND vias; the fab may enlarge the GND-pour antipads." The four 0.150 mm pairs are
  against tracks, so antipad edits do not apply: accept a query, or nudge those tracks about 0.05 mm.
- **Prior:** Space Bug N13.

### D12. Mantis power-eco.md still gives R48 = 100 Ω / 14.7 A for the eFuse limit — note

- **Where:** `hardware/tinker-mantis/power-eco.md` line 56 (the ILIM row) and line 70 (the margin note).
- **Evidence:** the Mantis schematic, netlist, BOM (line 46) and PCB all have R48 = 127 Ω (RC0402FR-07127RL) on
  Net-(U19-ILIM). TPS25982 SLVSEI3D Eq. 8 (p25): ILIM = 1460/127 + 0.11 = 11.6 A. 127 Ω is not tabulated, so the spread is
  an estimate from the 100 Ω and 182 Ω rows plus the 1 % resistor: about 10.1–12.9 A. WORKLIST D-1 (2026-08-09): "eFuse
  ILIM stays at 11.6 A", with a 3 A aggregate servo design point; WORKLIST line 90 and schematic-review-2026-09-21 line 62
  agree. The power-eco table claims to be "as built on V10 (re-verified from the netlist 2026-09-11)".
  pack-fire-holdup-options.md line 31 hedges with "11.6–14.7 A".
- **Consequence:** host documentation only; the fault analysis here (F3) uses 11.6 A.
- **Proposal:** line 56: "R48 127 Ω (RC0402FR-07127RL) | ~11.6 A typ (SLVSEI3D Eq. 8), per WORKLIST D-1". Line 70: "127 Ω →
  11.6 A typ, about 10.1–12.9 A across temperature (estimate)", with the 4-servo stall sentence replaced by a pointer to
  D-1. Optionally tighten pack-fire-holdup-options line 31.
- **Prior:** none.

---

## 6. Firmware and system

No SkyTraq driver exists yet; these are requirements and system items, not code reviews.

### F1. V_BCKP is tied to VCC: every carrier power cut is a 29 s cold start that loses SRAM configuration — minor

- **Where:** U1.6 V_BCKP (48.575, 35.56) and U1.8 VCC (48.575, 37.76), both on +3V3 in the same F.Cu zone. Host: R82
  (100 kΩ GPS_ACT pull-down) and U27 on VBATT.
- **Evidence:**
  - PX-DS rev 2 p5 (pin 6): V_BCKP "must be applied whenever VCC is applied"; with both removed the receiver starts "in
    factory default mode ... all user configuration set is lost"; "For applications the does not care cold starting every
    time, this pin can be connected to VCC". p2: TTFF 1 s hot, 28 s warm, 29 s cold. p7: backup current 13 µA with VCC
    off, 54 µA with VCC on; the application circuit uses an RB420/RB425, 1 µF, 390 Ω and a 3 V cell.
  - **Hold-up** (TDK DC-bias proxy, 3.45 V dropout floor): the carrier alone holds +3V3 for 0.28–0.36 ms at 125 mA
    (6.4–8.4 V); with the Mantis's C7 0.48–0.64 ms.
  - **Host:** GPS_ACT = U17.16 (GPIO15), with no input enable or pull at or after reset (P4 DS v1.2 Table 2-1), so a panic,
    watchdog or software reset releases it, R82 turns U27 off, and QOD discharges the branch.
  - **Mantis docs:** v10-power-parity lines 9–12 say losing GNSS through a brief interruption is not acceptable; §1.3 (lines
    86–90) calls a cold start after any VBATT interruption "the accepted state of the design today"; lines 99–105 name a
    carrier backup domain "a carrier-revision item". §3.2 events: bounces under 0.27 s, a trip and re-enable dark for
    1.8–4.3 s, 92 ms eFuse retries.
  - A firing sag never cuts VBATT on the Mantis (the C94 deglitch, and the LDO keeps more than 3 V of headroom), so the
    "survive a firing event" rule is met regardless of V_BCKP.
  - AN0037 0x05 offers "update to both SRAM & FLASH"; whether flash settings survive VCC + V_BCKP removal is unresolved
    (F11).
- **Consequence:** a sub-second cut, a P4 reset or a U2 thermal-shutdown cycle (M2's tail) costs a 29 s cold start plus
  reconfiguration, on flights that may last under a minute and for #1580's GNSS-led canopy mode; an FC reboot during pad
  prep costs a full cold start too. A backup domain turns that into a hot start of about 1 s with SRAM settings kept. It
  does not give ride-through.
- **Proposal (the owner's call; the DS p7 topology):**
  - A low-reverse-leakage Schottky (RB420/RB425/RB520 class) from +3V3 to V_BCKP with 1 µF at the pin, then about 390 Ω to
    a reservoir. Reverse leakage into the dead +3V3 rail adds to the 13 µA load, so pick a part that stays low at 85 °C.
  - Size the reservoir against the documented outages: a 22 µF 0805 gives about 1.4 s (bounces, 92 ms retries, a P4 reboot;
    not a trip and re-enable); a 100 µF 6.3 V 1206 about 5 s, or a chip supercap rated ≥ 3.3 V (an 11 mF class part about
    20 min; part not verified) covers every listed event. The 390 Ω limits charging to 7.6 mA or less, inside the 30–80 mA
    start-up current-limit margin.
  - Layout: cut U1.6 out of the +3V3 zone that feeds U1.8 and place the diode and the 1 µF at pin 6, a local re-route.
  - If declined: record in the Mantis hold-up notes that this carrier draws about 125 mA continuously and loses its
    configuration on every cut (F7), and make it a driver requirement to detect a cold-started receiver (115200 NMEA) at
    any time, including in flight, and re-run the configuration (F4).
- **Prior:** Space Bug F4; SAM prefab-review-2026-08-04 A4 (declined as 09-02 C6); SAM field-review C7.

### F2. A J3.2–J3.3 short puts the pack voltage onto Mantis GPIO3, which has no series resistor — minor

- **Where:** carrier J3 pad 2 at (54.01, 49.425) and pad 3 (VSYS) at (53.01, 49.425): 0.66 × 1.55 mm pads at 1.0 mm pitch,
  a 0.34 mm copper gap with a 0.34 mm mask dam (mask expansion 0). The Mantis J1 uses the same footprint. Mantis J1.2
  GNSS_TX → U17.3 (GPIO3); J1.3 ← FL1 (BLM18PG471SN1D) ← U27 ← VBATT. Harness conductors 2 and 3 are adjacent.
- **Evidence:**
  - Mantis netlist: GNSS_TX = {J1.2, U17.3}, two nodes, nothing in series (GNSS_RX and GNSS_RXD2 likewise). On the carrier
    R8 sits behind pad 2 (Net-(J3-Pad2) = {J3.2, R8.2}), so it cannot help when pad 2 itself is shorted to VSYS; it does
    limit the carrier's own receiver to about 4 mA.
  - U27 TPS22810: thermal shutdown only, no current limit (SLVSDH0C p1, §9.1, §9.3.5); RON 79 mΩ typ. The Mantis FL1 is
    0.2 Ω max.
  - Estimate: 0.4–3.5 A into the GPIO3 clamp over 6.4–8.4 V (0.6–2.6 A at 7.4 V); the clamp's resistance is not published.
  - ESP32-P4 DS v1.2: no injection rating (Table 5-1); VIH ≤ VDD + 0.3 V (Table 5-4); latch-up qualified at ±200 mA
    (Table 5-10), so 2–17× that trigger. GPIO3 is in the VDD_LP/VDD_BAT domain on V_MCU_SWTCH, fed by U30 (another
    TPS22810 with no reverse blocking), so any lift spreads onto +3V3 (S3 U15, flash U11/U13/U16, INA230).
  - cables.md lines 38–40 note that GPIO3 has no series resistor, for the reversed-cable case only. Lines 25–32 record the
    2026-07-31 pack-voltage injection into an S3 GPIO: the processor survived and the radio on the same 3.3 V rail did not.
  - The SAM carrier flew with the same pinout, so the exposure is not new, and the carrier cannot fix it (the pinout and
    footprint are the host's).
- **Consequence:** at the first GPS_ACT after a bridge, or during a pinch, the 3.3 V tree is likely driven above the 3.6 V
  abs max of the P4, S3 and sensors; damage to the GPIO3 pad is plausible. Probability is low: a 0.34 mm gap with a mask dam
  at 1.0 mm pitch, or a pinch that hits exactly conductors 2 and 3.
- **Proposal:**
  - **Every carrier, before first mating:** DMM J3.2–J3.3 above about 1 kΩ in both polarities (R8 is in the path); J3.3–J3.4
    never beeps (diode mode shows only D2's drop, red lead on J3.4); inspect pads 1–5 under magnification.
  - **Bench power-up** at 7.4 V with a 0.3 A limit through the actual flight jumper: J3.2 ≈ 3.3 V (TXD idling high through
    R8), and J3.1 and J3.5 not at VSYS. The supply current alone does not reveal a 2–3 bridge.
  - **cables.md:** extend the per-jumper check to isolation between adjacent conductors (2–3, 3–4) and a pinch-point check
    after airframe integration; record the 2–3 case beside the reversed case, noting the back-feed onto +3V3 through U30.
  - **Mantis V11:** 1 kΩ 0402 at the P4 end of GNSS_TX (U17.3), GNSS_RX (U17.4) and GNSS_RXD2 (U17.2). That bounds this
    fault to 3.5–4.5 mA and the reversed-cable case to a few mA; the added RC is about 2 ns against a 1.085 µs bit at
    921600.
  - No carrier redesign: re-pinning J3 breaks the hosts and the SAM carrier, and a TVS cannot clamp an 8.4 V source.
- **Severity:** minor for this board; the impact check would carry the host resistors as a major-class item on the Mantis
  V11 list.
- **Prior:** none (related: Space Bug S7, cables.md).

### F3. The Mantis GNSS branch has no current limit below the 11.6 A eFuse; a hard VSYS short draws about 9–25 A — minor

- **Where:** carrier D2 SMF10A at (52.97, 54.43), across VSYS–VSS ahead of FL1; J3.3/J3.4; the jumper. Mantis J1.3 ← FL1 ←
  U27 TPS22810 (VOUT and QOD tied; VIN on VBATT; EN = GPS_ACT) ← VBATT ← U19 TPS259824L (R48 127 Ω; C50 10 nF ITIMER;
  C46 2.2 nF; NRETRY to GND; UVLO divider R44 1 MΩ / R45 210 kΩ with C94 10 µF).
- **Evidence:**
  - **Faults:** D2 failed short or fitted reversed; a pinch at J3.3–J3.4, in the cable or at J1. (A screw abrading the H3
    Net-(U2-EN) pour is after the carrier's 320 mA choke: about 4.9–14 A with 7–39 W per winding, so FL1 likely opens
    first; VSYS and VSS copper is at least 4.8 mm from every hole.)
  - **Path resistance:** pack 0.15–0.20 Ω (median 0.171 Ω from my regression of the 2026-07-05 logs: about 124 samples above
    0.5 A, median 0.67 A, peaks 1.7 A); U19 2.7 mΩ typ (4.5 max); U27 79–105 mΩ; Mantis FL1 0.10–0.20 Ω; SH contacts
    5–20 mΩ each; 30 AWG × 51 mm about 17 mΩ per conductor; carrier VSYS/VSS copper about 13 mΩ.
  - **Current:** 9.2–21 A for a short at the carrier, 10.5–24.6 A at the host end (the top end reaching the 2.1 × ILIM fast
    trip, about 24.4 A). VBAT_CON falls to 3.6–6.0 V. The Mantis FL1 dissipates 17–60 W (1 A rated), U27 9.7–35 W (3 A
    rated; its pulse rating was deleted in Rev A).
  - **eFuse (TPS25982 SLVSEI3D):** ILIM 11.6 A (Eq. 8); ITIMER 2.5–9.3 ms; the L variant then limits current until thermal
    shutdown and retries. The UVLO senses VBAT_CON with τ = 1.74 s and a 6.34 V trip: 1.6–3.4 s from 8.4 V, 0.7–1.2 s from
    7.4 V, 0.04–0.06 s from 6.4 V with nominal C94 (a 0402 6.3 V X5R, so DC bias likely cuts these by 30–50 %).
  - `high-side-switch-design.md` lines 84–92 set no current-limit requirement for the branch.
- **Consequence (estimate; no fusing I²t is published):** the Mantis 0603 bead most likely opens first; U27 may simply
  thermal-cycle (TI states its thermal shutdown protects its SOA). GNSS is lost for the flight and the bead or U27 needs
  rework. VBATT loads dip to 3.7–6.0 V for milliseconds; the pyro channels fire from VBAT_CON; +3V3 has its mux, buck and
  2 F holdup. A flight-computer blackout needs a pack already at the #1029 UVLO edge.
- **Proposal:**
  - **Mantis V11, the real fix:** replace U27 with TI's TPS25961 (2.7–19 V, 21 V abs max, 0.1–2 A set by one resistor,
    auto-retry, a 2 × 2 mm WSON like the TPS22810DRV; the pinout differs, so check the footprint), limited at about 0.4 A
    (0.32–0.48 A spread, about 2.5× the 127 mA load), and confirm that the turn-on inrush into C7 and C136 does not cause a
    nuisance fault. The TPS2596 also fits electrically but is an HSOIC-8 (about 4.9 × 6 mm). Cheaper: a very fast 0603
    0.5 A, ≥ 32 V fuse in series with the Mantis FL1 (it covers the cable and J3 and must be replaced after a fault).
  - **Carrier, this order:** no layout change. A carrier fuse would protect only D2 and the J3–D2 copper, not J3 or the
    cable, and would need VSYS rerouted out from under J3 (the branch via at (53.75, 51.76) is inside J3's courtyard); if it
    is ever fitted, put it ahead of D2.
  - **First article:** a diode test across J3.3(+)/J3.4(−) (a reversed D2 reads 0.6–0.8 V) and a bench power-up at 8.4 V
    with a 200–250 mA limit before mating. A destructive short on a sacrificial Mantis is optional and not needed to justify
    the V11 change.
- **Prior:** none.

### F4. No consolidated configuration spec exists for the future SkyTraq driver, and the factory state is not fit for flight — minor

- **Where:** the future Mantis SkyTraq driver. U1's UART: J3.1 → R7 → U1.3 (GNSS_RX), U1.2 → R8 → J3.2 (GNSS_TX). Mantis
  GPS_ACT → U27 → carrier VSYS.
- **Evidence:**
  - **Factory state:** NMEA at 115200 8N1; RTK up to 10 Hz, raw up to 20 Hz; the fix is withheld above 515 m/s or 80 km
    while the receiver keeps streaming (PX-DS rev 2 p2). AN0037 v1.4.69 p21 (0x0C): "By default power save mode is enabled".
  - **Power save on the owner's SDR bench** (cocom results README, about l.1364–1450): 30–36 multi-channel collapses per
    600 s pad against 1–2 in normal mode, 6–13 fix dropouts over 0.3 s against 0–2, and about a 1 s fix drop every 11–13 s
    on the pad. (The "never fixed / fixed in 53 s in normal mode" result in that file is the PX1125R.) The 1200 s real-sky
    capture shows no collapses, with the power mode unrecorded.
  - **Navigation mode** (3 g, 82.5 km gentle profile, power save, factory 15° mask, one run per mode): p95 |altitude error|
    airborne 57 m, SLR 97 m, quadcopter 133 m, balloon 234 m (that run had a HackRF underrun), pedestrian 1775 m (loses the
    fix). All modes ACK and read back. The owner's current bench setting is SLR.
  - **Bandwidth:** the PX-DS pp15–19 example epoch (25 sentences) is 1571 B: 136 % of 115200 at 10 Hz, 68 % of 230400,
    34 % of 460800. AN0037 p22 (0x0E): rate 10 or higher "may require baud rate 230400 or 460800". The owner's real-sky
    20 Hz raw capture measured 23.35 kB/s framed, more than 230400 carries. A trimmed set (GGA and RMC at 10 Hz, the rest at
    1 Hz) is about 3.1 kB/s, 27 % of 115200.
  - **Host needs:** the EKF consumes vertical velocity (EkfGnssFeed.h l.81); GGA altitude is documented only to 17,999.9 m
    (PX-DS p15); the LC86 driver refuses to present a fix without vertical velocity.
  - **The core already exists:** the owner's results README (l.1415–1419) says flight software "must, at every boot" turn
    power save off (0x0C 00), set a 3–5° mask (0x2B) and set a dynamics mode (0x64/0x17), all in SRAM; the P4 bridge
    firmware (`px1105r_p4_bridge.c` l.31–33) says SRAM only (attribute 0).
  - **Precedents:** the UBX driver prefers 460800, validates framing before accepting a baud (l.79–231) and has a 4096 B
    RX buffer. TinkerNav's `TR_SkyTraqNMEA` writes attribute 1 (l.192 0x0E; l.201–211 0x64/0x3B), sends 0x04 type 02
    (undefined; AN0037 p18 defines only 01; NACKed in every 2026-10-11 log where the receiver answered) and the
    undocumented 0x64/0x1E (which ACKs on hardware), with 1000 ms ACK waits.
- **Consequence:** a driver written by analogy would run in power save with a 15° mask and default dynamics, overflow 115200
  at 10 Hz with the default sentences, write flash at every boot (F11), and could power-cycle or cold-start the receiver on
  an export-limit fix gap. Firmware work that gates the first flight, not the board order.
- **Proposal:** a driver spec that consolidates the owner's rule.
  - **Must-haves:**
    - R1: configure at every boot, attribute 0 only, read every item back, ACK window ≥ 1.5 s.
    - R2: no flash writes on any automatic path (attribute 1, 0x04, 0x64/0x3F type 1, 0x64/0x1E).
    - R5: baud discovery with a framing check (the flight baud first, then 115200, then the other AN0037 codes; accept only
      valid SkyTraq frames or NMEA with a valid checksum). If nothing frames before the deadline, run GNSS-absent as the
      UBX driver does; 0x04 01 is a bench command (R3), not a boot step.
    - R9: power normal (0x0C 00 00), checked by 0x15 → 0xB9.
    - R11: 3° elevation mask, CNR mask 0 (0x2B 01 03 00 00).
    - R8: navigation mode as a parameter, default SLR (9) until a power-normal, 3°-mask re-run of airborne against SLR,
      scored on altitude and velocity, picks the winner.
    - R7: 10 Hz (0x0E 0A 00), with the primary output carrying 3-D velocity and an altitude valid above 18 km: binary
      0xDF (AN0039, about 81 B) or 0xA8 (AN0037 p182), enabled as on the owner's bench (including kinematic-base mode
      0x6A if this firmware needs it for 0xDF/0xE5), mapped to vel_n/vel_e/vel_u. GGA and RMC only as liveness and
      fallback; bench-check GGA altitude above 17,999.9 m; no fix without a vertical-velocity source.
    - R6: 460800 (0x05 00 07 00, then 0x10 → 0x86 at the new baud), justified by the binary and raw load.
    - R14: liveness from frames, never from the fix; re-apply the configuration when 115200 NMEA reappears; power-cycle
      through GPS_ACT only on the pad, after parking the host TX lines (main.cpp l.4020) and a measured +3V3 decay
      hold-off; never in flight.
    - R13: log 0x02 01 → 0x80 and 0x64/0x01 → 0x64/0x80 at boot.
    - R15: do not port `TR_SkyTraqNMEA` unchanged.
  - **Optional** (log a NACK and carry on): interference detection 0x64/0x06 (R12); QZSS 0x62/0x03 (0 of 7 QZSS satellites
    were used from the owner's longitude); the constellation mask, queried and logged (F9).
  - R3: flash provisioning only from an explicit bench command, after F11's persistence test. R4: never download receiver
    firmware in the rocket (a BOOT_SEL recovery needs a wire to U1.18).
  - Measure U2's thermal (M2) in the flight configuration (power normal, 10 Hz, flight baud): power save lowers the search
    current (AN0037 p21).
  - **First-article bench** (Mantis or 8.4 V VSYS, 3.3 V USB-UART on J3.1/J3.2): log the factory state (0x02, 0x64/0x01,
    0x15, 0x2F, 0x64/0x18, 0x10, 0x64/0x1A) and ACK latencies; apply the SRAM set and read it back; switch baud; count
    sentences for 60 s at 10 Hz (settles the 0x64/0x3B unit); power-cycle through GPS_ACT and confirm every item reverts;
    scope the +3V3 decay with the host pins parked; 30 min of real sky in power save, then 30 min in normal, counting fix
    dropouts.
- **Contested?** No. The skeptic agreed the gap is real and showed that the first headline, "no requirements exist", was
  overstated; the severity came down from major because nothing here gates the board order.
- **Prior:** none.

### F5. On today's UBX-only image every FC boot with this carrier stalls at least 60 s in GNSS bring-up — note

- **Where:** tinkerrocket-idf `TR_GNSSReceiverUBlox_Serial.cpp` `begin()`; `TR_Sensor_Collector.cpp:715`; flight_computer
  `main.cpp:4647` (`setup_fc`); the in-flight reboot recovery block at about main.cpp 4955–5340.
- **Evidence:**
  - `kBeginTimeoutMs = 60000` (l.116), set when `begin()` starts; probe bauds {38400, 9600, 230400, 115200, 460800, 57600}
    with swapped-pin attempts; every false return is gated by the deadline.
  - In this library copy `_signsOfLife` is set only by a valid UBX checksum (u-blox_GNSS.cpp l.3322); `isNMEAHeaderValid()`
    (l.2368) compares the stored "$" to "*", so NMEA never counts. The PX1105R's 115200 NMEA never connects, and `begin()`
    returns false at about 60.4–65 s (my model).
  - It runs inside `sensor_collector.begin()`, before `startPollingTask()`, called from `setup_fc()` at line 4647, ahead of
    the in-flight reboot recovery. The "~35 s" comments at lines 4641, 4644 and 4664 are stale. `USE_GNSS` is a constexpr
    `true` in `board_v9.h` line 120, which covers the V9 and V10 Mantis builds.
  - The PX1105R ignores UBX frames (SkyTraq needs 0xA0 0xA1, a length and an XOR checksum, AN0037). The swapped-pin probes
    drive GPIO3 against U1's TXD through R8, at most about 3.3 mA. No firmware captures PPS.
- **Consequence:** about a minute from power-up to sensor polling on every boot. After an in-flight FC reboot, IMU polling,
  the state machine and the snapshot restore wait about a minute, which defeats the reboot recovery. (A dead SAM-M10Q
  causes the same window today.)
- **Proposal:**
  - A fly/no-fly line in this carrier's release notes: "Do not fly on a UBX-only FC image (TR_BOARD_V9, which covers V9 and
    V10)."
  - Interim firmware, cheapest first: a board-header or variant build with `USE_GNSS = false` (a constexpr, not a flag); an
    NVS carrier identity that skips UBX bring-up; or have `begin()` give up as soon as it sees checksum-valid NMEA at a baud
    that never answers UBX (gnss_framing.h already detects it).
  - Root fix beyond this board: move the in-flight reboot recovery ahead of the blocking GNSS bring-up, or run the bring-up
    in the GNSS poll task.
  - Driver references: 0x05 (baud codes 0–8; 5 = 115200, 8 = 921600; AN0037 p19), 0x0E (p22), 0x64/0x3B (p99). PX-DS lists
    no supported messages, so the first driver work should query 0x02 and check ACK/NACK replies.
- **Prior:** Space Bug F1 (the Mantis speaks only UBX).

### F6. The Tinker-Beetle cannot host this carrier — note

- **Evidence:** the Beetle's connectors are J2 (pyro screw terminal), J3 (JST PH battery), J6 (USB) and J8 (SMA); it has no
  BM05B-SRSS-TB, and its GNSS is the on-board U5 LC86GLAMD. Its holes at (74.88, 104.36), (92.60, 104.35), (74.89, 153.00)
  and (92.60, 152.98) form a 17.72 × 48.64 mm pattern; the closest pair misses the carrier's 16.75 mm by 0.97 mm (an M2 in a
  2.2 mm hole leaves about ±0.1 mm). The Mantis J1 (BM05B, B side, at (74.77, 115.35)) and its H2/H4/H6/H8 match. The
  README's SAM-carrier row says "for Tinker-Mantis"; the PX1105R row names no host; only this review's brief named the
  Beetle. Beetle pin-budget.md line 353 ("J1, the external GNSS connector") is stale history from before the fork.
- **Proposal:** treat the carrier as Mantis-only: correct the brief; add "for Tinker-Mantis" to README line 45; name the
  Mantis J1 and H2/H4/H6/H8 (and the V11 requirement, K4) in the fab notes; optionally mark the Beetle pin-budget line as
  history. No hardware change.
- **Prior:** none.

### F7. The Mantis hold-up documents size GNSS ride-through for a 25–40 mA buck carrier; this one draws 105–127 mA — note

- **Where:** `tinker-mantis/pack-fire-holdup-options.md` lines 47–50 and 114–131; `v10-power-parity-2026-09-11.md` lines
  74–79 and 93–102; `high-side-switch-design.md` line 13.
- **Evidence:**
  - The options doc still has both carriers on a TPS62913 buck with a "~3.8 V floor" (stale since the SAM V3 of
    2026-09-02) and sizes Option C at 40 mA. The parity doc: "C7 22 uF holds the branch for a few ms at the module's
    ~25 mA", a 23 s cold start, a backup current of "~3 uA", and 4.7 mF ≈ 0.7 s / 10 mF ≈ 1.6 s at 25 mA. The high-side
    switch doc says "daughterboard buck input".
  - This carrier draws 104.9–127.2 mA of constant current at any VIN (PX-DS p7; BGA855N6 Table 4; ADP7142 IGND ≤ 0.32 mA).
    The floor at the Option C node is about 3.3–3.7 V (VCC 3.0–3.3 V plus dropout 0.13–0.27 V, FL1 0.078 V, FL2, the host
    bead and the harness), below the old 3.8 V, so "protect against the cut, not the sag" still holds.
  - Option C recomputed (0.35 V Schottky, −20 % on the capacitor): 10 mF bridges 0.15–0.26 s at 6.4 V, 0.22–0.36 s at
    7.4 V, 0.28–0.45 s at 8.4 V; 4.7 mF 0.07–0.21 s. Bridging a 1.8–4.3 s trip-and-re-enable dark period would need about
    40–280 mF.
  - PX1105R values for the docs: 95/115 mA (p7), 29 s cold and 1 s hot start (p2), 13 µA backup with VCC off (p7); this
    carrier ties V_BCKP to +3V3 (F1).
- **Consequence:** an Option C drawn from these docs would be 3–5× too small and would fully cover a 0.27 s bounce only at
  8.4 V.
- **Proposal:** in the docs, a constant 105–127 mA (the capacitor discharges linearly, t = C·ΔV/I) and a 3.3–3.7 V floor.
  If Option C is ever drawn, size it at 0.13 A, 6.4 V and −20 %: about 18 mF (for example 22 mF/16 V) for a 0.27 s bounce,
  32 mF for 0.5 s, and say that bridging an eFuse trip is impractical. Replace the SAM-M10Q figures with the PX1105R's.
  Describe F1's backup domain as a faster restart that keeps the configuration, not as ride-through. Change only "buck
  input" to "LDO input" in the high-side switch doc; its 0.5 A design current stands.
- **Prior:** none.

### F8. The pad wait drains about 65–112 mA more than with the SAM carrier — note

- **Evidence:** the SAM carrier draws 15–40 mA at the pack (prefab-review-2026-09-02 line 145); this one 105–127 mA, i.e.
  0.78–1.07 W at 7.4–8.4 V. The extra is 49–84 mAh per 45-minute pad wait (624-checkpoint-a-matrix line 273, a planning
  figure from a phone-battery test; recorded pad sessions were 6–15 min), or 7.5–12.9 % of a ~650 mAh pack (the pack is not
  specified anywhere I found). The only measured baseline, 92–100 mA whole-board median in three of the four 2026-07-05
  logs, comes from pre-Mantis computers with unrecorded GNSS hardware, so a ratio to the Mantis draw is not available; the
  Mantis's own pad draw is unmeasured and probably higher. During a long wait the receiver tracks: about +80–90 mA.
  Pack sag from the carrier is ≤ 23 mV at 0.18 Ω. 7.2 V is the battery verdict's OK/DEGRADED boundary, and "Nothing refuses
  ARM" on it (RocketComputerTypes.h ~1891); arm refusal is only proposed. The delta comes from the receiver choice; a
  converter would draw about 49–77 mA, which the owner's LDO decision forgoes.
- **Proposal:** plan pad time and pack size against the absolute +65–112 mA. Measure the carrier's current at J3.3 (bench
  ammeter or the Mantis INA230) in acquisition and tracking at 6.4 / 7.4 / 8.4 V, and the Mantis's own pad draw with GPS_ACT
  off on the same day. Deferring GPS_ACT is an optional firmware change: keep the #700 rail-before-UART order, and each
  enable is a 29 s cold start (F1).
- **Prior:** none.

### F9. The patch puts B1I about 4 dB and GLONASS G1 4–8 dB below L1 in RHCP: keep the constellation mask at 0x001D, GLONASS off — note

- **Evidence:**
  - GV-DS rev B pp16–18, re-digitized at 1575.42 / 1561.1 / 1598.06 / 1602 / 1605.4 MHz: efficiency 37.75 / 22.37 / 21.96 /
    15.64 / 11.13 %; return loss −14.13 / −5.93 / −2.23 / −1.68 / −1.38 dB; axial ratio 4.37 dB at L1, 16.28 dB at B1I and
    off scale (> 20 dB) above about 1594.5 MHz. The plotted average gain equals 10·log(efficiency) (−4.24 dB at L1), so it
    is total gain, not RHCP. GV-DS p4 marks B1I and G1 as not covered. (The p5 table's 50.2 % at L1 disagrees with the
    curves; I used the curves throughout.)
  - Handicap against L1: total gain B1I −2.27 dB, G1 −2.35 / −3.83 / −5.30 dB; co-polar fraction −0.26 dB at L1, −1.87 dB
    at B1I, −2.23 to −3.01 dB on G1. RHCP handicap: B1I −3.89 dB; G1 −4.32 to −8.06 dB.
  - SAW typical loss (B8389 Fig. 4, digitized): 1.41 dB at L1, 1.55 at B1I, 1.65–1.78 on G1. GLONASS group-delay ripple is
    up to 8 ns per SAW (p7), two in series. GLONASS L1's minimum power is about 2.5 dB below GPS L1 C/A (ICDs).
  - The owner's outdoor capture (24,000 epochs): 6 GPS, 8 BDS, 6 Galileo satellites; medians B1I 41 dB-Hz (10th percentile
    31), L1 C/A 42, B2a 43, E1 41, E5a 47, L5 46. On this carrier I expect B1I at about 37 dB-Hz, the PX-DS p13 RTK guideline.
  - AN0037 p80 / p81 / p114: 0x64/0x19 (mask bits GPS, GLONASS, Galileo, BeiDou, NavIC), query 0x64/0x1A, reply 0x64/0x8C.
    The owner's receiver reads 0x001D. The patch's detuning on this plane (M1) moves the match toward B1I and away from G1.
- **Proposal:** at every boot query 0x64/0x1A and write `64 19 00 1D 00` (attribute 0, SRAM) only if it differs; V_BCKP on
  +3V3 loses SRAM at every cut (F1). Keep BeiDou: it supplied 8 of 20 satellites and all of B2a. GLONASS off is the factory
  default on these units, so this keeps it rather than changing it. First article: compare B1I and L1 C/A at similar
  elevations, and record the B1I 10th percentile.
- **Prior:** none.

### F10. GLONASS is absent from the owner's firmware build (rev 2023-08-31), not from the product — note

- **Evidence:**
  - PX-DS rev 2 claims GLONASS L1 (p1, p2, p12, p29; p29 also lists a PX1105R-01 GLONASS variant without NavIC), RTCM
    1084–1087 decoding in the rover (p3) and 1084/1087 output in the base (p4).
  - TinkerNav commit `73b0726` (2026-10-11, on the unpushed branch `fix/px1105r-rtcm-no-glonass`): on kernel 03.06.00, ODM
    01.07.33, rev 2023-08-31, message 0x20 NACKs with the GLONASS MSM field set and ACKs with it clear, also after a 0x04
    restore-defaults. The payload matches AN0039 v1.4.42 PDF p15–16 field for field, so the NACK is about content. NACKs
    come back in about 0.1–0.2 s.
  - The owner's 1200 s outdoor capture has no GLONASS entry in 24,000 0xE5, 1200 0xE7 or 1200 0xE8 messages and no 0xE1/0xE3
    at all; 0xE7 lists untracked NavIC (12) and QZSS (7) satellites but no GLONASS. The Space Bug review M1 recorded "rev
    230831, the GPS/Galileo/BDS/NavIC build". The firmware date predates PX-DS rev 1.
  - The version strings live only in the commit message and session memory (no saved 0x80 reply), and the 0x001D mask is a
    source comment, not a logged readback.
  - The hardware passes G1: B8389 PB2 (1559–1607 MHz), and the module's internal L1 SAW by inference from PX-DS p3.
  - Unresolved: whether the build lacks GLONASS or only ships with mask bit 1 clear. Only a 0x001F test settles it.
- **Consequence:** a later PX1105R may ship with a GLONASS build and GLONASS enabled, at the poor G1 antenna response of F9.
- **Proposal (driver):** query 0x02 and 0x64/0x1A at every boot and log kernel, ODM, revision and mask. Treat a revision
  other than 230831, or a mask other than 0x001D, as an unqualified build: log it and do not fly it until re-tested. Write
  0x64/0x19 with attribute 0 and handle a NACK rather than writing a hard-coded mask blindly. On a GLONASS-capable build,
  decide the policy from a G1 C/N0 measurement. Bench test: send 0x64/0x19 = `00 1F 00` to a 230831 unit and record
  ACK/NACK, the 0x8C readback and whether 0xE7 then lists type-2 channels; reword the memory note only after that (until
  then: "GLONASS absent at factory default on rev 230831"). Pushing `73b0726` is a TinkerNav action.
- **Prior:** Space Bug M1.

### F11. Saving the configuration to flash has no power-loss protection; keep it SRAM-only — note

- **Where:** +3V3 (C135 → FL2 → C1, C5, C27, U1.8/U1.6); Net-(U1-RSTN) = {R2.2, U1.9} only; U1.18 open; GPS_ACT/U27.
- **Evidence:**
  - PX-DS rev 2 p5 (both supplies removed → factory defaults) and p6 (BOOT_SEL: pull low "for loading firmware into empty or
    corrupted Flash memory from ROM mode"). Neither PX-DS, AN0037 v1.4.69 nor AN0039 gives flash write time, endurance or
    power-loss behavior.
  - AN0037: attribute 1 "update to both SRAM & FLASH" on 0x05, 0x0C, 0x0E, 0x64/0x17, 0x64/0x19 and 0x64/0x3B (p19, p21,
    p22, p78, p80, p99); 0x64/0x3F type 1 "Reboot with SRAM data save to Flash" (p101); 0x64/0x80 "Boot from ROM due to
    flash boot failure" (p107). The p43 remark that flash settings take effect at power-up without a backup supply belongs
    to 0x54, "supported only in timing mode receivers".
  - The owner's 2026-10-11 logs (0.2 s granularity): the receiver ACKs an attribute-1 command and is then busy about
    1.0–1.2 s, the same in four units (an inferred commit time); attribute-0 0x20 ACKs within 0.2 s.
  - Hold-up is 0.28–0.65 ms on the carrier's capacitors (0.44–1.02 ms with the Mantis C7 and the TPS22810 discharge),
    1500–4300× shorter than a commit. Riding through 1 s at 125 mA from 8.4 to 3.4 V would need about 25 mF.
  - TinkerNav's base sketches commit six times per boot (the 0x64/0x3B calls; its attribute-1 0x0E is never called); the
    rover sketches commit none, and all four send the undocumented 0x64/0x1E. The owner's P4 bridge firmware already says
    flash writes lose the receiver "for good, so use SRAM only".
- **Consequence:** only if a future driver writes flash in the field: a second interruption within the roughly 6 s of
  commits could tear a record, and whether a torn record is detected is undocumented (best case factory defaults, worst case
  silence at an unknown baud). An interrupted image download falls back to ROM, or needs BOOT_SEL pulled low with a wire to
  U1.18 (TP1 was removed by the owner).
- **Proposal:** no board change. The rule, whatever the bench shows: attribute 0 at every boot; never attribute 1 or
  0x64/0x3F type 1 from flight firmware; do not port the attribute-1 calls or 0x64/0x1E; re-send the configuration whenever
  the default output reappears (a carrier cut need not reset the P4). A persistence test that cannot pass falsely: query
  0x10 for the default rate and pick another; run a control first (attribute 0, power-cycle, confirm the default returns,
  because V_BCKP is specified down to 1.3 V and +3V3 can linger near 1 V at microamp loads); confirm +3V3 is below 0.1 V
  during the off time; then repeat with attribute 1. If flash does persist, provisioning stays an optional bench-only step.
  An open-drain reset IC on RSTN only if the bench shows hung receivers after short dips (PX-DS p6: RSTN is needed only for
  a very slow rise or a software reset).
- **Prior:** Space Bug S4, S13; F1 here.

---

## 7. Notes (no action needed)

### N1. The filter-first cascade's NF is above the PX1105R's "< 2 dB" antenna guidance at L1 — note

- **Where:** AE1 feed → D6 → C20/L1/C21 → FL4 → L2/C22/L3 → U3 → … → U4 → RF_IN.
- **Evidence:**
  - The series line ahead of FL4 is about 5.6 mm (feed to C20.1 3.28 mm, C20.2 to C21.1 0.84 mm, FL4-IN 1.44 mm); the other
    ~2.6 mm of RF copper ahead of FL4 is stubs to D6, C19 and L1.
  - Friis cascade with estimated line and passive losses: NF about 2.64 dB at L1 typical (2.60–2.71 by method), 1.98–2.10 dB
    at L5; worst case 4.40 / 3.30 dB. Gain 27.7 dB (L1) and 32.6 dB (L5), under PX-DS p12's 40 dB.
  - PX-DS rev 2 p12: "gain up to 40dB and noise figure less than 2dB". AN596 Rev 1.3 Table 1, Option B: 15.7 dB / 0.70 dB at
    1575 MHz, 17.6 dB / 0.70 dB at 1176 MHz. BGA855N6 Table 4 (1214 MHz only; the datasheet covers 1164–1300 MHz): NF 0.60
    typ, 1.10 max; gain up to 1 dB low. B8389 v2.1 p7: PB2 1.8 typ / 2.5 max, PB1 1.0 / 1.6 dB (Fig. 4 digitized: 1.40 dB at
    L1, 0.88 at L5). D6 ≤ 0.05 dB (SLVSDG9C Fig. 5-11).
  - C/N0 cost against a 2 dB-NF active antenna: L1 typical 0.7–1.1 dB (antenna temperature 100–290 K), worst 3.2–3.6 dB;
    L5 typical ≤ 0.1 dB.
  - What it buys: ≥ 20 dB of rejection at 600–1112 MHz (LoRa) and ≥ 24 dB at 2–3 GHz. Taoglas's TFM.100A, with the same
    SAW/LNA/SAW/LNA topology, specifies NF 2.2 dB at L1. The values and topology are the Space Bug's (its N1: 2.9 / 2.0 dB).
- **Proposal:** no change. The feed-to-C20 run (3.3 mm) is already near the minimum the Ø3.0 mm keep-out allows (at most
  about 0.05 dB to gain). Confirm per-band C/N0 at first article (M19 rows 8 and 11).
- **Prior:** Space Bug N1, N2.

### N2. Pad parasitics on 0.0994 mm prepreg: 0402 lands are about 21 Ω sections, and the feed node carries about 0.5 pF — note

- **Where:** every 0402 RF land (C20–C22, C24, C26, L1–L8); the feed branch from (55.06, 43.79) to C19.1 (54.66, 44.61) and
  D6.2 (55.61, 44.50).
- **Evidence:**
  - 2-D solve: the 0.16 mm line is 104–109 pF/m (51–53 Ω, εeff 2.77). A land 0.62 mm (capacitors) or 0.64 mm (inductors)
    wide across the path is 280–287 pF/m (21.0–21.5 Ω): about 0.10 pF of excess per land (0.097 for capacitors, 0.106 for
    inductors), |Γ| 0.024–0.034 per land at L1.
  - The AN596 board (Megtron 6, 0.25 mm, εr 3.7) has about 2.2× less land capacitance, and its 50 Ω lines (0.45–0.5 mm) are
    about as wide as an 0402, so its lands are barely discontinuities.
  - Feed node: 0.89 mm of branch track outside pad copper (0.09–0.10 pF), the C19.1 land (0.16–0.18 pF), the D6.2 land
    (0.07–0.09 pF) and D6 itself (0.15 pF typ, 0.18 max, SLVSDG9C §5.6): about 0.47–0.51 pF typical, about 0.55 pF with D6 at
    its maximum. That is |Γ| 0.118 / 0.088 and 0.061 / 0.034 dB of mismatch loss at L1 / L5. The longest real stub (C20's pad
    edge to D6) is 0.68 mm, 2.1° at L1.
  - In1 and In2 GND are 100 % under every RF land; no RF net has a test pad. The Space Bug uses the same prepreg and
    footprints.
- **Consequence:** a small fixed detuning of AN596's Option B match and of the patch match, minor next to M1.
- **Proposal:** no action now; C19 is the rematch and S11 port. If the first article misses, retune the BOM before any respin
  (populate C19, trim L1, L3/L7, L4/L8), measured end to end (S11 at C19 with C20 lifted; gain from C19 to C26 pad 1 with C26
  lifted). If In1 cutouts are ever made: only under signal lands (both lands of C20, C21, C22, C24, C26, L3, L4, L7, L8; the
  RF land of L1, L2, L5, L6; C19.1 and D6.2), never under GND lands, keeping the stitching vias, and sized to about the land
  outline (a cut out to the 0.2 mm gap gives about 55 Ω, a slight overcorrection). In2 is solid under every land.
- **Prior:** none.

### N3. R7/R8/R9 sit at the receiver end, so the host-driven J3 lines run 20–45 mm on In2 before their resistor — note

- **Where:** Net-(J3-Pad1) 20.22 mm, Net-(J3-Pad2) 21.09 mm, Net-(J3-Pad5) 45.11 mm, all 0.127 mm on In2, with vias in J3 pads
  1, 2 and 5. R7, R8 and R9 sit 0.555–0.789 mm from their U1 pads.
- **Evidence:**
  - Host-driven: J3.1 (GPIO4 → R7 → RXD) and J3.5 (GPIO2 → R9 → RXD2). J3.2 is U1's TXD through R8, which is correctly
    placed at the source.
  - The three lines run as a bundle at 0.227 mm pitch (the vertical run, Pad1/Pad5) to 0.325 mm (the Pad1/Pad2 diagonal),
    16–19 mm in parallel, ordered Pad2 / Pad1 / Pad5, 3.17–3.73 mm from the feed axis (2.51 mm copper to the feed's In2
    pad). The closest approach to H2's In2 pad is 0.100 mm edge to edge, which passes the Default 0.1 mm.
  - In1 and In3 are 93–98 % covered within 0.3 mm of each track (gaps only at the end antipads): stripline of about 49 Ω,
    0.14–0.31 ns one way (estimate), electrically short against nanosecond edges.
  - VSS reaches GND only through FL1 pins 4 → 3, so the Space Bug's L12 mechanism (returns through the common-mode choke)
    does apply in part; the solid In1/In3 stripline is what makes it benign. The P4's default drive is 20 mA (DS v1.2 pin
    note 3; 40 mA only at PAD_DRIVER 3).
  - Crosstalk from J3.1 onto J3.2 (held through 1 kΩ) is about 0.3–0.45 V (estimate), inside the P4's thresholds, and only
    while the host transmits.
  - The SAM carrier had its resistors 1.6–3.2 mm from J3 (its prefab review 09-02 line 175).
- **Proposal:** no action on its own. If S1 makes pin 5 PPS out, its series resistor belongs at the source, beside U1.4 and
  R1, as R8 is for TXD, and the 45 mm loop past H2 goes away (S1). Moving R7 to J3 is optional; the back-feed current does
  not depend on where the 1 kΩ sits along the line (S2 concerns its value).
- **Prior:** Space Bug L12.

### N4. The host signal lines have only 1 kΩ in series and no ESD clamp — note

- **Evidence:** each J3 signal net has two nodes (the J3 pad and the resistor); D2 clamps only VSYS–VSS. The SAM carrier, both
  the current V3 and the V2 that flew in the V9 nose cone on 2026-08-29 (tag `gnss-sam10m8-18mm-hv-v2.1.0`), has the same
  posture, and the Mantis J1 GNSS lines run bare to the P4 (its schematic review 2026-09-21 line 355). PX-DS p13 says only
  "ESD sensitive device"; the datasheet has no HBM, CDM or IEC figure. Estimate: a 2 kV HBM event peaks at 1.33 A with
  nothing in series and 0.80 A through 1.5 kΩ + 1 kΩ (about 1.67× more HBM margin), but an IEC 61000-4-2 gun discharge is a
  different class (8 kV: about 24 A falling to about 6 A). R7–R9 sit at U1, 18–20 mm from J3; the J3-side runs are on In2
  between GND layers.
- **Proposal:** no change, for parity with the flown SAM carrier; handle the bare carrier with ESD precautions (a strap; plug
  the harness with the host off and both boards at the same potential). If clamps are ever wanted: a 3–4-channel TVS array
  (VRWM ≥ 3.3 V) at the J3 pads on J3.1, J3.2 and J3.5, keeping the 1 kΩ resistors at U1 (do not move them to J3), and
  returning the array to the connector ground side, since carrier GND reaches J3.4 only through FL1 (the SAM prefab review's
  B1 is the precedent).
- **Prior:** none.

### N5. A shorted U2 pass element destroys the carrier but stays bounded at the host — note

- **Evidence:**
  - +3V3 rises to VSYS − 0.1 V (6.2–8.3 V). Over their limits: U1's VCC and V_BCKP (3.6 V, PX-DS p6), U5's VIN/EN (6 V,
    SNVS798Q p4), and U3/U4 if U5 passes it on (VCC 3.6 V, PON VCC + 0.3 V, BGA855N6 Table 2).
  - P4 injection through the 1 kΩ R7/R8/R9 (estimate): 4.4 mA into a clamp, 5.0 mA into a driver held high, 8.3 mA into one
    held low (IOL 28 mA, Table 5-4). The worst case into the 3.3 V rail is about 14.4 mA, below the P4's own draw of at
    least 26–35 mA (Table 5-7) and 45× below the ±200 mA latch-up trigger (Table 5-10). With S2's 10 kΩ on R7/R9 those two
    lines carry 0.44–0.83 mA; R8 stays at 4.4 mA.
  - If U1 or U5 then fails to a hard short: 4.3–9.1 A. The carrier's FL1, at 13–28× its 320 mA rating (5.8–16.6 W per
    winding), is the expected fuse. The Mantis's own FL1 (BLM18PG471SN1D, the same part as the carrier's FL2) carries the same
    current at 3.8–8.3 W, and U27 (3 A, no current limit) thermal-cycles. VBAT_CON sags to 5.4–7.3 V, so on a nearly flat
    pack a short lasting more than about 0.11 s can trip U19's UVLO (6.34 V) and brown out the Mantis.
  - Faults downstream of an intact U2 are contained by its 250–460 mA current limit and 150 °C thermal shutdown (Rev H
    Table 1).
- **Proposal:** no action beyond S2. Record it as the accepted residual risk of the linear regulator; a buck's high-side
  switch has the same failure mode.
- **Prior:** none.

---

## 8. Checked and fine

**Files, gates and rules**

- **Committed state:** `git status` clean at `c942cfd1`; live and scratch md5s identical; `.kicad_prl`, the `*-backups/` zips
  and lock files correctly ignored.
- **Gates:** DRC with parity 0 errors, 0 unconnected, 0 parity (7 warnings); `check_board_parity.py` links all 49 symbols;
  ERC has no power-pin-not-driven, no dangling wires and no symbol mismatch (its only errors are U1 pin 18, S4).
- **Libraries:** all 9 project footprints, 8 `Custom:` symbols and 8 3D models (including the GVLB208.A STEP and its
  `build.py`) are tracked; board copies equal the library for AE1, U1, U3, U4, FL1, FL2, FL4, FL5, D6 and J3. CI's parity
  check discovers the board automatically.
- **`.kicad_dru`:** committed and not load-bearing (DRC is the same without it); raising its minimum to 0.30 mm fires the
  TSNP rule 13 times, so it parses and matches U3/U4.
- **Rule floors (Space Bug D3 does not recur):** minimum track 0.09, clearance 0.09, copper to edge 0.2, hole clearance 0.1,
  hole to hole 0.2, via 0.4/0.3, annular 0.05 mm; Default 0.09 track / 0.1 clearance; RF 0.16 / 0.2 mm, equal to the drawn
  width.
- **Revision and silk:** both title blocks "PX1105R GNSS carrier, LDO supply", rev V1, 2026-10-08; `${REVISION}` resolves to
  V1; the `.gbrjob` says V1; no tag yet, consistent with board-versioning line 35. The back silk (y 27.32–31.68) sits outside
  the patch and stays readable when assembled (SAM 08-04 B6).

**Stackup, outline and fab output**

- **Stackup:** JLC 3313: F.Cu 0.035 / 3313 0.0994 (εr 4.05) / In1 / core 0.55 (4.38) / In2 / 2116 0.1088 (4.16) / In3 / core
  0.55 / In4 / 3313 0.0994 / B.Cu; 1.5994 mm; ENIG; fill and cap set; tenting both sides. The `.gbrjob` agrees (6 layers,
  1.5994 mm, ENIG).
- **Outputs:** `gerbers/` holds only `.gitkeep` per the README tag convention. `plot_gerbers.sh` produces 29 gerbers, a
  `.gbrjob` and one drill file: 146 × 0.300, 1 × 1.000 and 4 × 2.200 mm, all plated, no NPTH.
- **Outline and holes:** 22.00 × 30.00 mm with 1 mm corner arcs. H1–H4 2.2 mm plated with Ø3.8 GND pads, solid (as on the SAM
  carrier, B4); hole edge to other-net copper at least 0.9 mm on every layer. AE1's 1.0 mm hole per GV-DS p9–10 (ring: S5).
- **Vias and holes against JLC:** 146 vias, all 0.40/0.30 (JLC minimum 0.15/0.25); hole to hole at least 0.218 mm (same
  net) and 0.284 mm (different nets), with one pair under 0.25 mm (Space Bug L15 resolved); minimum drawn track 0.10 mm.

**Planes, stitching and assembly**

- **Courtyards:** present on every footprint, including U3/U4 for the `.kicad_dru` rule (Space Bug S12); no overlaps; the
  smallest pad-to-pad gap between parts is 0.30 mm (C24–C23).
- **Mask webs:** at least 0.196 mm between separate openings (U1.1/U1.2, FL1.1/FL1.4), apart from the deliberate U3/U4
  windows; JLC's bridge minimum is 0.10 mm. No other-net copper inside any opening at nominal (the opening-to-pour margin is
  L5).
- **Ground planes:** In1, In3 and In4 each one 625.9 mm² piece (95 % of the board; voids are antipads and the feed
  keep-out); In2 597.3 mm² GND plus 89 mm of signal; B.Cu 604.2 mm², one piece; F.Cu one 489.7 mm² piece plus small islands
  at U3.4, U4.1 and U4.4, each with its own via. Island removal is set to always; every fill piece holds a GND via or pad;
  a scratch refill differs only in vertex order (SAM 09-02 B3).
- **Stitching:** 129 GND vias (19.6 per cm²); the worst point is 5.5 mm from a GND via and the largest perimeter gap 5.1 mm,
  well under λ/10 at L1 in FR-4 (about 9 mm), with four solid inner GND layers.
- **Paste, other than U3/U4 and U2's EP:** no B.Paste (one top stencil). U1 prints 100 % of its 0.7 × 1.0 mm pads (Space Bug
  S8 fixed), area ratio 2.06 at 100 µm and 2.57 at 80 µm; FL4/FL5 and D6 1:1; FL1's four polygons 89–91 % (SAM 09-02 S6);
  every other aperture ≥ 1.55 at 100 µm. Every SMD pad gets paste (C19: D7).
- **Single-sided SMT:** all 48 SMD footprints on F; AE1 is the only B-side part (through-hole, no paste), fitted after
  reflow. U1's castellations are reworkable with an iron (Space Bug N12).
- **Acid traps:** none acute; the only non-orthogonal junctions are a 45° T on the B.Cu +1V8 run and one inside C25.1's pad.

**Antenna land and placement**

- **Feed keep-outs (Space Bug L2 fixed):** rule areas Ø3.5 mm on F.Cu and Ø3.0 mm on B.Cu and In1–In4, centered on the feed
  (GV-DS rev B p10 §4.3). Measured: the F.Cu pad has a 0.65 mm gap to GND, the inner and B pads 0.90 mm. The nearest other-net
  copper: In2 J3-Pad5 at r 3.11 mm, B.Cu +3V3 at 4.91 mm. A ring of 16–17 GND vias at r 2.03–2.18 mm, plus 3 within 3 mm,
  with no angular gap over 38°.
- **Orientation:** the library front view puts the feed at (0, +1.2) with the chamfer top-right (GV-DS p6 top view); the
  flipped board copy puts the B.Silk chamfer bottom-right (the p6 bottom view); the hole forces the orientation; the STEP
  model is 20 × 20 mm, centered.
- **Placement:** the patch is centered on the board (0.04/0.02 mm) and inside the outline; J3 is top-entry on F, so the
  patch faces away from the host, as on the SAM carrier.

**RF chain**

- **Connectivity:** AE1 → C20/L1 → C21 → FL4 → L2/C22 → L3 → U3 → L4 → FL5/L5 → L6/C24 → L7 → U4 → L8 → C26 → U1.11, with
  FL4/FL5 pads 2/3/5, U3/U4 pins 1/4, L1/L2/L5/L6 pin 2, C19.2 and D6.1 on GND. No ERC entries on any RF part.
- **Identical to the Space Bug and to AN596 Option B:** 0 pin differences over the front end; every value and MPN the same
  (33 pF GRM1555C1H330JA01D, 1 nF GRM1555C1H102JA01D, LQW15AN 5.1/9.1/2.4 nH, B39162B8389P810, TPD1E0B04DPYR,
  BGA855N6E6327XTSA1, LP5907MFX-1.8, CL05A105KO5NNNC); only the antenna changed. C22/C24 1 nF, L3/L7 9.1 nH to AI, L4/L8
  2.4 nH at AO, C23/C25 1 nF, VCC 1.8 V (AN596 Rev 1.3 Fig. 4, Table 4).
- **LNA pins and footprint:** Custom:BGA855N6 pins 1 GND, 2 VCC, 3 AO, 4 GND, 5 AI, 6 PON (DS Rev 2.2 Table 1); the six
  0.25 mm pads are DS Fig. 3 rotated 180°, not mirrored, and match Fig. 4; identical to KiCad 10's stock footprint apart from
  the model path. Single NSMD window, six Ø0.25 apertures, courtyard ±0.6 × ±0.8 mm (Space Bug S10 fixed for the LNAs).
- **DC paths:** U3/U4 AI see only L3–C22 / L7–C24 (AI abs max 0.9 V, no internal block); AO is internally blocked (DS
  Fig. 1) and U4's AO is also kept off U1's 3.3 V bias by C26 (AO abs max 2.1 V). Every SAW port sits at 0 V DC (B8389 p9
  "blocking capacitors are mandatory").
- **SAWs:** pad 1 in, 4 out, 2/3/5 GND, matching Fig. 2; FL4.1 faces the antenna, FL5.4 U4; shunts per Fig. 3: L2, L5, L6
  5.1 nH at FL4.4, FL5.1, FL5.4, and L1 5.1 nH through C21 at FL4.1.
- **ESD path:** D6 at the feed, then C20 33 pF, L1 5.1 nH shunt, C21 33 pF (B8389 p17 Fig. 13). D6 0.15 pF typ (0.009 dB at
  L1), clamp 10.1 V at 5 A TLP, IEC ±8 kV contact, VRWM ±3.6 V against 0 V DC on the feed. C20–L1 resonates at 388 MHz;
  high-pass attenuation 5.8 dB at 433 MHz, 2.3 at 868, 2.1 at 915 (calc).
- **RF_IN:** {U1.11, C26.2} only, 1.09 mm of 0.16 mm track leading away from the module (no RF under U1, PX-DS p13); C26
  (33 pF C0G) 0.465 mm from pad 11 blocks the 3.3 V antenna bias (PX-DS p6); U1.10/U1.12 0.40 mm away, each with a GND via in
  the pad.
- **LNA supply:** PON tied to VCC (+1V8), inside the 0.8 V–VCC window (DS Table 4); LP5907-1.8 ±2 % (1.764–1.836 V) inside
  the BGA855N6's 1.1–3.3 V; U5 EN tied to IN; LNA load 11.6 mA max against 250 mA; U5 dissipates about 15 mW.
- **Gain and coverage:** port to RF_IN 27.7 dB (L1) and 32.6 dB (L5) typical, at most about 35.4 dB, under 40 dB (PX-DS p12).
  Compressing U3 needs about +10 dBm at the port at 915 MHz or +12 dBm at 2.44 GHz. B8389 PB1 (1166.22–1186.68 MHz) covers
  L5/E5a/B2a; PB2 (1559–1607 MHz) covers L1/E1/B1I/B1C and G1, every band the PX1105R tracks.
- **Stability basis:** BGA855N6 k > 1 from 20 MHz to 10 GHz "guaranteed by device design" (DS Table 4); AN596 Option B k > 1
  to 10 GHz (minimum about 1.1–1.2 near 1–2 GHz); 21–22 dB reverse isolation per stage.
- **Line impedance:** 14 RF nets, 23.51 mm in 42 segments, all 0.16 mm on F.Cu, no vias, no sharp bends; CPWG gap 0.200 mm
  minimum, 0.263 mm median; about 48.5–51.2 Ω bare and 47.6–48.7 Ω under mask (Wadell with thickness correction and a 2-D
  solve); 45.5–54.0 Ω over ±10 % prepreg.
- **Reference plane (Space Bug L5 fixed):** In1 under the RF copper has 0.001 mm² of void outside the designed feed
  keep-out; no RF track crosses another part's pads or courtyard; U5's pads are 3.11 mm from the nearest RF copper.
- **Fencing (Space Bug L4 fixed):** 45 GND vias within 1 mm of the RF centerlines; every RF point within 1.66 mm of a GND via;
  vias in pad on L1, L2, L5, L6, C19, D6 and C23; U4.4's via 0.184 mm away (drill overlaps the pad). U3.1 is the exception
  (L3).
- **Isolation by coupling:** U3 AI is 3.12 mm from U4 AO and 3.64 mm from RF_IN; the tightest pair (U3's output against its
  input side, 0.325–0.355 mm) couples about −55 to −60 dB at L1 against 15.7–17.6 dB stage gain. The supply path is L4.
- **Tuning and test access:** C19 (DNP shunt), C20 and L1 form a constrained pi (GV-DS p11); C19's pads are the S11 port
  (Space Bug M2); FL2 is the lift point to feed +3V3 from a bench supply.

**Power**

- **Entry:** J3.3 VSYS → D2.1, FL1.1; J3.4 VSS → D2.2, FL1.4; FL1.2 → U2 VIN/EN; FL1.3 → GND. FL1 is oriented as a
  common-mode choke (Murata DLW21S dots at 1 and 4) at 39 % of its 320 mA rating, at most 78 mV drop; board GND sits at most
  about 45 mV above host ground. Same as the SAM carrier.
- **D2 SMF10A (SAM 08-04 B1):** VRWM 10 V, VBR ≥ 11.1 V, so it never conducts at 8.4 V; clamp ≤ 17 V against U2's 44 V and
  FL1's 20 V; on the connector side of FL1; cathode (pad 1) on VSYS, silk closed end on pad 1. The pack's hot-plug ring
  cannot reach J3 (R82 holds U27 off until GPS_ACT).
- **FL2 and +3V3 at U1:** BLM18PG471SN1D, 1 A, ≤ 0.2 Ω (≤ 25 mV at 125 mA); SENSE tied to VOUT ahead of FL2; 3.22–3.36 V at
  U1, inside 3.0–3.6 V; the FL2/C1 resonance (about 30–58 kHz, Q 1–3) rings about 10–18 mV for a 20 mA step, under 50 mVpp.
- **U2 (ADP7142ARDZ-3.3):** pins match Rev H Table 5; EN tied to VIN by a 2.54 mm In2 strap between vias in U2.5/U2.7 (SAM
  09-02 B1); C7 2.2 nF gives a 1.53 ms soft start (Rev H Eq. 8); inrush 172–222 mA including the load, under the 250 mA
  minimum current limit; U2 outside U1's courtyard (SAM 09-02 C1).
- **U2 capacitors:** C136 about 3.9 µF at 8.4 V (2.65 worst) and C135 about 10.4 µF at 3.3 V (7.1 worst), above CMIN 1.5 µF;
  ESR inside ADI's window; 16 V parts (C7 50 V; RF caps C0G 50 V). The input network's Q is about 1.
- **Headroom:** dropout about 0.14 V typical at 125 mA, so regulation holds down to about 3.6–3.7 V of VBATT, below the
  Mantis firing-sag analysis's 3.8 V floor for buck carriers.
- **Decoupling distances:** C136–U2.8 0.51 mm, C3 0.81–0.84, C135–U2.3 0.54, C7–U2.6 0.46, C27–U5.1 0.41, C28–U5.5 0.50 mm.
- **Routing drops:** +3V3 on B.Cu about 26 mΩ and +1V8 about 34 mΩ at about 10 mA; under 0.5 mV.
- **U2 exposed pad (Space Bug L7 fixed):** six GND vias in the land, solid SMD connection to the 490 mm² F.Cu GND and all five
  other GND layers; filled and capped declared.
- **Hot-plug:** C136 charging through the cable rings to at most about 10 V, below D2's 11.1 V; normal power-up is soft (U27
  CT 10 nF, then U2's soft start).
- **Sequencing and RSTN (SAM 08-04 C5):** +3V3 rises monotonically in about 1.5 ms from a UVLO ≤ 2.69 V; RSTN pulled to the
  receiver's own rail by R2 10 kΩ, no capacitor, which suits the datasheet (needed only for a very slow rise); V_BCKP is on
  whenever VCC is (p5).
- **LEDs and budget:** D1/D3 about 70–90 µA through 10 kΩ (R3 kept by the owner); +3V3 about 125 / 105 mA; JST SH 1 A per
  contact. No switching regulator on the board (owner decision).

**Receiver and host interface**

- **U1 pins:** GND pins 1/10/12 each with a GND via dead center plus a second within 0.27–0.49 mm (PX-DS p13); TXD → R8 →
  J3.2 → P4 GPIO3 (FC RX) and RXD ← R7 ← J3.1 ← GPIO4 (FC TX), matching `board_v9.h`; pins 5, 7, 13, 14, 16, 17 open per the
  p7 application circuit; pins 6/8 on +3V3; pin 18 open per p6 (owner decision; the flag is S4).
- **U1 symbol and land:** pins numbered as DS p5/p6; 18 pads 1.0 × 0.7 mm at 1.1 mm pitch, columns 9.70 mm apart, counter-
  clockwise from the top, not mirrored; mask +0.102 per pad (opening margin: L5); CPL offset about 0.067 mm; F.Cu under the
  body about 90 % GND with no signal track; In1/In3 solid.
- **Logic levels:** P4 VOH ≥ 2.64 V and VOL ≤ 0.33 V against the PX1105R's VIH 2.0 V and VIL 0.8 V; PX VOL 0.4 V against P4
  VIL 0.825 V; 1 kΩ × about 15 pF is about 15 ns against 8.7 µs per bit. TXD's high is M8.
- **J3 against Mantis J1 and the SAM carrier (Space Bug S2 fixed):** pins 1–4 and tabs 6/7 identical; tabs on GND with vias in
  the pads; straight A-type jumper per cables.md and Mantis FAB B10; the land matches JST's SH drawing rotated 180°, not
  mirrored; the "1" silk sits beside pad 1 (SAM 09-02 S5; its height is L1's). Nothing inside J3's courtyard.
- **Holes against the host:** the carrier's 16.75 × 25.00 mm pattern matches the stretched Mantis H2/H4/H6/H8 (V11; K4).
- **Host behavior:** R82 holds GPS_ACT low while the P4 is unpowered or in reset, so the carrier stays off; GPIO2's reset
  pull-up keeps RXD2 high on the Mantis (at most about 70 µA through R9 into a dark carrier). Today's UBX traffic is ignored
  by the PX1105R; probe contention is limited by R8.
- **Polarity marks:** D2's closed end on pad 1 (VSYS, cathode); D1/D3 dot at pad 1 = cathode on GND (XL-1005UGC p7); U5's
  triangle at pin 1; FL4/FL5 dots at pad 1 (input); FL1's circle beside pad 1 (no polarity); D6 bidirectional.
- **BOM content:** every MPN matches its value, package and voltage; every fitted symbol has MPN and Mfr; H1–H4 excluded
  from the BOM; C19's DNP flags consistent and `--exclude-dnp` drops it.
- **Contested checks.** The regression pass marked SAM 08-04 C7 (host power gating) fine because firmware raises GPS_ACT
  before the UART; S2 shows that USB-only power and VBATT interruptions still leave the carrier dark with the UART driven.
  It marked SAM 09-02 B8 (fixed-variant regulator) fine on the MPN and Description; D2 shows the guard does not reach a
  release file.

**Host faults, flight loads, receiver configuration and test access (gap passes)**

- **Supply chain traced** in both netlists (carrier VSYS/VSS/GND/EN nets; Mantis J1.3 ← FL1 ← U27 ← VBATT ← U19 ← VBAT_CON,
  with R48 127 Ω, C50 10 nF, C46 2.2 nF and the R44/R45/C94 UVLO network).
- **Faults downstream of U2 are bounded** by its 250–460 mA limit and 150 °C shutdown: the host sees at most 0.46 A, and the
  carrier's FL1 at most 1.44× its rating, only while U2 cycles. This includes C135, 0.53 mm from the edge.
- **Normal load against ratings:** JST SH 1 A; Mantis FL1 1 A (≤ 25 mV); U27 3 A (13 mV, 1.6 mW); U19 11.6 A; carrier FL1 40 %.
  GPS_ACT turn-on inrush 23–128 mA into C136 (U27 rise about 1.3 ms), then about 43 mA during the LDO soft start.
- **Firing sag:** the branch floor (about 3.3–3.6 V) sits more than 2.4 V below the tired-pack sag minimum (about 6.0 V); the
  deglitch arithmetic reproduces the parity doc's 269 ms. C3 (0402 X7R) is 5.55 mm from H3: low flex exposure.
- **Flight loads** from the four 2026-07-05 logs (high-g channel at about 963 Hz, no railed samples): boost 5.2–9.4 g peaks,
  ignition rise 155–231 ms; ejection 38.6–118 g; snatch 27.5–34.9 g; landing 53.6–146.4 g. The patch's mass is centered on
  the hole pattern (0.215 / 0.03 mm). With the patch bonded the response to these events is quasi-static; U1's castellated
  joints carry about 0.14 N each at 146 g; board strain stays at or below about 350 µε; each M2 hole carries about 7.4 N.
  Random-vibration oscillator phase noise is 0.4–1.3° rms. (K6 covers the faster archive logs.)
- **Receiver checks:** the 1560/1600 MHz harmonic arithmetic agrees with Space Bug M1, and 40 MHz-multiple lines miss the
  L1 C/A, E1, B1C and L5/E5a/B2a main lobes; the TinkerNav 0x20 NACK is about GLONASS, not a malformed frame; constellation
  selection works and reads back; all eight navigation modes ACK; 1 kΩ on the UART lines is fine at 460800 (tens of ns
  against 2.17 µs); the receiver offers an interference monitor (AN0037 p70, p109).
- **Test access:** pad connection THT_THERMAL means SMD pads are solid in the pour; the access pads carry the intended nets;
  S2's back-feed is measurable across R7/R8/R9 without rework; the feed node and C26.1 carry no DC; C19's footprint works as
  a port or a terminator; every F-side thermal site stays reachable after the AE1 fit; BOOT_SEL recovery remains possible
  without TP1 (U1.18's land protrudes 0.50 mm beyond the body, with H2's GND pad 1.23 mm away); 1PPS is probeable at R1.1
  (44.75, 37.10) with D1.1's GND 1.01 mm away; the owner's rig and scripts cover the RF items (M19), and the bench-injection
  limit re-derives (M7).

---

## 9. Not covered, or not checkable from the files

- **Thermal:** the PX1105R's maximum current and its L1/L5 split (only typical 115/95 mA is given); real board-to-air
  transfer in the bay (air temperature, airflow, the Mantis's heat, sun); ADP7142 ψJT. Only M2's soak settles it. The 3M 9077
  adhesive's temperature rating is not in GV-DS.
- **RF performance on this board:** the patch's resonance, gain and axial ratio on the 22 × 30 mm plane and installed on the
  Mantis (M1); B8389 and BGA855N6 S-parameters on this stack, so the exact match, per-stage µ and cascade stability margin;
  end-to-end isolation and the BGA855N6's internal VCC choke reactance (L4); LQW15AN Q and GRM1555 ESL at L-band; the L-band
  noise on +3V3, +1V8 and VSYS and how much reaches the feed (L2); the PX1105R's own NF, minimum gain, RF_IN out-of-band
  tolerance, bias short protection and antenna-open behavior; JLC's as-built prepreg and mask thickness and permittivity.
- **Interference levels:** the real coupled levels of the host's BLE, LoRa and clock harmonics at the patch port (M3, M4,
  M6), which depend on the stack geometry; the IEC 61000-4-2 robustness of D6 and the high-pass protecting the 250 V SAWs
  (RF360's ESD report was not obtained).
- **Mechanical and installation:** which side faces the host, the standoff material and height, the screws, what sits near
  the patch, the airframe inside diameter and the boresight (K4); shock and vibration levels at the carrier's location
  beyond the logs, and transmissibility from the FC's IMU to the carrier; the 3M 9077's strength on solder mask and its
  dynamic modulus; a Taoglas shock rating; the JST SH withdrawal force; the jumper's gauge and length (inferred from the part
  number); the GVLB208.A's real pin length and Taoglas's reference-board thickness; Steinberg's constants and the AEC-Q200
  flex magnitude (quoted as commonly cited); whether future motors exceed the logged loads.
- **Receiver internals:** pin-1 orientation of the real module relative to its shield dot (L1); internal pulls, thresholds,
  clamp limits, TXD drive and ESD rating; RSTN pull-up and POR/BOR thresholds and the off-time for a clean reset; behavior
  when started from a pre-biased +3V3 (S2); what keeps running with RSTN low (M16); the internal TCXO's part number,
  g-sensitivity and mounting (M10) and the tracking-loop design; whether flash settings survive VCC + V_BCKP removal, the
  flash commit time, endurance and record protection (F11); whether the build lacks GLONASS or only masks it, and what
  build future units ship with (F10); which AN0037 messages the firmware supports (PX-DS lists none), the 0x64/0x3B interval
  unit, and whether binary 0xA8 works in rover mode; whether a rover drops GLONASS RTCM silently; whether the ROM fallback
  accepts an image without BOOT_SEL; the actual NMEA epoch size at 10 Hz on real sky.
- **Host:** the P4 GPIO clamp resistance and injection rating (only the JESD78 ±200 mA latch-up qualification is published),
  so F2's 0.6–2.6 A is an estimate; which part opens first at 5–23 A (no fusing I²t or pulse ratings for the BLM18PG,
  DLW21SN or TPS22810; the TPS22810's pulse rating was deleted in Rev A); the TPS25982's time to thermal shutdown in current
  limit; whether metal hardware bonds the carrier ground to Mantis GND (K2, K4); DLW21SN behavior under unbalanced DC; the
  P4 silicon revision on each Mantis; the Mantis's own pad current and the flight pack's capacity and resistance (the
  0.15–0.18 Ω comes from pre-Mantis logs).
- **Statistics:** whether a C136 crack is bolted or resistive (L9); tombstoning and EP voiding, which depend on the reflow
  profile (M9, S3).
- **Fab and assembly:** who assembles and with which foil and paste; JLC's handling of the 0.025 mm mask slivers (L5), the
  0.15–0.18 mm hole-to-copper spacing and the 0.2 mm inner-plane edge clearance (D11), and the 0.10 mm AE1 ring (S5);
  whether fill and cap are actually selected; whether JLC runs a custom profile with a 240 °C peak at U1; the BGA855N6's MSL
  and reflow limits (not in DS Rev 2.2); global-sourcing availability and price of B39162B8389P810, BGA855N6, PX1105R,
  GVLB208.A and ADP7142ARDZ-3.3 (LCSC stock is a 2026-10-12 snapshot); whether Space Bug V2 leftovers exist; the lifecycle of
  LQW15AN9N1G00D (distributors disagree); Murata's land patterns for DLW21SN670HQ2L and BLM18PG471SN1D and ADI's land for the
  ADP7142 signal pads (checked against outlines and built boards only); the PX1105R's real pad depth (the drawings are not to
  scale); the internal geometry of the vendor STEP models; the 84 off-grid ERC entries one by one; the exact IPC-A-610 fillet,
  fill and voiding numbers (they need the owner's licensed copy).
- **Owner equipment and mission numbers:** whether a VNA, oscilloscope, thermocouple logger, IR camera, near-field probe,
  X-ray or CT is available (M17); the absolute cascade NF (no calibrated noise source); whether the PX1105R acquires from an
  L5-only injection; the maximum bay ambient the thermal pass criteria depend on; the HackRF's maximum L-band TX power (from
  general knowledge).
- **Process:** the live project was open in KiCad during the review (lock files dated 06:46, after the 05:49 commit); I
  reviewed only the committed `c942cfd1`. Which function the owner intends for J3 pin 5 (S1) is the owner's to settle. The
  SAM field review's §6 item 9 (a CI check of the copper-layer count against the `.gbrjob`) is still not implemented; this
  board's `.gbrjob` correctly says 6 layers.

---

## Review method and verification

- **Reviewers:** nine specialist passes (power, RF schematic, RF layout, receiver, host system, layout/DFM, footprints,
  release, regression against the Space Bug and SAM-carrier reviews) and four gap passes (host faults, flight loads, receiver
  configuration, first-article test), all working from the same scratch exports, plus three datasheet librarians whose notes
  the others used before going to the web.
- **Findings:** the nine specialists raised 97 findings, merged to 45; the gap passes added 25; 70 were kept and none dropped.
- **Verification:** an independent verifier re-derived every one of the 70 from the files and datasheets: 37 confirmed as
  stated, 33 confirmed with corrected details, none refuted. Ten (S1, K1, D1, D2, D3, M1, M2, M5, F2, F4) also got an impact
  check and a skeptic pass. Where a verifier corrected a number this report uses the corrected value, and where a verifier
  judged a proposal wrong or incomplete (36 of the 70) the report uses the verifier's version.
- **Severity changes after verification:** D3, F2, F4 and M5 major → minor; K4 note → minor; D7, D8, F7, F11, M10, M11, M12,
  M13, M14, M15 minor → note. K1 and D2 stayed major after the verifier proposed minor and the impact check major.
- **Contested:** S1 only. The review was briefed from my session notes that pin 5 is PPS out. The skeptic accepted every
  fact and showed from the 2026-10-08 project backups that the RXD2 wiring was an owner edit in the GUI, and I re-derived
  the two backup netlists myself. S1 is therefore a decision for the owner, not a major defect. Other skeptic passes refined findings
  without refuting them: K1 (a DMM check through the top-face pin, a solder run-through risk), D1 (the maker's 100 µm
  recommendation makes 100 µm marginal, not failing), D2 and D3 (LCSC does list the -3.3 and the PX1105R, with no stock;
  two more out-of-stock lines), M1 (the host ground extends the plane in y only), M2 (the open-air case is optimistic, B11
  removes the metal-standoff lever), M5 (the per-unit screen), F2 (the 3.3 V tree, not just the pad, is at risk) and F4 (the
  owner had already written the core rule).
- **Dropped:** none.
- **Spot checks after the run:** I re-derived S1 from the netlist and the two backups, K1's pin length from GV-DS rev B p6
  (1.2 ± 0.2 mm, Ø0.8 ± 0.1), S5's padstack from the footprint (Ø1.0 hole, Ø1.2 pad) and D2's stray MP field in the
  schematic. All held.
