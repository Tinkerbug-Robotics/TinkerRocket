# rocket-computer-mini — design and first-article review, 2026-09-21

Review of the fabricated `rocket-computer-mini` (V1, tag `rocket-computer-mini-v1.0.1`)
against its schematic, its layout and the parts' datasheets, prompted by the first
article: solder-paste problems during assembly, and a board that has never enumerated
on USB.

## What this review is, and what it is not

Twenty-one reviewers were run over the two boards. Seventeen finished; the adversarial
verification pass, the completeness critic and the report writers did not, because the
run reached an account spend limit. **Almost every finding below is therefore
single-source and unverified.** Nine of them I re-derived myself from the board file and
the plotted gerbers, and those are marked *Verified here*. Treat the rest as leads with
their evidence attached, not as settled defects. Confidence is the reviewer's own number.

Nothing in the repository was modified. The board files were read from a copy; the
owner's live project was never opened.

### Method

| source | what it is |
|---|---|
| `rocket-computer-mini` at git HEAD `d3c1aa30` and in the owner's working copy | identical schematics; the board differs only by two silkscreen texts at S1 |
| tag `rocket-computer-mini-v1.0.1` | the fabricated board. Copper and placement identical to HEAD; only the module's pad 12 paste and 3D references differ |
| tag `rocket-computer-v9.0.0` | the fabricated, working full computer, used as the reference for every shared circuit |

Ground truth came from `kicad-cli` netlist, electrical-rule and design-rule exports, from
the board file through `pcbnew`, from the plotted paste and mask gerbers, and from the
parts' own datasheets. Pin maps were taken from the netlist exports, never from the board
file.

## The first article that will not enumerate

**The design is not the cause.** The whole path that produces the sink termination a host
looks for is passive: the connector's CC1 and CC2 pads, one 5.11 k resistor each, and
ground. I compared it against the fabricated, working full computer at three levels, and
it is identical at all three: the same netlist connections, the same connector footprint
with the same pad geometry, and the same resistor values. No regulator, no multiplexer, no
strapping pin and no firmware state sits in that path, so nothing powered can hide it.
Absence of termination in **both** plug orientations, on several ports, needs a common
physical cause.

The review found four mechanisms that fit, in the order I would test them:

1. **The connector's signal row is not wetted.** Its twelve signal pins are 0.30 mm wide
   on 0.5 mm pitch. The land pattern is identical to the working full computer's, and so
   are the apertures, but the stencil is not: this board uses the 0.08 mm foil chosen for
   the flash balls, against the 0.10 mm default the full computer got. Every joint on this
   connector therefore receives 20 percent less solder than the same joint on the board
   that works. Transfer is not the issue, since the area ratio is 1.49 against a floor of
   0.66. Absolute volume on a 0.30 mm tail is: about 14 nL of solder per pin, which leaves
   little tolerance for a lead that is not coplanar. One under-filled row loses both
   configuration pads at once, which matches the symptom better than any single-pad fault.
2. **Both pull-down resistors are open.** Each of them, and 68 other two-terminal parts,
   has one pad tied solid into a copper pour and the other on a thin track. That
   is textbook tombstone geometry, and with the thin foil there is less paste to hold the
   part down. A single open resistor would fail only one orientation; both failing is what
   the bench saw.
3. **The plug is not seating.** On the board as built the receptacle face sits 0.445 mm
   behind the board edge, where the working full computer has 0.225 mm. Contacts for the
   configuration channel mate second, so a plug that bottoms on the board edge before full
   insertion shows the host no termination while power and ground may still touch.
   **Already addressed:** the owner moved the connector 0.24 mm toward the edge on
   2026-09-21, bringing the recess to 0.205 mm, slightly tighter than the full computer's.
   That change is in the working copy, not in the board that was built.
4. **The cable or the port.** Cheapest to eliminate and not yet eliminated.

A separate fact that is not a fault, and that will otherwise waste bench time: with the
switch in the flight-computer position **the flight computer cannot appear on USB at all**
until the out computer is alive and has been commanded to raise its rail. That is the
designed power-on order, it matches the full computer, and neither processor has a reset
button. Bring up the out computer first.

The ordered probe sequence is in `first-article-bringup.md` beside this file.

**Two findings in this report were acted on while it was being written.** The owner moved
the connector toward the board edge, and deleted the 29 mm dead-end branch on the flight
computer's reset net. Both are in the working copy. Neither is in the fabricated board, so
both remain valid descriptions of the article on the bench.

## Findings


### USB, the mux and getting a processor to answer

#### 1. [Minor] J6's twelve 0.5 mm-pitch signal pins (CC1 = A5, CC2 = B5) get 0.028 mm3 of paste each from the 0.08 mm foil chosen for the WLCSP, while the four shell pads print 100 % (0.35 mm3 each) - the one DFM mechanism that opens both CC lines at once

- **Refs:** J6 (GCT USB4110GFA, F side), stencil B1
- **Nets:** Net-(J6-CC1), Net-(J6-CC2), D+, D-, VBUS, GND
- **Reviewer:** `mini-dfm-paste-5`, confidence 0.45
- **Verified here, and the framing corrected:** the geometry is right but "starved pins versus 100 % shell pads" overstates it. Both are 1:1, so both deposit the same *height*: about 40 um of solder at this foil. The shell pads hold 76 % of the connector's paste only because they are 12.6 times the area. What is true, and is the sharper point, is that this footprint is **identical to the fabricated, working V9's** down to the pad and aperture, and the V9 enumerates. The one paste difference between the two boards is the foil: 0.08 mm here against the repo default 0.10 mm there, so every J6 joint on the mini gets **20 % less solder than the same joint on the V9** (27.6 nL of paste per signal pin against 34.5 nL). Aperture area ratio is 1.49, far above the 0.66 floor, so transfer is not the problem; absolute volume on a 0.30 mm tail is.
- **First article:** possible

**Claim.** The flat 0.08 mm foil is set by U13/U33 (AR 0.79) and applies to the connector on the same side. The 0.30x1.15 mm pin apertures deposit 0.0276 mm3 (~0.014 mm3 of solder after reflow, ~0.04-0.05 mm joint height under a 0.3 mm-wide tail), against GCT's stated +/-0.05 mm drawing tolerance; the 2.18x2.00 shell pads print 100 % (4.36 mm2, 0.35 mm3) and dominate the connector's seating.

**Evidence.** paste gerber: J6 F flashes R0.300x1.150 x8 (AR 1.49), R0.600x1.150 x4, R2.180x2.000 x4 at (79.25/89.47, 164.29/168.22); mask RR0.40x1.25 at 0.5 pitch (web 0.105); no via in A5/B5; two NPTH 0.65 mm pegs unpasted. GCT usb4110 drawing text: 'Tolerance:+/-0.05mm', no coplanarity figure readable (image-only). B1: 'ONE FLAT 0.08 mm FOIL SERVES BOTH SIDES'. First article: no Rd seen on CC in either S1 position across several re-plugs (BRIEF).

**Consequence.** A connector seated on well-fed shell pads with starved 0.5 mm pins can leave the whole signal row - including both CC pads and the CC pull-down path - open or hairline; that matches 'no Rd in any orientation' better than any single-pad mechanism found here.

**Fix.** Inspect J6's A5/B5/A4/B4 fillets at 30-40 deg on the first article and probe CC1/CC2 to R41/R47 at the connector tail. Next spin: windowpane the shell pads to ~50-60 % and consider a step (0.10-0.12 mm) over J6, or over-size the pin apertures (0.30 -> 0.35 x 1.30) within the 0.105 mm mask web.

#### 2. [Minor] USB VBUS reaches the TPS2121 through ~52 mm of 0.4 mm 0.5 oz inner track (~0.15 ohm) with a single via at each end

- **Refs:** J6 (84.36,164.79), U21 pin 7 IN1 (81.085,123.74), vias (84.27,165.00) and (80.25,123.96)
- **Nets:** Net-(J6-VBUS)
- **Reviewer:** `mini-pcb-power-2`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** Both VBUS pad pairs of J6 funnel on F.Cu 0.4 mm tracks into one via at (84.27,165.00), then run on In5 (0.0152 mm copper) as a 0.4 mm track along the left edge (x=72.89 from y=163.11 to 128.23, then to (77.58,126.65) and (80.25,123.96)) and up through one via to a 0.4 mm B.Cu stub into U21.7. Total In5 length 56.7 mm; the series path is about 52 mm.

**Evidence.** tracks.txt: Net-(J6-VBUS) In5.Cu 9 segments 0.4 mm totalling 56.7 mm, F.Cu 9 segments 0.4 mm 8.7 mm, B.Cu 0.4 mm 0.9 mm; vias (84.27,165.00) F-In5, (77.37,165.68) F-In5, (74.32,164.54) F-In5, (77.58,126.65) In2-In5-B, (80.25,123.96) In5-B. Min-cut from J6.A4/B9 to U21.7 = 1 via ((80.25,123.96)). Stackup block: inner copper 0.0152 mm, outer 0.035 mm. R = 1.72e-8 x 0.052 / (0.4e-3 x 15.2e-6) = 0.147 ohm. V9 rocket-computer.kicad_pcb routes VBUS as 0.8 mm on In2 (21 segments) plus 0.4 mm F.Cu, i.e. twice the width on the same 0.5 oz inner copper.

**Consequence.** At the board's USB-only draw (0.23-0.53 A at 5 V for the 0.3-0.7 A rail scenarios in power-budget.md) the drop is 35-80 mV and the IPC-2221 internal-layer rise is 8-28 C; at 1 A it is 0.15 V and roughly 65 C. It shifts the TPS2121 priority/OV comparison point slightly and is the hottest copper on the board under USB power, but it is not a fault at the first-article currents.

**Fix.** Widen the In5 run to >=0.8 mm (the In5 GND fill borders it; the 0.225 mm edge clearance is already accepted), or duplicate it on In2, and add a second via at the J6 end and at the U21 end.

#### 3. [Minor] J6 receptacle face sits 0.47 mm behind the board edge (V9: 0.275 mm) — a plug that cannot seat fully never mates CC

- **Refs:** J6, FID2
- **Nets:** Net-(J6-CC1), Net-(J6-CC2), D+, D-
- **Reviewer:** `mini-pcb-usb-mcu-1`, confidence 0.4
- **Verification:** not run (single source).
- **First article:** possible

**Claim.** The USB4110 footprint's front (component-outline/silk line 6.28 mm from the peg centres, = GCT layout 'lower shell pad row + 2.85') lands at y = 164.79 + 6.28 = 171.07 mm; the board edge is y = 171.54 (Edge.Cuts line 73.49→93.99 at 171.54). The receptacle face is therefore recessed 0.47 mm behind the PCB edge. The fabbed, USB-working V9 uses the same footprint at the same coordinates with its edge at 171.345, i.e. 0.275 mm recess.

**Evidence.** pcbnew: J6 pos (84.36,164.79) rot 0, F.CrtYd to 171.32, F.SilkS front line at +6.28 (GCT_USB4110GFA.kicad_mod fp_line 6.28); Edge.Cuts bottom line y=171.54. V9 rocket-computer.kicad_pcb: J6 (84.36,164.79), board bbox bottom 171.345. GCT USB4110 drawing (pdftotext + rendered page, work/mini-pcb-usb-mcu/gct_layout2.png): shell-pad rows 3.93 apart, component outline 2.85 below the lower row, pegs Ø0.50±0.05 into Ø0.65 holes, peg protrusion 0.63; the drawing carries no PCB-edge line.

**Consequence.** On a top-mount Type-C the plug overmold must clear the PCB edge; with the face 0.47 mm inside the edge a plug with a short shell bottoms on the PCB before full insertion. CC and D± are second-mate contacts, so an under-inserted plug shows the host no Rd on CC — the exact first-article symptom — while VBUS/GND may still touch. The V9 works at 0.275 mm; the mini is 0.2 mm deeper.

**Fix.** On the built board: check whether the plug clicks home and whether its overmold touches the PCB edge; caliper the face-to-edge distance. Next spin: move J6 0.4–0.5 mm toward the edge so the face is flush to slightly proud (nothing is in the way — FID2 already sits in the courtyard by decision; the shell pads/pegs stay ≥0.2 mm from the edge).

#### 4. [Minor] FC_CHIP_PU carries a 29 mm dead-end branch on In5 to a removed driver

- **Refs:** U32, R110, C114
- **Nets:** FC_CHIP_PU
- **Reviewer:** `mini-pcb-usb-mcu-2`, confidence 0.95
- **Verification:** not run (single source).

**Claim.** The FC reset net has only three pads in the netlist (U32.4, R110.2, C114.1), all joined on F.Cu within 1.4 mm, but the board also routes a 29.0 mm In5 track from the via at (78.297,150.308) down the left side (73.49–73.58, 156.5–158.7), along y = 162.87 (77.8→82.6), across to (88.26,159.29), through a via to a 0.19 mm F.Cu tail ending at (88.392,159.157) on nothing (nearest item a GND via 0.57 mm away). It is the leftover of the OC-driven reset the README says was removed ('FC_CHIP_PU is back to its reset RC only'). Present identically on the fabbed V1.0.1 file.

**Evidence.** nets.py dump of FC_CHIP_PU: In5 segments totalling 29.01 mm (live and mini-v101), F.Cu (88.392,159.157)->(88.26,159.29), via (88.26,159.29); dump.py net FC_CHIP_PU = C114.1 R110.2 U32.4; epad.py neighbour scan at (88.39,159.16); README line 179.

**Consequence.** A 29 mm unterminated branch on the FC's high-impedance reset node (10 k / 1 µF), running 0.39 mm from the VBUS In5 trace for ~2 mm and across the pyro channel region. The 1 µF keeps it benign at AC, so it is hygiene rather than a bring-up cause, but it is noise pickup on a reset threshold and an unexplained feature for anyone probing the board.

**Fix.** Delete the In5/F.Cu branch from (78.297,150.308) to (88.392,159.157) and the two vias at (78.297,150.308) and (88.26,159.29).

#### 5. [Minor] VBUS is a 0.4 mm, 0.5 oz inner trace for 57 mm — fine for the budget's ≤0.5 A, not for 1.5 A

- **Refs:** J6, CR3, C76, U21
- **Nets:** Net-(J6-VBUS)
- **Reviewer:** `mini-pcb-usb-mcu-7`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** Both VBUS pad pairs join on F.Cu (0.4 mm, 1 oz) at one via (84.27,165.0); the run to U21 pin 7 is 56.7 mm of 0.4 mm track on In5 (15.2 µm copper) with single 0.3 mm vias at each layer change ((84.27,165.0), (77.37,165.68), (74.32,164.54), (77.58,126.65), (80.255,123.965)) and a 0.20 mm copper-to-edge run at x = 72.89 (accepted). Cross-section 0.0061 mm² (9.4 mil²): ≈0.16 Ω end to end; IPC-2221 internal-trace limits ≈0.34 A at 10 °C rise, 0.46 A at 20 °C, ~0.54 A at 30 °C.

**Evidence.** nets.py Net-(J6-VBUS): In5 segments 34.88+6.22+3.783+3.11+2.234+2.022+1.91+1.612+0.962 mm at w=0.4; stackup In5 copper 0.0152 mm; power-budget.md: pad idle + cap charging ≈260 mA; the sink is Rd-only (R41/R47 5.11 k, no CC reader) so it cannot claim more than USB default current.

**Consequence.** At the design's own draw (≤0.5 A) the trace is adequate with ~25 °C worst-case rise; the 1.5 A figure in the review brief is not supported by this trace (IPC-2221 predicts >100 °C) and is not something the board can request. The single vias are each rated well above 0.5 A.

**Fix.** No change needed for the current budget. If USB-only charging current is ever raised, widen the In5 run to ≥0.6 mm or parallel it on In2, and double the vias.

#### 6. [Note] Bring-up trap: with S1 in the F position the FC cannot appear on USB until the OC has been commanded (BLE cmd 8) to raise FC_EN_OC, and neither S3 has a reset button

- **Refs:** U32 CHIP_PU/R110/C114, SW2, U30, D9, R84, C105, S1, U1 HSD2
- **Nets:** FC_CHIP_PU, POWER_SWITCH, V_MCU_SWTCH, FC_D+, FC_D-
- **Reviewer:** `mini-fc-s3-4`, confidence 0.9
- **Verification:** not run (single source).
- **First article:** possible

**Claim.** FC_CHIP_PU is only the R110 10 k / C114 1 uF RC from V_MCU_SWTCH (identical to the OC's R36/C25 and the V9's R36/C25: no reset button on any of them). V_MCU_SWTCH is held off by R84 100 k until FC_EN_OC (OC GPIO7) or FC_EN_HOLD (FC GPIO17) lifts POWER_SWITCH through D9. The OC firmware starts with PWR_PIN LOW (main.cpp:9668-9669) and only raises it on the app's power-on command. So on a fresh board the sequence is forced: OC must enumerate and be flashed first (S1 = O), then the app must power the rail, and only then can the host see the FC (S1 = F). The FC's ROM download mode is reachable by USB-Serial-JTAG auto-reset, or by holding SW2 (GPIO0) while the OC cycles the rail; there is no other reset path.

**Evidence.** Netlist: FC_CHIP_PU: C114.1 R110.2 U32.4; POWER_SWITCH: C105.1 D9.3[K] R84.1 U30.5[EN/UVLO]; FC_EN_OC: D9.2 U15.12; FC_EN_HOLD: D9.1 U32.23; Net-(U32-GPIO0): SW2.2 U32.5 only. V9 netlist: Net-(U15-CHIP_PU): C25.1 R36.2 U15.4 (same shape). out_computer main.cpp:9668-9673 'Start with power rail OFF'. README 'The flight computer starts off'.

**Consequence.** Not a defect and identical to the working V9, but during the first article's bench session an FC that never enumerates is the EXPECTED result until the OC is alive and commanded; it must not be read as a dead FC. It cannot explain the observed no-Rd-on-CC symptom, which precedes any S3 involvement.

**Fix.** Add the forced order (OC first at S1=O; cmd 8; then S1=F) to FABRICATION-NOTES/bring-up notes. If a way to power the FC without a working OC is wanted for bring-up, that is a design discussion (e.g. a jumper pad on POWER_SWITCH), not a fix to draw.

#### 7. [Note] OC USB D+/D− pair routed asymmetrically (3 vias / In5 on D+, 2 vias on D−) over 22–24 mm to the mux

- **Refs:** U15 pins 25/26, R38, R39, U1 pins 9/10
- **Nets:** Net-(U15-GPIO20), OC_D+, Net-(U15-GPIO19), OC_D-
- **Reviewer:** `mini-oc-s3-7`, confidence 0.4
- **Verification:** not run (single source).

**Claim.** OC_D+ is 23.5 mm with 3 vias touching F.Cu/In2.Cu/In5.Cu; OC_D− is 22.4 mm with 2 vias on F.Cu/In2.Cu; both 0.1 mm wide, not routed as a coupled pair.

**Evidence.** pcbnew live mini: OC_D+ 23 segs 23.53 mm layers F/In2/In5 vias 3; OC_D- 25 segs 22.44 mm layers F/In2 vias 2; R38/R39 22 Ω at (85.87,131.72)/(86.82,131.73), ~2 mm from pads 26/25 at (86.71/87.11,133.36). Espressif layout page: 'route them in parallel at equal lengths… 90 Ω ±10 %'. V9 uses the same 22 Ω series parts.

**Consequence.** Full-speed (12 Mb/s) USB-Serial-JTAG tolerates this; nothing to do with the first article's missing CC termination.

**Fix.** Optional tidy on the next spin: same layer set and via count for both lines. No action now.

#### 8. [Note] OC_D+ and OC_D- run on different layers for 7.8 mm

- **Refs:** U1, R38, R39
- **Nets:** OC_D+, OC_D-
- **Reviewer:** `mini-pcb-usb-mcu-9`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** From U1, OC_D+ drops to In5 ((75.942,141.962)→(81.188,141.962)→(82.79,140.36)) while OC_D- runs on In2 ((76.548,142.312)→(82.153,141.48)); they rejoin as a 0.2 mm-pitch pair on In2 only from x ≈ 83 to R38/R39. The FC pair and the J6→U1 pair are routed as coupled pairs throughout.

**Evidence.** nets.py OC_D+ (In5 7.8 mm, then In2), OC_D- (In2 throughout); FC_D± both In2; D± both In2.

**Consequence.** None at full speed (12 Mb/s, the S3's only mode); noted for parity with the other two pairs and Espressif's 'route as a differential pair' rule.

**Fix.** Optional: move the OC_D+ In5 segment to In2 beside OC_D-.

#### 9. [Note] First-article 'no Rd' is not producible from the design files — it is an assembly/connector fault on J6's CC or GND path

- **Refs:** J6, R41, R47, J6 pads A5/B5/A1-B12/B1-A12/S1-S4
- **Nets:** Net-(J6-CC1), Net-(J6-CC2), GND
- **Reviewer:** `mini-usb-1`, confidence 0.7
- **Verification:** not run (single source).
- **First article:** likely-cause

**Claim.** Every element that produces the sink termination the Mac looks for is passive and correct: CC1 (J6.A5) -> R41 5.11 k -> GND and CC2 (J6.B5) -> R47 5.11 k -> GND, on the pads the GCT drawing brings out as CC1/CC2, with copper traces and a connected GND return. Nothing powered is in that path, so no regulator, mux, strapping or firmware state can hide Rd. Absence of Rd in BOTH orientations on several ports therefore needs a common physical cause: J6's GND pads/shell not wetted (no return for either Rd), the whole 0.30 mm pad row not wetted (both CC pads open), R41 and R47 both open/tombstoned, a damaged receptacle, or the cable/port.

**Evidence.** $S/out/live-mini/netlist.xml: Net-(J6-CC1) = J6.A5[CC1_A5] + R41.2; Net-(J6-CC2) = J6.B5[CC2_B5] + R47.1; R41.1 and R47.2 on GND; R41/R47 value 5.11 k (RC0402FR-075K11L). GCT USB4110 drawing (gct.co/files/drawings/usb4110.pdf, pin table): A5 = CC1, B5 = CC2, A6/B6 = Dp, A7/B7 = Dn, A4/B4/A9/B9 = VBUS, A1/B1/A12/B12 = GND, shell = GND; recommended layout pad order left->right A1B12 A4B9 B8 A5 B7 A6 A7 B6 A8 B5 B4A9 B1A12 at +/-0.25/0.75/1.25/1.75/2.40 mm — footprint GCT_USB4110GFA pads at x offsets -3.2,-2.4,-1.75,-1.25,-0.75,-0.25,+0.25,+0.75,+1.25,+1.75,+2.4,+3.2 (A5 at -1.25, B5 at +1.75), 0.30x1.15 and 0.60x1.15 mm, shells 2.18x2.00 at 10.22 mm, NPTH 0.65 at 5.78 mm — all match the drawing. Board (pcbnew, live mini): J6.A5 (83.11,163.715) -> F.Cu track -> R41.2 (82.37,162.41); J6.B5 (86.11,163.715) -> F.Cu track -> R47.1 (88.66,162.33); R41.1 GND via at (81.32,162.24); R47.2 GND through the F.Cu GND zone (HitTestFilledArea true); drc.json unconnected_items = 0. Footprint file GCT_USB4110GFA.kicad_mod is byte-identical (excluding uuid/3D) to the V9 copy and J6 pad geometry is identical on the V9 board, which enumerates.

**Consequence.** If the review stops at the schematic the owner keeps looking for a design bug that is not there; the connector row (16 x 0.30/0.60 mm pads, 0.5 mm pitch, hidden under the shell) and its GND shell pads are exactly where paste problems would remove Rd for both orientations at once.

**Fix.** Bench: with the board unpowered, measure CC1 and CC2 at the receptacle contacts (breakout cable) to board GND — expect 5.11 k each. If open at the contacts but 5.11 k at R41.2/R47.1, reflow J6's signal row; if open at the resistor pads too, check R41/R47 presence and the GND via/pour under them. Also check < 1 ohm from each J6 shell pad and the A1/B12, B1/A12 pads to board GND. Full ordered probe list is in coverage_notes.

#### 10. [Note] Bring-up order: with USB only and S1 in F, the host sees VBUS + Rd but no device until the OC has firmware and has been told to power the FC

- **Refs:** U15, U32, U30, D9, R84, C105, SW2, S1
- **Nets:** FC_EN_OC, FC_EN_HOLD, POWER_SWITCH, V_MCU_SWTCH
- **Reviewer:** `mini-usb-2`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** The FC's rail V_MCU_SWTCH is off at power-up (U30 EN/UVLO held low by R84 100 k) and is raised only by FC_EN_OC (OC GPIO7) or FC_EN_HOLD (FC GPIO17) through D9. The out_computer firmware boots with PWR_PIN LOW and waits for the BLE power-on command (except the #825 rail-restore path). So on a virgin board the only USB-visible processor is the OC in the O position; the FC's USB-Serial-JTAG cannot appear until the OC is flashed and commanded, or POWER_SWITCH is jumpered high on the bench. This is by design (identical to the V9's P4_EN_S3/P4_EN_HOLD/D9/U30/R84 arrangement), not a defect, but it determines the order of operations.

**Evidence.** netlist: POWER_SWITCH = C105.1, D9.3[K], R84.1, U30.5[EN/UVLO]; R84.2 = GND, R84 = 100 k; D9 BAV170M anodes FC_EN_HOLD (U32.23 GPIO17) and FC_EN_OC (U15.12 GPIO7); U30 TPS22810 VIN = +3V3, VOUT/QOD = V_MCU_SWTCH; U32 VDD3P3_RTC/CPU/VDDA on V_MCU_SWTCH, U15 on +3V3. Firmware tinkerrocket-idf/projects/out_computer/main/main.cpp:9668-9669 'digitalWrite(config::PWR_PIN, LOW); // Start with power rail OFF', :10032 'OutComputer ready (PWR_PIN OFF, waiting for power-on command)'; board_m1.h PWR_PIN = 7 (FC_EN_OC). V9 netlist: D9.1 P4_EN_HOLD, D9.2 P4_EN_S3, D9.3 POWER_SWITCH, U30 identical, R84 100 k, C105 10 uF — identical.

**Consequence.** A tester who plugs in with S1 at F sees Rd and VBUS but 'no device' and may conclude the USB path is dead. The FC cannot be flashed at all until the OC runs, unless the rail is forced.

**Fix.** Document the sequence in FABRICATION-NOTES/README bring-up: (1) S1 = O, flash the OC; (2) power the FC from the app/BLE cmd 8 (or, for a bare-board test, jumper POWER_SWITCH — R84.1/C105.1 — to +3V3, e.g. R36.1 at (94.44,141.43); V_MCU_SWTCH then reads ~3.4 V at R110.1 (78.81,149.11)); (3) S1 = F, flash the FC. Consider adding this to the first-article checklist.

#### 11. [Note] S1 electrical mapping confirmed (6-7 closed = SEL0 low = FSUSB63 port 2 = FC); which actuator end closes 6-7, and hence the new O/F silk, is not verifiable from the drawings

- **Refs:** S1, U1, R1, R2, silk 'O' (73.9,138.84) and 'F' (81.52,138.79) on B.SilkS
- **Nets:** SEL0, Net-(U1-SEL1), FC_D+/FC_D-, OC_D+/OC_D-
- **Reviewer:** `mini-usb-3`, confidence 0.6
- **Verification:** not run (single source).

**Claim.** With S1 open SEL[1:0] = 11 (R1, R2 100 k to +3V3) which the onsemi functional table maps to HSD3 = OC_D+/OC_D- (U15). Closing S1 (pad 6 common to pad 7 GND) gives 10 = HSD2 = FC_D+/FC_D- (U32). 00 (sleep) and 01 (HSD1, unconnected) are unreachable because SEL1 is hard-pulled high. This closes the open question in rocket-computer/prefab-review-2026-07-30.md line 838 (truth table unconfirmed). The 'F' silk sits at the pad-7 end of S1 (x = 81.5 vs pad 7 at 79.8, pad 5 at 74.8, 'O' at 73.9); that is the right end only if the JS series closes common-to-terminal on the side the actuator is moved TO, which the C&K page render does not settle. S1 is on the back of the mini (V9: front), but the pad-7 end is fixed by the footprint so the answer transfers from the V9 unchanged.

**Evidence.** onsemi FSUSB63 datasheet (fsusb63-d.pdf) Functional Table: SEL1/SEL0 00 = 'Sleep Mode, D+, D- Switch Paths Open', 01 = HSD1, 10 = HSD2, 11 = HSD3; pin table 4 SEL[0], 11 SEL[1], 7/8 HSD2-/+, 9/10 HSD3-/+. netlist: SEL0 = R2.1 + S1.6 + U1.4; Net-(U1-SEL1) = R1.1 + U1.11; R1 = R2 = 100 k to +3V3; S1.7 = GND, S1.1/2/3/5 NC; U1.7 FC_D-, U1.8 FC_D+, U1.9 OC_D-, U1.10 OC_D+. Board: S1 at (77.3,141.22) B.Cu rot -90, pad 6 (77.3,142.42), pad 7 (79.8,142.42), pad 5 (74.8,142.42); gr_text 'O' at (73.9,138.84) and 'F' at (81.52,138.79) B.SilkS (live only; HEAD lacks them). C&K JS datasheet page I-51 (ck-components js.pdf p.3): DPDT terminals 1-2-3 / 4-5-6 with the middle terminal common, 'shown in position 1' — the position-to-contact direction is only drawn, not stated. V9: identical S1 nets and footprint (SW_JS202011JCQN byte-identical), on F.Cu.

**Consequence.** If the mechanism closes toward the opposite end, the O/F letters are swapped and a tester in 'F' would be talking to the OC (which still enumerates, so the mistake would surface as 'wrong chip' rather than 'no device').

**Fix.** Bench: DMM continuity between S1 pads 6 and 7 (B side, (77.3,142.42)-(79.8,142.42)) with the actuator at the F-marked end; expect closed. If open there, swap the O/F silk before the next fab. Optionally record in README which end is which for both boards.

#### 12. [Note] Neither S3 has a reset button or an external GPIO0 pull-up; the FC's ROM download mode by button needs SW2 held through an OC-controlled rail cycle

- **Refs:** U15, U32, SW2, SW3, R36, C25, R110, C114
- **Nets:** Net-(U15-GPIO0), Net-(U32-GPIO0), Net-(U15-CHIP_PU), FC_CHIP_PU
- **Reviewer:** `mini-usb-4`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** GPIO0 on both S3s is wired only to its button (SW3 -> OC, SW2 -> FC) and relies on the internal weak pull-up at reset (Table 2-1: GPIO0 'WPU, IE' at reset; Table 3-1 default 1); CHIP_PU on each is only an RC (10 k / 1 uF, ~10 ms) from its own rail with no button. Download mode therefore requires either the USB-Serial-JTAG DTR/RTS-emulated reset (works whenever the selected chip's USB-JTAG is enumerated, no buttons) or holding the boot button while the chip's rail rises: unplug/replug USB for the OC, or an OC-driven FC_EN_OC cycle (or the POWER_SWITCH bench jumper) for the FC. The OC side is identical to the fabbed V9, which flashes this way.

**Evidence.** netlist: Net-(U15-GPIO0) = SW3.2 + U15.5 only; Net-(U32-GPIO0) = SW2.2 + U32.5 only; Net-(U15-CHIP_PU) = C25.1 (1 uF) + R36.2 (10 k, R36.1 = +3V3) + U15.4; FC_CHIP_PU = C114.1 (1 uF) + R110.2 (10 k, R110.1 = V_MCU_SWTCH) + U32.4. esp32-s3_datasheet_en.txt Table 2-1 pin 5 GPIO0 'WPU, IE' at reset; Table 3-1 GPIO0 default 1 (weak pull-up), GPIO46 default 0; Table 3-3 joint download boot = GPIO0 0 and GPIO46 0. V9 netlist: Net-(U15-GPIO0) = SW3.2 + U15.5; R36 10 k / C25 1 uF — identical.

**Consequence.** Not a defect. A bricked FC application that reconfigures GPIO19/20 would need the button route, which on the mini means coordinating with the OC's rail control; testers should know the recipe.

**Fix.** No hardware change. Add to firmware-notes/bring-up: 'FC download mode: hold SW2, then raise the FC rail (BLE power-on or POWER_SWITCH jumper); release after the ROM banner'. Optionally place a CHIP_PU reset button on a future spin if bench flashing proves awkward.

#### 13. [Note] USB-only, no-supercap start-up of U47 is expected to come up in bypass, but the U18->U47->+3V3 chain is not V9-proven and the config-latch moment during the V_BUCK ramp is the one exposure

- **Refs:** U47, U18, C142, C130 (not fitted at bring-up), R134/R135/R136
- **Nets:** V_BUCK, V_SCAP, +3V3, OSEL_SET, VCHG_SET, ICHG_SET
- **Reviewer:** `mini-usb-5`, confidence 0.5
- **Verification:** not run (single source).

**Claim.** U47 has EN = MODE = V_BUCK (Auto buck-or-boost mode). Once V_BUCK is at its DEF-high value of 3.465 V it exceeds the 3.0 V OSEL target by more than the 100 mV bypass threshold, so the bypass FET is on, +3V3 ~ V_BUCK minus I x 100 mohm, and the buck leg merely charges C142 (10 uF, the only capacitance on SUP with C130 absent) to VCHG 2.5 V and idles. VIN UVLO 1.7 V and EN high are the only enable conditions; VSUP is not required for bypass. The datasheet start-up sequence, however, precharges VOUT through the bypass, then latches OSEL/VCHG/ICHG and chooses boost or buck by comparing VIN with the target at that instant; if that happens while V_BUCK is still ramping (< 3.0 V) the device enters boost with an empty SUP and the bypass off, and +3V3 only recovers when VIN crosses target + 100 mV. This is a start-up glitch question for the mini-power reviewer, not a reason for missing Rd (the CC pull-downs are unpowered).

**Evidence.** netlist: U47.2 MODE, U47.3 EN, U47.4 VIN all on V_BUCK; U47.6 SUP = V_SCAP = C130.1 + C142.1 (10 uF) + L11.2 + R125.1; U47.9/10 VOUT = +3V3; OSEL R134 3.09 k, VCHG R135 6.65 k, ICHG R136 22.1 k. U18 TPS62152 (fixed 3.3 V, tps62152.txt line 143) with DEF (pin 8) on V_BUCK = 'nominal + 5%' (line 186) -> 3.465 V; FSW on V_BUCK per note (2) 'Connect FSW to VOUT'; EN (pin 13) tied to AVIN. tps61094.txt 7.1 'Auto buck or boost mode (EN = 1; MODE = 1)'; Table 7-4 Auto row: bypass on when VIN > target + 100 mV; VBYPASS 50/100/150 mV; 7.3.1 VIN UVLO 1.7 V typ; 7.3.2 'starts charging the output capacitor with a 300-mA constant current through the bypass switch ... After the output voltage reaches close to the input voltage, the TPS61094 starts to detect the configuration ... then latches ... enters Boost mode or Buck mode'; Table 7-4 boost-only row shows BYPASS = x. V9: U18 DEF = GND (3.3 V), FSW = +3V3, no U47 — the mini's chain is new. holdup-tps61094-rework.md line 206 requires DEF high (decision).

**Consequence.** Worst case a brief +3V3 dip at plug-in that could re-reset the OC (CHIP_PU RC 10 ms) once; in the expected case +3V3 comes up cleanly. Does not affect the CC termination.

**Fix.** No change proposed. Bench: scope V_BUCK (C141.1) and +3V3 (C143.1) together at USB plug-in with C130 absent; expect V_BUCK -> 3.46 V and +3V3 following within the bypass drop with no dip after the initial 300 mA precharge; V_SCAP (C142.1 at (83.43,133.36) B side) -> 2.5 V. Hand to mini-power for the ramp-timing analysis if a dip is seen.


### Assembly, paste and the stencil

#### 14. [Major] B1 tells the assembler U18/U19/U47 EPADs get no paste and forbids a stencil that has them; every plotted B.Paste (live, V1.0.1 tag, repo gerbers, release zip) HAS apertures on all three

- **Refs:** U18, U19, U47; FABRICATION-NOTES.md B1
- **Nets:** GND (EPADs), V_BUCK, +3V3
- **Reviewer:** `mini-dfm-paste-1`, confidence 0.95
- **Verified here:** CONFIRMED by my own pcbnew pass: U18 EPAD carries one 1.12 mm2 paste polygon (40 %), U19 four paste-only pads 0.96 x 1.25 mm (4.8 mm2, 70 %), U47 two 0.95 x 1.17 mm (2.22 mm2, 84 %), U30 two 1.0 x 0.7 mm (87 %), Q11 four 0.6 x 1.0 mm (65 %). A pad-layer-only scan misses all of these because the apertures are separate paste-only pads.
- **First article:** possible

**Claim.** B1 states 'THREE EXPOSED PADS HAVE A MASK OPENING AND NO APERTURE ... U18, U19, U47 ... DO NOT ADD PASTE BY HAND AND DO NOT SUBSTITUTE A STENCIL THAT HAS THEM', checked 'F.Paste GERBER ... AGAINST THE V1.0.1 RELEASE ZIP'. All three parts are on the BOTTOM side; the note looked at the wrong file. The B.Paste plot carries EPAD apertures on all three.

**Evidence.** kicad-cli plot of live board and of the mini-v101 tag (only difference between them: U5 pad 12), plus gerbers/rocket-computer-mini.zip and gerbers/release/rocket-computer-mini-v1.0.1-gerbers.zip (B_Paste 18748 bytes, 2026-09-05/06): U18 pad 17 (1.68x1.68 = 2.82 mm2) has one fp_poly region 1.060x1.060 = 1.124 mm2 (39.8 %); U19 pad 9 (2.29x3.00 = 6.87 mm2) has four paste-only pads RR0.96x1.25 = 4.60 mm2 (67.0 %); U47 pad 13 (1.00x2.65 = 2.65 mm2) has two paste-only pads R0.95x1.17 = 2.22 mm2 (83.9 %) - exactly TI DSS0012B's stencil example '2X (0.95) x 2X (1.17), 83 % printed coverage'. TI tps61094 package note 3: 'The package thermal pad must be soldered to the printed circuit board for optimal thermal and mechanical performance'; tps62152 RGT note 4: 'designed to be soldered to a thermal pad on the board'. The EPAD areas B1 quotes (2.8/6.9/2.7) are the copper pads, not the gerber.

**Consequence.** An assembler following B1 masks three EPAD apertures the footprints deliberately draw, and U18 (buck) and U47 (hold-up/+3V3 bypass) then sit on 0.26-0.5 mm perimeter pins only, at an 0.08 mm foil, with no mechanical/thermal EPAD joint - on the +3V3 supply chain. If instead the stencil was used as cut, the note is simply false and the next assembler cannot trust it.

**Fix.** Rewrite B1 to say the three EPADs ARE window-paned as plotted (U18 1 window 40 %, U19 4 windows 67 %, U47 2 windows 84 %) and must be printed; delete the 'do not substitute a stencil that has them' sentence. Ask the owner which way the V1 stencil was actually used.

#### 15. [Major] A3 promises filled+capped vias for U5 only and 'ALL OTHER VIAS TENTED' - but 250 vias sit inside SMD pads on ~100 footprints, including J6 D+, R41's GND end (CC1 pull-down), CR3 D-, U15 pin 15 and three BMP581 pins

- **Refs:** FABRICATION-NOTES.md A3; J6, CR3, R41, U15, U4, U16, U18, U19, U23, U30, U47, U11, S1, Q3-Q5, +~80 passives
- **Nets:** D+, D-, GND, L_RXEN, SENS_SCLK/SDI, BMP585_INT, +3V3, V_MCU_SWTCH, VBAT_CON
- **Reviewer:** `mini-dfm-paste-2`, confidence 0.7
- **Verified here:** PARTLY CONFIRMED: I count 252 vias inside SMD pads across 107 footprints (U5 52, J8 12, U15 11, U9 11, U16 9, U19 8, J3 7, U47 6). The note text is more defensible than the finding allows, because its first line specifies filled and capped for all vias; the risk is that the justification names only U5, so a fab may quote the cheaper reading. Severity reduced to minor.
- **First article:** possible

**Claim.** A3's wording ('VIA-IN-PAD IS MANDATORY ... THE GNSS MODULE (U5) CARRIES A VIA IN 24 OF ITS 26 GROUND PADS ... ALL OTHER VIAS TENTED BOTH SIDES') describes the U5 vias as the via-in-pad population. The board has 250 vias whose centre lies inside an SMD pad, 199 of them outside U5, plus 47 more whose 0.30 mm drill crosses a pad edge. A via inside a paste-printed pad cannot be tented; unless every via is epoxy-filled and capped, 0.08 mm paste wicks into the barrel.

**Evidence.** vip.py over pcbnew pad polygons vs via centres: U5 51 (pads 1,2,4x3,5x2,6 signal pads included), J8 12, U15 11 (pad 15 L_RXEN via at (91.16,133.32) in a 0.22 mm-wide pin pad, ring 0.40 > pad width), U9 11, U16 9 (pads 3,10,11,13,14,20 signal), U19 8, J3 7, U47 6, U30 5, U32 5, C13 4, U23 4, U4 3 (pads 2,4,7: 0.30 drill in 0.30x0.30 pads), U11 3, U18 3, J6 3 (A1/B12 GND (81.15,163.36), B6 D+ (85.11,163.715), S1 GND (80.18,163.49)), CR3 2 (pad 4 D- at (74.08,163.09), 0.07 mm from the pad edge), R41 1 (pad 1 GND at (81.32,162.24)), S1 2, Q3/Q4/Q5 2 each, ~75 0402s with one via. JLC: 'Epoxy Filled & Capped ... 0.15 to 0.55 mm', a board-wide option. No JLC order record in the repo.

**Consequence.** If the V1 order did not carry board-wide epoxy fill (the note invites 'tent the rest'), every one of these pads loses part of an already-thin 0.08 mm deposit into an open barrel: opens on 0402 decoupling, on J6 B6 (D+), on R41's GND end (CC1 loses its Rd in one plug orientation), on U15 pin 15, U4 pins 2/4/7.

**Fix.** A3: state that ALL 598 vias are filled and capped (IPC-4761 VII), give the count (250 in SMD pads), and remove 'all other vias tented'. Verify on the V1 JLC order; if fill was not ordered, treat every in-pad via as a suspect joint on the first article.

#### 16. [Major] Every pour is 'reliefs for PTH only': all SMD pads in the F/B pours are solid, so 97 two-terminal 0402s have one pad heat-sunk into a pour and the other on a 0.09-0.13 mm trace or a via - R41/R47 (CC pull-downs), SW2/SW3 (GPIO0), C76 (VBUS) included

- **Refs:** GND zone (F/B/In1/In2/In5/In6) and 21 B.Cu/F.Cu pours; R41, R47, C76, SW2, SW3, C25, C114, C120, C122, C33, R37, R111, D6-D11, C36, C142, C144-C149
- **Nets:** GND, CC1, CC2, VBUS, GPIO0, CHIP_PU, VDD3P3, +3V3
- **Reviewer:** `mini-dfm-paste-3`, confidence 0.85
- **Verified here:** CONFIRMED, with my own count rather than the reviewer's: all 22 non-rule zones report pad connection mode 3 (thermal reliefs for plated holes only), so every SMD pad in a pour is tied solid. Of 180 two-terminal surface-mount parts, 70 have one pad solid in a pour while the other reaches only tracks of 0.15 mm or less. Both USB configuration-channel pull-downs are in that set. The reviewers' own counts (66 in one pass, 97 in another) are both wrong; use 70.
- **First article:** possible

**Claim.** pcbnew: all 22 non-rule-area zones have pad_connection = 3 (THT_THERMAL: thermal relief for through-hole only, SMD pads solid); no footprint or pad override (-1). 97 two-terminal SMD parts have one pad whose centre is inside a same-net pour on its layer and the other pad not in any pour; with an 0.08 mm foil there is less paste to hold the part during the asymmetric wetting.

**Evidence.** solid.py sampled 16 points 0.06 mm outside each GND pad edge inside the GND fill: R47 pad 2 16/16, C76 pad 2 16/16, C41 16/16, C142 15/16, D10 16/16, C120 16/16, C9 16/16, SW3 pad 1 16/16, R37 14/16, R41 pad 1 11/16 (solid plus via-in-pad) - only C1 pad 2 (7/16) is thermal. Worst cases (other pad has NO track on its layer, only a via): C120, C142, C144, C145, C146, C147, C148, C149, C36, C76 (VBUS decoupling, pad 1 via-only at (77.37,165.68)), C9. On the USB path: R41 pad 1 GND solid + via, pad 2 CC1 on a 0.10 mm trace; R47 pad 2 GND solid, pad 1 CC2 on 0.10 mm; SW2/SW3 pad 1 GND solid, pad 2 GPIO0 0.10 mm; C25/C114 (CHIP_PU) pad 2 solid, pad 1 0.10 mm x3.

**Consequence.** Tombstoning / skew on 0402s during either pass; a lifted R41 or R47 removes the 5.11 k Rd from that CC line, a lifted SW2/SW3 or C25/C114 changes boot/EN behaviour; a lifted C76 loses the VBUS input cap.

**Fix.** Set the F.Cu and B.Cu pours (at least GND, +3V3, V_MCU_SWTCH, VBUS-adjacent) to 'Thermal reliefs' (0.25-0.3 mm spokes) or add a per-footprint thermal-relief override on the 0402/0603 footprints; keep inner planes solid. Report with coordinates for the owner's layout pass.

#### 17. [Major] Solder-mask dams below JLC's 0.10 mm minimum at U15/U32 (0.080 mm between all 56 pins), U23 (0.081 straight, 0.029 diagonal at the four corner pairs incl. SDA/SCL and the shunt-sense pair) and U18 (0.073 diagonal); the board's mask min-width rule is 0 so DRC cannot see them

- **Refs:** U15, U32, U23, U18 (below); U1 0.100, U14 0.100, J6 0.105, U21 0.20 (at/above)
- **Nets:** all S3 pins; SEN_SDA/SEN_SCL, VBAT_CON/VBAT_Terminal (U23 pads 4/5, 12/13)
- **Reviewer:** `mini-dfm-paste-4`, confidence 0.85
- **Verified here:** CONFIRMED in substance by my own mask-geometry pass: minimum web 0.070 mm at U15 and U32 (52 pairs under 0.10 mm each), 0.000 mm at U23 (pad against an aperture polygon, 73 pairs under 0.10), 0.035 mm at U18 pins 1-16. U1, U14 and J6 measure exactly 0.100 mm; U47 0.200 mm; U13 and U33 0.246 mm.
- **First article:** possible

**Claim.** Mask openings closer than the fab minimum are merged by the fab into gang openings, leaving no dam between adjacent paste bricks.

**Evidence.** gaps.py on the plotted F/B mask gerbers (polygon-to-polygon): U15/U32 pin openings are fp_poly 0.32x0.75 at 0.40 mm pitch (pads 0.22x0.65) -> 0.080 mm web, 52 pairs each, at e.g. (85.08,134.59); paste bricks 0.65x0.22 are 0.18 mm apart. U23 (footprint RGT16_1P7X1P7): pins 0.28x0.85 with 1:1 flash PLUS a 0.989x0.419 region (0.07 expansion) at 0.5 pitch -> 0.081 web; corner pin regions (pin 1 x-1.97..-0.98/y0.54..0.96 vs pin 16 x-0.96..-0.541/y0.98..1.97) 0.029 mm diagonal at (77.46,114.53),(79.69,115.25),(77.46,113.03),(79.69,112.30); pin openings to the EPAD strip openings 0.067. U18 corner pins (RR0.96x0.36 at 0.05 expansion) 0.073 mm diagonal at (73.67,131.16),(74.35,133.34),(75.85,130.47),(75.85,133.34). Board design rules: m_SolderMaskMinWidth 0, m_SolderMaskToCopperClearance 0. JLC capabilities: 'Min. pad spacing 0.10 mm' green/red/yellow/blue/purple, 0.13 black/white. Stackup names 'JLCPCB Soldermask' without a colour.

**Consequence.** Gang-opened rows on both 0.4 mm S3s and on the INA230: with hand-printed 0.08 mm paste, a smear between 0.18 mm-apart bricks has no dam to stop a bridge; conversely a merged opening removes the mask that keeps paste on the pad. U23 corner merges pair SDA with SCL and IN+ with IN-.

**Fix.** Set solder-mask min width 0.10 mm in the board rules and rerun DRC; on U15/U32 reduce the pin mask polygons from 0.32 to 0.30 wide (web 0.10); on U23 drop the redundant 1:1 pin flashes and shrink the pin regions to 0.05 expansion; accept the U18 corner diagonals or trim them. If black/white mask was ordered the minimum is 0.13 and J6/U1/U14 also fall below.

#### 18. [Minor] U23's EPAD paste is nine windows including four 0.281 x 0.281 mm corners at AR 0.83 - the lowest non-WLCSP area ratio on the board - for 40 % coverage, and its pin apertures are reduced to 77 %

- **Refs:** U23 (INA230, Footprints:RGT16_1P7X1P7, B side)
- **Nets:** GND (EPAD), VBAT_CON, VBAT_Terminal, SEN_SDA/SCL, +3V3
- **Reviewer:** `mini-dfm-paste-9`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** The vendor-style footprint draws all its paste as polygons: 16 pin apertures 0.80x0.23 (77.4 % of 0.85x0.28), an EPAD cross of 0.587x0.587 centre, four 0.587x0.281 arms and four 0.281x0.281 corners; total 1.163 mm2 on a 2.89 mm2 pad (40.2 %). TI's RGT example is 85 % at 0.125 mm. The 0.281 mm corner windows are the least releasable apertures after the WLCSP.

**Evidence.** gerber regions per %TO.C U23: poly6v 0.281x0.281 x4 AR 0.829, poly7v 0.281x0.587 x4 AR 1.155, poly9v 0.587x0.587 x1 AR 1.889, 16 pin regions AR 1.117; pcbnew: pads 1-16 have no B.Paste layer, pad 17 has no B.Mask/B.Paste layer (mask drawn as six strips 1.827x0.320/0.387 forming a '#'). TI ina230 RGT0016C: '16X (0.6)', '16X (0.24)', 'EXPOSED PAD 17: 85 % PRINTED SOLDER COVERAGE ... BASED ON 0.125 mm THICK STENCIL'.

**Consequence.** At 0.08 mm the EPAD gets 0.093 mm3 of paste (TI's example ~0.31 mm3); with the '#' mask the copper under the mask islands is never wetted. The INA230 is on the pack path, not the USB power-up path.

**Fix.** Replace the polygon paste with a 2x2 windowpane of ~0.6 mm squares (AR ~1.9, ~50 %), give pad 17 a normal 1:1 mask opening, and restore the pins to full-pad apertures.

#### 19. [Minor] U13/U33 WLCSP ball pads have a mask aperture exactly equal to the copper (0.254 mm) — neither NSMD nor SMD

- **Refs:** U13, U33
- **Nets:** OC_SPI_*, FC_SPI_*, +3V3, V_MCU_SWTCH, GND
- **Reviewer:** `mini-pcb-usb-mcu-5`, confidence 0.6
- **Verification:** not run (single source).
- **First article:** possible

**Claim.** The 24-WLCSP_Y_WIN pads are 0.254 mm circles with no local mask margin, and the board's solder_mask_to_copper_clearance is 0.0, so the fabbed F.Mask gerber flashes a 0.254 mm circle (aperture D30/D29 = C,0.254) on each ball pad. With the fab's mask registration (typically ±0.05 mm) each ball lands on a pad that is randomly mask-defined on one side and copper-defined on the other; the effective land varies ball to ball by up to 20 % of its diameter.

**Evidence.** dump1.py: U13/U33 pads size 0.254 layers F.Cu/F.Mask/F.Paste maskm None; kicad_pro solder_mask_to_copper_clearance 0.0 (same on V9 and mini-v101); gerbers/rocket-computer-mini-F_Mask.gbr %ADD30C,0.254000*% and %ADD29C,0.254000*%; the V9's flash was SOIC (W25Q128JVSIQ), so there is no working reference for this land. Via check: no via inside any of the 48 ball pads; nearest drill edges 0.045 mm (U13.C2 ↔ OC_SPI_HD via at 0.322 c-c) and 0.054 mm (U33.E3 ↔ GND via at 0.331 c-c) — in both the 0.4 mm via ring is contiguous with its own-net ball pad, tented and type-VII capped, so harmless.

**Consequence.** Ball-to-ball joint variation and reduced tolerance to the paste problems the first article already had; a boot-NOR joint that does not wet stops the S3 from booting the application (it does not stop USB-Serial/JTAG enumeration, which the ROM provides).

**Fix.** Give the WLCSP pads a local solder-mask margin (about +0.05 to +0.06 mm → 0.35–0.37 mm NSMD openings; the 0.5 mm pitch still leaves ≥0.13 mm of mask web) or the fab's minimum NSMD expansion, and confirm the land against Winbond's WLCSP land recommendation if one can be obtained.

#### 20. [Minor] Q11 and U30 exposed pads get no paste and are not in the B1 no-paste list; V9's U30 windows its pad

- **Refs:** Q11 (AONR21321, DFN-8 EP, B side 92.74,132.23), U30 (TPS22810 DRV, B side 84.28,140.06)
- **Nets:** VBAT_J8, VBAT_Terminal, GND
- **Reviewer:** `mini-pcb-visual-1`, confidence 0.85
- **Verified here:** CONFIRMED in part: U30 and Q11 are pasted (87 % and 65 %) and are absent from the B1 list. The finding states they get no paste; the direction is inverted but the documentation gap it names is real.

**Claim.** Two more exposed pads than FABRICATION-NOTES B1 documents carry no stencil aperture: Q11 pad 9 (1.55x2.40 mm, net VBAT_J8 = the FET's drain, the pack current path) and U30 pad 7 (1.00x1.60 mm, GND, thermal). B1 lists only U18/U19/U47 as the 'three exposed pads' with no aperture, and the V9 (working) U30 footprint carries a paste polygon on its pad 7 while the mini's carries none.

**Evidence.** pcbnew pad scan of live rocket-computer-mini.kicad_pcb: Q11 pad 9 layers lack B.Paste, fp has 0 paste polygons; U30 pad 7 lacks B.Paste, 0 paste polygons. Netlist: Q11.9 PAD_9 = VBAT_J8 (D_5..D_8 also VBAT_J8), Q11.4 G = GND, Q11.1-3 S = VBAT_Terminal (reverse-battery P-FET). FABRICATION-NOTES.md lines 153-163 name only U18, U19, U47. V9 rocket-computer.kicad_pcb: U30/U27/U29 TPS22810 pad 7 paste=False but 1 paste polygon each (windowed).

**Consequence.** Q11's drain current (system + pyro fire) is carried by leads 5-8 only; both parts rely on perimeter pads for attachment and cooling. Electrically both remain connected (drain leads share the paddle; U30 GND is also on pad 4), so this is thermal/mechanical margin and a documentation gap, not a function loss. Given the first article's paste problems, an undocumented paste-less pad is the kind of thing that gets 'fixed' by hand.

**Fix.** Either add window-paned apertures on Q11 pad 9 and U30 pad 7 (board-copy footprint, like U11/U15/U32) or add both to the B1 list so nobody adds paste by hand; make U30 match the V9 footprint's windowed pad.

#### 21. [Note] Quectel LC86G stencil requirements cannot be checked: the hardware design guide defers stencil thickness to 'Quectel_Module_Stencil_Design_Requirements', which is not vendored, while the board prints all 35 U5 pads 1:1 at 0.08 mm

- **Refs:** U5; SOLDER-PASTE-CONVENTION.md; FABRICATION-NOTES.md B1
- **Nets:** GND, V_MCU_SWTCH, GNSS_RX/TX
- **Reviewer:** `mini-dfm-paste-10`, confidence 0.6
- **Verification:** not run (single source).

**Claim.** The convention argues 1:1 coverage sets standoff, but the absolute volume is set by the foil: 1.0x1.0 mm interior pads receive 0.08 mm3 and the 2.5x1.0 castellation lands 0.20 mm3. Whether that meets Quectel's module stencil requirement is unverifiable from the repo.

**Evidence.** datasheets/Quectel_LC86G_Series_Hardware_Design_V1.5.txt line 2881: 'For more information about the stencil thickness for the module, see document [8] module stencil design requirements'; [8] Quectel_Module_Stencil_Design_Requirements not in hardware/datasheets. Paste gerber: U5 R1.000x1.000 x24 (AR 3.1), RR2.500x1.000 x11 (AR 4.7), pad 12 four windows 1.08x0.97 (74.5 %). Min aperture pitch 2.2 mm, so no bridging.

**Consequence.** If Quectel asks for 0.12-0.15 mm foils for LCC modules (common for its M-series), U5 is receiving roughly half the intended solder, compounding the blind-joint risk B2/B5 already record.

**Fix.** Obtain the Quectel stencil document and record its number in B1; if it exceeds 0.08 mm, a step stencil over U5 (raised, not lowered) is the only way to keep the WLCSP at 0.08 mm on the same side.

#### 22. [Note] Fab-note geometry drift: U5's signal pads carry eight vias the note says they do not; A2/A7 call U21 0.4 mm pitch (it is 0.5); J8 area quoted as 26.2 mm2 counts the two F-side pads

- **Refs:** FABRICATION-NOTES.md A2, A7, B7, 'U5's ten non-ground pads' paragraph; U5, U21, J8
- **Nets:** GNSS_RX, GNSS_TX, V_MCU_SWTCH (U5 VCC/V_BCKP), Net-(U5-1PPS)
- **Reviewer:** `mini-dfm-paste-7`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** The notes describe the U5 signal pads as via-free ('Route them out on F.Cu ... without a via and without cutting In1'); the board has vias inside pad 1 (GNSS_RX), pad 2 (GNSS_TX), pad 4 (three, V_MCU_SWTCH), pad 5 (two) and pad 6 (1PPS). A2 lists 'THE 0.4 mm X2QFN (U21)' and A7 counts U21 among 0.40 mm parts; TI RUX0012A is '6X 0.5' and the board's U21 pins sit at x = +/-0.25/+/-0.75 mm.

**Evidence.** vip.py: U5 pad 1 via (74.489,112.916); pad 2 (74.67,115.68); pad 4 (75.07,120.53),(75.99,120.54),(74.41,120.7); pad 5 (76.623,123.092),(74.87,123.07); pad 6 (75.61,125.63). U5 GND pads with a via: 3,12-31,34-36 = 24 (matches A3); 32/33 none (matches). U21 pad table from board.json; tps2121.txt RUX stencil example '6X 0.5', '8X (0.2)', '8X (0.6)', '4X (0.4)', '4X (1.05)', 'EXPOSED PAD 100 % PRINTED COVERAGE' - the board's U21 apertures match TI exactly. J8: three B pads 1.5x3.5 = 15.75 mm2 on the reflow side, five pads = 26.25.

**Consequence.** Nothing electrical; the assembler/fab reads a process class (0.4 vs 0.5 pitch) and a via inventory that do not match the gerbers.

**Fix.** Propose replacement text: U5 signal pads 1, 2, 4, 5, 6 carry vias (filled+capped under A3); U21 is 0.5 mm pitch VQFN-HR; finest pitch parts are U1, U14, U15, U32.

#### 23. [Note] Fiducials: all four visible at both paste stages; occlusion cannot be the misprint cause - but they are 0.5 mm copper in a 1.0 mm mask ring, 1.6-2.7 mm from the board edge, which most vision systems will not use

- **Refs:** FID1 (78.63,103.50) F, FID2 (84.26,168.92) F, FID3 (84.24,168.95) B, FID4 (82.22,112.16) B; J6, J2, C130
- **Nets:** -
- **Reviewer:** `mini-dfm-paste-8`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** At the paste stage nothing is on either side, so a stencil printer's camera sees FID1+FID2 (top) and FID3+FID4 (bottom). At placement, FID2 is inside J6's courtyard and FID3 inside J2's but those parts are placed after the fiducials are read (accepted, #1008); FID4 is under C130's can only after hand assembly. A paste misprint on V1 is therefore not a fiducial-occlusion outcome; it is a stencil-alignment/printing outcome (owner-printed, per #1167 'the owner is the assembler').

**Evidence.** board.json: all four Fiducial_0.5mm_Mask1mm, copper 0.50, maskE 0.25 (mask C1.000 in the gerber), no copper pad within 2 mm on the same side, FID4 has GND vias at 1.10-1.42 mm. Edge distances: FID1 1.56 mm from the top edge; FID2/FID3 2.62-2.65 mm from the bottom edge; F pair baseline 65.4 mm in y but only 5.6 mm apart in x; B pair 56.8 mm in y, 2.0 mm in x. JLC capabilities pages publish no fiducial rule (their SMT line uses its own marks).

**Consequence.** Fine for a two-point transform, but below the usual 1 mm copper / 2-3 mm clearance / >=5 mm-from-edge practice; an assembler may ignore them and align on pads, which is what an owner-printed stencil does anyway.

**Fix.** None for V1 (decided). Next spin, if a contract assembler is used: 1.0 mm copper, 2.0-3.0 mm mask clearance, >=5 mm from the edge, and spread the F pair in x.


### Footprints and land patterns

#### 24. [Major] U9 AON7534 (DFN 3x3, 3.0 mm body) sits on the 3.3 mm TSON-Advance land: its terminals reach the pin pads by only ~0.05 mm

- **Refs:** U9 (mini F.Cu at 92.985,161.138 rot 90); V10 U9 inherits the same land via PR #1452
- **Nets:** GND (U9.1-3), Net-(U9-GATE) (U9.4), PYRO_GND (U9.5-8 + tab)
- **Reviewer:** `mini-footprint-pinout-1`, confidence 0.6
- **Verified here:** CONFIRMED arithmetic: the shared land pads are rotated 90 degrees, so they span 1.448 to 2.108 mm from the part centre, not 1.587 to 1.969. My first extraction missed the rotation and was wrong. Against a 3.0 mm body with 0.4 mm terminals the overlap is 0.052 mm; against the 3.3 mm parts the land was drawn for it is about 0.20 mm. The package dimensions themselves are taken from the vendor outline the finder cites and are not independently re-derived here.

**Claim.** Pin names are correct (S,S,S,G / D,D,D,D = AOS top view), but the land geometry was drawn for a 3.3 x 3.3 body (Toshiba TSON Advance / TI CSD16323Q3 on the V9). The AON7534 is AOS 'DFN 3x3 EP' = outline DFN3x3A_8L_EP1_P: body D/E 2.90-3.10, terminal L 0.30-0.50 measured inward from the body edge, so each terminal occupies 1.10-1.50 mm from the package centre (edge 1.45-1.55 over tolerance). The TSON land's pin pads are 0.381 x 0.660 centred 1.778 mm from centre, i.e. copper from 1.448 to 2.108 mm. Underside overlap with the terminal is 1.448-1.50 = 0.052 mm nominal, 0.00-0.10 mm over tolerance; the remaining 0.6 mm of each pad is outside the package. AOS's own recommended land puts the lead pads 0.35 x 0.50 spanning 1.15-1.65 mm (drawing 'RECOMMENDED LAND PATTERN', total height 3.30). The 2.5 x 2.5 tab (centre offset +0.28 toward the drain) is also much larger than the AOS EP (D1 2.35 x E2 1.75): along the pin axis it spans -0.97..+1.53 mm vs the EP's +/-0.875, so it carries full paste to within ~0.13 mm of the source/gate terminals' inner ends (at -1.10) and runs 0.66 mm under the drain terminals.

**Evidence.** Footprints.pretty/TSON Advance_TOS.kicad_mod pads 1-8 at (+/-1.778, +/-0.325/0.975) size 0.381x0.660 rot 90, pad 5 tab 2.5x2.5 at (0.28,0.01) with F.Paste; board copy identical (boardvslib2 transform (270,False)). Board pads: U9.1-4 at y=162.916 (inner copper edge 162.586), U9.5-8 at y=159.360 (inner edge 159.690); a nominal AON7534 terminal spans y=162.238-162.638 and 159.638-160.038 -> 0.052 mm overlap each side. AOS PO-00047 'DFN3x3A_8L_EP1_P PACKAGE OUTLINE' (aosmd.com/res/packaging_information/DFN3x3A-8L EP1_P.pdf): D/E 2.90/3.00/3.10, L 0.30/0.40/0.50, D1 2.25-2.45, E2 1.65-1.85, e 0.65, land: pads 0.35x0.50, pattern height 3.30, EP 2.45x1.65. AON7534 datasheet Rev1.1 p1 names the package 'DFN 3x3 EP' with top view S1 S2 S3 G4 / D8 D7 D6 D5. Winsok WSD20L50DN33 Rev3.0 p7 (E 3.15-3.45, L 0.3-0.5) gives the P-FETs 0.13-0.20 mm overlap, equal to the V9's Toshiba part, so U6-U8/U10 are not affected. mini-part-parity-2026-09-12.md rows U6-U8/U10 and U9 ('land compatibility closed by numeric overlay').

**Consequence.** The pyro ARM FET's gate and source joints depend on a toe/flank fillet on a 0.4 mm terminal with almost no pad under it; with the first article's paste problems this is the joint most likely to be open or weak. An open source or gate on U9 leaves PYRO_GND floating (no channel can fire); a drain-tab paste bridge to the gate terminal ties the gate to PYRO_GND. Neither affects USB enumeration.

**Fix.** Give U9 its own AOS-pattern land (8 pads 0.35 x 0.50 spanning 1.15-1.65 mm from centre, EP pad 2.45 x 1.65 centred, paste windowed to the EP) instead of the shared TSON Advance land; or, if one land must serve both vendors, extend the pin pads inward to 1.15 mm (length 0.95) and shrink/centre the tab paste to the 2.35 x 1.75 EP. Owner does layout; this is a library-footprint proposal to review at V10 layout time (#1365) and before the next mini spin. On the first article, inspect/reflow U9's four source/gate joints.

#### 25. [Minor] BMP581: a via sits under the package and solder mask covers the fill under the sensor; pads are smaller than Bosch's land

- **Refs:** U4
- **Nets:** GND
- **Reviewer:** `mini-sensors-rf-4`, confidence 0.7
- **Verification:** not run (single source).
- **First article:** possible

**Claim.** Bosch BMP581 DS004-13 §8.2: 'We do not recommend vias or traces under the BMP581 ... it is recommended that there is no solder mask under the sensor' and the land should be footprint +25 µm per side (0.375 x 0.35 mm pads). On the mini a GND via is at (77.60,161.41), 0.10 mm from U4's centre (77.70,161.43), the F.Cu GND fill under the body is mask-covered (the footprint BMP581_LGA-10_2x2 has no F.Mask opening under the body), and the pads are 0.30 x 0.30 mm. Filled-and-capped vias (fab spec) remove the coplanarity part of the concern; the mask-contact concern remains. Pad numbering/geometry against Figure 23/32 checks out (footprint is the top view rotated 90 deg CCW, pin-1 mark consistent). V10 uses the same footprint, so the same applies at V10 placement.

**Evidence.** pcbnew live-mini: VIA GND (77.60,161.41) d=0.10 mm from U4; U4 pads at 77.45-78.48, 160.65-162.21; footprints/Footprints.pretty/BMP581_LGA-10_2x2.kicad_mod pads 0.3x0.3, only F.Cu/F.Mask/F.Paste on pads, silk circle at (-1.35,-0.25); bmp581.txt / PDF p.69 §8.2 landing pattern.

**Consequence.** Possible pressure-offset/noise degradation if mask under the port-side body contacts the package; smaller pads reduce joint area on a 2 mm LGA that already had a paste-troubled first article.

**Fix.** Next spin: move the GND via out from under the body, add an F.Mask opening under the body (over solid GND, no traces), and grow pads to 0.375 x 0.35 per §8.2. Owner does layout.

#### 26. [Note] U19 board copy of the SOIC-8 PowerPAD footprint lacks the library's six thermal vias and back-side pad

- **Refs:** U19 TPS259631DDAR (B.Cu at 89.015,123.655 rot 180)
- **Nets:** GND (EP pad 9)
- **Reviewer:** `mini-footprint-pinout-2`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** The library file Footprints:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.29x3mm_ThermalVias carries six 0.5 mm through-hole pads numbered 9 at (+/-0.65, -1/0/+1) plus a 1.8x2.5 pad on the opposite copper layer; the board-embedded U19 (live and mini-v1.0.1, identical) has only pads 1-8, the 2.29x3.0 EP pad 9 and the four paste windows, and its description text is KiCad's generic 'SOIC, 8 Pin (analog.com ada4898...)'. The board copy therefore predates (or was placed from) the no-via KiCad footprint; the fabbed board has no vias under the eFuse EP.

**Evidence.** boardvslib2.py: U19 -> no rotation/mirror maps the library pad set onto the board pad set (ANOMALY); board text block for U19 lists pads '', 1-8, 9 (2.29x3) only, in both $S/live and $S/mini-v101 boards; library .kicad_mod lines with '(pad "9" thru_hole circle' x6. Pin-1 position and pins 1-8 match TI TPS2596 DDA top view (GND, dVdt, EN/UVLO, IN down the left; OUT, FLT, ILM, OVLO up the right).

**Consequence.** Electrically nothing (pins 1-8 and the EP net are right); thermally the eFuse EP has no via path to the inner grounds, and the EP is unpasted by decision (B1) anyway. Library and board disagree, so a future 'update footprint from library' would silently add six vias under U19.

**Fix.** Decide which is intended and make the library match the board (drop the ThermalVias variant or rename it), or accept the vias at the next spin. No first-article action.

#### 27. [Note] ROHM EMT3 pin numbering is reversed relative to the footprint's pad numbers by design; the footprint text does not say so

- **Refs:** Q3-Q6, Q12, Q14 DTC123JETL; Q13 DTA123JETL; Footprints:TRANS_SOT-416_EMT3-EMT3F_ROHM; symbols Transistor_BJT:DTC123J / DTA123J (KiCad library, pins 1=B 2=E 3=C)
- **Nets:** PYRO1..4_FIRE, OC_ARM_EN, Net-(Q12-B/C), Net-(Q13-B/C), VBAT_CON
- **Reviewer:** `mini-footprint-pinout-3`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** The mapping is CORRECT but only because the custom footprint numbers its pads the KiCad/EMT3F way (pad 1 = front-left, pad 2 = front-right, pad 3 = lone rear lead). ROHM numbers the EMT3 (SOT-416) package the other way round: DTx123JE inner circuit '(1) EMITTER, (2) BASE, (3) COLLECTOR' with (2) drawn front-left and (1) front-right, while the EMT3F/VMT3/UMT3F variants are '(1) BASE, (2) EMITTER' with (1) front-left. The physical lead arrangement is identical (base front-left, emitter front-right, collector rear), so symbol B on pad 1 (front-left) and E on pad 2 (front-right) lands on the right leads. Nothing in the footprint description records this, and a well-meaning 'fix' that renumbers the pads to ROHM's EMT3 numbers, or swaps the symbol to a 1=E/2=B ROHM-style symbol, would put PYRO_FIRE on the emitter and GND on the base and silently kill every pyro driver and the arm chain.

**Evidence.** ROHM DTA123J datasheet 20171215-Rev.002 p1: outline pictures SOT-416 (EMT3) with (2) front-left, (1) front-right, (3) rear; SOT-416FL (EMT3F) with (1) front-left, (2) front-right; inner circuit 'DTA123JE/JU3/JKA: (1) GND(+) (EMITTER), (2) IN (BASE), (3) OUT (COLLECTOR)' vs 'DTA123JM/JEB/JUB: (1) IN (BASE), (2) GND (EMITTER)'. Same layout on the DTC123J datasheet p1 (20150311-Rev.002). Footprint pads: 1 at (-0.5,+0.65), 2 at (+0.5,+0.65), 3 at (0,-0.65) (KiCad y-down => 1,2 are the front pair, 1 on the left). Netlist: Q3.1 B_1=PYRO2_FIRE, Q3.2 E_2=GND, Q3.3 C_3=Net-(Q3-C)->U6 gate; Q13.2 E_2=VBAT_CON (PNP emitter to the pack, correct). Board copies of all seven transistors match the library under the expected transform.

**Consequence.** None today. A latent documentation trap for anyone who checks the EMT3 pinout against ROHM's numbers.

**Fix.** Add one sentence to the footprint description: 'Pad numbers follow EMT3F/KiCad SOT-416 (1 = base lead, front-left; 2 = emitter, front-right; 3 = collector). ROHM's EMT3 (DTx123JE) numbers the same leads 2/1/3 - do not renumber.' Owner's text; proposal only.


### Power tree

#### 28. [Minor] The whole V_MCU_SWTCH rail enters the In4 plane through one 0.3 mm via in U30 pad 1

- **Refs:** U30 (TPS22810, B side at 84.28,140.06), via (84.93,141.08), In4 plane
- **Nets:** V_MCU_SWTCH
- **Reviewer:** `mini-pcb-power-1`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** U30's VOUT pad 1 is a 0.24 mm2 copper island on B.Cu with exactly one via (0.4/0.3 mm) in it; pad 2 (QOD) has its own via but the two pads are not joined by copper, and QOD is the discharge-FET pin, not a current path. Every load on the switched rail (U32 FC, U5 GNSS, U16 LoRa, U11 NAND, U2/U3/U4, U33) therefore hangs on a single via-in-pad; the min-cut analysis returns 1 via at (84.93,141.08) for U5.4/U5.5, U2.5/8, U16.1 and every U32 supply pin.

**Evidence.** Netlist out/live-mini/netlist.xml: U30.1 VOUT_1 and U30.2 QOD_2 both on V_MCU_SWTCH; U30.6 VIN on +3V3. Board raster (0.01 mm) of V_MCU_SWTCH on B.Cu: island containing U30.1 = 0.24 mm2 with via (84.93,141.08) only; island containing U30.2 = 0.24 mm2 with via (84.28,141.08) only; no B.Cu track of the net touches either pad (tracks.txt). Pad size 0.69x0.32 mm, via diameter 0.40 mm (wider than the pad), drill 0.30. In4 fill is one island of 1285 mm2 and all 30 net vias land on it. V9 (fabbed) rocket-computer.kicad_pcb: U30 pad 1 has 2 vias within 1.5 mm ((75.25,116.9),(75.25,116.25)) plus F.Cu and In2 zones on the net, so the mini regressed from >=2 to 1.

**Consequence.** Current-wise it is fine: a 0.3 mm drill with 20-25 um wall is 0.019-0.024 mm2 (about a 0.55-0.7 mm 1 oz trace), roughly 1.1-1.5 A at 10 C rise, so 0.3-0.6 A gives ~1-3 C. The risk is reliability: a single capped via-in-pad (void, dimple, barrel crack under the solder joint) takes the flight computer, GNSS, radio and NAND down with no redundancy, and the failure is invisible to the OC except as an FC that never answers.

**Fix.** Bridge U30 pads 1 and 2 on B.Cu (same net; a 0.3 mm stub between (84.93,141.08) and (84.28,141.08)) so the QOD pad's via shares the current, and/or add a second via on a short stub off pad 1's outer end (the +3V3 0.2 mm track at x=85.78 limits the space; a via at about (85.35,141.55) needs checking against it). Two vias in parallel is what the V9 has.

#### 29. [Minor] All four pyro channels draw their pulse through one 1.4-2.5 mm wide 0.5 oz strip on In5 and four vias

- **Refs:** In5 VBAT_CON pour (85.4-93.1 x 127.1-147.2), vias (91.63,128.14) (92.28,127.49) (90.89,128.16) (90.93,127.50) (92.19,128.14) (91.64,127.49) at the R72 end and (87.62,147.01) (87.59,146.44) (87.15,146.05) (87.11,146.72) at the FET end
- **Nets:** VBAT_CON
- **Reviewer:** `mini-pcb-power-3`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** VBAT_CON on B.Cu is three separate islands (R72/U19 side 10.5 mm2, pyro-FET side 49.6 mm2, U23 sense 0.7 mm2); the only copper joining the pack side to U6/U7/U8/U10 is the In5 strip, whose row cuts (perpendicular to the y-direction flow) are min 1.63 mm at y=133.25, 1.38 mm at y=146.48 by the FET-end vias, median 2.51 mm, 10th percentile 1.93 mm, in 0.0152 mm copper. Six vias lift it from the R72 pour and four drop it to the FET pour.

**Evidence.** pours.txt VBAT_CON In5.Cu row profile (1.63 at 133.25, 1.38 at 146.48); vialayers.py: exactly six VBAT_CON vias touch In5 and B.Cu at the R72 end and four at the FET end; spof.py: min-cut from R72.1 to U6/U7/U8/U10 source pads = the four FET-end vias. Cross-section at the narrowest = 1.63 x 0.0152 = 0.0248 mm2; strip resistance about 9-10 milliohm end to end (20 mm at ~2.4 mm mean). Adiabatic rise for 10 A / 20 ms (I2t = 2 A2s, rho 1.72e-8, c.d 3.45e6): 17 K at the 1.63 mm cut, 7 K at the median, 0.5 K in the six vias and 1.1 K in the four (0.0236 mm2 each at 25 um wall). IPC-2221 internal for a hypothetical steady 2 A on the 1.63 mm cut: ~56 C (only pulses flow here: the buck input goes R72->U19->VBATT on B.Cu, never through In5). Drop at 10 A about 0.1 V.

**Consequence.** Adequate for single e-match pulses with margin, but it is the thinnest copper in a 10 A path on the board (a 1 oz outer route of the same width would run 2.3x cooler), and four channels fired inside a second stack their adiabatic rises (~4 x 17 K at the narrowest before the copper cools).

**Fix.** Widen the In5 pour to >=3 mm where the neighbouring In5 GND fill allows (y 132-134 and 145.5-147) and add two more vias at the FET end near (87.4,146.4); alternatively carry VBAT_CON on B.Cu 1 oz alongside the In5 strip.

#### 30. [Minor] +3V3 worst-case peak is within ~40 mV of the 3.6 V absolute maximum shared by every part on the rail, and PSM ripple is not in the documented stack

- **Refs:** U18 (DEF pin 8 -> V_BUCK), U47 bypass, U15, U32, U5, U13/U33, U2/U3/U4, U11, U16, R134
- **Nets:** V_BUCK, +3V3, V_MCU_SWTCH
- **Reviewer:** `mini-power-3`, confidence 0.6
- **Verification:** not run (single source).

**Claim.** DEF is tied high (V_BUCK), so U18 regulates 3.3 V + 5 % = 3.465 V nominal; +3V3 is V_BUCK minus the bypass FET drop (150 mOhm at 3.6 V, ~7 mV at the 45 mA OC-only load). TPS62152 Power-Save-Mode accuracy with COUT = 22 uF is +2.8 % (822.4/800), giving 3.562 V DC worst-high, which the hold-up note already records as 38 mV under the 3.6 V ceiling. That accuracy row is a regulation band, not a peak: PSM output ripple with the fitted 22 uF (C135) rides on top, and the rail is in PSM at every light-load condition (pad standby, OC-only). The peak therefore approaches or touches the 3.6 V absolute maximum of the ESP32-S3 (recommended 3.0-3.6 V), LC86G (abs max 3.63 V), the NOR/NAND, IMU, baro and mag (3.6 V class) on worst-case units. The V9, the working reference, runs DEF = GND (3.3 V) with FSW/VOS/PG on +3V3.

**Evidence.** Netlist live-mini: U18.8 DEF_8 -> V_BUCK; V9: U18.8 DEF_8 -> GND. tps62152.txt lines 318-325: 'DEF = 1 (VOUT): VOUT + 5%'; 'Power Save Mode operation, COUT = 22 uF: 781.6 / 800 / 822.4 mV' (= -2.3 % / +2.8 %). tps61094.txt line 377-378: RDS(on)_BYP 150 mOhm at VOUT = 3.6 V. esp32-s3_datasheet_en.txt lines 3645-3648: VDDA, VDD3P3, VDD3P3_RTC, VDD3P3_CPU recommended 3.0 / 3.3 / 3.6 V. Quectel LC86G HW design line 2422: VCC absolute maximum 3.63 V. holdup-tps61094-rework.md: 'Worst-high is already 3.465 x 1.028 = 3.562 V ... about 38 mV under the 3.6 V absolute maximum'; and 'DEF must also stay high: at 3.3 V nominal the worst-low 3.224 V leaves only 74 mV' of charge-entry margin (OSEL 3.0 V + VBYPASS 150 mV = 3.150 V).

**Consequence.** On a worst-case regulator sample the rail's PSM peaks can exceed 3.6 V at the S3s' VDD pins, which is an absolute-maximum excursion (not immediate damage, but outside the guaranteed envelope) on the always-on OC and, once switched, on the FC, GNSS and both NORs. The margin argument in the doc omits the ripple term, so the true margin is unmeasured.

**Fix.** Measure, then decide (rail-level decision, owner's): scope V_BUCK and +3V3 at U15 pin 20/46 on the first article at OC-only load (PSM) with pack and with USB, record peak. If peaks exceed ~3.58 V, the options are DEF low (3.3 V nominal; the cap still charges with 74 mV worst-case margin at OSEL 3.0 V) or a smaller COUT-driven ripple; the V9 precedent is DEF low.

#### 31. [Note] Buck input capacitors sit 2.6-3.9 mm from PVIN and return through the In6 plane, not local copper

- **Refs:** U18 pins 11/12 (75.35/75.85,130.47), C43 (79.13,128.76), C65 (77.69,128.81), PGND pins 15/16 (76.535,132.155/132.655)
- **Nets:** Net-(U18-AVIN), GND
- **Reviewer:** `mini-pcb-power-5`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** The 22 uF input cap C43 is 3.86 mm from PVIN and its GND pad is on B.Cu GND island 1 while U18's PGND/EPAD are on island 10; the return closes through vias into In6 (0.116 mm below B.Cu). The V9 puts C43 1.83 mm away (on the opposite side) and C65 2.36 mm; the mini is looser but the same order.

**Evidence.** Pad distances measured from geom.json (C43.1->U18.11 3.86, C65.1->U18.11 2.62, C43.2->U18.15 5.06); gnd.py island membership (C43.2/C65.2 island 1: 411.8 mm2/232 vias; U18.15/16/17 island 10: 216.9 mm2/91 vias); C43.2 has a via at its pad, U18.16 has a via in its pad. V9 board: C43.1 1.83 mm from PVIN, C65 2.36 mm, L6 3.20 mm from SW (mini 2.76). SW node pour Net-(L6-Pad1) is 5.2 mm2, 1.0-1.55 mm wide, 2.8 mm to L6.1. VOS: pin 14 -> 0.2 mm track 1.3 mm to C53 (10 uF); L6.2 reaches C53 on a separate 0.2 mm track 4.9 mm long while the power path L6 -> C135/C141 -> U47 is a pour 1.8-8 mm wide, matching the datasheet's 'separate VOS from the VOUT power line'. Output caps: C135 22 uF 2.9 mm from L6.2 with a GND via 0.24 mm away, C141 10 uF at 4.9 mm.

**Consequence.** Input loop inductance is a little higher than the V9 or the TI layout example; with the AVIN pour 5-6 mm wide over a solid In6 plane the loop area is small (~0.5 mm2 equivalent) and the fabbed V9 works with a comparable geometry. Cosmetic ripple/EMI, not function.

**Fix.** At the next spin move C65 (100 nF) to abut pins 11/12 with a GND via at its pad; leave C43 where it is.

#### 32. [Note] U30 has no local +3V3 input capacitor; the switched rail's ~96 uF is charged from In3 through two vias

- **Refs:** U30 pin 6 (84.93,139.04), vias (84.43,138.17) and (87.01,141.55), C17/C143 (86.88/88.86,133.7)
- **Nets:** +3V3, V_MCU_SWTCH
- **Reviewer:** `mini-pcb-power-6`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** U30 VIN is a 1.28 mm2 B.Cu island fed by two 0.2 mm tracks from two In3 vias; the nearest +3V3 capacitors are C17/C143 (2 x 22 uF) 4-6 mm away at U47 and the OC's 0402s on the F side. TPS22810 section 10.3 recommends (optionally) 1 uF at VIN.

**Evidence.** spof.py +3V3: U30.6 min-cut 2 vias, attached tracks 0.2 mm; netlist: no capacitor on +3V3 within U30's bbox (83.0-85.56 x 138.36-142.17); V_MCU_SWTCH capacitors from the netlist: C3/C4/C5/C121/C125 10 uF, C23/C37 22 uF, C119/C124 1 uF, ten 100 nF, C117/C39 10 nF = ~96 uF nominal, ~50 uF effective at 3.4 V for these 0402/0805 X5R parts. TPS22810 Table 2: CT = 10 nF (C104) gives a 550 us 10-90 % rise at 3.3 V, so the charge current is ~50 uF x 3.3 V / 0.55 ms = 0.3 A for half a millisecond. +3V3 B pour to In3: 4 vias (86.93,134.43) (86.71,133.99) (85.96,135.74) (89.11,133.98); narrowest cut of that pour 0.60 mm at x=85.9 beside VOUT pins 9/10 (1 oz, ~1.6 A at 10 C).

**Consequence.** The slew-limited 0.3 A step is small against the plane and 44 uF at U47; no dip that matters is expected. Recorded because the datasheet asks for the cap and because the FC-rail capacitance total was requested.

**Fix.** Optional: a 1 uF 0402 on +3V3 at U30 pin 6 (room at about (85.6,139.0) on B.Cu) at the next spin.

#### 33. [Note] Supercap can rests on the 2.5 mm LoRa module shield; pads and polarity mark are correct

- **Refs:** C130 (83.95,129.46, B side), U16 (89.32,114.49, B side), pads (81.45,129.46) V_SCAP and (86.45,129.46) GND
- **Nets:** V_SCAP, GND
- **Reviewer:** `mini-pcb-power-7`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** The can outline (B.Fab 78.95-88.95 x 106.96-126.96) overlaps U16 (84.22-94.56 x 109.39-121.21), which is 2.5 mm tall with its shield, so the lying can is propped 2.5 mm off the board at that end and the RTV bonds to the module shield and to U19/U21/U23 rather than to bare copper; FABRICATION-NOTES B8 already documents 26 parts under the can and a 2.5 mm bend allowance. Silk '+' is on the pad-1 (V_SCAP) side; both pads connect through 4 x 0.5 mm thermal spokes per layer.

**Evidence.** Footprint graphics from pcbnew: B.Fab rectangle (78.95,106.96)-(88.95,126.96), B.Silkscreen text '+' at (79.96,129.39) beside pad 1 at (81.45,129.46) whose net is V_SCAP (netlist C130.1 V_SCAP, C130.2 GND). E220 manual section 3: size 10*10*2.5 mm with shield. Zone settings: V_SCAP zone pad connection THT_THERMAL, gap 0.25, spoke 0.5 (B.Cu only); GND zone spoke 0.5 gap 0.5 on F/B/In1/In2/In5/In6; C130.2 is the only THT GND pad on the board. Four 0.5 mm x 35 um spokes = 2 mm of 1 oz per layer, fine for the 100 mA charge and the ~1 A boost draw at low cap voltage. V_SCAP pour 12.5 mm2, 2.6-4.2 mm wide to L11.2; SUP pin 6 reaches the node on a 0.2 mm track (sense/supply pin, not inductor current; CSUP C142 10 uF 1.0 mm from the pin).

**Consequence.** None electrically; mechanically the can is not flat on the board, so the lead bend and the RTV fillet must be made with the 2.5 mm standoff in mind (already in B8).

**Fix.** None; keep B8 as written. If a flat bond is wanted the can would have to move off U16, which is a placement change for the owner.

#### 34. [Note] Single-via GND returns for the OC's 32 kHz load caps, its RTC decoupling and the OC boot-NOR ground ball

- **Refs:** F.Cu GND island 90.09-92.69 x 128.69-132.91 (C19.2, C21.2, C29.2, R142.2, 1 via); F.Cu island 80.50-81.52 x 136.37-138.44 (U13.E3, C152.2, 1 via)
- **Nets:** GND
- **Reviewer:** `mini-pcb-power-8`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** Two F.Cu GND pockets each hang on one via: Y1's 22 pF load caps C19/C21 and the U15 VDD3P3_RTC 100 nF C29 share a 6.1 mm2 island with one via, and U13's GND ball E3 shares a 1.1 mm2 island with C152 and one via. All other GND copper on every layer has at least one via or a through-hole pad; In1 and In6 are single unbroken islands.

**Evidence.** gnd.py island tables (F.Cu 25 islands, all with >=1 via; In2 15, In5 25, B.Cu 32, all with >=1 via; In1/In6 one island each with all 387 GND vias). Netlist: C19/C21 22 pF, C29 100 nF, C152 100 nF; U13.E3 GND_E3.

**Consequence.** Small currents, so nothing fails; the crystal and RTC-domain return share one via's inductance, which is the classic cause of noisy 32 kHz starts. Below the threshold of a defect.

**Fix.** Add one via to each of the two islands at the next spin (room at about (91.5,132.5) and (81.0,137.0)).

#### 35. [Note] #1009 closed as 'connector changed', but J3 is still the 2 A JST-PH B2B-PH-SM4-TB in the fabbed and live netlists and BOM, and pyro pulses pass through it

- **Refs:** J3, Q11, R72, U6/U7/U8/U10, U9, J2
- **Nets:** VBAT_J8, VBAT_Terminal, VBAT_CON, PYROn_EXT, PYRO_GND, GND
- **Reviewer:** `mini-power-2`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** Issue #1009 (review finding 16) noted J3 was 'still the JST-PH (2 A per contact) while pyro now fires from the pack through it' and was closed 2026-09-03 with 'connector changed and the silk + corrected'. The connector was not changed in the design files: J3 = JST_B2B-PH-SM4-TB / MPN B2B-PH-SM4-TB in the v1.0.1 netlist, the live netlist and bom.csv row 19; git shows no change to that value since 2026-08-15. The fire path has no series element: channel P-FET source on VBAT_CON, drain on PYROn_EXT -> J2 -> match -> PYRO_GND -> U9 AON7534 -> GND; the whole pulse returns through J3 pin 1 and leaves through J3 pin 2 / Q11 / R72.

**Evidence.** dump.py comps live-mini and mini-v101: J3 'JST_B2B-PH-SM4-TB Footprints:JST_B2B-PH-SM4-TB B2B-PH-SM4-TB'; bom.csv line 20: "J3";"JST_B2B-PH-SM4-TB";...;"B2B-PH-SM4-TB";"JST";"Fit". git log -S'B2B-PH-SM4-TB' -- hardware/rocket-computer-mini: last touch ee41b18c 2026-08-15. gh issue view 1009: close comment 'Fixed by the owner on the board — connector changed and the silk + corrected'. Netlist: U6.1-3 SOURCE=VBAT_CON, U6.5-8 DRAIN=PYRO2_EXT; PYRO1_EXT: J2.1_1/1_2, U8 drains, R8/R9 (continuity 49.9 k) only; PYRO_GND: J2.5_1/5_2, R73, U9 drains 5-8; U9 sources GND. V9 for comparison: J8 = JST_B2P-VH (B2P-VH(LF)(SN)), and R20 150 R limited the fire path there. Q11 AONR21321 datasheet: ID -24 A, IDM -66 A, RDS(on) <29.5 mOhm at -4.5 V — fine. R72 Yageo PA0805 series is a 1 W-class 0805 (PA0805FRF870R01L listed 1 W at DigiKey; the 2 m sibling not individually confirmed): 6 A pulse = 72 mW — fine.

**Consequence.** A pack-direct e-match pulse of several amps (8.4 V into ~1-2 ohm plus FET and wiring) passes through PH contacts rated 2 A continuous. Brief pulses will normally survive, but the rating is exceeded on every firing and the closed issue reads as if it were fixed. Q11, R72 and the copper are not the limit; the connector is.

**Fix.** Owner question, not a drawing change: confirm what was actually fitted/intended at J3 on the first article (PH as drawn, or a VH/XT30 hand-change), and if the PH stays, record the accepted pulse duty on it in README/#1009 so the closed issue matches the design files. If a change is wanted, it is a connector-contract decision (VH like the V9, or XT30) for discussion.

#### 36. [Note] Hold-up ends in a hard cut into forced bypass when V_BUCK decays to the EN/MODE low band — not the 'benign, voltages equal' end the rework note describes

- **Refs:** U47 EN (pin 3), MODE (pin 2), VIN (pin 4), R137/R138, C135/C141/C53, U18
- **Nets:** V_BUCK, +3V3, V_SCAP
- **Reviewer:** `mini-power-4`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** EN and MODE are both tied to V_BUCK. After pack loss the boost holds +3V3 at 3.0 V from C130 while V_BUCK decays through R137+R138 (460 k) and leakage. When V_BUCK falls into the EN/MODE low band (VEN_L min 0.2 V, VEN_H max 0.58 V at VOUT > 1.8 V) the part enters Forced bypass (EN = 0, MODE = 0): bypass FET ON, boost OFF, VOUT = VIN. At that instant +3V3 is 3.0 V and V_BUCK is <= 0.58 V, so the rail is dumped into the dead V_BUCK node (~42 uF) and the hold-up ends abruptly at ~34-54 s (tau ~19 s per the doc), well before the cap is spent at OC-only load (~90 s on energy). The rework note says 'Voltages are equal by then, so it is benign' — they are not equal; the consequence (rail ends) is the same but the mechanism is a cut, not a fade, and it caps the standby hold. The V10 shares the EN/MODE = VIN topology.

**Evidence.** Netlist live-mini: U47.2 MODE -> V_BUCK, U47.3 EN -> V_BUCK, U47.4 VIN -> V_BUCK; V_BUCK members C135 22u, C141 10u, C53 10u, R137 100k (-> VBUCK_OK -> R138 360k -> GND). tps61094.txt lines 428-446: VEN_H max 0.58 V / VEN_L min 0.2 V (VOUT > 1.8 V), 800 k internal pulldowns; Table 7-4 (lines 853-860): 'Forced bypass 0 0: Turn on bypass MOSFET, turn off boost/buck, VOUT = VIN'; line 867-871: in Forced bypass 'it cannot protect the reverse current from output to input'. holdup-tps61094-rework.md 'Consequences to know about on the bench': 'EN crosses 0.58 V about 34 s after pack loss and truncates the hold there ... The hold terminates in forced bypass, not shutdown ... Voltages are equal by then, so it is benign'. Note also that the buck's high-side body diode couples V_BUCK to the AVIN/V_MCU_2S caps once VIN < VOUT, so the 19 s tau is an estimate in both directions.

**Consequence.** At flight loads (cap spent in ~21 s) this never binds. For pad-standby/OC-only survival after a pack drop-out the hold is 34-54 s, not the ~90 s the energy suggests, and it ends with a step to ~0.3 V rather than a decline the OC could log. No design change is implied (decided topology); the doc statement should not be relied on.

**Fix.** Correct the sentence in holdup-tps61094-rework.md (owner's text) and keep the existing 'measure it on the first article' item: yank the pack at OC-only load and record the time and shape of the +3V3 collapse. If a longer standby hold is ever wanted, the doc's 100 k/test-pad EN affordance or an RC on EN from V_SCAP is a discussion item, not a drawing.

#### 37. [Note] TPS62152 input capacitance at PVIN is below the datasheet's 10 uF after DC-bias derating — identical to the working V9, recorded as a clearance-with-caveat

- **Refs:** U18 PVIN (pins 11/12), C43 22 uF 0805 16 V, C65 100 nF, L5, C47/C48/C49
- **Nets:** Net-(U18-AVIN), V_MCU_2S
- **Reviewer:** `mini-power-5`, confidence 0.7
- **Verification:** not run (single source).

**Claim.** The buck's local input capacitor is C43 (22 uF 0805 16 V X5R, 3.86 mm from PVIN pad 11) plus C65 100 nF (2.62 mm); L5 (2.2 uH, 74 mOhm) separates it from the 3 x 22 uF on V_MCU_2S. At 8.4 V bias the repo's own C56 note puts this part at ~18 % retention (~4 uF), i.e. roughly 4-11 uF effective against the datasheet's '10 uF is sufficient and is recommended'. The V9 carries exactly the same C43/C65/L5 arrangement and is the working reference, so this is an accepted margin, not a new defect.

**Evidence.** Netlist live-mini: Net-(U18-AVIN): C43.1, C65.1, L5.2, U18.10/11/12/13; V_MCU_2S: C47/C48/C49 22 uF, C59 1 uF, L5.1. V9 netlist: Net-(U18-AVIN): C43.1, C65.1, L5.2, U18.10-13 (C43 22 uF, C65 100 nF, L5 VLS3012CX-2R2M-1 — same values). pcbnew live-mini: U18.11 -> C43.1 3.86 mm, -> C65.1 2.62 mm. tps62152.txt 9.2.2.2.2.2: 'For most applications, 10 uF is sufficient and is recommended ... placed between PVIN and PGND as close as possible'. power-budget.md C56 section: 'At 8.4 V these parts retain about 18% of their rating — roughly 4 uF each' (single-sourced).

**Consequence.** Higher input ripple and a lower-damped L5/C43 input filter than the datasheet baseline; the doc's Middlebrook check (16-27 dB) already covers stability. No functional consequence expected given the V9 precedent.

**Fix.** None required. If a spin is made anyway, a second 22 uF 0805 at PVIN (same line item as C43) restores the datasheet figure; otherwise record the derated value in power-budget.md.

#### 38. [Note] U30 has no local input capacitor and its output capacitance exceeds its input capacitance — benign here, recorded with numbers

- **Refs:** U30 TPS22810 VIN (pin 6), C17/C143, C26/C31, V_MCU_SWTCH capacitor set, C104 CT 10 nF
- **Nets:** +3V3, V_MCU_SWTCH
- **Reviewer:** `mini-power-6`, confidence 0.75
- **Verification:** not run (single source).

**Claim.** The TPS22810 datasheet asks for a 1 uF ceramic close to VIN (Recommended Operating Conditions CIN 1 uF; 10.3) and recommends CIN > CL (10.4) so VOUT cannot exceed VIN through the body diode when the supply is removed. On the mini the nearest +3V3 capacitors to U30.6 are C31 100 nF at 3.06 mm (top side, U30 is on the bottom), C26 100 nF at 3.80 mm and C17 22 uF at 5.68 mm; V_MCU_SWTCH carries ~97 uF nameplate (C23/C37 22 uF 0805, C3/C4/C5/C121/C125 10 uF 0402, 2 x 1 uF, 11 x 100 nF, 2 x 10 nF) = roughly 50-55 uF effective at 3.3 V, against +3V3's ~42 uF effective. Inrush at turn-on with CT = 10 nF is bounded and small; the CL > CIN case only back-feeds +3V3 from the FC rail at pack loss, which helps rather than hurts. Same U30/CT/R84/C105 values as the V9.

**Evidence.** tps22810.txt line 256: 'CIN Input capacitor 1 (2) uF'; 10.3: 'A 1-uF ceramic capacitor, CIN, placed close to the pins, is usually sufficient'; 10.4: 'a CIN greater than CL is highly recommended. A CL greater than CIN can cause VOUT to exceed VIN when the system supply is removed'; 9.3.4: SR = 46.62 / CT(pF) V/us -> 4.66 mV/us at 10 nF, Table 2: 550 us 10-90 % at 3.3 V. pcbnew live-mini distances from U30.6: C31 3.06, C26 3.80, C145 5.66, C30 5.75, C17 5.68 (B), C143 6.65 (B). Netlist V_MCU_SWTCH capacitors as listed. Inrush = 55 uF x 4.66 V/ms = ~0.26 A for ~0.7 ms (0.45 A at nameplate) through U47's 150 mOhm bypass = ~40-70 mV transient dip on +3V3, inside the buck's 1 A. V9 netlist: U30 pins identical (CT C104 10 nF, EN/UVLO POWER_SWITCH with R84 100 k/C105 10 uF/D9).

**Consequence.** A ~50 mV +3V3 dip for under a millisecond when the OC enables the FC rail; the OC's own decoupling (C32/C36 10 uF, C30/C35 1 uF within 3-4 mm of U15 pin 46) absorbs it. No defect; recorded so the synthesis has the switching transient numbers.

**Fix.** None needed. If the +3V3 dip is ever visible in the OC's brownout log at FC power-up, a 1 uF 0402 at U30 pin 6 is the datasheet's answer.

#### 39. [Note] TPS2121 IN1 (USB) has no capacitor near the pin — C76 1 uF sits at the connector 42 mm away; same schematic as the V9

- **Refs:** U21 IN1 (pin 7), C76, J6, CR3, C8/C54 (IN2 side)
- **Nets:** Net-(J6-VBUS), VBATT, V_MCU_2S
- **Reviewer:** `mini-power-7`, confidence 0.7
- **Verification:** not run (single source).

**Claim.** The only capacitor on the VBUS net is C76 1 uF, placed beside J6 on the top side; U21 pin 7 is on the bottom side 42.1 mm away. The TPS2121 datasheet asks for bypass capacitors on IN1/IN2/OUT 'as close to the device as possible' and warns of ringing on long cable inputs during fast switchover. The pack input (IN2 = VBATT) has C8 22 uF + C54 1 uF near U19's output. The V9 has the identical VBUS net (C76 1 uF, CR3, U21.7); its layout was not compared.

**Evidence.** Netlist live-mini Net-(J6-VBUS): C76.1, CR3.2, J6.A4/B9, J6.B4/A9, R51.2, R62.2, U21.7 — no other capacitor. pcbnew live-mini: U21.7 (B) -> C76.1 (F) 42.10 mm; C76 at (77.37,166.16), U21 at (81.76,123.39). tps2121.txt Section 11: 'Bypass capacitors on these pins should be placed as close to the device as possible ... where there are long cables ... there may be ringing on the supply, especially during the fast switchover'. V9 netlist Net-(J6-VBUS): C76.1, CR3.2, J6 VBUS pads, R51.2, R62.2, U21.7 (same).

**Consequence.** USB-only operation is unaffected (soft start C52 1 uF limits inrush to ~88 V/s; the 3 x 22 uF on OUT are the hold-up). At a USB unplug with the pack present the fast switchover draws from OUT, not IN1, so the missing IN1 cap costs only some VBUS ringing at plug-in. Not a first-article factor.

**Fix.** None required for function; at a spin a 1 uF 0402 at U21 pin 7 (bottom side) is the textbook placement.


### The two processors

#### 40. [Minor] U14 chip antenna: placement and copper cut-out deviate from Abracon's reference (rotated 90°, ground rail through the clearance, feed 2.35 mm from ground)

- **Refs:** U14, L1, C7, C12, U15 pin 1; V10: U22, L2, C23, C12
- **Nets:** Net-(U15-LNA_IN), Net-(C12-Pad1), GND
- **Reviewer:** `mini-oc-s3-1`, confidence 0.7
- **Verification:** not run (single source).

**Claim.** The mini places the AANI-CH-0070 with its 1.0 mm axis perpendicular to the board edge, its GND terminals on a continuous 0.4 mm edge ground rail that runs through the clearance zone on six copper layers, and a copper-free area that is 2.8 mm deep from that rail; Abracon's reference (datasheet Rev A p.4) has the antenna parallel to the edge, 0.2–0.5 mm from a bare edge, a 4.60 × 3.50 mm cut-out with no copper on any layer, the feed pads 0.75 mm from the ground plane and only a 0.30 mm strip from the GND pads to one side.

**Evidence.** pcbnew, live mini board: U14 at (94.125,145.470) rot 0, pads 1/4 FEED at x=93.85, pads 2/3 GND at x=94.40; board edge x=95.015 (0.89 mm away). Per-layer copper map (work/mini-oc-s3/cumap.py): copper-free region x 91.5–94.2, y 144.6–149.1 on F/In1/In2/In5/In6/B; a GND fill column at x 94.3–94.7 on all six GND layers spans y 141.5–149.5+ (i.e. both sides of the antenna), stitched by GND vias at 1.25/1.40/1.81/1.91 mm from the antenna; In3 (+3V3) and In4 (V_MCU_SWTCH) planes stop at x≈91.0 for y>144.6 (clear). Abracon AANI-CH-0070 datasheet Rev A 12-20-24, 'Recommended PCB layout': 'Recommended Ground Clearance for Antenna 4.6 x 3.5 mm'; 'The rectangular copper cutout in the footprint must extend through all layers of the PCB stack-up, ensuring there is no copper on any layer in this area'; detail drawing: feed pads 0.75 mm and GND pads 1.40 mm from the ground edge, 0.30 mm GND strip 0.50 mm below the PCB edge, antenna long axis along the edge. Part terminal map (p.3): 1/4 FEED, 2/3 GND, 0.59 mm along the long axis — footprint pitch 0.55 (x) × 0.40 (y) confirms the long axis is along x = perpendicular to the edge. V10 live schematic copies the same antenna and match (U22, L2 2.2 nH, C23 5.1 pF, C12 DNP); V10 placement is in progress.

**Consequence.** The loop antenna is detuned relative to the vendor's characterisation (resonance/efficiency shift of unknown size); BLE range to the phone app is lower than the datasheet's 70 % efficiency implies. Not a connectivity fault: the chip still radiates and the match is tunable.

**Fix.** Mini: measure the BLE link (RSSI vs a devkit) on the bench and tune with the fitted placeholders (C12 DNP shunt, C7 series, L1 shunt) — the Abracon EVB values 5.1 pF / 2.2 nH are fitted verbatim. Next mini spin and the V10 placement now: put the antenna at a board corner (Abracon option 1) with its long axis along the edge, a 4.6 × 3.5 mm all-layer copper-free rectangle, feed 0.75 mm from the ground plane, GND terminal returned by a single 0.3 mm strip, no edge ground rail through the cut-out, and stitching vias around (not inside) the cut-out. Log the V10 item in #1365.

#### 41. [Minor] 32.768 kHz crystal Y1: traces 2.6× longer than the V9's and top-layer signal traces under the crystal, while firmware selects it as the RTC slow clock

- **Refs:** Y1, C19, C21, U15 pins 21/22
- **Nets:** Net-(U15-XTAL_32K_P), Net-(U15-XTAL_32K_N), L_CS, OC_ARM_EN
- **Reviewer:** `mini-oc-s3-2`, confidence 0.5
- **Verification:** not run (single source).

**Claim.** The mini's 32 kHz crystal nets are 6.47 mm (P) and 11.00 mm (N) of 0.1 mm trace on F.Cu versus 3.95/4.23 mm on the fabbed V9, and two F.Cu signal traces (L_CS, OC_ARM_EN) pass under the crystal body; the out-computer firmware uses this crystal (CONFIG_RTC_CLK_SRC_EXT_CRYS=y).

**Evidence.** pcbnew live mini: Y1 at (91.46,130.58), U15 pad 21 (88.71,133.36), pad 22 (88.31,133.36); XTAL_32K_P 15 segs 6.47 mm, XTAL_32K_N 15 segs 11.00 mm (V9: 6 segs 3.95 mm / 6 segs 4.23 mm, Y1 1.83 mm from pad 21). Items inside Y1 bbox (x 89.37–93.55, y 129.44–131.72): F.Cu L_CS and OC_ARM_EN plus the crystal's own nets; In1 GND fill 44/45 sample points under the crystal (inner-layer traces GNSS_RX/TX, IND_1/2, L_BUSY, L_RXEN, M_MOSI are below the In1 plane). C19/C21 22 pF with a 12.5 pF-CL crystal (ABS07) → CL_eff ≈ 11 pF + stray ≈ 13–14 pF. tinkerrocket-idf/projects/out_computer/sdkconfig.defaults:114 CONFIG_RTC_CLK_SRC_EXT_CRYS=y. Espressif S3 layout guideline: 'It is best not to route any signal trace under the crystal'; checklist: ESR ≤ 70 kΩ, no parallel resistor needed in general (none fitted, same as V9).

**Consequence.** A 32 kHz oscillator that fails to start is not fatal (ESP-IDF falls back to the internal RC with a boot warning) but the fallback silently degrades RTC timekeeping and light-sleep timing; a marginal start-up would look like intermittent warnings across boards.

**Fix.** On the bench, look for the IDF '32 kHz XTAL' fallback warning in the OC boot log and, if seen, confirm with a scope probe on XTAL_32K_N. Next spin: move Y1 next to pads 21/22 (as on the V9), keep L_CS/OC_ARM_EN off the footprint, and consider 18 pF caps if the measured CL is high.

#### 42. [Minor] U32 exposed pad has 5 ground vias; Espressif asks for at least nine (U15 has 10)

- **Refs:** U32
- **Nets:** GND
- **Reviewer:** `mini-pcb-usb-mcu-3`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** Inside U32's 4.1 × 4.1 mm EPAD (centre 84.525,150.86) there are 5 GND vias (0.3 mm drill); U15's EPAD (88.51,136.785) has 10. The FC_D± pair on In2 runs under the EPAD at x = 84.43/84.63, y 148.8–151.8, which is presumably why the via field is thin there.

**Evidence.** epad.py: U32 vias inside 5 {GND:5}, U15 10 {GND:10}; Espressif ESP32-S3 PCB layout guideline: 'The ground pad at the bottom of the chip should be connected to the ground plane through at least nine ground vias.'

**Consequence.** Weaker thermal path and ground return for the FC; not a functional blocker at the mini's duty but below the vendor rule, and asymmetric with the OC.

**Fix.** Add ≥4 more 0.3 mm filled/capped GND vias in the EPAD quadrants clear of the In2 USB pair (x < 84.3 or x > 84.8).

#### 43. [Note] The FC has NO antenna: U32 LNA_IN (pad 1) is a no-connect; that is acceptable per Espressif's checklist because the M1 FC build never initialises the RF stack - but the review brief's description of the RF parts is wrong

- **Refs:** U32 pad 1, L3/L9 (24 nH), L4/L10 (2 nH), L1/C7/U14; sdkconfig.defaults.m1, flight_computer/CMakeLists.txt:22
- **Nets:** unconnected-(U32-LNA_IN-Pad1), Net-(U32-XTAL_P), Net-(U32-VDD3P3)
- **Reviewer:** `mini-fc-s3-2`, confidence 0.95
- **Verification:** not run (single source).

**Claim.** The brief says 'the FC's antenna is a second match (L3/L4/L9/L10 are the two ESP32 RF matches)'. In the netlist L3 and L9 are the 24 nH series inductors on the two XTAL_P nets (Net-(C20-Pad1): C20.1 L3.1 Y2.3; Net-(C112-Pad1): C112.1 L9.1 Y4.3) and L4/L10 are the 2 nH chokes feeding VDD3P3 pins 2/3 (Net-(U32-VDD3P3): C122.1 L10.1 U32.2 U32.3). The only RF match on the board is the OC's L1 2.2 nH / C7 5.1 pF into U14. U32.1 is 'unconnected-(U32-LNA_IN-Pad1)', pintype no_connect, no track on the net; the board pad still receives a paste flash at (81.100, 148.260) which is harmless.

**Evidence.** dump.py pins U32 -> 'U32.1 LNA_IN_1 bidirectional+no_connect unconnected-(U32-LNA_IN-Pad1)'; pcbnew: 0 tracks on that net; F_Paste.gtp flash X81100000Y-148260000D03. Espressif ESP32-S3 schematic checklist (fetched 2026-09-21): 'If RF function is not required ... do not initialize the RF stack ... RF pin can be left floating' and 'if RF function is enabled, make sure an antenna is connected. Operation without an antenna may result in unstable behavior or potential damage'. flight_computer/sdkconfig.defaults.m1:40-41 CONFIG_BT_ENABLED=n, CONFIG_ESP_WIFI_ENABLED=n; flight_computer/CMakeLists.txt:22 EXCLUDE_COMPONENTS TR_BLE_To_APP TR_LoRa_Comms TR_INA230. The one ERC item on fc_esp32s3.kicad_sch is 'endpoint_off_grid' on U32 pin 1.

**Consequence.** No electrical defect. The risk is purely a future firmware change: any M1 FC build that turns BT/Wi-Fi back on (e.g. copying the OC's sdkconfig) would run the PA into an open pad, which Espressif warns can damage the part.

**Fix.** Correct the brief/synthesis text (L3/L9 = crystal series inductors, L4/L10 = VDD3P3 chokes, FC has no antenna). Optionally add a static guard in flight_computer/config.h: '#if TR_BOARD_M1 && (CONFIG_BT_ENABLED || CONFIG_ESP_WIFI_ENABLED) #error U32 has no antenna'. Snap U32 pin 1 to grid to clear the ERC item (owner's sheet).

#### 44. [Note] GPIO3 carrying the VBUCK_OK divider is safe only while EFUSE_STRAP_JTAG_SEL stays unburned; once burned, the divider's hold-up level (~2.39 V) falls inside the S3's undefined input band

- **Refs:** U32 GPIO3 (pad 8), R137 100 k, R138 360 k, C152 100 nF
- **Nets:** VBUCK_OK, V_BUCK
- **Reviewer:** `mini-fc-s3-3`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** VBUCK_OK = V_BUCK x 360/(360+100) = 0.783 x V_BUCK (about 2.71 V regulating, about 2.39 V on cap energy per board_m1.h). Datasheet Table 3-5: with the default eFuses (DIS_PAD_JTAG=0, DIS_USB_JTAG=0, STRAP_JTAG_SEL=0) GPIO3 is 'Ignored' and the JTAG source is the USB Serial/JTAG controller, so today the strap is inert and the pad is a plain ADC1_CH2 input (Table 2-10 row 8). Section 3.4 also says GPIO3 'does not have any internal pull resistors and the strapping value must be controlled by the external circuit' - the divider does that. If STRAP_JTAG_SEL were ever burned, 2.71 V > VIH (0.75 x 3.3 = 2.475 V, Table 5-4) reads 1 = USB JTAG, but 2.39 V during hold-up is between VIL (0.825 V) and VIH: a reset while running on the supercap would sample an undefined JTAG source. Back-injection when V_MCU_SWTCH is off: V_BUCK is upstream and always up, so the divider pushes at most (3.46 - ~0.5)/100 k = ~30 uA into the unpowered pad; against U30's QOD (265-350 Ohm) that is ~10 mV on the rail - negligible.

**Evidence.** Netlist: VBUCK_OK: C152.1 R137.2 R138.1 U32.8[GPIO3]; V_BUCK: R137.1 ...; GND: R138.2. Datasheet lines 1729-1760 (Section 3.4, Table 3-5), Table 5-4 VIH/VIL, Table 2-10 GPIO3 = ADC1_CH2. board_m1.h VBUCK_OK_PIN comment already records the eFuse dependency. Decisions.md: VBUCK_OK on FC GPIO3 as ADC is an owner decision.

**Consequence.** None today. A future eFuse burn (e.g. to force pad JTAG for debugging) would make the FC's boot JTAG source depend on whether the buck or the supercap is supplying at that instant.

**Fix.** No schematic change. Record 'never burn EFUSE_STRAP_JTAG_SEL / EFUSE_DIS_USB_JTAG on the mini FC' in firmware-notes.md next to the existing header comment; the C152 100 nF already meets the checklist's ADC filter recommendation.

#### 45. [Note] 40 MHz crystal load: 12 pF caps on a 10 pF-CL crystal give ~9 pF effective, a few ppm high — identical to the working V9, so a bench measurement rather than a change

- **Refs:** Y2, C20, C22, L3 (V10: same refs; FC copy Y4/C112/C113)
- **Nets:** Net-(U15-XTAL_N), Net-(U15-XTAL_P), Net-(C20-Pad1)
- **Reviewer:** `mini-oc-s3-3`, confidence 0.5
- **Verification:** not run (single source).

**Claim.** ECS-400-10-37B2 is a 10 pF-CL, 40 Ω-ESR crystal; C20 = C22 = 12 pF gives 6 pF plus stray (2 pF pin capacitance + ~1 pF of trace on the 12.6 mm XTAL_N run) ≈ 9 pF, so the oscillator runs slightly above nominal (order 10–20 ppm depending on pullability).

**Evidence.** Netlist: C20 12 pF (Net-(C20-Pad1) = Y2.3 + L3.1), C22 12 pF on XTAL_N; L3 24 nH in series on XTAL_P (Espressif checklist: 'inductor of 24 nH to reduce the impact of high-frequency crystal harmonics' — matches). Distributor listing for ECS-400-10-37B2-CKY-TR: 40 MHz, 10 pF, ±10 ppm, 40 Ω ESR (ecsxtal.com returned 403; value not read from the vendor PDF). S3 datasheet Table 5-4 CIN 2 pF. Espressif formula CL = C1·C4/(C1+C4) + Cstray. V9 (fabbed, BLE working) has the same C20/C22 12 pF and a 13.8 mm XTAL_N trace.

**Consequence.** A frequency offset of this order is inside BLE's ±50 ppm budget but eats into Espressif's ±10 ppm recommendation; no functional failure expected.

**Fix.** Measure the BLE carrier or the 40 MHz on the bench once a board boots; if it reads > +10 ppm, fit 15 pF on the next spin. No change now.

#### 46. [Note] OC decoupling follows the V9, not the guideline or the V10: no 10 µF at VDD3P3 pins 2/3, no 0.1 µF at VDD_SPI, no external GPIO0 pull-up

- **Refs:** C33, L4, C36, C27, SW3 (V10 added C148 10 µF, C145 100 nF, R141 10 k)
- **Nets:** Net-(U15-VDD3P3), OC_VDD_SPI, Net-(U15-GPIO0), +3V3
- **Reviewer:** `mini-oc-s3-4`, confidence 0.6
- **Verification:** not run (single source).

**Claim.** The mini's U15 decoupling set is identical to the fabbed V9's (C33 100 nF behind L4 2 nH on VDD3P3, C27 1 µF alone on VDD_SPI, C28–C36 on +3V3, GPIO0 on the internal 45 k pull-up only). The V10 live schematic has since added C148 10 µF on VDD3P3, C145 100 nF on VDD_SPI and R141 10 k on GPIO0, which is what Espressif's checklist asks for; the mini did not get those.

**Evidence.** Netlist live-mini: Net-(U15-VDD3P3) = C33.1, L4.1, U15.2, U15.3; OC_VDD_SPI = C27.1, U15.29; Net-(U15-GPIO0) = SW3.2, U15.5. V9 identical. V10 live: Net-(U15-VDD3P3) adds C148 (10 uF 0402), OUT_VDD_SPI adds C145 (100 nF), Net-(U15-GPIO0) adds R141 10 k to +3V3. pcbnew mini: C33 pad 1.58 mm from pin 2, C36 10 µF 1.85 mm from L4 on the +3V3 side; C27 1.19 mm from pin 29; C29 100 nF 1.24 mm from pin 20; C31 100 nF 1.19 mm from pin 46; C32 10 µF / C30 1 µF 1.1–1.5 mm from pins 55/56; C25 1 µF 2.75 mm from CHIP_PU. Espressif S3 checklist: 'highly recommended to add a 10 μF capacitor to the power rail' for pins 2/3, 'add extra 0.1 μF and 1 μF decoupling capacitors close to VDD_SPI', 'It is recommended to place a pull-up resistor at the GPIO0 pin'; layout page: 'place a 10 µF capacitor for each pin' (2 and 3). GPIO0 net on the mini is 12.68 mm with 2 vias to SW3.

**Consequence.** None demonstrated — the V9 boots and radios on the same set. Slightly higher RF supply ripple and a marginally larger chance of a noise-induced download-mode strap on a long GPIO0 trace.

**Fix.** Next mini spin: add a 10 µF at the VDD3P3 node (chip side of L4), a 100 nF beside C27, and a 10 k pull-up on GPIO0, matching the V10. Nothing for the first article.

#### 47. [Note] Housekeeping on the OC sheet: four off-grid pin ends in the antenna/crystal/NOR wiring, U15 footprint differs from its library copy, C22/Y2 silk overlap

- **Refs:** U15 pin 1, L1 pin 1, Y1 pin 1, U13 pin B2; C22, Y2
- **Nets:** Net-(U15-LNA_IN), Net-(U15-XTAL_32K_N), +3V3
- **Reviewer:** `mini-oc-s3-6`, confidence 0.6
- **Verification:** not run (single source).

**Claim.** The live-mini ERC reports 'Symbol pin or wire end off connection grid' on U15 pin 1, L1 pin 1, Y1 pin 1 and U13 pin B2; the netlist shows all four connected as intended, so it is cosmetic, but these are exactly the pins where a future drag could silently break a wire. The DRC reports U15's board footprint 'IC_ESP32-S3' does not match the library copy (expected if the EPAD paste windowing lives on the board copy) and a C22/Y2 silkscreen overlap.

**Evidence.** $S/out/live-mini/erc.json: endpoint_off_grid items for U15 Pin 1 [LNA_IN], L1 Pin 1, Y1 Pin 1, U13 Pin B2 [VCC]; drc.json: lib_footprint_mismatch warning 'Footprint IC_ESP32-S3 does not match copy in library' [U15], silk_overlap C22/Y2. Netlist: Net-(U15-LNA_IN) = C7.2, L1.1, U15.1; Net-(U15-XTAL_32K_N) = C21.1, U15.22, Y1.1; +3V3 includes U13.B2.

**Consequence.** None electrically today.

**Fix.** Snap the four pins to grid at the next schematic touch; confirm the U15 footprint mismatch is the intended window-paned EPAD (decisions.md) and push that copy to the library so the V10 U15 gets the same paste.

#### 48. [Note] Three Espressif checklist items absent on both S3s — all identical to the working V9, so carry-overs, not defects

- **Refs:** U15, U32, SW2, SW3, C27, C116, L4, L10, C33, C122
- **Nets:** Net-(U15-GPIO0), Net-(U32-GPIO0), OC_VDD_SPI, FC_VDD_SPI, Net-(U15-VDD3P3), Net-(U32-VDD3P3)
- **Reviewer:** `mini-pcb-usb-mcu-10`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** (a) GPIO0 has no external pull-up on either S3 (SW3/SW2 to GND only; internal WPU) where Espressif 'recommends a pull-up resistor at the GPIO0 pin'; (b) VDD_SPI has 1 µF only (C27/C116) where Espressif recommends 0.1 µF + 1 µF; (c) VDD3P3 pins 2/3 have 100 nF (C33/C122) behind the 2 nH L4/L10 where Espressif asks for 10 µF per pin (the 10 µF C32/C125 sit on the rail side of the inductor, 1.1–2.0 mm from pins 55/56). The V9 netlist has exactly the same C33/L4, C27 and GPIO0 arrangement and boots and enumerates.

**Evidence.** dump.py nets on live-mini and v9 netlists (Net-(U15-GPIO0): SW3.2 U15.5; OC_VDD_SPI: C27.1 U15.29; Net-(U15-VDD3P3): C33.1 L4.1 U15.2 U15.3 on both); Espressif ESP32-S3 schematic checklist quotes fetched 2026-09-21.

**Consequence.** Not a bring-up risk (V9 proves the arrangement); listed so the synthesis does not re-derive them as layout faults. Also confirms L3/L9 24 nH in series with XTAL_P is Espressif's own recommendation ('add a series component on the XTAL_P clock trace… 24 nH').

**Fix.** None required; optional 0402 positions at a future spin.

#### 49. [Note] 2.4 GHz feed: 0.18 mm track with a 0.15 mm pour gap is ~42 Ω by the notes' own model, and the feed is 8.1 mm, not 2.99 mm

- **Refs:** U15, L1, C7, C12, U14
- **Nets:** Net-(U15-LNA_IN), Net-(C12-Pad1)
- **Reviewer:** `mini-pcb-usb-mcu-8`, confidence 0.6
- **Verification:** not run (single source).

**Claim.** LNA_IN: U15 pad 1 (91.935,139.385) → 1.37 mm south → 2.67 mm diagonal → L1/C7 at (93.0–93.9,142.64): 5.09 mm of 0.18 mm F.Cu; C7 → U14: 2.99 mm of 0.18 mm F.Cu vertical at x = 93.83. F.Cu GND pour edge is 0.24 mm from the track centreline (0.15 mm gap) on both sides of the vertical feed; In1 GND is continuous under the whole feed until it enters the keepout at y ≈ 145.0. FABRICATION-NOTES A4 says 0.18/0.127 CPWG on this stack is ~42 Ω and 50 Ω needs 0.15 mm / 0.19 mm gaps; a plain microstrip estimate (Hammerstad, h 0.1164, εr 4.16) for 0.18 mm is 56–58 Ω, so the adjacent pour is what pulls it low. At 2.44 GHz (εeff ≈ 3.1) 8.1 mm is ≈40° electrical, not the '<15°' the note derives from the 2.99 mm segment alone.

**Evidence.** nets.py Net-(U15-LNA_IN) total 5.091 mm w=0.18; Net-(C12-Pad1) 2.991 mm w=0.18; ref.py pour gaps 0.24/0.23 mm from centreline, In1 present at (93.83,144.5) absent at (93.83,145.0); FABRICATION-NOTES.md A4; Espressif S3 layout guideline: 'RF trace should have a 50 Ω characteristic impedance… routed on the outer layer without vias… 135° angle' (satisfied: F.Cu only, one 45° bend).

**Consequence.** A ~42 Ω, 40° line between the LNA and the match shifts the impedance the pi-match sees; with C12 DNP the tuning margin is one part. Small loss (<0.3 dB) but it compounds with finding 4.

**Fix.** When the fab confirms the 8-layer template, set the feed to the note's CPWG geometry (0.15 mm / 0.19 mm) or pull the pour back to ≥0.30 mm and use ~0.23 mm microstrip; count the LNA_IN segment in the length budget.


### Recovery-deployment channels and the arming interlock

#### 50. [Major] The OC consent contract the hardware safety case rests on (raise only on explicit arm, drop on FC heartbeat loss) is not implemented; today nothing raises or drops it

- **Refs:** U15 GPIO11 (mini) / U15 GPIO14 (V10), Q14, board_m1.h ARM_CONSENT_PIN, out_computer main.cpp
- **Nets:** OC_ARM_EN
- **Reviewer:** `mini-pyro-arm-2`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** The pyro sheet note and README ('the OC term is the fail-safe: OC firmware drops OC_ARM_EN when FC heartbeats stop') describe the mechanism that makes finding 1 tolerable. In the tree at d3c1aa30 the only code touching ARM_CONSENT_PIN configures it as an output and drives it low at boot (#1168); there is no raise path and no heartbeat-loss drop path. The board is therefore safe by omission (no channel can be armed at all), which is the state #1316 already expects, but the documented fail-safe does not exist yet and #1168 is closed.

**Evidence.** grep of tinkerrocket-idf/projects/out_computer/main/main.cpp: ARM_CONSENT_PIN appears only at lines 9438–9452 (set low, gpio_config output, set low, log). No other reference in out_computer or components. README 'Arming needs both processors' and pyro.kicad_sch text 'Supervised arm' state the drop-on-heartbeat-loss contract. FC side: pyroSetArmLocked() (main.cpp 1325–1332) raises FC_ARM for the fire window; PYRO_FIRE_DURATION_MS = 200, PYRO_ARM_SETTLE_MS = 10 (config.h 413/416).

**Consequence.** When consent-raising is added for flight, if the heartbeat-loss drop is not added with it, an FC reset mid-flight (brownout, panic, WDT) leaves U9 closed for the entire OC consent window through GPIO44's reset pull-up (finding 1), and the only thing between the pack and an initiator is the FIRE pin pull-down. It also means the mini/V10 cannot be fire-tested (cmd PYRO_FIRE_TEST) on today's firmware — a bench surprise, not a defect.

**Fix.** Track the two halves as one firmware item: (a) OC raises OC_ARM_EN only on an explicit arm command and only while FC heartbeats are fresh; (b) OC drops it on heartbeat loss, on observing an FC reboot, on disarm, and never enters light sleep while consenting; (c) both line states reported in the pyro status the FC already emits. Add the 'FC held in reset while OC consents → U9 must open within the heartbeat timeout' case to the #1316 bench list.

#### 51. [Minor] FC arm term is ON, not idle, during every FC boot, reset and USB flash — the OC consent transistor is the only barrier

- **Refs:** U32 pin 50 (GPIO44/U0RXD), R132 100, Q12 DTC123J, Q14, Q13, U9, R22
- **Nets:** FC_ARM, Net-(Q12-B), Net-(Q12-C), OC_ARM_EN, /Pyro/ARM_GATE
- **Reviewer:** `mini-pyro-arm-1`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** With the flight computer in reset, in ROM download mode or unconfigured, GPIO44's weak pull-up (Table 2-1 pin 50: WPU, IE at reset AND after reset; RPU 45 kΩ, Table 5-4) puts ~1.7 V on Q12's input (3.3 V × 49.2 k / (45 k + 49.2 k); 1.4–2.1 V over the R1/R2 spread), above the DTC123J VI(on) 1.1 V with ~0.45 mA base drive, so Q12 saturates. The FC term of the two-transistor AND is therefore asserted in exactly the windows the rework was written for; only Q14 (OC_ARM_EN driven low by OC firmware, or floating → 47 k B-E holds it off) keeps Q13/U9 open. The #994 defence-in-depth (pull-down on the Q12 input, or moving FC_ARM to a pad without a reset pull-up) was recorded as 'available, not applied'. On the V10 this does not exist: FC_ARM is P4 GPIO33 (pad 64), which the P4 datasheet Table 2-1 lists as IE only, no pull.

**Evidence.** live-mini netlist.xml: FC_ARM = {R132.2, U32.50[U0RXD/GPIO44]} (no pull-down); Net-(Q12-B) = {Q12.1, R132.1}; Q12.2 = GND; Net-(Q12-C) = {Q12.3, Q14.2}; Net-(Q13-B) = {Q13.1, Q14.3}; Q13.2 = VBAT_CON. esp32-s3_datasheet_en.txt line 840 (pin 50 U0RXD 'WPU, IE / WPU, IE'), line 3707 (RPU 45 kΩ). DTC123J datasheet (work/mini-pyro-arm/dtc123j.txt lines 117–125): VI(off) 0.5 V max, VI(on) 1.1 V, R1 1.54–2.86 k, R2/R1 17–26. P4 datasheet (work/mini-pyro-arm/esp32-p4.txt line 838): pin 64 GPIO33 'IE / –'. Firmware: out_computer main.cpp 9438–9452 drives OC_ARM_EN low at boot; flight_computer main.cpp safePyroOutputInit() (1250–1283) reconfigures GPIO44 only once the app runs.

**Consequence.** During every FC power-up, panic/WDT reset and USB download session the arm return switch is one OC fault (GPIO11 stuck high, OC firmware error) away from closing. No initiator can fire from this alone — PYRO1–4_FIRE sit on pads 43/40/39/38 (Table 2-1: IE only, no pull; not in Table 2-2's glitch list) with R78–R81 5.11 k pull-downs — but the 'both processors' property degrades to 'one processor' for the FC-reset windows, and the pyro sheet's own sentence 'an MCU that is off, in reset, or high-Z holds its transistor off through the built-in 47 k' is untrue for the FC term.

**Fix.** Simple fix if the owner wants the two-fault property back in those windows: a 3.3–4.7 k pull-down from Net-(Q12-B) to GND (4.7 k: 0.31 V with RPU 45 k, 0.45 V at a 30 k corner — under VI(off) 0.5 V; the driven-high pin still gives Q12 ~1.1 mA base current). Otherwise keep the circuit and correct the sheet note/README to say the FC term is NOT idle at reset and the OC term alone covers those windows (see finding 5). Either way the #1316 scope of FC_ARM/ARM_GATE through a boot, reset and download session stays on the bench list.

#### 52. [Minor] Nothing on the board bounds a fire pulse into a shorted harness: the pack sets the current and the shared Q11 / R72 / five In5→B.Cu vias carry it

- **Refs:** U6–U8, U10 WSD20L50DN33; U9 AON7534; Q11 AONR21321; R72 2 mΩ; VBAT_CON vias (mini) at (87.11,146.72) (87.15,146.05) (87.33,149.20) (87.59,146.44) (87.62,147.01); J2
- **Nets:** VBAT_CON, VBAT_Terminal, VBAT_J8, PYRO1–4_EXT, PYRO_GND
- **Reviewer:** `mini-pyro-arm-3`, confidence 0.75
- **Verification:** not run (single source).

**Claim.** With rework 4 the 74LVC1G17 one-shot ceiling is gone (decisions.md). What now limits a channel left on: (1) duration — only the FC's PYRO_FIRE_DURATION_MS = 200 ms and, once implemented, the OC's consent drop; if the FC wedges with a FIRE pin high while the OC consents, nothing else opens the path. (2) current — no series element, no eFuse (VBAT_CON is ahead of U19), no PTC; a shorted initiator/harness is limited only by the pack's internal resistance plus the FET/shunt/copper stack (~40 mΩ on-board), i.e. roughly 50–80 A for a typical 2S pack (estimate). That current passes through parts shared by the whole computer: Q11 (pulsed rating −66 A, RDS(on) < 29.5 mΩ at −4.5 V), R72 (2 mΩ 0805), and on the mini the five 0.3 mm-drill vias that join the In5 VBAT_CON plane to the B.Cu FET-source zone. The nominal case (1–2 Ω e-match, 5–8 A, two channels ≈ 11 A) is comfortable everywhere: U8 ≤ 11 mΩ → 0.3 W; U9 ≤ 8.5 mΩ at 8.2 V gate → 1 W; Q11 ~2.4 W for 200 ms (PDSM 4.1 W ≤ 10 s); R72 0.24 W; J2 common pole 11 A vs 12 A family rating.

**Evidence.** live-mini netlist.xml: VBAT_CON = {U6/U7/U8/U10 SOURCE, R15–R17/R23, R72.1, U19.4 IN, U23 BUS/IN-, Q13.2}; VBAT_Terminal = {Q11.1–3 S, R72.2, U23.13}; VBAT_J8 = {J3.2, Q11.5–9 D}; PYRO_GND = {J2.5_1/5_2, R73.1, U9 DRAIN}. pcbnew (rocket-computer-mini.kicad_pcb): two VBAT_CON B.Cu zones (47.7 mm² around the FETs, 8.94 mm² at R72/U19) joined only through the In5 zone (52.7 mm²) by 5 vias near (87.1–87.6, 146.0–149.2) and 7 near (90.9–94.3, 127.5–129.0); FET source/drain pads and J2 pads x_1 sit inside their zones; PYRO_GND returns through 8 vias at U9's drain pads (92.0–94.0, 159.4–160.0). AON7534 datasheet: IDM 120 A, ID 30 A, RDS 8.5 mΩ max @4.5 V. AONR21321 datasheet: IDM −66 A, IDSM −13 A (Ta), RDS < 29.5 mΩ @ −4.5 V, PDSM 4.1 W. Winsok selecting guide row WSD20L50DN33: ID −50 A, RDS 9/11 mΩ @4.5 V (IDM not listed). CUI TBLH11-350 (the footprint's original part): 10 A UL / 17.5 A IEC, 20 mΩ contact; JILN JL212 3.5 mm family sibling page: 12 A / 300 V, 14–28 AWG. config.h: PYRO_FIRE_DURATION_MS 200.

**Consequence.** A shorted deployment harness (crushed leads, a shorted e-match) turns a 200 ms deployment pulse into a pack short through the computer's own pack-entry parts. Q11 failing open takes the pack away from the whole avionics stack (the supercap then carries +3V3 for seconds); the mini's five plane vias run at 10–16 A each during such a pulse. Not a first-article or nominal-flight problem; it is the cost of removing the hardware ceiling and it is undocumented.

**Fix.** Discussion item, not a drawing: (a) document the new ceiling plainly on the pyro sheet ('fire duration = firmware 200 ms; current = pack-limited; shared Q11/R72 see the sum of channels'); (b) firmware pre-fire sanity: CONT cannot tell 1 Ω from 0 Ω, but the INA230 on R72 can see the pulse current and the OC could refuse/abort a second channel if the first pulse read as a short; (c) if a hardware bound is wanted, options are a low-value series element per channel or a pack-side PTC — both change the protection philosophy and belong to the owner. On the mini layout, more In5→B.Cu VBAT_CON stitching under the FET-source zone is a cheap margin item (coordinates above).

#### 53. [Note] Known but worth restating for the synthesis: the OC firmware never raises OC_ARM_EN, so the mini cannot fire any channel today; and because Q14's base floats at every OC reset, an in-flight OC reboot drops the arm until consent is re-asserted - there is no re-assert path yet

- **Refs:** U15 GPIO11 (OC_ARM_EN), Q14 DTC123J, Q12, Q13, U9; out_computer main.cpp:9430-9452, board_m1.h:96-114
- **Nets:** OC_ARM_EN, FC_ARM, ARM_GATE, PYRO_GND
- **Reviewer:** `mini-fc-s3-5`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** The #994 fix is implemented in the netlist exactly as the closing comment describes: Q12.C -> Q14.E, Q14.C -> Q13.B, Q14.B <- OC GPIO11 (no series R; the DTC123J's internal 2.2 k/47 k do the job). GPIO11 is 'IE' with no pull at/after reset (Table 2-1 pin 16), so Q14 is held off by its 47 k base-emitter resistor whenever the OC is in reset, and the FC's GPIO44 reset pull-up (~1.7 V on Q12's input) cannot close U9 alone. The only writers of ARM_CONSENT_PIN in the whole firmware tree are main.cpp:9440 and :9448, both gpio_set_level(...,0), with the comment 'There is deliberately no path that raises it yet'. Consequently (a) no pyro channel can fire on the mini until flight software raises consent (documented in #1168 and assumed by #1211's M1 bench section), and (b) once such a path exists it must also run on the #825/#1176 rail-restored boot, because an OC watchdog/panic reboot in flight floats GPIO11 and opens U9 for the whole OC boot.

**Evidence.** Netlist: Net-(Q12-C): Q12.3 Q14.2[E]; Net-(Q13-B): Q13.1 Q14.3[C]; OC_ARM_EN: Q14.1[B] U15.16[GPIO11]; FC_ARM: R132.2 U32.50 (no pull-down, as #994's closure says). gh issue view 994 closing comment. grep -rn ARM_CONSENT tinkerrocket-idf: only board_*.h definitions and main.cpp:9438-9452. Datasheet Table 2-1 pin 16 GPIO11: after reset IE only.

**Consequence.** (a) is the intended safe default. (b) is a flight-safety property of the consent architecture the owner chose: an OC reboot between apogee and main deploy disarms the pyros for the OC's boot time plus whatever the re-consent path takes.

**Fix.** No hardware change (architecture is an owner decision). When the consent path is written (#1409 firmware pick-ups), make the rail-restored boot re-assert consent from the retained flight token before the deferred initPeripherals(), and log the gap.

#### 54. [Note] USB-only bring-up: the continuity pull-ups back-feed VBAT_CON through the P-FET body diodes, so the INA230 reports a phantom ~3 V 'pack' with no pack fitted

- **Refs:** R8/R10/R12/R18 49.9 k, U6–U8/U10 body diodes, U23 INA230, U19 IN, Q11, J3
- **Nets:** V_MCU_SWTCH, PYRO1–4_EXT, VBAT_CON, VBAT_Terminal, VBAT_J8
- **Reviewer:** `mini-pyro-arm-4`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** With USB only (no pack) and the FC rail up, each PYRO_EXT is pulled to V_MCU_SWTCH through 49.9 k; the high-side P-FET's body diode (anode = drain = PYRO_EXT, cathode = source = VBAT_CON) is forward biased, so up to 4 × ~60 µA lifts VBAT_CON to roughly 2.5–3 V (loads there are only U19's UVLO divider, U23's bus input and leakage). Q11 then sees Vgs ≈ −3 V and partially conducts, so the same voltage appears on J3's pack pins. Harmless electrically (µA), but the INA230 bus reading and any 'pack present' logic will see ~3 V, and a bench meter on J3 will too. The V9 has the same structure (pull-ups and sources both on V_CAP).

**Evidence.** live-mini netlist.xml: R8.2 = V_MCU_SWTCH, R8.1 = PYRO1_EXT = U8 DRAIN; U8 SOURCE = VBAT_CON; VBAT_CON members include U23.11 BUS, U19.4 IN, R44.2; Q11 S = VBAT_Terminal (R72.2), D = VBAT_J8 (J3.2), G = GND. AONR21321 datasheet: P-channel, body diode drain→source. Same topology in out/v9/netlist.xml with V_CAP.

**Consequence.** A bring-up trap only: 'pack voltage' telemetry of ~3 V on a USB-only board is not a wiring fault, and a continuity read on a USB-only board still works (open → CONT high; initiator → CONT ≈ 0.14–0.49 V via R73). No effect on USB enumeration.

**Fix.** No change; add one line to FABRICATION-NOTES/bench notes so the first-article log is not misread. If a clean 0 V is ever wanted, the pull-ups would have to move to VBAT_CON (as the V9 had them on the source rail) — a rail decision, not a fix.

#### 55. [Note] Arm-chain documentation drift: the sheet note/README idle-at-reset sentence is false for the FC term, and the V10 parity table puts the gate pull-ups on the wrong rail

- **Refs:** pyro.kicad_sch 'Supervised arm' text; README 'Arming needs both processors'; rocket-computer/v10-power-parity-2026-09-11.md rail table
- **Nets:** FC_ARM, VBAT_CON, VBATT
- **Reviewer:** `mini-pyro-arm-5`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** (a) Both the sheet note and the README say 'An MCU that is off, in reset, or high-Z holds its transistor off through the built-in 47 k base-emitter resistor' and then, two sentences later, concede GPIO44 carries a pull-up at reset — the first sentence is only true for the OC term (finding 1). (b) v10-power-parity-2026-09-11.md's rail table lists 'pyro gate pull-ups' under VBATT (eFuse output); the live V10 netlist has R15/R16/R17/R23 on VBAT_CON, the same rail as the P-FET sources — which is the safe arrangement (a pull-up on a different rail than the source would turn every channel on if that rail dropped while VBAT_CON stayed up). (c) The sheet's 'Pyro Arm Hardware Protection' title text predates rework 4.

**Evidence.** pyro.kicad_sch text block (extracted): 'An MCU that is off, in reset, or high-Z holds its transistor off through the built-in 47k ... GPIO44 has a weak pull-up at reset, so the OC term is the fail-safe'. README lines 154–171. live-v10 netlist.xml: R15.2/R16.2/R17.2/R23.2 = VBAT_CON; v10-power-parity-2026-09-11.md line 158: 'VBATT (eFuse output) | ... pyro gate pull-ups'.

**Consequence.** A reader of the sheet takes the FC term as reset-safe; a reader of the parity table would think the V10 pull-ups sit on a rail the eFuse can drop.

**Fix.** Proposed replacement text for the sheet note (owner's call): 'Q14 (OC) is off whenever the OC is off, in reset or high-Z (47 k B-E). Q12 (FC) is ON while GPIO44 sits at its reset pull-up — every FC boot, reset and download — so the OC term alone disarms in those windows.' Correct the V10 rail table row to VBAT_CON.

#### 56. [Note] WSD20L50DN33 gate rating verified only from the manufacturer's selection guide: |Vgs| = pack voltage, ±12 V max, 3.6 V margin at 8.4 V and no margin above a 2S pack

- **Refs:** U6–U8, U10 WSD20L50DN33; R15–R17/R23 10 k; Q3–Q6 DTC123J
- **Nets:** VBAT_CON, Net-(Q3-C)/(Q4-C)/(Q5-C)/(Q6-C)
- **Reviewer:** `mini-pyro-arm-6`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** The fire drive pulls the P-FET gate to Q5's Vce(sat) (~0.1 V) against a 10 k pull-up to VBAT_CON, so |Vgs| equals the pack voltage: 8.4 V at full charge, 7.0 V at the BAD threshold. The Winsok selecting guide row for WSD20L50DN33 gives VGS ±12 V, VDS −20 V, ID −50 A, Vth −0.5…−1 V, RDS(on) 9/11 mΩ at 4.5 V and 11/15 mΩ at 2.5 V — so the drive is inside the rating and RDS(on) at 7–8.4 V is ≤ 11 mΩ (0.3 W at 5.5 A). This closes the #1316 'gate-source rating' checkbox for Vgs, but on a distributor selection-guide row, not the part datasheet (LCSC's PDF endpoint returned an HTML shell). The pulsed-current and avalanche figures were not obtainable. The margin is 3.6 V; the drive is the raw pack, so anything above a 2S pack on J3 (a 3S at 12.6 V) exceeds the gate rating — worth one line on the pack-input sheet if it is not already there.

**Evidence.** live-mini netlist.xml: Net-(Q5-C) = {Q5.3, R15.1, U8.4 GATE}, R15.2 = VBAT_CON, U8.1–3 SOURCE = VBAT_CON; Q5.1 = PYRO1_FIRE (R78 5.11 k to GND). work/mini-pyro-arm/winsok-guide.txt line 206: 'WSD20L50DN33 Single P-Ch -20 12 -50 -0.5 - -1 ... 9 11 11 15 ... 25 1.6 11 1620 320 290 DFN3X3-8L'. DTC123J: VO(on) 0.1 typ / 0.3 max at 5 mA.

**Consequence.** None at 2S. Rating margin is thinner than the V9's TPN4R712MD family and the arm FET's ±20 V.

**Fix.** No change. Record the source in the #1316 checkbox; if the vendor datasheet is obtained, confirm VGS ±12 V, IDM and EAS from it. The gate pull-up could be split (e.g. a 10 k / 33 k divider to clamp |Vgs| ≈ 6–7 V) if a wider margin were ever wanted, but that is a discussion item, not a defect.


### Sensors and radios

#### 57. [Minor] Magnetometer U3 sits over the PYRO1 high-side FET and pyro copper with copper on all eight layers; the '2 x 2 mm all-layer no-pour rule area' does not exist

- **Refs:** U3, U8, C9, R78, VBAT_CON (B.Cu pour), PYRO1_EXT (B.Cu pour), MAG_SCL/MAG_SDA, H2
- **Nets:** VBAT_CON, PYRO1_EXT, MAG_SCL, MAG_SDA, Net-(U3-C1), V_MCU_SWTCH, +3V3
- **Reviewer:** `mini-pcb-sensors-rf-1`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** There is no keepout of any kind under the magnetometer: U3 (F side, (75.170,148.053), rot 90) has copper on every layer beneath its 3 x 3 mm body, and the B side directly opposite it carries the PYRO1 channel: U8 WSD20L50DN33 at 1.70 mm centre distance, C9 22 uF at 1.29 mm, R78 at 2.40 mm, the VBAT_CON B.Cu pour (74.28,144.94)-(88.64,151.22) and the PYRO1_EXT pour whose top edge (y 149.97) is at U3's body edge (y 149.98). The QMC5883P datasheet section 4.3 asks for 'no conducting copper line under/near the sensor in any of the PCB layers' and ferrous parts kept away on both sides.

**Evidence.** pcbnew scan of the U3 body (rotated 3 x 3 box): F.Cu GND fill 301/441 sample points, tracks V_MCU_SWTCH (77.35,148.30)-(76.43,148.30), Net-(U3-C1), MAG_SCL/MAG_SDA, a GND via (73.88,149.37) and a MAG_SCL via (75.13,148.58) under the body; box grown 0.5 mm: fills on In1/In2/In5/In6 (GND), In3 (+3V3), In4 (V_MCU_SWTCH), B.Cu VBAT_CON 467/2499 pts, B.Cu PYRO1_EXT 110/2499 pts, In5 track Net-(Q5-C), In2 track Net-(Q4-C). Board zones: only five rule areas exist (U14 8-layer (91.38,144.63)-(94.27,149.22); two B.Cu-only at (84.32,120.81)-(85.24,121.66) and (80.80,127.06)-(86.98,128.05); one 7-layer under U14's pads) plus footprint-embedded ones for U5 and D1-D4 - none near U3, in live, HEAD or the mini-v1.0.1 tag (grep 'keepout' in all three .kicad_pcb files). The QMC5883P_LGA-16_3x3.kicad_mod description itself says 'keep copper pours/tracks off all layers under the body'. Field estimates (B = mu0 I / 2 pi r): a pyro pulse of 1.5-4 A through U8 and the PYRO1_EXT pour at ~2 mm (through the 1.63 mm board) gives 150-420 uT, 3-8x Earth's ~50 uT, but only while firing and 5 orders below the 50000 gauss absolute maximum (datasheet Table 3); the AMR bridge is re-magnetised every cycle by the set/reset mode the driver enables (register 0x0B). The continuous pack path (J3 -> Q11 -> R72 -> U19 -> U21 -> L5 -> U18 -> L6 -> U47) is 10.7-25 mm away: 0.4 A at 10.7 mm (V_BUCK pour edge) = 7.5 uT, at L6 (12.2 mm) = 6.6 uT; the supercap charge (100 mA) via L11 at 15.8 mm = 1.3 uT. Ferromagnetic candidates: M2 screw/nut at H2 4.95 mm, J2's terminal screws ~15 mm, U5/U16 cans 30.6/36.5 mm, J8 43 mm.

**Consequence.** In normal operation the pyro copper under the sensor carries only the continuity-sense current, so the static field is a hard-iron offset that calibration absorbs; the risk is a large, changing offset whenever current flows there (fire pulses: irrelevant to heading; continuity checks: microamps) and an unverifiable soft-iron distortion from the 4.95 mm M2 hardware. The real defect is that the V10 parity document tells the V10 layout to copy a keepout that was never drawn.

**Fix.** For the next mini spin: move U3 off the pyro FET block (e.g. swap with a passive-only area) or at least add the documented all-layer rule area (tracks, vias, fills) sized to the body and route MAG_SCL/SDA around it; keep the M2 hardware non-magnetic (A2/A4 stainless or nylon) at H2. Correct mini-part-parity-2026-09-12.md (see finding 5). No change is needed for the first article.

#### 58. [Minor] BLE chip antenna U14: the all-layer cutout is 2.89 x 4.59 mm, not the 4.6 x 3.5 mm the datasheet asks for, and a 0.5 mm GND strip runs along the board edge inside the clearance on six copper layers

- **Refs:** U14, L1, C7, C12, U15
- **Nets:** Net-(C12-Pad1), Net-(U15-LNA_IN), GND
- **Reviewer:** `mini-pcb-sensors-rf-2`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** The Abracon AANI-CH-0070 recommended layout (datasheet Rev A p.4) is a 4.60 mm (along the edge) x 3.50 mm (inward) rectangular copper cutout through all layers, open to the board edge, with the antenna at the edge in one corner. The mini's 8-layer rule area is (91.381,144.633)-(94.272,149.222) = 2.89 mm inward x 4.59 mm along the edge, and between its outer edge (x 94.27) and the board edge (x 95.015) the GND pour continues on F.Cu, B.Cu, In1, In2, In5 and In6 (x 94.27-94.77, y 144.63-149.18, ~2.4 mm2 per layer). The antenna body (F.Fab x 93.61-94.64) therefore straddles the cutout boundary with its outer 0.37 mm over the GND strip, and the datasheet's 'no copper on any layer in this area' is not met for the outer 0.75 mm of the clearance. FABRICATION-NOTES.md still lists 'Its 4.60 x 3.50 mm ground clearance must be reproduced as an all-layer keep-out' as an open item.

**Evidence.** Rule-area vertices from pcbnew: (94.272,144.637),(94.030,144.640),(94.045,145.841),(93.648,145.859),(93.670,144.636),(91.388,144.633),(91.381,149.219),(94.268,149.222). Zone-fill sampling on a 0.05 mm grid inside (94.27,144.63)-(95.02,149.22): GND fill present on F.Cu, B.Cu, In1, In2, In5, In6 (967 points each); In3/In4 clear (blocked by the rule area). U14 pads: 1/4 feed at x 93.850 (y 145.67/145.27), 2/3 GND at x 94.400; U14 F.Fab outline x 93.61-94.64, y 145.21-145.74; board edge x 95.015. Datasheet p.4 detail figure: cutout 4.60 x 3.50 with the antenna pads 0.30/0.50 mm from the cutout's corner and the cutout reaching the PCB edge; 'The rectangular copper cutout in the footprint must extend through all layers of the PCB stack-up, ensuring there is no copper on any layer in this area' (p.6). Twenty GND vias ring the rule area within 0.8 mm (good). Feed: U15.1 (91.935,139.385) -> 5.09 mm of 0.18 mm F.Cu track to the L1 (shunt 2.2 nH, at 93.025,142.64) / C7 (series 5.1 pF) node, then 2.99 mm to U14.1; C12 (shunt, DNP) at (93.04,143.57). 0.18 mm on the 0.1164 mm / er 4.16 prepreg is 53.6 ohm as a microstrip (Hammerstad, t 35 um), ~56 ohm with the coplanar F.Cu pour at a 0.13 mm gap.

**Consequence.** A ground strip under and beside the antenna's outer half detunes a 1.0 x 0.5 mm loop antenna (it lowers the resonance and raises the near-field loss); the pi match (L1/C7/C12) was not tuned on this board, and the datasheet says the match must be measured in the final device. BLE range/link margin may be below what the same match gives on the reference board. It does not affect boot or USB.

**Fix.** Extend the rule area to the board edge (x 95.015) so the cutout is 3.5-3.6 mm deep on all layers, leaving only the two GND pad connections on F.Cu as short stubs to the pour at the cutout's top corner (as the datasheet figure draws them); then tune L1/C7/C12 in situ with a VNA. Close or update the FABRICATION-NOTES open item once done.

#### 59. [Minor] IMU U2 and barometer U4 sit back-to-back with the SMT screw-terminal block J2's pads, 4.5 mm from mounting hole H2 and 3.8 mm from the board edge, with a via and supply traces under the IMU body

- **Refs:** U2, U4, J2, H2, C3, C150, R5
- **Nets:** PYRO1_EXT, PYRO2_EXT, V_MCU_SWTCH, ISM6HG256_CS, SENS_SCLK/SDI/SDO
- **Reviewer:** `mini-pcb-sensors-rf-3`, confidence 0.6
- **Verification:** not run (single source).

**Claim.** The ISM6HG256 (U2, F side, (76.268,157.325), rot 45) is directly opposite J2's pad 1_1 (B side, (75.55,159.57), 1.6 x 4.0 mm; 2.2 mm centre-to-centre, the pad's copper spans y 157.57-161.57 so it overlaps U2's footprint through the board), 4.54 mm from H2's M2 hole and 3.80 mm from the left board edge. The BMP581 (U4, (77.700,161.430)) is 2.3 mm from J2 pads 1_1 and 2_1 and 0.86 mm from J6's F.Fab body edge (y 163.67). Under the IMU body on F.Cu there are nine V_MCU_SWTCH 0.10 mm track segments, a GND via (77.34,157.65) and a V_MCU_SWTCH via (76.94,158.67); a GND via (77.60,161.41) sits 0.1 mm from U4's centre. Torquing the pyro terminal screws loads J2's SMT joints and the board directly beneath both MEMS.

**Evidence.** pcbnew: U2/U4/J2/H2 positions above; U2 F.Cu scan (rotated 2.5 x 3.0 body) lists the tracks/vias; J2 pads J2.1_1 (75.55,159.57) ... J2.5_2 (89.55,168.87), all 1.60 x 4.00 SMT, J2 B.Fab body (73.39,155.21)-(91.71,169.53). B.Cu fills under U2's bbox: PYRO1_EXT 1214/2401 pts, PYRO2_EXT 213/2401. TDK AN-000393 (IMU PCB design guidelines, the only vendor guidance reachable; ST TN1383/TN0018 timed out): 'Trace, via, and filled copper are not allowed under the IMU chip directly'; 'Keeping a distance larger than 3 mm to any PCB anchor is recommended'; 'Do not place connectors ... on the PCB surface below the IMU location'; 'Keep the IMU away from the edge of the PCB'. The V9 (working) places its U2 with the same footprint/rotation (rot 45, pad 1 offset (-1.349,+0.288) on both boards) but 3.6 mm off its centreline and opposite a pin header J3 rather than a screw block. All vias on this board are filled and capped (fab spec), so the via under U2 is flat.

**Consequence.** Board strain from terminal-screw torque, the M2 screw at H2 and edge deflection shifts the accelerometer offset (tens of mg class per the vendor notes) and, to a lesser degree, the gyro bias; launch-detect and the boot-time bias seeding absorb a static shift, but a shift that changes when the pyro wires are re-terminated between boot and flight is not seeded out. Not a bring-up blocker.

**Fix.** Next spin: move U2 (and U4) off the J2 footprint area toward the board centreline, at least 3 mm from H2 and the outline, with no tracks or vias under the body (route V_MCU_SWTCH/CS around it); if J2 must stay, keep the block's pads away from the MEMS by at least the block's own width. For the first article: torque the J2 screws before any bench calibration and re-check accel offsets after re-wiring.

#### 60. [Minor] LC86G VCC is fed straight from the switching rail; the datasheet asks for an LDO and adds a 33 pF + TVS the mini omits

- **Refs:** U5, U18, U47, U30, C37, C38, C39
- **Nets:** V_MCU_SWTCH, V_BUCK
- **Reviewer:** `mini-sensors-rf-1`, confidence 0.6
- **Verification:** not run (single source).

**Claim.** U5's VCC (pad 4) and V_BCKP (pad 5) sit on V_MCU_SWTCH, which is U18 TPS62152 buck -> U47 TPS61094 (bypass) -> U30 TPS22810 with no post-regulator. Quectel LC86G HW Design V1.5 §3.2.1 says 'It is not recommended to use a switching DC-DC converter', wants 'an LDO with a high PSRR', recommended ripple < 50 mV (§3.4), and Figure 4 shows TVS + 10 µF + 100 nF + 33 pF at VCC 'the minimum value capacitor closest to the VCC pin'. The mini has 22 µF (C37) + 100 nF (C38) + 10 nF (C39) under the VCC pads and no 33 pF, no TVS. The V9 does not carry this module: its GNSS carriers (gnss-sam10m8-18mm-hv, gnss-px1105r) were redrawn with an LDO (carrier v3.0.0), so there is no fabbed reference for a buck-fed LC86G.

**Evidence.** out/live-mini/netlist.xml: U5.4 VCC -> V_MCU_SWTCH, U5.5 V_BCKP -> V_MCU_SWTCH; V_MCU_SWTCH members include U30.1 VOUT, U47.9/10 VOUT; in_sensors.kicad_sch caps at U5 = C23 22 µF / C24 100 nF (sheet) but the board places C37/C38/C39 at U5's pads (pcbnew: U5 pad 4 at 75.59,120.51 F.Cu; C37 76.14,119.80 / C38 74.66,120.22 / C39 73.69,120.21 B.Cu, V_MCU_SWTCH vias at 75.07,120.53 and 75.99,120.54). Quectel V1.5 lines 1166-1200 (§3.2.1), 1397-1410 (§3.4).

**Consequence.** Buck switching spurs and load-step ripple on the GNSS supply can cost C/N0 and satellite count; the earlier V9 nose deficit was blamed on installation, so a buck-fed receiver has never been measured on this project. Not a bring-up blocker.

**Fix.** Measure C/N0 with the receiver on the bench rail (fold into the #1032 LoRa/GNSS interference test: compare against a bench LDO feeding pad 4/5 through the R72 path). If degraded, the remedy is a design discussion (small LDO or ferrite + 10 µF/100 nF/33 pF island on VCC), not a drawn change. Adding the 33 pF at pad 4 is a simple fix that can be drawn.

#### 61. [Minor] QMC5883P sits over F.Cu GND fill and both supply planes; radio TX current in those planes can modulate the field by an estimated ~50 mG

- **Refs:** U3, U16, U32, U15
- **Nets:** +3V3 (In3), V_MCU_SWTCH (In4), GND
- **Reviewer:** `mini-sensors-rf-5`, confidence 0.4
- **Verification:** not run (single source).

**Claim.** QMC5883P datasheet Rev E §4.3: 'no conducting copper line under/near the sensor in any of the PCB layers' and keep ferrous parts away. Under U3's centre the mini has F.Cu GND, In1/In2 GND, In3 +3V3 plane, In4 V_MCU_SWTCH plane, In5/In6 GND, B.Cu GND (eight layers of copper), plus MAG_SCL on F.Cu/In5 0.5 mm from centre and a pyro trace on In2 0.5 mm away. The In4 plane carries every V_MCU_SWTCH load; the E220 draws ~100 mA per TX burst (Ebyte §2.2) and the GNSS ~31 mA. Order of magnitude: 100 mA spread over ~10 mm of plane 0.6-1.0 mm below the die gives B = µ0·K/2 ≈ 6 µT ≈ 60 mG, i.e. ~10 % of the geomagnetic field, switching with each telemetry packet; hard-iron calibration cannot remove a load-correlated term. The V9 IIS2MDC had the same plane geometry with no datasheet keep-out rule, so this is new with the part change (PR #1452 carries it to the V10).

**Evidence.** pcbnew live-mini zones containing (75.17,148.05): F.Cu GND, B.Cu GND, In1/In2/In5/In6 GND, In3 +3V3, In4 V_MCU_SWTCH; tracks within 1.6 mm: MAG_SCL F.Cu/In5 d=0.53, Net-(Q4-C) In2 d=0.53; qmc5883p.txt lines 351-356 (§4.3); Ebyte manual §2.2 TX 100 mA; stackup: F.Cu->In3 ≈ 0.62 mm, ->In4 ≈ 0.93 mm.

**Consequence.** Heading/roll from the magnetometer may show a TX-synchronous step; the mag is poll-only so it cannot be blanked by DRDY, but it can be sampled away from the OC's TX window if the effect is measured.

**Fix.** Bench: log U3 with the radio muted vs transmitting at 22 dBm (add to #1032). If visible, V10 placement (in progress now, #1365) should keep U3 off the FC/radio supply path and over a plane void; the mini would need a re-spin.

#### 62. [Note] USB VBUS runs 2.3 mm from the magnetometer on In5: a bench-only field of tens of uT that a magnetometer calibration done on USB power would bake in

- **Refs:** U3, J6, U21
- **Nets:** Net-(J6-VBUS)
- **Reviewer:** `mini-pcb-sensors-rf-4`, confidence 0.7
- **Verification:** not run (single source).

**Claim.** Net-(J6-VBUS) is a 0.40 mm In5.Cu track from (72.89,163.11) to (72.89,128.23) along the left edge, passing 2.28 mm laterally (about 2.8 mm straight-line, In5 is ~1.1 mm below F.Cu) from U3's centre. With the board on USB power it carries the whole system current (~0.3-0.5 A); at 0.4 A and 2.8 mm that is B = 2e-7 x 0.4 / 0.0028 = 29 uT before return-path cancellation (up to 43 uT at 0.5 A / 2.3 mm), i.e. comparable to Earth's 50 uT. On the pack (flight) that track carries nothing.

**Evidence.** pcbnew scan around U3 (box grown 0.5 mm) lists 'trk net=Net-(J6-VBUS) In5.Cu w=0.40 (72.89,163.11)->(72.89,128.23)'; U3 centre (75.170,148.053). Stackup: F.Cu to In5 = 0.035+0.1164+0.0152+0.3+0.0152+0.1528+0.0152+0.3+0.0152+0.1528 = ~1.12 mm.

**Consequence.** A hard-iron calibration captured on USB differs from the flight state by up to a few tens of uT along one axis; on battery the nearest continuous current path is 10.7 mm away (<8 uT).

**Fix.** Firmware/procedure: run the magnetometer calibration on the pack, not on USB (a one-line note in the mag-cal instructions). Layout, next spin: keep VBUS/pack current tracks >5 mm from U3.

#### 63. [Note] Magnetometer axis mapping is unverified in firmware and the board carries no axis silk; the constant reused is the V9's bench-derived value for a different part

- **Refs:** U3
- **Nets:** MAG_SCL, MAG_SDA
- **Reviewer:** `mini-pcb-sensors-rf-6`, confidence 0.75
- **Verification:** not run (single source).

**Claim.** config.h applies IIS2MDC_ROT_Z_DEG = 90 (derived on the bench in #204 for the V9's ST IIS2MDC at rot 0) to the mag slot; on the mini the slot holds a QMC5883P placed at rot 90 with pad 1 at (+1.263,+0.750) from the centre. TR_QMC5883P.h says the chip-to-board rotation 'is a placement fact of the mini layout and is still marked VERIFY there; the axis silk on the board waits on the same bench.' The mini has no F.SilkS text at all (every gr_text is on B.SilkS) and the QMC footprint description itself says 'axis orientation vs board must be mapped in firmware'. The IMU, by contrast, is cleared: U2 uses the same footprint, side, rotation (45) and pad-1 offset as the V9's, matching ISM6HG256_ROT_Z_DEG = -45.

**Evidence.** tinkerrocket-idf/projects/flight_computer/main/config.h lines 252-262; components/TR_QMC5883P/TR_QMC5883P.h lines 49-51; pcbnew: U3 rot 90, pad1 (76.433,148.803); gr_text list of the board (15 texts, all B.SilkS).

**Consequence.** Heading from the magnetometer may be rotated by a multiple of 90 degrees or mirrored until the bench check is done; this is a known bench line, recorded here only because the placement facts (rot 90, pad-1 position) are what the check needs.

**Fix.** On the bench: point board +X (toward the J8/GNSS end) north and east, read the QMC raw axes, set the mini's own rotation constant (not the IIS2MDC value) and add an axis arrow to F.SilkS next to U3.

#### 64. [Note] V_BCKP is on the switched rail, so every FC rail cycle is a cold start (no warm/hot start retention)

- **Refs:** U5
- **Nets:** V_MCU_SWTCH, +3V3
- **Reviewer:** `mini-sensors-rf-2`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** U5.5 V_BCKP is tied to VCC on V_MCU_SWTCH. Quectel §3.2.2 / Table 6: 'V_BCKP ... should always be powered if hot (warm) start is needed', reference design Figure 5 powers it from a 3.3 V always-on supply through TVS + 4.7 µF + 100 nF + 33 pF. Powering it with VCC is permitted (§3.4: 'simultaneously with the VCC or before it'), so this is a capability trade, not a defect: after the OC drops the FC rail (pad standby, or the post-flight power-down) the receiver loses ephemeris/time and does a cold start (TTFF tens of seconds) on the next power-up. The always-on +3V3 exists 10 mm away (U23 VS+, R67/R69).

**Evidence.** out/live-mini/netlist.xml U5.5 -> V_MCU_SWTCH; Quectel V1.5 lines 1014-1035 (Table 6 V_BCKP remark), 1208-1245 (§3.2.2, Figure 5), 1397-1405 (§3.4).

**Consequence.** Longer TTFF after each rail cycle; on the pad this is the 'no fix yet' wait after the OC re-enables the FC. No safety impact.

**Fix.** Discussion item only (rail contract): V_BCKP from +3V3 through the Figure 5 network would cost a few tens of µA of always-on current and keep warm starts; the trade is the owner's.

#### 65. [Note] LC86G TXD/1PPS VOH-min (2.4 V) is below the S3's VIH-min (0.75 x 3.3 = 2.475 V) on paper

- **Refs:** U5, U32
- **Nets:** GNSS_TX, Net-(U5-1PPS)
- **Reviewer:** `mini-sensors-rf-3`, confidence 0.7
- **Verification:** not run (single source).

**Claim.** Quectel Table 6 guarantees TXD and 1PPS VOHmin = 2.4 V; the ESP32-S3 Table 5-4 requires VIH >= 0.75 x VDD = 2.475 V at 3.3 V. The module's CMOS output sits near VCC unloaded so it works in practice (the 2.4 V figure is a loaded TTL-style guarantee), but the worst-case datasheet numbers do not close by 75 mV on the only UART line the FC receives. The other direction closes: S3 VOHmin 2.64 V vs module RXD VIHmin 2.0 V; module VOLmax 0.4 V vs S3 VILmax 0.825 V.

**Evidence.** Quectel V1.5 lines 1036-1052 (TXD VOHmin 2.4 V, 1PPS VOHmin 2.4 V, RXD VIHmin 2 V); esp32-s3_datasheet_en.txt line 3695 (VIH 0.75 x VDD); netlist GNSS_TX: U5.2 TXD -> U32.44 MTCK (GPIO39), GNSS_RX: U32.45 MTDO (GPIO40) -> U5.1 RXD; both parts on V_MCU_SWTCH so no level shift is needed (Quectel §4.1.1.1 note 2).

**Consequence.** None expected on real parts; recorded so a marginal-high NMEA line is on the list if the FC ever sees framing errors at low rail (hold-up mode drops the rail to 3.0 V, where VIH-min becomes 2.25 V and the margin actually improves).

**Fix.** No change. If the bench shows UART errors, scope GNSS_TX high level at the U32 pad.

#### 66. [Note] E220 reference parts (ANT 330 nH shunt, NRST 100 k/100 nF) are absent and NRST floats while the OC is in reset; identical to the fabbed V9 daughterboard

- **Refs:** U16, J8, U15
- **Nets:** Net-(U16-ANT), L_RST, L_RXEN, L_CS
- **Reviewer:** `mini-sensors-rf-6`, confidence 0.6
- **Verification:** not run (single source).

**Claim.** Ebyte §5.1 Basic Circuit puts L1 330 nH from ANT to GND ('a protective device ... The user should add this inductor') and pulls NRST to VCC with 100 k + 100 nF. The mini has Net-(U16-ANT) = J8.1 + U16.6 only and L_RST = U16.3 + U15.24 (GPIO18) only. GPIO18 is 'IE' with no pull at/after reset (S3 Table 2-1) and has a 60 µs high-level then low-level glitch at power-up (Table 2-2), so NRST floats until firmware drives it. The V9 lora-daughterboard (fabbed, working) is wired the same way (Net-(U14-ANT): J8.1, U14.6; L_RST: U14.3, U28.43 GPIO38 with no pull), so this is a clearance by parity, recorded because the LLCC68 datasheet could not be fetched to confirm NRESET's internal pull.

**Evidence.** out/live-mini/netlist.xml nets Net-(U16-ANT), L_RST; work/mini-sensors-rf/lora-db-v9.xml nets Net-(U14-ANT), L_RST; E220 manual p.9 §5.1 figure; esp32-s3 Table 2-1 row 24 (GPIO18 IE), Table 2-2 GPIO18 glitches.

**Consequence.** At worst the radio is in an undefined reset state until the OC initialises it (which it does at boot), and J8 has no DC path to ground for static on the SMA centre pin.

**Fix.** None required for bring-up. If wanted next spin: 330 nH 0402 shunt at U16 pad 6 (also cheap ESD relief for the hand-soldered SMA) and a 100 k pull-up on L_RST to V_MCU_SWTCH (not +3V3, to keep the rail crossing clean).

#### 67. [Note] Schematic decoupling assignment for U5 and U16 is swapped relative to where the caps sit on the board

- **Refs:** C23, C24, C37, C38, C39, U5, U16
- **Nets:** V_MCU_SWTCH
- **Reviewer:** `mini-sensors-rf-7`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** in_sensors.kicad_sch draws C23 22 µF + C24 100 nF beside the GNSS module and C37 22 µF + C38 100 nF + C39 10 nF beside the LoRa module. On the board C37/C38/C39 are under U5's VCC/V_BCKP pads (B.Cu, 0.4-1.2 mm from the pad vias) and C23/C24 are at U16's VCC pad 1 (C24 100 nF 1.3 mm away, 0.4 mm B.Cu trace). Electrically both are on the same rail so each module is properly bypassed (LC86G gets 22 µ/100 n/10 n, E220 gets 22 µ/100 n); only the sheet-to-placement bookkeeping is crossed, which matters when someone edits 'the GNSS caps'.

**Evidence.** in_sensors.kicad_sch symbol positions: C23 (116.2,47.6), C24 (128.9,47.6) near U5 (234.3,48.3) text 'GNSS Module'; C37/C38/C39 (191-212, 41.9) near U16 (146.7,65.4). pcbnew: U5 pad 4 (75.59,120.51) F.Cu, C37 (76.14,119.80) B.Cu, C38 (74.66,120.22), C39 (73.69,120.21); U16 pad 1 (84.74,117.10) B.Cu, C24 (83.75,116.29), C23 (82.26,115.36); V_MCU_SWTCH track B.Cu (83.75,116.77)->(84.74,117.10).

**Consequence.** Doc/edit-hazard only.

**Fix.** Swap the refdes in the schematic (or annotate) so the sheet matches placement; no copper change.

#### 68. [Note] INA230 supply bypass C149 reaches VS+ through two vias and ~3.5 mm of trace

- **Refs:** U23, C149
- **Nets:** +3V3
- **Reviewer:** `mini-sensors-rf-8`, confidence 0.6
- **Verification:** not run (single source).

**Claim.** TI INA230 §8.4.1 asks for the supply bypass to be placed as close as possible to VS and GND. C149 (100 nF, B.Cu at 82.18,110.02) connects to U23 pin 9 (80.41,113.03) via a via at (81.70,110.02), an inner-layer run, a via at (80.42,111.91) and 1.1 mm of 0.2 mm B.Cu trace. The device draws ~330 µA and the pack bus sits at 8 V, so the consequence is small; recorded for completeness.

**Evidence.** pcbnew live-mini: +3V3 tracks B.Cu (80.41,112.49)->(80.41,113.03) etc., vias (81.70,110.02) and (80.42,111.91); ina230.txt line 1825 (§8.4).

**Consequence.** Marginal supply noise on the ADC/I2C; no functional risk identified.

**Fix.** Next spin, if the area is reworked anyway: move C149 to the pin-9/pin-10 corner on B.Cu.


### Memories, the inter-processor link and rail crossings

#### 69. [Major] OC power-off path (cmd 8) is ordered for the V9: it drops V_MCU_SWTCH first, then writes the flight log into a NAND that on the mini just lost its rail, and leaves M_FLASH_CS and L_RXEN driven HIGH into the dead domain until the OC resets

- **Refs:** U15 (OC) GPIO36/GPIO10, U11 GD5F1GQ5 pin 1 CS# / pin 8 VCC, U16 E220 pin 10 RXEN / pin 1 VCC, U30 TPS22810 QOD, R140, R142; out_computer/main/main.cpp lines 11555-11660, board_m1.h NAND_CS=36 LORA_RXEN_PIN=10
- **Nets:** V_MCU_SWTCH, M_FLASH_CS, L_RXEN, M_SCK, M_MOSI, L_CS, L_RST
- **Reviewer:** `mini-fc-s3-1`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** On the mini, U11 (NAND) and U16 (LoRa) are on V_MCU_SWTCH by decision, but the out_computer cmd-8 OFF sequence is unchanged from V9: digitalWrite(PWR_PIN, LOW) at main.cpp:11555, THEN quiesceStorageForRestart() (log close + index commit to the NAND, up to 10 s + 250 ms park) at :11566, THEN the GPIO teardown. The teardown's driven-LOW list is {LORA_ACT, LORA_UART_TX, LORA_SPI_SCK(=37), LORA_SPI_MOSI(=38), LORA_CS(13), LORA_RST(18)} and its high-Z list is the I2S/MISO/DIO1/BUSY inputs; NAND_CS (GPIO36 = M_FLASH_CS) and LORA_RXEN_PIN (GPIO10 = L_RXEN) are in neither. The comment block at :11605-11625 states the V9 assumption in words: 'U11 stays powered ... the quiesce above has parked the SPI mutex with CS HIGH'. On the mini that CS-HIGH park is a push-pull 3.3 V drive into an unpowered NAND's CS# pad, and RadioLib's RF-switch RXEN (set via setRfSwitchPins, TR_LoRa_Comms.cpp:105-109) stays at whatever level it last had.

**Evidence.** Netlist ($S/out/live-mini/netlist.xml): V_MCU_SWTCH members include U11.8[VCC], U16.1[VCC], U30.1[VOUT], U30.2[QOD]; M_FLASH_CS: R140.2 U11.1[CS#] U15.41[GPIO36] (no series R; R140 100 k pulls to V_MCU_SWTCH, i.e. to 0 V when off); L_RXEN: R142.1 U15.15[GPIO10] U16.10[RXEN] with R142 100 k to GND. main.cpp:11555 digitalWrite(PWR_PIN, LOW); :11566 quiesceStorageForRestart("power-off") with the comment 'AFTER the rail drop on purpose ... U11 (GD5F2GQ5UE SPI NAND) has VCC on +3V3 ... ALWAYS ON'; kSwitchedRailPins/kHighZPins at :11611-11648 lack NAND_CS and LORA_RXEN_PIN; TR_LogToFlash::parkSpiBusForReset (TR_LogToFlash.cpp:933) only takes the mutex, CS stays as csHigh() left it. grep of the whole out_computer tree shows no TR_BOARD_M1-conditional ordering (only config.h:42). TPS22810 datasheet (ti.com, pdftotext) RPD = 265-350 Ohm with QOD tied to VOUT, so the dead rail is a ~300 Ohm load that a single S3 pad (IOH 40 mA typ, Table 5-4) can hold at ~1-2 V through U11's CS input clamp for the whole quiesce window.

**Consequence.** Every operator power-off on the mini (a) issues the final log flush and index commit to a NAND whose VCC is collapsing/back-fed - the tail of the flight and possibly the dual-copy index are written into a brown-out part, and (b) forward-biases U11's CS# and U16's RXEN input clamps at up to the pad's drive current for up to ~10 s, partially powering the whole switched domain (FC, sensors, GNSS, radio) at an undefined 1-2 V. Latch-up/ESD-diode stress on U11/U16 and a corrupt or stale flight index are both plausible outcomes; neither is visible until a post-flight download.

**Fix.** In the M1 build of out_computer: run quiesceStorageForRestart() BEFORE digitalWrite(PWR_PIN, LOW) (the NAND must be alive to accept the close), then add config::NAND_CS and config::LORA_RXEN_PIN to kSwitchedRailPins (driven LOW, floating pull) for boards whose NAND/radio sit on V_MCU_SWTCH, then drop the rail, then reset. Keep the V9 order for V9/V10 (NAND on +3V3). Board-side there is no series resistor on any of the nine M_*/L_* lines, so nothing limits the injection; a hardware mitigation (series 1 k on M_FLASH_CS/L_RXEN, or a P-FET isolating U11's CS) is a design discussion, not a fix to draw. Check whether the open '#1228 mini follow-up' already covers this before filing.

#### 70. [Note] Two OC outputs that idle HIGH into switched-domain parts are missing from the power-off back-feed list (NAND_CS, LORA_RXEN)

- **Refs:** U11.1, U16.10, U15 GPIO36/GPIO10, U30 (RPD); main.cpp kSwitchedRailPins[]
- **Nets:** M_FLASH_CS, L_RXEN, V_MCU_SWTCH
- **Reviewer:** `mini-memory-link-2`, confidence 0.7
- **Verification:** not run (single source).

**Claim.** kSwitchedRailPins drives LORA_SPI_SCK, LORA_SPI_MOSI, LORA_CS_PIN and LORA_RST_PIN LOW before the reset but omits NAND_CS (GPIO36, parked HIGH by the SPI mutex) and LORA_RXEN (GPIO10, HIGH whenever the radio is in RX). Those are the only OC push-pull outputs that can sit high into an unpowered part. Today this is harmless only because the C105 window keeps V_MCU_SWTCH up for ≥0.45 s while the OC resets after 100 ms (post-reset pads are high-Z), i.e. the guard is carried by hardware timing the list does not know about.

**Evidence.** main.cpp: kSwitchedRailPins = {LORA_ACT_PIN, LORA_UART_TX_PIN, LORA_SPI_SCK, LORA_SPI_MOSI, LORA_CS_PIN, LORA_RST_PIN}; kHighZPins = {I2S ×4, LORA_UART_RX_PIN, LORA_SPI_MISO, LORA_DIO1_PIN, LORA_BUSY_PIN}; NAND_CS and LORA_RXEN appear in neither. Quantified injection if an output is left high into a dead part: S3 Table 5-4 IOH 40 mA at PAD_DRIVER 3 (≈20 mA at the default drive 2, ~33 Ω source), TPS22810 RPD 250 typ / 400 max Ω (QOD tied to VOUT: U30.2 = V_MCU_SWTCH). One pin: I ≈ (3.3 − 0.7 diode − V)/33 = V/250 → ≈9 mA, rail ≈ 2.3 V; two pins ≈ 2.7 V. R110 10 k ties FC_CHIP_PU to that same rail, so a phantom rail at 2.3–2.7 V would hold the FC in a brownout loop and half-power the LC86G (2.55 V min) and E220. No passive path exists: the only part touching both +3V3 and V_MCU_SWTCH is U30 (netlist scan), and every pull-up on an OC-facing switched net (R140, R141, R31, R33, R115, R116) returns to V_MCU_SWTCH.

**Consequence.** None on the shipped ordering. If the ordering or the reset ever changes (e.g. a power-off that does not esp_restart), NAND_CS/L_RXEN would back-feed the dead rail at ~9 mA each and hold it near 2.3 V.

**Fix.** Add config::NAND_CS and config::LORA_RXEN_PIN to kSwitchedRailPins (driven LOW after the quiesce). No hardware change.

#### 71. [Note] Firmware comments still describe six crossings and V9 rails; the mini has fifteen

- **Refs:** out_computer/main/board/board_m1.h (I2S block comment), out_computer/main/main.cpp power-off comment block
- **Nets:** ESP_I2S_*, ESP_SCL/SDA, L_CS, L_RST, L_RXEN, L_BUSY, L_DI01, M_SCK, M_MOSI, M_MISO, M_FLASH_CS
- **Reviewer:** `mini-memory-link-3`, confidence 0.95
- **Verification:** not run (single source).

**Claim.** board_m1.h says "These six link nets are the ONLY signals live during pad standby with the flight computer off"; the netlist has fifteen OC signals crossing into the V_MCU_SWTCH domain (six link + five radio + four memory), which the README corrected on 2026-09-12. The main.cpp power-off block asserts U11 is on always-on +3V3 and that LoRa is a J5 daughterboard, both false on the mini.

**Evidence.** live-mini netlist: U16.1 VCC = V_MCU_SWTCH, U11.8 VCC = V_MCU_SWTCH; nets L_CS/L_RST/L_RXEN/L_BUSY/L_DI01 and M_SCK/M_MOSI/M_MISO/M_FLASH_CS each join U15 to U16 and/or U11. README 'The link between them': "fifteen crossings in all (netlist, 2026-09-12; this paragraph said six until then)". board_m1.h lines ~150–153 and main.cpp lines ~11578–11600 (worktree HEAD).

**Consequence.** Someone extending the park lists from the header comment will protect six of fifteen lines.

**Fix.** Replacement text for the header: list all fifteen crossings by direction (OC inputs: BCLK, WS, SD, FSYNC, L_BUSY, L_DI01, M_MISO; OC open-drain: ESP_SCL/SDA; OC outputs: L_CS, L_RST, L_RXEN, M_SCK, M_MOSI, M_FLASH_CS) and state that the NAND and radio share the switched rail on this board. Owner's text; proposed, not changed.

#### 72. [Note] No series resistance on any of the fifteen crossings; the link flips drivers during an OTA relay — option for discussion, not a defect

- **Refs:** U15↔U32 (mini), U15↔U17 (V9/V10)
- **Nets:** ESP_I2S_BCLK, ESP_I2S_WS, ESP_I2S_SD, ESP_I2S_FSYNC
- **Reviewer:** `mini-memory-link-4`, confidence 0.5
- **Verification:** not run (single source).

**Claim.** Every crossing is pin-to-pin with no series resistor, identical to the V9's six-wire link. That is a clearance by identity, but the OTA image pump reverses BCLK/WS/SD (FC master→slave, OC slave→master) and the V9 already had one real contention event (i2s_del_channel() leaving pins driven), which the firmware now guards in TR_I2S_Stream::end(). With no series R, a future teardown bug pits two 40 mA drivers directly.

**Evidence.** live-mini netlist: ESP_I2S_BCLK = U15.7 + U32.27 only; ESP_I2S_WS = U15.6 + U32.24; ESP_I2S_FSYNC = U15.9 + U32.19; ESP_I2S_SD adds only R34 100 k to GND. v9 netlist: ESP_SCLK/ESP_CS/ESP_SDI/ESP_SDO likewise pin-to-pin. TR_I2S_Stream.cpp end() comment documents the reversal and the past contention; main.cpp calls beginSlaveRx (3620) and beginMasterTx (3661) on the OC, beginSlaveRx (2008) and beginMasterTx (2073/4194) on the FC. S3 Table 5-1 note 2 says the part survives all IOs pulled high while grounded for 24 h, so this is robustness, not damage.

**Consequence.** A driver-contention bug corrupts the OTA stream instead of merely degrading it; no hardware damage per the S3 abs-max note.

**Fix.** Discussion item only (owner rule): 33–100 Ω series on the four I2S lines at one end would bound contention current to ~30–100 mA total and cost nothing at the ~3 MHz I2S rate; leaving them out keeps parity with the fabbed V9. Not for the first article.

#### 73. [Note] OC link inputs float with the FC rail down and during FC boot (only ESP_I2S_SD has a pull)

- **Refs:** U15 GPIO1/2/4/14/17/35, R34
- **Nets:** ESP_I2S_BCLK, ESP_I2S_WS, ESP_I2S_FSYNC, L_BUSY, L_DI01, M_MISO
- **Reviewer:** `mini-memory-link-5`, confidence 0.6
- **Verification:** not run (single source).

**Claim.** With V_MCU_SWTCH off, the OC's six input-direction crossings float: their far-end drivers are dead and the S3 gives GPIO1/2 IE-only at reset and GPIO14/17/35 IE-only after reset with no pull (Table 2-1). R34 (100 k) defines only ESP_I2S_SD. The same lines float between rail-up and the FC's I2S start.

**Evidence.** S3 datasheet Table 2-1 rows 6, 7, 9, 19, 23, 40; live-mini netlist shows no passive part on ESP_I2S_BCLK/WS/FSYNC, L_BUSY, L_DI01, M_MISO. R34 = ESP_I2S_SD to GND. initPeripherals() is deferred until rail-on (main.cpp 9990/11500), so no OC peripheral drives or samples these in pad standby.

**Consequence.** Floating CMOS inputs in pad standby: small shoot-through current and noise sensitivity, nothing functional. During FC boot the I2S slave may clock garbage until frame sync resyncs, which the protocol already tolerates.

**Fix.** Firmware: gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY) on those six OC pins while the rail is off (a 45 k pull-down into a dead part injects nothing). No hardware change; do not add pull-ups.


### Netlist, bill of materials and documents

#### 74. [Minor] single_global_label is set to 'ignore' in the project ERC; eight global labels occur exactly once, so a typo in one would be a silent open

- **Refs:** rocket-computer-mini.kicad_pro erc.rule_severities.single_global_label = ignore; labels FC_CHIP_PU, FC_VDD_SPI, OC_VDD_SPI, ICHG_SET, OSEL_SET, VCHG_SET, SCAP_SW, V_SCAP
- **Nets:** FC_CHIP_PU, FC_VDD_SPI, OC_VDD_SPI, ICHG_SET, OSEL_SET, VCHG_SET, SCAP_SW, V_SCAP
- **Reviewer:** `mini-erc-bom-4`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** The ERC that reports 737 items with --severity-all is not seeing the single-global-label class at all (erc.json ignored_checks: single_global_label, four_way_junction, simulation_model_issue, footprint_filter). Eight global labels are used once; all eight nets are complete within their own sheet today, so nothing is broken, but the check that would catch a mistyped global label (the cheapest way to make a silent open on a power/strap net) is off.

**Evidence.** Scan of all six .kicad_sch files: 84 distinct global_label names, eight with count 1. Netlist membership: FC_CHIP_PU = C114.1 R110.2 U32.4; FC_VDD_SPI = C116.1 U32.29; OC_VDD_SPI = C27.1 U15.29; ICHG_SET = R136.1 U47.11; OSEL_SET = R134.1 U47.1; VCHG_SET = R135.1 U47.12; SCAP_SW = L11.1 U47.5; V_SCAP = C130.1 C142.1 L11.2 R125.1 U47.6 - each complete. Also hidden by 'ignore': four_way_junction (cosmetic).

**Consequence.** A future edit that misspells a global label produces no ERC item; the documented 'ERC floor = the 10 spare pads' silently excludes this class.

**Fix.** Set single_global_label to warning and either convert the eight one-sheet names to local labels or accept the eight warnings as part of the floor (documented in README).

#### 75. [Minor] mini-part-parity-2026-09-12.md tells the V10 layout that 'the mini puts a 2 x 2 mm all-layer no-pour rule area under the magnetometer die' - no such rule area exists in any mini snapshot

- **Refs:** U3 (mini), U3 (V10)
- **Reviewer:** `mini-pcb-sensors-rf-5`, confidence 0.95
- **Verification:** not run (single source).
- **Also reported as:** `mini-pcb-sensors-rf-1`

**Claim.** The V10 placement guidance for the QMC5883P cites a mini rule area 'on the board, not in the footprint' as the reference to copy. The live, HEAD and mini-v1.0.1 board files contain no rule area near U3 (all five board rule areas and the four footprint-embedded ones are listed in finding 1); the copper under U3 is continuous on all eight layers.

**Evidence.** $S/live/hardware/rocket-computer/mini-part-parity-2026-09-12.md lines 67-69; pcbnew zone listing of live/head/mini-v101 rocket-computer-mini.kicad_pcb (grep 'keepout' hits only the LC86G feed keepout and the D1-D4/U14 areas).

**Consequence.** The V10 placement (in progress now) will either look for a reference that is not there or assume the mini's U3 environment is clean when it is the worst on the board.

**Fix.** Rewrite the bullet: the mini has no magnetometer keepout; the V10 should draw one (all copper layers, tracks/vias/fills, at least the 3 x 3 body plus 0.5 mm) and keep switching inductors, pyro copper and ferrous hardware away, per QMC5883P datasheet section 4.3.

#### 76. [Minor] 'FC PWR' bottom silk glyphs sit 0.10 mm from the top board edge

- **Refs:** B.Silkscreen text 'FC\nPWR' at (80.11,104.89), labels D6
- **Reviewer:** `mini-pcb-visual-2`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** The top of the 'FC' glyphs is at y=102.047 mm against a board edge at y=101.945 mm: 0.10 mm clearance, inside typical fab routing tolerance, so the top of 'FC' may be nicked or clipped. DRC does not flag it because the board's silk-to-edge clearance is set to 0.

**Evidence.** pcbnew TransformTextToPolySet of the text: glyph bbox x 76.926..79.840, y 102.047..104.857; Edge.Cuts top line y=101.945..101.995; DesignSettings silk-to-edge = 0. Crop of B.assembly.png confirms the 'FC' cap line touching the edge margin. All other bottom texts keep >= 0.22 mm ('PWR' right glyph edge 94.790 vs edge 95.015; 'O' 72.811 vs 72.465; 'TR-Mini' 94.702).

**Consequence.** Cosmetic: a partially printed 'FC' label on the fabbed board.

**Fix.** Move the 'FC PWR' text down ~0.3 mm (there is free silk area to y~105.1 above D6's outline) or set a 0.2 mm silk-to-edge rule so DRC catches this class.

#### 77. [Minor] Pin-1 marks of U47 and Q11 are clipped by neighbouring pads' mask openings

- **Refs:** U47 (TPS61094, B, pin-1 triangle 82.69..83.09 x 137.13..137.53) vs R137 pad 2 (82.47,137.13); Q11 (AONR21321, B, pin-1 triangle 90.86..91.46 x 129.63..130.23) vs R72 pad 1 (91.25,129.11)
- **Reviewer:** `mini-pcb-visual-3`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** The only silkscreen orientation marks on two orientation-critical parts overlap adjacent pad mask openings, so the fab clips part of each triangle. Both marks survive partially (crops show the overlap is a corner), but they are the marks a first-article inspector would use to check rotation of the hold-up converter and the reverse-battery FET.

**Evidence.** drc.json silk_over_copper: 'Polygon of U47 on B.Silkscreen clipped by Pad 2 [VBUCK_OK] of R137' at (83.03,137.19) and 'Polygon of Q11 on B.Silkscreen clipped by Pad 1 [VBAT_CON] of R72' at (91.40,130.17). pcbnew: U47 pad 1 at (83.45,136.87) net OSEL_SET, triangle just beyond it; Q11 pad 1 at (91.765,130.683). Crops crop_U47_R137.png / crop_Q11_R72.png show the triangle corner inside the pad's mask window.

**Consequence.** Reduced visual pin-1 identification on two parts; no electrical effect.

**Fix.** Nudge R137 0.15 mm in +x or shrink/relocate the U47 triangle; move the Q11 triangle to the free side of pad 1 (away from R72). Owner's layout call.

#### 78. [Note] FABRICATION-NOTES and README carry six stale or imprecise statements about the board state

- **Refs:** FABRICATION-NOTES.md header and A7; README.md 'Where the numbers stand now'; on-board Dwgs.User note
- **Nets:** -
- **Reviewer:** `mini-erc-bom-3`, confidence 0.95
- **Verification:** not run (single source).

**Claim.** (1) Header: '22 DRC items ... five library-copy warnings' - live DRC is 23 with six lib_footprint_mismatch (U5, U15, U11, U32, U18, U19); the text explains the sixth (U5) but the headline numbers were not updated (README already says 23/six). (2) Header: 'One accepted schematic-parity item: U9's symbol carries a pin 9' - schematic parity is 0; the symbol's pin 9 was renumbered 5 (v10-power-parity-2026-09-11.md) and the netlist's U9 has pins 1-8 only. (3) Header: '2,296 track segments' - 2296 at HEAD and V1.0.1, 2295 live (one re-serialised M_SCK segment); harmless. (4) README: U30's cached copy 'is a differently drawn variant ... other pin positions' - the cached Custom:TPS22810DRVR_1 in power.kicad_sch has exactly the seven pins of the library's TPS22810DRVR (same names, numbers, electrical types and positions: VOUT 1 (13.335,7.62), QOD 2, CT 3, GND 4 (0,-8.255), EN/UVLO 5, VIN 6 (-13.335,7.62), GND 7); the lib_symbol_mismatch is cosmetic (lib_name suffix/graphics), not a wiring difference, and the pinout matches the TI DRV package (datasheet §6: 1 VOUT, 2 QOD, 3 CT, 4 GND, 5 EN/UVLO, 6 VIN). (5) README: 'C130's outline over four pads' - three silk_over_copper items (C43.1, R59.1, U19.9) plus two silk_overlap items (R62, U19). (6) On-board Dwgs.User note still reads '0.30 mm PITCH SMD' (A7 admits this) and its ENIG list omits U14 which the file's A2 includes.

**Evidence.** out/live-mini/drc.json: 23 violations, six lib_footprint_mismatch, schematic_parity [] ; out/live-mini/netlist.xml U9 pins 1-8; pcbnew track counts live 2295 / head 2296 / mini-v101 2296; power.kicad_sch lines 5842-6070 vs symbols/Custom.kicad_sym TPS22810DRVR pin blocks; tps22810.txt lines 172-215; drc.json silk items; live kicad_pcb User.Drawings text.

**Consequence.** None functional; the header numbers are what an assembler or a future reviewer reads first, and the U9 parity sentence describes a state that no longer exists.

**Fix.** Update the FABRICATION-NOTES header block (23 items / six library-copy warnings / no parity item) and README's U30 and C130 sentences; regenerate the on-board note text from A2/A7 if the owner wants them to match.

#### 79. [Note] Net names inherited from V9 mislead on this board: VBAT_J8 is the pack input from J3 (J8 is the SMA here), and several multi-node power nodes are auto-named

- **Refs:** J3, Q11, R72, U23, U21, U18, U15, CR3, C76, R51, R62
- **Nets:** VBAT_J8, VBAT_Terminal, Net-(J6-VBUS), Net-(U18-AVIN), Net-(U15-CHIP_PU), Net-(C12-Pad1), Net-(U16-ANT)
- **Reviewer:** `mini-erc-bom-5`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** VBAT_J8 (J3.2 -> Q11 drain) names the V9's pack connector; on the mini J8 is the 900 MHz SMA and the pack enters on J3 (JST PH). Net-(J6-VBUS) (7 members: J6 VBUS pads, CR3.2, C76, R51, R62, U21.7) is what every document calls 'VBUS'; Net-(U18-AVIN) (L5.2, C43, C65, U18 AVIN/PVIN/EN) is the buck input node and even owns a B.Cu zone under that auto-name; Net-(U15-CHIP_PU) is unlabelled while its FC twin is FC_CHIP_PU; the two RF feeds are Net-(C12-Pad1) and Net-(U16-ANT).

**Evidence.** dump.py nets on out/live-mini/netlist.xml; pcbnew zone list (B.Cu zone net Net-(U18-AVIN), 11.5 mm2; B.Cu zone VBAT_J8 9.3 mm2).

**Consequence.** Cosmetic; bench probing and doc cross-references use names the netlist does not have (VBUS), and VBAT_J8 invites the wrong connector on the bench.

**Fix.** Label them (VBUS, V_BUCK_IN or BUCK_VIN, OC_CHIP_PU, BLE_FEED, LORA_FEED, VBAT_J3) when the schematic is next opened; no electrical change.

#### 80. [Note] C130 '+' and can outline are clipped by pads (C43, R59, U19 pad 9) - silk under the supercap is partly unprintable

- **Refs:** C130 (B, pads at 81.45/86.45,129.46), C43 pad 1 (79.13,129.71), R59 pad 1 (89.11,126.87), U19 pad 9 (89.015,123.655)
- **Nets:** V_SCAP, GND
- **Reviewer:** `mini-pcb-visual-4`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** The C130 polarity '+' (glyph 92.43..93.39 x 134.66..135.62 is J3's; C130's '+' is at 79.11..80.81 x 128.65..130.13) runs into C43's pad, and the can outline segment at x=89.15 crosses R59 pad 1 and U19's exposed pad, so the fab clips them. B8 already says 'confirm polarity against the schematic, not the silkscreen', so this is a known limitation, recorded for completeness.

**Evidence.** drc.json silk_over_copper items at (79.96,129.39), (89.15,126.96) x2; silk_overlap of C130 outline with R62 (78.71,125.95) and U19 (86.955,126.215) silk. pcbnew: C130 pad 1 = V_SCAP square pad, pad 2 = GND round pad; '+' text adjacent to pad 1 (correct side). Crop crop_C130_plus.png shows the bar entering C43's pad.

**Consequence.** Cosmetic; polarity is still identifiable by the square pad 1.

**Fix.** None required; if the owner touches the area, drop the can outline (B.Fab already carries it) and move the '+' 0.4 mm in -x.

#### 81. [Note] No reference designators on either side and no pin-1 dot on the two ESP32 footprints (same as V9)

- **Refs:** all 230 footprints (Reference.IsVisible()=False on every one); U15 (88.51,136.785, rot 180) and U32 (84.525,150.86, rot 0)
- **Reviewer:** `mini-pcb-visual-5`, confidence 0.95
- **Verification:** not run (single source).

**Claim.** The silkscreen carries no refdes at all (0 of 230 visible; the V9 is the same at 0 of 255), and the IC_ESP32-S3 footprint's silk is four identical 0.6 mm corner brackets with no pin-1 marker; pin 1 is identifiable only from the fab layer/3D (U15 pad 1 at 91.935,139.385 = bottom-right corner; U32 pad 1 at 81.10,148.26 = top-left corner, both consistent with the 3D model dots). First-article debugging therefore needs the KiCad file or an assembly drawing at the bench.

**Evidence.** fps.txt dump: refvis=False for every footprint; U15/U32 GraphicalItems on F.Silkscreen: 8 lines forming 4 symmetric L brackets (e.g. U15: 84.947..85.543 x 133.221..133.819 and the three mirror images), no circle/polygon. V9 rocket-computer.kicad_pcb: refvis 0/255, same footprint name.

**Consequence.** Not a defect on a board this dense and identical to the working V9; noted so the synthesis knows that visual pin-1/refdes checks on the bench must use the fab drawing.

**Fix.** Optional: add a single silk dot at the pin-1 corner of U15/U32 in the board-copy footprint (there is free silk at 92.3,139.9 and 80.7,147.8).

#### 82. [Note] PYRO1_EXT runs 0.225 mm from the right and bottom board edges on B.Cu (same class as the accepted D+/VBUS edge runs)

- **Refs:** B.Cu tracks net PYRO1_EXT at (94.74,158.99), (93.93,158.18), (94.74,165.71), (94.73,167.07), (90.51,171.29), (77.59,171.29)
- **Nets:** PYRO1_EXT
- **Reviewer:** `mini-pcb-visual-7`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** Besides the two accepted 0.23 mm edge runs (USB D+ on In2 at x=72.74, VBUS on In5 at x=72.89), the PYRO1 firing line hugs the right edge (x=94.73-94.74, y 158-167) and the bottom edge (y=171.29, x 77.6-90.5) at 0.225 mm, above the 0.20 mm rule. Decisions say mins are the fab's promise (#1031), so this is not a finding; noted because a pyro output line at the edge is the one that would matter if a routing nick exposed copper against a metal airframe.

**Evidence.** pcbnew edge-distance sweep of all tracks (< 0.25 mm listed above); DesignSettings copper-to-edge 0.20 mm; drc.json has no copper_edge_clearance violations.

**Consequence.** None on a clean board; an exposed edge copper sliver on this net could short PYRO1 to a conductive airframe/coupler.

**Fix.** None required by decision; if the owner revisits the pyro pours, moving the PYRO1_EXT edge segments 0.1 mm inboard costs nothing.

#### 83. [Note] S1 'O'/'F' labels are electrically correct and match the V9's convention; the slide-switch actuator convention itself could not be datasheet-verified

- **Refs:** S1 (JS202011JCQN, B, 77.30,141.22, rot -90); texts 'F' (80.26..81.25 x 137.19..139.06) and 'O' (72.45..73.90 x 137.24..139.11); U1 FSUSB63, R1, R2
- **Nets:** SEL0, Net-(U1-SEL1), FC_D+, FC_D-, OC_D+, OC_D-
- **Reviewer:** `mini-pcb-visual-8`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** S1 pad 6 (common, 77.30,142.42) = SEL0, pad 7 (79.80,142.42) = GND, pad 5 (74.80,142.42) = NC; R2 pulls SEL0 to +3V3 and R1 pulls SEL1 to +3V3. FSUSB63 functional table: SEL1=1,SEL0=0 -> port 2 = HSD2 = FC_D+/-; SEL1=1,SEL0=1 -> port 3 = HSD3 = OC_D+/-. So the GND end (pad 7, x=79.8) is FC and the 'F' label sits at that end (x 80.3-81.3); the NC end (pad 5, x=74.8) is OC and 'O' sits there (x 72.8-73.7). The V9 uses the identical part, identical wiring (S1.6 SEL0, S1.7 GND, HSD2 = CEN = flight computer, HSD3 = OUT) and places 'O' at its pad-5 end and 'F' at its pad-7 end - the same convention, on a board that works. What I could not verify is the C&K datasheet's actuator convention (knob over the connected terminal), because every C&K/Littelfuse/Digi-Key/Mouser URL returned 403 or timed out.

**Evidence.** dump.py pins S1/U1/R1/R2 (live-mini netlist.xml); fsusb63.txt lines 103-108 (functional table) and 86-99 (pin table); V9 netlist S1/U1 pins and V9 silk texts 'O' at (76.98,164.49) next to pad 5 (76.05,163.78), 'F' at (76.98,169.41) next to pad 7 (76.05,168.78); crop_S1_OF.png shows pad 7 merged into the GND pour at the 'F' end.

**Consequence.** If the V9's labels are right on the bench, the mini's are right. If the actuator convention were the opposite, both boards would be mislabelled the same way, which the owner would already have noticed on the V9.

**Fix.** Owner to confirm on the V9 which physical knob end enumerates the FC; no change expected.


### Second reports of the same defects, and strays

Most of what follows is a **second, independent reviewer reaching a finding already made
above**, from a different starting point. That is corroboration, not padding, and it is
kept because two independent derivations of the same geometry are the nearest thing this
run has to the verification pass that never executed. Three of the strongest findings in
this report are here twice for that reason: the exposed-pad paste contradiction (items 85
and 93 restate item 14), the shared transistor land (item 86 restates item 24), and the
power-off ordering (item 87 restates item 69). One entry, item 88, is about the full
computer and is repeated in that board's report.


#### 84. [Minor] WLCSP U13/U33 ball pads are mask 1:1 (0.254 mm opening on 0.254 mm pads, no expansion) - the mask edge sits on the pad edge and registration decides whether each ball is NSMD or SMD

- **Refs:** U13, U33 (Footprints:24-WLCSP_Y_WIN) · **Reviewer:** `mini-dfm-paste-6`, confidence 0.7

**Claim.** Winbond 10.7: ball b = 0.24/0.30/0.36 mm at 0.5 mm pitch; the pad is 0.254 mm (0.85x ball, an NSMD size) but the mask opening equals the pad (maskE = 0), so any mask misregistration covers part of the pad and the 0.254 mm paste aperture (AR 0.794, the board's lowest) lands partly on mask.

**Evidence.** pcbnew: U13/U33 pads circle 0.254, layers F.Cu/F.Mask/F.Paste, GetSolderMaskExpansion = 0; mask gerber C0.254 x48; paste C0.254 x48. TI's DSBGA land example (tps2121 YFP): 0.25 mm metal, NSMD, '0.05 MAX' mask pull-back. JLC: 'soldermask opening can be 1:1' (allowed, not recommended). V1.0.1 already cleared vias from every ball pad (nearest drill edge 45 um per the fab note; vip.py finds no via inside or crossing any U13/U33 pad).

**Consequence.** Reduced and asymmetric wettable area on 0.3 mm balls at the board's tightest area ratio; opens on the boot NORs (boards boot from these). Does not affect the passive CC pull-downs.

**Fix.** Give the WLCSP pads a 0.05 mm mask expansion (0.35 mm opening) or make them 0.30 mm SMD with a 0.25 mm opening per Winbond/TI practice; keep the 0.254 paste aperture.

#### 85. [Minor] FABRICATION-NOTES B1 says U18/U19/U47 exposed pads have no paste aperture; every board version and the shipped V1.0.1 B_Paste gerber carry them

- **Refs:** U18, U19, U47, U30 (FABRICATION-NOTES.md B1; decisions.md 'U18/U19/U47 exposed pads get NO paste') · **Reviewer:** `mini-erc-bom-1`, confidence 0.95

**Claim.** The assembler-facing note states that three exposed pads (U18 buck 2.8 mm2, U19 eFuse 6.9 mm2, U47 hold-up 2.7 mm2) 'HAVE A MASK OPENING AND NO APERTURE, SO THEY GET NO PASTE ... DO NOT SUBSTITUTE A STENCIL THAT HAS THEM', 'checked 2026-09-12 against the V1.0.1 release zip F.Paste gerber'. All four of these parts are on B.Cu, and their EPADs do have paste apertures in the V1.0.1 tag, HEAD and the live board, and in the plotted B_Paste gerber.

**Evidence.** pcbnew on live/head/mini-v101 rocket-computer-mini.kicad_pcb: U18 (B.Cu, 75.10,131.905) EPAD pad 17 1.68x1.68 has no B.Paste on the pad but a 1.07x1.07 mm fp_poly on B.Paste centred on it (1.14 mm2 = 41 %); U19 (89.015,123.655) EPAD pad 9 2.29x3.0 plus four aperture-only pads 0.96x1.25 at (89.585/88.445, 124.405/122.905) = 4.8 mm2 = 70 %; U47 (84.4,135.62) EPAD pad 13 1.0x2.65 plus two aperture-only pads 0.95x1.17 at (84.4, 134.935/136.305) = 84 %; U30 (84.28,140.06) EPAD pad 7 1.0x1.6 plus two 1.0x0.7 aperture-only pads = 87 %. gerbers/rocket-computer-mini-B_Paste.gbr (plotted 2026-09-06 from the tag): %ADD47R,0.950000X1.170000 flashed at X84400000Y-134935000 and Y-136305000 (U47); round-rect flashes at X89585000Y-124405000, X89585000Y-122905000, X88445000Y-124405000, X88445000Y-122905000 (U19); X83830000/X84730000 Y-140060000 (U30); region vertex X74570000Y-131375000 (U18 poly). The same coordinates do not appear in F_Paste.gbr (0 hits), which is the file the note says was checked. The shared library QFN50P300X300X100-17N has four 0.65x0.65 aperture pads + one poly, i.e. the board copy of U18 differs from the library only in the paste construction (the lib_footprint_mismatch warning). TPS62152 datasheet (tps62152.txt lines 196, 1763, 2271): the thermal pad 'must be soldered to the printed circuit board for thermal and mechanical performance'.

**Consequence.** The assembler is told to withhold paste from three thermal pads that the stencil cut from the gerbers already prints, and told not to accept a stencil that has them; following the note literally means masking apertures that the datasheets want pasted. For this review it also changes what the first article received: the V1.0.1 stencil DID paste all four back-side EPADs, so any 'the EPADs were unsoldered' hypothesis from the note is wrong. Also relevant to decisions.md, which records the note's claim as fact.

**Fix.** Rewrite B1 to state the plotted truth per side: back side U18 41 % (one aperture), U19 70 % (four), U47 84 % (two), U30 87 % (two); front side U15/U32 55 % (nine), U11 76 % (six), U5 pad 12 74.5 % (four). Drop the 'do not add paste / do not substitute a stencil that has them' instruction or re-justify it against B.Paste. Correct the decisions record.

#### 86. [Major] U9 AON7534 (DFN3x3A, 3.0 mm body, 0.40 mm leads) sits on the 3.3 mm TSON-Advance land: source/gate leads overlap their pads by ~0.05 mm nominal and the drain-tab pad runs to within ~0.13 mm of the source leads

- **Refs:** U9 (mini and live V10), footprint Footprints:TSON Advance_TOS (tags list AON7534) · **Reviewer:** `mini-erc-bom-2`, confidence 0.6

**Claim.** The arm FET's package is AOS DFN3x3A_8L_EP1_P (PO-00047): body D=E 3.00 (2.90-3.10), lead length L 0.40 (0.30-0.50) from the body edge, lead width b 0.30, pitch 0.65, drain tab D1 2.35 x E2 1.75 merged into pins 5-8; AOS's recommended land has four 0.35 x 0.50 source/gate pads whose outer edge is 1.65 mm from centre and a 2.45 x 2.28 tab land. The placed footprint was drawn for a 3.3 mm TSON Advance: perimeter pads 0.381 x 0.66 centred at +/-1.778 mm (spanning 1.448-2.108) and a 2.5 x 2.5 tab offset 0.28 toward the drain side (spanning -1.53..+0.97). Against the AON7534 the source/gate leads (1.10..1.50 from centre) overlap the pad copper by 0.05 mm nominal (0.10 at the +0.10 body max), 0.20 mm of the 0.50 mm AOS land; and the tab pad extends 0.34 mm past the AOS tab's inner edge (+0.63) to +0.97, 0.13 mm from the start of the source leads, with 1:1 paste on the 6.25 mm2 tab pad (~2x the AOS tab-land area).

**Evidence.** Board: live rocket-computer-mini.kicad_pcb U9 at (92.985,161.138) rot 90: pads 1-4 at y=162.916 (delta +1.778) 0.381x0.66; pads 5-8 at y=159.36 (delta -1.778); tab pad '5' 2.5x2.5 at (92.995,160.858) (delta y -0.28); all pads carry F.Paste 1:1. AON7534 datasheet rev 1.1 page 1 (rendered): 'DFN 3x3 EP', bottom-view photo shows four discrete leads on the S/G side and the tab merged with pins 5-8 = the 'A' variant; pinout S1 S2 S3 G4 D5-D8 matches the netlist (U9.1-3 GND, U9.4 gate, U9.5-8 PYRO_GND). AOS PO-00047 rev G (DFN3x3A_8L_EP1_P, text + rendered drawing): D 3.00, b 0.30, L 0.40, e 0.65, D1 2.35, E2 1.75, L2 0.43; recommended land 3.30 tall, source pads 0.35 x 0.50 at the bottom edge, tab land 2.45 wide x 2.28. Reference: the V9 (fabbed, working) carries CSD16323Q3 (TI SON 3.3x3.3, 0.5 mm leads) and TPN4R712MD (Toshiba TSON Advance 3.3x3.3) on this footprint, for which the lead-to-pad overlap is ~0.20 mm plus a wettable flank. mini-part-parity-2026-09-12.md row U9 says 'pinout and land verified on the mini' without numbers; the WSD20L50DN33 row says its land was closed 'by numeric overlay of the vendor drawing' - no such overlay is recorded for the AON7534.

**Consequence.** The three source joints and the gate joint of the pyro ARM FET rely on a toe fillet against a sawn DFN lead end, with most of the pad outside the package; a weak or open source joint raises the fire-path resistance or opens it (no pyro fire), an open gate joint leaves the arm FET's gate floating, and excess tab paste squeezed toward the source leads can bridge drain (PYRO_GND) to source (GND), defeating the arm switch. Pyro-path, flight-safety relevant. Not related to the first article's USB symptom.

**Fix.** Discuss, do not draw: (a) give U9 its own footprint derived from AOS PO-00047's recommended land (four 0.35 x 0.50 pads at 0.65 pitch with outer edges at +/-1.65, 2.45 x 2.28 notched tab with windowed paste), or (b) pick an arm N-FET in a true 3.3 x 3.3 TSON-Advance-class package (the same family as the fire FETs) so the shared footprint is right for all five. Either way, inspect U9 on the first article (angled view of pads 1-4, or X-ray) before trusting the arm path. Same applies to the live V10 (U9 AON7534 on the same footprint).

#### 87. [Minor] OC power-off flushes the log to a NAND that is on the rail it just dropped — the ordering was designed for V9's always-on NAND

- **Refs:** U11, U30, C105, R84, D9; out_computer/main/main.cpp power-off branch (digitalWrite(PWR_PIN, LOW) then quiesceStorageForRestart("power-off")), board_m1.h · **Reviewer:** `mini-memory-link-1`, confidence 0.65

**Claim.** On the mini the NAND's VCC (U11.8) is V_MCU_SWTCH, but the OC's deliberate power-off path drops PWR_PIN first and only then quiesces the logger ("AFTER the rail drop on purpose — the wait doubles as rail-discharge time"), with an in-code justification that U11 "has VCC on +3V3 … ALWAYS ON". That justification is V9's (v9 netlist: U11.8 = +3V3); on the mini the NAND only stays alive for the C105/R84 enable-decay window while the flush runs.

**Evidence.** live-mini netlist: U11.8 VCC_8 = V_MCU_SWTCH; U30 EN/UVLO = POWER_SWITCH with R84 100 k to GND and C105 10 µF, fed by D9 from FC_EN_OC (OC GPIO7) and FC_EN_HOLD (FC GPIO17). v9 netlist: U11.8 = +3V3, R31/R33 to +3V3. main.cpp (worktree, out_computer) power-off branch: digitalWrite(config::PWR_PIN, LOW) → quiesceStorageForRestart("power-off") (kSpiParkTimeoutMs = 250) → kSwitchedRailPins driven LOW → 100 ms → esp_restart(); the comment block above it states U11 never loses power and that LoRa is a J5 daughterboard, both V9 facts. README 'The flight computer starts off' gives the enable decay as 0.94 s calculated and ~0.45 s with C105's DC-bias loss, and says the FC does not hold the rail before launch detect — so on the ground the NAND's supply is on that timer from the moment PWR_PIN falls. board_m1.h sets NAND_CS = 36 and LORA_SPI_* = SPI_* (shared bus), so the code path is live for TR_BOARD_M1.

**Consequence.** If the close/park takes longer than the derated ~0.45 s window (250 ms park timeout plus file closes on a slow NAND erase), the last page program or a LittleFS metadata commit is cut by U30 turning off: the tail of the flight log is lost or a page is left corrupt. LittleFS's power-loss design limits this to the tail; nothing else is at risk. Ground power-off only; in flight the FC holds the rail and the off is refused.

**Fix.** Firmware, not hardware: under TR_BOARD_M1 run quiesceStorageForRestart() BEFORE digitalWrite(PWR_PIN, LOW) (stop I2S ingest first so the FC's continuing frames do not matter), or bound the whole quiesce to well under the ~0.45 s window and state the dependency on C105 in the code. Either way rewrite the V9-specific comment block for the mini's rails.

#### 88. [Note] V10 did not pull in the mini's GPIO3 (ESP_I2S_SD) pull-down; the S3's strapping pad still floats at reset while the P4 is unpowered

- **Refs:** U15 pin 8, U17 pin 20 (mini: R34 100 k) · **Reviewer:** `mini-oc-s3-5`, confidence 0.7

**Claim.** The mini added R34 100 k to GND on ESP_I2S_SD because the far end (the FC) is unpowered while the OC boots and GPIO3 is a strapping pin; the V10 live schematic carries ESP_I2S_SD between U15.8 and U17.20 with no resistor, the same as the fabbed V9.

**Evidence.** Netlist live-mini: ESP_I2S_SD = R34.1, U15.8, U32.18 (R34 100 k, R34.2 GND). live-v10: ESP_I2S_SD = U15.8, U17.20 only; v9: ESP_SDO = U15.8, U17.20 only. S3 datasheet Table 3-1: GPIO3 default 'Floating'; §3.4 the JTAG-source strap is only honoured when EFUSE_STRAP_JTAG_SEL is burnt. pin-budget.md 'Correction — GPIO3 no longer has the pulldown this document claimed' records the mini's reasoning.

**Consequence.** No effect on an unburnt part (V9 proves it); it is a parity gap against the mini's deliberate fix and would matter only if the JTAG-select eFuse were ever burnt.

**Fix.** Add a 100 k pull-down on ESP_I2S_SD to the V10 schematic to match R34, or record in mini-part-parity-2026-09-12.md that it was left out on purpose.

#### 89. [Note] Unpasted exposed pads (owner decision): vias are present and on GND, but the buck's heat path is thin

- **Refs:** U18 pad 17 (75.10,131.905), U47 pad 13 (84.40,135.62), U19 pad 9 (89.015,123.655) · **Reviewer:** `mini-pcb-power-4`, confidence 0.75

**Claim.** All three no-paste exposed pads have GND thermal vias (U18: 1 at the pad centre; U47: 3; U19: 5, pad connection forced solid) and their pins that must tie to the pad are on GND copper. With the pads unsoldered by decision (FABRICATION-NOTES B1) the only heat path is the perimeter pins; U18 has just 11.3 mm2 of B.Cu GND and 3 GND vias within 3 mm.

**Evidence.** epads.py: vias inside U18.17 [(75.10,131.91)], U47.13 [(83.96,135.10),(84.45,135.95),(84.42,134.82)], U19.9 five vias, all net GND, drill 0.3; U19.9 zone_conn=2 (solid). TPS62152 datasheet pin table: EPAD 'Must be connected to AGND (pin 6), PGND (pin 15,16)... Must be soldered to achieve appropriate power dissipation'; section 11.1: AGND and PGND 'are directly connected to the exposed thermal pad' inside the package, so electrical GND is intact through pins 5/6/15/16 (all on the B.Cu GND island with 91 vias). Thermal: TPS62152 RthJA 45 C/W (pad soldered); no unsoldered figure published. TPS2596 gives 52.7 C/W soldered vs 119.8 C/W unsoldered for the same situation. At 1 A out from 8.4 V with ~90 % efficiency (12 V curves) the buck dissipates ~0.4 W: +18 C soldered, ~+40 C if the unsoldered ratio is similar; at the realistic 0.3 A (~0.13 W) ~+13 C. U19 at its 0.9 A limit (R48 1 k, datasheet 909 ohm -> 1.005 A) dissipates 0.089 x 0.81 = 72 mW: +9 C even at 119.8 C/W. U47 bypass: 0.1 ohm x 0.7 A2 = 49 mW. B.Cu GND copper within r=3 mm: U18 11.3 mm2 / 3 vias, U47 11.4 mm2 / 8 vias, U19 19.1 mm2 / 10 vias.

**Consequence.** No electrical defect; the buck junction runs perhaps 40 C above ambient at the 1 A corner with the pad unsoldered, well inside 125 C, and only ~13 C at flight loads. Recorded so the decision is made with numbers.

**Fix.** None required. If the paste decision is ever revisited, U18 is the part that benefits most; otherwise adding B.Cu GND copper around U18 pins 5/6/15/16 and a via at each would halve the pin-only path.

#### 90. [Minor] U14 chip antenna is rotated 90° from Abracon's layout and its GND end sits on a 0.72 mm edge ground strip inside the recommended clearance

- **Refs:** U14, C7, L1, C12 · **Reviewer:** `mini-pcb-usb-mcu-4`, confidence 0.6

**Claim.** Abracon's recommended layout puts the 1.0 mm axis of the antenna along the PCB edge, 0.3–0.8 mm from the edge, inside a 4.6 mm (along edge) × 3.5 mm (deep) copper-free rectangle that reaches the board edge, with the GND pads at the clearance's ground-side end. On the mini the part is at (94.125,145.47) rot 0: feed pads at x = 93.85, GND pads at x = 94.4, i.e. the 1.0 mm axis is perpendicular to the edge (x = 94.99). The all-layer rule area is 2.89 × 4.59 mm at (91.381,144.633)–(94.272,149.222); the 0.72 mm strip between it and the edge (94.27–94.99) carries GND on F.Cu, In1, In2, In5, In6 and B.Cu, and the antenna's GND terminals sit on that strip. 4.59 along the edge matches the datasheet; the clearance depth is 2.89 mm plus a ground strip where the datasheet has 3.5 mm of clear board to the edge. Inside the rule area no copper exists on any of the 8 layers (verified by sampling).

**Evidence.** keepout.py per-layer sampling (keepout: F.Cu only the feed track + U14 pads; In1–B.Cu empty; edge_strip: GND fill 108/144 samples on F/In1/In2/In5/In6/B); zones.py rule areas; U14 pad list; ANT_AANI-CH-0070_ABR.kicad_mod pads (feed at x = −0.275, GND at +0.275); Abracon AANI-CH-0070 datasheet page 4 render (work/mini-pcb-usb-mcu/aani4-04.png): 4.60 × 3.50 clearance at the edge, pad columns at 0.75/1.40 along the edge, rows 0.30/0.50 from the edge; page 10: 'must be placed along the PCB edge', 'ground clearance is respected through all layers', 'robust via structure around the cutout'. Board has a via fence around the keepout (7 GND vias north, 7 south).

**Consequence.** The loop's near field sees ground on the edge side instead of open board, and the feed end points inward; expect a resonance shift and lower efficiency versus the datasheet curves. The match (C7 5.1 pF series, L1 2.2 nH shunt, C12 DNP shunt) can re-centre frequency but not recover efficiency. BLE is the mini's phone link.

**Fix.** Measure the first article's antenna (S11 at LNA_IN side of C7, or range vs a V9) before deciding. Next spin: rotate U14 so its long axis runs along the edge with the GND pads abutting the ground at the keepout's north end, extend the keepout to the board edge over the 3.5 mm depth (drop the edge strip inside it), keep the via fence.

#### 91. [Minor] FC VDD3P3_CPU (U32 pin 46) reaches its 100 nF only through two vias and the In4 plane

- **Refs:** U32, C120 · **Reviewer:** `mini-pcb-usb-mcu-6`, confidence 0.85

**Claim.** U32 pin 46 (85.925,147.435) leaves on 1.06 mm of F.Cu to a via at (86.0,146.373) into the In4 V_MCU_SWTCH plane; C120 100 nF (85.57,145.8), 1.21 mm from the pin, is not on that F.Cu island — it reaches the pin only through its own via and the plane. Every other S3 supply pin on both processors has its nearest capacitor on a direct F.Cu link (see coverage): OC pin 46 → C31 1.19 mm direct.

**Evidence.** caps.py: U32 pad 46 nearest C120 d=1.21 sameFcuCluster=False, vias<1.2mm=1; V_MCU_SWTCH tracks near pad 46: F.Cu 0.127 (85.925,147.435)->(85.925,146.435)->(86.0,146.373) + via (86.0,146.373); C120 pads (1 V_MCU_SWTCH, 2 GND).

**Consequence.** The CPU-rail decoupling loop includes two ~0.9 mm via barrels and a plane hop (roughly 1–2 nH extra); the rail still works from the plane and C115/C117/C125, so this is marginal, not broken.

**Fix.** Join C120 pad 1 to the pin-46 F.Cu track (they are ~0.3 mm apart) or nudge C120 so its pad lands on the track.

#### 92. [Note] J6 USB-C face is recessed 0.22 mm from the board edge; the V9 places it flush

- **Refs:** J6 (GCT USB4110GFA, F, 84.36,164.79 on both boards) · **Reviewer:** `mini-pcb-visual-6`, confidence 0.7

**Claim.** J6 sits at the identical footprint position on both boards (pads at y=163.715, front outline y=171.345), but the mini's bottom edge is at y=171.565 versus the V9's 171.345, so on the mini the PCB edge stands 0.22 mm proud of the receptacle face. This is within the connector's own 0.30 mm tolerances and far short of the plug overmold gap, so it does not block mating; recorded because it is the one J6 difference from the working reference.

**Evidence.** pcbnew: mini outline y 101.945..171.565, J6 bbox y 162.36..171.345; V9 outline y 96.295..171.345, J6 bbox y 162.36..171.345, same pad coordinates and nets (A5 CC1, B5 CC2, A6/B6 D+, A7/B7 D-, A4/B9+B4/A9 VBUS, S1-S4 GND, pegs 0.65 mm at x 81.47/87.25). GCT drawing text (usb4110.txt) gives 7.35+/-0.30 and 7.70+/-0.30 depth dimensions; the pad-row-to-edge distance is 7.85 mm on the mini vs 7.63 mm on the V9.

**Consequence.** None expected; a plug still seats (the mating stop is the receptacle shell, not the PCB edge).

**Fix.** No change; if the outline is ever revised, pull the bottom edge in 0.2 mm to match the V9.

#### 93. [Note] U18 and U47 exposed pads DO carry stencil paste on the bottom side (40 % / 84 %); B1 and decisions.md say none — the check was made on F.Paste for parts that sit on B

- **Refs:** U47 (TPS61094, WSON-12 EP 1.0x2.65), U18 (TPS62152, QFN EP 1.68x1.68), U19 (TPS259631 EP), FABRICATION-NOTES.md B1, decisions.md · **Reviewer:** `mini-power-1`, confidence 0.9

**Claim.** FABRICATION-NOTES B1 states that U18, U19 and U47 exposed pads have 'a mask opening and no aperture, so they get no paste', checked 'against the F.Paste gerber' of the V1.0.1 zip, and decisions.md repeats it. All three parts are on the BACK side (B.Cu). The board file (live and the rocket-computer-mini-v1.0.1 tag, identical) carries a B.Paste polygon on U18's EP and two paste-only aperture pads on U47's EP; U19's EP alone has none. A fresh kicad-cli B.Paste plot of the v1.0.1 board confirms they reach the gerber.

**Evidence.** pcbnew on $S/mini-v101/.../rocket-computer-mini.kicad_pcb: U18 GraphicalItems: B.Paste POLY bbox (74.56,131.37)-(75.64,132.44) = 1.07x1.07 mm over the 1.68x1.68 EP pad 17 (layers B.Cu,B.Mask only) = 40 % coverage; U47: two unnumbered pads 0.95x1.17 mm, layer set ['B.Paste'] only, at (84.40,134.94) and (84.40,136.31) over EP pad 13 1.00x2.65 (B.Cu,B.Mask) = 2.22/2.65 = 84 % coverage; U19 pad 9 2.29x3.0 (B.Cu,B.Mask), no paste graphics. kicad-cli pcb export gerbers --layers B.Paste of the v1.0.1 board -> $S/work/mini-power/gerb/rocket-computer-mini-B_Paste.gbp: lines 76-88 a G36 region X74.57..75.63 / Y131.375..132.435 tagged %TO.C,U18*; lines 848-849 flashes X84400000Y-134935000 and X84400000Y-136305000 inside %TO.C,U47*; no flash at U19's EP (89.02,123.66). Live board identical. FABRICATION-NOTES.md lines 153-160: 'AS PLOTTED (F.Paste GERBER, CHECKED 2026-09-12 AGAINST THE V1.0.1 RELEASE ZIP) ... THREE EXPOSED PADS HAVE A MASK OPENING AND NO APERTURE, SO THEY GET NO PASTE: U18 ... U19 ... U47'.

**Consequence.** The fab note and the stencil disagree, and the stencil is what the first article was built with. U47's EP at 84 % aperture coverage on a 0.08 mm foil, 0.5 mm-pitch WSON with 0.25 mm-wide lead apertures 0.95 mm away, is a classic float/bridge case in a paste-troubled build; U47 is in series with the entire +3V3 rail (bypass FET VIN->VOUT), so a floated U47 or a bridge between pins 2-4 (V_BUCK) and pin 1 (OSEL) / pin 5 (SW) leaves +3V3 dead or the hold-up mis-coded, i.e. no processor boots. U18's 40 % square is unremarkable. U19 truly has none, as documented.

**Fix.** Correct B1/decisions to what is plotted (U18 40 %, U47 84 %, U19 none — plot B.Paste, not F.Paste, for bottom-side parts). Decide the U47 EP coverage on its merits (50-65 % window-pane would match U15/U32 practice); owner's call. On the first article: check U47 seating and pin 1-5 bridging under magnification, and measure VIN(pin 4) vs VOUT(pin 9) with USB only — they should differ by <10 mV in bypass.

## Checked and cleared

Recorded so the next reviewer does not repeat the work. Each line is something a reviewer
verified against the primary source and found correct.

**The USB front end matches the fabricated full computer exactly.** Connector pad map
against the vendor drawing, both configuration-channel pull-downs at 5.11 k on the pads
the drawing brings out, the electrostatic-discharge array, the 2:1 high-speed multiplexer
with its 22 ohm supply feed and bypass, the slide switch, and the power multiplexer are
identical part for part, value for value and net for net. The connector footprint is
byte-identical to the full computer's copy, and I confirmed the pad geometry independently.

**The power multiplexer, buck converter and hold-up converter are configured to their
datasheets.** Every configuration resistor was re-derived: the hold-up converter's output
select at 3.0 V, charge voltage 2.5 V and charge current 100 mA all match the datasheet
tables, and the electronic fuse's undervoltage thresholds compute to 6.96 V on and 6.38 V
off from the fitted divider.

**The arming interlock is wired as the rework specified.** Both transistor terms, the
consent transistor in series, the gate resistor and the pull-down are present and match
the drawing; the fire pins sit on pads with no reset pull-up and each carries a 5.11 k
pull-down.

**The processors' pin assignments are consistent with the netlist**, the hold pin sits
inside the range where the latch survives a panic reset, and the barometer, inertial
sensor, magnetometer and satellite receiver are on the buses the pin budget records.

**The boot flash ball maps match the manufacturer's assignment** and no via sits inside
any ball pad, which was the fabricator's own query on this board.

**The satellite module's land, its antenna-feed keep-out and its pad-12 window** are as
the fabrication notes describe, and the four fiducials are visible at both paste stages,
so an occluded fiducial is not a candidate cause of the misprint.

## Open questions for the owner

These need a decision rather than a fix, and several are documentation the review cannot
change on its own.

1. **The assembly note tells the assembler the wrong thing about paste.** Note B1 says
   three exposed pads get no paste and forbids a stencil that has them. All three are
   pasted on every version of the board including the one that shipped, and two more parts
   are pasted and unmentioned. This is the note that a paste post-mortem would start from.
2. **Does the arm interlock keep its two-processor property during flight-computer
   resets?** It does not today: the flight-computer term is held on by that pin's reset
   pull-up, so the consent transistor is the only barrier during boot, reset and flashing.
   A pull-down would restore it. The alternative is to correct the sheet note, which
   currently says the opposite.
3. **Should the shared transistor land be split?** The arm switch is a 3.0 mm part on a
   land drawn for 3.3 mm parts, and its terminals reach the pads by about 0.05 mm against
   0.20 mm for the four channel switches.
4. **The magnetometer keep-out that the parity document promises does not exist.** No
   rule area sits under it on any version of this board, and the full computer's placement
   guidance tells that board to copy it.
5. **Should the two processors get the decoupling the full computer gained in September?**
   The supply-pin bulk capacitor, the flash-domain 100 nF and the external boot pull-up
   went onto the full computer and not onto this board.

## Bench and first-article items

For #1316. Each is a measurement, not a change.

- Probe the configuration channels and the connector's signal row before anything else;
  the sequence is in `first-article-bringup.md`.
- Inspect the connector's signal-row fillets at 30 to 40 degrees, and both pull-down
  resistors under magnification for a lifted end.
- Caliper the receptacle face against the board edge and check whether a plug clicks home.
- Inspect or X-ray the arm switch's source and gate joints before trusting the arm path.
- Scope the arm gate and the flight-computer arm line through a power-on, a reset and a
  download session; the gate must never reach the switch's turn-on voltage.
- Measure the hold-up converter's start-up on USB alone with no supercapacitor fitted.
- Confirm whether the printer recognised the fiducials cleanly, and ask the assembler
  which foil was actually used.
