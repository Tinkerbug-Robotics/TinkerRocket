# Tinker-Beetle V2 — pre-production review, 2026-11-24

This reviews the board as it stood in the working copy on 2026-11-24: `tinker-beetle.kicad_pcb` saved
06:16, `gnss.kicad_sch` sha a5565876, `pyro.kicad_sch` sha edd15988, all uncommitted on main. It covers:
- the physical board;
- component selection;
- layout;
- how closely each circuit follows its maker's reference design.

The two newest circuits got the closest look: the PX100 GNSS receiver with its L1/L5 front end and
patch, and the TPS2HC16-Q1 pyro switches.

Nothing in the project was changed. Everything was read from copies.

## How it was checked

**Source data.** Pin maps came from `kicad-cli` netlist exports. Geometry came from the board through
`pcbnew`. ERC and DRC were run with `--severity-all`, and DRC was run both on the board as saved and
after a refill.

**Reference documents:**
- SkyTraq PX100-5C-01A data sheet v2: reference schematic and BOM.
- TI SLVSI29, December 2025 (TPS2HC16-Q1).
- Taoglas GVLB258.A spec rev I.
- Laird BMI-S-102 drawing.
- Molex PS-479480001 rev F.
- Espressif ESP32-S3 hardware design guidelines.
- Ebyte E220-900MM22S manual.
- MEMSIC MMC5983MA data sheet.
- TI data sheets for every power part.
- CDA CXHP supercap sheet.

**Tested comparisons.** I compared against the PX100 carrier V2 as ordered, which is built and
tracking, and against the V9 and V1 Beetle boards.

**Who did what.** I reviewed the GNSS, RF, pyro and physical board myself. Two further reviewers took
the power tree and the processors, sensors and radios. Every finding from all three was then checked
again by two independent checkers who tried to refute it: one re-measured the files, the other judged
the consequence and severity against the data sheets. A completeness critic then looked for anything
missed.

**How the result is reported.** What follows is what survived that check, with the corrected numbers.
Where the checkers moved a severity, the item says so.

Figure: `~/Projects/ModelRockets/report-figures/beetle-v2-gnss-review-2026-11-24.png` (the GNSS corner,
annotated).

## State of the files

| Check | Result |
|---|---|
| Netlist | 286 parts, 273 nets, 274 fitted parts on 88 part numbers |
| DRC, after refill | 0 unconnected, 0 schematic-parity, 9 courtyard overlaps at J2 (accepted at routing) |
| DRC warnings | 9 silk overlaps, 9 silk over copper, 2 silk at the SMA edge, 2 text heights, 5 library-copy mismatches, 1 logo library lookup |
| DRC on the file as saved | adds 4 clearance and 4 mask-bridge errors: the GND fill is stale over the four fiducials |
| ERC | 701 total: 666 off-grid, 22 pin-type warnings, 1 symbol mismatch (U30), and 12 errors |

The 12 ERC errors are all out-computer spare pins without no-connect flags. Seven come from the
11-21 pin reassignment; GPIO9, 12, 43, 44 and 45 were already unflagged. None is a real connection
problem.

- **Board:** 26.83 x 70.35 mm at the cut line. 8 layers. 632 vias, all 0.4/0.3.
- **Board setup:** filled and capped vias are already set.
- **Via-in-pad is how this board is built:** 326 vias sit centered in SMD pads, and 48 more cut a pad edge.

---

## Must resolve before ordering

### 1. The BLE chip antenna has left the board edge (U14, Molex 0479480001)

**Where it is.** U14 is on the back at (90.48, 152.69). Its body is about 5.1 mm inside the right edge
and 17.9 mm above the bottom. In HEAD (0d1ec0b9) it sat 0.4 mm from the edge, as on the V9 that flew.
It was already in the new spot in the 11-21 06:40 save, before routing, so it moved by hand when the
board widened. The router's move list does not include it.

**What surrounds it:**
- The out computer's 40 MHz crystal Y2 sits on the front over the antenna's feed corner, covering
  about 47 % of Y2's own body.
- Y2's crystal-side XTAL_P node crosses the antenna's outline.
- H4's floating 3.8 mm plated ring and the M2 screw are 0.8-1.05 mm from the antenna body.
- The WAGO terminal J2 is 0.9 mm away, body to body.
- 2480 MHz (BLE advertising channel 39) and 2440 MHz are both harmonics of 40 MHz.

**What Molex asks for.** PS-479480001 §2.3 lists "PCB corner mounting". §9.3 draws a 4.2 x 4.2 mm
copper-free area at the corner on the antenna's own layer. The "no ground clearance needed" line the
README quotes comes from Molex sales copy and means the inner and opposite-side ground may stay. The
V9 flew with ground under its antenna on every layer, but it was at an edge.

**Fix:**
- Put U14 back at an edge, ideally a corner, on the back, with the feed pad toward the edge. The
  checkers found three candidate spots:
  - the right edge where the boot button SW3 sits (y 155-160), moving SW3 inboard;
  - the bottom-right corner (x 94-97, y 160-167), moving the LED D8, R68 and FID3, though it is close
    to J2's wire exit;
  - HEAD's old spot beside U15 (y 146-150), moving D13/D14. This gives the shortest feed but still sits
    near H4.
- Clear B.Cu in the 4.2 x 4.2 mm area; inner ground can stay.
- Keep it several mm from H4, J2, Y2 and Y4, or tie H4's ring to GND.
- Redo the ESP32 feed run as a short grounded line on the antenna's face (see item 21).
- Correct the README.

The footprint's 0.86 mm pads are the part's own terminals, 41 % of Molex's 1.35 mm land. The V9 flew
on them, so widening them is optional.

### 2. The PX100 cannot be recovered once the shield is on (BOOT_SEL)

**What is there:**
- BOOT_SEL (U5 pin 30) reaches only R14's 1 k pull-up. R14 is on the back inside the one-piece shield
  (and under C130's can), with no via.
- TP1/TP2 were removed, but README lines 115-116 and a schematic note still describe them.
- RSTN is exposed only at R7/C62 on the front.
- The flight computer has no hardware reset of the receiver. It cannot drop V_MCU_SWTCH on its own
  either, because U30's enable is diode-ORed with the out computer's FC_EN_OC.

**Why it matters.** The PX1105R/PX100 reflash failures seen so far needed a true reset plus a UART
reflash. BOOT_SEL (ROM boot) is the last resort, and on this board it is the one line that can never
be reached after the bake.

**Fix, cheapest first:**
- **Wire RSTN to flight-computer GPIO2.** Drive it open-drain, or through 220-470 ohm, so the pin does
  not dump C62. Keep R7/C62.
- **Wire BOOT_SEL to GPIO15 or GPIO16.** R14 still holds it high while the flight computer is in reset
  or unpowered, since they share the rail.
- **Result.** The flight computer can then reset the receiver and force ROM boot over its own UART,
  with no fixture.
- **For a bench path as well,** add one small BOOT_SEL pad on the front near R7/C62. J9 has no room in
  line.
- **GPIO43 is free but unsuitable.** It is the boot-ROM log TX and has no via site.

### 3. The fab notes and the KiCad stackup are V1

FABRICATION-NOTES.md, and its copy on the board's drawing layer, is the V1 LC86G document at
22.55 x 69.62 mm. Rewrite block A for V2 with:
- 26.83 x 70.35 mm;
- 8 layers on JLC08161H-2116, 1.6 mm, 1 oz outer and 0.5 oz inner;
- ENIG, for the 0.4 mm-pitch PX100, the two ESP32-S3s, the TPS2HC16s and the LGAs;
- **all vias epoxy-filled and copper-capped, via-in-pad throughout**. Drop "all other vias tented"
  and the LC86G rationale.

Also:
- **Through-hole list:** add AE1, C130, J2 and J6's legs.
- **Assembly block:** item 15 (bake order) and item 16 (C130).
- **No paid impedance test is needed.** The RF lines are 7.2 mm or shorter and work out to about
  50 ohm on the real stack.

The KiCad stackup no longer matches JLC08161H-2116:
- every dielectric thickness differs (0.100 / 0.333 / 0.100 against 0.1164 / 0.300 / 0.1528);
- the lower half has In5/In6 at 35 µm and generic FR4 at er 4.5, including the B.Cu-to-In6 layer that
  sets the GNSS lines.

Restore the stack V1 carried, from the V1 tag or the Mantis board. Otherwise the job file and any
impedance figure KiCad quotes describe a board JLC won't build. The README's "RF lines over In1"
should read "B.Cu over In6".

### 4. Decide the GNSS shield and how the patch pin gets soldered (build plan, no copper change)

**The pin.** AE1's feed pin comes through to the back at (83.72, 124.29), 2.2 mm inside the shield
wall. The BMI-S-102 lid is solid there: the nearest vent hole is 1.3 mm off and shows only 17-19 % of
the 2.2 mm pad.

**What is sealed in.** Once the lid is baked on, the pin's joint can't be made or inspected. The
following are all sealed in with it:
- the unfitted tuning pads C157/C158;
- the 915 MHz trap (L14, C159, C160);
- the L5 amplifier's input parts;
- R14.

The pin itself fits under the lid with 2.2 mm or more to spare. Taoglas rev I says only "pin and
adhesive mount", so hand soldering is an inference, but no reflow profile is given.

**Two workable plans.** Both use the same land, so the gerbers and stencil are unchanged.
- **Two-piece shield.**
  - Fit the BMI-S-202-F frame in the back bake and hand-solder the pin through the frame window after
    both bakes.
  - Tune, then snap on the BMI-S-202-C cover.
  - The project's model assumes a 1 mm lip. With that lip the window clears the pin pad by only
    0.135 mm, and L14 and C160 sit at the lip edge. Measure the frames on the PX100 carrier V2 board
    before buying.
- **One-piece shield, fitted last.**
  - Paste and tin SH1's land in the back bake but leave the can off.
  - Hand-solder the pin and tune.
  - Then fit the BMI-S-102 with flux, preheat and hot air. Access is lost again afterwards.

**C130's position matters either way.** The supercap lies on the lid (item 16), so the cover can't
come off, and the can can't be reworked, without unsoldering C130 first. If the two-piece plan is
chosen, consider moving C130 off the shield.

---

## Cheap fixes worth making before ordering

### GNSS

**5. Move the L5 amplifier's match inductor to the amplifier pin (L13 to U13).**
- **What is there.** Net-(U13-RFIN), between L13 (9.1 nH) and the ATR2652S pin 1, is 7.15 mm long. The
  carrier's equivalent was 1.49 mm.
- **What it does.** At 1176 MHz that is about 17° of line. It turns the 50 + j66 ohm source the match
  sets up into about 120-130 + j66-78 ohm at the pin.
- **Cost.** With no published noise parameters for the part, the likely cost is 0.1-0.6 dB of L5
  noise figure in the first stage, plus some gain. This is the band that is already short.
- **Fix.** Move L13 to pin 1; C69 (a 100 pF DC block) can stay at the diplexer.
- **Space.** There is no free spot at pin 1 today. It is boxed in by the shield wall pads, U12 pin 6,
  the GND via at (72.5, 129.2) and C70/C71. U13 has to shift about 1 mm down, or those caps move.
- **Leave the others.** The L1 amplifier's line (4.67 mm against the carrier's 3.48 mm, which tracks)
  and the PX100's second L5 input (5.98 mm, after about 19 dB of gain) cost almost nothing.

**6. Update AE1 from the library so inner layers 5 and 6 clear the feed pin.**
- **What is there.** In5 and In6 pour GND 0.9 mm from the feed pin's center, against the 1.5 mm
  radius Taoglas §4.3 asks for on every layer but the bottom. The In6 pour under the 2.2 mm bottom pad
  adds about 0.4-0.5 pF at the feed, inside the match you can't reach (item 4).
- **Fix.** The library footprint already lists In5/In6 in its keep-out; only the board copy is old.
  Update from the library, review the diff, refill and re-run DRC.
- **Afterwards.** Delete the re-appeared F.Fab `${REFERENCE}` texts, per the hidden-labels rule.

**7. Move M_MOSI out of the ring of ground vias around the feed pin.**
- **What is there.** The NAND/LoRa SPI data line runs on In2 1.14 mm from the feed pin's barrel. That
  is just outside the In1/In3 clearance, so neither plane screens it, and it sits inside the 14 GND
  vias at 2.1-2.8 mm.
- **Disagreement.** The checkers split. One estimates the coupled noise at 15 dB or more below
  thermal; the other estimates 0.1-2 dB of noise-floor rise at the antenna node while the bus is
  busy. The coupling is real; its size is uncertain.
- **Cost of fixing it.** About 6 mm of reroute into the 0.7 mm channel at x ≈ 86.2. That cuts the
  coupling by more than 1000 times.
- **Leave alone.** The SPI clock and the TCXO lines are far enough away.

**8. Put V_BACK on +3V3.**
- **What is there.** U5 pin 6 rides V_MCU_SWTCH, so every flight-computer rail drop loses ephemeris
  and RTC.
- **Effect.** That means a cold start (30 s average) instead of a hot start (1 s).
- **Cost of the change.** On +3V3 it draws 13 µA with the main supply off. No back-powering or
  sequencing path exists: U30's quick discharge clamps the rail, and every receiver pull-up is on it.
- **Fix:**
  - Split pin 6 from pin 7.
  - Take it by one via to the In3 +3V3 plane; C46 can move onto it.
  - Reroute pin 5's track to C40 so it clears pin 6.
- **Benefit.** Pad turnaround, not flight safety.
- **Pair it with item 2's RSTN line.** Without it, the only full receiver reset left would be pulling
  the pack and draining the supercap.

### Out computer, sensors and radios

**9. Move the magnetometer away from the steel shield (U3, MMC5983MA).**
- **Where it is.** U3 sits 2.6 mm from SH1's wall, which is 0.2 mm half-hard cold-rolled steel.
- **Why it matters.** That is enough soft-iron distortion that the firmware's hard-iron-only
  calibration likely can't remove it: expect a rejected calibration, or tens of degrees of heading
  error on the pad.
- **Where it was.** In HEAD it sat near the IMU, about 13 mm from the shield.
- **Fix.** Move it back if the routed layout allows. Either way, add a full ellipsoid (soft-iron) fit
  to the magnetometer calibration; even at 13 mm, hard-iron alone leaves about 4-6° peak to peak.
- **Calibrating.** Calibrate on the fully built board, on battery with USB unplugged, after the
  supercap has charged.
- **CAP capacitor.** C6 is a 10 µF 6.3 V X5R 0402 that keeps about 4 µF at the 3.45 V rail, against
  MEMSIC's "minimum 10 µF". Use the 22 µF 0805 already on the BOM (CL21A226MOQNNNE) if it fits beside
  pin 10.
- **VDD decoupling.** VDD is shorted to CAP inside the part, so the separate 1 µF at VDD is optional.

**10. Tidy the LoRa feed (U16 pin 6 to J8).**
- **What is there.** The impedance is fine (47-54 ohm) and the loss is small. But the run goes down to
  In5 for 8.75 mm, where the nearest plane is In4, V_MCU_SWTCH, 0.1 mm away.
- **Effect.** About half the +22 dBm return current rides the switched rail that feeds the GNSS front
  end and sensors. The estimate is 10-15 mV rms of 915 MHz on the rail during transmit.
- **Route it as V1 did:**
  - Run it on B.Cu at the RF class width.
  - First move R142 off y ≈ 107.
  - Add ground vias along it, and put the net in the RF class.
- **Smallest alternative:** a GND strip on In4 over the run plus a GND via beside each RF via.
- **SMA pad.** J8's center pad sits 0.1 mm over solid In6 GND: about 2 pF extra and 0.1-0.5 dB of
  mismatch. Put a keep-out on In6 only under the pad. Voiding In5 as well would leave V_MCU_SWTCH as
  its reference.
- **SMA position.** J8 sits 1.54 mm back from the edge relative to its flange line, so the
  connector's pin lies over B.Cu GND for 1.2 mm. Move it out about 1.3 mm, or clear B.Cu under the pin.
- **Missing inductor.** Ebyte's reference (§5.1) has a 330 nH shunt on ANT that "the user should add".
  It is an ESD part with no RF effect. Tinker-Base fits LQW18ANR33G00D.

**11. Rotate the out computer's 32 kHz crystal (Y1).**
- **What is there.** Its pins are crossed relative to U15 pins 21/22, so XTAL_32K_N takes two vias and
  6.5 mm of In5. Y1 also sits partly over the buck inductor L6, with a solid In1 plane between.
- **Fix.** Rotate Y1 180° and swap C19 with C21. Both lines then run on F.Cu with no vias.
- **Check.** On the first board, the out computer's log should say it is using the external crystal.

**12. Add missing pin-1 marks, and thicken the thin ones.** With every label hidden, these are the only
orientation cue during hand placement.
- **No pin-1 mark on silk:** U1, U23, U16, U15 and U32.
- **U1 fits at 180°.** The FSUSB63 has four identical corner brackets and a land that fits either way
  round. Rotated, pin 1 D+ lands on FC_D- and GND lands on OC_D-.
- **Marks to add:**
  - a pin-1 dot of at least 0.2 mm stroke on those five parts;
  - "G T R" beside J9;
  - "+/-" beside J3.
- **Marks to redraw.** These are below JLC's 0.15 mm minimum silk width: Y3, FL1-FL3, the LEDs and U5.
  Redraw them at 0.15 mm or more.

**13. Fix the vertical USB-C footprint's drill features (J6), or expect a JLC engineering query.**
- **Status.** This SHOU HAN part has never been built on any of the owner's boards.
- **Post hole.** The oval non-plated post hole is 0.72 x 0.52 mm, against JLC's 1.0 mm minimum
  non-plated slot. Make it round, sized to the post.
- **Shell slots.** The plated shell slots are 0.85 x 0.60 mm, a 1.4:1 ratio against JLC's 2:1. Check
  the maker's drawing and redraw, or accept the query.
- **First article.** Inspect the shell fillets. Consider an epoxy fillet at the base, since USB gets
  plugged at the pad.

### Power and pyro

**14. Smaller layout items, each a few minutes:**
- **INA230 sense lines (U23, R72).**
  - Neither line is Kelvin. IN+ taps the pour 0.48 mm outside R72. IN- lands on the via in C41's pad,
    which carries about 40 % of the firing current.
  - The part reads about +18 % on firing current and about +10 % on board current, so one firmware
    scale factor can't correct both.
  - The error is safe for the trip (it errs high), but telemetry is off.
  - Route the pair from the facing inner edges of R72's pads. Mind that both B.Cu pours meet inside
    R72's pad gap, and that a new IN- via would join the In5 VBAT_CON pour unless In5 is cleared
    around it.
- **Extra VBB vias in the TPS2HC16 pads (U8, U6, U7, U10).**
  - Each has one via in its VBB pad. TI's land pattern, and the project footprint's own description,
    call for three, at x = -0.7, 0, +0.7 mm.
  - With filled vias already ordered, adding them costs nothing.
  - The gain is thermal margin, not safety: one via carries even a fault pulse with under 1 K of
    barrel rise. The checkers split between should-fix and note.
  - U8 benefits most: it has no other VBAT_CON via within 4.6 mm. On U6 and U10, the x = -0.7
    positions collide with B.Cu FIRE tracks.
- **U18 input loop (TPS62152).** Its VOS link to C53 cuts the B.Cu ground, so C43's ground pad is on
  its own island. Hop that 1 mm onto In5, where V_BUCK already runs, and refill so the pour bridges
  C43.2 to PGND.
- **USB supply into U21 (TPS2121).** Add 1 µF (CL05A105KO5NNNC) at U21 pin 7, which has no local
  capacitor. Widening the 0.2 mm VBUS route is optional: about 0.2 ohm, 0.13 V at the worst-case
  0.65 A.
- **Fiducials.** Refill and save before plotting, or plot with `--check-zones`. As saved, the four
  fiducials merge into GND copper.
- **BOM fields:**
  - U11 is 2 Gbit by Value and 1 Gbit by MPN, Datasheet and symbol name. Firmware sizes from the chip
    ID, so it only matters when buying.
  - U19 carries SnapEDA leftovers for a different part (MP TPS259824LNRGET, QFN-24, "24 V, 2.7 mohm,
    15 A").
  - U47 should read 2.5 V / 100 mA charge, 3.0 V boost.
  - R134 should read 3.0 V.
- **ERC.** Put no-connect flags on the 12 spare out-computer pins so ERC reads 0 errors. Note on
  GPIO45 that it must stay floating or pulled low.
- **Mounting holes.** They no longer form a rectangle (21.95 / 22.03 across, 48.79 / 48.75 down). H1
  and H3 can move to x 72.78 and 94.81 with clear copper around them; leave H2 and H4. Record the
  pattern and the 26.8 mm width in the README. The V1 reflow carrier, sleds and nose-cone plate do not
  fit V2.

---

## The two newest circuits in detail

### PX100 receiver: schematic matches SkyTraq's reference

Every point checked matches or exceeds the reference:
- one 1 µF per RF-core pin;
- 2.2 µF on the TCXO, backup and core regulators;
- 4.7 µF on V_CORE, plus an extra 2.2 µF on VDD11;
- the pin-14 2.2 µF, which is in SkyTraq's own drawing;
- 5 ohm and 2.2 µF on DCDC_IN;
- SkyTraq's own LQM18PN2R2MFRL;
- the Schottky on DCDC_OUT;
- 10 k / 1 µF on RSTN and 1 k on BOOT_SEL;
- the 32 kHz crystal with no load caps. The ABS07-32.768KHZ-T is 12.5 pF, ±20 ppm, 70 kohm max ESR,
  the same CL as the carrier's SC-32S;
- the final L-matches: 3 pF + 8.2 nH on L1, and 4 pF + 12 nH on L5.

The AT2659S is wired exactly as on the carrier that tracks. The only difference from the reference is
a 100 k RXD pull-up against 10 k, which is harmless.

**Supply.** The receiver draws **120 mA**, not the LC86G's 30-45 mA that power-budget.md still uses.
During supercap hold-up, VDD_MAIN is the TPS61094's 3.0 V ±2 % less 20-40 mV across U30, so 2.90-3.04
V. That is nominally right at the PX100's 2.97 V minimum, the same margin the ESP32-S3s already
accept. The internal 3.0 V TCXO regulator drops out, but the TCXO itself runs to 1.7 V. There is no
clean hardware fix, since a 3.3 V OSEL breaks the charge guarantee (#999). Test it on the first
article (below).

### PX100 front end and patch: layout

What checks out:
- **RF lines.** All on B.Cu at one width (0.18 mm on a 0.20 mm gap), over solid In6 except at the feed
  pin. That is about 50 ohm on JLC's 2116 stack with copper and mask counted.
- **Decoupling.** The caps sit 1.1-1.5 mm from their pins.
- **Exposed pad.** U5's has nine vias.
- **Shield.** 34 stitching vias in the wall pads, never more than 3.8 mm apart.
- **Traces under the wall.** The three traces crossing under it (the RTC lines and L_DI01) pass
  through the 3.0 x 0.5 mm wall notches on Laird's drawing.
- **Patch keep-outs.** Met on F, In1-In4 and B.
- **Under the patch.** Copper on F is about 90 % GND; the rest is via clearances.

Beyond items 4-8, two first-article checks, neither needed for the order:
- **TCXO beside the radio.** Y3 and C55 sit outside the shield, about 1 mm from the LoRa module. The
  output runs 9.8 mm: 8.7 mm buried on In2 between In1 GND and the +3V3 plane, plus 1.1 mm on B.Cu.
  1170 MHz (45 x 26 MHz) is in the L5 band. The carrier had its TCXO inside the shield, about 4 mm
  from pin 4. Compare L5 and L1 C/N0 with telemetry transmitting and silent.
- **Switcher diode next to the patch.** D15 sits on the front, outside the shield, 0.9-1.3 mm from the
  patch's edge. Its loop is no longer than the carrier's, so the only question is in-band pickup from
  the switching edge. Move it beside L8 only if a SOD-882/923-size Schottky fits there; otherwise look
  for spurs on the first board.

### TPS2HC16-Q1 pyro switches: circuit correct

Checked against TI SLVSI29:

| Point | Board | Data sheet |
|---|---|---|
| Channels per switch | One, both outputs on the channel, channel 2 off | Not paralleled, so a fault can't double the current |
| EN2 and SEL | Left on internal pull-downs | EN2 is 150-500 k; SEL is 100-500 k |
| DIAG_EN | Floats low | A solder bridge to its EN1 neighbor can't fire a channel |
| SNS and FLT | Open | Allowed with diagnostics off (SNS leakage ±100 nA) |
| EN drive | 10 k series, 5.11 k GPIO-side pull-down | Matches TI's protection resistor; VIH 1.5 V |
| VBB to GND | 100 nF per switch | TI's CIC |
| RLIM | 23.2 k 1 % | Regulated band; ICL 11.9 A minimum, capped by ICB at about 16-18 A |
| UVLO | — | 2.8-4.0 V against a 6.4-8.4 V pack |
| Fold-back | — | VDET1 is 19 V or more, so the limit never folds back |
| Turn-on | — | 10-70 µs from sleep |

- **Unarmed.** A commanded channel passes 3.8 mA through R73, far below any e-match no-fire current.
- **Pull-down caps.** The 100 nF caps sit 2.4 mm from U6, U7 and U10's VBB pads and 4.35 mm from U8's.
  Moving C153 next to U8 is optional.
- **Optional:**
  - 24.9 k would clear the typical-only 22.5 k "RLIM short" detection threshold by more. 23.2 k −1 %
    is only 2 % above it. Misread, the fallback is a fixed 15 A, which is harmless.
  - A TVS on VBAT_CON, after Q11, not on J3's pin. A reversed pack would forward-bias a unidirectional
    one; use SMF10CA if it must sit at J3. Even an undamped hot-plug ring (16.8 V) is within every
    part's absolute maximum.

**Battery feed.**
- **The copper.** One 0.5 oz In2 strip from the shunt to the switches, about 17-27 mohm. It necks to
  about 1.3 mm at y ≈ 138 between L_CS's via and a GND via, and 67 foreign vias punch it. From R72,
  three vias carry the firing current into the inner layers.
- **Nothing fuses:**
  - A hard short is chopped by the switch to a few amps average and latches off within about 0.5 s.
  - The worst sustained case is about 18 A for the 200 ms fire pulse: about 6 A per shunt via (under
    1 K barrel rise) and tens of kelvin at the neck, once.
  - Normal fires are 6.5-9.6 A for a few ms.
  - J3's 2 A rating is continuous; J3 and its exposure are unchanged from V1.
- **Not needed.** The earlier "10+ vias and an XT30" proposal isn't required.
- **If the board is touched anyway,** move one via out of the y 138 neck and add vias at R72 pad 1.

---

## Before the first flight (firmware and docs, not the board)

- **Consent trip (#1553, #1588).** The out computer still drops consent at 30 A for 250 ms. With these
  switches, one channel held near its limit plus a good channel fired with it plus the board comes to
  about 33 A. One resistive fault can therefore cut the good channel beside it. Land #1588 for the
  Beetle V2 (TR_BOARD_M2) at 38 A. That is still below the INA230's 40.96 A clip, so a failed-short
  switch still trips. Add a static_assert on "faulted + good + board below the limit".
- **README pyro section.** It is stale: "nothing limits the fire current", the 47 k drivers, and
  PYRO_GND on J2 pin 5 (it is pin 1).
- **Hold-up times.** They are about 2.5 s at a realistic 0.40 A (2.0 s at −20 % capacitance, about
  1 s at end of life), about 4 s at 0.30 A, and 0.7-0.9 s at 0.69 A. The documented figure is 5 s at
  300 mA. Hold-up ends in the boost current limit at about 1.3-1.45 V on the cap, not at 0.9 V.
  Update power-budget.md, the README and the hold-up note.
- **Cheapest hold-up margin.** Firmware can shed WiFi and LoRa transmit while VBUCK_OK is low. Keeping
  WiFi transmit and LoRa transmit mutually exclusive also brings the worst-case 3.3 V load back near
  0.4 A.
- **Supply load.** U18 is at about 84 % of its usable current in the worst corner (cap charging, all
  radios on). That is within rating.
- **Sensor SPI.** Keep all sensor drivers on SPI mode 0, and note why in the code. The MMC5983MA's I2C
  side can't be disabled; mode 0 avoids the false-START case. Optionally set the IMU's I2C/I3C disable
  bit at init.
- **Barometer ID.** Its nets are named BMP585; make sure the chip-ID check accepts the BMP581.
- **power-budget.md.** Still lists the LC86G.

## Build plan

**15. Bake the front first, then the back.** The V1 notes bake the back first. The V2 back carries the
heavy parts, which must not hang upside down in the second pass: U16, J6, SH1, J3, J8 and the PX100.
The front carries only light parts. Keep the 80 µm stencil, now justified by FL3.

**16. Fit C130 last, by hand.**
- CDA allows reflow only with a rated profile, and caps the body at 150 °C.
- Form the leads so the can sits 3.8-4.0 mm up, resting on the lid. KiCad's model is already lifted
  4 mm and clears the shield by 0.4 mm; only the footprint description says it rests on the board.
- That puts the back side at 11.6-12 mm. J2 is already 12.8 mm.
- Stake the can with neutral-cure RTV.
- C130 has to come off before any shield work.
- It is the lowest temperature-rated part on the board (70 °C), and it sits over a 0.4 W receiver. At
  2.5 V the life is ample, but measure the lid temperature in the closed bay with the receiver
  tracking and the radio transmitting.
- FABRICATION-NOTES B8 still describes V1's 5 F, 10 x 20 mm cell.

**17. Position file.** Export with `--exclude-dnp`; C157, C158 and C12 are otherwise listed. For a
machine placement file, also drop AE1, J2, C130 and H1-H4. Don't use `--exclude-fp-th`: it also drops
J6, which is reflowed.

## First-article checks

- **GNSS, with the lid still off** (or the two-piece cover off):
  - Inspect the 25 0402s inside the shield for tombstones. Each has one pad on a solid pour and the
    other on a track; 122 such 0402s exist on the board, the same convention as the carrier.
  - Measure VDD_RF, VDD11, V_CORE and V_LNA, and the two amplifier currents. After the lid goes on,
    only DCDC_IN (R6), DCDC_OUT (D15), VDD_TCXO (C55) and RSTN (C62) stay reachable.
  - Tune C157/C158, and check the trap if a VNA is available.
- **GNSS in operation:**
  - Compare L1 and L5 C/N0 against the carrier, with telemetry transmitting and silent. This covers the
    TCXO, D15 and LoRa co-site questions.
  - Check the 32 kHz oscillator starts cold.
  - Pull the pack while the receiver tracks and the radio transmits. Log VDD_MAIN, C/N0, lock, cycle
    slips and any restart through the whole hold-up.
- **Pyro:**
  - Bench-fire one channel into about 0.5-0.7 ohm for 200 ms while another fires into 0.8 ohm. Scope
    VBAT_CON, U19's EN/UVLO and +3V3, and put an IR camera on the back over y 138 and the shunt.
  - Watch for VBAT_CON sag tripping U19's UVLO during a long resistive fault.
- **WAGO pins.** Five 0402s sit 0.43-0.49 mm from J2's hand-soldered pin pads: R10, R12 (V_MCU_SWTCH),
  and R11, R13, R19 (continuity). Before any e-match is connected:
  - inspect them under magnification;
  - meter each J2 pin to V_MCU_SWTCH and to its neighbor;
  - check that J2 seats flat. The nine back-side 0402s at its wire entry clear the housing by
    0.37 mm or more.
- **BLE.** Tune the ESP32 match (L1, R148, C12) on a VNA after item 1. If item 1 doesn't move the
  antenna, at least put two GND vias beside the LNA_IN via (about 1.3 nH unreturned today).
- **Out computer's 40 MHz crystal.** Measure it from a TX tone. Its crystal is rated −30 °C, the only
  part on the board not rated to −40 °C.
- **Magnetometer.** Calibrate on the finished board, on battery. Log the field step when USB plugs in
  and when the supercap starts charging.
- **Power parts.**
  - Measure the V_BUCK to +3V3 drop and U47's temperature at the worst-case load with the cap charging.
  - Look at U18's switch-node ringing.
  - Scope a hot-plug with the flight pack.
- **J9.** Power the board before connecting a programming adapter, or put 1 k in the adapter lead.
  Pads 2 and 3 go straight to the PX100's UART pins and would back-power an unpowered receiver.
- **Edge clearance.**
  - Six pads sit 0.21-0.30 mm from the routed edge (R23, C48, R147, L4, R36, C24). They pass JLC's
    0.2 mm rule by 0.01-0.02 mm.
  - C48's courtyard crosses the edge. It is an 0805 on V_MCU_2S, so move it in 0.3 mm if convenient.
  - Otherwise handle the board by its holes.

## Checked and fine

- **ESP32-S3 basics (both):**
  - strapping pins (GPIO0, 3, 45, 46);
  - CHIP_PU RC;
  - USB 22 ohm resistors 2.2-2.4 mm from the pins;
  - VDDA, VDD3P3_CPU and VDD3P3_RTC caps 1.1-1.8 mm away;
  - the 40 MHz load (12 pF pair, about 8-11 pF total, against a 10 pF crystal).
- **ESP32-S3 notes:**
  - VDD_SPI has 1 µF only; the guide asks for 0.1 µF plus 1 µF.
  - The exposed pads have 4 (flight computer) and 6 (out computer) vias against Espressif's
    four-layer "at least 9". That is adequate on 8 layers with ground 0.1 mm below, and extra filled
    vias cost nothing if routing allows.
- **ESP32 RF match.** The match is the V9's Molex L network, which flew. Espressif's chip-side CLC is
  "preferred", not required. 35 versus 50 ohm costs about 0.1 dB.
- **Other parts:**
  - INA230 pin table, address 0x40 and decoupling.
  - IMU straps and decoupling.
  - NAND WP#/HOLD# pull-ups and its 12 pad vias.
  - The LoRa module's TXEN tied to DIO2, so firmware must enable DIO2 as the RF switch.
- **Power parts:**
  - U18 strapping and soft start.
  - U47's code resistors, charge headroom and land against DSS0012B.
  - U21 limit (2.25 A) and overvoltage (6.35 V).
  - U19 thresholds and U30 enable levels.
  - USB-C CC resistors.
  - Q11's reverse protection.
- **Capacitor ratings.** Every capacitor on an 8.4 V net is rated 16 V, and every 6.3 V part is on a
  net of 3.6 V or less. L6 peaks at 1.25 A against 1.7 A saturation. U30 carries 0.36 A of its 3 A
  rating.
- **Accepted DRC items.** The nine J2 courtyard overlaps: the 0402s are outside the housing and under
  the lever tips about 10 mm up. Add them as DRC exclusions with a comment.
- **ESD.** USB D+/D-/VBUS are covered by CR3, the GNSS feed by D5, and the pyro outputs by the
  switches' 4 kV rating plus the continuity clamp.
- **Flight-computer GPIO3.** VBUCK_OK back-feeds about 28 µA into the unpowered flight-computer rail,
  which U30's discharge holds at about 10 mV. That is harmless.

## Corrections to earlier figures during this review

- **The supercap's 3D model doesn't collide with the shield.** It is already lifted 4 mm.
- **D15's loop is no longer than the carrier's.**
- **The RF lines are about 50 ohm on the real stack.** My first closed-form estimate gave 52-58 ohm;
  the field-solver figures agree with about 50.
- **The AE1 library footprint already has the In5/In6 keep-out.** Only the board copy is old.
- **A measurement-method note.** A zone refill done outside the project uses default clearances and
  overstates pour gaps. Re-take any pour or clearance measurement from a refill inside the project.
