# Tinker-Beetle V2 — fabrication and assembly notes

> **These notes are for the V2 order: written 2026-11-24, re-checked 2026-11-25 against the release
> board (tag `tinker-beetle-v2.0.0`).**
> They replace the V1.0.1 notes (tag `rocket-computer-mini-v1.0.1`, 091601cb), which stay in git
> history at the tag. V1 carried the Quectel LC86G module. V2 carries the SkyTraq PX100 receiver
> with its own L1/L5 front end under a two-piece shield. So V1's LC86G items are not carried over:
> the via-in-pad heat-path rationale, the blind module joints, the one-reflow-cycle limit and the
> witness-pad scheme.
>
> Standing rule: **never send this file to a fab house or an assembler with a `<TBD>` in it.**
> The `README` warns that stale fab notes are the kind of error that reaches a fab house, and that
> warning applies to this file first. After any layout change, re-check the numbers here against
> the board and update the board's Dwgs.User copy of block A to match.

**Paste block A into the bare-board fab's order notes verbatim. Paste block B into the assembler's
instructions.**

This file exists for the same reason the rocket computer's and the LoRa board's do: **none of it
can travel in the design files.** Via protection, surface finish, inner-layer copper weight and
stencil thickness are each absent from the gerbers, the `.gbrjob`, the Excellon drill file,
IPC-2581 and ODB++.

Board: **26.83 × 70.35 mm, 8 layers, 1.630 mm stack.** Square corners, one routed outline.
F / In1 GND / In2 signal / In3 +3V3 / In4 V_MCU_SWTCH / In5 signal / In6 GND / B.

---

## Block A — bare-board fabrication

```
TINKER-BEETLE V2 - FABRICATION NOTES
BOARD 26.83 x 70.35 mm, 8 LAYER, 1.630 mm STACK.

A1. STACKUP: JLCPCB JLC08161H-2116, EIGHT LAYER, 1.630 mm.
    1 oz OUTER / 0.5 oz INNER. BUILD TO THIS TEMPLATE.
    OUTER PREPREG 2116 AT 0.1164 mm (er 4.16), CORES 0.300 mm
    (er 4.41), INNER PREPREG 1080 x2 AT 0.1528 mm (er 3.91),
    ALL Nan Ya NP-155F. THE BUILD IS SYMMETRIC.

A2. SURFACE FINISH: ENIG. REQUIRED FOR THE 0.40 mm PITCH
    PARTS - U5 (QFN-40), U15 AND U32 (QFN-56), U1 (uQFN-12),
    FL3 (SAW) - AND THE 0.50 mm PITCH QFN, WSON AND LGA PARTS
    (U2, U3, U4, U18, U21, U23, U47, FL2, J6).
    DO NOT SUBSTITUTE HASL.

A3. VIAS: ALL 701 VIAS FILLED WITH NON-CONDUCTIVE EPOXY AND
    PLATED OVER (CAPPED), IPC-4761 TYPE VII, BOTH SIDES.
    *** VIA-IN-PAD IS USED THROUGHOUT THIS BOARD. ***
    341 VIAS ARE CENTRED IN SMD PADS AND 58 MORE CUT A PAD
    EDGE, ON BOTH FACES, UNDER PASTE: EXPOSED PADS (U5, U11,
    U15, U18, U19, U23, U30, U32, U47), THE TPS2HC16 VBB PADS,
    THE SHIELD FRAME LAND, J8, J3, IC PINS AND 188 PASSIVE PADS.
    NO VIA IS TO BE LEFT OPEN OR ONLY TENTED. IF THIS LINE IS
    NOT ON THE QUOTE, THE BOARD IS NOT BUILDABLE.

A4. CONTROLLED IMPEDANCE: NOT ORDERED. NO COUPON.
    THE RF LINES ARE DRAWN ON THE A1 TEMPLATE, ARE SHORT, AND
    THEIR MATCHES ARE TUNED ON THE BENCH:
    - GNSS L1/L5: 0.18 mm TRACK, 0.20 mm GAPS, B.Cu OVER
      In6.Cu GND, ABOUT 50 OHM. LONGEST NET 6.0 mm.
    - 2.4 GHz BLE (U15 TO U14): 0.18 mm, 7.8 mm IN ALL.
    - 900 MHz LORA (U16 TO J8): 0.18 mm ON B.Cu, 12.0 mm.
    DO NOT ADJUST TRACK WIDTHS FOR IMPEDANCE.

A5. ALL VIAS 0.40 mm PAD ON 0.30 mm DRILL (ASPECT 5.4:1).
    MIN ANNULAR RING 0.05 mm.

A6. MIN TRACK 0.10 mm. MIN SPACING 0.10 mm.
    HOLE TO HOLE 0.25 mm BETWEEN DIFFERENT NETS, MEASURED
    HOLE EDGE TO HOLE EDGE, NOT CENTRE TO CENTRE.
    COPPER TO BOARD EDGE 0.20 mm: ROUTE THE OUTLINE, DO NOT
    V-SCORE.

A7. MIXED TECHNOLOGY. PLATED HOLES: AE1 (1 x 1.00 mm),
    C130 (2 x 0.90 mm), J2 (10 x 1.20 mm), H1-H4 (4 x 2.20 mm,
    NO NET). PLATED SLOTS: J6 (4 x 1.20 x 0.60 mm).
    NON-PLATED: J6 (0.60 mm AND 0.80 mm ROUND).
    FINEST SMD PITCH 0.40 mm (SEE A2).

A8. SOLDER MASK: U15 AND U32 HAVE 0.07 mm MASK WEBS BETWEEN
    PADS. IF THAT IS BELOW YOUR MINIMUM, REMOVE THE WEBS
    (GANG RELIEF). DO NOT RESIZE ANY PAD OPENING.
```

---

## Block B — assembly

```
TINKER-BEETLE V2 - ASSEMBLY NOTES

B1. *** STENCIL FOIL 0.08 mm, FLAT, BOTH FACES - NO STEP. ***
    LASER CUT, ELECTROPOLISHED AND NANO-COATED.
    THE BACK NEEDS 0.08 mm: FL3 (L5 SAW FILTER, 0.20 x 0.25 mm
    LANDS) HAS AREA RATIO 0.69 AT 0.08 mm AND 0.56 AT 0.10 mm.
    *** EVERY EXPOSED PAD ON THIS BOARD IS PASTED. ***
    PRINT THE STENCILS AS PLOTTED. DO NOT MASK ANY EXPOSED-PAD
    APERTURE AND DO NOT ADD PASTE BY HAND TO ONE:
      PART      SIDE  PAD mm2  APERTURES  COVERAGE
      U15         F    16.8        9        55 %   PROCESSOR
      U32         F    16.8        9        55 %   PROCESSOR
      U11         F    15.0        6        76 %   NAND
      U9          F     6.3        1       100 %   PYRO FET
      U6-U8,U10   F     2.0        5        82 %   PYRO SWITCH VBB
      U5          B    11.6        4        73 %   GNSS RECEIVER
      U19         B     6.9        4        67 %   EFUSE
      Q11         B     3.7        4        65 %   PACK FET
      U23         B     2.9        9        40 %   PACK MONITOR
      U18         B     2.8        1        41 %   BUCK
      U47         B     2.7        2        84 %   HOLD-UP CONV
      U30         B     1.6        2        87 %   FC RAIL SWITCH
    MOST OF THESE APERTURES ARE PASTE-ONLY PADS OR PASTE
    POLYGONS, NOT THE COPPER PAD'S OWN LAYER LIST. CHECK BOTH
    F.Paste AND B.Paste.
    NO PASTE BY DESIGN: J9 POGO PADS, FID1-FID4, THE H1-H4
    RINGS, AND THE HAND-FITTED PARTS AE1, C130 AND J2.
    J8: MASK ALL FIVE OF ITS APERTURES (B6).

B2. *** TWO PASSES: FRONT FIRST, THEN BACK. ***
    THE HEAVY PARTS ARE ON THE BACK - U16, J6, J3, THE SH1
    FRAME, L11, U5 - AND RIDE RIGHT WAY UP IN THE SECOND PASS.
    FRONT PARTS HANG INVERTED IN THE SECOND PASS; THE WORST
    (D15) IS UNDER A QUARTER OF THE 30 g/in2 BUDGET, SO NO
    ADHESIVE IS NEEDED. FRONT PARTS SEE TWO REFLOWS; U5 SEES
    ONE.

B3. PROFILE WITHIN U5'S LIMITS (PX100-5C-01A, J-STD-020
    LEAD-FREE):
      RAMP-UP                 3 C/s MAX
      PREHEAT 150-200 C       60-80 s
      TIME ABOVE 217 C        60-150 s
      PEAK                    260 C MAX
      WITHIN 5 C OF PEAK      20-40 s
      RAMP-DOWN               6 C/s MAX
      25 C TO PEAK            8 min MAX
    SKYTRAQ PUBLISHES NO MSL FOR U5. HANDLE IT AS MSL 3: IF
    ITS FLOOR LIFE IS UNKNOWN, BAKE PER J-STD-033 BEFORE THE
    BACK PASS.

B4. SH1 IS A TWO-PIECE SHIELD. THE BMI-S-202-F FRAME IS
    REFLOWED IN THE BACK PASS. THE BMI-S-202-C COVER (SH2) IS
    NOT FITTED UNTIL B5 AND B7 ARE DONE. WHILE THE COVER IS
    OFF, INSPECT THE PARTS INSIDE THE FRAME (0402 TOMBSTONES,
    U5, U12, U13, FL1-FL3).

B5. AE1 (25 mm GNSS PATCH, FRONT, 18 g) IS FITTED BY HAND
    AFTER BOTH PASSES: PIN AND ADHESIVE MOUNT. ITS 0.8 mm PIN
    GOES THROUGH THE 1.00 mm PLATED HOLE AND IS SOLDERED ON
    THE BACK, TO THE 2.2 mm PAD, THROUGH THE FRAME WINDOW
    WITH THE COVER OFF. THE JOINT IS 2.4 mm INSIDE THE FRAME
    OUTLINE; CHECK THAT THE FRAME'S LIP CLEARS THE PAD BEFORE
    SOLDERING. NO FRONT PART SITS UNDER THE PATCH.

B6. J8 (SMA EDGE CONNECTOR) IS HAND SOLDERED AFTER BOTH
    PASSES. ITS PADS 1, G1 AND G2 ARE ON THE BACK AND G3 AND
    G4 ON THE FRONT, AND THE FOOTPRINT PASTES ALL FIVE: MASK
    THOSE APERTURES. IF THEY ARE PRINTED ANYWAY, G3 AND G4
    COME OUT OF THE FRONT PASS AS BARE SOLDER BUMPS; REFLOW
    THEM WITH THE CONNECTOR. NOT MACHINE PLACED.

B7. TUNING PARTS ARE UNFITTED: C157, C158 (GNSS MATCH,
    INSIDE THE FRAME) AND C12 (BLE MATCH). FIT ONLY WHAT THE
    BENCH TUNE CALLS FOR, THEN SNAP THE COVER ON.

B8. C130 (CDA CXHP2R7205R-TW, 2 F 2.7 V RADIAL SUPERCAP)
    GOES ON LAST, BY HAND, STANDING UPRIGHT IN ITS PRINTED TPU
    SUPPORT (c130-support/). NEVER REFLOW IT: IRON BELOW 350 C
    FOR UNDER 4 s; THE PART IS RATED 70 C.
    POLARISED: PAD 1 (SQUARE) = V_SCAP (+), PAD 2 = GND. THE
    NOTCH IN THE SUPPORT'S RIM SITS OVER PAD 1.
    STAKE THE SUPPORT'S FOOT TO THE BOARD (EAST SIDE, BETWEEN
    U30 AND U2) WITH NEUTRAL/ALKOXY-CURE RTV - NEVER ACETOXY.
    THREAD THE STRAIGHT LEADS THROUGH THE SEAT AND PUSH THE CAN
    DOWN ONTO IT: 2.0 mm OFF THE BACK, OVER C6, U3, CR3 AND THE
    SMALL PARTS ROUND ITS HOLES. SOLDER FROM THE FRONT; C118 IS
    0.7 mm FROM ITS PADS. THEN FILL THE SLEEVE WITH RTV.
    THE BACK IS THEN 14.0 mm TALL (15.5 mm AT THE CAN'S LENGTH
    TOLERANCE; J2 IS 12.8 mm). THE SHIELD COVER COMES OFF WITH
    C130 IN PLACE. CALIBRATE THE MAGNETOMETER (U3, 0.9 mm FROM
    THE CAN) WITH C130 FITTED.

B9. J2 (WAGO 2601-1105) IS HAND SOLDERED FROM THE FRONT AFTER
    BOTH PASSES. FIVE 0402s (R10-R13, R19) SIT 0.43-0.49 mm
    FROM ITS PADS: FINE TIP, THEN INSPECT FOR BRIDGES.

B10. J6 (VERTICAL USB-C) IS PIN-IN-PASTE ON THE BACK PASS:
    ITS FOUR SLOTS ARE PASTED 1:1. EXPECT FILLETS ON THE BACK
    ONLY; INSPECT THE SHELL JOINTS.

B11. PLACEMENT FILE: kicad-cli pcb export pos --exclude-dnp
    --smd-only (277 PARTS PLUS 4 FIDUCIALS), THEN DROP J8
    (B6), LEAVING 276 REFLOWED PARTS. DO NOT USE --exclude-fp-th: IT ALSO DROPS
    J6. THE BOM EXPORT MUST HONOUR DNP (C12 IS DNP).

B12. ORIENTATION. POLARISED ON THE BACK: D5-D14, CR3, C130,
    AND Q11 (ITS EXPOSED PAD IS THE DRAIN; MIRRORED, THE
    REVERSE-POLARITY PROTECTION IS SILENTLY LOST). ON THE
    FRONT: D1-D4, D15, CR4. U14 (BLE CHIP ANTENNA, BACK): MOLEX'S
    FEEDING-PAD CORNER GOES ON PAD 4 - CHECK IT AGAINST THE
    MOLEX DRAWING, NOT THE FOOTPRINT'S NOTCH MARK.
```

---

## Why each item is here

**A1 — the board file now matches the template.** Until 2026-11-24 the KiCad stackup had the wrong
dielectric thicknesses and 35 µm In5/In6 on generic FR4. It was restored from the V1.0.1 board,
which was built to JLC08161H-2116, so the job file and any impedance figure KiCad quotes describe
the board JLC builds.

**A3 — via-in-pad is a property of the whole board, not of one part.** V1 needed it for the LC86G's
ground pads. V2 uses it everywhere: 341 vias sit centred in SMD pads on both faces, under paste,
including nine in the PX100's exposed pad, 24 in the shield frame's land and three in each pyro
switch's VBB pad. An unfilled barrel there wicks the joint it sits in, so the old "all other vias
tented" line no longer describes any via on the board. Measured 2026-11-25 on the release board
(701 vias in all; 58 more cut a pad edge).

**A4 — no impedance service.** The GNSS lines were checked with a field solver at about 50 Ω on
the A1 template. The runs are short (6.0 mm on the longest GNSS net, 7.8 mm at 2.4 GHz, 12.0 mm at 915 MHz), so a
few ohms of deviation costs a fraction of a dB, and the BLE and GNSS matches are tuned on the bench
anyway (B7). A coupon would measure geometry the template already fixes.

**B1 — one foil, set by the back.** FL3's five 0.20 × 0.25 mm lands fail the 0.66 area-ratio floor
at 0.10 mm and pass at 0.08 mm; nothing else on the board fails at 0.10 mm. The front's worst
aperture (the TPS2HC16 VBB end windows, 0.705 at 0.10 mm) would allow a 0.10 mm front foil; 0.08 mm
on both faces keeps one foil thickness, as V1 did. Thinner foil means every coverage percentage in
the table converts to less solder than it reads as.

**B2 — front first, then back.** V1 baked the back first because its 8 g GNSS module had to be on
the top in the second pass. V2's heavy parts are all on the back, so the back goes second. The
front parts that hang inverted are all far under their weight budget (estimated masses from the 3D
models). The patch antenna is the exception at 18 g, which is why it is hand-fitted (B5).

**B4, B5, B7 — the two-piece shield exists for the patch pin.** AE1's pin is soldered on the back,
2.4 mm inside the shield. A one-piece lid would seal that joint, the tuning pads and the L5
amplifier's input before they could be made or checked. The frame goes on in the reflow; the cover
goes on by hand once the pin is soldered and the front end is tuned. The project's 3D model assumes
a 1 mm lip on the frame, which would leave the 2.2 mm pad only about 0.135 mm clear of the window;
Laird does not draw the lip, so measure a real frame first (Still open).

**B8 — C130 stands clear of the shield.** Lying down, its can covered the shield, so the cover
could not come off while C130 was fitted. Standing on its own two holes in the TPU support, it sits
2.0 mm over the low parts round the holes (C6 at 1.33 mm is the tallest) and 0.30 mm outside the
cover's outline. `c130-support/README.md` has the fit check against the board's 3D export, and the
board's C130 3D model shows it standing. The price is height: the back grows from J2's 12.8 mm to
14.0 mm, or 15.5 mm at the can's length tolerance.

---

## Still open

- **Frame window against the patch pin pad (B5).** Measure the BMI-S-202-F frames on the PX100
  carrier V2 before buying. The 0.135 mm figure comes from the 3D model's assumed 1 mm lip.
- **J8 paste apertures (B6).** The shared `Footprints:CON-SMA-EDGE-S` footprint pastes all five
  pads, two of them on the face opposite the connector. Strip them in the library or give this
  board its own variant, then drop the masking instruction.
- **U14's orientation mark (B12).** The footprint's Fab notch is on the pads 1/4 face, while
  Molex's drawing appears to put the part's notch on the opposite face. Add a back-silk mark at
  pad 4 (design review item 12).
- **U5's moisture sensitivity (B3).** Not in SkyTraq's data sheet; ask them, and replace the MSL 3
  assumption with the real figure.
- **Board copy.** Block A is also on the board's Dwgs.User layer. Keep the two identical.
