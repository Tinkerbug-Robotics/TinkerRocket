# Rocket computer mini — fabrication and assembly notes

> ## ⚠ PRE-LAYOUT — NOT RELEASABLE
>
> **This board has no `Edge.Cuts` outline, 106 track segments and 31 vias. It is placed, not routed.**
>
> Every `<TBD>` below is a number that cannot exist until layout closes. **Do not send this file
> to a fab house or an assembler with a `<TBD>` still in it.** The `README` warns specifically
> that stale fab notes are the kind of error that reaches a fab house — that warning applies to
> this file first.
>
> What *is* written down here are the decisions already made, recorded now so the layout is done
> against them rather than around them. The GNSS-module items (A3, B1–B6) are design inputs to
> the layout, not just instructions to the assembler — see
> [#960](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/960).

**Paste block A into the bare-board fab's order notes verbatim. Paste block B into the assembler's instructions.**

This file exists for the same reason the rocket computer's and the LoRa board's do: **none of it can
travel in the design files.** Via protection, surface finish, inner-layer copper weight and stencil
thickness are each absent from the gerbers, the `.gbrjob`, the Excellon drill file, IPC-2581 and
ODB++. On this board the stencil is the worst of them — see B1, and [#959](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/959).

Board: **22.55 × 69.62 mm, 8 layer, 1.630 mm stack.**
F / In1 GND / In2 signal / In3 +3V3 / In4 V_MCU_SWTCH / In5 signal / In6 GND / B.

---

## Block A — bare-board fabrication

```
TINKERROCKET ROCKET COMPUTER MINI - FABRICATION NOTES
BOARD 22.55 x 69.62 mm, 8 LAYER, 1.630 mm STACK.

A1. STACKUP: JLCPCB JLC08161H-2116, EIGHT LAYER, 1.630 mm.
    1 oz OUTER / 0.5 oz INNER. BUILD TO THIS TEMPLATE.
    OUTER PREPREG 2116 AT 0.1164 mm (er 4.16), CORES 0.300 mm
    (er 4.41), INNER PREPREG 1080 x2 AT 0.1528 mm (er 3.91),
    ALL Nan Ya NP-155F. THE BUILD IS SYMMETRIC.
    NOTE: JLC PUBLISH 4- AND 6-LAYER IMPEDANCE TEMPLATES ONLY;
    CONFIRM THIS 8-LAYER TEMPLATE AND ITS IMPEDANCE SERVICE
    WITH THE FAB BEFORE ORDERING.

A2. SURFACE FINISH: ENIG. REQUIRED FOR THE 0.5 mm PITCH 24-BALL
    WLCSP FLASH (U13, U33), THE TWO 0.4 mm PITCH QFN-56 (U15,
    U32), THE 0.4 mm X2QFN (U21), THE 0.4 mm 12-LEAD (U1) AND
    THE 0.40 mm PITCH CHIP ANTENNA (U14).
    DO NOT SUBSTITUTE HASL.

A3. VIAS: FILLED WITH NON-CONDUCTIVE EPOXY AND PLATED OVER
    (CAPPED), IPC-4761 TYPE VII.
    *** VIA-IN-PAD IS MANDATORY ON THIS BOARD, NOT OPTIONAL. ***
    THE GNSS MODULE (U5) CARRIES A VIA IN EVERY GROUND PAD THAT
    HAS ONE AS A DELIBERATE HEAT PATH INTO A BLIND JOINT PLANE.
    (LAYOUT NOTE, NOT A FAB INSTRUCTION: PADS 18, 30, 33 AND 34
    HAVE NO VIA TODAY - EITHER ADD THEM OR ACCEPT THE REDUCED
    HEAT PATH BEFORE THIS GOES OUT.) AN
    UNPLUGGED BARREL DRAINS THE JOINT IT SITS IN, AND THAT JOINT
    CANNOT BE INSPECTED OR REWORKED. IF THIS LINE IS NOT ON THE
    QUOTE, THE BOARD IS NOT BUILDABLE.
    ALL OTHER VIAS TENTED BOTH SIDES.

A4. CONTROLLED IMPEDANCE: 50 OHM SINGLE-ENDED ON TWO FEEDS -
    THE 2.4 GHz CHIP ANTENNA FEED (U14) AND THE 900 MHz RADIO
    FEED (U16 TO J8).
    THE 900 MHz FEED IS ON **B.Cu REFERENCED TO In6.Cu** - NOT
    F.Cu/In1.Cu AS THIS NOTE PREVIOUSLY SAID.
    ON THE A1 STACK (0.1164 mm 2116, er 4.16) A COPLANAR
    WAVEGUIDE WITH GROUND MAKES 50 OHM AT **0.15 mm TRACK WITH
    0.19 mm GAPS**; THE 0.127 mm GAP CURRENTLY DRAWN WOULD NEED
    A 0.123 mm TRACK. AS DRAWN TODAY (0.18 mm / 0.127 mm) IT IS
    ABOUT 42 OHM - WAS 38 OHM ON THE OLD STACK, SO THE NEW
    TEMPLATE MOVES IT TOWARDS TARGET.
    THE FEED IS ONLY ~6.3 mm LONG, UNDER 15 DEGREES ELECTRICAL
    AT 915 MHz, SO THE MISMATCH COSTS UNDER 0.02 dB. IMPEDANCE
    CONTROL ON THIS NET IS OPTIONAL - IF THE FAB CHARGES FOR IT,
    DROPPING IT IS DEFENSIBLE. RE-DERIVE IF THE FEED IS EVER
    LENGTHENED.
    THE 2.4 GHz FEED'S LAYER IS SET AT PLACEMENT (U14 IS NOT YET
    PLACED); ABRACON ASKS FOR A CPWG KEPT AS SHORT AS POSSIBLE.

A5. MIN ANNULAR RING 0.05 mm.
    MIN VIA 0.40 mm PAD ON 0.30 mm DRILL.

A6. MIN TRACK AND SPACING 0.09 mm.
    HOLE TO HOLE 0.25 mm BETWEEN DIFFERENT NETS,
    MEASURED HOLE EDGE TO HOLE EDGE (WALL OF MATERIAL
    BETWEEN THE TWO DRILLED BARRELS), NOT CENTRE TO
    CENTRE. ON THE 0.30 mm DRILLS USED THROUGHOUT THIS
    IS 0.55 mm CENTRE TO CENTRE.

A7. MIXED TECHNOLOGY - ONE THROUGH-HOLE PART (C130) SHARES THE
    BOARD WITH 0.306 mm PITCH SMD.
```

---

## Block B — assembly

```
TINKERROCKET ROCKET COMPUTER MINI - ASSEMBLY NOTES

B1. *** STENCIL FOIL 0.08 mm (3 mil), FLAT - NO STEP. ***
    LASER CUT, ELECTROPOLISHED AND NANO-COATED.
    THE STEP THIS NOTE USED TO CALL FOR WAS SET BY A TDK CHIP
    ANTENNA THAT IS NO LONGER FITTED. THE FINEST APERTURE ON THE
    BOARD IS NOW THE 24-BALL WLCSP FLASH (U13, U33): 0.254 mm
    ROUND PADS, AREA RATIO 0.64 AT A 0.10 mm FOIL - BELOW THE
    0.66 FLOOR - AND 0.79 AT 0.08 mm. EVERY OTHER APERTURE,
    INCLUDING THE 0.30 x 0.30 mm ANTENNA PADS (U14, RATIO 0.94
    AT 0.08 mm), CLEARS COMFORTABLY AT THAT FOIL, SO ONE FLAT
    0.08 mm FOIL SERVES BOTH SIDES.
    (SUPERSEDED TEXT, FOR REFERENCE: FOUR 0.20 x 0.30 mm
    APERTURES ON 0.306 mm CENTRES, AREA RATIO 0.60 AT A 0.10 mm
    FOIL - BELOW THE 0.66 FLOOR. DO NOT SUPPLY A 0.10 mm OR
    THICKER FLAT FOIL FOR THIS SIDE.
    STEP UP OVER U5 (GNSS MODULE) ONLY. STEP HEIGHT <TBD>.
    B SIDE: FLAT FOIL 0.11 mm OR THINNER, SET BY U21.

B2. *** U5 - ALL 36 JOINTS ARE BLIND. ***
    EVERY PAD LIES UNDER THE 18.4 x 18.4 mm MODULE BODY. NO
    JOINT ON THIS PART FORMS AN EXTERNAL FILLET; NONE CAN BE
    INSPECTED VISUALLY OR BY AOI. 26 OF THE 36 PADS ARE GROUND,
    SO A COLD GROUND JOINT IS ALSO ELECTRICALLY SILENT ON A
    NORMAL CONTINUITY CHECK.
    THE PREVIOUS GENERATION OF THIS PART ON THE GNSS
    DAUGHTERBOARD FAILED EXACTLY THIS WAY - DEAD ON ARRIVAL,
    DIAGNOSED ONLY BY ELIMINATION.

B3. U5 GOES ON THE SECOND (TOP) REFLOW PASS. IT MUST NOT BE
    PLACED ON THE FIRST, INVERTED PASS: 57.1 mm2 OF LAND AT
    30 g/in2 IS A 2.7 g BUDGET AND THE MODULE IS HEAVIER.

B4. PROFILE TO U5'S JOINTS, NOT TO THE BOARD TOP.
    THERMOCOUPLE THE WITNESS PADS (B5) AND VERIFY TIME ABOVE
    217 C THERE. EXTEND THE SOAK SO THE CERAMIC PATCH ANTENNA
    EQUALISES BEFORE THE RAMP - THE JOINT PLANE UNDER THIS
    MODULE LAGS THE REST OF THE BOARD.

B5. *** U5 INTERIOR JOINTS ARE NOT ELECTRICALLY TESTABLE. ***
    !!! THE WITNESS-PAD SCHEME DESCRIBED BELOW WAS NEVER BUILT.
    THE BOARD HAS NO TEST POINTS, AND EVERY U5 GROUND PAD IS
    TIED TO THE POUR THROUGH ITS OWN VIA, SO NO PAIR OF PADS
    ISOLATES A JOINT. X-RAY IS THE ONLY ACCEPTANCE METHOD FOR
    THE 24 BLIND INTERIOR JOINTS UNTIL WITNESS PADS ARE ACTUALLY
    DRAWN. DECIDE WHICH BEFORE COMMITTING TO A BATCH. !!!
    (UNBUILT SCHEME, RETAINED SO IT CAN BE IMPLEMENTED:)
    FIVE OF U5'S GROUND PADS ARE ISOLATED FROM THE POUR AND
    ROUTED OUT TO TEST PADS AT <TBD>. CONTINUITY BETWEEN ANY
    PAIR RUNS THROUGH ONE JOINT, THE MODULE'S INTERNAL GROUND,
    AND ANOTHER JOINT. MEASURE ALL PAIRS. A PAD THAT READS OPEN
    AGAINST ALL THE OTHERS IS A COLD JOINT ON THAT PAD.
    THIS IS THE ONLY TEST THAT SEES THESE JOINTS.
    FOUR PADS ARE THE LGA CORNERS AND ONE IS THE CENTRE - IF
    THE CORNERS OPEN THE MODULE IS WARPING; IF THE CENTRE
    OPENS THE PROFILE IS SHORT. REPORT WHICH.

B6. *** NO REWORK PATH. THIS BOARD IS BUILT FOR THE OVEN. ***
    U5 IS NOT TO BE HAND-PLACED OR HOT-AIR ATTACHED, AND A
    FAILED U5 IS SCRAP, NOT A REPAIR. BUILD THE FIRST ARTICLES
    AS A QUALIFICATION RUN AND CLEAR B5 BEFORE COMMITTING TO A
    BATCH - NOTE B5 CURRENTLY MEANS X-RAY, NOT A CONTINUITY
    TEST, BECAUSE THE WITNESS PADS DO NOT EXIST.

B7a. J8 (SMA EDGE CONNECTOR) IS HAND SOLDERED AFTER BOTH
    REFLOW PASSES. IT STRADDLES THE BOARD EDGE AND ITS TWO
    TOP-SIDE PADS CARRY PASTE APERTURES IN THE FOOTPRINT - DO
    NOT PASTE THEM. THE TOP STENCIL CANNOT SEAT OVER A FITTED
    CONNECTOR, AND PASTE REFLOWED ONTO EMPTY PADS HAS TO BE
    CLEANED OFF BEFORE THE PART CAN GO ON.

B7. BOTTOM-SIDE PARTS AT OR OVER THE INVERTED-PASS WEIGHT
    LIMIT (30 g/in2 OF LAND):
      U16  E220-900MM22S RADIO  12.0 mm2 ->  0.56 g
      J8   SMA EDGE             26.2 mm2 ->  1.22 g
      J2   TERMINAL BLOCK       64.0 mm2 ->  2.98 g
    THESE NEED ADHESIVE DOTS BEFORE THE FIRST PASS, OR HAND
    ATTACH AFTER THE SECOND.

B8. C130 (5 F 2.7 V RADIAL SUPERCAPACITOR, 10 mm DIA x 20 mm,
    5 mm LEAD PITCH, MOUNTED LYING DOWN) IS THE ONLY THROUGH-
    HOLE PART ON AN OTHERWISE REFLOW BOARD. HAND SOLDER AFTER
    BOTH PASSES. IT IS POLARISED: PAD 1 = V_SCAP (+), PAD 2 =
    GND (-). CONFIRM AGAINST THE SCHEMATIC, NOT THE SILKSCREEN.
    THE CAN LIES OVER U16, J8, U21, U23 AND D7 AND IS BONDED
    DOWN WITH NEUTRAL/ALKOXY-CURE RTV - NEVER ACETOXY-CURE,
    WHICH RELEASES ACETIC ACID ONTO THE COPPER BENEATH IT.
    EVERYTHING UNDER THE CAN IS UNREWORKABLE ONCE BONDED, SO
    COMPLETE ELECTRICAL BRING-UP FIRST.
```

---

## Why each item is here

**A3 — via protection is load-bearing on this board.** On the rocket computer and the LoRa board,
filled-and-capped is a cost adder worth confirming on the quote. Here it is a build gate. `U5`'s
ground pads each carry a via *on purpose*: bare FR4 under the module's courtyard conducts about
69 mW/K vertically, and one 0.30 mm via with 25 µm plating adds 6.6 mW/K (151 K/W), so one per
ground pad takes the vertical heat path to roughly 3.9× bare board. That is the mitigation that
replaces the bottom-side preheat this board cannot have. An unplugged barrel turns each of those
vias from a heat path into a drain, on a joint nobody can see and nobody can fix.

**A4 — two RF feeds, not one.** The mini carries a 2.4 GHz chip antenna and a 900 MHz radio feed
to an SMA edge connector. The rocket computer's notes list one impedance-controlled feed; copying
that line across would silently drop the 900 MHz one.

**B1 — the stencil conflict is real and it is not about the GNSS module.** The front side carries
the smallest aperture in the repo (now `U13`/`U33`, the 24-ball WLCSP flash — area ratio 0.64 at
0.10 mm, 0.79 at 0.08 mm; the old `U31` chip antenna that set this is no longer fitted) *and* the largest module on
the board. Those two want opposite foils, and placement cannot separate them because `U5` has to be
opposite the other large parts. A step is the only way to serve both from one print. Note the
direction of the surprise: the base foil is *thinner* than the 0.10 mm that earlier paste-coverage
arithmetic assumed, so every coverage percentage converts to less solder than it reads as.

**B2, B5 — the part is blind, so the test structure is the inspection.** Measured off the
footprint, the outermost pads end at ±9.00 mm inside an 18.40 mm body: nothing forms an external
fillet. Combined with 26 of 36 pads being ground, an ordinary electrical check cannot distinguish a
sound module from one hanging on eleven signal joints. The witness pads are the entire incoming
inspection for this part, which is why B5 is worded as mandatory rather than advisory.

**B3, B7 — the second pass fully remelts the first.** Pass 2 takes the whole board above liquidus,
so every bottom-side joint is liquid for 45–90 s with nothing but surface tension holding the part.
`U5` is far too heavy for that, which fixes it to the top side — and that in turn puts `U16`, `J8`
and `J2` on the inverted pass, all three at or under their own weight budget.

**B6 — the recovery path the last board had does not exist here.** The GNSS daughterboard's
SAM-M10Q was recovered by reworking `U1`. Designing for the oven only means a cold joint on the
mini is a scrapped board, which is why the first build is a qualification run rather than a batch.

---

## Still open

- **Board outline.** No `Edge.Cuts` geometry exists. `U5`'s 18.9 mm courtyard is the widest part on
  the board; if the inherited 22.35 mm width is kept it occupies 85% of it, which leaves no room
  for the step stencil's apertureless margin except along the length.
- **Step height (B1).** Deliberately unset. Only the warpage mechanism argues for more volume, and
  which mechanism is at work is what B5 measures. Set it after the first qualification run, not
  before — see [#906](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/906).
- **Witness pad locations (B5).** Five ground pads, four LGA corners plus one centre, routed out to
  an accessible test field. Positions depend on the outline.
- **Impedance track widths (A4).** Both feeds, once the stackup is confirmed against the fab's
  actual template.
- **`U14` land pattern.** The Abracon chip antenna replaced the TDK part; its land is derived from
  the part's terminal drawing and cross-checked against three dimensions on the datasheet's
  recommended-layout figure, but it has not been checked against a vendor CAD library. Its 4.60 ×
  3.50 mm ground clearance must be reproduced as an all-layer keep-out.
- **`U31` land pattern (part no longer fitted).** The 0.20 × 0.30 mm lands have never been checked against TDK's published
  land pattern; the part arrived with the cost-reduction sweep. Tracked in [#959](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/959).
- **The LC86G's LCC land length.** Pads 1–12 stop 0.20 mm inside the body edge, so no castellation
  fillet forms. Whether that matches Quectel's Figure 31 is unverified — no LC86G datasheet is in
  the repo.
