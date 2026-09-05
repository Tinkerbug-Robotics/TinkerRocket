# Rocket computer mini — fabrication and assembly notes

> ## Board state — routed, placed, fills current, ready to plot
>
> **22.55 × 69.62 mm, 8 layers, 1.630 mm. Fully routed: 0 unconnected items, 2,194 track
> segments, 596 vias, 221 fitted parts against a 221-designator `bom.csv`. 18 DRC items remain:
> 14 silkscreen cosmetics, the two accepted fiducial-in-courtyard overlaps (#1008) and two
> library-copy warnings — none affects fabrication. One schematic-parity item, also accepted:
> `U9`'s symbol carries a pin 9 that its footprint does not, on the same `PYRO_GND` net as pads
> 5–8.** Zone fills refilled and verified current; a stale fill is exactly how a 3V3–GND short
> reached a gerber on this project once before.
>
> **Fixed immediately before this plot:** `C152`'s only tie to `VBUCK_OK` was a 14 µm corner
> overlap with a via — connected as far as KiCad and DRC were concerned, and opened by less
> etch variation than a normal process holds. It now has a 0.10 mm stub. And the file's cached
> board thickness read 1.5765 mm while the stackup summed to 1.6301, so the `.gbrjob` was
> telling the fab a different thickness from A1 below; the cache now agrees with the stackup.
>
> Standing rule: **never send this file to a fab house or an assembler with a `<TBD>` in it.**
> There are none as of 2026-09-04; if one reappears, it is a number that does not exist yet. The `README` warns specifically
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

`U5` is the **LC86G (LA)** — 18.4 × 18.4 × 7.05 mm, 8.0 g. The 18.4 mm is the **ceramic patch
antenna**; the module PCB under it is 16 mm (+0.3/−0.15), and pads 1–12 are castellated flush with
*that* edge. Every claim here is against *LC86G Series Hardware Design v1.5* (Quectel, 2025-10-09):
pin dispositions from Table 6, the reflow limits in B4 from Table 28, the land pattern from
Figure 26 (§5.3), the module geometry from Figure 28 (§7.1), and the keepout and ground-plane rules
from §5.1.4 and §5.3. The datasheet is vendored at
`hardware/datasheets/Quectel_LC86G_Series_Hardware_Design_V1.5.pdf` — use it rather than a
web copy. **Figure and section numbers moved between revisions** and v1.1 does not dimension the
antenna offset at all, which makes its Figure 33 read as though the pads are inset 1.2 mm inside
the body. v1.1 had the land pattern as Figure 31 and the design guide as §5.2.4. Quote the
revision when citing.

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
    (LAYOUT NOTE, NOT A FAB INSTRUCTION: PADS 32 AND 33 HAVE NO
    VIA TODAY - MEASURED 2026-09-04. PADS 18, 30 AND 34 DO, AND
    THIS NOTE NAMED THEM IN ERROR UNTIL NOW. PAD 32 IS A CORNER
    OF THE ARRAY, SO IT IS THE MORE USEFUL OF THE TWO TO STITCH.
    ACCEPTED AS-IS FOR THIS RUN.) AN
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
    THE 2.4 GHz FEED IS PLACED AND ROUTED: U14 SITS AT THE BOARD
    EDGE AND ITS FEED IS 2.99 mm OF 0.18 mm TRACK ON F.Cu
    REFERENCED TO In1.Cu, COPLANAR WITH THE F.Cu GROUND POUR.
    AT 2.4 GHz THAT LENGTH IS UNDER 15 DEGREES ELECTRICAL, SO
    IMPEDANCE CONTROL ON THIS NET IS OPTIONAL TOO - QUOTE THE
    IMPEDANCE SERVICE AGAINST THE 900 MHz FEED IF IT IS CHARGED
    FOR SEPARATELY.

A5. MIN ANNULAR RING 0.05 mm.
    MIN VIA 0.40 mm PAD ON 0.30 mm DRILL.

A6. MIN TRACK AND SPACING 0.09 mm.
    HOLE TO HOLE 0.25 mm BETWEEN DIFFERENT NETS,
    MEASURED HOLE EDGE TO HOLE EDGE (WALL OF MATERIAL
    BETWEEN THE TWO DRILLED BARRELS), NOT CENTRE TO
    CENTRE. ON THE 0.30 mm DRILLS USED THROUGHOUT THIS
    IS 0.55 mm CENTRE TO CENTRE.

A7. MIXED TECHNOLOGY - ONE THROUGH-HOLE PART (C130) SHARES THE
    BOARD WITH 0.40 mm PITCH SMD, WHICH IS THE FINEST PITCH ON
    IT (U1, U14, U15, U32 - MEASURED, AND AGREES WITH A2).
    NOTE: THE ON-BOARD Dwgs.User COPY OF THIS NOTE STILL READS
    0.30 mm AND OVERSTATES THE PROCESS CLASS; THE FAB SHOULD
    TAKE 0.40 mm FROM THIS FILE.
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

B2. *** U5 - THE 24 INTERIOR JOINTS ARE BLIND. THE 12
    PERIMETER JOINTS ARE CASTELLATED - INSPECT THEM. ***
    18.4 mm IS THE CERAMIC PATCH ANTENNA. THE MODULE PCB IS
    16 mm (+0.3/-0.15) AND PADS 1-12 ARE FLUSH WITH ITS EDGE
    WITH CASTELLATION NOTCHES. THOSE 12 FORM EXTERNAL,
    WETTABLE FILLETS AT +/-8.0 mm AND CARRY EVERY SIGNAL ON
    THE PART. INSPECT THEM: THE ANTENNA OVERHANGS THE PCB BY
    1.2 mm PER SIDE, SO USE AN ANGLED VIEW (ROUGHLY 30-40 DEG
    OFF THE BOARD), NOT TOP-DOWN AOI.
    THE 24 INTERIOR ARRAY JOINTS LIE UNDER THE MODULE PCB AND
    CANNOT BE SEEN BY ANY METHOD - B5 IS THEIR ONLY TEST.
    26 OF THE 36 PADS ARE GROUND,
    SO A COLD GROUND JOINT IS ALSO ELECTRICALLY SILENT ON A
    NORMAL CONTINUITY CHECK.
    THE PREVIOUS GENERATION OF THIS PART ON THE GNSS
    DAUGHTERBOARD FAILED EXACTLY THIS WAY - DEAD ON ARRIVAL,
    DIAGNOSED ONLY BY ELIMINATION.

B3. *** U5 GETS ONE REFLOW CYCLE. THAT IS THE DATASHEET
    LIMIT, NOT A PREFERENCE. ***
    IT GOES ON THE SECOND (TOP) PASS AND MUST NOT BE PLACED
    ON THE FIRST, INVERTED PASS: 57.1 mm2 OF LAND AT 30 g/in2
    IS A 2.7 g BUDGET AND THE MODULE WEIGHS 8.0 g.
    QUECTEL RATES MAX. REFLOW CYCLE = 1 AND REQUIRES THE
    MODULE MOUNTED ONLY AFTER THE OTHER SIDE IS REFLOWED.
    THE BOARD MUST NOT SEE A THIRD PASS ONCE U5 IS ON IT,
    AND U5 IS NOT TO BE REFLOWED TWICE FOR ANY REASON.

B4. PROFILE TO U5'S JOINTS, NOT TO THE BOARD TOP.
    THERMOCOUPLE THE WITNESS PADS (B5) AND HOLD QUECTEL'S
    LIMITS *AT THOSE PADS*:
      RAMP-TO-SOAK SLOPE      0-3 C/s
      SOAK 150 C TO 200 C     70-120 s
      TIME ABOVE 217 C        40-70 s
      PEAK TEMPERATURE        235-246 C
      ABSOLUTE MAX PEAK       246 C
      COOL-DOWN SLOPE         -3-0 C/s
    THE DATASHEET STATES THESE ARE SOLDER-JOINT TEMPERATURES
    AND THAT *BOTH THE HOTTEST AND THE COLDEST* JOINT ON THE
    BOARD MUST MEET THEM. 246 C IS THEREFORE A CEILING ON THE
    WHOLE BOARD, NOT JUST ON U5.
    EXTEND THE SOAK SO THE PATCH ANTENNA EQUALISES BEFORE THE
    RAMP. THE 8.0 g MODULE RUNS ROUGHLY 20 K BEHIND THE BOARD
    ON A 1 C/s RAMP, SO THE SOAK IS WHAT CLOSES THAT GAP - A
    HOTTER PEAK IS NOT AVAILABLE AS A SUBSTITUTE.

B5. *** U5 INTERIOR JOINTS ARE NOT ELECTRICALLY TESTABLE. ***
    !!! THE WITNESS-PAD SCHEME DESCRIBED BELOW WAS NEVER BUILT.
    THE BOARD HAS NO TEST POINTS, AND EVERY U5 GROUND PAD IS
    TIED TO THE POUR THROUGH ITS OWN VIA, SO NO PAIR OF PADS
    ISOLATES A JOINT. X-RAY IS THE ONLY ACCEPTANCE METHOD FOR
    THE 24 BLIND INTERIOR JOINTS UNTIL WITNESS PADS ARE ACTUALLY
    DRAWN. DECIDE WHICH BEFORE COMMITTING TO A BATCH. !!!
    (UNBUILT SCHEME, RETAINED SO IT CAN BE IMPLEMENTED:)
    FIVE OF U5'S GROUND PADS ARE ISOLATED FROM THE POUR AND
    ROUTED OUT TO TEST PADS (NOT YET DRAWN). CONTINUITY BETWEEN ANY
    PAIR RUNS THROUGH ONE JOINT, THE MODULE'S INTERNAL GROUND,
    AND ANOTHER JOINT. MEASURE ALL PAIRS. A PAD THAT READS OPEN
    AGAINST ALL THE OTHERS IS A COLD JOINT ON THAT PAD.
    THIS IS THE ONLY TEST THAT SEES THESE JOINTS.
    FOUR PADS ARE THE ARRAY CORNERS (13, 17, 32, 36) AND ONE
    IS THE NEAREST-TO-CENTRE PAD - USE 20 OR 29. THERE IS NO
    CENTRE PAD: THE SIX MIDDLE POSITIONS ARE THE ANTENNA FEED
    KEEPOUT. IF THE CORNERS OPEN THE MODULE IS WARPING; IF THE
    CENTRE PAD OPENS THE PROFILE IS SHORT. REPORT WHICH.

B6. *** BUILT FOR THE OVEN. LIMITED REWORK ONLY. ***
    U5 IS NOT TO BE HAND-PLACED OR HOT-AIR ATTACHED, AND IT
    GETS ONE REFLOW CYCLE (B3), SO IT CANNOT BE REMOVED AND
    REFITTED.
    A COLD JOINT ON PADS 1-12 IS REPAIRABLE IN PLACE: THE
    CASTELLATION IS ACCESSIBLE TO A FINE IRON FROM THE SIDE.
    A COLD JOINT ANYWHERE IN THE 24-PAD INTERIOR ARRAY IS NOT
    REACHABLE AND THE BOARD IS SCRAP.
    BUILD THE FIRST ARTICLES AS A QUALIFICATION RUN AND CLEAR
    B5 BEFORE COMMITTING TO A BATCH - NOTE B5 CURRENTLY MEANS
    X-RAY OF THE 24 INTERIOR JOINTS, NOT A CONTINUITY TEST,
    BECAUSE THE WITNESS PADS DO NOT EXIST.

B7a. J8 (SMA EDGE CONNECTOR) IS HAND SOLDERED AFTER BOTH
    REFLOW PASSES. IT STRADDLES THE BOARD EDGE AND ITS TWO
    TOP-SIDE PADS CARRY PASTE APERTURES IN THE FOOTPRINT - DO
    NOT PASTE THEM. THE TOP STENCIL CANNOT SEAT OVER A FITTED
    CONNECTOR, AND PASTE REFLOWED ONTO EMPTY PADS HAS TO BE
    CLEANED OFF BEFORE THE PART CAN GO ON.

B7. BOTTOM-SIDE PARTS AT OR OVER THE INVERTED-PASS WEIGHT
    LIMIT (30 g/in2 OF LAND):
      U16  900 MHz LORA MODULE  12.0 mm2 ->  0.56 g
      J8   SMA EDGE             26.2 mm2 ->  1.22 g
      J2   TERMINAL BLOCK       64.0 mm2 ->  2.98 g
    THESE NEED ADHESIVE DOTS BEFORE THE FIRST PASS, OR HAND
    ATTACH AFTER THE SECOND.

B8. C130 (5 F 2.7 V RADIAL SUPERCAPACITOR, 10 mm DIA x 20 mm,
    5 mm LEAD PITCH, MOUNTED LYING DOWN) IS THE ONLY THROUGH-
    HOLE PART ON AN OTHERWISE REFLOW BOARD. HAND SOLDER AFTER
    BOTH PASSES. IT IS POLARISED: PAD 1 = V_SCAP (+), PAD 2 =
    GND (-). CONFIRM AGAINST THE SCHEMATIC, NOT THE SILKSCREEN.
    THE CAN LIES OVER 26 PARTS - FID4, J8, U16, U19, U21 AND
    U23 PLUS 20 SMALL PASSIVES INCLUDING C149 - AND IS BONDED
    DOWN WITH NEUTRAL/ALKOXY-CURE RTV - NEVER ACETOXY-CURE,
    WHICH RELEASES ACETIC ACID ONTO THE COPPER BENEATH IT.
    EVERYTHING UNDER THE CAN IS UNREWORKABLE ONCE BONDED, SO
    COMPLETE ELECTRICAL BRING-UP FIRST.
```

---

## Why each item is here

**A3 — via protection is load-bearing on this board.** On the rocket computer and the LoRa board,
filled-and-capped is a cost adder worth confirming on the quote. Here it is a build gate. `U5`'s
ground pads each carry a via *on purpose*.

The plane that matters is **`In1.Cu`, 0.0994 mm below `F.Cu`** on the A1 stackup — not the far side
of the board. Through that prepreg an interior ground pad's own 1.00 mm² of land conducts about
3.0 mW/K, while one 0.30 mm via with 25 µm plating conducts **103 mW/K**: the via is worth roughly
34× the pad it sits in.

Aggregated, the vias are the whole path, not a trim on it. **The module is heated only through its
36 joints** — the pour between the pads faces a ~50 µm air gap and delivers nothing. Summed over
the array that is **2.83 W/K with the vias against 0.16 W/K without: a factor of 17.** An 8.0 g
module (≈6 J/K) hung off 0.16 W/K does not reach liquidus on any profile that respects the 246 °C
ceiling. A3 is not an optimisation.

Where the *remaining* headroom is, is worth recording so the next person does not go hunting in the
wrong place. Above ~2.8 W/K the joint path stops being the limit and **lateral spreading into the
courtyard through 0.0152 mm inner copper** becomes it. Two consequences: every signal escape routed
under the module cuts that feed and costs more than a via buys, and **inner copper weight is the
only large lever left** — 0.5 oz to 1 oz roughly doubles it, which no arrangement of vias can. The
soak in B4 is what actually drains the lag; the vias make draining it possible.

An unplugged barrel turns each of those vias from a heat path into a drain, on a joint nobody can
see and nobody can fix.

**A4 — two RF feeds, not one.** The mini carries a 2.4 GHz chip antenna and a 900 MHz radio feed
to an SMA edge connector. The rocket computer's notes list one impedance-controlled feed; copying
that line across would silently drop the 900 MHz one.

**B1 — the stencil conflict is real and it is not about the GNSS module.** The front side carries
the smallest aperture in the repo (now `U13`/`U33`, the 24-ball WLCSP flash — area ratio 0.64 at
0.10 mm, 0.79 at 0.08 mm; the old `U31` chip antenna that set this is no longer fitted) *and* the largest module on
the board. With the TDK antenna gone, nothing on the board needs a foil thicker than 0.08 mm, and the
WLCSP needs one no thicker — so one flat 0.08 mm foil serves both sides and no step is required. Note the
direction of the surprise: the base foil is *thinner* than the 0.10 mm that earlier paste-coverage
arithmetic assumed, so every coverage percentage converts to less solder than it reads as.

**B5's centre pad had to be renamed.** The clause called for "the centre", but the middle of the
array is Figure 26's Ø2.4 mm feed keepout — the footprint omits six positions there, so no centre
pad exists to route out. Pads 20 and 29 sit 3.3 mm either side of the array centre and are the
closest real candidates; the diagnostic still works, since a short profile shows up first at
whichever ground joint is furthest from the board's lateral heat feed.

**B2, B5 — two-thirds of the part is blind; the third that is not is the third that matters.**
The v1.5 bottom view (Figure 28) makes the geometry explicit, and it is not what "18.4 mm module"
suggests. **18.4 mm is the ceramic patch antenna.** The module PCB underneath is **16 mm
(+0.3/−0.15)**, and pads 1–12 are 1.5 × 1.0 mm flush with its edge, drawn with castellation notches.
The antenna overhangs that PCB — v1.5 added the dimensions that prove it, 0.94 + 16 + 1.46 = 18.4.

So the twelve perimeter joints are **castellated edge joints**. They wet externally, they form a
real fillet at ±8.00 mm, and our 2.5 mm land (±6.50 to ±9.00) already gives them 1.00 mm of exposed
toe beyond the PCB edge. They carry `RXD`, `TXD`, `VCC`, `V_BCKP`, `1PPS` and two grounds — every
signal on the part. The antenna's 1.2 mm overhang means they are not visible from directly above,
but an angled view sees them, which is what castellations are for.

That leaves the 24-pad interior array as the genuinely blind population, all of it ground. B5 is
their only test and it is aimed at exactly them — the four array corners and a near-centre pad.
Combined with 26 of 36 pads being ground, an ordinary electrical check cannot distinguish a sound
module from one hanging on its ten signal joints — those work or they do not, and the ground array
stays silent either way. The witness pads are the only incoming inspection that reaches the interior
array, which is why B5 is worded as mandatory rather than advisory.

**B3, B7 — the second pass fully remelts the first.** Pass 2 takes the whole board above liquidus,
so every bottom-side joint is liquid for 45–90 s with nothing but surface tension holding the part.
`U5` is far too heavy for that, which fixes it to the top side — and that in turn puts `U16`, `J8`
and `J2` on the inverted pass, all three at or under their own weight budget. The datasheet then
makes it a requirement rather than an inference: max. reflow cycle is **1**, and Quectel asks for
the module to be mounted only after the opposite side has been reflowed.

**That 45–90 s figure now conflicts with B4 and the conflict is real.** Quectel caps time above
217 °C at 70 s and applies it to *every* joint on the board, hottest and coldest. A pass-2 profile
that leaves bottom-side joints liquid for 90 s is outside that limit even if `U5`'s own lagging
joints land inside it. Pass 2 has to be developed against both ends at once — long enough at the
witness pads, short enough at the bottom-side joints — and that is a profiling exercise for the
qualification run in B6, not something that can be settled from the file.

**B6 — the recovery path is narrower than the last board's, not absent.** The GNSS daughterboard's
SAM-M10Q was recovered by reworking `U1` wholesale. That is not available here: one reflow cycle
means `U5` cannot be lifted and refitted. What *is* available is the castellation — a cold joint on
any of pads 1–12 is reachable with a fine iron in place, and those twelve carry every signal. A cold
joint in the 24-pad interior array is unreachable and scraps the board. The first build is still a
qualification run, because B5 is the only thing that finds the unreachable failures.

**U5's ten non-ground pads — settled against the datasheet, and none of them may be grounded.**
Recorded because the schematic is already right and the temptation to "improve" it is real: a
grounded pad here would be 103 mW/K of free heat path into `In1`, and Quectel forbids it.

| pad | pin | LC86G Table 6 |
|---|---|---|
| 7, 9 | `RESERVED` | *"must be left floating and cannot be connected to power or GND"* |
| 8 | `AADET_N` | *"If unused, leave the pin N/C."* |
| 10 | `RESET_N` | active-low input; global note: *"Leave RESERVED and unused pins N/C."* |
| 11 | `EX_ANT` | *"50 Ω characteristic impedance. If unused, keep this pin N/C."* |
| 3, 12–36 | `GND` | *"Ensure a good GND connection to all module GND pins, preferably with a large ground plane."* |

All five are unconnected in `in_sensors.kicad_sch` today, which is correct. `RXD`, `TXD`, `VCC`,
`V_BCKP` and `1PPS` are live. There is no thermal case for a via in any of the ten: a signal via
needs an antipad, so it clears `In1` and lands on its own track, which is why the pads keep only
the ~7.5 mW/K their own 2.5 mm² of land gives them either way. Route them **out on `F.Cu`** — the
land already extends 1.00 mm past the module PCB edge, so they escape into open board without a via
and without cutting `In1`.

---

## Still open

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
- ~~**The LC86G's LCC land length is unverified.**~~ **CLOSED — the land is correct as drawn.**
  The module PCB is 16 mm (+0.3/−0.15) and its perimeter pads are 1.5 × 1.0 mm castellated, flush
  at ±8.00 mm. Our 2.5 mm land at ±7.75 mm centres runs ±6.50 to ±9.00: inner edge exactly on the
  module pad's inner edge, 1.00 mm of exposed toe past the castellation. Pitch (2.54 mm) and the
  interior grid (1.0 mm pads, 2.2 mm, 6 × 5) already matched Figure 26. Do not move it.

- **Decide the inspection method for pads 1–12 — angled view, or extend the land for top-down AOI.**
  The castellation fillet at ±8.00 mm is the informative feature and it already exists. The ceramic
  antenna overhangs it by 1.2 mm per side, so seeing it needs a view roughly 30–40° off the board.
  If the assembler only runs top-down AOI, the alternative is extending the land past the antenna
  outline — to about ±9.60 mm to clear the antenna's ±0.2 mm tolerance, so a land of ~3.1 mm rather
  than 2.5 mm, with the stencil aperture scaled to match. That buys a solder feature a vertical
  camera can see, at the cost of ~25% more land area drawing solder away from the castellation it
  is supposed to witness. **Ask the assembler which they can do before changing the footprint** —
  angled inspection of the real fillet is the better answer if it is available.

- ~~**The centre vacancy is the antenna feed keepout.**~~ **Already enforced — leave it alone.** The
  six interior positions the footprint omits (x ±2.2 to 0, y −1.6 and +0.6) are Figure 26's Ø2.4 mm
  keepout, and v1.5 spells out what that means: *"'Keepout' mentioned in Figure 26 is a restricted
  area for traces and vias."* §5.1.4 asks for at least 2.5 mm diameter *on every layer*. The
  footprint already carries it as a **rule area** named `LC86G patch antenna feed keepout` — Ø2.50 mm
  at local (−0.60, −0.64), tracks and vias `not_allowed` on all six copper layers, in both the
  library and the board. DRC enforces it; nothing to add. Ground stitching under the module is still
  wanted everywhere else — Quectel asks for a large ground plane on the GND pins — but not there.

- **§5.1.4 wants a ground plane of at least 30 mm × 30 mm around the module,** with no components
  and no interfering vias in it. `U5` is 18.4 mm, so that is ~5.8 mm of clear plane beyond the body
  on every side. Whether the outline can give it that is an open question against the 22.35 mm
  inherited width.

- ~~**The footprint does not carry v1.5's antenna offset.**~~ **RESOLVED and applied.** The patch is
  offset on the module PCB, and measuring Figure 28 against the interior pad grid (2.2 mm pitch,
  ~122.5 px/mm at 1200 dpi) puts numbers on it. In footprint coordinates:

  | edge | measured | overhang from the ±8.00 PCB |
  |---|---|---|
  | antenna −y | −8.952 | **0.943** — datasheet 0.94 |
  | antenna +y | +9.493 | **1.461** — datasheet 1.46 |
  | antenna −x | −8.992 | 0.992 |
  | antenna +x | +9.414 | 1.414 |

  The Y pair reproduces Quectel's dimensions to 0.003 mm, which settles the axis: **0.94 is on −y,
  1.46 on +y**, and the patch sits toward the +x/+y corner, away from pin 1. The orientation is
  confirmed independently by pad 12 — the tall 2.25 mm land — appearing top-left in the bottom view,
  which is where footprint (+x, −y) predicts. X is offset too, by an amount Quectel does not
  dimension; the values above are measured, not quoted.

  `F.SilkS` now draws the true outline (x −8.99 to +9.41, y −8.94 to +9.46, using the datasheet's
  exact Y overhangs and the measured X) instead of a centred ±9.20 square. `F.Fab` stays at ±8.00 —
  measurement confirms the PCB *is* centred on the pad field (−7.998 / +8.032). The courtyard at
  ±9.75 clears the antenna by 0.29 mm on its worst side, so it needs no further change.

- **§5.3 wants ≥3 mm between the module and other components; thirteen top-side items are
  inside it.** Re-measured courtyard-to-courtyard on 2026-09-04, replacing an older count of
  four: `R33` (0.00), `R140` (0.00), `C19` (0.00), `C21` (0.00), `U11` (0.77), `Y1` (0.90),
  `R141` (1.32), `SW3` (1.90), `R38` (2.21), `R39` (2.22), `C144` (2.26), and the mounting holes
  `H1` (2.41) and `H3` (2.42). The four at 0.00 touch the courtyard box without their outlines
  overlapping — DRC reports no `courtyards_overlap` for any of them. `C38`, which this bullet
  used to name, is on the **bottom** side and so is not what §5.3 is about. On a 22.55 mm board
  this rule cannot be met without moving the module, and the deficit is accepted for this run;
  it is recorded here because a GNSS installation on this project has already lost ≥8 dB once.
  Note the courtyard does **not** encode this rule — it is sized to the part (±9.75), not to the
  3 mm keepout (which would be ±12.2). Enforcing 3 mm through the courtyard is an option if DRC
  should catch it automatically; it would make the courtyard wider than the inherited board width.
