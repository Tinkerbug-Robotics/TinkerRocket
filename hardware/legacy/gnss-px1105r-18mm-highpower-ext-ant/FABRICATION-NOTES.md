# Space Bug — fabrication and assembly notes

This folder's board is the Space Bug, V2: the GNSS receiver with its own processor and IMU, 8 layers.

**Paste the block below into the fab's order notes verbatim.**

None of it can travel in the design files. The surface finish, the via treatment, the reflow limits and the assembly
steps do not reach the fab through the gerbers or the Excellon drill files, so written text is the only channel.
Stencil thickness, the area-ratio floor and the paste-coverage convention are repo-wide and live in
[`hardware/SOLDER-PASTE-CONVENTION.md`](../../SOLDER-PASTE-CONVENTION.md) (#959, #906).

---

```
TINKERROCKET SPACE BUG V2 - FABRICATION AND ASSEMBLY NOTES
BOARD 35.0 x 35.0 mm, 8 LAYER, 1.63 mm STACK.

1. STACKUP: JLCPCB JLC08161H-2116, 1 oz OUTER / 0.5 oz INNER.
   BUILD TO THIS TEMPLATE; THE RF TRACE WIDTH IS DRAWN FOR IT.
   OUTER PREPREG 2116 AT 0.1164 mm (er 4.16), CORES 0.300 mm
   (er 4.41), INNER PREPREG 1080 x2 AT 0.1528 mm (er 3.91).

2. SURFACE FINISH: ENIG. REQUIRED FOR THE 0.35 mm PITCH QFN-104
   PROCESSOR (U6), THE 0.5 mm PITCH 24-BALL WLCSP FLASH (U7), THE
   0.4 mm PITCH AMPLIFIERS (U3, U4) AND THE USB-C RECEPTACLE (J5).
   DO NOT SUBSTITUTE HASL.

3. VIAS: 441 THROUGH VIAS ON 0.30 mm DRILLS (439 WITH 0.40 mm
   PADS, 2 WITH 0.60 mm). FILL WITH NON-CONDUCTIVE EPOXY AND PLATE
   OVER (CAPPED), IPC-4761 TYPE VII.
   *** VIA-IN-PAD IS MANDATORY: 161 VIAS SIT IN SMD LANDS, 29 OF
   THEM IN U6'S EXPOSED PAD AND 9 IN U2'S. ***
   OUTSIDE THE LANDS, ALL VIAS ARE TENTED BOTH SIDES.

4. RF TRACES: 0.18 mm ON F.Cu WITH A 0.20 mm GAP TO THE GROUND
   POUR, OVER In1.Cu GROUND: 50 OHM ON THIS STACKUP. DO NOT ADJUST
   THE WIDTHS. THE LONGEST RUN IS 6.5 mm (ABOUT 22 DEGREES AT
   1.6 GHz), SO IMPEDANCE TESTING IS OPTIONAL.

5. SOLDER MASK: THE WEB BETWEEN U6'S 0.35 mm PITCH PINS IS 0.06 mm
   AS DRAWN. IF THAT CANNOT BE HELD, GANG THE OPENING ALONG EACH
   ROW RATHER THAN SHRINKING THE PADS.
   U3 AND U4 EACH HAVE ONE OPENING OVER THE WHOLE PAD ARRAY ON
   PURPOSE (NON-MASK-DEFINED LANDS, AS THEIR MAKER SPECIFIES). DO
   NOT ADD DAMS THERE.

6. MINIMUM ANNULAR RING 0.05 mm (0.40 mm VIA PAD ON 0.30 mm DRILL).

7. MINIMUM TRACK 0.09 mm, MINIMUM SPACING 0.10 mm.
   HOLE TO HOLE 0.25 mm BETWEEN DIFFERENT NETS AND 0.21 mm WITHIN
   A NET, MEASURED HOLE EDGE TO HOLE EDGE.

8. OTHER HOLES: 4 x 2.20 mm PLATED (H1-H4), 1 x 1.00 mm PLATED
   (AE1), 2 x 0.65 mm NON-PLATED (J5 LOCATING PEGS).

ASSEMBLY - STENCIL
STENCIL FOIL 0.08 mm (3 mil), FLAT - NO STEP. TOP STENCIL ONLY;
THE BOTTOM HAS NO PASTE.
LASER CUT, ELECTROPOLISHED AND NANO-COATED.
PRINT THE PASTE LAYER AS SUPPLIED, APERTURES 1:1. THE EXPOSED PADS
ARE WINDOWED ON PURPOSE: U6 40 % (9 WINDOWS), U2 51 % (4 WINDOWS),
U9 57 % (4 WINDOWS).
TIGHTEST APERTURES: U3/U4 0.25 mm ROUND, AREA RATIO 0.78 AT
0.08 mm (0.63 AT 0.10 mm, BELOW THE IPC-7525 FLOOR OF 0.66); U7
0.254 mm ROUND, 0.79 AT 0.08 mm.

ASSEMBLY - REFLOW
ONE PASS, TOP SIDE ONLY.
*** U1 (THE GNSS MODULE) SETS THE CEILING: PEAK 240 C FOR 25-35 s
AND 60-80 s ABOVE 220 C, MEASURED AT U1. ITS MAKER SAYS THESE
SHOULD NOT BE EXCEEDED. *** THE NEXT LOWEST LIMIT IS THE SAW
FILTERS (FL4, FL5) AT 250 C.
THE BOARD IS HEAVY FOR ITS SIZE (SIX GROUND PLANES): PROFILE TO
235-240 C AT U1 AND CHECK IT WITH A THERMOCOUPLE ON THE FIRST
ARTICLE.
U1 IS MSL 4: 72 h FLOOR LIFE, THEN BAKE AT 85 C FOR 8-12 h.
FL4 AND FL5 ARE ESD SENSITIVE (250 V HBM).

ASSEMBLY - PLACEMENT
- FIDUCIALS FID1-FID3, TOP SIDE.
- U6 MUST BE CHIP REVISION v3.x. CONFIRM IT ON THE PACKAGE MARKING.
  THE TINKER-MANTIS TAKES v1.3: DO NOT SHARE ITS STOCK.
- U6 AND U10 HAVE NO PIN-1 MARK ON THE TOP SILK. CHECK BOTH IN
  THE PLACEMENT PREVIEW.
- U7 (24-BALL WLCSP) IS 180-DEGREE SYMMETRIC: A ROTATED PART STILL
  FITS AND PUTS VCC ON THE GROUND BALL. BALL A1 IS THE CORNER
  NEAREST THE SILK DOT.
- U10, C70 AND C71 SIT AT 45 / 135 DEGREES.
- C19 IS DO-NOT-FIT.

ASSEMBLY - HAND WORK
- AE1 (PATCH ANTENNA) FITS AFTER REFLOW ON THE BOTTOM, HELD BY ITS
  ADHESIVE PAD. ITS PIN GOES THROUGH THE 1.00 mm PLATED HOLE AND IS
  SOLDERED FROM THE TOP.
- TP1-TP4, TP7 AND TP8 ON THE BOTTOM ARE BARE TEST AND PROGRAMMING
  PADS. NOTHING IS FITTED THERE.
```

---

## Why each item is here

Every figure in the block was measured from the board file on 2026-09-28, after the design review's fixes
([`design-review-2026-09-27.md`](design-review-2026-09-27.md), D1 and D7).

**1 — stackup.** The RF line is grounded coplanar waveguide over 0.1164 mm of 2116 prepreg. A 2-D field solve puts
the drawn 0.18 mm track with its 0.20 mm gaps at 50 Ω under mask and 52 Ω bare. On JLC's standard 4-layer stack
(0.2104 mm of 7628 prepreg) the same copper would be 63–65 Ω, so the template is part of the design.

**2 — finish.** The finish reaches the `.gbrjob` only as a string that is easy to miss. HASL's coplanarity is wrong for
the 0.35 mm QFN and the WLCSP.

**3 — vias.** Vias sit in 161 lands: the processor's exposed pad (29, its heat path), U2's (9) and U9's (3), and the
decoupling capacitors' pads. An unfilled barrel in a land wicks solder out of the joint above it. The board setup
declares IPC-4761 Type VII (filled and capped), as the other multilayer boards do, but no fab reads that from the
gerbers.

**4 — RF.** At 1.6 GHz the longest run, the 6.5 mm into the second filter (FL5), is about 22 degrees. A
10 % impedance miss on it costs under 0.01 dB, so the width matters more than a test coupon.

**5 — mask.** The processor footprint gives each pin its own opening, 0.055 mm beyond the pad on a 0.35 mm pitch, which
leaves a 0.06 mm web: below what most fabs hold. The Tinker-Mantis uses the same footprint. Ganging keeps the lands at
full size; shrinking the openings would make them mask-defined. U3 and U4's single window is deliberate (review S10).

**6, 7, 8 — minimums and holes.** They are declared up front so the order doesn't sit in DFM review. The 0.21 mm
same-net spacing occurs at eight via pairs: seven on ground and one on the processor core supply.

**Stencil.** The amplifiers set the foil. Their 0.25 mm round apertures, the maker's recommended stencil opening,
reach area ratio 0.78 on an 80 µm foil and 0.63 on the 100 µm repo default, below the IPC-7525 floor of 0.66. They clear
0.66 only on a foil of 94 µm or thinner, so 80 µm is the next standard step. The next finest apertures:

| Part | Aperture | Area ratio at 80 µm |
|---|---|---|
| U7, the boot flash | 0.254 mm round | 0.79 (0.64 at 100 µm) |
| U6, the processor | 0.18 × 0.65 mm | 0.88 |
| FL4, FL5 | 0.30 × 0.375 mm | 1.04 |
| U10 | 0.305 × 0.533 mm | 1.21 |
| D6 | 0.30 × 0.50 mm | 1.23 |
| U9 | 0.26 × 0.86 mm | 1.25 |
| U8 | 0.30 × 0.67 mm | 1.30 |
| J5, the USB-C receptacle | 0.30 × 1.15 mm | 1.49 |
| FL2's four windows | 0.90 × 0.35 mm | 1.58 |

Everything else is at area ratio 1.9 or more. The exposed-pad windows follow the paste convention: U6's come from its
footprint, U9's from 4078070b, and U2's from review S11. U1 prints at full pad (review S8). Nothing on the bottom
takes paste: the patch antenna's pin is soldered by hand and the six test pads stay bare.

**Reflow.** U1's limits come from its datasheet, which says they should not be exceeded. An 8-layer board with six
ground planes and a 7.5 mm exposed pad under the processor tempts a hotter profile, so the usable window at U1 is
narrow: 235–240 °C. The SAW filters' datasheet allows 250 °C and rates them 250 V HBM.

**Placement.**
- **U6:** the revision cannot be read from the footprint, and the Tinker-Mantis's fab notes (B8) ask for the opposite
  silicon.
- **U6 and U10:** neither has a top-side pin-1 mark (review L14). U6's front silk is eight identical corner brackets,
  and U10's circle is on the back silk, under AE1. Both parts fit rotated.
- **U7:** a rotated WLCSP powers the flash backwards.
