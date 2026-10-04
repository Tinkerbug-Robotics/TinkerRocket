# Space Bug M8T — fabrication and assembly notes

This folder's board is the Space Bug M8T, V1: the Space Bug's processor and IMU around a u-blox NEO-M8T timing
receiver, 6 layers.

**Paste the block below into the fab's order notes verbatim.**

None of it can travel in the design files. The surface finish, the via treatment, the reflow limits and the assembly
steps do not reach the fab through the gerbers or the Excellon drill files, so written text is the only channel.
Stencil thickness, the area-ratio floor and the paste-coverage convention are repo-wide and live in
[`hardware/SOLDER-PASTE-CONVENTION.md`](../SOLDER-PASTE-CONVENTION.md) (#959, #906).

---

```
TINKERROCKET SPACE BUG M8T V1 - FABRICATION AND ASSEMBLY NOTES
BOARD 35.0 x 35.0 mm, 6 LAYER, 1.6 mm NOMINAL.

1. STACKUP: JLCPCB JLC06161H-3313, 1 oz OUTER / 0.5 oz INNER.
   BUILD TO THIS TEMPLATE; THE RF TRACE WIDTH IS DRAWN FOR IT.
   OUTER PREPREGS 3313 AT 0.0994 mm (er 4.05), CORES 0.55 mm
   (er 4.38), MIDDLE PREPREG 2116 AT 0.1088 mm (er 4.16).
   LAYERS: F.Cu SIGNAL, In1 GND, In2 SIGNAL, In3 GND, In4 POWER,
   B.Cu GROUND POUR.

2. SURFACE FINISH: ENIG. REQUIRED FOR THE 0.35 mm PITCH QFN-104
   PROCESSOR (U6), THE 0.5 mm PITCH 24-BALL WLCSP FLASH (U7), THE
   0.4 mm PITCH AMPLIFIER (U3) AND THE USB-C RECEPTACLE (J5).
   DO NOT SUBSTITUTE HASL.

3. VIAS: 355 THROUGH VIAS, ALL 0.30 mm DRILL WITH 0.40 mm PADS.
   FILL WITH NON-CONDUCTIVE EPOXY AND PLATE OVER (CAPPED),
   IPC-4761 TYPE VII.
   *** VIA-IN-PAD IS MANDATORY: 164 VIAS SIT IN SMD LANDS, 29 OF
   THEM IN U6'S EXPOSED PAD, 9 IN U2'S AND 4 IN U9'S, 12 IN THE
   LANDS OF THE GNSS MODULE (U1), 4 IN BALLS OF THE 0.5 mm-PITCH
   FLASH (U7) AND 3 IN PADS OF THE USB-C RECEPTACLE (J5). ***
   OUTSIDE THE LANDS, ALL VIAS ARE TENTED BOTH SIDES.

4. RF TRACES: 0.16 mm ON F.Cu WITH A 0.20 mm GAP TO THE GROUND
   POUR, OVER In1.Cu GROUND: 50 OHM ON THIS STACKUP. DO NOT ADJUST
   THE WIDTHS. THE LONGEST RF NET IS 4.2 mm (ABOUT 14 DEGREES AT
   1.6 GHz), SO IMPEDANCE TESTING IS OPTIONAL.

5. SOLDER MASK: THE WEB BETWEEN U6'S 0.35 mm PITCH PINS IS 0.06 mm
   AS DRAWN. IF THAT CANNOT BE HELD, GANG THE OPENING ALONG EACH
   ROW RATHER THAN SHRINKING THE PADS.
   U3 HAS ONE OPENING OVER THE WHOLE PAD ARRAY ON PURPOSE
   (NON-MASK-DEFINED LANDS, AS ITS MAKER SPECIFIES). DO NOT ADD
   DAMS THERE.

6. MINIMUM ANNULAR RING 0.05 mm (0.40 mm VIA PAD ON 0.30 mm DRILL).

7. MINIMUM TRACK 0.10 mm (0.09 mm DESIGN RULE), MINIMUM SPACING
   0.10 mm, COPPER TO BOARD EDGE 0.20 mm.
   HOLE TO HOLE 0.20 mm, MEASURED HOLE EDGE TO HOLE EDGE.

8. OTHER HOLES: 4 x 2.20 mm PLATED (H1-H4), 1 x 1.00 mm PLATED
   (AE1), 2 x 0.65 mm NON-PLATED (J5 LOCATING PEGS).

ASSEMBLY - STENCIL
STENCIL FOIL 0.08 mm (3 mil), FLAT - NO STEP. TOP STENCIL ONLY;
THE BOTTOM HAS NO PASTE.
LASER CUT, ELECTROPOLISHED AND NANO-COATED.
PRINT THE PASTE LAYER AS SUPPLIED, APERTURES 1:1. THE EXPOSED PADS
ARE WINDOWED ON PURPOSE: U6 40 % (9 WINDOWS), U2 51 % (4 WINDOWS),
U9 57 % (4 WINDOWS). U1'S APERTURES RUN 0.4 mm PAST EACH LAND,
AWAY FROM THE MODULE, ON PURPOSE.
TIGHTEST APERTURES: U3 0.25 mm ROUND, AREA RATIO 0.78 AT 0.08 mm
(0.63 AT 0.10 mm, BELOW THE IPC-7525 FLOOR OF 0.66); U7 0.254 mm
ROUND, 0.79 AT 0.08 mm.

ASSEMBLY - REFLOW
ONE PASS, TOP SIDE ONLY. NO-CLEAN PASTE, AND NO CLEANING AFTER
REFLOW: NO WATER, NO SOLVENT, NO ULTRASONIC BATH. ULTRASOUND
DAMAGES U1 (THE GNSS MODULE), AND WASH LIQUID TRAPPED UNDER IT
LEAVES LEAKAGE PATHS BETWEEN ITS PADS.
*** U1 SETS THE CEILING: PEAK 245 C, 40-60 s ABOVE 217 C,
PREHEAT TO 150-200 C OVER 60-120 s, RAMP MAX 3 C/s, COOLING MAX
4 C/s. ITS MAKER SAYS EXCEEDING THE PEAK CAN DAMAGE IT. ***
FL4 (SAW FILTER) NEEDS AT LEAST 10 s ABOVE 230 C AND ALLOWS UP TO
250 C: PROFILE TO 240-245 C AT U1 AND CHECK IT WITH A THERMOCOUPLE
ON THE FIRST ARTICLE.
U1 IS MSL 4: 72 h FLOOR LIFE; PAST THAT, DRY IT PER J-STD-033
BEFORE REFLOW.
FL4 IS ESD SENSITIVE (225 V HBM).

ASSEMBLY - PLACEMENT
- FIDUCIALS FID1-FID3, TOP SIDE. FID1 LIES UNDER U1 (THE GNSS
  MODULE): READ IT BEFORE U1 IS PLACED.
- PLACE U1 (GNSS MODULE) BY ITS COPPER PADS, NOT BY THE MODULE'S
  EDGE.
- U6 MUST BE CHIP REVISION v3.x. CONFIRM IT ON THE PACKAGE MARKING.
  THE TINKER-MANTIS TAKES v1.3: DO NOT SHARE ITS STOCK.
- U6 AND U10 HAVE NO PIN-1 MARK ON THE TOP SILK. CHECK BOTH IN
  THE PLACEMENT PREVIEW.
- U7 (24-BALL WLCSP) IS 180-DEGREE SYMMETRIC: A ROTATED PART STILL
  FITS AND PUTS VCC ON THE GROUND BALL. BALL A1 IS THE CORNER
  NEAREST THE SILK DOT.
- U10, C70, C71, C72 AND R19 SIT AT 45-DEGREE ANGLES.
- C19 IS DO-NOT-FIT.

ASSEMBLY - HAND WORK
- AE1 (PATCH ANTENNA) FITS AFTER REFLOW ON THE BOTTOM, HELD BY ITS
  ADHESIVE PAD. ITS PIN GOES THROUGH THE 1.00 mm PLATED HOLE AND IS
  SOLDERED FROM THE TOP. THE NEAREST PARTS (L1, C20, C42) ARE
  0.85-0.95 mm FROM THE PIN'S PAD: USE A FINE TIP.
```

---

## Why each item is here

Every figure in the block was measured on 2026-10-01 from the board file the package was plotted from: the routed
6-layer layout, md5 `4f1e2012`. Its zone fills are a kicad-cli fixed point.

**1 — stackup.** The same template and layer roles as the Space Bug V3 (#1552), whose notes derive the numbers. The
RF line is grounded coplanar waveguide over 0.0994 mm of 3313 prepreg: the drawn 0.16 mm track with 0.20 mm gaps is
49 Ω under mask and 51 Ω bare. On a different stack the same copper is a different impedance, so the template is
part of the design. Every signal layer sits next to ground: In2 is 0.11 mm over In3, and the power layer (In4) is
0.10 mm over the bottom pour. In4 carries the receiver's 3.3 V west of x ≈ 56 mm and the processor's east of it.

**2 — finish.** HASL's coplanarity is wrong for the 0.35 mm QFN and the WLCSP. The finish reaches the `.gbrjob`
only as a string that is easy to miss.

**3 — vias.** Vias sit in 164 lands:
- 29 in the processor's exposed pad, its heat path to the inner ground planes;
- nine in U2's exposed pad and four in U9's;
- twelve in the NEO-M8T's castellated lands;
- four in the flash's balls, whose inner balls have no other way out at 0.5 mm pitch;
- three in the USB-C receptacle's pads;
- the rest in the decoupling capacitors' pads.

An unfilled barrel in a land wicks solder out of the joint above it; under a 0.25 mm ball it would take the whole
joint. The board setup declares IPC-4761 Type VII (filled and capped), but no fab reads that from the gerbers.

**4 — RF.** The chain is compact. The eight RF nets total 10 mm. The longest is the feed with its ESD and
do-not-fit shunt branches, at 4.2 mm, about 14 degrees at 1.6 GHz. A 10 % impedance miss over that costs well under
0.1 dB, so the width matters more than a test coupon.

**5 — mask.** The processor footprint gives each pin its own opening, 0.055 mm beyond the pad on a 0.35 mm pitch,
which leaves a 0.06 mm web: below what most fabs hold. Ganging keeps the lands at full size; shrinking the openings
would make them mask-defined. U3's single window is deliberate (Space Bug review S10).

**6, 7, 8 — minimums and holes.** They are declared up front so the order doesn't sit in DFM review. The design
rules follow the Tinker-Mantis: 0.10 mm clearance, 0.09 mm minimum track, and 0.20 mm hole to hole and copper to
edge. The closest holes are 0.202 mm apart edge to edge between different nets, and 0.208 mm within a net (two pairs
of ground vias).

**Stencil.** The amplifier sets the foil. Its 0.25 mm round apertures, the maker's recommended stencil opening, reach
area ratio 0.78 on an 80 µm foil and 0.63 on the 100 µm repo default, below the IPC-7525 floor of 0.66. The flash's
0.254 mm balls reach 0.79 (0.64 at 100 µm). Everything else clears 0.91 or more at 80 µm: the processor's
0.18 × 0.65 mm pins 0.91, the SAW filter's 0.30 × 0.375 mm 1.04, U10 1.21, D6 1.23, the NEO's 2.2 × 0.8 mm 3.7
(436 apertures, each measured on its drawn shape). The exposed-pad windows follow the paste convention and are the
same footprints as the Space Bug V3.

The NEO-M8T's integration manual draws a T-shaped paste pattern that reaches past each land, to 14.6 mm across the
module, for a 150 µm stencil. U1's apertures follow that outer extent, but at 80 µm they print about half the paste
volume u-blox assumes. **Check U1's edge fillets on the first article**; if they are thin, a stencil stepped up over
U1 is the fix.

**Reflow.** U1's limits come from the NEO-M8 hardware integration manual: peak 245 °C, 40–60 s above the 217 °C
liquidus, one reflow pass, no washing and no ultrasonic process of any kind. The SAW filter (FL4) wants at least 10 s
above 230 °C for wetting and allows 250 °C peak, so the usable window at U1 is 240–245 °C. The filter is rated 225 V
HBM.

**Placement.**
- **U1:** u-blox asks for placement aligned to the copper, not the module edge. U1's silk marks pin 1 at its
  lower-right corner as placed.
- **U6:** the revision cannot be read from the footprint, and the Tinker-Mantis's fab notes (B8) ask for the opposite
  silicon.
- **U6 and U10:** neither has a top-side pin-1 mark. U6's front silk is eight identical corner brackets, and U10's
  circle is on the back silk, under AE1. Both parts fit rotated.
- **U7:** a rotated WLCSP powers the flash backwards.
