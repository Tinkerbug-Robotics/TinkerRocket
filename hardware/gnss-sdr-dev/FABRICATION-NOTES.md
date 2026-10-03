# gnss-sdr-dev — fabrication and assembly notes

This folder's board is the GNSS SDR development board, V0: the L1/L5 front ends, the ECP5 FPGA and the ESP32-P4,
8 layers, 80 × 60 mm.

**Paste the block below into the fab's order notes verbatim.**

None of it can travel in the design files. The surface finish, the via treatment, the reflow limits and the assembly
steps do not reach the fab through the gerbers or the Excellon drill files, so written text is the only channel.
Stencil thickness, the area-ratio floor and the paste-coverage convention are repo-wide and live in
[`hardware/SOLDER-PASTE-CONVENTION.md`](../SOLDER-PASTE-CONVENTION.md).

---

```
TINKERROCKET GNSS-SDR-DEV V0 - FABRICATION AND ASSEMBLY NOTES
BOARD 80.0 x 60.0 mm, 8 LAYER, 1.6 mm NOMINAL.

1. STACKUP: JLCPCB JLC08161H-2116, 1 oz OUTER / 0.5 oz INNER.
   BUILD TO THIS TEMPLATE; THE RF AND USB TRACES ARE DRAWN FOR IT.
   OUTER PREPREGS 2116 AT 0.1164 mm (er 4.16), CORES 0.30 mm
   (er 4.41), INNER PREPREGS 1080 x2 AT 0.1528 mm (er 3.91).
   LAYERS: F.Cu SIGNAL, In1 GND, In2 SIGNAL, In3 GND, In4 POWER,
   In5 SIGNAL, In6 SIGNAL, B.Cu GROUND POUR.

2. SURFACE FINISH: ENIG. REQUIRED FOR THE 0.35 mm PITCH QFN-104
   PROCESSOR (U6), THE 0.8 mm PITCH 256-BALL BGA (U501), THE
   0.5 mm PITCH 24-BALL WLCSP FLASH (U7), THE 0.4 mm PITCH
   AMPLIFIERS (U101, U102) AND USB SWITCH (U602), AND THE USB-C
   RECEPTACLE (J601). DO NOT SUBSTITUTE HASL.

3. VIAS: 1164 THROUGH VIAS, ALL 0.40 mm PADS ON 0.30 mm DRILLS.
   FILL WITH NON-CONDUCTIVE EPOXY AND PLATE OVER (CAPPED),
   IPC-4761 TYPE VII.
   *** VIA-IN-PAD IS MANDATORY: 438 VIAS SIT IN SMD LANDS, 84 OF
   THEM IN U501'S BALL LANDS, 35 IN U6'S EXPOSED PAD AND 45 IN THE
   SHIELD FRAME'S LANDS (SH1). ***
   OUTSIDE THE LANDS, ALL VIAS ARE TENTED BOTH SIDES.
   U7'S VIAS SIT BESIDE ITS 0.25 mm BALL LANDS, NOT IN THEM.

4. CONTROLLED IMPEDANCE ON THIS STACKUP. DO NOT ADJUST THE WIDTHS.
   - RF: 0.16 mm ON F.Cu WITH A 0.20 mm GAP TO THE GROUND POUR,
     OVER In1.Cu GROUND: 52 OHM. LONGEST RUN 13.6 mm.
   - USB 2.0: 0.13 mm TRACES WITH A 0.12 mm GAP ON In2.Cu, BETWEEN
     THE In1 AND In3 GROUND PLANES: 90 OHM DIFFERENTIAL.
   IMPEDANCE TESTING IS OPTIONAL.

5. SOLDER MASK: THE WEB BETWEEN U6'S 0.35 mm PITCH PINS IS 0.06 mm
   AS DRAWN. IF THAT CANNOT BE HELD, GANG THE OPENING ALONG EACH
   ROW RATHER THAN SHRINKING THE PADS.
   U101 AND U102 EACH HAVE ONE OPENING OVER THE WHOLE PAD ARRAY ON
   PURPOSE (NON-MASK-DEFINED LANDS, AS THEIR MAKER SPECIFIES). DO
   NOT ADD DAMS THERE.
   U501'S BALL LANDS ARE NON-MASK-DEFINED, 0.07 mm MASK CLEARANCE.

6. MINIMUM ANNULAR RING 0.05 mm (0.40 mm VIA PAD ON 0.30 mm DRILL).

7. MINIMUM TRACK 0.10 mm, MINIMUM SPACING 0.10 mm (0.20 mm FROM THE
   RF TRACES TO THE GROUND POUR). HOLE TO HOLE 0.20 mm MINIMUM,
   MEASURED HOLE EDGE TO HOLE EDGE. COPPER TO BOARD EDGE 0.20 mm,
   EXCEPT J101: THE EDGE-LAUNCH SMA'S PADS, ON BOTH SIDES, RUN TO
   THE BOARD EDGE BY DESIGN. DO NOT PULL THAT COPPER BACK.

8. OTHER HOLES: 4 x 2.20 mm PLATED (H1-H4), 26 x 1.00 mm PLATED
   (J501 AND J502 HEADER PINS), 2 x 0.65 mm NON-PLATED (J601
   LOCATING PEGS).

ASSEMBLY - STENCIL
STENCIL FOIL 0.08 mm (3 mil), FLAT - NO STEP. TOP STENCIL ONLY.
THE BOTTOM PASTE LAYER HOLDS ONLY J101'S TWO GROUND PADS, WHICH
ARE SOLDERED BY HAND: NO BOTTOM STENCIL.
LASER CUT, ELECTROPOLISHED AND NANO-COATED.
PRINT THE PASTE LAYER AS SUPPLIED, APERTURES 1:1. THE EXPOSED PADS
ARE WINDOWED ON PURPOSE: U6 40 % (9 WINDOWS), U701 76 % (6), U201
AND U301 63 % (4 EACH), U9 56 % (4), U401 51 % (4).
TIGHTEST APERTURES: U101/U102 0.25 mm ROUND, AREA RATIO 0.77 AT
0.08 mm (0.62 AT 0.10 mm, BELOW THE IPC-7525 FLOOR OF 0.66); U7
0.25 mm ROUND, 0.79 AT 0.08 mm.

ASSEMBLY - REFLOW
ONE PASS, TOP SIDE ONLY.
*** THE SAW FILTERS (FL101, FL102) SET THE CEILING: 250 C PEAK. ***
PROFILE TO 240-245 C AND CHECK IT WITH A THERMOCOUPLE AT U501 ON
THE FIRST ARTICLE: TWO SOLID GROUND PLANES, A NEAR-SOLID BOTTOM
POUR AND THE SHIELD FRAME ALL SINK HEAT.
X-RAY U501 (BGA) AND U7 (WLCSP) ON THE FIRST ARTICLE.
THE SAW FILTERS ARE ESD SENSITIVE.

ASSEMBLY - PLACEMENT
- FIDUCIALS FID1-FID3, TOP SIDE.
- U6 MUST BE CHIP REVISION v3.x. CONFIRM IT ON THE PACKAGE MARKING.
- U6 AND U602 HAVE NO PIN-1 MARK ON THE TOP SILK. CHECK BOTH IN
  THE PLACEMENT PREVIEW.
- U7 (24-BALL WLCSP) IS 180-DEGREE SYMMETRIC: A ROTATED PART STILL
  FITS AND PUTS VCC ON THE GROUND BALL. BALL A1 IS THE CORNER
  NEAREST THE SILK DOT.
- U10, C70, C71 AND C72 SIT AT 45 DEGREES, R19 AT 135.
- DO NOT FIT: C103, C527-C535, R502, R511, R512, U502.

ASSEMBLY - HAND WORK
- J501 (1 x 6) AND J502 (2 x 10) ARE 2.54 mm THROUGH-HOLE HEADERS,
  SOLDERED AFTER REFLOW.
- J101 (EDGE-LAUNCH SMA) STRADDLES THE EDGE: SOLDER ITS TWO BOTTOM
  GROUND PADS BY HAND AFTER REFLOW.
- SH1 IS THE SHIELD FRAME AND IS REFLOWED. ITS SNAP-ON COVER IS A
  SEPARATE PART, FITTED AFTER BRING-UP.
```

---

## Why each item is here

Every figure in the block was measured on 2026-10-03 from the board as saved at 05:30 that morning. That is the
committed board (113169c9) plus three F.Cu pours at the 1.1 V buck (U9 input, switch node, +1V1_FPGA). DRC: 0 errors,
0 unconnected, schematic parity clean, and a kicad-cli refill reproduces the stored fills byte for byte.

**1 — stackup.** It is the Beetle and Mantis stack. The board started on 6 layers and went to 8 on 2026-10-01 so the
processor's pins could fan out. In1 and In3 are solid ground planes. In2 carries the USB pairs and other signals that
need a reference above and below. In5 runs north-south and In6 east-west under the processor and FPGA. In4 carries
the supplies.

**2 — finish.** HASL's coplanarity is wrong for the 0.35 mm QFN, the 0.8 mm BGA and the WLCSP.

**3 — vias.** All vias are 0.40/0.30 mm. 438 of them sit in SMD lands:

| Where | Vias |
|---|---|
| U501 ball lands (BGA fan-out) | 84 |
| SH1 shield-frame lands | 45 |
| U6 (35 in the exposed pad) | 39 |
| U401 (12 in the exposed pad) | 14 |
| U9, U201, U301 (exposed pads and pins) | 9 each |
| J101 ground pads | 6 |
| U701 | 5 |
| Capacitor, crystal and connector lands | the rest |

An unfilled barrel in a land wicks solder out of the joint above it. JLC queried an earlier revision for vias the size
of U7's 0.25 mm ball lands. They now sit beside the balls, with none touching a ball land, and the block says so to
head off the same query.

**4 — impedance.** Both figures come from a 2-D field solve of this stackup, including the 0.015 mm solder mask and the
0.20 mm gap to the pour:

| Line | Drawn | Result |
|---|---|---|
| RF, F.Cu over In1 | 0.16 mm, 0.20 mm gaps | 52 Ω under mask (55 Ω bare) |
| RF legs on In2 (ANT, L1 bypass) | 0.13 mm | 55 Ω |
| USB 2.0 pairs on In2 | 0.13 mm / 0.12 mm gap | 89–90 Ω differential |

The RF lines were drawn at 0.16 mm for the 6-layer stack; on this stack 0.18 mm would be 50 Ω. A 52 Ω section in a
50 Ω chain reflects 2 %, a mismatch loss under 0.01 dB. The longest runs are short electrically:
- RF_L5, 13.6 mm, about 34 degrees at L5;
- RF_L1, 11.4 mm, about 38 degrees at L1.

USB 2.0 allows 90 Ω ± 15 %. Neither needs a test coupon.

**5 — mask.** The processor footprint draws a 0.29 × 0.76 mm opening per pin on a 0.35 mm pitch, which leaves a
0.06 mm web: below what most fabs hold. Ganging keeps the lands at full size; shrinking the openings would make them
mask-defined. U101 and U102 allow mask bridges in their footprints on purpose. The next tightest webs are 0.10 mm (U602,
U9, J601).

**6, 7, 8 — minimums and holes.** They are declared up front so the order doesn't sit in DFM review. The rules
(Tinker-Mantis) allow 0.09 mm track; the narrowest drawn track is 0.10 mm. J101's pads start 0.05 mm inside the edge
line, top and bottom; the board's rules exempt them from the 0.20 mm edge clearance.

**Stencil.** The amplifiers' and the flash's round apertures set the foil:

| Part | Aperture | Area ratio at 80 µm |
|---|---|---|
| U101, U102 | 0.248 mm round | 0.77 (0.62 at 100 µm) |
| U7, U502 | 0.252 mm round | 0.79 (0.63 at 100 µm) |
| U6 | 0.18 × 0.65 mm | 0.91 |
| U602 | 0.20 × 0.80 mm | 1.00 |
| FL101, FL102 | 0.30 × 0.375 mm | 1.04 |
| U501 | 0.357 mm round | 1.12 |

Everything else is at area ratio 1.2 or more. The exposed-pad windows come from the footprints. U502 is do-not-fit
but its apertures are in the paste layer; its lands will carry solder. The bottom paste layer has exactly two
apertures, J101's 3.5 × 1.5 mm ground pads. A second stencil for two pads on a straddle connector is not worth it.

**Reflow.** FL102, the L5 chain's filter, is the Space Bug's filter. Its datasheet allows 250 °C and rates it
250 V HBM. FL101, the L1 chain's filter, comes in the same 1.4 × 1.1 mm package; its datasheet was not re-read. The boot
button (SW1) and the TCXO (Y201) ratings were not confirmed either, so check them before setting a profile above
245 °C.

**Placement.**
- **U6:** the revision cannot be read from the footprint. Every P4 in this project moves to v3.x (owner decision,
  2026-10-01).
- **U6 and U602:** neither footprint carries a pin-1 mark on the top silk. Reference designators are hidden on this
  board's silkscreen, so placement goes by the position file.
- **U7:** a rotated WLCSP powers the flash backwards.
- **Do-not-fit:** the parts the schematic marks DNP. The position file leaves them out.
