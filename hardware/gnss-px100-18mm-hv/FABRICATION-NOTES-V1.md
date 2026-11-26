# gnss-px100-18mm-hv V1 — fabrication and assembly notes (superseded)

**These are the V1 notes as they stood on 2026-10-30, kept for the record of the V1 order and the V1 HASL boards.**
They describe parts that V2 no longer has (the TSNP LNAs, the B8389 and B4327 filters, L13, C37 at 1.3 pF). Do not
build a V2 board from them: use [`FABRICATION-NOTES.md`](FABRICATION-NOTES.md).


**V1, released 2026-10-23** (tag `gnss-px100-18mm-hv-v1.0.0`) and ordered as bare boards from JLC. V2 is in work
and these notes still apply to it unchanged.

**The V1 order doesn't match notes 1-3.** JLC order Y349-3297386A (2026-10-23, shipped) was placed with Surface Finish
HASL (with lead), Via Covering Plugged and Specify Stackup "No requirement", which is JLC's default template for 4
layers at 1.6 mm, JLC04161H-7628 (the owner left it unselected). So the V1 boards are HASL, U1's five exposed-pad vias
are not filled and capped (JLC can't ink-plug vias in or within 0.35 mm of a pad), and the stackup is 7628. The
stackup turns out not to matter (note 1). The finish and the vias do: use the *Order checklist* below for any
re-order. To build the V1 boards as they are, see *V1 HASL boards* below and
[`design-review-2026-10-27.md`](design-review-2026-10-27.md), section 1. The review recommends re-ordering before a
PX100 goes on, and using the HASL boards for a stage-1-only build (README *Bring-up*, step 1).

**Parts changes of 2026-10-26, for the V1 builds and V2:**
- L13 is not fitted, and C37 is 1.3 pF instead of 4 pF. L13 and L15 would ground the PX100's L5 input (pin 37) for
  DC; README *Front end* explains why.
- C48 is a 0 Ω jumper instead of a 1 nF capacitor, so the TCXO drives TCXO_I directly, as in SkyTraq's reference
  (README, Y1 row).

The tag's BOM still lists L13 fitted, C37 at 4 pF and C48 at 1 nF, so build from the schematic in work, not from the
tag.
Stencil thickness,
the area-ratio floor and the paste-coverage convention are repo-wide in
[`hardware/SOLDER-PASTE-CONVENTION.md`](../SOLDER-PASTE-CONVENTION.md).

```
TINKERROCKET PX100 GNSS CARRIER PROTOTYPE - FABRICATION AND ASSEMBLY NOTES
BOARD 28.3 x 30.8 mm, 2 mm CORNER RADIUS, 4 LAYER, 1.6 mm NOMINAL.

1. STACKUP: 4 LAYERS, 1.6 mm, 1 oz OUTER / 0.5 oz INNER, JLCPCB'S
   DEFAULT TEMPLATE JLC04161H-7628, AS V1 WAS BUILT. THE RF LINES WERE
   DRAWN FOR JLC04161H-3313, BUT ON THIS COPPER 7628 IS THE BETTER FIT:
   THE LINES ARE SHORT, AND J1'S PAD IS HALF AS CAPACITIVE.
   LAYERS: L1 PARTS + RF, L2 GND, L3 SIGNAL AND POWER, L4 GND.
   *** 1.6 mm: THE EDGE SMA (J1) IS RATED FOR 1.2-1.6 mm BOARDS. ***

2. SURFACE FINISH: ENIG (0.4 mm PITCH PX100 AND LNAs, SMALL SAW LANDS).

3. VIAS: 201 THROUGH VIAS, 0.40 mm PAD / 0.30 mm DRILL, EPOXY FILLED
   AND COPPER CAPPED (IPC-4761 TYPE VII), THEN TENTED BOTH SIDES.
   *** ORDER FILLED-AND-CAPPED VIAS (A PAID OPTION ON 4 LAYERS AT
   JLC). IT APPLIES TO EVERY VIA ON THE BOARD. ***
   FIVE VIAS ARE IN U1'S EXPOSED PAD (PIN 41, THE PX100'S ONLY GROUND),
   THE ONLY VIAS IN ANY PAD; THEIR CAPS MUST BE PLANAR AND SOLDERABLE.
   EVERY OTHER VIA IS AT LEAST 0.09 mm FROM ANY SOLDER MASK OPENING,
   EXCEPT ONE VDD_RF VIA 0.03 mm FROM C50'S PAD.

4. RF TRACES: 0.16 mm ON F.Cu, 0.20 mm GAP TO THE GROUND POUR, OVER
   In1.Cu GROUND: ABOUT 68 OHM ON 7628 (50 OHM ON 3313). DO NOT
   ADJUST THE WIDTHS.

ASSEMBLY - STENCIL
STENCIL FOIL 0.08 mm, FLAT. TOP STENCIL ONLY.
*** ORDER THE FOIL AS 0.08 mm. ***
TIGHTEST APERTURES: U3/U4 (TSNP) 0.25 mm ROUND, AREA RATIO 0.78 AT
0.08 mm; FL7 (MURATA 1109 SAW) 0.20 x 0.25 mm, AREA RATIO 0.69 AT
0.08 mm (0.56 AT 0.10 mm, BELOW THE 0.66 FLOOR).
U1'S EXPOSED PAD PRINTS AS A 2 x 2 WINDOWPANE (73 %), AS DRAWN.
L13 IS NOT FITTED BUT THE V1 STENCIL STILL HAS ITS TWO APERTURES.
PASTE ON THOSE LANDS IS FINE: IT REFLOWS INTO A BUMP ON EACH LAND
(0.38 mm APART, WITH MASK BETWEEN). AFTER REFLOW, CHECK THEY DIDN'T
BRIDGE: ONE LAND IS GROUND, AND A BRIDGE SHORTS FL7'S OUTPUT.

ASSEMBLY - REFLOW
ONE PASS, TOP SIDE ONLY.
THE SAW FILTERS SET THE CEILING: B4327 (FL6) PEAK 250 C +0/-5;
B8389 (FL4, FL5) 30-70 s ABOVE 220 C. THE PX100 ALLOWS 260 C PEAK,
60-150 s ABOVE 217 C. RUN PEAK 240-245 C, 60-70 s ABOVE 220 C.
FL6 IS MSL 2a; FL4/FL5 ARE MSL 3 AND ESD SENSITIVE (250 V HBM).
Y1 (EPSON TCXO): FOLLOW EPSON'S REFLOW AND STORAGE LIMITS.

ASSEMBLY - PLACEMENT
- U1 (PX100, QFN-40): PIN 1 AT THE SILK DOT.
- U3, U4: PLACE BY THE PART'S PIN-1 "X" MARK AGAINST PAD 1: U3'S PAD 1
  IS ITS LOWER-LEFT PAD ON THE BOARD (45.59, 38.07), U4'S ITS UPPER-LEFT
  PAD (46.95, 39.29). U4'S SILK TRIANGLE SITS ABOUT EQUALLY BETWEEN U4
  PIN 1 AND U3 PIN 3, SO DON'T GO BY THE TRIANGLE ALONE. A ROTATED PART
  PUTS 1.8 V ON ITS RF INPUT.
- FL4, FL5 (B8389) AND FL6 (B4327) SHARE ONE LAND PATTERN. CHECK THE
  MARKING: 865 ON FL4/FL5, 477 ON FL6. PIN 1 AT THE SILK DOT FOR EACH
  PART'S OWN ROTATION, BEFORE AND AFTER REFLOW. THE LAND IS NOT
  SYMMETRIC: TURNED 180 DEGREES, A FILTER'S PIN 1 STRADDLES ITS OUTPUT
  AND A GROUND LAND, WHICH KILLS L1 (FL6) OR BOTH BANDS (FL4, FL5).
- FL6: BEFORE FITTING, METER A LOOSE B4327 FROM THE SAME REEL. PAD 4
  TO PADS 1, 2, 3 AND 5 MUST READ OPEN. A SHORT WOULD GROUND U1 PIN 40
  AND LOSE BOTH BANDS (L5 NEEDS L1): FIT A DC-OPEN L1 SAW ON THE SAME
  LAND INSTEAD (README, FRONT END).
- FL7 (MURATA SAFFB1G17AA0F0A): PORT 1 IS THE LONE LAND NEXT TO THE
  SILK DOT; ON THE BOARD (ROTATED 90 DEGREES) THAT IS THE BOTTOM-CENTER
  LAND. THE LAND PATTERN IS NOT SYMMETRIC.
- Y1 (EPSON TG2520SMN): PIN 1 AT THE SILK DOT (BOTTOM LEFT). ITS LONG
  SIDE RUNS ALONG THE BOARD'S X AXIS ON PURPOSE (ACCELERATION
  SENSITIVITY); DO NOT ROTATE IT 90 DEGREES.
  V1 STAND-IN: EPSON TG2016SMN 26 MHz MCGNNM (2.0 x 1.6 mm), CENTERED
  ON THE SAME LANDS, PIN 1 AT THE DOT, LONG SIDE ALONG X. ITS
  TERMINALS SIT ON THE PADS' INNER CORNERS (ABOUT 88 % ON THE PAD);
  USE LESS PASTE THAN THE STENCIL PRINTS.
  DO NOT FIT THE EPSON TG-5006CG-12H: EPSON'S DOCUMENTS DISAGREE ON
  ITS PIN 1 (PAD 1 IS GROUND HERE), AND ITS -12H SPECIFICATION IS NOT
  PUBLISHED.
- U2 IS ADP7142ARDZ-3.3 ONLY; THE ADJUSTABLE ADP7142ARDZ REGULATES AT
  1.2 V ON THIS BOARD.
- D2 (SMF10A): BAND (CATHODE) AT THE VSYS END, AS THE SILK SHOWS. THE
  SYMBOL CAN'T SHOW POLARITY, AND VISHAY NUMBERS PIN 1 AS THE ANODE,
  THE REVERSE OF THE FOOTPRINT: MATCH THE BAND, NOT THE PIN NUMBER.
  BEFORE FIRST POWER, DIODE MODE, RED ON VSS AND BLACK ON VSYS READS
  0.6-0.8 V, AND OPEN THE OTHER WAY. REVERSED, D2 SHORTS THE PACK.
- L13: DO NOT FIT (SOLDER ON ITS EMPTY LANDS IS FINE). C37: 1.3 pF (MURATA
  GJM1555C1H1R3BB01D), NOT 4 pF. A FITTED L13 GROUNDS U1 PIN 37.
- C48 IS A 0 OHM JUMPER (0402WGF0000TCE) ON 0402 CAPACITOR PADS, NOT
  A CAPACITOR.
- SH1 (SHIELD FRAME), SH2 (SHIELD COVER) AND J1 (EDGE SMA) ARE NOT
  FITTED BY THE ASSEMBLER.

ASSEMBLY - HAND WORK
- SH1 (LAIRD BMI-S-202-F FRAME): FIT BY HAND AFTER THE RF BRING-UP AND
  ANY MATCH TUNING. ITS CROSS BRACE AND 6 mm PICK-UP DISC COVER L14,
  L15, C49, C53 AND U1 PINS 31-40. THE RING PADS ARE PRE-TINNED BY THE
  STENCIL AND SOLID TO A POUR STITCHED TO ALL FOUR LAYERS, AND L2, C23,
  C43, C38, C34, Y1 AND D6 SIT 0.43-0.55 mm FROM THE RING. PREHEAT THE
  BOARD FROM BELOW TO 150-180 C, THEN SOLDER THE RING WITH AN IRON.
  NEVER TOP-SIDE HOT AIR ALONE. BAKE FIRST IF THE SAWS ARE PAST THEIR
  MSL FLOOR LIFE. AFTERWARDS RE-INSPECT THOSE PARTS AND RE-CHECK THE
  CURRENT AND RAILS.
- SH2 (LAIRD BMI-S-202-C COVER): SNAP ON AFTER THE FIRST SKY TEST; IT
  COMES OFF AGAIN FOR REWORK.
- J1 (RF SOLUTIONS CON-SMA-EDGE-S): THE BOARD EDGE IS 1.57 mm OUTSIDE
  THE EDGE THE FOOTPRINT EXPECTS, SO WITH THE FLANGE ON THE EDGE THE
  CENTER PIN CROSSES ABOUT 0.5 mm OF MASK-COVERED GROUND POUR
  (y 27.0-27.5) BEFORE IT REACHES THE PAD (2.24 mm OF OVERLAP). CHECK
  THAT MASK AT 10x AND DAB UV SOLDER MASK ON IT IF IN DOUBT, KEEPING
  IT OFF THE PAD OPENING. SLIDE J1 ON SQUARELY WITHOUT PRESSING THE
  PIN DOWN, SEAT THE FLANGE ON THE EDGE, AND SOLDER THE CENTER PIN AND
  ALL FOUR GROUND LEGS, TOP AND BOTTOM. THEN J1 CENTER TO SHELL MUST
  READ OPEN (C20 BLOCKS DC; D6 LEAKS nA). REPEAT AFTER HEAVY USE.
```

## Order checklist

The gerbers don't carry these settings. Set them on JLC's order form and check them on the Product Detail before
paying, then paste the Product Detail into `gerbers/README.txt`:
- 4 layers, 1.6 mm, 1 oz outer / 0.5 oz inner;
- Specify Stackup: leave JLC's default (JLC04161H-7628) for these V1 gerbers. The RF lines were drawn for 3313, but
  J1's land is a 2.1 pF shunt on 3313 against about 1.1 pF on 7628, and the review's model puts the board as drawn
  about 1 dB better at L1 and 0.4 dB at L5 on 7628. Choose the stackup deliberately on V2, with the J1 launch redrawn;
- Surface Finish: ENIG;
- Via Covering: Epoxy Filled & Capped (a paid option on 4 layers; U1's exposed pad needs it);
- stencil: 0.08 mm, top.

## V1 HASL boards

How to build the boards as ordered (HASL with lead, plugged vias). Stage-1-only builds need only steps 2 and 3.
1. **U1's exposed pad, before paste.** Inspect the five vias at 10-20x. Flush or slightly dimpled: build as is.
   Solder domes over the holes: level them with flux and braid. Open holes: seal them from the top with a little solder,
   or UV-cure mask scraped flush (about 5 % of the pad), then level.
2. **Flatten the HASL on the fine-pitch lands** with flux and solder braid, leaving a thin flat tin layer: U1's
   perimeter and exposed pad, U3, U4, FL4-FL7 and Y1. Clean with IPA, then print the 0.08 mm stencil as usual.
3. **After reflow.** Look for bridges on U1 at 10-20x, and at U3, U4 and FL7 from the side for tilt and fillets. U1 must
   sit flat with even fillets. Check U1's exposed-pad joint electrically: diode mode, red probe on board GND, black on
   a U1 I/O (TXD1, at R8's U1-side pad) reads a 0.4-0.8 V clamp drop; OL means the pad isn't joined.
4. The VDD_RF via 0.025 mm from C50 pad 1 was accepted because the vias would be filled; on these boards it is open on
   top. Check C50 pad 1's joint after reflow.

FL1 (DLW21SN) on an ENIG re-order: Murata's land note warns against a gilded land. It doesn't apply to the HASL boards.

## Sourcing for the two V1 builds

Checked 2026-10-23. LCSC stocks every part except those in the table, and four BOM lines go to in-stock equivalents
there: C20/C21/C26 GRM1555C1H330JA01D (none in stock) to GRM1555C1H330GA01D (C162097, 2 % instead of 5 %); FL1
DLW21SN670HQ2L (too few) to DLW21SN670SQ2L (C113043, the same 67 Ω, rated 400 mA and 250 mΩ instead of 320 mA and
310 mΩ); the 25 V 1 µF CL05A105KA5NQNC on C40, C44, C47, C49, C50, C53, C55 and C56 (backordered) to the 16 V
CL05A105KO5NNNC already on C27/C28 (C29266; every one of those capacitors sits on 3.3 V or less); the 10 kΩ
0402WGF1002TCE on R12, R14 and R15 (backordered) to the RC0402FR-0710KL already on R1/R3 (C60490).

Changed 2026-10-26: C37 is now Murata GJM1555C1H1R3BB01D, 1.3 pF ±0.1 pF (LCSC C161309, 17,589 in stock), replacing
GJM1555C1H4R0BB01D (C161323). L13 is not fitted; its part, LQW15AN6N8G8ZD (C2043812), is shared with L10 and L11,
so that line needs two per board instead of three. If the LCSC order isn't placed yet, swap the C37 line and cut the
6.8 nH line; if the parts have already arrived, the spare inductor stays in the bag.

Also changed 2026-10-26: C48 is the UNI-ROYAL 0402WGF0000TCE 0 Ω jumper (LCSC C17168, a JLC basic part). It replaces
GRM1555C1H102JA01D (C76947), which C22–C25 still use, so that 1 nF line drops from five per board to four. C48 stays
0 Ω unless the scope at U1 pin 4 shows a bad clock (README *Bring-up*, step 2); the fallback then is 10 nF, Epson's
test value (for example GRM155R71H103KA88D), not 1 nF.

**Parts for the 2026-10-27 review's changes** (none is in the schematic yet; README *Review of 2026-10-27*):

| Use | Part | Note |
|---|---|---|
| L1, same lands, optional | Murata LQW15AN3N3B80D, 3.3 nH 0402 | not needed on 7628 (about 0.2 dB at L1 in the model); worth about 0.4 dB at each band on a 3313 board; never 2.4 nH on 7628 |
| R7 and R14, same lands | 10 kΩ (RC0402FR-0710KL, on the BOM) on R7, 100 kΩ 0402 on R14 | change both together, never R7 alone |
| U1 pin 14 bodge, if the pin measures as a supply node | CL05A225MQ5NSNC 2.2 µF 0402 (on the BOM) plus 0.1 mm enameled wire | |
| U3 VCC bodge, if the sniff finds a line | 0201 1 nF, C0G or X7R | stood upright between U3's VCC trace and C25 pad 2 |
| FL6 DC block, only if pin 40 carries DC | GRM1555C1H330 33 pF 0402 (on the BOM) | across a cut in FL6-OUT between C36 and L14 |
| FL6 replacement, only if a loose B4327 meters short | SkyTraq/Tai-Saw TA1573A (3 V DC rating) or Golledge MP05419 | meter the MP05419 first; it is 0 V-rated too |

| Ref | Part | Source |
|---|---|---|
| U1 | SkyTraq PX100-5C-01A | SkyTraq; LCSC does not list it |
| U2 | Analog Devices ADP7142ARDZ-3.3 | DigiKey 505-ADP7142ARDZ-3.3-ND (352 in stock) |
| U3, U4 | Infineon BGA855N6E6327XTSA1 | DigiKey BGA855N6E6327XTSA1CT-ND (28,517 in stock) |
| J1 | RF Solutions CON-SMA-EDGE-S | DigiKey CON-SMA-EDGE-S-ND (13,162 in stock) |
| SH1 | Laird BMI-S-202-F frame | DigiKey 903-1051-1-ND (59,465 in stock) |
| SH2 | Laird BMI-S-202-C cover | DigiKey 903-1014-ND (44,644 in stock) |
| Y1 | Epson TG2520SMN 26.000000MHz MCGNNM | Not at LCSC; fit the TG2016SMN below |

Y1 has two in-stock stand-ins at LCSC, both Epson and both ±0.5 ppm:

- **TG2016SMN 26 MHz MCGNNM** (X1G0054410304, LCSC C7527385): the TG2520SMN's own family and data sheet, so the same
  2.66–3.465 V supply, −40…85 °C range and acceleration data, and pin 1 may be open or grounded. It is 2.0 × 1.6 mm;
  centered on the 2.5 × 2.0 lands, about 88 % of each terminal sits on its pad, at the pad's inner corner. Fit this one.
- **TG-5006CG-12H** (X1G0042110003, LCSC C1998253): the TG2520SMN's size, pinout and lands, 3.0 V ±5 %, −30…85 °C,
  no published acceleration data. Don't fit it. Its part sheet says pin 1 (N.C.) must not be connected, Epson's
  TG-5006 series brief says N.C. may be open or grounded, and the -12H specification itself is unpublished. This board
  grounds pad 1.
