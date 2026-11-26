# gnss-px100-18mm-hv — fabrication and assembly notes

**V2.** Schematic and layout of 2026-11-03 (gerbers of that date are in `gerbers/`); the first V2 board was built and
brought up on 2026-11-20. These notes were rewritten for the V2 parts on 2026-11-20, from the board file, the
schematic and the parts' data sheets. The V1 notes are kept in [`FABRICATION-NOTES-V1.md`](FABRICATION-NOTES-V1.md);
they describe parts V2 no longer has, so don't build a V2 board from them.

**What V2 changed.** Only the front end: the two TSNP LNAs, the two B8389 filters and their matching parts are gone.
In their place are the Bourns diplexer `FL3`, one leaded LNA per band (`U3` AT2659S on L1, `U4` MAX2640 on L5), the
band filters `FL6` B9621 and `FL7` TA2471A, the antenna-supply parts (`L20`, `C60`, `R20`, `JP1`), `R23` and the LNA
supply capacitors, and 19 test pads on the back. `C37` is 4 pF again and there is no `L13`. The receiver core
(`U1`, `Y1`, `Y2`, `U2`) is unchanged.

**V2 order as placed: not recorded.** The repo doesn't say what finish, via covering and stackup the V2 boards were
ordered with. The V1 boards were HASL with plugged vias on JLC's default 7628 stackup, not what notes 1-3 ask for.
Paste JLC's Product Detail for the V2 order here. If the V2 boards are HASL too, follow *V1 HASL boards* in the V1
notes: seal and level the vias in U1's exposed pad and flatten the fine-pitch lands before printing paste.

Stencil thickness, the area-ratio floor and the paste-coverage convention are repo-wide in
[`hardware/SOLDER-PASTE-CONVENTION.md`](../SOLDER-PASTE-CONVENTION.md).

```
TINKERROCKET PX100 GNSS CARRIER PROTOTYPE V2 - FABRICATION AND ASSEMBLY NOTES
BOARD 28.3 x 30.8 mm, 2 mm CORNER RADIUS, 4 LAYER, 1.6 mm NOMINAL.
COORDINATES ARE THE BOARD FILE'S, IN mm. "UP", "LEFT" AND "RIGHT"
ARE WITH THE PARTS FACING YOU AND J1, THE ANTENNA CONNECTOR, AT THE
TOP.

1. STACKUP: 4 LAYERS, 1.6 mm, 1 oz OUTER / 0.5 oz INNER.
   LAYERS: L1 PARTS + RF, L2 GND, L3 SIGNAL AND POWER, L4 GND AND
   THE TEST PADS.
   THE RF LINES ARE DRAWN FOR JLC04161H-3313 (50 OHM). ON JLC'S
   DEFAULT JLC04161H-7628 THEY ARE ABOUT 68 OHM; THE RUNS ARE SHORT,
   SO BOTH WORK. J1'S LAND SITS OVER SOLID L2 GROUND (V2 DID NOT VOID
   IT): ABOUT 2.1 pF ON 3313, ABOUT 1.1 pF ON 7628.
   *** 1.6 mm: THE EDGE SMA (J1) IS RATED FOR 1.2-1.6 mm BOARDS. ***

2. SURFACE FINISH: ENIG (0.4 mm PITCH PX100, SMALL FILTER LANDS).

3. VIAS: 259 THROUGH VIAS, 0.40 mm PAD / 0.30 mm DRILL, EPOXY FILLED
   AND COPPER CAPPED (IPC-4761 TYPE VII), THEN TENTED BOTH SIDES.
   *** ORDER FILLED-AND-CAPPED VIAS (A PAID OPTION ON 4 LAYERS AT
   JLC). IT APPLIES TO EVERY VIA ON THE BOARD. ***
   FIVE VIAS ARE IN U1'S EXPOSED PAD (PIN 41, THE PX100'S ONLY
   GROUND); THEIR CAPS MUST BE PLANAR AND SOLDERABLE. THREE MORE ARE
   IN BACK-SIDE LANDS THAT GET NO PASTE (JP1 PAD 3, R21 PAD 1, R22
   PAD 1).

4. RF TRACES: 0.16 mm ON F.Cu, 0.20 mm GAP TO THE GROUND POUR, OVER
   In1.Cu GROUND. DO NOT ADJUST THE WIDTHS.

ASSEMBLY - STENCIL
STENCIL FOIL 0.08 mm, FLAT. TOP STENCIL ONLY.
*** ORDER THE FOIL AS 0.08 mm. ***
TIGHTEST APERTURE: FL7 (1.1 x 0.9 mm SAW) 0.20 x 0.25 mm, AREA RATIO
0.69 AT 0.08 mm (0.56 AT 0.10 mm, BELOW THE 0.66 FLOOR). NEXT: FL6
0.30 x 0.375 mm (1.04), U1'S PERIMETER LANDS 0.22 x 0.83 mm (1.09),
D6 0.30 x 0.50 mm (1.17).
U1'S EXPOSED PAD PRINTS AS A 2 x 2 WINDOWPANE (73 %), AS DRAWN.
C71 IS NOT FITTED BUT THE STENCIL HAS ITS TWO APERTURES. PASTE ON
THOSE LANDS IS FINE: IT REFLOWS INTO A BUMP ON EACH LAND (0.40 mm
APART). AFTER REFLOW, CHECK THEY DIDN'T BRIDGE: ONE LAND IS GROUND,
THE OTHER IS U4'S OUTPUT SIDE, AFTER C70.

ASSEMBLY - REFLOW
ONE PASS, TOP SIDE ONLY. PLAN ON ONE BAKE: FL7 ALLOWS TWO REFLOWS IN
ALL, FL3 THREE.
THE FILTERS SET THE CEILING.
- FL6 (B9621): PEAK 250 C +0/-5, 30-70 s ABOVE 220 C, AT MOST 20 s
  ABOVE 245 C, NEVER 255 C, RAMP AND COOLING 3 K/s OR SLOWER.
- FL7 (TA2471A): PREHEAT 150-180 C FOR 60-90 s, 50-80 s AT 220 C,
  PEAK 245-260 C.
- FL3 (FDP2009N122157H4): BELOW 260 C.
- THE PX100 ALLOWS 260 C PEAK, 60-150 s ABOVE 217 C.
RUN PEAK 245-250 C, 50-70 s ABOVE 220 C.
FL6 AND FL7 ARE MSL 2a (FOUR WEEKS OUT OF THE BAG AT 30 C / 60 % RH,
THEN BAKE BEFORE REFLOW). FL3 IS MSL 1.
ESD: FL7 IS RATED 100 V HUMAN-BODY, 50 V MACHINE MODEL; FL6 225 V
AND 50 V. HANDLE BOTH, AND THE LNAs, GROUNDED.
Y1 (EPSON TCXO): FOLLOW EPSON'S REFLOW AND STORAGE LIMITS.

ASSEMBLY - PLACEMENT
73 PARTS GO THROUGH THE OVEN, ALL ON THE TOP SIDE (LIST BELOW).
NOT FITTED: C71 (TOP), R21 AND R22 (BACK). NOT IN THE OVEN: SH1,
SH2, J1 (HAND WORK).
- U3 AND U4 ARE BOTH SOT-23-6 AND ARE NOT INTERCHANGEABLE. U3 IS THE
  AT2659S (L1), AT (46.86, 40.49), BELOW THE DIPLEXER. U4 IS THE
  MAX2640 (L5, TOP MARK AAAV), AT (49.89, 37.32), UP AND RIGHT OF U3.
  PIN 1 IS THE UPPER-RIGHT PAD OF EACH, AT THE SILK TRIANGLE: U3
  (47.81, 39.35), U4 (50.84, 36.18). THE AT2659S HAS A DOT AT PIN 1.
  ON THE FIRST V2 BOARD THE "+" OF THE MAX2640'S TOP MARK SAT AT
  PIN 1.
  SWAPPED, OR EITHER ONE TURNED 180 DEGREES, A GROUND PIN LANDS ON
  THE LNA SUPPLY AND SHORTS IT: METER VLNA TO GND BEFORE POWER
  (CHECKS BELOW).
- 0402 INDUCTORS: FIVE VALUES, NO MARKINGS. PLACE ONE VALUE AT A
  TIME. L20 (47 nH, ANTENNA SUPPLY CHOKE) IS AT THE UPPER LEFT,
  OUTSIDE THE SHIELD RING, AT (44.83, 31.39). L22 (5.6 nH, U4'S
  INPUT) IS INSIDE THE RING, RIGHT OF C66, AT (50.57, 34.78). THOSE
  TWO SWAPPED LEAVES L5 WITH ABOUT -1 dB OF LNA GAIN AND A 9 dB NOISE
  FIGURE, AND AN ACTIVE ANTENNA HIDES IT. THE CHECKS BELOW CATCH IT.
  L21 7.5 nH, L14 8.2 nH AND L15 12 nH ARE FORGIVING: ANY OF THEM AT
  L22 COSTS UNDER 2 dB OF GAIN.
- FL3 (BOURNS DIPLEXER): THE DOT ON THE PART IS AT ONE SHORT EDGE,
  BETWEEN TERMINALS 1 AND 6. PUT THAT EDGE ON THE RIGHT, FACING C66;
  THE SILK DOT IS AT THE UPPER RIGHT (47.66, 34.02). TURNED 180
  DEGREES, THE COMMON PORT LANDS ON A GROUND LAND AND BOTH BANDS ARE
  LOST.
- FL6 (B9621, 1.4 x 1.1 mm, MARKED 9CN OVER A LOT CODE): PAD 1 IS THE
  LONE LAND ON THE LEFT (46.27, 45.35), NEXT TO THE SILK DOT. THE DOT
  ON THE PART MARKS PAD 1 AND GOES AT THE LOWER LEFT. THE LAND IS NOT
  SYMMETRIC. THE PART IS RATED 0 V DC; C64 AND C65 KEEP DC OFF IT.
- FL7 (TA2471A, 1.1 x 0.9 mm, MARKED "k" AND A DATE LETTER, NO DOT):
  PAD 1 (INPUT) IS THE LONE LAND AT THE TOP (49.25, 42.14), NEXT TO
  THE SILK DOT. ON THE PART, PIN 1 IS THE EDGE NEAREST THE "k". PLACE
  IT WITH THE "k" UP, AT THE SILK DOT, AND THE DATE LETTER BELOW IT.
  THE LAND IS NOT SYMMETRIC.
- U1 (PX100, QFN-40): PIN 1 AT THE SILK DOT, LOWER LEFT (PAD 1 AT
  (54.72, 42.73)).
- Y1 (EPSON TG2520SMN): PIN 1 AT THE SILK DOT, LOWER LEFT (PAD 1 AT
  (54.18, 47.45)). ITS LONG SIDE RUNS ALONG THE BOARD'S X AXIS ON
  PURPOSE (ACCELERATION SENSITIVITY); DO NOT ROTATE IT 90 DEGREES.
  STAND-INS: SEE "Y1 OPTIONS" BELOW.
- U2 IS ADP7142ARDZ-3.3 ONLY; THE ADJUSTABLE ADP7142ARDZ REGULATES AT
  1.2 V ON THIS BOARD. PIN 1 IS THE UPPER-RIGHT PAD (40.86, 43.99),
  AT THE SILK DOT.
- D2 (SMF10A): BAND (CATHODE) AT THE VSYS END, THE UPPER END (PAD 1
  AT (47.94, 53.15)), AS THE SILK BAR SHOWS. THE SYMBOL CAN'T SHOW
  POLARITY, AND VISHAY NUMBERS PIN 1 AS THE ANODE, THE REVERSE OF THE
  FOOTPRINT: MATCH THE BAND, NOT THE PIN NUMBER. REVERSED, D2 SHORTS
  THE SUPPLY.
- D7 (NSR0320, SOD-323): BAND (CATHODE) ON THE RIGHT (PAD 1 AT
  (56.41, 34.37), THE DCDC_OUT SIDE), AT THE SILK BAR.
- D1, D3 (GREEN LEDs): CATHODE AT THE SILK DOT: D1'S LOWER END, D3'S
  RIGHT END.
- D6 (TPD1E0B04) IS BIDIRECTIONAL: EITHER WAY ROUND.
- C48 IS A 0 OHM JUMPER (0402WGF0000TCE) ON 0402 CAPACITOR PADS, NOT
  A CAPACITOR.
- C37 IS 4 pF (GJM1555C1H4R0BB01D) IN V2, NOT V1'S 1.3 pF.

ASSEMBLY - HAND WORK, AFTER THE BAKE
- J1 (RF SOLUTIONS CON-SMA-EDGE-S): THE BOARD EDGE IS 1.57 mm OUTSIDE
  THE EDGE THE FOOTPRINT EXPECTS, SO WITH THE FLANGE ON THE EDGE THE
  CENTER PIN CROSSES ABOUT 1.2 mm OF MASK-COVERED GROUND POUR
  (y 26.35-27.5) BEFORE IT REACHES THE PAD (2.24 mm OF OVERLAP). CHECK
  THAT MASK AT 10x AND DAB UV SOLDER MASK ON IT IF IN DOUBT, KEEPING
  IT OFF THE PAD OPENING. SLIDE J1 ON SQUARELY WITHOUT PRESSING THE
  PIN DOWN, SEAT THE FLANGE ON THE EDGE, AND SOLDER THE CENTER PIN AND
  ALL FOUR GROUND LEGS, TOP AND BOTTOM. THEN J1 CENTER TO SHELL MUST
  READ OPEN (C20 BLOCKS DC; D6 LEAKS nA; JP1 OPEN).
- JP1 (BACK, THREE PADS): LEAVE OPEN FOR A PASSIVE ANTENNA. BRIDGE
  2-3 TO FEED +3V3 TO AN ACTIVE ANTENNA THROUGH R20 AND L20; 1-2
  USES THE PX100'S ANT_BIAS OUTPUT INSTEAD (45 mA LIMIT).
- U4 SUPPLY CAPACITOR (NOT ON THE BOARD: U4'S NEAREST SUPPLY
  CAPACITOR IS 12 mm OF TRACE AND TWO VIAS AWAY, AND THE MAX2640
  DATA SHEET WANTS ONE AT THE PIN). AFTER THE FIRST CURRENT READING,
  SOLDER A 100 pF 0402 (GRM1555C1H101JA01D, THE C62/C66/C68 PART)
  ACROSS THE OUTER ENDS OF U4 PADS 6 AND 5: THE TWO RIGHT-HAND PADS
  OF U4'S LOWER ROW, AT (50.84, 38.46) AND (49.89, 38.46), JUST BELOW
  THE LEAD TOES. PAD 6 IS THE LNA SUPPLY AND PAD 5 IS GROUND, SO
  METER VLNA TO GND FOR A SHORT BEFORE POWER, THEN READ THE CURRENT
  AGAIN.
- SH1 (LAIRD BMI-S-202-F FRAME): FIT LAST, BY HAND, AFTER THE RF
  CHECKS AND ANY REWORK; THE PARTS INSIDE ARE HARD TO REACH ONCE IT IS
  ON (L22 IS 1.3 mm FROM THE TOP WALL'S LAND). ITS 6 mm CENTER DISC
  SITS OVER L15, C37, C49, C53, C72, C76, C77 AND U1 PINS 31-40, AND
  THE BRACE THAT CARRIES IT COVERS MORE.
  THE RING LANDS ARE PRE-TINNED BY THE STENCIL AND SOLID TO A POUR
  STITCHED TO ALL FOUR LAYERS. C20, C38, C43, C61, C64, D6 AND Y1
  INSIDE THE RING, AND C74, L20, R11 AND Y2 OUTSIDE IT, SIT
  0.41-0.58 mm FROM THE RING. PREHEAT THE BOARD FROM BELOW TO
  150-180 C, THEN SOLDER THE RING WITH AN IRON. NEVER TOP-SIDE HOT
  AIR ALONE, AND KEEP THE FILTERS BELOW SOLDER MELT: FL7 HAS ONE
  REFLOW LEFT AFTER THE BAKE. AFTERWARDS RE-INSPECT THOSE PARTS AND
  RE-CHECK THE CURRENT AND RAILS.
- SH2 (LAIRD BMI-S-202-C COVER): SNAP ON AFTER THE SKY TESTS; IT
  COMES OFF AGAIN FOR REWORK.
```

## Checks after the bake

Everything goes on in one pass, so these replace part-by-part checks. The pads named are the test pads on the back,
by their silkscreen labels.

**Power off.** Touch the probes together first and subtract that reading.

| Check | Probe | Expect | A wrong reading means |
|---|---|---|---|
| LNA supply not shorted | VLNA to GND | not a short; the reading climbs as the capacitors charge | near 0 Ω: U3 and U4 swapped or one turned 180°, or a bridge |
| R23 | 3V3 to VLNA | 10 Ω | wrong part at R23 |
| L20 is the 47 nH | AFEED to J1's center pin | about 1 Ω (1.08 Ω max) | near 0 Ω: a small inductor is at L20, and the 47 nH is probably at L22 |
| L22 is the 5.6 nH | across L22's two pads | same as shorted probes (0.04 Ω max) | 0.7-1.1 Ω: the 47 nH is here |
| R20 | ASUP to AFEED | 10 Ω | wrong part at R20 |
| Antenna input | J1 center pin to shell | open | a bridge at J1, D6 or C20, or JP1 not open |
| D2 the right way round | diode mode, red on VSS, black on VSYS | 0.6-0.8 V, and open the other way | D2 reversed: it would short the supply |
| C71's empty lands | look, at 10x | two separate bumps | a bridge grounds U4's output |

Also look, at 10x, for tombstoned or skewed 0402 parts along the L5 chain (`C66`, `L22`, `C70`, `C72`, `L15`,
`C37`), for both LNAs' leads wetted at the heel, and for `FL3`, `FL6` and `FL7` sitting flat and square. The joints
under the two SAW filters can't be seen; the active-antenna run below is their test.

**First power.** Use a current-limited supply if there is one.

| Pad | Expect |
|---|---|
| VSYS to VSS | the supply, 3.7-8.4 V |
| 3V3 | 3.3 V |
| VLNA | about 3.2 V |
| VRF, VDD11 | 1.1 V |
| VCORE | about 1.3 V |
| VTCXO | 3.0 V |
| RST, BOOT | high |

- **LNA current** is (3V3 − VLNA) / 10 Ω. The data sheets give 7.9 mA typical and 9.8 mA at most for the pair: the
  AT2659S draws 3.8 / 4.4 / 5.1 mA (min / typ / max at 2.85 V), the MAX2640 3.5 mA typical and 4.7 mA at most at
  25 °C. The first V2 board read 10.4 mA, without the U4 supply capacitor.
- **Then the U4 supply capacitor** (hand work above), and the current again. A change of more than about 0.5 mA
  means the stage was not stable without it.
- **To tell which LNA is off** when the current is wrong, compare DC voltages with a known board at `C64`'s U3-side
  pad and `L21` (U3's output and input) and at `C70`'s U4-side pad and `L22` (U4's). The AT2659S data sheet gives
  0.83 V at its input.
- **Receiver:** 115200 baud, NMEA by default. Log the firmware version before comparing boards; the first V2 board
  reports kernel 3.6.0, ODM 1.7.33.
- **RF, active antenna first** (JP1 2-3): both bands tracking proves the hidden joints under `FL3`, `FL6` and
  `FL7`. The first V2 board reached 49 dB-Hz on L1 and 47 on L5 with an outdoor active antenna.
- **Then the passive antenna** (JP1 open), back-to-back with another board on the same antenna and the same
  satellites. Keep phones and the 915 MHz radio away: nothing filters U4's input below 1.3 GHz.

## Y1 options

- **Epson TG2520SMN 26.000000 MHz MCGNNM** is the design part (2.5 × 2.0 mm).
- **Epson TG2016SMN 26 MHz MCGNNM, X1G0054410304** (LCSC C7527385) can replace it. It is the TG2520SMN's own
  family and data sheet: the same 2.66-3.465 V supply range (`VDD_TCXO` is 3.0 V), ±0.5 ppm, −40 to +85 °C, the same
  pinout and acceleration data, and its pin 1 may be open or grounded (this board grounds pad 1). It is 2.0 × 1.6 mm:
  center it on the 2.5 × 2.0 lands, pin 1 at the dot, long side along X. About 88 % of each terminal sits on its
  pad, at the pad's inner corner, so use less paste than the stencil prints and check after the bake that it hasn't
  turned or slid.
- **Epson TG-5006CG-12H, X1G0042110003** (LCSC C1998253): don't fit it as the board is. It has the TG2520SMN's
  size, pinout and lands and the right 3.0 V supply, but its part sheet says pin 1 (N.C.) must not be connected to
  anything and this board grounds pad 1; it is specified from −30 °C, not −40 °C; and it has no published
  acceleration data, which is why Y1 was chosen. If it has to be used, keep pad 1 free of paste and insulated so
  pin 1 floats.

## V2 parts through the oven

From the schematic on 2026-11-20: 73 parts, top side. Not in this list: `J1`, `SH1`, `SH2` (hand work), `C71`, `R21`,
`R22` (not fitted), `JP1` and the test pads (copper only).

| Qty | Value | Part number | Package | References | LCSC |
|---|---|---|---|---|---|
| 1 | PX100 | PX100-5C-01A | QFN-40 | U1 | SkyTraq only |
| 1 | 3.3 V LDO | ADP7142ARDZ-3.3 | SOIC-8 EP | U2 | DigiKey |
| 1 | L1 LNA | AT2659S | SOT-23-6 | U3 | C883443 |
| 1 | L5 LNA | MAX2640EUT+T | SOT-23-6 | U4 | C967082 |
| 1 | diplexer | FDP2009N122157H4 | 2.0 × 1.25 mm | FL3 | Mouser, DigiKey |
| 1 | L1 SAW | B39162B9621P810 | 1.4 × 1.1 mm | FL6 | C5556163 |
| 1 | L5 SAW | TA2471A | 1.1 × 0.9 mm | FL7 | C5975435 |
| 1 | common-mode choke | DLW21SN670HQ2L | 0805 | FL1 | |
| 1 | ferrite bead | BLM18PG471SN1D | 0603 | FL2 | |
| 1 | 26 MHz TCXO | TG2520SMN 26.000000MHz MCGNNM | 2.5 × 2.0 mm | Y1 | see *Y1 options* |
| 1 | 32.768 kHz | SC-32S-32.768kHz-20PPM-12.5pF | 3.2 × 1.5 mm | Y2 | C97606 |
| 2 | green LED | XL-1005UGC | 0402 | D1, D3 | |
| 1 | TVS | SMF10A | SMF | D2 | |
| 1 | ESD diode | TPD1E0B04DPYR | 0.6 × 1.0 mm | D6 | C1975594 |
| 1 | Schottky | NSR0320MW2T1G | SOD-323 | D7 | C48192 |
| 1 | 5.6 nH | LQW15AN5N6G80D | 0402 | L22 | C3223887 |
| 1 | 7.5 nH | LQW15AN7N5G00D | 0402 | L21 | C82918 |
| 1 | 8.2 nH | LQW15AN8N2G80D | 0402 | L14 | C307610 |
| 1 | 12 nH | LQW15AN12NG80D | 0402 | L15 | C3223181 |
| 1 | 47 nH | LQW15AN47NG00D | 0402 | L20 | C22334 |
| 1 | 2.2 µH | LQM18PN2R2MFRL | 0603 | L16 | C337910 |
| 1 | 0 Ω jumper | 0402WGF0000TCE | 0402 | C48 | C17168 |
| 1 | 3 pF | GJM1555C1H3R0BB01D | 0402 | C36 | C88891 |
| 1 | 3.9 pF | GJM1555C1H3R9BB01D | 0402 | C70 | C76907 |
| 1 | 4 pF | GJM1555C1H4R0BB01D | 0402 | C37 | C161323 |
| 4 | 33 pF | GRM1555C1H330JA01D | 0402 | C20, C64, C65, C72 | C76964 |
| 3 | 100 pF | GRM1555C1H101JA01D | 0402 | C62, C66, C68 | C77177 |
| 1 | 470 pF | GRM1555C1H471JA01D | 0402 | C61 | C76971 |
| 1 | 2.2 nF | CC0402KRX7R9BB222 | 0402 | C7 | |
| 3 | 100 nF | CL05B104KO5NNNC | 0402 | C3, C63, C69 | C1525 |
| 8 | 1 µF 25 V | CL05A105KA5NQNC | 0402 | C40, C44, C47, C49, C50, C53, C55, C56 | C52923 |
| 5 | 1 µF 16 V | CL05A105KO5NNNC | 0402 | C60, C73, C76, C77, C78 | C29266 |
| 6 | 2.2 µF | CL05A225MQ5NSNC | 0402 | C38, C41, C42, C43, C46, C74 | C12530 |
| 1 | 4.7 µF | CL05A475MP5NRNC | 0402 | C39 | C23733 |
| 3 | 22 µF | CL21A226MOQNNNE | 0805 | C1, C135, C136 | |
| 1 | 5.1 Ω | RC0603FR-075R1L | 0603 | R11 | C114626 |
| 2 | 10 Ω | RC0402FR-0710RL | 0402 | R20, R23 | C138066 |
| 2 | 1 kΩ | RC0402FR-071KL | 0402 | R8, R9 | |
| 1 | 1 kΩ | 0402WGF1001TCE | 0402 | R13 | C11702 |
| 3 | 10 kΩ | RC0402FR-0710KL | 0402 | R1, R3, R7 | |
| 2 | 10 kΩ | 0402WGF1002TCE | 0402 | R12, R15 | C25744 |
| 1 | 100 kΩ | RC0402FR-07100KL | 0402 | R14 | C60491 |

The 16 V 1 µF (C29266) is an accepted stand-in for the 25 V one; every one of those capacitors sits on 3.3 V or
less.

**Stock at LCSC on 2026-11-20:** AT2659S 20,360; MAX2640EUT+T 204; B39162B9621P810 250; TA2471A none;
X1G0054410304 (the Y1 stand-in) 3,024. LCSC doesn't list the PX100 or the FDP2009N122157H4. DigiKey has `U2`
(505-ADP7142ARDZ-3.3-ND), `J1` (CON-SMA-EDGE-S-ND), `SH1` (903-1051-1-ND) and `SH2` (903-1014-ND).

## Order checklist

The gerbers don't carry these settings. Set them on JLC's order form and check them on the Product Detail before
paying, then paste the Product Detail at the top of these notes:
- 4 layers, 1.6 mm, 1 oz outer / 0.5 oz inner;
- Specify Stackup: choose it on purpose. JLC04161H-3313 gives 50 Ω lines and a 2.1 pF J1 land; the default
  JLC04161H-7628 gives 68 Ω lines and a 1.1 pF land. For V1's layout the review's model put 7628 slightly ahead,
  mainly because of the J1 land; it has not been re-run for V2;
- Surface Finish: ENIG;
- Via Covering: Epoxy Filled & Capped (a paid option on 4 layers; U1's exposed pad needs it);
- stencil: 0.08 mm, top.

## Sources for the V2 part data

Checked 2026-11-20 against the parts' own sheets: Maxim MAX2640/MAX2641 (19-1384 rev 2), ZHONGKEWEI AT2659S, Bourns
FDP2009N122157H4 (rev 07/26), RF360 B9621 (version 2.3, 2019-06-17), Tai-Saw TA2471A (rev 2.0), and Murata's
LQW15AN listings for the inductor resistances. The Y1 stand-in notes are from the 2026-10-23 sourcing check and
Epson's TG2016SMN data sheet. Positions, lands and distances are from the board file.
