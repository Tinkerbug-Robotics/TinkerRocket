# Tinker-Mantis (rocket computer) — fabrication and assembly notes

**Paste block A into the bare-board fab's order notes verbatim. Paste block B into the assembler's instructions.**

This file exists for the same reason the LoRa board's does: **none of it can travel in the design files.** Via protection, the stencil foil and the process limits are absent from the gerbers, the `.gbrjob`, the Excellon drill file, IPC-2581 and ODB++. The P4 silicon revision (B8) is worse than absent: no file can express it at all, and the wrong part cannot be reworked off the board.

Board: **22.35 × 75.00 mm, 8 layer, 1.630 mm stack**, rev V10. F / In1 / In2 / In3 / In4 / In5 / In6 / B, with In1 and In6 solid GND, In3 the +3V3 plane, In4 the switched 3.3 V plane (`V_MCU_SWTCH`), and In2/In5 routing (In5 also carries a VBATT pour). Two internal 8.0 × 3.0 mm slots are routed as part of the outline.

> **Mirrored onto `User.Drawings`** (the board's `Dwgs.User` layer, rewritten for V10 on 2026-09-30). The board's saved plot set does not select that layer, so a `kicad-cli pcb export gerbers --board-plot-params` plot does not ship it. The fab package's `tinker-mantis-fabrication-notes.txt` is block A verbatim, and the order notes are the channel. `tools/plot_gerbers.sh` passes no layer list, so it does emit `User_Drawings.gbr`.

---

## Block A — bare-board fabrication

```
TINKERROCKET TINKER-MANTIS (ROCKET COMPUTER V10) - FABRICATION NOTES
BOARD 22.35 x 75.00 mm, 8 LAYER, 1.630 mm STACK.
TWO INTERNAL SLOTS 8.0 x 3.0 mm, ROUTED; THEY ARE PART OF THE OUTLINE.

A1. STACKUP: JLCPCB JLC08161H-2116, EIGHT LAYER, 1.630 mm.
    1 oz OUTER / 0.5 oz INNER. BUILD TO THIS TEMPLATE.
    OUTER PREPREG 2116 AT 0.1164 mm (er 4.16), CORES 0.300 mm
    (er 4.41), INNER PREPREG 1080 x2 AT 0.1528 mm (er 3.91),
    ALL Nan Ya NP-155F. THE BUILD IS SYMMETRIC.
    LAYERS: F SIGNAL / In1 GND / In2 SIGNAL / In3 +3V3 PLANE /
    In4 SWITCHED 3V3 PLANE / In5 SIGNAL / In6 GND / B SIGNAL.
    NOTE: JLC PUBLISH 4- AND 6-LAYER IMPEDANCE TEMPLATES ONLY;
    CONFIRM THIS 8-LAYER TEMPLATE WITH THE FAB BEFORE ORDERING.

A2. SURFACE FINISH: ENIG. REQUIRED FOR THE 0.35 mm PITCH
    QFN-104 (U17), THE 0.4 mm PITCH QFN-56 (U15), THE 0.4 mm
    PITCH USB SWITCH (U1), THE 0.5 mm PITCH 24-BALL WLCSP FLASH
    (U13, U16), THE 0.45 mm PITCH LOAD SWITCHES (U26, U28) AND
    THE CHIP ANTENNA (U22). DO NOT SUBSTITUTE HASL.

A3. VIAS: ALL THROUGH, 0.40 mm PAD ON 0.30 mm DRILL.
    FILLED WITH NON-CONDUCTIVE EPOXY AND PLATED OVER (CAPPED),
    IPC-4761 TYPE VII. VIA-IN-PAD IS REQUIRED - ABOUT 200 VIAS
    SIT IN SMD LANDS, 27 OF THEM IN U17'S THERMAL PAD.

A4. RF: THE 2.4 GHz CHIP-ANTENNA FEED IS 0.18 mm ON F.Cu WITH
    0.13 mm GAPS TO THE F.Cu GROUND POUR, OVER In1 GROUND -
    ABOUT 47 OHM ON THE A1 STACK AND ABOUT 6 mm LONG, SO
    IMPEDANCE CONTROL IS OPTIONAL. DO NOT ADJUST THE WIDTH.
    *** THE CHIP ANTENNA U22 AT THE LEFT EDGE HAS A DELIBERATE
    COPPER-FREE WINDOW ON ALL EIGHT LAYERS. DO NOT FILL IT. ***

A5. MIN ANNULAR RING 0.05 mm.
    MIN VIA 0.40 mm PAD ON 0.30 mm DRILL.

A6. MIN TRACK AND SPACING 0.09 mm.
    MIN HOLE TO HOLE 0.20 mm, MEASURED HOLE EDGE TO HOLE EDGE.
    COPPER TO BOARD EDGE AND TO THE SLOT EDGES 0.20 mm.

A7. SOLDER MASK: THE DAM BETWEEN U17'S 0.35 mm PITCH PINS IS
    0.06 mm (U19 0.056 mm, U15 0.070 mm, U23 0.029 mm AT ITS
    FOUR CORNERS). IF A DAM CANNOT BE HELD, GANG THE OPENING
    ALONG THAT ROW RATHER THAN SHRINKING THE PADS.

A8. MIXED TECHNOLOGY - TWO THROUGH-HOLE PARTS (J8, C130) AND
    EIGHT PLATED 2.2 mm MOUNTING HOLES (H1-H8) SHARE THE BOARD
    WITH 0.35 mm PITCH SMD. NON-PLATED HOLES: 0.65 mm (J6 PEGS)
    AND 1.04 mm (J3 PEGS).

A9. NO BREAK-AWAY TABS, MOUSE BITES OR V-SCORE ON THE TOP EDGE
    (THE 22.35 mm EDGE OPPOSITE THE J2 SCREW TERMINAL). R83 IS
    0.25 mm FROM IT; THE MAGNETOMETER U3 AND C5, C6 AND C147
    ARE 0.41-0.49 mm FROM IT.
```

---

## Block B — assembly

```
TINKERROCKET TINKER-MANTIS (ROCKET COMPUTER V10) - ASSEMBLY NOTES

B0. *** STENCIL FOIL 0.08 mm (3 mil), FLAT - NO STEP.
    SEE hardware/SOLDER-PASTE-CONVENTION.md FOR THE RULE
    THIS COMES FROM (#959, #906).
    LASER CUT, ELECTROPOLISHED AND NANO-COATED.
    TOP AND BOTTOM STENCILS BOTH REQUIRED. ***
    AREA RATIO, TIGHTEST APERTURES (AT 0.08 mm / AT 0.10 mm):
      F.Cu  U13, U16 WLCSP 0.254 mm ROUND    AR 0.79 / 0.64
            U17 ESP32-P4 0.65 x 0.18 OVALS   AR 0.91 / 0.73
      B.Cu  U23 INA230 EPAD CORNER WINDOWS   AR 0.83 / 0.66
    THE IPC-7525 FLOOR IS 0.66. AT 0.10 mm THE TWO WLCSP BOOT
    FLASHES FALL BELOW IT, WHICH IS WHY THIS BOARD IS ON
    0.08 mm. DO NOT SUBSTITUTE A THICKER FOIL.

B1. PLACEMENT AND REFLOW.
    - REFLOW THE TOP SIDE FIRST AND THE BOTTOM SIDE SECOND, SO
      THE HEAVY BOTTOM-SIDE SMT PARTS (J2 TERMINAL BLOCK, J3,
      J4) REFLOW ONCE, UPRIGHT.
    - FIDUCIALS: FID1 AND FID2 TOP; FID3 AND FID4 BOTTOM.
    - U13 AND U16 (24-BALL WLCSP) ARE 180-DEGREE SYMMETRIC: A
      ROTATED PART STILL FITS AND PUTS VCC ON THE GROUND BALL.
      CHECK THE PLACEMENT PREVIEW: BALL A1 IS THE CORNER
      NEAREST THE SILK DOT.
    - DO NOT FIT: C12, C93, R74, R75, R76 (C93 AND R74-R76: SEE
      B8). THE POSITION FILE IS EXPORTED WITH --exclude-dnp.

B2. THROUGH-HOLE: J8 (BATTERY INPUT, JST VH) AND C130 (HOLD-UP
    SUPERCAPACITOR), BOTH ON THE BOTTOM SIDE. HAND OR SELECTIVE
    SOLDER AFTER BOTH REFLOW PASSES.

B3. C130 STAKING: C130 IS AN 8 mm CAN STANDING 12 mm TALL ON TWO
    LEADS. BOND IT TO THE BOARD WITH NON-CORROSIVE ALKOXY-CURE
    RTV SILICONE TO MIL-A-46146, FLEXIBLE AND GAP-FILLING.
    *** DO NOT SUBSTITUTE ACETOXY-CURE ("VINEGAR SMELL") RTV - IT
    RELEASES ACETIC ACID ONTO THE PARTS AROUND THE CAN.
    DO NOT SUBSTITUTE EPOXY - IT IS RIGID AND COUPLES THE CAN'S
    INERTIA INTO THE JOINTS INSTEAD OF DAMPING IT. ***

B4. C130 POLARITY: PAD 1 = V_SCAP (+), THE SQUARE PAD, MARKED "+";
    PAD 2 = GND.

B5. J8 IS THE BATTERY INPUT (JST VH). CONFIRM PIGTAIL POLARITY AGAINST
    THE SCHEMATIC, NOT THE SILKSCREEN LEGEND.

B6. CAMERA PORT J4 PIN 1 SUPPLIES SWITCHED PACK VOLTAGE, 6.4-8.4 V
    (2S LiPo); PIN 2 IS GND. *** THESE TWO PINS MOVED AT THE HIGH-SIDE
    SWITCH REWORK - PIN 2 WAS THE SUPPLY ON EARLIER REVISIONS. BUILD
    THE CAMERA PIGTAIL TO THIS PINOUT, NOT TO AN OLDER CABLE. ***
    QUALIFIED CAMERAS: RUNCAM SPLIT 4 (5-20 V) AND GOPRO
    HERO10 BLACK BONES (2S-6S, 5-27 V). THE PORT'S GUARANTEED RANGE IS
    THE NARROWER OF THE TWO: 5-20 V.
    *** DO NOT FIT A 5 V REGULATOR ON THIS BRANCH. *** THE BONES IS
    REPORTED TO STOP RECORDING INTERMITTENTLY ON 5 V; GOPRO RECOMMENDS
    A HIGHER-VOLTAGE SUPPLY. RAW 2S IS THE INTENDED SUPPLY.
    THE BONES SHUTTER-CONNECT WIRE MUST NOT EXCEED 5 V. J4 PINS 3/4
    (Camera_RX/TX) ARE 3.3 V GPIO THROUGH 1 k - SAFE. DO NOT RE-PURPOSE
    THOSE PINS FOR ANYTHING AT A HIGHER LEVEL.

B7. *** SERVO/EXP PORT J3 - POWER AND GROUND CHANGED ENDS. ***
    CURRENT PINOUT:  J3.1 AND J3.2  = SWITCHED SERVO SUPPLY, 6.4-8.4 V
                     J3.15 AND J3.16 = GND
    EARLIER REVISIONS HAD THE OPPOSITE: J3.15/16 CARRIED THE SUPPLY AND
    J3.1/2 CARRIED THE LOW-SIDE SWITCHED RETURN. THE PINS WERE SWAPPED
    DELIBERATELY TO MAKE THE HIGH-SIDE ROUTING CLOSE.
    THE SERVO ADAPTER'S MATING HEADER IS PIN 1 = GND, PIN 6 = LiPoPos.
    A CABLE BUILT FOR AN EARLIER REVISION APPLIES REVERSE POLARITY TO
    THE ADAPTER - 400 uF OF BULK AND FOUR SERVOS. RE-PIN OR REMAKE THE
    HARNESS, AND LABEL IT TO THIS REVISION.
    J3'S PEGS ALSO FIT ROTATED 180 DEGREES. PIN 1 IS THE END WITH THE
    SILK DOT AT THE HOUSING CORNER.

B8. *** U17 MUST BE A v1.3 ESP32-P4. NOT v3.x. ***
    THE PART NUMBER IS THE SAME FOR BOTH - CHECK THE DATE CODE OR
    PACKAGE MARKING, NOT THE ORDER CODE. NO DESIGN FILE CAN CARRY
    THIS: THE BOM, GERBERS AND PICK-AND-PLACE ALL SEE ONLY
    "ESP32-P4NRW32".
    THE BOARD IS BUILT FOR v1.3: R74, R75, R76 AND C93 ARE DNP,
    THE BUCK (U20) TAKES ITS FEEDBACK FROM THE P4 ITSELF ON PIN 78,
    AND VDD_HP_1 (PIN 54) IS DELIBERATELY LEFT ISOLATED.
    A v3.x PART IN THIS SOCKET RUNS WITH A CORE RAIL UNPOWERED AND
    NO EXTERNAL BUCK FEEDBACK, ON A 0.35 mm PITCH QFN-104 THAT
    CANNOT BE REWORKED.

B9. EXP LINES 09/10/11 (J3 PINS 11/12/13) CARRY P4 BOOT STRAPS
    (GPIO38/37/34). INTERFACE RULE FOR ANY PAYLOAD: THESE THREE
    LINES MUST BE INPUTS OR HIGH-Z UNTIL THE FLIGHT COMPUTER IS UP.
    PAYLOADS THAT DRIVE AT POWER-ON MUST USE THE OTHER NINE LINES.
    *** THE V9 HAD THESE STRAPS ON J3 PINS 12/11/10 - A V9 PAYLOAD
    HARNESS ON J3 PINS 9-14 DOES NOT MATCH THIS REVISION. ***
    THE FLIGHT SOFTWARE POWERS THIS PORT BEFORE THE P4 FOR THE SAME
    REASON - AN UNPOWERED PAYLOAD'S ESD CLAMPS WOULD PULL THE STRAPS
    LOW THROUGH NO FAULT OF ITS OWN.
    NOTE: THE P4 ROM BOOT LOG TRANSMITS ON EXP_10 (J3 PIN 12, ~200 ms,
    115200 BAUD) AT EVERY RESET - AVOID PARKING A TWITCH-SENSITIVE
    SERVO THERE, OR SEE THE eFUSE OPTION IN ISSUE #725.

B10. THE LoRa (J5, 4-PIN SR) AND GNSS (J1, 5-PIN SR) JUMPERS ARE
    STRAIGHT, PAD 1 TO PAD 1. ORDER THE "A" SUFFIX (A04SR04SR30K51A,
    A05SR05SR30K51A) - THE CATALOGUE CALLS IT "REVERSED"; THE PLAIN
    "SOCKET TO SOCKET" B PART FLIPS THE PIN ORDER AND PUTS PACK VOLTAGE
    ON THE DAUGHTERBOARD'S UART PIN. CHECK EVERY JUMPER PAD 1 TO PAD 1
    BEFORE IT IS CONNECTED. SEE ../cables.md.

B11. MOUNTING HARDWARE, H1-H8 (M2):
    - SCREW HEADS NO LARGER THAN 3.8 mm DIAMETER. ROUND STANDOFFS
      NO LARGER THAN 4.0 mm, OR 3.5 mm ACROSS-FLATS HEX. NO METAL
      WASHERS. OTHER-NET COPPER STARTS 0.1 mm OUTSIDE EACH PAD.
    - BOTTOM SIDE: NYLON STANDOFFS AT H2, H4, H6 AND H8 - PACK AND
      PYRO COPPER RUNS JUST OUTSIDE THOSE PADS. H2 IS THE ONLY
      HOLE BONDED TO GND; IF IT TAKES A METAL STANDOFF, PUT AN
      INSULATING WASHER UNDER IT ON THE BOTTOM SIDE.
    - *** H1 AND H3: NON-MAGNETIC SCREWS, STANDOFFS AND NUTS ONLY
      (BRASS, TITANIUM OR NYLON). THE MAGNETOMETER U3 IS 0.6 mm
      FROM H3'S SCREW HEAD; A MAGNETISED STEEL SCREW OFFSETS IT BY
      UP TO SEVERAL HUNDRED uT. ***

```

---

## Why each item is here

**A1 — the stackup, and specifically the inner copper weight.** The board file declares `JLC08161H-2116` exactly, the same block as the Tinker-Beetle: 2116 outer prepregs at 0.1164 mm, 0.300 mm cores, 1080×2 inner prepregs at 0.1528 mm, **1 oz outer and 0.5 oz inner (0.0152 mm)**, 1.630 mm total. The 0.5 oz figure sets the ampacity of everything inner: the In5 VBATT pour that feeds the servo, camera and daughterboard switches, and the In2/In5 routing. The 2026-09-29 layout review measured the servo path at about 13 mΩ from the eFuse to U28. JLC do not publish an 8-layer impedance template, so the stack has to be confirmed with them rather than assumed.

**A2 — surface finish.** HASL coplanarity is wrong for a 0.35 mm-pitch QFN-104, which is the finest-pitch part on the board and is chip-down where it cannot be reworked. The two 0.5 mm-pitch WLCSP boot flashes need a flat finish as much.

**A3 — via protection.** Via-in-pad is used throughout this layout: about 200 of the 513 vias sit in SMD lands, 27 of them in the P4's thermal pad (counted 2026-09-30). The `(filling yes)` and `(capping yes)` flags in board setup drive KiCad's DRC and 3D view and nothing else, so the process has to be stated here.

**A4 — the antenna feed.** The 2.4 GHz feed runs 5.8 mm from the S3 through L2 (2.2 nH shunt) and C23 (5.1 pF series) to U22, at 0.18 mm with 0.13 mm gaps to the F.Cu pour over In1. A 2-D field solve of that grounded coplanar line on the 0.1164 mm 2116 prepreg gives about 47 Ω. The mismatch against 50 Ω costs well under 0.1 dB over that length, which is why impedance control is optional. The match (C23, L2 and the DNP C12 shunt position) is the tuning provision, and the copper-free window matches Abracon's recommended 4.6 × 3.5 mm on every layer.

**A7 — the mask dams.** At 0.35 mm pitch the web between U17's pin openings is 0.06 mm, finer than a fab usually holds. Ganging the opening along the row keeps the pads and their paste intact; shrinking the pads would trade a cosmetic dam for a weaker joint on a part that cannot be reworked.

**A9 — the top edge.** R83 sits 0.25 mm from the top edge, and the magnetometer U3 and C5, C6 and C147 sit 0.41–0.49 mm from it (measured 2026-09-30). Breaking a tab or a V-score there bends the board directly under them. That can crack the capacitors, and it stresses the magnetometer.

**B0 — the foil.** Both boot flashes print 0.254 mm round apertures. At 0.10 mm that is area ratio 0.64, under the IPC-7525 floor of 0.66: poor paste release on a ball land means an open joint under the chip that cannot be inspected. At 0.08 mm the worst aperture on the board is 0.79. The Tinker-Beetle, the Tinker-Base and the LoRa board are on 0.08 mm for the same part.

**B1 — the reflow order.** The bottom side carries the heaviest SMT parts (the J2 screw terminal, the J3 box header and the J4 camera connector), so it reflows second and those parts see one pass, upright. The WLCSP flash has no mechanical key: rotated 180° it still fits, and puts VCC on the ground ball.

**B2 to B4 — C130.** The hold-up supercapacitor is an 8 × 12 mm radial can standing on two leads on the bottom side. It weighs about a gram, so staking is what keeps flight vibration out of its joints. The adhesive restrictions are not stylistic: acetoxy-cure RTV releases acetic acid as it cures, and a rigid epoxy couples the can's inertia straight into the joints instead of damping it. Pad 1 is square and carries the "+" legend.

**B8 — the silicon revision, which no file can carry.** This is the purest example of why this document exists. `ESP32-P4NRW32` is the order code for both v1.3 and v3.x; the revision lives in the date code and package marking. So the BOM cannot express it, the pick-and-place cannot express it, and a distributor will ship whichever reel is current. Meanwhile the board states its assumption only implicitly, through four unpopulated parts — R74, R75, R76 and C93 — and a reader who does not already know what that DNP set means will not infer "v1.3 silicon" from it. Fitting a v3.x part gives a core rail (VDD_HP_1, pin 54) with no supply and an external buck with no feedback divider, on a 0.35 mm pitch QFN-104 that cannot be reworked off the board. The DNP set is the tell; this note is what makes it readable.

**B6 and B7 — the two connectors whose power pins moved.** The low-side to high-side switch conversion re-pinned both J4 and J3, and neither change is visible in anything the assembler or the harness builder receives. J4's supply moved from pin 2 to pin 1, and J3's moved from the 15/16 end to the 1/2 end, with ground taking the vacated pins in both cases. Nothing on the silkscreen distinguishes them, the connectors mate mechanically either way, and the failure is not a dead port — it is reverse polarity into a camera, or into the servo adapter's 400 µF of bulk and four servos. The servo adapter carries no protection of its own; it takes LiPoPos and GND straight off the J3 cable. Cables built for any earlier revision must be re-pinned, and every cable should be labelled with the revision it was built for.

**B9 — the strap pins moved too.** The V10 puts EXP_07 to EXP_12 on J3 pins 9 to 14 in ascending order; the V9 had them descending. The three strap lines (GPIO38/37/34) moved from J3 pins 12/11/10 to 11/12/13, and the ROM boot-log pin (EXP_10) from J3.11 to J3.12. The fin servos (EXP_01 to EXP_04 on J3.3 to J3.6) and the power pins did not move, so the servo adapter and its cable are unaffected.

**B11 — the mounting hardware.** Every hole's Ø3.8 pad has other-net copper starting 0.10–0.13 mm outside it, and on the bottom side that copper is the battery supply upstream of the eFuse (at H2 and H8), a pyro output (H6) and VBATT (H4). Solder mask is the only insulation between a clamped, vibrating standoff and that copper, and H2 is bonded to GND. The magnetometer sits 0.64 mm from H3's head circle; a screw magnetised by a driver bit is estimated at 180–460 µT at the sensor against a ±800 µT range, and it changes whenever the screw is swapped, which invalidates the calibration.

---

## Still open

Nothing that affects a fab order. #1556 tracks the items left for a future revision.

## Done

- **#1556 layout pass, 2026-09-30.**
  - **F4:** pin-1 dots were added to the board copies of U17, U15, U23, U1, U6, U7, U8, U10, U9, J1 and J3; the library footprints are unchanged. U26's and U28's dots moved clear of R86's pad and the board edge. U2's pin-1 circle moved to the top silkscreen, on the board and in the library.
  - **F5:** the "F" legend moved clear of C44 to S1's inner end. The two orphan "+" marks were deleted. The "O" stays where it is (the owner's call): no clean site fits near its end of the switch, and the fab clips the part that sits over R52's and R63's pads.
  - **F6:** R41.1, R47.2 and R22.2 connect to the pour through thermal relief.
  - **F7:** the OC_ARM_EN via moved clear of J2's pad. The VDDO_PSRAM ring, 0.008 mm outside J4's mask opening, stays as it is.
  - **F8:** FID3 moved to the top-left corner of the bottom side. The bottom pair is now 35.3 mm apart on the diagonal.
  - **A3** now counts the via-in-pad lands as "about 200", and **A9** (the top edge) is new. The `User.Drawings` copy was updated to match.

- **V10 rewrite, 2026-09-30.**
  - Both blocks were rewritten for the 8-layer layout: stack, finish, vias, antenna feed, mask dams, 0.08 mm foil, reflow order, C130, the J3 strap pins and the mounting hardware.
  - The `User.Drawings` copy of block A was replaced to match.
  - The V9 items went with the V9 layout: the C12 can standoff, interposer and adhesive (old B1–B4) and the 6-layer `JLC06161H-3313` stack. They survive at tag `rocket-computer-v9.0.0`.
- **Paste and mask, 2026-09-29.** U18, U27, U29 and U30 were updated from the library: window-pane paste and exposed-pad mask margin 0.
