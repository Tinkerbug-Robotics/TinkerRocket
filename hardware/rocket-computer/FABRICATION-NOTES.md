# Rocket computer — fabrication and assembly notes

**Paste block A into the bare-board fab's order notes verbatim. Paste block B into the assembler's instructions.**

This file exists for the same reason the LoRa board's does: **none of it can travel in the design files.** Via protection, surface finish and inner-layer copper weight are each absent from the gerbers, the `.gbrjob`, the Excellon drill file, IPC-2581 and ODB++. The C12 standoff is worse than absent — a can floating 4 mm above the board looks like a defect to anyone who has not been told otherwise, and the instinct to "fix" it by seating it flush destroys the parts underneath.

Board: **22.35 × 75.00 mm, 6 layer, 1.546 mm stack.** F / In1 / In2 / In3 / In4 / B, with In1 and In4 solid GND, In2 split power and In3 routing.

> **Mirrored onto `Dwgs.User`** (2026-08-05) so the block plots as `User_Drawings.gbr` and reaches the fab even if the order notes are lost. Worth knowing: the LoRa board's notes claim the same thing, but that board actually carries no user-layer text at all — its only `gr_text` items are four on `F.SilkS`. Verify the gerber, don't trust the claim.

---

## Block A — bare-board fabrication

```
TINKERROCKET ROCKET COMPUTER - FABRICATION NOTES
BOARD 22.35 x 75.00 mm, 6 LAYER, 1.546 mm STACK.

1. STACKUP: JLCPCB JLC06161H-3313, 1 oz OUTER / 0.5 oz INNER.
   BUILD TO THIS TEMPLATE. JLC LISTS TWO TEMPLATES UNDER THIS
   NAME - USE THE DEFAULT, WITH THE 2116 MIDDLE PREPREG AT
   0.1164 mm (NOT THE 0.1088 mm VARIANT).

2. SURFACE FINISH: ENIG. REQUIRED FOR THE 0.35 mm PITCH
   QFN-104 (U17), THE 0.4 mm QFN-56 (U15) AND THE 0.4 mm
   X2QFN (U21). DO NOT SUBSTITUTE HASL.

3. VIAS: FILLED WITH NON-CONDUCTIVE EPOXY AND PLATED OVER
   (CAPPED), IPC-4761 TYPE VII. VIA-IN-PAD IS REQUIRED -
   FIVE VIAS SIT INSIDE THE eFUSE IN THERMAL PAD (U19).
   ALL OTHER VIAS TENTED BOTH SIDES.

4. CONTROLLED IMPEDANCE: 50 OHM SINGLE-ENDED ON THE 2.4 GHz
   ANTENNA FEED, F.Cu REFERENCED TO In1.Cu, 0.18 mm WIDE.

5. MIN ANNULAR RING 0.05 mm.
   MIN VIA 0.40 mm PAD ON 0.30 mm DRILL.

6. MIN TRACK AND SPACING 0.09 mm.

7. MIXED TECHNOLOGY - THROUGH-HOLE PARTS (C12, J8, J2)
   SHARE THE BOARD WITH 0.35 mm PITCH SMD.
```

---

## Block B — assembly

```
TINKERROCKET ROCKET COMPUTER - ASSEMBLY NOTES

B1. *** C12 IS DELIBERATELY STOOD OFF ABOVE THE BOTTOM-SIDE PARTS. ***
    C12 (10 000 uF 16 V ALUMINIUM ELECTROLYTIC RADIAL CAN,
    O18 x 25 mm, 7.5 mm LEAD PITCH, BOTTOM SIDE) SITS 4.0 mm
    ABOVE THE BOARD ON AN INTERPOSER.
    ITS BODY OVERHANGS ROUGHLY 19 POPULATED PARTS. THIS IS BY DESIGN.
    DO NOT SEAT IT FLUSH. DO NOT SHORTEN THE LEADS TO CLOSE THE GAP.

B2. INTERPOSER: 3D-PRINTED TPU, SHORE ~95A, 4.0 mm NOMINAL HEIGHT,
    SOLID OR >=50% INFILL, LAYERS NORMAL TO THE BOARD.
    - POCKETS CLEAR C56 (3.1 mm), U13 SOIC-8 (1.75 mm) AND THE
      SOT-23 GATE DRIVERS (1.1 mm).
    - TWO LEAD CLEARANCE HOLES ON 7.5 mm PITCH.
    - LOAD-BEARING FEET LAND ON BARE BOARD ONLY, NEVER ON A COMPONENT.

B3. ADHESIVE: NON-CORROSIVE ALKOXY-CURE RTV SILICONE TO MIL-A-46146,
    FLEXIBLE AND GAP-FILLING, RATED FOR A 4 mm BONDLINE.
    BOND CAN TO BOARD THROUGH THE INTERPOSER OPENINGS. THE TPU IS
    CAPTURED MECHANICALLY AND IS NOT ITSELF BONDED.
    *** DO NOT SUBSTITUTE ACETOXY-CURE ("VINEGAR SMELL") RTV - IT
    RELEASES ACETIC ACID ONTO THE PYRO GATE DRIVERS DIRECTLY BENEATH.
    DO NOT SUBSTITUTE EPOXY - RIGID, AND QUALIFIED ONLY TO A 0.2 mm
    BONDLINE; THIS JOINT IS 4 mm. ***
    LEAVE THE CAN PERIMETER UNFILLED SO A BLADE CAN REACH THE BOND.

B4. C12 POLARITY: PAD 1 = V_CAP (+), PAD 2 = GND. THE CAN VENT IS ON
    THE FAR END, FACING AWAY FROM THE BOARD - KEEP IT UNOBSTRUCTED AND
    DO NOT AIM IT AT THE E-MATCH HARNESS.

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

B9. EXP LINES 09/10/11 (J3 PINS 12/11/10) CARRY P4 BOOT STRAPS
    (GPIO38/37/34). INTERFACE RULE FOR ANY PAYLOAD: THESE THREE
    LINES MUST BE INPUTS OR HIGH-Z UNTIL THE FLIGHT COMPUTER IS UP.
    PAYLOADS THAT DRIVE AT POWER-ON MUST USE THE OTHER NINE LINES.
    THE FLIGHT SOFTWARE POWERS THIS PORT BEFORE THE P4 FOR THE SAME
    REASON - AN UNPOWERED PAYLOAD'S ESD CLAMPS WOULD PULL THE STRAPS
    LOW THROUGH NO FAULT OF ITS OWN.
    NOTE: THE P4 ROM BOOT LOG TRANSMITS ON EXP_10 (~200 ms, 115200
    BAUD) AT EVERY RESET - AVOID PARKING A TWITCH-SENSITIVE SERVO
    THERE, OR SEE THE eFUSE OPTION IN ISSUE #725.

```

---

## Why each item is here

**A1 — the stackup, and specifically the inner copper weight.** The board file now declares `JLC06161H-3313` exactly: 0.0994 mm 3313 prepreg at Er 4.1, 0.55 mm NP-155F cores at Er 4.41, 0.1164 mm 2116 middle prepreg at Er 4.16, **1 oz outer and 0.5 oz inner (0.0152 mm)**, 1.546 mm total. Before 2026-08-05 it declared a generic FR4 stack with 1 oz on all six layers, which overstated every inner-layer conductor by 2×.

This matters more than yield. The VBATT distribution to the servo connector, the camera and both radio daughterboards runs almost entirely on In2, so its ampacity is set by the 0.5 oz figure, not the 1 oz one the file used to claim. Every corridor width in the power-path review is now evaluated against 0.0152 mm. JLC's two identically-named templates are the same trap the LoRa board hit — the 2116 middle prepreg must be the 0.1164 mm variant, and this board is built to that one.

**A2 — surface finish.** `copper_finish` in the board file was the string `"None"` until 2026-08-05, which reaches the `.gbrjob` as nothing useful and lets most fabs default to HASL. It is now `"ENIG"`, but state it in the order notes anyway. HASL coplanarity is wrong for a 0.35 mm-pitch QFN-104, which is the finest-pitch part on either side and is chip-down where it cannot be reworked.

**A3 — via protection.** `(filling yes)` and `(capping yes)` are set in board setup, but those flags drive KiCad's DRC and 3D view and nothing else. If the boards come back merely tented, the joints over the in-pad vias drain at reflow — including the five inside the eFuse input thermal pad, which is the highest-current joint on the board. This is a cost adder, so confirm it appears on the quote rather than assuming it carried.

**A4 — the antenna feed.** The 2.4 GHz feed is 0.18 mm wide, which computes to **≈48.7 Ω** against the 0.0994 mm 3313 prepreg at Er 4.1 (it was ≈46.9 Ω against the old declared generic stack, so the stackup correction improved it). 0.17 mm would be a nominal 50.2 Ω, but 48.7 Ω is inside 3% and not worth a re-route. On a house stack with a thicker first prepreg the same trace runs low and the match is gone. There is no tuning provision on this build — the matching network is Molex's nominal single-shunt L-network with no series or second shunt position — so the trace width and the dielectric are the entire match.

**B1 to B3 — the standoff and the adhesive.** The can is stood off because there is no board area to give it: its Ø18 body spans x 75.89–93.89 on a board running 72.36–94.71, leaving 3.5 mm clear on one side and 0.8 mm on the other, and the bands fore and aft of it are fully occupied by the eFuse, the battery connector, the power mux and the pyro connector. Lifting it over the parts is a deliberate trade of vertical space for board area.

That leaves the leads as the only load path, which is acceptable for the loads this vehicle actually sees — a 12 g can at 50 g transport shock puts about 13 N in each lead, roughly 26 MPa against copper's 200–300 MPa yield — but not acceptable as the only thing stopping the can rocking onto the parts underneath. The interposer takes that job. Its feet land on the two pockets of bare bottom-side copper under the can, which measure 30.1 mm² at x 76.0–88.3 / y 142.0–150.0 and 20.4 mm² at x 80.8–88.3 / y 135.5–143.7, so load reaches the board rather than a component top. Both pockets sit left of centre while the right of the can is dense with C56, C43 and R33, so the right side of the interposer should be compliant enough that the can seats flat instead of rocking on the feet.

The adhesive restrictions are not stylistic. Acetoxy-cure RTV releases acetic acid as it cures, and the pyro gate-drive transistors sit 1.0 mm from the can's centreline. Epoxy is excluded on two independent grounds: it is rigid, which couples the can's inertia straight into 0402 terminations instead of damping it, and the toughened structural epoxies you would reach for are qualified at a 0.13–0.20 mm bondline against the 4 mm gap here — typically Shore D 75+ cured, with a CTE around 80 ppm/°C below Tg against FR4's ~15. The specification to buy against is the one in B3: non-corrosive alkoxy cure, flexible, gap-filling.

**B8 — the silicon revision, which no file can carry.** This is the purest example of why this document exists. `ESP32-P4NRW32` is the order code for both v1.3 and v3.x; the revision lives in the date code and package marking. So the BOM cannot express it, the pick-and-place cannot express it, and a distributor will ship whichever reel is current. Meanwhile the board states its assumption only implicitly, through four unpopulated parts — R74, R75, R76 and C93 — and a reader who does not already know what that DNP set means will not infer "v1.3 silicon" from it. Fitting a v3.x part gives a core rail (VDD_HP_1, pin 54) with no supply and an external buck with no feedback divider, on a 0.35 mm pitch QFN-104 that cannot be reworked off the board. The DNP set is the tell; this note is what makes it readable.

**B6 and B7 — the two connectors whose power pins moved.** The low-side to high-side switch conversion re-pinned both J4 and J3, and neither change is visible in anything the assembler or the harness builder receives. J4's supply moved from pin 2 to pin 1, and J3's moved from the 15/16 end to the 1/2 end, with ground taking the vacated pins in both cases. Nothing on the silkscreen distinguishes them, the connectors mate mechanically either way, and the failure is not a dead port — it is reverse polarity into a camera, or into the servo adapter's 400 µF of bulk and four servos. The servo adapter carries no protection of its own; it takes LiPoPos and GND straight off the J3 cable. Cables built for any earlier revision must be re-pinned, and every cable should be labelled with the revision it was built for.

**B4 — polarity and vent.** The C12 footprint carries no `+` marking; its only silkscreen polarity feature is an ambiguous ring on the *negative* pad, which is why the polarity is stated here in text. A reversed 16 V electrolytic charged to 8.4 V on the deployment energy store vents hot electrolyte, so the vent orientation matters as much as the polarity.

---

## Still open

- **Solder mask bridge at the eFuse — DRC error.** The `Net-(U19-DVDT)` via at (76.43, 134.01) sits 0.43 mm from pad 14 (GND) of U19, and their rear mask apertures merge with no dam between two different nets. Introduced with the via batch added on 2026-08-05. Move the via or add local mask expansion; as drawn it is a solder-bridge path on the protection device.
- **VBATT corridor at y 123.75–126.75.** With the copper weight now honestly declared at 0.5 oz inner, the binding constraint has moved here: 2.15–2.55 mm on In2 with no parallel copper on any other layer, giving roughly 1.25 A at 10 °C rise and 2.4 A at 45 °C. The eFuse exit at y 133–135 was fixed by the B.Cu added on 2026-08-05; the same treatment extended down through y 120–130 would clear the remaining neck.
- **Fiducials** — the board still carries exactly one, inside the USB-C connector's courtyard where the shell covers it. Two chip-down fine-pitch QFNs want three clear fiducials per populated side before this goes to an assembler.

## Done

- **Inner copper weight** — resolved 2026-08-05 by adopting `JLC06161H-3313` (0.5 oz inner). See A1.
- **Block A mirrored onto `Dwgs.User`** — 2026-08-05, plots as `User_Drawings.gbr`.
- **`+` polarity silk at C12 pad 1** — 2026-08-05, placed at (83.90, 138.87) on `B.SilkS`, outboard of pad 1 so it cannot be read as belonging to the negative pad, and clear of C56's outline and every pad. It sits under the can's overhang but is visible through the 4 mm standoff gap; there is no clear silk area outside the Ø18 body on this board.
