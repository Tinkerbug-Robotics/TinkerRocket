# spin-rig-controller — fabrication and assembly notes

This folder's board is the spin rig controller, V1. It is an ESP32-S3 board that runs a StepperOnline CL57T
closed-loop NEMA 23 kit from a 6S pack on the GNSS spin bench rig. It is 4 layers on a standard 1.6 mm build.
I wrote these notes on 2026-10-16 from the board file. The design note is [`README.md`](README.md).

**Paste the block below into the fab's order notes verbatim.**

None of it can travel in the design files. The via treatment, the finish and the hand work do not reach the fab
through the gerbers or the drill files, so written text is the only channel. Stencil thickness, the area-ratio
floor and the paste-coverage convention are repo-wide; they live in
[`hardware/SOLDER-PASTE-CONVENTION.md`](../SOLDER-PASTE-CONVENTION.md).

---

```
TINKERROCKET SPIN RIG CONTROLLER V1 - FABRICATION AND ASSEMBLY NOTES
BOARD 110.0 x 80.0 mm, 4 LAYER, 1.6 mm. GERBERS, DRILL AND CPL ALL
USE THE BOARD'S BOTTOM-LEFT CORNER AS 0,0.

1. STACKUP: JLCPCB JLC04161H-7628, 1.6 mm, 1 oz OUTER / 0.5 oz INNER.
   LAYERS: F.Cu SIGNAL + POURS, In1 GND PLANE, In2 5 V PLANE (BOTTOM)
   + SIGNALS, B.Cu SIGNAL + GND POUR. NO CONTROLLED IMPEDANCE.

2. SURFACE FINISH: ENIG. FOR THE 0.4 mm PITCH QFN-56 (U201) AND ITS
   4.1 mm EXPOSED PAD.

3. VIAS: 702 THROUGH VIAS, TENTED BOTH SIDES. STANDARD PROCESS: NO
   FILLING, NO CAPPING.
   685 x 0.60 mm PAD / 0.30 mm DRILL, 13 x 0.80 / 0.40, 4 x 0.45 / 0.30.
   NO VIA-IN-PAD: NO VIA SITS IN OR CUTS INTO AN SMD LAND, INCLUDING
   THE EXPOSED PADS OF U201 AND U104.

4. MINIMUM TRACK 0.15 mm, MINIMUM SPACING 0.15 mm (0.30 mm ON THE
   MOTOR-CURRENT NETS). MINIMUM ANNULAR RING 0.075 mm (THE FOUR
   0.45 / 0.30 VIAS), 0.15 mm ELSEWHERE. COPPER TO BOARD EDGE 0.30 mm.

5. HOLES: 4 x 3.20 mm NON-PLATED (H1-H4, M3), 2 x 0.65 mm NON-PLATED
   (J201 USB-C PEGS).
   PLATED: 2 x 2.70 mm (J101 XT60), 2 x 0.60 x 1.70 mm OVAL PLATED
   SLOTS (J101 XT60 PEGS), 2 x 1.70 mm (J102 JST VH), 23 x 1.20 mm
   (J103, J301, J302, J305: 3.5 mm TERMINAL HEADERS), 5 x 1.00 mm
   (J202 HEADER), 9 x 0.75 mm (J104, J303, J304: JST PH).

6. USB-C RECEPTACLE J201 OVERHANGS THE BOTTOM EDGE BY ABOUT 0.6 mm
   ON PURPOSE; ITS SILKSCREEN RUNS TO THE EDGE.

7. ORDER NUMBER: AT THE "JLCJLCJLCJLC" MARK ABOVE THE TITLE, BOTTOM
   LEFT.

ASSEMBLY
ECONOMIC PCBA, SINGLE SIDED: ALL SMD PARTS ON THE TOP. BOM AND CPL
CARRY LCSC CODES.
DO NOT PLACE R305, R313, R314 (DNP).
THROUGH-HOLE CONNECTORS J101-J104, J202, J301-J305 ARE NOT IN THE BOM
OR CPL: THEY ARE HAND-SOLDERED AFTER ASSEMBLY. DO NOT FIT THEM.
TOOLING HOLES (1.152 mm NPTH), IF NEEDED: PLEASE USE THESE SPOTS,
CLEAR OF TRACKS ON EVERY LAYER (mm FROM THE BOTTOM-LEFT CORNER):
X 107.50 Y 71.00, X 3.00 Y 8.25, X 101.75 Y 5.00.
CHECK EVERY IC AND CONNECTOR ROTATION IN THE CPL PREVIEW.
```

---

## Before ordering

- Plot with [`tools/plot_gerbers.sh spin-rig-controller --aux-origin`](../../tools/plot_gerbers.sh) from a clean tree. `--aux-origin` puts the gerbers and drill file in the CPL's frame (the board's bottom-left corner). Tag the ordered commit `spin-rig-controller-v1.0.0` ([`docs/board-versioning.md`](../../docs/board-versioning.md)).
- `gen/make_fab.py` writes a review copy of the gerbers, plus the JLC BOM and CPL, into `gerbers/`, which git ignores. The BOM and CPL are what to upload; take the gerbers from `plot_gerbers.sh`.
- It also writes `spin-rig-controller-hand.csv`: the through-hole connectors to buy and solder by hand, with their plugs listed in the design note ("Leads to the driver").
- The design note's "Before ordering" list still applies:
  - the routing review;
  - the CL57T bench checks (S3 switch, ENA polarity);
  - the CL57T's terminal pitch (the headers are 3.5 mm);
  - the XT60's polarity against the pack lead;
  - the 8 mm motor shaft.

## Ordering assembled boards from JLC, step by step

The order is one PCB order with assembly switched on. JLC fits the 138 SMD parts on the top side. The ten
through-hole connectors come separately and are soldered by hand.

**1. The bare boards.** Upload the gerber zip at jlcpcb.com ("Order now"). JLC should read 110 x 80 mm and 4
layers. Then set:

| Option | Setting |
|---|---|
| PCB qty | 5 (the minimum) |
| Delivery format | Single PCB |
| PCB thickness | 1.6 mm |
| Color, silkscreen | green, white (any color works; green is the cheapest) |
| Surface finish | ENIG |
| Outer / inner copper | 1 oz / 0.5 oz |
| Specify stackup | JLC04161H-7628 (JLC's standard 1.6 mm 4-layer stackup) |
| Via covering | Tented |
| Min via hole size / diameter | 0.3 mm / (0.4 / 0.45 mm) |
| Remove order number | Specify a location (the board carries the "JLCJLCJLCJLC" mark) |

Leave anything not listed at its default. Small charges to expect: ENIG, the 4-layer stackup, and possibly a
routing fee for the XT60's two 0.6 mm plated slots.

**2. Assembly.** Switch on "PCB Assembly", then set:

| Option | Setting |
|---|---|
| PCBA type | Economic. It takes 4 layers, 1.6 mm, JLC's standard stackup and top-side parts down to 0402 and 0.4 mm pitch, which is this board; it needs no edge rails |
| Assembly side | Top side |
| PCBA qty | 2 to start (each assembled board adds about $23 of parts; the other 3 boards stay bare) |
| Tooling holes | Added by JLCPCB. The order notes ask for the three clear spots above |
| Confirm parts placement | Yes, if offered: JLC shows you the placement before production |

**3. Files.** Upload `spin-rig-controller-bom.csv` and `spin-rig-controller-cpl.csv`. JLC matches the parts by
their LCSC codes. All 60 lines should match, with nothing short: on 2026-10-16 the tightest stock was the
40 MHz crystal (263) and the INA238 (408).

**4. The placement preview.** This is the step that catches mistakes. Check every polarized part against the
table below: its pin-1 dot or cathode band must sit where the table says. Rotate any that don't (90° steps), and
check that every part sits centered on its pads. Positions are millimetres from the board's bottom-left corner,
with the connector edge at the top.

| Part | Package | Position (x, y) | Pin 1 or polarity mark |
|---|---|---|---|
| U201 (ESP32-S3) | QFN-56 | 72.0, 36.0 | pin 1 top-left |
| U101 (INA238) | MSOP-10 | 14.0, 58.0 | pin 1 top-left |
| U102 (TLV3011) | SOT-23-6 | 33.0, 44.0 | pin 1 top-left |
| U103 (UCC27517) | SOT-23-5 | 33.0, 37.0 | pin 1 top-left |
| U104 (TPS54560B) | SO-8 PowerPAD | 20.0, 18.0 | pin 1 top-left |
| U105 (AP2112K) | SOT-23-5 | 68.0, 27.8 | pin 1 top-left |
| U301 (74ACT244) | TSSOP-20 | 60.0, 62.0 | pin 1 top-left |
| U302 (AM26LV32E) | SOIC-16 | 85.0, 62.0 | pin 1 bottom-left (turned 90°) |
| U303 (74LVC1G17) | SOT-23-5 | 94.5, 49.0 | pin 1 bottom-left (turned 90°) |
| U304 (74LVC2G125) | SSOP-8 | 93.5, 59.5 | pin 1 bottom-left (turned 90°) |
| Q101 (P-FET) | TO-252 | 29.5, 66.0 | tab faces left, toward the XT60; the gate lead is bottom-right |
| Q102 (AO4264E) | SOIC-8 | 22.5, 34.0 | pin 1 bottom-right (turned 180°) |
| CR201 (USB ESD) | SOT-143 | 69.0, 12.5 | pin 1 top-left |
| Y201 (40 MHz) | 4-pad crystal | 68.2, 44.2 | pin 1 bottom-right |
| D101 (12 V zener) | SOD-123 | 37.8, 60.5 | cathode left |
| D102 (TVS) | SMC | 29.0, 58.0 | cathode left |
| D103 (brake diode) | SMC | 14.8, 35.4 | cathode top |
| D105 (catch diode) | SMC | 17.0, 24.4 | cathode right |
| D107, D108 (Schottky) | SOD-323 | 67.8, 24.2 and 64.2, 10.5 | cathode left |
| D104, D202, D203 (LEDs) | 0603 | 36.5, 37.5; 58.2, 9.2; 52.2, 9.2 | cathode bottom |
| D106, D201 (LEDs) | 0603 | 40.8, 16.5; 55.2, 8.2 | cathode left |
| C101 (100 µF) | 8 x 10.5 electrolytic | 38.5, 74.4 | + left |

J201 (USB-C) has no pin 1 to check, but its opening must face the bottom edge. Its footprint origin sits 1.4 mm
off its pads, so check that it lands on them. Q101's origin is off its pads too (it's KiCad's TO-252).

**5. Order notes.** Paste the block at the top of this file into the order's remark box. If the box is too
short, keep items 2, 3 and 7 and the ASSEMBLY lines.

**6. The hand-soldered parts.** `spin-rig-controller-hand.csv` lists them. The JST PH parts are on hand; buy the
rest, and the plugs, from LCSC or Digi-Key:
- J101 XT60PW-M (LCSC C98732);
- J103 Phoenix 1844210, J302 and J305 Phoenix 1844252, J301 Phoenix 1844281 (LCSC had no 9-pin in stock on
  2026-10-16; Digi-Key and Mouser stock Phoenix's range);
- plugs for the board end of each driver lead: Phoenix MC 1,5/2-ST-3,5 (1840366), MC 1,5/6-ST-3,5 (1840405) x2
  and MC 1,5/9-ST-3,5 (1840434). The driver end uses the CL57T's own plugs;
- J102 JST B2P-VH (LCSC C160315) and J202 1x5 2.54 mm header (LCSC C492404).

**7. After ordering.** Tag the ordered commit `spin-rig-controller-v1.0.0` and move the board's in-work revision
to V2 ([`docs/board-versioning.md`](../../docs/board-versioning.md)).
