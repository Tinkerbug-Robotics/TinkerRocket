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
BOARD 110.0 x 80.0 mm, 4 LAYER, 1.6 mm.

1. STACKUP: JLCPCB JLC04161H-7628, 1.6 mm, 1 oz OUTER / 0.5 oz INNER.
   LAYERS: F.Cu SIGNAL + POURS, In1 GND PLANE, In2 5 V PLANE (BOTTOM)
   + SIGNALS, B.Cu SIGNAL + GND POUR. NO CONTROLLED IMPEDANCE.

2. SURFACE FINISH: ENIG. FOR THE 0.4 mm PITCH QFN-56 (U201) AND ITS
   4.1 mm EXPOSED PAD.

3. VIAS: 705 THROUGH VIAS, TENTED BOTH SIDES. STANDARD PROCESS: NO
   FILLING, NO CAPPING.
   690 x 0.60 mm PAD / 0.30 mm DRILL, 12 x 0.80 / 0.40, 3 x 0.45 / 0.25.
   NO VIA-IN-PAD: NO VIA SITS IN OR CUTS INTO AN SMD LAND, INCLUDING
   THE EXPOSED PADS OF U201 AND U104.

4. MINIMUM TRACK 0.15 mm, MINIMUM SPACING 0.15 mm (0.30 mm ON THE
   MOTOR-CURRENT NETS). MINIMUM ANNULAR RING 0.10 mm. COPPER TO BOARD
   EDGE 0.30 mm.

5. HOLES: 4 x 3.20 mm NON-PLATED (H1-H4, M3), 2 x 0.65 mm NON-PLATED
   (J201 USB-C PEGS), 6 x 1.70 mm PLATED (J101-J103, JST VH),
   5 x 1.00 mm PLATED (J202, 2.54 mm HEADER).

6. USB-C RECEPTACLE J201 OVERHANGS THE BOTTOM EDGE BY ABOUT 0.6 mm
   ON PURPOSE; ITS SILKSCREEN RUNS TO THE EDGE.

ASSEMBLY
SINGLE SIDED: ALL SMD PARTS ON THE TOP. BOM AND CPL CARRY LCSC CODES.
DO NOT PLACE R305, R313, R314, R315 (DNP).
THROUGH-HOLE PARTS J101, J102, J103 (JST B2P-VH) AND J202 (1x5 2.54 mm
HEADER): HAND-SOLDERED, OR JLC THROUGH-HOLE ASSEMBLY IF ORDERED.
CHECK EVERY IC AND CONNECTOR ROTATION IN THE CPL PREVIEW.
```

---

## Before ordering

- Plot with [`tools/plot_gerbers.sh spin-rig-controller`](../../tools/plot_gerbers.sh) from a clean tree, and tag the commit `spin-rig-controller-v1.0.0` ([`docs/board-versioning.md`](../../docs/board-versioning.md)).
- `gen/make_fab.py` writes a review copy of the gerbers, plus the JLC BOM and CPL, into `gerbers/`, which git ignores. The BOM and CPL are what to upload; take the gerbers from `plot_gerbers.sh`.
- The design note's "Before ordering" list still applies:
  - the routing review;
  - the CL57T bench checks (S3 switch, ENA polarity);
  - the three look-alike VH power connectors;
  - the 8 mm motor shaft.
