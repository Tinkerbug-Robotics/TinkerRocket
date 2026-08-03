# LoRa daughterboard — fabrication notes

**Paste the block below into the fab's order notes verbatim.** The same text is on the `Dwgs.User` layer, which plots as `User_Drawings.gbr`.

This file exists because **none of it can travel in the design files.** Via protection, surface finish and the deliberate plane void were each verified absent from the gerbers, the `.gbrjob`, the Excellon drill file, IPC-2581 *and* ODB++. Written text is the only channel to the fab.

---

```
TINKERROCKET LORA DAUGHTERBOARD - FABRICATION NOTES
BOARD 22.0 x 27.5 mm, 6 LAYER, 1.6 mm NOMINAL.

1. STACKUP: JLCPCB JLC06161H-3313, 1 oz OUTER / 0.5 oz INNER. BUILD TO THIS
   TEMPLATE. JLC LISTS TWO TEMPLATES UNDER THIS NAME - USE THE DEFAULT ONE,
   WITH THE 2116 MIDDLE PREPREG AT 0.1164 mm (NOT THE 0.1088 mm VARIANT).

2. SURFACE FINISH: ENIG. REQUIRED FOR THE 0.4 mm PITCH QFN-56 (U28),
   THE EDGE-LAUNCH SMA (J8) AND THE USB-C CONNECTOR (J2).

3. VIAS: FILLED WITH NON-CONDUCTIVE EPOXY AND PLATED OVER (CAPPED),
   IPC-4761 TYPE VII. 66 VIAS FALL INSIDE SMD LANDS, INCLUDING SIX IN THE
   QFN-56 THERMAL PAD (U28). VIA-IN-PAD PROCESS IS REQUIRED.
   ALL OTHER VIAS TENTED BOTH SIDES.

4. CONTROLLED IMPEDANCE: 50 OHM SINGLE-ENDED ON THE ANT NET, F.Cu,
   FROM J8 PAD 1 TO U14 PAD 6.
   *** In1.Cu AND In2.Cu ARE DELIBERATELY VOIDED BENEATH THE ENTIRE ANT
   PATH BY A RULE AREA. DO NOT FILL THIS VOID. THE RF REFERENCE PLANE IS
   In3.Cu. FILLING IT DESTROYS THE ANTENNA MATCH. ***

5. SOLDER MASK: MINIMUM DAM 0.08 mm AT U28 (0.4 mm PITCH). IF THAT CANNOT
   BE HELD, GANG THE OPENING RATHER THAN SHRINKING THE PADS.

6. MINIMUM ANNULAR RING IS 0.050 mm - 194 VIAS ARE 0.40 mm PAD ON 0.30 mm
   DRILL. CONFIRM THIS IS WITHIN CAPABILITY BEFORE BUILDING.

7. MINIMUM TRACK AND SPACING: 0.10 mm.
```

---

## Why each item is here

**1 — stackup.** JLC's template API returns two stackups with the identical name `JLC06161H-3313`; they are indistinguishable by name on the order page. The default has a 0.1164 mm middle prepreg (compressionThickness 1.546), the other 0.1088 mm (1.5384). The board file is built to the default.

**2 — finish.** `copper_finish` in the board file says `ENIG`, but it reaches the `.gbrjob` only as a `Finish` string that is easy to miss; and a bare `None` would have defaulted most fabs to HASL, whose coplanarity is wrong for a 0.4 mm-pitch QFN.

**3 — via protection.** `(capping yes) (filling yes)` in KiCad board setup drives DRC and the 3D view and *nothing else*. If the boards come back merely tented, the joints over those lands drain at reflow. This is a cost adder — **confirm it appears on the quote**, because it gets silently dropped.

**4 — the plane void.** This is the item most likely to be "helpfully" corrected by a fab or a CAM operator who sees a void in two planes and assumes it is an error. It is not. `J8`'s pad is 1.5 mm wide over what would otherwise be a 0.0994 mm dielectric — a ~10 Ω line, ~2 pF of shunt directly at the antenna port. With In1 and In2 voided the launch measures 41.6 dB return loss / VSWR 1.02; filled, it is 10.3 dB / VSWR 1.87, which costs about 9 % of range at each end of the link.

**5 — mask dam.** 0.4 mm pitch leaves ~0.1 mm between adjacent lands; a fab defaulting to a 0.1 mm minimum dam will produce slivers that lift.

**6 — annular ring.** 0.050 mm is advanced capability at several vendors. Declaring it up front avoids a DFM hold, and if the fab cannot hold it the fix is to widen the via pads to 0.45–0.50 mm rather than shrink the drill.

**7 — track/space.** 0.10 mm is the board minimum (39 segments); most routing is 0.127 mm.
