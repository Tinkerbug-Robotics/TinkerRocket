# Tinker-Base (base station) — fabrication and assembly notes

**Paste the block below into the fab's order notes verbatim.**

It exists because none of it can travel in the design files. Surface finish, via treatment, the deliberate antenna void
and the assembly steps are absent from the gerbers, the `.gbrjob`, the Excellon drill file, IPC-2581 and ODB++. Written
text is the only channel to the fab. Stencil thickness, the area-ratio floor and the paste-coverage convention are
repo-wide and live in [`hardware/SOLDER-PASTE-CONVENTION.md`](../SOLDER-PASTE-CONVENTION.md) (#959, #906).

---

```
TINKERROCKET TINKER-BASE (BASE STATION) - FABRICATION NOTES
BOARD 30.3 x 90.5 mm, 4 LAYER, 1.6 mm NOMINAL.

1. STACKUP: JLCPCB JLC04161H-7628, 1 oz OUTER / 0.5 oz INNER. BUILD TO THIS
   TEMPLATE; THE RF TRACE WIDTHS ARE DRAWN FOR IT.

2. SURFACE FINISH: ENIG. REQUIRED FOR THE 0.4 mm PITCH QFN-56 (U3), THE
   0.5 mm PITCH 24-BALL WLCSP (U1), THE USB-C RECEPTACLE (J2) AND THE
   EDGE-LAUNCH SMA (J8).

3. VIAS: 0.45 mm PAD ON 0.30 mm DRILL, TENTED BOTH SIDES. NINE VIAS SIT
   INSIDE U3'S THERMAL PAD, IN THE GAPS BETWEEN ITS PASTE WINDOWS; THEY ARE
   OPEN ON THE TOP SIDE WITHIN THAT PAD'S MASK OPENING. NO OTHER VIA IS IN
   AN SMD LAND.

4. RF TRACES: 0.36 mm ON F.Cu WITH A 0.15 mm GAP TO THE GROUND POUR, OVER
   In1.Cu GROUND, DRAWN FOR 50 OHM ON THIS STACKUP. DO NOT ADJUST WIDTHS.
   *** THE 2.4 GHz CHIP ANTENNA U15 AT THE LEFT EDGE HAS A DELIBERATE
   COPPER-FREE WINDOW ON ALL FOUR LAYERS. DO NOT FILL IT. ***

5. SOLDER MASK: THE DAM BETWEEN U3'S 0.4 mm PITCH PINS IS 0.07 mm. IF THAT
   CANNOT BE HELD, GANG THE OPENING RATHER THAN SHRINKING THE PADS.

6. MINIMUM ANNULAR RING: VIAS 0.075 mm (0.45 / 0.30).

7. MINIMUM TRACK AND SPACING: 0.10 mm (U1 ESCAPE).

ASSEMBLY - STENCIL
STENCIL FOIL 0.08 mm (3 mil), FLAT - NO STEP. TOP STENCIL ONLY.
LASER CUT, ELECTROPOLISHED AND NANO-COATED.
TIGHTEST APERTURE: U1 WLCSP 0.254 mm ROUND, AREA RATIO 0.79 AT 0.08 mm
(0.64 AT 0.10 mm, BELOW THE IPC-7525 FLOOR OF 0.66).

ASSEMBLY - PLACEMENT
- FIDUCIALS FID1-FID3, TOP SIDE.
- U1 (24-BALL WLCSP) IS 180-DEGREE SYMMETRIC: A ROTATED PART STILL FITS AND
  PUTS VCC ON THE GROUND BALL. CHECK THE PLACEMENT PREVIEW: BALL A1 IS THE
  CORNER NEAREST THE SILK DOT.
- C5 IS DO-NOT-FIT.
- J2 (USB-C, GCT USB4110-GF-A) IS FULLY SMT: ITS FOUR SHELL TABS ARE
  PASTED PADS AND REFLOW WITH THE SIGNAL PADS. NOTHING OF IT IS HAND-FITTED.

ASSEMBLY - THROUGH-HOLE AND HAND WORK
- J8 (EDGE SMA): FIT AFTER REFLOW, FLANGE AGAINST THE BOARD EDGE, CENTRE
  PIN AND BOTH GROUND LEGS SOLDERED.
- S1 (PANEL PUSHBUTTON), D6 AND D9 (3 mm LEDs): TOP SIDE, THROUGH-HOLE.
- S1'S TWO LOCATING POSTS PROTRUDE ABOUT 1.2 mm BELOW THE BOARD,
  UNDER THE BATTERY HOLDER. TRIM THEM FLUSH BEFORE FITTING BT2.
- BT2 (18650 HOLDER): BOTTOM SIDE, THROUGH-HOLE. THE HOLDER IS NOT
  POLARISED: FIT IT WITH ITS + MARK AT THE USB-CONNECTOR END (PAD P) AND -
  AT THE ANTENNA-CONNECTOR END (PAD N). PAD P IS 0.84 mm FROM J2'S NEAREST
  PAD - NO SOLDER BRIDGE.
```

---

## Why each item is here

**1 — stackup.** Both RF lines are 0.36 mm grounded coplanar waveguide over a 0.2104 mm 7628 prepreg. That width is only
50 Ω (45–50 Ω with copper thickness) on this exact stack, so a different template changes the match.

**2 — finish.** The finish reaches the `.gbrjob` only as a string that is easy to miss. HASL's coplanarity is wrong for
the 0.4 mm QFN and the WLCSP.

**3 — vias.** The nine thermal-pad vias follow Espressif's rule for the S3's only ground pad (HWDG PCB layout: "at least
nine ground vias"). They sit between the paste windows, so no paste lands on an open barrel. Every other via is tented
by the board setup.

**4 — the antenna window.** A fab or CAM operator who sees a void in the planes may "fix" it. The loop antenna works only
against that window (Abracon AANI-CH-0070 rev A: "no copper on any layer"), and filling it detunes the antenna by an
amount no one can predict.

**5 — mask dam.** The ESP32-S3 footprint's pin openings leave 0.07 mm between neighbours, under the usual 0.10 mm
minimum. The Tinker-Beetle was built with the same footprint and a ganged opening.

**6, 7 — minimums.** They are declared up front so the order doesn't sit in DFM review.

**Stencil.** The WLCSP sets the foil. The next finest apertures are the chip antenna's 0.30 × 0.30 mm pads (U15, AR 0.94
at 80 µm) and the ESP32-S3's 0.65 × 0.22 mm pins (U3, 1.03 at 80 µm, 0.82 at 100 µm). All of these figures come from
the board's paste layer, where each aperture matches its pad 1:1. The only bottom-side paste apertures are J8's two
ground pads, and J8 is hand-fitted, so no bottom stencil is needed.

**Assembly.**
- **U1:** its orientation is a known failure class on this fleet. A rotated WLCSP powers the flash backwards.
- **J2:** since 2026-10-03 it is the GCT USB4110-GF-A the other boards use. Its shell tabs are SMD pads with paste, so
  it reflows with everything else and nothing of it reaches the bottom side; two ground vias between the tabs tie the
  shell to the In1.Cu and B.Cu ground. The V1 boards carry the HRO TYPE-C-31-M-12 instead, whose shell legs are plated
  slots with no paste: those are soldered by hand, and the bottom fillets of the two rear legs must stay flat because
  the end of BT2's flat base sits over them.
- **S1:** its posts land inside the battery holder's outline. DRC is silent because `npth_inside_courtyard` is ignored
  on this board.
- **BT2:** a holder fitted backwards reverses the cell onto the protection circuit.

See [`design-review-2026-09-28.md`](design-review-2026-09-28.md) for the evidence behind each item.
