# gnss-px1105r-18mm-hv — fabrication and assembly notes

This folder's board is the PX1105R GNSS carrier, V1: the SkyTraq PX1105R L1/L5 receiver with the Space Bug's SAW–LNA–SAW–LNA
front end and a Taoglas GVLB208.A patch on the back, on the SAM carrier's host connector. The prototype is 4 layers on a
1.2 mm board. Written 2026-10-12 from the board file and the design review
([`design-review-2026-10-12.md`](design-review-2026-10-12.md), D1, D2, K1).

**Paste the block below into the fab's order notes verbatim.**

None of it can travel in the design files. The board thickness, the via treatment, the stencil foil, the reflow limits and
the hand work do not reach the fab through the gerbers or the drill files, so written text is the only channel. Stencil
thickness, the area-ratio floor and the paste-coverage convention are repo-wide and live in
[`hardware/SOLDER-PASTE-CONVENTION.md`](../SOLDER-PASTE-CONVENTION.md) (#959, #906).

---

```
TINKERROCKET PX1105R GNSS CARRIER V1 - FABRICATION AND ASSEMBLY NOTES
BOARD 22.0 x 30.0 mm, 4 LAYER, 1.2 mm NOMINAL.

1. STACKUP: JLCPCB JLC04121H-3313, 1.2 mm, 1 oz OUTER / 0.5 oz INNER.
   BUILD TO THIS TEMPLATE; THE RF TRACE WIDTH IS DRAWN FOR IT.
   OUTER PREPREGS 3313 AT 0.0994 mm (er 4.1), CORE 0.865 mm (er 4.6).
   LAYERS: F.Cu SIGNAL + GND POUR, In1 GND, In2 SIGNAL/POWER + GND POUR,
   B.Cu GND POUR (THE ANTENNA'S GROUND).
   *** THE THICKNESS MUST BE 1.2 mm, NOT 1.6 mm: THE PATCH ANTENNA'S
   FEED PIN IS SIZED TO IT. ***

2. SURFACE FINISH: ENIG. REQUIRED FOR THE 0.4 mm PITCH AMPLIFIERS
   (U3, U4), THE SMALL SAW FILTER LANDS (FL4, FL5) AND THE CASTELLATED
   GNSS MODULE (U1). DO NOT SUBSTITUTE HASL.

3. VIAS: 148 THROUGH VIAS, ALL 0.40 mm PADS ON 0.30 mm DRILLS,
   TENTED BOTH SIDES. STANDARD PROCESS: NO FILLING, NO CAPPING.
   NO VIA-IN-PAD: NO VIA SITS IN OR CUTS INTO AN SMD LAND, AND EVERY
   VIA'S COPPER IS AT LEAST 0.09 mm FROM ANY SOLDER MASK OPENING.

4. RF TRACES: 0.16 mm ON F.Cu WITH A 0.20 mm GAP TO THE GROUND POUR,
   OVER In1.Cu GROUND: 50 OHM ON THIS STACKUP. DO NOT ADJUST THE
   WIDTHS. THE LONGEST RUN IS 4.9 mm (ABOUT 17 DEGREES AT 1.6 GHz),
   SO IMPEDANCE TESTING IS OPTIONAL.

5. SOLDER MASK: U3 AND U4 EACH HAVE ONE OPENING OVER THE WHOLE PAD
   ARRAY ON PURPOSE (NON-MASK-DEFINED LANDS, AS THEIR MAKER
   SPECIFIES). DO NOT ADD DAMS THERE.

6. MINIMUM ANNULAR RING 0.05 mm (0.40 mm VIA PAD ON 0.30 mm DRILL).
   THE 1.00 mm ANTENNA HOLE (AE1) HAS A 0.20 mm RING ON B.Cu AND THE
   INNER LAYERS AND 0.60 mm ON F.Cu.

7. MINIMUM TRACK 0.10 mm (RULE 0.09 mm), MINIMUM SPACING 0.09 mm
   (0.20 mm AROUND THE RF TRACES). HOLE TO HOLE 0.20 mm MINIMUM,
   MEASURED HOLE EDGE TO HOLE EDGE. COPPER TO BOARD EDGE 0.20 mm.

8. OTHER HOLES: 4 x 2.20 mm PLATED (H1-H4), 1 x 1.00 mm PLATED (AE1).
   NO NON-PLATED HOLES.

ASSEMBLY - STENCIL
STENCIL FOIL 0.08 mm (3 mil), FLAT - NO STEP. TOP STENCIL ONLY;
THE BOTTOM HAS NO PASTE.
*** ORDER THE FOIL AS 0.08 mm. DO NOT SUBSTITUTE 0.10 OR 0.12 mm. ***
LASER CUT, ELECTROPOLISHED AND NANO-COATED.
PRINT THE PASTE LAYER AS SUPPLIED, APERTURES 1:1. U1 AND U2'S
EXPOSED PAD PRINT AT FULL PAD.
TIGHTEST APERTURES: U3/U4 0.25 mm ROUND, AREA RATIO 0.78 AT
0.08 mm (0.63 AT 0.10 mm, BELOW THE IPC-7525 FLOOR OF 0.66).

ASSEMBLY - REFLOW
ONE PASS, TOP SIDE ONLY.
*** U1 (THE GNSS MODULE) SETS THE CEILING: PEAK 240 C FOR 25-35 s
AND 60-80 s ABOVE 220 C, MEASURED AT U1. ITS MAKER SAYS THESE
SHOULD NOT BE EXCEEDED. ***
THE SAW FILTERS (FL4, FL5) ALLOW 30-70 s ABOVE 220 C, SO RUN
PEAK 235-240 C AT U1 WITH 60-70 s ABOVE 220 C, AND CHECK IT WITH A
THERMOCOUPLE ON THE FIRST ARTICLE.
U1 IS MSL 4: 72 h FLOOR LIFE, THEN BAKE AT 85 C FOR 8-12 h.
FL4 AND FL5 ARE MSL 3 AND ESD SENSITIVE (250 V HBM).

ASSEMBLY - PLACEMENT
- FIDUCIALS FID1-FID3, TOP SIDE (0.5 mm COPPER, 1.0 mm MASK OPENING).
- U1: PIN 1 IS AT THE SILK DOT. ITS 18 PADS ARE SYMMETRIC UNDER
  180 DEGREES, SO A ROTATED MODULE STILL FITS AND THE BOARD IS DEAD.
  CHECK U1 IN THE PLACEMENT PREVIEW.
- U2 IS ADP7142ARDZ-3.3 ONLY (TUBE ADP7142ARDZ-3.3, REEL
  ADP7142ARDZ-3.3-R7). THE ADJUSTABLE ADP7142ARDZ, INCLUDING LCSC
  C3745280, REGULATES AT 1.2 V ON THIS BOARD: DO NOT SUBSTITUTE.
  PIN 1 IS AT THE SILK DOT; A ROTATED U2 SHORTS THE SUPPLY.
- U3, U4: PIN 1 AT THE SILK TRIANGLE. A ROTATED PART PUTS 1.8 V ON
  ITS RF INPUT.
- D2 (SMF10A): CATHODE BAR TOWARD VSYS (PAD 1, J3 PIN 3).
- C19 IS DO-NOT-FIT.
- AE1 IS NOT FITTED BY THE ASSEMBLER (SEE HAND WORK).

ASSEMBLY - HAND WORK
- AE1 (PATCH ANTENNA) FITS AFTER REFLOW ON THE BOTTOM, HELD BY ITS
  ADHESIVE PAD (REMOVE THE RELEASE LINER). ON THIS 1.2 mm BOARD ITS
  PIN ENDS ABOUT FLUSH WITH THE TOP COPPER: 0.05 mm BELOW NOMINAL,
  BETWEEN 0.37 mm BELOW AND 0.28 mm PROUD ACROSS TOLERANCES.
- SOLDER IT FROM THE TOP: FLUX THE BARREL (DO NOT PRE-TIN IT), PRESS
  THE PATCH ONTO THE CLEAN BOTTOM MASK, THEN FILL THE BARREL UNTIL THE
  2.2 mm TOP PAD WETS FLUSH OR CONCAVE AROUND THE PIN. 360-380 C,
  3-4 s PER ATTEMPT, COOLING BETWEEN ATTEMPTS. DO NOT OVERFEED:
  SOLDER THAT RUNS THROUGH REACHES THE ANTENNA-SIDE PAD UNDER THE
  PATCH. U1'S BODY IS 0.24 mm FROM THE PAD: SHIELD IT WITH KAPTON.
- BEFORE THE TAPE GOES DOWN: IPA-CLEAN THE BOTTOM MASK, WORK AT
  21-38 C, PRESS FIRMLY AND EVENLY, AND LET THE BOND DWELL BEFORE
  SOLDERING.
- AFTER SOLDERING: DMM FROM THE PATCH'S TOP FEED BUMP TO C20 PAD 1
  WHILE TAPPING THE PATCH - LOW AND STEADY.
- THEN STAKE THE PATCH: NON-CORROSIVE ALKOXY-CURE RTV SILICONE
  (MIL-A-46146), NOT ACETOXY, NOT EPOXY. SMALL LOW FILLETS (1.5 mm OR
  LESS UP THE CERAMIC) AT MID-SPAN OF THE TWO LONG EDGES; KEEP IT OFF
  THE H1-H4 PADS AND INSIDE THE BOARD OUTLINE.
- FIT AE1 LAST: X-RAY AND ANY TEST THAT NEEDS THE AREA UNDER THE
  PATCH COME FIRST.
```

---

## Why each item is here

Every figure in the block was measured from the board file on 2026-10-12, after the 4-layer conversion and the review
fixes that followed it.

**1 — stackup.** The RF line is grounded coplanar waveguide over 0.0994 mm of 3313 prepreg, the same top dielectric as the
6-layer JLC06161H-3313 build it was drawn on. A 2-D field solve put the 0.16 mm track with its 0.20 mm gaps at 49 Ω under
mask and 51 Ω bare (Space Bug notes), so the width carries over unchanged. On JLC's standard 4-layer stacks (0.2104 mm of
7628 prepreg) the same copper would be over 60 Ω, so the template is part of the design. JLC04121H-3313 is JLC's 1.2 mm
4-layer stack with that prepreg (jlcpcb.com/impedance, read 2026-10-12): Cu 35 µm / 3313 0.0994 / Cu 15.2 µm / core
0.865 / Cu 15.2 µm / 3313 0.0994 / Cu 35 µm. The board setup carries it, so the `.gbrjob` says `BoardThickness: 1.2252`
(the sum includes both masks).

The thickness is also mechanical. The GVLB208.A's feed pin stands 1.2 ± 0.2 mm below the ceramic (GV-DS rev B p6), 1.15 mm
beyond the adhesive face once the liner is off. On the 1.6 mm board it stopped 0.42 mm below the top copper (review K1). On
1.2 mm the top copper is 1.195 mm above the bottom mask face, so the tip sits 0.05 mm below it nominally. Across the pin's
±0.2 mm and JLC's ±10 % board thickness, that spans 0.37 mm below to 0.28 mm proud. A thicker board brings the blind
barrel fill back.

**2 — finish.** The finish reaches the `.gbrjob` only as a string that is easy to miss. HASL's coplanarity is wrong for the
0.4 mm-pitch TSNP amplifiers.

**3 — vias.** JLC fills and caps vias by default only from 6 layers up, so this 4-layer build has no via-in-pad: an
unfilled barrel in or beside a land wicks solder out of the joint above it. Every via was checked against every pad's
mask opening and against the single mask windows that U3 and U4's footprints draw over their pad arrays. Each via's
copper is at least 0.09 mm from any opening, JLC's guidance for copper beside a mask opening, so the tent covers the
whole ring. U2's exposed pad has no vias in it; six GND vias sit 0.42–0.51 mm outside it. The board setup declares
tented-only (no filling, no capping), but the written note is what the fab reads.

**4 — RF.** 14 nets, 24.2 mm in all. The longest, the feed from AE1 to the ESD high-pass, is 4.9 mm, about 17 degrees at
1.6 GHz. A 10 % impedance miss on it costs under 0.01 dB, so the width matters more than a test coupon.

**5 — mask.** U3 and U4's single window is deliberate (Space Bug review S10).

**6 — AE1's ring.** The antenna maker's land is a Ø1.20 mm pad on the antenna side over a Ø1.00 mm hole, a 0.10 mm ring,
below JLC's 0.15 mm PTH minimum (review S5). The footprint now has Ø1.40 mm on the antenna side and the inner layers;
the Ø2.20 mm solder-side pad, the drill and the Ø3.0/Ø3.5 mm copper keep-outs are unchanged.

**7, 8 — minimums and holes.** These are the board's design rules, matching the Tinker-Mantis and the Tinker-Beetle; they
are stated up front so the order does not sit in DFM review.

**Stencil.** The amplifiers set the foil. Their 0.25 mm round apertures, the maker's recommended stencil opening, reach area
ratio 0.78 on an 80 µm foil and 0.63 on the 100 µm repo default, below the IPC-7525 floor of 0.66. They clear 0.66 only on a
foil of 94.7 µm or thinner, so 80 µm is the next standard step. JLC picks from 0.10/0.12 mm foils by default; 0.08 mm has to
be named in the order. The next finest apertures:

| Part | Area ratio at 80 µm |
|---|---|
| U3, U4 (0.25 mm round) | 0.78 (0.63 at 100 µm) |
| FL4, FL5 | 1.04 |
| D6 | 1.23 |
| FL1 | 1.58 |
| U1 | about 2.6 |

Everything else is at area ratio 1.9 or more. U2's exposed pad prints one full aperture (review S3 left windowing open;
at 80 µm a full print is not an excess). Nothing on the bottom takes paste: the patch's pin is soldered by hand.

**Reflow.** U1's limits come from PX-DS rev 2 p12, which says they should not be exceeded. The SAW filters (B8389 v2.1
Table 3) allow a 250 °C peak but only 30–70 s above 220 °C, so the window that satisfies both is 60–70 s above 220 °C. U1's
MSL 4 and bake come from PX-DS p13. The BGA855N6 datasheet (Rev 2.2) states no MSL.

**Placement.**
- **U1:** the module's 9 + 9 castellations are symmetric, so a rotated part fits and puts VCC on an open pad (review L1).
  The silk dot beside pad 1 was added for this.
- **U2:** the fixed and adjustable ADP7142 differ only in the suffix, and the only LCSC listing in stock is the
  adjustable part (review D2). The pin-1 dot moved off C1's pad, where it did not print.
- **U3, U4:** the BGA855N6's AI pin is rated 0.9 V absolute maximum.
- **Fiducials:** board-only footprints, excluded from the BOM and the placement file. JLC adds rail fiducials as well.

**Hand work.** GV-DS p14 gives hand soldering at 360–380 °C for 3–4 s and no reflow profile, so AE1 stays out of the
assembly order: leave it out of the assembly BOM and CPL, or mark it do-not-place. Taoglas gives only the iron
temperature and time; the rest of the procedure comes from the review (K1, M5, K3):
- **The joint is the only RF path into the receiver.** The feed net is DC-isolated from ground (C20 in series, D6 a
  TVS), so a pin-to-ground reading proves nothing. The drawing shows the feed as a bump on the patch's top electrode, so
  a DMM from that bump to C20 pad 1 tests pin, joint, barrel and trace. Confirm on a loose patch first that the bump is
  the pin (inferred from GV-DS p6). An unwetted pin can still couple about 0.3 pF and pass a bench C/N0 check, then open
  under vibration. That is why the method is qualified once on a spare bare board with a real patch, then sectioned
  through the pin and inspected at 10–20×. Accept it when the pin end is wetted and the barrel is filled over at least
  half its depth with no blowhole.
- **Tape.** 3M 9077 (TDS, Nov 2006) asks for clean surfaces, firm pressure and application at 21–38 °C. Its 90° peel is
  11 N/cm on glass-epoxy. The flight logs show 118–146 g ejection and landing shocks, against which the tape alone has
  about 1.5–2× margin by estimate, hence the staking.
- **Staking.** RTV to MIL-A-46146 as on the Tinker-Mantis (its fab notes, B3). Acetoxy cure releases acetic acid, and
  epoxy is rigid against a ceramic. The patch's corners are 0.9–1.2 mm from the H1–H4 lands and its long edges about
  1.0 mm from the board edge, hence fillets at mid-span. Compare C/N0 before and after staking.
- **Order.** Once fitted, the 8 mm ceramic sits in the X-ray path of about 30 top-side parts, including most of U1 and
  U2, and the tape and joint are one-shot, so the patch goes on last (review M13).

## Bring-up reference (not for the fab)

J3 as built, from the netlist (JST BM05B-SRSS-TB, pin 1 marked "1"):

| J3 pin | Net | Direction |
|---|---|---|
| 1 | GNSS_RX via R7 (1 kΩ) to U1.3 RXD | host TX → receiver |
| 2 | GNSS_TX via R8 (1 kΩ) from U1.2 TXD | receiver → host RX |
| 3 | VSYS (pack) via FL1 to U2 | power in |
| 4 | VSS → FL1 → GND | return |
| 5 | RXD2 via R9 (1 kΩ) to U1.15 | host → receiver (RTCM input) |
| 6, 7 | GND (mounting tabs) | — |

1PPS reaches only D1. Review S1 asks whether pin 5 should stay RXD2 or carry PPS.

## Open before ordering

- **U1's pin 1 against its own marking.** The datasheet's top view and its rendered photo may put the shield dot at
  pin 10's corner, not pin 1's (review L1). Confirm on a sample (pins 1, 10 and 12 are GND and common with the shield;
  pin 3, pin 12's 180° partner, is not) or ask SkyTraq, and add the answer to the placement section.
- **J3 pin 5** is RXD2 as built (review S1); 1PPS is not on the connector.

## Change history

- 2026-10-12 V1 prototype (tag `gnss-px1105r-18mm-hv-v1.0.0`): 4 layers on JLC04121H-3313, 1.2 mm; supply routes moved
  off B.Cu under the patch; fiducials, pin-1 marks for U1 and U2, AE1 feed pad ring 0.20 mm; no via-in-pad, tented
  vias, every via's copper at least 0.09 mm from a mask opening.
