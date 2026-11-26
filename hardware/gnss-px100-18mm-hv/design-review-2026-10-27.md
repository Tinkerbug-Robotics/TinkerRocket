# PX100 GNSS carrier (gnss-px100-18mm-hv) — design review, 2026-10-27

**Board:** V1 as fabricated (tag `gnss-px100-18mm-hv-v1.0.0`) plus the parts changes of 2026-10-26 (L13 not fitted,
C37 1.3 pF, C48 0 Ω). The working files match the ordered gerbers line for line apart from header attributes, so
everything below applies to the bare boards on hand.

**Question:** will this board work as designed, especially the RF front end, and what can still be fixed without new
copper?

**What I used:**
- the schematic, board and rules from the working tree, exported to a scratch copy (KiCad 10.0.3: netlist, BOM, DRC
  with parity, ERC, pad/track/zone dumps, layer plots);
- SkyTraq PX100-5C-01A and PX100-5S-02A data sheets (v2, "Preliminary Draft"), including the reference schematic,
  RF reference and BOM pages;
- Infineon BGA855N6 DS rev 2.2, AN596 rev 1.3 (the L1/L5 retune), AN580, AN619; BGA524N6 DS rev 3.5;
- Qualcomm RF360 B39162B8389P810 v2.1 and B39162B4327P810 v2.2; Murata SAFFB1G17AA0F0A rev D; Tai-Saw TA1573A and
  TA2471A, Golledge MP05419 (for the TinkerNav comparison);
- Murata LQW15AN, GJM/GRM1555, DLW21SN, BLM18PG, LQM18PN; ADI ADP7142 rev H; TI LP5907 and TPD1E0B04; onsemi
  NSR0320; Epson TG2016/2520SMN v2.0 and TG-5006CG; Seiko SC-32S; RF Solutions CON-SMA-EDGE; Laird BMI-S-202;
- the JLC order record for V1 (order Y349-3297386A, read from the order history page on 2026-10-27);
- the PX1105R carrier and its two reviews, the legacy Space Bug, and the TinkerNav PX100 V0/V1/V2 KiCad files.

**Method:** nine specialist passes (stage 1, stage 2, a network simulation of the whole RF path, RF layout, PX100
core, power and host, footprints and assembly, a cross-board failure hunt, bring-up), each re-derived by an
independent verifier, with a separate refutation attempt on every major finding, then a completeness pass that added
five more checks (receiver firmware state, the L1-to-L5 dependence, stability, the as-ordered fab options and the open
PX100 questions). What survived that is below; where a verifier corrected a number or a fix, I use the corrected one.

---

**Update 2026-10-28.** The owner confirmed the stackup was left at JLC's default, so the V1 boards are JLC04161H-7628.
Two recommendations below change with that:
- **L1:** keep 5.1 nH on these boards. On 7628, 3.3 nH gains only about 0.2 dB at L1 in the model (3.08 to 2.87 dB)
  and nothing at L5. It is worth fitting only on a 3313 board.
- **Re-order:** keep JLC's default stackup and order only ENIG and Epoxy Filled & Capped. On this copper the model puts
  7628 ahead of 3313 (noise figure 3.08 / 2.18 dB against 4.06 / 2.54 dB at L1 / L5), because J1's land is half as
  capacitive and the RF runs are too short for the 68 Ω lines to matter.

## Verdict

**I found no design error that would stop this board from working.** The circuit is right where the earlier boards
went wrong:
- Every pin map and land pattern matches its data sheet in the right view: the PX100 QFN-40, both BGA855N6s, the
  three RF360 SAWs (pad 1 in, pad 4 out, 2/3/5 ground, top view), the Murata L5 SAW (port 1 on the lone mid-left land),
  the Epson TCXO, the LDOs, the common-mode choke, the Schottky and the LEDs.
- Stage 1 is Infineon's AN596 L1/L5 retune value for value (9.1 nH in, 2.4 nH out, 1 nF DC blocks, 1.8 V), with the
  B8389's 5.1 nH port shunts.
- Every DC node is right. The BGA855N6 output is DC-blocked inside the package (DS p3 block diagram; AN596: "DC block
  is NOT integrated at input"), so the 5.1 nH shunt at FL5's input shorts nothing. C22/C24 keep DC off the LNA inputs
  (0.9 V absolute maximum). Every SAW port sits at 0 V.
- Stage 2 is SkyTraq's reference split, band filters and L-matches. The 2026-10-26 change holds up: 1.3 pF equals
  4 pF in parallel with 6.8 nH at 1176.45 MHz within C37's own tolerance, and pin 37 now has no DC path to ground.
- The simulated chain gives about +22 dB (L1) and +27 dB (L5) into the PX100 and a system noise figure of about
  3 dB (L1) and 2.2-2.5 dB (L5), close to the README budget. The fitted models show both LNAs stable in this layout,
  with a thin margin that public data can't pin down (section 3, item 6).

**What I did find matters more than any circuit detail:**

1. **The boards were not ordered as the notes say** (section 1). The JLC order reads Surface Finish "HASL(with lead)",
   Via Covering "Plugged" and Specify Stackup "No requirement". So the board is HASL, not ENIG; U1's five exposed-pad
   vias (the PX100's only ground) are not filled and capped; and the stackup is probably JLC's default 7628, not 3313.
   The RF consequence is small. The assembly consequence for a 0.4 mm-pitch QFN, two TSNP-6 LNAs and 1.1 × 0.9 mm SAW
   lands is not.
2. **U1 pin 14.** The pin table says "NC, leave unconnected", but both SkyTraq reference schematics (5C and 5S) and
   both BOMs put a 2.2 µF capacitor from pin 14 to ground. This board, and both TinkerNavs, leave it open (section 2).
3. **The bring-up plan can't tell a dead front end from a receiver in the wrong state.** On this same Phoenix silicon,
   a fallback image, and a saved kinematic-base setting, have already produced "talks on the UART, 0 dB-Hz on every
   channel". The plan needs a firmware and configuration check before any RF verdict, and it should prove stage 1
   on its own before a PX100 goes on (*Bring-up*).

**Why the earlier boards failed, as far as the files show:**
- **TinkerNav PX100 V2:** both LNA inputs are DC-grounded. U5 (BGA524N6) and U6 (BGA855N6) take their input from
  the B8389's output node, which carries the mandatory 5.1 nH shunt to ground, with no DC block. Both PX100 inputs
  also have 5.1 nH shunts to ground. Either is enough to explain "tracked nothing". This board has neither.
- **TinkerNav PX100 S3 V0/V1:** the L1 SAW (MP05419) footprint swaps the output and a ground pad against the part's top
  view (it follows the data sheet's mirrored measurement-circuit drawing). The L5 SAW on the same boards is right.
  That damages the L1 path, though it does not by itself explain "BeiDou B1I but no GPS L1": B1I and GPS L1 share the
  same SAW passband and L-match, so that split points at receiver configuration or firmware state. This board uses
  the RF360 footprint, which is right.
- **PX1105R carrier V1:** still no design fault in its stage 1, which this board copies. Its 0-satellite run is
  confounded: the V9 image sent the kinematic-base setup at every boot, its firmware version was never recorded, and
  it was built on the same HASL / plugged-via order with a 0.10 mm stencil (TSNP area ratio 0.625). Reading that
  carrier's firmware version (step 0 below) is the cheapest test in this whole review.

---

## 1. The V1 order (fix before committing a PX100)

**What JLC built:** HASL with lead, vias "Plugged", no stackup specified, 1.6 mm, 1 oz / 0.5 oz, TG135. The fab
notes, the README and `gerbers/README.txt` all describe ENIG, epoxy-filled-and-capped vias and JLC04161H-3313. Those
were the settings I listed on 2026-10-23; the order was placed without them, and the records then copied my list.

**What it does:**
- **HASL on fine pitch.** JLC gives HASL as 1-40 µm, uneven across a pad, and "less suitable for fine-pitch". That is
  the same order as the 0.08 mm stencil's printed solder. Risks: bridges on U1's 0.18 mm gaps, tilt or opens under
  U3, U4 and FL7 (hidden, RF-critical joints), and a domed exposed pad under U1.
- **U1's exposed-pad vias.** JLC states that in-pad vias, and vias closer than 0.35 mm to a pad, can't be ink-plugged.
  So the five 0.3 mm vias in pin 41 are open on top (maybe partly HASL-filled or domed) and tented underneath. With the
  tent intact little solder wicks in, but expect voids and outgassing, and domes would lift the part. The barrels still
  ground the pad.
- **Stackup.** JLC's "No requirement" record for 4 layers, 1.6 mm is JLC04161H-7628 (0.21 mm prepreg, er 4.4). On
  7628 the 0.16/0.2 mm lines are about 68 Ω instead of 49 Ω, but every RF run is under 3.5 mm (about 3 % of a
  wavelength), so that costs little. J1's center land drops from about 2.1 pF to about 1.1 pF, which actually helps
  (section 3, item 1). Board thickness can't tell the two stackups apart; capacitance to ground can (RF_IN on a bare board: about
  2.9 pF on 3313 against 1.8 pF on 7628), or ask JLC support which template the order used.

**My recommendation:** re-order the same gerbers with ENIG, Epoxy Filled & Capped and Specify Stackup =
JLC04161H-3313 (ENIG and filled-and-capped vias are paid options on 4 layers; the 0.08 mm stencil on hand still
fits), and check the Product Detail before paying. Use the HASL boards meanwhile for a stage-1-only build (*Bring-up*, step 1), which needs no PX100
and doesn't depend on the exposed pad.

**If you build a PX100 on a HASL board anyway:**
1. Inspect the U1 exposed pad at 10-20×. Flush or slightly dimpled holes: build as is. Solder domes over the holes:
   level them with flux and braid. Open holes: seal from the top with a little solder or UV-cure mask scraped flush.
2. Flatten the HASL on the fine-pitch lands with flux and braid (U1, U3, U4, FL4-FL7, Y1), clean with IPA, then print.
3. After reflow: U1 must sit flat with even fillets; look at U3, U4 and FL7 from the side. Check the exposed-pad joint
   electrically: diode mode, red probe on board GND, black on a U1 I/O (TXD1 at R8's U1-side pad) should read a
   0.4-0.8 V clamp drop; OL means the pad isn't joined.

---

## 2. U1 pin 14 (measure at bring-up, probably fit)

Both reference schematics put C21 2.2 µF from pin 14 to ground: at 4× zoom the C21 line meets the pin named "NC"
between GPIO9 (15) and GPIO6 (13), which carry no-connect crosses, and both BOMs list C21. The pin table says leave it
unconnected. A drawing slip (C21 meant for VDD11) is possible, but two data-sheet revisions repeat it. Leaving it open
doesn't stop the chip booting (both TinkerNavs boot), but none of the three PX100 designs, all with pin 14 open, has
ever tracked GPS. If it is an internal regulator or reference node, an open pin can cost C/N0.

**On the printed board:** at first power-up, meter pin 14's exposed toe (x 59.02-59.375, y 40.885) and scope it.
A level that matches no external rail (about 0.9-2.5 V), or ripple, means fit the capacitor. 0 V or 3.3 V is
ambiguous (a GPIO pull could read the same): load it briefly with 100 kΩ and see whether it holds. To fit it, don't
put an 0402 terminal on the toe; at 0.4 mm pitch it bridges pins 13 and 15. Tack a 0.1 mm enameled wire to the toe and
land an 0402 2.2 µF (CL05A225MQ5NSNC, already on the BOM) with its ground end on scraped F.Cu pour right of pins 12-13
(about y 41.8-42.6, below the GND via at (59.735, 41.478)). Do one board with the cap and one without for an A/B.

---

## 3. RF front end

**Correct, and checked:** the items in the verdict, plus:
- the lumped Wilkinson is centerd on L1 (6.8 nH / 1.5 pF resonate at 1576 MHz): about 3.0 dB per arm at L1 and
  3.95 dB at L5, input match 21 / 11.5 dB, isolation 27 / 12.6 dB;
- L12 10 nH at FL7's input cancels the SAW's input capacitance at 1176 MHz, so it belongs there; it is worth about
  0.4 dB on the L5 arm;
- FL6 (B4327) needs no matching at 50 Ω and rejects about 45 dB at 1176 MHz; FL7 covers 1164-1189 MHz at 1.1 dB;
- out-of-band rejection to the pins is at least 70-80 dB from 700 to 1100 MHz and above 1700 MHz;
- the drive into the PX100 (about +17-25 dB) sits inside the range SkyTraq's own modules imply, and the system noise
  figure barely depends on the PX100's own noise figure (3, 6 or 10 dB adds 0.01-0.15 dB);
- In1 ground lies under every RF segment and pad; the 0.16 mm line is 50.5 ± 1 Ω on 3313 (field solve);
- the high-gain output side is 5-8 mm from the input side over ground; estimated coupling is well under -55 dB.

**Items, in order of value:**

1. **J1's center land is a 2.1 pF shunt at the antenna input** (on 3313; about 1.1 pF on 7628). The 1.5 × 3.5 mm land
   sits 0.1 mm over solid In1, In2 and B.Cu ground, a 10.5 Ω line section. It cancels most of L1, the 5.1 nH shunt that
   matches FL4's input, and costs about 1 dB of noise figure at L1 on 3313 (less on 7628). **Same-lands fix:** change
   L1 from 5.1 nH to **3.3 nH** (for example Murata LQW15AN3N3B80D, 0402). On the model that value helps or is neutral
   on both stackups: noise figure L1/L5 3.08/2.18 → 2.87/2.19 dB on 7628, 4.06/2.54 → 3.66/2.32 dB on 3313. Don't fit
   the 2.4 nH some passes proposed unless the stackup is confirmed as 3313: on 7628 it costs 0.4 dB at L5. With a
   NanoVNA, measure S11 at J1 at low drive and pick from 2.7-3.9 nH instead.
2. **J1 sits 1.57 mm further out than its footprint assumes.** The footprint expects the board edge at the land's
   start (y 27.70); the edge is at y 26.13. With the flange against the edge, the 3.81 mm pin overlaps the land by
   2.24 mm (enough to solder) and crosses 0.49 mm of F.Cu ground pour (y 27.015-27.502) covered only by solder mask.
   Before fitting J1, check that mask at 10× and dab UV-cure mask on it if in doubt; slide J1 on squarely; afterwards
   J1 center to shell must read open (C20 blocks DC, D6 leaks nA). Repeat after heavy use. Don't file a notch: it
   would reach the GND via at (50.85, 27.58).
3. **FL6's DC state decides whether L1 can be lost silently.** Pin 40 reaches FL6 pad 4 through L14, and FL6's input
   side is at 0 V through L10/L11/L12. So any DC path in the B4327 from pad 4 to its ground pads, or to pad 1, grounds
   pin 40. The stock firmware opens L5 channels only for satellites it already tracks on L1 (seen on the rig: L5-only
   replays never open an L5 channel), so losing L1 loses both bands. **Gate before the first RF test:** meter a loose
   B4327 from the reel: pad 4 to 2/3/5 and pad 4 to pad 1 must read open; its marking must read 477. If it reads a
   short, fit a TA1573A (rated 3 V DC; centerd, it covers about 75 % of the outer lands) or an MP05419 (same lands;
   meter it first). The README's "no parts-only change fixes it" is wrong for that reason.
4. **The B4327 is rated for 0 V DC but its output is DC-coupled to pin 40.** If bring-up finds a bias voltage on pin
   40, FL6's output carries it permanently. SkyTraq's reference has the same topology with a TA1573A rated 3 V DC.
   Short-term bench use is fine. To comply, cut the 1.06 mm FL6-OUT segment between C36 and L14 ((51.688, 45.843) to
   (52.438, 45.093)) and bridge it with an 0402 33 pF; the match doesn't change (|Z| 0.6 Ω at L1). V2: a 33-100 pF
   series cap between each band SAW and its series inductor.
5. **The shared SAW land can be fitted wrong in two ways.** FL4, FL5 and FL6 use the same land. A B4327 on FL4 or FL5
   takes out L5; a B8389 on FL6 costs about 2.7 dB on the L1 arm (under 0.1 dB of system noise figure). Rotation is the
   real risk: FL6 turned 180° puts its pin 1 across land 4 (output) and land 3 (ground), which kills L1 and therefore
   both bands; FL4 or FL5 rotated kills both bands directly. Check markings (865 on FL4/FL5, 477 on FL6) and each pin-1
   dot against the silk dot, before and after reflow.
6. **U3's supply pin has no RF bypass.** C23 sits on U3's PON pin (pin 6). U3's VCC (pin 2) reaches a 1 nF only through
   1.12 mm of 0.12 mm track, a via, 1.6-1.8 mm of In2 and another via: about 1.9-2.8 nH, 14-24 Ω at L5. The same via
   feeds U4's PON. U4 is fine (C25 is 1.0 mm from its VCC pin). The README's "C23/C25 now bypass each LNA" is true for U4
   only. Infineon calls the VCC capacitor optional and the device is designed for k > 1, so I don't expect oscillation,
   and the fitted models agree; but the device's stability margin is thin (mu about 1.0-1.1 in AN580) and those models
   can't predict even the sign of the effect, because Infineon's S-parameters are behind a login. **On the printed
   board:** power up as is and run the oscillation sniff (*Bring-up*, step 1). If a line appears that comes and goes
   with VSYS, or as a deliberate A/B, stand an 0201 1 nF upright from the VCC trace corner (about (46.00, 38.36-38.48),
   scrape 0.15 × 0.2 mm of mask) to the exposed top of C25 pad 2 (y 38.78-38.93). It clears U3 pins 1 and 3 by only
   about 0.18 mm, and a bridge to pin 3 shorts +1V8 through L4 + L5, so ohm +1V8 to GND before and after.
7. **The L-matches are not at the pins** (C36 sits 1.7 mm before L14, and L14 is 2.9 mm from pin 40; L15 is 1.6 mm from
   pin 37). The model says a retune (L14 6.8 nH, L15 10 nH) would add 1-3 dB of gain, but it rests on a PX100 input
   impedance inferred from SkyTraq's own L-match, and SkyTraq's evaluation-board line lengths are unknown, so even the
   sign is uncertain. With 20+ dB ahead of the pins it is worth at most about 0.1 dB of system noise figure. **Keep
   SkyTraq's values on V1**; an A/B on the rig is optional.
8. **Overdrive and DC on J1.** The PX100's LNA inputs have a 0 dBm absolute maximum. With +22.7 / +27 dB of gain, an
   in-band CW of about -23 dBm (L1) or -27 dBm (L5) at J1 reaches it, before U4 compresses. A HackRF straight into J1,
   or the rig at full CW with its amplifier on, can supply that; replayed sky files can't. D6 sits ahead of C20, so J1
   takes only ±3.6 V DC (D6's working voltage): a HackRF bias of 3.0-3.3 V is fine, 5 V gear is out of spec, and about
   6.7 V or more can latch D6 into snapback and burn it short. Rules: no TX amplifier, CW tests with at least 40 dB of
   pad (J1 at -50 dBm or below), DC block on anything that carries more than 3.3 V.
9. **The rig level into J1 is undocumented.** The rig was calibrated by C/N0 on receivers with no LNA (the PX1105R
   module's C/N0 stops improving from gain 36), so it sits at least about 10 dB above kT, and this board adds its own
   22-27 dB. That leaves the PX100 about 7-12 dB under the top of SkyTraq's documented range, but a hotter rig would
   overdrive it, and overdrive "looks exactly like too cold". Start about 20 dB colder than the standard L1 40 / L5 36
   (or a 20 dB pad after the DC block), step up in 3 dB steps and log C/N0 per signal; use the plateau. Injecting at the
   split input (README step 5) needs active-antenna level there, or the PX100 runs 28-33 dB under its operating point
   and "no tracking" proves nothing.
10. **Smaller layout items, V2 only:** the LNA ground pins reach their vias through 0.5-1.7 mm of pour; FL7's center
    ground pad has no via of its own; L3 (U3's input inductor) and L5 (U3's output shunt) sit end to end with a 0.34 mm
    gap, a weak magnetic path around U3; 26 MHz harmonics land in band (45 × 26 = 1170 MHz in L5, 60 × 26 = 1560 MHz on
    B1I), which is normal for a 26 MHz receiver, and Y1's path is short and far from the inputs.

---

## 4. PX100 core

**Correct, and checked:** all 41 symbol pins against Table 2; the QFN land (0.22 × 0.83 mm, 0.4 mm pitch, 3.4 mm
exposed pad) and its placement (pin 1 at the silk dot, pins 31-40 facing the chain, a rotation, not a mirror); every
LDO output and its capacitor value against the reference BOM; R11 5.1 Ω + C38; L16 is the exact part SkyTraq names;
D7's cathode on DCDC_OUT; the TCXO's 3.0 V supply band, pinout and grounded pin 1 (Epson allows it); the RTC crystal
with no load caps (as the reference); the 10 k / 1 µF reset (about 10 ms, far over the 10-clock minimum); BOOT_SEL 1 k
to +3V3; UART direction and levels; the 1PPS load; unused pins left open as the reference's crosses show; current about
131 mA on +3V3. The core matches the TinkerNavs, which do boot.

**Items:**
1. **Pin 14:** section 2.
2. **C48 = 0 Ω is right; change the fallback rule.** SkyTraq's reference and both booting TinkerNavs drive TCXO_I
   directly. Epson specifies its output through a ≥10 nF DC cut, but if TCXO_I isn't self-biased, a capacitor there
   could stop the clock. README step 2 says to refit C48 as 1 nF if the UART is silent. Silence has other causes (a
   saved "no output" setting, wrong baud, BOOT_SEL or RSTN low), so decide on C48 from the scope at pin 4 alone. If the
   clock is bad: check 3.0 V at Y1 pad 4 and Y1's output at pad 3 with C48 lifted into 10 kΩ, and only then try 10 nF
   (Epson's value) in C48.
3. **VDD11 (pin 16) has no nearby capacitor.** Its nearest, C40/C41, sit by pin 23, 10.65 mm and two vias away
   (about 4-5 nH). SkyTraq's reference has no capacitor at pin 16 either, so this is good practice rather than a
   deviation. If bring-up shows spurs or instability: an 0402 1-2.2 µF across the scraped caps of the VDD11 via at
   (59.820, 40.650) and the GND via at (59.735, 41.478), 0.83 mm apart. It shares that pour with the pin 14 bodge.
4. **VDD_RF has three 1 µF capacitors where the reference has six**; VDD_PLL shares C50 with VDD_ANA1 and the two LNA
   supplies share C53. The positions are good (1.0-1.4 mm). SkyTraq publishes no minimum; V2: back to six.
5. **The DC-DC switch node is long:** L16 is 3.8 mm and D7 5.2 mm of track from pin 21 (about 1.4 nH), not "tight" as
   the README says. A possible spur source; nothing points at harm. If the rig with no signal shows CW or jamming
   reports near L1/L5, probe there with the cover off. V2: L16 and D7 within 1 mm of pin 21, C39 at pin 22.
6. **UART back-feed.** With VSYS off and a USB-UART idling high on J3.1, about 1.2-2.7 mA flows through R7 (1 k) into
   RXD1's clamp and the +3V3 rail, so a "power cycle" during bench work isn't clean (C56 and RSTN stay partly charged).
   Raising R7 alone breaks the UART: R14's 10 k pull-up sits on the receiver side, so R7 = 4.7 k puts a host low at
   1.06 V, above VIL (0.8 V). That also means the PX1105R review's "R7 to 4.7 k" must not be copied here. **Same-lands
   fix:** R7 → 10 kΩ and R14 → 100 kΩ together (host low = 0.30 V; back-feed 0.27 mA; about 80 ns edge, fine to
   460800 baud; use R7 = 4.7 k with R14 = 100 k above that). Worth doing on the board used for firmware work.
7. **BOOT_SEL is reachable only at R13 pad 2 (53.59, 36.755), inside the shield frame**, with no test pad. Strapping it
   low there selects the ROM boot-loader, but on the PX1105R the cold ROM was echo-only, so don't treat it as a proven
   recovery path. Prove a reflash round trip (or use the RAM-run route) before any flash experiment. V2: BOOT_SEL and
   UART test pads.
8. **V_BACK on +3V3** means every power-up is a cold start, and a slow brown-out may not reset the chip through the RC.
   Fine for a bench board.
9. **TCXO and RTC layout:** In2 tracks run under Y1 and Y2, and F.Cu ground pour sits between Y1's pads, against the
   Epson/Seiko "no pattern under the part" notes; Y2's land is smaller than Seiko's. Info for V2. The README's
   "2 ppb/g guaranteed" for the TG2520SMN isn't in Epson's data sheet.

---

## 5. Power and host

**Correct, and checked:** J3's pinout against the Mantis J1 and the stated host pinout; D2's standoff for 2S; FL1 wired
as a common-mode choke (windings 1-2 and 4-3 end to end); the ADP7142ARDZ-3.3 pinout, EN tied to VIN, soft start
1.53 ms; the LP5907 pinout and load; FL2's drop (13-34 mV) and its resonance (about 22 kHz, Q about 3), both harmless.

**Items:**
1. **Heat at 8.4 V:** U2 dissipates about 0.65 W and the board about 1.1 W. In open air at room temperature
   everything stays inside its rating with 10-20 K to spare on the 85 °C parts; flat on foam or in a closed box it
   doesn't. On the bench feed VSYS at about 5 V (U2 then 0.21 W; minimum about 3.7 V), keep the board off the bench,
   and log U2's ground pin and the shield temperature at first power-up.
2. **D2's polarity rests on the silk alone.** The symbol has A1/A2 pins, and Vishay numbers pin 1 as the anode, the
   reverse of KiCad's D_SMF footprint. The copper and silk are right (cathode, the band, at the VSYS end, y ≈ 53.15).
   Before first power: diode mode, red on VSS and black on VSYS reads 0.6-0.8 V, open the other way. Reversed, D2
   shorts the pack. V2: a polarized symbol.
3. **D2 caps VSYS at about 10 V**, so no 3S pack.
4. **The board's ground reaches the host only through FL1's return winding.** It works; the side effects on the UART
   return and the PPS edge are small.
5. **C28 (1 µF 16 V X5R 0402) at the LP5907 output** is close to TI's 0.7 µF effective minimum at the tolerance and
   temperature corners. Info.
6. FL1 on ENIG would break Murata's "no gilded land" note; on these HASL boards it doesn't apply. Keep it in mind for
   an ENIG re-order.

---

## 6. Assembly notes beyond section 1

- **L13's paste.** The V1 stencil still prints paste on L13's lands, one of them ground. A bridge shorts FL7's output
  and grounds pin 37 through L15. Tape over those two apertures with Kapton or wipe the paste off, and inspect before
  the frame goes on. (On an assembled board, FL7-OUT to ground won't read open, because L15 reaches pin 37; the pass
  criterion is "not a short".)
- **The BOM.** A grouped export (`kicad-cli` grouping by value and footprint) puts L10, L11 and L13 on one 6.8 nH line
  flagged DNP. The project's own BOM preset doesn't group them that way, but check whatever list you build from: L10 and
  L11 are fitted.
- **U4's pin-1 triangle** sits about equally between U4 pin 1 and U3 pin 3. A rotated BGA855N6 puts +1V8 on its input.
  Place U3/U4 by the part's marking against pad 1, not by the triangle alone.
- **The shield frame SH1.** Its pad ring is solid to a pour stitched to all four layers, so it needs a lot of heat, and
  L2, C23, C43, C38, C34, Y1 and D6 sit 0.43-0.55 mm from it. Its cross brace and 6 mm pick-up disc cover L15, C53,
  C49, U1 pins 31-40, and L14's pin end. So: keep SH1 off through RF bring-up and any match tuning; then fit it with
  bottom heat (preheat the board to 150-180 °C), iron on the ring, never top-side hot air alone; bake first if the
  SAWs are past their MSL3 floor life; re-check current and rails afterwards. After the frame is on, C50.1
  (54.57, 43.855) is the reachable VDD_RF probe point.

---

## Bring-up

Each step localizes one part of the chain. Do them in order; the first that fails names the fault.

0. **The PX1105R carrier first (free).** Send it 0x02 and 0x03 and log the version and CRC. If it reports the fallback
   rev 190701 (CRC 0xBB59), its 0-satellite run says nothing about the stage 1 this board copies. Then its 10-21
   steps 2, 6, 8 and 9.
1. **Stage 1 alone, on a HASL board.** Fit the power parts, U5, C27, C28, J1, D6, C20-C26, L1-L8, FL4, FL5, U3 and U4;
   leave U1, C33, L10 and L11 off.
   - VSYS current at 5 V: about 10.2 mA (9.8-11.8): two LNAs at about 5 mA plus D3.
   - DC at L3 pad 2 (46.605, 36.77) and L7 pad 2 (48.25, 39.875), the LNA inputs: equal, under 0.9 V. 0 V means an
     unpowered, dead or shorted input; about 1.8 V means the input is bridged to VCC or PON.
   - Oscillation sniff: PortaPack or HackRF with a small loop over U3/U4, 1 MHz-6 GHz, VSYS on and off. A line that
     comes and goes with VSYS means fit the U3 bypass (section 3 item 6).
   - Gain: pigtail center on C26 pad 2 (47.29, 42.985), shield on C33's ground pad (46.29, 42.825). Inject at
     -50 dBm or less at J1 (the cascade's input P1dB is about -40 dBm). Expect about 27 dB at 1575.42 MHz and about
     33 dB at 1176.45 MHz.
   - If you have a NanoVNA: S11 at J1 at low drive, to pick L1 (section 3 item 1).
2. **Full build (preferably an ENIG re-order), before SH1.** Meter a loose B4327 first (section 3 item 3). Then:
   - rails: +3V3, +1V8, VDD_RF 1.1 V (C49/C53/C50), VDD11 1.1 V, V_CORE about 1.3 V, VDD_TCXO 3.0 V;
   - pin 14's toe (section 2);
   - 26 MHz at pin 4 on a scope; decide on C48 from this alone;
   - DC at the U1 ends of L14 and L15 (pins 40/37); write the readings into the README.
3. **Receiver state before any RF verdict.** 3.3 V USB-UART on J3.1/J3.2/J3.4, 115200 first, then 4800-921600. Log the
   raw replies BEFORE any reset: version 0x02 (`A0 A1 00 02 02 01 03 0D 0A`), CRC 0x03 (`A0 A1 00 02 03 01 02 0D 0A`),
   message type 0x16, navigation constellation 0x64/0x1A, elevation/CNR mask 0x2F, and last boot status 0x64/0x01
   (`A0 A1 00 02 64 01 65 0D 0A`). Pass: boot status 00, CRC not 0xBB59, the constellation mask includes GPS + Galileo +
   BeiDou. Then Set Factory Defaults (`A0 A1 00 02 04 01 05 0D 0A`) and query again. A power cycle does not clear
   settings saved to flash, so this step can't be skipped. Use attribute 0 (SRAM only) for every setup command during
   bring-up, and don't send the PX1105R kinematic-base sequence until GPS tracks.
4. **Sky or rig, L1 first.** A passive antenna outdoors; or an active antenna through a DC-blocked bias tee plus a
   15-20 dB pad; or the rig started 20 dB cold and stepped up. L1 counts as working with at least 4 satellites at
   30 dB-Hz or more and a fix. Only then judge L5: L5-group channels used in the fix (0xE7 status bit 0x10) for at
   least 120 s at about 30 dB-Hz. On the rig, also require an underrun-free transmit. If only BeiDou appears again with
   good C/N0, suspect configuration, not RF.
5. **Then the frame** (section 6), re-check current and rails, and repeat the sky or rig test with the cover on.

---

## V2 changes

1. Order checklist in the board folder (finish, via covering, stackup, foil), checked on JLC's Product Detail before
   paying, with the as-placed Product Detail pasted into `gerbers/README.txt`.
2. J1: place the footprint so the land starts at the board edge; void In1 under the land (or narrow it) to make the
   launch about 50 Ω on the stackup actually ordered; pull the pour back from the pin path; then L1 returns to 5.1 nH.
3. A 1 nF at each LNA VCC pin within 0.5 mm with its own ground via; separate +1V8 branches; a via at each LNA ground
   pin and at FL7's center pad; a 0 Ω link in +1V8 for LNA current.
4. A capacitor pad from U1 pin 14 to ground (fitted or not, per SkyTraq); a 1-2.2 µF at pin 16; C40 at pin 23; six
   VDD_RF capacitors.
5. A 33-100 pF series capacitor between each band SAW output and its series inductor, if pins 37/40 carry DC.
6. Test access: BOOT_SEL, UART and supply pads; coax pads or a DNP U.FL at LNA_OUT.
7. L16 and D7 within 1 mm of pin 21; no copper under Y1/Y2; Seiko's Y2 land.
8. R7 10 k / R14 100 k; a polarized symbol for D2; L13 deleted (or DNP parts kept out of F.Paste).
9. The frame into the reflow once the match is frozen, or clip-type fences.
10. Optional: a dropper resistor ahead of U2 (for example 15 Ω, 1206) to move about 0.25 W off it.

## Questions for SkyTraq (U1 comes from them)

1. Is there a newer data sheet than v2 (2022, "Preliminary Draft"), a hardware design guide, or the EVB schematic and
   layout (in particular the trace length from the L-match inductors to pins 40/37)?
2. Pin 14: the pin table says NC, but both reference schematics put C21 2.2 µF on it. What is it, and is the capacitor
   required? Or is C21 meant for VDD11?
3. LNA_IN1/2: input impedance at 1575.42 / 1176.45 MHz, DC level in operation, whether a DC path to ground is allowed,
   recommended gain ahead of the pins, maximum operating input level and AGC range.
4. TCXO_I: input level, impedance and DC bias; direct or through a DC-cut capacitor?
5. LDO_RF_OUT minimum capacitance; the DC-DC switching frequency.
6. The production firmware version and CRC for PX100-5C-01A, its default output, the image, and the download
   procedure. Is the on-hand 03.07.01 / 20230608 image (GPS L1L5, BDS B1I/B2a, GAL E1/E5a) a PX100 image?
7. Can L5/E5a/B2a be acquired without L1?

## Records to correct

Done on 2026-10-28 in README.md, FABRICATION-NOTES.md and `gerbers/README.txt` (the bring-up and assembly steps above
went in too). The 2026-10-21 PX1105R review is left as written.

- `gerbers/README.txt` "Order form", FABRICATION-NOTES notes 1-3 and the README state paragraph describe ENIG,
  filled-and-capped vias and 3313; the order was HASL, plugged, no stackup.
- README: "C23/C25 now bypass each LNA" (U4 only); "L16, D7 and C39 tight to pins 21/22"; "2 ppb/g guaranteed"; "C20
  blocks DC" (D6 limits J1 to ±3.6 V); bring-up step 2's C48 rule; step 4's meaning with FL6 fitted; "no parts-only
  change fixes it" for FL6; step 5's split-input injection level; the rig level into J1.
- FABRICATION-NOTES: SAW markings and orientation for FL4/FL5/FL6; L13's stencil apertures; FL7's "mid-left" is in
  the footprint's frame; the TG-5006CG pin-1 sentence.
- The 2026-10-21 PX1105R review: "a power cycle always leaves the receiver in its factory state" is wrong for settings
  saved to flash.

## Not checked

- BGA855N6 S-parameters: Infineon's zip needs a myInfineon login. With it, the U3 stability question (section 3 item 6)
  can be settled on vendor data.
- SAW S-parameters (RF360 supplies them through sales only), so the SAW ports were modeled from data-sheet
  measurement circuits.
- The PX100's input impedance and LNA_IN DC level: not published.
- The physical boards: actual stackup, via state, HASL flatness.
