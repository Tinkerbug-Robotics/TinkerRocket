# gnss-px100-18mm-hv — PX100 GNSS carrier (prototype)

The PX1105R carrier ([`gnss-px1105r-18mm-hv/`](../gnss-px1105r-18mm-hv/)) with the PX1105R module replaced by the SkyTraq
PX100-5C-01A single-chip L1/L5 raw-measurement receiver and a new RF front end. Same outline, holes, host connector (J3)
and LDO supply. Started 2026-10-22 from the carrier at 5c300f3a.

**Prototype (since 2026-10-23, the owner's call).** No patch antenna: the RF input is an SMA edge connector (J1, RF
Solutions CON-SMA-EDGE-S, as on the Beetle) for a passive antenna or the HackRF rig. V1 and V2 are 4-layer 1.6 mm
boards drawn for JLC04161H-3313; V3 is 6-layer, JLC06161H-3313. It is a bench and rig board, not the flight carrier.

**State (2026-11-25):** V3 is released: tag `gnss-px100-18mm-hv-v3.0.0`, with the JLC gerbers and a hand-placement
package (BOM, placement map, positions) on the GitHub release of that name. It carries the Tinker-Beetle V2's GNSS
section on 6 layers; *V3 front end* below lists what changed. V4 is in work: the title blocks and silkscreen read V4,
and nothing else has changed yet. V2 was ordered (JLC, HASL, plotted 2026-11-03) and two boards are built, but it was
never tagged; its as-ordered files and gerbers are kept locally in `gnss-px100-18mm-hv-backups/V2-as-ordered-2026-11-03/`
(git-ignored). The paragraphs after this one describe V1 and V2.

**State:** V1 released on 2026-10-23 (tag `gnss-px100-18mm-hv-v1.0.0`) and ordered as bare boards from JLC (order
Y349-3297386A, shipped). V2 is in work: the title blocks and the silkscreen read V2. The only other changes are two
parts changes of 2026-10-26, so they apply to the V1 builds too:
- the L5 input fix: L13 not fitted, C37 1.3 pF (see *Front end*);
- C48 as a 0 Ω link, so the TCXO drives TCXO_I directly (see Y1 under *Part choices*).

**The V1 boards were not ordered as these notes describe.** JLC's Product Detail reads Surface Finish "HASL(with
lead)", Via Covering "Plugged" and Specify Stackup "No requirement". So the V1 boards are HASL, not ENIG; U1's five
exposed-pad vias are not filled and capped; and the stackup is JLC's default, JLC04161H-7628, not 3313 (the owner
left it unselected). The stackup doesn't hurt: on this copper the model puts 7628 slightly ahead (*Layout* items 4
and 7). The assembly risk on the 0.4 mm-pitch PX100, the TSNP-6 LNAs and the small SAW lands
is not: see FABRICATION-NOTES (*V1 HASL boards*, *Order checklist*).

ERC is clean apart from inherited off-grid warnings; DRC with schematic parity finds no errors and nothing
unconnected. The board setup asks for filled and capped vias, the owner's choice of 2026-10-23, because U1's exposed
pad needs vias in it (*Layout*, item 3); the V1 order didn't include that option.

## V3 front end (released 2026-11-25, `gnss-px100-18mm-hv-v3.0.0`)

The Tinker-Beetle V2's GNSS sheet started as a copy of this V2 front end and has moved on since. V3 brings the carrier's
RF path back in line with the Beetle's, so the carrier can try the Beetle's chain before a Beetle is built.

```
SMA J1 -> [C81 pad] -> D6 -> [L20 bias tee, JP1] -> C20 -> [C82 pad] -> FL3 diplexer
  HB (L1) -> C61 470 pF -> L21 7.5 nH -> U3 AT2659S -> C64 -> FL6 B9621 -> C36 / C65 / L14 -> pin 40      (as V2)
  LB (L5) -> [trap: L23 + C79, C80] -> C66 100 pF -> L22 9.1 nH -> U4 ATR2652S -> C70 100 pF -> FL7 TA2471A
          -> C37 / C72 / L15 -> pin 37
```

- **L5 amplifier: U4 MAX2640 -> Zhongkewei ATR2652S** (LCSC C46955986), the L5-band sibling of the AT2659S. It uses the
  same SOT-23-6 land and pinout, except that pin 5 is its enable (EN, high = on), now tied to V_LNA instead of ground.
  Data sheet: 0.75 dB noise figure and 22.5 dB at 1176 MHz with its 1150-1300 MHz circuit (DC block plus a series
  9.1 nH at the input, output direct), 4.5 mA. So L22 goes from 5.6 to 9.1 nH (LQW15AN9N1G00D), and C70 goes from
  3.9 pF to 100 pF, a DC block only. C71, the unfitted output shunt, is gone. On board 1 the MAX2640 stage added
  nothing measurable (bypass test, 2026-11-20); the ATR2652S is the part the Beetle uses.
- **915 MHz trap at U4's input**, on the diplexer side of C66 as on the Beetle: L23 10 nH (LQW15AN10NG80D) in series
  with C79 3 pF to ground, plus C80 5.1 pF to ground. The diplexer's L5 port is a low-pass that lets 915 MHz through,
  and the Beetle's own LoRa radio sends +20 dBm at 902-928 MHz. Model: 21-25 dB of rejection over 902-928 MHz (at
  least 15 dB with 2 % and ±0.1 pF parts) for about 0.4 dB of L5 loss. The carrier has no radio. On it, the trap is
  there to measure that loss, and its rejection with a 915 MHz source nearby.
- **Antenna match pads C81 and C82:** unfitted, out of the BOM, value "tune". With C20 they form a pi network, as at the
  Beetle's patch feed. C81 sits on RF_IN at the feed point beside D6, and C82 on the COM line between C20 and FL3 pad 2, as on the Beetle. With a 50 Ω antenna on J1 both stay
  empty.
- **Layout: a copy of the Tinker-Beetle V2's GNSS section (2026-11-25).** The owner asked for the Beetle's layout as
  closely as possible. Every part inside the shield sits where its Beetle twin does: the Beetle's back-side section is
  mirrored onto this board's front about the shield's center, and all 55 parts' pads land on their Beetle pads. The
  Beetle's copper inside the shield comes with it (its front-layer routing, its inner routing and 106 ground vias),
  including the 915 MHz trap (L23, C79, C80), the tune pads C81/C82 and the LNA cap pairs. Outside the wall, the RTC
  crystal Y2, R11 (DCDC_IN) and the reset RC R12/C56 sit where the Beetle has Y5, R6 and R7/C62 (R12/C56 just left of
  the frame, since the Beetle's are on its other side). RF lines keep this board's 0.16 mm: the Beetle's 0.18 mm is for
  its 0.1164 mm 2116 prepreg, and this board's 0.0994 mm 3313 under F.Cu makes 0.16 mm about 50 ohm.
- **Where the carrier differs from the Beetle:** J1 feeds RF_IN through the top wall gap to the Beetle's antenna-pin
  point (48.25, 35.0). The TCXO Y1 and its 1 uF C47 sit outside the top wall right of J1, because the Beetle's Y3/C55
  spot falls on J1; the clock and its supply run on inner layers, as on the Beetle. The PPS LED trio (D1, R1, R9) moved
  up 0.35 mm, and the "PPS" text now stands upright right of C47. D7, the DC-DC Schottky, is on the back under L16,
  where the Beetle has D15: it is the one part soldered by hand after the bake. TP7, TP10 and TP13 moved to clear the
  copied copper. The links to the outside and to the test pads (UART, PPS, JP1, supplies) were routed by my grid router,
  one width per net.
- **6 layers, vias in pads allowed (owner, 2026-11-25).** The Beetle puts vias in cap and LNA ground pads, in the shield
  frame pads and beside the PX100 pins. JLC fills and caps every via on 6 layers, so the owner moved V3 to 6 layers and
  allowed them; the 0.09 mm mask rule of 2026-10-23 no longer applies to this board. Stackup JLC06161H-3313, 1.5994 mm
  (as gnss-sam10m8 V3): F.Cu parts and RF, In1 ground, In2 signal, In3 +3V3 plane (the Beetle's V_MCU_SWTCH plane), In4
  signal, B.Cu ground pour, test pads and D7. The PX100 fan-out rule is now the Beetle's: 0.15 mm only for RF-class
  copper inside U1's courtyard.
- **Schematic changes (owner, 2026-11-25):** R21/R22, the active-antenna bypass, are removed. On this layout their RF
  ends are 4 and about 12 mm apart, so even unfitted they would leave open stubs on the L1/L5 paths, and the Beetle has
  none. U1 pin 9 (ANT_BIAS) is no longer connected, as on the Beetle: its escape is taken by the +3V3 feed to pins 6/7
  and the C42-C44 row. With only two options left, JP1 is now a 2-pad solder jumper (SolderJumper-2_P1.3mm, its pads
  where the 3-pad's ANT_SUP and +3V3 pads were): open = passive antenna, no DC on J1; closed = +3V3 to the antenna
  through R20 and L20. TP17 (ABIAS) went with the ANT_BIAS net.
- **Unchanged on purpose:** the two-piece shield (BMI-S-202-F and -C) on the same pads as the Beetle's one-piece
  BMI-S-102, the antenna supply through JP1, the bias tee and the other test pads.
- **Every capacitor is drawn on the pin it serves**, and its Description field (on the board too) names that pin, e.g.
  "at U1 pin 17 (RSTN)". Redrawn 2026-11-23 with the netlist unchanged: U4's supply now lands on pin 6 with pin 5 tied
  off it; C56, the reset delay, moved from beside R12 to U1 pin 17; C1 moved from a loose +3V3 symbol onto FL2 pin 2,
  where it already sits on the board.
- **Checks:** DRC has no errors and no unconnected items. Its five warnings are V2's (J1's silkscreen at the board edge
  and three library-version notes). Schematic parity is clean; ERC is the same 67 off-grid warnings. Three router jogs
  are shorter than 0.2 mm (VDD_TCXO on B.Cu and In4, U1_P14 on In2), where a straight run would break clearance. The
  board before this layout is in `gnss-px100-18mm-hv-backups/V3-before-beetle-copy-2026-11-25/`.
- **Parts:** 10 nH, 9.1 nH, 3 pF, 5.1 pF and 100 pF are in the inventory workbook. The ATR2652S line has no count on
  hand (LCSC C46955986, minimum order 5).

## V2 front end (plotted for the order 2026-11-03)

The first V1 build booted and answered every query on the Mantis V10 (2026-11-02) but read 0 dB-Hz on every
channel for 35 minutes on two antennas, and its TSNP-6 LNAs can't be hand-soldered reliably. V2 replaces the front
end with leaded LNAs and makes antenna power optional. The schematic is done; placement and routing are not.

```
SMA J1 -> D6 -> [L20 47 nH bias tee, JP1] -> C20 -> FL3 diplexer (Bourns FDP2009N122157H4)
  HB (L1) -> C61 470 pF -> L21 7.5 nH -> U3 AT2659S -> C64 -> FL6 B9621 -> C36 / C65 / L14 -> pin 40
  LB (L5) -> C66 100 pF -> L22 5.6 nH -> U4 MAX2640 -> C70 3.9 pF -> FL7 TA2471A -> C37 / C72 / L15 -> pin 37
```

- **Antenna supply, JP1 (3-pad solder jumper):** open = passive antenna, no DC on J1 (the default); 1-2 = the PX100's
  ANT_BIAS output (pin 9, 45 mA limit; its voltage is unmeasured, so meter it on a V1 first); 2-3 = +3V3. R20 (10 Ω)
  sits between TP18 and TP19, so the antenna current reads as a voltage. D6 still limits J1 to ±3.6 V DC.
- **Active antenna:** fit R21 and R22 (0 Ω) and leave C61, C64, C66 and C70 off. That takes both LNAs out of the path,
  so an active antenna's 25-30 dB doesn't stack on top of theirs.
- **LNAs:** U3 AT2659S (SOT-23-6, 1550-1615 MHz, 18.5 dB, 0.8 dB NF, 4.4 mA) with its data sheet's 470 pF + 7.5 nH input;
  its output is matched on chip, C64 only blocks DC. U4 MAX2640 (SOT-23-6, 400-1500 MHz, 3.5 mA) has no vendor circuit
  at 1176 MHz, so I designed its match from the data sheet's tables 1 and 3: L22 5.6 nH puts the source near the noise
  optimum (Γopt about 0.36 at 61°, Fmin 0.92 dB), and C70 3.9 pF is a one-part output match (about 0.9 dB short of a
  conjugate match). C67 and C71 are unfitted shunt pads for tuning with the NanoVNA. Both LNAs run from +3V3 through
  R23 (10 Ω, V_LNA, about 8 mA), so U5 and the +1V8 rail are gone.
- **Expected chain (calculated, not measured):** L1 about 16 dB of gain and 1.7 dB noise figure (diplexer 0.87 dB +
  AT2659S); L5 about 12 dB and 1.8 dB. That replaces V1's 6 dB resistive split ahead of the bands with a 0.6-0.9 dB
  diplexer.
- **From the 2026-10-27 review:** C74 2.2 µF on pin 14 (both SkyTraq reference schematics fit it; the PX100 symbol's pin
  14 is passive now), C76-C78 (one 1 µF per VDD_RF pin, each drawn at its pin), C41 placed at pin 16 and C40 at pin 23
  (V1 had both by pin 23), C65/C72 between each SAW and its series
  inductor, R7 10 k with R14 100 k (UART back-feed), L12/L13 gone and C37 back to SkyTraq's 4 pF with the TA2471A.
- **Test pads on the back (TestPoint_Pad_D1.0mm), labeled on B.Silkscreen:**

  | Label | Pad | Net | Expect |
  |---|---|---|---|
  | VSYS / VSS | TP1 / TP2 | J3 supply and return, ahead of FL1 | 3.7-8.4 V between them |
  | GND | TP3, TP4 | board ground | |
  | 3V3 | TP5 | +3V3 (U2) | 3.3 V |
  | VLNA | TP6 | V_LNA, after R23 | about 3.2 V; LNA current = (3V3 - VLNA) / 10 ohm, about 8 mA |
  | VRF | TP7 | VDD_RF (pin 34 LDO) | 1.1 V |
  | VDD11 | TP8 | VDD11 (pin 23 LDO) | 1.1 V |
  | VCORE | TP9 | V_CORE (on-chip DC/DC) | about 1.3 V |
  | VTCXO | TP10 | VDD_TCXO (pin 3 LDO) | 3.0 V |
  | P14 | TP11 | U1 pin 14 (C74) | unknown: log it at first power-up |
  | RST | TP12 | RSTN | high = running |
  | BOOT | TP13 | BOOT_SEL | high = boot from flash; low at power-up = ROM loader |
  | RXD / TXD | TP14 / TP15 | the PX100's UART (RXD1 in, TXD1 out), receiver side of R7/R8 | idle high |
  | PPS | TP16 | 1PPS | pulses once a second after a fix |
  | ABIAS | TP17 | U1 pin 9 ANT_BIAS output | unmeasured |
  | ASUP / AFEED | TP18 / TP19 | either side of R20 | antenna current = (ASUP - AFEED) / 10 ohm; 0 with JP1 open |

- **Parts:** AT2659S (LCSC C883443), MAX2640EUT+T (LCSC C967082), 5.6 nH, 3.9 pF and 4 pF are new lines; every other
  passive is already in the inventory workbook. FDP2009N122157H4 is not at LCSC or DigiKey (Mouser lists it); the
  Johanson 1224DP15A1572001E covers the same bands on a different land. The TA2471A is out of stock at LCSC (spares on
  hand).
- **Firmware:** the V1's PX100 runs kernel 3.0.1 / 1.7.32 rev 22.04.27, the L1-only family, so L5 won't track on any
  board until SkyTraq supplies an L1/L5 image.
- **Layout still to do:** placement and routing; the test pads on B.Cu; the review's layout items (J1 launch, L16 and D7
  within 1 mm of pin 21, no copper under Y1/Y2, a ground via at each LNA ground pin); whether the shield frame still fits
  around the larger LNAs.

## Review of 2026-10-27

[`design-review-2026-10-27.md`](design-review-2026-10-27.md) found no design error that would stop the board working.
Every pin map and land matches its data sheet, stage 1 is Infineon's AN596 L1/L5 retune value for value, stage 2 is
SkyTraq's reference network, and every DC node is right. What it did find:

- **The V1 order** (*State*). I recommend re-ordering the same gerbers with ENIG and epoxy filled and capped vias
  before a PX100 goes on; keep JLC's default stackup. The HASL boards are fine for a stage-1-only build (*Bring-up*, step 1).
- **U1 pin 14.** The pin table says "NC, leave unconnected", but both SkyTraq reference schematics (5C and 5S) and
  their BOMs put C21 2.2 µF from pin 14 to ground. This board leaves it open, as both TinkerNavs did, and none of the
  three has ever tracked GPS. At first power-up, meter and scope pin 14's exposed toe (x 59.02-59.375, y 40.885).
  - A level that matches no rail (about 0.9-2.5 V), or ripple, means fit the capacitor.
  - 0 V or 3.3 V is ambiguous: load the pin briefly with 100 kΩ and see whether the level holds.
  - To fit it, tack a 0.1 mm enameled wire to the toe and run it to an 0402 2.2 µF (CL05A225MQ5NSNC) whose other end
    is soldered to scraped pour right of pins 12-13 (about y 41.8-42.6, below the GND via at (59.735, 41.478)). Never
    put an 0402 terminal on the toe itself: at 0.4 mm pitch it bridges pins 13 and 15.
  - Fit one board and leave one open, for an A/B.
- **Same-lands changes, not yet in the schematic:**
  - L1: keep 5.1 nH on these 7628 boards (updated 2026-10-28, once the stackup was known). 3.3 nH (Murata
    LQW15AN3N3B80D) would gain only about 0.2 dB at L1 in the model, inside its error; it is worth fitting only on a
    3313 board (about 0.4 dB at L1, 0.2 dB at L5), or if a NanoVNA's S11 at J1 says so. Never 2.4 nH on 7628: it
    costs 0.4 dB at L5.
  - R7 → 10 kΩ and R14 → 100 kΩ, together. Today a live USB-UART on J3.1 back-powers a switched-off board with about
    1.2-2.7 mA, so a power cycle during firmware work isn't clean. Never raise R7 alone: R14 sits on the receiver side,
    so R7 = 4.7 kΩ already puts a host low at 1.06 V, above the PX100's 0.8 V VIL. (The PX1105R carrier review's
    "R7 to 4.7 k" doesn't apply here.)
- **Bodges only if bring-up asks for them:** an 0201 1 nF at U3's VCC pin if the oscillation sniff finds a line
  (*Other changes from the PX1105R carrier*); a 1-2.2 µF 0402 at VDD11 across the scraped vias at (59.820, 40.650) and
  (59.735, 41.478) if bring-up shows spurs (pin 16's nearest capacitor is 10.65 mm and two vias away).
- **Keep SkyTraq's L-match values** (C36/L14, C37/L15). A modeled retune gained 1-3 dB, but it rests on a guessed
  PX100 input impedance and is worth at most about 0.1 dB of system noise figure.
- **Bring-up order**, the questions for SkyTraq and the V2 list: *Bring-up* below, and the end of the review.

## Sheets

| Sheet | Holds |
|---|---|
| root (`gnss-px100-18mm-hv.kicad_sch`) | J3 and its series resistors, the ADP7142 3.3 V LDO and its input filter, the LP5907 1.8 V LDO for the LNAs, the power LED, the mounting holes |
| `receiver.kicad_sch` ("RF chain and PX100") | The whole RF chain on one row: SMA J1, the feed cap C20 and ESD, FL4 → U3 → FL5 → U4 → C26 (the carrier's front end), then SkyTraq's split, band filters and input matches; below it the PX100 with its TCXO, 32 kHz crystal, core regulator parts and decoupling, the PPS LED (R1/D1), and the shield frame and cover |

## Front end

```
SMA J1 -> C20 -> B8389 -> BGA855N6 -> B8389 -> BGA855N6 -> C26                                 [stage 1]
      -> C33 2.7 pF -> split: L10/L11 6.8 nH, C34/C35 1.5 pF, R10 100 ohm                       [stage 2]
         L1 arm: FL6 B39162B4327P810 -> C36 3 pF / L14 8.2 nH -> PX100 pin 40 (LNA_IN1)
         L5 arm: FL7 SAFFB1G17AA0F0A (L12 10 nH input shunt) -> C37 1.3 pF / L15 12 nH -> pin 37 (LNA_IN2)
                 (L13, FL7's 6.8 nH output shunt, is not fitted)
```

**Why this shape.** The PX100 has two RF inputs with internal LNAs, and SkyTraq's reference (PX100 DS p13) feeds them from
an *active* antenna through a lumped Wilkinson splitter, a band SAW per arm and an L-match per input. The splitter and
filters cost about 5–6 dB, so with no gain in front of them the receiver's noise figure would be 7–8 dB. The board was
drawn for a passive patch, so the carrier's two-stage chain stays in front as the on-board "active antenna" and the
reference network follows it unchanged. Feed J1 from a passive antenna or the rig: there is no antenna bias, and an
active antenna's 25-30 dB on top of the two LNAs would drive the PX100 far harder than intended.

**No DC path at the PX100's RF inputs (2026-10-26).** Both SkyTraq references (5C and 5S data sheets) feed each LNA
input the same way: a shunt capacitor at the band SAW, then a series inductor to the pin. That leaves the pin with no DC
path to ground, since a SAW port is open at DC. As drawn on 2026-10-22, the L5 arm broke that: FL7's 6.8 nH output shunt
(L13) and the 12 nH series inductor (L15) grounded pin 37 for DC. The data sheet doesn't say whether the PX100 biases its
LNA inputs through the pins. If it does, that ground kills the L5 LNA. The TinkerNav PX100 V2 grounds both inputs the
same way, with 5.1 nH shunts right at the pins, and it never tracked anything. So L13 is not fitted, and C37 changes
from 4 pF to 1.3 pF:
- At 1176.45 MHz the reference 4 pF in parallel with 6.8 nH is +9.7 mS, which is a 1.31 pF capacitor, so the SAW and the
  L-match see the same admittance.
- Across 1164–1189 MHz the two differ by about 0.4 mS, about 2 % of 1/50 Ω and less than C37's own ±0.1 pF tolerance
  (±0.74 mS).
- C37 is Murata GJM1555C1H1R3BB01D (LCSC C161309), the same ±0.1 pF family as C33–C36.

Pin 40 has no such path as long as FL6 (B4327) is DC-open at its output. FL6's input side sits at 0 V through
L10/L11/L12, so a path from pad 4 to its ground pads (2, 3, 5) or to pad 1 would ground pin 40. That would cost both
bands, not just L1: the stock Phoenix firmware opens L5 channels only for satellites it already tracks on L1. A loose
B4327 must read open from pad 4 to pads 1, 2, 3 and 5. If it reads a short, fit a DC-open L1 SAW on the same land
instead: SkyTraq's TA1573A (rated 3 V DC; 1.1 × 0.9 mm, centered it covers about 75 % of the outer lands) or a Golledge
MP05419 (the same 1.4 × 1.1 mm pads; meter it first, it is 0 V-rated too).

The B4327 is rated for 0 V DC, yet its output is DC-coupled to pin 40 through L14. If bring-up step 2 finds a bias on
pin 40, FL6's output carries it permanently. SkyTraq's reference does the same with a TA1573A, which is rated for 3 V,
and short-term bench use is fine. To comply, cut the 1.06 mm FL6-OUT segment between C36 and L14 ((51.688, 45.843) to
(52.438, 45.093)) and bridge the cut with an 0402 33 pF; the match doesn't change (0.6 Ω at L1).

**No working PX100 to copy.** Neither TinkerNav PX100 has ever worked. Both boot and talk on the UART, but neither picks
up much signal (the owner, 2026-10-26; possibly their assembly). Nothing on this board's PX100 side is proven in
hardware, and SkyTraq's reference is the only design it treats as known-good.
- The V1 copies the reference network, with an MP05419 for the L1 SAW and a TA2471A on a 1.4 × 1.1 mm footprint.
  Outdoors on an active antenna on 2026-10-22 it held three BeiDou satellites (B1I 35–38 dB-Hz, B2a 31–33 dB-Hz). It
  never got GPS, Galileo or QZSS, and it never got a fix. The 2026-10-27 review found that its L1 SAW footprint swaps
  the part's output with a ground pad (it follows the data sheet's mirrored measurement-circuit drawing, not the top
  view). That alone doesn't explain "BeiDou but no GPS": B1I and GPS L1 share the same SAW passband and L-match, so that
  split points at receiver configuration or firmware state.
- The V2 (SAW, a reactive split, a per-band LNA, then a B8389 with 5.1 nH shunts on each PX100 input) tracked nothing.
  Both of its LNA inputs are DC-grounded: they take their signal from the B8389's output node, which carries the
  mandatory 5.1 nH shunt to ground, with no DC block, so the input transistors have no bias. That topology is not used
  here; this board's C22/C24 block the LNA inputs.

**Budget (estimates, typical datasheet values, PX100 input noise figure assumed 3 dB):**

| | L1 | L5 |
|---|---|---|
| Feed match (J1, was the patch) | −0.3 dB | −0.3 dB |
| FL4 + FL5 (B8389) | −3.6 dB | −2.0 dB |
| U3 + U4 (BGA855N6, AN596 option B) | +31.4 dB | +35.2 dB |
| Split (Wilkinson, centered on L1) | −3.5 dB | about −4.5 dB |
| Band SAW | −1.0 dB (B4327) | −1.1 dB (SAFFB) |
| L-match | −0.3 dB | −0.3 dB |
| **Into the PX100** | **about +22 dB** | **about +27 dB** |
| System noise figure | about 2.9 dB | about 2.0 dB |

SkyTraq's reference with a typical 28 dB active antenna and 3 dB of cable puts about 19 dB into the chip, so this board
drives it a little harder. One LNA stage instead of two would give about 3.7 dB at L1 and only ~6 dB of drive.

The 2026-10-27 review's network model of the board as routed agrees on the gain. Its main correction is the feed: J1's
1.5 × 3.5 mm land over In1 is a 2.1 pF shunt on 3313 (about 1.1 pF on 7628), not 0.3 dB, and it costs about 1 dB of
noise figure at L1 on 3313. The modeled system noise figure is about 3.1 dB (L1) and 2.2 dB (L5) on 7628, and 4.1 /
2.5 dB on 3313. L1 = 3.3 nH brings that to about 2.9 / 2.2 dB on 7628 and 3.7 / 2.3 dB on 3313.

**Part choices** (stock checked on JLC/LCSC 2026-10-22):

| Ref | Part | Why |
|---|---|---|
| FL6 | Qualcomm B39162B4327P810 (C2650956) | L1 band: GPS/Galileo L1 1.0 dB, BeiDou B1I 1.4 dB, GLONASS; 50 Ω, no matching; same 1.4 × 1.1 mm package and land as the B8389. SkyTraq's TA1573A is not sold by LCSC |
| FL7 | Murata SAFFB1G17AA0F0A (C910638) | L5 band 1164–1189 MHz, 1.1 dB; its measurement circuit has a 10 nH input shunt (L12) and a 7 nH output shunt. The output shunt (L13, 6.8 nH) is not fitted, because it would ground pin 37 through L15; C37 at 1.3 pF stands in for it (*Front end*). SkyTraq's TA2471A is out of stock |
| Y1 | Seiko Epson TG2520SMN 26.000000MHz MCGNNM (DigiKey, about $0.45–0.89) | 26 MHz, ±0.5 ppm over −40…85 °C, 2.8–3.3 V (runs from the PX100's 3.0 V TCXO LDO), 1.5 mA, 2.5 × 2.0 mm. Chosen for acceleration: Epson's 26 MHz data gives about 0.4 ppb/g on the package's long side (X) and under 0.05 on Y and Z (typical curves; the data sheet gives no guaranteed maximum), the part our GNSS report modeled (section 3.2.3). Keep its long side along board x, across the thrust; the footprint's fab arrow marks it. C48 is a 0 Ω link (2026-10-26; it was a 1 nF DC cut), so the TCXO drives TCXO_I directly. SkyTraq's reference wires it that way, and so do the TinkerNav PX100s, which boot (their ECS TCXO has the same clipped-sine output). Epson specifies its output through a ≥10 nF DC cut. SkyTraq publishes nothing on TCXO_I's bias, so a capacitor there could leave the input floating: keep 0 Ω unless the scope at pin 4 shows a bad clock, and then check Y1's supply and output before trying 10 nF in C48 (*Bring-up*, step 2). Replaced the Abracon ATX-11 (no acceleration spec) on 2026-10-22. LCSC has none for V1's builds; FABRICATION-NOTES.md names the stand-in, the TG2016SMN |
| Y2 | Seiko SC-32S 12.5 pF (C97606) | RTC crystal, no load capacitors (as the reference and the TinkerNav) |
| D7 | onsemi NSR0320MW2T1G (C48192) | the reference's DCDC_OUT Schottky clamp, in SOD-323 |
| L16 | Murata LQM18PN2R2MFRL (C337910) | the reference's 2.2 µH core inductor |
| SH1, SH2 | Laird (DuPont) BMI-S-202-F frame + BMI-S-202-C cover (DigiKey 903-1051-1-ND + 903-1014-ND, $1.02 + $0.68; in stock 2026-10-23) | two-piece shield: a 16.5 × 16.5 mm solder-on frame on a 17.3 mm pad ring (about 15.3 mm inside) with a snap-on cover that comes off for rework and rig tests. The frame goes on by hand after inspection, the cover after the first sky test. Back on the Laird on 2026-10-22 at the owner's request, after a spell on the one-piece Masach MS148-20S-NS (16.1 mm ring, 13.1 mm inside, not removable) |

**Larger parts?** Asked 2026-10-22. Nothing suitable is stocked: low-current GNSS LNAs that cover L1 and L5 all come in
≈ 1 mm packages (BGA855N6 0.7 × 1.1); the 2 × 2 mm wideband LNAs in stock (QPL9503, QPL9547, MAAL-011078) draw
50–56 mA each, which the already-hot LDO cannot take; the MAX2659 (1.0 × 1.5) is L1-only. GNSS SAWs are 1.4 × 1.1 mm at
most (the L1 band filter uses that size); no L5-only SAW larger than 1.1 × 0.9 is stocked. The TCXO is a 2.5 × 2.0 mm Epson, picked for its low
acceleration sensitivity on the thrust axis rather than size.

## Other changes from the PX1105R carrier

- J3 pin 5 is now **1PPS** (through R9 1 kΩ, as on the SAM carrier). The PX100 has no second UART input.
- Dropped on 2026-10-22 so everything fits inside the Laird frame (the owner's call): the DNP pi-match pads C19/C31
  at the feed (review V2 item 4), the extra 1 nF caps C29/C30 at U3/U4 VCC (review V2 item 2) and three of the six
  VDD_RF caps (C51, C52, C54). C49, C50 and C53 serve pins 34/35, 1/2 and 38/39.
- **Only U4's VCC pin has a local bypass (2026-10-27 review).** C25 sits 1.0 mm from U4's VCC (pin 2), but C23 is
  wired to U3's PON (pin 6). U3's VCC reaches a 1 nF only through 1.12 mm of 0.12 mm track, a via, 1.6-1.8 mm of In2
  and another via, about 2-3 nH, and the same via feeds U4's PON. Infineon calls the VCC capacitor optional and the
  models show no oscillation, but the BGA855N6's stability margin is thin. If the bring-up sniff finds a line that
  comes and goes with VSYS, stand an 0201 1 nF upright from U3's VCC trace corner (about (46.00, 38.36-38.48); scrape
  0.15 × 0.2 mm of mask) to the exposed top of C25 pad 2 (y 38.78-38.93). It clears U3 pins 1 and 3 by only about
  0.18 mm, and a bridge to pin 3 shorts +1V8 through L4 and L5, so ohm +1V8 to GND before and after.
- J4, the DNP U.FL rig test port at the split input, was deleted by the owner on 2026-10-22. There is no antenna-bias
  option either: SkyTraq's 47 nH bias feed and its 100 pF DC block (L9, C32) came out on
  2026-10-22 because C26 already blocks the LNA's DC, and the carrier's old module decoupling C5 went with them.
- R2 (the PX1105R's RSTN pull-up) is gone; the PX100 has its own reset RC (R12/C56).

## Layout (placed by the owner, routed 2026-10-23)

1. **Update PCB from Schematic** after every schematic change, with "Delete footprints with no symbols" ticked and
   re-link left **un**ticked. `tools/check_board_parity.py` passes (85 symbols) as of 2026-10-23.
2. **Placement.** The owner's, on the 28.3 × 30.8 mm outline: J1 on the top edge feeding the frame through its top
   gap, stage 1 down the left inside the frame, the split and both band arms below it, U1 at (56.52, 40.28) rotated
   90° with its RF pins (31-40) facing the chain, the DC-DC parts above U1 and Y1 with its caps below. Outside the
   frame: the LDOs on the left, R11 and Y2 on the right edge, R1/R9/D1 top right, the UART and reset parts above J3.
   I moved five small parts while routing (2026-10-23), each by under a millimeter; DRC adds no courtyard overlaps:
   - **C53** (52.93, 41.15, 90°) → (52.25, 41.485, 180°). It sat in LNA_IN2's only path to pin 37. It now lies
     between the LNA_IN1 and LNA_IN2 lines, with a VDD_RF via between it and pins 38/39.
   - **L15** (51.98, 42.57, 90°) → (52.115, 40.55, 0°), so LNA_IN2 reaches pin 37 from above C53 (1.6 mm long).
   - **C49** (52.51, 38.77) → (52.45, 39.60), level with pins 34/35 and out of LDO_ADC_OUT's way to C55.
   - **L4** (47.81, 37.74) → (47.81, 38.265) and **L8** (46.80, 41.04, 0°) → (46.30, 41.04, 180°). U3 pin 4 and U4
     pin 4, the LNAs' ground pins between input and output, were boxed in by RF pads; each now has a short track to
     its own via.
3. **Four layers, 1.6 mm (2026-10-23, the owner's call; the board was briefly 6 layers).** JLC04161H-3313: 3313
   prepreg (0.0994 mm) under both outer layers around a 1.265 mm core, about 1.63 mm in all, which J1 is rated for
   (1.2-1.6 mm). Layers: F.Cu parts and RF; In1 GND (the RF reference); In2 signals and power; B.Cu GND. No
   via-in-pad on 4 layers (the owner's rule; vias tented, copper at least 0.09 mm off mask openings), with one
   exception the board cannot avoid. **U1's exposed pad needs filled-and-capped vias.** Pin 41 is the PX100's only
   ground, and the corner necks this note used to promise do not fit. The corner pads are 0.208 mm apart, but a neck
   needs at least 0.27 mm (a 0.09 mm track with 0.09 mm either side). So the pad carries five 0.4/0.3 mm vias in the
   paste-free cross of its windowpane, the only vias in any pad. The owner chose epoxy-filled and copper-capped vias
   on 4 layers (2026-10-23) over unfilled vias, which would wick solder from under U1, and over going back to 6
   layers, where JLC fills and caps for free. JLC fills every via on the board when the option is ordered, so the
   board setup says filling and capping yes for all vias (tented on top of that), and fab note 3 says so. **The V1
   order didn't include the option** (Via Covering: Plugged), and it named no stackup, so the V1 boards are JLC's
   default JLC04161H-7628 (0.21 mm prepreg under F.Cu). See *State* and FABRICATION-NOTES (*V1 HASL boards*).
4. RF routing as on the carrier: 0.16 mm on F.Cu with a 0.20 mm gap over In1 ground (50 Ω; JLC04161H-3313 has the
   same 0.0994 mm 3313 prepreg under F.Cu as the 1.2 mm board, so the width is unchanged). A 2-D field solve
   (2026-10-27) gives 50.5 ± 1 Ω on 3313 and about 68 Ω on 7628; every RF run is under 3.5 mm, about 3 % of a
   wavelength, so the 7628 V1 boards lose little. Keep the
   split symmetric, the L-matches (C36/L14, C37/L15) at the PX100 pins, and the LNA_IN traces short. As routed, C36
   sits 1.7 mm before L14 and L14 2.9 mm from pin 40; L15 is 1.6 mm from pin 37 (V2: closer). The RF net class
   covers the whole chain to pins 40/37 (LNA_OUT, ARM_L5, both band-filter nets, LNA_IN1/2); a board rule allows
   0.15 mm inside U1's courtyard for its 0.18 mm pad gaps, as one does at the LNAs.
5. Decoupling at its pin (see the receiver sheet note). As routed, the DC-DC parts are not tight: L16 is 3.8 mm and D7
   5.2 mm of track from pin 21 (about 1.4 nH on the switch node). Nothing points at harm; if the rig with no signal
   shows CW or jamming reports near L1/L5, probe there with the cover off. V2: L16 and D7 within 1 mm of pin 21, C39
   at pin 22. VDD11 (pin 16) has no capacitor of its own; its nearest, C40/C41, are 10.65 mm and two vias away.
6. **Routing (2026-10-23).** A script routed the board on a 0.025 mm grid after a hand-drawn fan-out for U1 and
   the two LNAs. The owner then reworked it before release: rounded 2 mm corners, C135 moved 0.05 mm (clearing its
   courtyard overlap with FL2), new runs for +3V3, VDD_RF, VSYS/VSS, the UART, the RTC lines and J3.5, and a copper
   pour on U2's input (U2-EN) on F.Cu. The facts below are the released board's. KiCad DRC (zones refilled) finds no
   clearance errors and nothing unconnected; the warnings left (the U2/R1/D1 library mismatches, J1's silk over the
   edge, the U3/U4 silk) were on the board before routing.
   - F.Cu: the RF chain (0.16 mm with the 0.2 mm gap), U1's fan-out and the short local links. In1: untouched ground.
     In2: +3V3 (0.3 mm), VDD_RF and VDD11 (0.3 mm), +1V8 (0.25 mm), U2's input hop, PPS, the J3.5 PPS output and
     the UART. B.Cu: ground pour, with RSTN and a +3V3 hop on it.
   - Leaving the frame: RF_IN on F.Cu through the top gap; DCDC_IN, TX and RX on F.Cu through the upper right gap,
     then TX and RX drop to In2; RTC_XIN/XOUT stay on F.Cu out to Y2. PPS, RSTN, VDD11, VDD_RF and +3V3 change
     layer at vias beside U1.
   - VDD11 runs from pin 23 (LDO_C_OUT) to C40/C41 and on to pin 16 through In2. The three VDD_RF groups (pins
     34/35 with C49, 38/39 with C53, 1/2 with C50) join on In2.
   - 0.12 mm tracks where U1's fan-out squeezes between pin rows and caps: BOOT_SEL, VDD11 from pin 23,
     LDO_ADC_OUT, PPS, LDO_B_OUT, TCXO_I, RTC_XOUT, VDD_TCXO's hop between C46 and C50, and +3V3 into pins 6/7.
     All carry well under 0.15 A.
   - 201 vias, 0.4/0.3 mm, drawn filled, capped and tented (V1 was ordered "Plugged"; see item 3): 174 ground (5 in
     U1's pad, the rest beside ground pads, along both sides of the shield ring, beside the RF lines and on a grid in
     open pour) and 27 signal or supply. One VDD_RF via sits 0.025 mm from C50's pad opening, under the 0.09 mm
     guideline. It was accepted because a filled and capped via can't wick solder; on the V1 boards it is open on top,
     so check C50 pad 1's joint. Every other via outside U1's pad keeps at least 0.09 mm from every mask opening,
     including the LNAs' shared mask windows.
   - RF line lengths: RF_IN 3.4 mm, LNA_OUT 2.2 mm, ARM_L5 3.0 mm, LNA_IN1 2.9 mm, LNA_IN2 1.6 mm.
   - The ground zone stops 0.9 mm short of the top edge, as it did before routing (it predates the wider outline),
     so that strip has no pour. I left the zone as it was.
   - Not checked: any 3D fit of the shield cover over the parts. (The line impedance was checked on 2026-10-27, item 4.)
7. **J1 launch (2026-10-27 review).** Two things about J1:
   - **Its land is capacitive.** The 1.5 × 3.5 mm land sits 0.1 mm over solid In1 (and In2/B.Cu) ground: a 10.5 Ω
     section, about 2.1 pF on 3313 (about 1.1 pF on 7628). It cancels most of L1, the 5.1 nH shunt that matches FL4's
     input, and costs about 1 dB of noise figure at L1 on 3313, much less on the 7628 V1 boards. On 3313, L1 = 3.3 nH
     recovers most of it; on 7628 it isn't needed (*Review of 2026-10-27*).
     V2: void In1 under the land (or narrow it) for about 50 Ω on the stackup ordered, then L1 goes back to 5.1 nH.
   - **It sits 1.57 mm further out than the footprint assumes.** The footprint expects the board edge at the land's
     start (y 27.70); the edge is at y 26.13. With the flange on the edge, the 3.81 mm pin overlaps the land by 2.24 mm
     (enough to solder) and first crosses 0.49 mm of F.Cu ground pour (y 27.015-27.502) covered only by solder mask.
     FABRICATION-NOTES (hand work) says how to fit it and check it. V2: put the land start on the edge and pull the
     pour back from the pin path.

## Bring-up

Rewritten on 2026-10-27 from the review. Each step isolates one part of the chain; do them in order, and the first
one that fails names the fault. On this same SkyTraq silicon, a fallback firmware image and a saved base-mode setting
have each looked exactly like a dead front end ("talks on the UART, 0 dB-Hz on every channel"), so step 3 comes before
any RF verdict.

0. **The PX1105R carrier first.** Its stage 1 is this board's. Send it 0x02 and 0x03 and log the version and CRC. The
   fallback image (rev 190701, CRC 0xBB59) tracks nothing while it answers commands; if that is what it runs, its
   0-satellite result says nothing about stage 1. Then its 2026-10-21 review's steps 2, 6, 8 and 9.
1. **Stage 1 alone, on a V1 (HASL) board.** Fit the power parts, U5, C27, C28, J1, D6, C20-C26, L1-L8, FL4, FL5, U3
   and U4; leave U1, C33, L10 and L11 off. Check D2's polarity before power (FABRICATION-NOTES). Supply 5 V on J3.3/J3.4.
   - VSYS current: about 10.2 mA (9.8-11.8 mA), two LNAs at about 5 mA plus D3.
   - DC at L3 pad 2 (46.605, 36.77) and L7 pad 2 (48.25, 39.875), the LNA inputs: the same on both, under 0.9 V. 0 V
     means an unpowered, dead or shorted input; about 1.8 V means the input is bridged to VCC or PON.
   - Oscillation sniff: the PortaPack or a HackRF with a small loop over U3/U4, 1 MHz-6 GHz, VSYS on and off. A line
     that comes and goes with VSYS means fit the U3 VCC bypass (*Other changes from the PX1105R carrier*).
   - Gain: a pigtail with its center on C26 pad 2 (47.29, 42.985) and its shield on C33's ground pad (46.29, 42.825).
     Inject -50 dBm or less at J1 (the cascade's input P1dB is about -40 dBm). Expect about 27 dB at 1575.42 MHz and
     about 33 dB at 1176.45 MHz.
   - With a NanoVNA: S11 at J1 at low drive, to choose L1 (*Layout* item 7).
2. **Full build, before SH1** (preferably on a re-ordered ENIG board). Meter a loose B4327 first (*Front end*). Then:
   - rails: +3V3 (C1), +1V8 (C28), VDD_RF 1.1 V (C49/C53/C50), VDD11 1.1 V (C40/C41), LDO_B_OUT (C42), LDO_ADC_OUT
     (C55), V_CORE about 1.3 V (C39), VDD_TCXO 3.0 V (C46), RSTN and BOOT_SEL at 3.3 V;
   - U1 pin 14 (*Review of 2026-10-27*);
   - Y1 at 26 MHz on a scope at pin 4 (C48 is a 0 Ω link), and Y1 placed with its long side along board x. Decide on
     C48 from this reading alone: UART silence has other causes (a saved "no output" setting, the baud, BOOT_SEL or RSTN
     low). If the clock is bad, check 3.0 V at Y1 pad 4 and Y1's output at pad 3 with C48 lifted, into 10 kΩ, and only
     then try 10 nF in C48 (Epson's value);
   - with no RF connected, DC from pin 37 and pin 40 to ground, at the U1 end of L15 and L14. A steady voltage means the
     PX100 biases its LNA inputs through the pins: L13 must stay off, and FL6's output carries that DC (*Front end*).
     Write the readings down here.
3. **Receiver state, before any RF verdict.** A 3.3 V USB-UART: TX to J3.1, RX from J3.2, GND to J3.4; 115200 first,
   then 4800-921600. Log the raw replies before any reset:
   - version, 0x02: `A0 A1 00 02 02 01 03 0D 0A`
   - CRC, 0x03: `A0 A1 00 02 03 01 02 0D 0A`
   - message type, 0x16: `A0 A1 00 01 16 16 0D 0A`
   - navigation constellation, 0x64/0x1A: `A0 A1 00 02 64 1A 7E 0D 0A`
   - elevation and CNR mask, 0x2F: `A0 A1 00 01 2F 2F 0D 0A`
   - last, boot status, 0x64/0x01: `A0 A1 00 02 64 01 65 0D 0A`

   Pass: boot status 00, CRC not 0xBB59, and a constellation mask that includes GPS, Galileo and BeiDou. Then send Set
   Factory Defaults (`A0 A1 00 02 04 01 05 0D 0A`; the receiver reboots) and query again. A power cycle doesn't clear
   settings saved to flash, so this step can't be skipped. During bring-up send every setup command with attribute 0
   (SRAM only), and don't send the PX1105R kinematic-base sequence until GPS tracks. Pulling J3 is a full power cycle
   (U2's EN is tied to VIN), but a USB-UART left on J3.1 back-powers the board unless R7/R14 are changed.
4. **Sky or rig, L1 first.** A passive antenna outdoors; an active antenna through a DC-blocked bias tee plus a
   15-20 dB pad; or the rig.
   - Limits: J1 takes at most 3.6 V DC (D6, which sits ahead of C20, sets that: a HackRF bias of 3.0-3.3 V is fine, 5 V
     gear is not). The PX100's inputs take at most 0 dBm, which an in-band CW of about -23 dBm (L1) or -27 dBm (L5) at
     J1 reaches. Never use the HackRF TX amplifier with this board, and run CW tests through at least 40 dB of pad (J1
     at -50 dBm or below).
   - Rig level: undocumented, and set on receivers with no LNA, while this board adds 22-27 dB. Start about 20 dB
     colder than the standard L1 40 / L5 36 (or put a 20 dB pad after the DC block), step up 3 dB at a time and log
     C/N0 per signal; use the plateau. Overdrive looks like a dead board.
   - Rig recipe: dual_tx from rig/linux-host at 0045178 or later, with its sysctl set on the Pi; `RING_MIB=1024`;
     `--rate 17500000 --clock-from l5`; the `s17p5_sky2_w` L1 file (the narrow L1 file has no B1I) with `s17p5_sky_l5`.
     Discard any run with an underrun on either board in its first 15 s, and run at least 120 s.
   - L1 works with at least 4 satellites at 30 dB-Hz or more and a fix. Only then judge L5: L5-group channels used in
     the fix (0xE7 channel-status bit 0x10) for at least 120 s at about 30 dB-Hz. Short L5 entries up to 27 dB-Hz appear
     even with no L5 transmitted. If only BeiDou shows again with good C/N0, suspect configuration, not RF.
   - To split the chain from the receiver, inject at the split input with C26 lifted, but at active-antenna level: at
     J1's level the PX100 runs 28-33 dB under its operating point, and "no tracking" there proves nothing.
5. **Then the frame** (FABRICATION-NOTES, hand work). Re-check current and rails, and repeat the sky or rig test with
   the cover on. PPS appears on J3.5 once the receiver has a fix.
6. **Heat on the bench.** At 8.4 V, U2 dissipates about 0.65 W and the board about 1.1 W: inside every rating in open
   air at room temperature, not flat on foam or in a closed box. Feed VSYS at about 5 V on the bench (minimum about
   3.7 V), keep the board off the bench surface, and log U2's ground pin and the frame temperature at first power-up.
