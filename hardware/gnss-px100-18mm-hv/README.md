# gnss-px100-18mm-hv — PX100 GNSS carrier (prototype)

The PX1105R carrier ([`gnss-px1105r-18mm-hv/`](../gnss-px1105r-18mm-hv/)) with the PX1105R module replaced by the SkyTraq
PX100-5C-01A single-chip L1/L5 raw-measurement receiver and a new RF front end. Same outline, holes, host connector (J3)
and LDO supply. Started 2026-10-22 from the carrier at 5c300f3a.

**Prototype (since 2026-10-23, the owner's call).** No patch antenna: the RF input is an SMA edge connector (J1, RF
Solutions CON-SMA-EDGE-S, as on the Beetle) for a passive antenna or the HackRF rig, on a 4-layer 1.6 mm board
(JLC04161H-3313). It is a bench and rig board, not the flight carrier.

**State:** V1 released on 2026-10-23 (tag `gnss-px100-18mm-hv-v1.0.0`) and ordered as bare boards from JLC; not yet
fabricated. V2 is in work: the title blocks and the silkscreen read V2. The only other change is the L5 input fix of
2026-10-26 (L13 not fitted, C37 1.3 pF; see *Front end*). It changes parts, not copper, so it applies to the V1 builds too.
ERC is clean apart from inherited off-grid warnings; DRC with schematic parity finds no errors and nothing
unconnected. Every via is filled and capped, the owner's choice of 2026-10-23, because U1's exposed pad
needs vias in it (*Layout*, item 3).

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

Pin 40 has no such path as long as FL6 (B4327) is open from its output to ground. A loose B4327 should read open from
pad 4 to its ground pads (2, 3, 5); if it reads a short, pin 40 is grounded too, and no parts-only change fixes it.

**No working PX100 to copy.** Neither TinkerNav PX100 has ever worked. Both boot and talk on the UART, but neither picks
up much signal (the owner, 2026-10-26; possibly their assembly). Nothing on this board's PX100 side is proven in
hardware, and SkyTraq's reference is the only design it treats as known-good.
- The V1 copies the reference network, with an MP05419 for the L1 SAW and a TA2471A on a 1.4 × 1.1 mm footprint.
  Outdoors on an active antenna on 2026-10-22 it held three BeiDou satellites (B1I 35–38 dB-Hz, B2a 31–33 dB-Hz). It
  never got GPS, Galileo or QZSS, and it never got a fix.
- The V2 (SAW, a reactive split, a per-band LNA, then a B8389 with 5.1 nH shunts on each PX100 input) tracked nothing.
  That topology is not used here.

**Budget (estimates, typical datasheet values, PX100 input noise figure assumed 3 dB):**

| | L1 | L5 |
|---|---|---|
| Feed match (J1, was the patch) | −0.3 dB | −0.3 dB |
| FL4 + FL5 (B8389) | −3.6 dB | −2.0 dB |
| U3 + U4 (BGA855N6, AN596 option B) | +31.4 dB | +35.2 dB |
| Split (Wilkinson, centred on L1) | −3.5 dB | about −4.5 dB |
| Band SAW | −1.0 dB (B4327) | −1.1 dB (SAFFB) |
| L-match | −0.3 dB | −0.3 dB |
| **Into the PX100** | **about +22 dB** | **about +27 dB** |
| System noise figure | about 2.9 dB | about 2.0 dB |

SkyTraq's reference with a typical 28 dB active antenna and 3 dB of cable puts about 19 dB into the chip, so this board
drives it a little harder. One LNA stage instead of two would give about 3.7 dB at L1 and only ~6 dB of drive.

**Part choices** (stock checked on JLC/LCSC 2026-10-22):

| Ref | Part | Why |
|---|---|---|
| FL6 | Qualcomm B39162B4327P810 (C2650956) | L1 band: GPS/Galileo L1 1.0 dB, BeiDou B1I 1.4 dB, GLONASS; 50 Ω, no matching; same 1.4 × 1.1 mm package and land as the B8389. SkyTraq's TA1573A is not sold by LCSC |
| FL7 | Murata SAFFB1G17AA0F0A (C910638) | L5 band 1164–1189 MHz, 1.1 dB; its measurement circuit has a 10 nH input shunt (L12) and a 7 nH output shunt. The output shunt (L13, 6.8 nH) is not fitted, because it would ground pin 37 through L15; C37 at 1.3 pF stands in for it (*Front end*). SkyTraq's TA2471A is out of stock |
| Y1 | Seiko Epson TG2520SMN 26.000000MHz MCGNNM (DigiKey, about $0.45–0.89) | 26 MHz, ±0.5 ppm over −40…85 °C, 2.8–3.3 V (runs from the PX100's 3.0 V TCXO LDO), 1.5 mA, 2.5 × 2.0 mm. Chosen for acceleration: Epson's 26 MHz data gives about 0.4 ppb/g on the package's long side (X) and under 0.05 on Y and Z (2 ppb/g guaranteed), the part our GNSS report modelled (section 3.2.3). Keep its long side along board x, across the thrust; the footprint's fab arrow marks it. C48 is the DC-cut its output needs. Replaced the Abracon ATX-11 (no acceleration spec) on 2026-10-22. LCSC has none for V1's builds; FABRICATION-NOTES.md names the stand-in, the TG2016SMN |
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
  at the feed (review V2 item 4), the extra 1 nF caps C29/C30 at U3/U4 VCC (review V2 item 2; C23/C25 now bypass
  each LNA, whose PON and VCC are both +1V8) and three of the six VDD_RF caps (C51, C52, C54). C49, C50 and C53 serve
  pins 34/35, 1/2 and 38/39.
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
   board setup says filling and capping yes for all vias (tented on top of that), and fab note 3 says so.
4. RF routing as on the carrier: 0.16 mm on F.Cu with a 0.20 mm gap over In1 ground (50 Ω; JLC04161H-3313 has the
   same 0.0994 mm 3313 prepreg under F.Cu as the 1.2 mm board, so the width is unchanged). Keep the
   split symmetric, the L-matches (C36/L14, C37/L15) at the PX100 pins, and the LNA_IN traces short. The RF net class
   covers the whole chain to pins 40/37 (LNA_OUT, ARM_L5, both band-filter nets, LNA_IN1/2); a board rule allows
   0.15 mm inside U1's courtyard for its 0.18 mm pad gaps, as one does at the LNAs.
5. Decoupling at its pin (see the receiver sheet note); L16, D7 and C39 tight to pins 21/22.
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
   - 201 vias, 0.4/0.3 mm, filled, capped and tented: 174 ground (5 in U1's pad, the rest beside ground pads, along
     both sides of the shield ring, beside the RF lines and on a grid in open pour) and 27 signal or supply. One
     VDD_RF via sits 0.025 mm from C50's pad opening, under the 0.09 mm guideline; filled and capped, it cannot wick
     solder. Every other via outside U1's pad keeps at least 0.09 mm from every mask opening, including the LNAs'
     shared mask windows.
   - RF line lengths: RF_IN 3.4 mm, LNA_OUT 2.2 mm, ARM_L5 3.0 mm, LNA_IN1 2.9 mm, LNA_IN2 1.6 mm.
   - The ground zone stops 0.9 mm short of the top edge, as it did before routing (it predates the wider outline),
     so that strip has no pour. I left the zone as it was.
   - Not checked: impedance in a field solver (the 0.16/0.2 mm line is the carrier's), and any 3D fit of the
     shield cover over the parts.

## Bring-up

1. Rails: +3V3, +1V8, then the PX100's own LDO outputs: VDD_RF and VDD11 at 1.1 V, VDD_TCXO at 3.0 V, V_CORE about 1.3 V.
2. Y1 at 26 MHz on a scope (clipped sine at C48), and Y1 placed with its long side along board x.
3. UART at 115200 on J3.1/J3.2; PPS on J3.5 once it has a fix.
4. With no RF connected, meter DC from pin 37 and pin 40 to ground, at the U1 end of L15 and L14. A steady voltage
   there means the PX100 biases its LNA inputs through the pins, which confirms that L13 must stay off. Write the
   readings down here.
5. Sky through a passive antenna on J1, or the HackRF rig straight into J1 (C20 blocks DC). To split the chain from the
   receiver, take the cover off and inject the rig at the split input instead (C26 lifted). The rig needs
   `RING_MIB=1024` on the Pi.
