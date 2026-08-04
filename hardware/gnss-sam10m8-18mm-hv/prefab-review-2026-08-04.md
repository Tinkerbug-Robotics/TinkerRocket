# SAM-M10Q GNSS carrier — pre-manufacturing review

**Board:** `hardware/gnss-sam10m8-18mm-hv` · 22.00 × 27.50 mm · 4-layer · KiCad 10 (file version 20260206)
**Date:** 2026-08-04
**Reviewed against:** TI SLVSFP4B (TPS62912/TPS62913, Rev B, Mar 2021), u-blox UBX-22020019 R02 (SAM-M10Q integration manual), part datasheets for FL1/FB1/L8/CR1, and the host-side connector on `rocket-computer`.

## Verdict

**No fabrication blockers.** DRC is clean: 9 warnings, all cosmetic (4 library-sync mismatches, 5 silkscreen-clipped-by-mask), 0 unconnected items, 0 schematic-parity errors. The board will build.

The findings below are correctness, RF-performance and BOM-risk items. The power stage is a faithful implementation of TI's reference design — component *values* are almost all exactly right. The problems are concentrated in three places: **the capacitors have no part numbers**, **the feedback network is placed and routed against TI's explicit layout rules**, and **the ground plane is far below what u-blox requires for the integrated patch antenna**.

| # | Finding | Severity |
|---|---|---|
| A1 | Capacitors have no MPNs; C8/C9 may be under-rated for 8.4 V, COUT may fall below TI's 40 µF minimum | **Fix before ordering** |
| A2 | FB trace runs 0.25–0.30 mm from the SW node for 2.9 mm; FB divider 6–7 mm from the FB pin | **Fix before ordering** |
| A3 | PSNS (U3.7) has no ground via | **Fix before ordering** |
| A4 | V_BCKP tied to VCC — contradicts u-blox, forfeits hot start | **Fix before ordering** |
| B1 | TVS clamp current returns through the common-mode choke; TVS 7 mm from the connector | Should fix |
| B2 | Only 5 GND vias under the module; u-blox asks for the plane under it to be filled | Should fix |
| B3 | In2.Cu is a full-board +3V3 plane where the antenna wants ground | Should fix |
| B4 | Mounting-hole pads are netless — 3.8 mm floating copper on all 4 layers, at the plane corners | Should fix |
| B5 | No decoupling at U1 pin 2 (V_IO) / pin 3 (V_BCKP); nearest cap 10–11 mm | Should fix |
| B6 | Board ID silkscreen is entirely hidden under U1; no refdes anywhere on the board | Should fix |
| B7 | Input HF cap 1.6 mm away instead of bridging VIN/PGND | Should fix |
| B8 | J3 pin 5 dead; PG pull-up dangling; J3 hold-downs netless | Should fix |
| C1–C8 | Accepted-limitation and tidy-up items (see below) | Consider |

---

## A — Fix before ordering

### A1. Capacitors have no part numbers, and two groups have hard constraints

No capacitor or resistor in the schematic carries an MPN, manufacturer or distributor field (only CR1 does). There is no `bom.csv` for this board. The project convention elsewhere (`rocket-computer/bom.csv`, `base-station/bom.csv`) is value-only with an empty supplier column — which means the assembler picks the part. For most of this board that is fine. For two groups it is not:

**C8, C9 — 2 × 22 µF 0805 on VIN.** VIN is raw pack voltage: 7.4 V nominal, **8.4 V on a fully charged 2S**, and CR1 does not begin clamping until 11.1 V. The default 22 µF 0805 at every distributor and in JLC's basic library is 6.3 V or 10 V rated. A 6.3 V part is operated **over its rating**; a 10 V part has 1.6 V of margin against a full pack and none against hot-plug ringing.

> Specify **2 × 10 µF 25 V X7R 0805** — this is exactly what TI's reference schematic uses (`CIN = 2 × 10 µF`), it clears the 5 µF effective minimum with margin after DC-bias derating (~5–6 µF each at 8.4 V), and it halves the hot-plug inrush versus 2 × 22 µF.

**C11, C13, C15 — 3 × 22 µF 0805, first-stage COUT.** TI specifies a *window*, not a minimum: with a second-stage filter in use, **COUT must be 40–80 µF effective** (§8.2.2.2.3), because the loop compensation is internal and non-adjustable. Effective means after DC-bias derating at 3.33 V:

| Part class | Typ. C at 3.3 V bias | 3 × total | vs 40–80 µF window |
|---|---|---|---|
| 22 µF 0805 6.3 V X5R (common default) | ~10–12 µF | 30–36 µF | **below minimum** |
| 22 µF 0805 10 V X7S — TI's own recommendation, TDK `C2012X7S1A226M125AC` | ~15–16 µF | ~46 µF | in window |

> Specify TI's `C2012X7S1A226M125AC` (or a part whose published DC-bias curve puts three of them in the window) for C11/C13/C15, and verify against the curve rather than the nameplate value.

C1/C12/C14 (Cf, second stage) only need ≥ 20 µF effective and total < 200 µF — they are safe with any of these parts. C7 (2.2 nF on VIN) needs a ≥ 25 V rating; 0402 parts are 50 V by default.

### A2. Feedback network placement and routing violate two explicit TI rules

TI layout guidelines, §10.1:
> *"Sensitive traces, such as the connections to the NR/SS, VO, and FB pins need to be connected with short traces and be **routed away from any noise source, such as the SW pin**."*
> *"**Place the FB resistors, R1 and R2, close to the FB pin** and route the VOUT connection from R1 to the load as a remote sense trace."*

Measured on the board:

- R5 (15.4 k) is **6.90 mm** from the FB pin; R6 (4.87 k) is **5.95 mm**. The FB net is 8.47 mm of 0.127 mm trace. The FB node impedance is R5‖R6 = **3.70 kΩ** — a high-impedance node run a long way.
- The dominant segment runs straight along y = 44.890 from x = 52.689 to 55.600 — **2.9 mm parallel to the switch node**. Clearances:
  - to the SW copper zone (top edge y = 44.58): **0.247 mm**
  - to L8 pad 1, the inductor's SW-side pad (top edge y = 44.53): **0.297 mm**
- L8 is a `VLS3012CX` — a *semi-shielded* wound-ferrite part. The FB trace runs 0.36 mm outside its body edge, in the fringing field.

The In1.Cu ground plane 0.1 mm below both conductors does reduce lateral coupling substantially, so this may well work. But the entire reason to pay for a TPS62913 is its sub-10 µV<sub>RMS</sub> output noise, and this geometry injects 2.2 MHz switch-node coupling straight into the reference node that sets the output — on a board whose only load is a GNSS receiver, where supply noise costs C/N₀ directly. There is no upside to accepting the risk.

> Move R5/R6 adjacent to U3 pin 9, keep the divider node under ~1 mm, and run the post-bead sense from +3V3 to R5 as a thin remote-sense trace that does not pass the SW zone or the inductor. Sensing after the bead is correct and should stay (§10.1: *"If a second L-C filter is used, this connection should be made after Lf"*).

NR/SS is a lesser case of the same thing: closest approach to the SW zone is 1.33 mm. Worth improving in the same pass since the parts move together.

### A3. PSNS (U3.7) has no ground via

TI, §10.1: *"**Connect the PSNS pin directly to the system GND plane with a via.**"* PSNS is the device's internal ground sense; its absolute maximum is **−0.3 to +0.3 V** — the tightest rating on the part.

On the board, U3.7 reaches ground only through the F.Cu pour. The nearest via of any kind is **1.37 mm away and belongs to Net-(U3-PG)**; the nearest actual GND via is 4.09 mm. PGND (U3.4) is no better — nearest GND via 5.17 mm.

> Drop a GND via directly on U3.7, and a second at U3.4. Free change, explicit datasheet requirement.

### A4. V_BCKP is tied to VCC — u-blox says leave it open

u-blox integration manual §4.1.3:
> *"**If the hardware backup mode is not used, leave the V_BCKP pin open.**"*
> *"If the power supply at V_IO is interrupted, but the V_BCKP pin is supplied, the receiver enters the hardware backup mode. In this mode, the RTC time and the GNSS orbit data in the BBR are maintained. Valid time and GNSS orbit data at startup improves positioning performance by enabling hot starts, warm starts, and AssistNow Autonomous."*

U1 pin 3 (V_BCKP) is on +3V3, the same rail as VCC (pin 17) and V_IO (pin 2). All three die together. The current arrangement is neither of the two configurations u-blox documents: it gets no backup benefit *and* it is not the recommended open-pin case.

This has a concrete cost here. The host (`rocket-computer` J1 pin 4 → Q1, a `PMPB14XNX` low-side N-FET) power-gates this board by opening its ground return. Every gate cycle drops the BBR, so every restart is a **cold start (~26 s TTFF)** rather than a hot start (~1–2 s). On a pad, that is the difference between "GPS is back" and "we wait".

> Two clean resolutions:
> - **Minimum:** leave pin 3 open, per the manual. Zero cost, removes the deviation, changes nothing functionally.
> - **Better:** give V_BCKP a real backup. It is referenced to *board* ground, so a local supercap works even though the host switches the ground: SAM-M10Q backup draw is in the tens of µA, so a 0.1 F supercap holds the BBR for hours, and even a 100 µF ceramic covers a brief power blip. Feed it through a Schottky from +3V3. §4.1.3 warns to *"avoid high resistance on the V_BCKP line"*, so keep the series element low-drop.

---

## B — Should fix

### B1. The TVS discharges through the common-mode choke

CR1 (SMF10A) is wired **VSYS → GND**, but board GND only reaches the harness return (VSS, J3 pin 4) *through one winding of FL1*. So a clamped transient takes this path:

```
J3.3 → CR1 → GND → FL1 winding 3→4 → VSS → J3.4
```

Current in one winding only is **common-mode** as far as FL1 is concerned, so the choke presents its full common-mode impedance in the middle of the clamp path — exactly where you want zero impedance. `DLW21SN670HQ2L` is rated **320 mA** with 0.31 Ω DCR; SMF10A is rated to 11.8 A of peak pulse current. The clamp is degraded and the choke is the part at risk.

CR1 is also **7 mm from the connector** it protects (CR1 at 46.105, 48.740; J3.3 at 53.010, 49.425), with 11.2 mm of VSYS track total.

> Move CR1 next to J3 and clamp **J3.3 ↔ J3.4 (VSYS ↔ VSS)**, both on the connector side of FL1. The transient loop then stays local to the connector and never enters the choke. Schematic + placement change only, no BOM change.

Note this is inherited — `gnss-px1105r-18mm-highpower-ext-ant` has the identical CR1/FL1/VSYS/VSS structure, so the fix applies to both boards.

### B2. The ground plane under the module is barely stitched

u-blox §4.4: *"The GND plane below the module is **filled with GND vias** to increase GND reference and to tie separate ground plane areas together."*

Inside the 15.5 × 15.5 mm module footprint there are **5 GND vias**, three of which sit at the very edge beside U1 pads 1 and 20. The other 42 GND vias ring the board perimeter. Worst module GND pad is U1.16 at **4.23 mm** from the nearest GND via.

This matters more than usual here because B.Cu — the module's own reference layer — is slotted by its escape routing (GNSS_RX 13.5 mm, GNSS_TX 12.3 mm, PG 5.9 mm, TIMEPULSE 4.8 mm, RESET 2.0 mm), and the nearest *solid* ground plane, In1.Cu, is 1.34 mm below it.

> Add a GND via grid under the module at roughly 2 mm pitch. Vias are fine there — the SAM-M10Q is a perimeter LGA with no centre contact, and tenting is already enabled front and back.

### B3. In2.Cu is a full-board +3V3 plane where the antenna wants ground

Current stackup:

| Layer | Assignment | Dielectric to next |
|---|---|---|
| F.Cu | GND pour + buck converter + local VIN/SW/VO/+3V3 zones | 0.1 mm |
| In1.Cu | **solid GND plane, completely uncut** — excellent | 1.24 mm |
| In2.Cu | **+3V3, full board** (42.22–63.87 × 29.63–56.79) | 0.1 mm |
| B.Cu | GND pour + SAM-M10Q + escape routing | — |

F.Cu/In1 is very well done: the buck's return loops are 0.1 mm from an unbroken ground plane. The problem is the other half. u-blox §4.4: *"**A solid ground plane around the antenna is required for free flow of the antenna currents.** The size and shape of the ground plane as well as discontinuities (e.g. holes or signal traces) in the ground plane may affect the radiation pattern."*

The plane 0.1 mm beneath the antenna module is +3V3, not ground. The total load on it is ~50 mA — a full-board plane is roughly 60× more copper than that needs.

> Make In2.Cu a solid GND plane. It currently carries +3V3 from the bead cluster to U1 pins 2/3 (that is the only path — the two B.Cu +3V3 zones are not otherwise connected), so this needs ~12 mm of +3V3 routing added on F.Cu down the right side (x 58–63, y 41–48 is open) plus one via. Given that the antenna is already starved for ground plane (C1), this is the single highest-value RF change available.
>
> Lower-effort fallback: keep the +3V3 on In2 as a narrow highway and fill the rest of In2 with GND.

### B4. Mounting-hole pads are floating copper

H1–H4 use `MountingHole_2.2mm_M2_DIN965_Pad`: a **3.8 mm** annular pad on `*.Cu` and `*.Mask`, with `zone_connect 2` (solid) — and **no net**. That is 3.8 mm of unmasked, electrically floating copper on all four layers at each corner, and because it is netless, the surrounding zones hold clearance around it, enlarging the void further.

Combined area removed: ~45 mm², **7.5 % of the board**, taken from the four corners of a ground plane that is already too small (C1) — precisely the *"holes … in the ground plane"* u-blox warns about. This is the same class as the missing ground on the LoRa board.

> Assign the H1–H4 pads to GND. Also consider dropping the pad diameter — 3.8 mm is generous for M2, and `DIN965` is a countersunk head that will not sit flat on a plain pad anyway.

Mechanical note while you are there: H1/H2 pads clear the module body by only **0.65 mm**, so there is no room for an M2 washer on the module side.

### B5. No decoupling at V_IO or V_BCKP

| Pin | Nearest +3V3 capacitor |
|---|---|
| U1.17 VCC | C12 at 1.48 mm, C14 at 1.49 mm ✓ |
| U1.2 V_IO | C14 at **10.29 mm** |
| U1.3 V_BCKP | C14 at **11.12 mm** |

V_IO is not a trivial rail — u-blox §4.1.2: *"V_IO supplies all the digital IOs, **clock, and the backup domain**."* Both pins are fed only by a B.Cu zone in the opposite corner, via the In2 plane.

Separately, the HF decoupling is in the wrong order at VCC: C5 (100 nF) is 4.51 mm and C6 (100 pF) is 3.83 mm from U1.17, while the 22 µF bulk caps are at 1.48 mm. The smallest capacitor should be closest.

> Add a 100 nF 0402 adjacent to U1.2/U1.3 (there is room on F.Cu opposite them, around 53–55 × 48–50), and swap C5/C6 inboard of C12/C14.

### B6. The board cannot be identified once assembled

- The only board-identifying silkscreen is `TinkerRocket / GNSS SAM-M10Q / ${REVISION}` on **B.SilkS at (53.15, 43.29)** — dead centre under U1, which spans 45.1–60.6 × 34.5–50.0. It is completely covered by the module. (This is what four of the five DRC silkscreen warnings are about.)
- **All 33 footprints have their reference designator hidden.** There is no refdes anywhere on the board. Top-side silkscreen is `PWR` and `PPS` only.

Two consequences. First, rework and bench debug have nothing to navigate by — and on this project assembly defects are a first-tier hypothesis on new boards. Second, and worse: `gnss-px1105r-18mm-highpower-ext-ant` is **also 22.00 × 27.50 mm with the same BM05B-SRSS-TB connector on the same pins**. Two different GNSS boards, identical outline, identical mating harness, and neither is labelled where you can read it.

> Move the ID text to a bottom-side strip clear of the module (y < 34.5 or y > 50.0 both work) or put it on F.SilkS. Add refdes for at least U1, U3, J3, L8, FB1, CR1 and the connector pin 1. The top strip (y 29.5–34.5) and the area around J3 are largely free.
>
> Add an orientation marker too: U1 is on **B.Cu**, so the patch antenna looks out the *bottom* of the board. Nothing on the board says which way is up.

### B7. Input high-frequency cap is not at the pins

TI, §8.2.2.2.5: *"In addition to the bulk input cap, **a smaller cap must be placed directly from the VIN pin to the PGND pin** to minimize input loop parasitic inductance."* And §10.1: *"The input capacitor or capacitors should be placed as close as possible to the VIN and PGND pins. **This is the most critical component placement.**"*

| Cap | to U3.6 (VIN) | to U3.4 (PGND) |
|---|---|---|
| C7 (2.2 nF) | 1.58 mm | 1.57 mm |
| C9 (22 µF) | 3.11 mm | 3.02 mm |
| C8 (22 µF) | 5.03 mm | 4.98 mm |

U3.6 and U3.4 are at the same x (50.115) and exactly **1.0 mm apart in y** — a 0402 rotated 90° lands its pads on 43.59/44.55 against pin centres of 43.570/44.570. That is a near-perfect bridge.

> Relocate C7 to bridge U3.6 and U3.4 directly. The spot is currently occupied by the Net-(U3-PG) via at (49.318, 44.102) — which disappears if you also act on B8 and delete the dangling PG pull-up.

### B8. Three dangling connections

**J3 pin 5 (`RXD2`) is a single-pad net.** It goes nowhere on this board. But it is not a spare pin on the system: on `rocket-computer`, J1 pin 5 is `GNSS_RXD2` wired directly to **ESP32-P4 pad 2**, and on the sibling `gnss-px1105r` board the same pin reaches U1.15. So the host has a GPIO sitting on an unterminated stub. Harmless if firmware drives it; floating if firmware reads it.

> Best use is **TIMEPULSE (PPS)** — right now PPS only drives a 0.13 mA LED and never reaches the flight computer, and a 1 PPS edge into a P4 GPIO is genuinely useful for time-stamping. **Two conditions if you do this:** put a 1 kΩ series resistor in it (matching R7/R8), and make it a firmware rule that the host pin stays an input. u-blox §3.2.3.3: *"The TIMEPULSE and SAFEBOOT_N functions share the same internal IC function. If this pin is low at receiver startup, the receiver will enter safeboot mode. Make sure there is no load on this pin that could pull it low at startup."*
>
> Safer alternative: route **RESET_N** there instead (input-only, internal pull-up, no startup hazard) for a hard-recovery path — at the cost that a reset clears the BBR and forces a cold start.

**R4 / PG.** U3.5 (PG) is pulled to VO through R4 (10 k) and connects to nothing else. TI: *"[PG] can be left open or tied to GND if not used."* Delete R4 and its two vias; that also frees the spot needed for B7.

**J3 pads 6 and 7** (the JST hold-down tabs) have no net. They are mechanical only and do not mate electrically, so this costs nothing functionally — but the host board ties its equivalents (J1.6, J1.7) to GND, and netless pads leave voids in the pour. Tie them to GND for consistency.

---

## C — Consider / accept knowingly

**C1. The ground plane is well below u-blox's minimum.** This is the board's defining limitation and it should be recorded rather than fixed. u-blox §4.4:

> *"The optimal radiation pattern is achieved with a 50 × 50 mm² ground plane. … on a small ground plane the antenna gain and radiation efficiency are reduced. **Significant degradation in performance occurs with a ground plane smaller than 40 × 40 mm².**"*
> *"It is recommended not to place anything closer than 10 mm to each edge of SAM-M10Q."*

| | Required | This board |
|---|---|---|
| Ground plane | 50 × 50 mm (2500 mm²) optimal, 40 × 40 mm (1600 mm²) before significant degradation | 22.0 × 27.5 mm = **605 mm²** |
| Plane extension past module | — | 3.25 mm in X, 6.0 mm in Y |
| Clearance to nearest component | ≥ 10 mm from each module edge | every part except J3 sits *inside* the module footprint |

The module *is* centred on the board (offset 0.3 × 1.0 mm from centre), which is what u-blox asks for and is the one part of this you can control. Expect reduced gain and a distorted low-elevation pattern; this is the price of a patch-antenna module in an 18 mm airframe. It does make B2/B3/B4 worth more than they would be on a larger board, since each one recovers a piece of the little plane there is.

**C2. S-CONF = VIN disables spread spectrum.** U3.10 tied to VIN is a legal, documented mode (Table 7-1: 2.2 MHz, spread spectrum OFF, output discharge OFF, no sync) — not a floating pin. But the 716th harmonic of 2.2 MHz falls at 1575.2 MHz, 0.2 MHz off the GPS L1 carrier, and spread spectrum is the standard mitigation for a switcher this close to a GNSS receiver. It costs one resistor:

| R<sub>S-CONF</sub> to GND | Result |
|---|---|
| 7.5 kΩ | 2.2 MHz, **random** spread spectrum, discharge off |
| 6.04 kΩ | 2.2 MHz, **triangle** spread spectrum, discharge off |
| 42.2 kΩ | 2.2 MHz, **random** spread spectrum, **discharge on** |

42.2 kΩ is the interesting one — output discharge also gives a clean rail collapse when the host opens the ground switch. Worth a bench A/B on C/N₀ if you have the setup.

**C3. Both LEDs run at 0.13 mA** — (3.3 − 2.0)/10 kΩ, through R1 (PPS) and R3 (PWR). That is barely visible indoors, and D1 is additionally gated by TIMEPULSE's low duty cycle. 1.5–2.2 kΩ gives 0.6–0.9 mA, ~7× brighter, still <4 % of the module's own draw and inside the 2 mA TIMEPULSE spec.

> **Do not replace D1 with a plain resistor to ground, and do not fit a low-V<sub>f</sub> part.** The only thing keeping SAFEBOOT_N out of the safeboot window at startup is the LED's forward drop (~1.8–2.4 V for green) clamping the node far above logic-low. As drawn it is safe at any R1 value; as a bare resistor divider it would not be.

**C4. C15 is a remote output cap** — 5.63 mm from the inductor and 6.59 mm from PGND, against TI's *"minimize the length of the connection from the inductor to the output capacitor"* and *"place the output capacitor ground close to the PGND pin."* Its ESL contribution to the first-stage COUT is degraded. Either pull it into the C11/C13 cluster or treat it explicitly as bulk (and then recheck A1's 40–80 µF window against the two caps that are actually close).

**C5. R2 is redundant.** u-blox §3.2.3.1: RESET_N *"is input-only with an internal pull-up resistor to V_IO and should be left open for normal operation."* R2 (1 k to +3V3) duplicates the internal pull-up on a pin that goes nowhere else. Harmless — no capacitor to ground, which is the thing the manual actually forbids — but it can be deleted, or better, repurposed as the series resistor if you route RESET_N to J3 pin 5 (B8).

**C6. Stackup and finish are not aligned with the other boards.** `copper_finish` is `"None"`; both `lora-daughterboard` and `base-station` were set to **ENIG** during their pre-fab reviews. Inner copper is modelled as 0.035 mm (1 oz) where JLC's 4-layer default is 0.5 oz. Nothing here needs controlled impedance, so the dielectric numbers do not matter much — but ENIG does: the SAM-M10Q is a 20-pad LGA, and HASL's domed pads hurt coplanarity and standoff on flat-pad packages.

> Set the stackup to the real JLC 4-layer profile and `copper_finish "ENIG"`, matching what the other two boards already got. `gnss-px1105r` needs the same treatment.

**C7. Host power-gating is not a clean off.** `rocket-computer` feeds J1.3 from VBATT through a ferrite (always live) and switches only the **ground** return via Q1. When Q1 opens, VSYS remains applied and the board's ground floats. Meanwhile the host may still be driving GNSS_RX into U1.14. u-blox's design-in table is explicit: *"Do not drive IO pins when VCC and V_IO are not supplied. Otherwise permanent damage may result. If driving the IO pins cannot be avoided, buffers are required."*

R7/R8 (1 kΩ series on both UART lines) are the right mitigation and bound the injected current to a few mA — that is why they are there and they should stay. The remaining gap is procedural:

> Make it a firmware rule that the host tri-states GNSS_RX (and J3 pin 5, if you populate it) **before** opening Q1, and re-drives only after closing it.
>
> Longer term, the architecturally correct answer is u-blox's own: route **EXTINT** to a host GPIO and use host-controlled on/off (PSMOO). That puts the receiver into backup mode without cutting power at all — preserving the BBR for hot starts and removing the back-drive question entirely. There is exactly one spare connector pin, so this competes with the PPS proposal in B8.

**C8. No explicit no-connect flags.** U1 pins 8 (SAFEBOOT_N), 9 (SDA), 12 (SCL) and 19 (EXTINT) are left open, which is correct — u-blox Appendix B.1: *"For an absolute minimum design using UART, other PIOs (RESET_N, EXTINT, TIMEPULSE, SDA, SCL, SAFEBOOT_N) can be left open."* But the schematic has zero no-connect flags, so intent is not recorded and ERC cannot distinguish deliberate from forgotten. Add them.

---

## D — Verified correct; do not change

Recorded so these do not get re-litigated on the next pass.

- **CR1 = SMF10A is exactly right, and the reasoning in its Notes field checks out.** Standoff 10 V clears an 8.4 V full 2S pack; V<sub>C</sub> max 17 V at 11.8 A sits under the TPS62913's 18 V VIN absolute maximum with 1 V to spare. SMF12A would clamp at 19.9 V and is indeed not a substitute.
- **FB1 = Würth 782853200 is well chosen.** TI §8.2.2.2.4 asks for 8–20 Ω at 100 MHz, DCR below 10 mΩ, and a current rating far above the load. The part is 20 Ω / 8 mΩ / 5 A → L ≈ 20/(2π·10⁸) = **31.8 nH**, inside the ≤ 50 nH stability limit for the internal compensation. It also clears u-blox's *"do not add series resistance greater than 0.2 Ω on the supply line"* by 25×.
- **The power stage tracks TI's reference design closely.** C<sub>NR/SS</sub> = 470 nF is TI's own characterization value (→ 5.0 ms soft start at 75 µA to 0.8 V). R6 = 4.87 kΩ is TI's reference value verbatim. With V<sub>FB</sub> = 0.800 V, 15.4 k/4.87 k gives **3.330 V** — 0.9 % high, comfortably inside the SAM-M10Q's 2.7–3.6 V range. C<sub>FF</sub> unpopulated matches the datasheet's baseline PSRR condition. L1 = 2.2 µH matches the nominal; the VLS3012CX-2R2M-1 at ~2.55 A / 74 mΩ is heavily over-specified for a ~50 mA load, which is fine.
- **VO (U3.3) is connected directly after the inductor**, per §5 Pin Functions. PSNS is on GND (it just needs the via, A3). S-CONF tied to VIN is a real table entry, not a floating pin.
- **The switch node is small** — 1.6 mm² of F.Cu zone, inductor 2.5 mm from the SW pin — per *"minimize the copper area at the switch node."*
- **In1.Cu is a completely uncut GND plane 0.1 mm below the buck.** This is the best single thing about the layout and it is why the input-loop and FB findings are risks rather than certainties.
- **U1 and FL1 solder mask and paste are present.** They are drawn as `fp_poly` on B.Mask/B.Paste and F.Paste rather than being generated from the pad layer list, which is the SnapEDA convention — easy to misread as missing. U1 gets 1.6 × 1.9 mm mask openings on 1.5 × 1.8 mm pads and a 2 × 2 windowpane paste pattern at ~62 % coverage, which is right for a large LGA.
- **Zone pad connection is `thru_hole_only`** — SMD pads solid, PTH thermally relieved. Solid is the better choice for U1's ground pads; u-blox offers airgap reliefs only as an optional soldering aid.
- **R7/R8 (1 kΩ series on the UART)** are correct and load-insignificant at any baud this link will run.
- **Copper-to-edge clearance is 0.5 mm**, so the In2 power plane is properly pulled back from the board outline.
- **DRC is clean.** The 4 `lib_footprint_mismatch` warnings (U3, R1, R2, D1) are library-sync drift and worth resolving before the fab package is generated, but they are not errors.

---

## Verification method

- Netlist, pad geometry, zone/track/via extraction parsed directly from `gnss-sam10m8-18mm-hv.kicad_pcb` (KiCad 10 s-expression, `(net "NAME")` inline form).
- `kicad-cli pcb drc --severity-all` for DRC; `kicad-cli pcb render` and per-layer SVG export for visual inspection of all four copper layers.
- All clearances, distances and areas above are computed from the board file, not read off a plot.
- Datasheet text extracted from the TI and u-blox PDFs directly rather than from search summaries; part specs for FB1, FL1, L8 and CR1 taken from distributor/manufacturer data.
- Host-side interface cross-checked against `rocket-computer.kicad_pcb` (J1, Q1, U17) and `gnss-px1105r-18mm-highpower-ext-ant.kicad_pcb` (J4, RXD2).

## Sources

- [TI TPS62912/TPS62913 datasheet, SLVSFP4B](https://www.ti.com/lit/ds/symlink/tps62913.pdf)
- [u-blox SAM-M10Q Integration Manual, UBX-22020019](https://content.u-blox.com/sites/default/files/documents/SAM-M10Q_IntegrationManual_UBX-22020019.pdf)
- [u-blox SAM-M10Q Data Sheet, UBX-22013293](https://content.u-blox.com/sites/default/files/documents/SAM-M10Q_DataSheet_UBX-22013293.pdf)
- [Würth WE-CBA SMT EMI suppression ferrite bead series](https://www.we-online.com/en/components/products/WE-CBA)
- [Murata DLW21S common mode choke coil data](https://pim.murata.com/asset/pim4/commonModeChokeCoilCommonModeNoiseFilter/EFLC0010_PDF_COMMONMODECHOKECOILCOMMONMODENOISEFILTER)
- [TDK VLS3012CX-2R2M-1 product page](https://product.tdk.com/en/search/inductor/inductor/smd/info?part_no=VLS3012CX-2R2M-1)
- [Littelfuse SMF series TVS diode datasheet](https://m.littelfuse.com/~/media/electronics/datasheets/tvs_diodes/littelfuse_tvs_diode_smf_datasheet.pdf.pdf)
