# TinkerRocket LoRa daughterboard ("LoRa V3") — Pre-Manufacturing Review

**Date:** 2026-07-30
**Source:** `TinkerRocket-Hardware/hardware/lora-daughterboard/` @ `4eebebe` (branch `hw/657-p4-v3x-provisions`, tree clean)
**Scope:** every component + its supporting parts, system architecture, PCB layout beyond DRC (datasheet compliance, best practice), and "anything dumb" — the lora-missing-ground class.
**Method:** fresh `kicad-cli` netlist / ERC / DRC / BOM / gerber / render export from a scratch copy of the live files; s-expression parsing of the `.kicad_pcb` for geometry; gerber-level verification of mask/paste/copper; datasheets pulled for TPS62913, E220-900MM22S, ESP32-S3 HDG, ECS crystal, CUS10S30; cross-checks against `rocket-computer`, `base-station` and `gnss-*` netlists and against `tinkerrocket-idf/projects/radio_board`.

**Board:** 22.0 × 27.5 mm, 6-layer (F / In1 GND / In2 +3V3 / In3 GND+routing / In4 GND / B), 1.6 mm.
Top: E220-900MM22S LoRa module, edge SMA, TPS62913 buck + L8 + ferrite bead, ORing Schottkys, CM choke.
Bottom: ESP32-S3RH2 (QFN-56), W25Q128 boot flash, USB-C, JST-SH host link, both crystals.

**Baseline health: DRC 23 violations (1 error), 0 unconnected, 0 schematic-parity issues.** Nothing here is a "this board is dead" defect — the RF path, the buck topology and the module wiring are all correct. The list below is a must-fix set plus a decide-consciously set. (Finding 2 was closed during review — the intended ferrite bead was confirmed and verified compliant.)

---

## What is verified correct (worth stating — a lot of this is done well)

- **E220-900MM22S wiring is pin-for-pin correct against the datasheet**, all 20 pins: VCC/GND/NRST/ANT/GND/TXEN/RXEN/BUSY/MISO/MOSI/NSS/SCK/GND/DIO1, NC on 4/5/8/17, DIO3 floating. Critically, **DIO2 → TXEN and RXEN → MCU GPIO** is exactly the split EBYTE prescribes ("DIO2 can be connected with TXEN, not with the IO port of MCU"). The `radio_board` firmware already models it (`LORA_RXEN_PIN = 35 // TX side is DIO2-driven`).
- **RF path is genuinely well executed.** ANT → SMA is 3.53 mm of 0.2 mm F.Cu over a solid In1 GND plane at 0.1 mm prepreg, with a 0.1275 mm coplanar gap → ≈46–48 Ω GCPW (calculated); GND stitching vias flank both sides at ~0.75 mm pitch, 1.3–1.4 mm off the centreline; In1 is unbroken under the whole corridor; the SMA is grounded on both F and B. Module GND pads 2/7/16 each have vias in-pad.
- **TPS62913 circuit is TI's front-page typical application**: R6 15.4 k / R7 4.87 k → 0.8 × 4.162 = **3.33 V**, C10 470 nF on NR/SS, C7 2.2 nF on VIN, L8 2.2 µH — the exact reference values. VO (pin 3) senses directly after L8 and the FB divider senses after the bead: that is the intended remote-sense-through-the-second-stage-filter topology, not a mistake.
- **S-CONF = 7.5 kΩ → 2.2 MHz + random spread spectrum, discharge off.** A deliberate and good choice for a board with a receiver on it.
- **EN/SYNC tied to VIN is legal** — TPS62913 abs max on VIN/EN/SYNC/PG/S-CONF is 18 V; the 2S pack tops out at 8.4 V.
- **FB1 is 9.4 mm from U3 and 6.55 mm from L8** — well outside TI's ferrite-bead keepout region (Fig. 10-2).
- **L10 = 24 nH in series with XTAL_P is prescribed**, not a mystery part: ESP32-S3 HDG says "add a series component on the XTAL_P clock trace… an inductor of 24 nH to reduce the impact of high-frequency crystal harmonics."
- **R75/R76 = 22 Ω on USB D+/D− is per Espressif** ("reserve series resistors (initial value can be 22/33 Ω)"), and they sit ~2 mm from the S3 pins with the TVS at the connector.
- **USB-C**: 5.11 k Rd on both CC (correct UFP), SP0503BAHTG TVS on VBUS/D+/D−, ORing Schottkys CUS10S30 (30 V/1 A, Vf 230 mV @100 mA) — CR3 also gives reverse-polarity protection on the battery input.
- **Cross-board link is correct**: J6.1/2/3/4 = GND / +BATT / host-RX / host-TX matches rocket-computer J5 (`Net-(J5-Pin_1)` via Q10, `Net-(J5-Pin_2)` via FL2→VBATT, `LoRa_RX`→U15 GPIO10, `LoRa_TX`→U15 GPIO11) and base-station J6 (GND, V_LORA≈5.0 V from the TPS61023, GPIO36/GPIO35). Both host supplies land inside the TPS62913's 3–17 V VIN window (≈4.5 V from the BS after the Schottky + choke; 6.0–8.0 V from the rocket).
- **Firmware pin map matches the schematic 1:1** on all eight LoRa signals (SCK 17, MISO 33, MOSI 21, CS 18, DIO1 2, RST 38, BUSY 34, RXEN 35) and both LEDs (GPIO7→D5 red, GPIO8→D4 blue).
- **GPIO45 NC → internal pulldown → 3.3 V VDD_SPI mode** ✓; GPIO46 pulled down ✓; GPIO0 on the tact switch with the S3's internal pull-up ✓. No strapping pin is mis-set.
- **The odd `(layers "B.Cu" "B.Paste")` / `(layers "B.Cu" "B.Mask")` pad layer lists in the custom footprints are cosmetic only.** Verified at gerber level: every SMD pad gets a mask opening, and the S3 thermal pad gets a proper 3×3 × 1.0 mm windowpane paste pattern (~54 % coverage). Only the two fiducials lack paste, which is correct. *(This does make file-level audits misleading and would bite on export to another tool — worth tidying, but it is not a defect.)*
- No different-net via inside any pad; D+/D− do not cross the In2 plane slots; all footprints have courtyards.

---

## Must fix before fab

### 1. The CHIP_PU RC capacitor was on the wrong side of the resistor — **FIXED (schematic), PCB pending**
Both parts of Espressif's reset network were present and both were the prescribed values (**R = 10 kΩ, C = 1 µF**) — but C95 was wired to the supply side of R73 instead of the CHIP_PU node, so it acted as rail decoupling and formed no delay.

**Fixed in the schematic 2026-07-30.** `Net-(U28-CHIP_PU)` is now {C95.1, R73.2, U28.4} and R73 pin 1 goes to +3V3 — the RC is correct. **PCB still pending:** the layout was not touched, so C95's copper still ties to +3V3 (pad at (86.75,123.91) B.Cu). Run *Update PCB from Schematic*, re-route, and move C95 next to U28 pin 4 at (90.20,122.61) — alongside R73 at (89.63,123.78) — since 3.7 mm is fine for a bypass cap but too far for a reset RC.

The original analysis, for the record — as drawn:
```
+3V3 (#PWR0235 @154.94,25.4)
  |
  +---- junction (154.94,26.67) ---- C95 1 uF ---- GND (#PWR0238 @160.02,34.29)
  |
  +---- R73 10 k ---- CHIP_PU ---- U28.4
```
Netlist confirms: `+3V3` contains {C95.1, R73.2, ...}; `Net-(U28-CHIP_PU)` = {R73.1, U28.4} — two nodes.

Espressif's ESP32-S3 schematic checklist requires ≥50 µs between the rails being stable and CHIP_PU going high. That delay comes from the 10 kΩ charging a capacitor **on the pin**. With C95 on the supply side, the only capacitance on the CHIP_PU node is pin 4's input capacitance plus trace (~5 pF): **τ ≈ 10 kΩ × 5 pF ≈ 50 ns**, i.e. CHIP_PU rises in lockstep with +3V3 (and C95, by slowing the rail slightly, if anything tightens that lockstep). Moved to the pin: **τ = 10 kΩ × 1 µF = 10 ms**, comfortably past the 50 µs floor.

As drawn, every cold boot and every brownout recovery was out of spec. The rocket-computer review raised the same issue (H8) as a missing capacitor; here the capacitor existed and was simply on the wrong node.

**While you are there:** there is no reset button and no CHIP_PU test point. The only way to reset this board is to unplug it. Add at least a pad.

### 2. FB1 — RESOLVED: the intended part is Würth **782853200**, and it checks out
*(Raised in review as "value-only, could be anything"; the designer confirmed the intended order code. Verified against the datasheet and TI's requirements — no change needed beyond recording it.)*

The FB divider senses **after** FB1, so the bead is part of the compensated plant. TI: **8 Ω to 20 Ω at 100 MHz**, **DCR below 10 mΩ**, current rating well above the load, and internal compensation "designed to be stable with up to **50 nH**".

| TI requirement (§8.2.2.2.4, Table 8-6) | 782853200 | |
|---|---|---|
| Impedance 8–20 Ω @ 100 MHz | 20 Ω ±25 % | ✓ top of window |
| DC resistance below 10 mΩ | 0.008 Ω max | ✓ |
| Current rating >> load | 5000 mA vs ~180 mA (28×) | ✓ |
| Inductance ≤ 50 nH | 31.8 nH nom, **39.8 nH at +25 %** | ✓ worst-case unit inside |

The tolerance case is the one that matters and it passes. It is also the only compliant choice at this size: **782853200 is the lowest-impedance 0805 High Current member Würth makes**; the next step up (782853270, 27 Ω) is 43 nH nominal / 54 nH at +25 %, which breaks the 50 nH limit.

The land pattern already matches: Würth's recommended pad is 1.0 × 1.2 mm with total span **W = 4.0 for the High Current type**, and the footprint is literally named `WE-CBA_0805_W4.0` (pads 1.4 × 1.0 at ±1.3 → span 4.0, width 1.0 — 0.2 mm extra toe, otherwise identical). Fab outline 2.0 × 1.2 mm matches the body.

**Remaining action:** none electrically — just get `782853200` into the BOM (see finding 3), because nothing in the schematic, BOM or footprint records it today.
**Note for bench:** TI's own examples cluster at 8–10 Ω / 12.7–15.9 nH, so this runs ~2× their inductance — worth ~3 dB more ripple attenuation at 2.2 MHz at the cost of phase margin. In spec, but it makes the first-article loop-response measurement (work order step 6) worth doing rather than skipping.

### 3. BOM had no MPN field — **capacitors DONE**, remaining parts still open
No component carried an MPN field; a handful of SnapEDA-sourced parts (J2, L8, S3, U3) have inconsistent `MP`/`MF`/`MANUFACTURER`/`DigiKey_Part_Number` fields, so no single export produced a purchasable BOM, and **no passive had a voltage rating, dielectric or tolerance.**

**Done 2026-07-30 (capacitors).** All 27 capacitors now carry `MPN`, `Mfr` and `MinVRating` fields in the schematic, sourced from `Circuit Board BOMs/TinkerRocket_CrossBoard_BOM.xlsx` (sheet "Cross-Board BOM"). Field names mirror the spreadsheet columns so the two stay diff-checkable; `MinVRating` is the *design requirement*, not the part's rating (e.g. the 470 nF requirement is 6.3 V while `CL05A474KP5NNNC` is a 10 V part). Verified: nets unchanged, ERC error count unchanged, BOM export emits all three fields on every capacitor row.

| Value | MPN | Mfr | MinVRating | Refs |
|---|---|---|---|---|
| 2.2 nF 0402 | CC0402KRX7R9BB222 | Yageo | 16 V | C7 |
| 22 uF 0805 | CL21A226MOQNNNE | Samsung | 16 V | C8, C9, C11–C16 |
| 470 nF 0402 | CL05A474KP5NNNC | Samsung | 6.3 V | C10 |
| 100 nF 0402 | CL05B104KO5NNNC | Samsung | 16 V | C17, C99, C101, C103, C104 |
| 22 pF 0402 | CL05C220JB5NNNC | Samsung | 6.3 V | C89, C91 |
| 18 pF 0402 | CL05C180JB5NNNC | Samsung | 6.3 V | C90, C92 |
| 1 uF 0402 | CL05A105KO5NNNC | Samsung | 16 V | C95, C96, C100, C105 |
| 10 uF 0402 | CL05A106MQ5NUNC | Samsung | 6.3 V | C97, C102, C106 |
| 10 nF 0402 | CL05B103KB5NNNC | Samsung | 6.3 V | C98 |

**COUT changed in the same pass — see finding 3a.**

**Still open (non-capacitor):**
- **D4 "Blue" / D5 "Red" / D6 "LED"** — no part, no Vf (see finding 6).
- **Y5 "SC-32S-32.768kHz-12.5pF"** — Espressif requires **ESR ≤ 70 kΩ**, which is exactly the typical max for this size class. Pin the grade.
- FB1 = 782853200 (finding 2) — the Value field carries it; it should also get `MPN`/`Mfr` for consistency with the caps.
- U22's `Datasheet` field still reads `W25Q64JVXGIQ TR`, inherited from the symbol it was derived from (the Description property says so). Harmless today because nothing orders from it — but it is a live trap now that MPN fields exist. Clear it.
- The resistors, inductors, connectors, crystals and ICs still have no `MPN`/`Mfr`; the spreadsheet has rows for them, so the same one-pass annotation would close it.

### 3a. COUT: 2 × 47 µF 6.3 V → 3 × 22 µF 16 V — **DONE (schematic), PCB pending**
C11/C13 were `CL21A476MQYNNNE` (47 µF, **6.3 V** X5R, 0805) on `Net-(U3-VO)`, which is the TPS62913's COUT — specified as **40 µF min / 47 nom / 80 µF max effective**. A 6.3 V 0805 X5R sitting at 3.3 V is at 52 % of rated voltage, where that class typically loses 55–70 %: two of them land ≈28–42 µF, straddling and probably under the floor.

Changed to **three × `CL21A226MOQNNNE`** (22 µF, **16 V**, 0805) — already a stocked line on this board, so no new BOM row. At 3.3 V it is only at 21 % of rated, so it derates far less: ≈42–53 µF for three, inside the window. This also matches TI's own Table 8-5 recommendation ("3 × 22 µF, 10 V, X7S, 0805").

Applied: C11 and C13 revalued, **C14 added** at schematic (179.07, 150.495) with `#PWR096` GND and a junction on the VO wire at (179.07, 146.685). Netlist verified: `Net-(U3-VO)` = {C11.1, C13.1, C14.1, FB1.1, L8.2, R4.2, U3.3}.

**PCB pending:** run *Update PCB from Schematic*, then place C14. Free F.Cu slot confirmed at **(93.40, 128.8)** — 2.6 × 1.8 mm clear, directly under L8's VO pad and already inside the `Net-(U3-VO)` pour.

**Caveat:** the derating percentages are the usual range for the class, not measured — confirm both parts against Samsung's DC-bias curves. If the 22 µF derates worse than expected, a fourth (≈56–70 µF) still fits inside the 80 µF ceiling.

### 4. VDD_SPI: no 0.1 µF at the pin, a 10 µF where Espressif says not to, both 6 mm away — and the boot flash shares the internal 14 Ω
`OUT_VDD_SPI` = {U28.29, U22.8 (VCC), C96 1 µF, C97 10 µF}. C96/C97 are at (92.15, 113.6–114.6) — **1.6 mm from the flash but 6.1–6.3 mm from U28 pin 29** at (86.20,115.76). The rail reaches the chip over a 0.2 mm, 6.6 mm-long trace cut through the In2 plane.
Espressif: "add extra **0.1 µF and 1 µF** decoupling capacitors **close to VDD_SPI**. Please do not add excessively large capacitors." There is no 0.1 µF at all, the 1 µF is 6 mm away, and the 10 µF is the thing they warn against.
Separately, this is the same **R_SPI budget** question as the rocket computer (H7): in 3.3 V mode VDD_SPI is fed internally from VDD3P3_RTC through a **14 Ω** resistor, and on the **S3RH2 the in-package 2 MB quad PSRAM hangs on that same node** (min 2.7 V), as does the external W25Q128 (min 2.7 V, ≤20 mA read, ≤25 mA program/erase). Flash alone: 3.3 − 0.025×14 = 2.95 V — fine. Flash + PSRAM active (~50 mA): 3.3 − 0.70 = **2.60 V, below both minimums**, on the chip's *boot* flash.
**Fix:** move U22 pin 8 off `OUT_VDD_SPI` to +3V3 (this board's +3V3 is a low-noise TPS62913 output, so it is clean), leave VDD_SPI with 0.1 µF + 1 µF **at pin 29**, and drop C97 to 1 µF.
**If instead you keep the shared node:** it only works while firmware never enables PSRAM. `radio_board/sdkconfig.defaults` does not enable SPIRAM today — write that constraint down, because turning it on later silently breaks the rail.

### 4a. VDD_SPI bulk: C97 stays at 10 µF (the HDG warning does not apply here)
Finding 4 originally cut C97 from 10 µF to 100 nF on the strength of Espressif's *"do not add excessively large capacitors"*. Tracing the mechanism behind that warning, neither driver applies to this board, so **C97 was put back to 10 µF**:

- **Startup delay.** VDD_SPI charges through the same 14 Ω, so it lags VDD3P3_RTC by τ = 14 Ω × C. At 1 µF + 10 µF (≈5.6 µF effective) τ ≈ 78 µs, settling in ~400 µs. CHIP_PU now releases on a 10 kΩ × 1 µF RC, ~2–3 ms after the rail — VDD_SPI is up an order of magnitude early. Even 100 µF would only settle in ~7 ms.
- **Light-sleep wake.** Table 5-10: *"Light-sleep: VDD_SPI and Wi-Fi are powered down."* Recovery has to recharge through the 14 Ω. This board is an always-on modem on the host's switched ground and never light-sleeps.

Against that, the 14 Ω makes VDD_SPI the **highest-source-impedance rail on the board**: a 30 mA transient through it is a 420 mV dip unless local capacitance supplies it. Bulk is worth more here than anywhere else, so the 10 µF earns its place.

Open nit: the node is 1 µF + 10 µF with **no 100 nF**, so nothing covers above the bulk caps' self-resonance (~3–5 MHz in 0402) for a quad-SPI PSRAM clocking to 80 MHz. Changing C96 1 µF → 100 nF would close it at zero part count. Raised and consciously declined.

Separately: **C18 (the flash's +3V3 decoupling) stays 100 nF.** The bulk argument above does *not* transfer — +3V3 is a plane with no series impedance, so its ~57 µF is already electrically local to U22, and the 100 nF is the only cap at that pin covering the HF decade. Headroom was checked anyway: swapping it to 10 µF would put Cf at ~61 µF and COUT+Cf at ~109 µF, both inside TI's 160 / 200 µF ceilings — permitted, just not useful.

### 5. ~~The 40 MHz crystal is 11 mm from the S3~~ — **RETRACTED, the layout is compliant**
*Original claim: Y6 sits 11 mm from U28 with 10.9/11.5 mm traces to XTAL_P/XTAL_N, and should be moved adjacent to pins 53/54. **That was wrong**, and the proposed fix would have been a regression. Retracted 2026-07-30 after checking Espressif's ESP32-S3 PCB layout guideline rather than assuming a "keep it short" rule.*

The guideline specifies a **minimum**, not a maximum: *"The crystal should be placed **far from** the clock pin to avoid interference on the chip. The gap should be **at least 2.0 mm**."* No maximum distance appears anywhere in it. Checked against the board:

| rule | board | |
|---|---|---|
| "gap should be at least 2.0 mm" | 7.90 mm | ✓ |
| "There should be no vias for the clock input and output traces" | 0 vias on XTAL nets | ✓ |
| "It is best not to route any signal trace under the crystal" | 0 non-clock segments under Y6's courtyard | ✓ |
| "two sides of the clock trace should be surrounded by ground copper" | GND pour on both sides | ✓ |

The proposed fix failed on two counts: placing Y6 at 0.62 mm from pins 53/54 would have **violated the 2.0 mm minimum**, and relocating it to F.Cu requires two vias on the clock traces, which the guideline **explicitly prohibits**. It would also have moved the crystal to the same side as L8 at ~2.6 mm, against "do not place any magnetic components nearby."

**What actually remains, both minor:**
- **Ground via density.** The guideline asks for "high-density ground vias" encircling the clock trace. There are 9 GND vias within 1.5 mm of the run across 24.6 mm — 0.37/mm, one every ~2.7 mm. Tightening toward ~1/mm is cheap and is the only place the layout is light against a stated rule.
- **L8 proximity.** A 2.2 µH buck inductor 4.52 mm away, against "do not place any magnetic components nearby… for example large inductance component." Judged acceptable — but be precise about *why*, because the intervening copper is worth less than it looks for this mechanism:
   - A buck couples by **E-field** (SW node dV/dt) and by **H-field** (inductor stray flux); this rule is the H-field one.
   - Against the E-field the stack is an excellent shield at every frequency here — In1 is 0.1 mm under F.Cu.
   - Against the H-field, 35 µm copper only shields above ~1 skin depth: **6.9 dB/layer at the 2.2 MHz fundamental** (δ = 44.0 µm) rising to **29.5 dB/layer at 40 MHz** (δ = 10.3 µm). Nearly transparent where the switcher is loudest.
   - That lands the right way regardless: only energy near 40 MHz can pull the oscillator — the ~18th harmonic, already far down and smeared by the random spread spectrum (S-CONF 7.5 k, and smearing widens with harmonic number). 2.2 MHz coupling shows up at worst as low-level phase modulation.
   - Geometry caveat: L8→Y6 is 4.52 mm lateral / 1.60 mm vertical — **20° off the plane of the board**, so the planes are not broadside to the coupling path and per-layer figures overstate the benefit.
   - **The real mitigations are the shielded inductor** (VLS3012CX, "Shielded Wirewound" — 20–30 dB less stray field than an open part) **and distance at 1/r³** (4.79 mm 3D). For contrast, the retracted F.Cu proposal at ~2.6 mm would have been **+16 dB** of H-field — an independent second reason it was wrong.
   - Cheapest lever if more margin is ever wanted: GND via density along the clock traces, not moving parts.

**Lesson for the next review:** "component should be close to the pin" is a real rule for decoupling and a *false* one for crystals on this part. Check the PCB layout guideline, not just the schematic checklist — they are separate documents and only the former covers placement.

### 6. Load caps on Y6 are ~2 pF too high for a 10 pF crystal
ECS-400-10-37B2-CKY-TR is **CL = 10 pF**, 40 Ω ESR, ±10 ppm. The board fits **18 pF** on each leg: C_L = 18·18/36 + C_stray = 9 + (~1 pF trace + ~0.5 pF pad + ~2 pF pin) ≈ **12.5 pF**, i.e. roughly **−30 ppm** of pulling — outside the crystal's own ±10 ppm and outside Espressif's "tune to within ±10 ppm" instruction.
Harmless for USB (±2500 ppm) and irrelevant to the LoRa link (the E220 has its own 32 MHz crystal), so it is not a boot risk — but fix it while the crystal is moving anyway.
**Fix:** fit **12 pF**, then trim on the first article. (The stray estimate assumes the existing 11 mm routing, which finding 5 now confirms is staying — so 12 pF, not the 12–15 pF range originally given when the crystal position was still in play.)

### 7. Host-UART polarity: the firmware placeholder is the reverse of the hardware
Cable pin 3 = host RX ↔ daughterboard **GPIO6**; cable pin 4 = host TX ↔ daughterboard **GPIO5**. Both hosts confirm the convention (`out_computer/board_v8.h`: `LORA_UART_TX_PIN = 11 // LoRa_TX (label-perspective)` → J5.4; `base_station/board_v3.h`: TX = 35 → J6.4).
`radio_board/main/config.h` currently has:
```
static constexpr int HOST_UART_TX = 5;   // TODO: TBD from V8 schematic
static constexpr int HOST_UART_RX = 6;   // TODO: TBD from V8 schematic
```
That is backwards. As written the modem drives GPIO5 onto cable pin 4 — **the same wire the host is driving** (two push-pull CMOS outputs in contention) — and listens on GPIO6, which nothing drives. The TODO comments say the values were never resolved; now they can be: **TX = 6, RX = 5**.
Note the root cause is a naming trap worth fixing in the schematic too: on the daughterboard the net named `LoRa_TX` is the pin the daughterboard **receives** on (the labels are host-perspective on all three boards). Rename to `HOST_TX`/`HOST_RX`, or annotate the sheet, or the next person will re-derive it wrong.

---

## Should fix / decide consciously

### 8. VDD3P3_RTC (pin 20) has no local 0.1 µF
Espressif asks for a 0.1 µF close to each digital supply pin. Pin 46 (VDD3P3_CPU) has C101 at 1.34 mm ✓; **pin 20's nearest cap on +3V3 is 4.27 mm away** (C99), because the space to the left of U28 is taken by Y5 and the USB resistors. A 0402 fits around (84.3, 118.5).

### 9. LED series resistors are 10 kΩ — the blue LED may not light at all
R59/R60/R3 = 10 kΩ from a 3.3 V rail. Red (Vf ≈ 1.9 V) → **140 µA**; blue (Vf ≈ 2.7–3.4 V) → **0–70 µA**, and a blue part with Vf > 3.3 V simply does not conduct. Typical indicator design is 1–2 mA.
If the intent was low power, 2.2–4.7 kΩ (0.3–0.7 mA) is the sensible compromise; if these are meant to be readable on a launch rail in daylight, 1 kΩ. Either way pin actual LED MPNs so Vf is known.

### 10. Two fiducials, one per side, 0.5 mm — and FID1 is inside the USB connector's courtyard
FID1 (B.Cu, 102.08,115.73) sits fully inside J2's courtyard (x 95.76–104.19, y 111.41–124.31) — that is the board's **only DRC error**. FID2 is at the same X/Y on the front. A 0.5 mm copper dot is below the 1 mm many assemblers require, and one fiducial per side gives no rotational reference for a 0.4 mm-pitch QFN-56 plus a 1.27 mm-pitch shielded module.
**Fix:** three fiducials per populated side, 1.0 mm copper / 2.0 mm mask, asymmetric, ≥5 mm from the edge, clear of all courtyards. Same finding as rocket-computer H12 — the same mistake has been made twice.

### 11. There are no reference designators on the silkscreen
All 66 refdes properties are `(hide yes)`. The silkscreen gerbers contain only outlines, pin-1 marks and four board texts (`LoRa V3`, `PWR`, `T`, `R`). Nothing on this board can be identified during assembly inspection, bring-up or rework. On a 22 × 27 mm board you will not fit all of them — but at minimum the ICs, connectors, the LEDs, the crystals and the ferrite bead should be marked, and a pin-1 dot for U28 and U22.

### 12. Hot-plug onto a live 2S pack, with no TVS and no bulk on VSYS
Rocket-computer J5 pin 2 is unswitched VBATT (`Net-(J5-Pin_2)` = {FL2.2, J5.2}), so plugging the daughterboard in always hot-connects a live 6.4–8.4 V pack. On this board VBUS/+BATT go straight through a Schottky into **VSYS, which has no capacitor at all** — the first bulk is C8/C9 (44 µF nominal) *behind* the CM choke. Cable + choke leakage inductance ringing into that cap can approach 2 × V_pack ≈ 16.8 V against the TPS62913's **18 V** abs max; the DCR of the choke and diode probably damp it, but "probably" on a 3-cent part is the wrong trade.
**Fix:** a TVS on VSYS (SMAJ12A / SMBJ13A class, standoff above 8.4 V, clamp well under 18 V), and consider a small bulk cap on VSYS ahead of the choke. This also covers the USB side, where 44 µF of ceramic behind a diode exceeds the USB inrush guidance.

### 13. 921.6 kbaud host UART with no series termination and a ground return that only exists through the CM choke
`HOST_UART_BAUD = 921'600` over an unshielded 1 mm-pitch JST-SH cable running next to a 22 dBm 915 MHz transmitter, with **no series resistors at either driver** and no ESD on LoRa_TX/LoRa_RX.
More interesting: **`VSS` (J6.1) reaches board GND only through FL1's second winding** — the same house pattern as the GNSS daughterboards (verified: their J4.3/J4.4 → FL1 → VIN/GND is identical). That is a deliberate common-mode filter on the power pair, and it is fine for DC. But the UART's return current has no counterpart in the other winding, so it sees the choke as common mode, and the signal pair itself is unfiltered while the power pair is. Worth a conscious decision rather than inheritance.
**Suggest:** 33 Ω series source terminators at each driver (there is room on this board; the host side is a separate change), and confirm the DLW21SN670HQ2L's rated current and DCR against the real load — measured budget is **~90 mA from the rocket (7.4 V) and ~145 mA from the base station (4.6 V)**, against a nominal ~370–400 mA rating for this 0805 class. Comfortable, but it has never been written down.

### 14. Power-good goes nowhere, and the base station cannot power-cycle the modem
`Net-(U3-PG)` = {U3.5, R4.1} with R4 pulled up to `Net-(U3-VO)`. No GPIO sees it. Free telemetry left on the table — and PG is the one signal that would distinguish "modem hung" from "modem rail collapsed".
Related: the rocket computer *can* power-cycle this board (J5 pin 1 is switched by Q10). The base station cannot — J6 pin 1 is hard GND, and V_LORA comes from a TPS61023 **boost** whose EN is tied to VCC; even if EN were driven, a boost passes VIN through its body diode, so the rail never actually falls. On the ground station the only recovery from a wedged modem is unplugging the base station.

### 15. Buck layout details that miss the datasheet
- **C9 is 2.76 mm and C8 is 4.77 mm from VIN** (C7's 2.2 nF is at 1.25 mm and does carry the fastest edges). TI: place the input capacitors "as close as possible" to VIN/PGND. At 2.2 MHz this is the loop that matters most.
- **PSNS (pin 7) — "Connect directly to the system GND plane with a via."** The nearest via is **1.19 mm** away; the pin reaches ground through the F.Cu pour. Put a via at the pad.
- The FB divider's high side taps +3V3 next to R6, i.e. near the converter rather than at the load, so the remote sense is nominal rather than real. With a full In2 plane the IR drop is small — noting it for completeness.

### 16. SMA: the board edge is 0.38 mm short of where the footprint puts the connector face
J8's fab outline draws the connector body to **y = 103.32** and the board-slot region from 103.32 → 104.97, where the pads begin. The actual Edge.Cuts top edge is **y = 103.70**. The board therefore does not bottom out in the connector by 0.38 mm; retention becomes solder-only, and the pin/tab overlap shifts. Pads are 3.5 mm long so there is solderable area either way.
**Fix:** check against the RF Solutions CON-SMA-EDGE-S mechanical drawing and move the edge (or the footprint) to match. On a high-g vehicle with a coax pigtail hanging off it, a connector that is not butted is worth 10 minutes.

### 17. 54 unplugged vias-in-pad, two of them in sub-0.25 mm lands
Board settings are `tenting yes / plugging no / filling no`. Same-net via-in-pad appears in 54 SMD pads. Most are benign (0805 GND pads, the 3×3 EPAD array). Two are not:
- **U28 pad 29 (VDD_SPI): a 0.3 mm-drill / 0.4 mm-pad via inside a 0.22 mm-wide QFN land** on a 0.4 mm-pitch part — the annulus is wider than the land.
- **U3 pad 4 (PGND): a 0.3 mm-drill via inside a 1.0 × 0.2 mm land.**
Both are solder-wicking and bridging hazards. Either move them out of the land or specify IPC-4761 Type VII (filled and capped) on the fab notes.

### 18. Stackup declares no impedance control, and there is one netclass at 0.09 mm
`dielectric_constraints: no`, `copper_finish: None`, and a single `Default` netclass with **track width 0.09 mm and clearance 0.09 mm**. For a 915 MHz board whose antenna feed depends on the F.Cu-to-In1 prepreg being 0.1 mm, the fab needs to be told (a) this is a controlled-impedance layer pair, or at minimum (b) that the F/In1 prepreg thickness is not theirs to substitute. Most 6-layer house stackups will not give you 0.1 mm there by default — and the calculated 46–48 Ω moves with it.
Also: 0.09 mm/0.09 mm is an advanced spec that pushes the price up; only L_MOSI actually uses 0.09 mm (2.9 mm of it). Raising the class minimum to 0.1 mm and re-routing that one net would likely drop a fab tier.

### 19. Smaller items
- **`missing_courtyard` is set to `ignore`** in the DRC config (along with `pth_inside_courtyard`/`npth_inside_courtyard`). Every footprint here does have a courtyard, so nothing is hiding — but this is the exact setting that made the rocket-computer C12 collision invisible. Turn it back on.
- **Silkscreen DRC:** 4 × clipped by board edge (U22 circle, three J2 segments), 2 × silk over copper (Y5 outline over C91's pads), 1 × silk overlap (Y5/C91). Cosmetic, but the Y5 outline over C91's pads can cause solder defects on a 0402.
- **Mounting-hole pads (H1–H4) are floating copper** — 3.8 mm annular pads on both sides with no net, and the plated barrel joins them. Four unterminated 3.8 mm plates on a 915 MHz board, which a metal screw will bond to the surrounding GND pour anyway. Tie them to GND deliberately.
- **No test points anywhere, and U0TXD/U0RXD (pins 49/50) are unconnected.** USB-C is the only console and the only programming path (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`). If the USB path is damaged the board is a brick. Six pads (3V3, GND, TXD0, RXD0, CHIP_PU, GPIO0) cost nothing.
- **Flash size is stated three different ways:** the part is a **W25Q128** (16 MB), the stale `Datasheet` field says W25Q64 (8 MB), and `radio_board/sdkconfig.defaults` sets `CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y` with a comment about not knowing the final part. Settle it on 16 MB now — this is the same class as the OC 512 MB lesson (#492).
- **ESP32-S3 LNA_IN (pin 1) is unconnected** with no matching network. Espressif explicitly permits this ("if RF function is not required, it is recommended not to initialize the RF stack in firmware. In this case, the RF pin can be left floating"), and the RF supply is only a 2 nH 0402 + 100 nF, so it is clearly intentional. Record it as a **firmware constraint**: this board must never init Wi-Fi/BT.
- **In2 (+3V3 plane) is cut by three slots** — VSS (0.4 mm × 15 mm), +BATT (0.4 mm × 8.9 mm), OUT_VDD_SPI (0.2 mm × 6.6 mm). Only LoRa_RX/LoRa_TX on In3 cross them (at the J6 fan-out); at 921 kbaud with In4 GND 0.535 mm below as a continuous second reference, this is negligible. Noted so it is not re-discovered.
- **ERC's 294 violations are almost entirely noise** — 172 off-grid endpoints, 46 lib-symbol mismatches, 15 missing-library warnings. The 7 `power_pin_not_driven` and 25 `pin_not_connected` errors are all explained (deliberate NCs plus flag-less power nets). Worth a cleanup pass so real ERC errors are visible next time, but nothing is hiding in there.

---

## Suggested order of work

1. **Schematic edits:** ~~move C95 to the CHIP_PU node~~ (done); ~~COUT 2x47 uF -> 3x22 uF~~ (done, finding 3a); ~~move U22 VCC to +3V3~~ (done, finding 4); ~~C97 10 µF → 1 µF~~ **(reverted — see finding 4a)**; 0.1 µF at VDD3P3_RTC; PG → a spare GPIO; VSYS TVS; LED resistors; Y6 load caps 18 pF → 12 pF; series terminators on LoRa_TX/RX; rename `LoRa_TX`/`LoRa_RX` to host-perspective-explicit names.
2. **BOM:** ~~capacitors~~ (done, finding 3 — MPN/Mfr/MinVRating on all 27); still to do: FB1 MPN/Mfr fields, the three LEDs, Y5's ESR grade, and the same annotation pass over resistors/inductors/connectors/crystals/ICs.
3. **Layout:** ~~C95 re-route + place C14~~ (done); ~~relocate Y6~~ (retracted, finding 5 — layout is compliant); tighten GND via density along the clock traces toward ~1/mm; pull C8/C9 in toward VIN; via at PSNS; three 1 mm fiducials per side (delete FID1 from under J2); vias out of U28.29 and U3.4; refdes silkscreen; tie H1–H4 to GND; fix the SMA board-edge offset.
4. **Fab package:** declare the F/In1 prepreg (or full controlled impedance) on the fab notes; raise the netclass minimum off 0.09 mm if you want a cheaper tier; re-enable the courtyard DRC checks and re-run; clear the silk violations.
5. **Firmware:** `HOST_UART_TX = 6`, `HOST_UART_RX = 5`; flash size to 16 MB; document "never init RF" and "never enable PSRAM unless VDD_SPI is re-fed".
6. **Bench, once boards exist:** confirm the 40 MHz frequency error and trim C90/C92; scope VIN on hot-plug against the 18 V limit; measure the buck's loop response with the real ferrite bead; verify the SMA's return loss at 915 MHz.
