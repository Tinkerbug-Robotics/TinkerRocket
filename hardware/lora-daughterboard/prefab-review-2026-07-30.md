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

> **2026-08-01 — removed, then restored; recording why so it is not removed again.**
> The inductor was briefly deleted from all three S3 boards on the reasoning that
> it does almost nothing at 40 MHz: 6.0 Ω against the 331 Ω of the 12 pF load cap,
> worth −1.1 ppm, forming a ~297 MHz low-pass with that cap. Those numbers are
> correct, and they are also the whole point — that low-pass is the function
> Espressif is asking for, and on **these** boards it lands where it matters:
>
> | harmonic | frequency | falls in | attenuation |
> |---|---|---|---|
> | 23rd | **920 MHz** | **the 902–928 MHz LoRa band** | **−18.7 dB** |
> | 61st | 2.44 GHz | the BLE band | −36.5 dB |
>
> A LoRa receiver working near −130 dBm does not need a large on-board spur to
> lose sensitivity, and the 23rd harmonic of the S3's own crystal landing in its
> own receive band is exactly the case the guideline is aimed at. The 1 ppm and
> the 0402 saved by deleting it are not worth that. Restored in `e7b81a8`
> (schematics; PCBs re-placed by hand, since the crystal areas had moved).
> Side effect in the right direction: the +0.055 pF of effective load takes the
> base-station's network from +10.9 to **+9.9 ppm**, inside ±10 (issue #704).

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

### 3. BOM had no MPN field — **CLOSED 2026-08-01**
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

**Done 2026-08-01 (everything else).** The same annotation pass ran over the
resistors, inductors, LEDs, connectors, crystals and ICs — and, since the trap
is identical on the sibling boards, over **rocket-computer and base-station**
too. **401 of 401 purchasable parts now carry `MPN` and `Mfr`.** The only
exemptions are three DNP v3.x provisions on the rocket-computer (R74/R75
499 k, R76 0 R): not fitted, never purchased, correctly absent from the BOM.

Each sub-item above, resolved:

- **D4/D5/D6** — D6's value was the placeholder `LED` with no colour, so no
  part could attach; it is now `Green` / `XL-1005UGC`. D4 = `APHHS1005QBC/D`
  (Kingbright), D5 = `NCD0402R1` (NationStar).
- **Y5** — replaced with **`ABS07-32.768KHZ-T`** (Abracon), which pins the
  grade Espressif requires: 12.5 pF load, **ESR 70 kΩ max**. The existing
  `XTAL_SC-32S_3215` footprint is kept — both parts are 2-pad 3.2 × 1.5 mm
  tuning forks, so the land is right and only the footprint's *name* now reads
  for the superseded vendor. Same swap applied to rocket Y1/Y3 and base Y1.
- **FB1** — now carries `MPN`/`Mfr` (782853200 / Würth) like the caps.
- **U22's `Datasheet`** — cleared. The sweep for that trap found **eight**
  across the three boards, two of them naming a genuinely different device
  rather than a stale revision: rocket **U9** read `TPN4R712MD` when the part
  is the `CSD16323Q3` pyro FET, and **R72** read `PMR18EZPFV2L00` when the
  shunt is `CSSH0805FT2L00`. All cleared (commit `59d2c67`).

Two stale legacy `MP` fields also disagreed with both the Value and the BOM
and were corrected — a symbol carrying two part numbers is exactly the
wrong-part-order hazard this finding is about:

- rocket **U19** `TPS259824LNRGER` → `…RGET`. Same die and RGE-24 package; the
  suffix is reel quantity, and `power-eco.md` documents RGET deliberately.
- base **U9** `TPS63021DSJR` → `TPS63020DSJR`. These are **not**
  interchangeable — 63021 is fixed 3.3 V, 63020 is adjustable. U9 pin 3 (FB)
  drives a real divider (R13 1 M / R16 180 k → 0.5 × (1 + 1M/180k) = 3.28 V),
  which only the adjustable part uses. The BOM was right and the `MP` field
  was wrong.

Also normalised: LoRa **R74** `10 K` → `10 k`, whose capital K had been
splitting one part across two BOM lines. ERC clean on all three boards after
the pass (commit `bdcc507`).

### 3a. COUT: 2 × 47 µF 6.3 V → 3 × 22 µF 16 V — **DONE (schematic), PCB pending**
C11/C13 were `CL21A476MQYNNNE` (47 µF, **6.3 V** X5R, 0805) on `Net-(U3-VO)`, which is the TPS62913's COUT — specified as **40 µF min / 47 nom / 80 µF max effective**. A 6.3 V 0805 X5R sitting at 3.3 V is at 52 % of rated voltage, where that class typically loses 55–70 %: two of them land ≈28–42 µF, straddling and probably under the floor.

Changed to **three × `CL21A226MOQNNNE`** (22 µF, **16 V**, 0805) — already a stocked line on this board, so no new BOM row. At 3.3 V it is only at 21 % of rated, so it derates far less: ≈42–53 µF for three, inside the window. This also matches TI's own Table 8-5 recommendation ("3 × 22 µF, 10 V, X7S, 0805").

Applied: C11 and C13 revalued, **C14 added** at schematic (179.07, 150.495) with `#PWR096` GND and a junction on the VO wire at (179.07, 146.685). Netlist verified: `Net-(U3-VO)` = {C11.1, C13.1, C14.1, FB1.1, L8.2, R4.2, U3.3}.

**PCB pending:** run *Update PCB from Schematic*, then place C14. Free F.Cu slot confirmed at **(93.40, 128.8)** — 2.6 × 1.8 mm clear, directly under L8's VO pad and already inside the `Net-(U3-VO)` pour.

**Caveat:** the derating percentages are the usual range for the class, not measured — confirm both parts against Samsung's DC-bias curves. If the 22 µF derates worse than expected, a fourth (≈56–70 µF) still fits inside the 80 µF ceiling.

### 4. VDD_SPI: no 0.1 µF at the pin, a 10 µF where Espressif says not to, both 6 mm away — and the boot flash shares the internal 14 Ω — **RESOLVED 2026-08-01; the residual distance is accepted, with a firmware constraint**
`OUT_VDD_SPI` = {U28.29, U22.8 (VCC), C96 1 µF, C97 10 µF}. C96/C97 are at (92.15, 113.6–114.6) — **1.6 mm from the flash but 6.1–6.3 mm from U28 pin 29** at (86.20,115.76). The rail reaches the chip over a 0.2 mm, 6.6 mm-long trace cut through the In2 plane.
Espressif: "add extra **0.1 µF and 1 µF** decoupling capacitors **close to VDD_SPI**. Please do not add excessively large capacitors." There is no 0.1 µF at all, the 1 µF is 6 mm away, and the 10 µF is the thing they warn against.
Separately, this is the same **R_SPI budget** question as the rocket computer (H7): in 3.3 V mode VDD_SPI is fed internally from VDD3P3_RTC through a **14 Ω** resistor, and on the **S3RH2 the in-package 2 MB quad PSRAM hangs on that same node** (min 2.7 V), as does the external W25Q128 (min 2.7 V, ≤20 mA read, ≤25 mA program/erase). Flash alone: 3.3 − 0.025×14 = 2.95 V — fine. Flash + PSRAM active (~50 mA): 3.3 − 0.70 = **2.60 V, below both minimums**, on the chip's *boot* flash.
**Fix:** move U22 pin 8 off `OUT_VDD_SPI` to +3V3 (this board's +3V3 is a low-noise TPS62913 output, so it is clean), leave VDD_SPI with 0.1 µF + 1 µF **at pin 29**, and drop C97 to 1 µF.
**If instead you keep the shared node:** it only works while firmware never enables PSRAM. `radio_board/sdkconfig.defaults` does not enable SPIRAM today — write that constraint down, because turning it on later silently breaks the rail.

**Resolution 2026-08-01.** The load moved, and that changes the answer more than moving the caps would have. As built:

```
OUT_VDD_SPI = { U28.29, C96 10 µF, C97 100 nF }        <- three items, nothing else
U22.8 (VCC) = +3V3                                      <- flash is off this rail
```

- **The flash is off the rail**, so its 20–25 mA never crosses the 14 Ω. That was the whole 2.60 V problem above, and it is gone.
- **The 0.1 µF that was genuinely missing now exists** (C97). The composition is now 0.1 µF + 10 µF — Espressif asks for 0.1 µF + 1 µF, and finding 4a already argued why the bulk stays 10 µF here.
- **PSRAM is disabled**, so the in-package die draws leakage only: `radio_board/sdkconfig.defaults` contains no `CONFIG_SPIRAM` line at all and IDF leaves it off unless asked.

**What is left on the rail is the SPI pad ring alone** — SPICLK/SPID/SPIQ/SPIWP/SPIHD/SPICS0 (pins 30–35) switching at 80 MHz (`CONFIG_ESPTOOLPY_FLASHFREQ_80M=y`). Each line drives ~10 pF of short trace plus flash input, so I ≈ C·V·f ≈ 2.6 mA for the clock and less for the data lines — **under ~10 mA average**, a 0.14 V DC drop through the 14 Ω, against no external device on the rail that has a Vmin to violate.

**So the 6 mm is accepted, and it is worth being explicit about why rather than just declining the rework.** Six millimetres over a close plane is ~2–3 nH plus vias, call it 3–4 nH:

| | C96/C97 at 6 mm | the only alternative: the internal 14 Ω |
|---|---|---|
| at 80 MHz | ~1.8 Ω | 14 Ω |
| at 300 MHz | ~6.6 Ω | 14 Ω |

Even at 6 mm the local capacitance is **~8× lower impedance than the chip's own internal feed** at the clock frequency. Moving it to 1 mm buys another 3–4×, but the benefit scales with the current being switched and that is now under 10 mA: a 50 mA fast transient into ~2 Ω is ~100 mV on a 3.16 V rail, ~3%, on an internal pad supply. Not worth re-working a finished layout for.

**Firmware constraint — this is the part that must not be lost.** Setting `CONFIG_SPIRAM=y` puts the in-package PSRAM's current back on this node, behind the 14 Ω, and reinstates both the DC drop and the decoupling-distance argument in full. **This board must not enable PSRAM without re-feeding VDD_SPI.** Same class of constraint as "never init RF" in finding 19.

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

### 6. ~~Load caps on Y6 are too high for a 10 pF crystal~~ — **DONE 2026-08-01, 12 pF fitted**
ECS-400-10-37B2-CKY-TR is **CL = 10 pF**, 40 Ω ESR, ±10 ppm. Per-leg stray with the existing 11 mm routing: ~0.95 pF trace (0.086 pF/mm on B.Cu over In4 at 0.1 mm) + ~0.5 pF pad + **2.0 pF pin** (S3 datasheet Table 5-4, C_IN) = **3.45 pF**.

| C1 = C2 | C_L actual | freq error | startup margin vs 18 pF |
|---|---|---|---|
| **18 pF** (present) | 12.45 pF | −33 ppm | 1.00× |
| 15 pF | 10.95 pF | −14 ppm | 1.44× |
| 13 pF | 9.95 pF | +1 ppm | 1.92× |
| **12 pF** ← fit this | 9.45 pF | +9 ppm | 2.25× |

**13 pF is the arithmetic nominal; 12 pF is the practical pick** — +9 ppm is inside the crystal's own ±10 ppm band, 12 pF is a standard E12 value where 13 pF is E24 and thinly stocked, and it gives the best startup margin of the three.

**The real reason to change is margin, not accuracy.** |Rneg| ∝ gm/(ω²·C1·C2), so 18 → 12 pF more than doubles the oscillator's negative resistance against a 40 Ω-ESR part. Frequency accuracy barely matters here: USB tolerates ±2500 ppm, the S3's RF is unused, and LoRa timing comes from the E220's own 32 MHz crystal.

Sensitivity is ~15 ppm per pF of C_L, so a ±1 pF error in the stray estimate is worth about that much — measure and trim on the first article as Espressif instructs.

**Done 2026-08-01.** C90/C92 are `CL05C120JB5NNNC` 12 pF in both schematic and copper. Applied fleet-wide — all four ECS-400 networks across the three boards now use 12 pF, which is what made the base-station's short clock traces visible as the outlier (issue #704, closed: its legs went 3.7/7.2 mm → 8.94/11.83 mm, and the four networks now sit within 2.3 ppm of each other). The first-article measure-and-trim above still stands as the real close-out.

### 7. ~~Host-UART polarity: the firmware placeholder is the reverse of the hardware~~ — **FIXED 2026-08-02**
Cable pin 3 = host RX ↔ daughterboard **GPIO6**; cable pin 4 = host TX ↔ daughterboard **GPIO5**. Both hosts confirm the convention (`out_computer/board_v8.h`: `LORA_UART_TX_PIN = 11 // LoRa_TX (label-perspective)` → J5.4; `base_station/board_v3.h`: TX = 35 → J6.4).
`radio_board/main/config.h` currently has:
```
static constexpr int HOST_UART_TX = 5;   // TODO: TBD from V8 schematic
static constexpr int HOST_UART_RX = 6;   // TODO: TBD from V8 schematic
```
That is backwards. As written the modem drives GPIO5 onto cable pin 4 — **the same wire the host is driving** (two push-pull CMOS outputs in contention) — and listens on GPIO6, which nothing drives. The TODO comments say the values were never resolved; now they can be: **TX = 6, RX = 5**.
**Fixed 2026-08-02** in `radio_board/main/config.h`: `HOST_UART_TX = 6`, `HOST_UART_RX = 5`, and both `// TODO: TBD from V8 schematic` comments removed.

**Correction to the original note.** It said "on the daughterboard the net named `LoRa_TX` is the pin the daughterboard **receives** on (the labels are host-perspective on all three boards)". That is wrong, and the wrong version is more confusing than the truth. Traced end-to-end from the netlists of all three boards:

| | cable pin 3 | cable pin 4 |
|---|---|---|
| rocket-computer | `LoRa_RX`, U15 GPIO10 = **OC's RX** | `LoRa_TX`, U15 GPIO11 = **OC's TX** |
| base-station | `LoRa_RX`, U3 GPIO36 = **BS's RX** | `LoRa_TX`, U3 GPIO35 = **BS's TX** |
| daughterboard | `LoRa_TX`, U28 GPIO6 = **its TX** | `LoRa_RX`, U28 GPIO5 = **its RX** |

Every board names its UART nets from **its own** perspective, not the host's, and the cable is straight-through. The crossover is achieved precisely *because* each end names the same wire differently — cable pin 3 is `LoRa_TX` on the daughterboard and `LoRa_RX` on both hosts. That is a coherent convention, not a mistake; the trap is only that the same string means opposite things depending which board's netlist you are reading, so the pin assignment must come from the schematic and never from a net name seen on the other side. `config.h` now carries that reasoning inline.

---

## Should fix / decide consciously

### 8. ~~VDD3P3_RTC (pin 20) has no local 0.1 µF~~ — **FIXED 2026-08-01**
Espressif asks for a 0.1 µF close to each digital supply pin. Pin 46 (VDD3P3_CPU) had C101 at 1.34 mm ✓; **pin 20's nearest cap on +3V3 was 4.27 mm away** (C99), because the space to the left of U28 is taken by Y5 and the USB resistors.

**Fixed:** C99 moved to **F.Cu at (84.73, 120.57)** — the opposite side, directly over pin 20 at (85.38, 119.78). **1.02 mm lateral**, 1.90 mm through-board, with a +3V3 via 0.06 mm from the pad (effectively via-in-pad) and a GND via at 0.62 mm.

**Opposite-side is fine on this stackup**, and worth recording why, because it would not be on a 2-layer board. The delivery path is not copper from cap to pin — it is the **In2 (+3V3) / In3 (GND) plane pair at 0.1 mm spacing**, which the cap feeds through a short via and which then delivers under the chip with very low spreading inductance. Estimated loop inductance ~1.5–2 nH (via-dominated) against ~2–3 nH before (lateral-spreading-dominated): the ~1 nH the through-board vias cost is less than the 3.25 mm of lateral distance removed.

**Process note.** The first attempt at this move put C99's +3V3 pad and its via into GND copper and squeezed the XTAL_32K_P track, taking the board from 1 DRC error to 15 + 1 unconnected — a +3V3-to-GND short at 0.0000 mm actual clearance, the mirror image of the GND-via-in-+3V3-plane short elsewhere on this board. It was cleaned up in the next pass. The lesson is that in this corridor (between U28's left edge, Y5 and the USB resistors) there is no room to move anything without re-running DRC immediately — an electrical argument for a placement is not a check that the placement fits.

### 9. ~~LED series resistors are 10 kΩ~~ — **CLOSED, refuted on hardware**
*Original claim: 10 kΩ from 3.3 V gives 140 µA (red) and 0–70 µA (blue), and a blue part with Vf > 3.3 V would not conduct at all. **Closed 2026-07-31** — the same 10 kΩ arrangement is in service on existing boards and the LEDs are clearly visible.*

The arithmetic was right; the inference from it was not. I reasoned from a generic 2.7–3.4 V Vf range to "may not light", when modern high-efficiency blue 0402s conduct fine at 3.3 V and are readable at tens of µA indoors. Bench evidence from shipped boards outranks a datasheet-range argument.

Still worth doing at some point: **pin actual LED MPNs** so Vf is a known quantity rather than an assumption — this is the only reason the question was open to argument in the first place.

### 10. Only one fiducial per side, at 0.5 mm — **restated; the courtyard overlap is not the issue**
*Originally written as "FID1 is inside the USB connector's courtyard". That part was over-weighted and is **withdrawn** — see below.*

**The overlap does not matter.** Fiducials are read by the placement machine on a **bare board, before any component is placed**, so FID1 sitting inside J2's courtyard does not obscure it at the moment it is used. It is a geometry complaint (and the board's only remaining DRC error), not a functional one.

**What does matter:**
- **Count.** One fiducial per side gives the machine **translation correction only**. Rotation and scale need at least two per side, placed far apart; three asymmetric is the norm. On a board carrying a 0.4 mm-pitch QFN-56 chip-down (U28) that is the part worth fixing.
- **Size.** 0.5 mm copper is below what many assemblers specify — 1.0 mm copper with a 2.0 mm mask opening is the common requirement. Confirm against whoever is building these before committing.
- **Exposed copper under a metal shell.** FID1 is an unnetted 0.5 mm dot with a 1 mm mask opening sitting under the USB-C body. Floating copper under a grounded shell is not a fault, but it is a solder-bridge and debris trap for no benefit.

**Fix:** two or three fiducials per populated side, sized to the assembler's spec, placed asymmetrically and well apart. Repositioning FID1 falls out of that, and the DRC error clears as a side effect. Same underlying gap as rocket-computer H12.

**Count DONE 2026-08-01 — size still open.** There are now **six fiducials, three per side, asymmetric**: F.Cu FID2 (102.09, 115.77), FID3 (86.97, 107.89), FID4 (90.64, 123.85); B.Cu FID1 (102.08, 115.73), FID5 (86.99, 107.91), FID6 (93.18, 130.17). That gives the placement machine rotation and scale, not just translation, which was the part that mattered for a 0.4 mm-pitch QFN-56.

**Still 0.5 mm copper** — confirm against whoever is building these before committing, since 1.0 mm copper / 2.0 mm mask is the common requirement. The two remaining board DRC errors are FID1/J2 and FID6/J6 courtyard overlaps, which per the reasoning above are geometry complaints rather than functional ones.

### 11. There are no reference designators on the silkscreen
All 66 refdes properties are `(hide yes)`. The silkscreen gerbers contain only outlines, pin-1 marks and four board texts (`LoRa V3`, `PWR`, `T`, `R`). Nothing on this board can be identified during assembly inspection, bring-up or rework. On a 22 × 27 mm board you will not fit all of them — but at minimum the ICs, connectors, the LEDs, the crystals and the ferrite bead should be marked, and a pin-1 dot for U28 and U22.

### 12. ~~Hot-plug onto a live 2S pack, with no TVS and no bulk on VSYS~~ — **CLOSED 2026-08-02, SMF10A fitted here and on both GNSS boards**
Rocket-computer J5 pin 2 is unswitched VBATT (`Net-(J5-Pin_2)` = {FL2.2, J5.2}), so plugging the daughterboard in always hot-connects a live 6.4–8.4 V pack. On this board VBUS/+BATT go straight through a Schottky into **VSYS, which has no capacitor at all** — the first bulk is C8/C9 (44 µF nominal) *behind* the CM choke. Cable + choke leakage inductance ringing into that cap can approach 2 × V_pack ≈ 16.8 V against the TPS62913's **18 V** abs max; the DCR of the choke and diode probably damp it, but "probably" on a 3-cent part is the wrong trade.
**Fix:** a TVS on VSYS (SMAJ12A / SMBJ13A class, standoff above 8.4 V, clamp well under 18 V), and consider a small bulk cap on VSYS ahead of the choke. This also covers the USB side, where 44 µF of ceramic behind a diode exceeds the USB inrush guidance.

### 13. 921.6 kbaud host UART with no series termination — **series R DONE 2026-08-01 (1 kΩ, not 33 Ω — see below)**; the CM-choke ground return stands
`HOST_UART_BAUD = 921'600` over an unshielded 1 mm-pitch JST-SH cable running next to a 22 dBm 915 MHz transmitter, with **no series resistors at either driver** and no ESD on LoRa_TX/LoRa_RX.
More interesting: **`VSS` (J6.1) reaches board GND only through FL1's second winding** — the same house pattern as the GNSS daughterboards (verified: their J4.3/J4.4 → FL1 → VIN/GND is identical). That is a deliberate common-mode filter on the power pair, and it is fine for DC. But the UART's return current has no counterpart in the other winding, so it sees the choke as common mode, and the signal pair itself is unfiltered while the power pair is. Worth a conscious decision rather than inheritance.
**Suggest:** 33 Ω series source terminators at each driver (there is room on this board; the host side is a separate change), and confirm the DLW21SN670HQ2L's rated current and DCR against the real load — measured budget is **~90 mA from the rocket (7.4 V) and ~145 mA from the base station (4.6 V)**, against a nominal ~370–400 mA rating for this 0805 class. Comfortable, but it has never been written down.

**Done 2026-08-01 — but with 1 kΩ, not 33 Ω, and the difference is worth recording.** R77/R78 now sit in series on `LoRa_RX`/`LoRa_TX` between the chip and J6 pins 3/4. 33 Ω would have been a *source terminator*, sized to match the driver's output impedance to the cable so reflections are absorbed at the source. 1 kΩ is not that — it is slew-rate and fault-current limiting.

That is fine here, and arguably the better fit for the actual risk. At 921.6 kbaud the bit period is 1085 ns and the cable is well under a metre, so the line is electrically short: reflections settle in a few ns, orders of magnitude inside a bit, and were never the failure mode. What the finding actually cared about was radiated coupling next to a 22 dBm 915 MHz transmitter, and a 1 kΩ series element attacks that directly by limiting edge rate and fault current.

The cost is rise time, which stays comfortable:

| cable + input C | RC | 10–90 % rise | fraction of a bit |
|---|---|---|---|
| 30 pF | 30 ns | 66 ns | 6 % |
| 60 pF | 60 ns | 132 ns | 12 % |
| 100 pF | 100 ns | 220 ns | 20 % |
| 150 pF | 150 ns | 330 ns | 30 % |

A UART samples mid-bit, so anything under ~40 % is uncontentious. **The CM-choke ground return (VSS reaching GND only through FL1's second winding) is unchanged and remains an open, conscious inheritance.**

### 14. Power-good goes nowhere, and the base station cannot power-cycle the modem
`Net-(U3-PG)` = {U3.5, R4.1} with R4 pulled up to `Net-(U3-VO)`. No GPIO sees it. Free telemetry left on the table — and PG is the one signal that would distinguish "modem hung" from "modem rail collapsed".
Related: the rocket computer *can* power-cycle this board (J5 pin 1 is switched by Q10). The base station cannot — J6 pin 1 is hard GND, and V_LORA comes from a TPS61023 **boost** whose EN is tied to VCC; even if EN were driven, a boost passes VIN through its body diode, so the rail never actually falls. On the ground station the only recovery from a wedged modem is unplugging the base station.

### 15. Buck layout details that miss the datasheet
- **C9 is 2.76 mm and C8 is 4.77 mm from VIN** (C7's 2.2 nF is at 1.25 mm and does carry the fastest edges). TI: place the input capacitors "as close as possible" to VIN/PGND. At 2.2 MHz this is the loop that matters most.
- ~~**PSNS (pin 7) — "Connect directly to the system GND plane with a via."** The nearest via was **1.19 mm** away; the pin reached ground through the F.Cu pour.~~ **DONE 2026-08-01 — a GND via now sits at the pad (0.00 mm).**
- The FB divider's high side taps +3V3 next to R6, i.e. near the converter rather than at the load, so the remote sense is nominal rather than real. With a full In2 plane the IR drop is small — noting it for completeness.

### 16. SMA: the board edge is 0.38 mm short of where the footprint puts the connector face — **OPEN, needs a physical check**
J8's fab outline draws the connector body to **y = 103.32** and the board-slot region from 103.32 → 104.97, where the pads begin. The actual Edge.Cuts top edge is **y = 103.70**. The board therefore does not bottom out in the connector by 0.38 mm; retention becomes solder-only, and the pin/tab overlap shifts. Pads are 3.5 mm long so there is solderable area either way.
**Fix:** check against the RF Solutions CON-SMA-EDGE-S mechanical drawing and move the edge (or the footprint) to match. On a high-g vehicle with a coax pigtail hanging off it, a connector that is not butted is worth 10 minutes.

**Status 2026-08-02 — still open, and it needs a part in hand rather than more analysis.** J8 sits at (92.4, 106.7225) rot -90 against a board that spans x 81.86–103.86, y 103.70–131.20, so the connector launches off the y-minimum edge. The 0.38 mm is the gap between where `Edge.Cuts` puts the board edge and where the `CON-SMA-EDGE-S` footprint expects the connector's inner face to land.

Deliberately **not** fixed by moving the edge: this is an edge-launch SMA whose body straddles the board edge, so the number that matters is how the real connector's slot seats on the real board thickness (1.6 mm), and that is a caliper measurement, not a CAD one. Worth doing before fab because it is mechanical and unrecoverable — if the slot does not seat, the centre pin does not land on the feed. Also note the part itself is a supply risk carried elsewhere in this review: CON-SMA-EDGE-S was 0 stock everywhere with a 20-week lead, so whichever connector is actually bought is the one to measure.

### 17. ~~54 unplugged vias-in-pad, two of them in sub-0.25 mm lands~~ — **CLOSED 2026-08-01, both ways**
Board settings were `tenting yes / plugging no / filling no`. Same-net via-in-pad appeared in 54 SMD pads. Most were benign (0805 GND pads, the 3×3 EPAD array). Two were not:
- **U28 pad 29 (VDD_SPI): a 0.3 mm-drill / 0.4 mm-pad via inside a 0.22 mm-wide QFN land** on a 0.4 mm-pitch part — the annulus was wider than the land.
- **U3 pad 4 (PGND): a 0.3 mm-drill via inside a 1.0 × 0.2 mm land.**

Both were solder-wicking and bridging hazards. The review offered two fixes;
both were taken.

**The two hazards are gone** — the user removed those specific vias from the
lands, so the sub-0.25 mm cases no longer exist.

**And the remaining benign via-in-pad is now declared.** The board file said
`(filling no) (capping no)`, so nothing in the exported data told the fab what
the layout actually needs. It now declares **IPC-4761 Type VII (filled and
capped)**, which is the process via-in-pad requires. Applied to
lora-daughterboard and rocket-computer — both 6-layer, both with via-in-pad.
**base-station is deliberately left at `no`**: it is 4-layer and does not get
the treatment.

This matters beyond solder defects: it is a *cost and process* declaration. A
fab quoting a 6-layer board with filled-and-capped vias prices differently
than one quoting plain tented vias, and an undeclared Type VII requirement is
the kind of thing that surfaces as a post-quote change order.

### 18. ~~Stackup declares no impedance control, and there is one netclass at 0.09 mm~~ — **BOTH DONE 2026-08-02**
`dielectric_constraints: no`, `copper_finish: None`, and a single `Default` netclass with **track width 0.09 mm and clearance 0.09 mm**. For a 915 MHz board whose antenna feed depends on the F.Cu-to-In1 prepreg being 0.1 mm, the fab needs to be told (a) this is a controlled-impedance layer pair, or at minimum (b) that the F/In1 prepreg thickness is not theirs to substitute. Most 6-layer house stackups will not give you 0.1 mm there by default — and the calculated 46–48 Ω moves with it.
Also: 0.09 mm/0.09 mm is an advanced spec that pushes the price up; only L_MOSI actually uses 0.09 mm (2.9 mm of it). Raising the class minimum to 0.1 mm and re-routing that one net would likely drop a fab tier.

**Both done 2026-08-02, and verified in the fab-facing output rather than just the board file.**

**18a — impedance control declared.** `dielectric_constraints` set to `yes`. The effect is bigger than the setting name suggests: exporting gerbers before and after, the F.Cu/In1.Cu dielectric entry in the `.gbrjob` changed from

```
{Type: Dielectric, Thickness: 0.1, Material: FR4}                              <- before
{Type: Dielectric, Thickness: 0.1, Material: FR4, DielectricConstant: 4.5,     <- after
 LossTangent: 0.02}
```

The fab was previously receiving a thickness with **no dielectric constant**, so nothing in the package pinned the impedance of the layer pair the 46-48 Ohm antenna feed depends on. It now ships the full spec. (A human-readable fab note is still worth adding — most houses quote from a fab drawing, not by parsing the job file.)

**18b — netclass minimum raised to 0.1 mm.** Cheaper than the review estimated: only **6.84 mm of track (1.9%)** sat at 0.09 mm, across two nets — `L_MOSI` (2.94 mm) and `Net-(C90-Pad1)` (3.90 mm, the crystal leg, which post-dates the review). Those 7 segments were widened to 0.1 mm and the netclass, `min_track_width` and `min_clearance` all moved 0.09 -> 0.1. `MinLineWidth` in the `.gbrjob` follows, 0.09 -> 0.1, which is the number a fab quotes against.

**Also re-enabled here:** `missing_courtyard`, `pth_inside_courtyard` and `npth_inside_courtyard` went from `ignore` back to `error` (finding 19). Those were off during layout, which is exactly when they are worth having on -- it is the setting that hid the rocket-computer's C12 collision.

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

1. **Schematic edits:** ~~move C95 to the CHIP_PU node~~ (done); ~~COUT 2x47 uF -> 3x22 uF~~ (done, finding 3a); ~~move U22 VCC to +3V3~~ (done, finding 4); ~~C97 10 µF → 1 µF~~ **(reverted — see finding 4a)**; ~~0.1 µF at VDD3P3_RTC~~ (done, finding 8); PG → a spare GPIO; VSYS TVS; ~~LED resistors~~ (closed, finding 9); Y6 load caps 18 pF → 12 pF; series terminators on LoRa_TX/RX; rename `LoRa_TX`/`LoRa_RX` to host-perspective-explicit names.
2. **BOM:** ~~capacitors~~ ~~FB1 MPN/Mfr~~ ~~the three LEDs~~ ~~Y5's ESR grade~~ ~~the annotation pass over resistors/inductors/connectors/crystals/ICs~~ — **all done, finding 3 closed.** 401/401 purchasable parts across all three ESP32 boards carry MPN/Mfr; eight stale part numbers cleared out of `Datasheet` fields; two contradicting legacy `MP` fields corrected.
3. **Layout:** ~~C95 re-route + place C14~~ (done); ~~relocate Y6~~ (retracted, finding 5 — layout is compliant); tighten GND via density along the clock traces toward ~1/mm; pull C8/C9 in toward VIN; via at PSNS; three 1 mm fiducials per side (delete FID1 from under J2); ~~vias out of U28.29 and U3.4~~ (done, finding 17); refdes silkscreen; ~~tie H1–H4 to GND~~ (done); fix the SMA board-edge offset.
4. **Fab package:** ~~declare filled-and-capped vias~~ (done, finding 17 — the board file now carries IPC-4761 Type VII); declare the F/In1 prepreg (or full controlled impedance) on the fab notes; raise the netclass minimum off 0.09 mm if you want a cheaper tier; re-enable the courtyard DRC checks and re-run; clear the silk violations.
5. **Firmware** (owner: user): `HOST_UART_TX = 6`, `HOST_UART_RX = 5`; flash size to 16 MB; document "never init RF" and "never enable PSRAM unless VDD_SPI is re-fed".
6. **Bench, once boards exist:** confirm the 40 MHz frequency error and trim C90/C92; scope VIN on hot-plug against the 18 V limit; measure the buck's loop response with the real ferrite bead; verify the SMA's return loss at 915 MHz.
