# TinkerRocket Base Station — Power-Switching Architecture Review

**Date:** 2026-08-02
**Source:** `TinkerRocket-Hardware/hardware/base-station/` (branch `main`)
**Scope:** the power-switching architecture only — is the switch wired correctly, and does it actually turn things off. Schematic-level; PCB explicitly out of scope at the user's request.
**Method:** `kicad-cli` XML netlist export as connectivity ground truth (never eyeball-reading the schematic), compared against the archived `TinkerRocket Base Station V5` project **and** its fabbed gerbers; datasheets pulled for TPS22918, TPS61023, TPS63020, BQ21040, DW01A, MP2672A, PB400EEQR1BLK.

---

## Headline: the V5 defect is fixed

**V5 (the version that shipped):** the LM5157 boost that made `V_LORA` had its input *and* enable on the raw battery rail `VCC`, **upstream** of the TPS22810 load switch. Switching off killed the ESP32 but left the boost regulating 5 V into the LoRa daughterboard indefinitely. That is the "the power switch didn't turn off the LoRa" symptom.

**Current revision:** battery → `S1` (PB400 latching DPST) → `U22` TPS22918 load switch → `V_SWITCH`, and **both** converters now hang off `V_SWITCH`:

```
BT2 18650 ──┬── U5  BQ21040   (USB charger, always live)
            ├── U14 DW01A + 3x FS8205A  (protection, always live)
            ├── R44 1M/1M sense divider (always live)
            └── S1 ──> U22 TPS22918 ──> V_SWITCH ──┬── U9 TPS63020 ──> +3V3  (ESP32-S3)
                                                    └── U2 TPS61023 ──> V_LORA ──> J6 (LoRa board)
VBUS(USB) ──┬── U5  BQ21040  ──> VCC        (internal 18650)
            └── U12 MP2672A  ──> J4          (external 2S pack)
```

Switch off removes power from the LoRa daughterboard along with everything else. Both chargers sit upstream of the switch and remain fully functional with the switch off, indicator LEDs included. The optional requirement is met, not merely approximated.

**Measured standby with S1 open (~6 µA typical, ~12 µA worst case):**

| Path | Draw | Notes |
|---|---|---|
| U14 DW01A via R55 | ~3 µA typ / 6 µA max | by design, unswitchable |
| R44 + R46 (1 M + 1 M) | 1.9–2.1 µA | fixed sense divider across the cell |
| U5 BQ21040 OUT reverse leakage | ~1–2 µA | VIN absent |
| U22 TPS22918 off-state | ≤1 µA | 0.4–0.5 µA typ at 25 °C |
| Injection into unpowered GPIO1 via `Volt_Read` | ≤~2 µA | see open item 2 |

Orders of magnitude below 18650 self-discharge. (Add ~40 µA through `R7` while S1 is latched **on** — on-state only.)

---

## Fixed during this review

### 1. `U9` TPS63020 VINA + EN were floating — the board could never have booted — **FIXED (schematic), PCB PENDING**
`Net-(U9-EN)` contained exactly three nodes: `U9.1` (VINA), `U9.12` (EN), and `C29` 100 nF to GND. It never reached `V_SWITCH`. VINA is the control-stage supply (1.5 V UVLO) and EN needs ≥1.2 V, so as drawn the buck-boost is permanently disabled and `+3V3` never comes up.

This defect was **present in the original import commit, in the archived V5 project, and in the fabbed V5 gerbers** — verified at gerber level, not inferred. Existing V5 boards only boot because the TPS63021's VINA rail self-biases parasitically from the powered VIN pins; that is out of spec and lot/temperature dependent. TI's current datasheet revision (Rev G, "removed the connection from VINA to VIN") draws Figure 7/28 with the VINA/EN node visually unconnected, which is how the error propagated.

Now wired to `V_SWITCH`. **The PCB has not been updated.**

### 2. `R53` 210 k TS fail-safe added — **FIXED (schematic), PCB PENDING**
A floating TS pin on the BQ21040 does **not** suspend charging — it enters TTDM (datasheet §8.4.6), which continues charging the cell with **NTC monitoring, charge termination and the 10-hour safety timer all disabled**. `TH1` is a detachable 2-pin header with the thermistor taped to the cell holder, so an unplugged connector or failed tape silently removes every charge safety.

210 k from TS to GND parks the pin in the cold-fault window (charging suspended) when the NTC is absent.

**Why 210 k and not TI's 237 k.** The guaranteed band is set by two bounds: reads cold-fault alone requires R > V_TS-0C(max) / I_NTC(min) = 1.28 V / 48 µA ≈ 27 k; never enters TTDM requires R < V_TTDM(min) / I_FLDBK(max) = 1.55 V / 6.5 µA ≈ 238 k. TI's 237 k computes to 1.556 V against a 1.550 V TTDM floor — *at* the limit. 210 k keeps ~11 % margin and is an already-stocked fleet MPN. With the NTC plugged in, the parallel 210 k moves the cold cutoff from ~+3 °C to ~0 °C and the hot cutoff by <1 °C.

**`300 k` — which this board also stocks — is NOT a valid substitute:** 300 k × 6.5 µA = 1.95 V, well inside TTDM.

### 3. `R45`/`R47` MID balance resistors 0805 → 1206 0.75 W — **FIXED (schematic), PCB PENDING**
`RC0805FR-0760R4L` (Yageo, 0.125 W) → **`CRCW120660R4FKEAHP`** (Vishay, 60.4 Ω 1 % **0.75 W** 1206).

The pair is 2 × 60.4 Ω in parallel = 30.2 Ω in the MP2672A's MID balance path. With `R_ON_BHS` = 2.1 Ω (datasheet p.9) and the high cell at 4.2 V, balance current is 4.2 / 32.3 = **130 mA**; each resistor carries 65 mA and dissipates **0.255 W**. Balancing runs **249.8 ms of every 250 ms cycle** (Figure 13, p.24) — continuous, not a pulse.

So the 0805 parts were at **204 %** of rating, and a *standard* 0.25 W 1206 would still be at **102 %** — the package change alone does not fix it, the wattage does. At 0.75 W they run 34 % loaded. Balance current deliberately unchanged; MPS characterizes the part at ~17 Ω, so 30.2 Ω is already the conservative side.

### 4. Metadata lockstep — three parts whose values moved without their MPNs
Caught by the exact MPN-keyed BOM audit, not by ERC/DRC:

| Ref | Value | Was carrying | Now |
|---|---|---|---|
| `R43` | 10 k (RISET) | `RC0402FR-076K04L` (6.04 k) | `RC0402FR-0710KL` |
| `C42` | 10 µF | `CL05A105KO5NNNC` (1 µF) | `CL05A106MQ5NUNC` |
| `R53` | 210 k | `RC0402FR-07675RL` (675 Ω) | `RC0402FR-07210KL` |

`R53`'s was the dangerous one: copy-pasted from `R52`, it would have been **built as 675 Ω**. At 675 Ω the TS pin sits at ~34 mV, below the 88 mV `V_TS-EN-10K` threshold — that is the "pull TS low" state, which **disables the charger outright**. The board would simply never have charged, and the schematic would have looked correct.

### 5. `R43` RISET 6.04 k → 10 k (user decision, recorded here)
MP2672A `ICC = 12 kΩ / RISET` (Eq. 1, p.21), so 10 k → **1.2 A** fast charge, down from ~2 A. Input draw falls from ~4 A to ~2.2 A from 5 V VBUS. The MP2672A has **no input current limit** — only the 4.49 V VLIM foldback — so the old setting would drag a normal USB port down and hold it there. Also relieves `L6` (SRP4020, 7 A Isat vs the datasheet's >8 A ask). Note 1.2 A is the *autonomous* default; the ESP32 can trim `ICC` down over I²C for small packs (disable the 40 s register watchdog first or the write reverts).

---

## Verified correct — do not re-review

- **TPS22918 (`U22`)**: VIN/ON abs max fine at 4.2 V; V_IH 1.0 V min so a 2.8 V cell still registers ON; QOD tied directly to VOUT is one of the three datasheet-sanctioned configurations and the ~44 µF load is well under the 200 µF internal-RPD limit; `C88` 2.2 nF → 3.4–4.6 ms slew, inrush ~80 mA worst case against a 2 A rating; off-state leakage 0.4–0.5 µA typ.
- **TPS61023 (`U2`)**: `R11`/`R12` = 732 k/100 k → 4.95 V (PWM) / 5.00 V (PFM) — TI's own Figure 8-1 values; EN tied to VIN is a legal always-on config; **true input-to-output disconnect in shutdown** (0.1 µA I_SD), so there is no VIN→VOUT body-diode path.
- **TPS63020 (`U9`)**: correct *adjustable* MPN (63021 is fixed-3.3 V and would conflict with the divider); `R13`/`R16` = 1 M/180 k → 3.28 V; PS/SYNC = GND selects power-save; PG floating is permitted; `C29` 100 nF on VINA is compliant and **must not exceed 0.22 µF** (§8.2.2.5) — do not "improve" it later.
- **BQ21040 (`U5`)**: pinout confirmed (TS=1, OUT=2, /CHG=3, ISET=4, GND=5, IN=6); `R52` 675 Ω → ~0.8 A; `V_OCP` 4.30 V ±50 mV on the DW01A sits cleanly above the charger's 4.2 V ±1 % regulation, so the protector is a true backstop and will not fight the charger.
- **DW01A + 3 × FS8205A**: topology, gate orientation and low-side placement match the standard protected-pack circuit; `R55` 100 Ω is the datasheet value; ~150 mV OC threshold trips around 7–8 A.
- **MP2672A (`U12`)**: `-0000` charges autonomously with no I²C; NTC 100 k/100 k divider is a legitimate temperature-qualification defeat and sits inside the valid window; VLIM 5.49 k/2 k → 4.49 V input-voltage regulation (droop foldback, not brownout); leaving SYS unloaded is sanctioned; I²C pins tolerate 3.3 V pull-ups while the chip is unpowered.
- **`S1` PB400EEQR1BLK** is a **latching DPST**, contacts bridge row-wise ({1,3} vs {2,4} matches the net split), and it carries only the ON pin's ~42 µA — not load current. Support-peg NPTHs are present in the footprint.
- **USB-C**: 5.11 k CC pulldowns present and correct for a sink.
- **`U14` DW01A `TD` (pin 4) is correctly LEFT FLOATING** — see below.

---

## `DW01A` TD pin — settled: leave it floating (a prior flag was wrong)

An earlier pass in this review flagged floating TD as a defect, claiming "the typical application circuit ties it to GND". **That is false.** Seven vendor datasheets were checked and **zero connect TD to anything**:

- Fortune Semiconductor DW01A Rev.1.2 §8, DW01-P V1.0, H&M Semi, PJSEMI (LCSC C686633) and PUOLOP (C351410) all name pin 4 *"Test pin for reduce delay time"* and draw it unwired.
- Slkor Rev.2 and HMSemicon DW01A-E rename pin 4 to **"NC / 不接 / Not connected"** outright.

Fortune's and PJSEMI's block diagrams show TD with an **internal pull-down resistor to VSS** feeding the oscillator control block, so the test mode is **active-HIGH** and floating is precisely the state in which the specified TOC/TOD/TOI delays are guaranteed. No vendor warns against leaving it open.

Grounding would be electrically harmless but is **worse here for assembly reasons**: pin 4 is adjacent to **pin 5 = VCC**, so a GND net at pin 4 turns a 4–5 solder bridge into a dead protector and an unprotected cell, whereas with pin 4 floating that same bridge is a no-op.

> **Pin-numbering trap on this part:** GND is **pin 6**, not pin 3 — **pin 3 is OC, the charge-FET gate** (order: 1 OD, 2 CS/VM, 3 OC, 4 TD, 5 VCC, 6 GND). Tying TD to "pin 3" believing it were ground would clamp the charge gate and permanently disable charging. This schematic is correct (`U14.6` → `B-`).

---

## Open items

**1. `C42` is a 10 µF 0402 rated 6.3 V — it will deliver ~2–3 µF at 4.2 V bias.** The intent was 10–22 µF of real local capacitance at `U22`'s VIN (the datasheet wants CIN ≥ CL and warns that CL > CIN can pull VOUT above VIN through the body diode on supply removal — e.g. a DW01A trip; currently CL ≥ 44 µF against CIN 1 µF, ~44:1 the wrong way). DC-bias derating eats most of the 0402. **Recommend the 22 µF 0805 already on this board** (`CL21A226MOQNNNE`, 16 V, 10 placements) → ~14 µF effective. Zero new BOM lines; costs one 0805 of board area.

**2. `R44`'s divider top could move from `VCC` to `V_SWITCH`.** Kills the last ~2 µA of standby drain *and* stops holding ~VCC/2 on the unpowered ESP32's non-failsafe GPIO1 pad in every off state. Reading is unchanged when on (minus the switch's few mV); firmware reads nothing when off anyway.

**3. Add 1 k series resistors in `LoRa_RX`/`LoRa_TX` at `J6`.** Same rationale as the V9 review's H6 fix on the camera UART: on the bench the daughterboard runs from its own USB while the base station can be switched off, driving UART levels into unpowered ESP32 pads. **This is also a prerequisite for item 5.**

**4. Delete the orphan `M_FLASH_CS` / `M_MISO` / `M_MOSI` / `M_SCK` labels** on GPIO4–7 — single-node nets left over from V5's S25FL256, which the W25Q128-on-VDD_SPI replaced.

**5. Optional: put `U2`'s EN under firmware control** so the base station can power-cycle a wedged radio board (`LORA_ACT_PIN` is currently `-1` in `board_v3.h` with a TODO pointing at exactly this; recovery hammer for #412). Designed and verified but **not implemented**:

```
GPIO21 ──[ R_S 1 k ]──┬── U2 pin 2 (EN)
                      └──[ R_PU 22 k ]── +3V3
   (delete the existing EN → V_SWITCH tie)
```

- **22 k, not 100 k.** The ESP32-S3's internal weak pull-down is 45 kΩ typ; with 100 k, if that WPD is ever active EN divides to **1.04 V**, landing between V_EN_L(max) 0.45 V and V_EN_H 1.2 V — the indeterminate band. Breakpoint is ~80 k. 22 k gives 2.23 V in that case and 0.143 V driven low (vs 0.35 V worst-case V_EN_L). **Do not use 10 k** — driven low it yields 0.30 V, only 50 mV of margin.
- **Pull up to `+3V3`, never to `V_SWITCH`.** 4.2 V exceeds the S3's 3.6 V pad limit and back-feeds the 3.3 V rail through the pad ESD clamp during the entire high-Z reset window. EN is rated ±7 V referenced to **GND, not VIN**, so 3.3 V EN with VIN at 2.8 V is explicitly fine, and sequencing improves (+3V3 derives from `V_SWITCH`, so EN can only rise after VIN).
- **`GPIO21` specifically.** Of the free pins it is the only one that is Priority-2 unrestricted, **absent from the power-up glitch table** (GPIO1–14, 17, 18 all drive a 60 µs low output glitch at cold start), blank in both At Reset and After Reset so nothing fights the pull-up, and RTC-capable. It is also the physically closest free pin to `U2` (14.2 mm). `GPIO17` is `LORA_RXEN_PIN` on BS V2 — avoid for cross-revision consistency.
- **Default-ON is deliberate**: for a ground station whose only job is the link, fail-safe is fail-ON. (The OC V8 correctly does the opposite with a 10 k pull-down — a flight computer should not power a radio it has not configured.)

> **The catch — EN alone does NOT de-power the daughterboard.** The base station's UART TX idles high at 3.3 V and reaches the daughterboard through `J6.4 → R78 (1 k) → U28 GPIO6`, injecting up to ~2.5 mA into that board's ESD clamp and onto its `+3V3`. That node sits **downstream of `CR3`** (CUS10S30 diode-OR), so this board can inject into it but cannot discharge it. The daughterboard parks around 1.5–2.6 V: the E220 module (1.8–3.7 V) keeps running while its host ESP32-S3 (3.0 V min) and flash (2.7 V min) sit below spec — exactly the wedged state the power-cycle exists to break. Firmware must therefore put `GPIO35`/`GPIO36` into `GPIO_MODE_INPUT` with **both** internal pulls explicitly disabled before dropping EN, and restore them after.
> **Bench trap:** `V_LORA` reads a convincing 0 V while this is happening, because `CR3` hides the daughterboard's still-live rails. Validate on the daughterboard's own `+3V3`, or on a fresh `BOOT` identity frame — never on `V_LORA`.
> The off-time cannot be derived (it depends on the daughterboard's unspecified leakage behind `CR3`) and **must be measured on the bench**.
> Note `J6` is only 4 pins, so there is no spare conductor for a reset line to the daughterboard's `CHIP_PU`, which would be the genuinely robust recovery mechanism. If a connector change is on the table, that beats the EN approach outright.

**6. Smaller items:** `BT2`'s MYOUNG holder is not mechanically polarized — a reversed cell puts −4.2 V straight on `VCC` with no series element; silk the polarity prominently. A second 22 µF on `V_LORA` (a lone 22 µF derates to ~10 µF effective at 5 V; TI's 5 V reference uses 2 × 22 µF). Pin the DW01A vendor at ordering — Slkor's 4.22 V low corner sits under the BQ21040's 4.23 V regulation max. `D9` only lights on the *first* charge cycle after USB plug-in (§8.3.6) and is dim below ~3.3 V cell. Don't *store* a 2S pack on `J4` — the always-connected dividers drain the bottom cell ~2× the top, ~15 mAh/month.

---

## Status

**All five fixes above are schematic-only. The PCB has not been updated for any of them.** Run *Update PCB from Schematic*, then place `R53` (0402), re-place `R45`/`R47` as 1206, and re-route `U9`'s VINA/EN to `V_SWITCH`.

BOM (`TinkerRocket_CrossBoard_BOM.xlsx`) is updated and reconciles exactly against the netlist: **109 fitted parts, 55 distinct MPNs, 0 quantity mismatches, 0 orphans**, base component cost $18.89/bd. The 6.04 k line is retired the sheet's way (quantities cleared, row kept).
