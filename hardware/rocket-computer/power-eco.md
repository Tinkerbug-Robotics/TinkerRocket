# TinkerRocket V9 — Power Architecture ECO (change draft)

**Date:** July 12, 2026
**Assumes:** V9 fully implements the new power architecture. Battery = 2S LiPo (6.4–8.4 V). Pyro energy store scoped to firing only. E-match output topology unchanged (high-side FET switching, deliberate positive control); with the move to a high-side main disconnect, battery negative = GND and the old switched `−BATT` return is eliminated (e-match return now goes to GND).

This is a change list keyed to existing designators, ready to drive schematic edits. Each item: what, where, why, and suggested values/parts.

---

## 0. As-built power domains (confirmed from netlist)

| Domain | Path | Loads | Notes |
|--------|------|-------|-------|
| **Always-on logic** | Pack → **U2** (TPS2121 mux, +USB) → **VCC** → **L9** → **U10** (TPS62152 buck) → **+3V3** | **S3 (U16)**, its flash (U13, `OUT_VDD_SPI`), USB mux **U3** | Must never drop. This is the protected domain. |
| **Switched compute** | +3V3 → **U4** (TPS22918) → **VPP** | **P4 (U18)**, P4 buck **U19**, sensors **U5/U6/U23**, memory **U11/U12**, pyro continuity sense, buzzer | Gated by U4 = the "S3 powers P4 at the pad" interlock. Downstream of +3V3, so protected by the same hold-up. |
| **Raw-VBATT aggressors** | Pack → **Q9** → J5 (servos, C10 330 µF); Pack → **Q3** → J6 (camera, C80 100 µF) | Servos, camera | Big transients. Tap raw VBATT (mux IN1). Already firmware-switchable (`SERVO_ACT`, `CAM_ACT`). |
| **Switched peripherals** | **Q8** (`GPS_ACT`, ferrite FL1) → J3 (GNSS); LoRa daughterboard → J9 (`LoRa_ACT`/R73) | GNSS + LoRa daughterboards | Already independently switchable + filtered. |
| **Pyro** | VBATT → R69 (150 Ω) → cap **C1** (10 mF, EKYC160ELL103MM25S) → **U9** arm → `PYRO_POS` → **U7/U24/U25/U26** fire; e-match return → GND | E-matches (J4) | Isolated; fires from cap; downstream of the eFuse now (Change 7). |

Takeaway: peripherals are already well-isolated. The architecture gaps are (1) MCU-rail hold-up and (2) the LVC/cutoff. Those are the substance of this ECO.

---

## CHANGE 1 — MCU-rail hold-up (fast / ms transients)

**What:** Add bulk on the **`VCC`** node (U2 OUT, pins 1/8 — the `[VCC]` net at U2.1/U2.8/L9.1).
**Value (chosen):** **1× TCJE337M016R0050** — 330 µF, 16 V, tantalum-polymer, E-case, ~50 mΩ ESR — **‖ 10 µF X7R** ceramic. (Same part as the servo cap C10 → BOM consolidation; single cap chosen for board-space constraints.)
**Why:** When servos/camera sag the raw VBATT node (mux IN1), the TPS2121 reverse-blocks IN1 and this cap holds `VCC` up while the buck keeps regulating 3.3 V (valid down to VCC ≈ 3.5 V). Sized by energy (buck draws constant power, so input current rises as VCC sags): `t = ½C(V_start²−V_min²)·η / P_load`. At the actual ~50 mA MCU load (~0.17 W), 330 µF from 8.4→3.5 V gives **~52 ms** hold-up — ~10–50× a vibration/micro-dropout; even at a 500 mA burst it still holds ~5 ms. Voltage derating is comfortable (8.4 V on 16 V ≈ 53%, within the ~80% polymer-tant ceiling); polymer tant also has a benign failure mode and good cold stability. *(If MCU load ever climbs well above ~200 mA at low battery, add a 2nd 330 µF in parallel — but 50 mA leaves large margin.)*
> **CORRECTION 2026-08-08 (worklist H-8/D-3).** The 52 ms figure above does not describe the board as flown and must not be used for sizing. Three things changed or were wrong: (1) the ~50 mA load assumption — the 3V3 chain now carries the S3 *and* the P4 domain (via U30), a realistic flight load of 0.5–0.8 A, which cuts the hold-up to **~3 ms**; (2) C15's 330 µF has moved to the servo branch output, so C56 is the only hold-up on this node; (3) the eFuse outage this was implicitly sized against is **~92 ms** (C46 = 2.2 nF on RETRY_DLY per TPS25982 Table 7-5), so ride-through was never within 30× of closing. The design intent is superseded: transient sags are absorbed *upstream* by the EN/UVLO deglitch (C94, τ ≈ 174 ms, worklist H-1), and any event that still trips the eFuse produces a clean reboot with deployment energy preserved on V_CAP. See WORKLIST.md H-8 for the full arithmetic.

**Soft-start — ✅ verified adequate (no change):** the cap charges through U2 at power-on/USB insert. The existing **C7 = 1 µF** on SS gives an output slew of ~78 V/s (TPS2121: 100 nF ≈ 780 V/s, scales inversely with C_SS), so inrush = C_OUT·dV/dt ≈ 340 µF × 78 V/s ≈ **27 mA** — roughly two orders of magnitude below the R16-set current limit (amps range; TPS2121 min ~1 A) and trivially within the tantalum's surge tolerance. Output ramps gently to ~8.4 V over ~100 ms. No change to C7 needed.
**Watch:** Observe **polarity** — + to VCC, − to GND.
**Placement:** directly across VCC–GND at U2 OUT; keep the buck's existing local input cap (C16) after L9.

---

## CHANGE 2 — Over-discharge cutoff + short protection — ✅ IMPLEMENTED & VERIFIED

Replaced the low-side supervisor+FET LVC with a **high-side smart eFuse** as the single main disconnect for the whole board (Option A: over-discharge protection *and* short/overcurrent protection, carrying servos + camera + logic + pyro charge). Netlist-verified pin-by-pin in `power.kicad_sch`.

**Removed:** Q4 (CSD17308 −BATT low-side FET), U20 (TPS3840PL30 supervisor), and support R47/R61/R62/R78. All gone, no dangling parts.

**Part (U1): `TPS259824LNRGET`** — TI 2.7–24 V, 15 A, 2.7 mΩ smart eFuse, RGE QFN-24. Suffix decode: `24` = 16.9 V OVLO (only OVLO option compatible with an 8.4 V rail; protects the 16 V-rated caps against gross overvoltage); `L` = active current-limiter overcurrent response; RGET = reel. Latch-vs-retry is set by the RETRY_DLY/NRETRY *pins*, configured for **auto-retry** so a transient fault can't brick the flight computer mid-flight.

**Power path (as built):** J2.2 (Batt+) → INA230 shunt R24 → `VBAT_CON` → eFuse IN → eFuse OUT → `VBATT` → TPS2121 mux IN2 (+ servos Q9, camera Q3, pyro charge R69). USB → mux IN1. J2.1 (Batt−) → GND. INA230 re-positioned across the new shunt location (IN+ = battery side, IN−/BUS = VBAT_CON) — measures true pack current + voltage. UVLO senses `VBAT_CON` (pack side, upstream of the FET) so it works even when the output is off.

**Pin map (verified):**

| Pin | Net / part | Value / note |
|-----|-----------|--------------|
| IN (1,2,3,16,pad) | VBAT_CON | CIN = C93 1 µF + C77 100 nF (≥25 V); CR3 CUS10S30 clamp (cathode→IN, anode→GND) |
| OUT (17–24) | VBATT | COUT = C96 1 µF (≥25 V) |
| GND (4,5,14,pad) | GND | continuous ground (Pad1=IN, Pad2=GND are separate) |
| EN/UVLO (6) | R25 1 M (→IN) / R26 210 k (→GND) | **6.34 V falling cutoff / 6.91 V turn-on** (≈3.17 V/cell) |
| ITIMER (7) | C95 4.3 nF | ~2 ms overcurrent blanking (rides servo inrush) |
| ILIM (8) | R18 100 Ω | ~14.7 A limit (no cap on this pin) |
| IMON (9) | GND | unused (INA230 kept for telemetry) |
| RETRY_DLY (10) | C92 2.2 nF | auto-retry delay |
| NRETRY (11) | C78 560 nF | finite retries |
| LDSTRT (12) | GND | load-detect/handshake disabled |
| PG (13) | R27 100 k → +3V3; PG_RAIL → MCU | main-power-good / fault telemetry |
| dVdt (15) | C94 8.2 nF | ~21 ms soft-start, ~0.55 A inrush |

**Negative-transient clamp:** CR3 (CUS10S30, reused) from GND→IN clamps VIN below ground when the eFuse fast-trips into a short (input-lead L·di/dt). Carries a µs-scale ~30 A pulse — within a 1 A Schottky's surge rating; reverse voltage 30 V ≫ 8.4 V. No output-side clamp needed (OUT pin tolerates −0.8 V and downstream capacitance absorbs any inductive kick).

**−BATT eliminated:** with high-side switching, battery negative ties directly to GND — one continuous node. The old switched `−BATT` net is removed; the pyro e-match return (J4.5) now goes to **GND**. *Layout:* star-ground the pyro firing return at the battery-negative/eFuse ground so firing current doesn't bounce the sensor/MCU grounds.

**EN latch — decided OPTIONAL (not fitted).** Earlier flagged as needed, but re-assessed against the actual ~50 mA idle load: the 6.34→6.91 V hysteresis (0.57 V) is far wider than any sag/recovery a 50 mA load can produce, so the eFuse effectively stays off after cutoff, and off-state draw is only the divider (~5 µA) + eFuse Iq (<1 µA). Worst case is minor slow-cycling near the discharge knee from LiPo relaxation, which self-limits and won't deeply over-discharge the pack. A discrete SR latch on EN would guarantee a single clean cutoff, but is not required for pack protection at this load — omitted.

**Current-limit margin note:** RILIM = 100 Ω → 14.7 A typ, 12.85 A cold-min. With 4 servos × 2–3 A the worst-case simultaneous stall (~14 A) sits close to that; the ITIMER blanking covers brief peaks, but if sustained 4-servo stall is real, step to the **TPS25983 (20 A)** for headroom.

---

## CHANGE 3 — Peripheral shedding (confirm, minor adds)

Firmware must be able to shed camera/radio/GNSS to protect the MCU domain (your "OK to lose those, not the MCUs").
- **Confirm** `SERVO_ACT`→Q9, `CAM_ACT`→Q3, `GPS_ACT`→Q8, `LoRa_ACT`/R73 each actually gate their branch **power** (not just an enable line). If `LoRa_ACT` only toggles a logic pin, add a P-FET high-side switch on the J9 supply so the daughterboard can be powered down.
- **Local bulk:** servo (C10 330 µF) ✓, camera (C80 100 µF) ✓. **Add ~10–47 µF** on the GNSS (J3) and LoRa (J9) supply pins if not already present, so their own inrush/TX bursts stay on their branch.

---

## CHANGE 4 — Star the raw-VBATT taps (layout-driven net separation)

Route the **servo (Q9)** and **camera (Q3)** VBATT feeds as **separate star branches from the eFuse output node**, not daisy-chained through the mux-input trace. Keeps servo di/dt off the buck input copper. Reflect this in the schematic by drawing them as distinct branches from a single post-disconnect `VBATT_SW` node rather than a shared net stub.

---

## CHANGE 5 — Housekeeping after the topology change

- With Q4 gone from −BATT, re-verify **pyro return (J4.5 `−BATT`)** and **INA230 IN−/reference** tie to the now-continuous ground.
- Rename the post-disconnect rail (e.g. `VBATT_SW`) and confirm mux IN1, Q9, Q3, and the pyro charge resistors all tap it (so the eFuse protects everything, while the supervisor senses the pack side).
- Re-check the U4 (TPS22918) `ON` default: pulldown so **VPP/P4 default OFF** at power-up (C-4 interlock).

---

## CHANGE 6 — Eliminate sneak-power into the OFF P4/VPP domain

Goal (per the "S3 up first at minimal power" intent): when only S3 (+3V3) is powered and VPP is off, the P4 domain should draw ~0. Current sneaks in wherever an **always-on output drives a pin on an off chip** — it flows through that chip's I/O clamp diode into the off VPP rail, wasting power *and* floating VPP up toward ~(V_drive − 0.5 V), partially waking the whole domain and defeating the U4 switch.

**What's already right (no change):** the inter-MCU I²C pull-ups **R57/R58**, the sensor CS pull-ups **R80/R81**, the mag I²C **R79/R82**, and the P4-flash pull-ups **R43–R46** are all referenced to **VPP**, so they die with the domain. Sensor↔P4 lines are intra-VPP (off together). No unavoidable resistive leak exists.

**Issue 1 — S3 logging memory (U11 NAND, U12 MRAM) is powered from VPP** but driven by the always-on S3:
`M_SCK/M_MOSI/M_MISO` (U11.6/5/2, U12.6/5/2 ↔ U16.42/43/40), `M_FLASH_CS` (U11.1↔U16.41), `MRAM_CS` (U12.1↔U16.39). Before VPP-on, S3 driving these injects into U11/U12 clamps → VPP; and S3 can't use its own store until the P4 domain is up.

**Issue 2 — inter-MCU SPI** `ESP_CS/SCLK/SDO/SDI` (U16↔U18) is push-pull, no pull-ups → same injection into U18 if driven before VPP-on. (The I²C half is safe — pull-ups on VPP.)

**Fixes, in priority:**
1. **✅ IMPLEMENTED (V9) — moved the S3 log memory off VPP to +3V3.** In `esp32s3_outputs.kicad_sch`, U11 (NAND) and U12 (MRAM) VDD (pin 8, with C18/C19 decoupling) were moved from VPP → **+3V3**, and the memory **WP#/HOLD# pull-ups R43–R46** (U11/U12 pins 3 & 7) were moved to **+3V3** with them so those control pins stay defined when VPP is off. The S3 sheet now has zero VPP dependency; logging can run whenever S3 is up, and there's no cross-domain drive into the memory. (Power-symbol swap only — connection nodes, symbol/wire counts unchanged; verified.)
2. **Firmware (still applies to the inter-MCU `ESP_*` SPI):** the `ESP_CS/SCLK/SDO/SDI` bus still bridges the always-on S3 to the VPP-domain P4. At S3 boot, hold these **LOW or hi-Z** and don't initialize that SPI peripheral until VPP is enabled/settled. (No longer needed for the memory bus after fix 1.)
3. If any remaining cross-domain line must be active before VPP: add **1 kΩ series resistors** or a **bus-isolation switch** (NX3L/TS3A class).
4. Confirm **U4 (TPS22918) QOD** is enabled (or add a small bleed) so VPP sits firmly at 0 V when off — any residual injection into the P4/sensor pins is then clamped rather than floating the rail.

**Result:** memory is now on the S3's own +3V3 domain (done); with the `ESP_*` SPI tristated in firmware and VPP pulled to 0 when off, the P4 domain is genuinely cold at first S3 power-up.

## CHANGE 7 — Pyro energy store: 0.5 F supercap → 10 mF electrolytic

**Replace C13** (CPM-12R0L504R-TW, ~0.5 F multi-cell supercap) with **Nippon Chemi-Con KY series `EKYC160ELL103MM25S`** — 10,000 µF, 16 V, low-impedance 105 °C radial aluminum electrolytic (~10 mF).

**Why:** firing 4 e-match channels needs only ~10–30 mC total (all-fire ~0.33–0.5 A, ~1 Ω match; at the ~8.4 V rail ignition is sub-millisecond). 10 mF holds the rail up with ≤~1 V droop even firing all four at once — ~15–50× margin — while being far smaller/cheaper than the 0.5 F supercap. KY low ESR (tens of mΩ vs the supercap's hundreds) gives higher, faster peak firing current. 16 V rating = 1.9× margin over 8.4 V.

**Footprint:** now a **polarized 2-terminal radial** (was a non-polar supercap). Update the land pattern (KY 10,000 µF is ~7.5 mm lead pitch) and set polarity: **+ → V_CAP, − → GND.** Fits the existing ~18 mm-square body keep-out. ⚠️ **Verify** the exact can diameter (≤18 mm) and height for this specific case-code against the Chemi-Con KY datasheet and your bay clearance — the common 10,000 µF/16 V KY case is Ø18×40; confirm your `MM25S` variant's dimensions before committing the footprint.

**Charge resistor — ✅ IMPLEMENTED (V9):** the 4× 100 Ω ∥ bank was consolidated to a **single RCP0603 150 Ω** (R70/R71/R72 removed; R69 value → `RCP0603W150RGEB`, 150 Ω). Rationale: with only 10 mF, the old 25 Ω fast charge is unnecessary, and the per-charge energy dropped ~50× (½CV² ≈ 0.35 J vs 17.6 J) so one 0603 handles it. At 150 Ω the peak charge power is 8.4²/150 ≈ **0.47 W** (within a pulse-rated 0603), τ ≈ 1.5 s → recharge ~7 s, and inrush is a gentle ~56 mA (was ~336 mA) — easier on VBATT. Recharge speed is non-critical because the 10 mF cap already buffers ~3–5 fires (24 mC usable from 8.4→6 V) and firing current comes from the cap through the FETs, not through R69. *Layout note:* delete the R70–R72 footprints; a few now-redundant junction dots remain on the V_CAP/VBATT rails in the schematic (harmless — tidy with Cleanup in the editor).

## Net effect

| Threat | Before | After |
|--------|--------|-------|
| Servo/vibration ms sag | Resets MCUs | Absorbed upstream by the C94 EN/UVLO deglitch; the Change-1 hold-up is ~3 ms at flight load, not 52 ms — see the 2026-08-08 correction above |
| Sustained low pack | −BATT cutoff, no deglitch, floats ground, can chatter | High-side eFuse, UVLO 6.91/6.34 V, deglitched by C94 (τ ≈ 174 ms, added 2026-08-06 — did not exist when this row was first written), **auto-retry not latched** (~92 ms × ~1024 per C46/C45) (Change 2) |
| Radio/GNSS/cam transient | Shared rails | Already switched; shed in firmware; local bulk (Change 3) |
| Pyro energy vs logic | — | Fully isolated (supercap = pyro only) |
| Sneak-power into off P4 domain | S3 drives log memory (U11/U12) + SPI on VPP → back-powers VPP | Memory on always-on rail + firmware tristate; off-state VPP ≈ 0 (Change 6) |
| Oversized pyro store | 0.5 F supercap, ~18 mm, slow ~12 s recharge, 4× charge R | 10 mF KY electrolytic (EKYC160ELL103MM25S) + single RCP0603 150 Ω charge R, lower ESR, ~7 s recharge, buffers 3–5 fires (Change 7) |
| Ground integrity | Return through −BATT FET | Continuous ground; high-side disconnect (Change 2A) |

---

## Bench checklist before committing V9

1. TPS2121 reverse-blocking hold-up with the new 330 µF on VCC — confirm VCC holds while VBATT (IN1) sags and no disruptive switchover occurs with no USB present. (Startup inrush already calc-verified ~27 mA ≪ ILIM — see Change 1.)
2. Worst-case servo move: confirm `VCC`/+3V3 stays in reg and the LVC does **not** trip (deglitch adequate).
3. Sustained low-pack: LVC latches off at ~6.4 V and re-enables only on charge/USB; measure off-state pack drain.
4. Pyro continuity + fire still function with sensors/P4 on VPP and the new ground topology.
5. (Deferred) ERC once pin labels are cleaned up — several symbols need correct pin naming first.

---

*Open choice for you: Change 2A (integrated eFuse — fewer parts, cleaner) vs 2B (keep TPS3840, add deglitch+latch+high-side FET — reuses what you know). I'd go 2A unless eFuse footprint/availability is a problem. Say the word and I'll spec a specific eFuse part with the UVLO/ILIM resistor values for 2S + your hold-up/servo load.*
