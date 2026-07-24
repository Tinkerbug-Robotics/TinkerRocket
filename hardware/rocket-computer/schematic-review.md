# TinkerRocket Full V9 — Schematic / Electrical Review

**Rev 1:** July 12, 2026 — initial review
**Rev 2:** July 12, 2026 — incorporates Christian's answers; C-1/C-2 closed, Section 3 notes retracted, C-3 expanded into a build-ready power-architecture spec.
**Scope:** Schematic-level electrical review of all five sheets. No PCB layout / DRC in this pass.
**Method:** Parsed the KiCad 10 source files to rebuild pin-level net connectivity, cross-checked part functions against datasheets, reviewed against intended function.

---

## Status at a glance

| # | Finding | Status |
|---|---------|--------|
| C-1 | Stale BOMs | **Closed** — BOM re-exported from current schematic |
| C-2 | Pyro FET V_GS rating | **Closed** — TPN4R712MD is ±12 V; 2S worst-case ≈ −8.4 V, ~3.6 V margin |
| C-3 | Power-path brownout / battery over-discharge | **Open — design detail below (primary focus)** |
| C-4 | Pyro arming | **Acceptable for drogue/main**; physical interlock still required for airstart/staging |
| §3 | TPS3840 / TPS62152 notes | **Retracted** — both were correct as designed |
| §4 | Supercap discharge / safe state | **Closed** — verified working; supercap now scoped to pyro-only |
| §5–6 | BLE arch, antenna, NAND | **Closed / noted** — see below |

---

## 1. Architecture as understood (confirmed)

- **ESP32-P4 (U18)** — flight computer: sensors, pyros, servos/GPIO, camera control, own buck (U19 TLV62569, `EN_DCDC`), local flash (U17). Native USB pins 52/53 (`CEN_D±`). No radio.
- **ESP32-S3 (U16)** — logging + radio: LoRa (E220-900, UART), **BLE/Wi-Fi to the iOS app via on-board 2.4 GHz antenna AT1** (confirmed), logging memory (NAND U11, MRAM buffer U12, flash U13). Also gates P4 power via the TPS22918 (see C-4).
- **Inter-MCU link:** SPI (`ESP_SCLK/SDI/SDO/CS`) + I²C (`ESP_SCL/SDA`).
- **Power:** TPS2121 mux (U2) VBATT/USB → `VCC` → filter L9 → TPS62152 buck (U10) → **+3V3**; TPS22918 (U4) → `VPP`; INA230 (U8) current sense; TPS3840 (U20) LVC → Q4 (−BATT cutoff).
- **Pyro:** supercap `V_CAP` (C13) → arm P-FET U9 → `PYRO_POS` → 4× fire P-FET (U7/U24/U25/U26) → `PYROx_EXT` → e-match → `−BATT`. Continuity from VPP divider.
- **Actuators/IO:** servo/GPIO J5 (switched VBATT via Q9, C10 330 µF), camera J6 (via Q3), status LEDs + piezo (LS1/M1).

---

## 2. Closed findings (record)

**C-1 — BOM:** Re-exported. The prior `.txt` (V7-era) and `.csv` (listed absent VN5E160) are superseded. Going forward, treat the schematic as source of truth and re-export per revision.

**C-2 — Pyro gate rating:** TPN4R712MD confirmed **±12 V V_GSS**. On 2S, worst-case firing Vgs ≈ −8.4 V (gate to GND, source at fully-charged supercap ≈ pack voltage). ~3.6 V margin — **no gate clamp needed.** Only revisit if the pack ever moves to 3S (12.6 V would exceed the rating).

**§3 retractions:** The TPS3840PL30 + R47/R61 divider on VDD is an intentional LVC trip-setter: 3.0 V × (11.3k+10k)/10k ≈ **6.4 V** (~3.2 V/cell on 2S) — correct by design. The TPS62152 is the fixed-3.3 V variant, so no FB divider is expected. Both original notes withdrawn.

**§4 — Supercap safe state:** Discharge path verified working on the bench. With the Rev-2 decision to scope the supercap to **pyro firing only** (no MCU hold-up coupling), this subsystem is clean and isolated.

**§5–6:**
- BLE architecture confirmed: app connects to the **S3**; P4 flight data crosses the inter-MCU bus.
- The unintended second 2.4 GHz antenna was removed; replacement planned in V9. No double-load.
- NAND `F35SQB004G` is the intended availability-driven replacement for `MX35UF4G24AD` (likely oversized; a cost-optimization item for a later spin, not a functional blocker). Confirm the footprint/pinout match on the placed part.

**C-4 — Pyro arming (acceptable for drogue/main):** Two-stage arm→fire with DTC123J internal pulldowns keeps all FETs off at boot (fail-safe). Additional layer confirmed: **the P4 (pyro controller) is held unpowered by an S3-controlled TPS22918 until at the pad**, so two independent MCUs must both act to fire apogee/main. Two confirmations to close fully: (a) the TPS22918 `ON` pin has a pulldown so P4 defaults **off**; (b) S3 firmware asserts P4 power only as a deliberate pad step. **For airstart/staging** (fire while airborne and powered), add a physical interlock in the pyro energy path — software layering is not sufficient for in-flight ignition.

---

## 3. C-3 — Power distribution redesign (build-ready)

### 3.0 Two problems, two timescales — keep them separate
The V8/V9 power pain is really two different jobs that got folded into one node and one cutoff:

1. **Fast (ms):** battery voltage momentarily dips from vibration/connector micro-disconnects and servo/radio current steps. The MCUs must not notice. → **solved with hold-up capacitance + the mux's reverse blocking**, not with the LVC.
2. **Slow (minutes–hours):** the unit is left on and drags the 2S pack below a safe voltage. → **solved with a battery-sensed, deglitched, latching LVC** that shuts the whole board down. This must *not* react to the fast transients.

Designing these as separate mechanisms on separate timescales is the core of the fix.

### 3.1 Decision: supercap fires pyros only
No diode-OR from the supercap to the MCU rail. It keeps the energetic pyro store away from logic, removes a fault path, and avoids the "firing browns the MCU" coupling. The supercap subsystem is unchanged except for whatever isolation the star distribution below implies.

### 3.2 Load domains and priority
Star-distribute from the pack so each domain's transient stays local. Priority in flight: **MCUs must stay up; camera/radio/GNSS are nice-to-keep but droppable; servos are the main aggressor; pyros are gentle on the pack (cap-fed through 25 Ω).**

| Domain | Loads | Character | Local bulk | In-flight priority |
|--------|-------|-----------|-----------|--------------------|
| **Logic (protected)** | Both MCUs, sensors, logging (via mux → buck → 3V3) | Sensitive, low current | **Hold-up cap on VCC (0.47–1 mF)** | Must stay up |
| **Peripheral (droppable)** | LoRa/E220, GNSS, camera | Bursty (radio TX), tolerant of blips | Per-branch bulk (e.g. 47–100 µF each) | Keep if possible; OK to lose |
| **Servo (aggressor)** | Servos on J5 | Large stall/inrush steps | Big bulk (330 µF onboard + 400 µF daughterboard) | Isolated so its sag stays local |
| **Pyro** | Supercap → e-matches | Slow charge, fires from cap | Supercap C13 | Isolated; charge draw is gentle |

### 3.3 Proposed topology
```
                         2S PACK (6.4–8.4 V)
                                │
                        [INA230 shunt R24]         ← pack current telemetry
                                │  (Kelvin sense here)
                   ┌────────────┴─────────────┐
                   │                           │
          [LVC sense network]          MAIN DISCONNECT (high-side)
       RC-filtered divider → TPS3840   ── gated by LVC, deglitched, latched
       (battery-referenced, slow)                 │
                                          RAW VBATT NODE (post-disconnect)
        ┌──────────────┬───────────────┬──────────────┬───────────────┐
   [TPS2121 IN1]  [servo sw Q9]   [cam sw Q3]   [radio/GNSS sw]  [pyro chg 25Ω]
   (+USB on IN2)       │ +bulk         │ +bulk         │ +bulk          │
        │           SERVOS          CAMERA        LoRa / GNSS      SUPERCAP C13
   [VCC + HOLD-UP    (aggressor)   (droppable)    (droppable)         │
    bulk 0.47–1 mF]                                                pyro FETs
        │  ▲ reverse-blocked by TPS2121 when IN1 sags
   [buck TPS62152] → +3V3 → MCUs + sensors  (PROTECTED DOMAIN)
```
Key point: servos/camera/radio/GNSS tap the **raw VBATT node** (mux IN1), so their sag lives on that node. The logic domain sits **behind the TPS2121**, whose reverse-current blocking + the VCC hold-up cap keep `VCC` up while IN1 dips. That single arrangement is what stops a servo step from resetting the MCUs.

### 3.4 MCU rail hold-up sizing (the ms problem)
Mechanism: bulk on `VCC` (TPS2121 output / buck input); when the raw node (IN1) sags below `VCC`, the mux blocks reverse current and the cap carries the buck. The buck keeps regulating 3.3 V as long as `VCC` stays above ~3.5 V, so the usable droop window is large (8.4 → 3.5 V).

Sizing: `C = I_in,max · t_dropout / ΔV_allowed`
- Worst-case buck input current (constant-power, near dropout, ~3.3 W logic load): `I_in,max ≈ 1 A`.
- Ride-through target for vibration/micro-disconnect: `t_dropout ≈ 2–5 ms`.
- Allowable droop: `ΔV ≈ 8.4 − 3.5 = 4.9 V`.

→ `C ≈ 1 A × 0.005 s / 4.9 V ≈ 1.0 mF` for a full 5 ms disconnect at 1 A; **470 µF gives ~2.4 ms** at the same load, and much more for shallow sags. Recommend **470 µF–1 mF low-ESR (polymer or electrolytic) on VCC, paralleled with a 10 µF ceramic.** ESR is negligible here (1 A × ~50 mΩ ≈ 50 mV step).

Confirm two things:
- **Inrush:** the added VCC bulk is charged through the TPS2121 at power-on / USB insertion — verify it's within the mux's soft-start (`SS`) and current-limit (`ILIM`) settings so the part doesn't fault or the source doesn't collapse.
- **Reverse-blocking / switchover:** verify from the TPS2121 datasheet that when IN1 sags with no valid IN2 (in flight, no USB), the part holds output from the cap rather than forcing a disruptive switchover. This is the behavior the hold-up relies on.

### 3.5 Battery over-discharge cutoff (the hours problem)
Goal restated: don't let the board sit powered for hours and pull the 2S pack below safe. Redesign points:
- **Sense the battery, slowly.** Keep the TPS3840 + divider (trip ≈ 6.4 V ≈ 3.2 V/cell — good), but put a deliberate **RC deglitch (≥100 ms)** on the sense so servo/radio transients can't trip it. The LVC should only react to *sustained* low pack voltage.
- **Latch off.** Once tripped, latch the disconnect off and require a power-cycle or charge/USB to re-enable. Prevents chatter and the "recover → keep draining → re-trip" cycle that slowly kills the pack anyway. A simple SR latch or the supervisor's `MR`/enable with a latching P-FET stage does this.
- **Prefer a high-side main disconnect** over the current −BATT (Q4) ground switch. Ground-side switching puts every return — servo, pyro (`−BATT` at J4.5), and the INA230 shunt reference — on top of Q4's Rds and only referenced while Q4 is on; a transient open floats the common return mid-flight. A high-side switch keeps a clean, continuous ground. If you keep the low-side switch for layout reasons, make absolutely sure the deglitch/latch means it never opens transiently.
- **Watch quiescent draw.** The divider itself pulls ~0.4 mA continuously (~10 mAh/day) — negligible vs the pack, but confirm nothing else stays alive downstream of a "cut" board.

### 3.6 Minimal vs. full change (your call for V9 vs V10)
- **Minimal (drop into V9 now):** add the 0.47–1 mF hold-up cap on `VCC`; add the RC deglitch + latch to the existing TPS3840/Q4 LVC; add/confirm local bulk on the servo and camera/radio branches. This alone should stop the resets and protect the pack.
- **Full (V10):** move to the high-side main disconnect and the clean star distribution in 3.3, with per-domain switches so firmware can shed camera/radio/GNSS on demand and guarantee the MCU domain last.

---

## 4. Remaining verification checklist

1. TPS2121 inrush + reverse-blocking behavior with the new VCC hold-up cap (3.4).
2. TPS22918 `ON` defaults P4 **off** at power-up (C-4).
3. LVC deglitch + latch values chosen against measured worst-case servo sag (3.5).
4. Run KiCad **ERC** and resolve any strap/pull warnings on both MCUs (couldn't fully verify boot straps from nets alone).
5. Confirm placed NAND footprint/pinout matches `F35SQB004G` (§6).
6. E-match return: decide whether to keep common `−BATT` return (safer against airframe shorts) or move to per-channel return pairs (better fault isolation) — see prior discussion; recommendation is per-channel returns while **keeping** the high-side switches.

---

*Rev 2 reflects: BOM re-exported; 2S battery; supercap scoped to pyro-only; MCU hold-up via VCC bulk + mux blocking (not the supercap); LVC to be battery-sensed, deglitched, latched, preferably high-side; camera/radio/GNSS droppable, MCUs protected last.*
