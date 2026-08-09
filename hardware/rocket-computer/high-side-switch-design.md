# High-side branch switching — design to replace the four low-side N-FETs

**Purpose.** Replace Q1 (GNSS), Q7 (camera), Q8 (servo/EXP) and Q10 (LoRa) with high-side switches so each connector's ground pin becomes a hard ground. That removes the floating-ground failure mode at its source, rather than bounding its symptoms with series resistors, and it lets the twelve EXP lines stay free of series impedance so the header remains usable for I2C, SPI or anything else.

**Control polarity is unchanged** — GPIO high still means branch on — so no firmware change is needed.

---

## What each branch has to carry

| Branch | Connector | Downstream bulk | Design current |
|---|---|---|---|
| GNSS | J1.3 (via FL1) | 22 µF on-board + daughterboard buck input | 0.5 A |
| LoRa | J5.2 (via FL2) | 22 µF on-board + daughterboard buck input | 0.5 A |
| Camera | J4.2 | 22 µF on-board + camera input caps | 1.5 A |
| Servo / EXP | J3.15, J3.16 | **400 µF** on the servo adapter (4 × 100 µF) | per D-1, assume 3 A |

The servo branch is the awkward one on both counts — highest current and, far more importantly, 400 µF of bulk sitting behind the switch. Hot-switching into that is what drives the slew-control design below.

---

## Circuit — discrete version

Per branch, five parts:

```
                VBATT
                  │
        ┌─────────┼─────────┐
        │                   │
     [R_pu]              S ─┴─
      470k              [P-FET]      body diode blocks
        │               D ─┬─        with S on VBATT
        ├───────[Cslew]────┤
        │                   │
     [R_g]                  ├──► ferrite ──► connector supply pin
      100k                  │
        │
     collector
   [digital transistor]  ── emitter ──► GND
     base ◄── 3V3 GPIO (internal 2.2k series, 47k pulldown)
```

**R_pu = 470 kΩ 0402** — gate to source. This is what holds the switch **off with no drive at all**, which is the power-up requirement (it replaces the 10 kΩ gate pulldowns the N-FETs use today).

**R_g = 100 kΩ 0402** — gate to the driver's collector. With R_pu it sets the on-state gate drive:

- Vgs(on) = −VBATT × 470/570 = **−6.9 V at 8.4 V**, **−5.3 V at 6.4 V**
- Divider current when on: 15 µA. Off-state current: zero.

**Driver: a digital transistor with the internal 2.2 k / 47 k network** — the same family already used for the pyro gate drives. The internal base-emitter shunt gives default-off with no external parts, which is why this is cheaper than a plain N-FET plus its own pulldown.

**Cslew** — gate to drain (Miller). This is what turns a hot-swap into a controlled ramp. During the Miller plateau the gate current is roughly V_plateau / R_g ≈ 64 µA, and the drain ramps at dV/dt = 64 µA / Cslew:

| Branch | Cslew | Ramp | Peak inrush |
|---|---|---|---|
| Servo / EXP | **15 nF** | ~2.0 ms | **1.7 A** into 400 µF |
| Camera | **2.2 nF** | ~0.3 ms | ~1.2 A |
| GNSS, LoRa | **2.2 nF** | ~0.3 ms | ~1.2 A |

Without Cslew the servo branch would slam 400 µF from a hard source and draw tens of amps — enough to threaten the eFuse fast-trip at 2.1 × ILIM ≈ 24 A. This is not optional on that branch.

### P-FET selection

| Parameter | Small branches | Servo / EXP |
|---|---|---|
| V_DS | ≥ 20 V | ≥ 20 V |
| V_GS | ≥ ±12 V (we reach −6.9 V) | ≥ ±12 V |
| R_DS(on) **specified at V_GS = −4.5 V** | ≤ 100 mΩ | ≤ 30 mΩ |
| I_D | ≥ 2 A | ≥ 6 A |
| Package | SOT-563, SOT-23, 1×1 DFN | 2×2 or 3×3 DFN |

Dissipation is trivial in steady state — 0.1 W on a small branch, 0.27 W on the servo branch at 3 A.

**The one thing to check carefully is turn-off SOA on the servo branch.** Cslew slows turn-off as well: τ ≈ R_pu × Cslew = 470 k × 15 nF ≈ 7 ms, during which the FET traverses its linear region carrying load current. At 3 A across a mid-rail 4 V that is roughly 12 W peak and ~84 mJ per shed event. A 3×3 DFN takes that comfortably; a SOT-23 may not. If you would rather shorten it, drop R_pu to 220 kΩ — turn-off falls to ~3.3 ms and Vgs(on) is still −5.8 V at 8.4 V and −4.4 V at 6.4 V.

---

## Circuit — integrated load switch alternative

This is probably the better answer for the three small branches, and possibly all four. You already use the pattern for the P4 domain, just with a part limited to 5.5 V input — this needs a higher-voltage sibling.

Specification:

- **V_IN range covering 6.0–8.5 V** — pick a part rated ≥ 12 V for headroom
- **EN directly 3.3 V-compatible**, with an internal pulldown so the default is off (add one 100 kΩ 0402 if the part lacks it)
- **Adjustable slew rate via a single CT capacitor** — soft-start becomes a datasheet parameter instead of a Miller calculation
- **Current**: ≥ 1.5 A for GNSS / LoRa / camera; ≥ 4 A for servo
- **Reverse-current blocking** is a nice-to-have — it stops a branch back-feeding VBATT
- Package: SOT-23-6, SC-70-6 or 2×2 DFN for the small ones

Parts per branch: **IC + 1 slew capacitor**, optionally + 1 EN pulldown.

### Part count comparison

| Approach | Parts per branch | Four branches |
|---|---|---|
| Today (low-side) | N-FET + 1 k + 10 k | 12 |
| Discrete high-side | P-FET + driver + 470 k + 100 k + Cslew | 20 |
| Integrated load switch | IC + Cslew (+ EN pulldown) | 8–12 |

The integrated route is the smaller board footprint *and* the smaller part count, and it makes soft-start a specification rather than something to calculate and verify. The discrete route's advantage is that you can pick exactly the FET you want for the servo branch's current and SOA.

---

## Net changes

**Grounds become hard** — this is the whole point:

- `Net-(J1-Pad4)` → GND
- `Net-(J3-Pad1)` (J3.1, J3.2) → GND
- `Net-(J4-Pad1)` → GND
- `Net-(J5-Pin_1)` → GND

**Supplies become switched** — each connector's power pin moves off VBATT onto its own switch output:

- J1.3 ← FL1 ← GNSS switch output
- J4.2 ← camera switch output
- J3.15, J3.16 ← servo switch output
- J5.2 ← FL2 ← LoRa switch output

Keep each ferrite between the switch output and the connector so the switch itself sees clean VBATT.

**Removed:** Q1, Q7, Q8, Q10 and their gate networks — R6/R7, R24/R25, R27/R29, R34/R35.

**Control nets unchanged:** `GPS_ACT`, `CAM_ACT`, `SERVO_ACT` and `LoRa_ACT` drive the digital-transistor bases (or the EN pins) with the same high-equals-on sense as today.

---

## Things that will bite if missed

**Orientation.** Source to VBATT, drain to the load. Reversed, the body diode conducts from VBATT to the load continuously and the switch can never turn off. This is the single most common way to get a P-FET high-side switch wrong.

**Default-off at power-up.** R_pu (or the EN pulldown) must hold the switch off with no active drive, before either MCU boots. Verify this specifically — it is the property the current design gets from its 10 kΩ gate pulldowns, and it must survive the change.

**Gate drive at low battery.** Everything above assumes R_DS(on) specified at V_GS = −4.5 V, because at a 6.4 V pack the drive is only −5.3 V. A FET characterised only at −10 V will disappoint.

**Servo inrush.** 400 µF behind the switch. Do not fit that branch without Cslew.

**EXP pins stay bare.** Once J3's ground is hard, the twelve EXP lines need no series resistors — which was the reason for doing this rather than adding them. Leave them clean.
