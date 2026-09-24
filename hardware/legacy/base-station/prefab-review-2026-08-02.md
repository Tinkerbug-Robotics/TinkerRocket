# TinkerRocket Base Station — Pre-Manufacturing Review

**Date:** 2026-08-02
**Source:** `TinkerRocket-Hardware/hardware/base-station/` — **the live working tree**, including the
uncommitted edits (schematic + PCB) that implement the 2026-08-02 power-switch review. Title block
still reads `rev V5`.
**Scope:** every fitted component and its supporting parts, the power/storage/RF architecture, and
the PCB layout — specifically the classes of defect DRC does *not* catch (datasheet compliance,
footprint-vs-part mismatch, plane/return-path structure, EPAD stitching, firmware/hardware drift).
**Method:** `kicad-cli` XML netlist as connectivity ground truth; direct s-expression parsing of
`base-station.kicad_pcb` for geometry (pad transforms calibrated against routed track endpoints);
`kicad-cli pcb drc --severity-all`; exported F.Mask/F.Paste gerbers to verify apertures; primary
datasheets pulled for TPS61023, TPS63020, MP2672A, Molex 47948-0001, JST XH/EH, ESP32-S3;
cross-checked against the fabbed-and-working `rocket-computer` and `lora-daughterboard`.

**Board:** 30.27 × 90.50 mm, 4-layer, 1.6 mm. 118 placements, 104 nets, 622 track segments,
222 vias, 20 zones.
**Stackup as declared *at the time of review*:** F.Cu / **0.1 mm prepreg** / In1.Cu /
**1.24 mm core** / In2.Cu / **0.1 mm prepreg** / B.Cu — which was KiCad's untouched 4-layer
default, not an ordered stack. **Superseded by S7:** the board now carries JLCPCB
**JLC04161H-7628** (0.035 / **0.2104** / 0.0152 / **1.065** / 0.0152 / **0.2104** / 0.035 mm,
1 oz outer / 0.5 oz inner, ENIG). Quote the JLC figures, not the ones on this line — every
impedance number in S7 depends on it. `In1.Cu` and `B.Cu` and `F.Cu` are one GND pour;
`In2.Cu` is a +3V3 plane.

---

## STATUS — updated 2026-08-04

The board is **DRC-clean: 0 errors, 0 unconnected items, 0 schematic-parity issues.** Only
silkscreen warnings remain (26 `silk_overlap`, 6 `silk_edge_clearance`, 2 `silk_over_copper`,
2 `nonmirrored_text_on_back_layer`). Two findings below were **wrong** and are withdrawn; see the
corrections.

**The one open item that is a deliberate trade, not an oversight:** the antenna ships with `L4`
(4.3 nH shunt) alone and no matching provision — see A1 under the antenna section. Characterise it
on this build with the cell installed and carry the result into the next revision.

| # | Finding | Status |
|---|---|---|
| B1 | `Net-(U2-VIN)` orphaned | **FIXED** — `V_SWITCH` now carries `C34.1`/`L8.1`/`U2.3` |
| B2 | `R17`/`R18` wrong MPN | **FIXED** — `R17` 18.7 k `RC0402FR-0718K7L`, `R18` 1 k `RC0402FR-071KL` |
| B3 | No vias in the exposed pads | **WITHDRAWN — my error.** See correction 1 |
| B4 | `J4` XH part in an EH footprint | **PARTLY WITHDRAWN.** See correction 2. Kept as EH by decision |
| B5 | Firmware board profile stale | **TRACKED** — issue #714, does not gate fab |
| S6 | No series R on the LoRa UART | **WITHDRAWN** — the daughterboard's `R77`/`R78` (1 k each) sit between its ESP32 and the connector, so they limit injection in *both* directions |
| S7 | Impedance not declared | **FIXED** — JLC04161H-7628 stackup, RF netclass, feed widened 0.20 → 0.36 mm |
| S9 | Netclass defaults illegal | **FIXED** — via 0.45/0.30, and the board rules moved to JLC's standard tier |
| S8 | In2 plane cut by power routing | **OPEN** |
| N10–N18 | assorted | mostly **OPEN**, all low severity — see the residual list at the end |

### Correction 1 — B3 was overstated; the via ring is fine

The finding measured distance to *vias only* and ignored plated through-holes, and it treated
"0 vias inside the pad" as the metric. The metric that matters is **spreading distance**, and the
board is good: **15 GND vias sit within 1.5 mm of the S3's EPAD edge** (closest 0.32 mm, mean
0.51 mm, distributed on all four sides). Path inductance EPAD → planes is roughly

```
15 vias in parallel              ~0.02 nH
+ ~0.5 mm of top-pour spreading  ~0.20 nH
                          total  ~0.23 nH      (a 3x3 in-pad array would be ~0.04 nH)
```

0.23 nH is 3.5 Ω at 2.4 GHz — below the QFN's own paddle bond inductance. With no via filling
available on a 4-layer order, a tight ring is the *correct* trade: it protects the EPAD solder joint,
which matters more here than 0.2 nH. The sibling boards' in-pad vias are a different choice, not a
better one. **Closed.**

### Correction 2 — B4's "will not seat" claim was false

KiCad's own `JST_XH_S3B-XH-A_1x03_P2.50mm_Horizontal` uses the **same 0.95 mm drill** as the EH
footprint, and fabs treat that as the finished hole size, so an XH post (□0.64 mm, 0.905 mm diagonal)
seats fine. The hole size was never the problem.

What *is* different is the body: the real `S3B-XH-A` is **9.2 mm deep from the pin row vs the EH
outline's 6.7 mm**. A side-entry housing only mounts one way, so the real connector overhangs the
**board edge** by ~3.3 mm where the drawing shows 0.79 mm. (An earlier draft of this correction
claimed a collision with the 18650 — that was an artifact of KiCad drawing its XH and EH horizontal
footprints with the body on opposite sides of the pin row. There is no battery interference.)

Kept on the EH footprint by decision, since the pads are identical and swapping would wrongly imply
a different connector. **Residual: silk/courtyard/3D understate the real part by 2.5 mm at the board
edge** — worth a note if the enclosure is tight there.

---

## Headline (as originally written)

Five things should stop the gerbers. One of them — **B1** — is a regression introduced by
implementing the previous review's own recommendation, and it leaves the 5 V rail that powers the
LoRa daughterboard with no input connection at all.

| # | Finding | Why it blocks |
|---|---|---|
| **B1** | `Net-(U2-VIN)` has no power source; PCB "connects" it only via an illegal via-on-pad short | V_LORA rail dead as drawn |
| **B2** | `R17`/`R18` both carry MPN `RC0402FR-075K11L` (5.11 k) | LoRa power-cycle recovery silently never works |
| **B3** | Zero vias in the ESP32-S3 EPAD and the TPS63020 PowerPAD | RF/power ground return + thermal path; both sibling boards do this correctly |
| **B4** | `J4` is a JST **XH** part in a JST **EH** footprint (Ø0.95 mm holes) | Connector will not seat |
| **B5** | Firmware board profile describes a fuel gauge and a NAND that are no longer on the board | No battery readout at all; log capacity drops ~250× |

---

## B1 — The TPS61023 boost input is orphaned, and the PCB fakes the connection

**Schematic.** `Net-(U2-VIN)` contains exactly three nodes:

```
Net-(U2-VIN) = { C34.1 , L8.1 , U2.3 (VIN) }
```

There is no source. `V_SWITCH` never reaches it. As drawn, the boost that produces `V_LORA` — the
only rail feeding the radio daughterboard through `J6` — has no input.

**PCB.** DRC reports **two unconnected items** on that net (`U2.3 ↔ C34.1` and `C34.1 ↔ L8.1`), and
then three hard errors all at the same coordinate:

```
ERROR shorting_items      Pad 1 [Net-(U2-VIN)] of C34  ×  Via [V_SWITCH]   @ (75.68, 145.48)
ERROR hole_clearance      constraint 0.127 mm; actual 0.000 mm            @ (75.68, 145.48)
ERROR solder_mask_bridge  front mask aperture bridges different nets      @ (75.68, 145.48)
```

A `V_SWITCH` via has been dropped dead-centre on `C34` pad 1. So the *only* thing feeding the boost
input is an unintentional via-in-pad that also bridges solder mask between two nets. A fabbed board
might well run — which is the worst outcome, because the next *Update PCB from Schematic* or zone
refill removes it with no warning.

**Root cause.** The previous review's open item 5 said, verbatim, *"(delete the existing EN →
V_SWITCH tie)"*. That same review had also recorded *"EN tied to VIN is a legal always-on config"* —
so the wire that was deleted was carrying `V_SWITCH` into **both** EN and VIN. Removing it orphaned
VIN as collateral damage.

**Fix.** In the schematic, connect `Net-(U2-VIN)` to `V_SWITCH` (this is what the review's own block
diagram shows). Re-import, route VIN properly, and delete the via at `(75.68, 145.48)`. Then confirm
DRC returns zero unconnected items.

---

## B2 — `R17` and `R18` carry a 5.11 kΩ MPN

This is the fourth instance of the value-moved-without-its-MPN failure the last review caught on
`R43`/`C42`/`R53`. Both new resistors in the GPIO21→EN circuit were copy-pasted from a 5.11 k part:

| Ref | Value | MPN carried | MPN's actual value |
|---|---|---|---|
| `R17` | 10 k | `RC0402FR-075K11L` | **5.11 k** |
| `R18` | 1 k | `RC0402FR-075K11L` | **5.11 k** |

TPS61023 datasheet §6.5 (SLVSF14B): **V_EN_H max 1.2 V**, **V_EN_L min 0.35 V / typ 0.42 / max 0.45**.

- **Built to MPN (5.11 k / 5.11 k):** GPIO21 low → EN = 3.3 × 5.11/10.22 = **1.65 V**, above
  V_EN_H max. EN is permanently high; the boost can never be commanded off. The wedged-radio
  power-cycle recovery (#412) is dead code with no symptom.
- **Built to Value (10 k / 1 k):** GPIO21 low → 3.3/11 = **0.30 V** before V_OL, ~0.32–0.39 V with
  it — straddling the 0.35 V guaranteed-low floor. Indeterminate. The previous review explicitly
  said *"Do not use 10 k"* and specified 22 k for exactly this reason.

**Fix.** `R17` = **22 kΩ** (`RC0402FR-0722KL`), `R18` = **1 kΩ** (`RC0402FR-071KL`, already stocked —
it is `R39`'s part). 22 k/1 k gives 0.143 V driven low (2.4× margin under 0.35 V) and 2.23 V if the
S3's 45 k internal pull-down is ever active (clear of the 1.2 V high threshold).

---

## B3 — No stitching vias in either exposed pad

| Pad | Size | Net | Vias inside | Nearest GND via |
|---|---|---|---|---|
| `U3` pad 57 — ESP32-S3 EPAD | 4.1 × 4.1 mm | GND | **0** | 2.37 mm |
| `U9` pad 15 — TPS63020 PowerPAD | 1.58 × 4.4 mm | GND (PGND) | **0** | 2.89 mm |

Both pads *are* solidly connected to the F.Cu GND pour (the zone uses `connect_pads thru_hole_only`,
so SMD pads get a solid fill, not a thermal relief). What is missing is the **vertical** path: there
is no connection from either exposed pad down to the `In1.Cu` or `B.Cu` ground planes anywhere under
the part.

For `U3` this is not merely thermal — the EPAD is the ESP32-S3's primary ground return, including
for the 2.4 GHz radio, and every return current is forced to travel laterally across top copper to a
via ≥2.4 mm away. For `U9` it is the PGND return of a converter whose switch current limit is 4 A.

**This board is the outlier.** The same footprint on the two boards that have been fabbed and bench-
proven is stitched:

```
base-station        U3   EPAD:  0 vias
rocket-computer     U15  EPAD:  4 vias
lora-daughterboard  U28  EPAD:  7 vias
```

**Fix.** A 3×3 array of 0.45/0.3 vias in `U3`'s EPAD and a 2×4 column in `U9`'s PowerPAD (both already
the board's standard via size, so no new drill). Keep them tented or plugged so paste does not wick.

---

## B4 — `J4`: JST XH part, JST EH footprint

| | |
|---|---|
| BOM MPN | `S3B-XH-A` — JST **XH** series, 3-circuit, side entry |
| Footprint | `Connector_JST:JST_EH_S3B-EH_1x03_P2.50mm_Horizontal` — JST **EH** series |
| Footprint holes | **Ø0.95 mm** drill, 1.70 × 1.95 mm pads |

Both series are 2.5 mm pitch, which is what hides the mismatch. The posts are not the same: the JST
XH datasheet marks the header post **□0.64 mm** — a **0.905 mm diagonal** — and JST specifies Ø1.0 mm
holes for it. A 0.95 mm drill finishes at roughly 0.85–0.90 mm after plating, i.e. at or under the
post diagonal. The connector will not seat. The EH footprint's body outline, courtyard and silk are
also wrong for an XH header, and EH and XH housings do not intermate.

**Fix the footprint, not the part.** `J4` is the external 2S flight-pack connector, and JST-XH *is*
the industry-standard LiPo balance-lead connector — XH is certainly the intended choice. Swap in a
proper JST XH 3-circuit side-entry footprint with Ø1.0 mm holes.

---

## B5 — The firmware's board profile no longer matches the board

`board_v3.h` is the build target for this hardware (`idf.py -B build_v3 -DTR_BS_BOARD=3`). It
describes a board that no longer exists:

**No fuel gauge.** `board_v3.h` asserts `EXPECT_MAX17303 = true` with a comment that reads *"BOM
assertion: the part at 0x36 IS a MAX17303"*. There is **no gauge on this PCB** — the only I²C device
is `U12` (MP2672A, 0x4B). Battery voltage now comes from the `R44`/`R46` 1 M/1 M divider into GPIO1
(`Volt_Read`). But there is **no ADC code anywhere in the base-station firmware** (`grep` for
`adc_oneshot|adc_cali|ADC_UNIT_1|adc_channel` across `projects/base_station/` returns nothing), and
`gauge_kind` has no fallback branch — it simply stays `GaugeKind::None`. **As it stands the base
station will report no battery voltage and no state of charge at all**, and the `Volt_Read` divider
has no consumer. The same applies to the new `PosADC` (GPIO8) and `MidADC` (GPIO9) 2S-pack sense
dividers — hardware with no firmware.

**No external NAND.** `HAS_EXT_NAND = true` with `FLASH_SCK/MOSI/CS/MISO = GPIO4/5/6/7`. Those four
pins are `unconnected-` on this board. The FORESEE F35SQB004G (512 MB) is gone; storage is now the
`W25Q128JVSIQ` (`U1`) on the SPI0 boot-flash bus off `VDD_SPI`. NAND init will fail
(`"SPI NAND init failed ... unsupported chip or wiring"`) and fall back to internal flash — the
`spiffs` partition, **0x1F0000 = 1.94 MB**. That is a ~250× reduction in flight-log capacity against
what `docs/architecture/base-station.md` still promises ("CSV per flight on external flash, mounted
FAT"). Worth deciding deliberately rather than discovering at the field.

Also: `partitions.csv` totals 8 MB but `U1` is 16 MB — half the flash is unallocated and could absorb
the log partition.

**Stale pin.** `LORA_ACT_PIN = -1` with a TODO, while the hardware now provides GPIO21 → `LoRa_EN`
(see B2).

**Fix.** This is a board-revision-level change; it needs its own `board_v4.h` plus an ADC battery
path, and a decision on where flight CSVs live.

---

## Should fix

### S6 — No series resistors on `LoRa_RX` / `LoRa_TX` (open item 3, not done)
Both are bare two-node nets (`J6.3 ↔ U3.41`, `J6.4 ↔ U3.40`). On the bench the daughterboard runs
from its own USB while the base station can be switched off, driving 3.3 V UART levels into unpowered
ESP32 pads. 1 k in series at `J6`, as the V9 review's H6 fix did for the camera UART. This is also a
prerequisite for the `LoRa_EN` power-cycle feature to be usable at all.

### S7 — Controlled impedance is not declared, and the stackup is not a house default
`dielectric_constraints: no`; a single `Default` netclass; no impedance-controlled class and no
`netclass_patterns`.

The RF geometry is actually *right*: the antenna feed (`Net-(U15-Feed)`, `Net-(U3-LNA_IN)`) is
0.2 mm wide on F.Cu over the 0.1 mm prepreg to the `In1.Cu` GND plane, which computes to **≈48 Ω** —
clearly deliberate. But that only holds if the fab builds the declared 0.1 / 1.24 / 0.1 mm stackup.
On a typical house 1.6 mm 4-layer stack (≈0.21 mm prepreg), the same 0.2 mm trace becomes **≈68 Ω**
and the match is gone. Declare 50 Ω single-ended on the feed and order the stackup explicitly — the
same fix the LoRa board took in `768071ca`.

### S8 — `In2.Cu` is the +3V3 plane, it is sliced by power routing, and it references all bottom-side signals
`In2.Cu` carries ~104 mm of routed power on top of being the +3V3 pour:

| Net on In2.Cu | Length | Extent |
|---|---|---|
| `VCC` | 52.8 mm | y 139.6 → 177.4 |
| `V_SWITCH` | 46.7 mm | y 145.5 → 174.8 |
| `VDD_SPI` | 6.5 mm | |
| `V_LORA` | 4.8 mm | |

Those two long runs carve slots through the middle third of the board — exactly under `U3` (y 151.8),
`U1` (152.6), `J6` (152.1) and `U2` (147.1). Meanwhile `B.Cu` carries **325.8 mm of routing over 26
nets**, all referenced to that fragmented plane 0.1 mm below, including:

- `D-` 48.7 mm and `D+` 35.1 mm — **the USB pair**
- `Net-(U22-ON)` 45.8 mm — the 100 kΩ latching-switch node
- `Volt_Read` 28.6 mm — the high-impedance battery ADC node

Every place one of those crosses a `VCC` or `V_SWITCH` slot, the return current has to detour. Top-
side signals get a solid GND reference at 0.1 mm, which is excellent; bottom-side signals get a
chopped-up +3V3 reference. Cheapest fixes, in order of preference: move `VCC`/`V_SWITCH` off `In2`
onto F.Cu/B.Cu, or swap the plane assignment so the layer adjacent to `B.Cu` is ground.

Related: `D+`/`D-` are not a declared differential pair and are mismatched **35.1 vs 48.7 mm**. USB
Serial/JTAG is full-speed (12 Mbps) so this will work, but it is free to fix while the router is open.

### S9 — Netclass defaults are illegal against the board's own rules
The `Default` netclass specifies **via 0.3 mm Ø / 0.2 mm drill**, but board setup sets
`min_via_diameter = 0.4 mm`. Every via *currently placed* is 0.45/0.3 (220) or 0.75/0.5 (2), so
nothing built today is affected — but the next person to route gets DRC errors by default. Also
`min_track_width = 0.1 mm` and `min_clearance = 0.09 mm` sit at or below standard capability at most
houses, and **34 segments are actually routed at 0.1 mm** (`SPI_CLK`, both 32 kHz crystal legs,
`Net-(U5-ISET)`, `BatteryPos`, `BatteryMid`). Raise the netclass to 0.127 mm / 0.45 mm-via and
confirm your fab's standard tier.

---

## Nits and verify-on-bench

- **`lib_footprint_mismatch` on `Y2`** — the board's copy of `XTAL_ECS-400-10-37B2-CKY-TR` differs
  from the library. Reconcile before plotting so you know which one you shipped.
- **Two dangling stub tracks** left on unconnected ESP32 pads: `unconnected-(U3-GPIO11-Pad16)` at
  (57.88, 155.25) and `unconnected-(U3-GPIO12-Pad17)` at (58.27, 155.25), 0.27/0.28 mm long. Routing
  debris; `track_dangling` is warning-only so they will not stop a plot.
- **Silk clipped by board edge** on `S1` (×2), `J2` (×2), `J4` (×2). `J4`'s and `S1`'s clipped marks
  are the ones worth restoring — they carry orientation information.
- **Silk over copper:** `BT2`'s reference designator runs across both of `D6`'s through-hole pads.
- **`+` / `-` on `B.SilkS` are not mirrored** (`nonmirrored_text_on_back_layer`). Cosmetic only —
  **the positions are correct**: `+` at (52.08, 191.73) sits by `BT2.P`/`VCC` (y 184.90) and `-` at
  (53.64, 110.47) by `BT2.N`/`/Battery/B-` (y 112.60), which matches the `/Battery/B-` pour at
  y 110.1–115.9 and the `VCC` pour at y 178.9–187.3. Add the mirror flag.
- **Inductor saturation vs IC current limit.** `L8`/`L9` are `VLS3012CX-2R2M-1`: 2.2 µH, DCR 74 mΩ,
  **Isat ≈ 2.83 A**. The *values* are explicitly sanctioned — TPS61023 works with 0.37–2.9 µH
  (§8.2.2.2), and TI recommends 1–1.5 µH for the TPS63020 "or 2.2 µH at lower load current", which is
  this board. But the inductor saturates **below** both ICs' limits (TPS61023 valley limit 2.7 A min /
  3.7 A typ; TPS63020 switch limit 4 A typ). Normal operation is nowhere near this; it is a
  startup/fault-robustness gap, not a functional one. Note it and move on, or step up to a 3–4 A part
  in the same 3012 footprint.
- **MP2672A input capacitor placement.** `C48` (22 µF) is **4.41 mm** from `U12.1` (IN) on a switcher
  whose default f_SW is 1.2 MHz; `C54` is 4.10 mm from `BATT`. Everything else on the board decouples
  within 1.2–2.2 mm, so these two stand out. Pull `C48` in if the router allows.
- **DRC severities set to `ignore`:** `missing_courtyard`, `pth_inside_courtyard`,
  `npth_inside_courtyard`, `footprint_type_mismatch`, `track_not_centered_on_via`. The first three
  are KiCad defaults; `footprint_type_mismatch` is the one worth turning back on — it is the check
  that would have flagged **B4**.
- **`S1` support-peg NPTHs land under the cell.** The PB400's two Ø1.9 mm pegs sit at (59.03, 134.45)
  and (59.03, 138.85), which is inside the 18650's footprint on the back side (holder clips at
  y 112.6 and 184.9, cell body x 49.8–67.8). Check peg protrusion against the 1.6 mm board before
  fab — a peg standing proud under a Li-ion wrapper is not something to discover later.
- **`U15` dummy pads.** Molex calls pads 1–3 *"dummy SMD pads to be attached to the PCB for strong
  mechanical bonding"*; here they sit on isolated `unconnected-` nets, leaving three floating copper
  islands under the antenna. Electrically irrelevant at 2.4 GHz (0.86 mm ≪ λ/4 = 31 mm) but tying
  them to GND is free and removes the islands.
- **40 MHz crystal loading.** `C3`/`C6` = 12 pF against the `ECS-400-10-37B2` 10 pF C_L computes to
  ≈ +30 ppm at a nominal 2.5 pF stray, versus Espressif's ±10 ppm target. **All three TinkerRocket
  boards use the same 12 pF and BLE works on two of them**, so real stray is evidently higher than
  nominal — treat this as measure-and-trim, not a change.
- **`J6` mounting tabs** carry no net. Tying them to GND improves retention and gives the shell an
  ESD path.

---

## Verified correct — do not re-review

These are the things that *look* wrong on this board and are not. Each was checked against a primary
source, and several cost real time to clear.

- **The antenna needs no keepout, and the absence of one is correct.** `U15` is a Molex 47948-0001,
  an **on-ground** LDS antenna: the datasheet states **"Ground Clearance: None needed"** and
  *"No removal of ground layers from beneath the antenna is needed"*, and explicitly sanctions
  components on the reverse side — which is where the 18650 sits. The full-board GND pour on F.Cu /
  In1.Cu / B.Cu under it is the intended configuration. Molex's reference platform is a 100 × 40 mm
  board with the antenna at a corner; this board is 30.3 × 90.5 mm with the antenna mid-edge, so
  expect the pattern and the match to differ from the datasheet plots — the `C25`/`L4`/`L7`/`C26`
  network is the right place to trim, and `L4` (4.3 nH shunt) ‖ `C25` (1.5 pF) nets to ≈0.51 pF
  while still giving the feed a DC path to ground for ESD. Sound design.
- **Solder mask and paste apertures are all present.** `U3`'s and `U9`'s pads list only
  `("F.Cu" "F.Paste")` in the pad definitions, which reads like a missing mask opening — but both
  footprints carry per-pad `F.Mask` polygons as footprint *graphics* (56 on `IC_ESP32-S3`, 15 on
  `IC_TPS63021DSJR`). Confirmed against the exported `F.Mask` gerber: every pad has its aperture.
- **`U12` has no exposed pad, and the footprint correctly omits one.** The MP2672A package drawing
  (datasheet p.5) shows QFN-18 2×3 mm with 18 perimeter pads only. Pin-for-pin match with the
  schematic: 1 IN, 2 SW, 3 BST, 4 VCC, 5 ISET, 6 AGND, 7 VLIM, 8 NTC, 9 MID, 10 BATT, 11 SYS, 12 SW,
  13 PGND, 14 SCL, 15 SDA, 16 STAT, 17 CV, 18 ACOK.
- **`CV` tied to `VCC` selects host-control mode, and the unattended case is safe.** This is worth
  stating explicitly because the failure mode would be a 2S overcharge. With `CV` → `VCC` the pack
  full-voltage comes from the I²C register, and the ESP32 is downstream of the power switch — so with
  `S1` off and USB plugged in, the charger runs on `-0000` power-on defaults with nothing to service
  the 40 s watchdog. Those defaults are **REG00H = 0011 1000 → V_BATT_REG[2:0] = 001 = 8.4 V** with
  **CHG_CONFIG = 1 (charging enabled)**, and the watchdog reverts to exactly those same values. 8.4 V
  is correct for 2S. No change needed. (`REG02H = 1001 0101` → 1.2 MHz, 40 s WD, 20 h safety timer,
  boost enabled.)
- **`ESP32-S3RH2` is the 2 MB *quad* PSRAM variant**, so GPIO33–37 stay free — `SCL_SENS`/`SDA_SENS`
  on GPIO33/34 and the LoRa UART on GPIO35/36 are safe. `SPICS1` (pad 28) is correctly left
  unrouted; it is the in-package PSRAM chip select. Only octal variants (R8/R16V) would conflict.
- **The 22 Ω USB series resistors `R3`/`R4` are correct.** Espressif's ESP32-S3 schematic checklist:
  *"It is recommended to reserve series resistors (initial value can be 22/33 Ω)."*
- **`CHIP_PU` RC is exactly Espressif's recommendation** — `R1` 10 k + `C8` 1 µF.
- **`VDD_SPI` decoupling meets the checklist** (`C1` 100 nF + `C9` 100 nF + `C10` 10 µF against the
  required *"extra 0.1 µF and 1 µF"*). Note the standing caveat that at 3.3 V, `VDD_SPI` is fed from
  `VDD3P3_RTC` through the internal **14 Ω** R_SPI, and RH2 puts the in-package PSRAM on that rail
  alongside `U1` — the same budget question already tracked for the rocket computer. `C10` is a 10 µF
  0402 6.3 V part that derates to ~3 µF at 3.3 V; if you want margin here, the 22 µF 0805 already on
  this board (`CL21A226MOQNNNE`, 10 placements) is the free upgrade.
- **Decoupling placement is good.** Every IC supply pin has a same-net cap within 1.2–2.2 mm except
  the two MP2672A cases noted above: `U3.2/3` → `C16` 1.73 mm, `U3.20` → `C12` 1.42 mm,
  `U3.46` → `C14` 1.19 mm, `U3.55/56` → `C11` 1.29/1.19 mm, `U3.29` → `C9` 1.19 mm,
  `U1.8` → `C1` 1.55 mm, `U2.3` → `C34` 2.14 mm, `U9.10` → `C59` 1.66 mm, `U22.1` → `C42` 1.66 mm.
- **All four remaining open items from the power-switch review are implemented and verified in the
  netlist:** `C42` is now 22 µF 0805 (item 1); `R44`'s divider top moved to `V_SWITCH` (item 2); the
  orphan `M_*` labels on GPIO4–7 are gone (item 4); `V_LORA` now carries 2 × 22 µF, `C40`+`C41`
  (item 6). Item 5 is implemented but broken — see **B1** and **B2**.
- **`U9` VINA + EN now reach `V_SWITCH`** — the fix from the last review's finding 1 is present and
  correct in both schematic and PCB.
- **DW01A / FS8205A protection topology is correct**: `B-` = cell negative + all three FET sources +
  `U14.6`; `PROT_DRAIN` is the isolated common-drain node; `OD`→G1, `OC`→G2; `VM` senses P− through
  `R57` 2 k; `TD` correctly left floating.
- **USB-C**: `R14`/`R15` 5.11 k CC pulldowns present; all four shield tabs and both GND pin pairs on
  GND; `SBU1`/`SBU2` correctly unused.

---

## Antenna — findings from Molex AS-479480001 Rev G (added 2026-08-04)

The application specification was obtained and read in full. It settles the dummy-pad question and
surfaces one finding that matters more than anything else left on the board.

### A1 — The matching network does not match Molex's, and the shunt element has the wrong sign

**§6.0** specifies an "L" network, with a layout figure and a component table:

```
ANT ──┬────── C2 = 0 Ω (series) ────── RF Signal
      │
   C1 = 4.3 nH   (Murata LQG15HS4N3B02)
      │
     GND                                 C3 = NA (not fitted)
```

The board, with **all four positions fitted, none DNP**:

```
U15.4 ──┬── L4  4.3 nH ── GND        <- matches Molex C1
        ├── C25 1.5 pF ── GND        <- NOT in Molex's design
        └── L7  2.7 nH (series) ──┬── U3.1 (LNA_IN)      <- Molex specifies 0 Ω here
                                  └── C26 1.5 pF ── GND  <- Molex specifies NA here
```

At 2.44 GHz:

| | |
|---|---|
| Molex `C1`, 4.3 nH alone | **+j65.9 Ω — inductive** |
| This board, `L4` ∥ `C25` | **−j127.8 Ω — capacitive** (≈0.51 pF equivalent) |

Fitting `C25` alongside `L4` does not merely shift the value — it **inverts the sign of the shunt
element** relative to Molex's design intent. Add a 2.7 nH series where Molex calls for 0 Ω, plus a
1.5 pF shunt where Molex fits nothing, and the network is materially different from the
characterised one. Nothing in DRC can see this.

**RESOLVED 2026-08-04 — shipping with the inductor only.** `C25`, `C26` and `L7` were removed
(118 → 115 footprints). The network is now exactly Molex's §6.0 recommendation, with `C2` = 0 Ω
realised as a direct trace:

```
U15.4 (ANT) ──┬────── direct trace ────── U3.1 (LNA_IN)
              │
           L4 = 4.3 nH        -> +j65.9 Ω at 2.44 GHz, inductive, correct sign
              │
             GND
```

(`L4`'s `LQW15AN4N3C00D` vs Molex's `LQG15HS4N3B02` is fine — same 4.3 nH, wire-wound, higher Q.)

**Accepted trade, recorded deliberately.** Deleting the footprints rather than fitting them DNP means
this build has **no tuning provision and no VNA break point**, and Molex is explicit that validation
is the user's responsibility (§1.0) and that off-tune from surrounding metal "can be compensated
through matching" (§4.2). This board differs from Molex's reference in every dimension that loads an
antenna — 1.6 mm vs 0.8 mm, 4-layer vs 2-layer, 30 × 90 vs 100 × 40 mm, mid-edge vs corner, plus an
18650 can 5.8 mm behind it — so some off-tune is expected. The decision is to accept that on this
prototype and fold the measured result into the next iteration.

**Carry into the next revision:** measure return loss and efficiency on an assembled unit **with the
cell installed**, and restore a shunt/series/shunt provision (Molex's `C1`/`C2`/`C3` positions) so
the next board can be tuned rather than characterised-and-respun. If the measured match is poor and
this revision needs rescuing, `L4`'s own pads are the only place to intervene — a different shunt
value is the single degree of freedom available.

### A2 — The "fixing pads" have no specified net; leave them isolated

**§4.1**: *"There are one feeding pad and **three fixing pads**."* Molex calls them **fixing** pads.
Across all 17 pages the document **never specifies their net** — it neither requires grounding nor
requires isolation.

Given this is an LDS part whose plating visibly wraps the moulded body, and Molex nowhere states the
fixing pads are internally isolated from the radiator, grounding them is an unforced risk on the one
component whose behaviour is hardest to verify without a VNA. Three 0.86 mm squares are electrically
invisible at λ/4 = 31 mm, so isolation costs nothing. **Leave `U15` pads 1–3 unconnected.**

### A3 — The 18650 clears Molex's 5 mm plane rule, but only just

**§7.1, §7.2 and §7.3 each state:** *"the minimum distance between antenna and plane ground is
recommended to be **5mm**."* The 18650's steel can is exactly such a plane, parallel to the board and
behind the antenna.

Measured: the cell axis runs along y at x = 58.84 with a 9 mm radius. The antenna centre sits at
x = 49.71 — **9.13 mm from the axis, so just outside the can's silhouette entirely**. At the
antenna's inner edge (x = 51.21) the can surface is **5.82 mm below the antenna base**, plus holder
standoff.

So it passes, with ~0.8 mm of margin. But Molex's own data (Figures 7.1.1/7.1.2) shows that at 5 mm
the resonance shifts and efficiency falls from ~78 % to ~56–65 %. **Tune the match with the cell
installed, not on a bare board.**

### A4 — Reference conditions differ; expect to retune regardless

Molex's reference is a **100 × 40 × 0.8 mm double-sided** board with the antenna **at a corner**
(§4.1). This board is 30.3 × 90.5 × 1.6 mm, 4-layer, antenna at the **middle of a long edge**. §4.2
warns explicitly: *"the frequency resonant might be off-tune due to the loading of surrounding
components especially metal plane. This off-tune can be compensated through matching"* and *"The peak
gain will be degraded by 1 to 2 dBi in the actual implementation."*

### A5 — Verified good, and one assembly note

- **Ground stitching along the feed** matches the density of Molex's reference layout: 4 GND vias
  within 1.0 mm of the feed, 7 within 1.5 mm, 13 within 2.5 mm.
- **§5.0 assembly:** *"For mechanically challenging applications Molex recommends using surface mount
  adhesive (e.g. Loctite 3611) before reflow soldering process, to ensure increased mechanical
  retention on the PCB."* Worth adding to the assembly notes — it is a 0.03 g part on four small
  pads on a field-carried unit. Reflow profile: peak 255–260 °C, ramp-up ≤3 °C/s, ramp-down
  ≤6 °C/s, 60–150 s above 217 °C; recommended paste ALPHA CAP-390 SAC305.

---

## Residual list as of 2026-08-04

Nothing here blocks the gerbers. Ranked by what actually matters.

**1. `In2.Cu` is a +3V3 plane cut by 104 mm of power routing, and it references every bottom-side
signal** (S8, unchanged). `VCC` 52.8 mm + `V_SWITCH` 46.8 mm + `VDD_SPI` 6.5 + `V_LORA` 4.8 slice the
pour through the middle third of the board, while `B.Cu` carries **332 mm of routing over 27 nets** —
including `D+` 35.1 mm and `D−` 48.7 mm — all referenced to that fragmented plane 0.1 mm below.
Cheapest fix: move `VCC`/`V_SWITCH` off In2, or swap the plane assignment so the layer adjacent to
B.Cu is ground.

**2. `VCC` on `In2.Cu` is now 0.5 oz copper.** 52.8 mm at 0.5 mm width → **~120 mΩ, ~120 mV/A**.
Fixing the stackup surfaced this: JLC builds 4-layer with **0.5 oz inner** by default, so the old
file's 0.035 mm inner layers were fiction. Widen that run, or confirm it is not a series path for
charge current.

**3. Re-enable `footprint_type_mismatch` in DRC severities.** Still `ignore`, along with
`missing_courtyard`, `pth/npth_inside_courtyard`, `track_not_centered_on_via`. The first is the check
that would have caught the `J4` XH-part-in-EH-footprint mismatch in the first place.

**4. `lib_footprint_mismatch` on `Y2`** — the board's copy of `XTAL_ECS-400-10-37B2-CKY-TR` differs
from the library. Reconcile before plotting so you know which one you shipped.

**5. Silkscreen sweep** — 24 `silk_overlap`, 6 `silk_edge_clearance`, 2 `silk_over_copper`. The
clipped ones on `J4` and `S1` carry orientation information; `BT2`'s reference designator runs across
both of `D6`'s through-hole pads. Also the `+`/`-` marks on B.SilkS lack the mirror flag (positions
are correct — `+` by `BT2.P`/`VCC`, `-` by `BT2.N`/`B−`).

**6. Two dangling stubs** on `unconnected-(U3-GPIO11-Pad16)` and `-GPIO12-Pad17`, 0.27/0.28 mm at
(57.88, 155.25) and (58.27, 155.25). Plus 7 sub-0.1 mm `V_SWITCH` junction stubs left over from the
VIN rework — DRC reports no dangling ends on those, so they are cosmetic.

**7. `S1`'s two Ø1.9 mm support-peg NPTHs at (59.03, 134.45) and (59.03, 138.85) sit under the
18650.** Still unverified — check peg protrusion against the 1.6 mm board in the 3D viewer. A peg
standing proud under a Li-ion wrapper is not something to find out later.

**8. `MP2672A` input cap placement** — `C48` 4.41 mm from `U12` pin 1 (IN), `C54` 4.10 mm from pin 10
(BATT), on a switcher whose default f_SW is 1.2 MHz. Everything else on the board decouples within
1.2–2.2 mm.

**9. `U15` dummy pads 1–3 are on isolated nets.** Molex calls them mechanical bonding pads; tying
them to GND costs nothing and removes three floating islands under the antenna.

**10. Inductor saturation** — `L8`/`L9` `VLS3012CX-2R2M-1` Isat ≈ 2.83 A against the TPS63020's 4 A
switch limit and the TPS61023's 2.7 A min valley limit. In-spec for normal operation; a
fault/startup robustness gap only.

**11. `min_hole_clearance` left at 0.127 mm** by decision — JLC publishes 0.20 mm for via-hole-to-
track, but boards have been manufactured at these clearances. Recorded, not actioned.

---

## Suggested order of work (as originally written)

1. **B1** — reconnect `Net-(U2-VIN)` to `V_SWITCH`, delete the offending via, re-run DRC to zero.
2. **B2** — `R17` → 22 k, `R18` → 1 k, both with matching MPNs. Then re-run the MPN-keyed BOM audit;
   the last review's exact-MPN pass is what catches this class, and it needs re-running after any
   value edit.
3. **B4** — swap `J4` to a JST XH footprint (Ø1.0 mm holes).
4. **S6** — add the two 1 k series resistors at `J6` while the schematic is open.
5. **B3** — via arrays in both exposed pads.
6. **S7 / S8 / S9** — declare impedance, get `VCC`/`V_SWITCH` off `In2.Cu`, fix the netclass.
7. **B5** — firmware `board_v4.h`, an ADC battery path, and a decision on log storage. This one does
   not gate the gerbers, but it gates the board being useful when it arrives.
