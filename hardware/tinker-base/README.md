# Tinker-Base

> **Tinker-Base** is the product name, from 2026-09-24, of the board this repo
> called `base-station-mini` — the one base station in the product line, for
> both the Tinker-Mantis and the Tinker-Beetle. The board it forked from, the
> full `base-station`, is now in [`../legacy/`](../legacy/base-station/).

A reduced-feature ground station. **"Mini" means fewer features, not a smaller
board** — the outline is still base-station's 30.27 × 90.50 mm. Nothing here is
trying to shrink; it is trying to do less.

Currently rev **V2**, in work. V1 was ordered on 2026-09-29
(`tinker-base-v1.0.0`) and built; see *Status*.

## What it drops, and what it gains

Against [`base-station/`](../legacy/base-station/):

- **The LoRa radio is on-board.** `U16`, an E220-900MM22S on SPI, replaces the
  UART header that fed an external [`lora-daughterboard/`](../lora-daughterboard/).
  That removes the daughterboard, its connector, and the 5 V `V_LORA` boost that
  supplied it — the module runs straight off +3V3.
- **No external charger sheet.** Dropped from the root, along with the
  `PosADC`/`MidADC` sense lines it fed. `external_charger.kicad_sch` was deleted
  outright on 2026-08-12 rather than left orphaned on disk.
- **No sensor I²C.** `SDA_SENS`/`SCL_SENS` are gone, which is what freed GPIO33
  and GPIO34 for the radio.
- **`+3V3` comes from the adjustable TPS63020** and its feedback divider,
  `R13` 1 MΩ over `R16` 180 kΩ for 3.28 V. base-station moved to the
  fixed-output TPS63021 at V6; this board went back to the adjustable part on
  2026-08-16. The two share a footprint, and the divider is what tells them
  apart. With the divider, the fixed part would drive about 21 V. V2 replaces
  both with a fixed 3.3 V TPS62152 buck; see *Status*.

## Parts and patterns taken from the Beetle and Mantis

On 2026-09-27 the ESP32-S3 sheet took on the newer parts the Beetle and Mantis
carry, plus the decoupling Espressif's hardware checklist asks for. The layout
followed on 2026-09-28; see *Status*.

- **BLE antenna.** `U15` is now the edge-mounted 2.4 GHz loop chip antenna
  (1.0 × 0.5 mm) that both flight computers use, with the same pi match: `C7`
  (5.1 pF) in series, `L4` (2.2 nH) to ground on the radio side, and `C5`, an
  unfitted shunt on the antenna side kept for tuning. `L4` was the old antenna's
  4.3 nH shunt and keeps its footprint. The old antenna worked on top of the
  ground plane; this one needs a keep-out.
  - It sits in a ground band along the board edge (copper from 0.3 to 0.75 mm in
    from the edge), with its ground pads in the band.
  - Inboard of the band, a window 4.6 mm along the edge and 3.0 mm deep is clear
    of copper on every layer, including under the feed pads. Only F.Cu has a notch
    in it, for the feed trace. That is Abracon's evaluation-board layout (rev A
    p.4), and the Beetle's.
  - Ground vias stitch the far side and both ends.
  - The first layout, on 2026-09-28, cut the notch on every layer and left planes
    under the feed pads. The design review caught it (L2).
- **Boot flash** (V1 only; V2 removes it, see *Status*). `U1` is the Beetle and
  Mantis's 16 MB quad-SPI NOR in a 24-ball WLCSP. It is fed from `+3V3`, with `C1` (100 nF) beside it, instead of
  from the S3's `VDD_SPI` pin. The full base-station's V5 showed why.
  `VDD_SPI` sits behind a switch inside the S3. With an in-package-PSRAM S3 on
  the same pin, the flash's supply budget fails, and every other board already
  feeds its NOR from the 3.3 V rail. The balls are on a 0.5 mm pitch, but every
  signal is in the two middle columns. The traces escape through the 0.55 mm
  gaps beside them at 0.1 mm track and space, below this board's 0.2 mm
  default. Keep vias out of the ball pads; JLCPCB queried exactly that on the
  Beetle.
- **`VDD_SPI`** keeps `C9` (100 nF) and `C10` (10 µF). Espressif asks for
  1 µF here, and the Mantis fits that. But the pin is fed through the S3's
  internal switch (about 14 Ω), which makes it the board's highest-impedance
  rail, so the extra bulk helps. Espressif's reasons for a small cap don't
  apply here: the rail still settles well before `CHIP_PU` releases, and the
  board never light-sleeps. The LoRa daughterboard's review keeps 10 µF for the
  same reason ([finding 4a](../lora-daughterboard/prefab-review-2026-07-30.md)).
- **`VDD3P3`** (pins 2 and 3, behind `L3`) gains `C24`, 10 µF, beside `C16`, as
  on the Mantis.
- **`GPIO0`** gains `R5`, 10 kΩ to `+3V3`, on top of the S3's weak internal
  pull-up, as on the Mantis.
- **`L_RXEN`** gains `R6`, 100 kΩ to ground. GPIO35 has no reset pull, so the
  radio's receive enable used to float through every reset. Now it is held off
  until firmware drives it, which is the Beetle's fix for the same line.

From the 2026-09-28 [design review](design-review-2026-09-28.md):

- **LoRa antenna port** gains `L5`, 330 nH from the SMA line to ground. It is the
  protective shunt Ebyte's manual asks for (§5.1). `J8` moved 1.14 mm to the
  edge, so the connector's centre pin now covers its whole pad.
- **`U9`** gains `C25`, 100 nF from `V_SWITCH` to the ground pin beside the
  input pins. That gives the input a short high-frequency loop. The bulk input
  cap can't provide one, because its ground pad is walled off by the
  switch-node pours.
- **`U9`'s output** gains `C26`, 100 nF from `+3V3` to ground beside the output
  pins, for the same reason on the output side: the 22 µF caps' ground pads
  return around the switch-node pour. `U9`'s FB line runs under its body,
  between the pads. Its ground pad shares a via with `U9` pin 2.
- **Charge current:** `R52` 680 Ω → **1.2 kΩ**, about 0.45 A (0.40–0.50 A)
  instead of 0.79 A. The old value sat at the charger's 0.8 A limit, and it made
  the charger heat the board past its own thermistor's trip point; see the next
  section.

## The battery thermistor is board-mounted, and that is a compromise

`TH1` feeds the BQ21040's `TS` pin, which gates charging on temperature. It is a
10 kΩ-at-25 °C NTC, B25/85 = 3435 K, ±1 % on both, in an 0603 chip package, on
the component side at the far end of the board from the charger. `R53` (210 kΩ,
`TS` to ground) is TI's TTDM-defeat resistor and is unchanged — with the same
resistance-temperature curve the charger's temperature window does not move.

Until 2026-08-27 `TH1` was a two-pin 2.54 mm land for a **leaded** thermistor of
that same curve, meant to be taped to the 18650 so `TS` read *cell* temperature.
The cell holder leaves no room for a part under the cell, so the sensor moved
onto the board.

**What that costs.** The NTC now reads the ground plane, not the cell, and its
self-heat bias runs one way: a warm board makes the charger believe the pack is
warmer than it is.

- On the hot side that is conservative — charging is inhibited early.
- On the **cold** side it is not. Board self-heat can mask a genuinely cold cell
  and allow a sub-0 °C charge, which is the failure the `TS` pin exists to
  prevent.

Placement mitigates this; it does not remove it. `TH1` sits 25 mm from the
charger — the dominant heat source while charging, dissipating up to about
0.7 W as a linear regulator at the ~0.45 A `R52` programs (up to 1.6 W at the
original 0.79 A) — 27 mm from the
buck-boost and 15 mm from the ESP32-S3. Weighted by dissipation over distance
that is about 37 % less coupling than the old land, which sat 8.6 mm from the
buck-boost and 13.5 mm from the charger. A solid inner ground plane keeps the
board close to isothermal, so what remains is a board-wide offset rather than a
local hot spot.

Everywhere on the board that is further from the heat needs a sensor trace
threaded through the ESP32-S3 fanout or the chip antenna's ground-via fence;
neither is worth it for a few degrees. The route as built is 19 mm on `F.Cu`
with no vias.

**Charging heat reaches `TH1` anyway.** The solid planes spread the charger's
heat, so `TH1` reads roughly the board's mean temperature: about +19 K per watt
of charger dissipation in still air. The design review modelled this (±25 %).
- **At the original 0.79 A:** the charger folds back to about 0.9 W, which trips
  the 40.6 °C hot threshold from about 25 °C ambient.
- **At 1.2 kΩ:** it stays out of thermal regulation, and `TH1` rises 11–14 K.
- **With the base station on:** its own ~0.3 W adds about 6 K.
- **No power path:** the load hangs on the cell node, so while charging is
  suspended it drains the cell even with USB plugged in.

Two consequences:
- USB charges the cell; it does not run the base station. Charge with `S1`
  off.
- The charger's 10-hour safety timer ends a charge that never terminates,
  because a running board keeps the current above the termination threshold.
  Only unplugging USB re-arms it.

**Open at first article:** with the board charging, compare the `TS` node
against a reference probe on the cell at room temperature, near 0 °C and at
35 °C, with `S1` off and on, and confirm the offset is small enough to accept. If it is not, the fix is a wired
probe on a connector, not a different chip.

## The radio pinout is deliberately identical to lora-daughterboard

All eight ESP32↔E220 signals land on the same GPIOs as the fabbed
`lora-daughterboard`, so the `radio_board` firmware pin map applies unchanged:

| net | E220 | GPIO |
|---|---|---|
| `L_SCK` | 15 SCK | 17 |
| `L_CS` | 14 NSS | 18 |
| `L_MOSI` | 13 MOSI | 21 |
| `L_MISO` | 12 MISO | 33 |
| `L_BUSY` | 11 BUSY | 34 |
| `L_RXEN` | 10 RXEN | 35 |
| `L_RST` | 3 NRST | 38 |
| `L_DI01` | 20 DIO1 | 2 |

`L_DI02` is a module-local loop from DIO2 to TXEN, so the SX1262 drives its own
RF switch and costs no GPIO. DIO3 is unconnected. Both match the daughterboard.

## Provenance

The design started as a copy of `base-station` and diverges from there. There is
**no ongoing relationship** between them — nothing is shared, nothing tracks
upstream, and a change to one is not expected to reach the other. Treat anything
still inherited as a first draft to be justified on its own terms.

Two things were deliberately left behind at the fork:

- **`base-station`'s design reviews** (`prefab-review-2026-08-02.md`,
  `power-switch-review-2026-08-02.md`) — records of a review of *that* board.
  Copying them here would assert this board has been reviewed when it has not.
  They remain the best reading on the inherited power architecture; read them
  there.
- **`outputs.kicad_sch`**, an empty sheet in `legacy/base-station/` referenced by
  nothing. See the leftovers note in [`../README.md`](../README.md).

## Status

### In work: in-package flash, a log flash, the USB-C, the LED resistors and the 3.3 V buck (2026-10-29/30)

This is V2. The title blocks on all four sheets and on the PCB say V2. The
PCB caught up with the schematic on 2026-10-30; see *PCB* below.

- **`U3` is an ESP32-S3FH4R2**, with 4 MB flash and 2 MB PSRAM in the package.
  The Beetle V2, the Mantis V11 out computer and the LoRa daughterboard V5 moved
  to the same part on the same day. The symbol and the QFN-56 footprint are
  unchanged.
- **`U1`, the WLCSP boot NOR, is gone,** along with its 100 nF `C1` and the
  `SPI_*` nets. Pads 30–35 carry no-connect flags. `VDD_SPI` keeps `C9`/`C10`,
  because it now feeds the in-package flash and PSRAM.
- **The log flash is `U23`,** a GigaDevice GD25Q128ESIGR (16 MB NOR, SOP-8
  208 mil, LCSC C2928891). It sits on the S3's general-purpose SPI at GPIO4–7,
  pads that V1 left unconnected. The pins and net names match the NAND bus on
  the V2 base station (`board_v2.h`, SPI3), so that firmware pin map carries
  over:

  | net | `U23` | GPIO |
  |---|---|---|
  | `M_SCK` | 6 CLK | 4 |
  | `M_MOSI` | 5 DI | 5 |
  | `M_FLASH_CS` | 1 /CS | 6 |
  | `M_MISO` | 2 DO | 7 |

  `R58`–`R60` (10 kΩ) pull `/CS`, `/WP` and `/HOLD` up to `+3V3`. `/CS` keeps
  the chip deselected while GPIO6 floats through reset. `/WP` and `/HOLD` stay
  high, since the bus is 1-bit SPI. `C89` (100 nF) decouples VCC on `+3V3`. The
  symbol and footprint are the ones the full base station used for its
  GD25Q128ESIG (`Custom:W25Q128JVSIQ`, `Footprints:SOIC127P790X216-8N`).
- **Why a separate chip:** each received packet logs as 75 bytes, about
  1.6 MB per hour with three rockets transmitting. The FH4R2's 4 MB leaves
  about 1.9 MB after two OTA slots, roughly one hour. All 16 MB of `U23` holds
  logs, which is about 8 hours, more than V1's 9.94 MB spiffs partition. At
  JLC (2026-10-29, qty 100) the FH4R2 is $2.52 and the GD25Q128E is $0.97.
  JLC stocks neither the RH2 nor the W25Q128JVYIQ.
- **`J2` is the GCT USB4110-GF-A** (2026-10-30), the right-angle USB-C on the
  Mantis and the LoRa daughterboard, in place of HRO's TYPE-C-31-M-12. It uses
  the same `Custom:USB4110GFA` symbol, so the nets do not change. Its shell is
  four SMT pads plus two 0.65 mm non-plated pegs, so the hand-soldered shell
  legs and plated slots go away. The land is bigger than HRO's (courtyard
  12.9 × 8.4 mm against 10.6 × 9.4 mm), and its origin sits at the rear pad
  row rather than the body center. JLC, 2026-10-30, qty 100: $0.88, C5143397,
  3,326 in stock (HRO's was $0.15).
- **The LED resistors `R41` and `R54` are 330 Ω** (Yageo RC0402FR-07330RL,
  C105875), down from 150 Ω. No board in the fleet uses a value between 150 Ω
  and 1 kΩ, and these two were the only 150 Ω parts, so the BOM keeps the same
  line count. At a typical 2.0 V drop, `D6` (green, from `+3V3`) runs about
  4 mA instead of about 9 mA. `D9` (blue, from the battery through `U5`'s
  `CHG`) runs about 2–4 mA at a 3.0 V drop instead of 5–8 mA.
- **`U9` is a TPS62152RGTR** (2026-10-30), the fixed 3.3 V, 1 A buck the
  Beetle and Mantis use (LCSC C2070611), in place of the TPS63020 buck-boost.
  `R13` and `R16` are gone; nothing sets the output any more.
  - **Why:** the first V1 board, on the bench on 2026-10-30, ran `+3V3` at
    6.4 V from a 4.05 V cell. Its boot flash died during the first flash and the
    ESP32-S3 stopped enumerating minutes later. V1's `U9` symbol carried a stale
    SnapEDA field, `MP = TPS63021DSJR`, beside `MPN = TPS63020DSJR`, and JLC
    stocks both. The fixed TPS63021 on this divider runs away (see above).
    `U9`'s marking has not been read to confirm it.
  - **Why a buck is enough:** `+3V3` now carries only the ESP32-S3 and the
    E220, about 55–90 mA running and roughly 250–300 mA at peaks. The TPS6215x
    is specified from 3 V, keeps running to its 2.7 V UVLO, and goes to 100 %
    duty as the cell nears 3.3 V. With its 120 mΩ high-side switch (at 3 V) and
    `L9`'s 74 mΩ, it holds 3.3 V down to a cell of about 3.36 V at 300 mA
    (SLVSAL5E eq. 3). Below that, `+3V3` follows the cell, and the S3's 3.0 V
    minimum ends the run near the firmware's 0 % point (3.00 V). The
    buck-boost used the cell down to the protection cutoff; the difference is
    about the last 5–10 % of an 18650.
  - **Wiring:** as the Beetle's `U18`, except DEF. PVIN, AVIN and EN are on
    `V_SWITCH`, after `U22`. SW drives `L9`, the same VLS3012CX-2R2M-1. VOS
    and FSW are on `+3V3` (FSW high is about 1.25 MHz). FB, DEF, AGND, PGND and
    the pad go to GND. DEF low gives 3.3 V; the Beetle ties DEF to its output
    for +5 % because its TPS61094 follows the buck. PG is unused and flagged
    no-connect. `C90` (10 nF, CL05B103KB5NNNC) sets the soft start, as the
    Beetle's `C44`. The input caps (`C59`, `C29`, `C23`, `C25`) and output caps
    (`C58`, `C60`–`C62`, `C26`) stay. The footprint is the Beetle's
    `Footprints:QFN50P300X300X100-17N`.
  - **A wrong variant still hurts, less:** a TPS62150 (adjustable) or TPS62153
    (5 V) in this land, with FB grounded, runs at 100 % duty and puts the cell,
    up to 4.2 V, on `+3V3`. That is above the S3's 3.6 V maximum. The fab notes
    ask for the part by its LCSC code.
  - **Checked:** in the netlist only `U9`, `R13`, `R16` and the new `C90`
    change; every other pin keeps its net. ERC loses V1's two `U9` errors (the
    tied VOUT outputs and the open PG) and its `lib_symbol_mismatch`, and gains
    only off-grid wire-end warnings. `U9` sits on a half-grid point, as before.
- **The BLE antenna is the Molex 47948-0001 again** (2026-11-25, `U15`,
  `0479480001`), as on the Beetle V2, in place of the Abracon AANI-CH-0070.
  Molex's "Ground Clearance: None needed" is sales copy. The drawing
  (PS-479480001 §9.3) wants copper-free board on the antenna's own layer,
  while inner and opposite-side ground may stay. So the AANI's two all-layer
  keep-outs are gone. In their place, an F.Cu-only rule area (no pour;
  tracks and vias allowed) covers `U15`'s courtyard out to the board edge,
  (47.45, 147.77) to (51.454, 151.48). That matches the B.Cu area under the
  Beetle V2's `U14`. In1 (GND), In2 (+3V3) and B.Cu (GND) stay solid under
  the antenna.
  - **Match:** Molex's L network on the existing pi footprints. `L4` is
    4.3 nH (`LQW15AN4N3C00D`, the old base station's and the V9's part) to
    ground. `C7` (5.1 pF) becomes `R61`, a 0 Ω link (`RC0402JR-070RL`), with
    the same symbol uuid. `C5` stays unfitted as the antenna-side tuning slot.
  - **Wiring:** fixing pads 1-3 are no-connect, and the ground symbol on the
    AANI's ground pins is gone. The feed net is now `Net-(U15-Feed)`, and the
    RF net-class pattern that was `Net-(C5-Pad1)` follows it.
  - **Placement:** the old base station's spot, nudged 0.12 mm left and
    0.2 mm down to (49.6, 149.625) to clear `C5`'s and `R61`'s courtyards. It
    sits at rotation 0, with fixing pads 1 and 2 on the left edge and feed
    pad 4 inboard, facing `R61`; the body is 0.6 mm from the edge. A straight
    0.36 mm feed runs up to `R61`. The ground-via fence at x = 51.4 stays.
  - **Checked:** in the netlist, only the feed net, `U15`'s pads 1-3, `R61`
    and `L4` change. ERC has no new errors; it gains the Beetle's one warning
    (the Molex `Feed` pin is "unspecified" against `C5`'s passive pin). DRC is
    unchanged: 0 errors, 0 unconnected items, 0 parity issues. As on the
    Beetle, check the match with a VNA on the first article.
- **`BT2` is MYOUNG's BH-18650-A6AJ012** (2026-11-13, LCSC C19184084), an
  open-frame holder in the Keystone 1042/1043 outline, in place of the
  BH-18650-A5BJ001-2D box. It is 77 × 20.7 × 14.9 mm against 77.8 × 20.8 ×
  21.5 mm, and its housing is PA46, UL94 V-0, where the box was ABS with no
  flame rating. New symbol `Custom:BH-18650-A6AJ012`, footprint
  `Footprints:BAT_BH-18650-A6AJ012` and LCSC's 3D model
  (`3dmodels/BH-18650-A6AJ012.step`). The pins are still P and N, so the
  nets do not change.
  - **Land (drawing MY-CP-0373 rev A/0, top view):** terminals 71.45 mm
    apart, P 35.8 mm and N 35.65 mm from the centre between the posts. Two
    Ø3.3 mm snap posts sit 27.6 mm along and 8 mm off the axis, on opposite
    sides, with a Ø2.4 mm key peg 8 mm off the axis beside P. The box's two
    bosses were on the axis.
  - **The posts come through:** in LCSC's 3D model they reach 3.43 mm below
    the housing, about 1.8 mm proud of the top of a 1.6 mm board. The key peg
    (1.57 mm) does not.
  - **Not polarized**, like the box. + is at the key-peg end.
  - **Open underneath:** the box sat the cell on a 3.25 mm floor. Here
    nothing separates the cell's wrap from the bottom of the board, so `S1`'s
    trimmed posts and any lead under the holder must stay flush.
- **Checked:** the netlist differs from V1 only by this change. ERC has
  nothing new. The six new power symbols carry the same `lib_symbol_mismatch`
  as every other power symbol on the board. The `+3V3` net's
  `power_pin_not_driven` now names `U3` pin 20 instead of `U1`.
- **PCB (2026-10-30):** brought up to this schematic with pcbnew scripts, the
  footprints linked to their symbols by path as Update PCB links them.
  `kicad-cli pcb drc --schematic-parity` finds 0 errors, 0 unconnected items
  and 0 parity issues, with 26 warnings, as at V1's sign-off. The zones are a
  fresh fill.
  - **Removed:** `U1`, `C1`, their `SPI_*` escape traces and vias, `U1`'s
    `+3V3` stub with its two vias, `R13` and `R16`.
  - **The buck** follows the Beetle's arrangement. `U9` is on the QFN-16 land
    at (50.85, 168.95), turned −90°:
    - SW faces `L9` at (52.1, 164.98).
    - PVIN and AVIN face down toward `U22`.
    - VOS and PGND face the output caps.
    - FB, AGND, FSW and DEF face the board edge.

    The four 22 µF output caps (`C58`, `C60`–`C62`) stand in a column to the
    right, with their `+3V3` pads toward `L9` and a ground via beside each
    ground pad. `C59` (22 µF) sits under PVIN. `C29` and `C25` are beside it
    on the same V_SWITCH strip, and `C23` and `C90` are on the left. `C26`
    moved next to the output column, and `C88` moved to the right of `U22`.
    - **F.Cu pours:** SW joins pins 1–3 to `L9` pad 1. `+3V3` takes in `L9`
      pad 2, the output caps, VOS and the pull-up ends of `R58` and `R59`.
      V_SWITCH takes in PVIN, AVIN, EN, the input caps, `U22`'s outputs
      and `R44`.
    - **Ground:** the exposed pad has one via to In1, under JLC's exception
      to the no-via-in-pad rule for QFN ground pads. PGND, FB, AGND and DEF
      tie to the pad on F.Cu.
    - **FSW** has its own via to the In2 `+3V3` plane, 0.145 mm clear of
      In2's VCC trace along the left edge. That trace is why the block sits
      0.5 mm right of the old `U9`.
    - **Silk:** `U9`'s pin-1 dot lands between `U9` and `L9`, over `L9`'s pad
      1. That gives one silk_over_copper warning and one silk_overlap
      warning. The fab clips the dot; the F.Fab marker stays.
  - **`U23`** (SOP-8) is at (52.2, 158.95), turned 180°, in the free copper
    below the antenna's via fence. Its left pads are 0.39 mm from the board
    edge. `C89`, `R60`, `R58` and `R59` sit in a row under it.
    - **Fan-out:** GPIO4–7 leave `U3`'s left side at 0.25 mm pitch on
      0.127 mm tracks, between `C17`/`C18` and the pads.
    - **Layers:** MOSI, CS and MISO stay on F.Cu and enter their pins from
      under the body. SCK would have to cross MOSI, so it drops to B.Cu at
      (53.4, 154.0) and comes back up beside pin 6. WP runs round the
      outside to `R59`.
    - **To make room:** `L_DI01`'s via moved from (55.66, 152.44) to
      (55.9, 151.95). Five ground stitching vias went: `C17`/`C18`'s own via
      at (55.675, 151.815), two in the fan-out corridor, one that would have
      sat in `U23`'s ground pad, and one by the crystal. `C17`/`C18`'s ground
      via is back at (55.425, 151.6); without it their ground was an island.
    - **`Volt_Read`** used to run diagonally on B.Cu under the old buck. It
      now runs straight down x = 54.2 and turns under `U22`'s top edge, which
      frees B.Cu under the input caps.
  - **`J2`** is on the GCT land at (63.06, 189.0). The front face of its fab
    outline is at y = 195.33, the same as HRO's, so the case's USB slot lines
    up.
    - **Traces:** the signal pads land within 0.075 mm of HRO's, so the
      existing traces end inside them.
    - **BT2:** pad P is 0.838 mm from J2's copper (the A1/B12 pad), against
      0.834 mm to HRO's shell on V1. The plated shell slots are gone, and with
      them two BT2 silk warnings. (With the A6AJ012's 2.8 × 3.6 mm slot pad
      the gap is about 1.08 mm.)
  - **`BT2` (2026-11-25):** the A6AJ012 is on the bottom at (63.1, 149.13),
    −90°, on the axis of `J2` and `J8`. Pad P is at (63.1, 184.93) and pad N
    at (63.1, 113.48). The snap posts are at (71.1, 121.53) and
    (55.1, 176.73), and the key peg at (71.1, 184.93). The board is re-placed
    and re-routed around it. DRC finds 0 errors, 0 unconnected items and
    0 parity issues. It reports 29 warnings: the 26 from before, plus the
    3 listed below.
    - **Pad P:** `R14` moves to (60.5, 186.05) and `R15` to (65.65, 185.15).
      That leaves 1 mm between the pad and `J2`'s pads. In it, D− joins B7
      and A7 on F.Cu and drops through a via at (62.81, 186.75). D+ drops
      beside B6 at (63.81, 186.85). Both go round the pad's right side on
      B.Cu to `CR1`. VBUS rises at 45° from B4/A9 to x = 67.21. The VCC zone
      wrapped the pad's left and top sides until 2026-11-26, when pad P
      became `B+` and the zone went (battery-current bullet below).
    - **Pad N:** `U16` (E220) moves 0.5 mm right, to (70.17, 114.37), and
      clears the pad by 0.21 mm. Its fan-out moves with it. `C33` and `R55`
      stand in a column below the pad at x = 63.33. `U14`'s VCC pin reaches
      them by a trace under the SOT-23 body. `R6` moves to (64.3, 119.04).
      The B− zone reaches further over the pad and now joins pads solidly:
      with thermal relief, only one of the two required spokes landed.
    - **Post at (71.1, 121.53):** `L_CS` and `L_SCK` drop through vias just
      inside `U16`'s outline. They run down the right side of the hole on
      B.Cu. `L_MOSI`'s via moves to (69.6, 120.15).
    - **Post at (55.1, 176.73):** `U22` moves 0.1 mm left and `C88` moves to
      (54.7, 174.55). `U22`'s ON line goes round under the hole to a via at
      (58.3, 174.1). On B.Cu, D+ and D− come down at x = 57.0 and 57.23
      instead of 55.27 and 55.49, and `Volt_Read` turns at y = 174.6.
    - **Key peg:** `D6` moves 3.36 mm right, to (75.0, 182.05), with `R54`
      below it. At x = 74, its leads would sit under the holder's side wall
      (x = 73.45). The `3v3` label is now above `D6` at (74.03, 180.35).
      BT2's back-side "−" mark is now on the new axis at (63.1, 108.55). The
      case's green-LED hole has to follow `D6`.
    - **`U24` (INA230)** was parked on the top side at (59.5, 124.5),
      unwired. It is now wired and sits beside pad P; see the
      battery-current bullet below.
    - **New warnings:** `S1`'s pads 1 and 3 (x = 72.93) sit on BT2's
      outline, under the holder's right wall; that came with the x = 63.1
      placement. `U22`'s silk outline touches the hole at (55.1, 176.73),
      and the fab clips it.
- **`U24` (INA230) measures the cell's current** (2026-11-26), wired as on
  the Beetle V2. `R62`, a 20 mΩ 0805 shunt (`PA0805FRE470R02L`, LCSC
  C723411; the same Yageo PA series as the Beetle's 2 mΩ `R72`), sits between
  BT2's + pad and VCC. The cell side is a new net, `B+`.
  Charge current from `U5` and load current both pass through the shunt, in
  opposite directions, so the INA230 reads the cell's net current.
  - **Schematic** (`battery.kicad_sch`, "1S Battery Current"): `IN+` is
    `B+`; `IN−` and `BUS` are VCC. `VS` is +3V3, with `C91` (100 nF) at the
    pin. `A0` and `A1` are grounded, so the address is 0x40. `ALERT` is open,
    as on the Beetle. The I²C bus is new on this board: `I2C_SCL` is GPIO10
    (`U3` pin 15) and `I2C_SDA` is GPIO12 (pin 17). Each has a 5.11 k pull-up
    to +3V3 (`R63`, `R64`).
  - **Shunt and taps:** `R62` lies left of pad P at (58.81, 184.31), with
    its `B+` end 1.07 mm from the pad and joined to it by a 0.8 mm track.
    `U24` sits above it at (57.56, 181.11), rotated 270°, so its sense pins
    face the shunt. `IN−` with `BUS`, and `IN+`, reach the shunt's two pads
    on 0.1 mm tracks that no pour touches. `C91` sits under `VS`, with a
    +3V3 via beside it. There is no via in the exposed pad. (Positions as
    saved 2026-11-26 06:22, after the owner moved this group 0.74 mm left
    and re-routed it.)
  - **VCC:** the F.Cu VCC zone is gone. A 0.5 mm track runs from the shunt's
    VCC end to `C42` and `U22`. A via on that track carries VCC on In2 to the
    via at (62.59, 180.91), which feeds `U5`'s output and `C28` on F.Cu and
    `R41` on B.Cu. The `U5`-side current is the charge current, at most
    450 mA.
  - **I²C route:** each line leaves `U3`'s bottom row through a via just
    below its pad. The two run south on B.Cu between the ground-via rows and
    come back up at y ≈ 168.5. From there they run on F.Cu down the channel
    between `U22`'s ON line and `U5`'s TS line to `U24`. `R63` and `R64` sit
    on that run.
  - **Checked:** ERC loses the 19 errors on `U24`'s pins and the 2 on
    GPIO10/12, and adds only warnings of kinds the sheet already has.
    The netlist was checked pin by pin. Schematic parity is 0 and
    `tools/check_board_parity.py` passes with 98 symbols. DRC on the board
    as saved 2026-11-26 06:32, after the owner's fixes: 0 errors,
    0 unconnected, and 31 warnings (silkscreen, plus library notes on `J8`
    and the logo), the same after a zone refill.
  - **Why 20 mΩ:** the flight computers' 2 mΩ would give 1.25 mA per bit
    over ±41 A, where the base station draws well under 1 A. 20 mΩ gives
    0.125 mA per bit over ±4.1 A. It drops 9 mV at the 450 mA charge
    current and dissipates 4 mW there, against the part's 0.4 W rating.
- **Not done:**
  - `FABRICATION-NOTES.md` item 3 and its *Why* paragraph: `U3`'s thermal
    pad has no vias since 2026-11-26. This board takes no via in any pad
    (owner), so the nine V1 thermal-pad vias are gone and item 3 needs its
    V2 rewrite.
  - PCB: the gerbers and the position file are not re-plotted. That waits for
    a V2 order.
  - Firmware: SPIFFS mounted on the external NOR (`U23`) over SPI3. The V2
    board header (board 5: the 4 MB partition table, and the INA230 at 0x40
    with a 20 mΩ shunt on GPIO12/GPIO10) is PR #1825, not yet merged.
  - `bom.csv` and `FABRICATION-NOTES.md`, which still describe V1.

### V1

V1 was ordered from f85acb89 (tag `tinker-base-v1.0.0`). The rest of this
section describes V1 as ordered. Schematic and PCB are in sync and fully routed, with 0
parity issues, 0 unconnected, no DRC errors and zone fills that match a fresh
refill. That was checked on 2026-09-28, after the design review's fixes. The
remaining 26 DRC warnings are silkscreen and the logo's library nickname.
Layout details worth knowing:

- `R1`, the `CHIP_PU` pull-up, moved beside `C8` to free the spot above `C16`
  for `C24`.
- `CLK` and `WP` cross the other flash lines on `B.Cu`, through four vias. The
  flash's ball order and the ESP32's SPI pin order leave no arrangement without
  a crossing.
- **Ground vias:** `U3`'s exposed pad has nine, in the gaps between its paste
  windows (Espressif asks for at least nine). Nine more sit beside signal vias.
- **`S3`**'s body area has no top copper (Mitsumi's "no pattern" zone). Its
  `GPIO0` via moved out from under the switch.
- **`J2`**'s signal pads are trimmed at the rear to HRO's own land, which keeps
  0.84 mm between the battery's + pad and the connector.
- **Fiducials** `FID1`–`FID3` are on the top side.

**Firmware:** `tinkerrocket-idf/projects/base_station` built with
`-DTR_BS_BOARD=4` — the pin map is `main/board/board_v4.h`, taken from this
board's netlist: the radio over SPI on the pins in the table above, no I²C, no
pack charger, the cell on V3's divider. It builds but has not run, because no
board exists yet; the firmware release keeps shipping the full base station's
V3 image in the Tinker-Base slot until a first article proves V4. The V3 image
does not run on this board (it drives a `lora-daughterboard` over UART).

Reviewed 2026-08-12: [`prefab-review-2026-08-12.md`](prefab-review-2026-08-12.md).
Its "Verified correct" section is superseded: the Molex antenna and the
fixed-output regulator it describes are gone.

Reviewed again 2026-09-28: [`design-review-2026-09-28.md`](design-review-2026-09-28.md).
Its *Fixes applied* section lists what was done and what was left, and why.
The fabrication and assembly steps are in
[`FABRICATION-NOTES.md`](FABRICATION-NOTES.md). See also *Sending a board to fab* in
[`../README.md`](../README.md).
