# Tinker-Base

> **Tinker-Base** is the product name, from 2026-09-24, of the board this repo
> called `base-station-mini` — the one base station in the product line, for
> both the Tinker-Mantis and the Tinker-Beetle. The board it forked from, the
> full `base-station`, is now in [`../legacy/`](../legacy/base-station/).

A reduced-feature ground station. **"Mini" means fewer features, not a smaller
board** — the outline is still base-station's 30.27 × 90.50 mm. Nothing here is
trying to shrink; it is trying to do less.

Currently rev **V1**, never fabricated.

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
  apart. With the divider, the fixed part would drive about 21 V.

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
- **Boot flash.** `U1` is the Beetle and Mantis's 16 MB quad-SPI NOR in a
  24-ball WLCSP. It is fed from `+3V3`, with `C1` (100 nF) beside it, instead of
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

**2026-10-03: `J2` is now the GCT `USB4110-GF-A`** that the Mantis, Beetle, LoRa
and Space Bug boards use, in place of the HRO `TYPE-C-31-M-12`. The symbol and
its pin numbers are unchanged; only the footprint and part fields moved, so the
netlist is identical. On the PCB it sits where the HRO did, front face 1.11 mm
past the board edge, so its signal pads, pegs and every existing track line up.
DRC still shows 0 errors, 0 unconnected and 0 parity issues, and the fills match
a fresh refill. The V1 boards ordered on 2026-09-29 still take a
`TYPE-C-31-M-12`.

V1 was ordered on 2026-09-29 (tag `tinker-base-v1.0.0`). Schematic and PCB are
in sync and fully routed, with 0 parity issues, 0 unconnected, no DRC errors
and zone fills that match a fresh refill. That was checked on 2026-09-28, after
the design review's fixes, and again on 2026-10-03 after the `J2` change. The
remaining 24 DRC warnings are silkscreen and the logo's library nickname.
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
- **`J2`** (GCT `USB4110-GF-A`) keeps 0.84 mm between the battery's + pad and its
  nearest pad. Its shell tabs are SMD pads; two ground vias between them tie the
  shell to the planes.
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
