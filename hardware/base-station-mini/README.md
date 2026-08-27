# base-station-mini

A reduced-feature ground station. **"Mini" means fewer features, not a smaller
board** — the outline is still base-station's 30.27 × 90.50 mm. Nothing here is
trying to shrink; it is trying to do less.

Currently rev **V1**, never fabricated.

## What it drops, and what it gains

Against [`base-station/`](../base-station/):

- **The LoRa radio is on-board.** `U16`, an E220-900MM22S on SPI, replaces the
  UART header that fed an external [`lora-daughterboard/`](../lora-daughterboard/).
  That removes the daughterboard, its connector, and the 5 V `V_LORA` boost that
  supplied it — the module runs straight off +3V3.
- **No external charger sheet.** Dropped from the root, along with the
  `PosADC`/`MidADC` sense lines it fed. `external_charger.kicad_sch` was deleted
  outright on 2026-08-12 rather than left orphaned on disk.
- **No sensor I²C.** `SDA_SENS`/`SCL_SENS` are gone, which is what freed GPIO33
  and GPIO34 for the radio.
- **`+3V3` comes from a fixed-output TPS63021** buck-boost rather than the
  adjustable TPS63020 and its feedback divider. base-station moved to the same
  part at V6.

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
charger — the dominant heat source while charging, dissipating on the order of
1 W as a linear regulator at the ~0.79 A `R52` programs — 27 mm from the
buck-boost and 15 mm from the ESP32-S3. Weighted by dissipation over distance
that is about 37 % less coupling than the old land, which sat 8.6 mm from the
buck-boost and 13.5 mm from the charger. A solid inner ground plane keeps the
board close to isothermal, so what remains is a board-wide offset rather than a
local hot spot.

Everywhere on the board that is further from the heat needs a sensor trace
threaded through the ESP32-S3 fanout or the chip antenna's ground-via fence;
neither is worth it for a few degrees. The route as built is 19 mm on `F.Cu`
with no vias.

**Open at first article:** with the board charging, compare the `TS` node
against a reference probe on the cell at room temperature and near 0 °C, and
confirm the offset is small enough to accept. If it is not, the fix is a wired
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
- **`outputs.kicad_sch`**, an empty sheet in `base-station/` referenced by
  nothing. See the leftovers note in [`../README.md`](../README.md).

## Status

Not fabbed, no tag, no firmware project of its own. Schematic and PCB are in
sync and fully routed — 0 parity issues, 0 unconnected.

Reviewed 2026-08-12: [`prefab-review-2026-08-12.md`](prefab-review-2026-08-12.md).
Four items to close before fab, none of them blockers; the review's *Before fab*
section is the checklist. See also *Sending a board to fab* in
[`../README.md`](../README.md).
