# rocket-computer-mini

A reduced-capability flight computer. The design starts from a copy of
[`rocket-computer/`](../rocket-computer/) as it stood at the fork point, and
diverges from there by **removing** capability rather than adding it.

## This is a fork, not a variant

There is **no ongoing relationship** between the two boards. `rocket-computer-mini`
is not a derived revision, a build option, or a stripped configuration of
`rocket-computer` — nothing is shared, nothing tracks upstream, and a change to
one is never expected to propagate to the other. The starting geometry and
schematic were simply a convenient place to begin.

Treat anything inherited as a first draft to be justified on its own terms, not
as a decision already made.

## The ESP32-S3 is the only processor

The board arrived from the fork carrying both of `rocket-computer`'s processors.
The ESP32-P4 has been removed entirely, leaving the ESP32-S3 as the sole
processor — and since it is no longer one of two, it is no longer called *OUT*.

| | at fork | now |
|---|---|---|
| Parts | 255 | **190** |
| Nets | 234 | **191** |
| Sheets | 5 | **4** |

**Removed — the whole `Central Processing - ESP32-P4` sheet, 65 parts:** `U17`
(ESP32-P4), `U16` its SPI flash, `U20` its dedicated TLV62569 buck, `Y3`/`Y4`
its crystals, `SW2`, LEDs `D7`/`D8`, and 57 passives.

**Renamed — the S3 sheet is now `MCU - ESP32-S3`** in
[`esp32s3_mcu.kicad_sch`](esp32s3_mcu.kicad_sch), and its nine `OUT_*` nets are
now `MCU_*` (`MCU_SPI_CLK`, `MCU_D+`, …). Pin names that merely contain "OUT"
(`VOUT`, `OUT_17`, `Net-(U28-OUT)`) were deliberately left alone. The note on
the power sheet claiming *"ESP32-OUT … enables Flight Computer"* described the
S3 gating the P4 and is now wrong; it has been reworded to the switched sensor
and peripheral rail, but **the topology itself has not been rethought** — the S3
still gates a rail whose original purpose was powering a processor that no
longer exists.

### What the P4 took with it

The P4 owned nearly every subsystem. Removing it left **34 named signals with a
single remaining endpoint** and several more that kept a pull-up but lost their
driver. Nothing has been rewired to the S3 yet:

| Orphaned | Signals |
|---|---|
| Sensors | IMU / mag / baro SPI + I2C + interrupts, GNSS UART |
| Pyro | all four `PYRO*_FIRE` / `PYRO*_CONT`, `PYRO_ARM` |
| Expansion | all twelve `EXP_01`–`EXP_12` |
| Other | camera UART, `PIEZZO`, `SERVO_ACT`, the six `ESP_*` link lines |
| Power | `P4_EN_HOLD`, now a single node on `D9` |

**This board cannot fly as it stands.** Re-attaching these to the S3 is the next
design decision, and it is a real one — the S3 has far fewer usable pins than
the P4 did, so some of the above will not survive the move.

## What was and wasn't carried over

Copied and rewritten for the new project name: the root schematic, the five
sub-sheets (`in_sensors`, `power`, `central_processing_p4`, `esp32s3_outputs`,
`external_connections`), the PCB, the project file, the library tables, and
`bom.csv`. Of those, `central_processing_p4` has since been deleted and
`esp32s3_outputs` renamed, as above.

Deliberately **not** copied — `rocket-computer`'s seven design documents. All of
them are records of *that* board, and each would be actively misleading here:

- **The reviews** — `prefab-review-2026-07-30.md`, `prefab-review-2026-08-05.md`
  and `schematic-review.md`. Carrying them would assert this board has been
  reviewed when it has not.
- **`WORKLIST.md`** — the closing record of the V9 pre-fab review, written
  against V9's live files. Its board state, its closed items, and its bench list
  describe a board this one will not be.
- **`FABRICATION-NOTES.md`** — fab and assembly instructions keyed to V9's exact
  geometry (22.35 × 75.00 mm, 6 layer, 1.546 mm stack) and its specific parts.
  A reduced board will not share those numbers, and stale fab notes are the kind
  of error that reaches a fab house.
- **`power-eco.md`** and **`high-side-switch-design.md`** — design rationale for
  the V9 power architecture. These remain the best reading on *why* the
  inherited power tree looks the way it does; read them in
  [`../rocket-computer/`](../rocket-computer/) rather than in a copy here that
  would drift.

## Inherited state — read before editing

The fork is faithful: at the fork commit the exported netlist, ERC report and
DRC report are identical to `rocket-computer`'s apart from the project name and
timestamps (255 components, 234 nets). That also means the **inherited
violations came along with it** — 1012 ERC at `--severity-all` and 26 DRC, plus
11 schematic-parity items, 0 unconnected. None were introduced by the copy, and
none have been fixed here.

### Where the numbers stand now

| | at fork | after `rev1` | after P4 removal |
|---|---|---|---|
| ERC (`--severity-all`) | 1012 | 1012 | **823** |
| DRC (`--severity-all`) | 26 | 28 | **88** |
| Schematic parity | 11 | 11 | **8** |
| Unconnected | 0 | 0 | **0** |

Every pre-existing DRC category went *down* with the parts count. The rise to 88
is one thing: **66 `track_dangling` + 2 `via_dangling`** — stubs that used to run
to a P4 pad on nets that still have pads elsewhere, so they survived the
dead-net sweep.

**Left in place deliberately.** These signals are about to be rewired to the S3
and the affected region re-laid-out, so trimming the stubs now would be work
thrown away by the next pass. They are warnings, not errors, and `unconnected`
is still 0.

Three stale items are worth knowing about, none fixed:

- **`C12`'s footprint disagrees between schematic and board.** The pyro energy
  store was changed from the Nichicon `EKYC160ELL103MM25S` to a Rubycon 2200 µF
  16 V (`16ZLH2200MEFC12.5X20`) in a horizontal lay-down can with a hold-down
  strap — which is what the inherited fab notes' "can floating 4 mm above the
  board" warning was about. **The schematic carries the change; the PCB still
  has the old vertical footprint.** Swapping it on the board means re-placing
  C12 and re-routing it, which is layout work for the next pass.
- **`bom.csv` is still V9's full bill of materials** — 255 parts, including the
  65 that no longer exist. The C12 row is the one line that is current.
- **The on-board fabrication-note text still cites `QFN-104 (U17)`** as the
  reason ENIG is required. `U17` was the P4 and is gone. ENIG is still justified
  by `U15` (0.4 mm QFN-56) and `U21` (0.4 mm X2QFN), but the citation is dead and
  the note also still calls the board "ROCKET COMPUTER" at V9's dimensions.

### The revision, and what changing it cost

The title block arrived reading `(rev "V9")` — the revision of a *different*
design's fab release — and the front silkscreen renders
`Tinker\nRocket\n${REVISION}` from it. It now reads **`rev1`**, this board's own
first revision.

That is the only edit made since the fork, and it is not free. `rev1` is two
characters wider than `V9`, so the silkscreen block grew and now touches R1:

| | at fork | after `rev1` |
|---|---|---|
| DRC (`--severity-all`) | 26 | **28** |

The two new items are `silk_overlap` (the text against R1's silkscreen segment)
and `silk_over_copper` (the text against R1's pad 1), both at
`@(72.64 mm, 159.51 mm)`, both severity *warning*. They are cosmetic and in the
same class as the 26 already inherited — but they are **new, not inherited**,
and they are a layout problem, not a naming one.

Deliberately not fixed by nudging the text: this board exists to have most of
its content removed, and that silkscreen will have to be repositioned anyway
once the layout is cut down. Fixing it now would be laying out a board that is
about to stop existing. Whoever does the reduction should clear it then — or
shorten the string, since a two-character revision (`V1`, matching the `V9` /
`V5` convention on the other boards) reoccupies the original footprint exactly
and drops both warnings.

## Status

**Mid-reduction, not buildable.** The P4 is gone and the S3 is the sole
processor, but nothing has been rewired to it yet — the sensors, all four pyro
channels and the expansion header currently connect to nothing.

Not reviewed, not fabbed, no tag, no firmware project of its own. Further
reduction is expected, which is why the dangling stubs, the V9 BOM, the stale
on-board fab note and the `rev1` silkscreen overlap are all recorded here rather
than fixed — each one sits in a region that the next pass is going to rework.

Before it goes to fab it needs its own pre-manufacturing review — see *Sending a
board to fab* in [`../README.md`](../README.md).
