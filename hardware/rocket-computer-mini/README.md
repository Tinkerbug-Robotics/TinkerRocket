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
as a decision already made. That applies with force here: the fork currently
carries the **full** V9 design — both processors, all five sheets, 255
components — and the entire point of the board is that most of it goes away.
Nothing has been removed yet.

## What was and wasn't carried over

Copied and rewritten for the new project name: the root schematic, the five
sub-sheets (`in_sensors`, `power`, `central_processing_p4`, `esp32s3_outputs`,
`external_connections`), the PCB, the project file, the library tables, and
`bom.csv`.

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

Two inherited items are wrong for this board rather than merely unreviewed, and
should be fixed early:

- The PCB title block still reads `(rev "V9")`, and the front silkscreen renders
  `Tinker\nRocket\n${REVISION}` from it — so the board currently silkscreens the
  revision of a *different* design's fab release. This board has no revision
  history of its own yet.
- `bom.csv` is V9's full bill of materials and will be wrong the moment anything
  is removed.

## Status

Fresh fork, nothing reduced yet. Not reviewed, not fabbed, no tag, no firmware
project of its own — the inherited schematic still targets both the ESP32-P4
flight computer and the ESP32-S3 out computer.

Before it goes to fab it needs its own pre-manufacturing review — see *Sending a
board to fab* in [`../README.md`](../README.md).
