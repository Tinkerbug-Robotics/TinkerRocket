# base-station-mini

A new ground-station board. The design starts from a copy of
[`base-station/`](../base-station/) as it stood at the fork point, and diverges
from there.

## This is a fork, not a variant

There is **no ongoing relationship** between the two boards. `base-station-mini`
is not a derived revision, a build option, or a stripped configuration of
`base-station` — nothing is shared, nothing tracks upstream, and a change to one
is never expected to propagate to the other. The starting geometry and
schematic were simply a convenient place to begin.

Treat anything inherited as a first draft to be justified on its own terms, not
as a decision already made.

## What was and wasn't carried over

Copied and rewritten for the new project name: the root schematic, the four
sub-sheets (`battery`, `esp32_p3`, `power`, `external_charger`), the PCB, the
project file, the library tables, and `bom.csv`.

Deliberately **not** copied:

- **`base-station`'s design reviews** (`prefab-review-2026-08-02.md`,
  `power-switch-review-2026-08-02.md`). Those are records of a review performed
  on *that* board — carrying them here would assert this board has been reviewed
  when it has not. They remain the best reading on why the inherited power
  architecture looks the way it does; read them there.
- **`outputs.kicad_sch`**, an empty sheet in `base-station/` referenced by
  nothing. See the leftovers note in [`../README.md`](../README.md).

The fork is faithful: at the fork commit the exported netlist, ERC report, and
DRC report are identical to `base-station`'s apart from the project name and
timestamps. That also means the **inherited violations came along with it** —
534 ERC (mostly off-grid endpoints and library-symbol drift) and 36 DRC (all
silkscreen). None were introduced by the copy, and none have been fixed here
yet.

## Status

Fresh fork. Not reviewed, not fabbed, no tag, no firmware project of its own.
Before it goes to fab it needs its own pre-manufacturing review — see *Sending a
board to fab* in [`../README.md`](../README.md).
