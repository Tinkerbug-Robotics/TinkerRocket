# Hardware

KiCad design sources for the TinkerRocket boards. Imported from the local
`Circuit Board Designs/` folder — see #596 for the rationale and the size
analysis behind what is and isn't tracked here.

Built with **KiCad 10.0.3**.

These design files are licensed under the [CERN Open Hardware Licence v2 — Strongly
Reciprocal](LICENSE) (`CERN-OHL-S-2.0`), which is separate from the GPL covering the
software. If you distribute a product based on them, or publish a modified design, the
complete sources must be available under the same licence.

## Boards

| Folder | Board | Firmware target |
|---|---|---|
| [`rocket-computer/`](rocket-computer/) | Flight computer + out computer, one board | `tinkerrocket-idf/projects/flight_computer` (ESP32-P4) and `.../out_computer` (ESP32-S3) |
| [`rocket-computer-mini/`](rocket-computer-mini/) | Flight computer, reduced capability — forked from `rocket-computer`, no ongoing link | — (none yet) |
| [`base-station/`](base-station/) | Ground station | `tinkerrocket-idf/projects/base_station` |
| [`base-station-mini/`](base-station-mini/) | Ground station, second design — forked from `base-station`, no ongoing link | — (none yet) |
| [`lora-daughterboard/`](lora-daughterboard/) | Swappable UART radio module | `tinkerrocket-idf/projects/radio_board` |
| [`gnss-px1105r-18mm-highpower-ext-ant/`](gnss-px1105r-18mm-highpower-ext-ant/) | GNSS carrier, PX1105R, external antenna | — (module carrier) |
| [`gnss-sam10m8-18mm-hv/`](gnss-sam10m8-18mm-hv/) | GNSS carrier, SAM-M10Q, high-voltage variant | — (module carrier) |
| [`servo-adapter/`](servo-adapter/) | Cable-to-servo adapter with capacitor | — (passive) |

**Revision numbers are no longer in the filenames.** What was `TinkerRocket
Full V9` is now `rocket-computer`. Git history and tags carry the revision
instead — see *Sending a board to fab* below. Each project was renamed with
KiCad's own internal references rewritten (`(project ...)` instance data,
`(sheetfile ...)`, and `schematic.top_level_sheets`); the imported netlists
were diffed against the originals and match exactly on component count,
values, footprints, and full connectivity.

## Working on these locally

Yes — branch, commit, PR, merge, same as the firmware. The tooling is the
same. There are four differences that matter, and one of them will corrupt a
board if you ignore it.

### 1. Never let git merge a board file

`.kicad_pcb` and `.kicad_sch` are text, so git will *happily* three-way-merge
them. The result is not a board with both changes — it is a corrupt board, or
worse, a plausible-looking one with silently mangled geometry. Git cannot know
that two hunks 400 lines apart are the same net.

So treat each board as **one writer at a time**:

- Don't edit the same board on two branches concurrently.
- Prefer `git pull --rebase` over merge.
- If you ever do get a conflict inside a `.kicad_pcb`/`.kicad_sch`, **do not
  hand-resolve it.** Take one side whole and redo the other edit in KiCad:

```bash
git checkout --ours hardware/rocket-computer/rocket-computer.kicad_pcb
```

Sub-sheets are separate files, so two people *can* safely work on different
sheets of the same schematic — but the PCB is always a single-writer file.

### 2. Close the project before switching branches

KiCad holds the board in memory and writes on save. If you check out another
branch with the project open, KiCad will happily overwrite the new files with
the old in-memory copy. Close the project first, then switch. The stray `*.lck`
files scattered through the original folders are the fossil record of this.

### 3. Review with generated artifacts, not diffs

Nobody can review an S-expression diff. Generate something lookable-at and put
it in the PR:

```bash
kicad-cli sch export pdf -o /tmp/sch.pdf hardware/rocket-computer/rocket-computer.kicad_sch
```

```bash
kicad-cli pcb export svg --mode-single --layers F.Cu,F.Silkscreen,Edge.Cuts -o /tmp/pcb.svg hardware/rocket-computer/rocket-computer.kicad_pcb
```

A BOM/netlist diff is the highest-signal check for "did this change what I
think it changed":

```bash
kicad-cli sch export netlist -o /tmp/after.net hardware/rocket-computer/rocket-computer.kicad_sch
```

### 4. Sending a board to fab

Gerbers are **not** tracked — they are a pure function of the board file, and a
checked-in copy only ever drifts out of sync with it. Each board keeps an empty
`gerbers/` directory as the export target. Generate the package, then tag the
commit so the revision stays recoverable:

```bash
kicad-cli pcb export gerbers -o hardware/rocket-computer/gerbers/ hardware/rocket-computer/rocket-computer.kicad_pcb
```

```bash
git tag hw/rocket-computer/v10 -m "sent to fab 2026-xx-xx"
```

The tag is what replaces `V9` in the filename. `git show hw/rocket-computer/v9`
gets you exactly what was fabbed.

## Libraries

Custom symbols and footprints are vendored into the repo, so a fresh clone
resolves them without depending on anyone's local KiCad setup:

- [`symbols/Custom.kicad_sym`](symbols/Custom.kicad_sym) — 58 symbols
- [`footprints/Footprints.pretty/`](footprints/Footprints.pretty/) — 60 footprints

These are **only the parts these eight boards use.** The shared libraries they
came from hold 240 symbols and 288 footprints accumulated across every past
design; carrying the unused ~75% into the repo would mean every future board
inherits a junk drawer.

Each board has a project-local `sym-lib-table` / `fp-lib-table` pointing at
`${KIPRJMOD}/../symbols/` and `${KIPRJMOD}/../footprints/`. The library
nicknames are unchanged (`Custom`, `Footprints`) and a project-local table wins
over the global one for the same nickname, so **no board file needed editing** —
they simply resolve from the repo now. KiCad's stock libraries (`Device`,
`power`, `Capacitor_SMD`, …) still come from your KiCad install, as intended.

**Adding a part:** add it to `hardware/symbols/Custom.kicad_sym` and
`hardware/footprints/Footprints.pretty/` — inside the repo, in the same commit
as the board that uses it. Adding it only to your personal shared library will
work on your machine and break on a clean clone.

## What is tracked

Design sources (`.kicad_pro`, `.kicad_pcb`, `.kicad_sch`), the vendored
libraries, BOMs, and design review notes. Everything KiCad regenerates —
gerbers, autosave zips, `fp-info-cache`, `.kicad_prl` GUI state, STEP exports —
is ignored; see [`.gitignore`](.gitignore).

## Known issues in the imported data

These are pre-existing problems in the source folders, carried over as-is
rather than silently "fixed". Each is worth a look before the next respin.

- **The original gerber folders had drifted from their boards** — the rocket
  computer's held V9 `.gbr` files next to a `-job.gbrjob` indexing
  `TinkerRocket Full V8-*.gbr`. That drift is the reason gerbers aren't
  tracked here; regenerate at order time.
- ~~**3D models are still machine-local.**~~ **Fixed.** The 54 available models
  are vendored in [`3dmodels/`](3dmodels/) and every reference now resolves
  through `${KIPRJMOD}`. Two remain unavailable and are listed in
  [`3dmodels/README.md`](3dmodels/README.md); both were already broken before
  the import.
- **`PMPB14XNX` had already been deleted from the shared symbol library** while
  the rocket computer still used it. It was recovered from the schematic's own
  embedded copy — which is by definition what the board was built with — rather
  than from a same-named file of unknown vintage elsewhere on disk.
- **`Custom Library:2337019-1`** (used by the PX1105R board) refers to a KiCad
  6.0-era library that no longer exists on disk at all. The symbol is embedded
  so the board is fine, but that one link is dangling and can't be re-vendored.
- **Some source folders were cloned from other boards and kept the leftovers.**
  `LoRa Board V3/` contained five Full-board sub-sheets
  (`central_processing_p4`, `in_sensors`, …) unreferenced by its own flat root
  sheet, plus a BOM listing 234 designators against a 65-component netlist.
  Those were **not** imported. The original folders are untouched if you want
  to confirm.
