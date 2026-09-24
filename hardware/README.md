# Hardware

KiCad design sources for the TinkerRocket boards. Imported from the local
`Circuit Board Designs/` folder — see #596 for the rationale and the size
analysis behind what is and isn't tracked here.

Built with **KiCad 10.0.3**.

These design files are licensed under the [CERN Open Hardware Licence v2 — Strongly
Reciprocal](LICENSE) (`CERN-OHL-S-2.0`), which is separate from the GPL covering the
software. If you distribute a product based on them, or publish a modified design, the
complete sources must be available under the same licence.

**Assembly conventions that cannot travel in the design files** — stencil
thickness, the area-ratio floor and paste coverage — live in
[`SOLDER-PASTE-CONVENTION.md`](SOLDER-PASTE-CONVENTION.md) (#959, #906). Every
board's `FABRICATION-NOTES.md` points there rather than restating them.
The inter-board jumpers — which part to order and the catalogue-naming trap
that once cooked a radio — are in [`cables.md`](cables.md) (#677).

## Product line

The boards offered as products, and the firmware each runs. The product names
replaced the engineering names on 2026-09-24; the old names still identify
earlier revisions in git tags, issues and the dated review records, so both are
listed.

| Folder | Product | Was | What it is | Firmware |
|---|---|---|---|---|
| [`tinker-mantis/`](tinker-mantis/) | **Tinker-Mantis** | `rocket-computer` | Full-size flight computer: ESP32-P4 flight computer and ESP32-S3 out computer on one board | `flight_computer` + `out_computer`, `-DTR_BOARD_V9=1` (V9/V10) |
| [`lora-daughterboard/`](lora-daughterboard/) | LoRa daughterboard for Tinker-Mantis | — | Swappable UART radio module | `radio_board` |
| [`gnss-sam10m8-18mm-hv/`](gnss-sam10m8-18mm-hv/) | GNSS carrier for Tinker-Mantis | — | SAM-M10Q carrier, pack-voltage input | — (module carrier) |
| [`tinker-beetle/`](tinker-beetle/) | **Tinker-Beetle** | `rocket-computer-mini` | Reduced-capability flight computer: two ESP32-S3s, on-board LoRa and GNSS. Forked from `rocket-computer`, no ongoing link | `flight_computer` + `out_computer`, `-DTR_BOARD_M1=1` |
| [`tinker-base/`](tinker-base/) | **Tinker-Base** | `base-station-mini` | Ground station with on-board LoRa, the one base station for both computers. Forked from the full base station, no ongoing link | `base_station` — its V3 build is the starting point; the board has no map of its own yet |

Firmware project names are in `tinkerrocket-idf/projects/`; see
[`tinkerrocket-idf/projects/README.md`](../tinkerrocket-idf/projects/README.md) for
the product-to-build map and why the project names did not change.

## Legacy boards

[`legacy/`](legacy/) holds the boards that left the product line — the full
base station, the PX1105R GNSS carrier and the servo adapter. They still open,
still pass the parity gate and can still be plotted from their tags; see
[`legacy/README.md`](legacy/README.md).

## Names and revisions

**Revision numbers are not in the filenames.** What was `TinkerRocket Full V9`
became `rocket-computer`, and on 2026-09-24 `tinker-mantis`. Git history and
tags carry the revision instead — see *Sending a board to fab* below. Both
renames rewrote KiCad's own internal references (`(project ...)` instance data,
`(sheetfile ...)`, and the `.kicad_pro` file names and `top_level_sheets`); the
second also renamed the schematic title-block titles that carried the old
project name (six sheets, all on the Tinker-Beetle). The first import's netlists were diffed against the originals and
match exactly on component count, values, footprints, and full connectivity.
The 2026-09-24 rename was checked the same way and further: netlist, BOM
export, every gerber and drill file (timestamps and project name aside) and the
DRC report are identical before and after, on all six boards it touched.
Silkscreen text was not changed by it: the boards still read `Tinker Rocket`,
`TR-Mini` and `TinkerRocket Base Station Mini` until someone edits the artwork.

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
git checkout --ours hardware/tinker-mantis/tinker-mantis.kicad_pcb
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
kicad-cli sch export pdf -o /tmp/sch.pdf hardware/tinker-mantis/tinker-mantis.kicad_sch
```

```bash
kicad-cli pcb export svg --mode-single --layers F.Cu,F.Silkscreen,Edge.Cuts -o /tmp/pcb.svg hardware/tinker-mantis/tinker-mantis.kicad_pcb
```

A BOM/netlist diff is the highest-signal check for "did this change what I
think it changed":

```bash
kicad-cli sch export netlist -o /tmp/after.net hardware/tinker-mantis/tinker-mantis.kicad_sch
```

### 4. Sending a board to fab

Gerbers are **not** tracked — they are a pure function of the board file, and a
checked-in copy only ever drifts out of sync with it. Each board keeps an empty
`gerbers/` directory as the export target. Generate the package, then tag the
commit so the revision stays recoverable:

Use `tools/plot_gerbers.sh` rather than a bare `kicad-cli` invocation — it refuses to
plot a dirty tree, stamps the commit and tag into the package's `README.txt`, and warns
when the title-block rev disagrees with the tag:

```bash
tools/plot_gerbers.sh tinker-mantis
```

A legacy board is plotted the same way, as `legacy/base-station` or just
`base-station`.

Tag with the scheme in [docs/board-versioning.md](../docs/board-versioning.md) —
lightweight, `<board>-v<major>.<minor>.<patch>`:

```bash
git tag tinker-mantis-v10.0.0
```

`git show rocket-computer-v9.0.0` gets you exactly what was fabbed — revisions
fabbed before the 2026-09-24 rename keep the tag they were given, under the old
name and the old folder (`hardware/rocket-computer/` at that tag).
`plot_gerbers.sh` searches both names, so a Tinker-Mantis plot made before its
first `tinker-mantis-v*` tag still records `rocket-computer-v9.0.0` as its
lineage.

This section used to give `git tag hw/rocket-computer/v10` and
`git show hw/rocket-computer/v9`. That scheme does not exist: `git tag -l "hw/*"`
returns nothing, `plot_gerbers.sh` matches `<board>-v*`, so a tag made the old way was
invisible to the provenance tooling — the gerber package would have carried the
PREVIOUS tag as its provenance while warning about a rev mismatch, and the documented
`git show` would have returned `fatal: invalid object name` (#838 item 8).

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
