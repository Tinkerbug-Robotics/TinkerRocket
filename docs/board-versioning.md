# Board versioning

Semver's fields don't map onto a PCB by themselves, so this defines what each one
means physically. The question every field is trying to answer is: **do I need to
order new boards, or just different parts?**

## What the fields mean

| field | meaning | do I need new bare boards? |
|---|---|---|
| **major** | copper changed — layout, netlist, footprints, stackup | **yes** |
| **minor** | BOM only — component values or MPNs, identical gerbers | no |
| **patch** | documentation, silkscreen text, metadata, field tidy-ups | no |

Worked examples from the LoRa V3 pre-fab review:

- adding C14 and C18 → **major**, new copper
- 18 pF → 12 pF crystal caps, capacitor MPNs, the C97 revert → **minor**, same board, different reel
- correcting a review document or a `Datasheet` field → **patch**

A board that has never been fabricated stays at `major.0.0` while it is being
worked on; roll major only when a *previously fabricated* revision is superseded.

## Where the version lives

**Git tags are authoritative.** A version committed to a file describes a commit
that cannot include it — the hash changes when you commit. Tags avoid that.

```
lora-daughterboard-v3.0.0
rocket-computer-v9.0.0
base-station-v5.0.0
gnss-sam10m8-18mm-hv-v2.0.0
gnss-px1105r-18mm-highpower-ext-ant-v1.0.0
servo-adapter-v1.0.0
```

Per-board prefixes because the six boards revise independently. The format matches
the existing `android-vX.Y.Z` convention: lightweight tags, `<board>-v<semver>`.

**The silkscreen carries the marketing version only** — `V3`, not a git hash.
There is no room on a 22 × 27.5 mm board, and the fab does not need it there.
That string is single-sourced through the PCB title block:

```
(title_block (rev "V3"))          <- the one place it is written
(gr_text "LoRa ${REVISION}")      <- silkscreen references it
```

KiCad resolves `${REVISION}` at plot time, including from `kicad-cli`, so the
artwork is unchanged — verified by identical stroke counts across all five boards
before and after the switch.

**Full provenance goes in the gerber package, not on the board.**
`tools/plot_gerbers.sh` writes a `README.txt` beside the gerbers carrying
`git describe --tags --always --dirty`, so every plotted set says exactly which
commit produced it, and says `-dirty` if the tree had uncommitted changes.

## Releasing a board

1. Clear DRC (`--severity-error --schematic-parity`). Nothing in CI validates
   `hardware/`, so this is the only gate.
2. Bump `(rev "...")` in the PCB title block if the marketing version changed.
3. Commit, then tag: `git tag <board>-v<major>.<minor>.<patch>`
4. `tools/plot_gerbers.sh <board>` — refuses on a dirty tree unless `--allow-dirty`,
   and warns if the title-block rev disagrees with the tag.

## Current state

Tagged at introduction of this scheme (2026-08-01):

| board | version | DRC at tagging |
|---|---|---|
| gnss-px1105r-18mm-highpower-ext-ant | v1.0.0 | 0 errors ✓ |
| gnss-sam10m8-18mm-hv | v2.0.0 | 0 errors ✓ |

Not tagged — these are not in a releasable state and a tag would imply otherwise:

| board | blocker |
|---|---|
| rocket-computer | 237 DRC errors, 26 unconnected — battery rework mid-flight, J8 unwired |
| base-station | 10 × malformed_courtyard on U15 (antenna footprint) |
| lora-daughterboard | 2 courtyard overlaps; PCB still lags the schematic (C18 unplaced, C90/C92 read 18 pF in copper, U22 pad 8 on the wrong net) |
| servo-adapter | 4 courtyard overlaps (J1 vs C1/C3/C4); no version on silkscreen yet |

Tag each as its DRC clears.
