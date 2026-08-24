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

## Boot NOR size per revision

The one board fact that firmware must get right per revision, because getting
it wrong in one direction is not a warning but an unbootable board.

`CONFIG_ESPTOOLPY_FLASHSIZE` goes into the image header the ROM reads before
the second-stage bootloader runs. Declaring **less** than is fitted is benign —
the board boots, logs `Detected size(16384k) larger than the size in the binary
image header(8192k)`, and the top of the part is unreachable. Declaring
**more** is fatal, in the bootloader, before `app_main` and before any of our
logging exists:

```
E spi_flash: Detected size(8192k) smaller than the size in the binary image
             header(16384k). Probe failed.
E esp_core_dump_flash: Core dump flash config is corrupted!
```

A continuous boot loop on hardware that is perfectly good, with no firmware log
to explain it. No runtime assertion can catch this, because nothing of ours
runs.

**The fleet is mixed and the split has a date.** The 2026-08-09 fab close-out
moved every board from a Winbond `W25Q64JVXGIQ` (8 MB, DFN) to a
`W25Q128JVSIQ`, then `GD25Q128ESIG` from 2026-08-15 (both 16 MB, SOIC-8).
Boards fabbed before that day carry the small part. Stated by the board owner
2026-08-24 as the general rule: **the latest revision of each board is 16 MB,
every earlier one is 8 MB.**

| board | revision | boot NOR | fitted | evidence |
|---|---|---|---|---|
| rocket-computer | V7 | U13 / U16 `W25Q64` | 8 MB | **measured** 2026-08-24, `esptool flash-id` `ef 7017` on both MCUs (FC `30:ed:a0:e3:64:9c`, OC `e0:72:a1:ca:e1:50`) |
| rocket-computer | V8 | `W25Q64` | 8 MB | predates the swap; confirmed by the board owner 2026-08-24 ("latest version only is 16 MB, others are 8"). No V8 artwork was ever committed, so there is no BOM to check it against |
| rocket-computer | V9 / V10 | U13 / U16 `GD25Q128ESIG` | 16 MB | BOM; and a V9 FC logged `Detected size(16384k)` against an 8 MB header |
| base-station | V1, V2 | `W25Q64` | 8 MB | BOM history + confirmed by the board owner 2026-08-24 ("latest version only is 16 MB, others are 8") |
| base-station | V3 (PCB V5/V6) | U1 `GD25Q128ESIG` | 16 MB | BOM |
| lora-daughterboard | as-built V3 | U22 `W25Q64JVXGIQ` | 8 MB | the board the firmware was written against |
| lora-daughterboard | current artwork | U22 `GD25Q128ESIG` | 16 MB | BOM — first article is unflashable, so untested |
| rocket-computer-mini | first article | U13 `GD25Q128ESIG` | 16 MB | BOM; board postdates the swap. **No hardware exists yet to measure** (2026-08-24) |
| base-station-mini | first article | U1 `GD25Q128ESIG` | 16 MB | BOM; board postdates the swap. **Unmeasured** |

**How firmware handles it.** Each project declares the *smallest* part it might
meet in `sdkconfig.defaults`, and the revisions known to carry the big part
raise it in a per-board overlay picked by the project `CMakeLists.txt` from the
board flag — the same mechanism `out_computer` uses for V9 PSRAM:

| project | base | overlay |
|---|---|---|
| `flight_computer` | 8 MB | `sdkconfig.defaults.v9` → 16 MB (`-DTR_BOARD_V9=1`) |
| `out_computer` | 8 MB | `sdkconfig.defaults.v9` → 16 MB (`-DTR_BOARD_V9=1`) |
| `base_station` | 8 MB + `partitions.csv` | `sdkconfig.defaults.v3` → 16 MB + `partitions_v3.csv` (`-DTR_BS_BOARD=3`) |
| `radio_board` | 8 MB | none — no board flag exists, so 8 MB has to cover both revisions |
| `rocket_computer_mini` | 16 MB | none — only one revision, and it postdates the swap |

The base value is deliberately the safe one, so a forgotten or unsupported flag
costs a big board the top of its part rather than costing a small board its
ability to boot.

`base_station` is the only project where flash size and partition layout are
coupled: V3 has no external NAND (`board_v3.h`, `HAS_EXT_NAND = false`), so
spiffs is its only log store and #835 item 1 grew it to 9.94 MB — a table that
does not fit in 8 MB at all. `gen_esp32part.py` refuses a table larger than the
declared flash, which is the check that fires if those two ever drift apart.

**Before flashing an unfamiliar board**, read the part and compare it to what
the build declares:

```
python tinkerrocket-idf/tools/check_flash_size.py     --port /dev/cu.usbmodem101     --build-dir tinkerrocket-idf/projects/flight_computer/build_v7
```

It reads the JEDEC id off the chip and the flash-size nibble out of the
`bootloader.bin` that would be flashed, and exits non-zero only for the fatal
direction.

**The trap that hid this for two days.** IDF does not re-apply
`sdkconfig.defaults` to an `sdkconfig` that already exists, and `idf.py
fullclean` does not delete one kept at the project root. A wrong default is
therefore invisible on any machine that has built before, and appears only in a
fresh build dir — a clean clone, CI, or the first time someone removes `build/`.
Every project now keeps its generated `sdkconfig` inside the build dir
(`set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")`) so that a changed default
takes effect where someone would look for it. Any root `projects/*/sdkconfig`
left over from before is now ignored and can be deleted.

## Current state

Fab set 2026-08-09 — all six boards tagged as a set at 5e3a426. Four fab
packages were exported and uploaded that evening (base-station v5, lora
v3, rocket-computer v9, sam10m8 v2), each verified geometry-identical —
every copper/mask/paste/silk/edge layer, timestamps aside — to plots
from the tagged commit, so the tags are the record of what was sent. No
export existed for servo-adapter or px1105r. The as-uploaded zips ride
on the `hardware-fab-2026-08-09` GitHub release (fab output stays out of
the tree, per hardware/.gitignore); reference plots at the tags cover
the two boards without an upload. DRC below is
`--severity-error --schematic-parity` at tagging.

| board | version | DRC at tagging |
|---|---|---|
| base-station | v5.0.0 | 0 errors ✓ |
| lora-daughterboard | v3.0.0 | 3 courtyard overlaps (review-waived), 0 unconnected |
| rocket-computer | v9.0.0 | 3 courtyard overlaps (review-waived), 0 unconnected |
| servo-adapter | v1.0.0 | 4 courtyard overlaps (J1 vs C1/C3/C4, waived); silkscreen still has no `${REVISION}` |
| gnss-sam10m8-18mm-hv | v2.1.0 | 0 errors ✓ |
| gnss-px1105r-18mm-highpower-ext-ant | v1.1.0 | 6 errors — stale zone fills from 8f87aa0 shorted +3V3 to two GND vias and the GND pour to two RXD2 tracks in the *stored artwork*; design intent sound. Refilled and cleared to 0 in the commit after the tag; verify which artwork the fab actually received. |

The gnss v1.1.0 / v2.1.0 tags supersede v1.0.0 / v2.0.0, which were never
fabricated. Copper changed in the 8f87aa0 net merge — strictly a major
bump — but the silkscreen majors stay V1 / V2 because no prior revision
ever reached a fab; the tag majors follow the silkscreen on the physical
boards.
