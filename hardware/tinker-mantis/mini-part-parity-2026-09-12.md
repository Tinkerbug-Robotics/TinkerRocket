# rocket-computer (V10) — part parity with the mini

**Date:** 2026-09-12, barometer added 2026-09-13. **Scope:** the V10 schematic,
its `bom.csv`, and the four footprints on the board whose land changed. Placement and routing of those three
are the owner's. Everything below was taken from `kicad-cli` netlist exports of
both boards.

`v10-power-parity-2026-09-11.md` closed the **circuit** differences between the
two boards, measuring from the TPS61094 rework (2026-08-29) forward. It did not
look further back, and the **part** swaps from `a4667604` (2026-08-16,
"hardware(**minis**): cost-reduction part swaps") never reached this board —
that commit touched the two mini boards only. Two of its six items had since
arrived here by other routes (`DTC123JETL` on 09-05, the `PA0805` shunt); the
rest had not. This pass closes that gap, plus the boot NOR the mini changed
again during its pre-fab session.

## 1. Carried over

| ref | was | is | why | land |
|---|---|---|---|---|
| `U13`, `U16` boot NOR | GD25Q128ESIG, SOIC-8 208 mil | **W25Q128JVYIQ**, 24-ball WLCSP | the mini moved to the WLCSP on 2026-09-04; fleet parity on one NOR | **changes** — SOIC-8 → 0.4 mm WLCSP |
| `U6`–`U8`, `U10` pyro high-side P-FET | TPN4R712MD (Toshiba) | **WSD20L50DN33** (Winsok) | the incumbent is unbuyable at build quantity until the Q4-2026 restock; land compatibility was closed on the mini by numeric overlay of the vendor drawing | same (`TSON Advance_TOS`) |
| `U9` pyro-ARM N-FET | CSD16323Q3 (TI) | **AON7534** (AOS) | same swap, same pass; pinout and land verified on the mini | same |
| `U3` magnetometer | IIS2MDCTR (ST), LGA-12 2×2 | **QMC5883P** (QST), LGA-16 3×3 | one magnetometer and one driver across the fleet | **changes** |
| `C6` (the `U3` C1 reservoir) | 220 nF | **10 uF** | what the QMC5883P's C1 pin wants, as fitted on the mini | same 0402 |
| `U4` barometer | BMP585, LGA-9 | **BMP581**, LGA-10 2×2 | owner's call, 2026-09-13 | **changes** |
| eleven 22 µF 0805 | CL21A226M**O**QNNNE | CL21A226**K**OQNNNE | tighter tolerance grade; `C141` was already the K part | same |

The two NOR placements keep every net; only the pad names change (1–8 → ball
designators). The magnetometer is not a pad-for-pad swap:

- `IIS2MDCTR_SCL` / `IIS2MDCTR_SDA` are renamed **`MAG_SCL` / `MAG_SDA`**, the
  mini's names — they name the slot, not the part sitting in it.
- The QMC land exposes **no DRDY/INT pad** (data-ready is a status-register bit,
  cleared by reading it). `IIS2MDCTR_INT` is gone and **P4 GPIO46 is now free**,
  carrying a no-connect.
- The QMC has a single `VDD`; the ST part's separate `VDD_IO` and its `CS`
  strap to `V_MCU_SWTCH` are gone with it.

The barometer **is** a drop-in on the drawing: the two symbols are built to the
same pin grid, so `U4` keeps its placement and every wire and label, and only the
pad numbers change (SCK 1→2, SDI 2→4, SDO 3→5, VDDIO 4→1, INT 5→7, CSB 7→6,
VDD 8→10, and the BMP585's single `VSS` becomes the BMP581's three). The
BMP585's pin 9 `L/M` has no counterpart and was already a no-connect. The nets
keep the mini's names — `BMP585_CS` / `BMP585_INT` are what the mini calls them
even with a BMP581 fitted, so leaving them is the parity choice; say the word if
you want them renamed on both boards. On the **board** it is not a drop-in:
LGA-9 → LGA-10 2×2 is a different land, so `U4` comes off with the others.

## 2. Deliberately not carried

| mini change | why not |
|---|---|
| `U19` eFuse TPS259824LNRGET → TPS259631DDAR | different job on this board. `U19` here feeds the **entire avionics complex** (S3, P4 and its core buck, all three flash, the sensor suite, INA230, USB switch, buzzer, pyro continuity sense) — see `WORKLIST.md`. The 15 A / 2.7 mΩ part is the deliberate choice, and a current limit is a protection decision, not a parity one. |
| `U11` log NAND 2 Gbit → 1 Gbit | the mini downsized for cost (−$3.81/bd) on the cheaper board. The V10 is the full logger. Owner's call, 2026-09-12: **stays at 2 Gbit.** Same land, same pinout, and the driver reads RDID at runtime, so this is reversible as a value change whenever the cost matters. |
| everything in `v10-power-parity-2026-09-11.md` §2 | already classified there — ported, does-not-port, or mini-specific. |

## 3. What is left for the owner

- **Place four parts.** `U3`, `U4`, `U13` and `U16` were removed from the board along
  with the copper that existed only to reach them; the schematic has them, so
  they arrive unplaced on the next *Update PCB from Schematic*. Nothing else on
  the board moved.
  - The two NORs are 0.4 mm-pitch WLCSP. The mini's V1.0.1 had to take the vias
    **out of the WLCSP ball pads** after a JLCPCB engineering query — the same
    constraint applies here.
  - The mini puts a 2 × 2 mm all-layer no-pour rule area under the magnetometer
    die. It lives on the board, not in the footprint, so it does not travel with
    the part.
  - `U4`'s land grows from LGA-9 to LGA-10 2×2; the mini's placement is the
    reference for both it and the magnetometer beside it.
- **Firmware.** `board_v9.h` still declares `IIS2MDC_INT = 46` and selects the ST
  driver. The QMC5883P driver already exists — `TR_QMC5883P` behind
  `TR_Sensor_Collector`'s `TR_MAG_DRIVER_QMC5883P` seam, as `board_m1.h` uses it
  — but the V10 board header has to select it, drop the INT pin, and settle the
  **axis mapping**, which is still open on the mini. The barometer needs nothing:
  the BMP581 and BMP585 share the BMP5xx register map and the mini already runs
  the BMP581 through the same driver.
- **Cost.** The NOR swap gives back part of the `ee41b18c` saving (that commit
  moved the whole fleet to the GigaDevice SOIC-8 for −$1.05/pc over a 700 pc
  line). Worth re-pricing against the workbook before the next order.

## 4. Verification record

- **Netlist** (kicadxml, refdes-keyed diff, pre vs post): component count
  unchanged at 277. Only `U3`, `U4`, `C6`, `U6`–`U10`, `U13`, `U16` change value
  or footprint. Every net keeps its members; the NOR and magnetometer nets change
  only pad names. New: 32 `unconnected-(U13/U16-NC-Pad…)` and 10 for the QMC's
  NC balls, `MAG_SCL` / `MAG_SDA` in place of the `IIS2MDCTR_*` pair, and
  `unconnected-(U17A-GPIO46-Pad88)`.
- **ERC** `--severity-all`: 982 → 918, still **0 errors**. `pin_to_pin` 97 → 53
  and `lib_symbol_mismatch` 44 → 36, because the new symbols type their pins and
  are byte-identical to `Custom.kicad_sym`.
- **DRC** `--severity-all --schematic-parity`: violations 173 → 162 (dangling
  tracks 12 → 7, courtyard overlaps 4 → 3, silk overlaps 60 → 55), unconnected
  items 8 → 7, and schematic parity **202 → 97** (`net_conflict` 130 → 47).
  `missing_footprint` 26 → 30: the four parts above, and no others. Every one of
  the seven dangling tracks that remain was dangling at HEAD.
- **Copper**: 254 objects removed (239 tracks, 15 vias), every one of them on a
  net that reached a removed pad and dangled without it. Signal nets first (171
  for the flash and magnetometer, 73 for the barometer), then the ten `+3V3` and
  `V_MCU_SWTCH` orphans the four parts' power pads left behind — each traced back
  to a pad position taken from HEAD, none of them vias. The `FLASH_CS`,
  `MAG_SCL` and `MAG_SDA` branches to their pull-ups (`R46`, `R40`, `R43`) and
  the shared `SENS_SCLK` / `SENS_SDI` / `SENS_SDO` buses serving the other
  sensors are untouched, and no `GND` copper was pruned: none of its danglers
  trace to a removed pad. Zones refilled after each step: total filled copper
  area 6881.8 → 7076.3 mm², the pours closing over the freed space.
- **BOM** regenerated and reconciled: all 265 designators agree with a fresh
  `kicad-cli sch export bom`.
