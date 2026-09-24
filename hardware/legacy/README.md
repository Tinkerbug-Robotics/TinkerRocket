# Legacy boards

Boards that are no longer part of the product line (see
[`../README.md`](../README.md#product-line)), kept whole so that nothing is lost.
Each one still opens in KiCad, still passes the parity gate
(`tools/check_board_parity.py` scans this folder too) and can still be
re-plotted.

| Folder | Board | Latest tag | Tree | Why it left the line |
|---|---|---|---|---|
| [`base-station/`](base-station/) | Full ground station, paired with a [`lora-daughterboard`](../lora-daughterboard/) over UART | `base-station-v5.0.0` | V6 | Replaced by [Tinker-Base](../tinker-base/), which carries its radio on board |
| [`gnss-px1105r-18mm-highpower-ext-ant/`](gnss-px1105r-18mm-highpower-ext-ant/) | GNSS carrier, PX1105R, external antenna | `gnss-px1105r-18mm-highpower-ext-ant-v1.1.0` | V2 | Superseded by the SAM-M10Q carrier, [`gnss-sam10m8-18mm-hv`](../gnss-sam10m8-18mm-hv/) |
| [`servo-adapter/`](servo-adapter/) | Passive cable-to-servo adapter with a capacitor, for the rocket computer's expansion port | `servo-adapter-v1.0.0` | no rev in the title block | Not offered |

## What moving them changed

Only paths. One folder deeper, `${KIPRJMOD}/../` became `${KIPRJMOD}/../../` in
each board's `sym-lib-table`, `fp-lib-table` and 3D-model references, so the
shared `hardware/symbols/`, `hardware/footprints/` and `hardware/3dmodels/`
still resolve. Checked when they moved: netlist, BOM export, every gerber and
drill file (timestamps aside) and the DRC report are identical to before.

## Plotting and tags

```bash
tools/plot_gerbers.sh legacy/base-station
```

(`tools/plot_gerbers.sh base-station` finds it too.) Tags keep their names —
`<board>-v<semver>` as in [docs/board-versioning.md](../../docs/board-versioning.md) —
and every tag predates the move, so at a tag the board is still at
`hardware/<board>/`, and that tag's own `tools/plot_gerbers.sh` is the one to use.

**Do not plot the PX1105R carrier from `gnss-px1105r-18mm-highpower-ext-ant-v1.1.0`.**
That tag sits on 5e3a426, whose stored zone fills (left stale by 8f87aa0) short
+3V3 to two GND vias. They were refilled in 8a95e02, the next commit to touch
the board; plot from there or later, and re-point the tag to whatever commit is
actually sent if that board is ever ordered.

## Firmware

- **base-station** runs `tinkerrocket-idf/projects/base_station`. Its V1/V2 pin
  maps are in `main/board/legacy/`; the V3 map (this board, PCB V5/V6) stays in
  `main/board/` because its image fills the product release's Tinker-Base slot
  until the Tinker-Base's own map (`board_v4.h`) has run on hardware. The V2
  image is published on `fw-legacy-v*` tags (a V1 board is built by hand,
  `-DTR_BS_BOARD=1`), V3 on the product `fw-v*` tags.
- **gnss-px1105r** and **servo-adapter** carry no firmware.
