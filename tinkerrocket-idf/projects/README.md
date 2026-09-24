# Firmware projects

Which project runs on which board, and which of them are released. The board
names are the product names from 2026-09-24 — see
[`hardware/README.md`](../../hardware/README.md#product-line).

## Product line

Published on `fw-v*` tags by
[`firmware-release.yml`](../../.github/workflows/firmware-release.yml); both apps
find, verify and flash these.

| Product | Project | Build | Chip | Release image |
|---|---|---|---|---|
| **Tinker-Mantis** (V9/V10) | [`flight_computer`](flight_computer/) | `idf.py -B build_v9 -DTR_BOARD_V9=1 build` | ESP32-P4 | `tinker-mantis-flight_computer-V9.bin` |
| | [`out_computer`](out_computer/) | `idf.py -B build_v9 -DTR_BOARD_V9=1 build` | ESP32-S3 | `tinker-mantis-out_computer-V9.bin` |
| **Tinker-Beetle** | [`flight_computer`](flight_computer/) | `idf.py -B build_m1 -DTR_BOARD_M1=1 build` | ESP32-S3 | `tinker-beetle-flight_computer-M1.bin` |
| | [`out_computer`](out_computer/) | `idf.py -B build_m1 -DTR_BOARD_M1=1 build` | ESP32-S3 | `tinker-beetle-out_computer-M1.bin` |
| **Tinker-Base** | [`base_station`](base_station/) | `idf.py -B build_v4 -DTR_BS_BOARD=4 build` | ESP32-S3 | `tinker-base-base_station-V3.bin` — the V3 image, until V4 has run on hardware |
| LoRa daughterboard (Tinker-Mantis) | [`radio_board`](radio_board/) | `idf.py build` | ESP32-S3 | `lora-daughterboard-radio_board.bin` |

**The Tinker-Base's map is `main/board/board_v4.h`** (`-DTR_BS_BOARD=4`, stamped
`-v4`): the on-board E220 radio over SPI — the direct path the V1/V2 boards
already take, on the same GPIOs the `lora-daughterboard` uses — no I²C bus, no
fuel gauge, no pack charger, and the cell read on V3's divider. It is
netlist-verified and builds in CI, but no Tinker-Base has been built, so it has
not run. Until a first article proves it, the release's Tinker-Base slot keeps
carrying the V3 image — the full base station it forked from, now
`hardware/legacy/base-station` — which does not run on a Tinker-Base; swap the
release entry to V4 then.

## Why the project names did not change

The folders and CMake `project()` names are the ones the boards already run.
Both apps read `project_name` out of an image's header and refuse to flash one
that is not the unit's own (`EspImage.kt` / `EspImage.swift`), and the release
catalog filters on it — so renaming `flight_computer` would make every board in
the field refuse its next update. The product name lives in the release asset
name, the board code (`V9`, `M1`, `-DTR_BS_BOARD=3`) selects the pin map, and a
single project serves more than one product: `flight_computer` and
`out_computer` build both the Tinker-Mantis and the Tinker-Beetle.

## Legacy

Firmware for boards that are no longer offered. CI still builds all of it on
every push (`firmware-build.yml`, the `legacy` jobs) so it does not rot, and
[`firmware-release-legacy.yml`](../../.github/workflows/firmware-release-legacy.yml)
publishes it on `fw-legacy-v*` tags. The apps never offer those; flash an image
with the app's file picker or over USB.

| Board | Where | Build |
|---|---|---|
| Rocket computer V8 | `flight_computer`, `out_computer` — `main/board/legacy/board_v8.h` | `-B build_v8 -DTR_BOARD_V8=1` |
| Rocket computer V7 | `flight_computer`, `out_computer` — `main/board/legacy/board_v7.h` | `-B build_v7 -DTR_BOARD_V7=1` |
| Base station V1/V2 | `base_station` — `main/board/legacy/board_v{1,2}.h` | `-DTR_BS_BOARD=1`, or no flag for V2 |
| Single-MCU mini (never fabbed) | [`legacy/rocket_computer_mini`](legacy/rocket_computer_mini/) | `idf.py build` |

The legacy pin maps stay inside the projects they belong to because those
projects are the product firmware too: a V8 build is the same code as a
Tinker-Mantis build with a different map, and forking the code to separate them
would split every future fix in two.

## Bench tools

[`bench/`](bench/) — `pyro_channel_test`, `power_test`, `max17205_test`,
`gpio_wire_rx`/`gpio_wire_tx` and `i2s_rx_test`/`i2s_tx_test`. Hardware
bring-up utilities, never released. `pyro_channel_test` drives squib FETs by
hand; CI builds it for every rocket-computer map so none of them rots.
`gpio_wire_rx`/`gpio_wire_tx` and `i2s_rx_test`/`i2s_tx_test` still require the
`arduino` component the firmware has since dropped, so they do not configure
today; `power_test` and `max17205_test` build.
