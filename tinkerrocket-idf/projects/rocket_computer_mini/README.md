# rocket_computer_mini

Firmware for [`hardware/rocket-computer-mini`](../../../hardware/rocket-computer-mini/):
one ESP32-S3 doing both of the big rocket-computer's jobs. The board deleted
the ESP32-P4, so this project is a merge of
[`flight_computer`](../flight_computer/) (sensors, EKF, orientation, kinematic
detectors, four pyro channels, mag/sensor calibration, sim) and
[`out_computer`](../out_computer/) (NAND flight logging, LoRa telemetry, BLE
to the app, battery monitoring, OTA, power modes) into one process.

**Everything external stays fleet-compatible.** The NAND log stream is
byte-identical to the OC's (same `AA55AA55|type|len|payload|CRC16` frames), so
the flight-report tooling and BLE download parse mini logs unchanged. On the
two-MCU board the OC logged the FC's `OUT_STATUS_QUERY` frames as received;
the mini has no query wire, so it builds and logs the same sensor-config
snapshot itself (`logOutStatusQuery()` alongside each FlightSettings emission)
— v6 of that frame stamps `mag_type`, which is how the analysis tooling picks
the mag count scale per log (IIS2MDC 0.15 µT/LSB vs the mini's QMC5883P
100/3750 µT/LSB, #797). The LoRa
air protocol is v4 — existing base stations work. The BLE service and
characteristics are the OC's — the apps connect without modification (they
will show servo/camera controls this board answers with "unsupported").

## What replaced the inter-MCU link

The two-MCU board moved data over I2S (telemetry), I2C (commands) and five
wires of ceremony. Here that link is [`main/mini_link.h`](main/mini_link.h) —
the same message frames, no wires:

| was | is now |
|---|---|
| FC → OC I2S sensor/telemetry stream | `mini_link::logFrame()` straight into the log ring, plus the `mini_link::telem` latest-state cache the comms side reads |
| OC → FC I2C pending-command dance | `mini_link::cmd_queue` carrying the identical `*_MSG` frames; the flight loop drains it once per tick |
| OC MRAM crash snapshot (#261) | `SNAPSHOT_MSG` at 10 Hz into the NAND log; boot-time tail-scan restore (see *ACTIVE-restore* below) |

Porting stayed deliberately line-faithful on both sides so future
`flight_computer`/`out_computer` fixes cross-apply; where behavior deviates,
the code comments say so and why.

## Task and core layout (ACTIVE mode)

| task | core | prio | source |
|---|---|---|---|
| `flight` | 1 | max−1 | flight loop — free-running, task-WDT subscribed, alone on its core like the FC's |
| `Poll IMU Data` | 0 | 4 | SensorCollector (unchanged) |
| `Poll GNSS Data` | 0 | 3 | SensorCollector (unchanged) |
| `comms_loop` | 0 | 2 | the `oc_loop` successor |
| `log_flush` | 0 | 1 | TR_LogToFlash |
| NimBLE host | — | IDF | TR_BLE_To_APP |

The I2S sender/parser, OTA feeder and I2C slave tasks of the two-MCU design
do not exist here — that is where the merged budget comes from. The EKF is
the piece to re-benchmark: the P4 measured ~554 µs per pass at decimation 2;
the S3 runs the same code at 240 MHz. `EKF_DECIMATION` lives in
[`main/config_flight.inc`](main/config_flight.inc) with the sanctioned
fallback (4) documented — detector hysteresis counts assume the ~1 kHz gated
loop and must not be silently retuned.

## Power model

`PWR_PIN` (GPIO3) gates the `V_MCU_SWTCH` rail feeding **all** peripherals:
IMU, mag, baro, GNSS (VCC *and* V_BCKP), radio, NAND. Only the MCU, its boot
flash and the INA230 are always-on.

- **IDLE** (boot default): rail off, BLE advertising, INA230 polling, light
  sleep armed (#519 machinery carried from the OC — 32 kHz crystal, USJ
  grace, XT WDT). Pyro FIRE/ARM are driven low from the first lines of boot.
- **power-on** (BLE command 8): lock 240 MHz, raise the rail, settle, bring
  up NAND+radio, then the whole flight stack. GNSS is a cold start every
  time — V_BCKP dies with the rail — so first fix is ~30 s.
- **power-off**: refused while in flight / pyro busy / cal running / log not
  finalized; otherwise drop the rail and `esp_restart()` into clean IDLE
  (SensorCollector has no deinit; a reset **is** the teardown). The rail must
  stay low ≥1.2 s (`PERIPH_RAIL_OFF_MIN_MS`) — Quectel requires ≥1 s below
  100 mV for a clean GNSS restart.

**ACTIVE-restore.** On the big board a flight-task wedge reboots only the P4
while the OC holds the rail and the MRAM snapshot restores state. Here every
reset is whole-system and the boot default is IDLE, so `main.cpp` persists
"ACTIVE was commanded" in NVS: an unexpected reset while ACTIVE auto
re-enters, and the flight side scans the tail of the interrupted flight's
NAND log for the last valid snapshot frame to restore EKF/flight state.
Capped at 3 consecutive attempts (crash-loop guard), forgiven after 60 s of
healthy ACTIVE.

## GNSS: LC86G, not u-blox

The mini's receiver is a Quectel LC86G (UART, NMEA + PAIR/PQTM). The new
[`TR_GNSSReceiverLC86_Serial`](../../components/TR_GNSSReceiverLC86_Serial/)
component mirrors the u-blox driver's exact public surface;
`TR_Sensor_Collector` selects it at compile time via
`add_compile_definitions(TR_GNSS_DRIVER_LC86=1)` in this project's
CMakeLists. The load-bearing sentence is proprietary `$PQTMPVT` — the only
output with **down velocity**, which `TR_GpsInsEKF` hard-fuses at σ=0.3 m/s
with no disable flag. If `$PQTMPVT` cannot be enabled the driver reports
GNSS-absent rather than synthesize a zero climb rate. The parser is IDF-free
and host-tested (`tests_cpp/test_lc86_parser.cpp`).

Driver policy: stay at the module's default 115200 baud (no reboot-to-apply
dance, ample for ~200 B/epoch at 10 Hz), reconfigure on every `begin()`
instead of saving to module flash (a rail cycle cold-starts it anyway).

## Board map

`main/board/board_v1.h`, selected by `-DTR_MINI_BOARD=<n>` (default 1, the
base-station integer-flag dialect). **The pin map is netlist-verified** from
a `kicad-cli` export of the mini schematic — the hardware repo's
`pin-budget.md` assignment table predates several moves and disagrees; the
netlist wins. Only pins and part-presence flags live there; policy stays in
the two `config_*.inc` fragments (`config_flight.inc` = FC ancestry,
`config_comms.inc` = OC ancestry).

## Capability deltas vs. the two-MCU board

Kept: full sensor suite at full rates, 15-state EKF, voting apogee/landing
detectors, four pyro channels with the FC's complete safe-init discipline,
NAND flight logging with auto-evict, LoRa telemetry + uplink + hopping, BLE
app link, sim mode, mag/sensor calibration, OTA self-update with rollback.

Gone with the hardware: servos → roll control, fin config, servo replay and
**guidance** (excluded from the build entirely); camera; piezo; MRAM (crash
snapshot now rides the NAND log, and anything still in the 128 KB RAM ring at
a hard reset — ~1 s of data — is lost); Wi-Fi stays off (power budget: the
one ~350 mA load the rail was never sized to carry alongside a radio burst).

## Build / flash

```
. $IDF_PATH/export.sh          # ESP-IDF v6.0.x, same as CI
idf.py build                   # board rev 1 implicit
idf.py -B build_b2 -DTR_MINI_BOARD=2 build   # future revisions
idf.py flash monitor           # console is USB-Serial-JTAG (UART0 pins are spent)
```

CI builds this project in the firmware matrix alongside the other four.
Firmware identity (`PROJECT_VER` = git sha + build date) flows into the BLE
`config_identity` "fw" field and OTA metadata, so releasing updates for this
board and the big one is the same act: merge, let CI build both images, and
ship each board its own `.bin` over BLE per
[`docs/plans/08-ota-firmware-update.md`](../../../docs/plans/08-ota-firmware-update.md).

## Hardware moved under this port (PR #797)

The cost-reduction sweep landed while this firmware was being written and
swapped two sensors in place — **every MCU pin net is unchanged** (verified
against a fresh netlist export), so the board map stands:

- **BMP585 → BMP581**: no firmware change needed — the Bosch BMP5 driver
  accepts both chip IDs (0x50/0x51), and the register map is shared.
- **IIS2MDC → QMC5883P**: the magnetometer driver does NOT exist yet. Until
  it does, the collector's IIS2MDC probe fails cleanly into the established
  no-magnetometer path: the EKF flies on GNSS-course + accel-match heading,
  and the scorecard shows SH_MAG absent. Mag calibration flows are inert.
  The QMC5883P driver + collector seam is tracked as its own follow-up.

## Before first flight — bench items this port cannot settle

- **Sensor rotation constants** (`ISM6HG256_ROT_Z_DEG`, `IIS2MDC_ROT_Z_DEG`)
  are inherited from the FC and are **placement facts of a different PCB** —
  re-derive for the mini layout (marked VERIFY in `config_flight.inc`).
- **`$PQTMPVT` at 10 Hz**: the protocol doc doesn't state whether PQTM
  messages follow the fix rate above 1 Hz. The driver warns if the observed
  epoch rate is low; confirm on the bench.
- **EKF/loop timing on the S3**: profile with the TASKCPU dump
  (`PROFILE_TASK_CPU`); fall back to `EKF_DECIMATION = 4` if the loop can't
  hold ~1 kHz.
- **Radio RXEN behavior**: the base station was once bitten by RXEN floating
  in RX; this board drives it (GPIO44) via `TR_LoRa_Comms` — verify RX
  sensitivity on the bench against a known-good board.
- **Shared SPI3 bus under load**: NAND page bursts + radio transactions
  interleave under the IDF driver's per-transaction locking; watch the log
  flush stats while telemetering.
- **Continuity sense polarity live**: the circuit reads LOW = igniter
  present (pull-up to the switched rail, R73 return); confirm with a real
  igniter before trusting the scorecard.
- **IDLE I²C pull-up leak (accepted)**: the internal pull-ups keep the
  always-on INA230 reachable while the rail is down, at the cost of tens of
  µA sourced into the unpowered magnetometer's clamp diodes. Measure the
  IDLE floor; if it matters, the fix is hardware (always-on external
  pull-ups), not firmware.
