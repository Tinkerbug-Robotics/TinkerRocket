# I2C Driver Migration: Legacy → New ESP-IDF 5.x API

## Context
The old I2C driver (`driver/i2c.h`) produces deprecation warnings on every build. Two components need migration to the new API (`driver/i2c_master.h` + `driver/i2c_slave.h`).

## Components to Migrate

### 1. TR_INA230 (simpler — master-only)
**Files:** `components/TR_INA230/TR_INA230.h` + `.cpp`
**Used by:** OC (I2C_NUM_1, addr 0x40, SDA=7, SCL=8, 400kHz)

**Old API calls:**
- `i2c_param_config()` + `i2c_driver_install()` → init
- `i2c_cmd_link_create/delete` + `i2c_master_*` → register read/write

**New API replacement:**
- `i2c_new_master_bus()` + `i2c_master_bus_add_device()` → init
- `i2c_master_transmit()` → register write (3 bytes: reg + 16-bit value)
- `i2c_master_transmit_receive()` → register read (write reg addr, read 2 bytes)
- Constructor change: accept `i2c_master_bus_handle_t` from caller instead of `i2c_port_t`

**Reference:** Base station already uses new master API for MAX17205G fuel gauge — same pattern.

### 2. TR_I2C_Interface (complex — master + slave)
**Files:** `components/TR_I2C_Interface/TR_I2C_Interface.h` + `.cpp`
**Used by:** FC as master (I2C_NUM_0, SDA=41, SCL=42, 400kHz) + OC as slave (I2C_NUM_0, addr 0x42, SDA=4, SCL=5, 1.2MHz, rx=8192, tx=256)

**Master side (FC):**
- `beginMaster()`: `i2c_new_master_bus()` + `i2c_master_bus_add_device()`
- `sendMessage()`: Replace cmd_link with `i2c_master_transmit(dev, frame, len, timeout)`
- `masterRead()`: `i2c_master_receive(dev, buf, len, timeout)`
- `getOutReady()`: `i2c_master_transmit()` then `i2c_master_receive()`

**Slave side (OC):**
- `beginSlave()`: `i2c_new_slave()` with `i2c_slave_config_t`
- `readFromSlave()`: `i2c_slave_receive()` (blocking or callback)
- `writeToSlave()`: `i2c_slave_transmit()`
- **Note:** New slave API has no shared FIFO — uses explicit receive/transmit calls

**Header changes:**
- `#include <driver/i2c.h>` → `#include <driver/i2c_master.h>` (master) / `#include <driver/i2c_slave.h>` (slave)
- Store `i2c_master_bus_handle_t` + `i2c_master_dev_handle_t` for master mode
- Store `i2c_slave_dev_handle_t` for slave mode

### 3. Project main.cpp updates
- **OC:** INA230 init — create bus, pass handle to INA230 constructor
- **FC:** Minimal changes if TR_I2C_Interface public API stays similar
- **BS:** Already uses new API — no changes needed

## Verification
1. Build all 3 projects — no I2C deprecation warnings
2. FC→OC I2C communication works (telemetry flows via I2S, commands via I2C)
3. INA230 power readings work on OC (battery voltage/current in BLE telemetry)
4. Run sim from app — full data pipeline FC→OC→BLE→app

## Session Status (end of 2026-03-27)

### Completed
- All 3 projects (FC, OC, BS) migrated from Arduino to pure ESP-IDF
- TR_Compat shim (millis/delay/GPIO/SPI/attachInterrupt)
- TR_NVS (Preferences drop-in replacement)
- RadioLib → native EspHal
- BLE → direct NimBLE API
- GNSS → ESP-IDF UART
- All 3 projects build clean
- FC sensors all working (ISM6 at 935/s, BMP at 485/s, MMC at 222/s, GNSS at 18/s)
- OC boots, BLE connects, LoRa initialized
- BS builds clean

### Remaining Issues (Priority Order)

#### P0 — RESOLVED
1. ~~**I2S data not reaching OC**~~ — **FIXED.** The I2S was working all along. The issue was ISM6 DRDY interrupt not firing, so NonSensorData was never populated → OC saw zeros. Once the ISM6 polling fallback was added, real data flowed through I2S → OC → BLE → app.

2. ~~**OC NimBLE BLE telemetry**~~ — **WORKING.** Data now reaches the iOS app after the ISM6 fix.

#### P1 — Functional but degraded
3. **ISM6 SPI MODE3 not working** — ESP-IDF `spi_master` on ESP32-P4 doesn't work with MODE3 for the ISM6HG256. WHOAMI returns 0x00 with MODE3 but works with MODE0. MODE0 is functionally correct per datasheet, but the GPIO DRDY interrupt doesn't fire (edge missed). Added polling fallback that works (935 reads/s). Root cause unknown — may be ESP32-P4 SPI driver bug with CPOL=1.

4. **ISM6 DRDY interrupt not firing** — `attachInterrupt` on GPIO 21 registers correctly but ISR never triggers. The DRDY pin goes HIGH once and stays HIGH. Added `digitalRead` polling fallback in `pollIMUdata` task which works. The interrupt issue may be related to the compat `attachInterrupt` wrapper or ESP32-P4 GPIO interrupt behavior. BMP585 and MMC5983MA interrupts work fine on other GPIO pins.

5. **First-boot crash** — FC crashes during sensor init on cold boot (USB-UART reset), then boots fine on warm restart. Backtrace points to `esp_log_write` during `gpio_config`. Likely a stack overflow or memory issue during the very first boot. Not blocking since warm restart works.

#### P2 — Cleanup
6. **I2C driver migration** — Old `driver/i2c.h` API produces deprecation warning. TR_I2C_Interface and TR_INA230 need migration to `driver/i2c_master.h` + `driver/i2c_slave.h`. See plan above.

7. **Pyro continuity test** — Was the original goal of the session. GPIO 14-19 are now free (ESP-Hosted removed). Need to retest pyro continuity now that the FC boots clean. The compat `pinMode()` no longer calls `gpio_reset_pin()` — the pyro `initPyroPins()` does its own `gpio_reset_pin()` explicitly.

8. **Pyro channel programming** — The pyro settings NVS persistence, config readback after power-on, apogee detection voting, and iOS app tile UI were all implemented earlier in the session but not fully tested after the Arduino migration.

9. **Flight sim fidelity** — Pitch dynamics model was planned but not implemented (see old plan in `.claude/plans/`). Needed for realistic sim testing of apogee detection and pyro firing.

10. **Compiler warnings cleanup** — `volatile++` suppressed with pragma. Some unused variables marked. The IRAM_ATTR section conflicts were fixed. May still be warnings in OC/BS builds.
