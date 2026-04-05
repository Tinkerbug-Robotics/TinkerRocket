# TinkerRocket TODO List
*Updated: April 3, 2026*


Inflight reboot
Multiple flight computer units
Automated test approach for changes
Higher current consumption before turning on, went from ~10ma to ~20ma
Inconsistent power data update rate on OutComputer


## Critical — Fix Before Next Flight

### I2S Data Pipeline
- [x] **Stale MRAM data in log files** — When logging starts, old MRAM data from previous sessions leaks into the log file despite `clearRing()`. Fixed by adding timestamp monotonicity filter in `processFrame()`: frames whose timestamp is >5s behind the latest seen timestamp are rejected as stale. Diagnostic counter `stale_drops` added to I2S logging.
- [ ] **Max speed shows unrealistic values** — The JSON metadata file shows `max_speed_mps: 16480451.7` even though bin file data is clean. The bogus speed is computed on the OC from corrupted live telemetry values (not from logged frames). Need to clamp velocity in the live telemetry path, not just in the logger stats.



## Software — Non-Critical Improvements

### App / GUI
- [x] **File deletion in GUI** — Files don't always update properly after deletion. Need to refresh file list after delete operation.
- [ ] **Logging button state lag** — App button state updates from BLE telemetry JSON, which has 1-second latency. Consider optimistic UI update on button press.

### Data Logging
- [x] **Pre-launch buffering with MRAM** — Re-enabled now that `clearRing()` zeros MRAM and `processFrame()` has a timestamp monotonicity filter. `enqueueFrame()` now accepts frames when `file_open` (PRELAUNCH) in addition to `logging_active`.
- [ ] **NVS persistence on ESP-IDF** — Preferences `NOT_FOUND` errors on first boot. Works after first save. Consider seeding default NVS values on first boot.
- [ ] **Timestamp rename during logging** — Currently deferred to end-of-logging. For auto-launch logging, the file gets a timestamp name only after landing. Consider applying timestamp from GNSS time sync when available.

### RunCam Camera
- [ ] **Camera state management** — Boot with power off, first press powers on + auto-records after 5s. Stop just sends UART toggle. Need to test with actual RunCam Split 4.
- [ ] **No auto-start on launch** — Camera is purely manual. May want option to auto-start on PRELAUNCH state for flight cameras.

### LoRa / Base Station
- [ ] **Base station converted to ESP-IDF** — Builds and runs. WDT fix applied. Need field testing.
- [ ] **LoRa uplink commands** — Base station can send commands to rocket (start/stop logging, etc.). Not tested on new board.

### Code Cleanup
- [x] **Rename I2C variables to I2S** — `i2c_tx_queue`, `i2c_tx_ok`, `i2cSendWithStats`, etc. still named for I2C even though they use I2S.
- [ ] **Remove mag debug prints** — Removed from TR_MMC5983MA.cpp but verify no debug prints remain in other files.
- [ ] **Arduino IDE compatibility** — The shared source files (symlinked) have `#include <esp_log.h>` and `ESP_LOGI()` calls that won't compile on Arduino IDE. Need `#ifdef` guards or separate builds.

### Ground Test
- [ ] **Roll-only mode servo behavior** — Ground test in roll-only mode tries to hold a specific roll angle instead of just nulling roll rate. Need to check control mode in ground test code path.
- [ ] **Ground test EKF wait indicator** — App shows "Waiting for EKF..." when starting ground test without GPS. Good UX improvement.

## Performance Benchmarks (Current State)

### I2S Data Pipeline
- **IMU (ISM6HG256)**: 960 Hz target → ~960 Hz achieved (100%)
- **Barometer (BMP585)**: 500 Hz target → ~490 Hz achieved (98%)
- **Magnetometer (MMC5983MA)**: 200 Hz target → ~220 Hz achieved (110%)
- **Non-Sensor (EKF output)**: 500 Hz target → ~490 Hz achieved (98%)
- **GNSS**: 18 Hz target → 34 Hz achieved (via I2C)
- **Frame drops during logging**: 0
- **Ring buffer overflows**: 0
- **Max IMU gap**: 9.4ms
- **Gaps >10ms**: 0 in 30-second test

### Power Consumption
- **Standby (BLE advertising)**: ~12 mA with light sleep + BLE modem sleep
- **Active (all sensors + logging)**: ~130 mA
- **Battery life (600mAh, standby)**: ~50 hours

### I2S Configuration
- **Sample rate**: 22050 Hz (88 KB/s bandwidth)
- **Data rate**: ~68 KB/s framed sensor data
- **Idle fill**: ~50% of bandwidth (zeros between frames)
- **DMA**: 8 descriptors x 256 frames, zero-copy callbacks
- **Buffer**: 128 KB MRAM ring → NAND flash via LittleFS
