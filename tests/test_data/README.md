# Test-data fixtures

Binary flight logs (`flight_*.bin`) used as parametric inputs by:

- `tests/integration/test_bin_replay.py` — board-variant invariants
  (CRC pass rate, timestamp monotonicity, sensor frame rates, IMU gap
  thresholds, board-specific magnetometer expectations).
- `tests/test_flight_report.py` — smoke test for the flight_report suite
  (`Data_Analysis/flight_report/`).

Each `.bin` must be accompanied by a sidecar JSON declaring which PCB
revision produced the capture:

```json
{ "board": "old" }   # MMC5983MA over SPI; no IIS2MDC frames expected
{ "board": "new" }   # IIS2MDC over I2C; no MMC5983MA frames expected
```

Naming: `<binfile-stem>.meta.json` (e.g., `flight_20260615_170318.meta.json`).
See `tests/integration/conftest.py:117` for the convention. Without a
`.meta.json`, `board_variant()` falls back to `BOARD_OLD` so the
integration tests will fail on a new-PCB capture.

Note: `<binfile-stem>.json` (no `.meta` infix) is the iOS app's flight
summary file (apogee/burnout/max altitude/max speed) — written by
TinkerRocketApp and intentionally not consulted by the integration tests.
