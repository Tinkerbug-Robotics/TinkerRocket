# TinkerRocket

Full featured flight computer with 1 kHz logging, remote control power 'WiFi' switch, four pyro channels, camera control, and control of up to six servos for active roll control, guidance, or other functions. Downlink and GPS tracking via a LoRa radio to ground station and companion iOS app for configuration, monitoring, and voice call outs during flight.

<!-- TODO: Add hero photo of the rocket -->
![TinkerRocket](docs/images/rocket_hero.jpg)

## Overview

The system comprises three physical cooperating components:

| Component | Hardware | Role |
|-----------|----------|------|
| **Flight Computer** | ESP32-P4 & ESP32-S3 | Sensor fusion, EKF, guidance, servo control, data logging, LoRa downlink, BLE telemetry |
| **Base Station** | ESP32-S3 | LoRa receiver, BLE gateway, SD card logging |
| **iOS App** | iPhone/iPad | Real-time dashboard, file management, configuration |

The onboard computer has both the ESP32-P4 main processor with two cores running at 400 MHz for sensor intake, flight processing, and controls. An ESP32-S3 serves as the WiFi/BlueTooth LE radio as well as high speed data logger and LoRa radio control. To support guidance and control functions, the onboard flight computer runs a 15-state Extended Kalman Filter fusing IMU, barometer, magnetometer, and GNSS data at up to 1000 Hz. Optional roll control or, a proportional navigation guidance law commands four fin-tab servos through cascaded PID controllers with velocity-based gain scheduling. There are four fully programmable pyro channels. There is also an interface to power and control an on board camera, with RunCam Split4 and GoPro Hero 10 Black support currently implemented.

Remove the need for a dedicated through wall power switch using the built in low power 'WiFi' type switch. After plugging in a battery the unit draws only 10-15 mA, which gives you over 2.5 days of battery life on a typical 600 mAh battery. Power up the main processor using the app before leaving the pad over the BlueTooth connection and control powering up the camera remotely from the base station over LoRa to maximize battery life and reduce camera run time. Control recording remotely from the base station over LoRa as well.

The base station relays data sent over LoRa from the flight computer to a nearby iOS device, giving the user a realtime view of telemetry data prior to launch, as well as telemetry and tracking data post launch. Use your phone's speakers to call out altitude, apogee, max speed, and descent rate during the flight and to locate the rocket via an arrow that points towards the rocket and/or a map view of where the rocket landed.

Finally, manage the flight data intuitively by uploading from the flight computer to an iOS device and then sharing that data using email, text, air drop, or any other iOS supported sharing means.

## Architecture

<!-- TODO: Add system architecture photo/diagram -->
![Architecture](docs/images/architecture.jpg)

```
                        ┌─────────────────────────────────┐
                        │        FLIGHT COMPUTER          │
                        │                                 │
 ISM6HG256 (1000 Hz) ──>│  Sensor       EKF        PID   │
 BMP585    (500 Hz) ──>│  Collector ──> (15-state) ──> Mixer ──> 1-4x Servos
 MMC5983MA (200 Hz) ──>│              Roll Control / Guidance PN        │
 u-blox M10 (10-25 Hz) ──>│                                 │
                        └──────┬──────────────────────────┘
                               │ I2S (22 kHz DMA)
                               │ I2C (commands)
                        ┌──────▼──────────────────────────┐
                        │         OUT COMPUTER             │
                        │                                  │
                        │  MRAM Ring ──> NAND Flash (log)  │
                        │  LoRa TX (2 Hz) ──> 915 MHz     │──── BLE ────> iOS App
                        │  BLE GATT Server                 │     (direct)
                        └──────┬───────────────────────────┘
                               │ LoRa 915 MHz
                        ┌──────▼───────────────────────────┐
                        │         BASE STATION              │
                        │                                   │
                        │  LoRa RX ──> SD Card (CSV log)   │──── BLE ────> iOS App
                        │  BLE GATT Server                  │    (relayed)
                        │  MAX17205G Battery Monitor        │
                        └───────────────────────────────────┘
```

## Hardware

### Sensors

| Sensor | Type | Interface | Rate | Range |
|--------|------|-----------|------|-------|
| **ISM6HG256** | 6-axis IMU | SPI @ 10 MHz | 960 Hz | Low-g: +/-16g, High-g: +/-256g, Gyro: +/-4000 dps |
| **BMP585** | Barometer | SPI | 500 Hz | 300-1250 hPa |
| **IIS2MDCTR** | Magnetometer | I2C | 150 Hz | +/-50 Gauss |
| **u-blox M10** | GNSS | UART 115200 | 18 Hz | GPS/GLONASS/Galileo/BeiDou |
| **INA230** | Power monitor | I2C | 10 Hz | Voltage, current, SOC |

### Radios

| Radio | Protocol | Frequency | Data Rate | Purpose |
|-------|----------|-----------|-----------|---------|
| **LLCC68** | LoRa | 915 MHz | 2-10 Hz | Rocket-to-ground telemetry |
| **NimBLE** | BLE 5.0 | 2.4 GHz | ~10 Hz | Ground-to-app telemetry |

### Control

- **1-4 fin-tab servos** configurable PWM control for roll or guide straight up
- **4x pyro channels** with continuity monitoring and configurable triggers
- **RunCam Split 4 and GoPro Hero 10 Black** support via UART/GPIO control

## Repository Structure

```
TinkerRocket/
├── tinkerrocket-idf/           # ESP-IDF firmware
│   ├── projects/
│   │   ├── flight_computer/    # Flight computer firmware
│   │   ├── out_computer/       # Out computer firmware
│   │   └── base_station/       # Base station firmware
│   └── components/             # First-party + vendored ESP-IDF components
│       ├── TR_GpsInsEKF/       # 15-state GPS/INS Extended Kalman Filter
│       ├── TR_PID/             # PID controller (derivative-on-measurement)
│       ├── TR_ControlMixer/    # 4-fin cruciform mixing + gain scheduling
│       ├── TR_GuidancePN/      # Proportional navigation (git submodule)
│       ├── TR_Coordinates/     # ECEF/LLA/ENU coordinate transforms
│       ├── TR_KinematicChecks/ # Launch/apogee/landing event detection
│       ├── TR_Sensor_Data_Converter/ # Raw sensor -> SI unit conversion
│       ├── TR_RocketComputerTypes/ # Shared packed data structures
│       ├── TR_Compat/          # Arduino-API shims backed by ESP-IDF
│       ├── CRC/                # CRC16 for frame integrity
│       └── ...                 # Sensor drivers, I2S, BLE, LoRa, etc.
│
├── tests_cpp/                  # GoogleTest host-side unit tests
│   └── host_shim/              # Arduino.h/compat.h shim for non-IDF builds
│
├── TinkerRocketApp/            # iOS companion app (SwiftUI)
│
├── tinkerrocket-sim/           # 6-DOF flight simulation (Python + C++)
│   ├── src/tinkerrocket_sim/   # Physics, sensors, control, visualization
│   ├── cpp/                    # pybind11 bindings for EKF, PID, mixer, guidance
│   └── tests/                  # Pytest regression suite
│
├── tests_cpp/                  # GoogleTest host-side unit tests
├── tests/integration/          # Binary log replay integration tests
├── preflight/                  # Pre-flight go/no-go checklist
│
├── .github/workflows/          # CI: cpp-tests, sim-tests, ios-tests
└── ARCHIVE/                    # Previous hardware iterations
```

## Building

### Prerequisites

- [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/) for firmware
- Xcode 16+ for iOS app
- Python 3.10+ for simulation
- CMake 3.16+ for host-side tests

### Guidance (optional)

The proportional-navigation guidance law lives in a separate private submodule at [`tinkerrocket-idf/components/TR_GuidancePN`](https://github.com/Tinkerbug-Robotics/TR_GuidancePN). Everything else — roll control, EKF, sensor collection, telemetry, logging, LoRa, BLE, iOS integration, ground-test modes — builds and runs **without** it. Public contributors can clone, build, flash, and contribute to any non-guidance feature with a standard, non-recursive clone.

When the submodule is not initialized:
- Firmware compiles against a header-only no-op stub (`TR_GuidancePN_Stub`). At boot the flight computer logs `PN Guidance: NOT COMPILED IN (stub active)`. All `guidance.*` call sites still exist, but each method returns zero / false.
- `tests_cpp`: the `test_guidance_pn` target is skipped at CMake configure time.
- `tinkerrocket-sim`: the `_guidance` pybind11 extension is skipped at `pip install -e` time; `tests/test_guidance_pn.py` is skipped via `pytest.importorskip`.

Contributors with access can opt in:
```bash
git submodule update --init tinkerrocket-idf/components/TR_GuidancePN
```

### Firmware (ESP-IDF)

```bash
# Flight Computer
cd tinkerrocket-idf/projects/flight_computer
idf.py build
idf.py flash

# Out Computer
cd tinkerrocket-idf/projects/out_computer
idf.py build
idf.py flash

# Base Station
cd tinkerrocket-idf/projects/base_station
idf.py build
idf.py flash
```

### iOS App

Open `TinkerRocketApp/TinkerRocketApp.xcodeproj` in Xcode and build for your device.

### Simulation

```bash
cd tinkerrocket-sim
pip install -e ".[dev]"
python -m tinkerrocket_sim.simulation.closed_loop_sim
```

### Cloning with Submodules

The guidance library is a private submodule. Clone with:

```bash
git clone --recurse-submodules https://github.com/Tinkerbug-Robotics/TinkerRocket.git
```

## Testing

### C++ Unit Tests (GoogleTest)

90 tests covering all safety-critical flight libraries:

```bash
cmake -S tests_cpp -B tests_cpp/build -DCMAKE_BUILD_TYPE=Debug
cmake --build tests_cpp/build -j$(nproc)
ctest --test-dir tests_cpp/build --output-on-failure
```

| Test Suite | Tests | Coverage |
|------------|-------|----------|
| `test_ekf` | 15 | EKF convergence, quaternion norm, bias estimation, NaN safety |
| `test_kinematic_checks` | 11 | Launch/apogee/landing detection, spike rejection |
| `test_pid` | 11 | P/I/D terms, anti-windup, derivative-on-measurement |
| `test_control_mixer` | 10 | Cruciform mixing, gain scheduling, fin clamping |
| `test_guidance_pn` | 9 | PN commands, acceleration clamping, CPA detection |
| `test_coordinates` | 7 | ECEF/LLA roundtrip, ENU transforms, Quat2Euler |
| `test_sensor_data_converter` | 10 | All sensor types: IMU, baro, GNSS, mag, power |
| `test_lora_roundtrip` | 6 | Full OC -> Base Station -> App data pipeline |
| `test_rocket_computer_types` | 6 | Struct sizes, flag bits, frame constants |
| `test_crc16_compat` | 5 | CRC cross-platform compatibility (C++ and Swift) |

### Simulation Tests (pytest)

48 tests validating pybind11 bindings match flight code:

```bash
cd tinkerrocket-sim
python -m pytest tests/ -v
```

### Integration Tests

Replay binary flight logs to validate sensor rates, CRC integrity, and timestamps:

```bash
python -m pytest tests/integration/ -v
```

Place `.bin` bench test files in `tests/test_data/` -- tests auto-skip when no data is present.

### Pre-Flight Checklist

```bash
python preflight/run_preflight.py tests/test_data/bench_static_60s.bin
```

Validates sensor rates, frame integrity, timestamp health, and data completeness with a clear GO/NO-GO result.

### CI/CD

Three GitHub Actions workflows run automatically on push:

- **cpp-tests.yml** -- Triggered by changes to `tinkerrocket-idf/components/`, `tests_cpp/`, or `tests/integration/`
- **firmware-build.yml** -- Full ESP-IDF compilation of `out_computer` and `base_station` (Docker: `espressif/idf:v5.3.2`)
- **sim-tests.yml** -- Triggered by changes to `tinkerrocket-sim/` or the component sources it builds from
- **ios-tests.yml** -- Triggered by changes to `TinkerRocketApp/`

## Communication Protocols

### Binary Frame Format

All inter-board and logged data uses a consistent framing protocol:

```
[0xAA 0x55 0xAA 0x55] [Type] [Length] [Payload] [CRC16_MSB] [CRC16_LSB]
     (4 bytes)         (1)    (1)      (0-68)     (1)         (1)
```

CRC-16 polynomial: 0x8001, initial: 0x0000.

### Message Types

| Type | Name | Size | Rate |
|------|------|------|------|
| 0xA1 | GNSS | 42 B | 18 Hz |
| 0xA2 | ISM6HG256 (IMU) | 22 B | 960 Hz |
| 0xA3 | BMP585 (Baro) | 12 B | 500 Hz |
| 0xA4 | MMC5983MA (Mag) | 16 B | 200 Hz |
| 0xA5 | NonSensor (EKF) | 43 B | 500 Hz |
| 0xA6 | Power | 10 B | 10 Hz |
| 0xF1 | LoRa Telemetry | 59 B | 2 Hz |

### I2S Telemetry Pipeline

The flight computer streams sensor data to the out computer via I2S DMA at 22,050 Hz sample rate (88 KB/s bandwidth). Frames are zero-copy from ISR callbacks into an MRAM ring buffer, then flushed to NAND flash.

## Performance

| Metric | Target | Achieved |
|--------|--------|----------|
| IMU rate | 960 Hz | 960 Hz |
| Barometer rate | 500 Hz | 490 Hz |
| Magnetometer rate | 200 Hz | 220 Hz |
| EKF output rate | 500 Hz | 490 Hz |
| GNSS rate | 18 Hz | 34 Hz |
| I2S frame drops | 0 | 0 |
| Max IMU gap | < 10 ms | 9.4 ms |
| Standby power | -- | 12 mA |
| Active power | -- | 130 mA |

## iOS App

<!-- TODO: Add iOS app screenshot -->
![iOS App](docs/images/ios_app.jpg)

The TinkerRocketApp is a SwiftUI companion app providing:

- Real-time telemetry dashboard (attitude, altitude, velocity, GPS, battery)
- 3D flight trajectory visualization
- Binary flight log download and CSV export
- Servo and pyro channel testing
- Flight configuration upload (PID gains, roll profiles, guidance params)
- Ballistic drift prediction (DriftCast)
- Audio/haptic flight event announcements

Connects via BLE directly to the Out Computer (on-pad) or through the Base Station (in-flight via LoRa relay).

## Multi-Device Support

The system supports connecting to multiple flight computers and base stations simultaneously, with collision avoidance for multiple users operating at the same field.

### Device Identity

Every unit has a persistent identity stored in NVS flash:

| Field | Size | Purpose |
|-------|------|---------|
| `unit_id` | 4 bytes | Hardware fingerprint (last 4 bytes of efuse MAC), immutable |
| `unit_name` | 1-20 chars | User-settable nickname (e.g. "Atlas", "PadAlpha") |
| `network_id` | 1 byte | LoRa network namespace (0-255), isolates different users |
| `rocket_id` | 1 byte | Unique ID per rocket within a network (1-254) |

BLE advertising names use the format `TR-R-<name>` for rockets and `TR-B-<name>` for base stations (e.g. `TR-R-Atlas`, `TR-B-PadAlpha`). The app configures identity on first connect via BLE commands 40/41/42.

### Network Isolation

Multiple users at the same field are isolated via three layers:

1. **Network ID** -- each user picks a network name on first app launch, hashed to a 1-byte ID. Devices only process packets matching their network.
2. **LoRa syncword** -- configurable via the app's LoRa settings. Different syncwords prevent radio-level decoding of other users' packets.
3. **Rocket ID** -- within a network, each rocket has a unique ID (1-254) so the base station can track and route to individual rockets.

### LoRa Frame Format

Telemetry packets (rocket to base station) carry a 2-byte routing header:

```
[network_id:1][rocket_id:1][telemetry_payload:57] = 59 bytes total
```

Uplink commands (base station to rocket) are addressed:

```
[0xCA][network_id:1][target_rocket_id:1][cmd:1][len:1][payload:0-18]
```

Target rocket ID `0xFF` is broadcast (e.g., time sync). The rocket filters incoming uplinks and ignores packets for other networks or rocket IDs.

### Base Station Multi-Rocket Tracker

The base station maintains a tracker array of up to 4 rockets. Each slot stores the last-known telemetry, RSSI/SNR, GPS position, and unit name. Rockets announce their name via a LoRa beacon (`0xBE` sync byte) every 2 seconds while in READY or PRELAUNCH state. The tracker auto-evicts the oldest slot when all 4 are full.

Telemetry relayed to the iOS app via BLE includes `rid` (rocket ID) and `run` (rocket unit name) fields so the app can demultiplex multiple rockets arriving through a single base station.

### iOS App Architecture

The app uses a fleet-based architecture for multi-device management:

```
BLEFleet (owns CBCentralManager, scanning, connection routing)
  ├── devices: [BLEDevice]          -- directly connected via BLE
  │   ├── BLEDevice (rocket)        -- telemetry, config, files, commands
  │   └── BLEDevice (base station)  -- relays telemetry from remote rockets
  │       └── remoteRockets: [RemoteRocket]  -- rockets seen via LoRa relay
  └── activeDeviceID                -- which device the dashboard shows
```

- **BLEFleet** manages the shared `CBCentralManager`, scan/discovery, and connection routing
- **BLEDevice** holds per-peripheral state (telemetry, config, file downloads, RSSI) and implements `CBPeripheralDelegate`
- **RemoteRocket** holds telemetry forwarded by a base station, identified by `(baseStationID, rocketID)`

When multiple devices are connected, the dashboard shows a horizontal chip bar for switching between them. Remote rockets appear as orange chips with an antenna icon.

### First-Launch Onboarding

On first app launch, the user picks a network name (e.g. "My Backyard", "Skyhawks Club"). This is hashed to a `network_id` and pushed to every device on first connect. When connecting to a new device (unknown `unit_id`), a provisioning sheet lets the user name the device and assign a rocket ID.

### Command Relay

Commands can be sent to a rocket either directly over BLE or relayed through a base station using BLE command 50:

```
Command 50: [target_rocket_id:1][inner_command:1][inner_payload:0-18]
```

The base station unpacks this and queues a LoRa uplink addressed to the target rocket. This enables controlling rockets that are out of BLE range but within LoRa range of the base station.

## Simulation

The `tinkerrocket-sim` package provides a full 6-DOF closed-loop flight simulation:

- RK4 physics integration at 10 kHz
- Sensor noise models (IMU bias/noise, baro spikes, GPS dropout)
- Multi-rate scheduling matching flight hardware rates
- pybind11 bindings to the same C++ EKF, PID, mixer, and guidance code used in flight
- Interactive 3D trajectory visualization (Plotly)
- OpenRocket `.ork` file import

```bash
cd tinkerrocket-sim
python scripts/run_closed_loop.py --config config/sim_config.yaml
```

## License

Copyright (c) 2026 Tinkerbug Robotics. All rights reserved.

<!-- TODO: Choose and add license (MIT, Apache 2.0, etc.) -->
