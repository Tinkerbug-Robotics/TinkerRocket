# TinkerRocket

Full featured flight computer with 1 kHz logging, remote control power 'WiFi' switch, four pyro channels, camera control, and control of up to six servos for active roll control, guidance, or other functions. Downlink and GPS tracking via a LoRa radio to ground station and companion iOS app for configuration, monitoring, and voice call outs during flight.

<!-- TODO: Add hero photo of the rocket -->
![TinkerRocket](docs/images/rocket_hero.jpg)

## Overview

The onboard flight computer runs a 16-state Extended Kalman Filter fusing IMU, barometer, magnetometer, and GNSS data at up to 1000 Hz. Optional roll control or, a proportional navigation guidance law commands four fin-tab servos through cascaded PID controllers with velocity-based gain scheduling.

The system comprises four cooperating components:

| Component | Hardware | Role |
|-----------|----------|------|
| **Flight Computer** | ESP32-P4 & ESP32-S3 | Sensor fusion, EKF, guidance, servo control, Data logging, LoRa downlink, BLE telemetry |
| **Base Station** | ESP32-S3 | LoRa receiver, BLE gateway, SD card logging |
| **iOS App** | iPhone/iPad | Real-time dashboard, file management, configuration |

## Architecture

<!-- TODO: Add system architecture photo/diagram -->
![Architecture](docs/images/architecture.jpg)

```
                        ┌─────────────────────────────────┐
                        │        FLIGHT COMPUTER          │
                        │                                 │
 ISM6HG256 (960 Hz) ──>│  Sensor       EKF        PID   │
 BMP585    (500 Hz) ──>│  Collector ──> (16-state) ──> Mixer ──> 4x Servos
 MMC5983MA (200 Hz) ──>│              Guidance PN        │
 u-blox M10 (18 Hz) ──>│                                 │
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
| **MMC5983MA** | Magnetometer | SPI | 200 Hz | +/-8 Gauss |
| **u-blox M10** | GNSS | UART 115200 | 18 Hz | GPS/GLONASS/Galileo/BeiDou |
| **INA230** | Power monitor | I2C | 10 Hz | Voltage, current, SOC |

### Radios

| Radio | Protocol | Frequency | Data Rate | Purpose |
|-------|----------|-----------|-----------|---------|
| **LLCC68** | LoRa | 915 MHz | 2 Hz | Rocket-to-ground telemetry |
| **NimBLE** | BLE 5.0 | 2.4 GHz | ~10 Hz | Ground-to-app telemetry |

### Control

- **4x fin-tab servos** at 333 Hz PWM (1000-2000 us pulse)
- **2x pyro channels** with continuity monitoring and configurable triggers
- **RunCam Split 4** support via UART control

## Repository Structure

```
TinkerRocket/
├── tinkerrocket-idf/           # ESP-IDF firmware
│   ├── projects/
│   │   ├── flight_computer/    # Flight computer firmware
│   │   ├── out_computer/       # Out computer firmware
│   │   └── base_station/       # Base station firmware
│   └── components/             # 27 reusable ESP-IDF components
│
├── libraries/                  # Shared C++ libraries (platform-independent)
│   ├── TR_GpsInsEKF/           # 16-state GPS/INS Extended Kalman Filter
│   ├── TR_PID/                 # PID controller (derivative-on-measurement)
│   ├── TR_ControlMixer/        # 4-fin cruciform mixing + gain scheduling
│   ├── TR_GuidancePN/          # Proportional navigation (private submodule)
│   ├── TR_Coordinates/         # ECEF/LLA/ENU coordinate transforms
│   ├── TR_KinematicChecks/     # Launch/apogee/landing event detection
│   ├── TR_Sensor_Data_Converter/ # Raw sensor -> SI unit conversion
│   ├── TR_RocketComputerTypes/ # Shared packed data structures
│   ├── CRC/                    # CRC16 for frame integrity
│   └── ...                     # Sensor drivers, I2S, BLE, LoRa, etc.
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

- **cpp-tests.yml** -- Triggered by changes to `libraries/` or `tests_cpp/`
- **sim-tests.yml** -- Triggered by changes to `tinkerrocket-sim/` or `libraries/`
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
| 0xF1 | LoRa Telemetry | 57 B | 2 Hz |

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
