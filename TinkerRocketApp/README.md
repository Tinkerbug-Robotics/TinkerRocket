# TinkerRocket iOS App

BLE-based iPhone app for monitoring TinkerRocket flight computer telemetry in real-time.

## Features

- **Real-time telemetry display** via Bluetooth LE
- **Battery monitoring**: SoC %, voltage, current
- **GPS data**: Coordinates, satellite count, GDOP
- **Performance metrics**: Max altitude, max speed
- **Rocket state**: INITIALIZATION → READY → PRELAUNCH → INFLIGHT → COMPLETE
- **Data rates**: I2C RX rate, flash write rate, frame stats
- **Status indicators**: Camera recording, logging active

## Project Setup

### 1. Create Xcode Project

1. Open Xcode
2. **File → New → Project**
3. Select **iOS → App**
4. **Product Name**: `TinkerRocketApp`
5. **Interface**: SwiftUI
6. **Language**: Swift
7. **Organization Identifier**: `com.yourname` (or your identifier)
8. Save to: `/Users/christianpedersen/Documents/Hobbies/Model Rockets/Code/TinkerRocket_iOS_App/`

### 2. Add Source Files

Delete the default `ContentView.swift`, then add the project files:

**In Xcode:**
1. Right-click `TinkerRocketApp` folder → **New Group** → Name it `Models`
2. Right-click `TinkerRocketApp` folder → **New Group** → Name it `Views`
3. Drag the following files into Xcode:
   - `Models/TelemetryData.swift` → into **Models** group
   - `Models/BLEManager.swift` → into **Models** group
   - `Views/DashboardView.swift` → into **Views** group
4. Replace `TinkerRocketApp.swift` with the provided version

**File structure should look like:**
```
TinkerRocketApp/
├── TinkerRocketApp.swift        (App entry point)
├── Models/
│   ├── TelemetryData.swift      (Data model)
│   └── BLEManager.swift         (BLE communication)
└── Views/
    └── DashboardView.swift      (Main UI)
```

### 3. Configure Info.plist

**CRITICAL:** You must add Bluetooth permission to Info.plist or the app will crash!

1. In Xcode, open `Info.plist` (or select your target → Info tab)
2. **Add new entry** (click the + button):
   - **Key**: `Privacy - Bluetooth Always Usage Description`
   - **Value**: `TinkerRocket app uses Bluetooth to communicate with the flight computer`

**Alternative (if using raw Info.plist):**
```xml
<key>NSBluetoothAlwaysUsageDescription</key>
<string>TinkerRocket app uses Bluetooth to communicate with the flight computer</string>
```

### 4. Set Deployment Target

1. Select your project in Xcode
2. **General** tab → **Deployment Info**
3. Set **Minimum Deployments** to **iOS 15.0** or higher

### 5. Build & Run

1. Connect your iPhone via USB (Simulator won't work - BLE requires physical device)
2. Select your iPhone as the target device
3. Click **Run** (⌘R)
4. If prompted, **trust the developer certificate** on your iPhone

## Usage

### First Launch

1. App will request Bluetooth permissions → **Allow**
2. Tap **Scan** button in top-right corner
3. App will search for "TinkerRocket" BLE device
4. Once found, it will auto-connect

### During Flight

- **Real-time updates** push automatically (no polling needed)
- **Rocket state** changes color based on flight phase:
  - Gray: INITIALIZATION
  - Green: READY
  - Orange: PRELAUNCH
  - Red: INFLIGHT
  - Blue: COMPLETE

### Disconnection

- If connection drops, app shows "Disconnected"
- Tap **Scan** to reconnect
- ESP32 auto-restarts advertising when disconnected

## Troubleshooting

### "Bluetooth not ready"
- Check iPhone Bluetooth is enabled (Settings → Bluetooth)
- Restart app

### "TinkerRocket not found"
- Ensure ESP32 is powered on
- Check ESP32 Serial output for `[BLE] Server started and advertising`
- Try scanning again
- Move iPhone closer to rocket

### App crashes on launch
- Check Info.plist has `NSBluetoothAlwaysUsageDescription` key
- Ensure deployment target is iOS 15.0+

### JSON parsing errors in console
- Check ESP32 JSON format matches TelemetryData structure
- Use Serial monitor to verify JSON output

## ESP32 Side

### Testing BLE

Upload firmware to ESP32-S3 (OutComputer) and check Serial output:

```
[BLE] Initializing BLE server...
[BLE] Server started and advertising
[BLE] Device name: TinkerRocket
OutComputer ready.
```

When iPhone connects:
```
[BLE] Device connected
```

### Telemetry Rate

- BLE sends telemetry at the same rate as web GUI updates (typically 5-10 Hz)
- No manual triggering needed - updates are automatic via BLE notifications

## Next Steps (Future Features)

- [ ] Add camera control button (send command 1)
- [ ] Add state toggle button (send command 2)
- [ ] File browser and download
- [ ] Parameter editing
- [ ] Background mode support (updates while app is backgrounded)
- [ ] Flight data recording and playback

## Technical Details

### BLE Service UUIDs (must match ESP32)

- **Service**: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
- **Telemetry Characteristic**: `beb5483e-36e1-4688-b7f5-ea07361b26a8` (NOTIFY + READ)
- **Command Characteristic**: `cba1d466-344c-4be3-ab3f-189f80dd7518` (WRITE)

### Data Format

JSON over BLE notifications, ~300 bytes per update:
```json
{
  "soc": 95.0,
  "voltage": 4.15,
  "current": 120.5,
  "latitude": 39.9717,
  "longitude": -74.9363,
  "state": "PRELAUNCH",
  ...
}
```

## License

Same as TinkerRocket project.
