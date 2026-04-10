//
//  DashboardView.swift
//  TinkerRocketApp
//
//  Main telemetry dashboard
//

import SwiftUI
import Combine
import CoreBluetooth
import CoreLocation

struct IdentifiableInt: Identifiable {
    let value: Int
    var id: Int { value }
}

/// Which modal sheet is currently presented from the dashboard.
enum DashboardSheet: Identifiable {
    case simulator
    case settings
    case servoTest
    case driftCast

    var id: Int { hashValue }
}

struct DashboardView: View {
    @StateObject private var fleet = BLEFleet()
    @StateObject private var flightAnnouncer = FlightAnnouncer()
    @StateObject private var locationManager = LocationManager()
    @State private var activeSheet: DashboardSheet?
    @State private var showProvisioning = false

    var body: some View {
        NavigationView {
            ScrollView {
                VStack(spacing: 20) {
                    if let device = fleet.activeDevice {
                        ConnectedDashboardView(
                            device: device,
                            fleet: fleet,
                            flightAnnouncer: flightAnnouncer,
                            locationManager: locationManager,
                            activeSheet: $activeSheet
                        )
                    } else {
                        // --- Not connected ---
                        NavigationLink(destination: FlightLogsView()) {
                            HStack {
                                Image(systemName: "archivebox.fill")
                                Text("Saved Flights")
                            }
                            .frame(maxWidth: .infinity)
                            .padding()
                            .background(Color.blue)
                            .foregroundColor(.white)
                            .cornerRadius(10)
                        }

                        Button {
                            activeSheet = .driftCast
                        } label: {
                            HStack {
                                Image(systemName: "wind")
                                Text("Reverse Drift Cast")
                            }
                            .frame(maxWidth: .infinity)
                            .padding()
                            .background(Color.red)
                            .foregroundColor(.white)
                            .cornerRadius(10)
                        }

                        HStack(spacing: 12) {
                            Button {
                                if fleet.isScanning {
                                    fleet.stopScanning()
                                } else {
                                    fleet.startScanning()
                                }
                            } label: {
                                Text(fleet.isScanning ? "Stop" : "Scan")
                                    .fontWeight(.semibold)
                                    .padding(.horizontal, 20)
                                    .padding(.vertical, 12)
                                    .background(fleet.isScanning ? Color.red : Color.green)
                                    .foregroundColor(.white)
                                    .cornerRadius(10)
                            }

                            ConnectionStatusView(
                                isConnected: false,
                                isScanning: fleet.isScanning,
                                statusMessage: fleet.statusMessage,
                                connectedDeviceName: ""
                            )
                        }

                        DevicePickerView(fleet: fleet)
                    }
                }
                .padding()
            }
            .navigationTitle(fleet.isConnected ? "" : "TinkerRocket")
            .toolbar {
                ToolbarItem(placement: .navigationBarLeading) {
                    if fleet.isConnected {
                        Button {
                            fleet.disconnectAll()
                        } label: {
                            Image(systemName: "chevron.backward")
                        }
                    }
                }

                ToolbarItem(placement: .navigationBarTrailing) {
                    if let device = fleet.activeDevice {
                        HStack(spacing: 16) {
                            NavigationLink(destination: FileManagerView(device: device)) {
                                Image("RocketIcon")
                                    .resizable()
                                    .renderingMode(.template)
                                    .aspectRatio(contentMode: .fit)
                                    .frame(width: 22, height: 22)
                            }

                            NavigationLink(destination: MapView(device: device)) {
                                Image(systemName: "map.fill")
                            }

                            Button {
                                flightAnnouncer.isEnabled.toggle()
                            } label: {
                                Image(systemName: flightAnnouncer.isEnabled
                                      ? "speaker.wave.2.fill" : "speaker.slash")
                                    .foregroundColor(flightAnnouncer.isEnabled ? .blue : .gray)
                            }
                            .simultaneousGesture(
                                LongPressGesture().onEnded { _ in
                                    flightAnnouncer.testVoice()
                                }
                            )

                            Button {
                                activeSheet = .settings
                            } label: {
                                Image(systemName: "gearshape")
                            }
                        }
                    } else {
                        Image("Small Tinkerbug Robotics Logo Vertical")
                            .resizable()
                            .aspectRatio(contentMode: .fit)
                            .frame(height: 30)
                    }
                }
            }
        }
        .navigationViewStyle(.stack)
        .onAppear {
            fleet.activeDevice?.flightAnnouncer = flightAnnouncer
        }
        .onChange(of: fleet.isConnected) { connected in
            if connected {
                fleet.activeDevice?.flightAnnouncer = flightAnnouncer
            }
            if !connected && !fleet.isScanning {
                fleet.startScanning()
            }
            if connected, let dev = fleet.activeDevice, dev.isBaseStation {
                locationManager.startUpdates()
            } else if !connected {
                locationManager.stopUpdates()
            }
        }
        .sheet(item: $activeSheet) { sheet in
            if let device = fleet.activeDevice {
                switch sheet {
                case .simulator:
                    SimulationView(device: device)
                case .settings:
                    SettingsView(device: device)
                case .servoTest:
                    ServoTestView(device: device)
                case .driftCast:
                    DriftCastView(device: device)
                }
            }
        }
        .sheet(isPresented: $showProvisioning) {
            if let device = fleet.activeDevice {
                DeviceProvisioningSheet(device: device)
            }
        }
        .sheet(isPresented: Binding(
            get: { fleet.isScanning && fleet.isConnected },
            set: { if !$0 { fleet.stopScanning() } }
        )) {
            NavigationView {
                VStack {
                    DevicePickerView(fleet: fleet)
                    Spacer()
                }
                .padding()
                .navigationTitle("Add Device")
                .navigationBarTitleDisplayMode(.inline)
                .toolbar {
                    ToolbarItem(placement: .cancellationAction) {
                        Button("Done") { fleet.stopScanning() }
                    }
                }
            }
        }
        .onChange(of: fleet.activeDevice?.unitID) { newID in
            // When config_identity readback populates the unitID,
            // show provisioning sheet if this is a new device.
            guard let id = newID, !id.isEmpty,
                  !DeviceProvisioningSheet.isDeviceKnown(id),
                  !showProvisioning else { return }
            showProvisioning = true
        }
    }
}

// MARK: - Connected device dashboard (observes the BLEDevice for live updates)

struct ConnectedDashboardView: View {
    @ObservedObject var device: BLEDevice
    @ObservedObject var fleet: BLEFleet
    @ObservedObject var flightAnnouncer: FlightAnnouncer
    @ObservedObject var locationManager: LocationManager
    @Binding var activeSheet: DashboardSheet?

    var body: some View {
        // Device chip bar (multi-device) or simple status (single device)
        if fleet.devices.count > 1 {
            DeviceChipBar(fleet: fleet, activeDevice: device)
        } else {
            ConnectionStatusView(
                isConnected: true,
                isScanning: fleet.isScanning,
                statusMessage: fleet.statusMessage,
                connectedDeviceName: device.displayName
            )
        }

        if !device.isBaseStation && !device.telemetry.pwr_pin_on {
            // --- Powered OFF (rocket only): show battery + power on button ---
            BatteryView(telemetry: device.telemetry, isBaseStation: false)

            Button {
                device.sendPowerToggle()
            } label: {
                HStack {
                    Image(systemName: "bolt.fill")
                    Text("Power On")
                }
                .font(.system(.title2, design: .default).weight(.semibold))
                .frame(maxWidth: .infinity)
                .padding()
                .background(Color.green)
                .foregroundColor(.white)
                .cornerRadius(10)
            }
        } else {
            // --- Powered ON (or base station): full telemetry dashboard ---
            RocketStateView(state: device.telemetry.state)

            if device.simLaunched {
                SimModeBannerView {
                    device.sendCommand(7)
                    device.clearSimBanner()
                }
            }

            if device.groundTestActive {
                GroundTestBannerView {
                    device.sendCommand(16)
                    device.groundTestActive = false
                }
            }

            if device.isBaseStation {
                FlightSummaryView(telemetry: device.telemetry)
            }

            SignalStrengthView(
                telemetry: device.telemetry,
                bleRSSI: device.connectedRSSI,
                isBaseStation: device.isBaseStation,
                locationManager: device.isBaseStation ? locationManager : nil
            )

            BatteryView(telemetry: device.telemetry,
                        isBaseStation: device.isBaseStation)

            IMUView(telemetry: device.telemetry,
                    isBaseStation: device.isBaseStation)

            if !device.isBaseStation {
                FlightSummaryView(telemetry: device.telemetry)
            }

            GPSView(telemetry: device.telemetry, compact: true)

            StatusFlagsView(telemetry: device.telemetry,
                            isBaseStation: device.isBaseStation)

            if !device.isBaseStation {
                PyroChannelsView(device: device)
            }

            ControlsView(device: device)

            TestingControlsView(device: device, activeSheet: $activeSheet)

            if !device.isBaseStation {
                OnPadCalibrationView(device: device)
            }

            if !device.isBaseStation {
                Button {
                    device.sendPowerToggle()
                } label: {
                    HStack {
                        Image(systemName: "bolt.slash.fill")
                        Text("Power Off")
                    }
                    .font(.system(.body, design: .default).weight(.semibold))
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(Color.red)
                    .foregroundColor(.white)
                    .cornerRadius(10)
                }
            }
        }
    }
}

// MARK: - Component Views

struct ConnectionStatusView: View {
    let isConnected: Bool
    let isScanning: Bool
    let statusMessage: String
    var connectedDeviceName: String = ""

    var body: some View {
        HStack {
            Circle()
                .fill(isConnected ? Color.green : (isScanning ? Color.orange : Color.gray))
                .frame(width: 12, height: 12)
            if isConnected && !connectedDeviceName.isEmpty {
                VStack(alignment: .leading, spacing: 2) {
                    Text(connectedDeviceName)
                        .font(.headline)
                    Text((connectedDeviceName.contains("Base") || connectedDeviceName.contains("BS")) ? "Base Station" : "Rocket")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            } else {
                Text(statusMessage)
                    .font(.headline)
            }
            Spacer()
            if isScanning {
                ProgressView()
                    .scaleEffect(0.8)
            }
        }
        .padding()
        .frame(maxWidth: .infinity)
        .background(isConnected ? Color.green.opacity(0.1) : Color(.systemGray6))
        .cornerRadius(10)
    }
}

/// Horizontal scroll of device "chips" for switching between connected devices.
struct DeviceChipBar: View {
    @ObservedObject var fleet: BLEFleet
    @ObservedObject var activeDevice: BLEDevice

    var body: some View {
        ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: 8) {
                ForEach(fleet.devices, id: \.peripheral?.identifier) { device in
                    let isActive = device.peripheral?.identifier == activeDevice.peripheral?.identifier

                    Button {
                        fleet.activeDeviceID = device.peripheral?.identifier
                    } label: {
                        HStack(spacing: 6) {
                            if device.isBaseStation {
                                Image(systemName: "antenna.radiowaves.left.and.right")
                                    .font(.caption)
                            } else {
                                Image("RocketIcon")
                                    .resizable()
                                    .renderingMode(.template)
                                    .aspectRatio(contentMode: .fit)
                                    .frame(height: 14)
                            }
                            Text(device.displayName)
                                .font(.caption)
                                .fontWeight(.medium)
                                .lineLimit(1)
                        }
                        .padding(.horizontal, 12)
                        .padding(.vertical, 8)
                        .background(isActive ? Color.blue : Color(.systemGray5))
                        .foregroundColor(isActive ? .white : .primary)
                        .cornerRadius(20)
                    }
                    .contextMenu {
                        Button {
                            fleet.disconnect(device)
                        } label: {
                            Label("Disconnect", systemImage: "xmark.circle")
                        }
                    }
                }

                // Remote rockets (seen via base station)
                ForEach(fleet.remoteRockets) { remote in
                    HStack(spacing: 6) {
                        Image("RocketIcon")
                            .resizable()
                            .renderingMode(.template)
                            .aspectRatio(contentMode: .fit)
                            .frame(height: 14)
                        Text(remote.displayName)
                            .font(.caption)
                            .fontWeight(.medium)
                            .lineLimit(1)
                        Image(systemName: "antenna.radiowaves.left.and.right")
                            .font(.system(size: 8))
                    }
                    .padding(.horizontal, 12)
                    .padding(.vertical, 8)
                    .background(Color.orange.opacity(0.2))
                    .foregroundColor(.orange)
                    .cornerRadius(20)
                }

                // Scan button in chip bar
                Button {
                    fleet.startScanning()
                } label: {
                    HStack(spacing: 4) {
                        Image(systemName: "plus")
                            .font(.caption)
                        Text("Add")
                            .font(.caption)
                    }
                    .padding(.horizontal, 12)
                    .padding(.vertical, 8)
                    .background(Color(.systemGray5))
                    .foregroundColor(.blue)
                    .cornerRadius(20)
                }
            }
            .padding(.horizontal, 4)
        }
        .padding(.vertical, 4)
    }
}

struct RocketStateView: View {
    let state: String

    var stateColor: Color {
        switch state {
        case "READY": return .green
        case "PRELAUNCH": return .orange
        case "INFLIGHT": return .red
        case "COMPLETE": return .blue
        default: return .gray
        }
    }

    var body: some View {
        VStack {
            Text(state)
                .font(.system(size: 36, weight: .bold))
                .foregroundColor(stateColor)
            Text("Rocket State")
                .font(.caption)
                .foregroundColor(.secondary)
        }
        .padding()
        .frame(maxWidth: .infinity)
        .background(stateColor.opacity(0.1))
        .cornerRadius(10)
    }
}

struct SimModeBannerView: View {
    var onStop: () -> Void

    var body: some View {
        HStack {
            VStack(spacing: 4) {
                Text("SIM MODE")
                    .font(.system(size: 36, weight: .bold, design: .monospaced))
                    .foregroundColor(.orange)
                Text("Flight simulation active")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            .frame(maxWidth: .infinity)

            Button(action: onStop) {
                VStack(spacing: 4) {
                    Image(systemName: "stop.fill")
                        .font(.title2)
                    Text("Stop")
                        .font(.caption.bold())
                }
                .foregroundColor(.white)
                .frame(width: 70, height: 70)
                .background(Color.red)
                .cornerRadius(10)
            }
        }
        .padding()
        .frame(maxWidth: .infinity)
        .background(Color.orange.opacity(0.1))
        .cornerRadius(10)
    }
}

struct GroundTestBannerView: View {
    var onStop: () -> Void

    var body: some View {
        HStack {
            VStack(spacing: 4) {
                Text("GROUND TEST")
                    .font(.system(size: 36, weight: .bold, design: .monospaced))
                    .foregroundColor(.blue)
                Text("Servos responding to live sensors")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            .frame(maxWidth: .infinity)

            Button(action: onStop) {
                VStack(spacing: 4) {
                    Image(systemName: "stop.fill")
                        .font(.title2)
                    Text("Stop")
                        .font(.caption.bold())
                }
                .foregroundColor(.white)
                .frame(width: 70, height: 70)
                .background(Color.red)
                .cornerRadius(10)
            }
        }
        .padding()
        .frame(maxWidth: .infinity)
        .background(Color.blue.opacity(0.1))
        .cornerRadius(10)
    }
}

struct BatteryView: View {
    let telemetry: TelemetryData
    var isBaseStation: Bool = false

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Battery")
                .font(.headline)

            // Column headers
            HStack(spacing: 0) {
                Text("")
                    .frame(width: 70, alignment: .leading)
                Text("Charge")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                Text("Voltage")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                Text("Current")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
            }

            // Rocket battery row (always shown)
            BatteryRow(label: "Rocket",
                       charge: telemetry.socDisplay,
                       voltage: telemetry.voltageDisplay,
                       current: telemetry.currentDisplay)

            // Base station row (only when connected via base station)
            if isBaseStation {
                BatteryRow(label: "Base Stn",
                           charge: telemetry.bsSocDisplay,
                           voltage: telemetry.bsVoltageDisplay,
                           current: telemetry.bsCurrentDisplay)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

struct BatteryRow: View {
    let label: String
    let charge: String
    let voltage: String
    let current: String

    var body: some View {
        HStack(spacing: 0) {
            Text(label)
                .font(.caption)
                .frame(width: 70, alignment: .leading)
            Text(charge)
                .font(.system(.caption, design: .monospaced))
                .frame(maxWidth: .infinity)
            Text(voltage)
                .font(.system(.caption, design: .monospaced))
                .frame(maxWidth: .infinity)
            Text(current)
                .font(.system(.caption, design: .monospaced))
                .frame(maxWidth: .infinity)
        }
    }
}

struct GPSView: View {
    let telemetry: TelemetryData
    var compact: Bool = false

    var body: some View {
        if compact {
            HStack(spacing: 12) {
                Text("GPS")
                    .font(.headline)
                Text(telemetry.coordinatesDisplay)
                    .font(.system(.caption, design: .monospaced))
                Spacer()
                Text("\(telemetry.num_sats) sats")
                    .font(.system(.caption, design: .monospaced))
                    .foregroundColor(.secondary)
            }
            .padding()
            .frame(maxWidth: .infinity)
            .background(Color(.systemGray6))
            .cornerRadius(10)
        } else {
            VStack(alignment: .leading, spacing: 10) {
                Text("GPS")
                    .font(.headline)

                DataItem(label: "Coordinates", value: telemetry.coordinatesDisplay)

                HStack(spacing: 20) {
                    DataItem(label: "Satellites", value: "\(telemetry.num_sats)")
                    DataItem(label: "GDOP", value: telemetry.gdop.map { String(format: "%.1f", $0) } ?? "N/A")
                }
            }
            .padding()
            .frame(maxWidth: .infinity, alignment: .leading)
            .background(Color(.systemGray6))
            .cornerRadius(10)
        }
    }
}

struct PerformanceView: View {
    let telemetry: TelemetryData

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Performance")
                .font(.headline)

            HStack(spacing: 20) {
                DataItem(label: "Max Altitude", value: telemetry.maxAltDisplay)
                DataItem(label: "Max Speed", value: telemetry.maxSpeedDisplay)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

struct AltitudeView: View {
    let telemetry: TelemetryData

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Altitude")
                .font(.headline)

            HStack(spacing: 20) {
                DataItem(label: "Pressure Alt", value: telemetry.pressureAltDisplay)
                DataItem(label: "Vert Rate", value: telemetry.altitudeRateDisplay)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

struct IMUView: View {
    let telemetry: TelemetryData
    var isBaseStation: Bool = false

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("IMU")
                .font(.headline)

            HStack(spacing: 0) {
                Text("")
                    .frame(width: 60, alignment: .leading)
                Text("X")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                Text("Y")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                Text("Z")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
            }

            IMURow(label: "Low-G", unit: "m/s\u{00B2}",
                   x: telemetry.low_g_x, y: telemetry.low_g_y, z: telemetry.low_g_z,
                   decimals: 2)

            // High-G only available on direct rocket connection (not via LoRa)
            if !isBaseStation {
                IMURow(label: "High-G", unit: "m/s\u{00B2}",
                       x: telemetry.high_g_x, y: telemetry.high_g_y, z: telemetry.high_g_z,
                       decimals: 1)
            }

            IMURow(label: "Gyro", unit: "\u{00B0}/s",
                   x: telemetry.gyro_x, y: telemetry.gyro_y, z: telemetry.gyro_z,
                   decimals: 1)
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

struct IMURow: View {
    let label: String
    let unit: String
    let x: Float?
    let y: Float?
    let z: Float?
    let decimals: Int

    private func fmt(_ val: Float?) -> String {
        guard let v = val else { return "—" }
        return String(format: "%.\(decimals)f", v)
    }

    var body: some View {
        HStack(spacing: 0) {
            VStack(alignment: .leading, spacing: 2) {
                Text(label)
                    .font(.caption)
                Text(unit)
                    .font(.caption2)
                    .foregroundColor(.secondary)
            }
            .frame(width: 60, alignment: .leading)

            Text(fmt(x))
                .font(.system(.caption, design: .monospaced))
                .frame(maxWidth: .infinity)
            Text(fmt(y))
                .font(.system(.caption, design: .monospaced))
                .frame(maxWidth: .infinity)
            Text(fmt(z))
                .font(.system(.caption, design: .monospaced))
                .frame(maxWidth: .infinity)
        }
    }
}

struct DataRatesView: View {
    let telemetry: TelemetryData

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Data Rates")
                .font(.headline)

            HStack(spacing: 20) {
                DataItem(label: "RX", value: telemetry.rx_kbs.map { String(format: "%.1f kB/s", $0) } ?? "N/A")
                DataItem(label: "Write", value: telemetry.wr_kbs.map { String(format: "%.1f kB/s", $0) } ?? "N/A")
            }

            HStack(spacing: 20) {
                DataItem(label: "Frames RX", value: "\(telemetry.frames_rx)")
                DataItem(label: "Frames Dropped", value: "\(telemetry.frames_drop)")
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

struct StatusFlagsView: View {
    let telemetry: TelemetryData
    var isBaseStation: Bool = false

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Status")
                .font(.headline)

            HStack(spacing: 20) {
                StatusBadge(
                    label: "Camera",
                    active: telemetry.camera_recording
                )
                StatusBadge(
                    label: isBaseStation ? "Rocket Log" : "Logging",
                    active: telemetry.logging_active
                )
                if isBaseStation {
                    StatusBadge(
                        label: "Base Stn Log",
                        active: telemetry.bs_logging_active
                    )
                }
            }

            if !telemetry.active_file.isEmpty {
                Text("File: \(telemetry.active_file)")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

struct DevicePickerView: View {
    @ObservedObject var fleet: BLEFleet

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Available Devices")
                .font(.headline)

            if fleet.discoveredDevices.isEmpty && fleet.isScanning {
                HStack {
                    Spacer()
                    VStack(spacing: 8) {
                        ProgressView()
                        Text("Searching...")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                    Spacer()
                }
                .padding(.vertical, 20)
            } else if fleet.discoveredDevices.isEmpty {
                Text("No devices found. Tap Scan to search.")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .padding(.vertical, 10)
            }

            ForEach(fleet.discoveredDevices) { device in
                let alreadyConnected = fleet.devices.contains { $0.peripheral?.identifier == device.id }
                Button(action: {
                    if !alreadyConnected { fleet.connect(to: device) }
                }) {
                    HStack {
                        Group {
                            if device.isBaseStation {
                                Image(systemName: "antenna.radiowaves.left.and.right")
                                    .font(.title2)
                            } else {
                                Image("RocketIcon")
                                    .resizable()
                                    .renderingMode(.template)
                                    .aspectRatio(contentMode: .fit)
                                    .frame(height: 24)
                            }
                        }
                        .foregroundColor(device.isBaseStation ? .orange : .blue)
                        .frame(width: 36)

                        VStack(alignment: .leading, spacing: 2) {
                            Text(device.name)
                                .font(.body)
                                .fontWeight(.medium)
                            Text(device.typeLabel)
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }

                        Spacer()

                        // BLE signal strength or connected badge
                        if alreadyConnected {
                            Image(systemName: "checkmark.circle.fill")
                                .foregroundColor(.green)
                        } else {
                            BLESignalIndicator(rssi: device.rssi)
                        }
                    }
                    .padding(.vertical, 8)
                    .padding(.horizontal, 12)
                    .background(alreadyConnected ? Color.green.opacity(0.1) : Color(.systemGray5))
                    .cornerRadius(10)
                }
                .buttonStyle(PlainButtonStyle())
                .disabled(alreadyConnected)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

struct BLESignalIndicator: View {
    let rssi: Int

    // RSSI ranges: > -50 excellent, > -70 good, > -85 fair, < -85 weak
    var bars: Int {
        if rssi > -50 { return 4 }
        if rssi > -65 { return 3 }
        if rssi > -80 { return 2 }
        return 1
    }

    var body: some View {
        HStack(spacing: 2) {
            ForEach(0..<4) { i in
                RoundedRectangle(cornerRadius: 1)
                    .fill(i < bars ? Color.green : Color(.systemGray4))
                    .frame(width: 4, height: CGFloat(6 + i * 4))
            }
        }
    }
}

struct FlightSummaryView: View {
    let telemetry: TelemetryData

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            // Column headers
            HStack(spacing: 0) {
                Text("")
                    .frame(width: 70, alignment: .leading)
                Text("Current")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                Text("Max")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
            }

            HStack(spacing: 0) {
                Text("Altitude")
                    .font(.caption)
                    .frame(width: 70, alignment: .leading)
                Text(telemetry.pressureAltDisplay)
                    .font(.system(.caption, design: .monospaced))
                    .frame(maxWidth: .infinity)
                Text(telemetry.maxAltDisplay)
                    .font(.system(.caption, design: .monospaced))
                    .frame(maxWidth: .infinity)
            }

            HStack(spacing: 0) {
                Text("Speed")
                    .font(.caption)
                    .frame(width: 70, alignment: .leading)
                Text(telemetry.altitudeRateDisplay)
                    .font(.system(.caption, design: .monospaced))
                    .frame(maxWidth: .infinity)
                Text(telemetry.maxSpeedDisplay)
                    .font(.system(.caption, design: .monospaced))
                    .frame(maxWidth: .infinity)
            }

            Divider()

            // Attitude header row
            HStack(spacing: 0) {
                Text("")
                    .frame(width: 70, alignment: .leading)
                Text("Roll")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                Text("Pitch")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                Text("Yaw")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
            }

            // Attitude values row
            HStack(spacing: 0) {
                Text("Attitude")
                    .font(.caption)
                    .frame(width: 70, alignment: .leading)
                Text(telemetry.rollDisplay)
                    .font(.system(.caption, design: .monospaced))
                    .frame(maxWidth: .infinity)
                Text(telemetry.pitchDisplay)
                    .font(.system(.caption, design: .monospaced))
                    .frame(maxWidth: .infinity)
                Text(telemetry.yawDisplay)
                    .font(.system(.caption, design: .monospaced))
                    .frame(maxWidth: .infinity)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

struct LoRaSignalView: View {
    let telemetry: TelemetryData

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("LoRa Signal")
                .font(.headline)

            HStack(spacing: 20) {
                DataItem(label: "RSSI", value: telemetry.rssiDisplay)
                DataItem(label: "SNR", value: telemetry.snrDisplay)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

struct DataItem: View {
    let label: String
    let value: String

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(label)
                .font(.caption)
                .foregroundColor(.secondary)
            Text(value)
                .font(.system(.body, design: .monospaced))
        }
    }
}

struct StatusBadge: View {
    let label: String
    let active: Bool

    var body: some View {
        HStack {
            Circle()
                .fill(active ? Color.green : Color.gray)
                .frame(width: 8, height: 8)
            Text(label)
                .font(.caption)
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 6)
        .background(active ? Color.green.opacity(0.2) : Color(.systemGray5))
        .cornerRadius(8)
    }
}

struct RocketDirectionView: View {
    let telemetry: TelemetryData
    @ObservedObject var locationManager: LocationManager

    var body: some View {
        if let rocketLat = telemetry.latitude,
           let rocketLon = telemetry.longitude,
           !rocketLat.isNaN && !rocketLon.isNaN,
           let phoneLoc = locationManager.userLocation {

            let dist = LocationManager.haversineDistance(
                lat1: phoneLoc.latitude, lon1: phoneLoc.longitude,
                lat2: rocketLat, lon2: rocketLon
            )
            let bear = LocationManager.bearing(
                lat1: phoneLoc.latitude, lon1: phoneLoc.longitude,
                lat2: rocketLat, lon2: rocketLon
            )
            // Arrow rotation: bearing relative to phone's compass heading
            let arrowAngle = bear - locationManager.heading

            VStack(spacing: 8) {
                Text("Rocket Direction")
                    .font(.headline)

                Image(systemName: "location.north.fill")
                    .font(.system(size: 64))
                    .foregroundColor(.blue)
                    .rotationEffect(.degrees(arrowAngle))
                    .animation(.easeOut(duration: 0.3), value: arrowAngle)

                Text(formatDistance(dist))
                    .font(.title2)
                    .fontWeight(.semibold)
            }
            .padding()
            .frame(maxWidth: .infinity)
            .background(Color(.systemGray6))
            .cornerRadius(10)
        }
    }

    private func formatDistance(_ meters: Double) -> String {
        if meters >= 1000 {
            return String(format: "%.1f km", meters / 1000.0)
        } else {
            return String(format: "%.0f m", meters)
        }
    }
}

// MARK: - Pyro Channels

struct PyroChannelsView: View {
    @ObservedObject var device: BLEDevice
    @State private var showPyroSheet = false
    @State private var editingChannel: Int = 1
    @State private var contTestChannel: Int = 0   // 0 = none, 1 = CH1, 2 = CH2
    @State private var pyroTestChannel: Int?       // nil = hidden, 1 or 2 = show test view

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Pyro Channels")
                .font(.headline)

            HStack(spacing: 10) {
                pyroTile(
                    channel: 1,
                    enabled: device.rocketConfig?.pyro1Enabled ?? false,
                    armed: device.telemetry.pyro1_armed,
                    continuity: device.telemetry.pyro1_cont,
                    fired: device.telemetry.pyro1_fired,
                    mode: device.rocketConfig?.pyro1TriggerMode ?? 0,
                    value: device.rocketConfig?.pyro1TriggerValue ?? 0
                )

                pyroTile(
                    channel: 2,
                    enabled: device.rocketConfig?.pyro2Enabled ?? false,
                    armed: device.telemetry.pyro2_armed,
                    continuity: device.telemetry.pyro2_cont,
                    fired: device.telemetry.pyro2_fired,
                    mode: device.rocketConfig?.pyro2TriggerMode ?? 0,
                    value: device.rocketConfig?.pyro2TriggerValue ?? 0
                )
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
        .sheet(isPresented: $showPyroSheet) {
            PyroConfigSheet(device: device, channel: editingChannel)
        }
        .fullScreenCover(item: Binding<IdentifiableInt?>(
            get: { pyroTestChannel.map { IdentifiableInt(value: $0) } },
            set: { pyroTestChannel = $0?.value }
        )) { item in
            PyroTestView(device: device, channel: item.value)
        }
    }

    func pyroTile(channel: Int, enabled: Bool, armed: Bool,
                  continuity: Bool, fired: Bool, mode: UInt8, value: Float) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            // Tap to configure
            Button {
                editingChannel = channel
                showPyroSheet = true
            } label: {
                VStack(alignment: .leading, spacing: 6) {
                    HStack {
                        Text("CH \(channel)")
                            .font(.subheadline.weight(.bold))
                            .foregroundColor(.primary)
                        Spacer()
                        if fired {
                            Text("FIRED")
                                .font(.caption2.weight(.bold))
                                .foregroundColor(.white)
                                .padding(.horizontal, 6)
                                .padding(.vertical, 2)
                                .background(Color.orange)
                                .cornerRadius(4)
                        } else if armed || contTestChannel == channel {
                            HStack(spacing: 4) {
                                Circle()
                                    .fill(continuity ? Color.green : Color.red)
                                    .frame(width: 8, height: 8)
                                Text(continuity ? "CONT" : "NO CONT")
                                    .font(.caption2.weight(.bold))
                                    .foregroundColor(continuity ? .green : .red)
                            }
                        }
                    }
                    Text(enabled ? triggerDescription(mode: mode, value: value) : "Disabled")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            .buttonStyle(.plain)

            // Test continuity button (only when not in flight)
            if !armed && !fired && device.telemetry.state != "INFLIGHT" {
                Button {
                    device.sendPyroContTest(channel: UInt8(channel))
                    contTestChannel = channel
                    // Auto-hide result after 5 seconds
                    DispatchQueue.main.asyncAfter(deadline: .now() + 5) {
                        if contTestChannel == channel { contTestChannel = 0 }
                    }
                } label: {
                    Text("Test Continuity")
                        .font(.caption2)
                        .foregroundColor(.blue)
                }
                .buttonStyle(.plain)

                Button {
                    pyroTestChannel = channel
                } label: {
                    Text("Test Pyro Channel")
                        .font(.caption2)
                        .foregroundColor(.orange)
                }
                .buttonStyle(.plain)
            }
        }
        .padding(10)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray5))
        .cornerRadius(8)
    }

    func triggerDescription(mode: UInt8, value: Float) -> String {
        if mode == 0 {
            return String(format: "%.1fs after apogee", value)
        } else {
            return String(format: "%.0fm on descent", value)
        }
    }
}

struct PyroConfigSheet: View {
    @ObservedObject var device: BLEDevice
    let channel: Int
    @Environment(\.dismiss) var dismiss

    @State private var enabled: Bool = false
    @State private var triggerMode: Int = 0  // 0=time, 1=altitude
    @State private var triggerValue: String = ""

    var body: some View {
        NavigationView {
            Form {
                Toggle("Enabled", isOn: $enabled)

                Picker("Trigger", selection: $triggerMode) {
                    Text("Time after apogee").tag(0)
                    Text("Altitude on descent").tag(1)
                }

                HStack {
                    Text(triggerMode == 0 ? "Delay (s)" : "Altitude (m)")
                    Spacer()
                    TextField("Value", text: $triggerValue)
                        .keyboardType(.decimalPad)
                        .multilineTextAlignment(.trailing)
                        .frame(width: 100)
                }
            }
            .navigationTitle("Pyro Channel \(channel)")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel") { dismiss() }
                }
                ToolbarItem(placement: .confirmationAction) {
                    Button("Save") { saveAndSend(); dismiss() }
                }
            }
            .onAppear { loadCurrent() }
        }
    }

    func loadCurrent() {
        guard let cfg = device.rocketConfig else { return }
        if channel == 1 {
            enabled = cfg.pyro1Enabled
            triggerMode = Int(cfg.pyro1TriggerMode)
            triggerValue = String(format: triggerMode == 0 ? "%.1f" : "%.0f", cfg.pyro1TriggerValue)
        } else {
            enabled = cfg.pyro2Enabled
            triggerMode = Int(cfg.pyro2TriggerMode)
            triggerValue = String(format: triggerMode == 0 ? "%.1f" : "%.0f", cfg.pyro2TriggerValue)
        }
    }

    func saveAndSend() {
        let val = Float(triggerValue) ?? 0
        guard var cfg = device.rocketConfig else { return }
        if channel == 1 {
            cfg.pyro1Enabled = enabled
            cfg.pyro1TriggerMode = UInt8(triggerMode)
            cfg.pyro1TriggerValue = val
        } else {
            cfg.pyro2Enabled = enabled
            cfg.pyro2TriggerMode = UInt8(triggerMode)
            cfg.pyro2TriggerValue = val
        }
        device.rocketConfig = cfg
        device.sendPyroConfig(
            ch1Enabled: cfg.pyro1Enabled, ch1Mode: cfg.pyro1TriggerMode, ch1Value: cfg.pyro1TriggerValue,
            ch2Enabled: cfg.pyro2Enabled, ch2Mode: cfg.pyro2TriggerMode, ch2Value: cfg.pyro2TriggerValue
        )
    }
}

// MARK: - Controls

struct ControlsView: View {
    @ObservedObject var device: BLEDevice

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Controls")
                .font(.headline)

            Button(action: {
                device.sendCommand(1)
            }) {
                HStack {
                    Image(systemName: device.telemetry.camera_recording ? "video.fill" : "video")
                    Text(device.telemetry.camera_recording ? "Stop Camera" : "Start Camera")
                }
                .frame(maxWidth: .infinity)
                .padding()
                .background(device.telemetry.camera_recording ? Color.red : Color.blue)
                .foregroundColor(.white)
                .cornerRadius(10)
            }

            Button(action: {
                device.sendCommand(23)
            }) {
                HStack {
                    Image(systemName: device.telemetry.logging_active ? "stop.circle.fill" : "record.circle")
                    Text(device.telemetry.logging_active ? "Stop Logging" : "Start Logging")
                }
                .frame(maxWidth: .infinity)
                .padding()
                .background(device.telemetry.logging_active ? Color.red : Color.orange)
                .foregroundColor(.white)
                .cornerRadius(10)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

struct TestingControlsView: View {
    @ObservedObject var device: BLEDevice
    @Binding var activeSheet: DashboardSheet?
    @State private var showSimWarning = false

    // Sim config (shared with SimulationView via @AppStorage)
    @AppStorage("simMassGrams") private var massGrams: String = "500"
    @AppStorage("simThrustNewtons") private var thrustNewtons: String = "40"
    @AppStorage("simBurnTimeSeconds") private var burnTimeSeconds: String = "1.5"
    @AppStorage("simDescentRateMps") private var descentRateMps: String = "5.0"

    private var isOnPadState: Bool {
        let s = device.telemetry.state
        return s == "READY" || s == "PRELAUNCH"
    }

    private var canLaunchSim: Bool {
        guard let m = Float(massGrams), m > 0,
              let t = Float(thrustNewtons), t > 0,
              let b = Float(burnTimeSeconds), b > 0,
              let d = Float(descentRateMps), d > 0 else { return false }
        return isOnPadState
            && !device.simLaunched
            && !device.groundTestActive
    }

    private var canStartGroundTest: Bool {
        return isOnPadState
            && !device.simLaunched
            && !device.groundTestActive
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Testing")
                .font(.headline)

            VStack(spacing: 10) {
                // Flight Simulation button + settings gear
                HStack(spacing: 0) {
                    Button { showSimWarning = true } label: {
                        HStack {
                            Image(systemName: "airplane")
                            Text("Simulate")
                        }
                        .font(.system(.body, weight: .semibold))
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                    }
                    .disabled(!canLaunchSim)

                    Divider()
                        .frame(height: 30)
                        .background(Color.white.opacity(0.3))

                    Button {
                        activeSheet = .simulator
                    } label: {
                        Image(systemName: "gearshape")
                            .font(.body)
                            .padding(.horizontal, 14)
                            .padding(.vertical, 14)
                    }
                }
                .foregroundColor(.white)
                .background(canLaunchSim ? Color.orange : Color.orange.opacity(0.4))
                .cornerRadius(10)

                // Ground Test button
                Button(action: toggleGroundTest) {
                    HStack {
                        Image(systemName: device.groundTestActive ? "stop.fill" : "gyroscope")
                        Text(device.groundTestActive ? "Stop Test" : "Ground Test")
                    }
                    .font(.system(.body, weight: .semibold))
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 14)
                    .foregroundColor(.white)
                    .background(device.groundTestActive ? Color.red : (canStartGroundTest ? Color.blue : Color.blue.opacity(0.4)))
                    .cornerRadius(10)
                }
                .disabled(!canStartGroundTest && !device.groundTestActive)

                // GPS status hint when ground test is active but no attitude data
                if device.groundTestActive && device.telemetry.num_sats < 4 {
                    HStack(spacing: 6) {
                        ProgressView()
                            .scaleEffect(0.8)
                        Text("Waiting for GPS fix (\(device.telemetry.num_sats) sats) — ground test needs attitude from EKF")
                            .font(.caption)
                            .foregroundColor(.orange)
                    }
                    .padding(.horizontal, 8)
                }

                // Servo Test button
                Button {
                    activeSheet = .servoTest
                } label: {
                    HStack {
                        Image(systemName: "slider.horizontal.3")
                        Text("Servo Test")
                    }
                    .font(.system(.body, weight: .semibold))
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 14)
                    .foregroundColor(.white)
                    .background(canStartGroundTest ? Color.teal : Color.teal.opacity(0.4))
                    .cornerRadius(10)
                }
                .disabled(!canStartGroundTest)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
        .alert("Warning", isPresented: $showSimWarning) {
            Button("Continue", role: .destructive) {
                launchSim()
            }
            Button("Cancel", role: .cancel) { }
        } message: {
            Text("Any attached pyro charges will fire in the sim mode flight. Continue?")
        }
    }

    private func launchSim() {
        guard let massG = Float(massGrams),
              let thrustN = Float(thrustNewtons),
              let burnS = Float(burnTimeSeconds),
              let descentR = Float(descentRateMps) else { return }

        var payload = Data()
        var m = massG, t = thrustN, b = burnS, d = descentR
        payload.append(Data(bytes: &m, count: 4))
        payload.append(Data(bytes: &t, count: 4))
        payload.append(Data(bytes: &b, count: 4))
        payload.append(Data(bytes: &d, count: 4))

        device.sendTimeSync()  // Fresh phone time for unique sim filenames
        device.sendRawCommand(5, payload: payload)
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) {
            device.markSimLaunched()
            device.sendCommand(6)
        }
    }

    private func toggleGroundTest() {
        if device.groundTestActive {
            device.sendCommand(16)
            device.groundTestActive = false
        } else {
            device.sendCommand(15)
            device.groundTestActive = true
        }
    }
}

struct OnPadCalibrationView: View {
    @ObservedObject var device: BLEDevice
    @State private var calibrating = false
    @State private var showGravityWarning = false
    @State private var gravityMag: Float = 0.0
    @State private var gravityError: Float = 0.0

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("On Pad Calibration")
                .font(.headline)

            Button(action: {
                calibrating = true
                device.sendCommand(21)
                // Calibration takes ~10 seconds (1000 samples x 10ms)
                DispatchQueue.main.asyncAfter(deadline: .now() + 12.0) {
                    calibrating = false

                    // Check gravity magnitude from low-g accelerometer
                    let lx = device.telemetry.low_g_x ?? 0
                    let ly = device.telemetry.low_g_y ?? 0
                    let lz = device.telemetry.low_g_z ?? 0
                    let mag = sqrtf(lx * lx + ly * ly + lz * lz)
                    let expected: Float = 9.80665
                    let errorPct = fabsf(mag - expected) / expected * 100.0

                    // Only alert if we actually have accel data (mag > 0)
                    if mag > 0.1 && errorPct > 3.0 {
                        gravityMag = mag
                        gravityError = errorPct
                        showGravityWarning = true
                    }
                }
            }) {
                HStack {
                    if calibrating {
                        ProgressView()
                            .tint(.white)
                    }
                    Image(systemName: "gyroscope")
                    Text(calibrating ? "Calibrating..." : "Calibrate Sensors")
                }
                .frame(maxWidth: .infinity)
                .padding()
                .background(calibrating ? Color.orange : Color.blue)
                .foregroundColor(.white)
                .cornerRadius(10)
            }
            .disabled(calibrating || !device.isConnected)

            Text("Place rocket on pad and keep still. Calibrates gyro bias and accelerometer offsets (~10 seconds).")
                .font(.caption)
                .foregroundColor(.secondary)
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
        .alert("Accelerometer Warning", isPresented: $showGravityWarning) {
            Button("OK", role: .cancel) { }
        } message: {
            Text("Low-G accelerometer magnitude (\(String(format: "%.2f", gravityMag)) m/s²) differs from expected gravity (9.81 m/s²) by \(String(format: "%.1f", gravityError))%. Consider running a bench calibration before flight.")
        }
    }
}

// MARK: - Branded Toolbar

struct BrandedToolbar: ViewModifier {
    func body(content: Content) -> some View {
        content
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Image("Small Tinkerbug Robotics Logo Vertical")
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .frame(height: 30)
                }
            }
    }
}

extension View {
    func brandedToolbar() -> some View {
        modifier(BrandedToolbar())
    }
}

// MARK: - Signal Strength Tile

struct SignalStrengthView: View {
    let telemetry: TelemetryData
    let bleRSSI: Int?
    let isBaseStation: Bool
    var locationManager: LocationManager? = nil

    var body: some View {
        VStack(spacing: 10) {
            Text("Signal")
                .font(.headline)

            HStack(spacing: 30) {
                // Direction arrow (base station only)
                if isBaseStation, let locMgr = locationManager,
                   let rocketLat = telemetry.latitude,
                   let rocketLon = telemetry.longitude,
                   !rocketLat.isNaN && !rocketLon.isNaN,
                   let phoneLoc = locMgr.userLocation {

                    let dist = LocationManager.haversineDistance(
                        lat1: phoneLoc.latitude, lon1: phoneLoc.longitude,
                        lat2: rocketLat, lon2: rocketLon
                    )
                    let bear = LocationManager.bearing(
                        lat1: phoneLoc.latitude, lon1: phoneLoc.longitude,
                        lat2: rocketLat, lon2: rocketLon
                    )
                    let arrowAngle = bear - locMgr.heading

                    VStack(spacing: 6) {
                        Image(systemName: "location.north.fill")
                            .font(.system(size: 48))
                            .foregroundColor(.blue)
                            .rotationEffect(.degrees(arrowAngle))
                            .animation(.easeOut(duration: 0.3), value: arrowAngle)

                        if let phoneAlt = locMgr.userAltitude,
                           let rocketAlt = telemetry.gnss_alt ?? telemetry.pressure_alt {
                            let altDiff = Double(rocketAlt) - phoneAlt
                            Text("\(formatDistance(abs(altDiff))) \(altDiff >= 0 ? "Up" : "Down")")
                                .font(.system(.caption, design: .monospaced))
                                .foregroundColor(.primary)
                        }

                        Text("\(formatDistance(dist)) Away")
                            .font(.system(.caption, design: .monospaced))
                            .foregroundColor(.primary)

                        Text("Rocket")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                }

                // LoRa RSSI (base station only — continuous RX from rocket)
                if isBaseStation {
                    ThermometerIndicator(
                        fillFraction: loraFill,
                        fillColor: loraColor,
                        label: "LoRa",
                        valueText: loraText
                    )
                }

                // GNSS Satellites
                ThermometerIndicator(
                    fillFraction: gnssFill,
                    fillColor: gnssColor,
                    label: "GNSS",
                    valueText: gnssText
                )

                // BLE RSSI
                ThermometerIndicator(
                    fillFraction: bleFill,
                    fillColor: bleColor,
                    label: "BLE",
                    valueText: bleText
                )
            }
        }
        .padding()
        .frame(maxWidth: .infinity)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }

    private func formatDistance(_ meters: Double) -> String {
        if meters >= 1000 {
            return String(format: "%.1f km", meters / 1000.0)
        } else {
            return String(format: "%.0f m", meters)
        }
    }

    // MARK: - LoRa RSSI mapping (-130 to -30 dBm)

    private var loraText: String {
        guard let rssi = telemetry.rssi else { return "--" }
        return String(format: "%.0f", rssi)
    }

    private var loraFill: Double {
        guard let rssi = telemetry.rssi else { return 0 }
        return max(0, min(1, (Double(rssi) + 130) / 100.0))
    }

    private var loraColor: Color {
        guard let rssi = telemetry.rssi else { return .gray }
        if rssi > -70 { return .green }
        if rssi > -90 { return .yellow }
        if rssi > -110 { return .orange }
        return .red
    }

    // MARK: - GNSS Satellites (0 to 30)

    private var gnssText: String {
        return "\(telemetry.num_sats)"
    }

    private var gnssFill: Double {
        let sats = Double(telemetry.num_sats)
        return max(0, min(1, sats / 30.0))
    }

    private var gnssColor: Color {
        let sats = telemetry.num_sats
        if sats > 15 { return .blue }
        if sats >= 11 { return .green }
        if sats >= 6 { return .yellow }
        return .red
    }

    // MARK: - BLE RSSI mapping (-100 to -30 dBm)

    private var bleText: String {
        guard let rssi = bleRSSI else { return "--" }
        return "\(rssi)"
    }

    private var bleFill: Double {
        guard let rssi = bleRSSI else { return 0 }
        return max(0, min(1, (Double(rssi) + 100) / 70.0))
    }

    private var bleColor: Color {
        guard let rssi = bleRSSI else { return .gray }
        if rssi > -50 { return .green }
        if rssi > -65 { return .yellow }
        if rssi > -80 { return .orange }
        return .red
    }
}

struct ThermometerIndicator: View {
    let fillFraction: Double
    let fillColor: Color
    let label: String
    let valueText: String

    private let barWidth: CGFloat = 22
    private let barHeight: CGFloat = 100

    var body: some View {
        VStack(spacing: 6) {
            // Thermometer bar
            ZStack(alignment: .bottom) {
                // Outer track
                RoundedRectangle(cornerRadius: barWidth / 2)
                    .stroke(Color(.systemGray4), lineWidth: 1.5)
                    .frame(width: barWidth, height: barHeight)

                // Fill (from bottom)
                RoundedRectangle(cornerRadius: barWidth / 2)
                    .fill(fillColor)
                    .frame(
                        width: barWidth - 4,
                        height: max(0, (barHeight - 4) * CGFloat(fillFraction))
                    )
                    .padding(.bottom, 2)
            }
            .frame(width: barWidth, height: barHeight)

            // Value
            Text(valueText)
                .font(.system(.caption, design: .monospaced))
                .foregroundColor(.primary)

            // Label
            Text(label)
                .font(.caption2)
                .foregroundColor(.secondary)
        }
    }
}

// MARK: - Preview

struct DashboardView_Previews: PreviewProvider {
    static var previews: some View {
        DashboardView()
    }
}
