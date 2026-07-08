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
    @EnvironmentObject private var profileStore: RocketProfileStore
    @EnvironmentObject private var syncer: ActiveRocketSyncer
    @AppStorage("unitSystem") private var unitSystem: UnitSystem = .metric
    @State private var activeSheet: DashboardSheet?
    @State private var showProvisioning = false

    /// Toolbar speaker icon — color reflects audio session readiness so the
    /// operator can verify voice is alive at a glance before launch:
    ///   gray   = voice off
    ///   green  = voice on, audio session active (READY)
    ///   orange = voice on, session not active (probably interrupted)
    ///   red    = last session call returned an error
    private var voiceIconName: String {
        flightAnnouncer.isEnabled ? "speaker.wave.2.fill" : "speaker.slash"
    }
    private var voiceIconColor: Color {
        guard flightAnnouncer.isEnabled else { return .gray }
        if flightAnnouncer.lastSessionError != nil { return .red }
        return flightAnnouncer.audioSessionActive ? .green : .orange
    }

    /// App-wide display-unit picker (#160).  Lives on the front screen so it's
    /// reachable whether or not a rocket is connected; the choice is global.
    private var unitsMenu: some View {
        Menu {
            Picker("Display Units", selection: $unitSystem) {
                ForEach(UnitSystem.allCases) { Text($0.label).tag($0) }
            }
        } label: {
            Image(systemName: "ruler")
        }
    }

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
                    HStack(spacing: 16) {
                        if fleet.isConnected {
                            Button {
                                fleet.disconnectAll()
                            } label: {
                                Image(systemName: "chevron.backward")
                            }
                        }
                        unitsMenu
                    }
                }

                ToolbarItem(placement: .navigationBarTrailing) {
                    if let device = fleet.activeDevice {
                        HStack(spacing: 16) {
                            // Rocket icon → pick / manage rocket profiles (#132).
                            // Hidden on a base station: rocket selection is
                            // read-only there (you manage rockets connected
                            // directly to a rocket).
                            if !device.isBaseStation {
                                NavigationLink(destination: RocketProfileView(device: device)) {
                                    Image("RocketIcon")
                                        .resizable()
                                        .renderingMode(.template)
                                        .aspectRatio(contentMode: .fit)
                                        .frame(width: 22, height: 22)
                                }
                            }

                            // Book icon → stored flights on the rocket (where the
                            // rocket icon used to go).
                            NavigationLink(destination: FileManagerView(device: device)) {
                                Image(systemName: "book.fill")
                            }

                            NavigationLink(destination: MapView(device: device)) {
                                Image(systemName: "map.fill")
                            }

                            Button {
                                flightAnnouncer.isEnabled.toggle()
                            } label: {
                                Image(systemName: voiceIconName)
                                    .foregroundColor(voiceIconColor)
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
            attachActiveDevice()
        }
        .onChange(of: fleet.isConnected) { connected in
            attachActiveDevice()
            if !connected && !fleet.isScanning {
                fleet.startScanning()
            }
            // Phone-location lifecycle is driven from ConnectedDashboardView,
            // which observes the device — fleet.isConnected flips before the
            // device's type resolves, so it's the wrong signal to gate on here.
        }
        // #375: the syncer/announcer used to attach only on the fleet's 0→1
        // connection edge, but the real lifecycle is per-BLEDevice — every
        // reconnect creates a NEW device object (BLEFleet.didConnect), and a
        // rocket connecting while the base station is already up never flips
        // isConnected. Re-attach whenever the device list changes (reconnect,
        // second device, removal) or the operator switches the active chip.
        // Redundant calls are safe: syncer.attach is idempotent for the same
        // device object, and the announcer assignment is too.
        .onChange(of: fleet.devices) { _ in
            attachActiveDevice()
        }
        .onChange(of: fleet.activeDeviceID) { _ in
            attachActiveDevice()
        }
        .sheet(item: $activeSheet) { sheet in
            Group {
                switch sheet {
                // Reverse Drift Cast is a standalone wind/trajectory tool — it
                // runs with no live device (#42).  "Send to Unit" inside the
                // view is the only part gated on an active connection.
                case .driftCast:
                    DriftCastView(device: fleet.activeDevice)
                case .simulator:
                    if let device = fleet.activeDevice { SimulationView(device: device) }
                case .settings:
                    if let device = fleet.activeDevice { SettingsView(device: device) }
                case .servoTest:
                    if let device = fleet.activeDevice {
                        ServoTestView(device: device,
                                      finMinDeg: Double(profileStore.activeProfile?.finMinDeg ?? -20),
                                      finMaxDeg: Double(profileStore.activeProfile?.finMaxDeg ?? 20))
                    }
                }
            }
            // SwiftUI sheets get a fresh environment by default — re-inject
            // fleet so SettingsView's FirmwareUpdateView can resolve the
            // OTASession off it (#15 second pass).
            .environmentObject(fleet)
        }
        .sheet(isPresented: $showProvisioning) {
            if let device = fleet.activeDevice {
                DeviceProvisioningSheet(device: device)
            }
        }
        .sheet(isPresented: Binding(
            // #394: only present the "Add Device" sheet for an explicit user
            // scan, not for a background reconnect/auto-scan that happens to
            // land while connected (which popped the sheet unsolicited).
            get: { fleet.userInitiatedScan && fleet.isConnected },
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

    /// Point the profile syncer and the flight announcer at the current
    /// active device (#375). Called from every lifecycle edge that can change
    /// which BLEDevice object is active: appear, connect/disconnect, device
    /// list changes (reconnects create a new object), and chip switches.
    /// The announcer follows the active device only — it's cleared from the
    /// others so a background rocket's telemetry can't interleave callouts.
    private func attachActiveDevice() {
        guard let dev = fleet.activeDevice else {
            syncer.detach()
            return
        }
        for other in fleet.devices where other !== dev {
            other.flightAnnouncer = nil
        }
        dev.flightAnnouncer = flightAnnouncer
        syncer.attach(device: dev, store: profileStore)
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
        // Active rocket profile summary + sync status (#132).
        ActiveRocketHeader(device: device)
            // Phone-location lifecycle lives here, on the view that actually
            // @ObservedObject's the device. deviceType (→ isBaseStation) resolves
            // a beat after connect, and BLEFleet doesn't forward child-device
            // changes, so the outer DashboardView never sees it. Start phone GPS
            // once the active device identifies as a base station; stop otherwise
            // and on teardown (disconnect / leaving the dashboard).
            .onAppear { if device.isBaseStation { locationManager.startUpdates() } }
            .onChange(of: device.isBaseStation) { isBaseStation in
                if isBaseStation { locationManager.startUpdates() }
                else { locationManager.stopUpdates() }
            }
            .onDisappear { locationManager.stopUpdates() }

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

        if device.deviceType == .unknown {
            // --- Role not yet known (#330): neutral "identifying" state.
            //     The connected-device view used to fail *open* to the rocket
            //     Power-On screen for anything not positively a base station,
            //     so `.unknown` (name parse miss, or the window before
            //     config_identity "dt" resolves) wrongly showed a rocket UI.
            //     Drive the screen off a *positive* role instead, with an
            //     explicit identifying state for the unresolved case.
            IdentifyingDeviceView(deviceName: device.displayName)
        } else if device.deviceType == .rocket && !device.hasReceivedTelemetry {
            // --- Power state unknown (#377): same fail-to-positive principle
            //     as #330 above, applied to power. A fresh/reconnected
            //     BLEDevice's zeroed TelemetryData reads pwr_pin_on == false,
            //     so this screen used to show "Power On" for an already-on
            //     rocket in the reconnect→first-frame window — and cmd 8 is a
            //     blind toggle, so one tap killed the FC rail (and the 180 s
            //     poweringOn spinner then masked it). Hold a neutral waiting
            //     state until the first frame confirms the actual state; a
            //     genuinely-off rocket still sends OC telemetry, so this
            //     resolves within ~a second either way.
            VStack(spacing: 12) {
                ProgressView()
                Text("Waiting for telemetry…")
                    .font(.headline)
                Text("Power state unknown until the first frame arrives.")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            .frame(maxWidth: .infinity)
            .padding(.vertical, 24)
        } else if device.deviceType == .rocket && !device.telemetry.pwr_pin_on {
            // --- Powered OFF (rocket only, confirmed by telemetry): show
            //     battery + power on button ---
            BatteryView(telemetry: device.telemetry, isBaseStation: false)

            Button {
                device.beginPowerOn()
            } label: {
                HStack {
                    if device.poweringOn {
                        ProgressView()
                            .progressViewStyle(CircularProgressViewStyle(tint: .white))
                        Text("Powering on…")
                    } else {
                        Image(systemName: "bolt.fill")
                        Text("Power On")
                    }
                }
                .font(.system(.title2, design: .default).weight(.semibold))
                .frame(maxWidth: .infinity)
                .padding()
                .background(device.poweringOn ? Color.gray : Color.green)
                .foregroundColor(.white)
                .cornerRadius(10)
            }
            .disabled(device.poweringOn)
        } else {
            // --- Powered ON (or base station): full telemetry dashboard ---

            // #95: telemetry freshness gate.  When BS is in SYNCING (no
            // rocket caught yet) we hide the rocket-data views so the
            // operator doesn't see the empty zero-init values rendered as
            // a fake "INIT" rocket.  When STALE we keep the views visible
            // but dim them and show a banner with the age.  Direct rocket
            // connections always come through as .live (no banner, no dim).
            let dataStatus = device.telemetry.data_status
            let showRocketViews = dataStatus != .syncing
            let staleOpacity: Double = dataStatus == .stale ? 0.5 : 1.0

            if device.isBaseStation && dataStatus != .live {
                TelemetryStatusBanner(status: dataStatus,
                                      ageMs: device.telemetry.data_age_ms)
            }

            if showRocketViews {
                RocketStateView(state: device.telemetry.state)
                    .opacity(staleOpacity)
            }

            // #393: gate on the rocket's REPORTED sim state (fs bit 8) OR the
            // local launch latch.  The latch alone dies on BLE reconnect
            // (BLEDevice is recreated), which made the Stop-sim control vanish
            // mid-sim; telemetry keeps the banner up as long as the sim runs.
            if device.simLaunched || device.telemetry.sim_active {
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

            if showRocketViews && device.isBaseStation {
                FlightSummaryView(telemetry: device.telemetry)
                    .opacity(staleOpacity)
            }

            SignalStrengthView(
                telemetry: device.telemetry,
                bleRSSI: device.connectedRSSI,
                isBaseStation: device.isBaseStation,
                locationManager: device.isBaseStation ? locationManager : nil,
                rocketFix: device.isBaseStation ? device.lastValidRocketFix : nil
            )

            // BatteryView shows BS battery (always live) + rocket battery.
            // When syncing/stale the rocket row is meaningless; we still
            // render the view (BS battery row is useful) but dim it on
            // stale to match the rest, and hide it entirely on syncing
            // since both rows would be empty/cached.
            if device.isBaseStation && dataStatus == .syncing {
                // BS-only: show a stripped view with just the BS battery row
                BSOnlyBatteryView(telemetry: device.telemetry)
            } else {
                BatteryView(telemetry: device.telemetry,
                            isBaseStation: device.isBaseStation)
                    .opacity(staleOpacity)
            }

            if showRocketViews {
                IMUView(telemetry: device.telemetry,
                        isBaseStation: device.isBaseStation,
                        orientationName: device.isBaseStation ? "" : device.imuOrientationName,
                        orientationMode: device.imuOrientationMode)
                    .opacity(staleOpacity)
            }

            if showRocketViews && !device.isBaseStation {
                FlightSummaryView(telemetry: device.telemetry)
                    .opacity(staleOpacity)
            }

            if showRocketViews {
                GPSView(telemetry: device.telemetry, compact: true)
                    .opacity(staleOpacity)
            }

            StatusFlagsView(telemetry: device.telemetry,
                            isBaseStation: device.isBaseStation)
                .opacity(showRocketViews ? staleOpacity : 1.0)

            // Pre-launch sensor health scorecard + go/no-go (#303), sitting just
            // above the pyro channels.  Only shown once the rocket reports a
            // scorecard ("h" present → hasSensorHealth).
            if showRocketViews && device.telemetry.hasSensorHealth {
                HealthCardView(telemetry: device.telemetry)
                    .opacity(staleOpacity)
            }

            if !device.isBaseStation {
                PyroChannelsView(device: device)
            }

            ControlsView(device: device)

            TestingControlsView(device: device, activeSheet: $activeSheet)

            // Gyro/accel calibration lives in Settings → General → Calibration
            // now (#132), so it's no longer shown on the dashboard.

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

/// Neutral placeholder shown while a connected device's role is still
/// `.unknown` (#330) — i.e. its advertised name didn't parse to a role and
/// the `config_identity` "dt" readback hasn't arrived yet.  This replaces the
/// old fail-open behaviour where any non-base-station device (including the
/// not-yet-identified state) landed on the rocket "Power On" screen.  As soon
/// as the role resolves to .rocket or .baseStation the parent re-renders into
/// the appropriate screen, so this is normally only visible for a beat.
struct IdentifyingDeviceView: View {
    let deviceName: String

    var body: some View {
        VStack(spacing: 12) {
            ProgressView()
                .progressViewStyle(CircularProgressViewStyle())
            Text("Identifying device…")
                .font(.headline)
            if !deviceName.isEmpty {
                Text(deviceName)
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 32)
        .background(Color(.secondarySystemBackground))
        .cornerRadius(10)
    }
}

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
                    fleet.startScanning(userInitiated: true)  // #394: explicit "Add" opens the sheet
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
        case "MAG_CAL": return .blue   // ground-only excursion (issue #96)
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

/// Pre-launch sensor health scorecard + go/no-go recommendation (#303).
/// Driven by the packed "h" bitfield (FC sensors/EKF + OC battery); the parent
/// only renders it once the rocket actually reports a scorecard.  The rollup
/// rule lives in TelemetryData.flightReadiness — EKF-init and a GNSS fix gate
/// green, configured pyros must have continuity, and mag is advisory.
struct HealthCardView: View {
    let telemetry: TelemetryData

    private func color(for state: TelemetryData.SensorHealth) -> Color {
        switch state {
        case .ok:       return .green
        case .degraded: return .orange
        case .bad:      return .red
        case .na:       return .gray
        }
    }
    private var readinessColor: Color {
        switch telemetry.flightReadiness {
        case .ready:    return .green
        case .caution:  return .orange
        case .notReady: return .red
        case .unknown:  return .gray
        }
    }
    private var readinessIcon: String {
        switch telemetry.flightReadiness {
        case .ready:    return "checkmark.seal.fill"
        case .caution:  return "exclamationmark.triangle.fill"
        case .notReady: return "xmark.octagon.fill"
        case .unknown:  return "hourglass"
        }
    }

    private let columns = [GridItem(.adaptive(minimum: 104), spacing: 8)]

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Sensor Health")
                .font(.headline)

            // Go/no-go recommendation banner.
            HStack(spacing: 10) {
                Image(systemName: readinessIcon)
                    .font(.title2)
                    .foregroundColor(readinessColor)
                Text(telemetry.flightReadiness.label)
                    .font(.system(.title3, design: .default).weight(.semibold))
                    .foregroundColor(readinessColor)
                Spacer(minLength: 0)
            }
            .padding(10)
            .frame(maxWidth: .infinity, alignment: .leading)
            .background(readinessColor.opacity(0.12))
            .cornerRadius(8)

            // Per-sensor chips — core sensors always, configured pyros appended.
            LazyVGrid(columns: columns, alignment: .leading, spacing: 8) {
                ForEach(telemetry.sensorHealthRows) { row in
                    HStack(spacing: 6) {
                        Circle()
                            .fill(color(for: row.state))
                            .frame(width: 8, height: 8)
                        Text(row.name)
                            .font(.caption)
                        Spacer(minLength: 2)
                        Text(row.state.label)
                            .font(.caption2.weight(.semibold))
                            .foregroundColor(color(for: row.state))
                    }
                    .padding(.horizontal, 8)
                    .padding(.vertical, 6)
                    .background(color(for: row.state).opacity(0.10))
                    .cornerRadius(6)
                }
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

/// Banner shown when the BS is reporting non-LIVE telemetry status (#95).
/// SYNCING: BS hasn't caught the rocket yet — operator-facing "searching"
/// hint, paired with hiding rocket-data views in the parent.
/// STALE: BS is still re-pushing cached values older than the staleness
/// threshold — show how old in seconds so the operator knows it's not
/// live; parent dims rocket-data views.
struct TelemetryStatusBanner: View {
    let status: TelemetryData.DataStatus
    let ageMs: UInt32

    private var icon: String {
        status == .syncing ? "antenna.radiowaves.left.and.right.slash" : "clock.badge.exclamationmark"
    }
    private var title: String {
        status == .syncing ? "Searching for rocket…" : "Telemetry stale"
    }
    private var subtitle: String {
        switch status {
        case .syncing: return "Base station hasn't received any telemetry yet. Power on the rocket and wait for the link to sync."
        case .stale:   return "Last packet \(formatElapsed(ms: ageMs)) ago. Showing cached values."
        case .live:    return ""
        }
    }
    private var tint: Color {
        status == .syncing ? .blue : .orange
    }

    var body: some View {
        HStack(alignment: .center, spacing: 12) {
            Image(systemName: icon)
                .font(.title2)
                .foregroundColor(tint)
            VStack(alignment: .leading, spacing: 2) {
                Text(title).font(.headline)
                Text(subtitle).font(.caption).foregroundColor(.secondary)
            }
            Spacer()
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(tint.opacity(0.1))
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

// Flash-space card: a segmented used / reserved / free bar for the connected
// device — rocket NAND on a direct rocket link, base-station FS on a BS link
// (v1: each device reports its own; no rocket-over-LoRa plumbing).
struct StorageBarView: View {
    @ObservedObject var device: BLEDevice

    var body: some View {
        Group {
            if device.isBaseStation {
                if let s = device.bsStorage, s.totalBytes > 0 {
                    card(title: "Base Station Storage", subtitle: s.backendName,
                         used: s.usedBytes, reserved: s.reservedBytes, free: s.freeBytes,
                         total: s.totalBytes)
                }
            } else if let s = device.rocketStorage, s.initialized {
                card(title: "Rocket Storage",
                     subtitle: "\(s.flightCount) flight\(s.flightCount == 1 ? "" : "s")",
                     used: s.usedBytes, reserved: s.reservedBytes, free: s.freeBytes,
                     total: s.totalBytes)
            }
        }
    }

    private func card(title: String, subtitle: String,
                      used: Int, reserved: Int, free: Int, total: Int) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text(title).font(.headline)
                Spacer()
                Text(subtitle).font(.caption).foregroundColor(.secondary)
            }
            StorageSegmentBar(used: used, reserved: reserved, free: free, total: total)
            HStack(spacing: 14) {
                legend(.orange, "Used", used)
                if reserved > 0 { legend(Color(.systemGray2), "Reserved", reserved) }
                legend(.green, "Free", free)
                Spacer()
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }

    private func legend(_ color: Color, _ label: String, _ bytes: Int) -> some View {
        HStack(spacing: 5) {
            RoundedRectangle(cornerRadius: 2).fill(color).frame(width: 10, height: 10)
            Text("\(label) \(StorageBarView.fmt(bytes))")
                .font(.caption).foregroundColor(.secondary)
        }
    }

    static func fmt(_ bytes: Int) -> String {
        let f = ByteCountFormatter()
        f.allowedUnits = [.useMB, .useGB]
        f.countStyle = .binary
        return f.string(fromByteCount: Int64(bytes))
    }
}

struct StorageSegmentBar: View {
    let used: Int
    let reserved: Int
    let free: Int
    let total: Int

    var body: some View {
        GeometryReader { geo in
            let w = geo.size.width
            let t = CGFloat(max(total, 1))
            HStack(spacing: 0) {
                Color.orange.frame(width: w * CGFloat(used) / t)
                Color(.systemGray2).frame(width: w * CGFloat(reserved) / t)
                Color.green.frame(width: w * CGFloat(free) / t)
                Spacer(minLength: 0)
            }
        }
        .frame(height: 14)
        .clipShape(Capsule())
        .overlay(Capsule().stroke(Color(.systemGray4), lineWidth: 0.5))
    }
}

/// Stripped-down BatteryView used while the BS hasn't caught the rocket yet
/// (#95).  Hides the rocket row entirely (its values would be empty/cached)
/// and shows only the live BS battery row, so the operator still has a
/// useful battery indicator without the misleading rocket data.
struct BSOnlyBatteryView: View {
    let telemetry: TelemetryData

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Battery")
                .font(.headline)

            HStack(spacing: 0) {
                Text("")
                    .frame(width: 70, alignment: .leading)
                Text("Charge").font(.caption).foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                Text("Voltage").font(.caption).foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                Text("Current").font(.caption).foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
            }

            BatteryRow(label: "Base Stn",
                       charge: telemetry.bsSocDisplay,
                       voltage: telemetry.bsVoltageDisplay,
                       current: telemetry.bsCurrentDisplay)
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
    // Board→rocket mounting orientation (direct rocket connection only).
    // Empty until the FC reports it; shown so the flyer can confirm the
    // auto-detected mounting before arming.
    var orientationName: String = ""
    var orientationMode: IMUOrientationMode = .unknown

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack {
                Text("IMU")
                    .font(.headline)
                Spacer()
                if !orientationName.isEmpty {
                    Text("nose: \(orientationName) (\(orientationMode.label))")
                        .font(.caption)
                        .foregroundColor(orientationMode == .autoExact ? .orange : .secondary)
                }
            }

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
                   decimals: 2, isAcceleration: true)

            // High-G only available on direct rocket connection (not via LoRa)
            if !isBaseStation {
                IMURow(label: "High-G", unit: "m/s\u{00B2}",
                       x: telemetry.high_g_x, y: telemetry.high_g_y, z: telemetry.high_g_z,
                       decimals: 1, isAcceleration: true)
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
    // Acceleration rows convert to g in imperial; gyro (°/s) stays as-is.
    var isAcceleration: Bool = false
    @AppStorage("unitSystem") private var unitSystem: UnitSystem = .metric

    private var displayUnit: String {
        isAcceleration ? UnitFormatter.accelerationUnit(unitSystem) : unit
    }

    private func fmt(_ val: Float?) -> String {
        guard let v = val else { return "—" }
        let value = isAcceleration
            ? UnitFormatter.accelerationValue(Double(v), system: unitSystem)
            : Double(v)
        return String(format: "%.\(decimals)f", value)
    }

    var body: some View {
        HStack(spacing: 0) {
            VStack(alignment: .leading, spacing: 2) {
                Text(label)
                    .font(.caption)
                Text(displayUnit)
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
                    active: telemetry.rocketLoggingActive
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

            // BS silence-close countdown.  Only rendered when the base
            // station reports a remaining-seconds value, which the BS only
            // sends while bs_logging_active=true (see TR_BLE_To_APP.cpp).
            // Color tracks urgency: > 60 s green, 11..60 s orange, ≤ 10 s
            // red so the operator can spot an imminent close at a glance.
            if isBaseStation,
               telemetry.bs_logging_active,
               let remaining = telemetry.bs_log_silence_remaining_s {
                let secs = Int(remaining)
                let color: Color = secs > 60 ? .green : (secs > 10 ? .orange : .red)
                HStack(spacing: 6) {
                    Image(systemName: "timer")
                        .foregroundColor(color)
                    Text("Auto-close in \(formatElapsed(seconds: secs))")
                        .font(.caption)
                        .foregroundColor(color)
                }
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

// MARK: - Pyro Channels

struct PyroChannelsView: View {
    @ObservedObject var device: BLEDevice
    @AppStorage("unitSystem") private var unitSystem: UnitSystem = .metric
    @State private var showPyroSheet = false
    @State private var editingChannel: Int = 1
    @State private var contTestChannel: Int = 0   // 0 = none, 1 = CH1, 2 = CH2

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Pyro Channels")
                .font(.headline)

            // 2x2 grid for the 4 channels. The global "armed" bit (single
            // shared ARM FET) drives the cont badge visibility for every tile.
            VStack(spacing: 10) {
                HStack(spacing: 10) {
                    pyroTileForChannel(1)
                    pyroTileForChannel(2)
                }
                HStack(spacing: 10) {
                    pyroTileForChannel(3)
                    pyroTileForChannel(4)
                }
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
        .sheet(isPresented: $showPyroSheet) {
            PyroConfigSheet(device: device, channel: editingChannel)
        }
    }

    /// Pull the right enabled/mode/value for the channel from rocketConfig.
    private func pyroChannelConfig(_ ch: Int) -> (enabled: Bool, mode: UInt8, value: Float) {
        guard let cfg = device.rocketConfig else { return (false, 0, 0) }
        switch ch {
        case 1: return (cfg.pyro1Enabled, cfg.pyro1TriggerMode, cfg.pyro1TriggerValue)
        case 2: return (cfg.pyro2Enabled, cfg.pyro2TriggerMode, cfg.pyro2TriggerValue)
        case 3: return (cfg.pyro3Enabled, cfg.pyro3TriggerMode, cfg.pyro3TriggerValue)
        case 4: return (cfg.pyro4Enabled, cfg.pyro4TriggerMode, cfg.pyro4TriggerValue)
        default: return (false, 0, 0)
        }
    }

    private func pyroTileForChannel(_ ch: Int) -> some View {
        let cfg = pyroChannelConfig(ch)
        return pyroTile(
            channel: ch,
            enabled: cfg.enabled,
            armed: device.telemetry.pyro_armed,
            // #297: only trust continuity from a LIVE frame.  After the
            // stale/auto-hide window a held-over frame would otherwise keep
            // showing green "CONT" from old data; gate to .live so a stale
            // frame reads NO CONT (fail-safe).
            continuity: device.telemetry.pyroCont(channel: ch)
                        && device.telemetry.data_status == .live,
            fired: device.telemetry.pyroFired(channel: ch),
            mode: cfg.mode,
            value: cfg.value)
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
                            if device.contTestPending(channel: UInt8(channel)) {
                                HStack(spacing: 4) {
                                    ProgressView().controlSize(.mini)
                                    Text("TESTING")
                                        .font(.caption2.weight(.bold))
                                        .foregroundColor(.secondary)
                                }
                            } else {
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
                // Test Pyro Channel (test-fire) lives on the detailed Pyro
                // settings screen only — kept off the main dashboard tile.
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
        }
        // Altitude on descent: stored in meters, shown in the display unit.
        let alt = UnitFormatter.metersToDisplay(Double(value), system: unitSystem)
        return String(format: "%.0f%@ on descent", alt, UnitFormatter.altitudeUnit(unitSystem))
    }
}

struct PyroConfigSheet: View {
    @ObservedObject var device: BLEDevice
    @EnvironmentObject var store: RocketProfileStore
    let channel: Int
    @Environment(\.dismiss) var dismiss
    @AppStorage("unitSystem") private var unitSystem: UnitSystem = .metric

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
                    Text(triggerMode == 0 ? "Delay (s)" : "Altitude (\(UnitFormatter.altitudeUnit(unitSystem)))")
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
        switch channel {
        case 1:
            enabled = cfg.pyro1Enabled
            triggerMode = Int(cfg.pyro1TriggerMode)
            triggerValue = formatTriggerValue(cfg.pyro1TriggerValue, mode: triggerMode)
        case 2:
            enabled = cfg.pyro2Enabled
            triggerMode = Int(cfg.pyro2TriggerMode)
            triggerValue = formatTriggerValue(cfg.pyro2TriggerValue, mode: triggerMode)
        case 3:
            enabled = cfg.pyro3Enabled
            triggerMode = Int(cfg.pyro3TriggerMode)
            triggerValue = formatTriggerValue(cfg.pyro3TriggerValue, mode: triggerMode)
        case 4:
            enabled = cfg.pyro4Enabled
            triggerMode = Int(cfg.pyro4TriggerMode)
            triggerValue = formatTriggerValue(cfg.pyro4TriggerValue, mode: triggerMode)
        default: break
        }
    }

    /// Time stays in seconds; altitude is stored in meters but shown/edited
    /// in the display unit.
    private func formatTriggerValue(_ v: Float, mode: Int) -> String {
        if mode == 0 { return String(format: "%.1f", v) }
        return String(format: "%.0f", UnitFormatter.metersToDisplay(Double(v), system: unitSystem))
    }

    private func parseTriggerValue(_ s: String, mode: Int) -> Float {
        let entered = Float(s) ?? 0
        if mode == 0 { return entered }
        return Float(UnitFormatter.displayToMeters(Double(entered), system: unitSystem))
    }

    func saveAndSend() {
        let val = parseTriggerValue(triggerValue, mode: triggerMode)
        guard var cfg = device.rocketConfig else { return }
        switch channel {
        case 1:
            cfg.pyro1Enabled = enabled
            cfg.pyro1TriggerMode = UInt8(triggerMode)
            cfg.pyro1TriggerValue = val
        case 2:
            cfg.pyro2Enabled = enabled
            cfg.pyro2TriggerMode = UInt8(triggerMode)
            cfg.pyro2TriggerValue = val
        case 3:
            cfg.pyro3Enabled = enabled
            cfg.pyro3TriggerMode = UInt8(triggerMode)
            cfg.pyro3TriggerValue = val
        case 4:
            cfg.pyro4Enabled = enabled
            cfg.pyro4TriggerMode = UInt8(triggerMode)
            cfg.pyro4TriggerValue = val
        default: return
        }
        device.rocketConfig = cfg
        device.sendPyroConfig(channels: [
            (cfg.pyro1Enabled, cfg.pyro1TriggerMode, cfg.pyro1TriggerValue),
            (cfg.pyro2Enabled, cfg.pyro2TriggerMode, cfg.pyro2TriggerValue),
            (cfg.pyro3Enabled, cfg.pyro3TriggerMode, cfg.pyro3TriggerValue),
            (cfg.pyro4Enabled, cfg.pyro4TriggerMode, cfg.pyro4TriggerValue),
        ])
        // Persist pyro config to the active rocket profile too (#132) so the
        // connect-time syncer doesn't push a stale value back over this edit.
        if let id = store.activeId {
            store.update(id) {
                $0.pyro1Enabled = cfg.pyro1Enabled
                $0.pyro1TriggerMode = cfg.pyro1TriggerMode
                $0.pyro1TriggerValue = cfg.pyro1TriggerValue
                $0.pyro2Enabled = cfg.pyro2Enabled
                $0.pyro2TriggerMode = cfg.pyro2TriggerMode
                $0.pyro2TriggerValue = cfg.pyro2TriggerValue
                $0.pyro3Enabled = cfg.pyro3Enabled
                $0.pyro3TriggerMode = cfg.pyro3TriggerMode
                $0.pyro3TriggerValue = cfg.pyro3TriggerValue
                $0.pyro4Enabled = cfg.pyro4Enabled
                $0.pyro4TriggerMode = cfg.pyro4TriggerMode
                $0.pyro4TriggerValue = cfg.pyro4TriggerValue
            }
        }
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
                // Cmd 23 starts/stops logging on BOTH rocket and base station, so
                // reflect "logging in progress" if either side reports active.
                // The rocket side is gated on rocket state (#137 follow-up) — see
                // rocketLoggingActive on TelemetryData for why.
                let isLogging = device.telemetry.rocketLoggingActive || device.telemetry.bs_logging_active
                HStack {
                    Image(systemName: isLogging ? "stop.circle.fill" : "record.circle")
                    Text(isLogging ? "Stop Logging" : "Start Logging")
                }
                .frame(maxWidth: .infinity)
                .padding()
                .background(isLogging ? Color.red : Color.orange)
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
    @EnvironmentObject var store: RocketProfileStore
    /// When embedded in a Form (e.g. the settings General tab) we drop the
    /// card chrome + headline so it sits naturally as a row (#132).
    var embedded: Bool = false
    @AppStorage("unitSystem") private var unitSystem: UnitSystem = .metric
    @State private var calibrating = false
    @State private var showGravityWarning = false
    @State private var gravityMag: Float = 0.0
    @State private var gravityError: Float = 0.0
    /// Set when the user kicks off a cal so the next sensor-cal readback gets
    /// snapshotted into the active profile (#132) — not arbitrary frames.
    @State private var pendingCalSnapshot = false

    /// Shared logic for the cal action so the embedded list-row Button
    /// and the standalone dashboard card use exactly the same path.
    private func runCalibration() {
        calibrating = true
        pendingCalSnapshot = true
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
    }

    private var calIsDisabled: Bool {
        calibrating || !device.isConnected || !device.telemetry.pwr_pin_on
    }

    var body: some View {
        Group {
            if embedded {
                // List-row form: matches the mag-cal NavigationLink style
                // so both calibration entry points read as identical
                // settings rows.  No background card, no chevron.
                Button(action: runCalibration) {
                    HStack {
                        Label(calibrating ? "Calibrating…" : "Gyro & Accel Calibration",
                              systemImage: "gyroscope")
                        Spacer()
                        if calibrating {
                            ProgressView()
                        }
                    }
                }
                .disabled(calIsDisabled)
            } else {
                standaloneCard
            }
        }
        .alert("Accelerometer Warning", isPresented: $showGravityWarning) {
            Button("OK", role: .cancel) { }
        } message: {
            Text("Low-G accelerometer magnitude (\(UnitFormatter.acceleration(Double(gravityMag), system: unitSystem))) differs from expected gravity (\(UnitFormatter.acceleration(9.80665, system: unitSystem))) by \(String(format: "%.1f", gravityError))%. Consider running a bench calibration before flight.")
        }
        // Snapshot the freshly-run sensor cal into the active profile (#132),
        // tagged with this board's id so the syncer re-applies it on connect.
        .onChange(of: device.sensorCalStatus) { newStatus in
            guard pendingCalSnapshot, let s = newStatus, s.valid,
                  !device.unitID.isEmpty, let id = store.activeId else { return }
            pendingCalSnapshot = false
            store.update(id) { $0.sensorCal = SensorCalData(status: s, unitID: device.unitID) }
        }
    }

    /// Standalone card style used on the Dashboard "On Pad" panel.
    private var standaloneCard: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("On Pad Calibration")
                .font(.headline)

            Button(action: runCalibration) {
                HStack {
                    if calibrating {
                        ProgressView()
                            .tint(.white)
                    }
                    Image(systemName: "gyroscope")
                    Text(calibrating ? "Calibrating..." : "Calibrate Gyro and Accel")
                }
                .frame(maxWidth: .infinity)
                .padding()
                .background(calibrating ? Color.orange : Color.blue)
                .foregroundColor(.white)
                .cornerRadius(10)
            }
            .disabled(calIsDisabled)

            Text("Place rocket on pad and keep still. Calibrates gyro bias and accelerometer offsets (~10 seconds).")
                .font(.caption)
                .foregroundColor(.secondary)
        }
        .padding(16)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
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
    var rocketFix: LastValidRocketFix? = nil
    @AppStorage("unitSystem") private var unitSystem: UnitSystem = .metric

    var body: some View {
        VStack(spacing: 10) {
            Text("Signal")
                .font(.headline)

            HStack(spacing: 30) {
                // Direction arrow (base station only). Uses the latched
                // lastValidRocketFix (same source as MapView) rather than
                // telemetry.latitude — relay frames frequently arrive with
                // lat/lon = nil (#140), so reading the live frame left the
                // arrow blank on almost every render.
                if isBaseStation, let locMgr = locationManager,
                   let fix = rocketFix,
                   let phoneLoc = locMgr.userLocation {

                    let dist = LocationManager.haversineDistance(
                        lat1: phoneLoc.latitude, lon1: phoneLoc.longitude,
                        lat2: fix.latitude, lon2: fix.longitude
                    )
                    let bear = LocationManager.bearing(
                        lat1: phoneLoc.latitude, lon1: phoneLoc.longitude,
                        lat2: fix.latitude, lon2: fix.longitude
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
                            Text("\(UnitFormatter.altitude(abs(altDiff), system: unitSystem)) \(altDiff >= 0 ? "Up" : "Down")")
                                .font(.system(.caption, design: .monospaced))
                                .foregroundColor(.primary)
                        }

                        Text("\(UnitFormatter.distance(dist, system: unitSystem)) Away")
                            .font(.system(.caption, design: .monospaced))
                            .foregroundColor(.primary)

                        Text("Rocket")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                } else if isBaseStation, let locMgr = locationManager {
                    // Say why the arrow is hidden instead of showing nothing.
                    // "phone" = app hasn't received a phone location yet (NOT a
                    // signal-quality problem); "rocket" = no GPS fix latched yet.
                    VStack(spacing: 6) {
                        Image(systemName: "location.slash")
                            .font(.system(size: 44))
                            .foregroundColor(.secondary)
                        Text(locMgr.userLocation == nil ? "Getting phone\nlocation…" : "Waiting for\nrocket GPS")
                            .font(.caption2)
                            .multilineTextAlignment(.center)
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
