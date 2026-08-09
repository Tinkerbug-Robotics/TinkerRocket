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
/// #390: sheets carry their target device — with one section per rocket
/// on screen, "the active device" no longer identifies which unit a
/// sheet should talk to.
enum DashboardSheet: Identifiable {
    case simulator(BLEDevice)
    case settings(BLEDevice)
    case servoTest(BLEDevice)
    case driftCast
    case frequencyScan(BLEDevice)   // #150: restored (removed in #136)

    var id: String {
        switch self {
        case .simulator(let d):     return "simulator-\(d.peripheral?.identifier.uuidString ?? "")"
        case .settings(let d):      return "settings-\(d.peripheral?.identifier.uuidString ?? "")"
        case .servoTest(let d):     return "servoTest-\(d.peripheral?.identifier.uuidString ?? "")"
        case .driftCast:            return "driftCast"
        case .frequencyScan(let d): return "frequencyScan-\(d.peripheral?.identifier.uuidString ?? "")"
        }
    }
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
                    if fleet.isConnected {
                        ConnectedFleetView(
                            fleet: fleet,
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
                            .background(TRColor.savedFlights)
                            .foregroundColor(.white)
                            .cornerRadius(TRShape.radiusButton)
                        }

                        // One place to rename any known rocket/base station and
                        // manage network IDs — including while they're offline
                        // (edits queue and apply on the next connect).
                        NavigationLink(destination: DeviceManagerView(fleet: fleet)) {
                            HStack {
                                Image(systemName: "list.bullet.rectangle")
                                Text("My Devices")
                            }
                            .frame(maxWidth: .infinity)
                            .padding()
                            .background(TRColor.myDevices)
                            .foregroundColor(.white)
                            .cornerRadius(TRShape.radiusButton)
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
                            .background(TRColor.driftCast)
                            .foregroundColor(.white)
                            .cornerRadius(TRShape.radiusButton)
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
                                    .background(fleet.isScanning ? TRColor.scanActive : TRColor.scan)
                                    .foregroundColor(.white)
                                    .cornerRadius(TRShape.radiusButton)
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
                    // #390: toolbar screens (files/map/settings) target the
                    // foreground base station when one is connected — it is
                    // the session's infrastructure — else the first link.
                    if let device = fleet.foregroundBaseStation ?? fleet.activeDevice {
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
                                activeSheet = .settings(device)
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
        // A renamed device can mis-parse its role from the BLE name until
        // config_identity lands; the syncer re-evaluates on a role flip
        // (attachedAsBaseStation), but only if attach is called again —
        // this edge is what calls it.
        .onChange(of: fleet.activeDevice?.deviceType) { _ in
            attachActiveDevice()
        }
        .onChange(of: fleet.foregroundBSID) { _ in
            attachActiveDevice()   // #390: voice follows the foreground pair
        }
        .sheet(item: $activeSheet) { sheet in
            Group {
                switch sheet {
                // Reverse Drift Cast is a standalone wind/trajectory tool — it
                // runs with no live device (#42).  "Send to Unit" inside the
                // view is the only part gated on an active connection.
                case .driftCast:
                    DriftCastView(device: fleet.activeDevice)
                case .simulator(let device):
                    SimulationView(device: device)
                case .settings(let device):
                    SettingsView(device: device)
                case .servoTest(let device):
                    ServoTestView(device: device,
                                  finMinDeg: Double(profileStore.activeProfile?.finMinDeg ?? -20),
                                  finMaxDeg: Double(profileStore.activeProfile?.finMaxDeg ?? 20))
                case .frequencyScan(let device):
                    // #150: restored — the scan pipeline (cmd 60 → 0xAA
                    // blob → scanSamples) survived #136 intact; while
                    // hopping, the BS runs it as a coordinated scan and
                    // pushes the resulting skip-mask via cmd 15.
                    NavigationView { FrequencyScanView(device: device) }
                }
            }
            // SwiftUI sheets get a fresh environment by default — re-inject
            // fleet so SettingsView's FirmwareUpdateView can resolve the
            // OTASession off it (#15 second pass).
            .environmentObject(fleet)
        }
        .sheet(isPresented: $showProvisioning) {
            if let device = fleet.activeDevice {
                DeviceProvisioningSheet(device: device, store: fleet.knownDevices)
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
                  !fleet.knownDevices.isProvisioned(id),
                  !showProvisioning else { return }
            showProvisioning = true
        }
    }

    /// Point the profile syncer and the flight announcer at the right links
    /// (#375, #390). Called from every lifecycle edge that can change which
    /// BLEDevice objects exist or matter: appear, connect/disconnect, device
    /// list changes (reconnects create a new object), pair switches.
    ///
    /// Voice: the first direct rocket link when one exists (full frame
    /// rate), else the foreground base station — whose stream is pinned to
    /// its focused rocket, so callouts never interleave two rockets.
    /// Sync: profiles only push over a direct rocket link.
    private func attachActiveDevice() {
        let directRocket = fleet.devices.first {
            $0.isConnected && $0.deviceType == .rocket
        }
        let voiceDevice = directRocket ?? fleet.foregroundBaseStation

        for other in fleet.devices where other !== voiceDevice {
            other.flightAnnouncer = nil
        }
        voiceDevice?.flightAnnouncer = flightAnnouncer

        if let rocket = directRocket {
            syncer.attach(device: rocket, store: profileStore)
        } else {
            syncer.detach()
        }
    }
}

// MARK: - Connected fleet dashboard (#390)

/// The rocket-centric dashboard: pair switcher (≥2 base stations), the
/// always-visible units bar, the foreground base station's strip, then one
/// section per displayed rocket. The base station stopped impersonating a
/// rocket — its stream renders only the section of the rocket its link is
/// focused on.
struct ConnectedFleetView: View {
    @ObservedObject var fleet: BLEFleet
    @ObservedObject var locationManager: LocationManager
    @Binding var activeSheet: DashboardSheet?

    /// Roster scoped to what this screen is about: rockets carried by the
    /// foreground base station, plus anything directly connected (an
    /// explicit user action always shows), plus identifying links.
    private var scopedSubjects: [RocketSubject] {
        let foreground = fleet.foregroundBaseStation
        return fleet.rockets.filter { subject in
            if subject.direct != nil { return true }
            guard let fg = foreground else { return false }
            return subject.relays.contains { $0.baseStation === fg }
        }
    }

    private var displayedSubjects: [RocketSubject] {
        scopedSubjects.filter { subject in
            guard let key = subject.key else { return true }
            return !fleet.hiddenRocketKeys.contains(key)
        }
    }

    var body: some View {
        let subjects = scopedSubjects
        let displayed = displayedSubjects
        let multi = displayed.count > 1
        let foregroundBS = fleet.foregroundBaseStation

        if fleet.baseStations.count > 1 {
            PairSwitcherView(fleet: fleet)
        }

        UnitsBarView(fleet: fleet, subjects: subjects)

        if let bs = foregroundBS {
            // Passive display, deliberately not a navigation entry: the
            // operator's model is "select the device you want to act on" —
            // the strip only says which rocket this BS follows.  BS tools
            // live in the toolbar (settings/logs) and the Controls card.
            BaseStationStripView(bs: bs)
                // Phone GPS drives the direction-to-rocket arrow — only
                // useful while a base station is relaying positions.
                .onAppear {
                    locationManager.startUpdates()
                    // Re-report promptly on (re)appear: the fix at the start
                    // of a logging session is the one worth having.
                    locationManager.resetFixThrottle()
                }
                .onDisappear { locationManager.stopUpdates() }
                // …and, since the phone IS the base station's position, push
                // it down so the CSV records it. The BS has no GNSS, so
                // without this the range between the two ends is displayed
                // and then discarded, and a range test cannot be
                // reconstructed from its own logs. Throttled inside
                // reportFix; the BS ignores it unless a session is open.
                .onReceive(locationManager.$userLocation) { _ in
                    locationManager.reportFix(to: bs)
                }
        }

        ForEach(displayed) { subject in
            if let direct = subject.direct {
                DirectRocketSection(subject: subject,
                                    device: direct,
                                    fleet: fleet,
                                    locationManager: locationManager,
                                    activeSheet: $activeSheet,
                                    collapsible: multi)
            } else if let bs = foregroundBS,
                      subject.key?.rocketID == bs.focusRocketID,
                      subject.relays.contains(where: { $0.baseStation === bs }) {
                // The focused rocket rides the BS link's pinned mirror —
                // the full interactive dashboard.
                FocusedRelaySection(subject: subject,
                                    bs: bs,
                                    fleet: fleet,
                                    locationManager: locationManager,
                                    activeSheet: $activeSheet,
                                    collapsible: multi)
            } else if let relay = subject.freshestRelay {
                RelayRocketSectionView(subject: subject,
                                       via: relay.baseStation,
                                       remote: relay.remote,
                                       collapsible: multi,
                                       fleet: fleet,
                                       locationManager: locationManager)
            }
        }

        // Base station connected but nothing heard yet: the searching
        // state (SYNCING) renders through the BS device content.
        if let bs = foregroundBS, bs.remoteRockets.isEmpty {
            ConnectedDashboardView(device: bs,
                                   fleet: fleet,
                                   locationManager: locationManager,
                                   activeSheet: $activeSheet)
        }
    }
}

/// Full interactive section for a directly connected rocket.
private struct DirectRocketSection: View {
    let subject: RocketSubject
    @ObservedObject var device: BLEDevice
    @ObservedObject var fleet: BLEFleet
    @ObservedObject var locationManager: LocationManager
    @Binding var activeSheet: DashboardSheet?
    let collapsible: Bool
    @State private var collapsed = false

    var body: some View {
        // Header only when it does real work: separating sections and
        // hosting the collapse control with 2+ rockets displayed. Solo,
        // the units-bar chip and (via BS) the strip already name the
        // rocket — phone-tested as pure redundancy.
        if collapsible {
            RocketSectionHeader(subject: subject,
                                collapsible: collapsible,
                                collapsed: $collapsed)
        }
        if collapsed {
            CollapsedRocketSummary(telemetry: device.telemetry)
        } else {
            // Active rocket profile summary + sync status (#132).
            ActiveRocketHeader(device: device)
            ConnectedDashboardView(device: device,
                                   fleet: fleet,
                                   locationManager: locationManager,
                                   activeSheet: $activeSheet)
        }
    }
}

/// Full interactive section for the base-station-focused rocket, driven by
/// the BS link's pinned telemetry mirror.
private struct FocusedRelaySection: View {
    let subject: RocketSubject
    @ObservedObject var bs: BLEDevice
    @ObservedObject var fleet: BLEFleet
    @ObservedObject var locationManager: LocationManager
    @Binding var activeSheet: DashboardSheet?
    let collapsible: Bool
    @State private var collapsed = false

    var body: some View {
        // Same solo-redundancy rule as DirectRocketSection: the BS strip
        // right above already says which rocket its radio follows.
        if collapsible {
            RocketSectionHeader(subject: subject,
                                collapsible: collapsible,
                                collapsed: $collapsed)
        }
        if collapsed {
            CollapsedRocketSummary(telemetry: bs.telemetry)
        } else {
            ConnectedDashboardView(device: bs,
                                   fleet: fleet,
                                   locationManager: locationManager,
                                   activeSheet: $activeSheet)
        }
    }
}

// MARK: - Connected device dashboard (observes the BLEDevice for live updates)

struct ConnectedDashboardView: View {
    @ObservedObject var device: BLEDevice
    @ObservedObject var fleet: BLEFleet
    @ObservedObject var locationManager: LocationManager
    @Binding var activeSheet: DashboardSheet?

    var body: some View {
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
            // #390: effectiveDataStatus overlays app-computed staleness of
            // the FOCUSED rocket — with the relay mirror pinned, the BS's
            // re-push may describe a different rocket and get dropped, so
            // the frame-carried status alone could freeze at .live.
            let dataStatus = device.effectiveDataStatus
            let showRocketViews = dataStatus != .syncing
            let staleOpacity: Double = dataStatus == .stale ? 0.5 : 1.0

            if device.isBaseStation && dataStatus != .live {
                TelemetryStatusBanner(status: dataStatus,
                                      ageMs: device.effectiveDataAgeMs)
            }

            if showRocketViews {
                RocketStateView(state: device.telemetry.state,
                                hopBadge: hopBadge(
                                    isBaseStation: device.isBaseStation,
                                    hopModeOn: device.rocketConfig?.loraHopDisabled == false,
                                    hopChannel: device.telemetry.hop_channel,
                                    state: device.telemetry.state))
                    .opacity(staleOpacity)
            }

            // #557: the FC lost its GNSS module and is flying a baro+IMU-only
            // degraded path — no absolute position/track, guidance off (recovery
            // is unaffected).  Shown on the pad too, so the operator sees it
            // before launching.  Rides sensor_health, so it appears on both the
            // direct-BLE and base-station-relay links.
            if showRocketViews && device.telemetry.gnssAbsentMode {
                GnssAbsentBannerView()
                    .opacity(staleOpacity)
            }

            // #393: gate on the rocket's REPORTED sim state (fs bit 8) OR the
            // local launch latch.  The latch alone dies on BLE reconnect
            // (BLEDevice is recreated), which made the Stop-sim control vanish
            // mid-sim; telemetry keeps the banner up as long as the sim runs.
            if device.simLaunched || device.telemetry.sim_active {
                SimModeBannerView {
                    // #390: stop the sim on the focused rocket only when the
                    // link is a BS relay (broadcast stop would also hit a
                    // neighboring rocket mid-sim).
                    if device.isBaseStation, let rid = device.focusRocketID {
                        device.sendRelayCommand(targetRocketID: rid, innerCommand: 7)
                    } else {
                        device.sendCommand(7)
                    }
                    device.clearSimBanner()
                }
            }

            if device.groundTestActive {
                GroundTestBannerView {
                    device.sendCommand(16)
                    device.groundTestActive = false
                }
            }

            // The whole rocket telemetry stack — see RocketTelemetryCards.
            // Rendered from the same definition the BS-relay branch uses, so
            // the two can no longer drift. Orientation on a BS link comes from
            // the LoRa-relayed flags2 field (#390); the shared view picks that
            // over the direct-link values below.
            RocketTelemetryCards(
                telemetry: device.telemetry,
                isBaseStation: device.isBaseStation,
                bleRSSI: device.connectedRSSI,
                locationManager: locationManager,
                rocketFix: device.lastValidRocketFix,
                trackingHealthy: dataStatus == .live,
                directOrientationName: device.imuOrientationName,
                directOrientationMode: device.imuOrientationMode,
                staleOpacity: staleOpacity,
                showRocketViews: showRocketViews,
                // On SYNCING (searching, no rocket data yet) keep the BS-only
                // battery so the operator still has a live indicator.
                showBSOnlyBattery: device.isBaseStation && dataStatus == .syncing
            )

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

// MARK: - Shared rocket telemetry cards

/// The rocket telemetry card stack, rendered identically on every link type.
///
/// This exists because the two dashboards used to assemble this list by hand
/// and drifted apart. `ConnectedDashboardView` had the full set;
/// `RelayRocketSectionView` — the branch you land on when watching a rocket
/// through a base station — was silently missing `SignalStrengthView` and with
/// it the direction arrow, the GNSS bar AND the LoRa bars, plus `IMUView`,
/// `StatusFlagsView`, `FlightEventFlagsView` and `HealthCardView`. Bench
/// 2026-08-03: three "features" appeared to vanish at once, which is what a
/// missing shared container looks like from the outside.
///
/// One definition, two call sites. A card added here cannot exist on one path
/// and not the other, which is the actual bug being fixed — the missing cards
/// were only the symptom.
///
/// Card ORDER reproduces the previous ConnectedDashboardView exactly, including
/// the quirk that FlightSummaryView sits ABOVE the signal card on a base-station
/// link and BELOW the IMU card on a direct one. That is deliberate: preserving
/// it keeps this refactor invisible on the dashboard that already worked.
struct RocketTelemetryCards: View {
    let telemetry: TelemetryData
    let isBaseStation: Bool
    var bleRSSI: Int? = nil
    var locationManager: LocationManager? = nil
    var rocketFix: LastValidRocketFix? = nil
    var trackingHealthy: Bool = false
    /// Board→rocket mounting orientation for a DIRECT link. On a BS link the
    /// orientation is relayed in flags2 (#390) and read off the telemetry
    /// instead, so callers only supply the direct-link values.
    var directOrientationName: String = ""
    var directOrientationMode: IMUOrientationMode = .unknown
    /// Dimming applied while the stream is stale; 1.0 when live.
    var staleOpacity: Double = 1.0
    /// False while a BS link is still searching and has no rocket data yet —
    /// suppresses the rocket-specific cards without hiding link status.
    var showRocketViews: Bool = true
    /// BS link that is syncing: show the BS-only battery so the operator keeps
    /// a live indicator before any rocket data arrives.
    var showBSOnlyBattery: Bool = false

    var body: some View {
        Group {
            Group {
                if showRocketViews && isBaseStation {
                    FlightSummaryView(telemetry: telemetry)
                        .opacity(staleOpacity)
                }

                SignalStrengthView(
                    telemetry: telemetry,
                    bleRSSI: bleRSSI,
                    isBaseStation: isBaseStation,
                    locationManager: isBaseStation ? locationManager : nil,
                    rocketFix: isBaseStation ? rocketFix : nil,
                    trackingHealthy: trackingHealthy
                )

                if showBSOnlyBattery {
                    BSOnlyBatteryView(telemetry: telemetry)
                } else {
                    // Rocket row + "Base Stn" row on a BS link, same as
                    // pre-#390: the strip-tile bare % phone-tested as
                    // ambiguous, and splitting battery info across two places
                    // wasn't worth a shorter card.
                    BatteryView(telemetry: telemetry, isBaseStation: isBaseStation)
                        .opacity(staleOpacity)
                }

                if showRocketViews {
                    IMUView(telemetry: telemetry,
                            isBaseStation: isBaseStation,
                            orientationName: isBaseStation
                                ? telemetry.relayedOrientationName
                                : directOrientationName,
                            orientationMode: isBaseStation
                                ? telemetry.relayedOrientationMode
                                : directOrientationMode)
                        .opacity(staleOpacity)
                }

                if showRocketViews && !isBaseStation {
                    FlightSummaryView(telemetry: telemetry)
                        .opacity(staleOpacity)
                }
            }

            Group {
                if showRocketViews {
                    GPSView(telemetry: telemetry, compact: true)
                        .opacity(staleOpacity)
                }

                // Numeric RSSI/SNR. The signal card's LoRa bar carries RSSI but
                // NOT snr, and this is the only place snr surfaces at all — it
                // used to be relay-only, so a BS link on the focused path could
                // never show it. LoRa-carried links only; a direct rocket link
                // has no LoRa hop to report.
                if isBaseStation {
                    LoRaSignalView(telemetry: telemetry)
                        .opacity(staleOpacity)
                }

                StatusFlagsView(telemetry: telemetry, isBaseStation: isBaseStation)
                    .opacity(showRocketViews ? staleOpacity : 1.0)

                // Flight-event flag chips — Android → iOS back-port
                // (docs/design-language.md queue, user-requested 2026-07-27).
                // Shown on BS links too: the relay carries the focused rocket's
                // fs bits (#138), and mid-flight these chips are exactly what
                // the operator is watching.
                FlightEventFlagsView(telemetry: telemetry)
                    .opacity(showRocketViews ? staleOpacity : 1.0)

                // Pre-launch sensor health scorecard + go/no-go (#303).  Only
                // shown once the rocket reports a scorecard ("h" present).
                if showRocketViews && telemetry.hasSensorHealth {
                    HealthCardView(telemetry: telemetry)
                        .opacity(staleOpacity)
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
    // Resolved role, not a name-substring guess: renamed devices carry no
    // type hint in the name, and a rocket called "SUBSONIC" contains "BS".
    var connectedDeviceType: BLEDeviceType = .unknown

    private var roleLabel: String {
        switch connectedDeviceType {
        case .rocket: return "Rocket"
        case .baseStation: return "Base Station"
        case .unknown: return "TinkerRocket device"
        }
    }

    var body: some View {
        HStack {
            Circle()
                .fill(isConnected ? Color.green : (isScanning ? Color.orange : Color.gray))
                .frame(width: 12, height: 12)
            if isConnected && !connectedDeviceName.isEmpty {
                VStack(alignment: .leading, spacing: 2) {
                    Text(connectedDeviceName)
                        .font(.headline)
                    Text(roleLabel)
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

// #150: the three honest hop states the tile can report.  Derived from the
// mode readback (lhd), the live-following signal (hch — only present while
// the BS is actually walking the schedule), and the rocket state:
//   armed    = mode on, rocket in READY/INIT — hopping starts at wire-state
//              PRELAUNCH by design, nothing is wrong (this wait is usually
//              GPS).  The label says "pad-ready" rather than "PRELAUNCH":
//              #382 renders wire READY as "PRELAUNCH" on screen too, so
//              naming the wire state here read as a contradiction ("in
//              PRELAUNCH, starts at PRELAUNCH, not hopping") while the
//              rocket was still acquiring GNSS.
//   engaging = mode on, rocket in a hop state, schedule not followed yet —
//              the handoff is in flight (normally 1-3 s; a missed bootstrap
//              self-heals within ~60 s)
//   active   = the BS is following the schedule
enum HopBadge {
    case armed, engaging, active
}

func hopBadge(isBaseStation: Bool, hopModeOn: Bool,
              hopChannel: Int?, state: String) -> HopBadge? {
    guard isBaseStation, hopModeOn else { return nil }
    if hopChannel != nil { return .active }
    switch state {
    case "PRELAUNCH", "INFLIGHT", "LANDED": return .engaging
    default: return .armed
    }
}

struct RocketStateView: View {
    let state: String
    var hopBadge: HopBadge? = nil

    // #382 (display-only): the wire states READY and PRELAUNCH both mean "on
    // the pad" — READY is still waiting on the OC/GNSS gates, PRELAUNCH means
    // the gates are met (GNSS lock, OC link, EKF running: the fully-armed
    // guided path). The raw names read backwards (READY sounds MORE ready
    // than PRELAUNCH), and the old colors — READY green, PRELAUNCH orange —
    // reinforced the inversion. Render ONE "PRELAUNCH" label for both pad
    // states; the banner COLOR carries the distinction (acquiring orange,
    // ready green). The text badge that used to sit here ("EKF Ready" /
    // "Acquiring GNSS") was removed 2026-07-31: "EKF Ready" beside an amber
    // EKF health dot read as the app contradicting itself, and the sensor
    // dots are the authority on live quality. Wire strings, CSV columns,
    // and the announcer's raw-state logic are untouched.
    static func displayLabel(for state: String) -> String {
        state == "READY" ? "PRELAUNCH" : state
    }

    var stateColor: Color {
        switch state {
        case "READY": return .orange     // acquiring (was green — inverted)
        case "PRELAUNCH": return .green  // fully ready (was orange)
        case "INFLIGHT": return .red
        case "COMPLETE": return .blue
        case "MAG_CAL": return .blue   // ground-only excursion (issue #96)
        default: return .gray
        }
    }

    var body: some View {
        VStack(spacing: 4) {
            Text(Self.displayLabel(for: state))
                .font(.system(size: 36, weight: .bold))
                .foregroundColor(stateColor)
            switch hopBadge {
            case .active:
                Label("Frequency Hopping", systemImage: "dot.radiowaves.left.and.right")
                    .font(.caption.weight(.semibold))
                    .foregroundColor(.blue)
            case .engaging:
                Label("Hopping — engaging…", systemImage: "dot.radiowaves.left.and.right")
                    .font(.caption.weight(.semibold))
                    .foregroundColor(.orange)
            case .armed:
                Label("Hopping armed — starts when pad-ready", systemImage: "dot.radiowaves.left.and.right")
                    .font(.caption.weight(.semibold))
                    .foregroundColor(.secondary)
            case nil:
                EmptyView()
            }
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
/// The compact per-subsystem dot row inside HealthCardView's horizontal
/// scroller.  Split out so the render test can draw it directly —
/// ImageRenderer produces empty output for ScrollView children.
struct HealthDotRow: View {
    let rows: [TelemetryData.SensorHealthRow]

    private func color(for state: TelemetryData.SensorHealth) -> Color {
        switch state {
        case .ok:       return .green
        case .degraded: return .orange
        case .bad:      return .red
        case .na:       return .gray
        }
    }

    var body: some View {
        HStack(spacing: 14) {
            ForEach(rows) { row in
                VStack(spacing: 3) {
                    Circle()
                        .fill(color(for: row.state))
                        .frame(width: 12, height: 12)
                    Text(row.name)
                        .font(.caption2)
                }
            }
        }
    }
}

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

            // Compact dot row — Android → iOS back-port (design-language
            // queue, user-requested 2026-07-27): one dot per subsystem,
            // green/amber/red/gray, name under each dot.  Replaced the
            // labeled chip grid — the per-sensor state TEXT moved out of the
            // glanceable path (the go/no-go banner above carries the verdict;
            // a wrong dot's meaning is one tap into MagCal/Settings anyway).
            // The row itself is a separate view because ImageRenderer draws
            // ScrollView content as EMPTY — the render test hits the row
            // directly (same reason the old LazyVGrid was render-untestable).
            ScrollView(.horizontal, showsIndicators: false) {
                HealthDotRow(rows: telemetry.sensorHealthRows)
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

// #557: GNSS-absent degraded-flight warning.  Passive/informational (no control
// — it reflects a hardware condition the operator can't toggle), modeled on
// SimModeBannerView.  Driven by the FC's SH_GNSS_ABSENT verdict in sensor_health.
struct GnssAbsentBannerView: View {
    var body: some View {
        VStack(spacing: 6) {
            HStack(spacing: 8) {
                Image(systemName: "location.slash.fill")
                    .font(.title2)
                Text("NO GNSS — DEGRADED FLIGHT")
                    .font(.system(size: 20, weight: .bold, design: .monospaced))
            }
            .foregroundColor(.orange)
            Text("Flying on baro + IMU only: no position or ground track, "
                 + "landing prediction and guidance are off. Apogee, deployment "
                 + "and landing detection are unaffected.")
                .font(.caption)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
        }
        .padding()
        .frame(maxWidth: .infinity)
        .background(Color.orange.opacity(0.12))
        .cornerRadius(10)
        .overlay(
            RoundedRectangle(cornerRadius: 10)
                .stroke(Color.orange.opacity(0.5), lineWidth: 1)
        )
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
                     total: s.totalBytes,
                     autoEvicted: s.autoEvicted)
            }
        }
    }

    private func card(title: String, subtitle: String,
                      used: Int, reserved: Int, free: Int, total: Int,
                      autoEvicted: Bool = false) -> some View {
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
            // #315: rolling-buffer note. When the card fills, the OC auto-deletes
            // the oldest flight(s) at arm time to make room — surface it so the
            // operator knows data rolled off rather than being silently dropped.
            if autoEvicted {
                HStack(spacing: 5) {
                    Image(systemName: "arrow.triangle.2.circlepath")
                    Text("Auto-reclaimed oldest flight(s) this session")
                }
                .font(.caption2)
                .foregroundColor(.secondary)
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

/// Flight-event flag chips: LAUNCH · BURNOUT · APOGEE · LANDED,
/// illuminating green as each event latches in the fs bitfield.  Born on
/// Android 2026-07-27; back-ported per docs/design-language.md.  The lit
/// green matches Android's (Material 800) so the two dashboards read alike.
/// No LOG chip on iOS: the Status card directly above already carries the
/// Logging badge + active-file line, so it would be pure duplication.
/// Android keeps its LOG chip until the camera/logging status section
/// ports over — it is Android's only logging indicator today (ledger).
struct FlightEventFlagsView: View {
    let telemetry: TelemetryData

    private static let litGreen = Color(red: 46 / 255, green: 125 / 255, blue: 50 / 255)

    private var chips: [(label: String, on: Bool)] {
        [("LAUNCH", telemetry.launch_flag),
         ("BURNOUT", telemetry.burnout_flag),
         ("APOGEE", telemetry.alt_apo || telemetry.vel_apo),
         ("LANDED", telemetry.landed_flag)]
    }

    var body: some View {
        HStack(spacing: 6) {
            ForEach(chips, id: \.label) { chip in
                Text(chip.label)
                    .font(.caption.weight(.semibold))
                    .foregroundColor(chip.on ? .white : .secondary)
                    .padding(.horizontal, 8)
                    .padding(.vertical, 5)
                    .background(chip.on ? Self.litGreen : Color.gray.opacity(0.2))
                    .cornerRadius(6)
            }
            Spacer(minLength: 0)
        }
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
                // Pre-#390 layout, restored by phone-test feedback: the BS
                // log badge + countdown live HERE with the other recording
                // state, not only on the detail screen — during a flight
                // the operator watches this card, and an imminent BS log
                // close is exactly what they need to catch.
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
                        // Resolved type: registry hint for renamed devices,
                        // name prefix for factory defaults, honest question
                        // mark when neither knows.
                        DeviceTypeIcon(type: device.deviceType, symbolFont: .title2)
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

            // Virtual Rocket (2026-07-30): the no-hardware world, matching
            // Android's secondary-row placement in the design language.  A
            // scripted fleet through the real stack — READY idle, one flight
            // per Simulation-sheet launch.  Names the THING (a virtual
            // rocket); "Simulate" flies the real one.
            Button {
                fleet.startVirtualRocket()
            } label: {
                HStack {
                    Image(systemName: "sparkles")
                        .frame(width: 36)
                    Text(fleet.virtualRocketActive ? "Virtual Rocket connected"
                                                   : "Virtual Rocket")
                        .font(.subheadline)
                    Spacer()
                    if fleet.virtualRocketActive {
                        Image(systemName: "checkmark.circle.fill")
                            .foregroundColor(.green)
                    }
                }
                .padding(.vertical, 8)
                .padding(.horizontal, 12)
                .background(Color(.systemGray5).opacity(0.6))
                .cornerRadius(10)
            }
            .buttonStyle(PlainButtonStyle())
            .disabled(fleet.virtualRocketActive)
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
                // #390: on a BS link, target the focused rocket explicitly
                // (cmd 50 relay) with the desired state computed from ITS
                // telemetry — the legacy BS camera toggle broadcast to every
                // rocket in range and keyed its on/off decision off whichever
                // rocket was heard last.
                if device.isBaseStation, let rid = device.focusRocketID {
                    let desired: UInt8 = device.telemetry.camera_recording ? 0 : 1
                    device.sendRelayCommand(targetRocketID: rid,
                                            innerCommand: 1,
                                            innerPayload: Data([desired]))
                } else {
                    device.sendCommand(1)
                }
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

            // Rocket flash recording and the BS CSV are separate controls
            // (phone-tested): the rocket starts NOT logging while the BS
            // auto-starts on first contact, so the old coupled cmd-23
            // toggle was always fighting one side's state.
            Button(action: {
                if device.isBaseStation, let rid = device.focusRocketID {
                    // Same targeted-relay pattern as the camera button.
                    let desired: UInt8 = device.telemetry.rocketLoggingActive ? 0 : 1
                    device.sendRelayCommand(targetRocketID: rid,
                                            innerCommand: 23,
                                            innerPayload: Data([desired]))
                } else {
                    device.sendCommand(23)   // direct link: OC toggle
                }
            }) {
                let active = device.telemetry.rocketLoggingActive
                HStack {
                    Image(systemName: active ? "stop.circle.fill" : "record.circle")
                    Text(device.isBaseStation
                         ? (active ? "Stop Rocket Log" : "Start Rocket Log")
                         : (active ? "Stop Logging" : "Start Logging"))
                }
                .frame(maxWidth: .infinity)
                .padding()
                .background(active ? Color.red : Color.orange)
                .foregroundColor(.white)
                .cornerRadius(10)
            }
            .disabled(device.isBaseStation && device.focusRocketID == nil)

            // BS-only CSV logging (cmd 46) — never uplinks to the rocket.
            if device.isBaseStation {
                Button(action: {
                    device.sendSetBSLogging(!device.telemetry.bs_logging_active)
                }) {
                    let active = device.telemetry.bs_logging_active
                    HStack {
                        Image(systemName: active ? "stop.circle.fill" : "record.circle")
                        Text(active ? "Stop Base Stn Log" : "Start Base Stn Log")
                    }
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(active ? Color.red : Color.orange)
                    .foregroundColor(.white)
                    .cornerRadius(10)
                }
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
                        activeSheet = .simulator(device)
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

                // Ground Test button — direct links only (#390): the BS has
                // no dispatch for cmds 15/16, so on a relay link this was a
                // dead button that silently dropped the tap.
                if !device.isBaseStation {
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
                }

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
                    activeSheet = .servoTest(device)
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

                // #150: Frequency Scan, back on this card (the #390 detail
                // screen it briefly moved to was cut — the strip is a
                // passive label now).  BS-only: the scan runs on the base
                // station's radio; in hopping mode the firmware coordinates
                // a hop pause and pushes the channel mask to the rocket.
                if device.isBaseStation {
                    Button {
                        activeSheet = .frequencyScan(device)
                    } label: {
                        HStack {
                            Image(systemName: "waveform.badge.magnifyingglass")
                            Text("Frequency Scan")
                        }
                        .font(.system(.body, weight: .semibold))
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .foregroundColor(.white)
                        .background(device.isConnected ? Color.purple : Color.purple.opacity(0.4))
                        .cornerRadius(10)
                    }
                    .disabled(!device.isConnected)
                }
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
        // Sim pyros DRY-FIRE as of the 2026-07-30 firmware: channels
        // sequence and report, the squib rail never energizes.  The old
        // destructive "charges will fire" warning was the only protection
        // before the firmware gate existed.
        .alert("Simulated flight", isPresented: $showSimWarning) {
            Button("Launch") {
                launchSim()
            }
            Button("Cancel", role: .cancel) { }
        } message: {
            Text("The rocket flies the full pipeline on synthetic sensors. Pyro channels sequence and report but are dry-fired — no charges are energized (requires current FC firmware).")
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
        // #390: on a BS link, target the focused rocket (cmd 50) instead of
        // the legacy broadcast — a second powered rocket must not launch a
        // sim because its neighbor did.
        if device.isBaseStation, let rid = device.focusRocketID {
            device.sendRelayCommand(targetRocketID: rid, innerCommand: 5,
                                    innerPayload: payload)
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) {
                device.markSimLaunched()
                device.sendRelayCommand(targetRocketID: rid, innerCommand: 6)
            }
        } else {
            device.sendRawCommand(5, payload: payload)
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) {
                device.markSimLaunched()
                device.sendCommand(6)
            }
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
    /// #390: true while the BS is live-tracking its focused rocket — the
    /// netid-drops line renders as info instead of a warning then.
    var trackingHealthy: Bool = false
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

            // #150: network-id mismatch drops — the failure that used to be
            // a silent "Searching for rocket…".  Only rendered once the BS
            // reports a non-zero count.
            // #390: while the BS is successfully tracking its rocket, other-
            // network packets are expected traffic (a second BS/rocket pair
            // on its own network id at the same site), not a fault — render
            // as info. The loud warning stays for the diagnostic case the
            // counter exists for: drops while hearing NO rocket.
            if isBaseStation, let drops = telemetry.netid_drops, drops > 0 {
                if trackingHealthy {
                    Label("Hearing \(drops) packets from other networks (another pair nearby is normal)",
                          systemImage: "info.circle")
                        .font(.caption)
                        .foregroundColor(.secondary)
                } else {
                    Label("NetID mismatch: \(drops) packets dropped — a device is on the wrong network ID (see Settings ▸ Network)",
                          systemImage: "exclamationmark.triangle.fill")
                        .font(.caption)
                        .foregroundColor(.orange)
                }
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
        // #150: while hopping, show the live channel index next to the
        // signal so the operator can see both ends walking the schedule.
        if let hch = telemetry.hop_channel {
            return String(format: "%.0f ch%d", rssi, hch)
        }
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
