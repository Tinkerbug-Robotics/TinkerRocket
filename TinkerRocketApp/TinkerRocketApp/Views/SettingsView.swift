//
//  SettingsView.swift
//  TinkerRocketApp
//
//  Per-rocket settings, edited against the ACTIVE rocket profile (issue
//  #132).  The app is source-of-truth: edits are written to the profile and,
//  when a rocket is connected, also pushed live so there's no
//  disconnect/reconnect needed to apply.  On the next connect the
//  ActiveRocketSyncer re-pushes the whole profile, so the rocket always
//  matches the profile regardless of what was in its NVS.
//
//  LoRa frequency / TX power stay device/BS-side (not part of a profile) and
//  are shown here read-only / BS-only as before.
//

import SwiftUI
import Combine

struct SettingsView: View {
    @ObservedObject var device: BLEDevice
    @EnvironmentObject var fleet: BLEFleet   // injected by DashboardView for FirmwareUpdateView → OTASession lookup
    @EnvironmentObject var store: RocketProfileStore
    @Environment(\.dismiss) var dismiss

    // App-wide display-unit preference (#160).  The picker lives on the
    // dashboard (front screen); this is read here only to show/edit the pyro
    // "altitude on descent" field in the chosen unit.  Display-layer only —
    // logs, exports, and the wire protocol stay SI.
    @AppStorage("unitSystem") private var unitSystem: UnitSystem = .metric

    // Editable string fields — parsed on focus loss, not per keystroke.
    @State private var sBias1 = ""
    @State private var sBias2 = ""
    @State private var sBias3 = ""
    @State private var sBias4 = ""
    @State private var sServoHz = ""
    @State private var sServoMinUs = ""
    @State private var sServoMaxUs = ""
    @State private var sFinTravelDeg = ""
    @State private var sPidKp = ""
    @State private var sPidKi = ""
    @State private var sPidKd = ""
    @State private var sPidMinCmd = ""
    @State private var sPidMaxCmd = ""
    @State private var sRollDelayMs = ""
    @State private var savedRollUsesAngle: Bool? = nil
    @State private var savedOrientCode: UInt8? = nil
    @State private var sRateCapDps = ""
    @State private var sKpAngle = ""
    @State private var sIntegralSep = ""
    @State private var sPnTargetAlt = ""
    @State private var sPnNavGain = ""
    @State private var sPnMaxAccel = ""
    @State private var sPnAccelToFin = ""
    @State private var sPnMaxFin = ""
    @State private var sPnMinSpeed = ""
    @State private var sPnKpPos = ""
    @State private var sPnKdVel = ""

    // Roll waypoints edited as strings; committed to the profile on change.
    @State private var rollWaypoints: [(time: String, angle: String)] = []

    // Pyro trigger values edited as strings (seconds or meters by mode).
    @State private var sPyroValue: [String] = ["", "", "", ""]

    // Recovery descent profile edited as strings (#191).  App-only fields:
    // consumed by the live landing predictor and Drift Cast, never sent to
    // the flight computer.
    @State private var sDrogueRate = ""
    @State private var sMainRate = ""
    @State private var sMainDeployAlt = ""
    @State private var sDragK = ""

    // Drives the full-screen pyro test-fire cover.  (The per-channel continuity
    // readout state lives in the PyroChannelTestControls child view.)
    @State private var pyroTestChannel: Int?

    // "Sent" feedback in section headers.
    @State private var servoApplied = false
    @State private var pidApplied = false
    @State private var rollControlApplied = false
    @State private var guidanceApplied = false
    @State private var finApplied = false
    @State private var pyroApplied = false

    // LoRa TX power (BS only) — hydrated from rocketConfig, debounced send.
    @State private var txPowerDbm: Int = 12
    @State private var txPowerSendWork: DispatchWorkItem?

    // #150: link mode (BS only) — device state, NOT a profile field.  The
    // picker pushes cmd 17 on change; hydration from the lhd readback
    // snaps it back if the firmware refuses (e.g. lhdw == 0).
    @State private var linkModeHopping = false
    // A hop DISABLE commits ~1.5 s after the tap (the BS drains its uplink
    // retries on the hop channel first) — readbacks arriving inside that
    // window still say "hopping" and would flip the picker back against
    // the user's tap.  Hold hydration briefly after a tap; the commit
    // readback lands after the hold and settles the true state.
    @State private var linkModeTouchedAt: Date?
    private static let linkModeHydrationHoldS: TimeInterval = 3.0

    @FocusState private var focusedField: EditField?
    @State private var lastFocusedField: EditField?

    // Settings are grouped into tabs (one panel at a time) on phone screens
    // (#132 / #147).  Switching a tab flushes the leaving tab's pending edit.
    // General is first + the default landing tab — it's where active rocket
    // info and calibration live, which most users want on entry.
    @State private var tab: SettingsTab = .general
    private enum SettingsTab: String, CaseIterable {
        case general = "General"
        case control = "Control"
        case camera = "Camera"
        case pyro = "Pyro"
    }

    private var currentFreqMHz: Float { device.rocketConfig?.loraFreqMHz ?? 915.0 }

    private var isInitializing: Bool {
        device.isConnected && !device.isBaseStation
            && device.telemetry.state == "INITIALIZATION"
    }

    /// The power rail is off, so the FC — the thing that actually answers
    /// config writes — isn't running.  This fails quietly rather than loudly:
    /// the OC's #366 command queue HOLDS commands while the rail is down and
    /// drains the batch at power-on, so a user can edit several screens of
    /// settings and have them land much later, out of order, or not at all.
    /// Block the sheet exactly as INITIALIZATION does, for the same reason —
    /// the rocket can't honour the write yet.
    ///
    /// Two scoping guards, both load-bearing:
    ///  - `!isBaseStation`: the BS relay's TelemetryData never carries
    ///    pwr_pin_on (the LoRa packet has no room for it), so over a
    ///    base-station link it reads false forever — an unscoped gate would
    ///    lock Settings permanently on every BS connection.
    ///  - `hasReceivedTelemetry` (#377): before the first frame `telemetry` is
    ///    all zeros, and a zeroed pwr_pin_on is indistinguishable from a
    ///    genuinely-off rocket.  Without this the overlay would flash on every
    ///    connect, including for a rocket that is already powered on.
    private var isRocketOff: Bool {
        device.isConnected && !device.isBaseStation
            && device.hasReceivedTelemetry
            && !device.telemetry.pwr_pin_on
    }

    /// Calibrations run on the rocket, so they need it connected AND powered
    /// on (its sensors aren't running otherwise).  Other settings stay
    /// editable offline — only the cal rows are gated.
    private var canCalibrate: Bool {
        device.isConnected && !device.isBaseStation && device.telemetry.pwr_pin_on
    }

    /// Convenience: the active profile, or a throwaway default so getters have
    /// something to read before a profile is selected.  Writes always go
    /// through `updateProfile`, which no-ops when there's no active id.
    private var profile: RocketProfile {
        store.activeProfile ?? RocketProfile.makeDefault(name: "")
    }

    /// Editing a rocket shows its name as the title; otherwise generic.
    private var navTitle: String {
        if !device.isBaseStation, let active = store.activeProfile { return active.name }
        return "Settings"
    }

    // MARK: - Focus-driven self-apply (#144)

    private enum EditField: Hashable {
        case bias1, bias2, bias3, bias4, servoHz, servoMin, servoMax, finTravel
        case pidKp, pidKi, pidKd, pidMin, pidMax
        case rollDelay
        case rateCap
        case kpAngle
        case integralSep
        case guidTargetAlt, guidNavGain, guidMaxAccel, guidAccelToFin, guidMaxFin, guidMinSpeed
        case guidKpPos, guidKdVel
        case wpTime(Int), wpAngle(Int)
        case pyroValue(Int)   // ch index 0..3
        case drogueRate, mainRate, mainDeployAlt, dragK
    }

    /// Fin-angle range implied by the pulse + travel fields as they are being
    /// edited. The endpoint angles are derived, never typed (#449); showing them
    /// live is what makes narrowing the pulse range visibly narrow the fins —
    /// the mismatch this used to silently allow flew as a 2× calibration error.
    @ViewBuilder private var finCalibrationReadout: some View {
        let mn = parseDouble(sServoMinUs, fallback: Double(profile.servoMinUs))
        let mx = parseDouble(sServoMaxUs, fallback: Double(profile.servoMaxUs))
        let travel = parseDouble(sFinTravelDeg, fallback: Double(profile.finTravelDeg))
        let span = mx - mn
        HStack {
            Text("Fin Range")
            Spacer()
            if span <= 0 {
                Text("Max Pulse must exceed Min Pulse")
                    .foregroundColor(.red)
            } else {
                Text(String(format: "%+.1f\u{00B0} \u{2026} %+.1f\u{00B0}   (%.3f\u{00B0}/\u{00B5}s)",
                            -travel / 2.0, travel / 2.0, travel / span))
                    .foregroundColor(.secondary)
                    .monospacedDigit()
            }
        }
        .font(.subheadline)
    }

    private enum EditGroup { case servo, pid, rollControl, guidance, rollWaypoints, pyro, recovery }

    private func group(of field: EditField?) -> EditGroup? {
        switch field {
        case .bias1, .bias2, .bias3, .bias4, .servoHz, .servoMin, .servoMax, .finTravel: return .servo
        case .pidKp, .pidKi, .pidKd, .pidMin, .pidMax: return .pid
        case .rollDelay, .rateCap, .kpAngle, .integralSep: return .rollControl
        case .guidTargetAlt, .guidNavGain, .guidMaxAccel, .guidAccelToFin, .guidMaxFin, .guidMinSpeed,
             .guidKpPos, .guidKdVel: return .guidance
        case .wpTime, .wpAngle: return .rollWaypoints
        case .pyroValue: return .pyro
        case .drogueRate, .mainRate, .mainDeployAlt, .dragK: return .recovery
        case nil: return nil
        }
    }

    private func applyGroup(_ g: EditGroup) {
        switch g {
        case .servo: applyServoConfig()
        case .pid: applyPIDConfig()
        case .rollControl: applyRollControlConfig()
        case .guidance: applyGuidanceConfig()
        case .rollWaypoints: applyRollProfile()
        case .pyro: applyPyroConfig()
        case .recovery: applyRecoveryConfig()
        }
    }

    private func handleFocusChange(_ newField: EditField?) {
        let previous = lastFocusedField
        lastFocusedField = newField
        if let leftGroup = group(of: previous), leftGroup != group(of: newField) {
            applyGroup(leftGroup)
        }
    }

    private func flushPendingEdits() {
        if let g = group(of: lastFocusedField) { applyGroup(g) }
        lastFocusedField = nil
    }

    var body: some View {
        NavigationView {
            Form {
                // Firmware update is device-level (not profile-level). BS and
                // no-profile devices have no tab bar, so they each carry a
                // single firmwareSection here. For rocket profiles it lives
                // inside generalSections so it shows on the General tab only,
                // not duplicated across every rocket tab (#314).
                if device.isBaseStation {
                    baseStationSections
                    firmwareSection
                } else if store.activeProfile == nil {
                    noProfileSection
                    firmwareSection
                } else if isRocketOff {
                    // Swap the config sections out rather than covering them with
                    // an INITIALIZATION-style overlay: firmwareSection lives
                    // inside rocketSettingsSections (#314), and a full-sheet
                    // overlay would take Firmware Update down with it. The OC
                    // flashes ITSELF over BLE and never touches the FC rail, so
                    // that one genuinely works with the rocket off — it's the
                    // only thing on this screen that does.
                    rocketOffSection
                    firmwareSection
                } else {
                    rocketSettingsSections
                }
            }
            .navigationTitle(navTitle)
            .overlay { if isInitializing { initializingOverlay } }
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") { dismiss() }
                }
                ToolbarItemGroup(placement: .keyboard) {
                    Spacer()
                    Button("Done") {
                        UIApplication.shared.sendAction(
                            #selector(UIResponder.resignFirstResponder),
                            to: nil, from: nil, for: nil)
                    }
                }
            }
            .onAppear { loadFromProfile() }
            // Test Pyro Channel (from the Pyro tab) presents the same full-screen
            // test-fire flow the dashboard uses.
            .fullScreenCover(item: Binding<IdentifiableInt?>(
                get: { pyroTestChannel.map { IdentifiableInt(value: $0) } },
                set: { pyroTestChannel = $0?.value }
            )) { item in
                PyroTestView(device: device, channel: item.value)
            }
            .onChange(of: store.activeId) { _ in loadFromProfile() }
            // Leaving a tab commits its pending text edit; reloading then snaps
            // any unparseable input back to the last saved (valid) value.
            .onChange(of: tab) { _ in
                flushPendingEdits()
                loadFromProfile()
            }
            .onChange(of: focusedField) { handleFocusChange($0) }
            // Flipping the unit picker reformats the altitude pyro field from
            // the canonical (metres) profile value into the new unit.  Reload
            // (not flush) so a mid-edit value isn't re-parsed in the wrong unit.
            .onChange(of: unitSystem) { _ in reloadPyroValueStrings() }
            .onDisappear { flushPendingEdits() }
            .onReceive(device.$rocketConfig.compactMap { $0 }) { cfg in
                // Only device-owned LoRa state is hydrated from readback —
                // every other setting is owned by the profile (app is source
                // of truth) so we no longer pull config back into local state.
                if let pwr = cfg.loraTxPower { txPowerDbm = Int(pwr) }
                // #150: skip lhd hydration inside the post-tap hold window
                // (see linkModeTouchedAt) so mid-drain readbacks can't flip
                // the picker against the user's tap.
                if let hd = cfg.loraHopDisabled,
                   linkModeTouchedAt.map({ Date().timeIntervalSince($0) > Self.linkModeHydrationHoldS }) ?? true {
                    linkModeHopping = !hd
                }
            }
        }
    }

    // MARK: - Base station (read-only display)

    @ViewBuilder
    private var baseStationSections: some View {
        Section(header: Text("Active Rocket")) {
            HStack {
                Image("RocketIcon")
                    .resizable().renderingMode(.template)
                    .aspectRatio(contentMode: .fit)
                    .frame(width: 20, height: 20)
                    .foregroundColor(.accentColor)
                Text(store.activeProfile?.name ?? "None selected")
                    .fontWeight(.semibold)
            }
            Text("Settings are stored per rocket in the app. To change them, connect directly to the rocket computer over Bluetooth.")
                .font(.caption)
                .foregroundColor(.secondary)
        }

        if let p = store.activeProfile {
            Section(header: Text("Summary")) {
                summaryRow("Control mode", p.controlModeLabel)
                summaryRow("Camera", cameraLabel(p.cameraType))
                summaryRow("IMU mounting", p.imuOrientSetting == 0xFF
                    ? "Auto" : "Nose \(FlightSettingsData.b2rName(code: p.imuOrientSetting))")
                summaryRow("Gain scheduling", p.gainScheduleEnabled ? "On" : "Off")
                summaryRow("Mag cal", p.magCal == nil ? "Not saved" : "Saved")
                summaryRow("Sensor cal", p.sensorCal == nil ? "Not saved" : "Saved")
            }
        }

        loRaSections
        networkSection
    }

    // #150: Network (restored from #136, with the drift bug fixed).  The
    // app stores the human-friendly name; the DEVICE's readback nid
    // (config_identity) is the truth about what's on the air.  The old UI
    // displayed only the app-local @AppStorage copy, which is exactly what
    // made the #133 nid drift undebuggable — hence the mismatch badge and
    // one-tap sync against the readback.
    @ViewBuilder
    private var networkSection: some View {
        Section(header: Text("Network"), footer: Text(networkFooter)) {
            HStack {
                Text(appNetworkName.isEmpty ? "Not set" : appNetworkName)
                Spacer()
                Text("ID: \(appNetworkID)")
                    .foregroundColor(.secondary)
                    .font(.system(.body, design: .monospaced))
            }
            HStack {
                Text("Device reports")
                Spacer()
                if deviceNidMismatch {
                    Image(systemName: "exclamationmark.triangle.fill")
                        .foregroundColor(.orange)
                }
                Text("ID: \(device.networkID)")
                    .foregroundColor(deviceNidMismatch ? .orange : .secondary)
                    .font(.system(.body, design: .monospaced))
            }
            if deviceNidMismatch {
                Button("Set device to \"\(appNetworkName)\" (ID \(appNetworkID))") {
                    // Route through the known-device registry (the single
                    // writer for identity edits) so the My Devices record
                    // updates immediately, not only via the readback echo.
                    if let store = device.fleet?.knownDevices {
                        store.setNetworkID(UInt8(clamping: appNetworkID),
                                           for: device.unitID, pusher: device)
                    } else {
                        device.sendSetNetworkID(UInt8(clamping: appNetworkID))
                    }
                }
                .disabled(!device.isConnected)
            }
        }
    }

    // Firmware update entry. Shown for any connected device (#8): Phase 2
    // shipped it BS-only; Phase 3 added the Out Computer firmware side, so the
    // entry is no longer gated on isBaseStation. (The connected device is
    // always a BS or OC — the FC has no BLE radio. Phase 4 will add an
    // FC-relay target picker when connected to an OC.)
    @ViewBuilder
    private var firmwareSection: some View {
        Section(header: Text("Firmware")) {
            HStack {
                Text("Version")
                Spacer()
                Text(device.firmwareVersion.isEmpty ? "(pre-#8)" : device.firmwareVersion)
                    .font(.body.monospaced())
                    .foregroundColor(.secondary)
                    .lineLimit(1)
                    .truncationMode(.middle)
            }
            NavigationLink {
                FirmwareUpdateView(device: device)
            } label: {
                Label("Update firmware…", systemImage: "arrow.up.circle")
            }
            .disabled(!device.isConnected)
        }
    }

    @ViewBuilder
    private var loRaSections: some View {
        // #150: Fixed vs Hopping link-mode picker.  Applies on change (no
        // Apply button, #144); non-focusable control keeps this section
        // safe next to live readback (#361).
        Section(header: Text("Link Mode"), footer: Text(linkModeFooter)) {
            Picker("Link Mode", selection: linkModeBinding) {
                Text("Fixed Channel").tag(false)
                Text("Frequency Hopping").tag(true)
            }
            .pickerStyle(.segmented)
            .disabled(device.autoApplyRefusalReason() != nil ||
                      (hopUnavailable && !linkModeHopping))
        }

        Section(header: Text("LoRa Frequency"),
                footer: Text("Both devices meet on the factory channel at boot. Both should report the same frequency.")) {
            HStack {
                Text("Current")
                Spacer()
                Text(String(format: "%.2f MHz", currentFreqMHz))
                    .foregroundColor(.secondary)
                    .font(.system(.body, design: .monospaced))
            }
        }

        Section(header: Text("LoRa TX Power"), footer: txPowerFooter) {
            Stepper(value: $txPowerDbm, in: -9...22) {
                HStack {
                    Text("Power")
                    Spacer()
                    Text("\(txPowerDbm) dBm")
                        .foregroundColor(.secondary)
                        .font(.system(.body, design: .monospaced))
                }
            }
            // #150: also locked while hopping — TX power rides cmd 10,
            // which the firmware refuses mid-hop (a change would silently
            // snap back).  Switch to Fixed first.
            .disabled(device.autoApplyRefusalReason() != nil || linkModeHopping)
            .onChange(of: txPowerDbm) { scheduleTxPowerSend($0) }
        }
    }

    // MARK: - No active profile

    private var noProfileSection: some View {
        Section {
            VStack(alignment: .leading, spacing: 8) {
                Text("No rocket selected")
                    .font(.headline)
                Text("Pick or create a rocket from the rocket menu to edit its settings.")
                    .font(.subheadline)
                    .foregroundColor(.secondary)
                NavigationLink {
                    RocketProfileView(device: device)
                } label: {
                    Label("Choose Rocket", systemImage: "list.bullet")
                }
            }
            .padding(.vertical, 4)
        }
    }

    // MARK: - Rocket settings (profile-backed)

    @ViewBuilder
    private var rocketSettingsSections: some View {
        Section {
            Picker("Group", selection: $tab) {
                ForEach(SettingsTab.allCases, id: \.self) { Text($0.rawValue).tag($0) }
            }
            .pickerStyle(.segmented)
        }

        switch tab {
        case .control:  controlSections
        case .camera:   cameraSections
        case .pyro:     pyroSections
        case .general:
            generalSections
            // #150: the network surface must exist on ROCKETS too — the
            // cmd-41 sync only reaches the device the app is connected
            // to, so a drifted rocket nid would otherwise be visible only
            // as the BS's rising drop counter, with no in-app fix.
            networkSection
        }
    }

    @ViewBuilder
    private var controlSections: some View {
        Section(header: configHeader("PID Gains", applied: pidApplied)) {
            stringRow("Kp", text: $sPidKp, field: .pidKp, decimal: true)
            stringRow("Ki", text: $sPidKi, field: .pidKi, decimal: true)
            stringRow("Kd", text: $sPidKd, field: .pidKd, decimal: true)
            Text("Roll-rate-null PID, in fin-degrees per deg/s of roll-rate error. Kp sets roll authority, Ki slowly cancels a steady fin-misalignment spin, Kd is normally 0.")
                .font(.caption).foregroundColor(.secondary)
            stringRow("Min Deflection", text: $sPidMinCmd, field: .pidMin, unit: "deg")
            stringRow("Max Deflection", text: $sPidMaxCmd, field: .pidMax, unit: "deg")
            Text("Hard clamp on commanded fin deflection (\u{00B1} max deflection). Stay inside the servo's mechanical fin-angle range set in the Servo section below.")
                .font(.caption).foregroundColor(.secondary)
            Toggle("Velocity Gain Scheduling", isOn: bind(\.gainScheduleEnabled) {
                device.sendGainScheduleConfig(enabled: $0)
            })
            Text("Scales the gains by (V_ref/V)\u{00B2} so fin authority stays roughly constant as speed changes. Disable for fixed gains at all speeds.")
                .font(.caption).foregroundColor(.secondary)
        }

        Section(header: configHeader("Servo", applied: servoApplied)) {
            Toggle("Enable Servo Control", isOn: bind(\.servoControlEnabled) {
                device.sendServoControlConfig(enabled: $0)
            })
            stringRow("Servo 1", text: $sBias1, field: .bias1)
            stringRow("Servo 2", text: $sBias2, field: .bias2)
            stringRow("Servo 3", text: $sBias3, field: .bias3)
            stringRow("Servo 4", text: $sBias4, field: .bias4)
            Text("Per-servo pulse offset (\u{00B5}s) to trim mechanical misalignment so a zero command holds the fin straight.")
                .font(.caption).foregroundColor(.secondary)
            stringRow("Frequency", text: $sServoHz, field: .servoHz, unit: "Hz")
            stringRow("Min Pulse", text: $sServoMinUs, field: .servoMin, unit: "\u{00B5}s")
            stringRow("Max Pulse", text: $sServoMaxUs, field: .servoMax, unit: "\u{00B5}s")
            stringRow("Fin Travel", text: $sFinTravelDeg, field: .finTravel, unit: "deg")
            finCalibrationReadout
            Text("Servo datasheet values: the pulse endpoints and the TOTAL fin deflection swept between them (servo travel \u{00D7} linkage ratio; 1:1 for a fin bolted straight to the arm). The controller maps a commanded fin angle onto the pulse that produces it, so these three numbers set the real authority \u{2014} narrowing the pulse range narrows the fins.")
                .font(.caption).foregroundColor(.secondary)
        }

        controlModeSection
        if profile.guidanceEnabled {
            guidanceSection
        } else {
            rollControlSection
        }
        finLayoutSection
    }

    @ViewBuilder
    private var cameraSections: some View {
        Section("Camera") {
            Picker("Camera Type", selection: cameraBinding) {
                Text("None").tag(0)
                Text("GoPro").tag(1)
                Text("RunCam").tag(2)
            }
            .pickerStyle(.segmented)
            Text(cameraHint(Int(profile.cameraType)))
                .font(.caption).foregroundColor(.secondary)
        }
    }

    @ViewBuilder
    private var pyroSections: some View {
        pyroChannelSection(1)
        pyroChannelSection(2)
        pyroChannelSection(3)
        pyroChannelSection(4)
        Section {
            Text("Single shared arm FET arms momentarily for each fire pulse. Test continuity and test-fire from the rocket dashboard before flight.")
                .font(.caption).foregroundColor(.secondary)
        }
        recoverySection
    }

    // Recovery descent profile (#191).  These fields live only in the app
    // profile (landing-prediction + drift-cast inputs) and are never sent to
    // the rocket — hence no "Sent" badge and no BLE push on apply.  The
    // static header also keeps this section free of telemetry-driven
    // re-renders next to a focused TextField (#361).
    @ViewBuilder
    private var recoverySection: some View {
        Section(header: Text("Recovery")) {
            stringRow("Drogue Rate", text: $sDrogueRate, field: .drogueRate, unit: "fps", decimal: true)
            stringRow("Main Rate", text: $sMainRate, field: .mainRate, unit: "fps", decimal: true)
            stringRow("Main Deploy", text: $sMainDeployAlt, field: .mainDeployAlt, unit: "ft", decimal: true)
            Text("Descent profile for the live landing prediction and Drift Cast: drogue rate above the main-deploy altitude (AGL), main rate below it. Stored per rocket in the app \u{2014} not sent to the flight computer.")
                .font(.caption).foregroundColor(.secondary)
            stringRow("Drag k", text: $sDragK, field: .dragK, unit: "1/m", decimal: true)
            Text("Quadratic drag coefficient for the coast-to-apogee ballistic prediction (a = \u{2212}k\u{00B7}|v|\u{00B7}v). \u{2248}0.0005 for typical 54\u{2013}65 mm airframes; 0 = gravity-only.")
                .font(.caption).foregroundColor(.secondary)
        }
    }

    private func pyroChannelSection(_ ch: Int) -> some View {
        let enabled = pyroChannelEnabled(ch)
        let mode = Int(pyroChannelTriggerMode(ch))
        let idx = ch - 1
        return Section(header: configHeader("Pyro Channel \(ch)", applied: pyroApplied)) {
            Toggle("Enabled", isOn: pyroEnabledBinding(ch))
            if enabled {
                Picker("Trigger", selection: pyroModeBinding(ch)) {
                    Text("Time after apogee").tag(0)
                    Text("Altitude on descent").tag(1)
                }
                .pickerStyle(.segmented)
                HStack {
                    Text(mode == 0 ? "Delay" : "Altitude")
                    Spacer()
                    TextField(mode == 0 ? "0.0" : "0", text: $sPyroValue[idx])
                        .keyboardType(.decimalPad)
                        .multilineTextAlignment(.trailing)
                        .frame(width: 80)
                        .focused($focusedField, equals: .pyroValue(idx))
                    Text(mode == 0 ? "s after apogee" : "\(UnitFormatter.altitudeUnit(unitSystem)) on descent")
                        .foregroundColor(.secondary)
                }
                Text(mode == 0
                    ? "Fires this many seconds after apogee is detected."
                    : "Fires when the rocket descends through this altitude (AGL).")
                    .font(.caption).foregroundColor(.secondary)
            }
            // Telemetry-driven controls live in a child view so this Section's
            // child list stays structurally stable and telemetry-independent.
            // Inlining them read device.telemetry in SettingsView's body, so the
            // continuity dot and buttons appeared/disappeared as siblings of the
            // focused Delay TextField on every live frame — which disrupted
            // keyboard layout on first focus (negative TUIKeyplane width → hang).
            PyroChannelTestControls(device: device, channel: ch) { pyroTestChannel = $0 }
        }
    }

    // MARK: - Per-channel profile accessors (centralise the 4-channel switch)

    private func pyroChannelEnabled(_ ch: Int) -> Bool {
        switch ch {
        case 1: return profile.pyro1Enabled
        case 2: return profile.pyro2Enabled
        case 3: return profile.pyro3Enabled
        case 4: return profile.pyro4Enabled
        default: return false
        }
    }

    private func pyroChannelTriggerMode(_ ch: Int) -> UInt8 {
        switch ch {
        case 1: return profile.pyro1TriggerMode
        case 2: return profile.pyro2TriggerMode
        case 3: return profile.pyro3TriggerMode
        case 4: return profile.pyro4TriggerMode
        default: return 0
        }
    }

    private func pyroChannelTriggerValue(_ ch: Int) -> Float {
        switch ch {
        case 1: return profile.pyro1TriggerValue
        case 2: return profile.pyro2TriggerValue
        case 3: return profile.pyro3TriggerValue
        case 4: return profile.pyro4TriggerValue
        default: return 0
        }
    }

    private func setPyroEnabled(_ ch: Int, in p: inout RocketProfile, _ value: Bool) {
        switch ch {
        case 1: p.pyro1Enabled = value
        case 2: p.pyro2Enabled = value
        case 3: p.pyro3Enabled = value
        case 4: p.pyro4Enabled = value
        default: break
        }
    }

    private func setPyroMode(_ ch: Int, in p: inout RocketProfile, _ value: UInt8) {
        switch ch {
        case 1: p.pyro1TriggerMode = value
        case 2: p.pyro2TriggerMode = value
        case 3: p.pyro3TriggerMode = value
        case 4: p.pyro4TriggerMode = value
        default: break
        }
    }

    private func setPyroValue(_ ch: Int, in p: inout RocketProfile, _ value: Float) {
        switch ch {
        case 1: p.pyro1TriggerValue = value
        case 2: p.pyro2TriggerValue = value
        case 3: p.pyro3TriggerValue = value
        case 4: p.pyro4TriggerValue = value
        default: break
        }
    }

    @ViewBuilder
    private var generalSections: some View {
        Section("Rocket") {
            HStack {
                Text("Active rocket")
                Spacer()
                Text(profile.name).foregroundColor(.secondary)
            }
            Toggle("Enable Sounds", isOn: bind(\.soundsEnabled) {
                device.sendSoundConfig(enabled: $0)
            })
            Text("Stored in the rocket profile. Persists across reboots.")
                .font(.caption).foregroundColor(.secondary)
        }

        Section("Calibration") {
            // Magnetometer: navigates to MagCalView for the full tumble flow.
            NavigationLink {
                MagCalView(device: device)
            } label: {
                Label("Magnetometer Calibration", systemImage: "location.north.line")
            }
            .disabled(!canCalibrate)
            Text("Solves for the rocket's static magnetic offset by tumbling. Saved to the active profile and re-applied on connect.")
                .font(.caption).foregroundColor(.secondary)

            // Gyro + accel: one-shot stationary cal.  OnPadCalibrationView
            // renders as a matching list-row button when embedded=true.
            OnPadCalibrationView(device: device, embedded: true)
            Text("Solves for gyro bias and accelerometer offsets. Place the rocket on the pad and keep still; takes ~10 seconds.")
                .font(.caption).foregroundColor(.secondary)

            if !canCalibrate {
                Label("Power on the rocket to run calibrations.", systemImage: "bolt.slash")
                    .font(.caption).foregroundColor(.orange)
            }
        }

        Section("IMU Mounting") {
            Picker("Mode", selection: orientModeBinding) {
                Text("Manual").tag(0)
                Text("Pad auto-detect").tag(1)
            }
            .pickerStyle(.segmented)

            if profile.imuOrientSetting != 0xFF {
                Picker("Nose axis", selection: noseAxisBinding) {
                    ForEach(0..<6, id: \.self) { i in
                        Text(["+X", "-X", "+Y", "-Y", "+Z", "-Z"][i]).tag(i)
                    }
                }
                Picker("Fin clocking", selection: clockingBinding) {
                    ForEach(0..<4, id: \.self) { i in
                        Text(["0\u{00B0}", "90\u{00B0}", "180\u{00B0}", "270\u{00B0}"][i]).tag(i)
                    }
                }
                Text("Which board axis points at the nose (up on the pad). Manual also fixes the fin clocking (quarter-turns about the nose) \u{2014} required for roll-controlled or guided flight.")
                    .font(.caption).foregroundColor(.secondary)
            } else {
                Text("Detects the nose axis from gravity on the pad. It can\u{2019}t observe fin clocking, so this is fine only for non-controlled flights \u{2014} not roll or guidance.")
                    .font(.caption).foregroundColor(.secondary)
            }

            if !device.imuOrientationName.isEmpty {
                HStack {
                    Text("Active on rocket")
                    Spacer()
                    Text("\(device.imuOrientationName) (\(device.imuOrientationMode.label))")
                        .foregroundColor(.secondary)
                }
            }
        }

        Section("IMU Logging Rate") {
            Picker("Rate", selection: imuRateBinding) {
                Text("Dynamic").tag(RocketProfile.imuRateDynamic)
                Text("1k").tag(UInt16(960))
                Text("2k").tag(UInt16(1920))
                Text("4k").tag(UInt16(3840))
            }
            .pickerStyle(.segmented)
            Text(profile.imuRateHz == RocketProfile.imuRateDynamic
                ? "Logs at 4k (3840 Hz) from the pad through boost and coast, then drops to 1k (960 Hz) once the rocket detects its recovery deployment \u{2014} full shock and vibration detail where it matters, without filling the log under canopy."
                : "Samples logged per second from the IMU (actual: 960 / 1920 / 3840 Hz). Higher rates capture faster shock and vibration detail; the control loop is unaffected. Applies on the pad \u{2014} never mid-flight.")
                .font(.caption).foregroundColor(.secondary)
        }

        // Device-level firmware update lives on the General tab only (#314).
        firmwareSection
    }

    // Board→rocket orientation: 0xFF = pad auto-detect, else code = axis*4 + clock.
    private var orientModeBinding: Binding<Int> {
        Binding(
            get: { profile.imuOrientSetting == 0xFF ? 1 : 0 },
            set: { newValue in
                let v: UInt8
                if newValue == 1 {
                    if profile.imuOrientSetting != 0xFF { savedOrientCode = profile.imuOrientSetting }
                    v = 0xFF
                } else {
                    v = savedOrientCode ?? 0
                }
                updateProfile { $0.imuOrientSetting = v }
                if device.isConnected { device.sendImuOrientationConfig(v) }
            })
    }
    private var noseAxisBinding: Binding<Int> {
        Binding(
            get: { profile.imuOrientSetting == 0xFF ? 0 : Int(profile.imuOrientSetting) / 4 },
            set: { axis in
                let clock = profile.imuOrientSetting == 0xFF ? 0 : Int(profile.imuOrientSetting) % 4
                setOrientCode(axis: axis, clock: clock)
            })
    }
    private var clockingBinding: Binding<Int> {
        Binding(
            get: { profile.imuOrientSetting == 0xFF ? 0 : Int(profile.imuOrientSetting) % 4 },
            set: { clock in
                let axis = profile.imuOrientSetting == 0xFF ? 0 : Int(profile.imuOrientSetting) / 4
                setOrientCode(axis: axis, clock: clock)
            })
    }
    private func setOrientCode(axis: Int, clock: Int) {
        let v = UInt8(clamping: axis * 4 + clock)
        updateProfile { $0.imuOrientSetting = v }
        if device.isConnected { device.sendImuOrientationConfig(v) }
    }

    // IMU logging rate: whitelisted ISM6HG256 ODR steps.
    private var imuRateBinding: Binding<UInt16> {
        Binding(
            get: { profile.imuRateHz },
            set: { hz in
                updateProfile { $0.imuRateHz = hz }
                if device.isConnected { device.sendImuRateConfig(hz) }
            })
    }

    @ViewBuilder
    private var controlModeSection: some View {
        Section(header: Text("Control Mode")) {
            Picker("Mode", selection: controlModeBinding) {
                Text("Roll Control").tag(0)
                Text("Guidance").tag(1)
            }
            .pickerStyle(.segmented)
            Text(profile.guidanceEnabled
                ? "PN guidance steers pitch/yaw toward the target. Roll is rate-nulled \u{2014} the profile is ignored."
                : "The roll controller holds the airframe (null roll, or track a roll-angle profile). No guidance.")
                .font(.caption).foregroundColor(.secondary)
        }
    }

    private var rollControlSection: some View {
        Section(header: configHeader("Roll Control", applied: rollControlApplied)) {
            Picker("Mode", selection: angleControlBinding) {
                Text("Null Roll").tag(false)
                Text("Track Profile").tag(true)
            }
            .pickerStyle(.segmented)
            Text(profile.useAngleControl
                ? "Cascaded angle control \u{2014} fins track the roll profile waypoints below."
                : "Rate-only control \u{2014} fins hold zero roll rate. No profile followed.")
                .font(.caption).foregroundColor(.secondary)

            HStack {
                Text("Activation Delay")
                Spacer()
                TextField("0", text: $sRollDelayMs)
                    .keyboardType(.numberPad)
                    .multilineTextAlignment(.trailing)
                    .frame(width: 80)
                    .focused($focusedField, equals: .rollDelay)
                Text("ms").foregroundColor(.secondary)
            }
            Text("Milliseconds after launch before control activates \u{2014} roll-rate-null and PN guidance both engage at this delay, keeping fins neutral through initial boost.")
                .font(.caption).foregroundColor(.secondary)

            HStack {
                Text("Rate Cap")
                Spacer()
                TextField("60", text: $sRateCapDps)
                    .keyboardType(.decimalPad)
                    .multilineTextAlignment(.trailing)
                    .frame(width: 80)
                    .focused($focusedField, equals: .rateCap)
                Text("\u{00B0}/s").foregroundColor(.secondary)
            }
            Text("Max roll rate the angle controller commands while slewing toward a profile angle (Track Profile mode). Higher = faster turns.")
                .font(.caption).foregroundColor(.secondary)

            HStack {
                Text("Angle Gain (Kp)")
                Spacer()
                TextField("2.0", text: $sKpAngle)
                    .keyboardType(.decimalPad)
                    .multilineTextAlignment(.trailing)
                    .frame(width: 80)
                    .focused($focusedField, equals: .kpAngle)
            }
            Text("Outer angle-loop proportional gain (Track Profile mode). Lower = gentler, better-damped return to target; higher returns faster but rings.")
                .font(.caption).foregroundColor(.secondary)

            HStack {
                Text("Anti-Windup")
                Spacer()
                TextField("40", text: $sIntegralSep)
                    .keyboardType(.decimalPad)
                    .multilineTextAlignment(.trailing)
                    .frame(width: 80)
                    .focused($focusedField, equals: .integralSep)
                Text("\u{00B0}/s").foregroundColor(.secondary)
            }
            Text("Integral-separation threshold: the roll-rate PID integrator freezes while the rate error exceeds this, preventing windup during the launch transient. 0 disables.")
                .font(.caption).foregroundColor(.secondary)

            if profile.useAngleControl {
                rollWaypointEditor
            }

            Text("Stored in the rocket profile. Persists across reboots.")
                .font(.caption).foregroundColor(.secondary)
        }
    }

    private var finLayoutSection: some View {
        Section(header: configHeader("Fin Layout", applied: finApplied)) {
            FinLayoutView(
                device: device,
                ringMode: profile.finRingMode,
                servoAtSlot: profile.finServoAtSlot,
                reverse: profile.finReverse,
                rollReverse: profile.finRollReverse,
                canJog: device.isConnected && !device.isBaseStation && device.telemetry.pwr_pin_on,
                finMinDeg: Double(profile.finMinDeg),
                finMaxDeg: Double(profile.finMaxDeg),
                onSetRingMode: { m in updateProfile { $0.finRingMode = m }; applyFinConfig() },
                onSetServoAtSlot: { s in updateProfile { $0.finServoAtSlot = s }; applyFinConfig() },
                onSetReverse: { r in updateProfile { $0.finReverse = r }; applyFinConfig() },
                onSetRollReverse: { r in updateProfile { $0.finRollReverse = r }; applyFinConfig() }
            )
            Text("Map each servo to its fin and set the ring orientation (+ on axes or \u{00D7} at 45\u{00B0}). If the rocket rolls the wrong way in ground test, flip Reverse roll direction \u{2014} that is the whole-airframe control. The per-fin toggles are for one wrong fin: Reverse pitch/yaw for backwards tilt, Reverse roll (this fin only) for a mis-linked servo. Jog each servo on the bench to confirm direction.")
                .font(.caption).foregroundColor(.secondary)
        }
    }

    // The section body is split into three @ViewBuilder pieces purely to stay
    // under SwiftUI's 10-subview ViewBuilder limit now that the law picker and
    // the per-law field set live here.
    private var guidanceSection: some View {
        Section(header: configHeader("Guidance", applied: guidanceApplied)) {
            guidanceIntroCopy
            guidanceLawFields
            guidanceSharedFields
        }
    }

    @ViewBuilder private var guidanceIntroCopy: some View {
        Group {
            Text("Roll axis is rate-nulled (held at zero spin) \u{2014} the roll profile is not followed in Guidance mode.")
                .font(.caption).foregroundColor(.secondary)
            Text("Engages at the Activation Delay after launch (shared with roll control), not after burnout.")
                .font(.caption).foregroundColor(.secondary)

            Picker("Guidance Law", selection: guidanceLawBinding) {
                Text("Proportional Nav").tag(UInt8(0))
                Text("Station-Keep").tag(UInt8(1))
            }
            .pickerStyle(.segmented)
            Text("Station-keep is a PD regulator that flies straight up over the aim point \u{2014} no closest-approach singularity. PN steers toward a target altitude and is the legacy law.")
                .font(.caption).foregroundColor(.secondary)
            Text("Aim point is currently the pad (overhead); wind-compensated aim points arrive with Drift-Cast.")
                .font(.caption).foregroundColor(.secondary)
        }
    }

    /// Per-law tuning. The other law's values stay in the profile and keep
    /// going out on the wire, so switching laws never zeroes its tune.
    @ViewBuilder private var guidanceLawFields: some View {
        if profile.pnGuidanceLaw == 0 {
            HStack {
                Text("Target Altitude")
                Spacer()
                TextField("600", text: $sPnTargetAlt)
                    .keyboardType(.decimalPad).multilineTextAlignment(.trailing).frame(width: 80)
                    .focused($focusedField, equals: .guidTargetAlt)
                Text("m").foregroundColor(.secondary)
            }
            Text("Aim-point altitude above the pad. Set near (or modestly above) expected apogee \u{2014} aiming far above weakens guidance (it reaches closest-approach early).")
                .font(.caption).foregroundColor(.secondary)

            HStack {
                Text("Nav Gain (N)")
                Spacer()
                TextField("5", text: $sPnNavGain)
                    .keyboardType(.decimalPad).multilineTextAlignment(.trailing).frame(width: 80)
                    .focused($focusedField, equals: .guidNavGain)
            }
        } else {
            HStack {
                Text("Pos Gain (kp)")
                Spacer()
                TextField("0.8", text: $sPnKpPos)
                    .keyboardType(.decimalPad).multilineTextAlignment(.trailing).frame(width: 80)
                    .focused($focusedField, equals: .guidKpPos)
                Text("1/s\u{00B2}").foregroundColor(.secondary)
            }
            HStack {
                Text("Vel Gain (kd)")
                Spacer()
                TextField("1.5", text: $sPnKdVel)
                    .keyboardType(.decimalPad).multilineTextAlignment(.trailing).frame(width: 80)
                    .focused($focusedField, equals: .guidKdVel)
                Text("1/s").foregroundColor(.secondary)
            }
            Text("Lateral accel = \u{2212}kp\u{00D7}offset \u{2212} kd\u{00D7}velocity. Both must be in (0, 10]; the rocket rejects anything outside and keeps its previous tune.")
                .font(.caption).foregroundColor(.secondary)
        }
    }

    /// Shared by both laws.
    @ViewBuilder private var guidanceSharedFields: some View {
        Group {
            HStack {
                Text("Max Accel")
                Spacer()
                TextField("20", text: $sPnMaxAccel)
                    .keyboardType(.decimalPad).multilineTextAlignment(.trailing).frame(width: 80)
                    .focused($focusedField, equals: .guidMaxAccel)
                Text("m/s\u{00B2}").foregroundColor(.secondary)
            }
            HStack {
                Text("Accel\u{2192}Fin")
                Spacer()
                TextField("4", text: $sPnAccelToFin)
                    .keyboardType(.decimalPad).multilineTextAlignment(.trailing).frame(width: 80)
                    .focused($focusedField, equals: .guidAccelToFin)
            }
            HStack {
                Text("Max Fin")
                Spacer()
                TextField("15", text: $sPnMaxFin)
                    .keyboardType(.decimalPad).multilineTextAlignment(.trailing).frame(width: 80)
                    .focused($focusedField, equals: .guidMaxFin)
                Text("deg").foregroundColor(.secondary)
            }
            HStack {
                Text("Min Speed")
                Spacer()
                TextField("15", text: $sPnMinSpeed)
                    .keyboardType(.decimalPad).multilineTextAlignment(.trailing).frame(width: 80)
                    .focused($focusedField, equals: .guidMinSpeed)
                Text("m/s").foregroundColor(.secondary)
            }
            HStack {
                Text("Activation Delay")
                Spacer()
                TextField("0", text: $sRollDelayMs)
                    .keyboardType(.numberPad).multilineTextAlignment(.trailing).frame(width: 80)
                    .focused($focusedField, equals: .rollDelay)
                Text("ms").foregroundColor(.secondary)
            }
            Text("Milliseconds after launch before control activates \u{2014} roll-rate-null and guidance both engage at this delay, keeping fins neutral through initial boost.")
                .font(.caption).foregroundColor(.secondary)
            Text(profile.pnGuidanceLaw == 0
                 ? "Nav gain = PN aggressiveness (3\u{2013}5). Min speed gates guidance off below useful fin authority. Stored in the rocket profile."
                 : "Min speed gates guidance off below useful fin authority. Stored in the rocket profile.")
                .font(.caption).foregroundColor(.secondary)
        }
    }

    @ViewBuilder
    private var rollWaypointEditor: some View {
        Text("The target roll angle ramps linearly between waypoints. Before the first waypoint the controller nulls roll rate (fins keep zero roll through boost); after the last waypoint the final angle is held. To hold an angle, give two consecutive waypoints the same angle.")
            .font(.caption).foregroundColor(.secondary)

        ForEach(rollWaypoints.indices, id: \.self) { i in
            HStack(spacing: 8) {
                Text("WP \(i + 1)")
                    .font(.caption).foregroundColor(.secondary).frame(width: 40)
                TextField("Time", text: Binding(
                    get: { rollWaypoints[i].time },
                    set: { rollWaypoints[i].time = $0 }))
                    .keyboardType(.decimalPad)
                    .multilineTextAlignment(.trailing).frame(width: 60)
                    .focused($focusedField, equals: .wpTime(i))
                Text("s").foregroundColor(.secondary).frame(width: 15)
                TextField("Angle", text: Binding(
                    get: { rollWaypoints[i].angle },
                    set: { rollWaypoints[i].angle = $0 }))
                    .keyboardType(.numbersAndPunctuation)
                    .multilineTextAlignment(.trailing).frame(width: 60)
                    .focused($focusedField, equals: .wpAngle(i))
                Text("\u{00B0}").foregroundColor(.secondary).frame(width: 15)
                Button(role: .destructive) {
                    rollWaypoints.remove(at: i)
                    applyRollProfile()
                } label: {
                    Image(systemName: "minus.circle.fill").foregroundColor(.red)
                }
                .buttonStyle(.borderless)
            }
        }

        if rollWaypoints.count < 8 {
            Button {
                let defaultTime = rollWaypoints.isEmpty ? "0.0" :
                    String(format: "%.1f", (Double(rollWaypoints.last?.time ?? "0") ?? 0) + 1.0)
                rollWaypoints.append((time: defaultTime, angle: "0"))
                applyRollProfile()
            } label: {
                HStack { Image(systemName: "plus.circle.fill"); Text("Add Waypoint") }
            }
        }

        Button(role: .destructive) {
            rollWaypoints.removeAll()
            updateProfile { $0.rollWaypoints = [] }
            if device.isConnected { device.sendRollProfileClear() }
            showApplied($rollControlApplied)
        } label: {
            HStack { Image(systemName: "trash"); Text("Clear Roll Profile") }
                .frame(maxWidth: .infinity)
        }
    }

    private var rocketOffSection: some View {
        Section {
            VStack(spacing: 12) {
                Image(systemName: "power")
                    .font(.system(size: 40)).foregroundColor(.secondary)
                Text("Rocket is off").font(.title3.bold())
                Text("The flight computer answers every setting on this screen, and it isn't running yet. Power on the rocket to configure it.")
                    .font(.subheadline).foregroundColor(.secondary)
                    .multilineTextAlignment(.center)
            }
            .frame(maxWidth: .infinity)
            .padding(.vertical, 12)
        }
    }

    private var initializingOverlay: some View {
        ZStack {
            Color(.systemBackground).opacity(0.9).ignoresSafeArea()
            VStack(spacing: 16) {
                ProgressView().scaleEffect(1.5)
                Text("Initializing...").font(.title2.bold())
                Text("Waiting for sensors to start up.\nSettings can be applied once ready.")
                    .font(.subheadline).foregroundColor(.secondary)
                    .multilineTextAlignment(.center)
            }
            .padding()
        }
    }

    // MARK: - Profile bindings

    private func updateProfile(_ mutate: (inout RocketProfile) -> Void) {
        guard let id = store.activeId else { return }
        store.update(id, mutate)
    }

    /// Binding into a profile field that persists the edit and, when a rocket
    /// is connected, pushes it live via `push`.
    private func bind<T>(_ kp: WritableKeyPath<RocketProfile, T>,
                         push: @escaping (T) -> Void) -> Binding<T> {
        Binding(
            get: { profile[keyPath: kp] },
            set: { newValue in
                updateProfile { $0[keyPath: kp] = newValue }
                if device.isConnected { push(newValue) }
            })
    }

    private var cameraBinding: Binding<Int> {
        Binding(
            get: { Int(profile.cameraType) },
            set: { newValue in
                updateProfile { $0.cameraType = UInt8(clamping: newValue) }
                if device.isConnected { device.sendCameraConfig(cameraType: UInt8(clamping: newValue)) }
            })
    }

    private var angleControlBinding: Binding<Bool> {
        Binding(
            get: { profile.useAngleControl },
            set: { newValue in
                updateProfile { $0.useAngleControl = newValue }
                applyRollControlConfig()
            })
    }

    /// Top-level control mode: 0 = Roll Control, 1 = Guidance.  Derived from
    /// guidanceEnabled.  Guidance forces useAngleControl=false so roll is rate-nulled
    /// in every phase (the profile is never followed); switching back to Roll Control
    /// restores the prior roll sub-mode (Null Roll / Track Profile).
    private var controlModeBinding: Binding<Int> {
        Binding(
            get: { profile.guidanceEnabled ? 1 : 0 },
            set: { newValue in
                if newValue == 1 {
                    savedRollUsesAngle = profile.useAngleControl
                    updateProfile { $0.guidanceEnabled = true; $0.useAngleControl = false }
                } else {
                    updateProfile {
                        $0.guidanceEnabled = false
                        if let prev = savedRollUsesAngle { $0.useAngleControl = prev }
                    }
                }
                applyGuidanceConfig()
                applyRollControlConfig()
            })
    }

    /// Guidance law: 0 = proportional navigation, 1 = station-keep.  Applies on
    /// change (#144) — no Apply button.  Both laws' parameters ride in every
    /// frame, so flipping the picker never drops the other law's tune.
    private var guidanceLawBinding: Binding<UInt8> {
        Binding(
            get: { profile.pnGuidanceLaw },
            set: { newValue in
                updateProfile { $0.pnGuidanceLaw = newValue }
                applyGuidanceConfig()
            })
    }

    // Pyro enable / mode apply immediately (like toggles); the trigger value
    // commits on focus loss via the .pyro EditGroup.
    private func pyroEnabledBinding(_ ch: Int) -> Binding<Bool> {
        Binding(
            get: { pyroChannelEnabled(ch) },
            set: { newValue in
                updateProfile { setPyroEnabled(ch, in: &$0, newValue) }
                applyPyroConfig()
            })
    }

    private func pyroModeBinding(_ ch: Int) -> Binding<Int> {
        Binding(
            get: { Int(pyroChannelTriggerMode(ch)) },
            set: { newValue in
                updateProfile { setPyroMode(ch, in: &$0, UInt8(newValue)) }
                reloadPyroValueStrings()   // reformat for the new unit (s vs m)
                applyPyroConfig()
            })
    }

    // MARK: - String ↔ profile sync

    private func loadFromProfile() {
        let p = profile
        sBias1 = formatInt(Double(p.servoBias1))
        sBias2 = formatInt(Double(p.servoBias2))
        sBias3 = formatInt(Double(p.servoBias3))
        sBias4 = formatInt(Double(p.servoBias4))
        sServoHz = formatInt(Double(p.servoHz))
        sServoMinUs = formatInt(Double(p.servoMinUs))
        sServoMaxUs = formatInt(Double(p.servoMaxUs))
        sFinTravelDeg = formatDecimal(Double(p.finTravelDeg))
        sPidKp = formatDecimal(Double(p.pidKp))
        sPidKi = formatDecimal(Double(p.pidKi))
        sPidKd = formatDecimal(Double(p.pidKd))
        sPidMinCmd = formatDecimal(Double(p.pidMinCmd))
        sPidMaxCmd = formatDecimal(Double(p.pidMaxCmd))
        sRollDelayMs = formatInt(Double(p.rollDelayMs))
        sRateCapDps = formatInt(Double(p.rateCapDps))
        sKpAngle = formatDecimal(Double(p.kpAngle))
        sIntegralSep = formatInt(Double(p.integralSepThreshold))
        sPnTargetAlt = formatInt(Double(p.pnTargetAltM))
        sPnNavGain = formatDecimal(Double(p.pnNavGain))
        sPnMaxAccel = formatDecimal(Double(p.pnMaxAccel))
        sPnAccelToFin = formatDecimal(Double(p.pnAccelToFin))
        sPnMaxFin = formatDecimal(Double(p.pnMaxFinDeg))
        sPnMinSpeed = formatDecimal(Double(p.pnMinSpeed))
        sPnKpPos = formatDecimal(Double(p.pnKpPos))
        sPnKdVel = formatDecimal(Double(p.pnKdVel))
        rollWaypoints = p.rollWaypoints.map {
            (time: trimFloat($0.timeSeconds), angle: trimFloat($0.angleDeg))
        }
        sDrogueRate = formatInt(p.drogueRateFps)
        sMainRate = formatInt(p.mainRateFps)
        sMainDeployAlt = formatInt(p.mainDeployAltAglFt)
        sDragK = formatDecimal(p.ballisticDragK)
        reloadPyroValueStrings()
    }

    private func reloadPyroValueStrings() {
        for ch in 1...4 {
            sPyroValue[ch - 1] = formatPyro(pyroChannelTriggerValue(ch),
                                             mode: pyroChannelTriggerMode(ch))
        }
    }

    /// Time-after-apogee shows one decimal (seconds); altitude is stored in
    /// metres but shown in the display unit, whole numbers (#160).
    private func formatPyro(_ v: Float, mode: UInt8) -> String {
        if mode == 0 { return String(format: "%.1f", v) }
        return String(format: "%.0f", UnitFormatter.metersToDisplay(Double(v), system: unitSystem))
    }

    /// Inverse of `formatPyro`: parse an entered value back to canonical units
    /// — seconds for time mode, metres for altitude mode (entered in the
    /// display unit).
    private func parsePyroValue(_ s: String, mode: UInt8, fallback: Float) -> Float {
        guard let entered = Float(s) else { return fallback }
        if mode == 0 { return entered }
        return Float(UnitFormatter.displayToMeters(Double(entered), system: unitSystem))
    }

    private func formatInt(_ v: Double) -> String {
        v == v.rounded() ? String(Int(v)) : String(v)
    }

    private func formatDecimal(_ v: Double) -> String {
        let s = String(format: "%.6f", v)
        var trimmed = s
        while trimmed.hasSuffix("0") && !trimmed.hasSuffix(".0") { trimmed.removeLast() }
        return trimmed
    }

    private func trimFloat(_ v: Float) -> String {
        v == v.rounded() ? String(Int(v)) : String(v)
    }

    private func parseDouble(_ s: String, fallback: Double) -> Double { Double(s) ?? fallback }

    // MARK: - Helpers

    private func stringRow(_ label: String, text: Binding<String>, field: EditField,
                           unit: String? = nil, decimal: Bool = false) -> some View {
        HStack {
            Text(label)
            Spacer()
            TextField(decimal ? "0.0" : "0", text: text)
                .keyboardType(decimal ? .decimalPad : .numbersAndPunctuation)
                .multilineTextAlignment(.trailing)
                .frame(width: 80)
                .focused($focusedField, equals: field)
            if let unit = unit {
                Text(unit).foregroundColor(.secondary).fixedSize().frame(minWidth: 30, alignment: .leading)
            }
        }
    }

    private func configHeader(_ title: String, applied: Bool) -> some View {
        HStack {
            Text(title)
            if applied {
                Spacer()
                Label("Sent", systemImage: "checkmark.circle.fill")
                    .font(.caption).foregroundColor(.green)
            }
        }
    }

    private func summaryRow(_ label: String, _ value: String) -> some View {
        HStack { Text(label); Spacer(); Text(value).foregroundColor(.secondary) }
    }

    private func cameraLabel(_ t: UInt8) -> String {
        switch t { case 1: return "GoPro"; case 2: return "RunCam"; default: return "None" }
    }

    private func cameraHint(_ t: Int) -> String {
        switch t {
        case 1: return "GoPro: controlled via GPIO pulse on shutter pin."
        case 2: return "RunCam: controlled via UART serial command."
        default: return "No camera connected."
        }
    }

    // MARK: - Apply actions (write profile + push when connected)

    private func applyServoConfig() {
        let b1 = Int16(clamping: Int(parseDouble(sBias1, fallback: Double(profile.servoBias1)).rounded()))
        let b2 = Int16(clamping: Int(parseDouble(sBias2, fallback: Double(profile.servoBias2)).rounded()))
        let b3 = Int16(clamping: Int(parseDouble(sBias3, fallback: Double(profile.servoBias3)).rounded()))
        let b4 = Int16(clamping: Int(parseDouble(sBias4, fallback: Double(profile.servoBias4)).rounded()))
        let hz = Int16(clamping: Int(parseDouble(sServoHz, fallback: Double(profile.servoHz)).rounded()))
        let mn = Int16(clamping: Int(parseDouble(sServoMinUs, fallback: Double(profile.servoMinUs)).rounded()))
        let mx = Int16(clamping: Int(parseDouble(sServoMaxUs, fallback: Double(profile.servoMaxUs)).rounded()))
        let travel = Float(parseDouble(sFinTravelDeg, fallback: Double(profile.finTravelDeg)))

        updateProfile {
            $0.servoBias1 = b1; $0.servoBias2 = b2; $0.servoBias3 = b3; $0.servoBias4 = b4
            $0.servoHz = hz; $0.servoMinUs = mn; $0.servoMaxUs = mx
            $0.finTravelDeg = travel
        }
        // #449: the endpoint angles the FC calibrates against are derived from the
        // travel, never typed — so the pair it receives always describes a real servo.
        let fmn = -travel / 2, fmx = travel / 2
        if device.isConnected {
            device.sendServoConfig(biases: [b1, b2, b3, b4], hz: hz, minUs: mn, maxUs: mx,
                                   finMinDeg: fmn, finMaxDeg: fmx)
        }
        showApplied($servoApplied)
    }

    private func applyPyroConfig() {
        // Parse all four text fields back to canonical units, then push.
        var parsed: [Float] = [0, 0, 0, 0]
        for ch in 1...4 {
            parsed[ch - 1] = parsePyroValue(
                sPyroValue[ch - 1],
                mode: pyroChannelTriggerMode(ch),
                fallback: pyroChannelTriggerValue(ch))
        }
        updateProfile {
            for ch in 1...4 { setPyroValue(ch, in: &$0, parsed[ch - 1]) }
        }
        let p = profile
        if device.isConnected {
            device.sendPyroConfig(channels: [
                (p.pyro1Enabled, p.pyro1TriggerMode, p.pyro1TriggerValue),
                (p.pyro2Enabled, p.pyro2TriggerMode, p.pyro2TriggerValue),
                (p.pyro3Enabled, p.pyro3TriggerMode, p.pyro3TriggerValue),
                (p.pyro4Enabled, p.pyro4TriggerMode, p.pyro4TriggerValue),
            ])
            // Mirror into rocketConfig so the dashboard pyro tiles update live.
            if var cfg = device.rocketConfig {
                cfg.pyro1Enabled = p.pyro1Enabled
                cfg.pyro1TriggerMode = p.pyro1TriggerMode
                cfg.pyro1TriggerValue = p.pyro1TriggerValue
                cfg.pyro2Enabled = p.pyro2Enabled
                cfg.pyro2TriggerMode = p.pyro2TriggerMode
                cfg.pyro2TriggerValue = p.pyro2TriggerValue
                cfg.pyro3Enabled = p.pyro3Enabled
                cfg.pyro3TriggerMode = p.pyro3TriggerMode
                cfg.pyro3TriggerValue = p.pyro3TriggerValue
                cfg.pyro4Enabled = p.pyro4Enabled
                cfg.pyro4TriggerMode = p.pyro4TriggerMode
                cfg.pyro4TriggerValue = p.pyro4TriggerValue
                device.rocketConfig = cfg
            }
        }
        showApplied($pyroApplied)
    }

    // Recovery fields are app-only (#191): "apply" is just parse + persist —
    // no BLE push, no "Sent" badge.  Non-positive rates/altitudes are
    // rejected: simulateDescent degrades a rate ≤ 0.1 fps to a one-point
    // track, which the live predictor would render as "lands where it is".
    // Drag k legitimately accepts 0 (= gravity-only ballistic).
    private func applyRecoveryConfig() {
        let dr = parseDouble(sDrogueRate, fallback: profile.drogueRateFps)
        let mr = parseDouble(sMainRate, fallback: profile.mainRateFps)
        let md = parseDouble(sMainDeployAlt, fallback: profile.mainDeployAltAglFt)
        let k = parseDouble(sDragK, fallback: profile.ballisticDragK)
        updateProfile {
            if dr > 0 { $0.drogueRateFps = dr }
            if mr > 0 { $0.mainRateFps = mr }
            if md > 0 { $0.mainDeployAltAglFt = md }
            if k >= 0 { $0.ballisticDragK = k }
        }
    }

    private func applyPIDConfig() {
        let kp = Float(parseDouble(sPidKp, fallback: Double(profile.pidKp)))
        let ki = Float(parseDouble(sPidKi, fallback: Double(profile.pidKi)))
        let kd = Float(parseDouble(sPidKd, fallback: Double(profile.pidKd)))
        let mn = Float(parseDouble(sPidMinCmd, fallback: Double(profile.pidMinCmd)))
        let mx = Float(parseDouble(sPidMaxCmd, fallback: Double(profile.pidMaxCmd)))

        updateProfile {
            $0.pidKp = kp; $0.pidKi = ki; $0.pidKd = kd; $0.pidMinCmd = mn; $0.pidMaxCmd = mx
        }
        if device.isConnected {
            device.sendPIDConfig(kp: kp, ki: ki, kd: kd, minCmd: mn, maxCmd: mx)
        }
        showApplied($pidApplied)
    }

    private func applyRollControlConfig() {
        let delayMs = UInt16(clamping: Int(Double(sRollDelayMs) ?? Double(profile.rollDelayMs)))
        let rateCap = max(0, Float(sRateCapDps) ?? profile.rateCapDps)
        let kpAngle = max(0, Float(sKpAngle) ?? profile.kpAngle)
        let iwind   = max(0, Float(sIntegralSep) ?? profile.integralSepThreshold)
        let useAngle = profile.useAngleControl
        updateProfile {
            $0.rollDelayMs = delayMs
            $0.rateCapDps = rateCap
            $0.kpAngle = kpAngle
            $0.integralSepThreshold = iwind
        }
        if device.isConnected {
            device.sendRollControlConfig(useAngleControl: useAngle, rollDelayMs: delayMs, rateCapDps: rateCap,
                                         kpAngle: kpAngle, integralSepThreshold: iwind)
        }
        showApplied($rollControlApplied)
    }

    private func applyFinConfig() {
        if device.isConnected {
            device.sendFinConfig(ringMode: profile.finRingMode,
                                 servoAtSlot: profile.finServoAtSlot,
                                 reverse: profile.finReverse,
                                 rollReverse: profile.finRollReverse)
        }
        showApplied($finApplied)
    }

    private func applyGuidanceConfig() {
        let navGain    = max(0, Float(sPnNavGain)    ?? profile.pnNavGain)
        let maxAccel   = max(0, Float(sPnMaxAccel)   ?? profile.pnMaxAccel)
        let accelToFin = max(0, Float(sPnAccelToFin) ?? profile.pnAccelToFin)
        let maxFin     = max(0, Float(sPnMaxFin)     ?? profile.pnMaxFinDeg)
        let minSpeed   = max(0, Float(sPnMinSpeed)   ?? profile.pnMinSpeed)
        let coastDelay = profile.pnCoastDelayMs  // inert in FW; guidance activation = rollDelayMs (Activation Delay)
        let targetAlt  = max(0, Float(sPnTargetAlt)  ?? profile.pnTargetAltM)
        // Station-keep gains (#534).  Same floor-of-0 idiom as the rest — note
        // Float("nan") survives it; the FC's (0, 10] gates are the real guard and
        // they log on rejection.  Both laws' values are sent every time.
        let kpPos      = max(0, Float(sPnKpPos)      ?? profile.pnKpPos)
        let kdVel      = max(0, Float(sPnKdVel)      ?? profile.pnKdVel)
        updateProfile {
            $0.pnNavGain = navGain; $0.pnMaxAccel = maxAccel; $0.pnAccelToFin = accelToFin
            $0.pnMaxFinDeg = maxFin; $0.pnMinSpeed = minSpeed; $0.pnCoastDelayMs = coastDelay
            $0.pnTargetAltM = targetAlt
            $0.pnKpPos = kpPos; $0.pnKdVel = kdVel
        }
        if device.isConnected {
            device.sendGuidanceConfig(enabled: profile.guidanceEnabled,
                                      navGain: navGain, maxAccel: maxAccel, accelToFin: accelToFin,
                                      maxFinDeg: maxFin, minSpeed: minSpeed, coastDelayMs: coastDelay,
                                      targetMode: profile.pnTargetMode,
                                      targetE: profile.pnTargetE, targetN: profile.pnTargetN,
                                      targetAlt: targetAlt,
                                      kpPos: kpPos, kdVel: kdVel,
                                      guidanceLaw: profile.pnGuidanceLaw)
        }
        showApplied($guidanceApplied)
    }

    private func applyRollProfile() {
        let waypoints: [RollWaypoint] = rollWaypoints.map {
            RollWaypoint(timeSeconds: Float($0.time) ?? 0,
                         angleDeg: Float($0.angle) ?? 0)
        }.sorted { $0.timeSeconds < $1.timeSeconds }

        updateProfile { $0.rollWaypoints = waypoints }
        if device.isConnected {
            // mode byte is a legacy wire field (pre-v4); always send .angle
            device.sendRollProfile(waypoints: waypoints.map {
                (time: $0.timeSeconds, angle: $0.angleDeg, mode: RollSegmentMode.angle.rawValue)
            })
        }
        showApplied($rollControlApplied)
    }

    // MARK: - LoRa TX power (base station only)

    // #150: network identity (app side; onboarding writes these).
    @AppStorage("networkName") private var appNetworkName: String = ""
    @AppStorage("networkID") private var appNetworkID: Int = 0

    private var deviceNidMismatch: Bool {
        // unitID empty = no config_identity readback yet — don't render
        // "identity unknown" as a confident mismatch at ID 0.
        appNetworkID > 0 && !device.unitID.isEmpty &&
        device.networkID != UInt8(clamping: appNetworkID)
    }

    private var networkFooter: String {
        if deviceNidMismatch {
            return NetworkCopy.deviceMismatchWarning
                + " Tap to sync it (a rising NetID Drops count on the dashboard is the same problem seen from the other side)."
        }
        return NetworkCopy.sameNetworkExplainer
            + " Set during onboarding; pushed to each device when provisioned."
    }

    // #150: hopping is unavailable when the firmware reports dwell 0 for
    // the current modulation (a packet wouldn't fit the FCC occupancy
    // window).  nil (pre-#150 firmware) is treated as available.
    private var hopUnavailable: Bool {
        (device.rocketConfig?.loraHopDwell ?? 1) == 0
    }

    private var linkModeFooter: String {
        // Surface WHY the picker is locked (same pattern as txPowerFooter)
        // instead of greying silently.
        if let refusal = device.autoApplyRefusalReason() {
            return refusal.rawValue
        }
        if hopUnavailable {
            return "Frequency hopping isn't available at the current radio settings — a packet wouldn't fit the FCC dwell window."
        }
        return linkModeHopping
            ? "FCC-compliant hopping across the channel set. The rocket follows automatically; switching modes takes a few seconds."
            : "Single fixed channel — the default."
    }

    private var linkModeBinding: Binding<Bool> {
        Binding(
            get: { linkModeHopping },
            set: { newValue in
                guard newValue != linkModeHopping else { return }
                linkModeHopping = newValue
                linkModeTouchedAt = Date()   // #150: hold off hydration (see onReceive)
                if device.isConnected {
                    device.sendLoRaHopDisabled(!newValue)
                }
            }
        )
    }

    private func scheduleTxPowerSend(_ newValue: Int) {
        txPowerSendWork?.cancel()
        let work = DispatchWorkItem {
            let current = device.rocketConfig?.loraTxPower.map(Int.init) ?? Int.min
            if newValue != current { device.autoApplyTxPower(Int8(newValue)) }
        }
        txPowerSendWork = work
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.5, execute: work)
    }

    @ViewBuilder
    private var txPowerFooter: some View {
        if let refusal = device.autoApplyRefusalReason() {
            Text(refusal.rawValue).foregroundColor(.orange)
        } else if linkModeHopping {
            Text("TX power changes are locked while hopping — switch Link Mode to Fixed Channel first.")
                .foregroundColor(.orange)
        } else {
            Text("Sets transmit power on both the base station and the rocket. The base station relays the change over LoRa, verifies the rocket joined the new setting, and rolls both sides back if it can't be reached.")
        }
    }

    private func showApplied(_ flag: Binding<Bool>) {
        guard device.isConnected else { return }
        flag.wrappedValue = true
        DispatchQueue.main.asyncAfter(deadline: .now() + 2.0) { flag.wrappedValue = false }
    }
}

// MARK: - Pyro live controls

/// Continuity readout + Test Continuity / Test Pyro Channel for one channel,
/// factored out of SettingsView's pyro Section so the section that holds the
/// focused Delay TextField stays telemetry-independent and structurally stable.
/// Inlined, these read live telemetry in SettingsView's body, so the continuity
/// dot and buttons appeared/disappeared *in the same Section as the focused
/// TextField* on every frame — which disrupted keyboard layout on the first
/// Delay focus (negative TUIKeyplane width → hang).  Isolating them in a child
/// keeps that telemetry churn out of the config Section entirely.
///
/// Mirrors the dashboard pyro tile: continuity shows only while the shared ARM
/// FET is engaged (after a Test Continuity pulse or when armed) and is gated to
/// a LIVE frame so a stale frame fails safe to NO CONT (#297); the actuating
/// buttons hide once armed / fired / in flight.
private struct PyroChannelTestControls: View {
    @ObservedObject var device: BLEDevice
    let channel: Int
    /// Open the full-screen test-fire cover for this channel (parent state).
    let onTestFire: (Int) -> Void

    @State private var contTestActive = false

    var body: some View {
        if device.isConnected && !device.isBaseStation {
            if device.telemetry.pyro_armed || contTestActive {
                let cont = device.telemetry.pyroCont(channel: channel)
                    && device.telemetry.data_status == .live
                HStack {
                    Text("Continuity")
                    Spacer()
                    if device.contTestPending(channel: UInt8(channel)) {
                        ProgressView().controlSize(.mini)
                        Text("TESTING")
                            .font(.subheadline.weight(.semibold))
                            .foregroundColor(.secondary)
                    } else {
                        Circle().fill(cont ? Color.green : Color.red)
                            .frame(width: 8, height: 8)
                        Text(cont ? "CONT" : "NO CONT")
                            .font(.subheadline.weight(.semibold))
                            .foregroundColor(cont ? .green : .red)
                    }
                }
            }

            let blocked = device.telemetry.pyro_armed
                || device.telemetry.pyroFired(channel: channel)
                || device.telemetry.state == "INFLIGHT"
            if !blocked {
                // Rail gate: with the FC powered off the OC refuses cmd 35/36
                // outright (a queued test would deliver its fire/ARM pulse at
                // the next power-on), so don't offer buttons that can only be
                // refused. #377: pwr_pin_on reads false until the first
                // telemetry frame, which fails safe to disabled here.
                let railOn = device.telemetry.pwr_pin_on
                Button("Test Continuity") {
                    device.sendPyroContTest(channel: UInt8(channel))
                    contTestActive = true
                    // Auto-hide the readout after 5s, matching the dashboard.
                    DispatchQueue.main.asyncAfter(deadline: .now() + 5) {
                        contTestActive = false
                    }
                }
                .disabled(!railOn)
                Button("Test Pyro Channel") { onTestFire(channel) }
                    .foregroundColor(railOn ? .orange : .secondary)
                    .disabled(!railOn)
                if !railOn {
                    Label("Power on the rocket to test.", systemImage: "bolt.slash")
                        .font(.caption).foregroundColor(.orange)
                }
            }
        }
    }
}
