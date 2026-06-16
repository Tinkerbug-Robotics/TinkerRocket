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
    @State private var sPidKp = ""
    @State private var sPidKi = ""
    @State private var sPidKd = ""
    @State private var sPidMinCmd = ""
    @State private var sPidMaxCmd = ""
    @State private var sRollDelayMs = ""
    @State private var sRateCapDps = ""

    // Roll waypoints edited as strings; committed to the profile on change.
    @State private var rollWaypoints: [(time: String, angle: String, mode: UInt8)] = []

    // Pyro trigger values edited as strings (seconds or meters by mode).
    @State private var sPyroValue: [String] = ["", "", "", ""]

    // "Sent" feedback in section headers.
    @State private var servoApplied = false
    @State private var pidApplied = false
    @State private var rollControlApplied = false
    @State private var pyroApplied = false

    // LoRa TX power (BS only) — hydrated from rocketConfig, debounced send.
    @State private var txPowerDbm: Int = 12
    @State private var txPowerSendWork: DispatchWorkItem?

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
        case bias1, bias2, bias3, bias4, servoHz, servoMin, servoMax
        case pidKp, pidKi, pidKd, pidMin, pidMax
        case rollDelay
        case rateCap
        case wpTime(Int), wpAngle(Int)
        case pyroValue(Int)   // ch index 0..3
    }

    private enum EditGroup { case servo, pid, rollControl, rollWaypoints, pyro }

    private func group(of field: EditField?) -> EditGroup? {
        switch field {
        case .bias1, .bias2, .bias3, .bias4, .servoHz, .servoMin, .servoMax: return .servo
        case .pidKp, .pidKi, .pidKd, .pidMin, .pidMax: return .pid
        case .rollDelay, .rateCap: return .rollControl
        case .wpTime, .wpAngle: return .rollWaypoints
        case .pyroValue: return .pyro
        case nil: return nil
        }
    }

    private func applyGroup(_ g: EditGroup) {
        switch g {
        case .servo: applyServoConfig()
        case .pid: applyPIDConfig()
        case .rollControl: applyRollControlConfig()
        case .rollWaypoints: applyRollProfile()
        case .pyro: applyPyroConfig()
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
                if device.isBaseStation {
                    baseStationSections
                } else if store.activeProfile == nil {
                    noProfileSection
                } else {
                    rocketSettingsSections
                }
                // Firmware update is device-level (not profile-level), so it
                // sits outside the branch above and shows for BS + OC alike.
                firmwareSection
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
                // Only LoRa TX power is hydrated from the rocket now — every
                // other setting is owned by the profile (app is source of
                // truth) so we no longer pull config back into local state.
                if let pwr = cfg.loraTxPower { txPowerDbm = Int(pwr) }
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
                summaryRow("Control mode", p.useAngleControl ? "Track Profile" : "Null Roll")
                summaryRow("Camera", cameraLabel(p.cameraType))
                summaryRow("IMU mounting", p.imuOrientSetting == 0xFF
                    ? "Auto" : "Nose \(FlightSettingsData.b2rName(code: p.imuOrientSetting))")
                summaryRow("Gain scheduling", p.gainScheduleEnabled ? "On" : "Off")
                summaryRow("Mag cal", p.magCal == nil ? "Not saved" : "Saved")
                summaryRow("Sensor cal", p.sensorCal == nil ? "Not saved" : "Saved")
            }
        }

        loRaSections
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
        Section(header: Text("LoRa Frequency"),
                footer: Text("Base station picks the quietest channel automatically at boot. Both devices should report the same frequency.")) {
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
            .disabled(device.autoApplyRefusalReason() != nil)
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
        case .general:  generalSections
        }
    }

    @ViewBuilder
    private var controlSections: some View {
        Section(header: configHeader("PID Gains", applied: pidApplied)) {
            stringRow("Kp", text: $sPidKp, field: .pidKp, decimal: true)
            stringRow("Ki", text: $sPidKi, field: .pidKi, decimal: true)
            stringRow("Kd", text: $sPidKd, field: .pidKd, decimal: true)
            stringRow("Min Cmd", text: $sPidMinCmd, field: .pidMin, unit: "deg")
            stringRow("Max Cmd", text: $sPidMaxCmd, field: .pidMax, unit: "deg")
            Toggle("Velocity Gain Scheduling", isOn: bind(\.gainScheduleEnabled) {
                device.sendGainScheduleConfig(enabled: $0)
            })
            Text("Scales PID gains with (V_ref/V)\u{00B2}. Disable for fixed gains at all speeds.")
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
            Text("Microsecond offset per servo to trim mechanical misalignment.")
                .font(.caption).foregroundColor(.secondary)
            stringRow("Frequency", text: $sServoHz, field: .servoHz, unit: "Hz")
            stringRow("Min Pulse", text: $sServoMinUs, field: .servoMin, unit: "\u{00B5}s")
            stringRow("Max Pulse", text: $sServoMaxUs, field: .servoMax, unit: "\u{00B5}s")
        }

        rollControlSection
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
            Picker("Orientation", selection: imuOrientBinding) {
                Text("Auto-detect").tag(255)
                ForEach(0..<24, id: \.self) { code in
                    Text("Nose \(FlightSettingsData.b2rName(code: UInt8(code)))").tag(code)
                }
            }
            if !device.imuOrientationName.isEmpty {
                HStack {
                    Text("Active on rocket")
                    Spacer()
                    Text("\(device.imuOrientationName) (\(device.imuOrientationMode.label))")
                        .foregroundColor(.secondary)
                }
            }
            Text(profile.imuOrientSetting == 0xFF
                ? "Auto detects which board axis points at the nose from gravity on the pad. Fine for non-controlled flights."
                : "Manual mounting also fixes the fin clocking (rNN = quarter-turns about the nose) — required for roll-controlled or guided flights when the board is mounted off-axis.")
                .font(.caption).foregroundColor(.secondary)
        }
    }

    private var imuOrientBinding: Binding<Int> {
        Binding(
            get: { Int(profile.imuOrientSetting) },
            set: { newValue in
                let v = UInt8(clamping: newValue)
                updateProfile { $0.imuOrientSetting = v }
                if device.isConnected { device.sendImuOrientationConfig(v) }
            })
    }

    @ViewBuilder
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
            Text("Milliseconds after launch before any roll control activates. Keeps fins neutral during initial boost.")
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

            if profile.useAngleControl {
                rollWaypointEditor
            }

            Text("Stored in the rocket profile. Persists across reboots.")
                .font(.caption).foregroundColor(.secondary)
        }
    }

    @ViewBuilder
    private var rollWaypointEditor: some View {
        Text("Each waypoint defines the START of a segment. Angle interpolates the target roll angle to the next waypoint; Null Rate holds zero roll rate (angle ignored) for the segment.")
            .font(.caption).foregroundColor(.secondary)

        ForEach(rollWaypoints.indices, id: \.self) { i in
            VStack(spacing: 4) {
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
                        .disabled(rollWaypoints[i].mode == 1)
                        .foregroundColor(rollWaypoints[i].mode == 1 ? .secondary : .primary)
                    Text("\u{00B0}").foregroundColor(.secondary).frame(width: 15)
                    Button(role: .destructive) {
                        rollWaypoints.remove(at: i)
                        applyRollProfile()
                    } label: {
                        Image(systemName: "minus.circle.fill").foregroundColor(.red)
                    }
                    .buttonStyle(.borderless)
                }
                Picker("Mode", selection: Binding(
                    get: { rollWaypoints[i].mode },
                    set: { rollWaypoints[i].mode = $0; applyRollProfile() })) {
                    Text("Angle").tag(UInt8(0))
                    Text("Null Rate").tag(UInt8(1))
                }
                .pickerStyle(.segmented)
                .padding(.leading, 40)
            }
        }

        if rollWaypoints.count < 8 {
            Button {
                let defaultTime = rollWaypoints.isEmpty ? "0.0" :
                    String(format: "%.1f", (Double(rollWaypoints.last?.time ?? "0") ?? 0) + 1.0)
                rollWaypoints.append((time: defaultTime, angle: "0", mode: 0))
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
        sPidKp = formatDecimal(Double(p.pidKp))
        sPidKi = formatDecimal(Double(p.pidKi))
        sPidKd = formatDecimal(Double(p.pidKd))
        sPidMinCmd = formatDecimal(Double(p.pidMinCmd))
        sPidMaxCmd = formatDecimal(Double(p.pidMaxCmd))
        sRollDelayMs = formatInt(Double(p.rollDelayMs))
        sRateCapDps = formatInt(Double(p.rateCapDps))
        rollWaypoints = p.rollWaypoints.map {
            (time: trimFloat($0.timeSeconds), angle: trimFloat($0.angleDeg), mode: $0.mode.rawValue)
        }
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

        updateProfile {
            $0.servoBias1 = b1; $0.servoBias2 = b2; $0.servoBias3 = b3; $0.servoBias4 = b4
            $0.servoHz = hz; $0.servoMinUs = mn; $0.servoMaxUs = mx
        }
        if device.isConnected {
            device.sendServoConfig(biases: [b1, b2, b3, b4], hz: hz, minUs: mn, maxUs: mx)
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
        let useAngle = profile.useAngleControl
        updateProfile {
            $0.rollDelayMs = delayMs
            $0.rateCapDps = rateCap
        }
        if device.isConnected {
            device.sendRollControlConfig(useAngleControl: useAngle, rollDelayMs: delayMs, rateCapDps: rateCap)
        }
        showApplied($rollControlApplied)
    }

    private func applyRollProfile() {
        let waypoints: [RollWaypoint] = rollWaypoints.map {
            RollWaypoint(timeSeconds: Float($0.time) ?? 0,
                         angleDeg: Float($0.angle) ?? 0,
                         mode: RollSegmentMode(rawValue: $0.mode) ?? .angle)
        }.sorted { $0.timeSeconds < $1.timeSeconds }

        updateProfile { $0.rollWaypoints = waypoints }
        if device.isConnected {
            device.sendRollProfile(waypoints: waypoints.map {
                (time: $0.timeSeconds, angle: $0.angleDeg, mode: $0.mode.rawValue)
            })
        }
        showApplied($rollControlApplied)
    }

    // MARK: - LoRa TX power (base station only)

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
