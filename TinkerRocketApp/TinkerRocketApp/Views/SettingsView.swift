//
//  SettingsView.swift
//  TinkerRocketApp
//
//  Device settings: rocket computer configuration + LoRa radio settings.
//  Rocket settings (sounds, etc.) are relayed via I2C to the FlightComputer.
//  LoRa settings are sent to whichever device is currently connected via BLE.
//

import SwiftUI
import Combine

// MARK: - LoRa UI policy (issue #136)
//
// LoRa configuration is no longer user-editable from the app.  The base
// station boots on the hardcoded rendezvous frequency, runs a noise scan
// once it acquires the rocket, and uses the cmd-10 transactional flow to
// move both ends onto the quietest channel — all without user
// involvement.  This view shows only a read-only "Current frequency" so
// the user can confirm both devices ended up on the same channel.
// Hopping / preset / TX-power controls are gated behind a developer flag
// in firmware (cmd 17 / cmd 10), not exposed here.

// MARK: - SettingsView

struct SettingsView: View {
    @ObservedObject var device: BLEDevice
    @Environment(\.dismiss) var dismiss

    // Persisted selections — Rocket settings
    @AppStorage("rocketSoundsEnabled")  private var soundsEnabled: Bool = false
    @AppStorage("servoControlEnabled")  private var servoControlEnabled: Bool = true
    @AppStorage("gainScheduleEnabled")  private var gainScheduleEnabled: Bool = true
    @AppStorage("useAngleControl")      private var useAngleControl: Bool = false
    @AppStorage("rollDelayMs")          private var rollDelayMs: Double = 0
    @AppStorage("guidanceEnabled")      private var guidanceEnabled: Bool = false
    @AppStorage("cameraType")           private var cameraType: Int = 2  // 0=None, 1=GoPro, 2=RunCam

    // Persisted servo/PID values (synced from rocket on connect, updated on Apply)
    @AppStorage("servoBias1")   private var storedBias1: Double = 85
    @AppStorage("servoBias2")   private var storedBias2: Double = 0
    @AppStorage("servoBias3")   private var storedBias3: Double = 0
    @AppStorage("servoBias4")   private var storedBias4: Double = 0
    @AppStorage("servoHz")      private var storedServoHz: Double = 333
    @AppStorage("servoMinUs")   private var storedServoMinUs: Double = 1250
    @AppStorage("servoMaxUs")   private var storedServoMaxUs: Double = 1750
    @AppStorage("pidKp")        private var storedPidKp: Double = 0.08
    @AppStorage("pidKi")        private var storedPidKi: Double = 0.005
    @AppStorage("pidKd")        private var storedPidKd: Double = 0.003
    @AppStorage("pidMinCmd")    private var storedPidMinCmd: Double = -10.0
    @AppStorage("pidMaxCmd")    private var storedPidMaxCmd: Double = 10.0

    // Editable string fields — no live parsing, parsed only on Apply
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

    // Roll profile waypoints — persisted as JSON string in @AppStorage.
    // mode is the segment mode for the segment STARTING at this waypoint:
    //   0 = ROLL_SEG_ANGLE     -- interpolate angle to next waypoint
    //   1 = ROLL_SEG_NULL_RATE -- hold rate=0; angle field ignored
    @AppStorage("rollProfileJSON") private var rollProfileJSON: String = "[]"
    @State private var rollWaypoints: [(time: String, angle: String, mode: UInt8)] = []

    // Roll control config
    @State private var sRollDelayMs = ""

    // Config "Sent" feedback — transient checkmark in section headers.
    @State private var servoApplied = false
    @State private var pidApplied = false
    @State private var rollControlApplied = false

    // LoRa TX power (base station only).  Hydrated from rocketConfig on
    // readback; user edits are debounced so a rapid Stepper drag fires one
    // Cmd 10 transaction at the final value instead of one per tick.
    @State private var txPowerDbm: Int = 12
    @State private var txPowerSendWork: DispatchWorkItem?

    // Self-apply on focus loss (#144): editable config fields are sent to
    // the rocket when keyboard focus leaves their section, so there's no
    // separate "apply" button to forget.
    @FocusState private var focusedField: EditField?
    @State private var lastFocusedField: EditField?

    /// Current active frequency for the connected device, read from the most
    /// recent config readback.  Falls back to the factory default so the
    /// read-only summary has something to show before the first readback
    /// lands (typically the rendezvous freq, since BS boots there too).
    private var currentFreqMHz: Float {
        device.rocketConfig?.loraFreqMHz ?? 915.0
    }

    /// True when the connected rocket is still initializing (sensors/GPS starting up).
    private var isInitializing: Bool {
        device.isConnected
        && !device.isBaseStation
        && device.telemetry.state == "INITIALIZATION"
    }

    // MARK: - Focus-driven self-apply (#144)

    /// Identifies every editable config text field for @FocusState tracking.
    private enum EditField: Hashable {
        case bias1, bias2, bias3, bias4, servoHz, servoMin, servoMax
        case pidKp, pidKi, pidKd, pidMin, pidMax
        case rollDelay
        case wpTime(Int), wpAngle(Int)
    }

    /// Config groups — every field in a group is pushed together by one
    /// `apply*` call when keyboard focus leaves the group.
    private enum EditGroup {
        case servo, pid, rollControl, rollWaypoints
    }

    private func group(of field: EditField?) -> EditGroup? {
        switch field {
        case .bias1, .bias2, .bias3, .bias4, .servoHz, .servoMin, .servoMax:
            return .servo
        case .pidKp, .pidKi, .pidKd, .pidMin, .pidMax:
            return .pid
        case .rollDelay:
            return .rollControl
        case .wpTime, .wpAngle:
            return .rollWaypoints
        case nil:
            return nil
        }
    }

    private func applyGroup(_ g: EditGroup) {
        switch g {
        case .servo:         applyServoConfig()
        case .pid:           applyPIDConfig()
        case .rollControl:   applyRollControlConfig()
        case .rollWaypoints: applyRollProfile()
        }
    }

    /// Push a group's config when keyboard focus leaves it — to another
    /// group, or to nil when the keyboard dismisses.  This is what makes
    /// every text field "apply on edit" with no separate button (#144).
    private func handleFocusChange(_ newField: EditField?) {
        let previous = lastFocusedField
        lastFocusedField = newField
        if let leftGroup = group(of: previous),
           leftGroup != group(of: newField) {
            applyGroup(leftGroup)
        }
    }

    /// Flush a still-focused field if the view goes away before focus
    /// cleared (e.g. nav-bar Done tapped with the keyboard still up).
    private func flushPendingEdits() {
        if let g = group(of: lastFocusedField) {
            applyGroup(g)
        }
        lastFocusedField = nil
    }

    var body: some View {
        NavigationView {
            Form {
                // Network settings hidden in #136: both ends are forced to
                // the firmware default network ID at boot so the BS and OC
                // can't drift apart silently.  The text-field UI showed the
                // app's @AppStorage "Network ID" — which had nothing to do
                // with what the connected device actually used — and that
                // confused users into thinking they were on the same
                // network when in fact the OC's NVS still carried an old
                // value while the BS had been reset to default.  The
                // user-facing network-name flow returns alongside hopping
                // in #150.

                // Rocket computer settings (only when connected directly to rocket)
                if !device.isBaseStation {
                    Section("Rocket Settings") {
                        Toggle("Enable Sounds", isOn: $soundsEnabled)
                            .onChange(of: soundsEnabled) { newValue in
                                device.sendSoundConfig(enabled: newValue)
                            }
                        Toggle("Enable Servo Control", isOn: $servoControlEnabled)
                            .onChange(of: servoControlEnabled) { newValue in
                                device.sendServoControlConfig(enabled: newValue)
                            }
                        // Hard-iron magnetometer cal entry (issue #96).  Only
                        // shown for direct rocket connections — the cal flow
                        // runs on the FC itself and the OC publishes status
                        // back over the same BLE link.
                        NavigationLink {
                            MagCalView(device: device)
                        } label: {
                            Label("Magnetometer Calibration", systemImage: "location.north.line")
                        }
                        Text("Persists across reboots.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }

                }

                // --- LoRa Frequency (read-only) ---
                //
                // Issue #136: the BS auto-acquires the rocket on the
                // hardcoded rendezvous frequency, runs a noise scan, and
                // moves both ends to the quietest channel via the
                // existing cmd-10 transactional handshake.  Nothing here
                // is user-editable.  We show the current channel so the
                // user can confirm both devices ended up on the same
                // frequency — if they don't match, the BS rolled back
                // and both should be reading the rendezvous (915 MHz).
                if device.isBaseStation {
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

                    // LoRa TX power — adjusts both BS and rocket via the
                    // existing Cmd 10 transactional flow.  Stepper changes
                    // are debounced; the BS firmware relays the new power
                    // to the rocket, verifies a beacon on the new setting,
                    // and rolls back if the rocket can't be heard.
                    Section(header: Text("LoRa TX Power"),
                            footer: txPowerFooter) {
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
                        .onChange(of: txPowerDbm) { newValue in
                            scheduleTxPowerSend(newValue)
                        }
                    }
                }

                // --- Servo & PID Settings (rocket only) ---

                if !device.isBaseStation {
                    // Servo biases + timing share one config message, so
                    // they live in one section that self-applies when focus
                    // leaves any of its fields (#144).
                    Section(header: configHeader("Servo", applied: servoApplied)) {
                        stringRow("Servo 1", text: $sBias1, field: .bias1)
                        stringRow("Servo 2", text: $sBias2, field: .bias2)
                        stringRow("Servo 3", text: $sBias3, field: .bias3)
                        stringRow("Servo 4", text: $sBias4, field: .bias4)
                        Text("Microsecond offset per servo to trim mechanical misalignment.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                        stringRow("Frequency", text: $sServoHz, field: .servoHz, unit: "Hz")
                        stringRow("Min Pulse", text: $sServoMinUs, field: .servoMin, unit: "\u{00B5}s")
                        stringRow("Max Pulse", text: $sServoMaxUs, field: .servoMax, unit: "\u{00B5}s")
                    }

                    Section(header: configHeader("PID Gains", applied: pidApplied)) {
                        stringRow("Kp", text: $sPidKp, field: .pidKp, decimal: true)
                        stringRow("Ki", text: $sPidKi, field: .pidKi, decimal: true)
                        stringRow("Kd", text: $sPidKd, field: .pidKd, decimal: true)
                        stringRow("Min Cmd", text: $sPidMinCmd, field: .pidMin, unit: "deg")
                        stringRow("Max Cmd", text: $sPidMaxCmd, field: .pidMax, unit: "deg")
                        Toggle("Velocity Gain Scheduling", isOn: $gainScheduleEnabled)
                            .onChange(of: gainScheduleEnabled) { newValue in
                                device.sendGainScheduleConfig(enabled: newValue)
                            }
                        Text("Scales PID gains with (V_ref/V)\u{00B2}. Disable for fixed gains at all speeds.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }

                    // Control Mode (PN guidance) picker hidden until PN
                    // guidance is flight-validated (#144).  guidanceEnabled
                    // @AppStorage + sendGuidanceConfig() plumbing kept intact.

                    Section("Camera") {
                        Picker("Camera Type", selection: $cameraType) {
                            Text("None").tag(0)
                            Text("GoPro").tag(1)
                            Text("RunCam").tag(2)
                        }
                        .pickerStyle(.segmented)
                        .onChange(of: cameraType) { newValue in
                            device.sendCameraConfig(cameraType: UInt8(newValue))
                        }
                        Text(cameraType == 1
                            ? "GoPro: controlled via GPIO pulse on shutter pin."
                            : cameraType == 2
                            ? "RunCam: controlled via UART serial command."
                            : "No camera connected.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }

                    // Roll Control + Roll Profile unified into one section
                    // (#144).  The mode picker self-applies cmd 31 on change
                    // — pre-#144 the "Use Roll Profile" toggle only flipped
                    // local @AppStorage, so a user could load a profile that
                    // the FC never activated.  The waypoint editor is only
                    // shown in Track Profile mode.
                    Section(header: configHeader("Roll Control", applied: rollControlApplied)) {
                        Picker("Mode", selection: $useAngleControl) {
                            Text("Null Roll").tag(false)
                            Text("Track Profile").tag(true)
                        }
                        .pickerStyle(.segmented)
                        .onChange(of: useAngleControl) { _ in
                            applyRollControlConfig()
                        }
                        Text(useAngleControl
                            ? "Cascaded angle control \u{2014} fins track the roll profile waypoints below."
                            : "Rate-only control \u{2014} fins hold zero roll rate. No profile followed.")
                            .font(.caption)
                            .foregroundColor(.secondary)

                        HStack {
                            Text("Activation Delay")
                            Spacer()
                            TextField("0", text: $sRollDelayMs)
                                .keyboardType(.numberPad)
                                .multilineTextAlignment(.trailing)
                                .frame(width: 80)
                                .focused($focusedField, equals: .rollDelay)
                            Text("ms")
                                .foregroundColor(.secondary)
                        }
                        Text("Milliseconds after launch before any roll control activates. Keeps fins neutral during initial boost.")
                            .font(.caption)
                            .foregroundColor(.secondary)

                        if useAngleControl {
                            Text("Each waypoint defines the START of a segment. Pick the mode per waypoint: Angle interpolates the target roll angle to the next waypoint's angle; Null Rate holds zero roll rate (angle field ignored) for the duration of the segment.")
                                .font(.caption)
                                .foregroundColor(.secondary)

                            ForEach(rollWaypoints.indices, id: \.self) { i in
                                VStack(spacing: 4) {
                                    HStack(spacing: 8) {
                                        Text("WP \(i + 1)")
                                            .font(.caption)
                                            .foregroundColor(.secondary)
                                            .frame(width: 40)
                                        TextField("Time", text: Binding(
                                            get: { rollWaypoints[i].time },
                                            set: { rollWaypoints[i].time = $0 }
                                        ))
                                        .keyboardType(.decimalPad)
                                        .multilineTextAlignment(.trailing)
                                        .frame(width: 60)
                                        .focused($focusedField, equals: .wpTime(i))
                                        Text("s")
                                            .foregroundColor(.secondary)
                                            .frame(width: 15)
                                        TextField("Angle", text: Binding(
                                            get: { rollWaypoints[i].angle },
                                            set: { rollWaypoints[i].angle = $0 }
                                        ))
                                        .keyboardType(.numbersAndPunctuation)
                                        .multilineTextAlignment(.trailing)
                                        .frame(width: 60)
                                        .focused($focusedField, equals: .wpAngle(i))
                                        .disabled(rollWaypoints[i].mode == 1)
                                        .foregroundColor(rollWaypoints[i].mode == 1 ? .secondary : .primary)
                                        Text("\u{00B0}")
                                            .foregroundColor(.secondary)
                                            .frame(width: 15)
                                        Button(role: .destructive) {
                                            rollWaypoints.remove(at: i)
                                            applyRollProfile()
                                        } label: {
                                            Image(systemName: "minus.circle.fill")
                                                .foregroundColor(.red)
                                        }
                                        .buttonStyle(.borderless)
                                    }
                                    Picker("Mode", selection: Binding(
                                        get: { rollWaypoints[i].mode },
                                        set: {
                                            rollWaypoints[i].mode = $0
                                            applyRollProfile()
                                        }
                                    )) {
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
                                    HStack {
                                        Image(systemName: "plus.circle.fill")
                                        Text("Add Waypoint")
                                    }
                                }
                            }

                            Button(role: .destructive) {
                                rollWaypoints.removeAll()
                                saveRollWaypointsToStorage()
                                device.sendRollProfileClear()
                                showApplied($rollControlApplied)
                            } label: {
                                HStack {
                                    Image(systemName: "trash")
                                    Text("Clear Roll Profile")
                                }
                                .frame(maxWidth: .infinity)
                            }
                            .disabled(!device.isConnected)
                        }

                        Text("Persists across reboots.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }
            }
            .navigationTitle("Settings")
            .overlay {
                if isInitializing {
                    ZStack {
                        Color(.systemBackground).opacity(0.9)
                            .ignoresSafeArea()
                        VStack(spacing: 16) {
                            ProgressView()
                                .scaleEffect(1.5)
                            Text("Initializing...")
                                .font(.title2.bold())
                            Text("Waiting for sensors to start up.\nSettings can be applied once ready.")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                                .multilineTextAlignment(.center)
                        }
                        .padding()
                    }
                }
            }
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
            .onAppear {
                loadStringsFromStorage()
                loadRollWaypointsFromStorage()
            }
            .onChange(of: focusedField) { newField in
                handleFocusChange(newField)
            }
            .onDisappear {
                flushPendingEdits()
            }
            // Sync settings from rocket config received on BLE connect
            .onReceive(device.$rocketConfig.compactMap { $0 }) { cfg in
                storedBias1 = Double(cfg.servoBias1)
                storedServoHz = Double(cfg.servoHz)
                storedServoMinUs = Double(cfg.servoMinUs)
                storedServoMaxUs = Double(cfg.servoMaxUs)
                storedPidKp = Double(cfg.pidKp)
                storedPidKi = Double(cfg.pidKi)
                storedPidKd = Double(cfg.pidKd)
                storedPidMinCmd = Double(cfg.pidMinCmd)
                storedPidMaxCmd = Double(cfg.pidMaxCmd)
                servoControlEnabled = cfg.servoEnabled
                gainScheduleEnabled = cfg.gainScheduleEnabled
                useAngleControl = cfg.useAngleControl
                rollDelayMs = Double(cfg.rollDelayMs)
                guidanceEnabled = cfg.guidanceEnabled
                cameraType = Int(cfg.cameraType)
                // LoRa state is displayed read-only via `currentFreqMHz`;
                // SF / BW / CR / hop are still gated behind a developer
                // flag (#136).  TX power is the one exception: the BS GUI
                // exposes a Stepper, hydrated here from the readback.
                if let pwr = cfg.loraTxPower {
                    txPowerDbm = Int(pwr)
                }
                loadStringsFromStorage()
            }
        }
    }

    // MARK: - String ↔ Storage sync

    private func loadStringsFromStorage() {
        sBias1 = formatInt(storedBias1)
        sBias2 = formatInt(storedBias2)
        sBias3 = formatInt(storedBias3)
        sBias4 = formatInt(storedBias4)
        sServoHz = formatInt(storedServoHz)
        sServoMinUs = formatInt(storedServoMinUs)
        sServoMaxUs = formatInt(storedServoMaxUs)
        sPidKp = formatDecimal(storedPidKp)
        sPidKi = formatDecimal(storedPidKi)
        sPidKd = formatDecimal(storedPidKd)
        sPidMinCmd = formatDecimal(storedPidMinCmd)
        sPidMaxCmd = formatDecimal(storedPidMaxCmd)
        sRollDelayMs = formatInt(rollDelayMs)
    }

    private func formatInt(_ v: Double) -> String {
        v == v.rounded() ? String(Int(v)) : String(v)
    }

    private func formatDecimal(_ v: Double) -> String {
        // Show enough precision for PID gains (e.g. 0.005)
        let s = String(format: "%.6f", v)
        // Trim trailing zeros but keep at least one decimal
        var trimmed = s
        while trimmed.hasSuffix("0") && !trimmed.hasSuffix(".0") {
            trimmed.removeLast()
        }
        return trimmed
    }

    private func parseDouble(_ s: String, fallback: Double) -> Double {
        Double(s) ?? fallback
    }

    // MARK: - Helpers

    private func stringRow(_ label: String, text: Binding<String>, field: EditField, unit: String? = nil, decimal: Bool = false) -> some View {
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

    /// Section header with a transient "Sent" checkmark, shown for ~2 s
    /// after the section's config is pushed to the rocket (#144).
    private func configHeader(_ title: String, applied: Bool) -> some View {
        HStack {
            Text(title)
            if applied {
                Spacer()
                Label("Sent", systemImage: "checkmark.circle.fill")
                    .font(.caption)
                    .foregroundColor(.green)
            }
        }
    }

    // MARK: - Apply actions

    private func applyServoConfig() {
        let b1 = parseDouble(sBias1, fallback: storedBias1)
        let b2 = parseDouble(sBias2, fallback: storedBias2)
        let b3 = parseDouble(sBias3, fallback: storedBias3)
        let b4 = parseDouble(sBias4, fallback: storedBias4)
        let hz = parseDouble(sServoHz, fallback: storedServoHz)
        let mn = parseDouble(sServoMinUs, fallback: storedServoMinUs)
        let mx = parseDouble(sServoMaxUs, fallback: storedServoMaxUs)

        // Persist parsed values
        storedBias1 = b1; storedBias2 = b2; storedBias3 = b3; storedBias4 = b4
        storedServoHz = hz; storedServoMinUs = mn; storedServoMaxUs = mx

        device.sendServoConfig(
            biases: [Int16(b1), Int16(b2), Int16(b3), Int16(b4)],
            hz: Int16(hz),
            minUs: Int16(mn),
            maxUs: Int16(mx)
        )
        showApplied($servoApplied)
    }

    private func applyPIDConfig() {
        let kp = parseDouble(sPidKp, fallback: storedPidKp)
        let ki = parseDouble(sPidKi, fallback: storedPidKi)
        let kd = parseDouble(sPidKd, fallback: storedPidKd)
        let mn = parseDouble(sPidMinCmd, fallback: storedPidMinCmd)
        let mx = parseDouble(sPidMaxCmd, fallback: storedPidMaxCmd)

        // Persist parsed values
        storedPidKp = kp; storedPidKi = ki; storedPidKd = kd
        storedPidMinCmd = mn; storedPidMaxCmd = mx

        device.sendPIDConfig(
            kp: Float(kp),
            ki: Float(ki),
            kd: Float(kd),
            minCmd: Float(mn),
            maxCmd: Float(mx)
        )
        showApplied($pidApplied)
    }

    private func applyRollControlConfig() {
        let delayMs = UInt16(clamping: Int(Double(sRollDelayMs) ?? rollDelayMs))
        rollDelayMs = Double(delayMs)
        device.sendRollControlConfig(useAngleControl: useAngleControl, rollDelayMs: delayMs)
        showApplied($rollControlApplied)
    }

    // MARK: - LoRa TX power (base station only)

    /// Debounce Stepper edits so a held +/- doesn't queue a Cmd 10 per
    /// tick — the firmware refuses overlapping transactions and the LoRa
    /// link is throughput-limited.  Skips the send if the new value
    /// matches what the BS already reports, so hydration from rocketConfig
    /// doesn't echo back as a no-op transaction.
    private func scheduleTxPowerSend(_ newValue: Int) {
        txPowerSendWork?.cancel()
        let work = DispatchWorkItem {
            let current = device.rocketConfig?.loraTxPower.map(Int.init) ?? Int.min
            if newValue != current {
                device.autoApplyTxPower(Int8(newValue))
            }
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

    private func applyRollProfile() {
        var waypoints: [(time: Float, angle: Float, mode: UInt8)] = []
        for wp in rollWaypoints {
            let t = Float(wp.time) ?? 0.0
            let a = Float(wp.angle) ?? 0.0
            waypoints.append((time: t, angle: a, mode: wp.mode))
        }
        // Sort by time
        waypoints.sort { $0.time < $1.time }
        device.sendRollProfile(waypoints: waypoints)
        saveRollWaypointsToStorage()
        showApplied($rollControlApplied)
    }

    // MARK: - Roll Profile Persistence

    private func loadRollWaypointsFromStorage() {
        guard let data = rollProfileJSON.data(using: .utf8),
              let decoded = try? JSONDecoder().decode([[String]].self, from: data) else {
            rollWaypoints = []
            return
        }
        // Triples are [time, angle, mode]; tolerate older [time, angle] pairs
        // by defaulting mode to "0" (ROLL_SEG_ANGLE) for backward compat.
        rollWaypoints = decoded.map { entry in
            let t = entry.count > 0 ? entry[0] : "0"
            let a = entry.count > 1 ? entry[1] : "0"
            let m = (entry.count > 2 ? UInt8(entry[2]) : nil) ?? 0
            return (time: t, angle: a, mode: m)
        }
    }

    private func saveRollWaypointsToStorage() {
        let encoded: [[String]] = rollWaypoints.map {
            [String($0.time), String($0.angle), String($0.mode)]
        }
        if let data = try? JSONEncoder().encode(encoded),
           let json = String(data: data, encoding: .utf8) {
            rollProfileJSON = json
        }
    }

    private func showApplied(_ flag: Binding<Bool>) {
        // Only flash "Sent" when there's actually a rocket to send to —
        // a false confirmation is the #144 anti-pattern in miniature.
        guard device.isConnected else { return }
        flag.wrappedValue = true
        DispatchQueue.main.asyncAfter(deadline: .now() + 2.0) {
            flag.wrappedValue = false
        }
    }
}
