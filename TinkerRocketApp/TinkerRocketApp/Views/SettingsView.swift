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

// MARK: - Channel & Preset Definitions

struct LoRaChannel: Identifiable {
    let id: Int
    let label: String
    let freqMHz: Float
}

struct LoRaPreset: Identifiable {
    let id: Int
    let name: String
    let sf: UInt8
    let bwKHz: Float
    let approxToA: String      // Time on air for 49-byte payload
    let maxTxHz: String        // Max TX rate
    let approxRange: String    // Approximate LOS range
}

private let loraChannels: [LoRaChannel] = [
    LoRaChannel(id: 0, label: "Ch 1 \u{00B7} 903.0 MHz", freqMHz: 903.0),
    LoRaChannel(id: 1, label: "Ch 2 \u{00B7} 904.6 MHz", freqMHz: 904.6),
    LoRaChannel(id: 2, label: "Ch 3 \u{00B7} 906.2 MHz", freqMHz: 906.2),
    LoRaChannel(id: 3, label: "Ch 4 \u{00B7} 907.8 MHz", freqMHz: 907.8),
    LoRaChannel(id: 4, label: "Ch 5 \u{00B7} 909.4 MHz", freqMHz: 909.4),
    LoRaChannel(id: 5, label: "Ch 6 \u{00B7} 911.0 MHz", freqMHz: 911.0),
    LoRaChannel(id: 6, label: "Ch 7 \u{00B7} 912.6 MHz", freqMHz: 912.6),
    LoRaChannel(id: 7, label: "Ch 8 \u{00B7} 914.2 MHz", freqMHz: 914.2),
    LoRaChannel(id: 8, label: "Ch 9 \u{00B7} 915.0 MHz", freqMHz: 915.0),
]

private let loraPresets: [LoRaPreset] = [
    LoRaPreset(id: 0, name: "Fast",       sf: 7,  bwKHz: 500.0, approxToA: "25 ms",  maxTxHz: "25 Hz", approxRange: "~3 km"),
    LoRaPreset(id: 1, name: "Standard",   sf: 8,  bwKHz: 250.0, approxToA: "91 ms",  maxTxHz: "10 Hz", approxRange: "~8 km"),
    LoRaPreset(id: 2, name: "Balanced",   sf: 9,  bwKHz: 250.0, approxToA: "165 ms", maxTxHz: "5 Hz",  approxRange: "~12 km"),
    LoRaPreset(id: 3, name: "Long Range", sf: 10, bwKHz: 250.0, approxToA: "330 ms", maxTxHz: "2 Hz",  approxRange: "~18 km"),
    LoRaPreset(id: 4, name: "Max Range",  sf: 9,  bwKHz: 125.0, approxToA: "330 ms", maxTxHz: "2 Hz",  approxRange: "~25 km"),
]

// MARK: - SettingsView

struct SettingsView: View {
    @ObservedObject var bleManager: BLEManager
    @Environment(\.dismiss) var dismiss

    // Persisted selections — LoRa
    @AppStorage("loraChannelId")        private var channelId: Int = 8          // Default: Ch 9 (915.0 MHz)
    @AppStorage("loraPresetId")         private var presetId: Int = 1           // Default: Standard
    @AppStorage("loraTxPower")          private var txPower: Double = 12.0      // Default: 12 dBm

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

    // Roll profile waypoints — persisted as JSON string in @AppStorage
    @AppStorage("rollProfileJSON") private var rollProfileJSON: String = "[]"
    @State private var rollWaypoints: [(time: String, angle: String)] = []
    @State private var rollProfileApplied = false

    // Roll control config
    @State private var sRollDelayMs = ""
    @State private var rollControlApplied = false

    // Apply button feedback
    @State private var loraApplied = false
    @State private var servoApplied = false
    @State private var pidApplied = false

    private var selectedChannel: LoRaChannel {
        loraChannels.first(where: { $0.id == channelId }) ?? loraChannels[8]
    }

    private var selectedPreset: LoRaPreset {
        loraPresets.first(where: { $0.id == presetId }) ?? loraPresets[1]
    }

    /// True when the connected rocket is still initializing (sensors/GPS starting up).
    private var isInitializing: Bool {
        bleManager.isConnected
        && !bleManager.isBaseStation
        && bleManager.telemetry.state == "INITIALIZATION"
    }

    var body: some View {
        NavigationView {
            Form {
                // Rocket computer settings (only when connected directly to rocket)
                if !bleManager.isBaseStation {
                    Section("Rocket Settings") {
                        Toggle("Enable Sounds", isOn: $soundsEnabled)
                            .onChange(of: soundsEnabled) { newValue in
                                bleManager.sendSoundConfig(enabled: newValue)
                            }
                        Toggle("Enable Servo Control", isOn: $servoControlEnabled)
                            .onChange(of: servoControlEnabled) { newValue in
                                bleManager.sendServoControlConfig(enabled: newValue)
                            }
                        Text("Persists across reboots.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }

                }

                // --- LoRa Settings ---

                // Channel picker (default Form style = navigation link, avoids
                // UIContextMenuInteraction issues that .menu causes in sheets)
                Section("LoRa Channel") {
                    Picker("Frequency", selection: $channelId) {
                        ForEach(loraChannels) { ch in
                            Text(ch.label).tag(ch.id)
                        }
                    }
                }

                // Preset picker
                Section("LoRa Range / Rate Preset") {
                    Picker("Preset", selection: $presetId) {
                        ForEach(loraPresets) { preset in
                            VStack(alignment: .leading, spacing: 2) {
                                Text(preset.name)
                                Text("\(preset.approxRange) \u{00B7} \(preset.maxTxHz) \u{00B7} \(preset.approxToA)")
                                    .font(.caption)
                                    .foregroundColor(.secondary)
                            }
                            .tag(preset.id)
                        }
                    }
                    .pickerStyle(.inline)
                    .labelsHidden()
                }

                // TX Power slider
                Section("LoRa TX Power") {
                    VStack {
                        HStack {
                            Text("Power")
                            Spacer()
                            Text("\(Int(txPower)) dBm")
                                .foregroundColor(.secondary)
                                .font(.system(.body, design: .monospaced))
                        }
                        Slider(value: $txPower, in: 2...22, step: 1)
                    }
                }

                // Summary
                Section("LoRa Summary") {
                    infoRow(label: "Frequency", value: String(format: "%.1f MHz", selectedChannel.freqMHz))
                    infoRow(label: "Spreading Factor", value: "SF\(selectedPreset.sf)")
                    infoRow(label: "Bandwidth", value: String(format: "%.0f kHz", selectedPreset.bwKHz))
                    infoRow(label: "Coding Rate", value: "4/5")
                    infoRow(label: "TX Power", value: "\(Int(txPower)) dBm")
                    infoRow(label: "Time on Air", value: selectedPreset.approxToA)
                    infoRow(label: "Est. Range (LOS)", value: selectedPreset.approxRange)
                }

                // Apply LoRa button
                Section {
                    applyButton(
                        icon: "antenna.radiowaves.left.and.right",
                        label: "Apply LoRa Config to \(bleManager.isBaseStation ? "Base Station" : "Rocket")",
                        applied: loraApplied
                    ) {
                        applyLoRaConfig()
                    }

                    Text("Both devices must use matching LoRa settings. Connect to each device separately and apply the same config.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }

                // --- Servo & PID Settings (rocket only) ---

                if !bleManager.isBaseStation {
                    Section("Servo Biases (\u{00B5}s)") {
                        stringRow("Servo 1", text: $sBias1)
                        stringRow("Servo 2", text: $sBias2)
                        stringRow("Servo 3", text: $sBias3)
                        stringRow("Servo 4", text: $sBias4)
                        Text("Microsecond offset per servo to trim mechanical misalignment.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }

                    Section("Servo Timing") {
                        stringRow("Frequency", text: $sServoHz, unit: "Hz")
                        stringRow("Min Pulse", text: $sServoMinUs, unit: "\u{00B5}s")
                        stringRow("Max Pulse", text: $sServoMaxUs, unit: "\u{00B5}s")
                        applyButton(
                            icon: "gearshape.2",
                            label: "Apply Servo Config",
                            applied: servoApplied
                        ) {
                            applyServoConfig()
                        }
                    }

                    Section("PID Gains") {
                        stringRow("Kp", text: $sPidKp, decimal: true)
                        stringRow("Ki", text: $sPidKi, decimal: true)
                        stringRow("Kd", text: $sPidKd, decimal: true)
                        stringRow("Min Cmd", text: $sPidMinCmd, unit: "deg")
                        stringRow("Max Cmd", text: $sPidMaxCmd, unit: "deg")
                        Toggle("Velocity Gain Scheduling", isOn: $gainScheduleEnabled)
                            .onChange(of: gainScheduleEnabled) { newValue in
                                bleManager.sendGainScheduleConfig(enabled: newValue)
                            }
                        Text("Scales PID gains with (V_ref/V)\u{00B2}. Disable for fixed gains at all speeds.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                        applyButton(
                            icon: "slider.horizontal.3",
                            label: "Apply PID Config",
                            applied: pidApplied
                        ) {
                            applyPIDConfig()
                        }
                    }

                    Section("Control Mode") {
                        Picker("Flight Mode", selection: $guidanceEnabled) {
                            Text("Roll Only").tag(false)
                            Text("Roll + Guidance").tag(true)
                        }
                        .pickerStyle(.segmented)
                        .onChange(of: guidanceEnabled) { newValue in
                            bleManager.sendGuidanceConfig(enabled: newValue)
                        }
                        Text(guidanceEnabled
                            ? "Roll control during boost, PN guidance from coast to apogee. Guidance activates after burnout + delay, stops at closest approach."
                            : "Roll rate control only during all flight phases. No lateral steering.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                        Text("Persists across reboots.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }

                    Section("Camera") {
                        Picker("Camera Type", selection: $cameraType) {
                            Text("None").tag(0)
                            Text("GoPro").tag(1)
                            Text("RunCam").tag(2)
                        }
                        .pickerStyle(.segmented)
                        .onChange(of: cameraType) { newValue in
                            bleManager.sendCameraConfig(cameraType: UInt8(newValue))
                        }
                        Text(cameraType == 1
                            ? "GoPro: controlled via GPIO pulse on shutter pin."
                            : cameraType == 2
                            ? "RunCam: controlled via UART serial command."
                            : "No camera connected.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }

                    Section("Roll Control") {
                        Toggle("Use Roll Profile", isOn: $useAngleControl)
                        Text("When enabled, uses cascaded angle control with roll profile waypoints below. When disabled, uses rate-only control (null roll).")
                            .font(.caption)
                            .foregroundColor(.secondary)

                        HStack {
                            Text("Activation Delay")
                            Spacer()
                            TextField("0", text: $sRollDelayMs)
                                .keyboardType(.numberPad)
                                .multilineTextAlignment(.trailing)
                                .frame(width: 80)
                            Text("ms")
                                .foregroundColor(.secondary)
                        }
                        Text("Milliseconds after launch before any roll control activates. Keeps fins neutral during initial boost.")
                            .font(.caption)
                            .foregroundColor(.secondary)

                        applyButton(
                            icon: "arrow.triangle.turn.up.right.diamond",
                            label: "Apply Roll Control",
                            applied: rollControlApplied
                        ) {
                            applyRollControlConfig()
                        }
                    }

                    Section("Roll Profile") {
                        Text("Define (time, angle) waypoints for the cascaded angle controller. During flight, the target roll angle is interpolated between waypoints.")
                            .font(.caption)
                            .foregroundColor(.secondary)

                        ForEach(rollWaypoints.indices, id: \.self) { i in
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
                                Text("\u{00B0}")
                                    .foregroundColor(.secondary)
                                    .frame(width: 15)
                                Button(role: .destructive) {
                                    rollWaypoints.remove(at: i)
                                    saveRollWaypointsToStorage()
                                } label: {
                                    Image(systemName: "minus.circle.fill")
                                        .foregroundColor(.red)
                                }
                                .buttonStyle(.borderless)
                            }
                        }

                        if rollWaypoints.count < 8 {
                            Button {
                                let defaultTime = rollWaypoints.isEmpty ? "0.0" :
                                    String(format: "%.1f", (Double(rollWaypoints.last?.time ?? "0") ?? 0) + 1.0)
                                rollWaypoints.append((time: defaultTime, angle: "0"))
                                saveRollWaypointsToStorage()
                            } label: {
                                HStack {
                                    Image(systemName: "plus.circle.fill")
                                    Text("Add Waypoint")
                                }
                            }
                        }

                        applyButton(
                            icon: "arrow.triangle.turn.up.right.diamond",
                            label: "Send Roll Profile",
                            applied: rollProfileApplied
                        ) {
                            applyRollProfile()
                        }

                        Button(role: .destructive) {
                            rollWaypoints.removeAll()
                            saveRollWaypointsToStorage()
                            bleManager.sendRollProfileClear()
                            showApplied($rollProfileApplied)
                        } label: {
                            HStack {
                                Image(systemName: "trash")
                                Text("Clear Roll Profile")
                            }
                            .frame(maxWidth: .infinity)
                        }
                        .disabled(!bleManager.isConnected)
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
            // Sync settings from rocket config received on BLE connect
            .onReceive(bleManager.$rocketConfig.compactMap { $0 }) { cfg in
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
                // Sync LoRa settings from device (reverse-map raw values to channel/preset IDs)
                if let freq = cfg.loraFreqMHz {
                    if let ch = loraChannels.first(where: { abs($0.freqMHz - freq) < 0.1 }) {
                        channelId = ch.id
                    }
                }
                if let sf = cfg.loraSF, let bw = cfg.loraBwKHz {
                    if let preset = loraPresets.first(where: { $0.sf == sf && abs($0.bwKHz - bw) < 1.0 }) {
                        presetId = preset.id
                    }
                }
                if let pwr = cfg.loraTxPower {
                    txPower = Double(pwr)
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

    private func infoRow(label: String, value: String) -> some View {
        HStack {
            Text(label)
            Spacer()
            Text(value)
                .foregroundColor(.secondary)
        }
    }

    private func stringRow(_ label: String, text: Binding<String>, unit: String? = nil, decimal: Bool = false) -> some View {
        HStack {
            Text(label)
            Spacer()
            TextField(decimal ? "0.0" : "0", text: text)
                .keyboardType(decimal ? .decimalPad : .numbersAndPunctuation)
                .multilineTextAlignment(.trailing)
                .frame(width: 80)
            if let unit = unit {
                Text(unit).foregroundColor(.secondary).fixedSize().frame(minWidth: 30, alignment: .leading)
            }
        }
    }

    @ViewBuilder
    private func applyButton(icon: String, label: String, applied: Bool, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            HStack {
                Image(systemName: applied ? "checkmark.circle.fill" : icon)
                Text(applied ? "Sent!" : label)
            }
            .frame(maxWidth: .infinity)
            .foregroundColor(applied ? .green : nil)
        }
        .disabled(!bleManager.isConnected || applied)
    }

    // MARK: - Apply actions

    private func applyLoRaConfig() {
        let channel = selectedChannel
        let preset = selectedPreset
        let cr: UInt8 = 5  // All presets use CR 4/5

        bleManager.sendLoRaConfig(
            freqMHz: channel.freqMHz,
            bwKHz: preset.bwKHz,
            sf: preset.sf,
            cr: cr,
            txPower: Int8(txPower)
        )
        showApplied($loraApplied)
    }

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

        bleManager.sendServoConfig(
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

        bleManager.sendPIDConfig(
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
        bleManager.sendRollControlConfig(useAngleControl: useAngleControl, rollDelayMs: delayMs)
        showApplied($rollControlApplied)
    }

    private func applyRollProfile() {
        var waypoints: [(time: Float, angle: Float)] = []
        for wp in rollWaypoints {
            let t = Float(wp.time) ?? 0.0
            let a = Float(wp.angle) ?? 0.0
            waypoints.append((time: t, angle: a))
        }
        // Sort by time
        waypoints.sort { $0.time < $1.time }
        bleManager.sendRollProfile(waypoints: waypoints)
        saveRollWaypointsToStorage()
        showApplied($rollProfileApplied)
    }

    // MARK: - Roll Profile Persistence

    private func loadRollWaypointsFromStorage() {
        guard let data = rollProfileJSON.data(using: .utf8),
              let decoded = try? JSONDecoder().decode([[String]].self, from: data) else {
            rollWaypoints = []
            return
        }
        rollWaypoints = decoded.map { pair in
            (time: pair.count > 0 ? pair[0] : "0", angle: pair.count > 1 ? pair[1] : "0")
        }
    }

    private func saveRollWaypointsToStorage() {
        let encoded: [[String]] = rollWaypoints.map { [String($0.time), String($0.angle)] }
        if let data = try? JSONEncoder().encode(encoded),
           let json = String(data: data, encoding: .utf8) {
            rollProfileJSON = json
        }
    }

    private func showApplied(_ flag: Binding<Bool>) {
        flag.wrappedValue = true
        DispatchQueue.main.asyncAfter(deadline: .now() + 2.0) {
            flag.wrappedValue = false
        }
    }
}
