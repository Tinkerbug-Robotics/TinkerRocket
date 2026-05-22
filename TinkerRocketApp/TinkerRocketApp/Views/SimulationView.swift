//
//  SimulationView.swift
//  TinkerRocketApp
//
//  Configures and controls the ESP32 flight simulator.
//  User enters rocket mass, motor thrust, and burn time.
//  Estimated performance is computed locally before uploading to ESP32.
//

import SwiftUI

struct SimulationView: View {
    @ObservedObject var device: BLEDevice
    @Environment(\.dismiss) var dismiss

    // User inputs (persisted across sessions)
    @AppStorage("simMassGrams") private var massGrams: String = "500"
    @AppStorage("simThrustNewtons") private var thrustNewtons: String = "40"
    @AppStorage("simBurnTimeSeconds") private var burnTimeSeconds: String = "1.5"
    @AppStorage("simDescentRateMps") private var descentRateMps: String = "5.0"

    @AppStorage("unitSystem") private var unitSystem: UnitSystem = .metric
    // Editable descent-rate text in the current display unit (m/s or ft/s).
    // The persisted store above stays canonical m/s; this is synced on appear
    // and whenever the unit system changes, and committed back on edit.
    @State private var descentRateText: String = ""

    /// Whether all input fields parse to valid positive numbers
    private var inputsValid: Bool {
        guard let m = Float(massGrams), m > 0,
              let t = Float(thrustNewtons), t > 0,
              let b = Float(burnTimeSeconds), b > 0,
              let d = Float(descentRateMps), d > 0 else {
            return false
        }
        return true
    }

    var body: some View {
        NavigationView {
            Form {
                Section("Rocket Parameters") {
                    parameterRow(label: "Mass", value: $massGrams, unit: "g", placeholder: "grams")
                    parameterRow(label: "Thrust", value: $thrustNewtons, unit: "N", placeholder: "newtons")
                    parameterRow(label: "Burn Time", value: $burnTimeSeconds, unit: "s", placeholder: "seconds")
                    parameterRow(label: "Descent Rate", value: $descentRateText,
                                 unit: UnitFormatter.speedUnit(unitSystem),
                                 placeholder: UnitFormatter.speedUnit(unitSystem))
                }

                Section("Estimated Performance") {
                    let est = estimatedPerformance()
                    infoRow(label: "Burnout Speed", value: est.maxSpeed)
                    infoRow(label: "Max Altitude", value: est.maxAlt)
                    infoRow(label: "Flight Duration", value: est.flightTime)
                }

                Section {
                    Button(action: launchSim) {
                        Label("Launch Simulation", systemImage: "flame")
                            .foregroundColor(.orange)
                    }
                    .disabled(!device.isConnected || !inputsValid)

                    if !device.isConnected {
                        Text("Not connected to rocket")
                            .font(.caption)
                            .foregroundColor(.red)
                    } else if !inputsValid {
                        Text("All fields must be positive numbers")
                            .font(.caption)
                            .foregroundColor(.red)
                    }
                }

                Section {
                    Text("The simulator runs a 1D physics model on the ESP32, generating synthetic sensor data through the full telemetry pipeline. Voice announcements, LoRa, and data logging all work during simulation.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            .navigationTitle("Simulation Settings")
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") { dismiss() }
                }
            }
            .onAppear { syncDescentRateText() }
            .onChange(of: descentRateText) { _ in commitDescentRate() }
            .onChange(of: unitSystem) { _ in syncDescentRateText() }
        }
    }

    // MARK: - Descent-rate unit sync (#160)

    /// Load the editable text from the canonical m/s store, in the current unit.
    private func syncDescentRateText() {
        guard let mps = Double(descentRateMps) else { descentRateText = descentRateMps; return }
        descentRateText = formatRate(UnitFormatter.mpsToDisplay(mps, system: unitSystem))
    }

    /// Commit the editable text back to the canonical m/s store.  Invalid /
    /// empty input clears the store so `inputsValid` fails, as before.
    private func commitDescentRate() {
        guard let display = Double(descentRateText) else { descentRateMps = ""; return }
        descentRateMps = String(UnitFormatter.displayToMps(display, system: unitSystem))
    }

    /// Trim a converted rate to at most 2 decimals with no trailing zeros.
    private func formatRate(_ value: Double) -> String {
        var s = String(format: "%.2f", value)
        while s.hasSuffix("0") { s.removeLast() }
        if s.hasSuffix(".") { s.removeLast() }
        return s
    }

    // MARK: - UI Helpers

    private func parameterRow(label: String, value: Binding<String>, unit: String, placeholder: String) -> some View {
        HStack {
            Text(label)
            Spacer()
            TextField(placeholder, text: value)
                .keyboardType(.decimalPad)
                .multilineTextAlignment(.trailing)
                .frame(width: 80)
            Text(unit)
                .foregroundColor(.secondary)
                .fixedSize()
                .frame(minWidth: 28, alignment: .leading)
        }
    }

    private func infoRow(label: String, value: String) -> some View {
        HStack {
            Text(label)
            Spacer()
            Text(value)
                .foregroundColor(.secondary)
        }
    }

    // MARK: - Actions

    /// Send config then start sim, and dismiss to dashboard
    private func launchSim() {
        guard let massG = Float(massGrams),
              let thrustN = Float(thrustNewtons),
              let burnS = Float(burnTimeSeconds),
              let descentR = Float(descentRateMps) else {
            return
        }

        // Pack 4 floats as little-endian bytes: [mass_g:4][thrust_n:4][burn_s:4][descent_rate_mps:4]
        var payload = Data()
        var m = massG
        var t = thrustN
        var b = burnS
        var d = descentR
        payload.append(Data(bytes: &m, count: 4))
        payload.append(Data(bytes: &t, count: 4))
        payload.append(Data(bytes: &b, count: 4))
        payload.append(Data(bytes: &d, count: 4))

        // Send config (command 5), then start (command 6) after a delay.
        // Use 1s when relaying via base station LoRa (uplink retries take ~300ms)
        // vs 0.3s for direct BLE to rocket.
        let delay: Double = device.isBaseStation ? 1.0 : 0.3
        device.sendTimeSync()  // Fresh phone time for unique sim filenames
        device.sendRawCommand(5, payload: payload)

        DispatchQueue.main.asyncAfter(deadline: .now() + delay) {
            device.markSimLaunched()
            device.sendCommand(6)
            dismiss()  // Return to dashboard — countdown tile will appear
        }
    }

    // MARK: - Estimated Performance (local preview)

    private func estimatedPerformance() -> (maxSpeed: String, maxAlt: String, flightTime: String) {
        guard let massG = Float(massGrams), massG > 0,
              let thrustN = Float(thrustNewtons), thrustN > 0,
              let burnS = Float(burnTimeSeconds), burnS > 0,
              let descentR = Float(descentRateMps), descentR > 0 else {
            return (maxSpeed: "\u{2014}", maxAlt: "\u{2014}", flightTime: "\u{2014}")
        }

        let massKg = massG / 1000.0
        let g: Float = 9.80665
        let cd: Float = 0.5
        let area: Float = 0.0019635  // pi * 0.025^2

        // Simple Euler integration
        var alt: Float = 0
        var vel: Float = 0
        var maxSpeed: Float = 0
        var time: Float = 0
        let dt: Float = 0.01

        // Powered phase
        while time < burnS {
            let rho = airDensityEstimate(alt)
            let dragF = 0.5 * cd * area * rho * vel * vel
            let dragA = dragF / massKg
            let accel = (thrustN / massKg) - g - (vel > 0 ? dragA : -dragA)
            vel += accel * dt
            alt += vel * dt
            if alt < 0 { alt = 0 }
            if vel > maxSpeed { maxSpeed = vel }
            time += dt
        }

        // Coasting phase
        while vel > 0 && time < 300 {
            let rho = airDensityEstimate(alt)
            let dragF = 0.5 * cd * area * rho * vel * vel
            let dragA = dragF / massKg
            let accel = -g - dragA
            vel += accel * dt
            alt += vel * dt
            time += dt
        }

        let maxAlt = alt
        let descentTime = maxAlt / descentR
        let totalTime = time + descentTime

        return (
            maxSpeed: UnitFormatter.speed(Double(maxSpeed), decimals: 0, system: unitSystem),
            maxAlt: UnitFormatter.altitude(Double(maxAlt), system: unitSystem),
            flightTime: String(format: "%.0f s", totalTime)
        )
    }

    private func airDensityEstimate(_ alt: Float) -> Float {
        let x = 1.0 - 2.2558e-5 * alt
        return x > 0 ? 1.225 * pow(x, 4.2559) : 0
    }
}
