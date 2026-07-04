//
//  ServoTestView.swift
//  TinkerRocketApp
//
//  Manual servo test mode. Presents a slider per servo spanning the active
//  profile's fin calibration (servo_min_us<->finMinDeg, servo_max_us<->
//  finMaxDeg), so the test can command the full mechanical travel — different
//  models use different servos/cals, so the range is per-profile, not a fixed
//  ±20. Angles are sent to the rocket in real time. On dismiss, servos return
//  to their midpoints.
//

import SwiftUI

struct ServoTestView: View {
    @ObservedObject var device: BLEDevice
    @Environment(\.dismiss) var dismiss

    // Fin-angle endpoints the test may command, from the active profile's fin
    // calibration. Falls back to ±20° when no calibration is available.
    var finMinDeg: Double = -20
    var finMaxDeg: Double = 20

    @State private var angles: [Double] = [0, 0, 0, 0]

    private var range: ClosedRange<Double> {
        let lo = Swift.min(finMinDeg, finMaxDeg)
        let hi = Swift.max(finMinDeg, finMaxDeg)
        return lo < hi ? lo...hi : -20...20
    }
    private let step: Double = 0.5

    var body: some View {
        NavigationView {
            Form {
                Section(header: Text("Servo Angles")) {
                    ForEach(0..<4, id: \.self) { index in
                        VStack(alignment: .leading, spacing: 4) {
                            HStack {
                                Text("Servo \(index + 1)")
                                    .font(.headline)
                                Spacer()
                                Text(String(format: "%.1f°", angles[index]))
                                    .font(.system(.body, design: .monospaced))
                                    .foregroundColor(.secondary)
                            }
                            Slider(
                                value: $angles[index],
                                in: range,
                                step: step
                            ) {
                                Text("Servo \(index + 1)")
                            }
                            .onChange(of: angles[index]) { _ in
                                sendAngles()
                            }
                        }
                        .padding(.vertical, 4)
                    }
                }

                Section {
                    Button {
                        angles = [0, 0, 0, 0]
                        sendAngles()
                    } label: {
                        HStack {
                            Image(systemName: "arrow.counterclockwise")
                            Text("Center All")
                        }
                        .frame(maxWidth: .infinity)
                        .foregroundColor(.white)
                    }
                    .listRowBackground(Color.blue)
                }
            }
            .navigationTitle("Servo Test")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") {
                        dismiss()
                    }
                }
            }
            .onAppear {
                // Send initial zero angles
                sendAngles()
            }
            .onDisappear {
                // Stop servo test — servos return to midpoints
                device.sendServoTestStop()
            }
        }
    }

    private func sendAngles() {
        device.sendServoTestAngles(angles)
    }
}
