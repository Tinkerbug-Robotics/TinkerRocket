//
//  ServoTestView.swift
//  TinkerRocketApp
//
//  Manual servo test mode. Presents sliders for each servo (-20° to +20°).
//  Angles are sent to the rocket in real time. On dismiss, servos return
//  to their midpoints.
//

import SwiftUI

struct ServoTestView: View {
    @ObservedObject var device: BLEDevice
    @Environment(\.dismiss) var dismiss

    @State private var angles: [Double] = [0, 0, 0, 0]

    private let range: ClosedRange<Double> = -20...20
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
