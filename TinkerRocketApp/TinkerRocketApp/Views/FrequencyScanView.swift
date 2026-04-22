//
//  FrequencyScanView.swift
//  TinkerRocketApp
//
//  Pre-launch RF collision-avoidance tool.  Tells the base station to sweep
//  the ISM band, reads the RSSI-per-channel result back over BLE, and plots
//  it as a bar chart so the user can pick a quiet channel.
//

import SwiftUI
import Charts

struct FrequencyScanView: View {
    @ObservedObject var device: BLEDevice
    @Environment(\.dismiss) var dismiss

    @State private var showApplyConfirm = false
    @State private var applyStatus: String? = nil    // transient "Applying…" / "Done" feedback

    // Scan parameters, persisted so the user gets the same range next time.
    @AppStorage("scanStartMHz")  private var startMHz: Double = 902.0
    @AppStorage("scanStopMHz")   private var stopMHz:  Double = 928.0
    @AppStorage("scanStepKHz")   private var stepKHz:  Double = 500.0
    @AppStorage("scanDwellMs")   private var dwellMs:  Double = 30.0

    // Reference ceiling used to convert dBm into a positive "noise margin"
    // that grows with channel quietness: margin = referenceDbm − rssi.
    // −40 dBm is stronger than any real traffic we'll ever see in free air,
    // so margin stays ≥ 0 in practice.
    private let referenceDbm: Int = -40

    // Bars at or above this margin are highlighted green ("comfortable").
    // 70 dB margin corresponds to rssi ≤ −110 dBm.
    private let quietMarginDb: Int = 70

    private func marginDb(_ rssi: Int) -> Int { referenceDbm - rssi }

    var body: some View {
        Form {
            if device.hasAutoSelectedChannel {
                Section {
                    Label {
                        Text("A quiet channel was auto-selected when this base station connected. Run another scan below to pick a new one.")
                            .font(.caption)
                    } icon: {
                        Image(systemName: "checkmark.circle.fill").foregroundColor(.green)
                    }
                }
            }

            Section("Scan Range") {
                HStack {
                    Text("Start")
                    Spacer()
                    TextField("902.0", value: $startMHz, format: .number.precision(.fractionLength(1)))
                        .keyboardType(.decimalPad)
                        .multilineTextAlignment(.trailing)
                        .frame(width: 80)
                    Text("MHz").foregroundColor(.secondary)
                }
                HStack {
                    Text("Stop")
                    Spacer()
                    TextField("928.0", value: $stopMHz, format: .number.precision(.fractionLength(1)))
                        .keyboardType(.decimalPad)
                        .multilineTextAlignment(.trailing)
                        .frame(width: 80)
                    Text("MHz").foregroundColor(.secondary)
                }
                HStack {
                    Text("Step")
                    Spacer()
                    TextField("500", value: $stepKHz, format: .number.precision(.fractionLength(0)))
                        .keyboardType(.numberPad)
                        .multilineTextAlignment(.trailing)
                        .frame(width: 80)
                    Text("kHz").foregroundColor(.secondary)
                }
                HStack {
                    Text("Dwell")
                    Spacer()
                    TextField("30", value: $dwellMs, format: .number.precision(.fractionLength(0)))
                        .keyboardType(.numberPad)
                        .multilineTextAlignment(.trailing)
                        .frame(width: 80)
                    Text("ms").foregroundColor(.secondary)
                }
                HStack {
                    Text("Channels")
                    Spacer()
                    Text("\(estimatedChannels)")
                        .foregroundColor(.secondary)
                        .font(.system(.body, design: .monospaced))
                }
                HStack {
                    Text("Est. Duration")
                    Spacer()
                    Text(estimatedDurationLabel)
                        .foregroundColor(.secondary)
                        .font(.system(.body, design: .monospaced))
                }
            }

            Section {
                Button {
                    device.startFrequencyScan(
                        startMHz: Float(startMHz),
                        stopMHz:  Float(stopMHz),
                        stepKHz:  UInt16(clamping: Int(stepKHz)),
                        dwellMs:  UInt16(clamping: Int(dwellMs))
                    )
                } label: {
                    HStack {
                        Spacer()
                        if device.isScanning {
                            ProgressView().padding(.trailing, 6)
                            Text("Scanning…")
                        } else {
                            Image(systemName: "waveform.badge.magnifyingglass")
                            Text("Start Scan")
                        }
                        Spacer()
                    }
                }
                .disabled(device.isScanning || !paramsValid)
            }

            if !device.scanSamples.isEmpty {
                Section("Result") {
                    chart
                        .frame(height: 240)
                        .padding(.vertical, 4)
                    summary
                }

                if let quiet = quietestSample {
                    Section("Auto-Apply") {
                        Button {
                            showApplyConfirm = true
                        } label: {
                            HStack {
                                Spacer()
                                Image(systemName: "wand.and.stars")
                                Text("Apply \(String(format: "%.2f", quiet.freqMHz)) MHz to Rocket + Base")
                                Spacer()
                            }
                        }
                        .disabled(!canApply || applyStatus != nil)

                        if let s = applyStatus {
                            Text(s).font(.caption).foregroundColor(.secondary)
                        } else if !canApply {
                            Text("Waiting for base-station config readback…")
                                .font(.caption).foregroundColor(.secondary)
                        } else {
                            Text("Relays the new frequency to every tracked rocket, then switches the base station. Keeps current SF/BW/CR/power.")
                                .font(.caption).foregroundColor(.secondary)
                        }

                        if device.remoteRockets.isEmpty {
                            Text("No rockets online — only the base station will be switched.")
                                .font(.caption).foregroundColor(.orange)
                        }
                    }
                }
            }
        }
        .navigationTitle("Frequency Scan")
        .navigationBarTitleDisplayMode(.inline)
        .toolbar {
            ToolbarItem(placement: .navigationBarTrailing) {
                Button("Done") { dismiss() }
            }
        }
        .alert("Switch frequency?", isPresented: $showApplyConfirm) {
            Button("Apply", role: .destructive) { applyQuietest() }
            Button("Cancel", role: .cancel) { }
        } message: {
            if let q = quietestSample {
                Text("The rocket and base station will both switch to \(String(format: "%.2f", q.freqMHz)) MHz. If the rocket misses the relay packet it will be stranded on the old channel.")
            }
        }
    }

    private var canApply: Bool {
        device.isBaseStation && device.rocketConfig?.loraFreqMHz != nil
    }

    private func applyQuietest() {
        guard let q = quietestSample else { return }
        applyStatus = "Applying \(String(format: "%.2f", q.freqMHz)) MHz…"
        let ok = device.autoApplyFrequency(q.freqMHz)
        if !ok {
            applyStatus = "Failed: base-station config unavailable"
            DispatchQueue.main.asyncAfter(deadline: .now() + 3) { applyStatus = nil }
            return
        }
        // Retry window (uplink retries) + BS switch delay + a little slack.
        DispatchQueue.main.asyncAfter(deadline: .now() + 4.0) {
            applyStatus = "Done"
            DispatchQueue.main.asyncAfter(deadline: .now() + 2.0) { applyStatus = nil }
        }
    }

    // MARK: - Derived values

    private var paramsValid: Bool {
        stopMHz > startMHz && stepKHz > 0 && dwellMs > 0 && estimatedChannels <= 128
    }

    private var estimatedChannels: Int {
        guard stepKHz > 0, stopMHz > startMHz else { return 0 }
        let span = (stopMHz - startMHz) * 1000.0
        return Int(span / stepKHz) + 1
    }

    private var estimatedDurationLabel: String {
        let totalMs = Double(estimatedChannels) * (dwellMs + 2.0)   // ~2 ms overhead per step
        if totalMs < 1000 {
            return String(format: "%.0f ms", totalMs)
        }
        return String(format: "%.1f s", totalMs / 1000.0)
    }

    private var quietestSample: FrequencyScanSample? {
        device.scanSamples.min(by: { $0.rssiDbm < $1.rssiDbm })
    }

    @ViewBuilder private var summary: some View {
        if let quiet = quietestSample {
            HStack {
                Text("Quietest channel")
                Spacer()
                Text(String(format: "%.2f MHz  \u{00B7}  %d dB",
                            quiet.freqMHz, marginDb(quiet.rssiDbm)))
                    .foregroundColor(.green)
                    .font(.system(.body, design: .monospaced))
            }
            let noisy = device.scanSamples.filter { marginDb($0.rssiDbm) < quietMarginDb }
            HStack {
                Text("Below \(quietMarginDb) dB margin")
                Spacer()
                Text("\(noisy.count) of \(device.scanSamples.count)")
                    .foregroundColor(noisy.isEmpty ? .green : .orange)
                    .font(.system(.body, design: .monospaced))
            }
        }
    }

    private var chartDomain: ClosedRange<Double> {
        // Prefer the actual scanned range; fall back to the form values so
        // Swift Charts never auto-extends to zero and crushes the bars into
        // a sliver at the right edge.
        if let first = device.scanSamples.first, let last = device.scanSamples.last {
            let lo = Double(min(first.freqMHz, last.freqMHz))
            let hi = Double(max(first.freqMHz, last.freqMHz))
            // Pad half a step either side so the first/last bars aren't clipped.
            let padMHz = Double(last.freqMHz - first.freqMHz) / Double(max(device.scanSamples.count - 1, 1)) * 0.5
            return (lo - padMHz)...(hi + padMHz)
        }
        return startMHz...stopMHz
    }

    @ViewBuilder private var chart: some View {
        Chart(device.scanSamples) { s in
            BarMark(
                x: .value("Freq", Double(s.freqMHz)),
                y: .value("Margin", marginDb(s.rssiDbm)),
                width: .fixed(6)
            )
            .foregroundStyle(marginDb(s.rssiDbm) >= quietMarginDb ? Color.green : Color.orange)
        }
        .chartXScale(domain: chartDomain)
        .chartYScale(domain: 0...100)
        .chartYAxisLabel("Noise Margin (dB)")
        .chartXAxisLabel("Frequency (MHz)")
        .chartXAxis {
            AxisMarks(values: .automatic(desiredCount: 5)) { value in
                AxisGridLine()
                AxisTick()
                if let mhz = value.as(Double.self) {
                    AxisValueLabel { Text(String(format: "%.0f", mhz)) }
                }
            }
        }
    }
}
