//
//  FlightDetailView.swift
//  TinkerRocketApp
//
//  Flight summary detail page showing all computed flight statistics
//

import SwiftUI

struct FlightDetailView: View {
    let flight: CachedFlight
    var onDelete: (() -> Void)?
    @Environment(\.dismiss) private var dismiss
    @State private var showDeleteConfirm = false

    var body: some View {
        List {
            // Flight info
            Section("Flight Info") {
                HStack {
                    Text("Type")
                    Spacer()
                    if flight.type == .rocket {
                        Text("Rocket Flight")
                            .foregroundColor(.blue)
                    } else {
                        Text("LoRa Log")
                            .foregroundColor(.orange)
                    }
                }

                HStack {
                    Text("File Size")
                    Spacer()
                    Text(formatFileSize(flight.size))
                        .foregroundColor(.secondary)
                }

                if let date = flight.flightDate {
                    HStack {
                        Text("Date")
                        Spacer()
                        Text(formatDate(date))
                            .foregroundColor(.secondary)
                    }
                }
            }

            // Flight summary stats
            if flight.type == .rocket {
                Section("Flight Summary") {
                    SummaryRow(icon: "arrow.up", label: "Max Altitude",
                               value: flight.maxAltitudeM.map { formatAltitude($0) })
                    SummaryRow(icon: "speedometer", label: "Max Speed",
                               value: flight.maxSpeedMps.map { formatSpeed($0) })
                    SummaryRow(icon: "flame", label: "Burnout",
                               value: flight.burnoutTimeS.map { formatTime($0) })
                    SummaryRow(icon: "arrow.up.to.line", label: "Apogee",
                               value: flight.apogeeTimeS.map { formatTime($0) })
                }
            }

            // Visualizations
            Section {
                NavigationLink(destination: FlightChartView(flight: flight)) {
                    HStack {
                        Image(systemName: "chart.xyaxis.line")
                            .foregroundColor(.blue)
                        Text("View Plot")
                    }
                }
                NavigationLink(destination: FlightTrajectoryView(flight: flight)) {
                    HStack {
                        Image(systemName: "map")
                            .foregroundColor(.green)
                        Text("View Trajectory")
                    }
                }
            }

            // Actions
            Section {
                Button {
                    var items: [URL] = [flight.csvURL]
                    if let binURL = flight.binaryURL {
                        items.append(binURL)
                    }
                    if let summaryURL = flight.summaryURL {
                        items.append(summaryURL)
                    }
                    ShareHelper.share(items: items)
                } label: {
                    HStack {
                        Image(systemName: "square.and.arrow.up")
                        Text("Share Flight Data")
                    }
                }

                if onDelete != nil {
                    Button(role: .destructive) {
                        showDeleteConfirm = true
                    } label: {
                        HStack {
                            Image(systemName: "trash")
                            Text("Delete Flight")
                        }
                    }
                }
            }
        }
        .listStyle(InsetGroupedListStyle())
        .navigationTitle(flight.displayTitle)
        .alert("Delete Flight Log?", isPresented: $showDeleteConfirm) {
            Button("Delete", role: .destructive) {
                onDelete?()
                dismiss()
            }
            Button("Cancel", role: .cancel) {}
        } message: {
            Text("Are you sure you want to delete \"\(flight.displayTitle)\"? This cannot be undone.")
        }
    }

    // MARK: - Formatting Helpers

    private func formatAltitude(_ meters: Double) -> String {
        if meters >= 1000 {
            return String(format: "%.2f km", meters / 1000.0)
        } else {
            return String(format: "%.0f m", meters)
        }
    }

    /// ISA speed of sound at a given altitude (meters).
    /// T = 288.15 - 0.0065*h (valid up to 11 km), a = sqrt(1.4 * 287.058 * T).
    private func speedOfSound(atAltitudeM h: Double) -> Double {
        let T = max(288.15 - 0.0065 * h, 216.65)  // clamp at tropopause temp
        return sqrt(1.4 * 287.058 * T)
    }

    private func formatSpeed(_ mps: Double) -> String {
        let altM = flight.maxAltitudeM ?? 0
        let a = altM > 0 ? speedOfSound(atAltitudeM: altM) : 343.0
        if mps >= a {
            return String(format: "Mach %.1f", mps / a)
        } else {
            return String(format: "%.0f m/s", mps)
        }
    }

    private func formatTime(_ seconds: Double) -> String {
        return String(format: "T+%.1f s", seconds)
    }

    private func formatFileSize(_ bytes: UInt64) -> String {
        let kb = Double(bytes) / 1024.0
        if kb < 1024 {
            return String(format: "%.1f KB", kb)
        } else {
            let mb = kb / 1024.0
            return String(format: "%.2f MB", mb)
        }
    }

    // Reusable DateFormatter — avoid allocating a new one per call
    private static let dateFormatter: DateFormatter = {
        let f = DateFormatter()
        f.dateStyle = .medium
        f.timeStyle = .medium
        f.timeZone = .current
        return f
    }()

    private func formatDate(_ date: Date) -> String {
        Self.dateFormatter.string(from: date)
    }
}

// MARK: - Summary Row

struct SummaryRow: View {
    let icon: String
    let label: String
    let value: String?

    var body: some View {
        HStack {
            Image(systemName: icon)
                .frame(width: 24)
                .foregroundColor(.blue)
            Text(label)
            Spacer()
            Text(value ?? "--")
                .font(.system(.body, design: .monospaced))
                .foregroundColor(.secondary)
        }
    }
}
