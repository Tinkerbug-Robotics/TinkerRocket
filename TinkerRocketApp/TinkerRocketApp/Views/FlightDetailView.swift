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
    @AppStorage("unitSystem") private var unitSystem: UnitSystem = .metric
    /// #1658: the FC board read from the log, for a summary that predates
    /// `settings.board`. nil until read, or when the log predates it too.
    @State private var logBoard: String?

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

            // #1658: which rocket and computer this flight came from, so a
            // flight can still be matched to its computer after launch day.
            if flight.type == .rocket {
                Section {
                    SourceRow(label: "Rocket", value: flight.source?.rocket_profile ?? "Unknown")
                    SourceRow(label: "Computer", value: flight.source?.unit_name ?? "Unknown",
                              detail: flight.source?.unit_id.map { "Unit \($0)" })
                    SourceRow(label: "Board", value: board ?? "Unknown")
                    if let fw = flight.source?.fc_fw {
                        SourceRow(label: "FC firmware", value: fw, monospaced: true)
                    }
                    if let fw = flight.source?.oc_fw {
                        SourceRow(label: "OC firmware", value: fw, monospaced: true)
                    }
                    if let gnss = flight.source?.fc_gnss {
                        SourceRow(label: "GNSS", value: gnss)
                    }
                    if let date = flight.source?.downloaded_at
                        .flatMap({ ISO8601DateFormatter().date(from: $0) }) {
                        SourceRow(label: "Downloaded", value: formatDate(date))
                    }
                } header: {
                    Text("Source")
                } footer: {
                    if flight.source == nil {
                        Text("Downloaded before the app recorded which computer a flight came from.")
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
        .task {
            // A summary written before #1658 has no board; the log has it.
            guard flight.type == .rocket, flight.fcBoard == nil,
                  let bin = flight.binaryURL else { return }
            logBoard = await Task.detached(priority: .utility) {
                FileCache.boardFromLog(bin)
            }.value
        }
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

    /// The computer's board: as provisioned (the OC's, then the FC's, at
    /// download), else what the log recorded.
    private var board: String? {
        flight.source?.oc_board ?? flight.source?.fc_board ?? flight.fcBoard ?? logBoard
    }

    // MARK: - Formatting Helpers

    private func formatAltitude(_ meters: Double) -> String {
        UnitFormatter.altitude(meters, system: unitSystem)
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
        }
        return UnitFormatter.speed(mps, decimals: 0, system: unitSystem)
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

// MARK: - Source Row

/// #1658: one line of the Source section, with an optional caption under the value.
private struct SourceRow: View {
    let label: String
    let value: String
    var detail: String? = nil
    var monospaced = false

    var body: some View {
        HStack(alignment: .firstTextBaseline) {
            Text(label)
            Spacer()
            VStack(alignment: .trailing, spacing: 2) {
                Text(value)
                    .font(monospaced ? .system(.caption, design: .monospaced) : .body)
                    .foregroundColor(.secondary)
                    .lineLimit(1)
                    .minimumScaleFactor(0.7)
                if let detail {
                    Text(detail)
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
        }
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
