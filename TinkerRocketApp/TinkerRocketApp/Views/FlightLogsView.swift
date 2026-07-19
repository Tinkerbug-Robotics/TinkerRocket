//
//  FlightLogsView.swift
//  TinkerRocketApp
//
//  Browse all locally cached flight data and LoRa logs
//

import SwiftUI

struct FlightLogsView: View {
    @State private var cachedFlights: [CachedFlight] = []

    // Multi-select for bulk delete of locally cached flights (id = filename).
    @State private var isSelecting = false
    @State private var selection: Set<String> = []
    @State private var showBulkDeleteConfirm = false

    private var allSelected: Bool {
        !cachedFlights.isEmpty && cachedFlights.allSatisfy { selection.contains($0.id) }
    }

    var body: some View {
        Group {
            if cachedFlights.isEmpty {
                VStack(spacing: 16) {
                    Image(systemName: "tray")
                        .font(.system(size: 48))
                        .foregroundColor(.secondary)
                    Text("No downloaded flights yet")
                        .font(.headline)
                        .foregroundColor(.secondary)
                    Text("Connect to a rocket or base station and download flight data to see it here.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                        .multilineTextAlignment(.center)
                        .padding(.horizontal, 40)
                }
                .frame(maxWidth: .infinity, maxHeight: .infinity)
            } else {
                VStack(spacing: 0) {
                    // Select / Cancel (+ Select all while selecting).
                    HStack {
                        if isSelecting {
                            Button(allSelected ? "Deselect all" : "Select all") {
                                if allSelected {
                                    selection.removeAll()
                                } else {
                                    selection = Set(cachedFlights.map { $0.id })
                                }
                            }
                            Spacer()
                            Button("Cancel") { exitSelection() }
                        } else {
                            Spacer()
                            Button("Select") { isSelecting = true }
                        }
                    }
                    .foregroundColor(.blue)
                    .padding(.horizontal)
                    .padding(.vertical, 8)

                    List {
                        ForEach(cachedFlights) { flight in
                            if isSelecting {
                                Button {
                                    toggleSelection(flight.id)
                                } label: {
                                    HStack {
                                        Image(systemName: selection.contains(flight.id)
                                              ? "checkmark.circle.fill" : "circle")
                                            .font(.title3)
                                            .foregroundColor(selection.contains(flight.id) ? .blue : .secondary)
                                        FlightLogRow(flight: flight)
                                    }
                                    .contentShape(Rectangle())
                                }
                                .buttonStyle(.plain)
                            } else {
                                NavigationLink(destination: FlightDetailView(flight: flight, onDelete: {
                                    FileCache.shared.deleteCachedFlight(flight)
                                    cachedFlights = FileCache.shared.listCachedFlights()
                                })) {
                                    FlightLogRow(flight: flight)
                                }
                            }
                        }
                    }
                    .listStyle(InsetGroupedListStyle())

                    // Bulk delete action bar.
                    if isSelecting {
                        HStack {
                            Spacer()
                            Button(action: { showBulkDeleteConfirm = true }) {
                                HStack(spacing: 6) {
                                    Image(systemName: "trash")
                                    Text("Delete\(selection.isEmpty ? "" : " (\(selection.count))")")
                                }
                                .padding(.horizontal, 16)
                                .padding(.vertical, 8)
                                .background(selection.isEmpty ? Color.gray : Color.red)
                                .foregroundColor(.white)
                                .cornerRadius(8)
                            }
                            .disabled(selection.isEmpty)
                        }
                        .padding(.horizontal)
                        .padding(.vertical, 8)
                    }
                }
                .alert("Delete \(selection.count) Flight\(selection.count == 1 ? "" : "s")?",
                       isPresented: $showBulkDeleteConfirm) {
                    Button("Delete", role: .destructive) { deleteSelected() }
                    Button("Cancel", role: .cancel) {}
                } message: {
                    Text("Are you sure you want to delete \(selection.count) downloaded flight\(selection.count == 1 ? "" : "s")? This removes the local copy only.")
                }
            }
        }
        .navigationTitle("Flight Logs")
        .brandedToolbar()
        .onAppear {
            cachedFlights = FileCache.shared.listCachedFlights()
        }
    }

    private func toggleSelection(_ id: String) {
        if selection.contains(id) { selection.remove(id) } else { selection.insert(id) }
    }

    private func exitSelection() {
        isSelecting = false
        selection.removeAll()
    }

    private func deleteSelected() {
        let toDelete = cachedFlights.filter { selection.contains($0.id) }
        for flight in toDelete { FileCache.shared.deleteCachedFlight(flight) }
        exitSelection()
        cachedFlights = FileCache.shared.listCachedFlights()
    }
}

struct FlightLogRow: View {
    let flight: CachedFlight
    @AppStorage("unitSystem") private var unitSystem: UnitSystem = .metric

    var body: some View {
        HStack(spacing: 12) {
            // Type icon
            DeviceTypeIcon(type: flight.type == .rocket ? .rocket : .baseStation)
                .frame(width: 32)

            // Title and subtitle
            VStack(alignment: .leading, spacing: 4) {
                Text(flight.displayTitle)
                    .font(.body)

                HStack(spacing: 6) {
                    Text(formatFileSize(flight.size))
                        .font(.caption)
                        .foregroundColor(.secondary)

                    if flight.type == .rocket {
                        Text("Rocket Flight")
                            .font(.caption)
                            .foregroundColor(.blue)
                    } else {
                        Text("LoRa Log")
                            .font(.caption)
                            .foregroundColor(.orange)
                    }
                }

                // Max altitude and max speed on the card
                if flight.type == .rocket,
                   flight.maxAltitudeM != nil || flight.maxSpeedMps != nil {
                    HStack(spacing: 12) {
                        if let alt = flight.maxAltitudeM {
                            HStack(spacing: 3) {
                                Image(systemName: "arrow.up")
                                    .font(.caption2)
                                Text(formatAltitude(alt))
                                    .font(.caption)
                            }
                            .foregroundColor(.secondary)
                        }
                        if let spd = flight.maxSpeedMps {
                            HStack(spacing: 3) {
                                Image(systemName: "speedometer")
                                    .font(.caption2)
                                Text(formatSpeed(spd))
                                    .font(.caption)
                            }
                            .foregroundColor(.secondary)
                        }
                    }
                }
            }
        }
        .padding(.vertical, 4)
    }

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

    private func formatFileSize(_ bytes: UInt64) -> String {
        let kb = Double(bytes) / 1024.0
        if kb < 1024 {
            return String(format: "%.1f KB", kb)
        } else {
            let mb = kb / 1024.0
            return String(format: "%.2f MB", mb)
        }
    }
}

struct FlightLogsView_Previews: PreviewProvider {
    static var previews: some View {
        NavigationView {
            FlightLogsView()
        }
    }
}
