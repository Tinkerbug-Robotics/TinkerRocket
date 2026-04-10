//
//  FlightLogsView.swift
//  TinkerRocketApp
//
//  Browse all locally cached flight data and LoRa logs
//

import SwiftUI

struct FlightLogsView: View {
    @State private var cachedFlights: [CachedFlight] = []

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
                List {
                    ForEach(cachedFlights) { flight in
                        NavigationLink(destination: FlightDetailView(flight: flight, onDelete: {
                            FileCache.shared.deleteCachedFlight(flight)
                            cachedFlights = FileCache.shared.listCachedFlights()
                        })) {
                            FlightLogRow(flight: flight)
                        }
                    }
                }
                .listStyle(InsetGroupedListStyle())
            }
        }
        .navigationTitle("Flight Logs")
        .brandedToolbar()
        .onAppear {
            cachedFlights = FileCache.shared.listCachedFlights()
        }
    }
}

struct FlightLogRow: View {
    let flight: CachedFlight

    var body: some View {
        HStack(spacing: 12) {
            // Type icon
            Group {
                if flight.type == .rocket {
                    Image("RocketIcon")
                        .resizable()
                        .renderingMode(.template)
                        .aspectRatio(contentMode: .fit)
                        .frame(width: 24, height: 24)
                        .foregroundColor(.blue)
                } else {
                    Image(systemName: "antenna.radiowaves.left.and.right")
                        .font(.title3)
                        .foregroundColor(.orange)
                }
            }
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
