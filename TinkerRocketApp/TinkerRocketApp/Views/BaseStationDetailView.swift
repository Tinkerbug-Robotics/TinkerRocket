//
//  BaseStationDetailView.swift
//  TinkerRocketApp
//
//  #390: the base station's own screen — battery, storage, SD logging,
//  radio focus, and tools. These used to be mixed into the main dashboard
//  as if the BS were a rocket; the dashboard is rocket-centric now and
//  this is the infrastructure's home, reached from the BS strip.
//

import SwiftUI

struct BaseStationDetailView: View {
    @ObservedObject var bs: BLEDevice
    @ObservedObject var fleet: BLEFleet
    @Binding var activeSheet: DashboardSheet?

    var body: some View {
        ScrollView {
            VStack(spacing: 16) {
                batteryCard
                StorageBarView(device: bs)
                focusCard
                loggingCard
                toolsCard
            }
            .padding()
        }
        .navigationTitle(bs.displayName)
        .navigationBarTitleDisplayMode(.inline)
    }

    // MARK: - Battery

    private var batteryCard: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Battery")
                .font(.headline)
            HStack(spacing: 0) {
                Text("")
                    .frame(width: 70, alignment: .leading)
                Text("Charge").font(.caption).foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                Text("Voltage").font(.caption).foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                Text("Current").font(.caption).foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
            }
            BatteryRow(label: "Base Stn",
                       charge: bs.telemetry.bsSocDisplay,
                       voltage: bs.telemetry.bsVoltageDisplay,
                       current: bs.telemetry.bsCurrentDisplay)
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }

    // MARK: - Radio focus

    private var focusCard: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Radio Focus")
                .font(.headline)
            Text("Hop-follow, in-flight heartbeats, and stale status track this rocket. It never switches on its own.")
                .font(.caption)
                .foregroundColor(.secondary)

            if bs.remoteRockets.isEmpty {
                Text("No rockets heard yet.")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .padding(.vertical, 4)
            } else {
                ForEach(bs.remoteRockets) { remote in
                    Button {
                        fleet.setFocus(baseStation: bs, rocketID: remote.rocketID)
                    } label: {
                        HStack {
                            Image(systemName: "scope")
                                .foregroundColor(remote.rocketID == bs.focusRocketID
                                                 ? .accentColor : .clear)
                            Text(remote.displayName)
                                .foregroundColor(.primary)
                            Spacer()
                            if remote.rocketID == bs.focusRocketID {
                                Text("following")
                                    .font(.caption)
                                    .foregroundColor(.secondary)
                            }
                        }
                        .padding(.vertical, 6)
                    }
                }
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }

    // MARK: - SD logging

    private var loggingCard: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("CSV Logging")
                .font(.headline)

            HStack {
                StatusBadge(label: "Base Stn Log",
                            active: bs.telemetry.bs_logging_active)
                Spacer()
            }

            if !bs.telemetry.active_file.isEmpty {
                Text("File: \(bs.telemetry.active_file)")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }

            // BS silence-close countdown (moved here from the dashboard
            // status card — it is a BS property, not a rocket one).
            if bs.telemetry.bs_logging_active,
               let remaining = bs.telemetry.bs_log_silence_remaining_s {
                let secs = Int(remaining)
                let color: Color = secs > 60 ? .green : (secs > 10 ? .orange : .red)
                HStack(spacing: 6) {
                    Image(systemName: "timer")
                        .foregroundColor(color)
                    Text("Auto-close in \(formatElapsed(seconds: secs))")
                        .font(.caption)
                        .foregroundColor(color)
                }
            }

            Button {
                // #390: decoupled BS-only toggle (cmd 46) — never uplinks a
                // rocket-logging command. Rocket recording is controlled
                // from each rocket's own section.
                bs.sendSetBSLogging(!bs.telemetry.bs_logging_active)
            } label: {
                let active = bs.telemetry.bs_logging_active
                HStack {
                    Image(systemName: active ? "stop.circle.fill" : "record.circle")
                    Text(active ? "Stop Logging" : "Start Logging")
                }
                .frame(maxWidth: .infinity)
                .padding()
                .background(active ? Color.red : Color.orange)
                .foregroundColor(.white)
                .cornerRadius(10)
            }
            Text("Base-station CSV only — rocket recording is controlled from the rocket's section.")
                .font(.caption2)
                .foregroundColor(.secondary)
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }

    // MARK: - Tools

    private var toolsCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Tools")
                .font(.headline)

            Button {
                activeSheet = .frequencyScan(bs)
            } label: {
                HStack {
                    Image(systemName: "waveform.badge.magnifyingglass")
                    Text("Frequency Scan")
                    Spacer()
                    Image(systemName: "chevron.right").font(.caption)
                }
                .foregroundColor(.primary)
                .padding(.vertical, 6)
            }

            Divider()

            NavigationLink(destination: FileManagerView(device: bs)) {
                HStack {
                    Image(systemName: "book.fill")
                    Text("Stored Flight Logs")
                    Spacer()
                    Image(systemName: "chevron.right").font(.caption)
                }
                .foregroundColor(.primary)
                .padding(.vertical, 6)
            }

            Divider()

            Button {
                activeSheet = .settings(bs)
            } label: {
                HStack {
                    Image(systemName: "gearshape")
                    Text("Settings (network, LoRa, firmware)")
                    Spacer()
                    Image(systemName: "chevron.right").font(.caption)
                }
                .foregroundColor(.primary)
                .padding(.vertical, 6)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}
