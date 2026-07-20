//
//  FleetDashboardComponents.swift
//  TinkerRocketApp
//
//  #390: the rocket-centric dashboard chrome — pair switcher (two
//  base-station/rocket pairs, one phone), always-visible units bar
//  (roster chips with link badges, freshness, focus and display
//  toggles), the base-station strip, and the read-only section a
//  non-focused relayed rocket renders as.
//

import SwiftUI
import CoreBluetooth

// MARK: - Pair switcher (≥2 base stations)

/// Segmented chips picking which base station's "pair" (the BS + the
/// rockets it carries) the dashboard foregrounds. Only rendered with two
/// or more BSes connected; switching is a pure view change — both pairs'
/// links keep streaming in the background.
struct PairSwitcherView: View {
    @ObservedObject var fleet: BLEFleet

    var body: some View {
        HStack(spacing: 8) {
            ForEach(fleet.baseStations, id: \.peripheral?.identifier) { bs in
                let isForeground = bs === fleet.foregroundBaseStation
                Button {
                    fleet.foregroundBSID = bs.peripheral?.identifier
                } label: {
                    HStack(spacing: 6) {
                        Image(systemName: "antenna.radiowaves.left.and.right")
                            .font(.caption)
                        Text(bs.displayName)
                            .font(.caption)
                            .fontWeight(.semibold)
                            .lineLimit(1)
                    }
                    .padding(.horizontal, 14)
                    .padding(.vertical, 8)
                    .frame(maxWidth: .infinity)
                    .background(isForeground ? Color.accentColor : Color(.systemGray5))
                    .foregroundColor(isForeground ? .white : .primary)
                    .cornerRadius(10)
                }
            }
        }
    }
}

// MARK: - Units bar (roster chips)

/// Always-visible roster of the foreground pair's rockets (+ any directly
/// connected ones). A chip is filled while its rocket is displayed on the
/// dashboard; tapping toggles display (a pure view filter). The context
/// menu carries the radio-affecting action (base-station focus) and link
/// management (disconnect a direct link).
struct UnitsBarView: View {
    @ObservedObject var fleet: BLEFleet
    let subjects: [RocketSubject]

    var body: some View {
        ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: 8) {
                ForEach(subjects) { subject in
                    RocketChip(fleet: fleet, subject: subject)
                }

                // Add device — explicit scan opens the picker sheet (#394).
                Button {
                    fleet.startScanning(userInitiated: true)
                } label: {
                    HStack(spacing: 4) {
                        Image(systemName: "plus").font(.caption)
                        Text("Add").font(.caption)
                    }
                    .padding(.horizontal, 12)
                    .padding(.vertical, 8)
                    .background(Color(.systemGray5))
                    .foregroundColor(.blue)
                    .cornerRadius(20)
                }
            }
            .padding(.horizontal, 4)
        }
        .padding(.vertical, 2)
    }
}

private struct RocketChip: View {
    @ObservedObject var fleet: BLEFleet
    let subject: RocketSubject

    private var isHidden: Bool {
        guard let key = subject.key else { return false }
        return fleet.hiddenRocketKeys.contains(key)
    }

    private var isFocused: Bool {
        guard let bs = fleet.foregroundBaseStation,
              let focus = bs.focusRocketID else { return false }
        return subject.relays.contains { $0.baseStation === bs }
            && subject.key?.rocketID == focus
    }

    private var freshnessColor: Color {
        switch subject.freshness() {
        case .live:  return .green
        case .stale: return .orange
        case .lost:  return .gray
        }
    }

    var body: some View {
        Button {
            guard let key = subject.key else { return }
            if isHidden {
                fleet.hiddenRocketKeys.remove(key)
            } else {
                fleet.hiddenRocketKeys.insert(key)
            }
        } label: {
            HStack(spacing: 6) {
                Circle()
                    .fill(freshnessColor)
                    .frame(width: 7, height: 7)
                Text(subject.name)
                    .font(.caption)
                    .fontWeight(.medium)
                    .lineLimit(1)
                if isFocused {
                    Image(systemName: "scope").font(.system(size: 9))
                }
                if subject.direct != nil {
                    Image(systemName: "iphone").font(.system(size: 9))
                }
                if !subject.relays.isEmpty {
                    Image(systemName: "antenna.radiowaves.left.and.right")
                        .font(.system(size: 9))
                }
            }
            .padding(.horizontal, 12)
            .padding(.vertical, 8)
            .background(isHidden ? Color(.systemGray5) : Color.blue)
            .foregroundColor(isHidden ? .secondary : .white)
            .cornerRadius(20)
        }
        .contextMenu {
            if let key = subject.key,
               let bs = fleet.foregroundBaseStation,
               subject.relays.contains(where: { $0.baseStation === bs }),
               bs.focusRocketID != key.rocketID {
                Button {
                    fleet.setFocus(baseStation: bs, rocketID: key.rocketID)
                } label: {
                    Label("Focus base station here", systemImage: "scope")
                }
            }
            Button {
                guard let key = subject.key else { return }
                if isHidden { fleet.hiddenRocketKeys.remove(key) }
                else { fleet.hiddenRocketKeys.insert(key) }
            } label: {
                Label(isHidden ? "Show on dashboard" : "Hide from dashboard",
                      systemImage: isHidden ? "eye" : "eye.slash")
            }
            if let direct = subject.direct {
                Button(role: .destructive) {
                    fleet.disconnect(direct)
                } label: {
                    Label("Disconnect", systemImage: "xmark.circle")
                }
            }
        }
    }
}

// MARK: - Base-station strip

/// Thin infrastructure strip for the foreground base station: battery,
/// SD logging, and which rocket its radio is following. The BS stopped
/// impersonating a rocket — this is its home on the main dashboard.
struct BaseStationStripView: View {
    @ObservedObject var bs: BLEDevice

    /// Name of the rocket the BS radio follows, from its remote roster.
    private var focusName: String {
        guard let focus = bs.focusRocketID else { return "—" }
        let remote = bs.remoteRockets.first { $0.rocketID == focus }
        return remote?.displayName ?? "Rocket \(focus)"
    }

    var body: some View {
        HStack(spacing: 10) {
            Image(systemName: "antenna.radiowaves.left.and.right")
                .foregroundColor(.orange)
            VStack(alignment: .leading, spacing: 2) {
                Text(bs.displayName)
                    .font(.subheadline.weight(.semibold))
                HStack(spacing: 8) {
                    if bs.focusRocketID != nil {
                        Label(focusName, systemImage: "scope")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                    if bs.telemetry.bs_logging_active {
                        Label("Logging", systemImage: "record.circle")
                            .font(.caption2)
                            .foregroundColor(.red)
                    }
                }
            }
            Spacer()
            // No battery % here — phone-tested as ambiguous.  BS battery
            // lives in the Battery card's "Base Stn" row (with the rocket's)
            // and in BaseStationDetailView.
            Image(systemName: "chevron.right")
                .font(.caption2)
                .foregroundColor(.secondary)
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 10)
        .background(Color(.secondarySystemBackground))
        .cornerRadius(10)
    }
}

// MARK: - Rocket section header

/// Header above each displayed rocket's cards: name, transport ("Direct
/// BLE" / "via <BS> · age"), freshness, and a collapse chevron when the
/// dashboard is showing more than one rocket.
struct RocketSectionHeader: View {
    let subject: RocketSubject
    let collapsible: Bool
    @Binding var collapsed: Bool

    private func sourceLine(now: Date) -> String {
        if subject.direct?.isConnected == true {
            return "Direct BLE"
        }
        guard let relay = subject.freshestRelay else { return "No link" }
        let via = "via \(relay.baseStation.displayName)"
        switch subject.freshness(now: now) {
        case .live:
            return via
        case .stale(let age):
            return "\(via) · \(Int(age)) s ago"
        case .lost(let seen):
            if let seen {
                return "\(via) · lost, last heard \(Int(now.timeIntervalSince(seen)) / 60) min ago"
            }
            return "\(via) · lost"
        }
    }

    private var freshnessColor: Color {
        switch subject.freshness() {
        case .live:  return .green
        case .stale: return .orange
        case .lost:  return .gray
        }
    }

    var body: some View {
        // 1 Hz timeline so the age label advances between packets.
        TimelineView(.periodic(from: .now, by: 1)) { context in
            HStack(spacing: 8) {
                Circle().fill(freshnessColor).frame(width: 8, height: 8)
                VStack(alignment: .leading, spacing: 1) {
                    Text(subject.name)
                        .font(.headline)
                    Text(sourceLine(now: context.date))
                        .font(.caption2)
                        .foregroundColor(.secondary)
                }
                Spacer()
                if collapsible {
                    Image(systemName: collapsed ? "chevron.down" : "chevron.up")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            .padding(.horizontal, 12)
            .padding(.vertical, 8)
            .background(Color(.tertiarySystemBackground))
            .cornerRadius(8)
            .contentShape(Rectangle())
            .onTapGesture {
                if collapsible {
                    withAnimation(.easeInOut(duration: 0.2)) { collapsed.toggle() }
                }
            }
        }
    }
}

// MARK: - Non-focused relayed rocket section

/// Telemetry cards for a relayed rocket that is NOT the one mirrored by
/// the base-station link (the focused rocket renders the full interactive
/// dashboard). Driven straight off its RemoteRocket stream; the action row
/// targets THIS rocket through the carrying base station (cmd 50), with
/// desired-state computed from this rocket's telemetry.
struct RelayRocketSectionView: View {
    let subject: RocketSubject
    /// The base station carrying this rocket's stream (command target).
    let via: BLEDevice
    @ObservedObject var remote: RemoteRocket
    let collapsible: Bool
    @State private var collapsed = false

    var body: some View {
        VStack(spacing: 12) {
            RocketSectionHeader(subject: subject,
                                collapsible: collapsible,
                                collapsed: $collapsed)
            if collapsed {
                CollapsedRocketSummary(telemetry: remote.telemetry)
            } else {
                RocketStateView(state: remote.telemetry.state)
                FlightSummaryView(telemetry: remote.telemetry)
                // isBaseStation: relayed frames carry the relaying BS's own
                // battery fields, so the card shows the Rocket row AND the
                // "Base Stn" row (charge/voltage/current) — same layout as
                // the pre-#390 dashboard.  The strip's % is just a glance;
                // this card is the real numbers.
                BatteryView(telemetry: remote.telemetry, isBaseStation: true)
                GPSView(telemetry: remote.telemetry, compact: true)
                LoRaSignalView(telemetry: remote.telemetry)
                relayActions
            }
        }
    }

    /// Camera + rocket logging, targeted at this rocket only. Rocket
    /// logging here is the rocket's own flash recording (LoRa cmd 23) —
    /// the base station's SD log is controlled from the BS screen.
    private var relayActions: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Controls")
                .font(.headline)

            Button {
                let desired: UInt8 = remote.telemetry.camera_recording ? 0 : 1
                via.sendRelayCommand(targetRocketID: remote.rocketID,
                                     innerCommand: 1,
                                     innerPayload: Data([desired]))
            } label: {
                HStack {
                    Image(systemName: remote.telemetry.camera_recording ? "video.fill" : "video")
                    Text(remote.telemetry.camera_recording ? "Stop Camera" : "Start Camera")
                }
                .frame(maxWidth: .infinity)
                .padding()
                .background(remote.telemetry.camera_recording ? Color.red : Color.blue)
                .foregroundColor(.white)
                .cornerRadius(10)
            }

            Button {
                let desired: UInt8 = remote.telemetry.rocketLoggingActive ? 0 : 1
                via.sendRelayCommand(targetRocketID: remote.rocketID,
                                     innerCommand: 23,
                                     innerPayload: Data([desired]))
            } label: {
                HStack {
                    Image(systemName: remote.telemetry.rocketLoggingActive
                          ? "stop.circle.fill" : "record.circle")
                    Text(remote.telemetry.rocketLoggingActive
                         ? "Stop Rocket Logging" : "Start Rocket Logging")
                }
                .frame(maxWidth: .infinity)
                .padding()
                .background(remote.telemetry.rocketLoggingActive ? Color.red : Color.orange)
                .foregroundColor(.white)
                .cornerRadius(10)
            }

            Label("Pyro, power, calibration, sim and settings need the base station's focus or a direct connection.",
                  systemImage: "info.circle")
                .font(.caption2)
                .foregroundColor(.secondary)
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

/// One-line summary shown while a rocket's section is collapsed:
/// state · altitude · speed · battery.
struct CollapsedRocketSummary: View {
    let telemetry: TelemetryData

    var body: some View {
        HStack(spacing: 12) {
            Text(RocketStateView.displayLabel(for: telemetry.state))
                .font(.caption.weight(.bold))
            Text(telemetry.pressureAltDisplay)
                .font(.system(.caption, design: .monospaced))
            Text(telemetry.altitudeRateDisplay)
                .font(.system(.caption, design: .monospaced))
            Spacer()
            Text(telemetry.socDisplay)
                .font(.system(.caption, design: .monospaced))
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 8)
        .background(Color(.systemGray6))
        .cornerRadius(8)
    }
}
