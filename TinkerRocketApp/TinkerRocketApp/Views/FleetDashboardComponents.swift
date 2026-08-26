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
                // Just the followed rocket — battery lives in the Battery
                // card's "Base Stn" row and logging state on the Status
                // card (phone-tested: the strip is a label, not a control).
                if bs.focusRocketID != nil {
                    Label(focusName, systemImage: "scope")
                        .font(.caption2)
                        .foregroundColor(.secondary)
                }
            }
            Spacer()
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
    /// Supplies the latched per-rocket fix that the direction arrow needs —
    /// BLEDevice.lastValidRocketFix only tracks the BS's FOCUSED rocket, and
    /// this view renders the non-focused ones.
    var fleet: BLEFleet? = nil
    /// Phone location, also for the arrow. Optional so previews stay cheap.
    var locationManager: LocationManager? = nil
    @State private var collapsed = false

    /// Latched fix for THIS rocket, keyed by (network, rocket) — rocket ids
    /// are only unique per network (#390).
    private var rocketFix: LastValidRocketFix? {
        fleet?.lastValidRocketFixes[RocketKey(networkID: via.networkID,
                                              rocketID: remote.rocketID)]
    }

    /// Live if the last relayed frame is recent. Recomputed on every frame
    /// (remote.lastSeen is @Published), so it only flips to stale once a newer
    /// frame arrives — enough for the signal card's warning styling.
    private var trackingHealthy: Bool {
        Date().timeIntervalSince(remote.lastSeen) < 5
    }

    var body: some View {
        VStack(spacing: 12) {
            RocketSectionHeader(subject: subject,
                                collapsible: collapsible,
                                collapsed: $collapsed)
            // 1 Hz clock.  remote.lastSeen is @Published, so a NEW frame
            // re-renders this section — but the case that matters here is
            // frames STOPPING, and then nothing publishes and the staleness
            // treatment would never engage.  The header runs its own ticker
            // for exactly this reason; the cards need one too.
            TimelineView(.periodic(from: .now, by: 1)) { context in
                let freshness = subject.freshness(now: context.date)
                staleAwareBody(freshness: freshness, now: context.date)
            }
        }
    }

    @ViewBuilder
    private func staleAwareBody(freshness: RocketFreshness, now: Date) -> some View {
        VStack(spacing: 12) {
            if collapsed {
                CollapsedRocketSummary(telemetry: remote.telemetry)
                    .opacity(freshness.cardOpacity)
            } else {
                RocketStateView(state: remote.telemetry.state)
                    .opacity(freshness.cardOpacity)

                // Advisory line under the state banner, same shape and rule
                // as the preflight and "LoRa off" lines: it never recolors
                // the banner, and renders nothing while the stream is live.
                StaleTelemetryAdvisoryRow(freshness: freshness, now: now)
                // The same card stack the focused/direct dashboard renders.
                // This branch used to hand-assemble a shorter list and had
                // silently lost SignalStrengthView (arrow + GNSS bar + LoRa
                // bars), IMUView, StatusFlagsView, FlightEventFlagsView and
                // HealthCardView. isBaseStation: true — relayed frames carry
                // the relaying BS's own battery fields, so the battery card
                // shows the Rocket row AND the "Base Stn" row, same as before.
                RocketTelemetryCards(
                    telemetry: remote.telemetry,
                    isBaseStation: true,
                    bleRSSI: via.connectedRSSI,
                    locationManager: locationManager,
                    rocketFix: rocketFix,
                    trackingHealthy: trackingHealthy,
                    staleOpacity: freshness.cardOpacity,
                    staleAgeSec: freshness.staleAgeSec(now: now)
                )
                // NOT dimmed: the controls still work on a stale rocket, and
                // a faded button reads as disabled.
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

        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

/// One-line summary shown while a rocket's section is collapsed:
/// state · altitude · speed · battery.
/// One quiet line under the rocket state banner while telemetry is not live
/// (#836 item 3).
///
/// The section header already carries the age ("via Base 1 · 47 s ago"), but
/// it is caption2 secondary text that doubles as the transport line, sitting
/// ABOVE the state banner — easy to read past on a pad, where the eye goes to
/// the big state word and the cards under it.  This repeats the fact where
/// those cards start.
///
/// Advisory only: it never recolors the state banner, matching the preflight
/// and "LoRa off" lines it sits alongside.  Renders nothing while live.
struct StaleTelemetryAdvisoryRow: View {
    let freshness: RocketFreshness
    var now: Date = Date()

    /// The rendered text, or nil when the line stays silent.
    /// `messageForTest` is the same value — SwiftUI bodies are not directly
    /// assertable, so the copy the test reads has to be the copy the body
    /// renders.
    var messageForTest: String? { message }

    private var message: String? {
        switch freshness {
        case .live:
            return nil
        case .stale(let age):
            return "Telemetry \(RocketFreshness.ageText(age)) old"
        case .lost(let seen):
            guard let seen else { return "No telemetry received" }
            return "Link lost — last heard \(RocketFreshness.ageText(now.timeIntervalSince(seen))) ago"
        }
    }

    var body: some View {
        if let message {
            HStack(spacing: 6) {
                Image(systemName: "clock.badge.exclamationmark")
                    .font(.caption)
                Text(message)
                    .font(.caption)
                Spacer(minLength: 0)
            }
            .foregroundColor(.orange)
            .padding(.horizontal, 4)
            .accessibilityElement(children: .combine)
            .accessibilityLabel(message)
        }
    }
}

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
