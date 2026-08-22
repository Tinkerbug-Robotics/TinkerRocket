//
//  PreflightRunView.swift
//  TinkerRocketApp
//
//  Run the active rocket's pre-flight checklist at the pad.  Manual steps
//  are checked off by hand; auto steps render their live verified state
//  from telemetry / sync status and can't be tapped — the list only
//  completes when the app has actually observed each condition.
//
//  Presented as a sheet from the dashboard's preflight advisory line
//  (below the rocket state banner).
//

import SwiftUI

struct PreflightRunView: View {
    @ObservedObject var device: BLEDevice

    @EnvironmentObject var preflight: PreflightStore
    @EnvironmentObject var profileStore: RocketProfileStore
    @EnvironmentObject var syncer: ActiveRocketSyncer
    @Environment(\.dismiss) private var dismiss

    @State private var confirmReset = false

    var body: some View {
        NavigationView {
            Group {
                if let profile = profileStore.activeProfile {
                    checklist(for: profile)
                } else {
                    VStack(spacing: 12) {
                        Image(systemName: "checklist")
                            .font(.largeTitle)
                            .foregroundColor(.secondary)
                        Text("No active rocket")
                            .font(.headline)
                        Text("Pick a rocket (rocket icon on the dashboard) to run its checklist.")
                            .font(.caption)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                    }
                    .padding()
                }
            }
            .navigationTitle("Preflight")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Done") { dismiss() }
                }
                if let profile = profileStore.activeProfile {
                    ToolbarItem(placement: .primaryAction) {
                        Button("Reset") { confirmReset = true }
                            .disabled(preflight.config(for: profile.id)?.checked.isEmpty ?? true)
                    }
                }
            }
            .confirmationDialog("Reset checklist?", isPresented: $confirmReset,
                                titleVisibility: .visible) {
                Button("Uncheck All Steps", role: .destructive) {
                    if let id = profileStore.activeProfile?.id {
                        preflight.resetRun(for: id)
                    }
                }
            } message: {
                Text("Clears every hand-checked step for the next flight. Auto steps aren't affected — they always show the live state.")
            }
        }
    }

    // MARK: - Checklist body

    private func checklist(for profile: RocketProfile) -> some View {
        let items = preflight.effectiveItems(for: profile.id)
        let ctx = autoContext(profile: profile)
        let progress = PreflightChecklist.progress(items: items,
                                                   config: preflight.config(for: profile.id),
                                                   ctx: ctx)
        return List {
            Section {
                HStack {
                    VStack(alignment: .leading, spacing: 4) {
                        Text(profile.name)
                            .font(.headline)
                        Text(progress.isComplete
                             ? "All \(progress.total) steps complete"
                             : "\(progress.done) of \(progress.total) steps done")
                            .font(.caption)
                            .foregroundColor(progress.isComplete ? .green : .secondary)
                    }
                    Spacer()
                    if progress.isComplete {
                        Image(systemName: "checkmark.seal.fill")
                            .font(.title2)
                            .foregroundColor(.green)
                    }
                }
                ProgressView(value: Double(progress.done),
                             total: Double(max(progress.total, 1)))
                    .tint(progress.isComplete ? .green : .orange)
            }

            Section {
                if items.isEmpty {
                    Text("No checklist for this rocket yet — set one up from Preflight Checklist on the front page.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                ForEach(items) { item in
                    row(item, profile: profile, ctx: ctx)
                }
            } footer: {
                if items.contains(where: { $0.kind.isAuto }) {
                    Text("Steps marked \u{201C}auto\u{201D} are verified live by the app and can't be checked by hand.")
                }
            }
        }
    }

    @ViewBuilder
    private func row(_ item: PreflightItem, profile: RocketProfile,
                     ctx: PreflightAutoContext) -> some View {
        if let status = PreflightChecklist.autoStatus(item.kind, in: ctx) {
            autoRow(item, status: status)
        } else {
            manualRow(item, profile: profile)
        }
    }

    private func manualRow(_ item: PreflightItem, profile: RocketProfile) -> some View {
        let checked = preflight.isChecked(item.id, for: profile.id)
        return Button {
            preflight.setChecked(item.id, checked: !checked, for: profile.id)
        } label: {
            HStack(spacing: 10) {
                Image(systemName: checked ? "checkmark.circle.fill" : "circle")
                    .font(.title3)
                    .foregroundColor(checked ? .green : .secondary)
                VStack(alignment: .leading, spacing: 2) {
                    Text(item.title)
                        .foregroundColor(checked ? .secondary : .primary)
                        .strikethrough(checked, color: .secondary)
                    if !item.detail.isEmpty {
                        Text(item.detail)
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }
                Spacer()
            }
            .contentShape(Rectangle())
        }
        .buttonStyle(.plain)
    }

    private func autoRow(_ item: PreflightItem, status: PreflightAutoStatus) -> some View {
        HStack(spacing: 10) {
            statusIcon(status)
                .font(.title3)
            VStack(alignment: .leading, spacing: 2) {
                Text(item.title)
                    .foregroundColor(status == .satisfied ? .secondary : .primary)
                if case .pending(let reason) = status {
                    Text(reason)
                        .font(.caption)
                        .foregroundColor(.orange)
                } else if case .notApplicable(let reason) = status {
                    Text(reason)
                        .font(.caption)
                        .foregroundColor(.secondary)
                } else if !item.detail.isEmpty {
                    Text(item.detail)
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            Spacer()
            Text("AUTO")
                .font(.caption2.weight(.semibold))
                .foregroundColor(.teal)
        }
    }

    @ViewBuilder
    private func statusIcon(_ status: PreflightAutoStatus) -> some View {
        switch status {
        case .satisfied:
            Image(systemName: "checkmark.circle.fill").foregroundColor(.green)
        case .pending:
            Image(systemName: "circle.dashed").foregroundColor(.orange)
        case .notApplicable:
            Image(systemName: "minus.circle").foregroundColor(.secondary)
        }
    }

    private func autoContext(profile: RocketProfile) -> PreflightAutoContext {
        PreflightAutoContext(isConnected: device.isConnected,
                             hasTelemetry: device.hasReceivedTelemetry,
                             isRelay: device.isBaseStation,
                             telemetry: device.telemetry,
                             syncState: syncer.syncState,
                             profile: profile)
    }
}

// MARK: - Dashboard advisory

/// One quiet line below the rocket state banner: checklist progress for
/// the active rocket, tap to open the run sheet.  Deliberately compact —
/// it must not add dashboard clutter, and it never recolors the state
/// banner (design decision 2026-08-22: sensor-health owns that).
/// Renders nothing when the active rocket has no checklist.
struct PreflightAdvisoryRow: View {
    @ObservedObject var device: BLEDevice
    @EnvironmentObject var preflight: PreflightStore
    @EnvironmentObject var profileStore: RocketProfileStore
    @EnvironmentObject var syncer: ActiveRocketSyncer
    let onTap: () -> Void

    var body: some View {
        if let profile = profileStore.activeProfile {
            let items = preflight.effectiveItems(for: profile.id)
            if !items.isEmpty {
                let ctx = PreflightAutoContext(isConnected: device.isConnected,
                                               hasTelemetry: device.hasReceivedTelemetry,
                                               isRelay: device.isBaseStation,
                                               telemetry: device.telemetry,
                                               syncState: syncer.syncState,
                                               profile: profile)
                let progress = PreflightChecklist.progress(items: items,
                                                           config: preflight.config(for: profile.id),
                                                           ctx: ctx)
                Button(action: onTap) {
                    HStack(spacing: 6) {
                        Image(systemName: progress.isComplete
                              ? "checkmark.circle.fill" : "checklist")
                            .foregroundColor(progress.isComplete ? .green : .orange)
                        Text(progress.isComplete
                             ? "Preflight complete"
                             : "Preflight \(progress.done)/\(progress.total)")
                            .foregroundColor(progress.isComplete ? .secondary : .orange)
                        Image(systemName: "chevron.right")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                    .font(.caption.weight(.semibold))
                    .frame(maxWidth: .infinity)
                }
                .buttonStyle(.plain)
            }
        }
    }
}
