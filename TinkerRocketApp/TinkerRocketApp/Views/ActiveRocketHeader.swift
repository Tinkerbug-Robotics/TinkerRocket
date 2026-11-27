//
//  ActiveRocketHeader.swift
//  TinkerRocketApp
//
//  Top-of-dashboard summary of the active rocket profile (issue #132):
//  name + a few key settings + the connect-time sync status.  Tapping it
//  opens the rocket picker.  On a base-station connection it's read-only and
//  reminds the user that settings are edited on the rocket.
//

import SwiftUI

struct ActiveRocketHeader: View {
    @ObservedObject var device: BLEDevice
    @EnvironmentObject var store: RocketProfileStore
    @EnvironmentObject var syncer: ActiveRocketSyncer

    var body: some View {
        // On a base station the active rocket is read-only — the picker isn't
        // reachable here (you change/manage rockets connected directly to a
        // rocket).  So the header navigates only for a direct rocket link.
        if device.isBaseStation {
            card
        } else {
            NavigationLink(destination: RocketProfileView(device: device)) { card }
                .buttonStyle(.plain)
        }
    }

    private var card: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 8) {
                // Always the rocket glyph (this header is about the active
                // rocket profile), accent-tinted to read as tappable.
                DeviceTypeIcon(type: .rocket, size: 20, tint: .custom(.accentColor))
                Text(store.activeProfile?.name ?? "No rocket selected")
                    .font(.headline)
                    .foregroundColor(.primary)
                Spacer()
                if !device.isBaseStation {
                    Image(systemName: "chevron.right")
                        .font(.caption).foregroundColor(.secondary)
                }
            }

            if let p = store.activeProfile {
                Text(keyline(p))
                    .font(.caption).foregroundColor(.secondary)
                // The keyline is the profile; this is the flight computer's
                // own word on guidance, when the two disagree or it has not
                // given one (GuidanceStateCheck).
                // Guidance flies the fins: no line for a board without them.
                if device.features.finControl {
                    GuidanceStateLine(check: GuidanceStateCheck.evaluate(
                        profile: p, config: device.rocketConfig,
                        isConnected: device.isConnected,
                        isBaseStation: device.isBaseStation))
                }
            } else if !device.isBaseStation {
                Text("Tap to pick or create a rocket.")
                    .font(.caption).foregroundColor(.secondary)
            }

            if device.isBaseStation {
                Text("Settings are edited on the rocket.")
                    .font(.caption2).foregroundColor(.secondary)
            } else {
                SyncStatusRow(state: syncer.syncState)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.secondarySystemBackground))
        .cornerRadius(10)
    }

    private func keyline(_ p: RocketProfile) -> String {
        // The control mode and camera only where the board has fins and a
        // camera (AppFeatures): a Beetle has neither, and a summary saying
        // "Null Roll \u{2022} RunCam" for it would describe hardware it lacks.
        let features = device.features
        var parts: [String] = []
        if features.finControl { parts.append(p.controlModeLabel) }
        if features.camera {
            switch p.cameraType {
            case 1: parts.append("GoPro")
            case 2: parts.append("RunCam")
            default: parts.append("No cam")
            }
        }
        let anyPyro = p.pyro1Enabled || p.pyro2Enabled || p.pyro3Enabled || p.pyro4Enabled
        parts.append(anyPyro ? "Pyro on" : "Pyro off")
        parts.append(p.magCal == nil ? "Uncalibrated" : "Cal saved")
        return parts.joined(separator: " \u{2022} ")
    }
}

/// One line saying whether the flight computer holds the guidance setting the
/// profile shows: red when it will fly something else, orange when nothing
/// from it confirms a guidance profile, nothing otherwise.  Shared by this
/// header and Settings' Control Mode section.
struct GuidanceStateLine: View {
    let check: GuidanceStateCheck

    var body: some View {
        if let line = check.message {
            Label(line, systemImage: check.isMismatch
                  ? "exclamationmark.octagon.fill" : "questionmark.circle")
                .font(.caption)
                .foregroundColor(check.isMismatch ? .red : .orange)
                .fixedSize(horizontal: false, vertical: true)
        }
    }
}
