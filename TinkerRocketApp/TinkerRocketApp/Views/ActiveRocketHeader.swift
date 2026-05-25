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
                Image("RocketIcon")
                    .resizable().renderingMode(.template)
                    .aspectRatio(contentMode: .fit)
                    .frame(width: 20, height: 20)
                    .foregroundColor(.accentColor)
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
        let mode = p.useAngleControl ? "Track Profile" : "Null Roll"
        let cam: String = {
            switch p.cameraType { case 1: return "GoPro"; case 2: return "RunCam"; default: return "No cam" }
        }()
        let anyPyro = p.pyro1Enabled || p.pyro2Enabled || p.pyro3Enabled || p.pyro4Enabled
        let pyro = anyPyro ? "Pyro on" : "Pyro off"
        let cal = p.magCal == nil ? "Uncalibrated" : "Cal saved"
        return "\(mode) \u{2022} \(cam) \u{2022} \(pyro) \u{2022} \(cal)"
    }
}
