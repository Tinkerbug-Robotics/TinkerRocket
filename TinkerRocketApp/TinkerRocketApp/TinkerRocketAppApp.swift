//
//  TinkerRocketAppApp.swift
//  TinkerRocketApp
//

import SwiftUI

@main
struct TinkerRocketAppApp: App {
    @AppStorage("hasOnboarded") private var hasOnboarded: Bool = false

    // Per-rocket profiles (#132).  App-level so the picker, settings, and the
    // connect-time syncer all share one source of truth.
    @StateObject private var profileStore = RocketProfileStore()
    @StateObject private var syncer = ActiveRocketSyncer()
    // Pre-flight checklists: master template + per-rocket diffs.  App-level
    // for the same reason — the front-page editor and the dashboard run
    // sheet share one store.
    @StateObject private var preflightStore = PreflightStore()

    var body: some Scene {
        WindowGroup {
            Group {
                if hasOnboarded {
                    DashboardView()
                } else {
                    OnboardingView()
                }
            }
            .environmentObject(profileStore)
            .environmentObject(syncer)
            .environmentObject(preflightStore)
        }
    }
}
