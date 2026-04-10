//
//  TinkerRocketAppApp.swift
//  TinkerRocketApp
//

import SwiftUI

@main
struct TinkerRocketAppApp: App {
    @AppStorage("hasOnboarded") private var hasOnboarded: Bool = false

    var body: some Scene {
        WindowGroup {
            if hasOnboarded {
                DashboardView()
            } else {
                OnboardingView()
            }
        }
    }
}
