//
//  NetworkMonitor.swift
//  TinkerRocketApp
//
//  Offline maps — Phase 4 (see docs/plans/offline-maps.md).
//
//  Lightweight reachability so the map views can show an "OFFLINE" pill when
//  there's no connection (and the basemap is therefore coming from cache).
//

import Foundation
import Network
import Combine
import SwiftUI

final class NetworkMonitor: ObservableObject {
    static let shared = NetworkMonitor()

    @Published private(set) var isOnline = true

    private let monitor = NWPathMonitor()

    private init() {
        monitor.pathUpdateHandler = { [weak self] path in
            let online = path.status == .satisfied
            DispatchQueue.main.async { self?.isOnline = online }
        }
        monitor.start(queue: DispatchQueue(label: "TinkerRocket.NetworkMonitor"))
    }
}

/// Small "OFFLINE" pill — renders nothing while online. Drop into any map's
/// overlay stack; it observes the shared monitor.
struct OfflinePill: View {
    @ObservedObject private var network = NetworkMonitor.shared

    var body: some View {
        if !network.isOnline {
            HStack(spacing: 6) {
                Image(systemName: "wifi.slash")
                Text("OFFLINE").fontWeight(.bold)
            }
            .font(.caption2)
            .padding(.horizontal, 10)
            .padding(.vertical, 5)
            .background(.ultraThinMaterial)
            .foregroundColor(.orange)
            .cornerRadius(20)
            .shadow(radius: 2)
        }
    }
}
