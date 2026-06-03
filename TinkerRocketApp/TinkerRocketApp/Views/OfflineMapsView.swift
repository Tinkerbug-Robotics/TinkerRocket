//
//  OfflineMapsView.swift
//  TinkerRocketApp
//
//  Offline maps — Trial 2 (see docs/plans/offline-maps.md).
//
//  Manage downloaded regions: see what's saved, how much space it uses, delete
//  to reclaim space, and add a new region. Shared entry point from both the
//  Rocket Map and Drift Cast.
//

import SwiftUI
import CoreLocation

struct OfflineMapsView: View {
    /// Default center for a new region (the calling map's current center).
    let suggestedCenter: CLLocationCoordinate2D

    @Environment(\.dismiss) private var dismiss
    @ObservedObject private var store = OfflineRegionStore.shared
    @State private var showSave = false

    var body: some View {
        NavigationView {
            List {
                if store.regions.isEmpty {
                    Section {
                        Text("No offline areas saved yet. Tap ﹢ to download imagery for a launch site while you have signal.")
                            .font(.callout)
                            .foregroundColor(.secondary)
                    }
                } else {
                    Section {
                        ForEach(store.regions) { region in
                            regionRow(region)
                        }
                        .onDelete { idx in
                            idx.map { store.regions[$0] }.forEach(store.delete)
                        }
                    } footer: {
                        Text("Total offline storage · \(formatMB(store.totalBytes)) across \(store.regions.count) region\(store.regions.count == 1 ? "" : "s")")
                    }
                }
            }
            .navigationTitle("Offline Maps")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Done") { dismiss() }
                }
                ToolbarItem(placement: .primaryAction) {
                    Button { showSave = true } label: { Image(systemName: "plus") }
                }
            }
            .sheet(isPresented: $showSave) {
                SaveAreaView(initialCenter: suggestedCenter)
            }
        }
    }

    private func regionRow(_ r: OfflineRegion) -> some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(r.name).fontWeight(.medium)
            Text("\(String(format: "%.1f", r.radiusMeters / 1000)) km · z\(r.minZoom)–\(r.maxZoom) · \(formatMB(r.bytes)) · \(r.tileSource?.displayName ?? r.source)")
                .font(.caption)
                .foregroundColor(.secondary)
            Text("Saved \(r.savedAt.formatted(date: .abbreviated, time: .omitted))")
                .font(.caption2)
                .foregroundColor(.secondary)
        }
        .padding(.vertical, 2)
    }

    private func formatMB(_ bytes: Int64) -> String {
        let mb = Double(bytes) / 1_048_576
        if mb < 0.1 { return "0 MB" }
        return String(format: "%.0f MB", mb)
    }
}
