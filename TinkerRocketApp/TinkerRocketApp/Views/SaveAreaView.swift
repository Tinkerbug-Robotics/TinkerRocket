//
//  SaveAreaView.swift
//  TinkerRocketApp
//
//  Offline maps — Trial 2 (see docs/plans/offline-maps.md).
//
//  Pick an area (center + radius) and a detail level, see a live tile/size
//  estimate, and download the tiles for offline use. Only cache-permissive
//  sources (USGS) can be saved.
//

import SwiftUI
import MapKit

// MARK: - Region picker map (center crosshair + radius circle)

struct RegionPickerMap: UIViewRepresentable {
    @Binding var region: MKCoordinateRegion
    var radiusMeters: Double
    var source: TileSource

    func makeUIView(context: Context) -> MKMapView {
        let map = MKMapView()
        map.delegate = context.coordinator
        map.mapType = source.appleMapType
        map.setRegion(region, animated: false)
        context.coordinator.basemap.apply(source, to: map)
        return map
    }

    func updateUIView(_ map: MKMapView, context: Context) {
        if map.mapType != source.appleMapType { map.mapType = source.appleMapType }
        context.coordinator.basemap.apply(source, to: map)

        // Redraw the radius circle (centered on the current map center).
        map.removeOverlays(map.overlays.filter { $0 is MKCircle })
        map.addOverlay(MKCircle(center: region.center, radius: radiusMeters), level: .aboveLabels)
    }

    func makeCoordinator() -> Coordinator { Coordinator(self) }

    class Coordinator: NSObject, MKMapViewDelegate {
        var parent: RegionPickerMap
        let basemap = BasemapOverlayController()
        init(_ parent: RegionPickerMap) { self.parent = parent }

        func mapView(_ mapView: MKMapView, regionDidChangeAnimated animated: Bool) {
            parent.region = mapView.region
        }

        func mapView(_ mapView: MKMapView, rendererFor overlay: MKOverlay) -> MKOverlayRenderer {
            if let tile = overlay as? MKTileOverlay {
                return MKTileOverlayRenderer(tileOverlay: tile)
            }
            if let circle = overlay as? MKCircle {
                let r = MKCircleRenderer(circle: circle)
                r.strokeColor = .systemBlue
                r.lineWidth = 2
                r.fillColor = UIColor.systemBlue.withAlphaComponent(0.15)
                return r
            }
            return MKOverlayRenderer(overlay: overlay)
        }
    }
}

// MARK: - Save area sheet

struct SaveAreaView: View {
    let initialCenter: CLLocationCoordinate2D

    @Environment(\.dismiss) private var dismiss
    @ObservedObject private var store = OfflineRegionStore.shared
    @StateObject private var downloader = TileDownloader()

    @State private var source: TileSource = .usgsImageryTopo
    @State private var region: MKCoordinateRegion
    @State private var radiusKm: Double = 5
    @State private var maxZoom: Double = 16
    @State private var name: String = ""

    private let cacheableSources = TileSource.allCases.filter { $0.isCacheable }

    init(initialCenter: CLLocationCoordinate2D) {
        self.initialCenter = initialCenter
        _region = State(initialValue: MKCoordinateRegion(
            center: initialCenter,
            span: MKCoordinateSpan(latitudeDelta: 0.15, longitudeDelta: 0.15)))
    }

    /// Never deeper than the source actually serves: those bands are all
    /// 404s, which the downloader counts as done with 0 bytes, so they would
    /// pad the estimate with tiles that can never arrive.
    private var effectiveMaxZoom: Int { min(Int(maxZoom), source.maxZoom) }

    private var spec: RegionSpec {
        RegionSpec(center: region.center, radiusMeters: radiusKm * 1000,
                   minZoom: 10, maxZoom: effectiveMaxZoom)
    }
    private var tileCount: Int { TileMath.tileCount(for: spec) }
    private var estMB: Double { Double(Int64(tileCount) * kEstimatedTileBytes) / 1_048_576 }
    private var tooBig: Bool { estMB > 200 }

    var body: some View {
        NavigationView {
            VStack(spacing: 0) {
                ZStack {
                    RegionPickerMap(region: $region, radiusMeters: radiusKm * 1000, source: source)
                    // Center crosshair = the region center used for the download.
                    Image(systemName: "plus.viewfinder")
                        .font(.system(size: 26, weight: .regular))
                        .foregroundColor(.blue)
                        .shadow(radius: 2)
                }
                .frame(height: 240)

                Form {
                    // Snapshotting the spec makes the manifest honest, but a
                    // picker that still moves during a run is its own lie: the
                    // sheet would show 20 km / Topo while 5 km of Imagery+Topo
                    // was being fetched. Frozen for the duration, like Cancel.
                    Section("Source") {
                        Picker("Imagery", selection: $source) {
                            ForEach(cacheableSources) { Text($0.displayName).tag($0) }
                        }
                        .disabled(downloader.isRunning)
                    }
                    Section("Area") {
                        sliderRow(title: "Radius", value: String(format: "%.1f km", radiusKm),
                                  binding: $radiusKm, range: 1...20, step: 0.5)
                            .disabled(downloader.isRunning)
                        sliderRow(title: "Detail (max zoom)", value: "z\(Int(maxZoom))",
                                  binding: $maxZoom,
                                  range: 13...Double(source.maxZoom), step: 1)
                            .disabled(downloader.isRunning)
                        HStack {
                            Text("Estimate").foregroundColor(.secondary)
                            Spacer()
                            Text("~\(tileCount) tiles · ~\(String(format: "%.0f", estMB)) MB")
                                .font(.system(.body, design: .monospaced))
                                .foregroundColor(tooBig ? .orange : .primary)
                        }
                        if tooBig {
                            Text("Large download — consider a smaller radius or lower max zoom.")
                                .font(.caption).foregroundColor(.orange)
                        }
                    }
                    Section("Name") {
                        TextField(defaultName, text: $name)
                    }
                    Section {
                        if downloader.isRunning {
                            VStack(alignment: .leading, spacing: 8) {
                                ProgressView(value: downloader.total > 0
                                             ? Double(downloader.done) / Double(downloader.total) : 0)
                                // The failed count is visible AS IT HAPPENS:
                                // the sheet closes on success, so this is the
                                // only place a partly-covered area can admit
                                // it.
                                Text("\(downloader.done) / \(downloader.total) tiles · "
                                     + String(format: "%.0f MB", Double(downloader.bytes) / 1_048_576)
                                     + (downloader.failed > 0 ? " · \(downloader.failed) failed" : ""))
                                    .font(.system(.caption, design: .monospaced))
                                    .foregroundColor(downloader.failed > 0 ? .orange : .secondary)
                                Button(role: .cancel) { downloader.cancel() } label: {
                                    Text("Cancel").frame(maxWidth: .infinity)
                                }
                            }
                        } else {
                            // A run that reached nothing used to end silently
                            // on a saved 0 MB area — the failure only showed
                            // up at the field.
                            if downloader.phase == .failed {
                                Text("Download failed — no tiles could be fetched. "
                                     + "Check your connection and try again.")
                                    .font(.caption)
                                    .foregroundColor(.red)
                            }
                            Button {
                                // The downloader records the region, not this
                                // sheet: it outlives us, and a dismissal
                                // mid-run used to finish the download with
                                // nobody left to write the manifest.
                                // Snapshot EVERYTHING the manifest needs before
                                // starting (#836 item 1). Only regionName and
                                // center used to be hoisted; radius, max zoom
                                // and source were read from @State inside this
                                // escaping closure, which runs minutes later
                                // when the download finishes — so adjusting a
                                // slider or the source picker mid-run recorded
                                // a region nobody had fetched. The spec is the
                                // same value handed to the downloader, so the
                                // two cannot disagree.
                                let regionName = name.isEmpty ? defaultName : name
                                let downloadedSpec = spec
                                let downloadedSource = source
                                downloader.start(region: downloadedSpec, source: downloadedSource) { tiles, byteCount in
                                    store.add(OfflineRegion(
                                        name: regionName,
                                        spec: downloadedSpec,
                                        source: downloadedSource,
                                        tileCount: tiles,
                                        bytes: byteCount,
                                        savedAt: Date()))
                                }
                            } label: {
                                Label("Download \(tileCount) tiles", systemImage: "arrow.down.circle.fill")
                                    .fontWeight(.semibold)
                                    .frame(maxWidth: .infinity)
                            }
                        }
                    }
                }
            }
            .navigationTitle("Save area offline")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel") { dismiss() }.disabled(downloader.isRunning)
                }
            }
            .onChange(of: downloader.phase) { phase in
                guard phase == .finished else { return }
                dismiss()
            }
        }
    }

    private var defaultName: String {
        String(format: "Area %.3f, %.3f", region.center.latitude, region.center.longitude)
    }

    private func sliderRow(title: String, value: String,
                           binding: Binding<Double>, range: ClosedRange<Double>,
                           step: Double) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            HStack {
                Text(title).foregroundColor(.secondary)
                Spacer()
                Text(value).font(.system(.body, design: .monospaced))
            }
            Slider(value: binding, in: range, step: step)
        }
    }
}
