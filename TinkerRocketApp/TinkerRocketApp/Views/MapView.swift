//
//  MapView.swift
//  TinkerRocketApp
//
//  Map view showing rocket's GPS position
//

import SwiftUI
import MapKit

// MARK: - UIViewRepresentable MKMapView wrapper

/// Annotation subclass so the renderer can tell rocket vs predicted-landing
/// pins apart by type, without smuggling a flag through the subtitle.
private final class PredictedLandingAnnotation: MKPointAnnotation {}

struct RocketMapView: UIViewRepresentable {
    /// Trial 0 (offline maps): selectable basemap. Apple cases use the native
    /// basemap; tile cases draw a basemap-replacing MKTileOverlay so we can A/B
    /// cache-permissive imagery on-device. See docs/plans/offline-maps.md.
    @Binding var tileSource: TileSource
    var rocketCoordinate: CLLocationCoordinate2D?
    var rocketSubtitle: String?
    /// Predicted landing location (issue #156).  Nil until the predictor
    /// has a valid prediction.  Once set, it stays drawn — even after
    /// telemetry stops — so the operator has a recovery target.
    var predictedLandingCoordinate: CLLocationCoordinate2D?
    var predictedLandingSubtitle: String?
    /// Descent path from snapshot → landing (dashed line overlay).
    var predictedDescentTrack: [CLLocationCoordinate2D] = []
    /// Uncertainty radius (m) drawn as a circle around the predicted-landing
    /// pin (#191).  Fixed per prediction — it shrinks as re-predictions run
    /// lower, and stays frozen once latched; 0 draws nothing.
    var predictedUncertaintyRadiusM: Double = 0
    @Binding var region: MKCoordinateRegion

    func makeUIView(context: Context) -> MKMapView {
        let mapView = MKMapView()
        mapView.delegate = context.coordinator
        mapView.mapType = tileSource.appleMapType
        // Where the flyer is standing. The whole screen is about walking to
        // the rocket, and without this the map never showed one end of that
        // walk. MapKit draws nothing until location is authorised, so this
        // is safe regardless of permission state.
        mapView.showsUserLocation = true
        mapView.setRegion(region, animated: false)
        context.coordinator.basemap.apply(tileSource, to: mapView)
        return mapView
    }

    func updateUIView(_ mapView: MKMapView, context: Context) {
        // Basemap: Apple mapType + optional tile overlay (Trial 0 source A/B).
        if mapView.mapType != tileSource.appleMapType {
            mapView.mapType = tileSource.appleMapType
        }
        context.coordinator.basemap.apply(tileSource, to: mapView)

        // Reset annotations + DATA overlays each pass (cheap).  The basemap tile
        // overlay is managed separately above so it isn't torn down/reloaded on
        // every telemetry tick.
        // MKUserLocation lives in `annotations` too, and it is MapKit's, not
        // ours — tearing it down every telemetry tick fights the framework
        // for the one annotation we do not manage.
        mapView.removeAnnotations(mapView.annotations.filter { !($0 is MKUserLocation) })
        mapView.removeOverlays(mapView.overlays.filter { !($0 is MKTileOverlay) })

        if let coordinate = rocketCoordinate {
            let annotation = MKPointAnnotation()
            annotation.coordinate = coordinate
            annotation.title = "TinkerRocket"
            annotation.subtitle = rocketSubtitle
            mapView.addAnnotation(annotation)
        }

        if let landing = predictedLandingCoordinate {
            let annotation = PredictedLandingAnnotation()
            annotation.coordinate = landing
            annotation.title = "Predicted Landing"
            annotation.subtitle = predictedLandingSubtitle
            mapView.addAnnotation(annotation)

            // Uncertainty circle (#191).  Added before the descent polyline
            // so the dashed track renders on top of the fill.
            if predictedUncertaintyRadiusM > 0 {
                mapView.addOverlay(MKCircle(center: landing,
                                            radius: predictedUncertaintyRadiusM))
            }
        }

        if predictedDescentTrack.count >= 2 {
            var coords = predictedDescentTrack
            let line = MKPolyline(coordinates: &coords, count: coords.count)
            line.title = "predicted-descent"
            mapView.addOverlay(line)
        }

        // Update region if significantly different (avoid fighting user gestures)
        let centerDelta = abs(mapView.region.center.latitude - region.center.latitude) +
                          abs(mapView.region.center.longitude - region.center.longitude)
        if centerDelta > 0.0001 && !context.coordinator.userIsInteracting {
            // Snap large jumps (e.g. the first center from the default location
            // to the rocket); an animated cross-region setRegion streams tiles
            // along the whole path and visibly hitches. Animate small re-centers.
            mapView.setRegion(region, animated: centerDelta < 1.0)
        }
    }

    func makeCoordinator() -> Coordinator {
        Coordinator(self)
    }

    class Coordinator: NSObject, MKMapViewDelegate {
        var parent: RocketMapView
        var userIsInteracting = false
        /// Shared basemap overlay controller (swaps tiles only on source change).
        let basemap = BasemapOverlayController()

        init(_ parent: RocketMapView) {
            self.parent = parent
        }

        func mapView(_ mapView: MKMapView, regionWillChangeAnimated animated: Bool) {
            // Check if the change is user-initiated (gesture recognizer active)
            if let gestureRecognizers = mapView.subviews.first?.gestureRecognizers {
                for recognizer in gestureRecognizers {
                    if recognizer.state == .began || recognizer.state == .changed {
                        userIsInteracting = true
                        return
                    }
                }
            }
        }

        func mapView(_ mapView: MKMapView, regionDidChangeAnimated animated: Bool) {
            userIsInteracting = false
            // Defer the binding write to the next runloop so we never mutate
            // SwiftUI state during a view-update pass. updateUIView calls
            // setRegion(), which feeds back here; writing parent.region inline
            // triggers "Modifying state during view update" and a re-render
            // churn that freezes the map. Async breaks the cycle while still
            // tracking the map (so user pans aren't snapped back).
            DispatchQueue.main.async { [weak self] in
                self?.parent.region = mapView.region
            }
        }

        func mapView(_ mapView: MKMapView,
                     viewFor annotation: MKAnnotation) -> MKAnnotationView? {
            // nil = MapKit's own blue dot. Without this the fall-through
            // below would give the flyer's own position the red rocket pin,
            // i.e. two rockets on the map and no way to tell them apart.
            if annotation is MKUserLocation { return nil }
            if annotation is PredictedLandingAnnotation {
                let id = "PredictedLandingPin"
                var view = mapView.dequeueReusableAnnotationView(withIdentifier: id)
                    as? MKMarkerAnnotationView
                if view == nil {
                    view = MKMarkerAnnotationView(annotation: annotation,
                                                   reuseIdentifier: id)
                    view?.glyphImage = UIImage(systemName: "flag.fill")
                    view?.markerTintColor = .systemGreen
                    view?.canShowCallout = true
                } else {
                    view?.annotation = annotation
                }
                return view
            }
            let identifier = "RocketPin"
            var view = mapView.dequeueReusableAnnotationView(withIdentifier: identifier)
                as? MKMarkerAnnotationView
            if view == nil {
                view = MKMarkerAnnotationView(
                    annotation: annotation, reuseIdentifier: identifier
                )
                view?.glyphImage = UIImage(systemName: "location.fill")
                view?.markerTintColor = .systemRed
                view?.canShowCallout = true
            } else {
                view?.annotation = annotation
            }
            return view
        }

        func mapView(_ mapView: MKMapView,
                     rendererFor overlay: MKOverlay) -> MKOverlayRenderer {
            if let tile = overlay as? MKTileOverlay {
                return MKTileOverlayRenderer(tileOverlay: tile)
            }
            if let line = overlay as? MKPolyline,
               line.title == "predicted-descent" {
                let r = MKPolylineRenderer(polyline: line)
                r.strokeColor = .systemGreen.withAlphaComponent(0.85)
                r.lineWidth = 2.5
                r.lineDashPattern = [6, 6]  // dashed = "this is a prediction"
                return r
            }
            if let circle = overlay as? MKCircle {
                // Uncertainty disc around the predicted-landing pin (#191):
                // prediction-green like the pin/track, near-transparent fill
                // so the basemap stays readable during a recovery walk.
                let r = MKCircleRenderer(circle: circle)
                r.strokeColor = .systemGreen.withAlphaComponent(0.7)
                r.lineWidth = 1.5
                r.fillColor = .systemGreen.withAlphaComponent(0.12)
                return r
            }
            return MKOverlayRenderer(overlay: overlay)
        }
    }
}

// MARK: - Main Map View

struct MapView: View {
    @ObservedObject var device: BLEDevice
    @ObservedObject var locationManager: LocationManager
    @EnvironmentObject var profileStore: RocketProfileStore

    @StateObject private var landingPredictor = LandingPredictor()

    @State private var tileSource: TileSource = .appleHybrid
    // Only a placeholder until onAppear picks the rocket or the phone; it is
    // never what the flyer sees unless both are unavailable.
    @State private var region = MKCoordinateRegion(
        center: CLLocationCoordinate2D(latitude: 37.334_900, longitude: -122.009_020),
        span: MKCoordinateSpan(latitudeDelta: 0.01, longitudeDelta: 0.01)
    )
    @State private var hasInitializedRegion = false
    @State private var hasCenteredOnPhone = false
    @State private var showOfflineMaps = false

    /// Always reads from the latched last-valid fix (#140), not raw
    /// telemetry — so a rocket-side GPS dropout or LoRa loss can't blank
    /// the marker.  A fresh BLE reconnect hydrates this from the fleet
    /// cache on the first telemetry packet.
    private var rocketCoordinate: CLLocationCoordinate2D? {
        device.lastValidRocketFix?.coordinate
    }

    var body: some View {
        ZStack(alignment: .topTrailing) {
            // TimelineView so the "last fix Ns ago" subtitle keeps ticking
            // even when telemetry has stopped arriving — without it the age
            // would freeze at whatever it was on the most recent re-render.
            TimelineView(.periodic(from: .now, by: 1.0)) { context in
                RocketMapView(
                    tileSource: $tileSource,
                    rocketCoordinate: rocketCoordinate,
                    rocketSubtitle: markerSubtitle(now: context.date),
                    predictedLandingCoordinate: landingPredictor.prediction?.landing,
                    predictedLandingSubtitle: predictedSubtitle(now: context.date),
                    predictedDescentTrack: descentTrackCoords(),
                    predictedUncertaintyRadiusM: landingPredictor.prediction?.uncertaintyMeters ?? 0,
                    region: $region
                )
            }
            .ignoresSafeArea(edges: .bottom)

            // Top-leading staleness badge — only shown while a prediction
            // exists.  Ticks with the same TimelineView upstream so the
            // age string stays live.
            if landingPredictor.prediction != nil {
                TimelineView(.periodic(from: .now, by: 1.0)) { context in
                    predictionBadge(now: context.date)
                }
                .padding(.leading, 12)
                .padding(.top, 12)
                .frame(maxWidth: .infinity, alignment: .topLeading)
            }

            // Offline indicator (top-center) when there's no connection.
            OfflinePill()
                .padding(.top, 12)
                .frame(maxWidth: .infinity, alignment: .top)

            // Active source + attribution, so it's clear which provider renders.
            // The name carries the "(online)" marker too: this plate is the
            // one place the active basemap is always visible, so it is where
            // "this will go blank out of signal" belongs.
            VStack(alignment: .leading, spacing: 1) {
                Text(tileSource.pickerLabel)
                    .font(.caption.bold())
                if let attr = tileSource.attribution {
                    Text(attr)
                        .font(.system(size: 9))
                        .foregroundColor(.secondary)
                }
            }
            .padding(.horizontal, 8)
            .padding(.vertical, 5)
            .background(.ultraThinMaterial)
            .cornerRadius(8)
            .shadow(radius: 2)
            .padding(.leading, 12)
            .padding(.bottom, 12)
            .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .bottomLeading)

            // Floating control buttons
            VStack(spacing: 12) {
                // Map source picker + offline downloads (shared with Drift Cast).
                Menu {
                    // A Picker (not Buttons) so the active row gets the
                    // framework checkmark; rows carry the online/offline
                    // marker, which is what makes "Manage offline maps…"
                    // below read as the answer to it.
                    Picker("Map source", selection: $tileSource) {
                        ForEach(TileSource.allCases) { src in
                            Label(src.pickerLabel, systemImage: src.symbol).tag(src)
                        }
                    }
                    Divider()
                    Button {
                        showOfflineMaps = true
                    } label: {
                        Label("Manage offline maps…", systemImage: "arrow.down.circle")
                    }
                } label: {
                    Image(systemName: "square.stack.3d.up")
                        .font(.system(size: 20))
                        .foregroundColor(.primary)
                        .frame(width: 44, height: 44)
                        .background(.ultraThinMaterial)
                        .cornerRadius(10)
                        .shadow(radius: 2)
                }

                // Re-center on rocket
                Button(action: {
                    centerOnRocket()
                }) {
                    Image(systemName: "location.fill")
                        .font(.system(size: 20))
                        .foregroundColor(.blue)
                        .frame(width: 44, height: 44)
                        .background(.ultraThinMaterial)
                        .cornerRadius(10)
                        .shadow(radius: 2)
                }
            }
            .padding(.trailing, 12)
            .padding(.top, 12)
        }
        .navigationTitle("Rocket Map")
        .navigationBarTitleDisplayMode(.inline)
        .brandedToolbar()
        .onChange(of: device.lastValidRocketFix) { _ in
            if !hasInitializedRegion {
                centerOnRocket()
                hasInitializedRegion = true
            }
        }
        .onReceive(locationManager.$userLocation) { _ in
            // A cold GPS lands well after the map does; take the first fix,
            // but never over a rocket we are already showing.
            if rocketCoordinate == nil { centerOnPhone() }
        }
        .onAppear {
            centerOnRocket()
            if rocketCoordinate == nil { centerOnPhone() }
            landingPredictor.attach(device: device, profileStore: profileStore)
        }
        .onDisappear {
            landingPredictor.detach()
        }
        .sheet(isPresented: $showOfflineMaps) {
            OfflineMapsView(suggestedCenter: region.center)
        }
    }

    /// SwiftUI badge displayed top-leading on the map.  Color-graded by
    /// staleness so a forgotten/frozen prediction doesn't read as live.
    @ViewBuilder
    private func predictionBadge(now: Date) -> some View {
        if let p = landingPredictor.prediction {
            let age = max(0, now.timeIntervalSince(p.sampleAt))
            let color: Color = {
                if p.snapshotSource == .latched { return .orange }
                if age < 5  { return .green }
                if age < 30 { return .yellow }
                return .red
            }()
            HStack(spacing: 6) {
                Image(systemName: "flag.fill")
                    .foregroundColor(color)
                Text("Predicted landing • T+\(formatAge(Int(age))) ago")
                    .font(.caption.monospacedDigit())
            }
            .padding(.horizontal, 10)
            .padding(.vertical, 6)
            .background(.ultraThinMaterial)
            .cornerRadius(8)
            .shadow(radius: 2)
        }
    }

    /// Callout text under the predicted-landing pin.
    private func predictedSubtitle(now: Date) -> String? {
        guard let p = landingPredictor.prediction else { return nil }
        let age = max(0, Int(now.timeIntervalSince(p.sampleAt)))
        let basis = (p.snapshotSource == .latched) ? "latched" : "live"
        let r = p.uncertaintyMeters
        let radius = r > 0 ? String(format: " ±%.0f m", r) : ""
        return "From telemetry \(formatAge(age)) ago (\(basis))\(radius)"
    }

    /// Polyline coordinates for the predicted descent track.  The track
    /// itself is in (lat, lon) — drop altitude/time for the 2D overlay.
    private func descentTrackCoords() -> [CLLocationCoordinate2D] {
        guard let pts = landingPredictor.prediction?.descentTrack else { return [] }
        return pts.map { CLLocationCoordinate2D(latitude: $0.lat, longitude: $0.lon) }
    }

    private func centerOnRocket() {
        guard let coordinate = rocketCoordinate else { return }
        region = MKCoordinateRegion(
            center: coordinate,
            span: MKCoordinateSpan(latitudeDelta: 0.005, longitudeDelta: 0.005)
        )
    }

    /// Opening point when the rocket has no fix yet — which is the normal
    /// case on the pad, where this screen is opened.  The phone knows where
    /// it is, so start there instead of on a default that is nowhere near
    /// the flyer.  Deliberately does NOT set `hasInitializedRegion`: the
    /// rocket's first fix must still take the camera.
    private func centerOnPhone() {
        guard !hasCenteredOnPhone,
              let coordinate = locationManager.userLocation,
              CLLocationCoordinate2DIsValid(coordinate) else { return }
        region = MKCoordinateRegion(
            center: coordinate,
            span: MKCoordinateSpan(latitudeDelta: 0.01, longitudeDelta: 0.01)
        )
        hasCenteredOnPhone = true
    }

    /// Callout text under the marker.  Shows sat count and the age of
    /// the fix so a stale marker can't be mistaken for a live one.
    private func markerSubtitle(now: Date) -> String? {
        guard let fix = device.lastValidRocketFix else { return nil }
        let age = max(0, Int(now.timeIntervalSince(fix.fixDate)))
        return "\(fix.numSats) sats • \(formatAge(age)) ago"
    }

    /// Compact age string: bare seconds under a minute, M:SS or H:MM:SS
    /// above.  Avoids the "0:05" noise of the longer formatter on very
    /// fresh fixes where seconds is the only thing that matters.
    private func formatAge(_ seconds: Int) -> String {
        if seconds < 60 { return "\(seconds)s" }
        return formatElapsed(seconds: seconds)
    }
}

// MARK: - Preview

// Preview requires a connected BLEDevice — omitted for now.
