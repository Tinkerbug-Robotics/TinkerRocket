//
//  MapView.swift
//  TinkerRocketApp
//
//  Map view showing rocket's GPS position
//

import SwiftUI
import MapKit

// MARK: - UIViewRepresentable MKMapView wrapper

struct RocketMapView: UIViewRepresentable {
    @Binding var mapType: MKMapType
    var rocketCoordinate: CLLocationCoordinate2D?
    @Binding var region: MKCoordinateRegion

    func makeUIView(context: Context) -> MKMapView {
        let mapView = MKMapView()
        mapView.delegate = context.coordinator
        mapView.mapType = mapType
        mapView.setRegion(region, animated: false)
        return mapView
    }

    func updateUIView(_ mapView: MKMapView, context: Context) {
        // Update map type when toggle changes
        if mapView.mapType != mapType {
            mapView.mapType = mapType
        }

        // Update annotation
        mapView.removeAnnotations(mapView.annotations)
        if let coordinate = rocketCoordinate {
            let annotation = MKPointAnnotation()
            annotation.coordinate = coordinate
            annotation.title = "TinkerRocket"
            mapView.addAnnotation(annotation)
        }

        // Update region if significantly different (avoid fighting user gestures)
        let centerDelta = abs(mapView.region.center.latitude - region.center.latitude) +
                          abs(mapView.region.center.longitude - region.center.longitude)
        if centerDelta > 0.0001 && !context.coordinator.userIsInteracting {
            mapView.setRegion(region, animated: true)
        }
    }

    func makeCoordinator() -> Coordinator {
        Coordinator(self)
    }

    class Coordinator: NSObject, MKMapViewDelegate {
        var parent: RocketMapView
        var userIsInteracting = false

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
            parent.region = mapView.region
            userIsInteracting = false
        }

        func mapView(_ mapView: MKMapView,
                     viewFor annotation: MKAnnotation) -> MKAnnotationView? {
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
    }
}

// MARK: - Main Map View

struct MapView: View {
    @ObservedObject var device: BLEDevice

    @State private var mapType: MKMapType = .hybrid
    @State private var region = MKCoordinateRegion(
        center: CLLocationCoordinate2D(latitude: 37.334_900, longitude: -122.009_020),
        span: MKCoordinateSpan(latitudeDelta: 0.01, longitudeDelta: 0.01)
    )
    @State private var hasInitializedRegion = false

    private var rocketCoordinate: CLLocationCoordinate2D? {
        guard let lat = device.telemetry.latitude,
              let lon = device.telemetry.longitude,
              !(lat == 0 && lon == 0) else {
            return nil
        }
        return CLLocationCoordinate2D(latitude: lat, longitude: lon)
    }

    var body: some View {
        ZStack(alignment: .topTrailing) {
            RocketMapView(
                mapType: $mapType,
                rocketCoordinate: rocketCoordinate,
                region: $region
            )
            .ignoresSafeArea(edges: .bottom)

            // Floating control buttons
            VStack(spacing: 12) {
                // Map type toggle
                Button(action: {
                    mapType = (mapType == .standard) ? .hybrid : .standard
                }) {
                    Image(systemName: mapType == .standard
                          ? "globe.americas.fill" : "map.fill")
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
        .onChange(of: device.telemetry.latitude) { _ in
            if !hasInitializedRegion {
                centerOnRocket()
                hasInitializedRegion = true
            }
        }
        .onAppear {
            centerOnRocket()
        }
    }

    private func centerOnRocket() {
        guard let coordinate = rocketCoordinate else { return }
        region = MKCoordinateRegion(
            center: coordinate,
            span: MKCoordinateSpan(latitudeDelta: 0.005, longitudeDelta: 0.005)
        )
    }
}

// MARK: - Preview

// Preview requires a connected BLEDevice — omitted for now.
