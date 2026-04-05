//
//  DriftCastView.swift
//  TinkerRocketApp
//
//  Reverse drift cast guidance point calculator.
//  Computes where to steer the rocket at apogee so wind drift
//  during parachute descent lands it at the target location.
//

import SwiftUI
import MapKit
import CoreLocation
import SceneKit

// MARK: - Map Representable

/// MapKit wrapper with tap gesture, annotations, and polyline overlays.
struct DriftCastMapView: UIViewRepresentable {
    @Binding var mapType: MKMapType
    var launchCoord: CLLocationCoordinate2D?
    var landingCoord: CLLocationCoordinate2D?
    var guidanceCoord: CLLocationCoordinate2D?
    var descentTrack: [CLLocationCoordinate2D]
    var boostLine: [CLLocationCoordinate2D]
    @Binding var region: MKCoordinateRegion
    var onTap: ((CLLocationCoordinate2D) -> Void)?

    func makeUIView(context: Context) -> MKMapView {
        let mapView = MKMapView()
        mapView.delegate = context.coordinator
        mapView.mapType = mapType
        mapView.setRegion(region, animated: false)

        // Add tap gesture
        let tap = UITapGestureRecognizer(target: context.coordinator, action: #selector(Coordinator.handleTap(_:)))
        mapView.addGestureRecognizer(tap)

        return mapView
    }

    func updateUIView(_ mapView: MKMapView, context: Context) {
        if mapView.mapType != mapType {
            mapView.mapType = mapType
        }

        // Update annotations
        mapView.removeAnnotations(mapView.annotations)

        if let coord = launchCoord {
            let ann = MKPointAnnotation()
            ann.coordinate = coord
            ann.title = "Launch Pad"
            ann.subtitle = "launch"
            mapView.addAnnotation(ann)
        }
        if let coord = landingCoord {
            let ann = MKPointAnnotation()
            ann.coordinate = coord
            ann.title = "Landing Target"
            ann.subtitle = "landing"
            mapView.addAnnotation(ann)
        }
        if let coord = guidanceCoord {
            let ann = MKPointAnnotation()
            ann.coordinate = coord
            ann.title = "Guidance Point"
            ann.subtitle = "guidance"
            mapView.addAnnotation(ann)
        }

        // Update overlays
        mapView.removeOverlays(mapView.overlays)

        if descentTrack.count > 1 {
            let polyline = MKPolyline(coordinates: descentTrack, count: descentTrack.count)
            polyline.title = "descent"
            mapView.addOverlay(polyline)
        }
        if boostLine.count == 2 {
            let polyline = MKPolyline(coordinates: boostLine, count: 2)
            polyline.title = "boost"
            mapView.addOverlay(polyline)
        }

        // Update region if significantly different
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
        var parent: DriftCastMapView
        var userIsInteracting = false

        init(_ parent: DriftCastMapView) {
            self.parent = parent
        }

        @objc func handleTap(_ gesture: UITapGestureRecognizer) {
            guard let mapView = gesture.view as? MKMapView else { return }
            let point = gesture.location(in: mapView)
            let coord = mapView.convert(point, toCoordinateFrom: mapView)
            parent.onTap?(coord)
        }

        func mapView(_ mapView: MKMapView, regionWillChangeAnimated animated: Bool) {
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

        func mapView(_ mapView: MKMapView, viewFor annotation: MKAnnotation) -> MKAnnotationView? {
            guard let pointAnn = annotation as? MKPointAnnotation else { return nil }

            let identifier = "DriftCastPin"
            var view = mapView.dequeueReusableAnnotationView(withIdentifier: identifier) as? MKMarkerAnnotationView
            if view == nil {
                view = MKMarkerAnnotationView(annotation: annotation, reuseIdentifier: identifier)
                view?.canShowCallout = true
            } else {
                view?.annotation = annotation
            }

            switch pointAnn.subtitle {
            case "launch":
                view?.markerTintColor = .systemRed
                view?.glyphImage = UIImage(systemName: "location.fill")
            case "landing":
                view?.markerTintColor = .systemGreen
                view?.glyphImage = UIImage(systemName: "flag.fill")
            case "guidance":
                view?.markerTintColor = .systemBlue
                view?.glyphImage = UIImage(systemName: "scope")
            default:
                view?.markerTintColor = .systemGray
            }

            return view
        }

        func mapView(_ mapView: MKMapView, rendererFor overlay: MKOverlay) -> MKOverlayRenderer {
            if let polyline = overlay as? MKPolyline {
                let renderer = MKPolylineRenderer(polyline: polyline)
                if polyline.title == "descent" {
                    renderer.strokeColor = UIColor.systemPurple
                    renderer.lineWidth = 3
                } else if polyline.title == "boost" {
                    renderer.strokeColor = UIColor.systemBlue.withAlphaComponent(0.5)
                    renderer.lineWidth = 2
                    renderer.lineDashPattern = [8, 6]
                }
                return renderer
            }
            return MKOverlayRenderer(overlay: overlay)
        }
    }
}


// MARK: - 3D Trajectory View (SceneKit + ArcGIS Satellite Imagery)

/// Native SceneKit 3D view for visualizing the descent trajectory at altitude.
/// Uses cylinder segments for polylines, spheres for markers, and
/// ArcGIS satellite tiles (via URLSession) textured onto the ground plane.
/// Supports interactive rotation/zoom via allowsCameraControl.
struct Trajectory3DView: UIViewRepresentable {
    var result: GuidanceResult?

    func makeUIView(context: Context) -> SCNView {
        let scnView = SCNView()
        scnView.backgroundColor = UIColor(red: 0.04, green: 0.04, blue: 0.10, alpha: 1.0)
        scnView.allowsCameraControl = true
        scnView.autoenablesDefaultLighting = true
        scnView.antialiasingMode = .multisampling4X
        return scnView
    }

    func updateUIView(_ scnView: SCNView, context: Context) {
        guard let r = result else {
            scnView.scene = nil
            return
        }
        // Only rebuild if result changed
        let hash = "\(r.guidanceLat),\(r.guidanceLon),\(r.apogeeFt)"
        if context.coordinator.lastHash != hash {
            context.coordinator.lastHash = hash
            let (scene, groundNode, extent) = Self.buildScene(for: r)
            scnView.scene = scene
            // Explicitly set the point of view so camera control works
            scnView.pointOfView = scene.rootNode.childNode(
                withName: "mainCamera", recursively: false)

            // Fetch ArcGIS satellite imagery asynchronously
            Self.fetchArcGISImagery(
                refLat: r.launchLat, refLon: r.launchLon,
                extent: extent, groundNode: groundNode,
                sceneRoot: scene.rootNode
            )
        }
    }

    func makeCoordinator() -> Coordinator { Coordinator() }
    class Coordinator { var lastHash = "" }

    // MARK: - Geo Conversion

    /// Convert lat/lon/alt to local scene coordinates (meters).
    /// X = east, Y = up, Z = south (relative to launch pad origin).
    private static func localPos(lat: Double, lon: Double, altM: Double,
                                  refLat: Double, refLon: Double) -> SCNVector3 {
        let x = Float((lon - refLon) * 111_320 * cos(refLat * .pi / 180))
        let z = Float(-(lat - refLat) * 110_540)
        return SCNVector3(x, Float(altM), z)
    }

    // MARK: - ArcGIS Satellite Imagery

    /// Fetch satellite image from ArcGIS MapServer export API (no API key needed)
    /// and apply as the ground plane texture.
    private static func fetchArcGISImagery(
        refLat: Double, refLon: Double, extent: Float,
        groundNode: SCNNode, sceneRoot: SCNNode
    ) {
        let halfM = Double(extent * 1.5)
        let mPerDegLat = 110_540.0
        let mPerDegLon = 111_320.0 * cos(refLat * .pi / 180)

        let south = refLat - halfM / mPerDegLat
        let north = refLat + halfM / mPerDegLat
        let west = refLon - halfM / mPerDegLon
        let east = refLon + halfM / mPerDegLon

        let bbox = "\(west),\(south),\(east),\(north)"
        let urlStr = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/export"
            + "?bbox=\(bbox)&bboxSR=4326&imageSR=4326"
            + "&size=1024,1024&format=png&f=image"

        guard let url = URL(string: urlStr) else { return }

        URLSession.shared.dataTask(with: url) { data, response, error in
            guard let data = data,
                  let httpResp = response as? HTTPURLResponse,
                  httpResp.statusCode == 200,
                  let image = UIImage(data: data) else { return }
            DispatchQueue.main.async {
                groundNode.geometry?.firstMaterial?.diffuse.contents = image
                groundNode.geometry?.firstMaterial?.diffuse.wrapS = .clamp
                groundNode.geometry?.firstMaterial?.diffuse.wrapT = .clamp
                groundNode.geometry?.firstMaterial?.lightingModel = .constant
                // Remove grid overlay now that we have imagery
                sceneRoot.childNode(withName: "groundGrid", recursively: false)?
                    .removeFromParentNode()
            }
        }.resume()
    }

    // MARK: - Scene Builder

    /// Build the 3D scene. Returns the scene, the ground node (for texture update),
    /// and the extent (for satellite fetch region).
    static func buildScene(for r: GuidanceResult) -> (SCNScene, SCNNode, Float) {
        let scene = SCNScene()
        let root = scene.rootNode
        let refLat = r.launchLat
        let refLon = r.launchLon
        let apogeeM = r.apogeeFt * 0.3048

        // Collect all positions for extent calculation
        let launchPos = SCNVector3(0, 0, 0)
        let guidancePos = localPos(lat: r.guidanceLat, lon: r.guidanceLon,
                                    altM: apogeeM, refLat: refLat, refLon: refLon)
        let landingPos = localPos(lat: r.forwardLandingLat, lon: r.forwardLandingLon,
                                   altM: 0, refLat: refLat, refLon: refLon)

        var trackPositions: [SCNVector3] = []
        for pt in r.descentTrack {
            trackPositions.append(localPos(lat: pt.lat, lon: pt.lon,
                                           altM: pt.altAglFt * 0.3048,
                                           refLat: refLat, refLon: refLon))
        }

        let allPos = [launchPos, guidancePos, landingPos] + trackPositions
        let xs = allPos.map(\.x), ys = allPos.map(\.y), zs = allPos.map(\.z)
        let extent = max(
            (xs.max()! - xs.min()!),
            (ys.max()! - ys.min()!),
            (zs.max()! - zs.min()!),
            100
        )
        let markerSize = CGFloat(extent * 0.016)

        // --- Ground plane (dark initially, satellite imagery applied async) ---
        let ground = SCNPlane(width: CGFloat(extent * 3), height: CGFloat(extent * 3))
        let groundMat = SCNMaterial()
        groundMat.diffuse.contents = UIColor(white: 0.10, alpha: 0.85)
        groundMat.isDoubleSided = true
        ground.materials = [groundMat]
        let groundNode = SCNNode(geometry: ground)
        groundNode.eulerAngles.x = -.pi / 2
        groundNode.name = "ground"
        root.addChildNode(groundNode)

        // --- Grid lines (shown until satellite imagery loads) ---
        let gridSpacing = Self.niceGridSpacing(extent)
        Self.addGroundGrid(to: root, extent: extent, spacing: gridSpacing)

        // --- Markers ---
        Self.addMarker(to: root, position: launchPos,
                       color: UIColor(red: 1.0, green: 0.42, blue: 0.42, alpha: 1),
                       radius: markerSize)
        Self.addMarker(to: root, position: landingPos,
                       color: UIColor(red: 0.32, green: 0.81, blue: 0.40, alpha: 1),
                       radius: markerSize)
        Self.addMarker(to: root, position: guidancePos,
                       color: UIColor(red: 0.30, green: 0.67, blue: 0.97, alpha: 1),
                       radius: markerSize * 1.3)

        // --- Descent track (purple tubes) ---
        if trackPositions.count >= 2 {
            Self.addTubeLine(to: root, points: trackPositions,
                            color: UIColor(red: 0.59, green: 0.46, blue: 0.98, alpha: 1),
                            radius: CGFloat(extent * 0.004))
        }

        // --- Boost trajectory (blue tubes from pad to guidance) ---
        Self.addTubeLine(to: root, points: [launchPos, guidancePos],
                        color: UIColor(red: 0.30, green: 0.67, blue: 0.97, alpha: 0.5),
                        radius: CGFloat(extent * 0.002))

        // --- Vertical drop line (thin, from guidance down to ground) ---
        let guidanceGround = SCNVector3(guidancePos.x, 0, guidancePos.z)
        Self.addTubeLine(to: root, points: [guidancePos, guidanceGround],
                        color: UIColor(red: 0.30, green: 0.67, blue: 0.97, alpha: 0.25),
                        radius: CGFloat(extent * 0.001))

        // --- Labels ---
        let labelScale = Float(extent * 0.0006)
        Self.addLabel(to: root, text: "Launch Pad",
                     position: SCNVector3(launchPos.x, Float(markerSize) * 2.5, launchPos.z),
                     color: .white, scale: labelScale)
        Self.addLabel(to: root, text: "Landing",
                     position: SCNVector3(landingPos.x, Float(markerSize) * 2.5, landingPos.z),
                     color: .white, scale: labelScale)
        Self.addLabel(to: root, text: "Guidance",
                     position: SCNVector3(guidancePos.x, guidancePos.y + Float(markerSize) * 2.5, guidancePos.z),
                     color: UIColor(red: 0.30, green: 0.67, blue: 0.97, alpha: 1), scale: labelScale)
        let altLabel = String(format: "%.0f ft AGL", r.apogeeFt)
        Self.addLabel(to: root, text: altLabel,
                     position: SCNVector3(guidancePos.x, guidancePos.y + Float(markerSize) * 1.0, guidancePos.z),
                     color: UIColor(white: 0.7, alpha: 1), scale: labelScale * 0.7)

        // --- Camera (side view, perpendicular to guidance bearing) ---
        let camNode = SCNNode()
        camNode.name = "mainCamera"
        camNode.camera = SCNCamera()
        camNode.camera?.zFar = Double(extent * 20)
        camNode.camera?.zNear = Double(max(extent * 0.001, 0.1))
        camNode.camera?.fieldOfView = 50

        let cx = (xs.min()! + xs.max()!) / 2
        let cy = (ys.min()! + ys.max()!) / 2
        let cz = (zs.min()! + zs.max()!) / 2

        let trackAngle = atan2(guidancePos.x, -guidancePos.z)
        let viewAngle = trackAngle + .pi / 2
        let dist = extent * 1.8
        camNode.position = SCNVector3(
            cx + dist * sin(viewAngle),
            cy + dist * 0.4,
            cz - dist * cos(viewAngle)
        )
        camNode.look(at: SCNVector3(cx, cy, cz))
        root.addChildNode(camNode)

        // --- Lighting ---
        let ambientNode = SCNNode()
        ambientNode.light = SCNLight()
        ambientNode.light?.type = .ambient
        ambientNode.light?.intensity = 700
        ambientNode.light?.color = UIColor(white: 1.0, alpha: 1)
        root.addChildNode(ambientNode)

        let dirNode = SCNNode()
        dirNode.light = SCNLight()
        dirNode.light?.type = .directional
        dirNode.light?.intensity = 600
        dirNode.position = SCNVector3(0, Float(extent), 0)
        dirNode.look(at: SCNVector3(cx, 0, cz))
        root.addChildNode(dirNode)

        return (scene, groundNode, extent)
    }

    // MARK: - Scene Helpers

    private static func addMarker(to parent: SCNNode, position: SCNVector3,
                                   color: UIColor, radius: CGFloat) {
        let geo = SCNSphere(radius: radius)
        geo.segmentCount = 24
        let mat = SCNMaterial()
        mat.diffuse.contents = color
        mat.emission.contents = color.withAlphaComponent(0.5)
        geo.materials = [mat]
        let node = SCNNode(geometry: geo)
        node.position = position
        parent.addChildNode(node)
    }

    private static func addTubeLine(to parent: SCNNode, points: [SCNVector3],
                                     color: UIColor, radius: CGFloat) {
        guard points.count >= 2 else { return }
        for i in 0..<(points.count - 1) {
            let a = points[i], b = points[i + 1]
            let dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z
            let len = sqrt(dx * dx + dy * dy + dz * dz)
            guard len > 0.01 else { continue }

            let cyl = SCNCylinder(radius: radius, height: CGFloat(len))
            let mat = SCNMaterial()
            mat.diffuse.contents = color
            mat.emission.contents = color.withAlphaComponent(0.4)
            cyl.materials = [mat]

            let node = SCNNode(geometry: cyl)
            node.position = SCNVector3(
                (a.x + b.x) / 2,
                (a.y + b.y) / 2,
                (a.z + b.z) / 2
            )
            node.look(at: SCNVector3(b.x, b.y, b.z),
                      up: SCNVector3(0, 1, 0),
                      localFront: SCNVector3(0, 1, 0))
            parent.addChildNode(node)
        }
    }

    private static func addLabel(to parent: SCNNode, text: String,
                                  position: SCNVector3, color: UIColor, scale: Float) {
        let textGeo = SCNText(string: text, extrusionDepth: 0.3)
        textGeo.font = UIFont.systemFont(ofSize: 10, weight: .bold)
        let mat = SCNMaterial()
        mat.diffuse.contents = color
        mat.emission.contents = color
        textGeo.materials = [mat]
        textGeo.flatness = 0.1

        let node = SCNNode(geometry: textGeo)
        let (minBound, maxBound) = node.boundingBox
        let midX = (maxBound.x - minBound.x) / 2
        node.pivot = SCNMatrix4MakeTranslation(midX + minBound.x, 0, 0)
        node.position = position
        node.scale = SCNVector3(scale, scale, scale)
        node.constraints = [SCNBillboardConstraint()]
        parent.addChildNode(node)
    }

    private static func niceGridSpacing(_ extent: Float) -> Float {
        let target = extent / 6
        let mag = pow(10, floor(log10(target)))
        let norm = target / mag
        if norm < 2 { return mag }
        if norm < 5 { return 2 * mag }
        return 5 * mag
    }

    private static func addGroundGrid(to parent: SCNNode, extent: Float, spacing: Float) {
        let half = extent * 1.2
        let gridY: Float = 0.05
        let color = UIColor(white: 0.3, alpha: 0.8)

        var vertices: [SCNVector3] = []
        var indices: [Int32] = []
        var idx: Int32 = 0

        var z = -half
        while z <= half {
            vertices.append(SCNVector3(-half, gridY, z))
            vertices.append(SCNVector3(half, gridY, z))
            indices.append(idx); indices.append(idx + 1)
            idx += 2
            z += spacing
        }
        var x = -half
        while x <= half {
            vertices.append(SCNVector3(x, gridY, -half))
            vertices.append(SCNVector3(x, gridY, half))
            indices.append(idx); indices.append(idx + 1)
            idx += 2
            x += spacing
        }

        let src = SCNGeometrySource(vertices: vertices)
        let elem = SCNGeometryElement(indices: indices, primitiveType: .line)
        let geo = SCNGeometry(sources: [src], elements: [elem])
        let mat = SCNMaterial()
        mat.diffuse.contents = color
        mat.emission.contents = color
        mat.isDoubleSided = true
        geo.materials = [mat]

        let node = SCNNode(geometry: geo)
        node.name = "groundGrid"
        parent.addChildNode(node)
    }
}


// MARK: - Main Drift Cast View

struct DriftCastView: View {
    @ObservedObject var bleManager: BLEManager
    @Environment(\.dismiss) var dismiss

    // Map state
    @State private var mapType: MKMapType = .hybrid
    @State private var mapRegion = MKCoordinateRegion(
        center: CLLocationCoordinate2D(latitude: 37.7608, longitude: -75.7446),
        span: MKCoordinateSpan(latitudeDelta: 0.05, longitudeDelta: 0.05)
    )
    @State private var tapMode: TapMode = .launch

    // Input fields (persisted)
    @AppStorage("dc_launchLat") private var launchLat: String = "37.7608"
    @AppStorage("dc_launchLon") private var launchLon: String = "-75.7446"
    @AppStorage("dc_landingLat") private var landingLat: String = "37.7608"
    @AppStorage("dc_landingLon") private var landingLon: String = "-75.7446"
    @AppStorage("dc_apogeeFt") private var apogeeFt: String = "10000"
    @AppStorage("dc_drogueRate") private var drogueRate: String = "60"
    @AppStorage("dc_mainRate") private var mainRate: String = "18"
    @AppStorage("dc_mainDeploy") private var mainDeploy: String = "700"
    @AppStorage("dc_maxSteering") private var maxSteering: String = "25"

    // Launch time (default: next hour from now)
    @State private var launchTime: Date = {
        let now = Date()
        let calendar = Calendar.current
        let comps = calendar.dateComponents([.year, .month, .day, .hour], from: now)
        let hourStart = calendar.date(from: comps) ?? now
        return hourStart.addingTimeInterval(3600)
    }()

    // 3D view toggle
    @State private var show3D = false

    // Compute state
    @State private var result: GuidanceResult?
    @State private var isComputing = false
    @State private var errorMessage: String?
    @State private var showSendConfirm = false
    @State private var showSentAlert = false

    // GPS for "Use GPS" button
    @State private var locationManager = CLLocationManager()
    @State private var lastUserLocation: CLLocationCoordinate2D?

    enum TapMode: String, CaseIterable {
        case launch = "Set Launch"
        case landing = "Set Landing"
    }

    // MARK: - Computed Properties

    private var launchCoord: CLLocationCoordinate2D? {
        guard let lat = Double(launchLat), let lon = Double(launchLon) else { return nil }
        return CLLocationCoordinate2D(latitude: lat, longitude: lon)
    }

    private var landingCoord: CLLocationCoordinate2D? {
        guard let lat = Double(landingLat), let lon = Double(landingLon) else { return nil }
        return CLLocationCoordinate2D(latitude: lat, longitude: lon)
    }

    private var guidanceCoord: CLLocationCoordinate2D? {
        guard let r = result else { return nil }
        return CLLocationCoordinate2D(latitude: r.guidanceLat, longitude: r.guidanceLon)
    }

    private var descentTrackCoords: [CLLocationCoordinate2D] {
        guard let r = result else { return [] }
        return r.descentTrack.map { CLLocationCoordinate2D(latitude: $0.lat, longitude: $0.lon) }
    }

    private var boostLineCoords: [CLLocationCoordinate2D] {
        guard let launch = launchCoord, let guidance = guidanceCoord else { return [] }
        return [launch, guidance]
    }

    private var inputsValid: Bool {
        guard let _ = Double(launchLat), let _ = Double(launchLon),
              let _ = Double(landingLat), let _ = Double(landingLon),
              let a = Double(apogeeFt), a > 0,
              let d = Double(drogueRate), d > 0,
              let m = Double(mainRate), m > 0,
              let md = Double(mainDeploy), md > 0,
              let ms = Double(maxSteering), ms > 0 else {
            return false
        }
        return true
    }

    private var canSend: Bool {
        bleManager.isConnected && result != nil && result!.feasible
    }

    // MARK: - Body

    var body: some View {
        NavigationView {
            ScrollView {
                VStack(spacing: 0) {
                    // Map section
                    mapSection

                    // Form sections
                    formContent
                }
            }
            .background(Color(.systemGroupedBackground))
            .navigationTitle("Reverse Drift Cast")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarLeading) {
                    Button(action: useGPS) {
                        Label("Use GPS", systemImage: "location.fill")
                    }
                }
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") { dismiss() }
                }
            }
            .alert("Guidance Sent", isPresented: $showSentAlert) {
                Button("OK") { }
            } message: {
                if let r = result {
                    Text(String(format: "Guidance point sent to unit:\n%.6f, %.6f\nAltitude: %.0f ft AGL",
                                r.guidanceLat, r.guidanceLon, r.apogeeFt))
                }
            }
            .alert("Send to Unit?", isPresented: $showSendConfirm) {
                Button("Send", role: .none) { sendToUnit() }
                Button("Cancel", role: .cancel) { }
            } message: {
                if let r = result {
                    Text(String(format: "Send guidance point (%.6f, %.6f) at %.0f ft AGL to %@?",
                                r.guidanceLat, r.guidanceLon, r.apogeeFt,
                                bleManager.connectedDeviceName))
                }
            }
        }
    }

    // MARK: - Map Section

    private var mapSection: some View {
        VStack(spacing: 0) {
            // Tap mode picker + 3D toggle
            HStack {
                if !show3D {
                    Picker("Tap Mode", selection: $tapMode) {
                        ForEach(TapMode.allCases, id: \.self) { mode in
                            Text(mode.rawValue)
                        }
                    }
                    .pickerStyle(.segmented)
                } else {
                    Spacer()
                }

                Button(action: { show3D.toggle() }) {
                    Label(show3D ? "2D" : "3D",
                          systemImage: show3D ? "map" : "cube")
                        .font(.caption.bold())
                        .padding(.horizontal, 10)
                        .padding(.vertical, 6)
                        .background(show3D ? Color.purple.opacity(0.2) : Color.blue.opacity(0.2))
                        .foregroundColor(show3D ? .purple : .blue)
                        .cornerRadius(8)
                }
            }
            .padding(.horizontal)
            .padding(.vertical, 8)

            // Map view: 2D or 3D
            if show3D {
                Trajectory3DView(result: result)
                    .frame(height: 350)
                    .clipShape(RoundedRectangle(cornerRadius: 12))
                    .padding(.horizontal)
            } else {
                DriftCastMapView(
                    mapType: $mapType,
                    launchCoord: launchCoord,
                    landingCoord: landingCoord,
                    guidanceCoord: guidanceCoord,
                    descentTrack: descentTrackCoords,
                    boostLine: boostLineCoords,
                    region: $mapRegion,
                    onTap: { coord in
                        handleMapTap(coord)
                    }
                )
                .frame(height: 250)
                .cornerRadius(12)
                .padding(.horizontal)
            }
        }
    }

    // MARK: - Form Content

    private var formContent: some View {
        VStack(spacing: 16) {
            // Coordinates
            sectionCard(title: "Launch Point") {
                coordRow(label: "Latitude", value: $launchLat, color: .red)
                coordRow(label: "Longitude", value: $launchLon, color: .red)
            }

            sectionCard(title: "Landing Point") {
                coordRow(label: "Latitude", value: $landingLat, color: .green)
                coordRow(label: "Longitude", value: $landingLon, color: .green)
            }

            // Flight parameters
            sectionCard(title: "Flight Parameters") {
                paramRow(label: "Apogee", value: $apogeeFt, unit: "ft AGL")
                paramRow(label: "Drogue rate", value: $drogueRate, unit: "fps")
                paramRow(label: "Main rate", value: $mainRate, unit: "fps")
                paramRow(label: "Main deploy", value: $mainDeploy, unit: "ft")
                paramRow(label: "Max steering", value: $maxSteering, unit: "°")

                DatePicker("Launch time",
                           selection: $launchTime,
                           displayedComponents: [.date, .hourAndMinute])
                    .datePickerStyle(.compact)
            }

            // Compute button
            Button(action: compute) {
                HStack {
                    if isComputing {
                        ProgressView()
                            .tint(.white)
                    }
                    Text(isComputing ? "COMPUTING..." : "COMPUTE")
                        .fontWeight(.bold)
                }
                .frame(maxWidth: .infinity)
                .padding()
                .background(inputsValid && !isComputing ? Color.red : Color.gray)
                .foregroundColor(.white)
                .cornerRadius(10)
            }
            .disabled(!inputsValid || isComputing)
            .padding(.horizontal)

            // Error
            if let error = errorMessage {
                Text(error)
                    .font(.caption)
                    .foregroundColor(.red)
                    .padding(.horizontal)
            }

            // Results
            if let r = result {
                resultsSection(r)
                windProfileSection(r.windProfile)

                // Send to unit
                Button(action: { showSendConfirm = true }) {
                    Label("Send to Unit", systemImage: "antenna.radiowaves.left.and.right")
                        .fontWeight(.bold)
                        .frame(maxWidth: .infinity)
                        .padding()
                        .background(canSend ? Color.blue : Color.gray)
                        .foregroundColor(.white)
                        .cornerRadius(10)
                }
                .disabled(!canSend)
                .padding(.horizontal)
                .padding(.bottom, 20)
            }
        }
        .padding(.top, 12)
    }

    // MARK: - Results Section

    private func resultsSection(_ r: GuidanceResult) -> some View {
        sectionCard(title: "Results") {
            resultRow("Guidance point",
                      String(format: "%.6f, %.6f", r.guidanceLat, r.guidanceLon))
            resultRow("Steering angle",
                      String(format: "%.1f° off vertical", r.steeringAngleDeg))
            resultRow("Steering bearing",
                      String(format: "%.0f°", r.steeringBearingDeg))

            HStack {
                Text("Status")
                    .foregroundColor(.secondary)
                Spacer()
                Text(r.feasible ? "FEASIBLE" : "INFEASIBLE")
                    .fontWeight(.bold)
                    .foregroundColor(r.feasible ? .green : .red)
            }

            if let reason = r.infeasibleReason {
                Text(reason)
                    .font(.caption)
                    .foregroundColor(.red)
            }

            resultRow("Descent time",
                      String(format: "%.0f s (%.1f min)", r.totalDescentTimeS, r.totalDescentTimeS / 60))
            resultRow("Total drift",
                      String(format: "%.0f m", r.totalDriftM))
            resultRow("Verification error",
                      String(format: "%.1f m", r.landingErrorM))
            resultRow("Ground elev",
                      String(format: "%.0f ft ASL", r.windProfile.groundElevFt))
        }
    }

    // MARK: - Wind Profile Section

    private func windProfileSection(_ wp: WindProfile) -> some View {
        sectionCard(title: "Wind Profile") {
            // Header
            HStack {
                Text("Alt (ft)")
                    .frame(width: 70, alignment: .trailing)
                Text("Speed (kts)")
                    .frame(width: 80, alignment: .trailing)
                Text("From (°)")
                    .frame(width: 70, alignment: .trailing)
            }
            .font(.caption.bold())
            .foregroundColor(.secondary)

            Divider()

            ForEach(wp.layers.indices, id: \.self) { i in
                let layer = wp.layers[i]
                HStack {
                    Text(String(format: "%.0f", layer.altFt))
                        .frame(width: 70, alignment: .trailing)
                    Text(String(format: "%.1f", layer.speedKts))
                        .frame(width: 80, alignment: .trailing)
                    Text(String(format: "%.0f", layer.directionDeg))
                        .frame(width: 70, alignment: .trailing)
                }
                .font(.system(.caption, design: .monospaced))
            }
        }
    }

    // MARK: - UI Helpers

    private func sectionCard<Content: View>(title: String, @ViewBuilder content: () -> Content) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            Text(title.uppercased())
                .font(.caption.bold())
                .foregroundColor(.red)
                .tracking(1)

            VStack(spacing: 6) {
                content()
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.secondarySystemGroupedBackground))
        .cornerRadius(12)
        .padding(.horizontal)
    }

    private func coordRow(label: String, value: Binding<String>, color: Color) -> some View {
        HStack {
            Text(label)
                .foregroundColor(color)
                .frame(width: 80, alignment: .trailing)
            TextField("0.0", text: value)
                .keyboardType(.numbersAndPunctuation)
                .multilineTextAlignment(.trailing)
                .font(.system(.body, design: .monospaced))
        }
    }

    private func paramRow(label: String, value: Binding<String>, unit: String) -> some View {
        HStack {
            Text(label)
                .foregroundColor(.secondary)
                .frame(width: 100, alignment: .trailing)
            TextField("0", text: value)
                .keyboardType(.decimalPad)
                .multilineTextAlignment(.trailing)
                .frame(width: 80)
                .font(.system(.body, design: .monospaced))
            Text(unit)
                .foregroundColor(.secondary)
                .frame(width: 50, alignment: .leading)
        }
    }

    private func resultRow(_ label: String, _ value: String) -> some View {
        HStack {
            Text(label)
                .foregroundColor(.secondary)
            Spacer()
            Text(value)
                .font(.system(.body, design: .monospaced))
        }
    }

    // MARK: - Actions

    private func handleMapTap(_ coord: CLLocationCoordinate2D) {
        if tapMode == .launch {
            launchLat = String(format: "%.6f", coord.latitude)
            launchLon = String(format: "%.6f", coord.longitude)
            // Auto-switch to landing mode
            tapMode = .landing
        } else {
            landingLat = String(format: "%.6f", coord.latitude)
            landingLon = String(format: "%.6f", coord.longitude)
        }
    }

    private func useGPS() {
        locationManager.requestWhenInUseAuthorization()
        locationManager.desiredAccuracy = kCLLocationAccuracyBest

        // Use the cached location directly instead of requestLocation(),
        // which requires a CLLocationManagerDelegate that isn't set up here.
        if let loc = locationManager.location {
            launchLat = String(format: "%.6f", loc.coordinate.latitude)
            launchLon = String(format: "%.6f", loc.coordinate.longitude)
            mapRegion = MKCoordinateRegion(
                center: loc.coordinate,
                span: MKCoordinateSpan(latitudeDelta: 0.02, longitudeDelta: 0.02)
            )
        }
    }

    private func compute() {
        guard let laLat = Double(launchLat), let laLon = Double(launchLon),
              let ldLat = Double(landingLat), let ldLon = Double(landingLon),
              let apo = Double(apogeeFt),
              let dRate = Double(drogueRate),
              let mRate = Double(mainRate),
              let mDeploy = Double(mainDeploy),
              let maxSteer = Double(maxSteering) else { return }

        isComputing = true
        errorMessage = nil
        result = nil

        Task {
            do {
                let r = try await computeGuidancePoint(
                    launchLat: laLat,
                    launchLon: laLon,
                    landingLat: ldLat,
                    landingLon: ldLon,
                    apogeeAglFt: apo,
                    drogueRateFps: dRate,
                    mainRateFps: mRate,
                    mainDeployAglFt: mDeploy,
                    launchTimeUTC: launchTime,
                    maxSteeringDeg: maxSteer
                )

                await MainActor.run {
                    result = r
                    isComputing = false
                    fitMapToResults(r)
                }
            } catch {
                await MainActor.run {
                    errorMessage = error.localizedDescription
                    isComputing = false
                }
            }
        }
    }

    private func fitMapToResults(_ r: GuidanceResult) {
        var coords: [CLLocationCoordinate2D] = []
        if let c = launchCoord { coords.append(c) }
        if let c = landingCoord { coords.append(c) }
        coords.append(CLLocationCoordinate2D(latitude: r.guidanceLat, longitude: r.guidanceLon))
        for tp in r.descentTrack {
            coords.append(CLLocationCoordinate2D(latitude: tp.lat, longitude: tp.lon))
        }

        guard !coords.isEmpty else { return }

        var minLat = coords.map(\.latitude).min()!
        var maxLat = coords.map(\.latitude).max()!
        var minLon = coords.map(\.longitude).min()!
        var maxLon = coords.map(\.longitude).max()!

        // Add padding
        let latPad = max((maxLat - minLat) * 0.2, 0.002)
        let lonPad = max((maxLon - minLon) * 0.2, 0.002)
        minLat -= latPad
        maxLat += latPad
        minLon -= lonPad
        maxLon += lonPad

        mapRegion = MKCoordinateRegion(
            center: CLLocationCoordinate2D(
                latitude: (minLat + maxLat) / 2,
                longitude: (minLon + maxLon) / 2
            ),
            span: MKCoordinateSpan(
                latitudeDelta: maxLat - minLat,
                longitudeDelta: maxLon - minLon
            )
        )
    }

    private func sendToUnit() {
        guard let r = result else { return }
        let altM = Float(ftToM(r.apogeeFt))
        bleManager.sendGuidancePoint(lat: r.guidanceLat, lon: r.guidanceLon, altitudeM: altM)
        showSentAlert = true
    }
}

// MARK: - Preview

struct DriftCastView_Previews: PreviewProvider {
    static var previews: some View {
        DriftCastView(bleManager: BLEManager())
    }
}
