//
//  FlightTrajectoryView.swift
//  TinkerRocketApp
//
//  2D map and 3D SceneKit visualization of a saved flight's GPS trajectory.
//

import SwiftUI
import MapKit
import CoreLocation
import SceneKit

// MARK: - Data Extraction

/// Extract GPS track points from columnar CSV data.
/// Handles both rocket flight and LoRa base station column naming conventions.
private func extractTrackPoints(from csv: FlightCSVData, type: CachedFlightType) -> [TrackPoint] {
    let latKey: String
    let lonKey: String
    let altKey: String
    let timeKey: String

    switch type {
    case .rocket:
        latKey = "Latitude (deg)"
        lonKey = "Longitude (deg)"
        // Prefer pressure altitude (AGL) — GNSS altitude is MSL and causes underground artifacts
        altKey = csv.columns["Pressure Altitude (m)"] != nil ? "Pressure Altitude (m)" : "GNSS Altitude (m)"
        timeKey = "Time (ms)"
    case .loraLog:
        latKey = "lat"
        lonKey = "lon"
        // Prefer pressure_alt (AGL) for consistency with rocket flights
        altKey = csv.columns["pressure_alt"] != nil ? "pressure_alt" : "alt_m"
        timeKey = "time_ms"
    }

    guard let lats = csv.columns[latKey],
          let lons = csv.columns[lonKey],
          let alts = csv.columns[altKey],
          let times = csv.columns[timeKey] else { return [] }

    let count = min(lats.count, lons.count, alts.count, times.count)
    var points: [TrackPoint] = []
    points.reserveCapacity(count)

    for i in 0..<count {
        let lat = lats[i], lon = lons[i], alt = alts[i], time = times[i]
        // Skip rows with no GPS fix
        if lat.isNaN || lon.isNaN || lat == 0 || lon == 0 { continue }
        if alt.isNaN { continue }
        // Clamp negative altitudes to 0 to prevent underground artifacts from sensor noise
        let altClamped = max(alt, 0)
        points.append(TrackPoint(
            lat: lat, lon: lon,
            altAglFt: mToFt(altClamped),
            timeS: time / 1000.0
        ))
    }

    return points
}

/// Identify key flight landmarks from the track.
private func findLandmarks(in points: [TrackPoint]) -> (launch: TrackPoint, apogee: TrackPoint, landing: TrackPoint)? {
    guard let first = points.first, let last = points.last else { return nil }
    let apogee = points.max(by: { $0.altAglFt < $1.altAglFt }) ?? first
    return (launch: first, apogee: apogee, landing: last)
}

/// Downsample track points by stride for 3D scene performance.
private func downsample(_ points: [TrackPoint], maxCount: Int) -> [TrackPoint] {
    guard points.count > maxCount else { return points }
    let stride = max(1, points.count / maxCount)
    var result: [TrackPoint] = []
    result.reserveCapacity(maxCount + 2)
    for i in Swift.stride(from: 0, to: points.count, by: stride) {
        result.append(points[i])
    }
    // Always include the last point
    if let last = points.last, result.last?.timeS != last.timeS {
        result.append(last)
    }
    return result
}

/// Altitude-based color: blue (low) to red (high) via HSB hue ramp.
private func altitudeColor(_ alt: Double, minAlt: Double, range: Double) -> UIColor {
    let t = range > 0 ? CGFloat((alt - minAlt) / range) : 0.5
    return UIColor(hue: (1.0 - t) * 0.65, saturation: 0.9, brightness: 0.95, alpha: 1.0)
}

// MARK: - 2D Map View

struct FlightMapView: UIViewRepresentable {
    let trackPoints: [TrackPoint]
    let launchPoint: TrackPoint?
    let apogeePoint: TrackPoint?
    let landingPoint: TrackPoint?

    func makeUIView(context: Context) -> MKMapView {
        let mapView = MKMapView()
        mapView.delegate = context.coordinator
        mapView.mapType = .hybrid
        return mapView
    }

    func updateUIView(_ mapView: MKMapView, context: Context) {
        let hash = "\(trackPoints.count),\(trackPoints.first?.lat ?? 0),\(trackPoints.last?.lat ?? 0)"
        guard hash != context.coordinator.lastHash else { return }
        context.coordinator.lastHash = hash
        context.coordinator.trackPoints = trackPoints

        // Annotations
        mapView.removeAnnotations(mapView.annotations)

        if let pt = launchPoint {
            let ann = MKPointAnnotation()
            ann.coordinate = CLLocationCoordinate2D(latitude: pt.lat, longitude: pt.lon)
            ann.title = "Launch"
            ann.subtitle = "launch"
            mapView.addAnnotation(ann)
        }
        if let pt = apogeePoint {
            let ann = MKPointAnnotation()
            ann.coordinate = CLLocationCoordinate2D(latitude: pt.lat, longitude: pt.lon)
            ann.title = String(format: "Apogee · %.0f ft", pt.altAglFt)
            ann.subtitle = "apogee"
            mapView.addAnnotation(ann)
        }
        if let pt = landingPoint {
            let ann = MKPointAnnotation()
            ann.coordinate = CLLocationCoordinate2D(latitude: pt.lat, longitude: pt.lon)
            ann.title = "Landing"
            ann.subtitle = "landing"
            mapView.addAnnotation(ann)
        }

        // Polyline overlay
        mapView.removeOverlays(mapView.overlays)

        if trackPoints.count > 1 {
            var coords = trackPoints.map {
                CLLocationCoordinate2D(latitude: $0.lat, longitude: $0.lon)
            }
            let polyline = MKPolyline(coordinates: &coords, count: coords.count)
            polyline.title = "flight"
            mapView.addOverlay(polyline)
        }

        // Fit region
        if !trackPoints.isEmpty {
            let lats = trackPoints.map(\.lat)
            let lons = trackPoints.map(\.lon)
            let minLat = lats.min()!, maxLat = lats.max()!
            let minLon = lons.min()!, maxLon = lons.max()!
            let latPad = max((maxLat - minLat) * 0.2, 0.002)
            let lonPad = max((maxLon - minLon) * 0.2, 0.002)
            let region = MKCoordinateRegion(
                center: CLLocationCoordinate2D(latitude: (minLat + maxLat) / 2,
                                               longitude: (minLon + maxLon) / 2),
                span: MKCoordinateSpan(latitudeDelta: maxLat - minLat + 2 * latPad,
                                        longitudeDelta: maxLon - minLon + 2 * lonPad)
            )
            mapView.setRegion(region, animated: false)
        }
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    class Coordinator: NSObject, MKMapViewDelegate {
        var lastHash = ""
        var trackPoints: [TrackPoint] = []

        func mapView(_ mapView: MKMapView, viewFor annotation: MKAnnotation) -> MKAnnotationView? {
            guard let pointAnn = annotation as? MKPointAnnotation else { return nil }
            let id = "FlightPin"
            var view = mapView.dequeueReusableAnnotationView(withIdentifier: id) as? MKMarkerAnnotationView
            if view == nil {
                view = MKMarkerAnnotationView(annotation: annotation, reuseIdentifier: id)
                view?.canShowCallout = true
            } else {
                view?.annotation = annotation
            }

            switch pointAnn.subtitle {
            case "launch":
                view?.markerTintColor = .systemRed
                view?.glyphImage = UIImage(systemName: "location.fill")
            case "apogee":
                view?.markerTintColor = .systemBlue
                view?.glyphImage = UIImage(systemName: "arrow.up.to.line")
            case "landing":
                view?.markerTintColor = .systemGreen
                view?.glyphImage = UIImage(systemName: "flag.fill")
            default:
                view?.markerTintColor = .systemGray
            }
            return view
        }

        func mapView(_ mapView: MKMapView, rendererFor overlay: MKOverlay) -> MKOverlayRenderer {
            if let polyline = overlay as? MKPolyline {
                let renderer = MKPolylineRenderer(polyline: polyline)
                // Altitude-gradient coloring: use mid-track color for the whole line
                let alts = trackPoints.map(\.altAglFt)
                let minAlt = alts.min() ?? 0
                let maxAlt = alts.max() ?? 1
                let range = max(maxAlt - minAlt, 1)
                let midAlt = (minAlt + maxAlt) / 2
                renderer.strokeColor = altitudeColor(midAlt, minAlt: minAlt, range: range)
                renderer.lineWidth = 3
                return renderer
            }
            return MKOverlayRenderer(overlay: overlay)
        }
    }
}

// MARK: - 3D Scene View

struct FlightScene3DView: UIViewRepresentable {
    let trackPoints: [TrackPoint]
    let launchPoint: TrackPoint?
    let apogeePoint: TrackPoint?
    let landingPoint: TrackPoint?

    func makeUIView(context: Context) -> SCNView {
        let scnView = SCNView()
        scnView.backgroundColor = UIColor(red: 0.04, green: 0.04, blue: 0.10, alpha: 1.0)
        scnView.allowsCameraControl = true
        scnView.autoenablesDefaultLighting = true
        scnView.antialiasingMode = .multisampling4X
        return scnView
    }

    func updateUIView(_ scnView: SCNView, context: Context) {
        guard !trackPoints.isEmpty else {
            scnView.scene = nil
            return
        }
        let hash = "\(trackPoints.count),\(trackPoints.first?.lat ?? 0),\(trackPoints.last?.lat ?? 0)"
        if context.coordinator.lastHash != hash {
            context.coordinator.lastHash = hash
            let scenePts = downsample(trackPoints, maxCount: 500)
            let (scene, groundNode, extent) = Self.buildScene(
                trackPoints: scenePts,
                launchPoint: launchPoint, apogeePoint: apogeePoint, landingPoint: landingPoint
            )
            scnView.scene = scene
            scnView.pointOfView = scene.rootNode.childNode(
                withName: "mainCamera", recursively: false)

            if let ref = trackPoints.first {
                Self.fetchArcGISImagery(
                    refLat: ref.lat, refLon: ref.lon,
                    extent: extent, groundNode: groundNode,
                    sceneRoot: scene.rootNode
                )
            }
        }
    }

    func makeCoordinator() -> Coordinator { Coordinator() }
    class Coordinator { var lastHash = "" }

    // MARK: - Geo Conversion

    private static func localPos(lat: Double, lon: Double, altM: Double,
                                  refLat: Double, refLon: Double) -> SCNVector3 {
        let x = Float((lon - refLon) * 111_320 * cos(refLat * .pi / 180))
        let z = Float(-(lat - refLat) * 110_540)
        return SCNVector3(x, Float(altM), z)
    }

    // MARK: - ArcGIS Satellite Imagery

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
                sceneRoot.childNode(withName: "groundGrid", recursively: false)?
                    .removeFromParentNode()
            }
        }.resume()
    }

    // MARK: - Scene Builder

    static func buildScene(
        trackPoints: [TrackPoint],
        launchPoint: TrackPoint?, apogeePoint: TrackPoint?, landingPoint: TrackPoint?
    ) -> (SCNScene, SCNNode, Float) {
        let scene = SCNScene()
        let root = scene.rootNode

        guard let launch = launchPoint else { return (scene, SCNNode(), 100) }
        let refLat = launch.lat
        let refLon = launch.lon
        let groundAltM = ftToM(launch.altAglFt)

        // Convert all track points to scene coordinates (altitude relative to launch)
        var trackPositions: [SCNVector3] = []
        for pt in trackPoints {
            trackPositions.append(localPos(lat: pt.lat, lon: pt.lon,
                                            altM: ftToM(pt.altAglFt) - groundAltM,
                                            refLat: refLat, refLon: refLon))
        }

        let launchPos = SCNVector3(0, 0, 0)
        let apogeePos: SCNVector3
        if let ap = apogeePoint {
            apogeePos = localPos(lat: ap.lat, lon: ap.lon,
                                  altM: ftToM(ap.altAglFt) - groundAltM,
                                  refLat: refLat, refLon: refLon)
        } else {
            apogeePos = launchPos
        }
        let landingPos: SCNVector3
        if let lp = landingPoint {
            landingPos = localPos(lat: lp.lat, lon: lp.lon,
                                   altM: ftToM(lp.altAglFt) - groundAltM,
                                   refLat: refLat, refLon: refLon)
        } else {
            landingPos = launchPos
        }

        // Compute scene extent
        let allPos = [launchPos, apogeePos, landingPos] + trackPositions
        let xs = allPos.map(\.x), ys = allPos.map(\.y), zs = allPos.map(\.z)
        let extent = max(
            (xs.max()! - xs.min()!),
            (ys.max()! - ys.min()!),
            (zs.max()! - zs.min()!),
            100
        )
        let markerSize = CGFloat(extent * 0.016)

        // Ground plane
        let ground = SCNPlane(width: CGFloat(extent * 3), height: CGFloat(extent * 3))
        let groundMat = SCNMaterial()
        groundMat.diffuse.contents = UIColor(white: 0.10, alpha: 0.85)
        groundMat.isDoubleSided = true
        ground.materials = [groundMat]
        let groundNode = SCNNode(geometry: ground)
        groundNode.eulerAngles.x = -.pi / 2
        groundNode.name = "ground"
        root.addChildNode(groundNode)

        // Grid lines
        let gridSpacing = niceGridSpacing(extent)
        addGroundGrid(to: root, extent: extent, spacing: gridSpacing)

        // Markers
        addMarker(to: root, position: launchPos,
                  color: UIColor(red: 1.0, green: 0.42, blue: 0.42, alpha: 1),
                  radius: markerSize)
        addMarker(to: root, position: landingPos,
                  color: UIColor(red: 0.32, green: 0.81, blue: 0.40, alpha: 1),
                  radius: markerSize)
        addMarker(to: root, position: apogeePos,
                  color: UIColor(red: 0.30, green: 0.67, blue: 0.97, alpha: 1),
                  radius: markerSize * 1.3)

        // Flight path — altitude-colored tube segments
        if trackPositions.count >= 2 {
            let alts = trackPoints.map(\.altAglFt)
            let minAlt = alts.min() ?? 0
            let maxAlt = alts.max() ?? 1
            let range = max(maxAlt - minAlt, 1)
            let tubeRadius = CGFloat(extent * 0.004)

            for i in 0..<(trackPositions.count - 1) {
                let a = trackPositions[i], b = trackPositions[i + 1]
                let dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z
                let len = sqrt(dx * dx + dy * dy + dz * dz)
                guard len > 0.01 else { continue }

                let midAlt = (trackPoints[i].altAglFt + trackPoints[i + 1].altAglFt) / 2
                let color = altitudeColor(midAlt, minAlt: minAlt, range: range)

                let cyl = SCNCylinder(radius: tubeRadius, height: CGFloat(len))
                let mat = SCNMaterial()
                mat.diffuse.contents = color
                mat.emission.contents = color.withAlphaComponent(0.4)
                cyl.materials = [mat]

                let node = SCNNode(geometry: cyl)
                node.position = SCNVector3(
                    (a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2
                )
                node.look(at: SCNVector3(b.x, b.y, b.z),
                          up: SCNVector3(0, 1, 0),
                          localFront: SCNVector3(0, 1, 0))
                root.addChildNode(node)
            }
        }

        // Vertical drop line from apogee to ground
        let apogeeGround = SCNVector3(apogeePos.x, 0, apogeePos.z)
        addTubeLine(to: root, points: [apogeePos, apogeeGround],
                    color: UIColor(red: 0.30, green: 0.67, blue: 0.97, alpha: 0.25),
                    radius: CGFloat(extent * 0.001))

        // Labels
        let labelScale = Float(extent * 0.0006)
        addLabel(to: root, text: "Launch",
                 position: SCNVector3(launchPos.x, Float(markerSize) * 2.5, launchPos.z),
                 color: .white, scale: labelScale)
        addLabel(to: root, text: "Landing",
                 position: SCNVector3(landingPos.x, Float(markerSize) * 2.5, landingPos.z),
                 color: .white, scale: labelScale)
        addLabel(to: root, text: "Apogee",
                 position: SCNVector3(apogeePos.x, apogeePos.y + Float(markerSize) * 2.5, apogeePos.z),
                 color: UIColor(red: 0.30, green: 0.67, blue: 0.97, alpha: 1), scale: labelScale)
        if let ap = apogeePoint {
            let altLabel = String(format: "%.0f ft AGL", ap.altAglFt - (launchPoint?.altAglFt ?? 0))
            addLabel(to: root, text: altLabel,
                     position: SCNVector3(apogeePos.x, apogeePos.y + Float(markerSize) * 1.0, apogeePos.z),
                     color: UIColor(white: 0.7, alpha: 1), scale: labelScale * 0.7)
        }

        // Camera — side view perpendicular to launch-to-apogee bearing
        let camNode = SCNNode()
        camNode.name = "mainCamera"
        camNode.camera = SCNCamera()
        camNode.camera?.zFar = Double(extent * 20)
        camNode.camera?.zNear = Double(max(extent * 0.001, 0.1))
        camNode.camera?.fieldOfView = 50

        let cx = (xs.min()! + xs.max()!) / 2
        let cy = (ys.min()! + ys.max()!) / 2
        let cz = (zs.min()! + zs.max()!) / 2

        let trackAngle = atan2(apogeePos.x, -apogeePos.z)
        let viewAngle = trackAngle + .pi / 2
        let dist = extent * 1.8
        camNode.position = SCNVector3(
            cx + dist * sin(viewAngle),
            cy + dist * 0.4,
            cz - dist * cos(viewAngle)
        )
        camNode.look(at: SCNVector3(cx, cy, cz))
        root.addChildNode(camNode)

        // Lighting
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
                (a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2
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

// MARK: - Container View

struct FlightTrajectoryView: View {
    let flight: CachedFlight

    @State private var trackPoints: [TrackPoint] = []
    @State private var launchPoint: TrackPoint?
    @State private var apogeePoint: TrackPoint?
    @State private var landingPoint: TrackPoint?
    @State private var isLoading = true
    @State private var errorMessage: String?
    @State private var show3D = false

    var body: some View {
        VStack(spacing: 0) {
            if isLoading {
                Spacer()
                ProgressView("Loading trajectory...")
                Spacer()
            } else if let error = errorMessage {
                Spacer()
                VStack(spacing: 12) {
                    Image(systemName: "exclamationmark.triangle")
                        .font(.system(size: 48))
                        .foregroundColor(.orange)
                    Text(error)
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                        .multilineTextAlignment(.center)
                }
                .padding()
                Spacer()
            } else if trackPoints.isEmpty {
                Spacer()
                VStack(spacing: 12) {
                    Image(systemName: "location.slash")
                        .font(.system(size: 48))
                        .foregroundColor(.secondary)
                    Text("No GPS data in this flight")
                        .font(.headline)
                        .foregroundColor(.secondary)
                }
                Spacer()
            } else {
                // 2D/3D toggle
                HStack {
                    Spacer()
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

                // Map or 3D scene
                if show3D {
                    FlightScene3DView(
                        trackPoints: trackPoints,
                        launchPoint: launchPoint,
                        apogeePoint: apogeePoint,
                        landingPoint: landingPoint
                    )
                    .clipShape(RoundedRectangle(cornerRadius: 12))
                    .padding(.horizontal)
                } else {
                    FlightMapView(
                        trackPoints: trackPoints,
                        launchPoint: launchPoint,
                        apogeePoint: apogeePoint,
                        landingPoint: landingPoint
                    )
                    .cornerRadius(12)
                    .padding(.horizontal)
                }

                // Stats bar
                statsBar
                    .padding(.horizontal)
                    .padding(.vertical, 8)
            }
        }
        .navigationTitle("Trajectory")
        .navigationBarTitleDisplayMode(.inline)
        .task {
            await loadTrajectory()
        }
    }

    private var statsBar: some View {
        let groundAlt = launchPoint?.altAglFt ?? 0
        let maxAlt = (apogeePoint?.altAglFt ?? 0) - groundAlt
        let duration = (trackPoints.last?.timeS ?? 0) - (trackPoints.first?.timeS ?? 0)

        return HStack(spacing: 20) {
            VStack(spacing: 2) {
                Text(String(format: "%.0f ft", maxAlt))
                    .font(.system(.caption, design: .monospaced).bold())
                Text("Max Alt")
                    .font(.caption2)
                    .foregroundColor(.secondary)
            }
            VStack(spacing: 2) {
                Text(String(format: "%.0f s", duration))
                    .font(.system(.caption, design: .monospaced).bold())
                Text("Duration")
                    .font(.caption2)
                    .foregroundColor(.secondary)
            }
            VStack(spacing: 2) {
                Text("\(trackPoints.count)")
                    .font(.system(.caption, design: .monospaced).bold())
                Text("GPS Points")
                    .font(.caption2)
                    .foregroundColor(.secondary)
            }
        }
        .frame(maxWidth: .infinity)
        .padding(10)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }

    private func loadTrajectory() async {
        do {
            let url = flight.csvURL
            let type = flight.type
            let data = try await Task.detached(priority: .userInitiated) {
                try CSVParser.parse(url: url)
            }.value

            let points = extractTrackPoints(from: data, type: type)
            let landmarks = findLandmarks(in: points)

            await MainActor.run {
                self.trackPoints = points
                self.launchPoint = landmarks?.launch
                self.apogeePoint = landmarks?.apogee
                self.landingPoint = landmarks?.landing
                self.isLoading = false
            }
        } catch {
            await MainActor.run {
                self.errorMessage = error.localizedDescription
                self.isLoading = false
            }
        }
    }
}
