//
//  MagCalSphereView.swift
//  TinkerRocketApp
//
//  Issue #148.  SwiftUI view that renders the MagCal sampling state as a
//  3D translucent sphere with 32 Voronoi cells (truncated-icosahedron
//  tessellation, see MagCalCells.swift) tiling the sphere.  A small
//  rocket model sits at the centre, oriented from the live accelerometer
//  vector so the user sees the rocket on screen mirror the physical
//  rocket's pose.  Captured cells fill semi-transparent green.
//
//  Replaces the old per-axis tap-grid + accel bars sampling UI — capture
//  is fully automatic; the user just rotates the rocket and watches the
//  sphere fill in.
//

import SwiftUI
import SceneKit
import simd

struct MagCalSphereView: UIViewRepresentable {
    /// Bit i set => cell i in MagCalCells.cellPolygons is captured.
    let coverageMask: UInt32

    /// Live accelerometer reading from the rocket, in m/s² body-frame.
    /// Used purely for visualization: nose direction = -normalize(accel)
    /// (the rocket points opposite to gravity when upright on the pad).
    /// Pass `nil` (or zero vector) for the pre-launch / no-data case.
    let liveAccel: SIMD3<Float>?

    // MARK: - Tunables

    /// Camera distance — pulled back enough that the whole sphere fits
    /// comfortably with margins.
    private static let cameraZ: Float = 3.5
    /// Sphere geometry radius (cells live on this surface).
    private static let sphereR: Float = 1.0
    /// Rocket body half-length.  Picked so the rocket clearly sits inside
    /// the sphere with a visible nose-tip pointing through.
    private static let rocketBodyHalf: Float = 0.5
    private static let rocketRadius: Float = 0.07

    // MARK: - UIViewRepresentable

    func makeUIView(context: Context) -> SCNView {
        let view = SCNView()
        view.backgroundColor = .systemBackground
        view.allowsCameraControl = false
        view.antialiasingMode = .multisampling4X

        let scene = SCNScene()
        view.scene = scene

        // Camera
        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        cameraNode.position = SCNVector3(0, 0, MagCalSphereView.cameraZ)
        scene.rootNode.addChildNode(cameraNode)

        // Ambient + directional light so the sphere + rocket read as 3D.
        let ambient = SCNNode()
        ambient.light = SCNLight()
        ambient.light?.type = .ambient
        ambient.light?.intensity = 400
        scene.rootNode.addChildNode(ambient)

        let key = SCNNode()
        key.light = SCNLight()
        key.light?.type = .directional
        key.light?.intensity = 700
        key.eulerAngles = SCNVector3(-0.6, 0.8, 0)
        scene.rootNode.addChildNode(key)

        // Body-fixed assembly: the sphere, the cells, and the rocket all
        // live under one parent that rotates with the live accel.
        let assembly = SCNNode()
        assembly.name = "assembly"
        scene.rootNode.addChildNode(assembly)

        // Translucent sphere shell so cells in the back are partially visible
        let shell = SCNSphere(radius: CGFloat(MagCalSphereView.sphereR * 0.995))
        shell.segmentCount = 48
        shell.firstMaterial?.diffuse.contents = UIColor(white: 0.0, alpha: 0.02)
        shell.firstMaterial?.transparency = 0.10
        shell.firstMaterial?.isDoubleSided = true
        shell.firstMaterial?.cullMode = .back
        let shellNode = SCNNode(geometry: shell)
        shellNode.name = "sphereShell"
        assembly.addChildNode(shellNode)

        // 32 cells
        for cellIdx in 0..<MagCalCells.cellPolygons.count {
            let cellNode = SCNNode(geometry: Self.makeCellGeometry(cellIdx: cellIdx,
                                                                   captured: false))
            cellNode.name = "cell-\(cellIdx)"
            assembly.addChildNode(cellNode)
        }

        // Rocket — simple cylinder body + cone nose along its local +Y
        // (SCNCylinder grows along Y by default).  Tail end at -Y, nose
        // at +Y.  When body-frame gravity points "down" the rocket's
        // local +Y will align with world's "up" so the rocket appears
        // nose-up on screen at rest.
        let bodyHalf = MagCalSphereView.rocketBodyHalf
        let bodyR = MagCalSphereView.rocketRadius
        let body = SCNCylinder(radius: CGFloat(bodyR), height: CGFloat(bodyHalf * 2 * 0.75))
        body.firstMaterial?.diffuse.contents = UIColor.systemGray3
        let bodyNode = SCNNode(geometry: body)
        bodyNode.position = SCNVector3(0, -bodyHalf * 0.25, 0)

        let nose = SCNCone(topRadius: 0, bottomRadius: CGFloat(bodyR), height: CGFloat(bodyHalf * 0.5))
        nose.firstMaterial?.diffuse.contents = UIColor.systemGray2
        let noseNode = SCNNode(geometry: nose)
        noseNode.position = SCNVector3(0, bodyHalf * 0.75, 0)

        let fin = SCNBox(width: CGFloat(bodyR * 1.4), height: CGFloat(bodyR * 1.4), length: 0.01, chamferRadius: 0)
        fin.firstMaterial?.diffuse.contents = UIColor.systemGray4

        let rocket = SCNNode()
        rocket.name = "rocket"
        rocket.addChildNode(bodyNode)
        rocket.addChildNode(noseNode)
        // 3 fins around the tail at 120° intervals
        for k in 0..<3 {
            let f = SCNNode(geometry: fin)
            let a = Float(k) * (2 * .pi / 3)
            f.position = SCNVector3(cos(a) * bodyR * 1.0, -bodyHalf * 0.8, sin(a) * bodyR * 1.0)
            f.eulerAngles = SCNVector3(0, a, 0)
            rocket.addChildNode(f)
        }
        assembly.addChildNode(rocket)

        // Initial state
        applyState(view: view, coverageMask: coverageMask, liveAccel: liveAccel)
        return view
    }

    func updateUIView(_ view: SCNView, context: Context) {
        applyState(view: view, coverageMask: coverageMask, liveAccel: liveAccel)
    }

    // MARK: - State application

    private func applyState(view: SCNView, coverageMask: UInt32, liveAccel: SIMD3<Float>?) {
        guard let scene = view.scene else { return }
        // Cell colors — captured = translucent green, uncaptured = pale grey.
        for cellIdx in 0..<MagCalCells.cellPolygons.count {
            guard let node = scene.rootNode.childNode(withName: "cell-\(cellIdx)", recursively: true),
                  let mat = node.geometry?.firstMaterial else { continue }
            let bit = (coverageMask >> UInt32(cellIdx)) & 1
            mat.diffuse.contents = bit == 1
                ? UIColor(red: 0.20, green: 0.78, blue: 0.35, alpha: 0.45)
                : UIColor(white: 0.78, alpha: 0.20)
            mat.transparency = bit == 1 ? 0.55 : 0.30
        }
        // Assembly orientation — rotate so the rocket nose mirrors the
        // physical pose.  Body-frame gravity = liveAccel; the rocket's
        // local "up" axis (+Y) should align with world's "up" when
        // gravity is along body's -Y.  i.e. we want a rotation that
        // maps -normalize(accel) → SCN world's +Y.
        if let g = liveAccel, simd_length(g) > 0.1 {
            let up: SIMD3<Float> = SIMD3<Float>(0, 1, 0)
            let bodyUp = -simd_normalize(g)
            // Rotation that takes bodyUp → up (in scene/world frame).
            let q = quaternionFromTo(bodyUp, up)
            scene.rootNode.childNode(withName: "assembly", recursively: false)?
                .simdOrientation = q
        }
    }

    /// Quaternion that rotates vector `a` to vector `b` (both unit-length).
    /// Handles the antipodal singularity by picking an arbitrary axis
    /// perpendicular to `a`.
    private func quaternionFromTo(_ a: SIMD3<Float>, _ b: SIMD3<Float>) -> simd_quatf {
        let d = simd_dot(a, b)
        if d > 0.99999 { return simd_quatf(real: 1, imag: SIMD3<Float>(0, 0, 0)) }
        if d < -0.99999 {
            // Antipodal — rotate 180° around any axis perpendicular to a.
            let ortho = abs(a.x) > 0.9
                ? SIMD3<Float>(0, 1, 0)
                : SIMD3<Float>(1, 0, 0)
            let axis = simd_normalize(simd_cross(a, ortho))
            return simd_quatf(angle: .pi, axis: axis)
        }
        let axis = simd_normalize(simd_cross(a, b))
        let angle = acos(d)
        return simd_quatf(angle: angle, axis: axis)
    }

    // MARK: - Cell geometry construction

    /// Build a fan-triangulated SCNGeometry for cell `cellIdx`.  Centre
    /// vertex sits at the cell's centre on the sphere; edge vertices
    /// are the cell's Voronoi boundary vertices.  Slightly lifted above
    /// the sphere shell (radius 1.002) to avoid z-fighting.
    private static func makeCellGeometry(cellIdx: Int, captured: Bool) -> SCNGeometry {
        let lift: Float = 1.002
        let center = MagCalCells.cellCenters[cellIdx] * lift
        let edgeIdx = MagCalCells.cellPolygons[cellIdx]
        var positions: [SIMD3<Float>] = [center]
        for vi in edgeIdx {
            positions.append(MagCalCells.voronoiVertices[vi] * lift)
        }
        // Fan triangulation: (center, v[i], v[i+1])  i = 0..N-1, wrap
        var indices: [UInt16] = []
        let n = edgeIdx.count
        for i in 0..<n {
            indices.append(0)
            indices.append(UInt16(1 + i))
            indices.append(UInt16(1 + ((i + 1) % n)))
        }
        // Normals = position (we're on a sphere)
        let normals = positions.map { simd_normalize($0) }

        let vertexData = Data(bytes: positions, count: positions.count * MemoryLayout<SIMD3<Float>>.size)
        let normalData = Data(bytes: normals,   count: normals.count   * MemoryLayout<SIMD3<Float>>.size)
        let vertexSource = SCNGeometrySource(
            data: vertexData,
            semantic: .vertex,
            vectorCount: positions.count,
            usesFloatComponents: true,
            componentsPerVector: 3,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: 0,
            dataStride: MemoryLayout<SIMD3<Float>>.size
        )
        let normalSource = SCNGeometrySource(
            data: normalData,
            semantic: .normal,
            vectorCount: normals.count,
            usesFloatComponents: true,
            componentsPerVector: 3,
            bytesPerComponent: MemoryLayout<Float>.size,
            dataOffset: 0,
            dataStride: MemoryLayout<SIMD3<Float>>.size
        )
        let elementData = Data(bytes: indices, count: indices.count * MemoryLayout<UInt16>.size)
        let element = SCNGeometryElement(
            data: elementData,
            primitiveType: .triangles,
            primitiveCount: indices.count / 3,
            bytesPerIndex: MemoryLayout<UInt16>.size
        )
        let geometry = SCNGeometry(sources: [vertexSource, normalSource], elements: [element])
        let mat = SCNMaterial()
        mat.diffuse.contents = captured
            ? UIColor(red: 0.20, green: 0.78, blue: 0.35, alpha: 0.45)
            : UIColor(white: 0.78, alpha: 0.20)
        mat.transparency = captured ? 0.55 : 0.30
        mat.isDoubleSided = true
        mat.lightingModel = .lambert
        geometry.firstMaterial = mat
        return geometry
    }
}
