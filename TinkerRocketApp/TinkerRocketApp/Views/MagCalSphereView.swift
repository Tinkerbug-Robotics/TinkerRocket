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
    /// Bit i set => cell i in MagCalCells.cellPolygons is captured
    /// (has accumulated MAG_CAL_MIN_SAMPLES_PER_WEDGE samples).
    let coverageMask: UInt32

    /// Bit i set => cell i has accumulated 1..(threshold-1) samples —
    /// in-progress, not yet captured.  Disjoint from coverageMask.
    /// Pass 0 for older firmware that doesn't send this field; the
    /// sphere then renders 2-state (untouched / captured) only.
    let partialMask: UInt32

    /// Live accelerometer reading from the rocket, in m/s² body-frame.
    /// Used purely for visualization: nose direction = -normalize(accel)
    /// (the rocket points opposite to gravity when upright on the pad).
    /// Pass `nil` (or zero vector) for the pre-launch / no-data case.
    /// Used as a FALLBACK only — if liveAttitude (the EKF quaternion)
    /// is also provided we prefer it because it includes yaw.
    let liveAccel: SIMD3<Float>?

    /// Live attitude quaternion from the FC's EKF, scalar-first
    /// (w, x, y, z) — the same convention as TelemetryData.q0..q3.
    /// Gives full attitude including yaw around gravity, which the
    /// accel-only path can't recover.  Pass `nil` when the EKF
    /// quaternion isn't available (pre-init, fields all zero).
    let liveAttitude: simd_quatf?

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
        // Continuous render — without this SCNView only redraws on
        // animation or input, so rocket-orientation updates pushed via
        // simdOrientation in updateUIView don't actually appear on
        // screen between SwiftUI re-renders.  Cost is minimal at 60 Hz
        // for this small scene.
        view.rendersContinuously = true

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

        // Static sphere container: shell + 32 cells, anchored to world
        // axes.  Does NOT rotate with the rocket — the user sees the
        // sphere as a fixed reference so they can read coverage at a
        // glance, including back-side cells through the translucent
        // material.
        let sphereContainer = SCNNode()
        sphereContainer.name = "sphereContainer"
        scene.rootNode.addChildNode(sphereContainer)

        // Translucent sphere shell — very subtle so it just hints at
        // the surface; the cells carry the visual weight.
        let shell = SCNSphere(radius: CGFloat(MagCalSphereView.sphereR * 0.995))
        shell.segmentCount = 48
        shell.firstMaterial?.diffuse.contents = UIColor(white: 0.0, alpha: 0.015)
        shell.firstMaterial?.transparency = 0.08
        shell.firstMaterial?.isDoubleSided = true
        shell.firstMaterial?.cullMode = .back
        let shellNode = SCNNode(geometry: shell)
        shellNode.name = "sphereShell"
        sphereContainer.addChildNode(shellNode)

        // 32 cells
        for cellIdx in 0..<MagCalCells.cellPolygons.count {
            let cellNode = SCNNode(geometry: Self.makeCellGeometry(cellIdx: cellIdx,
                                                                   captured: false))
            cellNode.name = "cell-\(cellIdx)"
            sphereContainer.addChildNode(cellNode)
        }

        // Rocket — simple cylinder body + cone nose.  Firmware body-frame
        // convention (user-confirmed): +X = nose direction, +Y = out the
        // right-hand side, +Z = up (right-handed).  SCNCylinder grows
        // along its local +Y, so we build the parts in a model frame
        // where +Y is nose-forward and then wrap them in a sub-node
        // rotated -90° around Z, which maps model's +Y → outer +X.
        // After the wrap, the rocket node's local +X is the nose
        // direction, matching the firmware convention.  Rocket is a
        // sibling of sphereContainer (not a child) so its rotation
        // doesn't drag the sphere along — the user sees the rocket
        // move freely inside a stationary sphere as they physically
        // rotate the rocket on the bench.
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

        // Inner model node — nose along its local +Y as built above.
        let model = SCNNode()
        model.addChildNode(bodyNode)
        model.addChildNode(noseNode)
        // 3 fins around the tail at 120° intervals (in model-local frame).
        for k in 0..<3 {
            let f = SCNNode(geometry: fin)
            let a = Float(k) * (2 * .pi / 3)
            f.position = SCNVector3(cos(a) * bodyR * 1.0, -bodyHalf * 0.8, sin(a) * bodyR * 1.0)
            f.eulerAngles = SCNVector3(0, a, 0)
            model.addChildNode(f)
        }
        // Rotate -90° around Z so model's +Y (nose) → outer +X (nose).
        model.eulerAngles = SCNVector3(0, 0, -Float.pi / 2)

        let rocket = SCNNode()
        rocket.name = "rocket"
        rocket.addChildNode(model)
        // Sibling of sphereContainer (not a child) — see comment above.
        scene.rootNode.addChildNode(rocket)

        // Initial state
        applyState(view: view, coverageMask: coverageMask,
                   partialMask: partialMask,
                   liveAccel: liveAccel, liveAttitude: liveAttitude)
        return view
    }

    func updateUIView(_ view: SCNView, context: Context) {
        applyState(view: view, coverageMask: coverageMask,
                   partialMask: partialMask,
                   liveAccel: liveAccel, liveAttitude: liveAttitude)
    }

    // MARK: - State application

    private func applyState(view: SCNView, coverageMask: UInt32,
                            partialMask: UInt32, liveAccel: SIMD3<Float>?,
                            liveAttitude: simd_quatf?) {
        guard let scene = view.scene else { return }
        // 3-state cell colouring (#148): cells start fully opaque red
        // (untouched), drop to mid-opacity once they have at least one
        // sample (in-progress / partialMask), then become transparent
        // once they cross the per-wedge sample threshold (captured /
        // coverageMask).  Gives the user honest feedback about how
        // much each orientation is contributing to the fit — a section
        // doesn't "vanish" on a fleeting visit.  Old firmware that
        // doesn't send partialMask just shows 2-state (untouched /
        // captured) — same as before.
        for cellIdx in 0..<MagCalCells.cellPolygons.count {
            guard let node = scene.rootNode.childNode(withName: "cell-\(cellIdx)", recursively: true),
                  let mat = node.geometry?.firstMaterial else { continue }
            let capturedBit = (coverageMask >> UInt32(cellIdx)) & 1
            let partialBit  = (partialMask  >> UInt32(cellIdx)) & 1
            mat.diffuse.contents = UIColor(red: 0.95, green: 0.35, blue: 0.35, alpha: 1.0)
            if capturedBit == 1 {
                mat.transparency = 0.0   // captured — clear
            } else if partialBit == 1 {
                mat.transparency = 0.28  // in-progress — half-faded
            } else {
                mat.transparency = 0.55  // untouched — full red
            }
        }
        // Rocket orientation.  Prefer the FC's EKF quaternion when
        // available — it gives full attitude including yaw, which the
        // accel-only path can't recover.  Falls back to the accel
        // method (pitch + roll only, yaw indeterminate) when the EKF
        // isn't publishing a valid quaternion yet.
        let rocketNode = scene.rootNode.childNode(withName: "rocket", recursively: false)
        if let q_body_to_world = liveAttitude {
            // EKF publishes q as body→NED (North-East-Down — see comment
            // on TelemetryData.q0).  SceneKit's world is right-handed
            // with Y-up.  We pre-multiply by a fixed remap that takes
            // NED vectors to SceneKit's world axes so the rocket
            // appears upright on screen when physically upright on the
            // pad.  Picked convention:
            //   NED X (north) → SceneKit -Z (away from viewer)
            //   NED Y (east)  → SceneKit +X (right)
            //   NED Z (down)  → SceneKit -Y (down)
            // Matrix has trace 0; quaternion = (w=0.5, x=0.5, y=0.5, z=-0.5).
            // If this looks wrong (rocket inverted or mirrored) the
            // EKF's world frame is likely a different ENU/ECEF/etc —
            // easy to iterate by tweaking these four numbers.
            let nedToScene = simd_quatf(ix: 0.5, iy: 0.5, iz: -0.5, r: 0.5)
            rocketNode?.simdOrientation = nedToScene * q_body_to_world
        } else if let g = liveAccel, simd_length(g) > 0.1 {
            // Accel fallback — pitch + roll only.  See class-comment
            // discussion of the yaw blind spot; spinning the rocket
            // around gravity doesn't move it on screen because the
            // accel-derived rotation has nothing to attach yaw to.
            let worldUp: SIMD3<Float> = SIMD3<Float>(0, 1, 0)
            let bodyUp = simd_normalize(g)
            let q = quaternionFromTo(bodyUp, worldUp)
            rocketNode?.simdOrientation = q
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
    /// vertex sits at the cell's centre on the sphere; edge points
    /// subdivide each Voronoi-arc edge so the straight chord segments
    /// approximate the great-circle arc.  Adjacent cells share the same
    /// subdivision points (lerp endpoints are the shared Voronoi
    /// vertices, and t = s / segmentsPerEdge is symmetric in s ↔ K−s),
    /// so the cells tile with no gaps along the boundary.  Slightly
    /// lifted above the sphere shell (radius 1.002) to avoid z-fighting.
    private static let segmentsPerEdge: Int = 6

    private static func makeCellGeometry(cellIdx: Int, captured: Bool) -> SCNGeometry {
        let lift: Float = 1.002
        let center = MagCalCells.cellCenters[cellIdx] * lift
        let edgeIdx = MagCalCells.cellPolygons[cellIdx]
        let nEdges = edgeIdx.count
        let segs = segmentsPerEdge

        var positions: [SIMD3<Float>] = [center]
        for i in 0..<nEdges {
            let a = MagCalCells.voronoiVertices[edgeIdx[i]]
            let b = MagCalCells.voronoiVertices[edgeIdx[(i + 1) % nEdges]]
            // Subdivide a → b into `segs` chord points; append s = 0..segs-1
            // (s = segs is the start of the next edge so it appears there).
            // Each point is lerped then re-normalized onto the unit sphere
            // — this matches the great-circle arc within ~(segs)⁻² of arc
            // length, which is invisible at segs = 6.
            for s in 0..<segs {
                let t = Float(s) / Float(segs)
                let p = simd_normalize(a + (b - a) * t) * lift
                positions.append(p)
            }
        }
        // Fan triangulation: (center, p[i], p[i+1])  i = 0..N-1, wrap
        var indices: [UInt16] = []
        let n = nEdges * segs    // total boundary points
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
        // Initial colour — applyState() re-tints on every update.  Keep
        // these in sync with applyState's branches so the very first
        // frame after the SCNView attaches matches the live state.
        mat.diffuse.contents = UIColor(red: 0.95, green: 0.35, blue: 0.35, alpha: 1.0)
        mat.transparency = captured ? 0.0 : 0.55
        mat.isDoubleSided = true
        mat.lightingModel = .lambert
        // Alpha-blended so captured (transparent) cells reveal whatever's
        // behind them — the rocket, and any back-side captured holes.
        mat.blendMode = .alpha
        mat.writesToDepthBuffer = false
        geometry.firstMaterial = mat
        return geometry
    }
}
