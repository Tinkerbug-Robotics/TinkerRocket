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
    /// `normalize(liveAccel)` is the body-frame direction of gravity —
    /// the exact direction the firmware's directionWedge() function
    /// buckets samples into.  Drives the on-sphere puck position.
    /// Pass `nil` (or zero vector) when no telemetry has arrived yet;
    /// the puck stays hidden in that case.
    let liveAccel: SIMD3<Float>?

    // MARK: - Tunables

    /// Camera distance — pulled back enough that the whole sphere fits
    /// comfortably with margins.
    private static let cameraZ: Float = 3.5
    /// Sphere geometry radius (cells live on this surface).
    private static let sphereR: Float = 1.0
    /// Ball radius — bigger than the old surface-puck because it's the
    /// star of the show now (iOS-style: ball inside wireframe globe).
    private static let ballRadius: Float = 0.15
    /// Ball position is normalize(accel) * ballOrbit, so it lives well
    /// INSIDE the cell sphere and is always visible through the cells'
    /// translucent material.  Fixes the previous "puck hidden behind
    /// the sphere" complaint — there is no front/back side at this
    /// radius, the ball just floats at the centre region of the sphere.
    private static let ballOrbit: Float = 0.62
    /// Halo radius — slightly larger ring of dark colour rendered just
    /// behind the ball to give it an outline so it pops against the
    /// red cells regardless of which one it's currently inside.
    private static let haloRadius: Float = 0.19

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
        // CRITICAL: don't write depth.  The shell is at radius 0.995 —
        // *closer* to the camera than the ball, which orbits at 0.62.
        // If the shell writes depth (the default), the depth buffer
        // ends up saying "this pixel is at depth 0.995, anything
        // further away is occluded" — and the ball, sitting at z<0.77
        // from the camera's view, fails the depth test and disappears.
        // That was the "no ball visible" bug in the iter13/iter14
        // screenshots.
        shell.firstMaterial?.writesToDepthBuffer = false
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

        // Gravity ball — bright YELLOW sphere positioned INSIDE the
        // cell sphere at the body-frame gravity direction, scaled to
        // ballOrbit (~0.62 of sphereR).  This is the iOS Compass-cal
        // pattern: ball lives inside a transparent shell, always
        // visible from any camera angle, and the user "rolls" it by
        // tilting the device.  Yellow because the cells are red — same
        // colour would blend in and the ball would disappear (which is
        // exactly what happened in the first ball-inside iteration).
        let ball = SCNSphere(radius: CGFloat(MagCalSphereView.ballRadius))
        ball.segmentCount = 32
        // Constant lighting + bright emission so the ball glows on its
        // own without depending on the scene's directional light.
        ball.firstMaterial?.diffuse.contents = UIColor(red: 1.0, green: 0.85, blue: 0.04, alpha: 1.0)
        ball.firstMaterial?.emission.contents = UIColor(red: 1.0, green: 0.78, blue: 0.0, alpha: 1.0)
        ball.firstMaterial?.lightingModel = .constant
        // Belt-and-suspenders: even if something else in the scene
        // ever writes depth above the ball, ignore depth reads so the
        // ball is guaranteed to paint (combined with renderingOrder
        // = 100 below, it always lands on top of cells + shell).
        ball.firstMaterial?.readsFromDepthBuffer = false
        ball.firstMaterial?.writesToDepthBuffer = false
        let ballNode = SCNNode(geometry: ball)
        ballNode.name = "gravityBall"
        // Render after the cells so the ball stays visible on top of
        // front-facing translucent cells.  Without this, occasional
        // sort-order quirks could let cells paint over the ball.
        ballNode.renderingOrder = 100
        // Start hidden until we get the first valid accel reading.
        ballNode.isHidden = true
        scene.rootNode.addChildNode(ballNode)

        // Dark halo around the ball — gives the yellow ball an outline
        // so it reads as a distinct object even when it's inside a red
        // cell that happens to render at the same screen depth.  Slightly
        // larger than the ball, low rendering order so the bright ball
        // paints on top.
        let halo = SCNSphere(radius: CGFloat(MagCalSphereView.haloRadius))
        halo.segmentCount = 24
        halo.firstMaterial?.diffuse.contents = UIColor(white: 0.0, alpha: 1.0)
        halo.firstMaterial?.emission.contents = UIColor(white: 0.0, alpha: 1.0)
        halo.firstMaterial?.lightingModel = .constant
        halo.firstMaterial?.readsFromDepthBuffer = false
        halo.firstMaterial?.writesToDepthBuffer = false
        let haloNode = SCNNode(geometry: halo)
        haloNode.name = "gravityBallHalo"
        haloNode.renderingOrder = 99
        haloNode.isHidden = true
        scene.rootNode.addChildNode(haloNode)

        // Initial state
        applyState(view: view, coverageMask: coverageMask,
                   partialMask: partialMask, liveAccel: liveAccel)
        return view
    }

    func updateUIView(_ view: SCNView, context: Context) {
        applyState(view: view, coverageMask: coverageMask,
                   partialMask: partialMask, liveAccel: liveAccel)
    }

    // MARK: - State application

    private func applyState(view: SCNView, coverageMask: UInt32,
                            partialMask: UInt32, liveAccel: SIMD3<Float>?) {
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
        // Ball position — placed INSIDE the cell sphere at radius
        // ballOrbit, in the direction of body-frame gravity.  Because
        // the ball is well inside the (translucent) cell shell, it's
        // visible from any camera angle — no more "puck hidden behind
        // the sphere" problem.  Yaw (rotation around gravity) doesn't
        // move the ball, which is correct: yawing the rocket doesn't
        // capture new cells either.
        let ballNode = scene.rootNode.childNode(withName: "gravityBall", recursively: false)
        let haloNode = scene.rootNode.childNode(withName: "gravityBallHalo", recursively: false)
        if let g = liveAccel, simd_length(g) > 0.1 {
            let bodyUp = simd_normalize(g)
            let pos = bodyUp * MagCalSphereView.ballOrbit
            ballNode?.simdPosition = pos
            haloNode?.simdPosition = pos
            ballNode?.isHidden = false
            haloNode?.isHidden = false
        } else {
            ballNode?.isHidden = true
            haloNode?.isHidden = true
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
