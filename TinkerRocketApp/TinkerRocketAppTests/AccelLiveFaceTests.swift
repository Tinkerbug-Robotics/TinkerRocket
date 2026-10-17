import XCTest
@testable import TinkerRocketApp

/// #1629 round 2: which face the board is on now, from telemetry -- the
/// mounting port (orientCodeToMatrix), the live face and its line, the
/// highlighted row, the Measure label and a tapped row's placement help.
final class AccelLiveFaceTests: XCTestCase {

    private typealias Face = AccelSixFace.Face
    private typealias LiveFace = AccelSixFace.LiveFace
    private typealias Mounting = AccelSixFace.Mounting

    private func face(_ name: String) -> Face {
        Face.allCases.first { $0.name == name }!
    }

    /// The live face through a mounting whose rotation is its code's (default,
    /// manual, auto-snap); `exact` for auto-exact, which names no tilt.
    private func live(_ rocket: [Double]?, code: Int?, exact: Bool = false) -> LiveFace {
        AccelSixFace.liveFace(rocket: rocket,
                              mounting: code.map { AccelSixFace.LiveMounting(code: $0, tiltKnown: !exact) })
    }

    // MARK: - The mounting port

    /// orientCodeToMatrix itself, compiled from TR_Orientation.cpp on the host
    /// and run for every code (row-major, v_rocket = R * v_board).
    private let firmwareMatrices: [[Int]] = [
        [1, 0, 0,  0, 1, 0,  0, 0, 1],   // 0
        [1, 0, 0,  0, 0, -1,  0, 1, 0],   // 1
        [1, 0, 0,  0, -1, 0,  0, 0, -1],   // 2
        [1, 0, 0,  0, 0, 1,  0, -1, 0],   // 3
        [-1, 0, 0,  0, -1, 0,  0, 0, 1],   // 4
        [-1, 0, 0,  0, 0, -1,  0, -1, 0],   // 5
        [-1, 0, 0,  0, 1, 0,  0, 0, -1],   // 6
        [-1, 0, 0,  0, 0, 1,  0, 1, 0],   // 7
        [0, 1, 0,  -1, 0, 0,  0, 0, 1],   // 8
        [0, 1, 0,  0, 0, -1,  -1, 0, 0],   // 9
        [0, 1, 0,  1, 0, 0,  0, 0, -1],   // 10
        [0, 1, 0,  0, 0, 1,  1, 0, 0],   // 11
        [0, -1, 0,  1, 0, 0,  0, 0, 1],   // 12
        [0, -1, 0,  0, 0, -1,  1, 0, 0],   // 13
        [0, -1, 0,  -1, 0, 0,  0, 0, -1],   // 14
        [0, -1, 0,  0, 0, 1,  -1, 0, 0],   // 15
        [0, 0, 1,  0, 1, 0,  -1, 0, 0],   // 16
        [0, 0, 1,  1, 0, 0,  0, 1, 0],   // 17
        [0, 0, 1,  0, -1, 0,  1, 0, 0],   // 18
        [0, 0, 1,  -1, 0, 0,  0, -1, 0],   // 19
        [0, 0, -1,  0, 1, 0,  1, 0, 0],   // 20
        [0, 0, -1,  -1, 0, 0,  0, 1, 0],   // 21
        [0, 0, -1,  0, -1, 0,  -1, 0, 0],   // 22
        [0, 0, -1,  1, 0, 0,  0, -1, 0],   // 23
    ]

    func testEveryCodeMatchesTheFirmwareFunction() throws {
        for code in 0..<24 {
            XCTAssertEqual(try XCTUnwrap(Mounting.matrix(code)), firmwareMatrices[code], "code \(code)")
        }
    }

    /// Codes 0, 16 and 23 worked by hand from R = Rx(clock) * B[nose_sel].
    func testCodes0And16And23ByHand() {
        // 0: +X to the nose, no clocking: identity.
        XCTAssertEqual(Mounting.matrix(0), [1, 0, 0, 0, 1, 0, 0, 0, 1])
        // 16: +Z to the nose (nose_sel 4), clock 0: B itself, 90 degrees about +Y.
        XCTAssertEqual(Mounting.matrix(16), [0, 0, 1, 0, 1, 0, -1, 0, 0])
        // 23: -Z to the nose (nose_sel 5), clock 3: c = 0, s = -1, so row 0 = B row 0,
        // row 1 = -s B row 2 = B row 2, row 2 = s B row 1 = -B row 1.
        XCTAssertEqual(Mounting.matrix(23), [0, 0, -1, 1, 0, 0, 0, -1, 0])
    }

    func testEveryCodeIsAProperRotation() throws {
        for code in 0..<24 {
            let r = try XCTUnwrap(Mounting.matrix(code))
            for i in 0..<3 {
                for j in 0..<3 {
                    let dot: Int = (0..<3).reduce(0) { $0 + r[3 * i + $1] * r[3 * j + $1] }
                    XCTAssertEqual(dot, i == j ? 1 : 0, "code \(code): rows \(i), \(j) not orthonormal")
                }
            }
            let minor0: Int = r[4] * r[8] - r[5] * r[7]
            let minor1: Int = r[3] * r[8] - r[5] * r[6]
            let minor2: Int = r[3] * r[7] - r[4] * r[6]
            let det: Int = r[0] * minor0 - r[1] * minor1 + r[2] * minor2
            XCTAssertEqual(det, 1, "code \(code)")
        }
    }

    func testTheSelectedBoardAxisGoesToTheNose() throws {
        // nose_sel = code / 4: +X, -X, +Y, -Y, +Z, -Z; the clocking turns about the nose.
        let noseAxes: [[Int]] = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
        for code in 0..<24 {
            let r = try XCTUnwrap(Mounting.matrix(code))
            let n = noseAxes[code / 4]
            let rocket: [Int] = (0..<3).map { row -> Int in
                (0..<3).reduce(0) { $0 + r[3 * row + $1] * n[$1] }
            }
            XCTAssertEqual(rocket, [1, 0, 0], "code \(code)")
        }
    }

    func testAnInvalidCodeGivesNoHint() {
        XCTAssertNil(Mounting.matrix(24))
        XCTAssertNil(Mounting.matrix(31), "auto-exact on the relay: no discrete code")
        XCTAssertNil(Mounting.matrix(-1))
        XCTAssertEqual(live([9.8, 0, 0], code: 24), .waiting)
        XCTAssertEqual(live([9.8, 0, 0], code: 31), .waiting)
    }

    // MARK: - The live face

    func testTheLiveFaceThroughTheMounting() {
        XCTAssertEqual(live([9.8, 0, 0], code: 16),
                       .on(face("+Z UP"), tiltDeg: 0), "+Z to the nose, the nose up")
        XCTAssertEqual(live([0, 9.8, 0], code: 0),
                       .on(face("+Y UP"), tiltDeg: 0))
        XCTAssertEqual(live([-9.8, 0, 0], code: 8),
                       .on(face("+Y DOWN"), tiltDeg: 0), "+Y to the nose, the nose down")
    }

    func testATiltedReadingIsAskew() {
        let t = 20 * Double.pi / 180
        let rocket: [Double] = [9.8 * cos(t), 9.8 * sin(t), 0]
        guard case .askew(let f, let tilt) = live(rocket, code: 0) else {
            return XCTFail("20 degrees off vertical is askew")
        }
        XCTAssertEqual(f, face("+X UP"))
        XCTAssertEqual(tilt, 20, accuracy: 1e-9)
        // 10 degrees is still square enough, as for a measurement.
        let ten = 9.9 * Double.pi / 180
        let square: [Double] = [9.8 * cos(ten), 0, 9.8 * sin(ten)]
        XCTAssertEqual(live(square, code: 0).face, face("+X UP"))
        if case .askew = live(square, code: 0) {
            XCTFail("9.9 degrees is on, not askew")
        }
    }

    func testAReadingOffOneGIsMoving() {
        XCTAssertEqual(live([0, 0, 8.5], code: 0), .moving)
        XCTAssertEqual(live([0, 0, 8.8], code: 0), .moving, "1.007 off")
        XCTAssertEqual(live([0, 0, 8.85], code: 0),
                       .on(face("+Z UP"), tiltDeg: 0), "0.957 off: within 0.1 g")
    }

    func testNothingToGoOnIsWaiting() {
        XCTAssertEqual(live(nil, code: 0), .waiting, "a field missing")
        XCTAssertEqual(live([0, 0, 9.8], code: nil), .waiting, "no mounting")
    }

    func testWhichMountingTheReadingIsIn() {
        typealias M = AccelSixFace.LiveMounting
        // The frame's own imo wins: code = imo & 0x1F (31, the relay's auto-exact, names none).
        XCTAssertEqual(AccelSixFace.liveMounting(imo: (2 << 5) | 16, reportedCode: 3, reportedMode: .manual),
                       M(code: 16, tiltKnown: true))
        XCTAssertEqual(AccelSixFace.liveMounting(imo: (3 << 5) | 31, reportedCode: 3, reportedMode: .manual),
                       M(code: 31, tiltKnown: true))
        // Without it, the rocket's own imu_orient report: modes 0, 1 and 2 with a true tilt...
        XCTAssertEqual(AccelSixFace.liveMounting(imo: nil, reportedCode: 0, reportedMode: .defaultMounting),
                       M(code: 0, tiltKnown: true))
        XCTAssertEqual(AccelSixFace.liveMounting(imo: nil, reportedCode: 5, reportedMode: .manual),
                       M(code: 5, tiltKnown: true))
        XCTAssertEqual(AccelSixFace.liveMounting(imo: nil, reportedCode: 8, reportedMode: .autoSnap),
                       M(code: 8, tiltKnown: true))
        // ...and auto-exact (3) with the nearest axis's code but no tilt.
        XCTAssertEqual(AccelSixFace.liveMounting(imo: nil, reportedCode: 16, reportedMode: .autoExact),
                       M(code: 16, tiltKnown: false))
        XCTAssertNil(AccelSixFace.liveMounting(imo: nil, reportedCode: 8, reportedMode: .unknown))
        XCTAssertNil(AccelSixFace.liveMounting(imo: nil, reportedCode: nil, reportedMode: .autoSnap))
    }

    /// Auto-exact: the code names the board axis nearest up, beside an exact
    /// rotation. The face is right; the tilt through the code snaps to ~0, so
    /// the line leaves it out and is never askew -- the measurement judges it.
    /// And only once the reading sits within 5° of rocket +X, as it does
    /// whenever the flight computer has read the board where it now lies.
    func testAutoExactNamesTheFaceWithoutATilt() {
        let exact = live([9.8, 0, 0], code: 16, exact: true)
        XCTAssertEqual(exact, .on(face("+Z UP"), tiltDeg: nil))
        XCTAssertEqual(exact.text, "On now: +Z UP (component side up)")
        XCTAssertEqual(exact.face, face("+Z UP"), "its row is highlighted")
        XCTAssertEqual(AccelSixFace.measureLabel(live: exact, allCaptured: false), "Measure +Z UP")
        let settled = 4 * Double.pi / 180
        let near: [Double] = [9.8 * cos(settled), 9.8 * sin(settled), 0]
        XCTAssertEqual(live(near, code: 0, exact: true), .on(face("+X UP"), tiltDeg: nil), "no askew")
        let t = 20 * Double.pi / 180
        let tilted: [Double] = [9.8 * cos(t), 9.8 * sin(t), 0]
        XCTAssertEqual(live(tilted, code: 0, exact: true), .moving, "not re-read yet")
        if case .askew(let f, let tilt) = live(tilted, code: 0) {
            XCTAssertEqual(f, face("+X UP"))
            XCTAssertEqual(tilt, 20, accuracy: 1e-9, "auto-snap keeps its tilt")
        } else {
            XCTFail("auto-snap: askew")
        }
        XCTAssertEqual(live([8.5, 0, 0], code: 16, exact: true), .moving, "still moving")
        XCTAssertEqual(live([9.8, 0, 0], code: 31, exact: true), .waiting, "still no code, no hint")
    }

    /// Why auto-exact waits for the reading to settle: the flight computer
    /// turns the reading with the exact rotation it last read
    /// (orientMatrixFromNoseVector, ported below) but names the nearest axis's
    /// code beside it, so a board set on another face reads, through that
    /// code, as some other face until the flight computer re-reads it (after
    /// 2 s still). Over exact mountings 16-40° off each axis and the board then
    /// set on any face: never a face it is not on, where the bare turn-back
    /// names a wrong one.
    func testAutoExactNeverNamesAFaceTheBoardIsNotOn() {
        var wrongWithoutTheWait = 0
        for nose in Face.allCases {
            for lean in [16.0, 25.0, 40.0] {
                for az in stride(from: 0.0, to: 360.0, by: 30.0) {
                    let n = leaned(nose, lean, az)
                    let r = exactRotation(n)
                    let code = nearestNoseCode(n)
                    XCTAssertEqual(live(rotate(r, n), code: code, exact: true), .on(nose, tiltDeg: nil),
                                   "settled: \(nose) \(lean) \(az)")
                    for now in Face.allCases {
                        for tilt in [0.0, 6.0] {
                            let reading = rotate(r, leaned(now, tilt, az + 45))
                            let shown = live(reading, code: code, exact: true)
                            XCTAssertTrue(shown == .moving || shown.face == now,
                                          "\(nose) \(lean) \(az) on \(now): \(shown)")
                            let bare = AccelSixFace.identify(Mounting.toBoard(reading, code: code)!)
                            if bare.face != now { wrongWithoutTheWait += 1 }
                        }
                    }
                }
            }
        }
        XCTAssertGreaterThan(wrongWithoutTheWait, 0, "the case the wait is for")
    }

    /// A unit vector `leanDeg` off `face`'s up direction, at `azDeg` around it.
    private func leaned(_ face: Face, _ leanDeg: Double, _ azDeg: Double) -> [Double] {
        var u = [0.0, 0.0, 0.0], p1 = [0.0, 0.0, 0.0], p2 = [0.0, 0.0, 0.0]
        u[face.axis] = face.up ? 1 : -1
        p1[(face.axis + 1) % 3] = 1
        p2[(face.axis + 2) % 3] = 1
        let l = leanDeg * .pi / 180, a = azDeg * .pi / 180
        return (0..<3).map { cos(l) * u[$0] + sin(l) * (cos(a) * p1[$0] + sin(a) * p2[$0]) }
    }

    /// orientMatrixFromNoseVector: the shortest arc taking unit `a` to rocket +X, row-major.
    private func exactRotation(_ a: [Double]) -> [Double] {
        let c = a[0]
        if c < -1 + 1e-6 { return [-1, 0, 0, 0, -1, 0, 0, 0, 1] }
        let ky = a[2], kz = -a[1], f = 1 / (1 + c)
        return [1 - f * (ky * ky + kz * kz), -kz, ky,
                kz, 1 - f * kz * kz, f * ky * kz,
                -ky, f * ky * kz, 1 - f * ky * ky]
    }

    /// orientNearestNoseCode: the clock-0 code of the board axis nearest `v`.
    private func nearestNoseCode(_ v: [Double]) -> Int {
        var axis = 0
        for i in 1..<3 where abs(v[i]) > abs(v[axis]) { axis = i }
        return (axis * 2 + (v[axis] < 0 ? 1 : 0)) * 4
    }

    /// One g along unit `v` (board frame), turned by `r` into the rocket frame.
    private func rotate(_ r: [Double], _ v: [Double]) -> [Double] {
        (0..<3).map { i in AccelSixFace.g0 * (r[3 * i] * v[0] + r[3 * i + 1] * v[1] + r[3 * i + 2] * v[2]) }
    }

    // MARK: - What the screen shows

    func testTheLiveLine() {
        XCTAssertEqual(LiveFace.on(face("+Y UP"), tiltDeg: 5.3).text,
                       "On now: +Y UP (left side up), 5° off vertical")
        XCTAssertEqual(LiveFace.on(face("+X DOWN"), tiltDeg: 0.4).text,
                       "On now: +X DOWN (forward end down), 0° off vertical")
        XCTAssertEqual(LiveFace.on(face("+Z UP"), tiltDeg: 9.6).text,
                       "On now: +Z UP (component side up), 10° off vertical")
        XCTAssertEqual(LiveFace.askew(face("+X UP"), tiltDeg: 12.6).text,
                       "Set it squarer: +X UP is 13° off vertical")
        XCTAssertEqual(LiveFace.moving.text, "Hold it still")
        XCTAssertEqual(LiveFace.waiting.text, "Waiting for the rocket's reading\u{2026}")
    }

    func testTheHighlightedRow() {
        XCTAssertEqual(LiveFace.on(face("+Y UP"), tiltDeg: 3).face, face("+Y UP"))
        XCTAssertEqual(LiveFace.askew(face("+Z DOWN"), tiltDeg: 14).face, face("+Z DOWN"))
        XCTAssertNil(LiveFace.moving.face)
        XCTAssertNil(LiveFace.waiting.face)
    }

    func testTheMeasureLabel() {
        let on = LiveFace.on(face("+Y UP"), tiltDeg: 5.3)
        XCTAssertEqual(AccelSixFace.measureLabel(live: on, allCaptured: false), "Measure +Y UP")
        XCTAssertEqual(AccelSixFace.measureLabel(live: on, allCaptured: true), "Measure +Y UP")
        for live in [LiveFace.askew(face("+Y UP"), tiltDeg: 12), .moving, .waiting] {
            XCTAssertEqual(AccelSixFace.measureLabel(live: live, allCaptured: false), "Measure This Face")
            XCTAssertEqual(AccelSixFace.measureLabel(live: live, allCaptured: true), "Measure a Face Again")
        }
    }

    func testEachFacesPlacement() {
        XCTAssertEqual(Face.allCases.map(\.placementText), [
            "To measure +X UP: stand it on its back end, forward end up.",
            "To measure +X DOWN: stand it on its forward end, forward end down.",
            "To measure +Y UP: lay it on its right side, left side up.",
            "To measure +Y DOWN: lay it on its left side, left side down.",
            "To measure +Z UP: lay it flat, component side up.",
            "To measure +Z DOWN: lay it flat, component side down.",
        ])
        XCTAssertEqual(Face.allCases.map(\.hint),
                       ["forward end up", "forward end down", "left side up", "left side down",
                        "component side up", "component side down"])
    }

    func testATappedRowShowsItsPlacementUntilAMeasurementStarts() {
        var flow = AccelCalFlow()
        XCTAssertNil(flow.placementFace)
        flow.showPlacement(face("+Y UP"))
        XCTAssertEqual(flow.placementFace, face("+Y UP"))
        flow.showPlacement(face("+Z DOWN"))
        XCTAssertEqual(flow.placementFace, face("+Z DOWN"), "the next tap replaces it")
        XCTAssertEqual(flow.measure(), .measureFace)
        XCTAssertNil(flow.placementFace, "cleared when a measurement starts")
        flow.showPlacement(face("+X UP"))
        XCTAssertNil(flow.measure(), "one request at a time")
        XCTAssertEqual(flow.placementFace, face("+X UP"), "a measure that did not start leaves it")
    }

    // MARK: - On the device

    @MainActor
    func testTheLiveFaceOnTheRocketsOwnLink() throws {
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        device.isConnected = true
        func feed(_ json: String) { device.parseTelemetryData(json.data(using: .utf8)!) }

        // The rocket's own link carries no imo: waiting until imu_orient says
        // which mounting lx/ly/lz are in.
        feed(#"{"st":"READY","fs":16,"lx":9.8,"ly":0.0,"lz":0.1}"#)
        XCTAssertEqual(device.accelLiveFace, .waiting)
        feed(#"{"type":"imu_orient","code":16,"mode":2,"name":"+Z","set":255}"#)
        XCTAssertEqual(device.imuOrientationCode, 16)
        guard case .on(let f, let tilt?) = device.accelLiveFace else {
            return XCTFail("+Z to the nose, the nose up: +Z UP")
        }
        XCTAssertEqual(f, face("+Z UP"))
        XCTAssertEqual(tilt, atan(0.1 / 9.8) * 180 / Double.pi, accuracy: 1e-4)
        XCTAssertEqual(device.accelLiveFace.text, "On now: +Z UP (component side up), 1° off vertical")

        // Auto-exact (mode 3, as the user's Beetle reports it): the face, no tilt.
        feed(#"{"type":"imu_orient","code":16,"mode":3,"name":"+Z","set":255}"#)
        XCTAssertEqual(device.accelLiveFace, .on(face("+Z UP"), tiltDeg: nil))
        XCTAssertEqual(device.accelLiveFace.text, "On now: +Z UP (component side up)")

        // A frame that carries its own mounting (mode 2, code 0) goes by it.
        feed(#"{"st":"READY","fs":16,"lx":9.8,"ly":0.0,"lz":0.1,"imo":64}"#)
        XCTAssertEqual(device.accelLiveFace.face, face("+X UP"))

        device.onDisconnect()
        XCTAssertEqual(device.accelLiveFace, .waiting)
        XCTAssertNil(device.imuOrientationCode, "not carried into the next link")
    }
}
