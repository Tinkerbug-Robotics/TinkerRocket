import Combine
import XCTest
@testable import TinkerRocketApp

/// #1629: the 0xD1 accelerometer-calibration status against the golden the C++
/// emitter wrote from the real AccelCalStatusData, command 76 byte for byte
/// against its golden, 77 and 78, and how the device hands the frames on.
final class AccelCalStatusTests: XCTestCase {

    private let statusFixture = "fileops/accelcal_0xD1_40.bin"
    private let commandFixture = "commands/cmd76_accelcal_12.bin"

    /// The golden's payload, after the 0xD1.
    private var goldenPayload: [UInt8] {
        Array([UInt8](WireFixtures.data(statusFixture)).dropFirst())
    }

    private func patched(_ edits: [Int: UInt8]) -> AccelCalStatus {
        var p = goldenPayload
        for (i, b) in edits { p[i] = b }
        return AccelCalStatus.decode(p)!
    }

    // MARK: - 0xD1 decode

    func testDecodesEveryFieldOfTheGolden() throws {
        let side = WireFixtures.sidecar(statusFixture)
        let frame = [UInt8](WireFixtures.data(statusFixture))
        XCTAssertEqual(frame.count, 41)
        XCTAssertEqual(frame.first, 0xD1)
        let s = try XCTUnwrap(AccelCalStatus.decode(Array(frame.dropFirst())))
        XCTAssertEqual(Int(s.flags), side.int("flags"))
        XCTAssertEqual(Int(s.faceState), side.int("face_state"))
        XCTAssertEqual(s.offset, [side.f32("offset_x"), side.f32("offset_y"), side.f32("offset_z")])
        XCTAssertEqual(s.faceMean, [side.f32("face_mean_x"), side.f32("face_mean_y"), side.f32("face_mean_z")])
        XCTAssertEqual(s.faceSD, [side.f32("face_sd_x"), side.f32("face_sd_y"), side.f32("face_sd_z")])
        XCTAssertEqual(Int(s.faceN), side.int("face_n"))

        // What the screen reads off them: flags 3, face_state 2.
        XCTAssertTrue(s.isStored)
        XCTAssertTrue(s.isApplied)
        XCTAssertEqual(s.state, .done)
        XCTAssertEqual(s.useText, "In use by the flight computer")
        XCTAssertEqual(s.offsetsText, "X +4.20 mg, Y -2.78 mg, Z +1.59 mg")
    }

    func testEachFlagBitReadsAlone() {
        let stored = patched([0: 0x01])
        XCTAssertTrue(stored.isStored)
        XCTAssertFalse(stored.isApplied)
        XCTAssertEqual(stored.useText, "Stored, but not in use while a sim runs")
        let applied = patched([0: 0x02])
        XCTAssertFalse(applied.isStored)
        XCTAssertTrue(applied.isApplied)
        let none = patched([0: 0x00])
        XCTAssertEqual(none.useText, "No offset stored")
        XCTAssertEqual(none.offsetsText, "no offset stored")
    }

    func testEachFaceStateDecodes() {
        XCTAssertEqual(patched([1: 0]).state, .idle)
        XCTAssertEqual(patched([1: 1]).state, .measuring)
        XCTAssertEqual(patched([1: 2]).state, .done)
        XCTAssertEqual(patched([1: 3]).state, .failed)
        let unknown = patched([1: 9])
        XCTAssertNil(unknown.state, "a newer state is kept raw, not guessed")
        XCTAssertEqual(unknown.faceState, 9)
    }

    func testAShortPayloadIsNotDecoded() {
        XCTAssertNil(AccelCalStatus.decode(Array(goldenPayload.prefix(39))))
        XCTAssertNotNil(AccelCalStatus.decode(goldenPayload))
    }

    /// The tests' frame builder (below) against the golden, so the frames the
    /// flow tests feed the device are laid out as the firmware lays them out.
    func testTheTestFrameBuilderRoundTripsTheGolden() throws {
        let s = try XCTUnwrap(AccelCalStatus.decode(goldenPayload))
        XCTAssertEqual(s.wireFrame, WireFixtures.data(statusFixture))
    }

    // MARK: - The device

    @MainActor
    func testTheDeviceTakesA41ByteFrameAndDropsAShorterOne() {
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        let frame = WireFixtures.data(statusFixture)
        device.handleFileOpsFrame(frame.prefix(40))
        XCTAssertNil(device.accelCalStatus, "frame length < 41: no decode")
        device.handleFileOpsFrame(frame)
        XCTAssertEqual(device.accelCalStatus, AccelCalStatus.decode(goldenPayload))
        // The neighbours keep their own frames.
        XCTAssertNil(device.padCalStatus)
    }

    @MainActor
    func testEveryFrameIsHandedOnInOrderEvenWhenTwoAreEqual() {
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        var seen: [AccelCalStatus] = []
        let sub = device.accelCalFrames.sink { seen.append($0) }
        let measuring = AccelCalStatus.face(.measuring, mean: [0, 0, 0], n: 0)
        device.handleFileOpsFrame(measuring.wireFrame)
        device.handleFileOpsFrame(measuring.wireFrame)
        device.handleFileOpsFrame(WireFixtures.data(statusFixture).prefix(40))   // dropped
        device.handleFileOpsFrame(WireFixtures.data(statusFixture))
        XCTAssertEqual(seen, [measuring, measuring, AccelCalStatus.decode(goldenPayload)!])
        sub.cancel()
    }

    @MainActor
    func testADisconnectForgetsTheStatus() {
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        device.isConnected = true
        device.handleFileOpsFrame(WireFixtures.data(statusFixture))
        XCTAssertNotNil(device.accelCalStatus)
        device.onDisconnect()
        XCTAssertNil(device.accelCalStatus)
    }

    // MARK: - Commands 76, 77, 78

    /// What sendRawCommand writes on the command characteristic.
    @MainActor
    private func wire(_ send: (BLEDevice) -> Void) -> [Data] {
        let device = BLEDevice(peripheral: nil, name: "TR-R-Test")
        var writes: [Data] = []
        device.commandTap = { cmd, payload in writes.append(Data([cmd]) + payload) }
        send(device)
        return writes
    }

    @MainActor
    func testCommand76IsByteExactAgainstTheGolden() {
        let side = WireFixtures.sidecar(commandFixture)
        XCTAssertEqual(side.int("cmd"), 76)
        XCTAssertEqual([side.f32("offset_x"), side.f32("offset_y"), side.f32("offset_z")],
                       [0.0412, -0.0273, 0.0156])
        let writes = wire { $0.sendAccelCalApply(x: 0.0412, y: -0.0273, z: 0.0156) }
        XCTAssertEqual(writes, [WireFixtures.data(commandFixture)])
        XCTAssertEqual(AccelCalApply.payload(x: 0.0412, y: -0.0273, z: 0.0156),
                       WireFixtures.data(commandFixture).dropFirst())
    }

    @MainActor
    func testAClearIsCommand76WithTwelveZeroBytes() {
        XCTAssertEqual(wire { $0.sendAccelCalApply(x: 0, y: 0, z: 0) }, [Data([76]) + Data(count: 12)])
    }

    @MainActor
    func testReadAndFaceAreBareCommands() {
        XCTAssertEqual(wire { $0.sendAccelCalRead() }, [Data([77])])
        XCTAssertEqual(wire { $0.sendAccelFaceMeasure() }, [Data([78])])
    }
}

// MARK: - Test frames

extension AccelCalStatus {

    /// A face-window frame: `mean` in m/s², SD 0.012 m/s² on every axis.
    static func face(_ state: FaceState, mean: [Double], n: UInt16 = 2880,
                     flags: UInt8 = 0, offset: [Float] = [0, 0, 0]) -> AccelCalStatus {
        AccelCalStatus(flags: flags, faceState: state.rawValue, offset: offset,
                       faceMean: mean.map(Float.init), faceSD: [0.012, 0.012, 0.012], faceN: n)
    }

    /// A status frame saying what is stored.
    static func stored(_ offset: [Float]?) -> AccelCalStatus {
        AccelCalStatus(flags: offset == nil ? 0 : AccelCalStatus.flagStored | AccelCalStatus.flagApplied,
                       faceState: FaceState.idle.rawValue, offset: offset ?? [0, 0, 0],
                       faceMean: [0, 0, 0], faceSD: [0, 0, 0], faceN: 0)
    }

    /// The [0xD1][AccelCalStatusData] frame for this status. Only drives the
    /// device in tests; the golden pins the layout (see the round-trip test).
    var wireFrame: Data {
        var d = Data([0xD1, flags, faceState])
        for v in offset + faceMean + faceSD {
            withUnsafeBytes(of: v.bitPattern.littleEndian) { d.append(contentsOf: $0) }
        }
        withUnsafeBytes(of: faceN.littleEndian) { d.append(contentsOf: $0) }
        return d
    }
}
