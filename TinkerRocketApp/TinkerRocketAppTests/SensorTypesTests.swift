import XCTest
@testable import TinkerRocketApp

/// Verifies that each sensor struct's binary `init(from:)` accepts exactly
/// the expected wire size.  Swift's `MemoryLayout<T>.size` is NOT the right
/// check here — it includes alignment padding and has no relation to what
/// the C++ side writes.  What does matter is that passing N bytes succeeds
/// and passing N-1 bytes throws, because the init reads the wire fields
/// by offset.
final class SensorTypesTests: XCTestCase {

    private func assertSize<T>(_ expected: Int, _ init_: (Data) throws -> T,
                                file: StaticString = #file, line: UInt = #line) {
        XCTAssertNoThrow(try init_(Data(count: expected)), file: file, line: line)
        XCTAssertThrowsError(try init_(Data(count: expected - 1)), file: file, line: line)
    }

    func testGNSSData_Size42() {
        assertSize(42) { try GNSSData(from: $0) }
    }

    func testISM6HG256Data_Size22() {
        assertSize(22) { try ISM6HG256Data(from: $0) }
    }

    func testBMP585Data_Size12() {
        assertSize(12) { try BMP585Data(from: $0) }
    }

    func testMMC5983MAData_Size16() {
        assertSize(16) { try MMC5983MAData(from: $0) }
    }

    func testPOWERData_Size10() {
        assertSize(10) { try POWERData(from: $0) }
    }

    func testNonSensorData_Size43_LegacyAccepted() {
        // Per #142/#143 the wire format grew from 43 to 44 bytes (added the
        // apogee_flags byte).  Legacy 43-byte logs must still parse so old
        // captures in TestFlights/ can be re-converted with new iOS builds.
        XCTAssertNoThrow(try NonSensorData(from: Data(count: 43)))
        XCTAssertThrowsError(try NonSensorData(from: Data(count: 42)))
    }

    func testNonSensorData_Size44_Current() {
        // Current firmware emits 44-byte frames.
        XCTAssertNoThrow(try NonSensorData(from: Data(count: 44)))
    }

    func testNonSensorData_ApogeeFlagsDecodes() {
        // Construct a 44-byte payload with apogee_flags = NSF2_MASTER_APOGEE.
        var bytes = [UInt8](repeating: 0, count: 44)
        bytes[43] = 1 << 2  // NSF2_MASTER_APOGEE
        let raw = try? NonSensorData(from: Data(bytes))
        XCTAssertNotNil(raw)
        XCTAssertEqual(raw?.apogee_flags, 1 << 2)
    }

    func testNonSensorData_LegacyApogeeFlagsZero() {
        // 43-byte legacy payloads have no apogee_flags byte; decoder must
        // return 0 (not crash, not read past the buffer).
        let raw = try? NonSensorData(from: Data(count: 43))
        XCTAssertNotNil(raw)
        XCTAssertEqual(raw?.apogee_flags, 0)
    }

    func testNonSensorData_Size50_EkfTicksDecodes() throws {
        // #529: 50-byte layout appends uint16 ekf_ticks after the uint32
        // sensor_health (#303, offset 44, never surfaced by the app).
        var bytes = [UInt8](repeating: 0, count: 50)
        bytes[48] = 0x34  // little-endian 0x1234 = 4660
        bytes[49] = 0x12
        let raw = try NonSensorData(from: Data(bytes))
        XCTAssertEqual(raw.ekf_ticks, 0x1234)
    }

    func testNonSensorData_PreEkfTicksLayoutsDecodeNil() throws {
        // 44- and 48-byte layouts predate ekf_ticks: decode nil (not 0 —
        // 0 is a real counter value, meaning "EKF not yet initialized").
        XCTAssertNil(try NonSensorData(from: Data(count: 44)).ekf_ticks)
        XCTAssertNil(try NonSensorData(from: Data(count: 48)).ekf_ticks)
    }
}
