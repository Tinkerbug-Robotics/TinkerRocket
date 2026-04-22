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

    // Swift parser currently reads 42 bytes.  The C++ wire format grew to
    // 43 bytes in commit 7d214f8 (#34) but the iOS side has not been
    // updated — tracked separately as an iOS/firmware sync bug.
    func testNonSensorData_Size42() {
        assertSize(42) { try NonSensorData(from: $0) }
    }
}
