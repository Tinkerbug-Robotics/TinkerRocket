import XCTest
@testable import TinkerRocketApp

/// Verifies that Swift struct sizes match C++ packed struct sizes.
/// Size mismatches would cause binary deserialization errors in
/// CSVParser and MessageParser.
final class SensorTypesTests: XCTestCase {

    func testGNSSData_Size42() {
        XCTAssertEqual(MemoryLayout<GNSSData>.size, 42,
                       "GNSSData must be 42 bytes to match C++ __attribute__((packed))")
    }

    func testISM6HG256Data_Size22() {
        XCTAssertEqual(MemoryLayout<ISM6HG256Data>.size, 22,
                       "ISM6HG256Data must be 22 bytes")
    }

    func testBMP585Data_Size12() {
        XCTAssertEqual(MemoryLayout<BMP585Data>.size, 12,
                       "BMP585Data must be 12 bytes")
    }

    func testMMC5983MAData_Size16() {
        XCTAssertEqual(MemoryLayout<MMC5983MAData>.size, 16,
                       "MMC5983MAData must be 16 bytes")
    }

    func testPOWERData_Size10() {
        XCTAssertEqual(MemoryLayout<POWERData>.size, 10,
                       "POWERData must be 10 bytes")
    }

    func testNonSensorData_Size43() {
        XCTAssertEqual(MemoryLayout<NonSensorData>.size, 43,
                       "NonSensorData must be 43 bytes")
    }
}
