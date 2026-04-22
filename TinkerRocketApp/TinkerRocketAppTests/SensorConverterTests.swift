import XCTest
@testable import TinkerRocketApp

// These structs expose only binary-data initialisers, so each test builds
// a Data blob with the fields it needs and lets the struct's init(from:)
// parse it.  Helpers below keep the byte-packing out of the test bodies.

private extension Data {
    mutating func appendLE<T: FixedWidthInteger>(_ v: T) {
        var le = v.littleEndian
        Swift.withUnsafeBytes(of: &le) { buf in self.append(contentsOf: buf) }
    }
}

private func makeISM6Data(
    timeUs: UInt32 = 0,
    accLow: (Int16, Int16, Int16) = (0, 0, 0),
    accHigh: (Int16, Int16, Int16) = (0, 0, 0),
    gyro: (Int16, Int16, Int16) = (0, 0, 0)
) throws -> ISM6HG256Data {
    var d = Data()
    d.appendLE(timeUs)
    d.appendLE(accLow.0); d.appendLE(accLow.1); d.appendLE(accLow.2)
    d.appendLE(accHigh.0); d.appendLE(accHigh.1); d.appendLE(accHigh.2)
    d.appendLE(gyro.0); d.appendLE(gyro.1); d.appendLE(gyro.2)
    return try ISM6HG256Data(from: d)
}

private func makeBMP585Data(timeUs: UInt32 = 0, tempC: Double, pressurePa: Double) throws -> BMP585Data {
    var d = Data()
    d.appendLE(timeUs)
    d.appendLE(Int32(tempC * 65536.0))   // temp_q16
    d.appendLE(UInt32(pressurePa * 64.0)) // press_q6
    return try BMP585Data(from: d)
}

private func makeMag(timeUs: UInt32 = 0, x: UInt32, y: UInt32, z: UInt32) throws -> MMC5983MAData {
    var d = Data()
    d.appendLE(timeUs)
    d.appendLE(x); d.appendLE(y); d.appendLE(z)
    return try MMC5983MAData(from: d)
}

private func makeGNSS(
    timeUs: UInt32 = 0,
    latE7: Int32, lonE7: Int32, altMm: Int32,
    numSats: UInt8 = 0, pdopX10: UInt8 = 0
) throws -> GNSSData {
    var d = Data()
    d.appendLE(timeUs)
    d.appendLE(UInt16(0))                 // year
    d.append(contentsOf: [0, 0])          // month, day
    d.append(contentsOf: [0, 0, 0])       // hour, minute, second
    d.appendLE(UInt16(0))                 // milli_second
    d.append(contentsOf: [0])             // fix_mode
    d.append(contentsOf: [numSats, pdopX10])
    d.appendLE(latE7); d.appendLE(lonE7); d.appendLE(altMm)
    d.appendLE(Int32(0)); d.appendLE(Int32(0)); d.appendLE(Int32(0))  // velocities
    d.append(contentsOf: [0, 0])          // h_acc, v_acc
    return try GNSSData(from: d)
}

final class SensorConverterTests: XCTestCase {

    let converter = SensorConverter()

    // MARK: - IMU

    func testIMU_ZeroRaw_ZeroSI() throws {
        let raw = try makeISM6Data(timeUs: 1000)
        let si = converter.convertISM6HG256(raw)

        XCTAssertEqual(si.time_us, 1000)
        XCTAssertEqual(si.low_g_acc_x, 0.0, accuracy: 1e-6)
        XCTAssertEqual(si.low_g_acc_y, 0.0, accuracy: 1e-6)
        XCTAssertEqual(si.low_g_acc_z, 0.0, accuracy: 1e-6)
        XCTAssertEqual(si.gyro_x, 0.0, accuracy: 1e-6)
        XCTAssertEqual(si.gyro_y, 0.0, accuracy: 1e-6)
        XCTAssertEqual(si.gyro_z, 0.0, accuracy: 1e-6)
    }

    func testIMU_FullScale_CorrectConversion() throws {
        // 16g full-scale, 16-bit two's complement: raw 32767 → (16*g) * 32767/32768.
        let raw = try makeISM6Data(accLow: (32767, 0, 0))
        let si = converter.convertISM6HG256(raw)

        let expected = 16.0 * 9.80665 * (32767.0 / 32768.0)
        XCTAssertEqual(si.low_g_acc_x, expected, accuracy: 0.1)
    }

    // MARK: - GNSS

    func testGNSS_LatLonAlt() throws {
        let raw = try makeGNSS(
            latE7: 337000000,     // 33.7 deg
            lonE7: -1184000000,   // -118.4 deg
            altMm: 150000,        // 150 m
            numSats: 12,
            pdopX10: 15           // PDOP 1.5
        )
        let si = converter.convertGNSS(raw)

        XCTAssertEqual(si.lat, 33.7, accuracy: 1e-6)
        XCTAssertEqual(si.lon, -118.4, accuracy: 1e-6)
        XCTAssertEqual(si.alt, 150.0, accuracy: 1e-3)
        XCTAssertEqual(si.num_sats, 12)
        XCTAssertEqual(si.pdop, 1.5, accuracy: 0.01)
    }

    // MARK: - Baro

    func testBaro_Conversion() throws {
        let raw = try makeBMP585Data(timeUs: 5000, tempC: 25.0, pressurePa: 101325.0)
        let si = converter.convertBMP585(raw)

        XCTAssertEqual(si.temperature, 25.0, accuracy: 0.01)
        XCTAssertEqual(si.pressure, 101325.0, accuracy: 1.0)
    }

    // MARK: - Magnetometer

    func testMag_ZeroCentered() throws {
        // Centre value is 2^17 = 131072; converted output should be 0.
        let raw = try makeMag(timeUs: 2000, x: 131072, y: 131072, z: 131072)
        let si = converter.convertMMC5983MA(raw)

        XCTAssertEqual(si.mag_x, 0.0, accuracy: 0.01)
        XCTAssertEqual(si.mag_y, 0.0, accuracy: 0.01)
        XCTAssertEqual(si.mag_z, 0.0, accuracy: 0.01)
    }
}
