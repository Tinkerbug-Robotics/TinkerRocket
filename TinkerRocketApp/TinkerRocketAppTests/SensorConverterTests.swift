import XCTest
@testable import TinkerRocketApp

final class SensorConverterTests: XCTestCase {

    let converter = SensorConverter()

    // MARK: - IMU Tests

    func testIMU_ZeroRaw_ZeroSI() {
        var raw = ISM6HG256Data()
        // All fields zero by default
        raw.time_us = 1000

        let si = converter.convertISM6(raw)

        XCTAssertEqual(si.time_us, 1000)
        XCTAssertEqual(si.low_g_acc_x, 0.0, accuracy: 1e-6)
        XCTAssertEqual(si.low_g_acc_y, 0.0, accuracy: 1e-6)
        XCTAssertEqual(si.low_g_acc_z, 0.0, accuracy: 1e-6)
        XCTAssertEqual(si.gyro_x, 0.0, accuracy: 1e-6)
        XCTAssertEqual(si.gyro_y, 0.0, accuracy: 1e-6)
        XCTAssertEqual(si.gyro_z, 0.0, accuracy: 1e-6)
    }

    func testIMU_FullScale_CorrectConversion() {
        var raw = ISM6HG256Data()
        raw.acc_low_raw = Vec3i16(x: 32767, y: 0, z: 0)

        let si = converter.convertISM6(raw)

        // At 16g FS: 32767 * (16/32768) * 9.80665 ≈ 156.9 m/s²
        let expected = 16.0 * 9.80665 * (32767.0 / 32768.0)
        XCTAssertEqual(si.low_g_acc_x, expected, accuracy: 0.1)
    }

    // MARK: - GNSS Tests

    func testGNSS_LatLonAlt() {
        var raw = GNSSData()
        raw.lat_e7 = 337000000    // 33.7 deg
        raw.lon_e7 = -1184000000  // -118.4 deg
        raw.alt_mm = 150000        // 150 m
        raw.num_sats = 12
        raw.pdop_x10 = 15          // PDOP = 1.5

        let si = converter.convertGNSS(raw)

        XCTAssertEqual(si.lat, 33.7, accuracy: 1e-6)
        XCTAssertEqual(si.lon, -118.4, accuracy: 1e-6)
        XCTAssertEqual(si.alt, 150.0, accuracy: 1e-3)
        XCTAssertEqual(si.num_sats, 12)
        XCTAssertEqual(si.pdop, 1.5, accuracy: 0.01)
    }

    // MARK: - Baro Tests

    func testBaro_Conversion() {
        var raw = BMP585Data()
        raw.time_us = 5000
        raw.temp_q16 = Int32(25.0 * 65536)   // 25 deg C
        raw.press_q6 = UInt32(101325.0 * 64)  // std atmosphere

        let si = converter.convertBMP585(raw)

        XCTAssertEqual(si.temperature, 25.0, accuracy: 0.01)
        XCTAssertEqual(si.pressure, 101325.0, accuracy: 1.0)
    }

    // MARK: - Magnetometer Tests

    func testMag_ZeroCentered() {
        var raw = MMC5983MAData()
        raw.time_us = 2000
        // Center value is 131072 (2^17)
        raw.mag_x = 131072
        raw.mag_y = 131072
        raw.mag_z = 131072

        let si = converter.convertMMC5983MA(raw)

        XCTAssertEqual(si.mag_x_uT, 0.0, accuracy: 0.01)
        XCTAssertEqual(si.mag_y_uT, 0.0, accuracy: 0.01)
        XCTAssertEqual(si.mag_z_uT, 0.0, accuracy: 0.01)
    }
}
