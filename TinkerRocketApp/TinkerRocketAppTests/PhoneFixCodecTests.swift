//
//  PhoneFixCodecTests.swift
//  TinkerRocketAppTests
//
//  The phone fix is the ONLY record of where the base station was — the BS has
//  no GNSS — so a silent encoding bug loses the range for a whole test day and
//  is not recoverable afterwards. These pin the byte layout the firmware
//  parses in BLE_BS_CMD_SET_PHONE_FIX.
//

import XCTest
import CoreLocation
@testable import TinkerRocketApp

final class PhoneFixCodecTests: XCTestCase {

    private func i32(_ d: Data, _ o: Int) -> Int32 {
        d.subdata(in: o..<(o + 4)).withUnsafeBytes {
            Int32(littleEndian: $0.loadUnaligned(as: Int32.self))
        }
    }
    private func i16(_ d: Data, _ o: Int) -> Int16 {
        d.subdata(in: o..<(o + 2)).withUnsafeBytes {
            Int16(littleEndian: $0.loadUnaligned(as: Int16.self))
        }
    }

    /// The real 2026-08-08 Kaua'i base-station site, round-tripped.
    func testEncodesRealSiteToFirmwareLayout() {
        let coord = CLLocationCoordinate2D(latitude: 22.062009925676062,
                                           longitude: -159.35412251709172)
        let d = PhoneFixCodec.encode(coord, altitude: 12.4, accuracy: 4.6)

        XCTAssertEqual(d.count, PhoneFixCodec.payloadLength)
        XCTAssertEqual(i32(d, 0), 220620099)
        XCTAssertEqual(i32(d, 4), -1593541225)
        XCTAssertEqual(i16(d, 8), 12)
        XCTAssertEqual(d[10], 5)

        // Decoding the way the firmware does must land back on the site to
        // well under a metre.
        XCTAssertEqual(Double(i32(d, 0)) * 1e-7, coord.latitude,  accuracy: 1e-6)
        XCTAssertEqual(Double(i32(d, 4)) * 1e-7, coord.longitude, accuracy: 1e-6)
    }

    func testMissingAltitudeAndAccuracyUseSentinels() {
        let d = PhoneFixCodec.encode(CLLocationCoordinate2D(latitude: 0, longitude: 0),
                                     altitude: nil, accuracy: nil)
        XCTAssertEqual(i16(d, 8), 0)
        XCTAssertEqual(d[10], PhoneFixCodec.accuracyUnknown)
    }

    /// CLLocation reports a negative horizontalAccuracy for an invalid fix.
    /// That must become the sentinel, not wrap to a small plausible number.
    func testNegativeAccuracyBecomesSentinel() {
        let d = PhoneFixCodec.encode(CLLocationCoordinate2D(latitude: 1, longitude: 1),
                                     altitude: 0, accuracy: -1)
        XCTAssertEqual(d[10], PhoneFixCodec.accuracyUnknown)
    }

    /// Clamping, not wrapping. A wrapped coordinate reads as a valid position
    /// somewhere else on Earth and would be undetectable in the log.
    func testExtremeValuesClamp() {
        let d = PhoneFixCodec.encode(CLLocationCoordinate2D(latitude: 90, longitude: -180),
                                     altitude: 1e9, accuracy: 1e9)
        XCTAssertEqual(i32(d, 0), 900000000)
        XCTAssertEqual(i32(d, 4), -1800000000)
        XCTAssertEqual(i16(d, 8), Int16.max)
        XCTAssertEqual(d[10], 255)
    }

    /// Southern/eastern hemispheres must stay signed through the round trip.
    func testSignedHemispheres() {
        let coord = CLLocationCoordinate2D(latitude: -33.8688, longitude: 151.2093)
        let d = PhoneFixCodec.encode(coord, altitude: -5, accuracy: 3)
        XCTAssertEqual(Double(i32(d, 0)) * 1e-7, coord.latitude,  accuracy: 1e-6)
        XCTAssertEqual(Double(i32(d, 4)) * 1e-7, coord.longitude, accuracy: 1e-6)
        XCTAssertEqual(i16(d, 8), -5)
    }
}
