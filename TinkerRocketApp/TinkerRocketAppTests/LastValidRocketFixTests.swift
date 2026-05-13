import XCTest
@testable import TinkerRocketApp

/// Issue #140: the map must keep showing the last reported rocket
/// position even when a fresh telemetry packet arrives without GPS
/// (LoRa loss or BLE drop during flight).  These tests pin the
/// validity check and the cache-preservation behaviour that make that
/// possible.
final class LastValidRocketFixTests: XCTestCase {

    // MARK: - fromTelemetry validity

    func testFromTelemetry_ValidFix_Returned() throws {
        var t = TelemetryData()
        t.latitude = 33.7
        t.longitude = -118.4
        t.num_sats = 8

        let fix = try XCTUnwrap(LastValidRocketFix.fromTelemetry(t, rocketID: 1))
        XCTAssertEqual(fix.latitude, 33.7, accuracy: 1e-9)
        XCTAssertEqual(fix.longitude, -118.4, accuracy: 1e-9)
        XCTAssertEqual(fix.numSats, 8)
        XCTAssertEqual(fix.rocketID, 1)
    }

    func testFromTelemetry_MissingLat_Rejected() {
        // The BS firmware omits NAN lat/lon from JSON to save MTU, so
        // a real "no fix" packet arrives with latitude == nil.  Must
        // not advance the cache — the whole point of issue #140.
        var t = TelemetryData()
        t.longitude = -118.4
        t.num_sats = 8
        XCTAssertNil(LastValidRocketFix.fromTelemetry(t, rocketID: 1))
    }

    func testFromTelemetry_MissingLon_Rejected() {
        var t = TelemetryData()
        t.latitude = 33.7
        t.num_sats = 8
        XCTAssertNil(LastValidRocketFix.fromTelemetry(t, rocketID: 1))
    }

    func testFromTelemetry_ZeroOrigin_Rejected() {
        // (0, 0) is in the Gulf of Guinea — an unrealistic rocket
        // position that almost always means "no fix" leaking through.
        var t = TelemetryData()
        t.latitude = 0
        t.longitude = 0
        t.num_sats = 8
        XCTAssertNil(LastValidRocketFix.fromTelemetry(t, rocketID: 1))
    }

    func testFromTelemetry_TooFewSats_Rejected() {
        var t = TelemetryData()
        t.latitude = 33.7
        t.longitude = -118.4
        t.num_sats = 3
        XCTAssertNil(LastValidRocketFix.fromTelemetry(t, rocketID: 1))
    }

    func testFromTelemetry_MinSatsBoundary_Accepted() {
        // The 4-sat floor is the boundary — anything ≥ 4 must pass so
        // a freshly acquired 3D fix can populate the cache.
        var t = TelemetryData()
        t.latitude = 33.7
        t.longitude = -118.4
        t.num_sats = LastValidRocketFix.minSatsForValidFix
        XCTAssertNotNil(LastValidRocketFix.fromTelemetry(t, rocketID: 1))
    }

    func testFromTelemetry_ZeroRocketID_Rejected() {
        // rocketID == 0 means "unknown rocket" (the default) — never
        // cache against that key or all rockets share the same slot.
        var t = TelemetryData()
        t.latitude = 33.7
        t.longitude = -118.4
        t.num_sats = 8
        XCTAssertNil(LastValidRocketFix.fromTelemetry(t, rocketID: 0))
    }

    // MARK: - BLEFleet.recordRocketFix cache behaviour

    func testRecordRocketFix_ValidPacket_PopulatesCache() throws {
        let fleet = BLEFleet()
        var t = TelemetryData()
        t.latitude = 33.7
        t.longitude = -118.4
        t.num_sats = 8

        XCTAssertNotNil(fleet.recordRocketFix(from: t, rocketID: 1))
        let cached = try XCTUnwrap(fleet.lastValidRocketFixes[1])
        XCTAssertEqual(cached.latitude, 33.7, accuracy: 1e-9)
    }

    func testRecordRocketFix_InvalidPacketWithCached_PreservesCache() throws {
        // THE bug fix for #140.  Once a valid fix is cached, an
        // incoming GPS-less packet must not blow it away — the map
        // must keep showing the last reported position.
        let fleet = BLEFleet()
        var valid = TelemetryData()
        valid.latitude = 33.7
        valid.longitude = -118.4
        valid.num_sats = 8
        fleet.recordRocketFix(from: valid, rocketID: 1)

        let gpsless = TelemetryData()  // no lat/lon, no sats
        let result = try XCTUnwrap(fleet.recordRocketFix(from: gpsless, rocketID: 1))
        XCTAssertEqual(result.latitude, 33.7, accuracy: 1e-9,
                       "Cache must survive a GPS-less packet")

        let cached = try XCTUnwrap(fleet.lastValidRocketFixes[1])
        XCTAssertEqual(cached.latitude, 33.7, accuracy: 1e-9)
    }

    func testRecordRocketFix_NewerValidReplacesOlder() throws {
        // Newer always wins so a recovery-grade fix isn't stuck on a
        // pre-launch one even if both meet the minimum quality bar.
        let fleet = BLEFleet()
        var first = TelemetryData()
        first.latitude = 33.7
        first.longitude = -118.4
        first.num_sats = 4
        fleet.recordRocketFix(from: first, rocketID: 1)

        var second = TelemetryData()
        second.latitude = 34.0
        second.longitude = -118.5
        second.num_sats = 10
        fleet.recordRocketFix(from: second, rocketID: 1)

        let cached = try XCTUnwrap(fleet.lastValidRocketFixes[1])
        XCTAssertEqual(cached.latitude, 34.0, accuracy: 1e-9)
        XCTAssertEqual(cached.numSats, 10)
    }

    func testRecordRocketFix_MultipleRockets_IndependentSlots() throws {
        let fleet = BLEFleet()
        var rocketA = TelemetryData()
        rocketA.latitude = 33.7
        rocketA.longitude = -118.4
        rocketA.num_sats = 8
        fleet.recordRocketFix(from: rocketA, rocketID: 1)

        var rocketB = TelemetryData()
        rocketB.latitude = 47.6
        rocketB.longitude = -122.3
        rocketB.num_sats = 6
        fleet.recordRocketFix(from: rocketB, rocketID: 2)

        let cachedA = try XCTUnwrap(fleet.lastValidRocketFixes[1])
        let cachedB = try XCTUnwrap(fleet.lastValidRocketFixes[2])
        XCTAssertEqual(cachedA.latitude, 33.7, accuracy: 1e-9)
        XCTAssertEqual(cachedB.latitude, 47.6, accuracy: 1e-9)
    }

    func testRecordRocketFix_EmptyCacheAndInvalidPacket_StaysEmpty() {
        let fleet = BLEFleet()
        let gpsless = TelemetryData()

        XCTAssertNil(fleet.recordRocketFix(from: gpsless, rocketID: 1))
        XCTAssertNil(fleet.lastValidRocketFixes[1])
    }
}
