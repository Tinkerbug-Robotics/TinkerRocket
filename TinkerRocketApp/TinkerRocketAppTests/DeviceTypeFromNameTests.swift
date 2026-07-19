//
//  DeviceTypeFromNameTests.swift
//  TinkerRocketAppTests
//
//  Pins the BLE-name → device-type heuristic.  Only factory-default names
//  carry the TR-R-/TR-B- prefix — a device renamed through My Devices
//  advertises its raw name and MUST come back .unknown here (the scan path
//  then recovers the type from the known-device registry).  The front-page
//  scanner must never filter on this: the scan is service-UUID-gated, and
//  the old TR-*/Tinker name guard made renamed devices invisible.
//

import XCTest
@testable import TinkerRocketApp

final class DeviceTypeFromNameTests: XCTestCase {

    func testFactoryDefaultPrefixes() {
        XCTAssertEqual(BLEDeviceType.from(name: "TR-R-1a2b"), .rocket)
        XCTAssertEqual(BLEDeviceType.from(name: "TR-B-3c4d"), .baseStation)
    }

    func testLegacyNames() {
        XCTAssertEqual(BLEDeviceType.from(name: "TinkerRocket"), .rocket)
        XCTAssertEqual(BLEDeviceType.from(name: "TinkerBaseStation"), .baseStation)
    }

    func testRenamedDevicesAreUnknownNotMistyped() {
        // Raw user names carry no type information — the heuristic must
        // say so rather than guess, so the registry hint can take over.
        XCTAssertEqual(BLEDeviceType.from(name: "Atlas"), .unknown)
        XCTAssertEqual(BLEDeviceType.from(name: "RollyPolly III"), .unknown)
    }
}
