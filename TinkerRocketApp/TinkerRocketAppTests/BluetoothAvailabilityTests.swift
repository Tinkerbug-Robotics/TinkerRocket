//
//  BluetoothAvailabilityTests.swift
//  TinkerRocketAppTests
//
//  #1413. The states were always distinguished; what was missing was a way
//  forward. These pin the mapping and the two decisions that follow from it —
//  what to say, and whether Settings is the right place to send someone —
//  because the simulator has no Bluetooth radio and cannot reach `.denied` or
//  `.off` at all, so a UI check on a phone is the only other way to see them.
//

import XCTest
import CoreBluetooth
@testable import TinkerRocketApp

final class BluetoothAvailabilityTests: XCTestCase {

    func testEveryCoreBluetoothStateMaps() {
        XCTAssertEqual(BluetoothAvailability(.poweredOn), .ready)
        XCTAssertEqual(BluetoothAvailability(.poweredOff), .off)
        XCTAssertEqual(BluetoothAvailability(.unauthorized), .denied)
        XCTAssertEqual(BluetoothAvailability(.unsupported), .unsupported)
        // Transient states say nothing rather than guessing — see .unknown.
        XCTAssertEqual(BluetoothAvailability(.resetting), .unknown)
        XCTAssertEqual(BluetoothAvailability(.unknown), .unknown)
    }

    func testOnlyTheActionableStatesGiveAdvice() {
        XCTAssertNotNil(BluetoothAvailability.off.advice)
        XCTAssertNotNil(BluetoothAvailability.denied.advice)
        // Nothing has gone wrong and there is nothing to do, so say nothing.
        XCTAssertNil(BluetoothAvailability.ready.advice)
        XCTAssertNil(BluetoothAvailability.unknown.advice)
        XCTAssertNil(BluetoothAvailability.unsupported.advice,
                     "simulator-only; there is no fix for the reader to apply")
    }

    func testSettingsIsOfferedOnlyWhereItLeadsSomewhere() {
        // openSettingsURLString opens the APP's page, which is where this
        // app's Bluetooth switch lives — the right destination for a denial.
        XCTAssertTrue(BluetoothAvailability.denied.offersAppSettings)
        // It is the wrong destination for a powered-off adapter: the app's own
        // page has no system Bluetooth toggle, and iOS has no supported deep
        // link to the one that does. The advice says so in words instead.
        XCTAssertFalse(BluetoothAvailability.off.offersAppSettings)
        XCTAssertFalse(BluetoothAvailability.ready.offersAppSettings)
        XCTAssertFalse(BluetoothAvailability.unsupported.offersAppSettings)
        XCTAssertFalse(BluetoothAvailability.unknown.offersAppSettings)
    }

    func testOnlyReadyCanScan() {
        XCTAssertTrue(BluetoothAvailability.ready.canScan)
        for state: BluetoothAvailability in [.off, .denied, .unsupported, .unknown] {
            XCTAssertFalse(state.canScan, "\(state) must not start a scan that cannot run")
        }
    }

    func testTheDeniedAdviceNamesTheAppAndTheOffAdviceNamesThePlace() {
        // Not a spelling test: a user who denied the prompt has to know it is
        // THIS app's permission, and one with the radio off has to be told
        // where the switch is, because no link can take them there.
        XCTAssertTrue(BluetoothAvailability.denied.advice?.contains("TinkerRocket") == true)
        let offAdvice = BluetoothAvailability.off.advice ?? ""
        XCTAssertTrue(offAdvice.contains("Control Center") || offAdvice.contains("Settings"))
    }

    func testHeadlinesAreUnchangedFromTheOnesTheFleetUsedToSet() {
        // The strings were already right and already distinguished (#637 desk
        // check, 2026-07-29). Moving them must not have rewritten them.
        XCTAssertEqual(BluetoothAvailability.ready.headline, "Bluetooth ready")
        XCTAssertEqual(BluetoothAvailability.off.headline, "Bluetooth is off")
        XCTAssertEqual(BluetoothAvailability.denied.headline, "Bluetooth not authorized")
        XCTAssertEqual(BluetoothAvailability.unsupported.headline, "Bluetooth not supported")
        XCTAssertEqual(BluetoothAvailability.unknown.headline, "Bluetooth unknown state")
    }
}
