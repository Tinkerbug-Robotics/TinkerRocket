//
//  NetworkIdTests.swift
//  TinkerRocketAppTests
//
//  #150: the network-name → network-ID mapping is the contract between
//  onboarding (preview), provisioning (push), and the firmware's identity
//  NVS.  Pin it so a refactor can't silently re-map every fielded device.
//

import XCTest
@testable import TinkerRocketApp

final class NetworkIdTests: XCTestCase {

    func testFnv1a8_Deterministic() {
        // Same input, same ID — across calls and instances.
        XCTAssertEqual(fnv1a8("Skyhawks Club"), fnv1a8("Skyhawks Club"))
        XCTAssertEqual(fnv1a8(""), fnv1a8(""))
    }

    func testFnv1a8_KnownVector() {
        // Golden pin: FNV-1a 32-bit of "a" is 0xE40C292C; XOR-folded to
        // 8 bits: 0x2C ^ 0x29 ^ 0x0C ^ 0xE4 = 0xED.  If this fails, the
        // hash changed and every provisioned device's ID mapping breaks.
        XCTAssertEqual(fnv1a8("a"), 0xED)
    }

    func testFnv1a8_DistinguishesTypicalNames() {
        // Not a collision-resistance proof (8 bits can't be) — just a
        // sanity check that obvious sibling names don't collide.
        XCTAssertNotEqual(fnv1a8("My Backyard"), fnv1a8("Skyhawks Club"))
    }

    func testNetworkIdForName_NeverZero() {
        // 0 is reserved: it's the firmware factory default AND the app's
        // "unset" sentinel — a name hashing to 0 would silently disable
        // the provisioning push and the mismatch badge.  networkIdForName
        // remaps that 1-in-256 case to 1; everything else passes through.
        XCTAssertEqual(networkIdForName("a"), fnv1a8("a"))
        // Brute-force a zero-hash input to prove the remap engages.
        var zeroName: String?
        for i in 0..<100_000 {
            let candidate = "net\(i)"
            if fnv1a8(candidate) == 0 { zeroName = candidate; break }
        }
        let name = try! XCTUnwrap(zeroName, "expected a zero-hash name within 100k candidates (p ≈ 1-(255/256)^100000 ≈ 1)")
        XCTAssertEqual(networkIdForName(name), 1)
    }
}
