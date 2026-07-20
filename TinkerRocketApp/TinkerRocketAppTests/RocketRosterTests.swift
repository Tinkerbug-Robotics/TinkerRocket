import XCTest
@testable import TinkerRocketApp

/// #390: the roster merges every reachable rocket into network-scoped
/// subjects — one entry per (networkID, rocketID) regardless of how many
/// links carry it — and routes commands direct-first.
final class RocketRosterTests: XCTestCase {

    private func makeBaseStation(nid: UInt8 = 5) -> BLEDevice {
        let bs = BLEDevice(peripheral: nil, name: "TR-B-Ridge")
        bs.networkID = nid
        bs.isConnected = true
        return bs
    }

    private func makeDirectRocket(rid: UInt8, nid: UInt8 = 5,
                                  name: String = "TR-R-Atlas") -> BLEDevice {
        let r = BLEDevice(peripheral: nil, name: name)
        r.networkID = nid
        r.rocketID = rid
        r.isConnected = true
        return r
    }

    private func relayedJSON(rocketID: Int, name: String = "") -> Data {
        let run = name.isEmpty ? "" : ",\"run\":\"\(name)\""
        return """
        {"rid":\(rocketID)\(run),"st":"READY","palt":1.0}
        """.data(using: .utf8)!
    }

    // MARK: - Merge

    func testRelayedRockets_OneSubjectEach() {
        let bs = makeBaseStation()
        bs.parseTelemetryData(relayedJSON(rocketID: 1, name: "Booster"))
        bs.parseTelemetryData(relayedJSON(rocketID: 2, name: "Sustainer"))

        let roster = RocketRoster.build(devices: [bs])
        XCTAssertEqual(roster.count, 2)
        XCTAssertEqual(roster.map(\.name), ["Booster", "Sustainer"])
        XCTAssertTrue(roster.allSatisfy { $0.isRelayOnly })
    }

    func testDirectAndRelaySameKey_MergeIntoOneSubject() {
        let bs = makeBaseStation(nid: 5)
        bs.parseTelemetryData(relayedJSON(rocketID: 1))
        let direct = makeDirectRocket(rid: 1, nid: 5)

        let roster = RocketRoster.build(devices: [bs, direct])
        XCTAssertEqual(roster.count, 1,
                       "Same (nid, rid) via BS and direct BLE is ONE rocket")
        let subject = roster[0]
        XCTAssertNotNil(subject.direct)
        XCTAssertEqual(subject.relays.count, 1)
        XCTAssertFalse(subject.isRelayOnly)
    }

    func testSameRidDifferentNetwork_StaySeparate() {
        // Two BS/rocket pairs, both using "rocket 1", on networks 5 and 9.
        let bsA = makeBaseStation(nid: 5)
        bsA.parseTelemetryData(relayedJSON(rocketID: 1, name: "PairA"))
        let bsB = makeBaseStation(nid: 9)
        bsB.parseTelemetryData(relayedJSON(rocketID: 1, name: "PairB"))

        let roster = RocketRoster.build(devices: [bsA, bsB])
        XCTAssertEqual(roster.count, 2,
                       "rid is only unique per network — never merge across nids")
        XCTAssertEqual(Set(roster.map(\.name)), ["PairA", "PairB"])
    }

    func testTwoBaseStationsSameNetwork_MergeRelaysForOneRocket() {
        let bsA = makeBaseStation(nid: 5)
        bsA.parseTelemetryData(relayedJSON(rocketID: 1))
        let bsB = makeBaseStation(nid: 5)
        bsB.parseTelemetryData(relayedJSON(rocketID: 1))

        let roster = RocketRoster.build(devices: [bsA, bsB])
        XCTAssertEqual(roster.count, 1)
        XCTAssertEqual(roster[0].relays.count, 2,
                       "Both BSes carry the same rocket — one subject, two paths")
    }

    func testDirectRocketWithoutIdentity_ShowsAsIdentifying() {
        let direct = makeDirectRocket(rid: 0)   // readback not in yet

        let roster = RocketRoster.build(devices: [direct])
        XCTAssertEqual(roster.count, 1)
        if case .known = roster[0].id {
            XCTFail("rid 0 must not mint a known (nid,0) identity")
        }
        XCTAssertNotNil(roster[0].direct)
    }

    func testDisconnectedDevices_NotInRoster() {
        let direct = makeDirectRocket(rid: 1)
        direct.isConnected = false
        XCTAssertTrue(RocketRoster.build(devices: [direct]).isEmpty)
    }

    // MARK: - Command routing

    func testCommandLink_DirectWinsOverRelay() throws {
        let bs = makeBaseStation(nid: 5)
        bs.parseTelemetryData(relayedJSON(rocketID: 1))
        let direct = makeDirectRocket(rid: 1, nid: 5)

        let subject = RocketRoster.build(devices: [bs, direct])[0]
        guard case .direct(let dev) = try XCTUnwrap(subject.commandLink()) else {
            return XCTFail("Connected direct link must win")
        }
        XCTAssertTrue(dev === direct)
    }

    func testCommandLink_RelayOnly_TargetsRocketID() throws {
        let bs = makeBaseStation()
        bs.parseTelemetryData(relayedJSON(rocketID: 3))

        let subject = RocketRoster.build(devices: [bs])[0]
        guard case .relay(let viaBS, let rid) = try XCTUnwrap(subject.commandLink()) else {
            return XCTFail("Relay-only rocket routes via the BS")
        }
        XCTAssertTrue(viaBS === bs)
        XCTAssertEqual(rid, 3)
    }

    func testCommandLink_PrefersFreshestRelay() throws {
        let bsStale = makeBaseStation(nid: 5)
        bsStale.parseTelemetryData(relayedJSON(rocketID: 1))
        bsStale.remoteRockets[0].lastSeen = Date().addingTimeInterval(-30)
        let bsFresh = makeBaseStation(nid: 5)
        bsFresh.parseTelemetryData(relayedJSON(rocketID: 1))

        let subject = RocketRoster.build(devices: [bsStale, bsFresh])[0]
        guard case .relay(let viaBS, _) = try XCTUnwrap(subject.commandLink()) else {
            return XCTFail()
        }
        XCTAssertTrue(viaBS === bsFresh,
                      "With no foreground preference, the freshest relay carries commands")
    }

    // MARK: - Freshness

    func testFreshness_Tiers() {
        let now = Date()
        XCTAssertEqual(RocketFreshness.from(lastSeen: now.addingTimeInterval(-1), now: now), .live)
        if case .stale(let age) = RocketFreshness.from(lastSeen: now.addingTimeInterval(-10), now: now) {
            XCTAssertEqual(age, 10, accuracy: 0.01)
        } else {
            XCTFail("10 s old should be stale")
        }
        if case .lost = RocketFreshness.from(lastSeen: now.addingTimeInterval(-120), now: now) {
        } else {
            XCTFail("2 min old should be lost")
        }
        if case .lost(let seen) = RocketFreshness.from(lastSeen: nil, now: now) {
            XCTAssertNil(seen)
        } else {
            XCTFail("Never seen should be lost")
        }
    }
}
