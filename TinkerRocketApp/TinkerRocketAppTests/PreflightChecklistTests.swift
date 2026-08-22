//
//  PreflightChecklistTests.swift
//  TinkerRocketAppTests
//
//  Pure-logic tests for the pre-flight checklist: effective-list
//  composition (live master template + per-rocket diff), auto-step
//  evaluation against telemetry / sync state, and the progress rollup.
//

import XCTest
@testable import TinkerRocketApp

final class PreflightChecklistTests: XCTestCase {

    // MARK: - Helpers

    private func manual(_ title: String) -> PreflightItem {
        PreflightItem(title: title)
    }

    /// Telemetry with the given packed bits/fields, avoiding a live parser.
    private func telemetry(fs: Int = 0, ps: Int = 0, sats: Int = 0,
                           lat: Double? = nil, lon: Double? = nil) -> TelemetryData {
        var t = TelemetryData()
        t.flight_status_bits = fs
        t.pyro_status_bits = ps
        t.num_sats = sats
        t.latitude = lat
        t.longitude = lon
        return t
    }

    private func connectedCtx(fs: Int = 0, ps: Int = 0, sats: Int = 0,
                              lat: Double? = nil, lon: Double? = nil,
                              isRelay: Bool = false,
                              syncState: ActiveRocketSyncer.SyncState = .idle,
                              profile: RocketProfile? = nil) -> PreflightAutoContext {
        PreflightAutoContext(isConnected: true, hasTelemetry: true,
                             isRelay: isRelay,
                             telemetry: telemetry(fs: fs, ps: ps, sats: sats,
                                                  lat: lat, lon: lon),
                             syncState: syncState, profile: profile)
    }

    // MARK: - Effective list

    func testEffectiveListIsMasterWhenNoConfig() {
        let master = PreflightMaster(items: [manual("A"), manual("B")])
        let items = PreflightChecklist.effectiveItems(master: master, config: nil)
        XCTAssertEqual(items.map(\.title), ["A", "B"])
    }

    func testEffectiveListExcludesDisabledAndAppendsExtras() {
        let a = manual("A"), b = manual("B")
        let master = PreflightMaster(items: [a, b])
        let config = PreflightRocketConfig(profileId: UUID(),
                                           disabledMasterIds: [a.id],
                                           extraItems: [manual("Extra")])
        let items = PreflightChecklist.effectiveItems(master: master, config: config)
        XCTAssertEqual(items.map(\.title), ["B", "Extra"])
    }

    /// The live-template contract: a master edit reaches a configured
    /// rocket without touching its stored diff.
    func testMasterEditFlowsThroughToConfiguredRocket() {
        var a = manual("A")
        let master = PreflightMaster(items: [a])
        let config = PreflightRocketConfig(profileId: UUID())
        a.title = "A, but sharper"
        let edited = PreflightMaster(items: [a])
        XCTAssertEqual(PreflightChecklist.effectiveItems(master: master, config: config)
                        .map(\.title), ["A"])
        XCTAssertEqual(PreflightChecklist.effectiveItems(master: edited, config: config)
                        .map(\.title), ["A, but sharper"])
    }

    // MARK: - Auto evaluation

    func testManualHasNoAutoStatus() {
        XCTAssertNil(PreflightChecklist.autoStatus(.manual, in: PreflightAutoContext()))
    }

    func testEveryAutoKindPendsWhenDisconnected() {
        let ctx = PreflightAutoContext()   // not connected
        for kind in PreflightItemKind.allCases where kind.isAuto {
            XCTAssertEqual(PreflightChecklist.autoStatus(kind, in: ctx),
                           .pending("Not connected"), "\(kind)")
        }
    }

    func testConnectedNeedsTelemetry() {
        var ctx = PreflightAutoContext(isConnected: true)
        XCTAssertEqual(PreflightChecklist.autoStatus(.connected, in: ctx),
                       .pending("Waiting for telemetry"))
        ctx.hasTelemetry = true
        XCTAssertEqual(PreflightChecklist.autoStatus(.connected, in: ctx), .satisfied)
    }

    func testSettingsSyncedFollowsSyncerState() {
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .settingsSynced, in: connectedCtx(syncState: .synced)), .satisfied)
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .settingsSynced, in: connectedCtx(syncState: .awaitingSync)),
            .pending("Not yet applied"))
        // A relay link can't sync — N/A, not pending-forever.
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .settingsSynced, in: connectedCtx(isRelay: true, syncState: .idle)),
            .notApplicable("Needs a direct connection"))
    }

    func testGnssFixNeedsPositionAndSats() {
        // Same bar as LastValidRocketFix: real lat/lon + >= 4 sats.
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .gnssFix, in: connectedCtx(sats: 7, lat: 40.0, lon: -105.0)), .satisfied)
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .gnssFix, in: connectedCtx(sats: 3, lat: 40.0, lon: -105.0)),
            .pending("No fix (3 sats)"))
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .gnssFix, in: connectedCtx(sats: 9, lat: 0, lon: 0)),
            .pending("No fix (9 sats)"))
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .gnssFix, in: connectedCtx(sats: 9)),
            .pending("No fix (9 sats)"))
    }

    func testCameraRecordingReadsFsBitAndProfile() {
        var withCam = RocketProfile.makeDefault(name: "cam")   // cameraType 2 = RunCam
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .cameraRecording, in: connectedCtx(fs: 0x20, profile: withCam)), .satisfied)
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .cameraRecording, in: connectedCtx(fs: 0, profile: withCam)),
            .pending("Not recording"))
        withCam.cameraType = 0
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .cameraRecording, in: connectedCtx(fs: 0x20, profile: withCam)),
            .notApplicable("No camera on this rocket"))
    }

    func testLoggingActiveReadsFsBit() {
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .loggingActive, in: connectedCtx(fs: 0x40)), .satisfied)
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .loggingActive, in: connectedCtx(fs: 0)), .pending("Not logging"))
    }

    func testPyroArmedGatedOnEnabledChannels() {
        var p = RocketProfile.makeDefault(name: "pyro")
        // No channels enabled (factory default) — N/A even when armed.
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .pyroArmed, in: connectedCtx(ps: 0x001, profile: p)),
            .notApplicable("No pyro channels enabled"))
        p.pyro1Enabled = true
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .pyroArmed, in: connectedCtx(ps: 0x001, profile: p)), .satisfied)
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .pyroArmed, in: connectedCtx(ps: 0, profile: p)), .pending("Not armed"))
    }

    func testPyroContinuityChecksOnlyEnabledChannels() {
        var p = RocketProfile.makeDefault(name: "pyro")
        p.pyro1Enabled = true
        p.pyro3Enabled = true
        // ch1 cont (0x002) + ch3 cont (0x020); ch2/4 open but disabled.
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .pyroContinuity, in: connectedCtx(ps: 0x002 | 0x020, profile: p)),
            .satisfied)
        // ch3 open → named in the reason.
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .pyroContinuity, in: connectedCtx(ps: 0x002, profile: p)),
            .pending("Ch 3 open"))
        XCTAssertEqual(PreflightChecklist.autoStatus(
            .pyroContinuity, in: connectedCtx(ps: 0, profile: RocketProfile.makeDefault(name: "x"))),
            .notApplicable("No pyro channels enabled"))
    }

    // MARK: - Progress rollup

    func testProgressCountsManualChecksAndAutoStates() {
        let m1 = manual("wadding"), m2 = manual("igniter")
        let cam = PreflightItem.auto(.cameraRecording)
        let master = PreflightMaster(items: [m1, m2, cam])
        var config = PreflightRocketConfig(profileId: UUID())
        config.checked[m1.id.uuidString] = Date()

        var profile = RocketProfile.makeDefault(name: "r")
        profile.cameraType = 0   // camera N/A → counts as done

        let items = PreflightChecklist.effectiveItems(master: master, config: config)
        let progress = PreflightChecklist.progress(
            items: items, config: config,
            ctx: connectedCtx(profile: profile))
        XCTAssertEqual(progress.done, 2)     // m1 checked + camera N/A
        XCTAssertEqual(progress.total, 3)
        XCTAssertFalse(progress.isComplete)
    }

    func testAutoStepCannotBeSatisfiedByManualCheck() {
        let cam = PreflightItem.auto(.cameraRecording)
        let master = PreflightMaster(items: [cam])
        var config = PreflightRocketConfig(profileId: UUID())
        // A stray checked entry for an auto item must not count.
        config.checked[cam.id.uuidString] = Date()

        let progress = PreflightChecklist.progress(
            items: master.items, config: config,
            ctx: connectedCtx(fs: 0, profile: RocketProfile.makeDefault(name: "r")))
        XCTAssertEqual(progress.done, 0)
        XCTAssertFalse(progress.isComplete)
    }

    func testEmptyChecklistIsNeverComplete() {
        let progress = PreflightChecklist.progress(items: [], config: nil,
                                                   ctx: PreflightAutoContext())
        XCTAssertFalse(progress.isComplete)
    }

    // MARK: - Codable

    func testItemDecodeUnknownKindFallsBackToManual() throws {
        let json = """
        {"id":"11111111-2222-3333-4444-555555555555","title":"future step",
         "detail":"","kind":"quantumFluxCheck"}
        """.data(using: .utf8)!
        let item = try JSONDecoder().decode(PreflightItem.self, from: json)
        XCTAssertEqual(item.kind, .manual)
        XCTAssertEqual(item.title, "future step")
    }

    func testConfigRoundTrip() throws {
        let extra = manual("rail buttons")
        var config = PreflightRocketConfig(profileId: UUID(),
                                           disabledMasterIds: [UUID()],
                                           extraItems: [extra])
        config.checked[extra.id.uuidString] = Date(timeIntervalSince1970: 1_700_000_000)

        let data = try JSONEncoder().encode(config)
        let decoded = try JSONDecoder().decode(PreflightRocketConfig.self, from: data)
        XCTAssertEqual(decoded, config)
    }

    func testMasterRoundTrip() throws {
        let master = PreflightMaster(items: [manual("A"), .auto(.loggingActive)])
        let data = try JSONEncoder().encode(master)
        let decoded = try JSONDecoder().decode(PreflightMaster.self, from: data)
        XCTAssertEqual(decoded, master)
    }
}
