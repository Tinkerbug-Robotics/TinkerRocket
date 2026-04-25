import XCTest
@testable import TinkerRocketApp

// Tests for BLEDevice.autoApplyRefusalReason — the gating that decides
// whether the Frequency Scan view's "Apply" button should be enabled.
// Issue #71: pushing a new frequency to the base station without a live
// rocket strands the rocket on the old channel, so we refuse unless all
// preconditions are met.  Logic was extracted to a pure static so it can
// be exercised without standing up a CoreBluetooth peripheral.

final class AutoApplyRefusalTests: XCTestCase {

    // Reference time pinned across tests so beacon-age math is deterministic.
    private let now = Date(timeIntervalSince1970: 1_000_000)

    /// Build a "valid" RocketConfig — every LoRa field populated so config
    /// gating doesn't trip.  Tests that want to break this property only
    /// override the field they care about.
    private func validConfig() -> RocketConfig {
        var cfg = RocketConfig()
        cfg.loraFreqMHz  = 915.0
        cfg.loraSF       = 8
        cfg.loraBwKHz    = 250.0
        cfg.loraCR       = 5
        cfg.loraTxPower  = 12
        return cfg
    }

    // ------------------------------------------------------------------
    // Refusal cases
    // ------------------------------------------------------------------

    func test_refuses_when_not_base_station() {
        let r = BLEDevice.autoApplyRefusalReason(
            isBaseStation: false,
            isConnected: true,
            config: validConfig(),
            rocketLastSeenTimes: [now],
            now: now
        )
        XCTAssertEqual(r, .notBaseStation)
    }

    func test_refuses_when_not_connected() {
        let r = BLEDevice.autoApplyRefusalReason(
            isBaseStation: true,
            isConnected: false,
            config: validConfig(),
            rocketLastSeenTimes: [now],
            now: now
        )
        XCTAssertEqual(r, .notConnected)
    }

    func test_refuses_when_config_missing_entirely() {
        let r = BLEDevice.autoApplyRefusalReason(
            isBaseStation: true,
            isConnected: true,
            config: nil,
            rocketLastSeenTimes: [now],
            now: now
        )
        XCTAssertEqual(r, .configMissing)
    }

    func test_refuses_when_config_missing_a_lora_field() {
        // Missing any one of loraSF/BW/CR/TxPower is treated as "config not
        // ready yet" — partial configs land here while the readback streams.
        var cfg = validConfig()
        cfg.loraSF = nil
        let r = BLEDevice.autoApplyRefusalReason(
            isBaseStation: true,
            isConnected: true,
            config: cfg,
            rocketLastSeenTimes: [now],
            now: now
        )
        XCTAssertEqual(r, .configMissing)
    }

    func test_refuses_when_no_tracked_rockets() {
        // Empty list — base station hasn't heard from any rocket since
        // booting, no point pushing a frequency change into the void.
        let r = BLEDevice.autoApplyRefusalReason(
            isBaseStation: true,
            isConnected: true,
            config: validConfig(),
            rocketLastSeenTimes: [],
            now: now
        )
        XCTAssertEqual(r, .noRocketPresent)
    }

    func test_refuses_when_only_stale_rocket_beacons() {
        // Last beacon was older than the freshness window; rocket has
        // probably gone off-air or out of range.
        let stale = now.addingTimeInterval(-(BLEDevice.autoApplyMaxBeaconAgeSeconds + 1))
        let r = BLEDevice.autoApplyRefusalReason(
            isBaseStation: true,
            isConnected: true,
            config: validConfig(),
            rocketLastSeenTimes: [stale],
            now: now
        )
        XCTAssertEqual(r, .noRocketPresent)
    }

    // ------------------------------------------------------------------
    // Allowance cases
    // ------------------------------------------------------------------

    func test_allows_when_rocket_just_beaconed() {
        let r = BLEDevice.autoApplyRefusalReason(
            isBaseStation: true,
            isConnected: true,
            config: validConfig(),
            rocketLastSeenTimes: [now],
            now: now
        )
        XCTAssertNil(r)
    }

    func test_allows_when_at_least_one_rocket_is_fresh() {
        // Mix of stale and fresh — any one fresh beacon is enough.
        let stale = now.addingTimeInterval(-100)
        let fresh = now.addingTimeInterval(-3)
        let r = BLEDevice.autoApplyRefusalReason(
            isBaseStation: true,
            isConnected: true,
            config: validConfig(),
            rocketLastSeenTimes: [stale, fresh],
            now: now
        )
        XCTAssertNil(r)
    }

    func test_freshness_boundary_is_inclusive_at_window_edge() {
        // Beacon exactly at the freshness cutoff should count as fresh
        // (>= cutoff, not strict >), so the gating doesn't briefly flap
        // off→on→off as the cutoff sweeps past a single beacon timestamp.
        let edge = now.addingTimeInterval(-BLEDevice.autoApplyMaxBeaconAgeSeconds)
        let r = BLEDevice.autoApplyRefusalReason(
            isBaseStation: true,
            isConnected: true,
            config: validConfig(),
            rocketLastSeenTimes: [edge],
            now: now
        )
        XCTAssertNil(r)
    }

    // ------------------------------------------------------------------
    // Refusal-message wording — these strings appear verbatim in the
    // Frequency Scan view to surface the failure reason; pin them so a
    // typo doesn't silently change user-facing copy.
    // ------------------------------------------------------------------

    func test_refusal_messages_are_user_friendly() {
        XCTAssertEqual(BLEDevice.AutoApplyRefusal.notBaseStation.rawValue,
                       "Connect to the base station first.")
        XCTAssertEqual(BLEDevice.AutoApplyRefusal.notConnected.rawValue,
                       "Base station is not connected over BLE.")
        XCTAssertEqual(BLEDevice.AutoApplyRefusal.configMissing.rawValue,
                       "Waiting for base-station config readback.")
        XCTAssertEqual(BLEDevice.AutoApplyRefusal.noRocketPresent.rawValue,
                       "No rocket has beaconed recently — power it on and wait for it to show up.")
    }
}
