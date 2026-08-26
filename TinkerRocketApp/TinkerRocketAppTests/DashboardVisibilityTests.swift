import XCTest
@testable import TinkerRocketApp

/// Regression guard for the state banner disappearing during FC boot.
///
/// Two correct changes collided.  #95 hid the rocket views while the stream
/// was SYNCING, because on the base-station path SYNCING means "no rocket
/// caught yet" and every value in the zero-initialised struct is a lie.  #831
/// then gave SYNCING a second meaning on the DIRECT path — "the FC has not
/// sent its first NonSensorData frame yet" — so the app went silent for the
/// whole ~25 s of setup_fc().  The state only reappeared at READY, which
/// displays as "PRELAUNCH", so the boot looked like it skipped INITIALIZATION.
///
/// The FC boot-progress line added for exactly that window rode inside the
/// hidden block, so it could never be seen on iOS at all.
final class DashboardVisibilityTests: XCTestCase {

    private func decode(_ json: String) throws -> TelemetryData {
        try JSONDecoder().decode(TelemetryData.self, from: json.data(using: .utf8)!)
    }

    // MARK: - The bug

    func testStateBannerIsShownOnADirectLinkWhileSyncing() {
        // THE regression.  On a direct connection SYNCING means "the FC has not
        // sent NonSensorData yet" — it is still inside setup_fc().  If this ever
        // goes false again the operator watches a blank dashboard for the whole
        // FC boot and the "bs" progress line becomes unreachable.
        XCTAssertTrue(DashboardVisibility.showStateBanner(.syncing, isBaseStation: false))
    }

    func testStateBannerIsShownOnADirectLinkForEveryStatus() {
        for s: TelemetryData.DataStatus in [.live, .stale, .syncing] {
            XCTAssertTrue(DashboardVisibility.showStateBanner(s, isBaseStation: false),
                          "a direct link always has a real state to show (status \(s))")
        }
    }

    // MARK: - ...without reviving the phantom rocket

    func testStateBannerStaysHiddenOnASyncingBaseStation() {
        // The other meaning of SYNCING.  With no rocket caught the BS pushes a
        // zero-init LoRaDataSI, and state 0 IS "INITIALIZATION" — so drawing the
        // banner here would invent a rocket sitting on the pad.  #95's point.
        XCTAssertFalse(DashboardVisibility.showStateBanner(.syncing, isBaseStation: true))
    }

    func testStateBannerIsShownOnABaseStationTrackingARocket() {
        // Once a rocket is caught the BS reports LIVE or STALE, and the state is
        // the rocket's own.  Hiding it there was never the intent.
        XCTAssertTrue(DashboardVisibility.showStateBanner(.live, isBaseStation: true))
        XCTAssertTrue(DashboardVisibility.showStateBanner(.stale, isBaseStation: true))
    }

    // MARK: - What the gate still protects

    func testValueViewsAreHiddenWhileSyncing() {
        // #95's actual concern: a base station with no rocket would otherwise
        // render 0 m altitude and green continuity as if they were measured.
        XCTAssertFalse(DashboardVisibility.showValueViews(.syncing))
    }

    func testValueViewsAreShownWhenLiveOrStale() {
        // STALE still draws — dimmed, with a banner.  The last real values are
        // better than nothing once we know they were real.
        XCTAssertTrue(DashboardVisibility.showValueViews(.live))
        XCTAssertTrue(DashboardVisibility.showValueViews(.stale))
    }

    // MARK: - End to end on a captured frame

    func testCapturedBootFrameShowsStateAndBootLine() throws {
        // A real direct-connect frame from the boot window: INITIALIZATION with
        // ds=2 (syncing).  This is the exact shape that rendered as nothing.
        let t = try decode(#"{"st":"INITIALIZATION","ds":2,"bs":2,"bt":4200}"#)
        XCTAssertEqual(t.data_status, .syncing)
        XCTAssertTrue(DashboardVisibility.showStateBanner(t.data_status,
                                                          isBaseStation: false))
        XCTAssertEqual(RocketStateView.displayLabel(for: t.state), "INITIALIZATION")

        // ...and the boot line that lives on that banner is the payload.
        let boot = try XCTUnwrap(t.fcBootProgress)
        XCTAssertEqual(boot.knownStep, .sensors)
    }
}
