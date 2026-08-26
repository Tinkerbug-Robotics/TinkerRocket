import XCTest
@testable import TinkerRocketApp

/// #836 item 8 — `disconnectAll()` with two devices connected left the second
/// one to silently auto-reconnect and take a full profile re-push.
///
/// `userInitiatedDisconnect` was one fleet-wide Bool. `disconnectAll()` set it
/// once and then cancelled every peripheral connection in a loop, but the
/// FIRST `didDisconnectPeripheral` consumed it:
///
///     if userInitiatedDisconnect { userInitiatedDisconnect = false; …; return }
///
/// so the second device fell through to the automatic-reconnect ladder and
/// came back about a second later. On the resulting `didConnect`, the syncer
/// re-attached and re-pushed the entire active profile — `sendPyroConfig`
/// included — to a rocket the user had just explicitly disconnected from.
///
/// Base station + direct rocket is an explicitly supported pairing, so this
/// was the normal two-device case, not a corner.
final class UserDisconnectLedgerTests: XCTestCase {

    private let baseStation = UUID()
    private let rocket = UUID()

    /// THE regression: disconnectAll marks both, and BOTH callbacks must
    /// answer "the user asked for this".
    func testDisconnectAllSuppressesReconnectForEveryDevice() {
        var ledger = UserDisconnectLedger()
        ledger.mark(baseStation)
        ledger.mark(rocket)

        XCTAssertTrue(ledger.consume(baseStation))
        XCTAssertTrue(ledger.consume(rocket),
                      "the second device fell through to the reconnect ladder")
        XCTAssertTrue(ledger.isEmpty)
    }

    /// Order must not matter — CoreBluetooth does not promise callback order.
    func testCallbackOrderDoesNotMatter() {
        var ledger = UserDisconnectLedger()
        ledger.mark(baseStation)
        ledger.mark(rocket)

        XCTAssertTrue(ledger.consume(rocket))
        XCTAssertTrue(ledger.consume(baseStation))
    }

    /// The other half of the contract: a link that dropped on its own MUST
    /// reconnect. Suppressing that would strand a rocket mid-flight.
    func testUnmarkedDropoutStillReconnects() {
        var ledger = UserDisconnectLedger()
        XCTAssertFalse(ledger.consume(rocket))
    }

    func testDisconnectingOneDeviceLeavesTheOtherFreeToReconnect() {
        var ledger = UserDisconnectLedger()
        ledger.mark(rocket)                       // user disconnects the rocket only

        XCTAssertTrue(ledger.consume(rocket))
        XCTAssertFalse(ledger.consume(baseStation),
                       "the base station dropped on its own and must come back")
    }

    /// A marker is consumed once. A later genuine dropout of the same
    /// peripheral is a real dropout.
    func testMarkerIsConsumedNotSticky() {
        var ledger = UserDisconnectLedger()
        ledger.mark(rocket)
        XCTAssertTrue(ledger.consume(rocket))
        XCTAssertFalse(ledger.consume(rocket),
                       "a stale marker suppressed a later real dropout")
    }

    /// Double-tapping disconnect before the callback lands must not queue a
    /// second suppression.
    func testMarkIsIdempotent() {
        var ledger = UserDisconnectLedger()
        ledger.mark(rocket)
        ledger.mark(rocket)
        XCTAssertEqual(ledger.count, 1)
        XCTAssertTrue(ledger.consume(rocket))
        XCTAssertFalse(ledger.consume(rocket))
    }

    /// The old Bool's second-order leak: `disconnect()` set it even when the
    /// device had no peripheral, and with no callback coming nothing ever
    /// cleared it — poisoning the next UNRELATED dropout into looking
    /// user-initiated, which suppressed its reconnect. Keyed by peripheral,
    /// there is nothing to poison.
    func testAMarkerCannotLeakOntoADifferentPeripheral() {
        var ledger = UserDisconnectLedger()
        ledger.mark(rocket)
        XCTAssertFalse(ledger.consume(baseStation))
        XCTAssertTrue(ledger.consume(rocket), "the rocket's own marker was eaten")
    }

    /// Reconnected, or the connect attempt failed: the marker is stale either
    /// way, and the next disconnect is a real dropout.
    func testForgetClearsAStaleMarkerSoTheNextDropoutReconnects() {
        var ledger = UserDisconnectLedger()
        ledger.mark(rocket)
        ledger.forget(rocket)
        XCTAssertFalse(ledger.consume(rocket))
        XCTAssertTrue(ledger.isEmpty)
    }

    func testForgetOfAnUnmarkedPeripheralIsHarmless() {
        var ledger = UserDisconnectLedger()
        ledger.mark(rocket)
        ledger.forget(baseStation)
        XCTAssertTrue(ledger.consume(rocket))
    }
}
