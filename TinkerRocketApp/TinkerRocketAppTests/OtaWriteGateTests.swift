import XCTest
@testable import TinkerRocketApp

/// #836 item 7 — the OTA backpressure timeout could never fire.
///
/// `sendOtaChunk` bounded its wait for `canSendWriteWithoutResponse` with a
/// `withTimeout` helper that raced two children in a `withTaskGroup`: one
/// running the body, one sleeping 5 s. It took `group.next()`, called
/// `cancelAll()` and returned — except a task group implicitly awaits ALL of
/// its children before returning, and the body child was parked in a plain
/// `withCheckedContinuation`, which is not cancellable.
///
/// So when the sleeper won, `withTimeout` did not return. The
/// `if !ready { throw }` branch was unreachable while the device stayed
/// connected: OTASession sat in `.uploading` with a frozen progress bar,
/// `Task.cancel()` was a no-op against the checked continuation, `reset()`
/// could not recover it, and only a BLE disconnect broke it out. Mid-flash on
/// the FC-relay path that leaves the flight computer with a half-written OTA
/// slot and the app still claiming an upload is in progress.
///
/// The finding notes this path had no coverage at all — OTASessionFlowTests
/// only drives a ScriptedLink.
@MainActor
final class OtaWriteGateTests: XCTestCase {

    /// THE regression: the timeout has to actually return.
    func testTimeoutFiresWhenNothingEverDrains() async {
        let gate = OtaWriteGate()
        // Nothing ever signals — the WRITE_NO_RSP-less characteristic case.
        let outcome = await gate.wait(seconds: 0.05, alreadyReady: { false })
        XCTAssertEqual(outcome, .timedOut)
        XCTAssertFalse(gate.isWaiting)
    }

    func testReadyBufferDoesNotPark() async {
        let gate = OtaWriteGate()
        let outcome = await gate.wait(seconds: 5.0, alreadyReady: { true })
        XCTAssertEqual(outcome, .ready)
        XCTAssertFalse(gate.isWaiting)
    }

    func testSignalReadyWakesAParkedChunk() async {
        let gate = OtaWriteGate()
        Task { @MainActor in
            // Let the waiter park first.
            try? await Task.sleep(nanoseconds: 20_000_000)
            gate.signal(.ready)
        }
        let outcome = await gate.wait(seconds: 5.0, alreadyReady: { false })
        XCTAssertEqual(outcome, .ready)
    }

    /// A disconnect mid-transfer must surface as a distinct outcome. The old
    /// code resumed the waiter and let execution fall through on the belief
    /// that "the next peripheral guard" would catch it — but `peripheral` is
    /// a local bound at function entry, so there is no next guard and the
    /// chunk was written to a dead peripheral and counted as sent.
    func testDisconnectIsDistinctFromTimeout() async {
        let gate = OtaWriteGate()
        Task { @MainActor in
            try? await Task.sleep(nanoseconds: 20_000_000)
            gate.signal(.disconnected)
        }
        let outcome = await gate.wait(seconds: 5.0, alreadyReady: { false })
        XCTAssertEqual(outcome, .disconnected)
    }

    /// Resuming a checked continuation twice is a TRAP, not an error. During a
    /// real transfer the buffer drains constantly and most of those signals
    /// arrive with no waiter parked.
    func testSignalWithNoWaiterIsHarmless() async {
        let gate = OtaWriteGate()
        gate.signal(.ready)
        gate.signal(.ready)
        gate.signal(.disconnected)
        // Still usable afterwards.
        let after = await gate.wait(seconds: 5.0, alreadyReady: { true })
        XCTAssertEqual(after, .ready)
    }

    /// A signal that lands after the timeout already fired must not double
    /// resume — the racy ordering when a slow buffer finally drains.
    func testSignalAfterTimeoutIsHarmless() async {
        let gate = OtaWriteGate()
        let timedOut = await gate.wait(seconds: 0.05, alreadyReady: { false })
        XCTAssertEqual(timedOut, .timedOut)
        gate.signal(.ready)
        XCTAssertFalse(gate.isWaiting)
    }

    /// The timer must be cancelled when a waiter is woken normally, or a
    /// later chunk's wait could be cut short by the previous chunk's timer.
    func testEarlierTimerCannotTimeOutALaterChunk() async {
        let gate = OtaWriteGate()
        Task { @MainActor in
            try? await Task.sleep(nanoseconds: 10_000_000)
            gate.signal(.ready)
        }
        let firstChunk = await gate.wait(seconds: 0.05, alreadyReady: { false })
        XCTAssertEqual(firstChunk, .ready)

        // The first chunk's 50 ms timer would land around here. The second
        // chunk must still be waiting on its OWN clock.
        try? await Task.sleep(nanoseconds: 80_000_000)
        Task { @MainActor in
            try? await Task.sleep(nanoseconds: 10_000_000)
            gate.signal(.ready)
        }
        let secondChunk = await gate.wait(seconds: 5.0, alreadyReady: { false })
        XCTAssertEqual(secondChunk, .ready)
    }

    /// The pump is serial so this should not arise, but overwriting a parked
    /// continuation would leak it — an unrecoverable hang, versus a retryable
    /// error for the older chunk.
    func testAStaleWaiterIsFailedRatherThanLeaked() async {
        let gate = OtaWriteGate()
        let first = Task { @MainActor in
            await gate.wait(seconds: 5.0, alreadyReady: { false })
        }
        try? await Task.sleep(nanoseconds: 20_000_000)
        XCTAssertTrue(gate.isWaiting)

        Task { @MainActor in
            try? await Task.sleep(nanoseconds: 20_000_000)
            gate.signal(.ready)
        }
        let second = await gate.wait(seconds: 5.0, alreadyReady: { false })

        let firstOutcome = await first.value
        XCTAssertEqual(firstOutcome, .timedOut, "the first waiter was leaked")
        XCTAssertEqual(second, .ready)
    }
}
