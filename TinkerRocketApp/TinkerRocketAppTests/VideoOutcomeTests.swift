import XCTest
import SwiftUI
@testable import TinkerRocketApp

/// #838 item 2 — PyroTestView showed a green "Video Saved" checkmark on a
/// follow-up test that was aborted before anything was saved.
///
/// `startCountdown()` reset `errorMessage`, `firedThisTest`, `state` and
/// `secondsRemaining` — but not `videoSaved`. The `.done` branch keyed its
/// icon and headline purely off that Bool, and several `.done` transitions
/// never touched it: the T-0 link-not-ready abort and the focus-changed abort.
///
/// Run a test that records and saves (`videoSaved = true`, `.done`), then tap
/// FIRE again without leaving the screen. At T-0 `pyroCommandPathReady` is
/// false, so the view jumps to `.done` with "Link lost before fire — no fire
/// command sent" — under a green checkmark reading "Video Saved", for a run
/// in which nothing was recorded or saved.
///
/// Resetting the Bool would not have been enough: `false` renders "Video save
/// failed", equally untrue of a run whose save has not come back yet — and an
/// abort at T-0 stops a recording that began at T-5, so that window is real.
final class VideoOutcomeTests: XCTestCase {

    /// THE regression: the state a reset run is in must not claim a save.
    func testPendingDoesNotClaimEitherVerdict() {
        let pending: VideoOutcome? = nil
        XCTAssertEqual(pending.headline, "Saving video…")
        XCTAssertNotEqual(pending.headline, "Video Saved",
                          "an unfinished run claimed the previous run's success")
        XCTAssertNotEqual(pending.headline, "Video save failed",
                          "an unfinished run was reported as a failure")
    }

    func testSavedAndFailedStillReadAsBefore() {
        let saved: VideoOutcome? = .saved
        let failed: VideoOutcome? = .failed
        XCTAssertEqual(saved.headline, "Video Saved")
        XCTAssertEqual(failed.headline, "Video save failed")
    }

    /// Green is reserved for an actual save — the glanceable signal on a
    /// screen about a pyro charge.
    func testOnlyASavedRunIsGreen() {
        let saved: VideoOutcome? = .saved
        let failed: VideoOutcome? = .failed
        XCTAssertEqual(saved.tint, .green)
        XCTAssertEqual(failed.tint, .orange)
        let pending: VideoOutcome? = nil
        XCTAssertNotEqual(pending.tint, .green, "a pending save rendered as success")
    }

    func testOnlyASavedRunGetsTheCheckmark() {
        let saved: VideoOutcome? = .saved
        let failed: VideoOutcome? = .failed
        XCTAssertEqual(saved.iconName, "checkmark.circle.fill")
        XCTAssertEqual(failed.iconName, "exclamationmark.circle.fill")
        let pending: VideoOutcome? = nil
        XCTAssertNotEqual(pending.iconName, "checkmark.circle.fill",
                          "a pending save wore the checkmark")
    }

    /// The three states are distinguishable in every channel — a screen that
    /// says "saving" while showing a green checkmark is no better than the
    /// bug.
    func testTheThreeStatesAreDistinctEverywhere() {
        let states: [VideoOutcome?] = [nil, .saved, .failed]
        XCTAssertEqual(Set(states.map(\.headline)).count, 3)
        XCTAssertEqual(Set(states.map(\.iconName)).count, 3)
    }
}
