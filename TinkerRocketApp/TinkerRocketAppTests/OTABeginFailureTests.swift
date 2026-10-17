import XCTest
@testable import TinkerRocketApp

/// The words an operator reads after trying to flash a flight computer that
/// did not answer: most likely one that is stuck booting.  Android twin:
/// OtaBeginFailureTest.kt.
final class OTABeginFailureTests: XCTestCase {

    func testASilentFlightComputerIsNamedAndTheWayOutIsSaid() {
        let s = OTABeginFailure.notAccepted(targetIsFC: true, timeoutS: 20)
        XCTAssertTrue(s.hasPrefix("The flight computer did not answer within 20s."), s)
        XCTAssertTrue(s.contains("rescue mode"), s)
        XCTAssertTrue(s.contains("USB"), s)
    }

    func testTheLocalWordingIsUnchanged() {
        XCTAssertEqual(OTABeginFailure.notAccepted(targetIsFC: false, timeoutS: 5),
                       "Device did not accept OTA_BEGIN within 5s")
    }

    func testTheWindowComesFromTheTimeoutTable() {
        let s = OTABeginFailure.notAccepted(targetIsFC: true,
                                            timeoutS: OTATimeouts.seconds(.begin, targetIsFC: true))
        XCTAssertTrue(s.contains("within 20s"), s)
    }

    func testWordingMatchesAndroid() {
        // Pinned verbatim, as OtaBeginFailureTest.bothPlatformsSayTheSameThing.
        XCTAssertEqual(
            OTABeginFailure.notAccepted(targetIsFC: true, timeoutS: 20),
            "The flight computer did not answer within 20s. It takes updates once it is "
                + "running, or in rescue mode, but not while it is still starting up. Firmware "
                + "with rescue mode switches to it about two minutes after a stalled start; try "
                + "again then. Older firmware needs a USB flash.")
    }
}
