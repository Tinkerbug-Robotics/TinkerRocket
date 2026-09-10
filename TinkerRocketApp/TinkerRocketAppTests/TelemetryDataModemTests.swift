import XCTest
@testable import TinkerRocketApp

/// #412: the LoRa daughterboard's identity and the out computer's verdict on
/// it. Both were console-only, so from the phone a radio-dead rocket looked
/// exactly like a quiet one.
///
/// Android twin: the `#412` block in `TelemetryDataTest.kt`, case for case.
final class TelemetryDataModemTests: XCTestCase {

    private func decode(_ json: String) throws -> TelemetryData {
        try JSONDecoder().decode(TelemetryData.self, from: Data(json.utf8))
    }

    func testModemKeysDecode() throws {
        let t = try decode(#"{"mst":1,"mfw":"8f73e281-v8+20260909-1835"}"#)
        XCTAssertEqual(t.modem_state, 1)
        XCTAssertEqual(t.modem_fw, "8f73e281-v8+20260909-1835")
        XCTAssertEqual(t.modemState, .up)
    }

    func testABoardWithNoDaughterboardSaysNothingAtAll() throws {
        // The out computer omits both keys rather than sending a zero, so nil
        // must mean "not applicable" and must NOT produce an advisory line.
        let t = try decode(#"{"soc":85.0}"#)
        XCTAssertNil(t.modem_state)
        XCTAssertNil(t.modem_fw)
        XCTAssertNil(t.modemAdvisoryText)
    }

    func testAHealthyModemIsSilent() throws {
        XCTAssertNil(try decode(#"{"mst":1}"#).modemAdvisoryText)
    }

    func testTheTwoFaultsAreToldApart() throws {
        // Separate because the fix differs: 3 is a reflash of the
        // daughterboard, 2 is a cable, a rail or a dead board.
        XCTAssertEqual(try decode(#"{"mst":2}"#).modemAdvisoryText,
                       "Radio daughterboard not answering — no LoRa")
        XCTAssertEqual(try decode(#"{"mst":3}"#).modemAdvisoryText,
                       "Radio daughterboard firmware mismatch — no LoRa")
    }

    func testAnUnknownStateCodeStaysSilentRatherThanInventingAFault() throws {
        let t = try decode(#"{"mst":9}"#)
        XCTAssertNil(t.modemState)
        XCTAssertNil(t.modemAdvisoryText)
    }

    func testAStateWithNoFirmwareStringStillDecodes() throws {
        // An absent modem never identified itself, so the out computer sends
        // mst without mfw.
        let t = try decode(#"{"mst":2}"#)
        XCTAssertEqual(t.modemState, .absent)
        XCTAssertNil(t.modem_fw)
        XCTAssertEqual(t.modemAdvisoryText, "Radio daughterboard not answering — no LoRa")
    }
}
