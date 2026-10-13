import XCTest
@testable import TinkerRocketApp

/// Port of core/protocol `KlobucharTest.kt` — keep the two in step.
final class KlobucharTests: XCTestCase {

    // The header of NOAA's brdc2860.26n (2026-10-13), as served.
    private let noaaHeader = """
     2.11           N: GPS NAV DATA                         RINEX VERSION / TYPE
teqc  2019Feb25     CORS-ADM Account    20261013 10:45:02UTCPGM / RUN BY / DATE
    1.9558D-08  0.0000D+00 -1.1921D-07  0.0000D+00          ION ALPHA
    1.2288D+05 -1.6384D+04 -2.6214D+05  1.9661D+05          ION BETA
    0.000000000000D+00 3.552713678801D-15   405504     2440 DELTA-UTC: A0,A1,T,W
                                                            END OF HEADER
 1 26 10 13  0  0  0.0 3.508096560836D-04 1.136868377216D-12 0.000000000000D+00

"""

    // The same parameters as a RINEX 3 file states them.
    private let rinex3Header = """
     3.04           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE
GPSA   1.9558E-08  0.0000E+00 -1.1921E-07  0.0000E+00       IONOSPHERIC CORR
GPSB   1.2288E+05 -1.6384E+04 -2.6214E+05  1.9661E+05       IONOSPHERIC CORR
GAL    2.5250E+01  2.3438E-01  4.3640E-03  0.0000E+00       IONOSPHERIC CORR
                                                            END OF HEADER

"""

    /// Kotlin's `LocalDate.of(y, m, d)`: that date's midnight UTC.
    private func utc(_ y: Int, _ m: Int, _ d: Int) -> Date {
        var cal = Calendar(identifier: .gregorian)
        cal.timeZone = TimeZone(identifier: "UTC")!
        return cal.date(from: DateComponents(year: y, month: m, day: d))!
    }

    private func lines(_ s: String) -> [String] { s.components(separatedBy: "\n") }

    func testReadsTheBroadcastBytesFromNoaasHeader() throws {
        let k = try XCTUnwrap(Klobuchar.fromRinexHeader(noaaHeader, gpsDay: 17082))
        XCTAssertEqual(k.raw, [21, 0, -2, 0, 60, -1, -4, 3])
        XCTAssertEqual(k.gpsDay, 17082)
        XCTAssertEqual(k.values()[0], 1.9558e-08, accuracy: 1e-12)
        XCTAssertEqual(k.values()[7], 196608.0)
    }

    func testReadsRinex3sGpsLinesAndNotGalileos() {
        XCTAssertEqual(Klobuchar.fromRinexHeader(noaaHeader, gpsDay: 17082),
                       Klobuchar.fromRinexHeader(rinex3Header, gpsDay: 17082))
    }

    func testRefusesAHeaderWithoutBothOrWithAValueOffItsUnit() {
        let noBeta = lines(noaaHeader).filter { !$0.hasSuffix("ION BETA") }.joined(separator: "\n")
        XCTAssertNil(Klobuchar.fromRinexHeader(noBeta, gpsDay: 17082))
        // 1.5 of alpha0's unit is not a value GPS can broadcast
        let off = noaaHeader.replacingOccurrences(of: "    1.9558D-08", with: "    1.3970D-09")
        XCTAssertNil(Klobuchar.fromRinexHeader(off, gpsDay: 17082))
        // nor is one past a signed byte
        let big = noaaHeader.replacingOccurrences(of: "    1.2288D+05", with: "    2.6419D+05")
        XCTAssertNil(Klobuchar.fromRinexHeader(big, gpsDay: 17082))
        // a field that parses but is no number
        let nan = noaaHeader.replacingOccurrences(of: "    1.9558D-08", with: "           NaN")
        XCTAssertNil(Klobuchar.fromRinexHeader(nan, gpsDay: 17082))
        // the ionosphere lines after the header are not read
        let late = lines(noaaHeader).filter { !$0.contains("ION ") }.joined(separator: "\n") +
            "\n    1.9558D-08  0.0000D+00 -1.1921D-07  0.0000D+00          ION ALPHA\n"
        XCTAssertNil(Klobuchar.fromRinexHeader(late, gpsDay: 17082))
    }

    func testGpsDayAndNoaasFileName() {
        XCTAssertEqual(Klobuchar.gpsDay(utc(1980, 1, 6)), 0)
        XCTAssertEqual(Klobuchar.gpsDay(utc(2026, 10, 13)), 17082)
        XCTAssertEqual(Klobuchar.date(17082), utc(2026, 10, 13))
        XCTAssertEqual(Klobuchar.noaaUrl(utc(2026, 10, 13)),
                       "https://geodesy.noaa.gov/corsdata/rinex/2026/286/brdc2860.26n.gz")
        XCTAssertEqual(Klobuchar.noaaUrl(utc(2027, 1, 1)),
                       "https://geodesy.noaa.gov/corsdata/rinex/2027/001/brdc0010.27n.gz")
    }

    func testPayloadIsTheBytesThenTheDay() throws {
        let k = try XCTUnwrap(Klobuchar(raw: [21, 0, -2, 0, 60, -1, -4, 3], gpsDay: 17082))
        XCTAssertEqual(k.payload(), Data([21, 0, 0xFE, 0, 60, 0xFF, 0xFC, 3, 0xBA, 0x42]))
    }
}
