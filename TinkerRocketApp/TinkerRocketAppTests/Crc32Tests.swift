import XCTest
@testable import TinkerRocketApp

// #526: pins the Swift Crc32 to the SAME canonical check value as the firmware's
// gtest (tests_cpp/test_crc32.cpp) and the reference CRC-32/ISO-HDLC vectors. A
// download over L2CAP completes only if these two implementations agree on the CRC
// over the received bytes, so any drift between them must fail a test, not a
// transfer.
final class Crc32Tests: XCTestCase {

    private func crc(_ s: String) -> UInt32 {
        Crc32.compute(s.data(using: .utf8)!)
    }

    func testCanonicalCheckValue() {
        // The value every CRC-32 reference quotes; identical to the firmware gtest.
        XCTAssertEqual(crc("123456789"), 0xCBF4_3926)
    }

    func testEmptyIsZero() {
        XCTAssertEqual(Crc32.compute(Data()), 0x0000_0000)
    }

    func testKnownVectors() {
        XCTAssertEqual(crc("a"), 0xE8B7_BE43)
        XCTAssertEqual(crc("abc"), 0x3524_41C2)
        XCTAssertEqual(crc("The quick brown fox jumps over the lazy dog"), 0x414F_A339)
    }

    // The firmware CRCs the file incrementally as DATA records arrive; the app does
    // the same as stream bytes arrive. Splitting anywhere must not change the value.
    func testIncrementalEqualsOneShot() {
        var buf = [UInt8]()
        for i in 0..<4096 { buf.append(UInt8((i * 31 + 7) & 0xFF)) }
        let data = Data(buf)
        let oneShot = Crc32.compute(data)

        for split in [0, 1, 500, 4095, 4096] {
            var c = Crc32.initial
            c = Crc32.update(c, data.prefix(split))
            c = Crc32.update(c, data.suffix(from: split))
            XCTAssertEqual(Crc32.finalize(c), oneShot, "split=\(split)")
        }
    }

    func testDetectsSingleBitFlip() {
        var buf = [UInt8](0...255).map { $0 }
        let good = Crc32.compute(Data(buf))
        buf[128] ^= 0x01
        XCTAssertNotEqual(Crc32.compute(Data(buf)), good)
    }
}
