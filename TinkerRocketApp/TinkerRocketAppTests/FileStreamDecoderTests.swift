import XCTest
@testable import TinkerRocketApp

// #526: the Swift FileStreamDecoder must reassemble records split at arbitrary byte
// boundaries — iOS delivers the L2CAP channel as a byte stream with no record
// boundaries. This mirrors the firmware's C++ reference test
// (tests_cpp/test_file_stream_framing.cpp): feed the same encoded transfer both in
// bulk and one byte at a time, and require identical decoding.
final class FileStreamDecoderTests: XCTestCase {

    // Build a BEGIN + one DATA + END stream the way the firmware encodes it.
    private func encodeTransfer(name: String, payload: [UInt8]) -> Data {
        var out = Data()
        // BEGIN: [0x01][len u32][ver][status][size_hint u32][name_len][name]
        let nameBytes = Array(name.utf8)
        var begin: [UInt8] = [1 /*ver*/, 0 /*status OK*/]
        begin += le32(UInt32(payload.count))          // size_hint
        begin.append(UInt8(nameBytes.count))
        begin += nameBytes
        out.append(0x01); out.append(contentsOf: le32(UInt32(begin.count))); out.append(contentsOf: begin)
        // DATA: [0x02][len u32][payload]
        out.append(0x02); out.append(contentsOf: le32(UInt32(payload.count))); out.append(contentsOf: payload)
        // END: [0x03][9][bytes u32][crc u32][status]
        let crc = Crc32.compute(Data(payload))
        var end = le32(UInt32(payload.count)); end += le32(crc); end.append(0)
        out.append(0x03); out.append(contentsOf: le32(9)); out.append(contentsOf: end)
        return out
    }
    private func le32(_ v: UInt32) -> [UInt8] {
        [UInt8(v & 0xFF), UInt8((v >> 8) & 0xFF), UInt8((v >> 16) & 0xFF), UInt8((v >> 24) & 0xFF)]
    }

    private struct Decoded {
        var name = ""; var beginStatus: UInt8 = 0xFF
        var data = Data(); var endBytes: UInt32 = 0; var endCrc: UInt32 = 0
        var gotEnd = false; var error: String?
    }
    private func run(_ stream: Data, chunk: Int) -> Decoded {
        var d = Decoded()
        let dec = FileStreamDecoder()
        dec.onBegin = { d.name = $0.name; d.beginStatus = $0.status }
        dec.onData = { d.data.append($0) }
        dec.onEnd = { d.endBytes = $0.bytes; d.endCrc = $0.crc; d.gotEnd = true }
        dec.onError = { d.error = $0 }
        var i = 0
        while i < stream.count {
            let end = min(i + chunk, stream.count)
            dec.feed(stream.subdata(in: i..<end))
            i = end
        }
        return d
    }

    func testRoundTripBulk() {
        let payload = (0..<1500).map { UInt8(($0 * 7 + 3) & 0xFF) }
        let stream = encodeTransfer(name: "flight_20260714_112346.bin", payload: payload)
        let d = run(stream, chunk: stream.count)
        XCTAssertNil(d.error)
        XCTAssertEqual(d.name, "flight_20260714_112346.bin")
        XCTAssertEqual(d.beginStatus, 0)
        XCTAssertEqual(Array(d.data), payload)
        XCTAssertTrue(d.gotEnd)
        XCTAssertEqual(d.endBytes, UInt32(payload.count))
        XCTAssertEqual(d.endCrc, Crc32.compute(Data(payload)))
    }

    func testSurvivesByteAtATime() {
        let payload = (0..<777).map { UInt8($0 & 0xFF) }
        let stream = encodeTransfer(name: "f.bin", payload: payload)
        let d = run(stream, chunk: 1)   // one byte per feed()
        XCTAssertNil(d.error)
        XCTAssertEqual(Array(d.data), payload)
        XCTAssertTrue(d.gotEnd)
        XCTAssertEqual(d.endCrc, Crc32.compute(Data(payload)))
    }

    func testEquivalentUnderEveryChunkSize() {
        let payload = (0..<300).map { UInt8(($0 ^ 0x5A) & 0xFF) }
        let stream = encodeTransfer(name: "x", payload: payload)
        for chunk in [1, 2, 3, 5, 7, 13, 64, 300, stream.count] {
            let d = run(stream, chunk: chunk)
            XCTAssertNil(d.error, "chunk=\(chunk)")
            XCTAssertEqual(Array(d.data), payload, "chunk=\(chunk)")
            XCTAssertTrue(d.gotEnd, "chunk=\(chunk)")
        }
    }

    func testRefusalBeginHasNoData() {
        // BEGIN status=2 (INFLIGHT), empty name, no DATA/END.
        var out = Data()
        var begin: [UInt8] = [1, 2]; begin += le32(0); begin.append(0)
        out.append(0x01); out.append(contentsOf: le32(UInt32(begin.count))); out.append(contentsOf: begin)
        let d = run(out, chunk: 1)
        XCTAssertNil(d.error)
        XCTAssertEqual(d.beginStatus, 2)
        XCTAssertEqual(d.data.count, 0)
        XCTAssertFalse(d.gotEnd)
    }

    func testRejectsUnknownRecordType() {
        let d = run(Data([0x09, 0, 0, 0, 0]), chunk: 5)
        XCTAssertNotNil(d.error)
    }

    func testStaysFailedAfterError() {
        let dec = FileStreamDecoder()
        var errors = 0, begins = 0
        dec.onError = { _ in errors += 1 }
        dec.onBegin = { _ in begins += 1 }
        dec.feed(Data([0xFF]))              // bad type -> fail
        XCTAssertTrue(dec.failed)
        let good = encodeTransfer(name: "y", payload: [1, 2, 3])
        dec.feed(good)                      // must NOT resurrect
        XCTAssertEqual(begins, 0)
    }
}
